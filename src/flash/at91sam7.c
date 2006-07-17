/***************************************************************************
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                    	                               *
 *									                                       *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

/***************************************************************************
There are some things to notice

* AT91SAM7S64 is tested
* All AT91SAM7Sxx  and  AT91SAM7Xxx should work but is not tested
* All parameters are identified from onchip configuartion registers 
*
* The flash controller handles erases automatically on a page (128/265 byte) basis
* Only an EraseAll command is supported by the controller
* Partial erases can be implemented in software by writing one 0xFFFFFFFF word to 
* some location in every page in the region to be erased
*  
* Lock regions (sectors) are 32 or 64 pages
*
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "replacements.h"

#include "at91sam7.h"

#include "flash.h"
#include "target.h"
#include "log.h"
#include "binarybuffer.h"
#include "types.h"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int at91sam7_register_commands(struct command_context_s *cmd_ctx);
int at91sam7_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank);
int at91sam7_erase(struct flash_bank_s *bank, int first, int last);
int at91sam7_protect(struct flash_bank_s *bank, int set, int first, int last);
int at91sam7_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count);
int at91sam7_probe(struct flash_bank_s *bank);
int at91sam7_erase_check(struct flash_bank_s *bank);
int at91sam7_protect_check(struct flash_bank_s *bank);
int at91sam7_info(struct flash_bank_s *bank, char *buf, int buf_size);

u32 at91sam7_get_flash_status(flash_bank_t *bank);
void at91sam7_set_flash_mode(flash_bank_t *bank,int mode);
u8 at91sam7_wait_status_busy(flash_bank_t *bank, int timeout);
int at91sam7_handle_gpnvm_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

flash_driver_t at91sam7_flash =
{
	.name = "at91sam7",
	.register_commands = at91sam7_register_commands,
	.flash_bank_command = at91sam7_flash_bank_command,
	.erase = at91sam7_erase,
	.protect = at91sam7_protect,
	.write = at91sam7_write,
	.probe = at91sam7_probe,
	.erase_check = at91sam7_erase_check,
	.protect_check = at91sam7_protect_check,
	.info = at91sam7_info
};


char * EPROC[8]= {"Unknown","ARM946-E","ARM7TDMI","Unknown","ARM920T","ARM926EJ-S","Unknown","Unknown"};
long NVPSIZ[16] = {
   0,
   0x2000, /*  8K */
   0x4000, /* 16K */ 
   0x8000, /* 32K */
   -1,
   0x10000, /* 64K */
   -1,
   0x20000, /* 128K */
   -1,
   0x40000, /* 256K */
   0x80000, /* 512K */
   -1,
   0x100000, /* 1024K */
   -1,
   0x200000, /* 2048K */
   -1
};

long SRAMSIZ[16] = {
   -1,
   0x0400, /*  1K */
   0x0800, /*  2K */ 
   -1, 
   0x1c000,  /* 112K */
   0x1000,  /*   4K */
   0x14000, /*  80K */
   0x28000, /* 160K */
   0x2000,  /*   8K */
   0x4000,  /*  16K */
   0x8000,  /*  32K */
   0x10000, /*  64K */
   0x20000, /* 128K */
   0x40000, /* 256K */
   0x18000, /* 96K */
   0x80000, /* 512K */
};

int at91sam7_register_commands(struct command_context_s *cmd_ctx)
{
	command_t *at91sam7_cmd = register_command(cmd_ctx, NULL, "at91sam7", NULL, COMMAND_ANY, NULL);
	register_command(cmd_ctx, at91sam7_cmd, "gpnvm", at91sam7_handle_gpnvm_command, COMMAND_EXEC,
			"at91sam7 gpnvm <num> <bit> set|clear, set or clear at91sam7 gpnvm bit");

	return ERROR_OK;
}

u32 at91sam7_get_flash_status(flash_bank_t *bank)
{
	at91sam7_flash_bank_t *at91sam7_info = bank->driver_priv;
	target_t *target = at91sam7_info->target;
	long fsr;
	
	target->type->read_memory(target, MC_FSR, 4, 1, (u8 *)&fsr);
	
	return fsr;
}

/** Read clock configuration and set at91sam7_info->usec_clocks*/ 
void at91sam7_read_clock_info(flash_bank_t *bank)
{
	at91sam7_flash_bank_t *at91sam7_info = bank->driver_priv;
	target_t *target = at91sam7_info->target;
	unsigned long mckr, mcfr, pllr, tmp, status, mainfreq;
	unsigned int css, pres, mul, div;

	/* Read main clock freqency register */
	target->type->read_memory(target, CKGR_MCFR, 4, 1, (u8 *)&mcfr);
	/* Read master clock register */
	target->type->read_memory(target, PMC_MCKR, 4, 1, (u8 *)&mckr);
	/* Read Clock Generator PLL Register  */
	target->type->read_memory(target, CKGR_PLLR, 4, 1, (u8 *)&pllr);

	pres = (mckr>>2)&0x7;
	mul = (pllr>>16)&0x7FF;
	div = pllr&0xFF;

	at91sam7_info->mck_valid = 0;
	switch (mckr & PMC_MCKR_CSS) {
	case 0:			/* Slow Clock */
		at91sam7_info->mck_valid = 1;
		tmp = RC_FREQ;
		break;
	case 1:			/* Main Clock */
		if (mcfr & CKGR_MCFR_MAINRDY) 
		{
			at91sam7_info->mck_valid = 1;
			mainfreq = RC_FREQ / 16ul * (mcfr & 0xffff);
			tmp = mainfreq;
		}
		break;
		  
	case 2:			/* Reserved */
		break;
	case 3:		/* PLL Clock */
		if (mcfr & CKGR_MCFR_MAINRDY) 
		{
		        target->type->read_memory(target, CKGR_PLLR, 4, 1,
						  (u8 *)&pllr);
			if (!(pllr & CKGR_PLLR_DIV))
			        break; /* 0 Hz */
			at91sam7_info->mck_valid = 1;
			mainfreq = RC_FREQ / 16ul * (mcfr & 0xffff);
			/* Integer arithmetic should have sufficient precision
			   as long as PLL is properly configured. */
			tmp = mainfreq / (pllr & CKGR_PLLR_DIV) *
			  (((pllr & CKGR_PLLR_MUL) >> 16) + 1);
		}
		break;
 	}
	
	/* Prescaler adjust */
	if (((mckr & PMC_MCKR_PRES) >> 2) == 7)
		at91sam7_info->mck_valid = 0;
	else
		at91sam7_info->mck_freq = tmp >> ((mckr & PMC_MCKR_PRES) >> 2);

	/* Forget old flash timing */
	at91sam7_set_flash_mode(bank,0);
}

/* Setup the timimg registers for nvbits or normal flash */
void at91sam7_set_flash_mode(flash_bank_t *bank,int mode)
{
	u32 fmcn, fmr;
	at91sam7_flash_bank_t *at91sam7_info = bank->driver_priv;
	target_t *target = at91sam7_info->target;
	
	if (mode && (mode != at91sam7_info->flashmode)) {
		/* Always round up (ceil) */
		if (mode==1)
			/* main clocks in 1uS */
			fmcn = (at91sam7_info->mck_freq/1000000ul)+1;
		else if (mode==2)
			/* main clocks in 1.5uS */
			fmcn = (at91sam7_info->mck_freq/666666ul)+1;

		/* Only allow fmcn=0 if clock period is > 30 us. */
		if (at91sam7_info->mck_freq <= 33333)
			fmcn = 0;

		DEBUG("fmcn: %i", fmcn); 
		fmr = fmcn<<16;
		target->type->write_memory(target, MC_FSR, 4, 1, (u8 *)&fmr);
	}
	at91sam7_info->flashmode = mode;		
}

u8 at91sam7_wait_status_busy(flash_bank_t *bank, int timeout)
{
	u32 status;
	
	while ((!((status = at91sam7_get_flash_status(bank)) & 0x01)) && (timeout-- > 0))
	{
		DEBUG("status: 0x%x", status);
		usleep(1000);
	}
	
	DEBUG("status: 0x%x", status);

	if (status&0x0C)
	{
		ERROR("status register: 0x%x", status);
		if (status & 0x4)
			ERROR("Lock Error Bit Detected, Operation Abort");
		if (status & 0x8)
			ERROR("Invalid command and/or bad keyword, Operation Abort");
		if (status & 0x10)
			ERROR("Security Bit Set, Operation Abort");
	}
	
	return status;
}

/* Send one command to the AT91SAM flash controller */
int at91sam7_flash_command(struct flash_bank_s *bank,u8 cmd,u16 pagen) 
{
	u32 fcr;
	at91sam7_flash_bank_t *at91sam7_info = bank->driver_priv;
	target_t *target = at91sam7_info->target;

	fcr = (0x5A<<24) | (pagen<<8) | cmd; 
	target->type->write_memory(target, MC_FCR, 4, 1, (u8 *)&fcr);
	DEBUG("Flash command: 0x%x, pagenumber:", fcr, pagen);

	if (at91sam7_wait_status_busy(bank, 10)&0x0C) 
	{
		return ERROR_FLASH_OPERATION_FAILED;
	}		
	return ERROR_OK;
}

/* Read device id register, main clock frequency register and fill in driver info structure */
int at91sam7_read_part_info(struct flash_bank_s *bank)
{
	at91sam7_flash_bank_t *at91sam7_info = bank->driver_priv;
	target_t *target = at91sam7_info->target;
	unsigned long cidr, mcfr, status;
	
	if (at91sam7_info->target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}
	
	/* Read and parse chip identification register */
	target->type->read_memory(target, DBGU_CIDR, 4, 1, (u8 *)&cidr);
	
	if (cidr == 0)
	{
		WARNING("Cannot identify target as an AT91SAM");
		return ERROR_FLASH_OPERATION_FAILED;
	}
	
	at91sam7_info->cidr = cidr;
	at91sam7_info->cidr_ext = (cidr>>31)&0x0001;
	at91sam7_info->cidr_nvptyp = (cidr>>28)&0x0007;
	at91sam7_info->cidr_arch = (cidr>>20)&0x00FF;
	at91sam7_info->cidr_sramsiz = (cidr>>16)&0x000F;
	at91sam7_info->cidr_nvpsiz2 = (cidr>>12)&0x000F;
	at91sam7_info->cidr_nvpsiz = (cidr>>8)&0x000F;
	at91sam7_info->cidr_eproc = (cidr>>5)&0x0007;
	at91sam7_info->cidr_version = cidr&0x001F;
	bank->size = NVPSIZ[at91sam7_info->cidr_nvpsiz];
	at91sam7_info->target_name = "Unknown";
	
	DEBUG("nvptyp: 0x%3.3x, arch: 0x%4.4x, alt_id: 0x%4.4x, alt_addr: 0x%4.4x", at91sam7_info->cidr_nvptyp, at91sam7_info->cidr_arch );

	/* Read main and master clock freqency register */
	at91sam7_read_clock_info(bank);
	
	status = at91sam7_get_flash_status(bank);
	at91sam7_info->lockbits = status>>16;
	at91sam7_info->securitybit = (status>>4)&0x01;
	
	if (at91sam7_info->cidr_arch == 0x70 )
	{
		at91sam7_info->num_nvmbits = 2;
		at91sam7_info->nvmbits = (status>>8)&0x03;
		bank->base = 0x100000;
		bank->bus_width = 4;
		if (bank->size==0x40000)  /* AT91SAM7S256 */
		{
			at91sam7_info->target_name = "AT91SAM7S256";
			at91sam7_info->num_lockbits = 16;
			at91sam7_info->pagesize = 256;
			at91sam7_info->pages_in_lockregion = 64;
			at91sam7_info->num_pages = 16*64;
		}
		if (bank->size==0x20000)  /* AT91SAM7S128 */
		{
			at91sam7_info->target_name = "AT91SAM7S128";
			at91sam7_info->num_lockbits = 8;
			at91sam7_info->pagesize = 256;
			at91sam7_info->pages_in_lockregion = 64;
			at91sam7_info->num_pages = 8*64;
		}
		if (bank->size==0x10000)  /* AT91SAM7S64 */
		{
			at91sam7_info->target_name = "AT91SAM7S64";
			at91sam7_info->num_lockbits = 16;
			at91sam7_info->pagesize = 128;
			at91sam7_info->pages_in_lockregion = 32;
			at91sam7_info->num_pages = 16*32;
		}
		if (bank->size==0x08000)  /* AT91SAM7S321/32 */
		{
			at91sam7_info->target_name = "AT91SAM7S321/32";
			at91sam7_info->num_lockbits = 8;
			at91sam7_info->pagesize = 128;
			at91sam7_info->pages_in_lockregion = 32;
			at91sam7_info->num_pages = 8*32;
		}
		
		return ERROR_OK;
	}

	if (at91sam7_info->cidr_arch == 0x71 )
	{
		at91sam7_info->num_nvmbits = 2;
		at91sam7_info->nvmbits = (status>>8)&0x03;
		bank->base = 0x100000;
		bank->bus_width = 4;
		if (bank->size==0x40000)  /* AT91SAM7XC256 */
		{
			at91sam7_info->target_name = "AT91SAM7XC256";
			at91sam7_info->num_lockbits = 16;
			at91sam7_info->pagesize = 256;
			at91sam7_info->pages_in_lockregion = 64;
			at91sam7_info->num_pages = 16*64;
		}
		if (bank->size==0x20000)  /* AT91SAM7XC128 */
		{
			at91sam7_info->target_name = "AT91SAM7XC128";
			at91sam7_info->num_lockbits = 8;
			at91sam7_info->pagesize = 256;
			at91sam7_info->pages_in_lockregion = 64;
			at91sam7_info->num_pages = 8*64;
		}
		
		return ERROR_OK;
	}
	
	if (at91sam7_info->cidr_arch == 0x75 )
	{
		at91sam7_info->num_nvmbits = 3;
		at91sam7_info->nvmbits = (status>>8)&0x07;
		bank->base = 0x100000;
		bank->bus_width = 4;
		if (bank->size==0x40000)  /* AT91SAM7X256 */
		{
			at91sam7_info->target_name = "AT91SAM7X256";
			at91sam7_info->num_lockbits = 16;
			at91sam7_info->pagesize = 256;
			at91sam7_info->pages_in_lockregion = 64;
			at91sam7_info->num_pages = 16*64;
		}
		if (bank->size==0x20000)  /* AT91SAM7X128 */
		{
			at91sam7_info->target_name = "AT91SAM7X128";
			at91sam7_info->num_lockbits = 8;
			at91sam7_info->pagesize = 256;
			at91sam7_info->pages_in_lockregion = 64;
			at91sam7_info->num_pages = 8*64;
		}
	
		return ERROR_OK;
	}
	
	if (at91sam7_info->cidr_arch == 0x60 )
	{
		at91sam7_info->num_nvmbits = 3;
		at91sam7_info->nvmbits = (status>>8)&0x07;
		bank->base = 0x100000;
		bank->bus_width = 4;
		
		if (bank->size == 0x40000)  /* AT91SAM7A3 */
		{
			at91sam7_info->target_name = "AT91SAM7A3";
			at91sam7_info->num_lockbits = 16;
			at91sam7_info->pagesize = 256;
			at91sam7_info->pages_in_lockregion = 64;
			at91sam7_info->num_pages = 16*64;
		}
		return ERROR_OK;
	}
	
   WARNING("at91sam7 flash only tested for AT91SAM7Sxx series");
	
   return ERROR_OK;
}

int at91sam7_erase_check(struct flash_bank_s *bank)
{
	at91sam7_flash_bank_t *at91sam7_info = bank->driver_priv;
	target_t *target = at91sam7_info->target;
	int i;
	
	if (!at91sam7_info->working_area_size)
	{
	}
	else
	{	
	}
	
	return ERROR_OK;
}

int at91sam7_protect_check(struct flash_bank_s *bank)
{
	u32 status;
	
	at91sam7_flash_bank_t *at91sam7_info = bank->driver_priv;
	target_t *target = at91sam7_info->target;

	if (at91sam7_info->cidr == 0)
	{
		at91sam7_read_part_info(bank);
	}

	if (at91sam7_info->cidr == 0)
	{
		WARNING("Cannot identify target as an AT91SAM");
		return ERROR_FLASH_OPERATION_FAILED;
	}
		
	status = at91sam7_get_flash_status(bank);
	at91sam7_info->lockbits = status>>16;
	
	return ERROR_OK;
}

int at91sam7_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank)
{
	at91sam7_flash_bank_t *at91sam7_info;
	
	if (argc < 6)
	{
		WARNING("incomplete flash_bank at91sam7 configuration");
		return ERROR_FLASH_BANK_INVALID;
	}
	
	at91sam7_info = malloc(sizeof(at91sam7_flash_bank_t));
	bank->driver_priv = at91sam7_info;
	
	at91sam7_info->target = get_target_by_num(strtoul(args[5], NULL, 0));
	if (!at91sam7_info->target)
	{
		ERROR("no target '%i' configured", args[5]);
		exit(-1);
	}
	
	
	/* part wasn't probed for info yet */
	at91sam7_info->cidr = 0;
	
	return ERROR_OK;
}

int at91sam7_erase(struct flash_bank_s *bank, int first, int last)
{
	at91sam7_flash_bank_t *at91sam7_info = bank->driver_priv;
	
	if (at91sam7_info->target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (at91sam7_info->cidr == 0)
	{
		at91sam7_read_part_info(bank);
	}

	if (at91sam7_info->cidr == 0)
	{
		WARNING("Cannot identify target as an AT91SAM");
		return ERROR_FLASH_OPERATION_FAILED;
	}	
	
	if ((first < 0) || (last < first) || (last >= at91sam7_info->num_lockbits))
	{
		return ERROR_FLASH_SECTOR_INVALID;
	}

	/* Configure the flash controller timing */
	at91sam7_read_clock_info(bank);	
	at91sam7_set_flash_mode(bank,2);

        if ((first == 0) && (last == (at91sam7_info->num_lockbits-1)))
        {
        	return at91sam7_flash_command(bank, EA, 0);
        }

	WARNING("Can only erase the whole flash area, pages are autoerased on write");
	return ERROR_FLASH_OPERATION_FAILED;
}

int at91sam7_protect(struct flash_bank_s *bank, int set, int first, int last)
{
	u32 cmd, pagen, status;
	int lockregion;
	
	at91sam7_flash_bank_t *at91sam7_info = bank->driver_priv;
	target_t *target = at91sam7_info->target;
	
	if (at91sam7_info->target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if ((first < 0) || (last < first) || (last >= at91sam7_info->num_lockbits))
	{
		return ERROR_FLASH_SECTOR_INVALID;
	}
	
	if (at91sam7_info->cidr == 0)
	{
		at91sam7_read_part_info(bank);
	}

	if (at91sam7_info->cidr == 0)
	{
		WARNING("Cannot identify target as an AT91SAM");
		return ERROR_FLASH_OPERATION_FAILED;
	}
	
	/* Configure the flash controller timing */
	at91sam7_read_clock_info(bank);	
	at91sam7_set_flash_mode(bank,1);
	
	for (lockregion=first;lockregion<=last;lockregion++) 
	{
		pagen = lockregion*at91sam7_info->pages_in_lockregion;	
		if (set)
			 cmd = SLB; 
		else
			 cmd = CLB; 		
		if (at91sam7_flash_command(bank, cmd, pagen) != ERROR_OK) 
		{
			return ERROR_FLASH_OPERATION_FAILED;
		}	
	}
	
	status = at91sam7_get_flash_status(bank);
	at91sam7_info->lockbits = status>>16;
		
	return ERROR_OK;
}


int at91sam7_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count)
{
	at91sam7_flash_bank_t *at91sam7_info = bank->driver_priv;
	target_t *target = at91sam7_info->target;
	u32 dst_min_alignment, wcount, bytes_remaining = count;
	u32 first_page, last_page, pagen, buffer_pos;
	u32 fcr;
	
	if (at91sam7_info->target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (at91sam7_info->cidr == 0)
	{
		at91sam7_read_part_info(bank);
	}

	if (at91sam7_info->cidr == 0)
	{
		WARNING("Cannot identify target as an AT91SAM");
		return ERROR_FLASH_OPERATION_FAILED;
	}
	
	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;
	
	dst_min_alignment = at91sam7_info->pagesize;

	if (offset % dst_min_alignment)
	{
		WARNING("offset 0x%x breaks required alignment 0x%x", offset, dst_min_alignment);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}
	
	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;
	
	if (at91sam7_info->cidr_arch == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	first_page = offset/dst_min_alignment;
	last_page = CEIL(offset + count, dst_min_alignment);
	
	DEBUG("first_page: %i, last_page: %i, count %i", first_page, last_page, count);
	
	/* Configure the flash controller timing */	
	at91sam7_read_clock_info(bank);	
	at91sam7_set_flash_mode(bank,2);

	for (pagen=first_page; pagen<last_page; pagen++) {
		if (bytes_remaining<dst_min_alignment) 
		count = bytes_remaining;
		else
		count = dst_min_alignment;
		bytes_remaining -= count;
		
		/* Write one block to the PageWriteBuffer */
		buffer_pos = (pagen-first_page)*dst_min_alignment;
		wcount = CEIL(count,4);
		target->type->write_memory(target, bank->base, 4, wcount, buffer+buffer_pos);
		
		/* Send Write Page command to Flash Controller */
		if (at91sam7_flash_command(bank, WP, pagen) != ERROR_OK) 
		{
			return ERROR_FLASH_OPERATION_FAILED;
		}	
		DEBUG("Flash command: 0x%x, pagenumber:", fcr, pagen);
	}
	
	return ERROR_OK;
}


int at91sam7_probe(struct flash_bank_s *bank)
{
	/* we can't probe on an at91sam7
	 * if this is an at91sam7, it has the configured flash
	 */
	at91sam7_flash_bank_t *at91sam7_info = bank->driver_priv;
	
	if (at91sam7_info->cidr == 0)
	{
		at91sam7_read_part_info(bank);
	}

	if (at91sam7_info->cidr == 0)
	{
		WARNING("Cannot identify target as an AT91SAM");
		return ERROR_FLASH_OPERATION_FAILED;
	}
	return ERROR_OK;
}

int at91sam7_info(struct flash_bank_s *bank, char *buf, int buf_size)
{
	int printed;
	at91sam7_flash_bank_t *at91sam7_info = bank->driver_priv;
	
	at91sam7_read_part_info(bank);

	if (at91sam7_info->cidr == 0)
	{
		printed = snprintf(buf, buf_size, "Cannot identify target as an AT91SAM\n");
		buf += printed;
		buf_size -= printed;
		return ERROR_FLASH_OPERATION_FAILED;
	}
	
	printed = snprintf(buf, buf_size, "\nat91sam7 information:\n");
	buf += printed;
	buf_size -= printed;
	
	printed = snprintf(buf, buf_size, "cidr: 0x%8.8x, arch: 0x%4.4x, eproc: %s, version:0x%3.3x,  flashsize: 0x%8.8x\n", at91sam7_info->cidr, at91sam7_info->cidr_arch, EPROC[at91sam7_info->cidr_eproc], at91sam7_info->cidr_version, bank->size);
	buf += printed;
	buf_size -= printed;
			
	printed = snprintf(buf, buf_size, "master clock(estimated): %ikHz \n", at91sam7_info->mck_freq / 1000);
	buf += printed;
	buf_size -= printed;
	
	if (at91sam7_info->num_lockbits>0) {		
		printed = snprintf(buf, buf_size, "pagesize: %i, lockbits: %i 0x%4.4x, pages in lock region: %i \n", at91sam7_info->pagesize, at91sam7_info->num_lockbits, at91sam7_info->lockbits,at91sam7_info->num_pages/at91sam7_info->num_lockbits);
		buf += printed;
		buf_size -= printed;
	}
			
	printed = snprintf(buf, buf_size, "securitybit: %i, nvmbits: 0x%1.1x\n", at91sam7_info->securitybit, at91sam7_info->nvmbits);
	buf += printed;
	buf_size -= printed;

	return ERROR_OK;
}

/* 
* On AT91SAM7S: When the gpnmv bits are set with 
* > at91sam7 gpnvm 0 bitnr set
* the changes are not visible in the flash controller status register MC_FSR 
* until the processor has been reset.
* On the Olimex board this requires a power cycle.
* Note that the AT91SAM7S has the following errata (doc6175.pdf sec 14.1.3):
* 	The maximum number of write/erase cycles for Non Volatile Memory bits is 100. This includes
*	Lock Bits (LOCKx), General Purpose NVM bits (GPNVMx) and the Security Bit.
*/
int at91sam7_handle_gpnvm_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *bank;
	int bit;
	u8  flashcmd;
	u32 status;
	char *value;
	at91sam7_flash_bank_t *at91sam7_info;

	if (argc < 3)
	{
		command_print(cmd_ctx, "at91sam7 gpnvm <num> <bit> <set|clear>");
		return ERROR_OK;
	}
	
	bank = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	bit = atoi(args[1]);
	value = args[2];

	if (!bank)
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
		return ERROR_OK;
	}

	at91sam7_info = bank->driver_priv;

	if (at91sam7_info->target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (at91sam7_info->cidr == 0)
	{
		at91sam7_read_part_info(bank);
	}

	if (at91sam7_info->cidr == 0)
	{
		WARNING("Cannot identify target as an AT91SAM");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	if ((bit<0) || (at91sam7_info->num_nvmbits <= bit))
	{ 
		command_print(cmd_ctx, "gpnvm bit '#%s' is out of bounds for target %s", args[1],at91sam7_info->target_name);
		return ERROR_OK;
	}

	if (strcmp(value, "set") == 0)
	{
		flashcmd = SGPB;
	}
	else if (strcmp(value, "clear") == 0)
	{
		flashcmd = CGPB;
	}
	else
	{
		command_print(cmd_ctx, "usage: at91sam7 gpnvm <num> <bit> <set|clear>");
		return ERROR_OK;
	}

	/* Configure the flash controller timing */
	at91sam7_read_clock_info(bank);	
	at91sam7_set_flash_mode(bank,1);
	
	if (at91sam7_flash_command(bank, flashcmd, (u16)(bit)) != ERROR_OK) 
	{
		return ERROR_FLASH_OPERATION_FAILED;
	}	

	status = at91sam7_get_flash_status(bank);
	DEBUG("at91sam7_handle_gpnvm_command: cmd 0x%x, value 0x%x, status 0x%x \n",flashcmd,bit,status);
	at91sam7_info->nvmbits = (status>>8)&((1<<at91sam7_info->num_nvmbits)-1);

	return ERROR_OK;
}
