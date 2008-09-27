/***************************************************************************
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
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

#include "at91sam7_old.h"

#include "flash.h"
#include "target.h"
#include "log.h"
#include "binarybuffer.h"
#include "types.h"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int at91sam7_old_register_commands(struct command_context_s *cmd_ctx);
int at91sam7_old_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank);
int at91sam7_old_erase(struct flash_bank_s *bank, int first, int last);
int at91sam7_old_protect(struct flash_bank_s *bank, int set, int first, int last);
int at91sam7_old_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count);
int at91sam7_old_probe(struct flash_bank_s *bank);
int at91sam7_old_auto_probe(struct flash_bank_s *bank);
int at91sam7_old_erase_check(struct flash_bank_s *bank);
int at91sam7_old_protect_check(struct flash_bank_s *bank);
int at91sam7_old_info(struct flash_bank_s *bank, char *buf, int buf_size);

u32 at91sam7_old_get_flash_status(flash_bank_t *bank, u8 flashplane);
void at91sam7_old_set_flash_mode(flash_bank_t *bank, u8 flashplane, int mode);
u32 at91sam7_old_wait_status_busy(flash_bank_t *bank, u8 flashplane, u32 waitbits, int timeout);
int at91sam7_old_flash_command(struct flash_bank_s *bank, u8 flashplane, u8 cmd, u16 pagen); 
int at91sam7_old_handle_gpnvm_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

flash_driver_t at91sam7_old_flash =
{
	.name = "at91sam7",
	.register_commands = at91sam7_old_register_commands,
	.flash_bank_command = at91sam7_old_flash_bank_command,
	.erase = at91sam7_old_erase,
	.protect = at91sam7_old_protect,
	.write = at91sam7_old_write,
	.probe = at91sam7_old_probe,
	.auto_probe = at91sam7_old_probe,
	.erase_check = at91sam7_old_erase_check,
	.protect_check = at91sam7_old_protect_check,
	.info = at91sam7_old_info
};

u32 MC_FMR_old[4] =	{ 0xFFFFFF60, 0xFFFFFF70, 0xFFFFFF80, 0xFFFFFF90 };
u32 MC_FCR_old[4] =	{ 0xFFFFFF64, 0xFFFFFF74, 0xFFFFFF84, 0xFFFFFF94 };
u32 MC_FSR_old[4] =	{ 0xFFFFFF68, 0xFFFFFF78, 0xFFFFFF88, 0xFFFFFF98 };

char * EPROC_old[8]= {"Unknown","ARM946-E","ARM7TDMI","Unknown","ARM920T","ARM926EJ-S","Unknown","Unknown"};
long NVPSIZ_old[16] = {
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

long SRAMSIZ_old[16] = {
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

int at91sam7_old_register_commands(struct command_context_s *cmd_ctx)
{
	command_t *at91sam7_old_cmd = register_command(cmd_ctx, NULL, "at91sam7", NULL, COMMAND_ANY, NULL);
	register_command(cmd_ctx, at91sam7_old_cmd, "gpnvm", at91sam7_old_handle_gpnvm_command, COMMAND_EXEC,
			"at91sam7 gpnvm <num> <bit> set|clear, set or clear at91sam7 gpnvm bit");

	return ERROR_OK;
}

u32 at91sam7_old_get_flash_status(flash_bank_t *bank, u8 flashplane)
{
	target_t *target = bank->target;
	u32 fsr;
	
	target_read_u32(target, MC_FSR_old[flashplane], &fsr);
	
	return fsr;
}

/* Read clock configuration and set at91sam7_old_info->usec_clocks*/
void at91sam7_old_read_clock_info(flash_bank_t *bank)
{
	at91sam7_old_flash_bank_t *at91sam7_old_info = bank->driver_priv;
	target_t *target = bank->target;
	u32 mckr, mcfr, pllr;
	unsigned long tmp = 0, mainfreq;
	int flashplane;

	/* Read main clock freqency register */
	target_read_u32(target, CKGR_MCFR_old, &mcfr);
	/* Read master clock register */
	target_read_u32(target, PMC_MCKR_old, &mckr);
	/* Read Clock Generator PLL Register  */
	target_read_u32(target, CKGR_PLLR_old, &pllr);

	at91sam7_old_info->mck_valid = 0;
	switch (mckr & PMC_MCKR_CSS_old) 
	{
		case 0:			/* Slow Clock */
			at91sam7_old_info->mck_valid = 1;
			mainfreq = RC_FREQ_old / 16ul * (mcfr & 0xffff);
			tmp = mainfreq;
			break;
		case 1:			/* Main Clock */
			if (mcfr & CKGR_MCFR_MAINRDY_old) 
			{
				at91sam7_old_info->mck_valid = 1;
				mainfreq = RC_FREQ_old / 16ul * (mcfr & 0xffff);
				tmp = mainfreq;
			}
			break;

		case 2:			/* Reserved */
			break;
		case 3:			/* PLL Clock */
			if (mcfr & CKGR_MCFR_MAINRDY_old) 
			{
				target_read_u32(target, CKGR_PLLR_old, &pllr);
				if (!(pllr & CKGR_PLLR_DIV_old))
					break; /* 0 Hz */
				at91sam7_old_info->mck_valid = 1;
				mainfreq = RC_FREQ_old / 16ul * (mcfr & 0xffff);
				/* Integer arithmetic should have sufficient precision
				   as long as PLL is properly configured. */
				tmp = mainfreq / (pllr & CKGR_PLLR_DIV_old) *
				  (((pllr & CKGR_PLLR_MUL_old) >> 16) + 1);
			}
			break;
	}
	
	/* Prescaler adjust */
	if (((mckr & PMC_MCKR_PRES_old) >> 2) == 7)
		at91sam7_old_info->mck_valid = 0;
	else
		at91sam7_old_info->mck_freq = tmp >> ((mckr & PMC_MCKR_PRES_old) >> 2);

	/* Forget old flash timing */
	for (flashplane = 0; flashplane<at91sam7_old_info->num_planes; flashplane++)
	{
		at91sam7_old_set_flash_mode(bank, flashplane, FMR_TIMING_NONE_old);
	}
}

/* Setup the timimg registers for nvbits or normal flash */
void at91sam7_old_set_flash_mode(flash_bank_t *bank, u8 flashplane, int mode)
{
	u32 fmr, fmcn = 0, fws = 0;
	at91sam7_old_flash_bank_t *at91sam7_old_info = bank->driver_priv;
	target_t *target = bank->target;
	
	if (mode && (mode != at91sam7_old_info->flashmode[flashplane]))
	{
		/* Always round up (ceil) */
		if (mode==FMR_TIMING_NVBITS_old)
		{
			if (at91sam7_old_info->cidr_arch == 0x60)
			{
				/* AT91SAM7A3 uses master clocks in 100 ns */
				fmcn = (at91sam7_old_info->mck_freq/10000000ul)+1;
			}
			else
			{
				/* master clocks in 1uS for ARCH 0x7 types */
				fmcn = (at91sam7_old_info->mck_freq/1000000ul)+1;
			}
		}
		else if (mode==FMR_TIMING_FLASH_old)
			/* main clocks in 1.5uS */
			fmcn = (at91sam7_old_info->mck_freq/666666ul)+1;

		/* Only allow fmcn=0 if clock period is > 30 us = 33kHz. */
 		if (at91sam7_old_info->mck_freq <= 33333ul)
			fmcn = 0;
		/* Only allow fws=0 if clock frequency is < 30 MHz. */
		if (at91sam7_old_info->mck_freq > 30000000ul)
			fws = 1;

		LOG_DEBUG("fmcn[%i]: %i", flashplane, fmcn); 
		fmr = fmcn << 16 | fws << 8;
		target_write_u32(target, MC_FMR_old[flashplane], fmr);
	}
	
	at91sam7_old_info->flashmode[flashplane] = mode;		
}

u32 at91sam7_old_wait_status_busy(flash_bank_t *bank, u8 flashplane, u32 waitbits, int timeout)
{
	u32 status;
	
	while ((!((status = at91sam7_old_get_flash_status(bank,flashplane)) & waitbits)) && (timeout-- > 0))
	{
		LOG_DEBUG("status[%i]: 0x%x", flashplane, status);
		alive_sleep(1);
	}
	
	LOG_DEBUG("status[%i]: 0x%x", flashplane, status);

	if (status & 0x0C)
	{
		LOG_ERROR("status register: 0x%x", status);
		if (status & 0x4)
			LOG_ERROR("Lock Error Bit Detected, Operation Abort");
		if (status & 0x8)
			LOG_ERROR("Invalid command and/or bad keyword, Operation Abort");
		if (status & 0x10)
			LOG_ERROR("Security Bit Set, Operation Abort");
	}
	
	return status;
}


/* Send one command to the AT91SAM flash controller */
int at91sam7_old_flash_command(struct flash_bank_s *bank, u8 flashplane, u8 cmd, u16 pagen) 
{
	u32 fcr;
	at91sam7_old_flash_bank_t *at91sam7_old_info = bank->driver_priv;
	target_t *target = bank->target;

	fcr = (0x5A<<24) | ((pagen&0x3FF)<<8) | cmd; 
	target_write_u32(target, MC_FCR_old[flashplane], fcr);
	LOG_DEBUG("Flash command: 0x%x, flashplane: %i, pagenumber:%u", fcr, flashplane, pagen);

	if ((at91sam7_old_info->cidr_arch == 0x60)&&((cmd==SLB_old)|(cmd==CLB_old)))
	{
		/* Lock bit manipulation on AT91SAM7A3 waits for FC_FSR bit 1, EOL */
		if (at91sam7_old_wait_status_busy(bank, flashplane, MC_FSR_EOL_old, 10)&0x0C) 
		{
			return ERROR_FLASH_OPERATION_FAILED;
		}
		return ERROR_OK;
	}

	if (at91sam7_old_wait_status_busy(bank, flashplane, MC_FSR_FRDY_old, 10)&0x0C) 
	{
		return ERROR_FLASH_OPERATION_FAILED;
	}
	return ERROR_OK;
}

/* Read device id register, main clock frequency register and fill in driver info structure */
int at91sam7_old_read_part_info(struct flash_bank_s *bank)
{
	at91sam7_old_flash_bank_t *at91sam7_old_info = bank->driver_priv;
	target_t *target = bank->target;
	u32 cidr, status;
	int sectornum;

	if (at91sam7_old_info->cidr != 0)
		return ERROR_OK; /* already probed, multiple probes may cause memory leak, not allowed */
	
	/* Read and parse chip identification register */
	target_read_u32(target, DBGU_CIDR_old, &cidr);
	
	if (cidr == 0)
	{
		LOG_WARNING("Cannot identify target as an AT91SAM");
		return ERROR_FLASH_OPERATION_FAILED;
	}
	
	at91sam7_old_info->cidr = cidr;
	at91sam7_old_info->cidr_ext = (cidr>>31)&0x0001;
	at91sam7_old_info->cidr_nvptyp = (cidr>>28)&0x0007;
	at91sam7_old_info->cidr_arch = (cidr>>20)&0x00FF;
	at91sam7_old_info->cidr_sramsiz = (cidr>>16)&0x000F;
	at91sam7_old_info->cidr_nvpsiz2 = (cidr>>12)&0x000F;
	at91sam7_old_info->cidr_nvpsiz = (cidr>>8)&0x000F;
	at91sam7_old_info->cidr_eproc = (cidr>>5)&0x0007;
	at91sam7_old_info->cidr_version = cidr&0x001F;
	bank->size = NVPSIZ_old[at91sam7_old_info->cidr_nvpsiz];
	at91sam7_old_info->target_name = "Unknown";

	/* Support just for bulk erase of a single flash plane, whole device if flash size <= 256k */
	if (NVPSIZ_old[at91sam7_old_info->cidr_nvpsiz]<0x80000)  /* Flash size less than 512K, one flash plane */
	{
		bank->num_sectors = 1;
		bank->sectors = malloc(sizeof(flash_sector_t));
		bank->sectors[0].offset = 0;
		bank->sectors[0].size = bank->size;
		bank->sectors[0].is_erased = -1;
		bank->sectors[0].is_protected = -1;
	}
	else	/* Flash size 512K or larger, several flash planes */
	{
		bank->num_sectors = NVPSIZ_old[at91sam7_old_info->cidr_nvpsiz]/0x40000;
		bank->sectors = malloc(bank->num_sectors*sizeof(flash_sector_t));
		for (sectornum=0; sectornum<bank->num_sectors; sectornum++)
		{
			bank->sectors[sectornum].offset = sectornum*0x40000;
			bank->sectors[sectornum].size = 0x40000;
			bank->sectors[sectornum].is_erased = -1;
			bank->sectors[sectornum].is_protected = -1;
		}
	}
		
	

	LOG_DEBUG("nvptyp: 0x%3.3x, arch: 0x%4.4x", at91sam7_old_info->cidr_nvptyp, at91sam7_old_info->cidr_arch );

	/* Read main and master clock freqency register */
	at91sam7_old_read_clock_info(bank);
	
	at91sam7_old_info->num_planes = 1;
	status = at91sam7_old_get_flash_status(bank, 0);
	at91sam7_old_info->securitybit = (status>>4)&0x01;
	at91sam7_old_protect_check(bank);   /* TODO Check the protect check */
	
	if (at91sam7_old_info->cidr_arch == 0x70 )
	{
		at91sam7_old_info->num_nvmbits = 2;
		at91sam7_old_info->nvmbits = (status>>8)&0x03;
		bank->base = 0x100000;
		bank->bus_width = 4;
		if (bank->size==0x80000)  /* AT91SAM7S512 */
		{
			at91sam7_old_info->target_name = "AT91SAM7S512";
			at91sam7_old_info->num_planes = 2;
			if (at91sam7_old_info->num_planes != bank->num_sectors)
				LOG_WARNING("Internal error: Number of flash planes and erase sectors does not match, please report");;
			at91sam7_old_info->num_lockbits = 2*16;
			at91sam7_old_info->pagesize = 256;
			at91sam7_old_info->pages_in_lockregion = 64;
			at91sam7_old_info->num_pages = 2*16*64;
		}
		if (bank->size==0x40000)  /* AT91SAM7S256 */
		{
			at91sam7_old_info->target_name = "AT91SAM7S256";
			at91sam7_old_info->num_lockbits = 16;
			at91sam7_old_info->pagesize = 256;
			at91sam7_old_info->pages_in_lockregion = 64;
			at91sam7_old_info->num_pages = 16*64;
		}
		if (bank->size==0x20000)  /* AT91SAM7S128 */
		{
			at91sam7_old_info->target_name = "AT91SAM7S128";
			at91sam7_old_info->num_lockbits = 8;
			at91sam7_old_info->pagesize = 256;
			at91sam7_old_info->pages_in_lockregion = 64;
			at91sam7_old_info->num_pages = 8*64;
		}
		if (bank->size==0x10000)  /* AT91SAM7S64 */
		{
			at91sam7_old_info->target_name = "AT91SAM7S64";
			at91sam7_old_info->num_lockbits = 16;
			at91sam7_old_info->pagesize = 128;
			at91sam7_old_info->pages_in_lockregion = 32;
			at91sam7_old_info->num_pages = 16*32;
		}
		if (bank->size==0x08000)  /* AT91SAM7S321/32 */
		{
			at91sam7_old_info->target_name = "AT91SAM7S321/32";
			at91sam7_old_info->num_lockbits = 8;
			at91sam7_old_info->pagesize = 128;
			at91sam7_old_info->pages_in_lockregion = 32;
			at91sam7_old_info->num_pages = 8*32;
		}
		
		return ERROR_OK;
	}

	if (at91sam7_old_info->cidr_arch == 0x71 )
	{
		at91sam7_old_info->num_nvmbits = 3;
		at91sam7_old_info->nvmbits = (status>>8)&0x07;
		bank->base = 0x100000;
		bank->bus_width = 4;
		if (bank->size==0x80000)  /* AT91SAM7XC512 */
		{
			at91sam7_old_info->target_name = "AT91SAM7XC512";
			at91sam7_old_info->num_planes = 2;
			if (at91sam7_old_info->num_planes != bank->num_sectors)
				LOG_WARNING("Internal error: Number of flash planes and erase sectors does not match, please report");;
			at91sam7_old_info->num_lockbits = 2*16;
			at91sam7_old_info->pagesize = 256;
			at91sam7_old_info->pages_in_lockregion = 64;
			at91sam7_old_info->num_pages = 2*16*64;
		}
		if (bank->size==0x40000)  /* AT91SAM7XC256 */
		{
			at91sam7_old_info->target_name = "AT91SAM7XC256";
			at91sam7_old_info->num_lockbits = 16;
			at91sam7_old_info->pagesize = 256;
			at91sam7_old_info->pages_in_lockregion = 64;
			at91sam7_old_info->num_pages = 16*64;
		}
		if (bank->size==0x20000)  /* AT91SAM7XC128 */
		{
			at91sam7_old_info->target_name = "AT91SAM7XC128";
			at91sam7_old_info->num_lockbits = 8;
			at91sam7_old_info->pagesize = 256;
			at91sam7_old_info->pages_in_lockregion = 64;
			at91sam7_old_info->num_pages = 8*64;
		}
		
		return ERROR_OK;
	}
	
	if (at91sam7_old_info->cidr_arch == 0x72 )
	{
		at91sam7_old_info->num_nvmbits = 3;
		at91sam7_old_info->nvmbits = (status>>8)&0x07;
		bank->base = 0x100000;
		bank->bus_width = 4;
		if (bank->size==0x80000) /* AT91SAM7SE512 */
		{
			at91sam7_old_info->target_name = "AT91SAM7SE512";
			at91sam7_old_info->num_planes = 2;
			if (at91sam7_old_info->num_planes != bank->num_sectors)
				LOG_WARNING("Internal error: Number of flash planes and erase sectors does not match, please report");;
			at91sam7_old_info->num_lockbits = 32;
			at91sam7_old_info->pagesize = 256;
			at91sam7_old_info->pages_in_lockregion = 64;
			at91sam7_old_info->num_pages = 32*64;
		}
		if (bank->size==0x40000)
		{
			at91sam7_old_info->target_name = "AT91SAM7SE256";
			at91sam7_old_info->num_lockbits = 16;
			at91sam7_old_info->pagesize = 256;
			at91sam7_old_info->pages_in_lockregion = 64;
			at91sam7_old_info->num_pages = 16*64;
		}
		if (bank->size==0x08000)
		{
			at91sam7_old_info->target_name = "AT91SAM7SE32";
			at91sam7_old_info->num_lockbits = 8;
			at91sam7_old_info->pagesize = 128;
			at91sam7_old_info->pages_in_lockregion = 32;
			at91sam7_old_info->num_pages = 8*32;
		}
		
		return ERROR_OK;
	}
	
	if (at91sam7_old_info->cidr_arch == 0x75 )
	{
		at91sam7_old_info->num_nvmbits = 3;
		at91sam7_old_info->nvmbits = (status>>8)&0x07;
		bank->base = 0x100000;
		bank->bus_width = 4;
		if (bank->size==0x80000)  /* AT91SAM7X512 */
		{
			at91sam7_old_info->target_name = "AT91SAM7X512";
			at91sam7_old_info->num_planes = 2;
			if (at91sam7_old_info->num_planes != bank->num_sectors)
				LOG_WARNING("Internal error: Number of flash planes and erase sectors does not match, please report");;
			at91sam7_old_info->num_lockbits = 32;
			at91sam7_old_info->pagesize = 256;
			at91sam7_old_info->pages_in_lockregion = 64;
			at91sam7_old_info->num_pages = 2*16*64;
			LOG_DEBUG("Support for AT91SAM7X512 is experimental in this version!");
		}
		if (bank->size==0x40000)  /* AT91SAM7X256 */
		{
			at91sam7_old_info->target_name = "AT91SAM7X256";
			at91sam7_old_info->num_lockbits = 16;
			at91sam7_old_info->pagesize = 256;
			at91sam7_old_info->pages_in_lockregion = 64;
			at91sam7_old_info->num_pages = 16*64;
		}
		if (bank->size==0x20000)  /* AT91SAM7X128 */
		{
			at91sam7_old_info->target_name = "AT91SAM7X128";
			at91sam7_old_info->num_lockbits = 8;
			at91sam7_old_info->pagesize = 256;
			at91sam7_old_info->pages_in_lockregion = 64;
			at91sam7_old_info->num_pages = 8*64;
		}
	
		return ERROR_OK;
	}
	
	if (at91sam7_old_info->cidr_arch == 0x60 )
	{
		at91sam7_old_info->num_nvmbits = 3;
		at91sam7_old_info->nvmbits = (status>>8)&0x07;
		bank->base = 0x100000;
		bank->bus_width = 4;
		
		if (bank->size == 0x40000)  /* AT91SAM7A3 */
		{
			at91sam7_old_info->target_name = "AT91SAM7A3";
			at91sam7_old_info->num_lockbits = 16;
			at91sam7_old_info->pagesize = 256;
			at91sam7_old_info->pages_in_lockregion = 16;
			at91sam7_old_info->num_pages = 16*64;
		}
		return ERROR_OK;
	}
	
	LOG_WARNING("at91sam7_old flash only tested for AT91SAM7Sxx series");
	return ERROR_OK;
}

int at91sam7_old_erase_check(struct flash_bank_s *bank)
{
	at91sam7_old_flash_bank_t *at91sam7_old_info = bank->driver_priv;
	
	if (!at91sam7_old_info->working_area_size)
	{
	}
	else
	{	
	}
	
	return ERROR_OK;
}

int at91sam7_old_protect_check(struct flash_bank_s *bank)
{
	u32 status;
	int flashplane;
	
	at91sam7_old_flash_bank_t *at91sam7_old_info = bank->driver_priv;

	if (at91sam7_old_info->cidr == 0)
	{
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	for (flashplane=0;flashplane<at91sam7_old_info->num_planes;flashplane++)
	{
		status = at91sam7_old_get_flash_status(bank, flashplane);
		at91sam7_old_info->lockbits[flashplane] = (status >> 16);
	}
	
	return ERROR_OK;
}

/* flash_bank at91sam7_old 0 0 0 0 <target#>
 */
int at91sam7_old_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank)
{
	at91sam7_old_flash_bank_t *at91sam7_old_info;
	int i;
	
	if (argc < 6)
	{
		LOG_WARNING("incomplete flash_bank at91sam7_old configuration");
		return ERROR_FLASH_BANK_INVALID;
	}
	
	at91sam7_old_info = malloc(sizeof(at91sam7_old_flash_bank_t));
	bank->driver_priv = at91sam7_old_info;
	
	/* part wasn't probed for info yet */
	at91sam7_old_info->cidr = 0;
	for (i=0;i<4;i++)
		at91sam7_old_info->flashmode[i]=0;
	
	return ERROR_OK;
}

int at91sam7_old_erase(struct flash_bank_s *bank, int first, int last)
{
	at91sam7_old_flash_bank_t *at91sam7_old_info = bank->driver_priv;
	u8 flashplane;

	if (at91sam7_old_info->cidr == 0)
	{
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if ((first < 0) || (last < first) || (last >= bank->num_sectors))
	{
		if ((first == 0) && (last == (at91sam7_old_info->num_lockbits-1)))
		{
			LOG_WARNING("Sector numbers based on lockbit count, probably a deprecated script");
			last = bank->num_sectors-1;
		}
		else return ERROR_FLASH_SECTOR_INVALID;
	}

	/* Configure the flash controller timing */
	at91sam7_old_read_clock_info(bank);	
	for (flashplane = first; flashplane<=last; flashplane++)
	{
		/* Configure the flash controller timing */
		at91sam7_old_set_flash_mode(bank, flashplane, FMR_TIMING_FLASH_old);
		if (at91sam7_old_flash_command(bank, flashplane, EA_old, 0) != ERROR_OK) 
		{
			return ERROR_FLASH_OPERATION_FAILED;
		}	
	}
	return ERROR_OK;

}

int at91sam7_old_protect(struct flash_bank_s *bank, int set, int first, int last)
{
	u32 cmd, pagen;
	u8 flashplane;
	int lockregion;
	
	at91sam7_old_flash_bank_t *at91sam7_old_info = bank->driver_priv;
	
	if (at91sam7_old_info->cidr == 0)
	{
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if ((first < 0) || (last < first) || (last >= at91sam7_old_info->num_lockbits))
	{
		return ERROR_FLASH_SECTOR_INVALID;
	}
	
	at91sam7_old_read_clock_info(bank);	
	
	for (lockregion=first;lockregion<=last;lockregion++) 
	{
		pagen = lockregion*at91sam7_old_info->pages_in_lockregion;
		flashplane = (pagen>>10)&0x03;
		/* Configure the flash controller timing */
		at91sam7_old_set_flash_mode(bank, flashplane, FMR_TIMING_NVBITS_old);
		
		if (set)
			 cmd = SLB_old; 
		else
			 cmd = CLB_old; 		

		if (at91sam7_old_flash_command(bank, flashplane, cmd, pagen) != ERROR_OK) 
		{
			return ERROR_FLASH_OPERATION_FAILED;
		}	
	}
	
	at91sam7_old_protect_check(bank);
		
	return ERROR_OK;
}


int at91sam7_old_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count)
{
	at91sam7_old_flash_bank_t *at91sam7_old_info = bank->driver_priv;
	target_t *target = bank->target;
	u32 dst_min_alignment, wcount, bytes_remaining = count;
	u32 first_page, last_page, pagen, buffer_pos;
	u8 flashplane;
	
	if (at91sam7_old_info->cidr == 0)
	{
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;
	
	dst_min_alignment = at91sam7_old_info->pagesize;

	if (offset % dst_min_alignment)
	{
		LOG_WARNING("offset 0x%x breaks required alignment 0x%x", offset, dst_min_alignment);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}
	
	if (at91sam7_old_info->cidr_arch == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	first_page = offset/dst_min_alignment;
	last_page = CEIL(offset + count, dst_min_alignment);
	
	LOG_DEBUG("first_page: %i, last_page: %i, count %i", first_page, last_page, count);
	
	at91sam7_old_read_clock_info(bank);	

	for (pagen=first_page; pagen<last_page; pagen++) 
	{
		if (bytes_remaining<dst_min_alignment)
			count = bytes_remaining;
		else
			count = dst_min_alignment;
		bytes_remaining -= count;
		
		/* Write one block to the PageWriteBuffer */
		buffer_pos = (pagen-first_page)*dst_min_alignment;
		wcount = CEIL(count,4);
		target->type->write_memory(target, bank->base+pagen*dst_min_alignment, 4, wcount, buffer+buffer_pos);
		flashplane = (pagen>>10)&0x3;
		
		/* Configure the flash controller timing */	
		at91sam7_old_set_flash_mode(bank, flashplane, FMR_TIMING_FLASH_old);
		/* Send Write Page command to Flash Controller */
		if (at91sam7_old_flash_command(bank, flashplane, WP_old, pagen) != ERROR_OK) 
		{
				return ERROR_FLASH_OPERATION_FAILED;
		}
		LOG_DEBUG("Write flash plane:%i page number:%i", flashplane, pagen);
	}
	
	return ERROR_OK;
}


int at91sam7_old_probe(struct flash_bank_s *bank)
{
	/* we can't probe on an at91sam7_old
	 * if this is an at91sam7_old, it has the configured flash
	 */
	at91sam7_old_flash_bank_t *at91sam7_old_info = bank->driver_priv;
	int retval;
	
	if (at91sam7_old_info->cidr != 0)
	{
		return ERROR_OK; /* already probed */
	}

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = at91sam7_old_read_part_info(bank);
	if (retval != ERROR_OK)
		return retval;
	
	return ERROR_OK;
}


int at91sam7_old_info(struct flash_bank_s *bank, char *buf, int buf_size)
{
	int printed, flashplane;
	at91sam7_old_flash_bank_t *at91sam7_old_info = bank->driver_priv;
	
	if (at91sam7_old_info->cidr == 0)
	{
		return ERROR_FLASH_BANK_NOT_PROBED;
	}
	
	printed = snprintf(buf, buf_size, "\nat91sam7_old information: Chip is %s\n",at91sam7_old_info->target_name);
	buf += printed;
	buf_size -= printed;
	
	printed = snprintf(buf, buf_size, "cidr: 0x%8.8x, arch: 0x%4.4x, eproc: %s, version:0x%3.3x,  flashsize: 0x%8.8x\n",
		  at91sam7_old_info->cidr, at91sam7_old_info->cidr_arch, EPROC_old[at91sam7_old_info->cidr_eproc], at91sam7_old_info->cidr_version, bank->size);
	buf += printed;
	buf_size -= printed;
			
	printed = snprintf(buf, buf_size, "master clock(estimated): %ikHz \n", at91sam7_old_info->mck_freq / 1000);
	buf += printed;
	buf_size -= printed;
	
	if (at91sam7_old_info->num_planes>1) {		
		printed = snprintf(buf, buf_size, "flashplanes: %i, pagesize: %i, lock regions: %i, pages in lock region: %i \n", 
			   at91sam7_old_info->num_planes, at91sam7_old_info->pagesize, at91sam7_old_info->num_lockbits, at91sam7_old_info->num_pages/at91sam7_old_info->num_lockbits);
		buf += printed;
		buf_size -= printed;
		for (flashplane=0; flashplane<at91sam7_old_info->num_planes; flashplane++)
		{
			printed = snprintf(buf, buf_size, "lockbits[%i]: 0x%4.4x,  ", flashplane, at91sam7_old_info->lockbits[flashplane]);
			buf += printed;
			buf_size -= printed;
		}
	}
	else
	if (at91sam7_old_info->num_lockbits>0) {		
		printed = snprintf(buf, buf_size, "pagesize: %i, lockbits: %i 0x%4.4x, pages in lock region: %i \n", 
			   at91sam7_old_info->pagesize, at91sam7_old_info->num_lockbits, at91sam7_old_info->lockbits[0], at91sam7_old_info->num_pages/at91sam7_old_info->num_lockbits);
		buf += printed;
		buf_size -= printed;
	}
			
	printed = snprintf(buf, buf_size, "securitybit: %i,  nvmbits(%i): 0x%1.1x\n", at91sam7_old_info->securitybit, at91sam7_old_info->num_nvmbits, at91sam7_old_info->nvmbits);
	buf += printed;
	buf_size -= printed;

	return ERROR_OK;
}

/* 
* On AT91SAM7S: When the gpnvm bits are set with 
* > at91sam7_old gpnvm 0 bitnr set
* the changes are not visible in the flash controller status register MC_FSR_old 
* until the processor has been reset.
* On the Olimex board this requires a power cycle.
* Note that the AT91SAM7S has the following errata (doc6175.pdf sec 14.1.3):
* 	The maximum number of write/erase cycles for Non Volatile Memory bits is 100. This includes
*	Lock Bits (LOCKx), General Purpose NVM bits (GPNVMx) and the Security Bit.
*/
int at91sam7_old_handle_gpnvm_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *bank;
	int bit;
	u8  flashcmd;
	u32 status;
	char *value;
	at91sam7_old_flash_bank_t *at91sam7_old_info;
	int retval;

	if (argc < 3)
	{
		command_print(cmd_ctx, "at91sam7_old gpnvm <num> <bit> <set|clear>");
		return ERROR_OK;
	}
	
	bank = get_flash_bank_by_num_noprobe(strtoul(args[0], NULL, 0));
	bit = atoi(args[1]);
	value = args[2];

	if (bank ==  NULL)
	{
		return ERROR_FLASH_BANK_INVALID;
	}

	if (bank->driver != &at91sam7_old_flash)
	{
		command_print(cmd_ctx, "not an at91sam7_old flash bank '%s'", args[0]);
		return ERROR_FLASH_BANK_INVALID;
	}

	if (strcmp(value, "set") == 0)
	{
		flashcmd = SGPB_old;
	}
	else if (strcmp(value, "clear") == 0)
	{
		flashcmd = CGPB_old;
	}
	else
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	at91sam7_old_info = bank->driver_priv;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("target has to be halted to perform flash operation");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (at91sam7_old_info->cidr == 0)
	{
		retval = at91sam7_old_read_part_info(bank);
		if (retval != ERROR_OK) {
			return retval;
		}
	}

	if ((bit<0) || (at91sam7_old_info->num_nvmbits <= bit))
	{ 
		command_print(cmd_ctx, "gpnvm bit '#%s' is out of bounds for target %s", args[1],at91sam7_old_info->target_name);
		return ERROR_OK;
	}

	/* Configure the flash controller timing */
	at91sam7_old_read_clock_info(bank);	
	at91sam7_old_set_flash_mode(bank, 0, FMR_TIMING_NVBITS_old);
	
	if (at91sam7_old_flash_command(bank, 0, flashcmd, (u16)(bit)) != ERROR_OK) 
	{
		return ERROR_FLASH_OPERATION_FAILED;
	}	

	status = at91sam7_old_get_flash_status(bank, 0);
	LOG_DEBUG("at91sam7_handle_gpnvm_command: cmd 0x%x, value 0x%x, status 0x%x \n",flashcmd,bit,status);
	at91sam7_old_info->nvmbits = (status>>8)&((1<<at91sam7_old_info->num_nvmbits)-1);

	return ERROR_OK;
}
