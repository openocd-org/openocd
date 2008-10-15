/***************************************************************************
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Gheorghe Guran (atlas)                          *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS for A PARTICULAR PURPOSE.  See the         *
 *   GNU General public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
****************************************************************************/

/***************************************************************************************************************************************************************************************
*
* New flash setup command:
*
* flash bank <driver> <base_addr> <size> <chip_width> <bus_width> <target_number> [<target_name> <banks> <sectors_per_bank> <pages_per_sector> <page_size> <num_nvmbits> <ext_freq_khz>]
*
*   <ext_freq_khz> - MUST be used if clock is from external source,
*                    CAN be used if main oscillator frequency is known (recomended)
* Examples:
*  flash bank at91sam7 0x00100000 0 0 4 0 0 AT91SAM7XC256 1 16 64 256 3 25000                   ==== RECOMENDED ============
*  flash bank at91sam7 0 0 0 0 0 0 0 0 0 0 0 0 25000    (auto-detection, except for clock)      ==== RECOMENDED ============
*  flash bank at91sam7 0x00100000 0 0 4 0 0 AT91SAM7XC256 1 16 64 256 3 0                       ==== NOT RECOMENDED !!! ====
*  flash bank at91sam7 0 0 0 0 0            (old style, full auto-detection)                    ==== NOT RECOMENDED !!! ====
****************************************************************************************************************************************************************************************/

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
int at91sam7_auto_probe(struct flash_bank_s *bank);
int at91sam7_erase_check(struct flash_bank_s *bank);
int at91sam7_protect_check(struct flash_bank_s *bank);
int at91sam7_info(struct flash_bank_s *bank, char *buf, int buf_size);

u32 at91sam7_get_flash_status(target_t *target, int bank_number);
void at91sam7_set_flash_mode(flash_bank_t *bank, int mode);
u32 at91sam7_wait_status_busy(flash_bank_t *bank, u32 waitbits, int timeout);
int at91sam7_flash_command(struct flash_bank_s *bank, u8 cmd, u16 pagen); 
int at91sam7_handle_gpnvm_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

flash_driver_t at91sam7_flash =
{
	.name = "at91sam7_new",
	.register_commands = at91sam7_register_commands,
	.flash_bank_command = at91sam7_flash_bank_command,
	.erase = at91sam7_erase,
	.protect = at91sam7_protect,
	.write = at91sam7_write,
	.probe = at91sam7_probe,
	.auto_probe = at91sam7_probe,
	.erase_check = at91sam7_erase_check,
	.protect_check = at91sam7_protect_check,
	.info = at91sam7_info
};

u32 MC_FMR[4] = { 0xFFFFFF60, 0xFFFFFF70, 0xFFFFFF80, 0xFFFFFF90 };
u32 MC_FCR[4] = { 0xFFFFFF64, 0xFFFFFF74, 0xFFFFFF84, 0xFFFFFF94 };
u32 MC_FSR[4] = { 0xFFFFFF68, 0xFFFFFF78, 0xFFFFFF88, 0xFFFFFF98 };

char * EPROC[8]= {"Unknown","ARM946-E","ARM7TDMI","Unknown","ARM920T","ARM926EJ-S","Unknown","Unknown"};

long SRAMSIZ[16] = {
	-1,
	0x0400,		/*  1K */
	0x0800,		/*  2K */ 
	-1, 
	0x1c000,	/* 112K */
	0x1000,		/*   4K */
	0x14000,	/*  80K */
	0x28000,	/* 160K */
	0x2000,		/*   8K */
	0x4000,		/*  16K */
	0x8000,		/*  32K */
	0x10000,	/*  64K */
	0x20000,	/* 128K */
	0x40000,	/* 256K */
	0x18000,	/*  96K */
	0x80000,	/* 512K */
};

int at91sam7_register_commands(struct command_context_s *cmd_ctx)
{
	command_t *at91sam7_cmd = register_command(cmd_ctx, NULL, "at91sam7_new", NULL, COMMAND_ANY, NULL);

	register_command(cmd_ctx, at91sam7_cmd, "gpnvm", at91sam7_handle_gpnvm_command, COMMAND_EXEC,
					"at91sam7 gpnvm <bit> set|clear, set or clear one gpnvm bit");
	return ERROR_OK;
}

u32 at91sam7_get_flash_status(target_t *target, int bank_number)
{
	u32 fsr;
	target_read_u32(target, MC_FSR[bank_number], &fsr);

	return fsr;
}

/* Read clock configuration and set at91sam7_info->mck_freq */
void at91sam7_read_clock_info(flash_bank_t *bank)
{
	at91sam7_flash_bank_t *at91sam7_info = bank->driver_priv;
	target_t *target = bank->target;
	u32 mckr, mcfr, pllr, mor;
	unsigned long tmp = 0, mainfreq;

	/* Read Clock Generator Main Oscillator Register */
	target_read_u32(target, CKGR_MOR, &mor);
	/* Read Clock Generator Main Clock Frequency Register */
	target_read_u32(target, CKGR_MCFR, &mcfr);
	/* Read Master Clock Register*/
	target_read_u32(target, PMC_MCKR, &mckr);
	/* Read Clock Generator PLL Register  */
	target_read_u32(target, CKGR_PLLR, &pllr);
	
	at91sam7_info->mck_valid = 0;
	at91sam7_info->mck_freq = 0;
	switch (mckr & PMC_MCKR_CSS) 
	{
		case 0:			/* Slow Clock */
			at91sam7_info->mck_valid = 1;
			tmp = RC_FREQ;
			break;

		case 1:			/* Main Clock */
			if ((mcfr & CKGR_MCFR_MAINRDY) && 
				(at91sam7_info->ext_freq == 0))
			{
				at91sam7_info->mck_valid = 1;
				tmp = RC_FREQ / 16ul * (mcfr & 0xffff);
			}
			else if (at91sam7_info->ext_freq != 0)
			{
				at91sam7_info->mck_valid = 1;
				tmp = at91sam7_info->ext_freq;
			}
			break;

		case 2:			/* Reserved */
			break;

		case 3:			/* PLL Clock */
			if ((mcfr & CKGR_MCFR_MAINRDY) && 
				(at91sam7_info->ext_freq == 0)) 
			{
				target_read_u32(target, CKGR_PLLR, &pllr);
				if (!(pllr & CKGR_PLLR_DIV))
					break; /* 0 Hz */
				at91sam7_info->mck_valid = 1;
				mainfreq = RC_FREQ / 16ul * (mcfr & 0xffff);
				/* Integer arithmetic should have sufficient precision
				 * as long as PLL is properly configured. */
				tmp = mainfreq / (pllr & CKGR_PLLR_DIV)*
					(((pllr & CKGR_PLLR_MUL) >> 16) + 1);
			}
			else if ((at91sam7_info->ext_freq != 0) &&
				((pllr&CKGR_PLLR_DIV) != 0))
			{
				at91sam7_info->mck_valid = 1;
				tmp = at91sam7_info->ext_freq / (pllr&CKGR_PLLR_DIV)*
					(((pllr & CKGR_PLLR_MUL) >> 16) + 1);
			}
			break;
	}

	/* Prescaler adjust */
	if ( (((mckr & PMC_MCKR_PRES) >> 2) == 7) || (tmp == 0) )
	{
		at91sam7_info->mck_valid = 0;
		at91sam7_info->mck_freq = 0;
	}
	else if (((mckr & PMC_MCKR_PRES) >> 2) != 0)
		at91sam7_info->mck_freq = tmp >> ((mckr & PMC_MCKR_PRES) >> 2);
	else
		at91sam7_info->mck_freq = tmp;
}

/* Setup the timimg registers for nvbits or normal flash */
void at91sam7_set_flash_mode(flash_bank_t *bank, int mode)
{
	u32 fmr, fmcn = 0, fws = 0;
	at91sam7_flash_bank_t *at91sam7_info = bank->driver_priv;
	target_t *target = bank->target;

	if (mode && (mode != at91sam7_info->flashmode))
	{
		/* Always round up (ceil) */
		if (mode == FMR_TIMING_NVBITS)
		{
			if (at91sam7_info->cidr_arch == 0x60)
			{
				/* AT91SAM7A3 uses master clocks in 100 ns */
				fmcn = (at91sam7_info->mck_freq/10000000ul)+1;
			}
			else
			{
				/* master clocks in 1uS for ARCH 0x7 types */
				fmcn = (at91sam7_info->mck_freq/1000000ul)+1;
			}
		}
		else if (mode == FMR_TIMING_FLASH)
		{
			/* main clocks in 1.5uS */
			fmcn = (at91sam7_info->mck_freq/1000000ul)+
				(at91sam7_info->mck_freq/2000000ul)+1;
		}

		/* hard overclocking */
		if (fmcn > 0xFF)
			fmcn = 0xFF;

		/* Only allow fmcn=0 if clock period is > 30 us = 33kHz. */
		if (at91sam7_info->mck_freq <= 33333ul)
			fmcn = 0;
		/* Only allow fws=0 if clock frequency is < 30 MHz. */
		if (at91sam7_info->mck_freq > 30000000ul)
			fws = 1;

		LOG_DEBUG("fmcn[%i]: %i", bank->bank_number, fmcn);
		fmr = fmcn << 16 | fws << 8;
		target_write_u32(target, MC_FMR[bank->bank_number], fmr);
	}

	at91sam7_info->flashmode = mode;
}

u32 at91sam7_wait_status_busy(flash_bank_t *bank, u32 waitbits, int timeout)
{
	u32 status;

	while ((!((status = at91sam7_get_flash_status(bank->target, bank->bank_number)) & waitbits)) && (timeout-- > 0))
	{
		LOG_DEBUG("status[%i]: 0x%x", bank->bank_number, status);
		alive_sleep(1);
	}

	LOG_DEBUG("status[%i]: 0x%x", bank->bank_number, status);

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
int at91sam7_flash_command(struct flash_bank_s *bank, u8 cmd, u16 pagen)
{
	u32 fcr;
	at91sam7_flash_bank_t *at91sam7_info = bank->driver_priv;
	target_t *target = bank->target;

	fcr = (0x5A<<24) | ((pagen&0x3FF)<<8) | cmd; 
	target_write_u32(target, MC_FCR[bank->bank_number], fcr);
	LOG_DEBUG("Flash command: 0x%x, flash bank: %i, page number: %u", fcr, bank->bank_number+1, pagen);

	if ((at91sam7_info->cidr_arch == 0x60)&&((cmd==SLB)|(cmd==CLB)))
	{
		/* Lock bit manipulation on AT91SAM7A3 waits for FC_FSR bit 1, EOL */
		if (at91sam7_wait_status_busy(bank, MC_FSR_EOL, 10)&0x0C)
		{
			return ERROR_FLASH_OPERATION_FAILED;
		}
		return ERROR_OK;
	}

	if (at91sam7_wait_status_busy(bank, MC_FSR_FRDY, 10)&0x0C) 
	{
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

/* Read device id register, main clock frequency register and fill in driver info structure */
int at91sam7_read_part_info(struct flash_bank_s *bank)
{
	flash_bank_t *t_bank = bank;
	at91sam7_flash_bank_t *at91sam7_info;
	target_t *target = t_bank->target;

	u16 bnk, sec;
	u16 arch;
	u32 cidr;
	u8 banks_num;
	u16 num_nvmbits;
	u16 sectors_num;
	u16 pages_per_sector;
	u16 page_size;
	u32 ext_freq;
	u32 bank_size;
	u32 base_address = 0;
	char *target_name = "Unknown";

	at91sam7_info = t_bank->driver_priv;

	if (at91sam7_info->cidr != 0)
	{
		/* flash already configured, update clock and check for protected sectors */
		flash_bank_t *fb = bank;
		t_bank = fb;

		while (t_bank)
		{
			/* re-calculate master clock frequency */
			at91sam7_read_clock_info(t_bank);

			/* no timming */
			at91sam7_set_flash_mode(t_bank, FMR_TIMING_NONE);

			/* check protect state */
			at91sam7_protect_check(t_bank);

			t_bank = fb->next;
			fb = t_bank;
		}

		return ERROR_OK;
	}

	/* Read and parse chip identification register */
	target_read_u32(target, DBGU_CIDR, &cidr);
	if (cidr == 0)
	{
		LOG_WARNING("Cannot identify target as an AT91SAM");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	if (at91sam7_info->flash_autodetection == 0)
	{
		/* banks and sectors are already created, based on data from input file */
		flash_bank_t *fb = bank;
		t_bank = fb;
		while (t_bank)
		{
			at91sam7_info = t_bank->driver_priv;

			at91sam7_info->cidr = cidr;
			at91sam7_info->cidr_ext = (cidr>>31)&0x0001;
			at91sam7_info->cidr_nvptyp = (cidr>>28)&0x0007;
			at91sam7_info->cidr_arch = (cidr>>20)&0x00FF;
			at91sam7_info->cidr_sramsiz = (cidr>>16)&0x000F;
			at91sam7_info->cidr_nvpsiz2 = (cidr>>12)&0x000F;
			at91sam7_info->cidr_nvpsiz = (cidr>>8)&0x000F;
			at91sam7_info->cidr_eproc = (cidr>>5)&0x0007;
			at91sam7_info->cidr_version = cidr&0x001F;

			/* calculate master clock frequency */
			at91sam7_read_clock_info(t_bank);

			/* no timming */
			at91sam7_set_flash_mode(t_bank, FMR_TIMING_NONE);

			/* check protect state */
			at91sam7_protect_check(t_bank);

			t_bank = fb->next;
			fb = t_bank;
		}

		return ERROR_OK;
	}

	arch = (cidr>>20)&0x00FF;

	/* check flash size */
	switch ((cidr>>8)&0x000F)
	{
		case FLASH_SIZE_8KB:
			break;

		case FLASH_SIZE_16KB:
			banks_num = 1;
			sectors_num = 8;
			pages_per_sector = 32;
			page_size  = 64;
			base_address = 0x00100000;
			if (arch == 0x70)
			{
				num_nvmbits = 2;
				target_name = "AT91SAM7S161/16";
			}
			break;

		case FLASH_SIZE_32KB:
			banks_num = 1;
			sectors_num = 8;
			pages_per_sector = 32;
			page_size  = 128;
			base_address = 0x00100000;
			if (arch == 0x70)
			{
				num_nvmbits = 2;
				target_name = "AT91SAM7S321/32";
			}
			if (arch == 0x72)
			{
				num_nvmbits = 3;
				target_name = "AT91SAM7SE32";
			}
			break;

		case FLASH_SIZE_64KB:
			banks_num = 1;
			sectors_num = 16;
			pages_per_sector = 32;
			page_size  = 128;
			base_address = 0x00100000;
			if (arch == 0x70)
			{
				num_nvmbits = 2;
				target_name = "AT91SAM7S64";
			}
			break;

		case FLASH_SIZE_128KB:
			banks_num = 1;
			sectors_num = 8;
			pages_per_sector = 64;
			page_size  = 256;
			base_address = 0x00100000;
			if (arch == 0x70)
			{
				num_nvmbits = 2;
				target_name = "AT91SAM7S128";
			}
			if (arch == 0x71)
			{
				num_nvmbits = 3;
				target_name = "AT91SAM7XC128";
			}
			if (arch == 0x72)
			{
				num_nvmbits = 3;
				target_name = "AT91SAM7SE128";
			}
			if (arch == 0x75)
			{
				num_nvmbits = 3;
				target_name = "AT91SAM7X128";
			}
			break;

		case FLASH_SIZE_256KB:
			banks_num = 1;
			sectors_num = 16;
			pages_per_sector = 64;
			page_size  = 256;
			base_address = 0x00100000;
			if (arch == 0x60)
			{
				num_nvmbits = 3;
				target_name = "AT91SAM7A3";
			}
			if (arch == 0x70)
			{
				num_nvmbits = 2;
				target_name = "AT91SAM7S256";
			}
			if (arch == 0x71)
			{
				num_nvmbits = 3;
				target_name = "AT91SAM7XC256";
			}
			if (arch == 0x72)
			{
				num_nvmbits = 3;
				target_name = "AT91SAM7SE256";
			}
			if (arch == 0x75)
			{
				num_nvmbits = 3;
				target_name = "AT91SAM7X256";
			}
			break;

		case FLASH_SIZE_512KB:
			banks_num = 2;
			sectors_num = 16;
			pages_per_sector = 64;
			page_size  = 256;
			base_address = 0x00100000;
			if (arch == 0x70)
			{
				num_nvmbits = 2;
				target_name = "AT91SAM7S512";
			}
			if (arch == 0x71)
			{
				num_nvmbits = 3;
				target_name = "AT91SAM7XC512";
			}
			if (arch == 0x72)
			{
				num_nvmbits = 3;
				target_name = "AT91SAM7SE512";
			}
			if (arch == 0x75)
			{
				num_nvmbits = 3;
				target_name = "AT91SAM7X512";
			}
			break;

		case FLASH_SIZE_1024KB:
			break;

		case FLASH_SIZE_2048KB:
			break;
	}

	if (strcmp(target_name, "Unknown") == 0)
	{
		LOG_ERROR("Target autodetection failed! Please specify target parameters in configuration file");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	ext_freq = at91sam7_info->ext_freq;

	/* calculate bank size  */
	bank_size = sectors_num * pages_per_sector * page_size;

	for (bnk=0; bnk<banks_num; bnk++)
	{
		if (bnk > 0)
		{
			/* create a new flash bank element */
			flash_bank_t *fb = malloc(sizeof(flash_bank_t));
			fb->target = target;
			fb->driver = &at91sam7_flash;
			fb->driver_priv = malloc(sizeof(at91sam7_flash_bank_t));
			fb->next = NULL;

			/* link created bank in 'flash_banks' list and redirect t_bank */
			t_bank->next = fb;
			t_bank = fb;
		}

		t_bank->bank_number = bnk;
		t_bank->base = base_address + bnk * bank_size;
		t_bank->size = bank_size;
		t_bank->chip_width = 0;
		t_bank->bus_width = 4;
		t_bank->num_sectors = sectors_num;

		/* allocate sectors */
		t_bank->sectors = malloc(sectors_num * sizeof(flash_sector_t));
		for (sec=0; sec<sectors_num; sec++)
		{
			t_bank->sectors[sec].offset = sec * pages_per_sector * page_size;
			t_bank->sectors[sec].size = pages_per_sector * page_size;
			t_bank->sectors[sec].is_erased = -1;
			t_bank->sectors[sec].is_protected = -1;
		}

		at91sam7_info = t_bank->driver_priv;

		at91sam7_info->cidr = cidr;
		at91sam7_info->cidr_ext = (cidr>>31)&0x0001;
		at91sam7_info->cidr_nvptyp = (cidr>>28)&0x0007;
		at91sam7_info->cidr_arch = (cidr>>20)&0x00FF;
		at91sam7_info->cidr_sramsiz = (cidr>>16)&0x000F;
		at91sam7_info->cidr_nvpsiz2 = (cidr>>12)&0x000F;
		at91sam7_info->cidr_nvpsiz = (cidr>>8)&0x000F;
		at91sam7_info->cidr_eproc = (cidr>>5)&0x0007;
		at91sam7_info->cidr_version = cidr&0x001F;

		at91sam7_info->target_name  = target_name;
		at91sam7_info->flashmode = 0;
		at91sam7_info->ext_freq = ext_freq;
		at91sam7_info->num_nvmbits = num_nvmbits;
		at91sam7_info->num_nvmbits_on = 0;
		at91sam7_info->pagesize = page_size;
		at91sam7_info->pages_per_sector = pages_per_sector;

		/* calculate master clock frequency */
		at91sam7_read_clock_info(t_bank);

		/* no timming */
		at91sam7_set_flash_mode(t_bank, FMR_TIMING_NONE);

		/* check protect state */
		at91sam7_protect_check(t_bank);
	}

	LOG_DEBUG("nvptyp: 0x%3.3x, arch: 0x%4.4x", at91sam7_info->cidr_nvptyp, at91sam7_info->cidr_arch );

	return ERROR_OK;
}

int at91sam7_erase_check(struct flash_bank_s *bank)
{
	target_t *target = bank->target;
	u16 retval;
	u32 blank;
	u16 fast_check;
	u8 *buffer;
	u16 nSector;
	u16 nByte;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Configure the flash controller timing */
	at91sam7_read_clock_info(bank); 
	at91sam7_set_flash_mode(bank, FMR_TIMING_FLASH);

	fast_check = 1;
	for (nSector=0; nSector<bank->num_sectors; nSector++)
	{
		retval = target_blank_check_memory(target, bank->base+bank->sectors[nSector].offset,
			bank->sectors[nSector].size, &blank);
		if (retval != ERROR_OK)
		{
			fast_check = 0;
			break;
		}
		if (blank == 0xFF)
			bank->sectors[nSector].is_erased = 1;
		else
			bank->sectors[nSector].is_erased = 0;
	}

	if (fast_check)
	{
		return ERROR_OK;
	}

	LOG_USER("Running slow fallback erase check - add working memory");

	buffer = malloc(bank->sectors[0].size);
	for (nSector=0; nSector<bank->num_sectors; nSector++)
	{
		bank->sectors[nSector].is_erased = 1;
		retval = target->type->read_memory(target, bank->base+bank->sectors[nSector].offset, 4,
			bank->sectors[nSector].size/4, buffer);
		if (retval != ERROR_OK)
			return retval;

		for (nByte=0; nByte<bank->sectors[nSector].size; nByte++)
		{
			if (buffer[nByte] != 0xFF)
			{
				bank->sectors[nSector].is_erased = 0;
				break;
			}
		}
	}
	free(buffer);

	return ERROR_OK;
}

int at91sam7_protect_check(struct flash_bank_s *bank)
{
	u8 lock_pos, gpnvm_pos;
	u32 status;

	at91sam7_flash_bank_t *at91sam7_info = bank->driver_priv;

	if (at91sam7_info->cidr == 0)
	{
		return ERROR_FLASH_BANK_NOT_PROBED;
	}
	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	status = at91sam7_get_flash_status(bank->target, bank->bank_number);
	at91sam7_info->lockbits = (status>>16);

	at91sam7_info->num_lockbits_on = 0;
	for (lock_pos=0; lock_pos<bank->num_sectors; lock_pos++)
	{
		if ( ((status>>(16+lock_pos))&(0x0001)) == 1)
		{
			at91sam7_info->num_lockbits_on++;
			bank->sectors[lock_pos].is_protected = 1;
		}
		else
			bank->sectors[lock_pos].is_protected = 0;
	}

	/* GPNVM and SECURITY bits apply only for MC_FSR of EFC0 */
	status = at91sam7_get_flash_status(bank->target, 0);

	at91sam7_info->securitybit = (status>>4)&0x01;
	at91sam7_info->nvmbits = (status>>8)&0xFF;

	at91sam7_info->num_nvmbits_on = 0;
	for (gpnvm_pos=0; gpnvm_pos<at91sam7_info->num_nvmbits; gpnvm_pos++)
	{
		if ( ((status>>(8+gpnvm_pos))&(0x01)) == 1)
		{
			at91sam7_info->num_nvmbits_on++;
		}
	}

	return ERROR_OK;
}

/***************************************************************************************************************************************************************************************
# flash bank <driver> <base_addr> <size> <chip_width> <bus_width> <target_number> [<target_name> <banks> <sectors_per_bank> <pages_per_sector> <page_size> <num_nvmbits> <ext_freq_khz>]
#   <ext_freq_khz> - MUST be used if clock is from external source
#                    CAN be used if main oscillator frequency is known
# Examples:
#  flash bank at91sam7 0x00100000 0 0 4 0 0 AT91SAM7XC256 1 16 64 256 3 25000                   ==== RECOMENDED ============
#  flash bank at91sam7 0 0 0 0 0 0 0 0 0 0 0 0 25000    (auto-detection, except for clock)      ==== RECOMENDED ============
#  flash bank at91sam7 0x00100000 0 0 4 0 0 AT91SAM7XC256 1 16 64 256 3 0                       ==== NOT RECOMENDED !!! ====
#  flash bank at91sam7 0 0 0 0 0                        (old style, full auto-detection)        ==== NOT RECOMENDED !!! ====
****************************************************************************************************************************************************************************************/
int at91sam7_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank)
{
	flash_bank_t *t_bank = bank;
	at91sam7_flash_bank_t *at91sam7_info;
	target_t *target = t_bank->target;

	u32 base_address;
	u32 bank_size;
	u32 ext_freq;

	int chip_width;
	int bus_width;
	int banks_num;
	int num_sectors;

	u16 pages_per_sector;
	u16 page_size;
	u16 num_nvmbits;

	char *target_name;

	int bnk, sec;

	at91sam7_info = malloc(sizeof(at91sam7_flash_bank_t));
	t_bank->driver_priv = at91sam7_info;

	/* part wasn't probed for info yet */
	at91sam7_info->cidr = 0;
	at91sam7_info->flashmode = 0;
	at91sam7_info->ext_freq = 0;
	at91sam7_info->flash_autodetection = 0;

	if (argc == 14)
	{
		ext_freq = atol(args[13]) * 1000;
		at91sam7_info->ext_freq = ext_freq;
	}

	if ((argc != 14)                ||
		(atoi(args[4]) == 0)        ||  /* bus width */
		(atoi(args[8]) == 0)        ||  /* banks number */
		(atoi(args[9]) == 0)        ||  /* sectors per bank */
		(atoi(args[10]) == 0)       ||  /* pages per sector */
		(atoi(args[11]) == 0)       ||  /* page size */
		(atoi(args[12]) == 0))          /* nvmbits number */
	{
		at91sam7_info->flash_autodetection = 1;
		return ERROR_OK;
	}

	base_address = strtoul(args[1], NULL, 0);
	chip_width = atoi(args[3]);
	bus_width = atoi(args[4]);
	banks_num = atoi(args[8]);
	num_sectors = atoi(args[9]);
	pages_per_sector = atoi(args[10]);
	page_size = atoi(args[11]);
	num_nvmbits = atoi(args[12]);

	target_name = calloc(strlen(args[7])+1, sizeof(char));
	strcpy(target_name, args[7]);

	/* calculate bank size  */
	bank_size = num_sectors * pages_per_sector * page_size;

	for (bnk=0; bnk<banks_num; bnk++)
	{
		if (bnk > 0)
		{
			/* create a new bank element */
			flash_bank_t *fb = malloc(sizeof(flash_bank_t));
			fb->target = target;
			fb->driver = &at91sam7_flash;
			fb->driver_priv = malloc(sizeof(at91sam7_flash_bank_t));
			fb->next = NULL;

			/* link created bank in 'flash_banks' list and redirect t_bank */
			t_bank->next = fb;
			t_bank = fb;
		}

		t_bank->bank_number = bnk;
		t_bank->base = base_address + bnk * bank_size;
		t_bank->size = bank_size;
		t_bank->chip_width = chip_width;
		t_bank->bus_width = bus_width;
		t_bank->num_sectors = num_sectors;

		/* allocate sectors */
		t_bank->sectors = malloc(num_sectors * sizeof(flash_sector_t));
		for (sec=0; sec<num_sectors; sec++)
		{
			t_bank->sectors[sec].offset = sec * pages_per_sector * page_size;
			t_bank->sectors[sec].size = pages_per_sector * page_size;
			t_bank->sectors[sec].is_erased = -1;
			t_bank->sectors[sec].is_protected = -1;
		}

		at91sam7_info = t_bank->driver_priv;

		at91sam7_info->target_name  = target_name;
		at91sam7_info->flashmode = 0;
		at91sam7_info->ext_freq  = ext_freq;
		at91sam7_info->num_nvmbits = num_nvmbits;
		at91sam7_info->num_nvmbits_on = 0;
		at91sam7_info->pagesize = page_size;
		at91sam7_info->pages_per_sector = pages_per_sector;
	}

	return ERROR_OK;
}

int at91sam7_erase(struct flash_bank_s *bank, int first, int last)
{
	at91sam7_flash_bank_t *at91sam7_info = bank->driver_priv;
	int sec;
	u32 nbytes, pos;
	u8 *buffer;
	u8 erase_all;

	if (at91sam7_info->cidr == 0)
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
		return ERROR_FLASH_SECTOR_INVALID;
	}

	erase_all = 0;
	if ((first == 0) && (last == (bank->num_sectors-1)))
	{
		erase_all = 1;
	}

	/* Configure the flash controller timing */
	at91sam7_read_clock_info(bank);
	at91sam7_set_flash_mode(bank, FMR_TIMING_FLASH);

	if(erase_all)
	{
		if (at91sam7_flash_command(bank, EA, 0) != ERROR_OK) 
		{
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}
	else
	{
		/* allocate and clean buffer  */
		nbytes = (last - first + 1) * bank->sectors[first].size;
		buffer = malloc(nbytes * sizeof(u8));
		for (pos=0; pos<nbytes; pos++)
		{
			buffer[pos] = 0xFF;
		}

		if ( at91sam7_write(bank, buffer, bank->sectors[first].offset, nbytes) != ERROR_OK)
		{
			return ERROR_FLASH_OPERATION_FAILED;
		}

		free(buffer);
	}

	/* mark erased sectors */
	for (sec=first; sec<=last; sec++)
	{
		bank->sectors[sec].is_erased = 1;
	}

	return ERROR_OK;
}

int at91sam7_protect(struct flash_bank_s *bank, int set, int first, int last)
{
	u32 cmd;
	u32 sector, pagen;

	at91sam7_flash_bank_t *at91sam7_info = bank->driver_priv;

	if (at91sam7_info->cidr == 0)
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
		return ERROR_FLASH_SECTOR_INVALID;
	}

	/* Configure the flash controller timing */
	at91sam7_read_clock_info(bank);
	at91sam7_set_flash_mode(bank, FMR_TIMING_NVBITS);

	for (sector=first; sector<=last; sector++)
	{
		if (set)
			cmd = SLB;
		else
			cmd = CLB;

		/* if we lock a page from one sector then entire sector will be locked, also,
		 * if we unlock a page from a locked sector, entire sector will be unlocked   */
		pagen = sector * at91sam7_info->pages_per_sector;

		if (at91sam7_flash_command(bank, cmd, pagen) != ERROR_OK)
		{
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}

	at91sam7_protect_check(bank);

	return ERROR_OK;
}

int at91sam7_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count)
{
	int retval;
	at91sam7_flash_bank_t *at91sam7_info = bank->driver_priv;
	target_t *target = bank->target;
	u32 dst_min_alignment, wcount, bytes_remaining = count;
	u32 first_page, last_page, pagen, buffer_pos;

	if (at91sam7_info->cidr == 0)
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

	dst_min_alignment = at91sam7_info->pagesize;

	if (offset % dst_min_alignment)
	{
		LOG_WARNING("offset 0x%x breaks required alignment 0x%x", offset, dst_min_alignment);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	if (at91sam7_info->cidr_arch == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	first_page = offset/dst_min_alignment;
	last_page = CEIL(offset + count, dst_min_alignment);

	LOG_DEBUG("first_page: %i, last_page: %i, count %i", first_page, last_page, count);

	/* Configure the flash controller timing */
	at91sam7_read_clock_info(bank);
	at91sam7_set_flash_mode(bank, FMR_TIMING_FLASH);

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
		if((retval = target->type->write_memory(target, bank->base+pagen*dst_min_alignment, 4, wcount, buffer+buffer_pos)) != ERROR_OK)
		{
			return retval;
		}

		/* Send Write Page command to Flash Controller */
		if (at91sam7_flash_command(bank, WP, pagen) != ERROR_OK)
		{
			return ERROR_FLASH_OPERATION_FAILED;
		}
		LOG_DEBUG("Write flash bank:%i page number:%i", bank->bank_number, pagen);
	}

	return ERROR_OK;
}

int at91sam7_probe(struct flash_bank_s *bank)
{
	/* we can't probe on an at91sam7
	 * if this is an at91sam7, it has the configured flash */
	int retval;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = at91sam7_read_part_info(bank);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

int at91sam7_info(struct flash_bank_s *bank, char *buf, int buf_size)
{
	int printed;
	at91sam7_flash_bank_t *at91sam7_info = bank->driver_priv;

	if (at91sam7_info->cidr == 0)
	{
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	printed = snprintf(buf, buf_size,
		"\n at91sam7 driver information: Chip is %s\n",
		at91sam7_info->target_name);

	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size,
		" Cidr: 0x%8.8x | Arch: 0x%4.4x | Eproc: %s | Version: 0x%3.3x | Flashsize: 0x%8.8x\n",
		at91sam7_info->cidr, at91sam7_info->cidr_arch, EPROC[at91sam7_info->cidr_eproc],
		at91sam7_info->cidr_version, bank->size);

	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size,
		" Master clock (estimated): %u KHz | External clock: %u KHz\n",
		at91sam7_info->mck_freq / 1000, at91sam7_info->ext_freq / 1000);

	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size,
		" Pagesize: %i bytes | Lockbits(%i): %i 0x%4.4x | Pages in lock region: %i \n",
		at91sam7_info->pagesize, bank->num_sectors, at91sam7_info->num_lockbits_on,
		at91sam7_info->lockbits, at91sam7_info->pages_per_sector*at91sam7_info->num_lockbits_on);

	buf += printed;
	buf_size -= printed;

	printed = snprintf(buf, buf_size,
		" Securitybit: %i | Nvmbits(%i): %i 0x%1.1x\n",
		at91sam7_info->securitybit, at91sam7_info->num_nvmbits,
		at91sam7_info->num_nvmbits_on, at91sam7_info->nvmbits);

	buf += printed;
	buf_size -= printed;

	return ERROR_OK;
}

/* 
* On AT91SAM7S: When the gpnvm bits are set with 
* > at91sam7 gpnvm bitnr set
* the changes are not visible in the flash controller status register MC_FSR 
* until the processor has been reset.
* On the Olimex board this requires a power cycle.
* Note that the AT91SAM7S has the following errata (doc6175.pdf sec 14.1.3):
*   The maximum number of write/erase cycles for Non volatile Memory bits is 100. this includes
*   Lock Bits (LOCKx), General Purpose NVM bits (GPNVMx) and the Security Bit.
*/
int at91sam7_handle_gpnvm_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *bank;
	int bit;
	u8  flashcmd;
	u32 status;
	at91sam7_flash_bank_t *at91sam7_info;
	int retval;

	if (argc != 2)
	{
		command_print(cmd_ctx, "at91sam7 gpnvm <bit> <set|clear>");
		return ERROR_OK;
	}

	bank = get_flash_bank_by_num_noprobe(0);
	if (bank ==  NULL)
	{
		return ERROR_FLASH_BANK_INVALID;
	}
	if (bank->driver != &at91sam7_flash)
	{
		command_print(cmd_ctx, "not an at91sam7 flash bank '%s'", args[0]);
		return ERROR_FLASH_BANK_INVALID;
	}
	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("target has to be halted to perform flash operation");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (strcmp(args[1], "set") == 0)
	{
		flashcmd = SGPB;
	}
	else if (strcmp(args[1], "clear") == 0)
	{
		flashcmd = CGPB;
	}
	else
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	at91sam7_info = bank->driver_priv;
	if (at91sam7_info->cidr == 0)
	{
		retval = at91sam7_read_part_info(bank);
		if (retval != ERROR_OK)
		{
			return retval;
		}
	}

	bit = atoi(args[0]);
	if ((bit < 0) || (bit >= at91sam7_info->num_nvmbits))
	{
		command_print(cmd_ctx, "gpnvm bit '#%s' is out of bounds for target %s", args[0], at91sam7_info->target_name);
		return ERROR_OK;
	}

	/* Configure the flash controller timing */
	at91sam7_read_clock_info(bank);
	at91sam7_set_flash_mode(bank, FMR_TIMING_NVBITS);
	
	if (at91sam7_flash_command(bank, flashcmd, bit) != ERROR_OK)
	{
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* GPNVM and SECURITY bits apply only for MC_FSR of EFC0 */
	status = at91sam7_get_flash_status(bank->target, 0);
	LOG_DEBUG("at91sam7_handle_gpnvm_command: cmd 0x%x, value 0x%x, status 0x%x \n", flashcmd, bit, status);

	/* check protect state */
	at91sam7_protect_check(bank);
	
	return ERROR_OK;
}
