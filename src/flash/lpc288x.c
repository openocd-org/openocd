/***************************************************************************
 *   Copyright (C) 2008 by			                                       *
 *   Karl RobinSod <karl.robinsod@gmail.com>                               *
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
* There are some things to notice
*
* You need to unprotect flash sectors each time you connect the OpenOCD
* Dumping 1MB takes about 60 Seconds
* Full erase (sectors 0-22 inclusive) takes 2-4 seconds
* Writing 1MB takes 88 seconds
*
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "replacements.h"

#include "lpc288x.h"

#include "flash.h"
#include "target.h"
#include "log.h"
#include "binarybuffer.h"
#include "types.h"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#define LOAD_TIMER_ERASE	0
#define LOAD_TIMER_WRITE	1

#define FLASH_PAGE_SIZE		512

/* LPC288X control registers */
#define DBGU_CIDR		0x8000507C
/* LPC288X flash registers */
#define F_CTRL			0x80102000	/* Flash control register R/W 0x5 */
#define F_STAT			0x80102004	/* Flash status register RO 0x45 */
#define F_PROG_TIME		0x80102008	/* Flash program time register R/W 0 */
#define F_WAIT			0x80102010	/* Flash read wait state register R/W 0xC004 */
#define F_CLK_TIME		0x8010201C	/* Flash clock divider for 66 kHz generation R/W 0 */
#define F_INTEN_CLR		0x80102FD8	/* Clear interrupt enable bits WO - */
#define F_INTEN_SET		0x80102FDC	/* Set interrupt enable bits WO - */
#define F_INT_STAT		0x80102FE0	/* Interrupt status bits RO 0 */
#define F_INTEN			0x80102FE4	/* Interrupt enable bits RO 0 */
#define F_INT_CLR		0x80102FE8	/* Clear interrupt status bits WO */
#define F_INT_SET		0x80102FEC	/* Set interrupt status bits WO - */
#define FLASH_PD		0x80005030	/* Allows turning off the Flash memory for power savings. R/W 1*/
#define FLASH_INIT		0x80005034	/* Monitors Flash readiness, such as recovery from Power Down mode. R/W -*/

/* F_CTRL bits */
#define FC_CS			0x0001
#define FC_FUNC			0x0002
#define FC_WEN			0x0004
#define FC_RD_LATCH		0x0020
#define FC_PROTECT		0x0080
#define FC_SET_DATA		0x0400
#define FC_RSSL			0x0800
#define FC_PROG_REQ		0x1000
#define FC_CLR_BUF		0x4000
#define FC_LOAD_REQ		0x8000
/* F_STAT bits */
#define FS_DONE			0x0001
#define FS_PROGGNT		0x0002
#define FS_RDY			0x0004
#define FS_ERR			0x0020
/* F_PROG_TIME */
#define FPT_TIME_MASK	0x7FFF

#define FPT_ENABLE		0x8000
/* F_WAIT */
#define FW_WAIT_STATES_MASK		0x00FF
#define FW_SET_MASK				0xC000

/* F_CLK_TIME */
#define FCT_CLK_DIV_MASK    0x0FFF

int lpc288x_register_commands(struct command_context_s *cmd_ctx);
int lpc288x_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank);
int lpc288x_erase(struct flash_bank_s *bank, int first, int last);
int lpc288x_protect(struct flash_bank_s *bank, int set, int first, int last);
int lpc288x_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count);
int lpc288x_probe(struct flash_bank_s *bank);
int lpc288x_auto_probe(struct flash_bank_s *bank);
int lpc288x_erase_check(struct flash_bank_s *bank);
int lpc288x_protect_check(struct flash_bank_s *bank);
int lpc288x_info(struct flash_bank_s *bank, char *buf, int buf_size);
void lpc288x_set_flash_mode(flash_bank_t *bank, u8 flashplane, int mode);
u32 lpc288x_wait_status_busy(flash_bank_t *bank, int timeout);
void lpc288x_load_timer(int erase, struct target_s *target);
void lpc288x_set_flash_clk(struct flash_bank_s *bank);
u32 lpc288x_system_ready(struct flash_bank_s *bank);
int lpc288x_handle_part_id_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

flash_driver_t lpc288x_flash =
{
	.name = "lpc288x",
	.register_commands = lpc288x_register_commands,
	.flash_bank_command = lpc288x_flash_bank_command,
	.erase = lpc288x_erase,
	.protect = lpc288x_protect,
	.write = lpc288x_write,
	.probe = lpc288x_probe,
	.auto_probe = lpc288x_probe,
	.erase_check = lpc288x_erase_check,
	.protect_check = lpc288x_protect_check,
	.info = lpc288x_info
};

int lpc288x_register_commands(struct command_context_s *cmd_ctx)
{
	return ERROR_OK;
}

u32 lpc288x_wait_status_busy(flash_bank_t *bank, int timeout)
{
	u32 status;
	target_t *target = bank->target;
	do
	{
		alive_sleep(1);
		timeout--;
		target_read_u32(target, F_STAT, &status);
	}while (((status & FS_DONE) == 0) && timeout);
	
	if(timeout == 0)
	{
		LOG_DEBUG("Timedout!");
		return ERROR_FLASH_OPERATION_FAILED;
	}
	return ERROR_OK;
}

/* Read device id register and fill in driver info structure */
int lpc288x_read_part_info(struct flash_bank_s *bank)
{
	lpc288x_flash_bank_t *lpc288x_info = bank->driver_priv;
	target_t *target = bank->target;
	u32 cidr;
	
	int i = 0;
	u32 offset;
	
	if (lpc288x_info->cidr == 0x0102100A)
		return ERROR_OK; /* already probed, multiple probes may cause memory leak, not allowed */
		
	/* Read and parse chip identification register */
	target_read_u32(target, DBGU_CIDR, &cidr);
	
	if (cidr != 0x0102100A)
	{
		LOG_WARNING("Cannot identify target as an LPC288X (%08X)",cidr);
		return ERROR_FLASH_OPERATION_FAILED;
	}
	
	lpc288x_info->cidr = cidr;
	lpc288x_info->sector_size_break = 0x000F0000;
	lpc288x_info->target_name = "LPC288x";
	
	/* setup the sector info... */
	offset = bank->base;
	bank->num_sectors = 23;
	bank->sectors = malloc(sizeof(flash_sector_t) * 23);
	
	for (i = 0; i < 15; i++)
	{
		bank->sectors[i].offset = offset;
		bank->sectors[i].size = 64 * 1024;
		offset += bank->sectors[i].size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}
	for (i = 15; i < 23; i++)
	{
		bank->sectors[i].offset = offset;
		bank->sectors[i].size = 8 * 1024;
		offset += bank->sectors[i].size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}
	
	return ERROR_OK;
}

int lpc288x_protect_check(struct flash_bank_s *bank)
{
	return ERROR_OK;
}

/* flash_bank LPC288x 0 0 0 0 <target#> <cclk> */
int lpc288x_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank)
{
	lpc288x_flash_bank_t *lpc288x_info;
	
	if (argc < 6)
	{
		LOG_WARNING("incomplete flash_bank LPC288x configuration");
		return ERROR_FLASH_BANK_INVALID;
	}
	
	lpc288x_info = malloc(sizeof(lpc288x_flash_bank_t));
	bank->driver_priv = lpc288x_info;
	
	/* part wasn't probed for info yet */
	lpc288x_info->cidr = 0;
	lpc288x_info->cclk = strtoul(args[6], NULL, 0);
	
	return ERROR_OK;
}

/* The frequency is the AHB clock frequency divided by (CLK_DIV ×3) + 1.
 * This must be programmed such that the Flash Programming clock frequency is 66 kHz ± 20%.
 * AHB = 12 MHz ?
 * 12000000/66000 = 182
 * CLK_DIV = 60 ? */
void lpc288x_set_flash_clk(struct flash_bank_s *bank)
{
	u32 clk_time;
	lpc288x_flash_bank_t *lpc288x_info = bank->driver_priv;
	clk_time = (lpc288x_info->cclk / 66000) / 3;
	target_write_u32(bank->target, F_CTRL, FC_CS | FC_WEN);
	target_write_u32(bank->target, F_CLK_TIME, clk_time);
}

/* AHB tcyc (in ns) 83 ns
 * LOAD_TIMER_ERASE		FPT_TIME	= ((400,000,000 / AHB tcyc (in ns)) - 2) / 512
 * 									= 9412 (9500) (AN10548 9375)
 * LOAD_TIMER_WRITE		FPT_TIME	= ((1,000,000 / AHB tcyc (in ns)) - 2) / 512
 * 									= 23 (75) (AN10548 72 - is this wrong?)
 * TODO: Sort out timing calcs ;) */
void lpc288x_load_timer(int erase, struct target_s *target)
{
	if (erase == LOAD_TIMER_ERASE)
	{
		target_write_u32(target, F_PROG_TIME, FPT_ENABLE | 9500);
	}
	else
	{
		target_write_u32(target, F_PROG_TIME, FPT_ENABLE | 75);
	}
}

u32 lpc288x_system_ready(struct flash_bank_s *bank)
{
	lpc288x_flash_bank_t *lpc288x_info = bank->driver_priv;
	if (lpc288x_info->cidr == 0)
	{
		return ERROR_FLASH_BANK_NOT_PROBED;
	}
	
	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	return ERROR_OK;
}

int lpc288x_erase_check(struct flash_bank_s *bank)
{
	u32 status = lpc288x_system_ready(bank);	/* probed? halted? */
	if (status != ERROR_OK)
	{
		LOG_INFO("Processor not halted/not probed");
		return status;
	}
	
	return ERROR_OK;
}

int lpc288x_erase(struct flash_bank_s *bank, int first, int last)
{
	u32 status;
	int sector;
	target_t *target = bank->target;
	
	status = lpc288x_system_ready(bank);    /* probed? halted? */
	if (status != ERROR_OK)
	{
		return status;
	}
	
	if ((first < 0) || (last < first) || (last >= bank->num_sectors))
	{
		LOG_INFO("Bad sector range");
		return ERROR_FLASH_SECTOR_INVALID;
	}
	
	/* Configure the flash controller timing */
	lpc288x_set_flash_clk(bank);
	
	for (sector = first; sector <= last; sector++)
	{
		if (lpc288x_wait_status_busy(bank, 1000) != ERROR_OK)
		{
			return ERROR_FLASH_OPERATION_FAILED;
		}
		
		lpc288x_load_timer(LOAD_TIMER_ERASE,target);
		
		target_write_u32(target, bank->sectors[sector].offset, 0x00);
		
		target_write_u32(target, F_CTRL, FC_PROG_REQ | FC_PROTECT | FC_CS);
	}
	if (lpc288x_wait_status_busy(bank, 1000) != ERROR_OK)
	{
		return ERROR_FLASH_OPERATION_FAILED;
	}
	return ERROR_OK;
}

int lpc288x_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count)
{
	u8 page_buffer[FLASH_PAGE_SIZE];
	u32 i, status, source_offset,dest_offset;
	target_t *target = bank->target;
	u32 bytes_remaining = count;
	u32 first_sector, last_sector, sector, page;
	
	/* probed? halted? */
	status = lpc288x_system_ready(bank);
	if (status != ERROR_OK)
	{
		return status;
	}
	
	/* Initialise search indices */
	first_sector = last_sector = 0xffffffff;
	
	/* validate the write range... */
	for (i = 0; i < bank->num_sectors; i++)
	{
		if ((offset >= bank->sectors[i].offset) &&
			(offset < (bank->sectors[i].offset + bank->sectors[i].size)) &&
			(first_sector == 0xffffffff))
		{
			first_sector = i;
			/* all writes must start on a sector boundary... */
			if (offset % bank->sectors[i].size)
			{
				LOG_INFO("offset 0x%x breaks required alignment 0x%x", offset, bank->sectors[i].size);
				return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
			}
		}
		if (((offset + count) > bank->sectors[i].offset) &&
			((offset + count) <= (bank->sectors[i].offset + bank->sectors[i].size)) &&
			(last_sector == 0xffffffff))
		{
			last_sector = i;
		}
	}
	
	/* Range check... */
	if (first_sector == 0xffffffff || last_sector == 0xffffffff)
	{
		LOG_INFO("Range check failed %x %x", offset, count);
		return ERROR_FLASH_DST_OUT_OF_BANK;
	}
	
	/* Configure the flash controller timing */
	lpc288x_set_flash_clk(bank);
	
	/* initialise the offsets */
	source_offset = 0;
	dest_offset = 0;
	
	for (sector = first_sector; sector <= last_sector; sector++)
	{
		for (page = 0; page < bank->sectors[sector].size / FLASH_PAGE_SIZE; page++)
		{
			if (bytes_remaining == 0)
			{
				count = 0;
				memset(page_buffer, 0xFF, FLASH_PAGE_SIZE);
			}
			else if (bytes_remaining < FLASH_PAGE_SIZE)
			{
				count = bytes_remaining;
				memset(page_buffer, 0xFF, FLASH_PAGE_SIZE);
				memcpy(page_buffer, &buffer[source_offset], count);
			}
			else
			{
				count = FLASH_PAGE_SIZE;
				memcpy(page_buffer, &buffer[source_offset], count);
			}
			
			/* Wait for flash to become ready */
			if (lpc288x_wait_status_busy(bank, 1000) != ERROR_OK)
			{
				return ERROR_FLASH_OPERATION_FAILED;
			}
			
			/* fill flash data latches with 1's */
			target_write_u32(target, F_CTRL, FC_CS | FC_SET_DATA | FC_WEN | FC_FUNC);
			
			target_write_u32(target, F_CTRL, FC_CS | FC_WEN | FC_FUNC);
			/*would be better to use the clean target_write_buffer() interface but
			 * it seems not to be a LOT slower....
			 * bulk_write_memory() is no quicker :(*/
#if 1
			if (target->type->write_memory(target, offset + dest_offset, 4, 128, page_buffer) != ERROR_OK)
			{
				LOG_ERROR("Write failed s %x p %x", sector, page);
				return ERROR_FLASH_OPERATION_FAILED;
			}
#else
			if (target_write_buffer(target, offset + dest_offset, FLASH_PAGE_SIZE, page_buffer) != ERROR_OK)
			{
				LOG_INFO("Write to flash buffer failed");
				return ERROR_FLASH_OPERATION_FAILED;
			}
#endif
			dest_offset += FLASH_PAGE_SIZE;
			source_offset += count;
			bytes_remaining -= count;
			
			lpc288x_load_timer(LOAD_TIMER_WRITE, target);
			
			target_write_u32(target, F_CTRL, FC_PROG_REQ | FC_PROTECT | FC_FUNC | FC_CS);
		}
	}
	
	return ERROR_OK;
}

int lpc288x_probe(struct flash_bank_s *bank)
{
	/* we only deal with LPC2888 so flash config is fixed */
	lpc288x_flash_bank_t *lpc288x_info = bank->driver_priv;
	int retval;
	
	if (lpc288x_info->cidr != 0)
	{
		return ERROR_OK; /* already probed */
	}
	
	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	retval = lpc288x_read_part_info(bank);
	if (retval != ERROR_OK)
		return retval;
	return ERROR_OK;
}

int lpc288x_info(struct flash_bank_s *bank, char *buf, int buf_size)
{
	snprintf(buf, buf_size, "lpc288x flash driver");
	return ERROR_OK;
}

int lpc288x_protect(struct flash_bank_s *bank, int set, int first, int last)
{
	int lockregion, status;
	u32 value;
	target_t *target = bank->target;
	
	/* probed? halted? */
	status = lpc288x_system_ready(bank);   
	if (status != ERROR_OK)
	{
		return status;
	}
	
	if ((first < 0) || (last < first) || (last >= bank->num_sectors))
	{
		return ERROR_FLASH_SECTOR_INVALID;
	}
	
	/* Configure the flash controller timing */
	lpc288x_set_flash_clk(bank);   
	
	for (lockregion = first; lockregion <= last; lockregion++)
	{
		if (set)
		{
			/* write an odd value to base addy to protect... */
			value = 0x01;
		}
		else
		{
			/* write an even value to base addy to unprotect... */
			value = 0x00;
		}
		target_write_u32(target, bank->sectors[lockregion].offset, value);
		target_write_u32(target, F_CTRL, FC_LOAD_REQ | FC_PROTECT | FC_WEN | FC_FUNC | FC_CS);
	}
	
	return ERROR_OK;
}
