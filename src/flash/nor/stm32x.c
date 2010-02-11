/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "stm32x.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>


static int stm32x_mass_erase(struct flash_bank *bank);

/* flash bank stm32x <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(stm32x_flash_bank_command)
{
	struct stm32x_flash_bank *stm32x_info;

	if (CMD_ARGC < 6)
	{
		LOG_WARNING("incomplete flash_bank stm32x configuration");
		return ERROR_FLASH_BANK_INVALID;
	}

	stm32x_info = malloc(sizeof(struct stm32x_flash_bank));
	bank->driver_priv = stm32x_info;

	stm32x_info->write_algorithm = NULL;
	stm32x_info->probed = 0;

	return ERROR_OK;
}

static uint32_t stm32x_get_flash_status(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t status;

	target_read_u32(target, STM32_FLASH_SR, &status);

	return status;
}

static uint32_t stm32x_wait_status_busy(struct flash_bank *bank, int timeout)
{
	struct target *target = bank->target;
	uint32_t status;

	/* wait for busy to clear */
	while (((status = stm32x_get_flash_status(bank)) & FLASH_BSY) && (timeout-- > 0))
	{
		LOG_DEBUG("status: 0x%" PRIx32 "", status);
		alive_sleep(1);
	}
	/* Clear but report errors */
	if (status & (FLASH_WRPRTERR | FLASH_PGERR))
	{
		target_write_u32(target, STM32_FLASH_SR, FLASH_WRPRTERR | FLASH_PGERR);
	}
	return status;
}

static int stm32x_read_options(struct flash_bank *bank)
{
	uint32_t optiondata;
	struct stm32x_flash_bank *stm32x_info = NULL;
	struct target *target = bank->target;

	stm32x_info = bank->driver_priv;

	/* read current option bytes */
	target_read_u32(target, STM32_FLASH_OBR, &optiondata);

	stm32x_info->option_bytes.user_options = (uint16_t)0xFFF8 | ((optiondata >> 2) & 0x07);
	stm32x_info->option_bytes.RDP = (optiondata & (1 << OPT_READOUT)) ? 0xFFFF : 0x5AA5;

	if (optiondata & (1 << OPT_READOUT))
		LOG_INFO("Device Security Bit Set");

	/* each bit refers to a 4bank protection */
	target_read_u32(target, STM32_FLASH_WRPR, &optiondata);

	stm32x_info->option_bytes.protection[0] = (uint16_t)optiondata;
	stm32x_info->option_bytes.protection[1] = (uint16_t)(optiondata >> 8);
	stm32x_info->option_bytes.protection[2] = (uint16_t)(optiondata >> 16);
	stm32x_info->option_bytes.protection[3] = (uint16_t)(optiondata >> 24);

	return ERROR_OK;
}

static int stm32x_erase_options(struct flash_bank *bank)
{
	struct stm32x_flash_bank *stm32x_info = NULL;
	struct target *target = bank->target;
	uint32_t status;

	stm32x_info = bank->driver_priv;

	/* read current options */
	stm32x_read_options(bank);

	/* unlock flash registers */
	target_write_u32(target, STM32_FLASH_KEYR, KEY1);
	target_write_u32(target, STM32_FLASH_KEYR, KEY2);

	/* unlock option flash registers */
	target_write_u32(target, STM32_FLASH_OPTKEYR, KEY1);
	target_write_u32(target, STM32_FLASH_OPTKEYR, KEY2);

	/* erase option bytes */
	target_write_u32(target, STM32_FLASH_CR, FLASH_OPTER | FLASH_OPTWRE);
	target_write_u32(target, STM32_FLASH_CR, FLASH_OPTER | FLASH_STRT | FLASH_OPTWRE);

	status = stm32x_wait_status_busy(bank, 10);

	if (status & FLASH_WRPRTERR)
		return ERROR_FLASH_OPERATION_FAILED;
	if (status & FLASH_PGERR)
		return ERROR_FLASH_OPERATION_FAILED;

	/* clear readout protection and complementary option bytes
	 * this will also force a device unlock if set */
	stm32x_info->option_bytes.RDP = 0x5AA5;

	return ERROR_OK;
}

static int stm32x_write_options(struct flash_bank *bank)
{
	struct stm32x_flash_bank *stm32x_info = NULL;
	struct target *target = bank->target;
	uint32_t status;

	stm32x_info = bank->driver_priv;

	/* unlock flash registers */
	target_write_u32(target, STM32_FLASH_KEYR, KEY1);
	target_write_u32(target, STM32_FLASH_KEYR, KEY2);

	/* unlock option flash registers */
	target_write_u32(target, STM32_FLASH_OPTKEYR, KEY1);
	target_write_u32(target, STM32_FLASH_OPTKEYR, KEY2);

	/* program option bytes */
	target_write_u32(target, STM32_FLASH_CR, FLASH_OPTPG | FLASH_OPTWRE);

	/* write user option byte */
	target_write_u16(target, STM32_OB_USER, stm32x_info->option_bytes.user_options);

	status = stm32x_wait_status_busy(bank, 10);

	if (status & FLASH_WRPRTERR)
		return ERROR_FLASH_OPERATION_FAILED;
	if (status & FLASH_PGERR)
		return ERROR_FLASH_OPERATION_FAILED;

	/* write protection byte 1 */
	target_write_u16(target, STM32_OB_WRP0, stm32x_info->option_bytes.protection[0]);

	status = stm32x_wait_status_busy(bank, 10);

	if (status & FLASH_WRPRTERR)
		return ERROR_FLASH_OPERATION_FAILED;
	if (status & FLASH_PGERR)
		return ERROR_FLASH_OPERATION_FAILED;

	/* write protection byte 2 */
	target_write_u16(target, STM32_OB_WRP1, stm32x_info->option_bytes.protection[1]);

	status = stm32x_wait_status_busy(bank, 10);

	if (status & FLASH_WRPRTERR)
		return ERROR_FLASH_OPERATION_FAILED;
	if (status & FLASH_PGERR)
		return ERROR_FLASH_OPERATION_FAILED;

	/* write protection byte 3 */
	target_write_u16(target, STM32_OB_WRP2, stm32x_info->option_bytes.protection[2]);

	status = stm32x_wait_status_busy(bank, 10);

	if (status & FLASH_WRPRTERR)
		return ERROR_FLASH_OPERATION_FAILED;
	if (status & FLASH_PGERR)
		return ERROR_FLASH_OPERATION_FAILED;

	/* write protection byte 4 */
	target_write_u16(target, STM32_OB_WRP3, stm32x_info->option_bytes.protection[3]);

	status = stm32x_wait_status_busy(bank, 10);

	if (status & FLASH_WRPRTERR)
		return ERROR_FLASH_OPERATION_FAILED;
	if (status & FLASH_PGERR)
		return ERROR_FLASH_OPERATION_FAILED;

	/* write readout protection bit */
	target_write_u16(target, STM32_OB_RDP, stm32x_info->option_bytes.RDP);

	status = stm32x_wait_status_busy(bank, 10);

	if (status & FLASH_WRPRTERR)
		return ERROR_FLASH_OPERATION_FAILED;
	if (status & FLASH_PGERR)
		return ERROR_FLASH_OPERATION_FAILED;

	target_write_u32(target, STM32_FLASH_CR, FLASH_LOCK);

	return ERROR_OK;
}

static int stm32x_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stm32x_flash_bank *stm32x_info = bank->driver_priv;

	uint32_t protection;
	int i, s;
	int num_bits;
	int set;

	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* medium density - each bit refers to a 4bank protection
	 * high density - each bit refers to a 2bank protection */
	target_read_u32(target, STM32_FLASH_WRPR, &protection);

	/* medium density - each protection bit is for 4 * 1K pages
	 * high density - each protection bit is for 2 * 2K pages */
	num_bits = (bank->num_sectors / stm32x_info->ppage_size);

	if (stm32x_info->ppage_size == 2)
	{
		/* high density flash/connectivity line protection */

		set = 1;

		if (protection & (1 << 31))
			set = 0;

		/* bit 31 controls sector 62 - 255 protection for high density
		 * bit 31 controls sector 62 - 127 protection for connectivity line */
		for (s = 62; s < bank->num_sectors; s++)
		{
			bank->sectors[s].is_protected = set;
		}

		if (bank->num_sectors > 61)
			num_bits = 31;

		for (i = 0; i < num_bits; i++)
		{
			set = 1;

			if (protection & (1 << i))
				set = 0;

			for (s = 0; s < stm32x_info->ppage_size; s++)
				bank->sectors[(i * stm32x_info->ppage_size) + s].is_protected = set;
		}
	}
	else
	{
		/* low/medium density flash protection */
		for (i = 0; i < num_bits; i++)
		{
			set = 1;

			if (protection & (1 << i))
				set = 0;

			for (s = 0; s < stm32x_info->ppage_size; s++)
				bank->sectors[(i * stm32x_info->ppage_size) + s].is_protected = set;
		}
	}

	return ERROR_OK;
}

static int stm32x_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	int i;
	uint32_t status;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first == 0) && (last == (bank->num_sectors - 1)))
	{
		return stm32x_mass_erase(bank);
	}

	/* unlock flash registers */
	target_write_u32(target, STM32_FLASH_KEYR, KEY1);
	target_write_u32(target, STM32_FLASH_KEYR, KEY2);

	for (i = first; i <= last; i++)
	{
		target_write_u32(target, STM32_FLASH_CR, FLASH_PER);
		target_write_u32(target, STM32_FLASH_AR, bank->base + bank->sectors[i].offset);
		target_write_u32(target, STM32_FLASH_CR, FLASH_PER | FLASH_STRT);

		status = stm32x_wait_status_busy(bank, 10);

		if (status & FLASH_WRPRTERR)
			return ERROR_FLASH_OPERATION_FAILED;
		if (status & FLASH_PGERR)
			return ERROR_FLASH_OPERATION_FAILED;
		bank->sectors[i].is_erased = 1;
	}

	target_write_u32(target, STM32_FLASH_CR, FLASH_LOCK);

	return ERROR_OK;
}

static int stm32x_protect(struct flash_bank *bank, int set, int first, int last)
{
	struct stm32x_flash_bank *stm32x_info = NULL;
	struct target *target = bank->target;
	uint16_t prot_reg[4] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
	int i, reg, bit;
	int status;
	uint32_t protection;

	stm32x_info = bank->driver_priv;

	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((first && (first % stm32x_info->ppage_size)) || ((last + 1) && (last + 1) % stm32x_info->ppage_size))
	{
		LOG_WARNING("Error: start and end sectors must be on a %d sector boundary", stm32x_info->ppage_size);
		return ERROR_FLASH_SECTOR_INVALID;
	}

	/* medium density - each bit refers to a 4bank protection
	 * high density - each bit refers to a 2bank protection */
	target_read_u32(target, STM32_FLASH_WRPR, &protection);

	prot_reg[0] = (uint16_t)protection;
	prot_reg[1] = (uint16_t)(protection >> 8);
	prot_reg[2] = (uint16_t)(protection >> 16);
	prot_reg[3] = (uint16_t)(protection >> 24);

	if (stm32x_info->ppage_size == 2)
	{
		/* high density flash */

		/* bit 7 controls sector 62 - 255 protection */
		if (last > 61)
		{
			if (set)
				prot_reg[3] &= ~(1 << 7);
			else
				prot_reg[3] |= (1 << 7);
		}

		if (first > 61)
			first = 62;
		if (last > 61)
			last = 61;

		for (i = first; i <= last; i++)
		{
			reg = (i / stm32x_info->ppage_size) / 8;
			bit = (i / stm32x_info->ppage_size) - (reg * 8);

			if (set)
				prot_reg[reg] &= ~(1 << bit);
			else
				prot_reg[reg] |= (1 << bit);
		}
	}
	else
	{
		/* medium density flash */
		for (i = first; i <= last; i++)
		{
			reg = (i / stm32x_info->ppage_size) / 8;
			bit = (i / stm32x_info->ppage_size) - (reg * 8);

			if (set)
				prot_reg[reg] &= ~(1 << bit);
			else
				prot_reg[reg] |= (1 << bit);
		}
	}

	if ((status = stm32x_erase_options(bank)) != ERROR_OK)
		return status;

	stm32x_info->option_bytes.protection[0] = prot_reg[0];
	stm32x_info->option_bytes.protection[1] = prot_reg[1];
	stm32x_info->option_bytes.protection[2] = prot_reg[2];
	stm32x_info->option_bytes.protection[3] = prot_reg[3];

	return stm32x_write_options(bank);
}

static int stm32x_write_block(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct stm32x_flash_bank *stm32x_info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t buffer_size = 16384;
	struct working_area *source;
	uint32_t address = bank->base + offset;
	struct reg_param reg_params[4];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;

	uint8_t stm32x_flash_write_code[] = {
									/* write: */
		0xDF, 0xF8, 0x24, 0x40,		/* ldr	r4, STM32_FLASH_CR */
		0x09, 0x4D,					/* ldr	r5, STM32_FLASH_SR */
		0x4F, 0xF0, 0x01, 0x03,		/* mov	r3, #1 */
		0x23, 0x60,					/* str	r3, [r4, #0] */
		0x30, 0xF8, 0x02, 0x3B,		/* ldrh r3, [r0], #2 */
		0x21, 0xF8, 0x02, 0x3B,		/* strh r3, [r1], #2 */
									/* busy: */
		0x2B, 0x68,					/* ldr 	r3, [r5, #0] */
		0x13, 0xF0, 0x01, 0x0F,		/* tst 	r3, #0x01 */
		0xFB, 0xD0,					/* beq 	busy */
		0x13, 0xF0, 0x14, 0x0F,		/* tst	r3, #0x14 */
		0x01, 0xD1,					/* bne	exit */
		0x01, 0x3A,					/* subs	r2, r2, #1 */
		0xED, 0xD1,					/* bne	write */
									/* exit: */
		0xFE, 0xE7,					/* b exit */
		0x10, 0x20, 0x02, 0x40,		/* STM32_FLASH_CR:	.word 0x40022010 */
		0x0C, 0x20, 0x02, 0x40		/* STM32_FLASH_SR:	.word 0x4002200C */
	};

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(stm32x_flash_write_code), &stm32x_info->write_algorithm) != ERROR_OK)
	{
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	if ((retval = target_write_buffer(target, stm32x_info->write_algorithm->address, sizeof(stm32x_flash_write_code), stm32x_flash_write_code)) != ERROR_OK)
		return retval;

	/* memory buffer */
	while (target_alloc_working_area(target, buffer_size, &source) != ERROR_OK)
	{
		buffer_size /= 2;
		if (buffer_size <= 256)
		{
			/* if we already allocated the writing code, but failed to get a buffer, free the algorithm */
			if (stm32x_info->write_algorithm)
				target_free_working_area(target, stm32x_info->write_algorithm);

			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	};

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARMV7M_MODE_ANY;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_IN);

	while (count > 0)
	{
		uint32_t thisrun_count = (count > (buffer_size / 2)) ? (buffer_size / 2) : count;

		if ((retval = target_write_buffer(target, source->address, thisrun_count * 2, buffer)) != ERROR_OK)
			break;

		buf_set_u32(reg_params[0].value, 0, 32, source->address);
		buf_set_u32(reg_params[1].value, 0, 32, address);
		buf_set_u32(reg_params[2].value, 0, 32, thisrun_count);

		if ((retval = target_run_algorithm(target, 0, NULL, 4, reg_params, stm32x_info->write_algorithm->address, \
				stm32x_info->write_algorithm->address + (sizeof(stm32x_flash_write_code) - 10), 10000, &armv7m_info)) != ERROR_OK)
		{
			LOG_ERROR("error executing stm32x flash write algorithm");
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		if (buf_get_u32(reg_params[3].value, 0, 32) & FLASH_PGERR)
		{
			LOG_ERROR("flash memory not erased before writing");
			/* Clear but report errors */
			target_write_u32(target, STM32_FLASH_SR, FLASH_PGERR);
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		if (buf_get_u32(reg_params[3].value, 0, 32) & FLASH_WRPRTERR)
		{
			LOG_ERROR("flash memory write protected");
			/* Clear but report errors */
			target_write_u32(target, STM32_FLASH_SR, FLASH_WRPRTERR);
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}

		buffer += thisrun_count * 2;
		address += thisrun_count * 2;
		count -= thisrun_count;
	}

	target_free_working_area(target, source);
	target_free_working_area(target, stm32x_info->write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);

	return retval;
}

static int stm32x_write(struct flash_bank *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t words_remaining = (count / 2);
	uint32_t bytes_remaining = (count & 0x00000001);
	uint32_t address = bank->base + offset;
	uint32_t bytes_written = 0;
	uint8_t status;
	int retval;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x1)
	{
		LOG_WARNING("offset 0x%" PRIx32 " breaks required 2-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* unlock flash registers */
	target_write_u32(target, STM32_FLASH_KEYR, KEY1);
	target_write_u32(target, STM32_FLASH_KEYR, KEY2);

	/* multiple half words (2-byte) to be programmed? */
	if (words_remaining > 0)
	{
		/* try using a block write */
		if ((retval = stm32x_write_block(bank, buffer, offset, words_remaining)) != ERROR_OK)
		{
			if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			{
				/* if block write failed (no sufficient working area),
				 * we use normal (slow) single dword accesses */
				LOG_WARNING("couldn't use block writes, falling back to single memory accesses");
			}
			else if (retval == ERROR_FLASH_OPERATION_FAILED)
			{
				LOG_ERROR("flash writing failed with error code: 0x%x", retval);
				return ERROR_FLASH_OPERATION_FAILED;
			}
		}
		else
		{
			buffer += words_remaining * 2;
			address += words_remaining * 2;
			words_remaining = 0;
		}
	}

	while (words_remaining > 0)
	{
		uint16_t value;
		memcpy(&value, buffer + bytes_written, sizeof(uint16_t));

		target_write_u32(target, STM32_FLASH_CR, FLASH_PG);
		target_write_u16(target, address, value);

		status = stm32x_wait_status_busy(bank, 5);

		if (status & FLASH_WRPRTERR)
		{
			LOG_ERROR("flash memory not erased before writing");
			return ERROR_FLASH_OPERATION_FAILED;
		}
		if (status & FLASH_PGERR)
		{
			LOG_ERROR("flash memory write protected");
			return ERROR_FLASH_OPERATION_FAILED;
		}

		bytes_written += 2;
		words_remaining--;
		address += 2;
	}

	if (bytes_remaining)
	{
		uint16_t value = 0xffff;
		memcpy(&value, buffer + bytes_written, bytes_remaining);

		target_write_u32(target, STM32_FLASH_CR, FLASH_PG);
		target_write_u16(target, address, value);

		status = stm32x_wait_status_busy(bank, 5);

		if (status & FLASH_WRPRTERR)
		{
			LOG_ERROR("flash memory not erased before writing");
			return ERROR_FLASH_OPERATION_FAILED;
		}
		if (status & FLASH_PGERR)
		{
			LOG_ERROR("flash memory write protected");
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}

	target_write_u32(target, STM32_FLASH_CR, FLASH_LOCK);

	return ERROR_OK;
}

static int stm32x_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stm32x_flash_bank *stm32x_info = bank->driver_priv;
	int i;
	uint16_t num_pages;
	uint32_t device_id;
	int page_size;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	stm32x_info->probed = 0;

	/* read stm32 device id register */
	target_read_u32(target, 0xE0042000, &device_id);
	LOG_INFO("device id = 0x%08" PRIx32 "", device_id);

	/* get flash size from target */
	if (target_read_u16(target, 0x1FFFF7E0, &num_pages) != ERROR_OK)
	{
		/* failed reading flash size, default to max target family */
		num_pages = 0xffff;
	}

	if ((device_id & 0x7ff) == 0x410)
	{
		/* medium density - we have 1k pages
		 * 4 pages for a protection area */
		page_size = 1024;
		stm32x_info->ppage_size = 4;

		/* check for early silicon */
		if (num_pages == 0xffff)
		{
			/* number of sectors incorrect on revA */
			LOG_WARNING("STM32 flash size failed, probe inaccurate - assuming 128k flash");
			num_pages = 128;
		}
	}
	else if ((device_id & 0x7ff) == 0x412)
	{
		/* low density - we have 1k pages
		 * 4 pages for a protection area */
		page_size = 1024;
		stm32x_info->ppage_size = 4;

		/* check for early silicon */
		if (num_pages == 0xffff)
		{
			/* number of sectors incorrect on revA */
			LOG_WARNING("STM32 flash size failed, probe inaccurate - assuming 32k flash");
			num_pages = 32;
		}
	}
	else if ((device_id & 0x7ff) == 0x414)
	{
		/* high density - we have 2k pages
		 * 2 pages for a protection area */
		page_size = 2048;
		stm32x_info->ppage_size = 2;

		/* check for early silicon */
		if (num_pages == 0xffff)
		{
			/* number of sectors incorrect on revZ */
			LOG_WARNING("STM32 flash size failed, probe inaccurate - assuming 512k flash");
			num_pages = 512;
		}
	}
	else if ((device_id & 0x7ff) == 0x418)
	{
		/* connectivity line density - we have 2k pages
		 * 2 pages for a protection area */
		page_size = 2048;
		stm32x_info->ppage_size = 2;

		/* check for early silicon */
		if (num_pages == 0xffff)
		{
			/* number of sectors incorrect on revZ */
			LOG_WARNING("STM32 flash size failed, probe inaccurate - assuming 256k flash");
			num_pages = 256;
		}
	}
	else
	{
		LOG_WARNING("Cannot identify target as a STM32 family.");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	LOG_INFO("flash size = %dkbytes", num_pages);

	/* calculate numbers of pages */
	num_pages /= (page_size / 1024);

	bank->base = 0x08000000;
	bank->size = (num_pages * page_size);
	bank->num_sectors = num_pages;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);

	for (i = 0; i < num_pages; i++)
	{
		bank->sectors[i].offset = i * page_size;
		bank->sectors[i].size = page_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	stm32x_info->probed = 1;

	return ERROR_OK;
}

static int stm32x_auto_probe(struct flash_bank *bank)
{
	struct stm32x_flash_bank *stm32x_info = bank->driver_priv;
	if (stm32x_info->probed)
		return ERROR_OK;
	return stm32x_probe(bank);
}

#if 0
COMMAND_HANDLER(stm32x_handle_part_id_command)
{
	return ERROR_OK;
}
#endif

static int stm32x_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct target *target = bank->target;
	uint32_t device_id;
	int printed;

	/* read stm32 device id register */
	target_read_u32(target, 0xE0042000, &device_id);

	if ((device_id & 0x7ff) == 0x410)
	{
		printed = snprintf(buf, buf_size, "stm32x (Medium Density) - Rev: ");
		buf += printed;
		buf_size -= printed;

		switch (device_id >> 16)
		{
			case 0x0000:
				snprintf(buf, buf_size, "A");
				break;

			case 0x2000:
				snprintf(buf, buf_size, "B");
				break;

			case 0x2001:
				snprintf(buf, buf_size, "Z");
				break;

			case 0x2003:
				snprintf(buf, buf_size, "Y");
				break;

			default:
				snprintf(buf, buf_size, "unknown");
				break;
		}
	}
	else if ((device_id & 0x7ff) == 0x412)
	{
		printed = snprintf(buf, buf_size, "stm32x (Low Density) - Rev: ");
		buf += printed;
		buf_size -= printed;

		switch (device_id >> 16)
		{
			case 0x1000:
				snprintf(buf, buf_size, "A");
				break;

			default:
				snprintf(buf, buf_size, "unknown");
				break;
		}
	}
	else if ((device_id & 0x7ff) == 0x414)
	{
		printed = snprintf(buf, buf_size, "stm32x (High Density) - Rev: ");
		buf += printed;
		buf_size -= printed;

		switch (device_id >> 16)
		{
			case 0x1000:
				snprintf(buf, buf_size, "A");
				break;

			case 0x1001:
				snprintf(buf, buf_size, "Z");
				break;

			default:
				snprintf(buf, buf_size, "unknown");
				break;
		}
	}
	else if ((device_id & 0x7ff) == 0x418)
	{
		printed = snprintf(buf, buf_size, "stm32x (Connectivity) - Rev: ");
		buf += printed;
		buf_size -= printed;

		switch (device_id >> 16)
		{
			case 0x1000:
				snprintf(buf, buf_size, "A");
				break;

			case 0x1001:
				snprintf(buf, buf_size, "Z");
				break;

			default:
				snprintf(buf, buf_size, "unknown");
				break;
		}
	}
	else
	{
		snprintf(buf, buf_size, "Cannot identify target as a stm32x\n");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_lock_command)
{
	struct target *target = NULL;
	struct stm32x_flash_bank *stm32x_info = NULL;

	if (CMD_ARGC < 1)
	{
		command_print(CMD_CTX, "stm32x lock <bank>");
		return ERROR_OK;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	stm32x_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (stm32x_erase_options(bank) != ERROR_OK)
	{
		command_print(CMD_CTX, "stm32x failed to erase options");
		return ERROR_OK;
	}

	/* set readout protection */
	stm32x_info->option_bytes.RDP = 0;

	if (stm32x_write_options(bank) != ERROR_OK)
	{
		command_print(CMD_CTX, "stm32x failed to lock device");
		return ERROR_OK;
	}

	command_print(CMD_CTX, "stm32x locked");

	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_unlock_command)
{
	struct target *target = NULL;
	struct stm32x_flash_bank *stm32x_info = NULL;

	if (CMD_ARGC < 1)
	{
		command_print(CMD_CTX, "stm32x unlock <bank>");
		return ERROR_OK;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	stm32x_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (stm32x_erase_options(bank) != ERROR_OK)
	{
		command_print(CMD_CTX, "stm32x failed to unlock device");
		return ERROR_OK;
	}

	if (stm32x_write_options(bank) != ERROR_OK)
	{
		command_print(CMD_CTX, "stm32x failed to lock device");
		return ERROR_OK;
	}

	command_print(CMD_CTX, "stm32x unlocked.\n"
			"INFO: a reset or power cycle is required "
			"for the new settings to take effect.");

	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_options_read_command)
{
	uint32_t optionbyte;
	struct target *target = NULL;
	struct stm32x_flash_bank *stm32x_info = NULL;

	if (CMD_ARGC < 1)
	{
		command_print(CMD_CTX, "stm32x options_read <bank>");
		return ERROR_OK;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	stm32x_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	target_read_u32(target, STM32_FLASH_OBR, &optionbyte);
	command_print(CMD_CTX, "Option Byte: 0x%" PRIx32 "", optionbyte);

	if (buf_get_u32((uint8_t*)&optionbyte, OPT_ERROR, 1))
		command_print(CMD_CTX, "Option Byte Complement Error");

	if (buf_get_u32((uint8_t*)&optionbyte, OPT_READOUT, 1))
		command_print(CMD_CTX, "Readout Protection On");
	else
		command_print(CMD_CTX, "Readout Protection Off");

	if (buf_get_u32((uint8_t*)&optionbyte, OPT_RDWDGSW, 1))
		command_print(CMD_CTX, "Software Watchdog");
	else
		command_print(CMD_CTX, "Hardware Watchdog");

	if (buf_get_u32((uint8_t*)&optionbyte, OPT_RDRSTSTOP, 1))
		command_print(CMD_CTX, "Stop: No reset generated");
	else
		command_print(CMD_CTX, "Stop: Reset generated");

	if (buf_get_u32((uint8_t*)&optionbyte, OPT_RDRSTSTDBY, 1))
		command_print(CMD_CTX, "Standby: No reset generated");
	else
		command_print(CMD_CTX, "Standby: Reset generated");

	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_options_write_command)
{
	struct target *target = NULL;
	struct stm32x_flash_bank *stm32x_info = NULL;
	uint16_t optionbyte = 0xF8;

	if (CMD_ARGC < 4)
	{
		command_print(CMD_CTX, "stm32x options_write <bank> <SWWDG | HWWDG> <RSTSTNDBY | NORSTSTNDBY> <RSTSTOP | NORSTSTOP>");
		return ERROR_OK;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	stm32x_info = bank->driver_priv;

	target = bank->target;

	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* REVISIT: ignores some options which we will display...
	 * and doesn't insist on the specified syntax.
	 */

	/* OPT_RDWDGSW */
	if (strcmp(CMD_ARGV[1], "SWWDG") == 0)
	{
		optionbyte |= (1 << 0);
	}
	else	/* REVISIT must be "HWWDG" then ... */
	{
		optionbyte &= ~(1 << 0);
	}

	/* OPT_RDRSTSTDBY */
	if (strcmp(CMD_ARGV[2], "NORSTSTNDBY") == 0)
	{
		optionbyte |= (1 << 1);
	}
	else	/* REVISIT must be "RSTSTNDBY" then ... */
	{
		optionbyte &= ~(1 << 1);
	}

	/* OPT_RDRSTSTOP */
	if (strcmp(CMD_ARGV[3], "NORSTSTOP") == 0)
	{
		optionbyte |= (1 << 2);
	}
	else	/* REVISIT must be "RSTSTOP" then ... */
	{
		optionbyte &= ~(1 << 2);
	}

	if (stm32x_erase_options(bank) != ERROR_OK)
	{
		command_print(CMD_CTX, "stm32x failed to erase options");
		return ERROR_OK;
	}

	stm32x_info->option_bytes.user_options = optionbyte;

	if (stm32x_write_options(bank) != ERROR_OK)
	{
		command_print(CMD_CTX, "stm32x failed to write options");
		return ERROR_OK;
	}

	command_print(CMD_CTX, "stm32x write options complete.\n"
				"INFO: a reset or power cycle is required "
				"for the new settings to take effect.");

	return ERROR_OK;
}

static int stm32x_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t status;

	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* unlock option flash registers */
	target_write_u32(target, STM32_FLASH_KEYR, KEY1);
	target_write_u32(target, STM32_FLASH_KEYR, KEY2);

	/* mass erase flash memory */
	target_write_u32(target, STM32_FLASH_CR, FLASH_MER);
	target_write_u32(target, STM32_FLASH_CR, FLASH_MER | FLASH_STRT);

	status = stm32x_wait_status_busy(bank, 10);

	target_write_u32(target, STM32_FLASH_CR, FLASH_LOCK);

	if (status & FLASH_WRPRTERR)
	{
		LOG_ERROR("stm32x device protected");
		return ERROR_OK;
	}

	if (status & FLASH_PGERR)
	{
		LOG_ERROR("stm32x device programming failed");
		return ERROR_OK;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(stm32x_handle_mass_erase_command)
{
	int i;

	if (CMD_ARGC < 1)
	{
		command_print(CMD_CTX, "stm32x mass_erase <bank>");
		return ERROR_OK;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	if (stm32x_mass_erase(bank) == ERROR_OK)
	{
		/* set all sectors as erased */
		for (i = 0; i < bank->num_sectors; i++)
		{
			bank->sectors[i].is_erased = 1;
		}

		command_print(CMD_CTX, "stm32x mass erase complete");
	}
	else
	{
		command_print(CMD_CTX, "stm32x mass erase failed");
	}

	return ERROR_OK;
}

static const struct command_registration stm32x_exec_command_handlers[] = {
	{
		.name = "lock",
		.handler = stm32x_handle_lock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Lock entire flash device.",
	},
	{
		.name = "unlock",
		.handler = stm32x_handle_unlock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Unlock entire protected flash device.",
	},
	{
		.name = "mass_erase",
		.handler = stm32x_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire flash device.",
	},
	{
		.name = "options_read",
		.handler = stm32x_handle_options_read_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Read and display device option byte.",
	},
	{
		.name = "options_write",
		.handler = stm32x_handle_options_write_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id ('SWWDG'|'HWWDG') "
			"('RSTSTNDBY'|'NORSTSTNDBY') "
			"('RSTSTOP'|'NORSTSTOP')",
		.help = "Replace bits in device option byte.",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration stm32x_command_handlers[] = {
	{
		.name = "stm32x",
		.mode = COMMAND_ANY,
		.help = "stm32x flash command group",
		.chain = stm32x_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver stm32x_flash = {
	.name = "stm32x",
	.commands = stm32x_command_handlers,
	.flash_bank_command = stm32x_flash_bank_command,
	.erase = stm32x_erase,
	.protect = stm32x_protect,
	.write = stm32x_write,
	.probe = stm32x_probe,
	.auto_probe = stm32x_auto_probe,
	.erase_check = default_flash_mem_blank_check,
	.protect_check = stm32x_protect_check,
	.info = stm32x_info,
};
