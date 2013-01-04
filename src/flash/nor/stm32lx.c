/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 by Clement Burin des Roziers                       *
 *   clement.burin-des-roziers@hikob.com                                   *
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
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <target/cortex_m.h>

/* stm32lx flash register locations */

#define FLASH_BASE		0x40023C00
#define FLASH_ACR		0x40023C00
#define FLASH_PECR		0x40023C04
#define FLASH_PDKEYR	0x40023C08
#define FLASH_PEKEYR	0x40023C0C
#define FLASH_PRGKEYR	0x40023C10
#define FLASH_OPTKEYR	0x40023C14
#define FLASH_SR		0x40023C18
#define FLASH_OBR		0x40023C1C
#define FLASH_WRPR		0x40023C20

/* FLASH_ACR bites */
#define FLASH_ACR__LATENCY		(1<<0)
#define FLASH_ACR__PRFTEN		(1<<1)
#define FLASH_ACR__ACC64		(1<<2)
#define FLASH_ACR__SLEEP_PD		(1<<3)
#define FLASH_ACR__RUN_PD		(1<<4)

/* FLASH_PECR bits */
#define FLASH_PECR__PELOCK		(1<<0)
#define FLASH_PECR__PRGLOCK		(1<<1)
#define FLASH_PECR__OPTLOCK		(1<<2)
#define FLASH_PECR__PROG		(1<<3)
#define FLASH_PECR__DATA		(1<<4)
#define FLASH_PECR__FTDW		(1<<8)
#define FLASH_PECR__ERASE		(1<<9)
#define FLASH_PECR__FPRG		(1<<10)
#define FLASH_PECR__EOPIE		(1<<16)
#define FLASH_PECR__ERRIE		(1<<17)
#define FLASH_PECR__OBL_LAUNCH	(1<<18)

/* FLASH_SR bits */
#define FLASH_SR__BSY		(1<<0)
#define FLASH_SR__EOP		(1<<1)
#define FLASH_SR__ENDHV		(1<<2)
#define FLASH_SR__READY		(1<<3)
#define FLASH_SR__WRPERR	(1<<8)
#define FLASH_SR__PGAERR	(1<<9)
#define FLASH_SR__SIZERR	(1<<10)
#define FLASH_SR__OPTVERR	(1<<11)

/* Unlock keys */
#define PEKEY1			0x89ABCDEF
#define PEKEY2			0x02030405
#define PRGKEY1			0x8C9DAEBF
#define PRGKEY2			0x13141516
#define OPTKEY1			0xFBEAD9C8
#define OPTKEY2			0x24252627

/* other registers */
#define DBGMCU_IDCODE	0xE0042000
#define F_SIZE			0x1FF8004C

/* Constants */
#define FLASH_PAGE_SIZE 256
#define FLASH_SECTOR_SIZE 4096
#define FLASH_PAGES_PER_SECTOR 16
#define FLASH_BANK0_ADDRESS 0x08000000

/* stm32lx option byte register location */
#define OB_RDP			0x1FF80000
#define OB_USER			0x1FF80004
#define OB_WRP0_1		0x1FF80008
#define OB_WRP2_3		0x1FF8000C

/* OB_RDP values */
#define OB_RDP__LEVEL0	0xFF5500AA
#define OB_RDP__LEVEL1	0xFFFF0000

/* stm32lx RCC register locations */
#define RCC_CR		0x40023800
#define RCC_ICSCR	0x40023804
#define RCC_CFGR	0x40023808

/* RCC_ICSCR bits */
#define RCC_ICSCR__MSIRANGE_MASK	(7<<13)

static int stm32lx_unlock_program_memory(struct flash_bank *bank);
static int stm32lx_lock_program_memory(struct flash_bank *bank);
static int stm32lx_enable_write_half_page(struct flash_bank *bank);
static int stm32lx_erase_sector(struct flash_bank *bank, int sector);
static int stm32lx_wait_until_bsy_clear(struct flash_bank *bank);

struct stm32lx_flash_bank {
	int probed;
};

/* flash bank stm32lx <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(stm32lx_flash_bank_command)
{
	struct stm32lx_flash_bank *stm32lx_info;
	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* Create the bank structure */
	stm32lx_info = malloc(sizeof(struct stm32lx_flash_bank));

	/* Check allocation */
	if (stm32lx_info == NULL) {
		LOG_ERROR("failed to allocate bank structure");
		return ERROR_FAIL;
	}

	bank->driver_priv = stm32lx_info;

	stm32lx_info->probed = 0;

	return ERROR_OK;
}

static int stm32lx_protect_check(struct flash_bank *bank)
{
	int retval;
	struct target *target = bank->target;

	uint32_t wrpr;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/*
	 * Read the WRPR word, and check each bit (corresponding to each
	 * flash sector
	 */
	retval = target_read_u32(target, FLASH_WRPR, &wrpr);
	if (retval != ERROR_OK)
		return retval;

	for (int i = 0; i < 32; i++) {
		if (wrpr & (1 << i))
			bank->sectors[i].is_protected = 1;
		else
			bank->sectors[i].is_protected = 0;
	}
	return ERROR_OK;
}

static int stm32lx_erase(struct flash_bank *bank, int first, int last)
{
	int retval;

	/*
	 * It could be possible to do a mass erase if all sectors must be
	 * erased, but it is not implemented yet.
	 */

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/*
	 * Loop over the selected sectors and erase them
	 */
	for (int i = first; i <= last; i++) {
		retval = stm32lx_erase_sector(bank, i);
		if (retval != ERROR_OK)
			return retval;
		bank->sectors[i].is_erased = 1;
	}
	return ERROR_OK;
}

static int stm32lx_protect(struct flash_bank *bank, int set, int first,
		int last)
{
	LOG_WARNING("protection of the STM32L flash is not implemented");
	return ERROR_OK;
}

static int stm32lx_write_half_pages(struct flash_bank *bank, uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t buffer_size = 16384;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = bank->base + offset;

	struct reg_param reg_params[3];
	struct armv7m_algorithm armv7m_info;

	int retval = ERROR_OK;

	/* see contib/loaders/flash/stm32lx.S for src */

	static const uint8_t stm32lx_flash_write_code[] = {
		/* write_word: */
		0x00, 0x23,             /* movs r3, #0 */
		0x04, 0xe0,             /* b test_done */

		/* write_word: */
		0x51, 0xf8, 0x04, 0xcb, /* ldr ip, [r1], #4 */
		0x40, 0xf8, 0x04, 0xcb, /* str ip, [r0], #4 */
		0x01, 0x33,             /* adds r3, #1 */

		/* test_done: */
		0x93, 0x42,             /* cmp r3, r2 */
		0xf8, 0xd3,             /* bcc write_word */
		0x00, 0xbe,             /* bkpt 0 */
	};

	/* Check if there is an even number of half pages (128bytes) */
	if (count % 128) {
		LOG_ERROR("there should be an even number "
				"of half pages = 128 bytes (count = %" PRIi32 " bytes)", count);
		return ERROR_FAIL;
	}

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(stm32lx_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_DEBUG("no working area for block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};

	/* Write the flashing code */
	retval = target_write_buffer(target,
			write_algorithm->address,
			sizeof(stm32lx_flash_write_code),
			(uint8_t *)stm32lx_flash_write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return retval;
	}

	/* Allocate half pages memory */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		if (buffer_size > 1024)
			buffer_size -= 1024;
		else
			buffer_size /= 2;

		if (buffer_size <= 256) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARMV7M_MODE_ANY;
	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);

	/* Enable half-page write */
	retval = stm32lx_enable_write_half_page(bank);
	if (retval != ERROR_OK) {
		target_free_working_area(target, source);
		target_free_working_area(target, write_algorithm);

		destroy_reg_param(&reg_params[0]);
		destroy_reg_param(&reg_params[1]);
		destroy_reg_param(&reg_params[2]);
		return retval;
	}

	struct armv7m_common *armv7m = target_to_armv7m(target);
	if (armv7m == NULL) {

		/* something is very wrong if armv7m is NULL */
		LOG_ERROR("unable to get armv7m target");
		return retval;
	}

	/* save any DEMCR flags and configure target to catch any Hard Faults */
	uint32_t demcr_save = armv7m->demcr;
	armv7m->demcr = VC_HARDERR;

	/* Loop while there are bytes to write */
	while (count > 0) {
		uint32_t this_count;
		this_count = (count > buffer_size) ? buffer_size : count;

		/* Write the next half pages */
		retval = target_write_buffer(target, source->address, this_count, buffer);
		if (retval != ERROR_OK)
			break;

		/* 4: Store useful information in the registers */
		/* the destination address of the copy (R0) */
		buf_set_u32(reg_params[0].value, 0, 32, address);
		/* The source address of the copy (R1) */
		buf_set_u32(reg_params[1].value, 0, 32, source->address);
		/* The length of the copy (R2) */
		buf_set_u32(reg_params[2].value, 0, 32, this_count / 4);

		/* 5: Execute the bunch of code */
		retval = target_run_algorithm(target, 0, NULL, sizeof(reg_params)
				/ sizeof(*reg_params), reg_params,
				write_algorithm->address, 0, 10000, &armv7m_info);
		if (retval != ERROR_OK)
			break;

		/* check for Hard Fault */
		if (armv7m->exception_number == 3)
			break;

		/* 6: Wait while busy */
		retval = stm32lx_wait_until_bsy_clear(bank);
		if (retval != ERROR_OK)
			break;

		buffer += this_count;
		address += this_count;
		count -= this_count;
	}

	/* restore previous flags */
	armv7m->demcr = demcr_save;

	if (armv7m->exception_number == 3) {

		/* the stm32l15x devices seem to have an issue when blank.
		 * if a ram loader is executed on a blank device it will
		 * Hard Fault, this issue does not happen for a already programmed device.
		 * A related issue is described in the stm32l151xx errata (Doc ID 17721 Rev 6 - 2.1.3).
		 * The workaround of handling the Hard Fault exception does work, but makes the
		 * loader more complicated, as a compromise we manually write the pages, programming time
		 * is reduced by 50% using this slower method.
		 */

		LOG_WARNING("couldn't use loader, falling back to page memory writes");

		while (count > 0) {
			uint32_t this_count;
			this_count = (count > 128) ? 128 : count;

			/* Write the next half pages */
			retval = target_write_buffer(target, address, this_count, buffer);
			if (retval != ERROR_OK)
				break;

			/* Wait while busy */
			retval = stm32lx_wait_until_bsy_clear(bank);
			if (retval != ERROR_OK)
				break;

			buffer += this_count;
			address += this_count;
			count -= this_count;
		}
	}

	if (retval == ERROR_OK)
		retval = stm32lx_lock_program_memory(bank);

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);

	return retval;
}

static int stm32lx_write(struct flash_bank *bank, uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;

	uint32_t halfpages_number;
	uint32_t bytes_remaining = 0;
	uint32_t address = bank->base + offset;
	uint32_t bytes_written = 0;
	int retval, retval2;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x3) {
		LOG_ERROR("offset 0x%" PRIx32 " breaks required 4-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	retval = stm32lx_unlock_program_memory(bank);
	if (retval != ERROR_OK)
		return retval;

	/* first we need to write any unaligned head bytes upto
	 * the next 128 byte page */

	if (offset % 128)
		bytes_remaining = MIN(count, 128 - (offset % 128));

	while (bytes_remaining > 0) {
		uint8_t value[4] = {0xff, 0xff, 0xff, 0xff};

		/* copy remaining bytes into the write buffer */
		uint32_t bytes_to_write = MIN(4, bytes_remaining);
		memcpy(value, buffer + bytes_written, bytes_to_write);

		retval = target_write_buffer(target, address, 4, value);
		if (retval != ERROR_OK)
			goto reset_pg_and_lock;

		bytes_written += bytes_to_write;
		bytes_remaining -= bytes_to_write;
		address += 4;

		retval = stm32lx_wait_until_bsy_clear(bank);
		if (retval != ERROR_OK)
			goto reset_pg_and_lock;
	}

	offset += bytes_written;
	count -= bytes_written;

	/* this should always pass this check here */
	assert((offset % 128) == 0);

	/* calculate half pages */
	halfpages_number = count / 128;

	if (halfpages_number) {
		retval = stm32lx_write_half_pages(bank, buffer + bytes_written, offset, 128 * halfpages_number);
		if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
			/* attempt slow memory writes */
			LOG_WARNING("couldn't use block writes, falling back to single memory accesses");
			halfpages_number = 0;
		} else {
			if (retval != ERROR_OK)
				return ERROR_FAIL;
		}
	}

	/* write any remaining bytes */
	uint32_t page_bytes_written = 128 * halfpages_number;
	bytes_written += page_bytes_written;
	address += page_bytes_written;
	bytes_remaining = count - page_bytes_written;

	retval = stm32lx_unlock_program_memory(bank);
	if (retval != ERROR_OK)
		return retval;

	while (bytes_remaining > 0) {
		uint8_t value[4] = {0xff, 0xff, 0xff, 0xff};

		/* copy remaining bytes into the write buffer */
		uint32_t bytes_to_write = MIN(4, bytes_remaining);
		memcpy(value, buffer + bytes_written, bytes_to_write);

		retval = target_write_buffer(target, address, 4, value);
		if (retval != ERROR_OK)
			goto reset_pg_and_lock;

		bytes_written += bytes_to_write;
		bytes_remaining -= bytes_to_write;
		address += 4;

		retval = stm32lx_wait_until_bsy_clear(bank);
		if (retval != ERROR_OK)
			goto reset_pg_and_lock;
	}

reset_pg_and_lock:
	retval2 = stm32lx_lock_program_memory(bank);
	if (retval == ERROR_OK)
		retval = retval2;

	return retval;
}

static int stm32lx_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct stm32lx_flash_bank *stm32lx_info = bank->driver_priv;
	int i;
	uint16_t flash_size_in_kb;
	uint16_t max_flash_size_in_kb;
	uint32_t device_id;

	stm32lx_info->probed = 0;

	/* read stm32 device id register */
	int retval = target_read_u32(target, DBGMCU_IDCODE, &device_id);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("device id = 0x%08" PRIx32 "", device_id);

	/* set max flash size depending on family */
	switch (device_id & 0xfff) {
	case 0x416:
		max_flash_size_in_kb = 128;
		break;
	case 0x436:
		max_flash_size_in_kb = 384;
		break;
	default:
		LOG_WARNING("Cannot identify target as a STM32L family.");
		return ERROR_FAIL;
	}

	/* get flash size from target. */
	retval = target_read_u16(target, F_SIZE, &flash_size_in_kb);

	/* failed reading flash size or flash size invalid (early silicon),
	 * default to max target family */
	if (retval != ERROR_OK || flash_size_in_kb == 0xffff || flash_size_in_kb == 0) {
		LOG_WARNING("STM32 flash size failed, probe inaccurate - assuming %dk flash",
			max_flash_size_in_kb);
		flash_size_in_kb = max_flash_size_in_kb;
	}

	/* STM32L - we have 32 sectors, 16 pages per sector -> 512 pages
	 * 16 pages for a protection area */

	/* calculate numbers of sectors (4kB per sector) */
	int num_sectors = (flash_size_in_kb * 1024) / FLASH_SECTOR_SIZE;
	LOG_INFO("flash size = %dkbytes", flash_size_in_kb);

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	bank->base = FLASH_BANK0_ADDRESS;
	bank->size = flash_size_in_kb * 1024;
	bank->num_sectors = num_sectors;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_sectors);
	if (bank->sectors == NULL) {
		LOG_ERROR("failed to allocate bank sectors");
		return ERROR_FAIL;
	}

	for (i = 0; i < num_sectors; i++) {
		bank->sectors[i].offset = i * FLASH_SECTOR_SIZE;
		bank->sectors[i].size = FLASH_SECTOR_SIZE;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	stm32lx_info->probed = 1;

	return ERROR_OK;
}

static int stm32lx_auto_probe(struct flash_bank *bank)
{
	struct stm32lx_flash_bank *stm32lx_info = bank->driver_priv;

	if (stm32lx_info->probed)
		return ERROR_OK;

	return stm32lx_probe(bank);
}

static int stm32lx_erase_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	const int buffer_size = 4096;
	int i;
	uint32_t nBytes;
	int retval = ERROR_OK;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	uint8_t *buffer = malloc(buffer_size);
	if (buffer == NULL) {
		LOG_ERROR("failed to allocate read buffer");
		return ERROR_FAIL;
	}

	for (i = 0; i < bank->num_sectors; i++) {
		uint32_t j;
		bank->sectors[i].is_erased = 1;

		/* Loop chunk by chunk over the sector */
		for (j = 0; j < bank->sectors[i].size; j += buffer_size) {
			uint32_t chunk;
			chunk = buffer_size;
			if (chunk > (j - bank->sectors[i].size))
				chunk = (j - bank->sectors[i].size);

			retval = target_read_memory(target, bank->base
					+ bank->sectors[i].offset + j, 4, chunk / 4, buffer);
			if (retval != ERROR_OK)
				break;

			for (nBytes = 0; nBytes < chunk; nBytes++) {
				if (buffer[nBytes] != 0x00) {
					bank->sectors[i].is_erased = 0;
					break;
				}
			}
		}
		if (retval != ERROR_OK)
			break;
	}
	free(buffer);

	return retval;
}

static int stm32lx_get_info(struct flash_bank *bank, char *buf, int buf_size)
{
	/* This method must return a string displaying information about the bank */

	struct target *target = bank->target;
	uint32_t device_id;
	int printed;

	/* read stm32 device id register */
	int retval = target_read_u32(target, DBGMCU_IDCODE, &device_id);
	if (retval != ERROR_OK)
		return retval;

	if ((device_id & 0xfff) == 0x416) {
		printed = snprintf(buf, buf_size, "stm32lx - Rev: ");
		buf += printed;
		buf_size -= printed;

		switch (device_id >> 16) {
			case 0x1000:
				snprintf(buf, buf_size, "A");
				break;

			case 0x1008:
				snprintf(buf, buf_size, "Y");
				break;

			case 0x1018:
				snprintf(buf, buf_size, "X");
				break;

			case 0x1038:
				snprintf(buf, buf_size, "W");
				break;

			case 0x1078:
				snprintf(buf, buf_size, "V");
				break;

			default:
				snprintf(buf, buf_size, "unknown");
				break;
		}
	} else if ((device_id & 0xfff) == 0x436) {
		printed = snprintf(buf, buf_size, "stm32lx (HD) - Rev: ");
		buf += printed;
		buf_size -= printed;

		switch (device_id >> 16) {
			case 0x1000:
				snprintf(buf, buf_size, "A");
				break;

			case 0x1008:
				snprintf(buf, buf_size, "Z");
				break;

			case 0x1018:
				snprintf(buf, buf_size, "Y");
				break;

			default:
				snprintf(buf, buf_size, "unknown");
				break;
		}
	} else {
		snprintf(buf, buf_size, "Cannot identify target as a stm32lx");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static const struct command_registration stm32lx_exec_command_handlers[] = {
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration stm32lx_command_handlers[] = {
	{
		.name = "stm32lx",
		.mode = COMMAND_ANY,
		.help = "stm32lx flash command group",
		.usage = "",
		.chain = stm32lx_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver stm32lx_flash = {
		.name = "stm32lx",
		.commands = stm32lx_command_handlers,
		.flash_bank_command = stm32lx_flash_bank_command,
		.erase = stm32lx_erase,
		.protect = stm32lx_protect,
		.write = stm32lx_write,
		.read = default_flash_read,
		.probe = stm32lx_probe,
		.auto_probe = stm32lx_auto_probe,
		.erase_check = stm32lx_erase_check,
		.protect_check = stm32lx_protect_check,
		.info = stm32lx_get_info,
};

/* Static methods implementation */
static int stm32lx_unlock_program_memory(struct flash_bank *bank)
{
	struct target *target = bank->target;
	int retval;
	uint32_t reg32;

	/*
	 * Unlocking the program memory is done by unlocking the PECR,
	 * then by writing the 2 PRGKEY to the PRGKEYR register
	 */

	/* check flash is not already unlocked */
	retval = target_read_u32(target, FLASH_PECR, &reg32);
	if (retval != ERROR_OK)
		return retval;

	if ((reg32 & FLASH_PECR__PRGLOCK) == 0)
		return ERROR_OK;

	/* To unlock the PECR write the 2 PEKEY to the PEKEYR register */
	retval = target_write_u32(target, FLASH_PEKEYR, PEKEY1);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, FLASH_PEKEYR, PEKEY2);
	if (retval != ERROR_OK)
		return retval;

	/* Make sure it worked */
	retval = target_read_u32(target, FLASH_PECR, &reg32);
	if (retval != ERROR_OK)
		return retval;

	if (reg32 & FLASH_PECR__PELOCK) {
		LOG_ERROR("PELOCK is not cleared :(");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	retval = target_write_u32(target, FLASH_PRGKEYR, PRGKEY1);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, FLASH_PRGKEYR, PRGKEY2);
	if (retval != ERROR_OK)
		return retval;

	/* Make sure it worked */
	retval = target_read_u32(target, FLASH_PECR, &reg32);
	if (retval != ERROR_OK)
		return retval;

	if (reg32 & FLASH_PECR__PRGLOCK) {
		LOG_ERROR("PRGLOCK is not cleared :(");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static int stm32lx_enable_write_half_page(struct flash_bank *bank)
{
	struct target *target = bank->target;
	int retval;
	uint32_t reg32;

	/**
	 * Unlock the program memory, then set the FPRG bit in the PECR register.
	 */
	retval = stm32lx_unlock_program_memory(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, FLASH_PECR, &reg32);
	if (retval != ERROR_OK)
		return retval;

	reg32 |= FLASH_PECR__FPRG;
	retval = target_write_u32(target, FLASH_PECR, reg32);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, FLASH_PECR, &reg32);
	if (retval != ERROR_OK)
		return retval;

	reg32 |= FLASH_PECR__PROG;
	retval = target_write_u32(target, FLASH_PECR, reg32);

	return retval;
}

static int stm32lx_lock_program_memory(struct flash_bank *bank)
{
	struct target *target = bank->target;
	int retval;
	uint32_t reg32;

	/* To lock the program memory, simply set the lock bit and lock PECR */

	retval = target_read_u32(target, FLASH_PECR, &reg32);
	if (retval != ERROR_OK)
		return retval;

	reg32 |= FLASH_PECR__PRGLOCK;
	retval = target_write_u32(target, FLASH_PECR, reg32);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target, FLASH_PECR, &reg32);
	if (retval != ERROR_OK)
		return retval;

	reg32 |= FLASH_PECR__PELOCK;
	retval = target_write_u32(target, FLASH_PECR, reg32);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stm32lx_erase_sector(struct flash_bank *bank, int sector)
{
	struct target *target = bank->target;
	int retval;
	uint32_t reg32;

	/*
	 * To erase a sector (i.e. FLASH_PAGES_PER_SECTOR pages),
	 * first unlock the memory, loop over the pages of this sector
	 * and write 0x0 to its first word.
	 */

	retval = stm32lx_unlock_program_memory(bank);
	if (retval != ERROR_OK)
		return retval;

	for (int page = 0; page < FLASH_PAGES_PER_SECTOR; page++) {
		reg32 = FLASH_PECR__PROG | FLASH_PECR__ERASE;
		retval = target_write_u32(target, FLASH_PECR, reg32);
		if (retval != ERROR_OK)
			return retval;

		retval = stm32lx_wait_until_bsy_clear(bank);
		if (retval != ERROR_OK)
			return retval;

		uint32_t addr = bank->base + bank->sectors[sector].offset + (page
				* FLASH_PAGE_SIZE);
		retval = target_write_u32(target, addr, 0x0);
		if (retval != ERROR_OK)
			return retval;

		retval = stm32lx_wait_until_bsy_clear(bank);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = stm32lx_lock_program_memory(bank);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int stm32lx_wait_until_bsy_clear(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t status;
	int retval = ERROR_OK;
	int timeout = 100;

	/* wait for busy to clear */
	for (;;) {
		retval = target_read_u32(target, FLASH_SR, &status);
		if (retval != ERROR_OK)
			return retval;

		if ((status & FLASH_SR__BSY) == 0)
			break;
		if (timeout-- <= 0) {
			LOG_ERROR("timed out waiting for flash");
			return ERROR_FAIL;
		}
		alive_sleep(1);
	}

	if (status & FLASH_SR__WRPERR) {
		LOG_ERROR("access denied / write protected");
		retval = ERROR_FAIL;
	}

	if (status & FLASH_SR__PGAERR) {
		LOG_ERROR("invalid program address");
		retval = ERROR_FAIL;
	}

	return retval;
}
