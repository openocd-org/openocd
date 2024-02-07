// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 * Copyright (C) 2024  Transcelestial Technologies PTE LTD                 *
 * Paul Fertser <fercerpav@gmail.com>                                      *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/arm_opcodes.h>
#include <target/armv7m.h>

/**
 * @file
 * Flash programming support for NXP LPC55xx series (tested on LPC55S36 1B)
 */

/*
 * ID of LPC55S36 on dev board is 0x502a11a1
 */

#define LPC55XX_DEVICEID_REG	0x40000ff8
#define LPC55XX_DIEID_REG	0x40000ffc

#define API_FLASH_INIT		1
#define API_FLASH_ERASE		2
#define API_FLASH_PROGRAM	3
#define API_FLASH_VERIFY_ERASE	4

#define BANK_WA_SIZE	1024

struct lpc55xx_flash_bank {
	bool probed;
	uint32_t flash_api_table;
	struct working_area *wa;
};

FLASH_BANK_COMMAND_HANDLER(lpc55xx_flash_bank_command)
{
	struct lpc55xx_flash_bank *lpc55xx_info;

	if (CMD_ARGC < 7)
		return ERROR_COMMAND_SYNTAX_ERROR;

	lpc55xx_info = calloc(1, sizeof(struct lpc55xx_flash_bank));

	bank->driver_priv = lpc55xx_info;
	lpc55xx_info->probed = false;
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[6], lpc55xx_info->flash_api_table);

	return ERROR_OK;
}

static int lpc55xx_api_call(struct flash_bank *bank, unsigned int func, int regs, struct reg_param reg_params[])
{
	struct lpc55xx_flash_bank *lpc55xx_info = bank->driver_priv;
	struct target *target = bank->target;
	struct armv7m_algorithm armv7m_info;
	uint32_t func_addr;
	uint32_t status;

	if (lpc55xx_info->wa == NULL) {
		if (target_alloc_working_area(target, BANK_WA_SIZE, &lpc55xx_info->wa) != ERROR_OK) {
			LOG_ERROR("No working area specified, can not use flash API");
			return ERROR_FAIL;
		}

		/* We'll be using this as LR target to halt after API function call */
		target_write_u32(target, lpc55xx_info->wa->address, ARMV5_T_BKPT(0));
	}

	target_read_u32(target, lpc55xx_info->flash_api_table + 4 * func, &func_addr);

	/* we have bkpt #0 there in the beginning of wa */
	init_reg_param(&reg_params[0], "lr", 32, PARAM_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, lpc55xx_info->wa->address | 1);
#if 0
	/* TODO what to set SP to?.. Can we count on "reset init" to leave it in suitable state? */
	init_reg_param(&reg_params[1], "sp", 32, PARAM_OUT);
	buf_set_u32(reg_params[1].value, 0, 32, lpc55xx_info->wa->address + BANK_WA_SIZE);
#else
	/* dummy set to keep the count */
	init_reg_param(&reg_params[1], "r0", 32, PARAM_OUT);
	buf_set_u32(reg_params[1].value, 0, 32, 0);
#endif
	init_reg_param(&reg_params[2], "r0", 32, PARAM_IN_OUT);
	buf_set_u32(reg_params[2].value, 0, 32, lpc55xx_info->wa->address + 4);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;
	target_run_algorithm(target, 0, NULL, regs, reg_params, func_addr - 1, 0, 10000, &armv7m_info);

	status = buf_get_u32(reg_params[2].value, 0, 32);
	LOG_INFO("API call %d status 0x%8.8" PRIx32, func, status);

	for (int i = 0; i < regs; i++)
		destroy_reg_param(&reg_params[i]);

	return status;
}

static int lpc55xx_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct reg_param reg_params[6];
	size_t sectors_size = 0;
	int retval;

	for (size_t i = first; i <= last; i++)
		sectors_size += bank->sectors[i].size;

	init_reg_param(&reg_params[3], "r1", 32, PARAM_IN_OUT);
	buf_set_u32(reg_params[3].value, 0, 32, bank->base + bank->sectors[first].offset);
	init_reg_param(&reg_params[4], "r2", 32, PARAM_IN_OUT);
	buf_set_u32(reg_params[4].value, 0, 32, sectors_size);
	init_reg_param(&reg_params[5], "r3", 32, PARAM_IN_OUT);
	buf_set_u32(reg_params[5].value, 0, 32, 'k' << 24 | 'e' << 16 | 'f' << 8 | 'l');

	retval = lpc55xx_api_call(bank, API_FLASH_ERASE, ARRAY_SIZE(reg_params), reg_params);

	return retval;
}

static int lpc55xx_erase_check(struct flash_bank *bank)
{
	struct reg_param reg_params[5];
	int retval = ERROR_FAIL;

	for (size_t i = 0; i < bank->num_sectors; i++) {
		init_reg_param(&reg_params[3], "r1", 32, PARAM_IN_OUT);
		buf_set_u32(reg_params[3].value, 0, 32, bank->base + bank->sectors[i].offset);
		init_reg_param(&reg_params[4], "r2", 32, PARAM_IN_OUT);
		buf_set_u32(reg_params[4].value, 0, 32, bank->sectors[i].size);

		retval = lpc55xx_api_call(bank, API_FLASH_VERIFY_ERASE, ARRAY_SIZE(reg_params), reg_params);

		if (retval == 0)
			bank->sectors[i].is_erased = true;
		else if (retval == 105)
			bank->sectors[i].is_erased = false;
		else
			break;
	}

	return retval;
}

static int lpc55xx_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct working_area *download_area;
	struct target *target = bank->target;
	struct reg_param reg_params[6];
	const size_t download_size = 8192;
	int retval;

	if (target_alloc_working_area(target, download_size, &download_area) != ERROR_OK) {
		LOG_ERROR("Failed to allocate working area for write buffer");
		return ERROR_FAIL;
	}

	while (count != 0) {
		size_t bytes_to_write = MIN(count, download_size);

		retval = target_write_buffer(target, download_area->address, bytes_to_write, buffer + offset);
		if (retval != ERROR_OK) {
			retval = ERROR_FLASH_OPERATION_FAILED;
			goto end;
		}

		init_reg_param(&reg_params[3], "r1", 32, PARAM_IN_OUT);
		buf_set_u32(reg_params[3].value, 0, 32, offset);
		init_reg_param(&reg_params[4], "r2", 32, PARAM_IN_OUT);
		buf_set_u32(reg_params[4].value, 0, 32, download_area->address);
		init_reg_param(&reg_params[5], "r3", 32, PARAM_IN_OUT);
		buf_set_u32(reg_params[5].value, 0, 32, bytes_to_write);

		retval = lpc55xx_api_call(bank, API_FLASH_PROGRAM, ARRAY_SIZE(reg_params), reg_params);
		if (retval != ERROR_OK) {
			LOG_ERROR("Error writing flash");
			goto end;
		}

		count -= bytes_to_write;
		offset += bytes_to_write;
	}

end:
	target_free_working_area(target, download_area);

	return retval;
}


static int lpc55xx_probe(struct flash_bank *bank)
{
	struct lpc55xx_flash_bank *lpc55xx_info = bank->driver_priv;
	uint32_t device_id, sector_size, ver, base;
	struct target *target = bank->target;
	struct reg_param reg_params[3];
	uint32_t last_sector_size;
	int retval;

	target_read_u32(target, LPC55XX_DEVICEID_REG, &device_id);
	LOG_INFO("Device ID: 0x%8.8" PRIx32, device_id);
	target_read_u32(target, LPC55XX_DIEID_REG, &ver);
	LOG_INFO("Chip revision ID and Number (DIEID): 0x%8.8" PRIx32, ver);

	if (lpc55xx_info->flash_api_table == 0) {
		uint32_t rom_api_table;

		if (device_id == 0x502a11a1) { /* LPC55S36 */
			rom_api_table = 0x1302fc00;
		/* } else if (device_id == XXXX) /\* LPC55S6x/LPC55S2x/LPC552x *\/ {*/
		/*	rom_api_table = 0x130010f0; */
		} else {
			LOG_ERROR("Unknown LPC55xx variant and ROM API table address not specified");
			return ERROR_FAIL;
		}

		target_read_u32(target, rom_api_table + 4, &ver);
		LOG_INFO("ROM API version: 0x%8.8" PRIx32, ver);
		target_read_u32(target, rom_api_table + 0x10, &lpc55xx_info->flash_api_table);
	}

	target_read_u32(target, lpc55xx_info->flash_api_table, &ver);
	LOG_INFO("Flash API version: 0x%8.8" PRIx32, ver);

	retval = lpc55xx_api_call(bank, API_FLASH_INIT, ARRAY_SIZE(reg_params), reg_params);
	if (retval != ERROR_OK) {
		LOG_ERROR("flash_init() failed");
		return retval;
	}

	target_read_u32(target, lpc55xx_info->wa->address + 4, &base);
	if (base != bank->base) {
		LOG_ERROR("Flash bank address mismatch: API reported 0x%8.8" PRIx32, base);
		return ERROR_FAIL;
	}

	target_read_u32(target, lpc55xx_info->wa->address + 8, &bank->size);
	target_read_u32(target, lpc55xx_info->wa->address + 0x14, &sector_size);

	free(bank->sectors);

	bank->num_sectors = bank->size / sector_size;
	last_sector_size = bank->size % sector_size;
	if (last_sector_size != 0)
		bank->num_sectors++;
	bank->sectors = alloc_block_array(0, sector_size, bank->num_sectors);
	if (!bank->sectors)
		return ERROR_FAIL;
	if (last_sector_size != 0)
		bank->sectors[bank->num_sectors - 1].size = last_sector_size;

	target_read_u32(target, lpc55xx_info->wa->address + 0x10, &bank->write_start_alignment);
	bank->write_end_alignment = bank->write_start_alignment;

	lpc55xx_info->probed = true;

	return ERROR_OK;
}

static int lpc55xx_auto_probe(struct flash_bank *bank)
{
	struct lpc55xx_flash_bank *lpc55xx_info = bank->driver_priv;
	if (lpc55xx_info->probed)
		return ERROR_OK;
	return lpc55xx_probe(bank);
}

const struct flash_driver lpc55xx_flash = {
	.name = "lpc55xx",
	.flash_bank_command = lpc55xx_flash_bank_command,
	.erase = lpc55xx_erase,
	.write = lpc55xx_write,
	.read = default_flash_read,
	.probe = lpc55xx_probe,
	.auto_probe = lpc55xx_auto_probe,
	.erase_check = lpc55xx_erase_check,
	.free_driver_priv = default_flash_free_driver_priv,
};
