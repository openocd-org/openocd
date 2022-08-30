// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2022 by Toms Stūrmanis                                  *
 *   toms.sturmanis@gmail.com                                              *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdint.h>

#include <helper/binarybuffer.h>
#include <helper/bits.h>

#include <target/algorithm.h>
#include <target/arm_adi_v5.h>
#include <target/armv7m.h>
#include <target/cortex_m.h>

#include "imp.h"

#define RSL10_FLASH_ADDRESS_MAIN              0x00100000
#define RSL10_FLASH_ADDRESS_NVR1              0x00080000
#define RSL10_FLASH_ADDRESS_NVR2              0x00080800
#define RSL10_FLASH_ADDRESS_NVR3              0x00081000
#define RSL10_FLASH_ADDRESS_NVR4              0x00081800
#define RSL10_FLASH_ADDRESS_LOCK_INFO_SETTING 0x00081040

#define RSL10_REG_ID 0x1FFFFFFC

#define RSL10_FLASH_REG_MAIN_WRITE_UNLOCK 0x40000504
#define RSL10_FLASH_REG_MAIN_CTRL         0x40000508
#define RSL10_FLASH_REG_IF_STATUS         0x40000538
#define RSL10_FLASH_REG_NVR_WRITE_UNLOCK  0x40000548
#define RSL10_FLASH_REG_NVR_CTRL          0x4000054C

#define RSL10_FLASH_REG_DEBUG_UNLOCK_KEY1 0x400000F0
#define RSL10_FLASH_REG_DEBUG_UNLOCK_KEY2 0x400000F4
#define RSL10_FLASH_REG_DEBUG_UNLOCK_KEY3 0x400000F8
#define RSL10_FLASH_REG_DEBUG_UNLOCK_KEY4 0x400000FC

#define RSL10_NVR3_USER_KEY_OFFSET 0x40

#define RSL10_ID             0x09010106
#define RSL10_FLASH_KEY_MAIN 0xDBC8264E
#define RSL10_FLASH_KEY_NVR  0x71B371F5
#define RSL10_KEY_DEBUG_LOCK 0x4C6F634B

#define RSL10_FLASH_REG_MAIN_CTRL_LOW_W_ENABLE    BIT(0)
#define RSL10_FLASH_REG_MAIN_CTRL_MIDDLE_W_ENABLE BIT(1)
#define RSL10_FLASH_REG_MAIN_CTRL_HIGH_W_ENABLE   BIT(2)

#define RSL10_FLASH_REG_NVR_CTRL_NVR1_W_ENABLE BIT(1)
#define RSL10_FLASH_REG_NVR_CTRL_NVR2_W_ENABLE BIT(2)
#define RSL10_FLASH_REG_NVR_CTRL_NVR3_W_ENABLE BIT(3)

#define RSL10_FLASH_REG_STATUS_LOW_W_UNLOCKED    BIT(0)
#define RSL10_FLASH_REG_STATUS_MIDDLE_W_UNLOCKED BIT(1)
#define RSL10_FLASH_REG_STATUS_HIGH_W_UNLOCKED   BIT(2)
#define RSL10_FLASH_REG_STATUS_NVR1_W_UNLOCKED   BIT(4)
#define RSL10_FLASH_REG_STATUS_NVR2_W_UNLOCKED   BIT(5)
#define RSL10_FLASH_REG_STATUS_NVR3_W_UNLOCKED   BIT(6)

#define RSL10_ROM_CMD_WRITE_WORD_PAIR 0x3C
#define RSL10_ROM_CMD_WRITE_BUFFER    0x40
#define RSL10_ROM_CMD_ERASE_SECTOR    0x44
#define RSL10_ROM_CMD_ERASE_ALL       0x48

#define FLASH_SECTOR_SIZE 0x2000

#define RSL10_ROM_CMD_WRITE_BUFFER_MAX_SIZE FLASH_SECTOR_SIZE

#define ALGO_STACK_POINTER_ADDR  0x20002000

/* Used to launch flash related functions from ROM
 * Params :
 * r0-r2 = arguments
 * r3 = target address in rom
 */
static const uint8_t rsl10_rom_launcher_code[] = {
#include "../../../contrib/loaders/flash/rsl10/rom_launcher.inc"
};

enum rsl10_flash_status {
	RSL10_FLASH_ERR_NONE              = 0x0,
	RSL10_FLASH_ERR_GENERAL_FAILURE   = 0x1,
	RSL10_FLASH_ERR_WRITE_NOT_ENABLED = 0x2,
	RSL10_FLASH_ERR_BAD_ADDRESS       = 0x3,
	RSL10_FLASH_ERR_ERASE_FAILED      = 0x4,
	RSL10_FLASH_ERR_BAD_LENGTH        = 0x5,
	RSL10_FLASH_ERR_INACCESSIBLE      = 0x6,
	RSL10_FLASH_ERR_COPIER_BUSY       = 0x7,
	RSL10_FLASH_ERR_PROG_FAILED       = 0x8,
	RSL10_FLASH_MAX_ERR_CODES /* must be the last one */
};

static const char *const rsl10_error_list[] = {
	[RSL10_FLASH_ERR_GENERAL_FAILURE]   = "general failure",
	[RSL10_FLASH_ERR_WRITE_NOT_ENABLED] = "write not enabled, protected",
	[RSL10_FLASH_ERR_BAD_ADDRESS]       = "bad address",
	[RSL10_FLASH_ERR_ERASE_FAILED]      = "erase failed",
	[RSL10_FLASH_ERR_BAD_LENGTH]        = "bad length",
	[RSL10_FLASH_ERR_INACCESSIBLE]      = "inaccessible: not powered up, or isolated",
	[RSL10_FLASH_ERR_COPIER_BUSY]       = "copier busy",
	[RSL10_FLASH_ERR_PROG_FAILED]       = "prog failed",
};

const char *rsl10_error(enum rsl10_flash_status x)
{
	if (x >= RSL10_FLASH_MAX_ERR_CODES || !rsl10_error_list[x])
		return "unknown";
	return rsl10_error_list[x];
}

const struct flash_driver rsl10_flash;

struct rsl10_info {
	unsigned int refcount;

	struct rsl10_bank {
		struct rsl10_info *chip;
		bool probed;
	} bank[5];
	struct target *target;

	unsigned int flash_size_kb;
};

static bool rsl10_bank_is_probed(const struct flash_bank *bank)
{
	struct rsl10_bank *nbank = bank->driver_priv;
	assert(nbank);
	return nbank->probed;
}

static int rsl10_probe(struct flash_bank *bank);

static int rsl10_get_probed_chip_if_halted(struct flash_bank *bank, struct rsl10_info **chip)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	struct rsl10_bank *nbank = bank->driver_priv;
	*chip                    = nbank->chip;

	if (rsl10_bank_is_probed(bank))
		return ERROR_OK;

	return rsl10_probe(bank);
}

static int rsl10_protect_check(struct flash_bank *bank)
{
	struct rsl10_bank *nbank = bank->driver_priv;
	struct rsl10_info *chip  = nbank->chip;

	assert(chip);

	uint32_t status;

	int retval = target_read_u32(bank->target, RSL10_FLASH_REG_IF_STATUS, &status);
	if (retval != ERROR_OK)
		return retval;

	if (bank->base == RSL10_FLASH_ADDRESS_MAIN) {
		for (unsigned int i = 0; i < bank->num_prot_blocks; i++)
			bank->prot_blocks[i].is_protected = (status & (1 << i)) ? 0 : 1;

	} else {
		uint32_t test_bit = 0;
		switch (bank->base) {
		case RSL10_FLASH_ADDRESS_NVR1:
			test_bit = RSL10_FLASH_REG_STATUS_NVR1_W_UNLOCKED;
			break;
		case RSL10_FLASH_ADDRESS_NVR2:
			test_bit = RSL10_FLASH_REG_STATUS_NVR2_W_UNLOCKED;
			break;
		case RSL10_FLASH_ADDRESS_NVR3:
			test_bit = RSL10_FLASH_REG_STATUS_NVR3_W_UNLOCKED;
			break;
		default:
			break;
		}

		bank->sectors[0].is_protected = (status & test_bit) ? 0 : 1;
	}
	return ERROR_OK;
}

static int rsl10_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last)
{

	struct rsl10_info *chip;
	int retval = rsl10_get_probed_chip_if_halted(bank, &chip);
	if (retval != ERROR_OK)
		return retval;

	if (bank->base == RSL10_FLASH_ADDRESS_MAIN) {
		uint32_t status;
		retval = target_read_u32(bank->target, RSL10_FLASH_REG_MAIN_CTRL, &status);
		if (retval != ERROR_OK)
			return retval;

		for (unsigned int i = first; i <= last; i++) {
			if (set)
				status &= ~(1 << i);
			else
				status |= (1 << i);
		}

		retval = target_write_u32(bank->target, RSL10_FLASH_REG_MAIN_CTRL, status);
		if (retval != ERROR_OK)
			return retval;

		retval = target_write_u32(bank->target, RSL10_FLASH_REG_MAIN_WRITE_UNLOCK, RSL10_FLASH_KEY_MAIN);
		if (retval != ERROR_OK)
			return retval;
	} else {
		uint32_t bit = 0;
		switch (bank->base) {
		case RSL10_FLASH_ADDRESS_NVR1:
			bit = RSL10_FLASH_REG_NVR_CTRL_NVR1_W_ENABLE;
			break;
		case RSL10_FLASH_ADDRESS_NVR2:
			bit = RSL10_FLASH_REG_NVR_CTRL_NVR2_W_ENABLE;
			break;
		case RSL10_FLASH_ADDRESS_NVR3:
			bit = RSL10_FLASH_REG_NVR_CTRL_NVR3_W_ENABLE;
			break;
		default:
			break;
		}

		uint32_t status;
		retval = target_read_u32(bank->target, RSL10_FLASH_REG_NVR_CTRL, &status);
		if (retval != ERROR_OK)
			return retval;

		if (set)
			status &= ~bit;
		else
			status |= bit;

		retval = target_write_u32(bank->target, RSL10_FLASH_REG_NVR_CTRL, status);
		if (retval != ERROR_OK)
			return retval;

		retval = target_write_u32(bank->target, RSL10_FLASH_REG_NVR_WRITE_UNLOCK, RSL10_FLASH_KEY_NVR);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int rsl10_check_device(struct flash_bank *bank)
{
	uint32_t configid;
	int retval = target_read_u32(bank->target, RSL10_REG_ID, &configid);
	if (retval != ERROR_OK)
		return retval;

	if (configid != RSL10_ID) {
		LOG_ERROR("This is not supported (RSL10) device, use other flash driver!!!");
		return ERROR_TARGET_INVALID;
	}
	return ERROR_OK;
}

static int rsl10_probe(struct flash_bank *bank)
{
	struct rsl10_bank *nbank = bank->driver_priv;
	struct rsl10_info *chip  = nbank->chip;

	int retval = rsl10_check_device(bank);
	if (retval != ERROR_OK)
		return retval;

	unsigned int bank_id;
	unsigned int num_prot_blocks = 0;
	switch (bank->base) {
	case RSL10_FLASH_ADDRESS_MAIN:
		bank_id         = 0;
		num_prot_blocks = 3;
		break;
	case RSL10_FLASH_ADDRESS_NVR1:
		bank_id = 1;
		break;
	case RSL10_FLASH_ADDRESS_NVR2:
		bank_id = 2;
		break;
	case RSL10_FLASH_ADDRESS_NVR3:
		bank_id = 3;
		break;
	default:
		return ERROR_FAIL;
	}

	uint32_t flash_page_size = 2048;

	bank->write_start_alignment = 8;
	bank->write_end_alignment   = 8;

	bank->num_sectors   = bank->size / flash_page_size;
	chip->flash_size_kb = bank->size / 1024;

	free(bank->sectors);
	bank->sectors = NULL;

	bank->sectors = alloc_block_array(0, flash_page_size, bank->num_sectors);
	if (!bank->sectors)
		return ERROR_FAIL;

	free(bank->prot_blocks);
	bank->prot_blocks = NULL;

	if (num_prot_blocks > 0) {
		bank->num_prot_blocks = num_prot_blocks;
		bank->prot_blocks     = alloc_block_array(0, bank->num_sectors / 3 * flash_page_size, bank->num_prot_blocks);
		if (!bank->prot_blocks)
			return ERROR_FAIL;
	}

	chip->bank[bank_id].probed = true;
	return ERROR_OK;
}

static int rsl10_auto_probe(struct flash_bank *bank)
{
	if (rsl10_bank_is_probed(bank))
		return ERROR_OK;

	return rsl10_probe(bank);
}

static int rsl10_ll_flash_erase(struct rsl10_info *chip, uint32_t address)
{
	struct target *target = chip->target;
	struct working_area *write_algorithm;

	LOG_DEBUG("erasing buffer flash address=0x%" PRIx32, address);

	int retval = target_alloc_working_area(target, sizeof(rsl10_rom_launcher_code), &write_algorithm);
	if (retval != ERROR_OK) {
		LOG_ERROR("Current working area 0x%x is too small! Increase working area size!", target->working_area_size);
		return ERROR_FAIL;
	}

	retval =
		target_write_buffer(target, write_algorithm->address, sizeof(rsl10_rom_launcher_code), rsl10_rom_launcher_code);
	if (retval != ERROR_OK)
		goto free_algorithm;

	struct reg_param reg_params[3];
	struct armv7m_algorithm armv7m_info;
	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode    = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT); /* address */
	init_reg_param(&reg_params[1], "r3", 32, PARAM_OUT);    /* cmd */
	init_reg_param(&reg_params[2], "sp", 32, PARAM_OUT);    /* stack pointer */

	buf_set_u32(reg_params[0].value, 0, 32, address);
	uint32_t cmd;
	retval = target_read_u32(target, RSL10_ROM_CMD_ERASE_SECTOR, &cmd);
	if (retval != ERROR_OK)
		goto free_reg_params;
	buf_set_u32(reg_params[1].value, 0, 32, cmd);
	buf_set_u32(reg_params[2].value, 0, 32, ALGO_STACK_POINTER_ADDR);

	retval = target_run_algorithm(
		target, 0, NULL, ARRAY_SIZE(reg_params), reg_params, write_algorithm->address,
		write_algorithm->address + sizeof(rsl10_rom_launcher_code) - 2, 1000, &armv7m_info
	);
	if (retval != ERROR_OK)
		goto free_reg_params;

	int algo_ret = buf_get_u32(reg_params[0].value, 0, 32);
	if (algo_ret != RSL10_FLASH_ERR_NONE) {
		LOG_ERROR("RSL10 ERASE ERROR: '%s' (%d)", rsl10_error(algo_ret), algo_ret);
		retval = ERROR_FLASH_SECTOR_NOT_ERASED;
	}

free_reg_params:
	for (unsigned int i = 0; i < ARRAY_SIZE(reg_params); i++)
		destroy_reg_param(&reg_params[i]);

free_algorithm:
	target_free_working_area(target, write_algorithm);
	return retval;
}

static int rsl10_ll_flash_write(struct rsl10_info *chip, uint32_t address, const uint8_t *buffer, uint32_t bytes)
{
	struct target *target = chip->target;
	struct working_area *write_algorithm;

	if (bytes == 8) {
		uint32_t data;
		data = buf_get_u32(buffer, 0, 32);
		LOG_DEBUG("Writing 0x%" PRIx32 " to flash address=0x%" PRIx32 " bytes=0x%" PRIx32, data, address, bytes);
	} else
		LOG_DEBUG("Writing buffer to flash address=0x%" PRIx32 " bytes=0x%" PRIx32, address, bytes);

	/* allocate working area with flash programming code */
	int retval = target_alloc_working_area(target, sizeof(rsl10_rom_launcher_code), &write_algorithm);
	if (retval != ERROR_OK) {
		LOG_ERROR("Current working area 0x%x is too small! Increase working area size!", target->working_area_size);
		return ERROR_FAIL;
	}

	retval =
		target_write_buffer(target, write_algorithm->address, sizeof(rsl10_rom_launcher_code), rsl10_rom_launcher_code);
	if (retval != ERROR_OK)
		goto free_algorithm;

	/* memory buffer, rounded down, to be multiple of 8 */
	uint32_t buffer_avail = target_get_working_area_avail(target) & ~7;
	uint32_t buffer_size  = MIN(RSL10_ROM_CMD_WRITE_BUFFER_MAX_SIZE, buffer_avail);
	struct working_area *source;
	retval = target_alloc_working_area(target, buffer_size, &source);
	if (retval != ERROR_OK) {
		LOG_ERROR("Current working area 0x%x is too small! Increase working area size!", target->working_area_size);
		goto free_algorithm;
	}

	struct reg_param reg_params[5];
	struct armv7m_algorithm armv7m_info;
	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode    = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT); /* start addr, return value */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);    /* length */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);    /* data */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);    /* cmd */
	init_reg_param(&reg_params[4], "sp", 32, PARAM_OUT);    /* stack pointer */
	buf_set_u32(reg_params[4].value, 0, 32, ALGO_STACK_POINTER_ADDR);

	uint32_t cmd             = 0;
	uint32_t sent_bytes      = 0;
	uint32_t write_address   = 0;
	uint32_t bytes_to_send   = 0;
	uint32_t remaining_bytes = 0;

	retval = target_read_u32(target, RSL10_ROM_CMD_WRITE_BUFFER, &cmd);
	if (retval != ERROR_OK)
		goto free_everything;

	while (sent_bytes < bytes) {
		remaining_bytes = bytes - sent_bytes;
		bytes_to_send   = remaining_bytes >= buffer_size ? buffer_size : remaining_bytes;

		retval = target_write_buffer(target, source->address, bytes_to_send, buffer + sent_bytes);
		if (retval != ERROR_OK)
			goto free_everything;

		write_address = address + sent_bytes;

		LOG_DEBUG(
			"write_address: 0x%" PRIx32 ", words: 0x%" PRIx32 ", source: 0x%" PRIx64 ", cmd: 0x%" PRIx32, write_address,
			bytes_to_send / 4, source->address, cmd
		);
		buf_set_u32(reg_params[0].value, 0, 32, write_address);
		buf_set_u32(reg_params[1].value, 0, 32, bytes_to_send / 4);
		buf_set_u32(reg_params[2].value, 0, 32, source->address);
		buf_set_u32(reg_params[3].value, 0, 32, cmd);

		retval = target_run_algorithm(
			target, 0, NULL, ARRAY_SIZE(reg_params), reg_params, write_algorithm->address,
			write_algorithm->address + sizeof(rsl10_rom_launcher_code) - 2, 1000, &armv7m_info
		);
		if (retval != ERROR_OK)
			goto free_everything;

		int algo_ret = buf_get_u32(reg_params[0].value, 0, 32);
		if (algo_ret != RSL10_FLASH_ERR_NONE) {
			LOG_ERROR("RSL10 WRITE ERROR: '%s' (%d)", rsl10_error(algo_ret), algo_ret);
			retval = ERROR_FLASH_OPERATION_FAILED;
			goto free_everything;
		}

		sent_bytes += bytes_to_send;
	}

free_everything:
	target_free_working_area(target, source);

	for (unsigned int i = 0; i < ARRAY_SIZE(reg_params); i++)
		destroy_reg_param(&reg_params[i]);

free_algorithm:
	target_free_working_area(target, write_algorithm);

	return retval;
}

static int rsl10_mass_erase(struct target *target)
{
	struct working_area *write_algorithm;

	int retval = target_alloc_working_area(target, sizeof(rsl10_rom_launcher_code), &write_algorithm);
	if (retval != ERROR_OK) {
		LOG_ERROR("Current working area 0x%x is too small! Increase working area size!", target->working_area_size);
		return ERROR_FAIL;
	}

	retval =
		target_write_buffer(target, write_algorithm->address, sizeof(rsl10_rom_launcher_code), rsl10_rom_launcher_code);
	if (retval != ERROR_OK)
		goto free_algorithm;

	struct reg_param reg_params[3];
	struct armv7m_algorithm armv7m_info;
	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode    = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT); /* return value */
	init_reg_param(&reg_params[1], "r3", 32, PARAM_OUT);    /* cmd */
	init_reg_param(&reg_params[2], "sp", 32, PARAM_OUT);    /* stack pointer */

	uint32_t cmd;
	retval = target_read_u32(target, RSL10_ROM_CMD_ERASE_ALL, &cmd);
	if (retval != ERROR_OK)
		goto free_reg_params;
	buf_set_u32(reg_params[1].value, 0, 32, cmd);
	buf_set_u32(reg_params[2].value, 0, 32, ALGO_STACK_POINTER_ADDR);

	retval = target_run_algorithm(
		target, 0, NULL, ARRAY_SIZE(reg_params), reg_params, write_algorithm->address,
		write_algorithm->address + sizeof(rsl10_rom_launcher_code) - 2, 1000, &armv7m_info
	);
	if (retval != ERROR_OK)
		goto free_reg_params;

	int algo_ret = buf_get_u32(reg_params[0].value, 0, 32);
	if (algo_ret != RSL10_FLASH_ERR_NONE) {
		LOG_ERROR("RSL10 MASS ERASE ERROR: '%s' (%d)", rsl10_error(algo_ret), algo_ret);
		retval = ERROR_FLASH_OPERATION_FAILED;
	}

free_reg_params:
	for (unsigned int i = 0; i < ARRAY_SIZE(reg_params); i++)
		destroy_reg_param(&reg_params[i]);

free_algorithm:
	target_free_working_area(target, write_algorithm);
	return retval;
}

static int rsl10_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct rsl10_info *chip;

	int retval = rsl10_get_probed_chip_if_halted(bank, &chip);
	if (retval != ERROR_OK)
		return retval;

	return rsl10_ll_flash_write(chip, bank->base + offset, buffer, count);
}

static int rsl10_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	LOG_INFO("erase bank: %x, %x", first, last);
	int retval;
	struct rsl10_info *chip;

	retval = rsl10_get_probed_chip_if_halted(bank, &chip);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int i = first; i <= last; i++) {
		retval = rsl10_ll_flash_erase(chip, bank->base + i * 0x800);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static void rsl10_free_driver_priv(struct flash_bank *bank)
{
	struct rsl10_bank *nbank = bank->driver_priv;
	struct rsl10_info *chip  = nbank->chip;
	if (!chip)
		return;

	chip->refcount--;
	if (chip->refcount == 0) {
		free(chip);
		bank->driver_priv = NULL;
	}
}

static struct rsl10_info *rsl10_get_chip(struct target *target)
{
	struct flash_bank *bank_iter;

	/* iterate over rsl10 banks of same target */
	for (bank_iter = flash_bank_list(); bank_iter; bank_iter = bank_iter->next) {
		if (bank_iter->driver != &rsl10_flash)
			continue;

		if (bank_iter->target != target)
			continue;

		struct rsl10_bank *nbank = bank_iter->driver_priv;
		if (!nbank)
			continue;

		if (nbank->chip)
			return nbank->chip;
	}
	return NULL;
}

FLASH_BANK_COMMAND_HANDLER(rsl10_flash_bank_command)
{
	struct rsl10_info *chip  = NULL;
	struct rsl10_bank *nbank = NULL;
	LOG_INFO("Creating flash @ " TARGET_ADDR_FMT, bank->base);

	switch (bank->base) {
	case RSL10_FLASH_ADDRESS_MAIN:
	case RSL10_FLASH_ADDRESS_NVR1:
	case RSL10_FLASH_ADDRESS_NVR2:
	case RSL10_FLASH_ADDRESS_NVR3:
	case RSL10_FLASH_ADDRESS_NVR4:
		break;
	default:
		LOG_ERROR("Invalid bank address " TARGET_ADDR_FMT, bank->base);
		return ERROR_FAIL;
	}

	chip = rsl10_get_chip(bank->target);
	if (!chip) {
		chip = calloc(1, sizeof(*chip));
		if (!chip)
			return ERROR_FAIL;

		chip->target = bank->target;
	}

	switch (bank->base) {
	case RSL10_FLASH_ADDRESS_MAIN:
		nbank = &chip->bank[0];
		break;
	case RSL10_FLASH_ADDRESS_NVR1:
		nbank = &chip->bank[1];
		break;
	case RSL10_FLASH_ADDRESS_NVR2:
		nbank = &chip->bank[2];
		break;
	case RSL10_FLASH_ADDRESS_NVR3:
		nbank = &chip->bank[3];
		break;
	case RSL10_FLASH_ADDRESS_NVR4:
		nbank = &chip->bank[4];
		break;
	}
	assert(nbank);

	chip->refcount++;
	nbank->chip       = chip;
	nbank->probed     = false;
	bank->driver_priv = nbank;

	return ERROR_OK;
}

COMMAND_HANDLER(rsl10_lock_command)
{
	struct target *target = get_current_target(CMD_CTX);

	if (CMD_ARGC != 4)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = get_flash_bank_by_addr(target, RSL10_FLASH_ADDRESS_NVR3, true, &bank);
	if (retval != ERROR_OK)
		return retval;

	LOG_INFO("Keys used: %s %s %s %s", CMD_ARGV[0], CMD_ARGV[1], CMD_ARGV[2], CMD_ARGV[3]);

	uint32_t user_key[4];
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], user_key[0]);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], user_key[1]);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], user_key[2]);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[3], user_key[3]);

	uint8_t write_buffer[6 * 4];
	target_buffer_set_u32(target, write_buffer, RSL10_KEY_DEBUG_LOCK);
	target_buffer_set_u32_array(target, &write_buffer[4], 4, user_key);
	/* pad the end to 64-bit word boundary */
	memset(&write_buffer[5 * 4], bank->default_padded_value, 4);

	retval = rsl10_erase(bank, 0, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = rsl10_write(bank, write_buffer, RSL10_NVR3_USER_KEY_OFFSET, sizeof(write_buffer));
	if (retval != ERROR_OK) {
		/* erase sector, if write fails, otherwise it can lock debug with wrong keys */
		return rsl10_erase(bank, 0, 0);
	}

	command_print(
		CMD, "****** WARNING ******\n"
			 "rsl10 device has been successfully prepared to lock.\n"
			 "Debug port is locked after restart.\n"
			 "Unlock with 'rsl10_unlock key0 key1 key2 key3'\n"
			 "****** ....... ******\n"
	);

	return rsl10_protect(bank, true, 0, 0);
}

COMMAND_HANDLER(rsl10_unlock_command)
{
	if (CMD_ARGC != 4)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target            = get_current_target(CMD_CTX);
	struct cortex_m_common *cortex_m = target_to_cm(target);

	struct adiv5_dap *dap = cortex_m->armv7m.arm.dap;
	struct adiv5_ap *ap   = dap_get_ap(dap, 0);

	uint32_t user_key[4];
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], user_key[0]);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], user_key[1]);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], user_key[2]);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[3], user_key[3]);

	uint8_t write_buffer1[4 * 4];
	target_buffer_set_u32_array(target, write_buffer1, 4, user_key);
	int retval = mem_ap_write_buf(ap, write_buffer1, 4, 4, RSL10_FLASH_REG_DEBUG_UNLOCK_KEY1);
	if (retval != ERROR_OK) {
		dap_put_ap(ap);
		return retval;
	}

	dap_put_ap(ap);

	uint32_t key;
	retval = mem_ap_read_atomic_u32(ap, RSL10_FLASH_ADDRESS_LOCK_INFO_SETTING, &key);
	if (retval != ERROR_OK)
		return retval;
	LOG_INFO("mem read: 0x%08" PRIx32, key);

	if (key == RSL10_KEY_DEBUG_LOCK) {
		retval = command_run_line(CMD_CTX, "reset init");
		if (retval != ERROR_OK)
			return retval;

		struct flash_bank *bank;
		retval = get_flash_bank_by_addr(target, RSL10_FLASH_ADDRESS_NVR3, true, &bank);
		if (retval != ERROR_OK)
			return retval;

		retval = rsl10_protect(bank, false, 0, 0);
		if (retval != ERROR_OK)
			return retval;

		uint8_t write_buffer2[4 * 2];
		target_buffer_set_u32(target, write_buffer2, 0x1);
		/* pad the end to 64-bit word boundary */
		memset(&write_buffer2[4], bank->default_padded_value, 4);

		/* let it fail, because sector is not erased, maybe just erase all? */
		(void)rsl10_write(bank, write_buffer2, RSL10_NVR3_USER_KEY_OFFSET, sizeof(write_buffer2));
		command_print(CMD, "Debug port is unlocked!");
	}

	return ERROR_OK;
}

COMMAND_HANDLER(rsl10_mass_erase_command)
{
	if (CMD_ARGC)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);

	int retval = rsl10_mass_erase(target);
	if (retval != ERROR_OK)
		return retval;

	command_print(CMD, "Mass erase was succesfull!");
	return ERROR_OK;
}

static const struct command_registration rsl10_exec_command_handlers[] = {
	{
		.name    = "lock",
		.handler = rsl10_lock_command,
		.mode    = COMMAND_EXEC,
		.help    = "Lock rsl10 debug, with passed keys",
		.usage   = "key1 key2 key3 key4",
	},
	{
		.name    = "unlock",
		.handler = rsl10_unlock_command,
		.mode    = COMMAND_EXEC,
		.help    = "Unlock rsl10 debug, with passed keys",
		.usage   = "key1 key2 key3 key4",
	},
	{
		.name    = "mass_erase",
		.handler = rsl10_mass_erase_command,
		.mode    = COMMAND_EXEC,
		.help    = "Mass erase all unprotected flash areas",
		.usage   = "",
	},
	COMMAND_REGISTRATION_DONE};

static const struct command_registration rsl10_command_handlers[] = {
	{
		.name  = "rsl10",
		.mode  = COMMAND_ANY,
		.help  = "rsl10 flash command group",
		.usage = "",
		.chain = rsl10_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE};

const struct flash_driver rsl10_flash = {
	.name               = "rsl10",
	.commands           = rsl10_command_handlers,
	.flash_bank_command = rsl10_flash_bank_command,
	.erase              = rsl10_erase,
	.protect            = rsl10_protect,
	.write              = rsl10_write,
	.read               = default_flash_read,
	.probe              = rsl10_probe,
	.auto_probe         = rsl10_auto_probe,
	.erase_check        = default_flash_blank_check,
	.protect_check      = rsl10_protect_check,
	.free_driver_priv   = rsl10_free_driver_priv,
};
