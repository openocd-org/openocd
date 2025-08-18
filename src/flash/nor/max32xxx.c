// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2016 by Maxim Integrated                                *
 *   Copyright (C) 2025 Analog Devices, Inc.                               *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <target/target.h>
#include <target/target_type.h>

// Register addresses
#define FLC_ADDR				0x00000000
#define FLC_CLKDIV				0x00000004
#define FLC_CN					0x00000008
#define FLC_PR1E_ADDR			0x0000000C
#define FLC_PR2S_ADDR			0x00000010
#define FLC_PR2E_ADDR			0x00000014
#define FLC_PR3S_ADDR			0x00000018
#define FLC_PR3E_ADDR			0x0000001C
#define FLC_MD					0x00000020
#define FLC_INT					0x00000024
#define FLC_DATA0				0x00000030
#define FLC_DATA1				0x00000034
#define FLC_DATA2				0x00000038
#define FLC_DATA3				0x0000003C
#define FLC_BL_CTRL				0x00000170
#define FLC_PROT				0x00000300

#define ARM_PID_REG				0xE00FFFE0
#define MAX326XX_ID_REG			0x40000838

// Register settings
#define FLC_INT_AF				0x00000002

#define FLC_CN_UNLOCK_MASK		0xF0000000
#define FLC_CN_UNLOCK_VALUE		0x20000000

#define FLC_CN_PEND				0x01000000
#define FLC_CN_ERASE_CODE_MASK	0x0000FF00
#define FLC_CN_ERASE_CODE_PGE	0x00005500
#define FLC_CN_ERASE_CODE_ME	0x0000AA00
#define FLC_CN_32BIT			0x00000010
#define FLC_CN_PGE				0x00000004
#define FLC_CN_ME				0x00000002
#define FLC_CN_WR				0x00000001
#define FLC_CN_PGE				0x00000004
#define FLC_CN_ME				0x00000002
#define FLC_CN_WR				0x00000001

#define FLC_BL_CTRL_23			0x00020000
#define FLC_BL_CTRL_IFREN		0x00000001

#define MASK_FLASH_BUSY			(0x048800E0 & ~0x04000000)
#define MASK_DISABLE_INTS		(0xFFFFFCFC)
#define MASK_FLASH_UNLOCKED		(0xF588FFEF & ~0x04000000)
#define MASK_FLASH_LOCK			(0xF588FFEF & ~0x04000000)
#define MASK_FLASH_ERASE		(0xF588FFEF & ~0x04000000)
#define MASK_FLASH_ERASED		(0xF48800EB & ~0x04000000)
#define MASK_ACCESS_VIOLATIONS	(0xFFFFFCFC)
#define MASK_FLASH_WRITE		(0xF588FFEF & ~0x04000000)
#define MASK_WRITE_ALIGNED		(0xF588FFEF & ~0x04000000)
#define MASK_WRITE_COMPLETE		(0xF488FFEE & ~0x04000000)
#define MASK_WRITE_BURST		(0xF588FFEF & ~0x04000000)
#define MASK_BURST_COMPLETE		(0xF488FFEE & ~0x04000000)
#define MASK_WRITE_REMAINING	(0xF588FFEF & ~0x04000000)
#define MASK_REMAINING_COMPLETE	(0xF488FFEE & ~0x04000000)
#define MASK_MASS_ERASE			(0xF588FFEF & ~0x04000000)
#define MASK_ERASE_COMPLETE		(0xF48800ED & ~0x04000000)

#define ARM_PID_DEFAULT_CM3		0x0000B4C3
#define ARM_PID_DEFAULT_CM4		0x0000B4C4
#define MAX326XX_ID				0x0000004D

#define OPTIONS_128             0x01 // Perform 128 bit flash writes
#define OPTIONS_ENC             0x02 // Encrypt the flash contents
#define OPTIONS_AUTH            0x04 // Authenticate the flash contents
#define OPTIONS_COUNT           0x08 // Add counter values to authentication
#define OPTIONS_INTER           0x10 // Interleave the authentication and count values
#define OPTIONS_RELATIVE_XOR    0x20 // Only XOR the offset of the address when encrypting
#define OPTIONS_KEYSIZE         0x40 // Use a 256 bit KEY

static int max32xxx_mass_erase(struct flash_bank *bank);

struct max32xxx_flash_bank {
	bool probed;
	bool max326xx;
	unsigned int flash_size;
	unsigned int flc_base;
	unsigned int sector_size;
	unsigned int clkdiv_value;
	unsigned int int_state;
	unsigned int options;
};

static const uint8_t write_code_arm[] = {
#include "../../../contrib/loaders/flash/max32xxx/max32xxx_write_arm.inc"
};

FLASH_BANK_COMMAND_HANDLER(max32xxx_flash_bank_command)
{
	struct max32xxx_flash_bank *info;

	if (CMD_ARGC != 10) {
		LOG_ERROR("incorrect flash bank max32xxx configuration: <base> <size> 0 0 <target> <FLC base> <sector size> <clkdiv> <options>");
		return ERROR_FLASH_BANK_INVALID;
	}

	info = calloc(1, sizeof(struct max32xxx_flash_bank));
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], info->flash_size);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], info->flc_base);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[7], info->sector_size);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[8], info->clkdiv_value);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[9], info->options);

	info->int_state = 0;
	bank->driver_priv = info;
	return ERROR_OK;
}

static int get_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct max32xxx_flash_bank *info = bank->driver_priv;

	if (!info->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	command_print_sameline(cmd, "\nMaxim Integrated max32xxx flash driver\n");
	return ERROR_OK;
}

static bool max32xxx_flash_busy(uint32_t flash_cn)
{
	if (flash_cn & (FLC_CN_PGE | FLC_CN_ME | FLC_CN_WR))
		return true;

	return false;
}

static int max32xxx_flash_op_pre(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct max32xxx_flash_bank *info = bank->driver_priv;
	uint32_t flash_cn;
	uint32_t bootloader;

	// Check if the flash controller is busy
	target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
	if (max32xxx_flash_busy(flash_cn))
		return ERROR_FLASH_BUSY;

	// Refresh flash controller timing
	target_write_u32(target, info->flc_base + FLC_CLKDIV, info->clkdiv_value);

	// Clear and disable flash programming interrupts
	target_read_u32(target, info->flc_base + FLC_INT, &info->int_state);
	target_write_u32(target, info->flc_base + FLC_INT, 0);

	/* Clear the lower bit in the bootloader configuration register in case flash page 0 has
	 * been replaced */
	if (target_read_u32(target, info->flc_base + FLC_BL_CTRL, &bootloader) != ERROR_OK) {
		LOG_ERROR("Read failure on FLC_BL_CTRL");
		return ERROR_FAIL;
	}
	if (bootloader & FLC_BL_CTRL_23) {
		LOG_WARNING("FLC_BL_CTRL indicates BL mode 2 or mode 3.");
		if (bootloader & FLC_BL_CTRL_IFREN) {
			LOG_WARNING("Flash page 0 swapped out, attempting to swap back in for programming");
			bootloader &= ~(FLC_BL_CTRL_IFREN);
			if (target_write_u32(target, info->flc_base + FLC_BL_CTRL,
					bootloader) != ERROR_OK) {
				LOG_ERROR("Write failure on FLC_BL_CTRL");
				return ERROR_FAIL;
			}
			if (target_read_u32(target, info->flc_base + FLC_BL_CTRL,
					&bootloader) != ERROR_OK) {
				LOG_ERROR("Read failure on FLC_BL_CTRL");
				return ERROR_FAIL;
			}
			if (bootloader & FLC_BL_CTRL_IFREN)
				LOG_ERROR("Unable to swap flash page 0 back in. Writes to page 0 will fail.");
		}
	}

	// Unlock flash
	flash_cn &= ~(FLC_CN_UNLOCK_MASK);
	flash_cn |= FLC_CN_UNLOCK_VALUE;
	target_write_u32(target, info->flc_base + FLC_CN, flash_cn);

	// Confirm flash is unlocked
	target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
	if ((flash_cn & FLC_CN_UNLOCK_VALUE) != FLC_CN_UNLOCK_VALUE)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int max32xxx_flash_op_post(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct max32xxx_flash_bank *info = bank->driver_priv;
	uint32_t flash_cn;

	// Restore flash programming interrupts
	target_write_u32(target, info->flc_base + FLC_INT, info->int_state);

	// Lock flash
	target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
	flash_cn &= ~(FLC_CN_UNLOCK_MASK);
	target_write_u32(target, info->flc_base + FLC_CN, flash_cn);
	return ERROR_OK;
}

static int max32xxx_protect_check(struct flash_bank *bank)
{
	struct max32xxx_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t temp_reg;

	if (!info->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (!info->max326xx) {
		for (unsigned int i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_protected = -1;

		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	// Check the protection
	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		if (i % 32 == 0)
			target_read_u32(target, info->flc_base + FLC_PROT + ((i / 32) * 4), &temp_reg);

		if (temp_reg & (0x1 << i % 32))
			bank->sectors[i].is_protected = 1;
		else
			bank->sectors[i].is_protected = 0;
	}
	return ERROR_OK;
}

static int max32xxx_erase(struct flash_bank *bank, unsigned int first,
	unsigned int last)
{
	uint32_t flash_cn, flash_int;
	struct max32xxx_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	int retval;
	int retry;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!info->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (last < first || last >= bank->num_sectors)
		return ERROR_FLASH_SECTOR_INVALID;

	if (first == 0 && last == (bank->num_sectors - 1))
		return max32xxx_mass_erase(bank);

	// Prepare to issue flash operation
	retval = max32xxx_flash_op_pre(bank);

	if (retval != ERROR_OK)
		return retval;

	int erased = 0;
	for (unsigned int banknr = first; banknr <= last; banknr++) {
		// Check the protection
		if (bank->sectors[banknr].is_protected == 1) {
			LOG_WARNING("Flash sector %u is protected", banknr);
			continue;
		} else {
			erased = 1;
		}

		// Address is first word in page
		target_write_u32(target, info->flc_base + FLC_ADDR, banknr * info->sector_size);

		// Write page erase code
		target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
		flash_cn |= FLC_CN_ERASE_CODE_PGE;
		target_write_u32(target, info->flc_base + FLC_CN, flash_cn);

		// Issue page erase command
		flash_cn |= FLC_CN_PGE;
		target_write_u32(target, info->flc_base + FLC_CN, flash_cn);

		// Wait until erase complete
		retry = 1000;
		do {
			target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
		} while ((--retry > 0) && max32xxx_flash_busy(flash_cn));

		if (retry <= 0) {
			LOG_ERROR("Timed out waiting for flash page erase @ 0x%08" PRIx32,
				(banknr * info->sector_size));
			return ERROR_FLASH_OPERATION_FAILED;
		}

		// Check access violations
		target_read_u32(target, info->flc_base + FLC_INT, &flash_int);
		if (flash_int & FLC_INT_AF) {
			LOG_ERROR("Error erasing flash page %i", banknr);
			target_write_u32(target, info->flc_base + FLC_INT, 0);
			max32xxx_flash_op_post(bank);
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}

	if (!erased) {
		LOG_ERROR("All pages protected %u to %u", first, last);
		max32xxx_flash_op_post(bank);
		return ERROR_FAIL;
	}

	if (max32xxx_flash_op_post(bank) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int max32xxx_protect(struct flash_bank *bank, int set,
	unsigned int first, unsigned int last)
{
	struct max32xxx_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t temp_reg;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!info->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if (!info->max326xx)
		return ERROR_FLASH_OPER_UNSUPPORTED;

	if (last < first || last >= bank->num_sectors)
		return ERROR_FLASH_SECTOR_INVALID;

	// Setup the protection on the pages given
	for (unsigned int page = first; page <= last; page++) {
		if (set) {
			// Set the write/erase bit for this page
			target_read_u32(target, info->flc_base + FLC_PROT + (page / 32), &temp_reg);
			temp_reg |= (0x1 << page % 32);
			target_write_u32(target, info->flc_base + FLC_PROT + (page / 32), temp_reg);
			bank->sectors[page].is_protected = 1;
		} else {
			// Clear the write/erase bit for this page
			target_read_u32(target, info->flc_base + FLC_PROT + (page / 32), &temp_reg);
			temp_reg &= ~(0x1 << page % 32);
			target_write_u32(target, info->flc_base + FLC_PROT + (page / 32), temp_reg);
			bank->sectors[page].is_protected = 0;
		}
	}

	return ERROR_OK;
}

static int max32xxx_write_block(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t len)
{
	struct max32xxx_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	const char *target_type_name = (const char *)target->type->name;
	uint32_t buffer_size = 16384;
	struct working_area *source;
	struct working_area *write_algorithm;
	struct reg_param reg_params[5];
	struct mem_param mem_param[2];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;
	// power of two, and multiple of word size
	static const unsigned int buf_min = 128;
	uint8_t *write_code;
	int write_code_size;


	if (strcmp(target_type_name, "cortex_m") == 0) {
		write_code = (uint8_t *)write_code_arm;
		write_code_size = sizeof(write_code_arm);
	} else {
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	LOG_DEBUG("bank=%p buffer=%p offset=%08" PRIx32 " len=%08" PRIx32 "",
		bank, buffer, offset, len);

	// flash write code
	if (target_alloc_working_area(target, write_code_size, &write_algorithm) != ERROR_OK) {
		LOG_DEBUG("no working area for block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	// memory buffer
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;

		if (buffer_size <= buf_min) {
			target_free_working_area(target, write_algorithm);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}

		LOG_DEBUG("retry target_alloc_working_area(%s, size=%" PRIu32 ")",
			target_name(target), buffer_size);
	}

	target_write_buffer(target, write_algorithm->address, write_code_size,
		write_code);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	// TODO: a0-a3 for RISCV

	if (strcmp(target_type_name, "cortex_m") == 0) {
		init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
		init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
		init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
		init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);
	} else {
		init_reg_param(&reg_params[0], "a0", 32, PARAM_OUT);
		init_reg_param(&reg_params[1], "a1", 32, PARAM_OUT);
		init_reg_param(&reg_params[2], "a2", 32, PARAM_OUT);
		init_reg_param(&reg_params[3], "a3", 32, PARAM_OUT);
	}
	init_reg_param(&reg_params[4], "sp", 32, PARAM_OUT);

	buf_set_u32(reg_params[0].value, 0, 32, source->address);
	buf_set_u32(reg_params[1].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[2].value, 0, 32, len);
	buf_set_u32(reg_params[3].value, 0, 32, offset);
	buf_set_u32(reg_params[4].value, 0, 32, source->address + source->size);

	// mem_params for options
	init_mem_param(&mem_param[0], source->address + (source->size - 8 - 128), 4, PARAM_OUT);
	init_mem_param(&mem_param[1], source->address + (source->size - 4 - 128), 4, PARAM_OUT);
	buf_set_u32(mem_param[0].value, 0, 32, info->options);
	buf_set_u32(mem_param[1].value, 0, 32, info->flc_base);

	// leave room for stack, 32-bit options and encryption buffer
	retval = target_run_flash_async_algorithm(target,
			buffer,
			len,
			1,
			2,
			mem_param,
			5,
			reg_params,
			source->address,
			(source->size - 8 - 256),
			write_algorithm->address,
			0,
			&armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED)
		LOG_ERROR("error %d executing max32xxx flash write algorithm", retval);

	target_free_working_area(target, write_algorithm);
	target_free_working_area(target, source);
	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	return retval;
}

static int max32xxx_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct max32xxx_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t flash_cn, flash_int;
	uint32_t address = offset;
	uint32_t remaining = count;
	int retval;
	int retry;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("bank=%p buffer=%p offset=%08" PRIx32 " count=%08" PRIx32 "",
		bank, buffer, offset, count);

	if (!info->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	if ((info->options & OPTIONS_128) == 0) {
		if (offset & 0x3) {
			LOG_ERROR("offset size must be 32-bit aligned");
			return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
		}
	} else {
		if (offset & 0xF) {
			LOG_ERROR("offset size must be 128-bit aligned");
			return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
		}
	}

	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;

	// Prepare to issue flash operation
	retval = max32xxx_flash_op_pre(bank);

	if (retval != ERROR_OK) {
		max32xxx_flash_op_post(bank);
		return retval;
	}

	if (remaining >= 16) {
		// try using a block write

		retval = max32xxx_write_block(bank, buffer, offset, remaining);
		if (retval != ERROR_OK) {
			if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
				if (info->options & OPTIONS_ENC) {
					LOG_ERROR("Must use algorithm in working area for encryption");
					return ERROR_FLASH_OPERATION_FAILED;
				}
				LOG_DEBUG("writing flash word-at-a-time");
			} else {
				max32xxx_flash_op_post(bank);
				return ERROR_FLASH_OPERATION_FAILED;
			}
		} else {
			// all words_remaining have been written
			buffer += remaining;
			address += remaining;
			remaining -= remaining;
		}
	}

	if (((info->options & OPTIONS_128) == 0) && remaining >= 4) {
		// write in 32-bit units
		target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
		flash_cn |= FLC_CN_32BIT;
		target_write_u32(target, info->flc_base + FLC_CN, flash_cn);

		while (remaining >= 4) {
			target_write_u32(target, info->flc_base + FLC_ADDR, address);
			target_write_buffer(target, info->flc_base + FLC_DATA0, 4, buffer);
			flash_cn |= FLC_CN_WR;
			target_write_u32(target, info->flc_base + FLC_CN, flash_cn);
			// Wait until flash operation is complete
			retry = 10;

			do {
				target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
			} while ((--retry > 0) && max32xxx_flash_busy(flash_cn));

			if (retry <= 0) {
				LOG_ERROR("Timed out waiting for flash write @ 0x%08" PRIx32,
					address);
				max32xxx_flash_op_post(bank);
				return ERROR_FLASH_OPERATION_FAILED;
			}

			buffer += 4;
			address += 4;
			remaining -= 4;
		}
	}

	if ((info->options & OPTIONS_128) && remaining >= 16) {
		// write in 128-bit units
		target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
		flash_cn &= ~(FLC_CN_32BIT);
		target_write_u32(target, info->flc_base + FLC_CN, flash_cn);

		while (remaining >= 16) {
			target_write_u32(target, info->flc_base + FLC_ADDR, address);

			if ((address & 0xFFF) == 0)
				LOG_DEBUG("Writing @ 0x%08" PRIx32, address);

			target_write_buffer(target, info->flc_base + FLC_DATA0, 16, buffer);
			flash_cn |= FLC_CN_WR;
			target_write_u32(target, info->flc_base + FLC_CN, flash_cn);
			// Wait until flash operation is complete
			retry = 10;

			do {
				target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
			} while ((--retry > 0) && max32xxx_flash_busy(flash_cn));

			if (retry <= 0) {
				LOG_ERROR("Timed out waiting for flash write @ 0x%08" PRIx32,
					address);
				max32xxx_flash_op_post(bank);
				return ERROR_FLASH_OPERATION_FAILED;
			}

			buffer += 16;
			address += 16;
			remaining -= 16;
		}
	}

	if (((info->options & OPTIONS_128) == 0) && remaining > 0) {
		// write remaining bytes in a 32-bit unit
		target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
		flash_cn |= FLC_CN_32BIT;
		target_write_u32(target, info->flc_base + FLC_CN, flash_cn);

		uint8_t last_word[4] = {0xFF, 0xFF, 0xFF, 0xFF};
		uint32_t i = 0;

		while (remaining > 0) {
			last_word[i++] = *buffer;
			buffer++;
			remaining--;
		}

		target_write_u32(target, info->flc_base + FLC_ADDR, address);
		target_write_buffer(target, info->flc_base + FLC_DATA0, 4, last_word);
		flash_cn |= FLC_CN_WR;
		target_write_u32(target, info->flc_base + FLC_CN, flash_cn);

		// Wait until flash operation is complete
		retry = 10;

		do {
			target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
		} while ((--retry > 0) && max32xxx_flash_busy(flash_cn));

		if (retry <= 0) {
			LOG_ERROR("Timed out waiting for flash write @ 0x%08" PRIx32, address);
			max32xxx_flash_op_post(bank);
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}

	if ((info->options & OPTIONS_128) && remaining > 0) {
		// write remaining bytes in a 128-bit unit
		if (target_read_u32(target, info->flc_base + FLC_CN, &flash_cn) != ERROR_OK) {
			max32xxx_flash_op_post(bank);
			return ERROR_FAIL;
		}

		target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
		flash_cn &= ~(FLC_CN_32BIT);
		target_write_u32(target, info->flc_base + FLC_CN, flash_cn);

		uint8_t last_words[16] = {0xFF, 0xFF, 0xFF, 0xFF,
					  0xFF, 0xFF, 0xFF, 0xFF,
					  0xFF, 0xFF, 0xFF, 0xFF,
					  0xFF, 0xFF, 0xFF, 0xFF};

		uint32_t i = 0;

		while (remaining > 0) {
			last_words[i++] = *buffer;
			buffer++;
			remaining--;
		}

		target_write_u32(target, info->flc_base + FLC_ADDR, address);
		target_write_buffer(target, info->flc_base + FLC_DATA0, 4, last_words);
		target_write_buffer(target, info->flc_base + FLC_DATA0 + 4, 4, last_words + 4);
		target_write_buffer(target, info->flc_base + FLC_DATA0 + 8, 4, last_words + 8);
		target_write_buffer(target, info->flc_base + FLC_DATA0 + 12, 4, last_words + 12);
		flash_cn |= FLC_CN_WR;
		target_write_u32(target, info->flc_base + FLC_CN, flash_cn);

		// Wait until flash operation is complete
		retry = 10;
		do {
			if (target_read_u32(target, info->flc_base + FLC_CN,
					&flash_cn) != ERROR_OK) {
				max32xxx_flash_op_post(bank);
				return ERROR_FAIL;
			}
		} while ((--retry > 0) && (flash_cn & FLC_CN_PEND));

		if (retry <= 0) {
			LOG_ERROR("Timed out waiting for flash write @ 0x%08" PRIx32, address);
			max32xxx_flash_op_post(bank);
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}

	// Check access violations
	target_read_u32(target, info->flc_base + FLC_INT, &flash_int);
	if (flash_int & FLC_INT_AF) {
		LOG_ERROR("Flash Error writing 0x%" PRIx32 " bytes at 0x%08" PRIx32, count, offset);
		max32xxx_flash_op_post(bank);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	if (max32xxx_flash_op_post(bank) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int max32xxx_probe(struct flash_bank *bank)
{
	struct max32xxx_flash_bank *info = bank->driver_priv;
	struct target *target = bank->target;
	uint32_t arm_id[2];
	uint16_t arm_pid;

	free(bank->sectors);

	// provide this for the benefit of the NOR flash framework
	bank->size = info->flash_size;
	bank->num_sectors = info->flash_size / info->sector_size;
	bank->sectors = calloc(bank->num_sectors, sizeof(struct flash_sector));

	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].offset = i * info->sector_size;
		bank->sectors[i].size = info->sector_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
	}

	// Probe to determine if this part is in the max326xx family
	info->max326xx = false;
	target_read_u32(target, ARM_PID_REG, &arm_id[0]);
	target_read_u32(target, ARM_PID_REG + 4, &arm_id[1]);
	arm_pid = (arm_id[1] << 8) + arm_id[0];
	LOG_DEBUG("arm_pid = 0x%x", arm_pid);

	if (arm_pid == ARM_PID_DEFAULT_CM3 || arm_pid == ARM_PID_DEFAULT_CM4) {
		uint32_t max326xx_id;
		target_read_u32(target, MAX326XX_ID_REG, &max326xx_id);
		LOG_DEBUG("max326xx_id = 0x%" PRIx32, max326xx_id);
		max326xx_id = ((max326xx_id & 0xFF000000) >> 24);
		if (max326xx_id == MAX326XX_ID)
			info->max326xx = true;
	}
	LOG_DEBUG("info->max326xx = %d", info->max326xx);

	// Initialize the protection bits for each flash page
	if (max32xxx_protect_check(bank) == ERROR_FLASH_OPER_UNSUPPORTED)
		LOG_WARNING("Flash protection not supported on this device");

	info->probed = true;
	return ERROR_OK;
}

static int max32xxx_mass_erase(struct flash_bank *bank)
{
	struct target *target = NULL;
	struct max32xxx_flash_bank *info = NULL;
	uint32_t flash_cn, flash_int;
	int retval;
	int retry;
	info = bank->driver_priv;
	target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!info->probed)
		return ERROR_FLASH_BANK_NOT_PROBED;

	bool protected = true;
	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		if (bank->sectors[i].is_protected == 1)
			LOG_WARNING("Flash sector %u is protected", i);
		else
			protected = false;
	}

	if (protected) {
		LOG_ERROR("All pages protected");
		return ERROR_FAIL;
	}

	// Prepare to issue flash operation
	retval = max32xxx_flash_op_pre(bank);

	if (retval != ERROR_OK)
		return retval;

	// Write mass erase code
	target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
	flash_cn |= FLC_CN_ERASE_CODE_ME;
	target_write_u32(target, info->flc_base + FLC_CN, flash_cn);

	// Issue mass erase command
	flash_cn |= FLC_CN_ME;
	target_write_u32(target, info->flc_base + FLC_CN, flash_cn);

	// Wait until erase complete
	retry = 1000;
	do {
		target_read_u32(target, info->flc_base + FLC_CN, &flash_cn);
	} while ((--retry > 0) && max32xxx_flash_busy(flash_cn));

	if (retry <= 0) {
		LOG_ERROR("Timed out waiting for flash mass erase");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	// Check access violations
	target_read_u32(target, info->flc_base + FLC_INT, &flash_int);
	if (flash_int & FLC_INT_AF) {
		LOG_ERROR("Error mass erasing");
		target_write_u32(target, info->flc_base + FLC_INT, 0);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	if (max32xxx_flash_op_post(bank) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

COMMAND_HANDLER(max32xxx_handle_mass_erase_command)
{
	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);

	if (CMD_ARGC < 1) {
		command_print(CMD, "max32xxx mass_erase <bank>");
		return ERROR_OK;
	}

	if (retval != ERROR_OK)
		return retval;

	if (max32xxx_mass_erase(bank) == ERROR_OK)
		command_print(CMD, "max32xxx mass erase complete");
	else
		command_print(CMD, "max32xxx mass erase failed");

	return ERROR_OK;
}

COMMAND_HANDLER(max32xxx_handle_protection_set_command)
{
	struct flash_bank *bank;
	int retval;
	struct max32xxx_flash_bank *info;
	uint32_t addr, len;

	if (CMD_ARGC != 3) {
		command_print(CMD, "max32xxx protection_set <bank> <addr> <size>");
		return ERROR_OK;
	}

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;
	info = bank->driver_priv;

	// Convert the range to the page numbers
	if (sscanf(CMD_ARGV[1], "0x%" SCNx32, &addr) != 1) {
		LOG_WARNING("Error parsing address");
		command_print(CMD, "max32xxx protection_set <bank> <addr> <size>");
		return ERROR_FAIL;
	}
	// Mask off the top portion on the address
	addr = (addr & 0x0FFFFFFF);

	if (sscanf(CMD_ARGV[2], "0x%" SCNx32, &len) != 1) {
		LOG_WARNING("Error parsing length");
		command_print(CMD, "max32xxx protection_set <bank> <addr> <size>");
		return ERROR_FAIL;
	}

	// Check the address is in the range of the flash
	if ((addr + len) >= info->flash_size)
		return ERROR_FLASH_SECTOR_INVALID;

	if (len == 0)
		return ERROR_OK;

	// Convert the address and length to the page boundaries
	addr = addr - (addr % info->sector_size);
	if (len % info->sector_size)
		len = len + info->sector_size - (len % info->sector_size);

	// Convert the address and length to page numbers
	addr = (addr / info->sector_size);
	len = addr + (len / info->sector_size) - 1;

	if (max32xxx_protect(bank, 1, addr, len) == ERROR_OK)
		command_print(CMD, "max32xxx protection set complete");
	else
		command_print(CMD, "max32xxx protection set failed");

	return ERROR_OK;
}

COMMAND_HANDLER(max32xxx_handle_protection_clr_command)
{
	struct flash_bank *bank;
	int retval;
	struct max32xxx_flash_bank *info;
	uint32_t addr, len;

	if (CMD_ARGC != 3) {
		command_print(CMD, "max32xxx protection_clr <bank> <addr> <size>");
		return ERROR_OK;
	}

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;
	info = bank->driver_priv;

	// Convert the range to the page numbers
	if (sscanf(CMD_ARGV[1], "0x%" SCNx32, &addr) != 1) {
		LOG_WARNING("Error parsing address");
		command_print(CMD, "max32xxx protection_clr <bank> <addr> <size>");
		return ERROR_FAIL;
	}
	// Mask off the top portion on the address
	addr = (addr & 0x0FFFFFFF);

	if (sscanf(CMD_ARGV[2], "0x%" SCNx32, &len) != 1) {
		LOG_WARNING("Error parsing length");
		command_print(CMD, "max32xxx protection_clr <bank> <addr> <size>");
		return ERROR_FAIL;
	}

	// Check the address is in the range of the flash
	if ((addr + len) >= info->flash_size)
		return ERROR_FLASH_SECTOR_INVALID;

	if (len == 0)
		return ERROR_OK;

	// Convert the address and length to the page boundaries
	addr = addr - (addr % info->sector_size);
	if (len % info->sector_size)
		len = len + info->sector_size - (len % info->sector_size);

	// Convert the address and length to page numbers
	addr = (addr / info->sector_size);
	len = addr + (len / info->sector_size) - 1;

	if (max32xxx_protect(bank, 0, addr, len) == ERROR_OK)
		command_print(CMD, "max32xxx protection clear complete");
	else
		command_print(CMD, "max32xxx protection clear failed");

	return ERROR_OK;
}

COMMAND_HANDLER(max32xxx_handle_protection_check_command)
{
	struct flash_bank *bank;
	int retval;
	struct max32xxx_flash_bank *info;

	if (CMD_ARGC < 1) {
		command_print(CMD, "max32xxx protection_check <bank>");
		return ERROR_OK;
	}

	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;
	info = bank->driver_priv;

	// Update the protection array
	retval = max32xxx_protect_check(bank);
	if (retval != ERROR_OK) {
		LOG_WARNING("Error updating the protection array");
		return retval;
	}

	LOG_WARNING("s:<sector number> a:<address> p:<protection bit>");
	for (unsigned int i = 0; i < bank->num_sectors; i += 4) {
		LOG_WARNING("s:%03d a:0x%06x p:%d | s:%03d a:0x%06x p:%d | s:%03d a:0x%06x p:%d | s:%03d a:0x%06x p:%d",
		(i + 0), (i + 0) * info->sector_size, bank->sectors[(i + 0)].is_protected,
		(i + 1), (i + 1) * info->sector_size, bank->sectors[(i + 1)].is_protected,
		(i + 2), (i + 2) * info->sector_size, bank->sectors[(i + 2)].is_protected,
		(i + 3), (i + 3) * info->sector_size, bank->sectors[(i + 3)].is_protected);
	}

	return ERROR_OK;
}

static const struct command_registration max32xxx_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = max32xxx_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "mass erase flash",
	},
	{
		.name = "protection_set",
		.handler = max32xxx_handle_protection_set_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id addr size",
		.help = "set flash protection for address range",
	},
	{
		.name = "protection_clr",
		.handler = max32xxx_handle_protection_clr_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id addr size",
		.help = "clear flash protection for address range",
	},
	{
		.name = "protection_check",
		.handler = max32xxx_handle_protection_check_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "check flash protection",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration max32xxx_command_handlers[] = {
	{
		.name = "max32xxx",
		.mode = COMMAND_EXEC,
		.help = "max32xxx flash command group",
		.chain = max32xxx_exec_command_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver max32xxx_flash = {
	.name = "max32xxx",
	.commands = max32xxx_command_handlers,
	.flash_bank_command = max32xxx_flash_bank_command,
	.erase = max32xxx_erase,
	.protect = max32xxx_protect,
	.write = max32xxx_write,
	.read = default_flash_read,
	.probe = max32xxx_probe,
	.auto_probe = max32xxx_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = max32xxx_protect_check,
	.info = get_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
