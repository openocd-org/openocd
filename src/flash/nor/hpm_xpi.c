// SPDX-License-Identifier: BSD-3-Clause
/*
 * Copyright (c) 2025 hpmicro
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include "imp.h"
#include <helper/bits.h>
#include <helper/binarybuffer.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/image.h>
#include "target/riscv/program.h"

#include "../../../contrib/loaders/flash/hpmicro/hpm_xpi_flash.h"
static uint8_t flash_algo[] = {
#include "../../../contrib/loaders/flash/hpmicro/hpm_xpi_flash.inc"
};
#define TIMEOUT_IN_MS (10000U)
#define ERASE_CHIP_TIMEOUT_IN_MS (100000U)
#define SECTOR_ERASE_TIMEOUT_IN_MS (100)
#define TYPICAL_TIMEOUT_IN_MS (500U)
#define BLOCK_SIZE (4096U)
#define NOR_CFG_OPT_HEADER (0xFCF90000UL)

struct hpm_flash_info {
	uint32_t total_sz_in_bytes;
	uint32_t sector_sz_in_bytes;
};

struct hpm_xpi_priv {
	uint32_t io_base;
	uint32_t header;
	uint32_t opt0;
	uint32_t opt1;
	bool probed;
};

static int hpm_xpi_run_algo_flash_init(struct flash_bank *bank,
									target_addr_t algo_entry)
{
	struct reg_param reg_params[6];
	struct target *target = bank->target;
	struct hpm_xpi_priv *xpi_priv = bank->driver_priv;

	int xlen = riscv_xlen(target);
	init_reg_param(&reg_params[0], "a0", xlen, PARAM_IN_OUT);
	init_reg_param(&reg_params[1], "a1", xlen, PARAM_OUT);
	init_reg_param(&reg_params[2], "a2", xlen, PARAM_OUT);
	init_reg_param(&reg_params[3], "a3", xlen, PARAM_OUT);
	init_reg_param(&reg_params[4], "a4", xlen, PARAM_OUT);
	init_reg_param(&reg_params[5], "ra", xlen, PARAM_OUT);
	buf_set_u64(reg_params[0].value, 0, xlen, bank->base);
	buf_set_u64(reg_params[1].value, 0, xlen, xpi_priv->header);
	buf_set_u64(reg_params[2].value, 0, xlen, xpi_priv->opt0);
	buf_set_u64(reg_params[3].value, 0, xlen, xpi_priv->opt1);
	buf_set_u64(reg_params[4].value, 0, xlen, xpi_priv->io_base);
	buf_set_u64(reg_params[5].value, 0, xlen, algo_entry + FLASH_INIT + 4);
	int retval = target_run_algorithm(target, 0, NULL, ARRAY_SIZE(reg_params), reg_params,
			algo_entry, algo_entry + FLASH_INIT + 4, TYPICAL_TIMEOUT_IN_MS, NULL);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to execute run algorithm: %d", retval);
		goto err;
	}

	uint32_t stat = buf_get_u32(reg_params[0].value, 0, xlen);
	if (stat) {
		retval = ERROR_TARGET_FAILURE;
		LOG_ERROR("init flash failed on target: 0x%" PRIx32, retval);
		goto err;
	}

err:
	for (size_t k = 0; k < ARRAY_SIZE(reg_params); k++)
		destroy_reg_param(&reg_params[k]);

	return retval;
}

static int hpm_xpi_probe(struct flash_bank *bank)
{
	struct hpm_flash_info flash_info = {0};
	struct working_area *data_wa = NULL;
	struct flash_sector *sectors = NULL;
	struct working_area *wa;
	struct target *target = bank->target;

	struct hpm_xpi_priv *xpi_priv = bank->driver_priv;

	if (xpi_priv->probed) {
		xpi_priv->probed = false;
		bank->size = 0;
		bank->num_sectors = 0;
		free(bank->sectors);
		bank->sectors = NULL;
	}

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval = target_alloc_working_area(target, sizeof(flash_algo), &wa);
	if (retval != ERROR_OK) {
		LOG_WARNING("Couldn't allocate %zd-byte working area",
					sizeof(flash_algo));
		return retval;
	}

	retval = target_write_buffer(target, wa->address,
			sizeof(flash_algo), flash_algo);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write code to 0x%" TARGET_PRIxADDR ": %d",
				wa->address, retval);
		target_free_working_area(target, wa);
		return retval;
	}

	retval = hpm_xpi_run_algo_flash_init(bank, wa->address);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to run init flash algorithm: %d", retval);
		return retval;
	}

	if (target_alloc_working_area(target, sizeof(flash_info),
					&data_wa) != ERROR_OK) {
		LOG_WARNING("Couldn't allocate %zd-byte working area",
					sizeof(flash_info));
		goto err;
	}

	int xlen = riscv_xlen(target);
	struct reg_param reg_params[3];
	init_reg_param(&reg_params[0], "a0", xlen, PARAM_IN_OUT);
	init_reg_param(&reg_params[1], "a1", xlen, PARAM_OUT);
	init_reg_param(&reg_params[2], "ra", xlen, PARAM_OUT);
	buf_set_u64(reg_params[0].value, 0, xlen, bank->base);
	buf_set_u64(reg_params[1].value, 0, xlen, data_wa->address);
	buf_set_u64(reg_params[2].value, 0, xlen, wa->address + FLASH_GET_INFO + 4);

	retval = target_run_algorithm(target, 0, NULL, ARRAY_SIZE(reg_params), reg_params,
			wa->address + FLASH_GET_INFO, wa->address + FLASH_GET_INFO + 4, TYPICAL_TIMEOUT_IN_MS, NULL);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to run algorithm at: %d", retval);
		goto err;
	}

	uint32_t stat = buf_get_u32(reg_params[0].value, 0, xlen);
	if (stat) {
		retval = ERROR_TARGET_FAILURE;
		LOG_ERROR("flash get info failed on target: 0x%" PRIx32, retval);
		goto err;
	}

	target_read_u32(target, data_wa->address, &flash_info.total_sz_in_bytes);
	target_read_u32(target, data_wa->address + 4, &flash_info.sector_sz_in_bytes);

	bank->size = flash_info.total_sz_in_bytes;
	bank->num_sectors = flash_info.total_sz_in_bytes / flash_info.sector_sz_in_bytes;

	/* create and fill sectors array */
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (!sectors) {
		LOG_ERROR("not enough memory");
		retval = ERROR_FAIL;
		goto err;
	}

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * (flash_info.sector_sz_in_bytes);
		sectors[sector].size =  flash_info.sector_sz_in_bytes;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;
	xpi_priv->probed = true;

err:
	for (size_t k = 0; k < ARRAY_SIZE(reg_params); k++)
		destroy_reg_param(&reg_params[k]);
	target_free_working_area(target, data_wa);
	target_free_working_area(target, wa);
	return retval;
}

static int hpm_xpi_auto_probe(struct flash_bank *bank)
{
	struct hpm_xpi_priv *xpi_priv = bank->driver_priv;
	if (xpi_priv->probed)
		return ERROR_OK;
	return hpm_xpi_probe(bank);
}

static int hpm_xpi_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct working_area *data_wa = NULL;
	struct working_area *wa = NULL;
	uint32_t data_size = BLOCK_SIZE;
	uint32_t left = count, i = 0;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval = target_alloc_working_area(target, sizeof(flash_algo), &wa);
	if (retval != ERROR_OK) {
		LOG_WARNING("Couldn't allocate %zd-byte working area",
					sizeof(flash_algo));
		return retval;
	}

	retval = target_write_buffer(target, wa->address,
			sizeof(flash_algo), flash_algo);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write code to 0x%" TARGET_PRIxADDR ": %d",
				wa->address, retval);
		target_free_working_area(target, wa);
		return retval;
	}

	retval = hpm_xpi_run_algo_flash_init(bank, wa->address);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to run init flash algorithm: %d", retval);
		return retval;
	}

	/* memory buffer */
	uint32_t avail_buffer_size;
	avail_buffer_size = target_get_working_area_avail(target);
	if (avail_buffer_size <= 256) {
		/* we already allocated the writing code, but failed to get a
		 * buffer, free the algorithm */
		target_free_working_area(target, wa);
		LOG_WARNING("no large enough working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	data_size = MIN(data_size, avail_buffer_size);
	retval = target_alloc_working_area(target, data_size, &data_wa);
	if (retval != ERROR_OK) {
		LOG_WARNING("Couldn't allocate %d-byte working area", data_size);
		return retval;
	}

	int xlen = riscv_xlen(target);
	struct reg_param reg_params[4];
	init_reg_param(&reg_params[0], "a0", xlen, PARAM_IN_OUT);
	init_reg_param(&reg_params[1], "a1", xlen, PARAM_OUT);
	init_reg_param(&reg_params[2], "a2", xlen, PARAM_OUT);
	init_reg_param(&reg_params[3], "a3", xlen, PARAM_OUT);

	while (left) {
		uint32_t trans_size = MIN(data_size, left);
		retval = target_write_buffer(target, data_wa->address, trans_size, buffer + i * data_size);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to write buffer to 0x%" TARGET_PRIxADDR ": %d", data_wa->address, retval);
			goto err;
		}

		buf_set_u32(reg_params[0].value, 0, xlen, bank->base);
		buf_set_u32(reg_params[1].value, 0, xlen, offset + i * data_size);
		buf_set_u32(reg_params[2].value, 0, xlen, data_wa->address);
		buf_set_u32(reg_params[3].value, 0, xlen, trans_size);

		retval = target_run_algorithm(target, 0, NULL, 4, reg_params,
				wa->address + FLASH_PROGRAM, wa->address + FLASH_PROGRAM + 4, TIMEOUT_IN_MS, NULL);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to execute algorithm at 0x%" TARGET_PRIxADDR ": %d", wa->address, retval);
			goto err;
		}

		uint32_t stat = buf_get_u32(reg_params[0].value, 0, xlen);
		if (stat) {
			retval = ERROR_FLASH_OPERATION_FAILED;
			LOG_ERROR("flash write failed on target: 0x%" PRIx32, retval);
			goto err;
		}
		i++;
		left -= trans_size;
	}

err:
	if (data_wa)
		target_free_working_area(target, data_wa);

	for (size_t k = 0; k < ARRAY_SIZE(reg_params); k++)
		destroy_reg_param(&reg_params[k]);

	target_free_working_area(target, wa);
	return retval;
}

static int hpm_xpi_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct target *target = bank->target;
	struct working_area *wa = NULL;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval = target_alloc_working_area(target, sizeof(flash_algo), &wa);
	if (retval != ERROR_OK) {
		LOG_WARNING("Couldn't allocate %zd-byte working area",
					sizeof(flash_algo));
		return retval;
	}

	retval = target_write_buffer(target, wa->address,
			sizeof(flash_algo), flash_algo);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write code to 0x%" TARGET_PRIxADDR ": %d",
				wa->address, retval);
		target_free_working_area(target, wa);
		return retval;
	}

	retval = hpm_xpi_run_algo_flash_init(bank, wa->address);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to run init flash algorithm: %d", retval);
		return retval;
	}

	LOG_DEBUG("from sector %u to sector %u", first, last);

	int xlen = riscv_xlen(target);
	struct reg_param reg_params[3];
	init_reg_param(&reg_params[0], "a0", xlen, PARAM_IN_OUT);
	init_reg_param(&reg_params[1], "a1", xlen, PARAM_OUT);
	init_reg_param(&reg_params[2], "a2", xlen, PARAM_OUT);

	buf_set_u32(reg_params[0].value, 0, xlen, bank->base);
	buf_set_u32(reg_params[1].value, 0, xlen, first * bank->sectors[0].size);
	buf_set_u32(reg_params[2].value, 0, xlen, (last - first + 1) * bank->sectors[0].size);

	retval = target_run_algorithm(target, 0, NULL, ARRAY_SIZE(reg_params), reg_params,
			wa->address + FLASH_ERASE, wa->address + FLASH_ERASE + 4,
			SECTOR_ERASE_TIMEOUT_IN_MS * (last - first + 1), NULL);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to execute algorithm at 0x%" TARGET_PRIxADDR ": %d", wa->address, retval);
		goto err;
	}

	uint32_t stat = buf_get_u32(reg_params[0].value, 0, xlen);
	if (stat) {
		retval = ERROR_FLASH_OPERATION_FAILED;
		LOG_ERROR("flash erase failed on target: 0x%" PRIx32, retval);
		goto err;
	}

err:
	target_free_working_area(target, wa);
	for (size_t k = 0; k < ARRAY_SIZE(reg_params); k++)
		destroy_reg_param(&reg_params[k]);
	return retval;
}

static int hpm_xpi_erase_chip(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct working_area *wa = NULL;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval = target_alloc_working_area(target, sizeof(flash_algo), &wa);
	if (retval != ERROR_OK) {
		LOG_WARNING("Couldn't allocate %zd-byte working area",
					sizeof(flash_algo));
		return retval;
	}

	retval = target_write_buffer(target, wa->address,
			sizeof(flash_algo), flash_algo);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write code to 0x%" TARGET_PRIxADDR ": %d",
				wa->address, retval);
		target_free_working_area(target, wa);
		return retval;
	}

	retval = hpm_xpi_run_algo_flash_init(bank, wa->address);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to run init flash algorithm: %d", retval);
		return retval;
	}

	int xlen = riscv_xlen(target);
	struct reg_param reg_params[1];
	init_reg_param(&reg_params[0], "a0", xlen, PARAM_IN_OUT);
	buf_set_u64(reg_params[0].value, 0, xlen, bank->base);

	retval = target_run_algorithm(target, 0, NULL, ARRAY_SIZE(reg_params), reg_params,
			wa->address + FLASH_ERASE_CHIP, wa->address + FLASH_ERASE_CHIP + 4, ERASE_CHIP_TIMEOUT_IN_MS, NULL);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to execute algorithm at 0x%" TARGET_PRIxADDR ": %d", wa->address, retval);
		goto err;
	}

	uint32_t stat = buf_get_u32(reg_params[0].value, 0, xlen);
	if (stat) {
		retval = ERROR_FLASH_OPERATION_FAILED;
		LOG_ERROR("flash erase chip failed on target: 0x%" PRIx32, retval);
		goto err;
	}

err:
	target_free_working_area(target, wa);
	for (size_t k = 0; k < ARRAY_SIZE(reg_params); k++)
		destroy_reg_param(&reg_params[k]);
	return retval;
}

COMMAND_HANDLER(hpm_xpi_handle_erase_chip_command)
{
	int retval;
	struct flash_bank *bank;
	struct target *target;
	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	target = bank->target;
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	return hpm_xpi_erase_chip(bank);
}

static const struct command_registration hpm_xpi_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = hpm_xpi_handle_erase_chip_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "erase entire flash device",
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration hpm_xpi_command_handlers[] = {
	{
		.name = "hpm_xpi",
		.mode = COMMAND_ANY,
		.help = "hpm_xpi command group",
		.usage = "",
		.chain = hpm_xpi_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

FLASH_BANK_COMMAND_HANDLER(hpm_xpi_flash_bank_command)
{
	struct hpm_xpi_priv *xpi_priv;
	uint32_t io_base;
	uint32_t header;
	uint32_t opt0;
	uint32_t opt1;

	if (CMD_ARGC < 7)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], io_base);

	switch (CMD_ARGC) {
	case 7:
		header = NOR_CFG_OPT_HEADER + 1;
		opt1 = 0;
		opt0 = 7;
		break;
	case 8:
		header = NOR_CFG_OPT_HEADER + 1;
		opt1 = 0;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[7], opt0);
		break;
	case 9:
		header = NOR_CFG_OPT_HEADER + 2;
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[7], opt0);
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[8], opt1);
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	xpi_priv = malloc(sizeof(struct hpm_xpi_priv));
	if (!xpi_priv) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	bank->driver_priv = xpi_priv;
	xpi_priv->io_base = io_base;
	xpi_priv->header = header;
	xpi_priv->opt0 = opt0;
	xpi_priv->opt1 = opt1;
	xpi_priv->probed = false;

	return ERROR_OK;
}

const struct flash_driver hpm_xpi_flash = {
	.name = "hpm_xpi",
	.flash_bank_command = hpm_xpi_flash_bank_command,
	.commands = hpm_xpi_command_handlers,
	.erase = hpm_xpi_erase,
	.write = hpm_xpi_write,
	.read = default_flash_read,
	.verify = default_flash_verify,
	.probe = hpm_xpi_probe,
	.auto_probe = hpm_xpi_auto_probe,
	.erase_check = default_flash_blank_check,
	.free_driver_priv = default_flash_free_driver_priv,
};
