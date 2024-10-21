// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2024 by Marek Kraus                                     *
 *   gamelaster@gami.ee                                                    *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <jtag/jtag.h>
#include <target/algorithm.h>
#include "spi.h"
#include "target/riscv/opcodes.h"
#include "target/riscv/gdb_regs.h"

/*
 * Flash bank driver for Bouffalo chips with BL602-like flash controller
 *
 * Supported chips (series): BL602, BL702 and BL702L
 * Not supported but compatible chips (series): BL616, BL606P and BL808
 *
 * SFlash supports both direct access and eXecute In Place (XIP) via L1 cache (L1C).
 * If XIP is configured and direct access is needed,
 * then the state needs to be saved and some settings need to be reset for it to work properly.
 * The only way to do this is to save/restore the entire SFlash core configuration, which has not been tested.
 * Because of this, on every OpenOCD flash probe, GPIO and SFlash will be reconfigured, resulting in XIP not working.
 * So it's recommended to reset the device after every OpenOCD flash operation.
 *
 * QSPI write is not supported yet.
 */

// SFlash CFG definitions
#define SFLASH_CFG_SIZE 84
#define SFLASH_CFG_BUSY_BIT_POS 0x2D
#define SFLASH_CFG_BUSY_INDEX_POS 0x2A
#define SFLASH_CFG_BUSY_READ_REG_LEN_POS 0x33
#define SFLASH_CFG_FAST_READ_CMD_POS 0x18
#define SFLASH_CFG_JEDEC_ID_CMD_DMY_CLK_POS 0x09
#define SFLASH_CFG_JEDEC_ID_CMD_POS 0x08
#define SFLASH_CFG_PAGE_PROGRAM_CMD_POS 0x15
#define SFLASH_CFG_PAGE_SIZE_POS 0x0E
#define SFLASH_CFG_READ_STATUS_REG1_POS 0x34
#define SFLASH_CFG_SECTOR_ERASE_CMD_POS 0x11
#define SFLASH_CFG_SECTOR_SIZE_POS 0x0C
#define SFLASH_CFG_TIME_ERASE_SECTOR_POS 0x48
#define SFLASH_CFG_TIME_PAGE_PGM_POS 0x4E
#define SFLASH_CFG_WRITE_ENABLE_BIT_POS 0x2B
#define SFLASH_CFG_WRITE_ENABLE_CMD_POS 0x14
#define SFLASH_CFG_WRITE_ENABLE_INDEX_POS 0x28
#define SFLASH_CFG_WRITE_ENABLE_READ_REG_LEN_POS 0x2F

#define BL602_DEFAULT_TIMEOUT_MS 5000

enum bflb_series {
	BFLB_SERIES_BL602,
	BFLB_SERIES_BL702,
	BFLB_SERIES_BL702L,
};

struct bl602_part_info {
	uint32_t idcode;
	enum bflb_series series;
	uint32_t romapi_get_jedec_id;
	uint32_t romapi_sflash_init_gpio;
	uint32_t romapi_sflash_init;
	uint32_t romapi_sflash_program;
	uint32_t romapi_sflash_read;
	uint32_t romapi_sflash_erase_sector;
};

struct bl602_flash_bank {
	// flag indicating successful flash probe
	bool probed;
	enum bflb_series series;
	const struct bl602_part_info *part_info;
	uint8_t sflash_cfg[SFLASH_CFG_SIZE];
	// detected model of SPI flash
	const struct flash_device *dev;
};

static const struct bl602_part_info bl602_parts[] = {
	{
		.idcode = 0x20000c05,
		.series = BFLB_SERIES_BL602,
		.romapi_get_jedec_id = 0x210109bc,
		.romapi_sflash_init_gpio = 0x21010a20,
		.romapi_sflash_init = 0x21010980,
		.romapi_sflash_program = 0x210109b4,
		.romapi_sflash_read = 0x210109fc,
		.romapi_sflash_erase_sector = 0x210109a4,
	},
	{
		.idcode = 0x20000e05,
		.series = BFLB_SERIES_BL702,
		.romapi_get_jedec_id = 0x210189a4,
		.romapi_sflash_init_gpio = 0x21018a78,
		.romapi_sflash_init = 0x21018960,
		.romapi_sflash_program = 0x2101899c,
		.romapi_sflash_read = 0x210189d8,
		.romapi_sflash_erase_sector = 0x2101898c,
	},
	{
		.idcode = 0x20000e05,
		.series = BFLB_SERIES_BL702L,
		.romapi_get_jedec_id = 0x21010d10,
		.romapi_sflash_init_gpio = 0x21010c54,
		.romapi_sflash_init = 0x21010cc8,
		.romapi_sflash_program = 0x21010d08,
		.romapi_sflash_read = 0x21010d44,
		.romapi_sflash_erase_sector = 0x21010cf8,
	},
};

static int bl602_call_func(struct target *target, uint32_t func_addr,
	uint32_t arg_data[], unsigned int n_args, uint32_t *return_data, unsigned int timeout_ms)
{
	static const char * const reg_names[] = { "a0", "a1", "a2", "a3", "a4", "a5" };
	struct reg_param reg_params[ARRAY_SIZE(reg_names) + 1];
	unsigned int n_reg_params = 0;

	assert(n_args <= ARRAY_SIZE(reg_names)); // only allow register arguments

	struct working_area *trampoline_algorithm;

	uint32_t trampoline_code[] = {
		ebreak(),
	};

	int retval = target_alloc_working_area(target, sizeof(trampoline_code),
			&trampoline_algorithm);
	if (retval != ERROR_OK) {
		LOG_ERROR("No working area available, can't do trampoline");
		return retval;
	}

	retval = target_write_buffer(target, trampoline_algorithm->address,
			sizeof(trampoline_code), (uint8_t *)trampoline_code);
	if (retval != ERROR_OK) {
		LOG_ERROR("Writing trampoline code failed, can't do trampoline");
		goto cleanup;
	}

	// initialize a0 register, which is used both as arg and also return register.
	if (!return_data || n_args > 0) {
		init_reg_param(&reg_params[0], reg_names[0], 32, PARAM_IN);
		if (n_args > 0) {
			buf_set_u32(reg_params[0].value, 0, 32, arg_data[0]);
			reg_params[0].direction = PARAM_IN_OUT;
		}
		n_reg_params++;
	}
	// initialize rest of registers, if any
	for (unsigned int i = 1; i < n_args; ++i) {
		init_reg_param(&reg_params[i], reg_names[i], 32, PARAM_OUT);
		buf_set_u32(reg_params[i].value, 0, 32, arg_data[i]);
		n_reg_params++;
	}
	// set return address to the ebreak instruction in working area
	init_reg_param(&reg_params[n_reg_params], "ra", 32, PARAM_OUT);
	buf_set_u32(reg_params[n_reg_params].value, 0, 32, trampoline_algorithm->address);
	n_reg_params++;

	retval = target_run_algorithm(target,
			0, NULL,
			n_reg_params, reg_params,
			func_addr,
			trampoline_algorithm->address,
			timeout_ms, NULL);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to execute algorithm at 0x%" TARGET_PRIxADDR ": %d",
				trampoline_algorithm->address, retval);
		goto cleanup;
	}

	if (return_data)
		*return_data = buf_get_u32(reg_params[0].value, 0, 32);

cleanup:
	for (unsigned int i = 0; i < n_reg_params; i++)
		destroy_reg_param(&reg_params[i]);

	target_free_working_area(target, trampoline_algorithm);
	return retval;
}

static int bl602_call_romapi_func(struct target *target, uint32_t romapi_func_addr,
	uint32_t arg_data[], unsigned int n_args, uint32_t *return_data, unsigned int timeout_ms)
{
	uint32_t func_addr;

	int retval = target_read_u32(target, romapi_func_addr, &func_addr);
	if (retval != ERROR_OK)
		return retval;

	return bl602_call_func(target, func_addr, arg_data, n_args, return_data, timeout_ms);
}

static int bl602_alloc_bounce_buffer(struct flash_bank *bank,
		struct working_area **working_area, uint32_t count)
{
	struct bl602_flash_bank *priv = bank->driver_priv;
	struct target *target = bank->target;

	unsigned int avail_pages = target_get_working_area_avail(target) / priv->dev->pagesize;
	if (avail_pages == 0) {
		LOG_ERROR("Not enough space for bounce buffer. Can't continue");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* We try to allocate working area rounded down to device page size,
	 * at least 1 page, at most the write data size */
	unsigned int chunk_size = MIN(MAX(avail_pages - 1, 1) * priv->dev->pagesize, count);
	int retval = target_alloc_working_area(target, chunk_size, working_area);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not allocate bounce buffer for flash manipulation. Can't continue");
		return retval;
	}

	LOG_DEBUG("Allocated flash bounce buffer @" TARGET_ADDR_FMT, (*working_area)->address);

	return ERROR_OK;
}

static int bl602_alloc_sflash_cfg(struct flash_bank *bank,
		struct working_area **working_area)
{
	struct bl602_flash_bank *priv = bank->driver_priv;
	struct target *target = bank->target;

	int retval = target_alloc_working_area(target, sizeof(priv->sflash_cfg),
			working_area);
	if (retval != ERROR_OK) {
		LOG_ERROR("No working area available, can't alloc sflash cfg");
		return retval;
	}

	retval = target_write_buffer(target, (*working_area)->address,
			sizeof(priv->sflash_cfg), priv->sflash_cfg);
	if (retval != ERROR_OK)
		target_free_working_area(target, *working_area);
	return retval;
}

static int bl602_flash_gpio_init(struct flash_bank *bank)
{
	struct bl602_flash_bank *priv = bank->driver_priv;
	struct target *target = bank->target;
	const struct bl602_part_info *part_info = priv->part_info;

	uint32_t flash_pin_cfg = 0;

	// Read device info from eFuse
	if (part_info->series == BFLB_SERIES_BL702 ||
		part_info->series == BFLB_SERIES_BL702L) {
		// Read device_info register
		uint32_t device_info;
		int retval = target_read_u32(target, 0x40007074, &device_info);
		if (retval != ERROR_OK || device_info == 0x0) {
			/* device_info being 0x0 shouldn't happen, since eFuse is loaded in BootROM,
				* but better to handle it */
			LOG_ERROR("Failed to read device info from eFuse");
			return retval;
		}

		uint32_t flash_cfg = (device_info >> 26) & 0x03;
		uint32_t sf_swap_cfg = (device_info >> 22) & 0x03;
		uint32_t sf_reverse_cfg = (device_info >> 29) & 0x01; // BL702L only
		if (part_info->series == BFLB_SERIES_BL702) {
			flash_pin_cfg = (flash_cfg << 2) | sf_swap_cfg;
		} else { // BL702L
			flash_pin_cfg = sf_swap_cfg + (sf_reverse_cfg != 0 ? 5 : 1);
		}
	} else if (part_info->series == BFLB_SERIES_BL602) {
		// Structure is not known.
		uint32_t sw_usage;
		int retval = target_read_u32(target, 0x40007010, &sw_usage);
		if (retval != ERROR_OK || sw_usage == 0x0) {
			/* sw_usage being 0x0 shouldn't happen, since eFuse is loaded in BootROM,
				* but better to handle it */
			LOG_ERROR("Failed to read sw usage from eFuse");
			return retval;
		}

		// We only assume it is sf_swap_cfg
		uint32_t sf_swap_cfg = (sw_usage >> 16) & 0x03;
		flash_pin_cfg = sf_swap_cfg;
	}

	// initialize flash GPIOs
	uint32_t args[] = {
		flash_pin_cfg,
		1,	// restoreDefault
	};

	int retval = bl602_call_romapi_func(target, part_info->romapi_sflash_init_gpio,
			args, ARRAY_SIZE(args), NULL, BL602_DEFAULT_TIMEOUT_MS);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to invoke spi flash gpio init function");
		return retval;
	}

	return ERROR_OK;
}

static const uint8_t bl602_sflash_ctrl_cfg[] = {
	0x00, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00, 0x00, 0x00
};

/* The structure of SF_Ctrl_Cfg_Type is same as on BL602 and BL702,
 * but BL702L's BootROM was compiled without -fshort-enums flag.
 * That caused that first three fields (enums) are 4 bytes instead of 1 byte. */
static const uint8_t bl702l_sflash_ctrl_cfg[] = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00,
	0xe1, 0xfd, 0x00, 0x00, 0x00, 0x01, 0x01, 0x00,
	0x00, 0x00, 0x01, 0x21,
};

static int bl602_flash_init(struct flash_bank *bank)
{
	struct bl602_flash_bank *priv = bank->driver_priv;
	struct target *target = bank->target;
	const struct bl602_part_info *part_info = priv->part_info;

	int retval = bl602_flash_gpio_init(bank);
	if (retval != ERROR_OK)
		return retval;

	// initialize SFlash peripheral
	// structure for this can be found in official SDK by name SF_Ctrl_Cfg_Type
	const uint8_t *sflash_ctrl_cfg = NULL;
	uint16_t sflash_ctrl_cfg_size = 0;

	switch (part_info->series) {
		case BFLB_SERIES_BL602:
		case BFLB_SERIES_BL702:
			sflash_ctrl_cfg = bl602_sflash_ctrl_cfg;
			sflash_ctrl_cfg_size = ARRAY_SIZE(bl602_sflash_ctrl_cfg);
			break;
		case BFLB_SERIES_BL702L:
			sflash_ctrl_cfg = bl702l_sflash_ctrl_cfg;
			sflash_ctrl_cfg_size = ARRAY_SIZE(bl702l_sflash_ctrl_cfg);
			break;
		default:
			LOG_ERROR("No SFlash Control Config available for this chip");
			return ERROR_NOT_IMPLEMENTED;
	}

	struct working_area *sflash_ctrl_cfg_area;
	retval = target_alloc_working_area(target, sflash_ctrl_cfg_size,
			&sflash_ctrl_cfg_area);
	if (retval != ERROR_OK) {
		LOG_WARNING("No working area available, can't init SFlash");
		return retval;
	}

	retval = target_write_buffer(target, sflash_ctrl_cfg_area->address,
		sflash_ctrl_cfg_size, sflash_ctrl_cfg);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write SFlash Control CFG into working area");
		goto cleanup;
	}

	uint32_t args[] = {
		sflash_ctrl_cfg_area->address,
	};

	retval = bl602_call_romapi_func(target, part_info->romapi_sflash_init,
			args, ARRAY_SIZE(args), NULL, BL602_DEFAULT_TIMEOUT_MS);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to invoke SFlash init function");
		goto cleanup;
	}

cleanup:
	target_free_working_area(target, sflash_ctrl_cfg_area);
	return retval;
}

static int bl602_flash_read_id(struct flash_bank *bank, uint32_t *jedec_id)
{
	struct bl602_flash_bank *priv = bank->driver_priv;
	struct target *target = bank->target;
	const struct bl602_part_info *part_info = priv->part_info;

	struct working_area *data_area;
	int retval = target_alloc_working_area(target, sizeof(uint32_t) + SFLASH_CFG_SIZE,
			&data_area);
	if (retval != ERROR_OK) {
		LOG_ERROR("No working area available, can't read flash id");
		return retval;
	}

	retval = target_write_buffer(target, data_area->address + sizeof(uint32_t),
			SFLASH_CFG_SIZE, priv->sflash_cfg);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write data required for flash id read");
		goto cleanup;
	}

	uint32_t args[2] = {
		data_area->address + 4,	// SFlash CFG
		data_area->address,		// JEDEC ID pointer
	};

	retval = bl602_call_romapi_func(target, part_info->romapi_get_jedec_id,
			args, ARRAY_SIZE(args), NULL, BL602_DEFAULT_TIMEOUT_MS);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to invoke get jedec id function");
		goto cleanup;
	}

	retval = target_read_u32(target, data_area->address, jedec_id);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read flash id from target");
		goto cleanup;
	}

	*jedec_id &= 0x00FFFFFF;

cleanup:
	target_free_working_area(target, data_area);

	return retval;
}

static int bl602_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct bl602_flash_bank *priv = bank->driver_priv;
	const struct bl602_part_info *part_info = priv->part_info;
	struct target *target = bank->target;
	struct working_area *sflash_cfg_area = NULL;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval = bl602_alloc_sflash_cfg(bank, &sflash_cfg_area);
	if (retval != ERROR_OK)
		return retval;

	uint32_t return_value;

	for (unsigned int sector = first; sector <= last; sector++) {
		LOG_INFO("Erasing sector %d", sector);
		uint32_t args[] = {
			sflash_cfg_area->address,
			sector,
		};
		retval = bl602_call_romapi_func(target, part_info->romapi_sflash_erase_sector,
				args, ARRAY_SIZE(args), &return_value, BL602_DEFAULT_TIMEOUT_MS);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to invoke flash erase code on target");
			break;
		}
		if (return_value != 0) {
			LOG_ERROR("Erase flash function returned wrong value: %02X", return_value);
			retval = ERROR_FAIL;
			break;
		}
	}

	target_free_working_area(target, sflash_cfg_area);
	return retval;
}

static int bl602_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct bl602_flash_bank *priv = bank->driver_priv;
	const struct bl602_part_info *part_info = priv->part_info;
	struct target *target = bank->target;
	struct working_area *bounce_area = NULL;
	struct working_area *sflash_cfg_area = NULL;

	LOG_DEBUG("bank->size=0x%x offset=0x%08" PRIx32 " count=0x%08" PRIx32,
			bank->size, offset, count);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > priv->dev->size_in_bytes) {
		LOG_WARNING("Write past end of flash. Extra data discarded");
		count = priv->dev->size_in_bytes - offset;
	}

	int retval = bl602_alloc_sflash_cfg(bank, &sflash_cfg_area);
	if (retval != ERROR_OK)
		return retval;

	retval = bl602_alloc_bounce_buffer(bank, &bounce_area, count);
	if (retval != ERROR_OK)
		goto cleanup;

	unsigned int chunk_size = bounce_area->size;
	uint32_t return_value;

	while (count > 0) {
		uint32_t write_size = count > chunk_size ? chunk_size : count;
		LOG_INFO("Writing %d bytes to offset 0x%" PRIx32, write_size, offset);
		retval = target_write_buffer(target, bounce_area->address, write_size, buffer);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not load data into target bounce buffer");
			goto cleanup;
		}
		uint32_t args[] = {
			sflash_cfg_area->address,
			0x0, // io_mode
			offset,
			bounce_area->address,
			write_size,
		};
		retval = bl602_call_romapi_func(target, part_info->romapi_sflash_program,
				args, ARRAY_SIZE(args), &return_value, BL602_DEFAULT_TIMEOUT_MS);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to invoke flash programming code on target");
			goto cleanup;
		}
		if (return_value != 0) {
			LOG_ERROR("Write flash function returned wrong value: %02X", return_value);
			retval = ERROR_FAIL;
			goto cleanup;
		}

		buffer += write_size;
		offset += write_size;
		count -= write_size;
	}


cleanup:
	target_free_working_area(target, bounce_area);
	target_free_working_area(target, sflash_cfg_area);
	return retval;
}

static int bl602_read(struct flash_bank *bank,
	uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct bl602_flash_bank *priv = bank->driver_priv;
	const struct bl602_part_info *part_info = priv->part_info;
	struct target *target = bank->target;
	struct working_area *bounce_area = NULL;
	struct working_area *sflash_cfg_area = NULL;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > priv->dev->size_in_bytes) {
		LOG_WARNING("Read past end of flash");
		count = priv->dev->size_in_bytes - offset;
	}

	int retval = bl602_alloc_sflash_cfg(bank, &sflash_cfg_area);
	if (retval != ERROR_OK)
		return retval;

	retval = bl602_alloc_bounce_buffer(bank, &bounce_area, count);
	if (retval != ERROR_OK)
		goto cleanup;

	unsigned int chunk_size = bounce_area->size;

	while (count > 0) {
		uint32_t read_size = count > chunk_size ? chunk_size : count;
		LOG_DEBUG("Read %d bytes from offset 0x%" PRIx32, read_size, offset);
		uint32_t args[] = {
			sflash_cfg_area->address,
			0x0, // io_mode
			false, // continous_read
			offset,
			bounce_area->address,
			read_size,
		};
		uint32_t return_value;
		retval = bl602_call_romapi_func(target, part_info->romapi_sflash_read,
				args, ARRAY_SIZE(args), &return_value, BL602_DEFAULT_TIMEOUT_MS);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to invoke flash read code on target");
			break;
		}
		if (return_value != 0) {
			LOG_ERROR("Read flash function returned wrong value: %02X", return_value);
			retval = ERROR_FAIL;
			break;
		}

		retval = target_read_buffer(target, bounce_area->address, read_size, buffer);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not load data from target bounce buffer");
			break;
		}

		buffer += read_size;
		offset += read_size;
		count -= read_size;
	}

cleanup:
	target_free_working_area(target, bounce_area);
	target_free_working_area(target, sflash_cfg_area);

	return retval;
}

static int bl602_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct bl602_flash_bank *priv = bank->driver_priv;

	if (!target_was_examined(target)) {
		LOG_ERROR("Target not examined yet");
		return ERROR_TARGET_NOT_EXAMINED;
	}

	priv->probed = false;

	for (unsigned int n = 0; n < ARRAY_SIZE(bl602_parts); n++) {
		const struct bl602_part_info *part_info = &bl602_parts[n];
		if (priv->series == part_info->series) {
			if (target->tap->idcode != part_info->idcode) {
				LOG_ERROR("TAP IDCODE is not valid for selected chip type");
				return ERROR_FAIL;
			}
			priv->part_info = part_info;
			break;
		}
	}

	if (!priv->part_info) {
		LOG_ERROR("Cannot identify target as an BL602 family device");
		return ERROR_FAIL;
	}

	int retval = bl602_flash_init(bank);
	if (retval != ERROR_OK) {
		LOG_ERROR("Initialization of flash failed");
		return retval;
	}

	uint32_t jedec_id;
	retval = bl602_flash_read_id(bank, &jedec_id);
	if (retval != ERROR_OK) {
		LOG_ERROR("Cannot identify flash JEDEC ID");
		return retval;
	}

	priv->dev = NULL;
	for (const struct flash_device *p = flash_devices; p->name ; p++) {
		if (p->device_id == jedec_id) {
			priv->dev = p;
			break;
		}
	}

	if (!priv->dev) {
		LOG_ERROR("Unknown flash device (ID 0x%08" PRIx32 ")", jedec_id);
		return ERROR_FAIL;
	}
	LOG_INFO("Found flash device '%s' (ID 0x%08" PRIx32 ")", priv->dev->name,
			priv->dev->device_id);

	// set correct size value
	bank->size = priv->dev->size_in_bytes;
	bank->num_sectors = bank->size / priv->dev->sectorsize;
	bank->write_start_alignment = 8;
	bank->write_end_alignment = 8;

	bank->sectors = alloc_block_array(0, priv->dev->sectorsize, bank->num_sectors);
	if (!bank->sectors)
		return ERROR_FAIL;

	priv->sflash_cfg[SFLASH_CFG_PAGE_PROGRAM_CMD_POS] = priv->dev->pprog_cmd;
	priv->sflash_cfg[SFLASH_CFG_PAGE_SIZE_POS] = priv->dev->pagesize & 0xFF;
	priv->sflash_cfg[SFLASH_CFG_PAGE_SIZE_POS + 1] = (priv->dev->pagesize >> 8) & 0xFF;
	priv->sflash_cfg[SFLASH_CFG_FAST_READ_CMD_POS] = priv->dev->read_cmd;
	priv->sflash_cfg[SFLASH_CFG_SECTOR_SIZE_POS] = priv->dev->sectorsize / 1024;
	priv->sflash_cfg[SFLASH_CFG_SECTOR_ERASE_CMD_POS] = priv->dev->erase_cmd;

	priv->probed = true;

	return retval;
}

static int bl602_auto_probe(struct flash_bank *bank)
{
	struct bl602_flash_bank *priv = bank->driver_priv;

	if (priv->probed)
		return ERROR_OK;

	return bl602_probe(bank);
}

/*
 * flash bank bl602 <base> 0 0 0 <target#> <chip_type>
 */
FLASH_BANK_COMMAND_HANDLER(bl602_flash_bank_command)
{
	if (CMD_ARGC < 7)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct bl602_flash_bank *priv;
	priv = malloc(sizeof(struct bl602_flash_bank));
	priv->probed = false;
	priv->part_info = NULL;

	if (strcmp(CMD_ARGV[6], "bl602") == 0) {
		priv->series = BFLB_SERIES_BL602;
	} else if (strcmp(CMD_ARGV[6], "bl702") == 0) {
		priv->series = BFLB_SERIES_BL702;
	} else if (strcmp(CMD_ARGV[6], "bl702l") == 0) {
		priv->series = BFLB_SERIES_BL702L;
	} else {
		LOG_ERROR("Unknown BL602 chip type: %s", CMD_ARGV[6]);
		free(priv);
		return ERROR_FLASH_BANK_INVALID;
	}

	// initialize default sflash_cfg fields
	priv->sflash_cfg[SFLASH_CFG_JEDEC_ID_CMD_POS] = 0x9F;
	priv->sflash_cfg[SFLASH_CFG_JEDEC_ID_CMD_DMY_CLK_POS] = 0x0;
	priv->sflash_cfg[SFLASH_CFG_TIME_PAGE_PGM_POS] = 200;
	priv->sflash_cfg[SFLASH_CFG_WRITE_ENABLE_CMD_POS] = 0x06;
	priv->sflash_cfg[SFLASH_CFG_WRITE_ENABLE_INDEX_POS] = 0;
	priv->sflash_cfg[SFLASH_CFG_WRITE_ENABLE_READ_REG_LEN_POS] = 1;
	priv->sflash_cfg[SFLASH_CFG_WRITE_ENABLE_BIT_POS] = 1;
	priv->sflash_cfg[SFLASH_CFG_READ_STATUS_REG1_POS] = 0x05;
	priv->sflash_cfg[SFLASH_CFG_BUSY_INDEX_POS] = 0;
	priv->sflash_cfg[SFLASH_CFG_BUSY_READ_REG_LEN_POS] = 1;
	priv->sflash_cfg[SFLASH_CFG_BUSY_BIT_POS] = 0;
	priv->sflash_cfg[SFLASH_CFG_TIME_ERASE_SECTOR_POS] = 1000 & 0xFF;
	priv->sflash_cfg[SFLASH_CFG_TIME_ERASE_SECTOR_POS + 1] = (1000 >> 8) & 0xFF;

	// set up driver_priv
	bank->driver_priv = priv;

	return ERROR_OK;
}

const struct flash_driver bl602_flash = {
	.name = "bl602",
	.flash_bank_command = bl602_flash_bank_command,
	.erase = bl602_erase,
	.write = bl602_write,
	.read = bl602_read,
	.probe = bl602_probe,
	.auto_probe = bl602_auto_probe,
	.erase_check = default_flash_blank_check,
	.free_driver_priv = default_flash_free_driver_priv
};
