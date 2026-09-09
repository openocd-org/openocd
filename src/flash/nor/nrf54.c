// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <helper/types.h>
#include <helper/time_support.h>
#include <helper/bits.h>

/*
 * Related documents
 *
 * nRF54L15, nRF54L10, and nRF54L05 Wireless SoCs
 * https://docs-be.nordicsemi.com/bundle/ps_nrf54L15/page/pdf/nRF54L15_nRF54L10_nRF54L05_Datasheet_v0.8.pdf
 *
 * nRF54L Series Production Programming
 * https://docs-be.nordicsemi.com/bundle/nan_047/attach/nan_047.pdf
 */

#define NRF54_FICR_BASE_ADDR			0x00FFC000

#define NRF54_FICR_INFO_PART_REG		0x31c
#define NRF54_FICR_INFO_VARIANT_REG		0x320
#define NRF54_FICR_INFO_PACKAGE_REG		0x324
#define NRF54_FICR_INFO_RAM_REG			0x328
#define NRF54_FICR_INFO_RRAM_REG		0x32C

#define NRF54_RRAM_BASE_ADDR	0x00000000

#define NRF54_RRAMC_BASE_ADDR	0x5004B000

#define NRF54_RRAMC_REG_CONFIG	0x500
#define NRF54_RRAMC_REG_READY	0x400

#define NRF54_RRAMC_CONFIG_WEN	BIT(0)

struct {
	uint32_t id;
	const char *name;
} nrf54_part_names[] = {
	{ 0x00054B15, "nRF54L15" },
	{ 0x00054B10, "nRF54L10" },
	{ 0x00054B05, "nRF54L05" },
};

struct nrf54_ficr_info {
		// Part identification number.
		uint32_t part;
		/*
		 * Part variant (V), hardware version (H) and production
		 * configuration (P) encoded as ASCII: <VV><H><P>.
		 */
		uint32_t variant;
		/*
		 * Package variant (P) encoded as ASCII: <__><PP>, where (_) indicates
		 * the null character.
		 */
		uint32_t package;
		// SRAM size in KiB.
		uint32_t sram_size;
		// RRAM size in KiB.
		uint32_t rram_size;
};

struct nrf54_info {
	bool probed;
	const char *part_name;

	struct nrf54_ficr_info ficr_info;
};

static int nrf54_read_ficr_info(struct target *target,
		struct nrf54_ficr_info *info)
{
	int retval = target_read_u32(target,
		NRF54_FICR_BASE_ADDR + NRF54_FICR_INFO_PART_REG, &info->part);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target,
		NRF54_FICR_BASE_ADDR + NRF54_FICR_INFO_VARIANT_REG, &info->variant);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target,
		NRF54_FICR_BASE_ADDR + NRF54_FICR_INFO_PACKAGE_REG, &info->package);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target,
		NRF54_FICR_BASE_ADDR + NRF54_FICR_INFO_RAM_REG, &info->sram_size);
	if (retval != ERROR_OK)
		return retval;

	retval = target_read_u32(target,
		NRF54_FICR_BASE_ADDR + NRF54_FICR_INFO_RRAM_REG, &info->rram_size);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static const char *nrf54_get_part_name(uint32_t id)
{
	for (size_t i = 0; i < ARRAY_SIZE(nrf54_part_names); i++) {
		if (id == nrf54_part_names[i].id)
			return nrf54_part_names[i].name;
	}

	return NULL;
}

static void nrf54_u32_to_str(uint32_t data, char *str)
{
	uint8_t tmp[4];

	h_u32_to_be(tmp, data);
	memcpy(str, tmp, 4);
	str[4] = '\0';
}

static bool nrf54_get_chip_type_str(const struct nrf54_info *info,
		char *buf, size_t size)
{
	char variant[5];
	nrf54_u32_to_str(info->ficr_info.variant, variant);

	char package[5];
	// Note that the first two characters are always null characters.
	nrf54_u32_to_str(info->ficr_info.package, package);

	int res = snprintf(buf, size, "%s-%c%c%c%c",
		info->part_name, package[2], package[3], variant[0], variant[1]);

	if (res < 0)
		return false;

	if ((size_t)res >= size)
		return false;

	return true;
}

static int nrf54_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct nrf54_info *info = bank->driver_priv;

	char chip_type[128];
	if (!nrf54_get_chip_type_str(info, chip_type, sizeof(chip_type))) {
		LOG_ERROR("Failed to build chip type string");
		return ERROR_FAIL;
	}

	command_print_sameline(cmd, "%s %" PRIu32 " KiB RRAM, %" PRIu32 " KiB SRAM",
		chip_type, info->ficr_info.rram_size, info->ficr_info.sram_size);

	return ERROR_OK;
}

static int nrf54_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct nrf54_info *nrf54_info = bank->driver_priv;
	struct nrf54_ficr_info *ficr_info = &nrf54_info->ficr_info;

	int retval = nrf54_read_ficr_info(target, ficr_info);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to load FICR from device");
		return retval;
	}

	const char *part_name = nrf54_get_part_name(ficr_info->part);

	if (!part_name) {
		LOG_ERROR("Unknown device with part number 0x%08" PRIx32,
			nrf54_info->ficr_info.part);
		return ERROR_FAIL;
	}

	nrf54_info->part_name = part_name;
	bank->size = ficr_info->rram_size * 1024;

	nrf54_info->probed = true;

	// RRAM has no alignment constraints, see section 4.2.6.2 in the datasheet.
	bank->write_start_alignment = 0;
	bank->write_end_alignment = 0;

	return ERROR_OK;
}

static int nrf54_auto_probe(struct flash_bank *bank)
{
	struct nrf54_info *info = bank->driver_priv;

	if (!info->probed)
		return nrf54_probe(bank);

	return ERROR_OK;
}

static int nrf54_wait_until_ready(struct target *target, uint32_t timeout)
{
	const int64_t start_time = timeval_ms();

	while (true) {
		uint32_t ready;
		int retval = target_read_u32(target,
			NRF54_RRAMC_BASE_ADDR + NRF54_RRAMC_REG_READY, &ready);

		if (retval != ERROR_OK)
			return retval;

		/*
		 * Send a 'keep-alive' signal before checking if the target is ready.
		 * The target is usually ready immediately, so without this 'keep-alive'
		 * it would process consecutive write calls without any keep-alive and
		 * eventually trigger a timeout.
		 */
		keep_alive();

		if (ready)
			break;

		if ((timeval_ms() - start_time) > timeout) {
			LOG_ERROR("Timed out waiting for RRAM");
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int nrf54_write(struct flash_bank *bank, const uint8_t *buffer,
					uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	// Write buffer size in 128-bit words, we always use the maximum.
	const uint32_t write_buffer_size = 32;

	// Unlock RRAM to enables writes and configure the write buffer size.
	int retval = target_write_u32(target,
		NRF54_RRAMC_BASE_ADDR + NRF54_RRAMC_REG_CONFIG,
		(write_buffer_size << 8) | NRF54_RRAMC_CONFIG_WEN);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to unlock RRAM");
		return retval;
	}

	retval = nrf54_wait_until_ready(target, 1000);
	if (retval != ERROR_OK)
		goto error;

	// See section (4.2.6.2) in the datasheet about writing to RRAM.
	const uint32_t buffer_size = (write_buffer_size == 0) ?
		4 : write_buffer_size * 16;

	while (count > 0) {
		const uint32_t write_size = MIN(buffer_size, count);
		retval = target_write_buffer(target, bank->base + offset, write_size,
			buffer);

		if (retval != ERROR_OK)
			goto error;

		retval = nrf54_wait_until_ready(target, 1000);
		if (retval != ERROR_OK)
			goto error;

		offset += write_size;
		buffer += write_size;
		count -= write_size;
	}

	// Check if there is data in the write-buffer that needs to be committed.
	uint32_t buf_status;
	retval = target_read_u32(target, NRF54_RRAMC_BASE_ADDR + 0x418, &buf_status);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read write-buffer status");
		goto error;
	}

	if (!buf_status) {
		LOG_DEBUG("Commit data from write-buffer to RRAM");

		retval = target_write_u32(target, NRF54_RRAMC_BASE_ADDR + 0x8, 0x1);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to start write-buffer commit task");
			goto error;
		}

		retval = nrf54_wait_until_ready(target, 1000);
		if (retval != ERROR_OK) {
			LOG_ERROR("Timed out during write-buffer commit task");
			goto error;
		}
	}

	int retval_lock;
error:
	// Lock RRAMC and disable writes.
	retval_lock = target_write_u32(target,
		NRF54_RRAMC_BASE_ADDR + NRF54_RRAMC_REG_CONFIG, 0x0);

	if (retval != ERROR_OK)
		return retval;

	if (retval_lock != ERROR_OK) {
		LOG_ERROR("Failed to lock RRAMC");
		return retval_lock;
	}

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(nrf54_flash_bank_command)
{
	if (bank->base != NRF54_RRAM_BASE_ADDR) {
		LOG_ERROR("Bank base address must be %" PRIx32, NRF54_RRAM_BASE_ADDR);
		return ERROR_FAIL;
	}

	struct nrf54_info *nrf54_info = calloc(1, sizeof(struct nrf54_info));

	if (!nrf54_info) {
		LOG_ERROR("Failed to allocate memory");
		return ERROR_FAIL;
	}

	nrf54_info->probed = false;
	bank->driver_priv = nrf54_info;

	return ERROR_OK;
}

static const struct command_registration nrf54_command_handlers[] = {
	{
		.name = "nrf54",
		.mode = COMMAND_ANY,
		.help = "nrf54 flash command group",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver nrf54_flash = {
	.name = "nrf54",
	.commands = nrf54_command_handlers,
	.flash_bank_command = nrf54_flash_bank_command,
	.info = nrf54_info,
	.write = nrf54_write,
	.read = default_flash_read,
	.probe = nrf54_probe,
	.auto_probe = nrf54_auto_probe,
	.erase_check = default_flash_blank_check,
	.free_driver_priv = default_flash_free_driver_priv,
};
