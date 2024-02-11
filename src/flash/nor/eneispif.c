// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Copyright (c) 2024 ENE Technology Inc.
 * steven@ene.com.tw
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/time_support.h>
#include <helper/bits.h>
#include <target/target.h>
#include <flash/nor/core.h>
#include <flash/nor/driver.h>
#include <flash/nor/spi.h>

#define ISPICFG  0x0000
#define ISPISTS  0x0004
#define ISPIADDR 0x0008
#define ISPICMD  0x000C
#define ISPIDAT  0x0100

#define ISPISTS_BUSY  BIT(0)
#define STATUS1_QE    BIT(1)

#define CFG_READ  0x372
#define CFG_WRITE 0x371

#define ISPI_CTRL_BASE 0x50101000

/*                name         read qread  page  erase chip  device_id   page   erase  flash
 *                             _cmd _cmd   _prog _cmd* _erase            size   size*  size
 *                                         _cmd        _cmd
 */
struct flash_device ene_flash_device =
	FLASH_ID("ISPI flash", 0x03, 0x00, 0x02, 0x20, 0x60, 0x00132085, 0x100, 0x1000, 0x80000);

struct eneispif_flash_bank {
	bool probed;
	target_addr_t ctrl_base;
	uint32_t dev_id;
	const struct flash_device *dev;
};

FLASH_BANK_COMMAND_HANDLER(eneispif_flash_bank_command)
{
	struct eneispif_flash_bank *eneispif_info;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	eneispif_info = malloc(sizeof(struct eneispif_flash_bank));
	if (!eneispif_info) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	bank->driver_priv = eneispif_info;
	eneispif_info->probed = false;
	eneispif_info->ctrl_base = ISPI_CTRL_BASE;
	if (CMD_ARGC >= 7) {
		COMMAND_PARSE_ADDRESS(CMD_ARGV[6], eneispif_info->ctrl_base);
		LOG_INFO("ASSUMING ISPI device at ctrl_base = " TARGET_ADDR_FMT,
			 eneispif_info->ctrl_base);
	}

	return ERROR_OK;
}

static int eneispif_read_reg(struct flash_bank *bank, uint32_t *value, target_addr_t address)
{
	struct target *target = bank->target;
	struct eneispif_flash_bank *eneispif_info = bank->driver_priv;

	int result = target_read_u32(target, eneispif_info->ctrl_base + address, value);
	if (result != ERROR_OK) {
		LOG_ERROR("%s error at " TARGET_ADDR_FMT, __func__,
			  eneispif_info->ctrl_base + address);
		return result;
	}
	LOG_DEBUG("Read address " TARGET_ADDR_FMT " = 0x%" PRIx32,
		  eneispif_info->ctrl_base + address, *value);
	return ERROR_OK;
}

static int eneispif_write_reg(struct flash_bank *bank, target_addr_t address, uint32_t value)
{
	struct target *target = bank->target;
	struct eneispif_flash_bank *eneispif_info = bank->driver_priv;

	LOG_DEBUG("Write address " TARGET_ADDR_FMT " = 0x%" PRIx32,
		  eneispif_info->ctrl_base + address, value);
	int result = target_write_u32(target, eneispif_info->ctrl_base + address, value);
	if (result != ERROR_OK) {
		LOG_ERROR("%s error writing 0x%" PRIx32 " to " TARGET_ADDR_FMT, __func__,
			  value, eneispif_info->ctrl_base + address);
		return result;
	}
	return ERROR_OK;
}

static int eneispif_wait(struct flash_bank *bank)
{
	int64_t start = timeval_ms();

	while (1) {
		uint32_t status;

		if (eneispif_read_reg(bank, &status, ISPISTS) != ERROR_OK)
			return ERROR_FAIL;

		if (!(status & ISPISTS_BUSY))
			break;

		int64_t now = timeval_ms();
		if (now - start > 1000) {
			LOG_ERROR("Busy more than 1000ms.");
			return ERROR_TARGET_TIMEOUT;
		}
	}

	return ERROR_OK;
}

static int eneispi_erase_sector(struct flash_bank *bank, int sector)
{
	int retval = ERROR_OK;
	struct eneispif_flash_bank *eneispif_info = bank->driver_priv;
	uint32_t offset;
	uint32_t conf;

	retval = eneispif_read_reg(bank, &conf, ISPICFG);
	if (retval != ERROR_OK)
		return retval;

	offset = bank->sectors[sector].offset;
	retval = eneispif_write_reg(bank, ISPIADDR, offset); /* Address */
	if (retval != ERROR_OK)
		goto done;

	eneispif_write_reg(bank, ISPICFG, CFG_WRITE);       /* Cmmmand enable */
	eneispif_write_reg(bank, ISPICMD, SPIFLASH_WRITE_ENABLE); /* Write enable */
	retval = eneispif_write_reg(bank, ISPICMD, eneispif_info->dev->erase_cmd); /* Erase page */
	if (retval != ERROR_OK)
		goto done;

	retval = eneispif_wait(bank);
	if (retval != ERROR_OK)
		goto done;

done:
	eneispif_write_reg(bank, ISPICFG, conf); /* restore */
	return retval;
}

static int eneispif_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct target *target = bank->target;
	struct eneispif_flash_bank *eneispif_info = bank->driver_priv;
	int retval = ERROR_OK;

	LOG_DEBUG("%s: from sector %u to sector %u", __func__, first, last);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (last < first || last >= bank->num_sectors) {
		LOG_ERROR("Flash sector invalid");
		return ERROR_FLASH_SECTOR_INVALID;
	}

	if (!(eneispif_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	for (unsigned int sector = first; sector <= last; sector++) {
		if (bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			return ERROR_FAIL;
		}
	}

	if (eneispif_info->dev->erase_cmd == 0x00)
		return ERROR_FLASH_OPER_UNSUPPORTED;

	for (unsigned int sector = first; sector <= last; sector++) {
		retval = eneispi_erase_sector(bank, sector);
		if (retval != ERROR_OK)
			break;
	}

	return retval;
}

static int eneispif_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last)
{
	for (unsigned int sector = first; sector <= last; sector++)
		bank->sectors[sector].is_protected = set;

	return ERROR_OK;
}

static int eneispif_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset,
			  uint32_t count)
{
	struct target *target = bank->target;
	struct eneispif_flash_bank *eneispif_info = bank->driver_priv;
	uint32_t page_size;
	uint32_t conf;
	int retval = ERROR_OK;

	LOG_DEBUG("bank->size=0x%x offset=0x%08" PRIx32 " count=0x%08" PRIx32, bank->size, offset,
		  count);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset + count > eneispif_info->dev->size_in_bytes) {
		LOG_WARNING("Write past end of flash. Extra data discarded.");
		count = eneispif_info->dev->size_in_bytes - offset;
	}

	/* Check sector protection */
	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		/* Start offset in or before this sector? */
		/* End offset in or behind this sector? */
		if ((offset < (bank->sectors[sector].offset + bank->sectors[sector].size)) &&
		    ((offset + count - 1) >= bank->sectors[sector].offset) &&
		    bank->sectors[sector].is_protected) {
			LOG_ERROR("Flash sector %u protected", sector);
			return ERROR_FAIL;
		}
	}

	retval = eneispif_read_reg(bank, &conf, ISPICFG);
	if (retval != ERROR_OK)
		return retval;

	eneispif_write_reg(bank, ISPICFG, CFG_WRITE); // Cmmmand enable

	/* If no valid page_size, use reasonable default. */
	page_size =
		eneispif_info->dev->pagesize ? eneispif_info->dev->pagesize : SPIFLASH_DEF_PAGESIZE;
	uint32_t page_offset = offset % page_size;

	while (count > 0) {
		uint32_t cur_count;

		/* clip block at page boundary */
		if (page_offset + count > page_size)
			cur_count = page_size - page_offset;
		else
			cur_count = count;

		eneispif_write_reg(bank, ISPICMD, SPIFLASH_WRITE_ENABLE); /* Write enable */
		target_write_buffer(target, eneispif_info->ctrl_base + ISPIDAT, cur_count, buffer);
		eneispif_write_reg(bank, ISPIADDR, offset);
		retval = eneispif_write_reg(bank, ISPICMD,
					    (cur_count << 16) | eneispif_info->dev->pprog_cmd);
		if (retval != ERROR_OK)
			goto err;

		page_offset = 0;
		buffer += cur_count;
		offset += cur_count;
		count -= cur_count;
		retval = eneispif_wait(bank);
		if (retval != ERROR_OK)
			goto err;
	}

err:
	eneispif_write_reg(bank, ISPICFG, conf); /* restore */
	return retval;
}

/* Return ID of flash device */
/* On exit, SW mode is kept */
static int eneispif_read_flash_id(struct flash_bank *bank, uint32_t *id)
{
	struct eneispif_flash_bank *eneispif_info = bank->driver_priv;
	struct target *target = bank->target;
	int retval;
	uint32_t conf, value;
	uint8_t buffer[4];

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = eneispif_read_reg(bank, &conf, ISPICFG);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("ISPCFG = (0x%08" PRIx32 ")", conf);

	/* read ID from Receive Register */
	eneispif_write_reg(bank, ISPICFG, CFG_WRITE); /* Cmmmand enable */
	retval = eneispif_write_reg(bank, ISPICMD, (3 << 16) | SPIFLASH_READ_ID);
	if (retval != ERROR_OK)
		goto done;

	retval = eneispif_wait(bank);
	if (retval != ERROR_OK)
		goto done;

	retval = target_read_buffer(target, eneispif_info->ctrl_base + ISPIDAT, 3, buffer);
	if (retval != ERROR_OK)
		goto done;
	value = (buffer[2] << 16) | (buffer[1] << 8) | buffer[0];
	LOG_DEBUG("ISPDAT = (0x%08" PRIx32 ")", value);

	*id = value;
done:
	eneispif_write_reg(bank, ISPICFG, conf); // restore
	return retval;
}

static int eneispif_probe(struct flash_bank *bank)
{
	struct eneispif_flash_bank *eneispif_info = bank->driver_priv;
	struct flash_sector *sectors;
	uint32_t id;
	int retval;
	uint32_t sectorsize;

	if (eneispif_info->probed)
		free(bank->sectors);

	eneispif_info->probed = false;

	LOG_INFO("Assuming ISPI flash at address " TARGET_ADDR_FMT
		 " with controller at " TARGET_ADDR_FMT,
		 bank->base, eneispif_info->ctrl_base);

	eneispif_write_reg(bank, ISPICFG, CFG_READ); /* RAM map enable */

	retval = eneispif_read_flash_id(bank, &id);
	if (retval != ERROR_OK)
		return retval;

	eneispif_info->dev_id = id;
	eneispif_info->dev = &ene_flash_device;

	LOG_INFO("Found flash device \'%s\' (ID 0x%08" PRIx32 ")", eneispif_info->dev->name,
		 eneispif_info->dev_id);

	/* Set correct size value */
	bank->size = eneispif_info->dev->size_in_bytes;

	if (bank->size <= (1UL << 16))
		LOG_WARNING("device needs 2-byte addresses - not implemented");

	/* if no sectors, treat whole bank as single sector */
	sectorsize = eneispif_info->dev->sectorsize ? eneispif_info->dev->sectorsize
						    : eneispif_info->dev->size_in_bytes;

	/* create and fill sectors array */
	bank->num_sectors = eneispif_info->dev->size_in_bytes / sectorsize;
	sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (!sectors) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * sectorsize;
		sectors[sector].size = sectorsize;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;
	eneispif_info->probed = true;
	return ERROR_OK;
}

static int eneispif_auto_probe(struct flash_bank *bank)
{
	struct eneispif_flash_bank *eneispif_info = bank->driver_priv;
	if (eneispif_info->probed)
		return ERROR_OK;
	return eneispif_probe(bank);
}

static int eneispif_protect_check(struct flash_bank *bank)
{
	/* Nothing to do. Protection is only handled in SW. */
	return ERROR_OK;
}

static int get_eneispif_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct eneispif_flash_bank *eneispif_info = bank->driver_priv;

	if (!(eneispif_info->probed)) {
		command_print(cmd, "ENE ISPI flash bank not probed yet.");
		return ERROR_OK;
	}

	command_print(cmd,
			"ENE ISPI flash information:\n"
			"  Device \'%s\' (ID 0x%08" PRIx32 ")",
			eneispif_info->dev->name, eneispif_info->dev_id);

	return ERROR_OK;
}

const struct flash_driver eneispif_flash = {
	.name = "eneispif",
	.usage = "flash bank <name> 'eneispif' <base_address> <size> 0 0 <target> <ctrl_base>",
	.flash_bank_command = eneispif_flash_bank_command,
	.erase = eneispif_erase,
	.protect = eneispif_protect,
	.write = eneispif_write,
	.read = default_flash_read,
	.probe = eneispif_probe,
	.auto_probe = eneispif_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = eneispif_protect_check,
	.info = get_eneispif_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
