/***************************************************************************
 *   Copyright (C) 2013 Synapse Product Development                        *
 *   Andrey Smirnov <andrew.smironv@gmail.com>                             *
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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"

enum {
	NRF51_FLASH_BASE = 0x00000000,
};

enum nrf51_ficr_registers {
	NRF51_FICR_BASE = 0x10000000, /* Factory Information Configuration Registers */

#define NRF51_FICR_REG(offset) (NRF51_FICR_BASE + offset)

	NRF51_FICR_CODEPAGESIZE		= NRF51_FICR_REG(0x010),
	NRF51_FICR_CODESIZE		= NRF51_FICR_REG(0x014),
	NRF51_FICR_CLENR0		= NRF51_FICR_REG(0x028),
	NRF51_FICR_PPFC			= NRF51_FICR_REG(0x02C),
	NRF51_FICR_NUMRAMBLOCK		= NRF51_FICR_REG(0x034),
	NRF51_FICR_SIZERAMBLOCK0	= NRF51_FICR_REG(0x038),
	NRF51_FICR_SIZERAMBLOCK1	= NRF51_FICR_REG(0x03C),
	NRF51_FICR_SIZERAMBLOCK2	= NRF51_FICR_REG(0x040),
	NRF51_FICR_SIZERAMBLOCK3	= NRF51_FICR_REG(0x044),
	NRF51_FICR_CONFIGID		= NRF51_FICR_REG(0x05C),
	NRF51_FICR_DEVICEID0		= NRF51_FICR_REG(0x060),
	NRF51_FICR_DEVICEID1		= NRF51_FICR_REG(0x064),
	NRF51_FICR_ER0			= NRF51_FICR_REG(0x080),
	NRF51_FICR_ER1			= NRF51_FICR_REG(0x084),
	NRF51_FICR_ER2			= NRF51_FICR_REG(0x088),
	NRF51_FICR_ER3			= NRF51_FICR_REG(0x08C),
	NRF51_FICR_IR0			= NRF51_FICR_REG(0x090),
	NRF51_FICR_IR1			= NRF51_FICR_REG(0x094),
	NRF51_FICR_IR2			= NRF51_FICR_REG(0x098),
	NRF51_FICR_IR3			= NRF51_FICR_REG(0x09C),
	NRF51_FICR_DEVICEADDRTYPE	= NRF51_FICR_REG(0x0A0),
	NRF51_FICR_DEVICEADDR0		= NRF51_FICR_REG(0x0A4),
	NRF51_FICR_DEVICEADDR1		= NRF51_FICR_REG(0x0A8),
	NRF51_FICR_OVERRIDEN		= NRF51_FICR_REG(0x0AC),
	NRF51_FICR_NRF_1MBIT0		= NRF51_FICR_REG(0x0B0),
	NRF51_FICR_NRF_1MBIT1		= NRF51_FICR_REG(0x0B4),
	NRF51_FICR_NRF_1MBIT2		= NRF51_FICR_REG(0x0B8),
	NRF51_FICR_NRF_1MBIT3		= NRF51_FICR_REG(0x0BC),
	NRF51_FICR_NRF_1MBIT4		= NRF51_FICR_REG(0x0C0),
	NRF51_FICR_BLE_1MBIT0		= NRF51_FICR_REG(0x0EC),
	NRF51_FICR_BLE_1MBIT1		= NRF51_FICR_REG(0x0F0),
	NRF51_FICR_BLE_1MBIT2		= NRF51_FICR_REG(0x0F4),
	NRF51_FICR_BLE_1MBIT3		= NRF51_FICR_REG(0x0F8),
	NRF51_FICR_BLE_1MBIT4		= NRF51_FICR_REG(0x0FC),
};

enum nrf51_uicr_registers {
	NRF51_UICR_BASE = 0x10001000, /* User Information
				       * Configuration Regsters */

#define NRF51_UICR_REG(offset) (NRF51_UICR_BASE + offset)

	NRF51_UICR_CLENR0	= NRF51_UICR_REG(0x000),
	NRF51_UICR_RBPCONF	= NRF51_UICR_REG(0x004),
	NRF51_UICR_XTALFREQ	= NRF51_UICR_REG(0x008),
	NRF51_UICR_FWID		= NRF51_UICR_REG(0x010),
};

enum nrf51_nvmc_registers {
	NRF51_NVMC_BASE = 0x4001E000, /* Non-Volatile Memory
				       * Controller Regsters */

#define NRF51_NVMC_REG(offset) (NRF51_NVMC_BASE + offset)

	NRF51_NVMC_READY	= NRF51_NVMC_REG(0x400),
	NRF51_NVMC_CONFIG	= NRF51_NVMC_REG(0x504),
	NRF51_NVMC_ERASEPAGE	= NRF51_NVMC_REG(0x508),
	NRF51_NVMC_ERASEALL	= NRF51_NVMC_REG(0x50C),
	NRF51_NVMC_ERASEUICR	= NRF51_NVMC_REG(0x514),
};

enum nrf51_nvmc_config_bits {
	NRF51_NVMC_CONFIG_REN = 0x00,
	NRF51_NVMC_CONFIG_WEN = 0x01,
	NRF51_NVMC_CONFIG_EEN = 0x02,

};

struct nrf51_info {
	uint32_t code_page_size;
	uint32_t code_memory_size;

	bool probed;
	struct target *target;
};

static int nrf51_probe(struct flash_bank *bank);

static int nrf51_get_probed_chip_if_halted(struct flash_bank *bank, struct nrf51_info **chip)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	*chip = (struct nrf51_info *)bank->driver_priv;

	if (!(*chip)->probed)
		return nrf51_probe(bank);

	return ERROR_OK;
}

static int nrf51_wait_for_nvmc(struct nrf51_info *chip)
{
	uint32_t ready;
	int res;
	int timeout = 100;

	do {
		res = target_read_u32(chip->target, NRF51_NVMC_READY, &ready);
		if (res != ERROR_OK) {
			LOG_ERROR("Couldn't read NVMC_READY register");
			return res;
		}

		if (ready == 0x00000001)
			return ERROR_OK;

		alive_sleep(1);
	} while (timeout--);

	return ERROR_FLASH_BUSY;
}

static int nrf51_nvmc_erase_enable(struct nrf51_info *chip)
{
	int res;
	res = target_write_u32(chip->target,
			       NRF51_NVMC_CONFIG,
			       NRF51_NVMC_CONFIG_EEN);

	if (res != ERROR_OK) {
		LOG_ERROR("Failed to enable erase operation");
		return res;
	}

	/*
	  According to NVMC examples in Nordic SDK busy status must be
	  checked after writing to NVMC_CONFIG
	 */
	res = nrf51_wait_for_nvmc(chip);
	if (res != ERROR_OK)
		LOG_ERROR("Erase enable did not complete");

	return res;
}

static int nrf51_nvmc_write_enable(struct nrf51_info *chip)
{
	int res;
	res = target_write_u32(chip->target,
			       NRF51_NVMC_CONFIG,
			       NRF51_NVMC_CONFIG_WEN);

	if (res != ERROR_OK) {
		LOG_ERROR("Failed to enable write operation");
		return res;
	}

	/*
	  According to NVMC examples in Nordic SDK busy status must be
	  checked after writing to NVMC_CONFIG
	 */
	res = nrf51_wait_for_nvmc(chip);
	if (res != ERROR_OK)
		LOG_ERROR("Write enable did not complete");

	return res;
}

static int nrf51_nvmc_read_only(struct nrf51_info *chip)
{
	int res;
	res = target_write_u32(chip->target,
			       NRF51_NVMC_CONFIG,
			       NRF51_NVMC_CONFIG_REN);

	if (res != ERROR_OK) {
		LOG_ERROR("Failed to enable read-only operation");
		return res;
	}
	/*
	  According to NVMC examples in Nordic SDK busy status must be
	  checked after writing to NVMC_CONFIG
	 */
	res = nrf51_wait_for_nvmc(chip);
	if (res != ERROR_OK)
		LOG_ERROR("Read only enable did not complete");

	return res;
}

static int nrf51_nvmc_generic_erase(struct nrf51_info *chip,
			       uint32_t erase_register, uint32_t erase_value)
{
	int res;

	res = nrf51_nvmc_erase_enable(chip);
	if (res != ERROR_OK)
		goto error;

	res = target_write_u32(chip->target,
			       erase_register,
			       erase_value);
	if (res != ERROR_OK)
		goto set_read_only;

	res = nrf51_wait_for_nvmc(chip);
	if (res != ERROR_OK)
		goto set_read_only;

	return nrf51_nvmc_read_only(chip);

set_read_only:
	nrf51_nvmc_read_only(chip);
error:
	LOG_ERROR("Failed to erase reg: 0x%08"PRIx32" val: 0x%08"PRIx32,
		  erase_register, erase_value);
	return ERROR_FAIL;
}

static int nrf51_protect_check(struct flash_bank *bank)
{
	int res;
	uint32_t clenr0;

	struct nrf51_info *chip = (struct nrf51_info *)bank->driver_priv;

	assert(chip != NULL);

	res = target_read_u32(chip->target, NRF51_FICR_CLENR0,
			      &clenr0);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read code region 0 size[FICR]");
		return res;
	}

	if (clenr0 == 0xFFFFFFFF) {
		res = target_read_u32(chip->target, NRF51_UICR_CLENR0,
				      &clenr0);
		if (res != ERROR_OK) {
			LOG_ERROR("Couldn't read code region 0 size[UICR]");
			return res;
		}
	}

	for (int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected =
			clenr0 != 0xFFFFFFFF && bank->sectors[i].offset < clenr0;

	return ERROR_OK;
}

static int nrf51_protect(struct flash_bank *bank, int set, int first, int last)
{
	int res;
	uint32_t clenr0, ppfc;
	struct nrf51_info *chip;

	res = nrf51_get_probed_chip_if_halted(bank, &chip);
	if (res != ERROR_OK)
		return res;

	if (first != 0) {
		LOG_ERROR("Code region 0 must start at the begining of the bank");
		return ERROR_FAIL;
	}

	res = target_read_u32(chip->target, NRF51_FICR_PPFC,
			      &ppfc);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read PPFC register");
		return res;
	}

	if ((ppfc & 0xFF) == 0x00) {
		LOG_ERROR("Code region 0 size was pre-programmed at the factory, can't change flash protection settings");
		return ERROR_FAIL;
	};

	res = target_read_u32(chip->target, NRF51_UICR_CLENR0,
			      &clenr0);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read code region 0 size[UICR]");
		return res;
	}

	if (clenr0 == 0xFFFFFFFF) {
		res = target_write_u32(chip->target, NRF51_UICR_CLENR0,
				       clenr0);
		if (res != ERROR_OK) {
			LOG_ERROR("Couldn't write code region 0 size[UICR]");
			return res;
		}

	} else {
		LOG_ERROR("You need to perform chip erase before changing the protection settings");
	}

	nrf51_protect_check(bank);

	return ERROR_OK;
}

static int nrf51_probe(struct flash_bank *bank)
{
	uint32_t id;
	int res;
	struct nrf51_info *chip = (struct nrf51_info *)bank->driver_priv;

	res = target_read_u32(chip->target, NRF51_FICR_DEVICEID0, &id);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read Device ID 0 register");
		return res;
	}

	res = target_read_u32(chip->target, NRF51_FICR_DEVICEID1, &id);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read Device ID 1 register");
		return res;
	}

	res = target_read_u32(chip->target, NRF51_FICR_CODEPAGESIZE,
			      &chip->code_page_size);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read code page size");
		return res;
	}

	res = target_read_u32(chip->target, NRF51_FICR_CODESIZE,
			      &chip->code_memory_size);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read code memory size");
		return res;
	}

	bank->size = chip->code_memory_size * 1024;
	bank->num_sectors = bank->size / chip->code_page_size;
	bank->sectors = calloc(bank->num_sectors,
			       sizeof((bank->sectors)[0]));
	if (!bank->sectors)
		return ERROR_FLASH_BANK_NOT_PROBED;

	/* Fill out the sector information: all NRF51 sectors are the same size and
	 * there is always a fixed number of them. */
	for (int i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].size = chip->code_page_size;
		bank->sectors[i].offset	= i * chip->code_page_size;

		/* mark as unknown */
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
	}

	nrf51_protect_check(bank);

	chip->probed = true;

	return ERROR_OK;
}

static int nrf51_auto_probe(struct flash_bank *bank)
{
	struct nrf51_info *chip = (struct nrf51_info *)bank->driver_priv;

	if (chip->probed)
		return ERROR_OK;

	return nrf51_probe(bank);
}

static struct flash_sector *nrf51_find_sector_by_address(struct flash_bank *bank, uint32_t address)
{
	struct nrf51_info *chip = (struct nrf51_info *)bank->driver_priv;

	for (int i = 0; i < bank->num_sectors; i++)
		if (bank->sectors[i].offset <= address &&
		    address < (bank->sectors[i].offset + chip->code_page_size))
			return &bank->sectors[i];
	return NULL;
}

static int nrf51_erase_all(struct nrf51_info *chip)
{
	return nrf51_nvmc_generic_erase(chip,
					NRF51_NVMC_ERASEALL,
					0x00000001);
}

static int nrf51_erase_page(struct nrf51_info *chip, struct flash_sector *sector)
{
	int res;

	if (sector->is_protected)
		return ERROR_FAIL;

	res = nrf51_nvmc_generic_erase(chip,
				       NRF51_NVMC_ERASEPAGE,
				       sector->offset);
	if (res == ERROR_OK)
		sector->is_erased = 1;

	return res;
}

static int nrf51_write_page(struct flash_bank *bank, uint32_t offset, const uint8_t *buffer)
{
	assert(offset % 4 == 0);

	int res = ERROR_FAIL;
	struct nrf51_info *chip = (struct nrf51_info *)bank->driver_priv;
	struct flash_sector *sector = nrf51_find_sector_by_address(bank, offset);

	if (!sector)
		goto error;

	if (sector->is_protected)
		goto error;

	if (!sector->is_erased) {
		res = nrf51_erase_page(chip, sector);
		if (res != ERROR_OK)
			goto error;
	}

	res = nrf51_nvmc_write_enable(chip);
	if (res != ERROR_OK)
		goto error;

	sector->is_erased = 0;
	res = target_write_memory(bank->target, offset, 4,
				  chip->code_page_size / 4, buffer);
	if (res != ERROR_OK)
		goto set_read_only;

	return nrf51_nvmc_read_only(chip);

set_read_only:
	nrf51_nvmc_read_only(chip);
error:
	LOG_ERROR("Failed to write sector @ 0x%08"PRIx32, sector->offset);
	return res;
}

static int nrf51_erase(struct flash_bank *bank, int first, int last)
{
	int res;
	struct nrf51_info *chip;

	res = nrf51_get_probed_chip_if_halted(bank, &chip);
	if (res != ERROR_OK)
		return res;

	/* For each sector to be erased */
	for (int s = first; s <= last && res == ERROR_OK; s++)
		res = nrf51_erase_page(chip, &bank->sectors[s]);

	return res;
}

static int nrf51_write(struct flash_bank *bank, const uint8_t *buffer,
		       uint32_t offset, uint32_t count)
{
	int res;
	struct {
		uint32_t start, end;
	} region;
	struct nrf51_info *chip;

	res = nrf51_get_probed_chip_if_halted(bank, &chip);
	if (res != ERROR_OK)
		return res;

	region.start = offset;
	region.end   = offset + count;

	struct {
		size_t   length;
		const uint8_t *buffer;
	}  start_extra, end_extra;

	start_extra.length	= region.start % chip->code_page_size;
	start_extra.buffer	= buffer;
	end_extra.length	= region.end  % chip->code_page_size;
	end_extra.buffer	= buffer + count - end_extra.length;

	if (start_extra.length) {
		uint8_t page[chip->code_page_size];

		res = target_read_memory(bank->target,
					 region.start - start_extra.length,
					 1, start_extra.length, page);
		if (res != ERROR_OK)
			return res;

		memcpy(page + start_extra.length,
		       start_extra.buffer,
		       chip->code_page_size - start_extra.length);

		res = nrf51_write_page(bank,
				       region.start - start_extra.length,
				       page);
		if (res != ERROR_OK)
			return res;
	}

	if (end_extra.length) {
		uint8_t page[chip->code_page_size];

		/* Retrieve the full row contents from Flash */
		res = target_read_memory(bank->target,
					 region.end,
					 1,
					 (chip->code_page_size - end_extra.length),
					 page + end_extra.length);
		if (res != ERROR_OK)
			return res;

		memcpy(page, end_extra.buffer, end_extra.length);

		res = nrf51_write_page(bank,
				       region.end - end_extra.length,
				       page);
		if (res != ERROR_OK)
			return res;
	}


	region.start += start_extra.length;
	region.end   -= end_extra.length;

	for (uint32_t address = region.start; address < region.end;
	     address += chip->code_page_size) {
		res = nrf51_write_page(bank, address, &buffer[address - region.start]);

		if (res != ERROR_OK)
			return res;

	}

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(nrf51_flash_bank_command)
{
	struct nrf51_info *chip;

	/* Create a new chip */
	chip = calloc(1, sizeof(*chip));
	if (!chip)
		return ERROR_FAIL;

	chip->target = bank->target;
	chip->probed = false;

	bank->driver_priv = chip;

	if (bank->base != NRF51_FLASH_BASE) {
		LOG_ERROR("Address 0x%08" PRIx32 " invalid bank address (try 0x%08" PRIx32
				"[nrf51 series] )",
				bank->base, NRF51_FLASH_BASE);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(nrf51_handle_mass_erase_command)
{
	int res;
	struct flash_bank *bank;

	res = get_flash_bank_by_num(0, &bank);
	if (res != ERROR_OK)
		return res;

	struct nrf51_info *chip;

	res = nrf51_get_probed_chip_if_halted(bank, &chip);
	if (res != ERROR_OK)
		return res;

	uint32_t ppfc;

	res = target_read_u32(chip->target, NRF51_FICR_PPFC,
			      &ppfc);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read PPFC register");
		return res;
	}

	if ((ppfc & 0xFF) == 0x00) {
		LOG_ERROR("Code region 0 size was pre-programmed at the factory, "
			  "mass erase command won't work.");
		return ERROR_FAIL;
	};

	res = nrf51_erase_all(chip);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to erase the chip");
		nrf51_protect_check(bank);
		return res;
	}

	for (int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_erased = 1;

	return nrf51_protect_check(bank);
}

static int nrf51_info(struct flash_bank *bank, char *buf, int buf_size)
{
	int res;

	struct nrf51_info *chip;

	res = nrf51_get_probed_chip_if_halted(bank, &chip);
	if (res != ERROR_OK)
		return res;

	struct {
		uint32_t address, value;
	} ficr[] = {
		{ .address = NRF51_FICR_CODEPAGESIZE	},
		{ .address = NRF51_FICR_CODESIZE	},
		{ .address = NRF51_FICR_CLENR0		},
		{ .address = NRF51_FICR_PPFC		},
		{ .address = NRF51_FICR_NUMRAMBLOCK	},
		{ .address = NRF51_FICR_SIZERAMBLOCK0	},
		{ .address = NRF51_FICR_SIZERAMBLOCK1	},
		{ .address = NRF51_FICR_SIZERAMBLOCK2	},
		{ .address = NRF51_FICR_SIZERAMBLOCK3	},
		{ .address = NRF51_FICR_CONFIGID	},
		{ .address = NRF51_FICR_DEVICEID0	},
		{ .address = NRF51_FICR_DEVICEID1	},
		{ .address = NRF51_FICR_ER0		},
		{ .address = NRF51_FICR_ER1		},
		{ .address = NRF51_FICR_ER2		},
		{ .address = NRF51_FICR_ER3		},
		{ .address = NRF51_FICR_IR0		},
		{ .address = NRF51_FICR_IR1		},
		{ .address = NRF51_FICR_IR2		},
		{ .address = NRF51_FICR_IR3		},
		{ .address = NRF51_FICR_DEVICEADDRTYPE	},
		{ .address = NRF51_FICR_DEVICEADDR0	},
		{ .address = NRF51_FICR_DEVICEADDR1	},
		{ .address = NRF51_FICR_OVERRIDEN	},
		{ .address = NRF51_FICR_NRF_1MBIT0	},
		{ .address = NRF51_FICR_NRF_1MBIT1	},
		{ .address = NRF51_FICR_NRF_1MBIT2	},
		{ .address = NRF51_FICR_NRF_1MBIT3	},
		{ .address = NRF51_FICR_NRF_1MBIT4	},
		{ .address = NRF51_FICR_BLE_1MBIT0	},
		{ .address = NRF51_FICR_BLE_1MBIT1	},
		{ .address = NRF51_FICR_BLE_1MBIT2	},
		{ .address = NRF51_FICR_BLE_1MBIT3	},
		{ .address = NRF51_FICR_BLE_1MBIT4	},
	}, uicr[] = {
		{ .address = NRF51_UICR_CLENR0,		},
		{ .address = NRF51_UICR_RBPCONF		},
		{ .address = NRF51_UICR_XTALFREQ	},
		{ .address = NRF51_UICR_FWID		},
	};

	for (size_t i = 0; i < ARRAY_SIZE(ficr); i++) {
		res = target_read_u32(chip->target, ficr[i].address,
				      &ficr[i].value);
		if (res != ERROR_OK) {
			LOG_ERROR("Couldn't read %" PRIx32, ficr[i].address);
			return res;
		}
	}

	for (size_t i = 0; i < ARRAY_SIZE(uicr); i++) {
		res = target_read_u32(chip->target, uicr[i].address,
				      &uicr[i].value);
		if (res != ERROR_OK) {
			LOG_ERROR("Couldn't read %" PRIx32, uicr[i].address);
			return res;
		}
	}

	snprintf(buf, buf_size,
		 "\n[factory information control block]\n\n"
		 "code page size: %"PRIu32"B\n"
		 "code memory size: %"PRIu32"kB\n"
		 "code region 0 size: %"PRIu32"kB\n"
		 "pre-programmed code: %s\n"
		 "number of ram blocks: %"PRIu32"\n"
		 "ram block 0 size: %"PRIu32"B\n"
		 "ram block 1 size: %"PRIu32"B\n"
		 "ram block 2 size: %"PRIu32"B\n"
		 "ram block 3 size: %"PRIu32 "B\n"
		 "config id: %" PRIx32 "\n"
		 "device id: 0x%"PRIx32"%08"PRIx32"\n"
		 "encryption root: 0x%08"PRIx32"%08"PRIx32"%08"PRIx32"%08"PRIx32"\n"
		 "identity root: 0x%08"PRIx32"%08"PRIx32"%08"PRIx32"%08"PRIx32"\n"
		 "device address type: 0x%"PRIx32"\n"
		 "device address: 0x%"PRIx32"%08"PRIx32"\n"
		 "override enable: %"PRIx32"\n"
		 "NRF_1MBIT values: %"PRIx32" %"PRIx32" %"PRIx32" %"PRIx32" %"PRIx32"\n"
		 "BLE_1MBIT values: %"PRIx32" %"PRIx32" %"PRIx32" %"PRIx32" %"PRIx32"\n"
		 "\n[user information control block]\n\n"
		 "code region 0 size: %"PRIu32"kB\n"
		 "read back protection configuration: %"PRIx32"\n"
		 "reset value for XTALFREQ: %"PRIx32"\n"
		 "firmware id: 0x%04"PRIx32,
		 ficr[0].value,
		 ficr[1].value,
		 (ficr[2].value == 0xFFFFFFFF) ? 0 : ficr[2].value / 1024,
		 ((ficr[3].value & 0xFF) == 0x00) ? "present" : "not present",
		 ficr[4].value,
		 ficr[5].value,
		 (ficr[6].value == 0xFFFFFFFF) ? 0 : ficr[6].value,
		 (ficr[7].value == 0xFFFFFFFF) ? 0 : ficr[7].value,
		 (ficr[8].value == 0xFFFFFFFF) ? 0 : ficr[8].value,
		 ficr[9].value,
		 ficr[10].value, ficr[11].value,
		 ficr[12].value, ficr[13].value, ficr[14].value, ficr[15].value,
		 ficr[16].value, ficr[17].value, ficr[18].value, ficr[19].value,
		 ficr[20].value,
		 ficr[21].value, ficr[22].value,
		 ficr[23].value,
		 ficr[24].value, ficr[25].value, ficr[26].value, ficr[27].value, ficr[28].value,
		 ficr[29].value, ficr[30].value, ficr[31].value, ficr[32].value, ficr[33].value,
		 (uicr[0].value == 0xFFFFFFFF) ? 0 : uicr[0].value / 1024,
		 uicr[1].value & 0xFFFF,
		 uicr[2].value & 0xFF,
		 uicr[3].value & 0xFFFF);

	return ERROR_OK;
}

static const struct command_registration nrf51_exec_command_handlers[] = {
	{
		.name		= "mass_erase",
		.handler	= nrf51_handle_mass_erase_command,
		.mode		= COMMAND_EXEC,
		.help		= "Erase all flash contents of the chip.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration nrf51_command_handlers[] = {
	{
		.name	= "nrf51",
		.mode	= COMMAND_ANY,
		.help	= "nrf51 flash command group",
		.usage	= "",
		.chain	= nrf51_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver nrf51_flash = {
	.name			= "nrf51",
	.commands		= nrf51_command_handlers,
	.flash_bank_command	= nrf51_flash_bank_command,
	.info			= nrf51_info,
	.erase			= nrf51_erase,
	.protect		= nrf51_protect,
	.write			= nrf51_write,
	.read			= default_flash_read,
	.probe			= nrf51_probe,
	.auto_probe		= nrf51_auto_probe,
	.erase_check		= default_flash_blank_check,
	.protect_check		= nrf51_protect_check,
};
