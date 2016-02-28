/***************************************************************************
 *   Copyright (C) 2013 Synapse Product Development                        *
 *   Andrey Smirnov <andrew.smironv@gmail.com>                             *
 *   Angus Gratton <gus@projectgus.com>                                    *
 *   Erdem U. Altunyurt <spamjunkeater@gmail.com>                          *
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
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <helper/types.h>

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

	NRF51_UICR_SIZE = 0x100,

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

	struct {
		bool probed;
		int (*write) (struct flash_bank *bank,
			      struct nrf51_info *chip,
			      const uint8_t *buffer, uint32_t offset, uint32_t count);
	} bank[2];
	struct target *target;
};

struct nrf51_device_spec {
	uint16_t hwid;
	const char *variant;
	const char *build_code;
	unsigned int flash_size_kb;
};

/* The known devices table below is derived from the "nRF51 Series
 * Compatibility Matrix" document, which can be found by searching for
 * ATTN-51 on the Nordic Semi website:
 *
 * http://www.nordicsemi.com/eng/content/search?SearchText=ATTN-51
 *
 * Up to date with Matrix v2.0, plus some additional HWIDs.
 *
 * The additional HWIDs apply where the build code in the matrix is
 * shown as Gx0, Bx0, etc. In these cases the HWID in the matrix is
 * for x==0, x!=0 means different (unspecified) HWIDs.
 */
static const struct nrf51_device_spec nrf51_known_devices_table[] = {
	/* nRF51822 Devices (IC rev 1). */
	{
		.hwid		= 0x001D,
		.variant	= "QFAA",
		.build_code	= "CA/C0",
		.flash_size_kb	= 256,
	},
	{
		.hwid		= 0x0026,
		.variant	= "QFAB",
		.build_code	= "AA",
		.flash_size_kb	= 128,
	},
	{
		.hwid		= 0x0027,
		.variant	= "QFAB",
		.build_code	= "A0",
		.flash_size_kb	= 128,
	},
	{
		.hwid		= 0x0020,
		.variant	= "CEAA",
		.build_code	= "BA",
		.flash_size_kb	= 256,
	},
	{
		.hwid		= 0x002F,
		.variant	= "CEAA",
		.build_code	= "B0",
		.flash_size_kb	= 256,
	},

	/* nRF51822 Devices (IC rev 2). */
	{
		.hwid		= 0x002A,
		.variant	= "QFAA",
		.build_code	= "FA0",
		.flash_size_kb	= 256,
	},
	{
		.hwid		= 0x0044,
		.variant	= "QFAA",
		.build_code	= "GC0",
		.flash_size_kb	= 256,
	},
	{
		.hwid		= 0x003C,
		.variant	= "QFAA",
		.build_code	= "G0",
		.flash_size_kb	= 256,
	},
	{
		.hwid		= 0x0057,
		.variant	= "QFAA",
		.build_code	= "G2",
		.flash_size_kb	= 256,
	},
	{
		.hwid		= 0x0058,
		.variant	= "QFAA",
		.build_code	= "G3",
		.flash_size_kb	= 256,
	},
	{
		.hwid		= 0x004C,
		.variant	= "QFAB",
		.build_code	= "B0",
		.flash_size_kb	= 128,
	},
	{
		.hwid		= 0x0040,
		.variant	= "CEAA",
		.build_code	= "CA0",
		.flash_size_kb	= 256,
	},
	{
		.hwid		= 0x0047,
		.variant	= "CEAA",
		.build_code	= "DA0",
		.flash_size_kb	= 256,
	},
	{
		.hwid		= 0x004D,
		.variant	= "CEAA",
		.build_code	= "D00",
		.flash_size_kb	= 256,
	},

	/* nRF51822 Devices (IC rev 3). */
	{
		.hwid		= 0x0072,
		.variant	= "QFAA",
		.build_code	= "H0",
		.flash_size_kb	= 256,
	},
	{
		.hwid		= 0x007B,
		.variant	= "QFAB",
		.build_code	= "C0",
		.flash_size_kb	= 128,
	},
	{
		.hwid		= 0x0083,
		.variant	= "QFAC",
		.build_code	= "A0",
		.flash_size_kb	= 256,
	},
	{
		.hwid		= 0x0084,
		.variant	= "QFAC",
		.build_code	= "A1",
		.flash_size_kb	= 256,
	},
	{
		.hwid		= 0x007D,
		.variant	= "CDAB",
		.build_code	= "A0",
		.flash_size_kb	= 128,
	},
	{
		.hwid		= 0x0079,
		.variant	= "CEAA",
		.build_code	= "E0",
		.flash_size_kb	= 256,
	},
	{
		.hwid		= 0x0087,
		.variant	= "CFAC",
		.build_code	= "A0",
		.flash_size_kb	= 256,
	},

	/* nRF51422 Devices (IC rev 1). */
	{
		.hwid		= 0x001E,
		.variant	= "QFAA",
		.build_code	= "CA",
		.flash_size_kb	= 256,
	},
	{
		.hwid		= 0x0024,
		.variant	= "QFAA",
		.build_code	= "C0",
		.flash_size_kb	= 256,
	},
	{
		.hwid		= 0x0031,
		.variant	= "CEAA",
		.build_code	= "A0A",
		.flash_size_kb	= 256,
	},

	/* nRF51422 Devices (IC rev 2). */
	{
		.hwid		= 0x002D,
		.variant	= "QFAA",
		.build_code	= "DAA",
		.flash_size_kb	= 256,
	},
	{
		.hwid		= 0x002E,
		.variant	= "QFAA",
		.build_code	= "E0",
		.flash_size_kb	= 256,
	},
	{
		.hwid		= 0x0061,
		.variant	= "QFAB",
		.build_code	= "A00",
		.flash_size_kb	= 128,
	},
	{
		.hwid		= 0x0050,
		.variant	= "CEAA",
		.build_code	= "B0",
		.flash_size_kb	= 256,
	},

	/* nRF51422 Devices (IC rev 3). */
	{
		.hwid		= 0x0073,
		.variant	= "QFAA",
		.build_code	= "F0",
		.flash_size_kb	= 256,
	},
	{
		.hwid		= 0x007C,
		.variant	= "QFAB",
		.build_code	= "B0",
		.flash_size_kb	= 128,
	},
	{
		.hwid		= 0x0085,
		.variant	= "QFAC",
		.build_code	= "A0",
		.flash_size_kb	= 256,
	},
	{
		.hwid		= 0x0086,
		.variant	= "QFAC",
		.build_code	= "A1",
		.flash_size_kb	= 256,
	},
	{
		.hwid		= 0x007E,
		.variant	= "CDAB",
		.build_code	= "A0",
		.flash_size_kb	= 128,
	},
	{
		.hwid		= 0x007A,
		.variant	= "CEAA",
		.build_code	= "C0",
		.flash_size_kb	= 256,
	},
	{
		.hwid		= 0x0088,
		.variant	= "CFAC",
		.build_code	= "A0",
		.flash_size_kb	= 256,
	},

	/* Some early nRF51-DK (PCA10028) & nRF51-Dongle (PCA10031) boards
	   with built-in jlink seem to use engineering samples not listed
	   in the nRF51 Series Compatibility Matrix V1.0. */
	{
		.hwid		= 0x0071,
		.variant	= "QFAC",
		.build_code	= "AB",
		.flash_size_kb	= 256,
	},
};

static int nrf51_bank_is_probed(struct flash_bank *bank)
{
	struct nrf51_info *chip = bank->driver_priv;

	assert(chip != NULL);

	return chip->bank[bank->bank_number].probed;
}
static int nrf51_probe(struct flash_bank *bank);

static int nrf51_get_probed_chip_if_halted(struct flash_bank *bank, struct nrf51_info **chip)
{
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	*chip = bank->driver_priv;

	int probed = nrf51_bank_is_probed(bank);
	if (probed < 0)
		return probed;
	else if (!probed)
		return nrf51_probe(bank);
	else
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

	LOG_DEBUG("Timed out waiting for NVMC_READY");
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

	/* UICR cannot be write protected so just return early */
	if (bank->base == NRF51_UICR_BASE)
		return ERROR_OK;

	struct nrf51_info *chip = bank->driver_priv;

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

	/* UICR cannot be write protected so just bail out early */
	if (bank->base == NRF51_UICR_BASE)
		return ERROR_FAIL;

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
	}

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
	uint32_t hwid;
	int res;
	struct nrf51_info *chip = bank->driver_priv;

	res = target_read_u32(chip->target, NRF51_FICR_CONFIGID, &hwid);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read CONFIGID register");
		return res;
	}

	hwid &= 0xFFFF;	/* HWID is stored in the lower two
			 * bytes of the CONFIGID register */

	const struct nrf51_device_spec *spec = NULL;
	for (size_t i = 0; i < ARRAY_SIZE(nrf51_known_devices_table); i++)
		if (hwid == nrf51_known_devices_table[i].hwid) {
			spec = &nrf51_known_devices_table[i];
			break;
		}

	if (!chip->bank[0].probed && !chip->bank[1].probed) {
		if (spec)
			LOG_INFO("nRF51822-%s(build code: %s) %ukB Flash",
				 spec->variant, spec->build_code, spec->flash_size_kb);
		else
			LOG_WARNING("Unknown device (HWID 0x%08" PRIx32 ")", hwid);
	}


	if (bank->base == NRF51_FLASH_BASE) {
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

		if (spec && chip->code_memory_size != spec->flash_size_kb) {
			LOG_ERROR("Chip's reported Flash capacity does not match expected one");
			return ERROR_FAIL;
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

		chip->bank[0].probed = true;
	} else {
		bank->size = NRF51_UICR_SIZE;
		bank->num_sectors = 1;
		bank->sectors = calloc(bank->num_sectors,
				       sizeof((bank->sectors)[0]));
		if (!bank->sectors)
			return ERROR_FLASH_BANK_NOT_PROBED;

		bank->sectors[0].size = bank->size;
		bank->sectors[0].offset	= 0;

		/* mark as unknown */
		bank->sectors[0].is_erased = 0;
		bank->sectors[0].is_protected = 0;

		chip->bank[1].probed = true;
	}

	return ERROR_OK;
}

static int nrf51_auto_probe(struct flash_bank *bank)
{
	int probed = nrf51_bank_is_probed(bank);

	if (probed < 0)
		return probed;
	else if (probed)
		return ERROR_OK;
	else
		return nrf51_probe(bank);
}

static struct flash_sector *nrf51_find_sector_by_address(struct flash_bank *bank, uint32_t address)
{
	struct nrf51_info *chip = bank->driver_priv;

	for (int i = 0; i < bank->num_sectors; i++)
		if (bank->sectors[i].offset <= address &&
		    address < (bank->sectors[i].offset + chip->code_page_size))
			return &bank->sectors[i];
	return NULL;
}

static int nrf51_erase_all(struct nrf51_info *chip)
{
	LOG_DEBUG("Erasing all non-volatile memory");
	return nrf51_nvmc_generic_erase(chip,
					NRF51_NVMC_ERASEALL,
					0x00000001);
}

static int nrf51_erase_page(struct flash_bank *bank,
							struct nrf51_info *chip,
							struct flash_sector *sector)
{
	int res;

	LOG_DEBUG("Erasing page at 0x%"PRIx32, sector->offset);
	if (sector->is_protected) {
		LOG_ERROR("Cannot erase protected sector at 0x%" PRIx32, sector->offset);
		return ERROR_FAIL;
	}

	if (bank->base == NRF51_UICR_BASE) {
		uint32_t ppfc;
		res = target_read_u32(chip->target, NRF51_FICR_PPFC,
				      &ppfc);
		if (res != ERROR_OK) {
			LOG_ERROR("Couldn't read PPFC register");
			return res;
		}

		if ((ppfc & 0xFF) == 0xFF) {
			/* We can't erase the UICR.  Double-check to
			   see if it's already erased before complaining. */
			default_flash_blank_check(bank);
			if (sector->is_erased == 1)
				return ERROR_OK;

			LOG_ERROR("The chip was not pre-programmed with SoftDevice stack and UICR cannot be erased separately. Please issue mass erase before trying to write to this region");
			return ERROR_FAIL;
		}

		res = nrf51_nvmc_generic_erase(chip,
					       NRF51_NVMC_ERASEUICR,
					       0x00000001);


	} else {
		res = nrf51_nvmc_generic_erase(chip,
					       NRF51_NVMC_ERASEPAGE,
					       sector->offset);
	}

	if (res == ERROR_OK)
		sector->is_erased = 1;

	return res;
}

static const uint8_t nrf51_flash_write_code[] = {
	/* See contrib/loaders/flash/cortex-m0.S */
/* <wait_fifo>: */
	0x0d, 0x68,		/* ldr	r5,	[r1,	#0] */
	0x00, 0x2d,		/* cmp	r5,	#0 */
	0x0b, 0xd0,		/* beq.n	1e <exit> */
	0x4c, 0x68,		/* ldr	r4,	[r1,	#4] */
	0xac, 0x42,		/* cmp	r4,	r5 */
	0xf9, 0xd0,		/* beq.n	0 <wait_fifo> */
	0x20, 0xcc,		/* ldmia	r4!,	{r5} */
	0x20, 0xc3,		/* stmia	r3!,	{r5} */
	0x94, 0x42,		/* cmp	r4,	r2 */
	0x01, 0xd3,		/* bcc.n	18 <no_wrap> */
	0x0c, 0x46,		/* mov	r4,	r1 */
	0x08, 0x34,		/* adds	r4,	#8 */
/* <no_wrap>: */
	0x4c, 0x60,		/* str	r4, [r1,	#4] */
	0x04, 0x38,		/* subs	r0, #4 */
	0xf0, 0xd1,		/* bne.n	0 <wait_fifo> */
/* <exit>: */
	0x00, 0xbe		/* bkpt	0x0000 */
};


/* Start a low level flash write for the specified region */
static int nrf51_ll_flash_write(struct nrf51_info *chip, uint32_t offset, const uint8_t *buffer, uint32_t bytes)
{
	struct target *target = chip->target;
	uint32_t buffer_size = 8192;
	struct working_area *write_algorithm;
	struct working_area *source;
	uint32_t address = NRF51_FLASH_BASE + offset;
	struct reg_param reg_params[4];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;


	LOG_DEBUG("Writing buffer to flash offset=0x%"PRIx32" bytes=0x%"PRIx32, offset, bytes);
	assert(bytes % 4 == 0);

	/* allocate working area with flash programming code */
	if (target_alloc_working_area(target, sizeof(nrf51_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, falling back to slow memory writes");

		for (; bytes > 0; bytes -= 4) {
			retval = target_write_memory(chip->target, offset, 4, 1, buffer);
			if (retval != ERROR_OK)
				return retval;

			retval = nrf51_wait_for_nvmc(chip);
			if (retval != ERROR_OK)
				return retval;

			offset += 4;
			buffer += 4;
		}

		return ERROR_OK;
	}

	LOG_WARNING("using fast async flash loader. This is currently supported");
	LOG_WARNING("only with ST-Link and CMSIS-DAP. If you have issues, add");
	LOG_WARNING("\"set WORKAREASIZE 0\" before sourcing nrf51.cfg to disable it");

	retval = target_write_buffer(target, write_algorithm->address,
				sizeof(nrf51_flash_write_code),
				nrf51_flash_write_code);
	if (retval != ERROR_OK)
		return retval;

	/* memory buffer */
	while (target_alloc_working_area(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		buffer_size &= ~3UL; /* Make sure it's 4 byte aligned */
		if (buffer_size <= 256) {
			/* free working area, write algorithm already allocated */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("No large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);	/* byte count */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* buffer start */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);	/* buffer end */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_IN_OUT);	/* target address */

	buf_set_u32(reg_params[0].value, 0, 32, bytes);
	buf_set_u32(reg_params[1].value, 0, 32, source->address);
	buf_set_u32(reg_params[2].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[3].value, 0, 32, address);

	retval = target_run_flash_async_algorithm(target, buffer, bytes/4, 4,
			0, NULL,
			4, reg_params,
			source->address, source->size,
			write_algorithm->address, 0,
			&armv7m_info);

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);

	return retval;
}

/* Check and erase flash sectors in specified range then start a low level page write.
   start/end must be sector aligned.
*/
static int nrf51_write_pages(struct flash_bank *bank, uint32_t start, uint32_t end, const uint8_t *buffer)
{
	int res = ERROR_FAIL;
	struct nrf51_info *chip = bank->driver_priv;
	struct flash_sector *sector;
	uint32_t offset;

	assert(start % chip->code_page_size == 0);
	assert(end % chip->code_page_size == 0);

	/* Erase all sectors */
	for (offset = start; offset < end; offset += chip->code_page_size) {
		sector = nrf51_find_sector_by_address(bank, offset);
		if (!sector) {
			LOG_ERROR("Invalid sector @ 0x%08"PRIx32, offset);
			return ERROR_FLASH_SECTOR_INVALID;
		}

		if (sector->is_protected) {
			LOG_ERROR("Can't erase protected sector @ 0x%08"PRIx32, offset);
			goto error;
		}

		if (sector->is_erased != 1) {	/* 1 = erased, 0= not erased, -1 = unknown */
			res = nrf51_erase_page(bank, chip, sector);
			if (res != ERROR_OK) {
				LOG_ERROR("Failed to erase sector @ 0x%08"PRIx32, sector->offset);
				goto error;
			}
		}
		sector->is_erased = 0;
	}

	res = nrf51_nvmc_write_enable(chip);
	if (res != ERROR_OK)
		goto error;

	res = nrf51_ll_flash_write(chip, start, buffer, (end - start));
	if (res != ERROR_OK)
		goto set_read_only;

	return nrf51_nvmc_read_only(chip);

set_read_only:
	nrf51_nvmc_read_only(chip);
error:
	LOG_ERROR("Failed to write to nrf51 flash");
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
		res = nrf51_erase_page(bank, chip, &bank->sectors[s]);

	return res;
}

static int nrf51_code_flash_write(struct flash_bank *bank,
				  struct nrf51_info *chip,
				  const uint8_t *buffer, uint32_t offset, uint32_t count)
{

	int res;
	/* Need to perform reads to fill any gaps we need to preserve in the first page,
	   before the start of buffer, or in the last page, after the end of buffer */
	uint32_t first_page = offset/chip->code_page_size;
	uint32_t last_page = DIV_ROUND_UP(offset+count, chip->code_page_size);

	uint32_t first_page_offset = first_page * chip->code_page_size;
	uint32_t last_page_offset = last_page * chip->code_page_size;

	LOG_DEBUG("Padding write from 0x%08"PRIx32"-0x%08"PRIx32" as 0x%08"PRIx32"-0x%08"PRIx32,
		offset, offset+count, first_page_offset, last_page_offset);

	uint32_t page_cnt = last_page - first_page;
	uint8_t buffer_to_flash[page_cnt*chip->code_page_size];

	/* Fill in any space between start of first page and start of buffer */
	uint32_t pre = offset - first_page_offset;
	if (pre > 0) {
		res = target_read_memory(bank->target,
					first_page_offset,
					1,
					pre,
					buffer_to_flash);
		if (res != ERROR_OK)
			return res;
	}

	/* Fill in main contents of buffer */
	memcpy(buffer_to_flash+pre, buffer, count);

	/* Fill in any space between end of buffer and end of last page */
	uint32_t post = last_page_offset - (offset+count);
	if (post > 0) {
		/* Retrieve the full row contents from Flash */
		res = target_read_memory(bank->target,
					offset + count,
					1,
					post,
					buffer_to_flash+pre+count);
		if (res != ERROR_OK)
			return res;
	}

	return nrf51_write_pages(bank, first_page_offset, last_page_offset, buffer_to_flash);
}

static int nrf51_uicr_flash_write(struct flash_bank *bank,
				  struct nrf51_info *chip,
				  const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	int res;
	uint8_t uicr[NRF51_UICR_SIZE];
	struct flash_sector *sector = &bank->sectors[0];

	if ((offset + count) > NRF51_UICR_SIZE)
		return ERROR_FAIL;

	res = target_read_memory(bank->target,
				 NRF51_UICR_BASE,
				 1,
				 NRF51_UICR_SIZE,
				 uicr);

	if (res != ERROR_OK)
		return res;

	if (sector->is_erased != 1) {
		res = nrf51_erase_page(bank, chip, sector);
		if (res != ERROR_OK)
			return res;
	}

	res = nrf51_nvmc_write_enable(chip);
	if (res != ERROR_OK)
		return res;

	memcpy(&uicr[offset], buffer, count);

	res = nrf51_ll_flash_write(chip, NRF51_UICR_BASE, uicr, NRF51_UICR_SIZE);
	if (res != ERROR_OK) {
		nrf51_nvmc_read_only(chip);
		return res;
	}

	return nrf51_nvmc_read_only(chip);
}


static int nrf51_write(struct flash_bank *bank, const uint8_t *buffer,
		       uint32_t offset, uint32_t count)
{
	int res;
	struct nrf51_info *chip;

	res = nrf51_get_probed_chip_if_halted(bank, &chip);
	if (res != ERROR_OK)
		return res;

	return chip->bank[bank->bank_number].write(bank, chip, buffer, offset, count);
}


FLASH_BANK_COMMAND_HANDLER(nrf51_flash_bank_command)
{
	static struct nrf51_info *chip;

	switch (bank->base) {
	case NRF51_FLASH_BASE:
		bank->bank_number = 0;
		break;
	case NRF51_UICR_BASE:
		bank->bank_number = 1;
		break;
	default:
		LOG_ERROR("Invalid bank address 0x%08" PRIx32, bank->base);
		return ERROR_FAIL;
	}

	if (!chip) {
		/* Create a new chip */
		chip = calloc(1, sizeof(*chip));
		if (!chip)
			return ERROR_FAIL;

		chip->target = bank->target;
	}

	switch (bank->base) {
	case NRF51_FLASH_BASE:
		chip->bank[bank->bank_number].write = nrf51_code_flash_write;
		break;
	case NRF51_UICR_BASE:
		chip->bank[bank->bank_number].write = nrf51_uicr_flash_write;
		break;
	}

	chip->bank[bank->bank_number].probed = false;
	bank->driver_priv = chip;

	return ERROR_OK;
}

COMMAND_HANDLER(nrf51_handle_mass_erase_command)
{
	int res;
	struct flash_bank *bank = NULL;
	struct target *target = get_current_target(CMD_CTX);

	res = get_flash_bank_by_addr(target, NRF51_FLASH_BASE, true, &bank);
	if (res != ERROR_OK)
		return res;

	assert(bank != NULL);

	struct nrf51_info *chip;

	res = nrf51_get_probed_chip_if_halted(bank, &chip);
	if (res != ERROR_OK)
		return res;

	uint32_t ppfc;

	res = target_read_u32(target, NRF51_FICR_PPFC,
			      &ppfc);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read PPFC register");
		return res;
	}

	if ((ppfc & 0xFF) == 0x00) {
		LOG_ERROR("Code region 0 size was pre-programmed at the factory, "
			  "mass erase command won't work.");
		return ERROR_FAIL;
	}

	res = nrf51_erase_all(chip);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to erase the chip");
		nrf51_protect_check(bank);
		return res;
	}

	for (int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_erased = 1;

	res = nrf51_protect_check(bank);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to check chip's write protection");
		return res;
	}

	res = get_flash_bank_by_addr(target, NRF51_UICR_BASE, true, &bank);
	if (res != ERROR_OK)
		return res;

	bank->sectors[0].is_erased = 1;

	return ERROR_OK;
}

static int nrf51_info(struct flash_bank *bank, char *buf, int buf_size)
{
	int res;

	struct nrf51_info *chip;

	res = nrf51_get_probed_chip_if_halted(bank, &chip);
	if (res != ERROR_OK)
		return res;

	static struct {
		const uint32_t address;
		uint32_t value;
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
