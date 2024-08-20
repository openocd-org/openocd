// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2013 Synapse Product Development                        *
 *   Andrey Smirnov <andrew.smironv@gmail.com>                             *
 *   Angus Gratton <gus@projectgus.com>                                    *
 *   Erdem U. Altunyurt <spamjunkeater@gmail.com>                          *
 ***************************************************************************/

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

/* The refresh code is constant across the current spectrum of nRF5 devices */
#define WATCHDOG_REFRESH_VALUE          0x6e524635

enum {
	NRF5_FLASH_BASE = 0x00000000,
};

enum nrf5_ficr_registers {
	NRF51_52_FICR_BASE = 0x10000000, /* Factory Information Configuration Registers */

#define NRF5_FICR_REG(offset) (NRF51_52_FICR_BASE + (offset))

	NRF51_FICR_CLENR0		= NRF5_FICR_REG(0x028),
	NRF51_FICR_PPFC			= NRF5_FICR_REG(0x02C),
	NRF51_FICR_NUMRAMBLOCK		= NRF5_FICR_REG(0x034),
	NRF51_FICR_SIZERAMBLOCK0	= NRF5_FICR_REG(0x038),
	NRF51_FICR_SIZERAMBLOCK1	= NRF5_FICR_REG(0x03C),
	NRF51_FICR_SIZERAMBLOCK2	= NRF5_FICR_REG(0x040),
	NRF51_FICR_SIZERAMBLOCK3	= NRF5_FICR_REG(0x044),
};

enum nrf5_uicr_registers {
	NRF51_52_UICR_BASE = 0x10001000, /* User Information
				       * Configuration Registers */

#define NRF5_UICR_REG(offset) (NRF51_52_UICR_BASE + (offset))

	NRF51_UICR_CLENR0	= NRF5_UICR_REG(0x000),
};

enum nrf5_nvmc_registers {
	NRF5_NVMC_READY		= 0x400,
	NRF5_NVMC_CONFIG	= 0x504,
	NRF5_NVMC_ERASEPAGE	= 0x508,
	NRF5_NVMC_ERASEALL	= 0x50C,
	NRF5_NVMC_ERASEUICR	= 0x514,

	NRF5_BPROT_BASE = 0x40000000,
};

enum nrf5_nvmc_config_bits {
	NRF5_NVMC_CONFIG_REN = 0x00,
	NRF5_NVMC_CONFIG_WEN = 0x01,
	NRF5_NVMC_CONFIG_EEN = 0x02,

};

struct nrf52_ficr_info {
	uint32_t part;
	uint32_t variant;
	uint32_t package;
	uint32_t ram;
	uint32_t flash;
};

enum nrf5_features {
	NRF5_FEATURE_SERIES_51	= BIT(0),
	NRF5_FEATURE_SERIES_52	= BIT(1),
	NRF5_FEATURE_BPROT		= BIT(2),
	NRF5_FEATURE_ACL_PROT	= BIT(3),
	NRF5_FEATURE_SERIES_53	= BIT(4),
	NRF5_FEATURE_SERIES_91	= BIT(5),
	NRF5_FEATURE_ERASE_BY_FLASH_WR = BIT(6),
};

struct nrf5_device_spec {
	uint16_t hwid;
	const char *part;
	const char *variant;
	const char *build_code;
	unsigned int flash_size_kb;
	enum nrf5_features features;
};

/* FICR registers offsets */
struct nrf5_ficr_map {
	uint32_t codepagesize;
	uint32_t codesize;
	uint32_t configid;
	uint32_t info_part;
	uint32_t info_variant;
	uint32_t info_package;
	uint32_t info_ram;
	uint32_t info_flash;
};

/* Map of device */
struct nrf5_map {
	uint32_t flash_base;
	uint32_t ficr_base;
	uint32_t uicr_base;
	uint32_t nvmc_base;

	uint32_t watchdog_refresh_addr;
};

struct nrf5_info {
	unsigned int refcount;
	bool chip_probed;

	struct nrf5_bank {
		struct nrf5_info *chip;
		bool probed;
	} bank[2];

	struct target *target;

	/* chip identification stored in nrf5_probe_chip()
	 * for use in nrf5_info() and nrf5_setup_bank() */
	bool ficr_info_valid;
	struct nrf52_ficr_info ficr_info;
	const struct nrf5_device_spec *spec;
	uint16_t hwid;
	enum nrf5_features features;
	uint32_t flash_page_size;
	uint32_t flash_num_sectors;
	unsigned int ram_size_kb;

	const struct nrf5_map *map;
	const struct nrf5_ficr_map *ficr_offsets;
};

#define NRF51_DEVICE_DEF(id, pt, var, bcode, fsize) \
{                                                   \
.hwid          = (id),                              \
.part          = pt,                                \
.variant       = var,                               \
.build_code    = bcode,                             \
.flash_size_kb = (fsize),                           \
.features      = NRF5_FEATURE_SERIES_51,            \
}

/*
 * The table maps known HWIDs to the part numbers, variant
 * build code and some other info. For nRF51 rev 1 and 2 devices
 * this is the only way how to get the part number and variant.
 *
 * All tested nRF51 rev 3 devices have FICR INFO fields
 * but the fields are not documented in RM so we keep HWIDs in
 * this table.
 *
 * nRF52 and newer devices have FICR INFO documented, the autodetection
 * can rely on it and HWIDs table is not used.
 *
 * The known devices table below is derived from the "nRF5x series
 * compatibility matrix" documents.
 *
 * Up to date with Matrix v2.0, plus some additional HWIDs.
 *
 * The additional HWIDs apply where the build code in the matrix is
 * shown as Gx0, Bx0, etc. In these cases the HWID in the matrix is
 * for x==0, x!=0 means different (unspecified) HWIDs.
 */
static const struct nrf5_device_spec nrf5_known_devices_table[] = {
	/* nRF51822 Devices (IC rev 1). */
	NRF51_DEVICE_DEF(0x001D, "51822", "QFAA", "CA/C0", 256),
	NRF51_DEVICE_DEF(0x0026, "51822", "QFAB", "AA",    128),
	NRF51_DEVICE_DEF(0x0027, "51822", "QFAB", "A0",    128),
	NRF51_DEVICE_DEF(0x0020, "51822", "CEAA", "BA",    256),
	NRF51_DEVICE_DEF(0x002F, "51822", "CEAA", "B0",    256),

	/* Some early nRF51-DK (PCA10028) & nRF51-Dongle (PCA10031) boards
	   with built-in jlink seem to use engineering samples not listed
	   in the nRF51 Series Compatibility Matrix V1.0. */
	NRF51_DEVICE_DEF(0x0071, "51822", "QFAC", "AB",    256),

	/* nRF51822 Devices (IC rev 2). */
	NRF51_DEVICE_DEF(0x002A, "51822", "QFAA", "FA0",   256),
	NRF51_DEVICE_DEF(0x0044, "51822", "QFAA", "GC0",   256),
	NRF51_DEVICE_DEF(0x003C, "51822", "QFAA", "G0",    256),
	NRF51_DEVICE_DEF(0x0057, "51822", "QFAA", "G2",    256),
	NRF51_DEVICE_DEF(0x0058, "51822", "QFAA", "G3",    256),
	NRF51_DEVICE_DEF(0x004C, "51822", "QFAB", "B0",    128),
	NRF51_DEVICE_DEF(0x0040, "51822", "CEAA", "CA0",   256),
	NRF51_DEVICE_DEF(0x0047, "51822", "CEAA", "DA0",   256),
	NRF51_DEVICE_DEF(0x004D, "51822", "CEAA", "D00",   256),

	/* nRF51822 Devices (IC rev 3). */
	NRF51_DEVICE_DEF(0x0072, "51822", "QFAA", "H0",    256),
	NRF51_DEVICE_DEF(0x00D1, "51822", "QFAA", "H2",    256),
	NRF51_DEVICE_DEF(0x007B, "51822", "QFAB", "C0",    128),
	NRF51_DEVICE_DEF(0x0083, "51822", "QFAC", "A0",    256),
	NRF51_DEVICE_DEF(0x0084, "51822", "QFAC", "A1",    256),
	NRF51_DEVICE_DEF(0x007D, "51822", "CDAB", "A0",    128),
	NRF51_DEVICE_DEF(0x0079, "51822", "CEAA", "E0",    256),
	NRF51_DEVICE_DEF(0x0087, "51822", "CFAC", "A0",    256),
	NRF51_DEVICE_DEF(0x008F, "51822", "QFAA", "H1",    256),

	/* nRF51422 Devices (IC rev 1). */
	NRF51_DEVICE_DEF(0x001E, "51422", "QFAA", "CA",    256),
	NRF51_DEVICE_DEF(0x0024, "51422", "QFAA", "C0",    256),
	NRF51_DEVICE_DEF(0x0031, "51422", "CEAA", "A0A",   256),

	/* nRF51422 Devices (IC rev 2). */
	NRF51_DEVICE_DEF(0x002D, "51422", "QFAA", "DAA",   256),
	NRF51_DEVICE_DEF(0x002E, "51422", "QFAA", "E0",    256),
	NRF51_DEVICE_DEF(0x0061, "51422", "QFAB", "A00",   128),
	NRF51_DEVICE_DEF(0x0050, "51422", "CEAA", "B0",    256),

	/* nRF51422 Devices (IC rev 3). */
	NRF51_DEVICE_DEF(0x0073, "51422", "QFAA", "F0",    256),
	NRF51_DEVICE_DEF(0x007C, "51422", "QFAB", "B0",    128),
	NRF51_DEVICE_DEF(0x0085, "51422", "QFAC", "A0",    256),
	NRF51_DEVICE_DEF(0x0086, "51422", "QFAC", "A1",    256),
	NRF51_DEVICE_DEF(0x007E, "51422", "CDAB", "A0",    128),
	NRF51_DEVICE_DEF(0x007A, "51422", "CEAA", "C0",    256),
	NRF51_DEVICE_DEF(0x0088, "51422", "CFAC", "A0",    256),

	/* The driver fully autodetects nRF52 series devices by FICR INFO,
	 * no need for nRF52xxx HWIDs in this table */
};

struct nrf5_device_package {
	uint32_t package;
	const char *code;
};

/* Newer devices have FICR INFO.PACKAGE.
 * This table converts its value to two character code */
static const struct nrf5_device_package nrf52_packages_table[] = {
	{ 0x2000, "QF" },
	{ 0x2001, "CH" },
	{ 0x2002, "CI" },
	{ 0x2003, "QC" },
	{ 0x2004, "QI/CA" },	/* differs nRF52805, 810, 811: CA, nRF52833, 840: QI */
	{ 0x2005, "CK" },
	{ 0x2007, "QD" },
	{ 0x2008, "CJ" },
	{ 0x2009, "CF" },
};

static const struct nrf5_ficr_map nrf51_52_ficr_offsets = {
	.codepagesize = 0x10,
	.codesize = 0x14,

	/* CONFIGID is documented on nRF51 series only.
	 * On nRF52 is present but not documented */
	.configid = 0x5c,

	/* Following registers are available on nRF52 and on nRF51 since rev 3 */
	.info_part = 0x100,
	.info_variant = 0x104,
	.info_package = 0x108,
	.info_ram = 0x10c,
	.info_flash = 0x110,
};

static const struct nrf5_map nrf51_52_map = {
	.flash_base = NRF5_FLASH_BASE,
	.ficr_base = NRF51_52_FICR_BASE,
	.uicr_base = NRF51_52_UICR_BASE,
	.nvmc_base = 0x4001E000,

	.watchdog_refresh_addr = 0x40010600,
};


/* Third generation devices (nRF53, nRF91) */

static const struct nrf5_ficr_map nrf53_91_ficr_offsets = {
	.codepagesize = 0x220,
	.codesize = 0x224,
	.configid = 0x200,
	.info_part = 0x20c,
	.info_variant = 0x210,
	.info_package = 0x214,
	.info_ram = 0x218,
	.info_flash = 0x21c,
};

/* Since nRF9160 Product Specification v2.1 there is
 * a new UICR field SIPINFO, which should be preferred.
 * The original INFO fields describe just a part of the chip
 * (PARTNO=9120 at nRF9161)
 */
static const struct nrf5_ficr_map nrf91new_ficr_offsets = {
	.codepagesize = 0x220,
	.codesize = 0x224,
	.configid = 0x200,
	.info_part = 0x140,		/* SIPINFO.PARTNO */
	.info_variant = 0x148,	/* SIPINFO.VARIANT */
	.info_package = 0x214,
	.info_ram = 0x218,
	.info_flash = 0x21c,
};

enum {
	NRF53APP_91_FICR_BASE = 0x00FF0000,
	NRF53APP_91_UICR_BASE = 0x00FF8000,
	NRF53NET_FLASH_BASE = 0x01000000,
	NRF53NET_FICR_BASE = 0x01FF0000,
	NRF53NET_UICR_BASE = 0x01FF8000,
};

static const struct nrf5_map nrf53app_91_map = {
	.flash_base = NRF5_FLASH_BASE,
	.ficr_base = NRF53APP_91_FICR_BASE,
	.uicr_base = NRF53APP_91_UICR_BASE,
	.nvmc_base = 0x50039000,

	.watchdog_refresh_addr = 0x50018600,
};

/* nRF53 duality:
 * SoC consists of two Cortex-M33 cores:
 * - application core with security extensions
 * - network core
 * Each core has its own RAM, flash, FICR and UICR
 * The flash driver probes and handles flash and UICR of one core
 * independently of those dedicated to the other core.
 */
static const struct nrf5_map nrf53net_map = {
	.flash_base = NRF53NET_FLASH_BASE,
	.ficr_base = NRF53NET_FICR_BASE,
	.uicr_base = NRF53NET_UICR_BASE,
	.nvmc_base = 0x41080000,

	.watchdog_refresh_addr = 0x41080000,
};


const struct flash_driver nrf5_flash, nrf51_flash;

static bool nrf5_bank_is_probed(const struct flash_bank *bank)
{
	struct nrf5_bank *nbank = bank->driver_priv;
	return nbank->probed;
}

static bool nrf5_chip_is_probed(const struct flash_bank *bank)
{
	struct nrf5_bank *nbank = bank->driver_priv;
	struct nrf5_info *chip = nbank->chip;
	return chip->chip_probed;
}

static bool nrf5_bank_is_uicr(const struct nrf5_bank *nbank)
{
	struct nrf5_info *chip = nbank->chip;
	return nbank == &chip->bank[1];
}

static int nrf5_nvmc_read_u32(struct nrf5_info *chip, uint32_t reg_offset, uint32_t *value)
{
	return target_read_u32(chip->target, chip->map->nvmc_base + reg_offset, value);
}

static int nrf5_nvmc_write_u32(struct nrf5_info *chip, uint32_t reg_offset, uint32_t value)
{
	return target_write_u32(chip->target, chip->map->nvmc_base + reg_offset, value);
}

static int nrf5_wait_for_nvmc(struct nrf5_info *chip)
{
	uint32_t ready;
	int res;
	int timeout_ms = 340;
	int64_t ts_start = timeval_ms();

	do {
		res = nrf5_nvmc_read_u32(chip, NRF5_NVMC_READY, &ready);
		if (res == ERROR_WAIT) {
			/* The adapter does not handle SWD WAIT properly,
			 * add some delay to reduce number of error messages */
			alive_sleep(10);
			continue;
		}
		if (res != ERROR_OK) {
			LOG_ERROR("Error waiting NVMC_READY: generic flash write/erase error (check protection etc...)");
			return res;
		}

		if (ready == 0x00000001)
			return ERROR_OK;

		keep_alive();

	} while ((timeval_ms()-ts_start) < timeout_ms);

	LOG_DEBUG("Timed out waiting for NVMC_READY");
	return ERROR_FLASH_BUSY;
}

static int nrf5_nvmc_erase_enable(struct nrf5_info *chip)
{
	int res;
	res = nrf5_nvmc_write_u32(chip,
			       NRF5_NVMC_CONFIG,
			       NRF5_NVMC_CONFIG_EEN);

	if (res != ERROR_OK) {
		LOG_ERROR("Failed to enable erase operation");
		return res;
	}

	/*
	  According to NVMC examples in Nordic SDK busy status must be
	  checked after writing to NVMC_CONFIG
	 */
	res = nrf5_wait_for_nvmc(chip);
	if (res != ERROR_OK)
		LOG_ERROR("Erase enable did not complete");

	return res;
}

static int nrf5_nvmc_write_enable(struct nrf5_info *chip)
{
	int res;
	res = nrf5_nvmc_write_u32(chip,
			       NRF5_NVMC_CONFIG,
			       NRF5_NVMC_CONFIG_WEN);

	if (res != ERROR_OK) {
		LOG_ERROR("Failed to enable write operation");
		return res;
	}

	/*
	  According to NVMC examples in Nordic SDK busy status must be
	  checked after writing to NVMC_CONFIG
	 */
	res = nrf5_wait_for_nvmc(chip);
	if (res != ERROR_OK)
		LOG_ERROR("Write enable did not complete");

	return res;
}

static int nrf5_nvmc_read_only(struct nrf5_info *chip)
{
	int res;
	res = nrf5_nvmc_write_u32(chip,
			       NRF5_NVMC_CONFIG,
			       NRF5_NVMC_CONFIG_REN);

	if (res != ERROR_OK) {
		LOG_ERROR("Failed to disable write/erase operation");
		return res;
	}
	/*
	  According to NVMC examples in Nordic SDK busy status must be
	  checked after writing to NVMC_CONFIG
	 */
	res = nrf5_wait_for_nvmc(chip);
	if (res != ERROR_OK)
		LOG_ERROR("Read only enable did not complete");

	return res;
}

/* nRF51 series only */
static int nrf51_protect_check_clenr0(struct flash_bank *bank)
{
	int res;
	uint32_t clenr0;

	struct nrf5_bank *nbank = bank->driver_priv;
	struct nrf5_info *chip = nbank->chip;

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

	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected =
			clenr0 != 0xFFFFFFFF && bank->sectors[i].offset < clenr0;

	return ERROR_OK;
}

/* nRF52 series only */
static int nrf52_protect_check_bprot(struct flash_bank *bank)
{
	struct nrf5_bank *nbank = bank->driver_priv;
	struct nrf5_info *chip = nbank->chip;

	static uint32_t nrf5_bprot_offsets[4] = { 0x600, 0x604, 0x610, 0x614 };
	uint32_t bprot_reg = 0;
	int res;

	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		unsigned int bit = i % 32;
		if (bit == 0) {
			unsigned int n_reg = i / 32;
			if (n_reg >= ARRAY_SIZE(nrf5_bprot_offsets))
				break;

			res = target_read_u32(chip->target, NRF5_BPROT_BASE + nrf5_bprot_offsets[n_reg], &bprot_reg);
			if (res != ERROR_OK)
				return res;
		}
		bank->sectors[i].is_protected = (bprot_reg & (1 << bit)) ? 1 : 0;
	}
	return ERROR_OK;
}

static int nrf5_protect_check(struct flash_bank *bank)
{
	struct nrf5_bank *nbank = bank->driver_priv;
	struct nrf5_info *chip = nbank->chip;

	/* UICR cannot be write protected so just return early */
	if (nrf5_bank_is_uicr(nbank))
		return ERROR_OK;

	if (chip->features & NRF5_FEATURE_BPROT)
		return nrf52_protect_check_bprot(bank);

	if (chip->features & NRF5_FEATURE_SERIES_51)
		return nrf51_protect_check_clenr0(bank);

	LOG_WARNING("Flash protection of this nRF device is not supported");
	return ERROR_FLASH_OPER_UNSUPPORTED;
}

/* nRF51 series only */
static int nrf51_protect_clenr0(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	int res;
	uint32_t clenr0, ppfc;

	struct nrf5_bank *nbank = bank->driver_priv;
	struct nrf5_info *chip = nbank->chip;

	if (first != 0) {
		LOG_ERROR("Code region 0 must start at the beginning of the bank");
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
		LOG_ERROR("Couldn't read code region 0 size from UICR");
		return res;
	}

	if (!set || clenr0 != 0xFFFFFFFF) {
		LOG_ERROR("You need to perform chip erase before changing the protection settings");
		return ERROR_FAIL;
	}

	res = nrf5_nvmc_write_enable(chip);
	if (res != ERROR_OK)
		goto error;

	clenr0 = bank->sectors[last].offset + bank->sectors[last].size;
	res = target_write_u32(chip->target, NRF51_UICR_CLENR0, clenr0);

	int res2 = nrf5_wait_for_nvmc(chip);

	if (res == ERROR_OK)
		res = res2;

	if (res == ERROR_OK)
		LOG_INFO("A reset or power cycle is required for the new protection settings to take effect.");
	else
		LOG_ERROR("Couldn't write code region 0 size to UICR");

error:
	nrf5_nvmc_read_only(chip);

	return res;
}

static int nrf5_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	struct nrf5_bank *nbank = bank->driver_priv;
	struct nrf5_info *chip = nbank->chip;

	/* UICR cannot be write protected so just bail out early */
	if (nrf5_bank_is_uicr(nbank)) {
		LOG_ERROR("UICR page does not support protection");
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (chip->features & NRF5_FEATURE_SERIES_51)
		return nrf51_protect_clenr0(bank, set, first, last);

	LOG_ERROR("Flash protection setting is not supported on this nRF5 device");
	return ERROR_FLASH_OPER_UNSUPPORTED;
}

static bool nrf5_info_variant_to_str(uint32_t variant, char *bf, bool swap)
{
	uint8_t b[4];

	if (swap)
		h_u32_to_le(b, variant);
	else
		h_u32_to_be(b, variant);

	if (isalnum(b[0]) && isalnum(b[1]) && isalnum(b[2]) &&
		 (isalnum(b[3]) || b[3] == 0)) {
		memcpy(bf, b, 4);
		bf[4] = 0;
		return true;
	}

	strcpy(bf, "xxxx");
	return false;
}

static const char *nrf5_decode_info_package(uint32_t package)
{
	for (size_t i = 0; i < ARRAY_SIZE(nrf52_packages_table); i++) {
		if (nrf52_packages_table[i].package == package)
			return nrf52_packages_table[i].code;
	}
	return "xx";
}

static int nrf5_get_chip_type_str(const struct nrf5_info *chip, char *buf, unsigned int buf_size)
{
	int res;
	if (chip->spec) {
		res = snprintf(buf, buf_size, "nRF%s-%s(build code: %s)",
				chip->spec->part, chip->spec->variant, chip->spec->build_code);
	} else if (chip->ficr_info_valid) {
		char variant[5];

		nrf5_info_variant_to_str(chip->ficr_info.variant, variant,
								chip->features & NRF5_FEATURE_SERIES_91);

		if (chip->features & (NRF5_FEATURE_SERIES_53 | NRF5_FEATURE_SERIES_91)) {
			res = snprintf(buf, buf_size, "nRF%" PRIx32 "-%s",
					chip->ficr_info.part, variant);
		} else {
			res = snprintf(buf, buf_size, "nRF%" PRIx32 "-%s%.2s(build code: %s)",
					chip->ficr_info.part,
					nrf5_decode_info_package(chip->ficr_info.package),
					variant, &variant[2]);
		}
	} else {
		res = snprintf(buf, buf_size, "nRF51xxx (HWID 0x%04" PRIx16 ")", chip->hwid);
	}

	/* safety: */
	if (res <= 0 || (unsigned int)res >= buf_size) {
		LOG_ERROR("BUG: buffer problem in %s", __func__);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int nrf5_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct nrf5_bank *nbank = bank->driver_priv;
	struct nrf5_info *chip = nbank->chip;

	char chip_type_str[256];
	if (nrf5_get_chip_type_str(chip, chip_type_str, sizeof(chip_type_str)) != ERROR_OK)
		return ERROR_FAIL;

	unsigned int flash_size_kb = chip->flash_num_sectors * chip->flash_page_size / 1024;
	command_print_sameline(cmd, "%s %ukB Flash, %ukB RAM",
			chip_type_str, flash_size_kb, chip->ram_size_kb);
	return ERROR_OK;
}

static int nrf5_read_ficr_info_part(struct nrf5_info *chip, const struct nrf5_map *map,
							   const struct nrf5_ficr_map *ficr_offsets)
{
	struct target *target = chip->target;
	uint32_t ficr_base = map->ficr_base;

	int res = target_read_u32(target, ficr_base + ficr_offsets->info_part, &chip->ficr_info.part);
	if (res != ERROR_OK)
		LOG_DEBUG("Couldn't read FICR INFO.PART register");

	return res;
}

static int nrf51_52_partno_check(struct nrf5_info *chip)
{

	uint32_t series = chip->ficr_info.part & 0xfffff000;
	switch (series) {
	case 0x51000:
		chip->features = NRF5_FEATURE_SERIES_51;
		return ERROR_OK;

	case 0x52000:
		chip->features = NRF5_FEATURE_SERIES_52;

		switch (chip->ficr_info.part) {
		case 0x52805:
		case 0x52810:
		case 0x52811:
		case 0x52832:
			chip->features |= NRF5_FEATURE_BPROT;
			break;

		case 0x52820:
		case 0x52833:
		case 0x52840:
			chip->features |= NRF5_FEATURE_ACL_PROT;
			break;
		}
		return ERROR_OK;

	default:
		LOG_DEBUG("FICR INFO likely not implemented. Invalid PART value 0x%08"
					PRIx32, chip->ficr_info.part);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
}

static int nrf53_91_partno_check(struct nrf5_info *chip)
{
	uint32_t series = chip->ficr_info.part & 0xffffff00;
	switch (series) {
	case 0x5300:
		chip->features = NRF5_FEATURE_SERIES_53 | NRF5_FEATURE_ERASE_BY_FLASH_WR;
		return ERROR_OK;

	case 0x9100:
		chip->features = NRF5_FEATURE_SERIES_91 | NRF5_FEATURE_ERASE_BY_FLASH_WR;
		return ERROR_OK;

	default:
		LOG_DEBUG("Invalid FICR INFO PART value 0x%08"
					PRIx32, chip->ficr_info.part);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
}

static int nrf5_read_ficr_more_info(struct nrf5_info *chip)
{
	int res;
	struct target *target = chip->target;
	const struct nrf5_ficr_map *ficr_offsets = chip->ficr_offsets;
	uint32_t ficr_base = chip->map->ficr_base;

	res = target_read_u32(target, ficr_base + ficr_offsets->info_variant, &chip->ficr_info.variant);
	if (res != ERROR_OK)
		return res;

	res = target_read_u32(target, ficr_base + ficr_offsets->info_package, &chip->ficr_info.package);
	if (res != ERROR_OK)
		return res;

	res = target_read_u32(target, ficr_base + ficr_offsets->info_ram, &chip->ficr_info.ram);
	if (res != ERROR_OK)
		return res;

	res = target_read_u32(target, ficr_base + ficr_offsets->info_flash, &chip->ficr_info.flash);
	return res;
}

/* nRF51 series only */
static int nrf51_get_ram_size(struct target *target, uint32_t *ram_size)
{
	int res;

	*ram_size = 0;

	uint32_t numramblock;
	res = target_read_u32(target, NRF51_FICR_NUMRAMBLOCK, &numramblock);
	if (res != ERROR_OK) {
		LOG_DEBUG("Couldn't read FICR NUMRAMBLOCK register");
		return res;
	}

	if (numramblock < 1 || numramblock > 4) {
		LOG_DEBUG("FICR NUMRAMBLOCK strange value %" PRIx32, numramblock);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	for (unsigned int i = 0; i < numramblock; i++) {
		uint32_t sizeramblock;
		res = target_read_u32(target, NRF51_FICR_SIZERAMBLOCK0 + sizeof(uint32_t)*i, &sizeramblock);
		if (res != ERROR_OK) {
			LOG_DEBUG("Couldn't read FICR NUMRAMBLOCK register");
			return res;
		}
		if (sizeramblock < 1024 || sizeramblock > 65536)
			LOG_DEBUG("FICR SIZERAMBLOCK strange value %" PRIx32, sizeramblock);
		else
			*ram_size += sizeramblock;
	}
	return res;
}

static int nrf5_probe_chip(struct flash_bank *bank)
{
	int res = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	struct nrf5_bank *nbank = bank->driver_priv;
	struct nrf5_info *chip = nbank->chip;
	struct target *target = chip->target;

	chip->spec = NULL;
	chip->ficr_info_valid = false;

	/* First try to detect nRF53/91 */
	switch (bank->base) {
	case NRF5_FLASH_BASE:
	case NRF53APP_91_UICR_BASE:
		res = nrf5_read_ficr_info_part(chip, &nrf53app_91_map, &nrf91new_ficr_offsets);
		if (res == ERROR_OK) {
			res = nrf53_91_partno_check(chip);
			if (res == ERROR_OK) {
				chip->map = &nrf53app_91_map;
				chip->ficr_offsets = &nrf91new_ficr_offsets;
				break;
			}
		}

		res = nrf5_read_ficr_info_part(chip, &nrf53app_91_map, &nrf53_91_ficr_offsets);
		if (res != ERROR_OK)
			break;

		res = nrf53_91_partno_check(chip);
		if (res != ERROR_OK)
			break;

		chip->map = &nrf53app_91_map;
		chip->ficr_offsets = &nrf53_91_ficr_offsets;
		break;

	case NRF53NET_FLASH_BASE:
	case NRF53NET_UICR_BASE:
		res = nrf5_read_ficr_info_part(chip, &nrf53net_map, &nrf53_91_ficr_offsets);
		if (res != ERROR_OK)
			break;

		res = nrf53_91_partno_check(chip);
		if (res != ERROR_OK)
			break;

		chip->map = &nrf53net_map;
		chip->ficr_offsets = &nrf53_91_ficr_offsets;
		break;

	default:
		break;
	}

	/* If nRF53/91 is not detected, try nRF51/52 */
	if (res != ERROR_OK) {
		/* Guess a nRF51 series if the device has no FICR INFO and we don't know HWID */
		chip->features = NRF5_FEATURE_SERIES_51;
		chip->map = &nrf51_52_map;
		chip->ficr_offsets = &nrf51_52_ficr_offsets;

		/* Don't bail out on error for the case that some old engineering
		 * sample has FICR INFO registers unreadable. We can proceed anyway. */
		res = nrf5_read_ficr_info_part(chip, chip->map, chip->ficr_offsets);
		if (res == ERROR_OK)
			res = nrf51_52_partno_check(chip);
	}

	if (res == ERROR_OK) {
		/* Now we know the device has FICR INFO filled by something relevant:
		 * Although it is not documented, the tested nRF51 rev 3 devices
		 * have FICR INFO.PART, RAM and FLASH of the same format as nRF52.
		 * VARIANT and PACKAGE coding is unknown for a nRF51 device.
		 * nRF52 devices have FICR INFO documented and always filled. */
		res = nrf5_read_ficr_more_info(chip);
		if (res == ERROR_OK) {
			chip->ficr_info_valid = true;
		} else if (chip->features & NRF5_FEATURE_SERIES_51) {
			LOG_DEBUG("Couldn't read some of FICR INFO registers");
		} else {
			LOG_ERROR("Couldn't read some of FICR INFO registers");
			return res;
		}
	}

	const struct nrf5_ficr_map *ficr_offsets = chip->ficr_offsets;
	uint32_t ficr_base = chip->map->ficr_base;
	uint32_t configid = 0;
	res = target_read_u32(target, ficr_base + ficr_offsets->configid, &configid);
	if (res != ERROR_OK) {
		if (chip->features & NRF5_FEATURE_SERIES_51) {
			LOG_ERROR("Couldn't read FICR CONFIGID register");
			return res;
		}

		LOG_DEBUG("Couldn't read FICR CONFIGID register, using FICR INFO");
	}

	/* HWID is stored in the lower two bytes of the CONFIGID register */
	chip->hwid = configid & 0xFFFF;

	for (size_t i = 0; i < ARRAY_SIZE(nrf5_known_devices_table); i++) {
		if (chip->hwid == nrf5_known_devices_table[i].hwid) {
			chip->spec = &nrf5_known_devices_table[i];
			chip->features = chip->spec->features;
			break;
		}
	}

	if (chip->spec && chip->ficr_info_valid) {
		/* check if HWID table gives the same part as FICR INFO */
		if (chip->ficr_info.part != strtoul(chip->spec->part, NULL, 16))
			LOG_WARNING("HWID 0x%04" PRIx32 " mismatch: FICR INFO.PART %"
						PRIx32, chip->hwid, chip->ficr_info.part);
	}

	if (chip->ficr_info_valid) {
		chip->ram_size_kb = chip->ficr_info.ram;
	} else if (chip->features & NRF5_FEATURE_SERIES_51) {
		uint32_t ram_size;
		nrf51_get_ram_size(target, &ram_size);
		chip->ram_size_kb = ram_size / 1024;
	} else {
		chip->ram_size_kb = 0;
	}

	/* The value stored in FICR CODEPAGESIZE is the number of bytes in one page of FLASH. */
	res = target_read_u32(chip->target, ficr_base + ficr_offsets->codepagesize,
				&chip->flash_page_size);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read code page size");
		return res;
	}

	/* Note the register name is misleading,
	 * FICR CODESIZE is the number of pages in flash memory, not the number of bytes! */
	res = target_read_u32(chip->target, ficr_base + ficr_offsets->codesize,
				&chip->flash_num_sectors);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read code memory size");
		return res;
	}

	char chip_type_str[256];
	if (nrf5_get_chip_type_str(chip, chip_type_str, sizeof(chip_type_str)) != ERROR_OK)
		return ERROR_FAIL;

	unsigned int flash_size_kb = chip->flash_num_sectors * chip->flash_page_size / 1024;
	const bool device_is_unknown = (!chip->spec && !chip->ficr_info_valid);
	LOG_INFO("%s%s %ukB Flash, %ukB RAM",
			device_is_unknown ? "Unknown device: " : "",
			chip_type_str,
			flash_size_kb,
			chip->ram_size_kb);

	chip->chip_probed = true;
	return ERROR_OK;
}

static int nrf5_setup_bank(struct flash_bank *bank)
{
	struct nrf5_bank *nbank = bank->driver_priv;
	struct nrf5_info *chip = nbank->chip;

	if (bank->base == chip->map->flash_base) {
		unsigned int flash_size_kb = chip->flash_num_sectors * chip->flash_page_size / 1024;
		/* Sanity check */
		if (chip->spec && flash_size_kb != chip->spec->flash_size_kb)
			LOG_WARNING("Chip's reported Flash capacity does not match expected one");
		if (chip->ficr_info_valid && flash_size_kb != chip->ficr_info.flash)
			LOG_WARNING("Chip's reported Flash capacity does not match FICR INFO.FLASH");

		bank->num_sectors = chip->flash_num_sectors;
		bank->size = chip->flash_num_sectors * chip->flash_page_size;

		bank->sectors = alloc_block_array(0, chip->flash_page_size, bank->num_sectors);
		if (!bank->sectors)
			return ERROR_FAIL;

		chip->bank[0].probed = true;

	} else if (bank->base == chip->map->uicr_base) {
		/* UICR bank */
		bank->num_sectors = 1;
		bank->size = chip->flash_page_size;

		bank->sectors = alloc_block_array(0, chip->flash_page_size, bank->num_sectors);
		if (!bank->sectors)
			return ERROR_FAIL;

		bank->sectors[0].is_protected = 0;

		chip->bank[1].probed = true;
	} else {
		LOG_ERROR("Invalid nRF bank address " TARGET_ADDR_FMT, bank->base);
		return ERROR_FLASH_BANK_INVALID;
	}

	return ERROR_OK;
}

static int nrf5_probe(struct flash_bank *bank)
{
	/* probe always reads actual info from the device */
	int res = nrf5_probe_chip(bank);
	if (res != ERROR_OK)
		return res;

	return nrf5_setup_bank(bank);
}

static int nrf5_auto_probe(struct flash_bank *bank)
{
	if (nrf5_bank_is_probed(bank))
		return ERROR_OK;

	if (!nrf5_chip_is_probed(bank)) {
		int res = nrf5_probe_chip(bank);
		if (res != ERROR_OK)
			return res;
	}

	return nrf5_setup_bank(bank);
}


static int nrf5_erase_page(struct flash_bank *bank,
							struct nrf5_info *chip,
							struct flash_sector *sector)
{
	int res;

	LOG_DEBUG("Erasing page at 0x%"PRIx32, sector->offset);

	if (bank->base == chip->map->uicr_base) {
		if (chip->features & NRF5_FEATURE_SERIES_51) {
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
		}

		res = nrf5_nvmc_write_u32(chip, NRF5_NVMC_ERASEUICR, 0x00000001);

	} else if (chip->features & NRF5_FEATURE_ERASE_BY_FLASH_WR) {
		res = target_write_u32(chip->target, bank->base + sector->offset, 0xffffffff);
		/* nRF9160 errata [2] NVMC: CPU code execution from RAM halted during
		 * flash page erase operation
		 * https://infocenter.nordicsemi.com/index.jsp?topic=%2Ferrata_nRF9160_Rev1%2FERR%2FnRF9160%2FRev1%2Flatest%2Fanomaly_160_2.html
		 * affects also erasing by debugger MEM-AP write:
		 *
		 * Write to a flash address stalls the bus for 87 ms until
		 * page erase finishes! This makes problems if the adapter does not
		 * handle SWD WAIT properly or does not wait long enough.
		 * Using a target algo would not help, AP gets unresponsive too.
		 * Neither sending AP ABORT helps, the next AP access stalls again.
		 * Simply wait long enough before accessing AP again...
		 *
		 * The same errata was observed in nRF9161
		 */
		if (chip->features & NRF5_FEATURE_SERIES_91)
			alive_sleep(90);

	} else {
		res = nrf5_nvmc_write_u32(chip, NRF5_NVMC_ERASEPAGE, sector->offset);
	}

	if (res != ERROR_OK) {
		/* caller logs the error */
		return res;
	}

	res = nrf5_wait_for_nvmc(chip);
	return res;
}

/* Start a low level flash write for the specified region */
static int nrf5_ll_flash_write(struct nrf5_info *chip, uint32_t address, const uint8_t *buffer, uint32_t bytes)
{
	struct target *target = chip->target;
	uint32_t buffer_size = 8192;
	struct working_area *write_algorithm;
	struct working_area *source;
	struct reg_param reg_params[6];
	struct armv7m_algorithm armv7m_info;
	int retval = ERROR_OK;

	static const uint8_t nrf5_flash_write_code[] = {
#include "../../../contrib/loaders/flash/nrf5/nrf5.inc"
	};

	LOG_DEBUG("Writing buffer to flash address=0x%"PRIx32" bytes=0x%"PRIx32, address, bytes);
	assert(bytes % 4 == 0);

	/* allocate working area with flash programming code */
	if (target_alloc_working_area(target, sizeof(nrf5_flash_write_code),
			&write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, falling back to slow memory writes");

		for (; bytes > 0; bytes -= 4) {
			retval = target_write_memory(target, address, 4, 1, buffer);
			if (retval != ERROR_OK)
				return retval;

			retval = nrf5_wait_for_nvmc(chip);
			if (retval != ERROR_OK)
				return retval;

			address += 4;
			buffer += 4;
		}

		return ERROR_OK;
	}

	retval = target_write_buffer(target, write_algorithm->address,
				sizeof(nrf5_flash_write_code),
				nrf5_flash_write_code);
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
	init_reg_param(&reg_params[4], "r6", 32, PARAM_OUT);	/* watchdog refresh value */
	init_reg_param(&reg_params[5], "r7", 32, PARAM_OUT);	/* watchdog refresh register address */

	buf_set_u32(reg_params[0].value, 0, 32, bytes);
	buf_set_u32(reg_params[1].value, 0, 32, source->address);
	buf_set_u32(reg_params[2].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[3].value, 0, 32, address);
	buf_set_u32(reg_params[4].value, 0, 32, WATCHDOG_REFRESH_VALUE);
	buf_set_u32(reg_params[5].value, 0, 32, chip->map->watchdog_refresh_addr);

	retval = target_run_flash_async_algorithm(target, buffer, bytes/4, 4,
			0, NULL,
			ARRAY_SIZE(reg_params), reg_params,
			source->address, source->size,
			write_algorithm->address, write_algorithm->address + sizeof(nrf5_flash_write_code) - 2,
			&armv7m_info);

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	destroy_reg_param(&reg_params[5]);

	return retval;
}

static int nrf5_write(struct flash_bank *bank, const uint8_t *buffer,
					uint32_t offset, uint32_t count)
{
	int res;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	struct nrf5_bank *nbank = bank->driver_priv;
	struct nrf5_info *chip = nbank->chip;

	assert(offset % 4 == 0);
	assert(count % 4 == 0);

	/* UICR CLENR0 based protection used on nRF51 is somewhat clumsy:
	 * RM reads: Code running from code region 1 will not be able to write
	 * to code region 0.
	 * Unfortunately the flash loader running from RAM can write to both
	 * code regions without any hint the protection is violated.
	 *
	 * Update protection state and check if any flash sector to be written
	 * is protected. */
	if (chip->features & NRF5_FEATURE_SERIES_51) {

		res = nrf51_protect_check_clenr0(bank);
		if (res != ERROR_OK)
			return res;

		for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
			struct flash_sector *bs = &bank->sectors[sector];

			/* Start offset in or before this sector? */
			/* End offset in or behind this sector? */
			if ((offset < (bs->offset + bs->size))
					&& ((offset + count - 1) >= bs->offset)
					&& bs->is_protected == 1) {
				LOG_ERROR("Write refused, sector %d is protected", sector);
				return ERROR_FLASH_PROTECTED;
			}
		}
	}

	res = nrf5_nvmc_write_enable(chip);
	if (res != ERROR_OK)
		goto error;

	res = nrf5_ll_flash_write(chip, bank->base + offset, buffer, count);
	if (res != ERROR_OK)
		goto error;

	return nrf5_nvmc_read_only(chip);

error:
	nrf5_nvmc_read_only(chip);
	LOG_ERROR("Failed to write to nrf5 flash");
	return res;
}

static int nrf5_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	int res;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	struct nrf5_bank *nbank = bank->driver_priv;
	struct nrf5_info *chip = nbank->chip;

	/* UICR CLENR0 based protection used on nRF51 prevents erase
	 * absolutely silently. NVMC has no flag to indicate the protection
	 * was violated.
	 *
	 * Update protection state and check if any flash sector to be erased
	 * is protected. */
	if (chip->features & NRF5_FEATURE_SERIES_51) {

		res = nrf51_protect_check_clenr0(bank);
		if (res != ERROR_OK)
			return res;
	}

	res = nrf5_nvmc_erase_enable(chip);
	if (res != ERROR_OK)
		goto error;

	/* For each sector to be erased */
	for (unsigned int s = first; s <= last; s++) {

		if (chip->features & NRF5_FEATURE_SERIES_51
				&& bank->sectors[s].is_protected == 1) {
			LOG_ERROR("Flash sector %d is protected", s);
			res = ERROR_FLASH_PROTECTED;
			break;
		}

		res = nrf5_erase_page(bank, chip, &bank->sectors[s]);
		if (res != ERROR_OK) {
			LOG_ERROR("Error erasing sector %d", s);
			return res;
		}
	}

error:
	nrf5_nvmc_read_only(chip);
	return res;
}

static void nrf5_free_driver_priv(struct flash_bank *bank)
{
	struct nrf5_bank *nbank = bank->driver_priv;
	struct nrf5_info *chip = nbank->chip;
	if (!chip)
		return;

	chip->refcount--;
	if (chip->refcount == 0) {
		free(chip);
		bank->driver_priv = NULL;
	}
}

static struct nrf5_info *nrf5_get_chip(struct target *target)
{
	struct flash_bank *bank_iter;

	/* iterate over nrf5 banks of same target */
	for (bank_iter = flash_bank_list(); bank_iter; bank_iter = bank_iter->next) {
		if (bank_iter->driver != &nrf5_flash && bank_iter->driver != &nrf51_flash)
			continue;

		if (bank_iter->target != target)
			continue;

		struct nrf5_bank *nbank = bank_iter->driver_priv;
		if (!nbank)
			continue;

		if (nbank->chip)
			return nbank->chip;
	}
	return NULL;
}

FLASH_BANK_COMMAND_HANDLER(nrf5_flash_bank_command)
{
	struct nrf5_info *chip;
	struct nrf5_bank *nbank = NULL;

	if (bank->driver == &nrf51_flash)
		LOG_WARNING("Flash driver 'nrf51' is deprecated! Use 'nrf5' instead.");

	switch (bank->base) {
	case NRF5_FLASH_BASE:
	case NRF53NET_FLASH_BASE:
	case NRF51_52_UICR_BASE:
	case NRF53APP_91_UICR_BASE:
	case NRF53NET_UICR_BASE:
		break;
	default:
		LOG_ERROR("Invalid nRF bank address " TARGET_ADDR_FMT, bank->base);
		return ERROR_FLASH_BANK_INVALID;
	}

	chip = nrf5_get_chip(bank->target);
	if (!chip) {
		/* Create a new chip */
		chip = calloc(1, sizeof(*chip));
		if (!chip)
			return ERROR_FAIL;

		chip->target = bank->target;
	}

	switch (bank->base) {
	case NRF5_FLASH_BASE:
	case NRF53NET_FLASH_BASE:
		nbank = &chip->bank[0];
		break;
	case NRF51_52_UICR_BASE:
	case NRF53APP_91_UICR_BASE:
	case NRF53NET_UICR_BASE:
		nbank = &chip->bank[1];
		break;
	}
	assert(nbank);

	chip->refcount++;
	nbank->chip = chip;
	nbank->probed = false;
	bank->driver_priv = nbank;
	bank->write_start_alignment = bank->write_end_alignment = 4;

	return ERROR_OK;
}

COMMAND_HANDLER(nrf5_handle_mass_erase_command)
{
	int res;
	struct flash_bank *bank = NULL;
	struct target *target = get_current_target(CMD_CTX);

	res = get_flash_bank_by_addr(target, NRF5_FLASH_BASE, true, &bank);
	if (res != ERROR_OK)
		return res;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	struct nrf5_bank *nbank = bank->driver_priv;
	struct nrf5_info *chip = nbank->chip;

	if (chip->features & NRF5_FEATURE_SERIES_51) {
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
	}

	res = nrf5_nvmc_erase_enable(chip);
	if (res != ERROR_OK)
		goto error;

	res = nrf5_nvmc_write_u32(chip, NRF5_NVMC_ERASEALL, 0x00000001);
	if (res != ERROR_OK) {
		LOG_ERROR("Mass erase failed");
		goto error;
	}

	res = nrf5_wait_for_nvmc(chip);
	if (res != ERROR_OK)
		LOG_ERROR("Mass erase did not complete");

error:
	nrf5_nvmc_read_only(chip);

	if (res == ERROR_OK) {
		LOG_INFO("Mass erase completed.");
		if (chip->features & NRF5_FEATURE_SERIES_51)
			LOG_INFO("A reset or power cycle is required if the flash was protected before.");
	}

	return res;
}


static const struct command_registration nrf5_exec_command_handlers[] = {
	{
		.name		= "mass_erase",
		.handler	= nrf5_handle_mass_erase_command,
		.mode		= COMMAND_EXEC,
		.help		= "Erase all flash contents of the chip.",
		.usage		= "",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration nrf5_command_handlers[] = {
	{
		.name	= "nrf5",
		.mode	= COMMAND_ANY,
		.help	= "nrf5 flash command group",
		.usage	= "",
		.chain	= nrf5_exec_command_handlers,
	},
	{
		.name	= "nrf51",
		.mode	= COMMAND_ANY,
		.help	= "nrf51 flash command group",
		.usage	= "",
		.chain	= nrf5_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver nrf5_flash = {
	.name			= "nrf5",
	.commands		= nrf5_command_handlers,
	.flash_bank_command	= nrf5_flash_bank_command,
	.info			= nrf5_info,
	.erase			= nrf5_erase,
	.protect		= nrf5_protect,
	.write			= nrf5_write,
	.read			= default_flash_read,
	.probe			= nrf5_probe,
	.auto_probe		= nrf5_auto_probe,
	.erase_check		= default_flash_blank_check,
	.protect_check		= nrf5_protect_check,
	.free_driver_priv	= nrf5_free_driver_priv,
};

/* We need to retain the flash-driver name as well as the commands
 * for backwards compatibility */
const struct flash_driver nrf51_flash = {
	.name			= "nrf51",
	.commands		= nrf5_command_handlers,
	.flash_bank_command	= nrf5_flash_bank_command,
	.info			= nrf5_info,
	.erase			= nrf5_erase,
	.protect		= nrf5_protect,
	.write			= nrf5_write,
	.read			= default_flash_read,
	.probe			= nrf5_probe,
	.auto_probe		= nrf5_auto_probe,
	.erase_check		= default_flash_blank_check,
	.protect_check		= nrf5_protect_check,
	.free_driver_priv	= nrf5_free_driver_priv,
};
