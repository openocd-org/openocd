/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Copyright (C) 2023 by Marc Schink <dev@zapb.de>
 */

#ifndef OPENOCD_FLASH_NOR_ARTERY
#define OPENOCD_FLASH_NOR_ARTERY

#define DEBUG_IDCODE	0xE0042000

#define FLASH_BASE	0x08000000

enum artery_series {
	ARTERY_SERIES_F403A_F407 = 0,
	ARTERY_SERIES_F413,
	ARTERY_SERIES_F415,
	ARTERY_SERIES_F421,
	ARTERY_SERIES_F423,
	ARTERY_SERIES_F425,
	ARTERY_SERIES_F435_F437,
	ARTERY_SERIES_WB415,
};

enum artery_flash_reg_index {
	ARTERY_FLASH_REG_PSR = 0,
	ARTERY_FLASH_REG_UNLOCK,
	ARTERY_FLASH_REG_USD_UNLOCK,
	ARTERY_FLASH_REG_STS,
	ARTERY_FLASH_REG_CTRL,
	ARTERY_FLASH_REG_ADDR,
	ARTERY_FLASH_REG_USD,
	ARTERY_FLASH_REG_EPPS0,
	ARTERY_FLASH_REG_EPPS1,
	ARTERY_FLASH_REG_INDEX_NUM,
};

enum artery_usd_reg_index {
	ARTERY_USD_FAP_INDEX = 0,
	ARTERY_USD_SSB_INDEX,
	ARTERY_USD_DATA_INDEX,
	ARTERY_USD_EPP_INDEX,
	ARTERY_USD_EPP_EXT_INDEX,
	ARTERY_USD_DATA_EXT_INDEX,
	ARTERY_USD_INDEX_NUM,
};

enum artery_fap_level {
	ARTERY_FAP_LEVEL_DISABLED = 0xa5,
	ARTERY_FAP_LEVEL_LOW = 0xff,
	ARTERY_FAP_LEVEL_HIGH = 0xcc,
};

struct artery_part_info {
	uint32_t pid;
	const char *name;
	enum artery_series series;
	// Flash size in bytes.
	uint32_t flash_size;
	// Page / sector size in bytes.
	uint32_t page_size;
	// User system data (USD) area size including the inverse bytes.
	uint32_t usd_size;
	// User data area (part of the USD) size excluding the inverse bytes.
	uint32_t usd_data_size;
};

struct artery_flash_bank {
	bool probed;
	uint32_t idcode;
	const struct artery_part_info *part_info;
};

struct artery_series_info {
	bool has_fap_high_level;
	bool has_epp_ext;
	uint32_t flash_regs_base;
	const uint32_t *flash_regs;
	uint32_t crm_base;
	uint32_t usd_base;
	const uint32_t *usd_offsets;
};

#define ARTERY_USD_DATA_MAX_SIZE	2012

struct artery_usd {
	enum artery_fap_level fap_level;
	uint8_t ssb;
	uint32_t protection;
	uint32_t protection_ext;
	uint8_t data[ARTERY_USD_DATA_MAX_SIZE];
};

#define CRM_REG_CTRL		0x000

/* CRM_CTRL register bits. */
#define CRM_CTRL_HICKSTBL	BIT(0)
#define CRM_CTRL_HICKEN		BIT(1)

/* FLASH_CTRL register bits. */
#define FLASH_CTRL_USDULKS	BIT(9)
#define FLASH_CTRL_OPLK		BIT(7)
#define FLASH_CTRL_ERSTR	BIT(6)
#define FLASH_CTRL_USDERS	BIT(5)
#define FLASH_CTRL_USDPRGM	BIT(4)
#define FLASH_CTRL_BANKERS	BIT(2)
#define FLASH_CTRL_SECERS	BIT(1)
#define FLASH_CTRL_FPRGM	BIT(0)

/* FLASH_STS register bits. */
#define FLASH_STS_OBF		BIT(0)
#define FLASH_STS_PRGMERR	BIT(2)
#define FLASH_STS_EPPERR	BIT(4)
#define FLASH_STS_ODF		BIT(5)

/* FLASH_USD register bits. */
#define FLASH_USD_FAP		BIT(1)
#define FLASH_USD_FAP_HL	BIT(26)

#define FLASH_USD_SSB_OFFSET		2
#define FLASH_USD_USER_D0_OFFSET	10
#define FLASH_USD_USER_D1_OFFSET	18

/* Flash and USD unlock keys. */
#define KEY1	0x45670123
#define KEY2	0xCDEF89AB

#endif /* OPENOCD_FLASH_NOR_ARTERY */
