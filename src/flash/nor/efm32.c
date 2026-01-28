// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
 *                                                                         *
 *   Copyright (C) 2013 by Roman Dmitrienko                                *
 *   me@iamroman.org                                                       *
 *                                                                         *
 *   Copyright (C) 2014 Nemui Trinomius                                    *
 *   nemuisan_kawausogasuki@live.jp                                        *
 *                                                                         *
 *   Copyright (C) 2021 Doug Brunner                                       *
 *   doug.a.brunner@gmail.com                                              *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <target/cortex_m.h>

/* Datasheet specifies ~22ms for page erase on first chip generation. 100ms
 * provides reasonable margin.
 */
#define EFM32_FLASH_OPERATION_TIMEOUT   100

#define EFM32_FLASH_BASE                0

/* size in bytes, not words; must fit all Gecko devices */
#define LOCKWORDS_SZ                    512

#define EFM32_MSC_INFO_BASE             0x0fe00000
#define EFM32_MSC_USER_DATA             (EFM32_MSC_INFO_BASE + 0x0000)
#define EFM32_MSC_LOCK_BITS             (EFM32_MSC_INFO_BASE + 0x4000)
#define EFM32_MSC_LOCK_BITS_EXTRA       (EFM32_MSC_INFO_BASE + 0x4200)

struct efm32_dev_info_addr {
	target_addr_t part_num;
	target_addr_t part_rev;
#define EFM32_DI_PARTINFO_NUM_MASK        0x0000ffff
#define EFM32_DI_PARTINFO_FAMILY_MASK     0x00ff0000
#define EFM32_DI_PARTINFO_TYPE_MASK       0x3f000000
	target_addr_t part_info;
	target_addr_t page_size;
	target_addr_t flash_sz;
	target_addr_t ram_sz;
};

#define EFM32_DI_PART_FAMILY           (EFM32_MSC_INFO_BASE + 0x81fe)

static const struct efm32_dev_info_addr efm32_dev_info_addr[] = {
	[0] = {
		.part_num  = EFM32_MSC_INFO_BASE + 0x81fc,
		.part_rev  = EFM32_MSC_INFO_BASE + 0x81ff,
		.part_info = 0x00000000, // Not used in Series 0/1
		.page_size = EFM32_MSC_INFO_BASE + 0x81e7,
		.flash_sz  = EFM32_MSC_INFO_BASE + 0x81f8,
		.ram_sz    = EFM32_MSC_INFO_BASE + 0x81fa,
	},
	[1] = {
		.part_num  = EFM32_MSC_INFO_BASE + 0x81fc,
		.part_rev  = EFM32_MSC_INFO_BASE + 0x81ff,
		.part_info = 0x0000000, // Not used in Series 0/1
		.page_size = EFM32_MSC_INFO_BASE + 0x81e7,
		.flash_sz  = EFM32_MSC_INFO_BASE + 0x81f8,
		.ram_sz    = EFM32_MSC_INFO_BASE + 0x81fa,
	},
	[2] = {
		.part_num  = 0x00000000, // Not used in Series 2
		.part_rev  = EFM32_MSC_INFO_BASE + 0x8002,
		.part_info = EFM32_MSC_INFO_BASE + 0x8004,
		.page_size = EFM32_MSC_INFO_BASE + 0x8008,
		.flash_sz  = EFM32_MSC_INFO_BASE + 0x800c,
		.ram_sz    = EFM32_MSC_INFO_BASE + 0x800e,
	},
};

#define EFM32_MSC_REG_WRITECTRL              0x0008
#define EFM32_MSC_WRITECTRL_WREN_MASK        0x0001
#define EFM32_MSC_REG_WRITECMD               0x000c
#define EFM32_MSC_WRITECMD_LADDRIM_MASK      0x0001
#define EFM32_MSC_WRITECMD_ERASEPAGE_MASK    0x0002
#define EFM32_MSC_WRITECMD_WRITEONCE_MASK    0x0008
#define EFM32_MSC_REG_ADDRB                  0x0010
#define EFM32_MSC_REG_WDATA                  0x0018
#define EFM32_MSC_REG_STATUS                 0x001c
#define EFM32_MSC_STATUS_BUSY_MASK           0x0001
#define EFM32_MSC_STATUS_LOCKED_MASK         0x0002
#define EFM32_MSC_STATUS_INVADDR_MASK        0x0004
#define EFM32_MSC_STATUS_WDATAREADY_MASK     0x0008
#define EFM32_MSC_STATUS_WORDTIMEOUT_MASK    0x0010
#define EFM32_MSC_STATUS_ERASEABORTED_MASK   0x0020
#define EFM32_MSC_LOCK_LOCKKEY               0x1b71

enum efm32_bank_index {
	EFM32_BANK_INDEX_MAIN,
	EFM32_BANK_INDEX_USER_DATA,
	EFM32_BANK_INDEX_LOCK_BITS,
	EFM32_N_BANKS
};

static int efm32_get_bank_index(target_addr_t base)
{
	switch (base) {
	case EFM32_FLASH_BASE:
		return EFM32_BANK_INDEX_MAIN;
	case EFM32_MSC_USER_DATA:
		return EFM32_BANK_INDEX_USER_DATA;
	case EFM32_MSC_LOCK_BITS:
		return EFM32_BANK_INDEX_LOCK_BITS;
	default:
		return ERROR_FAIL;
	}
}

struct efm32_family_data {
	uint8_t part_id;
	int series;
	const char *name;
	uint32_t msc_regbase;

	// Page size in bytes, or 0 to read from msc_di->page_size
	int page_size;

};

struct efm32_info {
	const struct efm32_family_data *family_data;
	const struct efm32_dev_info_addr *di_addr;
	uint16_t part_num;     // Series 0/1 only
	uint32_t part_info;    // Series 2 only
	uint8_t  part_rev;
	uint16_t flash_sz_kib;
	uint16_t ram_sz_kib;
	uint16_t page_size;
};

struct efm32_flash_chip {
	struct efm32_info info;
	bool probed[EFM32_N_BANKS];
	uint32_t lb_page[LOCKWORDS_SZ / 4];
	uint32_t refcount;
};

static const struct efm32_family_data efm32_families[] = {
	{  16, 1, "EFR32MG1P Mighty",  .msc_regbase = 0x400e0000 },
	{  17, 1, "EFR32MG1B Mighty",  .msc_regbase = 0x400e0000 },
	{  18, 1, "EFR32MG1V Mighty",  .msc_regbase = 0x400e0000 },
	{  19, 1, "EFR32BG1P Blue",    .msc_regbase = 0x400e0000 },
	{  20, 1, "EFR32BG1B Blue",    .msc_regbase = 0x400e0000 },
	{  21, 1, "EFR32BG1V Blue",    .msc_regbase = 0x400e0000 },
	{  25, 1, "EFR32FG1P Flex",    .msc_regbase = 0x400e0000 },
	{  26, 1, "EFR32FG1B Flex",    .msc_regbase = 0x400e0000 },
	{  27, 1, "EFR32FG1V Flex",    .msc_regbase = 0x400e0000 },
	{  28, 1, "EFR32MG12P Mighty", .msc_regbase = 0x400e0000 },
	{  29, 1, "EFR32MG12B Mighty", .msc_regbase = 0x400e0000 },
	{  30, 1, "EFR32MG12V Mighty", .msc_regbase = 0x400e0000 },
	{  31, 1, "EFR32BG12P Blue",   .msc_regbase = 0x400e0000 },
	{  32, 1, "EFR32BG12B Blue",   .msc_regbase = 0x400e0000 },
	{  33, 1, "EFR32BG12V Blue",   .msc_regbase = 0x400e0000 },
	{  37, 1, "EFR32FG12P Flex",   .msc_regbase = 0x400e0000 },
	{  38, 1, "EFR32FG12B Flex",   .msc_regbase = 0x400e0000 },
	{  39, 1, "EFR32FG12V Flex",   .msc_regbase = 0x400e0000 },
	{  40, 1, "EFR32MG13P Mighty", .msc_regbase = 0x400e0000 },
	{  41, 1, "EFR32MG13B Mighty", .msc_regbase = 0x400e0000 },
	{  42, 1, "EFR32MG13V Mighty", .msc_regbase = 0x400e0000 },
	{  43, 1, "EFR32BG13P Blue",   .msc_regbase = 0x400e0000 },
	{  44, 1, "EFR32BG13B Blue",   .msc_regbase = 0x400e0000 },
	{  45, 1, "EFR32BG13V Blue",   .msc_regbase = 0x400e0000 },
	{  46, 1, "EFR32ZG13P Zen",    .msc_regbase = 0x400e0000 },
	{  49, 1, "EFR32FG13P Flex",   .msc_regbase = 0x400e0000 },
	{  50, 1, "EFR32FG13B Flex",   .msc_regbase = 0x400e0000 },
	{  51, 1, "EFR32FG13V Flex",   .msc_regbase = 0x400e0000 },
	{  52, 1, "EFR32MG14P Mighty", .msc_regbase = 0x400e0000 },
	{  53, 1, "EFR32MG14B Mighty", .msc_regbase = 0x400e0000 },
	{  54, 1, "EFR32MG14V Mighty", .msc_regbase = 0x400e0000 },
	{  55, 1, "EFR32BG14P Blue",   .msc_regbase = 0x400e0000 },
	{  56, 1, "EFR32BG14B Blue",   .msc_regbase = 0x400e0000 },
	{  57, 1, "EFR32BG14V Blue",   .msc_regbase = 0x400e0000 },
	{  58, 1, "EFR32ZG14P Zen",    .msc_regbase = 0x400e0000 },
	{  61, 1, "EFR32FG14P Flex",   .msc_regbase = 0x400e0000 },
	{  62, 1, "EFR32FG14B Flex",   .msc_regbase = 0x400e0000 },
	{  63, 1, "EFR32FG14V Flex",   .msc_regbase = 0x400e0000 },
	{  71, 0, "EFM32G",            .msc_regbase = 0x400c0000, .page_size = 512 },
	{  72, 0, "EFM32GG Giant",     .msc_regbase = 0x400c0000 },
	{  73, 0, "EFM32TG Tiny",      .msc_regbase = 0x400c0000, .page_size = 512 },
	{  74, 0, "EFM32LG Leopard",   .msc_regbase = 0x400c0000 },
	{  75, 0, "EFM32WG Wonder",    .msc_regbase = 0x400c0000 },
	{  76, 0, "EFM32ZG Zero",      .msc_regbase = 0x400c0000, .page_size = 1024 },
	{  77, 0, "EFM32HG Happy",     .msc_regbase = 0x400c0000, .page_size = 1024 },
	{  81, 1, "EFM32PG1B Pearl",   .msc_regbase = 0x400e0000 },
	{  83, 1, "EFM32JG1B Jade",    .msc_regbase = 0x400e0000 },
	{  85, 1, "EFM32PG12B Pearl",  .msc_regbase = 0x400e0000 },
	{  87, 1, "EFM32JG12B Jade",   .msc_regbase = 0x400e0000 },
	{  89, 1, "EFM32PG13B Pearl",  .msc_regbase = 0x400e0000 },
	{  91, 1, "EFM32JG13B Jade",   .msc_regbase = 0x400e0000 },
	{ 100, 1, "EFM32GG11B Giant",  .msc_regbase = 0x40000000 },
	{ 103, 1, "EFM32TG11B Tiny",   .msc_regbase = 0x40000000 },
	{ 106, 1, "EFM32GG12B Giant",  .msc_regbase = 0x40000000 },
	{ 120, 0, "EZR32WG Wonder",    .msc_regbase = 0x400c0000 },
	{ 121, 0, "EZR32LG Leopard",   .msc_regbase = 0x400c0000 },
	{ 122, 0, "EZR32HG Happy",     .msc_regbase = 0x400c0000, .page_size = 1024 },
	{ 128, 2, "EFR32/EFM32 Series-2", .msc_regbase = 0x50030000 },
};

const struct flash_driver efm32_flash;

static int efm32_priv_write(struct flash_bank *bank, const uint8_t *buffer,
			    uint32_t addr, uint32_t count);

static int efm32_write_only_lockbits(struct flash_bank *bank);

static int efm32_read_reg_u32(struct flash_bank *bank, target_addr_t offset,
			      uint32_t *value)
{
	struct efm32_flash_chip *efm32_info = bank->driver_priv;

	return target_read_u32(bank->target,
			       efm32_info->info.family_data->msc_regbase + offset,
			       value);
}

static int efm32_write_reg_u32(struct flash_bank *bank, target_addr_t offset,
			       uint32_t value)
{
	struct efm32_flash_chip *efm32_info = bank->driver_priv;

	return target_write_u32(bank->target,
				efm32_info->info.family_data->msc_regbase + offset,
				value);
}

static int efm32_read_info(struct flash_bank *bank)
{
	struct efm32_flash_chip *efm32_info = bank->driver_priv;
	struct efm32_info *efm32_mcu_info = &efm32_info->info;
	uint8_t tmp;
	int ret;

	memset(efm32_mcu_info, 0, sizeof(struct efm32_info));

	ret = target_read_u8(bank->target, EFM32_DI_PART_FAMILY, &tmp);
	if (ret != ERROR_OK)
		return ret;
	for (size_t i = 0; i < ARRAY_SIZE(efm32_families); i++) {
		if (efm32_families[i].part_id == tmp)
			efm32_mcu_info->family_data = &efm32_families[i];
	}
	if (!efm32_mcu_info->family_data) {
		LOG_ERROR("Unknown MCU family %d", tmp);
		return ERROR_FAIL;
	}

	efm32_mcu_info->di_addr = &efm32_dev_info_addr[efm32_mcu_info->family_data->series];

	if (efm32_mcu_info->family_data->series == 2) {
		ret = target_read_u32(bank->target,
				      efm32_mcu_info->di_addr->part_info,
				      &efm32_mcu_info->part_info);
		if (ret != ERROR_OK)
			return ret;
	} else {
		ret = target_read_u16(bank->target,
				      efm32_mcu_info->di_addr->part_num,
				      &efm32_mcu_info->part_num);
		if (ret != ERROR_OK)
			return ret;
	}

	ret = target_read_u8(bank->target,
			     efm32_mcu_info->di_addr->part_rev,
			     &efm32_mcu_info->part_rev);
	if (ret != ERROR_OK)
		return ret;

	ret = target_read_u16(bank->target,
			      efm32_mcu_info->di_addr->flash_sz,
			      &efm32_mcu_info->flash_sz_kib);
	if (ret != ERROR_OK)
		return ret;

	ret = target_read_u16(bank->target,
			      efm32_mcu_info->di_addr->ram_sz,
			      &efm32_mcu_info->ram_sz_kib);
	if (ret != ERROR_OK)
		return ret;

	if (efm32_mcu_info->family_data->page_size != 0) {
		efm32_mcu_info->page_size = efm32_mcu_info->family_data->page_size;
	} else if ((efm32_mcu_info->family_data->part_id == 72 ||
		    efm32_mcu_info->family_data->part_id == 74) &&
		   efm32_mcu_info->part_rev < 18) {
		/* EFM32 GG/LG errata: MEM_INFO_PAGE_SIZE is invalid for MCUs
		 * with part_rev < 18
		 */
		if (efm32_mcu_info->flash_sz_kib < 512)
			efm32_mcu_info->page_size = 2048;
		else
			efm32_mcu_info->page_size = 4096;
	} else {
		ret = target_read_u8(bank->target,
				     efm32_mcu_info->di_addr->page_size,
				     &tmp);
		if (ret != ERROR_OK)
			return ret;

		efm32_mcu_info->page_size = BIT(tmp) * 1024;
	}
	if (efm32_mcu_info->page_size !=  512 &&
	    efm32_mcu_info->page_size != 1024 &&
	    efm32_mcu_info->page_size != 2048 &&
	    efm32_mcu_info->page_size != 4096 &&
	    efm32_mcu_info->page_size != 8192) {
		LOG_ERROR("Invalid page size %u", efm32_mcu_info->page_size);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/* flash bank efm32 <base> <size> 0 0 <target#> */
FLASH_BANK_COMMAND_HANDLER(efm32_flash_bank_command)
{
	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	int bank_index = efm32_get_bank_index(bank->base);
	if (bank_index < 0) {
		LOG_ERROR("Flash bank with base address %" PRIx32 " is not supported",
			(uint32_t)bank->base);
		return ERROR_FAIL;
	}

	/* look for an existing flash structure matching target */
	struct efm32_flash_chip *efm32_info = NULL;
	for (struct flash_bank *bank_iter = flash_bank_list();
	     bank_iter;
	     bank_iter = bank_iter->next) {
		if (bank_iter->driver == &efm32_flash &&
		    bank_iter->target == bank->target &&
		    bank->driver_priv) {
			efm32_info = bank->driver_priv;
			break;
		}
	}

	if (!efm32_info) {
		/* target not matched, make a new one */
		efm32_info = calloc(1, sizeof(struct efm32_flash_chip));

		memset(efm32_info->lb_page, 0xff, LOCKWORDS_SZ);
	}

	++efm32_info->refcount;
	bank->driver_priv = efm32_info;

	return ERROR_OK;
}

/**
 * Remove flash structure corresponding to this bank, if and only if it's not
 * used by any others
 */
static void efm32_free_driver_priv(struct flash_bank *bank)
{
	struct efm32_flash_chip *efm32_info = bank->driver_priv;

	if (efm32_info) {
		/* Use ref count to determine if it can be freed; scanning bank
		 * list doesn't work, because this function can be called after
		 * some banks in the list have been already destroyed.
		 */
		--efm32_info->refcount;
		if (efm32_info->refcount == 0) {
			free(efm32_info);
			bank->driver_priv = NULL;
		}
	}
}

/* set or reset given bits in a register */
static int efm32_set_reg_bits(struct flash_bank *bank, uint32_t reg,
			      uint32_t bitmask, int set)
{
	int ret = 0;
	uint32_t reg_val = 0;

	ret = efm32_read_reg_u32(bank, reg, &reg_val);
	if (ret != ERROR_OK)
		return ret;

	if (set)
		reg_val |= bitmask;
	else
		reg_val &= ~bitmask;

	return efm32_write_reg_u32(bank, reg, reg_val);
}

static int efm32_set_wren(struct flash_bank *bank, int write_enable)
{
	return efm32_set_reg_bits(bank, EFM32_MSC_REG_WRITECTRL,
				  EFM32_MSC_WRITECTRL_WREN_MASK, write_enable);
}

static int efm32_msc_lock(struct flash_bank *bank, int lock)
{
	struct efm32_flash_chip *efm32_info = bank->driver_priv;
	struct efm32_info *efm32_mcu_info = &efm32_info->info;
	uint32_t val = lock ? 0 : EFM32_MSC_LOCK_LOCKKEY;
	uint32_t reg;

	if (efm32_mcu_info->family_data->series == 1)
		reg = 0x040;
	else
		reg = 0x03c;

	return efm32_write_reg_u32(bank, reg, val);
}

static int efm32_wait_status(struct flash_bank *bank, int timeout_ms,
			     uint32_t wait_mask, bool wait_for_set)
{
	int64_t start_ms = timeval_ms();
	uint32_t status = 0;

	while (1) {
		int ret = efm32_read_reg_u32(bank, EFM32_MSC_REG_STATUS, &status);
		if (ret != ERROR_OK)
			return ret;

		LOG_DEBUG("status: 0x%" PRIx32, status);

		if (!(status & wait_mask) && !wait_for_set)
			break;
		if ((status & wait_mask) && wait_for_set)
			break;

		if (timeval_ms() - start_ms > timeout_ms) {
			LOG_ERROR("timed out waiting for MSC status");
			return ERROR_FAIL;
		}

		alive_sleep(1);
	}

	if (status & EFM32_MSC_STATUS_ERASEABORTED_MASK)
		LOG_WARNING("page erase was aborted");

	return ERROR_OK;
}

static int efm32_erase_page(struct flash_bank *bank, uint32_t addr)
{
	/* this function DOES NOT set WREN; must be set already */
	/* 1. write address to ADDRB
	   2. write LADDRIM
	   3. check status (INVADDR, LOCKED)
	   4. write ERASEPAGE
	   5. wait until !STATUS_BUSY
	 */
	int ret = 0;
	uint32_t status = 0;

	LOG_DEBUG("erasing flash page at 0x%08" PRIx32, addr);

	ret = efm32_write_reg_u32(bank, EFM32_MSC_REG_ADDRB, addr);
	if (ret != ERROR_OK)
		return ret;

	ret = efm32_set_reg_bits(bank, EFM32_MSC_REG_WRITECMD,
				 EFM32_MSC_WRITECMD_LADDRIM_MASK, 1);
	if (ret != ERROR_OK)
		return ret;

	ret = efm32_read_reg_u32(bank, EFM32_MSC_REG_STATUS, &status);
	if (ret != ERROR_OK)
		return ret;

	LOG_DEBUG("status 0x%" PRIx32, status);

	if (status & EFM32_MSC_STATUS_LOCKED_MASK) {
		LOG_ERROR("Page is locked");
		return ERROR_FAIL;
	} else if (status & EFM32_MSC_STATUS_INVADDR_MASK) {
		LOG_ERROR("Invalid address 0x%" PRIx32, addr);
		return ERROR_FAIL;
	}

	ret = efm32_set_reg_bits(bank, EFM32_MSC_REG_WRITECMD,
				 EFM32_MSC_WRITECMD_ERASEPAGE_MASK, 1);
	if (ret != ERROR_OK)
		return ret;

	return efm32_wait_status(bank, EFM32_FLASH_OPERATION_TIMEOUT,
				 EFM32_MSC_STATUS_BUSY_MASK, false);
}

static int efm32_erase(struct flash_bank *bank, unsigned int first,
		       unsigned int last)
{
	struct target *target = bank->target;
	int ret = 0;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	efm32_msc_lock(bank, 0);
	ret = efm32_set_wren(bank, 1);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to enable MSC write");
		return ret;
	}

	for (unsigned int i = first; i <= last; i++) {
		ret = efm32_erase_page(bank, bank->base + bank->sectors[i].offset);
		if (ret != ERROR_OK)
			LOG_ERROR("Failed to erase page %d", i);
	}

	ret = efm32_set_wren(bank, 0);
	efm32_msc_lock(bank, 1);
	if (ret != ERROR_OK)
		return ret;

	if (bank->base == EFM32_MSC_LOCK_BITS) {
		ret = efm32_write_only_lockbits(bank);
		if (ret != ERROR_OK)
			LOG_ERROR("Failed to restore lockbits after erase");
	}

	return ret;
}

static int efm32_read_lock_data(struct flash_bank *bank)
{
	struct efm32_flash_chip *efm32_info = bank->driver_priv;
	struct target *target = bank->target;
	int data_size = 0;
	uint32_t *ptr = NULL;
	int ret = 0;

	assert(bank->num_sectors > 0);

	/* calculate the number of 32-bit words to read (one lock bit per sector) */
	data_size = (bank->num_sectors + 31) / 32;

	ptr = efm32_info->lb_page;

	for (int i = 0; i < data_size; i++, ptr++) {
		ret = target_read_u32(target, EFM32_MSC_LOCK_BITS + i * 4, ptr);
		if (ret != ERROR_OK) {
			LOG_ERROR("Failed to read PLW %d", i);
			return ret;
		}
	}

	/* also, read ULW, DLW, MLW, ALW and CLW words */

	/* ULW, word 126 */
	ptr = efm32_info->lb_page + 126;
	ret = target_read_u32(target, EFM32_MSC_LOCK_BITS + 126 * 4, ptr);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to read ULW");
		return ret;
	}

	/* DLW, word 127 */
	ptr = efm32_info->lb_page + 127;
	ret = target_read_u32(target, EFM32_MSC_LOCK_BITS + 127 * 4, ptr);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to read DLW");
		return ret;
	}

	/* MLW, word 125, present in GG, LG, PG, JG, EFR32 */
	ptr = efm32_info->lb_page + 125;
	ret = target_read_u32(target, EFM32_MSC_LOCK_BITS + 125 * 4, ptr);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to read MLW");
		return ret;
	}

	/* ALW, word 124, present in GG, LG, PG, JG, EFR32 */
	ptr = efm32_info->lb_page + 124;
	ret = target_read_u32(target, EFM32_MSC_LOCK_BITS + 124 * 4, ptr);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to read ALW");
		return ret;
	}

	/* CLW1, word 123, present in EFR32 */
	ptr = efm32_info->lb_page + 123;
	ret = target_read_u32(target, EFM32_MSC_LOCK_BITS + 123 * 4, ptr);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to read CLW1");
		return ret;
	}

	/* CLW0, word 122, present in GG, LG, PG, JG, EFR32 */
	ptr = efm32_info->lb_page + 122;
	ret = target_read_u32(target, EFM32_MSC_LOCK_BITS + 122 * 4, ptr);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to read CLW0");
		return ret;
	}

	return ERROR_OK;
}

static int efm32_write_only_lockbits(struct flash_bank *bank)
{
	struct efm32_flash_chip *efm32_info = bank->driver_priv;

	return efm32_priv_write(bank, (uint8_t *)efm32_info->lb_page,
				EFM32_MSC_LOCK_BITS, LOCKWORDS_SZ);
}

static int efm32_write_lock_data(struct flash_bank *bank)
{
	struct efm32_flash_chip *efm32_info = bank->driver_priv;

	/* Preserve any data written to the high portion of the lockbits page */
	assert(efm32_info->info.page_size >= LOCKWORDS_SZ);

	uint32_t extra_bytes = efm32_info->info.page_size - LOCKWORDS_SZ;
	uint8_t *extra_data = NULL;
	int ret;
	if (extra_bytes) {
		extra_data = malloc(extra_bytes);
		ret = target_read_buffer(bank->target,
					 EFM32_MSC_LOCK_BITS_EXTRA,
					 extra_bytes, extra_data);
		if (ret != ERROR_OK) {
			LOG_ERROR("Failed to read extra contents of LB page");
			free(extra_data);
			return ret;
		}
	}

	ret = efm32_erase_page(bank, EFM32_MSC_LOCK_BITS);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to erase LB page");
		if (extra_data)
			free(extra_data);
		return ret;
	}

	if (extra_data) {
		ret = efm32_priv_write(bank, extra_data,
				       EFM32_MSC_LOCK_BITS_EXTRA,
				       extra_bytes);
		free(extra_data);
		if (ret != ERROR_OK) {
			LOG_ERROR("Failed to restore extra contents of LB page");
			return ret;
		}
	}

	return efm32_write_only_lockbits(bank);
}

static int efm32_get_page_lock(struct flash_bank *bank, size_t page)
{
	struct efm32_flash_chip *efm32_info = bank->driver_priv;
	uint32_t dw = 0;
	uint32_t mask = 0;

	switch (bank->base) {
	case EFM32_FLASH_BASE:
		dw = efm32_info->lb_page[page >> 5];
		mask = BIT(page & 0x1f);
		break;
	case EFM32_MSC_USER_DATA:
		dw = efm32_info->lb_page[126];
		mask = BIT(0);
		break;
	case EFM32_MSC_LOCK_BITS:
		dw = efm32_info->lb_page[126];
		mask = BIT(1);
		break;
	}

	return (dw & mask) ? 0 : 1;
}

static int efm32_set_page_lock(struct flash_bank *bank, size_t page, int set)
{
	struct efm32_flash_chip *efm32_info = bank->driver_priv;

	if (bank->base != EFM32_FLASH_BASE) {
		LOG_ERROR("Locking user and lockbits pages is not supported yet");
		return ERROR_FAIL;
	}

	uint32_t *dw = &efm32_info->lb_page[page >> 5];
	uint32_t mask = BIT(page & 0x1f);
	if (!set)
		*dw |= mask;
	else
		*dw &= ~mask;

	return ERROR_OK;
}

static int efm32_protect(struct flash_bank *bank, int set, unsigned int first,
			 unsigned int last)
{
	struct target *target = bank->target;
	int ret = 0;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	for (unsigned int i = first; i <= last; i++) {
		ret = efm32_set_page_lock(bank, i, set);
		if (ret != ERROR_OK) {
			LOG_ERROR("Failed to set lock on page %d", i);
			return ret;
		}
	}

	ret = efm32_write_lock_data(bank);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to write LB page");
		return ret;
	}

	return ERROR_OK;
}

static int efm32_write_block(struct flash_bank *bank, const uint8_t *buf,
			     uint32_t address, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t buffer_size = 16384;
	struct working_area *write_algorithm;
	struct working_area *source;
	struct reg_param reg_params[5];
	struct armv7m_algorithm armv7m_info;
	struct efm32_flash_chip *efm32_info = bank->driver_priv;
	int ret = ERROR_OK;

	/* see contrib/loaders/flash/efm32.S for src */
	static const uint8_t efm32_flash_write_code[] = {
		/* #define EFM32_MSC_WRITECTRL_OFFSET      0x008 */
		/* #define EFM32_MSC_WRITECMD_OFFSET       0x00c */
		/* #define EFM32_MSC_ADDRB_OFFSET          0x010 */
		/* #define EFM32_MSC_WDATA_OFFSET          0x018 */
		/* #define EFM32_MSC_STATUS_OFFSET         0x01c */

			0x01, 0x26,    /* movs    r6, #1 */
			0x86, 0x60,    /* str     r6, [r0, #EFM32_MSC_WRITECTRL_OFFSET] */

		/* wait_fifo: */
			0x16, 0x68,    /* ldr     r6, [r2, #0] */
			0x00, 0x2e,    /* cmp     r6, #0 */
			0x22, 0xd0,    /* beq     exit */
			0x55, 0x68,    /* ldr     r5, [r2, #4] */
			0xb5, 0x42,    /* cmp     r5, r6 */
			0xf9, 0xd0,    /* beq     wait_fifo */

			0x04, 0x61,    /* str     r4, [r0, #EFM32_MSC_ADDRB_OFFSET] */
			0x01, 0x26,    /* movs    r6, #1 */
			0xc6, 0x60,    /* str     r6, [r0, #EFM32_MSC_WRITECMD_OFFSET] */
			0xc6, 0x69,    /* ldr     r6, [r0, #EFM32_MSC_STATUS_OFFSET] */
			0x06, 0x27,    /* movs    r7, #6 */
			0x3e, 0x42,    /* tst     r6, r7 */
			0x16, 0xd1,    /* bne     error */

		/* wait_wdataready: */
			0xc6, 0x69,    /* ldr     r6, [r0, #EFM32_MSC_STATUS_OFFSET] */
			0x08, 0x27,    /* movs    r7, #8 */
			0x3e, 0x42,    /* tst     r6, r7 */
			0xfb, 0xd0,    /* beq     wait_wdataready */

			0x2e, 0x68,    /* ldr     r6, [r5] */
			0x86, 0x61,    /* str     r6, [r0, #EFM32_MSC_WDATA_OFFSET] */
			0x08, 0x26,    /* movs    r6, #8 */
			0xc6, 0x60,    /* str     r6, [r0, #EFM32_MSC_WRITECMD_OFFSET] */

			0x04, 0x35,    /* adds    r5, #4 */
			0x04, 0x34,    /* adds    r4, #4 */

		/* busy: */
			0xc6, 0x69,    /* ldr     r6, [r0, #EFM32_MSC_STATUS_OFFSET] */
			0x01, 0x27,    /* movs    r7, #1 */
			0x3e, 0x42,    /* tst     r6, r7 */
			0xfb, 0xd1,    /* bne     busy */

			0x9d, 0x42,    /* cmp     r5, r3 */
			0x01, 0xd3,    /* bcc     no_wrap */
			0x15, 0x46,    /* mov     r5, r2 */
			0x08, 0x35,    /* adds    r5, #8 */

		/* no_wrap: */
			0x55, 0x60,    /* str     r5, [r2, #4] */
			0x01, 0x39,    /* subs    r1, r1, #1 */
			0x00, 0x29,    /* cmp     r1, #0 */
			0x02, 0xd0,    /* beq     exit */
			0xdb, 0xe7,    /* b       wait_fifo */

		/* error: */
			0x00, 0x20,    /* movs    r0, #0 */
			0x50, 0x60,    /* str     r0, [r2, #4] */

		/* exit: */
			0x30, 0x46,    /* mov     r0, r6 */
			0x00, 0xbe,    /* bkpt    #0 */
	};

	/* flash write code */
	if (target_alloc_working_area(target, sizeof(efm32_flash_write_code),
				      &write_algorithm) != ERROR_OK) {
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	ret = target_write_buffer(target, write_algorithm->address,
				  sizeof(efm32_flash_write_code),
				  efm32_flash_write_code);
	if (ret != ERROR_OK)
		return ret;

	/* memory buffer */
	while (target_alloc_working_area_try(target, buffer_size, &source) != ERROR_OK) {
		buffer_size /= 2;
		buffer_size &= ~3UL; /* Make sure it's 4 byte aligned */
		if (buffer_size <= 256) {
			/* we already allocated the writing code, but failed to get a
			 * buffer, free the algorithm */
			target_free_working_area(target, write_algorithm);

			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);	/* flash base (in), status (out) */
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);	/* count (word-32bit) */
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);	/* buffer start */
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);	/* buffer end */
	init_reg_param(&reg_params[4], "r4", 32, PARAM_IN_OUT);	/* target address */

	buf_set_u32(reg_params[0].value, 0, 32, efm32_info->info.family_data->msc_regbase);
	buf_set_u32(reg_params[1].value, 0, 32, count);
	buf_set_u32(reg_params[2].value, 0, 32, source->address);
	buf_set_u32(reg_params[3].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[4].value, 0, 32, address);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	ret = target_run_flash_async_algorithm(target, buf, count, 4,
					       0, NULL,
					       5, reg_params,
					       source->address, source->size,
					       write_algorithm->address, 0,
					       &armv7m_info);

	if (ret == ERROR_FLASH_OPERATION_FAILED) {
		LOG_ERROR("flash write failed at address 0x%" PRIx32,
			  buf_get_u32(reg_params[4].value, 0, 32));

		if (buf_get_u32(reg_params[0].value, 0, 32) &
				EFM32_MSC_STATUS_LOCKED_MASK) {
			LOG_ERROR("flash memory write protected");
		}

		if (buf_get_u32(reg_params[0].value, 0, 32) &
				EFM32_MSC_STATUS_INVADDR_MASK) {
			LOG_ERROR("invalid flash memory write address");
		}
	}

	target_free_working_area(target, source);
	target_free_working_area(target, write_algorithm);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	return ret;
}

static int efm32_write_word(struct flash_bank *bank, uint32_t addr,
			    uint32_t val)
{
	/* this function DOES NOT set WREN; must be set already */
	/* 1. write address to ADDRB
	   2. write LADDRIM
	   3. check status (INVADDR, LOCKED)
	   4. wait for WDATAREADY
	   5. write data to WDATA
	   6. write WRITECMD_WRITEONCE to WRITECMD
	   7. wait until !STATUS_BUSY
	 */

	/* FIXME: EFM32G ref states (7.3.2) that writes should be
	 * performed twice per dword */

	int ret = 0;
	uint32_t status = 0;

	/* if not called, GDB errors will be reported during large writes */
	keep_alive();

	ret = efm32_write_reg_u32(bank, EFM32_MSC_REG_ADDRB, addr);
	if (ret != ERROR_OK)
		return ret;

	ret = efm32_set_reg_bits(bank, EFM32_MSC_REG_WRITECMD,
				 EFM32_MSC_WRITECMD_LADDRIM_MASK, 1);
	if (ret != ERROR_OK)
		return ret;

	ret = efm32_read_reg_u32(bank, EFM32_MSC_REG_STATUS, &status);
	if (ret != ERROR_OK)
		return ret;

	LOG_DEBUG("status 0x%" PRIx32, status);

	if (status & EFM32_MSC_STATUS_LOCKED_MASK) {
		LOG_ERROR("Page is locked");
		return ERROR_FAIL;
	} else if (status & EFM32_MSC_STATUS_INVADDR_MASK) {
		LOG_ERROR("Invalid address 0x%" PRIx32, addr);
		return ERROR_FAIL;
	}

	ret = efm32_wait_status(bank, EFM32_FLASH_OPERATION_TIMEOUT,
				EFM32_MSC_STATUS_WDATAREADY_MASK, true);
	if (ret != ERROR_OK) {
		LOG_ERROR("Wait for WDATAREADY failed");
		return ret;
	}

	ret = efm32_write_reg_u32(bank, EFM32_MSC_REG_WDATA, val);
	if (ret != ERROR_OK) {
		LOG_ERROR("WDATA write failed");
		return ret;
	}

	ret = efm32_write_reg_u32(bank, EFM32_MSC_REG_WRITECMD,
				  EFM32_MSC_WRITECMD_WRITEONCE_MASK);
	if (ret != ERROR_OK) {
		LOG_ERROR("WRITECMD write failed");
		return ret;
	}

	ret = efm32_wait_status(bank, EFM32_FLASH_OPERATION_TIMEOUT,
				EFM32_MSC_STATUS_BUSY_MASK, false);
	if (ret != ERROR_OK) {
		LOG_ERROR("Wait for BUSY failed");
		return ret;
	}

	return ERROR_OK;
}

static int efm32_priv_write(struct flash_bank *bank, const uint8_t *buffer,
			    uint32_t addr, uint32_t count)
{
	struct target *target = bank->target;
	uint8_t *new_buffer = NULL;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (addr & 0x3) {
		LOG_ERROR("addr 0x%" PRIx32 " breaks required 4-byte alignment", addr);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	if (count & 0x3) {
		uint32_t old_count = count;
		count = (old_count | 3) + 1;
		new_buffer = malloc(count);
		if (!new_buffer) {
			LOG_ERROR("odd number of bytes to write and no memory for padding buffer");
			return ERROR_FAIL;
		}
		LOG_INFO("odd number of bytes to write (%" PRIu32 "), extending to %"
			 PRIu32 " and padding with 0xff", old_count, count);
		memset(new_buffer, 0xff, count);
		buffer = memcpy(new_buffer, buffer, old_count);
	}

	uint32_t words_remaining = count / 4;
	int retval, retval2;

	/* unlock flash registers */
	efm32_msc_lock(bank, 0);
	retval = efm32_set_wren(bank, 1);
	if (retval != ERROR_OK)
		goto cleanup;

	/* try using a block write */
	retval = efm32_write_block(bank, buffer, addr, words_remaining);

	if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
		/* if block write failed (no sufficient working area),
		 * we use normal (slow) single word accesses */
		LOG_WARNING("couldn't use block writes, falling back to single memory accesses");

		while (words_remaining > 0) {
			uint32_t value;
			memcpy(&value, buffer, sizeof(uint32_t));

			retval = efm32_write_word(bank, addr, value);
			if (retval != ERROR_OK)
				goto reset_pg_and_lock;

			words_remaining--;
			buffer += 4;
			addr += 4;
		}
	}

reset_pg_and_lock:
	retval2 = efm32_set_wren(bank, 0);
	efm32_msc_lock(bank, 1);
	if (retval == ERROR_OK)
		retval = retval2;

cleanup:
	free(new_buffer);
	return retval;
}

static int efm32_write(struct flash_bank *bank, const uint8_t *buffer,
		       uint32_t offset, uint32_t count)
{
	if (bank->base == EFM32_MSC_LOCK_BITS && offset < LOCKWORDS_SZ) {
		LOG_ERROR("Cannot write to lock words");
		return ERROR_FAIL;
	}
	return efm32_priv_write(bank, buffer, bank->base + offset, count);
}

static int efm32_probe(struct flash_bank *bank)
{
	struct efm32_flash_chip *efm32_info = bank->driver_priv;
	struct efm32_info *efm32_mcu_info = &efm32_info->info;
	int ret;
	int bank_index = efm32_get_bank_index(bank->base);

	assert(bank_index >= 0);

	efm32_info->probed[bank_index] = false;
	memset(efm32_info->lb_page, 0xff, LOCKWORDS_SZ);

	ret = efm32_read_info(bank);
	if (ret != ERROR_OK)
		return ret;

	LOG_INFO("detected part: %s Gecko, rev %d",
		 efm32_mcu_info->family_data->name, efm32_mcu_info->part_rev);
	LOG_INFO("flash size = %d KiB", efm32_mcu_info->flash_sz_kib);
	LOG_INFO("flash page size = %d B", efm32_mcu_info->page_size);

	assert(efm32_mcu_info->page_size != 0);

	free(bank->sectors);
	bank->sectors = NULL;

	if (bank->base == EFM32_FLASH_BASE) {
		bank->num_sectors = efm32_mcu_info->flash_sz_kib * 1024 / efm32_mcu_info->page_size;
		assert(bank->num_sectors > 0);

		ret = efm32_read_lock_data(bank);
		if (ret != ERROR_OK) {
			LOG_ERROR("Failed to read LB data");
			return ret;
		}
	} else {
		bank->num_sectors = 1;
	}
	bank->size = bank->num_sectors * efm32_mcu_info->page_size;
	bank->sectors = alloc_block_array(0,
					  efm32_mcu_info->page_size,
					  bank->num_sectors);
	if (!bank->sectors)
		return ERROR_FAIL;

	efm32_info->probed[bank_index] = true;

	return ERROR_OK;
}

static int efm32_auto_probe(struct flash_bank *bank)
{
	struct efm32_flash_chip *efm32_info = bank->driver_priv;
	int bank_index = efm32_get_bank_index(bank->base);

	assert(bank_index >= 0);

	if (efm32_info->probed[bank_index])
		return ERROR_OK;
	return efm32_probe(bank);
}

static int efm32_protect_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	int ret = 0;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	ret = efm32_read_lock_data(bank);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to read LB data");
		return ret;
	}

	assert(bank->sectors);

	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = efm32_get_page_lock(bank, i);

	return ERROR_OK;
}

static int efm32_get_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct efm32_flash_chip *efm32_info = bank->driver_priv;
	int ret;

	ret = efm32_read_info(bank);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to read EFM32 info");
		return ret;
	}

	command_print_sameline(cmd, "%s Gecko, rev %d",
			       efm32_info->info.family_data->name,
			       efm32_info->info.part_rev);
	return ERROR_OK;
}

COMMAND_HANDLER(efm32_handle_debuglock_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int ret = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ret != ERROR_OK)
		return ret;

	struct efm32_flash_chip *efm32_info = bank->driver_priv;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	uint32_t *ptr = efm32_info->lb_page + 127;
	*ptr = 0;

	ret = efm32_write_lock_data(bank);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to write LB page");
		return ret;
	}

	command_print(CMD, "efm32 debug interface locked, reset the device to apply");

	return ERROR_OK;
}

static const struct command_registration efm32_exec_command_handlers[] = {
	{
		.name = "debuglock",
		.handler = efm32_handle_debuglock_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Lock the debug interface of the device.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration efm32_command_handlers[] = {
	{
		.name = "efm32",
		.mode = COMMAND_ANY,
		.help = "efm32 flash command group",
		.usage = "",
		.chain = efm32_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver efm32_flash = {
	.name               = "efm32",
	.commands           = efm32_command_handlers,
	.flash_bank_command = efm32_flash_bank_command,
	.erase              = efm32_erase,
	.protect            = efm32_protect,
	.write              = efm32_write,
	.read               = default_flash_read,
	.probe              = efm32_probe,
	.auto_probe         = efm32_auto_probe,
	.erase_check        = default_flash_blank_check,
	.protect_check      = efm32_protect_check,
	.info               = efm32_get_info,
	.free_driver_priv   = efm32_free_driver_priv,
};
