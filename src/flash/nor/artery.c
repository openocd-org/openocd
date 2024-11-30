// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Copyright (C) 2023 by Marc Schink <dev@zapb.de>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/align.h>
#include <helper/binarybuffer.h>
#include <helper/time_support.h>
#include <helper/bits.h>
#include <target/cortex_m.h>

#include "artery.h"

// Flash timeout values in milliseconds.
#define FLASH_MASS_ERASE_TIMEOUT	2400
#define FLASH_ERASE_TIMEOUT			500
#define FLASH_WRITE_TIMEOUT			5
#define HICK_STABLE_TIMEOUT			1000

/*
 * Flash memory register assignment for the following device series:
 * - AT32F403A / AT32F407
 * - AT32F413
 * - AT32F415
 * - AT32F421
 * - AT32F423
 * - AT32F425
 * - AT32WB415
 */
static const uint32_t flash_regs_f4xx_wb415[ARTERY_FLASH_REG_INDEX_NUM] = {
	[ARTERY_FLASH_REG_PSR] = 0x00,
	[ARTERY_FLASH_REG_UNLOCK] = 0x04,
	[ARTERY_FLASH_REG_USD_UNLOCK] = 0x08,
	[ARTERY_FLASH_REG_STS] = 0x0c,
	[ARTERY_FLASH_REG_CTRL] = 0x10,
	[ARTERY_FLASH_REG_ADDR] = 0x14,
	[ARTERY_FLASH_REG_USD] = 0x1c,
	[ARTERY_FLASH_REG_EPPS0] = 0x20,
	// [ARTERY_FLASH_REG_EPPS1] not available.
};

// Flash memory register assignment for the AT32F435 / AT32F437 series.
static const uint32_t flash_regs_f435_f437[ARTERY_FLASH_REG_INDEX_NUM] = {
	[ARTERY_FLASH_REG_PSR] = 0x00,
	[ARTERY_FLASH_REG_UNLOCK] = 0x04,
	[ARTERY_FLASH_REG_USD_UNLOCK] = 0x08,
	[ARTERY_FLASH_REG_STS] = 0x0c,
	[ARTERY_FLASH_REG_CTRL] = 0x10,
	[ARTERY_FLASH_REG_ADDR] = 0x14,
	[ARTERY_FLASH_REG_USD] = 0x1c,
	[ARTERY_FLASH_REG_EPPS0] = 0x20,
	[ARTERY_FLASH_REG_EPPS1] = 0x2c,
};

/*
 * User system data (USD) offsets for the following device series:
 * - AT32F415
 * - AT32F421
 * - AT32F423
 * - AT32F425
 * - AT32WB415
 */
static const uint32_t usd_offsets_f4xx_wb415[ARTERY_USD_INDEX_NUM] = {
	[ARTERY_USD_FAP_INDEX] = 0x00,
	[ARTERY_USD_SSB_INDEX] = 0x02,
	[ARTERY_USD_DATA_INDEX] = 0x04,
	[ARTERY_USD_EPP_INDEX] = 0x08,
	// [ARTERY_USD_EPP_EXT_INDEX] not available.
	[ARTERY_USD_DATA_EXT_INDEX] = 0x10,
};

// User system data (USD) offsets for the AT32F403A / AT32F407 / AT32F413 series.
static const uint32_t usd_offsets_f403a_f407_f413[ARTERY_USD_INDEX_NUM] = {
	[ARTERY_USD_FAP_INDEX] = 0x00,
	[ARTERY_USD_SSB_INDEX] = 0x02,
	[ARTERY_USD_DATA_INDEX] = 0x04,
	[ARTERY_USD_EPP_INDEX] = 0x08,
	// [ARTERY_USD_EPP_EXT_INDEX] not available.
	[ARTERY_USD_DATA_EXT_INDEX] = 0x14,
};

// User system data (USD) offsets for the AT32F435 / AT32F437 series.
static const uint32_t usd_offsets_f435_f437[ARTERY_USD_INDEX_NUM] = {
	[ARTERY_USD_FAP_INDEX] = 0x00,
	[ARTERY_USD_SSB_INDEX] = 0x02,
	[ARTERY_USD_DATA_INDEX] = 0x04,
	[ARTERY_USD_EPP_INDEX] = 0x08,
	[ARTERY_USD_EPP_EXT_INDEX] = 0x14,
	[ARTERY_USD_DATA_EXT_INDEX] = 0x4c,
};

static const struct artery_series_info artery_series[] = {
	[ARTERY_SERIES_F403A_F407] = {
		.has_fap_high_level = false,
		.has_epp_ext = false,
		.flash_regs_base = 0x40022000,
		.flash_regs = flash_regs_f4xx_wb415,
		.crm_base = 0x40021000,
		.usd_base = 0x1FFFF800,
		.usd_offsets = usd_offsets_f403a_f407_f413,
	},
	[ARTERY_SERIES_F413] = {
		.has_fap_high_level = false,
		.has_epp_ext = false,
		.flash_regs_base = 0x40022000,
		.flash_regs = flash_regs_f4xx_wb415,
		.crm_base = 0x40021000,
		.usd_base = 0x1FFFF800,
		.usd_offsets = usd_offsets_f403a_f407_f413,
	},
	[ARTERY_SERIES_F415] = {
		.has_fap_high_level = true,
		.has_epp_ext = false,
		.flash_regs_base = 0x40022000,
		.flash_regs = flash_regs_f4xx_wb415,
		.crm_base = 0x40021000,
		.usd_base = 0x1FFFF800,
		.usd_offsets = usd_offsets_f4xx_wb415,
	},
	[ARTERY_SERIES_F421] = {
		.has_fap_high_level = true,
		.has_epp_ext = false,
		.flash_regs_base = 0x40022000,
		.flash_regs = flash_regs_f4xx_wb415,
		.crm_base = 0x40021000,
		.usd_base = 0x1FFFF800,
		.usd_offsets = usd_offsets_f4xx_wb415,
	},
	[ARTERY_SERIES_F423] = {
		.has_fap_high_level = true,
		.has_epp_ext = false,
		.flash_regs_base = 0x40023C00,
		.flash_regs = flash_regs_f4xx_wb415,
		.crm_base = 0x40023800,
		.usd_base = 0x1FFFF800,
		.usd_offsets = usd_offsets_f4xx_wb415,
	},
	[ARTERY_SERIES_F425] = {
		.has_fap_high_level = true,
		.has_epp_ext = false,
		.flash_regs_base = 0x40022000,
		.flash_regs = flash_regs_f4xx_wb415,
		.crm_base = 0x40021000,
		.usd_base = 0x1FFFF800,
		.usd_offsets = usd_offsets_f4xx_wb415,
	},
	[ARTERY_SERIES_F435_F437] = {
		.has_fap_high_level = false,
		.has_epp_ext = true,
		.flash_regs_base = 0x40023C00,
		.flash_regs = flash_regs_f435_f437,
		.crm_base = 0x40023800,
		.usd_base = 0x1FFFC000,
		.usd_offsets = usd_offsets_f435_f437,
	},
	[ARTERY_SERIES_WB415] = {
		.has_fap_high_level = true,
		.has_epp_ext = false,
		.flash_regs_base = 0x40022000,
		.flash_regs = flash_regs_f4xx_wb415,
		.crm_base = 0x40021000,
		.usd_base = 0x1FFFF800,
		.usd_offsets = usd_offsets_f4xx_wb415,
	},
};

static const struct artery_part_info artery_parts[] = {
	{
	    .pid = 0x70050240,
	    .name = "AT32F403AVCT7",
	    .series = ARTERY_SERIES_F403A_F407,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 48,
	    .usd_data_size = 8,
	},
	{
	    .pid = 0x70050241,
	    .name = "AT32F403ARCT7",
	    .series = ARTERY_SERIES_F403A_F407,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 48,
	    .usd_data_size = 8,
	},
	{
	    .pid = 0x70050242,
	    .name = "AT32F403ACCT7",
	    .series = ARTERY_SERIES_F403A_F407,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 48,
	    .usd_data_size = 8,
	},
	{
	    .pid = 0x70050243,
	    .name = "AT32F403ACCU7",
	    .series = ARTERY_SERIES_F403A_F407,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 48,
	    .usd_data_size = 8,
	},
	{
	    .pid = 0x70050249,
	    .name = "AT32F407VCT7",
	    .series = ARTERY_SERIES_F403A_F407,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 48,
	    .usd_data_size = 8,
	},
	{
	    .pid = 0x7005024a,
	    .name = "AT32F407RCT7",
	    .series = ARTERY_SERIES_F403A_F407,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 48,
	    .usd_data_size = 8,
	},
	{
	    .pid = 0x700502cd,
	    .name = "AT32F403AVET7",
	    .series = ARTERY_SERIES_F403A_F407,
	    .flash_size = 512,
	    .page_size = 2048,
	    .usd_size = 48,
	    .usd_data_size = 8,
	},
	{
	    .pid = 0x700502ce,
	    .name = "AT32F403ARET7",
	    .series = ARTERY_SERIES_F403A_F407,
	    .flash_size = 512,
	    .page_size = 2048,
	    .usd_size = 48,
	    .usd_data_size = 8,
	},
	{
	    .pid = 0x700502cf,
	    .name = "AT32F403ACET7",
	    .series = ARTERY_SERIES_F403A_F407,
	    .flash_size = 512,
	    .page_size = 2048,
	    .usd_size = 48,
	    .usd_data_size = 8,
	},
	{
	    .pid = 0x700502d0,
	    .name = "AT32F403ACEU7",
	    .series = ARTERY_SERIES_F403A_F407,
	    .flash_size = 512,
	    .page_size = 2048,
	    .usd_size = 48,
	    .usd_data_size = 8,
	},
	{
	    .pid = 0x700502d1,
	    .name = "AT32F407VET7",
	    .series = ARTERY_SERIES_F403A_F407,
	    .flash_size = 512,
	    .page_size = 2048,
	    .usd_size = 48,
	    .usd_data_size = 8,
	},
	{
	    .pid = 0x700502d2,
	    .name = "AT32F407RET7",
	    .series = ARTERY_SERIES_F403A_F407,
	    .flash_size = 512,
	    .page_size = 2048,
	    .usd_size = 48,
	    .usd_data_size = 8,
	},
	{
	    .pid = 0x70050254,
	    .name = "AT32F407AVCT7",
	    .series = ARTERY_SERIES_F403A_F407,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 48,
	    .usd_data_size = 8,
	},
	{
	    .pid = 0x70030240,
	    .name = "AT32F413RCT7",
	    .series = ARTERY_SERIES_F413,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 48,
	    .usd_data_size = 8,
	},
	{
	    .pid = 0x700301c1,
	    .name = "AT32F413RBT7",
	    .series = ARTERY_SERIES_F413,
	    .flash_size = 128,
	    .page_size = 1024,
	    .usd_size = 48,
	    .usd_data_size = 8,
	},
	{
	    .pid = 0x70030242,
	    .name = "AT32F413CCT7",
	    .series = ARTERY_SERIES_F413,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 48,
	    .usd_data_size = 8,
	},
	{
	    .pid = 0x700301c3,
	    .name = "AT32F413CBT7",
	    .series = ARTERY_SERIES_F413,
	    .flash_size = 128,
	    .page_size = 1024,
	    .usd_size = 48,
	    .usd_data_size = 8,
	},
	{
	    .pid = 0x70030244,
	    .name = "AT32F413KCU7-4",
	    .series = ARTERY_SERIES_F413,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 48,
	    .usd_data_size = 8,
	},
	{
	    .pid = 0x700301c5,
	    .name = "AT32F413KBU7-4",
	    .series = ARTERY_SERIES_F413,
	    .flash_size = 128,
	    .page_size = 1024,
	    .usd_size = 48,
	    .usd_data_size = 8,
	},
	{
	    .pid = 0x70030106,
	    .name = "AT32F413C8T7",
	    .series = ARTERY_SERIES_F413,
	    .flash_size = 64,
	    .page_size = 1024,
	    .usd_size = 48,
	    .usd_data_size = 8,
	},
	{
	    .pid = 0x70030247,
	    .name = "AT32F413CCU7",
	    .series = ARTERY_SERIES_F413,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 48,
	    .usd_data_size = 8,
	},
	{
	    .pid = 0x700301ca,
	    .name = "AT32F413CBU7",
	    .series = ARTERY_SERIES_F413,
	    .flash_size = 128,
	    .page_size = 1024,
	    .usd_size = 48,
	    .usd_data_size = 8,
	},
	{
	    .pid = 0x70030240,
	    .name = "AT32F415RCT7",
	    .series = ARTERY_SERIES_F415,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 1024,
	    .usd_data_size = 506,
	},
	{
	    .pid = 0x70030241,
	    .name = "AT32F415CCT7",
	    .series = ARTERY_SERIES_F415,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 1024,
	    .usd_data_size = 506,
	},
	{
	    .pid = 0x70030242,
	    .name = "AT32F415KCU7-4",
	    .series = ARTERY_SERIES_F415,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 1024,
	    .usd_data_size = 506,
	},
	{
	    .pid = 0x70030243,
	    .name = "AT32F415RCT7-7",
	    .series = ARTERY_SERIES_F415,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 1024,
	    .usd_data_size = 506,
	},
	{
	    .pid = 0x700301c4,
	    .name = "AT32F415RBT7",
	    .series = ARTERY_SERIES_F415,
	    .flash_size = 128,
	    .page_size = 1024,
	    .usd_size = 1024,
	    .usd_data_size = 506,
	},
	{
	    .pid = 0x700301c5,
	    .name = "AT32F415CBT7",
	    .series = ARTERY_SERIES_F415,
	    .flash_size = 128,
	    .page_size = 1024,
	    .usd_size = 1024,
	    .usd_data_size = 506,
	},
	{
	    .pid = 0x700301c6,
	    .name = "AT32F415KBU7-4",
	    .series = ARTERY_SERIES_F415,
	    .flash_size = 128,
	    .page_size = 1024,
	    .usd_size = 1024,
	    .usd_data_size = 506,
	},
	{
	    .pid = 0x700301c7,
	    .name = "AT32F415RBT7-7",
	    .series = ARTERY_SERIES_F415,
	    .flash_size = 128,
	    .page_size = 1024,
	    .usd_size = 1024,
	    .usd_data_size = 506,
	},
	{
	    .pid = 0x70030108,
	    .name = "AT32F415R8T7",
	    .series = ARTERY_SERIES_F415,
	    .flash_size = 64,
	    .page_size = 1024,
	    .usd_size = 1024,
	    .usd_data_size = 506,
	},
	{
	    .pid = 0x70030109,
	    .name = "AT32F415C8T7",
	    .series = ARTERY_SERIES_F415,
	    .flash_size = 64,
	    .page_size = 1024,
	    .usd_size = 1024,
	    .usd_data_size = 506,
	},
	{
	    .pid = 0x7003010a,
	    .name = "AT32F415K8U7-4",
	    .series = ARTERY_SERIES_F415,
	    .flash_size = 64,
	    .page_size = 1024,
	    .usd_size = 1024,
	    .usd_data_size = 506,
	},
	{
	    .pid = 0x7003024c,
	    .name = "AT32F415CCU7",
	    .series = ARTERY_SERIES_F415,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 1024,
	    .usd_data_size = 506,
	},
	{
	    .pid = 0x700301cd,
	    .name = "AT32F415CBU7",
	    .series = ARTERY_SERIES_F415,
	    .flash_size = 128,
	    .page_size = 1024,
	    .usd_size = 1024,
	    .usd_data_size = 506,
	},
	{
	    .pid = 0x50020100,
	    .name = "AT32F421C8T7",
	    .series = ARTERY_SERIES_F421,
	    .flash_size = 64,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x50020101,
	    .name = "AT32F421K8T7",
	    .series = ARTERY_SERIES_F421,
	    .flash_size = 64,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x50020102,
	    .name = "AT32F421K8U7",
	    .series = ARTERY_SERIES_F421,
	    .flash_size = 64,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x50020103,
	    .name = "AT32F421K8U7-4",
	    .series = ARTERY_SERIES_F421,
	    .flash_size = 64,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x50020104,
	    .name = "AT32F421F8U7",
	    .series = ARTERY_SERIES_F421,
	    .flash_size = 64,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x50020105,
	    .name = "AT32F421F8P7",
	    .series = ARTERY_SERIES_F421,
	    .flash_size = 64,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x50020086,
	    .name = "AT32F421C6T7",
	    .series = ARTERY_SERIES_F421,
	    .flash_size = 32,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x50020087,
	    .name = "AT32F421K6T7",
	    .series = ARTERY_SERIES_F421,
	    .flash_size = 32,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x50020088,
	    .name = "AT32F421K6U7",
	    .series = ARTERY_SERIES_F421,
	    .flash_size = 32,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x50020089,
	    .name = "AT32F421K6U7-4",
	    .series = ARTERY_SERIES_F421,
	    .flash_size = 32,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x5002008a,
	    .name = "AT32F421F6U7",
	    .series = ARTERY_SERIES_F421,
	    .flash_size = 32,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x5002008b,
	    .name = "AT32F421F6P7",
	    .series = ARTERY_SERIES_F421,
	    .flash_size = 32,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x5001000c,
	    .name = "AT32F421C4T7",
	    .series = ARTERY_SERIES_F421,
	    .flash_size = 16,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x5001000d,
	    .name = "AT32F421K4T7",
	    .series = ARTERY_SERIES_F421,
	    .flash_size = 16,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x5001000e,
	    .name = "AT32F421K4U7",
	    .series = ARTERY_SERIES_F421,
	    .flash_size = 16,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x5001000f,
	    .name = "AT32F421K4U7-4",
	    .series = ARTERY_SERIES_F421,
	    .flash_size = 16,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x50010010,
	    .name = "AT32F421F4U7",
	    .series = ARTERY_SERIES_F421,
	    .flash_size = 16,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x50010011,
	    .name = "AT32F421F4P7",
	    .series = ARTERY_SERIES_F421,
	    .flash_size = 16,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x50020112,
	    .name = "AT32F421G8U7",
	    .series = ARTERY_SERIES_F421,
	    .flash_size = 64,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x50020093,
	    .name = "AT32F421G6U7",
	    .series = ARTERY_SERIES_F421,
	    .flash_size = 32,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x50010014,
	    .name = "AT32F421G4U7",
	    .series = ARTERY_SERIES_F421,
	    .flash_size = 16,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x700a3240,
	    .name = "AT32F423VCT7",
	    .series = ARTERY_SERIES_F423,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x700a21c1,
	    .name = "AT32F423VBT7",
	    .series = ARTERY_SERIES_F423,
	    .flash_size = 128,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x70032102,
	    .name = "AT32F423V8T7",
	    .series = ARTERY_SERIES_F423,
	    .flash_size = 64,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x700a3243,
	    .name = "AT32F423RCT7",
	    .series = ARTERY_SERIES_F423,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x700a21c4,
	    .name = "AT32F423RBT7",
	    .series = ARTERY_SERIES_F423,
	    .flash_size = 128,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x70032105,
	    .name = "AT32F423R8T7",
	    .series = ARTERY_SERIES_F423,
	    .flash_size = 64,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x700a3246,
	    .name = "AT32F423RCT7-7",
	    .series = ARTERY_SERIES_F423,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x700a21c7,
	    .name = "AT32F423RBT7-7",
	    .series = ARTERY_SERIES_F423,
	    .flash_size = 128,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x70032108,
	    .name = "AT32F423R8T7-7",
	    .series = ARTERY_SERIES_F423,
	    .flash_size = 64,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x700a3249,
	    .name = "AT32F423CCT7",
	    .series = ARTERY_SERIES_F423,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x700a21ca,
	    .name = "AT32F423CBT7",
	    .series = ARTERY_SERIES_F423,
	    .flash_size = 128,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x7003210b,
	    .name = "AT32F423C8T7",
	    .series = ARTERY_SERIES_F423,
	    .flash_size = 64,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x700a324c,
	    .name = "AT32F423CCU7",
	    .series = ARTERY_SERIES_F423,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x700a21cd,
	    .name = "AT32F423CBU7",
	    .series = ARTERY_SERIES_F423,
	    .flash_size = 128,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x7003210e,
	    .name = "AT32F423C8U7",
	    .series = ARTERY_SERIES_F423,
	    .flash_size = 64,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x700a3250,
	    .name = "AT32F423TCU7",
	    .series = ARTERY_SERIES_F423,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x700a21d1,
	    .name = "AT32F423TBU7",
	    .series = ARTERY_SERIES_F423,
	    .flash_size = 128,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x70032112,
	    .name = "AT32F423T8U7",
	    .series = ARTERY_SERIES_F423,
	    .flash_size = 64,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x700a3253,
	    .name = "AT32F423KCU7-4",
	    .series = ARTERY_SERIES_F423,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x700a21d4,
	    .name = "AT32F423KBU7-4",
	    .series = ARTERY_SERIES_F423,
	    .flash_size = 128,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x70032115,
	    .name = "AT32F423K8U7-4",
	    .series = ARTERY_SERIES_F423,
	    .flash_size = 64,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x50092100,
	    .name = "AT32F425R8T7",
	    .series = ARTERY_SERIES_F425,
	    .flash_size = 64,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x50092081,
	    .name = "AT32F425R6T7",
	    .series = ARTERY_SERIES_F425,
	    .flash_size = 32,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x50092103,
	    .name = "AT32F425R8T7-7",
	    .series = ARTERY_SERIES_F425,
	    .flash_size = 64,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x50092084,
	    .name = "AT32F425R6T7-7",
	    .series = ARTERY_SERIES_F425,
	    .flash_size = 32,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x50092106,
	    .name = "AT32F425C8T7",
	    .series = ARTERY_SERIES_F425,
	    .flash_size = 64,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x50092087,
	    .name = "AT32F425C6T7",
	    .series = ARTERY_SERIES_F425,
	    .flash_size = 32,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x50092109,
	    .name = "AT32F425C8U7",
	    .series = ARTERY_SERIES_F425,
	    .flash_size = 64,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x5009208a,
	    .name = "AT32F425C6U7",
	    .series = ARTERY_SERIES_F425,
	    .flash_size = 32,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x5009210c,
	    .name = "AT32F425K8T7",
	    .series = ARTERY_SERIES_F425,
	    .flash_size = 64,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x5009208d,
	    .name = "AT32F425K6T7",
	    .series = ARTERY_SERIES_F425,
	    .flash_size = 32,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x5009210f,
	    .name = "AT32F425K8U7-4",
	    .series = ARTERY_SERIES_F425,
	    .flash_size = 64,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x50092090,
	    .name = "AT32F425K6U7-4",
	    .series = ARTERY_SERIES_F425,
	    .flash_size = 32,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x50092112,
	    .name = "AT32F425F8P7",
	    .series = ARTERY_SERIES_F425,
	    .flash_size = 64,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x50092093,
	    .name = "AT32F425F6P7",
	    .series = ARTERY_SERIES_F425,
	    .flash_size = 32,
	    .page_size = 1024,
	    .usd_size = 512,
	    .usd_data_size = 250,
	},
	{
	    .pid = 0x70084598,
	    .name = "AT32F435ZDT7",
	    .series = ARTERY_SERIES_F435_F437,
	    .flash_size = 448,
	    .page_size = 4096,
	    .usd_size = 4096,
	    .usd_data_size = 2012,
	},
	{
	    .pid = 0x70083242,
	    .name = "AT32F435ZCT7",
	    .series = ARTERY_SERIES_F435_F437,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 512,
	    .usd_data_size = 220,
	},
	{
	    .pid = 0x70084599,
	    .name = "AT32F435VDT7",
	    .series = ARTERY_SERIES_F435_F437,
	    .flash_size = 448,
	    .page_size = 4096,
	    .usd_size = 4096,
	    .usd_data_size = 2012,
	},
	{
	    .pid = 0x70083245,
	    .name = "AT32F435VCT7",
	    .series = ARTERY_SERIES_F435_F437,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 512,
	    .usd_data_size = 220,
	},
	{
	    .pid = 0x7008459a,
	    .name = "AT32F435RDT7",
	    .series = ARTERY_SERIES_F435_F437,
	    .flash_size = 448,
	    .page_size = 4096,
	    .usd_size = 4096,
	    .usd_data_size = 2012,
	},
	{
	    .pid = 0x70083248,
	    .name = "AT32F435RCT7",
	    .series = ARTERY_SERIES_F435_F437,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 512,
	    .usd_data_size = 220,
	},
	{
	    .pid = 0x7008459b,
	    .name = "AT32F435CDT7",
	    .series = ARTERY_SERIES_F435_F437,
	    .flash_size = 448,
	    .page_size = 4096,
	    .usd_size = 4096,
	    .usd_data_size = 2012,
	},
	{
	    .pid = 0x7008324b,
	    .name = "AT32F435CCT7",
	    .series = ARTERY_SERIES_F435_F437,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 512,
	    .usd_data_size = 220,
	},
	{
	    .pid = 0x7008459c,
	    .name = "AT32F435CDU7",
	    .series = ARTERY_SERIES_F435_F437,
	    .flash_size = 448,
	    .page_size = 4096,
	    .usd_size = 4096,
	    .usd_data_size = 2012,
	},
	{
	    .pid = 0x7008324e,
	    .name = "AT32F435CCU7",
	    .series = ARTERY_SERIES_F435_F437,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 512,
	    .usd_data_size = 220,
	},
	{
	    .pid = 0x7008459d,
	    .name = "AT32F437ZDT7",
	    .series = ARTERY_SERIES_F435_F437,
	    .flash_size = 448,
	    .page_size = 4096,
	    .usd_size = 4096,
	    .usd_data_size = 2012,
	},
	{
	    .pid = 0x70083251,
	    .name = "AT32F437ZCT7",
	    .series = ARTERY_SERIES_F435_F437,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 512,
	    .usd_data_size = 220,
	},
	{
	    .pid = 0x7008459e,
	    .name = "AT32F437VDT7",
	    .series = ARTERY_SERIES_F435_F437,
	    .flash_size = 448,
	    .page_size = 4096,
	    .usd_size = 4096,
	    .usd_data_size = 2012,
	},
	{
	    .pid = 0x70083254,
	    .name = "AT32F437VCT7",
	    .series = ARTERY_SERIES_F435_F437,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 512,
	    .usd_data_size = 220,
	},
	{
	    .pid = 0x7008459f,
	    .name = "AT32F437RDT7",
	    .series = ARTERY_SERIES_F435_F437,
	    .flash_size = 448,
	    .page_size = 4096,
	    .usd_size = 4096,
	    .usd_data_size = 2012,
	},
	{
	    .pid = 0x70083257,
	    .name = "AT32F437RCT7",
	    .series = ARTERY_SERIES_F435_F437,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 512,
	    .usd_data_size = 220,
	},
	{
	    .pid = 0x70030250,
	    .name = "AT32WB415CCU7-7",
	    .series = ARTERY_SERIES_WB415,
	    .flash_size = 256,
	    .page_size = 2048,
	    .usd_size = 1024,
	    .usd_data_size = 506,
	},
};

/* flash bank artery <base> <size> 0 0 <target#> */
FLASH_BANK_COMMAND_HANDLER(artery_flash_bank_command)
{
	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct artery_flash_bank *artery_info = calloc(1,
		sizeof(struct artery_flash_bank));

	if (!artery_info)
		return ERROR_FAIL;

	bank->driver_priv = artery_info;
	artery_info->probed = false;

	return ERROR_OK;
}

static int artery_read_flash_register(struct flash_bank *bank,
		enum artery_flash_reg_index reg, uint32_t *value)
{
	const struct artery_flash_bank *artery_info = bank->driver_priv;
	const struct artery_part_info *part_info = artery_info->part_info;
	const struct artery_series_info *series_info = &artery_series[part_info->series];
	uint32_t reg_addr = series_info->flash_regs_base + series_info->flash_regs[reg];

	return target_read_u32(bank->target, reg_addr, value);
}

static int artery_write_flash_register(struct flash_bank *bank,
		enum artery_flash_reg_index reg, uint32_t value)
{
	const struct artery_flash_bank *artery_info = bank->driver_priv;
	const struct artery_part_info *part_info = artery_info->part_info;
	const struct artery_series_info *series_info = &artery_series[part_info->series];
	uint32_t reg_addr = series_info->flash_regs_base + series_info->flash_regs[reg];

	return target_write_u32(bank->target, reg_addr, value);
}

static int artery_wait_flash_busy(struct flash_bank *bank, unsigned int timeout)
{
	const int64_t start_time = timeval_ms();

	while (true) {
		uint32_t status;
		int retval = artery_read_flash_register(bank, ARTERY_FLASH_REG_STS,
			&status);

		if (retval != ERROR_OK)
			return retval;

		if (!(status & FLASH_STS_OBF))
			break;

		if ((timeval_ms() - start_time) > timeout) {
			LOG_ERROR("Timed out waiting for flash");
			return ERROR_FAIL;
		}

		keep_alive();
	}

	return ERROR_OK;
}

static int artery_enable_hiclk(struct flash_bank *bank)
{
	const struct artery_flash_bank *artery_info = bank->driver_priv;
	const struct artery_part_info *part_info = artery_info->part_info;
	const struct artery_series_info *series_info = &artery_series[part_info->series];
	uint32_t crm_base = series_info->crm_base;
	struct target *target = bank->target;

	uint32_t crm_ctrl;
	int ret = target_read_u32(target, crm_base + CRM_REG_CTRL, &crm_ctrl);

	if (ret != ERROR_OK)
		return ret;

	// High speed internal clock (HICK) is already enabled and ready.
	if (crm_ctrl & CRM_CTRL_HICKSTBL)
		return ERROR_OK;

	crm_ctrl |= CRM_CTRL_HICKEN;
	ret = target_write_u32(target, crm_base + CRM_REG_CTRL, crm_ctrl);

	if (ret != ERROR_OK)
		return ret;

	const int64_t start_time = timeval_ms();

	while (true) {
		ret = target_read_u32(target, crm_base + CRM_REG_CTRL, &crm_ctrl);

		if (ret != ERROR_OK)
			return ret;

		if (crm_ctrl & CRM_CTRL_HICKSTBL)
			break;

		if ((timeval_ms() - start_time) > HICK_STABLE_TIMEOUT) {
			LOG_ERROR("Timed out waiting for flash");
			return ERROR_FAIL;
		}

		keep_alive();
	}

	return ERROR_OK;
}

static int artery_usd_unlock(struct flash_bank *bank)
{
	uint32_t ctrl;
	int retval = artery_read_flash_register(bank, ARTERY_FLASH_REG_CTRL, &ctrl);

	if (retval != ERROR_OK)
		return retval;

	if (ctrl & FLASH_CTRL_USDULKS)
		return ERROR_OK;

	retval = artery_write_flash_register(bank, ARTERY_FLASH_REG_USD_UNLOCK, KEY1);

	if (retval != ERROR_OK)
		return retval;

	retval = artery_write_flash_register(bank, ARTERY_FLASH_REG_USD_UNLOCK, KEY2);

	if (retval != ERROR_OK)
		return retval;

	retval = artery_read_flash_register(bank, ARTERY_FLASH_REG_CTRL, &ctrl);

	if (retval != ERROR_OK)
		return retval;

	if (!(ctrl & FLASH_CTRL_USDULKS)) {
		LOG_ERROR("Failed to unlock user system data");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int artery_usd_lock(struct flash_bank *bank)
{
	uint32_t ctrl;

	int retval = artery_read_flash_register(bank, ARTERY_FLASH_REG_CTRL, &ctrl);

	if (retval != ERROR_OK)
		return retval;

	if (!(ctrl & FLASH_CTRL_USDULKS))
		return ERROR_OK;

	ctrl &= ~FLASH_CTRL_USDULKS;
	retval = artery_write_flash_register(bank, ARTERY_FLASH_REG_CTRL, ctrl);

	if (retval != ERROR_OK)
		return retval;

	retval = artery_read_flash_register(bank, ARTERY_FLASH_REG_CTRL, &ctrl);

	if (retval != ERROR_OK)
		return retval;

	if (ctrl & FLASH_CTRL_USDULKS) {
		LOG_ERROR("Failed to lock user system data");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

// Initialize the device for flash memory operations.
static int artery_init_flash(struct flash_bank *bank)
{
	/*
	 * The internal high speed clock (HICK) must be enabled before any flash
	 * operation is performed.
	 */
	int retval = artery_enable_hiclk(bank);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to enable HICLK");
		return retval;
	}

	uint32_t ctrl;
	retval = artery_read_flash_register(bank, ARTERY_FLASH_REG_CTRL, &ctrl);

	if (retval != ERROR_OK)
		return retval;

	if (!(ctrl & FLASH_CTRL_OPLK))
		return ERROR_OK;

	retval = artery_write_flash_register(bank, ARTERY_FLASH_REG_UNLOCK, KEY1);

	if (retval != ERROR_OK)
		return retval;

	retval = artery_write_flash_register(bank, ARTERY_FLASH_REG_UNLOCK, KEY2);

	if (retval != ERROR_OK)
		return retval;

	retval = artery_read_flash_register(bank, ARTERY_FLASH_REG_CTRL, &ctrl);

	if (retval != ERROR_OK)
		return retval;

	if (ctrl & FLASH_CTRL_OPLK) {
		LOG_ERROR("Failed to initialize flash memory");
		return ERROR_FAIL;
	}

	return artery_usd_unlock(bank);
}

// Deinitialize the flash memory controller.
static int artery_deinit_flash(struct flash_bank *bank)
{
	int retval = artery_usd_lock(bank);

	if (retval != ERROR_OK)
		return retval;

	uint32_t ctrl;
	retval = artery_read_flash_register(bank, ARTERY_FLASH_REG_CTRL, &ctrl);

	if (retval != ERROR_OK)
		return retval;

	if (ctrl & FLASH_CTRL_OPLK)
		return ERROR_OK;

	ctrl |= FLASH_CTRL_OPLK;
	retval = artery_write_flash_register(bank, ARTERY_FLASH_REG_CTRL, ctrl);

	if (retval != ERROR_OK)
		return retval;

	retval = artery_read_flash_register(bank, ARTERY_FLASH_REG_CTRL, &ctrl);

	if (retval != ERROR_OK)
		return retval;

	if (!(ctrl & FLASH_CTRL_OPLK)) {
		LOG_ERROR("Failed to lock flash");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int artery_read_protection(struct flash_bank *bank, uint64_t *protection)
{
	uint32_t epps0;
	int retval = artery_read_flash_register(bank, ARTERY_FLASH_REG_EPPS0,
		&epps0);

	if (retval != ERROR_OK)
		return retval;

	const struct artery_flash_bank *artery_info = bank->driver_priv;
	const struct artery_part_info *part_info = artery_info->part_info;

	uint64_t prot = epps0;

	if (artery_series[part_info->series].has_epp_ext) {
		uint32_t epps1;
		retval = artery_read_flash_register(bank, ARTERY_FLASH_REG_EPPS1,
			&epps1);

		if (retval != ERROR_OK)
			return retval;

		prot |= (((uint64_t)epps1) << 32);
	}

	*protection = prot;

	return ERROR_OK;
}

static int artery_protect_check(struct flash_bank *bank)
{
	uint64_t prot;
	int retval = artery_read_protection(bank, &prot);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read flash protection settings");
		return retval;
	}

	for (unsigned int i = 0; i < bank->num_prot_blocks; i++) {
		const bool protected = !(prot & (UINT64_C(1) << i));
		bank->prot_blocks[i].is_protected = protected ? 1 : 0;
	}

	return ERROR_OK;
}

static int artery_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval = artery_init_flash(bank);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to initialize flash controller");
		return retval;
	}

	// Clear the EPPERR bit, otherwise we may read an invalid value later on.
	retval = artery_write_flash_register(bank, ARTERY_FLASH_REG_STS,
		FLASH_STS_EPPERR);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write FLASH_STS register");
		goto flash_deinit;
	}

	retval = artery_wait_flash_busy(bank, FLASH_WRITE_TIMEOUT);

	if (retval != ERROR_OK)
		goto flash_deinit;

	for (unsigned int i = first; i <= last; i++) {
		retval = artery_write_flash_register(bank, ARTERY_FLASH_REG_ADDR,
			bank->base + bank->sectors[i].offset);

		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to write FLASH_ADDR register");
			goto flash_deinit;
		}

		retval = artery_write_flash_register(bank, ARTERY_FLASH_REG_CTRL,
			FLASH_CTRL_SECERS | FLASH_CTRL_ERSTR);

		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to write FLASH_CTRL register");
			goto flash_deinit;
		}

		retval = artery_wait_flash_busy(bank, FLASH_ERASE_TIMEOUT);

		if (retval != ERROR_OK)
			goto flash_deinit;

		uint32_t sts;
		retval = artery_read_flash_register(bank, ARTERY_FLASH_REG_STS, &sts);

		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to read FLASH_STS register");
			goto flash_deinit;
		}

		if (sts & FLASH_STS_EPPERR) {
			LOG_ERROR("Sector %u is write protected", i);
			retval = ERROR_FLASH_PROTECTED;
			goto flash_deinit;
		}
	}

	int retval_deinit;
flash_deinit:
	retval_deinit = artery_deinit_flash(bank);

	if (retval_deinit != ERROR_OK)
		return retval_deinit;

	return retval;
}

static int artery_usd_init(struct flash_bank *bank, uint8_t **buffer)
{
	const struct artery_flash_bank *artery_info = bank->driver_priv;
	const struct artery_part_info *part_info = artery_info->part_info;
	uint32_t usd_size = part_info->usd_size;

	*buffer = malloc(usd_size);

	if (!*buffer) {
		LOG_ERROR("Failed to allocate USD buffer");
		return ERROR_FAIL;
	}

	memset(*buffer, 0xff, usd_size);

	return ERROR_OK;
}

static int artery_usd_read(struct flash_bank *bank, uint8_t *buffer)
{
	const struct artery_flash_bank *artery_info = bank->driver_priv;
	const struct artery_part_info *part_info = artery_info->part_info;
	const struct artery_series_info *series_info = &artery_series[part_info->series];

	return target_read_buffer(bank->target, series_info->usd_base,
		part_info->usd_size, buffer);
}

static uint8_t artery_usd_read_buffer(const uint8_t *buffer, uint32_t base,
		uint32_t offset)
{
	return buffer[base + (offset * 2)];
}

static int artery_usd_load(const struct artery_part_info *part_info,
		const uint8_t *buffer, struct artery_usd *usd)
{
	const uint32_t *usd_regs = artery_series[part_info->series].usd_offsets;

	uint8_t fap_level = artery_usd_read_buffer(buffer,
		usd_regs[ARTERY_USD_FAP_INDEX], 0);

	switch (fap_level) {
	case ARTERY_FAP_LEVEL_DISABLED:
	case ARTERY_FAP_LEVEL_HIGH:
		usd->fap_level = fap_level;
		break;
	default:
		usd->fap_level = ARTERY_FAP_LEVEL_LOW;
	}

	usd->ssb = artery_usd_read_buffer(buffer, usd_regs[ARTERY_USD_SSB_INDEX], 0);
	usd->protection = 0;

	for (unsigned int i = 0; i < 4; i++) {
		const uint8_t prot = artery_usd_read_buffer(buffer,
			usd_regs[ARTERY_USD_EPP_INDEX], i);
		usd->protection |= (prot << (i * 8));
	}

	if (artery_series[part_info->series].has_epp_ext) {
		usd->protection_ext = 0;

		for (unsigned int i = 0; i < 4; i++) {
			const uint8_t prot = artery_usd_read_buffer(buffer,
				usd_regs[ARTERY_USD_EPP_INDEX], i);
			usd->protection_ext |= (prot << (i * 8));
		}
	}

	// All devices have at least two bytes of user data.
	usd->data[0] = artery_usd_read_buffer(buffer,
		usd_regs[ARTERY_USD_DATA_INDEX], 0);
	usd->data[1] = artery_usd_read_buffer(buffer,
		usd_regs[ARTERY_USD_DATA_INDEX], 1);

	for (unsigned int i = 0; i < part_info->usd_data_size - 2; i++) {
		usd->data[i + 2] = artery_usd_read_buffer(buffer,
			usd_regs[ARTERY_USD_DATA_EXT_INDEX], i);
	}

	return ERROR_OK;
}

static void artery_usd_write_buffer(uint8_t *buffer, uint32_t base,
		uint32_t offset, uint8_t data)
{
	buffer[base + (offset * 2)] = data;
	buffer[base + (offset * 2) + 1] = ~data;
}

static void artery_usd_update(const struct artery_part_info *part_info,
		uint8_t *buffer, const struct artery_usd *usd)
{
	const uint32_t *usd_regs = artery_series[part_info->series].usd_offsets;

	artery_usd_write_buffer(buffer, usd_regs[ARTERY_USD_FAP_INDEX], 0,
		usd->fap_level);
	artery_usd_write_buffer(buffer, usd_regs[ARTERY_USD_SSB_INDEX], 0,
		usd->ssb);

	for (unsigned int i = 0; i < 4; i++) {
		const uint8_t prot = usd->protection >> (i * 8);
		artery_usd_write_buffer(buffer, usd_regs[ARTERY_USD_EPP_INDEX], i, prot);
	}

	if (artery_series[part_info->series].has_epp_ext) {
		for (unsigned int i = 0; i < 4; i++) {
			const uint8_t prot = usd->protection_ext >> (i * 8);
			artery_usd_write_buffer(buffer, usd_regs[ARTERY_USD_EPP_EXT_INDEX],
				i, prot);
		}
	}

	artery_usd_write_buffer(buffer, usd_regs[ARTERY_USD_DATA_INDEX], 0,
		usd->data[0]);
	artery_usd_write_buffer(buffer, usd_regs[ARTERY_USD_DATA_INDEX], 1,
		usd->data[1]);

	for (unsigned int i = 0; i < part_info->usd_data_size - 2; i++) {
		artery_usd_write_buffer(buffer, usd_regs[ARTERY_USD_DATA_EXT_INDEX], i,
			usd->data[i + 2]);
	}
}

static int artery_usd_write(struct flash_bank *bank, const uint8_t *buffer)
{
	struct target *target = bank->target;

	int retval = artery_wait_flash_busy(bank, FLASH_WRITE_TIMEOUT);

	if (retval != ERROR_OK)
		return retval;

	// Clear the PRGMERR bit, otherwise we may read an invalid value later on.
	retval = artery_write_flash_register(bank, ARTERY_FLASH_REG_STS,
		FLASH_STS_PRGMERR);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write FLASH_STS register");
		return retval;
	}

	// Set the USDULKS bit to avoid locking the USD area.
	uint32_t ctrl = FLASH_CTRL_USDULKS | FLASH_CTRL_USDPRGM;
	retval = artery_write_flash_register(bank, ARTERY_FLASH_REG_CTRL, ctrl);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write FLASH_CTRL register");
		return retval;
	}

	const struct artery_flash_bank *artery_info = bank->driver_priv;
	const struct artery_part_info *part_info = artery_info->part_info;

	const target_addr_t usd_base = artery_series[part_info->series].usd_base;
	const uint32_t usd_size = part_info->usd_size;

	unsigned int bytes_written = 0;

	retval = artery_wait_flash_busy(bank, FLASH_WRITE_TIMEOUT);

	if (retval != ERROR_OK)
		return retval;

	while (bytes_written < usd_size) {
		uint32_t tmp;
		memcpy(&tmp, buffer + bytes_written, sizeof(tmp));

		if (tmp != 0xffffffff) {
			retval = target_write_u32(target, usd_base + bytes_written, tmp);

			if (retval != ERROR_OK) {
				LOG_ERROR("Failed to write user system data");
				return retval;
			}

			retval = artery_wait_flash_busy(bank, FLASH_WRITE_TIMEOUT);

			if (retval != ERROR_OK)
				return retval;
		}

		bytes_written += 4;
	}

	uint32_t sts;
	retval = artery_read_flash_register(bank, ARTERY_FLASH_REG_STS, &sts);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read FLASH_STS register");
		return retval;
	}

	if (sts & FLASH_STS_PRGMERR) {
		LOG_ERROR("Failed to program user system data");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static int artery_usd_erase(struct flash_bank *bank)
{
	int retval = artery_wait_flash_busy(bank, FLASH_WRITE_TIMEOUT);

	if (retval != ERROR_OK)
		return retval;

	// Set the USDULKS bit to avoid locking the USD area.
	uint32_t ctrl = FLASH_CTRL_USDULKS | FLASH_CTRL_USDERS | FLASH_CTRL_ERSTR;
	retval = artery_write_flash_register(bank, ARTERY_FLASH_REG_CTRL, ctrl);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write FLASH_CTRL register");
		return retval;
	}

	retval = artery_wait_flash_busy(bank, FLASH_WRITE_TIMEOUT);

	if (retval != ERROR_OK)
		return retval;

	uint32_t sts;
	retval = artery_read_flash_register(bank, ARTERY_FLASH_REG_STS, &sts);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read FLASH_STS register");
		return retval;
	}

	retval = artery_read_flash_register(bank, ARTERY_FLASH_REG_CTRL, &ctrl);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read FLASH_CTRL register");
		return retval;
	}

	return ERROR_OK;
}

static int artery_get_fap(struct flash_bank *bank,
		enum artery_fap_level *fap_level)
{
	uint32_t usd;
	int retval = artery_read_flash_register(bank, ARTERY_FLASH_REG_USD,
		&usd);

	if (retval != ERROR_OK)
		return retval;

	const struct artery_flash_bank *artery_info = bank->driver_priv;
	const struct artery_part_info *part_info = artery_info->part_info;
	const struct artery_series_info *series_info = &artery_series[part_info->series];

	const bool fap_high = series_info->has_fap_high_level && (usd & FLASH_USD_FAP_HL);
	const bool fap_low = usd & FLASH_USD_FAP;

	if (fap_high && fap_low)
		*fap_level = ARTERY_FAP_LEVEL_HIGH;
	else if (fap_low)
		*fap_level = ARTERY_FAP_LEVEL_LOW;
	else
		*fap_level = ARTERY_FAP_LEVEL_DISABLED;

	return ERROR_OK;
}

static int artery_protect(struct flash_bank *bank, int set, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	enum artery_fap_level fap_level;
	int retval = artery_get_fap(bank, &fap_level);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read FAP level");
		return retval;
	}

	if (fap_level != ARTERY_FAP_LEVEL_DISABLED) {
		LOG_ERROR("Protection cannot be modified when FAP is active");
		return ERROR_FAIL;
	}

	uint8_t *usd_buffer;
	retval = artery_usd_init(bank, &usd_buffer);

	if (retval != ERROR_OK)
		return retval;

	retval = artery_usd_read(bank, usd_buffer);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read user system data");
		return retval;
	}

	const struct artery_flash_bank *artery_info = bank->driver_priv;
	const struct artery_part_info *part_info = artery_info->part_info;

	struct artery_usd usd;
	retval = artery_usd_load(part_info, usd_buffer, &usd);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to load user system data");
		free(usd_buffer);
		return retval;
	}

	for (unsigned int i = first; i <= MIN(31, last); i++) {
		if (bank->prot_blocks[i].is_protected == set)
			continue;

		if (set)
			usd.protection &= ~BIT(i);
		else
			usd.protection |= BIT(i);
	}

	for (unsigned int i = 32; i <= last; i++) {
		if (bank->prot_blocks[i].is_protected == set)
			continue;

		if (set)
			usd.protection_ext &= ~BIT(i - 32);
		else
			usd.protection_ext |= BIT(i - 32);
	}

	artery_usd_update(part_info, usd_buffer, &usd);
	retval = artery_init_flash(bank);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to initialize flash controller");
		free(usd_buffer);
		return retval;
	}

	retval = artery_usd_erase(bank);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to erase user system data");
		free(usd_buffer);
		goto flash_deinit;
	}

	retval = artery_usd_write(bank, usd_buffer);

	free(usd_buffer);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write user system data");
		goto flash_deinit;
	}

	int retval_deinit;
flash_deinit:
	retval_deinit = artery_deinit_flash(bank);

	if (retval_deinit != ERROR_OK)
		return retval_deinit;

	return retval;
}

static int artery_write_without_loader(struct flash_bank *bank,
	const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;

	int retval = artery_wait_flash_busy(bank, FLASH_WRITE_TIMEOUT);

	if (retval != ERROR_OK)
		return retval;

	retval = artery_write_flash_register(bank, ARTERY_FLASH_REG_CTRL,
		FLASH_CTRL_FPRGM);

	if (retval != ERROR_OK) {
		LOG_ERROR("failed to write ctrl register");
		return retval;
	}

	const uint32_t block_size = 4;

	uint32_t bytes_written = 0;
	target_addr_t address = bank->base + offset;

	for (uint32_t i = 0; i < count / block_size; i++) {
		retval = target_write_memory(target, address, block_size, 1,
			buffer + bytes_written);

		if (retval != ERROR_OK)
			return retval;

		retval = artery_wait_flash_busy(bank, FLASH_WRITE_TIMEOUT);

		if (retval != ERROR_OK)
			return retval;

		address += block_size;
		bytes_written += block_size;
	}

	return ERROR_OK;
}

static int artery_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval = artery_init_flash(bank);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to initialize flash controller");
		return retval;
	}

	retval = artery_write_without_loader(bank, buffer, offset, count);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write flash memory");
		goto flash_deinit;
	}

	int retval_deinit;
flash_deinit:
	retval_deinit = artery_deinit_flash(bank);

	if (retval != ERROR_OK)
		return retval;

	return retval_deinit;
}

static int artery_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;

	if (!target_was_examined(target)) {
		LOG_ERROR("Target not examined yet");
		return ERROR_TARGET_NOT_EXAMINED;
	}

	const struct cortex_m_common *cortex_m = target_to_cortex_m_safe(target);

	if (!cortex_m) {
		LOG_ERROR("Target is not a Cortex-M device");
		return ERROR_TARGET_INVALID;
	}

	struct artery_flash_bank *artery_info = bank->driver_priv;

	artery_info->probed = false;

	int retval = target_read_u32(target, DEBUG_IDCODE, &artery_info->idcode);

	if (retval != ERROR_OK)
		return retval;

	const uint32_t pid = artery_info->idcode;
	const bool has_fpu = cortex_m->armv7m.fp_feature != FP_NONE;
	bool check_device_series = false;
	enum artery_series device_series;

	/*
	 * The following PIDs are used for AT32F413 and AT32F415 devices. In order
	 * to distinguish between the series, we use the presence of the FPU. Note
	 * that we do not rely on the unqiue device ID (UID) which also encodes the
	 * device series. The reason is that the UID registers are not accessible
	 * when the flash access protection (FAP) is active.
	 */
	switch (pid) {
	case 0x700301C5:
	case 0x70030240:
	case 0x70030242:
		check_device_series = true;
		device_series = has_fpu ? ARTERY_SERIES_F413 : ARTERY_SERIES_F415;
		break;
	default:
		break;
	}

	artery_info->part_info = NULL;

	for (size_t i = 0; i < ARRAY_SIZE(artery_parts); i++) {
		if (check_device_series && artery_parts[i].series != device_series)
			continue;

		if (artery_parts[i].pid == pid) {
			artery_info->part_info = &artery_parts[i];
			break;
		}
	}

	if (!artery_info->part_info) {
		LOG_ERROR("Cannot identify target as an Artery device");
		return ERROR_FAIL;
	}

	const struct artery_part_info *part_info = artery_info->part_info;

	LOG_INFO("Device ID = 0x%08" PRIx32 " (%s)", artery_info->idcode,
		part_info->name);
	LOG_INFO("Flash size = %d KiB", part_info->flash_size);

	free(bank->sectors);

	bank->base = FLASH_BASE;
	bank->size = part_info->flash_size * 1024;

	const unsigned int num_pages = (bank->size) / part_info->page_size;

	// Ensure that the flash infrastructure uses an alignment of 4 bytes.
	bank->write_start_alignment = 4;
	bank->write_end_alignment = 4;

	bank->num_sectors = num_pages;
	bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);

	if (!bank->sectors) {
		LOG_ERROR("Failed to allocate bank sectors");
		return ERROR_FAIL;
	}

	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].offset = i * part_info->page_size;
		bank->sectors[i].size = part_info->page_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
	}

	free(bank->prot_blocks);

	/*
	 * Flash erase/program protection (EPPx) registers configuration for each
	 * device series.
	 *
	 * - AT32F403A / AT32F407
	 * - AT32F413
	 *
	 *    Each bit represents a sector of 4 KiB. The last bit represents the
	 *    entire remaining flash memory and extension area.
	 *
	 * - AT32F415
	 * - AT32WB415
	 *
	 *    Each bit represents a sector of 2 KiB. The last bit represents the
	 *    entire remaining flash memory and extension area.
	 *
	 * - AT32F421
	 * - AT32F423
	 * - AT32F425
	 *
	 *    Each bit represents a sector of 4 KiB. Some bits may not used
	 *    depending on the flash memory size. The last bit represents only the
	 *    flash memory extension area.
	 *
	 * - AT32F435 / AT32F437
	 *
	 *    This device series has an additional erase/program protection (EPP)
	 *    register.
	 *
	 *    The first 32 bits represent a flash sector of 4 KiB per bit.
	 *
	 *    The additional 32 bits represent a sector of 128 KiB each. The
	 *    second last bit covers the remaining flash memory. The last bit is
	 *    always reserved.
	 *
	 */
	if (part_info->series == ARTERY_SERIES_F435_F437) {
		// See description above.
		const unsigned int num_prot_blocks_1 = 32;
		const unsigned int num_prot_blocks_2 = MIN(31, DIV_ROUND_UP(part_info->flash_size - 128, 128));
		const unsigned int num_prot_blocks = num_prot_blocks_1 + num_prot_blocks_2;
		bank->num_prot_blocks = num_prot_blocks;
		bank->prot_blocks = malloc(sizeof(struct flash_sector) * num_prot_blocks);

		if (!bank->prot_blocks) {
			LOG_ERROR("Failed to allocate protection blocks");
			return ERROR_FAIL;
		}

		const uint32_t prot_block_size = 4096;

		unsigned int i;
		uint32_t block_offset = 0;

		for (i = 0; i < 32; i++) {
			bank->prot_blocks[i].offset = block_offset;
			bank->prot_blocks[i].size = prot_block_size;
			bank->prot_blocks[i].is_erased = -1;
			bank->prot_blocks[i].is_protected = -1;

			block_offset += prot_block_size;
		}

		const uint32_t prot_block_size_2 = 128 * 1024;

		for (; i < (num_prot_blocks - 1); i++) {
			bank->prot_blocks[i].offset = block_offset;
			bank->prot_blocks[i].size = prot_block_size_2;
			bank->prot_blocks[i].is_erased = -1;
			bank->prot_blocks[i].is_protected = -1;

			block_offset += prot_block_size_2;
		}

		bank->prot_blocks[i].offset = block_offset;
		bank->prot_blocks[i].size = bank->size - block_offset;
		bank->prot_blocks[i].is_erased = -1;
		bank->prot_blocks[i].is_protected = -1;
	} else {
		uint32_t prot_block_size;

		switch (part_info->series) {
		case ARTERY_SERIES_F403A_F407:
		case ARTERY_SERIES_F413:
		case ARTERY_SERIES_F421:
		case ARTERY_SERIES_F423:
		case ARTERY_SERIES_F425:
			prot_block_size = 4096;
			break;
		case ARTERY_SERIES_F415:
		case ARTERY_SERIES_WB415:
			prot_block_size = 2048;
			break;
		default:
			LOG_ERROR("Unknown Artery device series");
			return ERROR_FAIL;
		}

		const unsigned int num_prot_blocks = MIN(bank->size / prot_block_size, 32);
		bank->num_prot_blocks = num_prot_blocks;
		bank->prot_blocks = malloc(sizeof(struct flash_sector) * num_prot_blocks);

		if (!bank->prot_blocks) {
			LOG_ERROR("Failed to allocate protection blocks");
			return ERROR_FAIL;
		}

		unsigned int i;

		for (i = 0; i < (num_prot_blocks - 1); i++) {
			bank->prot_blocks[i].offset = i * prot_block_size;
			bank->prot_blocks[i].size = prot_block_size;
			bank->prot_blocks[i].is_erased = -1;
			bank->prot_blocks[i].is_protected = -1;
		}

		bank->prot_blocks[i].offset = i * prot_block_size;
		bank->prot_blocks[i].size = (bank->size - (i * prot_block_size));
		bank->prot_blocks[i].is_erased = -1;
		bank->prot_blocks[i].is_protected = -1;
	}

	artery_info->probed = true;

	return ERROR_OK;
}

static int artery_auto_probe(struct flash_bank *bank)
{
	const struct artery_flash_bank *artery_info = bank->driver_priv;

	if (artery_info->probed)
		return ERROR_OK;

	return artery_probe(bank);
}

static int artery_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	const struct artery_flash_bank *artery_info = bank->driver_priv;
	const struct artery_part_info *part_info = artery_info->part_info;

	if (!part_info) {
		command_print_sameline(cmd, "Cannot identify target device");
		return ERROR_OK;
	}

	command_print_sameline(cmd, "%s - %u KiB flash", part_info->name,
		part_info->flash_size);

	return ERROR_OK;
}

static int artery_mass_erase(struct flash_bank *bank)
{
	int retval = artery_init_flash(bank);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to initialize flash controller");
		return retval;
	}

	retval = artery_wait_flash_busy(bank, FLASH_WRITE_TIMEOUT);

	if (retval != ERROR_OK)
		goto flash_deinit;

	// Clear the EPPERR bit, otherwise we may read an invalid value later on.
	retval = artery_write_flash_register(bank, ARTERY_FLASH_REG_STS,
		FLASH_STS_EPPERR);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write FLASH_CTRL register");
		goto flash_deinit;
	}

	uint32_t ctrl = FLASH_CTRL_BANKERS | FLASH_CTRL_ERSTR;
	retval = artery_write_flash_register(bank, ARTERY_FLASH_REG_CTRL, ctrl);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write FLASH_CTRL register");
		goto flash_deinit;
	}

	retval = artery_wait_flash_busy(bank, FLASH_MASS_ERASE_TIMEOUT);

	if (retval != ERROR_OK)
		goto flash_deinit;

	uint32_t sts;
	retval = artery_read_flash_register(bank, ARTERY_FLASH_REG_STS, &sts);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read FLASH_STS register");
		goto flash_deinit;
	}

	if (sts & FLASH_STS_EPPERR) {
		LOG_ERROR("Mass erase operation failed");
		goto flash_deinit;
	}

	int retval_deinit;
flash_deinit:
	retval_deinit = artery_deinit_flash(bank);

	if (retval != ERROR_OK)
		return retval;

	return retval_deinit;
}

COMMAND_HANDLER(artery_handle_fap_enable_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);

	if (retval != ERROR_OK)
		return retval;

	enum artery_fap_level fap_level;
	retval = artery_get_fap(bank, &fap_level);

	if (retval != ERROR_OK) {
		command_print(CMD, "failed to read FAP state");
		return retval;
	}

	if (fap_level != ARTERY_FAP_LEVEL_DISABLED) {
		command_print(CMD, "flash access protection is already enabled");
		return ERROR_FAIL;
	}

	uint8_t *usd_buffer;
	retval = artery_usd_init(bank, &usd_buffer);

	if (retval != ERROR_OK)
		return retval;

	retval = artery_usd_read(bank, usd_buffer);

	if (retval != ERROR_OK) {
		command_print(CMD, "failed to read user system data");
		return retval;
	}

	const struct artery_flash_bank *artery_info = bank->driver_priv;
	const struct artery_part_info *part_info = artery_info->part_info;

	struct artery_usd usd;
	retval = artery_usd_load(part_info, usd_buffer, &usd);

	if (retval != ERROR_OK) {
		command_print(CMD, "failed to load user system data");
		return retval;
	}

	usd.fap_level = ARTERY_FAP_LEVEL_LOW;
	artery_usd_update(part_info, usd_buffer, &usd);

	retval = artery_init_flash(bank);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to initialize flash controller");
		goto flash_deinit;
	}

	retval = artery_usd_erase(bank);

	if (retval != ERROR_OK) {
		free(usd_buffer);
		command_print(CMD, "failed to erase user system data");
		goto flash_deinit;
	}

	retval = artery_usd_write(bank, usd_buffer);
	free(usd_buffer);

	if (retval != ERROR_OK) {
		command_print(CMD, "failed to write user system data");
		goto flash_deinit;
	}

	int retval_deinit;
flash_deinit:
	retval_deinit = artery_deinit_flash(bank);

	if (retval_deinit != ERROR_OK)
		return retval_deinit;

	return retval;
}

/*
 * We use a dedicated operation to perform the FAP unlock operation that only
 * writes the corresponding FAP byte(s) instead of the entire user system
 * data (USD) area. The reason is that directly after the FAP byte(s) are
 * written, a device reset is performed and all other writes to the USD area
 * would fail and generate errors.
*/
static int artery_disable_fap(struct flash_bank *bank)
{
	int retval = artery_init_flash(bank);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to initialize flash controller");
		return retval;
	}

	retval = artery_usd_erase(bank);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to erase user system data");
		goto flash_deinit;
	}

	retval = artery_wait_flash_busy(bank, FLASH_WRITE_TIMEOUT);

	if (retval != ERROR_OK)
		goto flash_deinit;

	// Clear the PRGMERR bit, otherwise we may read an invalid value later on.
	retval = artery_write_flash_register(bank, ARTERY_FLASH_REG_STS,
		FLASH_STS_PRGMERR);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write FLASH_STS register");
		goto flash_deinit;
	}

	// Set the USDULKS bit to avoid locking the USD area.
	uint32_t ctrl = FLASH_CTRL_USDULKS | FLASH_CTRL_USDPRGM;
	retval = artery_write_flash_register(bank, ARTERY_FLASH_REG_CTRL, ctrl);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write FLASH_CTRL register");
		goto flash_deinit;
	}

	retval = artery_wait_flash_busy(bank, FLASH_WRITE_TIMEOUT);

	if (retval != ERROR_OK)
		goto flash_deinit;

	uint16_t buffer;

	const struct artery_flash_bank *artery_info = bank->driver_priv;
	const struct artery_part_info *part_info = artery_info->part_info;
	const uint32_t *usd_regs = artery_series[part_info->series].usd_offsets;

	artery_usd_write_buffer((uint8_t *)&buffer, usd_regs[ARTERY_USD_FAP_INDEX], 0,
		ARTERY_FAP_LEVEL_DISABLED);

	const target_addr_t usd_base = artery_series[part_info->series].usd_base;
	retval = target_write_u16(bank->target, usd_base, buffer);

	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write FAP level");
		goto flash_deinit;
	}

	/*
	 * Note that we do not need to deinitialize the flash memory because the
	 * device performed a reset anyway.
	 */

	return ERROR_OK;

	int retval_deinit;
flash_deinit:
	retval_deinit = artery_deinit_flash(bank);

	if (retval_deinit != ERROR_OK)
		return retval_deinit;

	return retval;
}

COMMAND_HANDLER(artery_handle_fap_disable_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);

	if (retval != ERROR_OK)
		return retval;

	enum artery_fap_level fap_level;
	retval = artery_get_fap(bank, &fap_level);

	if (retval != ERROR_OK) {
		command_print(CMD, "failed to read FAP state");
		return retval;
	}

	if (fap_level == ARTERY_FAP_LEVEL_DISABLED) {
		command_print(CMD, "flash access protection is not enabled");
		return ERROR_FAIL;
	}

	retval = artery_disable_fap(bank);

	if (retval != ERROR_OK) {
		command_print(CMD, "failed to disable flash access protection");
		return retval;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(artery_handle_fap_state_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);

	if (retval != ERROR_OK)
		return retval;

	enum artery_fap_level fap_level;
	retval = artery_get_fap(bank, &fap_level);

	if (retval != ERROR_OK) {
		command_print(CMD, "failed to read FAP level");
		return retval;
	}

	const bool fap_enabled = fap_level != ARTERY_FAP_LEVEL_DISABLED;
	command_print(CMD, "%u", fap_enabled);

	return ERROR_OK;
}

COMMAND_HANDLER(artery_handle_mass_erase_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);

	if (retval != ERROR_OK)
		return retval;

	retval = artery_mass_erase(bank);

	if (retval != ERROR_OK)  {
		command_print(CMD, "Mass erase failed");
		return retval;
	}

	return ERROR_OK;
}

static const struct command_registration artery_fap_command_handlers[] = {
	{
		.name = "enable",
		.handler = artery_handle_fap_enable_command,
		.mode = COMMAND_EXEC,
		.usage = "<bank_id>",
		.help = "Enable flash access protection (FAP)",
	},
	{
		.name = "disable",
		.handler = artery_handle_fap_disable_command,
		.mode = COMMAND_EXEC,
		.usage = "<bank_id>",
		.help = "Disable flash access protection (FAP)",
	},
	{
		.name = "state",
		.handler = artery_handle_fap_state_command,
		.mode = COMMAND_EXEC,
		.usage = "<bank_id>",
		.help = "Get the flash access protection (FAP) state",
	},

	COMMAND_REGISTRATION_DONE,
};

static const struct command_registration artery_exec_command_handlers[] = {
	{
		.name = "fap",
		.mode = COMMAND_ANY,
		.help = "flash access protection (FAP) command group",
		.usage = "",
		.chain = artery_fap_command_handlers,
	},
	{
		.name = "mass_erase",
		.handler = artery_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "<bank_id>",
		.help = "Erase entire flash memory",
	},
	COMMAND_REGISTRATION_DONE,
};

static const struct command_registration artery_command_handlers[] = {
	{
		.name = "artery",
		.mode = COMMAND_ANY,
		.help = "artery flash command group",
		.usage = "",
		.chain = artery_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE,
};

const struct flash_driver artery_flash = {
	.name = "artery",
	.commands = artery_command_handlers,
	.flash_bank_command = artery_flash_bank_command,
	.erase = artery_erase,
	.protect = artery_protect,
	.write = artery_write,
	.read = default_flash_read,
	.probe = artery_probe,
	.auto_probe = artery_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = artery_protect_check,
	.info = artery_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
