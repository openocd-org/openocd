// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 * Copyright (C) 2023-2025 Texas Instruments Incorporated - https://www.ti.com/
 *
 * NOR flash driver for MSPM0L and MSPM0G class of uC from Texas Instruments.
 *
 * See:
 * https://www.ti.com/microcontrollers-mcus-processors/arm-based-microcontrollers/arm-cortex-m0-mcus/overview.html
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/bits.h>
#include <helper/time_support.h>

/* MSPM0 Region memory map */
#define MSPM0_FLASH_BASE_NONMAIN        0x41C00000
#define MSPM0_FLASH_END_NONMAIN         0x41C00400
#define MSPM0_FLASH_BASE_MAIN           0x0
#define MSPM0_FLASH_BASE_DATA           0x41D00000

/* MSPM0 FACTORYREGION registers */
#define MSPM0_FACTORYREGION             0x41C40000
#define MSPM0_TRACEID                   (MSPM0_FACTORYREGION + 0x000)
#define MSPM0_DID                       (MSPM0_FACTORYREGION + 0x004)
#define MSPM0_USERID                    (MSPM0_FACTORYREGION + 0x008)
#define MSPM0_SRAMFLASH                 (MSPM0_FACTORYREGION + 0x018)

/* MSPM0 FCTL registers */
#define FLASH_CONTROL_BASE              0x400CD000
#define FCTL_REG_DESC                   (FLASH_CONTROL_BASE + 0x10FC)
#define FCTL_REG_CMDEXEC                (FLASH_CONTROL_BASE + 0x1100)
#define FCTL_REG_CMDTYPE                (FLASH_CONTROL_BASE + 0x1104)
#define FCTL_REG_CMDADDR                (FLASH_CONTROL_BASE + 0x1120)
#define FCTL_REG_CMDBYTEN               (FLASH_CONTROL_BASE + 0x1124)
#define FCTL_REG_CMDDATA0               (FLASH_CONTROL_BASE + 0x1130)
#define FCTL_REG_CMDWEPROTA             (FLASH_CONTROL_BASE + 0x11D0)
#define FCTL_REG_CMDWEPROTB             (FLASH_CONTROL_BASE + 0x11D4)
#define FCTL_REG_CMDWEPROTNM            (FLASH_CONTROL_BASE + 0x1210)
#define FCTL_REG_STATCMD                (FLASH_CONTROL_BASE + 0x13D0)

/* FCTL_STATCMD[CMDDONE] Bits */
#define FCTL_STATCMD_CMDDONE_MASK       0x00000001
#define FCTL_STATCMD_CMDDONE_STATDONE   0x00000001

/* FCTL_STATCMD[CMDPASS] Bits */
#define FCTL_STATCMD_CMDPASS_MASK       0x00000002
#define FCTL_STATCMD_CMDPASS_STATPASS   0x00000002

/*
 * FCTL_CMDEXEC Bits
 * FCTL_CMDEXEC[VAL] Bits
 */
#define FCTL_CMDEXEC_VAL_EXECUTE        0x00000001

/* FCTL_CMDTYPE[COMMAND] Bits */
#define FCTL_CMDTYPE_COMMAND_PROGRAM    0x00000001
#define FCTL_CMDTYPE_COMMAND_ERASE      0x00000002

/* FCTL_CMDTYPE[SIZE] Bits */
#define FCTL_CMDTYPE_SIZE_ONEWORD       0x00000000
#define FCTL_CMDTYPE_SIZE_SECTOR        0x00000040

/* FCTL_FEATURE_VER_B minimum */
#define FCTL_FEATURE_VER_B              0xA

#define MSPM0_MAX_PROTREGS              3

#define MSPM0_FLASH_TIMEOUT_MS          8000
#define ERR_STRING_MAX                  255

/* SYSCTL BASE */
#define SYSCTL_BASE                     0x400AF000
#define SYSCTL_SECCFG_SECSTATUS         (SYSCTL_BASE + 0x00003048)

/* TI manufacturer ID */
#define TI_MANUFACTURER_ID              0x17

/* Defines for probe status */
#define MSPM0_NO_ID_FOUND               0
#define MSPM0_DEV_ID_FOUND              1
#define MSPM0_DEV_PART_ID_FOUND         2

struct mspm0_flash_bank {
	/* chip id register */
	uint32_t did;
	/* Device Unique ID register */
	uint32_t traceid;
	unsigned char version;

	const char *name;

	/* Decoded flash information */
	unsigned int data_flash_size_kb;
	unsigned int main_flash_size_kb;
	unsigned int main_flash_num_banks;
	unsigned int sector_size;
	/* Decoded SRAM information */
	unsigned int sram_size_kb;

	/* Flash word size: 64 bit = 8, 128bit = 16 bytes */
	unsigned char flash_word_size_bytes;

	/* Protection register stuff */
	unsigned int protect_reg_base;
	unsigned int protect_reg_count;

	/* Flashctl version: A - CMDWEPROTA/B, B- CMDWEPROTB */
	unsigned char flash_version;
};

struct mspm0_part_info {
	const char *part_name;
	unsigned short part;
	unsigned char variant;
};

struct mspm0_family_info {
	const char *family_name;
	unsigned short part_num;
	unsigned char part_count;
	const struct mspm0_part_info *part_info;
};

/* https://www.ti.com/lit/ds/symlink/mspm0l1346.pdf Table 8-13 and so on */
static const struct mspm0_part_info mspm0l_parts[] = {
	{ "MSPM0L1105TDGS20R", 0x51DB, 0x16 },
	{ "MSPM0L1105TDGS28R", 0x51DB, 0x83 },
	{ "MSPM0L1105TDYYR", 0x51DB, 0x54 },
	{ "MSPM0L1105TRGER", 0x51DB, 0x86 },
	{ "MSPM0L1105TRHBR", 0x51DB, 0x68 },
	{ "MSPM0L1106TDGS20R", 0x5552, 0x4B },
	{ "MSPM0L1106TDGS28R", 0x5552, 0x98 },
	{ "MSPM0L1106TDYYR", 0x5552, 0x9D },
	{ "MSPM0L1106TRGER", 0x5552, 0x90 },
	{ "MSPM0L1106TRHBR", 0x5552, 0x53 },
	{ "MSPM0L1303SRGER", 0xef0, 0x17 },
	{ "MSPM0L1303TRGER", 0xef0, 0xe2 },
	{ "MSPM0L1304QDGS20R", 0xd717, 0x91 },
	{ "MSPM0L1304QDGS28R", 0xd717, 0xb6 },
	{ "MSPM0L1304QDYYR", 0xd717, 0xa0 },
	{ "MSPM0L1304QRHBR", 0xd717, 0xa9 },
	{ "MSPM0L1304SDGS20R", 0xd717, 0xfa },
	{ "MSPM0L1304SDGS28R", 0xd717, 0x73 },
	{ "MSPM0L1304SDYYR", 0xd717, 0xb7 },
	{ "MSPM0L1304SRGER", 0xd717, 0x26 },
	{ "MSPM0L1304SRHBR", 0xd717, 0xe4 },
	{ "MSPM0L1304TDGS20R", 0xd717, 0x33 },
	{ "MSPM0L1304TDGS28R", 0xd717, 0xa8 },
	{ "MSPM0L1304TDYYR", 0xd717, 0xf9 },
	{ "MSPM0L1304TRGER", 0xd717, 0xb7 },
	{ "MSPM0L1304TRHBR", 0xd717, 0x5a },
	{ "MSPM0L1305QDGS20R", 0x4d03, 0xb7 },
	{ "MSPM0L1305QDGS28R", 0x4d03, 0x74 },
	{ "MSPM0L1305QDYYR", 0x4d03, 0xec },
	{ "MSPM0L1305QRHBR", 0x4d03, 0x78 },
	{ "MSPM0L1305SDGS20R", 0x4d03, 0xc7 },
	{ "MSPM0L1305SDGS28R", 0x4d03, 0x64 },
	{ "MSPM0L1305SDYYR", 0x4d03, 0x91 },
	{ "MSPM0L1305SRGER", 0x4d03, 0x73 },
	{ "MSPM0L1305SRHBR", 0x4d03, 0x2d },
	{ "MSPM0L1305TDGS20R", 0x4d03, 0xa0 },
	{ "MSPM0L1305TDGS28R", 0x4d03, 0xfb },
	{ "MSPM0L1305TDYYR", 0x4d03, 0xde },
	{ "MSPM0L1305TRGER", 0x4d03, 0xea },
	{ "MSPM0L1305TRHBR", 0x4d03, 0x85 },
	{ "MSPM0L1306QDGS20R", 0xbb70, 0x59 },
	{ "MSPM0L1306QDGS28R", 0xbb70, 0xf7 },
	{ "MSPM0L1306QDYYR", 0xbb70, 0x9f },
	{ "MSPM0L1306QRHBR", 0xbb70, 0xc2 },
	{ "MSPM0L1306SDGS20R", 0xbb70, 0xf4 },
	{ "MSPM0L1306SDGS28R", 0xbb70, 0x5 },
	{ "MSPM0L1306SDYYR", 0xbb70, 0xe },
	{ "MSPM0L1306SRGER", 0xbb70, 0x7f },
	{ "MSPM0L1306SRHBR", 0xbb70, 0x3c },
	{ "MSPM0L1306TDGS20R", 0xbb70, 0xa },
	{ "MSPM0L1306TDGS28R", 0xbb70, 0x63 },
	{ "MSPM0L1306TDYYR", 0xbb70, 0x35 },
	{ "MSPM0L1306TRGER", 0xbb70, 0xaa },
	{ "MSPM0L1306TRHBR", 0xbb70, 0x52 },
	{ "MSPM0L1343TDGS20R", 0xb231, 0x2e },
	{ "MSPM0L1344TDGS20R", 0x40b0, 0xd0 },
	{ "MSPM0L1345TDGS28R", 0x98b4, 0x74 },
	{ "MSPM0L1346TDGS28R", 0xf2b5, 0xef },
};

/* https://www.ti.com/lit/ds/symlink/mspm0g3506.pdf Table 8-20 */
static const struct mspm0_part_info mspm0g_parts[] = {
	{ "MSPM0G1105TPTR", 0x8934, 0xD },
	{ "MSPM0G1105TRGZR", 0x8934, 0xFE },
	{ "MSPM0G1106TPMR", 0x477B, 0xD4 },
	{ "MSPM0G1106TPTR", 0x477B, 0x71 },
	{ "MSPM0G1106TRGZR", 0x477B, 0xBB },
	{ "MSPM0G1106TRHBR", 0x477B, 0x0 },
	{ "MSPM0G1107TDGS28R", 0x807B, 0x82 },
	{ "MSPM0G1107TPMR", 0x807B, 0xB3 },
	{ "MSPM0G1107TPTR", 0x807B, 0x32 },
	{ "MSPM0G1107TRGER", 0x807B, 0x79 },
	{ "MSPM0G1107TRGZR", 0x807B, 0x20 },
	{ "MSPM0G1107TRHBR", 0x807B, 0xBC },
	{ "MSPM0G1505SDGS28R", 0x13C4, 0x73 },
	{ "MSPM0G1505SPMR", 0x13C4, 0x53 },
	{ "MSPM0G1505SPTR", 0x13C4, 0x3E },
	{ "MSPM0G1505SRGER", 0x13C4, 0x47 },
	{ "MSPM0G1505SRGZR", 0x13C4, 0x34 },
	{ "MSPM0G1505SRHBR", 0x13C4, 0x30 },
	{ "MSPM0G1506SDGS28R", 0x5AE0, 0x3A },
	{ "MSPM0G1506SPMR", 0x5AE0, 0xF6 },
	{ "MSPM0G1506SRGER", 0x5AE0, 0x67 },
	{ "MSPM0G1506SRGZR", 0x5AE0, 0x75 },
	{ "MSPM0G1506SRHBR", 0x5AE0, 0x57 },
	{ "MSPM0G1507SDGS28R", 0x2655, 0x6D },
	{ "MSPM0G1507SPMR", 0x2655, 0x97 },
	{ "MSPM0G1507SRGER", 0x2655, 0x83 },
	{ "MSPM0G1507SRGZR", 0x2655, 0xD3 },
	{ "MSPM0G1507SRHBR", 0x2655, 0x4D },
	{ "MSPM0G3105SDGS20R", 0x4749, 0x21 },
	{ "MSPM0G3105SDGS28R", 0x4749, 0xDD },
	{ "MSPM0G3105SRHBR", 0x4749, 0xBE },
	{ "MSPM0G3106SDGS20R", 0x54C7, 0xD2 },
	{ "MSPM0G3106SDGS28R", 0x54C7, 0xB9 },
	{ "MSPM0G3106SRHBR", 0x54C7, 0x67 },
	{ "MSPM0G3107SDGS20R", 0xAB39, 0x5C },
	{ "MSPM0G3107SDGS28R", 0xAB39, 0xCC },
	{ "MSPM0G3107SRHBR", 0xAB39, 0xB7 },
	{ "MSPM0G3505SDGS28R", 0xc504, 0x8e },
	{ "MSPM0G3505SPMR", 0xc504, 0x1d },
	{ "MSPM0G3505SPTR", 0xc504, 0x93 },
	{ "MSPM0G3505SRGZR", 0xc504, 0xc7 },
	{ "MSPM0G3505SRHBR", 0xc504, 0xe7 },
	{ "MSPM0G3505TDGS28R", 0xc504, 0xdf },
	{ "MSPM0G3506SDGS28R", 0x151f, 0x8 },
	{ "MSPM0G3506SPMR", 0x151f, 0xd4 },
	{ "MSPM0G3506SPTR", 0x151f, 0x39 },
	{ "MSPM0G3506SRGZR", 0x151f, 0xfe },
	{ "MSPM0G3506SRHBR", 0x151f, 0xb5 },
	{ "MSPM0G3507SDGS28R", 0xae2d, 0xca },
	{ "MSPM0G3507SPMR", 0xae2d, 0xc7 },
	{ "MSPM0G3507SPTR", 0xae2d, 0x3f },
	{ "MSPM0G3507SRGZR", 0xae2d, 0xf7 },
	{ "MSPM0G3507SRHBR", 0xae2d, 0x4c },
	{ "M0G3107QPMRQ1", 0x4e2f, 0x51 },
	{ "M0G3107QPTRQ1", 0x4e2f, 0xc7},
	{ "M0G3107QRGZRQ1", 0x4e2f, 0x8a },
	{ "M0G3107QRHBRQ1", 0x4e2f, 0x9a},
	{ "M0G3107QDGS28RQ1", 0x4e2f, 0xd5},
	{ "M0G3107QDGS28RQ1", 0x4e2f, 0x67},
	{ "M0G3107QDGS20RQ1", 0x4e2f, 0xfd},
	{ "M0G3106QPMRQ1", 0x54C7, 0x08},
	{ "M0G3105QDGS32RQ1", 0x1349, 0x08},
	{ "M0G3106QPTRQ1", 0x54C7, 0x3F},
	{ "M0G3105QDGS28RQ1", 0x1349, 0x1B},
	{ "M0G3106QRGZRQ1", 0x94AD, 0xE6},
	{ "M0G3105QDGS20RQ1", 0x1349, 0xFB},
	{ "M0G3106QRHBRQ1", 0x94AD, 0x20},
	{ "M0G3106QDGS32RQ1", 0x94AD, 0x8D},
	{ "M0G3106QDGS28RQ1", 0x94AD, 0x03},
	{ "M0G3106QDGS20RQ1", 0x94AD, 0x6F},
	{ "M0G3105QPMRQ1", 0x1349, 0xD0},
	{ "M0G3105QPTRQ1", 0x1349, 0xEF},
	{ "M0G3105QRGZRQ1", 0x1349, 0x70},
	{ "M0G3105QRHBRQ1", 0x1349, 0x01},
};

/* https://www.ti.com/lit/gpn/mspm0c1104 Table 8-12 and so on */
static const struct mspm0_part_info mspm0c_parts[] = {
	{ "MSPS003F4SPW20R", 0x57b3, 0x70},
	{ "MSPM0C1104SDGS20R", 0x57b3, 0x71},
	{ "MSPM0C1104SRUKR", 0x57b3, 0x73},
	{ "MSPM0C1104SDYYR", 0x57b3, 0x75},
	{ "MSPM0C1104SDDFR", 0x57b3, 0x77},
	{ "MSPM0C1104SDSGR", 0x57b3, 0x79},
};

/* https://www.ti.com/lit/gpn/MSPM0L2228 Table 8-16 and so on */
static const struct mspm0_part_info mspm0lx22x_parts[] = {
	{ "MSPM0L1227SRGER", 0x7C32, 0xF1},
	{ "MSPM0L1227SPTR", 0x7C32, 0xC9},
	{ "MSPM0L1227SPMR", 0x7C32, 0x1C},
	{ "MSPM0L1227SPNAR", 0x7C32, 0x91},
	{ "MSPM0L1227SPNR", 0x7C32, 0x39},
	{ "MSPM0L1228SRGER", 0x33F7, 0x13},
	{ "MSPM0L1228SRHBR", 0x33F7, 0x3A},
	{ "MSPM0L1228SRGZR", 0x33F7, 0xBC},
	{ "MSPM0L1228SPTR", 0x33F7, 0xF8},
	{ "MSPM0L1228SPMR", 0x33F7, 0xCE},
	{ "MSPM0L1228SPNAR", 0x33F7, 0x59},
	{ "MSPM0L1228SPNR", 0x33F7, 0x7},
	{ "MSPM0L2227SRGZR", 0x5E8F, 0x90},
	{ "MSPM0L2227SPTR", 0x5E8F, 0xA},
	{ "MSPM0L2227SPMR", 0x5E8F, 0x6D},
	{ "MSPM0L2227SPNAR", 0x5E8F, 0x24},
	{ "MSPM0L2227SPNR", 0x5E8F, 0x68},
	{ "MSPM0L2228SRGZR", 0x2C38, 0xB8},
	{ "MSPM0L2228SPTR", 0x2C38, 0x25},
	{ "MSPM0L2228SPMR", 0x2C38, 0x6E},
	{ "MSPM0L2228SPNAR", 0x2C38, 0x63},
	{ "MSPM0L2228SPNR", 0x2C38, 0x3C},
};

static const struct mspm0_family_info mspm0_finf[] = {
	{ "MSPM0L", 0xbb82, ARRAY_SIZE(mspm0l_parts), mspm0l_parts },
	{ "MSPM0Lx22x", 0xbb9f, ARRAY_SIZE(mspm0lx22x_parts), mspm0lx22x_parts },
	{ "MSPM0G", 0xbb88, ARRAY_SIZE(mspm0g_parts), mspm0g_parts },
	{ "MSPM0C", 0xbba1, ARRAY_SIZE(mspm0c_parts), mspm0c_parts },
};

/*
 *	OpenOCD command interface
 */

/*
 * flash_bank mspm0 <base> <size> 0 0 <target#>
 */
FLASH_BANK_COMMAND_HANDLER(mspm0_flash_bank_command)
{
	struct mspm0_flash_bank *mspm0_info;

	switch (bank->base) {
	case MSPM0_FLASH_BASE_NONMAIN:
	case MSPM0_FLASH_BASE_MAIN:
	case MSPM0_FLASH_BASE_DATA:
		break;
	default:
		LOG_ERROR("Invalid bank address " TARGET_ADDR_FMT, bank->base);
		return ERROR_FAIL;
	}

	mspm0_info = calloc(1, sizeof(struct mspm0_flash_bank));
	if (!mspm0_info) {
		LOG_ERROR("%s: Out of memory for mspm0_info!", __func__);
		return ERROR_FAIL;
	}

	bank->driver_priv = mspm0_info;

	mspm0_info->sector_size = 0x400;

	return ERROR_OK;
}

/*
 * Chip identification and status
 */
static int get_mspm0_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct mspm0_flash_bank *mspm0_info = bank->driver_priv;

	if (mspm0_info->did == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	command_print_sameline(cmd,
		"\nTI MSPM0 information: Chip is "
		"%s rev %d Device Unique ID: 0x%" PRIu32 "\n",
		mspm0_info->name, mspm0_info->version,
		mspm0_info->traceid);
	command_print_sameline(cmd,
		"main flash: %uKiB in %u bank(s), sram: %uKiB, data flash: %uKiB",
		mspm0_info->main_flash_size_kb,
		mspm0_info->main_flash_num_banks, mspm0_info->sram_size_kb,
		mspm0_info->data_flash_size_kb);

	return ERROR_OK;
}

/* Extract a bitfield helper */
static unsigned int mspm0_extract_val(unsigned int var, unsigned char hi, unsigned char lo)
{
	return (var & GENMASK(hi, lo)) >> lo;
}

static int mspm0_read_part_info(struct flash_bank *bank)
{
	struct mspm0_flash_bank *mspm0_info = bank->driver_priv;
	struct target *target = bank->target;
	const struct mspm0_family_info *minfo = NULL;

	/* Read and parse chip identification and flash version register */
	uint32_t did;
	int retval = target_read_u32(target, MSPM0_DID, &did);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read device ID");
		return retval;
	}
	retval = target_read_u32(target, MSPM0_TRACEID, &mspm0_info->traceid);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read trace ID");
		return retval;
	}
	uint32_t userid;
	retval = target_read_u32(target, MSPM0_USERID, &userid);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read user ID");
		return retval;
	}
	uint32_t flashram;
	retval = target_read_u32(target, MSPM0_SRAMFLASH, &flashram);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read sramflash register");
		return retval;
	}
	uint32_t flashdesc;
	retval = target_read_u32(target, FCTL_REG_DESC, &flashdesc);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read flashctl description register");
		return retval;
	}

	unsigned char version = mspm0_extract_val(did, 31, 28);
	unsigned short pnum = mspm0_extract_val(did, 27, 12);
	unsigned char variant = mspm0_extract_val(userid, 23, 16);
	unsigned short part = mspm0_extract_val(userid, 15, 0);
	unsigned short manufacturer = mspm0_extract_val(did, 11, 1);

	/*
	 * Valid DIE and manufacturer ID?
	 * Check the ALWAYS_1 bit to be 1 and manufacturer to be 0x17. All MSPM0
	 * devices within the Device ID field of the factory constants will
	 * always read 0x17 as it is TI's JEDEC bank and company code. If 1
	 * and 0x17 is not read from their respective registers then it truly
	 * is not a MSPM0 device so we will return an error instead of
	 * going any further.
	 */
	if (!(did & BIT(0)) || !(manufacturer & TI_MANUFACTURER_ID)) {
		LOG_WARNING("Unknown Device ID[0x%" PRIx32 "], cannot identify target",
			did);
		LOG_DEBUG("did 0x%" PRIx32 ", traceid 0x%" PRIx32 ", userid 0x%" PRIx32
			", flashram 0x%" PRIx32 "", did, mspm0_info->traceid, userid,
			flashram);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* Initialize master index selector and probe status*/
	unsigned char minfo_idx = 0xff;
	unsigned char probe_status = MSPM0_NO_ID_FOUND;

	/* Check if we at least know the family of devices */
	for (unsigned int i = 0; i < ARRAY_SIZE(mspm0_finf); i++) {
		if (mspm0_finf[i].part_num == pnum) {
			minfo_idx = i;
			minfo = &mspm0_finf[i];
			probe_status = MSPM0_DEV_ID_FOUND;
			break;
		}
	}

	/* Initialize part index selector*/
	unsigned char pinfo_idx = 0xff;

	/*
	 * If we can identify the part number then we will attempt to identify
	 * the specific chip. Otherwise, if we do not know the part number then
	 * it would be useless to identify the specific chip.
	 */
	if (probe_status == MSPM0_DEV_ID_FOUND) {
		/* Can we specifically identify the chip */
		for (unsigned int i = 0; i < minfo->part_count; i++) {
			if (minfo->part_info[i].part == part
				&& minfo->part_info[i].variant == variant) {
				pinfo_idx = i;
				probe_status = MSPM0_DEV_PART_ID_FOUND;
				break;
			}
		}
	}

	/*
	 * We will check the status of our probe within this switch-case statement
	 * using these three scenarios.
	 *
	 * 1) Device, part, and variant ID is unknown.
	 * 2) Device ID is known but the part/variant ID is unknown.
	 * 3) Device ID and part/variant ID is known
	 *
	 * For scenario 1, we allow the user to continue because if the
	 * manufacturer matches TI's JEDEC value and ALWAYS_1 from the device ID
	 * field is correct then the assumption the user is using an MSPM0 device
	 * can be made.
	 */
	switch (probe_status) {
	case MSPM0_NO_ID_FOUND:
		mspm0_info->name = "mspm0x";
		LOG_INFO("Unidentified PART[0x%x]/variant[0x%x"
			"], unknown DeviceID[0x%x"
			"]. Attempting to proceed as %s.", part, variant, pnum,
			mspm0_info->name);
		break;
	case MSPM0_DEV_ID_FOUND:
		mspm0_info->name = mspm0_finf[minfo_idx].family_name;
		LOG_INFO("Unidentified PART[0x%x]/variant[0x%x"
			"], known DeviceID[0x%x"
			"]. Attempting to proceed as %s.", part, variant, pnum,
			mspm0_info->name);
		break;
	case MSPM0_DEV_PART_ID_FOUND:
	default:
		mspm0_info->name = mspm0_finf[minfo_idx].part_info[pinfo_idx].part_name;
		LOG_DEBUG("Part: %s detected", mspm0_info->name);
		break;
	}

	mspm0_info->did = did;
	mspm0_info->version = version;
	mspm0_info->data_flash_size_kb = mspm0_extract_val(flashram, 31, 26);
	mspm0_info->main_flash_size_kb = mspm0_extract_val(flashram, 11, 0);
	mspm0_info->main_flash_num_banks = mspm0_extract_val(flashram, 13, 12) + 1;
	mspm0_info->sram_size_kb = mspm0_extract_val(flashram, 25, 16);
	mspm0_info->flash_version = mspm0_extract_val(flashdesc, 15, 12);

	/*
	 * Hardcode flash_word_size unless we find some other pattern
	 * See section 7.7 (Foot note mentions the flash word size).
	 * almost all values seem to be 8 bytes, but if there are variance,
	 * then we should update mspm0_part_info structure with this info.
	 */
	mspm0_info->flash_word_size_bytes = 8;

	LOG_DEBUG("Detected: main flash: %uKb in %u banks, sram: %uKb, data flash: %uKb",
		mspm0_info->main_flash_size_kb, mspm0_info->main_flash_num_banks,
		mspm0_info->sram_size_kb, mspm0_info->data_flash_size_kb);

	return ERROR_OK;
}

/*
 * Decode error values
 */
static const struct {
	const unsigned char bit_offset;
	const char *fail_string;
} mspm0_fctl_fail_decode_strings[] = {
	{ 2, "CMDINPROGRESS" },
	{ 4, "FAILWEPROT" },
	{ 5, "FAILVERIFY" },
	{ 6, "FAILILLADDR" },
	{ 7, "FAILMODE" },
	{ 12, "FAILMISC" },
};

static const char *mspm0_fctl_translate_ret_err(unsigned int return_code)
{
	for (unsigned int i = 0; i < ARRAY_SIZE(mspm0_fctl_fail_decode_strings); i++) {
		if (return_code & BIT(mspm0_fctl_fail_decode_strings[i].bit_offset))
			return mspm0_fctl_fail_decode_strings[i].fail_string;
	}

	/* If unknown error notify the user*/
	return "FAILUNKNOWN";
}

static int mspm0_fctl_get_sector_reg(struct flash_bank *bank, unsigned int addr,
	unsigned int *reg, unsigned int *sector_mask)
{
	struct mspm0_flash_bank *mspm0_info = bank->driver_priv;
	struct target *target = bank->target;
	int ret = ERROR_OK;
	unsigned int sector_num = (addr >> 10);
	unsigned int sector_in_bank = sector_num;
	unsigned int phys_sector_num = sector_num;
	uint32_t sysctl_sec_status;
	unsigned int exec_upper_bank;

	/*
	 * If the device has dual banks we will need to check if it is configured
	 * to execute from the upper bank. In the scenario that we are executing
	 * from upper bank then we will need to protect it using CMDWEPROTA rather
	 * than CMDWEPROTB. We also need to take into account what sector
	 * we're using when going between banks.
	 */
	if (mspm0_info->main_flash_num_banks > 1 &&
		bank->base == MSPM0_FLASH_BASE_MAIN) {
		ret = target_read_u32(target, SYSCTL_SECCFG_SECSTATUS, &sysctl_sec_status);
		if (ret != ERROR_OK)
			return ret;
		exec_upper_bank = mspm0_extract_val(sysctl_sec_status, 12, 12);
		if (exec_upper_bank) {
			if (sector_num > (mspm0_info->main_flash_size_kb / 2)) {
				phys_sector_num =
					sector_num - (mspm0_info->main_flash_size_kb / 2);
			} else {
				phys_sector_num =
					sector_num + (mspm0_info->main_flash_size_kb / 2);
			}
		}
		sector_in_bank =
			sector_num % (mspm0_info->main_flash_size_kb /
			mspm0_info->main_flash_num_banks);
	}

	/*
	 * NOTE: MSPM0 devices of version A will use CMDWEPROTA and CMDWEPROTB
	 * for MAIN flash. CMDWEPROTC is included in the TRM/DATASHEET but for
	 * all practical purposes, it is considered reserved. If the flash
	 * version on the device is version B, then we will only use
	 * CMDWEPROTB for MAIN and DATA flash if the device has it.
	 */
	switch (bank->base) {
	case MSPM0_FLASH_BASE_MAIN:
	case MSPM0_FLASH_BASE_DATA:
		if (mspm0_info->flash_version < FCTL_FEATURE_VER_B) {
			/* Use CMDWEPROTA */
			if (phys_sector_num < 32) {
				*sector_mask = BIT(phys_sector_num);
				*reg = FCTL_REG_CMDWEPROTA;
			}

			/* Use CMDWEPROTB */
			if (phys_sector_num >= 32 && sector_in_bank < 256) {
				/* Dual bank system */
				if (mspm0_info->main_flash_num_banks > 1)
					*sector_mask = BIT(sector_in_bank / 8);
				else	/* Single bank system */
					*sector_mask = BIT((sector_in_bank - 32) / 8);
				*reg = FCTL_REG_CMDWEPROTB;
			}
		} else {
			*sector_mask = BIT((sector_in_bank / 8) % 32);
			*reg = FCTL_REG_CMDWEPROTB;
		}
		break;
	case MSPM0_FLASH_BASE_NONMAIN:
		*sector_mask = BIT(sector_num % 32);
		*reg = FCTL_REG_CMDWEPROTNM;
		break;
	default:
		/*
		 * Not expected to reach here due to check in mspm0_address_check()
		 * but adding it as another layer of safety.
		 */
		ret = ERROR_FLASH_DST_OUT_OF_BANK;
		break;
	}

	if (ret != ERROR_OK)
		LOG_ERROR("Unable to map sector protect reg for address 0x%08x", addr);

	return ret;
}

static int mspm0_address_check(struct flash_bank *bank, unsigned int addr)
{
	struct mspm0_flash_bank *mspm0_info = bank->driver_priv;
	unsigned int flash_main_size = mspm0_info->main_flash_size_kb * 1024;
	unsigned int flash_data_size = mspm0_info->data_flash_size_kb * 1024;
	int ret = ERROR_FLASH_SECTOR_INVALID;

	/*
	 * Before unprotecting any memory lets make sure that the address and
	 * bank given is a known bank and whether or not the address falls under
	 * the proper bank.
	 */
	switch (bank->base) {
	case MSPM0_FLASH_BASE_MAIN:
		if (addr <= (MSPM0_FLASH_BASE_MAIN + flash_main_size))
			ret = ERROR_OK;
		break;
	case MSPM0_FLASH_BASE_NONMAIN:
		if (addr >= MSPM0_FLASH_BASE_NONMAIN && addr <= MSPM0_FLASH_END_NONMAIN)
			ret = ERROR_OK;
		break;
	case MSPM0_FLASH_BASE_DATA:
		if (addr >= MSPM0_FLASH_BASE_DATA &&
		addr <= (MSPM0_FLASH_BASE_DATA + flash_data_size))
			ret = ERROR_OK;
		break;
	default:
		ret = ERROR_FLASH_DST_OUT_OF_BANK;
		break;
	}

	return ret;
}

static int mspm0_fctl_unprotect_sector(struct flash_bank *bank, unsigned int addr)
{
	struct target *target = bank->target;
	unsigned int reg = 0x0;
	uint32_t sector_mask = 0x0;
	int ret;

	ret = mspm0_address_check(bank, addr);
	switch (ret) {
	case ERROR_FLASH_SECTOR_INVALID:
		LOG_ERROR("Unable to map sector protect reg for address 0x%08x", addr);
		break;
	case ERROR_FLASH_DST_OUT_OF_BANK:
		LOG_ERROR("Unable to determine which bank to use 0x%08x", addr);
		break;
	default:
		mspm0_fctl_get_sector_reg(bank, addr, &reg, &sector_mask);
		ret = target_write_u32(target, reg, ~sector_mask);
		break;
	}

	return ret;
}

static int mspm0_fctl_cfg_command(struct flash_bank *bank,
	uint32_t addr,
	uint32_t cmd,
	uint32_t byte_en)
{
	struct target *target = bank->target;

	/*
	 * Configure the flash operation within the CMDTYPE register, byte_en
	 * bits if needed, and then set the address where the flash operation
	 * will execute.
	 */
	int retval = target_write_u32(target, FCTL_REG_CMDTYPE, cmd);
	if (retval != ERROR_OK)
		return retval;
	if (byte_en != 0) {
		retval = target_write_u32(target, FCTL_REG_CMDBYTEN, byte_en);
		if (retval != ERROR_OK)
			return retval;
	}

	return target_write_u32(target, FCTL_REG_CMDADDR, addr);
}

static int mspm0_fctl_wait_cmd_ok(struct flash_bank *bank)
{
	struct target *target = bank->target;
	uint32_t return_code = 0;
	int64_t start_ms;
	int64_t elapsed_ms;

	start_ms = timeval_ms();
	while ((return_code & FCTL_STATCMD_CMDDONE_MASK) != FCTL_STATCMD_CMDDONE_STATDONE) {
		int retval = target_read_u32(target, FCTL_REG_STATCMD, &return_code);
		if (retval != ERROR_OK)
			return retval;

		elapsed_ms = timeval_ms() - start_ms;
		if (elapsed_ms > 500)
			keep_alive();
		if (elapsed_ms > MSPM0_FLASH_TIMEOUT_MS)
			break;
	}

	if ((return_code & FCTL_STATCMD_CMDPASS_MASK) != FCTL_STATCMD_CMDPASS_STATPASS) {
		LOG_ERROR("Flash command failed: %s", mspm0_fctl_translate_ret_err(return_code));
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int mspm0_fctl_sector_erase(struct flash_bank *bank, uint32_t addr)
{
	struct target *target = bank->target;

	/*
	 * TRM Says:
	 * Note that the CMDWEPROTx registers are reset to a protected state
	 * at the end of all program and erase operations.  These registers
	 * must be re-configured by software before a new operation is
	 * initiated.
	 *
	 * This means that as we start erasing sector by sector, the protection
	 * registers are reset and need to be unprotected *again* for the next
	 * erase operation. Unfortunately, this means that we cannot do a unitary
	 * unprotect operation independent of flash erase operation
	 */
	int retval = mspm0_fctl_unprotect_sector(bank, addr);
	if (retval != ERROR_OK) {
		LOG_ERROR("Unprotecting sector of memory at address 0x%08" PRIx32
			" failed", addr);
		return retval;
	}

	/* Actual erase operation */
	retval = mspm0_fctl_cfg_command(bank, addr,
		(FCTL_CMDTYPE_COMMAND_ERASE | FCTL_CMDTYPE_SIZE_SECTOR), 0);
	if (retval != ERROR_OK)
		return retval;
	retval = target_write_u32(target, FCTL_REG_CMDEXEC, FCTL_CMDEXEC_VAL_EXECUTE);
	if (retval != ERROR_OK)
		return retval;

	return mspm0_fctl_wait_cmd_ok(bank);
}

static int mspm0_protect_check(struct flash_bank *bank)
{
	struct mspm0_flash_bank *mspm0_info = bank->driver_priv;

	if (mspm0_info->did == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	/*
	 * TRM Says:
	 * Note that the CMDWEPROTx registers are reset to a protected state
	 * at the end of all program and erase operations.  These registers
	 * must be re-configured by software before a new operation is
	 * initiated.
	 *
	 * This means that when any flash operation is performed at a block level,
	 * the block is locked back again. This prevents usage where we can set a
	 * protection level once at the flash level and then do erase / write
	 * operation without touching the protection register (since it is
	 * reset by hardware automatically). In effect, we cannot use the hardware
	 * defined protection scheme in openOCD.
	 *
	 * To deal with this protection scheme, the CMDWEPROTx register that
	 * correlates to the sector is modified at the time of operation and as far
	 * openOCD is concerned, the flash operates as completely un-protected
	 * flash.
	 */
	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = 0;

	return ERROR_OK;
}

static int mspm0_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct target *target = bank->target;
	struct mspm0_flash_bank *mspm0_info = bank->driver_priv;
	int retval = ERROR_OK;
	uint32_t protect_reg_cache[MSPM0_MAX_PROTREGS];

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Please halt target for erasing flash");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (mspm0_info->did == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	/* Pick a copy of the current protection config for later restoration */
	for (unsigned int i = 0; i < mspm0_info->protect_reg_count; i++) {
		retval = target_read_u32(target,
			mspm0_info->protect_reg_base + (i * 4),
			&protect_reg_cache[i]);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed saving flashctl protection status");
			return retval;
		}
	}

	switch (bank->base) {
	case MSPM0_FLASH_BASE_MAIN:
		for (unsigned int csa = first; csa <= last; csa++) {
			unsigned int addr = csa * mspm0_info->sector_size;
			retval = mspm0_fctl_sector_erase(bank, addr);
			if (retval != ERROR_OK)
				LOG_ERROR("Sector erase on MAIN failed at address 0x%08x "
						"(sector: %u)", addr, csa);
		}
		break;
	case MSPM0_FLASH_BASE_NONMAIN:
		retval = mspm0_fctl_sector_erase(bank, MSPM0_FLASH_BASE_NONMAIN);
		if (retval != ERROR_OK)
			LOG_ERROR("Sector erase on NONMAIN failed");
		break;
	case MSPM0_FLASH_BASE_DATA:
		for (unsigned int csa = first; csa <= last; csa++) {
			unsigned int addr = (MSPM0_FLASH_BASE_DATA +
			(csa * mspm0_info->sector_size));
			retval = mspm0_fctl_sector_erase(bank, addr);
			if (retval != ERROR_OK)
				LOG_ERROR("Sector erase on DATA bank failed at address 0x%08x "
						"(sector: %u)", addr, csa);
		}
		break;
	default:
		LOG_ERROR("Invalid memory region access");
		retval = ERROR_FLASH_BANK_INVALID;
		break;
	}

	/* If there were any issues in our checks, return the error */
	if (retval != ERROR_OK)
		return retval;

	/*
	 * TRM Says:
	 * Note that the CMDWEPROTx registers are reset to a protected state
	 * at the end of all program and erase operations.  These registers
	 * must be re-configured by software before a new operation is
	 * initiated
	 * Let us just Dump the protection registers back to the system.
	 * That way we retain the protection status as requested by the user
	 */
	for (unsigned int i = 0; i < mspm0_info->protect_reg_count; i++) {
		retval = target_write_u32(target, mspm0_info->protect_reg_base + (i * 4),
			protect_reg_cache[i]);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed re-applying protection status of flashctl");
			return retval;
		}
	}

	return retval;
}

static int mspm0_write(struct flash_bank *bank, const unsigned char *buffer,
	unsigned int offset, unsigned int count)
{
	struct target *target = bank->target;
	struct mspm0_flash_bank *mspm0_info = bank->driver_priv;
	uint32_t protect_reg_cache[MSPM0_MAX_PROTREGS];
	int retval;

	/*
	 * XXX: TRM Says:
	 * The number of program operations applied to a given word line must be
	 * monitored to ensure that the maximum word line program limit before
	 * erase is not violated.
	 *
	 * There is no reasonable way we can maintain that state in OpenOCD. So,
	 * Let the manufacturing path figure this out.
	 */

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Please halt target for programming flash");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (mspm0_info->did == 0)
		return ERROR_FLASH_BANK_NOT_PROBED;

	/*
	 * Pick a copy of the current protection config for later restoration
	 * We need to restore these regs after every write, so instead of trying
	 * to figure things out on the fly, we just context save and restore
	 */
	for (unsigned int i = 0; i < mspm0_info->protect_reg_count; i++) {
		retval = target_read_u32(target,
			mspm0_info->protect_reg_base + (i * 4),
			&protect_reg_cache[i]);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed saving flashctl protection status");
			return retval;
		}
	}

	/* Add proper memory offset for bank being written to */
	unsigned int addr = bank->base + offset;

	while (count) {
		unsigned int num_bytes_to_write;
		uint32_t bytes_en;

		/*
		 * If count is not 64 bit aligned, we will do byte wise op to keep things simple
		 * Usually this might mean we need to additional write ops towards
		 * trailing edge, but that is a tiny penalty for image downloads.
		 * NOTE: we are going to assume the device does not support multi-word
		 * programming - there does not seem to be discoverability!
		 */
		if (count < mspm0_info->flash_word_size_bytes)
			num_bytes_to_write = count;
		else
			num_bytes_to_write = mspm0_info->flash_word_size_bytes;

		/* Data bytes to write */
		bytes_en = (1 << num_bytes_to_write) - 1;
		/* ECC chunks to write */
		switch (mspm0_info->flash_word_size_bytes) {
		case 8:
			bytes_en |= BIT(8);
			break;
		case 16:
			bytes_en |= BIT(16);
			bytes_en |= (num_bytes_to_write > 8) ? BIT(17) : 0;
			break;
		default:
			LOG_ERROR("Invalid flash_word_size_bytes %d",
				mspm0_info->flash_word_size_bytes);
			return ERROR_FAIL;
		}

		retval = mspm0_fctl_cfg_command(bank, addr,
			(FCTL_CMDTYPE_COMMAND_PROGRAM | FCTL_CMDTYPE_SIZE_ONEWORD),
			bytes_en);
		if (retval != ERROR_OK)
			return retval;

		retval = mspm0_fctl_unprotect_sector(bank, addr);
		if (retval != ERROR_OK)
			return retval;

		retval = target_write_buffer(target, FCTL_REG_CMDDATA0, num_bytes_to_write, buffer);
		if (retval != ERROR_OK)
			return retval;

		addr += num_bytes_to_write;
		buffer += num_bytes_to_write;
		count -= num_bytes_to_write;

		retval = target_write_u32(target, FCTL_REG_CMDEXEC, FCTL_CMDEXEC_VAL_EXECUTE);
		if (retval != ERROR_OK)
			return retval;

		retval = mspm0_fctl_wait_cmd_ok(bank);
		if (retval != ERROR_OK)
			return retval;
	}

	/*
	 * TRM Says:
	 * Note that the CMDWEPROTx registers are reset to a protected state
	 * at the end of all program and erase operations.  These registers
	 * must be re-configured by software before a new operation is
	 * initiated
	 * Let us just Dump the protection registers back to the system.
	 * That way we retain the protection status as requested by the user
	 */
	for (unsigned int i = 0; i < mspm0_info->protect_reg_count; i++) {
		retval = target_write_u32(target,
			mspm0_info->protect_reg_base + (i * 4),
			protect_reg_cache[i]);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed re-applying protection status of flashctl");
			return retval;
		}
	}

	return ERROR_OK;
}

static int mspm0_probe(struct flash_bank *bank)
{
	struct mspm0_flash_bank *mspm0_info = bank->driver_priv;

	/*
	 * If this is a mspm0 chip, it has flash; probe() is just
	 * to figure out how much is present.  Only do it once.
	 */
	if (mspm0_info->did != 0)
		return ERROR_OK;

	/*
	 * mspm0_read_part_info() already handled error checking and
	 * reporting.  Note that it doesn't write, so we don't care about
	 * whether the target is halted or not.
	 */
	int retval = mspm0_read_part_info(bank);
	if (retval != ERROR_OK)
		return retval;

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	bank->write_start_alignment = 4;
	bank->write_end_alignment = 4;

	switch (bank->base) {
	case MSPM0_FLASH_BASE_NONMAIN:
		bank->size = 1024;
		bank->num_sectors = 0x1;
		mspm0_info->protect_reg_base = FCTL_REG_CMDWEPROTNM;
		mspm0_info->protect_reg_count = 1;
		break;
	case MSPM0_FLASH_BASE_MAIN:
		bank->size = (mspm0_info->main_flash_size_kb * 1024);
		bank->num_sectors = bank->size / mspm0_info->sector_size;
		/*
		 * If the feature version bit read from the FCTL_REG_DESC is
		 * greater than or equal to 0xA then it means that the device
		 * will exclusively use CMDWEPROTB ONLY for MAIN memory protection
		 */
		if (mspm0_info->flash_version >= FCTL_FEATURE_VER_B) {
			mspm0_info->protect_reg_base = FCTL_REG_CMDWEPROTB;
			mspm0_info->protect_reg_count = 1;
		} else {
			mspm0_info->protect_reg_base = FCTL_REG_CMDWEPROTA;
			mspm0_info->protect_reg_count = 3;
		}
		break;
	case MSPM0_FLASH_BASE_DATA:
		if (!mspm0_info->data_flash_size_kb) {
			LOG_INFO("Data region NOT available!");
			bank->size = 0x0;
			bank->num_sectors = 0x0;
			return ERROR_OK;
		}
		/*
		 * Any MSPM0 device containing data bank will have a flashctl
		 * feature version of 0xA or higher. Since data bank is treated
		 * like MAIN memory, it will also exclusively use CMDWEPROTB for
		 * protection.
		 */
		bank->size = (mspm0_info->data_flash_size_kb * 1024);
		bank->num_sectors = bank->size / mspm0_info->sector_size;
		mspm0_info->protect_reg_base = FCTL_REG_CMDWEPROTB;
		mspm0_info->protect_reg_count = 1;
		break;
	default:
		LOG_ERROR("Invalid bank address " TARGET_ADDR_FMT,
			bank->base);
		return ERROR_FAIL;
	}

	bank->sectors = calloc(bank->num_sectors, sizeof(struct flash_sector));
	if (!bank->sectors) {
		LOG_ERROR("Out of memory for sectors!");
		return ERROR_FAIL;
	}
	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].offset = i * mspm0_info->sector_size;
		bank->sectors[i].size = mspm0_info->sector_size;
		bank->sectors[i].is_erased = -1;
	}

	return ERROR_OK;
}

const struct flash_driver mspm0_flash = {
	.name = "mspm0",
	.flash_bank_command = mspm0_flash_bank_command,
	.erase = mspm0_erase,
	.protect = NULL,
	.write = mspm0_write,
	.read = default_flash_read,
	.probe = mspm0_probe,
	.auto_probe = mspm0_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = mspm0_protect_check,
	.info = get_mspm0_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
