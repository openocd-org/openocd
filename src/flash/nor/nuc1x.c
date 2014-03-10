/***************************************************************************
 *   Copyright (C) 2011 by James K. Larson                                 *
 *   jlarson@pacifier.com                                                  *
 *                                                                         *
 *   Copyright (C) 2013 Nemui Trinomius                                    *
 *   nemuisan_kawausogasuki@live.jp                                        *
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
#include <config.h>
#endif

#include "imp.h"

/* nuc1x register locations */
#define NUC1X_SYS_BASE        0x50000000
#define NUC1X_SYS_WRPROT      0x50000100
#define NUC1X_SYS_IPRSTC1     0x50000008

#define NUC1X_SYSCLK_BASE     0x50000200
#define NUC1X_SYSCLK_PWRCON   0x50000200
#define NUC1X_SYSCLK_CLKSEL0  0x50000210
#define NUC1X_SYSCLK_CLKDIV   0x50000218
#define NUC1X_SYSCLK_AHBCLK   0x50000204

#define NUC1X_FLASH_BASE      0x5000C000
#define NUC1X_FLASH_ISPCON    0x5000C000
#define NUC1X_FLASH_ISPCMD    0x5000C00C
#define NUC1X_FLASH_ISPADR    0x5000C004
#define NUC1X_FLASH_ISPDAT    0x5000C008
#define NUC1X_FLASH_ISPTRG    0x5000C010

/* Command register bits */
#define PWRCON_OSC22M         (1 << 2)
#define PWRCON_XTL12M         (1 << 0)

#define IPRSTC1_CPU_RST       (1<<1)
#define IPRSTC1_CHIP_RST      (1<<0)

#define AHBCLK_ISP_EN         (1 << 2)

#define ISPCON_ISPEN          (1 << 0)
#define ISPCON_BS_AP          (0 << 1)
#define ISPCON_BS_LP          (1 << 1)
#define ISPCON_CFGUEN         (1 << 4)
#define ISPCON_LDUEN          (1 << 5)
#define ISPCON_ISPFF          (1 << 6)

/* isp commands */
#define ISPCMD_FCTRL          (0x2)
#define ISPCMD_FCEN           (1 << 4)
#define ISPCMD_FOEN           (1 << 5)
#define ISPCMD_ERASE          (0x2 | ISPCMD_FOEN)
#define ISPCMD_WRITE          (0x1 | ISPCMD_FOEN)
#define ISPTRG_ISPGO          (1 << 0)

/* access unlock keys */
#define KEY1 0x59
#define KEY2 0x16
#define KEY3 0x88
#define LOCK 0x00

/* part structs */
static const struct {
	const char *partname;
	uint32_t partno;
	uint16_t num_page;
}
NuMicroParts[] = {
	/*PART NO*/   /*PART ID*/   /*NUM PAGE*/
	{"NUC100LC1", 0x00010008,   64},
	{"NUC100LD1", 0x00010005,   128},
	{"NUC100LD2", 0x00010004,   128},
	{"NUC100RC1", 0x00010017,   64},
	{"NUC100RD1", 0x00010014,   128},
	{"NUC100RD2", 0x00010013,   128},

	{"NUC100LD3", 0x00010003,   128},
	{"NUC100LE3", 0x00010000,   256},
	{"NUC100RD3", 0x00010012,   128},
	{"NUC100RE3", 0x00010009,   256},
	{"NUC100VD2", 0x00010022,   128},
	{"NUC100VD3", 0x00010021,   128},
	{"NUC100VE3", 0x00010018,   256},

	{"NUC120LC1", 0x00012008,   64},
	{"NUC120LD1", 0x00012005,   128},
	{"NUC120LD2", 0x00012004,   128},
	{"NUC120RC1", 0x00012017,   64},
	{"NUC120RD1", 0x00012014,   128},
	{"NUC120RD2", 0x00012013,   128},

	{"NUC120LD3", 0x00012003,   128},
	{"NUC120LE3", 0x00012000,   256},
	{"NUC120RD3", 0x00012012,   128},
	{"NUC120RE3", 0x00012009,   256},
	{"NUC120VD2", 0x00012022,   128},
	{"NUC120VD3", 0x00012021,   128},
	{"NUC120VE3", 0x00012018,   256},

	{"NUC122ZD2", 0x00012231,   128},
	{"NUC122ZC1", 0x00012235,   64},
	{"NUC122LD2", 0x00012204,   128},
	{"NUC122LC1", 0x00012208,   64},
	{"NUC122RD2", 0x00012213,   128},
	{"NUC122RC1", 0x00012217,   64},

	{"NUC123ZD4", 0x00012255,   136},
	{"NUC123ZC2", 0x00012245,   68},
	{"NUC123LD4", 0x00012235,   136},
	{"NUC123LC2", 0x00012225,   68},
	{"NUC123SD4", 0x00012215,   136},
	{"NUC123SC2", 0x00012205,   68},

	{"NUC130LC1", 0x00013008,   64},
	{"NUC130LD2", 0x00013004,   128},
	{"NUC130LE3", 0x00013000,   256},
	{"NUC130RC1", 0x00013017,   64},
	{"NUC130RD2", 0x00013013,   128},
	{"NUC130RE3", 0x00013009,   256},
	{"NUC130VE3", 0x00013018,   256},

	{"M052L",     0x00005200,   16},
	{"M052Z",     0x00005203,   16},
	{"M054L",     0x00005400,   32},
	{"M054Z",     0x00005403,   32},
	{"M058L",     0x00005800,   64},
	{"M058Z",     0x00005803,   64},
	{"M0516L",    0x00005A00,   128},
	{"M0516Z",    0x00005A03,   128},

	{"MINI51L",   0x00205100,   8},
	{"MINI51Z",   0x00205103,   8},
	{"MINI52L",   0x00205200,   16},
	{"MINI52Z",   0x00205203,   16},
	{"MINI54L",   0x00205400,   32},
	{"MINI54Z",   0x00205403,   32},

	{"UNKNOWN",   0x00000000,   256},
};

static int nuc1x_unlock(struct flash_bank *bank)
{
	uint32_t is_protected;
	struct target *target = bank->target;

	/* Check to see if Nuc is unlocked or not */
	int retval = target_read_u32(target, NUC1X_SYS_WRPROT, &is_protected);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("protected = 0x%08" PRIx32 "", is_protected);
	if (is_protected == 0) {	/* means protected - so unlock it */
		/* unlock flash registers */
		retval = target_write_u32(target, NUC1X_SYS_WRPROT, KEY1);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target, NUC1X_SYS_WRPROT, KEY2);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target, NUC1X_SYS_WRPROT, KEY3);
		if (retval != ERROR_OK)
			return retval;
	}
	/* Check that unlock worked */
	retval = target_read_u32(target, NUC1X_SYS_WRPROT, &is_protected);
	if (retval != ERROR_OK)
		return retval;

	if (is_protected == 1) {	/* means unprotected */
		LOG_DEBUG("protection removed");
	} else {
		LOG_DEBUG("still protected!!");
	}

	return ERROR_OK;
}

static int nuc1x_reset(struct flash_bank *bank)
{
	struct target *target = bank->target;

	nuc1x_unlock(bank);

	int retval = target_write_u32(target, NUC1X_SYS_IPRSTC1, IPRSTC1_CPU_RST);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int nuc1x_reset2lprom(struct flash_bank *bank)
{
	struct target *target = bank->target;

	nuc1x_unlock(bank);
	int retval = target_write_u32(target, NUC1X_FLASH_ISPCON, ISPCON_BS_LP);
	if (retval != ERROR_OK)
		return retval;

	nuc1x_reset(bank);

	return ERROR_OK;
}

static int nuc1x_init_iap(struct flash_bank *bank)
{
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval = nuc1x_unlock(bank);
	if (retval != ERROR_OK)
		return retval;

	/* enable isp clock and ispen bit */
	retval = target_write_u32(target, NUC1X_SYSCLK_AHBCLK, AHBCLK_ISP_EN);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, NUC1X_FLASH_ISPCON, ISPCON_ISPFF | ISPCON_LDUEN | ISPCON_CFGUEN | ISPCON_ISPEN);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

/* Private bank information for nuc1x. */
struct  nuc1x_flash_bank {
	struct working_area *write_algorithm;
	int probed;
};

/* This is the function called in the config file. */
FLASH_BANK_COMMAND_HANDLER(nuc1x_flash_bank_command)
{
	struct nuc1x_flash_bank *bank_info;

	if (CMD_ARGC < 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	LOG_INFO("add flash_bank nuc1x %s", bank->name);

	bank_info = malloc(sizeof(struct nuc1x_flash_bank));

	memset(bank_info, 0, sizeof(struct nuc1x_flash_bank));

	bank->driver_priv = bank_info;

	return ERROR_OK;

}

/* Protection checking - examines the lock bit. */
static int nuc1x_protect_check(struct flash_bank *bank)
{
	uint32_t is_protected, set;
	struct target *target = bank->target;
	int i;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Check to see if Nuc is unlocked or not */
	int retval = target_read_u32(target, NUC1X_SYS_WRPROT, &is_protected);
	if (retval != ERROR_OK)
		return retval;

	LOG_INFO("is_protected = 0x%08" PRIx32 "", is_protected);
	if (is_protected == 0) {	/* means protected */
		set = 1;
	} else {
		set = 0;
	}
	for (i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = set;

	return ERROR_OK;
}

static int nuc1x_erase(struct flash_bank *bank, int first, int last)
{
	struct target *target = bank->target;
	uint32_t timeout, status;
	int i;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_INFO("Nuvoton NUC: Sector Erase ... (%d to %d)", first, last);

	int retval = nuc1x_reset2lprom(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = nuc1x_init_iap(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = nuc1x_unlock(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, NUC1X_FLASH_ISPCMD, ISPCMD_ERASE);
	if (retval != ERROR_OK)
		return retval;

	for (i = first; i <= last; i++) {
		LOG_DEBUG("erasing sector %d at addresss 0x%" PRIx32 "", i, bank->base + bank->sectors[i].offset);
		retval = target_write_u32(target, NUC1X_FLASH_ISPADR, bank->base + bank->sectors[i].offset);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target, NUC1X_FLASH_ISPTRG, ISPTRG_ISPGO); /* This is the only bit available */
		if (retval != ERROR_OK)
			return retval;

		/* wait for busy to clear - check the GO flag */
		timeout = 100;
		for (;;) {
			retval = target_read_u32(target, NUC1X_FLASH_ISPTRG, &status);
			if (retval != ERROR_OK)
				return retval;
				LOG_DEBUG("status: 0x%" PRIx32 "", status);
			if (status == 0)
				break;
			if (timeout-- <= 0) {
				LOG_DEBUG("timed out waiting for flash");
				return ERROR_FAIL;
			}
			busy_sleep(1);	/* can use busy sleep for short times. */
		}

		/* check for failure */
		retval = target_read_u32(target, NUC1X_FLASH_ISPCON, &status);
		if (retval != ERROR_OK)
			return retval;
		if ((status & ISPCON_ISPFF) != 0) {
			LOG_DEBUG("failure: 0x%" PRIx32 "", status);
			/* if bit is set, then must write to it to clear it. */
			retval = target_write_u32(target, NUC1X_FLASH_ISPCON, ISPCON_ISPFF);
			if (retval != ERROR_OK)
				return retval;
		} else {
			bank->sectors[i].is_erased = 1;
		}
	}

	retval = nuc1x_reset(bank);
	if (retval != ERROR_OK)
		return retval;

	/* done, */
	LOG_DEBUG("Erase done.");

	return ERROR_OK;
}

/* The write routine stub. */
static int nuc1x_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;
	uint32_t i, timeout, status;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_INFO("Novoton NUC: FLASH Write ...");

	int retval = nuc1x_reset2lprom(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = nuc1x_init_iap(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = nuc1x_unlock(bank);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_u32(target, NUC1X_FLASH_ISPCMD, ISPCMD_WRITE);
	if (retval != ERROR_OK)
		return retval;

	/* program command */
	for (i = 0; i < count; i += 4) {

		LOG_DEBUG("write longword @ %08" PRIX32, (uint32_t)(offset + i));

		uint8_t padding[4] = {0xff, 0xff, 0xff, 0xff};
		memcpy(padding, buffer + i, MIN(4, count-i));

		retval = target_write_u32(target, NUC1X_FLASH_ISPADR, bank->base + offset + i);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_memory(target, NUC1X_FLASH_ISPDAT, 4, 1, padding);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target, NUC1X_FLASH_ISPTRG, ISPTRG_ISPGO);
		if (retval != ERROR_OK)
			return retval;

		/* wait for busy to clear - check the GO flag */
		timeout = 100;
		for (;;) {
			retval = target_read_u32(target, NUC1X_FLASH_ISPTRG, &status);
			if (retval != ERROR_OK)
				return retval;
				LOG_DEBUG("status: 0x%" PRIx32 "", status);
			if (status == 0)
				break;
			if (timeout-- <= 0) {
				LOG_DEBUG("timed out waiting for flash");
				return ERROR_FAIL;
			}
			busy_sleep(1);	/* can use busy sleep for short times. */
		}

		/* check for failure */
		retval = target_read_u32(target, NUC1X_FLASH_ISPCON, &status);
		if (retval != ERROR_OK)
			return retval;
		if ((status & ISPCON_ISPFF) != 0) {
			LOG_DEBUG("failure: 0x%" PRIx32 "", status);
			/* if bit is set, then must write to it to clear it. */
			retval = target_write_u32(target, NUC1X_FLASH_ISPCON, ISPCON_ISPFF);
			if (retval != ERROR_OK)
				return retval;
		} else {
			LOG_DEBUG("writed OK");
		}
	}

	retval = nuc1x_reset(bank);
	if (retval != ERROR_OK)
		return retval;

	/* done, */
	LOG_DEBUG("Write done.");

	return ERROR_OK;
}

/* The probe routine for the nuc. Only recognizes the nuc120 right now. */
static int nuc1x_probe(struct flash_bank *bank)
{
	struct target *target = bank->target;
	struct nuc1x_flash_bank *nuc1x_info = bank->driver_priv;
	int i;
	uint16_t num_pages;
	uint32_t device_id;
	int page_size;
	uint32_t base_address = 0x00000000;

	nuc1x_info->probed = 0;

	/* read nuc1x device id register */
	int retval = target_read_u32(target, 0x50000000, &device_id);
	if (retval != ERROR_OK)
		return retval;

	page_size = 512;	/* all nuc parts has 512 byte per sector */

	/* search part numbers */
	for (i = 0; NuMicroParts[i].partno; i++) {
		if (NuMicroParts[i].partno == (device_id & 0x0FFFFFFF)) {
			num_pages = NuMicroParts[i].num_page;
			break;
		}
	}
	if (!(NuMicroParts[i].partno == 0x00000000)) {
		LOG_INFO("DeviceID : 0x%08" PRIx32 "", device_id);
		LOG_INFO("Detect %s%cN!", NuMicroParts[i].partname, (char)('A'+(device_id>>28)));
	} else {
		LOG_INFO("No NUC Device Detected...");
		return ERROR_FAIL;
	}

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	bank->base = base_address;
	bank->size = (num_pages * page_size);
	bank->num_sectors = num_pages;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);

	for (i = 0; i < num_pages; i++) {
		bank->sectors[i].offset = i * page_size;
		bank->sectors[i].size = page_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	nuc1x_info->probed = 1;

	LOG_DEBUG("Novoton NUC: Probed ...");

	return ERROR_OK;
}

/* Standard approach to autoprobing. */
static int nuc1x_auto_probe(struct flash_bank *bank)
{
	struct nuc1x_flash_bank *nuc1x_info = bank->driver_priv;
	if (nuc1x_info->probed)
		return ERROR_OK;
	return nuc1x_probe(bank);
}

/* Info doesn't really add much, but works correctly. */
static int get_nuc1x_info(struct flash_bank *bank, char *buf, int buf_size)
{
	struct target *target = bank->target;
	uint32_t i, device_id;

	/* read nuc1x device id register */
	int retval = target_read_u32(target, 0x50000000, &device_id);
	if (retval != ERROR_OK)
		return retval;

	/* search part numbers */
	for (i = 0; NuMicroParts[i].partno; i++) {
		if (NuMicroParts[i].partno == (device_id & 0x0FFFFFFF))
			break;
	}
	if (!(NuMicroParts[i].partno == 0x00000000)) {
		LOG_INFO("DeviceID : 0x%08" PRIx32 "", device_id);
		LOG_INFO("Detect %s%cN!", NuMicroParts[i].partname, (char)('A'+(device_id>>28)));
	} else {
		LOG_INFO("No NUC Device Detected...");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/* The nuc120 doesn't support mass erase, so this will probably be removed soon.
 * The structure is left for now until I am sure I don't want to add any custom
 * commands. */
static int nuc1x_mass_erase(struct flash_bank *bank)
{
	struct target *target = bank->target;
	int retval = ERROR_OK;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_INFO("Novoton NUC: Chip Erase ... (may take several seconds)");

	return retval;
}

COMMAND_HANDLER(nuc1x_handle_mass_erase_command)
{
	int i; /* for erasing sectors */
	if (CMD_ARGC < 1) {
		command_print(CMD_CTX, "nuc1x mass_erase <bank>");
		return ERROR_OK;
	}

	struct flash_bank *bank;
	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (ERROR_OK != retval)
		return retval;

	retval = nuc1x_mass_erase(bank);
	if (retval == ERROR_OK) {
		/* set all sectors as erased */
		for (i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = 1;

		command_print(CMD_CTX, "nuc1x mass erase complete");
	} else
		command_print(CMD_CTX, "nuc1x mass erase failed");

	return retval;
}

static const struct command_registration nuc1x_exec_command_handlers[] = {
	{
		.name = "mass_erase",
		.handler = nuc1x_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Erase entire Flash device.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration nuc1x_command_handlers[] = {
	{
		.name = "nuc1x",
		.mode = COMMAND_ANY,
		.help = "nuc1x Flash command group",
		.chain = nuc1x_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};
struct flash_driver nuc1x_flash = {
	.name = "nuc1x",
	.commands = nuc1x_command_handlers,
	.flash_bank_command = nuc1x_flash_bank_command,
	.erase = nuc1x_erase,
	.write = nuc1x_write,
	.read = default_flash_read,
	.probe = nuc1x_probe,
	.auto_probe = nuc1x_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = nuc1x_protect_check,
	.info = get_nuc1x_info,
};
