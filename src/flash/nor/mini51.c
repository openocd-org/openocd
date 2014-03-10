/***************************************************************************
 *   Copyright (C) 2013 Cosmin Gorgovan                                    *
 *   cosmin [at] linux-geek [dot] org                                      *
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

/*
	Flash driver for the Nuvoton NuMicro Mini51 series microcontrollers

	Part		 |APROM Size |Part ID (at 0x5000_0000)
	----------------------------------------------
	MINI51LAN	 4 KB			0x00205100
	MINI51ZAN	 4 KB			0x00205103
	MINI51TAN	 4 KB			0x00205104
	MINI52LAN	 8 KB			0x00205200
	MINI52ZAN	 8 KB			0x00205203
	MINI52TAN	 8 KB			0x00205204
	MINI54LAN	 16 KB		 0x00205400
	MINI54ZAN	 16 KB		 0x00205403
	MINI54TAN	 16 KB		 0x00205404

	Datasheet & TRM
	---------------

	The ISP flash programming procedure is described on pages 130 and 131 of the (not very verbose) TRM.

	http://www.keil.com/dd/docs/datashts/nuvoton/mini51/da00-mini51_52_54c1.pdf

	This driver
	-----------

	* Only erase and write operations have been implemented;
	* Both operations only support the APROM, not the LDROM;
	* The TRM suggests that after the boot source has been selected, a software reset should be performed by
	  setting bit SWRST in ISPCON. However, this doesn't seem to have any effect on the MCU I'm using. At the
	  moment, the ARM core is reset using the IPRSTC1 register, which seems to do the trick.

	Flash access limitations
	------------------------

	APROM can only be modified when the MCU has booted off the LDROM. For write and erase operations, the
	microcontroller will probably need to be rebooted. Pseudocode:

	* If operation is write or erase, check bit BS (1) in ISPCON (0x5000_C000);
	* If BS is 0 (APROM):
		* unlock protected registers by writing 0x59, 0x16, 0x88 to RegLockAddr(0x5000_0100);
		* set BS to 1 (LDROM);
		* reboot by setting bit CPU_RST(1) in IPRSTC1 (0x50000008);
		* poll CPU_RST until it is reset (not sure it's necessary);
		* <Perform flash operation>
		* reboot from APROM using the same procedure but writing 0 to BS


	For implementing the read operation, please note that the APROM isn't memory mapped when booted from LDROM.
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"

#define PART_ID_REG     0x50000000
#define IPRSTC1         0x50000008
#define REGLOCKADDR     0x50000100
#define ISPCON          0x5000C000
#define ISPADR          0x5000C004
#define ISPDAT          0x5000C008
#define ISPCMD          0x5000C00C
#define ISPTRG          0x5000C010

#define PART_ID_MAIN_MASK   0xFFFFFFF8
#define IPRSTC_CPU_RST      0x02
#define ISPCON_BS_LDROM     0x02
#define ISPCON_ISPEN        0x01
#define ISPCON_SWRST        0x80
#define ISPCON_ISPFF        0x40
#define ISPCMD_PROGRAM      0x21
#define ISPCMD_ERASE        0x22
#define ISPTRG_ISPGO        0x01

#define MINI51 0x00205100
#define MINI52 0x00205200
#define MINI54 0x00205400

#define MINI51_APROM_BASE  0x00000000
#define MINI51_KB          1024
#define MINI51_PAGE_SIZE   512
#define MINI51_TIMEOUT     1000

struct mini51_flash_bank {
	bool probed;
};

enum mini51_boot_source {
	APROM = 0,
	LDROM = 1
};

/* Private methods */

static int mini51_unlock_reg(struct flash_bank *bank)
{
	int status;
	struct target *target = bank->target;

	status = target_write_u32(target, REGLOCKADDR, 0x59);
	if (status != ERROR_OK)
		return status;
	status = target_write_u32(target, REGLOCKADDR, 0x16);
	if (status != ERROR_OK)
		return status;
	status = target_write_u32(target, REGLOCKADDR, 0x88);
	if (status != ERROR_OK)
		return status;

	return ERROR_OK;
}

static int mini51_reboot_with_source(struct flash_bank *bank,
				enum mini51_boot_source new_source,
				enum mini51_boot_source *prev_source)
{
	uint32_t ispcon;
	uint32_t isprtc1;
	bool mini51_reboot = false;
	int status;
	int timeout = MINI51_TIMEOUT;

	/* Read current boot source */
	struct target *target = bank->target;
	status = target_read_u32(target, ISPCON, &ispcon);
	if (status != ERROR_OK)
		return status;

	*prev_source = (ispcon >> 1) & 1;

	if ((new_source == APROM) && (*prev_source != APROM)) {
		ispcon &= ~ISPCON_BS_LDROM;
		mini51_reboot = true;
	} else if ((new_source == LDROM) && (*prev_source != LDROM)) {
		ispcon |= ISPCON_BS_LDROM;
		mini51_reboot = true;
	}

	if (mini51_reboot) {
		mini51_unlock_reg(bank);
		status = target_write_u32(target, ISPCON, ispcon);
		if (status != ERROR_OK)
			return status;

		status = target_write_u32(target, IPRSTC1, IPRSTC_CPU_RST);
		if (status != ERROR_OK)
			return status;

		do {
			target_read_u32(target, IPRSTC1, &isprtc1);
			timeout--;
		} while ((isprtc1 & IPRSTC_CPU_RST) && timeout > 0);

		if (timeout == 0) {
			LOG_WARNING("Mini51 flash driver: timeout attempting to reboot\n");
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}

	return ERROR_OK;
}

static int mini51_get_part_id(struct flash_bank *bank, uint32_t *part_id)
{
	return target_read_u32(bank->target, PART_ID_REG, part_id);
}

static int mini51_get_flash_size(struct flash_bank *bank, uint32_t *flash_size)
{
	uint32_t part_id;
	int status;

	status = mini51_get_part_id(bank, &part_id);
	if (status != ERROR_OK)
		return status;

	switch (part_id & PART_ID_MAIN_MASK) {
		case MINI51:
			*flash_size = 4 * MINI51_KB;
			break;
		case MINI52:
			*flash_size = 8 * MINI51_KB;
			break;
		case MINI54:
			*flash_size = 16 * MINI51_KB;
			break;
		default:
			*flash_size = 0;
			break;
	}

	return ERROR_OK;
}

/* Public (API) methods */

FLASH_BANK_COMMAND_HANDLER(mini51_flash_bank_command)
{
	struct mini51_flash_bank *mini51_info;
	mini51_info = malloc(sizeof(struct mini51_flash_bank));
	mini51_info->probed = false;
	bank->driver_priv = mini51_info;

	return ERROR_OK;
}

static int mini51_protect_check(struct flash_bank *bank)
{
	LOG_WARNING("Mini51 flash driver: protect_check not implemented yet\n");

	return ERROR_FLASH_OPERATION_FAILED;
}

static int mini51_erase(struct flash_bank *bank, int first, int last)
{
	int status;
	int timeout;
	uint32_t ispcon;
	uint32_t isptrg;
	enum mini51_boot_source new_source;
	enum mini51_boot_source prev_source;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* TODO: add support for erasing the LDROM */
	new_source = LDROM;
	status = mini51_reboot_with_source(bank, new_source, &prev_source);
	if (status != ERROR_OK)
		return status;

	/* Enable ISP */
	status = target_read_u32(target, ISPCON, &ispcon);
	if (status != ERROR_OK)
		return status;
	ispcon |= ISPCON_ISPEN;
	status = target_write_u32(target, ISPCON, ispcon);

	for (int page_start = first; page_start <= last; page_start++) {
		/* Set up erase command */
		status = target_write_u32(target, ISPADR, page_start*MINI51_PAGE_SIZE);
		if (status != ERROR_OK)
			return status;
		status = target_write_u32(target, ISPCMD, ISPCMD_ERASE);
		if (status != ERROR_OK)
			return status;

		/* Erase the selected page */
		status = target_write_u32(target, ISPTRG, ISPTRG_ISPGO);
		if (status != ERROR_OK)
			return status;

		/* Wait for for command to finish executing */
		timeout = MINI51_TIMEOUT;
		do {
			target_read_u32(target, ISPTRG, &isptrg);
			timeout--;
		} while ((isptrg & ISPTRG_ISPGO) && (timeout > 0));
		if (timeout == 0) {
			LOG_WARNING("Mini51 flash driver: Timeout erasing flash\n");
			return ERROR_FLASH_OPERATION_FAILED;
		}

		/* Check for errors */
		status = target_read_u32(target, ISPCON, &ispcon);
		if (status != ERROR_OK)
			return status;
		if (ispcon & ISPCON_ISPFF) {
			LOG_WARNING("Mini51 flash driver: Erase operation failed\n");
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}

	/* Reboot from previous source */
	if (prev_source != new_source) {
		status = mini51_reboot_with_source(bank, prev_source, &new_source);
		if (status != ERROR_OK)
			return status;
	}

	return ERROR_OK;
}

static int mini51_protect(struct flash_bank *bank, int set, int first, int last)
{
	LOG_WARNING("Mini51 flash driver: protect operation not implemented yet\n");

	return ERROR_FLASH_OPERATION_FAILED;
}

static int mini51_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	int status;
	int timeout;
	uint32_t ispcon;
	uint32_t isptrg;
	uint32_t ispdat;
	enum mini51_boot_source new_source;
	enum mini51_boot_source prev_source;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if ((offset & 0x3) || (count & 0x3)) {
		LOG_WARNING("Mini51 flash driver: unaligned access not supported\n");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* TODO: add support for writing to LDROM */
	new_source = LDROM;
	status = mini51_reboot_with_source(bank, new_source, &prev_source);
	if (status != ERROR_OK)
		return status;

	/* Enable ISP */
	status = target_read_u32(target, ISPCON, &ispcon);
	if (status != ERROR_OK)
		return status;
	ispcon |= ISPCON_ISPEN;
	status = target_write_u32(target, ISPCON, ispcon);

	for (uint32_t i = offset; i < offset + count; i += 4) {
		/* Set up program command */
		status = target_write_u32(target, ISPADR, i);
		if (status != ERROR_OK)
			return status;
		status = target_write_u32(target, ISPCMD, ISPCMD_PROGRAM);
		if (status != ERROR_OK)
			return status;
		memcpy(&ispdat, buffer, sizeof(ispdat));
		buffer += sizeof(ispdat);
		status = target_write_u32(target, ISPDAT, ispdat);
		if (status != ERROR_OK)
			return status;

		/* Write the selected word */
		status = target_write_u32(target, ISPTRG, ISPTRG_ISPGO);
		if (status != ERROR_OK)
			return status;

		/* Wait for for command to finish executing */
		timeout = MINI51_TIMEOUT;
		do {
			target_read_u32(target, ISPTRG, &isptrg);
			timeout--;
		} while ((isptrg & ISPTRG_ISPGO) && (timeout > 0));
		if (timeout == 0) {
			LOG_WARNING("Mini51 flash driver: Timeout programming flash\n");
			return ERROR_FLASH_OPERATION_FAILED;
		}

		/* Check for errors */
		status = target_read_u32(target, ISPCON, &ispcon);
		if (status != ERROR_OK)
			return status;
		if (ispcon & ISPCON_ISPFF) {
			LOG_WARNING("Mini51 flash driver: Programming operation failed\n");
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}

	if (prev_source != new_source) {
		status = mini51_reboot_with_source(bank, prev_source, &new_source);
		if (status != ERROR_OK)
			return status;
	}

	return ERROR_OK;
}

static int mini51_probe(struct flash_bank *bank)
{
	uint32_t flash_size;
	int retval;
	int num_pages;
	uint32_t offset = 0;

	retval = mini51_get_flash_size(bank, &flash_size);
	if (retval != ERROR_OK || flash_size == 0) {
		LOG_WARNING("Mini51 flash driver: Failed to detect a known part\n");
		return ERROR_FLASH_OPERATION_FAILED;
	}

	num_pages = flash_size / MINI51_PAGE_SIZE;

	bank->base = MINI51_APROM_BASE;
	bank->num_sectors = num_pages;
	bank->sectors = malloc(sizeof(struct flash_sector) * num_pages);
	bank->size = flash_size;

	for (int i = 0; i < num_pages; i++) {
		bank->sectors[i].offset = offset;
		bank->sectors[i].size = MINI51_PAGE_SIZE;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
		offset += MINI51_PAGE_SIZE;
	}

	struct mini51_flash_bank *mini51_info = bank->driver_priv;
	mini51_info->probed = true;

	return ERROR_OK;
}

static int mini51_auto_probe(struct flash_bank *bank)
{
	struct mini51_flash_bank *mini51_info = bank->driver_priv;
	if (mini51_info->probed)
		return ERROR_OK;
	return mini51_probe(bank);
}

struct flash_driver mini51_flash = {
	.name = "mini51",
	.flash_bank_command = mini51_flash_bank_command,
	.erase = mini51_erase,
	.protect = mini51_protect,
	.write = mini51_write,
	.read = default_flash_read,
	.probe = mini51_probe,
	.auto_probe = mini51_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = mini51_protect_check,
};
