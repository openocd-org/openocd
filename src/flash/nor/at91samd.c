/***************************************************************************
 *   Copyright (C) 2013 by Andrey Yurovsky                                 *
 *   Andrey Yurovsky <yurovsky@gmail.com>                                  *
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
#include "helper/binarybuffer.h"

#define SAMD_NUM_SECTORS	16
#define SAMD_PAGE_SIZE_MAX	1024

#define SAMD_FLASH			((uint32_t)0x00000000)	/* physical Flash memory */
#define SAMD_USER_ROW		((uint32_t)0x00804000)	/* User Row of Flash */
#define SAMD_PAC1			0x41000000	/* Peripheral Access Control 1 */
#define SAMD_DSU			0x41002000	/* Device Service Unit */
#define SAMD_NVMCTRL		0x41004000	/* Non-volatile memory controller */

#define SAMD_DSU_DID		0x18		/* Device ID register */

#define SAMD_NVMCTRL_CTRLA		0x00	/* NVM control A register */
#define SAMD_NVMCTRL_CTRLB		0x04	/* NVM control B register */
#define SAMD_NVMCTRL_PARAM		0x08	/* NVM parameters register */
#define SAMD_NVMCTRL_INTFLAG	0x18	/* NVM Interupt Flag Status & Clear */
#define SAMD_NVMCTRL_STATUS		0x18	/* NVM status register */
#define SAMD_NVMCTRL_ADDR		0x1C	/* NVM address register */
#define SAMD_NVMCTRL_LOCK		0x20	/* NVM Lock section register */

#define SAMD_CMDEX_KEY		0xA5UL
#define SAMD_NVM_CMD(n)		((SAMD_CMDEX_KEY << 8) | (n & 0x7F))

/* NVMCTRL commands.  See Table 20-4 in 42129F–SAM–10/2013 */
#define SAMD_NVM_CMD_ER		0x02		/* Erase Row */
#define SAMD_NVM_CMD_WP		0x04		/* Write Page */
#define SAMD_NVM_CMD_EAR	0x05		/* Erase Auxilary Row */
#define SAMD_NVM_CMD_WAP	0x06		/* Write Auxilary Page */
#define SAMD_NVM_CMD_LR		0x40		/* Lock Region */
#define SAMD_NVM_CMD_UR		0x41		/* Unlock Region */
#define SAMD_NVM_CMD_SPRM	0x42		/* Set Power Reduction Mode */
#define SAMD_NVM_CMD_CPRM	0x43		/* Clear Power Reduction Mode */
#define SAMD_NVM_CMD_PBC	0x44		/* Page Buffer Clear */
#define SAMD_NVM_CMD_SSB	0x45		/* Set Security Bit */
#define SAMD_NVM_CMD_INVALL	0x46		/* Invalidate all caches */

/* Known identifiers */
#define SAMD_PROCESSOR_M0	0x01
#define SAMD_FAMILY_D		0x00
#define SAMD_FAMILY_L		0x01
#define SAMD_SERIES_20		0x00
#define SAMD_SERIES_21		0x01
#define SAMD_SERIES_10		0x02
#define SAMD_SERIES_11		0x03

struct samd_part {
	uint8_t id;
	const char *name;
	uint32_t flash_kb;
	uint32_t ram_kb;
};

/* Known SAMD10 parts */
static const struct samd_part samd10_parts[] = {
	{ 0x0, "SAMD10D14AMU", 16, 4 },
	{ 0x1, "SAMD10D13AMU", 8, 4 },
	{ 0x2, "SAMD10D12AMU", 4, 4 },
	{ 0x3, "SAMD10D14ASU", 16, 4 },
	{ 0x4, "SAMD10D13ASU", 8, 4 },
	{ 0x5, "SAMD10D12ASU", 4, 4 },
	{ 0x6, "SAMD10C14A", 16, 4 },
	{ 0x7, "SAMD10C13A", 8, 4 },
	{ 0x8, "SAMD10C12A", 4, 4 },
};

/* Known SAMD11 parts */
static const struct samd_part samd11_parts[] = {
	{ 0x0, "SAMD11D14AMU", 16, 4 },
	{ 0x1, "SAMD11D13AMU", 8, 4 },
	{ 0x2, "SAMD11D12AMU", 4, 4 },
	{ 0x3, "SAMD11D14ASU", 16, 4 },
	{ 0x4, "SAMD11D13ASU", 8, 4 },
	{ 0x5, "SAMD11D12ASU", 4, 4 },
	{ 0x6, "SAMD11C14A", 16, 4 },
	{ 0x7, "SAMD11C13A", 8, 4 },
	{ 0x8, "SAMD11C12A", 4, 4 },
};

/* Known SAMD20 parts. See Table 12-8 in 42129F–SAM–10/2013 */
static const struct samd_part samd20_parts[] = {
	{ 0x0, "SAMD20J18A", 256, 32 },
	{ 0x1, "SAMD20J17A", 128, 16 },
	{ 0x2, "SAMD20J16A", 64, 8 },
	{ 0x3, "SAMD20J15A", 32, 4 },
	{ 0x4, "SAMD20J14A", 16, 2 },
	{ 0x5, "SAMD20G18A", 256, 32 },
	{ 0x6, "SAMD20G17A", 128, 16 },
	{ 0x7, "SAMD20G16A", 64, 8 },
	{ 0x8, "SAMD20G15A", 32, 4 },
	{ 0x9, "SAMD20G14A", 16, 2 },
	{ 0xA, "SAMD20E18A", 256, 32 },
	{ 0xB, "SAMD20E17A", 128, 16 },
	{ 0xC, "SAMD20E16A", 64, 8 },
	{ 0xD, "SAMD20E15A", 32, 4 },
	{ 0xE, "SAMD20E14A", 16, 2 },
};

/* Known SAMD21 parts. */
static const struct samd_part samd21_parts[] = {
	{ 0x0, "SAMD21J18A", 256, 32 },
	{ 0x1, "SAMD21J17A", 128, 16 },
	{ 0x2, "SAMD21J16A", 64, 8 },
	{ 0x3, "SAMD21J15A", 32, 4 },
	{ 0x4, "SAMD21J14A", 16, 2 },
	{ 0x5, "SAMD21G18A", 256, 32 },
	{ 0x6, "SAMD21G17A", 128, 16 },
	{ 0x7, "SAMD21G16A", 64, 8 },
	{ 0x8, "SAMD21G15A", 32, 4 },
	{ 0x9, "SAMD21G14A", 16, 2 },
	{ 0xA, "SAMD21E18A", 256, 32 },
	{ 0xB, "SAMD21E17A", 128, 16 },
	{ 0xC, "SAMD21E16A", 64, 8 },
	{ 0xD, "SAMD21E15A", 32, 4 },
	{ 0xE, "SAMD21E14A", 16, 2 },
};

/* Known SAMR21 parts. */
static const struct samd_part samr21_parts[] = {
	{ 0x19, "SAMR21G18A", 256, 32 },
	{ 0x1A, "SAMR21G17A", 128, 32 },
	{ 0x1B, "SAMR21G16A",  64, 32 },
	{ 0x1C, "SAMR21E18A", 256, 32 },
	{ 0x1D, "SAMR21E17A", 128, 32 },
	{ 0x1E, "SAMR21E16A",  64, 32 },
};

/* Known SAML21 parts. */
static const struct samd_part saml21_parts[] = {
	{ 0x00, "SAML21J18A", 256, 32 },
	{ 0x01, "SAML21J17A", 128, 16 },
	{ 0x02, "SAML21J16A", 64, 8 },
	{ 0x05, "SAML21G18A", 256, 32 },
	{ 0x06, "SAML21G17A", 128, 16 },
	{ 0x07, "SAML21G16A", 64, 8 },
	{ 0x0A, "SAML21E18A", 256, 32 },
	{ 0x0B, "SAML21E17A", 128, 16 },
	{ 0x0C, "SAML21E16A", 64, 8 },
	{ 0x0D, "SAML21E15A", 32, 4 },
};

/* Each family of parts contains a parts table in the DEVSEL field of DID.  The
 * processor ID, family ID, and series ID are used to determine which exact
 * family this is and then we can use the corresponding table. */
struct samd_family {
	uint8_t processor;
	uint8_t family;
	uint8_t series;
	const struct samd_part *parts;
	size_t num_parts;
};

/* Known SAMD families */
static const struct samd_family samd_families[] = {
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_D, SAMD_SERIES_20,
		samd20_parts, ARRAY_SIZE(samd20_parts) },
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_D, SAMD_SERIES_21,
		samd21_parts, ARRAY_SIZE(samd21_parts) },
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_D, SAMD_SERIES_21,
		samr21_parts, ARRAY_SIZE(samr21_parts) },
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_D, SAMD_SERIES_10,
		samd10_parts, ARRAY_SIZE(samd10_parts) },
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_D, SAMD_SERIES_11,
		samd11_parts, ARRAY_SIZE(samd11_parts) },
	{ SAMD_PROCESSOR_M0, SAMD_FAMILY_L, SAMD_SERIES_21,
		saml21_parts, ARRAY_SIZE(saml21_parts) },
};

struct samd_info {
	uint32_t page_size;
	int num_pages;
	int sector_size;

	bool probed;
	struct target *target;
	struct samd_info *next;
};

static struct samd_info *samd_chips;

static const struct samd_part *samd_find_part(uint32_t id)
{
	uint8_t processor = (id >> 28);
	uint8_t family = (id >> 23) & 0x1F;
	uint8_t series = (id >> 16) & 0x3F;
	uint8_t devsel = id & 0xFF;

	for (unsigned i = 0; i < ARRAY_SIZE(samd_families); i++) {
		if (samd_families[i].processor == processor &&
			samd_families[i].series == series &&
			samd_families[i].family == family) {
			for (unsigned j = 0; j < samd_families[i].num_parts; j++) {
				if (samd_families[i].parts[j].id == devsel)
					return &samd_families[i].parts[j];
			}
		}
	}

	return NULL;
}

static int samd_protect_check(struct flash_bank *bank)
{
	int res;
	uint16_t lock;

	res = target_read_u16(bank->target,
			SAMD_NVMCTRL + SAMD_NVMCTRL_LOCK, &lock);
	if (res != ERROR_OK)
		return res;

	/* Lock bits are active-low */
	for (int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = !(lock & (1<<i));

	return ERROR_OK;
}

static int samd_get_flash_page_info(struct target *target,
		uint32_t *sizep, int *nump)
{
	int res;
	uint32_t param;

	res = target_read_u32(target, SAMD_NVMCTRL + SAMD_NVMCTRL_PARAM, &param);
	if (res == ERROR_OK) {
		/* The PSZ field (bits 18:16) indicate the page size bytes as 2^(3+n)
		 * so 0 is 8KB and 7 is 1024KB. */
		if (sizep)
			*sizep = (8 << ((param >> 16) & 0x7));
		/* The NVMP field (bits 15:0) indicates the total number of pages */
		if (nump)
			*nump = param & 0xFFFF;
	} else {
		LOG_ERROR("Couldn't read NVM Parameters register");
	}

	return res;
}

static int samd_probe(struct flash_bank *bank)
{
	uint32_t id;
	int res;
	struct samd_info *chip = (struct samd_info *)bank->driver_priv;
	const struct samd_part *part;

	if (chip->probed)
		return ERROR_OK;

	res = target_read_u32(bank->target, SAMD_DSU + SAMD_DSU_DID, &id);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read Device ID register");
		return res;
	}

	part = samd_find_part(id);
	if (part == NULL) {
		LOG_ERROR("Couldn't find part correspoding to DID %08" PRIx32, id);
		return ERROR_FAIL;
	}

	bank->size = part->flash_kb * 1024;

	chip->sector_size = bank->size / SAMD_NUM_SECTORS;

	res = samd_get_flash_page_info(bank->target, &chip->page_size,
			&chip->num_pages);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't determine Flash page size");
		return res;
	}

	/* Sanity check: the total flash size in the DSU should match the page size
	 * multiplied by the number of pages. */
	if (bank->size != chip->num_pages * chip->page_size) {
		LOG_WARNING("SAMD: bank size doesn't match NVM parameters. "
				"Identified %" PRIu32 "KB Flash but NVMCTRL reports %u %" PRIu32 "B pages",
				part->flash_kb, chip->num_pages, chip->page_size);
	}

	/* Allocate the sector table */
	bank->num_sectors = SAMD_NUM_SECTORS;
	bank->sectors = calloc(bank->num_sectors, sizeof((bank->sectors)[0]));
	if (!bank->sectors)
		return ERROR_FAIL;

	/* Fill out the sector information: all SAMD sectors are the same size and
	 * there is always a fixed number of them. */
	for (int i = 0; i < bank->num_sectors; i++) {
		bank->sectors[i].size = chip->sector_size;
		bank->sectors[i].offset = i * chip->sector_size;
		/* mark as unknown */
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
	}

	samd_protect_check(bank);

	/* Done */
	chip->probed = true;

	LOG_INFO("SAMD MCU: %s (%" PRIu32 "KB Flash, %" PRIu32 "KB RAM)", part->name,
			part->flash_kb, part->ram_kb);

	return ERROR_OK;
}

static bool samd_check_error(struct target *target)
{
	int ret;
	bool error;
	uint16_t status;

	ret = target_read_u16(target,
			SAMD_NVMCTRL + SAMD_NVMCTRL_STATUS, &status);
	if (ret != ERROR_OK) {
		LOG_ERROR("Can't read NVM status");
		return true;
	}

	if (status & 0x001C) {
		if (status & (1 << 4)) /* NVME */
			LOG_ERROR("SAMD: NVM Error");
		if (status & (1 << 3)) /* LOCKE */
			LOG_ERROR("SAMD: NVM lock error");
		if (status & (1 << 2)) /* PROGE */
			LOG_ERROR("SAMD: NVM programming error");

		error = true;
	} else {
		error = false;
	}

	/* Clear the error conditions by writing a one to them */
	ret = target_write_u16(target,
			SAMD_NVMCTRL + SAMD_NVMCTRL_STATUS, status);
	if (ret != ERROR_OK)
		LOG_ERROR("Can't clear NVM error conditions");

	return error;
}

static int samd_issue_nvmctrl_command(struct target *target, uint16_t cmd)
{
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Read current configuration. */
	uint16_t tmp = 0;
	int res = target_read_u16(target, SAMD_NVMCTRL + SAMD_NVMCTRL_CTRLB,
			&tmp);
	if (res != ERROR_OK)
		return res;

	/* Set cache disable. */
	res = target_write_u16(target, SAMD_NVMCTRL + SAMD_NVMCTRL_CTRLB,
			tmp | (1<<18));
	if (res != ERROR_OK)
		return res;

	/* Issue the NVM command */
	int res_cmd = target_write_u16(target,
			SAMD_NVMCTRL + SAMD_NVMCTRL_CTRLA, SAMD_NVM_CMD(cmd));

	/* Try to restore configuration, regardless of NVM command write
	 * status. */
	res = target_write_u16(target, SAMD_NVMCTRL + SAMD_NVMCTRL_CTRLB, tmp);

	if (res_cmd != ERROR_OK)
		return res_cmd;

	if (res != ERROR_OK)
		return res;

	/* Check to see if the NVM command resulted in an error condition. */
	if (samd_check_error(target))
		return ERROR_FAIL;

	return ERROR_OK;
}

static int samd_erase_row(struct target *target, uint32_t address)
{
	int res;

	/* Set an address contained in the row to be erased */
	res = target_write_u32(target,
			SAMD_NVMCTRL + SAMD_NVMCTRL_ADDR, address >> 1);

	/* Issue the Erase Row command to erase that row. */
	if (res == ERROR_OK)
		res = samd_issue_nvmctrl_command(target,
				address == SAMD_USER_ROW ? SAMD_NVM_CMD_EAR : SAMD_NVM_CMD_ER);

	if (res != ERROR_OK)  {
		LOG_ERROR("Failed to erase row containing %08" PRIx32, address);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static bool is_user_row_reserved_bit(uint8_t bit)
{
	/* See Table 9-3 in the SAMD20 datasheet for more information. */
	switch (bit) {
		/* Reserved bits */
		case 3:
		case 7:
		/* Voltage regulator internal configuration with default value of 0x70,
		 * may not be changed. */
		case 17 ... 24:
		/* 41 is voltage regulator internal configuration and must not be
		 * changed.  42 through 47 are reserved. */
		case 41 ... 47:
			return true;
		default:
			break;
	}

	return false;
}

/* Modify the contents of the User Row in Flash.  These are described in Table
 * 9-3 of the SAMD20 datasheet.  The User Row itself has a size of one page
 * and contains a combination of "fuses" and calibration data in bits 24:17.
 * We therefore try not to erase the row's contents unless we absolutely have
 * to and we don't permit modifying reserved bits. */
static int samd_modify_user_row(struct target *target, uint32_t value,
		uint8_t startb, uint8_t endb)
{
	int res;

	if (is_user_row_reserved_bit(startb) || is_user_row_reserved_bit(endb)) {
		LOG_ERROR("Can't modify bits in the requested range");
		return ERROR_FAIL;
	}

	/* Retrieve the MCU's page size, in bytes. This is also the size of the
	 * entire User Row. */
	uint32_t page_size;
	res = samd_get_flash_page_info(target, &page_size, NULL);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't determine Flash page size");
		return res;
	}

	/* Make sure the size is sane before we allocate. */
	assert(page_size > 0 && page_size <= SAMD_PAGE_SIZE_MAX);

	/* Make sure we're within the single page that comprises the User Row. */
	if (startb >= (page_size * 8) || endb >= (page_size * 8)) {
		LOG_ERROR("Can't modify bits outside the User Row page range");
		return ERROR_FAIL;
	}

	uint8_t *buf = malloc(page_size);
	if (!buf)
		return ERROR_FAIL;

	/* Read the user row (comprising one page) by half-words. */
	res = target_read_memory(target, SAMD_USER_ROW, 2, page_size / 2, buf);
	if (res != ERROR_OK)
		goto out_user_row;

	/* We will need to erase before writing if the new value needs a '1' in any
	 * position for which the current value had a '0'.  Otherwise we can avoid
	 * erasing. */
	uint32_t cur = buf_get_u32(buf, startb, endb - startb + 1);
	if ((~cur) & value) {
		res = samd_erase_row(target, SAMD_USER_ROW);
		if (res != ERROR_OK) {
			LOG_ERROR("Couldn't erase user row");
			goto out_user_row;
		}
	}

	/* Modify */
	buf_set_u32(buf, startb, endb - startb + 1, value);

	/* Write the page buffer back out to the target.  A Flash write will be
	 * triggered automatically. */
	res = target_write_memory(target, SAMD_USER_ROW, 4, page_size / 4, buf);
	if (res != ERROR_OK)
		goto out_user_row;

	if (samd_check_error(target)) {
		res = ERROR_FAIL;
		goto out_user_row;
	}

	/* Success */
	res = ERROR_OK;

out_user_row:
	free(buf);

	return res;
}

static int samd_protect(struct flash_bank *bank, int set, int first, int last)
{
	struct samd_info *chip = (struct samd_info *)bank->driver_priv;

	/* We can issue lock/unlock region commands with the target running but
	 * the settings won't persist unless we're able to modify the LOCK regions
	 * and that requires the target to be halted. */
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int res = ERROR_OK;

	for (int s = first; s <= last; s++) {
		if (set != bank->sectors[s].is_protected) {
			/* Load an address that is within this sector (we use offset 0) */
			res = target_write_u32(bank->target,
							SAMD_NVMCTRL + SAMD_NVMCTRL_ADDR,
							((s * chip->sector_size) >> 1));
			if (res != ERROR_OK)
				goto exit;

			/* Tell the controller to lock that sector */
			res = samd_issue_nvmctrl_command(bank->target,
					set ? SAMD_NVM_CMD_LR : SAMD_NVM_CMD_UR);
			if (res != ERROR_OK)
				goto exit;
		}
	}

	/* We've now applied our changes, however they will be undone by the next
	 * reset unless we also apply them to the LOCK bits in the User Page.  The
	 * LOCK bits start at bit 48, correspoding to Sector 0 and end with bit 63,
	 * corresponding to Sector 15.  A '1' means unlocked and a '0' means
	 * locked.  See Table 9-3 in the SAMD20 datasheet for more details. */

	res = samd_modify_user_row(bank->target, set ? 0x0000 : 0xFFFF,
			48 + first, 48 + last);
	if (res != ERROR_OK)
		LOG_WARNING("SAMD: protect settings were not made persistent!");

	res = ERROR_OK;

exit:
	samd_protect_check(bank);

	return res;
}

static int samd_erase(struct flash_bank *bank, int first, int last)
{
	int res;
	int rows_in_sector;
	struct samd_info *chip = (struct samd_info *)bank->driver_priv;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");

		return ERROR_TARGET_NOT_HALTED;
	}

	if (!chip->probed) {
		if (samd_probe(bank) != ERROR_OK)
			return ERROR_FLASH_BANK_NOT_PROBED;
	}

	/* The SAMD NVM has row erase granularity.  There are four pages in a row
	 * and the number of rows in a sector depends on the sector size, which in
	 * turn depends on the Flash capacity as there is a fixed number of
	 * sectors. */
	rows_in_sector = chip->sector_size / (chip->page_size * 4);

	/* For each sector to be erased */
	for (int s = first; s <= last; s++) {
		if (bank->sectors[s].is_protected) {
			LOG_ERROR("SAMD: failed to erase sector %d. That sector is write-protected", s);
			return ERROR_FLASH_OPERATION_FAILED;
		}

		if (bank->sectors[s].is_erased != 1) {
			/* For each row in that sector */
			for (int r = s * rows_in_sector; r < (s + 1) * rows_in_sector; r++) {
				res = samd_erase_row(bank->target, r * chip->page_size * 4);
				if (res != ERROR_OK) {
					LOG_ERROR("SAMD: failed to erase sector %d", s);
					return res;
				}
			}

			bank->sectors[s].is_erased = 1;
		}
	}

	return ERROR_OK;
}

static struct flash_sector *samd_find_sector_by_address(struct flash_bank *bank, uint32_t address)
{
	struct samd_info *chip = (struct samd_info *)bank->driver_priv;

	for (int i = 0; i < bank->num_sectors; i++) {
		if (bank->sectors[i].offset <= address &&
		    address < bank->sectors[i].offset + chip->sector_size)
			return &bank->sectors[i];
	}
	return NULL;
}

/* Write an entire row (four pages) from host buffer 'buf' to row-aligned
 * 'address' in the Flash. */
static int samd_write_row(struct flash_bank *bank, uint32_t address,
		const uint8_t *buf)
{
	int res;
	struct samd_info *chip = (struct samd_info *)bank->driver_priv;

	struct flash_sector *sector = samd_find_sector_by_address(bank, address);

	if (!sector) {
		LOG_ERROR("Can't find sector corresponding to address 0x%08" PRIx32, address);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	if (sector->is_protected) {
		LOG_ERROR("Trying to write to a protected sector at 0x%08" PRIx32, address);
		return ERROR_FLASH_OPERATION_FAILED;
	}

	/* Erase the row that we'll be writing to */
	res = samd_erase_row(bank->target, address);
	if (res != ERROR_OK)
		return res;

	/* Now write the pages in this row. */
	for (unsigned int i = 0; i < 4; i++) {
		bool error;

		/* Write the page contents to the target's page buffer.  A page write
		 * is issued automatically once the last location is written in the
		 * page buffer (ie: a complete page has been written out). */
		res = target_write_memory(bank->target, address, 4,
				chip->page_size / 4, buf);
		if (res != ERROR_OK) {
			LOG_ERROR("%s: %d", __func__, __LINE__);
			return res;
		}

		/* Access through AHB is stalled while flash is being programmed */
		usleep(200);

		error = samd_check_error(bank->target);
		if (error)
			return ERROR_FAIL;

		/* Next page */
		address += chip->page_size;
		buf += chip->page_size;
	}

	sector->is_erased = 0;

	return res;
}

/* Write partial contents into row-aligned 'address' on the Flash from host
 * buffer 'buf' by writing 'nb' of 'buf' at 'row_offset' into the Flash row. */
static int samd_write_row_partial(struct flash_bank *bank, uint32_t address,
		const uint8_t *buf, uint32_t row_offset, uint32_t nb)
{
	int res;
	struct samd_info *chip = (struct samd_info *)bank->driver_priv;
	uint32_t row_size = chip->page_size * 4;
	uint8_t *rb = malloc(row_size);
	if (!rb)
		return ERROR_FAIL;

	assert(row_offset + nb < row_size);
	assert((address % row_size) == 0);

	/* Retrieve the full row contents from Flash */
	res = target_read_memory(bank->target, address, 4, row_size / 4, rb);
	if (res != ERROR_OK) {
		free(rb);
		return res;
	}

	/* Insert our partial row over the data from Flash */
	memcpy(rb + (row_offset % row_size), buf, nb);

	/* Write the row back out */
	res = samd_write_row(bank, address, rb);
	free(rb);

	return res;
}

static int samd_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	int res;
	uint32_t address;
	uint32_t nb = 0;
	struct samd_info *chip = (struct samd_info *)bank->driver_priv;
	uint32_t row_size = chip->page_size * 4;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");

		return ERROR_TARGET_NOT_HALTED;
	}

	if (!chip->probed) {
		if (samd_probe(bank) != ERROR_OK)
			return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (offset % row_size) {
		/* We're starting at an unaligned offset so we'll write a partial row
		 * comprising that offset and up to the end of that row. */
		nb = row_size - (offset % row_size);
		if (nb > count)
			nb = count;
	} else if (count < row_size) {
		/* We're writing an aligned but partial row. */
		nb = count;
	}

	address = (offset / row_size) * row_size + bank->base;

	if (nb > 0) {
		res = samd_write_row_partial(bank, address, buffer,
				offset % row_size, nb);
		if (res != ERROR_OK)
			return res;

		/* We're done with the row contents */
		count -= nb;
		offset += nb;
		buffer += row_size;
	}

	/* There's at least one aligned row to write out. */
	if (count >= row_size) {
		int nr = count / row_size + ((count % row_size) ? 1 : 0);
		unsigned int r = 0;

		for (unsigned int i = address / row_size;
				(i < (address / row_size) + nr) && count > 0; i++) {
			address = (i * row_size) + bank->base;

			if (count >= row_size) {
				res = samd_write_row(bank, address, buffer + (r * row_size));
				/* Advance one row */
				offset += row_size;
				count -= row_size;
			} else {
				res = samd_write_row_partial(bank, address,
						buffer + (r * row_size), 0, count);
				/* We're done after this. */
				offset += count;
				count = 0;
			}

			r++;

			if (res != ERROR_OK)
				return res;
		}
	}

	return ERROR_OK;
}

FLASH_BANK_COMMAND_HANDLER(samd_flash_bank_command)
{
	struct samd_info *chip = samd_chips;

	while (chip) {
		if (chip->target == bank->target)
			break;
		chip = chip->next;
	}

	if (!chip) {
		/* Create a new chip */
		chip = calloc(1, sizeof(*chip));
		if (!chip)
			return ERROR_FAIL;

		chip->target = bank->target;
		chip->probed = false;

		bank->driver_priv = chip;

		/* Insert it into the chips list (at head) */
		chip->next = samd_chips;
		samd_chips = chip;
	}

	if (bank->base != SAMD_FLASH) {
		LOG_ERROR("Address 0x%08" PRIx32 " invalid bank address (try 0x%08" PRIx32
				"[at91samd series] )",
				bank->base, SAMD_FLASH);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(samd_handle_info_command)
{
	return ERROR_OK;
}

COMMAND_HANDLER(samd_handle_chip_erase_command)
{
	struct target *target = get_current_target(CMD_CTX);

	if (target) {
		/* Enable access to the DSU by disabling the write protect bit */
		target_write_u32(target, SAMD_PAC1, (1<<1));
		/* Tell the DSU to perform a full chip erase.  It takes about 240ms to
		 * perform the erase. */
		target_write_u8(target, SAMD_DSU, (1<<4));

		command_print(CMD_CTX, "chip erased");
	}

	return ERROR_OK;
}

COMMAND_HANDLER(samd_handle_set_security_command)
{
	int res = ERROR_OK;
	struct target *target = get_current_target(CMD_CTX);

	if (CMD_ARGC < 1 || (CMD_ARGC >= 1 && (strcmp(CMD_ARGV[0], "enable")))) {
		command_print(CMD_CTX, "supply the \"enable\" argument to proceed.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (target) {
		if (target->state != TARGET_HALTED) {
			LOG_ERROR("Target not halted");
			return ERROR_TARGET_NOT_HALTED;
		}

		res = samd_issue_nvmctrl_command(target, SAMD_NVM_CMD_SSB);

		/* Check (and clear) error conditions */
		if (res == ERROR_OK)
			command_print(CMD_CTX, "chip secured on next power-cycle");
		else
			command_print(CMD_CTX, "failed to secure chip");
	}

	return res;
}

COMMAND_HANDLER(samd_handle_eeprom_command)
{
	int res = ERROR_OK;
	struct target *target = get_current_target(CMD_CTX);

	if (target) {
		if (target->state != TARGET_HALTED) {
			LOG_ERROR("Target not halted");
			return ERROR_TARGET_NOT_HALTED;
		}

		if (CMD_ARGC >= 1) {
			int val = atoi(CMD_ARGV[0]);
			uint32_t code;

			if (val == 0)
				code = 7;
			else {
				/* Try to match size in bytes with corresponding size code */
				for (code = 0; code <= 6; code++) {
					if (val == (2 << (13 - code)))
						break;
				}

				if (code > 6) {
					command_print(CMD_CTX, "Invalid EEPROM size.  Please see "
							"datasheet for a list valid sizes.");
					return ERROR_COMMAND_SYNTAX_ERROR;
				}
			}

			res = samd_modify_user_row(target, code, 4, 6);
		} else {
			uint16_t val;
			res = target_read_u16(target, SAMD_USER_ROW, &val);
			if (res == ERROR_OK) {
				uint32_t size = ((val >> 4) & 0x7); /* grab size code */

				if (size == 0x7)
					command_print(CMD_CTX, "EEPROM is disabled");
				else {
					/* Otherwise, 6 is 256B, 0 is 16KB */
					command_print(CMD_CTX, "EEPROM size is %u bytes",
							(2 << (13 - size)));
				}
			}
		}
	}

	return res;
}

COMMAND_HANDLER(samd_handle_bootloader_command)
{
	int res = ERROR_OK;
	struct target *target = get_current_target(CMD_CTX);

	if (target) {
		if (target->state != TARGET_HALTED) {
			LOG_ERROR("Target not halted");
			return ERROR_TARGET_NOT_HALTED;
		}

		/* Retrieve the MCU's page size, in bytes. */
		uint32_t page_size;
		res = samd_get_flash_page_info(target, &page_size, NULL);
		if (res != ERROR_OK) {
			LOG_ERROR("Couldn't determine Flash page size");
			return res;
		}

		if (CMD_ARGC >= 1) {
			int val = atoi(CMD_ARGV[0]);
			uint32_t code;

			if (val == 0)
				code = 7;
			else {
				/* Try to match size in bytes with corresponding size code */
				for (code = 0; code <= 6; code++) {
					if ((unsigned int)val == (2UL << (8UL - code)) * page_size)
						break;
				}

				if (code > 6) {
					command_print(CMD_CTX, "Invalid bootloader size.  Please "
							"see datasheet for a list valid sizes.");
					return ERROR_COMMAND_SYNTAX_ERROR;
				}

			}

			res = samd_modify_user_row(target, code, 0, 2);
		} else {
			uint16_t val;
			res = target_read_u16(target, SAMD_USER_ROW, &val);
			if (res == ERROR_OK) {
				uint32_t size = (val & 0x7); /* grab size code */
				uint32_t nb;

				if (size == 0x7)
					nb = 0;
				else
					nb = (2 << (8 - size)) * page_size;

				/* There are 4 pages per row */
				command_print(CMD_CTX, "Bootloader size is %" PRIu32 " bytes (%" PRIu32 " rows)",
					   nb, (uint32_t)(nb / (page_size * 4)));
			}
		}
	}

	return res;
}

static const struct command_registration at91samd_exec_command_handlers[] = {
	{
		.name = "info",
		.handler = samd_handle_info_command,
		.mode = COMMAND_EXEC,
		.help = "Print information about the current at91samd chip"
			"and its flash configuration.",
	},
	{
		.name = "chip-erase",
		.handler = samd_handle_chip_erase_command,
		.mode = COMMAND_EXEC,
		.help = "Erase the entire Flash by using the Chip"
			"Erase feature in the Device Service Unit (DSU).",
	},
	{
		.name = "set-security",
		.handler = samd_handle_set_security_command,
		.mode = COMMAND_EXEC,
		.help = "Secure the chip's Flash by setting the Security Bit."
			"This makes it impossible to read the Flash contents."
			"The only way to undo this is to issue the chip-erase"
			"command.",
	},
	{
		.name = "eeprom",
		.usage = "[size_in_bytes]",
		.handler = samd_handle_eeprom_command,
		.mode = COMMAND_EXEC,
		.help = "Show or set the EEPROM size setting, stored in the User Row."
			"Please see Table 20-3 of the SAMD20 datasheet for allowed values."
			"Changes are stored immediately but take affect after the MCU is"
			"reset.",
	},
	{
		.name = "bootloader",
		.usage = "[size_in_bytes]",
		.handler = samd_handle_bootloader_command,
		.mode = COMMAND_EXEC,
		.help = "Show or set the bootloader size, stored in the User Row."
			"Please see Table 20-2 of the SAMD20 datasheet for allowed values."
			"Changes are stored immediately but take affect after the MCU is"
			"reset.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration at91samd_command_handlers[] = {
	{
		.name = "at91samd",
		.mode = COMMAND_ANY,
		.help = "at91samd flash command group",
		.usage = "",
		.chain = at91samd_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct flash_driver at91samd_flash = {
	.name = "at91samd",
	.commands = at91samd_command_handlers,
	.flash_bank_command = samd_flash_bank_command,
	.erase = samd_erase,
	.protect = samd_protect,
	.write = samd_write,
	.read = default_flash_read,
	.probe = samd_probe,
	.auto_probe = samd_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = samd_protect_check,
};
