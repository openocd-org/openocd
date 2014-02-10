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

#define SAMD_NUM_SECTORS	16

#define SAMD_FLASH			0x00000000	/* physical Flash memory */
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
#define SAMD_SERIES_20		0x00
#define SAMD_SERIES_21		0x01

struct samd_part {
	uint8_t id;
	const char *name;
	uint32_t flash_kb;
	uint32_t ram_kb;
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
	uint8_t family = (id >> 24) & 0x0F;
	uint8_t series = (id >> 16) & 0xFF;
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

static int samd_probe(struct flash_bank *bank)
{
	uint32_t id, param;
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

	res = target_read_u32(bank->target,
			SAMD_NVMCTRL + SAMD_NVMCTRL_PARAM, &param);
	if (res != ERROR_OK) {
		LOG_ERROR("Couldn't read NVM Parameters register");
		return res;
	}

	bank->size = part->flash_kb * 1024;

	chip->sector_size = bank->size / SAMD_NUM_SECTORS;

	/* The PSZ field (bits 18:16) indicate the page size bytes as 2^(3+n) so
	 * 0 is 8KB and 7 is 1024KB. */
	chip->page_size = (8 << ((param >> 16) & 0x7));
	/* The NVMP field (bits 15:0) indicates the total number of pages */
	chip->num_pages = param & 0xFFFF;

	/* Sanity check: the total flash size in the DSU should match the page size
	 * multiplied by the number of pages. */
	if (bank->size != chip->num_pages * chip->page_size) {
		LOG_WARNING("SAMD: bank size doesn't match NVM parameters. "
				"Identified %uKB Flash but NVMCTRL reports %u %uB pages",
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

	LOG_INFO("SAMD MCU: %s (%uKB Flash, %uKB RAM)", part->name,
			part->flash_kb, part->ram_kb);

	return ERROR_OK;
}

static int samd_protect(struct flash_bank *bank, int set, int first, int last)
{
	int res;
	struct samd_info *chip = (struct samd_info *)bank->driver_priv;

	res = ERROR_OK;

	for (int s = first; s <= last; s++) {
		if (set != bank->sectors[s].is_protected) {
			/* Load an address that is within this sector (we use offset 0) */
			res = target_write_u32(bank->target, SAMD_NVMCTRL + SAMD_NVMCTRL_ADDR,
					       s * chip->sector_size);
			if (res != ERROR_OK)
				goto exit;

			/* Tell the controller to lock that sector */

			uint16_t cmd = (set) ?
				SAMD_NVM_CMD(SAMD_NVM_CMD_LR) :
				SAMD_NVM_CMD(SAMD_NVM_CMD_UR);

			res = target_write_u16(bank->target,
					       SAMD_NVMCTRL + SAMD_NVMCTRL_CTRLA,
					       cmd);
			if (res != ERROR_OK)
				goto exit;
		}
	}
exit:
	samd_protect_check(bank);

	return res;
}

static bool samd_check_error(struct flash_bank *bank)
{
	int ret;
	bool error;
	uint16_t status;

	ret = target_read_u16(bank->target,
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
	ret = target_write_u16(bank->target,
			SAMD_NVMCTRL + SAMD_NVMCTRL_STATUS, status);
	if (ret != ERROR_OK)
		LOG_ERROR("Can't clear NVM error conditions");

	return error;
}

static int samd_erase_row(struct flash_bank *bank, uint32_t address)
{
	int res;
	bool error = false;

	/* Set an address contained in the row to be erased */
	res = target_write_u32(bank->target,
			SAMD_NVMCTRL + SAMD_NVMCTRL_ADDR, address >> 1);
	if (res == ERROR_OK) {
		/* Issue the Erase Row command to erase that row */
		res = target_write_u16(bank->target,
				SAMD_NVMCTRL + SAMD_NVMCTRL_CTRLA,
				SAMD_NVM_CMD(SAMD_NVM_CMD_ER));

		/* Check (and clear) error conditions */
		error = samd_check_error(bank);
	}

	if (res != ERROR_OK || error)  {
		LOG_ERROR("Failed to erase row containing %08X" PRIx32, address);
		return ERROR_FAIL;
	}

	return ERROR_OK;
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

		if (!bank->sectors[s].is_erased) {
			/* For each row in that sector */
			for (int r = s * rows_in_sector; r < (s + 1) * rows_in_sector; r++) {
				res = samd_erase_row(bank, r * chip->page_size * 4);
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
	res = samd_erase_row(bank, address);
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

		error = samd_check_error(bank);
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

static const struct command_registration at91samd_exec_command_handlers[] = {
	{
		.name = "info",
		.handler = samd_handle_info_command,
		.mode = COMMAND_EXEC,
		.help = "Print information about the current at91samd chip"
			"and its flash configuration.",
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
