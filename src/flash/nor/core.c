/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath <Dominic.Rath@gmx.de>              *
 *   Copyright (C) 2007,2008 Øyvind Harboe <oyvind.harboe@zylin.com>       *
 *   Copyright (C) 2008 by Spencer Oliver <spen@spen-soft.co.uk>           *
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif
#include <flash/common.h>
#include <flash/nor/core.h>
#include <flash/nor/imp.h>
#include <target/image.h>


/**
 * @file
 * Upper level of NOR flash framework.
 * The lower level interfaces are to drivers.  These upper level ones
 * primarily support access from Tcl scripts or from GDB.
 */

struct flash_bank *flash_banks;

int flash_driver_erase(struct flash_bank *bank, int first, int last)
{
	int retval;

	retval = bank->driver->erase(bank, first, last);
	if (retval != ERROR_OK)
	{
		LOG_ERROR("failed erasing sectors %d to %d (%d)", first, last, retval);
	}

	return retval;
}

int flash_driver_protect(struct flash_bank *bank, int set, int first, int last)
{
	int retval;

	retval = bank->driver->protect(bank, set, first, last);
	if (retval != ERROR_OK)
	{
		LOG_ERROR("failed setting protection for areas %d to %d (%d)", first, last, retval);
	}

	return retval;
}

int flash_driver_write(struct flash_bank *bank,
		uint8_t *buffer, uint32_t offset, uint32_t count)
{
	int retval;

	retval = bank->driver->write(bank, buffer, offset, count);
	if (retval != ERROR_OK)
	{
		LOG_ERROR("error writing to flash at address 0x%08" PRIx32 " at offset 0x%8.8" PRIx32 " (%d)",
			  bank->base, offset, retval);
	}

	return retval;
}

void flash_bank_add(struct flash_bank *bank)
{
	/* put flash bank in linked list */
	unsigned bank_num = 0;
	if (flash_banks)
	{
		/* find last flash bank */
		struct flash_bank *p = flash_banks;
		while (NULL != p->next)
		{
			bank_num += 1;
			p = p->next;
		}
		p->next = bank;
		bank_num += 1;
	}
	else
		flash_banks = bank;

	bank->bank_number = bank_num;
}

struct flash_bank *flash_bank_list(void)
{
	return flash_banks;
}

struct flash_bank *get_flash_bank_by_num_noprobe(int num)
{
	struct flash_bank *p;
	int i = 0;

	for (p = flash_banks; p; p = p->next)
	{
		if (i++ == num)
		{
			return p;
		}
	}
	LOG_ERROR("flash bank %d does not exist", num);
	return NULL;
}

int flash_get_bank_count(void)
{
	struct flash_bank *p;
	int i = 0;
	for (p = flash_banks; p; p = p->next)
	{
		i++;
	}
	return i;
}

struct flash_bank *get_flash_bank_by_name(const char *name)
{
	unsigned requested = get_flash_name_index(name);
	unsigned found = 0;

	struct flash_bank *bank;
	for (bank = flash_banks; NULL != bank; bank = bank->next)
	{
		if (strcmp(bank->name, name) == 0)
			return bank;
		if (!flash_driver_name_matches(bank->driver->name, name))
			continue;
		if (++found < requested)
			continue;
		return bank;
	}
	return NULL;
}

struct flash_bank *get_flash_bank_by_num(int num)
{
	struct flash_bank *p = get_flash_bank_by_num_noprobe(num);
	int retval;

	if (p == NULL)
		return NULL;

	retval = p->driver->auto_probe(p);

	if (retval != ERROR_OK)
	{
		LOG_ERROR("auto_probe failed %d\n", retval);
		return NULL;
	}
	return p;
}

/* lookup flash bank by address */
struct flash_bank *get_flash_bank_by_addr(struct target *target, uint32_t addr)
{
	struct flash_bank *c;

	/* cycle through bank list */
	for (c = flash_banks; c; c = c->next)
	{
		int retval;
		retval = c->driver->auto_probe(c);

		if (retval != ERROR_OK)
		{
			LOG_ERROR("auto_probe failed %d\n", retval);
			return NULL;
		}
		/* check whether address belongs to this flash bank */
		if ((addr >= c->base) && (addr <= c->base + (c->size - 1)) && target == c->target)
			return c;
	}
	LOG_ERROR("No flash at address 0x%08" PRIx32 "\n", addr);
	return NULL;
}

int default_flash_mem_blank_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	const int buffer_size = 1024;
	int i;
	uint32_t nBytes;
	int retval = ERROR_OK;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	uint8_t *buffer = malloc(buffer_size);

	for (i = 0; i < bank->num_sectors; i++)
	{
		uint32_t j;
		bank->sectors[i].is_erased = 1;

		for (j = 0; j < bank->sectors[i].size; j += buffer_size)
		{
			uint32_t chunk;
			chunk = buffer_size;
			if (chunk > (j - bank->sectors[i].size))
			{
				chunk = (j - bank->sectors[i].size);
			}

			retval = target_read_memory(target, bank->base + bank->sectors[i].offset + j, 4, chunk/4, buffer);
			if (retval != ERROR_OK)
			{
				goto done;
			}

			for (nBytes = 0; nBytes < chunk; nBytes++)
			{
				if (buffer[nBytes] != 0xFF)
				{
					bank->sectors[i].is_erased = 0;
					break;
				}
			}
		}
	}

	done:
	free(buffer);

	return retval;
}

int default_flash_blank_check(struct flash_bank *bank)
{
	struct target *target = bank->target;
	int i;
	int retval;
	int fast_check = 0;
	uint32_t blank;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	for (i = 0; i < bank->num_sectors; i++)
	{
		uint32_t address = bank->base + bank->sectors[i].offset;
		uint32_t size = bank->sectors[i].size;

		if ((retval = target_blank_check_memory(target, address, size, &blank)) != ERROR_OK)
		{
			fast_check = 0;
			break;
		}
		if (blank == 0xFF)
			bank->sectors[i].is_erased = 1;
		else
			bank->sectors[i].is_erased = 0;
		fast_check = 1;
	}

	if (!fast_check)
	{
		LOG_USER("Running slow fallback erase check - add working memory");
		return default_flash_mem_blank_check(bank);
	}

	return ERROR_OK;
}

/* Manipulate given flash region, selecting the bank according to target
 * and address.  Maps an address range to a set of sectors, and issues
 * the callback() on that set ... e.g. to erase or unprotect its members.
 *
 * (Note a current bad assumption:  that protection operates on the same
 * size sectors as erase operations use.)
 *
 * The "pad_reason" parameter is a kind of boolean:  when it's NULL, the
 * range must fit those sectors exactly.  This is clearly safe; it can't
 * erase data which the caller said to leave alone, for example.  If it's
 * non-NULL, rather than failing, extra data in the first and/or last
 * sectors will be added to the range, and that reason string is used when
 * warning about those additions.
 */
static int flash_iterate_address_range(struct target *target,
		char *pad_reason, uint32_t addr, uint32_t length,
		int (*callback)(struct flash_bank *bank, int first, int last))
{
	struct flash_bank *c;
	uint32_t last_addr = addr + length;	/* first address AFTER end */
	int first = -1;
	int last = -1;
	int i;

	if ((c = get_flash_bank_by_addr(target, addr)) == NULL)
		return ERROR_FLASH_DST_OUT_OF_BANK; /* no corresponding bank found */

	if (c->size == 0 || c->num_sectors == 0)
	{
		LOG_ERROR("Bank is invalid");
		return ERROR_FLASH_BANK_INVALID;
	}

	if (length == 0)
	{
		/* special case, erase whole bank when length is zero */
		if (addr != c->base)
		{
			LOG_ERROR("Whole bank access must start at beginning of bank.");
			return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
		}

		return callback(c, 0, c->num_sectors - 1);
	}

	/* check whether it all fits in this bank */
	if (addr + length - 1 > c->base + c->size - 1)
	{
		LOG_ERROR("Flash access does not fit into bank.");
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/** @todo: handle erasures that cross into adjacent banks */

	addr -= c->base;
	last_addr -= c->base;

	for (i = 0; i < c->num_sectors; i++)
	{
		struct flash_sector *f = c->sectors + i;
		uint32_t end = f->offset + f->size;

		/* start only on a sector boundary */
		if (first < 0) {
			/* scanned past the first sector? */
			if (addr < f->offset)
				break;

			/* is this the first sector? */
			if (addr == f->offset)
				first = i;

			/* Does this need head-padding?  If so, pad and warn;
			 * or else force an error.
			 *
			 * Such padding can make trouble, since *WE* can't
			 * ever know if that data was in use.  The warning
			 * should help users sort out messes later.
			 */
			else if (addr < end && pad_reason) {
				/* FIXME say how many bytes (e.g. 80 KB) */
				LOG_WARNING("Adding extra %s range, "
						"%#8.8x to %#8.8x",
					pad_reason,
					(unsigned) f->offset,
					(unsigned) addr - 1);
				first = i;
			} else
				continue;
		}

		/* is this (also?) the last sector? */
		if (last_addr == end) {
			last = i;
			break;
		}

		/* Does this need tail-padding?  If so, pad and warn;
		 * or else force an error.
		 */
		if (last_addr < end && pad_reason) {
			/* FIXME say how many bytes (e.g. 80 KB) */
			LOG_WARNING("Adding extra %s range, "
					"%#8.8x to %#8.8x",
				pad_reason,
				(unsigned) last_addr,
				(unsigned) end - 1);
			last = i;
			break;
		}

		/* MUST finish on a sector boundary */
		if (last_addr <= f->offset)
			break;
	}

	/* invalid start or end address? */
	if (first == -1 || last == -1) {
		LOG_ERROR("address range 0x%8.8x .. 0x%8.8x "
				"is not sector-aligned",
				(unsigned) (c->base + addr),
				(unsigned) (c->base + last_addr - 1));
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}

	/* The NOR driver may trim this range down, based on what
	 * sectors are already erased/unprotected.  GDB currently
	 * blocks such optimizations.
	 */
	return callback(c, first, last);
}

int flash_erase_address_range(struct target *target,
		bool pad, uint32_t addr, uint32_t length)
{
	return flash_iterate_address_range(target, pad ? "erase" : NULL,
			addr, length, &flash_driver_erase);
}

static int flash_driver_unprotect(struct flash_bank *bank, int first, int last)
{
	return flash_driver_protect(bank, 0, first, last);
}

static int flash_unlock_address_range(struct target *target, uint32_t addr, uint32_t length)
{
	/* By default, pad to sector boundaries ... the real issue here
	 * is that our (only) caller *permanently* removes protection,
	 * and doesn't restore it.
	 */
	return flash_iterate_address_range(target, "unprotect",
			addr, length, &flash_driver_unprotect);
}

int flash_write_unlock(struct target *target, struct image *image,
		uint32_t *written, int erase, bool unlock)
{
	int retval = ERROR_OK;

	int section;
	uint32_t section_offset;
	struct flash_bank *c;
	int *padding;

	/* REVISIT do_pad should perhaps just be another parameter.
	 * GDB wouldn't ever need it, since it erases separately.
	 * But "flash write_image" commands might want that option.
	 */
	bool do_pad = false;

	section = 0;
	section_offset = 0;

	if (written)
		*written = 0;

	if (erase)
	{
		/* assume all sectors need erasing - stops any problems
		 * when flash_write is called multiple times */

		flash_set_dirty();
	}

	/* allocate padding array */
	padding = calloc(image->num_sections, sizeof(*padding));

	/* loop until we reach end of the image */
	while (section < image->num_sections)
	{
		uint32_t buffer_size;
		uint8_t *buffer;
		int section_first;
		int section_last;
		uint32_t run_address = image->sections[section].base_address + section_offset;
		uint32_t run_size = image->sections[section].size - section_offset;
		int pad_bytes = 0;

		if (image->sections[section].size ==  0)
		{
			LOG_WARNING("empty section %d", section);
			section++;
			section_offset = 0;
			continue;
		}

		/* find the corresponding flash bank */
		if ((c = get_flash_bank_by_addr(target, run_address)) == NULL)
		{
			section++; /* and skip it */
			section_offset = 0;
			continue;
		}

		/* collect consecutive sections which fall into the same bank */
		section_first = section;
		section_last = section;
		padding[section] = 0;
		while ((run_address + run_size - 1 < c->base + c->size - 1)
				&& (section_last + 1 < image->num_sections))
		{
			if (image->sections[section_last + 1].base_address < (run_address + run_size))
			{
				LOG_DEBUG("section %d out of order "
						"(surprising, but supported)",
						section_last + 1);
				/* REVISIT this can break with autoerase ...
				 * clobbering data after it's written.
				 */
				break;
			}

			/* FIXME This needlessly touches sectors BETWEEN the
			 * sections it's writing.  Without auto erase, it just
			 * writes ones.  That WILL INVALIDATE data in cases
			 * like Stellaris Tempest chips, corrupting internal
			 * ECC codes; and at least FreeScale suggests issues
			 * with that approach (in HC11 documentation).
			 *
			 * With auto erase enabled, data in those sectors will
			 * be needlessly destroyed; and some of the limited
			 * number of flash erase cycles will be wasted...
			 *
			 * In both cases, the extra writes slow things down.
			 */

			/* if we have multiple sections within our image,
			 * flash programming could fail due to alignment issues
			 * attempt to rebuild a consecutive buffer for the flash loader */
			pad_bytes = (image->sections[section_last + 1].base_address) - (run_address + run_size);
			if ((run_address + run_size + pad_bytes) > (c->base + c->size))
				break;
			padding[section_last] = pad_bytes;
			run_size += image->sections[++section_last].size;
			run_size += pad_bytes;

			LOG_INFO("Padding image section %d with %d bytes", section_last-1, pad_bytes);
		}

		/* fit the run into bank constraints */
		if (run_address + run_size - 1 > c->base + c->size - 1)
		{
			/* REVISIT isn't this superfluous, given the while()
			 * loop conditions above??
			 */
			LOG_WARNING("writing %d bytes only - as image section is %d bytes and bank is only %d bytes", \
				    (int)(c->base + c->size - run_address), (int)(run_size), (int)(c->size));
			run_size = c->base + c->size - run_address;
		}

		/* If we're applying any sector automagic, then pad this
		 * (maybe-combined) segment to the end of its last sector.
		 */
		if (unlock || erase) {
			int sector;
			uint32_t offset_start = run_address - c->base;
			uint32_t offset_end = offset_start + run_size;
			uint32_t end = offset_end, delta;

			for (sector = 0; sector < c->num_sectors; sector++) {
				end = c->sectors[sector].offset
						+ c->sectors[sector].size;
				if (offset_end <= end)
					break;
			}

			delta = end - offset_end;
			padding[section_last] += delta;
			run_size += delta;
		}

		/* allocate buffer */
		buffer = malloc(run_size);
		buffer_size = 0;

		/* read sections to the buffer */
		while (buffer_size < run_size)
		{
			size_t size_read;

			size_read = run_size - buffer_size;
			if (size_read > image->sections[section].size - section_offset)
			    size_read = image->sections[section].size - section_offset;

			if ((retval = image_read_section(image, section, section_offset,
					size_read, buffer + buffer_size, &size_read)) != ERROR_OK || size_read == 0)
			{
				free(buffer);
				free(padding);
				return retval;
			}

			/* see if we need to pad the section */
			while (padding[section]--)
				 (buffer + buffer_size)[size_read++] = 0xff;

			buffer_size += size_read;
			section_offset += size_read;

			if (section_offset >= image->sections[section].size)
			{
				section++;
				section_offset = 0;
			}
		}

		retval = ERROR_OK;

		if (unlock)
		{
			retval = flash_unlock_address_range(target, run_address, run_size);
		}
		if (retval == ERROR_OK)
		{
			if (erase)
			{
				/* calculate and erase sectors */
				retval = flash_erase_address_range(target,
						do_pad, run_address, run_size);
			}
		}

		if (retval == ERROR_OK)
		{
			/* write flash sectors */
			retval = flash_driver_write(c, buffer, run_address - c->base, run_size);
		}

		free(buffer);

		if (retval != ERROR_OK)
		{
			free(padding);
			return retval; /* abort operation */
		}

		if (written != NULL)
			*written += run_size; /* add run size to total written counter */
	}

	free(padding);

	return retval;
}

int flash_write(struct target *target, struct image *image,
		uint32_t *written, int erase)
{
	return flash_write_unlock(target, image, written, erase, false);
}
