/***************************************************************************
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
#include <flash/flash.h>
#include <flash/nor/imp.h>
#include <target/image.h>

// in flash.c, to be moved here
extern struct flash_bank *flash_banks;

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

/* erase given flash region, selects proper bank according to target and address */
static int flash_iterate_address_range(struct target *target, uint32_t addr, uint32_t length,
		int (*callback)(struct flash_bank *bank, int first, int last))
{
	struct flash_bank *c;
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
			return ERROR_FLASH_DST_BREAKS_ALIGNMENT;

		return callback(c, 0, c->num_sectors - 1);
	}

	/* check whether it fits */
	if (addr + length - 1 > c->base + c->size - 1)
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;

	addr -= c->base;

	for (i = 0; i < c->num_sectors; i++)
	{
		/* check whether sector overlaps with the given range and is not yet erased */
		if (addr < c->sectors[i].offset + c->sectors[i].size && addr + length > c->sectors[i].offset && c->sectors[i].is_erased != 1) {
			/* if first is not set yet then this is the first sector */
			if (first == -1)
				first = i;
			last = i; /* and it is the last one so far in any case */
		}
	}

	if (first == -1 || last == -1)
		return ERROR_OK;

	return callback(c, first, last);
}

int flash_erase_address_range(struct target *target, uint32_t addr, uint32_t length)
{
	return flash_iterate_address_range(target,
			addr, length, &flash_driver_erase);
}

static int flash_driver_unprotect(struct flash_bank *bank, int first, int last)
{
	return flash_driver_protect(bank, 0, first, last);
}

static int flash_unlock_address_range(struct target *target, uint32_t addr, uint32_t length)
{
	return flash_iterate_address_range(target,
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
	padding = malloc(image->num_sections * sizeof(padding));

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
				LOG_DEBUG("section %d out of order(very slightly surprising, but supported)", section_last + 1);
				break;
			}
			/* if we have multiple sections within our image, flash programming could fail due to alignment issues
			 * attempt to rebuild a consecutive buffer for the flash loader */
			pad_bytes = (image->sections[section_last + 1].base_address) - (run_address + run_size);
			if ((run_address + run_size + pad_bytes) > (c->base + c->size))
				break;
			padding[section_last] = pad_bytes;
			run_size += image->sections[++section_last].size;
			run_size += pad_bytes;
			padding[section_last] = 0;

			LOG_INFO("Padding image section %d with %d bytes", section_last-1, pad_bytes);
		}

		/* fit the run into bank constraints */
		if (run_address + run_size - 1 > c->base + c->size - 1)
		{
			LOG_WARNING("writing %d bytes only - as image section is %d bytes and bank is only %d bytes", \
				    (int)(c->base + c->size - run_address), (int)(run_size), (int)(c->size));
			run_size = c->base + c->size - run_address;
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
				retval = flash_erase_address_range(target, run_address, run_size);
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
