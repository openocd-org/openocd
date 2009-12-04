/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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
#include "config.h"
#endif

#include "flash.h"
#include "common.h"
#include <target/image.h>
#include <helper/time_support.h>

struct flash_bank *flash_banks;

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
