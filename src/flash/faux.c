/***************************************************************************
 *   Copyright (C) 2009 Øyvind Harboe                                      *
 *   oyvind.harboe@zylin.com                                               *
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
#include "image.h"


typedef struct faux_flash_bank_s
{
	struct target_s *target;
	uint8_t *memory;
	uint32_t start_address;
} faux_flash_bank_t;

static const int sectorSize = 0x10000;


/* flash bank faux <base> <size> <chip_width> <bus_width> <target#> <driverPath>
 */
FLASH_BANK_COMMAND_HANDLER(faux_flash_bank_command)
{
	faux_flash_bank_t *info;

	if (argc < 6)
	{
		LOG_WARNING("incomplete flash_bank faux configuration");
		return ERROR_FLASH_BANK_INVALID;
	}

	info = malloc(sizeof(faux_flash_bank_t));
	if (info == NULL)
	{
		LOG_ERROR("no memory for flash bank info");
		return ERROR_FAIL;
	}
	info->memory = malloc(bank->size);
	if (info == NULL)
	{
		free(info);
		LOG_ERROR("no memory for flash bank info");
		return ERROR_FAIL;
	}
	bank->driver_priv = info;

	/* Use 0x10000 as a fixed sector size. */
	int i = 0;
	uint32_t offset = 0;
	bank->num_sectors = bank->size/sectorSize;
	bank->sectors = malloc(sizeof(flash_sector_t) * bank->num_sectors);
	for (i = 0; i < bank->num_sectors; i++)
	{
		bank->sectors[i].offset = offset;
		bank->sectors[i].size = sectorSize;
		offset += bank->sectors[i].size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
	}

	info->target = get_target(args[5]);
	if (info->target == NULL)
	{
		LOG_ERROR("target '%s' not defined", args[5]);
		free(info->memory);
		free(info);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int faux_register_commands(struct command_context_s *cmd_ctx)
{
	return ERROR_OK;
}

static int faux_erase(struct flash_bank_s *bank, int first, int last)
{
	faux_flash_bank_t *info = bank->driver_priv;
	memset(info->memory + first*sectorSize, 0xff, sectorSize*(last-first + 1));
	return ERROR_OK;
}

static int faux_protect(struct flash_bank_s *bank, int set, int first, int last)
{
	LOG_USER("set protection sector %d to %d to %s", first, last, set?"on":"off");
	return ERROR_OK;
}

static int faux_write(struct flash_bank_s *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
{
	faux_flash_bank_t *info = bank->driver_priv;
	memcpy(info->memory + offset, buffer, count);
	return ERROR_OK;
}

static int faux_protect_check(struct flash_bank_s *bank)
{
	return ERROR_OK;
}

static int faux_info(struct flash_bank_s *bank, char *buf, int buf_size)
{
	snprintf(buf, buf_size, "faux flash driver");
	return ERROR_OK;
}

static int faux_probe(struct flash_bank_s *bank)
{
	return ERROR_OK;
}

flash_driver_t faux_flash = {
		.name = "faux",
		.register_commands = &faux_register_commands,
		.flash_bank_command = &faux_flash_bank_command,
		.erase = &faux_erase,
		.protect = &faux_protect,
		.write = &faux_write,
		.probe = &faux_probe,
		.auto_probe = &faux_probe,
		.erase_check = &default_flash_blank_check,
		.protect_check = &faux_protect_check,
		.info = &faux_info
	};
