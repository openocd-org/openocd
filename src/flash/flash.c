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
#include "image.h"
#include "time_support.h"

/* command handlers */
static int handle_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_flash_info_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_flash_probe_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_flash_erase_check_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_flash_erase_address_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_flash_protect_check_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_flash_erase_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_flash_write_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_flash_write_image_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_flash_fill_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_flash_protect_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int flash_write_unlock(target_t *target, image_t *image, uint32_t *written, int erase, bool unlock);

/* flash drivers
 */
extern flash_driver_t lpc2000_flash;
extern flash_driver_t lpc288x_flash;
extern flash_driver_t lpc2900_flash;
extern flash_driver_t cfi_flash;
extern flash_driver_t at91sam3_flash;
extern flash_driver_t at91sam7_flash;
extern flash_driver_t str7x_flash;
extern flash_driver_t str9x_flash;
extern flash_driver_t aduc702x_flash;
extern flash_driver_t stellaris_flash;
extern flash_driver_t str9xpec_flash;
extern flash_driver_t stm32x_flash;
extern flash_driver_t tms470_flash;
extern flash_driver_t ecosflash_flash;
extern flash_driver_t ocl_flash;
extern flash_driver_t pic32mx_flash;
extern flash_driver_t avr_flash;
extern flash_driver_t faux_flash;

flash_driver_t *flash_drivers[] = {
	&lpc2000_flash,
	&lpc288x_flash,
	&lpc2900_flash,
	&cfi_flash,
	&at91sam7_flash,
	&at91sam3_flash,
	&str7x_flash,
	&str9x_flash,
	&aduc702x_flash,
	&stellaris_flash,
	&str9xpec_flash,
	&stm32x_flash,
	&tms470_flash,
	&ecosflash_flash,
	&ocl_flash,
	&pic32mx_flash,
	&avr_flash,
	&faux_flash,
	NULL,
};

flash_bank_t *flash_banks;
static 	command_t *flash_cmd;

/* wafer thin wrapper for invoking the flash driver */
static int flash_driver_write(struct flash_bank_s *bank, uint8_t *buffer, uint32_t offset, uint32_t count)
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

static int flash_driver_erase(struct flash_bank_s *bank, int first, int last)
{
	int retval;

	retval = bank->driver->erase(bank, first, last);
	if (retval != ERROR_OK)
	{
		LOG_ERROR("failed erasing sectors %d to %d (%d)", first, last, retval);
	}

	return retval;
}

int flash_driver_protect(struct flash_bank_s *bank, int set, int first, int last)
{
	int retval;

	retval = bank->driver->protect(bank, set, first, last);
	if (retval != ERROR_OK)
	{
		LOG_ERROR("failed setting protection for areas %d to %d (%d)", first, last, retval);
	}

	return retval;
}

int flash_register_commands(struct command_context_s *cmd_ctx)
{
	flash_cmd = register_command(cmd_ctx, NULL, "flash", NULL, COMMAND_ANY, NULL);

	register_command(cmd_ctx, flash_cmd, "bank", handle_flash_bank_command, COMMAND_CONFIG, "flash bank <driver> <base> <size> <chip_width> <bus_width> <target> [driver_options ...]");
	return ERROR_OK;
}

static int jim_flash_banks(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	flash_bank_t *p;

	if (argc != 1) {
		Jim_WrongNumArgs(interp, 1, argv, "no arguments to flash_banks command");
		return JIM_ERR;
	}

	Jim_Obj *list = Jim_NewListObj(interp, NULL, 0);
	for (p = flash_banks; p; p = p->next)
	{
		Jim_Obj *elem = Jim_NewListObj(interp, NULL, 0);

		Jim_ListAppendElement(interp, elem, Jim_NewStringObj(interp, "name", -1));
		Jim_ListAppendElement(interp, elem, Jim_NewStringObj(interp, p->driver->name, -1));
		Jim_ListAppendElement(interp, elem, Jim_NewStringObj(interp, "base", -1));
		Jim_ListAppendElement(interp, elem, Jim_NewIntObj(interp, p->base));
		Jim_ListAppendElement(interp, elem, Jim_NewStringObj(interp, "size", -1));
		Jim_ListAppendElement(interp, elem, Jim_NewIntObj(interp, p->size));
		Jim_ListAppendElement(interp, elem, Jim_NewStringObj(interp, "bus_width", -1));
		Jim_ListAppendElement(interp, elem, Jim_NewIntObj(interp, p->bus_width));
		Jim_ListAppendElement(interp, elem, Jim_NewStringObj(interp, "chip_width", -1));
		Jim_ListAppendElement(interp, elem, Jim_NewIntObj(interp, p->chip_width));

		Jim_ListAppendElement(interp, list, elem);
	}

	Jim_SetResult(interp, list);

	return JIM_OK;
}

int flash_init_drivers(struct command_context_s *cmd_ctx)
{
	register_jim(cmd_ctx, "ocd_flash_banks", jim_flash_banks, "return information about the flash banks");

	if (flash_banks)
	{
		register_command(cmd_ctx, flash_cmd, "info", handle_flash_info_command, COMMAND_EXEC,
						 "print info about flash bank <num>");
		register_command(cmd_ctx, flash_cmd, "probe", handle_flash_probe_command, COMMAND_EXEC,
						 "identify flash bank <num>");
		register_command(cmd_ctx, flash_cmd, "erase_check", handle_flash_erase_check_command, COMMAND_EXEC,
						 "check erase state of sectors in flash bank <num>");
		register_command(cmd_ctx, flash_cmd, "protect_check", handle_flash_protect_check_command, COMMAND_EXEC,
						 "check protection state of sectors in flash bank <num>");
		register_command(cmd_ctx, flash_cmd, "erase_sector", handle_flash_erase_command, COMMAND_EXEC,
						 "erase sectors at <bank> <first> <last>");
		register_command(cmd_ctx, flash_cmd, "erase_address", handle_flash_erase_address_command, COMMAND_EXEC,
						 "erase address range <address> <length>");

		register_command(cmd_ctx, flash_cmd, "fillw", handle_flash_fill_command, COMMAND_EXEC,
						 "fill with pattern (no autoerase) <address> <word_pattern> <count>");
		register_command(cmd_ctx, flash_cmd, "fillh", handle_flash_fill_command, COMMAND_EXEC,
						 "fill with pattern <address> <halfword_pattern> <count>");
		register_command(cmd_ctx, flash_cmd, "fillb", handle_flash_fill_command, COMMAND_EXEC,
						 "fill with pattern <address> <byte_pattern> <count>");

		register_command(cmd_ctx, flash_cmd, "write_bank", handle_flash_write_bank_command, COMMAND_EXEC,
						 "write binary data to <bank> <file> <offset>");
		register_command(cmd_ctx, flash_cmd, "write_image", handle_flash_write_image_command, COMMAND_EXEC,
						 "write_image [erase] [unlock] <file> [offset] [type]");
		register_command(cmd_ctx, flash_cmd, "protect", handle_flash_protect_command, COMMAND_EXEC,
						 "set protection of sectors at <bank> <first> <last> <on | off>");
	}

	return ERROR_OK;
}

flash_bank_t *get_flash_bank_by_num_noprobe(int num)
{
	flash_bank_t *p;
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
	flash_bank_t *p;
	int i = 0;
	for (p = flash_banks; p; p = p->next)
	{
		i++;
	}
	return i;
}

flash_bank_t *get_flash_bank_by_num(int num)
{
	flash_bank_t *p = get_flash_bank_by_num_noprobe(num);
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

static int handle_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int retval;
	int i;
	int found = 0;
	target_t *target;

	if (argc < 6)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if ((target = get_target(args[5])) == NULL)
	{
		LOG_ERROR("target '%s' not defined", args[5]);
		return ERROR_FAIL;
	}

	for (i = 0; flash_drivers[i]; i++)
	{
		if (strcmp(args[0], flash_drivers[i]->name) == 0)
		{
			flash_bank_t *p, *c;

			/* register flash specific commands */
			if (flash_drivers[i]->register_commands(cmd_ctx) != ERROR_OK)
			{
				LOG_ERROR("couldn't register '%s' commands", args[0]);
				return ERROR_FAIL;
			}

			c = malloc(sizeof(flash_bank_t));
			c->target = target;
			c->driver = flash_drivers[i];
			c->driver_priv = NULL;
			c->base = strtoul(args[1], NULL, 0);
			c->size = strtoul(args[2], NULL, 0);
			c->chip_width = strtoul(args[3], NULL, 0);
			c->bus_width = strtoul(args[4], NULL, 0);
			c->num_sectors = 0;
			c->sectors = NULL;
			c->next = NULL;

			if ((retval = flash_drivers[i]->flash_bank_command(cmd_ctx, cmd, args, argc, c)) != ERROR_OK)
			{
				LOG_ERROR("'%s' driver rejected flash bank at 0x%8.8" PRIx32 , args[0], c->base);
				free(c);
				return retval;
			}

			/* put flash bank in linked list */
			if (flash_banks)
			{
				int	bank_num = 0;
				/* find last flash bank */
				for (p = flash_banks; p && p->next; p = p->next) bank_num++;
				if (p)
					p->next = c;
				c->bank_number = bank_num + 1;
			}
			else
			{
				flash_banks = c;
				c->bank_number = 0;
			}

			found = 1;
		}
	}

	/* no matching flash driver found */
	if (!found)
	{
		LOG_ERROR("flash driver '%s' not found", args[0]);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int handle_flash_info_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *p;
	uint32_t i = 0;
	int j = 0;
	int retval;

	if (argc != 1)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	for (p = flash_banks; p; p = p->next, i++)
	{
		if (i == strtoul(args[0], NULL, 0))
		{
			char buf[1024];

			/* attempt auto probe */
			if ((retval = p->driver->auto_probe(p)) != ERROR_OK)
				return retval;

			command_print(cmd_ctx,
				      "#%" PRIi32 " : %s at 0x%8.8" PRIx32 ", size 0x%8.8" PRIx32 ", buswidth %i, chipwidth %i",
				      i,
				      p->driver->name,
				      p->base,
				      p->size,
				      p->bus_width,
				      p->chip_width);
			for (j = 0; j < p->num_sectors; j++)
			{
				char *protect_state;

				if (p->sectors[j].is_protected == 0)
					protect_state = "not protected";
				else if (p->sectors[j].is_protected == 1)
					protect_state = "protected";
				else
					protect_state = "protection state unknown";

				command_print(cmd_ctx,
					      "\t#%3i: 0x%8.8" PRIx32 " (0x%" PRIx32 " %" PRIi32 "kB) %s",
					      j,
					      p->sectors[j].offset,
					      p->sectors[j].size,
					      p->sectors[j].size >> 10,
					      protect_state);
			}

			*buf = '\0'; /* initialize buffer, otherwise it migh contain garbage if driver function fails */
			retval = p->driver->info(p, buf, sizeof(buf));
			command_print(cmd_ctx, "%s", buf);
			if (retval != ERROR_OK)
				LOG_ERROR("error retrieving flash info (%d)", retval);
		}
	}

	return ERROR_OK;
}

static int handle_flash_probe_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *p;
	int retval;

	if (argc != 1)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	p = get_flash_bank_by_num_noprobe(strtoul(args[0], NULL, 0));
	if (p)
	{
		if ((retval = p->driver->probe(p)) == ERROR_OK)
		{
			command_print(cmd_ctx, "flash '%s' found at 0x%8.8" PRIx32, p->driver->name, p->base);
		}
		else if (retval == ERROR_FLASH_BANK_INVALID)
		{
			command_print(cmd_ctx, "probing failed for flash bank '#%s' at 0x%8.8" PRIx32,
						  args[0], p->base);
		}
		else
		{
			command_print(cmd_ctx, "unknown error when probing flash bank '#%s' at 0x%8.8" PRIx32,
						  args[0], p->base);
		}
	}
	else
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
	}

	return ERROR_OK;
}

static int handle_flash_erase_check_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *p;
	int retval;

	if (argc != 1)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	p = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	if (p)
	{
		int j;
		if ((retval = p->driver->erase_check(p)) == ERROR_OK)
		{
			command_print(cmd_ctx, "successfully checked erase state");
		}
		else
		{
			command_print(cmd_ctx, "unknown error when checking erase state of flash bank #%s at 0x%8.8" PRIx32,
				args[0], p->base);
		}

		for (j = 0; j < p->num_sectors; j++)
		{
			char *erase_state;

			if (p->sectors[j].is_erased == 0)
				erase_state = "not erased";
			else if (p->sectors[j].is_erased == 1)
				erase_state = "erased";
			else
				erase_state = "erase state unknown";

			command_print(cmd_ctx,
				      "\t#%3i: 0x%8.8" PRIx32 " (0x%" PRIx32 " %" PRIi32 "kB) %s",
				      j,
				      p->sectors[j].offset,
				      p->sectors[j].size,
				      p->sectors[j].size >> 10,
				      erase_state);
		}
	}

	return ERROR_OK;
}

static int handle_flash_erase_address_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *p;
	int retval;
	int address;
	int length;
	duration_t duration;
	char *duration_text;

	target_t *target = get_current_target(cmd_ctx);

	if (argc != 2)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	address = strtoul(args[0], NULL, 0);
	length = strtoul(args[1], NULL, 0);
	if (length <= 0)
	{
		command_print(cmd_ctx, "Length must be >0");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	p = get_flash_bank_by_addr(target, address);
	if (p == NULL)
	{
		return ERROR_FAIL;
	}

	/* We can't know if we did a resume + halt, in which case we no longer know the erased state */
	flash_set_dirty();

	duration_start_measure(&duration);

	if ((retval = flash_erase_address_range(target, address, length)) == ERROR_OK)
	{
		if ((retval = duration_stop_measure(&duration, &duration_text)) != ERROR_OK)
		{
			return retval;
		}
		command_print(cmd_ctx, "erased address 0x%8.8x length %i in %s", address, length, duration_text);
		free(duration_text);
	}

	return retval;
}

static int handle_flash_protect_check_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *p;
	int retval;

	if (argc != 1)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	p = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	if (p)
	{
		if ((retval = p->driver->protect_check(p)) == ERROR_OK)
		{
			command_print(cmd_ctx, "successfully checked protect state");
		}
		else if (retval == ERROR_FLASH_OPERATION_FAILED)
		{
			command_print(cmd_ctx, "checking protection state failed (possibly unsupported) by flash #%s at 0x%8.8" PRIx32, args[0], p->base);
		}
		else
		{
			command_print(cmd_ctx, "unknown error when checking protection state of flash bank '#%s' at 0x%8.8" PRIx32, args[0], p->base);
		}
	}
	else
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}

static int flash_check_sector_parameters(struct command_context_s *cmd_ctx,
		uint32_t first, uint32_t last, uint32_t num_sectors)
{
	if (!(first <= last)) {
		command_print(cmd_ctx, "ERROR: "
				"first sector must be <= last sector");
		return ERROR_FAIL;
	}

	if (!(last <= (num_sectors - 1))) {
		command_print(cmd_ctx, "ERROR: last sector must be <= %d",
				(int) num_sectors - 1);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int handle_flash_erase_command(struct command_context_s *cmd_ctx,
		char *cmd, char **args, int argc)
{
	if (argc > 2)
	{
		uint32_t bank_nr;
		uint32_t first;
		uint32_t last;
		int retval;

		if ((retval = parse_u32(args[0], &bank_nr)) != ERROR_OK)
			return retval;

		flash_bank_t *p = get_flash_bank_by_num(bank_nr);
		if (!p)
			return ERROR_OK;

		if ((retval = parse_u32(args[1], &first)) != ERROR_OK)
			return retval;
		if (strcmp(args[2], "last") == 0)
			last = p->num_sectors - 1;
		else
			if ((retval = parse_u32(args[2], &last)) != ERROR_OK)
				return retval;

		if ((retval = flash_check_sector_parameters(cmd_ctx,
				first, last, p->num_sectors)) != ERROR_OK)
			return retval;

		duration_t duration;
		char *duration_text;
		duration_start_measure(&duration);

		if ((retval = flash_driver_erase(p, first, last)) == ERROR_OK) {
			if ((retval = duration_stop_measure(&duration,
						&duration_text)) != ERROR_OK)
				return retval;
			command_print(cmd_ctx, "erased sectors %i through %i "
					"on flash bank %i in %s",
				(int) first, (int) last, (int) bank_nr,
				duration_text);
			free(duration_text);
		}
	}
	else
		return ERROR_COMMAND_SYNTAX_ERROR;

	return ERROR_OK;
}

static int handle_flash_protect_command(struct command_context_s *cmd_ctx,
		char *cmd, char **args, int argc)
{
	if (argc > 3)
	{
		uint32_t bank_nr;
		uint32_t first;
		uint32_t last;
		int retval;
		int set;

		if ((retval = parse_u32(args[0], &bank_nr)) != ERROR_OK)
			return retval;

		flash_bank_t *p = get_flash_bank_by_num(bank_nr);
		if (!p)
			return ERROR_OK;

		if ((retval = parse_u32(args[1], &first)) != ERROR_OK)
			return retval;
		if (strcmp(args[2], "last") == 0)
			last = p->num_sectors - 1;
		else
			if ((retval = parse_u32(args[2], &last)) != ERROR_OK)
				return retval;

		if (strcmp(args[3], "on") == 0)
			set = 1;
		else if (strcmp(args[3], "off") == 0)
			set = 0;
		else
			return ERROR_COMMAND_SYNTAX_ERROR;

		if ((retval = flash_check_sector_parameters(cmd_ctx,
				first, last, p->num_sectors)) != ERROR_OK)
			return retval;

		retval = flash_driver_protect(p, set, first, last);
		if (retval == ERROR_OK) {
			command_print(cmd_ctx, "%s protection for sectors %i "
					"through %i on flash bank %i",
				(set) ? "set" : "cleared", (int) first,
				(int) last, (int) bank_nr);
		}
	}
	else
		return ERROR_COMMAND_SYNTAX_ERROR;

	return ERROR_OK;
}

static int handle_flash_write_image_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);

	image_t image;
	uint32_t written;

	duration_t duration;
	char *duration_text;

	int retval, retvaltemp;

	if (argc < 1)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* flash auto-erase is disabled by default*/
	int auto_erase = 0;
	bool auto_unlock = false;

	for (;;)
	{
		if (strcmp(args[0], "erase") == 0)
		{
			auto_erase = 1;
			args++;
			argc--;
			command_print(cmd_ctx, "auto erase enabled");
		} else if (strcmp(args[0], "unlock") == 0)
		{
			auto_unlock = true;
			args++;
			argc--;
			command_print(cmd_ctx, "auto unlock enabled");
		} else
		{
			break;
		}
	}

	if (argc < 1)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (!target)
	{
		LOG_ERROR("no target selected");
		return ERROR_FAIL;
	}

	duration_start_measure(&duration);

	if (argc >= 2)
	{
		image.base_address_set = 1;
		image.base_address = strtoul(args[1], NULL, 0);
	}
	else
	{
		image.base_address_set = 0;
		image.base_address = 0x0;
	}

	image.start_address_set = 0;

	retval = image_open(&image, args[0], (argc == 3) ? args[2] : NULL);
	if (retval != ERROR_OK)
	{
		return retval;
	}

	retval = flash_write_unlock(target, &image, &written, auto_erase, auto_unlock);
	if (retval != ERROR_OK)
	{
		image_close(&image);
		return retval;
	}

	if ((retvaltemp = duration_stop_measure(&duration, &duration_text)) != ERROR_OK)
	{
		image_close(&image);
		return retvaltemp;
	}

	float speed;

	speed = written / 1024.0;
	speed /= ((float)duration.duration.tv_sec
			+ ((float)duration.duration.tv_usec / 1000000.0));
	command_print(cmd_ctx,
			"wrote %" PRIu32 " byte from file %s in %s (%f kb/s)",
			written, args[0], duration_text, speed);

	free(duration_text);

	image_close(&image);

	return retval;
}

static int handle_flash_fill_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int err = ERROR_OK, retval;
	uint32_t address;
	uint32_t pattern;
	uint32_t count;
	uint8_t chunk[1024];
	uint8_t readback[1024];
	uint32_t wrote = 0;
	uint32_t cur_size = 0;
	uint32_t chunk_count;
	char *duration_text;
	duration_t duration;
	target_t *target = get_current_target(cmd_ctx);
	uint32_t i;
	uint32_t wordsize;

	if (argc != 3)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	address	= strtoul(args[0], NULL, 0);
	pattern	= strtoul(args[1], NULL, 0);
	count 	= strtoul(args[2], NULL, 0);

	if (count == 0)
		return ERROR_OK;

	switch (cmd[4])
	{
	case 'w':
		wordsize = 4;
		break;
	case 'h':
		wordsize = 2;
		break;
	case 'b':
		wordsize = 1;
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	chunk_count = MIN(count, (1024 / wordsize));
	switch (wordsize)
	{
	case 4:
		for (i = 0; i < chunk_count; i++)
		{
			target_buffer_set_u32(target, chunk + i * wordsize, pattern);
		}
		break;
	case 2:
		for (i = 0; i < chunk_count; i++)
		{
			target_buffer_set_u16(target, chunk + i * wordsize, pattern);
		}
		break;
	case 1:
		memset(chunk, pattern, chunk_count);
		break;
	default:
		LOG_ERROR("BUG: can't happen");
		exit(-1);
	}

	duration_start_measure(&duration);

	for (wrote = 0; wrote < (count*wordsize); wrote += cur_size)
	{
		cur_size = MIN((count*wordsize - wrote), sizeof(chunk));
		flash_bank_t *bank;
		bank = get_flash_bank_by_addr(target, address);
		if (bank == NULL)
		{
			return ERROR_FAIL;
		}
		err = flash_driver_write(bank, chunk, address - bank->base + wrote, cur_size);
		if (err != ERROR_OK)
			return err;

		err = target_read_buffer(target, address + wrote, cur_size, readback);
		if (err != ERROR_OK)
			return err;

		unsigned i;
		for (i = 0; i < cur_size; i++)
		{
			if (readback[i]!=chunk[i])
			{
				LOG_ERROR("Verfication error address 0x%08" PRIx32 ", read back 0x%02x, expected 0x%02x",
						  address + wrote + i, readback[i], chunk[i]);
				return ERROR_FAIL;
			}
		}

	}

	if ((retval = duration_stop_measure(&duration, &duration_text)) != ERROR_OK)
	{
		return retval;
	}

	float speed;

	speed = wrote / 1024.0;
	speed /= ((float)duration.duration.tv_sec
			+ ((float)duration.duration.tv_usec / 1000000.0));
	command_print(cmd_ctx,
			"wrote %" PRIu32 " bytes to 0x%8.8" PRIx32 " in %s (%f kb/s)",
			wrote, address, duration_text, speed);

	free(duration_text);
	return ERROR_OK;
}

static int handle_flash_write_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	uint32_t offset;
	uint8_t *buffer;
	uint32_t buf_cnt;

	fileio_t fileio;

	duration_t duration;
	char *duration_text;

	int retval, retvaltemp;
	flash_bank_t *p;

	if (argc != 3)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	duration_start_measure(&duration);

	offset = strtoul(args[2], NULL, 0);
	p = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	if (!p)
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
		return ERROR_OK;
	}

	if (fileio_open(&fileio, args[1], FILEIO_READ, FILEIO_BINARY) != ERROR_OK)
	{
		return ERROR_OK;
	}

	buffer = malloc(fileio.size);
	if (fileio_read(&fileio, fileio.size, buffer, &buf_cnt) != ERROR_OK)
	{
		free(buffer);
		fileio_close(&fileio);
		return ERROR_OK;
	}

	retval = flash_driver_write(p, buffer, offset, buf_cnt);

	free(buffer);
	buffer = NULL;

	if ((retvaltemp = duration_stop_measure(&duration, &duration_text)) != ERROR_OK)
	{
		fileio_close(&fileio);
		return retvaltemp;
	}
	if (retval == ERROR_OK)
	{
	command_print(cmd_ctx,
				  "wrote  %lld byte from file %s to flash bank %li at offset 0x%8.8" PRIx32 " in %s (%f kb/s)",
				  fileio.size,
				  args[1],
				  strtoul(args[0], NULL, 0),
				  offset,
				  duration_text,
				  (float)fileio.size / 1024.0 / ((float)duration.duration.tv_sec + ((float)duration.duration.tv_usec / 1000000.0)));
	}
	free(duration_text);

	fileio_close(&fileio);

	return retval;
}

void flash_set_dirty(void)
{
	flash_bank_t *c;
	int i;

	/* set all flash to require erasing */
	for (c = flash_banks; c; c = c->next)
	{
		for (i = 0; i < c->num_sectors; i++)
		{
			c->sectors[i].is_erased = 0;
		}
	}
}

/* lookup flash bank by address */
flash_bank_t *get_flash_bank_by_addr(target_t *target, uint32_t addr)
{
	flash_bank_t *c;

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

/* erase given flash region, selects proper bank according to target and address */
static int flash_iterate_address_range(target_t *target, uint32_t addr, uint32_t length,
		int (*callback)(struct flash_bank_s *bank, int first, int last))
{
	flash_bank_t *c;
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



int flash_erase_address_range(target_t *target, uint32_t addr, uint32_t length)
{
	return flash_iterate_address_range(target, addr, length, &flash_driver_erase);
}

static int flash_driver_unprotect(struct flash_bank_s *bank, int first, int last)
{
	return flash_driver_protect(bank, 0, first, last);
}

static int flash_unlock_address_range(target_t *target, uint32_t addr, uint32_t length)
{
	return flash_iterate_address_range(target, addr, length, &flash_driver_unprotect);
}


/* write (optional verify) an image to flash memory of the given target */
static int flash_write_unlock(target_t *target, image_t *image, uint32_t *written, int erase, bool unlock)
{
	int retval = ERROR_OK;

	int section;
	uint32_t section_offset;
	flash_bank_t *c;
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
			uint32_t size_read;

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

int flash_write(target_t *target, image_t *image, uint32_t *written, int erase)
{
	return flash_write_unlock(target, image, written, erase, false);
}

int default_flash_mem_blank_check(struct flash_bank_s *bank)
{
	target_t *target = bank->target;
	uint8_t buffer[1024];
	int buffer_size = sizeof(buffer);
	int i;
	uint32_t nBytes;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	for (i = 0; i < bank->num_sectors; i++)
	{
		uint32_t j;
		bank->sectors[i].is_erased = 1;

		for (j = 0; j < bank->sectors[i].size; j += buffer_size)
		{
			uint32_t chunk;
			int retval;
			chunk = buffer_size;
			if (chunk > (j - bank->sectors[i].size))
			{
				chunk = (j - bank->sectors[i].size);
			}

			retval = target_read_memory(target, bank->base + bank->sectors[i].offset + j, 4, chunk/4, buffer);
			if (retval != ERROR_OK)
				return retval;

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

	return ERROR_OK;
}

int default_flash_blank_check(struct flash_bank_s *bank)
{
	target_t *target = bank->target;
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
