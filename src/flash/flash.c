/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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
#include "command.h"
#include "log.h"
#include "target.h"
#include "time_support.h"

#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>

#include <fileio.h>

/* command handlers */
int handle_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_flash_banks_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_flash_info_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_flash_probe_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_flash_erase_check_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_flash_protect_check_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_flash_erase_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_flash_write_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_flash_protect_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

/* flash drivers
 */
extern flash_driver_t lpc2000_flash;
extern flash_driver_t cfi_flash;
extern flash_driver_t at91sam7_flash;
extern flash_driver_t str7x_flash;
extern flash_driver_t str9x_flash;

flash_driver_t *flash_drivers[] =
{
	&lpc2000_flash,
	&cfi_flash,
	&at91sam7_flash,
	&str7x_flash,
	&str9x_flash,
	NULL,
};

flash_bank_t *flash_banks;
static 	command_t *flash_cmd;

int flash_register_commands(struct command_context_s *cmd_ctx)
{
	flash_cmd = register_command(cmd_ctx, NULL, "flash", NULL, COMMAND_ANY, NULL);
	
	register_command(cmd_ctx, flash_cmd, "bank", handle_flash_bank_command, COMMAND_CONFIG, NULL);
	
	return ERROR_OK;
}

int flash_init(struct command_context_s *cmd_ctx)
{
	if (flash_banks)
	{
		register_command(cmd_ctx, flash_cmd, "banks", handle_flash_banks_command, COMMAND_EXEC,
						 "list configured flash banks ");
		register_command(cmd_ctx, flash_cmd, "info", handle_flash_info_command, COMMAND_EXEC,
						 "print info about flash bank <num>");
		register_command(cmd_ctx, flash_cmd, "probe", handle_flash_probe_command, COMMAND_EXEC,
						 "identify flash bank <num>");
		register_command(cmd_ctx, flash_cmd, "erase_check", handle_flash_erase_check_command, COMMAND_EXEC,
						 "check erase state of sectors in flash bank <num>");
		register_command(cmd_ctx, flash_cmd, "protect_check", handle_flash_protect_check_command, COMMAND_EXEC,
						 "check protection state of sectors in flash bank <num>");
		register_command(cmd_ctx, flash_cmd, "erase", handle_flash_erase_command, COMMAND_EXEC,
						 "erase sectors at <bank> <first> <last>");
		register_command(cmd_ctx, flash_cmd, "write", handle_flash_write_command, COMMAND_EXEC,
						 "write binary <bank> <file> <offset>");
		register_command(cmd_ctx, flash_cmd, "protect", handle_flash_protect_command, COMMAND_EXEC,
						 "set protection of sectors at <bank> <first> <last> <on|off>");
	}
	
	return ERROR_OK;
}

flash_bank_t *get_flash_bank_by_num(int num)
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
	
	return NULL;
}

/* flash_bank <driver> <base> <size> <chip_width> <bus_width> [driver_options ...]
 */
int handle_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int i;
	int found = 0;
		
	if (argc < 5)
	{
		WARNING("incomplete flash_bank configuration");
		return ERROR_OK;
	}
	
	for (i = 0; flash_drivers[i]; i++)
	{
		if (strcmp(args[0], flash_drivers[i]->name) == 0)
		{
			flash_bank_t *p, *c;
			
			/* register flash specific commands */
			if (flash_drivers[i]->register_commands(cmd_ctx) != ERROR_OK)
			{
				ERROR("couldn't register '%s' commands", args[0]);
				exit(-1);
			}
			
			c = malloc(sizeof(flash_bank_t));
			c->driver = flash_drivers[i];
			c->driver_priv = NULL;
			c->base = strtoul(args[1], NULL, 0);
			c->size = strtoul(args[2], NULL, 0);
			c->chip_width = strtoul(args[3], NULL, 0);
			c->bus_width = strtoul(args[4], NULL, 0);
			c->next = NULL;
			
			if (flash_drivers[i]->flash_bank_command(cmd_ctx, cmd, args, argc, c) != ERROR_OK)
			{
				ERROR("'%s' driver rejected flash bank at 0x%8.8x", args[0], c->base);
				free(c);
				return ERROR_OK;
			}
			
			/* put flash bank in linked list */
			if (flash_banks)
			{
				/* find last flash bank */
				for (p = flash_banks; p && p->next; p = p->next);
				if (p)
					p->next = c;
			}
			else
			{
				flash_banks = c;
			}
			
			found = 1;
		}
	}
		
	/* no matching flash driver found */
	if (!found)
	{
		ERROR("flash driver '%s' not found", args[0]);
		exit(-1);
	}
	
	return ERROR_OK;
}

int handle_flash_banks_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *p;
	int i = 0;
	
	if (!flash_banks)
	{
		command_print(cmd_ctx, "no flash banks configured");
		return ERROR_OK;
	}
	
	for (p = flash_banks; p; p = p->next)
	{
		command_print(cmd_ctx, "#%i: %s at 0x%8.8x, size 0x%8.8x, buswidth %i, chipwidth %i",
					  i++, p->driver->name, p->base, p->size, p->bus_width, p->chip_width);
	}
	
	return ERROR_OK;
}

int handle_flash_info_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *p;
	int i = 0;
	int j = 0;
		
	if (argc != 1)
	{
		command_print(cmd_ctx, "usage: flash info <num>");
		return ERROR_OK;
	}
	
	for (p = flash_banks; p; p = p->next)
	{
		if (i++ == strtoul(args[0], NULL, 0))
		{
			char buf[1024];
			
			command_print(cmd_ctx, "#%i: %s at 0x%8.8x, size 0x%8.8x, buswidth %i, chipwidth %i",
						i, p->driver->name, p->base, p->size, p->bus_width, p->chip_width);
			for (j = 0; j < p->num_sectors; j++)
			{
				char *erase_state, *protect_state;
				
				if (p->sectors[j].is_erased == 0)
					erase_state = "not erased";
				else if (p->sectors[j].is_erased == 1)
					erase_state = "erased";
				else
					erase_state = "erase state unknown";
				
				if (p->sectors[j].is_protected == 0)
					protect_state = "not protected";
				else if (p->sectors[j].is_protected == 1)
					protect_state = "protected";
				else
					protect_state = "protection state unknown";

				command_print(cmd_ctx, "\t#%i: 0x%8.8x (0x%xkB) %s, %s",
							j, p->sectors[j].offset, p->sectors[j].size,
							erase_state, protect_state);
			}
			
			p->driver->info(p, buf, 1024);
			command_print(cmd_ctx, "%s", buf);
		}
	}
	
	return ERROR_OK;
}

int handle_flash_probe_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *p;
	int retval;
		
	if (argc != 1)
	{
		command_print(cmd_ctx, "usage: flash probe <num>");
		return ERROR_OK;
	}
	
	p = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	if (p)
	{
		if ((retval = p->driver->probe(p)) == ERROR_OK)
		{
			command_print(cmd_ctx, "flash '%s' found at 0x%8.8x", p->driver->name, p->base);
		}
		else if (retval == ERROR_FLASH_BANK_INVALID)
		{
			command_print(cmd_ctx, "probing failed for flash bank '#%s' at 0x%8.8x",
						  args[0], p->base);
		}
		else
		{
			command_print(cmd_ctx, "unknown error when probing flash bank '#%s' at 0x%8.8x",
						  args[0], p->base);
		}
	}
	else
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
	}
	
	return ERROR_OK;
}

int handle_flash_erase_check_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *p;
	int retval;
		
	if (argc != 1)
	{
		command_print(cmd_ctx, "usage: flash erase_check <num>");
		return ERROR_OK;
	}
	
	p = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	if (p)
	{
		if ((retval = p->driver->erase_check(p)) == ERROR_OK)
		{
			command_print(cmd_ctx, "successfully checked erase state", p->driver->name, p->base);
		}
		else
		{
			command_print(cmd_ctx, "unknown error when checking erase state of flash bank #%s at 0x%8.8x",
				args[0], p->base);
		}
	}
	else
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
	}
	
	return ERROR_OK;
}

int handle_flash_protect_check_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *p;
	int retval;
		
	if (argc != 1)
	{
		command_print(cmd_ctx, "usage: flash protect_check <num>");
		return ERROR_OK;
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
			command_print(cmd_ctx, "checking protection state failed (possibly unsupported) by flash #%s at 0x%8.8x", args[0], p->base);
		}
		else
		{
			command_print(cmd_ctx, "unknown error when checking protection state of flash bank '#%s' at 0x%8.8x", args[0], p->base);
		}
	}
	else
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
	}
	
	return ERROR_OK;
}

int handle_flash_erase_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc > 2)
	{
		int first = strtoul(args[1], NULL, 0);
		int last = strtoul(args[2], NULL, 0);
		int retval;
		flash_bank_t *p = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
		struct timeval start, end, duration;

		gettimeofday(&start, NULL);
	
		if (!p)
		{
			command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
			return ERROR_OK;
		}
		
		if ((retval = p->driver->erase(p, first, last)) != ERROR_OK)
		{
			switch (retval)
			{
				case ERROR_TARGET_NOT_HALTED:
					command_print(cmd_ctx, "can't work with this flash while target is running");
					break;
				case ERROR_INVALID_ARGUMENTS:
					command_print(cmd_ctx, "usage: flash_erase <bank> <first> <last>");
					break;
				case ERROR_FLASH_BANK_INVALID:
					command_print(cmd_ctx, "no '%s' flash found at 0x%8.8x", p->driver->name, p->base);
					break;
				case ERROR_FLASH_OPERATION_FAILED:
					command_print(cmd_ctx, "flash erase error");
					break;
				case ERROR_FLASH_SECTOR_INVALID:
					command_print(cmd_ctx, "sector number(s) invalid");
					break;
				case ERROR_OK:
					command_print(cmd_ctx, "erased flash sectors %i to %i", first, last);
					break;
				default:
					command_print(cmd_ctx, "unknown error");
			}
		}
		else
		{
			gettimeofday(&end, NULL);	
			timeval_subtract(&duration, &end, &start);
		
			command_print(cmd_ctx, "erased sectors %i through %i on flash bank %i in %is %ius", first, last, strtoul(args[0], 0, 0), duration.tv_sec, duration.tv_usec);
		}
	}
	else
	{
		command_print(cmd_ctx, "usage: flash erase <bank> <first> <last>");
	}

	return ERROR_OK;
}

int handle_flash_protect_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc > 3)
	{
		int first = strtoul(args[1], NULL, 0);
		int last = strtoul(args[2], NULL, 0);
		int set;
		int retval;
		flash_bank_t *p = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
		if (!p)
		{
			command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
			return ERROR_OK;
		}
		
		if (strcmp(args[3], "on") == 0)
			set = 1;
		else if (strcmp(args[3], "off") == 0)
			set = 0;
		else
		{
			command_print(cmd_ctx, "usage: flash protect <bank> <first> <last> <on|off>");
			return ERROR_OK;
		}
		
		if ((retval = p->driver->protect(p, set, first, last)) != ERROR_OK)
		{
			switch (retval)
			{
				case ERROR_TARGET_NOT_HALTED:
					command_print(cmd_ctx, "can't work with this flash while target is running");
					break;
				case ERROR_INVALID_ARGUMENTS:
					command_print(cmd_ctx, "usage: flash protect <bank> <first> <last> <on|off>");
					break;
				case ERROR_FLASH_BANK_INVALID:
					command_print(cmd_ctx, "no '%s' flash found at 0x%8.8x", p->driver->name, p->base);
					break;
				case ERROR_FLASH_OPERATION_FAILED:
					command_print(cmd_ctx, "flash program error");
					break;
				case ERROR_FLASH_SECTOR_INVALID:
					command_print(cmd_ctx, "sector number(s) invalid");
					break;
				case ERROR_OK:
					command_print(cmd_ctx, "protection of flash sectors %i to %i turned %s", first, last, args[3]);
					break;
				default:
					command_print(cmd_ctx, "unknown error");
			}
		}
		else
		{
			command_print(cmd_ctx, "%s protection for sectors %i through %i on flash bank %i", (set) ? "set" : "cleared", first, last, strtoul(args[0], 0, 0));
		}
	}
	else
	{
		command_print(cmd_ctx, "usage: flash protect <bank> <first> <last> <on|off>");
	}

	return ERROR_OK;
}

int handle_flash_write_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	u32 offset;
	u32 binary_size;
	u8 *buffer;
	u32 buf_cnt;

	fileio_t file;
	fileio_image_t image_info;
	enum fileio_sec_type sec_type;
	
	duration_t duration;
	char *duration_text;
	
	int retval;
	flash_bank_t *p;

	if (argc < 3)
	{
		command_print(cmd_ctx, "usage: flash write <bank> <file> <offset> [type]");
		return ERROR_OK;
	}
	
	duration_start_measure(&duration);
	
	fileio_identify_image_type(&sec_type, (argc == 4) ? args[3] : NULL);
	
	offset = strtoul(args[2], NULL, 0);
	p = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	if (!p)
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
		return ERROR_OK;
	}
	
	image_info.base_address = strtoul(args[2], NULL, 0);
	image_info.has_start_address = 0;

	if (fileio_open(&file, args[1], FILEIO_READ, 
		FILEIO_IMAGE, &image_info, sec_type) != ERROR_OK)
	{
		command_print(cmd_ctx, "flash write error: %s", file.error_str);
		return ERROR_OK;
	}
	
	binary_size = file.size;
	buffer = malloc(binary_size);

	fileio_read(&file, binary_size, buffer, &buf_cnt);
	
	if ((retval = p->driver->write(p, buffer, offset, buf_cnt)) != ERROR_OK)
	{
		command_print(cmd_ctx, "failed writing file %s to flash bank %i at offset 0x%8.8x",
			args[1], strtoul(args[0], NULL, 0), strtoul(args[2], NULL, 0));
		switch (retval)
		{
			case ERROR_TARGET_NOT_HALTED:
				command_print(cmd_ctx, "can't work with this flash while target is running");
				break;
			case ERROR_INVALID_ARGUMENTS:
				command_print(cmd_ctx, "usage: flash write <bank> <file> <offset>");
				break;
			case ERROR_FLASH_BANK_INVALID:
				command_print(cmd_ctx, "no '%s' flash found at 0x%8.8x", p->driver->name, p->base);
				break;
			case ERROR_FLASH_OPERATION_FAILED:
				command_print(cmd_ctx, "flash program error");
				break;
			case ERROR_FLASH_DST_BREAKS_ALIGNMENT:
				command_print(cmd_ctx, "offset breaks required alignment");
				break;
			case ERROR_FLASH_DST_OUT_OF_BANK:
				command_print(cmd_ctx, "destination is out of flash bank (offset and/or file too large)");
				break;
			case ERROR_FLASH_SECTOR_NOT_ERASED:
				command_print(cmd_ctx, "destination sector(s) not erased");
				break;
			default:
				command_print(cmd_ctx, "unknown error");
		}
	}
	else
	{
		duration_stop_measure(&duration, &duration_text);
		command_print(cmd_ctx, "wrote file %s to flash bank %i at offset 0x%8.8x in %s",
			file.url, strtoul(args[0], NULL, 0), offset, duration_text);
		free(duration_text);
	}
	
	free(buffer);
	fileio_close(&file);
	
	return ERROR_OK;
}
