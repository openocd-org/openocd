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
#include "target.h"
#include "time_support.h"
#include "fileio.h"
#include "image.h"
#include "log.h"

#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <inttypes.h>

/* command handlers */
int handle_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_flash_banks_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_flash_info_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_flash_probe_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_flash_erase_check_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_flash_erase_address_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_flash_protect_check_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_flash_erase_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_flash_write_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_flash_write_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_flash_write_image_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_flash_protect_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_flash_auto_erase_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
flash_bank_t *get_flash_bank_by_addr(target_t *target, u32 addr);

/* flash drivers
 */
extern flash_driver_t lpc2000_flash;
extern flash_driver_t cfi_flash;
extern flash_driver_t at91sam7_flash;
extern flash_driver_t str7x_flash;
extern flash_driver_t str9x_flash;
extern flash_driver_t stellaris_flash;
extern flash_driver_t str9xpec_flash;
extern flash_driver_t stm32x_flash;
extern flash_driver_t tms470_flash;

flash_driver_t *flash_drivers[] =
{
	&lpc2000_flash,
	&cfi_flash,
	&at91sam7_flash,
	&str7x_flash,
	&str9x_flash,
	&stellaris_flash,
	&str9xpec_flash,
	&stm32x_flash,
	&tms470_flash,
	NULL,
};

flash_bank_t *flash_banks;
static 	command_t *flash_cmd;
static int auto_erase = 0;

/* wafer thin wrapper for invoking the flash driver */
static int flash_driver_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count)
{
	int retval=ERROR_OK;
	if (bank->target->state != TARGET_HALTED)
	{
		ERROR("target not halted - aborting flash write");
		retval=ERROR_TARGET_NOT_HALTED;
	} else
	{
		retval=bank->driver->write(bank, buffer, offset, count);
	}
	if (retval!=ERROR_OK)
	{
		ERROR("Writing to flash bank at address 0x%08x at offset 0x%8.8x", bank->base, offset);
	}
	return retval;
}

static int flash_driver_erase(struct flash_bank_s *bank, int first, int last)
{
	int retval=ERROR_OK;
	if (bank->target->state != TARGET_HALTED)
	{
		ERROR("target not halted - aborting flash erase");
		retval=ERROR_TARGET_NOT_HALTED;
	} else if ((first < 0) || (last < first) || (last >= bank->num_sectors))
	{
		ERROR("invalid flash sector");
		retval=ERROR_FLASH_SECTOR_INVALID;
	} else		
	{
		retval=bank->driver->erase(bank, first, last);
	}
	if (retval!=ERROR_OK)
	{
		ERROR("Failed erasing banks %d to %d", first, last);
	}
	return retval;
}

int flash_driver_protect(struct flash_bank_s *bank, int set, int first, int last)
{
	int retval;
	if (bank->target->state != TARGET_HALTED)
	{
		ERROR("target not halted - aborting flash erase");
		retval=ERROR_TARGET_NOT_HALTED;
	} else if ((first < 0) || (last < first) || (last >= bank->num_sectors))
	{
		ERROR("invalid flash sector");
		retval=ERROR_FLASH_SECTOR_INVALID;
	} else		
	{
		retval=bank->driver->protect(bank, set, first, last);
	}
	if (retval!=ERROR_OK)
	{
		ERROR("Failed protecting banks %d to %d", first, last);
	}
	return retval;
}


int flash_register_commands(struct command_context_s *cmd_ctx)
{
	flash_cmd = register_command(cmd_ctx, NULL, "flash", NULL, COMMAND_ANY, NULL);
	
	register_command(cmd_ctx, flash_cmd, "bank", handle_flash_bank_command, COMMAND_CONFIG, "flash_bank <driver> <base> <size> <chip_width> <bus_width> <target> [driver_options ...]");
	register_command(cmd_ctx, flash_cmd, "auto_erase", handle_flash_auto_erase_command, COMMAND_ANY,
						 "auto erase flash sectors <on|off>");
	return ERROR_OK;
}

int flash_init_drivers(struct command_context_s *cmd_ctx)
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
		register_command(cmd_ctx, flash_cmd, "erase_sector", handle_flash_erase_command, COMMAND_EXEC,
						 "erase sectors at <bank> <first> <last>");
		register_command(cmd_ctx, flash_cmd, "erase_address", handle_flash_erase_address_command, COMMAND_EXEC,
						 "erase address range <address> <length>");
		register_command(cmd_ctx, flash_cmd, "write_bank", handle_flash_write_bank_command, COMMAND_EXEC,
						 "write binary data to <bank> <file> <offset>");
		register_command(cmd_ctx, flash_cmd, "write_image", handle_flash_write_image_command, COMMAND_EXEC,
						 "write_image <file> [offset] [type]");
		register_command(cmd_ctx, flash_cmd, "protect", handle_flash_protect_command, COMMAND_EXEC,
						 "set protection of sectors at <bank> <first> <last> <on|off>");
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
	ERROR("Flash bank %d does not exist", num);
	return NULL;
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
		ERROR("auto_probe failed %d\n", retval);
		return NULL;
	}
	return p;
}

int handle_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int i;
	int found = 0;
	target_t *target;
		
	if (argc < 6)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	
	if ((target = get_target_by_num(strtoul(args[5], NULL, 0))) == NULL)
	{
		ERROR("target %lu not defined", strtoul(args[5], NULL, 0));
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
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	
	for (p = flash_banks; p; p = p->next, i++)
	{
		if (i == strtoul(args[0], NULL, 0))
		{
			char buf[1024];
			
			/* attempt auto probe */
			p->driver->auto_probe(p);
			
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

				command_print(cmd_ctx, "\t#%i: 0x%8.8x (0x%x %ikB) %s, %s",
							j, p->sectors[j].offset, p->sectors[j].size, p->sectors[j].size>>10,
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
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	
	p = get_flash_bank_by_num_noprobe(strtoul(args[0], NULL, 0));
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
		return ERROR_COMMAND_SYNTAX_ERROR;
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
	
	return ERROR_OK;
}

int handle_flash_erase_address_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
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
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	
	/* We can't know if we did a resume + halt, in which case we no longer know the erased state */
	flash_set_dirty();
	
	duration_start_measure(&duration);
	
	if ((retval = flash_erase_address_range(target, address, length)) == ERROR_OK)
	{
		duration_stop_measure(&duration, &duration_text);	
		command_print(cmd_ctx, "erased address 0x%8.8x length %i in %s", address, length, duration_text);
		free(duration_text);
	}
	
	return retval;
}

int handle_flash_protect_check_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
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
			command_print(cmd_ctx, "checking protection state failed (possibly unsupported) by flash #%s at 0x%8.8x", args[0], p->base);
		}
		else
		{
			command_print(cmd_ctx, "unknown error when checking protection state of flash bank '#%s' at 0x%8.8x", args[0], p->base);
		}
	}
	else
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
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
		duration_t duration;
		char *duration_text;
	
		duration_start_measure(&duration);
	
		if (!p)
		{
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		
		if ((retval = flash_driver_erase(p, first, last)) == ERROR_OK)
		{
			duration_stop_measure(&duration, &duration_text);	
			
			command_print(cmd_ctx, "erased sectors %i through %i on flash bank %i in %s", first, last, strtoul(args[0], 0, 0), duration_text);
			free(duration_text);
		}
	}
	else
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
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
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		
		retval = flash_driver_protect(p, set, first, last);
		if (retval == ERROR_OK)
		{
			command_print(cmd_ctx, "%s protection for sectors %i through %i on flash bank %i", (set) ? "set" : "cleared", first, last, strtoul(args[0], 0, 0));
		}
	}
	else
	{
		return ERROR_COMMAND_SYNTAX_ERROR;

	}

	return ERROR_OK;
}

int handle_flash_write_image_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	
	image_t image;
	u32 written;
	
	duration_t duration;
	char *duration_text;
	
	int retval;

	if (argc < 1)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;

	}
	
	if (!target)
	{
		ERROR("no target selected");
		return ERROR_OK;
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
		command_print(cmd_ctx, "image_open error: %s", image.error_str);
		return retval;
	}
	
	retval = flash_write(target, &image, &written, auto_erase);

	if (retval != ERROR_OK)
	{
		image_close(&image);
		return retval;
	}
	
	duration_stop_measure(&duration, &duration_text);
	if (retval == ERROR_OK)
	{
	command_print(cmd_ctx, "wrote %u byte from file %s in %s (%f kb/s)",
		written, args[0], duration_text,
		(float)written / 1024.0 / ((float)duration.duration.tv_sec + ((float)duration.duration.tv_usec / 1000000.0)));
	}
	free(duration_text);

	image_close(&image);
	
	return retval;
}

int handle_flash_write_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	u32 offset;
	u8 *buffer;
	u32 buf_cnt;

	fileio_t fileio;
	
	duration_t duration;
	char *duration_text;
	
	int retval;
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
		command_print(cmd_ctx, "flash write_binary error: %s", fileio.error_str);
		return ERROR_OK;
	}
	
	buffer = malloc(fileio.size);
	if (fileio_read(&fileio, fileio.size, buffer, &buf_cnt) != ERROR_OK)
	{
		command_print(cmd_ctx, "flash write_binary error: %s", fileio.error_str);
		return ERROR_OK;
	}
	
	retval = flash_driver_write(p, buffer, offset, buf_cnt);
		
	free(buffer);
	
	duration_stop_measure(&duration, &duration_text);
	if (retval!=ERROR_OK)
	{
	command_print(cmd_ctx, "wrote  %"PRIi64" byte from file %s to flash bank %i at offset 0x%8.8x in %s (%f kb/s)",
		fileio.size, args[1], strtoul(args[0], NULL, 0), offset, duration_text,
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
flash_bank_t *get_flash_bank_by_addr(target_t *target, u32 addr)
{
	flash_bank_t *c;

	/* cycle through bank list */
	for (c = flash_banks; c; c = c->next)
	{
		int retval;
		retval = c->driver->auto_probe(c);
		
		if (retval != ERROR_OK)
		{
			ERROR("auto_probe failed %d\n", retval);
			return NULL;
		}
		/* check whether address belongs to this flash bank */
		if ((addr >= c->base) && (addr < c->base + c->size) && target == c->target)
			return c;
	}
	ERROR("No flash at address 0x%08x\n", addr);
	return NULL;
}

/* erase given flash region, selects proper bank according to target and address */
int flash_erase_address_range(target_t *target, u32 addr, u32 length)
{
	flash_bank_t *c;
	int first = -1;
	int last = -1;
	int i;
	
	if ((c = get_flash_bank_by_addr(target, addr)) == NULL)
		return ERROR_FLASH_DST_OUT_OF_BANK; /* no corresponding bank found */

	if (c->size == 0 || c->num_sectors == 0)
		return ERROR_FLASH_BANK_INVALID;
	
	if (length == 0)
	{
		/* special case, erase whole bank when length is zero */
		if (addr != c->base)
			return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
		
		return flash_driver_erase(c, 0, c->num_sectors - 1);
	}

	/* check whether it fits */
	if (addr + length > c->base + c->size)
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
	
	if( first == -1 || last == -1 )
		return ERROR_OK;
	
	return flash_driver_erase(c, first, last);
}

/* write (optional verify) an image to flash memory of the given target */
int flash_write(target_t *target, image_t *image, u32 *written, int erase)
{
	int retval;

	int section;
	u32 section_offset;
	flash_bank_t *c;
	
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
	
	/* loop until we reach end of the image */
	while (section < image->num_sections)
	{
		u32 buffer_size;
		u8 *buffer;
		int section_first;
		int section_last;
		u32 run_address = image->sections[section].base_address + section_offset;
		u32 run_size = image->sections[section].size - section_offset;

		if (image->sections[section].size ==  0)
		{
			WARNING("empty section %d", section);
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
		while ((run_address + run_size < c->base + c->size)
				&& (section_last + 1 < image->num_sections))
		{
			if (image->sections[section_last + 1].base_address < (run_address + run_size))
			{
				DEBUG("section %d out of order(very slightly surprising, but supported)", section_last + 1);
				break;
			}
			if (image->sections[section_last + 1].base_address != (run_address + run_size))
				break;
			run_size += image->sections[++section_last].size;
		}

		/* fit the run into bank constraints */
		if (run_address + run_size > c->base + c->size)
			run_size = c->base + c->size - run_address;

		/* allocate buffer */
		buffer = malloc(run_size);
		buffer_size = 0;

		/* read sections to the buffer */
		while (buffer_size < run_size)
		{
			u32 size_read;
			
			if (buffer_size - run_size <= image->sections[section].size - section_offset)
				size_read = buffer_size - run_size;
			else
				size_read = image->sections[section].size - section_offset;
			
			if ((retval = image_read_section(image, section, section_offset,
					size_read, buffer + buffer_size, &size_read)) != ERROR_OK || size_read == 0)
			{
				free(buffer);
				
				return retval;
			}

			buffer_size += size_read;
			section_offset += size_read;

			if (section_offset >= image->sections[section].size)
			{
				section++;
				section_offset = 0;
			}
		}

		retval = ERROR_OK;
		
		if (erase)
		{
			/* calculate and erase sectors */
			retval = flash_erase_address_range( target, run_address, run_size );
		}
		
		if (retval == ERROR_OK)
		{
			/* write flash sectors */
			retval = flash_driver_write(c, buffer, run_address - c->base, run_size);
		}
		
		free(buffer);

		if (retval != ERROR_OK)
		{
				return retval; /* abort operation */
			}

		if (written != NULL)
			*written += run_size; /* add run size to total written counter */
	}

	return ERROR_OK;
}

int handle_flash_auto_erase_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc != 1)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;

	}
	
	if (strcmp(args[0], "on") == 0)
		auto_erase = 1;
	else if (strcmp(args[0], "off") == 0)
		auto_erase = 0;
	else 
		return ERROR_COMMAND_SYNTAX_ERROR;
	
	return ERROR_OK;
}
