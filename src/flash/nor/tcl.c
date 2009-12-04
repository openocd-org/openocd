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
#include "config.h"
#endif
#include "imp.h"

COMMAND_HANDLER(handle_flash_bank_command)
{
	if (CMD_ARGC < 7)
	{
		LOG_ERROR("usage: flash bank <name> <driver> "
				"<base> <size> <chip_width> <bus_width>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	// save bank name and advance arguments for compatibility
	const char *bank_name = *CMD_ARGV++;
	CMD_ARGC--;

	struct target *target;
	if ((target = get_target(CMD_ARGV[5])) == NULL)
	{
		LOG_ERROR("target '%s' not defined", CMD_ARGV[5]);
		return ERROR_FAIL;
	}

	const char *driver_name = CMD_ARGV[0];
	struct flash_driver *driver = flash_driver_find_by_name(driver_name);
	if (NULL == driver)
	{
		/* no matching flash driver found */
		LOG_ERROR("flash driver '%s' not found", driver_name);
		return ERROR_FAIL;
	}

	/* register flash specific commands */
	if (NULL != driver->commands)
	{
		int retval = register_commands(CMD_CTX, NULL,
				driver->commands);
		if (ERROR_OK != retval)
		{
			LOG_ERROR("couldn't register '%s' commands",
					driver_name);
			return ERROR_FAIL;
		}
	}

	struct flash_bank *c = malloc(sizeof(*c));
	c->name = strdup(bank_name);
	c->target = target;
	c->driver = driver;
	c->driver_priv = NULL;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], c->base);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], c->size);
	COMMAND_PARSE_NUMBER(int, CMD_ARGV[3], c->chip_width);
	COMMAND_PARSE_NUMBER(int, CMD_ARGV[4], c->bus_width);
	c->num_sectors = 0;
	c->sectors = NULL;
	c->next = NULL;

	int retval;
	retval = CALL_COMMAND_HANDLER(driver->flash_bank_command, c);
	if (ERROR_OK != retval)
	{
		LOG_ERROR("'%s' driver rejected flash bank at 0x%8.8" PRIx32,
				driver_name, c->base);
		free(c);
		return retval;
	}

	return ERROR_OK;

}


static int jim_flash_banks(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	if (argc != 1) {
		Jim_WrongNumArgs(interp, 1, argv, "no arguments to flash_banks command");
		return JIM_ERR;
	}

	Jim_Obj *list = Jim_NewListObj(interp, NULL, 0);

	for (struct flash_bank *p = flash_bank_list(); p; p = p->next)
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


COMMAND_HANDLER(handle_flash_init_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	static bool flash_initialized = false;
	if (flash_initialized)
	{
		LOG_INFO("'flash init' has already been called");
		return ERROR_OK;
	}
	flash_initialized = true;

	LOG_DEBUG("Initializing flash devices...");
	return flash_init_drivers(CMD_CTX);
}

static const struct command_registration flash_config_command_handlers[] = {
	{
		.name = "bank",
		.handler = &handle_flash_bank_command,
		.mode = COMMAND_CONFIG,
		.usage = "<name> <driver> <base> <size> "
			"<chip_width> <bus_width> <target> "
			"[driver_options ...]",
		.help = "Define a new bank with the given name, "
			"using the specified NOR flash driver.",
	},
	{
		.name = "init",
		.mode = COMMAND_CONFIG,
		.handler = &handle_flash_init_command,
		.help = "initialize flash devices",
	},
	{
		.name = "banks",
		.mode = COMMAND_ANY,
		.jim_handler = &jim_flash_banks,
		.help = "return information about the flash banks",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration flash_command_handlers[] = {
	{
		.name = "flash",
		.mode = COMMAND_ANY,
		.help = "NOR flash command group",
		.chain = flash_config_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

int flash_register_commands(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, flash_command_handlers);
}
