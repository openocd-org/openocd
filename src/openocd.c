/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 Richard Missenden                                  *
 *   richard.missenden@googlemail.com                                      *
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

#include "openocd.h"
#include <jtag/jtag.h>
#include <helper/ioutil.h>
#include <helper/configuration.h>
#include <xsvf/xsvf.h>
#include <svf/svf.h>
#include <flash/nor/core.h>
#include <flash/nand/core.h>
#include <pld/pld.h>
#include <flash/mflash.h>

#include <server/server.h>
#include <server/gdb_server.h>
#include <server/httpd.h>

#ifdef HAVE_STRINGS_H
#include <strings.h>
#endif


#define OPENOCD_VERSION \
		"Open On-Chip Debugger " VERSION RELSTR " (" PKGBLDDATE ")"

/* Give TELNET a way to find out what version this is */
COMMAND_HANDLER(handle_version_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	command_print(CMD_CTX, OPENOCD_VERSION);

	return ERROR_OK;
}

static int log_target_callback_event_handler(struct target *target, enum target_event event, void *priv)
{
	switch (event)
	{
		case TARGET_EVENT_GDB_START:
			target->display = 0;
			break;
		case TARGET_EVENT_GDB_END:
			target->display = 1;
			break;
		case TARGET_EVENT_HALTED:
			if (target->display)
			{
				/* do not display information when debugger caused the halt */
				target_arch_state(target);
			}
			break;
		default:
			break;
	}

	return ERROR_OK;
}

static bool init_at_startup = true;

COMMAND_HANDLER(handle_noinit_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;
	init_at_startup = false;
	return ERROR_OK;
}

/* OpenOCD can't really handle failure of this command. Patches welcome! :-) */
COMMAND_HANDLER(handle_init_command)
{

	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	int retval;
	static int initialized = 0;
	if (initialized)
		return ERROR_OK;

	initialized = 1;

	retval = command_run_line(CMD_CTX, "target init");
	if (ERROR_OK != retval)
		return ERROR_FAIL;

	if ((retval = jtag_interface_init(CMD_CTX)) != ERROR_OK)
	{
		/* we must be able to set up the jtag interface */
		return retval;
	}
	LOG_DEBUG("jtag interface init complete");

	/* Try to initialize & examine the JTAG chain at this point,
	 * but continue startup regardless.  Note that platforms
	 * need to be able to provide JTAG event handlers that use
	 * a variety of JTAG operations in order to do that...
	 */
	command_context_mode(CMD_CTX, COMMAND_EXEC);
	if (command_run_line(CMD_CTX, "jtag init") == ERROR_OK)
	{
		LOG_DEBUG("Examining targets...");
		if (target_examine() != ERROR_OK)
			LOG_DEBUG("target examination failed");
	}
	else
		LOG_WARNING("jtag initialization failed; try 'jtag init' again.");
	command_context_mode(CMD_CTX, COMMAND_CONFIG);

	if (command_run_line(CMD_CTX, "flash init") != ERROR_OK)
		return ERROR_FAIL;

	if (command_run_line(CMD_CTX, "mflash init") != ERROR_OK)
		return ERROR_FAIL;

	if (command_run_line(CMD_CTX, "nand init") != ERROR_OK)
		return ERROR_FAIL;

	if (command_run_line(CMD_CTX, "pld init") != ERROR_OK)
		return ERROR_FAIL;
	command_context_mode(CMD_CTX, COMMAND_EXEC);

	/* initialize telnet subsystem */
	gdb_target_add_all(all_targets);

	target_register_event_callback(log_target_callback_event_handler, CMD_CTX);

	return ERROR_OK;
}

static const struct command_registration openocd_command_handlers[] = {
	{
		.name = "version",
		.handler = &handle_version_command,
		.mode = COMMAND_ANY,
		.help = "show program version",
	},
	{
		.name = "noinit",
		.handler = &handle_noinit_command,
		.mode = COMMAND_CONFIG,
		.help = "Prevent 'init' from being called at startup.",
	},
	{
		.name = "init",
		.handler = &handle_init_command,
		.mode = COMMAND_ANY,
		.help = "Initializes configured targets and servers.  "
			"Changes command mode from CONFIG to EXEC.  "
			"Unless 'noinit' is called, this command is "
			"called automatically at the end of startup.",

	},
	COMMAND_REGISTRATION_DONE
};

int openocd_register_commands(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, openocd_command_handlers);
}

struct command_context *global_cmd_ctx;

/* NB! this fn can be invoked outside this file for non PC hosted builds */
struct command_context *setup_command_handler(Jim_Interp *interp)
{
	log_init();
	LOG_DEBUG("log_init: complete");

	const char *startup = openocd_startup_tcl;
	struct command_context *cmd_ctx = command_init(startup, interp);

	/* register subsystem commands */
	typedef int (*command_registrant_t)(struct command_context *cmd_ctx);
	command_registrant_t command_registrants[] = {
		&openocd_register_commands,
		&server_register_commands,
		&gdb_register_commands,
		&log_register_commands,
		&jtag_register_commands,
		&xsvf_register_commands,
		&svf_register_commands,
		&target_register_commands,
		&flash_register_commands,
		&nand_register_commands,
		&pld_register_commands,
		&mflash_register_commands,
		NULL
	};
	for (unsigned i = 0; NULL != command_registrants[i]; i++)
	{
		int retval = (*command_registrants[i])(cmd_ctx);
		if (ERROR_OK != retval)
		{
			command_done(cmd_ctx);
			return NULL;
		}
	}
	LOG_DEBUG("command registration: complete");

	LOG_OUTPUT(OPENOCD_VERSION "\n"
			"Licensed under GNU GPL v2\n");

	global_cmd_ctx = cmd_ctx;

	return cmd_ctx;
}

/* normally this is the main() function entry, but if OpenOCD is linked
 * into application, then this fn will not be invoked, but rather that
 * application will have it's own implementation of main(). */
int openocd_main(int argc, char *argv[])
{
	int ret;

	/* initialize commandline interface */
	struct command_context *cmd_ctx;

	cmd_ctx = setup_command_handler(NULL);

	if (ioutil_init(cmd_ctx) != ERROR_OK)
		return EXIT_FAILURE;

	LOG_OUTPUT("For bug reports, read\n\t"
		"http://openocd.berlios.de/doc/doxygen/bugs.html"
		"\n");

	command_context_mode(cmd_ctx, COMMAND_CONFIG);
	command_set_output_handler(cmd_ctx, configuration_output_handler, NULL);

	if (parse_cmdline_args(cmd_ctx, argc, argv) != ERROR_OK)
		return EXIT_FAILURE;

	if (server_preinit() != ERROR_OK)
		return EXIT_FAILURE;

	ret = parse_config_file(cmd_ctx);
	if (ret != ERROR_OK)
		return EXIT_FAILURE;

	if (httpd_start(cmd_ctx) != ERROR_OK)
		return EXIT_FAILURE;

	ret = server_init(cmd_ctx);
	if (ERROR_OK != ret)
		return EXIT_FAILURE;

	if (init_at_startup)
	{
		ret = command_run_line(cmd_ctx, "init");
		if (ERROR_OK != ret)
			ret = EXIT_FAILURE;
	}

	/* handle network connections */
	if (ERROR_OK == ret)
		server_loop(cmd_ctx);

	server_quit();

	httpd_stop();

	unregister_all_commands(cmd_ctx, NULL);

	/* free commandline interface */
	command_done(cmd_ctx);

	jtag_interface_quit();

	return ret;
}
