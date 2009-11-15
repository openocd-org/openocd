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

#include "jtag.h"
#include "configuration.h"
#include "xsvf.h"
#include "svf.h"
#include "nand.h"
#include "pld.h"
#include "mflash.h"

#include "server.h"
#include "telnet_server.h"
#include "gdb_server.h"
#include "tcl_server.h"

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

static void exit_handler(void)
{
	jtag_interface_quit();
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

int ioutil_init(struct command_context *cmd_ctx);

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

	atexit(exit_handler);

	if (target_init(CMD_CTX) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("target init complete");

	if ((retval = jtag_interface_init(CMD_CTX)) != ERROR_OK)
	{
		/* we must be able to set up the jtag interface */
		return retval;
	}
	LOG_DEBUG("jtag interface init complete");

	/* Try to initialize & examine the JTAG chain at this point, but
	 * continue startup regardless */
	if (jtag_init(CMD_CTX) == ERROR_OK)
	{
		LOG_DEBUG("jtag init complete");
		if (target_examine() == ERROR_OK)
		{
			LOG_DEBUG("jtag examine complete");
		}
	}

	if (flash_init_drivers(CMD_CTX) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("flash init complete");

	if (mflash_init_drivers(CMD_CTX) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("mflash init complete");

	if (nand_init(CMD_CTX) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("NAND init complete");

	if (pld_init(CMD_CTX) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("pld init complete");

	/* initialize tcp server */
	server_init();

	/* initialize telnet subsystem */
	telnet_init("Open On-Chip Debugger");
	gdb_init();
	tcl_init(); /* allows tcl to just connect without going thru telnet */

	target_register_event_callback(log_target_callback_event_handler, CMD_CTX);

	return ERROR_OK;
}

struct command_context *global_cmd_ctx;

/// src/hello.c gives a simple example for writing new command modules
int hello_register_commands(struct command_context *cmd_ctx);

/* NB! this fn can be invoked outside this file for non PC hosted builds */
struct command_context *setup_command_handler(void)
{
	struct command_context *cmd_ctx;

	global_cmd_ctx = cmd_ctx = command_init();

	register_command(cmd_ctx, NULL, "version", handle_version_command,
					 COMMAND_EXEC, "show OpenOCD version");

	/* register subsystem commands */
	hello_register_commands(cmd_ctx);
	server_register_commands(cmd_ctx);
	telnet_register_commands(cmd_ctx);
	gdb_register_commands(cmd_ctx);
	tcl_register_commands(cmd_ctx); /* tcl server commands */
	log_register_commands(cmd_ctx);
	jtag_register_commands(cmd_ctx);
	xsvf_register_commands(cmd_ctx);
	svf_register_commands(cmd_ctx);
	target_register_commands(cmd_ctx);
	flash_register_commands(cmd_ctx);
	nand_register_commands(cmd_ctx);
	pld_register_commands(cmd_ctx);
	mflash_register_commands(cmd_ctx);

	if (log_init(cmd_ctx) != ERROR_OK)
	{
		exit(-1);
	}
	LOG_DEBUG("log init complete");

	LOG_OUTPUT(OPENOCD_VERSION "\n");

	register_command(cmd_ctx, NULL, "init", handle_init_command,
					 COMMAND_ANY, "initializes target and servers - nop on subsequent invocations");

	return cmd_ctx;
}

int httpd_start(void);
void httpd_stop(void);


#if !BUILD_HTTPD && !BUILD_ECOSBOARD
/* implementations of OpenOCD that uses multithreading needs to know when
 * OpenOCD is sleeping. No-op in vanilla OpenOCD
 */
void openocd_sleep_prelude(void)
{
}

void openocd_sleep_postlude(void)
{
}
#endif


/* normally this is the main() function entry, but if OpenOCD is linked
 * into application, then this fn will not be invoked, but rather that
 * application will have it's own implementation of main(). */
int openocd_main(int argc, char *argv[])
{
	int ret;

	/* initialize commandline interface */
	struct command_context *cmd_ctx;

	cmd_ctx = setup_command_handler();

#if BUILD_IOUTIL
	if (ioutil_init(cmd_ctx) != ERROR_OK)
	{
		return EXIT_FAILURE;
	}
#endif

	LOG_OUTPUT("For bug reports, read\n\t"
		"http://openocd.berlios.de/doc/doxygen/bugs.html"
		"\n");


	command_context_mode(cmd_ctx, COMMAND_CONFIG);
	command_set_output_handler(cmd_ctx, configuration_output_handler, NULL);

	if (parse_cmdline_args(cmd_ctx, argc, argv) != ERROR_OK)
		return EXIT_FAILURE;

	ret = parse_config_file(cmd_ctx);
	if ((ret != ERROR_OK) && (ret != ERROR_COMMAND_CLOSE_CONNECTION))
		return EXIT_FAILURE;

#if BUILD_HTTPD
	if (httpd_start() != ERROR_OK)
		return EXIT_FAILURE;
#endif

	if (ret != ERROR_COMMAND_CLOSE_CONNECTION)
	{
		command_context_mode(cmd_ctx, COMMAND_EXEC);
		if (command_run_line(cmd_ctx, "init") != ERROR_OK)
			return EXIT_FAILURE;

		/* handle network connections */
		server_loop(cmd_ctx);
	}

	/* shut server down */
	server_quit();

#if BUILD_HTTPD
	httpd_stop();
#endif

	unregister_all_commands(cmd_ctx);

	/* free commandline interface */
	command_done(cmd_ctx);


	return EXIT_SUCCESS;
}
