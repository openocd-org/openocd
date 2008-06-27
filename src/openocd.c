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

#define OPENOCD_VERSION "Open On-Chip Debugger " VERSION " (" PKGBLDDATE ") svn:" PKGBLDREV

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "log.h"
#include "types.h"
#include "jtag.h"
#include "configuration.h"
#include "interpreter.h"
#include "xsvf.h"
#include "target.h"
#include "flash.h"
#include "nand.h"
#include "pld.h"

#include "command.h"
#include "server.h"
#include "telnet_server.h"
#include "gdb_server.h"

#include <sys/time.h>
#include <sys/types.h>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

/* Give TELNET a way to find out what version this is */
int handle_version_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	command_print(cmd_ctx, OPENOCD_VERSION);

	return ERROR_OK;
}

static int daemon_startup = 0;

int handle_daemon_startup_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc==0)
		return ERROR_OK;
	if (argc > 1 )
		return ERROR_COMMAND_SYNTAX_ERROR;
	
	daemon_startup = strcmp("reset", args[0])==0;
	
	command_print(cmd_ctx, OPENOCD_VERSION);

	return ERROR_OK;
}


void exit_handler(void)
{
	/* close JTAG interface */
	if (jtag && jtag->quit)
		jtag->quit();
}


/* OpenOCD can't really handle failure of this command. Patches welcome! :-) */
int handle_init_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int retval;
	static int initialized=0;
	if (initialized)
		return ERROR_OK;
	
	initialized=1;
	
	command_set_output_handler(cmd_ctx, configuration_output_handler, NULL);

	atexit(exit_handler);

	
	if (target_init(cmd_ctx) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("target init complete");

	if ((retval=jtag_interface_init(cmd_ctx)) != ERROR_OK)
	{
		/* we must be able to set up the jtag interface */
		return retval;
	}
	LOG_DEBUG("jtag interface init complete");

	/* Try to initialize & examine the JTAG chain at this point, but
	 * continue startup regardless
	 */
	if (jtag_init(cmd_ctx) == ERROR_OK)
	{
		LOG_DEBUG("jtag init complete");
		if (target_examine(cmd_ctx) == ERROR_OK)
		{
			LOG_DEBUG("jtag examine complete");
		}
	}

	
	if (flash_init_drivers(cmd_ctx) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("flash init complete");

	if (nand_init(cmd_ctx) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("NAND init complete");

	if (pld_init(cmd_ctx) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("pld init complete");

	/* initialize tcp server */
	server_init();

	/* initialize telnet subsystem */
	telnet_init("Open On-Chip Debugger");
	gdb_init();

	return ERROR_OK;
}


/* implementations of OpenOCD that uses multithreading needs to lock OpenOCD while calling
 * OpenOCD fn's. No-op in vanilla OpenOCD
 */
void lockBigLock()
{
}
void unlockBigLock()
{
}

/* Hook to add scripting language */
int jim_command(command_context_t *context, char *line)
{
	LOG_ERROR("Syntax error");
	return ERROR_COMMAND_SYNTAX_ERROR;
}

int main(int argc, char *argv[])
{
	/* initialize commandline interface */
	command_context_t *cmd_ctx, *cfg_cmd_ctx;
	cmd_ctx = command_init();

	register_command(cmd_ctx, NULL, "version", handle_version_command,
					 COMMAND_EXEC, "show OpenOCD version");
	register_command(cmd_ctx, NULL, "daemon_startup", handle_daemon_startup_command, COMMAND_CONFIG, 
			"deprecated - use \"init\" and \"reset\" at end of startup script instead");
	
	/* register subsystem commands */
	server_register_commands(cmd_ctx);
	telnet_register_commands(cmd_ctx);
	gdb_register_commands(cmd_ctx);
	log_register_commands(cmd_ctx);
	jtag_register_commands(cmd_ctx);
	interpreter_register_commands(cmd_ctx);
	xsvf_register_commands(cmd_ctx);
	target_register_commands(cmd_ctx);
	flash_register_commands(cmd_ctx);
	nand_register_commands(cmd_ctx);
	pld_register_commands(cmd_ctx);
	
	if (log_init(cmd_ctx) != ERROR_OK)
		return EXIT_FAILURE;
	LOG_DEBUG("log init complete");
	
	LOG_OUTPUT( OPENOCD_VERSION "\n" );
	
	
	/* DANGER!!! make sure that the line below does not appear in a patch, do not remove */
	/* DANGER!!! make sure that the line below does not appear in a patch, do not remove */
	/* DANGER!!! make sure that the line below does not appear in a patch, do not remove */
	/* DANGER!!! make sure that the line below does not appear in a patch, do not remove */
	/* DANGER!!! make sure that the line below does not appear in a patch, do not remove */
	LOG_OUTPUT( "$URL$\n");
	/* DANGER!!! make sure that the line above does not appear in a patch, do not remove */
	/* DANGER!!! make sure that the line above does not appear in a patch, do not remove */
	/* DANGER!!! make sure that the line above does not appear in a patch, do not remove */
	/* DANGER!!! make sure that the line above does not appear in a patch, do not remove */
	/* DANGER!!! make sure that the line above does not appear in a patch, do not remove */

	register_command(cmd_ctx, NULL, "init", handle_init_command,
					 COMMAND_ANY, "initializes target and servers - nop on subsequent invocations");

	cfg_cmd_ctx = copy_command_context(cmd_ctx);
	cfg_cmd_ctx->mode = COMMAND_CONFIG;
	command_set_output_handler(cfg_cmd_ctx, configuration_output_handler, NULL);
	
	if (parse_cmdline_args(cfg_cmd_ctx, argc, argv) != ERROR_OK)
		return EXIT_FAILURE;

	if (parse_config_file(cfg_cmd_ctx) != ERROR_OK)
		return EXIT_FAILURE;
	
	command_done(cfg_cmd_ctx);

	if (command_run_line(cmd_ctx, "init")!=ERROR_OK)
		return EXIT_FAILURE;
	
	if (daemon_startup)
		command_run_line(cmd_ctx, "reset");

	/* handle network connections */
	server_loop(cmd_ctx);

	/* shut server down */
	server_quit();

	unregister_all_commands(cmd_ctx);
	
	/* free commandline interface */
	command_done(cmd_ctx);

	return EXIT_SUCCESS;
}

