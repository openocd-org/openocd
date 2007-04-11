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

#define OPENOCD_VERSION "Open On-Chip Debugger (2007-04-11 16:20 CEST)"

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

int main(int argc, char *argv[])
{
	/* initialize commandline interface */
	command_context_t *cmd_ctx, *cfg_cmd_ctx;
	cmd_ctx = command_init();

	register_command(cmd_ctx, NULL, "version", handle_version_command,
					 COMMAND_EXEC, "show OpenOCD version");
	
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
	DEBUG("log init complete");
	
	INFO( OPENOCD_VERSION );

	cfg_cmd_ctx = copy_command_context(cmd_ctx);
	cfg_cmd_ctx->mode = COMMAND_CONFIG;
	command_set_output_handler(cfg_cmd_ctx, configuration_output_handler, NULL);
	
	if (parse_cmdline_args(cfg_cmd_ctx, argc, argv) != ERROR_OK)
		return EXIT_FAILURE;

	if (parse_config_file(cfg_cmd_ctx) != ERROR_OK)
		return EXIT_FAILURE;
	
	command_done(cfg_cmd_ctx);

	command_set_output_handler(cmd_ctx, configuration_output_handler, NULL);

	if (jtag_init(cmd_ctx) != ERROR_OK)
		return EXIT_FAILURE;
	DEBUG("jtag init complete");

	if (target_init(cmd_ctx) != ERROR_OK)
		return EXIT_FAILURE;
	DEBUG("target init complete");

	if (flash_init(cmd_ctx) != ERROR_OK)
		return EXIT_FAILURE;
	DEBUG("flash init complete");

	if (nand_init(cmd_ctx) != ERROR_OK)
		return EXIT_FAILURE;
	DEBUG("NAND init complete");

	if (pld_init(cmd_ctx) != ERROR_OK)
		return EXIT_FAILURE;
	DEBUG("pld init complete");

	/* initialize tcp server */
	server_init();
	
	/* initialize telnet subsystem */
	telnet_init("Open On-Chip Debugger");
	gdb_init();

	/* handle network connections */
	server_loop(cmd_ctx);
	
	/* shut server down */
	server_quit();
	
	/* free commandline interface */
	command_done(cmd_ctx);
	
	return EXIT_SUCCESS;
}
