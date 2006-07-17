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

#include "log.h"
#include "configuration.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdarg.h>

int debug_level = -1;

static FILE* log_output;

static char *log_strings[4] = 
{
	"Error:  ",
	"Warning:",
	"Info:   ",
	"Debug:  ",
};

void log_printf(enum log_levels level, const char *file, int line, const char *function, const char *format, ...)
{
	va_list args;
	char buffer[512];

	if (level > debug_level)
		return;

	va_start(args, format);
	vsnprintf(buffer, 512, format, args);

	fprintf(log_output, "%s %s:%d %s(): %s\n", log_strings[level], file, line, function, buffer);
	fflush(log_output);
	
	va_end(args);
}

void short_log_printf(enum log_levels level, const char *format, ...)
{
	va_list args;
	char buffer[512];

	if (level > debug_level)
		return;

	va_start(args, format);
	vsnprintf(buffer, 512, format, args);

	fprintf(log_output, "%s %s\n", log_strings[level], buffer);
	fflush(log_output);

	va_end(args);
}

/* change the current debug level on the fly
 * 0: only ERRORS
 * 1: + WARNINGS
 * 2: + INFORMATIONAL MSGS
 * 3: + DEBUG MSGS
 */
int handle_debug_level_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc == 0)
		command_print(cmd_ctx, "debug_level: %i", debug_level);

	if (argc > 0)
		debug_level = strtoul(args[0], NULL, 0);

	if (debug_level < 0)
		debug_level = 0;

	if (debug_level > 3)
		debug_level = 3;

	return ERROR_OK;
}

int handle_log_output_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc == 1)
	{
		FILE* file = fopen(args[0], "w");
		
		if (file)
		{
			log_output = file;
		}
	}

	return ERROR_OK;
}

int log_register_commands(struct command_context_s *cmd_ctx)
{
	register_command(cmd_ctx, NULL, "log_output", handle_log_output_command,
		COMMAND_ANY, "redirect logging to <file> (default: stderr)");
	register_command(cmd_ctx, NULL, "debug_level", handle_debug_level_command,
		COMMAND_ANY, "adjust debug level <0-3>");

	return ERROR_OK;
}

int log_init(struct command_context_s *cmd_ctx)
{
	/* set defaults for daemon configuration, if not set by cmdline or cfgfile */
	if (debug_level == -1)
		debug_level = LOG_INFO;
	
	if (log_output == NULL)
	{
		log_output = stderr;
	}
	
	return ERROR_OK;
}
