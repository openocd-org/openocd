/***************************************************************************
 *   Copyright (C) 2004, 2005 by Dominic Rath                              *
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

#include "types.h"
#include "command.h"
#include "configuration.h"
#include "log.h"

#include <stdio.h>
#include <stdlib.h>
#include <getopt.h>

char* config_file_name;

static int help_flag;

static struct option long_options[] =
{
	{"help",			no_argument,	&help_flag, 1},

	{"debug",			optional_argument,	0, 'd'},
	{"file", 			required_argument,	0, 'f'},
	{"log_output",		required_argument,	0, 'l'},
	
	{0, 0, 0, 0}
};

int configuration_output_handler(struct command_context_s *context, char* line)
{
	INFO(line);
	
	return ERROR_OK;
}

int parse_cmdline_args(struct command_context_s *cmd_ctx, int argc, char *argv[])
{
	int c;
	char command_buffer[128];
			
	while (1)
	{	
		/* getopt_long stores the option index here. */
		int option_index = 0;
		
		c = getopt_long(argc, argv, "hd::l:f:", long_options, &option_index);
		
		/* Detect the end of the options. */
		if (c == -1)
			break;
		
		switch (c)
		{
			case 0:
				break;
			case 'h':	/* --help | -h */
				help_flag = 1;
				break;
			case 'f':	/* --file | -f */
				config_file_name = optarg;
				break;
			case 'd':	/* --debug | -d */
				if (optarg)
					snprintf(command_buffer, 128, "debug_level %s", optarg);
				else
					snprintf(command_buffer, 128, "debug_level 3");
				command_run_line(cmd_ctx, command_buffer);
				break;
			case 'l':	/* --log_output | -l */
				if (optarg)
				{
					snprintf(command_buffer, 128, "log_output %s", optarg);
					command_run_line(cmd_ctx, command_buffer);
				}	
				break;
		}
	}

	if (help_flag)
	{
		printf("Open On-Chip Debugger\n(c) 2005 by Dominic Rath\n\n");
		printf("--help       | -h\tdisplay this help\n");
		printf("--file       | -f\tuse configuration file <name>\n");
		printf("--debug      | -d\tset debug level <0-3>\n");
		printf("--log_output | -l\tredirect log output to file <name>\n");
		exit(-1);
	}	

	return ERROR_OK;
}

int parse_config_file(struct command_context_s *cmd_ctx)
{
	FILE *config_file;

	if (!config_file_name)
		config_file_name = "openocd.cfg";

	config_file = fopen(config_file_name, "r");
	if (!config_file)
	{
		ERROR("couldn't open config file");
		return ERROR_NO_CONFIG_FILE;
	}

	command_run_file(cmd_ctx, config_file, COMMAND_CONFIG);

	fclose(config_file);

	return ERROR_OK;
}

