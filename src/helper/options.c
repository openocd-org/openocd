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
#include <string.h>

static int help_flag, version_flag;

static struct option long_options[] =
{
	{"help",	no_argument,		&help_flag,	1},
	{"version",	no_argument,		&version_flag,	1},
	{"debug",	optional_argument,	0,		'd'},
	{"file", 	required_argument,	0,		'f'},
	{"search",	required_argument,	0,		's'},
	{"log_output",	required_argument,	0,		'l'},
	{"command",	required_argument,	0,		'c'},
	{0, 0, 0, 0}
};

int configuration_output_handler(struct command_context_s *context, const char* line)
{
	LOG_INFO_N(line);

	return ERROR_OK;
}

int add_default_dirs(void)
{
#ifdef _WIN32
	/* Add the parent of the directory where openocd.exe resides to the
	 * config script search path.
	 * Directory layout: 
	 * bin\openocd.exe
	 * lib\openocd
	 * event\at91eb40a_reset.cfg
	 * target\at91eb40a.cfg
	 */
	{
		char strExePath [MAX_PATH];
		GetModuleFileName (NULL, strExePath, MAX_PATH);
		/* Either this code will *always* work or it will SEGFAULT giving
		 * excellent information on the culprit. 
		 */
		*strrchr(strExePath, '\\')=0;
		strcat(strExePath, "\\..");
		add_script_search_dir(strExePath);
	}
#else
	/* Add dir for openocd supplied scripts last so that user can over
	   ride those scripts if desired. */
	add_script_search_dir(PKGDATADIR);
	add_script_search_dir(PKGLIBDIR);
#endif
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
		
		c = getopt_long(argc, argv, "hvd::l:f:s:c:", long_options, &option_index);
		
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
			case 'v':	/* --version | -v */
				version_flag = 1;
				break;
			case 'f':	/* --file | -f */
			{
				snprintf(command_buffer, 128, "script %s", optarg);
				add_config_command(command_buffer);
				break;
			}
			case 's':	/* --search | -s */
				add_script_search_dir(optarg);
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
			case 'c':	/* --command | -c */
				if (optarg)
				{
					add_config_command(optarg);
				}	
				break;
				
		}
	}

	if (help_flag)
	{
		LOG_OUTPUT("Open On-Chip Debugger\n(c) 2005-2008 by Dominic Rath\n\n");
		LOG_OUTPUT("--help       | -h\tdisplay this help\n");
		LOG_OUTPUT("--version    | -v\tdisplay OpenOCD version\n");
		LOG_OUTPUT("--file       | -f\tuse configuration file <name>\n");
		LOG_OUTPUT("--search     | -s\tdir to search for config files and scripts\n");
		LOG_OUTPUT("--debug      | -d\tset debug level <0-3>\n");
		LOG_OUTPUT("--log_output | -l\tredirect log output to file <name>\n");
		LOG_OUTPUT("--command    | -c\trun <command>\n");
		exit(-1);
	}	

	if (version_flag)
	{
		/* Nothing to do, version gets printed automatically. */
		exit(-1);
	}	


	return ERROR_OK;
}
