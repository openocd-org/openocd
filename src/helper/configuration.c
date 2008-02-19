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

static size_t num_config_files;
static char** config_file_names;

static size_t num_script_dirs;
static char** script_search_dirs;

static int help_flag;

static struct option long_options[] =
{
	{"help",			no_argument,	&help_flag, 1},

	{"debug",			optional_argument,	0, 'd'},
	{"file", 			required_argument,	0, 'f'},
	{"search",			required_argument,	0, 's'},
	{"log_output",		required_argument,	0, 'l'},
	
	{0, 0, 0, 0}
};

int configuration_output_handler(struct command_context_s *context, char* line)
{
	INFO(line);
	
	return ERROR_OK;
}

void add_script_search_dir (const char *dir)
{
	num_script_dirs++;
	script_search_dirs = (char **)realloc(script_search_dirs, (num_script_dirs+1) * sizeof (char *));

	script_search_dirs[num_script_dirs-1] = strdup(dir);
	script_search_dirs[num_script_dirs] = NULL;
}

void add_config_file_name (const char *cfg)
{
	num_config_files++;
	config_file_names = (char **)realloc(config_file_names, (num_config_files+1) * sizeof (char *));

	config_file_names[num_config_files-1] = strdup(cfg);
	config_file_names[num_config_files] = NULL;
}

int parse_cmdline_args(struct command_context_s *cmd_ctx, int argc, char *argv[])
{
	int c;
	char command_buffer[128];

	/* Always search relative to current working dir first. */
	add_script_search_dir(".");

	while (1)
	{	
		/* getopt_long stores the option index here. */
		int option_index = 0;
		
		c = getopt_long(argc, argv, "hd::l:f:s:", long_options, &option_index);
		
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
				add_config_file_name(optarg);
				break;
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
		}
	}

	if (help_flag)
	{
		printf("Open On-Chip Debugger\n(c) 2005 by Dominic Rath\n\n");
		printf("--help       | -h\tdisplay this help\n");
		printf("--file       | -f\tuse configuration file <name>\n");
		printf("--search     | -s\tdir to search for config files and scripts.\n");
		printf("--debug      | -d\tset debug level <0-3>\n");
		printf("--log_output | -l\tredirect log output to file <name>\n");
		exit(-1);
	}	

	/* Add dir for openocd supplied scripts last so that user can over
	   ride those scripts if desired. */
	add_script_search_dir(PKGDATADIR);

	return ERROR_OK;
}

FILE *open_file_from_path (command_context_t *cmd_ctx, char *file, char *mode)
{
	FILE *fp = NULL;
	char **search_dirs = script_search_dirs;
	char *dir;
	char full_path[1024];

	while (!fp)
	{
		dir = *search_dirs++;

		if (!dir)
			break;

		snprintf(full_path, 1024, "%s/%s", dir, file);
		fp = fopen(full_path, mode);
	}

	if (fp)
		command_print(cmd_ctx, "opened %s", full_path);

	return fp;
}

int parse_config_file(struct command_context_s *cmd_ctx)
{
	char **cfg;
	FILE *config_file;

	if (!config_file_names)
		add_config_file_name ("openocd.cfg");

	cfg = config_file_names;

	while (*cfg)
	{
		config_file = open_file_from_path(cmd_ctx, *cfg, "r");
		if (!config_file)
		{
			ERROR("couldn't open config file");
			return ERROR_NO_CONFIG_FILE;
		}

		command_run_file(cmd_ctx, config_file, COMMAND_CONFIG);

		fclose(config_file);

		cfg++;
	}

	return ERROR_OK;
}
