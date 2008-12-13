/***************************************************************************
 *   Copyright (C) 2004, 2005 by Dominic Rath                              *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
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
#include <string.h>

static size_t num_config_files;
static char** config_file_names;

static size_t num_script_dirs;
static char** script_search_dirs;

void add_script_search_dir (const char *dir)
{
	num_script_dirs++;
	script_search_dirs = (char **)realloc(script_search_dirs, (num_script_dirs+1) * sizeof (char *));

	script_search_dirs[num_script_dirs-1] = strdup(dir);
	script_search_dirs[num_script_dirs] = NULL;
}

void add_config_command (const char *cfg)
{
	num_config_files++;
	config_file_names = (char **)realloc(config_file_names, (num_config_files+1) * sizeof (char *));

	config_file_names[num_config_files-1] = strdup(cfg);
	config_file_names[num_config_files] = NULL;
}

/* return full path or NULL according to search rules */
char *find_file(const char *file)
{
	FILE *fp = NULL;
	char **search_dirs = script_search_dirs;
	char *dir;
	char const *mode="r";
	char full_path[1024];

	/* Check absolute and relative to current working dir first.
	 * This keeps full_path reporting belowing working. */
	snprintf(full_path, 1024, "%s", file);
	fp = fopen(full_path, mode);

	while (!fp)
	{
		dir = *search_dirs++;

		if (!dir)
			break;

		snprintf(full_path, 1024, "%s/%s", dir, file);
		fp = fopen(full_path, mode);
	}

	if (fp)
	{
		fclose(fp);
		LOG_DEBUG("found %s", full_path);
		return strdup(full_path);
	}
	return NULL;
}

FILE *open_file_from_path (char *file, char *mode)
{
	if (mode[0]!='r')
	{
		return fopen(file, mode);
	} else
	{
		char *full_path=find_file(file);
		if (full_path==NULL)
			return NULL;
		FILE *fp = NULL;
		fp = fopen(full_path, mode);
		free(full_path);
		return fp;
	}
}

int parse_config_file(struct command_context_s *cmd_ctx)
{
	int retval;
	char **cfg;

	if (!config_file_names)
		add_config_command ("script openocd.cfg");

	cfg = config_file_names;

	while (*cfg)
	{
		retval=command_run_line(cmd_ctx, *cfg);
		if (retval!=ERROR_OK)
			return retval;
		cfg++;
	}

	return ERROR_OK;
}
