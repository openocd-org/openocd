/***************************************************************************
 *   Copyright (C) 2004, 2005 by Dominic Rath                              *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007-2010 Øyvind Harboe                                 *
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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "configuration.h"
/* @todo the inclusion of server.h here is a layering violation */
#include <server/server.h>

#include <getopt.h>

static int help_flag, version_flag;

static const struct option long_options[] = {
	{"help",		no_argument,			&help_flag,		1},
	{"version",		no_argument,			&version_flag,	1},
	{"debug",		optional_argument,		0,				'd'},
	{"file",		required_argument,		0,				'f'},
	{"search",		required_argument,		0,				's'},
	{"log_output",	required_argument,		0,				'l'},
	{"command",		required_argument,		0,				'c'},
	{"pipe",		no_argument,			0,				'p'},
	{0, 0, 0, 0}
};

int configuration_output_handler(struct command_context *context, const char *line)
{
	LOG_USER_N("%s", line);

	return ERROR_OK;
}

#ifdef _WIN32
static char *find_suffix(const char *text, const char *suffix)
{
	size_t text_len = strlen(text);
	size_t suffix_len = strlen(suffix);

	if (suffix_len == 0)
		return (char *)text + text_len;

	if (suffix_len > text_len || strncmp(text + text_len - suffix_len, suffix, suffix_len) != 0)
		return NULL; /* Not a suffix of text */

	return (char *)text + text_len - suffix_len;
}
#endif

static void add_default_dirs(void)
{
	const char *run_prefix;
	char *path;

#ifdef _WIN32
	char strExePath[MAX_PATH];
	GetModuleFileName(NULL, strExePath, MAX_PATH);

	/* Strip executable file name, leaving path */
	*strrchr(strExePath, '\\') = '\0';

	/* Convert path separators to UNIX style, should work on Windows also. */
	for (char *p = strExePath; *p; p++) {
		if (*p == '\\')
			*p = '/';
	}

	char *end_of_prefix = find_suffix(strExePath, BINDIR);
	if (end_of_prefix != NULL)
		*end_of_prefix = '\0';

	run_prefix = strExePath;
#else
	run_prefix = "";
#endif

	LOG_DEBUG("bindir=%s", BINDIR);
	LOG_DEBUG("pkgdatadir=%s", PKGDATADIR);
	LOG_DEBUG("run_prefix=%s", run_prefix);

	/*
	 * The directory containing OpenOCD-supplied scripts should be
	 * listed last in the built-in search order, so the user can
	 * override these scripts with site-specific customizations.
	 */
	const char *home = getenv("HOME");

	if (home) {
		path = alloc_printf("%s/.openocd", home);
		if (path) {
			add_script_search_dir(path);
			free(path);
		}
	}
#ifdef _WIN32
	const char *appdata = getenv("APPDATA");

	if (appdata) {
		path = alloc_printf("%s/OpenOCD", appdata);
		if (path) {
			add_script_search_dir(path);
			free(path);
		}
	}
#endif

	path = alloc_printf("%s%s%s", run_prefix, PKGDATADIR, "/site");
	if (path) {
		add_script_search_dir(path);
		free(path);
	}

	path = alloc_printf("%s%s%s", run_prefix, PKGDATADIR, "/scripts");
	if (path) {
		add_script_search_dir(path);
		free(path);
	}
}

int parse_cmdline_args(struct command_context *cmd_ctx, int argc, char *argv[])
{
	int c;
	char command_buffer[128];

	while (1) {
		/* getopt_long stores the option index here. */
		int option_index = 0;

		c = getopt_long(argc, argv, "hvd::l:f:s:c:p", long_options, &option_index);

		/* Detect the end of the options. */
		if (c == -1)
			break;

		switch (c) {
			case 0:
				break;
			case 'h':		/* --help | -h */
				help_flag = 1;
				break;
			case 'v':		/* --version | -v */
				version_flag = 1;
				break;
			case 'f':		/* --file | -f */
			{
				snprintf(command_buffer, 128, "script {%s}", optarg);
				add_config_command(command_buffer);
				break;
			}
			case 's':		/* --search | -s */
				add_script_search_dir(optarg);
				break;
			case 'd':		/* --debug | -d */
				if (optarg)
					snprintf(command_buffer, 128, "debug_level %s", optarg);
				else
					snprintf(command_buffer, 128, "debug_level 3");
				command_run_line(cmd_ctx, command_buffer);
				break;
			case 'l':		/* --log_output | -l */
				if (optarg) {
					snprintf(command_buffer, 128, "log_output %s", optarg);
					command_run_line(cmd_ctx, command_buffer);
				}
				break;
			case 'c':		/* --command | -c */
				if (optarg)
				    add_config_command(optarg);
				break;
			case 'p':
				/* to replicate the old syntax this needs to be synchronous
				 * otherwise the gdb stdin will overflow with the warning message */
				command_run_line(cmd_ctx, "gdb_port pipe; log_output openocd.log");
				LOG_WARNING("deprecated option: -p/--pipe. Use '-c \"gdb_port pipe; "
						"log_output openocd.log\"' instead.");
				break;
		}
	}

	if (help_flag) {
		LOG_OUTPUT("Open On-Chip Debugger\nLicensed under GNU GPL v2\n");
		LOG_OUTPUT("--help       | -h\tdisplay this help\n");
		LOG_OUTPUT("--version    | -v\tdisplay OpenOCD version\n");
		LOG_OUTPUT("--file       | -f\tuse configuration file <name>\n");
		LOG_OUTPUT("--search     | -s\tdir to search for config files and scripts\n");
		LOG_OUTPUT("--debug      | -d\tset debug level <0-3>\n");
		LOG_OUTPUT("--log_output | -l\tredirect log output to file <name>\n");
		LOG_OUTPUT("--command    | -c\trun <command>\n");
		exit(-1);
	}

	if (version_flag) {
		/* Nothing to do, version gets printed automatically. */
		/* It is not an error to request the VERSION number. */
		exit(0);
	}

	/* paths specified on the command line take precedence over these
	 * built-in paths
	 */
	add_default_dirs();

	return ERROR_OK;
}
