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
#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include "command.h"
#include "types.h"

extern int parse_cmdline_args(struct command_context_s *cmd_ctx, int argc, char *argv[]);
extern int parse_config_file(struct command_context_s *cmd_ctx);
extern void add_config_command (const char *cfg);
extern void add_script_search_dir (const char *dir);
extern int configuration_output_handler(struct command_context_s *context, const char* line);
extern FILE *open_file_from_path (char *file, char *mode);
extern char *find_file(const char *name);
int add_default_dirs(void);

#endif /* CONFIGURATION_H */
