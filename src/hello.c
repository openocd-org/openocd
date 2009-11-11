/***************************************************************************
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
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
#include <config.h>
#endif
#include "log.h"

static COMMAND_HELPER(handle_hello_args, const char **sep, const char **name)
{
	if (argc > 1)
	{
		LOG_ERROR("%s: too many arguments", CMD_NAME);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	if (1 == argc)
	{
		*sep = " ";
		*name = args[0];
	}
	else
		*sep = *name = "";

	return ERROR_OK;
}
COMMAND_HANDLER(handle_hello_command)
{
	const char *sep, *name;
	int retval = CALL_COMMAND_HANDLER(handle_hello_args, &sep, &name);
	if (ERROR_OK == retval)
		command_print(cmd_ctx, "Greetings%s%s!", sep, name);
	return retval;
}

int hello_register_commands(struct command_context_s *cmd_ctx)
{
	struct command_s *cmd = register_command(cmd_ctx, NULL, "hello",
			&handle_hello_command, COMMAND_ANY,
			"option");
	return cmd ? ERROR_OK : -ENOMEM;
}
