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

COMMAND_HANDLER(handle_foo_command)
{
	if (CMD_ARGC < 1 || CMD_ARGC > 2)
	{
		LOG_ERROR("%s: incorrect number of arguments", CMD_NAME);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	uint32_t address;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], address);

	const char *msg = "<unchanged>";
	if (CMD_ARGC == 2)
	{
		bool enable;
		COMMAND_PARSE_ENABLE(CMD_ARGV[1], enable);
		msg = enable ? "enable" : "disable";
	}

	LOG_INFO("%s: address=0x%8.8" PRIx32 " enabled=%s", CMD_NAME, address, msg);
	return ERROR_OK;
}

static bool foo_flag;

COMMAND_HANDLER(handle_flag_command)
{
	return CALL_COMMAND_HANDLER(handle_command_parse_bool,
			&foo_flag, "foo flag");
}

int foo_register_commands(struct command_context *cmd_ctx)
{
	// register several commands under the foo command
	struct command *cmd = register_command(cmd_ctx, NULL, "foo",
			NULL, COMMAND_ANY, "foo: command handler skeleton");

	register_command(cmd_ctx, cmd, "bar",
			&handle_foo_command, COMMAND_ANY,
			"<address> [enable|disable] - an example command");
	register_command(cmd_ctx, cmd, "baz",
			&handle_foo_command, COMMAND_ANY,
			"<address> [enable|disable] - a sample command");

	register_command(cmd_ctx, cmd, "flag",
			&handle_flag_command, COMMAND_ANY,
			"[on|off] - set a flag");

	return ERROR_OK;
}

static COMMAND_HELPER(handle_hello_args, const char **sep, const char **name)
{
	if (CMD_ARGC > 1)
	{
		LOG_ERROR("%s: too many arguments", CMD_NAME);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	if (1 == CMD_ARGC)
	{
		*sep = " ";
		*name = CMD_ARGV[0];
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
		command_print(CMD_CTX, "Greetings%s%s!", sep, name);
	return retval;
}

int hello_register_commands(struct command_context *cmd_ctx)
{
	foo_register_commands(cmd_ctx);

	struct command *cmd = register_command(cmd_ctx, NULL, "hello",
			&handle_hello_command, COMMAND_ANY,
			"[<name>] - prints a warm welcome");
	return cmd ? ERROR_OK : -ENOMEM;
}
