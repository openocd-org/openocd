// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2010 by Øyvind Harboe                                   *
 ***************************************************************************/

/* this file contains various functionality useful to standalone systems */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "log.h"
#include "time_support.h"
#include "util.h"

COMMAND_HANDLER(handler_util_ms)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	command_print(CMD, "%" PRId64, timeval_ms());

	return ERROR_OK;
}

static const struct command_registration util_command_handlers[] = {
	{
		.name = "ms",
		.mode = COMMAND_ANY,
		.handler = handler_util_ms,
		.help =
			"Returns ever increasing milliseconds. Used to calculate differences in time.",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

int util_init(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, util_command_handlers);
}
