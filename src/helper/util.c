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

static int jim_util_ms(Jim_Interp *interp,
	int argc,
	Jim_Obj * const *argv)
{
	if (argc != 1) {
		Jim_WrongNumArgs(interp, 1, argv, "ls ?dir?");
		return JIM_ERR;
	}

	/* Cast from 64 to 32 bit int works for 2's-compliment
	 * when calculating differences*/
	Jim_SetResult(interp, Jim_NewIntObj(interp, (int)timeval_ms()));

	return JIM_OK;
}

static const struct command_registration util_command_handlers[] = {
	/* jim handlers */
	{
		.name = "ms",
		.mode = COMMAND_ANY,
		.jim_handler = jim_util_ms,
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
