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

bool is_memory_regions_overlap(target_addr_t start1,
	unsigned int size1,
	target_addr_t start2,
	unsigned int  size2)
{
	/* Two memory regions: [S1,E1] and [S2,E2] where:
	 * E1 = S1 + size1 - 1, E2 = S2 + size2 - 1
	 *
	 * After normalization:
	 *  Region 1: [0, size1 - 1]
	 *  Region 2: [start2 - start1, (start2 - start1) + size2 - 1]
	 *
	 * Intersection cases:
	 *  1. Normalized region 2 wraps around 0 (unsigned overflow)
	 *  2. Start of normalized region 2 is within region 1
	 */
	start2 -= start1;
	target_addr_t end1 = size1 - 1;
	target_addr_t end2 = start2 + size2 - 1;

	return start2 > end2 || start2 <= end1;
}
