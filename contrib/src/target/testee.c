// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>

#include "target.h"
#include "target_type.h"
#include "hello.h"

static const struct command_registration testee_command_handlers[] = {
	{
		.name = "testee",
		.mode = COMMAND_ANY,
		.help = "testee target commands",
		.chain = hello_command_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static int testee_init(struct command_context *cmd_ctx, struct target *target)
{
	return ERROR_OK;
}
static int testee_poll(struct target *target)
{
	if ((target->state == TARGET_RUNNING) || (target->state == TARGET_DEBUG_RUNNING))
		target->state = TARGET_HALTED;
	return ERROR_OK;
}
static int testee_halt(struct target *target)
{
	target->state = TARGET_HALTED;
	return ERROR_OK;
}
static int testee_reset_assert(struct target *target)
{
	target->state = TARGET_RESET;
	return ERROR_OK;
}
static int testee_reset_deassert(struct target *target)
{
	target->state = TARGET_RUNNING;
	return ERROR_OK;
}
struct target_type testee_target = {
	.name = "testee",
	.commands = testee_command_handlers,

	.init_target = &testee_init,
	.poll = &testee_poll,
	.halt = &testee_halt,
	.assert_reset = &testee_reset_assert,
	.deassert_reset = &testee_reset_deassert,
};
