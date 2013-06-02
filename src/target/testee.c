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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
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
	},
	COMMAND_REGISTRATION_DONE
};

static int testee_init(struct command_context *cmd_ctx, struct target *target)
{
	return ERROR_OK;
}
static int testee_poll(struct target *target)
{
	return ERROR_OK;
}
static int testee_halt(struct target *target)
{
	return ERROR_OK;
}
static int testee_reset_assert(struct target *target)
{
	return ERROR_OK;
}
static int testee_reset_deassert(struct target *target)
{
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
