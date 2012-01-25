/***************************************************************************
 *   Copyright (C) 2011 by Mathias Kuester                                 *
 *   Mathias Kuester <kesmtp@freenet.de>                                   *
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

/* project specific includes */
#include <jtag/interface.h>
#include <jtag/tcl.h>
#include <transport/transport.h>
#include <helper/time_support.h>
#include <target/target.h>
#include <jtag/stlink/stlink_tcl.h>
#include <jtag/stlink/stlink_transport.h>
#include <jtag/stlink/stlink_interface.h>

COMMAND_HANDLER(stlink_transport_jtag_command)
{
	LOG_DEBUG("stlink_transport_jtag_command");

	return ERROR_OK;
}

static const struct command_registration
stlink_transport_stlink_subcommand_handlers[] = {
	{
	 .name = "newtap",
	 .mode = COMMAND_CONFIG,
	 .jim_handler = jim_stlink_newtap,
	 .help = "Create a new TAP instance named basename.tap_type, "
	 "and appends it to the scan chain.",
	 .usage = "basename tap_type '-irlen' count "
	 "['-expected_id' number] ",
	 },

	COMMAND_REGISTRATION_DONE
};

static const struct command_registration
stlink_transport_jtag_subcommand_handlers[] = {
	{
	 .name = "init",
	 .mode = COMMAND_ANY,
	 .handler = stlink_transport_jtag_command,
	 .usage = ""
	 },
	{
	 .name = "arp_init",
	 .mode = COMMAND_ANY,
	 .handler = stlink_transport_jtag_command,
	 .usage = ""
	 },
	{
	 .name = "arp_init-reset",
	 .mode = COMMAND_ANY,
	 .handler = stlink_transport_jtag_command,
	 .usage = ""
	 },
	{
	 .name = "tapisenabled",
	 .mode = COMMAND_EXEC,
	 .jim_handler = jim_jtag_tap_enabler,
	 },
	{
	 .name = "tapenable",
	 .mode = COMMAND_EXEC,
	 .jim_handler = jim_jtag_tap_enabler,
	 },
	{
	 .name = "tapdisable",
	 .mode = COMMAND_EXEC,
	 .handler = stlink_transport_jtag_command,
	 .usage = "",
	 },
	{
	 .name = "configure",
	 .mode = COMMAND_EXEC,
	 .handler = stlink_transport_jtag_command,
	 .usage = "",
	 },
	{
	 .name = "cget",
	 .mode = COMMAND_EXEC,
	 .jim_handler = jim_jtag_configure,
	 },
	{
	 .name = "names",
	 .mode = COMMAND_ANY,
	 .handler = stlink_transport_jtag_command,
	 .usage = "",
	 },

	COMMAND_REGISTRATION_DONE
};

static const struct command_registration stlink_transport_command_handlers[] = {

	{
	 .name = "stlink",
	 .mode = COMMAND_ANY,
	 .help = "perform stlink actions",
	 .usage = "",
	 .chain = stlink_transport_stlink_subcommand_handlers,
	 },
	{
	 .name = "jtag",
	 .mode = COMMAND_ANY,
	 .usage = "",
	 .chain = stlink_transport_jtag_subcommand_handlers,
	 },
	COMMAND_REGISTRATION_DONE
};

static int stlink_transport_register_commands(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL,
				 stlink_transport_command_handlers);
}

static int stlink_transport_init(struct command_context *cmd_ctx)
{
	LOG_DEBUG("stlink_transport_init");
	struct target *t = get_current_target(cmd_ctx);
	struct transport *transport;
	enum stlink_transports tr;

	if (!t) {
		LOG_ERROR("no current target");
		return ERROR_FAIL;
	}

	transport = get_current_transport();

	if (!transport) {
		LOG_ERROR("no transport selected");
		return ERROR_FAIL;
	}

	LOG_DEBUG("current transport %s", transport->name);

	/* get selected transport as enum */
	tr = STLINK_TRANSPORT_UNKNOWN;

	if (strcmp(transport->name, "stlink_swd") == 0)
		tr = STLINK_TRANSPORT_SWD;
	else if (strcmp(transport->name, "stlink_jtag") == 0)
		tr = STLINK_TRANSPORT_JTAG;
	else if (strcmp(transport->name, "stlink_swim") == 0)
		tr = STLINK_TRANSPORT_SWIM;

	int retval = stlink_interface_open(tr);

	if (retval != ERROR_OK)
		return retval;

	return stlink_interface_init_target(t);
}

static int stlink_transport_select(struct command_context *ctx)
{
	LOG_DEBUG("stlink_transport_select");

	int retval;

	/* NOTE:  interface init must already have been done.
	 * That works with only C code ... no Tcl glue required.
	 */

	retval = stlink_transport_register_commands(ctx);

	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static struct transport stlink_swd_transport = {
	.name = "stlink_swd",
	.select = stlink_transport_select,
	.init = stlink_transport_init,
};

static struct transport stlink_jtag_transport = {
	.name = "stlink_jtag",
	.select = stlink_transport_select,
	.init = stlink_transport_init,
};

static struct transport stlink_swim_transport = {
	.name = "stlink_swim",
	.select = stlink_transport_select,
	.init = stlink_transport_init,
};

const char *stlink_transports[] = { "stlink_swd", "stlink_jtag", "stlink_swim", NULL };

static void stlink_constructor(void) __attribute__ ((constructor));
static void stlink_constructor(void)
{
	transport_register(&stlink_swd_transport);
	transport_register(&stlink_jtag_transport);
	transport_register(&stlink_swim_transport);
}

bool transport_is_stlink(void)
{
	return get_current_transport() == &stlink_swd_transport;
}
