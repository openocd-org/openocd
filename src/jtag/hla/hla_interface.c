/***************************************************************************
 *   Copyright (C) 2011 by Mathias Kuester                                 *
 *   Mathias Kuester <kesmtp@freenet.de>                                   *
 *                                                                         *
 *   Copyright (C) 2012 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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

/* project specific includes */
#include <jtag/interface.h>
#include <transport/transport.h>
#include <helper/time_support.h>

#include <jtag/hla/hla_tcl.h>
#include <jtag/hla/hla_layout.h>
#include <jtag/hla/hla_transport.h>
#include <jtag/hla/hla_interface.h>

#include <target/target.h>

static struct hl_interface_s hl_if = { {0, 0, 0, 0, 0, HL_TRANSPORT_UNKNOWN, false, NULL, 0}, 0, 0 };

int hl_interface_open(enum hl_transports tr)
{
	LOG_DEBUG("hl_interface_open");

	enum reset_types jtag_reset_config = jtag_get_reset_config();

	if (jtag_reset_config & RESET_CNCT_UNDER_SRST) {
		if (jtag_reset_config & RESET_SRST_NO_GATING)
			hl_if.param.connect_under_reset = true;
		else
			LOG_WARNING("\'srst_nogate\' reset_config option is required");
	}

	/* set transport mode */
	hl_if.param.transport = tr;

	int result = hl_if.layout->open(&hl_if);
	if (result != ERROR_OK)
		return result;

	return hl_interface_init_reset();
}

int hl_interface_init_target(struct target *t)
{
	int res;

	LOG_DEBUG("hl_interface_init_target");

	/* this is the interface for the current target and we
	 * can setup the private pointer in the tap structure
	 * if the interface match the tap idcode
	 */
	res = hl_if.layout->api->idcode(hl_if.handle, &t->tap->idcode);

	if (res != ERROR_OK)
		return res;

	unsigned ii, limit = t->tap->expected_ids_cnt;
	int found = 0;

	for (ii = 0; ii < limit; ii++) {
		uint32_t expected = t->tap->expected_ids[ii];

		/* treat "-expected-id 0" as a "don't-warn" wildcard */
		if (!expected || !t->tap->idcode ||
		    (t->tap->idcode == expected)) {
			found = 1;
			break;
		}
	}

	if (found == 0) {
		LOG_ERROR("hl_interface_init_target: target not found: idcode: 0x%08" PRIx32,
				t->tap->idcode);
		return ERROR_FAIL;
	}

	t->tap->priv = &hl_if;
	t->tap->hasidcode = 1;

	return ERROR_OK;
}

static int hl_interface_init(void)
{
	LOG_DEBUG("hl_interface_init");

	/* here we can initialize the layout */
	return hl_layout_init(&hl_if);
}

static int hl_interface_quit(void)
{
	LOG_DEBUG("hl_interface_quit");

	if (hl_if.param.trace_f) {
		fclose(hl_if.param.trace_f);
		hl_if.param.trace_f = NULL;
	}
	hl_if.param.trace_source_hz = 0;

	return ERROR_OK;
}

static int hl_interface_execute_queue(void)
{
	LOG_DEBUG("hl_interface_execute_queue: ignored");

	return ERROR_OK;
}

int hl_interface_init_reset(void)
{
	/* incase the adapter has not already handled asserting srst
	 * we will attempt it again */
	if (hl_if.param.connect_under_reset) {
		jtag_add_reset(0, 1);
		hl_if.layout->api->assert_srst(hl_if.handle, 0);
	} else {
		jtag_add_reset(0, 0);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(hl_interface_handle_device_desc_command)
{
	LOG_DEBUG("hl_interface_handle_device_desc_command");

	if (CMD_ARGC == 1) {
		hl_if.param.device_desc = strdup(CMD_ARGV[0]);
	} else {
		LOG_ERROR("expected exactly one argument to hl_device_desc <description>");
	}

	return ERROR_OK;
}

COMMAND_HANDLER(hl_interface_handle_serial_command)
{
	LOG_DEBUG("hl_interface_handle_serial_command");

	if (CMD_ARGC == 1) {
		hl_if.param.serial = strdup(CMD_ARGV[0]);
	} else {
		LOG_ERROR("expected exactly one argument to hl_serial <serial-number>");
	}

	return ERROR_OK;
}

COMMAND_HANDLER(hl_interface_handle_layout_command)
{
	LOG_DEBUG("hl_interface_handle_layout_command");

	if (CMD_ARGC != 1) {
		LOG_ERROR("Need exactly one argument to stlink_layout");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (hl_if.layout) {
		LOG_ERROR("already specified hl_layout %s",
				hl_if.layout->name);
		return (strcmp(hl_if.layout->name, CMD_ARGV[0]) != 0)
		    ? ERROR_FAIL : ERROR_OK;
	}

	for (const struct hl_layout *l = hl_layout_get_list(); l->name;
	     l++) {
		if (strcmp(l->name, CMD_ARGV[0]) == 0) {
			hl_if.layout = l;
			return ERROR_OK;
		}
	}

	LOG_ERROR("No adapter layout '%s' found", CMD_ARGV[0]);
	return ERROR_FAIL;
}

COMMAND_HANDLER(hl_interface_handle_vid_pid_command)
{
	LOG_DEBUG("hl_interface_handle_vid_pid_command");

	if (CMD_ARGC != 2) {
		LOG_WARNING("ignoring extra IDs in hl_vid_pid (maximum is 1 pair)");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], hl_if.param.vid);
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], hl_if.param.pid);

	return ERROR_OK;
}

COMMAND_HANDLER(interface_handle_trace_command)
{
	FILE *f = NULL;
	unsigned source_hz;

	if ((CMD_ARGC < 1) || (CMD_ARGC > 2))
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], source_hz);
	if (source_hz == 0) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (CMD_ARGC == 2) {
		f = fopen(CMD_ARGV[1], "a");
		if (!f)
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	hl_if.param.trace_f = f;
	hl_if.param.trace_source_hz = source_hz;

	return ERROR_OK;
}

static const struct command_registration hl_interface_command_handlers[] = {
	{
	 .name = "hla_device_desc",
	 .handler = &hl_interface_handle_device_desc_command,
	 .mode = COMMAND_CONFIG,
	 .help = "set the a device description of the adapter",
	 .usage = "description_string",
	 },
	{
	 .name = "hla_serial",
	 .handler = &hl_interface_handle_serial_command,
	 .mode = COMMAND_CONFIG,
	 .help = "set the serial number of the adapter",
	 .usage = "serial_string",
	 },
	{
	 .name = "hla_layout",
	 .handler = &hl_interface_handle_layout_command,
	 .mode = COMMAND_CONFIG,
	 .help = "set the layout of the adapter",
	 .usage = "layout_name",
	 },
	{
	 .name = "hla_vid_pid",
	 .handler = &hl_interface_handle_vid_pid_command,
	 .mode = COMMAND_CONFIG,
	 .help = "the vendor and product ID of the adapter",
	 .usage = "(vid pid)* ",
	 },
	 {
	 .name = "trace",
	 .handler = &interface_handle_trace_command,
	 .mode = COMMAND_CONFIG,
	 .help = "configure trace reception",
	 .usage = "source_lock_hz [destination_path]",
	 },
	COMMAND_REGISTRATION_DONE
};

struct jtag_interface hl_interface = {
	.name = "hla",
	.supported = 0,
	.commands = hl_interface_command_handlers,
	.transports = hl_transports,
	.init = hl_interface_init,
	.quit = hl_interface_quit,
	.execute_queue = hl_interface_execute_queue,
};
