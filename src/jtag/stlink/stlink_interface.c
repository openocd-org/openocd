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
#include <transport/transport.h>
#include <helper/time_support.h>

#include <jtag/stlink/stlink_tcl.h>
#include <jtag/stlink/stlink_layout.h>
#include <jtag/stlink/stlink_transport.h>
#include <jtag/stlink/stlink_interface.h>

#include <target/target.h>

static struct stlink_interface_s stlink_if = { {0, 0, 0, 0, 0, 0}, 0, 0 };

int stlink_interface_open(enum stlink_transports tr)
{
	LOG_DEBUG("stlink_interface_open");

	/* set transport mode */
	stlink_if.param.transport = tr;

	return stlink_if.layout->open(&stlink_if);
}

int stlink_interface_init_target(struct target *t)
{
	int res;

	LOG_DEBUG("stlink_interface_init_target");

	/* this is the interface for the current target and we
	 * can setup the private pointer in the tap structure
	 * if the interface match the tap idcode
	 */
	res = stlink_if.layout->api->idcode(stlink_if.fd, &t->tap->idcode);

	if (res != ERROR_OK)
		return res;

	unsigned ii, limit = t->tap->expected_ids_cnt;
	int found = 0;

	for (ii = 0; ii < limit; ii++) {
		uint32_t expected = t->tap->expected_ids[ii];

		/* treat "-expected-id 0" as a "don't-warn" wildcard */
		if (!expected || (t->tap->idcode == expected)) {
			found = 1;
			break;
		}
	}

	if (found == 0) {
		LOG_ERROR("stlink_interface_init_target: target not found: idcode: 0x%08x",
				t->tap->idcode);
		return ERROR_FAIL;
	}

	t->tap->priv = &stlink_if;
	t->tap->hasidcode = 1;

	return ERROR_OK;
}

static int stlink_interface_init(void)
{
	LOG_DEBUG("stlink_interface_init");

	/* here we can initialize the layout */
	return stlink_layout_init(&stlink_if);
}

static int stlink_interface_quit(void)
{
	LOG_DEBUG("stlink_interface_quit");

	return ERROR_OK;
}

static int stlink_interface_speed(int speed)
{
	LOG_DEBUG("stlink_interface_speed: ignore speed %d", speed);

	return ERROR_OK;
}

static int stlink_speed_div(int speed, int *khz)
{
	*khz = speed;
	return ERROR_OK;
}

static int stlink_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz;
	return ERROR_OK;
}

static int stlink_interface_execute_queue(void)
{
	LOG_DEBUG("stlink_interface_execute_queue: ignored");

	return ERROR_OK;
}

COMMAND_HANDLER(stlink_interface_handle_device_desc_command)
{
	LOG_DEBUG("stlink_interface_handle_device_desc_command");

	if (CMD_ARGC == 1) {
		stlink_if.param.device_desc = strdup(CMD_ARGV[0]);
	} else {
		LOG_ERROR
		    ("expected exactly one argument to stlink_device_desc <description>");
	}

	return ERROR_OK;
}

COMMAND_HANDLER(stlink_interface_handle_serial_command)
{
	LOG_DEBUG("stlink_interface_handle_serial_command");

	if (CMD_ARGC == 1) {
		stlink_if.param.serial = strdup(CMD_ARGV[0]);
	} else {
		LOG_ERROR
		    ("expected exactly one argument to stlink_serial <serial-number>");
	}

	return ERROR_OK;
}

COMMAND_HANDLER(stlink_interface_handle_layout_command)
{
	LOG_DEBUG("stlink_interface_handle_layout_command");

	if (CMD_ARGC != 1) {
		LOG_ERROR("Need exactly one argument to stlink_layout");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (stlink_if.layout) {
		LOG_ERROR("already specified stlink_layout %s",
			  stlink_if.layout->name);
		return (strcmp(stlink_if.layout->name, CMD_ARGV[0]) != 0)
		    ? ERROR_FAIL : ERROR_OK;
	}

	for (const struct stlink_layout *l = stlink_layout_get_list(); l->name;
	     l++) {
		if (strcmp(l->name, CMD_ARGV[0]) == 0) {
			stlink_if.layout = l;
			return ERROR_OK;
		}
	}

	LOG_ERROR("No STLINK layout '%s' found", CMD_ARGV[0]);
	return ERROR_FAIL;
}

COMMAND_HANDLER(stlink_interface_handle_vid_pid_command)
{
	LOG_DEBUG("stlink_interface_handle_vid_pid_command");

	if (CMD_ARGC != 2) {
		LOG_WARNING
		    ("ignoring extra IDs in stlink_vid_pid (maximum is 1 pair)");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], stlink_if.param.vid);
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], stlink_if.param.pid);

	return ERROR_OK;
}

COMMAND_HANDLER(stlink_interface_handle_api_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	unsigned new_api;
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], new_api);
	if ((new_api == 0) || (new_api > 2))
		return ERROR_COMMAND_SYNTAX_ERROR;

	stlink_if.param.api = new_api;

	return ERROR_OK;
}

static const struct command_registration stlink_interface_command_handlers[] = {
	{
	 .name = "stlink_device_desc",
	 .handler = &stlink_interface_handle_device_desc_command,
	 .mode = COMMAND_CONFIG,
	 .help = "set the stlink device description of the STLINK device",
	 .usage = "description_string",
	 },
	{
	 .name = "stlink_serial",
	 .handler = &stlink_interface_handle_serial_command,
	 .mode = COMMAND_CONFIG,
	 .help = "set the serial number of the STLINK device",
	 .usage = "serial_string",
	 },
	{
	 .name = "stlink_layout",
	 .handler = &stlink_interface_handle_layout_command,
	 .mode = COMMAND_CONFIG,
	 .help = "set the layout of the STLINK to usb or sg",
	 .usage = "layout_name",
	 },
	{
	 .name = "stlink_vid_pid",
	 .handler = &stlink_interface_handle_vid_pid_command,
	 .mode = COMMAND_CONFIG,
	 .help = "the vendor and product ID of the STLINK device",
	 .usage = "(vid pid)* ",
	 },
	 {
	 .name = "stlink_api",
	 .handler = &stlink_interface_handle_api_command,
	 .mode = COMMAND_CONFIG,
	 .help = "set the desired stlink api level",
	 .usage = "api version 1 or 2",
	 },
	COMMAND_REGISTRATION_DONE
};

struct jtag_interface stlink_interface = {
	.name = "stlink",
	.supported = 0,
	.commands = stlink_interface_command_handlers,
	.transports = stlink_transports,
	.init = stlink_interface_init,
	.quit = stlink_interface_quit,
	.speed = stlink_interface_speed,
	.speed_div = stlink_speed_div,
	.khz = stlink_khz,
	.execute_queue = stlink_interface_execute_queue,
};
