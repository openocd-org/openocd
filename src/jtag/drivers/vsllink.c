/***************************************************************************
 *   Copyright (C) 2009-2010 by Simon Qian <SimonQian@SimonQian.com>       *
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

/* Versaloon is a programming tool for multiple MCUs.
 * It's distributed under GPLv3.
 * You can find it at http://www.Versaloon.com/.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include <jtag/commands.h>
#include "usb_common.h"

#include "versaloon/versaloon_include.h"
#include "versaloon/versaloon.h"

static int vsllink_tms_offset;

struct pending_scan_result {
	int src_offset;
	int dest_offset;
	int length;	/* Number of bits to read */
	struct scan_command *command;	/* Corresponding scan command */
	uint8_t *ack;
	uint8_t *buffer;
	bool last;	/* indicate the last scan pending */
};

#define MAX_PENDING_SCAN_RESULTS 256

static int pending_scan_results_length;
static struct pending_scan_result
	pending_scan_results_buffer[MAX_PENDING_SCAN_RESULTS];

/* Queue command functions */
static void vsllink_end_state(tap_state_t state);
static void vsllink_state_move(void);
static void vsllink_path_move(int num_states, tap_state_t *path);
static void vsllink_tms(int num_bits, const uint8_t *bits);
static void vsllink_runtest(int num_cycles);
static void vsllink_stableclocks(int num_cycles, int tms);
static void vsllink_scan(bool ir_scan, enum scan_type type,
		uint8_t *buffer, int scan_size, struct scan_command *command);
static void vsllink_reset(int trst, int srst);

/* VSLLink tap buffer functions */
static void vsllink_tap_append_step(int tms, int tdi);
static void vsllink_tap_init(void);
static int  vsllink_tap_execute(void);
static void vsllink_tap_ensure_pending(int scans);
static void vsllink_tap_append_scan(int length, uint8_t *buffer,
		struct scan_command *command);

/* VSLLink lowlevel functions */
struct vsllink {
	struct usb_dev_handle *usb_handle;
};

static struct vsllink *vsllink_usb_open(void);
static void vsllink_usb_close(struct vsllink *vsllink);

#if defined _DEBUG_JTAG_IO_
static void vsllink_debug_buffer(uint8_t *buffer, int length);
#endif

static int tap_length;
static int tap_buffer_size;
static uint8_t *tms_buffer;
static uint8_t *tdi_buffer;
static uint8_t *tdo_buffer;

struct vsllink *vsllink_handle;

static int vsllink_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;
	int scan_size;
	enum scan_type type;
	uint8_t *buffer;

	DEBUG_JTAG_IO("-------------------------------------"
		" vsllink "
		"-------------------------------------");

	while (cmd != NULL) {
		switch (cmd->type) {
			case JTAG_RUNTEST:
				DEBUG_JTAG_IO("runtest %i cycles, end in %s",
						cmd->cmd.runtest->num_cycles,
						tap_state_name(cmd->cmd.runtest->end_state));

				vsllink_end_state(cmd->cmd.runtest->end_state);
				vsllink_runtest(cmd->cmd.runtest->num_cycles);
				break;

			case JTAG_TLR_RESET:
				DEBUG_JTAG_IO("statemove end in %s",
						tap_state_name(cmd->cmd.statemove->end_state));

				vsllink_end_state(cmd->cmd.statemove->end_state);
				vsllink_state_move();
				break;

			case JTAG_PATHMOVE:
				DEBUG_JTAG_IO("pathmove: %i states, end in %s",
						cmd->cmd.pathmove->num_states,
						tap_state_name(cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]));

				vsllink_path_move(cmd->cmd.pathmove->num_states, cmd->cmd.pathmove->path);
				break;

			case JTAG_SCAN:
				DEBUG_JTAG_IO("JTAG Scan...");

				vsllink_end_state(cmd->cmd.scan->end_state);

				scan_size = jtag_build_buffer(
						cmd->cmd.scan, &buffer);

				if (cmd->cmd.scan->ir_scan)
					DEBUG_JTAG_IO(
							"JTAG Scan write IR(%d bits), "
							"end in %s:",
							scan_size,
							tap_state_name(cmd->cmd.scan->end_state));

				else
					DEBUG_JTAG_IO(
							"JTAG Scan write DR(%d bits), "
							"end in %s:",
							scan_size,
							tap_state_name(cmd->cmd.scan->end_state));

#ifdef _DEBUG_JTAG_IO_
				vsllink_debug_buffer(buffer,
						DIV_ROUND_UP(scan_size, 8));
#endif

				type = jtag_scan_type(cmd->cmd.scan);

				vsllink_scan(cmd->cmd.scan->ir_scan,
						type, buffer, scan_size,
						cmd->cmd.scan);
				break;

			case JTAG_RESET:
				DEBUG_JTAG_IO("reset trst: %i srst %i",
						cmd->cmd.reset->trst,
						cmd->cmd.reset->srst);

				vsllink_tap_execute();

				if (cmd->cmd.reset->trst == 1)
					tap_set_state(TAP_RESET);

				vsllink_reset(cmd->cmd.reset->trst,
						cmd->cmd.reset->srst);
				break;

			case JTAG_SLEEP:
				DEBUG_JTAG_IO("sleep %i", cmd->cmd.sleep->us);
				vsllink_tap_execute();
				jtag_sleep(cmd->cmd.sleep->us);
				break;

			case JTAG_STABLECLOCKS:
				DEBUG_JTAG_IO("add %d clocks",
						cmd->cmd.stableclocks->num_cycles);

				switch (tap_get_state()) {
				case TAP_RESET:
					/* tms must be '1' to stay
					 * n TAP_RESET mode
					 */
					scan_size = 1;
					break;
				case TAP_DRSHIFT:
				case TAP_IDLE:
				case TAP_DRPAUSE:
				case TAP_IRSHIFT:
				case TAP_IRPAUSE:
					/* else, tms should be '0' */
					scan_size = 0;
					break;
				/* above stable states are OK */
				default:
					LOG_ERROR("jtag_add_clocks() "
							"in non-stable state \"%s\"",
							tap_state_name(tap_get_state())
					);
					exit(-1);
				}
				vsllink_stableclocks(cmd->cmd.stableclocks->num_cycles, scan_size);
				break;

				case JTAG_TMS:
					DEBUG_JTAG_IO("add %d jtag tms",
							cmd->cmd.tms->num_bits);

					vsllink_tms(cmd->cmd.tms->num_bits, cmd->cmd.tms->bits);
					break;

				default:
					LOG_ERROR("BUG: unknown JTAG command type "
							"encountered: %d", cmd->type);
					exit(-1);
		}
		cmd = cmd->next;
	}

	return vsllink_tap_execute();
}

static int vsllink_speed(int speed)
{
	versaloon_interface.adaptors.jtag_raw.config(0, (uint16_t)speed);
	return versaloon_interface.adaptors.peripheral_commit();
}

static int vsllink_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz;

	return ERROR_OK;
}

static int vsllink_speed_div(int jtag_speed, int *khz)
{
	*khz = jtag_speed;

	return ERROR_OK;
}

static void vsllink_free_buffer(void)
{
	if (tdi_buffer != NULL) {
		free(tdi_buffer);
		tdi_buffer = NULL;
	}
	if (tdo_buffer != NULL) {
		free(tdo_buffer);
		tdo_buffer = NULL;
	}
	if (tms_buffer != NULL) {
		free(tms_buffer);
		tms_buffer = NULL;
	}
}

static int vsllink_quit(void)
{
	versaloon_interface.adaptors.gpio.config(0, GPIO_SRST | GPIO_TRST,
		0, 0, GPIO_SRST | GPIO_TRST);
	versaloon_interface.adaptors.gpio.fini(0);
	versaloon_interface.adaptors.jtag_raw.fini(0);
	versaloon_interface.adaptors.peripheral_commit();
	versaloon_interface.fini();

	vsllink_free_buffer();
	vsllink_usb_close(vsllink_handle);

	return ERROR_OK;
}

static int vsllink_init(void)
{
	vsllink_handle = vsllink_usb_open();
	if (vsllink_handle == 0) {
		LOG_ERROR("Can't find USB JTAG Interface!" \
			"Please check connection and permissions.");
		return ERROR_JTAG_INIT_FAILED;
	}
	LOG_DEBUG("vsllink found on %04X:%04X",
		versaloon_interface.usb_setting.vid,
		versaloon_interface.usb_setting.pid);
	versaloon_usb_device_handle = vsllink_handle->usb_handle;

	if (ERROR_OK != versaloon_interface.init())
		return ERROR_FAIL;
	if (versaloon_interface.usb_setting.buf_size < 32) {
		versaloon_interface.fini();
		return ERROR_FAIL;
	}

	/* malloc buffer size for tap */
	tap_buffer_size = versaloon_interface.usb_setting.buf_size / 2 - 32;
	vsllink_free_buffer();
	tdi_buffer = malloc(tap_buffer_size);
	tdo_buffer = malloc(tap_buffer_size);
	tms_buffer = malloc(tap_buffer_size);
	if ((NULL == tdi_buffer) || (NULL == tdo_buffer) || (NULL == tms_buffer)) {
		vsllink_quit();
		return ERROR_FAIL;
	}

	versaloon_interface.adaptors.jtag_raw.init(0);
	versaloon_interface.adaptors.jtag_raw.config(0, jtag_get_speed_khz());
	versaloon_interface.adaptors.gpio.init(0);
	versaloon_interface.adaptors.gpio.config(0, GPIO_SRST | GPIO_TRST,
		GPIO_TRST, GPIO_SRST, GPIO_SRST);
	if (ERROR_OK != versaloon_interface.adaptors.peripheral_commit())
		return ERROR_FAIL;

	vsllink_reset(0, 0);
	vsllink_tap_init();
	return ERROR_OK;
}

/**************************************************************************
 * Queue command implementations */

static void vsllink_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else {
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

/* Goes to the end state. */
static void vsllink_state_move(void)
{
	int i;
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(),
			tap_get_end_state());
	uint8_t tms_scan_bits = tap_get_tms_path_len(tap_get_state(),
			tap_get_end_state());

	for (i = 0; i < tms_scan_bits; i++)
		vsllink_tap_append_step((tms_scan >> i) & 1, 0);

	tap_set_state(tap_get_end_state());
}

static void vsllink_path_move(int num_states, tap_state_t *path)
{
	for (int i = 0; i < num_states; i++) {
		if (path[i] == tap_state_transition(tap_get_state(), false))
			vsllink_tap_append_step(0, 0);
		else if (path[i] == tap_state_transition(tap_get_state(), true))
			vsllink_tap_append_step(1, 0);
		else {
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition",
				tap_state_name(tap_get_state()),
				tap_state_name(path[i]));
			exit(-1);
		}

		tap_set_state(path[i]);
	}

	tap_set_end_state(tap_get_state());
}

static void vsllink_tms(int num_bits, const uint8_t *bits)
{
	for (int i = 0; i < num_bits; i++)
		vsllink_tap_append_step((bits[i / 8] >> (i % 8)) & 1, 0);
}

static void vsllink_stableclocks(int num_cycles, int tms)
{
	while (num_cycles > 0) {
		vsllink_tap_append_step(tms, 0);
		num_cycles--;
	}
}

static void vsllink_runtest(int num_cycles)
{
	tap_state_t saved_end_state = tap_get_end_state();

	if (tap_get_state() != TAP_IDLE) {
		/* enter IDLE state */
		vsllink_end_state(TAP_IDLE);
		vsllink_state_move();
	}

	vsllink_stableclocks(num_cycles, 0);

	/* post-process */
	/* set end_state */
	vsllink_end_state(saved_end_state);
	if (tap_get_end_state() != tap_get_end_state())
		vsllink_state_move();
}

static void vsllink_scan(bool ir_scan, enum scan_type type, uint8_t *buffer,
	int scan_size, struct scan_command *command)
{
	tap_state_t saved_end_state;

	saved_end_state = tap_get_end_state();

	/* Move to appropriate scan state */
	vsllink_end_state(ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT);

	if (tap_get_state() != tap_get_end_state())
		vsllink_state_move();
	vsllink_end_state(saved_end_state);

	/* Scan */
	vsllink_tap_append_scan(scan_size, buffer, command);

	/* Goto Pause and record position to insert tms:0 */
	vsllink_tap_append_step(0, 0);
	vsllink_tms_offset = tap_length;

	tap_set_state(ir_scan ? TAP_IRPAUSE : TAP_DRPAUSE);

	if (tap_get_state() != tap_get_end_state())
		vsllink_state_move();
}

static void vsllink_reset(int trst, int srst)
{
	LOG_DEBUG("trst: %i, srst: %i", trst, srst);

	if (!srst)
		versaloon_interface.adaptors.gpio.config(0, GPIO_SRST, 0, GPIO_SRST, GPIO_SRST);
	else
		versaloon_interface.adaptors.gpio.config(0, GPIO_SRST, GPIO_SRST, 0, 0);

	if (!trst)
		versaloon_interface.adaptors.gpio.out(0, GPIO_TRST, GPIO_TRST);
	else
		versaloon_interface.adaptors.gpio.out(0, GPIO_TRST, 0);
	versaloon_interface.adaptors.peripheral_commit();
}

COMMAND_HANDLER(vsllink_handle_usb_vid_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0],
		versaloon_interface.usb_setting.vid);
	return ERROR_OK;
}

COMMAND_HANDLER(vsllink_handle_usb_pid_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0],
		versaloon_interface.usb_setting.pid);
	return ERROR_OK;
}

COMMAND_HANDLER(vsllink_handle_usb_serial_command)
{
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	free(versaloon_interface.usb_setting.serialstring);

	if (CMD_ARGC == 1)
		versaloon_interface.usb_setting.serialstring = strdup(CMD_ARGV[0]);
	else
		versaloon_interface.usb_setting.serialstring = NULL;

	return ERROR_OK;
}

COMMAND_HANDLER(vsllink_handle_usb_bulkin_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0],
		versaloon_interface.usb_setting.ep_in);

	versaloon_interface.usb_setting.ep_in |= 0x80;

	return ERROR_OK;
}

COMMAND_HANDLER(vsllink_handle_usb_bulkout_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0],
		versaloon_interface.usb_setting.ep_out);

	versaloon_interface.usb_setting.ep_out &= ~0x80;

	return ERROR_OK;
}

COMMAND_HANDLER(vsllink_handle_usb_interface_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0],
		versaloon_interface.usb_setting.interface);
	return ERROR_OK;
}

/**************************************************************************
 * VSLLink tap functions */

static void vsllink_tap_init(void)
{
	tap_length = 0;
	pending_scan_results_length = 0;
	vsllink_tms_offset = 0;
}

static void vsllink_tap_ensure_pending(int scans)
{
	int available_scans =
		MAX_PENDING_SCAN_RESULTS - pending_scan_results_length;

	if (scans > available_scans)
		vsllink_tap_execute();
}

static void vsllink_tap_append_step(int tms, int tdi)
{
	int index_var = tap_length / 8;

	int bit_index = tap_length % 8;
	uint8_t bit = 1 << bit_index;

	if (tms)
		tms_buffer[index_var] |= bit;
	else
		tms_buffer[index_var] &= ~bit;

	if (tdi)
		tdi_buffer[index_var] |= bit;
	else
		tdi_buffer[index_var] &= ~bit;

	tap_length++;

	if (tap_buffer_size * 8 <= tap_length)
		vsllink_tap_execute();
}

static void vsllink_tap_append_scan(int length, uint8_t *buffer,
	struct scan_command *command)
{
	struct pending_scan_result *pending_scan_result;
	int len_tmp, len_all, i;

	len_all = 0;
	while (len_all < length) {
		vsllink_tap_ensure_pending(1);
		pending_scan_result =
			&pending_scan_results_buffer[
				pending_scan_results_length];

		if ((length - len_all) > (tap_buffer_size * 8 - tap_length)) {
			/* Use all memory available
			   vsllink_tap_append_step will commit automatically */
			len_tmp = tap_buffer_size * 8 - tap_length;
			pending_scan_result->last = false;
		} else {
			len_tmp = length - len_all;
			pending_scan_result->last = true;
		}
		pending_scan_result->src_offset = tap_length;
		pending_scan_result->dest_offset = len_all;
		pending_scan_result->length = len_tmp;
		pending_scan_result->command = command;
		pending_scan_result->buffer = buffer;
		pending_scan_results_length++;

		for (i = 0; i < len_tmp; i++) {
			vsllink_tap_append_step(((len_all + i) < length-1
						 ? 0 : 1),
				(buffer[(len_all + i)/8]
				 >> ((len_all + i)%8)) & 1);
		}

		len_all += len_tmp;
	}
}

static int vsllink_jtag_execute(void)
{
	int i;
	int result;

	if (tap_length <= 0)
		return ERROR_OK;

	versaloon_interface.adaptors.jtag_raw.execute(0, tdi_buffer, tms_buffer,
		tdo_buffer, tap_length);

	result = versaloon_interface.adaptors.peripheral_commit();

	if (result == ERROR_OK) {
		for (i = 0; i < pending_scan_results_length; i++) {
			struct pending_scan_result *pending_scan_result =
				&pending_scan_results_buffer[i];
			uint8_t *buffer = pending_scan_result->buffer;
			int length = pending_scan_result->length;
			int src_first = pending_scan_result->src_offset;
			int dest_first = pending_scan_result->dest_offset;
			bool last = pending_scan_result->last;

			struct scan_command *command;

			command = pending_scan_result->command;
			buf_set_buf(tdo_buffer, src_first, buffer, dest_first, length);

#ifdef _DEBUG_JTAG_IO_
			DEBUG_JTAG_IO(
				"JTAG scan read(%d bits, from src %d bits to dest %d bits):",
				length, src_first, dest_first);
			vsllink_debug_buffer(buffer + dest_first / 8,
				DIV_ROUND_UP(length, 7));
#endif

			if (last) {
				if (jtag_read_buffer(buffer, command)
				    != ERROR_OK) {
					vsllink_tap_init();
					return ERROR_JTAG_QUEUE_FAILED;
				}

				if (pending_scan_result->buffer != NULL)
					free(pending_scan_result->buffer);
			}
		}
	} else {
		LOG_ERROR("vsllink_jtag_execute failure");
		return ERROR_JTAG_QUEUE_FAILED;
	}

	vsllink_tap_init();

	return ERROR_OK;
}

static int vsllink_tap_execute(void)
{
	return vsllink_jtag_execute();
}

/****************************************************************************
 * VSLLink USB low-level functions */

static uint8_t usb_check_string(usb_dev_handle *usb, uint8_t stringidx,
	char *string, char *buff, uint16_t buf_size)
{
	int len;
	uint8_t alloced = 0;
	uint8_t ret = 1;

	if (NULL == buff) {
		buf_size = 256;
		buff = malloc(buf_size);
		if (NULL == buff) {
			ret = 0;
			goto free_and_return;
		}
		alloced = 1;
	}

	strcpy(buff, "");
	len = usb_get_string_simple(usb, stringidx, buff, buf_size);
	if ((len < 0) || ((size_t)len != strlen(buff))) {
		ret = 0;
		goto free_and_return;
	}

	buff[len] = '\0';
	if ((string != NULL) && strcmp(buff, string)) {
		ret = 0;
		goto free_and_return;
	}

free_and_return:
	if (alloced && (buff != NULL)) {
		free(buff);
		buff = NULL;
	}
	return ret;
}

static usb_dev_handle *find_usb_device(uint16_t VID, uint16_t PID, uint8_t interface,
		char *serialstring, char *productstring)
{
	usb_dev_handle *dev_handle = NULL;
	struct usb_bus *busses;
	struct usb_bus *bus;
	struct usb_device *dev;

	usb_init();
	usb_find_busses();
	usb_find_devices();
	busses = usb_get_busses();

	for (bus = busses; bus; bus = bus->next) {
		for (dev = bus->devices; dev; dev = dev->next) {
			if ((dev->descriptor.idVendor == VID)
			    && (dev->descriptor.idProduct == PID)) {
				dev_handle = usb_open(dev);
				if (NULL == dev_handle) {
					LOG_ERROR("failed to open %04X:%04X, %s", VID, PID,
						usb_strerror());
					continue;
				}

				/* check description string */
				if ((productstring != NULL && !usb_check_string(dev_handle,
						dev->descriptor.iProduct, productstring, NULL, 0))
					|| (serialstring != NULL && !usb_check_string(dev_handle,
								dev->descriptor.iSerialNumber, serialstring, NULL, 0))) {
					usb_close(dev_handle);
					dev_handle = NULL;
					continue;
				}

				if (usb_claim_interface(dev_handle, interface) != 0) {
					LOG_ERROR(ERRMSG_FAILURE_OPERATION_MESSAGE,
						"claim interface", usb_strerror());
					usb_close(dev_handle);
					dev_handle = NULL;
					continue;
				}

				if (dev_handle != NULL)
					return dev_handle;
			}
		}
	}

	return dev_handle;
}

static struct vsllink *vsllink_usb_open(void)
{
	usb_init();

	struct usb_dev_handle *dev;

	dev = find_usb_device(versaloon_interface.usb_setting.vid,
			versaloon_interface.usb_setting.pid,
			versaloon_interface.usb_setting.interface,
			versaloon_interface.usb_setting.serialstring, "Versaloon");
	if (NULL == dev)
		return NULL;

	struct vsllink *result = malloc(sizeof(struct vsllink));
	result->usb_handle = dev;
	return result;
}

static void vsllink_usb_close(struct vsllink *vsllink)
{
	int ret;

	ret = usb_release_interface(vsllink->usb_handle,
			versaloon_interface.usb_setting.interface);
	if (ret != 0) {
		LOG_ERROR("fail to release interface %d, %d returned",
			versaloon_interface.usb_setting.interface, ret);
		exit(-1);
	}

	ret = usb_close(vsllink->usb_handle);
	if (ret != 0) {
		LOG_ERROR("fail to close usb, %d returned", ret);
		exit(-1);
	}

	free(vsllink);
}

#define BYTES_PER_LINE  16

#if defined _DEBUG_JTAG_IO_
static void vsllink_debug_buffer(uint8_t *buffer, int length)
{
	char line[81];
	char s[4];
	int i;
	int j;

	for (i = 0; i < length; i += BYTES_PER_LINE) {
		snprintf(line, 5, "%04x", i);
		for (j = i; j < i + BYTES_PER_LINE && j < length; j++) {
			snprintf(s, 4, " %02x", buffer[j]);
			strcat(line, s);
		}
		LOG_DEBUG("%s", line);
	}
}
#endif	/* _DEBUG_JTAG_IO_ */

static const struct command_registration vsllink_command_handlers[] = {
	{
		.name = "vsllink_usb_vid",
		.handler = &vsllink_handle_usb_vid_command,
		.mode = COMMAND_CONFIG,
	},
	{
		.name = "vsllink_usb_pid",
		.handler = &vsllink_handle_usb_pid_command,
		.mode = COMMAND_CONFIG,
	},
	{
		.name = "vsllink_usb_serial",
		.handler = &vsllink_handle_usb_serial_command,
		.mode = COMMAND_CONFIG,
	},
	{
		.name = "vsllink_usb_bulkin",
		.handler = &vsllink_handle_usb_bulkin_command,
		.mode = COMMAND_CONFIG,
	},
	{
		.name = "vsllink_usb_bulkout",
		.handler = &vsllink_handle_usb_bulkout_command,
		.mode = COMMAND_CONFIG,
	},
	{
		.name = "vsllink_usb_interface",
		.handler = &vsllink_handle_usb_interface_command,
		.mode = COMMAND_CONFIG,
	},
	COMMAND_REGISTRATION_DONE
};

static const char *vsllink_transports[] = {"jtag", "swd", NULL};

struct jtag_interface vsllink_interface = {
	.name = "vsllink",
	.supported = DEBUG_CAP_TMS_SEQ,
	.commands = vsllink_command_handlers,
	.transports = vsllink_transports,

	.init = vsllink_init,
	.quit = vsllink_quit,
	.khz = vsllink_khz,
	.speed = vsllink_speed,
	.speed_div = vsllink_speed_div,
	.execute_queue = vsllink_execute_queue,
};
