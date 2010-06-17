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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

/* Versaloon is a programming tool for multiple MCUs.
 * It's distributed under GPLv3.
 * You can find it at http://www.SimonQian.com/en/Versaloon.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include <jtag/commands.h>
#include "usb_common.h"

//#define _VSLLINK_IN_DEBUG_MODE_

static uint16_t vsllink_usb_vid;
static uint16_t vsllink_usb_pid;
static uint8_t  vsllink_usb_bulkout;
static uint8_t  vsllink_usb_bulkin;
static uint8_t  vsllink_usb_interface;
static int      VSLLINK_USB_TIMEOUT = 1000;

static int vsllink_tms_offset;

/* Global USB buffers */
static uint8_t *vsllink_usb_in_buffer;
static uint8_t *vsllink_usb_out_buffer;
static int      vsllink_buffer_size    = 128;

/* Constants for Versaloon command */
#define VERSALOON_GET_INFO				0x00
#define VERSALOON_GET_TVCC				0x01

/* Constants for VSLLink command */
#define VSLLINK_CMD_CONN				0x80
#define VSLLINK_CMD_DISCONN				0x81
#define VSLLINK_CMD_SET_SPEED			0x82
#define VSLLINK_CMD_SET_PORT			0x90
#define VSLLINK_CMD_GET_PORT			0x91
#define VSLLINK_CMD_SET_PORTDIR			0x92
#define VSLLINK_CMD_HW_JTAGSEQCMD		0xA0
#define VSLLINK_CMD_HW_JTAGHLCMD		0xA1
#define VSLLINK_CMD_HW_SWDCMD			0xA2
#define VSLLINK_CMD_HW_JTAGRAWCMD		0xA3

#define VSLLINK_CMDJTAGSEQ_TMSBYTE		0x00
#define VSLLINK_CMDJTAGSEQ_TMSCLOCK		0x40
#define VSLLINK_CMDJTAGSEQ_SCAN			0x80

#define VSLLINK_CMDJTAGSEQ_CMDMSK		0xC0
#define VSLLINK_CMDJTAGSEQ_LENMSK		0x3F

#define JTAG_PINMSK_SRST				(1 << 0)
#define JTAG_PINMSK_TRST				(1 << 1)
#define JTAG_PINMSK_USR1				(1 << 2)
#define JTAG_PINMSK_USR2				(1 << 3)
#define JTAG_PINMSK_TCK					(1 << 4)
#define JTAG_PINMSK_TMS					(1 << 5)
#define JTAG_PINMSK_TDI					(1 << 6)
#define JTAG_PINMSK_TDO					(1 << 7)

struct pending_scan_result {
	int src_offset;
	int dest_offset;
	int length; /* Number of bits to read */
	struct scan_command *command; /* Corresponding scan command */
	uint8_t *buffer;
	bool last; /* indicate the last scan pending */
};

#define MAX_PENDING_SCAN_RESULTS 256

static int pending_scan_results_length;
static struct pending_scan_result
		pending_scan_results_buffer[MAX_PENDING_SCAN_RESULTS];

/* Queue command functions */
static void vsllink_end_state(tap_state_t state);
static void vsllink_state_move(void);
static void vsllink_path_move(int num_states, tap_state_t *path);
static void vsllink_runtest(int num_cycles);
static void vsllink_stableclocks(int num_cycles, int tms);
static void vsllink_scan(bool ir_scan, enum scan_type type,
	uint8_t *buffer, int scan_size, struct scan_command *command);
static void vsllink_reset(int trst, int srst);
static void vsllink_simple_command(uint8_t command);

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
static int vsllink_usb_message(struct vsllink *vsllink, int out_length,
								int in_length);
static int vsllink_usb_write(struct vsllink *vsllink, int out_length);
static int vsllink_usb_read(struct vsllink *vsllink);

#if defined _DEBUG_USB_COMMS_ || defined _DEBUG_JTAG_IO_
static void vsllink_debug_buffer(uint8_t *buffer, int length);
#endif

static int tap_length;
static int tap_buffer_size;
static uint8_t *tms_buffer;
static uint8_t *tdi_buffer;
static uint8_t *tdo_buffer;

static struct vsllink *vsllink_handle;

static void reset_command_pointer(void)
{
	tap_length = 0;
}

static int vsllink_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;
	int scan_size;
	enum scan_type type;
	uint8_t *buffer;

	DEBUG_JTAG_IO("-------------------------------------"
		" vsllink "
		"-------------------------------------");

	reset_command_pointer();
	while (cmd != NULL) {
		switch (cmd->type) {
			case JTAG_RUNTEST:
				DEBUG_JTAG_IO("runtest %i cycles, end in %s",
					cmd->cmd.runtest->num_cycles,
					tap_state_name(cmd->cmd.runtest
							->end_state));

				vsllink_end_state(cmd->cmd.runtest->end_state);
				vsllink_runtest(cmd->cmd.runtest->num_cycles);
				break;

			case JTAG_TLR_RESET:
				DEBUG_JTAG_IO("statemove end in %s",
					tap_state_name(cmd->cmd.statemove
							->end_state));

				vsllink_end_state(cmd->cmd.statemove
							->end_state);
				vsllink_state_move();
				break;

			case JTAG_PATHMOVE:
				DEBUG_JTAG_IO("pathmove: %i states, end in %s",
					cmd->cmd.pathmove->num_states,
					tap_state_name(cmd->cmd.pathmove
						->path[cmd->cmd.pathmove
							->num_states - 1]));

				vsllink_path_move(
					cmd->cmd.pathmove->num_states,
					cmd->cmd.pathmove->path);
				break;

			case JTAG_SCAN:
				vsllink_end_state(cmd->cmd.scan->end_state);

				scan_size = jtag_build_buffer(
					cmd->cmd.scan, &buffer);

				if (cmd->cmd.scan->ir_scan)
					DEBUG_JTAG_IO(
						"JTAG Scan write IR(%d bits), "
						"end in %s:",
						scan_size,
						tap_state_name(cmd->cmd.scan
								->end_state));

				else
					DEBUG_JTAG_IO(
						"JTAG Scan write DR(%d bits), "
						"end in %s:",
						scan_size,
						tap_state_name(cmd->cmd.scan
							->end_state));

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
				vsllink_stableclocks(cmd->cmd.stableclocks
						->num_cycles, scan_size);
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
	int result;

	vsllink_usb_out_buffer[0] = VSLLINK_CMD_SET_SPEED;
	vsllink_usb_out_buffer[1] = (speed >> 0) & 0xff;
	vsllink_usb_out_buffer[2] = (speed >> 8) & 0xFF;

	result = vsllink_usb_write(vsllink_handle, 3);

	if (result == 3)
		return ERROR_OK;
	else {
		LOG_ERROR("VSLLink setting speed failed (%d)", result);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
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

static int vsllink_init(void)
{
	int check_cnt, to_tmp;
	int result;
	char version_str[100];

	vsllink_usb_in_buffer = malloc(vsllink_buffer_size);
	vsllink_usb_out_buffer = malloc(vsllink_buffer_size);
	if ((vsllink_usb_in_buffer == NULL)
			|| (vsllink_usb_out_buffer == NULL)) {
		LOG_ERROR("Not enough memory");
		exit(-1);
	}

	vsllink_handle = vsllink_usb_open();
	if (vsllink_handle == 0) {
		LOG_ERROR("Can't find USB JTAG Interface!"\
				"Please check connection and permissions.");
		return ERROR_JTAG_INIT_FAILED;
	}
	LOG_DEBUG("vsllink found on %04X:%04X",
			vsllink_usb_vid, vsllink_usb_pid);

	to_tmp = VSLLINK_USB_TIMEOUT;
	VSLLINK_USB_TIMEOUT = 100;
	check_cnt = 0;
	while (check_cnt < 5) {
		vsllink_simple_command(VERSALOON_GET_INFO);
		result = vsllink_usb_read(vsllink_handle);

		if (result > 2) {
			vsllink_usb_in_buffer[result] = 0;
			vsllink_buffer_size = vsllink_usb_in_buffer[0]
					+ (vsllink_usb_in_buffer[1] << 8);
			strncpy(version_str, (char *)vsllink_usb_in_buffer + 2,
					sizeof(version_str));
			LOG_INFO("%s", version_str);

			/* free the pre-allocated memory */
			free(vsllink_usb_in_buffer);
			free(vsllink_usb_out_buffer);
			vsllink_usb_in_buffer = NULL;
			vsllink_usb_out_buffer = NULL;

			/* alloc new memory */
			vsllink_usb_in_buffer = malloc(vsllink_buffer_size);
			vsllink_usb_out_buffer = malloc(vsllink_buffer_size);
			if ((vsllink_usb_in_buffer == NULL) ||
				(vsllink_usb_out_buffer == NULL)) {
				LOG_ERROR("Not enough memory");
				exit(-1);
			} else
				LOG_INFO("buffer size for USB is %d bytes",
							vsllink_buffer_size);

			/* alloc tms/tdi/tdo buffer */
			tap_buffer_size = (vsllink_buffer_size - 3) / 2;
			tms_buffer = (uint8_t *)malloc(tap_buffer_size);
			tdi_buffer = (uint8_t *)malloc(tap_buffer_size);
			tdo_buffer = (uint8_t *)malloc(tap_buffer_size);
			if ((tms_buffer == NULL) || (tdi_buffer == NULL) ||
				(tdo_buffer == NULL)) {
				LOG_ERROR("Not enough memory");
				exit(-1);
			}
			break;
		}
		vsllink_simple_command(VSLLINK_CMD_DISCONN);
		check_cnt++;
	}
	if (check_cnt == 3) {
		/* Fail to access Versaloon */
		LOG_ERROR("VSLLink initial failed");
		exit(-1);
	}
	VSLLINK_USB_TIMEOUT = to_tmp;

	/* Some older firmware versions sometimes fail if the
	 * voltage isn't read first.
	 */
	vsllink_simple_command(VERSALOON_GET_TVCC);
	result = vsllink_usb_read(vsllink_handle);
	if (result != 2)
		LOG_WARNING("Fail to get target voltage");
	else
		LOG_INFO("Target runs at %d mV", vsllink_usb_in_buffer[0] +
					(vsllink_usb_in_buffer[1] << 8));

	/* connect to vsllink */
	vsllink_usb_out_buffer[0] = VSLLINK_CMD_CONN;
	vsllink_usb_out_buffer[1] = 1;
	vsllink_usb_message(vsllink_handle, 2, 0);
	if (vsllink_usb_read(vsllink_handle) > 2) {
		strncpy(version_str, (char *)vsllink_usb_in_buffer + 2,
				sizeof(version_str));
		LOG_INFO("%s", version_str);
	}

	/* Set SRST and TRST to output, Set USR1 and USR2 to input */
	vsllink_usb_out_buffer[0] = VSLLINK_CMD_SET_PORTDIR;
	vsllink_usb_out_buffer[1] = JTAG_PINMSK_SRST | JTAG_PINMSK_TRST
				| JTAG_PINMSK_USR1 | JTAG_PINMSK_USR2;
	vsllink_usb_out_buffer[2] = JTAG_PINMSK_SRST | JTAG_PINMSK_TRST;
	if (vsllink_usb_write(vsllink_handle, 3) != 3) {
		LOG_ERROR("VSLLink USB send data error");
		exit(-1);
	}

	vsllink_reset(0, 0);

	LOG_INFO("VSLLink Interface ready");

	vsllink_tap_init();

	return ERROR_OK;
}

static int vsllink_quit(void)
{
	if ((vsllink_usb_in_buffer != NULL)
			&& (vsllink_usb_out_buffer != NULL)) {
		// Set all pins to input
		vsllink_usb_out_buffer[0] = VSLLINK_CMD_SET_PORTDIR;
		vsllink_usb_out_buffer[1] = JTAG_PINMSK_SRST | JTAG_PINMSK_TRST
				| JTAG_PINMSK_USR1 | JTAG_PINMSK_USR2;
		vsllink_usb_out_buffer[2] = 0;
		if (vsllink_usb_write(vsllink_handle, 3) != 3) {
			LOG_ERROR("VSLLink USB send data error");
			exit(-1);
		}

		// disconnect
		vsllink_simple_command(VSLLINK_CMD_DISCONN);
		vsllink_usb_close(vsllink_handle);
		vsllink_handle = NULL;
	}

	if (vsllink_usb_in_buffer != NULL) {
		free(vsllink_usb_in_buffer);
		vsllink_usb_in_buffer = NULL;
	}
	if (vsllink_usb_out_buffer != NULL) {
		free(vsllink_usb_out_buffer);
		vsllink_usb_out_buffer = NULL;
	}

	return ERROR_OK;
}

/***************************************************************************/
/* Queue command implementations */

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

	// post-process
	// set end_state
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
	int result;

	LOG_DEBUG("trst: %i, srst: %i", trst, srst);

	/* Signals are active low */
	vsllink_usb_out_buffer[0] = VSLLINK_CMD_SET_PORT;
	vsllink_usb_out_buffer[1] = JTAG_PINMSK_SRST | JTAG_PINMSK_TRST;
	vsllink_usb_out_buffer[2] = 0;
	if (srst == 0)
		vsllink_usb_out_buffer[2] |= JTAG_PINMSK_SRST;
	if (trst == 0)
		vsllink_usb_out_buffer[2] |= JTAG_PINMSK_TRST;

	result = vsllink_usb_write(vsllink_handle, 3);
	if (result != 3)
		LOG_ERROR("VSLLink command VSLLINK_CMD_SET_PORT failed (%d)",
				result);
}

static void vsllink_simple_command(uint8_t command)
{
	int result;

	DEBUG_JTAG_IO("0x%02x", command);

	vsllink_usb_out_buffer[0] = command;
	result = vsllink_usb_write(vsllink_handle, 1);

	if (result != 1)
		LOG_ERROR("VSLLink command 0x%02x failed (%d)",
				command, result);
}

COMMAND_HANDLER(vsllink_handle_mode_command)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("parameter error, "
					"should be one parameter for mode");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(vsllink_handle_usb_vid_command)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("parameter error, "
					"should be one parameter for VID");
		return ERROR_OK;
	}

	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], vsllink_usb_vid);
	return ERROR_OK;
}

COMMAND_HANDLER(vsllink_handle_usb_pid_command)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("parameter error, "
					"should be one parameter for PID");
		return ERROR_OK;
	}
	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], vsllink_usb_pid);
	return ERROR_OK;
}

COMMAND_HANDLER(vsllink_handle_usb_bulkin_command)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("parameter error, "
			"should be one parameter for BULKIN endpoint");
		return ERROR_OK;
	}

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], vsllink_usb_bulkin);

	vsllink_usb_bulkin |= 0x80;

	return ERROR_OK;
}

COMMAND_HANDLER(vsllink_handle_usb_bulkout_command)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("parameter error, "
			"should be one parameter for BULKOUT endpoint");
		return ERROR_OK;
	}

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], vsllink_usb_bulkout);

	vsllink_usb_bulkout &= ~0x80;

	return ERROR_OK;
}

COMMAND_HANDLER(vsllink_handle_usb_interface_command)
{
	if (CMD_ARGC != 1) {
		LOG_ERROR("parameter error, "
			"should be one parameter for interface number");
		return ERROR_OK;
	}

	COMMAND_PARSE_NUMBER(u8, CMD_ARGV[0], vsllink_usb_interface);
	return ERROR_OK;
}

/***************************************************************************/
/* VSLLink tap functions */

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

static int vsllink_tap_execute(void)
{
	int byte_length;
	int i;
	int result;

	if (tap_length <= 0)
		return ERROR_OK;

	/* Pad data so that tap_length is divisible by 8 */
	if ((tap_length % 8) != 0) {
		if (vsllink_tms_offset > 0) {
			/* append tms:0 at vsllink_tms_offset,
			 * which is in Pause
			 */
			int start_pos = DIV_ROUND_UP(tap_length, 8) - 1;
			int end_pos = DIV_ROUND_UP(vsllink_tms_offset, 8) - 1;
			int shift_cnt = (start_pos + 1) * 8 - tap_length;
			uint8_t last_mask = ~(
				(1 << (vsllink_tms_offset % 8)) - 1);

			while (1) {
				if (start_pos == end_pos) {
					tms_buffer[start_pos] =
						(tms_buffer[start_pos]
							& ~last_mask)
						| ((tms_buffer[start_pos]
								& last_mask)
							<< shift_cnt);
					tdi_buffer[start_pos] =
						(tdi_buffer[start_pos]
							& ~last_mask)
						|
						((tdi_buffer[start_pos]
								& last_mask)
								<< shift_cnt);
					break;
				} else if (start_pos == (end_pos + 1)) {
					tms_buffer[start_pos] =
						(tms_buffer[start_pos]
							<< shift_cnt) |
						((tms_buffer[start_pos - 1]
								& last_mask)
							>> (8 - shift_cnt));
					tdi_buffer[start_pos] =
						(tdi_buffer[start_pos]
							<< shift_cnt) |
						((tdi_buffer[start_pos - 1]
								& last_mask)
							>> (8 - shift_cnt));
				} else {
					tms_buffer[start_pos] =
						(tms_buffer[start_pos]
							<< shift_cnt) |
						(tms_buffer[start_pos - 1]
							>> (8 - shift_cnt));
					tdi_buffer[start_pos] =
						(tdi_buffer[start_pos]
							<< shift_cnt) |
						(tdi_buffer[start_pos - 1]
							>> (8 - shift_cnt));
				}
				start_pos--;
			}
			tap_length = DIV_ROUND_UP(tap_length, 8) * 8;
		} else {
			/* append data at last */
			while ((tap_length % 8) != 0) {
				vsllink_tap_append_step(
					(tap_get_state() == TAP_RESET)
						? 1 : 0, 0);
			}
		}
	}
	byte_length = tap_length / 8;

	vsllink_usb_out_buffer[0] = VSLLINK_CMD_HW_JTAGRAWCMD;
	vsllink_usb_out_buffer[1] = ((byte_length * 2 + 3) >> 0) & 0xff;
	vsllink_usb_out_buffer[2] = ((byte_length * 2 + 3) >> 8) & 0xff;

	memcpy(&vsllink_usb_out_buffer[3], tdi_buffer, byte_length);
	memcpy(&vsllink_usb_out_buffer[3 + byte_length], tms_buffer,
			byte_length);

	result = vsllink_usb_message(vsllink_handle, 3 + 2 * byte_length,
			byte_length);

	if (result == byte_length) {
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
			buf_set_buf(vsllink_usb_in_buffer, src_first, buffer,
							dest_first, length);

			DEBUG_JTAG_IO("JTAG scan read(%d bits, from %d bits):",
					length, dest_first);
#ifdef _DEBUG_JTAG_IO_
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
		LOG_ERROR("vsllink_tap_execute, wrong result %d, expected %d",
					result, byte_length);
		return ERROR_JTAG_QUEUE_FAILED;
	}

	vsllink_tap_init();

	return ERROR_OK;
}

/*****************************************************************************/
/* VSLLink USB low-level functions */

static struct vsllink *vsllink_usb_open(void)
{
	usb_init();

	const uint16_t vids[] = { vsllink_usb_vid, 0 };
	const uint16_t pids[] = { vsllink_usb_pid, 0 };
	struct usb_dev_handle *dev;
	if (jtag_usb_open(vids, pids, &dev) != ERROR_OK)
		return NULL;

	/* usb_set_configuration required under win32 */
	struct usb_device *udev = usb_device(dev);
	int ret = usb_set_configuration(dev,
			udev->config[0].bConfigurationValue);
	if (ret != 0) {
		LOG_ERROR("fail to set configuration to %d (error %d)."
				"Not enough permissions for the device?",
				udev->config[0].bConfigurationValue, ret);
		return NULL;
	}
	ret = usb_claim_interface(dev, vsllink_usb_interface);
	if (ret != 0) {
		LOG_ERROR("fail to claim interface %d, %d returned",
				vsllink_usb_interface, ret);
		return NULL;
	}
#if 0
	/*
	* This makes problems under Mac OS X. And is not needed
	* under Windows. Hopefully this will not break a linux build
	*/
	usb_set_altinterface(dev, 0);
#endif

	struct vsllink *result = malloc(sizeof(struct vsllink));
	result->usb_handle = dev;
	return result;
}

static void vsllink_usb_close(struct vsllink *vsllink)
{
	int ret;

	ret = usb_release_interface(vsllink->usb_handle,
			vsllink_usb_interface);
	if (ret != 0) {
		LOG_ERROR("fail to release interface %d, %d returned",
					vsllink_usb_interface, ret);
		exit(-1);
	}

	ret = usb_close(vsllink->usb_handle);
	if (ret != 0) {
		LOG_ERROR("fail to close usb, %d returned", ret);
		exit(-1);
	}

	free(vsllink);
}

/* Send a message and receive the reply. */
static int vsllink_usb_message(struct vsllink *vsllink, int out_length,
								int in_length)
{
	int result;

	result = vsllink_usb_write(vsllink, out_length);
	if (result == out_length) {
		if (in_length > 0) {
			result = vsllink_usb_read(vsllink);
			if (result == in_length)
				return result;
			else {
				LOG_ERROR("usb_bulk_read failed "
					"(requested=%d, result=%d)",
							in_length, result);
				return -1;
			}
		}
		return 0;
	} else {
		LOG_ERROR("usb_bulk_write failed (requested=%d, result=%d)",
					out_length, result);
		return -1;
	}
}

/* Write data from out_buffer to USB. */
static int vsllink_usb_write(struct vsllink *vsllink, int out_length)
{
	int result;

	if (out_length > vsllink_buffer_size) {
		LOG_ERROR("vsllink_write illegal out_length=%d (max=%d)",
					out_length, vsllink_buffer_size);
		return -1;
	}

	result = usb_bulk_write(vsllink->usb_handle, vsllink_usb_bulkout,
		(char *)vsllink_usb_out_buffer, out_length,
				VSLLINK_USB_TIMEOUT);

	DEBUG_JTAG_IO("vsllink_usb_write, out_length = %d, result = %d",
					out_length, result);

#ifdef _DEBUG_USB_COMMS_
	LOG_DEBUG("USB out:");
	vsllink_debug_buffer(vsllink_usb_out_buffer, out_length);
#endif

#ifdef _VSLLINK_IN_DEBUG_MODE_
	usleep(100000);
#endif

	return result;
}

/* Read data from USB into in_buffer. */
static int vsllink_usb_read(struct vsllink *vsllink)
{
	int result = usb_bulk_read(vsllink->usb_handle, vsllink_usb_bulkin,
		(char *)vsllink_usb_in_buffer, vsllink_buffer_size,
		VSLLINK_USB_TIMEOUT);

	DEBUG_JTAG_IO("vsllink_usb_read, result = %d", result);

#ifdef _DEBUG_USB_COMMS_
	LOG_DEBUG("USB in:");
	vsllink_debug_buffer(vsllink_usb_in_buffer, result);
#endif
	return result;
}

#define BYTES_PER_LINE  16

#if defined _DEBUG_USB_COMMS_ || defined _DEBUG_JTAG_IO_
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
#endif /* _DEBUG_USB_COMMS_ || _DEBUG_JTAG_IO_ */

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
	{
		.name = "vsllink_mode",
		.handler = &vsllink_handle_mode_command,
		.mode = COMMAND_CONFIG,
	},
	COMMAND_REGISTRATION_DONE
};

struct jtag_interface vsllink_interface = {
	.name = "vsllink",
	.commands = vsllink_command_handlers,

	.init = vsllink_init,
	.quit = vsllink_quit,
	.khz = vsllink_khz,
	.speed = vsllink_speed,
	.speed_div = vsllink_speed_div,
	.execute_queue = vsllink_execute_queue,
};
