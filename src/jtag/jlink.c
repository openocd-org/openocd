/***************************************************************************
 *   Copyright (C) 2007 by Juergen Stuber <juergen@jstuber.net>            *
 *   based on Dominic Rath's and Benedikt Sauter's usbprog.c               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "replacements.h"

#include "jtag.h"

#include <usb.h>
#include <string.h>

#include "log.h"

#define VID 0x1366
#define PID 0x0101

#define JLINK_WRITE_ENDPOINT	0x02
#define JLINK_READ_ENDPOINT		0x81

#define JLINK_USB_TIMEOUT		1000

#define JLINK_IN_BUFFER_SIZE			8192
#define JLINK_OUT_BUFFER_SIZE			8192
#define JLINK_EMU_RESULT_BUFFER_SIZE	64

/* Global USB buffers */
static u8 usb_in_buffer[JLINK_IN_BUFFER_SIZE];
static u8 usb_out_buffer[JLINK_OUT_BUFFER_SIZE];
static u8 usb_emu_result_buffer[JLINK_EMU_RESULT_BUFFER_SIZE];

/* Constants for JLink command */
#define EMU_CMD_VERSION     0x01
#define EMU_CMD_SET_SPEED   0x05
#define EMU_CMD_GET_STATE   0x07
#define EMU_CMD_HW_JTAG3    0xcf
#define EMU_CMD_HW_RESET0   0xdc
#define EMU_CMD_HW_RESET1   0xdd
#define EMU_CMD_HW_TRST0    0xde
#define EMU_CMD_HW_TRST1    0xdf

/* max speed 12MHz v5.0 jlink */
#define JLINK_MAX_SPEED 12000

/* External interface functions */
static int jlink_execute_queue(void);
static int jlink_speed(int speed);
static int jlink_speed_div(int speed, int* khz);
static int jlink_khz(int khz, int *jtag_speed);
static int jlink_register_commands(struct command_context_s *cmd_ctx);
static int jlink_init(void);
static int jlink_quit(void);

/* CLI command handler functions */
static int jlink_handle_jlink_info_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

/* Queue command functions */
static void jlink_end_state(tap_state_t state);
static void jlink_state_move(void);
static void jlink_path_move(int num_states, tap_state_t *path);
static void jlink_runtest(int num_cycles);
static void jlink_scan(int ir_scan, enum scan_type type, u8 *buffer, int scan_size, scan_command_t *command);
static void jlink_reset(int trst, int srst);
static void jlink_simple_command(u8 command);
static int jlink_get_status(void);

/* J-Link tap buffer functions */
static void jlink_tap_init(void);
static int jlink_tap_execute(void);
static void jlink_tap_ensure_space(int scans, int bits);
static void jlink_tap_append_step(int tms, int tdi);
static void jlink_tap_append_scan(int length, u8 *buffer, scan_command_t *command);

/* Jlink lowlevel functions */
typedef struct jlink_jtag
{
	struct usb_dev_handle* usb_handle;
} jlink_jtag_t;

static jlink_jtag_t *jlink_usb_open(void);
static void jlink_usb_close(jlink_jtag_t *jlink_jtag);
static int jlink_usb_message(jlink_jtag_t *jlink_jtag, int out_length, int in_length);
static int jlink_usb_write(jlink_jtag_t *jlink_jtag, int out_length);
static int jlink_usb_read(jlink_jtag_t *jlink_jtag, int expected_size);
static int jlink_usb_read_emu_result(jlink_jtag_t *jlink_jtag);

/* helper functions */
static int jlink_get_version_info(void);

#ifdef _DEBUG_USB_COMMS_
static void jlink_debug_buffer(u8 *buffer, int length);
#endif

static jlink_jtag_t* jlink_jtag_handle;

/***************************************************************************/
/* External interface implementation */

jtag_interface_t jlink_interface =
{
	.name = "jlink",
	.execute_queue = jlink_execute_queue,
	.speed = jlink_speed,
	.speed_div = jlink_speed_div,
	.khz = jlink_khz,
	.register_commands = jlink_register_commands,
	.init = jlink_init,
	.quit = jlink_quit
};

static void jlink_execute_end_state(jtag_command_t *cmd)
{
	DEBUG_JTAG_IO("end_state: %i", cmd->cmd.end_state->end_state);

	if (cmd->cmd.end_state->end_state != TAP_INVALID)
		jlink_end_state(cmd->cmd.end_state->end_state);
}

static void jlink_execute_runtest(jtag_command_t *cmd)
{
	DEBUG_JTAG_IO("runtest %i cycles, end in %i",
			cmd->cmd.runtest->num_cycles,
			cmd->cmd.runtest->end_state);

	if (cmd->cmd.runtest->end_state != TAP_INVALID)
		jlink_end_state(cmd->cmd.runtest->end_state);

	jlink_runtest(cmd->cmd.runtest->num_cycles);
}

static void jlink_execute_statemove(jtag_command_t *cmd)
{
	DEBUG_JTAG_IO("statemove end in %i", cmd->cmd.statemove->end_state);

	if (cmd->cmd.statemove->end_state != TAP_INVALID)
	{
		jlink_end_state(cmd->cmd.statemove->end_state);
	}
	jlink_state_move();
}

static void jlink_execute_pathmove(jtag_command_t *cmd)
{
	DEBUG_JTAG_IO("pathmove: %i states, end in %i",
		cmd->cmd.pathmove->num_states,
		cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);

	jlink_path_move(cmd->cmd.pathmove->num_states,
			cmd->cmd.pathmove->path);
}

static void jlink_execute_scan(jtag_command_t *cmd)
{
	int scan_size;
	enum scan_type type;
	u8 *buffer;

	DEBUG_JTAG_IO("scan end in %i", cmd->cmd.scan->end_state);

	if (cmd->cmd.scan->end_state != TAP_INVALID)
		jlink_end_state(cmd->cmd.scan->end_state);

	scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
	DEBUG_JTAG_IO("scan input, length = %d", scan_size);

#ifdef _DEBUG_USB_COMMS_
	jlink_debug_buffer(buffer, (scan_size + 7) / 8);
#endif
	type = jtag_scan_type(cmd->cmd.scan);
	jlink_scan(cmd->cmd.scan->ir_scan,
			type, buffer, scan_size, cmd->cmd.scan);
}

static void jlink_execute_reset(jtag_command_t *cmd)
{
	DEBUG_JTAG_IO("reset trst: %i srst %i",
			cmd->cmd.reset->trst, cmd->cmd.reset->srst);

	jlink_tap_execute();

	if (cmd->cmd.reset->trst == 1)
		tap_set_state(TAP_RESET);

	jlink_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
}

static void jlink_execute_sleep(jtag_command_t *cmd)
{
	DEBUG_JTAG_IO("sleep %i", cmd->cmd.sleep->us);
	jlink_tap_execute();
	jtag_sleep(cmd->cmd.sleep->us);
}

static void jlink_execute_command(jtag_command_t *cmd)
{
	switch (cmd->type)
	{
	case JTAG_END_STATE: jlink_execute_end_state(cmd); break;
	case JTAG_RUNTEST:   jlink_execute_runtest(cmd); break;
	case JTAG_STATEMOVE: jlink_execute_statemove(cmd); break;
	case JTAG_PATHMOVE:  jlink_execute_pathmove(cmd); break;
	case JTAG_SCAN:      jlink_execute_scan(cmd); break;
	case JTAG_RESET:     jlink_execute_reset(cmd); break;
	case JTAG_SLEEP:     jlink_execute_sleep(cmd); break;
	default:
		LOG_ERROR("BUG: unknown JTAG command type encountered");
		exit(-1);
	}
}

static int jlink_execute_queue(void)
{
	jtag_command_t *cmd = jtag_command_queue;

	while (cmd != NULL)
	{
		jlink_execute_command(cmd);
		cmd = cmd->next;
	}

	return jlink_tap_execute();
}

/* Sets speed in kHz. */
static int jlink_speed(int speed)
{
	int result;

	if (speed > JLINK_MAX_SPEED)
	{
		LOG_INFO("Ignoring speed request: %dkHz exceeds %dkHz maximum",
				speed, JLINK_MAX_SPEED);
		return ERROR_OK;
	}

	/* check for RTCK setting */
	if (speed == 0)
		speed = -1;

	usb_out_buffer[0] = EMU_CMD_SET_SPEED;
	usb_out_buffer[1] = (speed >> 0) & 0xff;
	usb_out_buffer[2] = (speed >> 8) & 0xff;

	result = jlink_usb_write(jlink_jtag_handle, 3);
	if (result != 3)
	{
		LOG_ERROR("J-Link setting speed failed (%d)", result);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int jlink_speed_div(int speed, int* khz)
{
	*khz = speed;

	return ERROR_OK;
}

static int jlink_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz;

	return ERROR_OK;
}

static int jlink_register_commands(struct command_context_s *cmd_ctx)
{
	register_command(cmd_ctx, NULL, "jlink_info", jlink_handle_jlink_info_command, COMMAND_EXEC,
		"query jlink info");
	return ERROR_OK;
}

static int jlink_init(void)
{
	int check_cnt;

	jlink_jtag_handle = jlink_usb_open();

	if (jlink_jtag_handle == 0)
	{
		LOG_ERROR("Cannot find jlink Interface! Please check connection and permissions.");
		return ERROR_JTAG_INIT_FAILED;
	}

	check_cnt = 0;
	while (check_cnt < 3)
	{
		if (jlink_get_version_info() == ERROR_OK)
		{
			/* attempt to get status */
			jlink_get_status();
			break;
		}

		check_cnt++;
	}

	if (check_cnt == 3)
	{
		LOG_INFO("J-Link initial read failed, don't worry");
	}

	LOG_INFO("J-Link JTAG Interface ready");

	jlink_reset(0, 0);
	jlink_tap_init();

	return ERROR_OK;
}

static int jlink_quit(void)
{
	jlink_usb_close(jlink_jtag_handle);
	return ERROR_OK;
}

/***************************************************************************/
/* Queue command implementations */

static void jlink_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
	{
		tap_set_end_state(state);
	}
	else
	{
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

/* Goes to the end state. */
static void jlink_state_move(void)
{
	int i;
	int tms = 0;
	u8 tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());

	for (i = 0; i < 7; i++)
	{
		tms = (tms_scan >> i) & 1;
		jlink_tap_append_step(tms, 0);
	}

	tap_set_state(tap_get_end_state());
}

static void jlink_path_move(int num_states, tap_state_t *path)
{
	int i;

	for (i = 0; i < num_states; i++)
	{
		if (path[i] == tap_state_transition(tap_get_state(), false))
		{
			jlink_tap_append_step(0, 0);
		}
		else if (path[i] == tap_state_transition(tap_get_state(), true))
		{
			jlink_tap_append_step(1, 0);
		}
		else
		{
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition", tap_state_name(tap_get_state()), tap_state_name(path[i]));
			exit(-1);
		}

		tap_set_state(path[i]);
	}

	tap_set_end_state(tap_get_state());
}

static void jlink_runtest(int num_cycles)
{
	int i;

	tap_state_t saved_end_state = tap_get_end_state();

	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE)
	{
		jlink_end_state(TAP_IDLE);
		jlink_state_move();
	}

	/* execute num_cycles */
	for (i = 0; i < num_cycles; i++)
	{
		jlink_tap_append_step(0, 0);
	}

	/* finish in end_state */
	jlink_end_state(saved_end_state);
	if (tap_get_state() != tap_get_end_state())
	{
		jlink_state_move();
	}
}

static void jlink_scan(int ir_scan, enum scan_type type, u8 *buffer, int scan_size, scan_command_t *command)
{
	tap_state_t saved_end_state;

	jlink_tap_ensure_space(1, scan_size + 8);

	saved_end_state = tap_get_end_state();

	/* Move to appropriate scan state */
	jlink_end_state(ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT);

	/* Only move if we're not already there */
	if (tap_get_state() != tap_get_end_state())
		jlink_state_move();

	jlink_end_state(saved_end_state);

	/* Scan */
	jlink_tap_append_scan(scan_size, buffer, command);

	/* We are in Exit1, go to Pause */
	jlink_tap_append_step(0, 0);

	tap_set_state(ir_scan ? TAP_IRPAUSE : TAP_DRPAUSE);

	if (tap_get_state() != tap_get_end_state())
	{
		jlink_state_move();
	}
}

static void jlink_reset(int trst, int srst)
{
	LOG_DEBUG("trst: %i, srst: %i", trst, srst);

	/* Signals are active low */
	if (srst == 0)
	{
		jlink_simple_command(EMU_CMD_HW_RESET1);
	}
	else if (srst == 1)
	{
		jlink_simple_command(EMU_CMD_HW_RESET0);
	}

	if (trst == 0)
	{
		jlink_simple_command(EMU_CMD_HW_TRST1);
	}
	else if (trst == 1)
	{
		jlink_simple_command(EMU_CMD_HW_TRST0);
	}
}

static void jlink_simple_command(u8 command)
{
	int result;

	DEBUG_JTAG_IO("0x%02x", command);

	usb_out_buffer[0] = command;
	result = jlink_usb_write(jlink_jtag_handle, 1);

	if (result != 1)
	{
		LOG_ERROR("J-Link command 0x%02x failed (%d)", command, result);
	}
}

static int jlink_get_status(void)
{
	int result;

	jlink_simple_command(EMU_CMD_GET_STATE);

	result = jlink_usb_read(jlink_jtag_handle, 8);
	if (result != 8)
	{
		LOG_ERROR("J-Link command EMU_CMD_GET_STATE failed (%d)\n", result);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	int vref = usb_in_buffer[0] + (usb_in_buffer[1] << 8);
	LOG_INFO("Vref = %d.%d TCK = %d TDI = %d TDO = %d TMS = %d SRST = %d TRST = %d\n", \
		vref / 1000, vref % 1000, \
		usb_in_buffer[2], usb_in_buffer[3], usb_in_buffer[4], \
		usb_in_buffer[5], usb_in_buffer[6], usb_in_buffer[7]);

	if (vref < 1500)
		LOG_ERROR("Vref too low. Check Target Power\n");

	return ERROR_OK;
}

static int jlink_get_version_info(void)
{
	int result;
	int len;

	/* query hardware version */
	jlink_simple_command(EMU_CMD_VERSION);

	result = jlink_usb_read(jlink_jtag_handle, 2);
	if (2 != result)
	{
		LOG_ERROR("J-Link command EMU_CMD_VERSION failed (%d)\n",
				result);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	len = buf_get_u32(usb_in_buffer, 0, 16);
	result = jlink_usb_read(jlink_jtag_handle, len);
	if (result != len)
	{
		LOG_ERROR("J-Link command EMU_CMD_VERSION failed (%d)\n",
				result);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	usb_in_buffer[result] = 0;
	LOG_INFO("%s", (char *)usb_in_buffer);

	return ERROR_OK;
}

static int jlink_handle_jlink_info_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (jlink_get_version_info() == ERROR_OK)
	{
		/* attempt to get status */
		jlink_get_status();
	}

	return ERROR_OK;
}

/***************************************************************************/
/* J-Link tap functions */

/* 2048 is the max value we can use here */
#define JLINK_TAP_BUFFER_SIZE 2048

static unsigned tap_length;
static u8 tms_buffer[JLINK_TAP_BUFFER_SIZE];
static u8 tdi_buffer[JLINK_TAP_BUFFER_SIZE];
static u8 tdo_buffer[JLINK_TAP_BUFFER_SIZE];

typedef struct
{
	int first;	/* First bit position in tdo_buffer to read */
	int length; /* Number of bits to read */
	scan_command_t *command; /* Corresponding scan command */
	u8 *buffer;
} pending_scan_result_t;

#define MAX_PENDING_SCAN_RESULTS 256

static int pending_scan_results_length;
static pending_scan_result_t pending_scan_results_buffer[MAX_PENDING_SCAN_RESULTS];

static int last_tms;

static void jlink_tap_init(void)
{
	tap_length = 0;
	pending_scan_results_length = 0;
}

static void jlink_tap_ensure_space(int scans, int bits)
{
	int available_scans = MAX_PENDING_SCAN_RESULTS - pending_scan_results_length;
	int available_bits = JLINK_TAP_BUFFER_SIZE * 8 - tap_length;

	if (scans > available_scans || bits > available_bits)
	{
		jlink_tap_execute();
	}
}

static void jlink_tap_append_step(int tms, int tdi)
{
	last_tms = tms;
	int index = tap_length / 8;

	if (index >= JLINK_TAP_BUFFER_SIZE)
	{
		LOG_ERROR("jlink_tap_append_step: overflow");
		exit(-1);
	}

	int bit_index = tap_length % 8;
	u8 bit = 1 << bit_index;

	if (tms)
		tms_buffer[index] |= bit;
	else
		tms_buffer[index] &= ~bit;

	if (tdi)
		tdi_buffer[index] |= bit;
	else
		tdi_buffer[index] &= ~bit;

	tap_length++;
}

static void jlink_tap_append_scan(int length, u8 *buffer, scan_command_t *command)
{
	pending_scan_result_t *pending_scan_result =
		&pending_scan_results_buffer[pending_scan_results_length];
	int i;

	pending_scan_result->first = tap_length;
	pending_scan_result->length = length;
	pending_scan_result->command = command;
	pending_scan_result->buffer = buffer;

	for (i = 0; i < length; i++)
	{
		int tms = i < length - 1 ? 0 : 1;
		int tdi = buffer[i / 8] & (1 << (i % 8));
		jlink_tap_append_step(tms, tdi);
	}
	pending_scan_results_length++;
}

/* Pad and send a tap sequence to the device, and receive the answer.
 * For the purpose of padding we assume that we are in idle or pause state. */
static int jlink_tap_execute(void)
{
	int byte_length;
	int tms_offset;
	int tdi_offset;
	int i;
	int result;

	if (!tap_length)
		return ERROR_OK;

	/* Pad last byte so that tap_length is divisible by 8 */
	while (tap_length % 8 != 0)
	{
		/* More of the last TMS value keeps us in the same state,
		 * analogous to free-running JTAG interfaces. */
		jlink_tap_append_step(last_tms, 0);
	}

	byte_length = tap_length / 8;

	usb_out_buffer[0] = EMU_CMD_HW_JTAG3;
	usb_out_buffer[1] = 0;
	usb_out_buffer[2] = (tap_length >> 0) & 0xff;
	usb_out_buffer[3] = (tap_length >> 8) & 0xff;

	tms_offset = 4;
	for (i = 0; i < byte_length; i++)
	{
		usb_out_buffer[tms_offset + i] = tms_buffer[i];
	}

	tdi_offset = tms_offset + byte_length;
	for (i = 0; i < byte_length; i++)
	{
		usb_out_buffer[tdi_offset + i] = tdi_buffer[i];
	}

	result = jlink_usb_message(jlink_jtag_handle, 4 + 2 * byte_length, byte_length);

	if (result != byte_length)
	{
		LOG_ERROR("jlink_tap_execute, wrong result %d (expected %d)",
				result, byte_length);
		return ERROR_JTAG_QUEUE_FAILED;
	}

	for (i = 0; i < byte_length; i++)
		tdo_buffer[i] = usb_in_buffer[i];

	for (i = 0; i < pending_scan_results_length; i++)
	{
		pending_scan_result_t *pending_scan_result = &pending_scan_results_buffer[i];
		u8 *buffer = pending_scan_result->buffer;
		int length = pending_scan_result->length;
		int first = pending_scan_result->first;
		scan_command_t *command = pending_scan_result->command;

		/* Copy to buffer */
		buf_set_buf(tdo_buffer, first, buffer, 0, length);

		DEBUG_JTAG_IO("pending scan result, length = %d", length);

#ifdef _DEBUG_USB_COMMS_
		jlink_debug_buffer(buffer, byte_length);
#endif

		if (jtag_read_buffer(buffer, command) != ERROR_OK)
		{
			jlink_tap_init();
			return ERROR_JTAG_QUEUE_FAILED;
		}

		if (pending_scan_result->buffer != NULL)
		{
			free(pending_scan_result->buffer);
		}
	}

	jlink_tap_init();

	return ERROR_OK;
}

/*****************************************************************************/
/* JLink USB low-level functions */

static jlink_jtag_t* jlink_usb_open()
{
	struct usb_bus *busses;
	struct usb_bus *bus;
	struct usb_device *dev;

	jlink_jtag_t *result;

	result = (jlink_jtag_t*) malloc(sizeof(jlink_jtag_t));

	usb_init();
	usb_find_busses();
	usb_find_devices();

	busses = usb_get_busses();

	/* find jlink_jtag device in usb bus */

	for (bus = busses; bus; bus = bus->next)
	{
		for (dev = bus->devices; dev; dev = dev->next)
		{
			if ((dev->descriptor.idVendor == VID) && (dev->descriptor.idProduct == PID))
			{
				result->usb_handle = usb_open(dev);

				/* usb_set_configuration required under win32 */
				usb_set_configuration(result->usb_handle, dev->config[0].bConfigurationValue);
				usb_claim_interface(result->usb_handle, 0);

#if 0
				/*
				 * This makes problems under Mac OS X. And is not needed
				 * under Windows. Hopefully this will not break a linux build
				 */
				usb_set_altinterface(result->usb_handle, 0);
#endif
				return result;
			}
		}
	}

	free(result);
	return NULL;
}

static void jlink_usb_close(jlink_jtag_t *jlink_jtag)
{
	usb_close(jlink_jtag->usb_handle);
	free(jlink_jtag);
}

/* Send a message and receive the reply. */
static int jlink_usb_message(jlink_jtag_t *jlink_jtag, int out_length, int in_length)
{
	int result;
	int result2;

	result = jlink_usb_write(jlink_jtag, out_length);
	if (result != out_length)
	{
		LOG_ERROR("usb_bulk_write failed (requested=%d, result=%d)",
				out_length, result);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	result = jlink_usb_read(jlink_jtag, in_length);
	if ((result != in_length) && (result != in_length + 1))
	{
		LOG_ERROR("usb_bulk_read failed (requested=%d, result=%d)",
				in_length, result);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	if (result == in_length)
	{
		/* Must read the result from the EMU too */
		result2 = jlink_usb_read_emu_result(jlink_jtag);
		if (1 != result2)
		{
			LOG_ERROR("jlink_usb_read_emu_result failed "
				"(requested=1, result=%d)", result2);
			return ERROR_JTAG_DEVICE_ERROR;
		}

		/* Check the result itself */
		result2 = usb_emu_result_buffer[0];
	}
	else
	{
		/* Save the result, then remove it from return value */
		result2 = usb_in_buffer[result--];
	}

	if (result2)
	{
		LOG_ERROR("jlink_usb_message failed with result=%d)", result2);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return result;
}

/* calls the given usb_bulk_* function, allowing for the data to trickle in with some timeouts  */
static int usb_bulk_with_retries(
		int (*f)(usb_dev_handle *, int, char *, int, int),
		usb_dev_handle *dev, int ep,
		char *bytes, int size, int timeout)
{
	int rc = 0, tries = 3, this_size;

	while (tries && size) {

		this_size = f(dev, ep, bytes, size, timeout);
		if (this_size > 0) {
			
			size -= this_size;
			rc += this_size;
			bytes += this_size;

		} else
			tries --;
	}
	return rc;
}
static inline int usb_bulk_write_ex(usb_dev_handle *dev, int ep,
		char *bytes, int size, int timeout)
{
	return usb_bulk_with_retries(&usb_bulk_write,
			dev, ep, bytes, size, timeout);
}
static inline int usb_bulk_read_ex(usb_dev_handle *dev, int ep,
		char *bytes, int size, int timeout)
{
	return usb_bulk_with_retries(&usb_bulk_read,
			dev, ep, bytes, size, timeout);
}

/* Write data from out_buffer to USB. */
static int jlink_usb_write(jlink_jtag_t *jlink_jtag, int out_length)
{
	int result;

	if (out_length > JLINK_OUT_BUFFER_SIZE)
	{
		LOG_ERROR("jlink_jtag_write illegal out_length=%d (max=%d)", out_length, JLINK_OUT_BUFFER_SIZE);
		return -1;
	}

	result = usb_bulk_write_ex(jlink_jtag->usb_handle, JLINK_WRITE_ENDPOINT,
		(char *)usb_out_buffer, out_length, JLINK_USB_TIMEOUT);

	DEBUG_JTAG_IO("jlink_usb_write, out_length = %d, result = %d", out_length, result);

#ifdef _DEBUG_USB_COMMS_
	jlink_debug_buffer(usb_out_buffer, out_length);
#endif
	return result;
}

/* Read data from USB into in_buffer. */
static int jlink_usb_read(jlink_jtag_t *jlink_jtag, int expected_size)
{
	int result = usb_bulk_read_ex(jlink_jtag->usb_handle, JLINK_READ_ENDPOINT,
		(char *)usb_in_buffer, expected_size, JLINK_USB_TIMEOUT);

	DEBUG_JTAG_IO("jlink_usb_read, result = %d", result);

#ifdef _DEBUG_USB_COMMS_
	jlink_debug_buffer(usb_in_buffer, result);
#endif
	return result;
}

/* Read the result from the previous EMU cmd into result_buffer. */
static int jlink_usb_read_emu_result(jlink_jtag_t *jlink_jtag)
{
	int result = usb_bulk_read_ex(jlink_jtag->usb_handle, JLINK_READ_ENDPOINT,
		(char *)usb_emu_result_buffer, 1 /* JLINK_EMU_RESULT_BUFFER_SIZE */,
		JLINK_USB_TIMEOUT);

	DEBUG_JTAG_IO("jlink_usb_read_result, result = %d", result);

#ifdef _DEBUG_USB_COMMS_
	jlink_debug_buffer(usb_emu_result_buffer, result);
#endif
	return result;
}

#ifdef _DEBUG_USB_COMMS_
#define BYTES_PER_LINE  16

static void jlink_debug_buffer(u8 *buffer, int length)
{
	char line[81];
	char s[4];
	int i;
	int j;

	for (i = 0; i < length; i += BYTES_PER_LINE)
	{
		snprintf(line, 5, "%04x", i);
		for (j = i; j < i + BYTES_PER_LINE && j < length; j++)
		{
			snprintf(s, 4, " %02x", buffer[j]);
			strcat(line, s);
		}
		LOG_DEBUG("%s", line);
	}
}
#endif
