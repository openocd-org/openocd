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

/* enable this to debug communication
 */
#if 0
#define _DEBUG_USB_COMMS_
#endif

#ifdef _DEBUG_JTAG_IO_
#define DEBUG_JTAG_IO(expr ...)	LOG_DEBUG(expr)
#else
#define DEBUG_JTAG_IO(expr ...)
#endif

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
int jlink_execute_queue(void);
int jlink_speed(int speed);
int jlink_khz(int khz, int *jtag_speed);
int jlink_register_commands(struct command_context_s *cmd_ctx);
int jlink_init(void);
int jlink_quit(void);

/* CLI command handler functions */
int jlink_handle_jlink_info_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

/* Queue command functions */
void jlink_end_state(tap_state_t state);
void jlink_state_move(void);
void jlink_path_move(int num_states, tap_state_t *path);
void jlink_runtest(int num_cycles);
void jlink_scan(int ir_scan, enum scan_type type, u8 *buffer, int scan_size, scan_command_t *command);
void jlink_reset(int trst, int srst);
void jlink_simple_command(u8 command);
int jlink_get_status(void);

/* J-Link tap buffer functions */
void jlink_tap_init(void);
int jlink_tap_execute(void);
void jlink_tap_ensure_space(int scans, int bits);
void jlink_tap_append_step(int tms, int tdi);
void jlink_tap_append_scan(int length, u8 *buffer, scan_command_t *command);

/* Jlink lowlevel functions */
typedef struct jlink_jtag
{
	struct usb_dev_handle* usb_handle;
} jlink_jtag_t;

jlink_jtag_t *jlink_usb_open(void);
void jlink_usb_close(jlink_jtag_t *jlink_jtag);
int jlink_usb_message(jlink_jtag_t *jlink_jtag, int out_length, int in_length);
int jlink_usb_write(jlink_jtag_t *jlink_jtag, int out_length);
int jlink_usb_read(jlink_jtag_t *jlink_jtag);
int jlink_usb_read_emu_result(jlink_jtag_t *jlink_jtag);

/* helper functions */
int jlink_get_version_info(void);

#ifdef _DEBUG_USB_COMMS_
void jlink_debug_buffer(u8 *buffer, int length);
#endif

jlink_jtag_t* jlink_jtag_handle;

/***************************************************************************/
/* External interface implementation */

jtag_interface_t jlink_interface =
{
	.name = "jlink",
	.execute_queue = jlink_execute_queue,
	.speed = jlink_speed,
	.khz = jlink_khz,
	.register_commands = jlink_register_commands,
	.init = jlink_init,
	.quit = jlink_quit
};

int jlink_execute_queue(void)
{
	jtag_command_t *cmd = jtag_command_queue;
	int scan_size;
	enum scan_type type;
	u8 *buffer;

	while (cmd != NULL)
	{
		switch (cmd->type)
		{
			case JTAG_END_STATE:
				DEBUG_JTAG_IO("end_state: %i", cmd->cmd.end_state->end_state);

				if (cmd->cmd.end_state->end_state != -1)
				{
					jlink_end_state(cmd->cmd.end_state->end_state);
				}
				break;

			case JTAG_RUNTEST:
				DEBUG_JTAG_IO( "runtest %i cycles, end in %i", cmd->cmd.runtest->num_cycles, \
					cmd->cmd.runtest->end_state);

				if (cmd->cmd.runtest->end_state != -1)
				{
					jlink_end_state(cmd->cmd.runtest->end_state);
				}
				jlink_runtest(cmd->cmd.runtest->num_cycles);
				break;

			case JTAG_STATEMOVE:
				DEBUG_JTAG_IO("statemove end in %i", cmd->cmd.statemove->end_state);

				if (cmd->cmd.statemove->end_state != -1)
				{
					jlink_end_state(cmd->cmd.statemove->end_state);
				}
				jlink_state_move();
				break;

			case JTAG_PATHMOVE:
				DEBUG_JTAG_IO("pathmove: %i states, end in %i", \
					cmd->cmd.pathmove->num_states, \
					cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);

				jlink_path_move(cmd->cmd.pathmove->num_states, cmd->cmd.pathmove->path);
				break;

			case JTAG_SCAN:
				DEBUG_JTAG_IO("scan end in %i", cmd->cmd.scan->end_state);

				if (cmd->cmd.scan->end_state != -1)
				{
					jlink_end_state(cmd->cmd.scan->end_state);
				}

				scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
				DEBUG_JTAG_IO("scan input, length = %d", scan_size);

#ifdef _DEBUG_USB_COMMS_
				jlink_debug_buffer(buffer, (scan_size + 7) / 8);
#endif
				type = jtag_scan_type(cmd->cmd.scan);
				jlink_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size, cmd->cmd.scan);
				break;

			case JTAG_RESET:
				DEBUG_JTAG_IO("reset trst: %i srst %i", cmd->cmd.reset->trst, cmd->cmd.reset->srst);

				jlink_tap_execute();

				if (cmd->cmd.reset->trst == 1)
				{
					tap_set_state(TAP_RESET);
				}
				jlink_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
				break;

			case JTAG_SLEEP:
				DEBUG_JTAG_IO("sleep %i", cmd->cmd.sleep->us);
				jlink_tap_execute();
				jtag_sleep(cmd->cmd.sleep->us);
				break;

			default:
				LOG_ERROR("BUG: unknown JTAG command type encountered");
				exit(-1);
		}
		cmd = cmd->next;
	}

	return jlink_tap_execute();
}

/* Sets speed in kHz. */
int jlink_speed(int speed)
{
	int result;

	if (speed <= JLINK_MAX_SPEED)
	{
		/* check for RTCK setting */
		if (speed == 0)
			speed = -1;

		usb_out_buffer[0] = EMU_CMD_SET_SPEED;
		usb_out_buffer[1] = (speed >> 0) & 0xff;
		usb_out_buffer[2] = (speed >> 8) & 0xff;

		result = jlink_usb_write(jlink_jtag_handle, 3);

		if (result == 3)
		{
			return ERROR_OK;
		}
		else
		{
			LOG_ERROR("J-Link setting speed failed (%d)", result);
			return ERROR_JTAG_DEVICE_ERROR;
		}
	}
	else
	{
		LOG_INFO("Requested speed %dkHz exceeds maximum of %dkHz, ignored", speed, JLINK_MAX_SPEED);
	}

	return ERROR_OK;
}

int jlink_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz;

	return ERROR_OK;
}

int jlink_register_commands(struct command_context_s *cmd_ctx)
{
	register_command(cmd_ctx, NULL, "jlink_info", jlink_handle_jlink_info_command, COMMAND_EXEC,
		"query jlink info");
	return ERROR_OK;
}

int jlink_init(void)
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

int jlink_quit(void)
{
	jlink_usb_close(jlink_jtag_handle);
	return ERROR_OK;
}

/***************************************************************************/
/* Queue command implementations */

void jlink_end_state(tap_state_t state)
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
void jlink_state_move(void)
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

void jlink_path_move(int num_states, tap_state_t *path)
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

void jlink_runtest(int num_cycles)
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

void jlink_scan(int ir_scan, enum scan_type type, u8 *buffer, int scan_size, scan_command_t *command)
{
	tap_state_t saved_end_state;

	jlink_tap_ensure_space(1, scan_size + 8);

	saved_end_state = tap_get_end_state();

	/* Move to appropriate scan state */
	jlink_end_state(ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT);

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

void jlink_reset(int trst, int srst)
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

void jlink_simple_command(u8 command)
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

int jlink_get_status(void)
{
	int result;

	jlink_simple_command(EMU_CMD_GET_STATE);
	result = jlink_usb_read(jlink_jtag_handle);

	if (result == 8)
	{
		int vref = usb_in_buffer[0] + (usb_in_buffer[1] << 8);
		LOG_INFO("Vref = %d.%d TCK = %d TDI = %d TDO = %d TMS = %d SRST = %d TRST = %d\n", \
			vref / 1000, vref % 1000, \
			usb_in_buffer[2], usb_in_buffer[3], usb_in_buffer[4], \
			usb_in_buffer[5], usb_in_buffer[6], usb_in_buffer[7]);

		if (vref < 1500)
		{
			LOG_ERROR("Vref too low. Check Target Power\n");
		}
	}
	else
	{
		LOG_ERROR("J-Link command EMU_CMD_GET_STATE failed (%d)\n", result);
	}

	return ERROR_OK;
}

int jlink_get_version_info(void)
{
	int result;
	int len = 0;

	/* query hardware version */
	jlink_simple_command(EMU_CMD_VERSION);
	result = jlink_usb_read(jlink_jtag_handle);

	if (result == 2)
	{
		len = buf_get_u32(usb_in_buffer, 0, 16);
		result = jlink_usb_read(jlink_jtag_handle);

		if (result == len)
		{
			usb_in_buffer[result] = 0;
			LOG_INFO(usb_in_buffer);
			return ERROR_OK;
		}
	}

	LOG_ERROR("J-Link command EMU_CMD_VERSION failed (%d)\n", result);
	return ERROR_JTAG_DEVICE_ERROR;
}

int jlink_handle_jlink_info_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
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

static int tap_length;
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

void jlink_tap_init(void)
{
	tap_length = 0;
	pending_scan_results_length = 0;
}

void jlink_tap_ensure_space(int scans, int bits)
{
	int available_scans = MAX_PENDING_SCAN_RESULTS - pending_scan_results_length;
	int available_bits = JLINK_TAP_BUFFER_SIZE * 8 - tap_length;

	if (scans > available_scans || bits > available_bits)
	{
		jlink_tap_execute();
	}
}

void jlink_tap_append_step(int tms, int tdi)
{
	last_tms = tms;
	int index = tap_length / 8;

	if (index < JLINK_TAP_BUFFER_SIZE)
	{
		int bit_index = tap_length % 8;
		u8 bit = 1 << bit_index;

		if (tms)
		{
			tms_buffer[index] |= bit;
		}
		else
		{
			tms_buffer[index] &= ~bit;
		}

		if (tdi)
		{
			tdi_buffer[index] |= bit;
		}
		else
		{
			tdi_buffer[index] &= ~bit;
		}

		tap_length++;
	}
	else
	{
		LOG_ERROR("jlink_tap_append_step, overflow");
	}
}

void jlink_tap_append_scan(int length, u8 *buffer, scan_command_t *command)
{
	pending_scan_result_t *pending_scan_result = &pending_scan_results_buffer[pending_scan_results_length];
	int i;

	pending_scan_result->first = tap_length;
	pending_scan_result->length = length;
	pending_scan_result->command = command;
	pending_scan_result->buffer = buffer;

	for (i = 0; i < length; i++)
	{
		jlink_tap_append_step((i < length-1 ? 0 : 1), (buffer[i/8] >> (i%8)) & 1);
	}
	pending_scan_results_length++;
}

/* Pad and send a tap sequence to the device, and receive the answer.
 * For the purpose of padding we assume that we are in idle or pause state. */
int jlink_tap_execute(void)
{
	int byte_length;
	int tms_offset;
	int tdi_offset;
	int i;
	int result;

	if (tap_length > 0)
	{
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

		if (result == byte_length)
		{
			for (i = 0; i < byte_length; i++)
			{
				tdo_buffer[i] = usb_in_buffer[i];
			}

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
		}
		else
		{
			LOG_ERROR("jlink_tap_execute, wrong result %d, expected %d", result, byte_length);
			return ERROR_JTAG_QUEUE_FAILED;
		}

		jlink_tap_init();
	}

	return ERROR_OK;
}

/*****************************************************************************/
/* JLink USB low-level functions */

jlink_jtag_t* jlink_usb_open()
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

void jlink_usb_close(jlink_jtag_t *jlink_jtag)
{
	usb_close(jlink_jtag->usb_handle);
	free(jlink_jtag);
}

/* Send a message and receive the reply. */
int jlink_usb_message(jlink_jtag_t *jlink_jtag, int out_length, int in_length)
{
	int result;
	int result2;

	result = jlink_usb_write(jlink_jtag, out_length);
	if (result == out_length)
	{
		result = jlink_usb_read(jlink_jtag);
		if (result == in_length || result == in_length+1)
		{
			if (result == in_length)
			{
				/* Must read the result from the EMU too */
				result2 = jlink_usb_read_emu_result(jlink_jtag);
				if (1 == result2)
				{
					/* Check the result itself */
					if (0 == usb_emu_result_buffer[0])
					{
						return result;
					}
					else
					{
						LOG_ERROR("jlink_usb_read_emu_result (requested=0, result=%d)", usb_emu_result_buffer[0]);
						return -1;
					}
				}
				else
				{
					LOG_ERROR("jlink_usb_read_emu_result len (requested=1, result=%d)", result2);
					return -1;
				}
			}
			else
			{
				/* Check the result itself */
				if (0 == usb_in_buffer[result-1])
				{
					return result-1;
				}
				else
				{
					LOG_ERROR("jlink_usb_read_emu_result (requested=0, result=%d)", usb_in_buffer[result]);
					return -1;
				}
			}
		}
		else
		{
			LOG_ERROR("usb_bulk_read failed (requested=%d, result=%d)", in_length, result);
			return -1;
		}
	}
	else
	{
		LOG_ERROR("usb_bulk_write failed (requested=%d, result=%d)", out_length, result);
		return -1;
	}
}

/* Write data from out_buffer to USB. */
int jlink_usb_write(jlink_jtag_t *jlink_jtag, int out_length)
{
	int result;

	if (out_length > JLINK_OUT_BUFFER_SIZE)
	{
		LOG_ERROR("jlink_jtag_write illegal out_length=%d (max=%d)", out_length, JLINK_OUT_BUFFER_SIZE);
		return -1;
	}

	result = usb_bulk_write(jlink_jtag->usb_handle, JLINK_WRITE_ENDPOINT, \
		usb_out_buffer, out_length, JLINK_USB_TIMEOUT);

	DEBUG_JTAG_IO("jlink_usb_write, out_length = %d, result = %d", out_length, result);

#ifdef _DEBUG_USB_COMMS_
	jlink_debug_buffer(usb_out_buffer, out_length);
#endif
	return result;
}

/* Read data from USB into in_buffer. */
int jlink_usb_read(jlink_jtag_t *jlink_jtag)
{
	int result = usb_bulk_read(jlink_jtag->usb_handle, JLINK_READ_ENDPOINT, \
		usb_in_buffer, JLINK_IN_BUFFER_SIZE, JLINK_USB_TIMEOUT);

	DEBUG_JTAG_IO("jlink_usb_read, result = %d", result);

#ifdef _DEBUG_USB_COMMS_
	jlink_debug_buffer(usb_in_buffer, result);
#endif
	return result;
}

/* Read the result from the previous EMU cmd into result_buffer. */
int jlink_usb_read_emu_result(jlink_jtag_t *jlink_jtag)
{
	int result = usb_bulk_read(jlink_jtag->usb_handle, JLINK_READ_ENDPOINT, \
		usb_emu_result_buffer, JLINK_EMU_RESULT_BUFFER_SIZE, JLINK_USB_TIMEOUT);

	DEBUG_JTAG_IO("jlink_usb_read_result, result = %d", result);

#ifdef _DEBUG_USB_COMMS_
	jlink_debug_buffer(usb_emu_result_buffer, result);
#endif
	return result;
}

#ifdef _DEBUG_USB_COMMS_
#define BYTES_PER_LINE  16

void jlink_debug_buffer(u8 *buffer, int length)
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
		LOG_DEBUG(line);
	}
}
#endif
