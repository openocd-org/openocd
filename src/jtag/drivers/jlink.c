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

#include <jtag/interface.h>
#include <jtag/commands.h>
#include "usb_common.h"

/* See Segger's public documentation:
 *	Reference manual for J-Link USB Protocol
 *	Document RM08001-R6 Date: June 16, 2009
 */

#define VID 0x1366
#define PID 0x0101

#define JLINK_WRITE_ENDPOINT	0x02
#define JLINK_READ_ENDPOINT		0x81

static unsigned int jlink_write_ep = JLINK_WRITE_ENDPOINT;
static unsigned int jlink_read_ep = JLINK_READ_ENDPOINT;
static unsigned int jlink_hw_jtag_version = 2;

#define JLINK_USB_TIMEOUT		1000

// See Section 1.3.2 of the Segger JLink USB protocol manual
/* 2048 is the max value we can use here */
//#define JLINK_TAP_BUFFER_SIZE 2048
#define JLINK_TAP_BUFFER_SIZE 256
//#define JLINK_TAP_BUFFER_SIZE 384

#define JLINK_IN_BUFFER_SIZE			2048
#define JLINK_OUT_BUFFER_SIZE			2*2048 + 4
#define JLINK_EMU_RESULT_BUFFER_SIZE	64

/* Global USB buffers */
static uint8_t usb_in_buffer[JLINK_IN_BUFFER_SIZE];
static uint8_t usb_out_buffer[JLINK_OUT_BUFFER_SIZE];
static uint8_t usb_emu_result_buffer[JLINK_EMU_RESULT_BUFFER_SIZE];

/* Constants for JLink command */
#define EMU_CMD_VERSION		0x01
#define EMU_CMD_SET_SPEED		0x05
#define EMU_CMD_GET_STATE		0x07
#define EMU_CMD_HW_CLOCK			0xc8
#define EMU_CMD_HW_TMS0			0xc9
#define EMU_CMD_HW_TMS1			0xca
#define EMU_CMD_HW_JTAG2		0xce
#define EMU_CMD_HW_JTAG3		0xcf
#define EMU_CMD_GET_MAX_MEM_BLOCK	0xd4
#define EMU_CMD_HW_RESET0		0xdc
#define EMU_CMD_HW_RESET1		0xdd
#define EMU_CMD_HW_TRST0		0xde
#define EMU_CMD_HW_TRST1		0xdf
#define EMU_CMD_GET_CAPS		0xe8
#define EMU_CMD_GET_HW_VERSION	0xf0

/* bits return from EMU_CMD_GET_CAPS */
#define EMU_CAP_GET_HW_VERSION		1
#define EMU_CAP_GET_MAX_BLOCK_SIZE	11

/* max speed 12MHz v5.0 jlink */
#define JLINK_MAX_SPEED 12000

/* Queue command functions */
static void jlink_end_state(tap_state_t state);
static void jlink_state_move(void);
static void jlink_path_move(int num_states, tap_state_t *path);
static void jlink_runtest(int num_cycles);
static void jlink_scan(bool ir_scan, enum scan_type type, uint8_t *buffer, int scan_size, struct scan_command *command);
static void jlink_reset(int trst, int srst);
static void jlink_simple_command(uint8_t command);
static int jlink_get_status(void);

/* J-Link tap buffer functions */
static void jlink_tap_init(void);
static int jlink_tap_execute(void);
static void jlink_tap_ensure_space(int scans, int bits);
static void jlink_tap_append_step(int tms, int tdi);
static void jlink_tap_append_scan(int length, uint8_t *buffer, struct scan_command *command);

/* Jlink lowlevel functions */
struct jlink {
	struct usb_dev_handle* usb_handle;
};

static struct jlink *jlink_usb_open(void);
static void jlink_usb_close(struct jlink *jlink);
static int jlink_usb_message(struct jlink *jlink, int out_length, int in_length);
static int jlink_usb_write(struct jlink *jlink, int out_length);
static int jlink_usb_read(struct jlink *jlink, int expected_size);
static int jlink_usb_read_emu_result(struct jlink *jlink);

/* helper functions */
static int jlink_get_version_info(void);

#ifdef _DEBUG_USB_COMMS_
static void jlink_debug_buffer(uint8_t *buffer, int length);
#endif

static enum tap_state jlink_last_state = TAP_RESET;

static struct jlink* jlink_handle;

/***************************************************************************/
/* External interface implementation */

static void jlink_execute_runtest(struct jtag_command *cmd)
{
	DEBUG_JTAG_IO("runtest %i cycles, end in %i",
			cmd->cmd.runtest->num_cycles,
			cmd->cmd.runtest->end_state);

	jlink_end_state(cmd->cmd.runtest->end_state);

	jlink_runtest(cmd->cmd.runtest->num_cycles);
}

static void jlink_execute_statemove(struct jtag_command *cmd)
{
	DEBUG_JTAG_IO("statemove end in %i", cmd->cmd.statemove->end_state);

	jlink_end_state(cmd->cmd.statemove->end_state);
	jlink_state_move();
}

static void jlink_execute_pathmove(struct jtag_command *cmd)
{
	DEBUG_JTAG_IO("pathmove: %i states, end in %i",
		cmd->cmd.pathmove->num_states,
		cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);

	jlink_path_move(cmd->cmd.pathmove->num_states,
			cmd->cmd.pathmove->path);
}

static void jlink_execute_scan(struct jtag_command *cmd)
{
	int scan_size;
	enum scan_type type;
	uint8_t *buffer;

	DEBUG_JTAG_IO("scan end in %s", tap_state_name(cmd->cmd.scan->end_state));

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

static void jlink_execute_reset(struct jtag_command *cmd)
{
	DEBUG_JTAG_IO("reset trst: %i srst %i",
			cmd->cmd.reset->trst, cmd->cmd.reset->srst);

	jlink_tap_execute();
	jlink_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
	jlink_tap_execute();
}

static void jlink_execute_sleep(struct jtag_command *cmd)
{
	DEBUG_JTAG_IO("sleep %" PRIi32 "", cmd->cmd.sleep->us);
	jlink_tap_execute();
	jtag_sleep(cmd->cmd.sleep->us);
}

static void jlink_execute_command(struct jtag_command *cmd)
{
	switch (cmd->type)
	{
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
	struct jtag_command *cmd = jtag_command_queue;

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

	result = jlink_usb_write(jlink_handle, 3);
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

static int jlink_init(void)
{
	int i;

	jlink_handle = jlink_usb_open();

	if (jlink_handle == 0)
	{
		LOG_ERROR("Cannot find jlink Interface! Please check connection and permissions.");
		return ERROR_JTAG_INIT_FAILED;
	}

	/*
	 *  The next three instructions were added after discovering a problem while using an oscilloscope.  For the V8
	 *	SAM-ICE dongle (and likely other j-link device variants), the reset line to the target microprocessor was found to
	 *	cycle only intermittently during emulator startup (even after encountering the downstream reset instruction later
	 *	in the code).  This was found to create two issues:  1) In general it is a bad practice to not reset a CPU to a known
	 *	state when starting an emulator and 2) something critical happens inside the dongle when it does the first read
	 *	following a new USB session.  Keeping the processor in reset during the first read collecting version information
	 *	seems to prevent errant "J-Link command EMU_CMD_VERSION failed" issues.
	 */

	LOG_INFO("J-Link initialization started / target CPU reset initiated");
	jlink_simple_command(EMU_CMD_HW_TRST0);
	jlink_simple_command(EMU_CMD_HW_RESET0);
	usleep(1000);

	jlink_hw_jtag_version = 2;

	if (jlink_get_version_info() == ERROR_OK)
	{
		/* attempt to get status */
		jlink_get_status();
	}

	LOG_INFO("J-Link JTAG Interface ready");

	jlink_reset(0, 0);
	jtag_sleep(3000);
	jlink_tap_init();
	jlink_speed(jtag_get_speed());

	/* v5/6 jlink seems to have an issue if the first tap move
	 * is not divisible by 8, so we send a TLR on first power up */
	for (i = 0; i < 8; i++) {
		jlink_tap_append_step(1, 0);
	}
	jlink_tap_execute();

	return ERROR_OK;
}

static int jlink_quit(void)
{
	jlink_usb_close(jlink_handle);
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
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());
	uint8_t tms_scan_bits = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());

	for (i = 0; i < tms_scan_bits; i++)
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

	jlink_tap_ensure_space(1,num_cycles + 16);

	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE)
	{
		jlink_end_state(TAP_IDLE);
		jlink_state_move();
//		num_cycles--;
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

static void jlink_scan(bool ir_scan, enum scan_type type, uint8_t *buffer, int scan_size, struct scan_command *command)
{
	tap_state_t saved_end_state;

	jlink_tap_ensure_space(1, scan_size + 16);

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
	if (srst == 1)
	{
		jlink_simple_command(EMU_CMD_HW_RESET0);
	}

	if (trst == 1)
	{
		jlink_simple_command(EMU_CMD_HW_TRST0);
	}

	if (trst == 0)
	{
		jlink_simple_command(EMU_CMD_HW_TRST1);
	}
}

static void jlink_simple_command(uint8_t command)
{
	int result;

	DEBUG_JTAG_IO("0x%02x", command);

	usb_out_buffer[0] = command;
	result = jlink_usb_write(jlink_handle, 1);

	if (result != 1)
	{
		LOG_ERROR("J-Link command 0x%02x failed (%d)", command, result);
	}
}

static int jlink_get_status(void)
{
	int result;

	jlink_simple_command(EMU_CMD_GET_STATE);

	result = jlink_usb_read(jlink_handle, 8);
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
	uint32_t jlink_caps, jlink_max_size;

	/* query hardware version */
	jlink_simple_command(EMU_CMD_VERSION);

	result = jlink_usb_read(jlink_handle, 2);
	if (2 != result)
	{
		LOG_ERROR("J-Link command EMU_CMD_VERSION failed (%d)\n", result);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	len = buf_get_u32(usb_in_buffer, 0, 16);
	if (len > JLINK_IN_BUFFER_SIZE)
	{
		LOG_ERROR("J-Link command EMU_CMD_VERSION impossible return length 0x%0x", len);
		len = JLINK_IN_BUFFER_SIZE;
	}

	result = jlink_usb_read(jlink_handle, len);
	if (result != len)
	{
		LOG_ERROR("J-Link command EMU_CMD_VERSION failed (%d)\n", result);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	usb_in_buffer[result] = 0;
	LOG_INFO("%s", (char *)usb_in_buffer);

	/* query hardware capabilities */
	jlink_simple_command(EMU_CMD_GET_CAPS);

	result = jlink_usb_read(jlink_handle, 4);
	if (4 != result)
	{
		LOG_ERROR("J-Link command EMU_CMD_GET_CAPS failed (%d)\n", result);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	jlink_caps = buf_get_u32(usb_in_buffer, 0, 32);
	LOG_INFO("JLink caps 0x%x", (unsigned)jlink_caps);

	if (jlink_caps & (1 << EMU_CAP_GET_HW_VERSION))
	{
		/* query hardware version */
		jlink_simple_command(EMU_CMD_GET_HW_VERSION);

		result = jlink_usb_read(jlink_handle, 4);
		if (4 != result)
		{
			LOG_ERROR("J-Link command EMU_CMD_GET_HW_VERSION failed (%d)\n", result);
			return ERROR_JTAG_DEVICE_ERROR;
		}

		uint32_t jlink_hw_version = buf_get_u32(usb_in_buffer, 0, 32);
		uint32_t major_revision = (jlink_hw_version / 10000) % 100;
		if (major_revision >= 5)
			jlink_hw_jtag_version = 3;

		LOG_INFO("JLink hw version %i", (int)jlink_hw_version);
	}

	if (jlink_caps & (1 << EMU_CAP_GET_MAX_BLOCK_SIZE))
	{
		/* query hardware maximum memory block */
		jlink_simple_command(EMU_CMD_GET_MAX_MEM_BLOCK);

		result = jlink_usb_read(jlink_handle, 4);
		if (4 != result)
		{
			LOG_ERROR("J-Link command EMU_CMD_GET_MAX_MEM_BLOCK failed (%d)\n", result);
			return ERROR_JTAG_DEVICE_ERROR;
		}

		jlink_max_size = buf_get_u32(usb_in_buffer, 0, 32);
		LOG_INFO("JLink max mem block %i", (int)jlink_max_size);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(jlink_handle_jlink_info_command)
{
	if (jlink_get_version_info() == ERROR_OK)
	{
		/* attempt to get status */
		jlink_get_status();
	}

	return ERROR_OK;
}

COMMAND_HANDLER(jlink_handle_jlink_hw_jtag_command)
{
	switch (CMD_ARGC) {
	case 0:
		command_print(CMD_CTX, "jlink hw jtag  %i", jlink_hw_jtag_version);
		break;
	case 1: {
		int request_version = atoi(CMD_ARGV[0]);
		switch (request_version) {
		case 2: case 3:
			jlink_hw_jtag_version = request_version;
			break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		break;
	}
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}

static const struct command_registration jlink_command_handlers[] = {
	{
		.name = "jlink_info",
		.handler = &jlink_handle_jlink_info_command,
		.mode = COMMAND_EXEC,
		.help = "show jlink info",
	},
	{
		.name = "jlink_hw_jtag",
		.handler = &jlink_handle_jlink_hw_jtag_command,
		.mode = COMMAND_EXEC,
		.help = "access J-Link HW JTAG command version",
		.usage = "[2|3]",
	},
	COMMAND_REGISTRATION_DONE
};

struct jtag_interface jlink_interface = {
	.name = "jlink",
	.commands = jlink_command_handlers,

	.execute_queue = jlink_execute_queue,
	.speed = jlink_speed,
	.speed_div = jlink_speed_div,
	.khz = jlink_khz,
	.init = jlink_init,
	.quit = jlink_quit,
};

/***************************************************************************/
/* J-Link tap functions */


static unsigned tap_length = 0;
static uint8_t tms_buffer[JLINK_TAP_BUFFER_SIZE];
static uint8_t tdi_buffer[JLINK_TAP_BUFFER_SIZE];
static uint8_t tdo_buffer[JLINK_TAP_BUFFER_SIZE];

struct pending_scan_result {
	int first;	/* First bit position in tdo_buffer to read */
	int length; /* Number of bits to read */
	struct scan_command *command; /* Corresponding scan command */
	uint8_t *buffer;
};

#define MAX_PENDING_SCAN_RESULTS 256

static int pending_scan_results_length;
static struct pending_scan_result pending_scan_results_buffer[MAX_PENDING_SCAN_RESULTS];

static void jlink_tap_init(void)
{
	tap_length = 0;
	pending_scan_results_length = 0;
}

static void jlink_tap_ensure_space(int scans, int bits)
{
	int available_scans = MAX_PENDING_SCAN_RESULTS - pending_scan_results_length;
	int available_bits = JLINK_TAP_BUFFER_SIZE * 8 - tap_length - 32;

	if (scans > available_scans || bits > available_bits)
	{
		jlink_tap_execute();
	}
}

static void jlink_tap_append_step(int tms, int tdi)
{
	int index = tap_length / 8;

	if (index >= JLINK_TAP_BUFFER_SIZE)
	{
		LOG_ERROR("jlink_tap_append_step: overflow");
		*(uint32_t *)0xFFFFFFFF = 0;
		exit(-1);
	}

	int bit_index = tap_length % 8;
	uint8_t bit = 1 << bit_index;

	// we do not pad TMS, so be sure to initialize all bits
	if (0 == bit_index)
	{
		tms_buffer[index] = tdi_buffer[index] = 0;
	}

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

static void jlink_tap_append_scan(int length, uint8_t *buffer, struct scan_command *command)
{
	struct pending_scan_result *pending_scan_result =
		&pending_scan_results_buffer[pending_scan_results_length];
	int i;

	pending_scan_result->first = tap_length;
	pending_scan_result->length = length;
	pending_scan_result->command = command;
	pending_scan_result->buffer = buffer;

	for (i = 0; i < length; i++)
	{
		int tms = (i < (length - 1)) ? 0 : 1;
		int tdi = (buffer[i / 8] & (1 << (i % 8))) != 0;
		jlink_tap_append_step(tms, tdi);
	}
	pending_scan_results_length++;
}

/* Pad and send a tap sequence to the device, and receive the answer.
 * For the purpose of padding we assume that we are in idle or pause state. */
static int jlink_tap_execute(void)
{
	int byte_length;
	int i;
	int result;

	if (!tap_length)
		return ERROR_OK;

	/* JLink returns an extra NULL in packet when size of incoming
	 * message is a multiple of 64, creates problems with USB comms.
	 * WARNING: This will interfere with tap state counting. */
	while ((DIV_ROUND_UP(tap_length, 8) % 64) == 0)
	{
		jlink_tap_append_step((tap_get_state() == TAP_RESET)?1:0, 0);
	}

	// number of full bytes (plus one if some would be left over)
	byte_length = DIV_ROUND_UP(tap_length, 8);

	bool use_jtag3 = jlink_hw_jtag_version >= 3;
	usb_out_buffer[0] = use_jtag3 ? EMU_CMD_HW_JTAG3 : EMU_CMD_HW_JTAG2;
	usb_out_buffer[1] = 0;
	usb_out_buffer[2] = (tap_length >> 0) & 0xff;
	usb_out_buffer[3] = (tap_length >> 8) & 0xff;
	memcpy(usb_out_buffer + 4, tms_buffer, byte_length);
	memcpy(usb_out_buffer + 4 + byte_length, tdi_buffer, byte_length);

	jlink_last_state = jtag_debug_state_machine(tms_buffer, tdi_buffer,
			tap_length, jlink_last_state);

	result = jlink_usb_message(jlink_handle, 4 + 2 * byte_length, byte_length);
	if (result != byte_length)
	{
		LOG_ERROR("jlink_tap_execute, wrong result %d (expected %d)", result, byte_length);
		jlink_tap_init();
		return ERROR_JTAG_QUEUE_FAILED;
	}

	memcpy(tdo_buffer, usb_in_buffer, byte_length);

	for (i = 0; i < pending_scan_results_length; i++)
	{
		struct pending_scan_result *pending_scan_result = &pending_scan_results_buffer[i];
		uint8_t *buffer = pending_scan_result->buffer;
		int length = pending_scan_result->length;
		int first = pending_scan_result->first;
		struct scan_command *command = pending_scan_result->command;

		/* Copy to buffer */
		buf_set_buf(tdo_buffer, first, buffer, 0, length);

		DEBUG_JTAG_IO("pending scan result, length = %d", length);

#ifdef _DEBUG_USB_COMMS_
		jlink_debug_buffer(buffer, DIV_ROUND_UP(length, 8));
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

static struct jlink* jlink_usb_open()
{
	usb_init();

	const uint16_t vids[] = { VID, 0 };
	const uint16_t pids[] = { PID, 0 };
	struct usb_dev_handle *dev;
	if (jtag_usb_open(vids, pids, &dev) != ERROR_OK)
		return NULL;

	/* BE ***VERY CAREFUL*** ABOUT MAKING CHANGES IN THIS
	 * AREA!!!!!!!!!!!  The behavior of libusb is not completely
	 * consistent across Windows, Linux, and Mac OS X platforms.
	 * The actions taken in the following compiler conditionals may
	 * not agree with published documentation for libusb, but were
	 * found to be necessary through trials and tribulations.  Even
	 * little tweaks can break one or more platforms, so if you do
	 * make changes test them carefully on all platforms before
	 * committing them!
	 */

#if IS_WIN32 == 0

	usb_reset(dev);

#if IS_DARWIN == 0

	int timeout = 5;
	/* reopen jlink after usb_reset
	 * on win32 this may take a second or two to re-enumerate */
	int retval;
	while ((retval = jtag_usb_open(vids, pids, &dev)) != ERROR_OK)
	{
		usleep(1000);
		timeout--;
		if (!timeout) {
			break;
		}
	}
	if (ERROR_OK != retval)
		return NULL;
#endif

#endif

	/* usb_set_configuration required under win32 */
	struct usb_device *udev = usb_device(dev);
	usb_set_configuration(dev, udev->config[0].bConfigurationValue);
	usb_claim_interface(dev, 0);

#if 0
	/*
	 * This makes problems under Mac OS X. And is not needed
	 * under Windows. Hopefully this will not break a linux build
	 */
	usb_set_altinterface(result->usb_handle, 0);
#endif
	struct usb_interface *iface = udev->config->interface;
	struct usb_interface_descriptor *desc = iface->altsetting;
	for (int i = 0; i < desc->bNumEndpoints; i++)
	{
		uint8_t epnum = desc->endpoint[i].bEndpointAddress;
		bool is_input = epnum & 0x80;
		LOG_DEBUG("usb ep %s %02x", is_input ? "in" : "out", epnum);
		if (is_input)
			jlink_read_ep = epnum;
		else
			jlink_write_ep = epnum;
	}

	struct jlink *result = malloc(sizeof(struct jlink));
	result->usb_handle = dev;
	return result;
}

static void jlink_usb_close(struct jlink *jlink)
{
	usb_close(jlink->usb_handle);
	free(jlink);
}

/* Send a message and receive the reply. */
static int jlink_usb_message(struct jlink *jlink, int out_length, int in_length)
{
	int result;

	result = jlink_usb_write(jlink, out_length);
	if (result != out_length)
	{
		LOG_ERROR("usb_bulk_write failed (requested=%d, result=%d)",
				out_length, result);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	result = jlink_usb_read(jlink, in_length);
	if ((result != in_length) && (result != (in_length + 1)))
	{
		LOG_ERROR("usb_bulk_read failed (requested=%d, result=%d)",
				in_length, result);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	if (jlink_hw_jtag_version < 3)
		return result;

	int result2 = ERROR_OK;
	if (result == in_length)
	{
		/* Must read the result from the EMU too */
		result2 = jlink_usb_read_emu_result(jlink);
		if (1 != result2)
		{
			LOG_ERROR("jlink_usb_read_emu_result retried requested = 1, result=%d, in_length=%i", result2,in_length);
			/* Try again once, should only happen if (in_length%64 == 0) */
			result2 = jlink_usb_read_emu_result(jlink);
			if (1 != result2)
			{
				LOG_ERROR("jlink_usb_read_emu_result failed "
					"(requested = 1, result=%d)", result2);
				return ERROR_JTAG_DEVICE_ERROR;
			}
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
	int tries = 3, count = 0;

	while (tries && (count < size))
	{
		int result = f(dev, ep, bytes + count, size - count, timeout);
		if (result > 0)
			count += result;
		else if ((-ETIMEDOUT != result) || !--tries)
			return result;
	}
	return count;
}

static int wrap_usb_bulk_write(usb_dev_handle *dev, int ep,
			       char *buff, int size, int timeout)
{
	/* usb_bulk_write() takes const char *buff */
	return usb_bulk_write(dev, ep, buff, size, timeout);
}

static inline int usb_bulk_write_ex(usb_dev_handle *dev, int ep,
		char *bytes, int size, int timeout)
{
	return usb_bulk_with_retries(&wrap_usb_bulk_write,
			dev, ep, bytes, size, timeout);
}

static inline int usb_bulk_read_ex(usb_dev_handle *dev, int ep,
		char *bytes, int size, int timeout)
{
	return usb_bulk_with_retries(&usb_bulk_read,
			dev, ep, bytes, size, timeout);
}

/* Write data from out_buffer to USB. */
static int jlink_usb_write(struct jlink *jlink, int out_length)
{
	int result;

	if (out_length > JLINK_OUT_BUFFER_SIZE)
	{
		LOG_ERROR("jlink_write illegal out_length=%d (max=%d)", out_length, JLINK_OUT_BUFFER_SIZE);
		return -1;
	}

	result = usb_bulk_write_ex(jlink->usb_handle, jlink_write_ep,
		(char *)usb_out_buffer, out_length, JLINK_USB_TIMEOUT);

	DEBUG_JTAG_IO("jlink_usb_write, out_length = %d, result = %d", out_length, result);

#ifdef _DEBUG_USB_COMMS_
	jlink_debug_buffer(usb_out_buffer, out_length);
#endif
	return result;
}

/* Read data from USB into in_buffer. */
static int jlink_usb_read(struct jlink *jlink, int expected_size)
{
	int result = usb_bulk_read_ex(jlink->usb_handle, jlink_read_ep,
		(char *)usb_in_buffer, expected_size, JLINK_USB_TIMEOUT);

	DEBUG_JTAG_IO("jlink_usb_read, result = %d", result);

#ifdef _DEBUG_USB_COMMS_
	jlink_debug_buffer(usb_in_buffer, result);
#endif
	return result;
}

/* Read the result from the previous EMU cmd into result_buffer. */
static int jlink_usb_read_emu_result(struct jlink *jlink)
{
	int result = usb_bulk_read_ex(jlink->usb_handle, jlink_read_ep,
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

static void jlink_debug_buffer(uint8_t *buffer, int length)
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

