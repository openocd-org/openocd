// vim:ts=4 sw=4: 

/***************************************************************************
 *   Copyright (C) 2009 by Dimitar Dimitrov <dinuxbg@gmail.com>            *
 *   based on Dominic Rath's and Benedikt Sauter's usbprog.c               *
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
#include <ctype.h>

/* system includes */

#include "log.h"

/* enable this to debug communication
 */
#if 1
#define _DEBUG_USB_COMMS_
#define _DEBUG_JTAG_IO_
#endif

#ifdef _DEBUG_JTAG_IO_
#define DEBUG_JTAG_IO(expr ...)	LOG_DEBUG(expr)
#else
#define DEBUG_JTAG_IO(expr ...)
#endif

#define USB_VID						0x15ba
#define USB_PID						0x001e

#define ARMJTAGEW_EPT_BULK_OUT		0x01u
#define ARMJTAGEW_EPT_BULK_IN		0x82u

#define ARMJTAGEW_USB_TIMEOUT		2000

#define ARMJTAGEW_IN_BUFFER_SIZE	(4*1024)
#define ARMJTAGEW_OUT_BUFFER_SIZE	(4*1024)


/* USB command request codes. */
#define CMD_GET_VERSION				0x00
#define CMD_SELECT_DPIMPL			0x10
#define CMD_SET_TCK_FREQUENCY		0x11
#define CMD_GET_TCK_FREQUENCY		0x12
#define CMD_MEASURE_MAX_TCK_FREQ	0x15
#define CMD_MEASURE_RTCK_RESPONSE	0x16
#define CMD_TAP_SHIFT				0x17
#define CMD_SET_TAPHW_STATE			0x20
#define CMD_GET_TAPHW_STATE			0x21
#define CMD_TGPWR_SETUP				0x22

/* Global USB buffers */
static u8 usb_in_buffer[ARMJTAGEW_IN_BUFFER_SIZE];
static u8 usb_out_buffer[ARMJTAGEW_OUT_BUFFER_SIZE];

/* External interface functions */
int armjtagew_execute_queue(void);
int armjtagew_speed(int speed);
int armjtagew_khz(int khz, int *jtag_speed);
int armjtagew_register_commands(struct command_context_s *cmd_ctx);
int armjtagew_init(void);
int armjtagew_quit(void);

/* CLI command handler functions */
int armjtagew_handle_armjtagew_info_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

/* Queue command functions */
void armjtagew_end_state(tap_state_t state);
void armjtagew_state_move(void);
void armjtagew_path_move(int num_states, tap_state_t *path);
void armjtagew_runtest(int num_cycles);
void armjtagew_scan(int ir_scan, enum scan_type type, u8 *buffer, int scan_size, scan_command_t *command);
void armjtagew_reset(int trst, int srst);
void armjtagew_simple_command(u8 command);
int armjtagew_get_status(void);

/* tap buffer functions */
void armjtagew_tap_init(void);
int armjtagew_tap_execute(void);
void armjtagew_tap_ensure_space(int scans, int bits);
void armjtagew_tap_append_step(int tms, int tdi);
void armjtagew_tap_append_scan(int length, u8 *buffer, scan_command_t *command);

/* ARM-JTAG-EW lowlevel functions */
typedef struct armjtagew_jtag
{
	struct usb_dev_handle* usb_handle;
} armjtagew_jtag_t;

armjtagew_jtag_t *armjtagew_usb_open(void);
void armjtagew_usb_close(armjtagew_jtag_t *armjtagew_jtag);
int armjtagew_usb_message(armjtagew_jtag_t *armjtagew_jtag, int out_length, int in_length);
int armjtagew_usb_write(armjtagew_jtag_t *armjtagew_jtag, int out_length);
int armjtagew_usb_read(armjtagew_jtag_t *armjtagew_jtag, int exp_in_length);

/* helper functions */
int armjtagew_get_version_info(void);

#ifdef _DEBUG_USB_COMMS_
void armjtagew_debug_buffer(u8 *buffer, int length);
#endif

armjtagew_jtag_t* armjtagew_jtag_handle;



/***************************************************************************/
/* External interface implementation */

jtag_interface_t armjtagew_interface =
{
	.name = "arm-jtag-ew",
	.execute_queue = armjtagew_execute_queue,
	.speed = armjtagew_speed,
	.khz = armjtagew_khz,
	.register_commands = armjtagew_register_commands,
	.init = armjtagew_init,
	.quit = armjtagew_quit
};


int armjtagew_execute_queue(void)
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
					armjtagew_end_state(cmd->cmd.end_state->end_state);
				}
				break;

			case JTAG_RUNTEST:
				DEBUG_JTAG_IO( "runtest %i cycles, end in %i", cmd->cmd.runtest->num_cycles, \
					cmd->cmd.runtest->end_state);

				if (cmd->cmd.runtest->end_state != -1)
				{
					armjtagew_end_state(cmd->cmd.runtest->end_state);
				}
				armjtagew_runtest(cmd->cmd.runtest->num_cycles);
				break;

			case JTAG_STATEMOVE:
				DEBUG_JTAG_IO("statemove end in %i", cmd->cmd.statemove->end_state);

				if (cmd->cmd.statemove->end_state != -1)
				{
					armjtagew_end_state(cmd->cmd.statemove->end_state);
				}
				armjtagew_state_move();
				break;

			case JTAG_PATHMOVE:
				DEBUG_JTAG_IO("pathmove: %i states, end in %i", \
					cmd->cmd.pathmove->num_states, \
					cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);

				armjtagew_path_move(cmd->cmd.pathmove->num_states, cmd->cmd.pathmove->path);
				break;

			case JTAG_SCAN:
				DEBUG_JTAG_IO("scan end in %i", cmd->cmd.scan->end_state);

				if (cmd->cmd.scan->end_state != -1)
				{
					armjtagew_end_state(cmd->cmd.scan->end_state);
				}

				scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
				DEBUG_JTAG_IO("scan input, length = %d", scan_size);

#ifdef _DEBUG_USB_COMMS_
				armjtagew_debug_buffer(buffer, (scan_size + 7) / 8);
#endif
				type = jtag_scan_type(cmd->cmd.scan);
				armjtagew_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size, cmd->cmd.scan);
				break;

			case JTAG_RESET:
				DEBUG_JTAG_IO("reset trst: %i srst %i", cmd->cmd.reset->trst, cmd->cmd.reset->srst);

				armjtagew_tap_execute();

				if (cmd->cmd.reset->trst == 1)
				{
					tap_set_state(TAP_RESET);
				}
				armjtagew_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
				break;

			case JTAG_SLEEP:
				DEBUG_JTAG_IO("sleep %i", cmd->cmd.sleep->us);
				armjtagew_tap_execute();
				jtag_sleep(cmd->cmd.sleep->us);
				break;

			default:
				LOG_ERROR("BUG: unknown JTAG command type encountered");
				exit(-1);
		}
		cmd = cmd->next;
	}

	return armjtagew_tap_execute();
}


/* Sets speed in kHz. */
int armjtagew_speed(int speed)
{
    int result;
    int speed_real;


    usb_out_buffer[0] = CMD_SET_TCK_FREQUENCY;
	buf_set_u32(usb_out_buffer+1, 0, 32, speed);

    result = armjtagew_usb_message(armjtagew_jtag_handle, 4, 4);

    if (result < 0)
    {
        LOG_ERROR("ARM-JTAG-EW setting speed failed (%d)", result);
        return ERROR_JTAG_DEVICE_ERROR;
    }

	usb_out_buffer[0] = CMD_GET_TCK_FREQUENCY;
    result = armjtagew_usb_message(armjtagew_jtag_handle, 1, 4);
	speed_real = (int)buf_get_u32(usb_in_buffer,0,32);
	if(result < 0) 
	{
        LOG_ERROR("ARM-JTAG-EW getting speed failed (%d)", result);
        return ERROR_JTAG_DEVICE_ERROR;
	} 
	else
	{
    	LOG_INFO("Requested speed %dkHz, emulator reported %dkHz.", speed, speed_real);
	}

    return ERROR_OK;
}


int armjtagew_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz;

	return ERROR_OK;
}

int armjtagew_register_commands(struct command_context_s *cmd_ctx)
{
	register_command(cmd_ctx, NULL, "armjtagew_info", armjtagew_handle_armjtagew_info_command, COMMAND_EXEC,
		"query armjtagew info");
	return ERROR_OK;
}

int armjtagew_init(void)
{
	int check_cnt;

	armjtagew_jtag_handle = armjtagew_usb_open();

	if (armjtagew_jtag_handle == 0)
	{
		LOG_ERROR("Cannot find ARM-JTAG-EW Interface! Please check connection and permissions.");
		return ERROR_JTAG_INIT_FAILED;
	}

	check_cnt = 0;
	while (check_cnt < 3)
	{
		if (armjtagew_get_version_info() == ERROR_OK)
		{
			/* attempt to get status */
			armjtagew_get_status();
			break;
		}

		check_cnt++;
	}

	if (check_cnt == 3)
	{
		LOG_INFO("ARM-JTAG-EW initial read failed, don't worry");
	}

	LOG_INFO("ARM-JTAG-EW JTAG Interface ready");

	armjtagew_reset(0, 0);
	armjtagew_tap_init();

	return ERROR_OK;
}

int armjtagew_quit(void)
{
	armjtagew_usb_close(armjtagew_jtag_handle);
	return ERROR_OK;
}

/***************************************************************************/
/* Queue command implementations */

void armjtagew_end_state(tap_state_t state)
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
void armjtagew_state_move(void)
{
	int i;
	int tms = 0;
	u8 tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());

	for (i = 0; i < 7; i++)
	{
		tms = (tms_scan >> i) & 1;
		armjtagew_tap_append_step(tms, 0);
	}

	tap_set_state(tap_get_end_state());
}

void armjtagew_path_move(int num_states, tap_state_t *path)
{
	int i;

	for (i = 0; i < num_states; i++)
	{
		/*
		 * TODO: The ARM-JTAG-EW hardware delays TDI with 3 TCK cycles when in RTCK mode. 
		 * Either handle that here, or update the documentation with examples
		 * how to fix that in the configuration files.
		 */
		if (path[i] == tap_state_transition(tap_get_state(), false))
		{
			armjtagew_tap_append_step(0, 0);
		}
		else if (path[i] == tap_state_transition(tap_get_state(), true))
		{
			armjtagew_tap_append_step(1, 0);
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

void armjtagew_runtest(int num_cycles)
{
	int i;

	tap_state_t saved_end_state = tap_get_end_state();

	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE)
	{
		armjtagew_end_state(TAP_IDLE);
		armjtagew_state_move();
	}

	/* execute num_cycles */
	for (i = 0; i < num_cycles; i++)
	{
		armjtagew_tap_append_step(0, 0);
	}

	/* finish in end_state */
	armjtagew_end_state(saved_end_state);
	if (tap_get_state() != tap_get_end_state())
	{
		armjtagew_state_move();
	}
}

void armjtagew_scan(int ir_scan, enum scan_type type, u8 *buffer, int scan_size, scan_command_t *command)
{
	tap_state_t saved_end_state;

	armjtagew_tap_ensure_space(1, scan_size + 8);

	saved_end_state = tap_get_end_state();

	/* Move to appropriate scan state */
	armjtagew_end_state(ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT);

	armjtagew_state_move();
	armjtagew_end_state(saved_end_state);

	/* Scan */
	armjtagew_tap_append_scan(scan_size, buffer, command);

	/* We are in Exit1, go to Pause */
	armjtagew_tap_append_step(0, 0);

	tap_set_state(ir_scan ? TAP_IRPAUSE : TAP_DRPAUSE);

	if (tap_get_state() != tap_get_end_state())
	{
		armjtagew_state_move();
	}
}

void armjtagew_reset(int trst, int srst)
{
	const u8 trst_mask = (1u<<5);
	const u8 srst_mask = (1u<<6);
	u8 val = 0;
	u8 outp_en = 0;
	u8 change_mask = 0;
	int result;

	LOG_DEBUG("trst: %i, srst: %i", trst, srst);

	if (srst == 0)
	{
		val |= srst_mask;
		outp_en &= ~srst_mask;		/* tristate */
		change_mask |= srst_mask;
	}
	else if (srst == 1)
	{
		val &= ~srst_mask;
		outp_en |= srst_mask;
		change_mask |= srst_mask;
	}

	if (trst == 0)
	{
		val |= trst_mask;
		outp_en &= ~trst_mask;		/* tristate */
		change_mask |= trst_mask;
	}
	else if (trst == 1)
	{
		val &= ~trst_mask;
		outp_en |= trst_mask;
		change_mask |= trst_mask;
	}

	usb_out_buffer[0] = CMD_SET_TAPHW_STATE;
	usb_out_buffer[1] = val;
	usb_out_buffer[2] = outp_en;
	usb_out_buffer[3] = change_mask;
	result = armjtagew_usb_write(armjtagew_jtag_handle, 4);
	if (result != 4)
	{
		LOG_ERROR("ARM-JTAG-EW TRST/SRST pin set failed failed (%d)", result);
	}
}


int armjtagew_get_status(void)
{
	int result;

	usb_out_buffer[0] = CMD_GET_TAPHW_STATE;
	result = armjtagew_usb_message(armjtagew_jtag_handle, 1, 12);

	if (result == 0)
	{
		unsigned int u_tg = buf_get_u32(usb_in_buffer, 0, 16);
		LOG_INFO("U_tg = %d mV, U_aux = %d mV, U_tgpwr = %d mV, I_tgpwr = %d mA, D1 = %d, Target power %s %s\n", \
			buf_get_u32(usb_in_buffer + 0, 0, 16), \
			buf_get_u32(usb_in_buffer + 2, 0, 16), \
			buf_get_u32(usb_in_buffer + 4, 0, 16), \
			buf_get_u32(usb_in_buffer + 6, 0, 16), \
			usb_in_buffer[9], \
			usb_in_buffer[11] ? "OVERCURRENT" : "OK", \
			usb_in_buffer[10] ? "enabled" : "disabled");

		if (u_tg < 1500)
		{
			LOG_ERROR("Vref too low. Check Target Power\n");
		}
	}
	else
	{
		LOG_ERROR("ARM-JTAG-EW command CMD_GET_TAPHW_STATE failed (%d)\n", result);
	}

	return ERROR_OK;
}

int armjtagew_get_version_info(void)
{
	int result;
	char sn[16];
	char auxinfo[257];

	/* query hardware version */
	usb_out_buffer[0] = CMD_GET_VERSION;
	result = armjtagew_usb_message(armjtagew_jtag_handle, 1, 4+15+256);

	if (result != 0) 
	{
		LOG_ERROR("ARM-JTAG-EW command CMD_GET_VERSION failed (%d)\n", result);
		return ERROR_JTAG_DEVICE_ERROR;
	}


	memcpy(sn, usb_in_buffer+4, 15);
	sn[15] = '\0';
	memcpy(auxinfo, usb_in_buffer+4+15, 256);
	auxinfo[256] = '\0';

	LOG_INFO("ARM-JTAG-EW firmware version %d.%d, hardware revision %c, SN=%s, Additional info: %s", \
			usb_in_buffer[1], usb_in_buffer[0], \
			isgraph(usb_in_buffer[2]) ? usb_in_buffer[2] : 'X', \
			sn, auxinfo);
	return ERROR_OK;
}

int armjtagew_handle_armjtagew_info_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (armjtagew_get_version_info() == ERROR_OK)
	{
		/* attempt to get status */
		armjtagew_get_status();
	}

	return ERROR_OK;
}

/***************************************************************************/
/* ARM-JTAG-EW tap functions */

/* 2048 is the max value we can use here */
#define ARMJTAGEW_TAP_BUFFER_SIZE 2048

static int tap_length;
static u8 tms_buffer[ARMJTAGEW_TAP_BUFFER_SIZE];
static u8 tdi_buffer[ARMJTAGEW_TAP_BUFFER_SIZE];
static u8 tdo_buffer[ARMJTAGEW_TAP_BUFFER_SIZE];

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

void armjtagew_tap_init(void)
{
	tap_length = 0;
	pending_scan_results_length = 0;
}

void armjtagew_tap_ensure_space(int scans, int bits)
{
	int available_scans = MAX_PENDING_SCAN_RESULTS - pending_scan_results_length;
	int available_bits = ARMJTAGEW_TAP_BUFFER_SIZE * 8 - tap_length;

	if (scans > available_scans || bits > available_bits)
	{
		armjtagew_tap_execute();
	}
}

void armjtagew_tap_append_step(int tms, int tdi)
{
	last_tms = tms;
	int index = tap_length / 8;

	if (index < ARMJTAGEW_TAP_BUFFER_SIZE)
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
		LOG_ERROR("armjtagew_tap_append_step, overflow");
	}
}

void armjtagew_tap_append_scan(int length, u8 *buffer, scan_command_t *command)
{
	pending_scan_result_t *pending_scan_result = &pending_scan_results_buffer[pending_scan_results_length];
	int i;

	pending_scan_result->first = tap_length;
	pending_scan_result->length = length;
	pending_scan_result->command = command;
	pending_scan_result->buffer = buffer;

	for (i = 0; i < length; i++)
	{
		armjtagew_tap_append_step((i < length-1 ? 0 : 1), (buffer[i/8] >> (i%8)) & 1);
	}
	pending_scan_results_length++;
}

/* Pad and send a tap sequence to the device, and receive the answer.
 * For the purpose of padding we assume that we are in idle or pause state. */
int armjtagew_tap_execute(void)
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
			armjtagew_tap_append_step(last_tms, 0);
		}

		byte_length = tap_length / 8;

		usb_out_buffer[0] = CMD_TAP_SHIFT;
		buf_set_u32(usb_out_buffer+1, 0, 16, byte_length);

		tms_offset = 3;
		for (i = 0; i < byte_length; i++)
		{
			usb_out_buffer[tms_offset + i] = flip_u32(tms_buffer[i],8);
		}

		tdi_offset = tms_offset + byte_length;
		for (i = 0; i < byte_length; i++)
		{
			usb_out_buffer[tdi_offset + i] = flip_u32(tdi_buffer[i],8);
		}

		result = armjtagew_usb_message(armjtagew_jtag_handle, 3 + 2 * byte_length, byte_length + 4);

		if (result == 0)
		{
			int stat;

			stat = (int)buf_get_u32(usb_in_buffer + byte_length, 0, 32);
			if(stat) {
				LOG_ERROR("armjtagew_tap_execute, emulator returned error code %d for a CMD_TAP_SHIFT command", stat);
				return ERROR_JTAG_QUEUE_FAILED;
			}

			for (i = 0; i < byte_length; i++)
			{
				tdo_buffer[i] = flip_u32(usb_in_buffer[i],8);
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
				armjtagew_debug_buffer(buffer, byte_length);
#endif

				if (jtag_read_buffer(buffer, command) != ERROR_OK)
				{
					armjtagew_tap_init();
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
			LOG_ERROR("armjtagew_tap_execute, wrong result %d, expected %d", result, byte_length);
			return ERROR_JTAG_QUEUE_FAILED;
		}

		armjtagew_tap_init();
	}

	return ERROR_OK;
}

/*****************************************************************************/
/* JLink USB low-level functions */

armjtagew_jtag_t* armjtagew_usb_open()
{
	struct usb_bus *busses;
	struct usb_bus *bus;
	struct usb_device *dev;

	armjtagew_jtag_t *result;

	result = (armjtagew_jtag_t*) malloc(sizeof(armjtagew_jtag_t));

	usb_init();
	usb_find_busses();
	usb_find_devices();

	busses = usb_get_busses();

	/* find armjtagew_jtag device in usb bus */

	for (bus = busses; bus; bus = bus->next)
	{
		for (dev = bus->devices; dev; dev = dev->next)
		{
			if ((dev->descriptor.idVendor == USB_VID) && (dev->descriptor.idProduct == USB_PID))
			{
				result->usb_handle = usb_open(dev);

#if 0
				/* usb_set_configuration required under win32 */
				usb_set_configuration(result->usb_handle, dev->config[0].bConfigurationValue);
#endif
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

void armjtagew_usb_close(armjtagew_jtag_t *armjtagew_jtag)
{
	usb_close(armjtagew_jtag->usb_handle);
	free(armjtagew_jtag);
}

/* Send a message and receive the reply. */
int armjtagew_usb_message(armjtagew_jtag_t *armjtagew_jtag, int out_length, int in_length)
{
	int result;

	result = armjtagew_usb_write(armjtagew_jtag, out_length);
	if (result == out_length)
	{
		result = armjtagew_usb_read(armjtagew_jtag, in_length);
		if (result != in_length) 
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
	return 0;
}

/* Write data from out_buffer to USB. */
int armjtagew_usb_write(armjtagew_jtag_t *armjtagew_jtag, int out_length)
{
	int result;

	if (out_length > ARMJTAGEW_OUT_BUFFER_SIZE)
	{
		LOG_ERROR("armjtagew_jtag_write illegal out_length=%d (max=%d)", out_length, ARMJTAGEW_OUT_BUFFER_SIZE);
		return -1;
	}

	result = usb_bulk_write(armjtagew_jtag->usb_handle, ARMJTAGEW_EPT_BULK_OUT, \
		(char*)usb_out_buffer, out_length, ARMJTAGEW_USB_TIMEOUT);

	DEBUG_JTAG_IO("armjtagew_usb_write, out_length = %d, result = %d", out_length, result);

#ifdef _DEBUG_USB_COMMS_
	armjtagew_debug_buffer(usb_out_buffer, out_length);
#endif
	return result;
}

/* Read data from USB into in_buffer. */
int armjtagew_usb_read(armjtagew_jtag_t *armjtagew_jtag, int exp_in_length)
{
	int result = usb_bulk_read(armjtagew_jtag->usb_handle, ARMJTAGEW_EPT_BULK_IN, \
		(char*)usb_in_buffer, exp_in_length, ARMJTAGEW_USB_TIMEOUT);

	DEBUG_JTAG_IO("armjtagew_usb_read, result = %d", result);

#ifdef _DEBUG_USB_COMMS_
	armjtagew_debug_buffer(usb_in_buffer, result);
#endif
	return result;
}


#ifdef _DEBUG_USB_COMMS_
#define BYTES_PER_LINE  16

void armjtagew_debug_buffer(u8 *buffer, int length)
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

