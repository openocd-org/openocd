/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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

/* system includes */

#ifdef _WIN32
#include "errno.h"
#endif /* _WIN32 */

#include <string.h>
#include <stdlib.h>

#if PARPORT_USE_PPDEV == 1
#include <linux/parport.h>
#include <linux/ppdev.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <unistd.h>
#else /* not PARPORT_USE_PPDEV */
#ifndef _WIN32
#include <sys/io.h>
#endif
#endif

#if PARPORT_USE_GIVEIO == 1
#if IS_CYGWIN == 1
#include <windows.h>
#include <errno.h>
#endif
#endif

#include "log.h"

/* configuration */
u16 amt_jtagaccel_port;

/* interface variables
 */
static u8 aw_control_rst = 0x00;
static u8 aw_control_fsm = 0x10;
static u8 aw_control_baudrate = 0x20;

static int rtck_enabled = 0;

#if PARPORT_USE_PPDEV == 1
static int device_handle;
int addr_mode = IEEE1284_MODE_EPP | IEEE1284_ADDR ;
int data_mode = IEEE1284_MODE_EPP | IEEE1284_DATA ;
#define AMT_AW(val)	do { ioctl(device_handle, PPSETMODE, &addr_mode); write(device_handle, &val, 1); } while (0)
#define AMT_AR(val)	do { ioctl(device_handle, PPSETMODE, &addr_mode); read(device_handle, &val, 1); } while (0)
#define AMT_DW(val)	do { ioctl(device_handle, PPSETMODE, &data_mode); write(device_handle, &val, 1); } while (0)
#define AMT_DR(val)	do { ioctl(device_handle, PPSETMODE, &data_mode); read(device_handle, &val, 1); } while (0)
#else
#define AMT_AW(val)	do { outb(val, amt_jtagaccel_port + 3); } while (0)
#define AMT_AR(val)	do { val = inb(amt_jtagaccel_port + 3); } while (0)
#define AMT_DW(val)	do { outb(val, amt_jtagaccel_port + 4); } while (0)
#define AMT_DR(val)	do { val = inb(amt_jtagaccel_port + 4); } while (0)
#endif

int amt_jtagaccel_execute_queue(void);
int amt_jtagaccel_register_commands(struct command_context_s *cmd_ctx);
int amt_jtagaccel_speed(int speed);
int amt_jtagaccel_init(void);
int amt_jtagaccel_quit(void);

int amt_jtagaccel_handle_parport_port_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int amt_jtagaccel_handle_rtck_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

/* tap_move[i][j]: tap movement command to go from state i to state j
 * 0: Test-Logic-Reset
 * 1: Run-Test/Idle
 * 2: Shift-DR
 * 3: Pause-DR
 * 4: Shift-IR
 * 5: Pause-IR
 */
u8 amt_jtagaccel_tap_move[6][6][2] =
{
	/*	   RESET         IDLE        DRSHIFT       DRPAUSE       IRSHIFT       IRPAUSE             */
	{{0x1f, 0x00}, {0x0f, 0x00}, {0x8a, 0x04}, {0x0a, 0x00}, {0x06, 0x00}, {0x96, 0x00}},	/* RESET */
	{{0x1f, 0x00}, {0x00, 0x00}, {0x85, 0x08}, {0x05, 0x00}, {0x8b, 0x08}, {0x0b, 0x00}},	/* IDLE */
	{{0x1f, 0x00}, {0x0d, 0x00}, {0x00, 0x00}, {0x01, 0x00}, {0x8f, 0x09}, {0x8f, 0x01}},	/* DRSHIFT  */
	{{0x1f, 0x00}, {0x0c, 0x00}, {0x08, 0x00}, {0x00, 0x00}, {0x8f, 0x09}, {0x8f, 0x01}},	/* DRPAUSE  */
	{{0x1f, 0x00}, {0x0d, 0x00}, {0x07, 0x00}, {0x97, 0x00}, {0x00, 0x00}, {0x01, 0x00}},	/* IRSHIFT  */
	{{0x1f, 0x00}, {0x0c, 0x00}, {0x07, 0x00}, {0x97, 0x00}, {0x08, 0x00}, {0x00, 0x00}},	/* IRPAUSE  */
};


jtag_interface_t amt_jtagaccel_interface =
{
	.name = "amt_jtagaccel",

	.execute_queue = amt_jtagaccel_execute_queue,

	.speed = amt_jtagaccel_speed,
	.register_commands = amt_jtagaccel_register_commands,
	.init = amt_jtagaccel_init,
	.quit = amt_jtagaccel_quit,
};

int amt_jtagaccel_register_commands(struct command_context_s *cmd_ctx)
{
	register_command(cmd_ctx, NULL, "parport_port", amt_jtagaccel_handle_parport_port_command,
					 COMMAND_CONFIG, NULL);
	register_command(cmd_ctx, NULL, "rtck", amt_jtagaccel_handle_rtck_command,
					 COMMAND_CONFIG, NULL);

	return ERROR_OK;
}

void amt_jtagaccel_reset(int trst, int srst)
{
	if (trst == 1)
		aw_control_rst |= 0x4;
	else if (trst == 0)
		aw_control_rst &= ~0x4;

	if (srst == 1)
		aw_control_rst |= 0x1;
	else if (srst == 0)
		aw_control_rst &= ~0x1;

	AMT_AW(aw_control_rst);
}

int amt_jtagaccel_speed(int speed)
{
	aw_control_baudrate &= 0xf0;
	aw_control_baudrate |= speed & 0x0f;
	AMT_AW(aw_control_baudrate);

	return ERROR_OK;
}

void amt_jtagaccel_end_state(int state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else
	{
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

void amt_wait_scan_busy(void)
{
	int timeout = 4096;
	u8 ar_status;

	AMT_AR(ar_status);
	while (((ar_status) & 0x80) && (timeout-- > 0))
		AMT_AR(ar_status);

	if (ar_status & 0x80)
	{
		LOG_ERROR("amt_jtagaccel timed out while waiting for end of scan, rtck was %s, last AR_STATUS: 0x%2.2x", (rtck_enabled) ? "enabled" : "disabled", ar_status);
		exit(-1);
	}
}

void amt_jtagaccel_state_move(void)
{
	u8 aw_scan_tms_5;
	u8 tms_scan[2];

	tap_state_t	cur_state = tap_get_state();
	tap_state_t	end_state = tap_get_end_state();

	tms_scan[0] = amt_jtagaccel_tap_move[tap_move_ndx(cur_state)][tap_move_ndx(end_state)][0];
	tms_scan[1] = amt_jtagaccel_tap_move[tap_move_ndx(cur_state)][tap_move_ndx(end_state)][1];

	aw_scan_tms_5 = 0x40 | (tms_scan[0] & 0x1f);
	AMT_AW(aw_scan_tms_5);
	if (jtag_speed > 3 || rtck_enabled)
		amt_wait_scan_busy();

	if (tms_scan[0] & 0x80)
	{
		aw_scan_tms_5 = 0x40 | (tms_scan[1] & 0x1f);
		AMT_AW(aw_scan_tms_5);
		if (jtag_speed > 3 || rtck_enabled)
			amt_wait_scan_busy();
	}

	tap_set_state(end_state);
}

void amt_jtagaccel_runtest(int num_cycles)
{
	int i = 0;
	u8 aw_scan_tms_5;
	u8 aw_scan_tms_1to4;

	tap_state_t saved_end_state = tap_get_end_state();

	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE)
	{
		amt_jtagaccel_end_state(TAP_IDLE);
		amt_jtagaccel_state_move();
	}

	while (num_cycles - i >= 5)
	{
		aw_scan_tms_5 = 0x40;
		AMT_AW(aw_scan_tms_5);
		i += 5;
	}

	if (num_cycles - i > 0)
	{
		aw_scan_tms_1to4 = 0x80 | ((num_cycles - i - 1) & 0x3) << 4;
		AMT_AW(aw_scan_tms_1to4);
	}

	amt_jtagaccel_end_state(saved_end_state);
	if (tap_get_state() != tap_get_end_state())
		amt_jtagaccel_state_move();
}

void amt_jtagaccel_scan(int ir_scan, enum scan_type type, u8 *buffer, int scan_size)
{
	int bits_left = scan_size;
	int bit_count = 0;
	tap_state_t saved_end_state = tap_get_end_state();
	u8 aw_tdi_option;
	u8 dw_tdi_scan;
	u8 dr_tdo;
	u8 aw_tms_scan;
	u8 tms_scan[2];

	if (ir_scan)
		amt_jtagaccel_end_state(TAP_IRSHIFT);
	else
		amt_jtagaccel_end_state(TAP_DRSHIFT);

	amt_jtagaccel_state_move();
	amt_jtagaccel_end_state(saved_end_state);

	/* handle unaligned bits at the beginning */
	if ((scan_size - 1) % 8)
	{
		aw_tdi_option = 0x30 | (((scan_size - 1) % 8) - 1);
		AMT_AW(aw_tdi_option);

		dw_tdi_scan = buf_get_u32(buffer, bit_count, (scan_size - 1) % 8) & 0xff;
		AMT_DW(dw_tdi_scan);
		if (jtag_speed > 3 || rtck_enabled)
			amt_wait_scan_busy();

		if ((type == SCAN_IN) || (type == SCAN_IO))
		{
			AMT_DR(dr_tdo);
			dr_tdo = dr_tdo >> (8 - ((scan_size - 1) % 8));
			buf_set_u32(buffer, bit_count, (scan_size - 1) % 8, dr_tdo);
		}

		bit_count += (scan_size - 1) % 8;
		bits_left -= (scan_size - 1) % 8;
	}

	while (bits_left - 1 >= 8)
	{
		dw_tdi_scan = buf_get_u32(buffer, bit_count, 8) & 0xff;
		AMT_DW(dw_tdi_scan);
		if (jtag_speed > 3 || rtck_enabled)
			amt_wait_scan_busy();

		if ((type == SCAN_IN) || (type == SCAN_IO))
		{
			AMT_DR(dr_tdo);
			buf_set_u32(buffer, bit_count, 8, dr_tdo);
		}

		bit_count += 8;
		bits_left -= 8;
	}

	tms_scan[0] = amt_jtagaccel_tap_move[tap_move_ndx(tap_get_state())][tap_move_ndx(tap_get_end_state())][0];
	tms_scan[1] = amt_jtagaccel_tap_move[tap_move_ndx(tap_get_state())][tap_move_ndx(tap_get_end_state())][1];
	aw_tms_scan = 0x40 | (tms_scan[0] & 0x1f) | (buf_get_u32(buffer, bit_count, 1) << 5);
	AMT_AW(aw_tms_scan);
	if (jtag_speed > 3 || rtck_enabled)
		amt_wait_scan_busy();

	if ((type == SCAN_IN) || (type == SCAN_IO))
	{
		AMT_DR(dr_tdo);
		dr_tdo = dr_tdo >> 7;
		buf_set_u32(buffer, bit_count, 1, dr_tdo);
	}

	if (tms_scan[0] & 0x80)
	{
		aw_tms_scan = 0x40 | (tms_scan[1] & 0x1f);
		AMT_AW(aw_tms_scan);
		if (jtag_speed > 3 || rtck_enabled)
			amt_wait_scan_busy();
	}
	tap_set_state(tap_get_end_state());
}

int amt_jtagaccel_execute_queue(void)
{
	jtag_command_t *cmd = jtag_command_queue; /* currently processed command */
	int scan_size;
	enum scan_type type;
	u8 *buffer;
	int retval;

	/* return ERROR_OK, unless a jtag_read_buffer returns a failed check
	 * that wasn't handled by a caller-provided error handler
	 */
	retval = ERROR_OK;

	while (cmd)
	{
		switch (cmd->type)
		{
			case JTAG_END_STATE:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("end_state: %i", cmd->cmd.end_state->end_state);
#endif
				if (cmd->cmd.end_state->end_state != -1)
					amt_jtagaccel_end_state(cmd->cmd.end_state->end_state);
				break;
			case JTAG_RESET:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("reset trst: %i srst %i", cmd->cmd.reset->trst, cmd->cmd.reset->srst);
#endif
				if (cmd->cmd.reset->trst == 1)
				{
					tap_set_state(TAP_RESET);
				}
				amt_jtagaccel_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
				break;
			case JTAG_RUNTEST:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("runtest %i cycles, end in %i", cmd->cmd.runtest->num_cycles, cmd->cmd.runtest->end_state);
#endif
				if (cmd->cmd.runtest->end_state != -1)
					amt_jtagaccel_end_state(cmd->cmd.runtest->end_state);
				amt_jtagaccel_runtest(cmd->cmd.runtest->num_cycles);
				break;
			case JTAG_STATEMOVE:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("statemove end in %i", cmd->cmd.statemove->end_state);
#endif
				if (cmd->cmd.statemove->end_state != -1)
					amt_jtagaccel_end_state(cmd->cmd.statemove->end_state);
				amt_jtagaccel_state_move();
				break;
			case JTAG_SCAN:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("scan end in %i", cmd->cmd.scan->end_state);
#endif
				if (cmd->cmd.scan->end_state != -1)
					amt_jtagaccel_end_state(cmd->cmd.scan->end_state);
				scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
				type = jtag_scan_type(cmd->cmd.scan);
				amt_jtagaccel_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size);
				if (jtag_read_buffer(buffer, cmd->cmd.scan) != ERROR_OK)
					retval = ERROR_JTAG_QUEUE_FAILED;
				if (buffer)
					free(buffer);
				break;
			case JTAG_SLEEP:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("sleep %i", cmd->cmd.sleep->us);
#endif
				jtag_sleep(cmd->cmd.sleep->us);
				break;
			default:
				LOG_ERROR("BUG: unknown JTAG command type encountered");
				exit(-1);
		}
		cmd = cmd->next;
	}

	return retval;
}

#if PARPORT_USE_GIVEIO == 1
int amt_jtagaccel_get_giveio_access(void)
{
	HANDLE h;
	OSVERSIONINFO version;

	version.dwOSVersionInfoSize = sizeof version;
	if (!GetVersionEx( &version )) {
	errno = EINVAL;
	return -1;
	}
	if (version.dwPlatformId != VER_PLATFORM_WIN32_NT)
	return 0;

	h = CreateFile( "\\\\.\\giveio", GENERIC_READ, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL );
	if (h == INVALID_HANDLE_VALUE) {
	errno = ENODEV;
	return -1;
	}

	CloseHandle( h );

	return 0;
}
#endif

int amt_jtagaccel_init(void)
{
#if PARPORT_USE_PPDEV == 1
	char buffer[256];
	int i = 0;
	u8 control_port;
#else
	u8 status_port;
#endif
	u8 ar_status;

#if PARPORT_USE_PPDEV == 1
	if (device_handle > 0)
	{
		LOG_ERROR("device is already opened");
		return ERROR_JTAG_INIT_FAILED;
	}

	snprintf(buffer, 256, "/dev/parport%d", amt_jtagaccel_port);
	device_handle = open(buffer, O_RDWR);

	if (device_handle < 0)
	{
		LOG_ERROR("cannot open device. check it exists and that user read and write rights are set");
		return ERROR_JTAG_INIT_FAILED;
	}

	i = ioctl(device_handle, PPCLAIM);
	if (i < 0)
	{
		LOG_ERROR("cannot claim device");
		return ERROR_JTAG_INIT_FAILED;
	}

	i = IEEE1284_MODE_EPP;
	i = ioctl(device_handle, PPSETMODE, & i);
	if (i < 0)
	{
		LOG_ERROR(" cannot set compatible mode to device");
		return ERROR_JTAG_INIT_FAILED;
	}

	control_port = 0x00;
	i = ioctl(device_handle, PPWCONTROL, &control_port);

	control_port = 0x04;
	i = ioctl(device_handle, PPWCONTROL, &control_port);

#else
	if (amt_jtagaccel_port == 0)
	{
		amt_jtagaccel_port = 0x378;
		LOG_WARNING("No parport port specified, using default '0x378' (LPT1)");
	}

#if PARPORT_USE_GIVEIO == 1
	if (amt_jtagaccel_get_giveio_access() != 0) {
#else /* PARPORT_USE_GIVEIO */
	if (ioperm(amt_jtagaccel_port, 5, 1) != 0) {
#endif /* PARPORT_USE_GIVEIO */
		LOG_ERROR("missing privileges for direct i/o");
		return ERROR_JTAG_INIT_FAILED;
	}

	/* prepare epp port */
	/* clear timeout */
	status_port = inb(amt_jtagaccel_port + 1);
	outb(status_port | 0x1, amt_jtagaccel_port + 1);

	/* reset epp port */
	outb(0x00, amt_jtagaccel_port + 2);
	outb(0x04, amt_jtagaccel_port + 2);
#endif

	if (rtck_enabled)
	{
		/* set RTCK enable bit */
		aw_control_fsm |= 0x02;
	}

	/* enable JTAG port */
	aw_control_fsm |= 0x04;
	AMT_AW(aw_control_fsm);

	amt_jtagaccel_speed(jtag_speed);

	if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
		aw_control_rst &= ~0x8;
	else
		aw_control_rst |= 0x8;

	if (jtag_reset_config & RESET_SRST_PUSH_PULL)
		aw_control_rst &= ~0x2;
	else
		aw_control_rst |= 0x2;

	amt_jtagaccel_reset(0, 0);

	/* read status register */
	AMT_AR(ar_status);
	LOG_DEBUG("AR_STATUS: 0x%2.2x", ar_status);

	return ERROR_OK;
}

int amt_jtagaccel_quit(void)
{

	return ERROR_OK;
}

int amt_jtagaccel_handle_parport_port_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc == 0)
		return ERROR_OK;

	/* only if the port wasn't overwritten by cmdline */
	if (amt_jtagaccel_port == 0)
		amt_jtagaccel_port = strtoul(args[0], NULL, 0);

	return ERROR_OK;
}

int amt_jtagaccel_handle_rtck_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc == 0)
	{
		command_print(cmd_ctx, "amt_jtagaccel RTCK feature %s", (rtck_enabled) ? "enabled" : "disabled");
		return ERROR_OK;
	}
	else
	{
		if (strcmp(args[0], "enabled") == 0)
		{
			rtck_enabled = 1;
		}
		else
		{
			rtck_enabled = 0;
		}
	}

	return ERROR_OK;
}
