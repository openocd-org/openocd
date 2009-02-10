/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
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

#if 1
#define _DEBUG_GW16012_IO_
#endif

/* system includes */
/*  -ino: 060521-1036 */
#if defined(__FreeBSD__) || defined(__FreeBSD_kernel__)

#include <sys/types.h>
#include <machine/sysarch.h>
#include <machine/cpufunc.h>
#define ioperm(startport,length,enable)\
  i386_set_ioperm((startport), (length), (enable))

#else

#ifdef _WIN32
#include "errno.h"
#endif /* _WIN32 */

#endif /* __FreeBSD__, __FreeBSD_kernel__ */

#include <string.h>
#include <stdlib.h>

#if PARPORT_USE_PPDEV == 1
#if defined(__FreeBSD__) || defined(__FreeBSD_kernel__)
#include <dev/ppbus/ppi.h>
#include <dev/ppbus/ppbconf.h>
#define PPRSTATUS	PPIGSTATUS
#define PPWDATA		PPISDATA
#else
#include <linux/parport.h>
#include <linux/ppdev.h>
#endif
#include <fcntl.h>
#include <sys/ioctl.h>
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
u16 gw16012_port;

/* interface variables
 */
static u8 gw16012_msb = 0x0;
static u8 gw16012_control_value = 0x0;

#if PARPORT_USE_PPDEV == 1
static int device_handle;
#endif

int gw16012_execute_queue(void);
int gw16012_register_commands(struct command_context_s *cmd_ctx);
int gw16012_speed(int speed);
int gw16012_init(void);
int gw16012_quit(void);

int gw16012_handle_parport_port_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

jtag_interface_t gw16012_interface =
{
	.name = "gw16012",

	.execute_queue = gw16012_execute_queue,

	.speed = gw16012_speed,
	.register_commands = gw16012_register_commands,
	.init = gw16012_init,
	.quit = gw16012_quit,
};

int gw16012_register_commands(struct command_context_s *cmd_ctx)
{
	register_command(cmd_ctx, NULL, "parport_port", gw16012_handle_parport_port_command,
					 COMMAND_CONFIG, NULL);

	return ERROR_OK;
}

void gw16012_data(u8 value)
{
	value = (value & 0x7f) | gw16012_msb;
	gw16012_msb ^= 0x80; /* toggle MSB */

#ifdef _DEBUG_GW16012_IO_
	LOG_DEBUG("%2.2x", value);
#endif

	#if PARPORT_USE_PPDEV == 1
		ioctl(device_handle, PPWDATA, &value);
	#else
		#if defined(__FreeBSD__) || defined(__FreeBSD_kernel__)
			outb(gw16012_port, value);
		#else
			outb(value, gw16012_port);
		#endif
	#endif
}

void gw16012_control(u8 value)
{
	if (value != gw16012_control_value)
	{
		gw16012_control_value = value;

#ifdef _DEBUG_GW16012_IO_
		LOG_DEBUG("%2.2x", gw16012_control_value);
#endif

		#if PARPORT_USE_PPDEV == 1
			ioctl(device_handle, PPWCONTROL, &gw16012_control_value);
		#else
			#if defined(__FreeBSD__) || defined(__FreeBSD_kernel__)
				outb(gw16012_port + 2, gw16012_control_value);
			#else
				outb(gw16012_control_value, gw16012_port + 2);
			#endif
		#endif
	}
}

void gw16012_input(u8 *value)
{
	#if PARPORT_USE_PPDEV == 1
		ioctl(device_handle, PPRSTATUS, value);
	#else
		*value = inb(gw16012_port + 1);
	#endif

#ifdef _DEBUG_GW16012_IO_
	LOG_DEBUG("%2.2x", *value);
#endif
}

/* (1) assert or (0) deassert reset lines */
void gw16012_reset(int trst, int srst)
{
	LOG_DEBUG("trst: %i, srst: %i", trst, srst);

	if (trst == 0)
		gw16012_control(0x0d);
	else if (trst == 1)
		gw16012_control(0x0c);

	if (srst == 0)
		gw16012_control(0x0a);
	else if (srst == 1)
		gw16012_control(0x0b);
}

int gw16012_speed(int speed)
{

	return ERROR_OK;
}

void gw16012_end_state(int state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else
	{
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

void gw16012_state_move(void)
{
	int i=0, tms=0;
	u8 tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());

	gw16012_control(0x0); /* single-bit mode */

	for (i = 0; i < 7; i++)
	{
		tms = (tms_scan >> i) & 1;
		gw16012_data(tms << 1); /* output next TMS bit */
	}

	tap_set_state(tap_get_end_state());
}

void gw16012_path_move(pathmove_command_t *cmd)
{
	int num_states = cmd->num_states;
	int state_count;

	state_count = 0;
	while (num_states)
	{
		gw16012_control(0x0); /* single-bit mode */
		if (tap_state_transition(tap_get_state(), false) == cmd->path[state_count])
		{
			gw16012_data(0x0); /* TCK cycle with TMS low */
		}
		else if (tap_state_transition(tap_get_state(), true) == cmd->path[state_count])
		{
			gw16012_data(0x2); /* TCK cycle with TMS high */
		}
		else
		{
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition", tap_state_name(tap_get_state()), tap_state_name(cmd->path[state_count]));
			exit(-1);
		}

		tap_set_state(cmd->path[state_count]);
		state_count++;
		num_states--;
	}

	tap_set_end_state(tap_get_state());
}

void gw16012_runtest(int num_cycles)
{
	tap_state_t saved_end_state = tap_get_end_state();
	int i;

	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE)
	{
		gw16012_end_state(TAP_IDLE);
		gw16012_state_move();
	}

	for (i = 0; i < num_cycles; i++)
	{
		gw16012_control(0x0); /* single-bit mode */
		gw16012_data(0x0); /* TMS cycle with TMS low */
	}

	gw16012_end_state(saved_end_state);
	if (tap_get_state() != tap_get_end_state())
		gw16012_state_move();
}

void gw16012_scan(int ir_scan, enum scan_type type, u8 *buffer, int scan_size)
{
	int bits_left = scan_size;
	int bit_count = 0;
	tap_state_t saved_end_state = tap_get_end_state();
	u8 scan_out, scan_in;

	/* only if we're not already in the correct Shift state */
	if (!((!ir_scan && (tap_get_state() == TAP_DRSHIFT)) || (ir_scan && (tap_get_state() == TAP_IRSHIFT))))
	{
		if (ir_scan)
			gw16012_end_state(TAP_IRSHIFT);
		else
			gw16012_end_state(TAP_DRSHIFT);

		gw16012_state_move();
		gw16012_end_state(saved_end_state);
	}

	while (type == SCAN_OUT && ((bits_left - 1) > 7))
	{
		gw16012_control(0x2); /* seven-bit mode */
		scan_out = buf_get_u32(buffer, bit_count, 7);
		gw16012_data(scan_out);
		bit_count += 7;
		bits_left -= 7;
	}

	gw16012_control(0x0); /* single-bit mode */
	while (bits_left-- > 0)
	{
		u8 tms = 0;

		scan_out = buf_get_u32(buffer, bit_count, 1);

		if (bits_left == 0) /* last bit */
		{
			if ((ir_scan && (tap_get_end_state() == TAP_IRSHIFT))
				|| (!ir_scan && (tap_get_end_state() == TAP_DRSHIFT)))
			{
				tms = 0;
			}
			else
			{
				tms = 2;
			}
		}

		gw16012_data(scan_out | tms);

		if (type != SCAN_OUT)
		{
			gw16012_input(&scan_in);
			buf_set_u32(buffer, bit_count, 1, ((scan_in & 0x08) >> 3));
		}

		bit_count++;
	}

	if (!((ir_scan && (tap_get_end_state() == TAP_IRSHIFT)) ||
		(!ir_scan && (tap_get_end_state() == TAP_DRSHIFT))))
	{
		gw16012_data(0x0);
		if (ir_scan)
			tap_set_state(TAP_IRPAUSE);
		else
			tap_set_state(TAP_DRPAUSE);

		if (tap_get_state() != tap_get_end_state())
			gw16012_state_move();
	}
}

int gw16012_execute_queue(void)
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
					gw16012_end_state(cmd->cmd.end_state->end_state);
				break;
			case JTAG_RESET:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("reset trst: %i srst %i", cmd->cmd.reset->trst, cmd->cmd.reset->srst);
#endif
				if (cmd->cmd.reset->trst == 1)
				{
					tap_set_state(TAP_RESET);
				}
				gw16012_reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
				break;
			case JTAG_RUNTEST:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("runtest %i cycles, end in %i", cmd->cmd.runtest->num_cycles, cmd->cmd.runtest->end_state);
#endif
				if (cmd->cmd.runtest->end_state != -1)
					gw16012_end_state(cmd->cmd.runtest->end_state);
				gw16012_runtest(cmd->cmd.runtest->num_cycles);
				break;
			case JTAG_STATEMOVE:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("statemove end in %i", cmd->cmd.statemove->end_state);
#endif
				if (cmd->cmd.statemove->end_state != -1)
					gw16012_end_state(cmd->cmd.statemove->end_state);
				gw16012_state_move();
				break;
			case JTAG_PATHMOVE:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("pathmove: %i states, end in %i", cmd->cmd.pathmove->num_states, cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);
#endif
				gw16012_path_move(cmd->cmd.pathmove);
				break;
			case JTAG_SCAN:
				if (cmd->cmd.scan->end_state != -1)
					gw16012_end_state(cmd->cmd.scan->end_state);
				scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
				type = jtag_scan_type(cmd->cmd.scan);
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("%s scan (%i) %i bit end in %i", (cmd->cmd.scan->ir_scan) ? "ir" : "dr",
					type, scan_size, cmd->cmd.scan->end_state);
#endif
				gw16012_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size);
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
int gw16012_get_giveio_access()
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

int gw16012_init(void)
{
#if PARPORT_USE_PPDEV == 1
	char buffer[256];
	int i = 0;
#endif
	u8 status_port;

#if PARPORT_USE_PPDEV == 1
	if (device_handle>0)
	{
		LOG_ERROR("device is already opened");
		return ERROR_JTAG_INIT_FAILED;
	}

#if defined(__FreeBSD__) || defined(__FreeBSD_kernel__)
	LOG_DEBUG("opening /dev/ppi%d...", gw16012_port);

	snprintf(buffer, 256, "/dev/ppi%d", gw16012_port);
	device_handle = open(buffer, O_WRONLY);
#else
	LOG_DEBUG("opening /dev/parport%d...", gw16012_port);

	snprintf(buffer, 256, "/dev/parport%d", gw16012_port);
	device_handle = open(buffer, O_WRONLY);
#endif
	if (device_handle<0)
	{
		LOG_ERROR("cannot open device. check it exists and that user read and write rights are set");
		return ERROR_JTAG_INIT_FAILED;
	}

	LOG_DEBUG("...open");

#if defined(__FreeBSD__) || defined(__FreeBSD_kernel__)
	i=ioctl(device_handle, PPCLAIM);
	if (i<0)
	{
		LOG_ERROR("cannot claim device");
		return ERROR_JTAG_INIT_FAILED;
	}

	i = PARPORT_MODE_COMPAT;
	i= ioctl(device_handle, PPSETMODE, & i);
	if (i<0)
	{
		LOG_ERROR(" cannot set compatible mode to device");
		return ERROR_JTAG_INIT_FAILED;
	}

	i = IEEE1284_MODE_COMPAT;
	i = ioctl(device_handle, PPNEGOT, & i);
	if (i<0)
	{
		LOG_ERROR("cannot set compatible 1284 mode to device");
		return ERROR_JTAG_INIT_FAILED;
	}
#endif
#else
	if (gw16012_port == 0)
	{
		gw16012_port = 0x378;
		LOG_WARNING("No gw16012 port specified, using default '0x378' (LPT1)");
	}

	LOG_DEBUG("requesting privileges for parallel port 0x%lx...", (long unsigned)(gw16012_port) );
#if PARPORT_USE_GIVEIO == 1
	if (gw16012_get_giveio_access() != 0)
#else /* PARPORT_USE_GIVEIO */
	if (ioperm(gw16012_port, 3, 1) != 0)
#endif /* PARPORT_USE_GIVEIO */
	{
		LOG_ERROR("missing privileges for direct i/o");
		return ERROR_JTAG_INIT_FAILED;
	}
	LOG_DEBUG("...privileges granted");

	/* make sure parallel port is in right mode (clear tristate and interrupt */
#if defined(__FreeBSD__) || defined(__FreeBSD_kernel__)
	outb(gw16012_port + 2, 0x0);
#else
	outb(0x0, gw16012_port + 2);
#endif
#endif /* PARPORT_USE_PPDEV */

	gw16012_input(&status_port);
	gw16012_msb = (status_port & 0x80) ^ 0x80;

	gw16012_speed(jtag_speed);
	gw16012_reset(0, 0);

	return ERROR_OK;
}

int gw16012_quit(void)
{

	return ERROR_OK;
}

int gw16012_handle_parport_port_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc == 0)
		return ERROR_OK;

	/* only if the port wasn't overwritten by cmdline */
	if (gw16012_port == 0)
		gw16012_port = strtoul(args[0], NULL, 0);

	return ERROR_OK;
}
