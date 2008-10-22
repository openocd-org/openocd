/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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
#include "bitbang.h"

/* system includes */
/* -ino: 060521-1036 */
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

#endif /* __FreeBSD__ */

#include <string.h>
#include <stdlib.h>
#include <stdio.h>

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

/* parallel port cable description
 */
typedef struct cable_s
{
	char* name;
	u8 TDO_MASK;	/* status port bit containing current TDO value */
	u8 TRST_MASK;	/* data port bit for TRST */
	u8 TMS_MASK;	/* data port bit for TMS */
	u8 TCK_MASK;	/* data port bit for TCK */
	u8 TDI_MASK;	/* data port bit for TDI */
	u8 SRST_MASK;	/* data port bit for SRST */
	u8 OUTPUT_INVERT;	/* data port bits that should be inverted */
	u8 INPUT_INVERT;	/* status port that should be inverted */
	u8 PORT_INIT;	/* initialize data port with this value */
	u8 PORT_EXIT;	/* de-initialize data port with this value */
	u8 LED_MASK;	/* data port bit for LED */
} cable_t;

cable_t cables[] = 
{	
	/* name					tdo   trst  tms   tck   tdi   srst  o_inv i_inv init  exit  led */
	{ "wiggler",			0x80, 0x10, 0x02, 0x04, 0x08, 0x01, 0x01, 0x80, 0x80, 0x80, 0x00 },
	{ "wiggler2",			0x80, 0x10, 0x02, 0x04, 0x08, 0x01, 0x01, 0x80, 0x80, 0x00, 0x20 },
	{ "wiggler_ntrst_inverted",
							0x80, 0x10, 0x02, 0x04, 0x08, 0x01, 0x11, 0x80, 0x80, 0x80, 0x00 },
	{ "old_amt_wiggler",	0x80, 0x01, 0x02, 0x04, 0x08, 0x10, 0x11, 0x80, 0x80, 0x80, 0x00 },
	{ "arm-jtag",			0x80, 0x01, 0x02, 0x04, 0x08, 0x10, 0x01, 0x80, 0x80, 0x80, 0x00 },
	{ "chameleon",			0x80, 0x00, 0x04, 0x01, 0x02, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00 },
	{ "dlc5",				0x10, 0x00, 0x04, 0x02, 0x01, 0x00, 0x00, 0x00, 0x10, 0x10, 0x00 },
	{ "triton",				0x80, 0x08, 0x04, 0x01, 0x02, 0x00, 0x00, 0x80, 0x00, 0x00, 0x00 },
	{ "lattice",			0x40, 0x10, 0x04, 0x02, 0x01, 0x08, 0x00, 0x00, 0x18, 0x18, 0x00 },
	{ "flashlink",			0x20, 0x10, 0x02, 0x01, 0x04, 0x20, 0x30, 0x20, 0x00, 0x00, 0x00 },
/* Altium Universal JTAG cable. Set the cable to Xilinx Mode and wire to target as follows:
	HARD TCK - Target TCK
	HARD TMS - Target TMS
	HARD TDI - Target TDI
	HARD TDO - Target TDO
	SOFT TCK - Target TRST
	SOFT TDI - Target SRST
*/
	{ "altium",			0x10, 0x20, 0x04, 0x02, 0x01, 0x80, 0x00, 0x00, 0x10, 0x00, 0x08 },
	{ NULL,				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
};

/* configuration */
char* parport_cable = NULL;
u16 parport_port;
static int parport_exit = 0;

/* interface variables
 */
static cable_t* cable;
static u8 dataport_value = 0x0;

#if PARPORT_USE_PPDEV == 1
static int device_handle;
#else
static unsigned long dataport;
static unsigned long statusport;
#endif

/* low level command set
 */
int parport_read(void);
void parport_write(int tck, int tms, int tdi);
void parport_reset(int trst, int srst);
void parport_led(int on);

int parport_speed(int speed);
int parport_register_commands(struct command_context_s *cmd_ctx);
int parport_init(void);
int parport_quit(void);

/* interface commands */
int parport_handle_parport_port_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int parport_handle_parport_cable_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int parport_handle_write_on_exit_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

jtag_interface_t parport_interface = 
{
	.name = "parport",
	
	.execute_queue = bitbang_execute_queue,

	.speed = parport_speed,	
	.register_commands = parport_register_commands,
	.init = parport_init,
	.quit = parport_quit,
};

bitbang_interface_t parport_bitbang =
{
	.read = parport_read,
	.write = parport_write,
	.reset = parport_reset,
	.blink = parport_led
};

int parport_read(void)
{
	int data = 0;
	
#if PARPORT_USE_PPDEV == 1
	ioctl(device_handle, PPRSTATUS, & data);
#else
	data = inb(statusport);
#endif

	if ((data ^ cable->INPUT_INVERT) & cable->TDO_MASK)
		return 1;
	else
		return 0;
}

static __inline__ void parport_write_data(void)
{
	u8 output;
	output = dataport_value ^ cable->OUTPUT_INVERT;

#if PARPORT_USE_PPDEV == 1
	ioctl(device_handle, PPWDATA, &output);
#else
#if defined(__FreeBSD__) || defined(__FreeBSD_kernel__)
	outb(dataport, output);
#else
	outb(output, dataport);
#endif
#endif
}

void parport_write(int tck, int tms, int tdi)
{
	int i = jtag_speed + 1;
	
	if (tck)
		dataport_value |= cable->TCK_MASK;
	else
		dataport_value &= ~cable->TCK_MASK;
	
	if (tms)
		dataport_value |= cable->TMS_MASK;
	else
		dataport_value &= ~cable->TMS_MASK;
	
	if (tdi)
		dataport_value |= cable->TDI_MASK;
	else
		dataport_value &= ~cable->TDI_MASK;
		
	while (i-- > 0)
		parport_write_data();
}

/* (1) assert or (0) deassert reset lines */
void parport_reset(int trst, int srst)
{
	LOG_DEBUG("trst: %i, srst: %i", trst, srst);

	if (trst == 0)
		dataport_value |= cable->TRST_MASK;
	else if (trst == 1)
		dataport_value &= ~cable->TRST_MASK;

	if (srst == 0)
		dataport_value |= cable->SRST_MASK;
	else if (srst == 1)
		dataport_value &= ~cable->SRST_MASK;
	
	parport_write_data();
}
	
/* turn LED on parport adapter on (1) or off (0) */
void parport_led(int on)
{
	if (on)
		dataport_value |= cable->LED_MASK;
	else
		dataport_value &= ~cable->LED_MASK;

	parport_write_data();
}

int parport_speed(int speed)
{
	return ERROR_OK;
}

int parport_register_commands(struct command_context_s *cmd_ctx)
{
	register_command(cmd_ctx, NULL, "parport_port", parport_handle_parport_port_command,
		COMMAND_CONFIG, "either the address of the I/O port or the number of the ‘/dev/parport’ device");
	register_command(cmd_ctx, NULL, "parport_cable", parport_handle_parport_cable_command,
		COMMAND_CONFIG, "the layout of the parallel port cable used to connect to the target");
	register_command(cmd_ctx, NULL, "parport_write_on_exit", parport_handle_write_on_exit_command,
		COMMAND_CONFIG, "configure the parallel driver to write a known value to the parallel interface");

	return ERROR_OK;
}

#if PARPORT_USE_GIVEIO == 1
int parport_get_giveio_access(void)
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

int parport_init(void)
{
	cable_t *cur_cable;
#if PARPORT_USE_PPDEV == 1
	char buffer[256];
	int i = 0;
#endif
	
	cur_cable = cables;
	
	if ((parport_cable == NULL) || (parport_cable[0] == 0))
	{
		parport_cable = "wiggler";
		LOG_WARNING("No parport cable specified, using default 'wiggler'");
	}
	
	while (cur_cable->name)
	{
		if (strcmp(cur_cable->name, parport_cable) == 0)
		{
			cable = cur_cable;
			break;
		}
		cur_cable++;
	}

	if (!cable)
	{
		LOG_ERROR("No matching cable found for %s", parport_cable);
		return ERROR_JTAG_INIT_FAILED;
	}
	
	dataport_value = cable->PORT_INIT;
	
#if PARPORT_USE_PPDEV == 1
	if (device_handle > 0)
	{
		LOG_ERROR("device is already opened");
		return ERROR_JTAG_INIT_FAILED;
	}

#if defined(__FreeBSD__) || defined(__FreeBSD_kernel__)
	LOG_DEBUG("opening /dev/ppi%d...", parport_port);

	snprintf(buffer, 256, "/dev/ppi%d", parport_port);
	device_handle = open(buffer, O_WRONLY);
#else /* not __FreeBSD__, __FreeBSD_kernel__ */
	LOG_DEBUG("opening /dev/parport%d...", parport_port);

	snprintf(buffer, 256, "/dev/parport%d", parport_port);
	device_handle = open(buffer, O_WRONLY);
#endif /* __FreeBSD__, __FreeBSD_kernel__ */

	if (device_handle < 0)
	{
		LOG_ERROR("cannot open device. check it exists and that user read and write rights are set");
		return ERROR_JTAG_INIT_FAILED;
	}

	LOG_DEBUG("...open");

#if !defined(__FreeBSD__) && !defined(__FreeBSD_kernel__)
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
#endif /* not __FreeBSD__, __FreeBSD_kernel__ */

#else /* not PARPORT_USE_PPDEV */
	if (parport_port == 0)
	{
		parport_port = 0x378;
		LOG_WARNING("No parport port specified, using default '0x378' (LPT1)");
	}
	
	dataport = parport_port;
	statusport = parport_port + 1;
	
	LOG_DEBUG("requesting privileges for parallel port 0x%lx...", dataport);
#if PARPORT_USE_GIVEIO == 1
	if (parport_get_giveio_access() != 0)
#else /* PARPORT_USE_GIVEIO */
	if (ioperm(dataport, 3, 1) != 0)
#endif /* PARPORT_USE_GIVEIO */
	{
		LOG_ERROR("missing privileges for direct i/o");
		return ERROR_JTAG_INIT_FAILED;
	}
	LOG_DEBUG("...privileges granted");
	
	/* make sure parallel port is in right mode (clear tristate and interrupt */
	#if defined(__FreeBSD__) || defined(__FreeBSD_kernel__)
		outb(parport_port + 2, 0x0);
	#else
		outb(0x0, parport_port + 2);
	#endif
	
#endif /* PARPORT_USE_PPDEV */
	
	parport_reset(0, 0);
	parport_write(0, 0, 0);
	parport_led(1);

	bitbang_interface = &parport_bitbang;	

	return ERROR_OK;
}

int parport_quit(void)
{
	parport_led(0);

	if (parport_exit)
	{
		dataport_value = cable->PORT_EXIT;
		parport_write_data();
	}
	
	if (parport_cable)
	{
		free(parport_cable);
		parport_cable = NULL;
	}
	
	return ERROR_OK;
}

int parport_handle_parport_port_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc == 0)
		return ERROR_OK;

	/* only if the port wasn't overwritten by cmdline */
	if (parport_port == 0)
		parport_port = strtoul(args[0], NULL, 0);

	return ERROR_OK;
}

int parport_handle_parport_cable_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc == 0)
		return ERROR_OK;

	/* only if the cable name wasn't overwritten by cmdline */
	if (parport_cable == 0)
	{
		parport_cable = malloc(strlen(args[0]) + sizeof(char));
		strcpy(parport_cable, args[0]);
	}

	return ERROR_OK;
}

int parport_handle_write_on_exit_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc != 1)
	{
		command_print(cmd_ctx, "usage: parport_write_on_exit <on|off>");
		return ERROR_OK;
	}
	
	if (strcmp(args[0], "on") == 0)
		parport_exit = 1;
	else if (strcmp(args[0], "off") == 0)
		parport_exit = 0;
	
	return ERROR_OK;
}
