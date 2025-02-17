// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/adapter.h>
#include <jtag/interface.h>
#include "bitbang.h"

/* -ino: 060521-1036 */
#if defined(__FreeBSD__) || defined(__FreeBSD_kernel__)
#include <machine/sysarch.h>
#include <machine/cpufunc.h>
#define ioperm(startport, length, enable)\
	i386_set_ioperm((startport), (length), (enable))
#endif /* __FreeBSD__ */

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
#include <sys/ioctl.h>
#else /* not PARPORT_USE_PPDEV */
#ifndef _WIN32
#include <sys/io.h>
#endif
#endif

#if PARPORT_USE_GIVEIO == 1 && IS_CYGWIN == 1
#include <windows.h>
#endif

// Parallel port cable description.
struct cable {
	const char *name;
	// Status port bit containing current TDO value.
	uint8_t tdo_mask;
	// Data port bit for TRST.
	uint8_t trst_mask;
	// Data port bit for TMD.
	uint8_t tms_mask;
	// Data port bit for TCK.
	uint8_t tck_mask;
	// Data port bit for TDI.
	uint8_t tdi_mask;
	// Data port bit for SRST.
	uint8_t srst_mask;
	// Data port bits that should be inverted.
	uint8_t output_invert;
	// Status port that should be inverted.
	uint8_t input_invert;
	// Initialize data port with this value.
	uint8_t port_init;
	// De-initialize data port with this value.
	uint8_t port_exit;
	// Data port bit for LED.
	uint8_t led_mask;
};

static const struct cable cables[] = {
	/* name				tdo   trst  tms   tck   tdi   srst  o_inv i_inv init  exit  led */
	{ "wiggler",			0x80, 0x10, 0x02, 0x04, 0x08, 0x01, 0x01, 0x80, 0x80, 0x80, 0x00 },
	{ "wiggler2",			0x80, 0x10, 0x02, 0x04, 0x08, 0x01, 0x01, 0x80, 0x80, 0x00, 0x20 },
	{ "wiggler_ntrst_inverted",	0x80, 0x10, 0x02, 0x04, 0x08, 0x01, 0x11, 0x80, 0x80, 0x80, 0x00 },
	{ "old_amt_wiggler",		0x80, 0x01, 0x02, 0x04, 0x08, 0x10, 0x11, 0x80, 0x80, 0x80, 0x00 },
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
	{ "aspo",                       0x10, 0x01, 0x04, 0x08, 0x02, 0x10, 0x17, 0x00, 0x17, 0x17, 0x00 },
	{ NULL,				0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 }
};

// Configuration variables.
static char *parport_cable;
static uint16_t parport_port;
static bool parport_exit;
static uint32_t parport_toggling_time_ns = 1000;
static int wait_states;

// Interface variables.
static const struct cable *cable;
static uint8_t dataport_value;

#if PARPORT_USE_PPDEV == 1
static int device_handle;
#else
static unsigned long dataport;
static unsigned long statusport;
#endif

static enum bb_value parport_read(void)
{
	int data = 0;

#if PARPORT_USE_PPDEV == 1
	ioctl(device_handle, PPRSTATUS, &data);
#else
	data = inb(statusport);
#endif

	if ((data ^ cable->input_invert) & cable->tdo_mask)
		return BB_HIGH;
	else
		return BB_LOW;
}

static inline void parport_write_data(void)
{
	uint8_t output;
	output = dataport_value ^ cable->output_invert;

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

static int parport_write(int tck, int tms, int tdi)
{
	int i = wait_states + 1;

	if (tck)
		dataport_value |= cable->tck_mask;
	else
		dataport_value &= ~cable->tck_mask;

	if (tms)
		dataport_value |= cable->tms_mask;
	else
		dataport_value &= ~cable->tms_mask;

	if (tdi)
		dataport_value |= cable->tdi_mask;
	else
		dataport_value &= ~cable->tdi_mask;

	while (i-- > 0)
		parport_write_data();

	return ERROR_OK;
}

// (1) assert or (0) deassert reset lines.
static int parport_reset(int trst, int srst)
{
	LOG_DEBUG("trst: %i, srst: %i", trst, srst);

	if (trst == 0)
		dataport_value |= cable->trst_mask;
	else if (trst == 1)
		dataport_value &= ~cable->trst_mask;

	if (srst == 0)
		dataport_value |= cable->srst_mask;
	else if (srst == 1)
		dataport_value &= ~cable->srst_mask;

	parport_write_data();

	return ERROR_OK;
}

static int parport_led(bool on)
{
	if (on)
		dataport_value |= cable->led_mask;
	else
		dataport_value &= ~cable->led_mask;

	parport_write_data();

	return ERROR_OK;
}

static int parport_speed(int speed)
{
	wait_states = speed;
	return ERROR_OK;
}

static int parport_khz(int khz, int *jtag_speed)
{
	if (!khz) {
		LOG_DEBUG("RCLK not supported");
		return ERROR_FAIL;
	}

	*jtag_speed = 499999 / (khz * parport_toggling_time_ns);
	return ERROR_OK;
}

static int parport_speed_div(int speed, int *khz)
{
	uint32_t denominator = (speed + 1) * parport_toggling_time_ns;

	*khz = (499999 + denominator) / denominator;
	return ERROR_OK;
}

#if PARPORT_USE_GIVEIO == 1
static int parport_get_giveio_access(void)
{
	HANDLE h;
	OSVERSIONINFO version;

	version.dwOSVersionInfoSize = sizeof(version);
	if (!GetVersionEx(&version)) {
		errno = EINVAL;
		return -1;
	}
	if (version.dwPlatformId != VER_PLATFORM_WIN32_NT)
		return 0;

	h = CreateFile("\\\\.\\giveio", GENERIC_READ, 0, NULL, OPEN_EXISTING, FILE_ATTRIBUTE_NORMAL, NULL);
	if (h == INVALID_HANDLE_VALUE) {
		errno = ENODEV;
		return -1;
	}

	CloseHandle(h);

	return 0;
}
#endif

static const struct bitbang_interface parport_bitbang = {
	.read = &parport_read,
	.write = &parport_write,
	.blink = &parport_led,
};

static int parport_init(void)
{
	const struct cable *cur_cable;
#if PARPORT_USE_PPDEV == 1
	char buffer[256];
#endif

	cur_cable = cables;

	if (!parport_cable) {
		parport_cable = strdup("wiggler");
		LOG_WARNING("No parport cable specified, using default 'wiggler'");
	}

	while (cur_cable->name) {
		if (!strcmp(cur_cable->name, parport_cable)) {
			cable = cur_cable;
			break;
		}
		cur_cable++;
	}

	if (!cable) {
		LOG_ERROR("No matching cable found for %s", parport_cable);
		return ERROR_JTAG_INIT_FAILED;
	}

	dataport_value = cable->port_init;

#if PARPORT_USE_PPDEV == 1
	if (device_handle > 0) {
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

	if (device_handle < 0) {
		int err = errno;
		LOG_ERROR("cannot open device. check it exists and that user read and write rights are set. errno=%d", err);
		return ERROR_JTAG_INIT_FAILED;
	}

	LOG_DEBUG("...open");

#if !defined(__FreeBSD__) && !defined(__FreeBSD_kernel__)
	int i = ioctl(device_handle, PPCLAIM);

	if (i < 0) {
		LOG_ERROR("cannot claim device");
		return ERROR_JTAG_INIT_FAILED;
	}

	i = PARPORT_MODE_COMPAT;
	i = ioctl(device_handle, PPSETMODE, &i);
	if (i < 0) {
		LOG_ERROR(" cannot set compatible mode to device");
		return ERROR_JTAG_INIT_FAILED;
	}

	i = IEEE1284_MODE_COMPAT;
	i = ioctl(device_handle, PPNEGOT, &i);
	if (i < 0) {
		LOG_ERROR("cannot set compatible 1284 mode to device");
		return ERROR_JTAG_INIT_FAILED;
	}
#endif /* not __FreeBSD__, __FreeBSD_kernel__ */

#else /* not PARPORT_USE_PPDEV */
	if (!parport_port) {
		parport_port = 0x378;
		LOG_WARNING("No parport port specified, using default '0x378' (LPT1)");
	}

	dataport = parport_port;
	statusport = parport_port + 1;

	LOG_DEBUG("requesting privileges for parallel port 0x%lx...", dataport);
#if PARPORT_USE_GIVEIO == 1
	if (parport_get_giveio_access() != 0) {
#else /* PARPORT_USE_GIVEIO */
	if (ioperm(dataport, 3, 1) != 0) {
#endif /* PARPORT_USE_GIVEIO */
		LOG_ERROR("missing privileges for direct i/o");
		return ERROR_JTAG_INIT_FAILED;
	}
	LOG_DEBUG("...privileges granted");

	// Make sure parallel port is in right mode (clear tristate and interrupt.
	#if defined(__FreeBSD__) || defined(__FreeBSD_kernel__)
		outb(parport_port + 2, 0x0);
	#else
		outb(0x0, parport_port + 2);
	#endif

#endif /* PARPORT_USE_PPDEV */

	if (parport_reset(0, 0) != ERROR_OK)
		return ERROR_FAIL;
	if (parport_write(0, 0, 0) != ERROR_OK)
		return ERROR_FAIL;
	if (parport_led(true) != ERROR_OK)
		return ERROR_FAIL;

	bitbang_interface = &parport_bitbang;

	return ERROR_OK;
}

static int parport_quit(void)
{
	if (parport_led(false) != ERROR_OK)
		return ERROR_FAIL;

	if (parport_exit) {
		dataport_value = cable->port_exit;
		parport_write_data();
	}

	free(parport_cable);
	parport_cable = NULL;

	return ERROR_OK;
}

COMMAND_HANDLER(parport_handle_port_command)
{
	if (CMD_ARGC == 1) {
		// Only if the port wasn't overwritten by cmdline.
		if (!parport_port) {
			COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], parport_port);
		} else {
			LOG_ERROR("The parport port was already configured!");
			return ERROR_FAIL;
		}
	}

	command_print(CMD, "parport port = 0x%" PRIx16 "", parport_port);

	return ERROR_OK;
}

COMMAND_HANDLER(parport_handle_cable_command)
{
	if (!CMD_ARGC)
		return ERROR_OK;

	// Only if the cable name wasn't overwritten by cmdline.
	if (!parport_cable) {
		// TODO: REVISIT first verify that it's listed in cables[].
		parport_cable = malloc(strlen(CMD_ARGV[0]) + sizeof(char));
		if (!parport_cable) {
			LOG_ERROR("Out of memory");
			return ERROR_FAIL;
		}
		strcpy(parport_cable, CMD_ARGV[0]);
	}

	//  TODO: REVISIT it's probably worth returning the current value.

	return ERROR_OK;
}

COMMAND_HANDLER(parport_handle_write_on_exit_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], parport_exit);

	return ERROR_OK;
}

COMMAND_HANDLER(parport_handle_toggling_time_command)
{
	if (CMD_ARGC == 1) {
		uint32_t ns;
		int retval = parse_u32(CMD_ARGV[0], &ns);

		if (retval != ERROR_OK)
			return retval;

		if (!ns) {
			LOG_ERROR("0 ns is not a valid parport toggling time");
			return ERROR_FAIL;
		}

		parport_toggling_time_ns = ns;
		retval = adapter_get_speed(&wait_states);
		if (retval != ERROR_OK) {
			/*
			 * If adapter_get_speed fails then the clock_mode has
			 * not been configured, this happens if toggling_time is
			 * called before the adapter speed is set.
			 */
			LOG_INFO("no parport speed set - defaulting to zero wait states");
			wait_states = 0;
		}
	}

	command_print(CMD, "parport toggling time = %" PRIu32 " ns",
			parport_toggling_time_ns);

	return ERROR_OK;
}

static const struct command_registration parport_subcommand_handlers[] = {
	{
		.name = "port",
		.handler = parport_handle_port_command,
		.mode = COMMAND_CONFIG,
		.help = "Display the address of the I/O port (e.g. 0x378) "
			"or the number of the '/dev/parport' device used.  "
			"If a parameter is provided, first change that port.",
		.usage = "[port_number]",
	},
	{
		.name = "cable",
		.handler = parport_handle_cable_command,
		.mode = COMMAND_CONFIG,
		.help = "Set the layout of the parallel port cable "
			"used to connect to the target.",
		// TODO: REVISIT there's no way to list layouts we know.
		.usage = "[layout]",
	},
	{
		.name = "write_on_exit",
		.handler = parport_handle_write_on_exit_command,
		.mode = COMMAND_CONFIG,
		.help = "Configure the parallel driver to write "
			"a known value to the parallel interface on exit.",
		.usage = "('on'|'off')",
	},
	{
		.name = "toggling_time",
		.handler = parport_handle_toggling_time_command,
		.mode = COMMAND_CONFIG,
		.help = "Displays or assigns how many nanoseconds it "
			"takes for the hardware to toggle TCK.",
		.usage = "[nanoseconds]",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration parport_command_handlers[] = {
	{
		.name = "parport",
		.mode = COMMAND_ANY,
		.help = "perform parport management",
		.chain = parport_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static struct jtag_interface parport_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = bitbang_execute_queue,
};

struct adapter_driver parport_adapter_driver = {
	.name = "parport",
	.transports = jtag_only,
	.commands = parport_command_handlers,

	.init = parport_init,
	.quit = parport_quit,
	.reset = parport_reset,
	.speed = parport_speed,
	.khz = parport_khz,
	.speed_div = parport_speed_div,

	.jtag_ops = &parport_interface,
};
