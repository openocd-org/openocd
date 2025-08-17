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
#define ioperm(startport, length, enable) \
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

static const struct adapter_gpio_config *adapter_gpio_config;

#if PARPORT_USE_PPDEV == 0
static uint16_t parport_port;
#endif
static bool parport_write_exit_state;
static char *parport_device_file;
static uint32_t parport_toggling_time_ns = 1000;
static int wait_states;

// Interface variables.
static uint8_t dataport_value;

#if PARPORT_USE_PPDEV == 1
static int device_handle;
#else
static unsigned long dataport;
static unsigned long statusport;
#endif

// Bitmask map for the input pins.
static struct {
	uint8_t mask;
} input_pin_bitmask_map[] = {
	[10] = {0x40},
	[11] = {0x80},
	[12] = {0x20},
	[13] = {0x10},
	[15] = {0x08},
};

// Generate an output pin bitmask for an adapter signal.
#define OUTPUT_BITMASK(gpio_index) BIT((adapter_gpio_config[(gpio_index)].gpio_num - 2))

static enum bb_value parport_read(void)
{
	int data = 0;

#if PARPORT_USE_PPDEV == 1
	ioctl(device_handle, PPRSTATUS, &data);
#else
	data = inb(statusport);
#endif

	const struct adapter_gpio_config *gpio_config = &adapter_gpio_config[ADAPTER_GPIO_IDX_TDO];
	const bool tdo_state = data & input_pin_bitmask_map[gpio_config->gpio_num].mask;

	return (tdo_state ^ gpio_config->active_low) ? BB_HIGH : BB_LOW;
}

static void parport_write_data(void)
{
	const uint8_t output = dataport_value;

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

static bool is_gpio_configured(enum adapter_gpio_config_index gpio_index)
{
	return adapter_gpio_config[gpio_index].gpio_num != ADAPTER_GPIO_NOT_SET;
}

static void set_pin_state(enum adapter_gpio_config_index gpio_index,
	bool state)
{
	if (state ^ adapter_gpio_config[gpio_index].active_low)
		dataport_value |= OUTPUT_BITMASK(gpio_index);
	else
		dataport_value &= ~OUTPUT_BITMASK(gpio_index);
}

static int parport_write(int tck, int tms, int tdi)
{
	set_pin_state(ADAPTER_GPIO_IDX_TCK, tck == 1);
	set_pin_state(ADAPTER_GPIO_IDX_TMS, tms == 1);
	set_pin_state(ADAPTER_GPIO_IDX_TDI, tdi == 1);

	for (int i = 0; i < wait_states + 1; i++)
		parport_write_data();

	return ERROR_OK;
}

// (1) assert or (0) deassert reset lines.
static int parport_reset(int trst, int srst)
{
	LOG_DEBUG("trst: %i, srst: %i", trst, srst);

	if (is_gpio_configured(ADAPTER_GPIO_IDX_TRST))
		set_pin_state(ADAPTER_GPIO_IDX_TRST, trst == 0);

	if (is_gpio_configured(ADAPTER_GPIO_IDX_SRST))
		set_pin_state(ADAPTER_GPIO_IDX_SRST, srst == 0);

	parport_write_data();

	return ERROR_OK;
}

static int parport_led(bool on)
{
	if (!is_gpio_configured(ADAPTER_GPIO_IDX_LED))
		return ERROR_OK;

	set_pin_state(ADAPTER_GPIO_IDX_LED, on);
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
		LOG_ERROR("RCLK is not supported by the adapter");
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
static bool parport_get_giveio_access(void)
{
	OSVERSIONINFO version;

	version.dwOSVersionInfoSize = sizeof(version);
	if (!GetVersionEx(&version)) {
		errno = EINVAL;
		return false;
	}

	if (version.dwPlatformId != VER_PLATFORM_WIN32_NT)
		return true;

	HANDLE h = CreateFile("\\\\.\\giveio", GENERIC_READ, 0, NULL, OPEN_EXISTING,
		FILE_ATTRIBUTE_NORMAL, NULL);

	if (h == INVALID_HANDLE_VALUE) {
		errno = ENODEV;
		return false;
	}

	CloseHandle(h);

	return true;
}
#endif

static const struct bitbang_interface parport_bitbang = {
	.read = &parport_read,
	.write = &parport_write,
	.blink = &parport_led,
};

static const struct {
	enum adapter_gpio_config_index gpio_index;
	bool required;
} all_signals[] = {
	{ ADAPTER_GPIO_IDX_TDO, true },
	{ ADAPTER_GPIO_IDX_TDI, true },
	{ ADAPTER_GPIO_IDX_TMS, true },
	{ ADAPTER_GPIO_IDX_TCK, true },
	{ ADAPTER_GPIO_IDX_TRST, false },
	{ ADAPTER_GPIO_IDX_SRST, false },
	{ ADAPTER_GPIO_IDX_LED, false },
	{ ADAPTER_GPIO_IDX_USER0, false },
};

static int parport_init(void)
{
	adapter_gpio_config = adapter_gpio_get_config();

	// Check if all signals are configured properly.
	for (size_t i = 0; i < ARRAY_SIZE(all_signals); i++) {
		const enum adapter_gpio_config_index gpio_index = all_signals[i].gpio_index;
		const struct adapter_gpio_config gpio = adapter_gpio_config[gpio_index];

		if (gpio.gpio_num == ADAPTER_GPIO_NOT_SET) {
			if (all_signals[i].required) {
				LOG_ERROR("The signal '%s' is required and must be configured",
					adapter_gpio_get_name(gpio_index));
				return ERROR_FAIL;
			}

			continue;
		}

		if (gpio_index == ADAPTER_GPIO_IDX_TDO) {
			if (gpio.gpio_num < 10 || gpio.gpio_num > 15 || gpio.gpio_num == 14) {
				LOG_ERROR("The '%s' signal pin must be 10, 11, 12, 13, or 15",
					adapter_gpio_get_name(gpio_index));
				goto init_fail;
			}
		} else {
			if (gpio.gpio_num < 2 || gpio.gpio_num > 9) {
				LOG_ERROR("The '%s' signal pin must be 2, 3, 4, 5, 6, 7, 8, or 9",
					adapter_gpio_get_name(gpio_index));
				goto init_fail;
			}
		}
	}

	// Initialize signal pin states.
	for (size_t i = 0; i < ARRAY_SIZE(all_signals); i++) {
		const enum adapter_gpio_config_index gpio_index = all_signals[i].gpio_index;
		const struct adapter_gpio_config gpio = adapter_gpio_config[gpio_index];

		// The TDO (input) and LED (controlled by the adapter) pins cannot be
		// initialized.
		if (gpio_index == ADAPTER_GPIO_IDX_TDO || gpio_index == ADAPTER_GPIO_IDX_LED)
			continue;

		// Do not initialize unconfigured GPIO pins.
		if (gpio.gpio_num == ADAPTER_GPIO_NOT_SET)
			continue;

		if (gpio.init_state == ADAPTER_GPIO_INIT_STATE_ACTIVE)
			set_pin_state(gpio_index, true);
		else if (gpio.init_state == ADAPTER_GPIO_INIT_STATE_INACTIVE)
			set_pin_state(gpio_index, false);
	}

#if PARPORT_USE_PPDEV == 1
	if (device_handle > 0) {
		LOG_ERROR("Parallel port is already open");
		goto init_fail;
	}

	if (!parport_device_file) {
#if defined(__FreeBSD__) || defined(__FreeBSD_kernel__)
		parport_device_file = strdup("/dev/ppi0");
#else
		parport_device_file = strdup("/dev/parport0");
#endif
		LOG_WARNING("No parallel port specified, using %s", parport_device_file);
		LOG_WARNING("DEPRECATED! The lack of a parallel port specification is deprecated and will no longer work in the future");
	}

	LOG_DEBUG("Using parallel port %s", parport_device_file);

	device_handle = open(parport_device_file, O_WRONLY);

	if (device_handle < 0) {
		int err = errno;
		LOG_ERROR("Failed to open parallel port %s (errno = %d)",
			parport_device_file, err);
		LOG_ERROR("Check whether the device exists and if you have the required access rights");
		goto init_fail;
	}

#if !defined(__FreeBSD__) && !defined(__FreeBSD_kernel__)
	int retval = ioctl(device_handle, PPCLAIM);

	if (retval < 0) {
		LOG_ERROR("Failed to claim parallel port %s", parport_device_file);
		goto init_fail;
	}

	int value = PARPORT_MODE_COMPAT;
	retval = ioctl(device_handle, PPSETMODE, &value);

	if (retval < 0) {
		LOG_ERROR("Cannot set compatible mode to device");
		goto init_fail;
	}

	value = IEEE1284_MODE_COMPAT;
	retval = ioctl(device_handle, PPNEGOT, &value);

	if (retval < 0) {
		LOG_ERROR("Cannot set compatible 1284 mode to device");
		goto init_fail;
	}
#endif /* not __FreeBSD__, __FreeBSD_kernel__ */

#else /* not PARPORT_USE_PPDEV */
	LOG_WARNING("DEPRECATED: Parallel port access with direct I/O is deprecated and support will be removed in the next release");

	if (!parport_port) {
		parport_port = 0x378;
		LOG_WARNING("No parallel port specified, using default 0x378 (LPT1)");
	}

	LOG_DEBUG("Using parallel port 0x%x", parport_port);

	dataport = parport_port;
	statusport = parport_port + 1;

#if PARPORT_USE_GIVEIO == 1
	if (!parport_get_giveio_access()) {
#else /* PARPORT_USE_GIVEIO */
	if (ioperm(dataport, 3, 1) != 0) {
#endif /* PARPORT_USE_GIVEIO */
		LOG_ERROR("Missing privileges for direct I/O");
		goto init_fail;
	}

	// Make sure parallel port is in right mode (clear tristate and interrupt).
	#if defined(__FreeBSD__) || defined(__FreeBSD_kernel__)
		outb(parport_port + 2, 0x0);
	#else
		outb(0x0, parport_port + 2);
	#endif

#endif /* PARPORT_USE_PPDEV */

	if (parport_reset(0, 0) != ERROR_OK)
		goto init_fail;

	if (parport_write(0, 0, 0) != ERROR_OK)
		goto init_fail;

	if (parport_led(true) != ERROR_OK)
		goto init_fail;

	bitbang_interface = &parport_bitbang;

	return ERROR_OK;

init_fail:
	free(parport_device_file);
	return ERROR_JTAG_INIT_FAILED;
}

static int parport_quit(void)
{
	free(parport_device_file);

	// Deinitialize signal pin states.
	for (size_t i = 0; i < ARRAY_SIZE(all_signals); i++) {
		const enum adapter_gpio_config_index gpio_index = all_signals[i].gpio_index;
		const struct adapter_gpio_config gpio = adapter_gpio_config[gpio_index];

		// The TDO (input) and LED (controlled by the adapter) pins cannot be
		// deinitialized.
		if (gpio_index == ADAPTER_GPIO_IDX_TDO || gpio_index == ADAPTER_GPIO_IDX_LED)
			continue;

		// Do not deinitialize unconfigured GPIO pins.
		if (gpio.gpio_num == ADAPTER_GPIO_NOT_SET)
			continue;

		if (gpio.exit_state == ADAPTER_GPIO_EXIT_STATE_ACTIVE)
			set_pin_state(gpio_index, true);
		else if (gpio.exit_state == ADAPTER_GPIO_EXIT_STATE_INACTIVE)
			set_pin_state(gpio_index, false);
	}

	if (parport_write_exit_state)
		parport_write_data();

	if (parport_led(false) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

COMMAND_HANDLER(parport_handle_port_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

#if PARPORT_USE_PPDEV == 1
	if (parport_device_file) {
		LOG_ERROR("The parallel port device file is already configured");
		return ERROR_FAIL;
	}

	char *tmp;

	// We do not use the parse_xxx() or COMMAND_PARSE_xxx() functions here since
	// they generate an error message if parsing fails.
	char *endptr = NULL;
	unsigned long port_number = strtoul(CMD_ARGV[0], &endptr, 0);

	if (*endptr == '\0' && endptr != CMD_ARGV[0]) {
		LOG_WARNING("DEPRECATED! Using a port number is deprecated, use the device file instead");

#if defined(__FreeBSD__) || defined(__FreeBSD_kernel__)
		tmp = alloc_printf("/dev/ppi%lu", port_number);
#else
		tmp = alloc_printf("/dev/parport%lu", port_number);
#endif
	} else {
		tmp = strdup(CMD_ARGV[0]);
	}

	if (!tmp) {
		LOG_ERROR("Failed to allocate memory");
		return ERROR_FAIL;
	}

	free(parport_device_file);
	parport_device_file = tmp;
#else
	if (parport_port > 0) {
		command_print(CMD, "The parallel port is already configured");
		return ERROR_FAIL;
	}

	COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], parport_port);
#endif

	return ERROR_OK;
}

// This command is only for backward compatibility and will be removed in the
// future.
COMMAND_HANDLER(parport_handle_cable_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	return command_run_linef(CMD_CTX, "parport_select_cable %s", CMD_ARGV[0]);
}

COMMAND_HANDLER(parport_handle_write_on_exit_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	LOG_WARNING("DEPRECATED: 'parport write_on_exit' will be removed in the future, use the 'adapter gpio' command to configure the exit state for pins");

	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], parport_write_exit_state);

	return ERROR_OK;
}

COMMAND_HANDLER(parport_handle_toggling_time_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t toggling_time;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], toggling_time);

	if (!toggling_time) {
		command_print(CMD, "toggling time must not be 0 ns");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	parport_toggling_time_ns = toggling_time;
	int retval = adapter_get_speed(&wait_states);

	if (retval != ERROR_OK) {
		/*
		 * If adapter_get_speed fails then the clock_mode has not been
		 * configured, this happens if toggling_time is called before the
		 * adapter speed is set.
		 */
		LOG_INFO("No parallel port speed set, using zero wait states");
		wait_states = 0;
	}

	return ERROR_OK;
}

static const struct command_registration parport_subcommand_handlers[] = {
	{
		.name = "port",
		.handler = parport_handle_port_command,
		.mode = COMMAND_CONFIG,
		.help = "Specify the device file of the parallel port",
		.usage = "file",
	},
	{
		.name = "cable",
		.handler = parport_handle_cable_command,
		.mode = COMMAND_CONFIG,
		.help = "Set the layout of the parallel port cable "
			"used to connect to the target",
		.usage = "cable",
	},
	{
		.name = "write_on_exit",
		.handler = parport_handle_write_on_exit_command,
		.mode = COMMAND_CONFIG,
		.help = "Configure the driver to write a value to the parallel port on shutdown",
		.usage = "('on'|'off')",
	},
	{
		.name = "toggling_time",
		.handler = parport_handle_toggling_time_command,
		.mode = COMMAND_CONFIG,
		.help = "Configure how many nanoseconds it takes for the hardware to toggle TCK",
		.usage = "time",
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
	.transport_ids = TRANSPORT_JTAG,
	.transport_preferred_id = TRANSPORT_JTAG,
	.commands = parport_command_handlers,

	.init = parport_init,
	.quit = parport_quit,
	.reset = parport_reset,
	.speed = parport_speed,
	.khz = parport_khz,
	.speed_div = parport_speed_div,

	.jtag_ops = &parport_interface,
};
