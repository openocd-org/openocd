// SPDX-License-Identifier: GPL-2.0-or-later
/*
 * Copyright (C) 2005 by Dominic Rath <Dominic.Rath@gmx.de>
 * Copyright (C) 2007-2010 Øyvind Harboe <oyvind.harboe@zylin.com>
 * Copyright (C) 2009 SoftPLC Corporation, http://softplc.com, Dick Hollenbeck <dick@softplc.com>
 * Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>
 * Copyright (C) 2018 Pengutronix, Oleksij Rempel <kernel@pengutronix.de>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "adapter.h"
#include "jtag.h"
#include "minidriver.h"
#include "interface.h"
#include "interfaces.h"
#include <transport/transport.h>

#ifdef HAVE_STRINGS_H
#include <strings.h>
#endif

/**
 * @file
 * Holds support for configuring debug adapters from TCl scripts.
 */

struct adapter_driver *adapter_driver;
const char * const jtag_only[] = { "jtag", NULL };

enum adapter_clk_mode {
	CLOCK_MODE_UNSELECTED = 0,
	CLOCK_MODE_KHZ,
	CLOCK_MODE_RCLK
};

#define DEFAULT_CLOCK_SPEED_KHZ		100U

/**
 * Adapter configuration
 */
static struct {
	bool adapter_initialized;
	char *usb_location;
	char *serial;
	enum adapter_clk_mode clock_mode;
	int speed_khz;
	int rclk_fallback_speed_khz;
	struct adapter_gpio_config gpios[ADAPTER_GPIO_IDX_NUM];
	bool gpios_initialized; /* Initialization of GPIOs to their unset values performed at run time */
} adapter_config;

static const struct gpio_map {
	const char *name;
	enum adapter_gpio_direction direction;
	bool permit_drive_option;
	bool permit_init_state_option;
} gpio_map[ADAPTER_GPIO_IDX_NUM] = {
	[ADAPTER_GPIO_IDX_TDO] = { "tdo", ADAPTER_GPIO_DIRECTION_INPUT, false, true, },
	[ADAPTER_GPIO_IDX_TDI] = { "tdi", ADAPTER_GPIO_DIRECTION_OUTPUT, true, true, },
	[ADAPTER_GPIO_IDX_TMS] = { "tms", ADAPTER_GPIO_DIRECTION_OUTPUT, true, true, },
	[ADAPTER_GPIO_IDX_TCK] = { "tck", ADAPTER_GPIO_DIRECTION_OUTPUT, true, true, },
	[ADAPTER_GPIO_IDX_SWDIO] = { "swdio", ADAPTER_GPIO_DIRECTION_BIDIRECTIONAL, true, true, },
	[ADAPTER_GPIO_IDX_SWDIO_DIR] = { "swdio_dir", ADAPTER_GPIO_DIRECTION_OUTPUT, true, false, },
	[ADAPTER_GPIO_IDX_SWCLK] = { "swclk", ADAPTER_GPIO_DIRECTION_OUTPUT, true, true, },
	[ADAPTER_GPIO_IDX_TRST] = { "trst", ADAPTER_GPIO_DIRECTION_OUTPUT, false, true, },
	[ADAPTER_GPIO_IDX_SRST] = { "srst", ADAPTER_GPIO_DIRECTION_OUTPUT, false, true, },
	[ADAPTER_GPIO_IDX_LED] = { "led", ADAPTER_GPIO_DIRECTION_OUTPUT, true, true, },
};

bool is_adapter_initialized(void)
{
	return adapter_config.adapter_initialized;
}

/* For convenience of the bit-banging drivers keep the gpio_config drive
 * settings for srst and trst in sync with values set by the "adapter
 * reset_config" command.
 */
static void sync_adapter_reset_with_gpios(void)
{
	enum reset_types cfg = jtag_get_reset_config();
	if (cfg & RESET_SRST_PUSH_PULL)
		adapter_config.gpios[ADAPTER_GPIO_IDX_SRST].drive = ADAPTER_GPIO_DRIVE_MODE_PUSH_PULL;
	else
		adapter_config.gpios[ADAPTER_GPIO_IDX_SRST].drive = ADAPTER_GPIO_DRIVE_MODE_OPEN_DRAIN;
	if (cfg & RESET_TRST_OPEN_DRAIN)
		adapter_config.gpios[ADAPTER_GPIO_IDX_TRST].drive = ADAPTER_GPIO_DRIVE_MODE_OPEN_DRAIN;
	else
		adapter_config.gpios[ADAPTER_GPIO_IDX_TRST].drive = ADAPTER_GPIO_DRIVE_MODE_PUSH_PULL;
}

static void adapter_driver_gpios_init(void)
{
	if (adapter_config.gpios_initialized)
		return;

	for (int i = 0; i < ADAPTER_GPIO_IDX_NUM; ++i) {
		adapter_config.gpios[i].gpio_num = -1;
		adapter_config.gpios[i].chip_num = -1;
		if (gpio_map[i].direction == ADAPTER_GPIO_DIRECTION_INPUT)
			adapter_config.gpios[i].init_state = ADAPTER_GPIO_INIT_STATE_INPUT;
	}

	/* Drivers assume active low, and this is the normal behaviour for reset
	 * lines so should be the default. */
	adapter_config.gpios[ADAPTER_GPIO_IDX_SRST].active_low = true;
	adapter_config.gpios[ADAPTER_GPIO_IDX_TRST].active_low = true;
	sync_adapter_reset_with_gpios();

	/* JTAG GPIOs should be inactive except for tms */
	adapter_config.gpios[ADAPTER_GPIO_IDX_TMS].init_state = ADAPTER_GPIO_INIT_STATE_ACTIVE;

	adapter_config.gpios_initialized = true;
}

/**
 * Do low-level setup like initializing registers, output signals,
 * and clocking.
 */
int adapter_init(struct command_context *cmd_ctx)
{
	if (is_adapter_initialized())
		return ERROR_OK;

	if (!adapter_driver) {
		/* nothing was previously specified by "adapter driver" command */
		LOG_ERROR("Debug Adapter has to be specified, "
			"see \"adapter driver\" command");
		return ERROR_JTAG_INVALID_INTERFACE;
	}

	adapter_driver_gpios_init();

	int retval;

	if (adapter_config.clock_mode == CLOCK_MODE_UNSELECTED) {
		LOG_WARNING("An adapter speed is not selected in the init scripts."
			" OpenOCD will try to run the adapter at the low speed (%d kHz)",
			DEFAULT_CLOCK_SPEED_KHZ);
		LOG_WARNING("To remove this warnings and achieve reasonable communication speed with the target,"
		    " set \"adapter speed\" or \"jtag_rclk\" in the init scripts.");
		retval = adapter_config_khz(DEFAULT_CLOCK_SPEED_KHZ);
		if (retval != ERROR_OK)
			return ERROR_JTAG_INIT_FAILED;
	}

	retval = adapter_driver->init();
	if (retval != ERROR_OK)
		return retval;
	adapter_config.adapter_initialized = true;

	if (!adapter_driver->speed) {
		LOG_INFO("This adapter doesn't support configurable speed");
		return ERROR_OK;
	}

	int requested_khz = adapter_get_speed_khz();
	int actual_khz = requested_khz;
	int speed_var = 0;
	retval = adapter_get_speed(&speed_var);
	if (retval != ERROR_OK)
		return retval;
	retval = adapter_driver->speed(speed_var);
	if (retval != ERROR_OK)
		return retval;
	retval = adapter_get_speed_readable(&actual_khz);
	if (retval != ERROR_OK)
		LOG_INFO("adapter-specific clock speed value %d", speed_var);
	else if (actual_khz) {
		/* Adaptive clocking -- JTAG-specific */
		if ((adapter_config.clock_mode == CLOCK_MODE_RCLK)
				|| ((adapter_config.clock_mode == CLOCK_MODE_KHZ) && !requested_khz)) {
			LOG_INFO("RCLK (adaptive clock speed) not supported - fallback to %d kHz"
			, actual_khz);
		} else
			LOG_INFO("clock speed %d kHz", actual_khz);
	} else
		LOG_INFO("RCLK (adaptive clock speed)");

	return ERROR_OK;
}

int adapter_quit(void)
{
	if (is_adapter_initialized() && adapter_driver->quit) {
		/* close the JTAG interface */
		int result = adapter_driver->quit();
		if (result != ERROR_OK)
			LOG_ERROR("failed: %d", result);
	}

	free(adapter_config.serial);
	free(adapter_config.usb_location);

	struct jtag_tap *t = jtag_all_taps();
	while (t) {
		struct jtag_tap *n = t->next_tap;
		jtag_tap_free(t);
		t = n;
	}

	return ERROR_OK;
}

unsigned int adapter_get_speed_khz(void)
{
	return adapter_config.speed_khz;
}

static int adapter_khz_to_speed(unsigned int khz, int *speed)
{
	LOG_DEBUG("convert khz to adapter specific speed value");
	adapter_config.speed_khz = khz;
	if (!is_adapter_initialized())
		return ERROR_OK;
	LOG_DEBUG("have adapter set up");
	if (!adapter_driver->khz) {
		LOG_ERROR("Translation from khz to adapter speed not implemented");
		return ERROR_FAIL;
	}
	int speed_div1;
	int retval = adapter_driver->khz(adapter_get_speed_khz(), &speed_div1);
	if (retval != ERROR_OK)
		return retval;
	*speed = speed_div1;
	return ERROR_OK;
}

static int adapter_rclk_to_speed(unsigned int fallback_speed_khz, int *speed)
{
	int retval = adapter_khz_to_speed(0, speed);
	if ((retval != ERROR_OK) && fallback_speed_khz) {
		LOG_DEBUG("trying fallback speed...");
		retval = adapter_khz_to_speed(fallback_speed_khz, speed);
	}
	return retval;
}

static int adapter_set_speed(int speed)
{
	/* this command can be called during CONFIG,
	 * in which case adapter isn't initialized */
	return is_adapter_initialized() ? adapter_driver->speed(speed) : ERROR_OK;
}

int adapter_config_khz(unsigned int khz)
{
	LOG_DEBUG("handle adapter khz");
	adapter_config.clock_mode = CLOCK_MODE_KHZ;
	int speed = 0;
	int retval = adapter_khz_to_speed(khz, &speed);
	return (retval != ERROR_OK) ? retval : adapter_set_speed(speed);
}

int adapter_config_rclk(unsigned int fallback_speed_khz)
{
	LOG_DEBUG("handle adapter rclk");
	adapter_config.clock_mode = CLOCK_MODE_RCLK;
	adapter_config.rclk_fallback_speed_khz = fallback_speed_khz;
	int speed = 0;
	int retval = adapter_rclk_to_speed(fallback_speed_khz, &speed);
	return (retval != ERROR_OK) ? retval : adapter_set_speed(speed);
}

int adapter_get_speed(int *speed)
{
	switch (adapter_config.clock_mode) {
		case CLOCK_MODE_KHZ:
			adapter_khz_to_speed(adapter_get_speed_khz(), speed);
			break;
		case CLOCK_MODE_RCLK:
			adapter_rclk_to_speed(adapter_config.rclk_fallback_speed_khz, speed);
			break;
		default:
			LOG_ERROR("BUG: unknown adapter clock mode");
			return ERROR_FAIL;
	}
	return ERROR_OK;
}

int adapter_get_speed_readable(int *khz)
{
	int speed_var = 0;
	int retval = adapter_get_speed(&speed_var);
	if (retval != ERROR_OK)
		return retval;
	if (!is_adapter_initialized())
		return ERROR_OK;
	if (!adapter_driver->speed_div) {
		LOG_ERROR("Translation from adapter speed to khz not implemented");
		return ERROR_FAIL;
	}
	return adapter_driver->speed_div(speed_var, khz);
}

const char *adapter_get_required_serial(void)
{
	return adapter_config.serial;
}

/*
 * 1 char: bus
 * 2 * 7 chars: max 7 ports
 * 1 char: test for overflow
 * ------
 * 16 chars
 */
#define USB_MAX_LOCATION_LENGTH         16

#ifdef HAVE_LIBUSB_GET_PORT_NUMBERS
static void adapter_usb_set_location(const char *location)
{
	if (strnlen(location, USB_MAX_LOCATION_LENGTH) == USB_MAX_LOCATION_LENGTH)
		LOG_WARNING("usb location string is too long!!");

	free(adapter_config.usb_location);

	adapter_config.usb_location = strndup(location, USB_MAX_LOCATION_LENGTH);
}
#endif /* HAVE_LIBUSB_GET_PORT_NUMBERS */

const char *adapter_usb_get_location(void)
{
	return adapter_config.usb_location;
}

bool adapter_usb_location_equal(uint8_t dev_bus, uint8_t *port_path, size_t path_len)
{
	size_t path_step, string_length;
	char *ptr, *loc;
	bool equal = false;

	if (!adapter_usb_get_location())
		return equal;

	/* strtok need non const char */
	loc = strndup(adapter_usb_get_location(), USB_MAX_LOCATION_LENGTH);
	string_length = strnlen(loc, USB_MAX_LOCATION_LENGTH);

	ptr = strtok(loc, "-");
	if (!ptr) {
		LOG_WARNING("no '-' in usb path\n");
		goto done;
	}

	string_length -= strnlen(ptr, string_length);
	/* check bus mismatch */
	if (atoi(ptr) != dev_bus)
		goto done;

	path_step = 0;
	while (path_step < path_len) {
		ptr = strtok(NULL, ".");

		/* no more tokens in path */
		if (!ptr)
			break;

		/* path mismatch at some step */
		if (path_step < path_len && atoi(ptr) != port_path[path_step])
			break;

		path_step++;
		string_length -= strnlen(ptr, string_length) + 1;
	};

	/* walked the full path, all elements match */
	if (path_step == path_len && !string_length)
		equal = true;

done:
	free(loc);
	return equal;
}

static int jim_adapter_name(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	struct jim_getopt_info goi;
	jim_getopt_setup(&goi, interp, argc-1, argv + 1);

	/* return the name of the interface */
	/* TCL code might need to know the exact type... */
	/* FUTURE: we allow this as a means to "set" the interface. */
	if (goi.argc != 0) {
		Jim_WrongNumArgs(goi.interp, 1, goi.argv-1, "(no params)");
		return JIM_ERR;
	}
	const char *name = adapter_driver ? adapter_driver->name : NULL;
	Jim_SetResultString(goi.interp, name ? name : "undefined", -1);
	return JIM_OK;
}

COMMAND_HANDLER(adapter_transports_command)
{
	char **transports;
	int retval;

	retval = CALL_COMMAND_HANDLER(transport_list_parse, &transports);
	if (retval != ERROR_OK)
		return retval;

	retval = allow_transports(CMD_CTX, (const char **)transports);

	if (retval != ERROR_OK) {
		for (unsigned i = 0; transports[i]; i++)
			free(transports[i]);
		free(transports);
	}
	return retval;
}

COMMAND_HANDLER(handle_adapter_list_command)
{
	if (strcmp(CMD_NAME, "list") == 0 && CMD_ARGC > 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	command_print(CMD, "The following debug adapters are available:");
	for (unsigned i = 0; adapter_drivers[i]; i++) {
		const char *name = adapter_drivers[i]->name;
		command_print(CMD, "%u: %s", i + 1, name);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_adapter_driver_command)
{
	int retval;

	/* check whether the interface is already configured */
	if (adapter_driver) {
		LOG_WARNING("Interface already configured, ignoring");
		return ERROR_OK;
	}

	/* interface name is a mandatory argument */
	if (CMD_ARGC != 1 || CMD_ARGV[0][0] == '\0')
		return ERROR_COMMAND_SYNTAX_ERROR;

	for (unsigned i = 0; adapter_drivers[i]; i++) {
		if (strcmp(CMD_ARGV[0], adapter_drivers[i]->name) != 0)
			continue;

		if (adapter_drivers[i]->commands) {
			retval = register_commands(CMD_CTX, NULL, adapter_drivers[i]->commands);
			if (retval != ERROR_OK)
				return retval;
		}

		adapter_driver = adapter_drivers[i];

		return allow_transports(CMD_CTX, adapter_driver->transports);
	}

	/* no valid interface was found (i.e. the configuration option,
	 * didn't match one of the compiled-in interfaces
	 */
	LOG_ERROR("The specified debug interface was not found (%s)",
				CMD_ARGV[0]);
	CALL_COMMAND_HANDLER(handle_adapter_list_command);
	return ERROR_JTAG_INVALID_INTERFACE;
}

COMMAND_HANDLER(handle_reset_config_command)
{
	int new_cfg = 0;
	int mask = 0;

	/* Original versions cared about the order of these tokens:
	 *   reset_config signals [combination [trst_type [srst_type]]]
	 * They also clobbered the previous configuration even on error.
	 *
	 * Here we don't care about the order, and only change values
	 * which have been explicitly specified.
	 */
	for (; CMD_ARGC; CMD_ARGC--, CMD_ARGV++) {
		int tmp = 0;
		int m;

		/* gating */
		m = RESET_SRST_NO_GATING;
		if (strcmp(*CMD_ARGV, "srst_gates_jtag") == 0)
			/* default: don't use JTAG while SRST asserted */;
		else if (strcmp(*CMD_ARGV, "srst_nogate") == 0)
			tmp = RESET_SRST_NO_GATING;
		else
			m = 0;
		if (mask & m) {
			LOG_ERROR("extra reset_config %s spec (%s)",
					"gating", *CMD_ARGV);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		if (m)
			goto next;

		/* signals */
		m = RESET_HAS_TRST | RESET_HAS_SRST;
		if (strcmp(*CMD_ARGV, "none") == 0)
			tmp = RESET_NONE;
		else if (strcmp(*CMD_ARGV, "trst_only") == 0)
			tmp = RESET_HAS_TRST;
		else if (strcmp(*CMD_ARGV, "srst_only") == 0)
			tmp = RESET_HAS_SRST;
		else if (strcmp(*CMD_ARGV, "trst_and_srst") == 0)
			tmp = RESET_HAS_TRST | RESET_HAS_SRST;
		else
			m = 0;
		if (mask & m) {
			LOG_ERROR("extra reset_config %s spec (%s)",
					"signal", *CMD_ARGV);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		if (m)
			goto next;

		/* combination (options for broken wiring) */
		m = RESET_SRST_PULLS_TRST | RESET_TRST_PULLS_SRST;
		if (strcmp(*CMD_ARGV, "separate") == 0)
			/* separate reset lines - default */;
		else if (strcmp(*CMD_ARGV, "srst_pulls_trst") == 0)
			tmp |= RESET_SRST_PULLS_TRST;
		else if (strcmp(*CMD_ARGV, "trst_pulls_srst") == 0)
			tmp |= RESET_TRST_PULLS_SRST;
		else if (strcmp(*CMD_ARGV, "combined") == 0)
			tmp |= RESET_SRST_PULLS_TRST | RESET_TRST_PULLS_SRST;
		else
			m = 0;
		if (mask & m) {
			LOG_ERROR("extra reset_config %s spec (%s)",
					"combination", *CMD_ARGV);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		if (m)
			goto next;

		/* trst_type (NOP without HAS_TRST) */
		m = RESET_TRST_OPEN_DRAIN;
		if (strcmp(*CMD_ARGV, "trst_open_drain") == 0)
			tmp |= RESET_TRST_OPEN_DRAIN;
		else if (strcmp(*CMD_ARGV, "trst_push_pull") == 0)
			/* push/pull from adapter - default */;
		else
			m = 0;
		if (mask & m) {
			LOG_ERROR("extra reset_config %s spec (%s)",
					"trst_type", *CMD_ARGV);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		if (m)
			goto next;

		/* srst_type (NOP without HAS_SRST) */
		m = RESET_SRST_PUSH_PULL;
		if (strcmp(*CMD_ARGV, "srst_push_pull") == 0)
			tmp |= RESET_SRST_PUSH_PULL;
		else if (strcmp(*CMD_ARGV, "srst_open_drain") == 0)
			/* open drain from adapter - default */;
		else
			m = 0;
		if (mask & m) {
			LOG_ERROR("extra reset_config %s spec (%s)",
					"srst_type", *CMD_ARGV);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		if (m)
			goto next;

		/* connect_type - only valid when srst_nogate */
		m = RESET_CNCT_UNDER_SRST;
		if (strcmp(*CMD_ARGV, "connect_assert_srst") == 0)
			tmp |= RESET_CNCT_UNDER_SRST;
		else if (strcmp(*CMD_ARGV, "connect_deassert_srst") == 0)
			/* connect normally - default */;
		else
			m = 0;
		if (mask & m) {
			LOG_ERROR("extra reset_config %s spec (%s)",
					"connect_type", *CMD_ARGV);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		if (m)
			goto next;

		/* caller provided nonsense; fail */
		LOG_ERROR("unknown reset_config flag (%s)", *CMD_ARGV);
		return ERROR_COMMAND_SYNTAX_ERROR;

next:
		/* Remember the bits which were specified (mask)
		 * and their new values (new_cfg).
		 */
		mask |= m;
		new_cfg |= tmp;
	}

	/* clear previous values of those bits, save new values */
	if (mask) {
		int old_cfg = jtag_get_reset_config();

		old_cfg &= ~mask;
		new_cfg |= old_cfg;
		jtag_set_reset_config(new_cfg);
		sync_adapter_reset_with_gpios();

	} else
		new_cfg = jtag_get_reset_config();

	/*
	 * Display the (now-)current reset mode
	 */
	char *modes[6];

	/* minimal JTAG has neither SRST nor TRST (so that's the default) */
	switch (new_cfg & (RESET_HAS_TRST | RESET_HAS_SRST)) {
		case RESET_HAS_SRST:
			modes[0] = "srst_only";
			break;
		case RESET_HAS_TRST:
			modes[0] = "trst_only";
			break;
		case RESET_TRST_AND_SRST:
			modes[0] = "trst_and_srst";
			break;
		default:
			modes[0] = "none";
			break;
	}

	/* normally SRST and TRST are decoupled; but bugs happen ... */
	switch (new_cfg & (RESET_SRST_PULLS_TRST | RESET_TRST_PULLS_SRST)) {
		case RESET_SRST_PULLS_TRST:
			modes[1] = "srst_pulls_trst";
			break;
		case RESET_TRST_PULLS_SRST:
			modes[1] = "trst_pulls_srst";
			break;
		case RESET_SRST_PULLS_TRST | RESET_TRST_PULLS_SRST:
			modes[1] = "combined";
			break;
		default:
			modes[1] = "separate";
			break;
	}

	/* TRST-less connectors include Altera, Xilinx, and minimal JTAG */
	if (new_cfg & RESET_HAS_TRST) {
		if (new_cfg & RESET_TRST_OPEN_DRAIN)
			modes[3] = " trst_open_drain";
		else
			modes[3] = " trst_push_pull";
	} else
		modes[3] = "";

	/* SRST-less connectors include TI-14, Xilinx, and minimal JTAG */
	if (new_cfg & RESET_HAS_SRST) {
		if (new_cfg & RESET_SRST_NO_GATING)
			modes[2] = " srst_nogate";
		else
			modes[2] = " srst_gates_jtag";

		if (new_cfg & RESET_SRST_PUSH_PULL)
			modes[4] = " srst_push_pull";
		else
			modes[4] = " srst_open_drain";

		if (new_cfg & RESET_CNCT_UNDER_SRST)
			modes[5] = " connect_assert_srst";
		else
			modes[5] = " connect_deassert_srst";
	} else {
		modes[2] = "";
		modes[4] = "";
		modes[5] = "";
	}

	command_print(CMD, "%s %s%s%s%s%s",
			modes[0], modes[1],
			modes[2], modes[3], modes[4], modes[5]);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_adapter_srst_delay_command)
{
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (CMD_ARGC == 1) {
		unsigned delay;
		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], delay);

		jtag_set_nsrst_delay(delay);
	}
	command_print(CMD, "adapter srst delay: %u", jtag_get_nsrst_delay());
	return ERROR_OK;
}

COMMAND_HANDLER(handle_adapter_srst_pulse_width_command)
{
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (CMD_ARGC == 1) {
		unsigned width;
		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], width);

		jtag_set_nsrst_assert_width(width);
	}
	command_print(CMD, "adapter srst pulse_width: %u", jtag_get_nsrst_assert_width());
	return ERROR_OK;
}

COMMAND_HANDLER(handle_adapter_speed_command)
{
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	int retval = ERROR_OK;
	if (CMD_ARGC == 1) {
		unsigned khz = 0;
		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], khz);

		retval = adapter_config_khz(khz);
		if (retval != ERROR_OK)
			return retval;
	}

	int cur_speed = adapter_get_speed_khz();
	retval = adapter_get_speed_readable(&cur_speed);
	if (retval != ERROR_OK)
		return retval;

	if (cur_speed)
		command_print(CMD, "adapter speed: %d kHz", cur_speed);
	else
		command_print(CMD, "adapter speed: RCLK - adaptive");

	return retval;
}

COMMAND_HANDLER(handle_adapter_serial_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	free(adapter_config.serial);
	adapter_config.serial = strdup(CMD_ARGV[0]);
	return ERROR_OK;
}

COMMAND_HANDLER(handle_adapter_reset_de_assert)
{
	enum values {
		VALUE_UNDEFINED = -1,
		VALUE_DEASSERT  = 0,
		VALUE_ASSERT    = 1,
	};
	enum values value;
	enum values srst = VALUE_UNDEFINED;
	enum values trst = VALUE_UNDEFINED;
	enum reset_types jtag_reset_config = jtag_get_reset_config();
	char *signal;

	if (CMD_ARGC == 0) {
		if (transport_is_jtag()) {
			if (jtag_reset_config & RESET_HAS_TRST)
				signal = jtag_get_trst() ? "asserted" : "deasserted";
			else
				signal = "not present";
			command_print(CMD, "trst %s", signal);
		}

		if (jtag_reset_config & RESET_HAS_SRST)
			signal = jtag_get_srst() ? "asserted" : "deasserted";
		else
			signal = "not present";
		command_print(CMD, "srst %s", signal);

		return ERROR_OK;
	}

	if (CMD_ARGC != 1 && CMD_ARGC != 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	value = (strcmp(CMD_NAME, "assert") == 0) ? VALUE_ASSERT : VALUE_DEASSERT;
	if (strcmp(CMD_ARGV[0], "srst") == 0)
		srst = value;
	else if (strcmp(CMD_ARGV[0], "trst") == 0)
		trst = value;
	else
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC == 3) {
		if (strcmp(CMD_ARGV[1], "assert") == 0)
			value = VALUE_ASSERT;
		else if (strcmp(CMD_ARGV[1], "deassert") == 0)
			value = VALUE_DEASSERT;
		else
			return ERROR_COMMAND_SYNTAX_ERROR;

		if (strcmp(CMD_ARGV[2], "srst") == 0 && srst == VALUE_UNDEFINED)
			srst = value;
		else if (strcmp(CMD_ARGV[2], "trst") == 0 && trst == VALUE_UNDEFINED)
			trst = value;
		else
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (trst == VALUE_UNDEFINED) {
		if (transport_is_jtag())
			trst = jtag_get_trst() ? VALUE_ASSERT : VALUE_DEASSERT;
		else
			trst = VALUE_DEASSERT; /* unused, safe value */
	}

	if (srst == VALUE_UNDEFINED) {
		if (jtag_reset_config & RESET_HAS_SRST)
			srst = jtag_get_srst() ? VALUE_ASSERT : VALUE_DEASSERT;
		else
			srst = VALUE_DEASSERT; /* unused, safe value */
	}

	if (trst == VALUE_ASSERT && !transport_is_jtag()) {
		LOG_ERROR("transport has no trst signal");
		return ERROR_FAIL;
	}

	if (srst == VALUE_ASSERT && !(jtag_reset_config & RESET_HAS_SRST)) {
		LOG_ERROR("adapter has no srst signal");
		return ERROR_FAIL;
	}

	return adapter_resets((trst == VALUE_DEASSERT) ? TRST_DEASSERT : TRST_ASSERT,
						  (srst == VALUE_DEASSERT) ? SRST_DEASSERT : SRST_ASSERT);
}

static int get_gpio_index(const char *signal_name)
{
	for (int i = 0; i < ADAPTER_GPIO_IDX_NUM; ++i) {
		if (strcmp(gpio_map[i].name, signal_name) == 0)
			return i;
	}
	return -1;
}

static COMMAND_HELPER(helper_adapter_gpio_print_config, enum adapter_gpio_config_index gpio_idx)
{
	struct adapter_gpio_config *gpio_config = &adapter_config.gpios[gpio_idx];
	const char *active_state = gpio_config->active_low ? "low" : "high";
	const char *dir = "";
	const char *drive = "";
	const char *pull = "";
	const char *init_state = "";

	switch (gpio_map[gpio_idx].direction) {
	case ADAPTER_GPIO_DIRECTION_INPUT:
		dir = "input";
		break;
	case ADAPTER_GPIO_DIRECTION_OUTPUT:
		dir = "output";
		break;
	case ADAPTER_GPIO_DIRECTION_BIDIRECTIONAL:
		dir = "bidirectional";
		break;
	}

	if (gpio_map[gpio_idx].permit_drive_option) {
		switch (gpio_config->drive) {
		case ADAPTER_GPIO_DRIVE_MODE_PUSH_PULL:
			drive = ", push-pull";
			break;
		case ADAPTER_GPIO_DRIVE_MODE_OPEN_DRAIN:
			drive = ", open-drain";
			break;
		case ADAPTER_GPIO_DRIVE_MODE_OPEN_SOURCE:
			drive = ", open-source";
			break;
		}
	}

	switch (gpio_config->pull) {
	case ADAPTER_GPIO_PULL_NONE:
		pull = ", pull-none";
		break;
	case ADAPTER_GPIO_PULL_UP:
		pull = ", pull-up";
		break;
	case ADAPTER_GPIO_PULL_DOWN:
		pull = ", pull-down";
		break;
	}

	if (gpio_map[gpio_idx].permit_init_state_option) {
		switch (gpio_config->init_state) {
		case ADAPTER_GPIO_INIT_STATE_INACTIVE:
			init_state = ", init-state inactive";
			break;
		case ADAPTER_GPIO_INIT_STATE_ACTIVE:
			init_state = ", init-state active";
			break;
		case ADAPTER_GPIO_INIT_STATE_INPUT:
			init_state = ", init-state input";
			break;
		}
	}

	command_print(CMD, "adapter gpio %s (%s): num %d, chip %d, active-%s%s%s%s",
		gpio_map[gpio_idx].name, dir, gpio_config->gpio_num, gpio_config->chip_num, active_state,
		drive, pull, init_state);

	return ERROR_OK;
}

COMMAND_HANDLER(helper_adapter_gpio_print_all_configs)
{
	for (int i = 0; i < ADAPTER_GPIO_IDX_NUM; ++i)
		CALL_COMMAND_HANDLER(helper_adapter_gpio_print_config, i);
	return ERROR_OK;
}

COMMAND_HANDLER(adapter_gpio_config_handler)
{
	unsigned int i = 1;
	struct adapter_gpio_config *gpio_config;

	adapter_driver_gpios_init();

	if (CMD_ARGC == 0) {
		CALL_COMMAND_HANDLER(helper_adapter_gpio_print_all_configs);
		return ERROR_OK;
	}

	int gpio_idx = get_gpio_index(CMD_ARGV[0]);
	if (gpio_idx == -1) {
		LOG_ERROR("adapter has no gpio named %s", CMD_ARGV[0]);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (CMD_ARGC == 1) {
		CALL_COMMAND_HANDLER(helper_adapter_gpio_print_config, gpio_idx);
		return ERROR_OK;
	}

	gpio_config = &adapter_config.gpios[gpio_idx];
	while (i < CMD_ARGC) {
		LOG_DEBUG("Processing %s", CMD_ARGV[i]);

		if (isdigit(*CMD_ARGV[i])) {
			int gpio_num; /* Use a meaningful output parameter for more helpful error messages */
			COMMAND_PARSE_NUMBER(int, CMD_ARGV[i], gpio_num);
			gpio_config->gpio_num = gpio_num;
			++i;
			continue;
		}

		if (strcmp(CMD_ARGV[i], "-chip") == 0) {
			if (CMD_ARGC - i < 2) {
				LOG_ERROR("-chip option requires a parameter");
				return ERROR_FAIL;
			}
			LOG_DEBUG("-chip arg is %s", CMD_ARGV[i + 1]);
			int chip_num; /* Use a meaningful output parameter for more helpful error messages */
			COMMAND_PARSE_NUMBER(int, CMD_ARGV[i + 1], chip_num);
			gpio_config->chip_num = chip_num;
			i += 2;
			continue;
		}

		if (strcmp(CMD_ARGV[i], "-active-high") == 0) {
			++i;
			gpio_config->active_low = false;
			continue;
		}
		if (strcmp(CMD_ARGV[i], "-active-low") == 0) {
			++i;
			gpio_config->active_low = true;
			continue;
		}

		if (gpio_map[gpio_idx].permit_drive_option) {
			if (strcmp(CMD_ARGV[i], "-push-pull") == 0) {
				++i;
				gpio_config->drive = ADAPTER_GPIO_DRIVE_MODE_PUSH_PULL;
				continue;
			}
			if (strcmp(CMD_ARGV[i], "-open-drain") == 0) {
				++i;
				gpio_config->drive = ADAPTER_GPIO_DRIVE_MODE_OPEN_DRAIN;
				continue;
			}
			if (strcmp(CMD_ARGV[i], "-open-source") == 0) {
				++i;
				gpio_config->drive = ADAPTER_GPIO_DRIVE_MODE_OPEN_SOURCE;
				continue;
			}
		}

		if (strcmp(CMD_ARGV[i], "-pull-none") == 0) {
			++i;
			gpio_config->pull = ADAPTER_GPIO_PULL_NONE;
			continue;
		}
		if (strcmp(CMD_ARGV[i], "-pull-up") == 0) {
			++i;
			gpio_config->pull = ADAPTER_GPIO_PULL_UP;
			continue;
		}
		if (strcmp(CMD_ARGV[i], "-pull-down") == 0) {
			++i;
			gpio_config->pull = ADAPTER_GPIO_PULL_DOWN;
			continue;
		}

		if (gpio_map[gpio_idx].permit_init_state_option) {
			if (strcmp(CMD_ARGV[i], "-init-inactive") == 0) {
				++i;
				gpio_config->init_state = ADAPTER_GPIO_INIT_STATE_INACTIVE;
				continue;
			}
			if (strcmp(CMD_ARGV[i], "-init-active") == 0) {
				++i;
				gpio_config->init_state = ADAPTER_GPIO_INIT_STATE_ACTIVE;
				continue;
			}

			if (gpio_map[gpio_idx].direction == ADAPTER_GPIO_DIRECTION_BIDIRECTIONAL &&
					strcmp(CMD_ARGV[i], "-init-input") == 0) {
				++i;
				gpio_config->init_state = ADAPTER_GPIO_INIT_STATE_INPUT;
				continue;
			}
		}

		LOG_ERROR("illegal option for adapter %s %s: %s",
				CMD_NAME, gpio_map[gpio_idx].name, CMD_ARGV[i]);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* Force swdio_dir init state to be compatible with swdio init state */
	if (gpio_idx == ADAPTER_GPIO_IDX_SWDIO)
		adapter_config.gpios[ADAPTER_GPIO_IDX_SWDIO_DIR].init_state =
		(gpio_config->init_state == ADAPTER_GPIO_INIT_STATE_INPUT) ?
		ADAPTER_GPIO_INIT_STATE_INACTIVE :
		ADAPTER_GPIO_INIT_STATE_ACTIVE;

	return ERROR_OK;
}

#ifdef HAVE_LIBUSB_GET_PORT_NUMBERS
COMMAND_HANDLER(handle_usb_location_command)
{
	if (CMD_ARGC == 1)
		adapter_usb_set_location(CMD_ARGV[0]);

	command_print(CMD, "adapter usb location: %s", adapter_usb_get_location());

	return ERROR_OK;
}
#endif /* HAVE_LIBUSB_GET_PORT_NUMBERS */

static const struct command_registration adapter_usb_command_handlers[] = {
#ifdef HAVE_LIBUSB_GET_PORT_NUMBERS
	{
		.name = "location",
		.handler = &handle_usb_location_command,
		.mode = COMMAND_CONFIG,
		.help = "display or set the USB bus location of the USB device",
		.usage = "[<bus>-port[.port]...]",
	},
#endif /* HAVE_LIBUSB_GET_PORT_NUMBERS */
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration adapter_srst_command_handlers[] = {
	{
		.name = "delay",
		.handler = handle_adapter_srst_delay_command,
		.mode = COMMAND_ANY,
		.help = "delay after deasserting SRST in ms",
		.usage = "[milliseconds]",
	},
	{
		.name = "pulse_width",
		.handler = handle_adapter_srst_pulse_width_command,
		.mode = COMMAND_ANY,
		.help = "SRST assertion pulse width in ms",
		.usage = "[milliseconds]",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration adapter_command_handlers[] = {
	{
		.name = "driver",
		.handler = handle_adapter_driver_command,
		.mode = COMMAND_CONFIG,
		.help = "Select a debug adapter driver",
		.usage = "driver_name",
	},
	{
		.name = "speed",
		.handler = handle_adapter_speed_command,
		.mode = COMMAND_ANY,
		.help = "With an argument, change to the specified maximum "
			"jtag speed.  For JTAG, 0 KHz signifies adaptive "
			"clocking. "
			"With or without argument, display current setting.",
		.usage = "[khz]",
	},
	{
		.name = "serial",
		.handler = handle_adapter_serial_command,
		.mode = COMMAND_CONFIG,
		.help = "Set the serial number of the adapter",
		.usage = "serial_string",
	},
	{
		.name = "list",
		.handler = handle_adapter_list_command,
		.mode = COMMAND_ANY,
		.help = "List all built-in debug adapter drivers",
		.usage = "",
	},
	{
		.name = "name",
		.mode = COMMAND_ANY,
		.jim_handler = jim_adapter_name,
		.help = "Returns the name of the currently "
			"selected adapter (driver)",
	},
	{
		.name = "srst",
		.mode = COMMAND_ANY,
		.help = "srst adapter command group",
		.usage = "",
		.chain = adapter_srst_command_handlers,
	},
	{
		.name = "transports",
		.handler = adapter_transports_command,
		.mode = COMMAND_CONFIG,
		.help = "Declare transports the adapter supports.",
		.usage = "transport ...",
	},
	{
		.name = "usb",
		.mode = COMMAND_ANY,
		.help = "usb adapter command group",
		.usage = "",
		.chain = adapter_usb_command_handlers,
	},
	{
		.name = "assert",
		.handler = handle_adapter_reset_de_assert,
		.mode = COMMAND_EXEC,
		.help = "Controls SRST and TRST lines.",
		.usage = "|deassert [srst|trst [assert|deassert srst|trst]]",
	},
	{
		.name = "deassert",
		.handler = handle_adapter_reset_de_assert,
		.mode = COMMAND_EXEC,
		.help = "Controls SRST and TRST lines.",
		.usage = "|assert [srst|trst [deassert|assert srst|trst]]",
	},
	{
		.name = "gpio",
		.handler = adapter_gpio_config_handler,
		.mode = COMMAND_CONFIG,
		.help = "gpio adapter command group",
		.usage = "[ tdo|tdi|tms|tck|trst|swdio|swdio_dir|swclk|srst|led"
			"[gpio_number] "
			"[-chip chip_number] "
			"[-active-high|-active-low] "
			"[-push-pull|-open-drain|-open-source] "
			"[-pull-none|-pull-up|-pull-down]"
			"[-init-inactive|-init-active|-init-input] ]",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration interface_command_handlers[] = {
	{
		.name = "adapter",
		.mode = COMMAND_ANY,
		.help = "adapter command group",
		.usage = "",
		.chain = adapter_command_handlers,
	},
	{
		.name = "reset_config",
		.handler = handle_reset_config_command,
		.mode = COMMAND_ANY,
		.help = "configure adapter reset behavior",
		.usage = "[none|trst_only|srst_only|trst_and_srst] "
			"[srst_pulls_trst|trst_pulls_srst|combined|separate] "
			"[srst_gates_jtag|srst_nogate] "
			"[trst_push_pull|trst_open_drain] "
			"[srst_push_pull|srst_open_drain] "
			"[connect_deassert_srst|connect_assert_srst]",
	},
	COMMAND_REGISTRATION_DONE
};

/**
 * Register the commands which deal with arbitrary debug adapter drivers.
 *
 * @todo Remove internal assumptions that all debug adapters use JTAG for
 * transport.  Various types and data structures are not named generically.
 */
int adapter_register_commands(struct command_context *ctx)
{
	return register_commands(ctx, NULL, interface_command_handlers);
}

const char *adapter_gpio_get_name(enum adapter_gpio_config_index idx)
{
	return gpio_map[idx].name;
}

/* Allow drivers access to the GPIO configuration */
const struct adapter_gpio_config *adapter_gpio_get_config(void)
{
	return adapter_config.gpios;
}
