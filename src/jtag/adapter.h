/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (C) 2005 by Dominic Rath <Dominic.Rath@gmx.de>
 * Copyright (c) 2018 Pengutronix, Oleksij Rempel <kernel@pengutronix.de>
 */

#ifndef OPENOCD_JTAG_ADAPTER_H
#define OPENOCD_JTAG_ADAPTER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** Supported output drive modes for adaptor GPIO */
enum adapter_gpio_drive_mode {
	ADAPTER_GPIO_DRIVE_MODE_PUSH_PULL,
	ADAPTER_GPIO_DRIVE_MODE_OPEN_DRAIN,
	ADAPTER_GPIO_DRIVE_MODE_OPEN_SOURCE,
};

/** Supported GPIO directions */
enum adapter_gpio_direction {
	ADAPTER_GPIO_DIRECTION_INPUT,
	ADAPTER_GPIO_DIRECTION_OUTPUT,
	ADAPTER_GPIO_DIRECTION_BIDIRECTIONAL,
};

/** Supported initial states for GPIO */
enum adapter_gpio_init_state {
	ADAPTER_GPIO_INIT_STATE_INACTIVE, /* Should be zero so it is the default state */
	ADAPTER_GPIO_INIT_STATE_ACTIVE,
	ADAPTER_GPIO_INIT_STATE_INPUT,
};

/** Supported pull directions for GPIO */
enum adapter_gpio_pull {
	ADAPTER_GPIO_PULL_NONE,
	ADAPTER_GPIO_PULL_UP,
	ADAPTER_GPIO_PULL_DOWN,
};

/** Adapter GPIO */
enum adapter_gpio_config_index {
	ADAPTER_GPIO_IDX_TDO,
	ADAPTER_GPIO_IDX_TDI,
	ADAPTER_GPIO_IDX_TMS,
	ADAPTER_GPIO_IDX_TCK,
	ADAPTER_GPIO_IDX_TRST,
	ADAPTER_GPIO_IDX_SWDIO,
	ADAPTER_GPIO_IDX_SWDIO_DIR,
	ADAPTER_GPIO_IDX_SWCLK,
	ADAPTER_GPIO_IDX_SRST,
	ADAPTER_GPIO_IDX_LED,
	ADAPTER_GPIO_IDX_NUM, /* must be the last item */
};

/** Configuration options for a single GPIO */
struct adapter_gpio_config {
	int gpio_num;
	int chip_num;
	enum adapter_gpio_drive_mode drive; /* For outputs only */
	enum adapter_gpio_init_state init_state;
	bool active_low;
	enum adapter_gpio_pull pull;
};

struct command_context;

/** Register the adapter's commands */
int adapter_register_commands(struct command_context *ctx);

/** Initialize debug adapter upon startup.  */
int adapter_init(struct command_context *cmd_ctx);

/** Shutdown the debug adapter upon program exit. */
int adapter_quit(void);

/** @returns true if adapter has been initialized */
bool is_adapter_initialized(void);

/** @returns USB location string set with command 'adapter usb location' */
const char *adapter_usb_get_location(void);

/** @returns true if USB location string is "<dev_bus>-<port_path[0]>[.<port_path[1]>[...]]" */
bool adapter_usb_location_equal(uint8_t dev_bus, uint8_t *port_path, size_t path_len);

/** @returns The current adapter speed setting. */
int adapter_get_speed(int *speed);

/**
 * Given a @a speed setting, use the interface @c speed_div callback to
 * adjust the setting.
 * @param speed The speed setting to convert back to readable kHz.
 * @returns ERROR_OK if the interface has not been initialized or on success;
 *  otherwise, the error code produced by the @c speed_div callback.
 */
int adapter_get_speed_readable(int *speed);

/** Attempt to configure the adapter for the specified kHz. */
int adapter_config_khz(unsigned int khz);

/**
 * Attempt to enable RTCK/RCLK. If that fails, fallback to the
 * specified frequency.
 */
int adapter_config_rclk(unsigned int fallback_speed_khz);

/** Retrieves the clock speed of the adapter in kHz. */
unsigned int adapter_get_speed_khz(void);

/** Retrieves the serial number set with command 'adapter serial' */
const char *adapter_get_required_serial(void);

/**
 * Retrieves gpio name
 */
const char *adapter_gpio_get_name(enum adapter_gpio_config_index idx);

/**
 * Retrieves gpio configuration set with command 'adapter gpio <signal_name>'
 */
const struct adapter_gpio_config *adapter_gpio_get_config(void);

#endif /* OPENOCD_JTAG_ADAPTER_H */
