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

#endif /* OPENOCD_JTAG_ADAPTER_H */
