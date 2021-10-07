/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * Copyright (c) 2018 Pengutronix, Oleksij Rempel <kernel@pengutronix.de>
 */

#ifndef OPENOCD_JTAG_ADAPTER_H
#define OPENOCD_JTAG_ADAPTER_H

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

/** @returns true if adapter has been initialized */
bool is_adapter_initialized(void);

/** @returns USB location string set with command 'adapter usb location' */
const char *adapter_usb_get_location(void);

/** @returns true if USB location string is "<dev_bus>-<port_path[0]>[.<port_path[1]>[...]]" */
bool adapter_usb_location_equal(uint8_t dev_bus, uint8_t *port_path, size_t path_len);

#endif /* OPENOCD_JTAG_ADAPTER_H */
