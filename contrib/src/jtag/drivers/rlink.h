/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2008 Lou Deluxe                                         *
 *   lou.openocd012@fixit.nospammail.net                                   *
 ***************************************************************************/

#ifndef OPENOCD_JTAG_DRIVERS_RLINK_H
#define OPENOCD_JTAG_DRIVERS_RLINK_H

#include "helper/types.h"
struct rlink_speed_table {
	uint8_t const *dtc;
	uint16_t dtc_size;
	uint16_t khz;
	uint8_t prescaler;
};

extern const struct rlink_speed_table rlink_speed_table[];
extern const size_t rlink_speed_table_size;

#endif /* OPENOCD_JTAG_DRIVERS_RLINK_H */
