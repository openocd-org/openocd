/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath <Dominic.Rath@gmx.de>              *
 *   Copyright (C) 2007-2009 Øyvind Harboe <oyvind.harboe@zylin.com>       *
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
 ***************************************************************************/

#ifndef OPENOCD_JTAG_DRIVERS_MINIDRIVER_IMP_H
#define OPENOCD_JTAG_DRIVERS_MINIDRIVER_IMP_H

#include <jtag/commands.h>

static inline void interface_jtag_add_scan_check_alloc(struct scan_field *field)
{
	unsigned num_bytes = DIV_ROUND_UP(field->num_bits, 8);
	field->in_value = cmd_queue_alloc(num_bytes);
}

void interface_jtag_add_callback(jtag_callback1_t f, jtag_callback_data_t data0);

void interface_jtag_add_callback4(jtag_callback_t f, jtag_callback_data_t data0,
				  jtag_callback_data_t data1, jtag_callback_data_t data2,
				  jtag_callback_data_t data3);

void jtag_add_callback4(jtag_callback_t f, jtag_callback_data_t data0,
			jtag_callback_data_t data1, jtag_callback_data_t data2,
			jtag_callback_data_t data3);

#endif /* OPENOCD_JTAG_DRIVERS_MINIDRIVER_IMP_H */
