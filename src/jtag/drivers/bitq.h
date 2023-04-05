/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2007 by Pavel Chromy                                    *
 *   chromy@asix.cz                                                        *
 ***************************************************************************/

#ifndef OPENOCD_JTAG_DRIVERS_BITQ_H
#define OPENOCD_JTAG_DRIVERS_BITQ_H

#include <jtag/commands.h>

struct bitq_interface {
	/* function to enqueueing low level IO requests */
	int (*out)(int tms, int tdi, int tdo_req);
	int (*flush)(void);

	int (*sleep)(unsigned long us);
	int (*reset)(int trst, int srst);

	/* delayed read of requested TDO data,
	 * the input shall be checked after call to any enqueuing function
	 */
	int (*in_rdy)(void);
	int (*in)(void);
};

extern struct bitq_interface *bitq_interface;

int bitq_execute_queue(void);

void bitq_cleanup(void);

#endif /* OPENOCD_JTAG_DRIVERS_BITQ_H */
