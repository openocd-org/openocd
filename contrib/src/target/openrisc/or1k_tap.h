/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2012 by Franck Jullien                                  *
 *   elec4fun@gmail.com                                                    *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_OPENRISC_OR1K_TAP_H
#define OPENOCD_TARGET_OPENRISC_OR1K_TAP_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/list.h>
#include "or1k.h"

int or1k_tap_vjtag_register(void);
int or1k_tap_xilinx_bscan_register(void);
int or1k_tap_mohor_register(void);

/* Linear list over all available or1k taps */
extern struct list_head tap_list;

struct or1k_tap_ip {
	struct list_head list;
	int (*init)(struct or1k_jtag *jtag_info);
	const char *name;
};

#endif /* OPENOCD_TARGET_OPENRISC_OR1K_TAP_H */
