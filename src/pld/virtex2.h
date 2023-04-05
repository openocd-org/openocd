/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 ***************************************************************************/

#ifndef OPENOCD_PLD_VIRTEX2_H
#define OPENOCD_PLD_VIRTEX2_H

#include <jtag/jtag.h>

struct virtex2_pld_device {
	struct jtag_tap *tap;
	int no_jstart;
};

#endif /* OPENOCD_PLD_VIRTEX2_H */
