/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2013 by Andes Technology                                *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
 ***************************************************************************/

#ifndef OPENOCD_JTAG_AICE_AICE_INTERFACE_H
#define OPENOCD_JTAG_AICE_AICE_INTERFACE_H

int aice_init_targets(void);
int aice_scan_jtag_chain(void);

#endif /* OPENOCD_JTAG_AICE_AICE_INTERFACE_H */
