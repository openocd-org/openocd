/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2009 SoftPLC Corporation                                *
 *       http://softplc.com                                                *
 *   dick@softplc.com                                                      *
 *                                                                         *
 *   Copyright (C) 2009 Zachary T Welch                                    *
 *   zw@superlucidity.net                                                  *
 ***************************************************************************/

#ifndef OPENOCD_JTAG_INTERFACES_H
#define OPENOCD_JTAG_INTERFACES_H

/** @file
 * Exports the list of JTAG interface drivers, along with routines
 * for loading and unloading them dynamically from shared libraries.
 */

#include <jtag/interface.h>

extern struct adapter_driver *adapter_drivers[];

#endif /* OPENOCD_JTAG_INTERFACES_H */
