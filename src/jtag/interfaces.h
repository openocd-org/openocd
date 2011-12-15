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
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef OPENOCD_JTAG_INTERFACES_H
#define OPENOCD_JTAG_INTERFACES_H

/** @file
 * Exports the list of JTAG interface drivers, along with routines
 * for loading and unloading them dynamically from shared libraries.
 */

#include <jtag/interface.h>

/// Dynamically load all JTAG interface modules from specified directory.
void jtag_interface_modules_load(const char *path);

extern struct jtag_interface *jtag_interfaces[];

/** @file
 * This file includes declarations for all built-in jtag interfaces,
 * which are then listed in the jtag_interfaces array.
 *
 * Dynamic loading can be implemented be searching for shared libraries
 * that contain a jtag_interface structure that can added to this list.
 */

#if BUILD_ZY1000 == 1
extern struct jtag_interface zy1000_interface;
#elif defined(BUILD_MINIDRIVER_DUMMY)
extern struct jtag_interface minidummy_interface;
#else /* standard drivers */
#if BUILD_PARPORT == 1
extern struct jtag_interface parport_interface;
#endif
#if BUILD_DUMMY == 1
extern struct jtag_interface dummy_interface;
#endif
#if BUILD_FT2232_FTD2XX == 1
extern struct jtag_interface ft2232_interface;
#endif
#if BUILD_FT2232_LIBFTDI == 1
extern struct jtag_interface ft2232_interface;
#endif
#if BUILD_USB_BLASTER_LIBFTDI == 1 || BUILD_USB_BLASTER_FTD2XX == 1
extern struct jtag_interface usb_blaster_interface;
#endif
#if BUILD_AMTJTAGACCEL == 1
extern struct jtag_interface amt_jtagaccel_interface;
#endif
#if BUILD_EP93XX == 1
extern struct jtag_interface ep93xx_interface;
#endif
#if BUILD_AT91RM9200 == 1
extern struct jtag_interface at91rm9200_interface;
#endif
#if BUILD_GW16012 == 1
extern struct jtag_interface gw16012_interface;
#endif
#if BUILD_PRESTO_LIBFTDI == 1 || BUILD_PRESTO_FTD2XX == 1
extern struct jtag_interface presto_interface;
#endif
#if BUILD_USBPROG == 1
extern struct jtag_interface usbprog_interface;
#endif
#if BUILD_JLINK == 1
extern struct jtag_interface jlink_interface;
#endif
#if BUILD_VSLLINK == 1
extern struct jtag_interface vsllink_interface;
#endif
#if BUILD_RLINK == 1
extern struct jtag_interface rlink_interface;
#endif
#if BUILD_ULINK == 1
extern struct jtag_interface ulink_interface;
#endif
#if BUILD_ARMJTAGEW == 1
extern struct jtag_interface armjtagew_interface;
#endif
#if BUILD_BUSPIRATE == 1
extern struct jtag_interface buspirate_interface;
#endif
#if BUILD_REMOTE_BITBANG == 1
extern struct jtag_interface remote_bitbang_interface;
#endif
#endif /* standard drivers */

#endif /* OPENOCD_JTAG_INTERFACES_H */
