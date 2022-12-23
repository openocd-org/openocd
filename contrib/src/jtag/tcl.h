/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007-2010 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2009 SoftPLC Corporation                                *
 *       http://softplc.com                                                *
 *   dick@softplc.com                                                      *
 *                                                                         *
 *   Copyright (C) 2009 Zachary T Welch                                    *
 *   zw@superlucidity.net                                                  *
 ***************************************************************************/

#ifndef OPENOCD_JTAG_TCL_H
#define OPENOCD_JTAG_TCL_H

int jim_jtag_configure(Jim_Interp *interp, int argc,
		Jim_Obj * const *argv);
int jim_jtag_tap_enabler(Jim_Interp *interp, int argc,
		Jim_Obj * const *argv);

#endif /* OPENOCD_JTAG_TCL_H */
