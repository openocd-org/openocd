/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#if !BUILD_HTTPD
/* implementations of OpenOCD that uses multithreading needs to know when
 * OpenOCD is sleeping. No-op in vanilla OpenOCD
 */
void openocd_sleep_prelude(void)
{
}

void openocd_sleep_postlude(void)
{
}
#endif

/* This is the main entry for developer PC hosted OpenOCD.
 *
 * OpenOCD can also be used as a library that is linked with
 * another application(not mainstream yet, but possible), e.g.
 * w/as an embedded application.
 *
 * Those applications will have their own main() implementation
 * and use bits and pieces from openocd.c. */

extern int openocd_main(int argc, char *argv[]);

int main(int argc, char *argv[])
{
	return openocd_main(argc, argv);
}
