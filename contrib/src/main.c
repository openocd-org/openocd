// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include "openocd.h"
#include "helper/system.h"

/* This is the main entry for developer PC hosted OpenOCD.
 *
 * OpenOCD can also be used as a library that is linked with
 * another application(not mainstream yet, but possible), e.g.
 * w/as an embedded application.
 *
 * Those applications will have their own main() implementation
 * and use bits and pieces from openocd.c. */

int main(int argc, char *argv[])
{
	/* disable buffering otherwise piping to logs causes problems work */
	setvbuf(stdout, NULL, _IONBF, 0);
	setvbuf(stderr, NULL, _IONBF, 0);

	return openocd_main(argc, argv);
}
