/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath <Dominic.Rath@gmx.de>              *
 *   Copyright (C) 2009 by Zachary T Welch <zw@superlucidity.net>          *
 ***************************************************************************/

#ifndef OPENOCD_OPENOCD_H
#define OPENOCD_OPENOCD_H

/**
 * Different applications can define this entry point to override
 * the default openocd main function.  On most systems, this will be
 * defined in src/openocd.c.
 * @param argc normally passed from main()
 * @param argv normally passed from main()
 * @returns return code for main()
 */
int openocd_main(int argc, char *argv[]);

#endif /* OPENOCD_OPENOCD_H */
