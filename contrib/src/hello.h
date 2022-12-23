/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
 ***************************************************************************/

#ifndef OPENOCD_HELLO_H
#define OPENOCD_HELLO_H

struct command_registration;

/**
 * Export the registration for the hello command group, so it can be
 * embedded in example drivers.
 */
extern const struct command_registration hello_command_handlers[];

#endif /* OPENOCD_HELLO_H */
