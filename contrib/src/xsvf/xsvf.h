/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 ***************************************************************************/

#ifndef OPENOCD_XSVF_XSVF_H
#define OPENOCD_XSVF_XSVF_H

#include <helper/command.h>

int xsvf_register_commands(struct command_context *cmd_ctx);

#define ERROR_XSVF_EOF  (-200)
#define ERROR_XSVF_FAILED       (-201)

#endif /* OPENOCD_XSVF_XSVF_H */
