/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2008                                                    *
 ***************************************************************************/

#ifndef OPENOCD_SERVER_TCL_SERVER_H
#define OPENOCD_SERVER_TCL_SERVER_H

#include <server/server.h>

int tcl_init(void);
int tcl_register_commands(struct command_context *cmd_ctx);
void tcl_service_free(void);
bool tcl_is_from_tcl_session(struct command_context *ctx);

#endif /* OPENOCD_SERVER_TCL_SERVER_H */
