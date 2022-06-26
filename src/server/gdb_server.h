/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007-2009 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2011 by Broadcom Corporation                            *
 *   Evan Hunter - ehunter@broadcom.com                                    *
 ***************************************************************************/

#ifndef OPENOCD_SERVER_GDB_SERVER_H
#define OPENOCD_SERVER_GDB_SERVER_H

struct image;
struct reg;
#include <target/target.h>

#define GDB_BUFFER_SIZE 16384

int gdb_target_add_all(struct target *target);
int gdb_register_commands(struct command_context *command_context);
void gdb_service_free(void);

int gdb_put_packet(struct connection *connection, char *buffer, int len);

static inline struct target *get_target_from_connection(struct connection *connection)
{
	struct gdb_service *gdb_service = connection->service->priv;
	return gdb_service->target;
}

#define ERROR_GDB_BUFFER_TOO_SMALL (-800)
#define ERROR_GDB_TIMEOUT (-801)

#endif /* OPENOCD_SERVER_GDB_SERVER_H */
