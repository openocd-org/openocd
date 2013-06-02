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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifndef GDB_SERVER_H
#define GDB_SERVER_H

struct image;
struct reg;
#include <target/target.h>

#define GDB_BUFFER_SIZE 16384

int gdb_target_add_all(struct target *target);
int gdb_register_commands(struct command_context *command_context);

int gdb_put_packet(struct connection *connection, char *buffer, int len);

static inline struct target *get_target_from_connection(struct connection *connection)
{
	struct gdb_service *gdb_service = connection->service->priv;
	return gdb_service->target;
}

#define ERROR_GDB_BUFFER_TOO_SMALL (-800)
#define ERROR_GDB_TIMEOUT (-801)

#endif	/* GDB_SERVER_H */
