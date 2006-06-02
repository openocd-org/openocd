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
#ifndef GDB_SERVER_H
#define GDB_SERVER_H

#include "target.h"
#include "server.h"

#define GDB_BUFFER_SIZE	2048

typedef struct gdb_connection_s
{
	char buffer[GDB_BUFFER_SIZE];
	char *buf_p;
	int buf_cnt;
	int ctrl_c;
	enum target_state frontend_state;
} gdb_connection_t;

typedef struct gdb_service_s
{
	struct target_s *target;
} gdb_service_t;

extern int gdb_init();
extern int gdb_register_commands(command_context_t *command_context);

#define ERROR_GDB_BUFFER_TOO_SMALL (-800)

#endif /* GDB_SERVER_H */
