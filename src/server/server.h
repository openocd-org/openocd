/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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
#ifndef SERVER_H
#define SERVER_H

#include "command.h"
#include "binarybuffer.h"
#include "replacements.h"

#include <sys/types.h>

enum connection_type
{
	CONNECTION_TCP,
	CONNECTION_PIPE
};

typedef struct connection_s
{
	int fd;
	struct sockaddr_in sin;
	command_context_t *cmd_ctx;
	struct service_s *service;
	int input_pending;
	void *priv;
	struct connection_s *next;
} connection_t;

typedef int (*new_connection_handler_t)(connection_t *connection);
typedef int (*input_handler_t)(connection_t *connection);
typedef int (*connection_closed_handler_t)(connection_t *connection);

typedef struct service_s
{
	char *name;
	enum connection_type type;
	unsigned short port;
	int fd;
	struct sockaddr_in sin;
	int max_connections;
	connection_t *connections;
	new_connection_handler_t new_connection;
	input_handler_t input;
	connection_closed_handler_t connection_closed;
	void *priv;
	struct service_s *next;
} service_t;

extern int add_service(char *name, enum connection_type type, unsigned short port, int max_connections, new_connection_handler_t new_connection_handler, input_handler_t input_handler, connection_closed_handler_t connection_closed_handler, void *priv);
extern int server_init(void);
extern int server_quit(void);
extern int server_loop(command_context_t *command_context);
extern int server_register_commands(command_context_t *context);

extern int server_use_pipes;

#define ERROR_SERVER_REMOTE_CLOSED	(-400)
#define ERROR_CONNECTION_REJECTED	(-401)

#endif /* SERVER_H */
