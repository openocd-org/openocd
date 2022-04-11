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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef OPENOCD_SERVER_SERVER_H
#define OPENOCD_SERVER_SERVER_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>
#include <helper/replacements.h>

#ifdef HAVE_NETINET_IN_H
#include <netinet/in.h>
#endif

enum connection_type {
	CONNECTION_TCP,
	CONNECTION_PIPE,
	CONNECTION_STDINOUT
};

#define CONNECTION_LIMIT_UNLIMITED		(-1)

struct connection {
	int fd;
	int fd_out;	/* When using pipes we're writing to a different fd */
	struct sockaddr_in sin;
	struct command_context *cmd_ctx;
	struct service *service;
	bool input_pending;
	void *priv;
	struct connection *next;
};

struct service_driver {
	/** the name of the server */
	const char *name;
	/** optional minimal setup to accept a connection during keep-alive */
	int (*new_connection_during_keep_alive_handler)(struct connection *connection);
	/**
	 * complete code to accept a new connection.
	 * If 'new_connection_during_keep_alive_handler' above is present, this can be
	 * either called alone during the server_loop, or after the function above.
	 * Check the implementation in gdb_server.
	 * */
	int (*new_connection_handler)(struct connection *connection);
	/** callback to handle incoming data */
	int (*input_handler)(struct connection *connection);
	/** callback to tear down the connection */
	int (*connection_closed_handler)(struct connection *connection);
	/** called periodically to send keep-alive messages on the connection */
	void (*keep_client_alive_handler)(struct connection *connection);
};

struct service {
	char *name;
	enum connection_type type;
	char *port;
	unsigned short portnumber;
	int fd;
	struct sockaddr_in sin;
	int max_connections;
	struct connection *connections;
	int (*new_connection_during_keep_alive)(struct connection *connection);
	int (*new_connection)(struct connection *connection);
	int (*input)(struct connection *connection);
	int (*connection_closed)(struct connection *connection);
	void (*keep_client_alive)(struct connection *connection);
	void *priv;
	struct service *next;
};

int add_service(const struct service_driver *driver, const char *port,
		int max_connections, void *priv);
int remove_service(const char *name, const char *port);

int server_host_os_entry(void);
int server_host_os_close(void);

int server_preinit(void);
int server_init(struct command_context *cmd_ctx);
int server_quit(void);
void server_free(void);
void exit_on_signal(int sig);

void server_keep_clients_alive(void);

int server_loop(struct command_context *command_context);

int server_register_commands(struct command_context *context);

int connection_write(struct connection *connection, const void *data, int len);
int connection_read(struct connection *connection, void *data, int len);

/**
 * Defines an extended command handler function declaration to enable
 * access to (and manipulation of) the server port number.
 * Call server_port like a normal COMMAND_HANDLER with an extra @a out parameter
 * to receive the specified port number.
 */
COMMAND_HELPER(server_pipe_command, char **out);

COMMAND_HELPER(server_port_command, unsigned short *out);

#define ERROR_SERVER_REMOTE_CLOSED		(-400)
#define ERROR_CONNECTION_REJECTED		(-401)

#endif /* OPENOCD_SERVER_SERVER_H */
