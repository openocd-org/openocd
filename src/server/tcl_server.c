/***************************************************************************
 *   Copyright (C) 2008                                                    *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "tcl_server.h"


#define TCL_SERVER_VERSION	"TCL Server 0.1"
#define TCL_MAX_LINE		(4096)

struct tcl_connection {
	int tc_linedrop;
	int tc_lineoffset;
	char tc_line[TCL_MAX_LINE];
	int tc_outerror; /* flag an output error */
};

static unsigned short tcl_port = 6666;

/* handlers */
static int tcl_new_connection(struct connection *connection);
static int tcl_input(struct connection *connection);
static int tcl_output(struct connection *connection, const void *buf, ssize_t len);
static int tcl_closed(struct connection *connection);

/* write data out to a socket.
 *
 * this is a blocking write, so the return value must equal the length, if
 * that is not the case then flag the connection with an output error.
 */
int tcl_output(struct connection *connection, const void *data, ssize_t len)
{
	ssize_t wlen;
	struct tcl_connection *tclc;

	tclc = connection->priv;
	if (tclc->tc_outerror)
		return ERROR_SERVER_REMOTE_CLOSED;

	wlen = write_socket(connection->fd, data, len);
	if (wlen == len)
		return ERROR_OK;

	LOG_ERROR("error during write: %d != %d", (int)wlen, (int)len);
	tclc->tc_outerror = 1;
	return ERROR_SERVER_REMOTE_CLOSED;
}

/* connections */
static int tcl_new_connection(struct connection *connection)
{
	struct tcl_connection *tclc;

	tclc = malloc(sizeof(struct tcl_connection));
	if (tclc == NULL)
		return ERROR_CONNECTION_REJECTED;

	memset(tclc, 0, sizeof(struct tcl_connection));
	connection->priv = tclc;
	return ERROR_OK;
}

static int tcl_input(struct connection *connection)
{
	Jim_Interp *interp = (Jim_Interp *)connection->priv;
	int retval;
	int i;
	ssize_t rlen;
	const char *result;
	int reslen;
	struct tcl_connection *tclc;
	unsigned char in[256];

	rlen = read_socket(connection->fd, &in, sizeof(in));
	if (rlen <= 0) {
		if (rlen < 0)
			LOG_ERROR("error during read: %s", strerror(errno));
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	tclc = connection->priv;
	if (tclc == NULL)
		return ERROR_CONNECTION_REJECTED;

	/* push as much data into the line as possible */
	for (i = 0; i < rlen; i++)
	{
		if (!isprint(in[i]) && !isspace(in[i]))
		{
			/* drop this line */
			tclc->tc_linedrop = 1;
			continue;
		}

		/* buffer the data */
		tclc->tc_line[tclc->tc_lineoffset] = in[i];
		if (tclc->tc_lineoffset < TCL_MAX_LINE)
			tclc->tc_lineoffset++;
		else
			tclc->tc_linedrop = 1;

		if (in[i] != '\n')
			continue;

		/* process the line */
		if (tclc->tc_linedrop) {
#define ESTR "line too long\n"
			retval = tcl_output(connection, ESTR, sizeof(ESTR));
			if (retval != ERROR_OK)
				return retval;
#undef ESTR
		}
		else {
			tclc->tc_line[tclc->tc_lineoffset-1] = '\0';
			retval = Jim_Eval_Named(interp, tclc->tc_line, "remote:connection",1);
			result = Jim_GetString(Jim_GetResult(interp), &reslen);
			retval = tcl_output(connection, result, reslen);
			if (retval != ERROR_OK)
				return retval;
			if (memchr(result, '\n', reslen) == NULL)
				tcl_output(connection, "\n", 1);
		}

		tclc->tc_lineoffset = 0;
		tclc->tc_linedrop = 0;
	}

	return ERROR_OK;
}

static int tcl_closed(struct connection *connection)
{
	/* cleanup connection context */
	if (connection->priv) {
		free(connection->priv);
		connection->priv = NULL;
	}
	return ERROR_OK;
}

int tcl_init(struct command_context *cmd_ctx)
{
	int retval;

	if (tcl_port == 0)
	{
		LOG_INFO("tcl port disabled");
		return ERROR_OK;
	}

	retval = add_service("tcl", CONNECTION_TCP, tcl_port, 1,
			&tcl_new_connection, &tcl_input,
			&tcl_closed, cmd_ctx->interp);
	return retval;
}

COMMAND_HANDLER(handle_tcl_port_command)
{
	return CALL_COMMAND_HANDLER(server_port_command, &tcl_port);
}

static const struct command_registration tcl_command_handlers[] = {
	{
		.name = "tcl_port",
		.handler = handle_tcl_port_command,
		.mode = COMMAND_CONFIG,
		.help = "Specify port on which to listen "
			"for incoming Tcl syntax.  "
			"No arguments reports Tcl port; zero disables.",
		.usage = "[port_num]",
	},
	COMMAND_REGISTRATION_DONE
};

int tcl_register_commands(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, tcl_command_handlers);
}
