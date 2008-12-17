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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "replacements.h"

#include "telnet_server.h"

#include "server.h"
#include "log.h"
#include "command.h"
#include "target.h"
#include "target_request.h"

#include <stdlib.h>
#include <unistd.h>
#include <errno.h>
#include <string.h>
#include <ctype.h>

static unsigned short telnet_port = 0;

int handle_exit_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_telnet_port_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

static char *negotiate =
		"\xFF\xFB\x03"		/* IAC WILL Suppress Go Ahead */
		"\xFF\xFB\x01"		/* IAC WILL Echo */
		"\xFF\xFD\x03"		/* IAC DO Suppress Go Ahead */
		"\xFF\xFE\x01";		/* IAC DON'T Echo */

#define CTRL(c) (c - '@')

/* The only way we can detect that the socket is closed is the first time
 * we write to it, we will fail. Subsequent write operations will
 * succeed. Shudder!
 */
int telnet_write(connection_t *connection, const void *data, int len)
{
	telnet_connection_t *t_con = connection->priv;
	if (t_con->closed)
		return ERROR_SERVER_REMOTE_CLOSED;

	if (write_socket(connection->fd, data, len) == len)
	{
		return ERROR_OK;
	}
	t_con->closed = 1;
	return ERROR_SERVER_REMOTE_CLOSED;
}

int telnet_prompt(connection_t *connection)
{
	telnet_connection_t *t_con = connection->priv;

	telnet_write(connection, "\r", 1); /* the prompt is always placed at the line beginning */
	return telnet_write(connection, t_con->prompt, strlen(t_con->prompt));
}

int telnet_outputline(connection_t *connection, const char *line)
{
	int len;

	/* process lines in buffer */
	while (*line) {
		char *line_end = strchr(line, '\n');

		if (line_end)
			len = line_end-line;
		else
			len = strlen(line);

		telnet_write(connection, line, len);
		if (line_end)
		{
			telnet_write(connection, "\r\n", 2);
			line += len+1;
		}
		else
		{
			line += len;
		}
	}

	return ERROR_OK;
}

int telnet_output(struct command_context_s *cmd_ctx, const char* line)
{
	connection_t *connection = cmd_ctx->output_handler_priv;

	return telnet_outputline(connection, line);
}

void telnet_log_callback(void *priv, const char *file, int line,
		const char *function, const char *string)
{
	connection_t *connection = priv;
	telnet_connection_t *t_con = connection->priv;
	int i;

	/* if there is no prompt, simply output the message */
	if (t_con->line_cursor < 0)
	{
		telnet_outputline(connection, string);
		return;
	}

	/* clear the command line */
	telnet_write(connection, "\r", 1);
	for (i = strlen(t_con->prompt) + t_con->line_size; i>0; i-=16)
		telnet_write(connection, "                ", i>16 ? 16 : i);
	telnet_write(connection, "\r", 1);

	/* output the message */
	telnet_outputline(connection, string);

	/* put the command line to its previous state */
	telnet_prompt(connection);
	telnet_write(connection, t_con->line, t_con->line_size);
	for (i=t_con->line_size; i>t_con->line_cursor; i--)
		telnet_write(connection, "\b", 1);
}

int telnet_new_connection(connection_t *connection)
{
	telnet_connection_t *telnet_connection = malloc(sizeof(telnet_connection_t));
	telnet_service_t *telnet_service = connection->service->priv;
	int i;

	connection->priv = telnet_connection;

	/* initialize telnet connection information */
	telnet_connection->closed = 0;
	telnet_connection->line_size = 0;
	telnet_connection->line_cursor = 0;
	telnet_connection->option_size = 0;
	telnet_connection->prompt = strdup("> ");
	telnet_connection->state = TELNET_STATE_DATA;

	/* output goes through telnet connection */
	command_set_output_handler(connection->cmd_ctx, telnet_output, connection);

	/* negotiate telnet options */
	telnet_write(connection, negotiate, strlen(negotiate));

	/* print connection banner */
	if (telnet_service->banner)
	{
		telnet_write(connection, telnet_service->banner, strlen(telnet_service->banner));
		telnet_write(connection, "\r\n", 2);
	}

	telnet_prompt(connection);

	/* initialize history */
	for (i = 0; i < TELNET_LINE_HISTORY_SIZE; i++)
	{
		telnet_connection->history[i] = NULL;
	}
	telnet_connection->next_history = 0;
	telnet_connection->current_history = 0;

	log_add_callback(telnet_log_callback, connection);

	return ERROR_OK;
}

void telnet_clear_line(connection_t *connection, telnet_connection_t *t_con)
{
	/* move to end of line */
	if (t_con->line_cursor < t_con->line_size)
	{
		telnet_write(connection, t_con->line + t_con->line_cursor, t_con->line_size - t_con->line_cursor);
	}

	/* backspace, overwrite with space, backspace */
	while (t_con->line_size > 0)
	{
		telnet_write(connection, "\b \b", 3);
		t_con->line_size--;
	}
	t_con->line_cursor = 0;
}

int telnet_input(connection_t *connection)
{
	int bytes_read;
	char buffer[TELNET_BUFFER_SIZE];
	char *buf_p;
	telnet_connection_t *t_con = connection->priv;
	command_context_t *command_context = connection->cmd_ctx;

	bytes_read = read_socket(connection->fd, buffer, TELNET_BUFFER_SIZE);

	if (bytes_read == 0)
		return ERROR_SERVER_REMOTE_CLOSED;
	else if (bytes_read == -1)
	{
		LOG_ERROR("error during read: %s", strerror(errno));
		return ERROR_SERVER_REMOTE_CLOSED;
	}

	buf_p = buffer;
	while (bytes_read)
	{
		switch (t_con->state)
		{
			case TELNET_STATE_DATA:
				if (*buf_p == '\xff')
				{
					t_con->state = TELNET_STATE_IAC;
				}
				else
				{
					if (isprint(*buf_p)) /* printable character */
					{
						/* watch buffer size leaving one spare character for string null termination */
						if (t_con->line_size == TELNET_LINE_MAX_SIZE-1)
						{
							/* output audible bell if buffer is full */
							telnet_write(connection, "\x07", 1); /* "\a" does not work, at least on windows */
						}
						else if (t_con->line_cursor == t_con->line_size)
						{
							telnet_write(connection, buf_p, 1);
							t_con->line[t_con->line_size++] = *buf_p;
							t_con->line_cursor++;
						}
						else
						{
							int i;
							memmove(t_con->line + t_con->line_cursor + 1, t_con->line + t_con->line_cursor, t_con->line_size - t_con->line_cursor);
							t_con->line[t_con->line_cursor] = *buf_p;
							t_con->line_size++;
							telnet_write(connection, t_con->line + t_con->line_cursor, t_con->line_size - t_con->line_cursor);
							t_con->line_cursor++;
							for (i = t_con->line_cursor; i < t_con->line_size; i++)
							{
								telnet_write(connection, "\b", 1);
							}
						}
					}
					else /* non-printable */
					{
						if (*buf_p == 0x1b) /* escape */
						{
							t_con->state = TELNET_STATE_ESCAPE;
							t_con->last_escape = '\x00';
						}
						else if ((*buf_p == 0xd) || (*buf_p == 0xa)) /* CR/LF */
						{
							int retval;

							/* skip over combinations with CR/LF and NUL characters */
							if ((bytes_read > 1) && ((*(buf_p + 1) == 0xa) || (*(buf_p + 1) == 0xd)))
							{
								buf_p++;
								bytes_read--;
							}
							if ((bytes_read > 1) && (*(buf_p + 1) == 0))
							{
								buf_p++;
								bytes_read--;
							}
							t_con->line[t_con->line_size] = 0;

							telnet_write(connection, "\r\n\x00", 3);

							if (strcmp(t_con->line, "history") == 0)
							{
								int i;
								for (i = 1; i < TELNET_LINE_HISTORY_SIZE; i++)
								{
									/* the t_con->next_history line contains empty string (unless NULL), thus it is not printed */
									char *history_line = t_con->history[(t_con->next_history + i) % TELNET_LINE_HISTORY_SIZE];
									if (history_line)
									{
										telnet_write(connection, history_line, strlen(history_line));
										telnet_write(connection, "\r\n\x00", 3);
									}
								}
								t_con->line_size = 0;
								t_con->line_cursor = 0;
								continue;
							}

							/* save only non-blank not repeating lines in the history */
							char *prev_line = t_con->history[(t_con->current_history > 0) ? t_con->current_history - 1 : TELNET_LINE_HISTORY_SIZE-1];
							if (*t_con->line && (prev_line == NULL || strcmp(t_con->line, prev_line)))
							{
								/* if the history slot is already taken, free it */
								if (t_con->history[t_con->next_history])
								{
									free(t_con->history[t_con->next_history]);
								}

								/* add line to history */
								t_con->history[t_con->next_history] = strdup(t_con->line);

								/* wrap history at TELNET_LINE_HISTORY_SIZE */
								t_con->next_history = (t_con->next_history + 1) % TELNET_LINE_HISTORY_SIZE;

								/* current history line starts at the new entry */
								t_con->current_history = t_con->next_history;

								if (t_con->history[t_con->current_history])
								{
									free(t_con->history[t_con->current_history]);
								}
								t_con->history[t_con->current_history] = strdup("");
							}

							t_con->line_size = 0;

							t_con->line_cursor = -1; /* to supress prompt in log callback during command execution */

							retval = command_run_line(command_context, t_con->line);

							t_con->line_cursor = 0;

							if (retval == ERROR_COMMAND_CLOSE_CONNECTION)
								return ERROR_SERVER_REMOTE_CLOSED;

							retval = telnet_prompt(connection);
							if (retval == ERROR_SERVER_REMOTE_CLOSED)
								return ERROR_SERVER_REMOTE_CLOSED;

						}
						else if ((*buf_p == 0x7f) || (*buf_p == 0x8)) /* delete character */
						{
							if (t_con->line_cursor > 0)
							{
								if (t_con->line_cursor != t_con->line_size)
								{
									int i;
									telnet_write(connection, "\b", 1);
									t_con->line_cursor--;
									t_con->line_size--;
									memmove(t_con->line + t_con->line_cursor, t_con->line + t_con->line_cursor + 1, t_con->line_size - t_con->line_cursor);

									telnet_write(connection, t_con->line + t_con->line_cursor, t_con->line_size - t_con->line_cursor);
									telnet_write(connection, " \b", 2);
									for (i = t_con->line_cursor; i < t_con->line_size; i++)
									{
										telnet_write(connection, "\b", 1);
									}
								}
								else
								{
									t_con->line_size--;
									t_con->line_cursor--;
									/* back space: move the 'printer' head one char back, overwrite with space, move back again */
									telnet_write(connection, "\b \b", 3);
								}
							}
						}
						else if (*buf_p == 0x15) /* clear line */
						{
							telnet_clear_line(connection, t_con);
						}
						else if (*buf_p == CTRL('B')) /* cursor left */
						{
							if (t_con->line_cursor > 0)
							{
								telnet_write(connection, "\b", 1);
								t_con->line_cursor--;
							}
							t_con->state = TELNET_STATE_DATA;
						}
						else if (*buf_p == CTRL('F')) /* cursor right */
						{
							if (t_con->line_cursor < t_con->line_size)
							{
								telnet_write(connection, t_con->line + t_con->line_cursor++, 1);
							}
							t_con->state = TELNET_STATE_DATA;
						}
						else
						{
							LOG_DEBUG("unhandled nonprintable: %2.2x", *buf_p);
						}
					}
				}
				break;
			case TELNET_STATE_IAC:
				switch (*buf_p)
				{
					case '\xfe':
						t_con->state = TELNET_STATE_DONT;
						break;
					case '\xfd':
						t_con->state = TELNET_STATE_DO;
						break;
					case '\xfc':
						t_con->state = TELNET_STATE_WONT;
						break;
					case '\xfb':
						t_con->state = TELNET_STATE_WILL;
						break;
				}
				break;
			case TELNET_STATE_SB:
				break;
			case TELNET_STATE_SE:
				break;
			case TELNET_STATE_WILL:
			case TELNET_STATE_WONT:
			case TELNET_STATE_DO:
			case TELNET_STATE_DONT:
				t_con->state = TELNET_STATE_DATA;
				break;
			case TELNET_STATE_ESCAPE:
				if (t_con->last_escape == '[')
				{
					if (*buf_p == 'D') /* cursor left */
					{
						if (t_con->line_cursor > 0)
						{
							telnet_write(connection, "\b", 1);
							t_con->line_cursor--;
						}
						t_con->state = TELNET_STATE_DATA;
					}
					else if (*buf_p == 'C') /* cursor right */
					{
						if (t_con->line_cursor < t_con->line_size)
						{
							telnet_write(connection, t_con->line + t_con->line_cursor++, 1);
						}
						t_con->state = TELNET_STATE_DATA;
					}
					else if (*buf_p == 'A') /* cursor up */
					{
						int last_history = (t_con->current_history > 0) ? t_con->current_history - 1 : TELNET_LINE_HISTORY_SIZE-1;
						if (t_con->history[last_history])
						{
							telnet_clear_line(connection, t_con);
							t_con->line_size = strlen(t_con->history[last_history]);
							t_con->line_cursor = t_con->line_size;
							memcpy(t_con->line, t_con->history[last_history], t_con->line_size);
							telnet_write(connection, t_con->line, t_con->line_size);
							t_con->current_history = last_history;
						}
						t_con->state = TELNET_STATE_DATA;
					}
					else if (*buf_p == 'B') /* cursor down */
					{
						int next_history = (t_con->current_history + 1) % TELNET_LINE_HISTORY_SIZE;
						if (t_con->history[next_history])
						{
							telnet_clear_line(connection, t_con);
							t_con->line_size = strlen(t_con->history[next_history]);
							t_con->line_cursor = t_con->line_size;
							memcpy(t_con->line, t_con->history[next_history], t_con->line_size);
							telnet_write(connection, t_con->line, t_con->line_size);
							t_con->current_history = next_history;
						}
						t_con->state = TELNET_STATE_DATA;
					}
					else if (*buf_p == '3')
					{
						t_con->last_escape = *buf_p;
					}
					else
					{
						t_con->state = TELNET_STATE_DATA;
					}
				}
				else if (t_con->last_escape == '3')
				{
					/* Remove character */
					if (*buf_p == '~')
					{
						if (t_con->line_cursor < t_con->line_size)
						{
							int i;
							t_con->line_size--;
							/* remove char from line buffer */
							memmove(t_con->line + t_con->line_cursor, t_con->line + t_con->line_cursor + 1, t_con->line_size - t_con->line_cursor);

							/* print remainder of buffer */
							telnet_write(connection, t_con->line + t_con->line_cursor, t_con->line_size - t_con->line_cursor);
							/* overwrite last char with whitespace */
							telnet_write(connection, " \b", 2);

							/* move back to cursor position*/
							for (i = t_con->line_cursor; i < t_con->line_size; i++)
							{
								telnet_write(connection, "\b", 1);
							}
						}

						t_con->state = TELNET_STATE_DATA;
					}
					else
					{
						t_con->state = TELNET_STATE_DATA;
					}
				}
				else if (t_con->last_escape == '\x00')
				{
					if (*buf_p == '[')
					{
						t_con->last_escape = *buf_p;
					}
					else
					{
						t_con->state = TELNET_STATE_DATA;
					}
				}
				else
				{
					LOG_ERROR("BUG: unexpected value in t_con->last_escape");
					t_con->state = TELNET_STATE_DATA;
				}

				break;
			default:
				LOG_ERROR("unknown telnet state");
				exit(-1);
		}

		bytes_read--;
		buf_p++;
	}

	return ERROR_OK;
}

int telnet_connection_closed(connection_t *connection)
{
	telnet_connection_t *t_con = connection->priv;
	int i;

	log_remove_callback(telnet_log_callback, connection);

	if (t_con->prompt)
	{
		free(t_con->prompt);
		t_con->prompt = NULL;
	}

	for (i = 0; i < TELNET_LINE_HISTORY_SIZE; i++)
	{
		if (t_con->history[i])
		{
			free(t_con->history[i]);
			t_con->history[i] = NULL;
		}
	}

	/* if this connection registered a debug-message receiver delete it */
	delete_debug_msg_receiver(connection->cmd_ctx, NULL);

	if (connection->priv)
	{
		free(connection->priv);
		connection->priv = NULL;
	}
	else
	{
		LOG_ERROR("BUG: connection->priv == NULL");
	}

	return ERROR_OK;
}

int telnet_set_prompt(connection_t *connection, char *prompt)
{
	telnet_connection_t *t_con = connection->priv;

	if (t_con->prompt != NULL)
		free(t_con->prompt);

	t_con->prompt = strdup(prompt);

	return ERROR_OK;
}

int telnet_init(char *banner)
{
	telnet_service_t *telnet_service = malloc(sizeof(telnet_service_t));

	if (telnet_port == 0)
	{
		LOG_WARNING("no telnet port specified, using default port 4444");
		telnet_port = 4444;
	}

	telnet_service->banner = banner;

	add_service("telnet", CONNECTION_TCP, telnet_port, 1, telnet_new_connection, telnet_input, telnet_connection_closed, telnet_service);

	return ERROR_OK;
}

int telnet_register_commands(command_context_t *command_context)
{
	register_command(command_context, NULL, "exit", handle_exit_command,
					 COMMAND_EXEC, "exit telnet session");

	register_command(command_context, NULL, "telnet_port", handle_telnet_port_command,
					 COMMAND_ANY, "port on which to listen for incoming telnet connections");

	return ERROR_OK;
}

/* daemon configuration command telnet_port */
int handle_telnet_port_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc == 0)
	{
		command_print(cmd_ctx, "%d", telnet_port);
		return ERROR_OK;
	}

	telnet_port = strtoul(args[0], NULL, 0);

	return ERROR_OK;
}

int handle_exit_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	return ERROR_COMMAND_CLOSE_CONNECTION;
}
