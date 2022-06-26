/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 ***************************************************************************/

#ifndef OPENOCD_SERVER_TELNET_SERVER_H
#define OPENOCD_SERVER_TELNET_SERVER_H

#include <server/server.h>

#define TELNET_BUFFER_SIZE (10*1024)

#define TELNET_LINE_HISTORY_SIZE (128)
#define TELNET_LINE_MAX_SIZE (10*256)

enum telnet_states {
	TELNET_STATE_DATA,
	TELNET_STATE_IAC,
	TELNET_STATE_SB,
	TELNET_STATE_SE,
	TELNET_STATE_WILL,
	TELNET_STATE_WONT,
	TELNET_STATE_DO,
	TELNET_STATE_DONT,
	TELNET_STATE_ESCAPE,
};

struct telnet_connection {
	char *prompt;
	bool prompt_visible;
	enum telnet_states state;
	char line[TELNET_LINE_MAX_SIZE];
	size_t line_size;
	size_t line_cursor;
	char last_escape;
	char *history[TELNET_LINE_HISTORY_SIZE];
	size_t next_history;
	size_t current_history;
	bool closed;
};

struct telnet_service {
	char *banner;
};

int telnet_init(char *banner);
int telnet_register_commands(struct command_context *command_context);
void telnet_service_free(void);

#endif /* OPENOCD_SERVER_TELNET_SERVER_H */
