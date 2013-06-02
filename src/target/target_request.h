/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifndef TARGET_REQUEST_H
#define TARGET_REQUEST_H

struct target;
struct command_context;

typedef enum target_req_cmd {
	TARGET_REQ_TRACEMSG,
	TARGET_REQ_DEBUGMSG,
	TARGET_REQ_DEBUGCHAR,
/*	TARGET_REQ_SEMIHOSTING, */
} target_req_cmd_t;

struct debug_msg_receiver {
	struct command_context *cmd_ctx;
	struct debug_msg_receiver *next;
};

int target_request(struct target *target, uint32_t request);
int delete_debug_msg_receiver(struct command_context *cmd_ctx,
		struct target *target);
int target_request_register_commands(struct command_context *cmd_ctx);
/**
 * Read and clear the flag as to whether we got a message.
 *
 * This is used to implement the back-off algorithm on
 * sleeping in idle mode.
 */
bool target_got_message(void);

#endif /* TARGET_REQUEST_H */
