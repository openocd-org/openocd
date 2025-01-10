/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_TARGET_REQUEST_H
#define OPENOCD_TARGET_TARGET_REQUEST_H

struct target;
struct command_context;

enum target_req_cmd {
	TARGET_REQ_TRACEMSG,
	TARGET_REQ_DEBUGMSG,
	TARGET_REQ_DEBUGCHAR,
/*	TARGET_REQ_SEMIHOSTING, */
};

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

#endif /* OPENOCD_TARGET_TARGET_REQUEST_H */
