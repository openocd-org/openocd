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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "replacements.h"
#include "log.h"
#include "target.h"
#include "target_request.h"
#include "binarybuffer.h"
#include "command.h"
#include "trace.h"

#include <stdlib.h>
#include <string.h>

command_t *target_request_cmd = NULL;
static int charmsg_mode = 0;

int target_asciimsg(target_t *target, u32 length)
{
	char *msg = malloc(CEIL(length + 1, 4) * 4);
	debug_msg_receiver_t *c = target->dbgmsg;
	
	target->type->target_request_data(target, CEIL(length, 4), (u8*)msg);
	msg[length] = 0;
	
	LOG_DEBUG("%s", msg);
	
	while (c)
	{
		command_print(c->cmd_ctx, "%s", msg);
		c = c->next;
	}
	
	return ERROR_OK;
}

int target_charmsg(target_t *target, u8 msg)
{
	LOG_USER_N("%c", msg);
	
	return ERROR_OK;
}

int target_hexmsg(target_t *target, int size, u32 length)
{
	u8 *data = malloc(CEIL(length * size, 4) * 4);
	char line[128];
	int line_len;
	debug_msg_receiver_t *c = target->dbgmsg;
	int i;
	
	LOG_DEBUG("size: %i, length: %i", size, length);

	target->type->target_request_data(target, CEIL(length * size, 4), (u8*)data);

	line_len = 0;
	for (i = 0; i < length; i++)
	{
		switch (size)
		{
			case 4:
				line_len += snprintf(line + line_len, 128 - line_len, "%8.8x ", le_to_h_u32(data + (4*i)));
				break;
			case 2:
				line_len += snprintf(line + line_len, 128 - line_len, "%4.4x ", le_to_h_u16(data + (2*i)));
				break;
			case 1:
				line_len += snprintf(line + line_len, 128 - line_len, "%2.2x ", data[i]);
				break;
		}
		
		if ((i%8 == 7) || (i == length - 1))
		{
			LOG_DEBUG("%s", line);
			
			while (c)
			{
				command_print(c->cmd_ctx, "%s", line);
				c = c->next;
			}
			c = target->dbgmsg;
			line_len = 0;
		}
	}
	
	free(data);

	return ERROR_OK;
}

/* handle requests from the target received by a target specific
 * side-band channel (e.g. ARM7/9 DCC)
 */
int target_request(target_t *target, u32 request)
{
	target_req_cmd_t target_req_cmd = request & 0xff;

	if ( charmsg_mode ) {
		target_charmsg(target, target_req_cmd );
		return ERROR_OK;
	}
	switch (target_req_cmd)
	{
		case TARGET_REQ_TRACEMSG:
			trace_point(target, (request & 0xffffff00) >> 8);
			break;
		case TARGET_REQ_DEBUGMSG:
			if (((request & 0xff00) >> 8) == 0)
			{
				target_asciimsg(target, (request & 0xffff0000) >> 16);
			}
			else
			{
				target_hexmsg(target, (request & 0xff00) >> 8, (request & 0xffff0000) >> 16);
			}
			break;
		case TARGET_REQ_DEBUGCHAR:
			target_charmsg(target, (request & 0x00ff0000) >> 16);
			break;
/*		case TARGET_REQ_SEMIHOSTING:
 *			break;
 */
 		default:
 			LOG_ERROR("unknown target request: %2.2x", target_req_cmd);
 			break;
	}
	
	return ERROR_OK;
}

int add_debug_msg_receiver(struct command_context_s *cmd_ctx, target_t *target)
{
	debug_msg_receiver_t **p = &target->dbgmsg;
	
	if (target == NULL)
		return ERROR_INVALID_ARGUMENTS;

	/* see if there's already a list */
	if (*p)
	{
		/* find end of linked list */
		p = &target->dbgmsg;
		while ((*p)->next)
			p = &((*p)->next);
		p = &((*p)->next);
	}

	/* add new debug message receiver */
	(*p) = malloc(sizeof(debug_msg_receiver_t));
	(*p)->cmd_ctx = cmd_ctx;
	(*p)->next = NULL;
	
	/* enable callback */
	target->dbg_msg_enabled = 1;
	
	return ERROR_OK;
}

debug_msg_receiver_t* find_debug_msg_receiver(struct command_context_s *cmd_ctx, target_t *target)
{
	int do_all_targets = 0;
	debug_msg_receiver_t **p = &target->dbgmsg;
	
	/* if no target has been specified search all of them */
	if (target == NULL)
	{
		/* if no targets haven been specified */
		if (all_targets == NULL)
			return NULL;

		target = all_targets;
		do_all_targets = 1;
	}
	
	do
	{
		while (*p)
		{
			if ((*p)->cmd_ctx == cmd_ctx)
			{
				return *p;
			}
			p = &((*p)->next);
		}
		
		target = target->next;
	} while (target && do_all_targets);
	
	return NULL;
}

int delete_debug_msg_receiver(struct command_context_s *cmd_ctx, target_t *target)
{
	debug_msg_receiver_t **p;
	debug_msg_receiver_t *c;
	int do_all_targets = 0;
	
	/* if no target has been specified search all of them */
	if (target == NULL)
	{
		/* if no targets haven been specified */
		if (all_targets == NULL)
			return ERROR_OK;
		
		target = all_targets;
		do_all_targets = 1;
	}

	do
	{
		p = &target->dbgmsg;
		c = *p;
		while (c)
		{
			debug_msg_receiver_t *next = c->next;
			if (c->cmd_ctx == cmd_ctx)
			{
				*p = next;
				free(c);
				if (*p == NULL)
				{
					/* disable callback */
					target->dbg_msg_enabled = 0;
				}
				return ERROR_OK;
			}
			else
				p = &(c->next);
			c = next;
		}
	
		target = target->next;
	} while (target && do_all_targets);
	
	return ERROR_OK;
}

int handle_target_request_debugmsgs_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);

	int receiving = 0;
	
	/* see if reciever is already registered */
	if (find_debug_msg_receiver(cmd_ctx, target) != NULL)
		receiving = 1;

	if (argc > 0)
	{
		if (!strcmp(args[0], "enable") || !strcmp(args[0], "charmsg"))
		{
			/* don't register if this command context is already receiving */
			if (!receiving)
			{
				receiving = 1;
				add_debug_msg_receiver(cmd_ctx, target);
			}
			charmsg_mode = !strcmp(args[0], "charmsg");
		}
		else if (!strcmp(args[0], "disable"))
		{
			/* no need to delete a receiver if none is registered */
			if (receiving)
			{
				receiving = 0;
				delete_debug_msg_receiver(cmd_ctx, target);
			}
		}
		else
		{
			command_print(cmd_ctx, "usage: target_request debugmsgs ['enable'|'disable'|'charmsg']");
		}
	}
	
	command_print(cmd_ctx, "receiving debug messages from current target %s",
		      (receiving) ? (charmsg_mode?"charmsg":"enabled") : "disabled" );
	return ERROR_OK;
}

int target_request_register_commands(struct command_context_s *cmd_ctx)
{
	target_request_cmd =
		register_command(cmd_ctx, NULL, "target_request", NULL, COMMAND_ANY, "target_request commands");
	
	register_command(cmd_ctx, target_request_cmd, "debugmsgs", handle_target_request_debugmsgs_command,
		COMMAND_EXEC, "enable/disable reception of debug messages from target");

	return ERROR_OK;
}
