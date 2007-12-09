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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "replacements.h"
#include "target.h"
#include "target_request.h"

#include "log.h"
#include "configuration.h"
#include "binarybuffer.h"
#include "jtag.h"

#include <string.h>
#include <stdlib.h>
#include <inttypes.h>

#include <sys/types.h>
#include <sys/stat.h>
#include <unistd.h>
#include <errno.h>

#include <sys/time.h>
#include <time.h>

#include <time_support.h>

#include <fileio.h>
#include <image.h>

int cli_target_callback_event_handler(struct target_s *target, enum target_event event, void *priv);

int handle_target_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_daemon_startup_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_targets_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

int handle_target_script_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_run_and_halt_time_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_working_area_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

int handle_reg_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_poll_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_halt_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_wait_halt_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_reset_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_soft_reset_halt_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_resume_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_step_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_md_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_mw_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_load_image_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_dump_image_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_verify_image_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_bp_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_rbp_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_wp_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_rwp_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

/* targets
 */
extern target_type_t arm7tdmi_target;
extern target_type_t arm720t_target;
extern target_type_t arm9tdmi_target;
extern target_type_t arm920t_target;
extern target_type_t arm966e_target;
extern target_type_t arm926ejs_target;
extern target_type_t xscale_target;
extern target_type_t cortexm3_target;

target_type_t *target_types[] =
{
	&arm7tdmi_target,
	&arm9tdmi_target,
	&arm920t_target,
	&arm720t_target,
	&arm966e_target,
	&arm926ejs_target,
	&xscale_target,
	&cortexm3_target,
	NULL,
};

target_t *targets = NULL;
target_event_callback_t *target_event_callbacks = NULL;
target_timer_callback_t *target_timer_callbacks = NULL;

char *target_state_strings[] =
{
	"unknown",
	"running",
	"halted",
	"reset",
	"debug_running",
};

char *target_debug_reason_strings[] =
{
	"debug request", "breakpoint", "watchpoint",
	"watchpoint and breakpoint", "single step",
	"target not halted"
};

char *target_endianess_strings[] =
{
	"big endian",
	"little endian",
};

enum daemon_startup_mode startup_mode = DAEMON_ATTACH;

static int target_continous_poll = 1;

/* read a u32 from a buffer in target memory endianness */
u32 target_buffer_get_u32(target_t *target, u8 *buffer)
{
	if (target->endianness == TARGET_LITTLE_ENDIAN)
		return le_to_h_u32(buffer);
	else
		return be_to_h_u32(buffer);
}

/* read a u16 from a buffer in target memory endianness */
u16 target_buffer_get_u16(target_t *target, u8 *buffer)
{
	if (target->endianness == TARGET_LITTLE_ENDIAN)
		return le_to_h_u16(buffer);
	else
		return be_to_h_u16(buffer);
}

/* write a u32 to a buffer in target memory endianness */
void target_buffer_set_u32(target_t *target, u8 *buffer, u32 value)
{
	if (target->endianness == TARGET_LITTLE_ENDIAN)
		h_u32_to_le(buffer, value);
	else
		h_u32_to_be(buffer, value);
}

/* write a u16 to a buffer in target memory endianness */
void target_buffer_set_u16(target_t *target, u8 *buffer, u16 value)
{
	if (target->endianness == TARGET_LITTLE_ENDIAN)
		h_u16_to_le(buffer, value);
	else
		h_u16_to_be(buffer, value);
}

/* returns a pointer to the n-th configured target */
target_t* get_target_by_num(int num)
{
	target_t *target = targets;
	int i = 0;

	while (target)
	{
		if (num == i)
			return target;
		target = target->next;
		i++;
	}

	return NULL;
}

int get_num_by_target(target_t *query_target)
{
	target_t *target = targets;
	int i = 0;	
	
	while (target)
	{
		if (target == query_target)
			return i;
		target = target->next;
		i++;
	}
	
	return -1;
}

target_t* get_current_target(command_context_t *cmd_ctx)
{
	target_t *target = get_target_by_num(cmd_ctx->current_target);
	
	if (target == NULL)
	{
		ERROR("BUG: current_target out of bounds");
		exit(-1);
	}
	
	return target;
}

/* Process target initialization, when target entered debug out of reset
 * the handler is unregistered at the end of this function, so it's only called once
 */
int target_init_handler(struct target_s *target, enum target_event event, void *priv)
{
	FILE *script;
	struct command_context_s *cmd_ctx = priv;
	
	if ((event == TARGET_EVENT_HALTED) && (target->reset_script))
	{
		target_unregister_event_callback(target_init_handler, priv);

		script = fopen(target->reset_script, "r");
		if (!script)
		{
			ERROR("couldn't open script file %s", target->reset_script);
				return ERROR_OK;
		}

		INFO("executing reset script '%s'", target->reset_script);
		command_run_file(cmd_ctx, script, COMMAND_EXEC);
		fclose(script);

		jtag_execute_queue();
	}
	
	return ERROR_OK;
}

int target_run_and_halt_handler(void *priv)
{
	target_t *target = priv;
	
	target->type->halt(target);
	
	return ERROR_OK;
}

int target_process_reset(struct command_context_s *cmd_ctx)
{
	int retval = ERROR_OK;
	target_t *target;
	
	/* prepare reset_halt where necessary */
	target = targets;
	while (target)
	{
		switch (target->reset_mode)
		{
			case RESET_HALT:
			case RESET_INIT:
				target->type->prepare_reset_halt(target);
				break;
			default:
				break;
		}
		target = target->next;
	}
	
	target = targets;
	while (target)
	{
		target->type->assert_reset(target);
		target = target->next;
	}
	jtag_execute_queue();
	
	/* request target halt if necessary, and schedule further action */
	target = targets;
	while (target)
	{
		switch (target->reset_mode)
		{
			case RESET_RUN:
				/* nothing to do if target just wants to be run */
				break;
			case RESET_RUN_AND_HALT:
				/* schedule halt */
				target_register_timer_callback(target_run_and_halt_handler, target->run_and_halt_time, 0, target);
				break;
			case RESET_RUN_AND_INIT:
				/* schedule halt */
				target_register_timer_callback(target_run_and_halt_handler, target->run_and_halt_time, 0, target);
				target_register_event_callback(target_init_handler, cmd_ctx);
				break;
			case RESET_HALT:
				target->type->halt(target);
				break;
			case RESET_INIT:
				target->type->halt(target);
				target_register_event_callback(target_init_handler, cmd_ctx);
				break;
			default:
				ERROR("BUG: unknown target->reset_mode");
		}
		target = target->next;
	}
	
	target = targets;
	while (target)
	{
		target->type->deassert_reset(target);

		switch (target->reset_mode)
		{
			case RESET_INIT:
			case RESET_HALT:
				// If we're already halted, then this is harmless(reducing # of execution paths here)
				// If nSRST & nTRST are tied together then the halt during reset failed(logged) and
				// we use this as fallback(there is no other output to tell the user that reset halt 
				// didn't work).
				target->type->poll(target);
				target->type->halt(target);
				break;
			default:
				break;
		}
		
		
		target = target->next;
	}
	jtag_execute_queue();
	
	return retval;
}	

int target_init(struct command_context_s *cmd_ctx)
{
	target_t *target = targets;
	
	while (target)
	{
		if (target->type->init_target(cmd_ctx, target) != ERROR_OK)
		{
			ERROR("target '%s' init failed", target->type->name);
			exit(-1);
		}
		target = target->next;
	}
	
	if (targets)
	{
		target_register_user_commands(cmd_ctx);
		target_register_timer_callback(handle_target, 100, 1, NULL);
	}
		
	if (startup_mode == DAEMON_RESET)
		target_process_reset(cmd_ctx);
	
	return ERROR_OK;
}

int target_register_event_callback(int (*callback)(struct target_s *target, enum target_event event, void *priv), void *priv)
{
	target_event_callback_t **callbacks_p = &target_event_callbacks;
	
	if (callback == NULL)
	{
		return ERROR_INVALID_ARGUMENTS;
	}
	
	if (*callbacks_p)
	{
		while ((*callbacks_p)->next)
			callbacks_p = &((*callbacks_p)->next);
		callbacks_p = &((*callbacks_p)->next);
	}
	
	(*callbacks_p) = malloc(sizeof(target_event_callback_t));
	(*callbacks_p)->callback = callback;
	(*callbacks_p)->priv = priv;
	(*callbacks_p)->next = NULL;
	
	return ERROR_OK;
}

int target_register_timer_callback(int (*callback)(void *priv), int time_ms, int periodic, void *priv)
{
	target_timer_callback_t **callbacks_p = &target_timer_callbacks;
	struct timeval now;
	
	if (callback == NULL)
	{
		return ERROR_INVALID_ARGUMENTS;
	}
	
	if (*callbacks_p)
	{
		while ((*callbacks_p)->next)
			callbacks_p = &((*callbacks_p)->next);
		callbacks_p = &((*callbacks_p)->next);
	}
	
	(*callbacks_p) = malloc(sizeof(target_timer_callback_t));
	(*callbacks_p)->callback = callback;
	(*callbacks_p)->periodic = periodic;
	(*callbacks_p)->time_ms = time_ms;
	
	gettimeofday(&now, NULL);
	(*callbacks_p)->when.tv_usec = now.tv_usec + (time_ms % 1000) * 1000;
	time_ms -= (time_ms % 1000);
	(*callbacks_p)->when.tv_sec = now.tv_sec + (time_ms / 1000);
	if ((*callbacks_p)->when.tv_usec > 1000000)
	{
		(*callbacks_p)->when.tv_usec = (*callbacks_p)->when.tv_usec - 1000000;
		(*callbacks_p)->when.tv_sec += 1;
	}
	
	(*callbacks_p)->priv = priv;
	(*callbacks_p)->next = NULL;
	
	return ERROR_OK;
}

int target_unregister_event_callback(int (*callback)(struct target_s *target, enum target_event event, void *priv), void *priv)
{
	target_event_callback_t **p = &target_event_callbacks;
	target_event_callback_t *c = target_event_callbacks;
	
	if (callback == NULL)
	{
		return ERROR_INVALID_ARGUMENTS;
	}
		
	while (c)
	{
		target_event_callback_t *next = c->next;
		if ((c->callback == callback) && (c->priv == priv))
		{
			*p = next;
			free(c);
			return ERROR_OK;
		}
		else
			p = &(c->next);
		c = next;
	}
	
	return ERROR_OK;
}

int target_unregister_timer_callback(int (*callback)(void *priv), void *priv)
{
	target_timer_callback_t **p = &target_timer_callbacks;
	target_timer_callback_t *c = target_timer_callbacks;
	
	if (callback == NULL)
	{
		return ERROR_INVALID_ARGUMENTS;
	}
		
	while (c)
	{
		target_timer_callback_t *next = c->next;
		if ((c->callback == callback) && (c->priv == priv))
		{
			*p = next;
			free(c);
			return ERROR_OK;
		}
		else
			p = &(c->next);
		c = next;
	}
	
	return ERROR_OK;
}

int target_call_event_callbacks(target_t *target, enum target_event event)
{
	target_event_callback_t *callback = target_event_callbacks;
	target_event_callback_t *next_callback;
	
	DEBUG("target event %i", event);
	
	while (callback)
	{
		next_callback = callback->next;
		callback->callback(target, event, callback->priv);
		callback = next_callback;
	}
	
	return ERROR_OK;
}

int target_call_timer_callbacks()
{
	target_timer_callback_t *callback = target_timer_callbacks;
	target_timer_callback_t *next_callback;
	struct timeval now;

	gettimeofday(&now, NULL);
	
	while (callback)
	{
		next_callback = callback->next;
		
		if (((now.tv_sec >= callback->when.tv_sec) && (now.tv_usec >= callback->when.tv_usec))
			|| (now.tv_sec > callback->when.tv_sec))
		{
			callback->callback(callback->priv);
			if (callback->periodic)
			{
				int time_ms = callback->time_ms;
				callback->when.tv_usec = now.tv_usec + (time_ms % 1000) * 1000;
				time_ms -= (time_ms % 1000);
				callback->when.tv_sec = now.tv_sec + time_ms / 1000;
				if (callback->when.tv_usec > 1000000)
				{
					callback->when.tv_usec = callback->when.tv_usec - 1000000;
					callback->when.tv_sec += 1;
				}
			}
			else
				target_unregister_timer_callback(callback->callback, callback->priv);
		}
			
		callback = next_callback;
	}
	
	return ERROR_OK;
}

int target_alloc_working_area(struct target_s *target, u32 size, working_area_t **area)
{
	working_area_t *c = target->working_areas;
	working_area_t *new_wa = NULL;
	
	/* only allocate multiples of 4 byte */
	if (size % 4)
	{
		ERROR("BUG: code tried to allocate unaligned number of bytes, padding");
		size = CEIL(size, 4);
	}
	
	/* see if there's already a matching working area */
	while (c)
	{
		if ((c->free) && (c->size == size))
		{
			new_wa = c;
			break;
		}
		c = c->next;
	}
	
	/* if not, allocate a new one */
	if (!new_wa)
	{
		working_area_t **p = &target->working_areas;
		u32 first_free = target->working_area;
		u32 free_size = target->working_area_size;
		
		DEBUG("allocating new working area");
		
		c = target->working_areas;
		while (c)
		{
			first_free += c->size;
			free_size -= c->size;
			p = &c->next;
			c = c->next;
		}
		
		if (free_size < size)
		{
			WARNING("not enough working area available");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		
		new_wa = malloc(sizeof(working_area_t));
		new_wa->next = NULL;
		new_wa->size = size;
		new_wa->address = first_free;
		
		if (target->backup_working_area)
		{
			new_wa->backup = malloc(new_wa->size);
			target->type->read_memory(target, new_wa->address, 4, new_wa->size / 4, new_wa->backup);
		}
		else
		{
			new_wa->backup = NULL;
		}
		
		/* put new entry in list */
		*p = new_wa;
	}
	
	/* mark as used, and return the new (reused) area */
	new_wa->free = 0;
	*area = new_wa;
	
	/* user pointer */
	new_wa->user = area;
	
	return ERROR_OK;
}

int target_free_working_area(struct target_s *target, working_area_t *area)
{
	if (area->free)
		return ERROR_OK;
	
	if (target->backup_working_area)
		target->type->write_memory(target, area->address, 4, area->size / 4, area->backup);
	
	area->free = 1;
	
	/* mark user pointer invalid */
	*area->user = NULL;
	area->user = NULL;
	
	return ERROR_OK;
}

int target_free_all_working_areas(struct target_s *target)
{
	working_area_t *c = target->working_areas;

	while (c)
	{
		working_area_t *next = c->next;
		target_free_working_area(target, c);
		
		if (c->backup)
			free(c->backup);
		
		free(c);
		
		c = next;
	}
	
	target->working_areas = NULL;
	
	return ERROR_OK;
}

int target_register_commands(struct command_context_s *cmd_ctx)
{
	register_command(cmd_ctx, NULL, "target", handle_target_command, COMMAND_CONFIG, NULL);
	register_command(cmd_ctx, NULL, "targets", handle_targets_command, COMMAND_EXEC, NULL);
	register_command(cmd_ctx, NULL, "daemon_startup", handle_daemon_startup_command, COMMAND_CONFIG, NULL);
	register_command(cmd_ctx, NULL, "target_script", handle_target_script_command, COMMAND_CONFIG, NULL);
	register_command(cmd_ctx, NULL, "run_and_halt_time", handle_run_and_halt_time_command, COMMAND_CONFIG, NULL);
	register_command(cmd_ctx, NULL, "working_area", handle_working_area_command, COMMAND_CONFIG, NULL);

	return ERROR_OK;
}

int target_write_buffer(struct target_s *target, u32 address, u32 size, u8 *buffer)
{
	int retval;
	
	DEBUG("writing buffer of %i byte at 0x%8.8x", size, address);
	
	/* handle writes of less than 4 byte */
	if (size < 4)
	{
		if ((retval = target->type->write_memory(target, address, 1, size, buffer)) != ERROR_OK)
			return retval;
		return ERROR_OK;
	}
	
	/* handle unaligned head bytes */
	if (address % 4)
	{
		int unaligned = 4 - (address % 4);
		
		if ((retval = target->type->write_memory(target, address, 1, unaligned, buffer)) != ERROR_OK)
			return retval;
		
		buffer += unaligned;
		address += unaligned;
		size -= unaligned;
	}
		
	/* handle aligned words */
	if (size >= 4)
	{
		int aligned = size - (size % 4);
	
		/* use bulk writes above a certain limit. This may have to be changed */
		if (aligned > 128)
		{
			if ((retval = target->type->bulk_write_memory(target, address, aligned / 4, buffer)) != ERROR_OK)
				return retval;
		}
		else
		{
			if ((retval = target->type->write_memory(target, address, 4, aligned / 4, buffer)) != ERROR_OK)
				return retval;
		}
		
		buffer += aligned;
		address += aligned;
		size -= aligned;
	}
	
	/* handle tail writes of less than 4 bytes */
	if (size > 0)
	{
		if ((retval = target->type->write_memory(target, address, 1, size, buffer)) != ERROR_OK)
			return retval;
	}
	
	return ERROR_OK;
}

int target_read_buffer(struct target_s *target, u32 address, u32 size, u8 *buffer)
{
	int retval;
	
	DEBUG("reading buffer of %i byte at 0x%8.8x", size, address);
	
	/* handle reads of less than 4 byte */
	if (size < 4)
	{
		if ((retval = target->type->read_memory(target, address, 1, size, buffer)) != ERROR_OK)
			return retval;
		return ERROR_OK;
	}
	
	/* handle unaligned head bytes */
	if (address % 4)
	{
		int unaligned = 4 - (address % 4);
		
		if ((retval = target->type->read_memory(target, address, 1, unaligned, buffer)) != ERROR_OK)
			return retval;
		
		buffer += unaligned;
		address += unaligned;
		size -= unaligned;
	}
		
	/* handle aligned words */
	if (size >= 4)
	{
		int aligned = size - (size % 4);
	
		if ((retval = target->type->read_memory(target, address, 4, aligned / 4, buffer)) != ERROR_OK)
			return retval;
		
		buffer += aligned;
		address += aligned;
		size -= aligned;
	}
	
	/* handle tail writes of less than 4 bytes */
	if (size > 0)
	{
		if ((retval = target->type->read_memory(target, address, 1, size, buffer)) != ERROR_OK)
			return retval;
	}
	
	return ERROR_OK;
}

int target_checksum_memory(struct target_s *target, u32 address, u32 size, u32* crc)
{
	u8 *buffer;
	int retval;
	int i;
	u32 checksum = 0;
	
	if ((retval = target->type->checksum_memory(target, address,
		size, &checksum)) == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
	{
		buffer = malloc(size);
		target_read_buffer(target, address, size, buffer);

		/* convert to target endianess */
		for (i = 0; i < (size/sizeof(u32)); i++)
		{
			u32 target_data;
			target_data = target_buffer_get_u32(target, &buffer[i*sizeof(u32)]);
			target_buffer_set_u32(target, &buffer[i*sizeof(u32)], target_data);
		}

		retval = image_calculate_checksum( buffer, size, &checksum );
		free(buffer);
	}
	
	*crc = checksum;
	
	return retval;
}

int target_read_u32(struct target_s *target, u32 address, u32 *value)
{
	u8 value_buf[4];

	int retval = target->type->read_memory(target, address, 4, 1, value_buf);
	
	if (retval == ERROR_OK)
	{
		*value = target_buffer_get_u32(target, value_buf);
		DEBUG("address: 0x%8.8x, value: 0x%8.8x", address, *value);
	}
	else
	{
		*value = 0x0;
		DEBUG("address: 0x%8.8x failed", address);
	}
	
	return retval;
}

int target_read_u16(struct target_s *target, u32 address, u16 *value)
{
	u8 value_buf[2];
	
	int retval = target->type->read_memory(target, address, 2, 1, value_buf);
	
	if (retval == ERROR_OK)
	{
		*value = target_buffer_get_u16(target, value_buf);
		DEBUG("address: 0x%8.8x, value: 0x%4.4x", address, *value);
	}
	else
	{
		*value = 0x0;
		DEBUG("address: 0x%8.8x failed", address);
	}
	
	return retval;
}

int target_read_u8(struct target_s *target, u32 address, u8 *value)
{
	int retval = target->type->read_memory(target, address, 1, 1, value);

	if (retval == ERROR_OK)
	{
		DEBUG("address: 0x%8.8x, value: 0x%2.2x", address, *value);
	}
	else
	{
		*value = 0x0;
		DEBUG("address: 0x%8.8x failed", address);
	}
	
	return retval;
}

int target_write_u32(struct target_s *target, u32 address, u32 value)
{
	int retval;
	u8 value_buf[4];

	DEBUG("address: 0x%8.8x, value: 0x%8.8x", address, value);

	target_buffer_set_u32(target, value_buf, value);	
	if ((retval = target->type->write_memory(target, address, 4, 1, value_buf)) != ERROR_OK)
	{
		DEBUG("failed: %i", retval);
	}
	
	return retval;
}

int target_write_u16(struct target_s *target, u32 address, u16 value)
{
	int retval;
	u8 value_buf[2];
	
	DEBUG("address: 0x%8.8x, value: 0x%8.8x", address, value);

	target_buffer_set_u16(target, value_buf, value);	
	if ((retval = target->type->write_memory(target, address, 2, 1, value_buf)) != ERROR_OK)
	{
		DEBUG("failed: %i", retval);
	}
	
	return retval;
}

int target_write_u8(struct target_s *target, u32 address, u8 value)
{
	int retval;
	
	DEBUG("address: 0x%8.8x, value: 0x%2.2x", address, value);

	if ((retval = target->type->read_memory(target, address, 1, 1, &value)) != ERROR_OK)
	{
		DEBUG("failed: %i", retval);
	}
	
	return retval;
}

int target_register_user_commands(struct command_context_s *cmd_ctx)
{
	register_command(cmd_ctx,  NULL, "reg", handle_reg_command, COMMAND_EXEC, NULL);
	register_command(cmd_ctx,  NULL, "poll", handle_poll_command, COMMAND_EXEC, "poll target state");
	register_command(cmd_ctx,  NULL, "wait_halt", handle_wait_halt_command, COMMAND_EXEC, "wait for target halt [time (s)]");
	register_command(cmd_ctx,  NULL, "halt", handle_halt_command, COMMAND_EXEC, "halt target");
	register_command(cmd_ctx,  NULL, "resume", handle_resume_command, COMMAND_EXEC, "resume target [addr]");
	register_command(cmd_ctx,  NULL, "step", handle_step_command, COMMAND_EXEC, "step one instruction from current PC or [addr]");
	register_command(cmd_ctx,  NULL, "reset", handle_reset_command, COMMAND_EXEC, "reset target [run|halt|init|run_and_halt|run_and_init]");
	register_command(cmd_ctx,  NULL, "soft_reset_halt", handle_soft_reset_halt_command, COMMAND_EXEC, "halt the target and do a soft reset");

	register_command(cmd_ctx,  NULL, "mdw", handle_md_command, COMMAND_EXEC, "display memory words <addr> [count]");
	register_command(cmd_ctx,  NULL, "mdh", handle_md_command, COMMAND_EXEC, "display memory half-words <addr> [count]");
	register_command(cmd_ctx,  NULL, "mdb", handle_md_command, COMMAND_EXEC, "display memory bytes <addr> [count]");
	
	register_command(cmd_ctx,  NULL, "mww", handle_mw_command, COMMAND_EXEC, "write memory word <addr> <value>");
	register_command(cmd_ctx,  NULL, "mwh", handle_mw_command, COMMAND_EXEC, "write memory half-word <addr> <value>");
	register_command(cmd_ctx,  NULL, "mwb", handle_mw_command, COMMAND_EXEC, "write memory byte <addr> <value>");
	
	register_command(cmd_ctx,  NULL, "bp", handle_bp_command, COMMAND_EXEC, "set breakpoint <address> <length> [hw]");	
	register_command(cmd_ctx,  NULL, "rbp", handle_rbp_command, COMMAND_EXEC, "remove breakpoint <adress>");
	register_command(cmd_ctx,  NULL, "wp", handle_wp_command, COMMAND_EXEC, "set watchpoint <address> <length> <r/w/a> [value] [mask]");	
	register_command(cmd_ctx,  NULL, "rwp", handle_rwp_command, COMMAND_EXEC, "remove watchpoint <adress>");
	
	register_command(cmd_ctx,  NULL, "load_image", handle_load_image_command, COMMAND_EXEC, "load_image <file> <address> ['bin'|'ihex'|'elf'|'s19']");
	register_command(cmd_ctx,  NULL, "dump_image", handle_dump_image_command, COMMAND_EXEC, "dump_image <file> <address> <size>");
	register_command(cmd_ctx,  NULL, "verify_image", handle_verify_image_command, COMMAND_EXEC, "verify_image <file> [offset] [type]");
	register_command(cmd_ctx,  NULL, "load_binary", handle_load_image_command, COMMAND_EXEC, "[DEPRECATED] load_binary <file> <address>");
	register_command(cmd_ctx,  NULL, "dump_binary", handle_dump_image_command, COMMAND_EXEC, "[DEPRECATED] dump_binary <file> <address> <size>");
	
	target_request_register_commands(cmd_ctx);
	trace_register_commands(cmd_ctx);
	
	return ERROR_OK;
}

int handle_targets_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = targets;
	int count = 0;
	
	if (argc == 1)
	{
		int num = strtoul(args[0], NULL, 0);
		
		while (target)
		{
			count++;
			target = target->next;
		}
		
		if (num < count)
			cmd_ctx->current_target = num;
		else
			command_print(cmd_ctx, "%i is out of bounds, only %i targets are configured", num, count);
			
		return ERROR_OK;
	}
		
	while (target)
	{
		command_print(cmd_ctx, "%i: %s (%s), state: %s", count++, target->type->name, target_endianess_strings[target->endianness], target_state_strings[target->state]);
		target = target->next;
	}
	
	return ERROR_OK;
}

int handle_target_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int i;
	int found = 0;
	
	if (argc < 3)
	{
		ERROR("target command requires at least three arguments: <type> <endianess> <reset_mode>");
		exit(-1);
	}
	
	/* search for the specified target */
	if (args[0] && (args[0][0] != 0))
	{
		for (i = 0; target_types[i]; i++)
		{
			if (strcmp(args[0], target_types[i]->name) == 0)
			{
				target_t **last_target_p = &targets;
				
				/* register target specific commands */
				if (target_types[i]->register_commands(cmd_ctx) != ERROR_OK)
				{
					ERROR("couldn't register '%s' commands", args[0]);
					exit(-1);
				}

				if (*last_target_p)
				{
					while ((*last_target_p)->next)
						last_target_p = &((*last_target_p)->next);
					last_target_p = &((*last_target_p)->next);
				}

				*last_target_p = malloc(sizeof(target_t));
				
				(*last_target_p)->type = target_types[i];
				
				if (strcmp(args[1], "big") == 0)
					(*last_target_p)->endianness = TARGET_BIG_ENDIAN;
				else if (strcmp(args[1], "little") == 0)
					(*last_target_p)->endianness = TARGET_LITTLE_ENDIAN;
				else
				{
					ERROR("endianness must be either 'little' or 'big', not '%s'", args[1]);
					exit(-1);
				}
				
				/* what to do on a target reset */
				if (strcmp(args[2], "reset_halt") == 0)
					(*last_target_p)->reset_mode = RESET_HALT;
				else if (strcmp(args[2], "reset_run") == 0)
					(*last_target_p)->reset_mode = RESET_RUN;
				else if (strcmp(args[2], "reset_init") == 0)
					(*last_target_p)->reset_mode = RESET_INIT;
				else if (strcmp(args[2], "run_and_halt") == 0)
					(*last_target_p)->reset_mode = RESET_RUN_AND_HALT;
				else if (strcmp(args[2], "run_and_init") == 0)
					(*last_target_p)->reset_mode = RESET_RUN_AND_INIT;
				else
				{
					ERROR("unknown target startup mode %s", args[2]);
					exit(-1);
				}
				(*last_target_p)->run_and_halt_time = 1000; /* default 1s */
				
				(*last_target_p)->reset_script = NULL;
				(*last_target_p)->post_halt_script = NULL;
				(*last_target_p)->pre_resume_script = NULL;
				
				(*last_target_p)->working_area = 0x0;
				(*last_target_p)->working_area_size = 0x0;
				(*last_target_p)->working_areas = NULL;
				(*last_target_p)->backup_working_area = 0;
				
				(*last_target_p)->state = TARGET_UNKNOWN;
				(*last_target_p)->reg_cache = NULL;
				(*last_target_p)->breakpoints = NULL;
				(*last_target_p)->watchpoints = NULL;
				(*last_target_p)->next = NULL;
				(*last_target_p)->arch_info = NULL;
				
				/* initialize trace information */
				(*last_target_p)->trace_info = malloc(sizeof(trace_t));
				(*last_target_p)->trace_info->num_trace_points = 0;
				(*last_target_p)->trace_info->trace_points_size = 0;
				(*last_target_p)->trace_info->trace_points = NULL;
				(*last_target_p)->trace_info->trace_history_size = 0;
				(*last_target_p)->trace_info->trace_history = NULL;
				(*last_target_p)->trace_info->trace_history_pos = 0;
				(*last_target_p)->trace_info->trace_history_overflowed = 0;
				
				(*last_target_p)->dbgmsg = NULL;
								
				(*last_target_p)->type->target_command(cmd_ctx, cmd, args, argc, *last_target_p);
				
				found = 1;
				break;
			}
		}
	}
	
	/* no matching target found */
	if (!found)
	{
		ERROR("target '%s' not found", args[0]);
		exit(-1);
	}

	return ERROR_OK;
}

/* usage: target_script <target#> <event> <script_file> */
int handle_target_script_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = NULL;
	
	if (argc < 3)
	{
		ERROR("incomplete target_script command");
		exit(-1);
	}
	
	target = get_target_by_num(strtoul(args[0], NULL, 0));
	
	if (!target)
	{
		ERROR("target number '%s' not defined", args[0]);
		exit(-1);
	}
	
	if (strcmp(args[1], "reset") == 0)
	{
		if (target->reset_script)
			free(target->reset_script);
		target->reset_script = strdup(args[2]);
	}
	else if (strcmp(args[1], "post_halt") == 0)
	{
		if (target->post_halt_script)
			free(target->post_halt_script);
		target->post_halt_script = strdup(args[2]);
	}
	else if (strcmp(args[1], "pre_resume") == 0)
	{
		if (target->pre_resume_script)
			free(target->pre_resume_script);
		target->pre_resume_script = strdup(args[2]);
	}
	else
	{
		ERROR("unknown event type: '%s", args[1]);
		exit(-1);	
	}
	
	return ERROR_OK;
}

int handle_run_and_halt_time_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = NULL;
	
	if (argc < 2)
	{
		ERROR("incomplete run_and_halt_time command");
		exit(-1);
	}
	
	target = get_target_by_num(strtoul(args[0], NULL, 0));
	
	if (!target)
	{
		ERROR("target number '%s' not defined", args[0]);
		exit(-1);
	}
	
	target->run_and_halt_time = strtoul(args[1], NULL, 0);
	
	return ERROR_OK;
}

int handle_working_area_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = NULL;
	
	if (argc < 4)
	{
		ERROR("incomplete working_area command. usage: working_area <target#> <address> <size> <'backup'|'nobackup'>");
		exit(-1);
	}
	
	target = get_target_by_num(strtoul(args[0], NULL, 0));
	
	if (!target)
	{
		ERROR("target number '%s' not defined", args[0]);
		exit(-1);
	}
	
	target->working_area = strtoul(args[1], NULL, 0);
	target->working_area_size = strtoul(args[2], NULL, 0);
	
	if (strcmp(args[3], "backup") == 0)
	{
		target->backup_working_area = 1;
	}
	else if (strcmp(args[3], "nobackup") == 0)
	{
		target->backup_working_area = 0;
	}
	else
	{
		ERROR("unrecognized <backup|nobackup> argument (%s)", args[3]);
		exit(-1);
	}
	
	return ERROR_OK;
}


/* process target state changes */
int handle_target(void *priv)
{
	int retval;
	target_t *target = targets;
	
	while (target)
	{
		/* only poll if target isn't already halted */
		if (target->state != TARGET_HALTED)
		{
			if (target_continous_poll)
				if ((retval = target->type->poll(target)) < 0)
				{
					ERROR("couldn't poll target. It's due for a reset.");
				}
		}
	
		target = target->next;
	}
	
	return ERROR_OK;
}

int handle_reg_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target;
	reg_t *reg = NULL;
	int count = 0;
	char *value;
	
	DEBUG("-");
	
	target = get_current_target(cmd_ctx);
	
	/* list all available registers for the current target */
	if (argc == 0)
	{
		reg_cache_t *cache = target->reg_cache;
		
		count = 0;
		while(cache)
		{
			int i;
			for (i = 0; i < cache->num_regs; i++)
			{
				value = buf_to_str(cache->reg_list[i].value, cache->reg_list[i].size, 16);
				command_print(cmd_ctx, "(%i) %s (/%i): 0x%s (dirty: %i, valid: %i)", count++, cache->reg_list[i].name, cache->reg_list[i].size, value, cache->reg_list[i].dirty, cache->reg_list[i].valid);
				free(value);
			}
			cache = cache->next;
		}
		
		return ERROR_OK;
	}
	
	/* access a single register by its ordinal number */
	if ((args[0][0] >= '0') && (args[0][0] <= '9'))
	{
		int num = strtoul(args[0], NULL, 0);
		reg_cache_t *cache = target->reg_cache;
		
		count = 0;
		while(cache)
		{
			int i;
			for (i = 0; i < cache->num_regs; i++)
			{
				if (count++ == num)
				{
					reg = &cache->reg_list[i];
					break;
				}
			}
			if (reg)
				break;
			cache = cache->next;
		}
		
		if (!reg)
		{
			command_print(cmd_ctx, "%i is out of bounds, the current target has only %i registers (0 - %i)", num, count, count - 1);
			return ERROR_OK;
		}
	} else /* access a single register by its name */
	{
		reg = register_get_by_name(target->reg_cache, args[0], 1);
		
		if (!reg)
		{
			command_print(cmd_ctx, "register %s not found in current target", args[0]);
			return ERROR_OK;
		}
	}

	/* display a register */
	if ((argc == 1) || ((argc == 2) && !((args[1][0] >= '0') && (args[1][0] <= '9'))))
	{
		if ((argc == 2) && (strcmp(args[1], "force") == 0))
			reg->valid = 0;
		
		if (reg->valid == 0)
		{
			reg_arch_type_t *arch_type = register_get_arch_type(reg->arch_type);
			if (arch_type == NULL)
			{
				ERROR("BUG: encountered unregistered arch type");
				return ERROR_OK;
			}
			arch_type->get(reg);
		}
		value = buf_to_str(reg->value, reg->size, 16);
		command_print(cmd_ctx, "%s (/%i): 0x%s", reg->name, reg->size, value);
		free(value);
		return ERROR_OK;
	}
	
	/* set register value */
	if (argc == 2)
	{
		u8 *buf = malloc(CEIL(reg->size, 8));
		str_to_buf(args[1], strlen(args[1]), buf, reg->size, 0);

		reg_arch_type_t *arch_type = register_get_arch_type(reg->arch_type);
		if (arch_type == NULL)
		{
			ERROR("BUG: encountered unregistered arch type");
			return ERROR_OK;
		}
		
		arch_type->set(reg, buf);
		
		value = buf_to_str(reg->value, reg->size, 16);
		command_print(cmd_ctx, "%s (/%i): 0x%s", reg->name, reg->size, value);
		free(value);
		
		free(buf);
		
		return ERROR_OK;
	}
	
	command_print(cmd_ctx, "usage: reg <#|name> [value]");
	
	return ERROR_OK;
}

int handle_poll_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	char buffer[512];

	if (argc == 0)
	{
		command_print(cmd_ctx, "target state: %s", target_state_strings[target->type->poll(target)]);
		if (target->state == TARGET_HALTED)
		{
			target->type->arch_state(target, buffer, 512);
			buffer[511] = 0;
			command_print(cmd_ctx, "%s", buffer);
		}
	}
	else
	{
		if (strcmp(args[0], "on") == 0)
		{
			target_continous_poll = 1;
		}
		else if (strcmp(args[0], "off") == 0)
		{
			target_continous_poll = 0;
		}
	}
	
	
	return ERROR_OK;
}

int handle_wait_halt_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	struct timeval timeout, now;
	
	gettimeofday(&timeout, NULL);
	if (!argc)
		timeval_add_time(&timeout, 5, 0);
	else {
		char *end;

		timeval_add_time(&timeout, strtoul(args[0], &end, 0), 0);
		if (*end) {
			command_print(cmd_ctx, "usage: wait_halt [seconds]");
			return ERROR_OK;
		}
	}

	command_print(cmd_ctx, "waiting for target halted...");

	while(target->type->poll(target))
	{
		if (target->state == TARGET_HALTED)
		{
			command_print(cmd_ctx, "target halted");
			break;
		}
		target_call_timer_callbacks();
		
		gettimeofday(&now, NULL);
		if ((now.tv_sec >= timeout.tv_sec) && (now.tv_usec >= timeout.tv_usec))
		{
			command_print(cmd_ctx, "timed out while waiting for target halt");
			ERROR("timed out while waiting for target halt");
			break;
		}
	}
	
	return ERROR_OK;
}

int handle_halt_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int retval;
	target_t *target = get_current_target(cmd_ctx);

	DEBUG("-");
	
	command_print(cmd_ctx, "requesting target halt...");

	if ((retval = target->type->halt(target)) != ERROR_OK)
	{	
		switch (retval)
		{
			case ERROR_TARGET_ALREADY_HALTED:
				command_print(cmd_ctx, "target already halted");
				break;
			case ERROR_TARGET_TIMEOUT:
				command_print(cmd_ctx, "target timed out... shutting down");
				exit(-1);
			default:
				command_print(cmd_ctx, "unknown error... shutting down");
				exit(-1);
		}
	}
	
	return ERROR_OK;

}

/* what to do on daemon startup */
int handle_daemon_startup_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc == 1)
	{
		if (strcmp(args[0], "attach") == 0)
		{
			startup_mode = DAEMON_ATTACH;
			return ERROR_OK;
		}
		else if (strcmp(args[0], "reset") == 0)
		{
			startup_mode = DAEMON_RESET;
			return ERROR_OK;
		}
	}
	
	WARNING("invalid daemon_startup configuration directive: %s", args[0]);
	return ERROR_OK;

}
		
int handle_soft_reset_halt_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	int retval;
	
	command_print(cmd_ctx, "requesting target halt and executing a soft reset");
	
	if ((retval = target->type->soft_reset_halt(target)) != ERROR_OK)
	{	
		switch (retval)
		{
			case ERROR_TARGET_TIMEOUT:
				command_print(cmd_ctx, "target timed out... shutting down");
				exit(-1);
			default:
				command_print(cmd_ctx, "unknown error... shutting down");
				exit(-1);
		}
	}
	
	return ERROR_OK;
}

int handle_reset_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	enum target_reset_mode reset_mode = RESET_RUN;
	
	DEBUG("-");
	
	if (argc >= 1)
	{
		if (strcmp("run", args[0]) == 0)
			reset_mode = RESET_RUN;
		else if (strcmp("halt", args[0]) == 0)
			reset_mode = RESET_HALT;
		else if (strcmp("init", args[0]) == 0)
			reset_mode = RESET_INIT;
		else if (strcmp("run_and_halt", args[0]) == 0)
		{
			reset_mode = RESET_RUN_AND_HALT;
			if (argc >= 2)
			{
				target->run_and_halt_time = strtoul(args[1], NULL, 0);
			}
		}
		else if (strcmp("run_and_init", args[0]) == 0)
		{
			reset_mode = RESET_RUN_AND_INIT;
			if (argc >= 2)
			{
				target->run_and_halt_time = strtoul(args[1], NULL, 0);
			}
		}
		else
		{
			command_print(cmd_ctx, "usage: reset ['run', 'halt', 'init', 'run_and_halt', 'run_and_init]");
			return ERROR_OK;
		}
		target->reset_mode = reset_mode;
	}
	
	target_process_reset(cmd_ctx);
	
	return ERROR_OK;
}

int handle_resume_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int retval;
	target_t *target = get_current_target(cmd_ctx);
	
	DEBUG("-");
	
	if (argc == 0)
		retval = target->type->resume(target, 1, 0, 1, 0); /* current pc, addr = 0, handle breakpoints, not debugging */
	else if (argc == 1)
		retval = target->type->resume(target, 0, strtoul(args[0], NULL, 0), 1, 0); /* addr = args[0], handle breakpoints, not debugging */
	else
	{
		command_print(cmd_ctx, "usage: resume [address]");
		return ERROR_OK;
	}
	
	if (retval != ERROR_OK)
	{	
		switch (retval)
		{
			case ERROR_TARGET_NOT_HALTED:
				command_print(cmd_ctx, "target not halted");
				break;
			default:
				command_print(cmd_ctx, "unknown error... shutting down");
				exit(-1);
		}
	}

	return ERROR_OK;
}

int handle_step_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	
	DEBUG("-");
	
	if (argc == 0)
		target->type->step(target, 1, 0, 1); /* current pc, addr = 0, handle breakpoints */

	if (argc == 1)
		target->type->step(target, 0, strtoul(args[0], NULL, 0), 1); /* addr = args[0], handle breakpoints */
	
	return ERROR_OK;
}

int handle_md_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int count = 1;
	int size = 4;
	u32 address = 0;
	int i;

	char output[128];
	int output_len;

	int retval;

	u8 *buffer;
	target_t *target = get_current_target(cmd_ctx);

	if (argc < 1)
		return ERROR_OK;

	if (argc == 2)
		count = strtoul(args[1], NULL, 0);

	address = strtoul(args[0], NULL, 0);
	

	switch (cmd[2])
	{
		case 'w':
			size = 4;
			break;
		case 'h':
			size = 2;
			break;
		case 'b':
			size = 1;
			break;
		default:
			return ERROR_OK;
	}

	buffer = calloc(count, size);
	if ((retval  = target->type->read_memory(target, address, size, count, buffer)) != ERROR_OK)
	{
		switch (retval)
		{
			case ERROR_TARGET_UNALIGNED_ACCESS:
				command_print(cmd_ctx, "error: address not aligned");
				break;
			case ERROR_TARGET_NOT_HALTED:
				command_print(cmd_ctx, "error: target must be halted for memory accesses");
				break;			
			case ERROR_TARGET_DATA_ABORT:
				command_print(cmd_ctx, "error: access caused data abort, system possibly corrupted");
				break;
			default:
				command_print(cmd_ctx, "error: unknown error");
				break;
		}
		return ERROR_OK;
	}

	output_len = 0;

	for (i = 0; i < count; i++)
	{
		if (i%8 == 0)
			output_len += snprintf(output + output_len, 128 - output_len, "0x%8.8x: ", address + (i*size));
		
		switch (size)
		{
			case 4:
				output_len += snprintf(output + output_len, 128 - output_len, "%8.8x ", target_buffer_get_u32(target, &buffer[i*4]));
				break;
			case 2:
				output_len += snprintf(output + output_len, 128 - output_len, "%4.4x ", target_buffer_get_u16(target, &buffer[i*2]));
				break;
			case 1:
				output_len += snprintf(output + output_len, 128 - output_len, "%2.2x ", buffer[i*1]);
				break;
		}

		if ((i%8 == 7) || (i == count - 1))
		{
			command_print(cmd_ctx, output);
			output_len = 0;
		}
	}

	free(buffer);
	
	return ERROR_OK;
}

int handle_mw_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	u32 address = 0;
	u32 value = 0;
	int retval;
	target_t *target = get_current_target(cmd_ctx);
	u8 value_buf[4];

	if (argc < 2)
		return ERROR_OK;

	address = strtoul(args[0], NULL, 0);
	value = strtoul(args[1], NULL, 0);

	switch (cmd[2])
	{
		case 'w':
			target_buffer_set_u32(target, value_buf, value);
			retval = target->type->write_memory(target, address, 4, 1, value_buf);
			break;
		case 'h':
			target_buffer_set_u16(target, value_buf, value);
			retval = target->type->write_memory(target, address, 2, 1, value_buf);
			break;
		case 'b':
			value_buf[0] = value;
			retval = target->type->write_memory(target, address, 1, 1, value_buf);
			break;
		default:
			return ERROR_OK;
	}

	switch (retval)
	{
		case ERROR_TARGET_UNALIGNED_ACCESS:
			command_print(cmd_ctx, "error: address not aligned");
			break;
		case ERROR_TARGET_DATA_ABORT:
			command_print(cmd_ctx, "error: access caused data abort, system possibly corrupted");
			break;
		case ERROR_TARGET_NOT_HALTED:
			command_print(cmd_ctx, "error: target must be halted for memory accesses");
			break;
		case ERROR_OK:
			break;
		default:
			command_print(cmd_ctx, "error: unknown error");
			break;
	}

	return ERROR_OK;

}

int handle_load_image_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	u8 *buffer;
	u32 buf_cnt;
	u32 image_size;
	int i;
	int retval;

	image_t image;	
	
	duration_t duration;
	char *duration_text;
	
	target_t *target = get_current_target(cmd_ctx);

	if (argc < 1)
	{
		command_print(cmd_ctx, "usage: load_image <filename> [address] [type]");
		return ERROR_OK;
	}
	
	/* a base address isn't always necessary, default to 0x0 (i.e. don't relocate) */
	if (argc >= 2)
	{
		image.base_address_set = 1;
		image.base_address = strtoul(args[1], NULL, 0);
	}
	else
	{
		image.base_address_set = 0;
	}
	
	image.start_address_set = 0;

	duration_start_measure(&duration);
	
	if (image_open(&image, args[0], (argc >= 3) ? args[2] : NULL) != ERROR_OK)
	{
		command_print(cmd_ctx, "load_image error: %s", image.error_str);
		return ERROR_OK;
	}
	
	image_size = 0x0;
	for (i = 0; i < image.num_sections; i++)
	{
		buffer = malloc(image.sections[i].size);
		if ((retval = image_read_section(&image, i, 0x0, image.sections[i].size, buffer, &buf_cnt)) != ERROR_OK)
		{
			ERROR("image_read_section failed with error code: %i", retval);
			command_print(cmd_ctx, "image reading failed, download aborted");
			free(buffer);
			image_close(&image);
			return ERROR_OK;
		}
		target_write_buffer(target, image.sections[i].base_address, buf_cnt, buffer);
		image_size += buf_cnt;
		command_print(cmd_ctx, "%u byte written at address 0x%8.8x", buf_cnt, image.sections[i].base_address);
		
		free(buffer);
	}

	duration_stop_measure(&duration, &duration_text);
	command_print(cmd_ctx, "downloaded %u byte in %s", image_size, duration_text);
	free(duration_text);
	
	image_close(&image);

	return ERROR_OK;

}

int handle_dump_image_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	fileio_t fileio;
	
	u32 address;
	u32 size;
	u8 buffer[560];
	
	duration_t duration;
	char *duration_text;
	
	target_t *target = get_current_target(cmd_ctx);

	if (argc != 3)
	{
		command_print(cmd_ctx, "usage: dump_image <filename> <address> <size>");
		return ERROR_OK;
	}

	address = strtoul(args[1], NULL, 0);
	size = strtoul(args[2], NULL, 0);

	if ((address & 3) || (size & 3))
	{
		command_print(cmd_ctx, "only 32-bit aligned address and size are supported");
		return ERROR_OK;
	}
	
	if (fileio_open(&fileio, args[0], FILEIO_WRITE, FILEIO_BINARY) != ERROR_OK)
	{
		command_print(cmd_ctx, "dump_image error: %s", fileio.error_str);
		return ERROR_OK;
	}
	
	duration_start_measure(&duration);
	
	while (size > 0)
	{
		u32 size_written;
		u32 this_run_size = (size > 560) ? 560 : size;
		
		target->type->read_memory(target, address, 4, this_run_size / 4, buffer);
		fileio_write(&fileio, this_run_size, buffer, &size_written);
		
		size -= this_run_size;
		address += this_run_size;
	}

	fileio_close(&fileio);

	duration_stop_measure(&duration, &duration_text);
	command_print(cmd_ctx, "dumped %"PRIi64" byte in %s", fileio.size, duration_text);
	free(duration_text);
	
	return ERROR_OK;

}

int handle_verify_image_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	u8 *buffer;
	u32 buf_cnt;
	u32 image_size;
	int i;
	int retval;
	u32 checksum = 0;
	u32 mem_checksum = 0;

	image_t image;	
	
	duration_t duration;
	char *duration_text;
	
	target_t *target = get_current_target(cmd_ctx);
	
	if (argc < 1)
	{
		command_print(cmd_ctx, "usage: verify_image <file> [offset] [type]");
		return ERROR_OK;
	}
	
	if (!target)
	{
		ERROR("no target selected");
	return ERROR_OK;
	}
	
	duration_start_measure(&duration);
	
	if (argc >= 2)
	{
		image.base_address_set = 1;
		image.base_address = strtoul(args[1], NULL, 0);
	}
	else
	{
		image.base_address_set = 0;
		image.base_address = 0x0;
	}

	image.start_address_set = 0;

	if (image_open(&image, args[0], (argc == 3) ? args[2] : NULL) != ERROR_OK)
	{
		command_print(cmd_ctx, "verify_image error: %s", image.error_str);
		return ERROR_OK;
	}
	
	image_size = 0x0;
	for (i = 0; i < image.num_sections; i++)
	{
		buffer = malloc(image.sections[i].size);
		if ((retval = image_read_section(&image, i, 0x0, image.sections[i].size, buffer, &buf_cnt)) != ERROR_OK)
		{
			ERROR("image_read_section failed with error code: %i", retval);
			command_print(cmd_ctx, "image reading failed, verify aborted");
			free(buffer);
			image_close(&image);
			return ERROR_OK;
		}
		
		/* calculate checksum of image */
		image_calculate_checksum( buffer, buf_cnt, &checksum );
		free(buffer);
		
		retval = target_checksum_memory(target, image.sections[i].base_address, buf_cnt, &mem_checksum);
		
		if( retval != ERROR_OK )
		{
			command_print(cmd_ctx, "image verify failed, verify aborted");
			image_close(&image);
			return ERROR_OK;
		}
		
		if( checksum != mem_checksum )
		{
			command_print(cmd_ctx, "image verify failed, verify aborted");
			image_close(&image);
			return ERROR_OK;
		}
			
		image_size += buf_cnt;
	}

	duration_stop_measure(&duration, &duration_text);
	command_print(cmd_ctx, "verified %u bytes in %s", image_size, duration_text);
	free(duration_text);
	
	image_close(&image);
	
	return ERROR_OK;
}

int handle_bp_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int retval;
	target_t *target = get_current_target(cmd_ctx);

	if (argc == 0)
	{
		breakpoint_t *breakpoint = target->breakpoints;

		while (breakpoint)
		{
			if (breakpoint->type == BKPT_SOFT)
			{
				char* buf = buf_to_str(breakpoint->orig_instr, breakpoint->length, 16);
				command_print(cmd_ctx, "0x%8.8x, 0x%x, %i, 0x%s", breakpoint->address, breakpoint->length, breakpoint->set, buf);
				free(buf);
			}
			else
			{
				command_print(cmd_ctx, "0x%8.8x, 0x%x, %i", breakpoint->address, breakpoint->length, breakpoint->set);
			}
			breakpoint = breakpoint->next;
		}
	}
	else if (argc >= 2)
	{
		int hw = BKPT_SOFT;
		u32 length = 0;

		length = strtoul(args[1], NULL, 0);
		
		if (argc >= 3)
			if (strcmp(args[2], "hw") == 0)
				hw = BKPT_HARD;

		if ((retval = breakpoint_add(target, strtoul(args[0], NULL, 0), length, hw)) != ERROR_OK)
		{
			switch (retval)
			{
				case ERROR_TARGET_NOT_HALTED:
					command_print(cmd_ctx, "target must be halted to set breakpoints");
					break;
				case ERROR_TARGET_RESOURCE_NOT_AVAILABLE:
					command_print(cmd_ctx, "no more breakpoints available");
					break;
				default:
					command_print(cmd_ctx, "unknown error, breakpoint not set");
					break;
			}
		}
		else
		{
			command_print(cmd_ctx, "breakpoint added at address 0x%8.8x", strtoul(args[0], NULL, 0));
		}
	}
	else
	{
		command_print(cmd_ctx, "usage: bp <address> <length> ['hw']");
	}

	return ERROR_OK;
}

int handle_rbp_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);

	if (argc > 0)
		breakpoint_remove(target, strtoul(args[0], NULL, 0));

	return ERROR_OK;
}

int handle_wp_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	int retval;

	if (argc == 0)
	{
		watchpoint_t *watchpoint = target->watchpoints;

		while (watchpoint)
		{
			command_print(cmd_ctx, "address: 0x%8.8x, mask: 0x%8.8x, r/w/a: %i, value: 0x%8.8x, mask: 0x%8.8x", watchpoint->address, watchpoint->length, watchpoint->rw, watchpoint->value, watchpoint->mask);
			watchpoint = watchpoint->next;
		}
	} 
	else if (argc >= 2)
	{
		enum watchpoint_rw type = WPT_ACCESS;
		u32 data_value = 0x0;
		u32 data_mask = 0xffffffff;
		
		if (argc >= 3)
		{
			switch(args[2][0])
			{
				case 'r':
					type = WPT_READ;
					break;
				case 'w':
					type = WPT_WRITE;
					break;
				case 'a':
					type = WPT_ACCESS;
					break;
				default:
					command_print(cmd_ctx, "usage: wp <address> <length> [r/w/a] [value] [mask]");
					return ERROR_OK;
			}
		}
		if (argc >= 4)
		{
			data_value = strtoul(args[3], NULL, 0);
		}
		if (argc >= 5)
		{
			data_mask = strtoul(args[4], NULL, 0);
		}
		
		if ((retval = watchpoint_add(target, strtoul(args[0], NULL, 0),
				strtoul(args[1], NULL, 0), type, data_value, data_mask)) != ERROR_OK)
		{
			switch (retval)
			{
				case ERROR_TARGET_NOT_HALTED:
					command_print(cmd_ctx, "target must be halted to set watchpoints");
					break;
				case ERROR_TARGET_RESOURCE_NOT_AVAILABLE:
					command_print(cmd_ctx, "no more watchpoints available");
					break;
				default:
					command_print(cmd_ctx, "unknown error, watchpoint not set");
					break;
			}	
		}
	}
	else
	{
		command_print(cmd_ctx, "usage: wp <address> <length> [r/w/a] [value] [mask]");
	}
		
	return ERROR_OK;
}

int handle_rwp_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);

	if (argc > 0)
		watchpoint_remove(target, strtoul(args[0], NULL, 0));
	
	return ERROR_OK;
}


