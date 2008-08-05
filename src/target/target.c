/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                      *
 *   oyvind.harboe@zylin.com                                               *
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
int handle_targets_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

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
int handle_virt2phys_command(command_context_t *cmd_ctx, char *cmd, char **args, int argc);
int handle_profile_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int jim_array2mem(Jim_Interp *interp, int argc, Jim_Obj *const *argv);
static int jim_mem2array(Jim_Interp *interp, int argc, Jim_Obj *const *argv);


/* targets */
extern target_type_t arm7tdmi_target;
extern target_type_t arm720t_target;
extern target_type_t arm9tdmi_target;
extern target_type_t arm920t_target;
extern target_type_t arm966e_target;
extern target_type_t arm926ejs_target;
extern target_type_t feroceon_target;
extern target_type_t xscale_target;
extern target_type_t cortexm3_target;
extern target_type_t arm11_target;
extern target_type_t mips_m4k_target;

target_type_t *target_types[] =
{
	&arm7tdmi_target,
	&arm9tdmi_target,
	&arm920t_target,
	&arm720t_target,
	&arm966e_target,
	&arm926ejs_target,
	&feroceon_target,
	&xscale_target,
	&cortexm3_target,
	&arm11_target,
	&mips_m4k_target,
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
	"target not halted", "undefined"
};

char *target_endianess_strings[] =
{
	"big endian",
	"little endian",
};

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
		LOG_ERROR("BUG: current_target out of bounds");
		exit(-1);
	}

	return target;
}


int target_poll(struct target_s *target)
{
	/* We can't poll until after examine */
	if (!target->type->examined)
	{
		/* Fail silently lest we pollute the log */
		return ERROR_FAIL;
	}
	return target->type->poll(target);
}

int target_halt(struct target_s *target)
{
	/* We can't poll until after examine */
	if (!target->type->examined)
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}
	return target->type->halt(target);
}

int target_resume(struct target_s *target, int current, u32 address, int handle_breakpoints, int debug_execution)
{
	int retval;

	/* We can't poll until after examine */
	if (!target->type->examined)
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}

	/* note that resume *must* be asynchronous. The CPU can halt before we poll. The CPU can
	 * even halt at the current PC as a result of a software breakpoint being inserted by (a bug?)
	 * the application.
	 */
	if ((retval = target->type->resume(target, current, address, handle_breakpoints, debug_execution)) != ERROR_OK)
		return retval;

	return retval;
}

int target_process_reset(struct command_context_s *cmd_ctx, enum target_reset_mode reset_mode)
{
	int retval = ERROR_OK;
	target_t *target;

	target = targets;
	while (target)
	{
		target_invoke_script(cmd_ctx, target, "pre_reset");
		target = target->next;
	}

	if ((retval = jtag_init_reset(cmd_ctx)) != ERROR_OK)
		return retval;

	keep_alive(); /* we might be running on a very slow JTAG clk */

	/* First time this is executed after launching OpenOCD, it will read out
	 * the type of CPU, etc. and init Embedded ICE registers in host
	 * memory.
	 *
	 * It will also set up ICE registers in the target.
	 *
	 * However, if we assert TRST later, we need to set up the registers again.
	 *
	 * For the "reset halt/init" case we must only set up the registers here.
	 */
	if ((retval = target_examine()) != ERROR_OK)
		return retval;

	keep_alive(); /* we might be running on a very slow JTAG clk */

	target = targets;
	while (target)
	{
		/* we have no idea what state the target is in, so we
		 * have to drop working areas
		 */
		target_free_all_working_areas_restore(target, 0);
		target->reset_halt=((reset_mode==RESET_HALT)||(reset_mode==RESET_INIT));
		if ((retval = target->type->assert_reset(target))!=ERROR_OK)
			return retval;
		target = target->next;
	}

	target = targets;
	while (target)
	{
		if ((retval = target->type->deassert_reset(target))!=ERROR_OK)
			return retval;
		target = target->next;
	}

	target = targets;
	while (target)
	{
		/* We can fail to bring the target into the halted state, try after reset has been deasserted  */
		if (target->reset_halt)
		{
			/* wait up to 1 second for halt. */
			target_wait_state(target, TARGET_HALTED, 1000);
			if (target->state != TARGET_HALTED)
			{
				LOG_WARNING("Failed to reset target into halted mode - issuing halt");
				if ((retval = target->type->halt(target))!=ERROR_OK)
					return retval;
			}
		}

		target = target->next;
	}


	LOG_DEBUG("Waiting for halted stated as appropriate");

	if ((reset_mode == RESET_HALT) || (reset_mode == RESET_INIT))
	{
		target = targets;
		while (target)
		{
			/* Wait for reset to complete, maximum 5 seconds. */
			if (((retval=target_wait_state(target, TARGET_HALTED, 5000)))==ERROR_OK)
			{
				if (reset_mode == RESET_INIT)
					target_invoke_script(cmd_ctx, target, "post_reset");
			}
			target = target->next;
		}
	}

	/* We want any events to be processed before the prompt */
	target_call_timer_callbacks_now();

	return retval;
}

static int default_virt2phys(struct target_s *target, u32 virtual, u32 *physical)
{
	*physical = virtual;
	return ERROR_OK;
}

static int default_mmu(struct target_s *target, int *enabled)
{
	*enabled = 0;
	return ERROR_OK;
}

static int default_examine(struct target_s *target)
{
	target->type->examined = 1;
	return ERROR_OK;
}


/* Targets that correctly implement init+examine, i.e.
 * no communication with target during init:
 *
 * XScale
 */
int target_examine(struct command_context_s *cmd_ctx)
{
	int retval = ERROR_OK;
	target_t *target = targets;
	while (target)
	{
		if ((retval = target->type->examine(target))!=ERROR_OK)
			return retval;
		target = target->next;
	}
	return retval;
}

static int target_write_memory_imp(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer)
{
	if (!target->type->examined)
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}
	return target->type->write_memory_imp(target, address, size, count, buffer);
}

static int target_read_memory_imp(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer)
{
	if (!target->type->examined)
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}
	return target->type->read_memory_imp(target, address, size, count, buffer);
}

static int target_soft_reset_halt_imp(struct target_s *target)
{
	if (!target->type->examined)
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}
	return target->type->soft_reset_halt_imp(target);
}

static int target_run_algorithm_imp(struct target_s *target, int num_mem_params, mem_param_t *mem_params, int num_reg_params, reg_param_t *reg_param, u32 entry_point, u32 exit_point, int timeout_ms, void *arch_info)
{
	if (!target->type->examined)
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}
	return target->type->run_algorithm_imp(target, num_mem_params, mem_params, num_reg_params, reg_param, entry_point, exit_point, timeout_ms, arch_info);
}

int target_init(struct command_context_s *cmd_ctx)
{
	target_t *target = targets;

	while (target)
	{
		target->type->examined = 0;
		if (target->type->examine == NULL)
		{
			target->type->examine = default_examine;
		}

		if (target->type->init_target(cmd_ctx, target) != ERROR_OK)
		{
			LOG_ERROR("target '%s' init failed", target->type->name);
			exit(-1);
		}

		/* Set up default functions if none are provided by target */
		if (target->type->virt2phys == NULL)
		{
			target->type->virt2phys = default_virt2phys;
		}
		target->type->virt2phys = default_virt2phys;
		/* a non-invasive way(in terms of patches) to add some code that
		 * runs before the type->write/read_memory implementation
		 */
		target->type->write_memory_imp = target->type->write_memory;
		target->type->write_memory = target_write_memory_imp;
		target->type->read_memory_imp = target->type->read_memory;
		target->type->read_memory = target_read_memory_imp;
		target->type->soft_reset_halt_imp = target->type->soft_reset_halt;
		target->type->soft_reset_halt = target_soft_reset_halt_imp;
		target->type->run_algorithm_imp = target->type->run_algorithm;
		target->type->run_algorithm = target_run_algorithm_imp;


		if (target->type->mmu == NULL)
		{
			target->type->mmu = default_mmu;
		}
		target = target->next;
	}

	if (targets)
	{
		target_register_user_commands(cmd_ctx);
		target_register_timer_callback(handle_target, 100, 1, NULL);
	}

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

	LOG_DEBUG("target event %i", event);

	while (callback)
	{
		next_callback = callback->next;
		callback->callback(target, event, callback->priv);
		callback = next_callback;
	}

	return ERROR_OK;
}

static int target_call_timer_callbacks_check_time(int checktime)
{
	target_timer_callback_t *callback = target_timer_callbacks;
	target_timer_callback_t *next_callback;
	struct timeval now;

	keep_alive();

	gettimeofday(&now, NULL);

	while (callback)
	{
		next_callback = callback->next;

		if ((!checktime&&callback->periodic)||
				(((now.tv_sec >= callback->when.tv_sec) && (now.tv_usec >= callback->when.tv_usec))
						|| (now.tv_sec > callback->when.tv_sec)))
		{
			if(callback->callback != NULL)
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
		}

		callback = next_callback;
	}

	return ERROR_OK;
}

int target_call_timer_callbacks()
{
	return target_call_timer_callbacks_check_time(1);
}

/* invoke periodic callbacks immediately */
int target_call_timer_callbacks_now()
{
	return target_call_timer_callbacks(0);
}

int target_alloc_working_area(struct target_s *target, u32 size, working_area_t **area)
{
	working_area_t *c = target->working_areas;
	working_area_t *new_wa = NULL;

	/* Reevaluate working area address based on MMU state*/
	if (target->working_areas == NULL)
	{
		int retval;
		int enabled;
		retval = target->type->mmu(target, &enabled);
		if (retval != ERROR_OK)
		{
			return retval;
		}
		if (enabled)
		{
			target->working_area = target->working_area_virt;
		}
		else
		{
			target->working_area = target->working_area_phys;
		}
	}

	/* only allocate multiples of 4 byte */
	if (size % 4)
	{
		LOG_ERROR("BUG: code tried to allocate unaligned number of bytes, padding");
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

		LOG_DEBUG("allocating new working area");

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
			LOG_WARNING("not enough working area available(requested %d, free %d)", size, free_size);
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

int target_free_working_area_restore(struct target_s *target, working_area_t *area, int restore)
{
	if (area->free)
		return ERROR_OK;

	if (restore&&target->backup_working_area)
		target->type->write_memory(target, area->address, 4, area->size / 4, area->backup);

	area->free = 1;

	/* mark user pointer invalid */
	*area->user = NULL;
	area->user = NULL;

	return ERROR_OK;
}

int target_free_working_area(struct target_s *target, working_area_t *area)
{
	return target_free_working_area_restore(target, area, 1);
}

int target_free_all_working_areas_restore(struct target_s *target, int restore)
{
	working_area_t *c = target->working_areas;

	while (c)
	{
		working_area_t *next = c->next;
		target_free_working_area_restore(target, c, restore);

		if (c->backup)
			free(c->backup);

		free(c);

		c = next;
	}

	target->working_areas = NULL;

	return ERROR_OK;
}

int target_free_all_working_areas(struct target_s *target)
{
	return target_free_all_working_areas_restore(target, 1);
}

int target_register_commands(struct command_context_s *cmd_ctx)
{
	register_command(cmd_ctx, NULL, "target", handle_target_command, COMMAND_CONFIG, "target <cpu> [reset_init default - DEPRECATED] <chainpos> <endianness> <variant> [cpu type specifc args]");
	register_command(cmd_ctx, NULL, "targets", handle_targets_command, COMMAND_EXEC, NULL);
	register_command(cmd_ctx, NULL, "working_area", handle_working_area_command, COMMAND_ANY, "working_area <target#> <address> <size> <'backup'|'nobackup'> [virtual address]");
	register_command(cmd_ctx, NULL, "virt2phys", handle_virt2phys_command, COMMAND_ANY, "virt2phys <virtual address>");
	register_command(cmd_ctx, NULL, "profile", handle_profile_command, COMMAND_EXEC, "PRELIMINARY! - profile <seconds> <gmon.out>");


	/* script procedures */
	register_jim(cmd_ctx, "ocd_mem2array", jim_mem2array, "read memory and return as a TCL array for script processing");
	register_jim(cmd_ctx, "ocd_array2mem", jim_array2mem, "convert a TCL array to memory locations and write the values");
	return ERROR_OK;
}

int target_arch_state(struct target_s *target)
{
	int retval;
	if (target==NULL)
	{
		LOG_USER("No target has been configured");
		return ERROR_OK;
	}

	LOG_USER("target state: %s", target_state_strings[target->state]);

	if (target->state!=TARGET_HALTED)
		return ERROR_OK;

	retval=target->type->arch_state(target);
	return retval;
}

/* Single aligned words are guaranteed to use 16 or 32 bit access
 * mode respectively, otherwise data is handled as quickly as
 * possible
 */
int target_write_buffer(struct target_s *target, u32 address, u32 size, u8 *buffer)
{
	int retval;
	LOG_DEBUG("writing buffer of %i byte at 0x%8.8x", size, address);

	if (!target->type->examined)
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}

	if (address+size<address)
	{
		/* GDB can request this when e.g. PC is 0xfffffffc*/
		LOG_ERROR("address+size wrapped(0x%08x, 0x%08x)", address, size);
		return ERROR_FAIL;
	}

	if (((address % 2) == 0) && (size == 2))
	{
		return target->type->write_memory(target, address, 2, 1, buffer);
	}

	/* handle unaligned head bytes */
	if (address % 4)
	{
		int unaligned = 4 - (address % 4);

		if (unaligned > size)
			unaligned = size;

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


/* Single aligned words are guaranteed to use 16 or 32 bit access
 * mode respectively, otherwise data is handled as quickly as
 * possible
 */
int target_read_buffer(struct target_s *target, u32 address, u32 size, u8 *buffer)
{
	int retval;
	LOG_DEBUG("reading buffer of %i byte at 0x%8.8x", size, address);

	if (!target->type->examined)
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}

	if (address+size<address)
	{
		/* GDB can request this when e.g. PC is 0xfffffffc*/
		LOG_ERROR("address+size wrapped(0x%08x, 0x%08x)", address, size);
		return ERROR_FAIL;
	}

	if (((address % 2) == 0) && (size == 2))
	{
		return target->type->read_memory(target, address, 2, 1, buffer);
	}

	/* handle unaligned head bytes */
	if (address % 4)
	{
		int unaligned = 4 - (address % 4);

		if (unaligned > size)
			unaligned = size;

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
	if (!target->type->examined)
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}

	if ((retval = target->type->checksum_memory(target, address,
		size, &checksum)) == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
	{
		buffer = malloc(size);
		if (buffer == NULL)
		{
			LOG_ERROR("error allocating buffer for section (%d bytes)", size);
			return ERROR_INVALID_ARGUMENTS;
		}
		retval = target_read_buffer(target, address, size, buffer);
		if (retval != ERROR_OK)
		{
			free(buffer);
			return retval;
		}

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

int target_blank_check_memory(struct target_s *target, u32 address, u32 size, u32* blank)
{
	int retval;
	if (!target->type->examined)
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}

	if (target->type->blank_check_memory == 0)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	retval = target->type->blank_check_memory(target, address, size, blank);

	return retval;
}

int target_read_u32(struct target_s *target, u32 address, u32 *value)
{
	u8 value_buf[4];
	if (!target->type->examined)
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}

	int retval = target->type->read_memory(target, address, 4, 1, value_buf);

	if (retval == ERROR_OK)
	{
		*value = target_buffer_get_u32(target, value_buf);
		LOG_DEBUG("address: 0x%8.8x, value: 0x%8.8x", address, *value);
	}
	else
	{
		*value = 0x0;
		LOG_DEBUG("address: 0x%8.8x failed", address);
	}

	return retval;
}

int target_read_u16(struct target_s *target, u32 address, u16 *value)
{
	u8 value_buf[2];
	if (!target->type->examined)
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}

	int retval = target->type->read_memory(target, address, 2, 1, value_buf);

	if (retval == ERROR_OK)
	{
		*value = target_buffer_get_u16(target, value_buf);
		LOG_DEBUG("address: 0x%8.8x, value: 0x%4.4x", address, *value);
	}
	else
	{
		*value = 0x0;
		LOG_DEBUG("address: 0x%8.8x failed", address);
	}

	return retval;
}

int target_read_u8(struct target_s *target, u32 address, u8 *value)
{
	int retval = target->type->read_memory(target, address, 1, 1, value);
	if (!target->type->examined)
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}

	if (retval == ERROR_OK)
	{
		LOG_DEBUG("address: 0x%8.8x, value: 0x%2.2x", address, *value);
	}
	else
	{
		*value = 0x0;
		LOG_DEBUG("address: 0x%8.8x failed", address);
	}

	return retval;
}

int target_write_u32(struct target_s *target, u32 address, u32 value)
{
	int retval;
	u8 value_buf[4];
	if (!target->type->examined)
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}

	LOG_DEBUG("address: 0x%8.8x, value: 0x%8.8x", address, value);

	target_buffer_set_u32(target, value_buf, value);
	if ((retval = target->type->write_memory(target, address, 4, 1, value_buf)) != ERROR_OK)
	{
		LOG_DEBUG("failed: %i", retval);
	}

	return retval;
}

int target_write_u16(struct target_s *target, u32 address, u16 value)
{
	int retval;
	u8 value_buf[2];
	if (!target->type->examined)
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}

	LOG_DEBUG("address: 0x%8.8x, value: 0x%8.8x", address, value);

	target_buffer_set_u16(target, value_buf, value);
	if ((retval = target->type->write_memory(target, address, 2, 1, value_buf)) != ERROR_OK)
	{
		LOG_DEBUG("failed: %i", retval);
	}

	return retval;
}

int target_write_u8(struct target_s *target, u32 address, u8 value)
{
	int retval;
	if (!target->type->examined)
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}

	LOG_DEBUG("address: 0x%8.8x, value: 0x%2.2x", address, value);

	if ((retval = target->type->read_memory(target, address, 1, 1, &value)) != ERROR_OK)
	{
		LOG_DEBUG("failed: %i", retval);
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
	register_command(cmd_ctx,  NULL, "reset", handle_reset_command, COMMAND_EXEC, "reset target [run|halt|init]");
	register_command(cmd_ctx,  NULL, "soft_reset_halt", handle_soft_reset_halt_command, COMMAND_EXEC, "halt the target and do a soft reset");

	register_command(cmd_ctx,  NULL, "mdw", handle_md_command, COMMAND_EXEC, "display memory words <addr> [count]");
	register_command(cmd_ctx,  NULL, "mdh", handle_md_command, COMMAND_EXEC, "display memory half-words <addr> [count]");
	register_command(cmd_ctx,  NULL, "mdb", handle_md_command, COMMAND_EXEC, "display memory bytes <addr> [count]");

	register_command(cmd_ctx,  NULL, "mww", handle_mw_command, COMMAND_EXEC, "write memory word <addr> <value> [count]");
	register_command(cmd_ctx,  NULL, "mwh", handle_mw_command, COMMAND_EXEC, "write memory half-word <addr> <value> [count]");
	register_command(cmd_ctx,  NULL, "mwb", handle_mw_command, COMMAND_EXEC, "write memory byte <addr> <value> [count]");

	register_command(cmd_ctx,  NULL, "bp", handle_bp_command, COMMAND_EXEC, "set breakpoint <address> <length> [hw]");
	register_command(cmd_ctx,  NULL, "rbp", handle_rbp_command, COMMAND_EXEC, "remove breakpoint <adress>");
	register_command(cmd_ctx,  NULL, "wp", handle_wp_command, COMMAND_EXEC, "set watchpoint <address> <length> <r/w/a> [value] [mask]");
	register_command(cmd_ctx,  NULL, "rwp", handle_rwp_command, COMMAND_EXEC, "remove watchpoint <adress>");

	register_command(cmd_ctx,  NULL, "load_image", handle_load_image_command, COMMAND_EXEC, "load_image <file> <address> ['bin'|'ihex'|'elf'|'s19'] [min_address] [max_length]");
	register_command(cmd_ctx,  NULL, "dump_image", handle_dump_image_command, COMMAND_EXEC, "dump_image <file> <address> <size>");
	register_command(cmd_ctx,  NULL, "verify_image", handle_verify_image_command, COMMAND_EXEC, "verify_image <file> [offset] [type]");

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
		return ERROR_COMMAND_SYNTAX_ERROR;
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
					LOG_ERROR("couldn't register '%s' commands", args[0]);
					exit(-1);
				}

				if (*last_target_p)
				{
					while ((*last_target_p)->next)
						last_target_p = &((*last_target_p)->next);
					last_target_p = &((*last_target_p)->next);
				}

				*last_target_p = malloc(sizeof(target_t));

				/* allocate memory for each unique target type */
				(*last_target_p)->type = (target_type_t*)malloc(sizeof(target_type_t));
				*((*last_target_p)->type) = *target_types[i];

				if (strcmp(args[1], "big") == 0)
					(*last_target_p)->endianness = TARGET_BIG_ENDIAN;
				else if (strcmp(args[1], "little") == 0)
					(*last_target_p)->endianness = TARGET_LITTLE_ENDIAN;
				else
				{
					LOG_ERROR("endianness must be either 'little' or 'big', not '%s'", args[1]);
					return ERROR_COMMAND_SYNTAX_ERROR;
				}

				if (strcmp(args[2], "reset_halt") == 0)
				{
					LOG_WARNING("reset_mode argument is obsolete.");
					return ERROR_COMMAND_SYNTAX_ERROR;
				}
				else if (strcmp(args[2], "reset_run") == 0)
				{
					LOG_WARNING("reset_mode argument is obsolete.");
					return ERROR_COMMAND_SYNTAX_ERROR;
				}
				else if (strcmp(args[2], "reset_init") == 0)
				{
					LOG_WARNING("reset_mode argument is obsolete.");
					return ERROR_COMMAND_SYNTAX_ERROR;
				}
				else if (strcmp(args[2], "run_and_halt") == 0)
				{
					LOG_WARNING("reset_mode argument is obsolete.");
					return ERROR_COMMAND_SYNTAX_ERROR;
				}
				else if (strcmp(args[2], "run_and_init") == 0)
				{
					LOG_WARNING("reset_mode argument is obsolete.");
					return ERROR_COMMAND_SYNTAX_ERROR;
				}
				else
				{
					/* Kludge! we want to make this reset arg optional while remaining compatible! */
					args--;
					argc++;
				}

				(*last_target_p)->working_area = 0x0;
				(*last_target_p)->working_area_size = 0x0;
				(*last_target_p)->working_areas = NULL;
				(*last_target_p)->backup_working_area = 0;

				(*last_target_p)->state = TARGET_UNKNOWN;
				(*last_target_p)->debug_reason = DBG_REASON_UNDEFINED;
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
				(*last_target_p)->dbg_msg_enabled = 0;

				(*last_target_p)->type->target_command(cmd_ctx, cmd, args, argc, *last_target_p);

				found = 1;
				break;
			}
		}
	}

	/* no matching target found */
	if (!found)
	{
		LOG_ERROR("target '%s' not found", args[0]);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}

int target_invoke_script(struct command_context_s *cmd_ctx, target_t *target, char *name)
{
	return command_run_linef(cmd_ctx, " if {[catch {info body target_%d_%s} t]==0} {target_%d_%s}",
			get_num_by_target(target), name,
			get_num_by_target(target), name);
}

int handle_working_area_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = NULL;

	if ((argc < 4) || (argc > 5))
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	target = get_target_by_num(strtoul(args[0], NULL, 0));
	if (!target)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	target_free_all_working_areas(target);

	target->working_area_phys = target->working_area_virt = strtoul(args[1], NULL, 0);
	if (argc == 5)
	{
		target->working_area_virt = strtoul(args[4], NULL, 0);
	}
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
		LOG_ERROR("unrecognized <backup|nobackup> argument (%s)", args[3]);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}


/* process target state changes */
int handle_target(void *priv)
{
	target_t *target = targets;

	while (target)
	{
		if (target_continous_poll)
		{
			/* polling may fail silently until the target has been examined */
			target_poll(target);
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

	LOG_DEBUG("-");

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
				LOG_ERROR("BUG: encountered unregistered arch type");
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
			LOG_ERROR("BUG: encountered unregistered arch type");
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

	if (argc == 0)
	{
		target_poll(target);
		target_arch_state(target);
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
		else
		{
			command_print(cmd_ctx, "arg is \"on\" or \"off\"");
		}
	}


	return ERROR_OK;
}

int handle_wait_halt_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int ms = 5000;

	if (argc > 0)
	{
		char *end;

		ms = strtoul(args[0], &end, 0) * 1000;
		if (*end)
		{
			command_print(cmd_ctx, "usage: %s [seconds]", cmd);
			return ERROR_OK;
		}
	}
	target_t *target = get_current_target(cmd_ctx);

	return target_wait_state(target, TARGET_HALTED, ms);
}

int target_wait_state(target_t *target, enum target_state state, int ms)
{
	int retval;
	struct timeval timeout, now;
	int once=1;
	gettimeofday(&timeout, NULL);
	timeval_add_time(&timeout, 0, ms * 1000);

	for (;;)
	{
		if ((retval=target_poll(target))!=ERROR_OK)
			return retval;
		target_call_timer_callbacks_now();
		if (target->state == state)
		{
			break;
		}
		if (once)
		{
			once=0;
			LOG_USER("waiting for target %s...", target_state_strings[state]);
		}

		gettimeofday(&now, NULL);
		if ((now.tv_sec > timeout.tv_sec) || ((now.tv_sec == timeout.tv_sec) && (now.tv_usec >= timeout.tv_usec)))
		{
			LOG_ERROR("timed out while waiting for target %s", target_state_strings[state]);
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

int handle_halt_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int retval;
	target_t *target = get_current_target(cmd_ctx);

	LOG_DEBUG("-");

	if ((retval = target_halt(target)) != ERROR_OK)
	{
		return retval;
	}

	return handle_wait_halt_command(cmd_ctx, cmd, args, argc);
}

int handle_soft_reset_halt_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);

	LOG_USER("requesting target halt and executing a soft reset");

	target->type->soft_reset_halt(target);

	return ERROR_OK;
}

int handle_reset_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	enum target_reset_mode reset_mode = RESET_RUN;

	if (argc >= 1)
	{
		if (strcmp("run", args[0]) == 0)
			reset_mode = RESET_RUN;
		else if (strcmp("halt", args[0]) == 0)
			reset_mode = RESET_HALT;
		else if (strcmp("init", args[0]) == 0)
			reset_mode = RESET_INIT;
		else
		{
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	/* reset *all* targets */
	target_process_reset(cmd_ctx, reset_mode);

	return ERROR_OK;
}

int handle_resume_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int retval;
	target_t *target = get_current_target(cmd_ctx);

	target_invoke_script(cmd_ctx, target, "pre_resume");

	if (argc == 0)
		retval = target_resume(target, 1, 0, 1, 0); /* current pc, addr = 0, handle breakpoints, not debugging */
	else if (argc == 1)
		retval = target_resume(target, 0, strtoul(args[0], NULL, 0), 1, 0); /* addr = args[0], handle breakpoints, not debugging */
	else
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return retval;
}

int handle_step_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);

	LOG_DEBUG("-");

	if (argc == 0)
		target->type->step(target, 1, 0, 1); /* current pc, addr = 0, handle breakpoints */

	if (argc == 1)
		target->type->step(target, 0, strtoul(args[0], NULL, 0), 1); /* addr = args[0], handle breakpoints */

	return ERROR_OK;
}

int handle_md_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	const int line_bytecnt = 32;
	int count = 1;
	int size = 4;
	u32 address = 0;
	int line_modulo;
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
			size = 4; line_modulo = line_bytecnt / 4;
			break;
		case 'h':
			size = 2; line_modulo = line_bytecnt / 2;
			break;
		case 'b':
			size = 1; line_modulo = line_bytecnt / 1;
			break;
		default:
			return ERROR_OK;
	}

	buffer = calloc(count, size);
	retval  = target->type->read_memory(target, address, size, count, buffer);
	if (retval == ERROR_OK)
	{
		output_len = 0;

		for (i = 0; i < count; i++)
		{
			if (i%line_modulo == 0)
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

			if ((i%line_modulo == line_modulo-1) || (i == count - 1))
			{
				command_print(cmd_ctx, output);
				output_len = 0;
			}
		}
	}

	free(buffer);

	return retval;
}

int handle_mw_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	u32 address = 0;
	u32 value = 0;
	int count = 1;
	int i;
	int wordsize;
	target_t *target = get_current_target(cmd_ctx);
	u8 value_buf[4];

	 if ((argc < 2) || (argc > 3))
		return ERROR_COMMAND_SYNTAX_ERROR;

	address = strtoul(args[0], NULL, 0);
	value = strtoul(args[1], NULL, 0);
	if (argc == 3)
		count = strtoul(args[2], NULL, 0);

	switch (cmd[2])
	{
		case 'w':
			wordsize = 4;
			target_buffer_set_u32(target, value_buf, value);
			break;
		case 'h':
			wordsize = 2;
			target_buffer_set_u16(target, value_buf, value);
			break;
		case 'b':
			wordsize = 1;
			value_buf[0] = value;
			break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}
	for (i=0; i<count; i++)
	{
		int retval;
		switch (wordsize)
		{
			case 4:
				retval = target->type->write_memory(target, address + i*wordsize, 4, 1, value_buf);
				break;
			case 2:
				retval = target->type->write_memory(target, address + i*wordsize, 2, 1, value_buf);
				break;
			case 1:
				retval = target->type->write_memory(target, address + i*wordsize, 1, 1, value_buf);
			break;
			default:
			return ERROR_OK;
		}
		if (retval!=ERROR_OK)
		{
			return retval;
		}
	}

	return ERROR_OK;

}

int handle_load_image_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	u8 *buffer;
	u32 buf_cnt;
	u32 image_size;
	u32 min_address=0;
	u32 max_address=0xffffffff;
	int i;
	int retval;

	image_t image;

	duration_t duration;
	char *duration_text;

	target_t *target = get_current_target(cmd_ctx);

	if ((argc < 1)||(argc > 5))
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
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

	if (argc>=4)
	{
		min_address=strtoul(args[3], NULL, 0);
	}
	if (argc>=5)
	{
		max_address=strtoul(args[4], NULL, 0)+min_address;
	}

	if (min_address>max_address)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}


	duration_start_measure(&duration);

	if (image_open(&image, args[0], (argc >= 3) ? args[2] : NULL) != ERROR_OK)
	{
		return ERROR_OK;
	}

	image_size = 0x0;
	retval = ERROR_OK;
	for (i = 0; i < image.num_sections; i++)
	{
		buffer = malloc(image.sections[i].size);
		if (buffer == NULL)
		{
			command_print(cmd_ctx, "error allocating buffer for section (%d bytes)", image.sections[i].size);
			break;
		}

		if ((retval = image_read_section(&image, i, 0x0, image.sections[i].size, buffer, &buf_cnt)) != ERROR_OK)
		{
			free(buffer);
			break;
		}

		u32 offset=0;
		u32 length=buf_cnt;


		/* DANGER!!! beware of unsigned comparision here!!! */

		if ((image.sections[i].base_address+buf_cnt>=min_address)&&
				(image.sections[i].base_address<max_address))
		{
			if (image.sections[i].base_address<min_address)
			{
				/* clip addresses below */
				offset+=min_address-image.sections[i].base_address;
				length-=offset;
			}

			if (image.sections[i].base_address+buf_cnt>max_address)
			{
				length-=(image.sections[i].base_address+buf_cnt)-max_address;
			}

			if ((retval = target_write_buffer(target, image.sections[i].base_address+offset, length, buffer+offset)) != ERROR_OK)
			{
				free(buffer);
				break;
			}
			image_size += length;
			command_print(cmd_ctx, "%u byte written at address 0x%8.8x", length, image.sections[i].base_address+offset);
		}

		free(buffer);
	}

	duration_stop_measure(&duration, &duration_text);
	if (retval==ERROR_OK)
	{
		command_print(cmd_ctx, "downloaded %u byte in %s", image_size, duration_text);
	}
	free(duration_text);

	image_close(&image);

	return retval;

}

int handle_dump_image_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	fileio_t fileio;

	u32 address;
	u32 size;
	u8 buffer[560];
	int retval=ERROR_OK;

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
		return ERROR_OK;
	}

	duration_start_measure(&duration);

	while (size > 0)
	{
		u32 size_written;
		u32 this_run_size = (size > 560) ? 560 : size;

		retval = target->type->read_memory(target, address, 4, this_run_size / 4, buffer);
		if (retval != ERROR_OK)
		{
			break;
		}

		retval = fileio_write(&fileio, this_run_size, buffer, &size_written);
		if (retval != ERROR_OK)
		{
			break;
		}

		size -= this_run_size;
		address += this_run_size;
	}

	fileio_close(&fileio);

	duration_stop_measure(&duration, &duration_text);
	if (retval==ERROR_OK)
	{
		command_print(cmd_ctx, "dumped %"PRIi64" byte in %s", fileio.size, duration_text);
	}
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
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (!target)
	{
		LOG_ERROR("no target selected");
		return ERROR_FAIL;
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

	if ((retval=image_open(&image, args[0], (argc == 3) ? args[2] : NULL)) != ERROR_OK)
	{
		return retval;
	}

	image_size = 0x0;
	retval=ERROR_OK;
	for (i = 0; i < image.num_sections; i++)
	{
		buffer = malloc(image.sections[i].size);
		if (buffer == NULL)
		{
			command_print(cmd_ctx, "error allocating buffer for section (%d bytes)", image.sections[i].size);
			break;
		}
		if ((retval = image_read_section(&image, i, 0x0, image.sections[i].size, buffer, &buf_cnt)) != ERROR_OK)
		{
			free(buffer);
			break;
		}

		/* calculate checksum of image */
		image_calculate_checksum( buffer, buf_cnt, &checksum );

		retval = target_checksum_memory(target, image.sections[i].base_address, buf_cnt, &mem_checksum);
		if( retval != ERROR_OK )
		{
			free(buffer);
			break;
		}

		if( checksum != mem_checksum )
		{
			/* failed crc checksum, fall back to a binary compare */
			u8 *data;

			command_print(cmd_ctx, "checksum mismatch - attempting binary compare");

			data = (u8*)malloc(buf_cnt);

			/* Can we use 32bit word accesses? */
			int size = 1;
			int count = buf_cnt;
			if ((count % 4) == 0)
			{
				size *= 4;
				count /= 4;
			}
			retval = target->type->read_memory(target, image.sections[i].base_address, size, count, data);
			if (retval == ERROR_OK)
			{
				int t;
				for (t = 0; t < buf_cnt; t++)
				{
					if (data[t] != buffer[t])
					{
						command_print(cmd_ctx, "Verify operation failed address 0x%08x. Was 0x%02x instead of 0x%02x\n", t + image.sections[i].base_address, data[t], buffer[t]);
						free(data);
						free(buffer);
						retval=ERROR_FAIL;
						goto done;
					}
				}
			}

			free(data);
		}

		free(buffer);
		image_size += buf_cnt;
	}
done:
	duration_stop_measure(&duration, &duration_text);
	if (retval==ERROR_OK)
	{
		command_print(cmd_ctx, "verified %u bytes in %s", image_size, duration_text);
	}
	free(duration_text);

	image_close(&image);

	return retval;
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
			LOG_ERROR("Failure setting breakpoints");
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
			command_print(cmd_ctx, "address: 0x%8.8x, len: 0x%8.8x, r/w/a: %i, value: 0x%8.8x, mask: 0x%8.8x", watchpoint->address, watchpoint->length, watchpoint->rw, watchpoint->value, watchpoint->mask);
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
			LOG_ERROR("Failure setting breakpoints");
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

int handle_virt2phys_command(command_context_t *cmd_ctx, char *cmd, char **args, int argc)
{
	int retval;
	target_t *target = get_current_target(cmd_ctx);
	u32 va;
	u32 pa;

	if (argc != 1)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	va = strtoul(args[0], NULL, 0);

	retval = target->type->virt2phys(target, va, &pa);
	if (retval == ERROR_OK)
	{
		command_print(cmd_ctx, "Physical address 0x%08x", pa);
	}
	else
	{
		/* lower levels will have logged a detailed error which is
		 * forwarded to telnet/GDB session.
		 */
	}
	return retval;
}
static void writeLong(FILE *f, int l)
{
	int i;
	for (i=0; i<4; i++)
	{
		char c=(l>>(i*8))&0xff;
		fwrite(&c, 1, 1, f);
	}

}
static void writeString(FILE *f, char *s)
{
	fwrite(s, 1, strlen(s), f);
}



// Dump a gmon.out histogram file.
static void writeGmon(u32 *samples, int sampleNum, char *filename)
{
	int i;
	FILE *f=fopen(filename, "w");
	if (f==NULL)
		return;
	fwrite("gmon", 1, 4, f);
	writeLong(f, 0x00000001); // Version
	writeLong(f, 0); // padding
	writeLong(f, 0); // padding
	writeLong(f, 0); // padding

	fwrite("", 1, 1, f);  // GMON_TAG_TIME_HIST

	// figure out bucket size
	u32 min=samples[0];
	u32 max=samples[0];
	for (i=0; i<sampleNum; i++)
	{
		if (min>samples[i])
		{
			min=samples[i];
		}
		if (max<samples[i])
		{
			max=samples[i];
		}
	}

	int addressSpace=(max-min+1);

	static int const maxBuckets=256*1024; // maximum buckets.
	int length=addressSpace;
	if (length > maxBuckets)
	{
		length=maxBuckets;
	}
	int *buckets=malloc(sizeof(int)*length);
	if (buckets==NULL)
	{
		fclose(f);
		return;
	}
	memset(buckets, 0, sizeof(int)*length);
	for (i=0; i<sampleNum;i++)
	{
		u32 address=samples[i];
		long long a=address-min;
		long long b=length-1;
		long long c=addressSpace-1;
		int index=(a*b)/c; // danger!!!! int32 overflows
		buckets[index]++;
	}

	//			   append binary memory gmon.out &profile_hist_hdr ((char*)&profile_hist_hdr + sizeof(struct gmon_hist_hdr))
	writeLong(f, min); 					// low_pc
	writeLong(f, max);		// high_pc
	writeLong(f, length);		// # of samples
	writeLong(f, 64000000); 			// 64MHz
	writeString(f, "seconds");
	for (i=0; i<(15-strlen("seconds")); i++)
	{
		fwrite("", 1, 1, f);  // padding
	}
	writeString(f, "s");

//			   append binary memory gmon.out profile_hist_data (profile_hist_data + profile_hist_hdr.hist_size)

	char *data=malloc(2*length);
	if (data!=NULL)
	{
		for (i=0; i<length;i++)
		{
			int val;
			val=buckets[i];
			if (val>65535)
			{
				val=65535;
			}
			data[i*2]=val&0xff;
			data[i*2+1]=(val>>8)&0xff;
		}
		free(buckets);
		fwrite(data, 1, length*2, f);
		free(data);
	} else
	{
		free(buckets);
	}

	fclose(f);
}

/* profiling samples the CPU PC as quickly as OpenOCD is able, which will be used as a random sampling of PC */
int handle_profile_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	struct timeval timeout, now;

	gettimeofday(&timeout, NULL);
	if (argc!=2)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	char *end;
	timeval_add_time(&timeout, strtoul(args[0], &end, 0), 0);
	if (*end)
	{
		return ERROR_OK;
	}

	command_print(cmd_ctx, "Starting profiling. Halting and resuming the target as often as we can...");

	static const int maxSample=10000;
	u32 *samples=malloc(sizeof(u32)*maxSample);
	if (samples==NULL)
		return ERROR_OK;

	int numSamples=0;
	int retval=ERROR_OK;
	// hopefully it is safe to cache! We want to stop/restart as quickly as possible.
	reg_t *reg = register_get_by_name(target->reg_cache, "pc", 1);

	for (;;)
	{
		target_poll(target);
		if (target->state == TARGET_HALTED)
		{
			u32 t=*((u32 *)reg->value);
			samples[numSamples++]=t;
			retval = target_resume(target, 1, 0, 0, 0); /* current pc, addr = 0, do not handle breakpoints, not debugging */
			target_poll(target);
			usleep(10*1000); // sleep 10ms, i.e. <100 samples/second.
		} else if (target->state == TARGET_RUNNING)
		{
			// We want to quickly sample the PC.
			target_halt(target);
		} else
		{
			command_print(cmd_ctx, "Target not halted or running");
			retval=ERROR_OK;
			break;
		}
		if (retval!=ERROR_OK)
		{
			break;
		}

		gettimeofday(&now, NULL);
		if ((numSamples>=maxSample) || ((now.tv_sec >= timeout.tv_sec) && (now.tv_usec >= timeout.tv_usec)))
		{
			command_print(cmd_ctx, "Profiling completed. %d samples.", numSamples);
			target_poll(target);
			if (target->state == TARGET_HALTED)
			{
				target_resume(target, 1, 0, 0, 0); /* current pc, addr = 0, do not handle breakpoints, not debugging */
			}
			target_poll(target);
			writeGmon(samples, numSamples, args[1]);
			command_print(cmd_ctx, "Wrote %s", args[1]);
			break;
		}
	}
	free(samples);

	return ERROR_OK;
}

static int new_int_array_element(Jim_Interp * interp, const char *varname, int idx, u32 val)
{
	char *namebuf;
	Jim_Obj *nameObjPtr, *valObjPtr;
	int result;

	namebuf = alloc_printf("%s(%d)", varname, idx);
	if (!namebuf)
		return JIM_ERR;

	nameObjPtr = Jim_NewStringObj(interp, namebuf, -1);
	valObjPtr = Jim_NewIntObj(interp, val);
	if (!nameObjPtr || !valObjPtr)
	{
		free(namebuf);
		return JIM_ERR;
	}

	Jim_IncrRefCount(nameObjPtr);
	Jim_IncrRefCount(valObjPtr);
	result = Jim_SetVariable(interp, nameObjPtr, valObjPtr);
	Jim_DecrRefCount(interp, nameObjPtr);
	Jim_DecrRefCount(interp, valObjPtr);
	free(namebuf);
	/* printf("%s(%d) <= 0%08x\n", varname, idx, val); */
	return result;
}

static int jim_mem2array(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	target_t *target;
	command_context_t *context;
	long l;
	u32 width;
	u32 len;
	u32 addr;
	u32 count;
	u32 v;
	const char *varname;
	u8 buffer[4096];
	int  i, n, e, retval;

	/* argv[1] = name of array to receive the data
	 * argv[2] = desired width
	 * argv[3] = memory address
	 * argv[4] = count of times to read
	 */
	if (argc != 5) {
		Jim_WrongNumArgs(interp, 1, argv, "varname width addr nelems");
		return JIM_ERR;
	}
	varname = Jim_GetString(argv[1], &len);
	/* given "foo" get space for worse case "foo(%d)" .. add 20 */

	e = Jim_GetLong(interp, argv[2], &l);
	width = l;
	if (e != JIM_OK) {
		return e;
	}

	e = Jim_GetLong(interp, argv[3], &l);
	addr = l;
	if (e != JIM_OK) {
		return e;
	}
	e = Jim_GetLong(interp, argv[4], &l);
	len = l;
	if (e != JIM_OK) {
		return e;
	}
	switch (width) {
		case 8:
			width = 1;
			break;
		case 16:
			width = 2;
			break;
		case 32:
			width = 4;
			break;
		default:
			Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
			Jim_AppendStrings( interp, Jim_GetResult(interp), "Invalid width param, must be 8/16/32", NULL );
			return JIM_ERR;
	}
	if (len == 0) {
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		Jim_AppendStrings(interp, Jim_GetResult(interp), "mem2array: zero width read?", NULL);
		return JIM_ERR;
	}
	if ((addr + (len * width)) < addr) {
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		Jim_AppendStrings(interp, Jim_GetResult(interp), "mem2array: addr + len - wraps to zero?", NULL);
		return JIM_ERR;
	}
	/* absurd transfer size? */
	if (len > 65536) {
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		Jim_AppendStrings(interp, Jim_GetResult(interp), "mem2array: absurd > 64K item request", NULL);
		return JIM_ERR;
	}

	if ((width == 1) ||
		((width == 2) && ((addr & 1) == 0)) ||
		((width == 4) && ((addr & 3) == 0))) {
		/* all is well */
	} else {
		char buf[100];
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		sprintf(buf, "mem2array address: 0x%08x is not aligned for %d byte reads", addr, width);
		Jim_AppendStrings(interp, Jim_GetResult(interp), buf , NULL);
		return JIM_ERR;
	}

	context = Jim_GetAssocData(interp, "context");
	if (context == NULL)
	{
		LOG_ERROR("mem2array: no command context");
		return JIM_ERR;
	}
	target = get_current_target(context);
	if (target == NULL)
	{
		LOG_ERROR("mem2array: no current target");
		return JIM_ERR;
	}

	/* Transfer loop */

	/* index counter */
	n = 0;
	/* assume ok */
	e = JIM_OK;
	while (len) {
		/* Slurp... in buffer size chunks */

		count = len; /* in objects.. */
		if (count > (sizeof(buffer)/width)) {
			count = (sizeof(buffer)/width);
		}

		retval = target->type->read_memory( target, addr, width, count, buffer );
		if (retval != ERROR_OK) {
			/* BOO !*/
			LOG_ERROR("mem2array: Read @ 0x%08x, w=%d, cnt=%d, failed", addr, width, count);
			Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
			Jim_AppendStrings(interp, Jim_GetResult(interp), "mem2array: cannot read memory", NULL);
			e = JIM_ERR;
			len = 0;
		} else {
			v = 0; /* shut up gcc */
			for (i = 0 ;i < count ;i++, n++) {
				switch (width) {
					case 4:
						v = target_buffer_get_u32(target, &buffer[i*width]);
						break;
					case 2:
						v = target_buffer_get_u16(target, &buffer[i*width]);
						break;
					case 1:
						v = buffer[i] & 0x0ff;
						break;
				}
				new_int_array_element(interp, varname, n, v);
			}
			len -= count;
		}
	}

	Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));

	return JIM_OK;
}

static int get_int_array_element(Jim_Interp * interp, const char *varname, int idx, u32 *val)
{
	char *namebuf;
	Jim_Obj *nameObjPtr, *valObjPtr;
	int result;
	long l;

	namebuf = alloc_printf("%s(%d)", varname, idx);
	if (!namebuf)
		return JIM_ERR;

	nameObjPtr = Jim_NewStringObj(interp, namebuf, -1);
	if (!nameObjPtr)
	{
		free(namebuf);
		return JIM_ERR;
	}

	Jim_IncrRefCount(nameObjPtr);
	valObjPtr = Jim_GetVariable(interp, nameObjPtr, JIM_ERRMSG);
	Jim_DecrRefCount(interp, nameObjPtr);
	free(namebuf);
	if (valObjPtr == NULL)
		return JIM_ERR;

	result = Jim_GetLong(interp, valObjPtr, &l);
	/* printf("%s(%d) => 0%08x\n", varname, idx, val); */
	*val = l;
	return result;
}

static int jim_array2mem(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	target_t *target;
	command_context_t *context;
	long l;
	u32 width;
	u32 len;
	u32 addr;
	u32 count;
	u32 v;
	const char *varname;
	u8 buffer[4096];
	int  i, n, e, retval;

	/* argv[1] = name of array to get the data
	 * argv[2] = desired width
	 * argv[3] = memory address
	 * argv[4] = count to write
	 */
	if (argc != 5) {
		Jim_WrongNumArgs(interp, 1, argv, "varname width addr nelems");
		return JIM_ERR;
	}
	varname = Jim_GetString(argv[1], &len);
	/* given "foo" get space for worse case "foo(%d)" .. add 20 */

	e = Jim_GetLong(interp, argv[2], &l);
	width = l;
	if (e != JIM_OK) {
		return e;
	}

	e = Jim_GetLong(interp, argv[3], &l);
	addr = l;
	if (e != JIM_OK) {
		return e;
	}
	e = Jim_GetLong(interp, argv[4], &l);
	len = l;
	if (e != JIM_OK) {
		return e;
	}
	switch (width) {
		case 8:
			width = 1;
			break;
		case 16:
			width = 2;
			break;
		case 32:
			width = 4;
			break;
		default:
			Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
			Jim_AppendStrings( interp, Jim_GetResult(interp), "Invalid width param, must be 8/16/32", NULL );
			return JIM_ERR;
	}
	if (len == 0) {
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		Jim_AppendStrings(interp, Jim_GetResult(interp), "array2mem: zero width read?", NULL);
		return JIM_ERR;
	}
	if ((addr + (len * width)) < addr) {
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		Jim_AppendStrings(interp, Jim_GetResult(interp), "array2mem: addr + len - wraps to zero?", NULL);
		return JIM_ERR;
	}
	/* absurd transfer size? */
	if (len > 65536) {
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		Jim_AppendStrings(interp, Jim_GetResult(interp), "array2mem: absurd > 64K item request", NULL);
		return JIM_ERR;
	}

	if ((width == 1) ||
		((width == 2) && ((addr & 1) == 0)) ||
		((width == 4) && ((addr & 3) == 0))) {
		/* all is well */
	} else {
		char buf[100];
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		sprintf(buf, "array2mem address: 0x%08x is not aligned for %d byte reads", addr, width);
		Jim_AppendStrings(interp, Jim_GetResult(interp), buf , NULL);
		return JIM_ERR;
	}

	context = Jim_GetAssocData(interp, "context");
	if (context == NULL)
	{
		LOG_ERROR("array2mem: no command context");
		return JIM_ERR;
	}
	target = get_current_target(context);
	if (target == NULL)
	{
		LOG_ERROR("array2mem: no current target");
		return JIM_ERR;
	}

	/* Transfer loop */

	/* index counter */
	n = 0;
	/* assume ok */
	e = JIM_OK;
	while (len) {
		/* Slurp... in buffer size chunks */

		count = len; /* in objects.. */
		if (count > (sizeof(buffer)/width)) {
			count = (sizeof(buffer)/width);
		}

		v = 0; /* shut up gcc */
		for (i = 0 ;i < count ;i++, n++) {
			get_int_array_element(interp, varname, n, &v);
			switch (width) {
			case 4:
				target_buffer_set_u32(target, &buffer[i*width], v);
				break;
			case 2:
				target_buffer_set_u16(target, &buffer[i*width], v);
				break;
			case 1:
				buffer[i] = v & 0x0ff;
				break;
			}
		}
		len -= count;

		retval = target->type->write_memory(target, addr, width, count, buffer);
		if (retval != ERROR_OK) {
			/* BOO !*/
			LOG_ERROR("array2mem: Write @ 0x%08x, w=%d, cnt=%d, failed", addr, width, count);
			Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
			Jim_AppendStrings(interp, Jim_GetResult(interp), "mem2array: cannot read memory", NULL);
			e = JIM_ERR;
			len = 0;
		}
	}

	Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));

	return JIM_OK;
}
