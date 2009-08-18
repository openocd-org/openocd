/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008, Duane Ellis                                       *
 *   openocd@duaneeellis.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by Rick Altherr                                    *
 *   kc8apf@kc8apf.net>                                                    *
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

#include "target.h"
#include "target_type.h"
#include "target_request.h"
#include "time_support.h"
#include "register.h"
#include "trace.h"
#include "image.h"
#include "jtag.h"


static int handle_targets_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

static int handle_reg_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_poll_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_halt_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_wait_halt_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_reset_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_soft_reset_halt_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_resume_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_step_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_md_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_mw_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_load_image_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_dump_image_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_verify_image_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_test_image_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_bp_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_rbp_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_wp_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_rwp_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_virt2phys_command(command_context_t *cmd_ctx, char *cmd, char **args, int argc);
static int handle_profile_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_fast_load_image_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_fast_load_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

static int jim_array2mem(Jim_Interp *interp, int argc, Jim_Obj *const *argv);
static int jim_mem2array(Jim_Interp *interp, int argc, Jim_Obj *const *argv);
static int jim_target(Jim_Interp *interp, int argc, Jim_Obj *const *argv);

static int target_array2mem(Jim_Interp *interp, target_t *target, int argc, Jim_Obj *const *argv);
static int target_mem2array(Jim_Interp *interp, target_t *target, int argc, Jim_Obj *const *argv);

/* targets */
extern target_type_t arm7tdmi_target;
extern target_type_t arm720t_target;
extern target_type_t arm9tdmi_target;
extern target_type_t arm920t_target;
extern target_type_t arm966e_target;
extern target_type_t arm926ejs_target;
extern target_type_t fa526_target;
extern target_type_t feroceon_target;
extern target_type_t xscale_target;
extern target_type_t cortexm3_target;
extern target_type_t cortexa8_target;
extern target_type_t arm11_target;
extern target_type_t mips_m4k_target;
extern target_type_t avr_target;

target_type_t *target_types[] =
{
	&arm7tdmi_target,
	&arm9tdmi_target,
	&arm920t_target,
	&arm720t_target,
	&arm966e_target,
	&arm926ejs_target,
	&fa526_target,
	&feroceon_target,
	&xscale_target,
	&cortexm3_target,
	&cortexa8_target,
	&arm11_target,
	&mips_m4k_target,
	&avr_target,
	NULL,
};

target_t *all_targets = NULL;
target_event_callback_t *target_event_callbacks = NULL;
target_timer_callback_t *target_timer_callbacks = NULL;

const Jim_Nvp nvp_assert[] = {
	{ .name = "assert", NVP_ASSERT },
	{ .name = "deassert", NVP_DEASSERT },
	{ .name = "T", NVP_ASSERT },
	{ .name = "F", NVP_DEASSERT },
	{ .name = "t", NVP_ASSERT },
	{ .name = "f", NVP_DEASSERT },
	{ .name = NULL, .value = -1 }
};

const Jim_Nvp nvp_error_target[] = {
	{ .value = ERROR_TARGET_INVALID, .name = "err-invalid" },
	{ .value = ERROR_TARGET_INIT_FAILED, .name = "err-init-failed" },
	{ .value = ERROR_TARGET_TIMEOUT, .name = "err-timeout" },
	{ .value = ERROR_TARGET_NOT_HALTED, .name = "err-not-halted" },
	{ .value = ERROR_TARGET_FAILURE, .name = "err-failure" },
	{ .value = ERROR_TARGET_UNALIGNED_ACCESS   , .name = "err-unaligned-access" },
	{ .value = ERROR_TARGET_DATA_ABORT , .name = "err-data-abort" },
	{ .value = ERROR_TARGET_RESOURCE_NOT_AVAILABLE , .name = "err-resource-not-available" },
	{ .value = ERROR_TARGET_TRANSLATION_FAULT  , .name = "err-translation-fault" },
	{ .value = ERROR_TARGET_NOT_RUNNING, .name = "err-not-running" },
	{ .value = ERROR_TARGET_NOT_EXAMINED, .name = "err-not-examined" },
	{ .value = -1, .name = NULL }
};

const char *target_strerror_safe(int err)
{
	const Jim_Nvp *n;

	n = Jim_Nvp_value2name_simple(nvp_error_target, err);
	if (n->name == NULL) {
		return "unknown";
	} else {
		return n->name;
	}
}

static const Jim_Nvp nvp_target_event[] = {
	{ .value = TARGET_EVENT_OLD_gdb_program_config , .name = "old-gdb_program_config" },
	{ .value = TARGET_EVENT_OLD_pre_resume         , .name = "old-pre_resume" },

	{ .value = TARGET_EVENT_EARLY_HALTED, .name = "early-halted" },
	{ .value = TARGET_EVENT_HALTED, .name = "halted" },
	{ .value = TARGET_EVENT_RESUMED, .name = "resumed" },
	{ .value = TARGET_EVENT_RESUME_START, .name = "resume-start" },
	{ .value = TARGET_EVENT_RESUME_END, .name = "resume-end" },

	{ .name = "gdb-start", .value = TARGET_EVENT_GDB_START },
	{ .name = "gdb-end", .value = TARGET_EVENT_GDB_END },

	/* historical name */

	{ .value = TARGET_EVENT_RESET_START, .name = "reset-start" },

	{ .value = TARGET_EVENT_RESET_ASSERT_PRE,    .name = "reset-assert-pre" },
	{ .value = TARGET_EVENT_RESET_ASSERT_POST,   .name = "reset-assert-post" },
	{ .value = TARGET_EVENT_RESET_DEASSERT_PRE,  .name = "reset-deassert-pre" },
	{ .value = TARGET_EVENT_RESET_DEASSERT_POST, .name = "reset-deassert-post" },
	{ .value = TARGET_EVENT_RESET_HALT_PRE,      .name = "reset-halt-pre" },
	{ .value = TARGET_EVENT_RESET_HALT_POST,     .name = "reset-halt-post" },
	{ .value = TARGET_EVENT_RESET_WAIT_PRE,      .name = "reset-wait-pre" },
	{ .value = TARGET_EVENT_RESET_WAIT_POST,     .name = "reset-wait-post" },
	{ .value = TARGET_EVENT_RESET_INIT , .name = "reset-init" },
	{ .value = TARGET_EVENT_RESET_END, .name = "reset-end" },

	{ .value = TARGET_EVENT_EXAMINE_START, .name = "examine-start" },
	{ .value = TARGET_EVENT_EXAMINE_END, .name = "examine-end" },

	{ .value = TARGET_EVENT_DEBUG_HALTED, .name = "debug-halted" },
	{ .value = TARGET_EVENT_DEBUG_RESUMED, .name = "debug-resumed" },

	{ .value = TARGET_EVENT_GDB_ATTACH, .name = "gdb-attach" },
	{ .value = TARGET_EVENT_GDB_DETACH, .name = "gdb-detach" },

	{ .value = TARGET_EVENT_GDB_FLASH_WRITE_START, .name = "gdb-flash-write-start" },
	{ .value = TARGET_EVENT_GDB_FLASH_WRITE_END  , .name = "gdb-flash-write-end"   },

	{ .value = TARGET_EVENT_GDB_FLASH_ERASE_START, .name = "gdb-flash-erase-start" },
	{ .value = TARGET_EVENT_GDB_FLASH_ERASE_END  , .name = "gdb-flash-erase-end" },

	{ .value = TARGET_EVENT_RESUME_START, .name = "resume-start" },
	{ .value = TARGET_EVENT_RESUMED     , .name = "resume-ok" },
	{ .value = TARGET_EVENT_RESUME_END  , .name = "resume-end" },

	{ .name = NULL, .value = -1 }
};

const Jim_Nvp nvp_target_state[] = {
	{ .name = "unknown", .value = TARGET_UNKNOWN },
	{ .name = "running", .value = TARGET_RUNNING },
	{ .name = "halted",  .value = TARGET_HALTED },
	{ .name = "reset",   .value = TARGET_RESET },
	{ .name = "debug-running", .value = TARGET_DEBUG_RUNNING },
	{ .name = NULL, .value = -1 },
};

const Jim_Nvp nvp_target_debug_reason [] = {
	{ .name = "debug-request"            , .value = DBG_REASON_DBGRQ },
	{ .name = "breakpoint"               , .value = DBG_REASON_BREAKPOINT },
	{ .name = "watchpoint"               , .value = DBG_REASON_WATCHPOINT },
	{ .name = "watchpoint-and-breakpoint", .value = DBG_REASON_WPTANDBKPT },
	{ .name = "single-step"              , .value = DBG_REASON_SINGLESTEP },
	{ .name = "target-not-halted"        , .value = DBG_REASON_NOTHALTED  },
	{ .name = "undefined"                , .value = DBG_REASON_UNDEFINED },
	{ .name = NULL, .value = -1 },
};

const Jim_Nvp nvp_target_endian[] = {
	{ .name = "big",    .value = TARGET_BIG_ENDIAN },
	{ .name = "little", .value = TARGET_LITTLE_ENDIAN },
	{ .name = "be",     .value = TARGET_BIG_ENDIAN },
	{ .name = "le",     .value = TARGET_LITTLE_ENDIAN },
	{ .name = NULL,     .value = -1 },
};

const Jim_Nvp nvp_reset_modes[] = {
	{ .name = "unknown", .value = RESET_UNKNOWN },
	{ .name = "run"    , .value = RESET_RUN },
	{ .name = "halt"   , .value = RESET_HALT },
	{ .name = "init"   , .value = RESET_INIT },
	{ .name = NULL     , .value = -1 },
};

const char *
target_state_name( target_t *t )
{
	const char *cp;
	cp = Jim_Nvp_value2name_simple(nvp_target_state, t->state)->name;
	if( !cp ){
		LOG_ERROR("Invalid target state: %d", (int)(t->state));
		cp = "(*BUG*unknown*BUG*)";
	}
	return cp;
}

static int max_target_number(void)
{
	target_t *t;
	int x;

	x = -1;
	t = all_targets;
	while (t) {
		if (x < t->target_number) {
			x = (t->target_number) + 1;
		}
		t = t->next;
	}
	return x;
}

/* determine the number of the new target */
static int new_target_number(void)
{
	target_t *t;
	int x;

	/* number is 0 based */
	x = -1;
	t = all_targets;
	while (t) {
		if (x < t->target_number) {
			x = t->target_number;
		}
		t = t->next;
	}
	return x + 1;
}

static int target_continuous_poll = 1;

/* read a uint32_t from a buffer in target memory endianness */
uint32_t target_buffer_get_u32(target_t *target, const uint8_t *buffer)
{
	if (target->endianness == TARGET_LITTLE_ENDIAN)
		return le_to_h_u32(buffer);
	else
		return be_to_h_u32(buffer);
}

/* read a uint16_t from a buffer in target memory endianness */
uint16_t target_buffer_get_u16(target_t *target, const uint8_t *buffer)
{
	if (target->endianness == TARGET_LITTLE_ENDIAN)
		return le_to_h_u16(buffer);
	else
		return be_to_h_u16(buffer);
}

/* read a uint8_t from a buffer in target memory endianness */
uint8_t target_buffer_get_u8(target_t *target, const uint8_t *buffer)
{
	return *buffer & 0x0ff;
}

/* write a uint32_t to a buffer in target memory endianness */
void target_buffer_set_u32(target_t *target, uint8_t *buffer, uint32_t value)
{
	if (target->endianness == TARGET_LITTLE_ENDIAN)
		h_u32_to_le(buffer, value);
	else
		h_u32_to_be(buffer, value);
}

/* write a uint16_t to a buffer in target memory endianness */
void target_buffer_set_u16(target_t *target, uint8_t *buffer, uint16_t value)
{
	if (target->endianness == TARGET_LITTLE_ENDIAN)
		h_u16_to_le(buffer, value);
	else
		h_u16_to_be(buffer, value);
}

/* write a uint8_t to a buffer in target memory endianness */
void target_buffer_set_u8(target_t *target, uint8_t *buffer, uint8_t value)
{
	*buffer = value;
}

/* return a pointer to a configured target; id is name or number */
target_t *get_target(const char *id)
{
	target_t *target;

	/* try as tcltarget name */
	for (target = all_targets; target; target = target->next) {
		if (target->cmd_name == NULL)
			continue;
		if (strcmp(id, target->cmd_name) == 0)
			return target;
	}

	/* no match, try as number */
	unsigned num;
	if (parse_uint(id, &num) != ERROR_OK)
		return NULL;

	for (target = all_targets; target; target = target->next) {
		if (target->target_number == (int)num)
			return target;
	}

	return NULL;
}

/* returns a pointer to the n-th configured target */
static target_t *get_target_by_num(int num)
{
	target_t *target = all_targets;

	while (target) {
		if (target->target_number == num) {
			return target;
		}
		target = target->next;
	}

	return NULL;
}

int get_num_by_target(target_t *query_target)
{
	return query_target->target_number;
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
	if (!target_was_examined(target))
	{
		/* Fail silently lest we pollute the log */
		return ERROR_FAIL;
	}
	return target->type->poll(target);
}

int target_halt(struct target_s *target)
{
	/* We can't poll until after examine */
	if (!target_was_examined(target))
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}
	return target->type->halt(target);
}

int target_resume(struct target_s *target, int current, uint32_t address, int handle_breakpoints, int debug_execution)
{
	int retval;

	/* We can't poll until after examine */
	if (!target_was_examined(target))
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
	char buf[100];
	int retval;
	Jim_Nvp *n;
	n = Jim_Nvp_value2name_simple(nvp_reset_modes, reset_mode);
	if (n->name == NULL) {
		LOG_ERROR("invalid reset mode");
		return ERROR_FAIL;
	}

	/* disable polling during reset to make reset event scripts
	 * more predictable, i.e. dr/irscan & pathmove in events will
	 * not have JTAG operations injected into the middle of a sequence.
	 */
	int save_poll = target_continuous_poll;
	target_continuous_poll = 0;

	sprintf(buf, "ocd_process_reset %s", n->name);
	retval = Jim_Eval(interp, buf);

	target_continuous_poll = save_poll;

	if (retval != JIM_OK) {
		Jim_PrintErrorMessage(interp);
		return ERROR_FAIL;
	}

	/* We want any events to be processed before the prompt */
	retval = target_call_timer_callbacks_now();

	return retval;
}

static int default_virt2phys(struct target_s *target, uint32_t virtual, uint32_t *physical)
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
	target_set_examined(target);
	return ERROR_OK;
}

int target_examine_one(struct target_s *target)
{
	return target->type->examine(target);
}

static int jtag_enable_callback(enum jtag_event event, void *priv)
{
	target_t *target = priv;

	if (event != JTAG_TAP_EVENT_ENABLE || !target->tap->enabled)
		return ERROR_OK;

	jtag_unregister_event_callback(jtag_enable_callback, target);
	return target_examine_one(target);
}


/* Targets that correctly implement init + examine, i.e.
 * no communication with target during init:
 *
 * XScale
 */
int target_examine(void)
{
	int retval = ERROR_OK;
	target_t *target;

	for (target = all_targets; target; target = target->next)
	{
		/* defer examination, but don't skip it */
		if (!target->tap->enabled) {
			jtag_register_event_callback(jtag_enable_callback,
					target);
			continue;
		}
		if ((retval = target_examine_one(target)) != ERROR_OK)
			return retval;
	}
	return retval;
}
const char *target_get_name(struct target_s *target)
{
	return target->type->name;
}

static int target_write_memory_imp(struct target_s *target, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer)
{
	if (!target_was_examined(target))
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}
	return target->type->write_memory_imp(target, address, size, count, buffer);
}

static int target_read_memory_imp(struct target_s *target, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer)
{
	if (!target_was_examined(target))
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}
	return target->type->read_memory_imp(target, address, size, count, buffer);
}

static int target_soft_reset_halt_imp(struct target_s *target)
{
	if (!target_was_examined(target))
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}
	return target->type->soft_reset_halt_imp(target);
}

static int target_run_algorithm_imp(struct target_s *target, int num_mem_params, mem_param_t *mem_params, int num_reg_params, reg_param_t *reg_param, uint32_t entry_point, uint32_t exit_point, int timeout_ms, void *arch_info)
{
	if (!target_was_examined(target))
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}
	return target->type->run_algorithm_imp(target, num_mem_params, mem_params, num_reg_params, reg_param, entry_point, exit_point, timeout_ms, arch_info);
}

int target_read_memory(struct target_s *target,
		uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer)
{
	return target->type->read_memory(target, address, size, count, buffer);
}

int target_write_memory(struct target_s *target,
		uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer)
{
	return target->type->write_memory(target, address, size, count, buffer);
}
int target_bulk_write_memory(struct target_s *target,
		uint32_t address, uint32_t count, uint8_t *buffer)
{
	return target->type->bulk_write_memory(target, address, count, buffer);
}

int target_add_breakpoint(struct target_s *target,
		struct breakpoint_s *breakpoint)
{
	return target->type->add_breakpoint(target, breakpoint);
}
int target_remove_breakpoint(struct target_s *target,
		struct breakpoint_s *breakpoint)
{
	return target->type->remove_breakpoint(target, breakpoint);
}

int target_add_watchpoint(struct target_s *target,
		struct watchpoint_s *watchpoint)
{
	return target->type->add_watchpoint(target, watchpoint);
}
int target_remove_watchpoint(struct target_s *target,
		struct watchpoint_s *watchpoint)
{
	return target->type->remove_watchpoint(target, watchpoint);
}

int target_get_gdb_reg_list(struct target_s *target,
		struct reg_s **reg_list[], int *reg_list_size)
{
	return target->type->get_gdb_reg_list(target, reg_list, reg_list_size);
}
int target_step(struct target_s *target,
		int current, uint32_t address, int handle_breakpoints)
{
	return target->type->step(target, current, address, handle_breakpoints);
}


int target_run_algorithm(struct target_s *target,
		int num_mem_params, mem_param_t *mem_params,
		int num_reg_params, reg_param_t *reg_param,
		uint32_t entry_point, uint32_t exit_point,
		int timeout_ms, void *arch_info)
{
	return target->type->run_algorithm(target,
			num_mem_params, mem_params, num_reg_params, reg_param,
			entry_point, exit_point, timeout_ms, arch_info);
}

/// @returns @c true if the target has been examined.
bool target_was_examined(struct target_s *target)
{
	return target->type->examined;
}
/// Sets the @c examined flag for the given target.
void target_set_examined(struct target_s *target)
{
	target->type->examined = true;
}
// Reset the @c examined flag for the given target.
void target_reset_examined(struct target_s *target)
{
	target->type->examined = false;
}


int target_init(struct command_context_s *cmd_ctx)
{
	target_t *target = all_targets;
	int retval;

	while (target)
	{
		target_reset_examined(target);
		if (target->type->examine == NULL)
		{
			target->type->examine = default_examine;
		}

		if ((retval = target->type->init_target(cmd_ctx, target)) != ERROR_OK)
		{
			LOG_ERROR("target '%s' init failed", target_get_name(target));
			return retval;
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

	if (all_targets)
	{
		if ((retval = target_register_user_commands(cmd_ctx)) != ERROR_OK)
			return retval;
		if ((retval = target_register_timer_callback(handle_target, 100, 1, NULL)) != ERROR_OK)
			return retval;
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

	if (event == TARGET_EVENT_HALTED)
	{
		/* execute early halted first */
		target_call_event_callbacks(target, TARGET_EVENT_EARLY_HALTED);
	}

	LOG_DEBUG("target event %i (%s)",
			  event,
			  Jim_Nvp_value2name_simple(nvp_target_event, event)->name);

	target_handle_event(target, event);

	while (callback)
	{
		next_callback = callback->next;
		callback->callback(target, event, callback->priv);
		callback = next_callback;
	}

	return ERROR_OK;
}

static int target_timer_callback_periodic_restart(
		target_timer_callback_t *cb, struct timeval *now)
{
	int time_ms = cb->time_ms;
	cb->when.tv_usec = now->tv_usec + (time_ms % 1000) * 1000;
	time_ms -= (time_ms % 1000);
	cb->when.tv_sec = now->tv_sec + time_ms / 1000;
	if (cb->when.tv_usec > 1000000)
	{
		cb->when.tv_usec = cb->when.tv_usec - 1000000;
		cb->when.tv_sec += 1;
	}
	return ERROR_OK;
}

static int target_call_timer_callback(target_timer_callback_t *cb,
		struct timeval *now)
{
	cb->callback(cb->priv);

	if (cb->periodic)
		return target_timer_callback_periodic_restart(cb, now);

	return target_unregister_timer_callback(cb->callback, cb->priv);
}

static int target_call_timer_callbacks_check_time(int checktime)
{
	keep_alive();

	struct timeval now;
	gettimeofday(&now, NULL);

	target_timer_callback_t *callback = target_timer_callbacks;
	while (callback)
	{
		// cleaning up may unregister and free this callback
		target_timer_callback_t *next_callback = callback->next;

		bool call_it = callback->callback &&
			((!checktime && callback->periodic) ||
			  now.tv_sec > callback->when.tv_sec ||
			 (now.tv_sec == callback->when.tv_sec &&
			  now.tv_usec >= callback->when.tv_usec));

		if (call_it)
		{
			int retval = target_call_timer_callback(callback, &now);
			if (retval != ERROR_OK)
				return retval;
		}

		callback = next_callback;
	}

	return ERROR_OK;
}

int target_call_timer_callbacks(void)
{
	return target_call_timer_callbacks_check_time(1);
}

/* invoke periodic callbacks immediately */
int target_call_timer_callbacks_now(void)
{
	return target_call_timer_callbacks_check_time(0);
}

int target_alloc_working_area(struct target_s *target, uint32_t size, working_area_t **area)
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
		LOG_ERROR("BUG: code tried to allocate unaligned number of bytes (0x%08x), padding", ((unsigned)(size)));
		size = (size + 3) & (~3);
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
		uint32_t first_free = target->working_area;
		uint32_t free_size = target->working_area_size;

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
			LOG_WARNING("not enough working area available(requested %u, free %u)",
				    (unsigned)(size), (unsigned)(free_size));
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}

		new_wa = malloc(sizeof(working_area_t));
		new_wa->next = NULL;
		new_wa->size = size;
		new_wa->address = first_free;

		if (target->backup_working_area)
		{
			int retval;
			new_wa->backup = malloc(new_wa->size);
			if ((retval = target_read_memory(target, new_wa->address, 4, new_wa->size / 4, new_wa->backup)) != ERROR_OK)
			{
				free(new_wa->backup);
				free(new_wa);
				return retval;
			}
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

	if (restore && target->backup_working_area)
	{
		int retval;
		if ((retval = target_write_memory(target, area->address, 4, area->size / 4, area->backup)) != ERROR_OK)
			return retval;
	}

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

/* free resources and restore memory, if restoring memory fails,
 * free up resources anyway
 */
void target_free_all_working_areas_restore(struct target_s *target, int restore)
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
}

void target_free_all_working_areas(struct target_s *target)
{
	target_free_all_working_areas_restore(target, 1);
}

int target_register_commands(struct command_context_s *cmd_ctx)
{

	register_command(cmd_ctx, NULL, "targets", handle_targets_command, COMMAND_EXEC, "change the current command line target (one parameter) or lists targets (with no parameter)");




	register_jim(cmd_ctx, "target", jim_target, "configure target");

	return ERROR_OK;
}

int target_arch_state(struct target_s *target)
{
	int retval;
	if (target == NULL)
	{
		LOG_USER("No target has been configured");
		return ERROR_OK;
	}

	LOG_USER("target state: %s", target_state_name( target ));

	if (target->state != TARGET_HALTED)
		return ERROR_OK;

	retval = target->type->arch_state(target);
	return retval;
}

/* Single aligned words are guaranteed to use 16 or 32 bit access
 * mode respectively, otherwise data is handled as quickly as
 * possible
 */
int target_write_buffer(struct target_s *target, uint32_t address, uint32_t size, uint8_t *buffer)
{
	int retval;
	LOG_DEBUG("writing buffer of %i byte at 0x%8.8x",
		  (int)size, (unsigned)address);

	if (!target_was_examined(target))
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}

	if (size == 0) {
		return ERROR_OK;
	}

	if ((address + size - 1) < address)
	{
		/* GDB can request this when e.g. PC is 0xfffffffc*/
		LOG_ERROR("address + size wrapped(0x%08x, 0x%08x)",
				  (unsigned)address,
				  (unsigned)size);
		return ERROR_FAIL;
	}

	if (((address % 2) == 0) && (size == 2))
	{
		return target_write_memory(target, address, 2, 1, buffer);
	}

	/* handle unaligned head bytes */
	if (address % 4)
	{
		uint32_t unaligned = 4 - (address % 4);

		if (unaligned > size)
			unaligned = size;

		if ((retval = target_write_memory(target, address, 1, unaligned, buffer)) != ERROR_OK)
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
			if ((retval = target_write_memory(target, address, 4, aligned / 4, buffer)) != ERROR_OK)
				return retval;
		}

		buffer += aligned;
		address += aligned;
		size -= aligned;
	}

	/* handle tail writes of less than 4 bytes */
	if (size > 0)
	{
		if ((retval = target_write_memory(target, address, 1, size, buffer)) != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

/* Single aligned words are guaranteed to use 16 or 32 bit access
 * mode respectively, otherwise data is handled as quickly as
 * possible
 */
int target_read_buffer(struct target_s *target, uint32_t address, uint32_t size, uint8_t *buffer)
{
	int retval;
	LOG_DEBUG("reading buffer of %i byte at 0x%8.8x",
			  (int)size, (unsigned)address);

	if (!target_was_examined(target))
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}

	if (size == 0) {
		return ERROR_OK;
	}

	if ((address + size - 1) < address)
	{
		/* GDB can request this when e.g. PC is 0xfffffffc*/
		LOG_ERROR("address + size wrapped(0x%08" PRIx32 ", 0x%08" PRIx32 ")",
				  address,
				  size);
		return ERROR_FAIL;
	}

	if (((address % 2) == 0) && (size == 2))
	{
		return target_read_memory(target, address, 2, 1, buffer);
	}

	/* handle unaligned head bytes */
	if (address % 4)
	{
		uint32_t unaligned = 4 - (address % 4);

		if (unaligned > size)
			unaligned = size;

		if ((retval = target_read_memory(target, address, 1, unaligned, buffer)) != ERROR_OK)
			return retval;

		buffer += unaligned;
		address += unaligned;
		size -= unaligned;
	}

	/* handle aligned words */
	if (size >= 4)
	{
		int aligned = size - (size % 4);

		if ((retval = target_read_memory(target, address, 4, aligned / 4, buffer)) != ERROR_OK)
			return retval;

		buffer += aligned;
		address += aligned;
		size -= aligned;
	}

	/* handle tail writes of less than 4 bytes */
	if (size > 0)
	{
		if ((retval = target_read_memory(target, address, 1, size, buffer)) != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

int target_checksum_memory(struct target_s *target, uint32_t address, uint32_t size, uint32_t* crc)
{
	uint8_t *buffer;
	int retval;
	uint32_t i;
	uint32_t checksum = 0;
	if (!target_was_examined(target))
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}

	if ((retval = target->type->checksum_memory(target, address,
		size, &checksum)) != ERROR_OK)
	{
		buffer = malloc(size);
		if (buffer == NULL)
		{
			LOG_ERROR("error allocating buffer for section (%d bytes)", (int)size);
			return ERROR_INVALID_ARGUMENTS;
		}
		retval = target_read_buffer(target, address, size, buffer);
		if (retval != ERROR_OK)
		{
			free(buffer);
			return retval;
		}

		/* convert to target endianess */
		for (i = 0; i < (size/sizeof(uint32_t)); i++)
		{
			uint32_t target_data;
			target_data = target_buffer_get_u32(target, &buffer[i*sizeof(uint32_t)]);
			target_buffer_set_u32(target, &buffer[i*sizeof(uint32_t)], target_data);
		}

		retval = image_calculate_checksum(buffer, size, &checksum);
		free(buffer);
	}

	*crc = checksum;

	return retval;
}

int target_blank_check_memory(struct target_s *target, uint32_t address, uint32_t size, uint32_t* blank)
{
	int retval;
	if (!target_was_examined(target))
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}

	if (target->type->blank_check_memory == 0)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	retval = target->type->blank_check_memory(target, address, size, blank);

	return retval;
}

int target_read_u32(struct target_s *target, uint32_t address, uint32_t *value)
{
	uint8_t value_buf[4];
	if (!target_was_examined(target))
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}

	int retval = target_read_memory(target, address, 4, 1, value_buf);

	if (retval == ERROR_OK)
	{
		*value = target_buffer_get_u32(target, value_buf);
		LOG_DEBUG("address: 0x%8.8" PRIx32 ", value: 0x%8.8" PRIx32 "",
				  address,
				  *value);
	}
	else
	{
		*value = 0x0;
		LOG_DEBUG("address: 0x%8.8" PRIx32 " failed",
				  address);
	}

	return retval;
}

int target_read_u16(struct target_s *target, uint32_t address, uint16_t *value)
{
	uint8_t value_buf[2];
	if (!target_was_examined(target))
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}

	int retval = target_read_memory(target, address, 2, 1, value_buf);

	if (retval == ERROR_OK)
	{
		*value = target_buffer_get_u16(target, value_buf);
		LOG_DEBUG("address: 0x%8.8" PRIx32 ", value: 0x%4.4x",
				  address,
				  *value);
	}
	else
	{
		*value = 0x0;
		LOG_DEBUG("address: 0x%8.8" PRIx32 " failed",
				  address);
	}

	return retval;
}

int target_read_u8(struct target_s *target, uint32_t address, uint8_t *value)
{
	int retval = target_read_memory(target, address, 1, 1, value);
	if (!target_was_examined(target))
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}

	if (retval == ERROR_OK)
	{
		LOG_DEBUG("address: 0x%8.8" PRIx32 ", value: 0x%2.2x",
				  address,
				  *value);
	}
	else
	{
		*value = 0x0;
		LOG_DEBUG("address: 0x%8.8" PRIx32 " failed",
				  address);
	}

	return retval;
}

int target_write_u32(struct target_s *target, uint32_t address, uint32_t value)
{
	int retval;
	uint8_t value_buf[4];
	if (!target_was_examined(target))
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}

	LOG_DEBUG("address: 0x%8.8" PRIx32 ", value: 0x%8.8" PRIx32 "",
			  address,
			  value);

	target_buffer_set_u32(target, value_buf, value);
	if ((retval = target_write_memory(target, address, 4, 1, value_buf)) != ERROR_OK)
	{
		LOG_DEBUG("failed: %i", retval);
	}

	return retval;
}

int target_write_u16(struct target_s *target, uint32_t address, uint16_t value)
{
	int retval;
	uint8_t value_buf[2];
	if (!target_was_examined(target))
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}

	LOG_DEBUG("address: 0x%8.8" PRIx32 ", value: 0x%8.8x",
			  address,
			  value);

	target_buffer_set_u16(target, value_buf, value);
	if ((retval = target_write_memory(target, address, 2, 1, value_buf)) != ERROR_OK)
	{
		LOG_DEBUG("failed: %i", retval);
	}

	return retval;
}

int target_write_u8(struct target_s *target, uint32_t address, uint8_t value)
{
	int retval;
	if (!target_was_examined(target))
	{
		LOG_ERROR("Target not examined yet");
		return ERROR_FAIL;
	}

	LOG_DEBUG("address: 0x%8.8" PRIx32 ", value: 0x%2.2x",
			  address, value);

	if ((retval = target_write_memory(target, address, 1, 1, &value)) != ERROR_OK)
	{
		LOG_DEBUG("failed: %i", retval);
	}

	return retval;
}

int target_register_user_commands(struct command_context_s *cmd_ctx)
{
	int retval = ERROR_OK;


	/* script procedures */
	register_command(cmd_ctx, NULL, "profile", handle_profile_command, COMMAND_EXEC, "profiling samples the CPU PC");
	register_jim(cmd_ctx, "ocd_mem2array", jim_mem2array, "read memory and return as a TCL array for script processing <ARRAYNAME> <WIDTH = 32/16/8> <ADDRESS> <COUNT>");
	register_jim(cmd_ctx, "ocd_array2mem", jim_array2mem, "convert a TCL array to memory locations and write the values  <ARRAYNAME> <WIDTH = 32/16/8> <ADDRESS> <COUNT>");

	register_command(cmd_ctx, NULL, "fast_load_image", handle_fast_load_image_command, COMMAND_ANY,
			"same args as load_image, image stored in memory - mainly for profiling purposes");

	register_command(cmd_ctx, NULL, "fast_load", handle_fast_load_command, COMMAND_ANY,
			"loads active fast load image to current target - mainly for profiling purposes");


	register_command(cmd_ctx, NULL, "virt2phys", handle_virt2phys_command, COMMAND_ANY, "translate a virtual address into a physical address");
	register_command(cmd_ctx,  NULL, "reg", handle_reg_command, COMMAND_EXEC, "display or set a register");
	register_command(cmd_ctx,  NULL, "poll", handle_poll_command, COMMAND_EXEC, "poll target state");
	register_command(cmd_ctx,  NULL, "wait_halt", handle_wait_halt_command, COMMAND_EXEC, "wait for target halt [time (s)]");
	register_command(cmd_ctx,  NULL, "halt", handle_halt_command, COMMAND_EXEC, "halt target");
	register_command(cmd_ctx,  NULL, "resume", handle_resume_command, COMMAND_EXEC, "resume target [addr]");
	register_command(cmd_ctx,  NULL, "step", handle_step_command, COMMAND_EXEC, "step one instruction from current PC or [addr]");
	register_command(cmd_ctx,  NULL, "reset", handle_reset_command, COMMAND_EXEC, "reset target [run | halt | init] - default is run");
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
	register_command(cmd_ctx,  NULL, "test_image", handle_test_image_command, COMMAND_EXEC, "test_image <file> [offset] [type]");

	if ((retval = target_request_register_commands(cmd_ctx)) != ERROR_OK)
		return retval;
	if ((retval = trace_register_commands(cmd_ctx)) != ERROR_OK)
		return retval;

	return retval;
}

static int handle_targets_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = all_targets;

	if (argc == 1)
	{
		target = get_target(args[0]);
		if (target == NULL) {
			command_print(cmd_ctx,"Target: %s is unknown, try one of:\n", args[0]);
			goto DumpTargets;
		}
		if (!target->tap->enabled) {
			command_print(cmd_ctx,"Target: TAP %s is disabled, "
					"can't be the current target\n",
					target->tap->dotted_name);
			return ERROR_FAIL;
		}

		cmd_ctx->current_target = target->target_number;
		return ERROR_OK;
	}
DumpTargets:

	target = all_targets;
	command_print(cmd_ctx, "    TargetName         Type       Endian TapName            State       ");
	command_print(cmd_ctx, "--  ------------------ ---------- ------ ------------------ ------------");
	while (target)
	{
		const char *state;
		char marker = ' ';

		if (target->tap->enabled)
			state = target_state_name( target );
		else
			state = "tap-disabled";

		if (cmd_ctx->current_target == target->target_number)
			marker = '*';

		/* keep columns lined up to match the headers above */
		command_print(cmd_ctx, "%2d%c %-18s %-10s %-6s %-18s %s",
					  target->target_number,
					  marker,
					  target->cmd_name,
					  target_get_name(target),
					  Jim_Nvp_value2name_simple(nvp_target_endian,
								target->endianness)->name,
					  target->tap->dotted_name,
					  state);
		target = target->next;
	}

	return ERROR_OK;
}

/* every 300ms we check for reset & powerdropout and issue a "reset halt" if so. */

static int powerDropout;
static int srstAsserted;

static int runPowerRestore;
static int runPowerDropout;
static int runSrstAsserted;
static int runSrstDeasserted;

static int sense_handler(void)
{
	static int prevSrstAsserted = 0;
	static int prevPowerdropout = 0;

	int retval;
	if ((retval = jtag_power_dropout(&powerDropout)) != ERROR_OK)
		return retval;

	int powerRestored;
	powerRestored = prevPowerdropout && !powerDropout;
	if (powerRestored)
	{
		runPowerRestore = 1;
	}

	long long current = timeval_ms();
	static long long lastPower = 0;
	int waitMore = lastPower + 2000 > current;
	if (powerDropout && !waitMore)
	{
		runPowerDropout = 1;
		lastPower = current;
	}

	if ((retval = jtag_srst_asserted(&srstAsserted)) != ERROR_OK)
		return retval;

	int srstDeasserted;
	srstDeasserted = prevSrstAsserted && !srstAsserted;

	static long long lastSrst = 0;
	waitMore = lastSrst + 2000 > current;
	if (srstDeasserted && !waitMore)
	{
		runSrstDeasserted = 1;
		lastSrst = current;
	}

	if (!prevSrstAsserted && srstAsserted)
	{
		runSrstAsserted = 1;
	}

	prevSrstAsserted = srstAsserted;
	prevPowerdropout = powerDropout;

	if (srstDeasserted || powerRestored)
	{
		/* Other than logging the event we can't do anything here.
		 * Issuing a reset is a particularly bad idea as we might
		 * be inside a reset already.
		 */
	}

	return ERROR_OK;
}

/* process target state changes */
int handle_target(void *priv)
{
	int retval = ERROR_OK;

	/* we do not want to recurse here... */
	static int recursive = 0;
	if (! recursive)
	{
		recursive = 1;
		sense_handler();
		/* danger! running these procedures can trigger srst assertions and power dropouts.
		 * We need to avoid an infinite loop/recursion here and we do that by
		 * clearing the flags after running these events.
		 */
		int did_something = 0;
		if (runSrstAsserted)
		{
			Jim_Eval(interp, "srst_asserted");
			did_something = 1;
		}
		if (runSrstDeasserted)
		{
			Jim_Eval(interp, "srst_deasserted");
			did_something = 1;
		}
		if (runPowerDropout)
		{
			Jim_Eval(interp, "power_dropout");
			did_something = 1;
		}
		if (runPowerRestore)
		{
			Jim_Eval(interp, "power_restore");
			did_something = 1;
		}

		if (did_something)
		{
			/* clear detect flags */
			sense_handler();
		}

		/* clear action flags */

		runSrstAsserted = 0;
		runSrstDeasserted = 0;
		runPowerRestore = 0;
		runPowerDropout = 0;

		recursive = 0;
	}

	/* Poll targets for state changes unless that's globally disabled.
	 * Skip targets that are currently disabled.
	 */
	for (target_t *target = all_targets;
			target_continuous_poll && target;
			target = target->next)
	{
		if (!target->tap->enabled)
			continue;

		/* only poll target if we've got power and srst isn't asserted */
		if (!powerDropout && !srstAsserted)
		{
			/* polling may fail silently until the target has been examined */
			if ((retval = target_poll(target)) != ERROR_OK)
				return retval;
		}
	}

	return retval;
}

static int handle_reg_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
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
		while (cache)
		{
			int i;

			for (i = 0, reg = cache->reg_list;
					i < cache->num_regs;
					i++, reg++, count++)
			{
				/* only print cached values if they are valid */
				if (reg->valid) {
					value = buf_to_str(reg->value,
							reg->size, 16);
					command_print(cmd_ctx,
							"(%i) %s (/%u): 0x%s%s",
							count, reg->name,
							reg->size, value,
							reg->dirty
								? " (dirty)"
								: "");
					free(value);
				} else {
					command_print(cmd_ctx, "(%i) %s (/%u)",
							  count, reg->name,
							  reg->size) ;
				}
			}
			cache = cache->next;
		}

		return ERROR_OK;
	}

	/* access a single register by its ordinal number */
	if ((args[0][0] >= '0') && (args[0][0] <= '9'))
	{
		unsigned num;
		int retval = parse_uint(args[0], &num);
		if (ERROR_OK != retval)
			return ERROR_COMMAND_SYNTAX_ERROR;

		reg_cache_t *cache = target->reg_cache;
		count = 0;
		while (cache)
		{
			int i;
			for (i = 0; i < cache->num_regs; i++)
			{
				if (count++ == (int)num)
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
			arch_type->get(reg);
		}
		value = buf_to_str(reg->value, reg->size, 16);
		command_print(cmd_ctx, "%s (/%i): 0x%s", reg->name, (int)(reg->size), value);
		free(value);
		return ERROR_OK;
	}

	/* set register value */
	if (argc == 2)
	{
		uint8_t *buf = malloc(CEIL(reg->size, 8));
		str_to_buf(args[1], strlen(args[1]), buf, reg->size, 0);

		reg_arch_type_t *arch_type = register_get_arch_type(reg->arch_type);
		arch_type->set(reg, buf);

		value = buf_to_str(reg->value, reg->size, 16);
		command_print(cmd_ctx, "%s (/%i): 0x%s", reg->name, (int)(reg->size), value);
		free(value);

		free(buf);

		return ERROR_OK;
	}

	command_print(cmd_ctx, "usage: reg <#|name> [value]");

	return ERROR_OK;
}

static int handle_poll_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int retval = ERROR_OK;
	target_t *target = get_current_target(cmd_ctx);

	if (argc == 0)
	{
		command_print(cmd_ctx, "background polling: %s",
				target_continuous_poll ?  "on" : "off");
		command_print(cmd_ctx, "TAP: %s (%s)",
				target->tap->dotted_name,
				target->tap->enabled ? "enabled" : "disabled");
		if (!target->tap->enabled)
			return ERROR_OK;
		if ((retval = target_poll(target)) != ERROR_OK)
			return retval;
		if ((retval = target_arch_state(target)) != ERROR_OK)
			return retval;

	}
	else if (argc == 1)
	{
		if (strcmp(args[0], "on") == 0)
		{
			target_continuous_poll = 1;
		}
		else if (strcmp(args[0], "off") == 0)
		{
			target_continuous_poll = 0;
		}
		else
		{
			command_print(cmd_ctx, "arg is \"on\" or \"off\"");
		}
	} else
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return retval;
}

static int handle_wait_halt_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	unsigned ms = 5000;
	if (1 == argc)
	{
		int retval = parse_uint(args[0], &ms);
		if (ERROR_OK != retval)
		{
			command_print(cmd_ctx, "usage: %s [seconds]", cmd);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		// convert seconds (given) to milliseconds (needed)
		ms *= 1000;
	}

	target_t *target = get_current_target(cmd_ctx);
	return target_wait_state(target, TARGET_HALTED, ms);
}

/* wait for target state to change. The trick here is to have a low
 * latency for short waits and not to suck up all the CPU time
 * on longer waits.
 *
 * After 500ms, keep_alive() is invoked
 */
int target_wait_state(target_t *target, enum target_state state, int ms)
{
	int retval;
	long long then = 0, cur;
	int once = 1;

	for (;;)
	{
		if ((retval = target_poll(target)) != ERROR_OK)
			return retval;
		if (target->state == state)
		{
			break;
		}
		cur = timeval_ms();
		if (once)
		{
			once = 0;
			then = timeval_ms();
			LOG_DEBUG("waiting for target %s...",
				Jim_Nvp_value2name_simple(nvp_target_state,state)->name);
		}

		if (cur-then > 500)
		{
			keep_alive();
		}

		if ((cur-then) > ms)
		{
			LOG_ERROR("timed out while waiting for target %s",
				Jim_Nvp_value2name_simple(nvp_target_state,state)->name);
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int handle_halt_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	LOG_DEBUG("-");

	target_t *target = get_current_target(cmd_ctx);
	int retval = target_halt(target);
	if (ERROR_OK != retval)
		return retval;

	if (argc == 1)
	{
		unsigned wait;
		retval = parse_uint(args[0], &wait);
		if (ERROR_OK != retval)
			return ERROR_COMMAND_SYNTAX_ERROR;
		if (!wait)
			return ERROR_OK;
	}

	return handle_wait_halt_command(cmd_ctx, cmd, args, argc);
}

static int handle_soft_reset_halt_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);

	LOG_USER("requesting target halt and executing a soft reset");

	target->type->soft_reset_halt(target);

	return ERROR_OK;
}

static int handle_reset_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	enum target_reset_mode reset_mode = RESET_RUN;
	if (argc == 1)
	{
		const Jim_Nvp *n;
		n = Jim_Nvp_name2value_simple(nvp_reset_modes, args[0]);
		if ((n->name == NULL) || (n->value == RESET_UNKNOWN)) {
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		reset_mode = n->value;
	}

	/* reset *all* targets */
	return target_process_reset(cmd_ctx, reset_mode);
}


static int handle_resume_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int current = 1;
	if (argc > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	target_t *target = get_current_target(cmd_ctx);
	target_handle_event(target, TARGET_EVENT_OLD_pre_resume);

	/* with no args, resume from current pc, addr = 0,
	 * with one arguments, addr = args[0],
	 * handle breakpoints, not debugging */
	uint32_t addr = 0;
	if (argc == 1)
	{
		int retval = parse_u32(args[0], &addr);
		if (ERROR_OK != retval)
			return retval;
		current = 0;
	}

	return target_resume(target, current, addr, 1, 0);
}

static int handle_step_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	LOG_DEBUG("-");

	/* with no args, step from current pc, addr = 0,
	 * with one argument addr = args[0],
	 * handle breakpoints, debugging */
	uint32_t addr = 0;
	int current_pc = 1;
	if (argc == 1)
	{
		int retval = parse_u32(args[0], &addr);
		if (ERROR_OK != retval)
			return retval;
		current_pc = 0;
	}

	target_t *target = get_current_target(cmd_ctx);

	return target->type->step(target, current_pc, addr, 1);
}

static void handle_md_output(struct command_context_s *cmd_ctx,
		struct target_s *target, uint32_t address, unsigned size,
		unsigned count, const uint8_t *buffer)
{
	const unsigned line_bytecnt = 32;
	unsigned line_modulo = line_bytecnt / size;

	char output[line_bytecnt * 4 + 1];
	unsigned output_len = 0;

	const char *value_fmt;
	switch (size) {
	case 4: value_fmt = "%8.8x "; break;
	case 2: value_fmt = "%4.2x "; break;
	case 1: value_fmt = "%2.2x "; break;
	default:
		LOG_ERROR("invalid memory read size: %u", size);
		exit(-1);
	}

	for (unsigned i = 0; i < count; i++)
	{
		if (i % line_modulo == 0)
		{
			output_len += snprintf(output + output_len,
					sizeof(output) - output_len,
					"0x%8.8x: ",
					(unsigned)(address + (i*size)));
		}

		uint32_t value = 0;
		const uint8_t *value_ptr = buffer + i * size;
		switch (size) {
		case 4: value = target_buffer_get_u32(target, value_ptr); break;
		case 2: value = target_buffer_get_u16(target, value_ptr); break;
		case 1: value = *value_ptr;
		}
		output_len += snprintf(output + output_len,
				sizeof(output) - output_len,
				value_fmt, value);

		if ((i % line_modulo == line_modulo - 1) || (i == count - 1))
		{
			command_print(cmd_ctx, "%s", output);
			output_len = 0;
		}
	}
}

static int handle_md_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	unsigned size = 0;
	switch (cmd[2]) {
	case 'w': size = 4; break;
	case 'h': size = 2; break;
	case 'b': size = 1; break;
	default: return ERROR_COMMAND_SYNTAX_ERROR;
	}

	uint32_t address;
	int retval = parse_u32(args[0], &address);
	if (ERROR_OK != retval)
		return retval;

	unsigned count = 1;
	if (argc == 2)
	{
		retval = parse_uint(args[1], &count);
		if (ERROR_OK != retval)
			return retval;
	}

	uint8_t *buffer = calloc(count, size);

	target_t *target = get_current_target(cmd_ctx);
	retval = target_read_memory(target,
				address, size, count, buffer);
	if (ERROR_OK == retval)
		handle_md_output(cmd_ctx, target, address, size, count, buffer);

	free(buffer);

	return retval;
}

static int handle_mw_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	 if ((argc < 2) || (argc > 3))
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t address;
	int retval = parse_u32(args[0], &address);
	if (ERROR_OK != retval)
		return retval;

	uint32_t value;
	retval = parse_u32(args[1], &value);
	if (ERROR_OK != retval)
		return retval;

	unsigned count = 1;
	if (argc == 3)
	{
		retval = parse_uint(args[2], &count);
		if (ERROR_OK != retval)
			return retval;
	}

	target_t *target = get_current_target(cmd_ctx);
	unsigned wordsize;
	uint8_t value_buf[4];
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
	for (unsigned i = 0; i < count; i++)
	{
		retval = target_write_memory(target,
				address + i * wordsize, wordsize, 1, value_buf);
		if (ERROR_OK != retval)
			return retval;
		keep_alive();
	}

	return ERROR_OK;

}

static int parse_load_image_command_args(char **args, int argc,
		image_t *image, uint32_t *min_address, uint32_t *max_address)
{
	if (argc < 1 || argc > 5)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* a base address isn't always necessary,
	 * default to 0x0 (i.e. don't relocate) */
	if (argc >= 2)
	{
		uint32_t addr;
		int retval = parse_u32(args[1], &addr);
		if (ERROR_OK != retval)
			return ERROR_COMMAND_SYNTAX_ERROR;
		image->base_address = addr;
		image->base_address_set = 1;
	}
	else
		image->base_address_set = 0;

	image->start_address_set = 0;

	if (argc >= 4)
	{
		int retval = parse_u32(args[3], min_address);
		if (ERROR_OK != retval)
			return ERROR_COMMAND_SYNTAX_ERROR;
	}
	if (argc == 5)
	{
		int retval = parse_u32(args[4], max_address);
		if (ERROR_OK != retval)
			return ERROR_COMMAND_SYNTAX_ERROR;
		// use size (given) to find max (required)
		*max_address += *min_address;
	}

	if (*min_address > *max_address)
		return ERROR_COMMAND_SYNTAX_ERROR;

	return ERROR_OK;
}

static int handle_load_image_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	uint8_t *buffer;
	uint32_t buf_cnt;
	uint32_t image_size;
	uint32_t min_address = 0;
	uint32_t max_address = 0xffffffff;
	int i;
	int retvaltemp;

	image_t image;

	duration_t duration;
	char *duration_text;

	int retval = parse_load_image_command_args(args, argc,
			&image, &min_address, &max_address);
	if (ERROR_OK != retval)
		return retval;

	target_t *target = get_current_target(cmd_ctx);
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
			command_print(cmd_ctx,
						  "error allocating buffer for section (%d bytes)",
						  (int)(image.sections[i].size));
			break;
		}

		if ((retval = image_read_section(&image, i, 0x0, image.sections[i].size, buffer, &buf_cnt)) != ERROR_OK)
		{
			free(buffer);
			break;
		}

		uint32_t offset = 0;
		uint32_t length = buf_cnt;

		/* DANGER!!! beware of unsigned comparision here!!! */

		if ((image.sections[i].base_address + buf_cnt >= min_address)&&
				(image.sections[i].base_address < max_address))
		{
			if (image.sections[i].base_address < min_address)
			{
				/* clip addresses below */
				offset += min_address-image.sections[i].base_address;
				length -= offset;
			}

			if (image.sections[i].base_address + buf_cnt > max_address)
			{
				length -= (image.sections[i].base_address + buf_cnt)-max_address;
			}

			if ((retval = target_write_buffer(target, image.sections[i].base_address + offset, length, buffer + offset)) != ERROR_OK)
			{
				free(buffer);
				break;
			}
			image_size += length;
			command_print(cmd_ctx, "%u byte written at address 0x%8.8" PRIx32 "",
						  (unsigned int)length,
						  image.sections[i].base_address + offset);
		}

		free(buffer);
	}

	if ((retvaltemp = duration_stop_measure(&duration, &duration_text)) != ERROR_OK)
	{
		image_close(&image);
		return retvaltemp;
	}

	if (retval == ERROR_OK)
	{
		command_print(cmd_ctx, "downloaded %u byte in %s",
					  (unsigned int)image_size,
					  duration_text);
	}
	free(duration_text);

	image_close(&image);

	return retval;

}

static int handle_dump_image_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	fileio_t fileio;

	uint8_t buffer[560];
	int retvaltemp;

	duration_t duration;
	char *duration_text;

	target_t *target = get_current_target(cmd_ctx);

	if (argc != 3)
	{
		command_print(cmd_ctx, "usage: dump_image <filename> <address> <size>");
		return ERROR_OK;
	}

	uint32_t address;
	int retval = parse_u32(args[1], &address);
	if (ERROR_OK != retval)
		return retval;

	uint32_t size;
	retval = parse_u32(args[2], &size);
	if (ERROR_OK != retval)
		return retval;

	if (fileio_open(&fileio, args[0], FILEIO_WRITE, FILEIO_BINARY) != ERROR_OK)
	{
		return ERROR_OK;
	}

	duration_start_measure(&duration);

	while (size > 0)
	{
		uint32_t size_written;
		uint32_t this_run_size = (size > 560) ? 560 : size;

		retval = target_read_buffer(target, address, this_run_size, buffer);
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

	if ((retvaltemp = fileio_close(&fileio)) != ERROR_OK)
		return retvaltemp;

	if ((retvaltemp = duration_stop_measure(&duration, &duration_text)) != ERROR_OK)
		return retvaltemp;

	if (retval == ERROR_OK)
	{
		command_print(cmd_ctx, "dumped %lld byte in %s",
				fileio.size, duration_text);
		free(duration_text);
	}

	return retval;
}

static int handle_verify_image_command_internal(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, int verify)
{
	uint8_t *buffer;
	uint32_t buf_cnt;
	uint32_t image_size;
	int i;
	int retval, retvaltemp;
	uint32_t checksum = 0;
	uint32_t mem_checksum = 0;

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
		uint32_t addr;
		retval = parse_u32(args[1], &addr);
		if (ERROR_OK != retval)
			return ERROR_COMMAND_SYNTAX_ERROR;
		image.base_address = addr;
		image.base_address_set = 1;
	}
	else
	{
		image.base_address_set = 0;
		image.base_address = 0x0;
	}

	image.start_address_set = 0;

	if ((retval = image_open(&image, args[0], (argc == 3) ? args[2] : NULL)) != ERROR_OK)
	{
		return retval;
	}

	image_size = 0x0;
	retval = ERROR_OK;
	for (i = 0; i < image.num_sections; i++)
	{
		buffer = malloc(image.sections[i].size);
		if (buffer == NULL)
		{
			command_print(cmd_ctx,
						  "error allocating buffer for section (%d bytes)",
						  (int)(image.sections[i].size));
			break;
		}
		if ((retval = image_read_section(&image, i, 0x0, image.sections[i].size, buffer, &buf_cnt)) != ERROR_OK)
		{
			free(buffer);
			break;
		}

		if (verify)
		{
			/* calculate checksum of image */
			image_calculate_checksum(buffer, buf_cnt, &checksum);

			retval = target_checksum_memory(target, image.sections[i].base_address, buf_cnt, &mem_checksum);
			if (retval != ERROR_OK)
			{
				free(buffer);
				break;
			}

			if (checksum != mem_checksum)
			{
				/* failed crc checksum, fall back to a binary compare */
				uint8_t *data;

				command_print(cmd_ctx, "checksum mismatch - attempting binary compare");

				data = (uint8_t*)malloc(buf_cnt);

				/* Can we use 32bit word accesses? */
				int size = 1;
				int count = buf_cnt;
				if ((count % 4) == 0)
				{
					size *= 4;
					count /= 4;
				}
				retval = target_read_memory(target, image.sections[i].base_address, size, count, data);
				if (retval == ERROR_OK)
				{
					uint32_t t;
					for (t = 0; t < buf_cnt; t++)
					{
						if (data[t] != buffer[t])
						{
							command_print(cmd_ctx,
										  "Verify operation failed address 0x%08x. Was 0x%02x instead of 0x%02x\n",
										  (unsigned)(t + image.sections[i].base_address),
										  data[t],
										  buffer[t]);
							free(data);
							free(buffer);
							retval = ERROR_FAIL;
							goto done;
						}
						if ((t%16384) == 0)
						{
							keep_alive();
						}
					}
				}

				free(data);
			}
		} else
		{
			command_print(cmd_ctx, "address 0x%08" PRIx32 " length 0x%08" PRIx32 "",
						  image.sections[i].base_address,
						  buf_cnt);
		}

		free(buffer);
		image_size += buf_cnt;
	}
done:

	if ((retvaltemp = duration_stop_measure(&duration, &duration_text)) != ERROR_OK)
	{
		image_close(&image);
		return retvaltemp;
	}

	if (retval == ERROR_OK)
	{
		command_print(cmd_ctx, "verified %u bytes in %s",
					  (unsigned int)image_size,
					  duration_text);
	}
	free(duration_text);

	image_close(&image);

	return retval;
}

static int handle_verify_image_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	return handle_verify_image_command_internal(cmd_ctx, cmd, args, argc, 1);
}

static int handle_test_image_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	return handle_verify_image_command_internal(cmd_ctx, cmd, args, argc, 0);
}

static int handle_bp_command_list(struct command_context_s *cmd_ctx)
{
	target_t *target = get_current_target(cmd_ctx);
	breakpoint_t *breakpoint = target->breakpoints;
	while (breakpoint)
	{
		if (breakpoint->type == BKPT_SOFT)
		{
			char* buf = buf_to_str(breakpoint->orig_instr,
					breakpoint->length, 16);
			command_print(cmd_ctx, "0x%8.8" PRIx32 ", 0x%x, %i, 0x%s",
					breakpoint->address,
					breakpoint->length,
					breakpoint->set, buf);
			free(buf);
		}
		else
		{
			command_print(cmd_ctx, "0x%8.8" PRIx32 ", 0x%x, %i",
						  breakpoint->address,
						  breakpoint->length, breakpoint->set);
		}

		breakpoint = breakpoint->next;
	}
	return ERROR_OK;
}

static int handle_bp_command_set(struct command_context_s *cmd_ctx,
		uint32_t addr, uint32_t length, int hw)
{
	target_t *target = get_current_target(cmd_ctx);
	int retval = breakpoint_add(target, addr, length, hw);
	if (ERROR_OK == retval)
		command_print(cmd_ctx, "breakpoint set at 0x%8.8" PRIx32 "", addr);
	else
		LOG_ERROR("Failure setting breakpoint");
	return retval;
}

static int handle_bp_command(struct command_context_s *cmd_ctx,
		char *cmd, char **args, int argc)
{
	if (argc == 0)
		return handle_bp_command_list(cmd_ctx);

	if (argc < 2 || argc > 3)
	{
		command_print(cmd_ctx, "usage: bp <address> <length> ['hw']");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	uint32_t addr;
	int retval = parse_u32(args[0], &addr);
	if (ERROR_OK != retval)
		return retval;

	uint32_t length;
	retval = parse_u32(args[1], &length);
	if (ERROR_OK != retval)
		return retval;

	int hw = BKPT_SOFT;
	if (argc == 3)
	{
		if (strcmp(args[2], "hw") == 0)
			hw = BKPT_HARD;
		else
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return handle_bp_command_set(cmd_ctx, addr, length, hw);
}

static int handle_rbp_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t addr;
	int retval = parse_u32(args[0], &addr);
	if (ERROR_OK != retval)
		return retval;

	target_t *target = get_current_target(cmd_ctx);
	breakpoint_remove(target, addr);

	return ERROR_OK;
}

static int handle_wp_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);

	if (argc == 0)
	{
		watchpoint_t *watchpoint = target->watchpoints;

		while (watchpoint)
		{
			command_print(cmd_ctx,
						  "address: 0x%8.8" PRIx32 ", len: 0x%8.8x, r/w/a: %i, value: 0x%8.8" PRIx32 ", mask: 0x%8.8" PRIx32 "",
						  watchpoint->address,
						  watchpoint->length,
						  (int)(watchpoint->rw),
						  watchpoint->value,
						  watchpoint->mask);
			watchpoint = watchpoint->next;
		}
		return ERROR_OK;
	}

	enum watchpoint_rw type = WPT_ACCESS;
	uint32_t addr = 0;
	uint32_t length = 0;
	uint32_t data_value = 0x0;
	uint32_t data_mask = 0xffffffff;
	int retval;

	switch (argc)
	{
	case 5:
		retval = parse_u32(args[4], &data_mask);
		if (ERROR_OK != retval)
			return retval;
		// fall through
	case 4:
		retval = parse_u32(args[3], &data_value);
		if (ERROR_OK != retval)
			return retval;
		// fall through
	case 3:
		switch (args[2][0])
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
			LOG_ERROR("invalid watchpoint mode ('%c')", args[2][0]);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		// fall through
	case 2:
		retval = parse_u32(args[1], &length);
		if (ERROR_OK != retval)
			return retval;
		retval = parse_u32(args[0], &addr);
		if (ERROR_OK != retval)
			return retval;
		break;

	default:
		command_print(cmd_ctx, "usage: wp <address> <length> [r/w/a] [value] [mask]");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	retval = watchpoint_add(target, addr, length, type,
			data_value, data_mask);
	if (ERROR_OK != retval)
		LOG_ERROR("Failure setting watchpoints");

	return retval;
}

static int handle_rwp_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t addr;
	int retval = parse_u32(args[0], &addr);
	if (ERROR_OK != retval)
		return retval;

	target_t *target = get_current_target(cmd_ctx);
	watchpoint_remove(target, addr);

	return ERROR_OK;
}


/**
 * Translate a virtual address to a physical address.
 *
 * The low-level target implementation must have logged a detailed error
 * which is forwarded to telnet/GDB session.
 */
static int handle_virt2phys_command(command_context_t *cmd_ctx,
		char *cmd, char **args, int argc)
{
	if (argc != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t va;
	int retval = parse_u32(args[0], &va);
	if (ERROR_OK != retval)
		return retval;
	uint32_t pa;

	target_t *target = get_current_target(cmd_ctx);
	retval = target->type->virt2phys(target, va, &pa);
	if (retval == ERROR_OK)
		command_print(cmd_ctx, "Physical address 0x%08" PRIx32 "", pa);

	return retval;
}

static void writeData(FILE *f, const void *data, size_t len)
{
	size_t written = fwrite(data, 1, len, f);
	if (written != len)
		LOG_ERROR("failed to write %zu bytes: %s", len, strerror(errno));
}

static void writeLong(FILE *f, int l)
{
	int i;
	for (i = 0; i < 4; i++)
	{
		char c = (l >> (i*8))&0xff;
		writeData(f, &c, 1);
	}

}

static void writeString(FILE *f, char *s)
{
	writeData(f, s, strlen(s));
}

/* Dump a gmon.out histogram file. */
static void writeGmon(uint32_t *samples, uint32_t sampleNum, char *filename)
{
	uint32_t i;
	FILE *f = fopen(filename, "w");
	if (f == NULL)
		return;
	writeString(f, "gmon");
	writeLong(f, 0x00000001); /* Version */
	writeLong(f, 0); /* padding */
	writeLong(f, 0); /* padding */
	writeLong(f, 0); /* padding */

	uint8_t zero = 0;  /* GMON_TAG_TIME_HIST */
	writeData(f, &zero, 1);

	/* figure out bucket size */
	uint32_t min = samples[0];
	uint32_t max = samples[0];
	for (i = 0; i < sampleNum; i++)
	{
		if (min > samples[i])
		{
			min = samples[i];
		}
		if (max < samples[i])
		{
			max = samples[i];
		}
	}

	int addressSpace = (max-min + 1);

	static const uint32_t maxBuckets = 256 * 1024; /* maximum buckets. */
	uint32_t length = addressSpace;
	if (length > maxBuckets)
	{
		length = maxBuckets;
	}
	int *buckets = malloc(sizeof(int)*length);
	if (buckets == NULL)
	{
		fclose(f);
		return;
	}
	memset(buckets, 0, sizeof(int)*length);
	for (i = 0; i < sampleNum;i++)
	{
		uint32_t address = samples[i];
		long long a = address-min;
		long long b = length-1;
		long long c = addressSpace-1;
		int index = (a*b)/c; /* danger!!!! int32 overflows */
		buckets[index]++;
	}

	/* append binary memory gmon.out &profile_hist_hdr ((char*)&profile_hist_hdr + sizeof(struct gmon_hist_hdr)) */
	writeLong(f, min); 			/* low_pc */
	writeLong(f, max);			/* high_pc */
	writeLong(f, length);		/* # of samples */
	writeLong(f, 64000000); 	/* 64MHz */
	writeString(f, "seconds");
	for (i = 0; i < (15-strlen("seconds")); i++)
		writeData(f, &zero, 1);
	writeString(f, "s");

	/*append binary memory gmon.out profile_hist_data (profile_hist_data + profile_hist_hdr.hist_size) */

	char *data = malloc(2*length);
	if (data != NULL)
	{
		for (i = 0; i < length;i++)
		{
			int val;
			val = buckets[i];
			if (val > 65535)
			{
				val = 65535;
			}
			data[i*2]=val&0xff;
			data[i*2 + 1]=(val >> 8)&0xff;
		}
		free(buckets);
		writeData(f, data, length * 2);
		free(data);
	} else
	{
		free(buckets);
	}

	fclose(f);
}

/* profiling samples the CPU PC as quickly as OpenOCD is able, which will be used as a random sampling of PC */
static int handle_profile_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	struct timeval timeout, now;

	gettimeofday(&timeout, NULL);
	if (argc != 2)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	unsigned offset;
	int retval = parse_uint(args[0], &offset);
	if (ERROR_OK != retval)
		return retval;

	timeval_add_time(&timeout, offset, 0);

	command_print(cmd_ctx, "Starting profiling. Halting and resuming the target as often as we can...");

	static const int maxSample = 10000;
	uint32_t *samples = malloc(sizeof(uint32_t)*maxSample);
	if (samples == NULL)
		return ERROR_OK;

	int numSamples = 0;
	/* hopefully it is safe to cache! We want to stop/restart as quickly as possible. */
	reg_t *reg = register_get_by_name(target->reg_cache, "pc", 1);

	for (;;)
	{
		target_poll(target);
		if (target->state == TARGET_HALTED)
		{
			uint32_t t=*((uint32_t *)reg->value);
			samples[numSamples++]=t;
			retval = target_resume(target, 1, 0, 0, 0); /* current pc, addr = 0, do not handle breakpoints, not debugging */
			target_poll(target);
			alive_sleep(10); /* sleep 10ms, i.e. <100 samples/second. */
		} else if (target->state == TARGET_RUNNING)
		{
			/* We want to quickly sample the PC. */
			if ((retval = target_halt(target)) != ERROR_OK)
			{
				free(samples);
				return retval;
			}
		} else
		{
			command_print(cmd_ctx, "Target not halted or running");
			retval = ERROR_OK;
			break;
		}
		if (retval != ERROR_OK)
		{
			break;
		}

		gettimeofday(&now, NULL);
		if ((numSamples >= maxSample) || ((now.tv_sec >= timeout.tv_sec) && (now.tv_usec >= timeout.tv_usec)))
		{
			command_print(cmd_ctx, "Profiling completed. %d samples.", numSamples);
			if ((retval = target_poll(target)) != ERROR_OK)
			{
				free(samples);
				return retval;
			}
			if (target->state == TARGET_HALTED)
			{
				target_resume(target, 1, 0, 0, 0); /* current pc, addr = 0, do not handle breakpoints, not debugging */
			}
			if ((retval = target_poll(target)) != ERROR_OK)
			{
				free(samples);
				return retval;
			}
			writeGmon(samples, numSamples, args[1]);
			command_print(cmd_ctx, "Wrote %s", args[1]);
			break;
		}
	}
	free(samples);

	return ERROR_OK;
}

static int new_int_array_element(Jim_Interp * interp, const char *varname, int idx, uint32_t val)
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
	command_context_t *context;
	target_t *target;

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

	return 	target_mem2array(interp, target, argc-1, argv + 1);
}

static int target_mem2array(Jim_Interp *interp, target_t *target, int argc, Jim_Obj *const *argv)
{
	long l;
	uint32_t width;
	int len;
	uint32_t addr;
	uint32_t count;
	uint32_t v;
	const char *varname;
	uint8_t buffer[4096];
	int  n, e, retval;
	uint32_t i;

	/* argv[1] = name of array to receive the data
	 * argv[2] = desired width
	 * argv[3] = memory address
	 * argv[4] = count of times to read
	 */
	if (argc != 4) {
		Jim_WrongNumArgs(interp, 1, argv, "varname width addr nelems");
		return JIM_ERR;
	}
	varname = Jim_GetString(argv[0], &len);
	/* given "foo" get space for worse case "foo(%d)" .. add 20 */

	e = Jim_GetLong(interp, argv[1], &l);
	width = l;
	if (e != JIM_OK) {
		return e;
	}

	e = Jim_GetLong(interp, argv[2], &l);
	addr = l;
	if (e != JIM_OK) {
		return e;
	}
	e = Jim_GetLong(interp, argv[3], &l);
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
			Jim_AppendStrings(interp, Jim_GetResult(interp), "Invalid width param, must be 8/16/32", NULL);
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
		sprintf(buf, "mem2array address: 0x%08" PRIx32 " is not aligned for %" PRId32 " byte reads",
				addr,
				width);
		Jim_AppendStrings(interp, Jim_GetResult(interp), buf , NULL);
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

		retval = target_read_memory(target, addr, width, count, buffer);
		if (retval != ERROR_OK) {
			/* BOO !*/
			LOG_ERROR("mem2array: Read @ 0x%08x, w=%d, cnt=%d, failed",
					  (unsigned int)addr,
					  (int)width,
					  (int)count);
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

static int get_int_array_element(Jim_Interp * interp, const char *varname, int idx, uint32_t *val)
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
	command_context_t *context;
	target_t *target;

	context = Jim_GetAssocData(interp, "context");
	if (context == NULL) {
		LOG_ERROR("array2mem: no command context");
		return JIM_ERR;
	}
	target = get_current_target(context);
	if (target == NULL) {
		LOG_ERROR("array2mem: no current target");
		return JIM_ERR;
	}

	return target_array2mem(interp,target, argc-1, argv + 1);
}

static int target_array2mem(Jim_Interp *interp, target_t *target, int argc, Jim_Obj *const *argv)
{
	long l;
	uint32_t width;
	int len;
	uint32_t addr;
	uint32_t count;
	uint32_t v;
	const char *varname;
	uint8_t buffer[4096];
	int  n, e, retval;
	uint32_t i;

	/* argv[1] = name of array to get the data
	 * argv[2] = desired width
	 * argv[3] = memory address
	 * argv[4] = count to write
	 */
	if (argc != 4) {
		Jim_WrongNumArgs(interp, 1, argv, "varname width addr nelems");
		return JIM_ERR;
	}
	varname = Jim_GetString(argv[0], &len);
	/* given "foo" get space for worse case "foo(%d)" .. add 20 */

	e = Jim_GetLong(interp, argv[1], &l);
	width = l;
	if (e != JIM_OK) {
		return e;
	}

	e = Jim_GetLong(interp, argv[2], &l);
	addr = l;
	if (e != JIM_OK) {
		return e;
	}
	e = Jim_GetLong(interp, argv[3], &l);
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
			Jim_AppendStrings(interp, Jim_GetResult(interp), "Invalid width param, must be 8/16/32", NULL);
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
		sprintf(buf, "array2mem address: 0x%08x is not aligned for %d byte reads",
				(unsigned int)addr,
				(int)width);
		Jim_AppendStrings(interp, Jim_GetResult(interp), buf , NULL);
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

		retval = target_write_memory(target, addr, width, count, buffer);
		if (retval != ERROR_OK) {
			/* BOO !*/
			LOG_ERROR("array2mem: Write @ 0x%08x, w=%d, cnt=%d, failed",
					  (unsigned int)addr,
					  (int)width,
					  (int)count);
			Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
			Jim_AppendStrings(interp, Jim_GetResult(interp), "array2mem: cannot read memory", NULL);
			e = JIM_ERR;
			len = 0;
		}
	}

	Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));

	return JIM_OK;
}

void target_all_handle_event(enum target_event e)
{
	target_t *target;

	LOG_DEBUG("**all*targets: event: %d, %s",
			   (int)e,
			   Jim_Nvp_value2name_simple(nvp_target_event, e)->name);

	target = all_targets;
	while (target) {
		target_handle_event(target, e);
		target = target->next;
	}
}

void target_handle_event(target_t *target, enum target_event e)
{
	target_event_action_t *teap;
	int done;

	teap = target->event_action;

	done = 0;
	while (teap) {
		if (teap->event == e) {
			done = 1;
			LOG_DEBUG("target: (%d) %s (%s) event: %d (%s) action: %s",
					   target->target_number,
					   target->cmd_name,
					   target_get_name(target),
					   e,
					   Jim_Nvp_value2name_simple(nvp_target_event, e)->name,
					   Jim_GetString(teap->body, NULL));
			if (Jim_EvalObj(interp, teap->body) != JIM_OK)
			{
				Jim_PrintErrorMessage(interp);
			}
		}
		teap = teap->next;
	}
	if (!done) {
		LOG_DEBUG("event: %d %s - no action",
				   e,
				   Jim_Nvp_value2name_simple(nvp_target_event, e)->name);
	}
}

enum target_cfg_param {
	TCFG_TYPE,
	TCFG_EVENT,
	TCFG_WORK_AREA_VIRT,
	TCFG_WORK_AREA_PHYS,
	TCFG_WORK_AREA_SIZE,
	TCFG_WORK_AREA_BACKUP,
	TCFG_ENDIAN,
	TCFG_VARIANT,
	TCFG_CHAIN_POSITION,
};

static Jim_Nvp nvp_config_opts[] = {
	{ .name = "-type",             .value = TCFG_TYPE },
	{ .name = "-event",            .value = TCFG_EVENT },
	{ .name = "-work-area-virt",   .value = TCFG_WORK_AREA_VIRT },
	{ .name = "-work-area-phys",   .value = TCFG_WORK_AREA_PHYS },
	{ .name = "-work-area-size",   .value = TCFG_WORK_AREA_SIZE },
	{ .name = "-work-area-backup", .value = TCFG_WORK_AREA_BACKUP },
	{ .name = "-endian" ,          .value = TCFG_ENDIAN },
	{ .name = "-variant",          .value = TCFG_VARIANT },
	{ .name = "-chain-position",   .value = TCFG_CHAIN_POSITION },

	{ .name = NULL, .value = -1 }
};

static int target_configure(Jim_GetOptInfo *goi, target_t *target)
{
	Jim_Nvp *n;
	Jim_Obj *o;
	jim_wide w;
	char *cp;
	int e;

	/* parse config or cget options ... */
	while (goi->argc > 0) {
		Jim_SetEmptyResult(goi->interp);
		/* Jim_GetOpt_Debug(goi); */

		if (target->type->target_jim_configure) {
			/* target defines a configure function */
			/* target gets first dibs on parameters */
			e = (*(target->type->target_jim_configure))(target, goi);
			if (e == JIM_OK) {
				/* more? */
				continue;
			}
			if (e == JIM_ERR) {
				/* An error */
				return e;
			}
			/* otherwise we 'continue' below */
		}
		e = Jim_GetOpt_Nvp(goi, nvp_config_opts, &n);
		if (e != JIM_OK) {
			Jim_GetOpt_NvpUnknown(goi, nvp_config_opts, 0);
			return e;
		}
		switch (n->value) {
		case TCFG_TYPE:
			/* not setable */
			if (goi->isconfigure) {
				Jim_SetResult_sprintf(goi->interp, "not setable: %s", n->name);
				return JIM_ERR;
			} else {
			no_params:
				if (goi->argc != 0) {
					Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv, "NO PARAMS");
					return JIM_ERR;
				}
			}
			Jim_SetResultString(goi->interp, target_get_name(target), -1);
			/* loop for more */
			break;
		case TCFG_EVENT:
			if (goi->argc == 0) {
				Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv, "-event ?event-name? ...");
				return JIM_ERR;
			}

			e = Jim_GetOpt_Nvp(goi, nvp_target_event, &n);
			if (e != JIM_OK) {
				Jim_GetOpt_NvpUnknown(goi, nvp_target_event, 1);
				return e;
			}

			if (goi->isconfigure) {
				if (goi->argc != 1) {
					Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv, "-event ?event-name? ?EVENT-BODY?");
					return JIM_ERR;
				}
			} else {
				if (goi->argc != 0) {
					Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv, "-event ?event-name?");
					return JIM_ERR;
				}
			}

			{
				target_event_action_t *teap;

				teap = target->event_action;
				/* replace existing? */
				while (teap) {
					if (teap->event == (enum target_event)n->value) {
						break;
					}
					teap = teap->next;
				}

				if (goi->isconfigure) {
					if (teap == NULL) {
						/* create new */
						teap = calloc(1, sizeof(*teap));
					}
					teap->event = n->value;
					Jim_GetOpt_Obj(goi, &o);
					if (teap->body) {
						Jim_DecrRefCount(interp, teap->body);
					}
					teap->body  = Jim_DuplicateObj(goi->interp, o);
					/*
					 * FIXME:
					 *     Tcl/TK - "tk events" have a nice feature.
					 *     See the "BIND" command.
					 *    We should support that here.
					 *     You can specify %X and %Y in the event code.
					 *     The idea is: %T - target name.
					 *     The idea is: %N - target number
					 *     The idea is: %E - event name.
					 */
					Jim_IncrRefCount(teap->body);

					/* add to head of event list */
					teap->next = target->event_action;
					target->event_action = teap;
					Jim_SetEmptyResult(goi->interp);
				} else {
					/* get */
					if (teap == NULL) {
						Jim_SetEmptyResult(goi->interp);
					} else {
						Jim_SetResult(goi->interp, Jim_DuplicateObj(goi->interp, teap->body));
					}
				}
			}
			/* loop for more */
			break;

		case TCFG_WORK_AREA_VIRT:
			if (goi->isconfigure) {
				target_free_all_working_areas(target);
				e = Jim_GetOpt_Wide(goi, &w);
				if (e != JIM_OK) {
					return e;
				}
				target->working_area_virt = w;
			} else {
				if (goi->argc != 0) {
					goto no_params;
				}
			}
			Jim_SetResult(interp, Jim_NewIntObj(goi->interp, target->working_area_virt));
			/* loop for more */
			break;

		case TCFG_WORK_AREA_PHYS:
			if (goi->isconfigure) {
				target_free_all_working_areas(target);
				e = Jim_GetOpt_Wide(goi, &w);
				if (e != JIM_OK) {
					return e;
				}
				target->working_area_phys = w;
			} else {
				if (goi->argc != 0) {
					goto no_params;
				}
			}
			Jim_SetResult(interp, Jim_NewIntObj(goi->interp, target->working_area_phys));
			/* loop for more */
			break;

		case TCFG_WORK_AREA_SIZE:
			if (goi->isconfigure) {
				target_free_all_working_areas(target);
				e = Jim_GetOpt_Wide(goi, &w);
				if (e != JIM_OK) {
					return e;
				}
				target->working_area_size = w;
			} else {
				if (goi->argc != 0) {
					goto no_params;
				}
			}
			Jim_SetResult(interp, Jim_NewIntObj(goi->interp, target->working_area_size));
			/* loop for more */
			break;

		case TCFG_WORK_AREA_BACKUP:
			if (goi->isconfigure) {
				target_free_all_working_areas(target);
				e = Jim_GetOpt_Wide(goi, &w);
				if (e != JIM_OK) {
					return e;
				}
				/* make this exactly 1 or 0 */
				target->backup_working_area = (!!w);
			} else {
				if (goi->argc != 0) {
					goto no_params;
				}
			}
			Jim_SetResult(interp, Jim_NewIntObj(goi->interp, target->backup_working_area));
			/* loop for more e*/
			break;

		case TCFG_ENDIAN:
			if (goi->isconfigure) {
				e = Jim_GetOpt_Nvp(goi, nvp_target_endian, &n);
				if (e != JIM_OK) {
					Jim_GetOpt_NvpUnknown(goi, nvp_target_endian, 1);
					return e;
				}
				target->endianness = n->value;
			} else {
				if (goi->argc != 0) {
					goto no_params;
				}
			}
			n = Jim_Nvp_value2name_simple(nvp_target_endian, target->endianness);
			if (n->name == NULL) {
				target->endianness = TARGET_LITTLE_ENDIAN;
				n = Jim_Nvp_value2name_simple(nvp_target_endian, target->endianness);
			}
			Jim_SetResultString(goi->interp, n->name, -1);
			/* loop for more */
			break;

		case TCFG_VARIANT:
			if (goi->isconfigure) {
				if (goi->argc < 1) {
					Jim_SetResult_sprintf(goi->interp,
										   "%s ?STRING?",
										   n->name);
					return JIM_ERR;
				}
				if (target->variant) {
					free((void *)(target->variant));
				}
				e = Jim_GetOpt_String(goi, &cp, NULL);
				target->variant = strdup(cp);
			} else {
				if (goi->argc != 0) {
					goto no_params;
				}
			}
			Jim_SetResultString(goi->interp, target->variant,-1);
			/* loop for more */
			break;
		case TCFG_CHAIN_POSITION:
			if (goi->isconfigure) {
				Jim_Obj *o;
				jtag_tap_t *tap;
				target_free_all_working_areas(target);
				e = Jim_GetOpt_Obj(goi, &o);
				if (e != JIM_OK) {
					return e;
				}
				tap = jtag_tap_by_jim_obj(goi->interp, o);
				if (tap == NULL) {
					return JIM_ERR;
				}
				/* make this exactly 1 or 0 */
				target->tap = tap;
			} else {
				if (goi->argc != 0) {
					goto no_params;
				}
			}
			Jim_SetResultString(interp, target->tap->dotted_name, -1);
			/* loop for more e*/
			break;
		}
	} /* while (goi->argc) */


		/* done - we return */
	return JIM_OK;
}

/** this is the 'tcl' handler for the target specific command */
static int tcl_target_func(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	Jim_GetOptInfo goi;
	jim_wide a,b,c;
	int x,y,z;
	uint8_t  target_buf[32];
	Jim_Nvp *n;
	target_t *target;
	struct command_context_s *cmd_ctx;
	int e;

	enum {
		TS_CMD_CONFIGURE,
		TS_CMD_CGET,

		TS_CMD_MWW, TS_CMD_MWH, TS_CMD_MWB,
		TS_CMD_MDW, TS_CMD_MDH, TS_CMD_MDB,
		TS_CMD_MRW, TS_CMD_MRH, TS_CMD_MRB,
		TS_CMD_MEM2ARRAY, TS_CMD_ARRAY2MEM,
		TS_CMD_EXAMINE,
		TS_CMD_POLL,
		TS_CMD_RESET,
		TS_CMD_HALT,
		TS_CMD_WAITSTATE,
		TS_CMD_EVENTLIST,
		TS_CMD_CURSTATE,
		TS_CMD_INVOKE_EVENT,
	};

	static const Jim_Nvp target_options[] = {
		{ .name = "configure", .value = TS_CMD_CONFIGURE },
		{ .name = "cget", .value = TS_CMD_CGET },
		{ .name = "mww", .value = TS_CMD_MWW },
		{ .name = "mwh", .value = TS_CMD_MWH },
		{ .name = "mwb", .value = TS_CMD_MWB },
		{ .name = "mdw", .value = TS_CMD_MDW },
		{ .name = "mdh", .value = TS_CMD_MDH },
		{ .name = "mdb", .value = TS_CMD_MDB },
		{ .name = "mem2array", .value = TS_CMD_MEM2ARRAY },
		{ .name = "array2mem", .value = TS_CMD_ARRAY2MEM },
		{ .name = "eventlist", .value = TS_CMD_EVENTLIST },
		{ .name = "curstate",  .value = TS_CMD_CURSTATE },

		{ .name = "arp_examine", .value = TS_CMD_EXAMINE },
		{ .name = "arp_poll", .value = TS_CMD_POLL },
		{ .name = "arp_reset", .value = TS_CMD_RESET },
		{ .name = "arp_halt", .value = TS_CMD_HALT },
		{ .name = "arp_waitstate", .value = TS_CMD_WAITSTATE },
		{ .name = "invoke-event", .value = TS_CMD_INVOKE_EVENT },

		{ .name = NULL, .value = -1 },
	};

	/* go past the "command" */
	Jim_GetOpt_Setup(&goi, interp, argc-1, argv + 1);

	target = Jim_CmdPrivData(goi.interp);
	cmd_ctx = Jim_GetAssocData(goi.interp, "context");

	/* commands here are in an NVP table */
	e = Jim_GetOpt_Nvp(&goi, target_options, &n);
	if (e != JIM_OK) {
		Jim_GetOpt_NvpUnknown(&goi, target_options, 0);
		return e;
	}
	/* Assume blank result */
	Jim_SetEmptyResult(goi.interp);

	switch (n->value) {
	case TS_CMD_CONFIGURE:
		if (goi.argc < 2) {
			Jim_WrongNumArgs(goi.interp, goi.argc, goi.argv, "missing: -option VALUE ...");
			return JIM_ERR;
		}
		goi.isconfigure = 1;
		return target_configure(&goi, target);
	case TS_CMD_CGET:
		// some things take params
		if (goi.argc < 1) {
			Jim_WrongNumArgs(goi.interp, 0, goi.argv, "missing: ?-option?");
			return JIM_ERR;
		}
		goi.isconfigure = 0;
		return target_configure(&goi, target);
		break;
	case TS_CMD_MWW:
	case TS_CMD_MWH:
	case TS_CMD_MWB:
		/* argv[0] = cmd
		 * argv[1] = address
		 * argv[2] = data
		 * argv[3] = optional count.
		 */

		if ((goi.argc == 2) || (goi.argc == 3)) {
			/* all is well */
		} else {
		mwx_error:
			Jim_SetResult_sprintf(goi.interp, "expected: %s ADDR DATA [COUNT]", n->name);
			return JIM_ERR;
		}

		e = Jim_GetOpt_Wide(&goi, &a);
		if (e != JIM_OK) {
			goto mwx_error;
		}

		e = Jim_GetOpt_Wide(&goi, &b);
		if (e != JIM_OK) {
			goto mwx_error;
		}
		if (goi.argc == 3) {
			e = Jim_GetOpt_Wide(&goi, &c);
			if (e != JIM_OK) {
				goto mwx_error;
			}
		} else {
			c = 1;
		}

		switch (n->value) {
		case TS_CMD_MWW:
			target_buffer_set_u32(target, target_buf, b);
			b = 4;
			break;
		case TS_CMD_MWH:
			target_buffer_set_u16(target, target_buf, b);
			b = 2;
			break;
		case TS_CMD_MWB:
			target_buffer_set_u8(target, target_buf, b);
			b = 1;
			break;
		}
		for (x = 0 ; x < c ; x++) {
			e = target_write_memory(target, a, b, 1, target_buf);
			if (e != ERROR_OK) {
				Jim_SetResult_sprintf(interp, "Error writing @ 0x%08x: %d\n", (int)(a), e);
				return JIM_ERR;
			}
			/* b = width */
			a = a + b;
		}
		return JIM_OK;
		break;

		/* display */
	case TS_CMD_MDW:
	case TS_CMD_MDH:
	case TS_CMD_MDB:
		/* argv[0] = command
		 * argv[1] = address
		 * argv[2] = optional count
		 */
		if ((goi.argc == 2) || (goi.argc == 3)) {
			Jim_SetResult_sprintf(goi.interp, "expected: %s ADDR [COUNT]", n->name);
			return JIM_ERR;
		}
		e = Jim_GetOpt_Wide(&goi, &a);
		if (e != JIM_OK) {
			return JIM_ERR;
		}
		if (goi.argc) {
			e = Jim_GetOpt_Wide(&goi, &c);
			if (e != JIM_OK) {
				return JIM_ERR;
			}
		} else {
			c = 1;
		}
		b = 1; /* shut up gcc */
		switch (n->value) {
		case TS_CMD_MDW:
			b =  4;
			break;
		case TS_CMD_MDH:
			b = 2;
			break;
		case TS_CMD_MDB:
			b = 1;
			break;
		}

		/* convert to "bytes" */
		c = c * b;
		/* count is now in 'BYTES' */
		while (c > 0) {
			y = c;
			if (y > 16) {
				y = 16;
			}
			e = target_read_memory(target, a, b, y / b, target_buf);
			if (e != ERROR_OK) {
				Jim_SetResult_sprintf(interp, "error reading target @ 0x%08lx", (int)(a));
				return JIM_ERR;
			}

			Jim_fprintf(interp, interp->cookie_stdout, "0x%08x ", (int)(a));
			switch (b) {
			case 4:
				for (x = 0 ; (x < 16) && (x < y) ; x += 4) {
					z = target_buffer_get_u32(target, &(target_buf[ x * 4 ]));
					Jim_fprintf(interp, interp->cookie_stdout, "%08x ", (int)(z));
				}
				for (; (x < 16) ; x += 4) {
					Jim_fprintf(interp, interp->cookie_stdout, "         ");
				}
				break;
			case 2:
				for (x = 0 ; (x < 16) && (x < y) ; x += 2) {
					z = target_buffer_get_u16(target, &(target_buf[ x * 2 ]));
					Jim_fprintf(interp, interp->cookie_stdout, "%04x ", (int)(z));
				}
				for (; (x < 16) ; x += 2) {
					Jim_fprintf(interp, interp->cookie_stdout, "     ");
				}
				break;
			case 1:
			default:
				for (x = 0 ; (x < 16) && (x < y) ; x += 1) {
					z = target_buffer_get_u8(target, &(target_buf[ x * 4 ]));
					Jim_fprintf(interp, interp->cookie_stdout, "%02x ", (int)(z));
				}
				for (; (x < 16) ; x += 1) {
					Jim_fprintf(interp, interp->cookie_stdout, "   ");
				}
				break;
			}
			/* ascii-ify the bytes */
			for (x = 0 ; x < y ; x++) {
				if ((target_buf[x] >= 0x20) &&
					(target_buf[x] <= 0x7e)) {
					/* good */
				} else {
					/* smack it */
					target_buf[x] = '.';
				}
			}
			/* space pad  */
			while (x < 16) {
				target_buf[x] = ' ';
				x++;
			}
			/* terminate */
			target_buf[16] = 0;
			/* print - with a newline */
			Jim_fprintf(interp, interp->cookie_stdout, "%s\n", target_buf);
			/* NEXT... */
			c -= 16;
			a += 16;
		}
		return JIM_OK;
	case TS_CMD_MEM2ARRAY:
		return target_mem2array(goi.interp, target, goi.argc, goi.argv);
		break;
	case TS_CMD_ARRAY2MEM:
		return target_array2mem(goi.interp, target, goi.argc, goi.argv);
		break;
	case TS_CMD_EXAMINE:
		if (goi.argc) {
			Jim_WrongNumArgs(goi.interp, 2, argv, "[no parameters]");
			return JIM_ERR;
		}
		if (!target->tap->enabled)
			goto err_tap_disabled;
		e = target->type->examine(target);
		if (e != ERROR_OK) {
			Jim_SetResult_sprintf(interp, "examine-fails: %d", e);
			return JIM_ERR;
		}
		return JIM_OK;
	case TS_CMD_POLL:
		if (goi.argc) {
			Jim_WrongNumArgs(goi.interp, 2, argv, "[no parameters]");
			return JIM_ERR;
		}
		if (!target->tap->enabled)
			goto err_tap_disabled;
		if (!(target_was_examined(target))) {
			e = ERROR_TARGET_NOT_EXAMINED;
		} else {
			e = target->type->poll(target);
		}
		if (e != ERROR_OK) {
			Jim_SetResult_sprintf(interp, "poll-fails: %d", e);
			return JIM_ERR;
		} else {
			return JIM_OK;
		}
		break;
	case TS_CMD_RESET:
		if (goi.argc != 2) {
			Jim_WrongNumArgs(interp, 2, argv, "t | f|assert | deassert BOOL");
			return JIM_ERR;
		}
		e = Jim_GetOpt_Nvp(&goi, nvp_assert, &n);
		if (e != JIM_OK) {
			Jim_GetOpt_NvpUnknown(&goi, nvp_assert, 1);
			return e;
		}
		/* the halt or not param */
		e = Jim_GetOpt_Wide(&goi, &a);
		if (e != JIM_OK) {
			return e;
		}
		if (!target->tap->enabled)
			goto err_tap_disabled;
		/* determine if we should halt or not. */
		target->reset_halt = !!a;
		/* When this happens - all workareas are invalid. */
		target_free_all_working_areas_restore(target, 0);

		/* do the assert */
		if (n->value == NVP_ASSERT) {
			target->type->assert_reset(target);
		} else {
			target->type->deassert_reset(target);
		}
		return JIM_OK;
	case TS_CMD_HALT:
		if (goi.argc) {
			Jim_WrongNumArgs(goi.interp, 0, argv, "halt [no parameters]");
			return JIM_ERR;
		}
		if (!target->tap->enabled)
			goto err_tap_disabled;
		target->type->halt(target);
		return JIM_OK;
	case TS_CMD_WAITSTATE:
		/* params:  <name>  statename timeoutmsecs */
		if (goi.argc != 2) {
			Jim_SetResult_sprintf(goi.interp, "%s STATENAME TIMEOUTMSECS", n->name);
			return JIM_ERR;
		}
		e = Jim_GetOpt_Nvp(&goi, nvp_target_state, &n);
		if (e != JIM_OK) {
			Jim_GetOpt_NvpUnknown(&goi, nvp_target_state,1);
			return e;
		}
		e = Jim_GetOpt_Wide(&goi, &a);
		if (e != JIM_OK) {
			return e;
		}
		if (!target->tap->enabled)
			goto err_tap_disabled;
		e = target_wait_state(target, n->value, a);
		if (e != ERROR_OK) {
			Jim_SetResult_sprintf(goi.interp,
								   "target: %s wait %s fails (%d) %s",
								   target->cmd_name,
								   n->name,
								   e, target_strerror_safe(e));
			return JIM_ERR;
		} else {
			return JIM_OK;
		}
	case TS_CMD_EVENTLIST:
		/* List for human, Events defined for this target.
		 * scripts/programs should use 'name cget -event NAME'
		 */
		{
			target_event_action_t *teap;
			teap = target->event_action;
			command_print(cmd_ctx, "Event actions for target (%d) %s\n",
						   target->target_number,
						   target->cmd_name);
			command_print(cmd_ctx, "%-25s | Body", "Event");
			command_print(cmd_ctx, "------------------------- | ----------------------------------------");
			while (teap) {
				command_print(cmd_ctx,
							   "%-25s | %s",
							   Jim_Nvp_value2name_simple(nvp_target_event, teap->event)->name,
							   Jim_GetString(teap->body, NULL));
				teap = teap->next;
			}
			command_print(cmd_ctx, "***END***");
			return JIM_OK;
		}
	case TS_CMD_CURSTATE:
		if (goi.argc != 0) {
			Jim_WrongNumArgs(goi.interp, 0, argv, "[no parameters]");
			return JIM_ERR;
		}
		Jim_SetResultString(goi.interp,
							target_state_name( target ),
							-1);
		return JIM_OK;
	case TS_CMD_INVOKE_EVENT:
		if (goi.argc != 1) {
			Jim_SetResult_sprintf(goi.interp, "%s ?EVENTNAME?",n->name);
			return JIM_ERR;
		}
		e = Jim_GetOpt_Nvp(&goi, nvp_target_event, &n);
		if (e != JIM_OK) {
			Jim_GetOpt_NvpUnknown(&goi, nvp_target_event, 1);
			return e;
		}
		target_handle_event(target, n->value);
		return JIM_OK;
	}
	return JIM_ERR;

err_tap_disabled:
	Jim_SetResult_sprintf(interp, "[TAP is disabled]");
	return JIM_ERR;
}

static int target_create(Jim_GetOptInfo *goi)
{
	Jim_Obj *new_cmd;
	Jim_Cmd *cmd;
	const char *cp;
	char *cp2;
	int e;
	int x;
	target_t *target;
	struct command_context_s *cmd_ctx;

	cmd_ctx = Jim_GetAssocData(goi->interp, "context");
	if (goi->argc < 3) {
		Jim_WrongNumArgs(goi->interp, 1, goi->argv, "?name? ?type? ..options...");
		return JIM_ERR;
	}

	/* COMMAND */
	Jim_GetOpt_Obj(goi, &new_cmd);
	/* does this command exist? */
	cmd = Jim_GetCommand(goi->interp, new_cmd, JIM_ERRMSG);
	if (cmd) {
		cp = Jim_GetString(new_cmd, NULL);
		Jim_SetResult_sprintf(goi->interp, "Command/target: %s Exists", cp);
		return JIM_ERR;
	}

	/* TYPE */
	e = Jim_GetOpt_String(goi, &cp2, NULL);
	cp = cp2;
	/* now does target type exist */
	for (x = 0 ; target_types[x] ; x++) {
		if (0 == strcmp(cp, target_types[x]->name)) {
			/* found */
			break;
		}
	}
	if (target_types[x] == NULL) {
		Jim_SetResult_sprintf(goi->interp, "Unknown target type %s, try one of ", cp);
		for (x = 0 ; target_types[x] ; x++) {
			if (target_types[x + 1]) {
				Jim_AppendStrings(goi->interp,
								   Jim_GetResult(goi->interp),
								   target_types[x]->name,
								   ", ", NULL);
			} else {
				Jim_AppendStrings(goi->interp,
								   Jim_GetResult(goi->interp),
								   " or ",
								   target_types[x]->name,NULL);
			}
		}
		return JIM_ERR;
	}

	/* Create it */
	target = calloc(1,sizeof(target_t));
	/* set target number */
	target->target_number = new_target_number();

	/* allocate memory for each unique target type */
	target->type = (target_type_t*)calloc(1,sizeof(target_type_t));

	memcpy(target->type, target_types[x], sizeof(target_type_t));

	/* will be set by "-endian" */
	target->endianness = TARGET_ENDIAN_UNKNOWN;

	target->working_area        = 0x0;
	target->working_area_size   = 0x0;
	target->working_areas       = NULL;
	target->backup_working_area = 0;

	target->state               = TARGET_UNKNOWN;
	target->debug_reason        = DBG_REASON_UNDEFINED;
	target->reg_cache           = NULL;
	target->breakpoints         = NULL;
	target->watchpoints         = NULL;
	target->next                = NULL;
	target->arch_info           = NULL;

	target->display             = 1;

	/* initialize trace information */
	target->trace_info = malloc(sizeof(trace_t));
	target->trace_info->num_trace_points         = 0;
	target->trace_info->trace_points_size        = 0;
	target->trace_info->trace_points             = NULL;
	target->trace_info->trace_history_size       = 0;
	target->trace_info->trace_history            = NULL;
	target->trace_info->trace_history_pos        = 0;
	target->trace_info->trace_history_overflowed = 0;

	target->dbgmsg          = NULL;
	target->dbg_msg_enabled = 0;

	target->endianness = TARGET_ENDIAN_UNKNOWN;

	/* Do the rest as "configure" options */
	goi->isconfigure = 1;
	e = target_configure(goi, target);

	if (target->tap == NULL)
	{
		Jim_SetResultString(interp, "-chain-position required when creating target", -1);
		e = JIM_ERR;
	}

	if (e != JIM_OK) {
		free(target->type);
		free(target);
		return e;
	}

	if (target->endianness == TARGET_ENDIAN_UNKNOWN) {
		/* default endian to little if not specified */
		target->endianness = TARGET_LITTLE_ENDIAN;
	}

	/* incase variant is not set */
	if (!target->variant)
		target->variant = strdup("");

	/* create the target specific commands */
	if (target->type->register_commands) {
		(*(target->type->register_commands))(cmd_ctx);
	}
	if (target->type->target_create) {
		(*(target->type->target_create))(target, goi->interp);
	}

	/* append to end of list */
	{
		target_t **tpp;
		tpp = &(all_targets);
		while (*tpp) {
			tpp = &((*tpp)->next);
		}
		*tpp = target;
	}

	cp = Jim_GetString(new_cmd, NULL);
	target->cmd_name = strdup(cp);

	/* now - create the new target name command */
	e = Jim_CreateCommand(goi->interp,
						   /* name */
						   cp,
						   tcl_target_func, /* C function */
						   target, /* private data */
						   NULL); /* no del proc */

	return e;
}

static int jim_target(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	int x,r,e;
	jim_wide w;
	struct command_context_s *cmd_ctx;
	target_t *target;
	Jim_GetOptInfo goi;
	enum tcmd {
		/* TG = target generic */
		TG_CMD_CREATE,
		TG_CMD_TYPES,
		TG_CMD_NAMES,
		TG_CMD_CURRENT,
		TG_CMD_NUMBER,
		TG_CMD_COUNT,
	};
	const char *target_cmds[] = {
		"create", "types", "names", "current", "number",
		"count",
		NULL /* terminate */
	};

	LOG_DEBUG("Target command params:");
	LOG_DEBUG("%s", Jim_Debug_ArgvString(interp, argc, argv));

	cmd_ctx = Jim_GetAssocData(interp, "context");

	Jim_GetOpt_Setup(&goi, interp, argc-1, argv + 1);

	if (goi.argc == 0) {
		Jim_WrongNumArgs(interp, 1, argv, "missing: command ...");
		return JIM_ERR;
	}

	/* Jim_GetOpt_Debug(&goi); */
	r = Jim_GetOpt_Enum(&goi, target_cmds, &x);
	if (r != JIM_OK) {
		return r;
	}

	switch (x) {
	default:
		Jim_Panic(goi.interp,"Why am I here?");
		return JIM_ERR;
	case TG_CMD_CURRENT:
		if (goi.argc != 0) {
			Jim_WrongNumArgs(goi.interp, 1, goi.argv, "Too many parameters");
			return JIM_ERR;
		}
		Jim_SetResultString(goi.interp, get_current_target(cmd_ctx)->cmd_name, -1);
		return JIM_OK;
	case TG_CMD_TYPES:
		if (goi.argc != 0) {
			Jim_WrongNumArgs(goi.interp, 1, goi.argv, "Too many parameters");
			return JIM_ERR;
		}
		Jim_SetResult(goi.interp, Jim_NewListObj(goi.interp, NULL, 0));
		for (x = 0 ; target_types[x] ; x++) {
			Jim_ListAppendElement(goi.interp,
								   Jim_GetResult(goi.interp),
								   Jim_NewStringObj(goi.interp, target_types[x]->name, -1));
		}
		return JIM_OK;
	case TG_CMD_NAMES:
		if (goi.argc != 0) {
			Jim_WrongNumArgs(goi.interp, 1, goi.argv, "Too many parameters");
			return JIM_ERR;
		}
		Jim_SetResult(goi.interp, Jim_NewListObj(goi.interp, NULL, 0));
		target = all_targets;
		while (target) {
			Jim_ListAppendElement(goi.interp,
								   Jim_GetResult(goi.interp),
								   Jim_NewStringObj(goi.interp, target->cmd_name, -1));
			target = target->next;
		}
		return JIM_OK;
	case TG_CMD_CREATE:
		if (goi.argc < 3) {
			Jim_WrongNumArgs(goi.interp, goi.argc, goi.argv, "?name  ... config options ...");
			return JIM_ERR;
		}
		return target_create(&goi);
		break;
	case TG_CMD_NUMBER:
		if (goi.argc != 1) {
			Jim_SetResult_sprintf(goi.interp, "expected: target number ?NUMBER?");
			return JIM_ERR;
		}
		e = Jim_GetOpt_Wide(&goi, &w);
		if (e != JIM_OK) {
			return JIM_ERR;
		}
		{
			target_t *t;
			t = get_target_by_num(w);
			if (t == NULL) {
				Jim_SetResult_sprintf(goi.interp,"Target: number %d does not exist", (int)(w));
				return JIM_ERR;
			}
			Jim_SetResultString(goi.interp, t->cmd_name, -1);
			return JIM_OK;
		}
	case TG_CMD_COUNT:
		if (goi.argc != 0) {
			Jim_WrongNumArgs(goi.interp, 0, goi.argv, "<no parameters>");
			return JIM_ERR;
		}
		Jim_SetResult(goi.interp,
					   Jim_NewIntObj(goi.interp, max_target_number()));
		return JIM_OK;
	}

	return JIM_ERR;
}


struct FastLoad
{
	uint32_t address;
	uint8_t *data;
	int length;

};

static int fastload_num;
static struct FastLoad *fastload;

static void free_fastload(void)
{
	if (fastload != NULL)
	{
		int i;
		for (i = 0; i < fastload_num; i++)
		{
			if (fastload[i].data)
				free(fastload[i].data);
		}
		free(fastload);
		fastload = NULL;
	}
}




static int handle_fast_load_image_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	uint8_t *buffer;
	uint32_t buf_cnt;
	uint32_t image_size;
	uint32_t min_address = 0;
	uint32_t max_address = 0xffffffff;
	int i;

	image_t image;

	duration_t duration;
	char *duration_text;

	int retval = parse_load_image_command_args(args, argc,
			&image, &min_address, &max_address);
	if (ERROR_OK != retval)
		return retval;

	duration_start_measure(&duration);

	if (image_open(&image, args[0], (argc >= 3) ? args[2] : NULL) != ERROR_OK)
	{
		return ERROR_OK;
	}

	image_size = 0x0;
	retval = ERROR_OK;
	fastload_num = image.num_sections;
	fastload = (struct FastLoad *)malloc(sizeof(struct FastLoad)*image.num_sections);
	if (fastload == NULL)
	{
		image_close(&image);
		return ERROR_FAIL;
	}
	memset(fastload, 0, sizeof(struct FastLoad)*image.num_sections);
	for (i = 0; i < image.num_sections; i++)
	{
		buffer = malloc(image.sections[i].size);
		if (buffer == NULL)
		{
			command_print(cmd_ctx, "error allocating buffer for section (%d bytes)",
						  (int)(image.sections[i].size));
			break;
		}

		if ((retval = image_read_section(&image, i, 0x0, image.sections[i].size, buffer, &buf_cnt)) != ERROR_OK)
		{
			free(buffer);
			break;
		}

		uint32_t offset = 0;
		uint32_t length = buf_cnt;


		/* DANGER!!! beware of unsigned comparision here!!! */

		if ((image.sections[i].base_address + buf_cnt >= min_address)&&
				(image.sections[i].base_address < max_address))
		{
			if (image.sections[i].base_address < min_address)
			{
				/* clip addresses below */
				offset += min_address-image.sections[i].base_address;
				length -= offset;
			}

			if (image.sections[i].base_address + buf_cnt > max_address)
			{
				length -= (image.sections[i].base_address + buf_cnt)-max_address;
			}

			fastload[i].address = image.sections[i].base_address + offset;
			fastload[i].data = malloc(length);
			if (fastload[i].data == NULL)
			{
				free(buffer);
				break;
			}
			memcpy(fastload[i].data, buffer + offset, length);
			fastload[i].length = length;

			image_size += length;
			command_print(cmd_ctx, "%u byte written at address 0x%8.8x",
						  (unsigned int)length,
						  ((unsigned int)(image.sections[i].base_address + offset)));
		}

		free(buffer);
	}

	duration_stop_measure(&duration, &duration_text);
	if (retval == ERROR_OK)
	{
		command_print(cmd_ctx, "Loaded %u bytes in %s", (unsigned int)image_size, duration_text);
		command_print(cmd_ctx, "NB!!! image has not been loaded to target, issue a subsequent 'fast_load' to do so.");
	}
	free(duration_text);

	image_close(&image);

	if (retval != ERROR_OK)
	{
		free_fastload();
	}

	return retval;
}

static int handle_fast_load_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc > 0)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (fastload == NULL)
	{
		LOG_ERROR("No image in memory");
		return ERROR_FAIL;
	}
	int i;
	int ms = timeval_ms();
	int size = 0;
	int retval = ERROR_OK;
	for (i = 0; i < fastload_num;i++)
	{
		target_t *target = get_current_target(cmd_ctx);
		command_print(cmd_ctx, "Write to 0x%08x, length 0x%08x",
					  (unsigned int)(fastload[i].address),
					  (unsigned int)(fastload[i].length));
		if (retval == ERROR_OK)
		{
			retval = target_write_buffer(target, fastload[i].address, fastload[i].length, fastload[i].data);
		}
		size += fastload[i].length;
	}
	int after = timeval_ms();
	command_print(cmd_ctx, "Loaded image %f kBytes/s", (float)(size/1024.0)/((float)(after-ms)/1000.0));
	return retval;
}


/*
 * Local Variables:
 * c-basic-offset: 4
 * tab-width: 4
 * End:
 */
