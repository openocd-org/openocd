// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   ESP Xtensa SMP target API for OpenOCD                                 *
 *   Copyright (C) 2020 Espressif Systems Ltd. Co                          *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "assert.h"
#include <target/target.h>
#include <target/target_type.h>
#include <target/smp.h>
#include <target/semihosting_common.h>
#include "esp_xtensa_smp.h"
#include "esp_xtensa_semihosting.h"

/*
Multiprocessor stuff common:

The ESP Xtensa chip can have several cores in it, which can run in SMP-mode if an
SMP-capable OS is running. The hardware has a few features which makes
SMP debugging much easier.

First of all, there's something called a 'break network', consisting of a
BreakIn input  and a BreakOut output on each CPU. The idea is that as soon
as a CPU goes into debug mode for whatever reason, it'll signal that using
its DebugOut pin. This signal is connected to the other CPU's DebugIn
input, causing this CPU also to go into debugging mode. To resume execution
when using only this break network, we will need to manually resume both
CPUs.

An alternative to this is the XOCDMode output and the RunStall (or DebugStall)
input. When these are cross-connected, a CPU that goes into debug mode will
halt execution entirely on the other CPU. Execution on the other CPU can be
resumed by either the first CPU going out of debug mode, or the second CPU
going into debug mode: the stall is temporarily lifted as long as the stalled
CPU is in debug mode.

A third, separate, signal is CrossTrigger. This is connected in the same way
as the breakIn/breakOut network, but is for the TRAX (trace memory) feature;
it does not affect OCD in any way.
*/

/*
Multiprocessor stuff:

The ESP Xtensa chip has several Xtensa cores inside, but represent themself to the OCD
as one chip that works in multithreading mode under FreeRTOS OS.
The core that initiate the stop condition will be defined as an active cpu.
When one core stops, then other core will be stopped automatically by smpbreak.
The core that initiates stop condition will be defined as an active core, and
registers of this core will be transferred.
*/

#define ESP_XTENSA_SMP_EXAMINE_OTHER_CORES      5

static int esp_xtensa_smp_update_halt_gdb(struct target *target, bool *need_resume);

static inline struct esp_xtensa_smp_common *target_to_esp_xtensa_smp(struct target *target)
{
	return container_of(target->arch_info, struct esp_xtensa_smp_common, esp_xtensa);
}

int esp_xtensa_smp_assert_reset(struct target *target)
{
	return ERROR_OK;
}

int esp_xtensa_smp_deassert_reset(struct target *target)
{
	LOG_TARGET_DEBUG(target, "begin");

	int ret = xtensa_deassert_reset(target);
	if (ret != ERROR_OK)
		return ret;
	/* in SMP mode when chip was running single-core app the other core can be left un-examined,
	   because examination is done before SOC reset. But after SOC reset it is functional and should be handled.
	   So try to examine un-examined core just after SOC reset */
	if (target->smp && !target_was_examined(target))
		ret = xtensa_examine(target);
	return ret;
}

int esp_xtensa_smp_soft_reset_halt(struct target *target)
{
	int res;
	struct target_list *head;
	struct esp_xtensa_smp_common *esp_xtensa_smp = target_to_esp_xtensa_smp(target);

	LOG_TARGET_DEBUG(target, "begin");
	/* in SMP mode we need to ensure that at first we reset SOC on PRO-CPU
	   and then call xtensa_assert_reset() for all cores */
	if (target->smp && target->coreid != 0)
		return ERROR_OK;
	/* Reset the SoC first */
	if (esp_xtensa_smp->chip_ops->reset) {
		res = esp_xtensa_smp->chip_ops->reset(target);
		if (res != ERROR_OK)
			return res;
	}
	if (!target->smp)
		return xtensa_assert_reset(target);

	foreach_smp_target(head, target->smp_targets) {
		res = xtensa_assert_reset(head->target);
		if (res != ERROR_OK)
			return res;
	}
	return ERROR_OK;
}

static struct target *get_halted_esp_xtensa_smp(struct target *target, int32_t coreid)
{
	struct target_list *head;
	struct target *curr;

	foreach_smp_target(head, target->smp_targets) {
		curr = head->target;
		if ((curr->coreid == coreid) && (curr->state == TARGET_HALTED))
			return curr;
	}

	return target;
}

int esp_xtensa_smp_poll(struct target *target)
{
	enum target_state old_state = target->state;
	struct esp_xtensa_smp_common *esp_xtensa_smp = target_to_esp_xtensa_smp(target);
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);
	struct target_list *head;
	struct target *curr;
	bool other_core_resume_req = false;

	if (target->state == TARGET_HALTED && target->smp && target->gdb_service && !target->gdb_service->target) {
		target->gdb_service->target = get_halted_esp_xtensa_smp(target, target->gdb_service->core[1]);
		LOG_INFO("Switch GDB target to '%s'", target_name(target->gdb_service->target));
		if (esp_xtensa_smp->chip_ops->on_halt)
			esp_xtensa_smp->chip_ops->on_halt(target);
		target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		return ERROR_OK;
	}

	int ret = esp_xtensa_poll(target);
	if (ret != ERROR_OK)
		return ret;

	if (target->smp) {
		if (target->state == TARGET_RESET) {
			esp_xtensa_smp->examine_other_cores = ESP_XTENSA_SMP_EXAMINE_OTHER_CORES;
		} else if (esp_xtensa_smp->examine_other_cores > 0 &&
			(target->state == TARGET_RUNNING || target->state == TARGET_HALTED)) {
			LOG_TARGET_DEBUG(target, "Check for unexamined cores after reset");
			bool all_examined = true;
			foreach_smp_target(head, target->smp_targets) {
				curr = head->target;
				if (curr == target)
					continue;
				if (!target_was_examined(curr)) {
					if (target_examine_one(curr) != ERROR_OK) {
						LOG_DEBUG("Failed to examine!");
						all_examined = false;
					}
				}
			}
			if (all_examined)
				esp_xtensa_smp->examine_other_cores = 0;
			else
				esp_xtensa_smp->examine_other_cores--;
		}
	}

	if (old_state != TARGET_HALTED && target->state == TARGET_HALTED) {
		if (target->smp) {
			ret = esp_xtensa_smp_update_halt_gdb(target, &other_core_resume_req);
			if (ret != ERROR_OK)
				return ret;
		}
		/* Call any event callbacks that are applicable */
		if (old_state == TARGET_DEBUG_RUNNING) {
			target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
		} else {
			if (esp_xtensa_semihosting(target, &ret) == SEMIHOSTING_HANDLED) {
				if (ret == ERROR_OK && esp_xtensa->semihost.need_resume &&
					!esp_xtensa_smp->other_core_does_resume) {
					esp_xtensa->semihost.need_resume = false;
					/* Resume xtensa_resume will handle BREAK instruction. */
					ret = target_resume(target, 1, 0, 1, 0);
					if (ret != ERROR_OK) {
						LOG_ERROR("Failed to resume target");
						return ret;
					}
				}
				return ret;
			}
			/* check whether any core polled by esp_xtensa_smp_update_halt_gdb() requested resume */
			if (target->smp && other_core_resume_req) {
				/* Resume xtensa_resume will handle BREAK instruction. */
				ret = target_resume(target, 1, 0, 1, 0);
				if (ret != ERROR_OK) {
					LOG_ERROR("Failed to resume target");
					return ret;
				}
				return ERROR_OK;
			}
			if (esp_xtensa_smp->chip_ops->on_halt)
				esp_xtensa_smp->chip_ops->on_halt(target);
			target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		}
	}

	return ERROR_OK;
}

static int esp_xtensa_smp_update_halt_gdb(struct target *target, bool *need_resume)
{
	struct esp_xtensa_smp_common *esp_xtensa_smp;
	struct target *gdb_target = NULL;
	struct target_list *head;
	struct target *curr;
	int ret = ERROR_OK;

	*need_resume = false;

	if (target->gdb_service && target->gdb_service->target)
		LOG_DEBUG("GDB target '%s'", target_name(target->gdb_service->target));

	if (target->gdb_service && target->gdb_service->core[0] == -1) {
		target->gdb_service->target = target;
		target->gdb_service->core[0] = target->coreid;
		LOG_INFO("Set GDB target to '%s'", target_name(target));
	}

	if (target->gdb_service)
		gdb_target = target->gdb_service->target;

	/* due to smpbreak config other cores can also go to HALTED state */
	foreach_smp_target(head, target->smp_targets) {
		curr = head->target;
		LOG_DEBUG("Check target '%s'", target_name(curr));
		/* skip calling context */
		if (curr == target)
			continue;
		if (!target_was_examined(curr)) {
			curr->state = TARGET_HALTED;
			continue;
		}
		/* skip targets that were already halted */
		if (curr->state == TARGET_HALTED)
			continue;
		/* Skip gdb_target; it alerts GDB so has to be polled as last one */
		if (curr == gdb_target)
			continue;
		LOG_DEBUG("Poll target '%s'", target_name(curr));

		esp_xtensa_smp = target_to_esp_xtensa_smp(curr);
		/* avoid auto-resume after syscall, it will be done later */
		esp_xtensa_smp->other_core_does_resume = true;
		/* avoid recursion in esp_xtensa_smp_poll() */
		curr->smp = 0;
		if (esp_xtensa_smp->chip_ops->poll)
			ret = esp_xtensa_smp->chip_ops->poll(curr);
		else
			ret = esp_xtensa_smp_poll(curr);
		curr->smp = 1;
		if (ret != ERROR_OK)
			return ret;
		esp_xtensa_smp->other_core_does_resume = false;
		struct esp_xtensa_common *curr_esp_xtensa = target_to_esp_xtensa(curr);
		if (curr_esp_xtensa->semihost.need_resume) {
			curr_esp_xtensa->semihost.need_resume = false;
			*need_resume = true;
		}
	}

	/* after all targets were updated, poll the gdb serving target */
	if (gdb_target && gdb_target != target) {
		esp_xtensa_smp = target_to_esp_xtensa_smp(gdb_target);
		if (esp_xtensa_smp->chip_ops->poll)
			ret = esp_xtensa_smp->chip_ops->poll(gdb_target);
		else
			ret = esp_xtensa_smp_poll(gdb_target);
	}

	LOG_DEBUG("exit");

	return ret;
}

static inline int esp_xtensa_smp_smpbreak_disable(struct target *target, uint32_t *smp_break)
{
	int res = xtensa_smpbreak_get(target, smp_break);
	if (res != ERROR_OK)
		return res;
	return xtensa_smpbreak_set(target, 0);
}

static inline int esp_xtensa_smp_smpbreak_restore(struct target *target, uint32_t smp_break)
{
	return xtensa_smpbreak_set(target, smp_break);
}

static int esp_xtensa_smp_resume_cores(struct target *target,
	int handle_breakpoints,
	int debug_execution)
{
	struct target_list *head;
	struct target *curr;

	LOG_TARGET_DEBUG(target, "begin");

	foreach_smp_target(head, target->smp_targets) {
		curr = head->target;
		/* in single-core mode disabled core cannot be examined, but need to be resumed too*/
		if ((curr != target) && (curr->state != TARGET_RUNNING) && target_was_examined(curr)) {
			/*  resume current address, not in SMP mode */
			curr->smp = 0;
			int res = esp_xtensa_smp_resume(curr, 1, 0, handle_breakpoints, debug_execution);
			curr->smp = 1;
			if (res != ERROR_OK)
				return res;
		}
	}
	return ERROR_OK;
}

int esp_xtensa_smp_resume(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints,
	int debug_execution)
{
	int res;
	uint32_t smp_break;

	xtensa_smpbreak_get(target, &smp_break);
	LOG_TARGET_DEBUG(target, "smp_break=0x%" PRIx32, smp_break);

	/* dummy resume for smp toggle in order to reduce gdb impact  */
	if ((target->smp) && (target->gdb_service) && (target->gdb_service->core[1] != -1)) {
		/* simulate a start and halt of target */
		target->gdb_service->target = NULL;
		target->gdb_service->core[0] = target->gdb_service->core[1];
		/* fake resume at next poll we play the  target core[1], see poll*/
		LOG_TARGET_DEBUG(target, "Fake resume");
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		return ERROR_OK;
	}

	/* xtensa_prepare_resume() can step over breakpoint/watchpoint and generate signals on BreakInOut circuit for
	 * other cores. So disconnect this core from BreakInOut circuit and do xtensa_prepare_resume(). */
	res = esp_xtensa_smp_smpbreak_disable(target, &smp_break);
	if (res != ERROR_OK)
		return res;
	res = xtensa_prepare_resume(target, current, address, handle_breakpoints, debug_execution);
	/* restore configured BreakInOut signals config */
	int ret = esp_xtensa_smp_smpbreak_restore(target, smp_break);
	if (ret != ERROR_OK)
		return ret;
	if (res != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to prepare for resume!");
		return res;
	}

	if (target->smp) {
		if (target->gdb_service)
			target->gdb_service->core[0] = -1;
		res = esp_xtensa_smp_resume_cores(target, handle_breakpoints, debug_execution);
		if (res != ERROR_OK)
			return res;
	}

	res = xtensa_do_resume(target);
	if (res != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to resume!");
		return res;
	}

	target->debug_reason = DBG_REASON_NOTHALTED;
	if (!debug_execution)
		target->state = TARGET_RUNNING;
	else
		target->state = TARGET_DEBUG_RUNNING;

	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
	return ERROR_OK;
}

int esp_xtensa_smp_step(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints)
{
	int res;
	uint32_t smp_break = 0;
	struct esp_xtensa_smp_common *esp_xtensa_smp = target_to_esp_xtensa_smp(target);

	if (target->smp) {
		res = esp_xtensa_smp_smpbreak_disable(target, &smp_break);
		if (res != ERROR_OK)
			return res;
	}
	res = xtensa_step(target, current, address, handle_breakpoints);

	if (res == ERROR_OK) {
		if (esp_xtensa_smp->chip_ops->on_halt)
			esp_xtensa_smp->chip_ops->on_halt(target);
		target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	}

	if (target->smp) {
		int ret = esp_xtensa_smp_smpbreak_restore(target, smp_break);
		if (ret != ERROR_OK)
			return ret;
	}

	return res;
}

int esp_xtensa_smp_watchpoint_add(struct target *target, struct watchpoint *watchpoint)
{
	int res = xtensa_watchpoint_add(target, watchpoint);
	if (res != ERROR_OK)
		return res;

	if (!target->smp)
		return ERROR_OK;

	struct target_list *head;
	foreach_smp_target(head, target->smp_targets) {
		struct target *curr = head->target;
		if (curr == target || !target_was_examined(curr))
			continue;
		/* Need to use high level API here because every target for core contains list of watchpoints.
		 * GDB works with active core only, so we need to duplicate every watchpoint on other cores,
		 * otherwise watchpoint_free() on active core can fail if WP has been initially added on another core. */
		curr->smp = 0;
		res = watchpoint_add(curr, watchpoint->address, watchpoint->length,
			watchpoint->rw, watchpoint->value, watchpoint->mask);
		curr->smp = 1;
		if (res != ERROR_OK)
			return res;
	}
	return ERROR_OK;
}

int esp_xtensa_smp_watchpoint_remove(struct target *target, struct watchpoint *watchpoint)
{
	int res = xtensa_watchpoint_remove(target, watchpoint);
	if (res != ERROR_OK)
		return res;

	if (!target->smp)
		return ERROR_OK;

	struct target_list *head;
	foreach_smp_target(head, target->smp_targets) {
		struct target *curr = head->target;
		if (curr == target)
			continue;
		/* see big comment in esp_xtensa_smp_watchpoint_add() */
		curr->smp = 0;
		watchpoint_remove(curr, watchpoint->address);
		curr->smp = 1;
	}
	return ERROR_OK;
}

int esp_xtensa_smp_init_arch_info(struct target *target,
	struct esp_xtensa_smp_common *esp_xtensa_smp,
	struct xtensa_debug_module_config *dm_cfg,
	const struct esp_xtensa_smp_chip_ops *chip_ops,
	const struct esp_semihost_ops *semihost_ops)
{
	int ret = esp_xtensa_init_arch_info(target, &esp_xtensa_smp->esp_xtensa, dm_cfg, semihost_ops);
	if (ret != ERROR_OK)
		return ret;
	esp_xtensa_smp->chip_ops = chip_ops;
	esp_xtensa_smp->examine_other_cores = ESP_XTENSA_SMP_EXAMINE_OTHER_CORES;
	return ERROR_OK;
}

int esp_xtensa_smp_target_init(struct command_context *cmd_ctx, struct target *target)
{
	int ret = esp_xtensa_target_init(cmd_ctx, target);
	if (ret != ERROR_OK)
		return ret;

	if (target->smp) {
		struct target_list *head;
		foreach_smp_target(head, target->smp_targets) {
			struct target *curr = head->target;
			ret = esp_xtensa_semihosting_init(curr);
			if (ret != ERROR_OK)
				return ret;
		}
	} else {
		ret = esp_xtensa_semihosting_init(target);
		if (ret != ERROR_OK)
			return ret;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(esp_xtensa_smp_cmd_xtdef)
{
	struct target *target = get_current_target(CMD_CTX);
	if (target->smp && CMD_ARGC > 0) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->smp_targets) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(xtensa_cmd_xtdef_do,
				target_to_xtensa(curr));
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(xtensa_cmd_xtdef_do,
		target_to_xtensa(target));
}

COMMAND_HANDLER(esp_xtensa_smp_cmd_xtopt)
{
	struct target *target = get_current_target(CMD_CTX);
	if (target->smp && CMD_ARGC > 0) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->smp_targets) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(xtensa_cmd_xtopt_do,
				target_to_xtensa(curr));
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(xtensa_cmd_xtopt_do,
		target_to_xtensa(target));
}

COMMAND_HANDLER(esp_xtensa_smp_cmd_xtmem)
{
	struct target *target = get_current_target(CMD_CTX);
	if (target->smp && CMD_ARGC > 0) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->smp_targets) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(xtensa_cmd_xtmem_do,
				target_to_xtensa(curr));
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(xtensa_cmd_xtmem_do,
		target_to_xtensa(target));
}

COMMAND_HANDLER(esp_xtensa_smp_cmd_xtmpu)
{
	struct target *target = get_current_target(CMD_CTX);
	if (target->smp && CMD_ARGC > 0) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->smp_targets) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(xtensa_cmd_xtmpu_do,
				target_to_xtensa(curr));
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(xtensa_cmd_xtmpu_do,
		target_to_xtensa(target));
}

COMMAND_HANDLER(esp_xtensa_smp_cmd_xtmmu)
{
	struct target *target = get_current_target(CMD_CTX);
	if (target->smp && CMD_ARGC > 0) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->smp_targets) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(xtensa_cmd_xtmmu_do,
				target_to_xtensa(curr));
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(xtensa_cmd_xtmmu_do,
		target_to_xtensa(target));
}

COMMAND_HANDLER(esp_xtensa_smp_cmd_xtreg)
{
	struct target *target = get_current_target(CMD_CTX);
	if (target->smp && CMD_ARGC > 0) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->smp_targets) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(xtensa_cmd_xtreg_do,
				target_to_xtensa(curr));
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(xtensa_cmd_xtreg_do,
		target_to_xtensa(target));
}

COMMAND_HANDLER(esp_xtensa_smp_cmd_xtregfmt)
{
	struct target *target = get_current_target(CMD_CTX);
	if (target->smp && CMD_ARGC > 0) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->smp_targets) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(xtensa_cmd_xtregfmt_do,
				target_to_xtensa(curr));
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(xtensa_cmd_xtregfmt_do,
		target_to_xtensa(target));
}

COMMAND_HANDLER(esp_xtensa_smp_cmd_permissive_mode)
{
	struct target *target = get_current_target(CMD_CTX);
	if (target->smp && CMD_ARGC > 0) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->smp_targets) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(xtensa_cmd_permissive_mode_do,
				target_to_xtensa(curr));
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(xtensa_cmd_permissive_mode_do,
		target_to_xtensa(target));
}

COMMAND_HANDLER(esp_xtensa_smp_cmd_smpbreak)
{
	struct target *target = get_current_target(CMD_CTX);
	if (target->smp && CMD_ARGC > 0) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->smp_targets) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(xtensa_cmd_smpbreak_do, curr);
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(xtensa_cmd_smpbreak_do, target);
}

COMMAND_HANDLER(esp_xtensa_smp_cmd_mask_interrupts)
{
	struct target *target = get_current_target(CMD_CTX);
	if (target->smp && CMD_ARGC > 0) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->smp_targets) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(xtensa_cmd_mask_interrupts_do,
				target_to_xtensa(curr));
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(xtensa_cmd_mask_interrupts_do,
		target_to_xtensa(target));
}

COMMAND_HANDLER(esp_xtensa_smp_cmd_perfmon_enable)
{
	struct target *target = get_current_target(CMD_CTX);
	if (target->smp && CMD_ARGC > 0) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->smp_targets) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(xtensa_cmd_perfmon_enable_do,
				target_to_xtensa(curr));
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(xtensa_cmd_perfmon_enable_do,
		target_to_xtensa(target));
}

COMMAND_HANDLER(esp_xtensa_smp_cmd_perfmon_dump)
{
	struct target *target = get_current_target(CMD_CTX);
	if (target->smp) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->smp_targets) {
			curr = head->target;
			LOG_INFO("CPU%d:", curr->coreid);
			int ret = CALL_COMMAND_HANDLER(xtensa_cmd_perfmon_dump_do,
				target_to_xtensa(curr));
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(xtensa_cmd_perfmon_dump_do,
		target_to_xtensa(target));
}

COMMAND_HANDLER(esp_xtensa_smp_cmd_tracestart)
{
	struct target *target = get_current_target(CMD_CTX);
	if (target->smp) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->smp_targets) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(xtensa_cmd_tracestart_do,
				target_to_xtensa(curr));
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(xtensa_cmd_tracestart_do,
		target_to_xtensa(target));
}

COMMAND_HANDLER(esp_xtensa_smp_cmd_tracestop)
{
	struct target *target = get_current_target(CMD_CTX);
	if (target->smp) {
		struct target_list *head;
		struct target *curr;
		foreach_smp_target(head, target->smp_targets) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(xtensa_cmd_tracestop_do,
				target_to_xtensa(curr));
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(xtensa_cmd_tracestop_do,
		target_to_xtensa(target));
}

COMMAND_HANDLER(esp_xtensa_smp_cmd_tracedump)
{
	struct target *target = get_current_target(CMD_CTX);
	if (target->smp) {
		struct target_list *head;
		struct target *curr;
		int32_t cores_max_id = 0;
		/* assume that core IDs are assigned to SMP targets sequentially: 0,1,2... */
		foreach_smp_target(head, target->smp_targets) {
			curr = head->target;
			if (cores_max_id < curr->coreid)
				cores_max_id = curr->coreid;
		}
		if (CMD_ARGC < ((uint32_t)cores_max_id + 1)) {
			command_print(CMD,
				"Need %d filenames to dump to as output!",
				cores_max_id + 1);
			return ERROR_FAIL;
		}
		foreach_smp_target(head, target->smp_targets) {
			curr = head->target;
			int ret = CALL_COMMAND_HANDLER(xtensa_cmd_tracedump_do,
				target_to_xtensa(curr), CMD_ARGV[curr->coreid]);
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return CALL_COMMAND_HANDLER(xtensa_cmd_tracedump_do,
		target_to_xtensa(target), CMD_ARGV[0]);
}

const struct command_registration esp_xtensa_smp_xtensa_command_handlers[] = {
	{
		.name = "xtdef",
		.handler = esp_xtensa_smp_cmd_xtdef,
		.mode = COMMAND_CONFIG,
		.help = "Configure Xtensa core type",
		.usage = "<type>",
	},
	{
		.name = "xtopt",
		.handler = esp_xtensa_smp_cmd_xtopt,
		.mode = COMMAND_CONFIG,
		.help = "Configure Xtensa core option",
		.usage = "<name> <value>",
	},
	{
		.name = "xtmem",
		.handler = esp_xtensa_smp_cmd_xtmem,
		.mode = COMMAND_CONFIG,
		.help = "Configure Xtensa memory/cache option",
		.usage = "<type> [parameters]",
	},
	{
		.name = "xtmmu",
		.handler = esp_xtensa_smp_cmd_xtmmu,
		.mode = COMMAND_CONFIG,
		.help = "Configure Xtensa MMU option",
		.usage = "<NIREFILLENTRIES> <NDREFILLENTRIES> <IVARWAY56> <DVARWAY56>",
	},
	{
		.name = "xtmpu",
		.handler = esp_xtensa_smp_cmd_xtmpu,
		.mode = COMMAND_CONFIG,
		.help = "Configure Xtensa MPU option",
		.usage = "<num FG seg> <min seg size> <lockable> <executeonly>",
	},
	{
		.name = "xtreg",
		.handler = esp_xtensa_smp_cmd_xtreg,
		.mode = COMMAND_CONFIG,
		.help = "Configure Xtensa register",
		.usage = "<regname> <regnum>",
	},
	{
		.name = "xtregs",
		.handler = esp_xtensa_smp_cmd_xtreg,
		.mode = COMMAND_CONFIG,
		.help = "Configure number of Xtensa registers",
		.usage = "<numregs>",
	},
	{
		.name = "xtregfmt",
		.handler = esp_xtensa_smp_cmd_xtregfmt,
		.mode = COMMAND_CONFIG,
		.help = "Configure format of Xtensa register map",
		.usage = "<numgregs>",
	},
	{
		.name = "set_permissive",
		.handler = esp_xtensa_smp_cmd_permissive_mode,
		.mode = COMMAND_ANY,
		.help = "When set to 1, enable Xtensa permissive mode (less client-side checks)",
		.usage = "[0|1]",
	},
	{
		.name = "maskisr",
		.handler = esp_xtensa_smp_cmd_mask_interrupts,
		.mode = COMMAND_ANY,
		.help = "mask Xtensa interrupts at step",
		.usage = "['on'|'off']",
	},
	{
		.name = "smpbreak",
		.handler = esp_xtensa_smp_cmd_smpbreak,
		.mode = COMMAND_ANY,
		.help = "Set the way the CPU chains OCD breaks",
		.usage =
			"[none|breakinout|runstall] | [BreakIn] [BreakOut] [RunStallIn] [DebugModeOut]",
	},
	{
		.name = "perfmon_enable",
		.handler = esp_xtensa_smp_cmd_perfmon_enable,
		.mode = COMMAND_EXEC,
		.help = "Enable and start performance counter",
		.usage = "<counter_id> <select> [mask] [kernelcnt] [tracelevel]",
	},
	{
		.name = "perfmon_dump",
		.handler = esp_xtensa_smp_cmd_perfmon_dump,
		.mode = COMMAND_EXEC,
		.help =
			"Dump performance counter value. If no argument specified, dumps all counters.",
		.usage = "[counter_id]",
	},
	{
		.name = "tracestart",
		.handler = esp_xtensa_smp_cmd_tracestart,
		.mode = COMMAND_EXEC,
		.help =
			"Tracing: Set up and start a trace. Optionally set stop trigger address and amount of data captured after.",
		.usage = "[pc <pcval>/[maskbitcount]] [after <n> [ins|words]]",
	},
	{
		.name = "tracestop",
		.handler = esp_xtensa_smp_cmd_tracestop,
		.mode = COMMAND_EXEC,
		.help = "Tracing: Stop current trace as started by the tracestart command",
		.usage = "",
	},
	{
		.name = "tracedump",
		.handler = esp_xtensa_smp_cmd_tracedump,
		.mode = COMMAND_EXEC,
		.help = "Tracing: Dump trace memory to a files. One file per core.",
		.usage = "<outfile1> <outfile2>",
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration esp_xtensa_smp_command_handlers[] = {
	{
		.name = "xtensa",
		.usage = "",
		.chain = esp_xtensa_smp_xtensa_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};
