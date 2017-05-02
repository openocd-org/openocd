/***************************************************************************
 *   Copyright (C) 2015 by David Ung                                       *
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
 *                                                                         *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "breakpoints.h"
#include "aarch64.h"
#include "register.h"
#include "target_request.h"
#include "target_type.h"
#include "armv8_opcodes.h"
#include "armv8_cache.h"
#include <helper/time_support.h>

enum restart_mode {
	RESTART_LAZY,
	RESTART_SYNC,
};

enum halt_mode {
	HALT_LAZY,
	HALT_SYNC,
};

static int aarch64_poll(struct target *target);
static int aarch64_debug_entry(struct target *target);
static int aarch64_restore_context(struct target *target, bool bpwp);
static int aarch64_set_breakpoint(struct target *target,
	struct breakpoint *breakpoint, uint8_t matchmode);
static int aarch64_set_context_breakpoint(struct target *target,
	struct breakpoint *breakpoint, uint8_t matchmode);
static int aarch64_set_hybrid_breakpoint(struct target *target,
	struct breakpoint *breakpoint);
static int aarch64_unset_breakpoint(struct target *target,
	struct breakpoint *breakpoint);
static int aarch64_mmu(struct target *target, int *enabled);
static int aarch64_virt2phys(struct target *target,
	target_addr_t virt, target_addr_t *phys);
static int aarch64_read_apb_ap_memory(struct target *target,
	uint64_t address, uint32_t size, uint32_t count, uint8_t *buffer);

#define foreach_smp_target(pos, head) \
	for (pos = head; (pos != NULL); pos = pos->next)

static int aarch64_restore_system_control_reg(struct target *target)
{
	enum arm_mode target_mode = ARM_MODE_ANY;
	int retval = ERROR_OK;
	uint32_t instr;

	struct aarch64_common *aarch64 = target_to_aarch64(target);
	struct armv8_common *armv8 = target_to_armv8(target);

	if (aarch64->system_control_reg != aarch64->system_control_reg_curr) {
		aarch64->system_control_reg_curr = aarch64->system_control_reg;
		/* LOG_INFO("cp15_control_reg: %8.8" PRIx32, cortex_v8->cp15_control_reg); */

		switch (armv8->arm.core_mode) {
		case ARMV8_64_EL0T:
			target_mode = ARMV8_64_EL1H;
			/* fall through */
		case ARMV8_64_EL1T:
		case ARMV8_64_EL1H:
			instr = ARMV8_MSR_GP(SYSTEM_SCTLR_EL1, 0);
			break;
		case ARMV8_64_EL2T:
		case ARMV8_64_EL2H:
			instr = ARMV8_MSR_GP(SYSTEM_SCTLR_EL2, 0);
			break;
		case ARMV8_64_EL3H:
		case ARMV8_64_EL3T:
			instr = ARMV8_MSR_GP(SYSTEM_SCTLR_EL3, 0);
			break;

		case ARM_MODE_SVC:
		case ARM_MODE_ABT:
		case ARM_MODE_FIQ:
		case ARM_MODE_IRQ:
			instr = ARMV4_5_MCR(15, 0, 0, 1, 0, 0);
			break;

		default:
			LOG_INFO("cannot read system control register in this mode");
			return ERROR_FAIL;
		}

		if (target_mode != ARM_MODE_ANY)
			armv8_dpm_modeswitch(&armv8->dpm, target_mode);

		retval = armv8->dpm.instr_write_data_r0(&armv8->dpm, instr, aarch64->system_control_reg);
		if (retval != ERROR_OK)
			return retval;

		if (target_mode != ARM_MODE_ANY)
			armv8_dpm_modeswitch(&armv8->dpm, ARM_MODE_ANY);
	}

	return retval;
}

/*  modify system_control_reg in order to enable or disable mmu for :
 *  - virt2phys address conversion
 *  - read or write memory in phys or virt address */
static int aarch64_mmu_modify(struct target *target, int enable)
{
	struct aarch64_common *aarch64 = target_to_aarch64(target);
	struct armv8_common *armv8 = &aarch64->armv8_common;
	int retval = ERROR_OK;
	uint32_t instr = 0;

	if (enable) {
		/*	if mmu enabled at target stop and mmu not enable */
		if (!(aarch64->system_control_reg & 0x1U)) {
			LOG_ERROR("trying to enable mmu on target stopped with mmu disable");
			return ERROR_FAIL;
		}
		if (!(aarch64->system_control_reg_curr & 0x1U))
			aarch64->system_control_reg_curr |= 0x1U;
	} else {
		if (aarch64->system_control_reg_curr & 0x4U) {
			/*  data cache is active */
			aarch64->system_control_reg_curr &= ~0x4U;
			/* flush data cache armv8 function to be called */
			if (armv8->armv8_mmu.armv8_cache.flush_all_data_cache)
				armv8->armv8_mmu.armv8_cache.flush_all_data_cache(target);
		}
		if ((aarch64->system_control_reg_curr & 0x1U)) {
			aarch64->system_control_reg_curr &= ~0x1U;
		}
	}

	switch (armv8->arm.core_mode) {
	case ARMV8_64_EL0T:
	case ARMV8_64_EL1T:
	case ARMV8_64_EL1H:
		instr = ARMV8_MSR_GP(SYSTEM_SCTLR_EL1, 0);
		break;
	case ARMV8_64_EL2T:
	case ARMV8_64_EL2H:
		instr = ARMV8_MSR_GP(SYSTEM_SCTLR_EL2, 0);
		break;
	case ARMV8_64_EL3H:
	case ARMV8_64_EL3T:
		instr = ARMV8_MSR_GP(SYSTEM_SCTLR_EL3, 0);
		break;
	default:
		LOG_DEBUG("unknown cpu state 0x%x" PRIx32, armv8->arm.core_state);
		break;
	}

	retval = armv8->dpm.instr_write_data_r0(&armv8->dpm, instr,
				aarch64->system_control_reg_curr);
	return retval;
}

/*
 * Basic debug access, very low level assumes state is saved
 */
static int aarch64_init_debug_access(struct target *target)
{
	struct armv8_common *armv8 = target_to_armv8(target);
	int retval;
	uint32_t dummy;

	LOG_DEBUG(" ");

	retval = mem_ap_write_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_OSLAR, 0);
	if (retval != ERROR_OK) {
		LOG_DEBUG("Examine %s failed", "oslock");
		return retval;
	}

	/* Clear Sticky Power Down status Bit in PRSR to enable access to
	   the registers in the Core Power Domain */
	retval = mem_ap_read_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_PRSR, &dummy);
	if (retval != ERROR_OK)
		return retval;

	/*
	 * Static CTI configuration:
	 * Channel 0 -> trigger outputs HALT request to PE
	 * Channel 1 -> trigger outputs Resume request to PE
	 * Gate all channel trigger events from entering the CTM
	 */

	/* Enable CTI */
	retval = arm_cti_enable(armv8->cti, true);
	/* By default, gate all channel events to and from the CTM */
	if (retval == ERROR_OK)
		retval = arm_cti_write_reg(armv8->cti, CTI_GATE, 0);
	/* output halt requests to PE on channel 0 event */
	if (retval == ERROR_OK)
		retval = arm_cti_write_reg(armv8->cti, CTI_OUTEN0, CTI_CHNL(0));
	/* output restart requests to PE on channel 1 event */
	if (retval == ERROR_OK)
		retval = arm_cti_write_reg(armv8->cti, CTI_OUTEN1, CTI_CHNL(1));
	if (retval != ERROR_OK)
		return retval;

	/* Resync breakpoint registers */

	return ERROR_OK;
}

/* Write to memory mapped registers directly with no cache or mmu handling */
static int aarch64_dap_write_memap_register_u32(struct target *target,
	uint32_t address,
	uint32_t value)
{
	int retval;
	struct armv8_common *armv8 = target_to_armv8(target);

	retval = mem_ap_write_atomic_u32(armv8->debug_ap, address, value);

	return retval;
}

static int aarch64_dpm_setup(struct aarch64_common *a8, uint64_t debug)
{
	struct arm_dpm *dpm = &a8->armv8_common.dpm;
	int retval;

	dpm->arm = &a8->armv8_common.arm;
	dpm->didr = debug;

	retval = armv8_dpm_setup(dpm);
	if (retval == ERROR_OK)
		retval = armv8_dpm_initialize(dpm);

	return retval;
}

static int aarch64_set_dscr_bits(struct target *target, unsigned long bit_mask, unsigned long value)
{
	struct armv8_common *armv8 = target_to_armv8(target);
	return armv8_set_dbgreg_bits(armv8, CPUV8_DBG_DSCR, bit_mask, value);
}

static int aarch64_check_state_one(struct target *target,
		uint32_t mask, uint32_t val, int *p_result, uint32_t *p_prsr)
{
	struct armv8_common *armv8 = target_to_armv8(target);
	uint32_t prsr;
	int retval;

	retval = mem_ap_read_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_PRSR, &prsr);
	if (retval != ERROR_OK)
		return retval;

	if (p_prsr)
		*p_prsr = prsr;

	if (p_result)
		*p_result = (prsr & mask) == (val & mask);

	return ERROR_OK;
}

static int aarch64_wait_halt_one(struct target *target)
{
	int retval = ERROR_OK;
	uint32_t prsr;

	int64_t then = timeval_ms();
	for (;;) {
		int halted;

		retval = aarch64_check_state_one(target, PRSR_HALT, PRSR_HALT, &halted, &prsr);
		if (retval != ERROR_OK || halted)
			break;

		if (timeval_ms() > then + 1000) {
			retval = ERROR_TARGET_TIMEOUT;
			LOG_DEBUG("target %s timeout, prsr=0x%08"PRIx32, target_name(target), prsr);
			break;
		}
	}
	return retval;
}

static int aarch64_prepare_halt_smp(struct target *target, bool exc_target, struct target **p_first)
{
	int retval = ERROR_OK;
	struct target_list *head = target->head;
	struct target *first = NULL;

	LOG_DEBUG("target %s exc %i", target_name(target), exc_target);

	while (head != NULL) {
		struct target *curr = head->target;
		struct armv8_common *armv8 = target_to_armv8(curr);
		head = head->next;

		if (exc_target && curr == target)
			continue;
		if (!target_was_examined(curr))
			continue;
		if (curr->state != TARGET_RUNNING)
			continue;

		/* HACK: mark this target as prepared for halting */
		curr->debug_reason = DBG_REASON_DBGRQ;

		/* open the gate for channel 0 to let HALT requests pass to the CTM */
		retval = arm_cti_ungate_channel(armv8->cti, 0);
		if (retval == ERROR_OK)
			retval = aarch64_set_dscr_bits(curr, DSCR_HDE, DSCR_HDE);
		if (retval != ERROR_OK)
			break;

		LOG_DEBUG("target %s prepared", target_name(curr));

		if (first == NULL)
			first = curr;
	}

	if (p_first) {
		if (exc_target && first)
			*p_first = first;
		else
			*p_first = target;
	}

	return retval;
}

static int aarch64_halt_one(struct target *target, enum halt_mode mode)
{
	int retval = ERROR_OK;
	struct armv8_common *armv8 = target_to_armv8(target);

	LOG_DEBUG("%s", target_name(target));

	/* allow Halting Debug Mode */
	retval = aarch64_set_dscr_bits(target, DSCR_HDE, DSCR_HDE);
	if (retval != ERROR_OK)
		return retval;

	/* trigger an event on channel 0, this outputs a halt request to the PE */
	retval = arm_cti_pulse_channel(armv8->cti, 0);
	if (retval != ERROR_OK)
		return retval;

	if (mode == HALT_SYNC) {
		retval = aarch64_wait_halt_one(target);
		if (retval != ERROR_OK) {
			if (retval == ERROR_TARGET_TIMEOUT)
				LOG_ERROR("Timeout waiting for target %s halt", target_name(target));
			return retval;
		}
	}

	return ERROR_OK;
}

static int aarch64_halt_smp(struct target *target, bool exc_target)
{
	struct target *next = target;
	int retval;

	/* prepare halt on all PEs of the group */
	retval = aarch64_prepare_halt_smp(target, exc_target, &next);

	if (exc_target && next == target)
		return retval;

	/* halt the target PE */
	if (retval == ERROR_OK)
		retval = aarch64_halt_one(next, HALT_LAZY);

	if (retval != ERROR_OK)
		return retval;

	/* wait for all PEs to halt */
	int64_t then = timeval_ms();
	for (;;) {
		bool all_halted = true;
		struct target_list *head;
		struct target *curr;

		foreach_smp_target(head, target->head) {
			int halted;

			curr = head->target;

			if (!target_was_examined(curr))
				continue;

			retval = aarch64_check_state_one(curr, PRSR_HALT, PRSR_HALT, &halted, NULL);
			if (retval != ERROR_OK || !halted) {
				all_halted = false;
				break;
			}
		}

		if (all_halted)
			break;

		if (timeval_ms() > then + 1000) {
			retval = ERROR_TARGET_TIMEOUT;
			break;
		}

		/*
		 * HACK: on Hi6220 there are 8 cores organized in 2 clusters
		 * and it looks like the CTI's are not connected by a common
		 * trigger matrix. It seems that we need to halt one core in each
		 * cluster explicitly. So if we find that a core has not halted
		 * yet, we trigger an explicit halt for the second cluster.
		 */
		retval = aarch64_halt_one(curr, HALT_LAZY);
		if (retval != ERROR_OK)
			break;
	}

	return retval;
}

static int update_halt_gdb(struct target *target, enum target_debug_reason debug_reason)
{
	struct target *gdb_target = NULL;
	struct target_list *head;
	struct target *curr;

	if (debug_reason == DBG_REASON_NOTHALTED) {
		LOG_INFO("Halting remaining targets in SMP group");
		aarch64_halt_smp(target, true);
	}

	/* poll all targets in the group, but skip the target that serves GDB */
	foreach_smp_target(head, target->head) {
		curr = head->target;
		/* skip calling context */
		if (curr == target)
			continue;
		if (!target_was_examined(curr))
			continue;
		/* skip targets that were already halted */
		if (curr->state == TARGET_HALTED)
			continue;
		/* remember the gdb_service->target */
		if (curr->gdb_service != NULL)
			gdb_target = curr->gdb_service->target;
		/* skip it */
		if (curr == gdb_target)
			continue;

		/* avoid recursion in aarch64_poll() */
		curr->smp = 0;
		aarch64_poll(curr);
		curr->smp = 1;
	}

	/* after all targets were updated, poll the gdb serving target */
	if (gdb_target != NULL && gdb_target != target)
		aarch64_poll(gdb_target);

	return ERROR_OK;
}

/*
 * Aarch64 Run control
 */

static int aarch64_poll(struct target *target)
{
	enum target_state prev_target_state;
	int retval = ERROR_OK;
	int halted;

	retval = aarch64_check_state_one(target,
				PRSR_HALT, PRSR_HALT, &halted, NULL);
	if (retval != ERROR_OK)
		return retval;

	if (halted) {
		prev_target_state = target->state;
		if (prev_target_state != TARGET_HALTED) {
			enum target_debug_reason debug_reason = target->debug_reason;

			/* We have a halting debug event */
			target->state = TARGET_HALTED;
			LOG_DEBUG("Target %s halted", target_name(target));
			retval = aarch64_debug_entry(target);
			if (retval != ERROR_OK)
				return retval;

			if (target->smp)
				update_halt_gdb(target, debug_reason);

			switch (prev_target_state) {
			case TARGET_RUNNING:
			case TARGET_UNKNOWN:
			case TARGET_RESET:
				target_call_event_callbacks(target, TARGET_EVENT_HALTED);
				break;
			case TARGET_DEBUG_RUNNING:
				target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
				break;
			default:
				break;
			}
		}
	} else
		target->state = TARGET_RUNNING;

	return retval;
}

static int aarch64_halt(struct target *target)
{
	if (target->smp)
		return aarch64_halt_smp(target, false);

	return aarch64_halt_one(target, HALT_SYNC);
}

static int aarch64_restore_one(struct target *target, int current,
	uint64_t *address, int handle_breakpoints, int debug_execution)
{
	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm *arm = &armv8->arm;
	int retval;
	uint64_t resume_pc;

	LOG_DEBUG("%s", target_name(target));

	if (!debug_execution)
		target_free_all_working_areas(target);

	/* current = 1: continue on current pc, otherwise continue at <address> */
	resume_pc = buf_get_u64(arm->pc->value, 0, 64);
	if (!current)
		resume_pc = *address;
	else
		*address = resume_pc;

	/* Make sure that the Armv7 gdb thumb fixups does not
	 * kill the return address
	 */
	switch (arm->core_state) {
		case ARM_STATE_ARM:
			resume_pc &= 0xFFFFFFFC;
			break;
		case ARM_STATE_AARCH64:
			resume_pc &= 0xFFFFFFFFFFFFFFFC;
			break;
		case ARM_STATE_THUMB:
		case ARM_STATE_THUMB_EE:
			/* When the return address is loaded into PC
			 * bit 0 must be 1 to stay in Thumb state
			 */
			resume_pc |= 0x1;
			break;
		case ARM_STATE_JAZELLE:
			LOG_ERROR("How do I resume into Jazelle state??");
			return ERROR_FAIL;
	}
	LOG_DEBUG("resume pc = 0x%016" PRIx64, resume_pc);
	buf_set_u64(arm->pc->value, 0, 64, resume_pc);
	arm->pc->dirty = 1;
	arm->pc->valid = 1;

	/* called it now before restoring context because it uses cpu
	 * register r0 for restoring system control register */
	retval = aarch64_restore_system_control_reg(target);
	if (retval == ERROR_OK)
		retval = aarch64_restore_context(target, handle_breakpoints);

	return retval;
}

/**
 * prepare single target for restart
 *
 *
 */
static int aarch64_prepare_restart_one(struct target *target)
{
	struct armv8_common *armv8 = target_to_armv8(target);
	int retval;
	uint32_t dscr;
	uint32_t tmp;

	LOG_DEBUG("%s", target_name(target));

	retval = mem_ap_read_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		return retval;

	if ((dscr & DSCR_ITE) == 0)
		LOG_ERROR("DSCR.ITE must be set before leaving debug!");
	if ((dscr & DSCR_ERR) != 0)
		LOG_ERROR("DSCR.ERR must be cleared before leaving debug!");

	/* acknowledge a pending CTI halt event */
	retval = arm_cti_ack_events(armv8->cti, CTI_TRIG(HALT));
	/*
	 * open the CTI gate for channel 1 so that the restart events
	 * get passed along to all PEs. Also close gate for channel 0
	 * to isolate the PE from halt events.
	 */
	if (retval == ERROR_OK)
		retval = arm_cti_ungate_channel(armv8->cti, 1);
	if (retval == ERROR_OK)
		retval = arm_cti_gate_channel(armv8->cti, 0);

	/* make sure that DSCR.HDE is set */
	if (retval == ERROR_OK) {
		dscr |= DSCR_HDE;
		retval = mem_ap_write_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_DSCR, dscr);
	}

	/* clear sticky bits in PRSR, SDR is now 0 */
	retval = mem_ap_read_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_PRSR, &tmp);

	return retval;
}

static int aarch64_do_restart_one(struct target *target, enum restart_mode mode)
{
	struct armv8_common *armv8 = target_to_armv8(target);
	int retval;

	LOG_DEBUG("%s", target_name(target));

	/* trigger an event on channel 1, generates a restart request to the PE */
	retval = arm_cti_pulse_channel(armv8->cti, 1);
	if (retval != ERROR_OK)
		return retval;

	if (mode == RESTART_SYNC) {
		int64_t then = timeval_ms();
		for (;;) {
			int resumed;
			/*
			 * if PRSR.SDR is set now, the target did restart, even
			 * if it's now already halted again (e.g. due to breakpoint)
			 */
			retval = aarch64_check_state_one(target,
						PRSR_SDR, PRSR_SDR, &resumed, NULL);
			if (retval != ERROR_OK || resumed)
				break;

			if (timeval_ms() > then + 1000) {
				LOG_ERROR("%s: Timeout waiting for resume"PRIx32, target_name(target));
				retval = ERROR_TARGET_TIMEOUT;
				break;
			}
		}
	}

	if (retval != ERROR_OK)
		return retval;

	target->debug_reason = DBG_REASON_NOTHALTED;
	target->state = TARGET_RUNNING;

	return ERROR_OK;
}

static int aarch64_restart_one(struct target *target, enum restart_mode mode)
{
	int retval;

	LOG_DEBUG("%s", target_name(target));

	retval = aarch64_prepare_restart_one(target);
	if (retval == ERROR_OK)
		retval = aarch64_do_restart_one(target, mode);

	return retval;
}

/*
 * prepare all but the current target for restart
 */
static int aarch64_prep_restart_smp(struct target *target, int handle_breakpoints, struct target **p_first)
{
	int retval = ERROR_OK;
	struct target_list *head;
	struct target *first = NULL;
	uint64_t address;

	foreach_smp_target(head, target->head) {
		struct target *curr = head->target;

		/* skip calling target */
		if (curr == target)
			continue;
		if (!target_was_examined(curr))
			continue;
		if (curr->state != TARGET_HALTED)
			continue;

		/*  resume at current address, not in step mode */
		retval = aarch64_restore_one(curr, 1, &address, handle_breakpoints, 0);
		if (retval == ERROR_OK)
			retval = aarch64_prepare_restart_one(curr);
		if (retval != ERROR_OK) {
			LOG_ERROR("failed to restore target %s", target_name(curr));
			break;
		}
		/* remember the first valid target in the group */
		if (first == NULL)
			first = curr;
	}

	if (p_first)
		*p_first = first;

	return retval;
}


static int aarch64_step_restart_smp(struct target *target)
{
	int retval = ERROR_OK;
	struct target_list *head;
	struct target *first = NULL;

	LOG_DEBUG("%s", target_name(target));

	retval = aarch64_prep_restart_smp(target, 0, &first);
	if (retval != ERROR_OK)
		return retval;

	if (first != NULL)
		retval = aarch64_do_restart_one(first, RESTART_LAZY);
	if (retval != ERROR_OK) {
		LOG_DEBUG("error restarting target %s", target_name(first));
		return retval;
	}

	int64_t then = timeval_ms();
	for (;;) {
		struct target *curr = target;
		bool all_resumed = true;

		foreach_smp_target(head, target->head) {
			uint32_t prsr;
			int resumed;

			curr = head->target;

			if (curr == target)
				continue;

			retval = aarch64_check_state_one(curr,
					PRSR_SDR, PRSR_SDR, &resumed, &prsr);
			if (retval != ERROR_OK || (!resumed && (prsr & PRSR_HALT))) {
				all_resumed = false;
				break;
			}

			if (curr->state != TARGET_RUNNING) {
				curr->state = TARGET_RUNNING;
				curr->debug_reason = DBG_REASON_NOTHALTED;
				target_call_event_callbacks(curr, TARGET_EVENT_RESUMED);
			}
		}

		if (all_resumed)
			break;

		if (timeval_ms() > then + 1000) {
			LOG_ERROR("%s: timeout waiting for target resume", __func__);
			retval = ERROR_TARGET_TIMEOUT;
			break;
		}
		/*
		 * HACK: on Hi6220 there are 8 cores organized in 2 clusters
		 * and it looks like the CTI's are not connected by a common
		 * trigger matrix. It seems that we need to halt one core in each
		 * cluster explicitly. So if we find that a core has not halted
		 * yet, we trigger an explicit resume for the second cluster.
		 */
		retval = aarch64_do_restart_one(curr, RESTART_LAZY);
		if (retval != ERROR_OK)
			break;
}

	return retval;
}

static int aarch64_resume(struct target *target, int current,
	target_addr_t address, int handle_breakpoints, int debug_execution)
{
	int retval = 0;
	uint64_t addr = address;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	/*
	 * If this target is part of a SMP group, prepare the others
	 * targets for resuming. This involves restoring the complete
	 * target register context and setting up CTI gates to accept
	 * resume events from the trigger matrix.
	 */
	if (target->smp) {
		retval = aarch64_prep_restart_smp(target, handle_breakpoints, NULL);
		if (retval != ERROR_OK)
			return retval;
	}

	/* all targets prepared, restore and restart the current target */
	retval = aarch64_restore_one(target, current, &addr, handle_breakpoints,
				 debug_execution);
	if (retval == ERROR_OK)
		retval = aarch64_restart_one(target, RESTART_SYNC);
	if (retval != ERROR_OK)
		return retval;

	if (target->smp) {
		int64_t then = timeval_ms();
		for (;;) {
			struct target *curr = target;
			struct target_list *head;
			bool all_resumed = true;

			foreach_smp_target(head, target->head) {
				uint32_t prsr;
				int resumed;

				curr = head->target;
				if (curr == target)
					continue;
				if (!target_was_examined(curr))
					continue;

				retval = aarch64_check_state_one(curr,
						PRSR_SDR, PRSR_SDR, &resumed, &prsr);
				if (retval != ERROR_OK || (!resumed && (prsr & PRSR_HALT))) {
					all_resumed = false;
					break;
				}

				if (curr->state != TARGET_RUNNING) {
					curr->state = TARGET_RUNNING;
					curr->debug_reason = DBG_REASON_NOTHALTED;
					target_call_event_callbacks(curr, TARGET_EVENT_RESUMED);
				}
			}

			if (all_resumed)
				break;

			if (timeval_ms() > then + 1000) {
				LOG_ERROR("%s: timeout waiting for target %s to resume", __func__, target_name(curr));
				retval = ERROR_TARGET_TIMEOUT;
				break;
			}

			/*
			 * HACK: on Hi6220 there are 8 cores organized in 2 clusters
			 * and it looks like the CTI's are not connected by a common
			 * trigger matrix. It seems that we need to halt one core in each
			 * cluster explicitly. So if we find that a core has not halted
			 * yet, we trigger an explicit resume for the second cluster.
			 */
			retval = aarch64_do_restart_one(curr, RESTART_LAZY);
			if (retval != ERROR_OK)
				break;
		}
	}

	if (retval != ERROR_OK)
		return retval;

	target->debug_reason = DBG_REASON_NOTHALTED;

	if (!debug_execution) {
		target->state = TARGET_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		LOG_DEBUG("target resumed at 0x%" PRIx64, addr);
	} else {
		target->state = TARGET_DEBUG_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED);
		LOG_DEBUG("target debug resumed at 0x%" PRIx64, addr);
	}

	return ERROR_OK;
}

static int aarch64_debug_entry(struct target *target)
{
	int retval = ERROR_OK;
	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm_dpm *dpm = &armv8->dpm;
	enum arm_state core_state;
	uint32_t dscr;

	/* make sure to clear all sticky errors */
	retval = mem_ap_write_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_DRCR, DRCR_CSE);
	if (retval == ERROR_OK)
		retval = mem_ap_read_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_DSCR, &dscr);
	if (retval == ERROR_OK)
		retval = arm_cti_ack_events(armv8->cti, CTI_TRIG(HALT));

	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("%s dscr = 0x%08" PRIx32, target_name(target), dscr);

	dpm->dscr = dscr;
	core_state = armv8_dpm_get_core_state(dpm);
	armv8_select_opcodes(armv8, core_state == ARM_STATE_AARCH64);
	armv8_select_reg_access(armv8, core_state == ARM_STATE_AARCH64);

	/* close the CTI gate for all events */
	if (retval == ERROR_OK)
		retval = arm_cti_write_reg(armv8->cti, CTI_GATE, 0);
	/* discard async exceptions */
	if (retval == ERROR_OK)
		retval = dpm->instr_cpsr_sync(dpm);
	if (retval != ERROR_OK)
		return retval;

	/* Examine debug reason */
	armv8_dpm_report_dscr(dpm, dscr);

	/* save address of instruction that triggered the watchpoint? */
	if (target->debug_reason == DBG_REASON_WATCHPOINT) {
		uint32_t tmp;
		uint64_t wfar = 0;

		retval = mem_ap_read_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_WFAR1,
				&tmp);
		if (retval != ERROR_OK)
			return retval;
		wfar = tmp;
		wfar = (wfar << 32);
		retval = mem_ap_read_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_WFAR0,
				&tmp);
		if (retval != ERROR_OK)
			return retval;
		wfar |= tmp;
		armv8_dpm_report_wfar(&armv8->dpm, wfar);
	}

	retval = armv8_dpm_read_current_registers(&armv8->dpm);

	if (retval == ERROR_OK && armv8->post_debug_entry)
		retval = armv8->post_debug_entry(target);

	return retval;
}

static int aarch64_post_debug_entry(struct target *target)
{
	struct aarch64_common *aarch64 = target_to_aarch64(target);
	struct armv8_common *armv8 = &aarch64->armv8_common;
	int retval;
	enum arm_mode target_mode = ARM_MODE_ANY;
	uint32_t instr;

	switch (armv8->arm.core_mode) {
	case ARMV8_64_EL0T:
		target_mode = ARMV8_64_EL1H;
		/* fall through */
	case ARMV8_64_EL1T:
	case ARMV8_64_EL1H:
		instr = ARMV8_MRS(SYSTEM_SCTLR_EL1, 0);
		break;
	case ARMV8_64_EL2T:
	case ARMV8_64_EL2H:
		instr = ARMV8_MRS(SYSTEM_SCTLR_EL2, 0);
		break;
	case ARMV8_64_EL3H:
	case ARMV8_64_EL3T:
		instr = ARMV8_MRS(SYSTEM_SCTLR_EL3, 0);
		break;

	case ARM_MODE_SVC:
	case ARM_MODE_ABT:
	case ARM_MODE_FIQ:
	case ARM_MODE_IRQ:
		instr = ARMV4_5_MRC(15, 0, 0, 1, 0, 0);
		break;

	default:
		LOG_INFO("cannot read system control register in this mode");
		return ERROR_FAIL;
	}

	if (target_mode != ARM_MODE_ANY)
		armv8_dpm_modeswitch(&armv8->dpm, target_mode);

	retval = armv8->dpm.instr_read_data_r0(&armv8->dpm, instr, &aarch64->system_control_reg);
	if (retval != ERROR_OK)
		return retval;

	if (target_mode != ARM_MODE_ANY)
		armv8_dpm_modeswitch(&armv8->dpm, ARM_MODE_ANY);

	LOG_DEBUG("System_register: %8.8" PRIx32, aarch64->system_control_reg);
	aarch64->system_control_reg_curr = aarch64->system_control_reg;

	if (armv8->armv8_mmu.armv8_cache.info == -1) {
		armv8_identify_cache(armv8);
		armv8_read_mpidr(armv8);
	}

	armv8->armv8_mmu.mmu_enabled =
			(aarch64->system_control_reg & 0x1U) ? 1 : 0;
	armv8->armv8_mmu.armv8_cache.d_u_cache_enabled =
		(aarch64->system_control_reg & 0x4U) ? 1 : 0;
	armv8->armv8_mmu.armv8_cache.i_cache_enabled =
		(aarch64->system_control_reg & 0x1000U) ? 1 : 0;
	return ERROR_OK;
}

/*
 * single-step a target
 */
static int aarch64_step(struct target *target, int current, target_addr_t address,
	int handle_breakpoints)
{
	struct armv8_common *armv8 = target_to_armv8(target);
	int saved_retval = ERROR_OK;
	int retval;
	uint32_t edecr;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = mem_ap_read_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_EDECR, &edecr);
	/* make sure EDECR.SS is not set when restoring the register */

	if (retval == ERROR_OK) {
		edecr &= ~0x4;
		/* set EDECR.SS to enter hardware step mode */
		retval = mem_ap_write_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_EDECR, (edecr|0x4));
	}
	/* disable interrupts while stepping */
	if (retval == ERROR_OK)
		retval = aarch64_set_dscr_bits(target, 0x3 << 22, 0x3 << 22);
	/* bail out if stepping setup has failed */
	if (retval != ERROR_OK)
		return retval;

	if (target->smp && !handle_breakpoints) {
		/*
		 * isolate current target so that it doesn't get resumed
		 * together with the others
		 */
		retval = arm_cti_gate_channel(armv8->cti, 1);
		/* resume all other targets in the group */
		if (retval == ERROR_OK)
			retval = aarch64_step_restart_smp(target);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to restart non-stepping targets in SMP group");
			return retval;
		}
		LOG_DEBUG("Restarted all non-stepping targets in SMP group");
	}

	/* all other targets running, restore and restart the current target */
	retval = aarch64_restore_one(target, current, &address, 0, 0);
	if (retval == ERROR_OK)
		retval = aarch64_restart_one(target, RESTART_LAZY);

	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("target step-resumed at 0x%" PRIx64, address);
	if (!handle_breakpoints)
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);

	int64_t then = timeval_ms();
	for (;;) {
		int stepped;
		uint32_t prsr;

		retval = aarch64_check_state_one(target,
					PRSR_SDR|PRSR_HALT, PRSR_SDR|PRSR_HALT, &stepped, &prsr);
		if (retval != ERROR_OK || stepped)
			break;

		if (timeval_ms() > then + 1000) {
			LOG_ERROR("timeout waiting for target %s halt after step",
					target_name(target));
			retval = ERROR_TARGET_TIMEOUT;
			break;
		}
	}

	if (retval == ERROR_TARGET_TIMEOUT)
		saved_retval = retval;

	/* restore EDECR */
	retval = mem_ap_write_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_EDECR, edecr);
	if (retval != ERROR_OK)
		return retval;

	/* restore interrupts */
	retval = aarch64_set_dscr_bits(target, 0x3 << 22, 0);
	if (retval != ERROR_OK)
		return ERROR_OK;

	if (saved_retval != ERROR_OK)
		return saved_retval;

	return aarch64_poll(target);
}

static int aarch64_restore_context(struct target *target, bool bpwp)
{
	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm *arm = &armv8->arm;

	int retval;

	LOG_DEBUG("%s", target_name(target));

	if (armv8->pre_restore_context)
		armv8->pre_restore_context(target);

	retval = armv8_dpm_write_dirty_registers(&armv8->dpm, bpwp);
	if (retval == ERROR_OK) {
		/* registers are now invalid */
		register_cache_invalidate(arm->core_cache);
		register_cache_invalidate(arm->core_cache->next);
	}

	return retval;
}

/*
 * Cortex-A8 Breakpoint and watchpoint functions
 */

/* Setup hardware Breakpoint Register Pair */
static int aarch64_set_breakpoint(struct target *target,
	struct breakpoint *breakpoint, uint8_t matchmode)
{
	int retval;
	int brp_i = 0;
	uint32_t control;
	uint8_t byte_addr_select = 0x0F;
	struct aarch64_common *aarch64 = target_to_aarch64(target);
	struct armv8_common *armv8 = &aarch64->armv8_common;
	struct aarch64_brp *brp_list = aarch64->brp_list;

	if (breakpoint->set) {
		LOG_WARNING("breakpoint already set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD) {
		int64_t bpt_value;
		while (brp_list[brp_i].used && (brp_i < aarch64->brp_num))
			brp_i++;
		if (brp_i >= aarch64->brp_num) {
			LOG_ERROR("ERROR Can not find free Breakpoint Register Pair");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		breakpoint->set = brp_i + 1;
		if (breakpoint->length == 2)
			byte_addr_select = (3 << (breakpoint->address & 0x02));
		control = ((matchmode & 0x7) << 20)
			| (1 << 13)
			| (byte_addr_select << 5)
			| (3 << 1) | 1;
		brp_list[brp_i].used = 1;
		brp_list[brp_i].value = breakpoint->address & 0xFFFFFFFFFFFFFFFC;
		brp_list[brp_i].control = control;
		bpt_value = brp_list[brp_i].value;

		retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
				+ CPUV8_DBG_BVR_BASE + 16 * brp_list[brp_i].BRPn,
				(uint32_t)(bpt_value & 0xFFFFFFFF));
		if (retval != ERROR_OK)
			return retval;
		retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
				+ CPUV8_DBG_BVR_BASE + 4 + 16 * brp_list[brp_i].BRPn,
				(uint32_t)(bpt_value >> 32));
		if (retval != ERROR_OK)
			return retval;

		retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
				+ CPUV8_DBG_BCR_BASE + 16 * brp_list[brp_i].BRPn,
				brp_list[brp_i].control);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("brp %i control 0x%0" PRIx32 " value 0x%" TARGET_PRIxADDR, brp_i,
			brp_list[brp_i].control,
			brp_list[brp_i].value);

	} else if (breakpoint->type == BKPT_SOFT) {
		uint8_t code[4];

		buf_set_u32(code, 0, 32, armv8_opcode(armv8, ARMV8_OPC_HLT));
		retval = target_read_memory(target,
				breakpoint->address & 0xFFFFFFFFFFFFFFFE,
				breakpoint->length, 1,
				breakpoint->orig_instr);
		if (retval != ERROR_OK)
			return retval;

		armv8_cache_d_inner_flush_virt(armv8,
				breakpoint->address & 0xFFFFFFFFFFFFFFFE,
				breakpoint->length);

		retval = target_write_memory(target,
				breakpoint->address & 0xFFFFFFFFFFFFFFFE,
				breakpoint->length, 1, code);
		if (retval != ERROR_OK)
			return retval;

		armv8_cache_d_inner_flush_virt(armv8,
				breakpoint->address & 0xFFFFFFFFFFFFFFFE,
				breakpoint->length);

		armv8_cache_i_inner_inval_virt(armv8,
				breakpoint->address & 0xFFFFFFFFFFFFFFFE,
				breakpoint->length);

		breakpoint->set = 0x11;	/* Any nice value but 0 */
	}

	/* Ensure that halting debug mode is enable */
	retval = aarch64_set_dscr_bits(target, DSCR_HDE, DSCR_HDE);
	if (retval != ERROR_OK) {
		LOG_DEBUG("Failed to set DSCR.HDE");
		return retval;
	}

	return ERROR_OK;
}

static int aarch64_set_context_breakpoint(struct target *target,
	struct breakpoint *breakpoint, uint8_t matchmode)
{
	int retval = ERROR_FAIL;
	int brp_i = 0;
	uint32_t control;
	uint8_t byte_addr_select = 0x0F;
	struct aarch64_common *aarch64 = target_to_aarch64(target);
	struct armv8_common *armv8 = &aarch64->armv8_common;
	struct aarch64_brp *brp_list = aarch64->brp_list;

	if (breakpoint->set) {
		LOG_WARNING("breakpoint already set");
		return retval;
	}
	/*check available context BRPs*/
	while ((brp_list[brp_i].used ||
		(brp_list[brp_i].type != BRP_CONTEXT)) && (brp_i < aarch64->brp_num))
		brp_i++;

	if (brp_i >= aarch64->brp_num) {
		LOG_ERROR("ERROR Can not find free Breakpoint Register Pair");
		return ERROR_FAIL;
	}

	breakpoint->set = brp_i + 1;
	control = ((matchmode & 0x7) << 20)
		| (1 << 13)
		| (byte_addr_select << 5)
		| (3 << 1) | 1;
	brp_list[brp_i].used = 1;
	brp_list[brp_i].value = (breakpoint->asid);
	brp_list[brp_i].control = control;
	retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
			+ CPUV8_DBG_BVR_BASE + 16 * brp_list[brp_i].BRPn,
			brp_list[brp_i].value);
	if (retval != ERROR_OK)
		return retval;
	retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
			+ CPUV8_DBG_BCR_BASE + 16 * brp_list[brp_i].BRPn,
			brp_list[brp_i].control);
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("brp %i control 0x%0" PRIx32 " value 0x%" TARGET_PRIxADDR, brp_i,
		brp_list[brp_i].control,
		brp_list[brp_i].value);
	return ERROR_OK;

}

static int aarch64_set_hybrid_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	int retval = ERROR_FAIL;
	int brp_1 = 0;	/* holds the contextID pair */
	int brp_2 = 0;	/* holds the IVA pair */
	uint32_t control_CTX, control_IVA;
	uint8_t CTX_byte_addr_select = 0x0F;
	uint8_t IVA_byte_addr_select = 0x0F;
	uint8_t CTX_machmode = 0x03;
	uint8_t IVA_machmode = 0x01;
	struct aarch64_common *aarch64 = target_to_aarch64(target);
	struct armv8_common *armv8 = &aarch64->armv8_common;
	struct aarch64_brp *brp_list = aarch64->brp_list;

	if (breakpoint->set) {
		LOG_WARNING("breakpoint already set");
		return retval;
	}
	/*check available context BRPs*/
	while ((brp_list[brp_1].used ||
		(brp_list[brp_1].type != BRP_CONTEXT)) && (brp_1 < aarch64->brp_num))
		brp_1++;

	printf("brp(CTX) found num: %d\n", brp_1);
	if (brp_1 >= aarch64->brp_num) {
		LOG_ERROR("ERROR Can not find free Breakpoint Register Pair");
		return ERROR_FAIL;
	}

	while ((brp_list[brp_2].used ||
		(brp_list[brp_2].type != BRP_NORMAL)) && (brp_2 < aarch64->brp_num))
		brp_2++;

	printf("brp(IVA) found num: %d\n", brp_2);
	if (brp_2 >= aarch64->brp_num) {
		LOG_ERROR("ERROR Can not find free Breakpoint Register Pair");
		return ERROR_FAIL;
	}

	breakpoint->set = brp_1 + 1;
	breakpoint->linked_BRP = brp_2;
	control_CTX = ((CTX_machmode & 0x7) << 20)
		| (brp_2 << 16)
		| (0 << 14)
		| (CTX_byte_addr_select << 5)
		| (3 << 1) | 1;
	brp_list[brp_1].used = 1;
	brp_list[brp_1].value = (breakpoint->asid);
	brp_list[brp_1].control = control_CTX;
	retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
			+ CPUV8_DBG_BVR_BASE + 16 * brp_list[brp_1].BRPn,
			brp_list[brp_1].value);
	if (retval != ERROR_OK)
		return retval;
	retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
			+ CPUV8_DBG_BCR_BASE + 16 * brp_list[brp_1].BRPn,
			brp_list[brp_1].control);
	if (retval != ERROR_OK)
		return retval;

	control_IVA = ((IVA_machmode & 0x7) << 20)
		| (brp_1 << 16)
		| (1 << 13)
		| (IVA_byte_addr_select << 5)
		| (3 << 1) | 1;
	brp_list[brp_2].used = 1;
	brp_list[brp_2].value = breakpoint->address & 0xFFFFFFFFFFFFFFFC;
	brp_list[brp_2].control = control_IVA;
	retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
			+ CPUV8_DBG_BVR_BASE + 16 * brp_list[brp_2].BRPn,
			brp_list[brp_2].value & 0xFFFFFFFF);
	if (retval != ERROR_OK)
		return retval;
	retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
			+ CPUV8_DBG_BVR_BASE + 4 + 16 * brp_list[brp_2].BRPn,
			brp_list[brp_2].value >> 32);
	if (retval != ERROR_OK)
		return retval;
	retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
			+ CPUV8_DBG_BCR_BASE + 16 * brp_list[brp_2].BRPn,
			brp_list[brp_2].control);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int aarch64_unset_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	int retval;
	struct aarch64_common *aarch64 = target_to_aarch64(target);
	struct armv8_common *armv8 = &aarch64->armv8_common;
	struct aarch64_brp *brp_list = aarch64->brp_list;

	if (!breakpoint->set) {
		LOG_WARNING("breakpoint not set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD) {
		if ((breakpoint->address != 0) && (breakpoint->asid != 0)) {
			int brp_i = breakpoint->set - 1;
			int brp_j = breakpoint->linked_BRP;
			if ((brp_i < 0) || (brp_i >= aarch64->brp_num)) {
				LOG_DEBUG("Invalid BRP number in breakpoint");
				return ERROR_OK;
			}
			LOG_DEBUG("rbp %i control 0x%0" PRIx32 " value 0x%" TARGET_PRIxADDR, brp_i,
				brp_list[brp_i].control, brp_list[brp_i].value);
			brp_list[brp_i].used = 0;
			brp_list[brp_i].value = 0;
			brp_list[brp_i].control = 0;
			retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
					+ CPUV8_DBG_BCR_BASE + 16 * brp_list[brp_i].BRPn,
					brp_list[brp_i].control);
			if (retval != ERROR_OK)
				return retval;
			retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
					+ CPUV8_DBG_BVR_BASE + 16 * brp_list[brp_i].BRPn,
					(uint32_t)brp_list[brp_i].value);
			if (retval != ERROR_OK)
				return retval;
			retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
					+ CPUV8_DBG_BVR_BASE + 4 + 16 * brp_list[brp_i].BRPn,
					(uint32_t)brp_list[brp_i].value);
			if (retval != ERROR_OK)
				return retval;
			if ((brp_j < 0) || (brp_j >= aarch64->brp_num)) {
				LOG_DEBUG("Invalid BRP number in breakpoint");
				return ERROR_OK;
			}
			LOG_DEBUG("rbp %i control 0x%0" PRIx32 " value 0x%0" PRIx64, brp_j,
				brp_list[brp_j].control, brp_list[brp_j].value);
			brp_list[brp_j].used = 0;
			brp_list[brp_j].value = 0;
			brp_list[brp_j].control = 0;
			retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
					+ CPUV8_DBG_BCR_BASE + 16 * brp_list[brp_j].BRPn,
					brp_list[brp_j].control);
			if (retval != ERROR_OK)
				return retval;
			retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
					+ CPUV8_DBG_BVR_BASE + 16 * brp_list[brp_j].BRPn,
					(uint32_t)brp_list[brp_j].value);
			if (retval != ERROR_OK)
				return retval;
			retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
					+ CPUV8_DBG_BVR_BASE + 4 + 16 * brp_list[brp_j].BRPn,
					(uint32_t)brp_list[brp_j].value);
			if (retval != ERROR_OK)
				return retval;

			breakpoint->linked_BRP = 0;
			breakpoint->set = 0;
			return ERROR_OK;

		} else {
			int brp_i = breakpoint->set - 1;
			if ((brp_i < 0) || (brp_i >= aarch64->brp_num)) {
				LOG_DEBUG("Invalid BRP number in breakpoint");
				return ERROR_OK;
			}
			LOG_DEBUG("rbp %i control 0x%0" PRIx32 " value 0x%0" PRIx64, brp_i,
				brp_list[brp_i].control, brp_list[brp_i].value);
			brp_list[brp_i].used = 0;
			brp_list[brp_i].value = 0;
			brp_list[brp_i].control = 0;
			retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
					+ CPUV8_DBG_BCR_BASE + 16 * brp_list[brp_i].BRPn,
					brp_list[brp_i].control);
			if (retval != ERROR_OK)
				return retval;
			retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
					+ CPUV8_DBG_BVR_BASE + 16 * brp_list[brp_i].BRPn,
					brp_list[brp_i].value);
			if (retval != ERROR_OK)
				return retval;

			retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
					+ CPUV8_DBG_BVR_BASE + 4 + 16 * brp_list[brp_i].BRPn,
					(uint32_t)brp_list[brp_i].value);
			if (retval != ERROR_OK)
				return retval;
			breakpoint->set = 0;
			return ERROR_OK;
		}
	} else {
		/* restore original instruction (kept in target endianness) */

		armv8_cache_d_inner_flush_virt(armv8,
				breakpoint->address & 0xFFFFFFFFFFFFFFFE,
				breakpoint->length);

		if (breakpoint->length == 4) {
			retval = target_write_memory(target,
					breakpoint->address & 0xFFFFFFFFFFFFFFFE,
					4, 1, breakpoint->orig_instr);
			if (retval != ERROR_OK)
				return retval;
		} else {
			retval = target_write_memory(target,
					breakpoint->address & 0xFFFFFFFFFFFFFFFE,
					2, 1, breakpoint->orig_instr);
			if (retval != ERROR_OK)
				return retval;
		}

		armv8_cache_d_inner_flush_virt(armv8,
				breakpoint->address & 0xFFFFFFFFFFFFFFFE,
				breakpoint->length);

		armv8_cache_i_inner_inval_virt(armv8,
				breakpoint->address & 0xFFFFFFFFFFFFFFFE,
				breakpoint->length);
	}
	breakpoint->set = 0;

	return ERROR_OK;
}

static int aarch64_add_breakpoint(struct target *target,
	struct breakpoint *breakpoint)
{
	struct aarch64_common *aarch64 = target_to_aarch64(target);

	if ((breakpoint->type == BKPT_HARD) && (aarch64->brp_num_available < 1)) {
		LOG_INFO("no hardware breakpoint available");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (breakpoint->type == BKPT_HARD)
		aarch64->brp_num_available--;

	return aarch64_set_breakpoint(target, breakpoint, 0x00);	/* Exact match */
}

static int aarch64_add_context_breakpoint(struct target *target,
	struct breakpoint *breakpoint)
{
	struct aarch64_common *aarch64 = target_to_aarch64(target);

	if ((breakpoint->type == BKPT_HARD) && (aarch64->brp_num_available < 1)) {
		LOG_INFO("no hardware breakpoint available");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (breakpoint->type == BKPT_HARD)
		aarch64->brp_num_available--;

	return aarch64_set_context_breakpoint(target, breakpoint, 0x02);	/* asid match */
}

static int aarch64_add_hybrid_breakpoint(struct target *target,
	struct breakpoint *breakpoint)
{
	struct aarch64_common *aarch64 = target_to_aarch64(target);

	if ((breakpoint->type == BKPT_HARD) && (aarch64->brp_num_available < 1)) {
		LOG_INFO("no hardware breakpoint available");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (breakpoint->type == BKPT_HARD)
		aarch64->brp_num_available--;

	return aarch64_set_hybrid_breakpoint(target, breakpoint);	/* ??? */
}


static int aarch64_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct aarch64_common *aarch64 = target_to_aarch64(target);

#if 0
/* It is perfectly possible to remove breakpoints while the target is running */
	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
#endif

	if (breakpoint->set) {
		aarch64_unset_breakpoint(target, breakpoint);
		if (breakpoint->type == BKPT_HARD)
			aarch64->brp_num_available++;
	}

	return ERROR_OK;
}

/*
 * Cortex-A8 Reset functions
 */

static int aarch64_assert_reset(struct target *target)
{
	struct armv8_common *armv8 = target_to_armv8(target);

	LOG_DEBUG(" ");

	/* FIXME when halt is requested, make it work somehow... */

	/* Issue some kind of warm reset. */
	if (target_has_event_action(target, TARGET_EVENT_RESET_ASSERT))
		target_handle_event(target, TARGET_EVENT_RESET_ASSERT);
	else if (jtag_get_reset_config() & RESET_HAS_SRST) {
		/* REVISIT handle "pulls" cases, if there's
		 * hardware that needs them to work.
		 */
		jtag_add_reset(0, 1);
	} else {
		LOG_ERROR("%s: how to reset?", target_name(target));
		return ERROR_FAIL;
	}

	/* registers are now invalid */
	if (target_was_examined(target)) {
		register_cache_invalidate(armv8->arm.core_cache);
		register_cache_invalidate(armv8->arm.core_cache->next);
	}

	target->state = TARGET_RESET;

	return ERROR_OK;
}

static int aarch64_deassert_reset(struct target *target)
{
	int retval;

	LOG_DEBUG(" ");

	/* be certain SRST is off */
	jtag_add_reset(0, 0);

	if (!target_was_examined(target))
		return ERROR_OK;

	retval = aarch64_poll(target);
	if (retval != ERROR_OK)
		return retval;

	if (target->reset_halt) {
		if (target->state != TARGET_HALTED) {
			LOG_WARNING("%s: ran after reset and before halt ...",
				target_name(target));
			retval = target_halt(target);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	return aarch64_init_debug_access(target);
}

static int aarch64_write_apb_ap_memory(struct target *target,
	uint64_t address, uint32_t size,
	uint32_t count, const uint8_t *buffer)
{
	/* write memory through APB-AP */
	int retval = ERROR_COMMAND_SYNTAX_ERROR;
	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm_dpm *dpm = &armv8->dpm;
	struct arm *arm = &armv8->arm;
	int total_bytes = count * size;
	int total_u32;
	int start_byte = address & 0x3;
	int end_byte   = (address + total_bytes) & 0x3;
	struct reg *reg;
	uint32_t dscr;
	uint8_t *tmp_buff = NULL;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	total_u32 = DIV_ROUND_UP((address & 3) + total_bytes, 4);

	/* Mark register R0 as dirty, as it will be used
	 * for transferring the data.
	 * It will be restored automatically when exiting
	 * debug mode
	 */
	reg = armv8_reg_current(arm, 1);
	reg->dirty = true;

	reg = armv8_reg_current(arm, 0);
	reg->dirty = true;

	/* This algorithm comes from DDI0487A.g, chapter J9.1 */

	/* The algorithm only copies 32 bit words, so the buffer
	 * should be expanded to include the words at either end.
	 * The first and last words will be read first to avoid
	 * corruption if needed.
	 */
	tmp_buff = malloc(total_u32 * 4);

	if ((start_byte != 0) && (total_u32 > 1)) {
		/* First bytes not aligned - read the 32 bit word to avoid corrupting
		 * the other bytes in the word.
		 */
		retval = aarch64_read_apb_ap_memory(target, (address & ~0x3), 4, 1, tmp_buff);
		if (retval != ERROR_OK)
			goto error_free_buff_w;
	}

	/* If end of write is not aligned, or the write is less than 4 bytes */
	if ((end_byte != 0) ||
		((total_u32 == 1) && (total_bytes != 4))) {

		/* Read the last word to avoid corruption during 32 bit write */
		int mem_offset = (total_u32-1) * 4;
		retval = aarch64_read_apb_ap_memory(target, (address & ~0x3) + mem_offset, 4, 1, &tmp_buff[mem_offset]);
		if (retval != ERROR_OK)
			goto error_free_buff_w;
	}

	/* Copy the write buffer over the top of the temporary buffer */
	memcpy(&tmp_buff[start_byte], buffer, total_bytes);

	/* We now have a 32 bit aligned buffer that can be written */

	/* Read DSCR */
	retval = mem_ap_read_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		goto error_free_buff_w;

	/* Set Normal access mode  */
	dscr = (dscr & ~DSCR_MA);
	retval = mem_ap_write_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_DSCR, dscr);

	if (arm->core_state == ARM_STATE_AARCH64) {
		/* Write X0 with value 'address' using write procedure */
		/* Step 1.a+b - Write the address for read access into DBGDTR_EL0 */
		/* Step 1.c   - Copy value from DTR to R0 using instruction mrs DBGDTR_EL0, x0 */
		retval = dpm->instr_write_data_dcc_64(dpm,
				ARMV8_MRS(SYSTEM_DBG_DBGDTR_EL0, 0), address & ~0x3ULL);
	} else {
		/* Write R0 with value 'address' using write procedure */
		/* Step 1.a+b - Write the address for read access into DBGDTRRX */
		/* Step 1.c   - Copy value from DTR to R0 using instruction mrc DBGDTRTXint, r0 */
		dpm->instr_write_data_dcc(dpm,
				ARMV4_5_MRC(14, 0, 0, 0, 5, 0), address & ~0x3ULL);

	}
	/* Step 1.d   - Change DCC to memory mode */
	dscr = dscr | DSCR_MA;
	retval +=  mem_ap_write_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_DSCR, dscr);
	if (retval != ERROR_OK)
		goto error_unset_dtr_w;


	/* Step 2.a   - Do the write */
	retval = mem_ap_write_buf_noincr(armv8->debug_ap,
					tmp_buff, 4, total_u32, armv8->debug_base + CPUV8_DBG_DTRRX);
	if (retval != ERROR_OK)
		goto error_unset_dtr_w;

	/* Step 3.a   - Switch DTR mode back to Normal mode */
	dscr = (dscr & ~DSCR_MA);
	retval = mem_ap_write_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_DSCR, dscr);
	if (retval != ERROR_OK)
		goto error_unset_dtr_w;

	/* Check for sticky abort flags in the DSCR */
	retval = mem_ap_read_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		goto error_free_buff_w;

	dpm->dscr = dscr;
	if (dscr & (DSCR_ERR | DSCR_SYS_ERROR_PEND)) {
		/* Abort occurred - clear it and exit */
		LOG_ERROR("abort occurred - dscr = 0x%08" PRIx32, dscr);
		armv8_dpm_handle_exception(dpm);
		goto error_free_buff_w;
	}

	/* Done */
	free(tmp_buff);
	return ERROR_OK;

error_unset_dtr_w:
	/* Unset DTR mode */
	mem_ap_read_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_DSCR, &dscr);
	dscr = (dscr & ~DSCR_MA);
	mem_ap_write_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_DSCR, dscr);
error_free_buff_w:
	LOG_ERROR("error");
	free(tmp_buff);
	return ERROR_FAIL;
}

static int aarch64_read_apb_ap_memory(struct target *target,
	target_addr_t address, uint32_t size,
	uint32_t count, uint8_t *buffer)
{
	/* read memory through APB-AP */
	int retval = ERROR_COMMAND_SYNTAX_ERROR;
	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm_dpm *dpm = &armv8->dpm;
	struct arm *arm = &armv8->arm;
	int total_bytes = count * size;
	int total_u32;
	int start_byte = address & 0x3;
	int end_byte   = (address + total_bytes) & 0x3;
	struct reg *reg;
	uint32_t dscr;
	uint8_t *tmp_buff = NULL;
	uint8_t *u8buf_ptr;
	uint32_t value;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	total_u32 = DIV_ROUND_UP((address & 3) + total_bytes, 4);
	/* Mark register X0, X1 as dirty, as it will be used
	 * for transferring the data.
	 * It will be restored automatically when exiting
	 * debug mode
	 */
	reg = armv8_reg_current(arm, 1);
	reg->dirty = true;

	reg = armv8_reg_current(arm, 0);
	reg->dirty = true;

	/* Read DSCR */
	retval = mem_ap_read_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_DSCR, &dscr);

	/* This algorithm comes from DDI0487A.g, chapter J9.1 */

	/* Set Normal access mode  */
	dscr = (dscr & ~DSCR_MA);
	retval +=  mem_ap_write_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_DSCR, dscr);

	if (arm->core_state == ARM_STATE_AARCH64) {
		/* Write X0 with value 'address' using write procedure */
		/* Step 1.a+b - Write the address for read access into DBGDTR_EL0 */
		/* Step 1.c   - Copy value from DTR to R0 using instruction mrs DBGDTR_EL0, x0 */
		retval += dpm->instr_write_data_dcc_64(dpm,
				ARMV8_MRS(SYSTEM_DBG_DBGDTR_EL0, 0), address & ~0x3ULL);
		/* Step 1.d - Dummy operation to ensure EDSCR.Txfull == 1 */
		retval += dpm->instr_execute(dpm, ARMV8_MSR_GP(SYSTEM_DBG_DBGDTR_EL0, 0));
		/* Step 1.e - Change DCC to memory mode */
		dscr = dscr | DSCR_MA;
		retval +=  mem_ap_write_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_DSCR, dscr);
		/* Step 1.f - read DBGDTRTX and discard the value */
		retval += mem_ap_read_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_DTRTX, &value);
	} else {
		/* Write R0 with value 'address' using write procedure */
		/* Step 1.a+b - Write the address for read access into DBGDTRRXint */
		/* Step 1.c   - Copy value from DTR to R0 using instruction mrc DBGDTRTXint, r0 */
		retval += dpm->instr_write_data_dcc(dpm,
				ARMV4_5_MRC(14, 0, 0, 0, 5, 0), address & ~0x3ULL);
		/* Step 1.d - Dummy operation to ensure EDSCR.Txfull == 1 */
		retval += dpm->instr_execute(dpm, ARMV4_5_MCR(14, 0, 0, 0, 5, 0));
		/* Step 1.e - Change DCC to memory mode */
		dscr = dscr | DSCR_MA;
		retval +=  mem_ap_write_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_DSCR, dscr);
		/* Step 1.f - read DBGDTRTX and discard the value */
		retval += mem_ap_read_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_DTRTX, &value);

	}
	if (retval != ERROR_OK)
		goto error_unset_dtr_r;

	/* Optimize the read as much as we can, either way we read in a single pass  */
	if ((start_byte) || (end_byte)) {
		/* The algorithm only copies 32 bit words, so the buffer
		 * should be expanded to include the words at either end.
		 * The first and last words will be read into a temp buffer
		 * to avoid corruption
		 */
		tmp_buff = malloc(total_u32 * 4);
		if (!tmp_buff)
			goto error_unset_dtr_r;

		/* use the tmp buffer to read the entire data */
		u8buf_ptr = tmp_buff;
	} else
		/* address and read length are aligned so read directly into the passed buffer */
		u8buf_ptr = buffer;

	/* Read the data - Each read of the DTRTX register causes the instruction to be reissued
	 * Abort flags are sticky, so can be read at end of transactions
	 *
	 * This data is read in aligned to 32 bit boundary.
	 */

	/* Step 2.a - Loop n-1 times, each read of DBGDTRTX reads the data from [X0] and
	 * increments X0 by 4. */
	retval = mem_ap_read_buf_noincr(armv8->debug_ap, u8buf_ptr, 4, total_u32-1,
									armv8->debug_base + CPUV8_DBG_DTRTX);
	if (retval != ERROR_OK)
			goto error_unset_dtr_r;

	/* Step 3.a - set DTR access mode back to Normal mode	*/
	dscr = (dscr & ~DSCR_MA);
	retval =  mem_ap_write_atomic_u32(armv8->debug_ap,
					armv8->debug_base + CPUV8_DBG_DSCR, dscr);
	if (retval != ERROR_OK)
		goto error_free_buff_r;

	/* Step 3.b - read DBGDTRTX for the final value */
	retval = mem_ap_read_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_DTRTX, &value);
	memcpy(u8buf_ptr + (total_u32-1) * 4, &value, 4);

	/* Check for sticky abort flags in the DSCR */
	retval = mem_ap_read_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		goto error_free_buff_r;

	dpm->dscr = dscr;

	if (dscr & (DSCR_ERR | DSCR_SYS_ERROR_PEND)) {
		/* Abort occurred - clear it and exit */
		LOG_ERROR("abort occurred - dscr = 0x%08" PRIx32, dscr);
		armv8_dpm_handle_exception(dpm);
		goto error_free_buff_r;
	}

	/* check if we need to copy aligned data by applying any shift necessary */
	if (tmp_buff) {
		memcpy(buffer, tmp_buff + start_byte, total_bytes);
		free(tmp_buff);
	}

	/* Done */
	return ERROR_OK;

error_unset_dtr_r:
	/* Unset DTR mode */
	mem_ap_read_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_DSCR, &dscr);
	dscr = (dscr & ~DSCR_MA);
	mem_ap_write_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_DSCR, dscr);
error_free_buff_r:
	LOG_ERROR("error");
	free(tmp_buff);
	return ERROR_FAIL;
}

static int aarch64_read_phys_memory(struct target *target,
	target_addr_t address, uint32_t size,
	uint32_t count, uint8_t *buffer)
{
	int retval = ERROR_COMMAND_SYNTAX_ERROR;

	if (count && buffer) {
		/* read memory through APB-AP */
		retval = aarch64_mmu_modify(target, 0);
		if (retval != ERROR_OK)
			return retval;
		retval = aarch64_read_apb_ap_memory(target, address, size, count, buffer);
	}
	return retval;
}

static int aarch64_read_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	int mmu_enabled = 0;
	int retval;

	/* determine if MMU was enabled on target stop */
	retval = aarch64_mmu(target, &mmu_enabled);
	if (retval != ERROR_OK)
		return retval;

	if (mmu_enabled) {
		/* enable MMU as we could have disabled it for phys access */
		retval = aarch64_mmu_modify(target, 1);
		if (retval != ERROR_OK)
			return retval;
	}
	return aarch64_read_apb_ap_memory(target, address, size, count, buffer);
}

static int aarch64_write_phys_memory(struct target *target,
	target_addr_t address, uint32_t size,
	uint32_t count, const uint8_t *buffer)
{
	int retval = ERROR_COMMAND_SYNTAX_ERROR;

	if (count && buffer) {
		/* write memory through APB-AP */
		retval = aarch64_mmu_modify(target, 0);
		if (retval != ERROR_OK)
			return retval;
		return aarch64_write_apb_ap_memory(target, address, size, count, buffer);
	}

	return retval;
}

static int aarch64_write_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer)
{
	int mmu_enabled = 0;
	int retval;

	/* determine if MMU was enabled on target stop */
	retval = aarch64_mmu(target, &mmu_enabled);
	if (retval != ERROR_OK)
		return retval;

	if (mmu_enabled) {
		/* enable MMU as we could have disabled it for phys access */
		retval = aarch64_mmu_modify(target, 1);
		if (retval != ERROR_OK)
			return retval;
	}
	return aarch64_write_apb_ap_memory(target, address, size, count, buffer);
}

static int aarch64_handle_target_request(void *priv)
{
	struct target *target = priv;
	struct armv8_common *armv8 = target_to_armv8(target);
	int retval;

	if (!target_was_examined(target))
		return ERROR_OK;
	if (!target->dbg_msg_enabled)
		return ERROR_OK;

	if (target->state == TARGET_RUNNING) {
		uint32_t request;
		uint32_t dscr;
		retval = mem_ap_read_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_DSCR, &dscr);

		/* check if we have data */
		while ((dscr & DSCR_DTR_TX_FULL) && (retval == ERROR_OK)) {
			retval = mem_ap_read_atomic_u32(armv8->debug_ap,
					armv8->debug_base + CPUV8_DBG_DTRTX, &request);
			if (retval == ERROR_OK) {
				target_request(target, request);
				retval = mem_ap_read_atomic_u32(armv8->debug_ap,
						armv8->debug_base + CPUV8_DBG_DSCR, &dscr);
			}
		}
	}

	return ERROR_OK;
}

static int aarch64_examine_first(struct target *target)
{
	struct aarch64_common *aarch64 = target_to_aarch64(target);
	struct armv8_common *armv8 = &aarch64->armv8_common;
	struct adiv5_dap *swjdp = armv8->arm.dap;
	uint32_t cti_base;
	int i;
	int retval = ERROR_OK;
	uint64_t debug, ttypr;
	uint32_t cpuid;
	uint32_t tmp0, tmp1;
	debug = ttypr = cpuid = 0;

	retval = dap_dp_init(swjdp);
	if (retval != ERROR_OK)
		return retval;

	/* Search for the APB-AB - it is needed for access to debug registers */
	retval = dap_find_ap(swjdp, AP_TYPE_APB_AP, &armv8->debug_ap);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not find APB-AP for debug access");
		return retval;
	}

	retval = mem_ap_init(armv8->debug_ap);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not initialize the APB-AP");
		return retval;
	}

	armv8->debug_ap->memaccess_tck = 10;

	if (!target->dbgbase_set) {
		uint32_t dbgbase;
		/* Get ROM Table base */
		uint32_t apid;
		int32_t coreidx = target->coreid;
		retval = dap_get_debugbase(armv8->debug_ap, &dbgbase, &apid);
		if (retval != ERROR_OK)
			return retval;
		/* Lookup 0x15 -- Processor DAP */
		retval = dap_lookup_cs_component(armv8->debug_ap, dbgbase, 0x15,
				&armv8->debug_base, &coreidx);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("Detected core %" PRId32 " dbgbase: %08" PRIx32
				" apid: %08" PRIx32, coreidx, armv8->debug_base, apid);
	} else
		armv8->debug_base = target->dbgbase;

	uint32_t prsr;
	int64_t then = timeval_ms();
	do {
		retval = mem_ap_read_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_PRSR, &prsr);
		if (retval == ERROR_OK) {
			retval = mem_ap_write_atomic_u32(armv8->debug_ap,
					armv8->debug_base + CPUV8_DBG_PRCR, PRCR_COREPURQ|PRCR_CORENPDRQ);
			if (retval != ERROR_OK) {
				LOG_DEBUG("write to PRCR failed");
				break;
			}
		}

		if (timeval_ms() > then + 1000) {
			retval = ERROR_TARGET_TIMEOUT;
			break;
		}

	} while ((prsr & PRSR_PU) == 0);

	if (retval != ERROR_OK) {
		LOG_ERROR("target %s: failed to set power state of the core.", target_name(target));
		return retval;
	}

	retval = mem_ap_write_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_OSLAR, 0);
	if (retval != ERROR_OK) {
		LOG_DEBUG("Examine %s failed", "oslock");
		return retval;
	}

	retval = mem_ap_read_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_MAINID0, &cpuid);
	if (retval != ERROR_OK) {
		LOG_DEBUG("Examine %s failed", "CPUID");
		return retval;
	}

	retval = mem_ap_read_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_MEMFEATURE0, &tmp0);
	retval += mem_ap_read_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_MEMFEATURE0 + 4, &tmp1);
	if (retval != ERROR_OK) {
		LOG_DEBUG("Examine %s failed", "Memory Model Type");
		return retval;
	}
	ttypr |= tmp1;
	ttypr = (ttypr << 32) | tmp0;

	retval = mem_ap_read_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_DBGFEATURE0, &tmp0);
	retval += mem_ap_read_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_DBGFEATURE0 + 4, &tmp1);
	if (retval != ERROR_OK) {
		LOG_DEBUG("Examine %s failed", "ID_AA64DFR0_EL1");
		return retval;
	}
	debug |= tmp1;
	debug = (debug << 32) | tmp0;

	LOG_DEBUG("cpuid = 0x%08" PRIx32, cpuid);
	LOG_DEBUG("ttypr = 0x%08" PRIx64, ttypr);
	LOG_DEBUG("debug = 0x%08" PRIx64, debug);

	if (target->ctibase == 0) {
		/* assume a v8 rom table layout */
		cti_base = armv8->debug_base + 0x10000;
		LOG_INFO("Target ctibase is not set, assuming 0x%0" PRIx32, cti_base);
	} else
		cti_base = target->ctibase;

	armv8->cti = arm_cti_create(armv8->debug_ap, cti_base);
	if (armv8->cti == NULL)
		return ERROR_FAIL;

	retval = aarch64_dpm_setup(aarch64, debug);
	if (retval != ERROR_OK)
		return retval;

	/* Setup Breakpoint Register Pairs */
	aarch64->brp_num = (uint32_t)((debug >> 12) & 0x0F) + 1;
	aarch64->brp_num_context = (uint32_t)((debug >> 28) & 0x0F) + 1;
	aarch64->brp_num_available = aarch64->brp_num;
	aarch64->brp_list = calloc(aarch64->brp_num, sizeof(struct aarch64_brp));
	for (i = 0; i < aarch64->brp_num; i++) {
		aarch64->brp_list[i].used = 0;
		if (i < (aarch64->brp_num-aarch64->brp_num_context))
			aarch64->brp_list[i].type = BRP_NORMAL;
		else
			aarch64->brp_list[i].type = BRP_CONTEXT;
		aarch64->brp_list[i].value = 0;
		aarch64->brp_list[i].control = 0;
		aarch64->brp_list[i].BRPn = i;
	}

	LOG_DEBUG("Configured %i hw breakpoints", aarch64->brp_num);

	target->state = TARGET_RUNNING;
	target->debug_reason = DBG_REASON_NOTHALTED;

	target_set_examined(target);
	return ERROR_OK;
}

static int aarch64_examine(struct target *target)
{
	int retval = ERROR_OK;

	/* don't re-probe hardware after each reset */
	if (!target_was_examined(target))
		retval = aarch64_examine_first(target);

	/* Configure core debug access */
	if (retval == ERROR_OK)
		retval = aarch64_init_debug_access(target);

	return retval;
}

/*
 *	Cortex-A8 target creation and initialization
 */

static int aarch64_init_target(struct command_context *cmd_ctx,
	struct target *target)
{
	/* examine_first() does a bunch of this */
	return ERROR_OK;
}

static int aarch64_init_arch_info(struct target *target,
	struct aarch64_common *aarch64, struct jtag_tap *tap)
{
	struct armv8_common *armv8 = &aarch64->armv8_common;

	/* Setup struct aarch64_common */
	aarch64->common_magic = AARCH64_COMMON_MAGIC;
	/*  tap has no dap initialized */
	if (!tap->dap) {
		tap->dap = dap_init();
		tap->dap->tap = tap;
	}
	armv8->arm.dap = tap->dap;

	/* register arch-specific functions */
	armv8->examine_debug_reason = NULL;
	armv8->post_debug_entry = aarch64_post_debug_entry;
	armv8->pre_restore_context = NULL;
	armv8->armv8_mmu.read_physical_memory = aarch64_read_phys_memory;

	armv8_init_arch_info(target, armv8);
	target_register_timer_callback(aarch64_handle_target_request, 1, 1, target);

	return ERROR_OK;
}

static int aarch64_target_create(struct target *target, Jim_Interp *interp)
{
	struct aarch64_common *aarch64 = calloc(1, sizeof(struct aarch64_common));

	return aarch64_init_arch_info(target, aarch64, target->tap);
}

static int aarch64_mmu(struct target *target, int *enabled)
{
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("%s: target %s not halted", __func__, target_name(target));
		return ERROR_TARGET_INVALID;
	}

	*enabled = target_to_aarch64(target)->armv8_common.armv8_mmu.mmu_enabled;
	return ERROR_OK;
}

static int aarch64_virt2phys(struct target *target, target_addr_t virt,
			     target_addr_t *phys)
{
	return armv8_mmu_translate_va_pa(target, virt, phys, 1);
}

COMMAND_HANDLER(aarch64_handle_cache_info_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct armv8_common *armv8 = target_to_armv8(target);

	return armv8_handle_cache_info_command(CMD_CTX,
			&armv8->armv8_mmu.armv8_cache);
}


COMMAND_HANDLER(aarch64_handle_dbginit_command)
{
	struct target *target = get_current_target(CMD_CTX);
	if (!target_was_examined(target)) {
		LOG_ERROR("target not examined yet");
		return ERROR_FAIL;
	}

	return aarch64_init_debug_access(target);
}
COMMAND_HANDLER(aarch64_handle_smp_off_command)
{
	struct target *target = get_current_target(CMD_CTX);
	/* check target is an smp target */
	struct target_list *head;
	struct target *curr;
	head = target->head;
	target->smp = 0;
	if (head != (struct target_list *)NULL) {
		while (head != (struct target_list *)NULL) {
			curr = head->target;
			curr->smp = 0;
			head = head->next;
		}
		/*  fixes the target display to the debugger */
		target->gdb_service->target = target;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(aarch64_handle_smp_on_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct target_list *head;
	struct target *curr;
	head = target->head;
	if (head != (struct target_list *)NULL) {
		target->smp = 1;
		while (head != (struct target_list *)NULL) {
			curr = head->target;
			curr->smp = 1;
			head = head->next;
		}
	}
	return ERROR_OK;
}

static const struct command_registration aarch64_exec_command_handlers[] = {
	{
		.name = "cache_info",
		.handler = aarch64_handle_cache_info_command,
		.mode = COMMAND_EXEC,
		.help = "display information about target caches",
		.usage = "",
	},
	{
		.name = "dbginit",
		.handler = aarch64_handle_dbginit_command,
		.mode = COMMAND_EXEC,
		.help = "Initialize core debug",
		.usage = "",
	},
	{	.name = "smp_off",
		.handler = aarch64_handle_smp_off_command,
		.mode = COMMAND_EXEC,
		.help = "Stop smp handling",
		.usage = "",
	},
	{
		.name = "smp_on",
		.handler = aarch64_handle_smp_on_command,
		.mode = COMMAND_EXEC,
		.help = "Restart smp handling",
		.usage = "",
	},

	COMMAND_REGISTRATION_DONE
};
static const struct command_registration aarch64_command_handlers[] = {
	{
		.chain = armv8_command_handlers,
	},
	{
		.name = "aarch64",
		.mode = COMMAND_ANY,
		.help = "Aarch64 command group",
		.usage = "",
		.chain = aarch64_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct target_type aarch64_target = {
	.name = "aarch64",

	.poll = aarch64_poll,
	.arch_state = armv8_arch_state,

	.halt = aarch64_halt,
	.resume = aarch64_resume,
	.step = aarch64_step,

	.assert_reset = aarch64_assert_reset,
	.deassert_reset = aarch64_deassert_reset,

	/* REVISIT allow exporting VFP3 registers ... */
	.get_gdb_reg_list = armv8_get_gdb_reg_list,

	.read_memory = aarch64_read_memory,
	.write_memory = aarch64_write_memory,

	.add_breakpoint = aarch64_add_breakpoint,
	.add_context_breakpoint = aarch64_add_context_breakpoint,
	.add_hybrid_breakpoint = aarch64_add_hybrid_breakpoint,
	.remove_breakpoint = aarch64_remove_breakpoint,
	.add_watchpoint = NULL,
	.remove_watchpoint = NULL,

	.commands = aarch64_command_handlers,
	.target_create = aarch64_target_create,
	.init_target = aarch64_init_target,
	.examine = aarch64_examine,

	.read_phys_memory = aarch64_read_phys_memory,
	.write_phys_memory = aarch64_write_phys_memory,
	.mmu = aarch64_mmu,
	.virt2phys = aarch64_virt2phys,
};
