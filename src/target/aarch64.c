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

static int aarch64_restore_system_control_reg(struct target *target)
{
	int retval = ERROR_OK;

	struct aarch64_common *aarch64 = target_to_aarch64(target);
	struct armv8_common *armv8 = target_to_armv8(target);

	if (aarch64->system_control_reg != aarch64->system_control_reg_curr) {
		aarch64->system_control_reg_curr = aarch64->system_control_reg;
		/* LOG_INFO("cp15_control_reg: %8.8" PRIx32, cortex_v8->cp15_control_reg); */

		switch (armv8->arm.core_mode) {
			case ARMV8_64_EL0T:
			case ARMV8_64_EL1T:
			case ARMV8_64_EL1H:
				retval = armv8->arm.msr(target, 3, /*op 0*/
						0, 1,	/* op1, op2 */
						0, 0,	/* CRn, CRm */
						aarch64->system_control_reg);
				if (retval != ERROR_OK)
					return retval;
			break;
			case ARMV8_64_EL2T:
			case ARMV8_64_EL2H:
				retval = armv8->arm.msr(target, 3, /*op 0*/
						4, 1,	/* op1, op2 */
						0, 0,	/* CRn, CRm */
						aarch64->system_control_reg);
				if (retval != ERROR_OK)
					return retval;
			break;
			case ARMV8_64_EL3H:
			case ARMV8_64_EL3T:
				retval = armv8->arm.msr(target, 3, /*op 0*/
						6, 1,	/* op1, op2 */
						0, 0,	/* CRn, CRm */
						aarch64->system_control_reg);
				if (retval != ERROR_OK)
					return retval;
			break;
			default:
				retval = armv8->arm.mcr(target, 15, 0, 0, 1, 0, aarch64->system_control_reg);
				if (retval != ERROR_OK)
					return retval;
				break;
			}
	}
	return retval;
}

/*  check address before aarch64_apb read write access with mmu on
 *  remove apb predictible data abort */
static int aarch64_check_address(struct target *target, uint32_t address)
{
	/* TODO */
	return ERROR_OK;
}
/*  modify system_control_reg in order to enable or disable mmu for :
 *  - virt2phys address conversion
 *  - read or write memory in phys or virt address */
static int aarch64_mmu_modify(struct target *target, int enable)
{
	struct aarch64_common *aarch64 = target_to_aarch64(target);
	struct armv8_common *armv8 = &aarch64->armv8_common;
	int retval = ERROR_OK;

	if (enable) {
		/*	if mmu enabled at target stop and mmu not enable */
		if (!(aarch64->system_control_reg & 0x1U)) {
			LOG_ERROR("trying to enable mmu on target stopped with mmu disable");
			return ERROR_FAIL;
		}
		if (!(aarch64->system_control_reg_curr & 0x1U)) {
			aarch64->system_control_reg_curr |= 0x1U;
			switch (armv8->arm.core_mode) {
				case ARMV8_64_EL0T:
				case ARMV8_64_EL1T:
				case ARMV8_64_EL1H:
					retval = armv8->arm.msr(target, 3, /*op 0*/
							0, 0,	/* op1, op2 */
							1, 0,	/* CRn, CRm */
							aarch64->system_control_reg_curr);
					if (retval != ERROR_OK)
						return retval;
				break;
				case ARMV8_64_EL2T:
				case ARMV8_64_EL2H:
					retval = armv8->arm.msr(target, 3, /*op 0*/
							4, 0,	/* op1, op2 */
							1, 0,	/* CRn, CRm */
							aarch64->system_control_reg_curr);
					if (retval != ERROR_OK)
						return retval;
				break;
				case ARMV8_64_EL3H:
				case ARMV8_64_EL3T:
					retval = armv8->arm.msr(target, 3, /*op 0*/
							6, 0,	/* op1, op2 */
							1, 0,	/* CRn, CRm */
							aarch64->system_control_reg_curr);
					if (retval != ERROR_OK)
						return retval;
				break;
				default:
					LOG_DEBUG("unknow cpu state 0x%x" PRIx32, armv8->arm.core_state);
			}
		}
	} else {
		if (aarch64->system_control_reg_curr & 0x4U) {
			/*  data cache is active */
			aarch64->system_control_reg_curr &= ~0x4U;
			/* flush data cache armv7 function to be called */
			if (armv8->armv8_mmu.armv8_cache.flush_all_data_cache)
				armv8->armv8_mmu.armv8_cache.flush_all_data_cache(target);
		}
		if ((aarch64->system_control_reg_curr & 0x1U)) {
			aarch64->system_control_reg_curr &= ~0x1U;
			switch (armv8->arm.core_mode) {
				case ARMV8_64_EL0T:
				case ARMV8_64_EL1T:
				case ARMV8_64_EL1H:
					retval = armv8->arm.msr(target, 3, /*op 0*/
							0, 0,	/* op1, op2 */
							1, 0,	/* CRn, CRm */
							aarch64->system_control_reg_curr);
					if (retval != ERROR_OK)
						return retval;
					break;
				case ARMV8_64_EL2T:
				case ARMV8_64_EL2H:
					retval = armv8->arm.msr(target, 3, /*op 0*/
							4, 0,	/* op1, op2 */
							1, 0,	/* CRn, CRm */
							aarch64->system_control_reg_curr);
					if (retval != ERROR_OK)
						return retval;
					break;
				case ARMV8_64_EL3H:
				case ARMV8_64_EL3T:
					retval = armv8->arm.msr(target, 3, /*op 0*/
							6, 0,	/* op1, op2 */
							1, 0,	/* CRn, CRm */
							aarch64->system_control_reg_curr);
					if (retval != ERROR_OK)
						return retval;
					break;
				default:
					LOG_DEBUG("unknow cpu state 0x%x" PRIx32, armv8->arm.core_state);
					break;
			}
		}
	}
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
	retval = mem_ap_write_atomic_u32(armv8->debug_ap,
			armv8->cti_base + CTI_CTR, 1);
	/* By default, gate all channel triggers to and from the CTM */
	if (retval == ERROR_OK)
		retval = mem_ap_write_atomic_u32(armv8->debug_ap,
				armv8->cti_base + CTI_GATE, 0);
	/* output halt requests to PE on channel 0 trigger */
	if (retval == ERROR_OK)
		retval = mem_ap_write_atomic_u32(armv8->debug_ap,
				armv8->cti_base + CTI_OUTEN0, CTI_CHNL(0));
	/* output restart requests to PE on channel 1 trigger */
	if (retval == ERROR_OK)
		retval = mem_ap_write_atomic_u32(armv8->debug_ap,
				armv8->cti_base + CTI_OUTEN1, CTI_CHNL(1));
	if (retval != ERROR_OK)
		return retval;

	/* Resync breakpoint registers */

	/* Since this is likely called from init or reset, update target state information*/
	return aarch64_poll(target);
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
	uint32_t dscr;

	/* Read DSCR */
	int retval = mem_ap_read_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_DSCR, &dscr);
	if (ERROR_OK != retval)
		return retval;

	/* clear bitfield */
	dscr &= ~bit_mask;
	/* put new value */
	dscr |= value & bit_mask;

	/* write new DSCR */
	retval = mem_ap_write_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_DSCR, dscr);
	return retval;
}

static struct target *get_aarch64(struct target *target, int32_t coreid)
{
	struct target_list *head;
	struct target *curr;

	head = target->head;
	while (head != (struct target_list *)NULL) {
		curr = head->target;
		if ((curr->coreid == coreid) && (curr->state == TARGET_HALTED))
			return curr;
		head = head->next;
	}
	return target;
}
static int aarch64_halt(struct target *target);

static int aarch64_halt_smp(struct target *target)
{
	int retval = ERROR_OK;
	struct target_list *head = target->head;

	while (head != (struct target_list *)NULL) {
		struct target *curr = head->target;
		struct armv8_common *armv8 = target_to_armv8(curr);

		/* open the gate for channel 0 to let HALT requests pass to the CTM */
		if (curr->smp) {
			retval = mem_ap_write_atomic_u32(armv8->debug_ap,
					armv8->cti_base + CTI_GATE, CTI_CHNL(0));
			if (retval == ERROR_OK)
				retval = aarch64_set_dscr_bits(curr, DSCR_HDE, DSCR_HDE);
		}
		if (retval != ERROR_OK)
			break;

		head = head->next;
	}

	/* halt the target PE */
	if (retval == ERROR_OK)
		retval = aarch64_halt(target);

	return retval;
}

static int update_halt_gdb(struct target *target)
{
	int retval = 0;
	if (target->gdb_service && target->gdb_service->core[0] == -1) {
		target->gdb_service->target = target;
		target->gdb_service->core[0] = target->coreid;
		retval += aarch64_halt_smp(target);
	}
	return retval;
}

/*
 * Cortex-A8 Run control
 */

static int aarch64_poll(struct target *target)
{
	int retval = ERROR_OK;
	uint32_t dscr;
	struct aarch64_common *aarch64 = target_to_aarch64(target);
	struct armv8_common *armv8 = &aarch64->armv8_common;
	enum target_state prev_target_state = target->state;
	/*  toggle to another core is done by gdb as follow */
	/*  maint packet J core_id */
	/*  continue */
	/*  the next polling trigger an halt event sent to gdb */
	if ((target->state == TARGET_HALTED) && (target->smp) &&
		(target->gdb_service) &&
		(target->gdb_service->target == NULL)) {
		target->gdb_service->target =
			get_aarch64(target, target->gdb_service->core[1]);
		target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		return retval;
	}
	retval = mem_ap_read_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		return retval;
	aarch64->cpudbg_dscr = dscr;

	if (DSCR_RUN_MODE(dscr) == 0x3) {
		if (prev_target_state != TARGET_HALTED) {
			/* We have a halting debug event */
			LOG_DEBUG("Target halted");
			target->state = TARGET_HALTED;
			if ((prev_target_state == TARGET_RUNNING)
				|| (prev_target_state == TARGET_UNKNOWN)
				|| (prev_target_state == TARGET_RESET)) {
				retval = aarch64_debug_entry(target);
				if (retval != ERROR_OK)
					return retval;
				if (target->smp) {
					retval = update_halt_gdb(target);
					if (retval != ERROR_OK)
						return retval;
				}
				target_call_event_callbacks(target,
					TARGET_EVENT_HALTED);
			}
			if (prev_target_state == TARGET_DEBUG_RUNNING) {
				LOG_DEBUG(" ");

				retval = aarch64_debug_entry(target);
				if (retval != ERROR_OK)
					return retval;
				if (target->smp) {
					retval = update_halt_gdb(target);
					if (retval != ERROR_OK)
						return retval;
				}

				target_call_event_callbacks(target,
					TARGET_EVENT_DEBUG_HALTED);
			}
		}
	} else
		target->state = TARGET_RUNNING;

	return retval;
}

static int aarch64_halt(struct target *target)
{
	int retval = ERROR_OK;
	uint32_t dscr;
	struct armv8_common *armv8 = target_to_armv8(target);

	/*
	 * add HDE in halting debug mode
	 */
	retval = aarch64_set_dscr_bits(target, DSCR_HDE, DSCR_HDE);
	if (retval != ERROR_OK)
		return retval;

	/* trigger an event on channel 0, this outputs a halt request to the PE */
	retval = mem_ap_write_atomic_u32(armv8->debug_ap,
			armv8->cti_base + CTI_APPPULSE, CTI_CHNL(0));
	if (retval != ERROR_OK)
		return retval;

	long long then = timeval_ms();
	for (;; ) {
		retval = mem_ap_read_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_DSCR, &dscr);
		if (retval != ERROR_OK)
			return retval;
		if ((dscr & DSCRV8_HALT_MASK) != 0)
			break;
		if (timeval_ms() > then + 1000) {
			LOG_ERROR("Timeout waiting for halt");
			return ERROR_FAIL;
		}
	}

	target->debug_reason = DBG_REASON_DBGRQ;

	return ERROR_OK;
}

static int aarch64_internal_restore(struct target *target, int current,
	uint64_t *address, int handle_breakpoints, int debug_execution)
{
	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm *arm = &armv8->arm;
	int retval;
	uint64_t resume_pc;

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

static int aarch64_internal_restart(struct target *target, bool slave_pe)
{
	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm *arm = &armv8->arm;
	int retval;
	uint32_t dscr;
	/*
	 * * Restart core and wait for it to be started.  Clear ITRen and sticky
	 * * exception flags: see ARMv7 ARM, C5.9.
	 *
	 * REVISIT: for single stepping, we probably want to
	 * disable IRQs by default, with optional override...
	 */

	retval = mem_ap_read_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		return retval;

	if ((dscr & DSCR_ITE) == 0)
		LOG_ERROR("DSCR InstrCompl must be set before leaving debug!");

	/* make sure to acknowledge the halt event before resuming */
	retval = mem_ap_write_atomic_u32(armv8->debug_ap,
			armv8->cti_base + CTI_INACK, CTI_TRIG(HALT));

	/*
	 * open the CTI gate for channel 1 so that the restart events
	 * get passed along to all PEs
	 */
	if (retval == ERROR_OK)
		retval = mem_ap_write_atomic_u32(armv8->debug_ap,
				armv8->cti_base + CTI_GATE, CTI_CHNL(1));
	if (retval != ERROR_OK)
		return retval;

	if (!slave_pe) {
		/* trigger an event on channel 1, generates a restart request to the PE */
		retval = mem_ap_write_atomic_u32(armv8->debug_ap,
				armv8->cti_base + CTI_APPPULSE, CTI_CHNL(1));
		if (retval != ERROR_OK)
			return retval;

		long long then = timeval_ms();
		for (;; ) {
			retval = mem_ap_read_atomic_u32(armv8->debug_ap,
					armv8->debug_base + CPUV8_DBG_DSCR, &dscr);
			if (retval != ERROR_OK)
				return retval;
			if ((dscr & DSCR_HDE) != 0)
				break;
			if (timeval_ms() > then + 1000) {
				LOG_ERROR("Timeout waiting for resume");
				return ERROR_FAIL;
			}
		}
	}

	target->debug_reason = DBG_REASON_NOTHALTED;
	target->state = TARGET_RUNNING;

	/* registers are now invalid */
	register_cache_invalidate(arm->core_cache);
	register_cache_invalidate(arm->core_cache->next);

	return ERROR_OK;
}

static int aarch64_restore_smp(struct target *target, int handle_breakpoints)
{
	int retval = 0;
	struct target_list *head;
	struct target *curr;
	uint64_t address;
	head = target->head;
	while (head != (struct target_list *)NULL) {
		curr = head->target;
		if ((curr != target) && (curr->state != TARGET_RUNNING)) {
			/*  resume current address , not in step mode */
			retval += aarch64_internal_restore(curr, 1, &address,
					handle_breakpoints, 0);
			retval += aarch64_internal_restart(curr, true);
		}
		head = head->next;

	}
	return retval;
}

static int aarch64_resume(struct target *target, int current,
	target_addr_t address, int handle_breakpoints, int debug_execution)
{
	int retval = 0;
	uint64_t addr = address;

	/* dummy resume for smp toggle in order to reduce gdb impact  */
	if ((target->smp) && (target->gdb_service->core[1] != -1)) {
		/*   simulate a start and halt of target */
		target->gdb_service->target = NULL;
		target->gdb_service->core[0] = target->gdb_service->core[1];
		/*  fake resume at next poll we play the  target core[1], see poll*/
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		return 0;
	}
	aarch64_internal_restore(target, current, &addr, handle_breakpoints,
				 debug_execution);
	if (target->smp) {
		target->gdb_service->core[0] = -1;
		retval = aarch64_restore_smp(target, handle_breakpoints);
		if (retval != ERROR_OK)
			return retval;
	}
	aarch64_internal_restart(target, false);

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
	struct aarch64_common *aarch64 = target_to_aarch64(target);
	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm_dpm *dpm = &armv8->dpm;
	enum arm_state core_state;

	LOG_DEBUG("%s dscr = 0x%08" PRIx32, target_name(target), aarch64->cpudbg_dscr);

	dpm->dscr = aarch64->cpudbg_dscr;
	core_state = armv8_dpm_get_core_state(dpm);
	armv8_select_opcodes(armv8, core_state == ARM_STATE_AARCH64);
	armv8_select_reg_access(armv8, core_state == ARM_STATE_AARCH64);

	/* make sure to clear all sticky errors */
	retval = mem_ap_write_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_DRCR, DRCR_CSE);

	/* discard async exceptions */
	if (retval == ERROR_OK)
		retval = dpm->instr_cpsr_sync(dpm);

	if (retval != ERROR_OK)
		return retval;

	/* Examine debug reason */
	armv8_dpm_report_dscr(dpm, aarch64->cpudbg_dscr);

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

	/* clear sticky errors */
	mem_ap_write_atomic_u32(armv8->debug_ap,
				    armv8->debug_base + CPUV8_DBG_DRCR, DRCR_CSE);

	switch (armv8->arm.core_mode) {
		case ARMV8_64_EL0T:
			armv8_dpm_modeswitch(&armv8->dpm, ARMV8_64_EL1H);
			/* fall through */
		case ARMV8_64_EL1T:
		case ARMV8_64_EL1H:
			retval = armv8->arm.mrs(target, 3, /*op 0*/
					0, 0,	/* op1, op2 */
					1, 0,	/* CRn, CRm */
					&aarch64->system_control_reg);
			if (retval != ERROR_OK)
				return retval;
		break;
		case ARMV8_64_EL2T:
		case ARMV8_64_EL2H:
			retval = armv8->arm.mrs(target, 3, /*op 0*/
					4, 0,	/* op1, op2 */
					1, 0,	/* CRn, CRm */
					&aarch64->system_control_reg);
			if (retval != ERROR_OK)
				return retval;
		break;
		case ARMV8_64_EL3H:
		case ARMV8_64_EL3T:
			retval = armv8->arm.mrs(target, 3, /*op 0*/
					6, 0,	/* op1, op2 */
					1, 0,	/* CRn, CRm */
					&aarch64->system_control_reg);
			if (retval != ERROR_OK)
				return retval;
		break;

		case ARM_MODE_SVC:
			retval = armv8->arm.mrc(target, 15, 0, 0, 1, 0, &aarch64->system_control_reg);
			if (retval != ERROR_OK)
				return retval;
			break;

		default:
			LOG_INFO("cannot read system control register in this mode");
			break;
	}

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
	aarch64->curr_mode = armv8->arm.core_mode;
	return ERROR_OK;
}

static int aarch64_step(struct target *target, int current, target_addr_t address,
	int handle_breakpoints)
{
	struct armv8_common *armv8 = target_to_armv8(target);
	int retval;
	uint32_t edecr;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = mem_ap_read_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_EDECR, &edecr);
	if (retval != ERROR_OK)
		return retval;

	/* make sure EDECR.SS is not set when restoring the register */
	edecr &= ~0x4;

	/* set EDECR.SS to enter hardware step mode */
	retval = mem_ap_write_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_EDECR, (edecr|0x4));
	if (retval != ERROR_OK)
		return retval;

	/* disable interrupts while stepping */
	retval = aarch64_set_dscr_bits(target, 0x3 << 22, 0x3 << 22);
	if (retval != ERROR_OK)
		return ERROR_OK;

	/* resume the target */
	retval = aarch64_resume(target, current, address, 0, 0);
	if (retval != ERROR_OK)
		return retval;

	long long then = timeval_ms();
	while (target->state != TARGET_HALTED) {
		retval = aarch64_poll(target);
		if (retval != ERROR_OK)
			return retval;
		if (timeval_ms() > then + 1000) {
			LOG_ERROR("timeout waiting for target halt");
			return ERROR_FAIL;
		}
	}

	/* restore EDECR */
	retval = mem_ap_write_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_EDECR, edecr);
	if (retval != ERROR_OK)
		return retval;

	/* restore interrupts */
	retval = aarch64_set_dscr_bits(target, 0x3 << 22, 0);
	if (retval != ERROR_OK)
		return ERROR_OK;

	return ERROR_OK;
}

static int aarch64_restore_context(struct target *target, bool bpwp)
{
	struct armv8_common *armv8 = target_to_armv8(target);

	LOG_DEBUG(" ");

	if (armv8->pre_restore_context)
		armv8->pre_restore_context(target);

	return armv8_dpm_write_dirty_registers(&armv8->dpm, bpwp);

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

		buf_set_u32(code, 0, 32, ARMV8_HLT(0x11));
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
	register_cache_invalidate(armv8->arm.core_cache);

	target->state = TARGET_RESET;

	return ERROR_OK;
}

static int aarch64_deassert_reset(struct target *target)
{
	int retval;

	LOG_DEBUG(" ");

	/* be certain SRST is off */
	jtag_add_reset(0, 0);

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

	return ERROR_OK;
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

	LOG_DEBUG("Writing APB-AP memory address 0x%" PRIx64 " size %"  PRIu32 " count%"  PRIu32,
			  address, size, count);
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

	/*  clear any abort  */
	retval = mem_ap_write_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_DRCR, DRCR_CSE);
	if (retval != ERROR_OK)
		return retval;


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
		mem_ap_write_atomic_u32(armv8->debug_ap,
					armv8->debug_base + CPUV8_DBG_DRCR, 1<<2);
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

	LOG_DEBUG("Reading APB-AP memory address 0x%" TARGET_PRIxADDR " size %"	PRIu32 " count%"  PRIu32,
			  address, size, count);
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

	/*	clear any abort  */
	retval = mem_ap_write_atomic_u32(armv8->debug_ap,
				armv8->debug_base + CPUV8_DBG_DRCR, DRCR_CSE);
	if (retval != ERROR_OK)
		goto error_free_buff_r;

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
		mem_ap_write_atomic_u32(armv8->debug_ap,
					armv8->debug_base + CPUV8_DBG_DRCR, DRCR_CSE);
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
	LOG_DEBUG("Reading memory at real address 0x%" TARGET_PRIxADDR "; size %" PRId32 "; count %" PRId32,
		address, size, count);

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

	/* aarch64 handles unaligned memory access */
	LOG_DEBUG("Reading memory at address 0x%" TARGET_PRIxADDR "; size %" PRId32 "; count %" PRId32, address,
		size, count);

	/* determine if MMU was enabled on target stop */
	retval = aarch64_mmu(target, &mmu_enabled);
	if (retval != ERROR_OK)
		return retval;

	if (mmu_enabled) {
		retval = aarch64_check_address(target, address);
		if (retval != ERROR_OK)
			return retval;
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

	LOG_DEBUG("Writing memory to real address 0x%" TARGET_PRIxADDR "; size %" PRId32 "; count %" PRId32, address,
		size, count);

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

	/* aarch64 handles unaligned memory access */
	LOG_DEBUG("Writing memory at address 0x%" TARGET_PRIxADDR "; size %" PRId32
		  "; count %" PRId32, address, size, count);

	/* determine if MMU was enabled on target stop */
	retval = aarch64_mmu(target, &mmu_enabled);
	if (retval != ERROR_OK)
		return retval;

	if (mmu_enabled) {
		retval = aarch64_check_address(target, address);
		if (retval != ERROR_OK)
			return retval;
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
	int i;
	int retval = ERROR_OK;
	uint64_t debug, ttypr;
	uint32_t cpuid;
	uint32_t tmp0, tmp1;
	debug = ttypr = cpuid = 0;

	/* We do one extra read to ensure DAP is configured,
	 * we call ahbap_debugport_init(swjdp) instead
	 */
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

	armv8->debug_ap->memaccess_tck = 80;

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

	retval = mem_ap_write_atomic_u32(armv8->debug_ap,
			armv8->debug_base + CPUV8_DBG_LOCKACCESS, 0xC5ACCE55);
	if (retval != ERROR_OK) {
		LOG_DEBUG("LOCK debug access fail");
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
		armv8->cti_base = target->ctibase = armv8->debug_base + 0x10000;
		LOG_INFO("Target ctibase is not set, assuming 0x%0" PRIx32, target->ctibase);
	} else
		armv8->cti_base = target->ctibase;

	armv8->arm.core_type = ARM_MODE_MON;
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
	struct adiv5_dap *dap = armv8->arm.dap;

	armv8->arm.dap = dap;

	/* Setup struct aarch64_common */
	aarch64->common_magic = AARCH64_COMMON_MAGIC;
	/*  tap has no dap initialized */
	if (!tap->dap) {
		tap->dap = dap_init();

		/* Leave (only) generic DAP stuff for debugport_init() */
		tap->dap->tap = tap;
	}

	armv8->arm.dap = tap->dap;

	aarch64->fast_reg_read = 0;

	/* register arch-specific functions */
	armv8->examine_debug_reason = NULL;

	armv8->post_debug_entry = aarch64_post_debug_entry;

	armv8->pre_restore_context = NULL;

	armv8->armv8_mmu.read_physical_memory = aarch64_read_phys_memory;

	/* REVISIT v7a setup should be in a v7a-specific routine */
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
		LOG_ERROR("%s: target not halted", __func__);
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

COMMAND_HANDLER(aarch64_handle_smp_gdb_command)
{
	struct target *target = get_current_target(CMD_CTX);
	int retval = ERROR_OK;
	struct target_list *head;
	head = target->head;
	if (head != (struct target_list *)NULL) {
		if (CMD_ARGC == 1) {
			int coreid = 0;
			COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], coreid);
			if (ERROR_OK != retval)
				return retval;
			target->gdb_service->core[1] = coreid;

		}
		command_print(CMD_CTX, "gdb coreid  %" PRId32 " -> %" PRId32, target->gdb_service->core[0]
			, target->gdb_service->core[1]);
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
	{
		.name = "smp_gdb",
		.handler = aarch64_handle_smp_gdb_command,
		.mode = COMMAND_EXEC,
		.help = "display/fix current core played to gdb",
		.usage = "",
	},


	COMMAND_REGISTRATION_DONE
};
static const struct command_registration aarch64_command_handlers[] = {
	{
		.chain = arm_command_handlers,
	},
	{
		.chain = armv8_command_handlers,
	},
	{
		.name = "cortex_a",
		.mode = COMMAND_ANY,
		.help = "Cortex-A command group",
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

	.checksum_memory = arm_checksum_memory,
	.blank_check_memory = arm_blank_check_memory,

	.run_algorithm = armv4_5_run_algorithm,

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
