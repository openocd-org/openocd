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
#include "arm_opcodes.h"
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
	target_ulong virt, target_ulong *phys);
static int aarch64_read_apb_ab_memory(struct target *target,
	uint64_t address, uint32_t size, uint32_t count, uint8_t *buffer);
static int aarch64_instr_write_data_r0(struct arm_dpm *dpm,
	uint32_t opcode, uint32_t data);

static int aarch64_restore_system_control_reg(struct target *target)
{
	int retval = ERROR_OK;

	struct aarch64_common *aarch64 = target_to_aarch64(target);
	struct armv8_common *armv8 = target_to_armv8(target);

	if (aarch64->system_control_reg != aarch64->system_control_reg_curr) {
		aarch64->system_control_reg_curr = aarch64->system_control_reg;
		retval = aarch64_instr_write_data_r0(armv8->arm.dpm,
						     0xd5181000,
						     aarch64->system_control_reg);
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
		/*  if mmu enabled at target stop and mmu not enable */
		if (!(aarch64->system_control_reg & 0x1U)) {
			LOG_ERROR("trying to enable mmu on target stopped with mmu disable");
			return ERROR_FAIL;
		}
		if (!(aarch64->system_control_reg_curr & 0x1U)) {
			aarch64->system_control_reg_curr |= 0x1U;
			retval = aarch64_instr_write_data_r0(armv8->arm.dpm,
							     0xd5181000,
							     aarch64->system_control_reg_curr);
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
			retval = aarch64_instr_write_data_r0(armv8->arm.dpm,
							     0xd5181000,
							     aarch64->system_control_reg_curr);
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
	struct adiv5_dap *swjdp = armv8->arm.dap;
	int retval;
	uint32_t dummy;

	LOG_DEBUG(" ");

	/* Unlocking the debug registers for modification
	 * The debugport might be uninitialised so try twice */
	retval = mem_ap_sel_write_atomic_u32(swjdp, armv8->debug_ap,
			     armv8->debug_base + CPUDBG_LOCKACCESS, 0xC5ACCE55);
	if (retval != ERROR_OK) {
		/* try again */
		retval = mem_ap_sel_write_atomic_u32(swjdp, armv8->debug_ap,
			     armv8->debug_base + CPUDBG_LOCKACCESS, 0xC5ACCE55);
		if (retval == ERROR_OK)
			LOG_USER("Locking debug access failed on first, but succeeded on second try.");
	}
	if (retval != ERROR_OK)
		return retval;
	/* Clear Sticky Power Down status Bit in PRSR to enable access to
	   the registers in the Core Power Domain */
	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + CPUDBG_PRSR, &dummy);
	if (retval != ERROR_OK)
		return retval;

	/* Enabling of instruction execution in debug mode is done in debug_entry code */

	/* Resync breakpoint registers */

	/* Since this is likely called from init or reset, update target state information*/
	return aarch64_poll(target);
}

/* To reduce needless round-trips, pass in a pointer to the current
 * DSCR value.  Initialize it to zero if you just need to know the
 * value on return from this function; or DSCR_INSTR_COMP if you
 * happen to know that no instruction is pending.
 */
static int aarch64_exec_opcode(struct target *target,
	uint32_t opcode, uint32_t *dscr_p)
{
	uint32_t dscr;
	int retval;
	struct armv8_common *armv8 = target_to_armv8(target);
	struct adiv5_dap *swjdp = armv8->arm.dap;

	dscr = dscr_p ? *dscr_p : 0;

	LOG_DEBUG("exec opcode 0x%08" PRIx32, opcode);

	/* Wait for InstrCompl bit to be set */
	long long then = timeval_ms();
	while ((dscr & DSCR_INSTR_COMP) == 0) {
		retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
				armv8->debug_base + CPUDBG_DSCR, &dscr);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not read DSCR register, opcode = 0x%08" PRIx32, opcode);
			return retval;
		}
		if (timeval_ms() > then + 1000) {
			LOG_ERROR("Timeout waiting for aarch64_exec_opcode");
			return ERROR_FAIL;
		}
	}

	retval = mem_ap_sel_write_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + CPUDBG_ITR, opcode);
	if (retval != ERROR_OK)
		return retval;

	then = timeval_ms();
	do {
		retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
				armv8->debug_base + CPUDBG_DSCR, &dscr);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not read DSCR register");
			return retval;
		}
		if (timeval_ms() > then + 1000) {
			LOG_ERROR("Timeout waiting for aarch64_exec_opcode");
			return ERROR_FAIL;
		}
	} while ((dscr & DSCR_INSTR_COMP) == 0);	/* Wait for InstrCompl bit to be set */

	if (dscr_p)
		*dscr_p = dscr;

	return retval;
}

/* Write to memory mapped registers directly with no cache or mmu handling */
static int aarch64_dap_write_memap_register_u32(struct target *target,
	uint32_t address,
	uint32_t value)
{
	int retval;
	struct armv8_common *armv8 = target_to_armv8(target);
	struct adiv5_dap *swjdp = armv8->arm.dap;

	retval = mem_ap_sel_write_atomic_u32(swjdp, armv8->debug_ap, address, value);

	return retval;
}

/*
 * AARCH64 implementation of Debug Programmer's Model
 *
 * NOTE the invariant:  these routines return with DSCR_INSTR_COMP set,
 * so there's no need to poll for it before executing an instruction.
 *
 * NOTE that in several of these cases the "stall" mode might be useful.
 * It'd let us queue a few operations together... prepare/finish might
 * be the places to enable/disable that mode.
 */

static inline struct aarch64_common *dpm_to_a8(struct arm_dpm *dpm)
{
	return container_of(dpm, struct aarch64_common, armv8_common.dpm);
}

static int aarch64_write_dcc(struct aarch64_common *a8, uint32_t data)
{
	LOG_DEBUG("write DCC 0x%08" PRIx32, data);
	return mem_ap_sel_write_u32(a8->armv8_common.arm.dap,
		a8->armv8_common.debug_ap, a8->armv8_common.debug_base + CPUDBG_DTRRX, data);
}

static int aarch64_write_dcc_64(struct aarch64_common *a8, uint64_t data)
{
	int ret;
	LOG_DEBUG("write DCC 0x%08" PRIx32, (unsigned)data);
	LOG_DEBUG("write DCC 0x%08" PRIx32, (unsigned)(data >> 32));
	ret = mem_ap_sel_write_u32(a8->armv8_common.arm.dap,
		a8->armv8_common.debug_ap, a8->armv8_common.debug_base + CPUDBG_DTRRX, data);
	ret += mem_ap_sel_write_u32(a8->armv8_common.arm.dap,
		a8->armv8_common.debug_ap, a8->armv8_common.debug_base + CPUDBG_DTRTX, data >> 32);
	return ret;
}

static int aarch64_read_dcc(struct aarch64_common *a8, uint32_t *data,
	uint32_t *dscr_p)
{
	struct adiv5_dap *swjdp = a8->armv8_common.arm.dap;
	uint32_t dscr = DSCR_INSTR_COMP;
	int retval;

	if (dscr_p)
		dscr = *dscr_p;

	/* Wait for DTRRXfull */
	long long then = timeval_ms();
	while ((dscr & DSCR_DTR_TX_FULL) == 0) {
		retval = mem_ap_sel_read_atomic_u32(swjdp, a8->armv8_common.debug_ap,
				a8->armv8_common.debug_base + CPUDBG_DSCR,
				&dscr);
		if (retval != ERROR_OK)
			return retval;
		if (timeval_ms() > then + 1000) {
			LOG_ERROR("Timeout waiting for read dcc");
			return ERROR_FAIL;
		}
	}

	retval = mem_ap_sel_read_atomic_u32(swjdp, a8->armv8_common.debug_ap,
					    a8->armv8_common.debug_base + CPUDBG_DTRTX,
					    data);
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("read DCC 0x%08" PRIx32, *data);

	if (dscr_p)
		*dscr_p = dscr;

	return retval;
}
static int aarch64_read_dcc_64(struct aarch64_common *a8, uint64_t *data,
	uint32_t *dscr_p)
{
	struct adiv5_dap *swjdp = a8->armv8_common.arm.dap;
	uint32_t dscr = DSCR_INSTR_COMP;
	uint32_t higher;
	int retval;

	if (dscr_p)
		dscr = *dscr_p;

	/* Wait for DTRRXfull */
	long long then = timeval_ms();
	while ((dscr & DSCR_DTR_TX_FULL) == 0) {
		retval = mem_ap_sel_read_atomic_u32(swjdp, a8->armv8_common.debug_ap,
				a8->armv8_common.debug_base + CPUDBG_DSCR,
				&dscr);
		if (retval != ERROR_OK)
			return retval;
		if (timeval_ms() > then + 1000) {
			LOG_ERROR("Timeout waiting for read dcc");
			return ERROR_FAIL;
		}
	}

	retval = mem_ap_sel_read_atomic_u32(swjdp, a8->armv8_common.debug_ap,
					    a8->armv8_common.debug_base + CPUDBG_DTRTX,
					    (uint32_t *)data);
	if (retval != ERROR_OK)
		return retval;

	retval = mem_ap_sel_read_atomic_u32(swjdp, a8->armv8_common.debug_ap,
					    a8->armv8_common.debug_base + CPUDBG_DTRRX,
					    &higher);
	if (retval != ERROR_OK)
		return retval;

	*data = *(uint32_t *)data | (uint64_t)higher << 32;
	LOG_DEBUG("read DCC 0x%16.16" PRIx64, *data);

	if (dscr_p)
		*dscr_p = dscr;

	return retval;
}

static int aarch64_dpm_prepare(struct arm_dpm *dpm)
{
	struct aarch64_common *a8 = dpm_to_a8(dpm);
	struct adiv5_dap *swjdp = a8->armv8_common.arm.dap;
	uint32_t dscr;
	int retval;

	/* set up invariant:  INSTR_COMP is set after ever DPM operation */
	long long then = timeval_ms();
	for (;; ) {
		retval = mem_ap_sel_read_atomic_u32(swjdp, a8->armv8_common.debug_ap,
				a8->armv8_common.debug_base + CPUDBG_DSCR,
				&dscr);
		if (retval != ERROR_OK)
			return retval;
		if ((dscr & DSCR_INSTR_COMP) != 0)
			break;
		if (timeval_ms() > then + 1000) {
			LOG_ERROR("Timeout waiting for dpm prepare");
			return ERROR_FAIL;
		}
	}

	/* this "should never happen" ... */
	if (dscr & DSCR_DTR_RX_FULL) {
		LOG_ERROR("DSCR_DTR_RX_FULL, dscr 0x%08" PRIx32, dscr);
		/* Clear DCCRX */
		retval = aarch64_exec_opcode(
				a8->armv8_common.arm.target,
				0xd5130400,
				&dscr);
		if (retval != ERROR_OK)
			return retval;
	}

	return retval;
}

static int aarch64_dpm_finish(struct arm_dpm *dpm)
{
	/* REVISIT what could be done here? */
	return ERROR_OK;
}

static int aarch64_instr_write_data_dcc(struct arm_dpm *dpm,
	uint32_t opcode, uint32_t data)
{
	struct aarch64_common *a8 = dpm_to_a8(dpm);
	int retval;
	uint32_t dscr = DSCR_INSTR_COMP;

	retval = aarch64_write_dcc(a8, data);
	if (retval != ERROR_OK)
		return retval;

	return aarch64_exec_opcode(
			a8->armv8_common.arm.target,
			opcode,
			&dscr);
}

static int aarch64_instr_write_data_dcc_64(struct arm_dpm *dpm,
	uint32_t opcode, uint64_t data)
{
	struct aarch64_common *a8 = dpm_to_a8(dpm);
	int retval;
	uint32_t dscr = DSCR_INSTR_COMP;

	retval = aarch64_write_dcc_64(a8, data);
	if (retval != ERROR_OK)
		return retval;

	return aarch64_exec_opcode(
			a8->armv8_common.arm.target,
			opcode,
			&dscr);
}

static int aarch64_instr_write_data_r0(struct arm_dpm *dpm,
	uint32_t opcode, uint32_t data)
{
	struct aarch64_common *a8 = dpm_to_a8(dpm);
	uint32_t dscr = DSCR_INSTR_COMP;
	int retval;

	retval = aarch64_write_dcc(a8, data);
	if (retval != ERROR_OK)
		return retval;

	retval = aarch64_exec_opcode(
			a8->armv8_common.arm.target,
			0xd5330500,
			&dscr);
	if (retval != ERROR_OK)
		return retval;

	/* then the opcode, taking data from R0 */
	retval = aarch64_exec_opcode(
			a8->armv8_common.arm.target,
			opcode,
			&dscr);

	return retval;
}

static int aarch64_instr_write_data_r0_64(struct arm_dpm *dpm,
	uint32_t opcode, uint64_t data)
{
	struct aarch64_common *a8 = dpm_to_a8(dpm);
	uint32_t dscr = DSCR_INSTR_COMP;
	int retval;

	retval = aarch64_write_dcc_64(a8, data);
	if (retval != ERROR_OK)
		return retval;

	retval = aarch64_exec_opcode(
			a8->armv8_common.arm.target,
			0xd5330400,
			&dscr);
	if (retval != ERROR_OK)
		return retval;

	/* then the opcode, taking data from R0 */
	retval = aarch64_exec_opcode(
			a8->armv8_common.arm.target,
			opcode,
			&dscr);

	return retval;
}

static int aarch64_instr_cpsr_sync(struct arm_dpm *dpm)
{
	struct target *target = dpm->arm->target;
	uint32_t dscr = DSCR_INSTR_COMP;

	/* "Prefetch flush" after modifying execution status in CPSR */
	return aarch64_exec_opcode(target,
			ARMV4_5_MCR(15, 0, 0, 7, 5, 4),
			&dscr);
}

static int aarch64_instr_read_data_dcc(struct arm_dpm *dpm,
	uint32_t opcode, uint32_t *data)
{
	struct aarch64_common *a8 = dpm_to_a8(dpm);
	int retval;
	uint32_t dscr = DSCR_INSTR_COMP;

	/* the opcode, writing data to DCC */
	retval = aarch64_exec_opcode(
			a8->armv8_common.arm.target,
			opcode,
			&dscr);
	if (retval != ERROR_OK)
		return retval;

	return aarch64_read_dcc(a8, data, &dscr);
}

static int aarch64_instr_read_data_dcc_64(struct arm_dpm *dpm,
	uint32_t opcode, uint64_t *data)
{
	struct aarch64_common *a8 = dpm_to_a8(dpm);
	int retval;
	uint32_t dscr = DSCR_INSTR_COMP;

	/* the opcode, writing data to DCC */
	retval = aarch64_exec_opcode(
			a8->armv8_common.arm.target,
			opcode,
			&dscr);
	if (retval != ERROR_OK)
		return retval;

	return aarch64_read_dcc_64(a8, data, &dscr);
}

static int aarch64_instr_read_data_r0(struct arm_dpm *dpm,
	uint32_t opcode, uint32_t *data)
{
	struct aarch64_common *a8 = dpm_to_a8(dpm);
	uint32_t dscr = DSCR_INSTR_COMP;
	int retval;

	/* the opcode, writing data to R0 */
	retval = aarch64_exec_opcode(
			a8->armv8_common.arm.target,
			opcode,
			&dscr);
	if (retval != ERROR_OK)
		return retval;

	/* write R0 to DCC */
	retval = aarch64_exec_opcode(
			a8->armv8_common.arm.target,
			0xd5130400,  /* msr dbgdtr_el0, x0 */
			&dscr);
	if (retval != ERROR_OK)
		return retval;

	return aarch64_read_dcc(a8, data, &dscr);
}

static int aarch64_instr_read_data_r0_64(struct arm_dpm *dpm,
	uint32_t opcode, uint64_t *data)
{
	struct aarch64_common *a8 = dpm_to_a8(dpm);
	uint32_t dscr = DSCR_INSTR_COMP;
	int retval;

	/* the opcode, writing data to R0 */
	retval = aarch64_exec_opcode(
			a8->armv8_common.arm.target,
			opcode,
			&dscr);
	if (retval != ERROR_OK)
		return retval;

	/* write R0 to DCC */
	retval = aarch64_exec_opcode(
			a8->armv8_common.arm.target,
			0xd5130400,  /* msr dbgdtr_el0, x0 */
			&dscr);
	if (retval != ERROR_OK)
		return retval;

	return aarch64_read_dcc_64(a8, data, &dscr);
}

static int aarch64_bpwp_enable(struct arm_dpm *dpm, unsigned index_t,
	uint32_t addr, uint32_t control)
{
	struct aarch64_common *a8 = dpm_to_a8(dpm);
	uint32_t vr = a8->armv8_common.debug_base;
	uint32_t cr = a8->armv8_common.debug_base;
	int retval;

	switch (index_t) {
		case 0 ... 15:	/* breakpoints */
			vr += CPUDBG_BVR_BASE;
			cr += CPUDBG_BCR_BASE;
			break;
		case 16 ... 31:	/* watchpoints */
			vr += CPUDBG_WVR_BASE;
			cr += CPUDBG_WCR_BASE;
			index_t -= 16;
			break;
		default:
			return ERROR_FAIL;
	}
	vr += 4 * index_t;
	cr += 4 * index_t;

	LOG_DEBUG("A8: bpwp enable, vr %08x cr %08x",
		(unsigned) vr, (unsigned) cr);

	retval = aarch64_dap_write_memap_register_u32(dpm->arm->target,
			vr, addr);
	if (retval != ERROR_OK)
		return retval;
	retval = aarch64_dap_write_memap_register_u32(dpm->arm->target,
			cr, control);
	return retval;
}

static int aarch64_bpwp_disable(struct arm_dpm *dpm, unsigned index_t)
{
	return ERROR_OK;

#if 0
	struct aarch64_common *a8 = dpm_to_a8(dpm);
	uint32_t cr;

	switch (index_t) {
		case 0 ... 15:
			cr = a8->armv8_common.debug_base + CPUDBG_BCR_BASE;
			break;
		case 16 ... 31:
			cr = a8->armv8_common.debug_base + CPUDBG_WCR_BASE;
			index_t -= 16;
			break;
		default:
			return ERROR_FAIL;
	}
	cr += 4 * index_t;

	LOG_DEBUG("A8: bpwp disable, cr %08x", (unsigned) cr);

	/* clear control register */
	return aarch64_dap_write_memap_register_u32(dpm->arm->target, cr, 0);
#endif
}

static int aarch64_dpm_setup(struct aarch64_common *a8, uint32_t debug)
{
	struct arm_dpm *dpm = &a8->armv8_common.dpm;
	int retval;

	dpm->arm = &a8->armv8_common.arm;
	dpm->didr = debug;

	dpm->prepare = aarch64_dpm_prepare;
	dpm->finish = aarch64_dpm_finish;

	dpm->instr_write_data_dcc = aarch64_instr_write_data_dcc;
	dpm->instr_write_data_dcc_64 = aarch64_instr_write_data_dcc_64;
	dpm->instr_write_data_r0 = aarch64_instr_write_data_r0;
	dpm->instr_write_data_r0_64 = aarch64_instr_write_data_r0_64;
	dpm->instr_cpsr_sync = aarch64_instr_cpsr_sync;

	dpm->instr_read_data_dcc = aarch64_instr_read_data_dcc;
	dpm->instr_read_data_dcc_64 = aarch64_instr_read_data_dcc_64;
	dpm->instr_read_data_r0 = aarch64_instr_read_data_r0;
	dpm->instr_read_data_r0_64 = aarch64_instr_read_data_r0_64;

	dpm->arm_reg_current = armv8_reg_current;

	dpm->bpwp_enable = aarch64_bpwp_enable;
	dpm->bpwp_disable = aarch64_bpwp_disable;

	retval = arm_dpm_setup(dpm);
	if (retval == ERROR_OK)
		retval = arm_dpm_initialize(dpm);

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
	int retval = 0;
	struct target_list *head;
	struct target *curr;
	head = target->head;
	while (head != (struct target_list *)NULL) {
		curr = head->target;
		if ((curr != target) && (curr->state != TARGET_HALTED))
			retval += aarch64_halt(curr);
		head = head->next;
	}
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
	struct adiv5_dap *swjdp = armv8->arm.dap;
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
	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + CPUDBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		return retval;
	aarch64->cpudbg_dscr = dscr;

	if (DSCR_RUN_MODE(dscr) == (DSCR_CORE_HALTED | DSCR_CORE_RESTARTED)) {
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
	} else if (DSCR_RUN_MODE(dscr) == DSCR_CORE_RESTARTED)
		target->state = TARGET_RUNNING;
	else {
		LOG_DEBUG("Unknown target state dscr = 0x%08" PRIx32, dscr);
		target->state = TARGET_UNKNOWN;
	}

	return retval;
}

static int aarch64_halt(struct target *target)
{
	int retval = ERROR_OK;
	uint32_t dscr;
	struct armv8_common *armv8 = target_to_armv8(target);
	struct adiv5_dap *swjdp = armv8->arm.dap;

	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + 0x10000 + 0, &dscr);
	retval = mem_ap_sel_write_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + 0x10000 + 0, 1);
	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + 0x10000 + 0, &dscr);

	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + 0x10000 + 0x140, &dscr);
	retval = mem_ap_sel_write_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + 0x10000 + 0x140, 6);
	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + 0x10000 + 0x140, &dscr);

	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + 0x10000 + 0xa0, &dscr);
	retval = mem_ap_sel_write_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + 0x10000 + 0xa0, 5);
	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + 0x10000 + 0xa0, &dscr);

	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + 0x10000 + 0xa4, &dscr);
	retval = mem_ap_sel_write_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + 0x10000 + 0xa4, 2);
	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + 0x10000 + 0xa4, &dscr);

	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + 0x10000 + 0x20, &dscr);
	retval = mem_ap_sel_write_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + 0x10000 + 0x20, 4);
	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + 0x10000 + 0x20, &dscr);

	/*
	 * enter halting debug mode
	 */
	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + CPUDBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		return retval;

#	/* STATUS */
	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + 0x10000 + 0x134, &dscr);

	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + 0x10000 + 0x1c, &dscr);
	retval = mem_ap_sel_write_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + 0x10000 + 0x1c, 1);
	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + 0x10000 + 0x1c, &dscr);


	long long then = timeval_ms();
	for (;; ) {
		retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
				armv8->debug_base + CPUDBG_DSCR, &dscr);
		if (retval != ERROR_OK)
			return retval;
		if ((dscr & DSCR_CORE_HALTED) != 0)
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
	LOG_DEBUG("resume pc = 0x%16" PRIx64, resume_pc);
	buf_set_u64(arm->pc->value, 0, 64, resume_pc);
	arm->pc->dirty = 1;
	arm->pc->valid = 1;
#if 0
	/* restore dpm_mode at system halt */
	dpm_modeswitch(&armv8->dpm, ARM_MODE_ANY);
#endif
	/* called it now before restoring context because it uses cpu
	 * register r0 for restoring system control register */
	retval = aarch64_restore_system_control_reg(target);
	if (retval != ERROR_OK)
		return retval;
	retval = aarch64_restore_context(target, handle_breakpoints);
	if (retval != ERROR_OK)
		return retval;
	target->debug_reason = DBG_REASON_NOTHALTED;
	target->state = TARGET_RUNNING;

	/* registers are now invalid */
	register_cache_invalidate(arm->core_cache);

#if 0
	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		/* Single step past breakpoint at current address */
		breakpoint = breakpoint_find(target, resume_pc);
		if (breakpoint) {
			LOG_DEBUG("unset breakpoint at 0x%8.8x", breakpoint->address);
			cortex_m3_unset_breakpoint(target, breakpoint);
			cortex_m3_single_step_core(target);
			cortex_m3_set_breakpoint(target, breakpoint);
		}
	}
#endif

	return retval;
}

static int aarch64_internal_restart(struct target *target)
{
	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm *arm = &armv8->arm;
	struct adiv5_dap *swjdp = arm->dap;
	int retval;
	uint32_t dscr;
	/*
	 * * Restart core and wait for it to be started.  Clear ITRen and sticky
	 * * exception flags: see ARMv7 ARM, C5.9.
	 *
	 * REVISIT: for single stepping, we probably want to
	 * disable IRQs by default, with optional override...
	 */

	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + CPUDBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		return retval;

	if ((dscr & DSCR_INSTR_COMP) == 0)
		LOG_ERROR("DSCR InstrCompl must be set before leaving debug!");

	retval = mem_ap_sel_write_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + CPUDBG_DSCR, dscr & ~DSCR_ITR_EN);
	if (retval != ERROR_OK)
		return retval;

	retval = mem_ap_sel_write_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + CPUDBG_DRCR, DRCR_RESTART |
			DRCR_CLEAR_EXCEPTIONS);
	if (retval != ERROR_OK)
		return retval;

	retval = mem_ap_sel_write_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + 0x10000 + 0x10, 1);
	if (retval != ERROR_OK)
		return retval;

	retval = mem_ap_sel_write_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + 0x10000 + 0x1c, 2);
	if (retval != ERROR_OK)
		return retval;

	long long then = timeval_ms();
	for (;; ) {
		retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
				armv8->debug_base + CPUDBG_DSCR, &dscr);
		if (retval != ERROR_OK)
			return retval;
		if ((dscr & DSCR_CORE_RESTARTED) != 0)
			break;
		if (timeval_ms() > then + 1000) {
			LOG_ERROR("Timeout waiting for resume");
			return ERROR_FAIL;
		}
	}

	target->debug_reason = DBG_REASON_NOTHALTED;
	target->state = TARGET_RUNNING;

	/* registers are now invalid */
	register_cache_invalidate(arm->core_cache);

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
			retval += aarch64_internal_restart(curr);
		}
		head = head->next;

	}
	return retval;
}

static int aarch64_resume(struct target *target, int current,
	target_ulong address, int handle_breakpoints, int debug_execution)
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
	aarch64_internal_restart(target);

	if (!debug_execution) {
		target->state = TARGET_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		LOG_DEBUG("target resumed at 0x%" PRIu64, addr);
	} else {
		target->state = TARGET_DEBUG_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED);
		LOG_DEBUG("target debug resumed at 0x%" PRIu64, addr);
	}

	return ERROR_OK;
}

static int aarch64_debug_entry(struct target *target)
{
	uint32_t dscr;
	int retval = ERROR_OK;
	struct aarch64_common *aarch64 = target_to_aarch64(target);
	struct armv8_common *armv8 = target_to_armv8(target);
	struct adiv5_dap *swjdp = armv8->arm.dap;

	LOG_DEBUG("dscr = 0x%08" PRIx32, aarch64->cpudbg_dscr);

	/* REVISIT surely we should not re-read DSCR !! */
	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + CPUDBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		return retval;

	/* REVISIT see A8 TRM 12.11.4 steps 2..3 -- make sure that any
	 * imprecise data aborts get discarded by issuing a Data
	 * Synchronization Barrier:  ARMV4_5_MCR(15, 0, 0, 7, 10, 4).
	 */

	/* Enable the ITR execution once we are in debug mode */
	dscr |= DSCR_ITR_EN;
	retval = mem_ap_sel_write_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + CPUDBG_DSCR, dscr);
	if (retval != ERROR_OK)
		return retval;

	/* Examine debug reason */
	arm_dpm_report_dscr(&armv8->dpm, aarch64->cpudbg_dscr);

	/* save address of instruction that triggered the watchpoint? */
	if (target->debug_reason == DBG_REASON_WATCHPOINT) {
		uint32_t wfar;

		retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
				armv8->debug_base + CPUDBG_WFAR,
				&wfar);
		if (retval != ERROR_OK)
			return retval;
		arm_dpm_report_wfar(&armv8->dpm, wfar);
	}

	retval = arm_dpm_read_current_registers_64(&armv8->dpm);

	if (armv8->post_debug_entry) {
		retval = armv8->post_debug_entry(target);
		if (retval != ERROR_OK)
			return retval;
	}

	return retval;
}

static int aarch64_post_debug_entry(struct target *target)
{
	struct aarch64_common *aarch64 = target_to_aarch64(target);
	struct armv8_common *armv8 = &aarch64->armv8_common;
	struct armv8_mmu_common *armv8_mmu = &armv8->armv8_mmu;
	uint32_t sctlr_el1 = 0;
	int retval;

	mem_ap_sel_write_atomic_u32(armv8->arm.dap, armv8->debug_ap,
				    armv8->debug_base + CPUDBG_DRCR, 1<<2);
	retval = aarch64_instr_read_data_r0(armv8->arm.dpm,
					    0xd5381000, &sctlr_el1);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("sctlr_el1 = %#8.8x", sctlr_el1);
	aarch64->system_control_reg = sctlr_el1;
	aarch64->system_control_reg_curr = sctlr_el1;
	aarch64->curr_mode = armv8->arm.core_mode;

	armv8_mmu->mmu_enabled = sctlr_el1 & 0x1U ? 1 : 0;
	armv8_mmu->armv8_cache.d_u_cache_enabled = sctlr_el1 & 0x4U ? 1 : 0;
	armv8_mmu->armv8_cache.i_cache_enabled = sctlr_el1 & 0x1000U ? 1 : 0;

#if 0
	if (armv8->armv8_mmu.armv8_cache.ctype == -1)
		armv8_identify_cache(target);
#endif

	return ERROR_OK;
}

static int aarch64_step(struct target *target, int current, target_ulong address,
	int handle_breakpoints)
{
	struct armv8_common *armv8 = target_to_armv8(target);
	struct adiv5_dap *swjdp = armv8->arm.dap;
	int retval;
	uint32_t tmp;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + CPUDBG_DECR, &tmp);
	if (retval != ERROR_OK)
		return retval;

	retval = mem_ap_sel_write_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + CPUDBG_DECR, (tmp|0x4));
	if (retval != ERROR_OK)
		return retval;

	retval = aarch64_resume(target, 1, address, 0, 0);
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

	target->debug_reason = DBG_REASON_BREAKPOINT;
	retval = mem_ap_sel_write_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + CPUDBG_DECR, (tmp&(~0x4)));
	if (retval != ERROR_OK)
		return retval;

	if (target->state != TARGET_HALTED)
		LOG_DEBUG("target stepped");

	return ERROR_OK;
}

static int aarch64_restore_context(struct target *target, bool bpwp)
{
	struct armv8_common *armv8 = target_to_armv8(target);

	LOG_DEBUG(" ");

	if (armv8->pre_restore_context)
		armv8->pre_restore_context(target);

	return arm_dpm_write_dirty_registers(&armv8->dpm, bpwp);

	return ERROR_OK;
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
	uint32_t dscr;

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
				+ CPUDBG_BVR_BASE + 16 * brp_list[brp_i].BRPn,
				(uint32_t)(bpt_value & 0xFFFFFFFF));
		if (retval != ERROR_OK)
			return retval;
		retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
				+ CPUDBG_BVR_BASE + 4 + 16 * brp_list[brp_i].BRPn,
				(uint32_t)(bpt_value >> 32));
		if (retval != ERROR_OK)
			return retval;

		retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
				+ CPUDBG_BCR_BASE + 16 * brp_list[brp_i].BRPn,
				brp_list[brp_i].control);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("brp %i control 0x%0" PRIx32 " value 0x%" PRIXX, brp_i,
			brp_list[brp_i].control,
			brp_list[brp_i].value);

	} else if (breakpoint->type == BKPT_SOFT) {
		uint8_t code[4];
		buf_set_u32(code, 0, 32, 0xD4400000);

		retval = target_read_memory(target,
				breakpoint->address & 0xFFFFFFFFFFFFFFFE,
				breakpoint->length, 1,
				breakpoint->orig_instr);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_memory(target,
				breakpoint->address & 0xFFFFFFFFFFFFFFFE,
				breakpoint->length, 1, code);
		if (retval != ERROR_OK)
			return retval;
		breakpoint->set = 0x11;	/* Any nice value but 0 */
	}

	retval = mem_ap_sel_read_atomic_u32(armv8->arm.dap, armv8->debug_ap,
					    armv8->debug_base + CPUDBG_DSCR, &dscr);
	/* Ensure that halting debug mode is enable */
	dscr = dscr | DSCR_HALT_DBG_MODE;
	retval = mem_ap_sel_write_atomic_u32(armv8->arm.dap, armv8->debug_ap,
					     armv8->debug_base + CPUDBG_DSCR, dscr);
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
		| (byte_addr_select << 5)
		| (3 << 1) | 1;
	brp_list[brp_i].used = 1;
	brp_list[brp_i].value = (breakpoint->asid);
	brp_list[brp_i].control = control;
	retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
			+ CPUDBG_BVR_BASE + 4 * brp_list[brp_i].BRPn,
			brp_list[brp_i].value);
	if (retval != ERROR_OK)
		return retval;
	retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
			+ CPUDBG_BCR_BASE + 4 * brp_list[brp_i].BRPn,
			brp_list[brp_i].control);
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("brp %i control 0x%0" PRIx32 " value 0x%" PRIXX, brp_i,
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
			+ CPUDBG_BVR_BASE + 4 * brp_list[brp_1].BRPn,
			brp_list[brp_1].value);
	if (retval != ERROR_OK)
		return retval;
	retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
			+ CPUDBG_BCR_BASE + 4 * brp_list[brp_1].BRPn,
			brp_list[brp_1].control);
	if (retval != ERROR_OK)
		return retval;

	control_IVA = ((IVA_machmode & 0x7) << 20)
		| (brp_1 << 16)
		| (IVA_byte_addr_select << 5)
		| (3 << 1) | 1;
	brp_list[brp_2].used = 1;
	brp_list[brp_2].value = (breakpoint->address & 0xFFFFFFFC);
	brp_list[brp_2].control = control_IVA;
	retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
			+ CPUDBG_BVR_BASE + 4 * brp_list[brp_2].BRPn,
			brp_list[brp_2].value);
	if (retval != ERROR_OK)
		return retval;
	retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
			+ CPUDBG_BCR_BASE + 4 * brp_list[brp_2].BRPn,
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
			LOG_DEBUG("rbp %i control 0x%0" PRIx32 " value 0x%" PRIXX, brp_i,
				brp_list[brp_i].control, brp_list[brp_i].value);
			brp_list[brp_i].used = 0;
			brp_list[brp_i].value = 0;
			brp_list[brp_i].control = 0;
			retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
					+ CPUDBG_BCR_BASE + 16 * brp_list[brp_i].BRPn,
					brp_list[brp_i].control);
			if (retval != ERROR_OK)
				return retval;
			if ((brp_j < 0) || (brp_j >= aarch64->brp_num)) {
				LOG_DEBUG("Invalid BRP number in breakpoint");
				return ERROR_OK;
			}
			LOG_DEBUG("rbp %i control 0x%0" PRIx32 " value 0x%" PRIXX, brp_j,
				brp_list[brp_j].control, brp_list[brp_j].value);
			brp_list[brp_j].used = 0;
			brp_list[brp_j].value = 0;
			brp_list[brp_j].control = 0;
			retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
					+ CPUDBG_BCR_BASE + 16 * brp_list[brp_j].BRPn,
					brp_list[brp_j].control);
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
			LOG_DEBUG("rbp %i control 0x%0" PRIx32 " value 0x%" PRIXX, brp_i,
				brp_list[brp_i].control, brp_list[brp_i].value);
			brp_list[brp_i].used = 0;
			brp_list[brp_i].value = 0;
			brp_list[brp_i].control = 0;
			retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
					+ CPUDBG_BCR_BASE + 4 * brp_list[brp_i].BRPn,
					brp_list[brp_i].control);
			if (retval != ERROR_OK)
				return retval;
			retval = aarch64_dap_write_memap_register_u32(target, armv8->debug_base
					+ CPUDBG_BVR_BASE + 4 * brp_list[brp_i].BRPn,
					brp_list[brp_i].value);
			if (retval != ERROR_OK)
				return retval;
			breakpoint->set = 0;
			return ERROR_OK;
		}
	} else {
		/* restore original instruction (kept in target endianness) */
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

static int aarch64_write_apb_ab_memory(struct target *target,
	uint64_t address, uint32_t size,
	uint32_t count, const uint8_t *buffer)
{
	/* write memory through APB-AP */
	int retval = ERROR_COMMAND_SYNTAX_ERROR;
	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm *arm = &armv8->arm;
	struct adiv5_dap *swjdp = armv8->arm.dap;
	int total_bytes = count * size;
	int total_u32;
	int start_byte = address & 0x3;
	int end_byte   = (address + total_bytes) & 0x3;
	struct reg *reg;
	uint32_t dscr;
	uint8_t *tmp_buff = NULL;
	uint32_t i = 0;

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
	retval = mem_ap_sel_write_atomic_u32(swjdp, armv8->debug_ap, armv8->debug_base + CPUDBG_DRCR, 1<<2);
	if (retval != ERROR_OK)
		return retval;

	/* This algorithm comes from either :
	 * Cortex-A8 TRM Example 12-25
	 * Cortex-R4 TRM Example 11-26
	 * (slight differences)
	 */

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
		retval = aarch64_read_apb_ab_memory(target, (address & ~0x3), 4, 1, tmp_buff);
		if (retval != ERROR_OK)
			goto error_free_buff_w;
	}

	/* If end of write is not aligned, or the write is less than 4 bytes */
	if ((end_byte != 0) ||
		((total_u32 == 1) && (total_bytes != 4))) {

		/* Read the last word to avoid corruption during 32 bit write */
		int mem_offset = (total_u32-1) * 4;
		retval = aarch64_read_apb_ab_memory(target, (address & ~0x3) + mem_offset, 4, 1, &tmp_buff[mem_offset]);
		if (retval != ERROR_OK)
			goto error_free_buff_w;
	}

	/* Copy the write buffer over the top of the temporary buffer */
	memcpy(&tmp_buff[start_byte], buffer, total_bytes);

	/* We now have a 32 bit aligned buffer that can be written */

	/* Read DSCR */
	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + CPUDBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		goto error_free_buff_w;

	/* Set DTR mode to Normal*/
	dscr = (dscr & ~DSCR_EXT_DCC_MASK) | DSCR_EXT_DCC_NON_BLOCKING;
	retval = mem_ap_sel_write_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + CPUDBG_DSCR, dscr);
	if (retval != ERROR_OK)
		goto error_free_buff_w;

	if (size > 4) {
		LOG_WARNING("reading size >4 bytes not yet supported");
		goto error_unset_dtr_w;
	}

	retval = aarch64_instr_write_data_dcc_64(arm->dpm, 0xd5330401, address+4);
	if (retval != ERROR_OK)
		goto error_unset_dtr_w;

	dscr = DSCR_INSTR_COMP;
	while (i < count * size) {
		uint32_t val;

		memcpy(&val, &buffer[i], size);
		retval = aarch64_instr_write_data_dcc(arm->dpm, 0xd5330500, val);
		if (retval != ERROR_OK)
			goto error_unset_dtr_w;

		retval = aarch64_exec_opcode(target, 0xb81fc020, &dscr);
		if (retval != ERROR_OK)
			goto error_unset_dtr_w;

		retval = aarch64_exec_opcode(target, 0x91001021, &dscr);
		if (retval != ERROR_OK)
			goto error_unset_dtr_w;

		i += 4;
	}

	/* Check for sticky abort flags in the DSCR */
	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
				armv8->debug_base + CPUDBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		goto error_free_buff_w;
	if (dscr & (DSCR_STICKY_ABORT_PRECISE | DSCR_STICKY_ABORT_IMPRECISE)) {
		/* Abort occurred - clear it and exit */
		LOG_ERROR("abort occurred - dscr = 0x%08" PRIx32, dscr);
		mem_ap_sel_write_atomic_u32(swjdp, armv8->debug_ap,
					armv8->debug_base + CPUDBG_DRCR, 1<<2);
		goto error_free_buff_w;
	}

	/* Done */
	free(tmp_buff);
	return ERROR_OK;

error_unset_dtr_w:
	/* Unset DTR mode */
	mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
				armv8->debug_base + CPUDBG_DSCR, &dscr);
	dscr = (dscr & ~DSCR_EXT_DCC_MASK) | DSCR_EXT_DCC_NON_BLOCKING;
	mem_ap_sel_write_atomic_u32(swjdp, armv8->debug_ap,
				armv8->debug_base + CPUDBG_DSCR, dscr);
error_free_buff_w:
	LOG_ERROR("error");
	free(tmp_buff);
	return ERROR_FAIL;
}

static int aarch64_read_apb_ab_memory(struct target *target,
	uint64_t address, uint32_t size,
	uint32_t count, uint8_t *buffer)
{
	/* read memory through APB-AP */

	int retval = ERROR_COMMAND_SYNTAX_ERROR;
	struct armv8_common *armv8 = target_to_armv8(target);
	struct adiv5_dap *swjdp = armv8->arm.dap;
	struct arm *arm = &armv8->arm;
	struct reg *reg;
	uint32_t dscr, val;
	uint8_t *tmp_buff = NULL;
	uint32_t i = 0;

	LOG_DEBUG("Reading APB-AP memory address 0x%" PRIx64 " size %"  PRIu32 " count%"  PRIu32,
			  address, size, count);
	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Mark register R0 as dirty, as it will be used
	 * for transferring the data.
	 * It will be restored automatically when exiting
	 * debug mode
	 */
	reg = armv8_reg_current(arm, 0);
	reg->dirty = true;

	/*  clear any abort  */
	retval = mem_ap_sel_write_atomic_u32(swjdp, armv8->debug_ap,
		armv8->debug_base + CPUDBG_DRCR, 1<<2);
	if (retval != ERROR_OK)
		goto error_free_buff_r;

	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + CPUDBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		goto error_unset_dtr_r;

	if (size > 4) {
		LOG_WARNING("reading size >4 bytes not yet supported");
		goto error_unset_dtr_r;
	}

	while (i < count * size) {

		retval = aarch64_instr_write_data_dcc_64(arm->dpm, 0xd5330400, address+4);
		if (retval != ERROR_OK)
			goto error_unset_dtr_r;
		retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + CPUDBG_DSCR, &dscr);

		dscr = DSCR_INSTR_COMP;
		retval = aarch64_exec_opcode(target, 0xb85fc000, &dscr);
		if (retval != ERROR_OK)
			goto error_unset_dtr_r;
		retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + CPUDBG_DSCR, &dscr);

		retval = aarch64_instr_read_data_dcc(arm->dpm, 0xd5130400, &val);
		if (retval != ERROR_OK)
			goto error_unset_dtr_r;
		memcpy(&buffer[i], &val, size);
		i += 4;
		address += 4;
	}

	/* Clear any sticky error */
	mem_ap_sel_write_atomic_u32(swjdp, armv8->debug_ap,
		armv8->debug_base + CPUDBG_DRCR, 1<<2);

	/* Done */
	return ERROR_OK;

error_unset_dtr_r:
	LOG_WARNING("DSCR = 0x%" PRIx32, dscr);
	/* Todo: Unset DTR mode */

error_free_buff_r:
	LOG_ERROR("error");
	free(tmp_buff);

	/* Clear any sticky error */
	mem_ap_sel_write_atomic_u32(swjdp, armv8->debug_ap,
		armv8->debug_base + CPUDBG_DRCR, 1<<2);

	return ERROR_FAIL;
}

static int aarch64_read_phys_memory(struct target *target,
	target_ulong address, uint32_t size,
	uint32_t count, uint8_t *buffer)
{
	struct armv8_common *armv8 = target_to_armv8(target);
	struct adiv5_dap *swjdp = armv8->arm.dap;
	int retval = ERROR_COMMAND_SYNTAX_ERROR;
	uint8_t apsel = swjdp->apsel;
	LOG_DEBUG("Reading memory at real address 0x%" PRIXX "; size %" PRId32 "; count %" PRId32,
		address, size, count);

	if (count && buffer) {

		if (armv8->memory_ap_available && (apsel == armv8->memory_ap)) {

			/* read memory through AHB-AP */
			retval = mem_ap_sel_read_buf(swjdp, armv8->memory_ap, buffer, size, count, address);
		} else {
			/* read memory through APB-AP */
			retval = aarch64_mmu_modify(target, 0);
			if (retval != ERROR_OK)
				return retval;
			retval = aarch64_read_apb_ab_memory(target, address, size, count, buffer);
		}
	}
	return retval;
}

static int aarch64_read_memory(struct target *target, target_ulong address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	int mmu_enabled = 0;
	target_ulong virt, phys;
	int retval;
	struct armv8_common *armv8 = target_to_armv8(target);
	struct adiv5_dap *swjdp = armv8->arm.dap;
	uint8_t apsel = swjdp->apsel;

	/* aarch64 handles unaligned memory access */
	LOG_DEBUG("Reading memory at address 0x%" PRIXX "; size %" PRId32
		  "; count %" PRId32, address, size, count);

	/* determine if MMU was enabled on target stop */
	if (!armv8->is_armv7r) {
		retval = aarch64_mmu(target, &mmu_enabled);
		if (retval != ERROR_OK)
			return retval;
	}

	if (armv8->memory_ap_available && (apsel == armv8->memory_ap)) {
		if (mmu_enabled) {
			virt = address;
			retval = aarch64_virt2phys(target, virt, &phys);
			if (retval != ERROR_OK)
				return retval;

			LOG_DEBUG("Reading at virtual address. Translating v:0x%"
				  PRIXX " to r:0x%" PRIXX, virt, phys);
			address = phys;
		}
		retval = aarch64_read_phys_memory(target, address, size, count,
						  buffer);
	} else {
		if (mmu_enabled) {
			retval = aarch64_check_address(target, address);
			if (retval != ERROR_OK)
				return retval;
			/* enable MMU as we could have disabled it for phys
			   access */
			retval = aarch64_mmu_modify(target, 1);
			if (retval != ERROR_OK)
				return retval;
		}
		retval = aarch64_read_apb_ab_memory(target, address, size,
						    count, buffer);
	}
	return retval;
}

static int aarch64_write_phys_memory(struct target *target,
	target_ulong address, uint32_t size,
	uint32_t count, const uint8_t *buffer)
{
	struct armv8_common *armv8 = target_to_armv8(target);
	struct adiv5_dap *swjdp = armv8->arm.dap;
	int retval = ERROR_COMMAND_SYNTAX_ERROR;
	uint8_t apsel = swjdp->apsel;

	LOG_DEBUG("Writing memory to real address 0x%" PRIXX "; size %" PRId32 "; count %" PRId32, address,
		size, count);

	if (count && buffer) {

		if (armv8->memory_ap_available && (apsel == armv8->memory_ap)) {

			/* write memory through AHB-AP */
			retval = mem_ap_sel_write_buf(swjdp, armv8->memory_ap, buffer, size, count, address);
		} else {

			/* write memory through APB-AP */
			if (!armv8->is_armv7r) {
				retval = aarch64_mmu_modify(target, 0);
				if (retval != ERROR_OK)
					return retval;
			}
			return aarch64_write_apb_ab_memory(target, address, size, count, buffer);
		}
	}


	/* REVISIT this op is generic ARMv7-A/R stuff */
	if (retval == ERROR_OK && target->state == TARGET_HALTED) {
		struct arm_dpm *dpm = armv8->arm.dpm;

		retval = dpm->prepare(dpm);
		if (retval != ERROR_OK)
			return retval;

		/* The Cache handling will NOT work with MMU active, the
		 * wrong addresses will be invalidated!
		 *
		 * For both ICache and DCache, walk all cache lines in the
		 * address range. Cortex-A8 has fixed 64 byte line length.
		 *
		 * REVISIT per ARMv7, these may trigger watchpoints ...
		 */

		/* invalidate I-Cache */
		if (armv8->armv8_mmu.armv8_cache.i_cache_enabled) {
			/* ICIMVAU - Invalidate Cache single entry
			 * with MVA to PoU
			 *      MCR p15, 0, r0, c7, c5, 1
			 */
			for (uint32_t cacheline = address;
				cacheline < address + size * count;
				cacheline += 64) {
				retval = dpm->instr_write_data_r0(dpm,
						ARMV4_5_MCR(15, 0, 0, 7, 5, 1),
						cacheline);
				if (retval != ERROR_OK)
					return retval;
			}
		}

		/* invalidate D-Cache */
		if (armv8->armv8_mmu.armv8_cache.d_u_cache_enabled) {
			/* DCIMVAC - Invalidate data Cache line
			 * with MVA to PoC
			 *      MCR p15, 0, r0, c7, c6, 1
			 */
			for (uint32_t cacheline = address;
				cacheline < address + size * count;
				cacheline += 64) {
				retval = dpm->instr_write_data_r0(dpm,
						ARMV4_5_MCR(15, 0, 0, 7, 6, 1),
						cacheline);
				if (retval != ERROR_OK)
					return retval;
			}
		}

		/* (void) */ dpm->finish(dpm);
	}

	return retval;
}

static int aarch64_write_memory(struct target *target, target_ulong address,
	uint32_t size, uint32_t count, const uint8_t *buffer)
{
	int mmu_enabled = 0;
	target_ulong virt, phys;
	int retval;
	struct armv8_common *armv8 = target_to_armv8(target);
	struct adiv5_dap *swjdp = armv8->arm.dap;
	uint8_t apsel = swjdp->apsel;

	/* aarch64 handles unaligned memory access */
	LOG_DEBUG("Writing memory at address 0x%" PRIXX "; size %" PRId32
		  "; count %" PRId32, address, size, count);

	/* determine if MMU was enabled on target stop */
	if (!armv8->is_armv7r) {
		retval = aarch64_mmu(target, &mmu_enabled);
		if (retval != ERROR_OK)
			return retval;
	}

	if (armv8->memory_ap_available && (apsel == armv8->memory_ap)) {
		LOG_DEBUG("Writing memory to address 0x%" PRIXX "; size %"
			  PRId32 "; count %" PRId32, address, size, count);
		if (mmu_enabled) {
			virt = address;
			retval = aarch64_virt2phys(target, virt, &phys);
			if (retval != ERROR_OK)
				return retval;

			LOG_DEBUG("Writing to virtual address. Translating v:0x%"
				  PRIXX " to r:0x%" PRIXX, virt, phys);
			address = phys;
		}
		retval = aarch64_write_phys_memory(target, address, size,
				count, buffer);
	} else {
		if (mmu_enabled) {
			retval = aarch64_check_address(target, address);
			if (retval != ERROR_OK)
				return retval;
			/* enable MMU as we could have disabled it for phys access */
			retval = aarch64_mmu_modify(target, 1);
			if (retval != ERROR_OK)
				return retval;
		}
		retval = aarch64_write_apb_ab_memory(target, address, size, count, buffer);
	}
	return retval;
}

static int aarch64_handle_target_request(void *priv)
{
	struct target *target = priv;
	struct armv8_common *armv8 = target_to_armv8(target);
	struct adiv5_dap *swjdp = armv8->arm.dap;
	int retval;

	if (!target_was_examined(target))
		return ERROR_OK;
	if (!target->dbg_msg_enabled)
		return ERROR_OK;

	if (target->state == TARGET_RUNNING) {
		uint32_t request;
		uint32_t dscr;
		retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
				armv8->debug_base + CPUDBG_DSCR, &dscr);

		/* check if we have data */
		while ((dscr & DSCR_DTR_TX_FULL) && (retval == ERROR_OK)) {
			retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
					armv8->debug_base + CPUDBG_DTRTX, &request);
			if (retval == ERROR_OK) {
				target_request(target, request);
				retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
						armv8->debug_base + CPUDBG_DSCR, &dscr);
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
	uint32_t pfr, debug, ctypr, ttypr, cpuid;

	/* We do one extra read to ensure DAP is configured,
	 * we call ahbap_debugport_init(swjdp) instead
	 */
	retval = ahbap_debugport_init(swjdp);
	if (retval != ERROR_OK)
		return retval;

	/* Search for the APB-AB - it is needed for access to debug registers */
	retval = dap_find_ap(swjdp, AP_TYPE_APB_AP, &armv8->debug_ap);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not find APB-AP for debug access");
		return retval;
	}
	/* Search for the AHB-AB */
	retval = dap_find_ap(swjdp, AP_TYPE_AHB_AP, &armv8->memory_ap);
	if (retval != ERROR_OK) {
		/* AHB-AP not found - use APB-AP */
		LOG_DEBUG("Could not find AHB-AP - using APB-AP for memory access");
		armv8->memory_ap_available = false;
	} else {
		armv8->memory_ap_available = true;
	}


	if (!target->dbgbase_set) {
		uint32_t dbgbase;
		/* Get ROM Table base */
		uint32_t apid;
		int32_t coreidx = target->coreid;
		retval = dap_get_debugbase(swjdp, 1, &dbgbase, &apid);
		if (retval != ERROR_OK)
			return retval;
		/* Lookup 0x15 -- Processor DAP */
		retval = dap_lookup_cs_component(swjdp, 1, dbgbase, 0x15,
				&armv8->debug_base, &coreidx);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("Detected core %" PRId32 " dbgbase: %08" PRIx32,
			  coreidx, armv8->debug_base);
	} else
		armv8->debug_base = target->dbgbase;

	retval = mem_ap_sel_write_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + 0x300, 0);
	if (retval != ERROR_OK) {
		LOG_DEBUG("Examine %s failed", "oslock");
		return retval;
	}

	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + 0x88, &cpuid);
	LOG_DEBUG("0x88 = %x", cpuid);

	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + 0x314, &cpuid);
	LOG_DEBUG("0x314 = %x", cpuid);

	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + 0x310, &cpuid);
	LOG_DEBUG("0x310 = %x", cpuid);
	if (retval != ERROR_OK)
		return retval;

	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + CPUDBG_CPUID, &cpuid);
	if (retval != ERROR_OK) {
		LOG_DEBUG("Examine %s failed", "CPUID");
		return retval;
	}

	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + CPUDBG_CTYPR, &ctypr);
	if (retval != ERROR_OK) {
		LOG_DEBUG("Examine %s failed", "CTYPR");
		return retval;
	}

	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + CPUDBG_TTYPR, &ttypr);
	if (retval != ERROR_OK) {
		LOG_DEBUG("Examine %s failed", "TTYPR");
		return retval;
	}

	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + ID_AA64PFR0_EL1, &pfr);
	if (retval != ERROR_OK) {
		LOG_DEBUG("Examine %s failed", "ID_AA64DFR0_EL1");
		return retval;
	}
	retval = mem_ap_sel_read_atomic_u32(swjdp, armv8->debug_ap,
			armv8->debug_base + ID_AA64DFR0_EL1, &debug);
	if (retval != ERROR_OK) {
		LOG_DEBUG("Examine %s failed", "ID_AA64DFR0_EL1");
		return retval;
	}

	LOG_DEBUG("cpuid = 0x%08" PRIx32, cpuid);
	LOG_DEBUG("ctypr = 0x%08" PRIx32, ctypr);
	LOG_DEBUG("ttypr = 0x%08" PRIx32, ttypr);
	LOG_DEBUG("ID_AA64PFR0_EL1 = 0x%08" PRIx32, pfr);
	LOG_DEBUG("ID_AA64DFR0_EL1 = 0x%08" PRIx32, debug);

	armv8->arm.core_type = ARM_MODE_MON;
	armv8->arm.core_state = ARM_STATE_AARCH64;
	retval = aarch64_dpm_setup(aarch64, debug);
	if (retval != ERROR_OK)
		return retval;

	/* Setup Breakpoint Register Pairs */
	aarch64->brp_num = ((debug >> 12) & 0x0F) + 1;
	aarch64->brp_num_context = ((debug >> 28) & 0x0F) + 1;

	/* hack - no context bpt support yet */
	aarch64->brp_num_context = 0;

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
	struct adiv5_dap *dap = &armv8->dap;

	armv8->arm.dap = dap;

	/* Setup struct aarch64_common */
	aarch64->common_magic = AARCH64_COMMON_MAGIC;
	/*  tap has no dap initialized */
	if (!tap->dap) {
		armv8->arm.dap = dap;
		/* Setup struct aarch64_common */

		/* prepare JTAG information for the new target */
		aarch64->jtag_info.tap = tap;
		aarch64->jtag_info.scann_size = 4;

		/* Leave (only) generic DAP stuff for debugport_init() */
		dap->jtag_info = &aarch64->jtag_info;

		/* Number of bits for tar autoincrement, impl. dep. at least 10 */
		dap->tar_autoincr_block = (1 << 10);
		dap->memaccess_tck = 80;
		tap->dap = dap;
	} else
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

	aarch64->armv8_common.is_armv7r = false;

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

static int aarch64_virt2phys(struct target *target, target_ulong virt,
			     target_ulong *phys)
{
	int retval = ERROR_FAIL;
	struct armv8_common *armv8 = target_to_armv8(target);
	struct adiv5_dap *swjdp = armv8->arm.dap;
	uint8_t apsel = swjdp->apsel;
	if (armv8->memory_ap_available && (apsel == armv8->memory_ap)) {
		uint32_t ret;
		retval = armv8_mmu_translate_va(target,
				virt, &ret);
		if (retval != ERROR_OK)
			goto done;
		*phys = ret;
	} else {/*  use this method if armv8->memory_ap not selected
		 *  mmu must be enable in order to get a correct translation */
		retval = aarch64_mmu_modify(target, 1);
		if (retval != ERROR_OK)
			goto done;
		retval = armv8_mmu_translate_va_pa(target, virt,  phys, 1);
	}
done:
	return retval;
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
	{   .name = "smp_off",
	    .handler = aarch64_handle_smp_off_command,
	    .mode = COMMAND_EXEC,
	    .help = "Stop smp handling",
	    .usage = "",},
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
