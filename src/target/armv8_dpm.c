/*
 * Copyright (C) 2009 by David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arm.h"
#include "armv8.h"
#include "armv8_dpm.h"
#include <jtag/jtag.h>
#include "register.h"
#include "breakpoints.h"
#include "target_type.h"
#include "armv8_opcodes.h"


/**
 * @file
 * Implements various ARM DPM operations using architectural debug registers.
 * These routines layer over core-specific communication methods to cope with
 * implementation differences between cores like ARM1136 and Cortex-A8.
 *
 * The "Debug Programmers' Model" (DPM) for ARMv6 and ARMv7 is defined by
 * Part C (Debug Architecture) of the ARM Architecture Reference Manual,
 * ARMv7-A and ARMv7-R edition (ARM DDI 0406B).  In OpenOCD, DPM operations
 * are abstracted through internal programming interfaces to share code and
 * to minimize needless differences in debug behavior between cores.
 */

/*----------------------------------------------------------------------*/

/*
 * Coprocessor support
 */

/* Read coprocessor */
static int dpmv8_mrc(struct target *target, int cpnum,
	uint32_t op1, uint32_t op2, uint32_t CRn, uint32_t CRm,
	uint32_t *value)
{
	struct arm *arm = target_to_arm(target);
	struct arm_dpm *dpm = arm->dpm;
	int retval;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("MRC p%d, %d, r0, c%d, c%d, %d", cpnum,
		(int) op1, (int) CRn,
		(int) CRm, (int) op2);

	/* read coprocessor register into R0; return via DCC */
	retval = dpm->instr_read_data_r0(dpm,
			T32_FMTITR(ARMV4_5_MRC(cpnum, op1, 0, CRn, CRm, op2)),
			value);

	/* (void) */ dpm->finish(dpm);
	return retval;
}

static int dpmv8_mcr(struct target *target, int cpnum,
	uint32_t op1, uint32_t op2, uint32_t CRn, uint32_t CRm,
	uint32_t value)
{
	struct arm *arm = target_to_arm(target);
	struct arm_dpm *dpm = arm->dpm;
	int retval;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("MCR p%d, %d, r0, c%d, c%d, %d", cpnum,
		(int) op1, (int) CRn,
		(int) CRm, (int) op2);

	/* read DCC into r0; then write coprocessor register from R0 */
	retval = dpm->instr_write_data_r0(dpm,
			T32_FMTITR(ARMV4_5_MCR(cpnum, op1, 0, CRn, CRm, op2)),
			value);

	/* (void) */ dpm->finish(dpm);
	return retval;
}

static int dpmv8_mrs(struct target *target, uint32_t op0,
	uint32_t op1, uint32_t op2, uint32_t CRn, uint32_t CRm,
	uint32_t *value)
{
	struct arm *arm = target_to_arm(target);
	struct arm_dpm *dpm = arm->dpm;
	int retval;
	uint32_t op_code;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		return retval;
	op_code = ((op0 & 0x3) << 19 | (op1 & 0x7) << 16 | (CRn & 0xF) << 12 |\
				(CRm & 0xF) << 8 | (op2 & 0x7) << 5);
	op_code >>= 5;
	LOG_DEBUG("MRS p%d, %d, r0, c%d, c%d, %d", (int)op0,
		(int) op1, (int) CRn,
		(int) CRm, (int) op2);
	/* read coprocessor register into R0; return via DCC */
	retval = dpm->instr_read_data_r0(dpm,
			ARMV8_MRS(op_code, 0),
			value);

	/* (void) */ dpm->finish(dpm);
	return retval;
}

static int dpmv8_msr(struct target *target, uint32_t op0,
	uint32_t op1, uint32_t op2, uint32_t CRn, uint32_t CRm,
	uint32_t value)
{
	struct arm *arm = target_to_arm(target);
	struct arm_dpm *dpm = arm->dpm;
	int retval;
	uint32_t op_code;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		return retval;

	op_code = ((op0 & 0x3) << 19 | (op1 & 0x7) << 16 | (CRn & 0xF) << 12 |\
				(CRm & 0xF) << 8 | (op2 & 0x7) << 5);
	op_code >>= 5;
	LOG_DEBUG("MSR p%d, %d, r0, c%d, c%d, %d", (int)op0,
		(int) op1, (int) CRn,
		(int) CRm, (int) op2);

	/* read DCC into r0; then write coprocessor register from R0 */
	retval = dpm->instr_write_data_r0(dpm,
			ARMV8_MSR_GP(op_code, 0),
			value);

	/* (void) */ dpm->finish(dpm);
	return retval;
}

/*----------------------------------------------------------------------*/

/*
 * Register access utilities
 */

/* Toggles between recorded core mode (USR, SVC, etc) and a temporary one.
 * Routines *must* restore the original mode before returning!!
 */
int dpmv8_modeswitch(struct arm_dpm *dpm, enum arm_mode mode)
{
	int retval;
	uint32_t cpsr;

	/* restore previous mode */
	if (mode == ARM_MODE_ANY)
		cpsr = buf_get_u32(dpm->arm->cpsr->value, 0, 32);

	/* else force to the specified mode */
	else
		cpsr = mode >> 4;

	switch ((cpsr & 0xC) >> 2) {
			case SYSTEM_CUREL_EL1:
				retval = dpm->instr_execute(dpm, ARMV8_DCPS1(11));
				if (retval != ERROR_OK)
					return retval;
				break;
			case SYSTEM_CUREL_EL2:
				retval = dpm->instr_execute(dpm, ARMV8_DCPS2(11));
				if (retval != ERROR_OK)
					return retval;
				break;
			break;
			case SYSTEM_CUREL_EL3:
				retval = dpm->instr_execute(dpm, ARMV8_DCPS3(11));
				if (retval != ERROR_OK)
					return retval;
				break;
			break;
			default:
				LOG_DEBUG("unknow mode 0x%x", (unsigned) ((cpsr & 0xC) >> 2));
				break;
	}


	retval = dpm->instr_write_data_r0(dpm, ARMV8_MSR_DSPSR(0), cpsr);
	if (retval != ERROR_OK)
		return retval;

	if (dpm->instr_cpsr_sync)
		retval = dpm->instr_cpsr_sync(dpm);

	return retval;
}

/* just read the register -- rely on the core mode being right */
static int dpmv8_read_reg(struct arm_dpm *dpm, struct reg *r, unsigned regnum)
{
	uint32_t value;
	uint64_t value_64;
	int retval = ERROR_FAIL;

	switch (regnum) {
		case 0 ... 30:
			retval = dpm->instr_read_data_dcc_64(dpm,
				ARMV8_MSR_GP(SYSTEM_DBG_DBGDTR_EL0, regnum),
				&value_64);
			break;
		case 31:
			retval = dpm->instr_read_data_r0_64(dpm,
				ARMV8_MOVFSP_64(0),
				&value_64);
			break;
		case 32:
			retval = dpm->instr_read_data_r0_64(dpm,
				ARMV8_MRS_DLR(0),
				&value_64);
			break;
		case 33:
			retval = dpm->instr_read_data_r0(dpm,
				ARMV8_MRS_DSPSR(0),
				&value);
		default:
			LOG_DEBUG("READ: %s fail", r->name);
			break;
	}

	if (retval == ERROR_OK) {
		r->valid = true;
		r->dirty = false;
		buf_set_u64(r->value, 0, 32, value_64);
		if (r->size == 64)
			LOG_DEBUG("READ: %s, %16.8llx", r->name, (unsigned long long) value_64);
		else
			LOG_DEBUG("READ: %s, %8.8x", r->name, (unsigned) value);
	}
	return retval;
}

/* just write the register -- rely on the core mode being right */
static int dpmv8_write_reg(struct arm_dpm *dpm, struct reg *r, unsigned regnum)
{
	int retval = ERROR_FAIL;
	uint32_t value = 0xFFFFFFFF;
	uint64_t value_64 = 0xFFFFFFFFFFFFFFFF;

	switch (regnum) {
		case 0 ... 30:
			value_64 = buf_get_u64(r->value, 0, 64);
			retval = dpm->instr_write_data_dcc_64(dpm,
				ARMV8_MRS(SYSTEM_DBG_DBGDTR_EL0, regnum),
				value_64);
			break;
		case 31:
			value_64 = buf_get_u64(r->value, 0, 64);
			retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MOVTSP_64(0),
				value_64);
			break;
		case 32:
			value_64 = buf_get_u64(r->value, 0, 64);
			retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_MSR_DLR(0),
				value_64);
			break;
		case 33:
			value = buf_get_u32(r->value, 0, 32);
			retval = dpm->instr_write_data_r0(dpm,
				ARMV8_MSR_DSPSR(0),
				value);
		default:
			LOG_DEBUG("write: %s fail", r->name);
			break;
	}


	if (retval == ERROR_OK) {
		r->dirty = false;
		if (r->size == 64)
			LOG_DEBUG("WRITE: %s, %16.8llx", r->name, (unsigned long long) value_64);
		else
			LOG_DEBUG("WRITE: %s, %8.8x", r->name, (unsigned) value);
	}

	return retval;
}

/**
 * Read basic registers of the the current context:  R0 to R15, and CPSR;
 * sets the core mode (such as USR or IRQ) and state (such as ARM or Thumb).
 * In normal operation this is called on entry to halting debug state,
 * possibly after some other operations supporting restore of debug state
 * or making sure the CPU is fully idle (drain write buffer, etc).
 */
int armv8_dpm_read_current_registers(struct arm_dpm *dpm)
{
	struct arm *arm = dpm->arm;
	uint32_t cpsr;
	int retval;
	struct reg *r;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		return retval;

	/* read R0 first (it's used for scratch), then CPSR */
	r = arm->core_cache->reg_list + 0;
	if (!r->valid) {
		retval = dpmv8_read_reg(dpm, r, 0);
		if (retval != ERROR_OK)
			goto fail;
	}
	r->dirty = true;
	/* read cpsr to r0 and get it back */
	retval = dpm->instr_read_data_r0(dpm, ARMV8_MRS_DSPSR(0), &cpsr);
	if (retval != ERROR_OK)
		goto fail;

	/* update core mode and state, plus shadow mapping for R8..R14 */
	armv8_set_cpsr(arm, cpsr);

	/* REVISIT we can probably avoid reading R1..R14, saving time... */
	for (unsigned i = 1; i < arm->core_cache->num_regs ; i++) {
		r = armv8_reg_current(arm, i);
		if (r->valid)
			continue;

		retval = dpmv8_read_reg(dpm, r, i);
		if (retval != ERROR_OK)
			goto fail;
	}

	/* NOTE: SPSR ignored (if it's even relevant). */

	/* REVISIT the debugger can trigger various exceptions.  See the
	 * ARMv7A architecture spec, section C5.7, for more info about
	 * what defenses are needed; v6 debug has the most issues.
	 */

fail:
	/* (void) */ dpm->finish(dpm);
	return retval;
}

/* Avoid needless I/O ... leave breakpoints and watchpoints alone
 * unless they're removed, or need updating because of single-stepping
 * or running debugger code.
 */
static int dpmv8_maybe_update_bpwp(struct arm_dpm *dpm, bool bpwp,
	struct dpm_bpwp *xp, int *set_p)
{
	int retval = ERROR_OK;
	bool disable;

	if (!set_p) {
		if (!xp->dirty)
			goto done;
		xp->dirty = false;
		/* removed or startup; we must disable it */
		disable = true;
	} else if (bpwp) {
		if (!xp->dirty)
			goto done;
		/* disabled, but we must set it */
		xp->dirty = disable = false;
		*set_p = true;
	} else {
		if (!*set_p)
			goto done;
		/* set, but we must temporarily disable it */
		xp->dirty = disable = true;
		*set_p = false;
	}

	if (disable)
		retval = dpm->bpwp_disable(dpm, xp->number);
	else
		retval = dpm->bpwp_enable(dpm, xp->number,
				xp->address, xp->control);

	if (retval != ERROR_OK)
		LOG_ERROR("%s: can't %s HW %spoint %d",
			disable ? "disable" : "enable",
			target_name(dpm->arm->target),
			(xp->number < 16) ? "break" : "watch",
			xp->number & 0xf);
done:
	return retval;
}

static int dpmv8_add_breakpoint(struct target *target, struct breakpoint *bp);

/**
 * Writes all modified core registers for all processor modes.  In normal
 * operation this is called on exit from halting debug state.
 *
 * @param dpm: represents the processor
 * @param bpwp: true ensures breakpoints and watchpoints are set,
 *	false ensures they are cleared
 */
int armv8_dpm_write_dirty_registers(struct arm_dpm *dpm, bool bpwp)
{
	struct arm *arm = dpm->arm;
	struct reg_cache *cache = arm->core_cache;
	int retval;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;

	/* If we're managing hardware breakpoints for this core, enable
	 * or disable them as requested.
	 *
	 * REVISIT We don't yet manage them for ANY cores.  Eventually
	 * we should be able to assume we handle them; but until then,
	 * cope with the hand-crafted breakpoint code.
	 */
	if (arm->target->type->add_breakpoint == dpmv8_add_breakpoint) {
		for (unsigned i = 0; i < dpm->nbp; i++) {
			struct dpm_bp *dbp = dpm->dbp + i;
			struct breakpoint *bp = dbp->bp;

			retval = dpmv8_maybe_update_bpwp(dpm, bpwp, &dbp->bpwp,
					bp ? &bp->set : NULL);
			if (retval != ERROR_OK)
				goto done;
		}
	}

	/* enable/disable watchpoints */
	for (unsigned i = 0; i < dpm->nwp; i++) {
		struct dpm_wp *dwp = dpm->dwp + i;
		struct watchpoint *wp = dwp->wp;

		retval = dpmv8_maybe_update_bpwp(dpm, bpwp, &dwp->bpwp,
				wp ? &wp->set : NULL);
		if (retval != ERROR_OK)
			goto done;
	}

	/* NOTE:  writes to breakpoint and watchpoint registers might
	 * be queued, and need (efficient/batched) flushing later.
	 */

	/* Scan the registers until we find one that's both dirty and
	 * eligible for flushing.  Flush that and everything else that
	 * shares the same core mode setting.  Typically this won't
	 * actually find anything to do...
	 */

	/* check everything except our scratch register R0 */
	for (unsigned i = 1; i < cache->num_regs; i++) {
		struct arm_reg *r;
		unsigned regnum;

		/* also skip PC, CPSR, and non-dirty */
		if (i == (arm->core_cache->num_regs - 2))
			continue;
		if (arm->cpsr == cache->reg_list + i)
			continue;
		if (!cache->reg_list[i].dirty)
			continue;

		r = cache->reg_list[i].arch_info;
		regnum = r->num;

		retval = dpmv8_write_reg(dpm,
				&cache->reg_list[i],
				regnum);
		if (retval != ERROR_OK)
			goto done;
	}


	/* Restore original CPSR ... assuming either that we changed it,
	 * or it's dirty.  Must write PC to ensure the return address is
	 * defined, and must not write it before CPSR.
	 */
	retval = dpmv8_modeswitch(dpm, ARM_MODE_ANY);
	if (retval != ERROR_OK)
		goto done;
	arm->cpsr->dirty = false;

	retval = dpmv8_write_reg(dpm, arm->pc, (arm->core_cache->num_regs - 2));
	if (retval != ERROR_OK)
		goto done;
	arm->pc->dirty = false;

	/* flush R0 -- it's *very* dirty by now */
	retval = dpmv8_write_reg(dpm, &cache->reg_list[0], 0);
	if (retval != ERROR_OK)
		goto done;
	cache->reg_list[0].dirty = false;

	/* (void) */ dpm->finish(dpm);
done:
	return retval;
}

/*
 * Standard ARM register accessors ... there are three methods
 * in "struct arm", to support individual read/write and bulk read
 * of registers.
 */

static int armv8_dpm_read_core_reg(struct target *target, struct reg *r,
	int regnum, enum arm_mode mode)
{
	struct arm *arm = target_to_arm(target);
	struct arm_dpm *dpm = target_to_arm(target)->dpm;
	int retval;
	int max = arm->core_cache->num_regs;

	if (regnum < 0 || regnum > max)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* REVISIT what happens if we try to read SPSR in a core mode
	 * which has no such register?
	 */

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		return retval;

	retval = dpmv8_read_reg(dpm, r, regnum);
	if (retval != ERROR_OK)
		goto fail;

fail:
	/* (void) */ dpm->finish(dpm);
	return retval;
}

static int armv8_dpm_write_core_reg(struct target *target, struct reg *r,
	int regnum, enum arm_mode mode, uint8_t *value)
{
	struct arm *arm = target_to_arm(target);
	struct arm_dpm *dpm = target_to_arm(target)->dpm;
	int retval;
	int max = arm->core_cache->num_regs;

	if (regnum < 0 || regnum > max)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* REVISIT what happens if we try to write SPSR in a core mode
	 * which has no such register?
	 */

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		return retval;

	retval = dpmv8_write_reg(dpm, r, regnum);
	/* always clean up, regardless of error */

	/* (void) */ dpm->finish(dpm);
	return retval;
}

static int armv8_dpm_full_context(struct target *target)
{
	struct arm *arm = target_to_arm(target);
	struct arm_dpm *dpm = arm->dpm;
	struct reg_cache *cache = arm->core_cache;
	int retval;
	bool did_read;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;

	do {
		enum arm_mode mode = ARM_MODE_ANY;

		did_read = false;

		/* We "know" arm_dpm_read_current_registers() was called so
		 * the unmapped registers (R0..R7, PC, AND CPSR) and some
		 * view of R8..R14 are current.  We also "know" oddities of
		 * register mapping: special cases for R8..R12 and SPSR.
		 *
		 * Pick some mode with unread registers and read them all.
		 * Repeat until done.
		 */
		for (unsigned i = 0; i < cache->num_regs; i++) {
			struct arm_reg *r;

			if (cache->reg_list[i].valid)
				continue;
			r = cache->reg_list[i].arch_info;

			/* may need to pick a mode and set CPSR */
			if (!did_read) {
				did_read = true;
				mode = r->mode;

				/* For regular (ARM_MODE_ANY) R8..R12
				 * in case we've entered debug state
				 * in FIQ mode we need to patch mode.
				 */
				if (mode != ARM_MODE_ANY)
					retval = dpmv8_modeswitch(dpm, mode);
				else
					retval = dpmv8_modeswitch(dpm, ARM_MODE_USR);

				if (retval != ERROR_OK)
					goto done;
			}
			if (r->mode != mode)
				continue;

			/* CPSR was read, so "R16" must mean SPSR */
			retval = dpmv8_read_reg(dpm,
					&cache->reg_list[i],
					(r->num == 16) ? 17 : r->num);
			if (retval != ERROR_OK)
				goto done;
		}

	} while (did_read);

	retval = dpmv8_modeswitch(dpm, ARM_MODE_ANY);
	/* (void) */ dpm->finish(dpm);
done:
	return retval;
}


/*----------------------------------------------------------------------*/

/*
 * Breakpoint and Watchpoint support.
 *
 * Hardware {break,watch}points are usually left active, to minimize
 * debug entry/exit costs.  When they are set or cleared, it's done in
 * batches.  Also, DPM-conformant hardware can update debug registers
 * regardless of whether the CPU is running or halted ... though that
 * fact isn't currently leveraged.
 */

static int dpmv8_bpwp_setup(struct arm_dpm *dpm, struct dpm_bpwp *xp,
	uint32_t addr, uint32_t length)
{
	uint32_t control;

	control = (1 << 0)	/* enable */
		| (3 << 1);	/* both user and privileged access */

	/* Match 1, 2, or all 4 byte addresses in this word.
	 *
	 * FIXME:  v7 hardware allows lengths up to 2 GB for BP and WP.
	 * Support larger length, when addr is suitably aligned.  In
	 * particular, allow watchpoints on 8 byte "double" values.
	 *
	 * REVISIT allow watchpoints on unaligned 2-bit values; and on
	 * v7 hardware, unaligned 4-byte ones too.
	 */
	switch (length) {
		case 1:
			control |= (1 << (addr & 3)) << 5;
			break;
		case 2:
			/* require 2-byte alignment */
			if (!(addr & 1)) {
				control |= (3 << (addr & 2)) << 5;
				break;
			}
		/* FALL THROUGH */
		case 4:
			/* require 4-byte alignment */
			if (!(addr & 3)) {
				control |= 0xf << 5;
				break;
			}
		/* FALL THROUGH */
		default:
			LOG_ERROR("unsupported {break,watch}point length/alignment");
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* other shared control bits:
	 * bits 15:14 == 0 ... both secure and nonsecure states (v6.1+ only)
	 * bit 20 == 0 ... not linked to a context ID
	 * bit 28:24 == 0 ... not ignoring N LSBs (v7 only)
	 */

	xp->address = addr & ~3;
	xp->control = control;
	xp->dirty = true;

	LOG_DEBUG("BPWP: addr %8.8" PRIx32 ", control %" PRIx32 ", number %d",
		xp->address, control, xp->number);

	/* hardware is updated in write_dirty_registers() */
	return ERROR_OK;
}

static int dpmv8_add_breakpoint(struct target *target, struct breakpoint *bp)
{
	struct arm *arm = target_to_arm(target);
	struct arm_dpm *dpm = arm->dpm;
	int retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	if (bp->length < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (!dpm->bpwp_enable)
		return retval;

	/* FIXME we need a generic solution for software breakpoints. */
	if (bp->type == BKPT_SOFT)
		LOG_DEBUG("using HW bkpt, not SW...");

	for (unsigned i = 0; i < dpm->nbp; i++) {
		if (!dpm->dbp[i].bp) {
			retval = dpmv8_bpwp_setup(dpm, &dpm->dbp[i].bpwp,
					bp->address, bp->length);
			if (retval == ERROR_OK)
				dpm->dbp[i].bp = bp;
			break;
		}
	}

	return retval;
}

static int dpmv8_remove_breakpoint(struct target *target, struct breakpoint *bp)
{
	struct arm *arm = target_to_arm(target);
	struct arm_dpm *dpm = arm->dpm;
	int retval = ERROR_COMMAND_SYNTAX_ERROR;

	for (unsigned i = 0; i < dpm->nbp; i++) {
		if (dpm->dbp[i].bp == bp) {
			dpm->dbp[i].bp = NULL;
			dpm->dbp[i].bpwp.dirty = true;

			/* hardware is updated in write_dirty_registers() */
			retval = ERROR_OK;
			break;
		}
	}

	return retval;
}

static int dpmv8_watchpoint_setup(struct arm_dpm *dpm, unsigned index_t,
	struct watchpoint *wp)
{
	int retval;
	struct dpm_wp *dwp = dpm->dwp + index_t;
	uint32_t control;

	/* this hardware doesn't support data value matching or masking */
	if (wp->value || wp->mask != ~(uint32_t)0) {
		LOG_DEBUG("watchpoint values and masking not supported");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = dpmv8_bpwp_setup(dpm, &dwp->bpwp, wp->address, wp->length);
	if (retval != ERROR_OK)
		return retval;

	control = dwp->bpwp.control;
	switch (wp->rw) {
		case WPT_READ:
			control |= 1 << 3;
			break;
		case WPT_WRITE:
			control |= 2 << 3;
			break;
		case WPT_ACCESS:
			control |= 3 << 3;
			break;
	}
	dwp->bpwp.control = control;

	dpm->dwp[index_t].wp = wp;

	return retval;
}

static int dpmv8_add_watchpoint(struct target *target, struct watchpoint *wp)
{
	struct arm *arm = target_to_arm(target);
	struct arm_dpm *dpm = arm->dpm;
	int retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	if (dpm->bpwp_enable) {
		for (unsigned i = 0; i < dpm->nwp; i++) {
			if (!dpm->dwp[i].wp) {
				retval = dpmv8_watchpoint_setup(dpm, i, wp);
				break;
			}
		}
	}

	return retval;
}

static int dpmv8_remove_watchpoint(struct target *target, struct watchpoint *wp)
{
	struct arm *arm = target_to_arm(target);
	struct arm_dpm *dpm = arm->dpm;
	int retval = ERROR_COMMAND_SYNTAX_ERROR;

	for (unsigned i = 0; i < dpm->nwp; i++) {
		if (dpm->dwp[i].wp == wp) {
			dpm->dwp[i].wp = NULL;
			dpm->dwp[i].bpwp.dirty = true;

			/* hardware is updated in write_dirty_registers() */
			retval = ERROR_OK;
			break;
		}
	}

	return retval;
}

void armv8_dpm_report_wfar(struct arm_dpm *dpm, uint64_t addr)
{
	switch (dpm->arm->core_state) {
		case ARM_STATE_ARM:
		case ARM_STATE_AARCH64:
			addr -= 8;
			break;
		case ARM_STATE_THUMB:
		case ARM_STATE_THUMB_EE:
			addr -= 4;
			break;
		case ARM_STATE_JAZELLE:
			/* ?? */
			break;
		default:
			LOG_DEBUG("Unknow core_state");
			break;
	}
	dpm->wp_pc = addr;
}

/*----------------------------------------------------------------------*/

/*
 * Other debug and support utilities
 */

void armv8_dpm_report_dscr(struct arm_dpm *dpm, uint32_t dscr)
{
	struct target *target = dpm->arm->target;

	dpm->dscr = dscr;

	/* Examine debug reason */
	switch (DSCR_ENTRY(dscr)) {
		/* FALL THROUGH -- assume a v6 core in abort mode */
		case DSCRV8_ENTRY_HLT:	/* HALT request from debugger */
		case DSCRV8_ENTRY_EXT_DEBUG:	/* EDBGRQ */
			target->debug_reason = DBG_REASON_DBGRQ;
			break;
		case DSCRV8_ENTRY_HALT_STEP_EXECLU:	/* HALT step */
		case DSCRV8_ENTRY_BKPT:	/* SW BKPT */
		case DSCRV8_ENTRY_RESET_CATCH:	/* Reset catch */
		case DSCRV8_ENTRY_OS_UNLOCK:  /*OS unlock catch*/
		case DSCRV8_ENTRY_EXCEPTION_CATCH:  /*exception catch*/
		case DSCRV8_ENTRY_HALT_STEP_NORMAL: /* Halt step*/
		case DSCRV8_ENTRY_SW_ACCESS_DBG: /*SW access dbg register*/
			target->debug_reason = DBG_REASON_BREAKPOINT;
			break;
		case DSCRV8_ENTRY_WATCHPOINT:	/* asynch watchpoint */
			target->debug_reason = DBG_REASON_WATCHPOINT;
			break;
		default:
			target->debug_reason = DBG_REASON_UNDEFINED;
			break;
	}

}

/*----------------------------------------------------------------------*/

/*
 * Setup and management support.
 */

/**
 * Hooks up this DPM to its associated target; call only once.
 * Initially this only covers the register cache.
 *
 * Oh, and watchpoints.  Yeah.
 */
int armv8_dpm_setup(struct arm_dpm *dpm)
{
	struct arm *arm = dpm->arm;
	struct target *target = arm->target;
	struct reg_cache *cache;
	arm->dpm = dpm;

	/* register access setup */
	arm->full_context = armv8_dpm_full_context;
	arm->read_core_reg = armv8_dpm_read_core_reg;
	arm->write_core_reg = armv8_dpm_write_core_reg;

	if (arm->core_cache == NULL) {
		cache = armv8_build_reg_cache(target);
		if (!cache)
			return ERROR_FAIL;
	}

	/* coprocessor access setup */
	arm->mrc = dpmv8_mrc;
	arm->mcr = dpmv8_mcr;
	arm->mrs = dpmv8_mrs;
	arm->msr = dpmv8_msr;
	/* breakpoint setup -- optional until it works everywhere */
	if (!target->type->add_breakpoint) {
		target->type->add_breakpoint = dpmv8_add_breakpoint;
		target->type->remove_breakpoint = dpmv8_remove_breakpoint;
	}

	/* watchpoint setup */
	target->type->add_watchpoint = dpmv8_add_watchpoint;
	target->type->remove_watchpoint = dpmv8_remove_watchpoint;

	/* FIXME add vector catch support */

	dpm->nbp = 1 + ((dpm->didr >> 12) & 0xf);
	dpm->dbp = calloc(dpm->nbp, sizeof *dpm->dbp);

	dpm->nwp = 1 + ((dpm->didr >> 20) & 0xf);
	dpm->dwp = calloc(dpm->nwp, sizeof *dpm->dwp);

	if (!dpm->dbp || !dpm->dwp) {
		free(dpm->dbp);
		free(dpm->dwp);
		return ERROR_FAIL;
	}

	LOG_INFO("%s: hardware has %d breakpoints, %d watchpoints",
		target_name(target), dpm->nbp, dpm->nwp);

	/* REVISIT ... and some of those breakpoints could match
	 * execution context IDs...
	 */

	return ERROR_OK;
}

/**
 * Reinitializes DPM state at the beginning of a new debug session
 * or after a reset which may have affected the debug module.
 */
int armv8_dpm_initialize(struct arm_dpm *dpm)
{
	/* Disable all breakpoints and watchpoints at startup. */
	if (dpm->bpwp_disable) {
		unsigned i;

		for (i = 0; i < dpm->nbp; i++) {
			dpm->dbp[i].bpwp.number = i;
			(void) dpm->bpwp_disable(dpm, i);
		}
		for (i = 0; i < dpm->nwp; i++) {
			dpm->dwp[i].bpwp.number = 16 + i;
			(void) dpm->bpwp_disable(dpm, 16 + i);
		}
	} else
		LOG_WARNING("%s: can't disable breakpoints and watchpoints",
			target_name(dpm->arm->target));

	return ERROR_OK;
}
