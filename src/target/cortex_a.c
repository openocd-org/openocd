/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2009 by Dirk Behme                                      *
 *   dirk.behme@gmail.com - copy from cortex_m3                            *
 *                                                                         *
 *   Copyright (C) 2010 Øyvind Harboe                                      *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) ST-Ericsson SA 2011                                     *
 *   michel.jaouen@stericsson.com : smp minimum support                    *
 *                                                                         *
 *   Copyright (C) Broadcom 2012                                           *
 *   ehunter@broadcom.com : Cortex R4 support                              *
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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 *                                                                         *
 *   Cortex-A8(tm) TRM, ARM DDI 0344H                                      *
 *   Cortex-A9(tm) TRM, ARM DDI 0407F                                      *
 *   Cortex-A4(tm) TRM, ARM DDI 0363E                                      *
 *                                                                         *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "breakpoints.h"
#include "cortex_a.h"
#include "register.h"
#include "target_request.h"
#include "target_type.h"
#include "arm_opcodes.h"
#include <helper/time_support.h>

static int cortex_a8_poll(struct target *target);
static int cortex_a8_debug_entry(struct target *target);
static int cortex_a8_restore_context(struct target *target, bool bpwp);
static int cortex_a8_set_breakpoint(struct target *target,
	struct breakpoint *breakpoint, uint8_t matchmode);
static int cortex_a8_set_context_breakpoint(struct target *target,
	struct breakpoint *breakpoint, uint8_t matchmode);
static int cortex_a8_set_hybrid_breakpoint(struct target *target,
	struct breakpoint *breakpoint);
static int cortex_a8_unset_breakpoint(struct target *target,
	struct breakpoint *breakpoint);
static int cortex_a8_dap_read_coreregister_u32(struct target *target,
	uint32_t *value, int regnum);
static int cortex_a8_dap_write_coreregister_u32(struct target *target,
	uint32_t value, int regnum);
static int cortex_a8_mmu(struct target *target, int *enabled);
static int cortex_a8_virt2phys(struct target *target,
	uint32_t virt, uint32_t *phys);
static int cortex_a8_read_apb_ab_memory(struct target *target,
	uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer);


/*  restore cp15_control_reg at resume */
static int cortex_a8_restore_cp15_control_reg(struct target *target)
{
	int retval = ERROR_OK;
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);
	struct armv7a_common *armv7a = target_to_armv7a(target);

	if (cortex_a8->cp15_control_reg != cortex_a8->cp15_control_reg_curr) {
		cortex_a8->cp15_control_reg_curr = cortex_a8->cp15_control_reg;
		/* LOG_INFO("cp15_control_reg: %8.8" PRIx32, cortex_a8->cp15_control_reg); */
		retval = armv7a->arm.mcr(target, 15,
				0, 0,	/* op1, op2 */
				1, 0,	/* CRn, CRm */
				cortex_a8->cp15_control_reg);
	}
	return retval;
}

/*  check address before cortex_a8_apb read write access with mmu on
 *  remove apb predictible data abort */
static int cortex_a8_check_address(struct target *target, uint32_t address)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);
	uint32_t os_border = armv7a->armv7a_mmu.os_border;
	if ((address < os_border) &&
		(armv7a->arm.core_mode == ARM_MODE_SVC)) {
		LOG_ERROR("%" PRIx32 " access in userspace and target in supervisor", address);
		return ERROR_FAIL;
	}
	if ((address >= os_border) &&
		(cortex_a8->curr_mode != ARM_MODE_SVC)) {
		dpm_modeswitch(&armv7a->dpm, ARM_MODE_SVC);
		cortex_a8->curr_mode = ARM_MODE_SVC;
		LOG_INFO("%" PRIx32 " access in kernel space and target not in supervisor",
			address);
		return ERROR_OK;
	}
	if ((address < os_border) &&
		(cortex_a8->curr_mode == ARM_MODE_SVC)) {
		dpm_modeswitch(&armv7a->dpm, ARM_MODE_ANY);
		cortex_a8->curr_mode = ARM_MODE_ANY;
	}
	return ERROR_OK;
}
/*  modify cp15_control_reg in order to enable or disable mmu for :
 *  - virt2phys address conversion
 *  - read or write memory in phys or virt address */
static int cortex_a8_mmu_modify(struct target *target, int enable)
{
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);
	struct armv7a_common *armv7a = target_to_armv7a(target);
	int retval = ERROR_OK;
	if (enable) {
		/*  if mmu enabled at target stop and mmu not enable */
		if (!(cortex_a8->cp15_control_reg & 0x1U)) {
			LOG_ERROR("trying to enable mmu on target stopped with mmu disable");
			return ERROR_FAIL;
		}
		if (!(cortex_a8->cp15_control_reg_curr & 0x1U)) {
			cortex_a8->cp15_control_reg_curr |= 0x1U;
			retval = armv7a->arm.mcr(target, 15,
					0, 0,	/* op1, op2 */
					1, 0,	/* CRn, CRm */
					cortex_a8->cp15_control_reg_curr);
		}
	} else {
		if (cortex_a8->cp15_control_reg_curr & 0x4U) {
			/*  data cache is active */
			cortex_a8->cp15_control_reg_curr &= ~0x4U;
			/* flush data cache armv7 function to be called */
			if (armv7a->armv7a_mmu.armv7a_cache.flush_all_data_cache)
				armv7a->armv7a_mmu.armv7a_cache.flush_all_data_cache(target);
		}
		if ((cortex_a8->cp15_control_reg_curr & 0x1U)) {
			cortex_a8->cp15_control_reg_curr &= ~0x1U;
			retval = armv7a->arm.mcr(target, 15,
					0, 0,	/* op1, op2 */
					1, 0,	/* CRn, CRm */
					cortex_a8->cp15_control_reg_curr);
		}
	}
	return retval;
}

/*
 * Cortex-A8 Basic debug access, very low level assumes state is saved
 */
static int cortex_a8_init_debug_access(struct target *target)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct adiv5_dap *swjdp = armv7a->arm.dap;
	int retval;
	uint32_t dummy;

	LOG_DEBUG(" ");

	/* Unlocking the debug registers for modification
	 * The debugport might be uninitialised so try twice */
	retval = mem_ap_sel_write_atomic_u32(swjdp, armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_LOCKACCESS, 0xC5ACCE55);
	if (retval != ERROR_OK) {
		/* try again */
		retval = mem_ap_sel_write_atomic_u32(swjdp, armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_LOCKACCESS, 0xC5ACCE55);
		if (retval == ERROR_OK)
			LOG_USER(
				"Locking debug access failed on first, but succeeded on second try.");
	}
	if (retval != ERROR_OK)
		return retval;
	/* Clear Sticky Power Down status Bit in PRSR to enable access to
	   the registers in the Core Power Domain */
	retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_PRSR, &dummy);
	if (retval != ERROR_OK)
		return retval;

	/* Enabling of instruction execution in debug mode is done in debug_entry code */

	/* Resync breakpoint registers */

	/* Since this is likely called from init or reset, update target state information*/
	return cortex_a8_poll(target);
}

/* To reduce needless round-trips, pass in a pointer to the current
 * DSCR value.  Initialize it to zero if you just need to know the
 * value on return from this function; or DSCR_INSTR_COMP if you
 * happen to know that no instruction is pending.
 */
static int cortex_a8_exec_opcode(struct target *target,
	uint32_t opcode, uint32_t *dscr_p)
{
	uint32_t dscr;
	int retval;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct adiv5_dap *swjdp = armv7a->arm.dap;

	dscr = dscr_p ? *dscr_p : 0;

	LOG_DEBUG("exec opcode 0x%08" PRIx32, opcode);

	/* Wait for InstrCompl bit to be set */
	long long then = timeval_ms();
	while ((dscr & DSCR_INSTR_COMP) == 0) {
		retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_DSCR, &dscr);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not read DSCR register, opcode = 0x%08" PRIx32, opcode);
			return retval;
		}
		if (timeval_ms() > then + 1000) {
			LOG_ERROR("Timeout waiting for cortex_a8_exec_opcode");
			return ERROR_FAIL;
		}
	}

	retval = mem_ap_sel_write_u32(swjdp, armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_ITR, opcode);
	if (retval != ERROR_OK)
		return retval;

	then = timeval_ms();
	do {
		retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_DSCR, &dscr);
		if (retval != ERROR_OK) {
			LOG_ERROR("Could not read DSCR register");
			return retval;
		}
		if (timeval_ms() > then + 1000) {
			LOG_ERROR("Timeout waiting for cortex_a8_exec_opcode");
			return ERROR_FAIL;
		}
	} while ((dscr & DSCR_INSTR_COMP) == 0);	/* Wait for InstrCompl bit to be set */

	if (dscr_p)
		*dscr_p = dscr;

	return retval;
}

/**************************************************************************
Read core register with very few exec_opcode, fast but needs work_area.
This can cause problems with MMU active.
**************************************************************************/
static int cortex_a8_read_regs_through_mem(struct target *target, uint32_t address,
	uint32_t *regfile)
{
	int retval = ERROR_OK;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct adiv5_dap *swjdp = armv7a->arm.dap;

	retval = cortex_a8_dap_read_coreregister_u32(target, regfile, 0);
	if (retval != ERROR_OK)
		return retval;
	retval = cortex_a8_dap_write_coreregister_u32(target, address, 0);
	if (retval != ERROR_OK)
		return retval;
	retval = cortex_a8_exec_opcode(target, ARMV4_5_STMIA(0, 0xFFFE, 0, 0), NULL);
	if (retval != ERROR_OK)
		return retval;

	retval = mem_ap_sel_read_buf(swjdp, armv7a->memory_ap,
			(uint8_t *)(&regfile[1]), 4, 15, address);

	return retval;
}

static int cortex_a8_dap_read_coreregister_u32(struct target *target,
	uint32_t *value, int regnum)
{
	int retval = ERROR_OK;
	uint8_t reg = regnum&0xFF;
	uint32_t dscr = 0;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct adiv5_dap *swjdp = armv7a->arm.dap;

	if (reg > 17)
		return retval;

	if (reg < 15) {
		/* Rn to DCCTX, "MCR p14, 0, Rn, c0, c5, 0"  0xEE00nE15 */
		retval = cortex_a8_exec_opcode(target,
				ARMV4_5_MCR(14, 0, reg, 0, 5, 0),
				&dscr);
		if (retval != ERROR_OK)
			return retval;
	} else if (reg == 15) {
		/* "MOV r0, r15"; then move r0 to DCCTX */
		retval = cortex_a8_exec_opcode(target, 0xE1A0000F, &dscr);
		if (retval != ERROR_OK)
			return retval;
		retval = cortex_a8_exec_opcode(target,
				ARMV4_5_MCR(14, 0, 0, 0, 5, 0),
				&dscr);
		if (retval != ERROR_OK)
			return retval;
	} else {
		/* "MRS r0, CPSR" or "MRS r0, SPSR"
		 * then move r0 to DCCTX
		 */
		retval = cortex_a8_exec_opcode(target, ARMV4_5_MRS(0, reg & 1), &dscr);
		if (retval != ERROR_OK)
			return retval;
		retval = cortex_a8_exec_opcode(target,
				ARMV4_5_MCR(14, 0, 0, 0, 5, 0),
				&dscr);
		if (retval != ERROR_OK)
			return retval;
	}

	/* Wait for DTRRXfull then read DTRRTX */
	long long then = timeval_ms();
	while ((dscr & DSCR_DTR_TX_FULL) == 0) {
		retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_DSCR, &dscr);
		if (retval != ERROR_OK)
			return retval;
		if (timeval_ms() > then + 1000) {
			LOG_ERROR("Timeout waiting for cortex_a8_exec_opcode");
			return ERROR_FAIL;
		}
	}

	retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DTRTX, value);
	LOG_DEBUG("read DCC 0x%08" PRIx32, *value);

	return retval;
}

static int cortex_a8_dap_write_coreregister_u32(struct target *target,
	uint32_t value, int regnum)
{
	int retval = ERROR_OK;
	uint8_t Rd = regnum&0xFF;
	uint32_t dscr;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct adiv5_dap *swjdp = armv7a->arm.dap;

	LOG_DEBUG("register %i, value 0x%08" PRIx32, regnum, value);

	/* Check that DCCRX is not full */
	retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		return retval;
	if (dscr & DSCR_DTR_RX_FULL) {
		LOG_ERROR("DSCR_DTR_RX_FULL, dscr 0x%08" PRIx32, dscr);
		/* Clear DCCRX with MRC(p14, 0, Rd, c0, c5, 0), opcode  0xEE100E15 */
		retval = cortex_a8_exec_opcode(target, ARMV4_5_MRC(14, 0, 0, 0, 5, 0),
				&dscr);
		if (retval != ERROR_OK)
			return retval;
	}

	if (Rd > 17)
		return retval;

	/* Write DTRRX ... sets DSCR.DTRRXfull but exec_opcode() won't care */
	LOG_DEBUG("write DCC 0x%08" PRIx32, value);
	retval = mem_ap_sel_write_u32(swjdp, armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DTRRX, value);
	if (retval != ERROR_OK)
		return retval;

	if (Rd < 15) {
		/* DCCRX to Rn, "MRC p14, 0, Rn, c0, c5, 0", 0xEE10nE15 */
		retval = cortex_a8_exec_opcode(target, ARMV4_5_MRC(14, 0, Rd, 0, 5, 0),
				&dscr);

		if (retval != ERROR_OK)
			return retval;
	} else if (Rd == 15) {
		/* DCCRX to R0, "MRC p14, 0, R0, c0, c5, 0", 0xEE100E15
		 * then "mov r15, r0"
		 */
		retval = cortex_a8_exec_opcode(target, ARMV4_5_MRC(14, 0, 0, 0, 5, 0),
				&dscr);
		if (retval != ERROR_OK)
			return retval;
		retval = cortex_a8_exec_opcode(target, 0xE1A0F000, &dscr);
		if (retval != ERROR_OK)
			return retval;
	} else {
		/* DCCRX to R0, "MRC p14, 0, R0, c0, c5, 0", 0xEE100E15
		 * then "MSR CPSR_cxsf, r0" or "MSR SPSR_cxsf, r0" (all fields)
		 */
		retval = cortex_a8_exec_opcode(target, ARMV4_5_MRC(14, 0, 0, 0, 5, 0),
				&dscr);
		if (retval != ERROR_OK)
			return retval;
		retval = cortex_a8_exec_opcode(target, ARMV4_5_MSR_GP(0, 0xF, Rd & 1),
				&dscr);
		if (retval != ERROR_OK)
			return retval;

		/* "Prefetch flush" after modifying execution status in CPSR */
		if (Rd == 16) {
			retval = cortex_a8_exec_opcode(target,
					ARMV4_5_MCR(15, 0, 0, 7, 5, 4),
					&dscr);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	return retval;
}

/* Write to memory mapped registers directly with no cache or mmu handling */
static int cortex_a8_dap_write_memap_register_u32(struct target *target,
	uint32_t address,
	uint32_t value)
{
	int retval;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct adiv5_dap *swjdp = armv7a->arm.dap;

	retval = mem_ap_sel_write_atomic_u32(swjdp, armv7a->debug_ap, address, value);

	return retval;
}

/*
 * Cortex-A8 implementation of Debug Programmer's Model
 *
 * NOTE the invariant:  these routines return with DSCR_INSTR_COMP set,
 * so there's no need to poll for it before executing an instruction.
 *
 * NOTE that in several of these cases the "stall" mode might be useful.
 * It'd let us queue a few operations together... prepare/finish might
 * be the places to enable/disable that mode.
 */

static inline struct cortex_a8_common *dpm_to_a8(struct arm_dpm *dpm)
{
	return container_of(dpm, struct cortex_a8_common, armv7a_common.dpm);
}

static int cortex_a8_write_dcc(struct cortex_a8_common *a8, uint32_t data)
{
	LOG_DEBUG("write DCC 0x%08" PRIx32, data);
	return mem_ap_sel_write_u32(a8->armv7a_common.arm.dap,
		a8->armv7a_common.debug_ap, a8->armv7a_common.debug_base + CPUDBG_DTRRX, data);
}

static int cortex_a8_read_dcc(struct cortex_a8_common *a8, uint32_t *data,
	uint32_t *dscr_p)
{
	struct adiv5_dap *swjdp = a8->armv7a_common.arm.dap;
	uint32_t dscr = DSCR_INSTR_COMP;
	int retval;

	if (dscr_p)
		dscr = *dscr_p;

	/* Wait for DTRRXfull */
	long long then = timeval_ms();
	while ((dscr & DSCR_DTR_TX_FULL) == 0) {
		retval = mem_ap_sel_read_atomic_u32(swjdp, a8->armv7a_common.debug_ap,
				a8->armv7a_common.debug_base + CPUDBG_DSCR,
				&dscr);
		if (retval != ERROR_OK)
			return retval;
		if (timeval_ms() > then + 1000) {
			LOG_ERROR("Timeout waiting for read dcc");
			return ERROR_FAIL;
		}
	}

	retval = mem_ap_sel_read_atomic_u32(swjdp, a8->armv7a_common.debug_ap,
			a8->armv7a_common.debug_base + CPUDBG_DTRTX, data);
	if (retval != ERROR_OK)
		return retval;
	/* LOG_DEBUG("read DCC 0x%08" PRIx32, *data); */

	if (dscr_p)
		*dscr_p = dscr;

	return retval;
}

static int cortex_a8_dpm_prepare(struct arm_dpm *dpm)
{
	struct cortex_a8_common *a8 = dpm_to_a8(dpm);
	struct adiv5_dap *swjdp = a8->armv7a_common.arm.dap;
	uint32_t dscr;
	int retval;

	/* set up invariant:  INSTR_COMP is set after ever DPM operation */
	long long then = timeval_ms();
	for (;; ) {
		retval = mem_ap_sel_read_atomic_u32(swjdp, a8->armv7a_common.debug_ap,
				a8->armv7a_common.debug_base + CPUDBG_DSCR,
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
		retval = cortex_a8_exec_opcode(
				a8->armv7a_common.arm.target,
				ARMV4_5_MRC(14, 0, 0, 0, 5, 0),
				&dscr);
		if (retval != ERROR_OK)
			return retval;
	}

	return retval;
}

static int cortex_a8_dpm_finish(struct arm_dpm *dpm)
{
	/* REVISIT what could be done here? */
	return ERROR_OK;
}

static int cortex_a8_instr_write_data_dcc(struct arm_dpm *dpm,
	uint32_t opcode, uint32_t data)
{
	struct cortex_a8_common *a8 = dpm_to_a8(dpm);
	int retval;
	uint32_t dscr = DSCR_INSTR_COMP;

	retval = cortex_a8_write_dcc(a8, data);
	if (retval != ERROR_OK)
		return retval;

	return cortex_a8_exec_opcode(
			a8->armv7a_common.arm.target,
			opcode,
			&dscr);
}

static int cortex_a8_instr_write_data_r0(struct arm_dpm *dpm,
	uint32_t opcode, uint32_t data)
{
	struct cortex_a8_common *a8 = dpm_to_a8(dpm);
	uint32_t dscr = DSCR_INSTR_COMP;
	int retval;

	retval = cortex_a8_write_dcc(a8, data);
	if (retval != ERROR_OK)
		return retval;

	/* DCCRX to R0, "MCR p14, 0, R0, c0, c5, 0", 0xEE000E15 */
	retval = cortex_a8_exec_opcode(
			a8->armv7a_common.arm.target,
			ARMV4_5_MRC(14, 0, 0, 0, 5, 0),
			&dscr);
	if (retval != ERROR_OK)
		return retval;

	/* then the opcode, taking data from R0 */
	retval = cortex_a8_exec_opcode(
			a8->armv7a_common.arm.target,
			opcode,
			&dscr);

	return retval;
}

static int cortex_a8_instr_cpsr_sync(struct arm_dpm *dpm)
{
	struct target *target = dpm->arm->target;
	uint32_t dscr = DSCR_INSTR_COMP;

	/* "Prefetch flush" after modifying execution status in CPSR */
	return cortex_a8_exec_opcode(target,
			ARMV4_5_MCR(15, 0, 0, 7, 5, 4),
			&dscr);
}

static int cortex_a8_instr_read_data_dcc(struct arm_dpm *dpm,
	uint32_t opcode, uint32_t *data)
{
	struct cortex_a8_common *a8 = dpm_to_a8(dpm);
	int retval;
	uint32_t dscr = DSCR_INSTR_COMP;

	/* the opcode, writing data to DCC */
	retval = cortex_a8_exec_opcode(
			a8->armv7a_common.arm.target,
			opcode,
			&dscr);
	if (retval != ERROR_OK)
		return retval;

	return cortex_a8_read_dcc(a8, data, &dscr);
}


static int cortex_a8_instr_read_data_r0(struct arm_dpm *dpm,
	uint32_t opcode, uint32_t *data)
{
	struct cortex_a8_common *a8 = dpm_to_a8(dpm);
	uint32_t dscr = DSCR_INSTR_COMP;
	int retval;

	/* the opcode, writing data to R0 */
	retval = cortex_a8_exec_opcode(
			a8->armv7a_common.arm.target,
			opcode,
			&dscr);
	if (retval != ERROR_OK)
		return retval;

	/* write R0 to DCC */
	retval = cortex_a8_exec_opcode(
			a8->armv7a_common.arm.target,
			ARMV4_5_MCR(14, 0, 0, 0, 5, 0),
			&dscr);
	if (retval != ERROR_OK)
		return retval;

	return cortex_a8_read_dcc(a8, data, &dscr);
}

static int cortex_a8_bpwp_enable(struct arm_dpm *dpm, unsigned index_t,
	uint32_t addr, uint32_t control)
{
	struct cortex_a8_common *a8 = dpm_to_a8(dpm);
	uint32_t vr = a8->armv7a_common.debug_base;
	uint32_t cr = a8->armv7a_common.debug_base;
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

	retval = cortex_a8_dap_write_memap_register_u32(dpm->arm->target,
			vr, addr);
	if (retval != ERROR_OK)
		return retval;
	retval = cortex_a8_dap_write_memap_register_u32(dpm->arm->target,
			cr, control);
	return retval;
}

static int cortex_a8_bpwp_disable(struct arm_dpm *dpm, unsigned index_t)
{
	struct cortex_a8_common *a8 = dpm_to_a8(dpm);
	uint32_t cr;

	switch (index_t) {
		case 0 ... 15:
			cr = a8->armv7a_common.debug_base + CPUDBG_BCR_BASE;
			break;
		case 16 ... 31:
			cr = a8->armv7a_common.debug_base + CPUDBG_WCR_BASE;
			index_t -= 16;
			break;
		default:
			return ERROR_FAIL;
	}
	cr += 4 * index_t;

	LOG_DEBUG("A8: bpwp disable, cr %08x", (unsigned) cr);

	/* clear control register */
	return cortex_a8_dap_write_memap_register_u32(dpm->arm->target, cr, 0);
}

static int cortex_a8_dpm_setup(struct cortex_a8_common *a8, uint32_t didr)
{
	struct arm_dpm *dpm = &a8->armv7a_common.dpm;
	int retval;

	dpm->arm = &a8->armv7a_common.arm;
	dpm->didr = didr;

	dpm->prepare = cortex_a8_dpm_prepare;
	dpm->finish = cortex_a8_dpm_finish;

	dpm->instr_write_data_dcc = cortex_a8_instr_write_data_dcc;
	dpm->instr_write_data_r0 = cortex_a8_instr_write_data_r0;
	dpm->instr_cpsr_sync = cortex_a8_instr_cpsr_sync;

	dpm->instr_read_data_dcc = cortex_a8_instr_read_data_dcc;
	dpm->instr_read_data_r0 = cortex_a8_instr_read_data_r0;

	dpm->bpwp_enable = cortex_a8_bpwp_enable;
	dpm->bpwp_disable = cortex_a8_bpwp_disable;

	retval = arm_dpm_setup(dpm);
	if (retval == ERROR_OK)
		retval = arm_dpm_initialize(dpm);

	return retval;
}
static struct target *get_cortex_a8(struct target *target, int32_t coreid)
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
static int cortex_a8_halt(struct target *target);

static int cortex_a8_halt_smp(struct target *target)
{
	int retval = 0;
	struct target_list *head;
	struct target *curr;
	head = target->head;
	while (head != (struct target_list *)NULL) {
		curr = head->target;
		if ((curr != target) && (curr->state != TARGET_HALTED))
			retval += cortex_a8_halt(curr);
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
		retval += cortex_a8_halt_smp(target);
	}
	return retval;
}

/*
 * Cortex-A8 Run control
 */

static int cortex_a8_poll(struct target *target)
{
	int retval = ERROR_OK;
	uint32_t dscr;
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);
	struct armv7a_common *armv7a = &cortex_a8->armv7a_common;
	struct adiv5_dap *swjdp = armv7a->arm.dap;
	enum target_state prev_target_state = target->state;
	/*  toggle to another core is done by gdb as follow */
	/*  maint packet J core_id */
	/*  continue */
	/*  the next polling trigger an halt event sent to gdb */
	if ((target->state == TARGET_HALTED) && (target->smp) &&
		(target->gdb_service) &&
		(target->gdb_service->target == NULL)) {
		target->gdb_service->target =
			get_cortex_a8(target, target->gdb_service->core[1]);
		target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		return retval;
	}
	retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		return retval;
	cortex_a8->cpudbg_dscr = dscr;

	if (DSCR_RUN_MODE(dscr) == (DSCR_CORE_HALTED | DSCR_CORE_RESTARTED)) {
		if (prev_target_state != TARGET_HALTED) {
			/* We have a halting debug event */
			LOG_DEBUG("Target halted");
			target->state = TARGET_HALTED;
			if ((prev_target_state == TARGET_RUNNING)
				|| (prev_target_state == TARGET_UNKNOWN)
				|| (prev_target_state == TARGET_RESET)) {
				retval = cortex_a8_debug_entry(target);
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

				retval = cortex_a8_debug_entry(target);
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

static int cortex_a8_halt(struct target *target)
{
	int retval = ERROR_OK;
	uint32_t dscr;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct adiv5_dap *swjdp = armv7a->arm.dap;

	/*
	 * Tell the core to be halted by writing DRCR with 0x1
	 * and then wait for the core to be halted.
	 */
	retval = mem_ap_sel_write_atomic_u32(swjdp, armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DRCR, DRCR_HALT);
	if (retval != ERROR_OK)
		return retval;

	/*
	 * enter halting debug mode
	 */
	retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		return retval;

	retval = mem_ap_sel_write_atomic_u32(swjdp, armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DSCR, dscr | DSCR_HALT_DBG_MODE);
	if (retval != ERROR_OK)
		return retval;

	long long then = timeval_ms();
	for (;; ) {
		retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_DSCR, &dscr);
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

static int cortex_a8_internal_restore(struct target *target, int current,
	uint32_t *address, int handle_breakpoints, int debug_execution)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm *arm = &armv7a->arm;
	int retval;
	uint32_t resume_pc;

	if (!debug_execution)
		target_free_all_working_areas(target);

#if 0
	if (debug_execution) {
		/* Disable interrupts */
		/* We disable interrupts in the PRIMASK register instead of
		 * masking with C_MASKINTS,
		 * This is probably the same issue as Cortex-M3 Errata 377493:
		 * C_MASKINTS in parallel with disabled interrupts can cause
		 * local faults to not be taken. */
		buf_set_u32(armv7m->core_cache->reg_list[ARMV7M_PRIMASK].value, 0, 32, 1);
		armv7m->core_cache->reg_list[ARMV7M_PRIMASK].dirty = 1;
		armv7m->core_cache->reg_list[ARMV7M_PRIMASK].valid = 1;

		/* Make sure we are in Thumb mode */
		buf_set_u32(armv7m->core_cache->reg_list[ARMV7M_xPSR].value, 0, 32,
			buf_get_u32(armv7m->core_cache->reg_list[ARMV7M_xPSR].value, 0,
			32) | (1 << 24));
		armv7m->core_cache->reg_list[ARMV7M_xPSR].dirty = 1;
		armv7m->core_cache->reg_list[ARMV7M_xPSR].valid = 1;
	}
#endif

	/* current = 1: continue on current pc, otherwise continue at <address> */
	resume_pc = buf_get_u32(arm->pc->value, 0, 32);
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
	LOG_DEBUG("resume pc = 0x%08" PRIx32, resume_pc);
	buf_set_u32(arm->pc->value, 0, 32, resume_pc);
	arm->pc->dirty = 1;
	arm->pc->valid = 1;
	/* restore dpm_mode at system halt */
	dpm_modeswitch(&armv7a->dpm, ARM_MODE_ANY);
	/* called it now before restoring context because it uses cpu
	 * register r0 for restoring cp15 control register */
	retval = cortex_a8_restore_cp15_control_reg(target);
	if (retval != ERROR_OK)
		return retval;
	retval = cortex_a8_restore_context(target, handle_breakpoints);
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

static int cortex_a8_internal_restart(struct target *target)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm *arm = &armv7a->arm;
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

	retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		return retval;

	if ((dscr & DSCR_INSTR_COMP) == 0)
		LOG_ERROR("DSCR InstrCompl must be set before leaving debug!");

	retval = mem_ap_sel_write_atomic_u32(swjdp, armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DSCR, dscr & ~DSCR_ITR_EN);
	if (retval != ERROR_OK)
		return retval;

	retval = mem_ap_sel_write_atomic_u32(swjdp, armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DRCR, DRCR_RESTART |
			DRCR_CLEAR_EXCEPTIONS);
	if (retval != ERROR_OK)
		return retval;

	long long then = timeval_ms();
	for (;; ) {
		retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_DSCR, &dscr);
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

static int cortex_a8_restore_smp(struct target *target, int handle_breakpoints)
{
	int retval = 0;
	struct target_list *head;
	struct target *curr;
	uint32_t address;
	head = target->head;
	while (head != (struct target_list *)NULL) {
		curr = head->target;
		if ((curr != target) && (curr->state != TARGET_RUNNING)) {
			/*  resume current address , not in step mode */
			retval += cortex_a8_internal_restore(curr, 1, &address,
					handle_breakpoints, 0);
			retval += cortex_a8_internal_restart(curr);
		}
		head = head->next;

	}
	return retval;
}

static int cortex_a8_resume(struct target *target, int current,
	uint32_t address, int handle_breakpoints, int debug_execution)
{
	int retval = 0;
	/* dummy resume for smp toggle in order to reduce gdb impact  */
	if ((target->smp) && (target->gdb_service->core[1] != -1)) {
		/*   simulate a start and halt of target */
		target->gdb_service->target = NULL;
		target->gdb_service->core[0] = target->gdb_service->core[1];
		/*  fake resume at next poll we play the  target core[1], see poll*/
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		return 0;
	}
	cortex_a8_internal_restore(target, current, &address, handle_breakpoints, debug_execution);
	if (target->smp) {
		target->gdb_service->core[0] = -1;
		retval = cortex_a8_restore_smp(target, handle_breakpoints);
		if (retval != ERROR_OK)
			return retval;
	}
	cortex_a8_internal_restart(target);

	if (!debug_execution) {
		target->state = TARGET_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		LOG_DEBUG("target resumed at 0x%" PRIx32, address);
	} else {
		target->state = TARGET_DEBUG_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED);
		LOG_DEBUG("target debug resumed at 0x%" PRIx32, address);
	}

	return ERROR_OK;
}

static int cortex_a8_debug_entry(struct target *target)
{
	int i;
	uint32_t regfile[16], cpsr, dscr;
	int retval = ERROR_OK;
	struct working_area *regfile_working_area = NULL;
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm *arm = &armv7a->arm;
	struct adiv5_dap *swjdp = armv7a->arm.dap;
	struct reg *reg;

	LOG_DEBUG("dscr = 0x%08" PRIx32, cortex_a8->cpudbg_dscr);

	/* REVISIT surely we should not re-read DSCR !! */
	retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		return retval;

	/* REVISIT see A8 TRM 12.11.4 steps 2..3 -- make sure that any
	 * imprecise data aborts get discarded by issuing a Data
	 * Synchronization Barrier:  ARMV4_5_MCR(15, 0, 0, 7, 10, 4).
	 */

	/* Enable the ITR execution once we are in debug mode */
	dscr |= DSCR_ITR_EN;
	retval = mem_ap_sel_write_atomic_u32(swjdp, armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DSCR, dscr);
	if (retval != ERROR_OK)
		return retval;

	/* Examine debug reason */
	arm_dpm_report_dscr(&armv7a->dpm, cortex_a8->cpudbg_dscr);

	/* save address of instruction that triggered the watchpoint? */
	if (target->debug_reason == DBG_REASON_WATCHPOINT) {
		uint32_t wfar;

		retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_WFAR,
				&wfar);
		if (retval != ERROR_OK)
			return retval;
		arm_dpm_report_wfar(&armv7a->dpm, wfar);
	}

	/* REVISIT fast_reg_read is never set ... */

	/* Examine target state and mode */
	if (cortex_a8->fast_reg_read)
		target_alloc_working_area(target, 64, &regfile_working_area);

	/* First load register acessible through core debug port*/
	if (!regfile_working_area)
		retval = arm_dpm_read_current_registers(&armv7a->dpm);
	else {
		retval = cortex_a8_read_regs_through_mem(target,
				regfile_working_area->address, regfile);

		target_free_working_area(target, regfile_working_area);
		if (retval != ERROR_OK)
			return retval;

		/* read Current PSR */
		retval = cortex_a8_dap_read_coreregister_u32(target, &cpsr, 16);
		/*  store current cpsr */
		if (retval != ERROR_OK)
			return retval;

		LOG_DEBUG("cpsr: %8.8" PRIx32, cpsr);

		arm_set_cpsr(arm, cpsr);

		/* update cache */
		for (i = 0; i <= ARM_PC; i++) {
			reg = arm_reg_current(arm, i);

			buf_set_u32(reg->value, 0, 32, regfile[i]);
			reg->valid = 1;
			reg->dirty = 0;
		}

		/* Fixup PC Resume Address */
		if (cpsr & (1 << 5)) {
			/* T bit set for Thumb or ThumbEE state */
			regfile[ARM_PC] -= 4;
		} else {
			/* ARM state */
			regfile[ARM_PC] -= 8;
		}

		reg = arm->pc;
		buf_set_u32(reg->value, 0, 32, regfile[ARM_PC]);
		reg->dirty = reg->valid;
	}

#if 0
/* TODO, Move this */
	uint32_t cp15_control_register, cp15_cacr, cp15_nacr;
	cortex_a8_read_cp(target, &cp15_control_register, 15, 0, 1, 0, 0);
	LOG_DEBUG("cp15_control_register = 0x%08x", cp15_control_register);

	cortex_a8_read_cp(target, &cp15_cacr, 15, 0, 1, 0, 2);
	LOG_DEBUG("cp15 Coprocessor Access Control Register = 0x%08x", cp15_cacr);

	cortex_a8_read_cp(target, &cp15_nacr, 15, 0, 1, 1, 2);
	LOG_DEBUG("cp15 Nonsecure Access Control Register = 0x%08x", cp15_nacr);
#endif

	/* Are we in an exception handler */
/*	armv4_5->exception_number = 0; */
	if (armv7a->post_debug_entry) {
		retval = armv7a->post_debug_entry(target);
		if (retval != ERROR_OK)
			return retval;
	}

	return retval;
}

static int cortex_a8_post_debug_entry(struct target *target)
{
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);
	struct armv7a_common *armv7a = &cortex_a8->armv7a_common;
	int retval;

	/* MRC p15,0,<Rt>,c1,c0,0 ; Read CP15 System Control Register */
	retval = armv7a->arm.mrc(target, 15,
			0, 0,	/* op1, op2 */
			1, 0,	/* CRn, CRm */
			&cortex_a8->cp15_control_reg);
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("cp15_control_reg: %8.8" PRIx32, cortex_a8->cp15_control_reg);
	cortex_a8->cp15_control_reg_curr = cortex_a8->cp15_control_reg;

	if (armv7a->armv7a_mmu.armv7a_cache.ctype == -1)
		armv7a_identify_cache(target);

	if (armv7a->is_armv7r) {
		armv7a->armv7a_mmu.mmu_enabled = 0;
	} else {
		armv7a->armv7a_mmu.mmu_enabled =
			(cortex_a8->cp15_control_reg & 0x1U) ? 1 : 0;
	}
	armv7a->armv7a_mmu.armv7a_cache.d_u_cache_enabled =
		(cortex_a8->cp15_control_reg & 0x4U) ? 1 : 0;
	armv7a->armv7a_mmu.armv7a_cache.i_cache_enabled =
		(cortex_a8->cp15_control_reg & 0x1000U) ? 1 : 0;
	cortex_a8->curr_mode = armv7a->arm.core_mode;

	return ERROR_OK;
}

static int cortex_a8_step(struct target *target, int current, uint32_t address,
	int handle_breakpoints)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm *arm = &armv7a->arm;
	struct breakpoint *breakpoint = NULL;
	struct breakpoint stepbreakpoint;
	struct reg *r;
	int retval;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* current = 1: continue on current pc, otherwise continue at <address> */
	r = arm->pc;
	if (!current)
		buf_set_u32(r->value, 0, 32, address);
	else
		address = buf_get_u32(r->value, 0, 32);

	/* The front-end may request us not to handle breakpoints.
	 * But since Cortex-A8 uses breakpoint for single step,
	 * we MUST handle breakpoints.
	 */
	handle_breakpoints = 1;
	if (handle_breakpoints) {
		breakpoint = breakpoint_find(target, address);
		if (breakpoint)
			cortex_a8_unset_breakpoint(target, breakpoint);
	}

	/* Setup single step breakpoint */
	stepbreakpoint.address = address;
	stepbreakpoint.length = (arm->core_state == ARM_STATE_THUMB)
		? 2 : 4;
	stepbreakpoint.type = BKPT_HARD;
	stepbreakpoint.set = 0;

	/* Break on IVA mismatch */
	cortex_a8_set_breakpoint(target, &stepbreakpoint, 0x04);

	target->debug_reason = DBG_REASON_SINGLESTEP;

	retval = cortex_a8_resume(target, 1, address, 0, 0);
	if (retval != ERROR_OK)
		return retval;

	long long then = timeval_ms();
	while (target->state != TARGET_HALTED) {
		retval = cortex_a8_poll(target);
		if (retval != ERROR_OK)
			return retval;
		if (timeval_ms() > then + 1000) {
			LOG_ERROR("timeout waiting for target halt");
			return ERROR_FAIL;
		}
	}

	cortex_a8_unset_breakpoint(target, &stepbreakpoint);

	target->debug_reason = DBG_REASON_BREAKPOINT;

	if (breakpoint)
		cortex_a8_set_breakpoint(target, breakpoint, 0);

	if (target->state != TARGET_HALTED)
		LOG_DEBUG("target stepped");

	return ERROR_OK;
}

static int cortex_a8_restore_context(struct target *target, bool bpwp)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);

	LOG_DEBUG(" ");

	if (armv7a->pre_restore_context)
		armv7a->pre_restore_context(target);

	return arm_dpm_write_dirty_registers(&armv7a->dpm, bpwp);
}

/*
 * Cortex-A8 Breakpoint and watchpoint functions
 */

/* Setup hardware Breakpoint Register Pair */
static int cortex_a8_set_breakpoint(struct target *target,
	struct breakpoint *breakpoint, uint8_t matchmode)
{
	int retval;
	int brp_i = 0;
	uint32_t control;
	uint8_t byte_addr_select = 0x0F;
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);
	struct armv7a_common *armv7a = &cortex_a8->armv7a_common;
	struct cortex_a8_brp *brp_list = cortex_a8->brp_list;

	if (breakpoint->set) {
		LOG_WARNING("breakpoint already set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD) {
		while (brp_list[brp_i].used && (brp_i < cortex_a8->brp_num))
			brp_i++;
		if (brp_i >= cortex_a8->brp_num) {
			LOG_ERROR("ERROR Can not find free Breakpoint Register Pair");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		breakpoint->set = brp_i + 1;
		if (breakpoint->length == 2)
			byte_addr_select = (3 << (breakpoint->address & 0x02));
		control = ((matchmode & 0x7) << 20)
			| (byte_addr_select << 5)
			| (3 << 1) | 1;
		brp_list[brp_i].used = 1;
		brp_list[brp_i].value = (breakpoint->address & 0xFFFFFFFC);
		brp_list[brp_i].control = control;
		retval = cortex_a8_dap_write_memap_register_u32(target, armv7a->debug_base
				+ CPUDBG_BVR_BASE + 4 * brp_list[brp_i].BRPn,
				brp_list[brp_i].value);
		if (retval != ERROR_OK)
			return retval;
		retval = cortex_a8_dap_write_memap_register_u32(target, armv7a->debug_base
				+ CPUDBG_BCR_BASE + 4 * brp_list[brp_i].BRPn,
				brp_list[brp_i].control);
		if (retval != ERROR_OK)
			return retval;
		LOG_DEBUG("brp %i control 0x%0" PRIx32 " value 0x%0" PRIx32, brp_i,
			brp_list[brp_i].control,
			brp_list[brp_i].value);
	} else if (breakpoint->type == BKPT_SOFT) {
		uint8_t code[4];
		if (breakpoint->length == 2)
			buf_set_u32(code, 0, 32, ARMV5_T_BKPT(0x11));
		else
			buf_set_u32(code, 0, 32, ARMV5_BKPT(0x11));
		retval = target_read_memory(target,
				breakpoint->address & 0xFFFFFFFE,
				breakpoint->length, 1,
				breakpoint->orig_instr);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_memory(target,
				breakpoint->address & 0xFFFFFFFE,
				breakpoint->length, 1, code);
		if (retval != ERROR_OK)
			return retval;
		breakpoint->set = 0x11;	/* Any nice value but 0 */
	}

	return ERROR_OK;
}

static int cortex_a8_set_context_breakpoint(struct target *target,
	struct breakpoint *breakpoint, uint8_t matchmode)
{
	int retval = ERROR_FAIL;
	int brp_i = 0;
	uint32_t control;
	uint8_t byte_addr_select = 0x0F;
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);
	struct armv7a_common *armv7a = &cortex_a8->armv7a_common;
	struct cortex_a8_brp *brp_list = cortex_a8->brp_list;

	if (breakpoint->set) {
		LOG_WARNING("breakpoint already set");
		return retval;
	}
	/*check available context BRPs*/
	while ((brp_list[brp_i].used ||
		(brp_list[brp_i].type != BRP_CONTEXT)) && (brp_i < cortex_a8->brp_num))
		brp_i++;

	if (brp_i >= cortex_a8->brp_num) {
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
	retval = cortex_a8_dap_write_memap_register_u32(target, armv7a->debug_base
			+ CPUDBG_BVR_BASE + 4 * brp_list[brp_i].BRPn,
			brp_list[brp_i].value);
	if (retval != ERROR_OK)
		return retval;
	retval = cortex_a8_dap_write_memap_register_u32(target, armv7a->debug_base
			+ CPUDBG_BCR_BASE + 4 * brp_list[brp_i].BRPn,
			brp_list[brp_i].control);
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("brp %i control 0x%0" PRIx32 " value 0x%0" PRIx32, brp_i,
		brp_list[brp_i].control,
		brp_list[brp_i].value);
	return ERROR_OK;

}

static int cortex_a8_set_hybrid_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	int retval = ERROR_FAIL;
	int brp_1 = 0;	/* holds the contextID pair */
	int brp_2 = 0;	/* holds the IVA pair */
	uint32_t control_CTX, control_IVA;
	uint8_t CTX_byte_addr_select = 0x0F;
	uint8_t IVA_byte_addr_select = 0x0F;
	uint8_t CTX_machmode = 0x03;
	uint8_t IVA_machmode = 0x01;
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);
	struct armv7a_common *armv7a = &cortex_a8->armv7a_common;
	struct cortex_a8_brp *brp_list = cortex_a8->brp_list;

	if (breakpoint->set) {
		LOG_WARNING("breakpoint already set");
		return retval;
	}
	/*check available context BRPs*/
	while ((brp_list[brp_1].used ||
		(brp_list[brp_1].type != BRP_CONTEXT)) && (brp_1 < cortex_a8->brp_num))
		brp_1++;

	printf("brp(CTX) found num: %d\n", brp_1);
	if (brp_1 >= cortex_a8->brp_num) {
		LOG_ERROR("ERROR Can not find free Breakpoint Register Pair");
		return ERROR_FAIL;
	}

	while ((brp_list[brp_2].used ||
		(brp_list[brp_2].type != BRP_NORMAL)) && (brp_2 < cortex_a8->brp_num))
		brp_2++;

	printf("brp(IVA) found num: %d\n", brp_2);
	if (brp_2 >= cortex_a8->brp_num) {
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
	retval = cortex_a8_dap_write_memap_register_u32(target, armv7a->debug_base
			+ CPUDBG_BVR_BASE + 4 * brp_list[brp_1].BRPn,
			brp_list[brp_1].value);
	if (retval != ERROR_OK)
		return retval;
	retval = cortex_a8_dap_write_memap_register_u32(target, armv7a->debug_base
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
	retval = cortex_a8_dap_write_memap_register_u32(target, armv7a->debug_base
			+ CPUDBG_BVR_BASE + 4 * brp_list[brp_2].BRPn,
			brp_list[brp_2].value);
	if (retval != ERROR_OK)
		return retval;
	retval = cortex_a8_dap_write_memap_register_u32(target, armv7a->debug_base
			+ CPUDBG_BCR_BASE + 4 * brp_list[brp_2].BRPn,
			brp_list[brp_2].control);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int cortex_a8_unset_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	int retval;
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);
	struct armv7a_common *armv7a = &cortex_a8->armv7a_common;
	struct cortex_a8_brp *brp_list = cortex_a8->brp_list;

	if (!breakpoint->set) {
		LOG_WARNING("breakpoint not set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD) {
		if ((breakpoint->address != 0) && (breakpoint->asid != 0)) {
			int brp_i = breakpoint->set - 1;
			int brp_j = breakpoint->linked_BRP;
			if ((brp_i < 0) || (brp_i >= cortex_a8->brp_num)) {
				LOG_DEBUG("Invalid BRP number in breakpoint");
				return ERROR_OK;
			}
			LOG_DEBUG("rbp %i control 0x%0" PRIx32 " value 0x%0" PRIx32, brp_i,
				brp_list[brp_i].control, brp_list[brp_i].value);
			brp_list[brp_i].used = 0;
			brp_list[brp_i].value = 0;
			brp_list[brp_i].control = 0;
			retval = cortex_a8_dap_write_memap_register_u32(target, armv7a->debug_base
					+ CPUDBG_BCR_BASE + 4 * brp_list[brp_i].BRPn,
					brp_list[brp_i].control);
			if (retval != ERROR_OK)
				return retval;
			retval = cortex_a8_dap_write_memap_register_u32(target, armv7a->debug_base
					+ CPUDBG_BVR_BASE + 4 * brp_list[brp_i].BRPn,
					brp_list[brp_i].value);
			if (retval != ERROR_OK)
				return retval;
			if ((brp_j < 0) || (brp_j >= cortex_a8->brp_num)) {
				LOG_DEBUG("Invalid BRP number in breakpoint");
				return ERROR_OK;
			}
			LOG_DEBUG("rbp %i control 0x%0" PRIx32 " value 0x%0" PRIx32, brp_j,
				brp_list[brp_j].control, brp_list[brp_j].value);
			brp_list[brp_j].used = 0;
			brp_list[brp_j].value = 0;
			brp_list[brp_j].control = 0;
			retval = cortex_a8_dap_write_memap_register_u32(target, armv7a->debug_base
					+ CPUDBG_BCR_BASE + 4 * brp_list[brp_j].BRPn,
					brp_list[brp_j].control);
			if (retval != ERROR_OK)
				return retval;
			retval = cortex_a8_dap_write_memap_register_u32(target, armv7a->debug_base
					+ CPUDBG_BVR_BASE + 4 * brp_list[brp_j].BRPn,
					brp_list[brp_j].value);
			if (retval != ERROR_OK)
				return retval;
			breakpoint->linked_BRP = 0;
			breakpoint->set = 0;
			return ERROR_OK;

		} else {
			int brp_i = breakpoint->set - 1;
			if ((brp_i < 0) || (brp_i >= cortex_a8->brp_num)) {
				LOG_DEBUG("Invalid BRP number in breakpoint");
				return ERROR_OK;
			}
			LOG_DEBUG("rbp %i control 0x%0" PRIx32 " value 0x%0" PRIx32, brp_i,
				brp_list[brp_i].control, brp_list[brp_i].value);
			brp_list[brp_i].used = 0;
			brp_list[brp_i].value = 0;
			brp_list[brp_i].control = 0;
			retval = cortex_a8_dap_write_memap_register_u32(target, armv7a->debug_base
					+ CPUDBG_BCR_BASE + 4 * brp_list[brp_i].BRPn,
					brp_list[brp_i].control);
			if (retval != ERROR_OK)
				return retval;
			retval = cortex_a8_dap_write_memap_register_u32(target, armv7a->debug_base
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
					breakpoint->address & 0xFFFFFFFE,
					4, 1, breakpoint->orig_instr);
			if (retval != ERROR_OK)
				return retval;
		} else {
			retval = target_write_memory(target,
					breakpoint->address & 0xFFFFFFFE,
					2, 1, breakpoint->orig_instr);
			if (retval != ERROR_OK)
				return retval;
		}
	}
	breakpoint->set = 0;

	return ERROR_OK;
}

static int cortex_a8_add_breakpoint(struct target *target,
	struct breakpoint *breakpoint)
{
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);

	if ((breakpoint->type == BKPT_HARD) && (cortex_a8->brp_num_available < 1)) {
		LOG_INFO("no hardware breakpoint available");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (breakpoint->type == BKPT_HARD)
		cortex_a8->brp_num_available--;

	return cortex_a8_set_breakpoint(target, breakpoint, 0x00);	/* Exact match */
}

static int cortex_a8_add_context_breakpoint(struct target *target,
	struct breakpoint *breakpoint)
{
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);

	if ((breakpoint->type == BKPT_HARD) && (cortex_a8->brp_num_available < 1)) {
		LOG_INFO("no hardware breakpoint available");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (breakpoint->type == BKPT_HARD)
		cortex_a8->brp_num_available--;

	return cortex_a8_set_context_breakpoint(target, breakpoint, 0x02);	/* asid match */
}

static int cortex_a8_add_hybrid_breakpoint(struct target *target,
	struct breakpoint *breakpoint)
{
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);

	if ((breakpoint->type == BKPT_HARD) && (cortex_a8->brp_num_available < 1)) {
		LOG_INFO("no hardware breakpoint available");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (breakpoint->type == BKPT_HARD)
		cortex_a8->brp_num_available--;

	return cortex_a8_set_hybrid_breakpoint(target, breakpoint);	/* ??? */
}


static int cortex_a8_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);

#if 0
/* It is perfectly possible to remove breakpoints while the target is running */
	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
#endif

	if (breakpoint->set) {
		cortex_a8_unset_breakpoint(target, breakpoint);
		if (breakpoint->type == BKPT_HARD)
			cortex_a8->brp_num_available++;
	}


	return ERROR_OK;
}

/*
 * Cortex-A8 Reset functions
 */

static int cortex_a8_assert_reset(struct target *target)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);

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
	register_cache_invalidate(armv7a->arm.core_cache);

	target->state = TARGET_RESET;

	return ERROR_OK;
}

static int cortex_a8_deassert_reset(struct target *target)
{
	int retval;

	LOG_DEBUG(" ");

	/* be certain SRST is off */
	jtag_add_reset(0, 0);

	retval = cortex_a8_poll(target);
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

static int cortex_a8_write_apb_ab_memory(struct target *target,
	uint32_t address, uint32_t size,
	uint32_t count, const uint8_t *buffer)
{
	/* write memory through APB-AP */

	int retval = ERROR_COMMAND_SYNTAX_ERROR;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm *arm = &armv7a->arm;
	struct adiv5_dap *swjdp = armv7a->arm.dap;
	int total_bytes = count * size;
	int total_u32;
	int start_byte = address & 0x3;
	int end_byte   = (address + total_bytes) & 0x3;
	struct reg *reg;
	uint32_t dscr;
	uint8_t *tmp_buff = NULL;

	LOG_DEBUG("Writing APB-AP memory address 0x%" PRIx32 " size %"  PRIu32 " count%"  PRIu32,
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
	reg = arm_reg_current(arm, 0);
	reg->dirty = true;

	/*  clear any abort  */
	retval = mem_ap_sel_write_atomic_u32(swjdp, armv7a->debug_ap, armv7a->debug_base + CPUDBG_DRCR, 1<<2);
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
		retval = cortex_a8_read_apb_ab_memory(target, (address & ~0x3), 4, 1, tmp_buff);
		if (retval != ERROR_OK)
			goto error_free_buff_w;
	}

	/* If end of write is not aligned, or the write is less than 4 bytes */
	if ((end_byte != 0) ||
		((total_u32 == 1) && (total_bytes != 4))) {

		/* Read the last word to avoid corruption during 32 bit write */
		int mem_offset = (total_u32-1) * 4;
		retval = cortex_a8_read_apb_ab_memory(target, (address & ~0x3) + mem_offset, 4, 1, &tmp_buff[mem_offset]);
		if (retval != ERROR_OK)
			goto error_free_buff_w;
	}

	/* Copy the write buffer over the top of the temporary buffer */
	memcpy(&tmp_buff[start_byte], buffer, total_bytes);

	/* We now have a 32 bit aligned buffer that can be written */

	/* Read DSCR */
	retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		goto error_free_buff_w;

	/* Set DTR mode to Fast (2) */
	dscr = (dscr & ~DSCR_EXT_DCC_MASK) | DSCR_EXT_DCC_FAST_MODE;
	retval = mem_ap_sel_write_atomic_u32(swjdp, armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DSCR, dscr);
	if (retval != ERROR_OK)
		goto error_free_buff_w;

	/* Copy the destination address into R0 */
	/*  - pend an instruction  MRC p14, 0, R0, c5, c0 */
	retval = mem_ap_sel_write_atomic_u32(swjdp, armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_ITR, ARMV4_5_MRC(14, 0, 0, 0, 5, 0));
	if (retval != ERROR_OK)
		goto error_unset_dtr_w;
	/* Write address into DTRRX, which triggers previous instruction */
	retval = mem_ap_sel_write_atomic_u32(swjdp, armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_DTRRX, address & (~0x3));
	if (retval != ERROR_OK)
		goto error_unset_dtr_w;

	/* Write the data transfer instruction into the ITR
	 * (STC p14, c5, [R0], 4)
	 */
	retval = mem_ap_sel_write_atomic_u32(swjdp, armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_ITR, ARMV4_5_STC(0, 1, 0, 1, 14, 5, 0, 4));
	if (retval != ERROR_OK)
		goto error_unset_dtr_w;

	/* Do the write */
	retval = mem_ap_sel_write_buf_noincr(swjdp, armv7a->debug_ap,
					tmp_buff, 4, total_u32, armv7a->debug_base + CPUDBG_DTRRX);
	if (retval != ERROR_OK)
		goto error_unset_dtr_w;


	/* Switch DTR mode back to non-blocking (0) */
	dscr = (dscr & ~DSCR_EXT_DCC_MASK) | DSCR_EXT_DCC_NON_BLOCKING;
	retval = mem_ap_sel_write_atomic_u32(swjdp, armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_DSCR, dscr);
	if (retval != ERROR_OK)
		goto error_unset_dtr_w;

    /* Check for sticky abort flags in the DSCR */
	retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		goto error_free_buff_w;
	if (dscr & (DSCR_STICKY_ABORT_PRECISE | DSCR_STICKY_ABORT_IMPRECISE)) {
		/* Abort occurred - clear it and exit */
		LOG_ERROR("abort occurred - dscr = 0x%08" PRIx32, dscr);
		mem_ap_sel_write_atomic_u32(swjdp, armv7a->debug_ap,
					armv7a->debug_base + CPUDBG_DRCR, 1<<2);
		goto error_free_buff_w;
	}

	/* Done */
	free(tmp_buff);
	return ERROR_OK;

error_unset_dtr_w:
	/* Unset DTR mode */
	mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_DSCR, &dscr);
	dscr = (dscr & ~DSCR_EXT_DCC_MASK) | DSCR_EXT_DCC_NON_BLOCKING;
	mem_ap_sel_write_atomic_u32(swjdp, armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_DSCR, dscr);
error_free_buff_w:
	LOG_ERROR("error");
	free(tmp_buff);
	return ERROR_FAIL;
}

static int cortex_a8_read_apb_ab_memory(struct target *target,
	uint32_t address, uint32_t size,
	uint32_t count, uint8_t *buffer)
{
	/* read memory through APB-AP */

	int retval = ERROR_COMMAND_SYNTAX_ERROR;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct adiv5_dap *swjdp = armv7a->arm.dap;
	struct arm *arm = &armv7a->arm;
	int total_bytes = count * size;
	int total_u32;
	int start_byte = address & 0x3;
	int end_byte   = (address + total_bytes) & 0x3;
	struct reg *reg;
	uint32_t dscr;
	uint8_t *tmp_buff = NULL;
	uint8_t buf[8];
	uint8_t *u8buf_ptr;

	LOG_DEBUG("Reading APB-AP memory address 0x%" PRIx32 " size %"  PRIu32 " count%"  PRIu32,
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
	reg = arm_reg_current(arm, 0);
	reg->dirty = true;

	/*  clear any abort  */
	retval =
		mem_ap_sel_write_atomic_u32(swjdp, armv7a->debug_ap, armv7a->debug_base + CPUDBG_DRCR, 1<<2);
	if (retval != ERROR_OK)
		goto error_free_buff_r;

	/* Read DSCR */
	retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DSCR, &dscr);

	/* This algorithm comes from either :
	 * Cortex-A8 TRM Example 12-24
	 * Cortex-R4 TRM Example 11-25
	 * (slight differences)
	 */

	/* Set DTR access mode to stall mode b01  */
	dscr = (dscr & ~DSCR_EXT_DCC_MASK) | DSCR_EXT_DCC_STALL_MODE;
	retval +=  mem_ap_sel_write_atomic_u32(swjdp, armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DSCR, dscr);

	/* Write R0 with value 'address' using write procedure for stall mode */
	/*   - Write the address for read access into DTRRX */
	retval += mem_ap_sel_write_atomic_u32(swjdp, armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DTRRX, address & ~0x3);
	/*  - Copy value from DTRRX to R0 using instruction mrc p14, 0, r0, c5, c0 */
	cortex_a8_exec_opcode(target, ARMV4_5_MRC(14, 0, 0, 0, 5, 0), &dscr);

	/* Write the data transfer instruction (ldc p14, c5, [r0],4)
	 * and the DTR mode setting to fast mode
	 * in one combined write (since they are adjacent registers)
	 */
	u8buf_ptr = buf;
	target_buffer_set_u32(target, u8buf_ptr, ARMV4_5_LDC(0, 1, 0, 1, 14, 5, 0, 4));
	dscr = (dscr & ~DSCR_EXT_DCC_MASK) | DSCR_EXT_DCC_FAST_MODE;
	target_buffer_set_u32(target, u8buf_ptr + 4, dscr);
	/*  group the 2 access CPUDBG_ITR 0x84 and CPUDBG_DSCR 0x88 */
	retval += mem_ap_sel_write_buf(swjdp, armv7a->debug_ap, u8buf_ptr, 4, 2,
			armv7a->debug_base + CPUDBG_ITR);
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
		/* address and read length are aligned so read directely into the passed buffer */
		u8buf_ptr = buffer;

	/* Read the data - Each read of the DTRTX register causes the instruction to be reissued
	 * Abort flags are sticky, so can be read at end of transactions
	 *
	 * This data is read in aligned to 32 bit boundary.
	 */
	retval = mem_ap_sel_read_buf_noincr(swjdp, armv7a->debug_ap, u8buf_ptr, 4, total_u32,
									armv7a->debug_base + CPUDBG_DTRTX);
	if (retval != ERROR_OK)
			goto error_unset_dtr_r;

	/* set DTR access mode back to non blocking b00  */
	dscr = (dscr & ~DSCR_EXT_DCC_MASK) | DSCR_EXT_DCC_NON_BLOCKING;
	retval =  mem_ap_sel_write_atomic_u32(swjdp, armv7a->debug_ap,
					armv7a->debug_base + CPUDBG_DSCR, dscr);
	if (retval != ERROR_OK)
		goto error_free_buff_r;

	/* Wait for the final read instruction to finish */
	do {
		retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
					armv7a->debug_base + CPUDBG_DSCR, &dscr);
		if (retval != ERROR_OK)
			goto error_free_buff_r;
	} while ((dscr & DSCR_INSTR_COMP) == 0);

	/* Check for sticky abort flags in the DSCR */
	retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_DSCR, &dscr);
	if (retval != ERROR_OK)
		goto error_free_buff_r;
	if (dscr & (DSCR_STICKY_ABORT_PRECISE | DSCR_STICKY_ABORT_IMPRECISE)) {
		/* Abort occurred - clear it and exit */
		LOG_ERROR("abort occurred - dscr = 0x%08" PRIx32, dscr);
		mem_ap_sel_write_atomic_u32(swjdp, armv7a->debug_ap,
					armv7a->debug_base + CPUDBG_DRCR, 1<<2);
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
	mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_DSCR, &dscr);
	dscr = (dscr & ~DSCR_EXT_DCC_MASK) | DSCR_EXT_DCC_NON_BLOCKING;
	mem_ap_sel_write_atomic_u32(swjdp, armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_DSCR, dscr);
error_free_buff_r:
	LOG_ERROR("error");
	free(tmp_buff);
	return ERROR_FAIL;
}


/*
 * Cortex-A8 Memory access
 *
 * This is same Cortex M3 but we must also use the correct
 * ap number for every access.
 */

static int cortex_a8_read_phys_memory(struct target *target,
	uint32_t address, uint32_t size,
	uint32_t count, uint8_t *buffer)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct adiv5_dap *swjdp = armv7a->arm.dap;
	int retval = ERROR_COMMAND_SYNTAX_ERROR;
	uint8_t apsel = swjdp->apsel;
	LOG_DEBUG("Reading memory at real address 0x%" PRIx32 "; size %" PRId32 "; count %" PRId32,
		address, size, count);

	if (count && buffer) {

		if (armv7a->memory_ap_available && (apsel == armv7a->memory_ap)) {

			/* read memory through AHB-AP */
			retval = mem_ap_sel_read_buf(swjdp, armv7a->memory_ap, buffer, size, count, address);
		} else {

			/* read memory through APB-AP */
			if (!armv7a->is_armv7r) {
				/*  disable mmu */
				retval = cortex_a8_mmu_modify(target, 0);
				if (retval != ERROR_OK)
					return retval;
			}
			retval = cortex_a8_read_apb_ab_memory(target, address, size, count, buffer);
		}
	}
	return retval;
}

static int cortex_a8_read_memory(struct target *target, uint32_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	int mmu_enabled = 0;
	uint32_t virt, phys;
	int retval;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct adiv5_dap *swjdp = armv7a->arm.dap;
	uint8_t apsel = swjdp->apsel;

	/* cortex_a8 handles unaligned memory access */
	LOG_DEBUG("Reading memory at address 0x%" PRIx32 "; size %" PRId32 "; count %" PRId32, address,
		size, count);

	/* determine if MMU was enabled on target stop */
	if (!armv7a->is_armv7r) {
		retval = cortex_a8_mmu(target, &mmu_enabled);
		if (retval != ERROR_OK)
			return retval;
	}

	if (armv7a->memory_ap_available && (apsel == armv7a->memory_ap)) {
		if (mmu_enabled) {
			virt = address;
			retval = cortex_a8_virt2phys(target, virt, &phys);
			if (retval != ERROR_OK)
				return retval;

			LOG_DEBUG("Reading at virtual address. Translating v:0x%" PRIx32 " to r:0x%" PRIx32,
				  virt, phys);
			address = phys;
		}
		retval = cortex_a8_read_phys_memory(target, address, size, count, buffer);
	} else {
		if (mmu_enabled) {
			retval = cortex_a8_check_address(target, address);
			if (retval != ERROR_OK)
				return retval;
			/* enable MMU as we could have disabled it for phys access */
			retval = cortex_a8_mmu_modify(target, 1);
			if (retval != ERROR_OK)
				return retval;
		}
		retval = cortex_a8_read_apb_ab_memory(target, address, size, count, buffer);
	}
	return retval;
}

static int cortex_a8_write_phys_memory(struct target *target,
	uint32_t address, uint32_t size,
	uint32_t count, const uint8_t *buffer)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct adiv5_dap *swjdp = armv7a->arm.dap;
	int retval = ERROR_COMMAND_SYNTAX_ERROR;
	uint8_t apsel = swjdp->apsel;

	LOG_DEBUG("Writing memory to real address 0x%" PRIx32 "; size %" PRId32 "; count %" PRId32, address,
		size, count);

	if (count && buffer) {

		if (armv7a->memory_ap_available && (apsel == armv7a->memory_ap)) {

			/* write memory through AHB-AP */
			retval = mem_ap_sel_write_buf(swjdp, armv7a->memory_ap, buffer, size, count, address);
		} else {

			/* write memory through APB-AP */
			if (!armv7a->is_armv7r) {
				retval = cortex_a8_mmu_modify(target, 0);
				if (retval != ERROR_OK)
					return retval;
			}
			return cortex_a8_write_apb_ab_memory(target, address, size, count, buffer);
		}
	}


	/* REVISIT this op is generic ARMv7-A/R stuff */
	if (retval == ERROR_OK && target->state == TARGET_HALTED) {
		struct arm_dpm *dpm = armv7a->arm.dpm;

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
		if (armv7a->armv7a_mmu.armv7a_cache.i_cache_enabled) {
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
		if (armv7a->armv7a_mmu.armv7a_cache.d_u_cache_enabled) {
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

static int cortex_a8_write_memory(struct target *target, uint32_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer)
{
	int mmu_enabled = 0;
	uint32_t virt, phys;
	int retval;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct adiv5_dap *swjdp = armv7a->arm.dap;
	uint8_t apsel = swjdp->apsel;

	/* cortex_a8 handles unaligned memory access */
	LOG_DEBUG("Writing memory at address 0x%" PRIx32 "; size %" PRId32 "; count %" PRId32, address,
		size, count);

	/* determine if MMU was enabled on target stop */
	if (!armv7a->is_armv7r) {
		retval = cortex_a8_mmu(target, &mmu_enabled);
		if (retval != ERROR_OK)
			return retval;
	}

	if (armv7a->memory_ap_available && (apsel == armv7a->memory_ap)) {
		LOG_DEBUG("Writing memory to address 0x%" PRIx32 "; size %" PRId32 "; count %" PRId32, address, size,
			count);
		if (mmu_enabled) {
			virt = address;
			retval = cortex_a8_virt2phys(target, virt, &phys);
			if (retval != ERROR_OK)
				return retval;

			LOG_DEBUG("Writing to virtual address. Translating v:0x%" PRIx32 " to r:0x%" PRIx32,
				  virt,
				  phys);
			address = phys;
		}
		retval = cortex_a8_write_phys_memory(target, address, size,
				count, buffer);
	} else {
		if (mmu_enabled) {
			retval = cortex_a8_check_address(target, address);
			if (retval != ERROR_OK)
				return retval;
			/* enable MMU as we could have disabled it for phys access */
			retval = cortex_a8_mmu_modify(target, 1);
			if (retval != ERROR_OK)
				return retval;
		}
		retval = cortex_a8_write_apb_ab_memory(target, address, size, count, buffer);
	}
	return retval;
}

static int cortex_a8_handle_target_request(void *priv)
{
	struct target *target = priv;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct adiv5_dap *swjdp = armv7a->arm.dap;
	int retval;

	if (!target_was_examined(target))
		return ERROR_OK;
	if (!target->dbg_msg_enabled)
		return ERROR_OK;

	if (target->state == TARGET_RUNNING) {
		uint32_t request;
		uint32_t dscr;
		retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
				armv7a->debug_base + CPUDBG_DSCR, &dscr);

		/* check if we have data */
		while ((dscr & DSCR_DTR_TX_FULL) && (retval == ERROR_OK)) {
			retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
					armv7a->debug_base + CPUDBG_DTRTX, &request);
			if (retval == ERROR_OK) {
				target_request(target, request);
				retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
						armv7a->debug_base + CPUDBG_DSCR, &dscr);
			}
		}
	}

	return ERROR_OK;
}

/*
 * Cortex-A8 target information and configuration
 */

static int cortex_a8_examine_first(struct target *target)
{
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);
	struct armv7a_common *armv7a = &cortex_a8->armv7a_common;
	struct adiv5_dap *swjdp = armv7a->arm.dap;
	int i;
	int retval = ERROR_OK;
	uint32_t didr, ctypr, ttypr, cpuid;

	/* We do one extra read to ensure DAP is configured,
	 * we call ahbap_debugport_init(swjdp) instead
	 */
	retval = ahbap_debugport_init(swjdp);
	if (retval != ERROR_OK)
		return retval;

	/* Search for the APB-AB - it is needed for access to debug registers */
	retval = dap_find_ap(swjdp, AP_TYPE_APB_AP, &armv7a->debug_ap);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not find APB-AP for debug access");
		return retval;
	}
	/* Search for the AHB-AB */
	retval = dap_find_ap(swjdp, AP_TYPE_AHB_AP, &armv7a->memory_ap);
	if (retval != ERROR_OK) {
		/* AHB-AP not found - use APB-AP */
		LOG_DEBUG("Could not find AHB-AP - using APB-AP for memory access");
		armv7a->memory_ap_available = false;
	} else {
		armv7a->memory_ap_available = true;
	}


	if (!target->dbgbase_set) {
		uint32_t dbgbase;
		/* Get ROM Table base */
		uint32_t apid;
		retval = dap_get_debugbase(swjdp, 1, &dbgbase, &apid);
		if (retval != ERROR_OK)
			return retval;
		/* Lookup 0x15 -- Processor DAP */
		retval = dap_lookup_cs_component(swjdp, 1, dbgbase, 0x15,
				&armv7a->debug_base);
		if (retval != ERROR_OK)
			return retval;
	} else
		armv7a->debug_base = target->dbgbase;

	retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_CPUID, &cpuid);
	if (retval != ERROR_OK)
		return retval;

	retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_CPUID, &cpuid);
	if (retval != ERROR_OK) {
		LOG_DEBUG("Examine %s failed", "CPUID");
		return retval;
	}

	retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_CTYPR, &ctypr);
	if (retval != ERROR_OK) {
		LOG_DEBUG("Examine %s failed", "CTYPR");
		return retval;
	}

	retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_TTYPR, &ttypr);
	if (retval != ERROR_OK) {
		LOG_DEBUG("Examine %s failed", "TTYPR");
		return retval;
	}

	retval = mem_ap_sel_read_atomic_u32(swjdp, armv7a->debug_ap,
			armv7a->debug_base + CPUDBG_DIDR, &didr);
	if (retval != ERROR_OK) {
		LOG_DEBUG("Examine %s failed", "DIDR");
		return retval;
	}

	LOG_DEBUG("cpuid = 0x%08" PRIx32, cpuid);
	LOG_DEBUG("ctypr = 0x%08" PRIx32, ctypr);
	LOG_DEBUG("ttypr = 0x%08" PRIx32, ttypr);
	LOG_DEBUG("didr = 0x%08" PRIx32, didr);

	armv7a->arm.core_type = ARM_MODE_MON;
	retval = cortex_a8_dpm_setup(cortex_a8, didr);
	if (retval != ERROR_OK)
		return retval;

	/* Setup Breakpoint Register Pairs */
	cortex_a8->brp_num = ((didr >> 24) & 0x0F) + 1;
	cortex_a8->brp_num_context = ((didr >> 20) & 0x0F) + 1;
	cortex_a8->brp_num_available = cortex_a8->brp_num;
	cortex_a8->brp_list = calloc(cortex_a8->brp_num, sizeof(struct cortex_a8_brp));
/*	cortex_a8->brb_enabled = ????; */
	for (i = 0; i < cortex_a8->brp_num; i++) {
		cortex_a8->brp_list[i].used = 0;
		if (i < (cortex_a8->brp_num-cortex_a8->brp_num_context))
			cortex_a8->brp_list[i].type = BRP_NORMAL;
		else
			cortex_a8->brp_list[i].type = BRP_CONTEXT;
		cortex_a8->brp_list[i].value = 0;
		cortex_a8->brp_list[i].control = 0;
		cortex_a8->brp_list[i].BRPn = i;
	}

	LOG_DEBUG("Configured %i hw breakpoints", cortex_a8->brp_num);

	target_set_examined(target);
	return ERROR_OK;
}

static int cortex_a8_examine(struct target *target)
{
	int retval = ERROR_OK;

	/* don't re-probe hardware after each reset */
	if (!target_was_examined(target))
		retval = cortex_a8_examine_first(target);

	/* Configure core debug access */
	if (retval == ERROR_OK)
		retval = cortex_a8_init_debug_access(target);

	return retval;
}

/*
 *	Cortex-A8 target creation and initialization
 */

static int cortex_a8_init_target(struct command_context *cmd_ctx,
	struct target *target)
{
	/* examine_first() does a bunch of this */
	return ERROR_OK;
}

static int cortex_a8_init_arch_info(struct target *target,
	struct cortex_a8_common *cortex_a8, struct jtag_tap *tap)
{
	struct armv7a_common *armv7a = &cortex_a8->armv7a_common;
	struct adiv5_dap *dap = &armv7a->dap;

	armv7a->arm.dap = dap;

	/* Setup struct cortex_a8_common */
	cortex_a8->common_magic = CORTEX_A8_COMMON_MAGIC;
	/*  tap has no dap initialized */
	if (!tap->dap) {
		armv7a->arm.dap = dap;
		/* Setup struct cortex_a8_common */

		/* prepare JTAG information for the new target */
		cortex_a8->jtag_info.tap = tap;
		cortex_a8->jtag_info.scann_size = 4;

		/* Leave (only) generic DAP stuff for debugport_init() */
		dap->jtag_info = &cortex_a8->jtag_info;

		/* Number of bits for tar autoincrement, impl. dep. at least 10 */
		dap->tar_autoincr_block = (1 << 10);
		dap->memaccess_tck = 80;
		tap->dap = dap;
	} else
		armv7a->arm.dap = tap->dap;

	cortex_a8->fast_reg_read = 0;

	/* register arch-specific functions */
	armv7a->examine_debug_reason = NULL;

	armv7a->post_debug_entry = cortex_a8_post_debug_entry;

	armv7a->pre_restore_context = NULL;

	armv7a->armv7a_mmu.read_physical_memory = cortex_a8_read_phys_memory;


/*	arm7_9->handle_target_request = cortex_a8_handle_target_request; */

	/* REVISIT v7a setup should be in a v7a-specific routine */
	armv7a_init_arch_info(target, armv7a);
	target_register_timer_callback(cortex_a8_handle_target_request, 1, 1, target);

	return ERROR_OK;
}

static int cortex_a8_target_create(struct target *target, Jim_Interp *interp)
{
	struct cortex_a8_common *cortex_a8 = calloc(1, sizeof(struct cortex_a8_common));

	cortex_a8->armv7a_common.is_armv7r = false;

	return cortex_a8_init_arch_info(target, cortex_a8, target->tap);
}

static int cortex_r4_target_create(struct target *target, Jim_Interp *interp)
{
	struct cortex_a8_common *cortex_a8 = calloc(1, sizeof(struct cortex_a8_common));

	cortex_a8->armv7a_common.is_armv7r = true;

	return cortex_a8_init_arch_info(target, cortex_a8, target->tap);
}


static int cortex_a8_mmu(struct target *target, int *enabled)
{
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("%s: target not halted", __func__);
		return ERROR_TARGET_INVALID;
	}

	*enabled = target_to_cortex_a8(target)->armv7a_common.armv7a_mmu.mmu_enabled;
	return ERROR_OK;
}

static int cortex_a8_virt2phys(struct target *target,
	uint32_t virt, uint32_t *phys)
{
	int retval = ERROR_FAIL;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct adiv5_dap *swjdp = armv7a->arm.dap;
	uint8_t apsel = swjdp->apsel;
	if (armv7a->memory_ap_available && (apsel == armv7a->memory_ap)) {
		uint32_t ret;
		retval = armv7a_mmu_translate_va(target,
				virt, &ret);
		if (retval != ERROR_OK)
			goto done;
		*phys = ret;
	} else {/*  use this method if armv7a->memory_ap not selected
		 *  mmu must be enable in order to get a correct translation */
		retval = cortex_a8_mmu_modify(target, 1);
		if (retval != ERROR_OK)
			goto done;
		retval = armv7a_mmu_translate_va_pa(target, virt,  phys, 1);
	}
done:
	return retval;
}

COMMAND_HANDLER(cortex_a8_handle_cache_info_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct armv7a_common *armv7a = target_to_armv7a(target);

	return armv7a_handle_cache_info_command(CMD_CTX,
			&armv7a->armv7a_mmu.armv7a_cache);
}


COMMAND_HANDLER(cortex_a8_handle_dbginit_command)
{
	struct target *target = get_current_target(CMD_CTX);
	if (!target_was_examined(target)) {
		LOG_ERROR("target not examined yet");
		return ERROR_FAIL;
	}

	return cortex_a8_init_debug_access(target);
}
COMMAND_HANDLER(cortex_a8_handle_smp_off_command)
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

COMMAND_HANDLER(cortex_a8_handle_smp_on_command)
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

COMMAND_HANDLER(cortex_a8_handle_smp_gdb_command)
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

static const struct command_registration cortex_a8_exec_command_handlers[] = {
	{
		.name = "cache_info",
		.handler = cortex_a8_handle_cache_info_command,
		.mode = COMMAND_EXEC,
		.help = "display information about target caches",
		.usage = "",
	},
	{
		.name = "dbginit",
		.handler = cortex_a8_handle_dbginit_command,
		.mode = COMMAND_EXEC,
		.help = "Initialize core debug",
		.usage = "",
	},
	{   .name = "smp_off",
	    .handler = cortex_a8_handle_smp_off_command,
	    .mode = COMMAND_EXEC,
	    .help = "Stop smp handling",
	    .usage = "",},
	{
		.name = "smp_on",
		.handler = cortex_a8_handle_smp_on_command,
		.mode = COMMAND_EXEC,
		.help = "Restart smp handling",
		.usage = "",
	},
	{
		.name = "smp_gdb",
		.handler = cortex_a8_handle_smp_gdb_command,
		.mode = COMMAND_EXEC,
		.help = "display/fix current core played to gdb",
		.usage = "",
	},


	COMMAND_REGISTRATION_DONE
};
static const struct command_registration cortex_a8_command_handlers[] = {
	{
		.chain = arm_command_handlers,
	},
	{
		.chain = armv7a_command_handlers,
	},
	{
		.name = "cortex_a",
		.mode = COMMAND_ANY,
		.help = "Cortex-A command group",
		.usage = "",
		.chain = cortex_a8_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct target_type cortexa8_target = {
	.name = "cortex_a",
	.deprecated_name = "cortex_a8",

	.poll = cortex_a8_poll,
	.arch_state = armv7a_arch_state,

	.halt = cortex_a8_halt,
	.resume = cortex_a8_resume,
	.step = cortex_a8_step,

	.assert_reset = cortex_a8_assert_reset,
	.deassert_reset = cortex_a8_deassert_reset,

	/* REVISIT allow exporting VFP3 registers ... */
	.get_gdb_reg_list = arm_get_gdb_reg_list,

	.read_memory = cortex_a8_read_memory,
	.write_memory = cortex_a8_write_memory,

	.checksum_memory = arm_checksum_memory,
	.blank_check_memory = arm_blank_check_memory,

	.run_algorithm = armv4_5_run_algorithm,

	.add_breakpoint = cortex_a8_add_breakpoint,
	.add_context_breakpoint = cortex_a8_add_context_breakpoint,
	.add_hybrid_breakpoint = cortex_a8_add_hybrid_breakpoint,
	.remove_breakpoint = cortex_a8_remove_breakpoint,
	.add_watchpoint = NULL,
	.remove_watchpoint = NULL,

	.commands = cortex_a8_command_handlers,
	.target_create = cortex_a8_target_create,
	.init_target = cortex_a8_init_target,
	.examine = cortex_a8_examine,

	.read_phys_memory = cortex_a8_read_phys_memory,
	.write_phys_memory = cortex_a8_write_phys_memory,
	.mmu = cortex_a8_mmu,
	.virt2phys = cortex_a8_virt2phys,
};

static const struct command_registration cortex_r4_exec_command_handlers[] = {
	{
		.name = "cache_info",
		.handler = cortex_a8_handle_cache_info_command,
		.mode = COMMAND_EXEC,
		.help = "display information about target caches",
		.usage = "",
	},
	{
		.name = "dbginit",
		.handler = cortex_a8_handle_dbginit_command,
		.mode = COMMAND_EXEC,
		.help = "Initialize core debug",
		.usage = "",
	},

	COMMAND_REGISTRATION_DONE
};
static const struct command_registration cortex_r4_command_handlers[] = {
	{
		.chain = arm_command_handlers,
	},
	{
		.chain = armv7a_command_handlers,
	},
	{
		.name = "cortex_r4",
		.mode = COMMAND_ANY,
		.help = "Cortex-R4 command group",
		.usage = "",
		.chain = cortex_r4_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct target_type cortexr4_target = {
	.name = "cortex_r4",

	.poll = cortex_a8_poll,
	.arch_state = armv7a_arch_state,

	.halt = cortex_a8_halt,
	.resume = cortex_a8_resume,
	.step = cortex_a8_step,

	.assert_reset = cortex_a8_assert_reset,
	.deassert_reset = cortex_a8_deassert_reset,

	/* REVISIT allow exporting VFP3 registers ... */
	.get_gdb_reg_list = arm_get_gdb_reg_list,

	.read_memory = cortex_a8_read_memory,
	.write_memory = cortex_a8_write_memory,

	.checksum_memory = arm_checksum_memory,
	.blank_check_memory = arm_blank_check_memory,

	.run_algorithm = armv4_5_run_algorithm,

	.add_breakpoint = cortex_a8_add_breakpoint,
	.add_context_breakpoint = cortex_a8_add_context_breakpoint,
	.add_hybrid_breakpoint = cortex_a8_add_hybrid_breakpoint,
	.remove_breakpoint = cortex_a8_remove_breakpoint,
	.add_watchpoint = NULL,
	.remove_watchpoint = NULL,

	.commands = cortex_r4_command_handlers,
	.target_create = cortex_r4_target_create,
	.init_target = cortex_a8_init_target,
	.examine = cortex_a8_examine,
};
