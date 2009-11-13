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
 *                                                                         *
 *   Cortex-A8(tm) TRM, ARM DDI 0344H                                      *
 *                                                                         *
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "cortex_a8.h"
#include "armv7a.h"
#include "armv4_5.h"

#include "target_request.h"
#include "target_type.h"

static int cortex_a8_poll(target_t *target);
static int cortex_a8_debug_entry(target_t *target);
static int cortex_a8_restore_context(target_t *target);
static int cortex_a8_set_breakpoint(struct target_s *target,
		breakpoint_t *breakpoint, uint8_t matchmode);
static int cortex_a8_unset_breakpoint(struct target_s *target,
		breakpoint_t *breakpoint);
static int cortex_a8_dap_read_coreregister_u32(target_t *target,
		uint32_t *value, int regnum);
static int cortex_a8_dap_write_coreregister_u32(target_t *target,
		uint32_t value, int regnum);
/*
 * FIXME do topology discovery using the ROM; don't
 * assume this is an OMAP3.
 */
#define swjdp_memoryap 0
#define swjdp_debugap 1
#define OMAP3530_DEBUG_BASE 0x54011000

/*
 * Cortex-A8 Basic debug access, very low level assumes state is saved
 */
static int cortex_a8_init_debug_access(target_t *target)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	int retval;
	uint32_t dummy;

	LOG_DEBUG(" ");

	/* Unlocking the debug registers for modification */
	/* The debugport might be uninitialised so try twice */
	retval = mem_ap_write_atomic_u32(swjdp, armv7a->debug_base + CPUDBG_LOCKACCESS, 0xC5ACCE55);
	if (retval != ERROR_OK)
		mem_ap_write_atomic_u32(swjdp, armv7a->debug_base + CPUDBG_LOCKACCESS, 0xC5ACCE55);
	/* Clear Sticky Power Down status Bit in PRSR to enable access to
	   the registers in the Core Power Domain */
	retval = mem_ap_read_atomic_u32(swjdp, armv7a->debug_base + CPUDBG_PRSR, &dummy);
	/* Enabling of instruction execution in debug mode is done in debug_entry code */

	/* Resync breakpoint registers */

	/* Since this is likley called from init or reset, update targtet state information*/
	cortex_a8_poll(target);

	return retval;
}

int cortex_a8_exec_opcode(target_t *target, uint32_t opcode)
{
	uint32_t dscr;
	int retval;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	LOG_DEBUG("exec opcode 0x%08" PRIx32, opcode);
	do
	{
		retval = mem_ap_read_atomic_u32(swjdp,
				armv7a->debug_base + CPUDBG_DSCR, &dscr);
		if (retval != ERROR_OK)
		{
			LOG_ERROR("Could not read DSCR register, opcode = 0x%08" PRIx32, opcode);
			return retval;
		}
	}
	while ((dscr & (1 << DSCR_INSTR_COMP)) == 0); /* Wait for InstrCompl bit to be set */

	mem_ap_write_u32(swjdp, armv7a->debug_base + CPUDBG_ITR, opcode);

	do
	{
		retval = mem_ap_read_atomic_u32(swjdp,
				armv7a->debug_base + CPUDBG_DSCR, &dscr);
		if (retval != ERROR_OK)
		{
			LOG_ERROR("Could not read DSCR register");
			return retval;
		}
	}
	while ((dscr & (1 << DSCR_INSTR_COMP)) == 0); /* Wait for InstrCompl bit to be set */

	return retval;
}

/**************************************************************************
Read core register with very few exec_opcode, fast but needs work_area.
This can cause problems with MMU active.
**************************************************************************/
static int cortex_a8_read_regs_through_mem(target_t *target, uint32_t address,
		uint32_t * regfile)
{
	int retval = ERROR_OK;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	cortex_a8_dap_read_coreregister_u32(target, regfile, 0);
	cortex_a8_dap_write_coreregister_u32(target, address, 0);
	cortex_a8_exec_opcode(target, ARMV4_5_STMIA(0, 0xFFFE, 0, 0));
	dap_ap_select(swjdp, swjdp_memoryap);
	mem_ap_read_buf_u32(swjdp, (uint8_t *)(&regfile[1]), 4*15, address);
	dap_ap_select(swjdp, swjdp_debugap);

	return retval;
}

static int cortex_a8_read_cp(target_t *target, uint32_t *value, uint8_t CP,
		uint8_t op1, uint8_t CRn, uint8_t CRm, uint8_t op2)
{
	int retval;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	cortex_a8_exec_opcode(target, ARMV4_5_MRC(CP, op1, 0, CRn, CRm, op2));
	/* Move R0 to DTRTX */
	cortex_a8_exec_opcode(target, ARMV4_5_MCR(14, 0, 0, 0, 5, 0));

	/* Read DCCTX */
	retval = mem_ap_read_atomic_u32(swjdp,
			armv7a->debug_base + CPUDBG_DTRTX, value);

	return retval;
}

static int cortex_a8_write_cp(target_t *target, uint32_t value,
	uint8_t CP, uint8_t op1, uint8_t CRn, uint8_t CRm, uint8_t op2)
{
	int retval;
	uint32_t dscr;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	LOG_DEBUG("CP%i, CRn %i, value 0x%08" PRIx32, CP, CRn, value);

	/* Check that DCCRX is not full */
	retval = mem_ap_read_atomic_u32(swjdp,
				armv7a->debug_base + CPUDBG_DSCR, &dscr);
	if (dscr & (1 << DSCR_DTR_RX_FULL))
	{
		LOG_ERROR("DSCR_DTR_RX_FULL, dscr 0x%08" PRIx32, dscr);
		/* Clear DCCRX with MCR(p14, 0, Rd, c0, c5, 0), opcode  0xEE000E15 */
		cortex_a8_exec_opcode(target, ARMV4_5_MRC(14, 0, 0, 0, 5, 0));
	}

	retval = mem_ap_write_u32(swjdp,
			armv7a->debug_base + CPUDBG_DTRRX, value);
	/* Move DTRRX to r0 */
	cortex_a8_exec_opcode(target, ARMV4_5_MRC(14, 0, 0, 0, 5, 0));

	cortex_a8_exec_opcode(target, ARMV4_5_MCR(CP, op1, 0, CRn, CRm, op2));
	return retval;
}

static int cortex_a8_read_cp15(target_t *target, uint32_t op1, uint32_t op2,
		uint32_t CRn, uint32_t CRm, uint32_t *value)
{
	return cortex_a8_read_cp(target, value, 15, op1, CRn, CRm, op2);
}

static int cortex_a8_write_cp15(target_t *target, uint32_t op1, uint32_t op2,
		uint32_t CRn, uint32_t CRm, uint32_t value)
{
	return cortex_a8_write_cp(target, value, 15, op1, CRn, CRm, op2);
}

static int cortex_a8_mrc(target_t *target, int cpnum, uint32_t op1, uint32_t op2, uint32_t CRn, uint32_t CRm, uint32_t *value)
{
	if (cpnum!=15)
	{
		LOG_ERROR("Only cp15 is supported");
		return ERROR_FAIL;
	}
	return cortex_a8_read_cp15(target, op1, op2, CRn, CRm, value);
}

static int cortex_a8_mcr(target_t *target, int cpnum, uint32_t op1, uint32_t op2, uint32_t CRn, uint32_t CRm, uint32_t value)
{
	if (cpnum!=15)
	{
		LOG_ERROR("Only cp15 is supported");
		return ERROR_FAIL;
	}
	return cortex_a8_write_cp15(target, op1, op2, CRn, CRm, value);
}



static int cortex_a8_dap_read_coreregister_u32(target_t *target,
		uint32_t *value, int regnum)
{
	int retval = ERROR_OK;
	uint8_t reg = regnum&0xFF;
	uint32_t dscr;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	if (reg > 16)
		return retval;

	if (reg < 15)
	{
		/* Rn to DCCTX, MCR p14, 0, Rd, c0, c5, 0,  0xEE000E15 */
		cortex_a8_exec_opcode(target, ARMV4_5_MCR(14, 0, reg, 0, 5, 0));
	}
	else if (reg == 15)
	{
		cortex_a8_exec_opcode(target, 0xE1A0000F);
		cortex_a8_exec_opcode(target, ARMV4_5_MCR(14, 0, 0, 0, 5, 0));
	}
	else if (reg == 16)
	{
		cortex_a8_exec_opcode(target, ARMV4_5_MRS(0, 0));
		cortex_a8_exec_opcode(target, ARMV4_5_MCR(14, 0, 0, 0, 5, 0));
	}

	/* Read DTRRTX */
	do
	{
		retval = mem_ap_read_atomic_u32(swjdp,
				armv7a->debug_base + CPUDBG_DSCR, &dscr);
	}
	while ((dscr & (1 << DSCR_DTR_TX_FULL)) == 0); /* Wait for DTRRXfull */

	retval = mem_ap_read_atomic_u32(swjdp,
			armv7a->debug_base + CPUDBG_DTRTX, value);

	return retval;
}

static int cortex_a8_dap_write_coreregister_u32(target_t *target, uint32_t value, int regnum)
{
	int retval = ERROR_OK;
	uint8_t Rd = regnum&0xFF;
	uint32_t dscr;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	LOG_DEBUG("register %i, value 0x%08" PRIx32, regnum, value);

	/* Check that DCCRX is not full */
	retval = mem_ap_read_atomic_u32(swjdp,
				armv7a->debug_base + CPUDBG_DSCR, &dscr);
	if (dscr & (1 << DSCR_DTR_RX_FULL))
	{
		LOG_ERROR("DSCR_DTR_RX_FULL, dscr 0x%08" PRIx32, dscr);
		/* Clear DCCRX with MCR(p14, 0, Rd, c0, c5, 0), opcode  0xEE000E15 */
		cortex_a8_exec_opcode(target, ARMV4_5_MRC(14, 0, 0, 0, 5, 0));
	}

	if (Rd > 16)
		return retval;

	/* Write to DCCRX */
	retval = mem_ap_write_u32(swjdp,
			armv7a->debug_base + CPUDBG_DTRRX, value);

	if (Rd < 15)
	{
		/* DCCRX to Rd, MCR p14, 0, Rd, c0, c5, 0,  0xEE000E15 */
		cortex_a8_exec_opcode(target, ARMV4_5_MRC(14, 0, Rd, 0, 5, 0));
	}
	else if (Rd == 15)
	{
		cortex_a8_exec_opcode(target, ARMV4_5_MRC(14, 0, 0, 0, 5, 0));
		cortex_a8_exec_opcode(target, 0xE1A0F000);
	}
	else if (Rd == 16)
	{
		cortex_a8_exec_opcode(target, ARMV4_5_MRC(14, 0, 0, 0, 5, 0));
		cortex_a8_exec_opcode(target, ARMV4_5_MSR_GP(0, 0xF, 0));
		/* Execute a PrefetchFlush instruction through the ITR. */
		cortex_a8_exec_opcode(target, ARMV4_5_MCR(15, 0, 0, 7, 5, 4));
	}

	return retval;
}

/* Write to memory mapped registers directly with no cache or mmu handling */
static int cortex_a8_dap_write_memap_register_u32(target_t *target, uint32_t address, uint32_t value)
{
	int retval;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	retval = mem_ap_write_atomic_u32(swjdp, address, value);

	return retval;
}

/*
 * Cortex-A8 Run control
 */

static int cortex_a8_poll(target_t *target)
{
	int retval = ERROR_OK;
	uint32_t dscr;
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);
	struct armv7a_common *armv7a = &cortex_a8->armv7a_common;
	struct swjdp_common *swjdp = &armv7a->swjdp_info;
	enum target_state prev_target_state = target->state;
	uint8_t saved_apsel = dap_ap_get_select(swjdp);

	dap_ap_select(swjdp, swjdp_debugap);
	retval = mem_ap_read_atomic_u32(swjdp,
			armv7a->debug_base + CPUDBG_DSCR, &dscr);
	if (retval != ERROR_OK)
	{
		dap_ap_select(swjdp, saved_apsel);
		return retval;
	}
	cortex_a8->cpudbg_dscr = dscr;

	if ((dscr & 0x3) == 0x3)
	{
		if (prev_target_state != TARGET_HALTED)
		{
			/* We have a halting debug event */
			LOG_DEBUG("Target halted");
			target->state = TARGET_HALTED;
			if ((prev_target_state == TARGET_RUNNING)
					|| (prev_target_state == TARGET_RESET))
			{
				retval = cortex_a8_debug_entry(target);
				if (retval != ERROR_OK)
					return retval;

				target_call_event_callbacks(target,
						TARGET_EVENT_HALTED);
			}
			if (prev_target_state == TARGET_DEBUG_RUNNING)
			{
				LOG_DEBUG(" ");

				retval = cortex_a8_debug_entry(target);
				if (retval != ERROR_OK)
					return retval;

				target_call_event_callbacks(target,
						TARGET_EVENT_DEBUG_HALTED);
			}
		}
	}
	else if ((dscr & 0x3) == 0x2)
	{
		target->state = TARGET_RUNNING;
	}
	else
	{
		LOG_DEBUG("Unknown target state dscr = 0x%08" PRIx32, dscr);
		target->state = TARGET_UNKNOWN;
	}

	dap_ap_select(swjdp, saved_apsel);

	return retval;
}

static int cortex_a8_halt(target_t *target)
{
	int retval = ERROR_OK;
	uint32_t dscr;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;
	uint8_t saved_apsel = dap_ap_get_select(swjdp);
	dap_ap_select(swjdp, swjdp_debugap);

	/*
	 * Tell the core to be halted by writing DRCR with 0x1
	 * and then wait for the core to be halted.
	 */
	retval = mem_ap_write_atomic_u32(swjdp,
			armv7a->debug_base + CPUDBG_DRCR, 0x1);

	/*
	 * enter halting debug mode
	 */
	mem_ap_read_atomic_u32(swjdp, armv7a->debug_base + CPUDBG_DSCR, &dscr);
	retval = mem_ap_write_atomic_u32(swjdp,
		armv7a->debug_base + CPUDBG_DSCR, dscr | (1 << DSCR_HALT_DBG_MODE));

	if (retval != ERROR_OK)
		goto out;

	do {
		mem_ap_read_atomic_u32(swjdp,
			armv7a->debug_base + CPUDBG_DSCR, &dscr);
	} while ((dscr & (1 << DSCR_CORE_HALTED)) == 0);

	target->debug_reason = DBG_REASON_DBGRQ;

out:
	dap_ap_select(swjdp, saved_apsel);
	return retval;
}

static int cortex_a8_resume(struct target_s *target, int current,
		uint32_t address, int handle_breakpoints, int debug_execution)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct armv4_5_common_s *armv4_5 = &armv7a->armv4_5_common;
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

//	breakpoint_t *breakpoint = NULL;
	uint32_t resume_pc, dscr;

	uint8_t saved_apsel = dap_ap_get_select(swjdp);
	dap_ap_select(swjdp, swjdp_debugap);

	if (!debug_execution)
	{
		target_free_all_working_areas(target);
//		cortex_m3_enable_breakpoints(target);
//		cortex_m3_enable_watchpoints(target);
	}

#if 0
	if (debug_execution)
	{
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
			buf_get_u32(armv7m->core_cache->reg_list[ARMV7M_xPSR].value, 0, 32) | (1 << 24));
		armv7m->core_cache->reg_list[ARMV7M_xPSR].dirty = 1;
		armv7m->core_cache->reg_list[ARMV7M_xPSR].valid = 1;
	}
#endif

	/* current = 1: continue on current pc, otherwise continue at <address> */
	resume_pc = buf_get_u32(
			ARMV7A_CORE_REG_MODE(armv4_5->core_cache,
				armv4_5->core_mode, 15).value,
			0, 32);
	if (!current)
		resume_pc = address;

	/* Make sure that the Armv7 gdb thumb fixups does not
	 * kill the return address
	 */
	if (armv7a->core_state == ARMV7A_STATE_ARM)
	{
		resume_pc &= 0xFFFFFFFC;
	}
	/* When the return address is loaded into PC
	 * bit 0 must be 1 to stay in Thumb state
	 */
	if (armv7a->core_state == ARMV7A_STATE_THUMB)
	{
		resume_pc |= 0x1;
	}
	LOG_DEBUG("resume pc = 0x%08" PRIx32, resume_pc);
	buf_set_u32(ARMV7A_CORE_REG_MODE(armv4_5->core_cache,
				armv4_5->core_mode, 15).value,
			0, 32, resume_pc);
	ARMV7A_CORE_REG_MODE(armv4_5->core_cache,
			armv4_5->core_mode, 15).dirty = 1;
	ARMV7A_CORE_REG_MODE(armv4_5->core_cache,
			armv4_5->core_mode, 15).valid = 1;

	cortex_a8_restore_context(target);
//	arm7_9_restore_context(target); TODO Context is currently NOT Properly restored
#if 0
	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints)
	{
		/* Single step past breakpoint at current address */
		if ((breakpoint = breakpoint_find(target, resume_pc)))
		{
			LOG_DEBUG("unset breakpoint at 0x%8.8x", breakpoint->address);
			cortex_m3_unset_breakpoint(target, breakpoint);
			cortex_m3_single_step_core(target);
			cortex_m3_set_breakpoint(target, breakpoint);
		}
	}

#endif
	/* Restart core and wait for it to be started */
	mem_ap_write_atomic_u32(swjdp, armv7a->debug_base + CPUDBG_DRCR, 0x2);

	do {
		mem_ap_read_atomic_u32(swjdp,
			armv7a->debug_base + CPUDBG_DSCR, &dscr);
	} while ((dscr & (1 << DSCR_CORE_RESTARTED)) == 0);

	target->debug_reason = DBG_REASON_NOTHALTED;
	target->state = TARGET_RUNNING;

	/* registers are now invalid */
	armv4_5_invalidate_core_regs(target);

	if (!debug_execution)
	{
		target->state = TARGET_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		LOG_DEBUG("target resumed at 0x%" PRIx32, resume_pc);
	}
	else
	{
		target->state = TARGET_DEBUG_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED);
		LOG_DEBUG("target debug resumed at 0x%" PRIx32, resume_pc);
	}

	dap_ap_select(swjdp, saved_apsel);

	return ERROR_OK;
}

static int cortex_a8_debug_entry(target_t *target)
{
	int i;
	uint32_t regfile[16], pc, cpsr, dscr;
	int retval = ERROR_OK;
	struct working_area *regfile_working_area = NULL;
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct armv4_5_common_s *armv4_5 = &armv7a->armv4_5_common;
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	LOG_DEBUG("dscr = 0x%08" PRIx32, cortex_a8->cpudbg_dscr);

	/* Enable the ITR execution once we are in debug mode */
	mem_ap_read_atomic_u32(swjdp,
				armv7a->debug_base + CPUDBG_DSCR, &dscr);
	dscr |= (1 << DSCR_EXT_INT_EN);
	retval = mem_ap_write_atomic_u32(swjdp,
			armv7a->debug_base + CPUDBG_DSCR, dscr);

	/* Examine debug reason */
	switch ((cortex_a8->cpudbg_dscr >> 2)&0xF)
	{
		case 0:
		case 4:
			target->debug_reason = DBG_REASON_DBGRQ;
			break;
		case 1:
		case 3:
			target->debug_reason = DBG_REASON_BREAKPOINT;
			break;
		case 10:
			target->debug_reason = DBG_REASON_WATCHPOINT;
			break;
		default:
			target->debug_reason = DBG_REASON_UNDEFINED;
			break;
	}

	/* Examine target state and mode */
	if (cortex_a8->fast_reg_read)
		target_alloc_working_area(target, 64, &regfile_working_area);

	/* First load register acessible through core debug port*/
	if (!regfile_working_area)
	{
		for (i = 0; i <= 15; i++)
			cortex_a8_dap_read_coreregister_u32(target,
					&regfile[i], i);
	}
	else
	{
		dap_ap_select(swjdp, swjdp_memoryap);
		cortex_a8_read_regs_through_mem(target,
				regfile_working_area->address, regfile);
		dap_ap_select(swjdp, swjdp_memoryap);
		target_free_working_area(target, regfile_working_area);
	}

	cortex_a8_dap_read_coreregister_u32(target, &cpsr, 16);
	pc = regfile[15];
	dap_ap_select(swjdp, swjdp_debugap);
	LOG_DEBUG("cpsr: %8.8" PRIx32, cpsr);

	armv4_5->core_mode = cpsr & 0x1F;
	armv7a->core_state = (cpsr & 0x20)?ARMV7A_STATE_THUMB:ARMV7A_STATE_ARM;

	for (i = 0; i <= ARM_PC; i++)
	{
		buf_set_u32(ARMV7A_CORE_REG_MODE(armv4_5->core_cache,
					armv4_5->core_mode, i).value,
				0, 32, regfile[i]);
		ARMV7A_CORE_REG_MODE(armv4_5->core_cache,
				armv4_5->core_mode, i).valid = 1;
		ARMV7A_CORE_REG_MODE(armv4_5->core_cache,
				armv4_5->core_mode, i).dirty = 0;
	}
	buf_set_u32(ARMV7A_CORE_REG_MODE(armv4_5->core_cache,
				armv4_5->core_mode, 16).value,
			0, 32, cpsr);
	ARMV7A_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 16).valid = 1;
	ARMV7A_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 16).dirty = 0;

	/* Fixup PC Resume Address */
	if (armv7a->core_state == ARMV7A_STATE_THUMB)
	{
		// T bit set for Thumb or ThumbEE state
		regfile[ARM_PC] -= 4;
	}
	else
	{
		// ARM state
		regfile[ARM_PC] -= 8;
	}
	buf_set_u32(ARMV7A_CORE_REG_MODE(armv4_5->core_cache,
				armv4_5->core_mode, ARM_PC).value,
			0, 32, regfile[ARM_PC]);

	ARMV7A_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 0)
		.dirty = ARMV7A_CORE_REG_MODE(armv4_5->core_cache,
				armv4_5->core_mode, 0).valid;
	ARMV7A_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 15)
		.dirty = ARMV7A_CORE_REG_MODE(armv4_5->core_cache,
				armv4_5->core_mode, 15).valid;

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
//	armv4_5->exception_number = 0;
	if (armv7a->post_debug_entry)
		armv7a->post_debug_entry(target);



	return retval;

}

static void cortex_a8_post_debug_entry(target_t *target)
{
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);
	struct armv7a_common *armv7a = &cortex_a8->armv7a_common;

//	cortex_a8_read_cp(target, &cp15_control_register, 15, 0, 1, 0, 0);
	/* examine cp15 control reg */
	armv7a->read_cp15(target, 0, 0, 1, 0, &cortex_a8->cp15_control_reg);
	jtag_execute_queue();
	LOG_DEBUG("cp15_control_reg: %8.8" PRIx32, cortex_a8->cp15_control_reg);

	if (armv7a->armv4_5_mmu.armv4_5_cache.ctype == -1)
	{
		uint32_t cache_type_reg;
		/* identify caches */
		armv7a->read_cp15(target, 0, 1, 0, 0, &cache_type_reg);
		jtag_execute_queue();
		/* FIXME the armv4_4 cache info DOES NOT APPLY to Cortex-A8 */
		armv4_5_identify_cache(cache_type_reg,
				&armv7a->armv4_5_mmu.armv4_5_cache);
	}

	armv7a->armv4_5_mmu.mmu_enabled =
			(cortex_a8->cp15_control_reg & 0x1U) ? 1 : 0;
	armv7a->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled =
			(cortex_a8->cp15_control_reg & 0x4U) ? 1 : 0;
	armv7a->armv4_5_mmu.armv4_5_cache.i_cache_enabled =
			(cortex_a8->cp15_control_reg & 0x1000U) ? 1 : 0;


}

static int cortex_a8_step(struct target_s *target, int current, uint32_t address,
		int handle_breakpoints)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct armv4_5_common_s *armv4_5 = &armv7a->armv4_5_common;
	breakpoint_t *breakpoint = NULL;
	breakpoint_t stepbreakpoint;

	int timeout = 100;

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current)
	{
		buf_set_u32(ARMV7A_CORE_REG_MODE(armv4_5->core_cache,
					armv4_5->core_mode, ARM_PC).value,
				0, 32, address);
	}
	else
	{
		address = buf_get_u32(ARMV7A_CORE_REG_MODE(armv4_5->core_cache,
					armv4_5->core_mode, ARM_PC).value,
				0, 32);
	}

	/* The front-end may request us not to handle breakpoints.
	 * But since Cortex-A8 uses breakpoint for single step,
	 * we MUST handle breakpoints.
	 */
	handle_breakpoints = 1;
	if (handle_breakpoints) {
		breakpoint = breakpoint_find(target,
				buf_get_u32(ARMV7A_CORE_REG_MODE(armv4_5->core_cache,
					armv4_5->core_mode, 15).value,
			0, 32));
		if (breakpoint)
			cortex_a8_unset_breakpoint(target, breakpoint);
	}

	/* Setup single step breakpoint */
	stepbreakpoint.address = address;
	stepbreakpoint.length = (armv7a->core_state == ARMV7A_STATE_THUMB) ? 2 : 4;
	stepbreakpoint.type = BKPT_HARD;
	stepbreakpoint.set = 0;

	/* Break on IVA mismatch */
	cortex_a8_set_breakpoint(target, &stepbreakpoint, 0x04);

	target->debug_reason = DBG_REASON_SINGLESTEP;

	cortex_a8_resume(target, 1, address, 0, 0);

	while (target->state != TARGET_HALTED)
	{
		cortex_a8_poll(target);
		if (--timeout == 0)
		{
			LOG_WARNING("timeout waiting for target halt");
			break;
		}
	}

	cortex_a8_unset_breakpoint(target, &stepbreakpoint);
	if (timeout > 0) target->debug_reason = DBG_REASON_BREAKPOINT;

	if (breakpoint)
		cortex_a8_set_breakpoint(target, breakpoint, 0);

	if (target->state != TARGET_HALTED)
		LOG_DEBUG("target stepped");

	return ERROR_OK;
}

static int cortex_a8_restore_context(target_t *target)
{
	int i;
	uint32_t value;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct armv4_5_common_s *armv4_5 = &armv7a->armv4_5_common;

	LOG_DEBUG(" ");

	if (armv7a->pre_restore_context)
		armv7a->pre_restore_context(target);

	for (i = 15; i >= 0; i--)
	{
		if (ARMV7A_CORE_REG_MODE(armv4_5->core_cache,
					armv4_5->core_mode, i).dirty)
		{
			value = buf_get_u32(ARMV7A_CORE_REG_MODE(armv4_5->core_cache,
						armv4_5->core_mode, i).value,
					0, 32);
			/* TODO Check return values */
			cortex_a8_dap_write_coreregister_u32(target, value, i);
		}
	}

	if (armv7a->post_restore_context)
		armv7a->post_restore_context(target);

	return ERROR_OK;
}


#if 0
/*
 * Cortex-A8 Core register functions
 */
static int cortex_a8_load_core_reg_u32(struct target_s *target, int num,
		armv4_5_mode_t mode, uint32_t * value)
{
	int retval;
	struct armv4_5_common_s *armv4_5 = target_to_armv4_5(target);

	if ((num <= ARM_CPSR))
	{
		/* read a normal core register */
		retval = cortex_a8_dap_read_coreregister_u32(target, value, num);

		if (retval != ERROR_OK)
		{
			LOG_ERROR("JTAG failure %i", retval);
			return ERROR_JTAG_DEVICE_ERROR;
		}
		LOG_DEBUG("load from core reg %i value 0x%" PRIx32, num, *value);
	}
	else
	{
		return ERROR_INVALID_ARGUMENTS;
	}

	/* Register other than r0 - r14 uses r0 for access */
	if (num > 14)
		ARMV7A_CORE_REG_MODE(armv4_5->core_cache,
				armv4_5->core_mode, 0).dirty =
			ARMV7A_CORE_REG_MODE(armv4_5->core_cache,
				armv4_5->core_mode, 0).valid;
	ARMV7A_CORE_REG_MODE(armv4_5->core_cache,
				armv4_5->core_mode, 15).dirty =
			ARMV7A_CORE_REG_MODE(armv4_5->core_cache,
				armv4_5->core_mode, 15).valid;

	return ERROR_OK;
}

static int cortex_a8_store_core_reg_u32(struct target_s *target, int num,
		armv4_5_mode_t mode, uint32_t value)
{
	int retval;
//	uint32_t reg;
	struct armv4_5_common_s *armv4_5 = target_to_armv4_5(target);

#ifdef ARMV7_GDB_HACKS
	/* If the LR register is being modified, make sure it will put us
	 * in "thumb" mode, or an INVSTATE exception will occur. This is a
	 * hack to deal with the fact that gdb will sometimes "forge"
	 * return addresses, and doesn't set the LSB correctly (i.e., when
	 * printing expressions containing function calls, it sets LR=0.) */

	if (num == 14)
		value |= 0x01;
#endif

	if ((num <= ARM_CPSR))
	{
		retval = cortex_a8_dap_write_coreregister_u32(target, value, num);
		if (retval != ERROR_OK)
		{
			LOG_ERROR("JTAG failure %i", retval);
			ARMV7A_CORE_REG_MODE(armv4_5->core_cache,
					armv4_5->core_mode, num).dirty =
				ARMV7A_CORE_REG_MODE(armv4_5->core_cache,
					armv4_5->core_mode, num).valid;
			return ERROR_JTAG_DEVICE_ERROR;
		}
		LOG_DEBUG("write core reg %i value 0x%" PRIx32, num, value);
	}
	else
	{
		return ERROR_INVALID_ARGUMENTS;
	}

	return ERROR_OK;
}
#endif


static int cortex_a8_read_core_reg(struct target_s *target, int num,
		enum armv4_5_mode mode)
{
	uint32_t value;
	int retval;
	struct armv4_5_common_s *armv4_5 = target_to_armv4_5(target);

	cortex_a8_dap_read_coreregister_u32(target, &value, num);

	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		return retval;
	}

	ARMV7A_CORE_REG_MODE(armv4_5->core_cache, mode, num).valid = 1;
	ARMV7A_CORE_REG_MODE(armv4_5->core_cache, mode, num).dirty = 0;
	buf_set_u32(ARMV7A_CORE_REG_MODE(armv4_5->core_cache,
			mode, num).value, 0, 32, value);

	return ERROR_OK;
}

int cortex_a8_write_core_reg(struct target_s *target, int num,
		enum armv4_5_mode mode, uint32_t value)
{
	int retval;
	struct armv4_5_common_s *armv4_5 = target_to_armv4_5(target);

	cortex_a8_dap_write_coreregister_u32(target, value, num);
	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		return retval;
	}

	ARMV7A_CORE_REG_MODE(armv4_5->core_cache, mode, num).valid = 1;
	ARMV7A_CORE_REG_MODE(armv4_5->core_cache, mode, num).dirty = 0;

	return ERROR_OK;
}


/*
 * Cortex-A8 Breakpoint and watchpoint fuctions
 */

/* Setup hardware Breakpoint Register Pair */
static int cortex_a8_set_breakpoint(struct target_s *target,
		breakpoint_t *breakpoint, uint8_t matchmode)
{
	int retval;
	int brp_i=0;
	uint32_t control;
	uint8_t byte_addr_select = 0x0F;
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);
	struct armv7a_common *armv7a = &cortex_a8->armv7a_common;
	cortex_a8_brp_t * brp_list = cortex_a8->brp_list;

	if (breakpoint->set)
	{
		LOG_WARNING("breakpoint already set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD)
	{
		while (brp_list[brp_i].used && (brp_i < cortex_a8->brp_num))
			brp_i++ ;
		if (brp_i >= cortex_a8->brp_num)
		{
			LOG_ERROR("ERROR Can not find free Breakpoint Register Pair");
			exit(-1);
		}
		breakpoint->set = brp_i + 1;
		if (breakpoint->length == 2)
		{
			byte_addr_select = (3 << (breakpoint->address & 0x02));
		}
		control = ((matchmode & 0x7) << 20)
				| (byte_addr_select << 5)
				| (3 << 1) | 1;
		brp_list[brp_i].used = 1;
		brp_list[brp_i].value = (breakpoint->address & 0xFFFFFFFC);
		brp_list[brp_i].control = control;
		cortex_a8_dap_write_memap_register_u32(target, armv7a->debug_base
				+ CPUDBG_BVR_BASE + 4 * brp_list[brp_i].BRPn,
				brp_list[brp_i].value);
		cortex_a8_dap_write_memap_register_u32(target, armv7a->debug_base
				+ CPUDBG_BCR_BASE + 4 * brp_list[brp_i].BRPn,
				brp_list[brp_i].control);
		LOG_DEBUG("brp %i control 0x%0" PRIx32 " value 0x%0" PRIx32, brp_i,
				brp_list[brp_i].control,
				brp_list[brp_i].value);
	}
	else if (breakpoint->type == BKPT_SOFT)
	{
		uint8_t code[4];
		if (breakpoint->length == 2)
		{
			buf_set_u32(code, 0, 32, ARMV5_T_BKPT(0x11));
		}
		else
		{
			buf_set_u32(code, 0, 32, ARMV5_BKPT(0x11));
		}
		retval = target->type->read_memory(target,
				breakpoint->address & 0xFFFFFFFE,
				breakpoint->length, 1,
				breakpoint->orig_instr);
		if (retval != ERROR_OK)
			return retval;
		retval = target->type->write_memory(target,
				breakpoint->address & 0xFFFFFFFE,
				breakpoint->length, 1, code);
		if (retval != ERROR_OK)
			return retval;
		breakpoint->set = 0x11; /* Any nice value but 0 */
	}

	return ERROR_OK;
}

static int cortex_a8_unset_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	int retval;
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);
	struct armv7a_common *armv7a = &cortex_a8->armv7a_common;
	cortex_a8_brp_t * brp_list = cortex_a8->brp_list;

	if (!breakpoint->set)
	{
		LOG_WARNING("breakpoint not set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD)
	{
		int brp_i = breakpoint->set - 1;
		if ((brp_i < 0) || (brp_i >= cortex_a8->brp_num))
		{
			LOG_DEBUG("Invalid BRP number in breakpoint");
			return ERROR_OK;
		}
		LOG_DEBUG("rbp %i control 0x%0" PRIx32 " value 0x%0" PRIx32, brp_i,
				brp_list[brp_i].control, brp_list[brp_i].value);
		brp_list[brp_i].used = 0;
		brp_list[brp_i].value = 0;
		brp_list[brp_i].control = 0;
		cortex_a8_dap_write_memap_register_u32(target, armv7a->debug_base
				+ CPUDBG_BCR_BASE + 4 * brp_list[brp_i].BRPn,
				brp_list[brp_i].control);
		cortex_a8_dap_write_memap_register_u32(target, armv7a->debug_base
				+ CPUDBG_BVR_BASE + 4 * brp_list[brp_i].BRPn,
				brp_list[brp_i].value);
	}
	else
	{
		/* restore original instruction (kept in target endianness) */
		if (breakpoint->length == 4)
		{
			retval = target->type->write_memory(target,
					breakpoint->address & 0xFFFFFFFE,
					4, 1, breakpoint->orig_instr);
			if (retval != ERROR_OK)
				return retval;
		}
		else
		{
			retval = target->type->write_memory(target,
					breakpoint->address & 0xFFFFFFFE,
					2, 1, breakpoint->orig_instr);
			if (retval != ERROR_OK)
				return retval;
		}
	}
	breakpoint->set = 0;

	return ERROR_OK;
}

int cortex_a8_add_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);

	if ((breakpoint->type == BKPT_HARD) && (cortex_a8->brp_num_available < 1))
	{
		LOG_INFO("no hardware breakpoint available");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (breakpoint->type == BKPT_HARD)
		cortex_a8->brp_num_available--;
	cortex_a8_set_breakpoint(target, breakpoint, 0x00); /* Exact match */

	return ERROR_OK;
}

static int cortex_a8_remove_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);

#if 0
/* It is perfectly possible to remove brakpoints while the taget is running */
	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
#endif

	if (breakpoint->set)
	{
		cortex_a8_unset_breakpoint(target, breakpoint);
		if (breakpoint->type == BKPT_HARD)
			cortex_a8->brp_num_available++ ;
	}


	return ERROR_OK;
}



/*
 * Cortex-A8 Reset fuctions
 */

static int cortex_a8_assert_reset(target_t *target)
{

	LOG_DEBUG(" ");

	/* registers are now invalid */
	armv4_5_invalidate_core_regs(target);

	target->state = TARGET_RESET;

	return ERROR_OK;
}

static int cortex_a8_deassert_reset(target_t *target)
{

	LOG_DEBUG(" ");

	if (target->reset_halt)
	{
		int retval;
		if ((retval = target_halt(target)) != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

/*
 * Cortex-A8 Memory access
 *
 * This is same Cortex M3 but we must also use the correct
 * ap number for every access.
 */

static int cortex_a8_read_memory(struct target_s *target, uint32_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	int retval = ERROR_OK;

	/* sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_INVALID_ARGUMENTS;

	/* cortex_a8 handles unaligned memory access */

// ???	dap_ap_select(swjdp, swjdp_memoryap);

	switch (size)
	{
		case 4:
			retval = mem_ap_read_buf_u32(swjdp, buffer, 4 * count, address);
			break;
		case 2:
			retval = mem_ap_read_buf_u16(swjdp, buffer, 2 * count, address);
			break;
		case 1:
			retval = mem_ap_read_buf_u8(swjdp, buffer, count, address);
			break;
		default:
			LOG_ERROR("BUG: we shouldn't get here");
			exit(-1);
	}

	return retval;
}

int cortex_a8_write_memory(struct target_s *target, uint32_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	int retval;

	/* sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_INVALID_ARGUMENTS;

// ???	dap_ap_select(swjdp, swjdp_memoryap);

	switch (size)
	{
		case 4:
			retval = mem_ap_write_buf_u32(swjdp, buffer, 4 * count, address);
			break;
		case 2:
			retval = mem_ap_write_buf_u16(swjdp, buffer, 2 * count, address);
			break;
		case 1:
			retval = mem_ap_write_buf_u8(swjdp, buffer, count, address);
			break;
		default:
			LOG_ERROR("BUG: we shouldn't get here");
			exit(-1);
	}

	if (target->state == TARGET_HALTED)
	{
		/* The Cache handling will NOT work with MMU active, the wrong addresses will be invalidated */
		/* invalidate I-Cache */
		if (armv7a->armv4_5_mmu.armv4_5_cache.i_cache_enabled)
		{
			/* Invalidate ICache single entry with MVA, repeat this for all cache
			   lines in the address range, Cortex-A8 has fixed 64 byte line length */
			/* Invalidate Cache single entry with MVA to PoU */
			for (uint32_t cacheline=address; cacheline<address+size*count; cacheline+=64)
				armv7a->write_cp15(target, 0, 1, 7, 5, cacheline); /* I-Cache to PoU */
		}
		/* invalidate D-Cache */
		if (armv7a->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled)
		{
			/* Invalidate Cache single entry with MVA to PoC */
			for (uint32_t cacheline=address; cacheline<address+size*count; cacheline+=64)
				armv7a->write_cp15(target, 0, 1, 7, 6, cacheline); /* U/D cache to PoC */
		}
	}

	return retval;
}

static int cortex_a8_bulk_write_memory(target_t *target, uint32_t address,
		uint32_t count, uint8_t *buffer)
{
	return cortex_a8_write_memory(target, address, 4, count, buffer);
}


static int cortex_a8_dcc_read(struct swjdp_common *swjdp, uint8_t *value, uint8_t *ctrl)
{
#if 0
	u16 dcrdr;

	mem_ap_read_buf_u16(swjdp, (uint8_t*)&dcrdr, 1, DCB_DCRDR);
	*ctrl = (uint8_t)dcrdr;
	*value = (uint8_t)(dcrdr >> 8);

	LOG_DEBUG("data 0x%x ctrl 0x%x", *value, *ctrl);

	/* write ack back to software dcc register
	 * signify we have read data */
	if (dcrdr & (1 << 0))
	{
		dcrdr = 0;
		mem_ap_write_buf_u16(swjdp, (uint8_t*)&dcrdr, 1, DCB_DCRDR);
	}
#endif
	return ERROR_OK;
}


static int cortex_a8_handle_target_request(void *priv)
{
	target_t *target = priv;
	if (!target->type->examined)
		return ERROR_OK;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	if (!target->dbg_msg_enabled)
		return ERROR_OK;

	if (target->state == TARGET_RUNNING)
	{
		uint8_t data = 0;
		uint8_t ctrl = 0;

		cortex_a8_dcc_read(swjdp, &data, &ctrl);

		/* check if we have data */
		if (ctrl & (1 << 0))
		{
			uint32_t request;

			/* we assume target is quick enough */
			request = data;
			cortex_a8_dcc_read(swjdp, &data, &ctrl);
			request |= (data << 8);
			cortex_a8_dcc_read(swjdp, &data, &ctrl);
			request |= (data << 16);
			cortex_a8_dcc_read(swjdp, &data, &ctrl);
			request |= (data << 24);
			target_request(target, request);
		}
	}

	return ERROR_OK;
}

/*
 * Cortex-A8 target information and configuration
 */

static int cortex_a8_examine(struct target_s *target)
{
	struct cortex_a8_common *cortex_a8 = target_to_cortex_a8(target);
	struct armv7a_common *armv7a = &cortex_a8->armv7a_common;
	struct swjdp_common *swjdp = &armv7a->swjdp_info;
	int i;
	int retval = ERROR_OK;
	uint32_t didr, ctypr, ttypr, cpuid;

	LOG_DEBUG("TODO");

	/* Here we shall insert a proper ROM Table scan */
	armv7a->debug_base = OMAP3530_DEBUG_BASE;

	/* We do one extra read to ensure DAP is configured,
	 * we call ahbap_debugport_init(swjdp) instead
	 */
	ahbap_debugport_init(swjdp);
	mem_ap_read_atomic_u32(swjdp, armv7a->debug_base + CPUDBG_CPUID, &cpuid);
	if ((retval = mem_ap_read_atomic_u32(swjdp,
			armv7a->debug_base + CPUDBG_CPUID, &cpuid)) != ERROR_OK)
	{
		LOG_DEBUG("Examine failed");
		return retval;
	}

	if ((retval = mem_ap_read_atomic_u32(swjdp,
			armv7a->debug_base + CPUDBG_CTYPR, &ctypr)) != ERROR_OK)
	{
		LOG_DEBUG("Examine failed");
		return retval;
	}

	if ((retval = mem_ap_read_atomic_u32(swjdp,
			armv7a->debug_base + CPUDBG_TTYPR, &ttypr)) != ERROR_OK)
	{
		LOG_DEBUG("Examine failed");
		return retval;
	}

	if ((retval = mem_ap_read_atomic_u32(swjdp,
			armv7a->debug_base + CPUDBG_DIDR, &didr)) != ERROR_OK)
	{
		LOG_DEBUG("Examine failed");
		return retval;
	}

	LOG_DEBUG("cpuid = 0x%08" PRIx32, cpuid);
	LOG_DEBUG("ctypr = 0x%08" PRIx32, ctypr);
	LOG_DEBUG("ttypr = 0x%08" PRIx32, ttypr);
	LOG_DEBUG("didr = 0x%08" PRIx32, didr);

	/* Setup Breakpoint Register Pairs */
	cortex_a8->brp_num = ((didr >> 24) & 0x0F) + 1;
	cortex_a8->brp_num_context = ((didr >> 20) & 0x0F) + 1;
	cortex_a8->brp_num_available = cortex_a8->brp_num;
	cortex_a8->brp_list = calloc(cortex_a8->brp_num, sizeof(cortex_a8_brp_t));
//	cortex_a8->brb_enabled = ????;
	for (i = 0; i < cortex_a8->brp_num; i++)
	{
		cortex_a8->brp_list[i].used = 0;
		if (i < (cortex_a8->brp_num-cortex_a8->brp_num_context))
			cortex_a8->brp_list[i].type = BRP_NORMAL;
		else
			cortex_a8->brp_list[i].type = BRP_CONTEXT;
		cortex_a8->brp_list[i].value = 0;
		cortex_a8->brp_list[i].control = 0;
		cortex_a8->brp_list[i].BRPn = i;
	}

	/* Setup Watchpoint Register Pairs */
	cortex_a8->wrp_num = ((didr >> 28) & 0x0F) + 1;
	cortex_a8->wrp_num_available = cortex_a8->wrp_num;
	cortex_a8->wrp_list = calloc(cortex_a8->wrp_num, sizeof(cortex_a8_wrp_t));
	for (i = 0; i < cortex_a8->wrp_num; i++)
	{
		cortex_a8->wrp_list[i].used = 0;
		cortex_a8->wrp_list[i].type = 0;
		cortex_a8->wrp_list[i].value = 0;
		cortex_a8->wrp_list[i].control = 0;
		cortex_a8->wrp_list[i].WRPn = i;
	}
	LOG_DEBUG("Configured %i hw breakpoint pairs and %i hw watchpoint pairs",
			cortex_a8->brp_num , cortex_a8->wrp_num);

	/* Configure core debug access */
	cortex_a8_init_debug_access(target);

	target->type->examined = 1;

	return retval;
}

/*
 *	Cortex-A8 target creation and initialization
 */

static void cortex_a8_build_reg_cache(target_t *target)
{
	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct armv4_5_common_s *armv4_5 = target_to_armv4_5(target);

	(*cache_p) = armv4_5_build_reg_cache(target, armv4_5);
	armv4_5->core_cache = (*cache_p);
}


static int cortex_a8_init_target(struct command_context_s *cmd_ctx,
		struct target_s *target)
{
	cortex_a8_build_reg_cache(target);
	return ERROR_OK;
}

int cortex_a8_init_arch_info(target_t *target,
		struct cortex_a8_common *cortex_a8, struct jtag_tap *tap)
{
	armv4_5_common_t *armv4_5;
	struct armv7a_common *armv7a;

	armv7a = &cortex_a8->armv7a_common;
	armv4_5 = &armv7a->armv4_5_common;
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	/* Setup struct cortex_a8_common */
	cortex_a8->common_magic = CORTEX_A8_COMMON_MAGIC;
	armv4_5->arch_info = armv7a;

	armv4_5_init_arch_info(target, armv4_5);

	/* prepare JTAG information for the new target */
	cortex_a8->jtag_info.tap = tap;
	cortex_a8->jtag_info.scann_size = 4;
LOG_DEBUG(" ");
	swjdp->dp_select_value = -1;
	swjdp->ap_csw_value = -1;
	swjdp->ap_tar_value = -1;
	swjdp->jtag_info = &cortex_a8->jtag_info;
	swjdp->memaccess_tck = 80;

	/* Number of bits for tar autoincrement, impl. dep. at least 10 */
	swjdp->tar_autoincr_block = (1 << 10);

	cortex_a8->fast_reg_read = 0;


	/* register arch-specific functions */
	armv7a->examine_debug_reason = NULL;

	armv7a->post_debug_entry = cortex_a8_post_debug_entry;

	armv7a->pre_restore_context = NULL;
	armv7a->post_restore_context = NULL;
	armv7a->armv4_5_mmu.armv4_5_cache.ctype = -1;
//	armv7a->armv4_5_mmu.get_ttb = armv7a_get_ttb;
	armv7a->armv4_5_mmu.read_memory = cortex_a8_read_memory;
	armv7a->armv4_5_mmu.write_memory = cortex_a8_write_memory;
//	armv7a->armv4_5_mmu.disable_mmu_caches = armv7a_disable_mmu_caches;
//	armv7a->armv4_5_mmu.enable_mmu_caches = armv7a_enable_mmu_caches;
	armv7a->armv4_5_mmu.has_tiny_pages = 1;
	armv7a->armv4_5_mmu.mmu_enabled = 0;
	armv7a->read_cp15 = cortex_a8_read_cp15;
	armv7a->write_cp15 = cortex_a8_write_cp15;


//	arm7_9->handle_target_request = cortex_a8_handle_target_request;

	armv4_5->read_core_reg = cortex_a8_read_core_reg;
	armv4_5->write_core_reg = cortex_a8_write_core_reg;
//	armv4_5->full_context = arm7_9_full_context;

//	armv4_5->load_core_reg_u32 = cortex_a8_load_core_reg_u32;
//	armv4_5->store_core_reg_u32 = cortex_a8_store_core_reg_u32;
//	armv4_5->read_core_reg = armv4_5_read_core_reg; /* this is default */
//	armv4_5->write_core_reg = armv4_5_write_core_reg;

	target_register_timer_callback(cortex_a8_handle_target_request, 1, 1, target);

	return ERROR_OK;
}

static int cortex_a8_target_create(struct target_s *target, Jim_Interp *interp)
{
	struct cortex_a8_common *cortex_a8 = calloc(1, sizeof(struct cortex_a8_common));

	cortex_a8_init_arch_info(target, cortex_a8, target->tap);

	return ERROR_OK;
}

COMMAND_HANDLER(cortex_a8_handle_cache_info_command)
{
	target_t *target = get_current_target(cmd_ctx);
	struct armv7a_common *armv7a = target_to_armv7a(target);

	return armv4_5_handle_cache_info_command(cmd_ctx,
			&armv7a->armv4_5_mmu.armv4_5_cache);
}


COMMAND_HANDLER(cortex_a8_handle_dbginit_command)
{
	target_t *target = get_current_target(cmd_ctx);

	cortex_a8_init_debug_access(target);

	return ERROR_OK;
}


static int cortex_a8_register_commands(struct command_context_s *cmd_ctx)
{
	command_t *cortex_a8_cmd;
	int retval = ERROR_OK;

	armv4_5_register_commands(cmd_ctx);
	armv7a_register_commands(cmd_ctx);

	cortex_a8_cmd =	register_command(cmd_ctx, NULL, "cortex_a8",
			NULL, COMMAND_ANY,
			"cortex_a8 specific commands");

	register_command(cmd_ctx, cortex_a8_cmd, "cache_info",
			cortex_a8_handle_cache_info_command, COMMAND_EXEC,
			"display information about target caches");

	register_command(cmd_ctx, cortex_a8_cmd, "dbginit",
			cortex_a8_handle_dbginit_command, COMMAND_EXEC,
			"Initialize core debug");

	return retval;
}

target_type_t cortexa8_target = {
	.name = "cortex_a8",

	.poll = cortex_a8_poll,
	.arch_state = armv7a_arch_state,

	.target_request_data = NULL,

	.halt = cortex_a8_halt,
	.resume = cortex_a8_resume,
	.step = cortex_a8_step,

	.assert_reset = cortex_a8_assert_reset,
	.deassert_reset = cortex_a8_deassert_reset,
	.soft_reset_halt = NULL,

	.get_gdb_reg_list = armv4_5_get_gdb_reg_list,

	.read_memory = cortex_a8_read_memory,
	.write_memory = cortex_a8_write_memory,
	.bulk_write_memory = cortex_a8_bulk_write_memory,
	.checksum_memory = arm7_9_checksum_memory,
	.blank_check_memory = arm7_9_blank_check_memory,

	.run_algorithm = armv4_5_run_algorithm,

	.add_breakpoint = cortex_a8_add_breakpoint,
	.remove_breakpoint = cortex_a8_remove_breakpoint,
	.add_watchpoint = NULL,
	.remove_watchpoint = NULL,

	.register_commands = cortex_a8_register_commands,
	.target_create = cortex_a8_target_create,
	.init_target = cortex_a8_init_target,
	.examine = cortex_a8_examine,
	.mrc = cortex_a8_mrc,
	.mcr = cortex_a8_mcr,
};
