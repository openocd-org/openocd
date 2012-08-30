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
 *                                                                         *
 *   Cortex-M3(tm) TRM, ARM DDI 0337E (r1p1) and 0337G (r2p0)              *
 *                                                                         *
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "jtag/interface.h"
#include "breakpoints.h"
#include "cortex_m.h"
#include "target_request.h"
#include "target_type.h"
#include "arm_disassembler.h"
#include "register.h"
#include "arm_opcodes.h"
#include "arm_semihosting.h"
#include <helper/time_support.h>

/* NOTE:  most of this should work fine for the Cortex-M1 and
 * Cortex-M0 cores too, although they're ARMv6-M not ARMv7-M.
 * Some differences:  M0/M1 doesn't have FBP remapping or the
 * DWT tracing/profiling support.  (So the cycle counter will
 * not be usable; the other stuff isn't currently used here.)
 *
 * Although there are some workarounds for errata seen only in r0p0
 * silicon, such old parts are hard to find and thus not much tested
 * any longer.
 */

/**
 * Returns the type of a break point required by address location
 */
#define BKPT_TYPE_BY_ADDR(addr) ((addr) < 0x20000000 ? BKPT_HARD : BKPT_SOFT)


/* forward declarations */
static int cortex_m3_store_core_reg_u32(struct target *target,
	enum armv7m_regtype type, uint32_t num, uint32_t value);

static int cortexm3_dap_read_coreregister_u32(struct adiv5_dap *swjdp,
	uint32_t *value, int regnum)
{
	int retval;
	uint32_t dcrdr;

	/* because the DCB_DCRDR is used for the emulated dcc channel
	 * we have to save/restore the DCB_DCRDR when used */

	retval = mem_ap_read_u32(swjdp, DCB_DCRDR, &dcrdr);
	if (retval != ERROR_OK)
		return retval;

	/* mem_ap_write_u32(swjdp, DCB_DCRSR, regnum); */
	retval = dap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_OFF, DCB_DCRSR & 0xFFFFFFF0);
	if (retval != ERROR_OK)
		return retval;
	retval = dap_queue_ap_write(swjdp, AP_REG_BD0 | (DCB_DCRSR & 0xC), regnum);
	if (retval != ERROR_OK)
		return retval;

	/* mem_ap_read_u32(swjdp, DCB_DCRDR, value); */
	retval = dap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_OFF, DCB_DCRDR & 0xFFFFFFF0);
	if (retval != ERROR_OK)
		return retval;
	retval = dap_queue_ap_read(swjdp, AP_REG_BD0 | (DCB_DCRDR & 0xC), value);
	if (retval != ERROR_OK)
		return retval;

	retval = dap_run(swjdp);
	if (retval != ERROR_OK)
		return retval;

	/* restore DCB_DCRDR - this needs to be in a seperate
	 * transaction otherwise the emulated DCC channel breaks */
	if (retval == ERROR_OK)
		retval = mem_ap_write_atomic_u32(swjdp, DCB_DCRDR, dcrdr);

	return retval;
}

static int cortexm3_dap_write_coreregister_u32(struct adiv5_dap *swjdp,
	uint32_t value, int regnum)
{
	int retval;
	uint32_t dcrdr;

	/* because the DCB_DCRDR is used for the emulated dcc channel
	 * we have to save/restore the DCB_DCRDR when used */

	retval = mem_ap_read_u32(swjdp, DCB_DCRDR, &dcrdr);
	if (retval != ERROR_OK)
		return retval;

	/* mem_ap_write_u32(swjdp, DCB_DCRDR, core_regs[i]); */
	retval = dap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_OFF, DCB_DCRDR & 0xFFFFFFF0);
	if (retval != ERROR_OK)
		return retval;
	retval = dap_queue_ap_write(swjdp, AP_REG_BD0 | (DCB_DCRDR & 0xC), value);
	if (retval != ERROR_OK)
		return retval;

	/* mem_ap_write_u32(swjdp, DCB_DCRSR, i | DCRSR_WnR); */
	retval = dap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_OFF, DCB_DCRSR & 0xFFFFFFF0);
	if (retval != ERROR_OK)
		return retval;
	retval = dap_queue_ap_write(swjdp, AP_REG_BD0 | (DCB_DCRSR & 0xC), regnum | DCRSR_WnR);
	if (retval != ERROR_OK)
		return retval;

	retval = dap_run(swjdp);
	if (retval != ERROR_OK)
		return retval;

	/* restore DCB_DCRDR - this needs to be in a seperate
	 * transaction otherwise the emulated DCC channel breaks */
	if (retval == ERROR_OK)
		retval = mem_ap_write_atomic_u32(swjdp, DCB_DCRDR, dcrdr);

	return retval;
}

static int cortex_m3_write_debug_halt_mask(struct target *target,
	uint32_t mask_on, uint32_t mask_off)
{
	struct cortex_m3_common *cortex_m3 = target_to_cm3(target);
	struct adiv5_dap *swjdp = cortex_m3->armv7m.arm.dap;

	/* mask off status bits */
	cortex_m3->dcb_dhcsr &= ~((0xFFFF << 16) | mask_off);
	/* create new register mask */
	cortex_m3->dcb_dhcsr |= DBGKEY | C_DEBUGEN | mask_on;

	return mem_ap_write_atomic_u32(swjdp, DCB_DHCSR, cortex_m3->dcb_dhcsr);
}

static int cortex_m3_clear_halt(struct target *target)
{
	struct cortex_m3_common *cortex_m3 = target_to_cm3(target);
	struct adiv5_dap *swjdp = cortex_m3->armv7m.arm.dap;
	int retval;

	/* clear step if any */
	cortex_m3_write_debug_halt_mask(target, C_HALT, C_STEP);

	/* Read Debug Fault Status Register */
	retval = mem_ap_read_atomic_u32(swjdp, NVIC_DFSR, &cortex_m3->nvic_dfsr);
	if (retval != ERROR_OK)
		return retval;

	/* Clear Debug Fault Status */
	retval = mem_ap_write_atomic_u32(swjdp, NVIC_DFSR, cortex_m3->nvic_dfsr);
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG(" NVIC_DFSR 0x%" PRIx32 "", cortex_m3->nvic_dfsr);

	return ERROR_OK;
}

static int cortex_m3_single_step_core(struct target *target)
{
	struct cortex_m3_common *cortex_m3 = target_to_cm3(target);
	struct adiv5_dap *swjdp = cortex_m3->armv7m.arm.dap;
	uint32_t dhcsr_save;
	int retval;

	/* backup dhcsr reg */
	dhcsr_save = cortex_m3->dcb_dhcsr;

	/* Mask interrupts before clearing halt, if done already.  This avoids
	 * Erratum 377497 (fixed in r1p0) where setting MASKINTS while clearing
	 * HALT can put the core into an unknown state.
	 */
	if (!(cortex_m3->dcb_dhcsr & C_MASKINTS)) {
		retval = mem_ap_write_atomic_u32(swjdp, DCB_DHCSR,
				DBGKEY | C_MASKINTS | C_HALT | C_DEBUGEN);
		if (retval != ERROR_OK)
			return retval;
	}
	retval = mem_ap_write_atomic_u32(swjdp, DCB_DHCSR,
			DBGKEY | C_MASKINTS | C_STEP | C_DEBUGEN);
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG(" ");

	/* restore dhcsr reg */
	cortex_m3->dcb_dhcsr = dhcsr_save;
	cortex_m3_clear_halt(target);

	return ERROR_OK;
}

static int cortex_m3_endreset_event(struct target *target)
{
	int i;
	int retval;
	uint32_t dcb_demcr;
	struct cortex_m3_common *cortex_m3 = target_to_cm3(target);
	struct armv7m_common *armv7m = &cortex_m3->armv7m;
	struct adiv5_dap *swjdp = cortex_m3->armv7m.arm.dap;
	struct cortex_m3_fp_comparator *fp_list = cortex_m3->fp_comparator_list;
	struct cortex_m3_dwt_comparator *dwt_list = cortex_m3->dwt_comparator_list;

	/* REVISIT The four debug monitor bits are currently ignored... */
	retval = mem_ap_read_atomic_u32(swjdp, DCB_DEMCR, &dcb_demcr);
	if (retval != ERROR_OK)
		return retval;
	LOG_DEBUG("DCB_DEMCR = 0x%8.8" PRIx32 "", dcb_demcr);

	/* this register is used for emulated dcc channel */
	retval = mem_ap_write_u32(swjdp, DCB_DCRDR, 0);
	if (retval != ERROR_OK)
		return retval;

	/* Enable debug requests */
	retval = mem_ap_read_atomic_u32(swjdp, DCB_DHCSR, &cortex_m3->dcb_dhcsr);
	if (retval != ERROR_OK)
		return retval;
	if (!(cortex_m3->dcb_dhcsr & C_DEBUGEN)) {
		retval = mem_ap_write_u32(swjdp, DCB_DHCSR, DBGKEY | C_DEBUGEN);
		if (retval != ERROR_OK)
			return retval;
	}

	/* clear any interrupt masking */
	cortex_m3_write_debug_halt_mask(target, 0, C_MASKINTS);

	/* Enable features controlled by ITM and DWT blocks, and catch only
	 * the vectors we were told to pay attention to.
	 *
	 * Target firmware is responsible for all fault handling policy
	 * choices *EXCEPT* explicitly scripted overrides like "vector_catch"
	 * or manual updates to the NVIC SHCSR and CCR registers.
	 */
	retval = mem_ap_write_u32(swjdp, DCB_DEMCR, TRCENA | armv7m->demcr);
	if (retval != ERROR_OK)
		return retval;

	/* Paranoia: evidently some (early?) chips don't preserve all the
	 * debug state (including FBP, DWT, etc) across reset...
	 */

	/* Enable FPB */
	retval = target_write_u32(target, FP_CTRL, 3);
	if (retval != ERROR_OK)
		return retval;

	cortex_m3->fpb_enabled = 1;

	/* Restore FPB registers */
	for (i = 0; i < cortex_m3->fp_num_code + cortex_m3->fp_num_lit; i++) {
		retval = target_write_u32(target, fp_list[i].fpcr_address, fp_list[i].fpcr_value);
		if (retval != ERROR_OK)
			return retval;
	}

	/* Restore DWT registers */
	for (i = 0; i < cortex_m3->dwt_num_comp; i++) {
		retval = target_write_u32(target, dwt_list[i].dwt_comparator_address + 0,
				dwt_list[i].comp);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target, dwt_list[i].dwt_comparator_address + 4,
				dwt_list[i].mask);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_u32(target, dwt_list[i].dwt_comparator_address + 8,
				dwt_list[i].function);
		if (retval != ERROR_OK)
			return retval;
	}
	retval = dap_run(swjdp);
	if (retval != ERROR_OK)
		return retval;

	register_cache_invalidate(cortex_m3->armv7m.core_cache);

	/* make sure we have latest dhcsr flags */
	retval = mem_ap_read_atomic_u32(swjdp, DCB_DHCSR, &cortex_m3->dcb_dhcsr);

	return retval;
}

static int cortex_m3_examine_debug_reason(struct target *target)
{
	struct cortex_m3_common *cortex_m3 = target_to_cm3(target);

	/* THIS IS NOT GOOD, TODO - better logic for detection of debug state reason
	 * only check the debug reason if we don't know it already */

	if ((target->debug_reason != DBG_REASON_DBGRQ)
		&& (target->debug_reason != DBG_REASON_SINGLESTEP)) {
		if (cortex_m3->nvic_dfsr & DFSR_BKPT) {
			target->debug_reason = DBG_REASON_BREAKPOINT;
			if (cortex_m3->nvic_dfsr & DFSR_DWTTRAP)
				target->debug_reason = DBG_REASON_WPTANDBKPT;
		} else if (cortex_m3->nvic_dfsr & DFSR_DWTTRAP)
			target->debug_reason = DBG_REASON_WATCHPOINT;
		else if (cortex_m3->nvic_dfsr & DFSR_VCATCH)
			target->debug_reason = DBG_REASON_BREAKPOINT;
		else	/* EXTERNAL, HALTED */
			target->debug_reason = DBG_REASON_UNDEFINED;
	}

	return ERROR_OK;
}

static int cortex_m3_examine_exception_reason(struct target *target)
{
	uint32_t shcsr = 0, except_sr = 0, cfsr = -1, except_ar = -1;
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct adiv5_dap *swjdp = armv7m->arm.dap;
	int retval;

	retval = mem_ap_read_u32(swjdp, NVIC_SHCSR, &shcsr);
	if (retval != ERROR_OK)
		return retval;
	switch (armv7m->exception_number) {
		case 2:	/* NMI */
			break;
		case 3:	/* Hard Fault */
			retval = mem_ap_read_atomic_u32(swjdp, NVIC_HFSR, &except_sr);
			if (retval != ERROR_OK)
				return retval;
			if (except_sr & 0x40000000) {
				retval = mem_ap_read_u32(swjdp, NVIC_CFSR, &cfsr);
				if (retval != ERROR_OK)
					return retval;
			}
			break;
		case 4:	/* Memory Management */
			retval = mem_ap_read_u32(swjdp, NVIC_CFSR, &except_sr);
			if (retval != ERROR_OK)
				return retval;
			retval = mem_ap_read_u32(swjdp, NVIC_MMFAR, &except_ar);
			if (retval != ERROR_OK)
				return retval;
			break;
		case 5:	/* Bus Fault */
			retval = mem_ap_read_u32(swjdp, NVIC_CFSR, &except_sr);
			if (retval != ERROR_OK)
				return retval;
			retval = mem_ap_read_u32(swjdp, NVIC_BFAR, &except_ar);
			if (retval != ERROR_OK)
				return retval;
			break;
		case 6:	/* Usage Fault */
			retval = mem_ap_read_u32(swjdp, NVIC_CFSR, &except_sr);
			if (retval != ERROR_OK)
				return retval;
			break;
		case 11:	/* SVCall */
			break;
		case 12:	/* Debug Monitor */
			retval = mem_ap_read_u32(swjdp, NVIC_DFSR, &except_sr);
			if (retval != ERROR_OK)
				return retval;
			break;
		case 14:	/* PendSV */
			break;
		case 15:	/* SysTick */
			break;
		default:
			except_sr = 0;
			break;
	}
	retval = dap_run(swjdp);
	if (retval == ERROR_OK)
		LOG_DEBUG("%s SHCSR 0x%" PRIx32 ", SR 0x%" PRIx32
			", CFSR 0x%" PRIx32 ", AR 0x%" PRIx32,
			armv7m_exception_string(armv7m->exception_number),
			shcsr, except_sr, cfsr, except_ar);
	return retval;
}

static int cortex_m3_debug_entry(struct target *target)
{
	int i;
	uint32_t xPSR;
	int retval;
	struct cortex_m3_common *cortex_m3 = target_to_cm3(target);
	struct armv7m_common *armv7m = &cortex_m3->armv7m;
	struct arm *arm = &armv7m->arm;
	struct adiv5_dap *swjdp = armv7m->arm.dap;
	struct reg *r;

	LOG_DEBUG(" ");

	cortex_m3_clear_halt(target);
	retval = mem_ap_read_atomic_u32(swjdp, DCB_DHCSR, &cortex_m3->dcb_dhcsr);
	if (retval != ERROR_OK)
		return retval;

	retval = armv7m->examine_debug_reason(target);
	if (retval != ERROR_OK)
		return retval;

	/* Examine target state and mode
	 * First load register acessible through core debug port*/
	int num_regs = armv7m->core_cache->num_regs;

	for (i = 0; i < num_regs; i++) {
		if (!armv7m->core_cache->reg_list[i].valid)
			armv7m->read_core_reg(target, i);
	}

	r = armv7m->core_cache->reg_list + ARMV7M_xPSR;
	xPSR = buf_get_u32(r->value, 0, 32);

#ifdef ARMV7_GDB_HACKS
	/* FIXME this breaks on scan chains with more than one Cortex-M3.
	 * Instead, each CM3 should have its own dummy value...
	 */
	/* copy real xpsr reg for gdb, setting thumb bit */
	buf_set_u32(armv7m_gdb_dummy_cpsr_value, 0, 32, xPSR);
	buf_set_u32(armv7m_gdb_dummy_cpsr_value, 5, 1, 1);
	armv7m_gdb_dummy_cpsr_reg.valid = r->valid;
	armv7m_gdb_dummy_cpsr_reg.dirty = r->dirty;
#endif

	/* For IT instructions xPSR must be reloaded on resume and clear on debug exec */
	if (xPSR & 0xf00) {
		r->dirty = r->valid;
		cortex_m3_store_core_reg_u32(target, ARMV7M_REGISTER_CORE_GP, 16, xPSR & ~0xff);
	}

	/* Are we in an exception handler */
	if (xPSR & 0x1FF) {
		armv7m->core_mode = ARMV7M_MODE_HANDLER;
		armv7m->exception_number = (xPSR & 0x1FF);

		arm->core_mode = ARM_MODE_HANDLER;
		arm->map = armv7m_msp_reg_map;
	} else {
		unsigned control = buf_get_u32(armv7m->core_cache
				->reg_list[ARMV7M_CONTROL].value, 0, 2);

		/* is this thread privileged? */
		armv7m->core_mode = control & 1;
		arm->core_mode = armv7m->core_mode
			? ARM_MODE_USER_THREAD
			: ARM_MODE_THREAD;

		/* which stack is it using? */
		if (control & 2)
			arm->map = armv7m_psp_reg_map;
		else
			arm->map = armv7m_msp_reg_map;

		armv7m->exception_number = 0;
	}

	if (armv7m->exception_number)
		cortex_m3_examine_exception_reason(target);

	LOG_DEBUG("entered debug state in core mode: %s at PC 0x%" PRIx32 ", target->state: %s",
		armv7m_mode_strings[armv7m->core_mode],
		*(uint32_t *)(arm->pc->value),
		target_state_name(target));

	if (armv7m->post_debug_entry) {
		retval = armv7m->post_debug_entry(target);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int cortex_m3_poll(struct target *target)
{
	int detected_failure = ERROR_OK;
	int retval = ERROR_OK;
	enum target_state prev_target_state = target->state;
	struct cortex_m3_common *cortex_m3 = target_to_cm3(target);
	struct adiv5_dap *swjdp = cortex_m3->armv7m.arm.dap;

	/* Read from Debug Halting Control and Status Register */
	retval = mem_ap_read_atomic_u32(swjdp, DCB_DHCSR, &cortex_m3->dcb_dhcsr);
	if (retval != ERROR_OK) {
		target->state = TARGET_UNKNOWN;
		return retval;
	}

	/* Recover from lockup.  See ARMv7-M architecture spec,
	 * section B1.5.15 "Unrecoverable exception cases".
	 */
	if (cortex_m3->dcb_dhcsr & S_LOCKUP) {
		LOG_ERROR("%s -- clearing lockup after double fault",
			target_name(target));
		cortex_m3_write_debug_halt_mask(target, C_HALT, 0);
		target->debug_reason = DBG_REASON_DBGRQ;

		/* We have to execute the rest (the "finally" equivalent, but
		 * still throw this exception again).
		 */
		detected_failure = ERROR_FAIL;

		/* refresh status bits */
		retval = mem_ap_read_atomic_u32(swjdp, DCB_DHCSR, &cortex_m3->dcb_dhcsr);
		if (retval != ERROR_OK)
			return retval;
	}

	if (cortex_m3->dcb_dhcsr & S_RESET_ST) {
		/* check if still in reset */
		retval = mem_ap_read_atomic_u32(swjdp, DCB_DHCSR, &cortex_m3->dcb_dhcsr);
		if (retval != ERROR_OK)
			return retval;

		if (cortex_m3->dcb_dhcsr & S_RESET_ST) {
			target->state = TARGET_RESET;
			return ERROR_OK;
		}
	}

	if (target->state == TARGET_RESET) {
		/* Cannot switch context while running so endreset is
		 * called with target->state == TARGET_RESET
		 */
		LOG_DEBUG("Exit from reset with dcb_dhcsr 0x%" PRIx32,
			cortex_m3->dcb_dhcsr);
		cortex_m3_endreset_event(target);
		target->state = TARGET_RUNNING;
		prev_target_state = TARGET_RUNNING;
	}

	if (cortex_m3->dcb_dhcsr & S_HALT) {
		target->state = TARGET_HALTED;

		if ((prev_target_state == TARGET_RUNNING) || (prev_target_state == TARGET_RESET)) {
			retval = cortex_m3_debug_entry(target);
			if (retval != ERROR_OK)
				return retval;

			if (arm_semihosting(target, &retval) != 0)
				return retval;

			target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		}
		if (prev_target_state == TARGET_DEBUG_RUNNING) {
			LOG_DEBUG(" ");
			retval = cortex_m3_debug_entry(target);
			if (retval != ERROR_OK)
				return retval;

			target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
		}
	}

	/* REVISIT when S_SLEEP is set, it's in a Sleep or DeepSleep state.
	 * How best to model low power modes?
	 */

	if (target->state == TARGET_UNKNOWN) {
		/* check if processor is retiring instructions */
		if (cortex_m3->dcb_dhcsr & S_RETIRE_ST) {
			target->state = TARGET_RUNNING;
			retval = ERROR_OK;
		}
	}

	/* Did we detect a failure condition that we cleared? */
	if (detected_failure != ERROR_OK)
		retval = detected_failure;
	return retval;
}

static int cortex_m3_halt(struct target *target)
{
	LOG_DEBUG("target->state: %s",
		target_state_name(target));

	if (target->state == TARGET_HALTED) {
		LOG_DEBUG("target was already halted");
		return ERROR_OK;
	}

	if (target->state == TARGET_UNKNOWN)
		LOG_WARNING("target was in unknown state when halt was requested");

	if (target->state == TARGET_RESET) {
		if ((jtag_get_reset_config() & RESET_SRST_PULLS_TRST) && jtag_get_srst()) {
			LOG_ERROR("can't request a halt while in reset if nSRST pulls nTRST");
			return ERROR_TARGET_FAILURE;
		} else {
			/* we came here in a reset_halt or reset_init sequence
			 * debug entry was already prepared in cortex_m3_assert_reset()
			 */
			target->debug_reason = DBG_REASON_DBGRQ;

			return ERROR_OK;
		}
	}

	/* Write to Debug Halting Control and Status Register */
	cortex_m3_write_debug_halt_mask(target, C_HALT, 0);

	target->debug_reason = DBG_REASON_DBGRQ;

	return ERROR_OK;
}

static int cortex_m3_soft_reset_halt(struct target *target)
{
	struct cortex_m3_common *cortex_m3 = target_to_cm3(target);
	struct adiv5_dap *swjdp = cortex_m3->armv7m.arm.dap;
	uint32_t dcb_dhcsr = 0;
	int retval, timeout = 0;

	/* Enter debug state on reset; restore DEMCR in endreset_event() */
	retval = mem_ap_write_u32(swjdp, DCB_DEMCR,
			TRCENA | VC_HARDERR | VC_BUSERR | VC_CORERESET);
	if (retval != ERROR_OK)
		return retval;

	/* Request a core-only reset */
	retval = mem_ap_write_atomic_u32(swjdp, NVIC_AIRCR,
			AIRCR_VECTKEY | AIRCR_VECTRESET);
	if (retval != ERROR_OK)
		return retval;
	target->state = TARGET_RESET;

	/* registers are now invalid */
	register_cache_invalidate(cortex_m3->armv7m.core_cache);

	while (timeout < 100) {
		retval = mem_ap_read_atomic_u32(swjdp, DCB_DHCSR, &dcb_dhcsr);
		if (retval == ERROR_OK) {
			retval = mem_ap_read_atomic_u32(swjdp, NVIC_DFSR,
					&cortex_m3->nvic_dfsr);
			if (retval != ERROR_OK)
				return retval;
			if ((dcb_dhcsr & S_HALT)
				&& (cortex_m3->nvic_dfsr & DFSR_VCATCH)) {
				LOG_DEBUG("system reset-halted, DHCSR 0x%08x, "
					"DFSR 0x%08x",
					(unsigned) dcb_dhcsr,
					(unsigned) cortex_m3->nvic_dfsr);
				cortex_m3_poll(target);
				/* FIXME restore user's vector catch config */
				return ERROR_OK;
			} else
				LOG_DEBUG("waiting for system reset-halt, "
					"DHCSR 0x%08x, %d ms",
					(unsigned) dcb_dhcsr, timeout);
		}
		timeout++;
		alive_sleep(1);
	}

	return ERROR_OK;
}

static void cortex_m3_enable_breakpoints(struct target *target)
{
	struct breakpoint *breakpoint = target->breakpoints;

	/* set any pending breakpoints */
	while (breakpoint) {
		if (!breakpoint->set)
			cortex_m3_set_breakpoint(target, breakpoint);
		breakpoint = breakpoint->next;
	}
}

static int cortex_m3_resume(struct target *target, int current,
	uint32_t address, int handle_breakpoints, int debug_execution)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct breakpoint *breakpoint = NULL;
	uint32_t resume_pc;
	struct reg *r;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!debug_execution) {
		target_free_all_working_areas(target);
		cortex_m3_enable_breakpoints(target);
		cortex_m3_enable_watchpoints(target);
	}

	if (debug_execution) {
		r = armv7m->core_cache->reg_list + ARMV7M_PRIMASK;

		/* Disable interrupts */
		/* We disable interrupts in the PRIMASK register instead of
		 * masking with C_MASKINTS.  This is probably the same issue
		 * as Cortex-M3 Erratum 377493 (fixed in r1p0):  C_MASKINTS
		 * in parallel with disabled interrupts can cause local faults
		 * to not be taken.
		 *
		 * REVISIT this clearly breaks non-debug execution, since the
		 * PRIMASK register state isn't saved/restored...  workaround
		 * by never resuming app code after debug execution.
		 */
		buf_set_u32(r->value, 0, 1, 1);
		r->dirty = true;
		r->valid = true;

		/* Make sure we are in Thumb mode */
		r = armv7m->core_cache->reg_list + ARMV7M_xPSR;
		buf_set_u32(r->value, 24, 1, 1);
		r->dirty = true;
		r->valid = true;
	}

	/* current = 1: continue on current pc, otherwise continue at <address> */
	r = armv7m->arm.pc;
	if (!current) {
		buf_set_u32(r->value, 0, 32, address);
		r->dirty = true;
		r->valid = true;
	}

	/* if we halted last time due to a bkpt instruction
	 * then we have to manually step over it, otherwise
	 * the core will break again */

	if (!breakpoint_find(target, buf_get_u32(r->value, 0, 32))
		&& !debug_execution)
		armv7m_maybe_skip_bkpt_inst(target, NULL);

	resume_pc = buf_get_u32(r->value, 0, 32);

	armv7m_restore_context(target);

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		/* Single step past breakpoint at current address */
		breakpoint = breakpoint_find(target, resume_pc);
		if (breakpoint) {
			LOG_DEBUG("unset breakpoint at 0x%8.8" PRIx32 " (ID: %d)",
				breakpoint->address,
				breakpoint->unique_id);
			cortex_m3_unset_breakpoint(target, breakpoint);
			cortex_m3_single_step_core(target);
			cortex_m3_set_breakpoint(target, breakpoint);
		}
	}

	/* Restart core */
	cortex_m3_write_debug_halt_mask(target, 0, C_HALT);

	target->debug_reason = DBG_REASON_NOTHALTED;

	/* registers are now invalid */
	register_cache_invalidate(armv7m->core_cache);

	if (!debug_execution) {
		target->state = TARGET_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		LOG_DEBUG("target resumed at 0x%" PRIx32 "", resume_pc);
	} else {
		target->state = TARGET_DEBUG_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED);
		LOG_DEBUG("target debug resumed at 0x%" PRIx32 "", resume_pc);
	}

	return ERROR_OK;
}

/* int irqstepcount = 0; */
static int cortex_m3_step(struct target *target, int current,
	uint32_t address, int handle_breakpoints)
{
	struct cortex_m3_common *cortex_m3 = target_to_cm3(target);
	struct armv7m_common *armv7m = &cortex_m3->armv7m;
	struct adiv5_dap *swjdp = armv7m->arm.dap;
	struct breakpoint *breakpoint = NULL;
	struct reg *pc = armv7m->arm.pc;
	bool bkpt_inst_found = false;
	int retval;
	bool isr_timed_out = false;

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current)
		buf_set_u32(pc->value, 0, 32, address);

	uint32_t pc_value = buf_get_u32(pc->value, 0, 32);

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints) {
		breakpoint = breakpoint_find(target, pc_value);
		if (breakpoint)
			cortex_m3_unset_breakpoint(target, breakpoint);
	}

	armv7m_maybe_skip_bkpt_inst(target, &bkpt_inst_found);

	target->debug_reason = DBG_REASON_SINGLESTEP;

	armv7m_restore_context(target);

	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);

	/* if no bkpt instruction is found at pc then we can perform
	 * a normal step, otherwise we have to manually step over the bkpt
	 * instruction - as such simulate a step */
	if (bkpt_inst_found == false) {
		/* Automatic ISR masking mode off: Just step over the next instruction */
		if ((cortex_m3->isrmasking_mode != CORTEX_M3_ISRMASK_AUTO))
			cortex_m3_write_debug_halt_mask(target, C_STEP, C_HALT);
		else {
			/* Process interrupts during stepping in a way they don't interfere
			 * debugging.
			 *
			 * Principle:
			 *
			 * Set a temporary break point at the current pc and let the core run
			 * with interrupts enabled. Pending interrupts get served and we run
			 * into the breakpoint again afterwards. Then we step over the next
			 * instruction with interrupts disabled.
			 *
			 * If the pending interrupts don't complete within time, we leave the
			 * core running. This may happen if the interrupts trigger faster
			 * than the core can process them or the handler doesn't return.
			 *
			 * If no more breakpoints are available we simply do a step with
			 * interrupts enabled.
			 *
			 */

			/* Set a temporary break point */
			retval = breakpoint_add(target, pc_value, 2, BKPT_TYPE_BY_ADDR(pc_value));
			bool tmp_bp_set = (retval == ERROR_OK);

			/* No more breakpoints left, just do a step */
			if (!tmp_bp_set)
				cortex_m3_write_debug_halt_mask(target, C_STEP, C_HALT);
			else {
				/* Start the core */
				LOG_DEBUG("Starting core to serve pending interrupts");
				int64_t t_start = timeval_ms();
				cortex_m3_write_debug_halt_mask(target, 0, C_HALT | C_STEP);

				/* Wait for pending handlers to complete or timeout */
				do {
					retval = mem_ap_read_atomic_u32(swjdp,
							DCB_DHCSR,
							&cortex_m3->dcb_dhcsr);
					if (retval != ERROR_OK) {
						target->state = TARGET_UNKNOWN;
						return retval;
					}
					isr_timed_out = ((timeval_ms() - t_start) > 500);
				} while (!((cortex_m3->dcb_dhcsr & S_HALT) || isr_timed_out));

				/* Remove the temporary breakpoint */
				breakpoint_remove(target, pc_value);

				if (isr_timed_out) {
					LOG_DEBUG("Interrupt handlers didn't complete within time, "
						"leaving target running");
				} else {
					/* Step over next instruction with interrupts disabled */
					cortex_m3_write_debug_halt_mask(target,
						C_HALT | C_MASKINTS,
						0);
					cortex_m3_write_debug_halt_mask(target, C_STEP, C_HALT);
					/* Re-enable interrupts */
					cortex_m3_write_debug_halt_mask(target, C_HALT, C_MASKINTS);
				}
			}
		}
	}

	retval = mem_ap_read_atomic_u32(swjdp, DCB_DHCSR, &cortex_m3->dcb_dhcsr);
	if (retval != ERROR_OK)
		return retval;

	/* registers are now invalid */
	register_cache_invalidate(cortex_m3->armv7m.core_cache);

	if (breakpoint)
		cortex_m3_set_breakpoint(target, breakpoint);

	if (isr_timed_out) {
		/* Leave the core running. The user has to stop execution manually. */
		target->debug_reason = DBG_REASON_NOTHALTED;
		target->state = TARGET_RUNNING;
		return ERROR_OK;
	}

	LOG_DEBUG("target stepped dcb_dhcsr = 0x%" PRIx32
		" nvic_icsr = 0x%" PRIx32,
		cortex_m3->dcb_dhcsr, cortex_m3->nvic_icsr);

	retval = cortex_m3_debug_entry(target);
	if (retval != ERROR_OK)
		return retval;
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);

	LOG_DEBUG("target stepped dcb_dhcsr = 0x%" PRIx32
		" nvic_icsr = 0x%" PRIx32,
		cortex_m3->dcb_dhcsr, cortex_m3->nvic_icsr);

	return ERROR_OK;
}

static int cortex_m3_assert_reset(struct target *target)
{
	struct cortex_m3_common *cortex_m3 = target_to_cm3(target);
	struct adiv5_dap *swjdp = cortex_m3->armv7m.arm.dap;
	enum cortex_m3_soft_reset_config reset_config = cortex_m3->soft_reset_config;

	LOG_DEBUG("target->state: %s",
		target_state_name(target));

	enum reset_types jtag_reset_config = jtag_get_reset_config();

	if (target_has_event_action(target, TARGET_EVENT_RESET_ASSERT)) {
		/* allow scripts to override the reset event */

		target_handle_event(target, TARGET_EVENT_RESET_ASSERT);
		register_cache_invalidate(cortex_m3->armv7m.core_cache);
		target->state = TARGET_RESET;

		return ERROR_OK;
	}

	/* some cores support connecting while srst is asserted
	 * use that mode is it has been configured */

	bool srst_asserted = false;

	if (jtag_reset_config & RESET_SRST_NO_GATING) {
		adapter_assert_reset();
		srst_asserted = true;
	}

	/* Enable debug requests */
	int retval;
	retval = mem_ap_read_atomic_u32(swjdp, DCB_DHCSR, &cortex_m3->dcb_dhcsr);
	if (retval != ERROR_OK)
		return retval;
	if (!(cortex_m3->dcb_dhcsr & C_DEBUGEN)) {
		retval = mem_ap_write_u32(swjdp, DCB_DHCSR, DBGKEY | C_DEBUGEN);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = mem_ap_write_u32(swjdp, DCB_DCRDR, 0);
	if (retval != ERROR_OK)
		return retval;

	if (!target->reset_halt) {
		/* Set/Clear C_MASKINTS in a separate operation */
		if (cortex_m3->dcb_dhcsr & C_MASKINTS) {
			retval = mem_ap_write_atomic_u32(swjdp, DCB_DHCSR,
					DBGKEY | C_DEBUGEN | C_HALT);
			if (retval != ERROR_OK)
				return retval;
		}

		/* clear any debug flags before resuming */
		cortex_m3_clear_halt(target);

		/* clear C_HALT in dhcsr reg */
		cortex_m3_write_debug_halt_mask(target, 0, C_HALT);
	} else {
		/* Halt in debug on reset; endreset_event() restores DEMCR.
		 *
		 * REVISIT catching BUSERR presumably helps to defend against
		 * bad vector table entries.  Should this include MMERR or
		 * other flags too?
		 */
		retval = mem_ap_write_atomic_u32(swjdp, DCB_DEMCR,
				TRCENA | VC_HARDERR | VC_BUSERR | VC_CORERESET);
		if (retval != ERROR_OK)
			return retval;
	}

	if (jtag_reset_config & RESET_HAS_SRST) {
		/* default to asserting srst */
		if (!srst_asserted)
			adapter_assert_reset();
	} else {
		/* Use a standard Cortex-M3 software reset mechanism.
		 * We default to using VECRESET as it is supported on all current cores.
		 * This has the disadvantage of not resetting the peripherals, so a
		 * reset-init event handler is needed to perform any peripheral resets.
		 */
		retval = mem_ap_write_atomic_u32(swjdp, NVIC_AIRCR,
				AIRCR_VECTKEY | ((reset_config == CORTEX_M3_RESET_SYSRESETREQ)
				? AIRCR_SYSRESETREQ : AIRCR_VECTRESET));
		if (retval != ERROR_OK)
			return retval;

		LOG_DEBUG("Using Cortex-M3 %s", (reset_config == CORTEX_M3_RESET_SYSRESETREQ)
			? "SYSRESETREQ" : "VECTRESET");

		if (reset_config == CORTEX_M3_RESET_VECTRESET) {
			LOG_WARNING("Only resetting the Cortex-M3 core, use a reset-init event "
				"handler to reset any peripherals or configure hardware srst support.");
		}

		{
			/* I do not know why this is necessary, but it
			 * fixes strange effects (step/resume cause NMI
			 * after reset) on LM3S6918 -- Michael Schwingen
			 */
			uint32_t tmp;
			retval = mem_ap_read_atomic_u32(swjdp, NVIC_AIRCR, &tmp);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	target->state = TARGET_RESET;
	jtag_add_sleep(50000);

	register_cache_invalidate(cortex_m3->armv7m.core_cache);

	if (target->reset_halt) {
		retval = target_halt(target);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int cortex_m3_deassert_reset(struct target *target)
{
	LOG_DEBUG("target->state: %s",
		target_state_name(target));

	/* deassert reset lines */
	adapter_deassert_reset();

	return ERROR_OK;
}

int cortex_m3_set_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	int retval;
	int fp_num = 0;
	uint32_t hilo;
	struct cortex_m3_common *cortex_m3 = target_to_cm3(target);
	struct cortex_m3_fp_comparator *comparator_list = cortex_m3->fp_comparator_list;

	if (breakpoint->set) {
		LOG_WARNING("breakpoint (BPID: %d) already set", breakpoint->unique_id);
		return ERROR_OK;
	}

	if (cortex_m3->auto_bp_type)
		breakpoint->type = BKPT_TYPE_BY_ADDR(breakpoint->address);

	if (breakpoint->type == BKPT_HARD) {
		while (comparator_list[fp_num].used && (fp_num < cortex_m3->fp_num_code))
			fp_num++;
		if (fp_num >= cortex_m3->fp_num_code) {
			LOG_ERROR("Can not find free FPB Comparator!");
			return ERROR_FAIL;
		}
		breakpoint->set = fp_num + 1;
		hilo = (breakpoint->address & 0x2) ? FPCR_REPLACE_BKPT_HIGH : FPCR_REPLACE_BKPT_LOW;
		comparator_list[fp_num].used = 1;
		comparator_list[fp_num].fpcr_value = (breakpoint->address & 0x1FFFFFFC) | hilo | 1;
		target_write_u32(target, comparator_list[fp_num].fpcr_address,
			comparator_list[fp_num].fpcr_value);
		LOG_DEBUG("fpc_num %i fpcr_value 0x%" PRIx32 "",
			fp_num,
			comparator_list[fp_num].fpcr_value);
		if (!cortex_m3->fpb_enabled) {
			LOG_DEBUG("FPB wasn't enabled, do it now");
			target_write_u32(target, FP_CTRL, 3);
		}
	} else if (breakpoint->type == BKPT_SOFT) {
		uint8_t code[4];

		/* NOTE: on ARMv6-M and ARMv7-M, BKPT(0xab) is used for
		 * semihosting; don't use that.  Otherwise the BKPT
		 * parameter is arbitrary.
		 */
		buf_set_u32(code, 0, 32, ARMV5_T_BKPT(0x11));
		retval = target_read_memory(target,
				breakpoint->address & 0xFFFFFFFE,
				breakpoint->length, 1,
				breakpoint->orig_instr);
		if (retval != ERROR_OK)
			return retval;
		retval = target_write_memory(target,
				breakpoint->address & 0xFFFFFFFE,
				breakpoint->length, 1,
				code);
		if (retval != ERROR_OK)
			return retval;
		breakpoint->set = true;
	}

	LOG_DEBUG("BPID: %d, Type: %d, Address: 0x%08" PRIx32 " Length: %d (set=%d)",
		breakpoint->unique_id,
		(int)(breakpoint->type),
		breakpoint->address,
		breakpoint->length,
		breakpoint->set);

	return ERROR_OK;
}

int cortex_m3_unset_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	int retval;
	struct cortex_m3_common *cortex_m3 = target_to_cm3(target);
	struct cortex_m3_fp_comparator *comparator_list = cortex_m3->fp_comparator_list;

	if (!breakpoint->set) {
		LOG_WARNING("breakpoint not set");
		return ERROR_OK;
	}

	LOG_DEBUG("BPID: %d, Type: %d, Address: 0x%08" PRIx32 " Length: %d (set=%d)",
		breakpoint->unique_id,
		(int)(breakpoint->type),
		breakpoint->address,
		breakpoint->length,
		breakpoint->set);

	if (breakpoint->type == BKPT_HARD) {
		int fp_num = breakpoint->set - 1;
		if ((fp_num < 0) || (fp_num >= cortex_m3->fp_num_code)) {
			LOG_DEBUG("Invalid FP Comparator number in breakpoint");
			return ERROR_OK;
		}
		comparator_list[fp_num].used = 0;
		comparator_list[fp_num].fpcr_value = 0;
		target_write_u32(target, comparator_list[fp_num].fpcr_address,
			comparator_list[fp_num].fpcr_value);
	} else {
		/* restore original instruction (kept in target endianness) */
		if (breakpoint->length == 4) {
			retval = target_write_memory(target, breakpoint->address & 0xFFFFFFFE, 4, 1,
					breakpoint->orig_instr);
			if (retval != ERROR_OK)
				return retval;
		} else {
			retval = target_write_memory(target, breakpoint->address & 0xFFFFFFFE, 2, 1,
					breakpoint->orig_instr);
			if (retval != ERROR_OK)
				return retval;
		}
	}
	breakpoint->set = false;

	return ERROR_OK;
}

int cortex_m3_add_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct cortex_m3_common *cortex_m3 = target_to_cm3(target);

	if (cortex_m3->auto_bp_type) {
		breakpoint->type = BKPT_TYPE_BY_ADDR(breakpoint->address);
#ifdef ARMV7_GDB_HACKS
		if (breakpoint->length != 2) {
			/* XXX Hack: Replace all breakpoints with length != 2 with
			 * a hardware breakpoint. */
			breakpoint->type = BKPT_HARD;
			breakpoint->length = 2;
		}
#endif
	}

	if (breakpoint->type != BKPT_TYPE_BY_ADDR(breakpoint->address)) {
		if (breakpoint->type == BKPT_HARD) {
			LOG_INFO("flash patch comparator requested outside code memory region");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}

		if (breakpoint->type == BKPT_SOFT) {
			LOG_INFO("soft breakpoint requested in code (flash) memory region");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	if ((breakpoint->type == BKPT_HARD) && (cortex_m3->fp_code_available < 1)) {
		LOG_INFO("no flash patch comparator unit available for hardware breakpoint");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if ((breakpoint->length != 2)) {
		LOG_INFO("only breakpoints of two bytes length supported");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (breakpoint->type == BKPT_HARD)
		cortex_m3->fp_code_available--;

	return cortex_m3_set_breakpoint(target, breakpoint);
}

int cortex_m3_remove_breakpoint(struct target *target, struct breakpoint *breakpoint)
{
	struct cortex_m3_common *cortex_m3 = target_to_cm3(target);

	/* REVISIT why check? FBP can be updated with core running ... */
	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (cortex_m3->auto_bp_type)
		breakpoint->type = BKPT_TYPE_BY_ADDR(breakpoint->address);

	if (breakpoint->set)
		cortex_m3_unset_breakpoint(target, breakpoint);

	if (breakpoint->type == BKPT_HARD)
		cortex_m3->fp_code_available++;

	return ERROR_OK;
}

int cortex_m3_set_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	int dwt_num = 0;
	uint32_t mask, temp;
	struct cortex_m3_common *cortex_m3 = target_to_cm3(target);

	/* watchpoint params were validated earlier */
	mask = 0;
	temp = watchpoint->length;
	while (temp) {
		temp >>= 1;
		mask++;
	}
	mask--;

	/* REVISIT Don't fully trust these "not used" records ... users
	 * may set up breakpoints by hand, e.g. dual-address data value
	 * watchpoint using comparator #1; comparator #0 matching cycle
	 * count; send data trace info through ITM and TPIU; etc
	 */
	struct cortex_m3_dwt_comparator *comparator;

	for (comparator = cortex_m3->dwt_comparator_list;
		comparator->used && dwt_num < cortex_m3->dwt_num_comp;
		comparator++, dwt_num++)
		continue;
	if (dwt_num >= cortex_m3->dwt_num_comp) {
		LOG_ERROR("Can not find free DWT Comparator");
		return ERROR_FAIL;
	}
	comparator->used = 1;
	watchpoint->set = dwt_num + 1;

	comparator->comp = watchpoint->address;
	target_write_u32(target, comparator->dwt_comparator_address + 0,
		comparator->comp);

	comparator->mask = mask;
	target_write_u32(target, comparator->dwt_comparator_address + 4,
		comparator->mask);

	switch (watchpoint->rw) {
		case WPT_READ:
			comparator->function = 5;
			break;
		case WPT_WRITE:
			comparator->function = 6;
			break;
		case WPT_ACCESS:
			comparator->function = 7;
			break;
	}
	target_write_u32(target, comparator->dwt_comparator_address + 8,
		comparator->function);

	LOG_DEBUG("Watchpoint (ID %d) DWT%d 0x%08x 0x%x 0x%05x",
		watchpoint->unique_id, dwt_num,
		(unsigned) comparator->comp,
		(unsigned) comparator->mask,
		(unsigned) comparator->function);
	return ERROR_OK;
}

int cortex_m3_unset_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	struct cortex_m3_common *cortex_m3 = target_to_cm3(target);
	struct cortex_m3_dwt_comparator *comparator;
	int dwt_num;

	if (!watchpoint->set) {
		LOG_WARNING("watchpoint (wpid: %d) not set",
			watchpoint->unique_id);
		return ERROR_OK;
	}

	dwt_num = watchpoint->set - 1;

	LOG_DEBUG("Watchpoint (ID %d) DWT%d address: 0x%08x clear",
		watchpoint->unique_id, dwt_num,
		(unsigned) watchpoint->address);

	if ((dwt_num < 0) || (dwt_num >= cortex_m3->dwt_num_comp)) {
		LOG_DEBUG("Invalid DWT Comparator number in watchpoint");
		return ERROR_OK;
	}

	comparator = cortex_m3->dwt_comparator_list + dwt_num;
	comparator->used = 0;
	comparator->function = 0;
	target_write_u32(target, comparator->dwt_comparator_address + 8,
		comparator->function);

	watchpoint->set = false;

	return ERROR_OK;
}

int cortex_m3_add_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	struct cortex_m3_common *cortex_m3 = target_to_cm3(target);

	if (cortex_m3->dwt_comp_available < 1) {
		LOG_DEBUG("no comparators?");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* hardware doesn't support data value masking */
	if (watchpoint->mask != ~(uint32_t)0) {
		LOG_DEBUG("watchpoint value masks not supported");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* hardware allows address masks of up to 32K */
	unsigned mask;

	for (mask = 0; mask < 16; mask++) {
		if ((1u << mask) == watchpoint->length)
			break;
	}
	if (mask == 16) {
		LOG_DEBUG("unsupported watchpoint length");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	if (watchpoint->address & ((1 << mask) - 1)) {
		LOG_DEBUG("watchpoint address is unaligned");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* Caller doesn't seem to be able to describe watching for data
	 * values of zero; that flags "no value".
	 *
	 * REVISIT This DWT may well be able to watch for specific data
	 * values.  Requires comparator #1 to set DATAVMATCH and match
	 * the data, and another comparator (DATAVADDR0) matching addr.
	 */
	if (watchpoint->value) {
		LOG_DEBUG("data value watchpoint not YET supported");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	cortex_m3->dwt_comp_available--;
	LOG_DEBUG("dwt_comp_available: %d", cortex_m3->dwt_comp_available);

	return ERROR_OK;
}

int cortex_m3_remove_watchpoint(struct target *target, struct watchpoint *watchpoint)
{
	struct cortex_m3_common *cortex_m3 = target_to_cm3(target);

	/* REVISIT why check? DWT can be updated with core running ... */
	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (watchpoint->set)
		cortex_m3_unset_watchpoint(target, watchpoint);

	cortex_m3->dwt_comp_available++;
	LOG_DEBUG("dwt_comp_available: %d", cortex_m3->dwt_comp_available);

	return ERROR_OK;
}

void cortex_m3_enable_watchpoints(struct target *target)
{
	struct watchpoint *watchpoint = target->watchpoints;

	/* set any pending watchpoints */
	while (watchpoint) {
		if (!watchpoint->set)
			cortex_m3_set_watchpoint(target, watchpoint);
		watchpoint = watchpoint->next;
	}
}

static int cortex_m3_load_core_reg_u32(struct target *target,
	enum armv7m_regtype type, uint32_t num, uint32_t *value)
{
	int retval;
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct adiv5_dap *swjdp = armv7m->arm.dap;

	/* NOTE:  we "know" here that the register identifiers used
	 * in the v7m header match the Cortex-M3 Debug Core Register
	 * Selector values for R0..R15, xPSR, MSP, and PSP.
	 */
	switch (num) {
		case 0 ... 18:
			/* read a normal core register */
			retval = cortexm3_dap_read_coreregister_u32(swjdp, value, num);

			if (retval != ERROR_OK) {
				LOG_ERROR("JTAG failure %i", retval);
				return ERROR_JTAG_DEVICE_ERROR;
			}
			LOG_DEBUG("load from core reg %i  value 0x%" PRIx32 "", (int)num, *value);
			break;

		case ARMV7M_PRIMASK:
		case ARMV7M_BASEPRI:
		case ARMV7M_FAULTMASK:
		case ARMV7M_CONTROL:
			/* Cortex-M3 packages these four registers as bitfields
			 * in one Debug Core register.  So say r0 and r2 docs;
			 * it was removed from r1 docs, but still works.
			 */
			cortexm3_dap_read_coreregister_u32(swjdp, value, 20);

			switch (num) {
				case ARMV7M_PRIMASK:
					*value = buf_get_u32((uint8_t *)value, 0, 1);
					break;

				case ARMV7M_BASEPRI:
					*value = buf_get_u32((uint8_t *)value, 8, 8);
					break;

				case ARMV7M_FAULTMASK:
					*value = buf_get_u32((uint8_t *)value, 16, 1);
					break;

				case ARMV7M_CONTROL:
					*value = buf_get_u32((uint8_t *)value, 24, 2);
					break;
			}

			LOG_DEBUG("load from special reg %i value 0x%" PRIx32 "", (int)num, *value);
			break;

		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}

static int cortex_m3_store_core_reg_u32(struct target *target,
	enum armv7m_regtype type, uint32_t num, uint32_t value)
{
	int retval;
	uint32_t reg;
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct adiv5_dap *swjdp = armv7m->arm.dap;

#ifdef ARMV7_GDB_HACKS
	/* If the LR register is being modified, make sure it will put us
	 * in "thumb" mode, or an INVSTATE exception will occur. This is a
	 * hack to deal with the fact that gdb will sometimes "forge"
	 * return addresses, and doesn't set the LSB correctly (i.e., when
	 * printing expressions containing function calls, it sets LR = 0.)
	 * Valid exception return codes have bit 0 set too.
	 */
	if (num == ARMV7M_R14)
		value |= 0x01;
#endif

	/* NOTE:  we "know" here that the register identifiers used
	 * in the v7m header match the Cortex-M3 Debug Core Register
	 * Selector values for R0..R15, xPSR, MSP, and PSP.
	 */
	switch (num) {
		case 0 ... 18:
			retval = cortexm3_dap_write_coreregister_u32(swjdp, value, num);
			if (retval != ERROR_OK) {
				struct reg *r;

				LOG_ERROR("JTAG failure");
				r = armv7m->core_cache->reg_list + num;
				r->dirty = r->valid;
				return ERROR_JTAG_DEVICE_ERROR;
			}
			LOG_DEBUG("write core reg %i value 0x%" PRIx32 "", (int)num, value);
			break;

		case ARMV7M_PRIMASK:
		case ARMV7M_BASEPRI:
		case ARMV7M_FAULTMASK:
		case ARMV7M_CONTROL:
			/* Cortex-M3 packages these four registers as bitfields
			 * in one Debug Core register.  So say r0 and r2 docs;
			 * it was removed from r1 docs, but still works.
			 */
			cortexm3_dap_read_coreregister_u32(swjdp, &reg, 20);

			switch (num) {
				case ARMV7M_PRIMASK:
					buf_set_u32((uint8_t *)&reg, 0, 1, value);
					break;

				case ARMV7M_BASEPRI:
					buf_set_u32((uint8_t *)&reg, 8, 8, value);
					break;

				case ARMV7M_FAULTMASK:
					buf_set_u32((uint8_t *)&reg, 16, 1, value);
					break;

				case ARMV7M_CONTROL:
					buf_set_u32((uint8_t *)&reg, 24, 2, value);
					break;
			}

			cortexm3_dap_write_coreregister_u32(swjdp, reg, 20);

			LOG_DEBUG("write special reg %i value 0x%" PRIx32 " ", (int)num, value);
			break;

		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}

static int cortex_m3_read_memory(struct target *target, uint32_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct adiv5_dap *swjdp = armv7m->arm.dap;
	int retval = ERROR_COMMAND_SYNTAX_ERROR;

	if (armv7m->arm.is_armv6m) {
		/* armv6m does not handle unaligned memory access */
		if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u)))
			return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	/* cortex_m3 handles unaligned memory access */
	if (count && buffer) {
		switch (size) {
			case 4:
				retval = mem_ap_read_buf_u32(swjdp, buffer, 4 * count, address);
				break;
			case 2:
				retval = mem_ap_read_buf_u16(swjdp, buffer, 2 * count, address);
				break;
			case 1:
				retval = mem_ap_read_buf_u8(swjdp, buffer, count, address);
				break;
		}
	}

	return retval;
}

static int cortex_m3_write_memory(struct target *target, uint32_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct adiv5_dap *swjdp = armv7m->arm.dap;
	int retval = ERROR_COMMAND_SYNTAX_ERROR;

	if (armv7m->arm.is_armv6m) {
		/* armv6m does not handle unaligned memory access */
		if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u)))
			return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	if (count && buffer) {
		switch (size) {
			case 4:
				retval = mem_ap_write_buf_u32(swjdp, buffer, 4 * count, address);
				break;
			case 2:
				retval = mem_ap_write_buf_u16(swjdp, buffer, 2 * count, address);
				break;
			case 1:
				retval = mem_ap_write_buf_u8(swjdp, buffer, count, address);
				break;
		}
	}

	return retval;
}

static int cortex_m3_bulk_write_memory(struct target *target, uint32_t address,
	uint32_t count, const uint8_t *buffer)
{
	return cortex_m3_write_memory(target, address, 4, count, buffer);
}

static int cortex_m3_init_target(struct command_context *cmd_ctx,
	struct target *target)
{
	armv7m_build_reg_cache(target);
	return ERROR_OK;
}

/* REVISIT cache valid/dirty bits are unmaintained.  We could set "valid"
 * on r/w if the core is not running, and clear on resume or reset ... or
 * at least, in a post_restore_context() method.
 */

struct dwt_reg_state {
	struct target *target;
	uint32_t addr;
	uint32_t value;		/* scratch/cache */
};

static int cortex_m3_dwt_get_reg(struct reg *reg)
{
	struct dwt_reg_state *state = reg->arch_info;

	return target_read_u32(state->target, state->addr, &state->value);
}

static int cortex_m3_dwt_set_reg(struct reg *reg, uint8_t *buf)
{
	struct dwt_reg_state *state = reg->arch_info;

	return target_write_u32(state->target, state->addr,
			buf_get_u32(buf, 0, reg->size));
}

struct dwt_reg {
	uint32_t addr;
	char *name;
	unsigned size;
};

static struct dwt_reg dwt_base_regs[] = {
	{ DWT_CTRL, "dwt_ctrl", 32, },
	/* NOTE that Erratum 532314 (fixed r2p0) affects CYCCNT:  it wrongly
	 * increments while the core is asleep.
	 */
	{ DWT_CYCCNT, "dwt_cyccnt", 32, },
	/* plus some 8 bit counters, useful for profiling with TPIU */
};

static struct dwt_reg dwt_comp[] = {
#define DWT_COMPARATOR(i) \
		{ DWT_COMP0 + 0x10 * (i), "dwt_" #i "_comp", 32, }, \
		{ DWT_MASK0 + 0x10 * (i), "dwt_" #i "_mask", 4, }, \
		{ DWT_FUNCTION0 + 0x10 * (i), "dwt_" #i "_function", 32, }
	DWT_COMPARATOR(0),
	DWT_COMPARATOR(1),
	DWT_COMPARATOR(2),
	DWT_COMPARATOR(3),
#undef DWT_COMPARATOR
};

static const struct reg_arch_type dwt_reg_type = {
	.get = cortex_m3_dwt_get_reg,
	.set = cortex_m3_dwt_set_reg,
};

static void cortex_m3_dwt_addreg(struct target *t, struct reg *r, struct dwt_reg *d)
{
	struct dwt_reg_state *state;

	state = calloc(1, sizeof *state);
	if (!state)
		return;
	state->addr = d->addr;
	state->target = t;

	r->name = d->name;
	r->size = d->size;
	r->value = &state->value;
	r->arch_info = state;
	r->type = &dwt_reg_type;
}

void cortex_m3_dwt_setup(struct cortex_m3_common *cm3, struct target *target)
{
	uint32_t dwtcr;
	struct reg_cache *cache;
	struct cortex_m3_dwt_comparator *comparator;
	int reg, i;

	target_read_u32(target, DWT_CTRL, &dwtcr);
	if (!dwtcr) {
		LOG_DEBUG("no DWT");
		return;
	}

	cm3->dwt_num_comp = (dwtcr >> 28) & 0xF;
	cm3->dwt_comp_available = cm3->dwt_num_comp;
	cm3->dwt_comparator_list = calloc(cm3->dwt_num_comp,
			sizeof(struct cortex_m3_dwt_comparator));
	if (!cm3->dwt_comparator_list) {
fail0:
		cm3->dwt_num_comp = 0;
		LOG_ERROR("out of mem");
		return;
	}

	cache = calloc(1, sizeof *cache);
	if (!cache) {
fail1:
		free(cm3->dwt_comparator_list);
		goto fail0;
	}
	cache->name = "cortex-m3 dwt registers";
	cache->num_regs = 2 + cm3->dwt_num_comp * 3;
	cache->reg_list = calloc(cache->num_regs, sizeof *cache->reg_list);
	if (!cache->reg_list) {
		free(cache);
		goto fail1;
	}

	for (reg = 0; reg < 2; reg++)
		cortex_m3_dwt_addreg(target, cache->reg_list + reg,
			dwt_base_regs + reg);

	comparator = cm3->dwt_comparator_list;
	for (i = 0; i < cm3->dwt_num_comp; i++, comparator++) {
		int j;

		comparator->dwt_comparator_address = DWT_COMP0 + 0x10 * i;
		for (j = 0; j < 3; j++, reg++)
			cortex_m3_dwt_addreg(target, cache->reg_list + reg,
				dwt_comp + 3 * i + j);
	}

	*register_get_last_cache_p(&target->reg_cache) = cache;
	cm3->dwt_cache = cache;

	LOG_DEBUG("DWT dwtcr 0x%" PRIx32 ", comp %d, watch%s",
		dwtcr, cm3->dwt_num_comp,
		(dwtcr & (0xf << 24)) ? " only" : "/trigger");

	/* REVISIT:  if num_comp > 1, check whether comparator #1 can
	 * implement single-address data value watchpoints ... so we
	 * won't need to check it later, when asked to set one up.
	 */
}

#define MVFR0 0xe000ef40
#define MVFR1 0xe000ef44

#define MVFR0_DEFAULT_M4 0x10110021
#define MVFR1_DEFAULT_M4 0x11000011

int cortex_m3_examine(struct target *target)
{
	int retval;
	uint32_t cpuid, fpcr, mvfr0, mvfr1;
	int i;
	struct cortex_m3_common *cortex_m3 = target_to_cm3(target);
	struct adiv5_dap *swjdp = cortex_m3->armv7m.arm.dap;
	struct armv7m_common *armv7m = target_to_armv7m(target);

	/* stlink shares the examine handler but does not support
	 * all its calls */
	if (!armv7m->stlink) {
		retval = ahbap_debugport_init(swjdp);
		if (retval != ERROR_OK)
			return retval;
	}

	if (!target_was_examined(target)) {
		target_set_examined(target);

		/* Read from Device Identification Registers */
		retval = target_read_u32(target, CPUID, &cpuid);
		if (retval != ERROR_OK)
			return retval;

		/* Get CPU Type */
		i = (cpuid >> 4) & 0xf;

		LOG_DEBUG("Cortex-M%d r%" PRId8 "p%" PRId8 " processor detected",
				i, (uint8_t)((cpuid >> 20) & 0xf), (uint8_t)((cpuid >> 0) & 0xf));
		LOG_DEBUG("cpuid: 0x%8.8" PRIx32 "", cpuid);

		/* test for floating point feature on cortex-m4 */
		if (i == 4) {
			target_read_u32(target, MVFR0, &mvfr0);
			target_read_u32(target, MVFR1, &mvfr1);

			if ((mvfr0 == MVFR0_DEFAULT_M4) && (mvfr1 == MVFR1_DEFAULT_M4)) {
				LOG_DEBUG("Cortex-M%d floating point feature FPv4_SP found", i);
				armv7m->fp_feature = FPv4_SP;
			}
		} else if (i == 0) {
			/* Cortex-M0 does not support unaligned memory access */
			armv7m->arm.is_armv6m = true;
		}

		if (i == 4 || i == 3) {
			/* Cortex-M3/M4 has 4096 bytes autoincrement range */
			armv7m->dap.tar_autoincr_block = (1 << 12);
		}

		/* NOTE: FPB and DWT are both optional. */

		/* Setup FPB */
		target_read_u32(target, FP_CTRL, &fpcr);
		cortex_m3->auto_bp_type = 1;
		cortex_m3->fp_num_code = ((fpcr >> 8) & 0x70) | ((fpcr >> 4) & 0xF);	/* bits
											 *[14:12]
											 *and [7:4]
											 **/
		cortex_m3->fp_num_lit = (fpcr >> 8) & 0xF;
		cortex_m3->fp_code_available = cortex_m3->fp_num_code;
		cortex_m3->fp_comparator_list = calloc(
				cortex_m3->fp_num_code + cortex_m3->fp_num_lit,
				sizeof(struct cortex_m3_fp_comparator));
		cortex_m3->fpb_enabled = fpcr & 1;
		for (i = 0; i < cortex_m3->fp_num_code + cortex_m3->fp_num_lit; i++) {
			cortex_m3->fp_comparator_list[i].type =
				(i < cortex_m3->fp_num_code) ? FPCR_CODE : FPCR_LITERAL;
			cortex_m3->fp_comparator_list[i].fpcr_address = FP_COMP0 + 4 * i;
		}
		LOG_DEBUG("FPB fpcr 0x%" PRIx32 ", numcode %i, numlit %i",
			fpcr,
			cortex_m3->fp_num_code,
			cortex_m3->fp_num_lit);

		/* Setup DWT */
		cortex_m3_dwt_setup(cortex_m3, target);

		/* These hardware breakpoints only work for code in flash! */
		LOG_INFO("%s: hardware has %d breakpoints, %d watchpoints",
			target_name(target),
			cortex_m3->fp_num_code,
			cortex_m3->dwt_num_comp);
	}

	return ERROR_OK;
}

static int cortex_m3_dcc_read(struct adiv5_dap *swjdp, uint8_t *value, uint8_t *ctrl)
{
	uint16_t dcrdr;
	int retval;

	mem_ap_read_buf_u16(swjdp, (uint8_t *)&dcrdr, 1, DCB_DCRDR);
	*ctrl = (uint8_t)dcrdr;
	*value = (uint8_t)(dcrdr >> 8);

	LOG_DEBUG("data 0x%x ctrl 0x%x", *value, *ctrl);

	/* write ack back to software dcc register
	 * signify we have read data */
	if (dcrdr & (1 << 0)) {
		dcrdr = 0;
		retval = mem_ap_write_buf_u16(swjdp, (uint8_t *)&dcrdr, 1, DCB_DCRDR);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int cortex_m3_target_request_data(struct target *target,
	uint32_t size, uint8_t *buffer)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct adiv5_dap *swjdp = armv7m->arm.dap;
	uint8_t data;
	uint8_t ctrl;
	uint32_t i;

	for (i = 0; i < (size * 4); i++) {
		cortex_m3_dcc_read(swjdp, &data, &ctrl);
		buffer[i] = data;
	}

	return ERROR_OK;
}

static int cortex_m3_handle_target_request(void *priv)
{
	struct target *target = priv;
	if (!target_was_examined(target))
		return ERROR_OK;
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct adiv5_dap *swjdp = armv7m->arm.dap;

	if (!target->dbg_msg_enabled)
		return ERROR_OK;

	if (target->state == TARGET_RUNNING) {
		uint8_t data;
		uint8_t ctrl;

		cortex_m3_dcc_read(swjdp, &data, &ctrl);

		/* check if we have data */
		if (ctrl & (1 << 0)) {
			uint32_t request;

			/* we assume target is quick enough */
			request = data;
			cortex_m3_dcc_read(swjdp, &data, &ctrl);
			request |= (data << 8);
			cortex_m3_dcc_read(swjdp, &data, &ctrl);
			request |= (data << 16);
			cortex_m3_dcc_read(swjdp, &data, &ctrl);
			request |= (data << 24);
			target_request(target, request);
		}
	}

	return ERROR_OK;
}

static int cortex_m3_init_arch_info(struct target *target,
	struct cortex_m3_common *cortex_m3, struct jtag_tap *tap)
{
	int retval;
	struct armv7m_common *armv7m = &cortex_m3->armv7m;

	armv7m_init_arch_info(target, armv7m);

	/* prepare JTAG information for the new target */
	cortex_m3->jtag_info.tap = tap;
	cortex_m3->jtag_info.scann_size = 4;

	/* default reset mode is to use srst if fitted
	 * if not it will use CORTEX_M3_RESET_VECTRESET */
	cortex_m3->soft_reset_config = CORTEX_M3_RESET_VECTRESET;

	armv7m->arm.dap = &armv7m->dap;

	/* Leave (only) generic DAP stuff for debugport_init(); */
	armv7m->dap.jtag_info = &cortex_m3->jtag_info;
	armv7m->dap.memaccess_tck = 8;

	/* Cortex-M3/M4 has 4096 bytes autoincrement range
	 * but set a safe default to 1024 to support Cortex-M0
	 * this will be changed in cortex_m3_examine if a M3/M4 is detected */
	armv7m->dap.tar_autoincr_block = (1 << 10);

	/* register arch-specific functions */
	armv7m->examine_debug_reason = cortex_m3_examine_debug_reason;

	armv7m->post_debug_entry = NULL;

	armv7m->pre_restore_context = NULL;

	armv7m->load_core_reg_u32 = cortex_m3_load_core_reg_u32;
	armv7m->store_core_reg_u32 = cortex_m3_store_core_reg_u32;

	target_register_timer_callback(cortex_m3_handle_target_request, 1, 1, target);

	retval = arm_jtag_setup_connection(&cortex_m3->jtag_info);
	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static int cortex_m3_target_create(struct target *target, Jim_Interp *interp)
{
	struct cortex_m3_common *cortex_m3 = calloc(1, sizeof(struct cortex_m3_common));

	cortex_m3->common_magic = CORTEX_M3_COMMON_MAGIC;
	cortex_m3_init_arch_info(target, cortex_m3, target->tap);

	return ERROR_OK;
}

/*--------------------------------------------------------------------------*/

static int cortex_m3_verify_pointer(struct command_context *cmd_ctx,
	struct cortex_m3_common *cm3)
{
	if (cm3->common_magic != CORTEX_M3_COMMON_MAGIC) {
		command_print(cmd_ctx, "target is not a Cortex-M3");
		return ERROR_TARGET_INVALID;
	}
	return ERROR_OK;
}

/*
 * Only stuff below this line should need to verify that its target
 * is a Cortex-M3.  Everything else should have indirected through the
 * cortexm3_target structure, which is only used with CM3 targets.
 */

static const struct {
	char name[10];
	unsigned mask;
} vec_ids[] = {
	{ "hard_err",   VC_HARDERR, },
	{ "int_err",    VC_INTERR, },
	{ "bus_err",    VC_BUSERR, },
	{ "state_err",  VC_STATERR, },
	{ "chk_err",    VC_CHKERR, },
	{ "nocp_err",   VC_NOCPERR, },
	{ "mm_err",     VC_MMERR, },
	{ "reset",      VC_CORERESET, },
};

COMMAND_HANDLER(handle_cortex_m3_vector_catch_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct cortex_m3_common *cortex_m3 = target_to_cm3(target);
	struct armv7m_common *armv7m = &cortex_m3->armv7m;
	struct adiv5_dap *swjdp = armv7m->arm.dap;
	uint32_t demcr = 0;
	int retval;

	retval = cortex_m3_verify_pointer(CMD_CTX, cortex_m3);
	if (retval != ERROR_OK)
		return retval;

	retval = mem_ap_read_atomic_u32(swjdp, DCB_DEMCR, &demcr);
	if (retval != ERROR_OK)
		return retval;

	if (CMD_ARGC > 0) {
		unsigned catch = 0;

		if (CMD_ARGC == 1) {
			if (strcmp(CMD_ARGV[0], "all") == 0) {
				catch = VC_HARDERR | VC_INTERR | VC_BUSERR
					| VC_STATERR | VC_CHKERR | VC_NOCPERR
					| VC_MMERR | VC_CORERESET;
				goto write;
			} else if (strcmp(CMD_ARGV[0], "none") == 0)
				goto write;
		}
		while (CMD_ARGC-- > 0) {
			unsigned i;
			for (i = 0; i < ARRAY_SIZE(vec_ids); i++) {
				if (strcmp(CMD_ARGV[CMD_ARGC], vec_ids[i].name) != 0)
					continue;
				catch |= vec_ids[i].mask;
				break;
			}
			if (i == ARRAY_SIZE(vec_ids)) {
				LOG_ERROR("No CM3 vector '%s'", CMD_ARGV[CMD_ARGC]);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}
		}
write:
		/* For now, armv7m->demcr only stores vector catch flags. */
		armv7m->demcr = catch;

		demcr &= ~0xffff;
		demcr |= catch;

		/* write, but don't assume it stuck (why not??) */
		retval = mem_ap_write_u32(swjdp, DCB_DEMCR, demcr);
		if (retval != ERROR_OK)
			return retval;
		retval = mem_ap_read_atomic_u32(swjdp, DCB_DEMCR, &demcr);
		if (retval != ERROR_OK)
			return retval;

		/* FIXME be sure to clear DEMCR on clean server shutdown.
		 * Otherwise the vector catch hardware could fire when there's
		 * no debugger hooked up, causing much confusion...
		 */
	}

	for (unsigned i = 0; i < ARRAY_SIZE(vec_ids); i++) {
		command_print(CMD_CTX, "%9s: %s", vec_ids[i].name,
			(demcr & vec_ids[i].mask) ? "catch" : "ignore");
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_cortex_m3_mask_interrupts_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct cortex_m3_common *cortex_m3 = target_to_cm3(target);
	int retval;

	static const Jim_Nvp nvp_maskisr_modes[] = {
		{ .name = "auto", .value = CORTEX_M3_ISRMASK_AUTO },
		{ .name = "off", .value = CORTEX_M3_ISRMASK_OFF },
		{ .name = "on", .value = CORTEX_M3_ISRMASK_ON },
		{ .name = NULL, .value = -1 },
	};
	const Jim_Nvp *n;


	retval = cortex_m3_verify_pointer(CMD_CTX, cortex_m3);
	if (retval != ERROR_OK)
		return retval;

	if (target->state != TARGET_HALTED) {
		command_print(CMD_CTX, "target must be stopped for \"%s\" command", CMD_NAME);
		return ERROR_OK;
	}

	if (CMD_ARGC > 0) {
		n = Jim_Nvp_name2value_simple(nvp_maskisr_modes, CMD_ARGV[0]);
		if (n->name == NULL)
			return ERROR_COMMAND_SYNTAX_ERROR;
		cortex_m3->isrmasking_mode = n->value;


		if (cortex_m3->isrmasking_mode == CORTEX_M3_ISRMASK_ON)
			cortex_m3_write_debug_halt_mask(target, C_HALT | C_MASKINTS, 0);
		else
			cortex_m3_write_debug_halt_mask(target, C_HALT, C_MASKINTS);
	}

	n = Jim_Nvp_value2name_simple(nvp_maskisr_modes, cortex_m3->isrmasking_mode);
	command_print(CMD_CTX, "cortex_m3 interrupt mask %s", n->name);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_cortex_m3_reset_config_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct cortex_m3_common *cortex_m3 = target_to_cm3(target);
	int retval;
	char *reset_config;

	retval = cortex_m3_verify_pointer(CMD_CTX, cortex_m3);
	if (retval != ERROR_OK)
		return retval;

	if (CMD_ARGC > 0) {
		if (strcmp(*CMD_ARGV, "sysresetreq") == 0)
			cortex_m3->soft_reset_config = CORTEX_M3_RESET_SYSRESETREQ;
		else if (strcmp(*CMD_ARGV, "vectreset") == 0)
			cortex_m3->soft_reset_config = CORTEX_M3_RESET_VECTRESET;
	}

	switch (cortex_m3->soft_reset_config) {
		case CORTEX_M3_RESET_SYSRESETREQ:
			reset_config = "sysresetreq";
			break;

		case CORTEX_M3_RESET_VECTRESET:
			reset_config = "vectreset";
			break;

		default:
			reset_config = "unknown";
			break;
	}

	command_print(CMD_CTX, "cortex_m3 reset_config %s", reset_config);

	return ERROR_OK;
}

static const struct command_registration cortex_m3_exec_command_handlers[] = {
	{
		.name = "maskisr",
		.handler = handle_cortex_m3_mask_interrupts_command,
		.mode = COMMAND_EXEC,
		.help = "mask cortex_m3 interrupts",
		.usage = "['auto'|'on'|'off']",
	},
	{
		.name = "vector_catch",
		.handler = handle_cortex_m3_vector_catch_command,
		.mode = COMMAND_EXEC,
		.help = "configure hardware vectors to trigger debug entry",
		.usage = "['all'|'none'|('bus_err'|'chk_err'|...)*]",
	},
	{
		.name = "reset_config",
		.handler = handle_cortex_m3_reset_config_command,
		.mode = COMMAND_ANY,
		.help = "configure software reset handling",
		.usage = "['srst'|'sysresetreq'|'vectreset']",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration cortex_m3_command_handlers[] = {
	{
		.chain = armv7m_command_handlers,
	},
	{
		.name = "cortex_m3",
		.mode = COMMAND_EXEC,
		.help = "Cortex-M3 command group",
		.usage = "",
		.chain = cortex_m3_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct target_type cortexm3_target = {
	.name = "cortex_m3",

	.poll = cortex_m3_poll,
	.arch_state = armv7m_arch_state,

	.target_request_data = cortex_m3_target_request_data,

	.halt = cortex_m3_halt,
	.resume = cortex_m3_resume,
	.step = cortex_m3_step,

	.assert_reset = cortex_m3_assert_reset,
	.deassert_reset = cortex_m3_deassert_reset,
	.soft_reset_halt = cortex_m3_soft_reset_halt,

	.get_gdb_reg_list = armv7m_get_gdb_reg_list,

	.read_memory = cortex_m3_read_memory,
	.write_memory = cortex_m3_write_memory,
	.bulk_write_memory = cortex_m3_bulk_write_memory,
	.checksum_memory = armv7m_checksum_memory,
	.blank_check_memory = armv7m_blank_check_memory,

	.run_algorithm = armv7m_run_algorithm,
	.start_algorithm = armv7m_start_algorithm,
	.wait_algorithm = armv7m_wait_algorithm,

	.add_breakpoint = cortex_m3_add_breakpoint,
	.remove_breakpoint = cortex_m3_remove_breakpoint,
	.add_watchpoint = cortex_m3_add_watchpoint,
	.remove_watchpoint = cortex_m3_remove_watchpoint,

	.commands = cortex_m3_command_handlers,
	.target_create = cortex_m3_target_create,
	.init_target = cortex_m3_init_target,
	.examine = cortex_m3_examine,
};
