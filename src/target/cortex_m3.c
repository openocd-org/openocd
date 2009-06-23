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
 *   Cortex-M3(tm) TRM, ARM DDI 0337C                                      *
 *                                                                         *
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "cortex_m3.h"
#include "target_request.h"
#include "target_type.h"


/* cli handling */
int cortex_m3_register_commands(struct command_context_s *cmd_ctx);
int handle_cortex_m3_mask_interrupts_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

/* forward declarations */
void cortex_m3_enable_breakpoints(struct target_s *target);
void cortex_m3_enable_watchpoints(struct target_s *target);
int cortex_m3_target_create(struct target_s *target, Jim_Interp *interp);
int cortex_m3_init_target(struct command_context_s *cmd_ctx, struct target_s *target);
int cortex_m3_quit(void);
int cortex_m3_load_core_reg_u32(target_t *target, enum armv7m_regtype type, uint32_t num, uint32_t *value);
int cortex_m3_store_core_reg_u32(target_t *target, enum armv7m_regtype type, uint32_t num, uint32_t value);
int cortex_m3_target_request_data(target_t *target, uint32_t size, uint8_t *buffer);
int cortex_m3_examine(struct target_s *target);

#ifdef ARMV7_GDB_HACKS
extern uint8_t armv7m_gdb_dummy_cpsr_value[];
extern reg_t armv7m_gdb_dummy_cpsr_reg;
#endif

target_type_t cortexm3_target =
{
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

	.add_breakpoint = cortex_m3_add_breakpoint,
	.remove_breakpoint = cortex_m3_remove_breakpoint,
	.add_watchpoint = cortex_m3_add_watchpoint,
	.remove_watchpoint = cortex_m3_remove_watchpoint,

	.register_commands = cortex_m3_register_commands,
	.target_create = cortex_m3_target_create,
	.init_target = cortex_m3_init_target,
	.examine = cortex_m3_examine,
	.quit = cortex_m3_quit
};

int cortexm3_dap_read_coreregister_u32(swjdp_common_t *swjdp, uint32_t *value, int regnum)
{
	int retval;
	uint32_t dcrdr;

	/* because the DCB_DCRDR is used for the emulated dcc channel
	 * we gave to save/restore the DCB_DCRDR when used */

	mem_ap_read_u32(swjdp, DCB_DCRDR, &dcrdr);

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	/* mem_ap_write_u32(swjdp, DCB_DCRSR, regnum); */
	dap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_OFF, DCB_DCRSR & 0xFFFFFFF0);
	dap_ap_write_reg_u32(swjdp, AP_REG_BD0 | (DCB_DCRSR & 0xC), regnum );

	/* mem_ap_read_u32(swjdp, DCB_DCRDR, value); */
	dap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_OFF, DCB_DCRDR & 0xFFFFFFF0);
	dap_ap_read_reg_u32(swjdp, AP_REG_BD0 | (DCB_DCRDR & 0xC), value );

	mem_ap_write_u32(swjdp, DCB_DCRDR, dcrdr);
	retval = swjdp_transaction_endcheck(swjdp);
	return retval;
}

int cortexm3_dap_write_coreregister_u32(swjdp_common_t *swjdp, uint32_t value, int regnum)
{
	int retval;
	uint32_t dcrdr;

	/* because the DCB_DCRDR is used for the emulated dcc channel
	 * we gave to save/restore the DCB_DCRDR when used */

	mem_ap_read_u32(swjdp, DCB_DCRDR, &dcrdr);

	swjdp->trans_mode = TRANS_MODE_COMPOSITE;

	/* mem_ap_write_u32(swjdp, DCB_DCRDR, core_regs[i]); */
	dap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_OFF, DCB_DCRDR & 0xFFFFFFF0);
	dap_ap_write_reg_u32(swjdp, AP_REG_BD0 | (DCB_DCRDR & 0xC), value );

	/* mem_ap_write_u32(swjdp, DCB_DCRSR, i | DCRSR_WnR	); */
	dap_setup_accessport(swjdp, CSW_32BIT | CSW_ADDRINC_OFF, DCB_DCRSR & 0xFFFFFFF0);
	dap_ap_write_reg_u32(swjdp, AP_REG_BD0 | (DCB_DCRSR & 0xC), regnum | DCRSR_WnR );

	mem_ap_write_u32(swjdp, DCB_DCRDR, dcrdr);
	retval = swjdp_transaction_endcheck(swjdp);
	return retval;
}


int cortex_m3_write_debug_halt_mask(target_t *target, uint32_t mask_on, uint32_t mask_off)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;

	/* mask off status bits */
	cortex_m3->dcb_dhcsr &= ~((0xFFFF << 16) | mask_off);
	/* create new register mask */
	cortex_m3->dcb_dhcsr |= DBGKEY | C_DEBUGEN | mask_on;

	return mem_ap_write_atomic_u32(swjdp, DCB_DHCSR, cortex_m3->dcb_dhcsr);
}

int cortex_m3_clear_halt(target_t *target)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;

	/* clear step if any */
	cortex_m3_write_debug_halt_mask(target, C_HALT, C_STEP);

	/* Read Debug Fault Status Register */
	mem_ap_read_atomic_u32(swjdp, NVIC_DFSR, &cortex_m3->nvic_dfsr);
	/* Write Debug Fault Status Register to enable processing to resume ?? Try with and without this !! */
	mem_ap_write_atomic_u32(swjdp, NVIC_DFSR, cortex_m3->nvic_dfsr);
	LOG_DEBUG(" NVIC_DFSR 0x%" PRIx32 "", cortex_m3->nvic_dfsr);

	return ERROR_OK;
}

int cortex_m3_single_step_core(target_t *target)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;
	uint32_t dhcsr_save;

	/* backup dhcsr reg */
	dhcsr_save = cortex_m3->dcb_dhcsr;

	/* mask interrupts if not done already */
	if (!(cortex_m3->dcb_dhcsr & C_MASKINTS))
		mem_ap_write_atomic_u32(swjdp, DCB_DHCSR, DBGKEY | C_MASKINTS | C_HALT | C_DEBUGEN);
	mem_ap_write_atomic_u32(swjdp, DCB_DHCSR, DBGKEY | C_MASKINTS | C_STEP | C_DEBUGEN);
	LOG_DEBUG(" ");

	/* restore dhcsr reg */
	cortex_m3->dcb_dhcsr = dhcsr_save;
	cortex_m3_clear_halt(target);

	return ERROR_OK;
}

int cortex_m3_exec_opcode(target_t *target,uint32_t opcode, int len /* MODE, r0_invalue, &r0_outvalue */ )
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;
	uint32_t savedram;
	int retvalue;

	mem_ap_read_u32(swjdp, 0x20000000, &savedram);
	mem_ap_write_u32(swjdp, 0x20000000, opcode);
	cortexm3_dap_write_coreregister_u32(swjdp, 0x20000000, 15);
	cortex_m3_single_step_core(target);
	armv7m->core_cache->reg_list[15].dirty = armv7m->core_cache->reg_list[15].valid;
	retvalue = mem_ap_write_atomic_u32(swjdp, 0x20000000, savedram);

	return retvalue;
}

#if 0
/* Enable interrupts */
int cortex_m3_cpsie(target_t *target, uint32_t IF)
{
	return cortex_m3_exec_opcode(target, ARMV7M_T_CPSIE(IF), 2);
}

/* Disable interrupts */
int cortex_m3_cpsid(target_t *target, uint32_t IF)
{
	return cortex_m3_exec_opcode(target, ARMV7M_T_CPSID(IF), 2);
}
#endif

int cortex_m3_endreset_event(target_t *target)
{
	int i;
	uint32_t dcb_demcr;

	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;
	cortex_m3_fp_comparator_t *fp_list = cortex_m3->fp_comparator_list;
	cortex_m3_dwt_comparator_t *dwt_list = cortex_m3->dwt_comparator_list;

	mem_ap_read_atomic_u32(swjdp, DCB_DEMCR, &dcb_demcr);
	LOG_DEBUG("DCB_DEMCR = 0x%8.8" PRIx32 "",dcb_demcr);

	/* this regsiter is used for emulated dcc channel */
	mem_ap_write_u32(swjdp, DCB_DCRDR, 0);

	/* Enable debug requests */
	mem_ap_read_atomic_u32(swjdp, DCB_DHCSR, &cortex_m3->dcb_dhcsr);
	if (!(cortex_m3->dcb_dhcsr & C_DEBUGEN))
		mem_ap_write_u32(swjdp, DCB_DHCSR, DBGKEY | C_DEBUGEN);

	/* clear any interrupt masking */
	cortex_m3_write_debug_halt_mask(target, 0, C_MASKINTS);

	/* Enable trace and dwt */
	mem_ap_write_u32(swjdp, DCB_DEMCR, TRCENA | VC_HARDERR | VC_BUSERR);
	/* Monitor bus faults */
	mem_ap_write_u32(swjdp, NVIC_SHCSR, SHCSR_BUSFAULTENA);

	/* Enable FPB */
	target_write_u32(target, FP_CTRL, 3);
	cortex_m3->fpb_enabled = 1;

	/* Restore FPB registers */
	for (i = 0; i < cortex_m3->fp_num_code + cortex_m3->fp_num_lit; i++)
	{
		target_write_u32(target, fp_list[i].fpcr_address, fp_list[i].fpcr_value);
	}

	/* Restore DWT registers */
	for (i = 0; i < cortex_m3->dwt_num_comp; i++)
	{
		target_write_u32(target, dwt_list[i].dwt_comparator_address, dwt_list[i].comp);
		target_write_u32(target, dwt_list[i].dwt_comparator_address | 0x4, dwt_list[i].mask);
		target_write_u32(target, dwt_list[i].dwt_comparator_address | 0x8, dwt_list[i].function);
	}
	swjdp_transaction_endcheck(swjdp);

	armv7m_invalidate_core_regs(target);

	/* make sure we have latest dhcsr flags */
	mem_ap_read_atomic_u32(swjdp, DCB_DHCSR, &cortex_m3->dcb_dhcsr);

	return ERROR_OK;
}

int cortex_m3_examine_debug_reason(target_t *target)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;

	/* THIS IS NOT GOOD, TODO - better logic for detection of debug state reason */
	/* only check the debug reason if we don't know it already */

	if ((target->debug_reason != DBG_REASON_DBGRQ)
		&& (target->debug_reason != DBG_REASON_SINGLESTEP))
	{
		/*  INCOMPLETE */

		if (cortex_m3->nvic_dfsr & DFSR_BKPT)
		{
			target->debug_reason = DBG_REASON_BREAKPOINT;
			if (cortex_m3->nvic_dfsr & DFSR_DWTTRAP)
				target->debug_reason = DBG_REASON_WPTANDBKPT;
		}
		else if (cortex_m3->nvic_dfsr & DFSR_DWTTRAP)
			target->debug_reason = DBG_REASON_WATCHPOINT;
	}

	return ERROR_OK;
}

int cortex_m3_examine_exception_reason(target_t *target)
{
	uint32_t shcsr, except_sr, cfsr = -1, except_ar = -1;

	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;

	mem_ap_read_u32(swjdp, NVIC_SHCSR, &shcsr);
	switch (armv7m->exception_number)
	{
		case 2:	/* NMI */
			break;
		case 3:	/* Hard Fault */
			mem_ap_read_atomic_u32(swjdp, NVIC_HFSR, &except_sr);
			if (except_sr & 0x40000000)
			{
				mem_ap_read_u32(swjdp, NVIC_CFSR, &cfsr);
			}
			break;
		case 4:	/* Memory Management */
			mem_ap_read_u32(swjdp, NVIC_CFSR, &except_sr);
			mem_ap_read_u32(swjdp, NVIC_MMFAR, &except_ar);
			break;
		case 5:	/* Bus Fault */
			mem_ap_read_u32(swjdp, NVIC_CFSR, &except_sr);
			mem_ap_read_u32(swjdp, NVIC_BFAR, &except_ar);
			break;
		case 6:	/* Usage Fault */
			mem_ap_read_u32(swjdp, NVIC_CFSR, &except_sr);
			break;
		case 11:	/* SVCall */
			break;
		case 12:	/* Debug Monitor */
			mem_ap_read_u32(swjdp, NVIC_DFSR, &except_sr);
			break;
		case 14:	/* PendSV */
			break;
		case 15:	/* SysTick */
			break;
		default:
			except_sr = 0;
			break;
	}
	swjdp_transaction_endcheck(swjdp);
	LOG_DEBUG("%s SHCSR 0x%" PRIx32 ", SR 0x%" PRIx32 ", CFSR 0x%" PRIx32 ", AR 0x%" PRIx32 "", armv7m_exception_string(armv7m->exception_number), \
		shcsr, except_sr, cfsr, except_ar);
	return ERROR_OK;
}

int cortex_m3_debug_entry(target_t *target)
{
	int i;
	uint32_t xPSR;
	int retval;

	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;

	LOG_DEBUG(" ");
	if (armv7m->pre_debug_entry)
		armv7m->pre_debug_entry(target);

	cortex_m3_clear_halt(target);
	mem_ap_read_atomic_u32(swjdp, DCB_DHCSR, &cortex_m3->dcb_dhcsr);

	if ((retval = armv7m->examine_debug_reason(target)) != ERROR_OK)
		return retval;

	/* Examine target state and mode */
	/* First load register acessible through core debug port*/
	for (i = 0; i < ARMV7M_PRIMASK; i++)
	{
		if (!armv7m->core_cache->reg_list[i].valid)
			armv7m->read_core_reg(target, i);
	}

	xPSR = buf_get_u32(armv7m->core_cache->reg_list[ARMV7M_xPSR].value, 0, 32);

#ifdef ARMV7_GDB_HACKS
	/* copy real xpsr reg for gdb, setting thumb bit */
	buf_set_u32(armv7m_gdb_dummy_cpsr_value, 0, 32, xPSR);
	buf_set_u32(armv7m_gdb_dummy_cpsr_value, 5, 1, 1);
	armv7m_gdb_dummy_cpsr_reg.valid = armv7m->core_cache->reg_list[ARMV7M_xPSR].valid;
	armv7m_gdb_dummy_cpsr_reg.dirty = armv7m->core_cache->reg_list[ARMV7M_xPSR].dirty;
#endif

	/* For IT instructions xPSR must be reloaded on resume and clear on debug exec */
	if (xPSR & 0xf00)
	{
		armv7m->core_cache->reg_list[ARMV7M_xPSR].dirty = armv7m->core_cache->reg_list[ARMV7M_xPSR].valid;
		cortex_m3_store_core_reg_u32(target, ARMV7M_REGISTER_CORE_GP, 16, xPSR &~ 0xff);
	}

	/* Now we can load SP core registers */
	for (i = ARMV7M_PRIMASK; i < ARMV7NUMCOREREGS; i++)
	{
		if (!armv7m->core_cache->reg_list[i].valid)
			armv7m->read_core_reg(target, i);
	}

	/* Are we in an exception handler */
	if (xPSR & 0x1FF)
	{
		armv7m->core_mode = ARMV7M_MODE_HANDLER;
		armv7m->exception_number = (xPSR & 0x1FF);
	}
	else
	{
		armv7m->core_mode = buf_get_u32(armv7m->core_cache->reg_list[ARMV7M_CONTROL].value, 0, 1);
		armv7m->exception_number = 0;
	}

	if (armv7m->exception_number)
	{
		cortex_m3_examine_exception_reason(target);
	}

	LOG_DEBUG("entered debug state in core mode: %s at PC 0x%" PRIx32 ", target->state: %s",
		armv7m_mode_strings[armv7m->core_mode],
		*(uint32_t*)(armv7m->core_cache->reg_list[15].value),
		Jim_Nvp_value2name_simple( nvp_target_state, target->state )->name);

	if (armv7m->post_debug_entry)
		armv7m->post_debug_entry(target);

	return ERROR_OK;
}

int cortex_m3_poll(target_t *target)
{
	int retval;
	enum target_state prev_target_state = target->state;

	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;

	/* Read from Debug Halting Control and Status Register */
	retval = mem_ap_read_atomic_u32(swjdp, DCB_DHCSR, &cortex_m3->dcb_dhcsr);
	if (retval != ERROR_OK)
	{
		target->state = TARGET_UNKNOWN;
		return retval;
	}

	if (cortex_m3->dcb_dhcsr & S_RESET_ST)
	{
		/* check if still in reset */
		mem_ap_read_atomic_u32(swjdp, DCB_DHCSR, &cortex_m3->dcb_dhcsr);

		if (cortex_m3->dcb_dhcsr & S_RESET_ST)
		{
			target->state = TARGET_RESET;
			return ERROR_OK;
		}
	}

	if (target->state == TARGET_RESET)
	{
		/* Cannot switch context while running so endreset is called with target->state == TARGET_RESET */
		LOG_DEBUG("Exit from reset with dcb_dhcsr 0x%" PRIx32 "", cortex_m3->dcb_dhcsr);
		cortex_m3_endreset_event(target);
		target->state = TARGET_RUNNING;
		prev_target_state = TARGET_RUNNING;
	}

	if (cortex_m3->dcb_dhcsr & S_HALT)
	{
		target->state = TARGET_HALTED;

		if ((prev_target_state == TARGET_RUNNING) || (prev_target_state == TARGET_RESET))
		{
			if ((retval = cortex_m3_debug_entry(target)) != ERROR_OK)
				return retval;

			target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		}
		if (prev_target_state == TARGET_DEBUG_RUNNING)
		{
			LOG_DEBUG(" ");
			if ((retval = cortex_m3_debug_entry(target)) != ERROR_OK)
				return retval;

			target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
		}
	}

	/*
	if (cortex_m3->dcb_dhcsr & S_SLEEP)
		target->state = TARGET_SLEEP;
	*/

#if 0
	/* Read Debug Fault Status Register, added to figure out the lockup when running flashtest.script  */
	mem_ap_read_atomic_u32(swjdp, NVIC_DFSR, &cortex_m3->nvic_dfsr);
	LOG_DEBUG("dcb_dhcsr 0x%x, nvic_dfsr 0x%x, target->state: %s", cortex_m3->dcb_dhcsr, cortex_m3->nvic_dfsr, Jim_Nvp_value2name_simple( nvp_target_state, target->state )->name );
#endif

	return ERROR_OK;
}

int cortex_m3_halt(target_t *target)
{
	LOG_DEBUG("target->state: %s",
		Jim_Nvp_value2name_simple(nvp_target_state, target->state )->name);

	if (target->state == TARGET_HALTED)
	{
		LOG_DEBUG("target was already halted");
		return ERROR_OK;
	}

	if (target->state == TARGET_UNKNOWN)
	{
		LOG_WARNING("target was in unknown state when halt was requested");
	}

	if (target->state == TARGET_RESET)
	{
		if ((jtag_get_reset_config() & RESET_SRST_PULLS_TRST) && jtag_get_srst())
		{
			LOG_ERROR("can't request a halt while in reset if nSRST pulls nTRST");
			return ERROR_TARGET_FAILURE;
		}
		else
		{
			/* we came here in a reset_halt or reset_init sequence
			 * debug entry was already prepared in cortex_m3_prepare_reset_halt()
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

int cortex_m3_soft_reset_halt(struct target_s *target)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;
	uint32_t dcb_dhcsr = 0;
	int retval, timeout = 0;

	/* Enter debug state on reset, cf. end_reset_event() */
	mem_ap_write_u32(swjdp, DCB_DEMCR, TRCENA | VC_HARDERR | VC_BUSERR | VC_CORERESET);

	/* Request a reset */
	mem_ap_write_atomic_u32(swjdp, NVIC_AIRCR, AIRCR_VECTKEY | AIRCR_VECTRESET);
	target->state = TARGET_RESET;

	/* registers are now invalid */
	armv7m_invalidate_core_regs(target);

	while (timeout < 100)
	{
		retval = mem_ap_read_atomic_u32(swjdp, DCB_DHCSR, &dcb_dhcsr);
		if (retval == ERROR_OK)
		{
			mem_ap_read_atomic_u32(swjdp, NVIC_DFSR, &cortex_m3->nvic_dfsr);
			if ((dcb_dhcsr & S_HALT) && (cortex_m3->nvic_dfsr & DFSR_VCATCH))
			{
				LOG_DEBUG("system reset-halted, dcb_dhcsr 0x%" PRIx32 ", nvic_dfsr 0x%" PRIx32 "", dcb_dhcsr, cortex_m3->nvic_dfsr);
				cortex_m3_poll(target);
				return ERROR_OK;
			}
			else
				LOG_DEBUG("waiting for system reset-halt, dcb_dhcsr 0x%" PRIx32 ", %i ms", dcb_dhcsr, timeout);
		}
		timeout++;
		alive_sleep(1);
	}

	return ERROR_OK;
}

int cortex_m3_resume(struct target_s *target, int current, uint32_t address, int handle_breakpoints, int debug_execution)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	breakpoint_t *breakpoint = NULL;
	uint32_t resume_pc;

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!debug_execution)
	{
		target_free_all_working_areas(target);
		cortex_m3_enable_breakpoints(target);
		cortex_m3_enable_watchpoints(target);
	}

	if (debug_execution)
	{
		/* Disable interrupts */
		/* We disable interrupts in the PRIMASK register instead of masking with C_MASKINTS,
		 * This is probably the same issue as Cortex-M3 Errata	377493:
		 * C_MASKINTS in parallel with disabled interrupts can cause local faults to not be taken. */
		buf_set_u32(armv7m->core_cache->reg_list[ARMV7M_PRIMASK].value, 0, 32, 1);
		armv7m->core_cache->reg_list[ARMV7M_PRIMASK].dirty = 1;
		armv7m->core_cache->reg_list[ARMV7M_PRIMASK].valid = 1;

		/* Make sure we are in Thumb mode */
		buf_set_u32(armv7m->core_cache->reg_list[ARMV7M_xPSR].value, 0, 32,
			buf_get_u32(armv7m->core_cache->reg_list[ARMV7M_xPSR].value, 0, 32) | (1<<24));
		armv7m->core_cache->reg_list[ARMV7M_xPSR].dirty = 1;
		armv7m->core_cache->reg_list[ARMV7M_xPSR].valid = 1;
	}

	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current)
	{
		buf_set_u32(armv7m->core_cache->reg_list[15].value, 0, 32, address);
		armv7m->core_cache->reg_list[15].dirty = 1;
		armv7m->core_cache->reg_list[15].valid = 1;
	}

	resume_pc = buf_get_u32(armv7m->core_cache->reg_list[15].value, 0, 32);

	armv7m_restore_context(target);

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints)
	{
		/* Single step past breakpoint at current address */
		if ((breakpoint = breakpoint_find(target, resume_pc)))
		{
			LOG_DEBUG("unset breakpoint at 0x%8.8" PRIx32 "", breakpoint->address);
			cortex_m3_unset_breakpoint(target, breakpoint);
			cortex_m3_single_step_core(target);
			cortex_m3_set_breakpoint(target, breakpoint);
		}
	}

	/* Restart core */
	cortex_m3_write_debug_halt_mask(target, 0, C_HALT);

	target->debug_reason = DBG_REASON_NOTHALTED;

	/* registers are now invalid */
	armv7m_invalidate_core_regs(target);
	if (!debug_execution)
	{
		target->state = TARGET_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		LOG_DEBUG("target resumed at 0x%" PRIx32 "", resume_pc);
	}
	else
	{
		target->state = TARGET_DEBUG_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED);
		LOG_DEBUG("target debug resumed at 0x%" PRIx32 "", resume_pc);
	}

	return ERROR_OK;
}

/* int irqstepcount=0; */
int cortex_m3_step(struct target_s *target, int current, uint32_t address, int handle_breakpoints)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;
	breakpoint_t *breakpoint = NULL;

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* current = 1: continue on current pc, otherwise continue at <address> */
	if (!current)
		buf_set_u32(armv7m->core_cache->reg_list[15].value, 0, 32, address);

	/* the front-end may request us not to handle breakpoints */
	if (handle_breakpoints)
		if ((breakpoint = breakpoint_find(target, buf_get_u32(armv7m->core_cache->reg_list[15].value, 0, 32))))
			cortex_m3_unset_breakpoint(target, breakpoint);

	target->debug_reason = DBG_REASON_SINGLESTEP;

	armv7m_restore_context(target);

	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);

	/* set step and clear halt */
	cortex_m3_write_debug_halt_mask(target, C_STEP, C_HALT);
	mem_ap_read_atomic_u32(swjdp, DCB_DHCSR, &cortex_m3->dcb_dhcsr);

	/* registers are now invalid */
	armv7m_invalidate_core_regs(target);

	if (breakpoint)
		cortex_m3_set_breakpoint(target, breakpoint);

	LOG_DEBUG("target stepped dcb_dhcsr = 0x%" PRIx32 " nvic_icsr = 0x%" PRIx32 "", cortex_m3->dcb_dhcsr, cortex_m3->nvic_icsr);

	cortex_m3_debug_entry(target);
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);

	LOG_DEBUG("target stepped dcb_dhcsr = 0x%" PRIx32 " nvic_icsr = 0x%" PRIx32 "", cortex_m3->dcb_dhcsr, cortex_m3->nvic_icsr);
	return ERROR_OK;
}

int cortex_m3_assert_reset(target_t *target)
{
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;
	int assert_srst = 1;

	LOG_DEBUG("target->state: %s",
		Jim_Nvp_value2name_simple( nvp_target_state, target->state )->name );

	enum reset_types jtag_reset_config = jtag_get_reset_config();
	if (!(jtag_reset_config & RESET_HAS_SRST))
	{
		LOG_ERROR("Can't assert SRST");
		return ERROR_FAIL;
	}

	/* Enable debug requests */
	mem_ap_read_atomic_u32(swjdp, DCB_DHCSR, &cortex_m3->dcb_dhcsr);
	if (!(cortex_m3->dcb_dhcsr & C_DEBUGEN))
		mem_ap_write_u32(swjdp, DCB_DHCSR, DBGKEY | C_DEBUGEN);

	mem_ap_write_u32(swjdp, DCB_DCRDR, 0 );

	if (!target->reset_halt)
	{
		/* Set/Clear C_MASKINTS in a separate operation */
		if (cortex_m3->dcb_dhcsr & C_MASKINTS)
			mem_ap_write_atomic_u32(swjdp, DCB_DHCSR, DBGKEY | C_DEBUGEN | C_HALT);

		/* clear any debug flags before resuming */
		cortex_m3_clear_halt(target);

		/* clear C_HALT in dhcsr reg */
		cortex_m3_write_debug_halt_mask(target, 0, C_HALT);

		/* Enter debug state on reset, cf. end_reset_event() */
		mem_ap_write_u32(swjdp, DCB_DEMCR, TRCENA | VC_HARDERR | VC_BUSERR);
	}
	else
	{
		/* Enter debug state on reset, cf. end_reset_event() */
		mem_ap_write_atomic_u32(swjdp, DCB_DEMCR, TRCENA | VC_HARDERR | VC_BUSERR | VC_CORERESET);
	}

	/* following hack is to handle luminary reset
	 * when srst is asserted the luminary device seesm to also clear the debug registers
	 * which does not match the armv7 debug TRM */

	if (strcmp(target->variant, "lm3s") == 0)
	{
		/* get revision of lm3s target, only early silicon has this issue
		 * Fury Rev B, DustDevil Rev B, Tempest all ok */

		uint32_t did0;

		if (target_read_u32(target, 0x400fe000, &did0) == ERROR_OK)
		{
			switch ((did0 >> 16) & 0xff)
			{
				case 0:
					/* all Sandstorm suffer issue */
					assert_srst = 0;
					break;

				case 1:
				case 3:
					/* only Fury/DustDevil rev A suffer reset problems */
					if (((did0 >> 8) & 0xff) == 0)
						assert_srst = 0;
					break;
			}
		}
	}

	if (assert_srst)
	{
		/* default to asserting srst */
		if (jtag_reset_config & RESET_SRST_PULLS_TRST)
		{
			jtag_add_reset(1, 1);
		}
		else
		{
			jtag_add_reset(0, 1);
		}
	}
	else
	{
		/* this causes the luminary device to reset using the watchdog */
		mem_ap_write_atomic_u32(swjdp, NVIC_AIRCR, AIRCR_VECTKEY | AIRCR_SYSRESETREQ);
		LOG_DEBUG("Using Luminary Reset: SYSRESETREQ");

		{
			/* I do not know why this is necessary, but it fixes strange effects
			 * (step/resume cause a NMI after reset) on LM3S6918 -- Michael Schwingen */
			uint32_t tmp;
			mem_ap_read_atomic_u32(swjdp, NVIC_AIRCR, &tmp);
		}
	}

	target->state = TARGET_RESET;
	jtag_add_sleep(50000);

	armv7m_invalidate_core_regs(target);

	if (target->reset_halt)
	{
		int retval;
		if ((retval = target_halt(target))!=ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

int cortex_m3_deassert_reset(target_t *target)
{
	LOG_DEBUG("target->state: %s",
		Jim_Nvp_value2name_simple(nvp_target_state, target->state )->name);

	/* deassert reset lines */
	jtag_add_reset(0, 0);

	return ERROR_OK;
}

void cortex_m3_enable_breakpoints(struct target_s *target)
{
	breakpoint_t *breakpoint = target->breakpoints;

	/* set any pending breakpoints */
	while (breakpoint)
	{
		if (breakpoint->set == 0)
			cortex_m3_set_breakpoint(target, breakpoint);
		breakpoint = breakpoint->next;
	}
}

int cortex_m3_set_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	int retval;
	int fp_num=0;
	uint32_t hilo;

	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;

	cortex_m3_fp_comparator_t * comparator_list = cortex_m3->fp_comparator_list;

	if (breakpoint->set)
	{
		LOG_WARNING("breakpoint already set");
		return ERROR_OK;
	}

	if (cortex_m3->auto_bp_type)
	{
		breakpoint->type = (breakpoint->address < 0x20000000) ? BKPT_HARD : BKPT_SOFT;
	}

	if (breakpoint->type == BKPT_HARD)
	{
		while(comparator_list[fp_num].used && (fp_num < cortex_m3->fp_num_code))
			fp_num++;
		if (fp_num >= cortex_m3->fp_num_code)
		{
			LOG_DEBUG("ERROR Can not find free FP Comparator");
			LOG_WARNING("ERROR Can not find free FP Comparator");
			exit(-1);
		}
		breakpoint->set = fp_num + 1;
		hilo = (breakpoint->address & 0x2) ? FPCR_REPLACE_BKPT_HIGH : FPCR_REPLACE_BKPT_LOW;
		comparator_list[fp_num].used = 1;
		comparator_list[fp_num].fpcr_value = (breakpoint->address & 0x1FFFFFFC) | hilo | 1;
		target_write_u32(target, comparator_list[fp_num].fpcr_address, comparator_list[fp_num].fpcr_value);
		LOG_DEBUG("fpc_num %i fpcr_value 0x%" PRIx32 "", fp_num, comparator_list[fp_num].fpcr_value);
		if (!cortex_m3->fpb_enabled)
		{
			LOG_DEBUG("FPB wasn't enabled, do it now");
			target_write_u32(target, FP_CTRL, 3);
		}
	}
	else if (breakpoint->type == BKPT_SOFT)
	{
		uint8_t code[4];
		buf_set_u32(code, 0, 32, ARMV7M_T_BKPT(0x11));
		if ((retval = target_read_memory(target, breakpoint->address & 0xFFFFFFFE, breakpoint->length, 1, breakpoint->orig_instr)) != ERROR_OK)
		{
			return retval;
		}
		if ((retval = target_write_memory(target, breakpoint->address & 0xFFFFFFFE, breakpoint->length, 1, code)) != ERROR_OK)
		{
			return retval;
		}
		breakpoint->set = 0x11; /* Any nice value but 0 */
	}

	return ERROR_OK;
}

int cortex_m3_unset_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	int retval;
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	cortex_m3_fp_comparator_t * comparator_list = cortex_m3->fp_comparator_list;

	if (!breakpoint->set)
	{
		LOG_WARNING("breakpoint not set");
		return ERROR_OK;
	}

	if (breakpoint->type == BKPT_HARD)
	{
		int fp_num = breakpoint->set - 1;
		if ((fp_num < 0) || (fp_num >= cortex_m3->fp_num_code))
		{
			LOG_DEBUG("Invalid FP Comparator number in breakpoint");
			return ERROR_OK;
		}
		comparator_list[fp_num].used = 0;
		comparator_list[fp_num].fpcr_value = 0;
		target_write_u32(target, comparator_list[fp_num].fpcr_address, comparator_list[fp_num].fpcr_value);
	}
	else
	{
		/* restore original instruction (kept in target endianness) */
		if (breakpoint->length == 4)
		{
			if ((retval = target_write_memory(target, breakpoint->address & 0xFFFFFFFE, 4, 1, breakpoint->orig_instr)) != ERROR_OK)
			{
				return retval;
			}
		}
		else
		{
			if ((retval = target_write_memory(target, breakpoint->address & 0xFFFFFFFE, 2, 1, breakpoint->orig_instr)) != ERROR_OK)
			{
				return retval;
			}
		}
	}
	breakpoint->set = 0;

	return ERROR_OK;
}

int cortex_m3_add_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;

	if (cortex_m3->auto_bp_type)
	{
		breakpoint->type = (breakpoint->address < 0x20000000) ? BKPT_HARD : BKPT_SOFT;
#ifdef ARMV7_GDB_HACKS
		if (breakpoint->length != 2) {
			/* XXX Hack: Replace all breakpoints with length != 2 with
			 * a hardware breakpoint. */
			breakpoint->type = BKPT_HARD;
			breakpoint->length = 2;
		}
#endif
	}

	if ((breakpoint->type == BKPT_HARD) && (breakpoint->address >= 0x20000000))
	{
		LOG_INFO("flash patch comparator requested outside code memory region");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if ((breakpoint->type == BKPT_SOFT) && (breakpoint->address < 0x20000000))
	{
		LOG_INFO("soft breakpoint requested in code (flash) memory region");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if ((breakpoint->type == BKPT_HARD) && (cortex_m3->fp_code_available < 1))
	{
		LOG_INFO("no flash patch comparator unit available for hardware breakpoint");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if ((breakpoint->length != 2))
	{
		LOG_INFO("only breakpoints of two bytes length supported");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (breakpoint->type == BKPT_HARD)
		cortex_m3->fp_code_available--;
	cortex_m3_set_breakpoint(target, breakpoint);

	return ERROR_OK;
}

int cortex_m3_remove_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (cortex_m3->auto_bp_type)
	{
		breakpoint->type = (breakpoint->address < 0x20000000) ? BKPT_HARD : BKPT_SOFT;
	}

	if (breakpoint->set)
	{
		cortex_m3_unset_breakpoint(target, breakpoint);
	}

	if (breakpoint->type == BKPT_HARD)
		cortex_m3->fp_code_available++;

	return ERROR_OK;
}

int cortex_m3_set_watchpoint(struct target_s *target, watchpoint_t *watchpoint)
{
	int dwt_num=0;
	uint32_t mask, temp;

	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	cortex_m3_dwt_comparator_t * comparator_list = cortex_m3->dwt_comparator_list;

	if (watchpoint->set)
	{
		LOG_WARNING("watchpoint already set");
		return ERROR_OK;
	}

	if (watchpoint->mask == 0xffffffffu)
	{
		while(comparator_list[dwt_num].used && (dwt_num < cortex_m3->dwt_num_comp))
			dwt_num++;
		if (dwt_num >= cortex_m3->dwt_num_comp)
		{
			LOG_DEBUG("ERROR Can not find free DWT Comparator");
			LOG_WARNING("ERROR Can not find free DWT Comparator");
			return -1;
		}
		watchpoint->set = dwt_num + 1;
		mask = 0;
		temp = watchpoint->length;
		while (temp > 1)
		{
			temp = temp / 2;
			mask++;
		}
		comparator_list[dwt_num].used = 1;
		comparator_list[dwt_num].comp = watchpoint->address;
		comparator_list[dwt_num].mask = mask;
		comparator_list[dwt_num].function = watchpoint->rw + 5;
		target_write_u32(target, comparator_list[dwt_num].dwt_comparator_address, comparator_list[dwt_num].comp);
		target_write_u32(target, comparator_list[dwt_num].dwt_comparator_address|0x4, comparator_list[dwt_num].mask);
		target_write_u32(target, comparator_list[dwt_num].dwt_comparator_address|0x8, comparator_list[dwt_num].function);
		LOG_DEBUG("dwt_num %i 0x%" PRIx32 " 0x%" PRIx32 " 0x%" PRIx32 "", dwt_num, comparator_list[dwt_num].comp, comparator_list[dwt_num].mask, comparator_list[dwt_num].function);
	}
	else
	{
		LOG_WARNING("Cannot watch data values");  /* Move this test to add_watchpoint */
		return ERROR_OK;
	}

	return ERROR_OK;

}

int cortex_m3_unset_watchpoint(struct target_s *target, watchpoint_t *watchpoint)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	cortex_m3_dwt_comparator_t * comparator_list = cortex_m3->dwt_comparator_list;
	int dwt_num;

	if (!watchpoint->set)
	{
		LOG_WARNING("watchpoint not set");
		return ERROR_OK;
	}

	dwt_num = watchpoint->set - 1;

	if ((dwt_num < 0) || (dwt_num >= cortex_m3->dwt_num_comp))
	{
		LOG_DEBUG("Invalid DWT Comparator number in watchpoint");
		return ERROR_OK;
	}
	comparator_list[dwt_num].used = 0;
	comparator_list[dwt_num].function = 0;
	target_write_u32(target, comparator_list[dwt_num].dwt_comparator_address|0x8, comparator_list[dwt_num].function);

	watchpoint->set = 0;

	return ERROR_OK;
}

int cortex_m3_add_watchpoint(struct target_s *target, watchpoint_t *watchpoint)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (cortex_m3->dwt_comp_available < 1)
	{
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if ((watchpoint->length != 1) && (watchpoint->length != 2) && (watchpoint->length != 4))
	{
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	cortex_m3->dwt_comp_available--;

	return ERROR_OK;
}

int cortex_m3_remove_watchpoint(struct target_s *target, watchpoint_t *watchpoint)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (watchpoint->set)
	{
		cortex_m3_unset_watchpoint(target, watchpoint);
	}

	cortex_m3->dwt_comp_available++;

	return ERROR_OK;
}

void cortex_m3_enable_watchpoints(struct target_s *target)
{
	watchpoint_t *watchpoint = target->watchpoints;

	/* set any pending watchpoints */
	while (watchpoint)
	{
		if (watchpoint->set == 0)
			cortex_m3_set_watchpoint(target, watchpoint);
		watchpoint = watchpoint->next;
	}
}

int cortex_m3_load_core_reg_u32(struct target_s *target, enum armv7m_regtype type, uint32_t num, uint32_t * value)
{
	int retval;
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;

	if ((type == ARMV7M_REGISTER_CORE_GP) && (num <= ARMV7M_PSP))
	{
		/* read a normal core register */
		retval = cortexm3_dap_read_coreregister_u32(swjdp, value, num);

		if (retval != ERROR_OK)
		{
			LOG_ERROR("JTAG failure %i",retval);
			return ERROR_JTAG_DEVICE_ERROR;
		}
		LOG_DEBUG("load from core reg %i  value 0x%" PRIx32 "",(int)num,*value);
	}
	else if (type == ARMV7M_REGISTER_CORE_SP) /* Special purpose core register */
	{
		/* read other registers */
		cortexm3_dap_read_coreregister_u32(swjdp, value, 20);

		switch (num)
		{
			case 19:
				*value = buf_get_u32((uint8_t*)value, 0, 8);
				break;

			case 20:
				*value = buf_get_u32((uint8_t*)value, 8, 8);
				break;

			case 21:
				*value = buf_get_u32((uint8_t*)value, 16, 8);
				break;

			case 22:
				*value = buf_get_u32((uint8_t*)value, 24, 8);
				break;
		}

		LOG_DEBUG("load from special reg %i value 0x%" PRIx32 "", (int)num, *value);
	}
	else
	{
		return ERROR_INVALID_ARGUMENTS;
	}

	return ERROR_OK;
}

int cortex_m3_store_core_reg_u32(struct target_s *target, enum armv7m_regtype type, uint32_t num, uint32_t value)
{
	int retval;
	uint32_t reg;

	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;

#ifdef ARMV7_GDB_HACKS
	/* If the LR register is being modified, make sure it will put us
	 * in "thumb" mode, or an INVSTATE exception will occur. This is a
	 * hack to deal with the fact that gdb will sometimes "forge"
	 * return addresses, and doesn't set the LSB correctly (i.e., when
	 * printing expressions containing function calls, it sets LR=0.) */

	if (num == 14)
		value |= 0x01;
#endif

	if ((type == ARMV7M_REGISTER_CORE_GP) && (num <= ARMV7M_PSP))
	{
		retval = cortexm3_dap_write_coreregister_u32(swjdp, value, num);
		if (retval != ERROR_OK)
		{
			LOG_ERROR("JTAG failure %i", retval);
			armv7m->core_cache->reg_list[num].dirty = armv7m->core_cache->reg_list[num].valid;
			return ERROR_JTAG_DEVICE_ERROR;
		}
		LOG_DEBUG("write core reg %i value 0x%" PRIx32 "", (int)num, value);
	}
	else if (type == ARMV7M_REGISTER_CORE_SP) /* Special purpose core register */
	{
		/* write other registers */

		cortexm3_dap_read_coreregister_u32(swjdp, &reg, 20);

		switch (num)
		{
			case 19:
				buf_set_u32((uint8_t*)&reg, 0, 8, value);
				break;

			case 20:
				buf_set_u32((uint8_t*)&reg, 8, 8, value);
				break;

			case 21:
				buf_set_u32((uint8_t*)&reg, 16, 8, value);
				break;

			case 22:
				buf_set_u32((uint8_t*)&reg, 24, 8, value);
				break;
		}

		cortexm3_dap_write_coreregister_u32(swjdp, reg, 20);

		LOG_DEBUG("write special reg %i value 0x%" PRIx32 " ", (int)num, value);
	}
	else
	{
		return ERROR_INVALID_ARGUMENTS;
	}

	return ERROR_OK;
}

int cortex_m3_read_memory(struct target_s *target, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;
	int retval;

	/* sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_INVALID_ARGUMENTS;

	/* cortex_m3 handles unaligned memory access */

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

int cortex_m3_write_memory(struct target_s *target, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;
	int retval;

	/* sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_INVALID_ARGUMENTS;

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

	return retval;
}

int cortex_m3_bulk_write_memory(target_t *target, uint32_t address, uint32_t count, uint8_t *buffer)
{
	return cortex_m3_write_memory(target, address, 4, count, buffer);
}

void cortex_m3_build_reg_cache(target_t *target)
{
	armv7m_build_reg_cache(target);
}

int cortex_m3_init_target(struct command_context_s *cmd_ctx, struct target_s *target)
{
	cortex_m3_build_reg_cache(target);
	return ERROR_OK;
}

int cortex_m3_examine(struct target_s *target)
{
	int retval;
	uint32_t cpuid, fpcr, dwtcr, ictr;
	int i;

	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;

	if ((retval = ahbap_debugport_init(swjdp)) != ERROR_OK)
		return retval;

	if (!target_was_examined(target))
	{
		target_set_examined(target);

		/* Read from Device Identification Registers */
		if ((retval = target_read_u32(target, CPUID, &cpuid)) != ERROR_OK)
			return retval;

		if (((cpuid >> 4) & 0xc3f) == 0xc23)
			LOG_DEBUG("CORTEX-M3 processor detected");
		LOG_DEBUG("cpuid: 0x%8.8" PRIx32 "", cpuid);

		target_read_u32(target, NVIC_ICTR, &ictr);
		cortex_m3->intlinesnum = (ictr & 0x1F) + 1;
		cortex_m3->intsetenable = calloc(cortex_m3->intlinesnum, 4);
		for (i = 0; i < cortex_m3->intlinesnum; i++)
		{
			target_read_u32(target, NVIC_ISE0 + 4 * i, cortex_m3->intsetenable + i);
			LOG_DEBUG("interrupt enable[%i] = 0x%8.8" PRIx32 "", i, cortex_m3->intsetenable[i]);
		}

		/* Setup FPB */
		target_read_u32(target, FP_CTRL, &fpcr);
		cortex_m3->auto_bp_type = 1;
		cortex_m3->fp_num_code = ((fpcr >> 8) & 0x70) | ((fpcr >> 4) & 0xF); /* bits [14:12] and [7:4] */
		cortex_m3->fp_num_lit = (fpcr >> 8) & 0xF;
		cortex_m3->fp_code_available = cortex_m3->fp_num_code;
		cortex_m3->fp_comparator_list = calloc(cortex_m3->fp_num_code + cortex_m3->fp_num_lit, sizeof(cortex_m3_fp_comparator_t));
		cortex_m3->fpb_enabled = fpcr & 1;
		for (i = 0; i < cortex_m3->fp_num_code + cortex_m3->fp_num_lit; i++)
		{
			cortex_m3->fp_comparator_list[i].type = (i < cortex_m3->fp_num_code) ? FPCR_CODE : FPCR_LITERAL;
			cortex_m3->fp_comparator_list[i].fpcr_address = FP_COMP0 + 4 * i;
		}
		LOG_DEBUG("FPB fpcr 0x%" PRIx32 ", numcode %i, numlit %i", fpcr, cortex_m3->fp_num_code, cortex_m3->fp_num_lit);

		/* Setup DWT */
		target_read_u32(target, DWT_CTRL, &dwtcr);
		cortex_m3->dwt_num_comp = (dwtcr >> 28) & 0xF;
		cortex_m3->dwt_comp_available = cortex_m3->dwt_num_comp;
		cortex_m3->dwt_comparator_list = calloc(cortex_m3->dwt_num_comp, sizeof(cortex_m3_dwt_comparator_t));
		for (i = 0; i < cortex_m3->dwt_num_comp; i++)
		{
			cortex_m3->dwt_comparator_list[i].dwt_comparator_address = DWT_COMP0 + 0x10 * i;
		}
	}

	return ERROR_OK;
}

int cortex_m3_quit(void)
{

	return ERROR_OK;
}

int cortex_m3_dcc_read(swjdp_common_t *swjdp, uint8_t *value, uint8_t *ctrl)
{
	uint16_t dcrdr;

	mem_ap_read_buf_u16( swjdp, (uint8_t*)&dcrdr, 1, DCB_DCRDR);
	*ctrl = (uint8_t)dcrdr;
	*value = (uint8_t)(dcrdr >> 8);

	LOG_DEBUG("data 0x%x ctrl 0x%x", *value, *ctrl);

	/* write ack back to software dcc register
	 * signify we have read data */
	if (dcrdr & (1 << 0))
	{
		dcrdr = 0;
		mem_ap_write_buf_u16( swjdp, (uint8_t*)&dcrdr, 1, DCB_DCRDR);
	}

	return ERROR_OK;
}

int cortex_m3_target_request_data(target_t *target, uint32_t size, uint8_t *buffer)
{
	armv7m_common_t *armv7m = target->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;
	uint8_t data;
	uint8_t ctrl;
	uint32_t i;

	for (i = 0; i < (size * 4); i++)
	{
		cortex_m3_dcc_read(swjdp, &data, &ctrl);
		buffer[i] = data;
	}

	return ERROR_OK;
}

int cortex_m3_handle_target_request(void *priv)
{
	target_t *target = priv;
	if (!target_was_examined(target))
		return ERROR_OK;
	armv7m_common_t *armv7m = target->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;

	if (!target->dbg_msg_enabled)
		return ERROR_OK;

	if (target->state == TARGET_RUNNING)
	{
		uint8_t data;
		uint8_t ctrl;

		cortex_m3_dcc_read(swjdp, &data, &ctrl);

		/* check if we have data */
		if (ctrl & (1 << 0))
		{
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

int cortex_m3_init_arch_info(target_t *target, cortex_m3_common_t *cortex_m3, jtag_tap_t *tap)
{
	int retval;
	armv7m_common_t *armv7m;
	armv7m = &cortex_m3->armv7m;

	armv7m_init_arch_info(target, armv7m);

	/* prepare JTAG information for the new target */
	cortex_m3->jtag_info.tap = tap;
	cortex_m3->jtag_info.scann_size = 4;

	armv7m->swjdp_info.dp_select_value = -1;
	armv7m->swjdp_info.ap_csw_value = -1;
	armv7m->swjdp_info.ap_tar_value = -1;
	armv7m->swjdp_info.jtag_info = &cortex_m3->jtag_info;
	armv7m->swjdp_info.memaccess_tck = 8;
	armv7m->swjdp_info.tar_autoincr_block = (1<<12);	/* Cortex-M3 has 4096 bytes autoincrement range */

	/* initialize arch-specific breakpoint handling */

	cortex_m3->common_magic = CORTEX_M3_COMMON_MAGIC;
	cortex_m3->arch_info = NULL;

	/* register arch-specific functions */
	armv7m->examine_debug_reason = cortex_m3_examine_debug_reason;

	armv7m->pre_debug_entry = NULL;
	armv7m->post_debug_entry = NULL;

	armv7m->pre_restore_context = NULL;
	armv7m->post_restore_context = NULL;

	armv7m->arch_info = cortex_m3;
	armv7m->load_core_reg_u32 = cortex_m3_load_core_reg_u32;
	armv7m->store_core_reg_u32 = cortex_m3_store_core_reg_u32;

	target_register_timer_callback(cortex_m3_handle_target_request, 1, 1, target);

	if ((retval = arm_jtag_setup_connection(&cortex_m3->jtag_info)) != ERROR_OK)
	{
		return retval;
	}

	return ERROR_OK;
}

int cortex_m3_target_create(struct target_s *target, Jim_Interp *interp)
{
	cortex_m3_common_t *cortex_m3 = calloc(1,sizeof(cortex_m3_common_t));

	cortex_m3_init_arch_info(target, cortex_m3, target->tap);

	return ERROR_OK;
}

int cortex_m3_register_commands(struct command_context_s *cmd_ctx)
{
	int retval;
	command_t *cortex_m3_cmd;

	retval = armv7m_register_commands(cmd_ctx);

	cortex_m3_cmd = register_command(cmd_ctx, NULL, "cortex_m3", NULL, COMMAND_ANY, "cortex_m3 specific commands");
	register_command(cmd_ctx, cortex_m3_cmd, "maskisr", handle_cortex_m3_mask_interrupts_command, COMMAND_EXEC, "mask cortex_m3 interrupts ['on'|'off']");

	return retval;
}

int handle_cortex_m3_mask_interrupts_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;

	if (target->state != TARGET_HALTED)
	{
		command_print(cmd_ctx, "target must be stopped for \"%s\" command", cmd);
		return ERROR_OK;
	}

	if (argc > 0)
	{
		if (!strcmp(args[0], "on"))
		{
			cortex_m3_write_debug_halt_mask(target, C_HALT|C_MASKINTS, 0);
		}
		else if (!strcmp(args[0], "off"))
		{
			cortex_m3_write_debug_halt_mask(target, C_HALT, C_MASKINTS);
		}
		else
		{
			command_print(cmd_ctx, "usage: cortex_m3 maskisr ['on'|'off']");
		}
	}

	command_print(cmd_ctx, "cortex_m3 interrupt mask %s",
			(cortex_m3->dcb_dhcsr & C_MASKINTS) ? "on" : "off");

	return ERROR_OK;
}
