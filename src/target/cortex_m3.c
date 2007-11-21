/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
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

#include "replacements.h"

#include "cortex_m3.h"
#include "armv7m.h"

#include "register.h"
#include "target.h"
#include "log.h"
#include "jtag.h"
#include "arm_jtag.h"

#include <stdlib.h>
#include <string.h>

/* cli handling */
int cortex_m3_register_commands(struct command_context_s *cmd_ctx);

/* forward declarations */
void cortex_m3_unset_all_breakpoints_and_watchpoints(struct target_s *target);
void cortex_m3_enable_breakpoints(struct target_s *target);
void cortex_m3_enable_watchpoints(struct target_s *target);
void cortex_m3_disable_bkpts_and_wpts(struct target_s *target);
int cortex_m3_target_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct target_s *target);
int cortex_m3_init_target(struct command_context_s *cmd_ctx, struct target_s *target);
int cortex_m3_quit();
int cortex_m3_load_core_reg_u32(target_t *target, enum armv7m_regtype type, u32 num, u32 *value);
int cortex_m3_store_core_reg_u32(target_t *target, enum armv7m_regtype type, u32 num, u32 value);
		
target_type_t cortexm3_target =
{
	.name = "cortex_m3",

	.poll = cortex_m3_poll,
	.arch_state = armv7m_arch_state,

	.target_request_data = NULL,

	.halt = cortex_m3_halt,
	.resume = cortex_m3_resume,
	.step = cortex_m3_step,

	.assert_reset = cortex_m3_assert_reset,
	.deassert_reset = cortex_m3_deassert_reset,
	.soft_reset_halt = cortex_m3_soft_reset_halt,
	.prepare_reset_halt = cortex_m3_prepare_reset_halt,
	
	.get_gdb_reg_list = armv7m_get_gdb_reg_list,

	.read_memory = cortex_m3_read_memory,
	.write_memory = cortex_m3_write_memory,
	.bulk_write_memory = cortex_m3_bulk_write_memory,
	.checksum_memory = armv7m_checksum_memory,
	
	.run_algorithm = armv7m_run_algorithm,
	
	.add_breakpoint = cortex_m3_add_breakpoint,
	.remove_breakpoint = cortex_m3_remove_breakpoint,
	.add_watchpoint = cortex_m3_add_watchpoint,
	.remove_watchpoint = cortex_m3_remove_watchpoint,

	.register_commands = cortex_m3_register_commands,
	.target_command = cortex_m3_target_command,
	.init_target = cortex_m3_init_target,
	.quit = cortex_m3_quit
};

int cortex_m3_clear_halt(target_t *target)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &cortex_m3->swjdp_info;
    
    /* Read Debug Fault Status Register */
    ahbap_read_system_atomic_u32(swjdp, NVIC_DFSR, &cortex_m3->nvic_dfsr);
    /* Write Debug Fault Status Register to enable processing to resume ?? Try with and without this !! */
    ahbap_write_system_atomic_u32(swjdp, NVIC_DFSR, cortex_m3->nvic_dfsr);
    DEBUG(" NVIC_DFSR 0x%x", cortex_m3->nvic_dfsr);

    return ERROR_OK;
}

int cortex_m3_single_step_core(target_t *target)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &cortex_m3->swjdp_info;

	if (!(cortex_m3->dcb_dhcsr & C_MASKINTS))
		ahbap_write_system_atomic_u32(swjdp, DCB_DHCSR, DBGKEY | C_MASKINTS | C_HALT | C_DEBUGEN );
	ahbap_write_system_atomic_u32(swjdp, DCB_DHCSR, DBGKEY | C_MASKINTS | C_STEP | C_DEBUGEN );
	cortex_m3->dcb_dhcsr |= C_MASKINTS;
	DEBUG(" ");
	cortex_m3_clear_halt(target);
	
	return ERROR_OK;
}

int cortex_m3_exec_opcode(target_t *target,u32 opcode, int len /* MODE, r0_invalue, &r0_outvalue */ )
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &cortex_m3->swjdp_info;
	u32 savedram;
	int retvalue;
	
	ahbap_read_system_u32(swjdp, 0x20000000, &savedram);
	ahbap_write_system_u32(swjdp, 0x20000000, opcode);
	ahbap_write_coreregister_u32(swjdp, 0x20000000, 15);
	cortex_m3_single_step_core(target);
	armv7m->core_cache->reg_list[15].dirty = 1;
	retvalue = ahbap_write_system_atomic_u32(swjdp, 0x20000000, savedram);		
	
	return retvalue;
}

/* Enable interrupts */
int cortex_m3_cpsie(target_t *target, u32 IF)
{
	return cortex_m3_exec_opcode(target, ARMV7M_T_CPSIE(IF), 2);
}

/* Disable interrupts */
int cortex_m3_cpsid(target_t *target, u32 IF)
{
	return cortex_m3_exec_opcode(target, ARMV7M_T_CPSID(IF), 2);
}

int cortex_m3_endreset_event(target_t *target)
{
	int i;
	u32 dcb_demcr;
	
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &cortex_m3->swjdp_info;
	cortex_m3_fp_comparator_t *fp_list = cortex_m3->fp_comparator_list; 
	cortex_m3_dwt_comparator_t *dwt_list = cortex_m3->dwt_comparator_list;

	ahbap_read_system_atomic_u32(swjdp, DCB_DEMCR, &dcb_demcr);
	DEBUG("DCB_DEMCR = 0x%8.8x",dcb_demcr);
	
	/* Enable debug requests */
	ahbap_read_system_atomic_u32(swjdp, DCB_DHCSR, &cortex_m3->dcb_dhcsr);
	if (!(cortex_m3->dcb_dhcsr & C_DEBUGEN))
		ahbap_write_system_u32(swjdp, DCB_DHCSR, DBGKEY | C_DEBUGEN );
	/* Enable trace and dwt */
	ahbap_write_system_u32(swjdp, DCB_DEMCR, TRCENA | VC_HARDERR | VC_BUSERR );
	/* Monitor bus faults */
	ahbap_write_system_u32(swjdp, NVIC_SHCSR, SHCSR_BUSFAULTENA );

	/* Enable FPB */
	target_write_u32(target, FP_CTRL, 3);

	/* Restore FPB registers */
	for ( i = 0; i < cortex_m3->fp_num_code + cortex_m3->fp_num_lit; i++)
	{
		target_write_u32(target, fp_list[i].fpcr_address, fp_list[i].fpcr_value);
	}
	
	/* Restore DWT registers */
	for ( i = 0; i < cortex_m3->dwt_num_comp; i++)
	{
		target_write_u32(target, dwt_list[i].dwt_comparator_address, dwt_list[i].comp);
		target_write_u32(target, dwt_list[i].dwt_comparator_address | 0x4, dwt_list[i].mask);
		target_write_u32(target, dwt_list[i].dwt_comparator_address | 0x8, dwt_list[i].function);
	}
	swjdp_transaction_endcheck(swjdp);
	
	/* Make sure working_areas are all free */
	target_free_all_working_areas(target);
	
	/* We are in process context */
	armv7m_use_context(target, ARMV7M_PROCESS_CONTEXT);
	armv7m_invalidate_core_regs(target);
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
		/*  INCOPMPLETE */

		if (cortex_m3->nvic_dfsr & 0x2)
		{
			target->debug_reason = DBG_REASON_BREAKPOINT;
			if (cortex_m3->nvic_dfsr & 0x4)
				target->debug_reason = DBG_REASON_WPTANDBKPT;
		}
		else if (cortex_m3->nvic_dfsr & 0x4)
			target->debug_reason = DBG_REASON_WATCHPOINT;
	}

	return ERROR_OK;
}

int cortex_m3_examine_exception_reason(target_t *target)
{
	u32 shcsr, except_sr, cfsr = -1, except_ar = -1;

	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &cortex_m3->swjdp_info;

	ahbap_read_system_u32(swjdp, NVIC_SHCSR, &shcsr);
	switch (armv7m->exception_number)
	{
		case 2:	/* NMI */
			break;
		case 3:	/* Hard Fault */
			ahbap_read_system_atomic_u32(swjdp, NVIC_HFSR, &except_sr);
			if (except_sr & 0x40000000)
			{
				ahbap_read_system_u32(swjdp, NVIC_CFSR, &cfsr);
			}
			break;
		case 4:	/* Memory Management */
			ahbap_read_system_u32(swjdp, NVIC_CFSR, &except_sr);
			ahbap_read_system_u32(swjdp, NVIC_MMFAR, &except_ar);		
			break;
		case 5:	/* Bus Fault */
			ahbap_read_system_u32(swjdp, NVIC_CFSR, &except_sr);
			ahbap_read_system_u32(swjdp, NVIC_BFAR, &except_ar);				
			break;
		case 6:	/* Usage Fault */
			ahbap_read_system_u32(swjdp, NVIC_CFSR, &except_sr);
			break;
		case 11:	/* SVCall */
			break;
		case 12:	/* Debug Monitor */
			ahbap_read_system_u32(swjdp, NVIC_DFSR, &except_sr);
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
    DEBUG("%s SHCSR 0x%x, SR 0x%x, CFSR 0x%x, AR 0x%x", armv7m_exception_string(armv7m->exception_number), \
    	shcsr, except_sr, cfsr, except_ar);
	return ERROR_OK;
}

int cortex_m3_debug_entry(target_t *target)
{
	int i;
	u32 xPSR;
	int retval;

	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &cortex_m3->swjdp_info;

	DEBUG(" ");
	if (armv7m->pre_debug_entry)
		armv7m->pre_debug_entry(target);

	cortex_m3_clear_halt(target);
	ahbap_read_system_atomic_u32(swjdp, DCB_DHCSR, &cortex_m3->dcb_dhcsr);

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
	
	/* For IT instructions xPSR must be reloaded on resume and clear on debug exec*/
	if (xPSR & 0xf00)
	{
		armv7m->core_cache->reg_list[ARMV7M_xPSR].dirty = 1;
		cortex_m3_store_core_reg_u32(target, ARMV7M_REGISTER_CORE_GP, 16, xPSR &~ 0xff);
	}


	/* Now we can load SP core registers */	
	for (i = ARMV7M_PRIMASK; i < ARMV7NUMCOREREGS; i++)
	{
		if (!armv7m->core_cache->reg_list[i].valid)
			armv7m->read_core_reg(target, i);		
	}

	/* Are we in an exception handler */
    armv7m->core_mode = (xPSR & 0x1FF) ? ARMV7M_MODE_HANDLER : ARMV7M_MODE_THREAD;
    armv7m->exception_number = xPSR & 0x1FF;
	if (armv7m->exception_number)
	{
		cortex_m3_examine_exception_reason(target);
	}

	DEBUG("entered debug state at PC 0x%x, target->state: %s ", *(u32*)(armv7m->core_cache->reg_list[15].value), target_state_strings[target->state]);

	if (armv7m->post_debug_entry)
		armv7m->post_debug_entry(target);

	return ERROR_OK;
}

enum target_state cortex_m3_poll(target_t *target)
{
	int retval;
	u32 prev_target_state = target->state;
	
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &cortex_m3->swjdp_info;

	/* Read from Debug Halting Control and Status Register */
	retval = ahbap_read_system_atomic_u32(swjdp, DCB_DHCSR, &cortex_m3->dcb_dhcsr);
	if (retval != ERROR_OK)
	{
		target->state = TARGET_UNKNOWN;
		return TARGET_UNKNOWN;
	}
	
	if (cortex_m3->dcb_dhcsr & S_RESET_ST)
	{
		/* check if still in reset */
		ahbap_read_system_atomic_u32(swjdp, DCB_DHCSR, &cortex_m3->dcb_dhcsr);
		
		if (cortex_m3->dcb_dhcsr & S_RESET_ST)
		{
			target->state = TARGET_RESET;
			return target->state;
		}
	}
	
	if (target->state == TARGET_RESET)
	{
		/* Cannot switch context while running so endreset is called with target->state == TARGET_RESET */
		DEBUG("Exit from reset with dcb_dhcsr 0x%x", cortex_m3->dcb_dhcsr);
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
				return TARGET_UNKNOWN;
			
			target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		}
		if (prev_target_state == TARGET_DEBUG_RUNNING)
		{
			DEBUG(" ");
			if ((retval = cortex_m3_debug_entry(target)) != ERROR_OK)
				return TARGET_UNKNOWN;

			target_call_event_callbacks(target, TARGET_EVENT_DEBUG_HALTED);
		}
	}
		
	/*
	if (cortex_m3->dcb_dhcsr & S_SLEEP)
		target->state = TARGET_SLEEP;
	*/

    /* Read Debug Fault Status Register, added to figure out the lockup when running flashtest.script  */
    ahbap_read_system_atomic_u32(swjdp, NVIC_DFSR, &cortex_m3->nvic_dfsr);
	DEBUG("dcb_dhcsr 0x%x, nvic_dfsr 0x%x, target->state: %s", cortex_m3->dcb_dhcsr, cortex_m3->nvic_dfsr, target_state_strings[target->state]);	
	return target->state;
}

int cortex_m3_halt(target_t *target)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &cortex_m3->swjdp_info;
	
	DEBUG("target->state: %s", target_state_strings[target->state]);
	
	if (target->state == TARGET_HALTED)
	{
		WARNING("target was already halted");
		return ERROR_TARGET_ALREADY_HALTED;
	}
	
	if (target->state == TARGET_UNKNOWN)
	{
		WARNING("target was in unknown state when halt was requested");
	}
	
	if (target->state == TARGET_RESET) 
	{
		if ((jtag_reset_config & RESET_SRST_PULLS_TRST) && jtag_srst)
		{
			ERROR("can't request a halt while in reset if nSRST pulls nTRST");
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
	ahbap_write_system_atomic_u32(swjdp, DCB_DHCSR, DBGKEY | C_DEBUGEN | C_HALT );

	target->debug_reason = DBG_REASON_DBGRQ;
	
	return ERROR_OK;
}

int cortex_m3_soft_reset_halt(struct target_s *target)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &cortex_m3->swjdp_info;
	u32 dcb_dhcsr = 0;
	int retval, timeout = 0;
	
	/* Check that we are using process_context, or change and print warning */
	if (armv7m_get_context(target) != ARMV7M_PROCESS_CONTEXT)
	{
		DEBUG("Changing to process contex registers");
		armv7m_use_context(target, ARMV7M_PROCESS_CONTEXT);
	}

	/* Enter debug state on reset, cf. end_reset_event() */
	ahbap_write_system_u32(swjdp, DCB_DEMCR, TRCENA | VC_HARDERR | VC_BUSERR | VC_CORERESET );
	
	/* Request a reset */ 
	ahbap_write_system_atomic_u32(swjdp, NVIC_AIRCR, AIRCR_VECTKEY | AIRCR_VECTRESET );
	target->state = TARGET_RESET;

	/* registers are now invalid */
	armv7m_invalidate_core_regs(target);

	while (timeout < 100)
	{
		retval = ahbap_read_system_atomic_u32(swjdp, DCB_DHCSR, &dcb_dhcsr);
		if (retval == ERROR_OK)
		{
		    ahbap_read_system_atomic_u32(swjdp, NVIC_DFSR, &cortex_m3->nvic_dfsr);
			if ((dcb_dhcsr & S_HALT) && (cortex_m3->nvic_dfsr & DFSR_VCATCH))
			{
				DEBUG("system reset-halted, dcb_dhcsr 0x%x, nvic_dfsr 0x%x", dcb_dhcsr, cortex_m3->nvic_dfsr);
				cortex_m3_poll(target);
				return ERROR_OK;
			}
			else
				DEBUG("waiting for system reset-halt, dcb_dhcsr 0x%x, %i ms", dcb_dhcsr, timeout);
		}
		timeout++;
		usleep(1000);
	}
		
	return ERROR_OK;
}

int cortex_m3_prepare_reset_halt(struct target_s *target)
{
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &cortex_m3->swjdp_info;
	u32 dcb_demcr, dcb_dhcsr;
	
	/* Enable debug requests */
	ahbap_read_system_atomic_u32(swjdp, DCB_DHCSR, &cortex_m3->dcb_dhcsr);
	if (!(cortex_m3->dcb_dhcsr & C_DEBUGEN))
		ahbap_write_system_u32(swjdp, DCB_DHCSR, DBGKEY | C_DEBUGEN );
	
	/* Enter debug state on reset, cf. end_reset_event() */
	ahbap_write_system_atomic_u32(swjdp, DCB_DEMCR, TRCENA | VC_HARDERR | VC_BUSERR | VC_CORERESET );
	
	ahbap_read_system_atomic_u32(swjdp, DCB_DHCSR, &dcb_dhcsr);
	ahbap_read_system_atomic_u32(swjdp, DCB_DEMCR, &dcb_demcr);
	DEBUG("dcb_dhcsr 0x%x, dcb_demcr 0x%x, ", dcb_dhcsr, dcb_demcr);
	
	return ERROR_OK;
}

int cortex_m3_resume(struct target_s *target, int current, u32 address, int handle_breakpoints, int debug_execution)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &cortex_m3->swjdp_info;
	breakpoint_t *breakpoint = NULL;
	u32 dcb_dhcsr, resume_pc;
	
	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (!debug_execution)
	{
		/* Check that we are using process_context, or change and print warning */
		if (armv7m_get_context(target) != ARMV7M_PROCESS_CONTEXT)
		{
			WARNING("Incorrect context in resume");
			armv7m_use_context(target, ARMV7M_PROCESS_CONTEXT);
		}
		
		target_free_all_working_areas(target);
		cortex_m3_enable_breakpoints(target);
		cortex_m3_enable_watchpoints(target);

		/* TODOLATER Interrupt handling/disable for debug execution, cache ... ... */ 
	}
	
	dcb_dhcsr = DBGKEY | C_DEBUGEN;
	if (debug_execution)
	{
		/* Check that we are using debug_context, or change and print warning */
		if (armv7m_get_context(target) != ARMV7M_DEBUG_CONTEXT)
		{
			WARNING("Incorrect context in debug_exec resume");
			armv7m_use_context(target, ARMV7M_DEBUG_CONTEXT);
		}
		/* Disable interrupts */
		/* 
		   We disable interrupts in the PRIMASK register instead of masking with C_MASKINTS,
		   This is probably the same inssue as Cortex-M3 Errata	377493: 
		   C_MASKINTS in parallel with disabled interrupts can cause local faults to not be taken.
		*/
		buf_set_u32(armv7m->core_cache->reg_list[ARMV7M_PRIMASK].value, 0, 32, 1);
		/* Make sure we are in Thumb mode */
		buf_set_u32(armv7m->core_cache->reg_list[ARMV7M_xPSR].value, 0, 32, 
			buf_get_u32(armv7m->core_cache->reg_list[ARMV7M_xPSR].value, 0, 32) | (1<<24));
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
				DEBUG("unset breakpoint at 0x%8.8x", breakpoint->address);
				cortex_m3_unset_breakpoint(target, breakpoint);
				cortex_m3_single_step_core(target);
				cortex_m3_set_breakpoint(target, breakpoint);
		}
	}

	/* Set/Clear C_MASKINTS in a separate operation */
	if ((cortex_m3->dcb_dhcsr & C_MASKINTS) != (dcb_dhcsr & C_MASKINTS))
		ahbap_write_system_atomic_u32(swjdp, DCB_DHCSR, dcb_dhcsr | C_HALT );
	
	/* Restart core */
	ahbap_write_system_atomic_u32(swjdp, DCB_DHCSR, dcb_dhcsr );
	target->debug_reason = DBG_REASON_NOTHALTED;

	/* registers are now invalid */
	armv7m_invalidate_core_regs(target);
	if (!debug_execution)
	{
		target->state = TARGET_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_RESUMED);
		DEBUG("target resumed at 0x%x",resume_pc);
	}
	else
	{
		target->state = TARGET_DEBUG_RUNNING;
		target_call_event_callbacks(target, TARGET_EVENT_DEBUG_RESUMED);
		DEBUG("target debug resumed at 0x%x",resume_pc);
	}
	
	return ERROR_OK;
}

//int irqstepcount=0;
int cortex_m3_step(struct target_s *target, int current, u32 address, int handle_breakpoints)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &cortex_m3->swjdp_info;
	breakpoint_t *breakpoint = NULL;

	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	/* Check that we are using process_context, or change and print warning */
	if (armv7m_get_context(target) != ARMV7M_PROCESS_CONTEXT)
	{
		WARNING("Incorrect context in step, must be process");
		armv7m_use_context(target, ARMV7M_PROCESS_CONTEXT);
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
    
	if (cortex_m3->dcb_dhcsr & C_MASKINTS)
		ahbap_write_system_atomic_u32(swjdp, DCB_DHCSR, DBGKEY | C_HALT | C_DEBUGEN );
	ahbap_write_system_atomic_u32(swjdp, DCB_DHCSR, DBGKEY| C_STEP | C_DEBUGEN);
	ahbap_read_system_atomic_u32(swjdp, DCB_DHCSR, &cortex_m3->dcb_dhcsr);

	/* If we run in process context then registers are now invalid */
	if (armv7m_get_context(target) == ARMV7M_PROCESS_CONTEXT)
		armv7m_invalidate_core_regs(target);
	
	if (breakpoint)
		cortex_m3_set_breakpoint(target, breakpoint);

	DEBUG("target stepped dcb_dhcsr = 0x%x nvic_icsr = 0x%x", cortex_m3->dcb_dhcsr, cortex_m3->nvic_icsr);

	cortex_m3_debug_entry(target);
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);

	DEBUG("target stepped dcb_dhcsr = 0x%x nvic_icsr = 0x%x", cortex_m3->dcb_dhcsr, cortex_m3->nvic_icsr);
	return ERROR_OK;
}

int cortex_m3_assert_reset(target_t *target)
{
	int retval;
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &cortex_m3->swjdp_info;
	
	DEBUG("target->state: %s", target_state_strings[target->state]);
	
	if (target->reset_mode == RESET_RUN)
	{
		/* Set/Clear C_MASKINTS in a separate operation */
		if (cortex_m3->dcb_dhcsr & C_MASKINTS)
			ahbap_write_system_atomic_u32(swjdp, DCB_DHCSR, DBGKEY | C_DEBUGEN | C_HALT );
		
		cortex_m3_clear_halt(target);
							
		/* Enter debug state on reset, cf. end_reset_event() */	
		ahbap_write_system_u32(swjdp, DCB_DHCSR, DBGKEY | C_DEBUGEN );
		ahbap_write_system_u32(swjdp, DCB_DEMCR, TRCENA | VC_HARDERR | VC_BUSERR);
	}
	
	if (target->state == TARGET_HALTED || target->state == TARGET_UNKNOWN)
	{
		/* assert SRST and TRST */
		/* system would get ouf sync if we didn't reset test-logic, too */
		if ((retval = jtag_add_reset(1, 1)) != ERROR_OK)
		{
			if (retval == ERROR_JTAG_RESET_CANT_SRST)
			{
				WARNING("can't assert srst");
				return retval;
			}
			else
			{
				ERROR("unknown error");
				exit(-1);
			}
		}
		jtag_add_sleep(5000);
		if ((retval = jtag_add_reset(0, 1)) != ERROR_OK)
		{
			if (retval == ERROR_JTAG_RESET_WOULD_ASSERT_TRST)
			{
				WARNING("srst resets test logic, too");
				retval = jtag_add_reset(1, 1);
			}
		}
	}
	else
	{
		if ((retval = jtag_add_reset(0, 1)) != ERROR_OK)
		{
			if (retval == ERROR_JTAG_RESET_WOULD_ASSERT_TRST)
			{
				WARNING("srst resets test logic, too");
				retval = jtag_add_reset(1, 1);
			}
			
			if (retval == ERROR_JTAG_RESET_CANT_SRST)
			{
				WARNING("can't assert srsrt");
				return retval;
			}
			else if (retval != ERROR_OK)
			{
				ERROR("unknown error");
				exit(-1);
			}
		}
	}
	
	target->state = TARGET_RESET;
	jtag_add_sleep(50000);
	
	armv7m_use_context(target, ARMV7M_PROCESS_CONTEXT);
	armv7m_invalidate_core_regs(target);

	return ERROR_OK;
}

int cortex_m3_deassert_reset(target_t *target)
{		
	DEBUG("target->state: %s", target_state_strings[target->state]);
	
	/* deassert reset lines */
	jtag_add_reset(0, 0);
		
	return ERROR_OK;
}

void cortex_m3_unset_all_breakpoints_and_watchpoints(struct target_s *target)
{

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
	int fp_num=0;
	u32 hilo;
	
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	
	cortex_m3_fp_comparator_t * comparator_list = cortex_m3->fp_comparator_list;

	if (breakpoint->set)
	{
		WARNING("breakpoint already set");
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
			DEBUG("ERROR Can not find free FP Comparator");
			WARNING("ERROR Can not find free FP Comparator");
			exit(-1);
		}
		breakpoint->set = fp_num + 1;
		hilo = (breakpoint->address & 0x2) ? FPCR_REPLACE_BKPT_HIGH : FPCR_REPLACE_BKPT_LOW;
		comparator_list[fp_num].used = 1;
		comparator_list[fp_num].fpcr_value = (breakpoint->address & 0x1FFFFFFC) | hilo | 1;
		target_write_u32(target, comparator_list[fp_num].fpcr_address, comparator_list[fp_num].fpcr_value);
		DEBUG("fpc_num %i fpcr_value 0x%x", fp_num, comparator_list[fp_num].fpcr_value);
	}
	else if (breakpoint->type == BKPT_SOFT)
	{
		u8 code[4];
		buf_set_u32(code, 0, 32, ARMV7M_T_BKPT(0x11));
		target->type->read_memory(target, breakpoint->address & 0xFFFFFFFE, breakpoint->length, 1, breakpoint->orig_instr);
		target->type->write_memory(target, breakpoint->address & 0xFFFFFFFE, breakpoint->length, 1, code);
		breakpoint->set = 0x11; /* Any nice value but 0 */
	}

	return ERROR_OK;
}

int cortex_m3_unset_breakpoint(struct target_s *target, breakpoint_t *breakpoint)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	cortex_m3_fp_comparator_t * comparator_list = cortex_m3->fp_comparator_list;

	if (!breakpoint->set)
	{
		WARNING("breakpoint not set");
		return ERROR_OK;
	}
	
	if (breakpoint->type == BKPT_HARD)
	{
		int fp_num = breakpoint->set - 1;
		if ((fp_num < 0) || (fp_num >= cortex_m3->fp_num_code))
		{
			DEBUG("Invalid FP Comparator number in breakpoint");
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
			target->type->write_memory(target, breakpoint->address & 0xFFFFFFFE, 4, 1, breakpoint->orig_instr);
		}
		else
		{
			target->type->write_memory(target, breakpoint->address & 0xFFFFFFFE, 2, 1, breakpoint->orig_instr);
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
	}

	if ((breakpoint->type == BKPT_HARD) && (breakpoint->address >= 0x20000000))
	{
		INFO("flash patch comparator requested outside code memory region");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if ((breakpoint->type == BKPT_SOFT) && (breakpoint->address < 0x20000000))
	{
		INFO("soft breakpoint requested in code (flash) memory region");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if ((breakpoint->type == BKPT_HARD) && (cortex_m3->fp_code_available < 1))
	{
		INFO("no flash patch comparator unit available for hardware breakpoint");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if ((breakpoint->length != 2))
	{
		INFO("only breakpoints of two bytes length supported");
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
		WARNING("target not halted");
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
	u32 mask, temp;
	
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	cortex_m3_dwt_comparator_t * comparator_list = cortex_m3->dwt_comparator_list;

	if (watchpoint->set)
	{
		WARNING("watchpoint already set");
		return ERROR_OK;
	}

	if (watchpoint->mask == 0xffffffffu)
	{
		while(comparator_list[dwt_num].used && (dwt_num < cortex_m3->dwt_num_comp))
			dwt_num++;
		if (dwt_num >= cortex_m3->dwt_num_comp)
		{
			DEBUG("ERROR Can not find free DWT Comparator");
			WARNING("ERROR Can not find free DWT Comparator");
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
		DEBUG("dwt_num %i 0x%x 0x%x 0x%x", dwt_num, comparator_list[dwt_num].comp, comparator_list[dwt_num].mask, comparator_list[dwt_num].function);
	}
	else
	{
		WARNING("Cannot watch data values");  /* Move this test to add_watchpoint */
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
		WARNING("watchpoint not set");
		return ERROR_OK;
	}

	dwt_num = watchpoint->set - 1;

	if ((dwt_num < 0) || (dwt_num >= cortex_m3->dwt_num_comp))
	{
		DEBUG("Invalid DWT Comparator number in watchpoint");
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
		WARNING("target not halted");
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
		WARNING("target not halted");
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

int cortex_m3_load_core_reg_u32(struct target_s *target, enum armv7m_regtype type, u32 num, u32 * value)
{
	int retval;
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &cortex_m3->swjdp_info;
		
	if ((type == ARMV7M_REGISTER_CORE_GP) && (num <= ARMV7M_PSP))
	{
		/* read a normal core register */
		retval = ahbap_read_coreregister_u32(swjdp, value, num);
		
		if (retval != ERROR_OK)
		{
			ERROR("JTAG failure %i",retval);
			return ERROR_JTAG_DEVICE_ERROR;
		}
		//DEBUG("load from core reg %i  value 0x%x",num,*value);
	}
	else if (type == ARMV7M_REGISTER_CORE_SP) /* Special purpose core register */
	{
		/* read other registers */
		/* cortex_m3_MRS(struct target_s *target, int num, u32* value) */
		u32 savedram;
		u32 SYSm;
		u32 instr;
		SYSm = num & 0x1F;
		ahbap_read_system_u32(swjdp, 0x20000000, &savedram);
		instr = ARMV7M_T_MRS(0, SYSm);
		ahbap_write_system_u32(swjdp, 0x20000000, ARMV7M_T_MRS(0, SYSm));
		ahbap_write_coreregister_u32(swjdp, 0x20000000, 15);
		cortex_m3_single_step_core(target);
		ahbap_read_coreregister_u32(swjdp, value, 0);
		armv7m->core_cache->reg_list[0].dirty = 1;
		armv7m->core_cache->reg_list[15].dirty = 1;
		ahbap_write_system_u32(swjdp, 0x20000000, savedram);
		swjdp_transaction_endcheck(swjdp);
		DEBUG("load from special reg %i value 0x%x", SYSm, *value);
	}
	else return ERROR_INVALID_ARGUMENTS;
	
	return ERROR_OK;
}

int cortex_m3_store_core_reg_u32(struct target_s *target, enum armv7m_regtype type, u32 num, u32 value)
{
	int retval;
	
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &cortex_m3->swjdp_info;

	if ((type == ARMV7M_REGISTER_CORE_GP) && (num <= ARMV7M_PSP))
	{
		retval = ahbap_write_coreregister_u32(swjdp, value, num);
		if (retval != ERROR_OK)
		{
			ERROR("JTAG failure %i", retval);
			armv7m->core_cache->reg_list[num].dirty = 1;
			return ERROR_JTAG_DEVICE_ERROR;
		}
		DEBUG("write core reg %i value 0x%x", num, value);
	}
	else if (type == ARMV7M_REGISTER_CORE_SP) /* Special purpose core register */
	{
		/* write other registers */
		u32 savedram , tempr0;
		u32 SYSm;
		u32 instr;
		SYSm = num & 0x1F;
		ahbap_read_system_u32(swjdp, 0x20000000, &savedram);
		instr = ARMV7M_T_MSR(SYSm, 0);
		ahbap_write_system_u32(swjdp, 0x20000000, ARMV7M_T_MSR(SYSm, 0));
		ahbap_read_coreregister_u32(swjdp, &tempr0, 0);
		ahbap_write_coreregister_u32(swjdp, value, 0);
		ahbap_write_coreregister_u32(swjdp, 0x20000000, 15);
		cortex_m3_single_step_core(target);
		ahbap_write_coreregister_u32(swjdp, tempr0, 0);
		armv7m->core_cache->reg_list[15].dirty = 1;
		ahbap_write_system_u32(swjdp, 0x20000000, savedram);
		swjdp_transaction_endcheck(swjdp);
		DEBUG("write special reg %i value 0x%x ", SYSm, value);
	}
	else return ERROR_INVALID_ARGUMENTS;
	
	return ERROR_OK;	
}

int cortex_m3_read_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &cortex_m3->swjdp_info;
	
	/* sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_INVALID_ARGUMENTS;

	if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u)))
		return ERROR_TARGET_UNALIGNED_ACCESS;
	
	/* Is not optimal, autoincrement of tar should be used (  ahbap_block_read and CSW_ADDRINC_SINGLE ) */
	switch (size)
	{
		case 4:
			/* TODOLATER Check error return value ! */
			{
				ahbap_read_buf(swjdp, buffer, 4 * count, address);
			}
			break;
		case 2:
			{
				ahbap_read_buf_u16(swjdp, buffer, 2 * count, address);
			}	
			break;
		case 1:
			{
				ahbap_read_buf(swjdp, buffer, count, address);
			}	
			break;
		default:
			ERROR("BUG: we shouldn't get here");
			exit(-1);
	}
	
	return ERROR_OK;
}

int cortex_m3_write_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &cortex_m3->swjdp_info;
	
	/* sanitize arguments */
	if (((size != 4) && (size != 2) && (size != 1)) || (count == 0) || !(buffer))
		return ERROR_INVALID_ARGUMENTS;

	if (((size == 4) && (address & 0x3u)) || ((size == 2) && (address & 0x1u)))
		return ERROR_TARGET_UNALIGNED_ACCESS;
		
	switch (size)
	{
		case 4:
			/* TODOLATER Check error return value ! */
			{
				ahbap_write_buf(swjdp, buffer, 4 * count, address);
			}
			break;
		case 2:
			{
				ahbap_write_buf_u16(swjdp, buffer, 2 * count, address);
			}	
			break;
		case 1:
			{
				ahbap_write_buf(swjdp, buffer, count, address);
			}	
			break;
		default:
			ERROR("BUG: we shouldn't get here");
			exit(-1);
	}
	
	return ERROR_OK;
}

int cortex_m3_bulk_write_memory(target_t *target, u32 address, u32 count, u8 *buffer)
{
	cortex_m3_write_memory(target, address, 4, count, buffer);
	
	return ERROR_OK;
}

void cortex_m3_build_reg_cache(target_t *target)
{
	armv7m_build_reg_cache(target);
}

int cortex_m3_init_target(struct command_context_s *cmd_ctx, struct target_s *target)
{
	u32 cpuid, fpcr, dwtcr, ictr;
	int i;
	
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	cortex_m3_common_t *cortex_m3 = armv7m->arch_info;
	swjdp_common_t *swjdp = &cortex_m3->swjdp_info;

	cortex_m3_build_reg_cache(target);
	ahbap_debugport_init(swjdp);

	/* Read from Device Identification Registers */
	target_read_u32(target, CPUID, &cpuid);
	if (((cpuid >> 4) & 0xc3f) == 0xc23)
		DEBUG("CORTEX-M3 processor detected");
	DEBUG("cpuid: 0x%8.8x", cpuid);
	
	target_read_u32(target, NVIC_ICTR, &ictr);
	cortex_m3->intlinesnum = (ictr & 0x1F) + 1;
	cortex_m3->intsetenable = calloc(cortex_m3->intlinesnum, 4);
	for (i = 0; i < cortex_m3->intlinesnum; i++)
	{
		target_read_u32(target, NVIC_ISE0 + 4 * i, cortex_m3->intsetenable + i);
		DEBUG("interrupt enable[%i] = 0x%8.8x", i, cortex_m3->intsetenable[i]);
	}
	
	/* Setup FPB */
	target_read_u32(target, FP_CTRL, &fpcr);
	cortex_m3->auto_bp_type = 1;
	cortex_m3->fp_num_code = (fpcr >> 4) & 0xF;
	cortex_m3->fp_num_lit = (fpcr >> 8) & 0xF;
	cortex_m3->fp_code_available = cortex_m3->fp_num_code;
	cortex_m3->fp_comparator_list = calloc(cortex_m3->fp_num_code + cortex_m3->fp_num_lit, sizeof(cortex_m3_fp_comparator_t));
	for (i = 0; i < cortex_m3->fp_num_code + cortex_m3->fp_num_lit; i++)
	{
		cortex_m3->fp_comparator_list[i].type = (i < cortex_m3->fp_num_code) ? FPCR_CODE : FPCR_LITERAL;
		cortex_m3->fp_comparator_list[i].fpcr_address = FP_COMP0 + 4 * i;
	}
	DEBUG("FPB fpcr 0x%x, numcode %i, numlit %i", fpcr, cortex_m3->fp_num_code, cortex_m3->fp_num_lit);
		
	/* Setup DWT */
	target_read_u32(target, DWT_CTRL, &dwtcr);
	cortex_m3->dwt_num_comp = (dwtcr >> 28) & 0xF;
	cortex_m3->dwt_comp_available = cortex_m3->dwt_num_comp;
	cortex_m3->dwt_comparator_list=calloc(cortex_m3->dwt_num_comp, sizeof(cortex_m3_dwt_comparator_t));
	for (i = 0; i < cortex_m3->dwt_num_comp; i++)
	{
		cortex_m3->dwt_comparator_list[i].dwt_comparator_address = DWT_COMP0 + 0x10 * i;
	}
	
	return ERROR_OK;
}

int cortex_m3_quit()
{
	
	return ERROR_OK;
}

int cortex_m3_init_arch_info(target_t *target, cortex_m3_common_t *cortex_m3, int chain_pos, char *variant)
{
	armv7m_common_t *armv7m;
	armv7m = &cortex_m3->armv7m;

	/* prepare JTAG information for the new target */
	cortex_m3->jtag_info.chain_pos = chain_pos;
	cortex_m3->jtag_info.scann_size = 4;
	
	cortex_m3->swjdp_info.dp_select_value = -1;
	cortex_m3->swjdp_info.ap_csw_value = -1;
	cortex_m3->swjdp_info.ap_tar_value = -1;
	cortex_m3->swjdp_info.jtag_info = &cortex_m3->jtag_info;

	/* initialize arch-specific breakpoint handling */
	
	cortex_m3->common_magic = CORTEX_M3_COMMON_MAGIC;
	cortex_m3->arch_info = NULL;

	/* register arch-specific functions */
	armv7m->examine_debug_reason = cortex_m3_examine_debug_reason;

	armv7m->pre_debug_entry = NULL;
	armv7m->post_debug_entry = NULL;
	
	armv7m->pre_restore_context = NULL;
	armv7m->post_restore_context = NULL;
	
	armv7m_init_arch_info(target, armv7m);	
	armv7m->arch_info = cortex_m3;
	armv7m->load_core_reg_u32 = cortex_m3_load_core_reg_u32;
	armv7m->store_core_reg_u32 = cortex_m3_store_core_reg_u32;
//	armv7m->full_context = cortex_m3_full_context;
	
	return ERROR_OK;
}

/* target cortex_m3 <endianess> <startup_mode> <chain_pos> <variant>*/
int cortex_m3_target_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct target_s *target)
{
	int chain_pos;
	char *variant = NULL;
	cortex_m3_common_t *cortex_m3 = malloc(sizeof(cortex_m3_common_t));

	if (argc < 4)
	{
		ERROR("'target cortex_m3' requires at least one additional argument");
		exit(-1);
	}
	
	chain_pos = strtoul(args[3], NULL, 0);
	
	if (argc >= 5)
		variant = args[4];
	
	cortex_m3_init_arch_info(target, cortex_m3, chain_pos, variant);
	cortex_m3_register_commands(cmd_ctx);
	
	return ERROR_OK;
}

int cortex_m3_register_commands(struct command_context_s *cmd_ctx)
{
	int retval;
	
	retval = armv7m_register_commands(cmd_ctx);
	
	return retval;
}
