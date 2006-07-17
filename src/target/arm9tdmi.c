/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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

#include "arm9tdmi.h"

#include "arm7_9_common.h"
#include "register.h"
#include "target.h"
#include "armv4_5.h"
#include "embeddedice.h"
#include "log.h"
#include "jtag.h"
#include "arm_jtag.h"

#include <stdlib.h>
#include <string.h>

#if 0
#define _DEBUG_INSTRUCTION_EXECUTION_
#endif

/* cli handling */
int arm9tdmi_register_commands(struct command_context_s *cmd_ctx);

/* forward declarations */
int arm9tdmi_target_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct target_s *target);
int arm9tdmi_init_target(struct command_context_s *cmd_ctx, struct target_s *target);
int arm9tdmi_quit();

/* target function declarations */
enum target_state arm9tdmi_poll(struct target_s *target);
int arm9tdmi_halt(target_t *target);
int arm9tdmi_read_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);
		
target_type_t arm9tdmi_target =
{
	.name = "arm9tdmi",

	.poll = arm7_9_poll,
	.arch_state = armv4_5_arch_state,

	.halt = arm7_9_halt,
	.resume = arm7_9_resume,
	.step = arm7_9_step,

	.assert_reset = arm7_9_assert_reset,
	.deassert_reset = arm7_9_deassert_reset,
	.soft_reset_halt = arm7_9_soft_reset_halt,

	.get_gdb_reg_list = armv4_5_get_gdb_reg_list,

	.read_memory = arm7_9_read_memory,
	.write_memory = arm7_9_write_memory,
	.bulk_write_memory = arm7_9_bulk_write_memory,

	.add_breakpoint = arm7_9_add_breakpoint,
	.remove_breakpoint = arm7_9_remove_breakpoint,
	.add_watchpoint = arm7_9_add_watchpoint,
	.remove_watchpoint = arm7_9_remove_watchpoint,

	.register_commands = arm9tdmi_register_commands,
	.target_command = arm9tdmi_target_command,
	.init_target = arm9tdmi_init_target,
	.quit = arm9tdmi_quit
};

int arm9tdmi_examine_debug_reason(target_t *target)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	
	/* only check the debug reason if we don't know it already */
	if ((target->debug_reason != DBG_REASON_DBGRQ)
			&& (target->debug_reason != DBG_REASON_SINGLESTEP))
	{
		scan_field_t fields[3];
		u8 databus[4];
		u8 instructionbus[4];
		u8 debug_reason;

		jtag_add_end_state(TAP_PD);

		fields[0].device = arm7_9->jtag_info.chain_pos;
		fields[0].num_bits = 32;
		fields[0].out_value = NULL;
		fields[0].out_mask = NULL;
		fields[0].in_value = databus;
		fields[0].in_check_value = NULL;
		fields[0].in_check_mask = NULL;
		fields[0].in_handler = NULL;
		fields[0].in_handler_priv = NULL;
		
		fields[1].device = arm7_9->jtag_info.chain_pos;
		fields[1].num_bits = 3;
		fields[1].out_value = NULL;
		fields[1].out_mask = NULL;
		fields[1].in_value = &debug_reason;
		fields[1].in_check_value = NULL;
		fields[1].in_check_mask = NULL;
		fields[1].in_handler = NULL;
		fields[1].in_handler_priv = NULL;
		
		fields[2].device = arm7_9->jtag_info.chain_pos;
		fields[2].num_bits = 32;
		fields[2].out_value = NULL;
		fields[2].out_mask = NULL;
		fields[2].in_value = instructionbus;
		fields[2].in_check_value = NULL;
		fields[2].in_check_mask = NULL;
		fields[2].in_handler = NULL;
		fields[2].in_handler_priv = NULL;
		
		arm_jtag_scann(&arm7_9->jtag_info, 0x1);
		arm_jtag_set_instr(&arm7_9->jtag_info, arm7_9->jtag_info.intest_instr);

		jtag_add_dr_scan(3, fields, TAP_PD);
		jtag_execute_queue();
		
		fields[0].in_value = NULL;
		fields[0].out_value = databus;
		fields[1].in_value = NULL;
		fields[1].out_value = &debug_reason;
		fields[2].in_value = NULL;
		fields[2].out_value = instructionbus;
		
		jtag_add_dr_scan(3, fields, TAP_PD);

		if (debug_reason & 0x4)
			if (debug_reason & 0x2)
				target->debug_reason = DBG_REASON_WPTANDBKPT;
		else
			target->debug_reason = DBG_REASON_WATCHPOINT;
		else
			target->debug_reason = DBG_REASON_BREAKPOINT;
	}

	return ERROR_OK;
}

/* put an instruction in the ARM9TDMI pipeline or write the data bus, and optionally read data */
int arm9tdmi_clock_out(arm_jtag_t *jtag_info, u32 instr, u32 out, u32 *in, int sysspeed)
{
	scan_field_t fields[3];
	u8 out_buf[4];
	u8 instr_buf[4];
	u8 sysspeed_buf = 0x0;
	
	/* prepare buffer */
	buf_set_u32(out_buf, 0, 32, out);
	
	instr = flip_u32(instr, 32);
	buf_set_u32(instr_buf, 0, 32, instr);
	
	if (sysspeed)
		buf_set_u32(&sysspeed_buf, 2, 1, 1);
	
	jtag_add_end_state(TAP_PD);
	arm_jtag_scann(jtag_info, 0x1);
	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr);
		
	fields[0].device = jtag_info->chain_pos;
	fields[0].num_bits = 32;
	fields[0].out_value = out_buf;
	fields[0].out_mask = NULL;
	if (in)
	{
		fields[0].in_value = (u8*)in;
	} else
	{
		fields[0].in_value = NULL;
	}
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;
	
	fields[1].device = jtag_info->chain_pos;
	fields[1].num_bits = 3;
	fields[1].out_value = &sysspeed_buf;
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;
		
	fields[2].device = jtag_info->chain_pos;
	fields[2].num_bits = 32;
	fields[2].out_value = instr_buf;
	fields[2].out_mask = NULL;
	fields[2].in_value = NULL;
	fields[2].in_check_value = NULL;
	fields[2].in_check_mask = NULL;
	fields[2].in_handler = NULL;
	fields[2].in_handler_priv = NULL;

	jtag_add_dr_scan(3, fields, -1);

	jtag_add_runtest(0, -1);
	
#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	{
		char* in_string;
		jtag_execute_queue();
		
		if (in)
		{
			in_string = buf_to_char((u8*)in, 32);
			DEBUG("instr: 0x%8.8x, out: 0x%8.8x, in: %s", flip_u32(instr, 32), out, in_string);
			free(in_string);
		}
		else
			DEBUG("instr: 0x%8.8x, out: 0x%8.8x", flip_u32(instr, 32), out);
	}
#endif

	return ERROR_OK;
}

/* just read data (instruction and data-out = don't care) */
int arm9tdmi_clock_data_in(arm_jtag_t *jtag_info, u32 *in)
{
	scan_field_t fields[3];

	jtag_add_end_state(TAP_PD);
	arm_jtag_scann(jtag_info, 0x1);
	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr);
		
	fields[0].device = jtag_info->chain_pos;
	fields[0].num_bits = 32;
	fields[0].out_value = NULL;
	fields[0].out_mask = NULL;
	fields[0].in_value = (u8*)in;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	
	fields[1].device = jtag_info->chain_pos;
	fields[1].num_bits = 3;
	fields[1].out_value = NULL;
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;

	fields[2].device = jtag_info->chain_pos;
	fields[2].num_bits = 32;
	fields[2].out_value = NULL;
	fields[2].out_mask = NULL;
	fields[2].in_value = NULL;
	fields[2].in_check_value = NULL;
	fields[2].in_check_mask = NULL;
	fields[2].in_handler = NULL;
	fields[2].in_handler_priv = NULL;
	
	jtag_add_dr_scan(3, fields, -1);

	jtag_add_runtest(0, -1);
	
#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	{
		char* in_string;
		jtag_execute_queue();
			
		if (in)
		{
			in_string = buf_to_char((u8*)in, 32);
			DEBUG("in: %s", in_string);
			free(in_string);
		}
	}
#endif

	return ERROR_OK;
}

void arm9tdmi_change_to_arm(target_t *target, u32 *r0, u32 *pc)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	
	/* save r0 before using it and put system in ARM state 
	 * to allow common handling of ARM and THUMB debugging */
	
	/* fetch STR r0, [r0] */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_STR(0, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	/* STR r0, [r0] in Memory */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, r0, 0);

	/* MOV r0, r15 fetched, STR in Decode */	
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_MOV(0, 15), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_STR(0, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	/* nothing fetched, STR r0, [r0] in Memory */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, pc, 0);

	/* fetch MOV */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_MOV_IM(0, 0x0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);

	/* fetch BX */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_BX(0), 0, NULL, 0);
	/* NOP fetched, BX in Decode, MOV in Execute */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	/* NOP fetched, BX in Execute (1) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	
	jtag_execute_queue();
	
	/* fix program counter:
	 * MOV r0, r15 was the 5th instruction (+8)
	 * reading PC in Thumb state gives address of instruction + 4
	 */
	*pc -= 0xc;
}

void arm9tdmi_read_core_regs(target_t *target, u32 mask, u32* core_regs[16])
{
	int i;
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
		
	/* STMIA r0-15, [r0] at debug speed
	 * register values will start to appear on 4th DCLK
	 */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_STMIA(0, mask & 0xffff, 0, 0), 0, NULL, 0);

	/* fetch NOP, STM in DECODE stage */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* fetch NOP, STM in EXECUTE stage (1st cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	for (i = 0; i <= 15; i++)
	{
		if (mask & (1 << i))
			/* nothing fetched, STM in MEMORY (i'th cycle) */
			arm9tdmi_clock_data_in(jtag_info, core_regs[i]);
	}

}

void arm9tdmi_read_xpsr(target_t *target, u32 *xpsr, int spsr)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
		
	/* MRS r0, cpsr */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_MRS(0, spsr & 1), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	/* STR r0, [r15] */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_STR(0, 15), 0, NULL, 0);
	/* fetch NOP, STR in DECODE stage */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* fetch NOP, STR in EXECUTE stage (1st cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* nothing fetched, STR in MEMORY */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, xpsr, 0);

}

void arm9tdmi_write_xpsr(target_t *target, u32 xpsr, int spsr)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
		
	DEBUG("xpsr: %8.8x, spsr: %i", xpsr, spsr);

	/* MSR1 fetched */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM(xpsr & 0xff, 0, 1, spsr), 0, NULL, 0);
	/* MSR2 fetched, MSR1 in DECODE */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM((xpsr & 0xff00) >> 8, 0xc, 2, spsr), 0, NULL, 0);
	/* MSR3 fetched, MSR1 in EXECUTE (1), MSR2 in DECODE */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM((xpsr & 0xff0000) >> 16, 0x8, 4, spsr), 0, NULL, 0);
	/* nothing fetched, MSR1 in EXECUTE (2) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* nothing fetched, MSR1 in EXECUTE (3) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* MSR4 fetched, MSR2 in EXECUTE (1), MSR3 in DECODE */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM((xpsr & 0xff000000) >> 24, 0x4, 8, spsr), 0, NULL, 0);
	/* nothing fetched, MSR2 in EXECUTE (2) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* nothing fetched, MSR2 in EXECUTE (3) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* NOP fetched, MSR3 in EXECUTE (1), MSR4 in DECODE */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* nothing fetched, MSR3 in EXECUTE (2) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* nothing fetched, MSR3 in EXECUTE (3) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* NOP fetched, MSR4 in EXECUTE (1) */
	/* last MSR writes flags, which takes only one cycle */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
}

void arm9tdmi_write_xpsr_im8(target_t *target, u8 xpsr_im, int rot, int spsr)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
		
	DEBUG("xpsr_im: %2.2x, rot: %i, spsr: %i", xpsr_im, rot, spsr);
	
	/* MSR fetched */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM(xpsr_im, rot, 1, spsr), 0, NULL, 0);
	/* NOP fetched, MSR in DECODE */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* NOP fetched, MSR in EXECUTE (1) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	
	/* rot == 4 writes flags, which takes only one cycle */
	if (rot != 4)
	{
		/* nothing fetched, MSR in EXECUTE (2) */
		arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
		/* nothing fetched, MSR in EXECUTE (3) */
		arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	}
}

void arm9tdmi_write_core_regs(target_t *target, u32 mask, u32 core_regs[16])
{
	int i;
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
		
	/* LDMIA r0-15, [r0] at debug speed
	* register values will start to appear on 4th DCLK
	*/
	arm9tdmi_clock_out(jtag_info, ARMV4_5_LDMIA(0, mask & 0xffff, 0, 0), 0, NULL, 0);

	/* fetch NOP, LDM in DECODE stage */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* fetch NOP, LDM in EXECUTE stage (1st cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	for (i = 0; i <= 15; i++)
	{
		if (mask & (1 << i))
			/* nothing fetched, LDM still in EXECUTE (1+i cycle) */
			arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, core_regs[i], NULL, 0);
	}
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	
}

void arm9tdmi_load_word_regs(target_t *target, u32 mask)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	/* put system-speed load-multiple into the pipeline */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_LDMIA(0, mask & 0xffff, 0, 1), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 1);

}

void arm9tdmi_load_hword_reg(target_t *target, int num)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	
	/* put system-speed load half-word into the pipeline */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_LDRH_IP(num, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 1);
}

void arm9tdmi_load_byte_reg(target_t *target, int num)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	/* put system-speed load byte into the pipeline */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_LDRB_IP(num, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 1);

}

void arm9tdmi_store_word_regs(target_t *target, u32 mask)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	/* put system-speed store-multiple into the pipeline */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_STMIA(0, mask, 0, 1), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 1);

}

void arm9tdmi_store_hword_reg(target_t *target, int num)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	/* put system-speed store half-word into the pipeline */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_STRH_IP(num, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 1);

}

void arm9tdmi_store_byte_reg(target_t *target, int num)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	/* put system-speed store byte into the pipeline */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_STRB_IP(num, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 1);

}

void arm9tdmi_write_pc(target_t *target, u32 pc)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	
	/* LDMIA r0-15, [r0] at debug speed
	 * register values will start to appear on 4th DCLK
	 */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_LDMIA(0, 0x8000, 0, 0), 0, NULL, 0);

	/* fetch NOP, LDM in DECODE stage */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* fetch NOP, LDM in EXECUTE stage (1st cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* nothing fetched, LDM in EXECUTE stage (2nd cycle) (output data) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, pc, NULL, 0);
	/* nothing fetched, LDM in EXECUTE stage (3rd cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* fetch NOP, LDM in EXECUTE stage (4th cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* fetch NOP, LDM in EXECUTE stage (5th cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

}

void arm9tdmi_branch_resume(target_t *target)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	
	arm9tdmi_clock_out(jtag_info, ARMV4_5_B(0xfffffc, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 1);

}

void arm9tdmi_branch_resume_thumb(target_t *target)
{
	DEBUG("");
	
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	reg_t *dbg_stat = &arm7_9->eice_cache->reg_list[EICE_DBG_STAT];

	/* LDMIA r0-15, [r0] at debug speed
	* register values will start to appear on 4th DCLK
	*/
	arm9tdmi_clock_out(jtag_info, ARMV4_5_LDMIA(0, 0x1, 0, 0), 0, NULL, 0);

	/* fetch NOP, LDM in DECODE stage */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* fetch NOP, LDM in EXECUTE stage (1st cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	/* nothing fetched, LDM in EXECUTE stage (2nd cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32) | 1, NULL, 0);
	/* nothing fetched, LDM in EXECUTE stage (3rd cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	/* Branch and eXchange */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_BX(0), 0, NULL, 0);
	
	embeddedice_read_reg(dbg_stat);
	
	/* fetch NOP, BX in DECODE stage */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	
	embeddedice_read_reg(dbg_stat);
	
	/* fetch NOP, BX in EXECUTE stage (1st cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	/* target is now in Thumb state */
	embeddedice_read_reg(dbg_stat);

	/* clean r0 bits to avoid alignment problems */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_MOV_IM(0, 0x0), 0, NULL, 0);
	/* load r0 value, MOV_IM in Decode*/
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_LDR(0, 0), 0, NULL, 0);
	/* fetch NOP, LDR in Decode, MOV_IM in Execute */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	/* fetch NOP, LDR in Execute */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	/* nothing fetched, LDR in EXECUTE stage (2nd cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, buf_get_u32(armv4_5->core_cache->reg_list[0].value, 0, 32), NULL, 0);
	/* nothing fetched, LDR in EXECUTE stage (3rd cycle) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);

	embeddedice_read_reg(dbg_stat);
	
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_B(0x7f6), 0, NULL, 1);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);

}

void arm9tdmi_enable_single_step(target_t *target)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm9tdmi_common_t *arm9 = arm7_9->arch_info;
	
	if (arm9->has_single_step)
	{
		buf_set_u32(arm7_9->eice_cache->reg_list[EICE_DBG_CTRL].value, 3, 1, 1);
		embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_DBG_CTRL]);
	}
	else
	{
		arm7_9_enable_eice_step(target);
	}
}

void arm9tdmi_disable_single_step(target_t *target)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm9tdmi_common_t *arm9 = arm7_9->arch_info;
	
	if (arm9->has_single_step)
	{
		buf_set_u32(arm7_9->eice_cache->reg_list[EICE_DBG_CTRL].value, 3, 1, 0);
		embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_DBG_CTRL]);
	}
	else
	{
		arm7_9_disable_eice_step(target);
	}
}

void arm9tdmi_build_reg_cache(target_t *target)
{
	reg_cache_t **cache_p = register_get_last_cache_p(&target->reg_cache);
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	arm9tdmi_common_t *arm9tdmi = arm7_9->arch_info;


	(*cache_p) = armv4_5_build_reg_cache(target, armv4_5);
	armv4_5->core_cache = (*cache_p);
	
	(*cache_p)->next = embeddedice_build_reg_cache(target, jtag_info, 0);
	arm7_9->eice_cache = (*cache_p)->next;
	
	if (arm9tdmi->has_monitor_mode)
		(*cache_p)->next->reg_list[0].size = 6;
	else
		(*cache_p)->next->reg_list[0].size = 4;
	
	(*cache_p)->next->reg_list[1].size = 5;

}

int arm9tdmi_init_target(struct command_context_s *cmd_ctx, struct target_s *target)
{
	
	arm9tdmi_build_reg_cache(target);
	
	return ERROR_OK;
	
}

int arm9tdmi_quit()
{
	
	return ERROR_OK;
}

int arm9tdmi_init_arch_info(target_t *target, arm9tdmi_common_t *arm9tdmi, int chain_pos, char *variant)
{
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	
	arm7_9 = &arm9tdmi->arm7_9_common;
	armv4_5 = &arm7_9->armv4_5_common;
	
	/* prepare JTAG information for the new target */
	arm7_9->jtag_info.chain_pos = chain_pos;
	arm7_9->jtag_info.scann_size = 5;
	
	/* register arch-specific functions */
	arm7_9->examine_debug_reason = arm9tdmi_examine_debug_reason;
	arm7_9->change_to_arm = arm9tdmi_change_to_arm;
	arm7_9->read_core_regs = arm9tdmi_read_core_regs;
	arm7_9->read_xpsr = arm9tdmi_read_xpsr;
	
	arm7_9->write_xpsr = arm9tdmi_write_xpsr;
	arm7_9->write_xpsr_im8 = arm9tdmi_write_xpsr_im8;
	arm7_9->write_core_regs = arm9tdmi_write_core_regs;
	
	arm7_9->load_word_regs = arm9tdmi_load_word_regs;
	arm7_9->load_hword_reg = arm9tdmi_load_hword_reg;
	arm7_9->load_byte_reg = arm9tdmi_load_byte_reg;
	
	arm7_9->store_word_regs = arm9tdmi_store_word_regs;
	arm7_9->store_hword_reg = arm9tdmi_store_hword_reg;
	arm7_9->store_byte_reg = arm9tdmi_store_byte_reg;
	
	arm7_9->write_pc = arm9tdmi_write_pc;
	arm7_9->branch_resume = arm9tdmi_branch_resume;
	arm7_9->branch_resume_thumb = arm9tdmi_branch_resume_thumb;

	arm7_9->enable_single_step = arm9tdmi_enable_single_step;
	arm7_9->disable_single_step = arm9tdmi_disable_single_step;
	
	arm7_9->pre_debug_entry = NULL;
	arm7_9->post_debug_entry = NULL;
	
	arm7_9->pre_restore_context = NULL;
	arm7_9->post_restore_context = NULL;

	/* initialize arch-specific breakpoint handling */
	buf_set_u32((u8*)(&arm7_9->arm_bkpt), 0, 32, 0xdeeedeee);
	buf_set_u32((u8*)(&arm7_9->thumb_bkpt), 0, 16, 0xdeee);
	
	arm7_9->sw_bkpts_use_wp = 1;
	arm7_9->sw_bkpts_enabled = 0;
	arm7_9->dbgreq_adjust_pc = 3;
	arm7_9->arch_info = arm9tdmi;
	arm7_9->use_dbgrq = 1;
	
	arm9tdmi->common_magic = ARM9TDMI_COMMON_MAGIC;
	arm9tdmi->has_monitor_mode = 0;
	arm9tdmi->has_single_step = 0;
	arm9tdmi->arch_info = NULL;

	if (variant)
	{
		if (strcmp(variant, "arm920t") == 0)
			arm9tdmi->has_single_step = 1;
		else if (strcmp(variant, "arm922t") == 0)
			arm9tdmi->has_single_step = 1;
		else if (strcmp(variant, "arm940t") == 0)
			arm9tdmi->has_single_step = 1;
	}
	
	arm7_9_init_arch_info(target, arm7_9);
	
	return ERROR_OK;
}

/* target arm9tdmi <endianess> <startup_mode> <chain_pos> <variant>*/
int arm9tdmi_target_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct target_s *target)
{
	int chain_pos;
	char *variant = NULL;
	arm9tdmi_common_t *arm9tdmi = malloc(sizeof(arm9tdmi_common_t));

	if (argc < 4)
	{
		ERROR("'target arm9tdmi' requires at least one additional argument");
		exit(-1);
	}
	
	chain_pos = strtoul(args[3], NULL, 0);
	
	if (argc >= 5)
		variant = strdup(args[4]);
	
	arm9tdmi_init_arch_info(target, arm9tdmi, chain_pos, variant);
	
	return ERROR_OK;
}

int arm9tdmi_register_commands(struct command_context_s *cmd_ctx)
{
	int retval;
	
	retval = arm7_9_register_commands(cmd_ctx);
	
	return ERROR_OK;

}

