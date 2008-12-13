/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by Hongtao Zheng                                   *
 *   hontor@126.com                                                        *
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
#include "etm.h"
#include "etb.h"
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
int handle_arm9tdmi_catch_vectors_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

/* forward declarations */
int arm9tdmi_target_create( struct target_s *target, Jim_Interp *interp );

int arm9tdmi_init_target(struct command_context_s *cmd_ctx, struct target_s *target);
int arm9tdmi_quit(void);

target_type_t arm9tdmi_target =
{
	.name = "arm9tdmi",

	.poll = arm7_9_poll,
	.arch_state = armv4_5_arch_state,

	.target_request_data = arm7_9_target_request_data,

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
	.checksum_memory = arm7_9_checksum_memory,
	.blank_check_memory = arm7_9_blank_check_memory,

	.run_algorithm = armv4_5_run_algorithm,

	.add_breakpoint = arm7_9_add_breakpoint,
	.remove_breakpoint = arm7_9_remove_breakpoint,
	.add_watchpoint = arm7_9_add_watchpoint,
	.remove_watchpoint = arm7_9_remove_watchpoint,

	.register_commands = arm9tdmi_register_commands,
	.target_create = arm9tdmi_target_create,
	.init_target = arm9tdmi_init_target,
	.examine = arm9tdmi_examine,
	.quit = arm9tdmi_quit
};

arm9tdmi_vector_t arm9tdmi_vectors[] =
{
	{"reset", ARM9TDMI_RESET_VECTOR},
	{"undef", ARM9TDMI_UNDEF_VECTOR},
	{"swi", ARM9TDMI_SWI_VECTOR},
	{"pabt", ARM9TDMI_PABT_VECTOR},
	{"dabt", ARM9TDMI_DABT_VECTOR},
	{"reserved", ARM9TDMI_RESERVED_VECTOR},
	{"irq", ARM9TDMI_IRQ_VECTOR},
	{"fiq", ARM9TDMI_FIQ_VECTOR},
	{0, 0},
};

int arm9tdmi_examine_debug_reason(target_t *target)
{
	int retval = ERROR_OK;
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

		jtag_add_end_state(TAP_DRPAUSE);

		fields[0].tap = arm7_9->jtag_info.tap;
		fields[0].num_bits = 32;
		fields[0].out_value = NULL;
		fields[0].out_mask = NULL;
		fields[0].in_value = databus;
		fields[0].in_check_value = NULL;
		fields[0].in_check_mask = NULL;
		fields[0].in_handler = NULL;
		fields[0].in_handler_priv = NULL;

		fields[1].tap = arm7_9->jtag_info.tap;
		fields[1].num_bits = 3;
		fields[1].out_value = NULL;
		fields[1].out_mask = NULL;
		fields[1].in_value = &debug_reason;
		fields[1].in_check_value = NULL;
		fields[1].in_check_mask = NULL;
		fields[1].in_handler = NULL;
		fields[1].in_handler_priv = NULL;

		fields[2].tap = arm7_9->jtag_info.tap;
		fields[2].num_bits = 32;
		fields[2].out_value = NULL;
		fields[2].out_mask = NULL;
		fields[2].in_value = instructionbus;
		fields[2].in_check_value = NULL;
		fields[2].in_check_mask = NULL;
		fields[2].in_handler = NULL;
		fields[2].in_handler_priv = NULL;

		if((retval = arm_jtag_scann(&arm7_9->jtag_info, 0x1)) != ERROR_OK)
		{
			return retval;
		}
		arm_jtag_set_instr(&arm7_9->jtag_info, arm7_9->jtag_info.intest_instr, NULL);

		jtag_add_dr_scan(3, fields, TAP_DRPAUSE);
		if((retval = jtag_execute_queue()) != ERROR_OK)
		{
			return retval;
		}

		fields[0].in_value = NULL;
		fields[0].out_value = databus;
		fields[1].in_value = NULL;
		fields[1].out_value = &debug_reason;
		fields[2].in_value = NULL;
		fields[2].out_value = instructionbus;

		jtag_add_dr_scan(3, fields, TAP_DRPAUSE);

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
	int retval = ERROR_OK;
	scan_field_t fields[3];
	u8 out_buf[4];
	u8 instr_buf[4];
	u8 sysspeed_buf = 0x0;

	/* prepare buffer */
	buf_set_u32(out_buf, 0, 32, out);

	buf_set_u32(instr_buf, 0, 32, flip_u32(instr, 32));

	if (sysspeed)
		buf_set_u32(&sysspeed_buf, 2, 1, 1);

	jtag_add_end_state(TAP_DRPAUSE);
	if((retval = arm_jtag_scann(jtag_info, 0x1)) != ERROR_OK)
	{
		return retval;
	}

	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr, NULL);

	fields[0].tap = jtag_info->tap;
	fields[0].num_bits = 32;
	fields[0].out_value = out_buf;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	if (in)
	{
		fields[0].in_handler = arm_jtag_buf_to_u32;
		fields[0].in_handler_priv = in;
	}
	else
	{
		fields[0].in_handler = NULL;
		fields[0].in_handler_priv = NULL;
	}
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;

	fields[1].tap = jtag_info->tap;
	fields[1].num_bits = 3;
	fields[1].out_value = &sysspeed_buf;
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;

	fields[2].tap = jtag_info->tap;
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
		if((retval = jtag_execute_queue()) != ERROR_OK)
		{
			return retval;
		}

		if (in)
		{
			LOG_DEBUG("instr: 0x%8.8x, out: 0x%8.8x, in: 0x%8.8x", instr, out, *in);
		}
		else
			LOG_DEBUG("instr: 0x%8.8x, out: 0x%8.8x", instr, out);
	}
#endif

	return ERROR_OK;
}

/* just read data (instruction and data-out = don't care) */
int arm9tdmi_clock_data_in(arm_jtag_t *jtag_info, u32 *in)
{
	int retval = ERROR_OK;;
	scan_field_t fields[3];

	jtag_add_end_state(TAP_DRPAUSE);
	if((retval = arm_jtag_scann(jtag_info, 0x1)) != ERROR_OK)
	{
		return retval;
	}

	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr, NULL);

	fields[0].tap = jtag_info->tap;
	fields[0].num_bits = 32;
	fields[0].out_value = NULL;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_handler = arm_jtag_buf_to_u32;
	fields[0].in_handler_priv = in;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;

	fields[1].tap = jtag_info->tap;
	fields[1].num_bits = 3;
	fields[1].out_value = NULL;
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;

	fields[2].tap = jtag_info->tap;
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
		if((retval = jtag_execute_queue()) != ERROR_OK)
		{
			return retval;
		}

		if (in)
		{
			LOG_DEBUG("in: 0x%8.8x", *in);
		}
		else
		{
			LOG_ERROR("BUG: called with in == NULL");
		}
	}
#endif

	return ERROR_OK;
}

/* clock the target, and read the databus
 * the *in pointer points to a buffer where elements of 'size' bytes
 * are stored in big (be==1) or little (be==0) endianness
 */
int arm9tdmi_clock_data_in_endianness(arm_jtag_t *jtag_info, void *in, int size, int be)
{
	int retval = ERROR_OK;
	scan_field_t fields[3];

	jtag_add_end_state(TAP_DRPAUSE);
	if((retval = arm_jtag_scann(jtag_info, 0x1)) != ERROR_OK)
	{
		return retval;
	}

	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr, NULL);

	fields[0].tap = jtag_info->tap;
	fields[0].num_bits = 32;
	fields[0].out_value = NULL;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	switch (size)
	{
		case 4:
			fields[0].in_handler = (be) ? arm_jtag_buf_to_be32 : arm_jtag_buf_to_le32;
			break;
		case 2:
			fields[0].in_handler = (be) ? arm_jtag_buf_to_be16 : arm_jtag_buf_to_le16;
			break;
		case 1:
			fields[0].in_handler = arm_jtag_buf_to_8;
			break;
	}
	fields[0].in_handler_priv = in;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;

	fields[1].tap = jtag_info->tap;
	fields[1].num_bits = 3;
	fields[1].out_value = NULL;
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;

	fields[2].tap = jtag_info->tap;
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
		if((retval = jtag_execute_queue()) != ERROR_OK)
		{
			return retval;
		}

		if (in)
		{
			LOG_DEBUG("in: 0x%8.8x", *(u32*)in);
		}
		else
		{
			LOG_ERROR("BUG: called with in == NULL");
		}
	}
#endif

	return ERROR_OK;
}

void arm9tdmi_change_to_arm(target_t *target, u32 *r0, u32 *pc)
{
	int retval = ERROR_OK;
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

	/* use pc-relative LDR to clear r0[1:0] (for switch to ARM mode) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_LDR_PCREL(0), 0, NULL, 0);
	/* LDR in Decode */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	/* LDR in Execute */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	/* LDR in Memory (to account for interlock) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);

	/* fetch BX */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_BX(0), 0, NULL, 0);
	/* NOP fetched, BX in Decode, MOV in Execute */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	/* NOP fetched, BX in Execute (1) */
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);

	if((retval = jtag_execute_queue()) != ERROR_OK)
	{
		return;
	}

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

void arm9tdmi_read_core_regs_target_buffer(target_t *target, u32 mask, void* buffer, int size)
{
	int i;
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	int be = (target->endianness == TARGET_BIG_ENDIAN) ? 1 : 0;
	u32 *buf_u32 = buffer;
	u16 *buf_u16 = buffer;
	u8 *buf_u8 = buffer;

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
			switch (size)
			{
				case 4:
					arm9tdmi_clock_data_in_endianness(jtag_info, buf_u32++, 4, be);
					break;
				case 2:
					arm9tdmi_clock_data_in_endianness(jtag_info, buf_u16++, 2, be);
					break;
				case 1:
					arm9tdmi_clock_data_in_endianness(jtag_info, buf_u8++, 1, be);
					break;
			}
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

	LOG_DEBUG("xpsr: %8.8x, spsr: %i", xpsr, spsr);

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

	LOG_DEBUG("xpsr_im: %2.2x, rot: %i, spsr: %i", xpsr_im, rot, spsr);

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
	LOG_DEBUG("-");

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

	/* load r0 value, MOV_IM in Decode*/
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_LDR_PCREL(0), 0, NULL, 0);
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

	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_B(0x7f7), 0, NULL, 1);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
}

void arm9tdmi_enable_single_step(target_t *target, u32 next_pc)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;

	if (arm7_9->has_single_step)
	{
		buf_set_u32(arm7_9->eice_cache->reg_list[EICE_DBG_CTRL].value, 3, 1, 1);
		embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_DBG_CTRL]);
	}
	else
	{
		arm7_9_enable_eice_step(target, next_pc);
	}
}

void arm9tdmi_disable_single_step(target_t *target)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;

	if (arm7_9->has_single_step)
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

	(*cache_p) = armv4_5_build_reg_cache(target, armv4_5);
	armv4_5->core_cache = (*cache_p);
}

int arm9tdmi_examine(struct target_s *target)
{
	/* get pointers to arch-specific information */
	int retval;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	if (!target->type->examined)
	{
		reg_cache_t **cache_p = register_get_last_cache_p(&target->reg_cache);
		reg_cache_t *t;
		/* one extra register (vector catch) */
		t=embeddedice_build_reg_cache(target, arm7_9);
		if (t==NULL)
			return ERROR_FAIL;
		(*cache_p) = t;
		arm7_9->eice_cache = (*cache_p);

		if (arm7_9->etm_ctx)
		{
			arm_jtag_t *jtag_info = &arm7_9->jtag_info;
			(*cache_p)->next = etm_build_reg_cache(target, jtag_info, arm7_9->etm_ctx);
			arm7_9->etm_ctx->reg_cache = (*cache_p)->next;
		}
		target->type->examined = 1;
	}
	if ((retval=embeddedice_setup(target))!=ERROR_OK)
		return retval;
	if ((retval=arm7_9_setup(target))!=ERROR_OK)
		return retval;
	if (arm7_9->etm_ctx)
	{
		if ((retval=etm_setup(target))!=ERROR_OK)
			return retval;
	}
	return ERROR_OK;
}

int arm9tdmi_init_target(struct command_context_s *cmd_ctx, struct target_s *target)
{

	arm9tdmi_build_reg_cache(target);

	return ERROR_OK;
}

int arm9tdmi_quit(void)
{
	return ERROR_OK;
}

int arm9tdmi_init_arch_info(target_t *target, arm9tdmi_common_t *arm9tdmi, jtag_tap_t *tap)
{
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;

	arm7_9 = &arm9tdmi->arm7_9_common;
	armv4_5 = &arm7_9->armv4_5_common;

	/* prepare JTAG information for the new target */
	arm7_9->jtag_info.tap = tap;
	arm7_9->jtag_info.scann_size = 5;

	/* register arch-specific functions */
	arm7_9->examine_debug_reason = arm9tdmi_examine_debug_reason;
	arm7_9->change_to_arm = arm9tdmi_change_to_arm;
	arm7_9->read_core_regs = arm9tdmi_read_core_regs;
	arm7_9->read_core_regs_target_buffer = arm9tdmi_read_core_regs_target_buffer;
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
	arm7_9->arm_bkpt = 0xdeeedeee;
	arm7_9->thumb_bkpt = 0xdeee;

	arm7_9->dbgreq_adjust_pc = 3;
	arm7_9->arch_info = arm9tdmi;

	arm9tdmi->common_magic = ARM9TDMI_COMMON_MAGIC;
	arm9tdmi->arch_info = NULL;

	arm7_9_init_arch_info(target, arm7_9);

	/* override use of DBGRQ, this is safe on ARM9TDMI */
	arm7_9->use_dbgrq = 1;

	/* all ARM9s have the vector catch register */
	arm7_9->has_vector_catch = 1;

	return ERROR_OK;
}

int arm9tdmi_get_arch_pointers(target_t *target, armv4_5_common_t **armv4_5_p, arm7_9_common_t **arm7_9_p, arm9tdmi_common_t **arm9tdmi_p)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9;
	arm9tdmi_common_t *arm9tdmi;

	if (armv4_5->common_magic != ARMV4_5_COMMON_MAGIC)
	{
		return -1;
	}

	arm7_9 = armv4_5->arch_info;
	if (arm7_9->common_magic != ARM7_9_COMMON_MAGIC)
	{
		return -1;
	}

	arm9tdmi = arm7_9->arch_info;
	if (arm9tdmi->common_magic != ARM9TDMI_COMMON_MAGIC)
	{
		return -1;
	}

	*armv4_5_p = armv4_5;
	*arm7_9_p = arm7_9;
	*arm9tdmi_p = arm9tdmi;

	return ERROR_OK;
}

int arm9tdmi_target_create(struct target_s *target, Jim_Interp *interp)
{
	arm9tdmi_common_t *arm9tdmi = calloc(1,sizeof(arm9tdmi_common_t));

	arm9tdmi_init_arch_info(target, arm9tdmi, target->tap);

	return ERROR_OK;
}

int arm9tdmi_register_commands(struct command_context_s *cmd_ctx)
{
	int retval;
	command_t *arm9tdmi_cmd;
	
	retval = arm7_9_register_commands(cmd_ctx);
	arm9tdmi_cmd = register_command(cmd_ctx, NULL, "arm9tdmi", NULL, COMMAND_ANY, "arm9tdmi specific commands");
	register_command(cmd_ctx, arm9tdmi_cmd, "vector_catch", handle_arm9tdmi_catch_vectors_command, COMMAND_EXEC, "catch arm920t vectors ['all'|'none'|'<vec1 vec2 ...>']");
	
	return retval;
}

int handle_arm9tdmi_catch_vectors_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	arm9tdmi_common_t *arm9tdmi;
	reg_t *vector_catch;
	u32 vector_catch_value;
	int i, j;

	if (arm9tdmi_get_arch_pointers(target, &armv4_5, &arm7_9, &arm9tdmi) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM9TDMI based target");
		return ERROR_OK;
	}

	vector_catch = &arm7_9->eice_cache->reg_list[EICE_VEC_CATCH];

	/* read the vector catch register if necessary */
	if (!vector_catch->valid)
		embeddedice_read_reg(vector_catch);

	/* get the current setting */
	vector_catch_value = buf_get_u32(vector_catch->value, 0, 32);

	if (argc > 0)
	{
		vector_catch_value = 0x0;
		if (strcmp(args[0], "all") == 0)
		{
			vector_catch_value = 0xdf;
		}
		else if (strcmp(args[0], "none") == 0)
		{
			/* do nothing */
		}
		else
		{
			for (i = 0; i < argc; i++)
			{
				/* go through list of vectors */
				for(j = 0; arm9tdmi_vectors[j].name; j++)
				{
					if (strcmp(args[i], arm9tdmi_vectors[j].name) == 0)
					{
						vector_catch_value |= arm9tdmi_vectors[j].value;
						break;
					}
				}

				/* complain if vector wasn't found */
				if (!arm9tdmi_vectors[j].name)
				{
					command_print(cmd_ctx, "vector '%s' not found, leaving current setting unchanged", args[i]);

					/* reread current setting */
					vector_catch_value = buf_get_u32(vector_catch->value, 0, 32);

					break;
				}
			}
		}

		/* store new settings */
		buf_set_u32(vector_catch->value, 0, 32, vector_catch_value);
		embeddedice_store_reg(vector_catch);
	}

	/* output current settings (skip RESERVED vector) */
	for (i = 0; i < 8; i++)
	{
		if (i != 5)
		{
			command_print(cmd_ctx, "%s: %s", arm9tdmi_vectors[i].name,
				(vector_catch_value & (1 << i)) ? "catch" : "don't catch");
		}
	}

	return ERROR_OK;
}
