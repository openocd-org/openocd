/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arm7tdmi.h"

#include "arm7_9_common.h"
#include "register.h"
#include "target.h"
#include "armv4_5.h"
#include "embeddedice.h"
#include "etm.h"
#include "log.h"
#include "jtag.h"
#include "arm_jtag.h"

#include <stdlib.h>
#include <string.h>

#if 0
#define _DEBUG_INSTRUCTION_EXECUTION_
#endif

/* cli handling */
int arm7tdmi_register_commands(struct command_context_s *cmd_ctx);

/* forward declarations */

int arm7tdmi_target_create(struct target_s *target,Jim_Interp *interp);
int arm7tdmi_init_target(struct command_context_s *cmd_ctx, struct target_s *target);
int arm7tdmi_quit(void);

/* target function declarations */
int arm7tdmi_poll(struct target_s *target);
int arm7tdmi_halt(target_t *target);

target_type_t arm7tdmi_target =
{
	.name = "arm7tdmi",

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

	.register_commands  = arm7tdmi_register_commands,
	.target_create  = arm7tdmi_target_create,
	.init_target = arm7tdmi_init_target,
	.examine = arm7tdmi_examine,
	.quit = arm7tdmi_quit
};

int arm7tdmi_examine_debug_reason(target_t *target)
{
	int retval = ERROR_OK;
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;

	/* only check the debug reason if we don't know it already */
	if ((target->debug_reason != DBG_REASON_DBGRQ)
			&& (target->debug_reason != DBG_REASON_SINGLESTEP))
	{
		scan_field_t fields[2];
		u8 databus[4];
		u8 breakpoint;

		jtag_add_end_state(TAP_DRPAUSE);

		fields[0].tap = arm7_9->jtag_info.tap;
		fields[0].num_bits = 1;
		fields[0].out_value = NULL;
		fields[0].out_mask = NULL;
		fields[0].in_value = &breakpoint;
		fields[0].in_check_value = NULL;
		fields[0].in_check_mask = NULL;
		fields[0].in_handler = NULL;
		fields[0].in_handler_priv = NULL;

		fields[1].tap = arm7_9->jtag_info.tap;
		fields[1].num_bits = 32;
		fields[1].out_value = NULL;
		fields[1].out_mask = NULL;
		fields[1].in_value = databus;
		fields[1].in_check_value = NULL;
		fields[1].in_check_mask = NULL;
		fields[1].in_handler = NULL;
		fields[1].in_handler_priv = NULL;

		if((retval = arm_jtag_scann(&arm7_9->jtag_info, 0x1)) != ERROR_OK)
		{
			return retval;
		}
		arm_jtag_set_instr(&arm7_9->jtag_info, arm7_9->jtag_info.intest_instr, NULL);

		jtag_add_dr_scan(2, fields, TAP_DRPAUSE);
		if((retval = jtag_execute_queue()) != ERROR_OK)
		{
			return retval;
		}

		fields[0].in_value = NULL;
		fields[0].out_value = &breakpoint;
		fields[1].in_value = NULL;
		fields[1].out_value = databus;

		jtag_add_dr_scan(2, fields, TAP_DRPAUSE);

		if (breakpoint & 1)
			target->debug_reason = DBG_REASON_WATCHPOINT;
		else
			target->debug_reason = DBG_REASON_BREAKPOINT;
	}

	return ERROR_OK;
}

static int arm7tdmi_num_bits[]={1, 32};
static __inline int arm7tdmi_clock_out_inner(arm_jtag_t *jtag_info, u32 out, int breakpoint)
{
	u32 values[2]={breakpoint, flip_u32(out, 32)};

	jtag_add_dr_out(jtag_info->tap,
			2,
			arm7tdmi_num_bits,
			values,
			-1);

	jtag_add_runtest(0, -1);

	return ERROR_OK;
}

/* put an instruction in the ARM7TDMI pipeline or write the data bus, and optionally read data */
static __inline int arm7tdmi_clock_out(arm_jtag_t *jtag_info, u32 out, u32 *deprecated, int breakpoint)
{
	jtag_add_end_state(TAP_DRPAUSE);
	arm_jtag_scann(jtag_info, 0x1);
	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr, NULL);

	return arm7tdmi_clock_out_inner(jtag_info, out, breakpoint);
}

/* clock the target, reading the databus */
int arm7tdmi_clock_data_in(arm_jtag_t *jtag_info, u32 *in)
{
	int retval = ERROR_OK;
	scan_field_t fields[2];

	jtag_add_end_state(TAP_DRPAUSE);
	if((retval = arm_jtag_scann(jtag_info, 0x1)) != ERROR_OK)
	{
		return retval;
	}
	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr, NULL);

	fields[0].tap = jtag_info->tap;
	fields[0].num_bits = 1;
	fields[0].out_value = NULL;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;

	fields[1].tap = jtag_info->tap;
	fields[1].num_bits = 32;
	fields[1].out_value = NULL;
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_handler = arm_jtag_buf_to_u32_flip;
	fields[1].in_handler_priv = in;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;

	jtag_add_dr_scan(2, fields, -1);

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
int arm7tdmi_clock_data_in_endianness(arm_jtag_t *jtag_info, void *in, int size, int be)
{
	int retval = ERROR_OK;
	scan_field_t fields[2];

	jtag_add_end_state(TAP_DRPAUSE);
	if((retval = arm_jtag_scann(jtag_info, 0x1)) != ERROR_OK)
	{
		return retval;
	}
	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr, NULL);

	fields[0].tap = jtag_info->tap;
	fields[0].num_bits = 1;
	fields[0].out_value = NULL;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;

	fields[1].tap = jtag_info->tap;
	fields[1].num_bits = 32;
	fields[1].out_value = NULL;
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	switch (size)
	{
		case 4:
			fields[1].in_handler = (be) ? arm_jtag_buf_to_be32_flip : arm_jtag_buf_to_le32_flip;
			break;
		case 2:
			fields[1].in_handler = (be) ? arm_jtag_buf_to_be16_flip : arm_jtag_buf_to_le16_flip;
			break;
		case 1:
			fields[1].in_handler = arm_jtag_buf_to_8_flip;
			break;
	}
	fields[1].in_handler_priv = in;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;

	jtag_add_dr_scan(2, fields, -1);

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

void arm7tdmi_change_to_arm(target_t *target, u32 *r0, u32 *pc)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	/* save r0 before using it and put system in ARM state
	 * to allow common handling of ARM and THUMB debugging */

	/* fetch STR r0, [r0] */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_STR(0, 0), NULL, 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, NULL, 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, NULL, 0);
	/* nothing fetched, STR r0, [r0] in Execute (2) */
	arm7tdmi_clock_data_in(jtag_info, r0);

	/* MOV r0, r15 fetched, STR in Decode */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_MOV(0, 15), NULL, 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_STR(0, 0), NULL, 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, NULL, 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, NULL, 0);
	/* nothing fetched, STR r0, [r0] in Execute (2) */
	arm7tdmi_clock_data_in(jtag_info, pc);

	/* use pc-relative LDR to clear r0[1:0] (for switch to ARM mode) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_LDR_PCREL(0), NULL, 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, NULL, 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, NULL, 0);
	/* nothing fetched, data for LDR r0, [PC, #0] */
	arm7tdmi_clock_out(jtag_info, 0x0, NULL, 0);
	/* nothing fetched, data from previous cycle is written to register */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, NULL, 0);

	/* fetch BX */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_BX(0), NULL, 0);
	/* NOP fetched, BX in Decode, MOV in Execute */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, NULL, 0);
	/* NOP fetched, BX in Execute (1) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, NULL, 0);

	jtag_execute_queue();

	/* fix program counter:
	 * MOV r0, r15 was the 4th instruction (+6)
	 * reading PC in Thumb state gives address of instruction + 4
	 */
	*pc -= 0xa;
}

void arm7tdmi_read_core_regs(target_t *target, u32 mask, u32* core_regs[16])
{
	int i;
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	/* STMIA r0-15, [r0] at debug speed
	 * register values will start to appear on 4th DCLK
	 */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_STMIA(0, mask & 0xffff, 0, 0), NULL, 0);

	/* fetch NOP, STM in DECODE stage */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);
	/* fetch NOP, STM in EXECUTE stage (1st cycle) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);

	for (i = 0; i <= 15; i++)
	{
		if (mask & (1 << i))
			/* nothing fetched, STM still in EXECUTE (1+i cycle) */
			arm7tdmi_clock_data_in(jtag_info, core_regs[i]);
	}
}

void arm7tdmi_read_core_regs_target_buffer(target_t *target, u32 mask, void* buffer, int size)
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
	arm7tdmi_clock_out(jtag_info, ARMV4_5_STMIA(0, mask & 0xffff, 0, 0), NULL, 0);

	/* fetch NOP, STM in DECODE stage */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);
	/* fetch NOP, STM in EXECUTE stage (1st cycle) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);

	for (i = 0; i <= 15; i++)
	{
		/* nothing fetched, STM still in EXECUTE (1+i cycle), read databus */
		if (mask & (1 << i))
		{
			switch (size)
			{
				case 4:
					arm7tdmi_clock_data_in_endianness(jtag_info, buf_u32++, 4, be);
					break;
				case 2:
					arm7tdmi_clock_data_in_endianness(jtag_info, buf_u16++, 2, be);
					break;
				case 1:
					arm7tdmi_clock_data_in_endianness(jtag_info, buf_u8++, 1, be);
					break;
			}
		}
	}
}

void arm7tdmi_read_xpsr(target_t *target, u32 *xpsr, int spsr)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	/* MRS r0, cpsr */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_MRS(0, spsr & 1), NULL, 0);

	/* STR r0, [r15] */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_STR(0, 15), NULL, 0);
	/* fetch NOP, STR in DECODE stage */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);
	/* fetch NOP, STR in EXECUTE stage (1st cycle) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);
	/* nothing fetched, STR still in EXECUTE (2nd cycle) */
	arm7tdmi_clock_data_in(jtag_info, xpsr);
}

void arm7tdmi_write_xpsr(target_t *target, u32 xpsr, int spsr)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	LOG_DEBUG("xpsr: %8.8x, spsr: %i", xpsr, spsr);

	/* MSR1 fetched */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM(xpsr & 0xff, 0, 1, spsr), NULL, 0);
	/* MSR2 fetched, MSR1 in DECODE */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM((xpsr & 0xff00) >> 8, 0xc, 2, spsr), NULL, 0);
	/* MSR3 fetched, MSR1 in EXECUTE (1), MSR2 in DECODE */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM((xpsr & 0xff0000) >> 16, 0x8, 4, spsr), NULL, 0);
	/* nothing fetched, MSR1 in EXECUTE (2) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);
	/* MSR4 fetched, MSR2 in EXECUTE (1), MSR3 in DECODE */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM((xpsr & 0xff000000) >> 24, 0x4, 8, spsr), NULL, 0);
	/* nothing fetched, MSR2 in EXECUTE (2) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);
	/* NOP fetched, MSR3 in EXECUTE (1), MSR4 in DECODE */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);
	/* nothing fetched, MSR3 in EXECUTE (2) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);
	/* NOP fetched, MSR4 in EXECUTE (1) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);
	/* nothing fetched, MSR4 in EXECUTE (2) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);
}

void arm7tdmi_write_xpsr_im8(target_t *target, u8 xpsr_im, int rot, int spsr)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	LOG_DEBUG("xpsr_im: %2.2x, rot: %i, spsr: %i", xpsr_im, rot, spsr);

	/* MSR fetched */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM(xpsr_im, rot, 1, spsr), NULL, 0);
	/* NOP fetched, MSR in DECODE */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);
	/* NOP fetched, MSR in EXECUTE (1) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);
	/* nothing fetched, MSR in EXECUTE (2) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);
}

void arm7tdmi_write_core_regs(target_t *target, u32 mask, u32 core_regs[16])
{
	int i;
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	/* LDMIA r0-15, [r0] at debug speed
	* register values will start to appear on 4th DCLK
	*/
	arm7tdmi_clock_out(jtag_info, ARMV4_5_LDMIA(0, mask & 0xffff, 0, 0), NULL, 0);

	/* fetch NOP, LDM in DECODE stage */
	arm7tdmi_clock_out_inner(jtag_info, ARMV4_5_NOP, 0);
	/* fetch NOP, LDM in EXECUTE stage (1st cycle) */
	arm7tdmi_clock_out_inner(jtag_info, ARMV4_5_NOP, 0);

	for (i = 0; i <= 15; i++)
	{
		if (mask & (1 << i))
			/* nothing fetched, LDM still in EXECUTE (1+i cycle) */
			arm7tdmi_clock_out_inner(jtag_info, core_regs[i], 0);
	}
	arm7tdmi_clock_out_inner(jtag_info, ARMV4_5_NOP, 0);
}

void arm7tdmi_load_word_regs(target_t *target, u32 mask)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	/* put system-speed load-multiple into the pipeline */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 1);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_LDMIA(0, mask & 0xffff, 0, 1), NULL, 0);
}

void arm7tdmi_load_hword_reg(target_t *target, int num)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	/* put system-speed load half-word into the pipeline */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 1);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_LDRH_IP(num, 0), NULL, 0);
}

void arm7tdmi_load_byte_reg(target_t *target, int num)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	/* put system-speed load byte into the pipeline */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 1);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_LDRB_IP(num, 0), NULL, 0);
}

void arm7tdmi_store_word_regs(target_t *target, u32 mask)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	/* put system-speed store-multiple into the pipeline */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 1);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_STMIA(0, mask, 0, 1), NULL, 0);
}

void arm7tdmi_store_hword_reg(target_t *target, int num)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	/* put system-speed store half-word into the pipeline */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 1);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_STRH_IP(num, 0), NULL, 0);
}

void arm7tdmi_store_byte_reg(target_t *target, int num)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	/* put system-speed store byte into the pipeline */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 1);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_STRB_IP(num, 0), NULL, 0);
}

void arm7tdmi_write_pc(target_t *target, u32 pc)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	/* LDMIA r0-15, [r0] at debug speed
	 * register values will start to appear on 4th DCLK
	 */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_LDMIA(0, 0x8000, 0, 0), NULL, 0);
	/* fetch NOP, LDM in DECODE stage */
	arm7tdmi_clock_out_inner(jtag_info, ARMV4_5_NOP, 0);
	/* fetch NOP, LDM in EXECUTE stage (1st cycle) */
	arm7tdmi_clock_out_inner(jtag_info, ARMV4_5_NOP, 0);
	/* nothing fetched, LDM in EXECUTE stage (1st cycle) load register */
	arm7tdmi_clock_out_inner(jtag_info, pc, 0);
	/* nothing fetched, LDM in EXECUTE stage (2nd cycle) load register */
	arm7tdmi_clock_out_inner(jtag_info, ARMV4_5_NOP, 0);
	/* nothing fetched, LDM in EXECUTE stage (3rd cycle) load register */
	arm7tdmi_clock_out_inner(jtag_info, ARMV4_5_NOP, 0);
	/* fetch NOP, LDM in EXECUTE stage (4th cycle) */
	arm7tdmi_clock_out_inner(jtag_info, ARMV4_5_NOP, 0);
	/* fetch NOP, LDM in EXECUTE stage (5th cycle) */
	arm7tdmi_clock_out_inner(jtag_info, ARMV4_5_NOP, 0);
}

void arm7tdmi_branch_resume(target_t *target)
{
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 1);
	arm7tdmi_clock_out_inner(jtag_info, ARMV4_5_B(0xfffffa, 0), 0);
}

void arm7tdmi_branch_resume_thumb(target_t *target)
{
	LOG_DEBUG("-");

	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	reg_t *dbg_stat = &arm7_9->eice_cache->reg_list[EICE_DBG_STAT];

	/* LDMIA r0, [r0] at debug speed
	 * register values will start to appear on 4th DCLK
	 */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_LDMIA(0, 0x1, 0, 0), NULL, 0);

	/* fetch NOP, LDM in DECODE stage */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);
	/* fetch NOP, LDM in EXECUTE stage (1st cycle) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);
	/* nothing fetched, LDM in EXECUTE stage (2nd cycle) */
	arm7tdmi_clock_out(jtag_info, buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32) | 1, NULL, 0);
	/* nothing fetched, LDM in EXECUTE stage (3rd cycle) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);

	/* Branch and eXchange */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_BX(0), NULL, 0);

	embeddedice_read_reg(dbg_stat);

	/* fetch NOP, BX in DECODE stage */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);

	/* target is now in Thumb state */
	embeddedice_read_reg(dbg_stat);

	/* fetch NOP, BX in EXECUTE stage (1st cycle) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_NOP, NULL, 0);

	/* target is now in Thumb state */
	embeddedice_read_reg(dbg_stat);

	/* load r0 value */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_LDR_PCREL(0), NULL, 0);
	/* fetch NOP, LDR in Decode */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, NULL, 0);
	/* fetch NOP, LDR in Execute */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, NULL, 0);
	/* nothing fetched, LDR in EXECUTE stage (2nd cycle) */
	arm7tdmi_clock_out(jtag_info, buf_get_u32(armv4_5->core_cache->reg_list[0].value, 0, 32), NULL, 0);
	/* nothing fetched, LDR in EXECUTE stage (3rd cycle) */
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, NULL, 0);

	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, NULL, 0);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, NULL, 0);

	embeddedice_read_reg(dbg_stat);

	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, NULL, 1);
	arm7tdmi_clock_out(jtag_info, ARMV4_5_T_B(0x7f8), NULL, 0);
}

void arm7tdmi_build_reg_cache(target_t *target)
{
	reg_cache_t **cache_p = register_get_last_cache_p(&target->reg_cache);
	/* get pointers to arch-specific information */
	armv4_5_common_t *armv4_5 = target->arch_info;

	(*cache_p) = armv4_5_build_reg_cache(target, armv4_5);
	armv4_5->core_cache = (*cache_p);
}

int arm7tdmi_examine(struct target_s *target)
{
	int retval;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	if (!target->type->examined)
	{
		/* get pointers to arch-specific information */
		reg_cache_t **cache_p = register_get_last_cache_p(&target->reg_cache);
		reg_cache_t *t=embeddedice_build_reg_cache(target, arm7_9);
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

int arm7tdmi_init_target(struct command_context_s *cmd_ctx, struct target_s *target)
{
	arm7tdmi_build_reg_cache(target);

	return ERROR_OK;
}

int arm7tdmi_quit(void)
{
	return ERROR_OK;
}

int arm7tdmi_init_arch_info(target_t *target, arm7tdmi_common_t *arm7tdmi, jtag_tap_t *tap)
{
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;

	arm7_9 = &arm7tdmi->arm7_9_common;
	armv4_5 = &arm7_9->armv4_5_common;

	/* prepare JTAG information for the new target */
	arm7_9->jtag_info.tap = tap;
	arm7_9->jtag_info.scann_size = 4;

	/* register arch-specific functions */
	arm7_9->examine_debug_reason = arm7tdmi_examine_debug_reason;
	arm7_9->change_to_arm = arm7tdmi_change_to_arm;
	arm7_9->read_core_regs = arm7tdmi_read_core_regs;
	arm7_9->read_core_regs_target_buffer = arm7tdmi_read_core_regs_target_buffer;
	arm7_9->read_xpsr = arm7tdmi_read_xpsr;

	arm7_9->write_xpsr = arm7tdmi_write_xpsr;
	arm7_9->write_xpsr_im8 = arm7tdmi_write_xpsr_im8;
	arm7_9->write_core_regs = arm7tdmi_write_core_regs;

	arm7_9->load_word_regs = arm7tdmi_load_word_regs;
	arm7_9->load_hword_reg = arm7tdmi_load_hword_reg;
	arm7_9->load_byte_reg = arm7tdmi_load_byte_reg;

	arm7_9->store_word_regs = arm7tdmi_store_word_regs;
	arm7_9->store_hword_reg = arm7tdmi_store_hword_reg;
	arm7_9->store_byte_reg = arm7tdmi_store_byte_reg;

	arm7_9->write_pc = arm7tdmi_write_pc;
	arm7_9->branch_resume = arm7tdmi_branch_resume;
	arm7_9->branch_resume_thumb = arm7tdmi_branch_resume_thumb;

	arm7_9->enable_single_step = arm7_9_enable_eice_step;
	arm7_9->disable_single_step = arm7_9_disable_eice_step;

	arm7_9->pre_debug_entry = NULL;
	arm7_9->post_debug_entry = NULL;

	arm7_9->pre_restore_context = NULL;
	arm7_9->post_restore_context = NULL;

	/* initialize arch-specific breakpoint handling */
	arm7_9->arm_bkpt = 0xdeeedeee;
	arm7_9->thumb_bkpt = 0xdeee;

	arm7_9->dbgreq_adjust_pc = 2;
	arm7_9->arch_info = arm7tdmi;

	arm7tdmi->arch_info = NULL;
	arm7tdmi->common_magic = ARM7TDMI_COMMON_MAGIC;

	arm7_9_init_arch_info(target, arm7_9);

	return ERROR_OK;
}

int arm7tdmi_target_create( struct target_s *target, Jim_Interp *interp )
{
	arm7tdmi_common_t *arm7tdmi;

	arm7tdmi = calloc(1,sizeof(arm7tdmi_common_t));
	arm7tdmi_init_arch_info(target, arm7tdmi, target->tap);

	return ERROR_OK;
}

int arm7tdmi_register_commands(struct command_context_s *cmd_ctx)
{
	int retval;

	retval = arm7_9_register_commands(cmd_ctx);

	return retval;
}
