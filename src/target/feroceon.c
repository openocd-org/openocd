/***************************************************************************
 *   Copyright (C) 2008 by Marvell Semiconductors, Inc.                    *
 *   Written by Nicolas Pitre <nico@marvell.com>                           *
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

/*
 * Marvell Feroceon support, including Orion and Kirkwood SOCs.
 *
 * The Feroceon core mimics the ARM926 ICE interface with the following
 * differences:
 *
 * - the MOE (method of entry) reporting is not implemented
 *
 * - breakpoint/watchpoint comparator #1 is seemingly not implemented
 *
 * - due to a different pipeline implementation, some injected debug
 *   instruction sequences have to be somewhat different
 *
 * Other issues:
 *
 * - asserting DBGRQ doesn't work if target is looping on the undef vector
 *
 * - the EICE version signature in the COMMS_CTL reg is next to the flow bits
 *   not at the top, and rather meaningless due to existing discrepencies
 *
 * - the DCC channel is half duplex (only one FIFO for both directions) with
 *   seemingly no proper flow control.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arm926ejs.h"
#include "jtag.h"
#include "log.h"

#include <stdlib.h>
#include <string.h>

int feroceon_examine(struct target_s *target);
int feroceon_target_create(struct target_s *target, Jim_Interp *interp);
int feroceon_bulk_write_memory(target_t *target, u32 address, u32 count, u8 *buffer);
int feroceon_init_target(struct command_context_s *cmd_ctx, struct target_s *target);
int feroceon_quit(void);

int feroceon_assert_reset(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	int ud = arm7_9->use_dbgrq;

	arm7_9->use_dbgrq = 0;
	if (target->reset_halt)
		arm7_9_halt(target);
	arm7_9->use_dbgrq = ud;
	return arm7_9_assert_reset(target);
}

target_type_t feroceon_target =
{
	.name = "feroceon",

	.poll = arm7_9_poll,
	.arch_state = arm926ejs_arch_state,

	.target_request_data = arm7_9_target_request_data,

	.halt = arm7_9_halt,
	.resume = arm7_9_resume,
	.step = arm7_9_step,

	.assert_reset = feroceon_assert_reset,
	.deassert_reset = arm7_9_deassert_reset,
	.soft_reset_halt = arm926ejs_soft_reset_halt,

	.get_gdb_reg_list = armv4_5_get_gdb_reg_list,

	.read_memory = arm7_9_read_memory,
	.write_memory = arm926ejs_write_memory,
	.bulk_write_memory = feroceon_bulk_write_memory,
	.checksum_memory = arm7_9_checksum_memory,
	.blank_check_memory = arm7_9_blank_check_memory,

	.run_algorithm = armv4_5_run_algorithm,

	.add_breakpoint = arm7_9_add_breakpoint,
	.remove_breakpoint = arm7_9_remove_breakpoint,
	.add_watchpoint = arm7_9_add_watchpoint,
	.remove_watchpoint = arm7_9_remove_watchpoint,

	.register_commands = arm926ejs_register_commands,
	.target_create = feroceon_target_create,
	.init_target = feroceon_init_target,
	.examine = feroceon_examine,
	.quit = feroceon_quit
};


int feroceon_dummy_clock_out(arm_jtag_t *jtag_info, u32 instr)
{
	scan_field_t fields[3];
	u8 out_buf[4];
	u8 instr_buf[4];
	u8 sysspeed_buf = 0x0;

	/* prepare buffer */
	buf_set_u32(out_buf, 0, 32, 0);

	buf_set_u32(instr_buf, 0, 32, flip_u32(instr, 32));

	jtag_add_end_state(TAP_DRPAUSE);
	arm_jtag_scann(jtag_info, 0x1);

	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr, NULL);

	fields[0].tap = jtag_info->tap;
	fields[0].num_bits = 32;
	fields[0].out_value = out_buf;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;
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

	/* no jtag_add_runtest(0, -1) here */

	return ERROR_OK;
}

void feroceon_change_to_arm(target_t *target, u32 *r0, u32 *pc)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	/*
	 * save r0 before using it and put system in ARM state
	 * to allow common handling of ARM and THUMB debugging
	 */

	feroceon_dummy_clock_out(jtag_info, ARMV4_5_T_NOP);
	feroceon_dummy_clock_out(jtag_info, ARMV4_5_T_NOP);
	feroceon_dummy_clock_out(jtag_info, ARMV4_5_T_NOP);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_STR(0, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, r0, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_MOV(0, 15), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_STR(0, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, pc, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_BX(15), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);

	jtag_execute_queue();

	/*
	 * fix program counter:
	 * MOV R0, PC was the 7th instruction (+12)
	 * reading PC in Thumb state gives address of instruction + 4
	 */
	*pc -= (12 + 4);
}

void feroceon_read_core_regs(target_t *target, u32 mask, u32* core_regs[16])
{
	int i;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	arm9tdmi_clock_out(jtag_info, ARMV4_5_STMIA(0, mask & 0xffff, 0, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	for (i = 0; i <= 15; i++)
		if (mask & (1 << i))
			arm9tdmi_clock_data_in(jtag_info, core_regs[i]);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
}

void feroceon_read_core_regs_target_buffer(target_t *target, u32 mask, void* buffer, int size)
{
	int i;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	int be = (target->endianness == TARGET_BIG_ENDIAN) ? 1 : 0;
	u32 *buf_u32 = buffer;
	u16 *buf_u16 = buffer;
	u8 *buf_u8 = buffer;

	arm9tdmi_clock_out(jtag_info, ARMV4_5_STMIA(0, mask & 0xffff, 0, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	for (i = 0; i <= 15; i++)
	{
		if (mask & (1 << i)) {
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

	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
}

void feroceon_read_xpsr(target_t *target, u32 *xpsr, int spsr)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	arm9tdmi_clock_out(jtag_info, ARMV4_5_MRS(0, spsr & 1), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_STMIA(0, 1, 0, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, xpsr, 0);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
}

void feroceon_write_xpsr(target_t *target, u32 xpsr, int spsr)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	LOG_DEBUG("xpsr: %8.8x, spsr: %i", xpsr, spsr);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM(xpsr & 0xff, 0, 1, spsr), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM((xpsr & 0xff00) >> 8, 0xc, 2, spsr), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM((xpsr & 0xff0000) >> 16, 0x8, 4, spsr), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM((xpsr & 0xff000000) >> 24, 0x4, 8, spsr), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
}

void feroceon_write_xpsr_im8(target_t *target, u8 xpsr_im, int rot, int spsr)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	LOG_DEBUG("xpsr_im: %2.2x, rot: %i, spsr: %i", xpsr_im, rot, spsr);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_MSR_IM(xpsr_im, rot, 1, spsr), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
}

void feroceon_write_core_regs(target_t *target, u32 mask, u32 core_regs[16])
{
	int i;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	arm9tdmi_clock_out(jtag_info, ARMV4_5_LDMIA(0, mask & 0xffff, 0, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	for (i = 0; i <= 15; i++)
		if (mask & (1 << i))
			arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, core_regs[i], NULL, 0);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
}

void feroceon_branch_resume(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_B(0xfffff9, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 1);

	arm7_9->need_bypass_before_restart = 1;
}

void feroceon_branch_resume_thumb(target_t *target)
{
	LOG_DEBUG("-");

	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	u32 r0 = buf_get_u32(armv4_5->core_cache->reg_list[0].value, 0, 32);
	u32 pc = buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32);
	(void)(r0); // use R0...

	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	arm9tdmi_clock_out(jtag_info, 0xE28F0001, 0, NULL, 0); // add r0,pc,#1
	arm9tdmi_clock_out(jtag_info, ARMV4_5_BX(0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_LDMIA(0, 0x1), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, pc, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 0);

	pc = (pc & 2) >> 1;
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_B(0x7e9 + pc), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_T_NOP, 0, NULL, 1);

	arm7_9->need_bypass_before_restart = 1;
}

int feroceon_read_cp15(target_t *target, u32 op1, u32 op2, u32 CRn, u32 CRm, u32 *value)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	int err;

	arm9tdmi_clock_out(jtag_info, ARMV4_5_MRC(15, op1, 0, CRn, CRm, op2), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 1);
	err = arm7_9_execute_sys_speed(target);
	if (err != ERROR_OK)
		return err;

	arm9tdmi_clock_out(jtag_info, ARMV4_5_STMIA(0, 1, 0, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, value, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	return jtag_execute_queue();
}

int feroceon_write_cp15(target_t *target, u32 op1, u32 op2, u32 CRn, u32 CRm, u32 value)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;

	arm9tdmi_clock_out(jtag_info, ARMV4_5_LDMIA(0, 1, 0, 0), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, value, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 0);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_MCR(15, op1, 0, CRn, CRm, op2), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 1);
	return arm7_9_execute_sys_speed(target);
}

void feroceon_set_dbgrq(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	reg_t *dbg_ctrl = &arm7_9->eice_cache->reg_list[EICE_DBG_CTRL];

	buf_set_u32(dbg_ctrl->value, 0, 8, 2);
	embeddedice_store_reg(dbg_ctrl);
}

void feroceon_enable_single_step(target_t *target, u32 next_pc)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;

	/* set a breakpoint there */
	embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_VALUE], next_pc);
	embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_MASK], 0);
	embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_DATA_MASK], 0xffffffff);
	embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_VALUE], 0x100);
	embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_MASK], 0xf7);
}

void feroceon_disable_single_step(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;

	embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_VALUE]);
	embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_W0_ADDR_MASK]);
	embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_W0_DATA_MASK]);
	embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_MASK]);
	embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_W0_CONTROL_VALUE]);
}

int feroceon_examine_debug_reason(target_t *target)
{
	/* the MOE is not implemented */
	if (target->debug_reason != DBG_REASON_SINGLESTEP)
	{
		target->debug_reason = DBG_REASON_DBGRQ;
	}

	return ERROR_OK;
}

int feroceon_bulk_write_memory(target_t *target, u32 address, u32 count, u8 *buffer)
{
	int retval;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	enum armv4_5_state core_state = armv4_5->core_state;
	u32 x, flip, shift, save[7];
	int i;

	/*
	 * We can't use the dcc flow control bits, so let's transfer data
	 * with 31 bits and flip the MSB each time a new data word is sent.
	 */
	static u32 dcc_code[] =
	{
		0xee115e10,	/* 3:	mrc	p14, 0, r5, c1, c0, 0	*/
		0xe3a0301e,	/* 1:	mov	r3, #30			*/
		0xe3a04002,	/*	mov	r4, #2			*/
		0xee111e10,	/* 2:	mrc	p14, 0, r1, c1, c0, 0	*/
		0xe1310005,	/*	teq	r1, r5			*/
		0x0afffffc,	/*	beq	1b			*/
		0xe1a05001,	/*	mov	r5, r1			*/
		0xe1a01081,	/*	mov	r1, r1, lsl #1		*/
		0xee112e10,	/* 3:	mrc	p14, 0, r2, c1, c0, 0	*/
		0xe1320005,	/*	teq	r2, r5			*/
		0x0afffffc,	/*	beq	3b			*/
		0xe1a05002,	/*	mov	r5, r2			*/
		0xe3c22102,	/*	bic	r2, r2, #0x80000000	*/
		0xe1811332,	/*	orr	r1, r1, r2, lsr r3	*/
		0xe2533001,	/*	subs	r3, r3, #1		*/
		0xe4801004,	/*	str	r1, [r0], #4		*/
		0xe1a01412,	/*	mov	r1, r2, lsl r4		*/
		0xe2844001,	/*	add	r4, r4, #1		*/
		0x4affffed,	/*	bmi	1b			*/
		0xeafffff3,	/*	b	3b			*/
	};

	int dcc_size = sizeof(dcc_code);

	if (!arm7_9->dcc_downloads)
		return target->type->write_memory(target, address, 4, count, buffer);

	/* regrab previously allocated working_area, or allocate a new one */
	if (!arm7_9->dcc_working_area)
	{
		u8 dcc_code_buf[dcc_size];

		/* make sure we have a working area */
		if (target_alloc_working_area(target, dcc_size, &arm7_9->dcc_working_area) != ERROR_OK)
		{
			LOG_INFO("no working area available, falling back to memory writes");
			return target->type->write_memory(target, address, 4, count, buffer);
		}

		/* copy target instructions to target endianness */
		for (i = 0; i < dcc_size/4; i++)
			target_buffer_set_u32(target, dcc_code_buf + i*4, dcc_code[i]);

		/* write DCC code to working area */
		if((retval = target->type->write_memory(target, arm7_9->dcc_working_area->address, 4, dcc_size/4, dcc_code_buf)) != ERROR_OK)
		{
			return retval;
		}
	}

	/* backup clobbered processor state */
	for (i = 0; i <= 5; i++)
		save[i] = buf_get_u32(armv4_5->core_cache->reg_list[i].value, 0, 32);
	save[i] = buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32);

	/* set up target address in r0 */
	buf_set_u32(armv4_5->core_cache->reg_list[0].value, 0, 32, address);
	armv4_5->core_cache->reg_list[0].valid = 1;
	armv4_5->core_cache->reg_list[0].dirty = 1;
	armv4_5->core_state = ARMV4_5_STATE_ARM;

	embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_COMMS_DATA], 0);
	arm7_9_resume(target, 0, arm7_9->dcc_working_area->address, 1, 1);

	/* send data over */
	x = 0;
	flip = 0;
	shift = 1;
	for (i = 0; i < count; i++)
	{
		u32 y = target_buffer_get_u32(target, buffer);
		u32 z = (x >> 1) | (y >> shift) | (flip ^= 0x80000000);
		embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_COMMS_DATA], z);
		x = y << (32 - shift);
		if (++shift >= 32 || i + 1 >= count)
		{
			z = (x >> 1) | (flip ^= 0x80000000);
			embeddedice_write_reg(&arm7_9->eice_cache->reg_list[EICE_COMMS_DATA], z);
			x = 0;
			shift = 1;
		}
		buffer += 4;
	}

	target_halt(target);
	while (target->state != TARGET_HALTED)
		target_poll(target);

	/* restore target state */
	for (i = 0; i <= 5; i++)
	{
		buf_set_u32(armv4_5->core_cache->reg_list[i].value, 0, 32, save[i]);
		armv4_5->core_cache->reg_list[i].valid = 1;
		armv4_5->core_cache->reg_list[i].dirty = 1;
	}
	buf_set_u32(armv4_5->core_cache->reg_list[15].value, 0, 32, save[i]);
	armv4_5->core_cache->reg_list[15].valid = 1;
	armv4_5->core_cache->reg_list[15].dirty = 1;
	armv4_5->core_state = core_state;

	return ERROR_OK;
}

int feroceon_init_target(struct command_context_s *cmd_ctx, struct target_s *target)
{
	arm9tdmi_init_target(cmd_ctx, target);
	return ERROR_OK;
}

int feroceon_quit(void)
{
	return ERROR_OK;
}

int feroceon_target_create(struct target_s *target, Jim_Interp *interp)
{
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	arm926ejs_common_t *arm926ejs = calloc(1,sizeof(arm926ejs_common_t));

	arm926ejs_init_arch_info(target, arm926ejs, target->tap);

	armv4_5 = target->arch_info;
	arm7_9 = armv4_5->arch_info;

	/* override some insn sequence functions */
	arm7_9->change_to_arm = feroceon_change_to_arm;
	arm7_9->read_core_regs = feroceon_read_core_regs;
	arm7_9->read_core_regs_target_buffer = feroceon_read_core_regs_target_buffer;
	arm7_9->read_xpsr = feroceon_read_xpsr;
	arm7_9->write_xpsr = feroceon_write_xpsr;
	arm7_9->write_xpsr_im8 = feroceon_write_xpsr_im8;
	arm7_9->write_core_regs = feroceon_write_core_regs;
	arm7_9->branch_resume = feroceon_branch_resume;
	arm7_9->branch_resume_thumb = feroceon_branch_resume_thumb;

	/* must be implemented with only one comparator */
	arm7_9->enable_single_step = feroceon_enable_single_step;
	arm7_9->disable_single_step = feroceon_disable_single_step;

	/* MOE is not implemented */
	arm7_9->examine_debug_reason = feroceon_examine_debug_reason;

	/* the standard ARM926 methods don't always work (don't ask...) */
	arm926ejs->read_cp15 = feroceon_read_cp15;
	arm926ejs->write_cp15 = feroceon_write_cp15;

	/* Note: asserting DBGRQ might not win over the undef exception.
	   If that happens then just use "arm7_9 dbgrq disable". */
	arm7_9->use_dbgrq = 1;
	arm7_9->set_special_dbgrq = feroceon_set_dbgrq;

	/* only one working comparator */
	arm7_9->wp_available_max = 1;
	arm7_9->wp1_used_default = -1;

	return ERROR_OK;
}

int feroceon_examine(struct target_s *target)
{
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	int retval;

	retval = arm9tdmi_examine(target);
	if (retval!=ERROR_OK)
		return retval;

	armv4_5 = target->arch_info;
	arm7_9 = armv4_5->arch_info;

	/* the COMMS_CTRL bits are all contiguous */
	if (buf_get_u32(arm7_9->eice_cache->reg_list[EICE_COMMS_CTRL].value, 2, 4) != 6)
		LOG_ERROR("unexpected Feroceon EICE version signature");

	arm7_9->eice_cache->reg_list[EICE_DBG_CTRL].size = 6;
	arm7_9->eice_cache->reg_list[EICE_DBG_STAT].size = 5;
	arm7_9->has_monitor_mode = 1;

	/* vector catch reg is not initialized on reset */
	embeddedice_set_reg(&arm7_9->eice_cache->reg_list[EICE_VEC_CATCH], 0);

	/* clear monitor mode, enable comparators */
	embeddedice_read_reg(&arm7_9->eice_cache->reg_list[EICE_DBG_CTRL]);
	jtag_execute_queue();
	buf_set_u32(arm7_9->eice_cache->reg_list[EICE_DBG_CTRL].value, 4, 1, 0);
	buf_set_u32(arm7_9->eice_cache->reg_list[EICE_DBG_CTRL].value, 5, 1, 0);
	embeddedice_store_reg(&arm7_9->eice_cache->reg_list[EICE_DBG_CTRL]);

	return ERROR_OK;
}
