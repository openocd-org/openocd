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

#include "arm920t.h"
#include "jtag.h"
#include "log.h"
#include "time_support.h"

#include <stdlib.h>
#include <string.h>

#if 0
#define _DEBUG_INSTRUCTION_EXECUTION_
#endif

/* cli handling */
int arm920t_register_commands(struct command_context_s *cmd_ctx);

int arm920t_handle_cp15_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int arm920t_handle_cp15i_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int arm920t_handle_virt2phys_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int arm920t_handle_cache_info_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int arm920t_handle_md_phys_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int arm920t_handle_mw_phys_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

int arm920t_handle_read_cache_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int arm920t_handle_read_mmu_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

/* forward declarations */
int arm920t_target_create(struct target_s *target, Jim_Interp *interp);
int arm920t_init_target(struct command_context_s *cmd_ctx, struct target_s *target);
int arm920t_quit(void);
int arm920t_arch_state(struct target_s *target);
int arm920t_read_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);
int arm920t_write_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);
int arm920t_soft_reset_halt(struct target_s *target);

#define ARM920T_CP15_PHYS_ADDR(x, y, z) ((x << 5) | (y << 1) << (z))

target_type_t arm920t_target =
{
	.name = "arm920t",

	.poll = arm7_9_poll,
	.arch_state = arm920t_arch_state,

	.target_request_data = arm7_9_target_request_data,

	.halt = arm7_9_halt,
	.resume = arm7_9_resume,
	.step = arm7_9_step,

	.assert_reset = arm7_9_assert_reset,
	.deassert_reset = arm7_9_deassert_reset,
	.soft_reset_halt = arm920t_soft_reset_halt,

	.get_gdb_reg_list = armv4_5_get_gdb_reg_list,

	.read_memory = arm920t_read_memory,
	.write_memory = arm920t_write_memory,
	.bulk_write_memory = arm7_9_bulk_write_memory,
	.checksum_memory = arm7_9_checksum_memory,
	.blank_check_memory = arm7_9_blank_check_memory,

	.run_algorithm = armv4_5_run_algorithm,

	.add_breakpoint = arm7_9_add_breakpoint,
	.remove_breakpoint = arm7_9_remove_breakpoint,
	.add_watchpoint = arm7_9_add_watchpoint,
	.remove_watchpoint = arm7_9_remove_watchpoint,

	.register_commands = arm920t_register_commands,
	.target_create = arm920t_target_create,
	.init_target = arm920t_init_target,
	.examine = arm9tdmi_examine,
	.quit = arm920t_quit
};

int arm920t_read_cp15_physical(target_t *target, int reg_addr, u32 *value)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	scan_field_t fields[4];
	u8 access_type_buf = 1;
	u8 reg_addr_buf = reg_addr & 0x3f;
	u8 nr_w_buf = 0;

	jtag_add_end_state(TAP_IDLE);
	arm_jtag_scann(jtag_info, 0xf);
	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr, NULL);

	fields[0].tap = jtag_info->tap;
	fields[0].num_bits = 1;
	fields[0].out_value = &access_type_buf;
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
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;

	fields[2].tap = jtag_info->tap;
	fields[2].num_bits = 6;
	fields[2].out_value = &reg_addr_buf;
	fields[2].out_mask = NULL;
	fields[2].in_value = NULL;
	fields[2].in_check_value = NULL;
	fields[2].in_check_mask = NULL;
	fields[2].in_handler = NULL;
	fields[2].in_handler_priv = NULL;

	fields[3].tap = jtag_info->tap;
	fields[3].num_bits = 1;
	fields[3].out_value = &nr_w_buf;
	fields[3].out_mask = NULL;
	fields[3].in_value = NULL;
	fields[3].in_check_value = NULL;
	fields[3].in_check_mask = NULL;
	fields[3].in_handler = NULL;
	fields[3].in_handler_priv = NULL;

	jtag_add_dr_scan(4, fields, -1);

	fields[1].in_handler_priv = value;
	fields[1].in_handler = arm_jtag_buf_to_u32;

	jtag_add_dr_scan(4, fields, -1);

#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	jtag_execute_queue();
	LOG_DEBUG("addr: 0x%x value: %8.8x", reg_addr, *value);
#endif

	return ERROR_OK;
}

int arm920t_write_cp15_physical(target_t *target, int reg_addr, u32 value)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	scan_field_t fields[4];
	u8 access_type_buf = 1;
	u8 reg_addr_buf = reg_addr & 0x3f;
	u8 nr_w_buf = 1;
	u8 value_buf[4];

	buf_set_u32(value_buf, 0, 32, value);

	jtag_add_end_state(TAP_IDLE);
	arm_jtag_scann(jtag_info, 0xf);
	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr, NULL);

	fields[0].tap = jtag_info->tap;
	fields[0].num_bits = 1;
	fields[0].out_value = &access_type_buf;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;

	fields[1].tap = jtag_info->tap;
	fields[1].num_bits = 32;
	fields[1].out_value = value_buf;
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;

	fields[2].tap = jtag_info->tap;
	fields[2].num_bits = 6;
	fields[2].out_value = &reg_addr_buf;
	fields[2].out_mask = NULL;
	fields[2].in_value = NULL;
	fields[2].in_check_value = NULL;
	fields[2].in_check_mask = NULL;
	fields[2].in_handler = NULL;
	fields[2].in_handler_priv = NULL;

	fields[3].tap = jtag_info->tap;
	fields[3].num_bits = 1;
	fields[3].out_value = &nr_w_buf;
	fields[3].out_mask = NULL;
	fields[3].in_value = NULL;
	fields[3].in_check_value = NULL;
	fields[3].in_check_mask = NULL;
	fields[3].in_handler = NULL;
	fields[3].in_handler_priv = NULL;

	jtag_add_dr_scan(4, fields, -1);

#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	LOG_DEBUG("addr: 0x%x value: %8.8x", reg_addr, value);
#endif

	return ERROR_OK;
}

int arm920t_execute_cp15(target_t *target, u32 cp15_opcode, u32 arm_opcode)
{
	int retval;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	scan_field_t fields[4];
	u8 access_type_buf = 0;		/* interpreted access */
	u8 reg_addr_buf = 0x0;
	u8 nr_w_buf = 0;
	u8 cp15_opcode_buf[4];

	jtag_add_end_state(TAP_IDLE);
	arm_jtag_scann(jtag_info, 0xf);
	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr, NULL);

	buf_set_u32(cp15_opcode_buf, 0, 32, cp15_opcode);

	fields[0].tap = jtag_info->tap;
	fields[0].num_bits = 1;
	fields[0].out_value = &access_type_buf;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;

	fields[1].tap = jtag_info->tap;
	fields[1].num_bits = 32;
	fields[1].out_value = cp15_opcode_buf;
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;

	fields[2].tap = jtag_info->tap;
	fields[2].num_bits = 6;
	fields[2].out_value = &reg_addr_buf;
	fields[2].out_mask = NULL;
	fields[2].in_value = NULL;
	fields[2].in_check_value = NULL;
	fields[2].in_check_mask = NULL;
	fields[2].in_handler = NULL;
	fields[2].in_handler_priv = NULL;

	fields[3].tap = jtag_info->tap;
	fields[3].num_bits = 1;
	fields[3].out_value = &nr_w_buf;
	fields[3].out_mask = NULL;
	fields[3].in_value = NULL;
	fields[3].in_check_value = NULL;
	fields[3].in_check_mask = NULL;
	fields[3].in_handler = NULL;
	fields[3].in_handler_priv = NULL;

	jtag_add_dr_scan(4, fields, -1);

	arm9tdmi_clock_out(jtag_info, arm_opcode, 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 1);
	retval = arm7_9_execute_sys_speed(target);
	if (retval != ERROR_OK)
		return retval;

	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		LOG_ERROR("failed executing JTAG queue, exiting");
		return retval;
	}

	return ERROR_OK;
}

int arm920t_read_cp15_interpreted(target_t *target, u32 cp15_opcode, u32 address, u32 *value)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	u32* regs_p[1];
	u32 regs[2];
	u32 cp15c15 = 0x0;

	/* load address into R1 */
	regs[1] = address;
	arm9tdmi_write_core_regs(target, 0x2, regs);

	/* read-modify-write CP15 test state register
	* to enable interpreted access mode */
	arm920t_read_cp15_physical(target, 0x1e, &cp15c15);
	jtag_execute_queue();
	cp15c15 |= 1;	/* set interpret mode */
	arm920t_write_cp15_physical(target, 0x1e, cp15c15);

	/* execute CP15 instruction and ARM load (reading from coprocessor) */
	arm920t_execute_cp15(target, cp15_opcode, ARMV4_5_LDR(0, 1));

	/* disable interpreted access mode */
	cp15c15 &= ~1U;	/* clear interpret mode */
	arm920t_write_cp15_physical(target, 0x1e, cp15c15);

	/* retrieve value from R0 */
	regs_p[0] = value;
	arm9tdmi_read_core_regs(target, 0x1, regs_p);
	jtag_execute_queue();

#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	LOG_DEBUG("cp15_opcode: %8.8x, address: %8.8x, value: %8.8x", cp15_opcode, address, *value);
#endif

	if (armv4_5_mode_to_number(armv4_5->core_mode)==-1)
		return ERROR_FAIL;

	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 0).dirty = 1;
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 1).dirty = 1;

	return ERROR_OK;
}

int arm920t_write_cp15_interpreted(target_t *target, u32 cp15_opcode, u32 value, u32 address)
{
	u32 cp15c15 = 0x0;
	armv4_5_common_t *armv4_5 = target->arch_info;
	u32 regs[2];

	/* load value, address into R0, R1 */
	regs[0] = value;
	regs[1] = address;
	arm9tdmi_write_core_regs(target, 0x3, regs);

	/* read-modify-write CP15 test state register
	* to enable interpreted access mode */
	arm920t_read_cp15_physical(target, 0x1e, &cp15c15);
	jtag_execute_queue();
	cp15c15 |= 1;	/* set interpret mode */
	arm920t_write_cp15_physical(target, 0x1e, cp15c15);

	/* execute CP15 instruction and ARM store (writing to coprocessor) */
	arm920t_execute_cp15(target, cp15_opcode, ARMV4_5_STR(0, 1));

	/* disable interpreted access mode */
	cp15c15 &= ~1U;	/* set interpret mode */
	arm920t_write_cp15_physical(target, 0x1e, cp15c15);

#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	LOG_DEBUG("cp15_opcode: %8.8x, value: %8.8x, address: %8.8x", cp15_opcode, value, address);
#endif

	if (armv4_5_mode_to_number(armv4_5->core_mode)==-1)
		return ERROR_FAIL;

	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 0).dirty = 1;
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 1).dirty = 1;

	return ERROR_OK;
}

u32 arm920t_get_ttb(target_t *target)
{
	int retval;
	u32 ttb = 0x0;

	if ((retval = arm920t_read_cp15_interpreted(target, 0xeebf0f51, 0x0, &ttb)) != ERROR_OK)
		return retval;

	return ttb;
}

void arm920t_disable_mmu_caches(target_t *target, int mmu, int d_u_cache, int i_cache)
{
	u32 cp15_control;

	/* read cp15 control register */
	arm920t_read_cp15_physical(target, 0x2, &cp15_control);
	jtag_execute_queue();

	if (mmu)
		cp15_control &= ~0x1U;

	if (d_u_cache)
		cp15_control &= ~0x4U;

	if (i_cache)
		cp15_control &= ~0x1000U;

	arm920t_write_cp15_physical(target, 0x2, cp15_control);
}

void arm920t_enable_mmu_caches(target_t *target, int mmu, int d_u_cache, int i_cache)
{
	u32 cp15_control;

	/* read cp15 control register */
	arm920t_read_cp15_physical(target, 0x2, &cp15_control);
	jtag_execute_queue();

	if (mmu)
		cp15_control |= 0x1U;

	if (d_u_cache)
		cp15_control |= 0x4U;

	if (i_cache)
		cp15_control |= 0x1000U;

	arm920t_write_cp15_physical(target, 0x2, cp15_control);
}

void arm920t_post_debug_entry(target_t *target)
{
	u32 cp15c15;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm9tdmi_common_t *arm9tdmi = arm7_9->arch_info;
	arm920t_common_t *arm920t = arm9tdmi->arch_info;

	/* examine cp15 control reg */
	arm920t_read_cp15_physical(target, 0x2, &arm920t->cp15_control_reg);
	jtag_execute_queue();
	LOG_DEBUG("cp15_control_reg: %8.8x", arm920t->cp15_control_reg);

	if (arm920t->armv4_5_mmu.armv4_5_cache.ctype == -1)
	{
		u32 cache_type_reg;
		/* identify caches */
		arm920t_read_cp15_physical(target, 0x1, &cache_type_reg);
		jtag_execute_queue();
		armv4_5_identify_cache(cache_type_reg, &arm920t->armv4_5_mmu.armv4_5_cache);
	}

	arm920t->armv4_5_mmu.mmu_enabled = (arm920t->cp15_control_reg & 0x1U) ? 1 : 0;
	arm920t->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled = (arm920t->cp15_control_reg & 0x4U) ? 1 : 0;
	arm920t->armv4_5_mmu.armv4_5_cache.i_cache_enabled = (arm920t->cp15_control_reg & 0x1000U) ? 1 : 0;

	/* save i/d fault status and address register */
	arm920t_read_cp15_interpreted(target, 0xee150f10, 0x0, &arm920t->d_fsr);
	arm920t_read_cp15_interpreted(target, 0xee150f30, 0x0, &arm920t->i_fsr);
	arm920t_read_cp15_interpreted(target, 0xee160f10, 0x0, &arm920t->d_far);
	arm920t_read_cp15_interpreted(target, 0xee160f30, 0x0, &arm920t->i_far);

	LOG_DEBUG("D FSR: 0x%8.8x, D FAR: 0x%8.8x, I FSR: 0x%8.8x, I FAR: 0x%8.8x",
		arm920t->d_fsr, arm920t->d_far, arm920t->i_fsr, arm920t->i_far);

	if (arm920t->preserve_cache)
	{
		/* read-modify-write CP15 test state register
		 * to disable I/D-cache linefills */
		arm920t_read_cp15_physical(target, 0x1e, &cp15c15);
		jtag_execute_queue();
		cp15c15 |= 0x600;
		arm920t_write_cp15_physical(target, 0x1e, cp15c15);
	}
}

void arm920t_pre_restore_context(target_t *target)
{
	u32 cp15c15;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm9tdmi_common_t *arm9tdmi = arm7_9->arch_info;
	arm920t_common_t *arm920t = arm9tdmi->arch_info;

	/* restore i/d fault status and address register */
	arm920t_write_cp15_interpreted(target, 0xee050f10, arm920t->d_fsr, 0x0);
	arm920t_write_cp15_interpreted(target, 0xee050f30, arm920t->i_fsr, 0x0);
	arm920t_write_cp15_interpreted(target, 0xee060f10, arm920t->d_far, 0x0);
	arm920t_write_cp15_interpreted(target, 0xee060f30, arm920t->i_far, 0x0);

	/* read-modify-write CP15 test state register
	* to reenable I/D-cache linefills */
	if (arm920t->preserve_cache)
	{
		arm920t_read_cp15_physical(target, 0x1e, &cp15c15);
		jtag_execute_queue();
		cp15c15 &= ~0x600U;
		arm920t_write_cp15_physical(target, 0x1e, cp15c15);
	}
}

int arm920t_get_arch_pointers(target_t *target, armv4_5_common_t **armv4_5_p, arm7_9_common_t **arm7_9_p, arm9tdmi_common_t **arm9tdmi_p, arm920t_common_t **arm920t_p)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9;
	arm9tdmi_common_t *arm9tdmi;
	arm920t_common_t *arm920t;

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

	arm920t = arm9tdmi->arch_info;
	if (arm920t->common_magic != ARM920T_COMMON_MAGIC)
	{
		return -1;
	}

	*armv4_5_p = armv4_5;
	*arm7_9_p = arm7_9;
	*arm9tdmi_p = arm9tdmi;
	*arm920t_p = arm920t;

	return ERROR_OK;
}

int arm920t_arch_state(struct target_s *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm9tdmi_common_t *arm9tdmi = arm7_9->arch_info;
	arm920t_common_t *arm920t = arm9tdmi->arch_info;

	char *state[] =
	{
		"disabled", "enabled"
	};

	if (armv4_5->common_magic != ARMV4_5_COMMON_MAGIC)
	{
		LOG_ERROR("BUG: called for a non-ARMv4/5 target");
		exit(-1);
	}

	LOG_USER(	"target halted in %s state due to %s, current mode: %s\n"
			"cpsr: 0x%8.8x pc: 0x%8.8x\n"
			"MMU: %s, D-Cache: %s, I-Cache: %s",
			 armv4_5_state_strings[armv4_5->core_state],
			 Jim_Nvp_value2name_simple(nvp_target_debug_reason, target->debug_reason)->name,
			 armv4_5_mode_strings[armv4_5_mode_to_number(armv4_5->core_mode)],
			 buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 32),
			 buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32),
			 state[arm920t->armv4_5_mmu.mmu_enabled],
			 state[arm920t->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled],
			 state[arm920t->armv4_5_mmu.armv4_5_cache.i_cache_enabled]);

	return ERROR_OK;
}

int arm920t_read_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer)
{
	int retval;

	retval = arm7_9_read_memory(target, address, size, count, buffer);

	return retval;
}

int arm920t_write_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer)
{
	int retval;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm9tdmi_common_t *arm9tdmi = arm7_9->arch_info;
	arm920t_common_t *arm920t = arm9tdmi->arch_info;

	if ((retval = arm7_9_write_memory(target, address, size, count, buffer)) != ERROR_OK)
		return retval;

	if (((size == 4) || (size == 2)) && (count == 1))
	{
		if (arm920t->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled)
		{
			LOG_DEBUG("D-Cache enabled, writing through to main memory");
			u32 pa, cb, ap;
			int type, domain;

			pa = armv4_5_mmu_translate_va(target, &arm920t->armv4_5_mmu, address, &type, &cb, &domain, &ap);
			if (type == -1)
				return ERROR_OK;
			/* cacheable & bufferable means write-back region */
			if (cb == 3)
				armv4_5_mmu_write_physical(target, &arm920t->armv4_5_mmu, pa, size, count, buffer);
		}

		if (arm920t->armv4_5_mmu.armv4_5_cache.i_cache_enabled)
		{
			LOG_DEBUG("I-Cache enabled, invalidating affected I-Cache line");
			arm920t_write_cp15_interpreted(target, 0xee070f35, 0x0, address);
		}
	}

	return retval;
}

int arm920t_soft_reset_halt(struct target_s *target)
{
	int retval = ERROR_OK;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm9tdmi_common_t *arm9tdmi = arm7_9->arch_info;
	arm920t_common_t *arm920t = arm9tdmi->arch_info;
	reg_t *dbg_stat = &arm7_9->eice_cache->reg_list[EICE_DBG_STAT];

	if((retval = target_halt(target)) != ERROR_OK)
	{
		return retval;
	}

	long long then=timeval_ms();
	int timeout;
	while (!(timeout=((timeval_ms()-then)>1000)))
	{
		if (buf_get_u32(dbg_stat->value, EICE_DBG_STATUS_DBGACK, 1) == 0)
		{
			embeddedice_read_reg(dbg_stat);
			if((retval = jtag_execute_queue()) != ERROR_OK)
			{
				return retval;
			}
		} else
		{
			break;
		}
		if (debug_level>=3)
		{
			/* do not eat all CPU, time out after 1 se*/
			alive_sleep(100);
		} else
		{
			keep_alive();
		}
	}
	if (timeout)
	{
		LOG_ERROR("Failed to halt CPU after 1 sec");
		return ERROR_TARGET_TIMEOUT;
	}

	target->state = TARGET_HALTED;

	/* SVC, ARM state, IRQ and FIQ disabled */
	buf_set_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 8, 0xd3);
	armv4_5->core_cache->reg_list[ARMV4_5_CPSR].dirty = 1;
	armv4_5->core_cache->reg_list[ARMV4_5_CPSR].valid = 1;

	/* start fetching from 0x0 */
	buf_set_u32(armv4_5->core_cache->reg_list[15].value, 0, 32, 0x0);
	armv4_5->core_cache->reg_list[15].dirty = 1;
	armv4_5->core_cache->reg_list[15].valid = 1;

	armv4_5->core_mode = ARMV4_5_MODE_SVC;
	armv4_5->core_state = ARMV4_5_STATE_ARM;

	arm920t_disable_mmu_caches(target, 1, 1, 1);
	arm920t->armv4_5_mmu.mmu_enabled = 0;
	arm920t->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled = 0;
	arm920t->armv4_5_mmu.armv4_5_cache.i_cache_enabled = 0;

	if((retval = target_call_event_callbacks(target, TARGET_EVENT_HALTED)) != ERROR_OK)
	{
		return retval;
	}

	return ERROR_OK;
}

int arm920t_init_target(struct command_context_s *cmd_ctx, struct target_s *target)
{
	arm9tdmi_init_target(cmd_ctx, target);

	return ERROR_OK;
}

int arm920t_quit(void)
{
	return ERROR_OK;
}

int arm920t_init_arch_info(target_t *target, arm920t_common_t *arm920t, jtag_tap_t *tap)
{
	arm9tdmi_common_t *arm9tdmi = &arm920t->arm9tdmi_common;
	arm7_9_common_t *arm7_9 = &arm9tdmi->arm7_9_common;

	/* initialize arm9tdmi specific info (including arm7_9 and armv4_5)
	 */
	arm9tdmi_init_arch_info(target, arm9tdmi, tap);

	arm9tdmi->arch_info = arm920t;
	arm920t->common_magic = ARM920T_COMMON_MAGIC;

	arm7_9->post_debug_entry = arm920t_post_debug_entry;
	arm7_9->pre_restore_context = arm920t_pre_restore_context;

	arm920t->armv4_5_mmu.armv4_5_cache.ctype = -1;
	arm920t->armv4_5_mmu.get_ttb = arm920t_get_ttb;
	arm920t->armv4_5_mmu.read_memory = arm7_9_read_memory;
	arm920t->armv4_5_mmu.write_memory = arm7_9_write_memory;
	arm920t->armv4_5_mmu.disable_mmu_caches = arm920t_disable_mmu_caches;
	arm920t->armv4_5_mmu.enable_mmu_caches = arm920t_enable_mmu_caches;
	arm920t->armv4_5_mmu.has_tiny_pages = 1;
	arm920t->armv4_5_mmu.mmu_enabled = 0;

	/* disabling linefills leads to lockups, so keep them enabled for now
	 * this doesn't affect correctness, but might affect timing issues, if
	 * important data is evicted from the cache during the debug session
	 * */
	arm920t->preserve_cache = 0;

	/* override hw single-step capability from ARM9TDMI */
	arm7_9->has_single_step = 1;

	return ERROR_OK;
}

int arm920t_target_create(struct target_s *target, Jim_Interp *interp)
{
	arm920t_common_t *arm920t = calloc(1,sizeof(arm920t_common_t));

	arm920t_init_arch_info(target, arm920t, target->tap);

	return ERROR_OK;
}

int arm920t_register_commands(struct command_context_s *cmd_ctx)
{
	int retval;
	command_t *arm920t_cmd;


	retval = arm9tdmi_register_commands(cmd_ctx);

	arm920t_cmd = register_command(cmd_ctx, NULL, "arm920t", NULL, COMMAND_ANY, "arm920t specific commands");

	register_command(cmd_ctx, arm920t_cmd, "cp15", arm920t_handle_cp15_command, COMMAND_EXEC, "display/modify cp15 register <num> [value]");
	register_command(cmd_ctx, arm920t_cmd, "cp15i", arm920t_handle_cp15i_command, COMMAND_EXEC, "display/modify cp15 (interpreted access) <opcode> [value] [address]");
	register_command(cmd_ctx, arm920t_cmd, "cache_info", arm920t_handle_cache_info_command, COMMAND_EXEC, "display information about target caches");
	register_command(cmd_ctx, arm920t_cmd, "virt2phys", arm920t_handle_virt2phys_command, COMMAND_EXEC, "translate va to pa <va>");

	register_command(cmd_ctx, arm920t_cmd, "mdw_phys", arm920t_handle_md_phys_command, COMMAND_EXEC, "display memory words <physical addr> [count]");
	register_command(cmd_ctx, arm920t_cmd, "mdh_phys", arm920t_handle_md_phys_command, COMMAND_EXEC, "display memory half-words <physical addr> [count]");
	register_command(cmd_ctx, arm920t_cmd, "mdb_phys", arm920t_handle_md_phys_command, COMMAND_EXEC, "display memory bytes <physical addr> [count]");

	register_command(cmd_ctx, arm920t_cmd, "mww_phys", arm920t_handle_mw_phys_command, COMMAND_EXEC, "write memory word <physical addr> <value>");
	register_command(cmd_ctx, arm920t_cmd, "mwh_phys", arm920t_handle_mw_phys_command, COMMAND_EXEC, "write memory half-word <physical addr> <value>");
	register_command(cmd_ctx, arm920t_cmd, "mwb_phys", arm920t_handle_mw_phys_command, COMMAND_EXEC, "write memory byte <physical addr> <value>");

	register_command(cmd_ctx, arm920t_cmd, "read_cache", arm920t_handle_read_cache_command, COMMAND_EXEC, "display I/D cache content");
	register_command(cmd_ctx, arm920t_cmd, "read_mmu", arm920t_handle_read_mmu_command, COMMAND_EXEC, "display I/D mmu content");

	return retval;
}

int arm920t_handle_read_cache_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int retval = ERROR_OK;
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	arm9tdmi_common_t *arm9tdmi;
	arm920t_common_t *arm920t;
	arm_jtag_t *jtag_info;
	u32 cp15c15;
	u32 cp15_ctrl, cp15_ctrl_saved;
	u32 regs[16];
	u32 *regs_p[16];
	u32 C15_C_D_Ind, C15_C_I_Ind;
	int i;
	FILE *output;
	arm920t_cache_line_t d_cache[8][64], i_cache[8][64];
	int segment, index;

	if (argc != 1)
	{
		command_print(cmd_ctx, "usage: arm920t read_cache <filename>");
		return ERROR_OK;
	}

	if ((output = fopen(args[0], "w")) == NULL)
	{
		LOG_DEBUG("error opening cache content file");
		return ERROR_OK;
	}

	for (i = 0; i < 16; i++)
		regs_p[i] = &regs[i];

	if (arm920t_get_arch_pointers(target, &armv4_5, &arm7_9, &arm9tdmi, &arm920t) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM920t target");
		return ERROR_OK;
	}

	jtag_info = &arm7_9->jtag_info;

	/* disable MMU and Caches */
	arm920t_read_cp15_physical(target, ARM920T_CP15_PHYS_ADDR(0, 0x1, 0), &cp15_ctrl);
	if((retval = jtag_execute_queue()) != ERROR_OK)
	{
		return retval;
	}
	cp15_ctrl_saved = cp15_ctrl;
	cp15_ctrl &= ~(ARMV4_5_MMU_ENABLED | ARMV4_5_D_U_CACHE_ENABLED | ARMV4_5_I_CACHE_ENABLED);
	arm920t_write_cp15_physical(target, ARM920T_CP15_PHYS_ADDR(0, 0x1, 0), cp15_ctrl);

	/* read CP15 test state register */
	arm920t_read_cp15_physical(target, ARM920T_CP15_PHYS_ADDR(0, 0xf, 0), &cp15c15);
	jtag_execute_queue();

	/* read DCache content */
	fprintf(output, "DCache:\n");

	/* go through segments 0 to nsets (8 on ARM920T, 4 on ARM922T) */
	for (segment = 0; segment < arm920t->armv4_5_mmu.armv4_5_cache.d_u_size.nsets; segment++)
	{
		fprintf(output, "\nsegment: %i\n----------", segment);

		/* Ra: r0 = SBZ(31:8):segment(7:5):SBZ(4:0) */
		regs[0] = 0x0 | (segment << 5);
		arm9tdmi_write_core_regs(target, 0x1, regs);

		/* set interpret mode */
		cp15c15 |= 0x1;
		arm920t_write_cp15_physical(target, ARM920T_CP15_PHYS_ADDR(0, 0xf, 0), cp15c15);

		/* D CAM Read, loads current victim into C15.C.D.Ind */
		arm920t_execute_cp15(target, ARMV4_5_MCR(15,2,0,15,6,2), ARMV4_5_LDR(1, 0));

		/* read current victim */
		arm920t_read_cp15_physical(target, 0x3d, &C15_C_D_Ind);

		/* clear interpret mode */
		cp15c15 &= ~0x1;
		arm920t_write_cp15_physical(target, 0x1e, cp15c15);

		for (index = 0; index < 64; index++)
		{
			/* Ra: r0 = index(31:26):SBZ(25:8):segment(7:5):SBZ(4:0) */
			regs[0] = 0x0 | (segment << 5) | (index << 26);
			arm9tdmi_write_core_regs(target, 0x1, regs);

			/* set interpret mode */
			cp15c15 |= 0x1;
			arm920t_write_cp15_physical(target, ARM920T_CP15_PHYS_ADDR(0, 0xf, 0), cp15c15);

			/* Write DCache victim */
			arm920t_execute_cp15(target, ARMV4_5_MCR(15,0,0,9,1,0), ARMV4_5_LDR(1, 0));

			/* Read D RAM */
			arm920t_execute_cp15(target, ARMV4_5_MCR(15,2,0,15,10,2), ARMV4_5_LDMIA(0, 0x1fe, 0, 0));

			/* Read D CAM */
			arm920t_execute_cp15(target, ARMV4_5_MCR(15,2,0,15,6,2), ARMV4_5_LDR(9, 0));

			/* clear interpret mode */
			cp15c15 &= ~0x1;
			arm920t_write_cp15_physical(target, 0x1e, cp15c15);

			/* read D RAM and CAM content */
			arm9tdmi_read_core_regs(target, 0x3fe, regs_p);
			if((retval = jtag_execute_queue()) != ERROR_OK)
			{
				return retval;
			}

			d_cache[segment][index].cam = regs[9];

			/* mask LFSR[6] */
			regs[9] &= 0xfffffffe;
			fprintf(output, "\nsegment: %i, index: %i, CAM: 0x%8.8x, content (%s):\n", segment, index, regs[9], (regs[9] & 0x10) ? "valid" : "invalid");

			for (i = 1; i < 9; i++)
			{
				 d_cache[segment][index].data[i] = regs[i];
				 fprintf(output, "%i: 0x%8.8x\n", i-1, regs[i]);
			}

		}

		/* Ra: r0 = index(31:26):SBZ(25:8):segment(7:5):SBZ(4:0) */
		regs[0] = 0x0 | (segment << 5) | (C15_C_D_Ind << 26);
		arm9tdmi_write_core_regs(target, 0x1, regs);

		/* set interpret mode */
		cp15c15 |= 0x1;
		arm920t_write_cp15_physical(target, ARM920T_CP15_PHYS_ADDR(0, 0xf, 0), cp15c15);

		/* Write DCache victim */
		arm920t_execute_cp15(target, ARMV4_5_MCR(15,0,0,9,1,0), ARMV4_5_LDR(1, 0));

		/* clear interpret mode */
		cp15c15 &= ~0x1;
		arm920t_write_cp15_physical(target, 0x1e, cp15c15);
	}

	/* read ICache content */
	fprintf(output, "ICache:\n");

	/* go through segments 0 to nsets (8 on ARM920T, 4 on ARM922T) */
	for (segment = 0; segment < arm920t->armv4_5_mmu.armv4_5_cache.d_u_size.nsets; segment++)
	{
		fprintf(output, "segment: %i\n----------", segment);

		/* Ra: r0 = SBZ(31:8):segment(7:5):SBZ(4:0) */
		regs[0] = 0x0 | (segment << 5);
		arm9tdmi_write_core_regs(target, 0x1, regs);

		/* set interpret mode */
		cp15c15 |= 0x1;
		arm920t_write_cp15_physical(target, ARM920T_CP15_PHYS_ADDR(0, 0xf, 0), cp15c15);

		/* I CAM Read, loads current victim into C15.C.I.Ind */
		arm920t_execute_cp15(target, ARMV4_5_MCR(15,2,0,15,5,2), ARMV4_5_LDR(1, 0));

		/* read current victim */
		arm920t_read_cp15_physical(target, 0x3b, &C15_C_I_Ind);

		/* clear interpret mode */
		cp15c15 &= ~0x1;
		arm920t_write_cp15_physical(target, 0x1e, cp15c15);

		for (index = 0; index < 64; index++)
		{
			/* Ra: r0 = index(31:26):SBZ(25:8):segment(7:5):SBZ(4:0) */
			regs[0] = 0x0 | (segment << 5) | (index << 26);
			arm9tdmi_write_core_regs(target, 0x1, regs);

			/* set interpret mode */
			cp15c15 |= 0x1;
			arm920t_write_cp15_physical(target, ARM920T_CP15_PHYS_ADDR(0, 0xf, 0), cp15c15);

			/* Write ICache victim */
			arm920t_execute_cp15(target, ARMV4_5_MCR(15,0,0,9,1,1), ARMV4_5_LDR(1, 0));

			/* Read I RAM */
			arm920t_execute_cp15(target, ARMV4_5_MCR(15,2,0,15,9,2), ARMV4_5_LDMIA(0, 0x1fe, 0, 0));

			/* Read I CAM */
			arm920t_execute_cp15(target, ARMV4_5_MCR(15,2,0,15,5,2), ARMV4_5_LDR(9, 0));

			/* clear interpret mode */
			cp15c15 &= ~0x1;
			arm920t_write_cp15_physical(target, 0x1e, cp15c15);

			/* read I RAM and CAM content */
			arm9tdmi_read_core_regs(target, 0x3fe, regs_p);
			if((retval = jtag_execute_queue()) != ERROR_OK)
			{
				return retval;
			}

			i_cache[segment][index].cam = regs[9];

			/* mask LFSR[6] */
			regs[9] &= 0xfffffffe;
			fprintf(output, "\nsegment: %i, index: %i, CAM: 0x%8.8x, content (%s):\n", segment, index, regs[9], (regs[9] & 0x10) ? "valid" : "invalid");

			for (i = 1; i < 9; i++)
			{
				 i_cache[segment][index].data[i] = regs[i];
				 fprintf(output, "%i: 0x%8.8x\n", i-1, regs[i]);
			}
		}

		/* Ra: r0 = index(31:26):SBZ(25:8):segment(7:5):SBZ(4:0) */
		regs[0] = 0x0 | (segment << 5) | (C15_C_D_Ind << 26);
		arm9tdmi_write_core_regs(target, 0x1, regs);

		/* set interpret mode */
		cp15c15 |= 0x1;
		arm920t_write_cp15_physical(target, ARM920T_CP15_PHYS_ADDR(0, 0xf, 0), cp15c15);

		/* Write ICache victim */
		arm920t_execute_cp15(target, ARMV4_5_MCR(15,0,0,9,1,1), ARMV4_5_LDR(1, 0));

		/* clear interpret mode */
		cp15c15 &= ~0x1;
		arm920t_write_cp15_physical(target, 0x1e, cp15c15);
	}

	/* restore CP15 MMU and Cache settings */
	arm920t_write_cp15_physical(target, ARM920T_CP15_PHYS_ADDR(0, 0x1, 0), cp15_ctrl_saved);

	command_print(cmd_ctx, "cache content successfully output to %s", args[0]);

	fclose(output);

	if (armv4_5_mode_to_number(armv4_5->core_mode)==-1)
		return ERROR_FAIL;

	/* mark registers dirty. */
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 0).dirty = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 0).valid;
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 1).dirty = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 1).valid;
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 2).dirty = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 2).valid;
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 3).dirty = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 3).valid;
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 4).dirty = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 4).valid;
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 5).dirty = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 5).valid;
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 6).dirty = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 6).valid;
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 7).dirty = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 7).valid;
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 8).dirty = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 8).valid;
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 9).dirty = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 9).valid;

	return ERROR_OK;
}

int arm920t_handle_read_mmu_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int retval = ERROR_OK;
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	arm9tdmi_common_t *arm9tdmi;
	arm920t_common_t *arm920t;
	arm_jtag_t *jtag_info;
	u32 cp15c15;
	u32 cp15_ctrl, cp15_ctrl_saved;
	u32 regs[16];
	u32 *regs_p[16];
	int i;
	FILE *output;
	u32 Dlockdown, Ilockdown;
	arm920t_tlb_entry_t d_tlb[64], i_tlb[64];
	int victim;

	if (argc != 1)
	{
		command_print(cmd_ctx, "usage: arm920t read_mmu <filename>");
		return ERROR_OK;
	}

	if ((output = fopen(args[0], "w")) == NULL)
	{
		LOG_DEBUG("error opening mmu content file");
		return ERROR_OK;
	}

	for (i = 0; i < 16; i++)
		regs_p[i] = &regs[i];

	if (arm920t_get_arch_pointers(target, &armv4_5, &arm7_9, &arm9tdmi, &arm920t) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM920t target");
		return ERROR_OK;
	}

	jtag_info = &arm7_9->jtag_info;

	/* disable MMU and Caches */
	arm920t_read_cp15_physical(target, ARM920T_CP15_PHYS_ADDR(0, 0x1, 0), &cp15_ctrl);
	if((retval = jtag_execute_queue()) != ERROR_OK)
	{
		return retval;
	}
	cp15_ctrl_saved = cp15_ctrl;
	cp15_ctrl &= ~(ARMV4_5_MMU_ENABLED | ARMV4_5_D_U_CACHE_ENABLED | ARMV4_5_I_CACHE_ENABLED);
	arm920t_write_cp15_physical(target, ARM920T_CP15_PHYS_ADDR(0, 0x1, 0), cp15_ctrl);

	/* read CP15 test state register */
	arm920t_read_cp15_physical(target, ARM920T_CP15_PHYS_ADDR(0, 0xf, 0), &cp15c15);
	if((retval = jtag_execute_queue()) != ERROR_OK)
	{
		return retval;
	}

	/* prepare reading D TLB content
	 * */

	/* set interpret mode */
	cp15c15 |= 0x1;
	arm920t_write_cp15_physical(target, ARM920T_CP15_PHYS_ADDR(0, 0xf, 0), cp15c15);

	/* Read D TLB lockdown */
	arm920t_execute_cp15(target, ARMV4_5_MRC(15,0,0,10,0,0), ARMV4_5_LDR(1, 0));

	/* clear interpret mode */
	cp15c15 &= ~0x1;
	arm920t_write_cp15_physical(target, 0x1e, cp15c15);

	/* read D TLB lockdown stored to r1 */
	arm9tdmi_read_core_regs(target, 0x2, regs_p);
	if((retval = jtag_execute_queue()) != ERROR_OK)
	{
		return retval;
	}
	Dlockdown = regs[1];

	for (victim = 0; victim < 64; victim += 8)
	{
		/* new lockdown value: base[31:26]:victim[25:20]:SBZ[19:1]:p[0]
		 * base remains unchanged, victim goes through entries 0 to 63 */
		regs[1] = (Dlockdown & 0xfc000000) | (victim << 20);
		arm9tdmi_write_core_regs(target, 0x2, regs);

		/* set interpret mode */
		cp15c15 |= 0x1;
		arm920t_write_cp15_physical(target, ARM920T_CP15_PHYS_ADDR(0, 0xf, 0), cp15c15);

		/* Write D TLB lockdown */
		arm920t_execute_cp15(target, ARMV4_5_MCR(15,0,0,10,0,0), ARMV4_5_STR(1, 0));

		/* Read D TLB CAM */
		arm920t_execute_cp15(target, ARMV4_5_MCR(15,4,0,15,6,4), ARMV4_5_LDMIA(0, 0x3fc, 0, 0));

		/* clear interpret mode */
		cp15c15 &= ~0x1;
		arm920t_write_cp15_physical(target, 0x1e, cp15c15);

		/* read D TLB CAM content stored to r2-r9 */
		arm9tdmi_read_core_regs(target, 0x3fc, regs_p);
		if((retval = jtag_execute_queue()) != ERROR_OK)
		{
			return retval;
		}

		for (i = 0; i < 8; i++)
			d_tlb[victim + i].cam = regs[i + 2];
	}

	for (victim = 0; victim < 64; victim++)
	{
		/* new lockdown value: base[31:26]:victim[25:20]:SBZ[19:1]:p[0]
		 * base remains unchanged, victim goes through entries 0 to 63 */
		regs[1] = (Dlockdown & 0xfc000000) | (victim << 20);
		arm9tdmi_write_core_regs(target, 0x2, regs);

		/* set interpret mode */
		cp15c15 |= 0x1;
		arm920t_write_cp15_physical(target, ARM920T_CP15_PHYS_ADDR(0, 0xf, 0), cp15c15);

		/* Write D TLB lockdown */
		arm920t_execute_cp15(target, ARMV4_5_MCR(15,0,0,10,0,0), ARMV4_5_STR(1, 0));

		/* Read D TLB RAM1 */
		arm920t_execute_cp15(target, ARMV4_5_MCR(15,4,0,15,10,4), ARMV4_5_LDR(2,0));

		/* Read D TLB RAM2 */
		arm920t_execute_cp15(target, ARMV4_5_MCR(15,4,0,15,2,5), ARMV4_5_LDR(3,0));

		/* clear interpret mode */
		cp15c15 &= ~0x1;
		arm920t_write_cp15_physical(target, 0x1e, cp15c15);

		/* read D TLB RAM content stored to r2 and r3 */
		arm9tdmi_read_core_regs(target, 0xc, regs_p);
		if((retval = jtag_execute_queue()) != ERROR_OK)
		{
			return retval;
		}

		d_tlb[victim].ram1 = regs[2];
		d_tlb[victim].ram2 = regs[3];
	}

	/* restore D TLB lockdown */
	regs[1] = Dlockdown;
	arm9tdmi_write_core_regs(target, 0x2, regs);

	/* Write D TLB lockdown */
	arm920t_execute_cp15(target, ARMV4_5_MCR(15,0,0,10,0,0), ARMV4_5_STR(1, 0));

	/* prepare reading I TLB content
	 * */

	/* set interpret mode */
	cp15c15 |= 0x1;
	arm920t_write_cp15_physical(target, ARM920T_CP15_PHYS_ADDR(0, 0xf, 0), cp15c15);

	/* Read I TLB lockdown */
	arm920t_execute_cp15(target, ARMV4_5_MRC(15,0,0,10,0,1), ARMV4_5_LDR(1, 0));

	/* clear interpret mode */
	cp15c15 &= ~0x1;
	arm920t_write_cp15_physical(target, 0x1e, cp15c15);

	/* read I TLB lockdown stored to r1 */
	arm9tdmi_read_core_regs(target, 0x2, regs_p);
	if((retval = jtag_execute_queue()) != ERROR_OK)
	{
		return retval;
	}
	Ilockdown = regs[1];

	for (victim = 0; victim < 64; victim += 8)
	{
		/* new lockdown value: base[31:26]:victim[25:20]:SBZ[19:1]:p[0]
		 * base remains unchanged, victim goes through entries 0 to 63 */
		regs[1] = (Ilockdown & 0xfc000000) | (victim << 20);
		arm9tdmi_write_core_regs(target, 0x2, regs);

		/* set interpret mode */
		cp15c15 |= 0x1;
		arm920t_write_cp15_physical(target, ARM920T_CP15_PHYS_ADDR(0, 0xf, 0), cp15c15);

		/* Write I TLB lockdown */
		arm920t_execute_cp15(target, ARMV4_5_MCR(15,0,0,10,0,1), ARMV4_5_STR(1, 0));

		/* Read I TLB CAM */
		arm920t_execute_cp15(target, ARMV4_5_MCR(15,4,0,15,5,4), ARMV4_5_LDMIA(0, 0x3fc, 0, 0));

		/* clear interpret mode */
		cp15c15 &= ~0x1;
		arm920t_write_cp15_physical(target, 0x1e, cp15c15);

		/* read I TLB CAM content stored to r2-r9 */
		arm9tdmi_read_core_regs(target, 0x3fc, regs_p);
		if((retval = jtag_execute_queue()) != ERROR_OK)
		{
			return retval;
		}

		for (i = 0; i < 8; i++)
			i_tlb[i + victim].cam = regs[i + 2];
	}

	for (victim = 0; victim < 64; victim++)
	{
		/* new lockdown value: base[31:26]:victim[25:20]:SBZ[19:1]:p[0]
		 * base remains unchanged, victim goes through entries 0 to 63 */
		regs[1] = (Dlockdown & 0xfc000000) | (victim << 20);
		arm9tdmi_write_core_regs(target, 0x2, regs);

		/* set interpret mode */
		cp15c15 |= 0x1;
		arm920t_write_cp15_physical(target, ARM920T_CP15_PHYS_ADDR(0, 0xf, 0), cp15c15);

		/* Write I TLB lockdown */
		arm920t_execute_cp15(target, ARMV4_5_MCR(15,0,0,10,0,1), ARMV4_5_STR(1, 0));

		/* Read I TLB RAM1 */
		arm920t_execute_cp15(target, ARMV4_5_MCR(15,4,0,15,9,4), ARMV4_5_LDR(2,0));

		/* Read I TLB RAM2 */
		arm920t_execute_cp15(target, ARMV4_5_MCR(15,4,0,15,1,5), ARMV4_5_LDR(3,0));

		/* clear interpret mode */
		cp15c15 &= ~0x1;
		arm920t_write_cp15_physical(target, 0x1e, cp15c15);

		/* read I TLB RAM content stored to r2 and r3 */
		arm9tdmi_read_core_regs(target, 0xc, regs_p);
		if((retval = jtag_execute_queue()) != ERROR_OK)
		{
			return retval;
		}

		i_tlb[victim].ram1 = regs[2];
		i_tlb[victim].ram2 = regs[3];
	}

	/* restore I TLB lockdown */
	regs[1] = Ilockdown;
	arm9tdmi_write_core_regs(target, 0x2, regs);

	/* Write I TLB lockdown */
	arm920t_execute_cp15(target, ARMV4_5_MCR(15,0,0,10,0,1), ARMV4_5_STR(1, 0));

	/* restore CP15 MMU and Cache settings */
	arm920t_write_cp15_physical(target, ARM920T_CP15_PHYS_ADDR(0, 0x1, 0), cp15_ctrl_saved);

	/* output data to file */
	fprintf(output, "D TLB content:\n");
	for (i = 0; i < 64; i++)
	{
		fprintf(output, "%i: 0x%8.8x 0x%8.8x 0x%8.8x %s\n", i, d_tlb[i].cam, d_tlb[i].ram1, d_tlb[i].ram2, (d_tlb[i].cam & 0x20) ? "(valid)" : "(invalid)");
	}

	fprintf(output, "\n\nI TLB content:\n");
	for (i = 0; i < 64; i++)
	{
		fprintf(output, "%i: 0x%8.8x 0x%8.8x 0x%8.8x %s\n", i, i_tlb[i].cam, i_tlb[i].ram1, i_tlb[i].ram2, (i_tlb[i].cam & 0x20) ? "(valid)" : "(invalid)");
	}

	command_print(cmd_ctx, "mmu content successfully output to %s", args[0]);

	fclose(output);

	if (armv4_5_mode_to_number(armv4_5->core_mode)==-1)
		return ERROR_FAIL;

	/* mark registers dirty */
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 0).dirty = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 0).valid;
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 1).dirty = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 1).valid;
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 2).dirty = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 2).valid;
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 3).dirty = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 3).valid;
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 4).dirty = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 4).valid;
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 5).dirty = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 5).valid;
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 6).dirty = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 6).valid;
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 7).dirty = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 7).valid;
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 8).dirty = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 8).valid;
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 9).dirty = ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 9).valid;

	return ERROR_OK;
}
int arm920t_handle_cp15_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int retval;
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	arm9tdmi_common_t *arm9tdmi;
	arm920t_common_t *arm920t;
	arm_jtag_t *jtag_info;

	if (arm920t_get_arch_pointers(target, &armv4_5, &arm7_9, &arm9tdmi, &arm920t) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM920t target");
		return ERROR_OK;
	}

	jtag_info = &arm7_9->jtag_info;

	if (target->state != TARGET_HALTED)
	{
		command_print(cmd_ctx, "target must be stopped for \"%s\" command", cmd);
		return ERROR_OK;
	}

	/* one or more argument, access a single register (write if second argument is given */
	if (argc >= 1)
	{
		int address = strtoul(args[0], NULL, 0);

		if (argc == 1)
		{
			u32 value;
			if ((retval = arm920t_read_cp15_physical(target, address, &value)) != ERROR_OK)
			{
				command_print(cmd_ctx, "couldn't access reg %i", address);
				return ERROR_OK;
			}
			if((retval = jtag_execute_queue()) != ERROR_OK)
			{
				return retval;
			}

			command_print(cmd_ctx, "%i: %8.8x", address, value);
		}
		else if (argc == 2)
		{
			u32 value = strtoul(args[1], NULL, 0);
			if ((retval = arm920t_write_cp15_physical(target, address, value)) != ERROR_OK)
			{
				command_print(cmd_ctx, "couldn't access reg %i", address);
				return ERROR_OK;
			}
			command_print(cmd_ctx, "%i: %8.8x", address, value);
		}
	}

	return ERROR_OK;
}

int arm920t_handle_cp15i_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int retval;
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	arm9tdmi_common_t *arm9tdmi;
	arm920t_common_t *arm920t;
	arm_jtag_t *jtag_info;

	if (arm920t_get_arch_pointers(target, &armv4_5, &arm7_9, &arm9tdmi, &arm920t) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM920t target");
		return ERROR_OK;
	}

	jtag_info = &arm7_9->jtag_info;

	if (target->state != TARGET_HALTED)
	{
		command_print(cmd_ctx, "target must be stopped for \"%s\" command", cmd);
		return ERROR_OK;
	}

	/* one or more argument, access a single register (write if second argument is given */
	if (argc >= 1)
	{
		u32 opcode = strtoul(args[0], NULL, 0);

		if (argc == 1)
		{
			u32 value;
			if ((retval = arm920t_read_cp15_interpreted(target, opcode, 0x0, &value)) != ERROR_OK)
			{
				command_print(cmd_ctx, "couldn't execute %8.8x", opcode);
				return ERROR_OK;
			}

			command_print(cmd_ctx, "%8.8x: %8.8x", opcode, value);
		}
		else if (argc == 2)
		{
			u32 value = strtoul(args[1], NULL, 0);
			if ((retval = arm920t_write_cp15_interpreted(target, opcode, value, 0)) != ERROR_OK)
			{
				command_print(cmd_ctx, "couldn't execute %8.8x", opcode);
				return ERROR_OK;
			}
			command_print(cmd_ctx, "%8.8x: %8.8x", opcode, value);
		}
		else if (argc == 3)
		{
			u32 value = strtoul(args[1], NULL, 0);
			u32 address = strtoul(args[2], NULL, 0);
			if ((retval = arm920t_write_cp15_interpreted(target, opcode, value, address)) != ERROR_OK)
			{
				command_print(cmd_ctx, "couldn't execute %8.8x", opcode);
				return ERROR_OK;
			}
			command_print(cmd_ctx, "%8.8x: %8.8x %8.8x", opcode, value, address);
		}
	}
	else
	{
		command_print(cmd_ctx, "usage: arm920t cp15i <opcode> [value] [address]");
	}

	return ERROR_OK;
}

int arm920t_handle_cache_info_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	arm9tdmi_common_t *arm9tdmi;
	arm920t_common_t *arm920t;

	if (arm920t_get_arch_pointers(target, &armv4_5, &arm7_9, &arm9tdmi, &arm920t) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM920t target");
		return ERROR_OK;
	}

	return armv4_5_handle_cache_info_command(cmd_ctx, &arm920t->armv4_5_mmu.armv4_5_cache);
}

int arm920t_handle_virt2phys_command(command_context_t *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	arm9tdmi_common_t *arm9tdmi;
	arm920t_common_t *arm920t;
	arm_jtag_t *jtag_info;

	if (arm920t_get_arch_pointers(target, &armv4_5, &arm7_9, &arm9tdmi, &arm920t) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM920t target");
		return ERROR_OK;
	}

	jtag_info = &arm7_9->jtag_info;

	if (target->state != TARGET_HALTED)
	{
		command_print(cmd_ctx, "target must be stopped for \"%s\" command", cmd);
		return ERROR_OK;
	}

	return armv4_5_mmu_handle_virt2phys_command(cmd_ctx, cmd, args, argc, target, &arm920t->armv4_5_mmu);
}

int arm920t_handle_md_phys_command(command_context_t *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	arm9tdmi_common_t *arm9tdmi;
	arm920t_common_t *arm920t;
	arm_jtag_t *jtag_info;

	if (arm920t_get_arch_pointers(target, &armv4_5, &arm7_9, &arm9tdmi, &arm920t) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM920t target");
		return ERROR_OK;
	}

	jtag_info = &arm7_9->jtag_info;

	if (target->state != TARGET_HALTED)
	{
		command_print(cmd_ctx, "target must be stopped for \"%s\" command", cmd);
		return ERROR_OK;
	}

	return armv4_5_mmu_handle_md_phys_command(cmd_ctx, cmd, args, argc, target, &arm920t->armv4_5_mmu);
}

int arm920t_handle_mw_phys_command(command_context_t *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	arm9tdmi_common_t *arm9tdmi;
	arm920t_common_t *arm920t;
	arm_jtag_t *jtag_info;

	if (arm920t_get_arch_pointers(target, &armv4_5, &arm7_9, &arm9tdmi, &arm920t) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM920t target");
		return ERROR_OK;
	}

	jtag_info = &arm7_9->jtag_info;

	if (target->state != TARGET_HALTED)
	{
		command_print(cmd_ctx, "target must be stopped for \"%s\" command", cmd);
		return ERROR_OK;
	}

	return armv4_5_mmu_handle_mw_phys_command(cmd_ctx, cmd, args, argc, target, &arm920t->armv4_5_mmu);
}
