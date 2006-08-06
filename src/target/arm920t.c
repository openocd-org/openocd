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

/* forward declarations */
int arm920t_target_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct target_s *target);
int arm920t_init_target(struct command_context_s *cmd_ctx, struct target_s *target);
int arm920t_quit();
int arm920t_arch_state(struct target_s *target, char *buf, int buf_size);
int arm920t_read_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);
int arm920t_write_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);
int arm920t_soft_reset_halt(struct target_s *target);

target_type_t arm920t_target =
{
	.name = "arm920t",

	.poll = arm7_9_poll,
	.arch_state = arm920t_arch_state,

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

	.run_algorithm = armv4_5_run_algorithm,

	.add_breakpoint = arm7_9_add_breakpoint,
	.remove_breakpoint = arm7_9_remove_breakpoint,
	.add_watchpoint = arm7_9_add_watchpoint,
	.remove_watchpoint = arm7_9_remove_watchpoint,

	.register_commands = arm920t_register_commands,
	.target_command = arm920t_target_command,
	.init_target = arm920t_init_target,
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
	
	jtag_add_end_state(TAP_RTI);
	arm_jtag_scann(jtag_info, 0xf);
	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr);

	fields[0].device = jtag_info->chain_pos;
	fields[0].num_bits = 1;
	fields[0].out_value = &access_type_buf;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;

	fields[1].device = jtag_info->chain_pos;
	fields[1].num_bits = 32;
	fields[1].out_value = NULL;
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;

	fields[2].device = jtag_info->chain_pos;
	fields[2].num_bits = 6;
	fields[2].out_value = &reg_addr_buf;
	fields[2].out_mask = NULL;
	fields[2].in_value = NULL;
	fields[2].in_check_value = NULL;
	fields[2].in_check_mask = NULL;
	fields[2].in_handler = NULL;
	fields[2].in_handler_priv = NULL;

	fields[3].device = jtag_info->chain_pos;
	fields[3].num_bits = 1;
	fields[3].out_value = &nr_w_buf;
	fields[3].out_mask = NULL;
	fields[3].in_value = NULL;
	fields[3].in_check_value = NULL;
	fields[3].in_check_mask = NULL;
	fields[3].in_handler = NULL;
	fields[3].in_handler_priv = NULL;
	
	jtag_add_dr_scan(4, fields, -1);

	fields[1].in_value = (u8*)value;

	jtag_add_dr_scan(4, fields, -1);

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
	
	jtag_add_end_state(TAP_RTI);
	arm_jtag_scann(jtag_info, 0xf);
	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr);

	fields[0].device = jtag_info->chain_pos;
	fields[0].num_bits = 1;
	fields[0].out_value = &access_type_buf;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;

	fields[1].device = jtag_info->chain_pos;
	fields[1].num_bits = 32;
	fields[1].out_value = (u8*)&value;
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;

	fields[2].device = jtag_info->chain_pos;
	fields[2].num_bits = 6;
	fields[2].out_value = &reg_addr_buf;
	fields[2].out_mask = NULL;
	fields[2].in_value = NULL;
	fields[2].in_check_value = NULL;
	fields[2].in_check_mask = NULL;
	fields[2].in_handler = NULL;
	fields[2].in_handler_priv = NULL;

	fields[3].device = jtag_info->chain_pos;
	fields[3].num_bits = 1;
	fields[3].out_value = &nr_w_buf;
	fields[3].out_mask = NULL;
	fields[3].in_value = NULL;
	fields[3].in_check_value = NULL;
	fields[3].in_check_mask = NULL;
	fields[3].in_handler = NULL;
	fields[3].in_handler_priv = NULL;
	
	jtag_add_dr_scan(4, fields, -1);

	return ERROR_OK;
}

int arm920t_read_cp15_interpreted(target_t *target, u32 opcode, u32 *value)
{
	u32 cp15c15 = 0x0;
	scan_field_t fields[4];
	u8 access_type_buf = 0;		/* interpreted access */
	u8 reg_addr_buf = 0x0;
	u8 nr_w_buf = 0;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	u32* context_p[1];	
	
	/* read-modify-write CP15 test state register 
	* to enable interpreted access mode */
	arm920t_read_cp15_physical(target, 0x1e, &cp15c15);	
	jtag_execute_queue();
	cp15c15 |= 1;	/* set interpret mode */
	arm920t_write_cp15_physical(target, 0x1e, cp15c15);

	jtag_add_end_state(TAP_RTI);
	arm_jtag_scann(jtag_info, 0xf);
	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr);

	fields[0].device = jtag_info->chain_pos;
	fields[0].num_bits = 1;
	fields[0].out_value = &access_type_buf;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;

	fields[1].device = jtag_info->chain_pos;
	fields[1].num_bits = 32;
	fields[1].out_value = (u8*)&opcode;
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;

	fields[2].device = jtag_info->chain_pos;
	fields[2].num_bits = 6;
	fields[2].out_value = &reg_addr_buf;
	fields[2].out_mask = NULL;
	fields[2].in_value = NULL;
	fields[2].in_check_value = NULL;
	fields[2].in_check_mask = NULL;
	fields[2].in_handler = NULL;
	fields[2].in_handler_priv = NULL;

	fields[3].device = jtag_info->chain_pos;
	fields[3].num_bits = 1;
	fields[3].out_value = &nr_w_buf;
	fields[3].out_mask = NULL;
	fields[3].in_value = NULL;
	fields[3].in_check_value = NULL;
	fields[3].in_check_mask = NULL;
	fields[3].in_handler = NULL;
	fields[3].in_handler_priv = NULL;

	jtag_add_dr_scan(4, fields, -1);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_LDR(0, 15), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 1);
	arm7_9_execute_sys_speed(target);
	jtag_execute_queue();

	/* read-modify-write CP15 test state register 
	* to disable interpreted access mode */
	arm920t_read_cp15_physical(target, 0x1e, &cp15c15);	
	jtag_execute_queue();
	cp15c15 &= ~1U;	/* clear interpret mode */
	arm920t_write_cp15_physical(target, 0x1e, cp15c15);

	context_p[0] = value;
	arm9tdmi_read_core_regs(target, 0x1, context_p);
	jtag_execute_queue();
	
	DEBUG("opcode: %8.8x, value: %8.8x", opcode, *value);

	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 0).dirty = 1;
	ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 15).dirty = 1;

	return ERROR_OK;
}

int arm920t_write_cp15_interpreted(target_t *target, u32 opcode, u32 value, u32 address)
{
	u32 cp15c15 = 0x0;
	scan_field_t fields[4];
	u8 access_type_buf = 0;		/* interpreted access */
	u8 reg_addr_buf = 0x0;
	u8 nr_w_buf = 0;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	u32 regs[2];

	regs[0] = value;
	regs[1] = address;
	
	arm9tdmi_write_core_regs(target, 0x3, regs);

	/* read-modify-write CP15 test state register 
	* to enable interpreted access mode */
	arm920t_read_cp15_physical(target, 0x1e, &cp15c15);
	jtag_execute_queue();
	cp15c15 |= 1;	/* set interpret mode */
	arm920t_write_cp15_physical(target, 0x1e, cp15c15);

	jtag_add_end_state(TAP_RTI);
	arm_jtag_scann(jtag_info, 0xf);
	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr);

	fields[0].device = jtag_info->chain_pos;
	fields[0].num_bits = 1;
	fields[0].out_value = &access_type_buf;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;

	fields[1].device = jtag_info->chain_pos;
	fields[1].num_bits = 32;
	fields[1].out_value = (u8*)&opcode;
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;

	fields[2].device = jtag_info->chain_pos;
	fields[2].num_bits = 6;
	fields[2].out_value = &reg_addr_buf;
	fields[2].out_mask = NULL;
	fields[2].in_value = NULL;
	fields[2].in_check_value = NULL;
	fields[2].in_check_mask = NULL;
	fields[2].in_handler = NULL;
	fields[2].in_handler_priv = NULL;

	fields[3].device = jtag_info->chain_pos;
	fields[3].num_bits = 1;
	fields[3].out_value = &nr_w_buf;
	fields[3].out_mask = NULL;
	fields[3].in_value = NULL;
	fields[3].in_check_value = NULL;
	fields[3].in_check_mask = NULL;
	fields[3].in_handler = NULL;
	fields[3].in_handler_priv = NULL;

	jtag_add_dr_scan(4, fields, -1);

	arm9tdmi_clock_out(jtag_info, ARMV4_5_STR(0, 1), 0, NULL, 0);
	arm9tdmi_clock_out(jtag_info, ARMV4_5_NOP, 0, NULL, 1);
	arm7_9_execute_sys_speed(target);
	jtag_execute_queue();

	/* read-modify-write CP15 test state register 
	* to disable interpreted access mode */
	arm920t_read_cp15_physical(target, 0x1e, &cp15c15);	
	jtag_execute_queue();
	cp15c15 &= ~1U;	/* set interpret mode */
	arm920t_write_cp15_physical(target, 0x1e, cp15c15);

	DEBUG("opcode: %8.8x, value: %8.8x, address: %8.8x", opcode, value, address);

	return ERROR_OK;
}

u32 arm920t_get_ttb(target_t *target)
{
	int retval;
	u32 ttb = 0x0;

	if ((retval = arm920t_read_cp15_interpreted(target, 0xeebf0f51, &ttb)) != ERROR_OK)
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
	DEBUG("cp15_control_reg: %8.8x", arm920t->cp15_control_reg);

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
	arm920t_read_cp15_interpreted(target, 0xee150f10, &arm920t->d_fsr);
	arm920t_read_cp15_interpreted(target, 0xee150f30, &arm920t->i_fsr);
	arm920t_read_cp15_interpreted(target, 0xee160f10, &arm920t->d_far);
	arm920t_read_cp15_interpreted(target, 0xee160f30, &arm920t->i_far);

	/* read-modify-write CP15 test state register 
	* to disable I/D-cache linefills */
	arm920t_read_cp15_physical(target, 0x1e, &cp15c15);	
	jtag_execute_queue();
	cp15c15 |= 0x600;
	arm920t_write_cp15_physical(target, 0x1e, cp15c15);

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
	arm920t_read_cp15_physical(target, 0x1e, &cp15c15);
	jtag_execute_queue();
	cp15c15 &= ~0x600U;
	arm920t_write_cp15_physical(target, 0x1e, cp15c15);

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

int arm920t_arch_state(struct target_s *target, char *buf, int buf_size)
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
		ERROR("BUG: called for a non-ARMv4/5 target");
		exit(-1);
	}
	
	snprintf(buf, buf_size,
			"target halted in %s state due to %s, current mode: %s\n"
			"cpsr: 0x%8.8x pc: 0x%8.8x\n"
			"MMU: %s, D-Cache: %s, I-Cache: %s",
			 armv4_5_state_strings[armv4_5->core_state],
			 target_debug_reason_strings[target->debug_reason],
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
			DEBUG("D-Cache enabled, writing through to main memory");
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
			DEBUG("I-Cache enabled, invalidating affected I-Cache line");
			arm920t_write_cp15_interpreted(target, 0xee070f35, 0x0, address);
		}
	}

	return retval;
}

int arm920t_soft_reset_halt(struct target_s *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm9tdmi_common_t *arm9tdmi = arm7_9->arch_info;
	arm920t_common_t *arm920t = arm9tdmi->arch_info;
	reg_t *dbg_stat = &arm7_9->eice_cache->reg_list[EICE_DBG_STAT];
	
	if (target->state == TARGET_RUNNING)
	{
		target->type->halt(target);
	}
	
	while (buf_get_u32(dbg_stat->value, EICE_DBG_CONTROL_DBGACK, 1) == 0)
	{
		embeddedice_read_reg(dbg_stat);
		jtag_execute_queue();
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

	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	
	return ERROR_OK;
}

int arm920t_init_target(struct command_context_s *cmd_ctx, struct target_s *target)
{
	arm9tdmi_init_target(cmd_ctx, target);
		
	return ERROR_OK;
	
}

int arm920t_quit()
{
	
	return ERROR_OK;
}

int arm920t_init_arch_info(target_t *target, arm920t_common_t *arm920t, int chain_pos, char *variant)
{
	arm9tdmi_common_t *arm9tdmi = &arm920t->arm9tdmi_common;
	arm7_9_common_t *arm7_9 = &arm9tdmi->arm7_9_common;
	
	arm9tdmi_init_arch_info(target, arm9tdmi, chain_pos, variant);

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
	
	arm9tdmi->has_single_step = 1;
	
	return ERROR_OK;
}

int arm920t_target_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct target_s *target)
{
	int chain_pos;
	char *variant = NULL;
	arm920t_common_t *arm920t = malloc(sizeof(arm920t_common_t));
	
	if (argc < 4)
	{
		ERROR("'target arm920t' requires at least one additional argument");
		exit(-1);
	}
	
	chain_pos = strtoul(args[3], NULL, 0);
	
	if (argc >= 5)
		variant = args[4];
	
	DEBUG("chain_pos: %i, variant: %s", chain_pos, variant);
	
	arm920t_init_arch_info(target, arm920t, chain_pos, variant);

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
			jtag_execute_queue();
			
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
			if ((retval = arm920t_read_cp15_interpreted(target, opcode, &value)) != ERROR_OK)
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
