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

#include "arm720t.h"
#include "jtag.h"
#include "log.h"

#include <stdlib.h>
#include <string.h>

#if 0
#define _DEBUG_INSTRUCTION_EXECUTION_
#endif

/* cli handling */
int arm720t_register_commands(struct command_context_s *cmd_ctx);

int arm720t_handle_cp15_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int arm720t_handle_virt2phys_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int arm720t_handle_md_phys_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int arm720t_handle_mw_phys_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

/* forward declarations */
int arm720t_target_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct target_s *target);
int arm720t_init_target(struct command_context_s *cmd_ctx, struct target_s *target);
int arm720t_quit();
int arm720t_arch_state(struct target_s *target, char *buf, int buf_size);
int arm720t_read_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);
int arm720t_write_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);
int arm720t_soft_reset_halt(struct target_s *target);

target_type_t arm720t_target =
{
	.name = "arm720t",

	.poll = arm7_9_poll,
	.arch_state = arm720t_arch_state,

	.halt = arm7_9_halt,
	.resume = arm7_9_resume,
	.step = arm7_9_step,

	.assert_reset = arm7_9_assert_reset,
	.deassert_reset = arm7_9_deassert_reset,
	.soft_reset_halt = arm720t_soft_reset_halt,
	.prepare_reset_halt = arm7_9_prepare_reset_halt,
	
	.get_gdb_reg_list = armv4_5_get_gdb_reg_list,

	.read_memory = arm720t_read_memory,
	.write_memory = arm720t_write_memory,
	.bulk_write_memory = arm7_9_bulk_write_memory,
	.checksum_memory = arm7_9_checksum_memory,
	
	.run_algorithm = armv4_5_run_algorithm,

	.add_breakpoint = arm7_9_add_breakpoint,
	.remove_breakpoint = arm7_9_remove_breakpoint,
	.add_watchpoint = arm7_9_add_watchpoint,
	.remove_watchpoint = arm7_9_remove_watchpoint,

	.register_commands = arm720t_register_commands,
	.target_command = arm720t_target_command,
	.init_target = arm720t_init_target,
	.quit = arm720t_quit
};

int arm720t_scan_cp15(target_t *target, u32 out, u32 *in, int instruction, int clock)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	scan_field_t fields[2];
	u8 out_buf[4];
	u8 instruction_buf = instruction;
	
	buf_set_u32(out_buf, 0, 32, flip_u32(out, 32));
	
	jtag_add_end_state(TAP_PD);
	arm_jtag_scann(jtag_info, 0xf);
	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr, NULL);
		
	fields[0].device = jtag_info->chain_pos;
	fields[0].num_bits = 1;
	fields[0].out_value = &instruction_buf;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;

	fields[1].device = jtag_info->chain_pos;
	fields[1].num_bits = 32;
	fields[1].out_value = out_buf;
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	if (in)
	{
		fields[1].in_handler = arm_jtag_buf_to_u32_flip;
		fields[1].in_handler_priv = in;
	} else
	{
		fields[1].in_handler = NULL;
		fields[1].in_handler_priv = NULL;
	}
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	
	jtag_add_dr_scan(2, fields, -1, NULL);

	if (clock)
		jtag_add_runtest(0, -1);

#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	jtag_execute_queue();

	if (in)
		DEBUG("out: %8.8x, in: %8.8x, instruction: %i, clock: %i", out, *in, instruction, clock);
	else
		DEBUG("out: %8.8x, instruction: %i, clock: %i", out, instruction, clock);
#else
		DEBUG("out: %8.8x, instruction: %i, clock: %i", out, instruction, clock);
#endif

	return ERROR_OK;
}

int arm720t_read_cp15(target_t *target, u32 opcode, u32 *value)
{
	/* fetch CP15 opcode */
	arm720t_scan_cp15(target, opcode, NULL, 1, 1);
	/* "DECODE" stage */
	arm720t_scan_cp15(target, ARMV4_5_NOP, NULL, 1, 1);
	/* "EXECUTE" stage (1) */
	arm720t_scan_cp15(target, ARMV4_5_NOP, NULL, 1, 0);
	arm720t_scan_cp15(target, 0x0, NULL, 0, 1);
	/* "EXECUTE" stage (2) */
	arm720t_scan_cp15(target, 0x0, NULL, 0, 1);
	/* "EXECUTE" stage (3), CDATA is read */
	arm720t_scan_cp15(target, ARMV4_5_NOP, value, 1, 1);
	
	return ERROR_OK;
}

int arm720t_write_cp15(target_t *target, u32 opcode, u32 value)
{
	/* fetch CP15 opcode */
	arm720t_scan_cp15(target, opcode, NULL, 1, 1);
	/* "DECODE" stage */
	arm720t_scan_cp15(target, ARMV4_5_NOP, NULL, 1, 1);
	/* "EXECUTE" stage (1) */
	arm720t_scan_cp15(target, ARMV4_5_NOP, NULL, 1, 0);
	arm720t_scan_cp15(target, 0x0, NULL, 0, 1);
	/* "EXECUTE" stage (2) */
	arm720t_scan_cp15(target, value, NULL, 0, 1);
	arm720t_scan_cp15(target, ARMV4_5_NOP, NULL, 1, 1);

	return ERROR_OK;
}

u32 arm720t_get_ttb(target_t *target)
{
	u32 ttb = 0x0;

	arm720t_read_cp15(target, 0xee120f10, &ttb);
	jtag_execute_queue();
	
	ttb &= 0xffffc000;
	
	return ttb;
}

void arm720t_disable_mmu_caches(target_t *target, int mmu, int d_u_cache, int i_cache)
{
	u32 cp15_control;

	/* read cp15 control register */
	arm720t_read_cp15(target, 0xee110f10, &cp15_control);
	jtag_execute_queue();
		
	if (mmu)
		cp15_control &= ~0x1U;
	
	if (d_u_cache || i_cache)
		cp15_control &= ~0x4U;

	arm720t_write_cp15(target, 0xee010f10, cp15_control);
}

void arm720t_enable_mmu_caches(target_t *target, int mmu, int d_u_cache, int i_cache)
{
	u32 cp15_control;

	/* read cp15 control register */
	arm720t_read_cp15(target, 0xee110f10, &cp15_control);
	jtag_execute_queue();
		
	if (mmu)
		cp15_control |= 0x1U;
	
	if (d_u_cache || i_cache)
		cp15_control |= 0x4U;
	
	arm720t_write_cp15(target, 0xee010f10, cp15_control);
}

void arm720t_post_debug_entry(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm7tdmi_common_t *arm7tdmi = arm7_9->arch_info;
	arm720t_common_t *arm720t = arm7tdmi->arch_info;
	
	/* examine cp15 control reg */
	arm720t_read_cp15(target, 0xee110f10, &arm720t->cp15_control_reg);
	jtag_execute_queue();
	DEBUG("cp15_control_reg: %8.8x", arm720t->cp15_control_reg);

	arm720t->armv4_5_mmu.mmu_enabled = (arm720t->cp15_control_reg & 0x1U) ? 1 : 0;
	arm720t->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled = (arm720t->cp15_control_reg & 0x4U) ? 1 : 0;
	arm720t->armv4_5_mmu.armv4_5_cache.i_cache_enabled = 0;

	/* save i/d fault status and address register */
	arm720t_read_cp15(target, 0xee150f10, &arm720t->fsr_reg);
	arm720t_read_cp15(target, 0xee160f10, &arm720t->far_reg);
	jtag_execute_queue();
}

void arm720t_pre_restore_context(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm7tdmi_common_t *arm7tdmi = arm7_9->arch_info;
	arm720t_common_t *arm720t = arm7tdmi->arch_info;
	
	/* restore i/d fault status and address register */
	arm720t_write_cp15(target, 0xee050f10, arm720t->fsr_reg);
	arm720t_write_cp15(target, 0xee060f10, arm720t->far_reg);
}

int arm720t_get_arch_pointers(target_t *target, armv4_5_common_t **armv4_5_p, arm7_9_common_t **arm7_9_p, arm7tdmi_common_t **arm7tdmi_p, arm720t_common_t **arm720t_p)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9;
	arm7tdmi_common_t *arm7tdmi;
	arm720t_common_t *arm720t;
	
	if (armv4_5->common_magic != ARMV4_5_COMMON_MAGIC)
	{
		return -1;
	}
	
	arm7_9 = armv4_5->arch_info;
	if (arm7_9->common_magic != ARM7_9_COMMON_MAGIC)
	{
		return -1;
	}
	
	arm7tdmi = arm7_9->arch_info;
	if (arm7tdmi->common_magic != ARM7TDMI_COMMON_MAGIC)
	{
		return -1;
	}
	
	arm720t = arm7tdmi->arch_info;
	if (arm720t->common_magic != ARM720T_COMMON_MAGIC)
	{
		return -1;
	}
	
	*armv4_5_p = armv4_5;
	*arm7_9_p = arm7_9;
	*arm7tdmi_p = arm7tdmi;
	*arm720t_p = arm720t;
	
	return ERROR_OK;
}

int arm720t_arch_state(struct target_s *target, char *buf, int buf_size)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm7tdmi_common_t *arm7tdmi = arm7_9->arch_info;
	arm720t_common_t *arm720t = arm7tdmi->arch_info;
	
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
			"MMU: %s, Cache: %s",
			 armv4_5_state_strings[armv4_5->core_state],
			 target_debug_reason_strings[target->debug_reason],
			 armv4_5_mode_strings[armv4_5_mode_to_number(armv4_5->core_mode)],
			 buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 32),
			 buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32),
			 state[arm720t->armv4_5_mmu.mmu_enabled],
			 state[arm720t->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled]);
	
	return ERROR_OK;
}

int arm720t_read_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer)
{
	int retval;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm7tdmi_common_t *arm7tdmi = arm7_9->arch_info;
	arm720t_common_t *arm720t = arm7tdmi->arch_info;
	
	/* disable cache, but leave MMU enabled */
	if (arm720t->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled)
		arm720t_disable_mmu_caches(target, 0, 1, 0);
	
	retval = arm7_9_read_memory(target, address, size, count, buffer);
	
	if (arm720t->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled)
		arm720t_enable_mmu_caches(target, 0, 1, 0);
	
	return retval;
}

int arm720t_write_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer)
{
	int retval;
	
	if ((retval = arm7_9_write_memory(target, address, size, count, buffer)) != ERROR_OK)
		return retval;

	return retval;
}

int arm720t_soft_reset_halt(struct target_s *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm7tdmi_common_t *arm7tdmi = arm7_9->arch_info;
	arm720t_common_t *arm720t = arm7tdmi->arch_info;
	reg_t *dbg_stat = &arm7_9->eice_cache->reg_list[EICE_DBG_STAT];
	
	if (target->state == TARGET_RUNNING)
	{
		target->type->halt(target);
	}
	
	while (buf_get_u32(dbg_stat->value, EICE_DBG_STATUS_DBGACK, 1) == 0)
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
	
	arm720t_disable_mmu_caches(target, 1, 1, 1);
	arm720t->armv4_5_mmu.mmu_enabled = 0;
	arm720t->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled = 0;
	arm720t->armv4_5_mmu.armv4_5_cache.i_cache_enabled = 0;

	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	
	return ERROR_OK;
}

int arm720t_init_target(struct command_context_s *cmd_ctx, struct target_s *target)
{
	arm7tdmi_init_target(cmd_ctx, target);
		
	return ERROR_OK;
	
}

int arm720t_quit()
{
	
	return ERROR_OK;
}

int arm720t_init_arch_info(target_t *target, arm720t_common_t *arm720t, int chain_pos, char *variant)
{
	arm7tdmi_common_t *arm7tdmi = &arm720t->arm7tdmi_common;
	arm7_9_common_t *arm7_9 = &arm7tdmi->arm7_9_common;
	
	arm7tdmi_init_arch_info(target, arm7tdmi, chain_pos, variant);

	arm7tdmi->arch_info = arm720t;
	arm720t->common_magic = ARM720T_COMMON_MAGIC;
	
	arm7_9->post_debug_entry = arm720t_post_debug_entry;
	arm7_9->pre_restore_context = arm720t_pre_restore_context;
	
	arm720t->armv4_5_mmu.armv4_5_cache.ctype = -1;
	arm720t->armv4_5_mmu.get_ttb = arm720t_get_ttb;
	arm720t->armv4_5_mmu.read_memory = arm7_9_read_memory;
	arm720t->armv4_5_mmu.write_memory = arm7_9_write_memory;
	arm720t->armv4_5_mmu.disable_mmu_caches = arm720t_disable_mmu_caches;
	arm720t->armv4_5_mmu.enable_mmu_caches = arm720t_enable_mmu_caches;
	arm720t->armv4_5_mmu.has_tiny_pages = 0;
	arm720t->armv4_5_mmu.mmu_enabled = 0;
	
	return ERROR_OK;
}

int arm720t_target_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct target_s *target)
{
	int chain_pos;
	char *variant = NULL;
	arm720t_common_t *arm720t = malloc(sizeof(arm720t_common_t));
	
	if (argc < 4)
	{
		ERROR("'target arm720t' requires at least one additional argument");
		exit(-1);
	}
	
	chain_pos = strtoul(args[3], NULL, 0);
	
	if (argc >= 5)
		variant = args[4];
	
	DEBUG("chain_pos: %i, variant: %s", chain_pos, variant);
	
	arm720t_init_arch_info(target, arm720t, chain_pos, variant);

	return ERROR_OK;
}

int arm720t_register_commands(struct command_context_s *cmd_ctx)
{
	int retval;
	command_t *arm720t_cmd;
	
		
	retval = arm7tdmi_register_commands(cmd_ctx);
	
	arm720t_cmd = register_command(cmd_ctx, NULL, "arm720t", NULL, COMMAND_ANY, "arm720t specific commands");

	register_command(cmd_ctx, arm720t_cmd, "cp15", arm720t_handle_cp15_command, COMMAND_EXEC, "display/modify cp15 register <opcode> [value]");
	register_command(cmd_ctx, arm720t_cmd, "virt2phys", arm720t_handle_virt2phys_command, COMMAND_EXEC, "translate va to pa <va>");

	register_command(cmd_ctx, arm720t_cmd, "mdw_phys", arm720t_handle_md_phys_command, COMMAND_EXEC, "display memory words <physical addr> [count]");
	register_command(cmd_ctx, arm720t_cmd, "mdh_phys", arm720t_handle_md_phys_command, COMMAND_EXEC, "display memory half-words <physical addr> [count]");
	register_command(cmd_ctx, arm720t_cmd, "mdb_phys", arm720t_handle_md_phys_command, COMMAND_EXEC, "display memory bytes <physical addr> [count]");

	register_command(cmd_ctx, arm720t_cmd, "mww_phys", arm720t_handle_mw_phys_command, COMMAND_EXEC, "write memory word <physical addr> <value>");
	register_command(cmd_ctx, arm720t_cmd, "mwh_phys", arm720t_handle_mw_phys_command, COMMAND_EXEC, "write memory half-word <physical addr> <value>");
	register_command(cmd_ctx, arm720t_cmd, "mwb_phys", arm720t_handle_mw_phys_command, COMMAND_EXEC, "write memory byte <physical addr> <value>");
	
	return ERROR_OK;
}

int arm720t_handle_cp15_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int retval;
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	arm7tdmi_common_t *arm7tdmi;
	arm720t_common_t *arm720t;
	arm_jtag_t *jtag_info;

	if (arm720t_get_arch_pointers(target, &armv4_5, &arm7_9, &arm7tdmi, &arm720t) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM720t target");
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
			if ((retval = arm720t_read_cp15(target, opcode, &value)) != ERROR_OK)
			{
				command_print(cmd_ctx, "couldn't access cp15 with opcode 0x%8.8x", opcode);
				return ERROR_OK;
			}
			jtag_execute_queue();
			
			command_print(cmd_ctx, "0x%8.8x: 0x%8.8x", opcode, value);
		}
		else if (argc == 2)
		{
			u32 value = strtoul(args[1], NULL, 0);
			if ((retval = arm720t_write_cp15(target, opcode, value)) != ERROR_OK)
			{
				command_print(cmd_ctx, "couldn't access cp15 with opcode 0x%8.8x", opcode);
				return ERROR_OK;
			}
			command_print(cmd_ctx, "0x%8.8x: 0x%8.8x", opcode, value);
		}
	}

	return ERROR_OK;
}

int arm720t_handle_virt2phys_command(command_context_t *cmd_ctx, char *cmd, char **args, int argc)
{	
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	arm7tdmi_common_t *arm7tdmi;
	arm720t_common_t *arm720t;
	arm_jtag_t *jtag_info;

	if (arm720t_get_arch_pointers(target, &armv4_5, &arm7_9, &arm7tdmi, &arm720t) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM720t target");
		return ERROR_OK;
	}
	
	jtag_info = &arm7_9->jtag_info;
	
	if (target->state != TARGET_HALTED)
	{
		command_print(cmd_ctx, "target must be stopped for \"%s\" command", cmd);
		return ERROR_OK;
	}
		
	return armv4_5_mmu_handle_virt2phys_command(cmd_ctx, cmd, args, argc, target, &arm720t->armv4_5_mmu);
}

int arm720t_handle_md_phys_command(command_context_t *cmd_ctx, char *cmd, char **args, int argc)
{	
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	arm7tdmi_common_t *arm7tdmi;
	arm720t_common_t *arm720t;
	arm_jtag_t *jtag_info;

	if (arm720t_get_arch_pointers(target, &armv4_5, &arm7_9, &arm7tdmi, &arm720t) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM720t target");
		return ERROR_OK;
	}
	
	jtag_info = &arm7_9->jtag_info;
	
	if (target->state != TARGET_HALTED)
	{
		command_print(cmd_ctx, "target must be stopped for \"%s\" command", cmd);
		return ERROR_OK;
	}
	
	return armv4_5_mmu_handle_md_phys_command(cmd_ctx, cmd, args, argc, target, &arm720t->armv4_5_mmu);
}

int arm720t_handle_mw_phys_command(command_context_t *cmd_ctx, char *cmd, char **args, int argc)
{	
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	arm7tdmi_common_t *arm7tdmi;
	arm720t_common_t *arm720t;
	arm_jtag_t *jtag_info;

	if (arm720t_get_arch_pointers(target, &armv4_5, &arm7_9, &arm7tdmi, &arm720t) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM720t target");
		return ERROR_OK;
	}
	
	jtag_info = &arm7_9->jtag_info;
	
	if (target->state != TARGET_HALTED)
	{
		command_print(cmd_ctx, "target must be stopped for \"%s\" command", cmd);
		return ERROR_OK;
	}
	
	return armv4_5_mmu_handle_mw_phys_command(cmd_ctx, cmd, args, argc, target, &arm720t->armv4_5_mmu);
}

