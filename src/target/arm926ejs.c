/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
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

#include "arm926ejs.h"
#include "jtag.h"
#include "log.h"

#include <stdlib.h>
#include <string.h>

#if 1
#define _DEBUG_INSTRUCTION_EXECUTION_
#endif

/* cli handling */
int arm926ejs_register_commands(struct command_context_s *cmd_ctx);

int arm926ejs_handle_cp15_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int arm926ejs_handle_cp15i_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int arm926ejs_handle_virt2phys_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int arm926ejs_handle_cache_info_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int arm926ejs_handle_md_phys_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int arm926ejs_handle_mw_phys_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

int arm926ejs_handle_read_cache_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int arm926ejs_handle_read_mmu_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

/* forward declarations */
int arm926ejs_target_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct target_s *target);
int arm926ejs_init_target(struct command_context_s *cmd_ctx, struct target_s *target);
int arm926ejs_quit();
int arm926ejs_arch_state(struct target_s *target, char *buf, int buf_size);
int arm926ejs_read_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);
int arm926ejs_write_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);
int arm926ejs_soft_reset_halt(struct target_s *target);

#define ARM926EJS_CP15_ADDR(opcode_1, opcode_2, CRn, CRm) ((opcode_1 << 11) | (opcode_2 << 8) | (CRn << 4) | (CRm << 0))

target_type_t arm926ejs_target =
{
	.name = "arm926ejs",

	.poll = arm7_9_poll,
	.arch_state = arm926ejs_arch_state,

	.halt = arm7_9_halt,
	.resume = arm7_9_resume,
	.step = arm7_9_step,

	.assert_reset = arm7_9_assert_reset,
	.deassert_reset = arm7_9_deassert_reset,
	.soft_reset_halt = arm926ejs_soft_reset_halt,
	.prepare_reset_halt = arm7_9_prepare_reset_halt,
	
	.get_gdb_reg_list = armv4_5_get_gdb_reg_list,

	.read_memory = arm7_9_read_memory,
	.write_memory = arm926ejs_write_memory,
	.bulk_write_memory = arm7_9_bulk_write_memory,

	.run_algorithm = armv4_5_run_algorithm,

	.add_breakpoint = arm7_9_add_breakpoint,
	.remove_breakpoint = arm7_9_remove_breakpoint,
	.add_watchpoint = arm7_9_add_watchpoint,
	.remove_watchpoint = arm7_9_remove_watchpoint,

	.register_commands = arm926ejs_register_commands,
	.target_command = arm926ejs_target_command,
	.init_target = arm926ejs_init_target,
	.quit = arm926ejs_quit
};

int arm926ejs_catch_broken_irscan(u8 *in_value, void *priv)
{
	/* The ARM926EJ-S' instruction register is 4 bits wide */
	*in_value &= 0xf;
	
	if ((*in_value == 0x0f) || (*in_value == 0x00))
	{
		DEBUG("caught ARM926EJ-S invalid Capture-IR result after CP15 access");
		return ERROR_OK;
	}
	else
	{
		return ERROR_JTAG_QUEUE_FAILED;
	}
}

int arm926ejs_read_cp15(target_t *target, u32 address, u32 *value)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	scan_field_t fields[4];
	u8 address_buf[2];
	u8 nr_w_buf = 0;
	u8 access = 1;
	error_handler_t error_handler;
	
	buf_set_u32(address_buf, 0, 14, address);
	
	jtag_add_end_state(TAP_RTI);
	arm_jtag_scann(jtag_info, 0xf);
	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr, NULL);

	fields[0].device = jtag_info->chain_pos;
	fields[0].num_bits = 32;
	fields[0].out_value = NULL;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;

	fields[1].device = jtag_info->chain_pos;
	fields[1].num_bits = 1;
	fields[1].out_value = &access;
	fields[1].out_mask = NULL;
	fields[1].in_value = &access;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;

	fields[2].device = jtag_info->chain_pos;
	fields[2].num_bits = 14;
	fields[2].out_value = address_buf;
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
	
	jtag_add_dr_scan(4, fields, -1, NULL);

	fields[0].in_handler_priv = value;
	fields[0].in_handler = arm_jtag_buf_to_u32;
	
	do
	{
		/* rescan with NOP, to wait for the access to complete */
		access = 0;
		nr_w_buf = 0;
		jtag_add_dr_scan(4, fields, -1, NULL);
		jtag_execute_queue();
	} while (buf_get_u32(&access, 0, 1) != 1);

#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	DEBUG("addr: 0x%x value: %8.8x", address, *value);
#endif

	error_handler.error_handler = arm926ejs_catch_broken_irscan;
	error_handler.error_handler_priv = NULL;
	
	arm_jtag_set_instr(jtag_info, 0xc, &error_handler);

	return ERROR_OK;
}

int arm926ejs_write_cp15(target_t *target, u32 address, u32 value)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	scan_field_t fields[4];
	u8 value_buf[4];
	u8 address_buf[2];
	u8 nr_w_buf = 1;
	u8 access = 1;
	error_handler_t error_handler;
	
	buf_set_u32(address_buf, 0, 14, address);
	buf_set_u32(value_buf, 0, 32, value);
	
	jtag_add_end_state(TAP_RTI);
	arm_jtag_scann(jtag_info, 0xf);
	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr, NULL);

	fields[0].device = jtag_info->chain_pos;
	fields[0].num_bits = 32;
	fields[0].out_value = value_buf;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;

	fields[1].device = jtag_info->chain_pos;
	fields[1].num_bits = 1;
	fields[1].out_value = &access;
	fields[1].out_mask = NULL;
	fields[1].in_value = &access;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;

	fields[2].device = jtag_info->chain_pos;
	fields[2].num_bits = 14;
	fields[2].out_value = address_buf;
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
	
	jtag_add_dr_scan(4, fields, -1, NULL);

	do
	{
		/* rescan with NOP, to wait for the access to complete */
		access = 0;
		nr_w_buf = 0;
		jtag_add_dr_scan(4, fields, -1, NULL);
		jtag_execute_queue();
	} while (buf_get_u32(&access, 0, 1) != 1);

#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	DEBUG("addr: 0x%x value: %8.8x", address, value);
#endif

	error_handler.error_handler = arm926ejs_catch_broken_irscan;
	error_handler.error_handler_priv = NULL;
	
	arm_jtag_set_instr(jtag_info, 0xf, &error_handler);

	return ERROR_OK;
}

int arm926ejs_examine_debug_reason(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	reg_t *dbg_stat = &arm7_9->eice_cache->reg_list[EICE_DBG_STAT];
	int debug_reason;
	int retval;

	embeddedice_read_reg(dbg_stat);
	if ((retval = jtag_execute_queue()) != ERROR_OK)
		return retval;
	
	debug_reason = buf_get_u32(dbg_stat->value, 6, 4);
	
	switch (debug_reason)
	{
		case 1:
			DEBUG("breakpoint from EICE unit 0");
			target->debug_reason = DBG_REASON_BREAKPOINT;
			break;
		case 2:
			DEBUG("breakpoint from EICE unit 1");
			target->debug_reason = DBG_REASON_BREAKPOINT;
			break;
		case 3:
			DEBUG("soft breakpoint (BKPT instruction)");
			target->debug_reason = DBG_REASON_BREAKPOINT;
			break;
		case 4:
			DEBUG("vector catch breakpoint");
			target->debug_reason = DBG_REASON_BREAKPOINT;
			break;
		case 5:
			DEBUG("external breakpoint");
			target->debug_reason = DBG_REASON_BREAKPOINT;
			break;
		case 6:
			DEBUG("watchpoint from EICE unit 0");
			target->debug_reason = DBG_REASON_WATCHPOINT;
			break;
		case 7:
			DEBUG("watchpoint from EICE unit 1");
			target->debug_reason = DBG_REASON_WATCHPOINT;
			break;
		case 8:
			DEBUG("external watchpoint");
			target->debug_reason = DBG_REASON_WATCHPOINT;
			break;
		case 9:
			DEBUG("internal debug request");
			target->debug_reason = DBG_REASON_DBGRQ;
			break;
		case 10:
			DEBUG("external debug request");
			target->debug_reason = DBG_REASON_DBGRQ;
			break;
		case 11:
			ERROR("BUG: debug re-entry from system speed access shouldn't be handled here");
			break;
		default:
			ERROR("BUG: unknown debug reason: 0x%x", debug_reason);
			target->debug_reason = DBG_REASON_DBGRQ;
	}
	
	return ERROR_OK;
}

u32 arm926ejs_get_ttb(target_t *target)
{
	int retval;
	u32 ttb = 0x0;

	if ((retval = arm926ejs_read_cp15(target, ARM926EJS_CP15_ADDR(0, 0, 2, 0), &ttb)) != ERROR_OK)
		return retval;

	return ttb;
}

void arm926ejs_disable_mmu_caches(target_t *target, int mmu, int d_u_cache, int i_cache)
{
	u32 cp15_control;

	/* read cp15 control register */
	arm926ejs_read_cp15(target, ARM926EJS_CP15_ADDR(0, 0, 1, 0), &cp15_control);
	jtag_execute_queue();
	
	if (mmu)
	{
		/* invalidate TLB */
		arm926ejs_write_cp15(target, ARM926EJS_CP15_ADDR(0, 0, 8, 7), 0x0);
		
		cp15_control &= ~0x1U;
	}
	
	if (d_u_cache)
	{
		u32 debug_override;
		/* read-modify-write CP15 debug override register 
		 * to enable "test and clean all" */
		arm926ejs_read_cp15(target, ARM926EJS_CP15_ADDR(0, 0, 15, 0), &debug_override);
		debug_override |= 0x80000;
		arm926ejs_write_cp15(target, ARM926EJS_CP15_ADDR(0, 0, 15, 0), debug_override);
		
		/* clean and invalidate DCache */
		arm926ejs_write_cp15(target, ARM926EJS_CP15_ADDR(0, 0, 7, 5), 0x0);

		/* write CP15 debug override register 
		 * to disable "test and clean all" */
		debug_override &= ~0x80000;
		arm926ejs_write_cp15(target, ARM926EJS_CP15_ADDR(0, 0, 15, 0), debug_override);
		
		cp15_control &= ~0x4U;
	}
	
	if (i_cache)
	{
		/* invalidate ICache */
		arm926ejs_write_cp15(target, ARM926EJS_CP15_ADDR(0, 0, 7, 5), 0x0);
		
		cp15_control &= ~0x1000U;
	}
	
	arm926ejs_write_cp15(target, ARM926EJS_CP15_ADDR(0, 0, 1, 0), cp15_control);
}

void arm926ejs_enable_mmu_caches(target_t *target, int mmu, int d_u_cache, int i_cache)
{
	u32 cp15_control;

	/* read cp15 control register */
	arm926ejs_read_cp15(target, ARM926EJS_CP15_ADDR(0, 0, 1, 0), &cp15_control);
	jtag_execute_queue();
		
	if (mmu)
		cp15_control |= 0x1U;
	
	if (d_u_cache)
		cp15_control |= 0x4U;
	
	if (i_cache)
		cp15_control |= 0x1000U;
	
	arm926ejs_write_cp15(target, ARM926EJS_CP15_ADDR(0, 0, 1, 0), cp15_control);
}

void arm926ejs_post_debug_entry(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm9tdmi_common_t *arm9tdmi = arm7_9->arch_info;
	arm926ejs_common_t *arm926ejs = arm9tdmi->arch_info;

	/* examine cp15 control reg */
	arm926ejs_read_cp15(target, ARM926EJS_CP15_ADDR(0, 0, 1, 0), &arm926ejs->cp15_control_reg);
	jtag_execute_queue();
	DEBUG("cp15_control_reg: %8.8x", arm926ejs->cp15_control_reg);

	if (arm926ejs->armv4_5_mmu.armv4_5_cache.ctype == -1)
	{
		u32 cache_type_reg;
		/* identify caches */
		arm926ejs_read_cp15(target, ARM926EJS_CP15_ADDR(0, 1, 0, 0), &cache_type_reg);
		jtag_execute_queue();
		armv4_5_identify_cache(cache_type_reg, &arm926ejs->armv4_5_mmu.armv4_5_cache);
	}

	arm926ejs->armv4_5_mmu.mmu_enabled = (arm926ejs->cp15_control_reg & 0x1U) ? 1 : 0;
	arm926ejs->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled = (arm926ejs->cp15_control_reg & 0x4U) ? 1 : 0;
	arm926ejs->armv4_5_mmu.armv4_5_cache.i_cache_enabled = (arm926ejs->cp15_control_reg & 0x1000U) ? 1 : 0;

	/* save i/d fault status and address register */
	arm926ejs_read_cp15(target, ARM926EJS_CP15_ADDR(0, 0, 5, 0), &arm926ejs->d_fsr);
	arm926ejs_read_cp15(target, ARM926EJS_CP15_ADDR(0, 1, 5, 0), &arm926ejs->i_fsr);
	arm926ejs_read_cp15(target, ARM926EJS_CP15_ADDR(0, 0, 6, 0), &arm926ejs->d_far);
	
	DEBUG("D FSR: 0x%8.8x, D FAR: 0x%8.8x, I FSR: 0x%8.8x",
		arm926ejs->d_fsr, arm926ejs->d_far, arm926ejs->i_fsr);  


	u32 cache_dbg_ctrl;
	
	/* read-modify-write CP15 cache debug control register 
	 * to disable I/D-cache linefills and force WT */
	arm926ejs_read_cp15(target, ARM926EJS_CP15_ADDR(7, 0, 15, 0), &cache_dbg_ctrl);
	cache_dbg_ctrl |= 0x7;
	arm926ejs_write_cp15(target, ARM926EJS_CP15_ADDR(7, 0, 15, 0), cache_dbg_ctrl);
}

void arm926ejs_pre_restore_context(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm9tdmi_common_t *arm9tdmi = arm7_9->arch_info;
	arm926ejs_common_t *arm926ejs = arm9tdmi->arch_info;

	/* restore i/d fault status and address register */
	arm926ejs_write_cp15(target, ARM926EJS_CP15_ADDR(0, 0, 5, 0), arm926ejs->d_fsr);
	arm926ejs_write_cp15(target, ARM926EJS_CP15_ADDR(0, 1, 5, 0), arm926ejs->i_fsr);
	arm926ejs_write_cp15(target, ARM926EJS_CP15_ADDR(0, 0, 6, 0), arm926ejs->d_far);
	
	u32 cache_dbg_ctrl;
	
	/* read-modify-write CP15 cache debug control register 
	 * to reenable I/D-cache linefills and disable WT */
	arm926ejs_read_cp15(target, ARM926EJS_CP15_ADDR(7, 0, 15, 0), &cache_dbg_ctrl);
	cache_dbg_ctrl &= ~0x7;
	arm926ejs_write_cp15(target, ARM926EJS_CP15_ADDR(7, 0, 15, 0), cache_dbg_ctrl);
}

int arm926ejs_get_arch_pointers(target_t *target, armv4_5_common_t **armv4_5_p, arm7_9_common_t **arm7_9_p, arm9tdmi_common_t **arm9tdmi_p, arm926ejs_common_t **arm926ejs_p)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9;
	arm9tdmi_common_t *arm9tdmi;
	arm926ejs_common_t *arm926ejs;
	
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
	
	arm926ejs = arm9tdmi->arch_info;
	if (arm926ejs->common_magic != ARM926EJS_COMMON_MAGIC)
	{
		return -1;
	}
	
	*armv4_5_p = armv4_5;
	*arm7_9_p = arm7_9;
	*arm9tdmi_p = arm9tdmi;
	*arm926ejs_p = arm926ejs;
	
	return ERROR_OK;
}

int arm926ejs_arch_state(struct target_s *target, char *buf, int buf_size)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm9tdmi_common_t *arm9tdmi = arm7_9->arch_info;
	arm926ejs_common_t *arm926ejs = arm9tdmi->arch_info;
	
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
			 state[arm926ejs->armv4_5_mmu.mmu_enabled],
			 state[arm926ejs->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled], 
			 state[arm926ejs->armv4_5_mmu.armv4_5_cache.i_cache_enabled]);
	
	return ERROR_OK;
}

int arm926ejs_soft_reset_halt(struct target_s *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm9tdmi_common_t *arm9tdmi = arm7_9->arch_info;
	arm926ejs_common_t *arm926ejs = arm9tdmi->arch_info;
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
	
	arm926ejs_disable_mmu_caches(target, 1, 1, 1);
	arm926ejs->armv4_5_mmu.mmu_enabled = 0;
	arm926ejs->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled = 0;
	arm926ejs->armv4_5_mmu.armv4_5_cache.i_cache_enabled = 0;

	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	
	return ERROR_OK;
}

int arm926ejs_write_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer)
{
	int retval;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm9tdmi_common_t *arm9tdmi = arm7_9->arch_info;
	arm926ejs_common_t *arm926ejs = arm9tdmi->arch_info;
	
	if ((retval = arm7_9_write_memory(target, address, size, count, buffer)) != ERROR_OK)
		return retval;

	/* If ICache is enabled, we have to invalidate affected ICache lines
	 * the DCache is forced to write-through, so we don't have to clean it here
	 */
	if (arm926ejs->armv4_5_mmu.armv4_5_cache.i_cache_enabled)
	{
		if (count <= 1)
		{
			/* invalidate ICache single entry with MVA */
			arm926ejs_write_cp15(target, ARM926EJS_CP15_ADDR(0, 1, 7, 5), address);
		}
		else
		{
			/* invalidate ICache */
			arm926ejs_write_cp15(target, ARM926EJS_CP15_ADDR(0, 0, 7, 5), address);
		}
	}

	return retval;
}

int arm926ejs_init_target(struct command_context_s *cmd_ctx, struct target_s *target)
{
	arm9tdmi_init_target(cmd_ctx, target);
		
	return ERROR_OK;
	
}

int arm926ejs_quit()
{
	
	return ERROR_OK;
}

int arm926ejs_init_arch_info(target_t *target, arm926ejs_common_t *arm926ejs, int chain_pos, char *variant)
{
	arm9tdmi_common_t *arm9tdmi = &arm926ejs->arm9tdmi_common;
	arm7_9_common_t *arm7_9 = &arm9tdmi->arm7_9_common;
	
	/* initialize arm9tdmi specific info (including arm7_9 and armv4_5)
	 */
	arm9tdmi_init_arch_info(target, arm9tdmi, chain_pos, variant);

	arm9tdmi->arch_info = arm926ejs;
	arm926ejs->common_magic = ARM926EJS_COMMON_MAGIC;
	
	arm7_9->post_debug_entry = arm926ejs_post_debug_entry;
	arm7_9->pre_restore_context = arm926ejs_pre_restore_context;
	
	arm926ejs->armv4_5_mmu.armv4_5_cache.ctype = -1;
	arm926ejs->armv4_5_mmu.get_ttb = arm926ejs_get_ttb;
	arm926ejs->armv4_5_mmu.read_memory = arm7_9_read_memory;
	arm926ejs->armv4_5_mmu.write_memory = arm7_9_write_memory;
	arm926ejs->armv4_5_mmu.disable_mmu_caches = arm926ejs_disable_mmu_caches;
	arm926ejs->armv4_5_mmu.enable_mmu_caches = arm926ejs_enable_mmu_caches;
	arm926ejs->armv4_5_mmu.has_tiny_pages = 1;
	arm926ejs->armv4_5_mmu.mmu_enabled = 0;
	
	arm7_9->examine_debug_reason = arm926ejs_examine_debug_reason;
	
	/* The ARM926EJ-S implements the ARMv5TE architecture which
	 * has the BKPT instruction, so we don't have to use a watchpoint comparator
	 */
	arm7_9->arm_bkpt = ARMV5_BKPT(0x0);
	arm7_9->thumb_bkpt = ARMV5_T_BKPT(0x0) & 0xffff;
	
	arm7_9->sw_bkpts_use_wp = 0;
	arm7_9->sw_bkpts_enabled = 1;
	
	return ERROR_OK;
}

int arm926ejs_target_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct target_s *target)
{
	int chain_pos;
	char *variant = NULL;
	arm926ejs_common_t *arm926ejs = malloc(sizeof(arm926ejs_common_t));
	
	if (argc < 4)
	{
		ERROR("'target arm926ejs' requires at least one additional argument");
		exit(-1);
	}
	
	chain_pos = strtoul(args[3], NULL, 0);
	
	if (argc >= 5)
		variant = args[4];
	
	DEBUG("chain_pos: %i, variant: %s", chain_pos, variant);
	
	arm926ejs_init_arch_info(target, arm926ejs, chain_pos, variant);

	return ERROR_OK;
}

int arm926ejs_register_commands(struct command_context_s *cmd_ctx)
{
	int retval;
	command_t *arm926ejs_cmd;
	
		
	retval = arm9tdmi_register_commands(cmd_ctx);
	
	arm926ejs_cmd = register_command(cmd_ctx, NULL, "arm926ejs", NULL, COMMAND_ANY, "arm926ejs specific commands");

	register_command(cmd_ctx, arm926ejs_cmd, "cp15", arm926ejs_handle_cp15_command, COMMAND_EXEC, "display/modify cp15 register <opcode_1> <opcode_2> <CRn> <CRm> [value]");
	
	register_command(cmd_ctx, arm926ejs_cmd, "cache_info", arm926ejs_handle_cache_info_command, COMMAND_EXEC, "display information about target caches");
	register_command(cmd_ctx, arm926ejs_cmd, "virt2phys", arm926ejs_handle_virt2phys_command, COMMAND_EXEC, "translate va to pa <va>");

	register_command(cmd_ctx, arm926ejs_cmd, "mdw_phys", arm926ejs_handle_md_phys_command, COMMAND_EXEC, "display memory words <physical addr> [count]");
	register_command(cmd_ctx, arm926ejs_cmd, "mdh_phys", arm926ejs_handle_md_phys_command, COMMAND_EXEC, "display memory half-words <physical addr> [count]");
	register_command(cmd_ctx, arm926ejs_cmd, "mdb_phys", arm926ejs_handle_md_phys_command, COMMAND_EXEC, "display memory bytes <physical addr> [count]");

	register_command(cmd_ctx, arm926ejs_cmd, "mww_phys", arm926ejs_handle_mw_phys_command, COMMAND_EXEC, "write memory word <physical addr> <value>");
	register_command(cmd_ctx, arm926ejs_cmd, "mwh_phys", arm926ejs_handle_mw_phys_command, COMMAND_EXEC, "write memory half-word <physical addr> <value>");
	register_command(cmd_ctx, arm926ejs_cmd, "mwb_phys", arm926ejs_handle_mw_phys_command, COMMAND_EXEC, "write memory byte <physical addr> <value>");

	return ERROR_OK;
}

int arm926ejs_handle_cp15_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int retval;
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	arm9tdmi_common_t *arm9tdmi;
	arm926ejs_common_t *arm926ejs;
	int opcode_1;
	int opcode_2;
	int CRn;
	int CRm;

	if ((argc < 4) || (argc > 5))
	{
		command_print(cmd_ctx, "usage: arm926ejs cp15 <opcode_1> <opcode_2> <CRn> <CRm> [value]");
		return ERROR_OK;
	}
	
	opcode_1 = strtoul(args[0], NULL, 0);
	opcode_2 = strtoul(args[1], NULL, 0);
	CRn = strtoul(args[2], NULL, 0);
	CRm = strtoul(args[3], NULL, 0);

	if (arm926ejs_get_arch_pointers(target, &armv4_5, &arm7_9, &arm9tdmi, &arm926ejs) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM926EJ-S target");
		return ERROR_OK;
	}
	
	if (target->state != TARGET_HALTED)
	{
		command_print(cmd_ctx, "target must be stopped for \"%s\" command", cmd);
		return ERROR_OK;
	}
	
	if (argc == 4)
	{
		u32 value;
		if ((retval = arm926ejs_read_cp15(target, ARM926EJS_CP15_ADDR(opcode_1, opcode_2, CRn, CRm), &value)) != ERROR_OK)
		{
			command_print(cmd_ctx, "couldn't access register");
			return ERROR_OK;
		}
		jtag_execute_queue();
		
		command_print(cmd_ctx, "%i %i %i %i: %8.8x", opcode_1, opcode_2, CRn, CRm, value);
	}
	else
	{
		u32 value = strtoul(args[4], NULL, 0);
		if ((retval = arm926ejs_write_cp15(target, ARM926EJS_CP15_ADDR(opcode_1, opcode_2, CRn, CRm), value)) != ERROR_OK)
		{
			command_print(cmd_ctx, "couldn't access register");
			return ERROR_OK;
		}
		command_print(cmd_ctx, "%i %i %i %i: %8.8x", opcode_1, opcode_2, CRn, CRm, value);
	}

	return ERROR_OK;
}

int arm926ejs_handle_cache_info_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	arm9tdmi_common_t *arm9tdmi;
	arm926ejs_common_t *arm926ejs;
	
	if (arm926ejs_get_arch_pointers(target, &armv4_5, &arm7_9, &arm9tdmi, &arm926ejs) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM926EJ-S target");
		return ERROR_OK;
	}
	
	return armv4_5_handle_cache_info_command(cmd_ctx, &arm926ejs->armv4_5_mmu.armv4_5_cache);
}

int arm926ejs_handle_virt2phys_command(command_context_t *cmd_ctx, char *cmd, char **args, int argc)
{	
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	arm9tdmi_common_t *arm9tdmi;
	arm926ejs_common_t *arm926ejs;
	arm_jtag_t *jtag_info;

	if (arm926ejs_get_arch_pointers(target, &armv4_5, &arm7_9, &arm9tdmi, &arm926ejs) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM926EJ-S target");
		return ERROR_OK;
	}
	
	jtag_info = &arm7_9->jtag_info;
	
	if (target->state != TARGET_HALTED)
	{
		command_print(cmd_ctx, "target must be stopped for \"%s\" command", cmd);
		return ERROR_OK;
	}
		
	return armv4_5_mmu_handle_virt2phys_command(cmd_ctx, cmd, args, argc, target, &arm926ejs->armv4_5_mmu);
}

int arm926ejs_handle_md_phys_command(command_context_t *cmd_ctx, char *cmd, char **args, int argc)
{	
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	arm9tdmi_common_t *arm9tdmi;
	arm926ejs_common_t *arm926ejs;
	arm_jtag_t *jtag_info;

	if (arm926ejs_get_arch_pointers(target, &armv4_5, &arm7_9, &arm9tdmi, &arm926ejs) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM926EJ-S target");
		return ERROR_OK;
	}
	
	jtag_info = &arm7_9->jtag_info;
	
	if (target->state != TARGET_HALTED)
	{
		command_print(cmd_ctx, "target must be stopped for \"%s\" command", cmd);
		return ERROR_OK;
	}
	
	return armv4_5_mmu_handle_md_phys_command(cmd_ctx, cmd, args, argc, target, &arm926ejs->armv4_5_mmu);
}

int arm926ejs_handle_mw_phys_command(command_context_t *cmd_ctx, char *cmd, char **args, int argc)
{	
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	arm9tdmi_common_t *arm9tdmi;
	arm926ejs_common_t *arm926ejs;
	arm_jtag_t *jtag_info;

	if (arm926ejs_get_arch_pointers(target, &armv4_5, &arm7_9, &arm9tdmi, &arm926ejs) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM926EJ-S target");
		return ERROR_OK;
	}
	
	jtag_info = &arm7_9->jtag_info;
	
	if (target->state != TARGET_HALTED)
	{
		command_print(cmd_ctx, "target must be stopped for \"%s\" command", cmd);
		return ERROR_OK;
	}
	
	return armv4_5_mmu_handle_mw_phys_command(cmd_ctx, cmd, args, argc, target, &arm926ejs->armv4_5_mmu);
}
