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

#include "arm966e.h"

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
int arm966e_register_commands(struct command_context_s *cmd_ctx);

/* forward declarations */
int arm966e_target_create(struct target_s *target, Jim_Interp *interp);
int arm966e_init_target(struct command_context_s *cmd_ctx, struct target_s *target);
int arm966e_quit(void);

target_type_t arm966e_target =
{
	.name = "arm966e",

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

	.register_commands = arm966e_register_commands,
	.target_create = arm966e_target_create,
	.init_target = arm966e_init_target,
	.examine = arm9tdmi_examine,
	.quit = arm966e_quit,
};

int arm966e_init_target(struct command_context_s *cmd_ctx, struct target_s *target)
{
	arm9tdmi_init_target(cmd_ctx, target);

	return ERROR_OK;
}

int arm966e_quit(void)
{
	return ERROR_OK;
}

int arm966e_init_arch_info(target_t *target, arm966e_common_t *arm966e, jtag_tap_t *tap)
{
	arm9tdmi_common_t *arm9tdmi = &arm966e->arm9tdmi_common;
	arm7_9_common_t *arm7_9 = &arm9tdmi->arm7_9_common;

	arm9tdmi_init_arch_info(target, arm9tdmi, tap);

	arm9tdmi->arch_info = arm966e;
	arm966e->common_magic = ARM966E_COMMON_MAGIC;

	/* The ARM966E-S implements the ARMv5TE architecture which
	 * has the BKPT instruction, so we don't have to use a watchpoint comparator
	 */
	arm7_9->arm_bkpt = ARMV5_BKPT(0x0);
	arm7_9->thumb_bkpt = ARMV5_T_BKPT(0x0) & 0xffff;

	return ERROR_OK;
}

int arm966e_target_create( struct target_s *target, Jim_Interp *interp )
{
	arm966e_common_t *arm966e = calloc(1,sizeof(arm966e_common_t));

	arm966e_init_arch_info(target, arm966e, target->tap);

	return ERROR_OK;
}

int arm966e_get_arch_pointers(target_t *target, armv4_5_common_t **armv4_5_p, arm7_9_common_t **arm7_9_p, arm9tdmi_common_t **arm9tdmi_p, arm966e_common_t **arm966e_p)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9;
	arm9tdmi_common_t *arm9tdmi;
	arm966e_common_t *arm966e;

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

	arm966e = arm9tdmi->arch_info;
	if (arm966e->common_magic != ARM966E_COMMON_MAGIC)
	{
		return -1;
	}

	*armv4_5_p = armv4_5;
	*arm7_9_p = arm7_9;
	*arm9tdmi_p = arm9tdmi;
	*arm966e_p = arm966e;

	return ERROR_OK;
}

int arm966e_read_cp15(target_t *target, int reg_addr, u32 *value)
{
	int retval = ERROR_OK;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	scan_field_t fields[3];
	u8 reg_addr_buf = reg_addr & 0x3f;
	u8 nr_w_buf = 0;

	jtag_add_end_state(TAP_IDLE);
	if((retval = arm_jtag_scann(jtag_info, 0xf)) != ERROR_OK)
	{
		return retval;
	}
	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr, NULL);

	fields[0].tap = jtag_info->tap;
	fields[0].num_bits = 32;
	fields[0].out_value = NULL;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;

	fields[1].tap = jtag_info->tap;
	fields[1].num_bits = 6;
	fields[1].out_value = &reg_addr_buf;
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;

	fields[2].tap = jtag_info->tap;
	fields[2].num_bits = 1;
	fields[2].out_value = &nr_w_buf;
	fields[2].out_mask = NULL;
	fields[2].in_value = NULL;
	fields[2].in_check_value = NULL;
	fields[2].in_check_mask = NULL;
	fields[2].in_handler = NULL;
	fields[2].in_handler_priv = NULL;

	jtag_add_dr_scan(3, fields, -1);

	fields[0].in_handler_priv = value;
	fields[0].in_handler = arm_jtag_buf_to_u32;

	jtag_add_dr_scan(3, fields, -1);

#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	if((retval = jtag_execute_queue()) != ERROR_OK)
	{
		return retval;
	}
	LOG_DEBUG("addr: 0x%x value: %8.8x", reg_addr, *value);
#endif

	return ERROR_OK;
}

int arm966e_write_cp15(target_t *target, int reg_addr, u32 value)
{
	int retval = ERROR_OK;
	armv4_5_common_t *armv4_5 = target->arch_info;
	arm7_9_common_t *arm7_9 = armv4_5->arch_info;
	arm_jtag_t *jtag_info = &arm7_9->jtag_info;
	scan_field_t fields[3];
	u8 reg_addr_buf = reg_addr & 0x3f;
	u8 nr_w_buf = 1;
	u8 value_buf[4];

	buf_set_u32(value_buf, 0, 32, value);

	jtag_add_end_state(TAP_IDLE);
	if((retval = arm_jtag_scann(jtag_info, 0xf)) != ERROR_OK)
	{
		return retval;
	}
	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr, NULL);

	fields[0].tap = jtag_info->tap;
	fields[0].num_bits = 32;
	fields[0].out_value = value_buf;
	fields[0].out_mask = NULL;
	fields[0].in_value = NULL;
	fields[0].in_check_value = NULL;
	fields[0].in_check_mask = NULL;
	fields[0].in_handler = NULL;
	fields[0].in_handler_priv = NULL;

	fields[1].tap = jtag_info->tap;
	fields[1].num_bits = 6;
	fields[1].out_value = &reg_addr_buf;
	fields[1].out_mask = NULL;
	fields[1].in_value = NULL;
	fields[1].in_check_value = NULL;
	fields[1].in_check_mask = NULL;
	fields[1].in_handler = NULL;
	fields[1].in_handler_priv = NULL;

	fields[2].tap = jtag_info->tap;
	fields[2].num_bits = 1;
	fields[2].out_value = &nr_w_buf;
	fields[2].out_mask = NULL;
	fields[2].in_value = NULL;
	fields[2].in_check_value = NULL;
	fields[2].in_check_mask = NULL;
	fields[2].in_handler = NULL;
	fields[2].in_handler_priv = NULL;

	jtag_add_dr_scan(3, fields, -1);

#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	LOG_DEBUG("addr: 0x%x value: %8.8x", reg_addr, value);
#endif

	return ERROR_OK;
}

int arm966e_handle_cp15_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int retval;
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	arm9tdmi_common_t *arm9tdmi;
	arm966e_common_t *arm966e;
	arm_jtag_t *jtag_info;

	if (arm966e_get_arch_pointers(target, &armv4_5, &arm7_9, &arm9tdmi, &arm966e) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM966e target");
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
			if ((retval = arm966e_read_cp15(target, address, &value)) != ERROR_OK)
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
			if ((retval = arm966e_write_cp15(target, address, value)) != ERROR_OK)
			{
				command_print(cmd_ctx, "couldn't access reg %i", address);
				return ERROR_OK;
			}
			command_print(cmd_ctx, "%i: %8.8x", address, value);
		}
	}

	return ERROR_OK;
}

int arm966e_register_commands(struct command_context_s *cmd_ctx)
{
	int retval;
	command_t *arm966e_cmd;

	retval = arm9tdmi_register_commands(cmd_ctx);
	arm966e_cmd = register_command(cmd_ctx, NULL, "arm966e", NULL, COMMAND_ANY, "arm966e specific commands");
	register_command(cmd_ctx, arm966e_cmd, "cp15", arm966e_handle_cp15_command, COMMAND_EXEC, "display/modify cp15 register <num> [value]");

	return retval;
}
