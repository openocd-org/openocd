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
#include "target_type.h"


#if 0
#define _DEBUG_INSTRUCTION_EXECUTION_
#endif

int arm966e_init_arch_info(struct target *target, struct arm966e_common *arm966e, struct jtag_tap *tap)
{
	struct arm9tdmi_common *arm9tdmi = &arm966e->arm9tdmi_common;
	struct arm7_9_common *arm7_9 = &arm9tdmi->arm7_9_common;

	arm9tdmi_init_arch_info(target, arm9tdmi, tap);

	arm966e->common_magic = ARM966E_COMMON_MAGIC;

	/* The ARM966E-S implements the ARMv5TE architecture which
	 * has the BKPT instruction, so we don't have to use a watchpoint comparator
	 */
	arm7_9->arm_bkpt = ARMV5_BKPT(0x0);
	arm7_9->thumb_bkpt = ARMV5_T_BKPT(0x0) & 0xffff;

	return ERROR_OK;
}

static int arm966e_target_create(struct target *target, Jim_Interp *interp)
{
	struct arm966e_common *arm966e = calloc(1,sizeof(struct arm966e_common));

	return arm966e_init_arch_info(target, arm966e, target->tap);
}

static int arm966e_verify_pointer(struct command_context *cmd_ctx,
		struct arm966e_common *arm966e)
{
	if (arm966e->common_magic != ARM966E_COMMON_MAGIC) {
		command_print(cmd_ctx, "target is not an ARM966");
		return ERROR_TARGET_INVALID;
	}
	return ERROR_OK;
}

static int arm966e_read_cp15(struct target *target, int reg_addr, uint32_t *value)
{
	int retval = ERROR_OK;
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;
	struct scan_field fields[3];
	uint8_t reg_addr_buf = reg_addr & 0x3f;
	uint8_t nr_w_buf = 0;

	jtag_set_end_state(TAP_IDLE);
	if ((retval = arm_jtag_scann(jtag_info, 0xf)) != ERROR_OK)
	{
		return retval;
	}
	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr, NULL);

	fields[0].tap = jtag_info->tap;
	fields[0].num_bits = 32;
	fields[0].out_value = NULL;
	fields[0].in_value = NULL;

	fields[1].tap = jtag_info->tap;
	fields[1].num_bits = 6;
	fields[1].out_value = &reg_addr_buf;
	fields[1].in_value = NULL;

	fields[2].tap = jtag_info->tap;
	fields[2].num_bits = 1;
	fields[2].out_value = &nr_w_buf;
	fields[2].in_value = NULL;

	jtag_add_dr_scan(3, fields, jtag_get_end_state());

	fields[1].in_value = (uint8_t *)value;

	jtag_add_dr_scan(3, fields, jtag_get_end_state());

	jtag_add_callback(arm_le_to_h_u32, (jtag_callback_data_t)value);


#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	if ((retval = jtag_execute_queue()) != ERROR_OK)
	{
		return retval;
	}
	LOG_DEBUG("addr: 0x%x value: %8.8x", reg_addr, *value);
#endif

	return ERROR_OK;
}

// EXPORTED to str9x (flash)
int arm966e_write_cp15(struct target *target, int reg_addr, uint32_t value)
{
	int retval = ERROR_OK;
	struct arm7_9_common *arm7_9 = target_to_arm7_9(target);
	struct arm_jtag *jtag_info = &arm7_9->jtag_info;
	struct scan_field fields[3];
	uint8_t reg_addr_buf = reg_addr & 0x3f;
	uint8_t nr_w_buf = 1;
	uint8_t value_buf[4];

	buf_set_u32(value_buf, 0, 32, value);

	jtag_set_end_state(TAP_IDLE);
	if ((retval = arm_jtag_scann(jtag_info, 0xf)) != ERROR_OK)
	{
		return retval;
	}
	arm_jtag_set_instr(jtag_info, jtag_info->intest_instr, NULL);

	fields[0].tap = jtag_info->tap;
	fields[0].num_bits = 32;
	fields[0].out_value = value_buf;
	fields[0].in_value = NULL;

	fields[1].tap = jtag_info->tap;
	fields[1].num_bits = 6;
	fields[1].out_value = &reg_addr_buf;
	fields[1].in_value = NULL;

	fields[2].tap = jtag_info->tap;
	fields[2].num_bits = 1;
	fields[2].out_value = &nr_w_buf;
	fields[2].in_value = NULL;

	jtag_add_dr_scan(3, fields, jtag_get_end_state());

#ifdef _DEBUG_INSTRUCTION_EXECUTION_
	LOG_DEBUG("addr: 0x%x value: %8.8x", reg_addr, value);
#endif

	return ERROR_OK;
}

COMMAND_HANDLER(arm966e_handle_cp15_command)
{
	int retval;
	struct target *target = get_current_target(cmd_ctx);
	struct arm966e_common *arm966e = target_to_arm966(target);

	retval = arm966e_verify_pointer(cmd_ctx, arm966e);
	if (retval != ERROR_OK)
		return retval;

	if (target->state != TARGET_HALTED)
	{
		command_print(cmd_ctx, "target must be stopped for \"%s\" command", CMD_NAME);
		return ERROR_OK;
	}

	/* one or more argument, access a single register (write if second argument is given */
	if (argc >= 1)
	{
		uint32_t address;
		COMMAND_PARSE_NUMBER(u32, args[0], address);

		if (argc == 1)
		{
			uint32_t value;
			if ((retval = arm966e_read_cp15(target, address, &value)) != ERROR_OK)
			{
				command_print(cmd_ctx, "couldn't access reg %i", address);
				return ERROR_OK;
			}
			if ((retval = jtag_execute_queue()) != ERROR_OK)
			{
				return retval;
			}

			command_print(cmd_ctx, "%i: %8.8" PRIx32 "", address, value);
		}
		else if (argc == 2)
		{
			uint32_t value;
			COMMAND_PARSE_NUMBER(u32, args[1], value);
			if ((retval = arm966e_write_cp15(target, address, value)) != ERROR_OK)
			{
				command_print(cmd_ctx, "couldn't access reg %i", address);
				return ERROR_OK;
			}
			command_print(cmd_ctx, "%i: %8.8" PRIx32 "", address, value);
		}
	}

	return ERROR_OK;
}

/** Registers commands used to access coprocessor resources. */
int arm966e_register_commands(struct command_context *cmd_ctx)
{
	int retval;
	struct command *arm966e_cmd;

	retval = arm9tdmi_register_commands(cmd_ctx);
	arm966e_cmd = register_command(cmd_ctx, NULL, "arm966e",
			NULL, COMMAND_ANY,
			"arm966e specific commands");
	register_command(cmd_ctx, arm966e_cmd, "cp15",
			arm966e_handle_cp15_command, COMMAND_EXEC,
			"display/modify cp15 register <num> [value]");

	return retval;
}

/** Holds methods for ARM966 targets. */
struct target_type arm966e_target =
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
	.init_target = arm9tdmi_init_target,
	.examine = arm9tdmi_examine,
};
