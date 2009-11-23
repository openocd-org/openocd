/***************************************************************************
 *    Copyright (C) 2009 by David Brownell                                 *
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

#include "replacements.h"

#include "armv7a.h"
#include "arm_disassembler.h"

#include "register.h"
#include "binarybuffer.h"
#include "command.h"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>


static void armv7a_show_fault_registers(struct target *target)
{
	uint32_t dfsr, ifsr, dfar, ifar;
	struct armv7a_common *armv7a = target_to_armv7a(target);

	armv7a->read_cp15(target, 0, 0, 5, 0, &dfsr);
	armv7a->read_cp15(target, 0, 1, 5, 0, &ifsr);
	armv7a->read_cp15(target, 0, 0, 6, 0, &dfar);
	armv7a->read_cp15(target, 0, 2, 6, 0, &ifar);

	LOG_USER("Data fault registers        DFSR: %8.8" PRIx32
			", DFAR: %8.8" PRIx32, dfsr, dfar);
	LOG_USER("Instruction fault registers IFSR: %8.8" PRIx32
			", IFAR: %8.8" PRIx32, ifsr, ifar);

}

int armv7a_arch_state(struct target *target)
{
	static const char *state[] =
	{
		"disabled", "enabled"
	};

	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm *armv4_5 = &armv7a->armv4_5_common;

	if (armv7a->common_magic != ARMV7_COMMON_MAGIC)
	{
		LOG_ERROR("BUG: called for a non-ARMv7A target");
		return ERROR_INVALID_ARGUMENTS;
	}

	LOG_USER("target halted in %s state due to %s, current mode: %s\n"
			 "cpsr: 0x%8.8" PRIx32 " pc: 0x%8.8" PRIx32 "\n"
			 "MMU: %s, D-Cache: %s, I-Cache: %s",
		 armv4_5_state_strings[armv4_5->core_state],
		 Jim_Nvp_value2name_simple(nvp_target_debug_reason,
				target->debug_reason)->name,
		 arm_mode_name(armv4_5->core_mode),
		 buf_get_u32(armv4_5->cpsr->value, 0, 32),
		 buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32),
		 state[armv7a->armv4_5_mmu.mmu_enabled],
		 state[armv7a->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled],
		 state[armv7a->armv4_5_mmu.armv4_5_cache.i_cache_enabled]);

	if (armv4_5->core_mode == ARMV4_5_MODE_ABT)
		armv7a_show_fault_registers(target);

	return ERROR_OK;
}


COMMAND_HANDLER(handle_dap_baseaddr_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	return CALL_COMMAND_HANDLER(dap_baseaddr_command, swjdp);
}

COMMAND_HANDLER(handle_dap_memaccess_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	return CALL_COMMAND_HANDLER(dap_memaccess_command, swjdp);
}

COMMAND_HANDLER(handle_dap_apsel_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	return CALL_COMMAND_HANDLER(dap_apsel_command, swjdp);
}

COMMAND_HANDLER(handle_dap_apid_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	return CALL_COMMAND_HANDLER(dap_apid_command, swjdp);
}

COMMAND_HANDLER(handle_dap_info_command)
{
	struct target *target = get_current_target(CMD_CTX);
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;
	uint32_t apsel;

	switch (CMD_ARGC) {
	case 0:
		apsel = swjdp->apsel;
		break;
	case 1:
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], apsel);
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return dap_info_command(CMD_CTX, swjdp, apsel);
}

static const struct command_registration armv7a_exec_command_handlers[] = {
	{
		.name = "info",
		.handler = &handle_dap_info_command,
		.mode = COMMAND_EXEC,
		.help = "dap info for ap [num], "
			"default currently selected AP",
	},
	{
		.name = "apsel",
		.handler = &handle_dap_apsel_command,
		.mode = COMMAND_EXEC,
		.help = "select a different AP [num] (default 0)",
	},
	{
		.name = "apid",
		.handler = &handle_dap_apid_command,
		.mode = COMMAND_EXEC,
		.help = "return id reg from AP [num], "
			"default currently selected AP",
	},
	{
		.name = "baseaddr",
		.handler = &handle_dap_baseaddr_command,
		.mode = COMMAND_EXEC,
		.help = "return debug base address from AP [num], "
			"default currently selected AP",
	},
	{
		.name = "memaccess",
		.handler = &handle_dap_memaccess_command,
		.mode = COMMAND_EXEC,
		.help = "set/get number of extra tck for mem-ap memory "
			"bus access [0-255]",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration armv7a_command_handlers[] = {
	{
		.name = "dap",
		.mode = COMMAND_ANY,
		.help = "Cortex DAP command group",
		.chain = armv7a_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

int armv7a_register_commands(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, armv7a_command_handlers);
}
