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

#include <helper/replacements.h>

#include "armv7a.h"
#include "arm_disassembler.h"

#include "register.h"
#include <helper/binarybuffer.h>
#include <helper/command.h>

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "arm_opcodes.h"


static void armv7a_show_fault_registers(struct target *target)
{
	uint32_t dfsr, ifsr, dfar, ifar;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm_dpm *dpm = armv7a->armv4_5_common.dpm;
	int retval;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		return;

	/* ARMV4_5_MRC(cpnum, op1, r0, CRn, CRm, op2) */

	/* c5/c0 - {data, instruction} fault status registers */
	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 5, 0, 0),
			&dfsr);
	if (retval != ERROR_OK)
		goto done;

	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 5, 0, 1),
			&ifsr);
	if (retval != ERROR_OK)
		goto done;

	/* c6/c0 - {data, instruction} fault address registers */
	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 6, 0, 0),
			&dfar);
	if (retval != ERROR_OK)
		goto done;

	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 6, 0, 2),
			&ifar);
	if (retval != ERROR_OK)
		goto done;

	LOG_USER("Data fault registers        DFSR: %8.8" PRIx32
			", DFAR: %8.8" PRIx32, dfsr, dfar);
	LOG_USER("Instruction fault registers IFSR: %8.8" PRIx32
			", IFAR: %8.8" PRIx32, ifsr, ifar);

done:
	/* (void) */ dpm->finish(dpm);
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

	arm_arch_state(target);

	LOG_USER("MMU: %s, D-Cache: %s, I-Cache: %s",
		 state[armv7a->armv4_5_mmu.mmu_enabled],
		 state[armv7a->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled],
		 state[armv7a->armv4_5_mmu.armv4_5_cache.i_cache_enabled]);

	if (armv4_5->core_mode == ARM_MODE_ABT)
		armv7a_show_fault_registers(target);
	if (target->debug_reason == DBG_REASON_WATCHPOINT)
		LOG_USER("Watchpoint triggered at PC %#08x",
				(unsigned) armv7a->dpm.wp_pc);

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

/* FIXME this table should be part of generic DAP support, and
 * be shared by the ARMv7-A/R and ARMv7-M support ...
 */
static const struct command_registration armv7a_exec_command_handlers[] = {
	{
		.name = "info",
		.handler = handle_dap_info_command,
		.mode = COMMAND_EXEC,
		.help = "display ROM table for MEM-AP "
			"(default currently selected AP)",
		.usage = "[ap_num]",
	},
	{
		.name = "apsel",
		.handler = handle_dap_apsel_command,
		.mode = COMMAND_EXEC,
		.help = "Set the currently selected AP (default 0) "
			"and display the result",
		.usage = "[ap_num]",
	},
	{
		.name = "apid",
		.handler = handle_dap_apid_command,
		.mode = COMMAND_EXEC,
		.help = "return ID register from AP "
			"(default currently selected AP)",
		.usage = "[ap_num]",
	},
	{
		.name = "baseaddr",
		.handler = handle_dap_baseaddr_command,
		.mode = COMMAND_EXEC,
		.help = "return debug base address from MEM-AP "
			"(default currently selected AP)",
		.usage = "[ap_num]",
	},
	{
		.name = "memaccess",
		.handler = handle_dap_memaccess_command,
		.mode = COMMAND_EXEC,
		.help = "set/get number of extra tck for MEM-AP memory "
			"bus access [0-255]",
		.usage = "[cycles]",
	},
	COMMAND_REGISTRATION_DONE
};
const struct command_registration armv7a_command_handlers[] = {
	{
		.name = "dap",
		.mode = COMMAND_ANY,
		.help = "Cortex DAP command group",
		.chain = armv7a_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

