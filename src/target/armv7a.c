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

#include "target.h"
#include "register.h"
#include "log.h"
#include "binarybuffer.h"
#include "command.h"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

struct bitfield_desc armv7a_psr_bitfield_desc[] =
{
	{"M[4:0]", 5},
	{"T", 1},
	{"F", 1},
	{"I", 1},
	{"A", 1},
	{"E", 1},
	{"IT[7:2]", 6},
	{"GE[3:0]", 4},
	{"reserved(DNM)", 4},
	{"J", 1},
	{"IT[0:1]", 2},
	{"Q", 1},
	{"V", 1},
	{"C", 1},
	{"Z", 1},
	{"N", 1},
};

char* armv7a_core_reg_list[] =
{
	"r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7",
	"r8", "r9", "r10", "r11", "r12", "r13_usr", "lr_usr", "pc",
	"r8_fiq", "r9_fiq", "r10_fiq", "r11_fiq", "r12_fiq", "r13_fiq", "lr_fiq",
	"r13_irq", "lr_irq",
	"r13_svc", "lr_svc",
	"r13_abt", "lr_abt",
	"r13_und", "lr_und",
	"cpsr", "spsr_fiq", "spsr_irq", "spsr_svc", "spsr_abt", "spsr_und",
	"r13_mon", "lr_mon", "spsr_mon"
};

char * armv7a_mode_strings_list[] =
{
	"Illegal mode value", "User", "FIQ", "IRQ",
	"Supervisor", "Abort", "Undefined", "System", "Monitor"
};

/* Hack! Yuk! allow -1 index, which simplifies codepaths elsewhere in the code */
char** armv7a_mode_strings = armv7a_mode_strings_list+1;

char* armv7a_state_strings[] =
{
	"ARM", "Thumb", "Jazelle", "ThumbEE"
};

struct armv7a_core_reg armv7a_core_reg_list_arch_info[] =
{
	{0, ARMV4_5_MODE_ANY, NULL, NULL},
	{1, ARMV4_5_MODE_ANY, NULL, NULL},
	{2, ARMV4_5_MODE_ANY, NULL, NULL},
	{3, ARMV4_5_MODE_ANY, NULL, NULL},
	{4, ARMV4_5_MODE_ANY, NULL, NULL},
	{5, ARMV4_5_MODE_ANY, NULL, NULL},
	{6, ARMV4_5_MODE_ANY, NULL, NULL},
	{7, ARMV4_5_MODE_ANY, NULL, NULL},
	{8, ARMV4_5_MODE_ANY, NULL, NULL},
	{9, ARMV4_5_MODE_ANY, NULL, NULL},
	{10, ARMV4_5_MODE_ANY, NULL, NULL},
	{11, ARMV4_5_MODE_ANY, NULL, NULL},
	{12, ARMV4_5_MODE_ANY, NULL, NULL},
	{13, ARMV4_5_MODE_USR, NULL, NULL},
	{14, ARMV4_5_MODE_USR, NULL, NULL},
	{15, ARMV4_5_MODE_ANY, NULL, NULL},

	{8, ARMV4_5_MODE_FIQ, NULL, NULL},
	{9, ARMV4_5_MODE_FIQ, NULL, NULL},
	{10, ARMV4_5_MODE_FIQ, NULL, NULL},
	{11, ARMV4_5_MODE_FIQ, NULL, NULL},
	{12, ARMV4_5_MODE_FIQ, NULL, NULL},
	{13, ARMV4_5_MODE_FIQ, NULL, NULL},
	{14, ARMV4_5_MODE_FIQ, NULL, NULL},

	{13, ARMV4_5_MODE_IRQ, NULL, NULL},
	{14, ARMV4_5_MODE_IRQ, NULL, NULL},

	{13, ARMV4_5_MODE_SVC, NULL, NULL},
	{14, ARMV4_5_MODE_SVC, NULL, NULL},

	{13, ARMV4_5_MODE_ABT, NULL, NULL},
	{14, ARMV4_5_MODE_ABT, NULL, NULL},

	{13, ARMV4_5_MODE_UND, NULL, NULL},
	{14, ARMV4_5_MODE_UND, NULL, NULL},

	{16, ARMV4_5_MODE_ANY, NULL, NULL},
	{16, ARMV4_5_MODE_FIQ, NULL, NULL},
	{16, ARMV4_5_MODE_IRQ, NULL, NULL},
	{16, ARMV4_5_MODE_SVC, NULL, NULL},
	{16, ARMV4_5_MODE_ABT, NULL, NULL},
	{16, ARMV4_5_MODE_UND, NULL, NULL},

	{13, ARMV7A_MODE_MON, NULL, NULL},
	{14, ARMV7A_MODE_MON, NULL, NULL},
	{16, ARMV7A_MODE_MON, NULL, NULL}
};

/* map core mode (USR, FIQ, ...) and register number to indizes into the register cache */
int armv7a_core_reg_map[8][17] =
{
	{	/* USR */
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 31
	},
	{	/* FIQ */
		0, 1, 2, 3, 4, 5, 6, 7, 16, 17, 18, 19, 20, 21, 22, 15, 32
	},
	{	/* IRQ */
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 23, 24, 15, 33
	},
	{	/* SVC */
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 25, 26, 15, 34
	},
	{	/* ABT */
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 27, 28, 15, 35
	},
	{	/* UND */
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 29, 30, 15, 36
	},
	{	/* SYS */
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 31
	},
	{	/* MON */
		/* TODO Fix the register mapping for mon, we need r13_mon,
		 * r14_mon and spsr_mon
		 */
		0, 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12, 13, 14, 15, 31
	}
};

uint8_t armv7a_gdb_dummy_fp_value[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

reg_t armv7a_gdb_dummy_fp_reg =
{
	"GDB dummy floating-point register", armv7a_gdb_dummy_fp_value,
			0, 1, 96, NULL, 0, NULL, 0
};

void armv7a_show_fault_registers(target_t *target)
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

int armv7a_arch_state(struct target_s *target)
{
	static const char *state[] =
	{
		"disabled", "enabled"
	};

	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct armv4_5_common_s *armv4_5 = &armv7a->armv4_5_common;

	if (armv4_5->common_magic != ARMV4_5_COMMON_MAGIC)
	{
		LOG_ERROR("BUG: called for a non-ARMv4/5 target");
		exit(-1);
	}

	LOG_USER("target halted in %s state due to %s, current mode: %s\n"
			 "%s: 0x%8.8" PRIx32 " pc: 0x%8.8" PRIx32 "\n"
			 "MMU: %s, D-Cache: %s, I-Cache: %s",
		 armv7a_state_strings[armv7a->core_state],
		 Jim_Nvp_value2name_simple(nvp_target_debug_reason,
				target->debug_reason)->name,
		 armv7a_mode_strings[
			armv7a_mode_to_number(armv4_5->core_mode)],
		 armv7a_core_reg_list[armv7a_core_reg_map[
			armv7a_mode_to_number(armv4_5->core_mode)][16]],
		 buf_get_u32(ARMV7A_CORE_REG_MODE(armv4_5->core_cache,
				armv4_5->core_mode, 16).value, 0, 32),
		 buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32),
		 state[armv7a->armv4_5_mmu.mmu_enabled],
		 state[armv7a->armv4_5_mmu.armv4_5_cache.d_u_cache_enabled],
		 state[armv7a->armv4_5_mmu.armv4_5_cache.i_cache_enabled]);

	if (armv4_5->core_mode == ARMV7A_MODE_ABT)
		armv7a_show_fault_registers(target);

	return ERROR_OK;
}


COMMAND_HANDLER(handle_dap_baseaddr_command)
{
	target_t *target = get_current_target(cmd_ctx);
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	return CALL_COMMAND_HANDLER(dap_baseaddr_command, swjdp);
}

COMMAND_HANDLER(handle_dap_memaccess_command)
{
	target_t *target = get_current_target(cmd_ctx);
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	return CALL_COMMAND_HANDLER(dap_memaccess_command, swjdp);
}

COMMAND_HANDLER(handle_dap_apsel_command)
{
	target_t *target = get_current_target(cmd_ctx);
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	return CALL_COMMAND_HANDLER(dap_apsel_command, swjdp);
}

COMMAND_HANDLER(handle_dap_apid_command)
{
	target_t *target = get_current_target(cmd_ctx);
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;

	return CALL_COMMAND_HANDLER(dap_apid_command, swjdp);
}

COMMAND_HANDLER(handle_dap_info_command)
{
	target_t *target = get_current_target(cmd_ctx);
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct swjdp_common *swjdp = &armv7a->swjdp_info;
	uint32_t apsel;

	switch (argc) {
	case 0:
		apsel = swjdp->apsel;
		break;
	case 1:
		COMMAND_PARSE_NUMBER(u32, args[0], apsel);
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return dap_info_command(cmd_ctx, swjdp, apsel);
}

COMMAND_HANDLER(handle_armv7a_disassemble_command)
{
	target_t *target = get_current_target(cmd_ctx);
	struct armv4_5_common_s *armv4_5 = target_to_armv4_5(target);
	int thumb = 0;
	int count = 1;
	uint32_t address;
	int i;

	if (armv4_5->common_magic != ARMV4_5_COMMON_MAGIC) {
		command_print(cmd_ctx, "current target isn't an ARM target");
		return ERROR_OK;
	}

	/* REVISIT:  eventually support ThumbEE disassembly too;
	 * some opcodes work differently.
	 */

	switch (argc) {
	case 3:
		if (strcmp(args[2], "thumb") != 0)
			goto usage;
		thumb = 1;
		/* FALL THROUGH */
	case 2:
		COMMAND_PARSE_NUMBER(int, args[1], count);
		/* FALL THROUGH */
	case 1:
		COMMAND_PARSE_NUMBER(u32, args[0], address);
		if (address & 0x01) {
			if (!thumb) {
				command_print(cmd_ctx, "Disassemble as Thumb");
				thumb = 1;
			}
			address &= ~1;
		}
		break;
	default:
usage:
		command_print(cmd_ctx,
			"usage: armv7a disassemble <address> [<count> ['thumb']]");
		return ERROR_OK;
	}

	for (i = 0; i < count; i++) {
		struct arm_instruction cur_instruction;
		int retval;

		if (thumb) {
			retval = thumb2_opcode(target, address, &cur_instruction);
			if (retval != ERROR_OK)
				return retval;

			address += cur_instruction.instruction_size;
		} else {
			uint32_t opcode;

			retval = target_read_u32(target, address, &opcode);
			if (retval != ERROR_OK)
				return retval;

			retval = arm_evaluate_opcode(opcode, address,
					&cur_instruction);
			if (retval != ERROR_OK)
				return retval;

			address += 4;
		}
		command_print(cmd_ctx, "%s", cur_instruction.text);
	}

	return ERROR_OK;
}

int armv7a_register_commands(struct command_context_s *cmd_ctx)
{
	command_t *arm_adi_v5_dap_cmd;
	command_t *armv7a_cmd;

	arm_adi_v5_dap_cmd = register_command(cmd_ctx, NULL, "dap",
			NULL, COMMAND_ANY,
			"cortex dap specific commands");

	register_command(cmd_ctx, arm_adi_v5_dap_cmd, "info",
			handle_dap_info_command, COMMAND_EXEC,
			"dap info for ap [num], "
			"default currently selected AP");
	register_command(cmd_ctx, arm_adi_v5_dap_cmd, "apsel",
			handle_dap_apsel_command, COMMAND_EXEC,
			"select a different AP [num] (default 0)");
	register_command(cmd_ctx, arm_adi_v5_dap_cmd, "apid",
			handle_dap_apid_command, COMMAND_EXEC,
			"return id reg from AP [num], "
			"default currently selected AP");
	register_command(cmd_ctx, arm_adi_v5_dap_cmd, "baseaddr",
			handle_dap_baseaddr_command, COMMAND_EXEC,
			"return debug base address from AP [num], "
			"default currently selected AP");
	register_command(cmd_ctx, arm_adi_v5_dap_cmd, "memaccess",
			handle_dap_memaccess_command, COMMAND_EXEC,
			"set/get number of extra tck for mem-ap memory "
			"bus access [0-255]");

	armv7a_cmd = register_command(cmd_ctx, NULL, "armv7a",
			NULL, COMMAND_ANY,
			"ARMv7-A specific commands");

	register_command(cmd_ctx, armv7a_cmd, "disassemble",
			handle_armv7a_disassemble_command, COMMAND_EXEC,
			"disassemble instructions <address> [<count> ['thumb']]");

	return ERROR_OK;
}
