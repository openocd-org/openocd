/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by Oyvind Harboe                                   *
 *   oyvind.harboe@zylin.com                                               *
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

#include "arm_disassembler.h"

#include "armv4_5.h"

#include "target.h"
#include "register.h"
#include "log.h"
#include "binarybuffer.h"
#include "command.h"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

bitfield_desc_t armv4_5_psr_bitfield_desc[] =
{
	{"M[4:0]", 5},
	{"T", 1},
	{"F", 1},
	{"I", 1},
	{"reserved", 16},
	{"J", 1},
	{"reserved", 2},
	{"Q", 1},
	{"V", 1},
	{"C", 1},
	{"Z", 1},
	{"N", 1},
};

char* armv4_5_core_reg_list[] =
{
	"r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12", "r13_usr", "lr_usr", "pc",

	"r8_fiq", "r9_fiq", "r10_fiq", "r11_fiq", "r12_fiq", "r13_fiq", "lr_fiq",

	"r13_irq", "lr_irq",

	"r13_svc", "lr_svc",

	"r13_abt", "lr_abt",

	"r13_und", "lr_und",

	"cpsr", "spsr_fiq", "spsr_irq", "spsr_svc", "spsr_abt", "spsr_und"
};

char * armv4_5_mode_strings_list[] =
{
	"Illegal mode value", "User", "FIQ", "IRQ", "Supervisor", "Abort", "Undefined", "System"
};

/* Hack! Yuk! allow -1 index, which simplifies codepaths elsewhere in the code */
char** armv4_5_mode_strings = armv4_5_mode_strings_list+1;

char* armv4_5_state_strings[] =
{
	"ARM", "Thumb", "Jazelle"
};

int armv4_5_core_reg_arch_type = -1;

armv4_5_core_reg_t armv4_5_core_reg_list_arch_info[] =
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
	{16, ARMV4_5_MODE_UND, NULL, NULL}
};

/* map core mode (USR, FIQ, ...) and register number to indizes into the register cache */
int armv4_5_core_reg_map[7][17] =
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
	}
};

u8 armv4_5_gdb_dummy_fp_value[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

reg_t armv4_5_gdb_dummy_fp_reg =
{
	"GDB dummy floating-point register", armv4_5_gdb_dummy_fp_value, 0, 1, 96, NULL, 0, NULL, 0
};

u8 armv4_5_gdb_dummy_fps_value[] = {0, 0, 0, 0};

reg_t armv4_5_gdb_dummy_fps_reg =
{
	"GDB dummy floating-point status register", armv4_5_gdb_dummy_fps_value, 0, 1, 32, NULL, 0, NULL, 0
};

int armv4_5_get_core_reg(reg_t *reg)
{
	int retval;
	armv4_5_core_reg_t *armv4_5 = reg->arch_info;
	target_t *target = armv4_5->target;

	if (target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* retval = armv4_5->armv4_5_common->full_context(target); */
	retval = armv4_5->armv4_5_common->read_core_reg(target, armv4_5->num, armv4_5->mode);

	return retval;
}

int armv4_5_set_core_reg(reg_t *reg, u8 *buf)
{
	armv4_5_core_reg_t *armv4_5 = reg->arch_info;
	target_t *target = armv4_5->target;
	armv4_5_common_t *armv4_5_target = target->arch_info;
	u32 value = buf_get_u32(buf, 0, 32);

	if (target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}

	if (reg == &armv4_5_target->core_cache->reg_list[ARMV4_5_CPSR])
	{
		if (value & 0x20)
		{
			/* T bit should be set */
			if (armv4_5_target->core_state == ARMV4_5_STATE_ARM)
			{
				/* change state to Thumb */
				LOG_DEBUG("changing to Thumb state");
				armv4_5_target->core_state = ARMV4_5_STATE_THUMB;
			}
		}
		else
		{
			/* T bit should be cleared */
			if (armv4_5_target->core_state == ARMV4_5_STATE_THUMB)
			{
				/* change state to ARM */
				LOG_DEBUG("changing to ARM state");
				armv4_5_target->core_state = ARMV4_5_STATE_ARM;
			}
		}

		if (armv4_5_target->core_mode != (value & 0x1f))
		{
			LOG_DEBUG("changing ARM core mode to '%s'", armv4_5_mode_strings[armv4_5_mode_to_number(value & 0x1f)]);
			armv4_5_target->core_mode = value & 0x1f;
			armv4_5_target->write_core_reg(target, 16, ARMV4_5_MODE_ANY, value);
		}
	}

	buf_set_u32(reg->value, 0, 32, value);
	reg->dirty = 1;
	reg->valid = 1;

	return ERROR_OK;
}

int armv4_5_invalidate_core_regs(target_t *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	int i;

	for (i = 0; i < 37; i++)
	{
		armv4_5->core_cache->reg_list[i].valid = 0;
		armv4_5->core_cache->reg_list[i].dirty = 0;
	}

	return ERROR_OK;
}

reg_cache_t* armv4_5_build_reg_cache(target_t *target, armv4_5_common_t *armv4_5_common)
{
	int num_regs = 37;
	reg_cache_t *cache = malloc(sizeof(reg_cache_t));
	reg_t *reg_list = malloc(sizeof(reg_t) * num_regs);
	armv4_5_core_reg_t *arch_info = malloc(sizeof(armv4_5_core_reg_t) * num_regs);
	int i;

	cache->name = "arm v4/5 registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = num_regs;

	if (armv4_5_core_reg_arch_type == -1)
		armv4_5_core_reg_arch_type = register_reg_arch_type(armv4_5_get_core_reg, armv4_5_set_core_reg);

	register_init_dummy(&armv4_5_gdb_dummy_fp_reg);
	register_init_dummy(&armv4_5_gdb_dummy_fps_reg);

	for (i = 0; i < 37; i++)
	{
		arch_info[i] = armv4_5_core_reg_list_arch_info[i];
		arch_info[i].target = target;
		arch_info[i].armv4_5_common = armv4_5_common;
		reg_list[i].name = armv4_5_core_reg_list[i];
		reg_list[i].size = 32;
		reg_list[i].value = calloc(1, 4);
		reg_list[i].dirty = 0;
		reg_list[i].valid = 0;
		reg_list[i].bitfield_desc = NULL;
		reg_list[i].num_bitfields = 0;
		reg_list[i].arch_type = armv4_5_core_reg_arch_type;
		reg_list[i].arch_info = &arch_info[i];
	}

	return cache;
}

int armv4_5_arch_state(struct target_s *target)
{
	armv4_5_common_t *armv4_5 = target->arch_info;

	if (armv4_5->common_magic != ARMV4_5_COMMON_MAGIC)
	{
		LOG_ERROR("BUG: called for a non-ARMv4/5 target");
		exit(-1);
	}

	LOG_USER("target halted in %s state due to %s, current mode: %s\ncpsr: 0x%8.8x pc: 0x%8.8x",
			 armv4_5_state_strings[armv4_5->core_state],
			 Jim_Nvp_value2name_simple( nvp_target_debug_reason, target->debug_reason )->name,
			 armv4_5_mode_strings[armv4_5_mode_to_number(armv4_5->core_mode)],
			 buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 32),
			 buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32));

	return ERROR_OK;
}

int handle_armv4_5_reg_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	char output[128];
	int output_len;
	int mode, num;
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5 = target->arch_info;

	if (armv4_5->common_magic != ARMV4_5_COMMON_MAGIC)
	{
		command_print(cmd_ctx, "current target isn't an ARMV4/5 target");
		return ERROR_OK;
	}

	if (target->state != TARGET_HALTED)
	{
		command_print(cmd_ctx, "error: target must be halted for register accesses");
		return ERROR_OK;
	}

	if (armv4_5_mode_to_number(armv4_5->core_mode)==-1)
		return ERROR_FAIL;

	for (num = 0; num <= 15; num++)
	{
		output_len = 0;
		for (mode = 0; mode < 6; mode++)
		{
			if (!ARMV4_5_CORE_REG_MODENUM(armv4_5->core_cache, mode, num).valid)
			{
				armv4_5->full_context(target);
			}
			output_len += snprintf(output + output_len, 128 - output_len, "%8s: %8.8x ", ARMV4_5_CORE_REG_MODENUM(armv4_5->core_cache, mode, num).name,
				buf_get_u32(ARMV4_5_CORE_REG_MODENUM(armv4_5->core_cache, mode, num).value, 0, 32));
		}
		command_print(cmd_ctx, output);
	}
	command_print(cmd_ctx, "    cpsr: %8.8x spsr_fiq: %8.8x spsr_irq: %8.8x spsr_svc: %8.8x spsr_abt: %8.8x spsr_und: %8.8x",
			  buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 32),
			  buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_SPSR_FIQ].value, 0, 32),
			  buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_SPSR_IRQ].value, 0, 32),
			  buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_SPSR_SVC].value, 0, 32),
			  buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_SPSR_ABT].value, 0, 32),
			  buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_SPSR_UND].value, 0, 32));

	return ERROR_OK;
}

int handle_armv4_5_core_state_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5 = target->arch_info;

	if (armv4_5->common_magic != ARMV4_5_COMMON_MAGIC)
	{
		command_print(cmd_ctx, "current target isn't an ARMV4/5 target");
		return ERROR_OK;
	}

	if (argc > 0)
	{
		if (strcmp(args[0], "arm") == 0)
		{
			armv4_5->core_state = ARMV4_5_STATE_ARM;
		}
		if (strcmp(args[0], "thumb") == 0)
		{
			armv4_5->core_state = ARMV4_5_STATE_THUMB;
		}
	}

	command_print(cmd_ctx, "core state: %s", armv4_5_state_strings[armv4_5->core_state]);

	return ERROR_OK;
}

int handle_armv4_5_disassemble_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int retval = ERROR_OK;
	target_t *target = get_current_target(cmd_ctx);
	armv4_5_common_t *armv4_5 = target->arch_info;
	u32 address;
	int count;
	int i;
	arm_instruction_t cur_instruction;
	u32 opcode;
	u16 thumb_opcode;
	int thumb = 0;

	if (armv4_5->common_magic != ARMV4_5_COMMON_MAGIC)
	{
		command_print(cmd_ctx, "current target isn't an ARMV4/5 target");
		return ERROR_OK;
	}

	if (argc < 2)
	{
		command_print(cmd_ctx, "usage: armv4_5 disassemble <address> <count> ['thumb']");
		return ERROR_OK;
	}

	address = strtoul(args[0], NULL, 0);
	count = strtoul(args[1], NULL, 0);

	if (argc >= 3)
		if (strcmp(args[2], "thumb") == 0)
			thumb = 1;

	for (i = 0; i < count; i++)
	{
		if(thumb)
		{
			if((retval = target_read_u16(target, address, &thumb_opcode)) != ERROR_OK)
			{
				return retval;
			}
			if((retval = thumb_evaluate_opcode(thumb_opcode, address, &cur_instruction)) != ERROR_OK)
			{
				return retval;
			}
		}
		else {
			if((retval = target_read_u32(target, address, &opcode)) != ERROR_OK)
			{
				return retval;
			}
			if((retval = arm_evaluate_opcode(opcode, address, &cur_instruction)) != ERROR_OK)
			{
				return retval;
			}
		}
		command_print(cmd_ctx, "%s", cur_instruction.text);
		address += (thumb) ? 2 : 4;
	}

	return ERROR_OK;
}

int armv4_5_register_commands(struct command_context_s *cmd_ctx)
{
	command_t *armv4_5_cmd;

	armv4_5_cmd = register_command(cmd_ctx, NULL, "armv4_5", NULL, COMMAND_ANY, "armv4/5 specific commands");

	register_command(cmd_ctx, armv4_5_cmd, "reg", handle_armv4_5_reg_command, COMMAND_EXEC, "display ARM core registers");
	register_command(cmd_ctx, armv4_5_cmd, "core_state", handle_armv4_5_core_state_command, COMMAND_EXEC, "display/change ARM core state <arm|thumb>");

	register_command(cmd_ctx, armv4_5_cmd, "disassemble", handle_armv4_5_disassemble_command, COMMAND_EXEC, "disassemble instructions <address> <count> ['thumb']");
	return ERROR_OK;
}

int armv4_5_get_gdb_reg_list(target_t *target, reg_t **reg_list[], int *reg_list_size)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	int i;

	if (armv4_5_mode_to_number(armv4_5->core_mode)==-1)
		return ERROR_FAIL;

	*reg_list_size = 26;
	*reg_list = malloc(sizeof(reg_t*) * (*reg_list_size));

	for (i = 0; i < 16; i++)
	{
		(*reg_list)[i] = &ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, i);
	}

	for (i = 16; i < 24; i++)
	{
		(*reg_list)[i] = &armv4_5_gdb_dummy_fp_reg;
	}

	(*reg_list)[24] = &armv4_5_gdb_dummy_fps_reg;
	(*reg_list)[25] = &armv4_5->core_cache->reg_list[ARMV4_5_CPSR];

	return ERROR_OK;
}

/* wait for execution to complete and check exit point */
static int armv4_5_run_algorithm_completion(struct target_s *target, u32 exit_point, int timeout_ms, void *arch_info)
{
	int retval;
	armv4_5_common_t *armv4_5 = target->arch_info;

	if((retval = target_wait_state(target, TARGET_HALTED, timeout_ms)) != ERROR_OK)
	{
		return retval;
	}
	if (target->state != TARGET_HALTED)
	{
		if ((retval=target_halt(target))!=ERROR_OK)
			return retval;
		if ((retval=target_wait_state(target, TARGET_HALTED, 500))!=ERROR_OK)
		{
			return retval;
		}
		return ERROR_TARGET_TIMEOUT;
	}
	if (buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32) != exit_point)
	{
		LOG_WARNING("target reentered debug state, but not at the desired exit point: 0x%4.4x",
			buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32));
		return ERROR_TARGET_TIMEOUT;
	}

	return ERROR_OK;
}

int armv4_5_run_algorithm_inner(struct target_s *target, int num_mem_params, mem_param_t *mem_params, int num_reg_params, reg_param_t *reg_params, u32 entry_point, u32 exit_point, int timeout_ms, void *arch_info, int (*run_it)(struct target_s *target, u32 exit_point, int timeout_ms, void *arch_info))
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	armv4_5_algorithm_t *armv4_5_algorithm_info = arch_info;
	enum armv4_5_state core_state = armv4_5->core_state;
	enum armv4_5_mode core_mode = armv4_5->core_mode;
	u32 context[17];
	u32 cpsr;
	int exit_breakpoint_size = 0;
	int i;
	int retval = ERROR_OK;
	LOG_DEBUG("Running algorithm");

	if (armv4_5_algorithm_info->common_magic != ARMV4_5_COMMON_MAGIC)
	{
		LOG_ERROR("current target isn't an ARMV4/5 target");
		return ERROR_TARGET_INVALID;
	}

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (armv4_5_mode_to_number(armv4_5->core_mode)==-1)
		return ERROR_FAIL;

	for (i = 0; i <= 16; i++)
	{
		if (!ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_algorithm_info->core_mode, i).valid)
			armv4_5->read_core_reg(target, i, armv4_5_algorithm_info->core_mode);
		context[i] = buf_get_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_algorithm_info->core_mode, i).value, 0, 32);
	}
	cpsr = buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 32);

	for (i = 0; i < num_mem_params; i++)
	{
		if((retval = target_write_buffer(target, mem_params[i].address, mem_params[i].size, mem_params[i].value)) != ERROR_OK)
		{
			return retval;
		}
	}

	for (i = 0; i < num_reg_params; i++)
	{
		reg_t *reg = register_get_by_name(armv4_5->core_cache, reg_params[i].reg_name, 0);
		if (!reg)
		{
			LOG_ERROR("BUG: register '%s' not found", reg_params[i].reg_name);
			exit(-1);
		}

		if (reg->size != reg_params[i].size)
		{
			LOG_ERROR("BUG: register '%s' size doesn't match reg_params[i].size", reg_params[i].reg_name);
			exit(-1);
		}

		if((retval = armv4_5_set_core_reg(reg, reg_params[i].value)) != ERROR_OK)
		{
			return retval;
		}
	}

	armv4_5->core_state = armv4_5_algorithm_info->core_state;
	if (armv4_5->core_state == ARMV4_5_STATE_ARM)
		exit_breakpoint_size = 4;
	else if (armv4_5->core_state == ARMV4_5_STATE_THUMB)
		exit_breakpoint_size = 2;
	else
	{
		LOG_ERROR("BUG: can't execute algorithms when not in ARM or Thumb state");
		exit(-1);
	}

	if (armv4_5_algorithm_info->core_mode != ARMV4_5_MODE_ANY)
	{
		LOG_DEBUG("setting core_mode: 0x%2.2x", armv4_5_algorithm_info->core_mode);
		buf_set_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 5, armv4_5_algorithm_info->core_mode);
		armv4_5->core_cache->reg_list[ARMV4_5_CPSR].dirty = 1;
		armv4_5->core_cache->reg_list[ARMV4_5_CPSR].valid = 1;
	}

	if ((retval = breakpoint_add(target, exit_point, exit_breakpoint_size, BKPT_HARD)) != ERROR_OK)
	{
		LOG_ERROR("can't add breakpoint to finish algorithm execution");
		return ERROR_TARGET_FAILURE;
	}

	if((retval = target_resume(target, 0, entry_point, 1, 1)) != ERROR_OK)
	{
		return retval;
	}
	int retvaltemp;
	retval=run_it(target, exit_point, timeout_ms, arch_info);

	breakpoint_remove(target, exit_point);

	if (retval!=ERROR_OK)
		return retval;

	for (i = 0; i < num_mem_params; i++)
	{
		if (mem_params[i].direction != PARAM_OUT)
			if((retvaltemp = target_read_buffer(target, mem_params[i].address, mem_params[i].size, mem_params[i].value)) != ERROR_OK)
			{
					retval = retvaltemp;
			}
	}

	for (i = 0; i < num_reg_params; i++)
	{
		if (reg_params[i].direction != PARAM_OUT)
		{

			reg_t *reg = register_get_by_name(armv4_5->core_cache, reg_params[i].reg_name, 0);
			if (!reg)
			{
				LOG_ERROR("BUG: register '%s' not found", reg_params[i].reg_name);
				exit(-1);
			}

			if (reg->size != reg_params[i].size)
			{
				LOG_ERROR("BUG: register '%s' size doesn't match reg_params[i].size", reg_params[i].reg_name);
				exit(-1);
			}

			buf_set_u32(reg_params[i].value, 0, 32, buf_get_u32(reg->value, 0, 32));
		}
	}

	for (i = 0; i <= 16; i++)
	{
		u32 regvalue;
		regvalue = buf_get_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_algorithm_info->core_mode, i).value, 0, 32);
		if (regvalue != context[i])
		{
			LOG_DEBUG("restoring register %s with value 0x%8.8x", ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_algorithm_info->core_mode, i).name, context[i]);
			buf_set_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_algorithm_info->core_mode, i).value, 0, 32, context[i]);
			ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_algorithm_info->core_mode, i).valid = 1;
			ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5_algorithm_info->core_mode, i).dirty = 1;
		}
	}
	buf_set_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 32, cpsr);
	armv4_5->core_cache->reg_list[ARMV4_5_CPSR].valid = 1;
	armv4_5->core_cache->reg_list[ARMV4_5_CPSR].dirty = 1;

	armv4_5->core_state = core_state;
	armv4_5->core_mode = core_mode;

	return retval;
}

int armv4_5_run_algorithm(struct target_s *target, int num_mem_params, mem_param_t *mem_params, int num_reg_params, reg_param_t *reg_params, u32 entry_point, u32 exit_point, int timeout_ms, void *arch_info)
{
	return armv4_5_run_algorithm_inner(target, num_mem_params, mem_params, num_reg_params, reg_params, entry_point, exit_point, timeout_ms, arch_info, armv4_5_run_algorithm_completion);
}

int armv4_5_init_arch_info(target_t *target, armv4_5_common_t *armv4_5)
{
	target->arch_info = armv4_5;

	armv4_5->common_magic = ARMV4_5_COMMON_MAGIC;
	armv4_5->core_state = ARMV4_5_STATE_ARM;
	armv4_5->core_mode = ARMV4_5_MODE_USR;

	return ERROR_OK;
}
