/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
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

#include "armv7m.h"
#include "register.h"
#include "target.h"
#include "log.h"
#include "jtag.h"
#include "arm_jtag.h"

#include <stdlib.h>
#include <string.h>

#if 0
#define _DEBUG_INSTRUCTION_EXECUTION_
#endif

char* armv7m_mode_strings[] =
{
	"Handler", "Thread"
};

char* armv7m_state_strings[] =
{
	"Thumb", "Debug"
};

char* armv7m_exception_strings[] =
{
	"", "Reset", "NMI", "HardFault", "MemManage", "BusFault", "UsageFault", "RESERVED", "RESERVED", "RESERVED", "RESERVED",
	"SVCall", "DebugMonitor", "RESERVED", "PendSV", "SysTick"
};

char* armv7m_core_reg_list[] =
{
/* Registers accessed through core debug */
	"r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12",
	"sp", "lr", "pc",
	"xPSR", "msp", "psp",
/* Registers accessed through MSR instructions */
//	"apsr", "iapsr", "ipsr", "epsr",
	"primask", "basepri", "faultmask", "control"
};

char* armv7m_core_dbgreg_list[] =
{
/* Registers accessed through core debug */
	"r0", "r1", "r2", "r3", "r4", "r5", "r6", "r7", "r8", "r9", "r10", "r11", "r12",
	"sp", "lr", "pc",
	"xPSR", "msp", "psp",
/* Registers accessed through MSR instructions */
//	"dbg_apsr", "iapsr", "ipsr", "epsr",
	"primask", "basepri", "faultmask", "dbg_control"
};

u8 armv7m_gdb_dummy_fp_value[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

reg_t armv7m_gdb_dummy_fp_reg =
{
	"GDB dummy floating-point register", armv7m_gdb_dummy_fp_value, 0, 1, 96, NULL, 0, NULL, 0
};

armv7m_core_reg_t armv7m_core_reg_list_arch_info[] = 
{
	/*  CORE_GP are accesible using the core debug registers */
	{0, ARMV7M_REGISTER_CORE_GP, ARMV7M_MODE_ANY, NULL, NULL},
	{1, ARMV7M_REGISTER_CORE_GP, ARMV7M_MODE_ANY, NULL, NULL},
	{2, ARMV7M_REGISTER_CORE_GP, ARMV7M_MODE_ANY, NULL, NULL},
	{3, ARMV7M_REGISTER_CORE_GP, ARMV7M_MODE_ANY, NULL, NULL},
	{4, ARMV7M_REGISTER_CORE_GP, ARMV7M_MODE_ANY, NULL, NULL},
	{5, ARMV7M_REGISTER_CORE_GP, ARMV7M_MODE_ANY, NULL, NULL},
	{6, ARMV7M_REGISTER_CORE_GP, ARMV7M_MODE_ANY, NULL, NULL},
	{7, ARMV7M_REGISTER_CORE_GP, ARMV7M_MODE_ANY, NULL, NULL},
	{8, ARMV7M_REGISTER_CORE_GP, ARMV7M_MODE_ANY, NULL, NULL},
	{9, ARMV7M_REGISTER_CORE_GP, ARMV7M_MODE_ANY, NULL, NULL},
	{10, ARMV7M_REGISTER_CORE_GP, ARMV7M_MODE_ANY, NULL, NULL},
	{11, ARMV7M_REGISTER_CORE_GP, ARMV7M_MODE_ANY, NULL, NULL},
	{12, ARMV7M_REGISTER_CORE_GP, ARMV7M_MODE_ANY, NULL, NULL},
	{13, ARMV7M_REGISTER_CORE_GP, ARMV7M_MODE_ANY, NULL, NULL},
	{14, ARMV7M_REGISTER_CORE_GP, ARMV7M_MODE_ANY, NULL, NULL},	
	{15, ARMV7M_REGISTER_CORE_GP, ARMV7M_MODE_ANY, NULL, NULL},

	{16, ARMV7M_REGISTER_CORE_GP, ARMV7M_MODE_ANY, NULL, NULL}, /* xPSR */
	{17, ARMV7M_REGISTER_CORE_GP, ARMV7M_MODE_ANY, NULL, NULL}, /* MSP */
	{18, ARMV7M_REGISTER_CORE_GP, ARMV7M_MODE_ANY, NULL, NULL}, /* PSP */

	/*  CORE_SP are accesible using MSR and MRS instructions */
//	{0x00, ARMV7M_REGISTER_CORE_SP, ARMV7M_MODE_ANY, NULL, NULL}, /* APSR */
//	{0x01, ARMV7M_REGISTER_CORE_SP, ARMV7M_MODE_ANY, NULL, NULL}, /* IAPSR */
//	{0x05, ARMV7M_REGISTER_CORE_SP, ARMV7M_MODE_ANY, NULL, NULL}, /* IPSR */
//	{0x06, ARMV7M_REGISTER_CORE_SP, ARMV7M_MODE_ANY, NULL, NULL}, /* EPSR */

	{0x10, ARMV7M_REGISTER_CORE_SP, ARMV7M_MODE_ANY, NULL, NULL}, /* PRIMASK */
	{0x11, ARMV7M_REGISTER_CORE_SP, ARMV7M_MODE_ANY, NULL, NULL}, /* BASEPRI */
	{0x13, ARMV7M_REGISTER_CORE_SP, ARMV7M_MODE_ANY, NULL, NULL}, /* FAULTMASK */
	{0x14, ARMV7M_REGISTER_CORE_SP, ARMV7M_MODE_ANY, NULL, NULL}  /* CONTROL */
};

int armv7m_core_reg_arch_type = -1;


/* Keep different contexts for the process being debugged and debug algorithms */
enum armv7m_runcontext armv7m_get_context(target_t *target)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	
	if (armv7m->process_context == armv7m->core_cache)
		return ARMV7M_PROCESS_CONTEXT;
	if (armv7m->debug_context == armv7m->core_cache)
		return ARMV7M_DEBUG_CONTEXT;
	
	ERROR("Invalid runcontext");
	exit(-1);
}

int armv7m_use_context(target_t *target, enum armv7m_runcontext new_ctx)
{
	int i;
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	
	if ((target->state != TARGET_HALTED) && (target->state != TARGET_RESET))
	{
		WARNING("target not halted, switch context ");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (new_ctx == armv7m_get_context(target))
		return ERROR_OK;
		
	switch (new_ctx)
	{
		case ARMV7M_PROCESS_CONTEXT:
			 armv7m->core_cache = armv7m->process_context;
			 break;
		case ARMV7M_DEBUG_CONTEXT:
			 armv7m->core_cache = armv7m->debug_context;
			 break;
		default:
			ERROR("Invalid runcontext");
			exit(-1);		
	} 
	/* Mark registers in new context as dirty to force reload when run */
	
	for (i = 0; i < armv7m->core_cache->num_regs-1; i++) /* EXCLUDE CONTROL TODOLATER : CHECK THIS */
	{
		armv7m->core_cache->reg_list[i].dirty = 1;
	}
	
	return ERROR_OK;
}

/* Core state functions */
char enamebuf[32];
char *armv7m_exception_string(int number)
{
	if ((number < 0) | (number > 511))
		return "Invalid exception";
	if (number < 16)
		return armv7m_exception_strings[number];
	sprintf(enamebuf, "External Interrupt(%i)", number - 16);
	return enamebuf;
}

int armv7m_get_core_reg(reg_t *reg)
{
	int retval;
	armv7m_core_reg_t *armv7m_reg = reg->arch_info;
	target_t *target = armv7m_reg->target;
	armv7m_common_t *armv7m_target = target->arch_info;
	
	if (target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}

	retval = armv7m_target->read_core_reg(target, armv7m_reg->num);
	
	return retval;
}

int armv7m_set_core_reg(reg_t *reg, u8 *buf)
{
	armv7m_core_reg_t *armv7m_reg = reg->arch_info;
	target_t *target = armv7m_reg->target;
	armv7m_common_t *armv7m_target = target->arch_info;
	u32 value = buf_get_u32(buf, 0, 32);
		
	if (target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}
		
	buf_set_u32(reg->value, 0, 32, value);
	reg->dirty = 1;
	reg->valid = 1;

	return ERROR_OK;
}

int armv7m_read_core_reg(struct target_s *target, int num)
{
	u32 reg_value;
	int retval;
	armv7m_core_reg_t * armv7m_core_reg;
	
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
		
	if ((num < 0) || (num >= ARMV7NUMCOREREGS))
		return ERROR_INVALID_ARGUMENTS;

	armv7m_core_reg = armv7m->core_cache->reg_list[num].arch_info;
	retval = armv7m->load_core_reg_u32(target, armv7m_core_reg->type, armv7m_core_reg->num, &reg_value);
	buf_set_u32(armv7m->core_cache->reg_list[num].value, 0, 32, reg_value);
	armv7m->core_cache->reg_list[num].valid=1;
	armv7m->core_cache->reg_list[num].dirty=0;
		
	return ERROR_OK;	
}

int armv7m_write_core_reg(struct target_s *target, int num)
{
	int retval;
	u32 reg_value;
	armv7m_core_reg_t * armv7m_core_reg;
	
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;

	if ((num < 0) || (num >= ARMV7NUMCOREREGS))
		return ERROR_INVALID_ARGUMENTS;
	
	reg_value = buf_get_u32(armv7m->core_cache->reg_list[num].value, 0, 32);
	armv7m_core_reg = armv7m->core_cache->reg_list[num].arch_info;
	retval = armv7m->store_core_reg_u32(target, armv7m_core_reg->type, armv7m_core_reg->num, reg_value);
	if (retval != ERROR_OK)
	{
		ERROR("JTAG failure");
		armv7m->core_cache->reg_list[num].dirty=1;
		return ERROR_JTAG_DEVICE_ERROR;
	}
	DEBUG("write core reg %i value 0x%x",num ,reg_value);
	armv7m->core_cache->reg_list[num].valid=1;
	armv7m->core_cache->reg_list[num].dirty=0;
	
	return ERROR_OK;
}

int armv7m_invalidate_core_regs(target_t *target)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	int i;
	
	for (i = 0; i < armv7m->core_cache->num_regs; i++)
	{
		armv7m->core_cache->reg_list[i].valid = 0;
		armv7m->core_cache->reg_list[i].dirty = 0;
	}
	
	return ERROR_OK;
}

int armv7m_get_gdb_reg_list(target_t *target, reg_t **reg_list[], int *reg_list_size)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	int i;
	
	if (target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}
	
	*reg_list_size = 26;
	*reg_list = malloc(sizeof(reg_t*) * (*reg_list_size));
	
	/* TODOLATER correct list of registers, names ? */
	for (i = 0; i < *reg_list_size; i++)
	{
		if (i < ARMV7NUMCOREREGS)
			(*reg_list)[i] = &armv7m->process_context->reg_list[i];
			//(*reg_list)[i] = &armv7m->core_cache->reg_list[i];
		else
			(*reg_list)[i] = &armv7m_gdb_dummy_fp_reg;
	}
	/* ARMV7M is always in thumb mode, try to make GDB understand this if it does not support this arch */
	//armv7m->core_cache->reg_list[15].value[0] |= 1;	
	armv7m->process_context->reg_list[15].value[0] |= 1;	
	//armv7m->core_cache->reg_list[ARMV7M_xPSR].value[0] = (1<<5);
	//armv7m->process_context->reg_list[ARMV7M_xPSR].value[0] = (1<<5);
	(*reg_list)[25] = &armv7m->process_context->reg_list[ARMV7M_xPSR];	
	//(*reg_list)[25] = &armv7m->process_context->reg_list[ARMV7M_xPSR];	
	return ERROR_OK;
}

int armv7m_run_algorithm(struct target_s *target, int num_mem_params, mem_param_t *mem_params, int num_reg_params, reg_param_t *reg_params, u32 entry_point, u32 exit_point, int timeout_ms, void *arch_info)
{
	// get pointers to arch-specific information
	armv7m_common_t *armv7m = target->arch_info;
	armv7m_algorithm_t *armv7m_algorithm_info = arch_info;
	enum armv7m_state core_state = armv7m->core_state;
	enum armv7m_mode core_mode = armv7m->core_mode;
	int retval = ERROR_OK;
	u32 pc;
	int exit_breakpoint_size = 0;
	int i;
		
	armv7m->core_state = core_state;
	armv7m->core_mode = core_mode;

	if (armv7m_algorithm_info->common_magic != ARMV7M_COMMON_MAGIC)
	{
		ERROR("current target isn't an ARMV7M target");
		return ERROR_TARGET_INVALID;
	}
	
	if (target->state != TARGET_HALTED)
	{
		WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	/* refresh core register cache */
	/* Not needed if core register cache is always consistent with target process state */ 
	armv7m_use_context(target, ARMV7M_DEBUG_CONTEXT);
	
	for (i = 0; i < num_mem_params; i++)
	{
		target_write_buffer(target, mem_params[i].address, mem_params[i].size, mem_params[i].value);
	}
	
	for (i = 0; i < num_reg_params; i++)
	{
		reg_t *reg = register_get_by_name(armv7m->core_cache, reg_params[i].reg_name, 0);
		//reg_t *reg = register_get_by_name(armv7m->debug_context, reg_params[i].reg_name, 0);
		//armv7m_core_reg_t * armv7m_core_reg = reg->arch_info;
		u32 regvalue;
		
		if (!reg)
		{
			ERROR("BUG: register '%s' not found", reg_params[i].reg_name);
			exit(-1);
		}
		
		if (reg->size != reg_params[i].size)
		{
			ERROR("BUG: register '%s' size doesn't match reg_params[i].size", reg_params[i].reg_name);
			exit(-1);
		}
		
		regvalue = buf_get_u32(reg_params[i].value, 0, 32);
		//armv7m->store_core_reg_u32(target, armv7m_core_reg->type, armv7m_core_reg->num, regvalue);
		armv7m_set_core_reg(reg, reg_params[i].value);
	}
	
	/* ARMV7M always runs in Thumb state */
	exit_breakpoint_size = 2;
	if ((retval = breakpoint_add(target, exit_point, exit_breakpoint_size, BKPT_SOFT)) != ERROR_OK)
	{
		ERROR("can't add breakpoint to finish algorithm execution");
		return ERROR_TARGET_FAILURE;
	}
	
	/* This code relies on the target specific  resume() and  poll()->debug_entry() 
	sequence to write register values to the processor and the read them back */
	target->type->resume(target, 0, entry_point, 1, 1);
	target->type->poll(target);
	
	while (target->state != TARGET_HALTED)
	{
		usleep(5000);
		target->type->poll(target);
		if ((timeout_ms -= 5) <= 0)
		{
			ERROR("timeout waiting for algorithm to complete, trying to halt target");
			target->type->halt(target);
			timeout_ms = 1000;
			while (target->state != TARGET_HALTED)
			{
				usleep(10000);
				target->type->poll(target);
				if ((timeout_ms -= 10) <= 0)
				{
					ERROR("target didn't reenter debug state, exiting");
					exit(-1);
				}
			}
			armv7m->load_core_reg_u32(target, ARMV7M_REGISTER_CORE_GP, 15, &pc);
			DEBUG("failed algoritm halted at 0x%x ", pc); 
			retval = ERROR_TARGET_TIMEOUT;
		}
	}
	
	breakpoint_remove(target, exit_point);
	
	/* Read memory values to mem_params[] */
	for (i = 0; i < num_mem_params; i++)
	{
		if (mem_params[i].direction != PARAM_OUT)
			target_read_buffer(target, mem_params[i].address, mem_params[i].size, mem_params[i].value);
	}
	
	/* Copy core register values to reg_params[] */
	for (i = 0; i < num_reg_params; i++)
	{
		if (reg_params[i].direction != PARAM_OUT)
		{
			//reg_t *reg = register_get_by_name(armv7m->core_cache, reg_params[i].reg_name, 0);
			reg_t *reg = register_get_by_name(armv7m->debug_context, reg_params[i].reg_name, 0);
			u32 regvalue;
		
			if (!reg)
			{
				ERROR("BUG: register '%s' not found", reg_params[i].reg_name);
				exit(-1);
			}
			
			if (reg->size != reg_params[i].size)
			{
				ERROR("BUG: register '%s' size doesn't match reg_params[i].size", reg_params[i].reg_name);
				exit(-1);
			}
			
			armv7m_core_reg_t *armv7m_core_reg = reg->arch_info;
			//armv7m->load_core_reg_u32(target, armv7m_core_reg->type, armv7m_core_reg->num, &regvalue);
			//buf_set_u32(reg_params[i].value, 0, 32, regvalue);
			buf_set_u32(reg_params[i].value, 0, 32, buf_get_u32(reg->value, 0, 32));
		}
	}
	
	/* Mark all core registers !! except control !! as valid but dirty */
	/* This will be done by armv7m_use_context in resume function */
	//for (i = 0; i < armv7m->core_cache->num_regs-1; i++)
	//{
	//	armv7m->core_cache->reg_list[i].dirty = 1;
	//}

	// ????armv7m->core_state = core_state;
	// ????armv7m->core_mode = core_mode;

	return retval;
}

int armv7m_arch_state(struct target_s *target, char *buf, int buf_size)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	
	snprintf(buf, buf_size,
		 "target halted in %s state due to %s, current mode: %s %s\nxPSR: 0x%8.8x pc: 0x%8.8x",
		 armv7m_state_strings[armv7m->core_state],
		 target_debug_reason_strings[target->debug_reason],
		 armv7m_mode_strings[armv7m->core_mode],
		 armv7m_exception_string(armv7m->exception_number),
		 buf_get_u32(armv7m->core_cache->reg_list[ARMV7M_xPSR].value, 0, 32),
		 buf_get_u32(armv7m->core_cache->reg_list[15].value, 0, 32));
	
	return ERROR_OK;
}

reg_cache_t *armv7m_build_reg_cache(target_t *target)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	arm_jtag_t *jtag_info = &armv7m->jtag_info;

	int num_regs = ARMV7NUMCOREREGS;
	reg_cache_t **cache_p = register_get_last_cache_p(&target->reg_cache);
	reg_cache_t *cache = malloc(sizeof(reg_cache_t));
	reg_t *reg_list = malloc(sizeof(reg_t) * num_regs);
	armv7m_core_reg_t *arch_info = malloc(sizeof(armv7m_core_reg_t) * num_regs);
	int i;
	
	if (armv7m_core_reg_arch_type == -1)
		armv7m_core_reg_arch_type = register_reg_arch_type(armv7m_get_core_reg, armv7m_set_core_reg);
		
	/* Build the process context cache */ 
	cache->name = "arm v7m registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = num_regs;
	(*cache_p) = cache;
	armv7m->core_cache = cache;
	armv7m->process_context = cache;
	
	for (i = 0; i < num_regs; i++)
	{
		arch_info[i] = armv7m_core_reg_list_arch_info[i];
		arch_info[i].target = target;
		arch_info[i].armv7m_common = armv7m;
		reg_list[i].name = armv7m_core_reg_list[i];
		reg_list[i].size = 32;
		reg_list[i].value = calloc(1, 4);
		reg_list[i].dirty = 0;
		reg_list[i].valid = 0;
		reg_list[i].bitfield_desc = NULL;
		reg_list[i].num_bitfields = 0;
		reg_list[i].arch_type = armv7m_core_reg_arch_type;
		reg_list[i].arch_info = &arch_info[i];
	}

	/* Build the debug context cache*/
	cache = malloc(sizeof(reg_cache_t));
	reg_list = malloc(sizeof(reg_t) * num_regs);

	cache->name = "arm v7m debug registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = num_regs;
	armv7m->debug_context = cache;
	armv7m->process_context->next = cache;

	for (i = 0; i < num_regs; i++)
	{
		reg_list[i].name = armv7m_core_dbgreg_list[i];
		reg_list[i].size = 32;
		reg_list[i].value = calloc(1, 4);
		reg_list[i].dirty = 0;
		reg_list[i].valid = 0;
		reg_list[i].bitfield_desc = NULL;
		reg_list[i].num_bitfields = 0;
		reg_list[i].arch_type = armv7m_core_reg_arch_type;
		reg_list[i].arch_info = &arch_info[i];
	}
 
    return cache;
}

int armv7m_init_target(struct command_context_s *cmd_ctx, struct target_s *target)
{
	armv7m_build_reg_cache(target);
	
	return ERROR_OK;
}

int armv7m_init_arch_info(target_t *target, armv7m_common_t *armv7m)
{
	/* register arch-specific functions */
	
	target->arch_info = armv7m;
	armv7m->core_state = ARMV7M_STATE_THUMB;
	armv7m->read_core_reg = armv7m_read_core_reg;
	armv7m->write_core_reg = armv7m_write_core_reg;
	
	return ERROR_OK;
}

int armv7m_register_commands(struct command_context_s *cmd_ctx)
{
	int retval;
	
	return ERROR_OK;
}
