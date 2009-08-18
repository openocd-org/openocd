/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
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
 *                                                                         *
 *	ARMv7-M Architecture, Application Level Reference Manual               *
 *              ARM DDI 0405C (September 2008)                             *
 *                                                                         *
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "armv7m.h"

#define ARRAY_SIZE(x)	((int)(sizeof(x)/sizeof((x)[0])))


#if 0
#define _DEBUG_INSTRUCTION_EXECUTION_
#endif

char* armv7m_mode_strings[] =
{
	"Thread", "Thread (User)", "Handler",
};

static char *armv7m_exception_strings[] =
{
	"", "Reset", "NMI", "HardFault",
	"MemManage", "BusFault", "UsageFault", "RESERVED",
	"RESERVED", "RESERVED", "RESERVED", "SVCall",
	"DebugMonitor", "RESERVED", "PendSV", "SysTick"
};

uint8_t armv7m_gdb_dummy_fp_value[] = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0};

reg_t armv7m_gdb_dummy_fp_reg =
{
	"GDB dummy floating-point register", armv7m_gdb_dummy_fp_value, 0, 1, 96, NULL, 0, NULL, 0
};

uint8_t armv7m_gdb_dummy_fps_value[] = {0, 0, 0, 0};

reg_t armv7m_gdb_dummy_fps_reg =
{
	"GDB dummy floating-point status register", armv7m_gdb_dummy_fps_value, 0, 1, 32, NULL, 0, NULL, 0
};

#ifdef ARMV7_GDB_HACKS
uint8_t armv7m_gdb_dummy_cpsr_value[] = {0, 0, 0, 0};

reg_t armv7m_gdb_dummy_cpsr_reg =
{
	"GDB dummy cpsr register", armv7m_gdb_dummy_cpsr_value, 0, 1, 32, NULL, 0, NULL, 0
};
#endif

/*
 * These registers are not memory-mapped.  The ARMv7-M profile includes
 * memory mapped registers too, such as for the NVIC (interrupt controller)
 * and SysTick (timer) modules; those can mostly be treated as peripherals.
 */
static const struct {
	unsigned id;
	char *name;
	unsigned bits;
} armv7m_regs[] = {
	{ ARMV7M_R0, "r0", 32 },
	{ ARMV7M_R1, "r1", 32 },
	{ ARMV7M_R2, "r2", 32 },
	{ ARMV7M_R3, "r3", 32 },

	{ ARMV7M_R4, "r4", 32 },
	{ ARMV7M_R5, "r5", 32 },
	{ ARMV7M_R6, "r6", 32 },
	{ ARMV7M_R7, "r7", 32 },

	{ ARMV7M_R8, "r8", 32 },
	{ ARMV7M_R9, "r9", 32 },
	{ ARMV7M_R10, "r10", 32 },
	{ ARMV7M_R11, "r11", 32 },

	{ ARMV7M_R12, "r12", 32 },
	{ ARMV7M_R13, "sp", 32 },
	{ ARMV7M_R14, "lr", 32 },
	{ ARMV7M_PC, "pc", 32 },

	{ ARMV7M_xPSR, "xPSR", 32 },
	{ ARMV7M_MSP, "msp", 32 },
	{ ARMV7M_PSP, "psp", 32 },

	{ ARMV7M_PRIMASK, "primask", 1 },
	{ ARMV7M_BASEPRI, "basepri", 8 },
	{ ARMV7M_FAULTMASK, "faultmask", 1 },
	{ ARMV7M_CONTROL, "control", 2 },
};

#define ARMV7M_NUM_REGS	ARRAY_SIZE(armv7m_regs)

int armv7m_core_reg_arch_type = -1;
int armv7m_dummy_core_reg_arch_type = -1;

int armv7m_restore_context(target_t *target)
{
	int i;

	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;

	LOG_DEBUG(" ");

	if (armv7m->pre_restore_context)
		armv7m->pre_restore_context(target);

	for (i = ARMV7M_NUM_REGS - 1; i >= 0; i--)
	{
		if (armv7m->core_cache->reg_list[i].dirty)
		{
			armv7m->write_core_reg(target, i);
		}
	}

	if (armv7m->post_restore_context)
		armv7m->post_restore_context(target);

	return ERROR_OK;
}

/* Core state functions */
char *armv7m_exception_string(int number)
{
	static char enamebuf[32];

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

int armv7m_set_core_reg(reg_t *reg, uint8_t *buf)
{
	armv7m_core_reg_t *armv7m_reg = reg->arch_info;
	target_t *target = armv7m_reg->target;
	uint32_t value = buf_get_u32(buf, 0, 32);

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
	uint32_t reg_value;
	int retval;
	armv7m_core_reg_t * armv7m_core_reg;

	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;

	if ((num < 0) || (num >= ARMV7M_NUM_REGS))
		return ERROR_INVALID_ARGUMENTS;

	armv7m_core_reg = armv7m->core_cache->reg_list[num].arch_info;
	retval = armv7m->load_core_reg_u32(target, armv7m_core_reg->type, armv7m_core_reg->num, &reg_value);
	buf_set_u32(armv7m->core_cache->reg_list[num].value, 0, 32, reg_value);
	armv7m->core_cache->reg_list[num].valid = 1;
	armv7m->core_cache->reg_list[num].dirty = 0;

	return retval;
}

int armv7m_write_core_reg(struct target_s *target, int num)
{
	int retval;
	uint32_t reg_value;
	armv7m_core_reg_t *armv7m_core_reg;

	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;

	if ((num < 0) || (num >= ARMV7M_NUM_REGS))
		return ERROR_INVALID_ARGUMENTS;

	reg_value = buf_get_u32(armv7m->core_cache->reg_list[num].value, 0, 32);
	armv7m_core_reg = armv7m->core_cache->reg_list[num].arch_info;
	retval = armv7m->store_core_reg_u32(target, armv7m_core_reg->type, armv7m_core_reg->num, reg_value);
	if (retval != ERROR_OK)
	{
		LOG_ERROR("JTAG failure");
		armv7m->core_cache->reg_list[num].dirty = armv7m->core_cache->reg_list[num].valid;
		return ERROR_JTAG_DEVICE_ERROR;
	}
	LOG_DEBUG("write core reg %i value 0x%" PRIx32 "", num , reg_value);
	armv7m->core_cache->reg_list[num].valid = 1;
	armv7m->core_cache->reg_list[num].dirty = 0;

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

	*reg_list_size = 26;
	*reg_list = malloc(sizeof(reg_t*) * (*reg_list_size));

	/*
	 * GDB register packet format for ARM:
	 *  - the first 16 registers are r0..r15
	 *  - (obsolete) 8 FPA registers
	 *  - (obsolete) FPA status
	 *  - CPSR
	 */
	for (i = 0; i < 16; i++)
	{
		(*reg_list)[i] = &armv7m->core_cache->reg_list[i];
	}

	for (i = 16; i < 24; i++)
	{
		(*reg_list)[i] = &armv7m_gdb_dummy_fp_reg;
	}

	(*reg_list)[24] = &armv7m_gdb_dummy_fps_reg;

#ifdef ARMV7_GDB_HACKS
	/* use dummy cpsr reg otherwise gdb may try and set the thumb bit */
	(*reg_list)[25] = &armv7m_gdb_dummy_cpsr_reg;

	/* ARMV7M is always in thumb mode, try to make GDB understand this
	 * if it does not support this arch */
	*((char*)armv7m->core_cache->reg_list[15].value) |= 1;
#else
	(*reg_list)[25] = &armv7m->core_cache->reg_list[ARMV7M_xPSR];
#endif

	return ERROR_OK;
}

/* run to exit point. return error if exit point was not reached. */
static int armv7m_run_and_wait(struct target_s *target, uint32_t entry_point, int timeout_ms, uint32_t exit_point, armv7m_common_t *armv7m)
{
	uint32_t pc;
	int retval;
	/* This code relies on the target specific  resume() and  poll()->debug_entry()
	 * sequence to write register values to the processor and the read them back */
	if ((retval = target_resume(target, 0, entry_point, 1, 1)) != ERROR_OK)
	{
		return retval;
	}

	retval = target_wait_state(target, TARGET_HALTED, timeout_ms);
	/* If the target fails to halt due to the breakpoint, force a halt */
	if (retval != ERROR_OK || target->state != TARGET_HALTED)
	{
		if ((retval = target_halt(target)) != ERROR_OK)
			return retval;
		if ((retval = target_wait_state(target, TARGET_HALTED, 500)) != ERROR_OK)
		{
			return retval;
		}
		return ERROR_TARGET_TIMEOUT;
	}

	armv7m->load_core_reg_u32(target, ARMV7M_REGISTER_CORE_GP, 15, &pc);
	if (pc != exit_point)
	{
		LOG_DEBUG("failed algoritm halted at 0x%" PRIx32 " ", pc);
		return ERROR_TARGET_TIMEOUT;
	}

	return ERROR_OK;
}

int armv7m_run_algorithm(struct target_s *target, int num_mem_params, mem_param_t *mem_params, int num_reg_params, reg_param_t *reg_params, uint32_t entry_point, uint32_t exit_point, int timeout_ms, void *arch_info)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	armv7m_algorithm_t *armv7m_algorithm_info = arch_info;
	enum armv7m_mode core_mode = armv7m->core_mode;
	int retval = ERROR_OK;
	int i;
	uint32_t context[ARMV7M_NUM_REGS];

	if (armv7m_algorithm_info->common_magic != ARMV7M_COMMON_MAGIC)
	{
		LOG_ERROR("current target isn't an ARMV7M target");
		return ERROR_TARGET_INVALID;
	}

	if (target->state != TARGET_HALTED)
	{
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* refresh core register cache */
	/* Not needed if core register cache is always consistent with target process state */
	for (i = 0; i < ARMV7M_NUM_REGS; i++)
	{
		if (!armv7m->core_cache->reg_list[i].valid)
			armv7m->read_core_reg(target, i);
		context[i] = buf_get_u32(armv7m->core_cache->reg_list[i].value, 0, 32);
	}

	for (i = 0; i < num_mem_params; i++)
	{
		if ((retval = target_write_buffer(target, mem_params[i].address, mem_params[i].size, mem_params[i].value)) != ERROR_OK)
			return retval;
	}

	for (i = 0; i < num_reg_params; i++)
	{
		reg_t *reg = register_get_by_name(armv7m->core_cache, reg_params[i].reg_name, 0);
//		uint32_t regvalue;

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

//		regvalue = buf_get_u32(reg_params[i].value, 0, 32);
		armv7m_set_core_reg(reg, reg_params[i].value);
	}

	if (armv7m_algorithm_info->core_mode != ARMV7M_MODE_ANY)
	{
		LOG_DEBUG("setting core_mode: 0x%2.2x", armv7m_algorithm_info->core_mode);
		buf_set_u32(armv7m->core_cache->reg_list[ARMV7M_CONTROL].value,
				0, 1, armv7m_algorithm_info->core_mode);
		armv7m->core_cache->reg_list[ARMV7M_CONTROL].dirty = 1;
		armv7m->core_cache->reg_list[ARMV7M_CONTROL].valid = 1;
	}

	/* ARMV7M always runs in Thumb state */
	if ((retval = breakpoint_add(target, exit_point, 2, BKPT_SOFT)) != ERROR_OK)
	{
		LOG_ERROR("can't add breakpoint to finish algorithm execution");
		return ERROR_TARGET_FAILURE;
	}

	retval = armv7m_run_and_wait(target, entry_point, timeout_ms, exit_point, armv7m);

	breakpoint_remove(target, exit_point);

	if (retval != ERROR_OK)
	{
		return retval;
	}

	/* Read memory values to mem_params[] */
	for (i = 0; i < num_mem_params; i++)
	{
		if (mem_params[i].direction != PARAM_OUT)
			if ((retval = target_read_buffer(target, mem_params[i].address, mem_params[i].size, mem_params[i].value)) != ERROR_OK)
			{
				return retval;
			}
	}

	/* Copy core register values to reg_params[] */
	for (i = 0; i < num_reg_params; i++)
	{
		if (reg_params[i].direction != PARAM_OUT)
		{
			reg_t *reg = register_get_by_name(armv7m->core_cache, reg_params[i].reg_name, 0);

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

	for (i = ARMV7M_NUM_REGS - 1; i >= 0; i--)
	{
		uint32_t regvalue;
		regvalue = buf_get_u32(armv7m->core_cache->reg_list[i].value, 0, 32);
		if (regvalue != context[i])
		{
			LOG_DEBUG("restoring register %s with value 0x%8.8" PRIx32,
				armv7m->core_cache->reg_list[i].name, context[i]);
			buf_set_u32(armv7m->core_cache->reg_list[i].value,
					0, 32, context[i]);
			armv7m->core_cache->reg_list[i].valid = 1;
			armv7m->core_cache->reg_list[i].dirty = 1;
		}
	}

	armv7m->core_mode = core_mode;

	return retval;
}

int armv7m_arch_state(struct target_s *target)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;
	uint32_t ctrl, sp;

	ctrl = buf_get_u32(armv7m->core_cache->reg_list[ARMV7M_CONTROL].value, 0, 32);
	sp = buf_get_u32(armv7m->core_cache->reg_list[ARMV7M_R13].value, 0, 32);

	LOG_USER("target halted due to %s, current mode: %s %s\n"
		"xPSR: %#8.8" PRIx32 " pc: %#8.8" PRIx32 " %csp: %#8.8" PRIx32,
		Jim_Nvp_value2name_simple(nvp_target_debug_reason,
				target->debug_reason)->name,
		armv7m_mode_strings[armv7m->core_mode],
		armv7m_exception_string(armv7m->exception_number),
		buf_get_u32(armv7m->core_cache->reg_list[ARMV7M_xPSR].value, 0, 32),
		buf_get_u32(armv7m->core_cache->reg_list[ARMV7M_PC].value, 0, 32),
		(ctrl & 0x02) ? 'p' : 'm',
		sp);

	return ERROR_OK;
}

reg_cache_t *armv7m_build_reg_cache(target_t *target)
{
	/* get pointers to arch-specific information */
	armv7m_common_t *armv7m = target->arch_info;

	int num_regs = ARMV7M_NUM_REGS;
	reg_cache_t **cache_p = register_get_last_cache_p(&target->reg_cache);
	reg_cache_t *cache = malloc(sizeof(reg_cache_t));
	reg_t *reg_list = calloc(num_regs, sizeof(reg_t));
	armv7m_core_reg_t *arch_info = calloc(num_regs, sizeof(armv7m_core_reg_t));
	int i;

	if (armv7m_core_reg_arch_type == -1)
	{
		armv7m_core_reg_arch_type = register_reg_arch_type(armv7m_get_core_reg, armv7m_set_core_reg);
	}

	register_init_dummy(&armv7m_gdb_dummy_fps_reg);
#ifdef ARMV7_GDB_HACKS
	register_init_dummy(&armv7m_gdb_dummy_cpsr_reg);
#endif
	register_init_dummy(&armv7m_gdb_dummy_fp_reg);

	/* Build the process context cache */
	cache->name = "arm v7m registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = num_regs;
	(*cache_p) = cache;
	armv7m->core_cache = cache;

	for (i = 0; i < num_regs; i++)
	{
		arch_info[i].num = armv7m_regs[i].id;
		arch_info[i].target = target;
		arch_info[i].armv7m_common = armv7m;
		reg_list[i].name = armv7m_regs[i].name;
		reg_list[i].size = armv7m_regs[i].bits;
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
	armv7m->read_core_reg = armv7m_read_core_reg;
	armv7m->write_core_reg = armv7m_write_core_reg;

	return ERROR_OK;
}

int armv7m_checksum_memory(struct target_s *target, uint32_t address, uint32_t count, uint32_t* checksum)
{
	working_area_t *crc_algorithm;
	armv7m_algorithm_t armv7m_info;
	reg_param_t reg_params[2];
	int retval;

	uint16_t cortex_m3_crc_code[] = {
		0x4602,					/* mov	r2, r0 */
		0xF04F, 0x30FF,			/* mov	r0, #0xffffffff */
		0x460B,					/* mov	r3, r1 */
		0xF04F, 0x0400,			/* mov	r4, #0 */
		0xE013,					/* b	ncomp */
								/* nbyte: */
		0x5D11,					/* ldrb	r1, [r2, r4] */
		0xF8DF, 0x7028,			/* ldr		r7, CRC32XOR */
		0xEA80, 0x6001,			/* eor		r0, r0, r1, asl #24 */

		0xF04F, 0x0500,			/* mov		r5, #0 */
								/* loop: */
		0x2800,					/* cmp		r0, #0 */
		0xEA4F, 0x0640,			/* mov		r6, r0, asl #1 */
		0xF105, 0x0501,			/* add		r5, r5, #1 */
		0x4630,					/* mov		r0, r6 */
		0xBFB8,					/* it		lt */
		0xEA86, 0x0007,			/* eor		r0, r6, r7 */
		0x2D08, 				/* cmp		r5, #8 */
		0xD1F4,					/* bne		loop */

		0xF104, 0x0401,			/* add	r4, r4, #1 */
								/* ncomp: */
		0x429C,					/* cmp	r4, r3 */
		0xD1E9,					/* bne	nbyte */
								/* end: */
		0xE7FE,					/* b	end */
		0x1DB7, 0x04C1			/* CRC32XOR:	.word 0x04C11DB7 */
	};

	uint32_t i;

	if (target_alloc_working_area(target, sizeof(cortex_m3_crc_code), &crc_algorithm) != ERROR_OK)
	{
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* convert flash writing code into a buffer in target endianness */
	for (i = 0; i < (sizeof(cortex_m3_crc_code)/sizeof(uint16_t)); i++)
		if ((retval = target_write_u16(target, crc_algorithm->address + i*sizeof(uint16_t), cortex_m3_crc_code[i])) != ERROR_OK)
		{
			return retval;
		}

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARMV7M_MODE_ANY;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);

	buf_set_u32(reg_params[0].value, 0, 32, address);
	buf_set_u32(reg_params[1].value, 0, 32, count);

	if ((retval = target_run_algorithm(target, 0, NULL, 2, reg_params,
		crc_algorithm->address, crc_algorithm->address + (sizeof(cortex_m3_crc_code)-6), 20000, &armv7m_info)) != ERROR_OK)
	{
		LOG_ERROR("error executing cortex_m3 crc algorithm");
		destroy_reg_param(&reg_params[0]);
		destroy_reg_param(&reg_params[1]);
		target_free_working_area(target, crc_algorithm);
		return retval;
	}

	*checksum = buf_get_u32(reg_params[0].value, 0, 32);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);

	target_free_working_area(target, crc_algorithm);

	return ERROR_OK;
}

int armv7m_blank_check_memory(struct target_s *target, uint32_t address, uint32_t count, uint32_t* blank)
{
	working_area_t *erase_check_algorithm;
	reg_param_t reg_params[3];
	armv7m_algorithm_t armv7m_info;
	int retval;
	uint32_t i;

	uint16_t erase_check_code[] =
	{
							/* loop: */
		0xF810, 0x3B01,		/* ldrb 	r3, [r0], #1 */
		0xEA02, 0x0203,		/* and 	r2, r2, r3 */
		0x3901,				/* subs 	r1, r1, #1 */
		0xD1F9,				/* bne		loop */
							/* end: */
		0xE7FE,				/* b		end */
	};

	/* make sure we have a working area */
	if (target_alloc_working_area(target, sizeof(erase_check_code), &erase_check_algorithm) != ERROR_OK)
	{
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* convert flash writing code into a buffer in target endianness */
	for (i = 0; i < (sizeof(erase_check_code)/sizeof(uint16_t)); i++)
		target_write_u16(target, erase_check_algorithm->address + i*sizeof(uint16_t), erase_check_code[i]);

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARMV7M_MODE_ANY;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, address);

	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	buf_set_u32(reg_params[1].value, 0, 32, count);

	init_reg_param(&reg_params[2], "r2", 32, PARAM_IN_OUT);
	buf_set_u32(reg_params[2].value, 0, 32, 0xff);

	if ((retval = target_run_algorithm(target, 0, NULL, 3, reg_params,
			erase_check_algorithm->address, erase_check_algorithm->address + (sizeof(erase_check_code)-2), 10000, &armv7m_info)) != ERROR_OK)
	{
		destroy_reg_param(&reg_params[0]);
		destroy_reg_param(&reg_params[1]);
		destroy_reg_param(&reg_params[2]);
		target_free_working_area(target, erase_check_algorithm);
		return 0;
	}

	*blank = buf_get_u32(reg_params[2].value, 0, 32);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);

	target_free_working_area(target, erase_check_algorithm);

	return ERROR_OK;
}

/*
 * Return the debug ap baseaddress in hexadecimal;
 * no extra output to simplify script processing
 */
static int handle_dap_baseaddr_command(struct command_context_s *cmd_ctx,
		char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	armv7m_common_t *armv7m = target->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;
	uint32_t apsel, apselsave, baseaddr;
	int retval;

	apsel = swjdp->apsel;
	apselsave = swjdp->apsel;
	if (argc > 0)
	{
		apsel = strtoul(args[0], NULL, 0);
	}
	if (apselsave != apsel)
	{
		dap_ap_select(swjdp, apsel);
	}

	dap_ap_read_reg_u32(swjdp, 0xF8, &baseaddr);
	retval = swjdp_transaction_endcheck(swjdp);
	command_print(cmd_ctx, "0x%8.8" PRIx32 "", baseaddr);

	if (apselsave != apsel)
	{
		dap_ap_select(swjdp, apselsave);
	}

	return retval;
}


/*
 * Return the debug ap id in hexadecimal;
 * no extra output to simplify script processing
 */
extern int handle_dap_apid_command(struct command_context_s *cmd_ctx,
		char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	armv7m_common_t *armv7m = target->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;

	return dap_apid_command(cmd_ctx, swjdp, args, argc);
}

static int handle_dap_apsel_command(struct command_context_s *cmd_ctx,
		char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	armv7m_common_t *armv7m = target->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;

	return dap_apsel_command(cmd_ctx, swjdp, args, argc);
}

static int handle_dap_memaccess_command(struct command_context_s *cmd_ctx,
		char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	armv7m_common_t *armv7m = target->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;

	return dap_memaccess_command(cmd_ctx, swjdp, args, argc);
}


static int handle_dap_info_command(struct command_context_s *cmd_ctx,
		char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	armv7m_common_t *armv7m = target->arch_info;
	swjdp_common_t *swjdp = &armv7m->swjdp_info;
	uint32_t apsel;

	apsel =  swjdp->apsel;
	if (argc > 0)
		apsel = strtoul(args[0], NULL, 0);

	return dap_info_command(cmd_ctx, swjdp, apsel);
}

int armv7m_register_commands(struct command_context_s *cmd_ctx)
{
	command_t *arm_adi_v5_dap_cmd;

	arm_adi_v5_dap_cmd = register_command(cmd_ctx, NULL, "dap",
			NULL, COMMAND_ANY,
			"cortex dap specific commands");

	register_command(cmd_ctx, arm_adi_v5_dap_cmd, "info",
			handle_dap_info_command, COMMAND_EXEC,
			"Displays dap info for ap [num],"
			"default currently selected AP");
	register_command(cmd_ctx, arm_adi_v5_dap_cmd, "apsel",
			handle_dap_apsel_command, COMMAND_EXEC,
			"Select a different AP [num] (default 0)");
	register_command(cmd_ctx, arm_adi_v5_dap_cmd, "apid",
			handle_dap_apid_command, COMMAND_EXEC,
			"Displays id reg from AP [num], "
			"default currently selected AP");
	register_command(cmd_ctx, arm_adi_v5_dap_cmd, "baseaddr",
			handle_dap_baseaddr_command, COMMAND_EXEC,
			"Displays debug base address from AP [num],"
			"default currently selected AP");
	register_command(cmd_ctx, arm_adi_v5_dap_cmd, "memaccess",
			handle_dap_memaccess_command, COMMAND_EXEC,
			"set/get number of extra tck for mem-ap "
			"memory bus access [0-255]");

	return ERROR_OK;
}
