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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 *                                                                         *
 *   ARMv7-M Architecture, Application Level Reference Manual              *
 *              ARM DDI 0405C (September 2008)                             *
 *                                                                         *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "breakpoints.h"
#include "armv7m.h"
#include "algorithm.h"
#include "register.h"

#if 0
#define _DEBUG_INSTRUCTION_EXECUTION_
#endif

static char *armv7m_exception_strings[] = {
	"", "Reset", "NMI", "HardFault",
	"MemManage", "BusFault", "UsageFault", "RESERVED",
	"RESERVED", "RESERVED", "RESERVED", "SVCall",
	"DebugMonitor", "RESERVED", "PendSV", "SysTick"
};

/* PSP is used in some thread modes */
const int armv7m_psp_reg_map[ARMV7M_NUM_CORE_REGS] = {
	ARMV7M_R0, ARMV7M_R1, ARMV7M_R2, ARMV7M_R3,
	ARMV7M_R4, ARMV7M_R5, ARMV7M_R6, ARMV7M_R7,
	ARMV7M_R8, ARMV7M_R9, ARMV7M_R10, ARMV7M_R11,
	ARMV7M_R12, ARMV7M_PSP, ARMV7M_R14, ARMV7M_PC,
	ARMV7M_xPSR,
};

/* MSP is used in handler and some thread modes */
const int armv7m_msp_reg_map[ARMV7M_NUM_CORE_REGS] = {
	ARMV7M_R0, ARMV7M_R1, ARMV7M_R2, ARMV7M_R3,
	ARMV7M_R4, ARMV7M_R5, ARMV7M_R6, ARMV7M_R7,
	ARMV7M_R8, ARMV7M_R9, ARMV7M_R10, ARMV7M_R11,
	ARMV7M_R12, ARMV7M_MSP, ARMV7M_R14, ARMV7M_PC,
	ARMV7M_xPSR,
};

/*
 * These registers are not memory-mapped.  The ARMv7-M profile includes
 * memory mapped registers too, such as for the NVIC (interrupt controller)
 * and SysTick (timer) modules; those can mostly be treated as peripherals.
 *
 * The ARMv6-M profile is almost identical in this respect, except that it
 * doesn't include basepri or faultmask registers.
 */
static const struct {
	unsigned id;
	const char *name;
	unsigned bits;
	enum reg_type type;
	const char *group;
	const char *feature;
} armv7m_regs[] = {
	{ ARMV7M_R0, "r0", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R1, "r1", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R2, "r2", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R3, "r3", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R4, "r4", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R5, "r5", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R6, "r6", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R7, "r7", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R8, "r8", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R9, "r9", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R10, "r10", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R11, "r11", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R12, "r12", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R13, "sp", 32, REG_TYPE_DATA_PTR, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_R14, "lr", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_PC, "pc", 32, REG_TYPE_CODE_PTR, "general", "org.gnu.gdb.arm.m-profile" },
	{ ARMV7M_xPSR, "xPSR", 32, REG_TYPE_INT, "general", "org.gnu.gdb.arm.m-profile" },

	{ ARMV7M_MSP, "msp", 32, REG_TYPE_DATA_PTR, "system", "org.gnu.gdb.arm.m-system" },
	{ ARMV7M_PSP, "psp", 32, REG_TYPE_DATA_PTR, "system", "org.gnu.gdb.arm.m-system" },

	{ ARMV7M_PRIMASK, "primask", 1, REG_TYPE_INT8, "system", "org.gnu.gdb.arm.m-system" },
	{ ARMV7M_BASEPRI, "basepri", 8, REG_TYPE_INT8, "system", "org.gnu.gdb.arm.m-system" },
	{ ARMV7M_FAULTMASK, "faultmask", 1, REG_TYPE_INT8, "system", "org.gnu.gdb.arm.m-system" },
	{ ARMV7M_CONTROL, "control", 2, REG_TYPE_INT8, "system", "org.gnu.gdb.arm.m-system" },
};

#define ARMV7M_NUM_REGS ARRAY_SIZE(armv7m_regs)

/**
 * Restores target context using the cache of core registers set up
 * by armv7m_build_reg_cache(), calling optional core-specific hooks.
 */
int armv7m_restore_context(struct target *target)
{
	int i;
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct reg_cache *cache = armv7m->arm.core_cache;

	LOG_DEBUG(" ");

	if (armv7m->pre_restore_context)
		armv7m->pre_restore_context(target);

	for (i = ARMV7M_NUM_REGS - 1; i >= 0; i--) {
		if (cache->reg_list[i].dirty) {
			uint32_t value = buf_get_u32(cache->reg_list[i].value, 0, 32);
			armv7m->arm.write_core_reg(target, &cache->reg_list[i], i, ARM_MODE_ANY, value);
		}
	}

	return ERROR_OK;
}

/* Core state functions */

/**
 * Maps ISR number (from xPSR) to name.
 * Note that while names and meanings for the first sixteen are standardized
 * (with zero not a true exception), external interrupts are only numbered.
 * They are assigned by vendors, which generally assign different numbers to
 * peripherals (such as UART0 or a USB peripheral controller).
 */
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

static int armv7m_get_core_reg(struct reg *reg)
{
	int retval;
	struct arm_reg *armv7m_reg = reg->arch_info;
	struct target *target = armv7m_reg->target;
	struct arm *arm = target_to_arm(target);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	retval = arm->read_core_reg(target, reg, armv7m_reg->num, arm->core_mode);

	return retval;
}

static int armv7m_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct arm_reg *armv7m_reg = reg->arch_info;
	struct target *target = armv7m_reg->target;
	uint32_t value = buf_get_u32(buf, 0, 32);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	buf_set_u32(reg->value, 0, 32, value);
	reg->dirty = 1;
	reg->valid = 1;

	return ERROR_OK;
}

static int armv7m_read_core_reg(struct target *target, struct reg *r,
	int num, enum arm_mode mode)
{
	uint32_t reg_value;
	int retval;
	struct arm_reg *armv7m_core_reg;
	struct armv7m_common *armv7m = target_to_armv7m(target);

	assert(num < (int)armv7m->arm.core_cache->num_regs);

	armv7m_core_reg = armv7m->arm.core_cache->reg_list[num].arch_info;
	retval = armv7m->load_core_reg_u32(target,
			armv7m_core_reg->num, &reg_value);

	buf_set_u32(armv7m->arm.core_cache->reg_list[num].value, 0, 32, reg_value);
	armv7m->arm.core_cache->reg_list[num].valid = 1;
	armv7m->arm.core_cache->reg_list[num].dirty = 0;

	return retval;
}

static int armv7m_write_core_reg(struct target *target, struct reg *r,
	int num, enum arm_mode mode, uint32_t value)
{
	int retval;
	struct arm_reg *armv7m_core_reg;
	struct armv7m_common *armv7m = target_to_armv7m(target);

	assert(num < (int)armv7m->arm.core_cache->num_regs);

	armv7m_core_reg = armv7m->arm.core_cache->reg_list[num].arch_info;
	retval = armv7m->store_core_reg_u32(target,
					    armv7m_core_reg->num,
					    value);
	if (retval != ERROR_OK) {
		LOG_ERROR("JTAG failure");
		armv7m->arm.core_cache->reg_list[num].dirty = armv7m->arm.core_cache->reg_list[num].valid;
		return ERROR_JTAG_DEVICE_ERROR;
	}

	LOG_DEBUG("write core reg %i value 0x%" PRIx32 "", num, value);
	armv7m->arm.core_cache->reg_list[num].valid = 1;
	armv7m->arm.core_cache->reg_list[num].dirty = 0;

	return ERROR_OK;
}

/**
 * Returns generic ARM userspace registers to GDB.
 */
int armv7m_get_gdb_reg_list(struct target *target, struct reg **reg_list[],
		int *reg_list_size, enum target_register_class reg_class)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	int i;

	if (reg_class == REG_CLASS_ALL)
		*reg_list_size = ARMV7M_NUM_REGS;
	else
		*reg_list_size = ARMV7M_NUM_CORE_REGS;

	*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));
	if (*reg_list == NULL)
		return ERROR_FAIL;

	for (i = 0; i < *reg_list_size; i++)
		(*reg_list)[i] = &armv7m->arm.core_cache->reg_list[i];

	return ERROR_OK;
}

/** Runs a Thumb algorithm in the target. */
int armv7m_run_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	uint32_t entry_point, uint32_t exit_point,
	int timeout_ms, void *arch_info)
{
	int retval;

	retval = armv7m_start_algorithm(target,
			num_mem_params, mem_params,
			num_reg_params, reg_params,
			entry_point, exit_point,
			arch_info);

	if (retval == ERROR_OK)
		retval = armv7m_wait_algorithm(target,
				num_mem_params, mem_params,
				num_reg_params, reg_params,
				exit_point, timeout_ms,
				arch_info);

	return retval;
}

/** Starts a Thumb algorithm in the target. */
int armv7m_start_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	uint32_t entry_point, uint32_t exit_point,
	void *arch_info)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct armv7m_algorithm *armv7m_algorithm_info = arch_info;
	enum arm_mode core_mode = armv7m->arm.core_mode;
	int retval = ERROR_OK;

	/* NOTE: armv7m_run_algorithm requires that each algorithm uses a software breakpoint
	 * at the exit point */

	if (armv7m_algorithm_info->common_magic != ARMV7M_COMMON_MAGIC) {
		LOG_ERROR("current target isn't an ARMV7M target");
		return ERROR_TARGET_INVALID;
	}

	if (target->state != TARGET_HALTED) {
		LOG_WARNING("target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* refresh core register cache
	 * Not needed if core register cache is always consistent with target process state */
	for (unsigned i = 0; i < ARMV7M_NUM_REGS; i++) {

		armv7m_algorithm_info->context[i] = buf_get_u32(
				armv7m->arm.core_cache->reg_list[i].value,
				0,
				32);
	}

	for (int i = 0; i < num_mem_params; i++) {
		/* TODO: Write only out params */
		retval = target_write_buffer(target, mem_params[i].address,
				mem_params[i].size,
				mem_params[i].value);
		if (retval != ERROR_OK)
			return retval;
	}

	for (int i = 0; i < num_reg_params; i++) {
		struct reg *reg =
			register_get_by_name(armv7m->arm.core_cache, reg_params[i].reg_name, 0);
/*		uint32_t regvalue; */

		if (!reg) {
			LOG_ERROR("BUG: register '%s' not found", reg_params[i].reg_name);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		if (reg->size != reg_params[i].size) {
			LOG_ERROR("BUG: register '%s' size doesn't match reg_params[i].size",
				reg_params[i].reg_name);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

/*		regvalue = buf_get_u32(reg_params[i].value, 0, 32); */
		armv7m_set_core_reg(reg, reg_params[i].value);
	}

	if (armv7m_algorithm_info->core_mode != ARM_MODE_ANY &&
			armv7m_algorithm_info->core_mode != core_mode) {

		/* we cannot set ARM_MODE_HANDLER, so use ARM_MODE_THREAD instead */
		if (armv7m_algorithm_info->core_mode == ARM_MODE_HANDLER) {
			armv7m_algorithm_info->core_mode = ARM_MODE_THREAD;
			LOG_INFO("ARM_MODE_HANDLER not currently supported, using ARM_MODE_THREAD instead");
		}

		LOG_DEBUG("setting core_mode: 0x%2.2x", armv7m_algorithm_info->core_mode);
		buf_set_u32(armv7m->arm.core_cache->reg_list[ARMV7M_CONTROL].value,
			0, 1, armv7m_algorithm_info->core_mode);
		armv7m->arm.core_cache->reg_list[ARMV7M_CONTROL].dirty = 1;
		armv7m->arm.core_cache->reg_list[ARMV7M_CONTROL].valid = 1;
	}

	/* save previous core mode */
	armv7m_algorithm_info->core_mode = core_mode;

	retval = target_resume(target, 0, entry_point, 1, 1);

	return retval;
}

/** Waits for an algorithm in the target. */
int armv7m_wait_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	uint32_t exit_point, int timeout_ms,
	void *arch_info)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct armv7m_algorithm *armv7m_algorithm_info = arch_info;
	int retval = ERROR_OK;
	uint32_t pc;

	/* NOTE: armv7m_run_algorithm requires that each algorithm uses a software breakpoint
	 * at the exit point */

	if (armv7m_algorithm_info->common_magic != ARMV7M_COMMON_MAGIC) {
		LOG_ERROR("current target isn't an ARMV7M target");
		return ERROR_TARGET_INVALID;
	}

	retval = target_wait_state(target, TARGET_HALTED, timeout_ms);
	/* If the target fails to halt due to the breakpoint, force a halt */
	if (retval != ERROR_OK || target->state != TARGET_HALTED) {
		retval = target_halt(target);
		if (retval != ERROR_OK)
			return retval;
		retval = target_wait_state(target, TARGET_HALTED, 500);
		if (retval != ERROR_OK)
			return retval;
		return ERROR_TARGET_TIMEOUT;
	}

	armv7m->load_core_reg_u32(target, 15, &pc);
	if (exit_point && (pc != exit_point)) {
		LOG_DEBUG("failed algorithm halted at 0x%" PRIx32 ", expected 0x%" PRIx32,
			pc,
			exit_point);
		return ERROR_TARGET_TIMEOUT;
	}

	/* Read memory values to mem_params[] */
	for (int i = 0; i < num_mem_params; i++) {
		if (mem_params[i].direction != PARAM_OUT) {
			retval = target_read_buffer(target, mem_params[i].address,
					mem_params[i].size,
					mem_params[i].value);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	/* Copy core register values to reg_params[] */
	for (int i = 0; i < num_reg_params; i++) {
		if (reg_params[i].direction != PARAM_OUT) {
			struct reg *reg = register_get_by_name(armv7m->arm.core_cache,
					reg_params[i].reg_name,
					0);

			if (!reg) {
				LOG_ERROR("BUG: register '%s' not found", reg_params[i].reg_name);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}

			if (reg->size != reg_params[i].size) {
				LOG_ERROR(
					"BUG: register '%s' size doesn't match reg_params[i].size",
					reg_params[i].reg_name);
				return ERROR_COMMAND_SYNTAX_ERROR;
			}

			buf_set_u32(reg_params[i].value, 0, 32, buf_get_u32(reg->value, 0, 32));
		}
	}

	for (int i = ARMV7M_NUM_REGS - 1; i >= 0; i--) {
		uint32_t regvalue;
		regvalue = buf_get_u32(armv7m->arm.core_cache->reg_list[i].value, 0, 32);
		if (regvalue != armv7m_algorithm_info->context[i]) {
			LOG_DEBUG("restoring register %s with value 0x%8.8" PRIx32,
					armv7m->arm.core_cache->reg_list[i].name,
				armv7m_algorithm_info->context[i]);
			buf_set_u32(armv7m->arm.core_cache->reg_list[i].value,
				0, 32, armv7m_algorithm_info->context[i]);
			armv7m->arm.core_cache->reg_list[i].valid = 1;
			armv7m->arm.core_cache->reg_list[i].dirty = 1;
		}
	}

	/* restore previous core mode */
	if (armv7m_algorithm_info->core_mode != armv7m->arm.core_mode) {
		LOG_DEBUG("restoring core_mode: 0x%2.2x", armv7m_algorithm_info->core_mode);
		buf_set_u32(armv7m->arm.core_cache->reg_list[ARMV7M_CONTROL].value,
			0, 1, armv7m_algorithm_info->core_mode);
		armv7m->arm.core_cache->reg_list[ARMV7M_CONTROL].dirty = 1;
		armv7m->arm.core_cache->reg_list[ARMV7M_CONTROL].valid = 1;
	}

	armv7m->arm.core_mode = armv7m_algorithm_info->core_mode;

	return retval;
}

/** Logs summary of ARMv7-M state for a halted target. */
int armv7m_arch_state(struct target *target)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct arm *arm = &armv7m->arm;
	uint32_t ctrl, sp;

	ctrl = buf_get_u32(arm->core_cache->reg_list[ARMV7M_CONTROL].value, 0, 32);
	sp = buf_get_u32(arm->core_cache->reg_list[ARMV7M_R13].value, 0, 32);

	LOG_USER("target halted due to %s, current mode: %s %s\n"
		"xPSR: %#8.8" PRIx32 " pc: %#8.8" PRIx32 " %csp: %#8.8" PRIx32 "%s",
		debug_reason_name(target),
		arm_mode_name(arm->core_mode),
		armv7m_exception_string(armv7m->exception_number),
		buf_get_u32(arm->cpsr->value, 0, 32),
		buf_get_u32(arm->pc->value, 0, 32),
		(ctrl & 0x02) ? 'p' : 'm',
		sp,
		arm->is_semihosting ? ", semihosting" : "");

	return ERROR_OK;
}

static const struct reg_arch_type armv7m_reg_type = {
	.get = armv7m_get_core_reg,
	.set = armv7m_set_core_reg,
};

/** Builds cache of architecturally defined registers.  */
struct reg_cache *armv7m_build_reg_cache(struct target *target)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct arm *arm = &armv7m->arm;
	int num_regs = ARMV7M_NUM_REGS;
	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = calloc(num_regs, sizeof(struct reg));
	struct arm_reg *arch_info = calloc(num_regs, sizeof(struct arm_reg));
	struct reg_feature *feature;
	int i;

	/* Build the process context cache */
	cache->name = "arm v7m registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = num_regs;
	(*cache_p) = cache;

	for (i = 0; i < num_regs; i++) {
		arch_info[i].num = armv7m_regs[i].id;
		arch_info[i].target = target;
		arch_info[i].arm = arm;

		reg_list[i].name = armv7m_regs[i].name;
		reg_list[i].size = armv7m_regs[i].bits;
		reg_list[i].value = calloc(1, 4);
		reg_list[i].dirty = 0;
		reg_list[i].valid = 0;
		reg_list[i].type = &armv7m_reg_type;
		reg_list[i].arch_info = &arch_info[i];

		reg_list[i].group = armv7m_regs[i].group;
		reg_list[i].number = i;
		reg_list[i].exist = true;
		reg_list[i].caller_save = true;	/* gdb defaults to true */

		feature = calloc(1, sizeof(struct reg_feature));
		if (feature) {
			feature->name = armv7m_regs[i].feature;
			reg_list[i].feature = feature;
		} else
			LOG_ERROR("unable to allocate feature list");

		reg_list[i].reg_data_type = calloc(1, sizeof(struct reg_data_type));
		if (reg_list[i].reg_data_type)
			reg_list[i].reg_data_type->type = armv7m_regs[i].type;
		else
			LOG_ERROR("unable to allocate reg type list");
	}

	arm->cpsr = reg_list + ARMV7M_xPSR;
	arm->pc = reg_list + ARMV7M_PC;
	arm->core_cache = cache;

	return cache;
}

static int armv7m_setup_semihosting(struct target *target, int enable)
{
	/* nothing todo for armv7m */
	return ERROR_OK;
}

/** Sets up target as a generic ARMv7-M core */
int armv7m_init_arch_info(struct target *target, struct armv7m_common *armv7m)
{
	struct arm *arm = &armv7m->arm;

	armv7m->common_magic = ARMV7M_COMMON_MAGIC;
	armv7m->fp_feature = FP_NONE;

	arm->core_type = ARM_MODE_THREAD;
	arm->arch_info = armv7m;
	arm->setup_semihosting = armv7m_setup_semihosting;

	arm->read_core_reg = armv7m_read_core_reg;
	arm->write_core_reg = armv7m_write_core_reg;

	return arm_init_arch_info(target, arm);
}

/** Generates a CRC32 checksum of a memory region. */
int armv7m_checksum_memory(struct target *target,
	uint32_t address, uint32_t count, uint32_t *checksum)
{
	struct working_area *crc_algorithm;
	struct armv7m_algorithm armv7m_info;
	struct reg_param reg_params[2];
	int retval;

	/* see contrib/loaders/checksum/armv7m_crc.s for src */

	static const uint8_t cortex_m_crc_code[] = {
		/* main: */
		0x02, 0x46,			/* mov		r2, r0 */
		0x00, 0x20,			/* movs		r0, #0 */
		0xC0, 0x43,			/* mvns		r0, r0 */
		0x0A, 0x4E,			/* ldr		r6, CRC32XOR */
		0x0B, 0x46,			/* mov		r3, r1 */
		0x00, 0x24,			/* movs		r4, #0 */
		0x0D, 0xE0,			/* b		ncomp */
		/* nbyte: */
		0x11, 0x5D,			/* ldrb		r1, [r2, r4] */
		0x09, 0x06,			/* lsls		r1, r1, #24 */
		0x48, 0x40,			/* eors		r0, r0, r1 */
		0x00, 0x25,			/* movs		r5, #0 */
		/* loop: */
		0x00, 0x28,			/* cmp		r0, #0 */
		0x02, 0xDA,			/* bge		notset */
		0x40, 0x00,			/* lsls		r0, r0, #1 */
		0x70, 0x40,			/* eors		r0, r0, r6 */
		0x00, 0xE0,			/* b		cont */
		/* notset: */
		0x40, 0x00,			/* lsls		r0, r0, #1 */
		/* cont: */
		0x01, 0x35,			/* adds		r5, r5, #1 */
		0x08, 0x2D,			/* cmp		r5, #8 */
		0xF6, 0xD1,			/* bne		loop */
		0x01, 0x34,			/* adds		r4, r4, #1 */
		/* ncomp: */
		0x9C, 0x42,			/* cmp		r4, r3 */
		0xEF, 0xD1,			/* bne		nbyte */
		0x00, 0xBE,			/* bkpt		#0 */
		0xB7, 0x1D, 0xC1, 0x04	/* CRC32XOR:	.word	0x04c11db7 */
	};

	retval = target_alloc_working_area(target, sizeof(cortex_m_crc_code), &crc_algorithm);
	if (retval != ERROR_OK)
		return retval;

	retval = target_write_buffer(target, crc_algorithm->address,
			sizeof(cortex_m_crc_code), (uint8_t *)cortex_m_crc_code);
	if (retval != ERROR_OK)
		goto cleanup;

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);

	buf_set_u32(reg_params[0].value, 0, 32, address);
	buf_set_u32(reg_params[1].value, 0, 32, count);

	int timeout = 20000 * (1 + (count / (1024 * 1024)));

	retval = target_run_algorithm(target, 0, NULL, 2, reg_params, crc_algorithm->address,
			crc_algorithm->address + (sizeof(cortex_m_crc_code) - 6),
			timeout, &armv7m_info);

	if (retval == ERROR_OK)
		*checksum = buf_get_u32(reg_params[0].value, 0, 32);
	else
		LOG_ERROR("error executing cortex_m crc algorithm");

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);

cleanup:
	target_free_working_area(target, crc_algorithm);

	return retval;
}

/** Checks whether a memory region is zeroed. */
int armv7m_blank_check_memory(struct target *target,
	uint32_t address, uint32_t count, uint32_t *blank)
{
	struct working_area *erase_check_algorithm;
	struct reg_param reg_params[3];
	struct armv7m_algorithm armv7m_info;
	int retval;

	/* see contrib/loaders/erase_check/armv7m_erase_check.s for src */

	static const uint8_t erase_check_code[] = {
		/* loop: */
		0x03, 0x78,		/* ldrb	r3, [r0] */
		0x01, 0x30,		/* adds	r0, #1 */
		0x1A, 0x40,		/* ands	r2, r2, r3 */
		0x01, 0x39,		/* subs	r1, r1, #1 */
		0xFA, 0xD1,		/* bne	loop */
		0x00, 0xBE		/* bkpt	#0 */
	};

	/* make sure we have a working area */
	if (target_alloc_working_area(target, sizeof(erase_check_code),
		&erase_check_algorithm) != ERROR_OK)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	retval = target_write_buffer(target, erase_check_algorithm->address,
			sizeof(erase_check_code), (uint8_t *)erase_check_code);
	if (retval != ERROR_OK)
		return retval;

	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, address);

	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	buf_set_u32(reg_params[1].value, 0, 32, count);

	init_reg_param(&reg_params[2], "r2", 32, PARAM_IN_OUT);
	buf_set_u32(reg_params[2].value, 0, 32, 0xff);

	retval = target_run_algorithm(target,
			0,
			NULL,
			3,
			reg_params,
			erase_check_algorithm->address,
			erase_check_algorithm->address + (sizeof(erase_check_code) - 2),
			10000,
			&armv7m_info);

	if (retval == ERROR_OK)
		*blank = buf_get_u32(reg_params[2].value, 0, 32);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);

	target_free_working_area(target, erase_check_algorithm);

	return retval;
}

int armv7m_maybe_skip_bkpt_inst(struct target *target, bool *inst_found)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct reg *r = armv7m->arm.pc;
	bool result = false;


	/* if we halted last time due to a bkpt instruction
	 * then we have to manually step over it, otherwise
	 * the core will break again */

	if (target->debug_reason == DBG_REASON_BREAKPOINT) {
		uint16_t op;
		uint32_t pc = buf_get_u32(r->value, 0, 32);

		pc &= ~1;
		if (target_read_u16(target, pc, &op) == ERROR_OK) {
			if ((op & 0xFF00) == 0xBE00) {
				pc = buf_get_u32(r->value, 0, 32) + 2;
				buf_set_u32(r->value, 0, 32, pc);
				r->dirty = true;
				r->valid = true;
				result = true;
				LOG_DEBUG("Skipping over BKPT instruction");
			}
		}
	}

	if (inst_found)
		*inst_found = result;

	return ERROR_OK;
}

const struct command_registration armv7m_command_handlers[] = {
	{
		.chain = arm_command_handlers,
	},
	{
		.chain = dap_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};
