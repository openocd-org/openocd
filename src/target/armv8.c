/***************************************************************************
 *   Copyright (C) 2015 by David Ung                                       *
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
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/replacements.h>

#include "armv8.h"
#include "arm_disassembler.h"

#include "register.h"
#include <helper/binarybuffer.h>
#include <helper/command.h>

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

#include "arm_opcodes.h"
#include "target.h"
#include "target_type.h"

static int armv8_read_core_reg(struct target *target, struct reg *r,
	int num, enum arm_mode mode)
{
	uint64_t reg_value;
	int retval;
	struct arm_reg *armv8_core_reg;
	struct armv8_common *armv8 = target_to_armv8(target);

	assert(num < (int)armv8->arm.core_cache->num_regs);

	armv8_core_reg = armv8->arm.core_cache->reg_list[num].arch_info;
	retval = armv8->load_core_reg_u64(target,
			armv8_core_reg->num, &reg_value);

	buf_set_u64(armv8->arm.core_cache->reg_list[num].value, 0, 64, reg_value);
	armv8->arm.core_cache->reg_list[num].valid = 1;
	armv8->arm.core_cache->reg_list[num].dirty = 0;

	return retval;
}

#if 0
static int armv8_write_core_reg(struct target *target, struct reg *r,
	int num, enum arm_mode mode, target_ulong value)
{
	int retval;
	struct arm_reg *armv8_core_reg;
	struct armv8_common *armv8 = target_to_armv8(target);

	assert(num < (int)armv8->arm.core_cache->num_regs);

	armv8_core_reg = armv8->arm.core_cache->reg_list[num].arch_info;
	retval = armv8->store_core_reg_u64(target,
					    armv8_core_reg->num,
					    value);
	if (retval != ERROR_OK) {
		LOG_ERROR("JTAG failure");
		armv8->arm.core_cache->reg_list[num].dirty = armv8->arm.core_cache->reg_list[num].valid;
		return ERROR_JTAG_DEVICE_ERROR;
	}

	LOG_DEBUG("write core reg %i value 0x%" PRIx64 "", num, value);
	armv8->arm.core_cache->reg_list[num].valid = 1;
	armv8->arm.core_cache->reg_list[num].dirty = 0;

	return ERROR_OK;
}
#endif

static void armv8_show_fault_registers(struct target *target)
{
	/* TODO */
}

static int armv8_read_ttbcr(struct target *target)
{
	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm_dpm *dpm = armv8->arm.dpm;
	uint32_t ttbcr;
	int retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;
	/*  MRC p15,0,<Rt>,c2,c0,2 ; Read CP15 Translation Table Base Control Register*/
	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 2, 0, 2),
			&ttbcr);
	if (retval != ERROR_OK)
		goto done;
	armv8->armv8_mmu.ttbr1_used = ((ttbcr & 0x7) != 0) ? 1 : 0;
	armv8->armv8_mmu.ttbr0_mask  = 7 << (32 - ((ttbcr & 0x7)));
#if 0
	LOG_INFO("ttb1 %s ,ttb0_mask %x",
		armv8->armv8_mmu.ttbr1_used ? "used" : "not used",
		armv8->armv8_mmu.ttbr0_mask);
#endif
	if (armv8->armv8_mmu.ttbr1_used == 1) {
		LOG_INFO("SVC access above %" PRIx32,
			 (uint32_t)(0xffffffff & armv8->armv8_mmu.ttbr0_mask));
		armv8->armv8_mmu.os_border = 0xffffffff & armv8->armv8_mmu.ttbr0_mask;
	} else {
		/*  fix me , default is hard coded LINUX border  */
		armv8->armv8_mmu.os_border = 0xc0000000;
	}
done:
	dpm->finish(dpm);
	return retval;
}


/*  method adapted to cortex A : reused arm v4 v5 method*/
int armv8_mmu_translate_va(struct target *target,  uint32_t va, uint32_t *val)
{
	uint32_t first_lvl_descriptor = 0x0;
	uint32_t second_lvl_descriptor = 0x0;
	int retval;
	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm_dpm *dpm = armv8->arm.dpm;
	uint32_t ttb = 0;	/*  default ttb0 */
	if (armv8->armv8_mmu.ttbr1_used == -1)
		armv8_read_ttbcr(target);
	if ((armv8->armv8_mmu.ttbr1_used) &&
		(va > (0xffffffff & armv8->armv8_mmu.ttbr0_mask))) {
		/*  select ttb 1 */
		ttb = 1;
	}
	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;

	/*  MRC p15,0,<Rt>,c2,c0,ttb */
	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 2, 0, ttb),
			&ttb);
	if (retval != ERROR_OK)
		return retval;
	retval = armv8->armv8_mmu.read_physical_memory(target,
			(ttb & 0xffffc000) | ((va & 0xfff00000) >> 18),
			4, 1, (uint8_t *)&first_lvl_descriptor);
	if (retval != ERROR_OK)
		return retval;
	first_lvl_descriptor = target_buffer_get_u32(target, (uint8_t *)
			&first_lvl_descriptor);
	/*  reuse armv4_5 piece of code, specific armv8 changes may come later */
	LOG_DEBUG("1st lvl desc: %8.8" PRIx32 "", first_lvl_descriptor);

	if ((first_lvl_descriptor & 0x3) == 0) {
		LOG_ERROR("Address translation failure");
		return ERROR_TARGET_TRANSLATION_FAULT;
	}


	if ((first_lvl_descriptor & 0x3) == 2) {
		/* section descriptor */
		*val = (first_lvl_descriptor & 0xfff00000) | (va & 0x000fffff);
		return ERROR_OK;
	}

	if ((first_lvl_descriptor & 0x3) == 1) {
		/* coarse page table */
		retval = armv8->armv8_mmu.read_physical_memory(target,
				(first_lvl_descriptor & 0xfffffc00) | ((va & 0x000ff000) >> 10),
				4, 1, (uint8_t *)&second_lvl_descriptor);
		if (retval != ERROR_OK)
			return retval;
	} else if ((first_lvl_descriptor & 0x3) == 3)   {
		/* fine page table */
		retval = armv8->armv8_mmu.read_physical_memory(target,
				(first_lvl_descriptor & 0xfffff000) | ((va & 0x000ffc00) >> 8),
				4, 1, (uint8_t *)&second_lvl_descriptor);
		if (retval != ERROR_OK)
			return retval;
	}

	second_lvl_descriptor = target_buffer_get_u32(target, (uint8_t *)
			&second_lvl_descriptor);

	LOG_DEBUG("2nd lvl desc: %8.8" PRIx32 "", second_lvl_descriptor);

	if ((second_lvl_descriptor & 0x3) == 0) {
		LOG_ERROR("Address translation failure");
		return ERROR_TARGET_TRANSLATION_FAULT;
	}

	if ((second_lvl_descriptor & 0x3) == 1) {
		/* large page descriptor */
		*val = (second_lvl_descriptor & 0xffff0000) | (va & 0x0000ffff);
		return ERROR_OK;
	}

	if ((second_lvl_descriptor & 0x3) == 2) {
		/* small page descriptor */
		*val = (second_lvl_descriptor & 0xfffff000) | (va & 0x00000fff);
		return ERROR_OK;
	}

	if ((second_lvl_descriptor & 0x3) == 3) {
		*val = (second_lvl_descriptor & 0xfffffc00) | (va & 0x000003ff);
		return ERROR_OK;
	}

	/* should not happen */
	LOG_ERROR("Address translation failure");
	return ERROR_TARGET_TRANSLATION_FAULT;

done:
	return retval;
}

/*  V8 method VA TO PA  */
int armv8_mmu_translate_va_pa(struct target *target, target_ulong va,
	target_ulong *val, int meminfo)
{
	return ERROR_OK;
}

static int armv8_handle_inner_cache_info_command(struct command_context *cmd_ctx,
	struct armv8_cache_common *armv8_cache)
{
	if (armv8_cache->ctype == -1) {
		command_print(cmd_ctx, "cache not yet identified");
		return ERROR_OK;
	}

	command_print(cmd_ctx,
		"D-Cache: linelen %" PRIi32 ", associativity %" PRIi32 ", nsets %" PRIi32 ", cachesize %" PRId32 " KBytes",
		armv8_cache->d_u_size.linelen,
		armv8_cache->d_u_size.associativity,
		armv8_cache->d_u_size.nsets,
		armv8_cache->d_u_size.cachesize);

	command_print(cmd_ctx,
		"I-Cache: linelen %" PRIi32 ", associativity %" PRIi32 ", nsets %" PRIi32 ", cachesize %" PRId32 " KBytes",
		armv8_cache->i_size.linelen,
		armv8_cache->i_size.associativity,
		armv8_cache->i_size.nsets,
		armv8_cache->i_size.cachesize);

	return ERROR_OK;
}

static int _armv8_flush_all_data(struct target *target)
{
	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm_dpm *dpm = armv8->arm.dpm;
	struct armv8_cachesize *d_u_size =
		&(armv8->armv8_mmu.armv8_cache.d_u_size);
	int32_t c_way, c_index = d_u_size->index;
	int retval;
	/*  check that cache data is on at target halt */
	if (!armv8->armv8_mmu.armv8_cache.d_u_cache_enabled) {
		LOG_INFO("flushed not performed :cache not on at target halt");
		return ERROR_OK;
	}
	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;
	do {
		c_way = d_u_size->way;
		do {
			uint32_t value = (c_index << d_u_size->index_shift)
				| (c_way << d_u_size->way_shift);
			/*  DCCISW */
			/* LOG_INFO ("%d %d %x",c_way,c_index,value); */
			retval = dpm->instr_write_data_r0(dpm,
					ARMV4_5_MCR(15, 0, 0, 7, 14, 2),
					value);
			if (retval != ERROR_OK)
				goto done;
			c_way -= 1;
		} while (c_way >= 0);
		c_index -= 1;
	} while (c_index >= 0);
	return retval;
done:
	LOG_ERROR("flushed failed");
	dpm->finish(dpm);
	return retval;
}

static int  armv8_flush_all_data(struct target *target)
{
	int retval = ERROR_FAIL;
	/*  check that armv8_cache is correctly identify */
	struct armv8_common *armv8 = target_to_armv8(target);
	if (armv8->armv8_mmu.armv8_cache.ctype == -1) {
		LOG_ERROR("trying to flush un-identified cache");
		return retval;
	}

	if (target->smp) {
		/*  look if all the other target have been flushed in order to flush level
		 *  2 */
		struct target_list *head;
		struct target *curr;
		head = target->head;
		while (head != (struct target_list *)NULL) {
			curr = head->target;
			if (curr->state == TARGET_HALTED) {
				LOG_INFO("Wait flushing data l1 on core %" PRId32, curr->coreid);
				retval = _armv8_flush_all_data(curr);
			}
			head = head->next;
		}
	} else
		retval = _armv8_flush_all_data(target);
	return retval;
}

int armv8_handle_cache_info_command(struct command_context *cmd_ctx,
	struct armv8_cache_common *armv8_cache)
{
	if (armv8_cache->ctype == -1) {
		command_print(cmd_ctx, "cache not yet identified");
		return ERROR_OK;
	}

	if (armv8_cache->display_cache_info)
		armv8_cache->display_cache_info(cmd_ctx, armv8_cache);
	return ERROR_OK;
}

/*  retrieve core id cluster id  */
static int armv8_read_mpidr(struct target *target)
{
	int retval = ERROR_FAIL;
	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm_dpm *dpm = armv8->arm.dpm;
	uint32_t mpidr;
	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;
	/* MRC p15,0,<Rd>,c0,c0,5; read Multiprocessor ID register*/

	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 0, 0, 5),
			&mpidr);
	if (retval != ERROR_OK)
		goto done;
	if (mpidr & 1<<31) {
		armv8->multi_processor_system = (mpidr >> 30) & 1;
		armv8->cluster_id = (mpidr >> 8) & 0xf;
		armv8->cpu_id = mpidr & 0x3;
		LOG_INFO("%s cluster %x core %x %s", target_name(target),
			armv8->cluster_id,
			armv8->cpu_id,
			armv8->multi_processor_system == 0 ? "multi core" : "mono core");

	} else
		LOG_ERROR("mpdir not in multiprocessor format");

done:
	dpm->finish(dpm);
	return retval;


}

int armv8_identify_cache(struct target *target)
{
	/*  read cache descriptor */
	int retval = ERROR_FAIL;
	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm_dpm *dpm = armv8->arm.dpm;
	uint32_t cache_selected, clidr;
	uint32_t cache_i_reg, cache_d_reg;
	struct armv8_cache_common *cache = &(armv8->armv8_mmu.armv8_cache);
	if (!armv8->is_armv7r)
		armv8_read_ttbcr(target);
	retval = dpm->prepare(dpm);

	if (retval != ERROR_OK)
		goto done;
	/*  retrieve CLIDR
	 *  mrc	p15, 1, r0, c0, c0, 1		@ read clidr */
	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 1, 0, 0, 0, 1),
			&clidr);
	if (retval != ERROR_OK)
		goto done;
	clidr = (clidr & 0x7000000) >> 23;
	LOG_INFO("number of cache level %" PRIx32, (uint32_t)(clidr / 2));
	if ((clidr / 2) > 1) {
		/* FIXME not supported present in cortex A8 and later */
		/*  in cortex A7, A15 */
		LOG_ERROR("cache l2 present :not supported");
	}
	/*  retrieve selected cache
	 *  MRC p15, 2,<Rd>, c0, c0, 0; Read CSSELR */
	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 2, 0, 0, 0, 0),
			&cache_selected);
	if (retval != ERROR_OK)
		goto done;

	retval = armv8->arm.mrc(target, 15,
			2, 0,	/* op1, op2 */
			0, 0,	/* CRn, CRm */
			&cache_selected);
	if (retval != ERROR_OK)
		goto done;
	/* select instruction cache
	 *  MCR p15, 2,<Rd>, c0, c0, 0; Write CSSELR
	 *  [0]  : 1 instruction cache selection , 0 data cache selection */
	retval = dpm->instr_write_data_r0(dpm,
			ARMV4_5_MRC(15, 2, 0, 0, 0, 0),
			1);
	if (retval != ERROR_OK)
		goto done;

	/* read CCSIDR
	 * MRC P15,1,<RT>,C0, C0,0 ;on cortex A9 read CCSIDR
	 * [2:0] line size  001 eight word per line
	 * [27:13] NumSet 0x7f 16KB, 0xff 32Kbytes, 0x1ff 64Kbytes */
	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 1, 0, 0, 0, 0),
			&cache_i_reg);
	if (retval != ERROR_OK)
		goto done;

	/*  select data cache*/
	retval = dpm->instr_write_data_r0(dpm,
			ARMV4_5_MRC(15, 2, 0, 0, 0, 0),
			0);
	if (retval != ERROR_OK)
		goto done;

	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 1, 0, 0, 0, 0),
			&cache_d_reg);
	if (retval != ERROR_OK)
		goto done;

	/*  restore selected cache  */
	dpm->instr_write_data_r0(dpm,
		ARMV4_5_MRC(15, 2, 0, 0, 0, 0),
		cache_selected);

	if (retval != ERROR_OK)
		goto done;
	dpm->finish(dpm);

	/* put fake type */
	cache->d_u_size.linelen = 16 << (cache_d_reg & 0x7);
	cache->d_u_size.cachesize = (((cache_d_reg >> 13) & 0x7fff)+1)/8;
	cache->d_u_size.nsets = (cache_d_reg >> 13) & 0x7fff;
	cache->d_u_size.associativity = ((cache_d_reg >> 3) & 0x3ff) + 1;
	/*  compute info for set way operation on cache */
	cache->d_u_size.index_shift = (cache_d_reg & 0x7) + 4;
	cache->d_u_size.index = (cache_d_reg >> 13) & 0x7fff;
	cache->d_u_size.way = ((cache_d_reg >> 3) & 0x3ff);
	cache->d_u_size.way_shift = cache->d_u_size.way + 1;
	{
		int i = 0;
		while (((cache->d_u_size.way_shift >> i) & 1) != 1)
			i++;
		cache->d_u_size.way_shift = 32-i;
	}
#if 0
	LOG_INFO("data cache index %d << %d, way %d << %d",
			cache->d_u_size.index, cache->d_u_size.index_shift,
			cache->d_u_size.way,
			cache->d_u_size.way_shift);

	LOG_INFO("data cache %d bytes %d KBytes asso %d ways",
			cache->d_u_size.linelen,
			cache->d_u_size.cachesize,
			cache->d_u_size.associativity);
#endif
	cache->i_size.linelen = 16 << (cache_i_reg & 0x7);
	cache->i_size.associativity = ((cache_i_reg >> 3) & 0x3ff) + 1;
	cache->i_size.nsets = (cache_i_reg >> 13) & 0x7fff;
	cache->i_size.cachesize = (((cache_i_reg >> 13) & 0x7fff)+1)/8;
	/*  compute info for set way operation on cache */
	cache->i_size.index_shift = (cache_i_reg & 0x7) + 4;
	cache->i_size.index = (cache_i_reg >> 13) & 0x7fff;
	cache->i_size.way = ((cache_i_reg >> 3) & 0x3ff);
	cache->i_size.way_shift = cache->i_size.way + 1;
	{
		int i = 0;
		while (((cache->i_size.way_shift >> i) & 1) != 1)
			i++;
		cache->i_size.way_shift = 32-i;
	}
#if 0
	LOG_INFO("instruction cache index %d << %d, way %d << %d",
			cache->i_size.index, cache->i_size.index_shift,
			cache->i_size.way, cache->i_size.way_shift);

	LOG_INFO("instruction cache %d bytes %d KBytes asso %d ways",
			cache->i_size.linelen,
			cache->i_size.cachesize,
			cache->i_size.associativity);
#endif
	/*  if no l2 cache initialize l1 data cache flush function function */
	if (armv8->armv8_mmu.armv8_cache.flush_all_data_cache == NULL) {
		armv8->armv8_mmu.armv8_cache.display_cache_info =
			armv8_handle_inner_cache_info_command;
		armv8->armv8_mmu.armv8_cache.flush_all_data_cache =
			armv8_flush_all_data;
	}
	armv8->armv8_mmu.armv8_cache.ctype = 0;

done:
	dpm->finish(dpm);
	armv8_read_mpidr(target);
	return retval;

}

int armv8_init_arch_info(struct target *target, struct armv8_common *armv8)
{
	struct arm *arm = &armv8->arm;
	arm->arch_info = armv8;
	target->arch_info = &armv8->arm;
	/*  target is useful in all function arm v4 5 compatible */
	armv8->arm.target = target;
	armv8->arm.common_magic = ARM_COMMON_MAGIC;
	armv8->common_magic = ARMV8_COMMON_MAGIC;

	arm->read_core_reg = armv8_read_core_reg;
#if 0
	arm->write_core_reg = armv8_write_core_reg;
#endif

	armv8->armv8_mmu.armv8_cache.l2_cache = NULL;
	armv8->armv8_mmu.armv8_cache.ctype = -1;
	armv8->armv8_mmu.armv8_cache.flush_all_data_cache = NULL;
	armv8->armv8_mmu.armv8_cache.display_cache_info = NULL;
	return ERROR_OK;
}

int armv8_arch_state(struct target *target)
{
	static const char * const state[] = {
		"disabled", "enabled"
	};

	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm *arm = &armv8->arm;

	if (armv8->common_magic != ARMV8_COMMON_MAGIC) {
		LOG_ERROR("BUG: called for a non-Armv8 target");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	arm_arch_state(target);

	if (armv8->is_armv7r) {
		LOG_USER("D-Cache: %s, I-Cache: %s",
			state[armv8->armv8_mmu.armv8_cache.d_u_cache_enabled],
			state[armv8->armv8_mmu.armv8_cache.i_cache_enabled]);
	} else {
		LOG_USER("MMU: %s, D-Cache: %s, I-Cache: %s",
			state[armv8->armv8_mmu.mmu_enabled],
			state[armv8->armv8_mmu.armv8_cache.d_u_cache_enabled],
			state[armv8->armv8_mmu.armv8_cache.i_cache_enabled]);
	}

	if (arm->core_mode == ARM_MODE_ABT)
		armv8_show_fault_registers(target);
	if (target->debug_reason == DBG_REASON_WATCHPOINT)
		LOG_USER("Watchpoint triggered at PC %#08x",
			(unsigned) armv8->dpm.wp_pc);

	return ERROR_OK;
}

static const struct {
	unsigned id;
	const char *name;
	unsigned bits;
	enum reg_type type;
	const char *group;
	const char *feature;
} armv8_regs[] = {
	{ ARMV8_R0,  "x0",  64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R1,  "x1",  64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R2,  "x2",  64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R3,  "x3",  64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R4,  "x4",  64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R5,  "x5",  64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R6,  "x6",  64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R7,  "x7",  64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R8,  "x8",  64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R9,  "x9",  64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R10, "x10", 64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R11, "x11", 64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R12, "x12", 64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R13, "x13", 64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R14, "x14", 64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R15, "x15", 64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R16, "x16", 64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R17, "x17", 64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R18, "x18", 64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R19, "x19", 64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R20, "x20", 64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R21, "x21", 64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R22, "x22", 64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R23, "x23", 64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R24, "x24", 64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R25, "x25", 64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R26, "x26", 64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R27, "x27", 64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R28, "x28", 64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R29, "x29", 64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_R30, "x30", 64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },

	{ ARMV8_R31, "sp", 64, REG_TYPE_DATA_PTR, "general", "org.gnu.gdb.aarch64.core" },
	{ ARMV8_PC,  "pc", 64, REG_TYPE_CODE_PTR, "general", "org.gnu.gdb.aarch64.core" },

	{ ARMV8_xPSR, "CPSR", 64, REG_TYPE_INT, "general", "org.gnu.gdb.aarch64.core" },
};

#define ARMV8_NUM_REGS ARRAY_SIZE(armv8_regs)


static int armv8_get_core_reg(struct reg *reg)
{
	int retval;
	struct arm_reg *armv8_reg = reg->arch_info;
	struct target *target = armv8_reg->target;
	struct arm *arm = target_to_arm(target);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	retval = arm->read_core_reg(target, reg, armv8_reg->num, arm->core_mode);

	return retval;
}

static int armv8_set_core_reg(struct reg *reg, uint8_t *buf)
{
	struct arm_reg *armv8_reg = reg->arch_info;
	struct target *target = armv8_reg->target;
	uint64_t value = buf_get_u64(buf, 0, 64);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	buf_set_u64(reg->value, 0, 64, value);
	reg->dirty = 1;
	reg->valid = 1;

	return ERROR_OK;
}

static const struct reg_arch_type armv8_reg_type = {
	.get = armv8_get_core_reg,
	.set = armv8_set_core_reg,
};

/** Builds cache of architecturally defined registers.  */
struct reg_cache *armv8_build_reg_cache(struct target *target)
{
	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm *arm = &armv8->arm;
	int num_regs = ARMV8_NUM_REGS;
	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	struct reg_cache *cache = malloc(sizeof(struct reg_cache));
	struct reg *reg_list = calloc(num_regs, sizeof(struct reg));
	struct arm_reg *arch_info = calloc(num_regs, sizeof(struct arm_reg));
	struct reg_feature *feature;
	int i;

	/* Build the process context cache */
	cache->name = "arm v8 registers";
	cache->next = NULL;
	cache->reg_list = reg_list;
	cache->num_regs = num_regs;
	(*cache_p) = cache;

	for (i = 0; i < num_regs; i++) {
		arch_info[i].num = armv8_regs[i].id;
		arch_info[i].target = target;
		arch_info[i].arm = arm;

		reg_list[i].name = armv8_regs[i].name;
		reg_list[i].size = armv8_regs[i].bits;
		reg_list[i].value = calloc(1, 4);
		reg_list[i].dirty = 0;
		reg_list[i].valid = 0;
		reg_list[i].type = &armv8_reg_type;
		reg_list[i].arch_info = &arch_info[i];

		reg_list[i].group = armv8_regs[i].group;
		reg_list[i].number = i;
		reg_list[i].exist = true;
		reg_list[i].caller_save = true;	/* gdb defaults to true */

		feature = calloc(1, sizeof(struct reg_feature));
		if (feature) {
			feature->name = armv8_regs[i].feature;
			reg_list[i].feature = feature;
		} else
			LOG_ERROR("unable to allocate feature list");

		reg_list[i].reg_data_type = calloc(1, sizeof(struct reg_data_type));
		if (reg_list[i].reg_data_type)
			reg_list[i].reg_data_type->type = armv8_regs[i].type;
		else
			LOG_ERROR("unable to allocate reg type list");
	}

	arm->cpsr = reg_list + ARMV8_xPSR;
	arm->pc = reg_list + ARMV8_PC;
	arm->core_cache = cache;

	return cache;
}

struct reg *armv8_reg_current(struct arm *arm, unsigned regnum)
{
	struct reg *r;

	if (regnum > 33)
		return NULL;

	r = arm->core_cache->reg_list + regnum;
	return r;
}

const struct command_registration armv8_command_handlers[] = {
	{
		.chain = dap_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};


int armv8_get_gdb_reg_list(struct target *target,
	struct reg **reg_list[], int *reg_list_size,
	enum target_register_class reg_class)
{
	struct arm *arm = target_to_arm(target);
	int i;

	switch (reg_class) {
	case REG_CLASS_GENERAL:
	case REG_CLASS_ALL:
		*reg_list_size = 34;
		*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

		for (i = 0; i < *reg_list_size; i++)
				(*reg_list)[i] = armv8_reg_current(arm, i);

		return ERROR_OK;
		break;

	default:
		LOG_ERROR("not a valid register class type in query.");
		return ERROR_FAIL;
		break;
	}
}
