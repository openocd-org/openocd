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

#include "armv8_opcodes.h"
#include "target.h"
#include "target_type.h"

static const char * const armv8_state_strings[] = {
	"ARM", "Thumb", "Jazelle", "ThumbEE", "ARM64",
};

static const struct {
	const char *name;
	unsigned psr;
	/* For user and system modes, these list indices for all registers.
	 * otherwise they're just indices for the shadow registers and SPSR.
	 */
	unsigned short n_indices;
	const uint8_t *indices;
} armv8_mode_data[] = {
	/* These special modes are currently only supported
	 * by ARMv6M and ARMv7M profiles */
	{
		.name = "EL0T",
		.psr = ARMV8_64_EL0T,
	},
	{
		.name = "EL1T",
		.psr = ARMV8_64_EL1T,
	},
	{
		.name = "EL1H",
		.psr = ARMV8_64_EL1H,
	},
	{
		.name = "EL2T",
		.psr = ARMV8_64_EL2T,
	},
	{
		.name = "EL2H",
		.psr = ARMV8_64_EL2H,
	},
	{
		.name = "EL3T",
		.psr = ARMV8_64_EL3T,
	},
	{
		.name = "EL3H",
		.psr = ARMV8_64_EL3H,
	},
};

/** Map PSR mode bits to the name of an ARM processor operating mode. */
const char *armv8_mode_name(unsigned psr_mode)
{
	for (unsigned i = 0; i < ARRAY_SIZE(armv8_mode_data); i++) {
		if (armv8_mode_data[i].psr == psr_mode)
			return armv8_mode_data[i].name;
	}
	LOG_ERROR("unrecognized psr mode: %#02x", psr_mode);
	return "UNRECOGNIZED";
}

int armv8_mode_to_number(enum arm_mode mode)
{
	switch (mode) {
		case ARM_MODE_ANY:
		/* map MODE_ANY to user mode */
		case ARM_MODE_USR:
			return 0;
		case ARM_MODE_FIQ:
			return 1;
		case ARM_MODE_IRQ:
			return 2;
		case ARM_MODE_SVC:
			return 3;
		case ARM_MODE_ABT:
			return 4;
		case ARM_MODE_UND:
			return 5;
		case ARM_MODE_SYS:
			return 6;
		case ARM_MODE_MON:
			return 7;
		case ARMV8_64_EL0T:
			return 8;
		case ARMV8_64_EL1T:
			return 9;
		case ARMV8_64_EL1H:
			return 10;
		case ARMV8_64_EL2T:
			return 11;
		case ARMV8_64_EL2H:
			return 12;
		case ARMV8_64_EL3T:
			return 13;
		case ARMV8_64_EL3H:
			return 14;

		default:
			LOG_ERROR("invalid mode value encountered %d", mode);
			return -1;
	}
}


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
	int num, enum arm_mode mode, target_addr_t value)
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
/**
 * Configures host-side ARM records to reflect the specified CPSR.
 * Later, code can use arm_reg_current() to map register numbers
 * according to how they are exposed by this mode.
 */
void armv8_set_cpsr(struct arm *arm, uint32_t cpsr)
{
	uint32_t mode = cpsr & 0x1F;

	/* NOTE:  this may be called very early, before the register
	 * cache is set up.  We can't defend against many errors, in
	 * particular against CPSRs that aren't valid *here* ...
	 */
	if (arm->cpsr) {
		buf_set_u32(arm->cpsr->value, 0, 32, cpsr);
		arm->cpsr->valid = 1;
		arm->cpsr->dirty = 0;
	}

	/* Older ARMs won't have the J bit */
	enum arm_state state = 0xFF;

	if (((cpsr & 0x10) >> 4) == 0) {
		state = ARM_STATE_AARCH64;
	} else {
		if (cpsr & (1 << 5)) {	/* T */
			if (cpsr & (1 << 24)) { /* J */
				LOG_WARNING("ThumbEE -- incomplete support");
				state = ARM_STATE_THUMB_EE;
			} else
				state = ARM_STATE_THUMB;
		} else {
			if (cpsr & (1 << 24)) { /* J */
				LOG_ERROR("Jazelle state handling is BROKEN!");
				state = ARM_STATE_JAZELLE;
			} else
				state = ARM_STATE_ARM;
		}
	}
	arm->core_state = state;
	if (arm->core_state == ARM_STATE_AARCH64) {
		switch (mode) {
			case SYSTEM_AAR64_MODE_EL0t:
				arm->core_mode = ARMV8_64_EL0T;
			break;
			case SYSTEM_AAR64_MODE_EL1t:
				arm->core_mode = ARMV8_64_EL0T;
			break;
			case SYSTEM_AAR64_MODE_EL1h:
				arm->core_mode = ARMV8_64_EL1H;
			break;
			case SYSTEM_AAR64_MODE_EL2t:
				arm->core_mode = ARMV8_64_EL2T;
			break;
			case SYSTEM_AAR64_MODE_EL2h:
				arm->core_mode = ARMV8_64_EL2H;
			break;
			case SYSTEM_AAR64_MODE_EL3t:
				arm->core_mode = ARMV8_64_EL3T;
			break;
			case SYSTEM_AAR64_MODE_EL3h:
				arm->core_mode = ARMV8_64_EL3H;
			break;
			default:
				LOG_DEBUG("unknow mode 0x%x", (unsigned) (mode));
			break;
		}
	} else {
		arm->core_mode = mode;
	}

	LOG_DEBUG("set CPSR %#8.8x: %s mode, %s state", (unsigned) cpsr,
		armv8_mode_name(arm->core_mode),
		armv8_state_strings[arm->core_state]);
}

static void armv8_show_fault_registers(struct target *target)
{
	/* TODO */
}

static uint8_t armv8_pa_size(uint32_t ps)
{
	uint8_t ret = 0;
	switch (ps) {
		case 0:
			ret = 32;
			break;
		case 1:
			ret = 36;
			break;
		case 2:
			ret = 40;
			break;
		case 3:
			ret = 42;
			break;
		case 4:
			ret = 44;
			break;
		case 5:
			ret = 48;
			break;
		default:
			LOG_INFO("Unknow physicall address size");
			break;
	}
	return ret;
}

static int armv8_read_ttbcr(struct target *target)
{
	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm_dpm *dpm = armv8->arm.dpm;
	struct arm *arm = &armv8->arm;
	uint32_t ttbcr;
	uint64_t ttbcr_64;

	int retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;

	/* claaer ttrr1_used and ttbr0_mask */
	memset(&armv8->armv8_mmu.ttbr1_used, 0, sizeof(armv8->armv8_mmu.ttbr1_used));
	memset(&armv8->armv8_mmu.ttbr0_mask, 0, sizeof(armv8->armv8_mmu.ttbr0_mask));

	switch (arm->core_mode) {
		case ARMV8_64_EL3H:
		case ARMV8_64_EL3T:
			retval = dpm->instr_read_data_r0(dpm,
					ARMV8_MRS(SYSTEM_TCR_EL3, 0),
					&ttbcr);
			retval += dpm->instr_read_data_r0_64(dpm,
					ARMV8_MRS(SYSTEM_TTBR0_EL3, 0),
					&armv8->ttbr_base);
			if (retval != ERROR_OK)
				goto done;
			armv8->va_size = 64 - (ttbcr & 0x3F);
			armv8->pa_size = armv8_pa_size((ttbcr >> 16) & 7);
			armv8->page_size = (ttbcr >> 14) & 3;
			break;
		case ARMV8_64_EL2T:
		case ARMV8_64_EL2H:
			retval = dpm->instr_read_data_r0(dpm,
					ARMV8_MRS(SYSTEM_TCR_EL2, 0),
					&ttbcr);
			retval += dpm->instr_read_data_r0_64(dpm,
					ARMV8_MRS(SYSTEM_TTBR0_EL2, 0),
					&armv8->ttbr_base);
			if (retval != ERROR_OK)
				goto done;
			armv8->va_size = 64 - (ttbcr & 0x3F);
			armv8->pa_size = armv8_pa_size((ttbcr >> 16) & 7);
			armv8->page_size = (ttbcr >> 14) & 3;
			break;
		case ARMV8_64_EL0T:
		case ARMV8_64_EL1T:
		case ARMV8_64_EL1H:
			retval = dpm->instr_read_data_r0_64(dpm,
					ARMV8_MRS(SYSTEM_TCR_EL1, 0),
					&ttbcr_64);
			armv8->va_size = 64 - (ttbcr_64 & 0x3F);
			armv8->pa_size = armv8_pa_size((ttbcr_64 >> 32) & 7);
			armv8->page_size = (ttbcr_64 >> 14) & 3;
			armv8->armv8_mmu.ttbr1_used = (((ttbcr_64 >> 16) & 0x3F) != 0) ? 1 : 0;
			armv8->armv8_mmu.ttbr0_mask  = 0x0000FFFFFFFFFFFF;
			retval += dpm->instr_read_data_r0_64(dpm,
					ARMV8_MRS(SYSTEM_TTBR0_EL1 | (armv8->armv8_mmu.ttbr1_used), 0),
					&armv8->ttbr_base);
			if (retval != ERROR_OK)
				goto done;
			break;
		default:
			LOG_ERROR("unknow core state");
			retval = ERROR_FAIL;
			break;
	}
	if (retval != ERROR_OK)
		goto done;

#if 0
	LOG_INFO("ttb1 %s ,ttb0_mask %llx",
		armv8->armv8_mmu.ttbr1_used ? "used" : "not used",
		armv8->armv8_mmu.ttbr0_mask);
#endif
	if (armv8->armv8_mmu.ttbr1_used == 1) {
		LOG_INFO("TTBR0 access above %" PRIx64,
			 (uint64_t)(armv8->armv8_mmu.ttbr0_mask));
		armv8->armv8_mmu.os_border = armv8->armv8_mmu.ttbr0_mask;
	} else {
		/*  fix me , default is hard coded LINUX border  */
		armv8->armv8_mmu.os_border = 0xc0000000;
	}
done:
	dpm->finish(dpm);
	return retval;
}

static int armv8_4K_translate(struct target *target,  uint32_t va, uint32_t *val)
{
	LOG_ERROR("4K page Address translation need to add");
	return ERROR_FAIL;
}


/*  method adapted to cortex A : reused arm v4 v5 method*/
int armv8_mmu_translate_va(struct target *target,  uint32_t va, uint32_t *val)
{
	int retval = ERROR_FAIL;
	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm_dpm *dpm = armv8->arm.dpm;

	retval = dpm->prepare(dpm);
	retval += armv8_read_ttbcr(target);
	if (retval != ERROR_OK)
		goto done;
	if (armv8->page_size == 0)
		return armv8_4K_translate(target, va, val);

done:
	dpm->finish(dpm);
	return ERROR_FAIL;
}

/*  V8 method VA TO PA  */
int armv8_mmu_translate_va_pa(struct target *target, target_addr_t va,
	target_addr_t *val, int meminfo)
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
					ARMV8_MSR_GP(SYSTEM_DCCISW, 0),
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
			ARMV8_MRS(SYSTEM_MPIDR, 0),
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
	/*	read cache descriptor */
	int retval = ERROR_FAIL;
	struct armv8_common *armv8 = target_to_armv8(target);
	struct arm_dpm *dpm = armv8->arm.dpm;
	uint32_t cache_selected, clidr;
	uint32_t cache_i_reg, cache_d_reg;
	struct armv8_cache_common *cache = &(armv8->armv8_mmu.armv8_cache);
	armv8_read_ttbcr(target);
	retval = dpm->prepare(dpm);

	if (retval != ERROR_OK)
		goto done;
	/*	retrieve CLIDR
	 *	mrc p15, 1, r0, c0, c0, 1		@ read clidr */
	retval = dpm->instr_read_data_r0(dpm,
			ARMV8_MRS(SYSTEM_CLIDR, 0),
			&clidr);
	if (retval != ERROR_OK)
		goto done;
	clidr = (clidr & 0x7000000) >> 23;
	LOG_INFO("number of cache level %" PRIx32, (uint32_t)(clidr / 2));
	if ((clidr / 2) > 1) {
		/* FIXME not supported present in cortex A8 and later */
		/*	in cortex A7, A15 */
		LOG_ERROR("cache l2 present :not supported");
	}
	/*	retrieve selected cache*/
	retval = dpm->instr_read_data_r0(dpm,
			ARMV8_MRS(SYSTEM_CSSELR, 0),
			&cache_selected);
	if (retval != ERROR_OK)
		goto done;


	/* select instruction cache
	 *	[0]  : 1 instruction cache selection , 0 data cache selection */
	retval = dpm->instr_write_data_r0(dpm,
			ARMV8_MRS(SYSTEM_CSSELR, 0),
			1);
	if (retval != ERROR_OK)
		goto done;

	/* read CCSIDR
	 * MRC P15,1,<RT>,C0, C0,0 ;on cortex A9 read CCSIDR
	 * [2:0] line size	001 eight word per line
	 * [27:13] NumSet 0x7f 16KB, 0xff 32Kbytes, 0x1ff 64Kbytes */
	retval = dpm->instr_read_data_r0(dpm,
			ARMV8_MRS(SYSTEM_CCSIDR, 0),
			&cache_i_reg);
	if (retval != ERROR_OK)
		goto done;

	/*	select data cache*/
	retval = dpm->instr_write_data_r0(dpm,
			ARMV8_MRS(SYSTEM_CSSELR, 0),
			0);
	if (retval != ERROR_OK)
		goto done;

	retval = dpm->instr_read_data_r0(dpm,
			ARMV8_MRS(SYSTEM_CCSIDR, 0),
			&cache_d_reg);
	if (retval != ERROR_OK)
		goto done;

	/*	restore selected cache	*/
	dpm->instr_write_data_r0(dpm,
		ARMV8_MRS(SYSTEM_CSSELR, 0),
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

int armv8_aarch64_state(struct target *target)
{
	struct arm *arm = target_to_arm(target);

	if (arm->common_magic != ARM_COMMON_MAGIC) {
		LOG_ERROR("BUG: called for a non-ARM target");
		return ERROR_FAIL;
	}

	LOG_USER("target halted in %s state due to %s, current mode: %s\n"
		"cpsr: 0x%8.8" PRIx32 " pc: 0x%" PRIx64 "%s",
		armv8_state_strings[arm->core_state],
		debug_reason_name(target),
		armv8_mode_name(arm->core_mode),
		buf_get_u32(arm->cpsr->value, 0, 32),
		buf_get_u64(arm->pc->value, 0, 64),
		arm->is_semihosting ? ", semihosting" : "");

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

	if (arm->core_state == ARM_STATE_AARCH64)
		armv8_aarch64_state(target);
	else
		arm_arch_state(target);

	LOG_USER("MMU: %s, D-Cache: %s, I-Cache: %s",
		state[armv8->armv8_mmu.mmu_enabled],
		state[armv8->armv8_mmu.armv8_cache.d_u_cache_enabled],
		state[armv8->armv8_mmu.armv8_cache.i_cache_enabled]);

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
	struct arm *arm = target_to_arm(target);
	uint64_t value = buf_get_u64(buf, 0, 64);

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	if (reg == arm->cpsr) {
		armv8_set_cpsr(arm, (uint32_t)value);
	} else {
		buf_set_u64(reg->value, 0, 64, value);
		reg->valid = 1;
	}

	reg->dirty = 1;

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

	if (regnum > (ARMV8_LAST_REG - 1))
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
		*reg_list_size = ARMV8_LAST_REG;
		*reg_list = malloc(sizeof(struct reg *) * (*reg_list_size));

		for (i = 0; i < ARMV8_LAST_REG; i++)
				(*reg_list)[i] = armv8_reg_current(arm, i);

		return ERROR_OK;

	default:
		LOG_ERROR("not a valid register class type in query.");
		return ERROR_FAIL;
		break;
	}
}
