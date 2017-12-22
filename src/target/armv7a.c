/***************************************************************************
 *    Copyright (C) 2009 by David Brownell                                 *
 *                                                                         *
 *    Copyright (C) ST-Ericsson SA 2011 michel.jaouen@stericsson.com       *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
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
#include "target.h"
#include "target_type.h"

static void armv7a_show_fault_registers(struct target *target)
{
	uint32_t dfsr, ifsr, dfar, ifar;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm_dpm *dpm = armv7a->arm.dpm;
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


/*  retrieve main id register  */
static int armv7a_read_midr(struct target *target)
{
	int retval = ERROR_FAIL;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm_dpm *dpm = armv7a->arm.dpm;
	uint32_t midr;
	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;
	/* MRC p15,0,<Rd>,c0,c0,0; read main id register*/

	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 0, 0, 0),
			&midr);
	if (retval != ERROR_OK)
		goto done;

	armv7a->rev = (midr & 0xf);
	armv7a->partnum = (midr >> 4) & 0xfff;
	armv7a->arch = (midr >> 16) & 0xf;
	armv7a->variant = (midr >> 20) & 0xf;
	armv7a->implementor = (midr >> 24) & 0xff;
	LOG_INFO("%s rev %" PRIx32 ", partnum %" PRIx32 ", arch %" PRIx32
			 ", variant %" PRIx32 ", implementor %" PRIx32,
		 target->cmd_name,
		 armv7a->rev,
		 armv7a->partnum,
		 armv7a->arch,
		 armv7a->variant,
		 armv7a->implementor);

done:
	dpm->finish(dpm);
	return retval;
}

static int armv7a_read_ttbcr(struct target *target)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm_dpm *dpm = armv7a->arm.dpm;
	uint32_t ttbcr, ttbcr_n;
	int retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;
	/*  MRC p15,0,<Rt>,c2,c0,2 ; Read CP15 Translation Table Base Control Register*/
	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 2, 0, 2),
			&ttbcr);
	if (retval != ERROR_OK)
		goto done;

	LOG_DEBUG("ttbcr %" PRIx32, ttbcr);

	ttbcr_n = ttbcr & 0x7;
	armv7a->armv7a_mmu.ttbcr = ttbcr;
	armv7a->armv7a_mmu.cached = 1;

	/*
	 * ARM Architecture Reference Manual (ARMv7-A and ARMv7-Redition),
	 * document # ARM DDI 0406C
	 */
	armv7a->armv7a_mmu.ttbr_range[0]  = 0xffffffff >> ttbcr_n;
	armv7a->armv7a_mmu.ttbr_range[1] = 0xffffffff;
	armv7a->armv7a_mmu.ttbr_mask[0] = 0xffffffff << (14 - ttbcr_n);
	armv7a->armv7a_mmu.ttbr_mask[1] = 0xffffffff << 14;
	armv7a->armv7a_mmu.cached = 1;

	retval = armv7a_read_midr(target);
	if (retval != ERROR_OK)
		goto done;

	/* FIXME: why this special case based on part number? */
	if ((armv7a->partnum & 0xf) == 0) {
		/*  ARM DDI 0344H , ARM DDI 0407F */
		armv7a->armv7a_mmu.ttbr_mask[0]  = 7 << (32 - ttbcr_n);
	}

	LOG_DEBUG("ttbr1 %s, ttbr0_mask %" PRIx32 " ttbr1_mask %" PRIx32,
		  (ttbcr_n != 0) ? "used" : "not used",
		  armv7a->armv7a_mmu.ttbr_mask[0],
		  armv7a->armv7a_mmu.ttbr_mask[1]);

done:
	dpm->finish(dpm);
	return retval;
}

/*  method adapted to Cortex-A : reused ARM v4 v5 method */
int armv7a_mmu_translate_va(struct target *target,  uint32_t va, uint32_t *val)
{
	uint32_t first_lvl_descriptor = 0x0;
	uint32_t second_lvl_descriptor = 0x0;
	int retval;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm_dpm *dpm = armv7a->arm.dpm;
	uint32_t ttbidx = 0;	/*  default to ttbr0 */
	uint32_t ttb_mask;
	uint32_t va_mask;
	uint32_t ttbcr;
	uint32_t ttb;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;

	/*  MRC p15,0,<Rt>,c2,c0,2 ; Read CP15 Translation Table Base Control Register*/
	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 2, 0, 2),
			&ttbcr);
	if (retval != ERROR_OK)
		goto done;

	/* if ttbcr has changed or was not read before, re-read the information */
	if ((armv7a->armv7a_mmu.cached == 0) ||
		(armv7a->armv7a_mmu.ttbcr != ttbcr)) {
		armv7a_read_ttbcr(target);
	}

	/* if va is above the range handled by ttbr0, select ttbr1 */
	if (va > armv7a->armv7a_mmu.ttbr_range[0]) {
		/*  select ttb 1 */
		ttbidx = 1;
	}
	/*  MRC p15,0,<Rt>,c2,c0,ttbidx */
	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 2, 0, ttbidx),
			&ttb);
	if (retval != ERROR_OK)
		return retval;

	ttb_mask = armv7a->armv7a_mmu.ttbr_mask[ttbidx];
	va_mask = 0xfff00000 & armv7a->armv7a_mmu.ttbr_range[ttbidx];

	LOG_DEBUG("ttb_mask %" PRIx32 " va_mask %" PRIx32 " ttbidx %i",
		  ttb_mask, va_mask, ttbidx);
	retval = armv7a->armv7a_mmu.read_physical_memory(target,
			(ttb & ttb_mask) | ((va & va_mask) >> 18),
			4, 1, (uint8_t *)&first_lvl_descriptor);
	if (retval != ERROR_OK)
		return retval;
	first_lvl_descriptor = target_buffer_get_u32(target, (uint8_t *)
			&first_lvl_descriptor);
	/*  reuse armv4_5 piece of code, specific armv7a changes may come later */
	LOG_DEBUG("1st lvl desc: %8.8" PRIx32 "", first_lvl_descriptor);

	if ((first_lvl_descriptor & 0x3) == 0) {
		LOG_ERROR("Address translation failure");
		return ERROR_TARGET_TRANSLATION_FAULT;
	}


	if ((first_lvl_descriptor & 0x40002) == 2) {
		/* section descriptor */
		*val = (first_lvl_descriptor & 0xfff00000) | (va & 0x000fffff);
		return ERROR_OK;
	} else if ((first_lvl_descriptor & 0x40002) == 0x40002) {
		/* supersection descriptor */
		if (first_lvl_descriptor & 0x00f001e0) {
			LOG_ERROR("Physical address does not fit into 32 bits");
			return ERROR_TARGET_TRANSLATION_FAULT;
		}
		*val = (first_lvl_descriptor & 0xff000000) | (va & 0x00ffffff);
		return ERROR_OK;
	}

	/* page table */
	retval = armv7a->armv7a_mmu.read_physical_memory(target,
			(first_lvl_descriptor & 0xfffffc00) | ((va & 0x000ff000) >> 10),
			4, 1, (uint8_t *)&second_lvl_descriptor);
	if (retval != ERROR_OK)
		return retval;

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
	} else {
		/* small page descriptor */
		*val = (second_lvl_descriptor & 0xfffff000) | (va & 0x00000fff);
	}

	return ERROR_OK;

done:
	return retval;
}

/*  V7 method VA TO PA  */
int armv7a_mmu_translate_va_pa(struct target *target, uint32_t va,
	uint32_t *val, int meminfo)
{
	int retval = ERROR_FAIL;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm_dpm *dpm = armv7a->arm.dpm;
	uint32_t virt = va & ~0xfff;
	uint32_t NOS, NS, INNER, OUTER;
	*val = 0xdeadbeef;
	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;
	/*  mmu must be enable in order to get a correct translation
	 *  use VA to PA CP15 register for conversion */
	retval = dpm->instr_write_data_r0(dpm,
			ARMV4_5_MCR(15, 0, 0, 7, 8, 0),
			virt);
	if (retval != ERROR_OK)
		goto done;
	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 7, 4, 0),
			val);
	/* decode memory attribute */
	NOS = (*val >> 10) & 1;	/*  Not Outer shareable */
	NS = (*val >> 9) & 1;	/* Non secure */
	INNER = (*val >> 4) &  0x7;
	OUTER = (*val >> 2) & 0x3;

	if (retval != ERROR_OK)
		goto done;
	*val = (*val & ~0xfff)  +  (va & 0xfff);
	if (*val == va)
		LOG_WARNING("virt = phys  : MMU disable !!");
	if (meminfo) {
		LOG_INFO("%" PRIx32 " : %" PRIx32 " %s outer shareable %s secured",
			va, *val,
			NOS == 1 ? "not" : " ",
			NS == 1 ? "not" : "");
		switch (OUTER) {
			case 0:
				LOG_INFO("outer: Non-Cacheable");
				break;
			case 1:
				LOG_INFO("outer: Write-Back, Write-Allocate");
				break;
			case 2:
				LOG_INFO("outer: Write-Through, No Write-Allocate");
				break;
			case 3:
				LOG_INFO("outer: Write-Back, no Write-Allocate");
				break;
		}
		switch (INNER) {
			case 0:
				LOG_INFO("inner: Non-Cacheable");
				break;
			case 1:
				LOG_INFO("inner: Strongly-ordered");
				break;
			case 3:
				LOG_INFO("inner: Device");
				break;
			case 5:
				LOG_INFO("inner: Write-Back, Write-Allocate");
				break;
			case 6:
				LOG_INFO("inner:  Write-Through");
				break;
			case 7:
				LOG_INFO("inner: Write-Back, no Write-Allocate");
				break;
			default:
				LOG_INFO("inner: %" PRIx32 " ???", INNER);
		}
	}

done:
	dpm->finish(dpm);

	return retval;
}

/* FIXME: remove it */
static int armv7a_l2x_cache_init(struct target *target, uint32_t base, uint32_t way)
{
	struct armv7a_l2x_cache *l2x_cache;
	struct target_list *head = target->head;
	struct target *curr;

	struct armv7a_common *armv7a = target_to_armv7a(target);
	l2x_cache = calloc(1, sizeof(struct armv7a_l2x_cache));
	l2x_cache->base = base;
	l2x_cache->way = way;
	/*LOG_INFO("cache l2 initialized base %x  way %d",
	l2x_cache->base,l2x_cache->way);*/
	if (armv7a->armv7a_mmu.armv7a_cache.outer_cache)
		LOG_INFO("outer cache already initialized\n");
	armv7a->armv7a_mmu.armv7a_cache.outer_cache = l2x_cache;
	/*  initialize all target in this cluster (smp target)
	 *  l2 cache must be configured after smp declaration */
	while (head != (struct target_list *)NULL) {
		curr = head->target;
		if (curr != target) {
			armv7a = target_to_armv7a(curr);
			if (armv7a->armv7a_mmu.armv7a_cache.outer_cache)
				LOG_ERROR("smp target : outer cache already initialized\n");
			armv7a->armv7a_mmu.armv7a_cache.outer_cache = l2x_cache;
		}
		head = head->next;
	}
	return JIM_OK;
}

/* FIXME: remove it */
COMMAND_HANDLER(handle_cache_l2x)
{
	struct target *target = get_current_target(CMD_CTX);
	uint32_t base, way;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* command_print(CMD_CTX, "%s %s", CMD_ARGV[0], CMD_ARGV[1]); */
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], base);
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], way);

	/* AP address is in bits 31:24 of DP_SELECT */
	armv7a_l2x_cache_init(target, base, way);

	return ERROR_OK;
}

int armv7a_handle_cache_info_command(struct command_context *cmd_ctx,
	struct armv7a_cache_common *armv7a_cache)
{
	struct armv7a_l2x_cache *l2x_cache = (struct armv7a_l2x_cache *)
		(armv7a_cache->outer_cache);

	int cl;

	if (armv7a_cache->info == -1) {
		command_print(cmd_ctx, "cache not yet identified");
		return ERROR_OK;
	}

	for (cl = 0; cl < armv7a_cache->loc; cl++) {
		struct armv7a_arch_cache *arch = &(armv7a_cache->arch[cl]);

		if (arch->ctype & 1) {
			command_print(cmd_ctx,
				"L%d I-Cache: linelen %" PRIi32
				", associativity %" PRIi32
				", nsets %" PRIi32
				", cachesize %" PRId32 " KBytes",
				cl+1,
				arch->i_size.linelen,
				arch->i_size.associativity,
				arch->i_size.nsets,
				arch->i_size.cachesize);
		}

		if (arch->ctype >= 2) {
			command_print(cmd_ctx,
				"L%d D-Cache: linelen %" PRIi32
				", associativity %" PRIi32
				", nsets %" PRIi32
				", cachesize %" PRId32 " KBytes",
				cl+1,
				arch->d_u_size.linelen,
				arch->d_u_size.associativity,
				arch->d_u_size.nsets,
				arch->d_u_size.cachesize);
		}
	}

	if (l2x_cache != NULL)
		command_print(cmd_ctx, "Outer unified cache Base Address 0x%" PRIx32 ", %" PRId32 " ways",
			l2x_cache->base, l2x_cache->way);

	return ERROR_OK;
}

/*  retrieve core id cluster id  */
static int armv7a_read_mpidr(struct target *target)
{
	int retval = ERROR_FAIL;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm_dpm *dpm = armv7a->arm.dpm;
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

	/* ARMv7R uses a different format for MPIDR.
	 * When configured uniprocessor (most R cores) it reads as 0.
	 * This will need to be implemented for multiprocessor ARMv7R cores. */
	if (armv7a->is_armv7r) {
		if (mpidr)
			LOG_ERROR("MPIDR nonzero in ARMv7-R target");
		goto done;
	}

	if (mpidr & 1<<31) {
		armv7a->multi_processor_system = (mpidr >> 30) & 1;
		armv7a->cluster_id = (mpidr >> 8) & 0xf;
		armv7a->cpu_id = mpidr & 0x3;
		LOG_INFO("%s cluster %x core %x %s", target_name(target),
			armv7a->cluster_id,
			armv7a->cpu_id,
			armv7a->multi_processor_system == 0 ? "multi core" : "mono core");

	} else
		LOG_ERROR("MPIDR not in multiprocessor format");

done:
	dpm->finish(dpm);
	return retval;


}

static int get_cache_info(struct arm_dpm *dpm, int cl, int ct, uint32_t *cache_reg)
{
	int retval = ERROR_OK;

	/*  select cache level */
	retval = dpm->instr_write_data_r0(dpm,
			ARMV4_5_MCR(15, 2, 0, 0, 0, 0),
			(cl << 1) | (ct == 1 ? 1 : 0));
	if (retval != ERROR_OK)
		goto done;

	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 1, 0, 0, 0, 0),
			cache_reg);
 done:
	return retval;
}

static struct armv7a_cachesize decode_cache_reg(uint32_t cache_reg)
{
	struct armv7a_cachesize size;
	int i = 0;

	size.linelen = 16 << (cache_reg & 0x7);
	size.associativity = ((cache_reg >> 3) & 0x3ff) + 1;
	size.nsets = ((cache_reg >> 13) & 0x7fff) + 1;
	size.cachesize = size.linelen * size.associativity * size.nsets / 1024;

	/*  compute info for set way operation on cache */
	size.index_shift = (cache_reg & 0x7) + 4;
	size.index = (cache_reg >> 13) & 0x7fff;
	size.way = ((cache_reg >> 3) & 0x3ff);

	while (((size.way << i) & 0x80000000) == 0)
		i++;
	size.way_shift = i;

	return size;
}

int armv7a_identify_cache(struct target *target)
{
	/*  read cache descriptor */
	int retval = ERROR_FAIL;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm_dpm *dpm = armv7a->arm.dpm;
	uint32_t csselr, clidr, ctr;
	uint32_t cache_reg;
	int cl, ctype;
	struct armv7a_cache_common *cache =
		&(armv7a->armv7a_mmu.armv7a_cache);

	if (!armv7a->is_armv7r)
		armv7a_read_ttbcr(target);

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;

	/* retrieve CTR
	 * mrc p15, 0, r0, c0, c0, 1		@ read ctr */
	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 0, 0, 0, 0, 1),
			&ctr);
	if (retval != ERROR_OK)
		goto done;

	cache->iminline = 4UL << (ctr & 0xf);
	cache->dminline = 4UL << ((ctr & 0xf0000) >> 16);
	LOG_DEBUG("ctr %" PRIx32 " ctr.iminline %" PRId32 " ctr.dminline %" PRId32,
		 ctr, cache->iminline, cache->dminline);

	/*  retrieve CLIDR
	 *  mrc	p15, 1, r0, c0, c0, 1		@ read clidr */
	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 1, 0, 0, 0, 1),
			&clidr);
	if (retval != ERROR_OK)
		goto done;

	cache->loc = (clidr & 0x7000000) >> 24;
	LOG_DEBUG("Number of cache levels to PoC %" PRId32, cache->loc);

	/*  retrieve selected cache for later restore
	 *  MRC p15, 2,<Rd>, c0, c0, 0; Read CSSELR */
	retval = dpm->instr_read_data_r0(dpm,
			ARMV4_5_MRC(15, 2, 0, 0, 0, 0),
			&csselr);
	if (retval != ERROR_OK)
		goto done;

	/* retrieve all available inner caches */
	for (cl = 0; cl < cache->loc; clidr >>= 3, cl++) {

		/* isolate cache type at current level */
		ctype = clidr & 7;

		/* skip reserved values */
		if (ctype > CACHE_LEVEL_HAS_UNIFIED_CACHE)
			continue;

		/* separate d or unified d/i cache at this level ? */
		if (ctype & (CACHE_LEVEL_HAS_UNIFIED_CACHE | CACHE_LEVEL_HAS_D_CACHE)) {
			/* retrieve d-cache info */
			retval = get_cache_info(dpm, cl, 0, &cache_reg);
			if (retval != ERROR_OK)
				goto done;
			cache->arch[cl].d_u_size = decode_cache_reg(cache_reg);

			LOG_DEBUG("data/unified cache index %d << %d, way %d << %d",
					cache->arch[cl].d_u_size.index,
					cache->arch[cl].d_u_size.index_shift,
					cache->arch[cl].d_u_size.way,
					cache->arch[cl].d_u_size.way_shift);

			LOG_DEBUG("cacheline %d bytes %d KBytes asso %d ways",
					cache->arch[cl].d_u_size.linelen,
					cache->arch[cl].d_u_size.cachesize,
					cache->arch[cl].d_u_size.associativity);
		}

		/* separate i-cache at this level ? */
		if (ctype & CACHE_LEVEL_HAS_I_CACHE) {
			/* retrieve i-cache info */
			retval = get_cache_info(dpm, cl, 1, &cache_reg);
			if (retval != ERROR_OK)
				goto done;
			cache->arch[cl].i_size = decode_cache_reg(cache_reg);

			LOG_DEBUG("instruction cache index %d << %d, way %d << %d",
					cache->arch[cl].i_size.index,
					cache->arch[cl].i_size.index_shift,
					cache->arch[cl].i_size.way,
					cache->arch[cl].i_size.way_shift);

			LOG_DEBUG("cacheline %d bytes %d KBytes asso %d ways",
					cache->arch[cl].i_size.linelen,
					cache->arch[cl].i_size.cachesize,
					cache->arch[cl].i_size.associativity);
		}

		cache->arch[cl].ctype = ctype;
	}

	/*  restore selected cache  */
	dpm->instr_write_data_r0(dpm,
		ARMV4_5_MRC(15, 2, 0, 0, 0, 0),
		csselr);

	if (retval != ERROR_OK)
		goto done;

	/*  if no l2 cache initialize l1 data cache flush function function */
	if (armv7a->armv7a_mmu.armv7a_cache.flush_all_data_cache == NULL) {
		armv7a->armv7a_mmu.armv7a_cache.flush_all_data_cache =
			armv7a_cache_auto_flush_all_data;
	}

	armv7a->armv7a_mmu.armv7a_cache.info = 1;
done:
	dpm->finish(dpm);
	armv7a_read_mpidr(target);
	return retval;

}

static int armv7a_setup_semihosting(struct target *target, int enable)
{
	struct armv7a_common *armv7a = target_to_armv7a(target);
	uint32_t vcr;
	int ret;

	ret = mem_ap_read_atomic_u32(armv7a->debug_ap,
					 armv7a->debug_base + CPUDBG_VCR,
					 &vcr);
	if (ret < 0) {
		LOG_ERROR("Failed to read VCR register\n");
		return ret;
	}

	if (enable)
		vcr |= DBG_VCR_SVC_MASK;
	else
		vcr &= ~DBG_VCR_SVC_MASK;

	ret = mem_ap_write_atomic_u32(armv7a->debug_ap,
					  armv7a->debug_base + CPUDBG_VCR,
					  vcr);
	if (ret < 0)
		LOG_ERROR("Failed to write VCR register\n");

	return ret;
}

int armv7a_init_arch_info(struct target *target, struct armv7a_common *armv7a)
{
	struct arm *arm = &armv7a->arm;
	arm->arch_info = armv7a;
	target->arch_info = &armv7a->arm;
	arm->setup_semihosting = armv7a_setup_semihosting;
	/*  target is useful in all function arm v4 5 compatible */
	armv7a->arm.target = target;
	armv7a->arm.common_magic = ARM_COMMON_MAGIC;
	armv7a->common_magic = ARMV7_COMMON_MAGIC;
	armv7a->armv7a_mmu.armv7a_cache.info = -1;
	armv7a->armv7a_mmu.armv7a_cache.outer_cache = NULL;
	armv7a->armv7a_mmu.armv7a_cache.flush_all_data_cache = NULL;
	armv7a->armv7a_mmu.armv7a_cache.auto_cache_enabled = 1;
	return ERROR_OK;
}

int armv7a_arch_state(struct target *target)
{
	static const char *state[] = {
		"disabled", "enabled"
	};

	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct arm *arm = &armv7a->arm;

	if (armv7a->common_magic != ARMV7_COMMON_MAGIC) {
		LOG_ERROR("BUG: called for a non-ARMv7A target");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	arm_arch_state(target);

	if (armv7a->is_armv7r) {
		LOG_USER("D-Cache: %s, I-Cache: %s",
			state[armv7a->armv7a_mmu.armv7a_cache.d_u_cache_enabled],
			state[armv7a->armv7a_mmu.armv7a_cache.i_cache_enabled]);
	} else {
		LOG_USER("MMU: %s, D-Cache: %s, I-Cache: %s",
			state[armv7a->armv7a_mmu.mmu_enabled],
			state[armv7a->armv7a_mmu.armv7a_cache.d_u_cache_enabled],
			state[armv7a->armv7a_mmu.armv7a_cache.i_cache_enabled]);
	}

	if (arm->core_mode == ARM_MODE_ABT)
		armv7a_show_fault_registers(target);
	if (target->debug_reason == DBG_REASON_WATCHPOINT)
		LOG_USER("Watchpoint triggered at PC %#08x",
			(unsigned) armv7a->dpm.wp_pc);

	return ERROR_OK;
}

static const struct command_registration l2_cache_commands[] = {
	{
		.name = "l2x",
		.handler = handle_cache_l2x,
		.mode = COMMAND_EXEC,
		.help = "configure l2x cache "
			"",
		.usage = "[base_addr] [number_of_way]",
	},
	COMMAND_REGISTRATION_DONE

};

const struct command_registration l2x_cache_command_handlers[] = {
	{
		.name = "cache_config",
		.mode = COMMAND_EXEC,
		.help = "cache configuration for a target",
		.usage = "",
		.chain = l2_cache_commands,
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration armv7a_command_handlers[] = {
	{
		.chain = dap_command_handlers,
	},
	{
		.chain = l2x_cache_command_handlers,
	},
	{
		.chain = arm7a_cache_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};
