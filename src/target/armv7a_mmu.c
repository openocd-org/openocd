/***************************************************************************
 *   Copyright (C) 2016 by Matthias Welwarsky                              *
 *   matthias.welwarsky@sysgo.com                                          *
 *                                                                         *
 *   Copyright (C) ST-Ericsson SA 2011 michel.jaouen@stericsson.com        *
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

#include <helper/binarybuffer.h>
#include <helper/command.h>

#include "jtag/interface.h"
#include "arm.h"
#include "armv7a.h"
#include "armv7a_mmu.h"
#include "arm_opcodes.h"
#include "cortex_a.h"

#define SCTLR_BIT_AFE (1 << 29)

/*  method adapted to Cortex-A : reused ARM v4 v5 method */
int armv7a_mmu_translate_va(struct target *target,  uint32_t va, uint32_t *val)
{
	uint32_t first_lvl_descriptor = 0x0;
	uint32_t second_lvl_descriptor = 0x0;
	int retval;
	struct armv7a_common *armv7a = target_to_armv7a(target);
	uint32_t ttbidx = 0;	/*  default to ttbr0 */
	uint32_t ttb_mask;
	uint32_t va_mask;
	uint32_t ttb;

	if (target->state != TARGET_HALTED)
		LOG_INFO("target not halted, using cached values for translation table!");

	/* if va is above the range handled by ttbr0, select ttbr1 */
	if (va > armv7a->armv7a_mmu.ttbr_range[0]) {
		/*  select ttb 1 */
		ttbidx = 1;
	}

	ttb = armv7a->armv7a_mmu.ttbr[ttbidx];
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
		/* Avoid LOG_ERROR, probably GDB is guessing the stack frame */
		LOG_WARNING("Address translation failure [1]: va %8.8" PRIx32 "", va);
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
		/* Avoid LOG_ERROR, probably GDB is guessing the stack frame */
		LOG_WARNING("Address translation failure [2]: va %8.8" PRIx32 "", va);
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

static const char *desc_bits_to_string(bool c_bit, bool b_bit, bool s_bit, bool ap2, int ap10, bool afe)
{
	static char bits_string[64];
	unsigned int len;

	if (afe) {
		bool acc_r = true;
		bool acc_w = !ap2;
		bool priv = !(ap10 & 2);
		len = snprintf(bits_string, sizeof(bits_string), "%s%s%s access%s: %s%s",
					   s_bit ? "S " : "", c_bit ? "C " : "", b_bit ? "B " : "",
				 priv ? "(priv)" : "", acc_r ? "R" : "N", acc_w ? "W " : "O ");
	} else {
		bool priv_acc_w = !ap2;
		bool priv_acc_r = true;
		bool unpriv_acc_w = priv_acc_w;
		bool unpriv_acc_r = priv_acc_r;

		switch (ap10) {
		case 0:
			priv_acc_r = priv_acc_w = false;
			unpriv_acc_r = unpriv_acc_w = false;
			break;
		case 1:
			unpriv_acc_r = unpriv_acc_w = false;
			break;
		case 2:
			unpriv_acc_w = false;
			break;
		default:
			break;
		}

		len = snprintf(bits_string, sizeof(bits_string), "%s%s%s access(priv): %s%s access(unpriv): %s%s",
				s_bit ? "S " : "", c_bit ? "C " : "", b_bit ? "B " : "", priv_acc_r ? "R" : "N", priv_acc_w ? "W" : "O",
				unpriv_acc_r ? "R" : "N", unpriv_acc_w ? "W" : "O");
	}

	if (len >= sizeof(bits_string))
		bits_string[63] = 0;

	return bits_string;
}

static const char *l2_desc_bits_to_string(uint32_t l2_desc, bool afe)
{
	bool c_bit = !!(l2_desc & (1 << 3));
	bool b_bit = !!(l2_desc & (1 << 2));
	bool s_bit = !!(l2_desc & (1 << 10));
	bool ap2 = !!(l2_desc & (1 << 9));
	int ap10 = (l2_desc >> 4) & 3;

	return desc_bits_to_string(c_bit, b_bit, s_bit, ap2, ap10, afe);
}

static const char *l1_desc_bits_to_string(uint32_t l1_desc, bool afe)
{
	bool c_bit = !!(l1_desc & (1 << 3));
	bool b_bit = !!(l1_desc & (1 << 2));
	bool s_bit = !!(l1_desc & (1 << 16));
	bool ap2 = !!(l1_desc & (1 << 15));
	int ap10 = (l1_desc >> 10) & 3;

	return desc_bits_to_string(c_bit, b_bit, s_bit, ap2, ap10, afe);
}

COMMAND_HANDLER(armv7a_mmu_dump_table)
{
	struct target *target = get_current_target(CMD_CTX);
	struct cortex_a_common *cortex_a = target_to_cortex_a(target);
	struct armv7a_common *armv7a = target_to_armv7a(target);
	struct armv7a_mmu_common *mmu = &armv7a->armv7a_mmu;
	struct armv7a_cache_common *cache = &mmu->armv7a_cache;
	uint32_t *first_lvl_ptbl;
	target_addr_t ttb;
	int ttbidx = 0;
	int retval;
	int pt_idx;
	int max_pt_idx = 4095;
	bool afe;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (!strcmp(CMD_ARGV[0], "addr")) {
		if (CMD_ARGC < 2)
			return ERROR_COMMAND_SYNTAX_ERROR;

		COMMAND_PARSE_NUMBER(target_addr, CMD_ARGV[1], ttb);

		if (CMD_ARGC > 2) {
			COMMAND_PARSE_NUMBER(int, CMD_ARGV[2], max_pt_idx);

			if (max_pt_idx < 1 || max_pt_idx > 4096)
				return ERROR_COMMAND_ARGUMENT_INVALID;
			max_pt_idx -= 1;
		}
	} else {
		if (mmu->cached != 1) {
			LOG_ERROR("TTB not cached!");
			return ERROR_FAIL;
		}

		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], ttbidx);
		if (ttbidx < 0 || ttbidx > 1)
			return ERROR_COMMAND_ARGUMENT_INVALID;

		ttb = mmu->ttbr[ttbidx] & mmu->ttbr_mask[ttbidx];

		if (ttbidx == 0) {
			int ttbcr_n = mmu->ttbcr & 0x7;
			max_pt_idx = 0x0fff >> ttbcr_n;
		}
	}

	LOG_USER("Page Directory at (phys): %8.8" TARGET_PRIxADDR, ttb);

	first_lvl_ptbl = malloc(sizeof(uint32_t)*(max_pt_idx+1));
	if (first_lvl_ptbl == NULL)
		return ERROR_FAIL;

	/*
	 * this may or may not be necessary depending on whether
	 * the table walker is configured to use the cache or not.
	 */
	cache->flush_all_data_cache(target);

	retval = mmu->read_physical_memory(target, ttb, 4, max_pt_idx+1, (uint8_t *)first_lvl_ptbl);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read first-level page table!");
		return retval;
	}

	afe = !!(cortex_a->cp15_control_reg & SCTLR_BIT_AFE);

	for (pt_idx = 0; pt_idx <= max_pt_idx;) {
		uint32_t first_lvl_descriptor = target_buffer_get_u32(target,
						(uint8_t *)&first_lvl_ptbl[pt_idx]);

		LOG_DEBUG("L1 desc[%8.8"PRIx32"]: %8.8"PRIx32, pt_idx << 20, first_lvl_descriptor);

		/* skip empty entries in the first level table */
		if ((first_lvl_descriptor & 3) == 0) {
			pt_idx++;
		} else
		if ((first_lvl_descriptor & 0x40002) == 2) {
			/* section descriptor */
			uint32_t va_range = 1024*1024-1; /* 1MB range */
			uint32_t va_start = pt_idx << 20;
			uint32_t va_end = va_start + va_range;

			uint32_t pa_start = (first_lvl_descriptor & 0xfff00000);
			uint32_t pa_end = pa_start + va_range;

			LOG_USER("SECT: VA[%8.8"PRIx32" -- %8.8"PRIx32"]: PA[%8.8"PRIx32" -- %8.8"PRIx32"] %s",
				va_start, va_end, pa_start, pa_end, l1_desc_bits_to_string(first_lvl_descriptor, afe));
			pt_idx++;
		} else
		if ((first_lvl_descriptor & 0x40002) == 0x40002) {
			/* supersection descriptor */
			uint32_t va_range = 16*1024*1024-1; /* 16MB range */
			uint32_t va_start = pt_idx << 20;
			uint32_t va_end = va_start + va_range;

			uint32_t pa_start = (first_lvl_descriptor & 0xff000000);
			uint32_t pa_end = pa_start + va_range;

			LOG_USER("SSCT: VA[%8.8"PRIx32" -- %8.8"PRIx32"]: PA[%8.8"PRIx32" -- %8.8"PRIx32"] %s",
				va_start, va_end, pa_start, pa_end, l1_desc_bits_to_string(first_lvl_descriptor, afe));

			/* skip next 15 entries, they're duplicating the first entry */
			pt_idx += 16;
		} else {
			target_addr_t second_lvl_ptbl = first_lvl_descriptor & 0xfffffc00;
			uint32_t second_lvl_descriptor;
			uint32_t *pt2;
			int pt2_idx;

			/* page table, always 1KB long */
			pt2 = malloc(1024);
			retval = mmu->read_physical_memory(target, second_lvl_ptbl,
						  4, 256, (uint8_t *)pt2);
			if (retval != ERROR_OK) {
				LOG_ERROR("Failed to read second-level page table!");
				return ERROR_FAIL;
			}

			for (pt2_idx = 0; pt2_idx < 256; ) {
				second_lvl_descriptor = target_buffer_get_u32(target,
						(uint8_t *)&pt2[pt2_idx]);

				if ((second_lvl_descriptor & 3) == 0) {
					/* skip entry */
					pt2_idx++;
				} else
				if ((second_lvl_descriptor & 3) == 1) {
					/* large page */
					uint32_t va_range = 64*1024-1; /* 64KB range */
					uint32_t va_start = (pt_idx << 20) + (pt2_idx << 12);
					uint32_t va_end = va_start + va_range;

					uint32_t pa_start = (second_lvl_descriptor & 0xffff0000);
					uint32_t pa_end = pa_start + va_range;

					LOG_USER("LPGE: VA[%8.8"PRIx32" -- %8.8"PRIx32"]: PA[%8.8"PRIx32" -- %8.8"PRIx32"] %s",
						va_start, va_end, pa_start, pa_end, l2_desc_bits_to_string(second_lvl_descriptor, afe));

					pt2_idx += 16;
				} else {
					/* small page */
					uint32_t va_range = 4*1024-1; /* 4KB range */
					uint32_t va_start = (pt_idx << 20) + (pt2_idx << 12);
					uint32_t va_end = va_start + va_range;

					uint32_t pa_start = (second_lvl_descriptor & 0xfffff000);
					uint32_t pa_end = pa_start + va_range;

					LOG_USER("SPGE: VA[%8.8"PRIx32" -- %8.8"PRIx32"]: PA[%8.8"PRIx32" -- %8.8"PRIx32"] %s",
						va_start, va_end, pa_start, pa_end, l2_desc_bits_to_string(second_lvl_descriptor, afe));

					pt2_idx++;
				}
			}
			free(pt2);
			pt_idx++;
		}
	}

	free(first_lvl_ptbl);
	return ERROR_OK;
}

static const struct command_registration armv7a_mmu_group_handlers[] = {
	{
		.name = "dump",
		.handler = armv7a_mmu_dump_table,
		.mode = COMMAND_ANY,
		.help = "dump translation table 0, 1 or from <address>",
		.usage = "(0|1|addr <address> [num_entries])",
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration armv7a_mmu_command_handlers[] = {
	{
		.name = "mmu",
		.mode = COMMAND_ANY,
		.help = "mmu command group",
		.usage = "",
		.chain = armv7a_mmu_group_handlers,
	},
	COMMAND_REGISTRATION_DONE
};
