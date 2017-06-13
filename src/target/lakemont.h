/*
 * Copyright(c) 2013-2016 Intel Corporation.
 *
 * Adrian Burns (adrian.burns@intel.com)
 * Thomas Faust (thomas.faust@intel.com)
 * Ivan De Cesaris (ivan.de.cesaris@intel.com)
 * Julien Carreno (julien.carreno@intel.com)
 * Jeffrey Maxwell (jeffrey.r.maxwell@intel.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Contact Information:
 * Intel Corporation
 */

/*
 * @file
 * This is the interface to the probemode operations for Lakemont 1 (LMT1).
 */

#ifndef OPENOCD_TARGET_LAKEMONT_H
#define OPENOCD_TARGET_LAKEMONT_H

#include <jtag/jtag.h>
#include <helper/types.h>

/* The Intel Quark SoC X1000 Core is codenamed lakemont */

#define LMT_IRLEN		8

/* lakemont tap instruction opcodes */
#define IDCODE			2
#define SUBMITPIR		3
#define PROBEMODE		4
#define WRPIR			6
#define RDWRPDR			8
#define TAPSTATUS		11
#define BYPASS			255
#define NOT_NULL		2

/* DR sizes */
#define ID_SIZE			32
#define PM_SIZE			1
#define PIR_SIZE		64
#define PDR_SIZE		32
#define TS_SIZE			32
#define BP_SIZE			1
#define MAX_SCAN_SIZE	PIR_SIZE

/* needed during lakemont probemode */
#define NOT_PMREG		0xfe
#define NOT_AVAIL_REG		0xff
#define PM_DSB			((uint32_t)0x00000000)
#define PM_DSL			((uint32_t)0xFFFFFFFF)
#define PM_DSAR			((uint32_t)0x004F9300)
#define PM_DR7			((uint32_t)0x00000400)
#define DELAY_SUBMITPIR		0 /* for now 0 is working */

/* lakemont tapstatus bits */
#define TS_PRDY_BIT		((uint32_t)0x00000001)
#define TS_EN_PM_BIT		((uint32_t)0x00000002)
#define TS_PM_BIT		((uint32_t)0x00000004)
#define TS_PMCR_BIT		((uint32_t)0x00000008)
#define TS_SBP_BIT		((uint32_t)0x00000010)

struct lakemont_core_reg {
	uint32_t num;
	struct target *target;
	struct x86_32_common *x86_32_common;
	uint64_t op;
	uint8_t pm_idx;
};

struct scan_blk {
	uint8_t out[MAX_SCAN_SIZE]; /* scanned out to the tap */
	uint8_t in[MAX_SCAN_SIZE]; /* in to our capture buf */
	struct scan_field field;
};

#define I(name) (((struct lakemont_core_reg *)x86_32->cache->reg_list[name].arch_info)->pm_idx)

int lakemont_init_target(struct command_context *cmd_ctx, struct target *t);
int lakemont_init_arch_info(struct target *t, struct x86_32_common *x86_32);
int lakemont_poll(struct target *t);
int lakemont_arch_state(struct target *t);
int lakemont_halt(struct target *t);
int lakemont_resume(struct target *t, int current, target_addr_t address,
			int handle_breakpoints, int debug_execution);
int lakemont_step(struct target *t, int current,
			target_addr_t address, int handle_breakpoints);
int lakemont_reset_assert(struct target *t);
int lakemont_reset_deassert(struct target *t);
int lakemont_update_after_probemode_entry(struct target *t);

#endif /* OPENOCD_TARGET_LAKEMONT_H */
