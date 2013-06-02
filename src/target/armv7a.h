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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifndef ARMV7A_H
#define ARMV7A_H

#include "arm_adi_v5.h"
#include "arm.h"
#include "armv4_5_mmu.h"
#include "armv4_5_cache.h"
#include "arm_dpm.h"

enum {
	ARM_PC  = 15,
	ARM_CPSR = 16
};

#define ARMV7_COMMON_MAGIC 0x0A450999

/* VA to PA translation operations opc2 values*/
#define V2PCWPR  0
#define V2PCWPW  1
#define V2PCWUR  2
#define V2PCWUW  3
#define V2POWPR  4
#define V2POWPW  5
#define V2POWUR  6
#define V2POWUW  7
/*   L210/L220 cache controller support */
struct armv7a_l2x_cache {
	uint32_t base;
	uint32_t way;
};

struct armv7a_cachesize {
	uint32_t level_num;
	/*  cache dimensionning */
	uint32_t linelen;
	uint32_t associativity;
	uint32_t nsets;
	uint32_t cachesize;
	/* info for set way operation on cache */
	uint32_t index;
	uint32_t index_shift;
	uint32_t way;
	uint32_t way_shift;
};

struct armv7a_cache_common {
	int ctype;
	struct armv7a_cachesize d_u_size;	/* data cache */
	struct armv7a_cachesize i_size;		/* instruction cache */
	int i_cache_enabled;
	int d_u_cache_enabled;
	/* l2 external unified cache if some */
	void *l2_cache;
	int (*flush_all_data_cache)(struct target *target);
	int (*display_cache_info)(struct command_context *cmd_ctx,
			struct armv7a_cache_common *armv7a_cache);
};

struct armv7a_mmu_common {
	/* following field mmu working way */
	int32_t ttbr1_used; /*  -1 not initialized, 0 no ttbr1 1 ttbr1 used and  */
	uint32_t ttbr0_mask;/*  masked to be used  */
	uint32_t os_border;

	int (*read_physical_memory)(struct target *target, uint32_t address, uint32_t size,
			uint32_t count, uint8_t *buffer);
	struct armv7a_cache_common armv7a_cache;
	uint32_t mmu_enabled;
};

struct armv7a_common {
	struct arm arm;
	int common_magic;
	struct reg_cache *core_cache;

	struct adiv5_dap dap;

	/* Core Debug Unit */
	struct arm_dpm dpm;
	uint32_t debug_base;
	uint8_t debug_ap;
	uint8_t memory_ap;
	bool memory_ap_available;
	/* mdir */
	uint8_t multi_processor_system;
	uint8_t cluster_id;
	uint8_t cpu_id;
	bool is_armv7r;

	/* cache specific to V7 Memory Management Unit compatible with v4_5*/
	struct armv7a_mmu_common armv7a_mmu;

	int (*examine_debug_reason)(struct target *target);
	int (*post_debug_entry)(struct target *target);

	void (*pre_restore_context)(struct target *target);
};

static inline struct armv7a_common *
target_to_armv7a(struct target *target)
{
	return container_of(target->arch_info, struct armv7a_common, arm);
}

/* register offsets from armv7a.debug_base */

/* See ARMv7a arch spec section C10.2 */
#define CPUDBG_DIDR		0x000

/* See ARMv7a arch spec section C10.3 */
#define CPUDBG_WFAR		0x018
/* PCSR at 0x084 -or- 0x0a0 -or- both ... based on flags in DIDR */
#define CPUDBG_DSCR		0x088
#define CPUDBG_DRCR		0x090
#define CPUDBG_PRCR		0x310
#define CPUDBG_PRSR		0x314

/* See ARMv7a arch spec section C10.4 */
#define CPUDBG_DTRRX		0x080
#define CPUDBG_ITR		0x084
#define CPUDBG_DTRTX		0x08c

/* See ARMv7a arch spec section C10.5 */
#define CPUDBG_BVR_BASE		0x100
#define CPUDBG_BCR_BASE		0x140
#define CPUDBG_WVR_BASE		0x180
#define CPUDBG_WCR_BASE		0x1C0
#define CPUDBG_VCR		0x01C

/* See ARMv7a arch spec section C10.6 */
#define CPUDBG_OSLAR		0x300
#define CPUDBG_OSLSR		0x304
#define CPUDBG_OSSRR		0x308
#define CPUDBG_ECR		0x024

/* See ARMv7a arch spec section C10.7 */
#define CPUDBG_DSCCR		0x028

/* See ARMv7a arch spec section C10.8 */
#define CPUDBG_AUTHSTATUS	0xFB8

int armv7a_arch_state(struct target *target);
int armv7a_identify_cache(struct target *target);
int armv7a_init_arch_info(struct target *target, struct armv7a_common *armv7a);
int armv7a_mmu_translate_va_pa(struct target *target, uint32_t va,
		uint32_t *val, int meminfo);
int armv7a_mmu_translate_va(struct target *target,  uint32_t va, uint32_t *val);

int armv7a_handle_cache_info_command(struct command_context *cmd_ctx,
		struct armv7a_cache_common *armv7a_cache);

extern const struct command_registration armv7a_command_handlers[];

#endif /* ARMV4_5_H */
