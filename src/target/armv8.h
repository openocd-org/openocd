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

#ifndef ARMV8_H
#define ARMV8_H

#include "arm_adi_v5.h"
#include "arm.h"
#include "armv4_5_mmu.h"
#include "armv4_5_cache.h"
#include "arm_dpm.h"

enum {
	ARMV8_R0,
	ARMV8_R1,
	ARMV8_R2,
	ARMV8_R3,
	ARMV8_R4,
	ARMV8_R5,
	ARMV8_R6,
	ARMV8_R7,
	ARMV8_R8,
	ARMV8_R9,
	ARMV8_R10,
	ARMV8_R11,
	ARMV8_R12,
	ARMV8_R13,
	ARMV8_R14,
	ARMV8_R15,
	ARMV8_R16,
	ARMV8_R17,
	ARMV8_R18,
	ARMV8_R19,
	ARMV8_R20,
	ARMV8_R21,
	ARMV8_R22,
	ARMV8_R23,
	ARMV8_R24,
	ARMV8_R25,
	ARMV8_R26,
	ARMV8_R27,
	ARMV8_R28,
	ARMV8_R29,
	ARMV8_R30,
	ARMV8_R31,

	ARMV8_PC = 32,
	ARMV8_xPSR = 33,

	ARMV8_LAST_REG,
};


#define ARMV8_COMMON_MAGIC 0x0A450AAA

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
struct armv8_l2x_cache {
	uint32_t base;
	uint32_t way;
};

struct armv8_cachesize {
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

struct armv8_cache_common {
	int ctype;
	struct armv8_cachesize d_u_size;	/* data cache */
	struct armv8_cachesize i_size;		/* instruction cache */
	int i_cache_enabled;
	int d_u_cache_enabled;
	/* l2 external unified cache if some */
	void *l2_cache;
	int (*flush_all_data_cache)(struct target *target);
	int (*display_cache_info)(struct command_context *cmd_ctx,
			struct armv8_cache_common *armv8_cache);
};

struct armv8_mmu_common {
	/* following field mmu working way */
	int32_t ttbr1_used; /*  -1 not initialized, 0 no ttbr1 1 ttbr1 used and  */
	uint32_t ttbr0_mask;/*  masked to be used  */
	uint32_t os_border;

	int (*read_physical_memory)(struct target *target, target_ulong address,
			uint32_t size, uint32_t count, uint8_t *buffer);
	struct armv8_cache_common armv8_cache;
	uint32_t mmu_enabled;
};

struct armv8_common {
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
	struct armv8_mmu_common armv8_mmu;

	/* Direct processor core register read and writes */
	int (*load_core_reg_u64)(struct target *target, uint32_t num, uint64_t *value);
	int (*store_core_reg_u64)(struct target *target, uint32_t num, uint64_t value);

	int (*examine_debug_reason)(struct target *target);
	int (*post_debug_entry)(struct target *target);

	void (*pre_restore_context)(struct target *target);
};

static inline struct armv8_common *
target_to_armv8(struct target *target)
{
	return container_of(target->arch_info, struct armv8_common, arm);
}

/* register offsets from armv8.debug_base */

#define CPUDBG_WFAR		0x018
/* PCSR at 0x084 -or- 0x0a0 -or- both ... based on flags in DIDR */
#define CPUDBG_DSCR		0x088
#define CPUDBG_DRCR		0x090
#define CPUDBG_PRCR		0x310
#define CPUDBG_PRSR		0x314

#define CPUDBG_DTRRX		0x080
#define CPUDBG_ITR		0x084
#define CPUDBG_DTRTX		0x08c

#define CPUDBG_BVR_BASE		0x400
#define CPUDBG_BCR_BASE		0x408
#define CPUDBG_WVR_BASE		0x180
#define CPUDBG_WCR_BASE		0x1C0
#define CPUDBG_VCR		0x01C

#define CPUDBG_OSLAR		0x300
#define CPUDBG_OSLSR		0x304
#define CPUDBG_OSSRR		0x308
#define CPUDBG_ECR		0x024

#define CPUDBG_DSCCR		0x028

#define CPUDBG_AUTHSTATUS	0xFB8

int armv8_arch_state(struct target *target);
int armv8_identify_cache(struct target *target);
int armv8_init_arch_info(struct target *target, struct armv8_common *armv8);
int armv8_mmu_translate_va_pa(struct target *target, target_ulong va,
		target_ulong *val, int meminfo);
int armv8_mmu_translate_va(struct target *target,  uint32_t va, uint32_t *val);

int armv8_handle_cache_info_command(struct command_context *cmd_ctx,
		struct armv8_cache_common *armv8_cache);

extern const struct command_registration armv8_command_handlers[];

#endif
