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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef ARMV7A_H
#define ARMV7A_H

#include "arm_adi_v5.h"
#include "arm.h"
#include "armv4_5_mmu.h"
#include "armv4_5_cache.h"
#include "arm_dpm.h"

enum
{
	ARM_PC  = 15,
	ARM_CPSR = 16
}
;

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

struct armv7a_common
{
	struct arm armv4_5_common;
	int common_magic;
	struct reg_cache *core_cache;

	struct adiv5_dap dap;

	/* Core Debug Unit */
	struct arm_dpm dpm;
	uint32_t debug_base;
	uint8_t debug_ap;
	uint8_t memory_ap;

	/* Cache and Memory Management Unit */
	struct armv4_5_mmu_common armv4_5_mmu;

	int (*examine_debug_reason)(struct target *target);
	int (*post_debug_entry)(struct target *target);

	void (*pre_restore_context)(struct target *target);
};

static inline struct armv7a_common *
target_to_armv7a(struct target *target)
{
	return container_of(target->arch_info, struct armv7a_common,
			armv4_5_common);
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
struct reg_cache *armv7a_build_reg_cache(struct target *target,
		struct armv7a_common *armv7a_common);
int armv7a_init_arch_info(struct target *target, struct armv7a_common *armv7a);

extern const struct command_registration armv7a_command_handlers[];

#endif /* ARMV4_5_H */
