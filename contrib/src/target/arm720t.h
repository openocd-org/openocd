/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ARM720T_H
#define OPENOCD_TARGET_ARM720T_H

#include "arm7tdmi.h"
#include "armv4_5_mmu.h"

#define	ARM720T_COMMON_MAGIC 0xa720a720U

struct arm720t_common {
	unsigned int common_magic;

	struct arm7_9_common arm7_9_common;
	struct armv4_5_mmu_common armv4_5_mmu;
	uint32_t cp15_control_reg;
	uint32_t fsr_reg;
	uint32_t far_reg;
};

static inline struct arm720t_common *target_to_arm720(struct target *target)
{
	return container_of(target->arch_info, struct arm720t_common, arm7_9_common.arm);
}

#endif /* OPENOCD_TARGET_ARM720T_H */
