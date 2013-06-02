/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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

#ifndef ARM720T_H
#define ARM720T_H

#include "arm7tdmi.h"
#include "armv4_5_mmu.h"

#define	ARM720T_COMMON_MAGIC 0xa720a720

struct arm720t_common {
	struct arm7_9_common arm7_9_common;
	uint32_t common_magic;
	struct armv4_5_mmu_common armv4_5_mmu;
	uint32_t cp15_control_reg;
	uint32_t fsr_reg;
	uint32_t far_reg;
};

static inline struct arm720t_common *target_to_arm720(struct target *target)
{
	return container_of(target->arch_info, struct arm720t_common, arm7_9_common.arm);
}

#endif /* ARM720T_H */
