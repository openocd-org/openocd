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

#ifndef ARM920T_H
#define ARM920T_H

#include "arm9tdmi.h"
#include "armv4_5_mmu.h"

#define	ARM920T_COMMON_MAGIC 0xa920a920

struct arm920t_common {
	struct arm7_9_common arm7_9_common;
	uint32_t common_magic;
	struct armv4_5_mmu_common armv4_5_mmu;
	uint32_t cp15_control_reg;
	uint32_t d_fsr;
	uint32_t i_fsr;
	uint32_t d_far;
	uint32_t i_far;
	int preserve_cache;
};

static inline struct arm920t_common *target_to_arm920(struct target *target)
{
	return container_of(target->arch_info, struct arm920t_common, arm7_9_common.arm);
}

struct arm920t_cache_line {
	uint32_t cam;
	uint32_t data[8];
};

struct arm920t_tlb_entry {
	uint32_t cam;
	uint32_t ram1;
	uint32_t ram2;
};

int arm920t_arch_state(struct target *target);
int arm920t_soft_reset_halt(struct target *target);
int arm920t_read_memory(struct target *target,
	uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer);
int arm920t_write_memory(struct target *target,
	uint32_t address, uint32_t size, uint32_t count, const uint8_t *buffer);
int arm920t_post_debug_entry(struct target *target);
void arm920t_pre_restore_context(struct target *target);
int arm920t_get_ttb(struct target *target, uint32_t *result);
int arm920t_disable_mmu_caches(struct target *target,
	int mmu, int d_u_cache, int i_cache);
int arm920t_enable_mmu_caches(struct target *target,
	int mmu, int d_u_cache, int i_cache);

extern const struct command_registration arm920t_command_handlers[];

#endif /* ARM920T_H */
