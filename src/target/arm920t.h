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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef ARM920T_H
#define ARM920T_H

#include "arm9tdmi.h"
#include "armv4_5_mmu.h"

#define	ARM920T_COMMON_MAGIC 0xa920a920

typedef struct arm920t_common_s
{
	uint32_t common_magic;
	armv4_5_mmu_common_t armv4_5_mmu;
	arm9tdmi_common_t arm9tdmi_common;
	uint32_t cp15_control_reg;
	uint32_t d_fsr;
	uint32_t i_fsr;
	uint32_t d_far;
	uint32_t i_far;
	int preserve_cache;
} arm920t_common_t;

static inline struct arm920t_common_s *
target_to_arm920(struct target_s *target)
{
	return container_of(target->arch_info, struct arm920t_common_s,
			arm9tdmi_common.arm7_9_common.armv4_5_common);
}

typedef struct arm920t_cache_line_s
{
	uint32_t cam;
	uint32_t data[8];
} arm920t_cache_line_t;

typedef struct arm920t_tlb_entry_s
{
	uint32_t cam;
	uint32_t ram1;
	uint32_t ram2;
} arm920t_tlb_entry_t;

int arm920t_arch_state(struct target_s *target);
int arm920t_soft_reset_halt(struct target_s *target);
int arm920t_read_memory(struct target_s *target,
	uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer);
int arm920t_write_memory(struct target_s *target,
	uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer);
void arm920t_post_debug_entry(target_t *target);
void arm920t_pre_restore_context(target_t *target);
	uint32_t arm920t_get_ttb(target_t *target);
void arm920t_disable_mmu_caches(target_t *target,
	int mmu, int d_u_cache, int i_cache);
void arm920t_enable_mmu_caches(target_t *target,
	int mmu, int d_u_cache, int i_cache);
int arm920t_register_commands(struct command_context_s *cmd_ctx);

#endif /* ARM920T_H */
