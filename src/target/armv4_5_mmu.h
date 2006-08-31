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
#ifndef ARMV4_5_MMU_H
#define ARMV4_5_MMU_H

#include "armv4_5_cache.h"

typedef struct armv4_5_mmu_common_s
{
	u32 (*get_ttb)(target_t *target);
	int (*read_memory)(target_t *target, u32 address, u32 size, u32 count, u8 *buffer);
	int (*write_memory)(target_t *target, u32 address, u32 size, u32 count, u8 *buffer);
	void (*disable_mmu_caches)(target_t *target, int mmu, int d_u_cache, int i_cache);
	void (*enable_mmu_caches)(target_t *target, int mmu, int d_u_cache, int i_cache);
	armv4_5_cache_common_t armv4_5_cache;
	int has_tiny_pages;
	int mmu_enabled;
} armv4_5_mmu_common_t;

enum
{
	ARMV4_5_SECTION, ARMV4_5_LARGE_PAGE, ARMV4_5_SMALL_PAGE, ARMV4_5_TINY_PAGE
};

extern char* armv4_5_page_type_names[];

extern u32 armv4_5_mmu_translate_va(target_t *target, armv4_5_mmu_common_t *armv4_5_mmu, u32 va, int *type, u32 *cb, int *domain, u32 *ap);
extern int armv4_5_mmu_read_physical(target_t *target, armv4_5_mmu_common_t *armv4_5_mmu, u32 address, u32 size, u32 count, u8 *buffer);
extern int armv4_5_mmu_write_physical(target_t *target, armv4_5_mmu_common_t *armv4_5_mmu, u32 address, u32 size, u32 count, u8 *buffer);

extern int armv4_5_mmu_handle_virt2phys_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, target_t *target, armv4_5_mmu_common_t *armv4_5_mmu);
extern int armv4_5_mmu_handle_md_phys_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, target_t *target, armv4_5_mmu_common_t *armv4_5_mmu);
extern int armv4_5_mmu_handle_mw_phys_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, target_t *target, armv4_5_mmu_common_t *armv4_5_mmu);

enum
{
	ARMV4_5_MMU_ENABLED = 0x1,
	ARMV4_5_ALIGNMENT_CHECK = 0x2,
	ARMV4_5_MMU_S_BIT = 0x100,
	ARMV4_5_MMU_R_BIT = 0x200
};

#endif /* ARMV4_5_MMU_H */
