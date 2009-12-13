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

struct target;

struct armv4_5_mmu_common
{
	uint32_t (*get_ttb)(struct target *target);
	int (*read_memory)(struct target *target, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer);
	int (*write_memory)(struct target *target, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer);
	void (*disable_mmu_caches)(struct target *target, int mmu, int d_u_cache, int i_cache);
	void (*enable_mmu_caches)(struct target *target, int mmu, int d_u_cache, int i_cache);
	struct armv4_5_cache_common armv4_5_cache;
	int has_tiny_pages;
	int mmu_enabled;
};

enum
{
	ARMV4_5_SECTION, ARMV4_5_LARGE_PAGE, ARMV4_5_SMALL_PAGE, ARMV4_5_TINY_PAGE
};

extern char* armv4_5_page_type_names[];

uint32_t armv4_5_mmu_translate_va(struct target *target,
		struct armv4_5_mmu_common *armv4_5_mmu, uint32_t va, int *type,
		uint32_t *cb, int *domain, uint32_t *ap);

int armv4_5_mmu_read_physical(struct target *target,
		struct armv4_5_mmu_common *armv4_5_mmu,
		uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer);

int armv4_5_mmu_write_physical(struct target *target,
		struct armv4_5_mmu_common *armv4_5_mmu,
		uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer);

enum
{
	ARMV4_5_MMU_ENABLED = 0x1,
	ARMV4_5_ALIGNMENT_CHECK = 0x2,
	ARMV4_5_MMU_S_BIT = 0x100,
	ARMV4_5_MMU_R_BIT = 0x200
};

#endif /* ARMV4_5_MMU_H */
