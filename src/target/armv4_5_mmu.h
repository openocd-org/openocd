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

#ifndef ARMV4_5_MMU_H
#define ARMV4_5_MMU_H

#include "armv4_5_cache.h"

struct target;

struct armv4_5_mmu_common {
	int (*get_ttb)(struct target *target, uint32_t *result);
	int (*read_memory)(struct target *target, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer);
	int (*write_memory)(struct target *target, uint32_t address, uint32_t size, uint32_t count, const uint8_t *buffer);
	int (*disable_mmu_caches)(struct target *target, int mmu, int d_u_cache, int i_cache);
	int (*enable_mmu_caches)(struct target *target, int mmu, int d_u_cache, int i_cache);
	struct armv4_5_cache_common armv4_5_cache;
	int has_tiny_pages;
	int mmu_enabled;
};

int armv4_5_mmu_translate_va(struct target *target,
		struct armv4_5_mmu_common *armv4_5_mmu, uint32_t va,
		uint32_t *cb, uint32_t *val);

int armv4_5_mmu_read_physical(struct target *target,
		struct armv4_5_mmu_common *armv4_5_mmu,
		uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer);

int armv4_5_mmu_write_physical(struct target *target,
		struct armv4_5_mmu_common *armv4_5_mmu,
		uint32_t address, uint32_t size, uint32_t count, const uint8_t *buffer);

enum {
	ARMV4_5_MMU_ENABLED = 0x1,
	ARMV4_5_ALIGNMENT_CHECK = 0x2,
	ARMV4_5_MMU_S_BIT = 0x100,
	ARMV4_5_MMU_R_BIT = 0x200
};

#endif /* ARMV4_5_MMU_H */
