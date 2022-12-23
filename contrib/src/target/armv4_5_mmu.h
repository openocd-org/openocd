/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ARMV4_5_MMU_H
#define OPENOCD_TARGET_ARMV4_5_MMU_H

#include "armv4_5_cache.h"

struct target;

struct armv4_5_mmu_common {
	int (*get_ttb)(struct target *target, uint32_t *result);
	int (*read_memory)(struct target *target, target_addr_t address, uint32_t size, uint32_t count, uint8_t *buffer);
	int (*write_memory)(struct target *target, target_addr_t address,
			    uint32_t size, uint32_t count, const uint8_t *buffer);
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

#endif /* OPENOCD_TARGET_ARMV4_5_MMU_H */
