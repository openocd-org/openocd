/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2015 Oleksij Rempel                                     *
 *   linux@rempel-privat.de                                                *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ARM7A_CACHE_H
#define OPENOCD_TARGET_ARM7A_CACHE_H

#include "arm_jtag.h"
#include "armv7a_cache_l2x.h"

int armv7a_l1_d_cache_clean_virt(struct target *target, uint32_t virt,
					unsigned int size);
int armv7a_l1_d_cache_inval_virt(struct target *target, uint32_t virt,
					uint32_t size);
int armv7a_l1_d_cache_flush_virt(struct target *target, uint32_t virt,
					unsigned int size);
int armv7a_l1_i_cache_inval_all(struct target *target);
int armv7a_l1_i_cache_inval_virt(struct target *target, uint32_t virt,
					uint32_t size);
int armv7a_cache_auto_flush_on_write(struct target *target, uint32_t virt,
					uint32_t size);
int armv7a_cache_auto_flush_all_data(struct target *target);
int armv7a_cache_flush_virt(struct target *target, uint32_t virt,
				uint32_t size);
extern const struct command_registration arm7a_cache_command_handlers[];

/* CLIDR cache types */
#define CACHE_LEVEL_HAS_UNIFIED_CACHE	0x4
#define CACHE_LEVEL_HAS_D_CACHE		0x2
#define CACHE_LEVEL_HAS_I_CACHE		0x1

#endif /* OPENOCD_TARGET_ARM7A_CACHE_H */
