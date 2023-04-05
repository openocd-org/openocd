/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2016 by Matthias Welwarsky                              *
 *   matthias.welwarsky@sysgo.com                                          *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ARMV7A_MMU_H
#define OPENOCD_TARGET_ARMV7A_MMU_H

extern int armv7a_mmu_translate_va_pa(struct target *target, uint32_t va,
	target_addr_t *val, int meminfo);

extern const struct command_registration armv7a_mmu_command_handlers[];

#endif /* OPENOCD_TARGET_ARMV7A_MMU_H */
