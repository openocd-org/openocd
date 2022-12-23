/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ARM966E_H
#define OPENOCD_TARGET_ARM966E_H

#include "arm9tdmi.h"

#define	ARM966E_COMMON_MAGIC 0x20f920f9U

struct arm966e_common {
	unsigned int common_magic;

	struct arm7_9_common arm7_9_common;
	uint32_t cp15_control_reg;
};

static inline struct arm966e_common *
target_to_arm966(struct target *target)
{
	return container_of(target->arch_info, struct arm966e_common,
			arm7_9_common.arm);
}

int arm966e_init_arch_info(struct target *target,
		struct arm966e_common *arm966e, struct jtag_tap *tap);
int arm966e_write_cp15(struct target *target, int reg_addr, uint32_t value);

extern const struct command_registration arm966e_command_handlers[];

#endif /* OPENOCD_TARGET_ARM966E_H */
