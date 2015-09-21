/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ARM966E_H
#define OPENOCD_TARGET_ARM966E_H

#include "arm9tdmi.h"

#define	ARM966E_COMMON_MAGIC 0x20f920f9

struct arm966e_common {
	struct arm7_9_common arm7_9_common;
	int common_magic;
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
