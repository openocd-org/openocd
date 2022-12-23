/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2010 by Drasko DRASKOVIC                                *
 *   drasko.draskovic@gmail.com                                            *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ARM946E_H
#define OPENOCD_TARGET_ARM946E_H

#include "arm9tdmi.h"

#define ARM946E_COMMON_MAGIC 0x20f920f9U

struct arm946e_common {
	unsigned int common_magic;

	struct arm7_9_common arm7_9_common;
	uint32_t cp15_control_reg;
	uint32_t cp15_cache_info;
};

static inline struct arm946e_common *target_to_arm946(struct target *target)
{
	return container_of(target->arch_info, struct arm946e_common,
			arm7_9_common.arm);
}

#endif /* OPENOCD_TARGET_ARM946E_H */
