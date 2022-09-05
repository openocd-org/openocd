/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by David T.L. Wong                                 *
 *                                                                         *
 *   Copyright (C) 2011 by Drasko DRASKOVIC                                *
 *   drasko.draskovic@gmail.com                                            *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_MIPS_M4K_H
#define OPENOCD_TARGET_MIPS_M4K_H

struct target;

#define MIPSM4K_COMMON_MAGIC	0xB321B321U

struct mips_m4k_common {
	unsigned int common_magic;

	struct mips32_common mips32;

	bool is_pic32mx;
};

static inline struct mips_m4k_common *
target_to_m4k(struct target *target)
{
	return container_of(target->arch_info,
			struct mips_m4k_common, mips32);
}

static inline void mips_m4k_isa_filter(enum mips32_isa_imp isa_imp, target_addr_t  *addr)
{
	if (isa_imp <= 1) {	/* if only one isa implemented */
		target_addr_t address = (*addr & ~1) | isa_imp;

		if (address != *addr) {
			LOG_USER("Warning: isa bit changed due to isa not implemented");
			*addr = address;
		}
	}
}

#endif /* OPENOCD_TARGET_MIPS_M4K_H */
