/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ARM926EJS_H
#define OPENOCD_TARGET_ARM926EJS_H

#include "arm9tdmi.h"
#include "armv4_5_mmu.h"

#define	ARM926EJS_COMMON_MAGIC 0xa926a926U

struct arm926ejs_common {
	unsigned int common_magic;

	struct arm7_9_common arm7_9_common;
	struct armv4_5_mmu_common armv4_5_mmu;
	int (*read_cp15)(struct target *target, uint32_t op1, uint32_t op2,
			uint32_t crn, uint32_t crm, uint32_t *value);
	int (*write_cp15)(struct target *target, uint32_t op1, uint32_t op2,
			uint32_t crn, uint32_t crm, uint32_t value);
	uint32_t cp15_control_reg;
	uint32_t d_fsr;
	uint32_t i_fsr;
	uint32_t d_far;
};

static inline struct arm926ejs_common *target_to_arm926(struct target *target)
{
	return container_of(target->arch_info, struct arm926ejs_common, arm7_9_common.arm);
}

int arm926ejs_init_arch_info(struct target *target,
		struct arm926ejs_common *arm926ejs, struct jtag_tap *tap);
int arm926ejs_arch_state(struct target *target);
int arm926ejs_write_memory(struct target *target,
		target_addr_t address, uint32_t size, uint32_t count, const uint8_t *buffer);
int arm926ejs_soft_reset_halt(struct target *target);

extern const struct command_registration arm926ejs_command_handlers[];

#endif /* OPENOCD_TARGET_ARM926EJS_H */
