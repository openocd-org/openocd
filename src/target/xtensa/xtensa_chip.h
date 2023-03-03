/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Xtensa Chip-level Target Support for OpenOCD                          *
 *   Copyright (C) 2020-2022 Cadence Design Systems, Inc.                  *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_XTENSA_CHIP_H
#define OPENOCD_TARGET_XTENSA_CHIP_H

#include <target/target.h>
#include "xtensa.h"
#include "xtensa_debug_module.h"

struct xtensa_chip_common {
	struct xtensa xtensa;
	/* Chip-specific extensions can be added here */
};

static inline struct xtensa_chip_common *target_to_xtensa_chip(struct target *target)
{
	return container_of(target->arch_info, struct xtensa_chip_common, xtensa);
}

int xtensa_chip_init_arch_info(struct target *target, void *arch_info,
	struct xtensa_debug_module_config *dm_cfg);
int xtensa_chip_target_init(struct command_context *cmd_ctx, struct target *target);
int xtensa_chip_arch_state(struct target *target);
void xtensa_chip_queue_tdi_idle(struct target *target);
void xtensa_chip_on_reset(struct target *target);
bool xtensa_chip_on_halt(struct target *target);
void xtensa_chip_on_poll(struct target *target);

#endif	/* OPENOCD_TARGET_XTENSA_CHIP_H */
