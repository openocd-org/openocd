/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Generic ESP xtensa target implementation for OpenOCD                  *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ESP_XTENSA_H
#define OPENOCD_TARGET_ESP_XTENSA_H

#include <helper/command.h>
#include <target/target.h>
#include <target/xtensa/xtensa.h>

struct esp_xtensa_common {
	struct xtensa xtensa;	/* must be the first element */
};

static inline struct esp_xtensa_common *target_to_esp_xtensa(struct target *target)
{
	return container_of(target->arch_info, struct esp_xtensa_common, xtensa);
}

int esp_xtensa_init_arch_info(struct target *target,
	struct esp_xtensa_common *esp_xtensa,
	const struct xtensa_config *xtensa_cfg,
	struct xtensa_debug_module_config *dm_cfg);
int esp_xtensa_target_init(struct command_context *cmd_ctx, struct target *target);
void esp_xtensa_target_deinit(struct target *target);
int esp_xtensa_arch_state(struct target *target);
void esp_xtensa_queue_tdi_idle(struct target *target);
int esp_xtensa_breakpoint_add(struct target *target, struct breakpoint *breakpoint);
int esp_xtensa_breakpoint_remove(struct target *target, struct breakpoint *breakpoint);
int esp_xtensa_poll(struct target *target);

#endif	/* OPENOCD_TARGET_ESP_XTENSA_H */
