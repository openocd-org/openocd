/***************************************************************************
 *   Generic ESP xtensa target implementation for OpenOCD                  *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
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
