/***************************************************************************
 *   ESP Xtensa SMP target for OpenOCD                                     *
 *   Copyright (C) 2020 Espressif Systems Ltd. Co                          *
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

#ifndef OPENOCD_TARGET_XTENSA_ESP_SMP_H
#define OPENOCD_TARGET_XTENSA_ESP_SMP_H

#include "esp_xtensa.h"

struct esp_xtensa_smp_chip_ops {
	int (*poll)(struct target *target);
	int (*reset)(struct target *target);
	int (*on_halt)(struct target *target);
};

struct esp_xtensa_smp_common {
	struct esp_xtensa_common esp_xtensa;
	const struct esp_xtensa_smp_chip_ops *chip_ops;
	bool other_core_does_resume;
	/* number of attempts to examine other SMP cores, attempts are made after reset on target poll */
	int examine_other_cores;
};

int esp_xtensa_smp_poll(struct target *target);
int esp_xtensa_smp_resume(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints,
	int debug_execution);
int esp_xtensa_smp_step(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints);
int esp_xtensa_smp_assert_reset(struct target *target);
int esp_xtensa_smp_deassert_reset(struct target *target);
int esp_xtensa_smp_soft_reset_halt(struct target *target);
int esp_xtensa_smp_watchpoint_add(struct target *target, struct watchpoint *watchpoint);
int esp_xtensa_smp_watchpoint_remove(struct target *target, struct watchpoint *watchpoint);
int esp_xtensa_smp_handle_target_event(struct target *target, enum target_event event, void *priv);
int esp_xtensa_smp_target_init(struct command_context *cmd_ctx, struct target *target);
int esp_xtensa_smp_init_arch_info(struct target *target,
	struct esp_xtensa_smp_common *esp_xtensa_smp,
	const struct xtensa_config *xtensa_cfg,
	struct xtensa_debug_module_config *dm_cfg,
	const struct esp_xtensa_smp_chip_ops *chip_ops);

extern const struct command_registration esp_xtensa_smp_command_handlers[];
extern const struct command_registration esp_xtensa_smp_xtensa_command_handlers[];
extern const struct command_registration esp_xtensa_smp_esp_command_handlers[];

#endif	/* OPENOCD_TARGET_XTENSA_ESP_SMP_H */
