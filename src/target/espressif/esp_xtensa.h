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
#include "esp.h"
#include "esp_xtensa_apptrace.h"
#include "esp_xtensa_semihosting.h"

#define ESP_XTENSA_RESET_RSN_UNKNOWN    (-1)

enum esp_xtensa_exception_cause {
	ILLEGAL_INSTRUCTION = 0,
	SYSCALL = 1,
	INSTRUCTION_FETCH_ERROR = 2,
	LOAD_STORE_ERROR = 3,
	LEVEL1_INTERRUPT = 4,
	ALLOCA = 5,
	INTEGER_DIVIDE_BY_ZERO = 6,
	PRIVILEGED = 8,
	LOAD_STORE_ALIGNMENT = 9,
	INSTR_PIF_DATA_ERROR = 12,
	LOAD_STORE_PIF_DATA_ERROR = 13,
	INSTR_PIF_ADDR_ERROR = 14,
	LOAD_STORE_PIF_ADDR_ERROR = 15,
	INST_TLB_MISS = 16,
	INST_TLB_MULTIHIT = 17,
	INST_FETCH_PRIVILEGE = 18,
	INST_FETCH_PROHIBITED = 20,
	LOAD_STORE_TLB_MISS = 24,
	LOAD_STORE_TLB_MULTIHIT = 25,
	LOAD_STORE_PRIVILEGE = 26,
	LOAD_PROHIBITED = 28,
	STORE_PROHIBITED = 29,
	COPROCESSOR_N_DISABLED_0 = 32,
	COPROCESSOR_N_DISABLED_1 = 33,
	COPROCESSOR_N_DISABLED_2 = 34,
	COPROCESSOR_N_DISABLED_3 = 35,
	COPROCESSOR_N_DISABLED_4 = 36,
	COPROCESSOR_N_DISABLED_5 = 37,
	COPROCESSOR_N_DISABLED_6 = 38,
	COPROCESSOR_N_DISABLED_7 = 39,
};

struct esp_xtensa_common {
	struct xtensa xtensa;	/* must be the first element */
	struct esp_common esp;
	struct esp_semihost_data semihost;
	struct esp_xtensa_apptrace_info apptrace;
	int reset_reason;
	int (*reset_reason_fetch)(struct target *target, int *rsn_id, const char **rsn_str);
};

static inline struct esp_xtensa_common *target_to_esp_xtensa(struct target *target)
{
	return container_of(target->arch_info, struct esp_xtensa_common, xtensa);
}

int esp_xtensa_init_arch_info(struct target *target,
	struct esp_xtensa_common *esp_xtensa,
	struct xtensa_debug_module_config *dm_cfg,
	struct esp_ops *esp_ops);
int esp_xtensa_target_init(struct command_context *cmd_ctx, struct target *target);
void esp_xtensa_target_deinit(struct target *target);
int esp_xtensa_arch_state(struct target *target);
void esp_xtensa_queue_tdi_idle(struct target *target);
int esp_xtensa_breakpoint_add(struct target *target, struct breakpoint *breakpoint);
int esp_xtensa_breakpoint_remove(struct target *target, struct breakpoint *breakpoint);
int esp_xtensa_poll(struct target *target);
int esp_xtensa_reset_reason_read(struct target *target);
int esp_xtensa_profiling(struct target *target, uint32_t *samples,
	uint32_t max_num_samples, uint32_t *num_samples, uint32_t seconds);

int esp_xtensa_on_halt(struct target *target);

extern const struct command_registration esp_command_handlers[];

#endif	/* OPENOCD_TARGET_ESP_XTENSA_H */
