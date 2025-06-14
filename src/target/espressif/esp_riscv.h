/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP RISCV common definitions for OpenOCD                              *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ESP_RISCV_H
#define OPENOCD_TARGET_ESP_RISCV_H

#include <target/target.h>
#include <target/riscv/riscv.h>
#include <target/riscv/debug_defines.h>
#include "esp_riscv_apptrace.h"
#include "esp_riscv_algorithm.h"
#include "esp.h"

#define get_field(reg, mask) (((reg) & (mask)) / ((mask) & ~((mask) << 1)))
#define set_field(reg, mask, val) (((reg) & ~(mask)) | (((val) * ((mask) & ~((mask) << 1))) & (mask)))

struct esp_riscv_common {
	/* should be first, will be accessed by riscv generic code */
	struct riscv_info riscv;
	struct esp_common esp;
	struct esp_riscv_apptrace_info apptrace;
	struct esp_semihost_data semihost;
	struct esp_semihost_ops *semi_ops;
	target_addr_t *target_bp_addr;
	target_addr_t *target_wp_addr;
	uint8_t max_bp_num;
	uint8_t max_wp_num;
	uint32_t assist_debug_cpu0_mon_reg; /* cpu 0 monitor register address */
	uint32_t assist_debug_cpu_offset;   /* address offset to register of next cpu id */
	target_addr_t rtccntl_reset_state_reg;	/* to read reset cause */
	void (*print_reset_reason)(struct target *target, uint32_t reset_reason_reg_val);
	bool was_reset;
	const char **existent_csrs;
	size_t existent_csr_size;
	const char **existent_ro_csrs;
	size_t existent_ro_csr_size;
	bool (*is_iram_address)(target_addr_t addr);
	bool (*is_dram_address)(target_addr_t addr);
};

static inline struct esp_riscv_common *target_to_esp_riscv(const struct target *target)
{
	return target->arch_info;
}

static inline int esp_riscv_on_reset(struct target *target)
{
	LOG_TARGET_DEBUG(target, "on reset!");
	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);
	esp_riscv->was_reset = true;
	/* clear previous apptrace ctrl_addr to avoid invalid tracing control block usage during/after reset */
	esp_riscv->apptrace.ctrl_addr = 0;
	return ERROR_OK;
}

static inline int esp_riscv_init_arch_info(struct target *target,
	struct esp_riscv_common *esp_riscv,
	const struct esp_flash_breakpoint_ops *flash_brps_ops,
	const struct esp_semihost_ops *semi_ops)
{
	esp_riscv->riscv.on_reset = esp_riscv_on_reset;

	INIT_LIST_HEAD(&esp_riscv->semihost.dir_map_list);

	int ret = esp_common_init(target, &esp_riscv->esp, flash_brps_ops, &riscv_algo_hw);
	if (ret != ERROR_OK)
		return ret;

	esp_riscv->apptrace.hw = &esp_riscv_apptrace_hw;
	esp_riscv->semi_ops = (struct esp_semihost_ops *)semi_ops;

	return ERROR_OK;
}

int esp_riscv_examine(struct target *target);
int esp_riscv_poll(struct target *target);
int esp_riscv_alloc_trigger_addr(struct target *target);
int esp_riscv_semihosting(struct target *target);
int esp_riscv_breakpoint_add(struct target *target, struct breakpoint *breakpoint);
int esp_riscv_breakpoint_remove(struct target *target, struct breakpoint *breakpoint);
int esp_riscv_smp_watchpoint_add(struct target *target, struct watchpoint *watchpoint);
int esp_riscv_smp_watchpoint_remove(struct target *target, struct watchpoint *watchpoint);
int esp_riscv_hit_watchpoint(struct target *target, struct watchpoint **hit_watchpoint);
int esp_riscv_resume(struct target *target, bool current, target_addr_t address,
		bool handle_breakpoints, bool debug_execution);
int esp_riscv_start_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t entry_point, target_addr_t exit_point,
	void *arch_info);
int esp_riscv_wait_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t exit_point, unsigned int timeout_ms,
	void *arch_info);
int esp_riscv_run_algorithm(struct target *target, int num_mem_params,
	struct mem_param *mem_params, int num_reg_params,
	struct reg_param *reg_params, target_addr_t entry_point,
	target_addr_t exit_point, unsigned int timeout_ms, void *arch_info);
int esp_riscv_smp_run_func_image(struct target *target, struct esp_algorithm_run_data *run, uint32_t num_args, ...);
int esp_riscv_read_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer);
int esp_riscv_write_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer);

int esp_riscv_core_ebreaks_enable(struct target *target);
void esp_riscv_deinit_target(struct target *target);
int esp_riscv_assert_reset(struct target *target);
int esp_riscv_get_gdb_reg_list_noread(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class);

extern const struct command_registration esp_riscv_command_handlers[];

#endif	/* OPENOCD_TARGET_ESP_RISCV_H */
