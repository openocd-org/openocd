/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2013 Andes Technology                                   *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_NDS32_V3_COMMON_H
#define OPENOCD_TARGET_NDS32_V3_COMMON_H

#include "target.h"

struct nds32_v3_common_callback {
	int (*check_interrupt_stack)(struct nds32 *nds32);
	int (*restore_interrupt_stack)(struct nds32 *nds32);
	int (*activate_hardware_breakpoint)(struct target *target);
	int (*activate_hardware_watchpoint)(struct target *target);
	int (*deactivate_hardware_breakpoint)(struct target *target);
	int (*deactivate_hardware_watchpoint)(struct target *target);
};

void nds32_v3_common_register_callback(struct nds32_v3_common_callback *callback);
int nds32_v3_target_request_data(struct target *target,
		uint32_t size, uint8_t *buffer);
int nds32_v3_checksum_memory(struct target *target,
		target_addr_t address, uint32_t count, uint32_t *checksum);
int nds32_v3_hit_watchpoint(struct target *target,
		struct watchpoint **hit_watchpoint);
int nds32_v3_target_create_common(struct target *target, struct nds32 *nds32);
int nds32_v3_run_algorithm(struct target *target,
		int num_mem_params,
		struct mem_param *mem_params,
		int num_reg_params,
		struct reg_param *reg_params,
		target_addr_t entry_point,
		target_addr_t exit_point,
		int timeout_ms,
		void *arch_info);
int nds32_v3_read_buffer(struct target *target, target_addr_t address,
		uint32_t size, uint8_t *buffer);
int nds32_v3_write_buffer(struct target *target, target_addr_t address,
		uint32_t size, const uint8_t *buffer);
int nds32_v3_read_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, uint8_t *buffer);
int nds32_v3_write_memory(struct target *target, target_addr_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer);
int nds32_v3_init_target(struct command_context *cmd_ctx,
		struct target *target);

#endif /* OPENOCD_TARGET_NDS32_V3_COMMON_H */
