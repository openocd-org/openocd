/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Application level tracing API for Espressif RISCV chips               *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ESP_RISCV_APPTRACE_H
#define OPENOCD_TARGET_ESP_RISCV_APPTRACE_H

#include "esp32_apptrace.h"

/* should be same with `CONFIG_APPTRACE_BUF_SIZE` value in the stub sdkconfig.h */
#define ESP_RISCV_APPTRACE_BUF_SIZE     16384
#define ESP_RISCV_STACK_DATA_POOL_SIZE  (ESP_RISCV_APPTRACE_BUF_SIZE * 2)

struct esp_riscv_apptrace_mem_block {
	uint32_t start;	/* start address */
	uint32_t sz;	/* size */
};

struct esp_riscv_apptrace_info {
	const struct esp32_apptrace_hw *hw;
	target_addr_t ctrl_addr;
	struct esp_riscv_apptrace_mem_block mem_blocks[2];
};

extern struct esp32_apptrace_hw esp_riscv_apptrace_hw;

int esp_riscv_apptrace_info_init(struct target *target, target_addr_t ctrl_addr, target_addr_t *old_ctrl_addr);
int esp_riscv_apptrace_data_len_read(struct target *target, uint32_t *block_id, uint32_t *len);
int esp_riscv_apptrace_data_read(struct target *target, uint32_t size, uint8_t *buffer, uint32_t block_id, bool ack);
int esp_riscv_apptrace_ctrl_reg_read(struct target *target, uint32_t *block_id, uint32_t *len, bool *conn);
int esp_riscv_apptrace_ctrl_reg_write(struct target *target, uint32_t block_id, uint32_t len, bool conn, bool data);
uint32_t esp_riscv_apptrace_block_max_size_get(struct target *target);
uint32_t esp_riscv_apptrace_usr_block_max_size_get(struct target *target);
int esp_riscv_apptrace_usr_block_write(struct target *target, uint32_t block_id, const uint8_t *data, uint32_t size);

#endif	/* OPENOCD_TARGET_ESP_RISCV_APPTRACE_H */
