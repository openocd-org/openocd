/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Xtensa application tracing module for OpenOCD                         *
 *   Copyright (C) 2017 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ESP_XTENSA_APPTRACE_H
#define OPENOCD_TARGET_ESP_XTENSA_APPTRACE_H

#include "esp32_apptrace.h"

struct esp_xtensa_apptrace_info {
	const struct esp32_apptrace_hw *hw;
};

extern struct esp32_apptrace_hw esp_xtensa_apptrace_hw;

int esp_xtensa_apptrace_data_len_read(struct target *target, uint32_t *block_id, uint32_t *len);
int esp_xtensa_apptrace_data_read(struct target *target,
	uint32_t size,
	uint8_t *buffer,
	uint32_t block_id,
	bool ack);
int esp_xtensa_apptrace_ctrl_reg_read(struct target *target, uint32_t *block_id, uint32_t *len, bool *conn);
int esp_xtensa_apptrace_ctrl_reg_write(struct target *target,
	uint32_t block_id,
	uint32_t len,
	bool conn,
	bool data);
int esp_xtensa_apptrace_status_reg_write(struct target *target, uint32_t stat);
int esp_xtensa_apptrace_status_reg_read(struct target *target, uint32_t *stat);
uint32_t esp_xtensa_apptrace_block_max_size_get(struct target *target);
uint32_t esp_xtensa_apptrace_usr_block_max_size_get(struct target *target);
int esp_xtensa_apptrace_usr_block_write(struct target *target, uint32_t block_id, const uint8_t *data, uint32_t size);

#endif	/* OPENOCD_TARGET_ESP_XTENSA_APPTRACE_H */
