/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP RISCV flash driver for OpenOCD                                    *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "esp_riscv.h"
#include <target/espressif/esp_riscv_apptrace.h>
#include <target/espressif/esp_riscv_algorithm.h>

static const struct esp_flash_apptrace_hw s_esp_riscv_flash_apptrace_hw = {
	.info_init = esp_riscv_apptrace_info_init,
	.data_len_read = esp_riscv_apptrace_data_len_read,
	.data_read = esp_riscv_apptrace_data_read,
	.ctrl_reg_read = esp_riscv_apptrace_ctrl_reg_read,
	.ctrl_reg_write = esp_riscv_apptrace_ctrl_reg_write,
	.usr_block_write = esp_riscv_apptrace_usr_block_write,
	.usr_block_get = esp_apptrace_usr_block_get,
	.block_max_size_get = esp_riscv_apptrace_block_max_size_get,
	.usr_block_max_size_get = esp_riscv_apptrace_usr_block_max_size_get,
};

int esp_riscv_flash_init(struct esp_riscv_flash_bank *esp_info, uint32_t sec_sz,
	int (*run_func_image)(struct target *target, struct esp_algorithm_run_data *run,
		uint32_t num_args, ...),
	bool (*is_irom_address)(target_addr_t addr),
	bool (*is_drom_address)(target_addr_t addr),
	const struct esp_flasher_stub_config *(*get_stub)(struct flash_bank *bank, int cmd),
	bool check_preloaded_binary)
{
	memset(esp_info, 0, sizeof(*esp_info));
	return esp_algo_flash_init(&esp_info->esp, sec_sz, run_func_image, is_irom_address,
		is_drom_address, get_stub, &s_esp_riscv_flash_apptrace_hw, &riscv_algo_hw, check_preloaded_binary);
}
