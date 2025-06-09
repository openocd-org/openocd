/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Generic flash driver for Espressif chips                              *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifndef OPENOCD_FLASH_NOR_ESP_FLASH_H
#define OPENOCD_FLASH_NOR_ESP_FLASH_H

#include <target/target.h>
#include <target/espressif/esp_algorithm.h>
#include <target/breakpoints.h>
#include <flash/nor/core.h>

struct esp_flash_apptrace_hw {
	int (*info_init)(struct target *target,
		target_addr_t ctrl_addr,
		target_addr_t *old_ctrl_addr);
	int (*data_len_read)(struct target *target,
		uint32_t *block_id,
		uint32_t *len);
	int (*data_read)(struct target *target,
		uint32_t size,
		uint8_t *buffer,
		uint32_t block_id,
		bool ack);
	int (*ctrl_reg_read)(struct target *target,
		uint32_t *block_id,
		uint32_t *len,
		bool *conn);
	int (*ctrl_reg_write)(struct target *target,
		uint32_t block_id,
		uint32_t len,
		bool conn,
		bool data);
	int (*usr_block_write)(struct target *target,
		uint32_t block_id,
		const uint8_t *data,
		uint32_t size);
	uint8_t *(*usr_block_get)(struct target *target, uint8_t *buffer, uint32_t *size);
	uint32_t (*block_max_size_get)(struct target *target);
	uint32_t (*usr_block_max_size_get)(struct target *target);
};

struct esp_flasher_stub_config {
	const uint8_t *code;
	uint32_t code_sz;
	const uint8_t *data;
	uint32_t data_sz;
	target_addr_t entry_addr;
	uint32_t bss_sz;
	uint32_t first_user_reg_param;
	target_addr_t apptrace_ctrl_addr;
	uint32_t stack_default_sz; /* chip based default stack usage amount */
	uint32_t stack_data_pool_sz;
	target_addr_t log_buff_addr;
	uint32_t log_buff_size;	/* current_log_len + len(buff) */
	target_addr_t iram_org;
	uint32_t iram_len;
	target_addr_t dram_org;
	uint32_t dram_len;
	/* ibus address range can be in reverse order compared to dbus */
	bool reverse;
};

/* ESP flash data.
   It should be the first member of flash data structs for concrete chips.
   For example see ESP32 flash driver implementation. */
struct esp_flash_bank {
	int probed;
	/* Sector size */
	uint32_t sec_sz;
	/* Base address of the bank in the flash, 0 - for HW flash bank, non-zero for special
	 * IROM/DROM fake banks */
	/* Those fake banks are necessary for generating proper memory map for GDB and using flash
	 * breakpoints */
	uint32_t hw_flash_base;
	/* Offset of the application image in the HW flash bank */
	uint32_t appimage_flash_base;
	const struct esp_flasher_stub_config *(*get_stub)(struct flash_bank *bank, int cmd);
	/* function to run algorithm on Espressif target */
	int (*run_func_image)(struct target *target, struct esp_algorithm_run_data *run, uint32_t num_args, ...);
	bool (*is_irom_address)(target_addr_t addr);
	bool (*is_drom_address)(target_addr_t addr);
	const struct esp_flash_apptrace_hw *apptrace_hw;
	const struct esp_algorithm_hw *stub_hw;
	/* Upload compressed or uncompressed image */
	bool compression;
	/* Stub cpu frequency before boost */
	int old_cpu_freq;
	/* Inform stub flasher if encryption requires before writing to flash.  */
	int encryption_needed_on_chip;
	/* Enable/disable stub log*/
	bool stub_log_enabled;
	/* If exist at the target memory, allow to run preloaded stub code without loading again */
	bool check_preloaded_binary;
};

enum esp_flash_bp_action {
	ESP_BP_ACT_NAN,
	ESP_BP_ACT_REM = ESP_BP_ACT_NAN,
	ESP_BP_ACT_ADD
};

enum esp_flash_bp_status {
	ESP_BP_STAT_DONE,
	ESP_BP_STAT_PEND,
};

/*
Breakpoint States:
1 - Init state equal to Remove-Done: Initial state for the empty slot
2 - Add-Pending: Z1 package received but break inst is not written to flash yet
3 - Add-Done: Break inst is written to flash
3 - Remove-Pending: z1 package received but original inst is not written back to flash yet
3 - Remove-Done equal to Init: Original inst written back to flash and slot is empty

Transitions will normally follow the same order as described above
[ INIT (BP_REM-DONE) ] --> [ BP-ADD-PENDING ] --> [ BP-ADD-DONE ] --> [ BP-REM-PENDING ] --> [ BP_REM-DONE ]

In one case, we performed a special state change. When the breakpoint is at the rem-pending state,
it means the breakpoint instruction is written to the flash but not removed yet.
However, GDB thinks it is removed since the Z1 package has already been sent.
So, it might send an add package for the same address.
In this case, we don't need to remove the breakpoint and add it again.
It is fine to change the state to add-done (fake-add).
*/

struct esp_flash_breakpoint {
	struct breakpoint *oocd_bp;
	/* original insn or part of it */
	uint8_t insn[4];
	/* original insn size. Actually this is size of break instruction. */
	uint8_t insn_sz;
	struct flash_bank *bank;
	int sector_num;
	target_addr_t bp_address; /* virtual */
	uint32_t bp_flash_addr; /* physical */
	enum esp_flash_bp_action action;
	enum esp_flash_bp_status status;
};

int esp_algo_flash_init(struct esp_flash_bank *esp_info, uint32_t sec_sz,
	int (*run_func_image)(struct target *target, struct esp_algorithm_run_data *run,
		uint32_t num_args, ...),
	bool (*is_irom_address)(target_addr_t addr),
	bool (*is_drom_address)(target_addr_t addr),
	const struct esp_flasher_stub_config *(*get_stub)(struct flash_bank *bank, int cmd),
	const struct esp_flash_apptrace_hw *apptrace_hw,
	const struct esp_algorithm_hw *stub_hw,
	bool check_preloaded_binary);
int esp_algo_flash_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last);
int esp_algo_flash_protect_check(struct flash_bank *bank);
int esp_algo_flash_blank_check(struct flash_bank *bank);
int esp_algo_flash_erase(struct flash_bank *bank, unsigned int first, unsigned int last);
int esp_algo_flash_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count);
int esp_algo_flash_read(struct flash_bank *bank, uint8_t *buffer,
	uint32_t offset, uint32_t count);
int esp_algo_flash_probe(struct flash_bank *bank);
int esp_algo_flash_auto_probe(struct flash_bank *bank);
int esp_algo_flash_breakpoint_prepare(struct target *target,
	struct breakpoint *breakpoint,
	struct esp_flash_breakpoint *sw_bp);
int esp_algo_flash_breakpoint_add(struct target *target,
	struct esp_flash_breakpoint *sw_bp,
	size_t num_bps);
int esp_algo_flash_breakpoint_remove(struct target *target,
	struct esp_flash_breakpoint *sw_bp,
	size_t num_bps);

extern const struct command_registration esp_flash_exec_flash_command_handlers[];

#endif	/* OPENOCD_FLASH_NOR_ESP_FLASH_H */
