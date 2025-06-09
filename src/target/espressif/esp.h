/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Espressif chips common target API for OpenOCD                         *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ESP_H
#define OPENOCD_TARGET_ESP_H

#include <stdint.h>
#include "flash/nor/esp_flash.h"
#include <helper/bits.h>

#define IS_1XXX(v)		(((v) & 0x08) == 0x08)
#define IS_0100(v)      (((v) & 0x0f) == 0x04)

#define ESP_FLASH_BOOT_MODE	0x08

/* must be in sync with ESP-IDF version */
/** Size of the pre-compiled target buffer for stub trampoline.
 * @note Must be in sync with ESP-IDF version */
#define ESP_DBG_STUBS_CODE_BUF_SIZE         32	/* TODO: move this info to esp_dbg_stubs_ctl_data */
/** Size of the pre-compiled target buffer for stack.
 * @note Must be in sync with ESP-IDF version */
#define ESP_DBG_STUBS_STACK_MIN_SIZE        2048 /* TODO: move this info to esp_dbg_stubs_ctl_data */

/**
 * Debug stubs table entries IDs
 *
 * @note Must be in sync with ESP-IDF version in dbg_stubs.h
 */
enum esp_dbg_stub_id {
	ESP_DBG_STUB_MAGIC_NUM,
	ESP_DBG_STUB_TABLE_SIZE,
	ESP_DBG_STUB_CONTROL_DATA,
	ESP_DBG_STUB_ENTRY_FIRST, /*< Stubs descriptor entry */
	ESP_DBG_STUB_ENTRY_GCOV = ESP_DBG_STUB_ENTRY_FIRST,	/*< GCOV entry */
	ESP_DBG_STUB_ENTRY_CAPABILITIES,
	/* add new stub entries here */
	ESP_DBG_STUB_ENTRY_MAX,
};

#define ESP_DBG_STUB_MAGIC_NUM_VAL      0xFEEDBEEF
#define ESP_DBG_STUB_CAP_GCOV_THREAD    BIT(0)


/**
 * Debug stubs control data. ID: ESP_DBG_STUB_CONTROL_DATA
 *
 * @note Must be in sync with ESP-IDF version
 */
struct esp_dbg_stubs_ctl_data {
	/** Address of pre-compiled target buffer for stub trampoline.
	 * Size of the buffer is ESP_DBG_STUBS_CODE_BUF_SIZE
	 */
	uint32_t tramp_addr;
	/** Pre-compiled target buffer's addr for stack. The size of the buffer is ESP_DBG_STUBS_STACK_MIN_SIZE.
	 * Target has the buffer which is used for the stack of onboard algorithms.
	 * If stack size required by algorithm exceeds ESP_DBG_STUBS_STACK_MIN_SIZE,
	 * it should be allocated using onboard function pointed by 'data_alloc' and
	 * freed by 'data_free'. They fit to the minimal stack. See below.
	 */
	uint32_t min_stack_addr;
	/** Address of malloc-like function to allocate buffer on target. */
	uint32_t data_alloc;
	/** Address of free-like function to free buffer allocated with data_alloc. */
	uint32_t data_free;
};

/**
 * Debug stubs info.
 */
struct esp_dbg_stubs {
	/** Address. */
	uint32_t base;
	/** Table contents. */
	uint32_t entries[ESP_DBG_STUB_ENTRY_MAX];
	/** Number of table entries. */
	uint32_t entries_count;
	/** Debug stubs control data. */
	struct esp_dbg_stubs_ctl_data ctl_data;
};

struct esp_panic_reason {
	uint32_t addr;
	uint32_t len;
};

/**
 * Semihost calls handling operations.
 */
struct esp_semihost_ops {
	/** Callback called before handling semihost call */
	int (*prepare)(struct target *target);
	/** Callback called after chip reset */
	int (*post_reset)(struct target *target);
};

struct esp_semihost_data {
	uint32_t version;		/* sending with drvinfo syscall */
	bool need_resume;
	struct esp_semihost_ops *ops;
	struct list_head dir_map_list;
};

struct esp_flash_breakpoint_ops {
	int (*breakpoint_prepare)(struct target *target,
		struct breakpoint *breakpoint,
		struct esp_flash_breakpoint *bp);
	int (*breakpoint_add)(struct target *target,
		struct esp_flash_breakpoint *bp,
		size_t num_bps);
	int (*breakpoint_remove)(struct target *target,
		struct esp_flash_breakpoint *bp,
		size_t num_bps);
};

struct esp_flash_breakpoints {
	const struct esp_flash_breakpoint_ops *ops;
	struct esp_flash_breakpoint *brps;
};

struct esp_common {
	struct esp_flash_breakpoints flash_brps;
	const struct esp_algorithm_hw *algo_hw;
	struct esp_dbg_stubs dbg_stubs;
	struct esp_panic_reason panic_reason;
	bool breakpoint_lazy_process;
};

struct esp_ops {
	const struct esp_flash_breakpoint_ops *flash_brps_ops;
	const struct esp_xtensa_smp_chip_ops *chip_ops;
	const struct esp_semihost_ops *semihost_ops;
	int (*reset_reason_fetch)(struct target *target, int *rsn_id, const char **rsn_str);
};

struct esp_common *target_to_esp_common(struct target *target);
int esp_common_init(struct target *target, struct esp_common *esp,
	const struct esp_flash_breakpoint_ops *flash_brps_ops,
	const struct esp_algorithm_hw *algo_hw);
int esp_common_flash_breakpoint_add(struct target *target,
	struct esp_common *esp,
	struct breakpoint *breakpoint);
int esp_common_flash_breakpoint_remove(struct target *target,
	struct esp_common *esp,
	struct breakpoint *breakpoint);
bool esp_common_flash_breakpoint_exists(struct esp_common *esp,
	struct breakpoint *breakpoint);
int esp_common_handle_gdb_detach(struct target *target);
int esp_common_process_flash_breakpoints_command(struct command_invocation *cmd);
int esp_common_disable_lazy_breakpoints_command(struct command_invocation *cmd);
int esp_dbgstubs_table_read(struct target *target, struct esp_dbg_stubs *dbg_stubs);

void esp_common_assist_debug_monitor_disable(struct target *target, uint32_t address, uint32_t *value);
void esp_common_assist_debug_monitor_restore(struct target *target, uint32_t address, uint32_t value);
int esp_common_read_pseudo_ex_reason(struct target *target);
struct target *esp_common_get_halted_target(struct target *target, int32_t coreid);

static inline bool esp_is_flash_boot(uint32_t strap_reg)
{
	return (IS_1XXX(strap_reg) || IS_0100(strap_reg));
}

#endif	/* OPENOCD_TARGET_ESP_H */
