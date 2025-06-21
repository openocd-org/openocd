/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP32-C6 target for OpenOCD                                           *
 *   Copyright (C) 2022 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/command.h>
#include <helper/bits.h>
#include <target/target.h>
#include <target/smp.h>
#include <target/target_type.h>
#include <target/register.h>
#include <target/semihosting_common.h>
#include <target/riscv/debug_defines.h>

#include "esp_semihosting.h"
#include "esp_riscv_apptrace.h"
#include "esp_riscv.h"

#define ESP32C6_LP_CLKRST_BASE                  0x600B0400
#define ESP32C6_LP_CLKRST_RESET_CAUSE_REG       (ESP32C6_LP_CLKRST_BASE + 0x10)
#define ESP32C6_RTCCNTL_RESET_STATE_REG         (ESP32C6_LP_CLKRST_RESET_CAUSE_REG)

#define ESP32C6_RTCCNTL_RESET_CAUSE_MASK        (BIT(5) - 1)
#define ESP32C6_RESET_CAUSE(reg_val)            ((reg_val) & ESP32C6_RTCCNTL_RESET_CAUSE_MASK)

/* max supported hw breakpoint and watchpoint count */
#define ESP32C6_BP_NUM                          4
#define ESP32C6_WP_NUM                          4

/* ASSIST_DEBUG registers */
#define ESP32C6_ASSIST_DEBUG_CPU0_MON_REG       0x600C2000

#define ESP32C6_DRAM_LOW    0x40800000
#define ESP32C6_DRAM_HIGH   0x40880000

enum esp32c6_reset_reason {
	ESP32C6_CHIP_POWER_ON_RESET   = 0x01,	/* Power on reset */
	ESP32C6_CHIP_BROWN_OUT_RESET  = 0x01,	/* VDD voltage is not stable and resets the chip */
	ESP32C6_CORE_SW_RESET         = 0x03,	/* Software resets the digital core (hp system) by LP_AON_HPSYS_SW_RESET
						 ***/
	ESP32C6_CORE_DEEP_SLEEP_RESET = 0x05,	/* Deep sleep reset the digital core (hp system) */
	ESP32C6_CORE_SDIO_RESET       = 0x06,	/* SDIO module resets the digital core (hp system) */
	ESP32C6_CORE_MWDT0_RESET      = 0x07,	/* Main watch dog 0 resets digital core (hp system) */
	ESP32C6_CORE_MWDT1_RESET      = 0x08,	/* Main watch dog 1 resets digital core (hp system) */
	ESP32C6_CORE_RTC_WDT_RESET    = 0x09,	/* RTC watch dog resets digital core (hp system) */
	ESP32C6_CPU0_MWDT0_RESET      = 0x0B,	/* Main watch dog 0 resets CPU 0 */
	ESP32C6_CPU0_SW_RESET         = 0x0C,	/* Software resets CPU 0 by LP_AON_CPU_CORE0_SW_RESET */
	ESP32C6_CPU0_RTC_WDT_RESET    = 0x0D,	/* RTC watch dog resets CPU 0 */
	ESP32C6_SYS_BROWN_OUT_RESET   = 0x0F,	/* VDD voltage is not stable and resets the digital core */
	ESP32C6_SYS_RTC_WDT_RESET     = 0x10,	/* RTC watch dog resets digital core and rtc module */
	ESP32C6_CPU0_MWDT1_RESET      = 0x11,	/* Main watch dog 1 resets CPU 0 */
	ESP32C6_SYS_SUPER_WDT_RESET   = 0x12,	/* Super watch dog resets the digital core and rtc module */
	ESP32C6_CORE_EFUSE_CRC_RESET  = 0x14,	/* eFuse CRC error resets the digital core (hp system) */
	ESP32C6_CORE_USB_UART_RESET   = 0x15,	/* USB UART resets the digital core (hp system) */
	ESP32C6_CORE_USB_JTAG_RESET   = 0x16,	/* USB JTAG resets the digital core (hp system) */
	ESP32C6_CPU0_JTAG_RESET       = 0x18,	/* JTAG resets the CPU 0 */
};

static const char *esp32c6_get_reset_reason(uint32_t reset_reason_reg_val)
{
	switch (ESP32C6_RESET_CAUSE(reset_reason_reg_val)) {
	case ESP32C6_CHIP_POWER_ON_RESET:
		/* case ESP32C6_CHIP_BROWN_OUT_RESET: */
		return "Chip reset";
	case ESP32C6_CORE_SW_RESET:
		return "Software core reset";
	case ESP32C6_CORE_DEEP_SLEEP_RESET:
		return "Deep-sleep core reset";
	case ESP32C6_CORE_SDIO_RESET:
		return "SDIO core reset";
	case ESP32C6_CORE_MWDT0_RESET:
		return "Main WDT0 core reset";
	case ESP32C6_CORE_MWDT1_RESET:
		return "Main WDT1 core reset";
	case ESP32C6_CORE_RTC_WDT_RESET:
		return "RTC WDT core reset";
	case ESP32C6_CPU0_MWDT0_RESET:
		return "Main WDT0 CPU reset";
	case ESP32C6_CPU0_SW_RESET:
		return "Software CPU reset";
	case ESP32C6_CPU0_RTC_WDT_RESET:
		return "RTC WDT CPU reset";
	case ESP32C6_SYS_BROWN_OUT_RESET:
		return "Brown-out core reset";
	case ESP32C6_SYS_RTC_WDT_RESET:
		return "RTC WDT core and rtc reset";
	case ESP32C6_CPU0_MWDT1_RESET:
		return "Main WDT1 CPU reset";
	case ESP32C6_SYS_SUPER_WDT_RESET:
		return "Super Watchdog core and rtc";
	case ESP32C6_CORE_EFUSE_CRC_RESET:
		return "eFuse CRC error core reset";
	case ESP32C6_CORE_USB_UART_RESET:
		return "USB (UART) core reset";
	case ESP32C6_CORE_USB_JTAG_RESET:
		return "USB (JTAG) core reset";
	case ESP32C6_CPU0_JTAG_RESET:
		return "JTAG CPU reset";
	}
	return "Unknown reset cause";
}

static void esp32c6_print_reset_reason(struct target *target, uint32_t reset_reason_reg_val)
{
	LOG_TARGET_INFO(target, "Reset cause (%ld) - (%s)",
		ESP32C6_RESET_CAUSE(reset_reason_reg_val),
		esp32c6_get_reset_reason(reset_reason_reg_val));
}

static bool esp32c6_is_idram_address(target_addr_t addr)
{
	return addr >= ESP32C6_DRAM_LOW && addr < ESP32C6_DRAM_HIGH;
}

static void esp32c6_trigger_match_result_fixup(struct target *target, riscv_reg_t *tdata1_ignore_mask, bool t6)
{
	if (target->coreid == 1 && !t6) {
		// For LP core DM reports that triggers support BP in user mode ("u" bit is set in mcontrol).
		// After writing 0x28001044 to mcontrol 0x2800104c is read back. But LP core MISA register
		// has no "U" extension enabled. So ignore this field, otherwise RISCV code will fail to find
		// suitable HW trigger for breakpoint.
		*tdata1_ignore_mask |= CSR_MCONTROL_U;
	}
}

static const struct esp_semihost_ops esp32c6_semihost_ops = {
	.prepare = NULL,
	.post_reset = esp_semihosting_post_reset
};

static const struct esp_flash_breakpoint_ops esp32c6_flash_brp_ops = {
	.breakpoint_prepare = esp_algo_flash_breakpoint_prepare,
	.breakpoint_add = esp_algo_flash_breakpoint_add,
	.breakpoint_remove = esp_algo_flash_breakpoint_remove,
};

static const char *esp32c6_csrs[] = {
	"mideleg", "medeleg", "mie", "mip",
	/* custom exposed CSRs will start with 'csr_' prefix*/
	"csr_ustatus", "csr_uie", "csr_utvec", "csr_uepc", "csr_ucause", "csr_utval", "csr_uip",
	"csr_pma_cfg0", "csr_pma_cfg1", "csr_pma_cfg2", "csr_pma_cfg3", "csr_pma_cfg4", "csr_pma_cfg5",
	"csr_pma_cfg6", "csr_pma_cfg7", "csr_pma_cfg8", "csr_pma_cfg9", "csr_pma_cfg10", "csr_pma_cfg11",
	"csr_pma_cfg12", "csr_pma_cfg13", "csr_pma_cfg14", "csr_pma_cfg15", "csr_pma_addr0", "csr_pma_addr1",
	"csr_pma_addr2", "csr_pma_addr3", "csr_pma_addr4", "csr_pma_addr5", "csr_pma_addr6", "csr_pma_addr7",
	"csr_pma_addr8", "csr_pma_addr9", "csr_pma_addr10", "csr_pma_addr11", "csr_pma_addr12", "csr_pma_addr13",
	"csr_pma_addr14", "csr_pma_addr15",
};

static int esp32c6_target_create(struct target *target)
{
	struct esp_riscv_common *esp_riscv = calloc(1, sizeof(*esp_riscv));
	if (!esp_riscv)
		return ERROR_FAIL;

	target->arch_info = esp_riscv;

	esp_riscv->assist_debug_cpu0_mon_reg = ESP32C6_ASSIST_DEBUG_CPU0_MON_REG;
	esp_riscv->assist_debug_cpu_offset = 0;

	esp_riscv->max_bp_num = ESP32C6_BP_NUM;
	esp_riscv->max_wp_num = ESP32C6_WP_NUM;

	esp_riscv->rtccntl_reset_state_reg = ESP32C6_RTCCNTL_RESET_STATE_REG;
	esp_riscv->print_reset_reason = &esp32c6_print_reset_reason;
	esp_riscv->existent_csrs = esp32c6_csrs;
	esp_riscv->existent_csr_size = ARRAY_SIZE(esp32c6_csrs);
	esp_riscv->existent_ro_csrs = NULL;
	esp_riscv->existent_ro_csr_size = 0;
	esp_riscv->is_dram_address = esp32c6_is_idram_address;
	esp_riscv->is_iram_address = esp32c6_is_idram_address;

	if (esp_riscv_alloc_trigger_addr(target) != ERROR_OK)
		return ERROR_FAIL;

	riscv_info_init(target, &esp_riscv->riscv);

	return ERROR_OK;
}

static int esp32c6_init_target(struct command_context *cmd_ctx,
	struct target *target)
{
	int ret = riscv_target.init_target(cmd_ctx, target);
	if (ret != ERROR_OK)
		return ret;

	target->semihosting->user_command_extension = esp_semihosting_common;

	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);

	ret = esp_riscv_init_arch_info(target,
		esp_riscv,
		&esp32c6_flash_brp_ops,
		&esp32c6_semihost_ops);
	if (ret != ERROR_OK)
		return ret;

	esp_riscv->riscv.trigger_match_result_fixup = esp32c6_trigger_match_result_fixup;

	return ERROR_OK;
}

static int esp32c6_read_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct target *hp_core_target = target;
	// Only HP core can access whole memory
	if (target->smp && target->coreid != 0) {
		struct target_list *entry;
		foreach_smp_target(entry, target->smp_targets) {
			struct target *t = entry->target;
			if (t->coreid == 0) {
				hp_core_target = t;
				break;
			}
		}
	}
	return esp_riscv_read_memory(hp_core_target, address, size, count, buffer);
}

static int esp32c6_write_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer)
{
	struct target *hp_core_target = target;
	// Only HP core can access whole memory
	if (target->smp && target->coreid != 0) {
		struct target_list *entry;
		foreach_smp_target(entry, target->smp_targets) {
			struct target *t = entry->target;
			if (t->coreid == 0) {
				hp_core_target = t;
				break;
			}
		}
	}
	return esp_riscv_write_memory(hp_core_target, address, size, count, buffer);
}

static const struct command_registration esp32c6_command_handlers[] = {
	{
		.usage = "",
		.chain = riscv_command_handlers,
	},
	{
		.name = "esp",
		.usage = "",
		.chain = esp_riscv_command_handlers,
	},
	{
		.name = "esp",
		.usage = "",
		.chain = esp32_apptrace_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct target_type esp32c6_target = {
	.name = "esp32c6",

	.target_create = esp32c6_target_create,
	.init_target = esp32c6_init_target,
	.deinit_target = esp_riscv_deinit_target,
	.examine = esp_riscv_examine,

	/* poll current target status */
	.poll = esp_riscv_poll,

	.halt = riscv_halt,
	.resume = esp_riscv_resume,
	.step = riscv_openocd_step,

	.assert_reset = esp_riscv_assert_reset,
	.deassert_reset = riscv_deassert_reset,

	.read_memory = esp32c6_read_memory,
	.write_memory = esp32c6_write_memory,

	.checksum_memory = riscv_checksum_memory,

	.get_gdb_arch = riscv_get_gdb_arch,
	.get_gdb_reg_list = riscv_get_gdb_reg_list,
	.get_gdb_reg_list_noread = esp_riscv_get_gdb_reg_list_noread,

	.add_breakpoint = esp_riscv_breakpoint_add,
	.remove_breakpoint = esp_riscv_breakpoint_remove,

	.add_watchpoint = riscv_add_watchpoint,
	.remove_watchpoint = riscv_remove_watchpoint,
	.hit_watchpoint = esp_riscv_hit_watchpoint,

	.arch_state = riscv_arch_state,

	.run_algorithm = esp_riscv_run_algorithm,
	.start_algorithm = esp_riscv_start_algorithm,
	.wait_algorithm = esp_riscv_wait_algorithm,

	.commands = esp32c6_command_handlers,

	.address_bits = riscv_xlen_nonconst,
};
