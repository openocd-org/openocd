// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   ESP32-H4 target for OpenOCD                                           *
 *   Copyright (C) 2025 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/command.h>
#include <helper/bits.h>
#include <helper/align.h>
#include <target/target.h>
#include <target/target_type.h>
#include <target/register.h>
#include <target/semihosting_common.h>
#include <target/riscv/debug_defines.h>

#include "esp_semihosting.h"
#include "esp_riscv_apptrace.h"
#include "esp_riscv.h"

/* reset cause */
#define ESP32H4_LP_CLKRST_BASE					(0x600B3000)
#define ESP32H4_LP_CLKRST_RESET_CORE0_CAUSE_REG	(ESP32H4_LP_CLKRST_BASE + 0x10)
#define ESP32H4_LP_CLKRST_RESET_CORE1_CAUSE_REG	(ESP32H4_LP_CLKRST_BASE + 0x14)
#define ESP32H4_RESET_CAUSE_MASK				(BIT(5) - 1) /* 0x1F */
#define ESP32H4_HP_CORE0_RESET_CAUSE_SHIFT		0
#define ESP32H4_HP_CORE1_RESET_CAUSE_SHIFT		0
#define ESP32H4_RESET_CAUSE(reg_val, shift)		(((reg_val) >> (shift)) & ESP32H4_RESET_CAUSE_MASK)

// TODO: when DCache/ICache1 is disabled, extra space will be available for SRAM (OCD-1195)
#define ESP32H4_DRAM_LOW                        0x40810000
#define ESP32H4_DRAM_HIGH                       0x40860000
#define ESP32H4_IROM_MASK_LOW                   0x40000000
#define ESP32H4_IROM_MASK_HIGH                  0x40050000

#define ESP32H4_ADDR_IS_DRAM(addr)	((addr) >= ESP32H4_DRAM_LOW && (addr) <  ESP32H4_DRAM_HIGH)

/* max supported hw breakpoint and watchpoint count */
#define ESP32H4_BP_NUM                          3
#define ESP32H4_WP_NUM                          3

#define ESP32H4_ASSIST_DEBUG_CPU0_MON_REG       0x60002000
#define ESP32H4_ASSIST_DEBUG_CPU_OFFSET         0x88

/* components/soc/esp32H4/include/soc/reset_reasons.h */
enum esp32h4_reset_reason {
	RESET_REASON_CHIP_POWER_ON   = 0x01,// Power on reset
	RESET_REASON_CHIP_BROWN_OUT  = 0x01,// VDD voltage is not stable and resets the chip
	RESET_REASON_CORE_SW         = 0x03,// Software resets the digital core (hp system) by LP_AON_HPSYS_SW_RESET
	RESET_REASON_CORE_DEEP_SLEEP = 0x05,// Deep sleep reset the digital core (hp system)
	RESET_REASON_CORE_MWDT0      = 0x07,// Main watch dog 0 resets digital core (hp system)
	RESET_REASON_CORE_MWDT1      = 0x08,// Main watch dog 1 resets digital core (hp system)
	RESET_REASON_CORE_RTC_WDT    = 0x09,// RTC watch dog resets digital core (hp system)
	RESET_REASON_CPU0_MWDT0      = 0x0B,// Main watch dog 0 resets CPU 0
	RESET_REASON_CPU0_SW         = 0x0C,// Software resets CPU 0 by LP_AON_CPU_CORE0_SW_RESET
	RESET_REASON_CPU0_RTC_WDT    = 0x0D,// RTC watch dog resets CPU 0
	RESET_REASON_SYS_BROWN_OUT   = 0x0F,// VDD voltage is not stable and resets the digital core
	RESET_REASON_SYS_RTC_WDT     = 0x10,// RTC watch dog resets digital core and rtc module
	RESET_REASON_CPU0_MWDT1      = 0x11,// Main watch dog 1 resets CPU 0
	RESET_REASON_SYS_SUPER_WDT   = 0x12,// Super watch dog resets the digital core and rtc module
	RESET_REASON_CORE_EFUSE_CRC  = 0x14,// eFuse CRC error resets the digital core (hp system)
	RESET_REASON_CORE_USB_UART   = 0x15,// USB UART resets the digital core (hp system)
	RESET_REASON_CORE_USB_JTAG   = 0x16,// USB JTAG resets the digital core (hp system)
	RESET_REASON_CPU0_JTAG       = 0x18,// JTAG resets the CPU 0
};

static const char *esp32h4_get_reset_reason(uint32_t reset_reason_reg_val, int shift_val)
{
	switch (ESP32H4_RESET_CAUSE(reset_reason_reg_val, shift_val)) {
	case RESET_REASON_CHIP_POWER_ON:
		return "Power on chip reset";
	case RESET_REASON_CORE_SW:
		return "Software core reset";
	case RESET_REASON_CORE_DEEP_SLEEP:
		return "Deep sleep core reset";
	case RESET_REASON_CORE_MWDT0:
		return "Main watch dog 0 core reset";
	case RESET_REASON_CORE_MWDT1:
		return "Main watch dog 1 core reset";
	case RESET_REASON_CORE_RTC_WDT:
		return "RTC watch dog core reset";
	case RESET_REASON_CPU0_MWDT0:
		return "Main watch dog cpu reset";
	case RESET_REASON_CPU0_SW:
		return "Software cpu reset";
	case RESET_REASON_CPU0_RTC_WDT:
		return "RTC watchdog cpu reset";
	case RESET_REASON_SYS_BROWN_OUT:
		return "Brown out system reset";
	case RESET_REASON_SYS_RTC_WDT:
		return "RTC watch dog system reset";
	case RESET_REASON_CPU0_MWDT1:
		return "Main watch dog cpu reset";
	case RESET_REASON_SYS_SUPER_WDT:
		return "Super watch dog system reset";
	case RESET_REASON_CORE_EFUSE_CRC:
		return "Efuse crc error core reset";
	case RESET_REASON_CORE_USB_UART:
		return "USB UART core reset";
	case RESET_REASON_CORE_USB_JTAG:
		return "USB JTAG core reset";
	case RESET_REASON_CPU0_JTAG:
		return "JTAG cpu reset";
	}
	return "Unknown reset cause";
}

// TODO: Test and close OCD-1140
static void esp32h4_print_reset_reason(struct target *target, uint32_t reset_reason_reg_val)
{
	if (target->coreid == 0) {
		LOG_TARGET_INFO(target, "Reset cause (%ld) - (%s)",
			ESP32H4_RESET_CAUSE(reset_reason_reg_val, ESP32H4_HP_CORE0_RESET_CAUSE_SHIFT),
			esp32h4_get_reset_reason(reset_reason_reg_val, ESP32H4_HP_CORE0_RESET_CAUSE_SHIFT));
	} else {
		uint32_t reset_reason_core1 = 0;
		int res = target_read_u32(target, ESP32H4_LP_CLKRST_RESET_CORE1_CAUSE_REG, &reset_reason_core1);
		if (res != ERROR_OK)
			LOG_TARGET_WARNING(target, "Failed to read reset cause register (%d)!", res);
		else
			LOG_TARGET_INFO(target, "Reset cause (%ld) - (%s)",
				ESP32H4_RESET_CAUSE(reset_reason_core1, ESP32H4_HP_CORE1_RESET_CAUSE_SHIFT),
				esp32h4_get_reset_reason(reset_reason_core1, ESP32H4_HP_CORE1_RESET_CAUSE_SHIFT));
	}
}

static bool esp32h4_is_idram_address(target_addr_t addr)
{
	return ESP32H4_ADDR_IS_DRAM(addr);
}

static const struct esp_semihost_ops esp32h4_semihost_ops = {
	.prepare = NULL,
	.post_reset = esp_semihosting_post_reset
};

static const struct esp_flash_breakpoint_ops esp32h4_flash_brp_ops = {
	.breakpoint_prepare = esp_algo_flash_breakpoint_prepare,
	.breakpoint_add = esp_algo_flash_breakpoint_add,
	.breakpoint_remove = esp_algo_flash_breakpoint_remove,
};

// TODO: check if the CSRs are correct OCD-1143
static const char *esp32h4_csrs[] = {
	"mie", "mcause", "mip", "mtvt", "mnxti",
	"mscratchcsw", "mscratchcswl",
	"mcycle", "minstret", "mcounteren", "mcountinhibit",
	"mhpmcounter8", "mhpmcounter9", "mhpmcounter13", "mhpmevent8", "mhpmevent9", "mhpmevent13",
	"mcycleh", "minstreth", "mhpmcounter8h", "mhpmcounter9h", "mhpmcounter13h",
	"tdata3", "tinfo", "mcontext", "mintstatus",
	/* custom exposed CSRs will start with 'csr_' prefix*/
	"csr_mclicbase", "csr_mxstatus", "csr_mhcr", "csr_mhint", "csr_mraddr", "csr_mexstatus",
	"csr_mnmicause", "csr_mnmipc", "csr_mcpuid", "csr_cpu_testbus_ctrl", "csr_pm_user",
	"csr_gpio_oen_user", "csr_gpio_in_user", "csr_gpio_out_user",
	"csr_pma_cfg0", "csr_pma_cfg1", "csr_pma_cfg2", "csr_pma_cfg3", "csr_pma_cfg4", "csr_pma_cfg5",
	"csr_pma_cfg6", "csr_pma_cfg7", "csr_pma_cfg8", "csr_pma_cfg9", "csr_pma_cfg10", "csr_pma_cfg11",
	"csr_pma_cfg12", "csr_pma_cfg13", "csr_pma_cfg14", "csr_pma_cfg15", "csr_pma_addr0", "csr_pma_addr1",
	"csr_pma_addr2", "csr_pma_addr3", "csr_pma_addr4", "csr_pma_addr5", "csr_pma_addr6", "csr_pma_addr7",
	"csr_pma_addr8", "csr_pma_addr9", "csr_pma_addr10", "csr_pma_addr11", "csr_pma_addr12", "csr_pma_addr13",
	"csr_pma_addr14", "csr_pma_addr15",
};

static int esp32h4_target_create(struct target *target)
{
	struct esp_riscv_common *esp_riscv = calloc(1, sizeof(*esp_riscv));
	if (!esp_riscv)
		return ERROR_FAIL;

	target->arch_info = esp_riscv;

	esp_riscv->assist_debug_cpu0_mon_reg = ESP32H4_ASSIST_DEBUG_CPU0_MON_REG;
	esp_riscv->assist_debug_cpu_offset = ESP32H4_ASSIST_DEBUG_CPU_OFFSET;

	esp_riscv->max_bp_num = ESP32H4_BP_NUM;
	esp_riscv->max_wp_num = ESP32H4_WP_NUM;

	// ESP32H4 records reset reasons in 2 different registers. We will handle core1 in esp32h4_print_reset_reason
	esp_riscv->rtccntl_reset_state_reg = ESP32H4_LP_CLKRST_RESET_CORE0_CAUSE_REG;
	esp_riscv->print_reset_reason = &esp32h4_print_reset_reason;
	esp_riscv->existent_csrs = esp32h4_csrs;
	esp_riscv->existent_csr_size = ARRAY_SIZE(esp32h4_csrs);
	esp_riscv->existent_ro_csrs = NULL;
	esp_riscv->existent_ro_csr_size = 0;
	esp_riscv->is_dram_address = esp32h4_is_idram_address;
	esp_riscv->is_iram_address = esp32h4_is_idram_address;

	if (esp_riscv_alloc_trigger_addr(target) != ERROR_OK)
		return ERROR_FAIL;

	riscv_info_init(target, &esp_riscv->riscv);

	return ERROR_OK;
}

static int esp32h4_init_target(struct command_context *cmd_ctx,
	struct target *target)
{
	int ret = riscv_target.init_target(cmd_ctx, target);
	if (ret != ERROR_OK)
		return ret;

	target->semihosting->user_command_extension = esp_semihosting_common;

	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);

	ret = esp_riscv_init_arch_info(target,
		esp_riscv,
		&esp32h4_flash_brp_ops,
		&esp32h4_semihost_ops);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

static const struct command_registration esp32h4_command_handlers[] = {
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

struct target_type esp32h4_target = {
	.name = "esp32h4",

	.target_create = esp32h4_target_create,
	.init_target = esp32h4_init_target,
	.deinit_target = esp_riscv_deinit_target,
	.examine = esp_riscv_examine,

	/* poll current target status */
	.poll = esp_riscv_poll,

	.halt = riscv_halt,
	.resume = esp_riscv_resume,
	.step = riscv_openocd_step,

	.assert_reset = esp_riscv_assert_reset,
	.deassert_reset = riscv_deassert_reset,

	.read_memory = esp_riscv_read_memory,
	.write_memory = esp_riscv_write_memory,

	.checksum_memory = riscv_checksum_memory,

	.get_gdb_arch = riscv_get_gdb_arch,
	.get_gdb_reg_list = riscv_get_gdb_reg_list,
	.get_gdb_reg_list_noread = esp_riscv_get_gdb_reg_list_noread,

	.add_breakpoint = esp_riscv_breakpoint_add,
	.remove_breakpoint = esp_riscv_breakpoint_remove,

	.add_watchpoint = esp_riscv_smp_watchpoint_add,
	.remove_watchpoint = esp_riscv_smp_watchpoint_remove,
	.hit_watchpoint = esp_riscv_hit_watchpoint,

	.arch_state = riscv_arch_state,

	.run_algorithm = esp_riscv_run_algorithm,
	.start_algorithm = esp_riscv_start_algorithm,
	.wait_algorithm = esp_riscv_wait_algorithm,

	.commands = esp32h4_command_handlers,

	.address_bits = riscv_xlen_nonconst,
};
