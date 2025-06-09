// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   ESP32-P4 target for OpenOCD                                           *
 *   Copyright (C) 2023 Espressif Systems Ltd.                             *
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
#define ESP32P4_LP_AON_BASE                     0x50110000
#define ESP32P4_LP_CLKRST_RESET_CAUSE_REG       (ESP32P4_LP_AON_BASE + 0x1000 + 0x10)
#define ESP32P4_RESET_CAUSE_MASK                (BIT(6) - 1) /* 0x3F */
#define ESP32P4_LP_CORE_RESET_CAUSE_SHIFT       0
#define ESP32P4_HP_CORE0_RESET_CAUSE_SHIFT      7
#define ESP32P4_HP_CORE1_RESET_CAUSE_SHIFT      14
#define ESP32P4_RESET_CAUSE(reg_val, shift)     ((reg_val >> shift) & ESP32P4_RESET_CAUSE_MASK)

/* cache */
#define ESP32P4_CACHE_BASE                      (0x3FF00000 + 0x10000)
#define ESP32P4_CACHE_SYNC_CTRL_REG             (ESP32P4_CACHE_BASE + 0x98)
#define ESP32P4_CACHE_SYNC_MAP_REG              (ESP32P4_CACHE_BASE + 0x9C)
#define ESP32P4_CACHE_SYNC_ADDR_REG             (ESP32P4_CACHE_BASE + 0xA0)
#define ESP32P4_CACHE_SYNC_SIZE_REG             (ESP32P4_CACHE_BASE + 0xA4)

#define ESP32P4_CACHE_MAP_L1_ICACHE0            BIT(0)
#define ESP32P4_CACHE_MAP_L1_ICACHE1            BIT(1)
#define ESP32P4_CACHE_MAP_L1_DCACHE             BIT(4)
#define ESP32P4_CACHE_MAP_L2_CACHE              BIT(5)
#define ESP32P4_CACHE_SYNC_INVALIDATE           BIT(0)
#define ESP32P4_CACHE_SYNC_WRITEBACK            BIT(2)
#define ESP32P4_CACHE_SYNC_FLUSH                BIT(3) /* Writeback + invalidate */
#define ESP32P4_CACHE_SYNC_DONE                 BIT(4)

#define ESP32P4_CACHE_L1_LINE_SIZE              64

#define ESP32P4_CACHE_MAP_L1_ICACHE (ESP32P4_CACHE_MAP_L1_ICACHE0 | ESP32P4_CACHE_MAP_L1_ICACHE1)
#define ESP32P4_CACHE_MAP_L1_CACHE (ESP32P4_CACHE_MAP_L1_ICACHE | ESP32P4_CACHE_MAP_L1_DCACHE)
#define ESP32P4_CACHE_MAP_ALL (ESP32P4_CACHE_MAP_L1_CACHE | ESP32P4_CACHE_MAP_L2_CACHE)

#define ESP32P4_EXRAM_CACHEABLE_ADDR_LOW        0x48000000U
#define ESP32P4_EXRAM_CACHEABLE_ADDR_HIGH       0x4BFFFFFFU
#define ESP32P4_IRAM0_CACHEABLE_ADDR_LOW        0x4ff00000U
#define ESP32P4_IRAM0_CACHEABLE_ADDR_HIGH       0x4ffc0000U
#define ESP32P4_NON_CACHEABLE_OFFSET            0x40000000U
#define ESP32P4_NON_CACHEABLE_ADDR(addr)        ((addr) + ESP32P4_NON_CACHEABLE_OFFSET)
#define ESP32P4_EXRAM_NON_CACHEABLE_ADDR_LOW     ESP32P4_NON_CACHEABLE_ADDR(ESP32P4_EXRAM_CACHEABLE_ADDR_LOW)
#define ESP32P4_EXRAM_NON_CACHEABLE_ADDR_HIGH    ESP32P4_NON_CACHEABLE_ADDR(ESP32P4_EXRAM_CACHEABLE_ADDR_HIGH)
#define ESP32P4_IRAM0_NON_CACHEABLE_ADDR_LOW     ESP32P4_NON_CACHEABLE_ADDR(ESP32P4_IRAM0_CACHEABLE_ADDR_LOW)
#define ESP32P4_IRAM0_NON_CACHEABLE_ADDR_HIGH    ESP32P4_NON_CACHEABLE_ADDR(ESP32P4_IRAM0_CACHEABLE_ADDR_HIGH)

#define ESP32P4_ADDR_IS_EXRAM_CACHEABLE(addr)    ((addr) >= ESP32P4_EXRAM_CACHEABLE_ADDR_LOW && \
	(addr) < ESP32P4_EXRAM_CACHEABLE_ADDR_HIGH)
#define ESP32P4_ADDR_IS_EXRAM_NONCACHEABLE(addr) ((addr) >= (ESP32P4_EXRAM_NON_CACHEABLE_ADDR_LOW) && \
	(addr) < (ESP32P4_EXRAM_NON_CACHEABLE_ADDR_HIGH))
#define ESP32P4_ADDR_IS_EXMEM(addr) (ESP32P4_ADDR_IS_EXRAM_NONCACHEABLE(addr) || ESP32P4_ADDR_IS_EXRAM_CACHEABLE(addr))

#define ESP32P4_ADDR_IS_IRAM_CACHEABLE(addr)      ((addr) >= ESP32P4_IRAM0_CACHEABLE_ADDR_LOW && \
	(addr) < ESP32P4_IRAM0_CACHEABLE_ADDR_HIGH)
#define ESP32P4_ADDR_IS_IRAM_NONCACHEABLE(addr)   ((addr) >= (ESP32P4_IRAM0_NON_CACHEABLE_ADDR_LOW) && \
	(addr) < (ESP32P4_IRAM0_NON_CACHEABLE_ADDR_HIGH))
#define ESP32P4_ADDR_IS_L2MEM(addr) (ESP32P4_ADDR_IS_IRAM_NONCACHEABLE(addr) || ESP32P4_ADDR_IS_IRAM_CACHEABLE(addr))

#define ESP32P4_ADDR_IS_CACHEABLE(addr) (ESP32P4_ADDR_IS_L2MEM(addr) || ESP32P4_ADDR_IS_EXMEM(addr))

#define ESP32P4_TCM_ADDR_LOW                    0x30100000U
#define ESP32P4_TCM_ADDR_HIGH                   0x30102000U
#define ESP32P4_ADDR_IS_TCMEM(addr) ((addr) >= ESP32P4_TCM_ADDR_LOW && (addr) < ESP32P4_TCM_ADDR_HIGH)

#define ESP32P4_RESERVED_ADDR_LOW               0x00000000U
#define ESP32P4_RESERVED_ADDR_HIGH              0x300FFFFFU

/* max supported hw breakpoint and watchpoint count */
#define ESP32P4_BP_NUM                          3
#define ESP32P4_WP_NUM                          3

#define ESP32P4_ASSIST_DEBUG_CPU0_MON_REG       0x3FF06000
#define ESP32P4_ASSIST_DEBUG_CPU_OFFSET         0x80

/* components/soc/esp32p4/include/soc/reset_reasons.h */
enum esp32p4_reset_reason {
	ESP32P4_CHIP_POWER_ON_RESET   = 0x01,	/* Power on reset */
	ESP32P4_CORE_SW_RESET         = 0x03,	/* Software resets the digital core */
	ESP32P4_SYS_PMU_PWR_DOWN_RESET = 0x05,	/* PMU HP power down system reset */
	ESP32P4_CPU_PMU_PWR_DOWN_RESET = 0x06,	/* PMU HP power down CPU reset */
	ESP32P4_SYS_HP_WDT_RESET      = 0x07,	/* HP WDT resets system */
	ESP32P4_SYS_LP_WDT_RESET      = 0x09,	/* LP WDT resets system */
	ESP32P4_SYS_LP_CPU_RESET      = 0x0A,	/* LP CPU reset */
	ESP32P4_CORE_HP_WDT_RESET     = 0x0B,	/* HP WDT resets digital core */
	ESP32P4_CPU0_SW_RESET         = 0x0C,	/* Software resets CPU 0 */
	ESP32P4_CORE_LP_WDT_RESET     = 0x0D,	/* LP WDT resets digital core */
	ESP32P4_SYS_BROWN_OUT_RESET   = 0x0F,	/* VDD voltage is not stable and resets the digital core */
	ESP32P4_CHIP_LP_WDT_RESET     = 0x10,	/* LP WDT resets chip */
	ESP32P4_SYS_SUPER_WDT_RESET   = 0x12,	/* Super watch dog resets the digital core and rtc module */
	ESP32P4_SYS_CLK_GLITCH_RESET  = 0x13,	/* Glitch on clock resets the digital core and rtc module */
	ESP32P4_CORE_EFUSE_CRC_RESET  = 0x14,	/* eFuse CRC error resets the digital core */
	ESP32P4_CORE_USB_JTAG_RESET   = 0x16,	/* USB JTAG resets the digital core */
	ESP32P4_CORE_USB_UART_RESET   = 0x17,	/* UART resets the digital core */
	ESP32P4_CPU_JTAG_RESET        = 0x18,	/* JTAG resets the digital core */
	ESP32P4_CPU_LOCKUP_RESET      = 0x1A,	/* Cpu lockup resets the chip */
};

static const char *esp32p4_get_reset_reason(uint32_t reset_reason_reg_val, int shift_val)
{
	switch (ESP32P4_RESET_CAUSE(reset_reason_reg_val, shift_val)) {
	case ESP32P4_CHIP_POWER_ON_RESET:
		return "Power on reset";
	case ESP32P4_CORE_SW_RESET:
		return "Software core reset";
	case ESP32P4_SYS_PMU_PWR_DOWN_RESET:
		return "PMU HP power down system reset";
	case ESP32P4_SYS_HP_WDT_RESET:
		return "HP WDT resets system";
	case ESP32P4_SYS_LP_WDT_RESET:
		return "LP WDT resets system";
	case ESP32P4_SYS_LP_CPU_RESET:
		return "PMU LP CPU reset";
	case ESP32P4_CORE_HP_WDT_RESET:
		return "HP WDT resets digital core";
	case ESP32P4_CPU0_SW_RESET:
		return "Software CPU reset";
	case ESP32P4_CORE_LP_WDT_RESET:
		return "LP WDT resets digital core";
	case ESP32P4_SYS_BROWN_OUT_RESET:
		return "Brown-out core reset";
	case ESP32P4_CHIP_LP_WDT_RESET:
		return "LP WDT resets chip";
	case ESP32P4_SYS_SUPER_WDT_RESET:
		return "Super Watchdog core and rtc";
	case ESP32P4_SYS_CLK_GLITCH_RESET:
		return "Glitch on clock reset";
	case ESP32P4_CORE_EFUSE_CRC_RESET:
	if (shift_val == ESP32P4_LP_CORE_RESET_CAUSE_SHIFT)
		return "PMU LP CPU reset";
	else
		return "eFuse CRC error core reset";
	case ESP32P4_CORE_USB_JTAG_RESET:
		return "USB (JTAG) core reset";
	case ESP32P4_CORE_USB_UART_RESET:
		return "UART resets the digital core";
	case ESP32P4_CPU_JTAG_RESET:
		return "JTAG CPU reset";
	case ESP32P4_CPU_LOCKUP_RESET:
		return "CPU Lockup reset";
	}
	return "Unknown reset cause";
}

static void esp32p4_print_reset_reason(struct target *target, uint32_t reset_reason_reg_val)
{
	if (target->coreid == 0)
		LOG_TARGET_INFO(target, "Reset cause (%ld) - (%s)",
			ESP32P4_RESET_CAUSE(reset_reason_reg_val, ESP32P4_HP_CORE0_RESET_CAUSE_SHIFT),
			esp32p4_get_reset_reason(reset_reason_reg_val, ESP32P4_HP_CORE0_RESET_CAUSE_SHIFT));
	else
		LOG_TARGET_INFO(target, "Reset cause (%ld) - (%s)",
			ESP32P4_RESET_CAUSE(reset_reason_reg_val, ESP32P4_HP_CORE1_RESET_CAUSE_SHIFT),
			esp32p4_get_reset_reason(reset_reason_reg_val, ESP32P4_HP_CORE1_RESET_CAUSE_SHIFT));

}

static bool esp32p4_is_idram_address(target_addr_t addr)
{
	return ESP32P4_ADDR_IS_L2MEM(addr) || ESP32P4_ADDR_IS_TCMEM(addr);
}

static bool esp32p4_is_reserved_address(target_addr_t addr)
{
	return addr < ESP32P4_RESERVED_ADDR_HIGH;
}

static const struct esp_semihost_ops esp32p4_semihost_ops = {
	.prepare = NULL,
	.post_reset = esp_semihosting_post_reset
};

static const struct esp_flash_breakpoint_ops esp32p4_flash_brp_ops = {
	.breakpoint_prepare = esp_algo_flash_breakpoint_prepare,
	.breakpoint_add = esp_algo_flash_breakpoint_add,
	.breakpoint_remove = esp_algo_flash_breakpoint_remove,
};

static const char *esp32p4_csrs[] = {
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

static int esp32p4_target_create(struct target *target)
{
	struct esp_riscv_common *esp_riscv = calloc(1, sizeof(*esp_riscv));
	if (!esp_riscv)
		return ERROR_FAIL;

	target->arch_info = esp_riscv;

	esp_riscv->assist_debug_cpu0_mon_reg = ESP32P4_ASSIST_DEBUG_CPU0_MON_REG;
	esp_riscv->assist_debug_cpu_offset = ESP32P4_ASSIST_DEBUG_CPU_OFFSET;

	esp_riscv->max_bp_num = ESP32P4_BP_NUM;
	esp_riscv->max_wp_num = ESP32P4_WP_NUM;

	esp_riscv->rtccntl_reset_state_reg = ESP32P4_LP_CLKRST_RESET_CAUSE_REG;
	esp_riscv->print_reset_reason = &esp32p4_print_reset_reason;
	esp_riscv->existent_csrs = esp32p4_csrs;
	esp_riscv->existent_csr_size = ARRAY_SIZE(esp32p4_csrs);
	esp_riscv->existent_ro_csrs = NULL;
	esp_riscv->existent_ro_csr_size = 0;
	esp_riscv->is_dram_address = esp32p4_is_idram_address;
	esp_riscv->is_iram_address = esp32p4_is_idram_address;

	if (esp_riscv_alloc_trigger_addr(target) != ERROR_OK)
		return ERROR_FAIL;

	riscv_info_init(target, &esp_riscv->riscv);

	return ERROR_OK;
}

static int esp32p4_init_target(struct command_context *cmd_ctx,
	struct target *target)
{
	int ret = riscv_target.init_target(cmd_ctx, target);
	if (ret != ERROR_OK)
		return ret;

	target->semihosting->user_command_extension = esp_semihosting_common;

	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);

	ret = esp_riscv_init_arch_info(target,
		esp_riscv,
		&esp32p4_flash_brp_ops,
		&esp32p4_semihost_ops);
	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

static int esp32p4_sync_cache(struct target *target, target_addr_t address, uint32_t size, uint32_t map,
	uint32_t op)
{
	uint8_t value_buf[4];
	target_addr_t start_aligned_addr = ALIGN_DOWN(address, ESP32P4_CACHE_L1_LINE_SIZE);
	target_addr_t end_aligned_addr = ALIGN_DOWN(address + size + ESP32P4_CACHE_L1_LINE_SIZE - 1,
		ESP32P4_CACHE_L1_LINE_SIZE);
	uint32_t aligned_size = end_aligned_addr - start_aligned_addr;

	// TODO: what if cache is disabled! No way to understand from the OpenOCD point of view.

	target_buffer_set_u32(target, value_buf, map);
	int res = esp_riscv_write_memory(target, ESP32P4_CACHE_SYNC_MAP_REG, 4, 1, value_buf);
	if (res != ERROR_OK)
		return res;

	target_buffer_set_u32(target, value_buf, start_aligned_addr);
	res = esp_riscv_write_memory(target, ESP32P4_CACHE_SYNC_ADDR_REG, 4, 1, value_buf);
	if (res != ERROR_OK)
		return res;

	target_buffer_set_u32(target, value_buf, aligned_size);
	res = esp_riscv_write_memory(target, ESP32P4_CACHE_SYNC_SIZE_REG, 4, 1, value_buf);
	if (res != ERROR_OK)
		return res;

	target_buffer_set_u32(target, value_buf, op);
	return esp_riscv_write_memory(target, ESP32P4_CACHE_SYNC_CTRL_REG, 4, 1, value_buf);

	/* Looks like no need to wait for sync done. Everytime ESP32P4_CACHE_SYNC_CTRL_REG read as 0x10 at first try */
}

static int esp32p4_read_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	if (esp32p4_is_reserved_address(address)) {
		/* TODO: OCD-976 */
		memset(buffer, 0, size * count);
		return ERROR_OK;
	}

	if (ESP32P4_ADDR_IS_CACHEABLE(address)) {
		/* Write-back is for dcache and l2 cache only */
		int res = esp32p4_sync_cache(target, address, size * count,
			ESP32P4_CACHE_MAP_L1_DCACHE | ESP32P4_CACHE_MAP_L2_CACHE,
			ESP32P4_CACHE_SYNC_WRITEBACK);
		if (res != ERROR_OK)
			LOG_TARGET_WARNING(target, "Cache writeback failed! Read main memory anyway.");
	}

	return esp_riscv_read_memory(target, address, size, count, buffer);
}

static int esp32p4_write_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer)
{
	int map = -1;
	if (ESP32P4_ADDR_IS_CACHEABLE(address)) {
		/* Write-back is for dcache and l2 cache only */
		map = ESP32P4_CACHE_MAP_L1_DCACHE | ESP32P4_CACHE_MAP_L2_CACHE;
	}

	if (map > 0) {
		/* write to main memory and invalidate cache */
		int res = esp32p4_sync_cache(target, address, size * count, map, ESP32P4_CACHE_SYNC_WRITEBACK);
		if (res != ERROR_OK)
			LOG_TARGET_WARNING(target, "Cache writeback failed! Write main memory anyway.");
	}

	int res = esp_riscv_write_memory(target, address, size, count, buffer);

	if (map > 0) {
		/* Don't invalidate the L2CACHE here. We don't know if it has been written back to the PSRAM yet. */
		map = ESP32P4_CACHE_MAP_L1_CACHE;
		if (esp32p4_sync_cache(target, address, size * count, map, ESP32P4_CACHE_SYNC_INVALIDATE) != ERROR_OK)
			LOG_TARGET_WARNING(target, "Cache invalidate failed!");
	}
	return res;
}

static const struct command_registration esp32p4_command_handlers[] = {
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

struct target_type esp32p4_target = {
	.name = "esp32p4",

	.target_create = esp32p4_target_create,
	.init_target = esp32p4_init_target,
	.deinit_target = esp_riscv_deinit_target,
	.examine = esp_riscv_examine,

	/* poll current target status */
	.poll = esp_riscv_poll,

	.halt = riscv_halt,
	.resume = esp_riscv_resume,
	.step = riscv_openocd_step,

	.assert_reset = esp_riscv_assert_reset,
	.deassert_reset = riscv_deassert_reset,

	.read_memory = esp32p4_read_memory,
	.write_memory = esp32p4_write_memory,

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

	.commands = esp32p4_command_handlers,

	.address_bits = riscv_xlen_nonconst,
};
