/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP32-S3 target API for OpenOCD                                       *
 *   Copyright (C) 2020 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/time_support.h>
#include <target/target.h>
#include <target/target_type.h>
#include <target/smp.h>
#include <target/semihosting_common.h>
#include "assert.h"
#include <rtos/rtos.h>
#include <flash/nor/esp_xtensa.h>
#include "esp32_apptrace.h"
#include "esp_xtensa_smp.h"
#include "esp_semihosting.h"

/*
This is a JTAG driver for the ESP32_S3, the are two Tensilica cores inside
the ESP32_S3 chip. For more information please have a look into ESP32_S3 target
implementation.
*/

/* ESP32_S3 memory map */
#define ESP32_S3_IRAM_LOW               0x40370000
#define ESP32_S3_IRAM_HIGH              0x403E0000
#define ESP32_S3_IROM_MASK_LOW          0x40000000
#define ESP32_S3_IROM_MASK_HIGH         0x40060000
#define ESP32_S3_INTERNAL_DROM_LOW      0x3FF00000
#define ESP32_S3_INTERNAL_DROM_HIGH     0x3FF20000
#define ESP32_S3_DRAM_LOW               0x3FC88000
#define ESP32_S3_DRAM_HIGH              0x3FD00000
#define ESP32_S3_RTC_IRAM_LOW           0x600FE000
#define ESP32_S3_RTC_IRAM_HIGH          0x60100000
#define ESP32_S3_RTC_DRAM_LOW           0x600FE000
#define ESP32_S3_RTC_DRAM_HIGH          0x60100000
#define ESP32_S3_RTC_DATA_LOW           0x50000000
#define ESP32_S3_RTC_DATA_HIGH          0x50002000
#define ESP32_S3_EXTRAM_DATA_LOW        0x3D000000
#define ESP32_S3_EXTRAM_DATA_HIGH       0x3E000000
#define ESP32_S3_SYS_RAM_LOW            0x60000000UL
#define ESP32_S3_SYS_RAM_HIGH           (ESP32_S3_SYS_RAM_LOW + 0x10000000UL)
#define ESP32_S3_RTC_SLOW_MEM_BASE      ESP32_S3_RTC_DATA_LOW

/* ESP32_S3 WDT */
#define ESP32_S3_WDT_WKEY_VALUE       0x50D83AA1
#define ESP32_S3_TIMG0_BASE           0x6001F000
#define ESP32_S3_TIMG1_BASE           0x60020000
#define ESP32_S3_TIMGWDT_CFG0_OFF     0x48
#define ESP32_S3_TIMGWDT_PROTECT_OFF  0x64
#define ESP32_S3_TIMG0WDT_CFG0        (ESP32_S3_TIMG0_BASE + ESP32_S3_TIMGWDT_CFG0_OFF)
#define ESP32_S3_TIMG1WDT_CFG0        (ESP32_S3_TIMG1_BASE + ESP32_S3_TIMGWDT_CFG0_OFF)
#define ESP32_S3_TIMG0WDT_PROTECT     (ESP32_S3_TIMG0_BASE + ESP32_S3_TIMGWDT_PROTECT_OFF)
#define ESP32_S3_TIMG1WDT_PROTECT     (ESP32_S3_TIMG1_BASE + ESP32_S3_TIMGWDT_PROTECT_OFF)
#define ESP32_S3_RTCCNTL_BASE         0x60008000
#define ESP32_S3_RTCWDT_CFG_OFF       0x98
#define ESP32_S3_RTCWDT_PROTECT_OFF   0xB0
#define ESP32_S3_SWD_CONF_OFF         0xB0
#define ESP32_S3_SWD_WPROTECT_OFF     0xB4
#define ESP32_S3_RTCWDT_CFG           (ESP32_S3_RTCCNTL_BASE + ESP32_S3_RTCWDT_CFG_OFF)
#define ESP32_S3_RTCWDT_PROTECT       (ESP32_S3_RTCCNTL_BASE + ESP32_S3_RTCWDT_PROTECT_OFF)
#define ESP32_S3_SWD_CONF_REG         (ESP32_S3_RTCCNTL_BASE + ESP32_S3_SWD_CONF_OFF)
#define ESP32_S3_SWD_WPROTECT_REG     (ESP32_S3_RTCCNTL_BASE + ESP32_S3_SWD_WPROTECT_OFF)
#define ESP32_S3_SWD_AUTO_FEED_EN_M   BIT(31)
#define ESP32_S3_SWD_WKEY_VALUE       0x8F1D312AU

#define ESP32_S3_TRACEMEM_BLOCK_SZ    0x4000

/* ESP32_S3 dport regs */
#define ESP32_S3_DR_REG_SYSTEM_BASE                0x600c0000
#define ESP32_S3_SYSTEM_CORE_1_CONTROL_0_REG       (ESP32_S3_DR_REG_SYSTEM_BASE + 0x014)
#define ESP32_S3_SYSTEM_CONTROL_CORE_1_CLKGATE_EN  BIT(1)

/* ESP32_S3 RTC regs */
#define ESP32_S3_RTC_CNTL_SW_CPU_STALL_REG      (ESP32_S3_RTCCNTL_BASE + 0xBC)
#define ESP32_S3_RTC_CNTL_SW_CPU_STALL_DEF      0x0
#define ESP32S3_RTC_CNTL_RESET_STATE_REG        (ESP32_S3_RTCCNTL_BASE + 0x38)

/* RTC_CNTL_RESET_CAUSE_APPCPU : RO ;bitpos:[11:6] ;default: 0 ;
 *description: reset cause of APP CPU.*/
#define ESP32S3_RTC_CNTL_RESET_CAUSE_APPCPU    0x0000003F
#define ESP32S3_RTC_CNTL_RESET_CAUSE_APPCPU_M  ((ESP32S3_RTC_CNTL_RESET_CAUSE_APPCPU_V) << \
		(ESP32S3_RTC_CNTL_RESET_CAUSE_APPCPU_S))
#define ESP32S3_RTC_CNTL_RESET_CAUSE_APPCPU_V  0x3F
#define ESP32S3_RTC_CNTL_RESET_CAUSE_APPCPU_S  6
/* RTC_CNTL_RESET_CAUSE_PROCPU : RO ;bitpos:[5:0] ;default: 0 ;
 *description: reset cause of PRO CPU.*/
#define ESP32S3_RTC_CNTL_RESET_CAUSE_PROCPU    0x0000003F
#define ESP32S3_RTC_CNTL_RESET_CAUSE_PROCPU_M  ((ESP32S3_RTC_CNTL_RESET_CAUSE_PROCPU_V) << \
		(ESP32S3_RTC_CNTL_RESET_CAUSE_PROCPU_S))
#define ESP32S3_RTC_CNTL_RESET_CAUSE_PROCPU_V  0x3F
#define ESP32S3_RTC_CNTL_RESET_CAUSE_PROCPU_S  0

enum esp32s3_reset_reason {
	ESP32S3_CHIP_POWER_ON_RESET   = 0x01,	/* Power on reset */
	ESP32S3_CHIP_BROWN_OUT_RESET  = 0x01,	/* VDD voltage is not stable and resets the chip */
	ESP32S3_CHIP_SUPER_WDT_RESET  = 0x01,	/* Super watch dog resets the chip */
	ESP32S3_CORE_SW_RESET         = 0x03,	/* Software resets the digital core by RTC_CNTL_SW_SYS_RST */
	ESP32S3_CORE_DEEP_SLEEP_RESET = 0x05,	/* Deep sleep reset the digital core */
	ESP32S3_CORE_MWDT0_RESET      = 0x07,	/* Main watch dog 0 resets digital core */
	ESP32S3_CORE_MWDT1_RESET      = 0x08,	/* Main watch dog 1 resets digital core */
	ESP32S3_CORE_RTC_WDT_RESET    = 0x09,	/* RTC watch dog resets digital core */
	ESP32S3_CPU0_MWDT0_RESET      = 0x0B,	/* Main watch dog 0 resets CPU 0 */
	ESP32S3_CPU1_MWDT0_RESET      = 0x0B,	/* Main watch dog 0 resets CPU 1 */
	ESP32S3_CPU0_SW_RESET         = 0x0C,	/* Software resets CPU 0 by RTC_CNTL_SW_PROCPU_RST */
	ESP32S3_CPU1_SW_RESET         = 0x0C,	/* Software resets CPU 1 by RTC_CNTL_SW_APPCPU_RST */
	ESP32S3_CPU0_RTC_WDT_RESET    = 0x0D,	/* RTC watch dog resets CPU 0 */
	ESP32S3_CPU1_RTC_WDT_RESET    = 0x0D,	/* RTC watch dog resets CPU 1 */
	ESP32S3_SYS_BROWN_OUT_RESET   = 0x0F,	/* VDD voltage is not stable and resets the digital core */
	ESP32S3_SYS_RTC_WDT_RESET     = 0x10,	/* RTC watch dog resets digital core and rtc module */
	ESP32S3_CPU0_MWDT1_RESET      = 0x11,	/* Main watch dog 1 resets CPU 0 */
	ESP32S3_CPU1_MWDT1_RESET      = 0x11,	/* Main watch dog 1 resets CPU 1 */
	ESP32S3_SYS_SUPER_WDT_RESET   = 0x12,	/* Super watch dog resets the digital core and rtc module */
	ESP32S3_SYS_CLK_GLITCH_RESET  = 0x13,	/* Glitch on clock resets the digital core and rtc module */
	ESP32S3_CORE_EFUSE_CRC_RESET  = 0x14,	/* eFuse CRC error resets the digital core */
	ESP32S3_CORE_USB_UART_RESET   = 0x15,	/* USB UART resets the digital core */
	ESP32S3_CORE_USB_JTAG_RESET   = 0x16,	/* USB JTAG resets the digital core */
	ESP32S3_CORE_PWR_GLITCH_RESET = 0x17,	/* Glitch on power resets the digital core */
};

struct esp32s3_common {
	struct esp_xtensa_smp_common esp_xtensa_smp;
};

/* Reset ESP32-S3's peripherals.
 * 1. OpenOCD makes sure the target is halted; if not, tries to halt it.
 *    If that fails, tries to reset it (via OCD) and then halt.
 * 2. OpenOCD loads the stub code into RTC_SLOW_MEM.
 * 3. Executes the stub code from address 0x50000004.
 * 4. The stub code changes the reset vector to 0x50000000, and triggers
 *    a system reset using RTC_CNTL_SW_SYS_RST bit.
 * 5. Once the PRO CPU is out of reset, it executes the stub code from address 0x50000000.
 *    The stub code disables the watchdog, re-enables JTAG and the APP CPU,
 *    restores the reset vector, and enters an infinite loop.
 * 6. OpenOCD waits until it can talk to the OCD module again, then halts the target.
 * 7. OpenOCD restores the contents of RTC_SLOW_MEM.
 *
 * End result: all the peripherals except RTC_CNTL are reset, CPU's PC is undefined,
 * PRO CPU is halted, APP CPU is in reset.
 */

static const uint8_t esp32s3_reset_stub_code[] = {
#include "../../../contrib/loaders/reset/espressif/esp32s3/cpu_reset_handler_code.inc"
};

static int esp32s3_soc_reset(struct target *target)
{
	int res;
	struct target_list *head;
	struct xtensa *xtensa;

	LOG_DEBUG("start");
	/* In order to write to peripheral registers, target must be halted first */
	if (target->state != TARGET_HALTED) {
		LOG_DEBUG("Target not halted before SoC reset, trying to halt it first");
		xtensa_halt(target);
		res = target_wait_state(target, TARGET_HALTED, 1000);
		if (res != ERROR_OK) {
			LOG_DEBUG("Couldn't halt target before SoC reset, trying to do reset-halt");
			res = xtensa_assert_reset(target);
			if (res != ERROR_OK) {
				LOG_ERROR(
					"Couldn't halt target before SoC reset! (xtensa_assert_reset returned %d)",
					res);
				return res;
			}
			alive_sleep(10);
			xtensa_poll(target);
			bool reset_halt_save = target->reset_halt;
			target->reset_halt = true;
			res = xtensa_deassert_reset(target);
			target->reset_halt = reset_halt_save;
			if (res != ERROR_OK) {
				LOG_ERROR(
					"Couldn't halt target before SoC reset! (xtensa_deassert_reset returned %d)",
					res);
				return res;
			}
			alive_sleep(10);
			xtensa_poll(target);
			xtensa_halt(target);
			res = target_wait_state(target, TARGET_HALTED, 1000);
			if (res != ERROR_OK) {
				LOG_ERROR("Couldn't halt target before SoC reset");
				return res;
			}
		}
	}

	if (target->smp) {
		foreach_smp_target(head, target->smp_targets) {
			xtensa = target_to_xtensa(head->target);
			/* if any of the cores is stalled unstall them */
			if (xtensa_dm_core_is_stalled(&xtensa->dbg_mod)) {
				LOG_TARGET_DEBUG(head->target, "Unstall CPUs before SW reset!");
				res = target_write_u32(target,
					ESP32_S3_RTC_CNTL_SW_CPU_STALL_REG,
					ESP32_S3_RTC_CNTL_SW_CPU_STALL_DEF);
				if (res != ERROR_OK) {
					LOG_TARGET_ERROR(head->target, "Failed to unstall CPUs before SW reset!");
					return res;
				}
				break;	/* both cores are unstalled now, so exit the loop */
			}
		}
	}

	LOG_DEBUG("Loading stub code into RTC RAM");
	uint8_t slow_mem_save[sizeof(esp32s3_reset_stub_code)];

	/* Save contents of RTC_SLOW_MEM which we are about to overwrite */
	res = target_read_buffer(target, ESP32_S3_RTC_SLOW_MEM_BASE, sizeof(slow_mem_save), slow_mem_save);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to save contents of RTC_SLOW_MEM (%d)!", res);
		return res;
	}

	/* Write stub code into RTC_SLOW_MEM */
	res = target_write_buffer(target,
		ESP32_S3_RTC_SLOW_MEM_BASE,
		sizeof(esp32s3_reset_stub_code),
		esp32s3_reset_stub_code);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write stub (%d)!", res);
		return res;
	}

	LOG_DEBUG("Resuming the target");
	xtensa = target_to_xtensa(target);
	xtensa->suppress_dsr_errors = true;
	res = xtensa_resume(target, false, ESP32_S3_RTC_SLOW_MEM_BASE + 4, false,
		false);
	xtensa->suppress_dsr_errors = false;
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to run stub (%d)!", res);
		return res;
	}
	LOG_DEBUG("resume done, waiting for the target to come alive");

	/* Wait for SoC to reset */
	alive_sleep(100);
	int64_t timeout = timeval_ms() + 100;
	bool get_timeout = false;
	while (target->state != TARGET_RESET && target->state != TARGET_RUNNING) {
		alive_sleep(10);
		xtensa_poll(target);
		if (timeval_ms() >= timeout) {
			LOG_TARGET_ERROR(target,
				"Timed out waiting for CPU to be reset, target state=%d",
				target->state);
			get_timeout = true;
			break;
		}
	}

	/* Halt the CPU again */
	LOG_DEBUG("halting the target");
	xtensa_halt(target);
	res = target_wait_state(target, TARGET_HALTED, 1000);
	if (res == ERROR_OK) {
		LOG_DEBUG("restoring RTC_SLOW_MEM");
		res = target_write_buffer(target, ESP32_S3_RTC_SLOW_MEM_BASE, sizeof(slow_mem_save), slow_mem_save);
		if (res != ERROR_OK)
			LOG_TARGET_ERROR(target, "Failed to restore contents of RTC_SLOW_MEM (%d)!", res);
	} else {
		LOG_TARGET_ERROR(target, "Timed out waiting for CPU to be halted after SoC reset");
	}

	foreach_smp_target(head, target->smp_targets) {
		struct target *curr = head->target;
		if (target_was_examined(curr))
			esp_xtensa_reset_reason_read(curr);
	}

	/* Clear memory which is used by RTOS layer to get the task count */
	if (target->rtos && target->rtos->type->post_reset_cleanup) {
		res = (*target->rtos->type->post_reset_cleanup)(target);
		if (res != ERROR_OK)
			LOG_WARNING("Failed to do rtos-specific cleanup (%d)", res);
	}

	return get_timeout ? ERROR_TARGET_TIMEOUT : res;
}

static int esp32s3_disable_wdts(struct target *target)
{
	/* TIMG1 WDT */
	int res = target_write_u32(target, ESP32_S3_TIMG0WDT_PROTECT, ESP32_S3_WDT_WKEY_VALUE);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_S3_TIMG0WDT_PROTECT (%d)!", res);
		return res;
	}
	res = target_write_u32(target, ESP32_S3_TIMG0WDT_CFG0, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_S3_TIMG0WDT_CFG0 (%d)!", res);
		return res;
	}
	/* TIMG2 WDT */
	res = target_write_u32(target, ESP32_S3_TIMG1WDT_PROTECT, ESP32_S3_WDT_WKEY_VALUE);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_S3_TIMG1WDT_PROTECT (%d)!", res);
		return res;
	}
	res = target_write_u32(target, ESP32_S3_TIMG1WDT_CFG0, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_S3_TIMG1WDT_CFG0 (%d)!", res);
		return res;
	}
	/* RTC WDT */
	res = target_write_u32(target, ESP32_S3_RTCWDT_PROTECT, ESP32_S3_WDT_WKEY_VALUE);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_S3_RTCWDT_PROTECT (%d)!", res);
		return res;
	}
	res = target_write_u32(target, ESP32_S3_RTCWDT_CFG, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_S3_RTCWDT_CFG (%d)!", res);
		return res;
	}
	/* Enable SWD auto-feed */
	res = target_write_u32(target, ESP32_S3_SWD_WPROTECT_REG, ESP32_S3_SWD_WKEY_VALUE);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_S3_SWD_WPROTECT_REG (%d)!", res);
		return res;
	}
	uint32_t swd_conf_reg = 0;
	res = target_read_u32(target, ESP32_S3_SWD_CONF_REG, &swd_conf_reg);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to read ESP32_S3_SWD_CONF_REG (%d)!", res);
		return res;
	}
	swd_conf_reg |= ESP32_S3_SWD_AUTO_FEED_EN_M;
	res = target_write_u32(target, ESP32_S3_SWD_CONF_REG, swd_conf_reg);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write ESP32_S3_SWD_CONF_REG (%d)!", res);
		return res;
	}
	return ERROR_OK;
}

static int esp32s3_on_halt(struct target *target)
{
	int ret = esp32s3_disable_wdts(target);
	if (ret == ERROR_OK)
		ret = esp_xtensa_smp_on_halt(target);
	return ret;
}

static int esp32s3_arch_state(struct target *target)
{
	return ERROR_OK;
}

static int esp32s3_virt2phys(struct target *target,
	target_addr_t virtual, target_addr_t *physical)
{
	if (physical) {
		*physical = virtual;
		return ERROR_OK;
	}
	return ERROR_FAIL;
}

static const char *esp32s3_reset_reason_str(int coreid, enum esp32s3_reset_reason reset_number)
{
	switch (reset_number) {
	case ESP32S3_CHIP_POWER_ON_RESET:
		return "Power on reset";
	case ESP32S3_CORE_SW_RESET:
		return "Software core reset";
	case ESP32S3_CORE_DEEP_SLEEP_RESET:
		return "Deep-sleep core reset";
	case ESP32S3_CORE_MWDT0_RESET:
		return "Main WDT0 core reset";
	case ESP32S3_CORE_MWDT1_RESET:
		return "Main WDT1 core reset";
	case ESP32S3_CORE_RTC_WDT_RESET:
		return "RTC WDT core reset";
	case ESP32S3_CPU0_MWDT0_RESET:
		/* has the same value as ESP32S3_CPU0_MWDT0_RESET
		case ESP32S3_CPU1_MWDT0_RESET:*/
		return coreid ? "Main WDT0 CPU1 reset" : "Main WDT0 CPU0 reset";
	case ESP32S3_CPU0_SW_RESET:
		/* has the same value as ESP32S3_CPU0_SW_RESET
		case RESET_REASON_CPU1_SW_RESET:*/
		return coreid ? "Software CPU1 reset" : "Software CPU0 reset";
	case ESP32S3_CPU0_RTC_WDT_RESET:
		/* has the same value as ESP32S3_CPU0_RTC_WDT_RESET
		case ESP32S3_CPU1_RTC_WDT_RESET:*/
		return coreid ? "RTC WDT CPU1 reset" : "RTC WDT CPU0 reset";
	case ESP32S3_SYS_BROWN_OUT_RESET:
		return "Brown-out core reset";
	case ESP32S3_SYS_RTC_WDT_RESET:
		return "RTC WDT core and rtc reset";
	case ESP32S3_CPU0_MWDT1_RESET:
		/* has the same value as ESP32S3_CPU1_MWDT1_RESET
		case ESP32S3_CPU1_MWDT1_RESET:*/
		return coreid ? "Main WDT1 resets CPU1" : "Main WDT1 resets CPU0";
	case ESP32S3_SYS_SUPER_WDT_RESET:
		return "Super WDT reset";
	case ESP32S3_SYS_CLK_GLITCH_RESET:
		return "Glitch on clock reset";
	case ESP32S3_CORE_EFUSE_CRC_RESET:
		return "eFuse CRC error reset";
	case ESP32S3_CORE_USB_UART_RESET:
		return "USB UART reset";
	case ESP32S3_CORE_USB_JTAG_RESET:
		return "USB JTAG reset";
	case ESP32S3_CORE_PWR_GLITCH_RESET:
		return "Core power glitch reset";
	}
	return "Unknown reset cause";
}

static int esp32s3_reset_reason_fetch(struct target *target, int *rsn_id, const char **rsn_str)
{
	uint32_t rsn_val;

	int ret = target_read_u32(target, ESP32S3_RTC_CNTL_RESET_STATE_REG, &rsn_val);
	if (ret != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Error on read reset reason register (%d)!", ret);
		return ret;
	}
	if (target->coreid == 0) {
		rsn_val &= ESP32S3_RTC_CNTL_RESET_CAUSE_PROCPU_M;
		rsn_val >>= ESP32S3_RTC_CNTL_RESET_CAUSE_PROCPU_S;
	} else {
		rsn_val &= ESP32S3_RTC_CNTL_RESET_CAUSE_APPCPU_M;
		rsn_val >>= ESP32S3_RTC_CNTL_RESET_CAUSE_APPCPU_S;
	}

	/* sanity check for valid value */
	if (rsn_val == 0)
		return ERROR_FAIL;

	*rsn_id = rsn_val;
	*rsn_str = esp32s3_reset_reason_str(target->coreid, rsn_val);
	return ERROR_OK;
}

static int esp32s3_target_init(struct command_context *cmd_ctx, struct target *target)
{
	return esp_xtensa_smp_target_init(cmd_ctx, target);
}

static const struct xtensa_debug_ops esp32s3_dbg_ops = {
	.queue_enable = xtensa_dm_queue_enable,
	.queue_reg_read = xtensa_dm_queue_reg_read,
	.queue_reg_write = xtensa_dm_queue_reg_write
};

static const struct xtensa_power_ops esp32s3_pwr_ops = {
	.queue_reg_read = xtensa_dm_queue_pwr_reg_read,
	.queue_reg_write = xtensa_dm_queue_pwr_reg_write
};

static const struct esp_flash_breakpoint_ops esp32s3_flash_brp_ops = {
	.breakpoint_prepare = esp_algo_flash_breakpoint_prepare,
	.breakpoint_add = esp_algo_flash_breakpoint_add,
	.breakpoint_remove = esp_algo_flash_breakpoint_remove,
};

static const struct esp_xtensa_smp_chip_ops esp32s3_chip_ops = {
	.reset = esp32s3_soc_reset,
	.on_halt = esp32s3_on_halt
};

static const struct esp_semihost_ops esp32s3_semihost_ops = {
	.prepare = esp32s3_disable_wdts,
	.post_reset = esp_semihosting_post_reset
};

static int esp32s3_target_create(struct target *target)
{
	struct xtensa_debug_module_config esp32s3_dm_cfg = {
		.dbg_ops = &esp32s3_dbg_ops,
		.pwr_ops = &esp32s3_pwr_ops,
		.tap = target->tap,
		.queue_tdi_idle = NULL,
		.queue_tdi_idle_arg = NULL,
		.dap = NULL,
		.debug_ap = NULL,
		.debug_apsel = DP_APSEL_INVALID,
		.ap_offset = 0,
	};

	struct esp32s3_common *esp32s3 = calloc(1, sizeof(struct esp32s3_common));
	if (!esp32s3) {
		LOG_ERROR("Failed to alloc memory for arch info!");
		return ERROR_FAIL;
	}

	struct esp_ops esp32s3_ops = {
		.flash_brps_ops = &esp32s3_flash_brp_ops,
		.chip_ops = &esp32s3_chip_ops,
		.semihost_ops = &esp32s3_semihost_ops,
		.reset_reason_fetch = esp32s3_reset_reason_fetch
	};

	int ret = esp_xtensa_smp_init_arch_info(target,
		&esp32s3->esp_xtensa_smp,
		&esp32s3_dm_cfg,
		&esp32s3_ops);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to init arch info!");
		free(esp32s3);
		return ret;
	}

	/* Assume running target. If different, the first poll will fix this. */
	target->state = TARGET_RUNNING;
	target->debug_reason = DBG_REASON_NOTHALTED;
	return ERROR_OK;
}

static const struct command_registration esp32s3_command_handlers[] = {
	{
		.usage = "",
		.chain = esp_xtensa_smp_command_handlers,
	},
	{
		.name = "esp",
		.usage = "",
		.chain = esp32_apptrace_command_handlers,
	},
	{
		.name = "esp32",
		.usage = "",
		.chain = smp_command_handlers,
	},
	{
		.name = "arm",
		.mode = COMMAND_ANY,
		.help = "ARM Command Group",
		.usage = "",
		.chain = semihosting_common_handlers
	},
	COMMAND_REGISTRATION_DONE
};

/** Holds methods for Xtensa targets. */
struct target_type esp32s3_target = {
	.name = "esp32s3",

	.poll = esp_xtensa_smp_poll,
	.arch_state = esp32s3_arch_state,

	.halt = xtensa_halt,
	.resume = esp_xtensa_smp_resume,
	.step = esp_xtensa_smp_step,

	.assert_reset = esp_xtensa_smp_assert_reset,
	.deassert_reset = esp_xtensa_smp_deassert_reset,
	.soft_reset_halt = esp_xtensa_smp_soft_reset_halt,

	.virt2phys = esp32s3_virt2phys,
	.mmu = xtensa_mmu_is_enabled,
	.read_memory = xtensa_read_memory,
	.write_memory = xtensa_write_memory,

	.read_buffer = xtensa_read_buffer,
	.write_buffer = xtensa_write_buffer,

	.checksum_memory = xtensa_checksum_memory,

	.get_gdb_arch = xtensa_get_gdb_arch,
	.get_gdb_reg_list = xtensa_get_gdb_reg_list,

	.run_algorithm = xtensa_run_algorithm,
	.start_algorithm = xtensa_start_algorithm,
	.wait_algorithm = xtensa_wait_algorithm,

	.add_breakpoint = esp_xtensa_breakpoint_add,
	.remove_breakpoint = esp_xtensa_breakpoint_remove,

	.add_watchpoint = esp_xtensa_smp_watchpoint_add,
	.remove_watchpoint = esp_xtensa_smp_watchpoint_remove,
	.hit_watchpoint = xtensa_watchpoint_hit,

	.target_create = esp32s3_target_create,
	.init_target = esp32s3_target_init,
	.examine = xtensa_examine,
	.deinit_target = esp_xtensa_target_deinit,

	.commands = esp32s3_command_handlers,
	.profiling = esp_xtensa_profiling,
};
