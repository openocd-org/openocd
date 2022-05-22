/***************************************************************************
 *   ESP32-S3 target API for OpenOCD                                       *
 *   Copyright (C) 2020 Espressif Systems Ltd.                             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/time_support.h>
#include <target/target.h>
#include <target/target_type.h>
#include <target/smp.h>
#include "assert.h"
#include "esp32s3.h"
#include "esp_xtensa_smp.h"

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
#define ESP32_S3_RTC_CNTL_SW_CPU_STALL_REG (ESP32_S3_RTCCNTL_BASE + 0xBC)
#define ESP32_S3_RTC_CNTL_SW_CPU_STALL_DEF 0x0

/* this should map local reg IDs to GDB reg mapping as defined in xtensa-config.c 'rmap' in
 *xtensa-overlay */
static const unsigned int esp32s3_gdb_regs_mapping[ESP32_S3_NUM_REGS] = {
	XT_REG_IDX_PC,
	XT_REG_IDX_AR0, XT_REG_IDX_AR1, XT_REG_IDX_AR2, XT_REG_IDX_AR3,
	XT_REG_IDX_AR4, XT_REG_IDX_AR5, XT_REG_IDX_AR6, XT_REG_IDX_AR7,
	XT_REG_IDX_AR8, XT_REG_IDX_AR9, XT_REG_IDX_AR10, XT_REG_IDX_AR11,
	XT_REG_IDX_AR12, XT_REG_IDX_AR13, XT_REG_IDX_AR14, XT_REG_IDX_AR15,
	XT_REG_IDX_AR16, XT_REG_IDX_AR17, XT_REG_IDX_AR18, XT_REG_IDX_AR19,
	XT_REG_IDX_AR20, XT_REG_IDX_AR21, XT_REG_IDX_AR22, XT_REG_IDX_AR23,
	XT_REG_IDX_AR24, XT_REG_IDX_AR25, XT_REG_IDX_AR26, XT_REG_IDX_AR27,
	XT_REG_IDX_AR28, XT_REG_IDX_AR29, XT_REG_IDX_AR30, XT_REG_IDX_AR31,
	XT_REG_IDX_AR32, XT_REG_IDX_AR33, XT_REG_IDX_AR34, XT_REG_IDX_AR35,
	XT_REG_IDX_AR36, XT_REG_IDX_AR37, XT_REG_IDX_AR38, XT_REG_IDX_AR39,
	XT_REG_IDX_AR40, XT_REG_IDX_AR41, XT_REG_IDX_AR42, XT_REG_IDX_AR43,
	XT_REG_IDX_AR44, XT_REG_IDX_AR45, XT_REG_IDX_AR46, XT_REG_IDX_AR47,
	XT_REG_IDX_AR48, XT_REG_IDX_AR49, XT_REG_IDX_AR50, XT_REG_IDX_AR51,
	XT_REG_IDX_AR52, XT_REG_IDX_AR53, XT_REG_IDX_AR54, XT_REG_IDX_AR55,
	XT_REG_IDX_AR56, XT_REG_IDX_AR57, XT_REG_IDX_AR58, XT_REG_IDX_AR59,
	XT_REG_IDX_AR60, XT_REG_IDX_AR61, XT_REG_IDX_AR62, XT_REG_IDX_AR63,
	XT_REG_IDX_LBEG, XT_REG_IDX_LEND, XT_REG_IDX_LCOUNT, XT_REG_IDX_SAR,
	XT_REG_IDX_WINDOWBASE, XT_REG_IDX_WINDOWSTART, XT_REG_IDX_CONFIGID0, XT_REG_IDX_CONFIGID1,
	XT_REG_IDX_PS, XT_REG_IDX_THREADPTR, XT_REG_IDX_BR, XT_REG_IDX_SCOMPARE1,
	XT_REG_IDX_ACCLO, XT_REG_IDX_ACCHI,
	XT_REG_IDX_M0, XT_REG_IDX_M1, XT_REG_IDX_M2, XT_REG_IDX_M3,
	ESP32_S3_REG_IDX_GPIOOUT,
	XT_REG_IDX_F0, XT_REG_IDX_F1, XT_REG_IDX_F2, XT_REG_IDX_F3,
	XT_REG_IDX_F4, XT_REG_IDX_F5, XT_REG_IDX_F6, XT_REG_IDX_F7,
	XT_REG_IDX_F8, XT_REG_IDX_F9, XT_REG_IDX_F10, XT_REG_IDX_F11,
	XT_REG_IDX_F12, XT_REG_IDX_F13, XT_REG_IDX_F14, XT_REG_IDX_F15,
	XT_REG_IDX_FCR, XT_REG_IDX_FSR,
	ESP32_S3_REG_IDX_ACCX_0, ESP32_S3_REG_IDX_ACCX_1,
	ESP32_S3_REG_IDX_QACC_H_0, ESP32_S3_REG_IDX_QACC_H_1, ESP32_S3_REG_IDX_QACC_H_2,
	ESP32_S3_REG_IDX_QACC_H_3, ESP32_S3_REG_IDX_QACC_H_4,
	ESP32_S3_REG_IDX_QACC_L_0, ESP32_S3_REG_IDX_QACC_L_1, ESP32_S3_REG_IDX_QACC_L_2,
	ESP32_S3_REG_IDX_QACC_L_3, ESP32_S3_REG_IDX_QACC_L_4,
	ESP32_S3_REG_IDX_SAR_BYTE, ESP32_S3_REG_IDX_FFT_BIT_WIDTH,
	ESP32_S3_REG_IDX_UA_STATE_0, ESP32_S3_REG_IDX_UA_STATE_1, ESP32_S3_REG_IDX_UA_STATE_2,
	ESP32_S3_REG_IDX_UA_STATE_3,
	ESP32_S3_REG_IDX_Q0, ESP32_S3_REG_IDX_Q1, ESP32_S3_REG_IDX_Q2, ESP32_S3_REG_IDX_Q3,
	ESP32_S3_REG_IDX_Q4, ESP32_S3_REG_IDX_Q5, ESP32_S3_REG_IDX_Q6, ESP32_S3_REG_IDX_Q7,

	XT_REG_IDX_MMID, XT_REG_IDX_IBREAKENABLE,
	XT_REG_IDX_MEMCTL, XT_REG_IDX_ATOMCTL, XT_REG_IDX_OCD_DDR,
	XT_REG_IDX_IBREAKA0, XT_REG_IDX_IBREAKA1, XT_REG_IDX_DBREAKA0, XT_REG_IDX_DBREAKA1,
	XT_REG_IDX_DBREAKC0, XT_REG_IDX_DBREAKC1,
	XT_REG_IDX_EPC1, XT_REG_IDX_EPC2, XT_REG_IDX_EPC3, XT_REG_IDX_EPC4,
	XT_REG_IDX_EPC5, XT_REG_IDX_EPC6, XT_REG_IDX_EPC7, XT_REG_IDX_DEPC,
	XT_REG_IDX_EPS2, XT_REG_IDX_EPS3, XT_REG_IDX_EPS4, XT_REG_IDX_EPS5,
	XT_REG_IDX_EPS6, XT_REG_IDX_EPS7,
	XT_REG_IDX_EXCSAVE1, XT_REG_IDX_EXCSAVE2, XT_REG_IDX_EXCSAVE3, XT_REG_IDX_EXCSAVE4,
	XT_REG_IDX_EXCSAVE5, XT_REG_IDX_EXCSAVE6, XT_REG_IDX_EXCSAVE7, XT_REG_IDX_CPENABLE,
	XT_REG_IDX_INTERRUPT, XT_REG_IDX_INTSET, XT_REG_IDX_INTCLEAR, XT_REG_IDX_INTENABLE,
	XT_REG_IDX_VECBASE, XT_REG_IDX_EXCCAUSE, XT_REG_IDX_DEBUGCAUSE, XT_REG_IDX_CCOUNT,
	XT_REG_IDX_PRID, XT_REG_IDX_ICOUNT, XT_REG_IDX_ICOUNTLEVEL, XT_REG_IDX_EXCVADDR,
	XT_REG_IDX_CCOMPARE0, XT_REG_IDX_CCOMPARE1, XT_REG_IDX_CCOMPARE2,
	XT_REG_IDX_MISC0, XT_REG_IDX_MISC1, XT_REG_IDX_MISC2, XT_REG_IDX_MISC3,

	XT_REG_IDX_PWRCTL, XT_REG_IDX_PWRSTAT, XT_REG_IDX_ERISTAT,
	XT_REG_IDX_CS_ITCTRL, XT_REG_IDX_CS_CLAIMSET, XT_REG_IDX_CS_CLAIMCLR,
	XT_REG_IDX_CS_LOCKACCESS, XT_REG_IDX_CS_LOCKSTATUS, XT_REG_IDX_CS_AUTHSTATUS,
	XT_REG_IDX_FAULT_INFO,
	XT_REG_IDX_TRAX_ID, XT_REG_IDX_TRAX_CTRL, XT_REG_IDX_TRAX_STAT,
	XT_REG_IDX_TRAX_DATA, XT_REG_IDX_TRAX_ADDR, XT_REG_IDX_TRAX_PCTRIGGER,
	XT_REG_IDX_TRAX_PCMATCH, XT_REG_IDX_TRAX_DELAY, XT_REG_IDX_TRAX_MEMSTART,
	XT_REG_IDX_TRAX_MEMEND,
	XT_REG_IDX_PMG, XT_REG_IDX_PMPC, XT_REG_IDX_PM0, XT_REG_IDX_PM1,
	XT_REG_IDX_PMCTRL0, XT_REG_IDX_PMCTRL1, XT_REG_IDX_PMSTAT0, XT_REG_IDX_PMSTAT1,
	XT_REG_IDX_OCD_ID, XT_REG_IDX_OCD_DCRCLR, XT_REG_IDX_OCD_DCRSET, XT_REG_IDX_OCD_DSR,
	XT_REG_IDX_A0, XT_REG_IDX_A1, XT_REG_IDX_A2, XT_REG_IDX_A3,
	XT_REG_IDX_A4, XT_REG_IDX_A5, XT_REG_IDX_A6, XT_REG_IDX_A7,
	XT_REG_IDX_A8, XT_REG_IDX_A9, XT_REG_IDX_A10, XT_REG_IDX_A11,
	XT_REG_IDX_A12, XT_REG_IDX_A13, XT_REG_IDX_A14, XT_REG_IDX_A15,
};

/* actually this table contains user + TIE registers
 * TODO: for TIE registers we need to specify custom access functions instead of `xtensa_user_reg_xxx_type`*/
static const struct xtensa_user_reg_desc esp32s3_user_regs[ESP32_S3_NUM_REGS - XT_NUM_REGS] = {
	{ "gpio_out", 0x00, 0, 32, &xtensa_user_reg_u32_type },
	{ "accx_0", 0x01, 0, 32, &xtensa_user_reg_u32_type },
	{ "accx_1", 0x02, 0, 32, &xtensa_user_reg_u32_type },
	{ "qacc_h_0", 0x03, 0, 32, &xtensa_user_reg_u32_type },
	{ "qacc_h_1", 0x04, 0, 32, &xtensa_user_reg_u32_type },
	{ "qacc_h_2", 0x05, 0, 32, &xtensa_user_reg_u32_type },
	{ "qacc_h_3", 0x06, 0, 32, &xtensa_user_reg_u32_type },
	{ "qacc_h_4", 0x07, 0, 32, &xtensa_user_reg_u32_type },
	{ "qacc_l_0", 0x08, 0, 32, &xtensa_user_reg_u32_type },
	{ "qacc_l_1", 0x09, 0, 32, &xtensa_user_reg_u32_type },
	{ "qacc_l_2", 0x0A, 0, 32, &xtensa_user_reg_u32_type },
	{ "qacc_l_3", 0x0B, 0, 32, &xtensa_user_reg_u32_type },
	{ "qacc_l_4", 0x0C, 0, 32, &xtensa_user_reg_u32_type },
	{ "sar_byte", 0x0D, 0, 32, &xtensa_user_reg_u32_type },
	{ "fft_bit_width", 0x0E, 0, 32, &xtensa_user_reg_u32_type },
	{ "ua_state_0", 0x0F, 0, 32, &xtensa_user_reg_u32_type },
	{ "ua_state_1", 0x10, 0, 32, &xtensa_user_reg_u32_type },
	{ "ua_state_2", 0x11, 0, 32, &xtensa_user_reg_u32_type },
	{ "ua_state_3", 0x12, 0, 32, &xtensa_user_reg_u32_type },
	{ "q0", 0x13, 0, 128, &xtensa_user_reg_u128_type },
	{ "q1", 0x14, 0, 128, &xtensa_user_reg_u128_type },
	{ "q2", 0x15, 0, 128, &xtensa_user_reg_u128_type },
	{ "q3", 0x16, 0, 128, &xtensa_user_reg_u128_type },
	{ "q4", 0x17, 0, 128, &xtensa_user_reg_u128_type },
	{ "q5", 0x18, 0, 128, &xtensa_user_reg_u128_type },
	{ "q6", 0x19, 0, 128, &xtensa_user_reg_u128_type },
	{ "q7", 0x20, 0, 128, &xtensa_user_reg_u128_type },
};

struct esp32s3_common {
	struct esp_xtensa_smp_common esp_xtensa_smp;
};

static int esp32s3_fetch_user_regs(struct target *target);
static int esp32s3_queue_write_dirty_user_regs(struct target *target);

static const struct xtensa_config esp32s3_xtensa_cfg = {
	.density = true,
	.aregs_num = XT_AREGS_NUM_MAX,
	.windowed = true,
	.coproc = true,
	.fp_coproc = true,
	.loop = true,
	.miscregs_num = 4,
	.threadptr = true,
	.boolean = true,
	.reloc_vec = true,
	.proc_id = true,
	.cond_store = true,
	.mac16 = true,
	.user_regs_num = ARRAY_SIZE(esp32s3_user_regs),
	.user_regs = esp32s3_user_regs,
	.fetch_user_regs = esp32s3_fetch_user_regs,
	.queue_write_dirty_user_regs = esp32s3_queue_write_dirty_user_regs,
	.gdb_general_regs_num = ESP32_S3_NUM_REGS_G_COMMAND,
	.gdb_regs_mapping = esp32s3_gdb_regs_mapping,
	.irom = {
		.count = 2,
		.regions = {
			{
				.base = ESP32_S3_IROM_LOW,
				.size = ESP32_S3_IROM_HIGH - ESP32_S3_IROM_LOW,
				.access = XT_MEM_ACCESS_READ,
			},
			{
				.base = ESP32_S3_IROM_MASK_LOW,
				.size = ESP32_S3_IROM_MASK_HIGH - ESP32_S3_IROM_MASK_LOW,
				.access = XT_MEM_ACCESS_READ,
			}
		}
	},
	.iram = {
		.count = 2,
		.regions = {
			{
				.base = ESP32_S3_IRAM_LOW,
				.size = ESP32_S3_IRAM_HIGH - ESP32_S3_IRAM_LOW,
				.access = XT_MEM_ACCESS_READ | XT_MEM_ACCESS_WRITE,
			},
			{
				.base = ESP32_S3_RTC_IRAM_LOW,
				.size = ESP32_S3_RTC_IRAM_HIGH - ESP32_S3_RTC_IRAM_LOW,
				.access = XT_MEM_ACCESS_READ | XT_MEM_ACCESS_WRITE,
			},
		}
	},
	.drom = {
		.count = 1,
		.regions = {
			{
				.base = ESP32_S3_DROM_LOW,
				.size = ESP32_S3_DROM_HIGH - ESP32_S3_DROM_LOW,
				.access = XT_MEM_ACCESS_READ,
			},
		}
	},
	.dram = {
		.count = 4,
		.regions = {
			{
				.base = ESP32_S3_DRAM_LOW,
				.size = ESP32_S3_DRAM_HIGH - ESP32_S3_DRAM_LOW,
				.access = XT_MEM_ACCESS_READ | XT_MEM_ACCESS_WRITE,
			},
			{
				.base = ESP32_S3_RTC_DRAM_LOW,
				.size = ESP32_S3_RTC_DRAM_HIGH - ESP32_S3_RTC_DRAM_LOW,
				.access = XT_MEM_ACCESS_READ | XT_MEM_ACCESS_WRITE,
			},
			{
				.base = ESP32_S3_RTC_DATA_LOW,
				.size = ESP32_S3_RTC_DATA_HIGH - ESP32_S3_RTC_DATA_LOW,
				.access = XT_MEM_ACCESS_READ | XT_MEM_ACCESS_WRITE,
			},
			{
				.base = ESP32_S3_SYS_RAM_LOW,
				.size = ESP32_S3_SYS_RAM_HIGH - ESP32_S3_SYS_RAM_LOW,
				.access = XT_MEM_ACCESS_READ | XT_MEM_ACCESS_WRITE,
			},
		}
	},
	.exc = {
		.enabled = true,
	},
	.irq = {
		.enabled = true,
		.irq_num = 32,
	},
	.high_irq = {
		.enabled = true,
		.excm_level = 3,
		.nmi_num = 1,
	},
	.tim_irq = {
		.enabled = true,
		.comp_num = 3,
	},
	.debug = {
		.enabled = true,
		.irq_level = 6,
		.ibreaks_num = 2,
		.dbreaks_num = 2,
		.icount_sz = 32,
	},
	.trace = {
		.enabled = true,
		.mem_sz = ESP32_S3_TRACEMEM_BLOCK_SZ,
	},
};

static int esp32s3_fetch_user_regs(struct target *target)
{
	LOG_DEBUG("%s: user regs fetching is not implemented!", target_name(target));
	return ERROR_OK;
}

static int esp32s3_queue_write_dirty_user_regs(struct target *target)
{
	LOG_DEBUG("%s: user regs writing is not implemented!", target_name(target));
	return ERROR_OK;
}

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

const uint8_t esp32s3_reset_stub_code[] = {
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
	res = xtensa_resume(target, 0, ESP32_S3_RTC_SLOW_MEM_BASE + 4, 0, 0);
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
	return esp32s3_disable_wdts(target);
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


static int esp32s3_target_init(struct command_context *cmd_ctx, struct target *target)
{
	return esp_xtensa_target_init(cmd_ctx, target);
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

static const struct esp_xtensa_smp_chip_ops esp32s3_chip_ops = {
	.reset = esp32s3_soc_reset,
	.on_halt = esp32s3_on_halt
};

static int esp32s3_target_create(struct target *target, Jim_Interp *interp)
{
	struct xtensa_debug_module_config esp32s3_dm_cfg = {
		.dbg_ops = &esp32s3_dbg_ops,
		.pwr_ops = &esp32s3_pwr_ops,
		.tap = target->tap,
		.queue_tdi_idle = NULL,
		.queue_tdi_idle_arg = NULL
	};

	struct esp32s3_common *esp32s3 = calloc(1, sizeof(struct esp32s3_common));
	if (!esp32s3) {
		LOG_ERROR("Failed to alloc memory for arch info!");
		return ERROR_FAIL;
	}

	int ret = esp_xtensa_smp_init_arch_info(target,
		&esp32s3->esp_xtensa_smp,
		&esp32s3_xtensa_cfg,
		&esp32s3_dm_cfg,
		&esp32s3_chip_ops);
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
		.name = "esp32",
		.usage = "",
		.chain = smp_command_handlers,
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

	.add_breakpoint = esp_xtensa_breakpoint_add,
	.remove_breakpoint = esp_xtensa_breakpoint_remove,

	.add_watchpoint = esp_xtensa_smp_watchpoint_add,
	.remove_watchpoint = esp_xtensa_smp_watchpoint_remove,

	.target_create = esp32s3_target_create,
	.init_target = esp32s3_target_init,
	.examine = xtensa_examine,
	.deinit_target = esp_xtensa_target_deinit,

	.commands = esp32s3_command_handlers,
};
