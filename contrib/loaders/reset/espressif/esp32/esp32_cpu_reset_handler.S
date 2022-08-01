/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Reset stub used by esp32 target                                       *
 *   Copyright (C) 2017 Espressif Systems Ltd.                             *
 ***************************************************************************/

#define RTC_CNTL_RESET_STATE_REG    0x3ff48034
#define RTC_CNTL_RESET_STATE_DEF    0x3000
#define RTC_CNTL_CLK_CONF_REG       0x3ff48070
#define RTC_CNTL_CLK_CONF_DEF       0x2210
#define RTC_CNTL_STORE4_REG         0x3ff480b0
#define RTC_CNTL_STORE5_REG         0x3ff480b4
#define WDT_WKEY_VALUE              0x50D83AA1
#define TIMG0_WDTWPROTECT_REG       0x3ff5f064
#define TIMG0_WDTCONFIG0_REG        0x3ff5f048
#define TIMG1_WDTWPROTECT_REG       0x3FF60064
#define TIMG1_WDTCONFIG0_REG        0x3ff60048
#define RTC_CNTL_WDTCONFIG0_REG     0x3ff4808c
#define RTC_CNTL_WDTWPROTECT_REG    0x3ff480a4
#define JTAG_ENABLE_REG             0x3ff5a1fc
#define RTC_CNTL_OPTIONS0_REG       0x3ff48000
#define RTC_CNTL_OPTIONS0_DEF       0x1c492000
#define RTC_CNTL_SW_SYS_RST         0x80000000
#define DPORT_APPCPU_CTRL_A_REG     0x3ff0002c
#define DPORT_APPCPU_RST_EN         0x1
#define DPORT_APPCPU_CTRL_B_REG     0x3ff00030
#define DPORT_APPCPU_CLKGATE_EN     0x1
#define DPORT_APPCPU_CTRL_C_REG     0x3ff00034
#define DPORT_APPCPU_CTRL_D_REG     0x3ff00038


/* This stub is copied to RTC_SLOW_MEM by OpenOCD, and the CPU starts executing
 * it instead of the ROM code (0x40000400). This stub disables watchdogs and
 * goes into a loop.
 * OpenOCD will then halt the target and perform CPU reset using OCD.
 */


/* Has to be at offset 0. This is the entry point of the CPU, once
 * RTC_CNTL_PROCPU_STAT_VECTOR_SEL is cleared.
 * CPU will come here after the system reset, triggered by RTC_CNTL_SW_SYS_RST.
 */
    .global     cpu_at_start_handler
    .type       cpu_at_start_handler,@function
    .align      4
cpu_at_start_handler:
    j start


/* Has to be at offset 4. Once the stub code has been uploaded into RTC Slow
 * memory, OpenOCD will set the PC to this address, and resume execution.
 * The stub will then jump to 'reset' label and perform the reset.
 */
    .global     cpu_reset_handler
    .type       cpu_reset_handler,@function
    .align      4
cpu_reset_handler:
    j reset

    .align 4
    .literal_position

    .align 4
reset:
    /* Use a5 as a zero register */
    xor a5, a5, a5
    /* Select static reset vector 0 (XCHAL_RESET_VECTOR0_VADDR, 0x50000000) */
    movi a4, RTC_CNTL_RESET_STATE_REG
    s32i a5, a4, 0
    /* Set some clock-related RTC registers to the default values */
    movi a4, RTC_CNTL_STORE4_REG
    s32i a5, a4, 0
    movi a4, RTC_CNTL_STORE5_REG
    s32i a5, a4, 0
    movi a4, RTC_CNTL_CLK_CONF_REG
    movi a3, RTC_CNTL_CLK_CONF_DEF
    s32i a3, a4, 0
    /* Reset the digital part of the chip (RTC controller doesn't get reset) */
    movi a3, (RTC_CNTL_OPTIONS0_DEF | RTC_CNTL_SW_SYS_RST)
    movi a4, RTC_CNTL_OPTIONS0_REG
    s32i a3, a4, 0
    /* Doesn't reach beyond this instruction */

    .align 4
start:
    /* If running on the APP CPU, skip directly to the parking loop */
    rsr.prid a6
    extui a6, a6, 1, 1
    bnez a6, parking_loop

    /* Use a5 as a zero register */
    xor a5, a5, a5
    /* Disable the watchdogs */
    movi a3, WDT_WKEY_VALUE
    movi a4, RTC_CNTL_WDTWPROTECT_REG
    s32i.n a3, a4, 0
    movi a4, TIMG0_WDTWPROTECT_REG
    s32i.n a3, a4, 0
    movi a4, TIMG1_WDTWPROTECT_REG
    s32i.n a3, a4, 0
    movi a4, RTC_CNTL_WDTCONFIG0_REG
    s32i.n a5, a4, 0
    movi a4, TIMG0_WDTCONFIG0_REG
    s32i.n a5, a4, 0
    movi a4, TIMG1_WDTCONFIG0_REG
    s32i.n a5, a4, 0
    /* Enable JTAG (needed since rev. 3) */
    movi a4, JTAG_ENABLE_REG
    s32i.n a5, a4, 0
    /* Clear APP_CPU boot address */
    movi a4, DPORT_APPCPU_CTRL_D_REG
    s32i.n a5, a4, 0
    /* Clear APP_CPU clock gating */
    movi a4, DPORT_APPCPU_CTRL_B_REG
    movi a3, DPORT_APPCPU_CLKGATE_EN
    s32i.n a3, a4, 0
    /* Set and clear APP_CPU reset */
    movi a4, DPORT_APPCPU_CTRL_A_REG
    movi a3, DPORT_APPCPU_RST_EN
    s32i.n a3, a4, 0
    s32i.n a5, a4, 0
    /* Restore the reset vector to ROM */
    movi a4, RTC_CNTL_RESET_STATE_REG
    movi a3, RTC_CNTL_RESET_STATE_DEF
    s32i.n a3, a4, 0


parking_loop:
    /* PRO and APP CPU will be in this loop, until OpenOCD
     * finds the JTAG taps and puts the CPUs into debug mode.
     */
    waiti 0
    j parking_loop
