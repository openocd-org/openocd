/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Reset stub used by esp32s3 target                                     *
 *   Copyright (C) 2020 Espressif Systems (Shanghai) Co. Ltd.              *
 ***************************************************************************/

#define RTC_CNTL_RESET_STATE_REG            0x60008038
#define RTC_CNTL_RESET_STATE_DEF            0x3000
#define RTC_CNTL_CLK_CONF_REG               0x60008074
#define RTC_CNTL_CLK_CONF_DEF               0x1583218
#define RTC_CNTL_STORE4_REG                 0x600080C0
#define RTC_CNTL_STORE5_REG                 0x600080C4
#define WDT_WKEY_VALUE                      0x50D83AA1
#define TIMG0_WDTWPROTECT_REG               0x6001F064
#define TIMG0_WDTCONFIG0_REG                0x6001F048
#define TIMG1_WDTWPROTECT_REG               0x60020064
#define TIMG1_WDTCONFIG0_REG                0x60020048
#define RTC_CNTL_WDTCONFIG0_REG             0x60008094
#define RTC_CNTL_WDTWPROTECT_REG            0x600080AC
#define RTC_CNTL_OPTIONS0_REG               0x60008000
#define RTC_CNTL_OPTIONS0_DEF               0x1C00A000
#define RTC_CNTL_SW_SYS_RST                 0x80000000
#define RTC_CNTL_DIG_PWC_REG                0x60008090
#define RTC_CNTL_SWD_CONF_REG               0x600080B0
#define RTC_CNTL_SWD_CONF_VAL               0x84B00000
#define RTC_CNTL_SWD_WPROTECT_REG           0x600080B4
#define RTC_CNTL_SWD_WKEY_VALUE             0x8F1D312A
#define SYSTEM_CORE_1_CONTROL_0_REG         0x600C0000
#define SYSTEM_CONTROL_CORE_1_RESETING      0x4
#define SYSTEM_CONTROL_CORE_1_CLKGATE_EN    0x2
#define SYSTEM_CORE_1_CONTROL_1_REG         0x600C0004


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
    movi a4, RTC_CNTL_DIG_PWC_REG
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
    movi a4, RTC_CNTL_SWD_WPROTECT_REG
    movi a3, RTC_CNTL_SWD_WKEY_VALUE
    s32i.n a3, a4, 0
    movi a4, RTC_CNTL_SWD_CONF_REG
    movi a3, RTC_CNTL_SWD_CONF_VAL
    s32i.n a3, a4, 0
    /* Clear APP_CPU boot address */
    movi a4, SYSTEM_CORE_1_CONTROL_1_REG
    s32i.n a5, a4, 0
    /* Clear APP_CPU clock gating */
    movi a4, SYSTEM_CORE_1_CONTROL_0_REG
    movi a3, SYSTEM_CONTROL_CORE_1_CLKGATE_EN
    s32i.n a3, a4, 0
    /* Set and clear APP_CPU reset */
    movi a4, SYSTEM_CORE_1_CONTROL_0_REG
    movi a3, SYSTEM_CONTROL_CORE_1_RESETING
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
