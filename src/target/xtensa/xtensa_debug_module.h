/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Xtensa Debug Module (XDM) Support for OpenOCD                         *
 *   Copyright (C) 2020-2022 Cadence Design Systems, Inc.                  *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
 *   Derived from original ESP8266 target.                                 *
 *   Author: Angus Gratton gus@projectgus.com                              *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_XTENSA_DEBUG_MODULE_H
#define OPENOCD_TARGET_XTENSA_DEBUG_MODULE_H

#include <jtag/jtag.h>
#include <target/arm_adi_v5.h>
#include <helper/bits.h>
#include <target/target.h>

/* Virtual IDs for using with xtensa_power_ops API */
enum xtensa_dm_pwr_reg {
	XDMREG_PWRCTL = 0x00,
	XDMREG_PWRSTAT,
	XDMREG_PWRNUM
};

/* Debug Module Power Register offsets within APB */
struct xtensa_dm_pwr_reg_offsets {
	uint16_t apb;
};

/* Debug Module Power Register offset structure; must include XDMREG_PWRNUM entries */
#define XTENSA_DM_PWR_REG_OFFSETS	{				\
	/* Power/Reset Registers */						\
	{ .apb = 0x3020 },		/* XDMREG_PWRCTL */		\
	{ .apb = 0x3024 },		/* XDMREG_PWRSTAT */	\
}

/*
 From the manual:
 To properly use Debug registers through JTAG, software must ensure that:
 - Tap is out of reset
 - Xtensa Debug Module is out of reset
 - Other bits of PWRCTL are set to their desired values, and finally
 - JtagDebugUse transitions from 0 to 1
 The bit must continue to be 1 in order for JTAG accesses to the Debug
 Module to happen correctly. When it is set, any write to this bit clears it.
 Either don't access it, or re-write it to 1 so JTAG accesses continue.
*/
#define PWRCTL_JTAGDEBUGUSE(x)		(((x)->dbg_mod.dap) ? (0)     : BIT(7))
#define PWRCTL_DEBUGRESET(x)		(((x)->dbg_mod.dap) ? BIT(28) : BIT(6))
#define PWRCTL_CORERESET(x)			(((x)->dbg_mod.dap) ? BIT(16) : BIT(4))
#define PWRCTL_DEBUGWAKEUP(x)		(((x)->dbg_mod.dap) ? BIT(12) : BIT(2))
#define PWRCTL_MEMWAKEUP(x)			(((x)->dbg_mod.dap) ? BIT(8)  : BIT(1))
#define PWRCTL_COREWAKEUP(x)		(((x)->dbg_mod.dap) ? BIT(0)  : BIT(0))

#define PWRSTAT_DEBUGWASRESET_DM(d)	(((d)->dap) ? BIT(28) : BIT(6))
#define PWRSTAT_COREWASRESET_DM(d)	(((d)->dap) ? BIT(16) : BIT(4))
#define PWRSTAT_DEBUGWASRESET(x)	(PWRSTAT_DEBUGWASRESET_DM(&((x)->dbg_mod)))
#define PWRSTAT_COREWASRESET(x)		(PWRSTAT_COREWASRESET_DM(&((x)->dbg_mod)))
#define PWRSTAT_CORESTILLNEEDED(x)	(((x)->dbg_mod.dap) ? BIT(4)  : BIT(3))
#define PWRSTAT_DEBUGDOMAINON(x)	(((x)->dbg_mod.dap) ? BIT(12) : BIT(2))
#define PWRSTAT_MEMDOMAINON(x)		(((x)->dbg_mod.dap) ? BIT(8)  : BIT(1))
#define PWRSTAT_COREDOMAINON(x)		(((x)->dbg_mod.dap) ? BIT(0)  : BIT(0))

/* Virtual IDs for using with xtensa_debug_ops API */
enum xtensa_dm_reg {
	/* TRAX Registers */
	XDMREG_TRAXID = 0x00,
	XDMREG_TRAXCTRL,
	XDMREG_TRAXSTAT,
	XDMREG_TRAXDATA,
	XDMREG_TRAXADDR,
	XDMREG_TRIGGERPC,
	XDMREG_PCMATCHCTRL,
	XDMREG_DELAYCNT,
	XDMREG_MEMADDRSTART,
	XDMREG_MEMADDREND,

	/* Performance Monitor Registers */
	XDMREG_PMG,
	XDMREG_INTPC,
	XDMREG_PM0,
	XDMREG_PM1,
	XDMREG_PM2,
	XDMREG_PM3,
	XDMREG_PM4,
	XDMREG_PM5,
	XDMREG_PM6,
	XDMREG_PM7,
	XDMREG_PMCTRL0,
	XDMREG_PMCTRL1,
	XDMREG_PMCTRL2,
	XDMREG_PMCTRL3,
	XDMREG_PMCTRL4,
	XDMREG_PMCTRL5,
	XDMREG_PMCTRL6,
	XDMREG_PMCTRL7,
	XDMREG_PMSTAT0,
	XDMREG_PMSTAT1,
	XDMREG_PMSTAT2,
	XDMREG_PMSTAT3,
	XDMREG_PMSTAT4,
	XDMREG_PMSTAT5,
	XDMREG_PMSTAT6,
	XDMREG_PMSTAT7,

	/* OCD Registers */
	XDMREG_OCDID,
	XDMREG_DCRCLR,
	XDMREG_DCRSET,
	XDMREG_DSR,
	XDMREG_DDR,
	XDMREG_DDREXEC,
	XDMREG_DIR0EXEC,
	XDMREG_DIR0,
	XDMREG_DIR1,
	XDMREG_DIR2,
	XDMREG_DIR3,
	XDMREG_DIR4,
	XDMREG_DIR5,
	XDMREG_DIR6,
	XDMREG_DIR7,

	/* Misc Registers */
	XDMREG_ERISTAT,

	/* CoreSight Registers */
	XDMREG_ITCTRL,
	XDMREG_CLAIMSET,
	XDMREG_CLAIMCLR,
	XDMREG_LOCKACCESS,
	XDMREG_LOCKSTATUS,
	XDMREG_AUTHSTATUS,
	XDMREG_DEVID,
	XDMREG_DEVTYPE,
	XDMREG_PERID4,
	XDMREG_PERID5,
	XDMREG_PERID6,
	XDMREG_PERID7,
	XDMREG_PERID0,
	XDMREG_PERID1,
	XDMREG_PERID2,
	XDMREG_PERID3,
	XDMREG_COMPID0,
	XDMREG_COMPID1,
	XDMREG_COMPID2,
	XDMREG_COMPID3,

	XDMREG_NUM
};

/* Debug Module Register offsets within Nexus (NAR) or APB */
struct xtensa_dm_reg_offsets {
	uint8_t  nar;
	uint16_t apb;
};

/* Debug Module Register offset structure; must include XDMREG_NUM entries */
#define XTENSA_DM_REG_OFFSETS	{								\
	/* TRAX Registers */										\
	{ .nar = 0x00, .apb = 0x0000 },	/* XDMREG_TRAXID */			\
	{ .nar = 0x01, .apb = 0x0004 },	/* XDMREG_TRAXCTRL */		\
	{ .nar = 0x02, .apb = 0x0008 },	/* XDMREG_TRAXSTAT */		\
	{ .nar = 0x03, .apb = 0x000c },	/* XDMREG_TRAXDATA */		\
	{ .nar = 0x04, .apb = 0x0010 },	/* XDMREG_TRAXADDR */		\
	{ .nar = 0x05, .apb = 0x0014 },	/* XDMREG_TRIGGERPC */		\
	{ .nar = 0x06, .apb = 0x0018 },	/* XDMREG_PCMATCHCTRL */	\
	{ .nar = 0x07, .apb = 0x001c },	/* XDMREG_DELAYCNT */		\
	{ .nar = 0x08, .apb = 0x0020 },	/* XDMREG_MEMADDRSTART */	\
	{ .nar = 0x09, .apb = 0x0024 },	/* XDMREG_MEMADDREND */		\
																\
	/* Performance Monitor Registers */							\
	{ .nar = 0x20, .apb = 0x1000 },	/* XDMREG_PMG */			\
	{ .nar = 0x24, .apb = 0x1010 },	/* XDMREG_INTPC */			\
	{ .nar = 0x28, .apb = 0x1080 },	/* XDMREG_PM0 */			\
	{ .nar = 0x29, .apb = 0x1084 },	/* XDMREG_PM1 */			\
	{ .nar = 0x2a, .apb = 0x1088 },	/* XDMREG_PM2 */			\
	{ .nar = 0x2b, .apb = 0x108c },	/* XDMREG_PM3 */			\
	{ .nar = 0x2c, .apb = 0x1090 },	/* XDMREG_PM4 */			\
	{ .nar = 0x2d, .apb = 0x1094 },	/* XDMREG_PM5 */			\
	{ .nar = 0x2e, .apb = 0x1098 },	/* XDMREG_PM6 */			\
	{ .nar = 0x2f, .apb = 0x109c },	/* XDMREG_PM7 */			\
	{ .nar = 0x30, .apb = 0x1100 },	/* XDMREG_PMCTRL0 */		\
	{ .nar = 0x31, .apb = 0x1104 },	/* XDMREG_PMCTRL1 */		\
	{ .nar = 0x32, .apb = 0x1108 },	/* XDMREG_PMCTRL2 */		\
	{ .nar = 0x33, .apb = 0x110c },	/* XDMREG_PMCTRL3 */		\
	{ .nar = 0x34, .apb = 0x1110 },	/* XDMREG_PMCTRL4 */		\
	{ .nar = 0x35, .apb = 0x1114 },	/* XDMREG_PMCTRL5 */		\
	{ .nar = 0x36, .apb = 0x1118 },	/* XDMREG_PMCTRL6 */		\
	{ .nar = 0x37, .apb = 0x111c },	/* XDMREG_PMCTRL7 */		\
	{ .nar = 0x38, .apb = 0x1180 },	/* XDMREG_PMSTAT0 */		\
	{ .nar = 0x39, .apb = 0x1184 },	/* XDMREG_PMSTAT1 */		\
	{ .nar = 0x3a, .apb = 0x1188 },	/* XDMREG_PMSTAT2 */		\
	{ .nar = 0x3b, .apb = 0x118c },	/* XDMREG_PMSTAT3 */		\
	{ .nar = 0x3c, .apb = 0x1190 },	/* XDMREG_PMSTAT4 */		\
	{ .nar = 0x3d, .apb = 0x1194 },	/* XDMREG_PMSTAT5 */		\
	{ .nar = 0x3e, .apb = 0x1198 },	/* XDMREG_PMSTAT6 */		\
	{ .nar = 0x3f, .apb = 0x119c },	/* XDMREG_PMSTAT7 */		\
																\
	/* OCD Registers */											\
	{ .nar = 0x40, .apb = 0x2000 },	/* XDMREG_OCDID */			\
	{ .nar = 0x42, .apb = 0x2008 },	/* XDMREG_DCRCLR */			\
	{ .nar = 0x43, .apb = 0x200c },	/* XDMREG_DCRSET */			\
	{ .nar = 0x44, .apb = 0x2010 },	/* XDMREG_DSR */			\
	{ .nar = 0x45, .apb = 0x2014 },	/* XDMREG_DDR */			\
	{ .nar = 0x46, .apb = 0x2018 },	/* XDMREG_DDREXEC */		\
	{ .nar = 0x47, .apb = 0x201c },	/* XDMREG_DIR0EXEC */		\
	{ .nar = 0x48, .apb = 0x2020 },	/* XDMREG_DIR0 */			\
	{ .nar = 0x49, .apb = 0x2024 },	/* XDMREG_DIR1 */			\
	{ .nar = 0x4a, .apb = 0x2028 },	/* XDMREG_DIR2 */			\
	{ .nar = 0x4b, .apb = 0x202c },	/* XDMREG_DIR3 */			\
	{ .nar = 0x4c, .apb = 0x2030 },	/* XDMREG_DIR4 */			\
	{ .nar = 0x4d, .apb = 0x2034 },	/* XDMREG_DIR5 */			\
	{ .nar = 0x4e, .apb = 0x2038 },	/* XDMREG_DIR6 */			\
	{ .nar = 0x4f, .apb = 0x203c },	/* XDMREG_DIR7 */			\
																\
	/* Misc Registers */										\
	{ .nar = 0x5a, .apb = 0x3028 },	/* XDMREG_ERISTAT */		\
																\
	/* CoreSight Registers */									\
	{ .nar = 0x60, .apb = 0x3f00 },	/* XDMREG_ITCTRL */			\
	{ .nar = 0x68, .apb = 0x3fa0 },	/* XDMREG_CLAIMSET */		\
	{ .nar = 0x69, .apb = 0x3fa4 },	/* XDMREG_CLAIMCLR */		\
	{ .nar = 0x6c, .apb = 0x3fb0 },	/* XDMREG_LOCKACCESS */		\
	{ .nar = 0x6d, .apb = 0x3fb4 },	/* XDMREG_LOCKSTATUS */		\
	{ .nar = 0x6e, .apb = 0x3fb8 },	/* XDMREG_AUTHSTATUS */		\
	{ .nar = 0x72, .apb = 0x3fc8 },	/* XDMREG_DEVID */			\
	{ .nar = 0x73, .apb = 0x3fcc },	/* XDMREG_DEVTYPE */		\
	{ .nar = 0x74, .apb = 0x3fd0 },	/* XDMREG_PERID4 */			\
	{ .nar = 0x75, .apb = 0x3fd4 },	/* XDMREG_PERID5 */			\
	{ .nar = 0x76, .apb = 0x3fd8 },	/* XDMREG_PERID6 */			\
	{ .nar = 0x77, .apb = 0x3fdc },	/* XDMREG_PERID7 */			\
	{ .nar = 0x78, .apb = 0x3fe0 },	/* XDMREG_PERID0 */			\
	{ .nar = 0x79, .apb = 0x3fe4 },	/* XDMREG_PERID1 */			\
	{ .nar = 0x7a, .apb = 0x3fe8 },	/* XDMREG_PERID2 */			\
	{ .nar = 0x7b, .apb = 0x3fec },	/* XDMREG_PERID3 */			\
	{ .nar = 0x7c, .apb = 0x3ff0 },	/* XDMREG_COMPID0 */		\
	{ .nar = 0x7d, .apb = 0x3ff4 },	/* XDMREG_COMPID1 */		\
	{ .nar = 0x7e, .apb = 0x3ff8 },	/* XDMREG_COMPID2 */		\
	{ .nar = 0x7f, .apb = 0x3ffc },	/* XDMREG_COMPID3 */		\
}

#define XTENSA_DM_APB_ALIGN         0x4000

/* OCD registers, bit definitions */
#define OCDDCR_ENABLEOCD            BIT(0)
#define OCDDCR_DEBUGINTERRUPT       BIT(1)
#define OCDDCR_INTERRUPTALLCONDS    BIT(2)
#define OCDDCR_BREAKINEN            BIT(16)
#define OCDDCR_BREAKOUTEN           BIT(17)
#define OCDDCR_DEBUGSWACTIVE        BIT(20)
#define OCDDCR_RUNSTALLINEN         BIT(21)
#define OCDDCR_DEBUGMODEOUTEN       BIT(22)
#define OCDDCR_BREAKOUTITO          BIT(24)
#define OCDDCR_BREAKACKITO          BIT(25)

#define OCDDSR_EXECDONE             BIT(0)
#define OCDDSR_EXECEXCEPTION        BIT(1)
#define OCDDSR_EXECBUSY             BIT(2)
#define OCDDSR_EXECOVERRUN          BIT(3)
#define OCDDSR_STOPPED              BIT(4)
#define OCDDSR_COREWROTEDDR         BIT(10)
#define OCDDSR_COREREADDDR          BIT(11)
#define OCDDSR_HOSTWROTEDDR         BIT(14)
#define OCDDSR_HOSTREADDDR          BIT(15)
#define OCDDSR_DEBUGPENDBREAK       BIT(16)
#define OCDDSR_DEBUGPENDHOST        BIT(17)
#define OCDDSR_DEBUGPENDTRAX        BIT(18)
#define OCDDSR_DEBUGINTBREAK        BIT(20)
#define OCDDSR_DEBUGINTHOST         BIT(21)
#define OCDDSR_DEBUGINTTRAX         BIT(22)
#define OCDDSR_RUNSTALLTOGGLE       BIT(23)
#define OCDDSR_RUNSTALLSAMPLE       BIT(24)
#define OCDDSR_BREACKOUTACKITI      BIT(25)
#define OCDDSR_BREAKINITI           BIT(26)
#define OCDDSR_DBGMODPOWERON        BIT(31)

#define DEBUGCAUSE_IC               BIT(0)	/* ICOUNT exception */
#define DEBUGCAUSE_IB               BIT(1)	/* IBREAK exception */
#define DEBUGCAUSE_DB               BIT(2)	/* DBREAK exception */
#define DEBUGCAUSE_BI               BIT(3)	/* BREAK instruction encountered */
#define DEBUGCAUSE_BN               BIT(4)	/* BREAK.N instruction encountered */
#define DEBUGCAUSE_DI               BIT(5)	/* Debug Interrupt */

#define TRAXCTRL_TREN               BIT(0)	/* Trace enable. Tracing starts on 0->1 */
#define TRAXCTRL_TRSTP              BIT(1)	/* Trace Stop. Make 1 to stop trace. */
#define TRAXCTRL_PCMEN              BIT(2)	/* PC match enable */
#define TRAXCTRL_PTIEN              BIT(4)	/* Processor-trigger enable */
#define TRAXCTRL_CTIEN              BIT(5)	/* Cross-trigger enable */
#define TRAXCTRL_TMEN               BIT(7)	/* Tracemem Enable. Always set. */
#define TRAXCTRL_CNTU               BIT(9)	/* Post-stop-trigger countdown units; selects when DelayCount-- happens.
											 * 0 - every 32-bit word written to tracemem, 1 - every cpu instruction */
#define TRAXCTRL_TSEN               BIT(11)	/* Undocumented/deprecated? */
#define TRAXCTRL_SMPER_SHIFT        12		/* Send sync every 2^(9-smper) messages. 7=reserved, 0=no sync msg */
#define TRAXCTRL_SMPER_MASK         0x07	/* Synchronization message period */
#define TRAXCTRL_PTOWT              BIT(16)	/* Processor Trigger Out (OCD halt) enabled when stop triggered */
#define TRAXCTRL_PTOWS              BIT(17)	/* Processor Trigger Out (OCD halt) enabled when trace stop completes */
#define TRAXCTRL_CTOWT              BIT(20)	/* Cross-trigger Out enabled when stop triggered */
#define TRAXCTRL_CTOWS              BIT(21)	/* Cross-trigger Out enabled when trace stop completes */
#define TRAXCTRL_ITCTO              BIT(22)	/* Integration mode: cross-trigger output */
#define TRAXCTRL_ITCTIA             BIT(23)	/* Integration mode: cross-trigger ack */
#define TRAXCTRL_ITATV              BIT(24)	/* replaces ATID when in integration mode: ATVALID output */
#define TRAXCTRL_ATID_MASK          0x7F	/* ARB source ID */
#define TRAXCTRL_ATID_SHIFT         24
#define TRAXCTRL_ATEN               BIT(31)	/* ATB interface enable */

#define TRAXSTAT_TRACT              BIT(0)	/* Trace active flag. */
#define TRAXSTAT_TRIG               BIT(1)	/* Trace stop trigger. Clears on TREN 1->0 */
#define TRAXSTAT_PCMTG              BIT(2)	/* Stop trigger caused by PC match. Clears on TREN 1->0 */
#define TRAXSTAT_PJTR               BIT(3)	/* JTAG transaction result. 1=err in preceding jtag transaction. */
#define TRAXSTAT_PTITG              BIT(4)	/* Stop trigger caused by Processor Trigger Input.Clears on TREN 1->0 */
#define TRAXSTAT_CTITG              BIT(5)	/* Stop trigger caused by Cross-Trigger Input. Clears on TREN 1->0 */
#define TRAXSTAT_MEMSZ_SHIFT        8		/* Traceram size inducator. Usable trace ram is 2^MEMSZ bytes. */
#define TRAXSTAT_MEMSZ_MASK         0x1F
#define TRAXSTAT_PTO                BIT(16)	/* Processor Trigger Output: current value */
#define TRAXSTAT_CTO                BIT(17)	/* Cross-Trigger Output: current value */
#define TRAXSTAT_ITCTOA             BIT(22)	/* Cross-Trigger Out Ack: current value */
#define TRAXSTAT_ITCTI              BIT(23)	/* Cross-Trigger Input: current value */
#define TRAXSTAT_ITATR              BIT(24)	/* ATREADY Input: current value */

#define TRAXADDR_TADDR_SHIFT        0		/* Trax memory address, in 32-bit words. */
#define TRAXADDR_TADDR_MASK         0x1FFFFF	/* Actually is only as big as the trace buffer size max addr. */
#define TRAXADDR_TWRAP_SHIFT        21		/* Amount of times TADDR has overflown */
#define TRAXADDR_TWRAP_MASK         0x3FF
#define TRAXADDR_TWSAT              BIT(31)	/* 1 if TWRAP has overflown, clear by disabling tren.*/

#define PCMATCHCTRL_PCML_SHIFT      0		/* Amount of lower bits to ignore in pc trigger register */
#define PCMATCHCTRL_PCML_MASK       0x1F
#define PCMATCHCTRL_PCMS            BIT(31)	/* PC Match Sense, 0-match when procs PC is in-range, 1-match when
											 * out-of-range */

#define XTENSA_MAX_PERF_COUNTERS    2
#define XTENSA_MAX_PERF_SELECT      32
#define XTENSA_MAX_PERF_MASK        0xffff

#define XTENSA_STOPMASK_DISABLED    UINT32_MAX

struct xtensa_debug_module;

struct xtensa_debug_ops {
	/** enable operation */
	int (*queue_enable)(struct xtensa_debug_module *dm);
	/** register read. */
	int (*queue_reg_read)(struct xtensa_debug_module *dm, enum xtensa_dm_reg reg, uint8_t *data);
	/** register write. */
	int (*queue_reg_write)(struct xtensa_debug_module *dm, enum xtensa_dm_reg reg, uint32_t data);
};

/* Xtensa power registers are 8 bits wide on JTAG interfaces but 32 bits wide
 * when accessed via APB/DAP.  In order to use DAP queuing APIs (for optimal
 * performance), the XDM power register APIs take 32-bit register params.
 */
struct xtensa_power_ops {
	/** register read. */
	int (*queue_reg_read)(struct xtensa_debug_module *dm, enum xtensa_dm_pwr_reg reg, uint8_t *data,
		uint32_t clear);
	/** register write. */
	int (*queue_reg_write)(struct xtensa_debug_module *dm, enum xtensa_dm_pwr_reg reg, uint32_t data);
};

typedef uint32_t xtensa_pwrstat_t;
typedef uint32_t xtensa_ocdid_t;
typedef uint32_t xtensa_dsr_t;
typedef uint32_t xtensa_traxstat_t;

struct xtensa_power_status {
	xtensa_pwrstat_t stat;
	xtensa_pwrstat_t stath;
	/* TODO: do not need to keep previous status to detect that core or debug module has been
	 * reset, */
	/*       we can clear PWRSTAT_DEBUGWASRESET and PWRSTAT_COREWASRESET after reading will do
	 * the job; */
	/*       upon next reet those bits will be set again. So we can get rid of
	 *       xtensa_dm_power_status_cache_reset() and xtensa_dm_power_status_cache(). */
	xtensa_pwrstat_t prev_stat;
};

struct xtensa_core_status {
	xtensa_dsr_t dsr;
};

struct xtensa_trace_config {
	uint32_t ctrl;
	uint32_t memaddr_start;
	uint32_t memaddr_end;
	uint32_t addr;
};

struct xtensa_trace_status {
	xtensa_traxstat_t stat;
};

struct xtensa_trace_start_config {
	uint32_t stoppc;
	bool after_is_words;
	uint32_t after;
	uint32_t stopmask;	/* UINT32_MAX: disable PC match option */
};

struct xtensa_perfmon_config {
	int select;
	uint32_t mask;
	int kernelcnt;
	int tracelevel;
};

struct xtensa_perfmon_result {
	uint64_t value;
	bool overflow;
};

struct xtensa_debug_module_config {
	const struct xtensa_power_ops *pwr_ops;
	const struct xtensa_debug_ops *dbg_ops;

	/* Either JTAG or DAP structures will be populated */
	struct jtag_tap *tap;
	void (*queue_tdi_idle)(struct target *target);
	void *queue_tdi_idle_arg;

	/* For targets conforming to ARM Debug Interface v5,
	 * "dap" references the Debug Access Port (DAP)
	 * used to make requests to the target;
	 * "debug_ap" is AP instance connected to processor
	 */
	struct adiv5_dap *dap;
	struct adiv5_ap *debug_ap;
	int debug_apsel;
	uint32_t ap_offset;
};

struct xtensa_debug_module {
	const struct xtensa_power_ops *pwr_ops;
	const struct xtensa_debug_ops *dbg_ops;

	/* Either JTAG or DAP structures will be populated */
	struct jtag_tap *tap;
	void (*queue_tdi_idle)(struct target *target);
	void *queue_tdi_idle_arg;

	/* DAP struct; AP instance connected to processor */
	struct adiv5_dap *dap;
	struct adiv5_ap *debug_ap;
	int debug_apsel;

	struct xtensa_power_status power_status;
	struct xtensa_core_status core_status;
	xtensa_ocdid_t device_id;
	uint32_t ap_offset;
};

int xtensa_dm_init(struct xtensa_debug_module *dm, const struct xtensa_debug_module_config *cfg);
void xtensa_dm_deinit(struct xtensa_debug_module *dm);
int xtensa_dm_poll(struct xtensa_debug_module *dm);
int xtensa_dm_examine(struct xtensa_debug_module *dm);
int xtensa_dm_queue_enable(struct xtensa_debug_module *dm);
int xtensa_dm_queue_reg_read(struct xtensa_debug_module *dm, enum xtensa_dm_reg reg, uint8_t *value);
int xtensa_dm_queue_reg_write(struct xtensa_debug_module *dm, enum xtensa_dm_reg reg, uint32_t value);
int xtensa_dm_queue_pwr_reg_read(struct xtensa_debug_module *dm,
	enum xtensa_dm_pwr_reg reg,
	uint8_t *data,
	uint32_t clear);
int xtensa_dm_queue_pwr_reg_write(struct xtensa_debug_module *dm,
	enum xtensa_dm_pwr_reg reg,
	uint32_t data);

static inline int xtensa_dm_queue_execute(struct xtensa_debug_module *dm)
{
	return dm->dap ? dap_run(dm->dap) : jtag_execute_queue();
}

static inline void xtensa_dm_queue_tdi_idle(struct xtensa_debug_module *dm)
{
	if (dm->queue_tdi_idle)
		dm->queue_tdi_idle(dm->queue_tdi_idle_arg);
}

int xtensa_dm_power_status_read(struct xtensa_debug_module *dm, uint32_t clear);
static inline void xtensa_dm_power_status_cache_reset(struct xtensa_debug_module *dm)
{
	dm->power_status.prev_stat = 0;
}
static inline void xtensa_dm_power_status_cache(struct xtensa_debug_module *dm)
{
	dm->power_status.prev_stat = dm->power_status.stath;
}
static inline xtensa_pwrstat_t xtensa_dm_power_status_get(struct xtensa_debug_module *dm)
{
	return dm->power_status.stat;
}

int xtensa_dm_core_status_read(struct xtensa_debug_module *dm);
int xtensa_dm_core_status_clear(struct xtensa_debug_module *dm, xtensa_dsr_t bits);
int xtensa_dm_core_status_check(struct xtensa_debug_module *dm);
static inline xtensa_dsr_t xtensa_dm_core_status_get(struct xtensa_debug_module *dm)
{
	return dm->core_status.dsr;
}

int xtensa_dm_device_id_read(struct xtensa_debug_module *dm);
static inline xtensa_ocdid_t xtensa_dm_device_id_get(struct xtensa_debug_module *dm)
{
	return dm->device_id;
}

int xtensa_dm_trace_start(struct xtensa_debug_module *dm, struct xtensa_trace_start_config *cfg);
int xtensa_dm_trace_stop(struct xtensa_debug_module *dm, bool pto_enable);
int xtensa_dm_trace_config_read(struct xtensa_debug_module *dm, struct xtensa_trace_config *config);
int xtensa_dm_trace_status_read(struct xtensa_debug_module *dm, struct xtensa_trace_status *status);
int xtensa_dm_trace_data_read(struct xtensa_debug_module *dm, uint8_t *dest, uint32_t size);

static inline bool xtensa_dm_is_online(struct xtensa_debug_module *dm)
{
	int res = xtensa_dm_device_id_read(dm);
	if (res != ERROR_OK)
		return false;
	return dm->device_id != 0xffffffff && dm->device_id != 0;
}

static inline bool xtensa_dm_tap_was_reset(struct xtensa_debug_module *dm)
{
	return !(dm->power_status.prev_stat & PWRSTAT_DEBUGWASRESET_DM(dm)) &&
	       dm->power_status.stat & PWRSTAT_DEBUGWASRESET_DM(dm);
}

static inline bool xtensa_dm_core_was_reset(struct xtensa_debug_module *dm)
{
	return !(dm->power_status.prev_stat & PWRSTAT_COREWASRESET_DM(dm)) &&
	       dm->power_status.stat & PWRSTAT_COREWASRESET_DM(dm);
}

static inline bool xtensa_dm_core_is_stalled(struct xtensa_debug_module *dm)
{
	return dm->core_status.dsr & OCDDSR_RUNSTALLSAMPLE;
}

static inline bool xtensa_dm_is_powered(struct xtensa_debug_module *dm)
{
	return dm->core_status.dsr & OCDDSR_DBGMODPOWERON;
}

int xtensa_dm_perfmon_enable(struct xtensa_debug_module *dm, int counter_id,
	const struct xtensa_perfmon_config *config);
int xtensa_dm_perfmon_dump(struct xtensa_debug_module *dm, int counter_id,
	struct xtensa_perfmon_result *out_result);

#endif	/* OPENOCD_TARGET_XTENSA_DEBUG_MODULE_H */
