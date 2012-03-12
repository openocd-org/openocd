/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef CORTEX_M3_H
#define CORTEX_M3_H

#include "armv7m.h"

#define CORTEX_M3_COMMON_MAGIC 0x1A451A45

#define SYSTEM_CONTROL_BASE 0x400FE000

#define CPUID		0xE000ED00
/* Debug Control Block */
#define DCB_DHCSR	0xE000EDF0
#define DCB_DCRSR	0xE000EDF4
#define DCB_DCRDR	0xE000EDF8
#define DCB_DEMCR	0xE000EDFC

#define DCRSR_WnR	(1 << 16)

#define DWT_CTRL	0xE0001000
#define DWT_CYCCNT	0xE0001004
#define DWT_COMP0	0xE0001020
#define DWT_MASK0	0xE0001024
#define DWT_FUNCTION0	0xE0001028

#define FP_CTRL		0xE0002000
#define FP_REMAP	0xE0002004
#define FP_COMP0	0xE0002008
#define FP_COMP1	0xE000200C
#define FP_COMP2	0xE0002010
#define FP_COMP3	0xE0002014
#define FP_COMP4	0xE0002018
#define FP_COMP5	0xE000201C
#define FP_COMP6	0xE0002020
#define FP_COMP7	0xE0002024

/* DCB_DHCSR bit and field definitions */
#define DBGKEY		(0xA05F << 16)
#define C_DEBUGEN	(1 << 0)
#define C_HALT		(1 << 1)
#define C_STEP		(1 << 2)
#define C_MASKINTS	(1 << 3)
#define S_REGRDY	(1 << 16)
#define S_HALT		(1 << 17)
#define S_SLEEP		(1 << 18)
#define S_LOCKUP	(1 << 19)
#define S_RETIRE_ST	(1 << 24)
#define S_RESET_ST	(1 << 25)

/* DCB_DEMCR bit and field definitions */
#define	TRCENA			(1 << 24)
#define	VC_HARDERR		(1 << 10)
#define	VC_INTERR		(1 << 9)
#define	VC_BUSERR		(1 << 8)
#define	VC_STATERR		(1 << 7)
#define	VC_CHKERR		(1 << 6)
#define	VC_NOCPERR		(1 << 5)
#define	VC_MMERR		(1 << 4)
#define	VC_CORERESET	(1 << 0)

#define NVIC_ICTR		0xE000E004
#define NVIC_ISE0		0xE000E100
#define NVIC_ICSR		0xE000ED04
#define NVIC_AIRCR		0xE000ED0C
#define NVIC_SHCSR		0xE000ED24
#define NVIC_CFSR		0xE000ED28
#define NVIC_MMFSRb		0xE000ED28
#define NVIC_BFSRb		0xE000ED29
#define NVIC_USFSRh		0xE000ED2A
#define NVIC_HFSR		0xE000ED2C
#define NVIC_DFSR		0xE000ED30
#define NVIC_MMFAR		0xE000ED34
#define NVIC_BFAR		0xE000ED38

/* NVIC_AIRCR bits */
#define AIRCR_VECTKEY		(0x5FA << 16)
#define AIRCR_SYSRESETREQ	(1 << 2)
#define AIRCR_VECTCLRACTIVE	(1 << 1)
#define AIRCR_VECTRESET		(1 << 0)
/* NVIC_SHCSR bits */
#define SHCSR_BUSFAULTENA	(1 << 17)
/* NVIC_DFSR bits */
#define DFSR_HALTED			1
#define DFSR_BKPT			2
#define DFSR_DWTTRAP		4
#define DFSR_VCATCH			8

#define FPCR_CODE 0
#define FPCR_LITERAL 1
#define FPCR_REPLACE_REMAP  (0 << 30)
#define FPCR_REPLACE_BKPT_LOW  (1 << 30)
#define FPCR_REPLACE_BKPT_HIGH  (2 << 30)
#define FPCR_REPLACE_BKPT_BOTH  (3 << 30)

struct cortex_m3_fp_comparator {
	int used;
	int type;
	uint32_t fpcr_value;
	uint32_t fpcr_address;
};

struct cortex_m3_dwt_comparator {
	int used;
	uint32_t comp;
	uint32_t mask;
	uint32_t function;
	uint32_t dwt_comparator_address;
};

enum cortex_m3_soft_reset_config {
	CORTEX_M3_RESET_SYSRESETREQ,
	CORTEX_M3_RESET_VECTRESET,
};

enum cortex_m3_isrmasking_mode {
	CORTEX_M3_ISRMASK_AUTO,
	CORTEX_M3_ISRMASK_OFF,
	CORTEX_M3_ISRMASK_ON,
};

struct cortex_m3_common {
	int common_magic;
	struct arm_jtag jtag_info;

	/* Context information */
	uint32_t dcb_dhcsr;
	uint32_t nvic_dfsr;  /* Debug Fault Status Register - shows reason for debug halt */
	uint32_t nvic_icsr;  /* Interrupt Control State Register - shows active and pending IRQ */

	/* Flash Patch and Breakpoint (FPB) */
	int fp_num_lit;
	int fp_num_code;
	int fp_code_available;
	int fpb_enabled;
	int auto_bp_type;
	struct cortex_m3_fp_comparator *fp_comparator_list;

	/* Data Watchpoint and Trace (DWT) */
	int dwt_num_comp;
	int dwt_comp_available;
	struct cortex_m3_dwt_comparator *dwt_comparator_list;
	struct reg_cache *dwt_cache;

	enum cortex_m3_soft_reset_config soft_reset_config;

	enum cortex_m3_isrmasking_mode isrmasking_mode;

	struct armv7m_common armv7m;
};

static inline struct cortex_m3_common *
target_to_cm3(struct target *target)
{
	return container_of(target->arch_info,
			struct cortex_m3_common, armv7m);
}

int cortex_m3_examine(struct target *target);
int cortex_m3_set_breakpoint(struct target *target, struct breakpoint *breakpoint);
int cortex_m3_unset_breakpoint(struct target *target, struct breakpoint *breakpoint);
int cortex_m3_add_breakpoint(struct target *target, struct breakpoint *breakpoint);
int cortex_m3_remove_breakpoint(struct target *target, struct breakpoint *breakpoint);
int cortex_m3_set_watchpoint(struct target *target, struct watchpoint *watchpoint);
int cortex_m3_unset_watchpoint(struct target *target, struct watchpoint *watchpoint);
int cortex_m3_add_watchpoint(struct target *target, struct watchpoint *watchpoint);
int cortex_m3_remove_watchpoint(struct target *target, struct watchpoint *watchpoint);
void cortex_m3_enable_watchpoints(struct target *target);
void cortex_m3_dwt_setup(struct cortex_m3_common *cm3, struct target *target);

#endif /* CORTEX_M3_H */
