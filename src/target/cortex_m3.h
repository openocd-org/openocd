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

#include "register.h"
#include "target.h"
#include "armv7m.h"
#include "cortex_swjdp.h"

extern char* cortex_m3_state_strings[];

#define CORTEX_M3_COMMON_MAGIC 0x1A451A45

#define SYSTEM_CONTROL_BASE 0x400FE000

#define CPUID		0xE000ED00
/* Debug Control Block */
#define DCB_DHCSR	0xE000EDF0
#define DCB_DCRSR	0xE000EDF4
#define DCB_DCRDR	0xE000EDF8
#define DCB_DEMCR	0xE000EDFC

#define DCRSR_WnR	(1<<16)	

#define DWT_CTRL	0xE0001000
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

#define DWT_CTRL	0xE0001000

/* DCB_DHCSR bit and field definitions */
#define DBGKEY		(0xA05F<<16)
#define C_DEBUGEN	(1<<0)
#define C_HALT		(1<<1)
#define C_STEP		(1<<2)
#define C_MASKINTS	(1<<3)
#define S_REGRDY	(1<<16)
#define S_HALT		(1<<17)
#define S_SLEEP		(1<<18)
#define S_LOCKUP	(1<<19)
#define S_RETIRE_ST	(1<<24)
#define S_RESET_ST	(1<<25)

/* DCB_DEMCR bit and field definitions */
#define	TRCENA			(1<<24)
#define	VC_HARDERR		(1<<10)
#define	VC_BUSERR		(1<<8)
#define	VC_CORERESET	(1<<0)

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
#define AIRCR_VECTKEY		(0x5FA<<16)
#define AIRCR_SYSRESETREQ	(1<<2)
#define AIRCR_VECTCLRACTIVE	(1<<1)
#define AIRCR_VECTRESET		(1<<0)
/* NVIC_SHCSR bits */
#define SHCSR_BUSFAULTENA	(1<<17)
/* NVIC_DFSR bits */
#define DFSR_HALTED			1
#define DFSR_BKPT			2
#define DFSR_DWTTRAP		4
#define DFSR_VCATCH			8

#define FPCR_CODE 0
#define FPCR_LITERAL 1
#define FPCR_REPLACE_REMAP  (0<<30)
#define FPCR_REPLACE_BKPT_LOW  (1<<30)
#define FPCR_REPLACE_BKPT_HIGH  (2<<30)
#define FPCR_REPLACE_BKPT_BOTH  (3<<30)

typedef struct  cortex_m3_fp_comparator_s
{
	int used;
	int type;
	u32 fpcr_value;
	u32 fpcr_address;
} cortex_m3_fp_comparator_t;

typedef struct  cortex_m3_dwt_comparator_s
{
	int used;
	u32 comp;
	u32 mask;
	u32 function;
	u32 dwt_comparator_address;
} cortex_m3_dwt_comparator_t;

typedef struct cortex_m3_common_s
{
	int common_magic;
	arm_jtag_t jtag_info;
	
	/* Context information */
	u32 dcb_dhcsr;
	u32 nvic_dfsr;  /* Debug Fault Status Register - shows reason for debug halt */
	u32 nvic_icsr;  /* Interrupt Control State Register - shows active and pending IRQ */
	
	/* Flash Patch and Breakpoint (FPB) */
	int fp_num_lit;
	int fp_num_code;
	int fp_code_available;
	int fpb_enabled;
	int auto_bp_type;
	cortex_m3_fp_comparator_t *fp_comparator_list;
	
	/* Data Watchpoint and Trace (DWT) */
	int dwt_num_comp;
	int dwt_comp_available;
	cortex_m3_dwt_comparator_t *dwt_comparator_list;
	
	/* Interrupts */
	int intlinesnum;
	u32 *intsetenable;
	
	armv7m_common_t armv7m;
	swjdp_common_t swjdp_info;
	void *arch_info;
} cortex_m3_common_t;

extern void cortex_m3_build_reg_cache(target_t *target);

int cortex_m3_poll(target_t *target);
int cortex_m3_halt(target_t *target);
int cortex_m3_resume(struct target_s *target, int current, u32 address, int handle_breakpoints, int debug_execution);
int cortex_m3_step(struct target_s *target, int current, u32 address, int handle_breakpoints);

int cortex_m3_assert_reset(target_t *target);
int cortex_m3_deassert_reset(target_t *target);
int cortex_m3_soft_reset_halt(struct target_s *target);

int cortex_m3_read_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);
int cortex_m3_write_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);
int cortex_m3_bulk_write_memory(target_t *target, u32 address, u32 count, u8 *buffer);

int cortex_m3_set_breakpoint(struct target_s *target, breakpoint_t *breakpoint);
int cortex_m3_unset_breakpoint(struct target_s *target, breakpoint_t *breakpoint);
int cortex_m3_add_breakpoint(struct target_s *target, breakpoint_t *breakpoint);
int cortex_m3_remove_breakpoint(struct target_s *target, breakpoint_t *breakpoint);
int cortex_m3_add_watchpoint(struct target_s *target, watchpoint_t *watchpoint);
int cortex_m3_remove_watchpoint(struct target_s *target, watchpoint_t *watchpoint);

extern int cortex_m3_register_commands(struct command_context_s *cmd_ctx);
extern int cortex_m3_init_arch_info(target_t *target, cortex_m3_common_t *cortex_m3, jtag_tap_t *tap);

#endif /* CORTEX_M3_H */
