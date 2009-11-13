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
 *   Copyright (C) 2009 by Dirk Behme                                      *
 *   dirk.behme@gmail.com - copy from cortex_m3                            *
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
#ifndef CORTEX_A8_H
#define CORTEX_A8_H

#include "register.h"
#include "target.h"
#include "armv7a.h"
#include "arm7_9_common.h"

extern char* cortex_a8_state_strings[];

#define CORTEX_A8_COMMON_MAGIC 0x411fc082

#define CPUID		0x54011D00
/* Debug Control Block */
#define CPUDBG_DIDR		0x000
#define CPUDBG_WFAR		0x018
#define CPUDBG_VCR	0x01C
#define CPUDBG_ECR	0x024
#define CPUDBG_DSCCR	0x028
#define CPUDBG_DTRRX	0x080
#define CPUDBG_ITR	0x084
#define CPUDBG_DSCR	0x088
#define CPUDBG_DTRTX	0x08c
#define CPUDBG_DRCR	0x090
#define CPUDBG_BVR_BASE	0x100
#define CPUDBG_BCR_BASE	0x140
#define CPUDBG_WVR_BASE	0x180
#define CPUDBG_WCR_BASE	0x1C0

#define CPUDBG_OSLAR	0x300
#define CPUDBG_OSLSR	0x304
#define CPUDBG_OSSRR	0x308

#define CPUDBG_PRCR	0x310
#define CPUDBG_PRSR	0x314

#define CPUDBG_CPUID	0xD00
#define CPUDBG_CTYPR	0xD04
#define CPUDBG_TTYPR	0xD0C
#define CPUDBG_LOCKACCESS 0xFB0
#define CPUDBG_LOCKSTATUS 0xFB4
#define CPUDBG_AUTHSTATUS 0xFB8

#define BRP_NORMAL 0
#define BRP_CONTEXT 1

/* DSCR Bit offset */
#define DSCR_CORE_HALTED		0
#define DSCR_CORE_RESTARTED 	1
#define DSCR_EXT_INT_EN 		13
#define DSCR_HALT_DBG_MODE		14
#define DSCR_MON_DBG_MODE 		15
#define DSCR_INSTR_COMP 		24
#define DSCR_DTR_TX_FULL 		29
#define DSCR_DTR_RX_FULL 		30

typedef struct  cortex_a8_brp_s
{
	int used;
	int type;
	uint32_t value;
	uint32_t control;
	uint8_t 	BRPn;
} cortex_a8_brp_t;

typedef struct  cortex_a8_wrp_s
{
	int used;
	int type;
	uint32_t value;
	uint32_t control;
	uint8_t 	WRPn;
} cortex_a8_wrp_t;

struct cortex_a8_common
{
	int common_magic;
	struct arm_jtag jtag_info;

	/* Context information */
	uint32_t cpudbg_dscr;
	uint32_t nvic_dfsr;  /* Debug Fault Status Register - shows reason for debug halt */
	uint32_t nvic_icsr;  /* Interrupt Control State Register - shows active and pending IRQ */

	/* Saved cp15 registers */
	uint32_t cp15_control_reg;
	uint32_t cp15_aux_control_reg;

	/* Breakpoint register pairs */
	int brp_num_context;
	int brp_num;
	int brp_num_available;
//	int brp_enabled;
	cortex_a8_brp_t *brp_list;

	/* Watchpoint register pairs */
	int wrp_num;
	int wrp_num_available;
	cortex_a8_wrp_t *wrp_list;

	/* Interrupts */
	int intlinesnum;
	uint32_t *intsetenable;

	/* Use cortex_a8_read_regs_through_mem for fast register reads */
	int fast_reg_read;

	struct armv7a_common armv7a_common;
};

static inline struct cortex_a8_common *
target_to_cortex_a8(struct target_s *target)
{
	return container_of(target->arch_info, struct cortex_a8_common,
			armv7a_common.armv4_5_common);
}

int cortex_a8_init_arch_info(target_t *target,
		struct cortex_a8_common *cortex_a8, struct jtag_tap *tap);

#endif /* CORTEX_A8_H */
