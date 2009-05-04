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
#include "armv7m.h"

extern char* cortex_a8_state_strings[];

#define CORTEX_A8_COMMON_MAGIC 0x411fc082

#define CPUID		0x54011D00
/* Debug Control Block */
#define DCB_DHCSR	0x54011DF0
#define DCB_DCRSR	0x54011DF4
#define DCB_DCRDR	0x54011DF8
#define DCB_DEMCR	0x54011DFC

typedef struct  cortex_a8_fp_comparator_s
{
	int used;
	int type;
	u32 fpcr_value;
	u32 fpcr_address;
} cortex_a8_fp_comparator_t;

typedef struct  cortex_a8_dwt_comparator_s
{
	int used;
	u32 comp;
	u32 mask;
	u32 function;
	u32 dwt_comparator_address;
} cortex_a8_dwt_comparator_t;

typedef struct cortex_a8_common_s
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
	cortex_a8_fp_comparator_t *fp_comparator_list;

	/* Data Watchpoint and Trace (DWT) */
	int dwt_num_comp;
	int dwt_comp_available;
	cortex_a8_dwt_comparator_t *dwt_comparator_list;

	/* Interrupts */
	int intlinesnum;
	u32 *intsetenable;

	armv7m_common_t armv7m;
	void *arch_info;
} cortex_a8_common_t;

extern int cortex_a8_init_arch_info(target_t *target, cortex_a8_common_t *cortex_a8, jtag_tap_t *tap);
int cortex_a8_read_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);
int cortex_a8_write_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer);

#endif /* CORTEX_A8_H */
