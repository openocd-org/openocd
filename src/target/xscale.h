/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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
#ifndef XSCALE_H
#define XSCALE_H

#include "target.h"
#include "register.h"
#include "armv4_5.h"
#include "armv4_5_mmu.h"

#define	XSCALE_COMMON_MAGIC 0x58534341

typedef struct xscale_jtag_s
{
	/* position in JTAG scan chain */
	int chain_pos;

	/* IR length and instructions */	
	int ir_length;
	u32 dbgrx;
	u32 dbgtx;
	u32 ldic;
	u32 dcsr;
} xscale_jtag_t;

enum xscale_debug_reason
{
	XSCALE_DBG_REASON_GENERIC,
	XSCALE_DBG_REASON_RESET,
	XSCALE_DBG_REASON_TB_FULL,
};

typedef struct xscale_common_s
{
	int common_magic;
	
	/* XScale registers (CP15, DBG) */
	reg_cache_t *reg_cache;

	/* pxa250, pxa255, pxa27x, ixp42x, ... */
	char *variant;

	xscale_jtag_t jtag_info;
	
	/* current state of the debug handler */
	int handler_installed;
	int handler_running;
	u32 handler_address;
	
	/* target-endian buffers with exception vectors */
	u32 low_vectors[8];
	u32 high_vectors[8];
	
	/* static low vectors */
	u8 static_low_vectors_set;	/* bit field with static vectors set by the user */
	u8 static_high_vectors_set; /* bit field with static vectors set by the user */
	u32 static_low_vectors[8];
	u32 static_high_vectors[8];

	/* DCache cleaning */	
	u32 cache_clean_address;
	
	/* whether hold_rst and ext_dbg_break should be set */
	int hold_rst;
	int external_debug_break;
	
	/* breakpoint / watchpoint handling */
	int force_hw_bkpts;
	int dbr_available;
	int dbr0_used;
	int dbr1_used;
	int ibcr_available;
	int ibcr0_used;
	int	ibcr1_used;
	u32 arm_bkpt;
	u16 thumb_bkpt;
	
	u8 vector_catch;
	
	int trace_buffer_enabled;
	int trace_buffer_fill;
	
	int arch_debug_reason;
	
	/* armv4/5 common stuff */
	armv4_5_common_t armv4_5_common;
	
	/* MMU/Caches */
	armv4_5_mmu_common_t armv4_5_mmu;
	u32 cp15_control_reg;
	
	/* possible future enhancements that go beyond XScale common stuff */
	void *arch_info;
} xscale_common_t;

typedef struct xscale_reg_s
{
	int dbg_handler_number;
	target_t *target;
} xscale_reg_t;

enum
{
	XSCALE_MAINID,		/* 0 */
	XSCALE_CACHETYPE,
	XSCALE_CTRL,
	XSCALE_AUXCTRL,
	XSCALE_TTB,
	XSCALE_DAC,
	XSCALE_FSR,
	XSCALE_FAR,
	XSCALE_PID,
	XSCALE_CPACCESS,
	XSCALE_IBCR0,		/* 10 */
	XSCALE_IBCR1,
	XSCALE_DBR0,
	XSCALE_DBR1,
	XSCALE_DBCON,
	XSCALE_TBREG,
	XSCALE_CHKPT0,
	XSCALE_CHKPT1,
	XSCALE_DCSR,
	XSCALE_TX,
	XSCALE_RX,			/* 20 */
	XSCALE_TXRXCTRL,
};

#endif /* XSCALE_H */
