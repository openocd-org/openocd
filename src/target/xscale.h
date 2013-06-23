/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifndef XSCALE_H
#define XSCALE_H

#include "arm.h"
#include "armv4_5_mmu.h"
#include "trace.h"

#define	XSCALE_COMMON_MAGIC 0x58534341

/* These four JTAG instructions are architecturally defined.
 * Lengths are core-specific; originally 5 bits, later 7.
 */
#define XSCALE_DBGRX	0x02
#define XSCALE_DBGTX	0x10
#define XSCALE_LDIC	0x07
#define XSCALE_SELDCSR	0x09

/* Possible CPU types */
#define	XSCALE_IXP4XX_PXA2XX	0x0
#define	XSCALE_PXA3XX		0x4

enum xscale_debug_reason {
	XSCALE_DBG_REASON_GENERIC,
	XSCALE_DBG_REASON_RESET,
	XSCALE_DBG_REASON_TB_FULL,
};

enum xscale_trace_entry_type {
	XSCALE_TRACE_MESSAGE = 0x0,
	XSCALE_TRACE_ADDRESS = 0x1,
};

struct xscale_trace_entry {
	uint8_t data;
	enum xscale_trace_entry_type type;
};

struct xscale_trace_data {
	struct xscale_trace_entry *entries;
	int depth;
	uint32_t chkpt0;
	uint32_t chkpt1;
	uint32_t last_instruction;
	unsigned int num_checkpoints;
	struct xscale_trace_data *next;
};

enum trace_mode {
	XSCALE_TRACE_DISABLED,
	XSCALE_TRACE_FILL,
	XSCALE_TRACE_WRAP
};

struct xscale_trace {
	struct image *image;					/* source for target opcodes */
	struct xscale_trace_data *data;		/* linked list of collected trace data */
	int buffer_fill;				/* maximum number of trace runs to read */
	int fill_counter;				/* running count during trace collection */
	enum trace_mode mode;
	enum arm_state core_state;	/* current core state (ARM, Thumb) */
};

struct xscale_common {
	/* armv4/5 common stuff */
	struct arm arm;

	int common_magic;

	/* XScale registers (CP15, DBG) */
	struct reg_cache *reg_cache;

	/* current state of the debug handler */
	uint32_t handler_address;

	/* target-endian buffers with exception vectors */
	uint32_t low_vectors[8];
	uint32_t high_vectors[8];

	/* static low vectors */
	uint8_t static_low_vectors_set;	/* bit field with static vectors set by the user */
	uint8_t static_high_vectors_set; /* bit field with static vectors set by the user */
	uint32_t static_low_vectors[8];
	uint32_t static_high_vectors[8];

	/* DCache cleaning */
	uint32_t cache_clean_address;

	/* whether hold_rst and ext_dbg_break should be set */
	int hold_rst;
	int external_debug_break;

	/* breakpoint / watchpoint handling */
	int dbr_available;
	int dbr0_used;
	int dbr1_used;
	int ibcr_available;
	int ibcr0_used;
	int	ibcr1_used;
	uint32_t arm_bkpt;
	uint16_t thumb_bkpt;

	uint8_t vector_catch;

	struct xscale_trace trace;

	int arch_debug_reason;

	/* MMU/Caches */
	struct armv4_5_mmu_common armv4_5_mmu;
	uint32_t cp15_control_reg;

	int fast_memory_access;

	/* CPU variant */
	int xscale_variant;
};

static inline struct xscale_common *
target_to_xscale(struct target *target)
{
	return container_of(target->arch_info, struct xscale_common, arm);
}

struct xscale_reg {
	int dbg_handler_number;
	struct target *target;
};

enum {
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

#define ERROR_XSCALE_NO_TRACE_DATA	(-700)

/* DCSR bit and field definitions */
#define DCSR_TR	(1 << 16)
#define DCSR_TU	(1 << 17)
#define DCSR_TS	(1 << 18)
#define DCSR_TA	(1 << 19)
#define DCSR_TD	(1 << 20)
#define DCSR_TI	(1 << 22)
#define DCSR_TF	(1 << 23)
#define DCSR_TRAP_MASK \
	(DCSR_TF | DCSR_TI | DCSR_TD | DCSR_TA | DCSR_TS | DCSR_TU | DCSR_TR)

#endif /* XSCALE_H */
