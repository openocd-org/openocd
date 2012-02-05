/*
 * Copyright (C) 2009 by David Brownell
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the
 * Free Software Foundation, Inc.,
 * 59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 */

#ifndef __ARM_DPM_H
#define __ARM_DPM_H

/**
 * @file
 * This is the interface to the Debug Programmers Model for ARMv6 and
 * ARMv7 processors.  ARMv6 processors (such as ARM11xx implementations)
 * introduced a model which became part of the ARMv7-AR architecture
 * which is most familiar through the Cortex-A series parts.  While
 * specific details differ (like how to write the instruction register),
 * the high level models easily support shared code because those
 * registers are compatible.
 */

struct dpm_bpwp {
	unsigned number;
	uint32_t address;
	uint32_t control;
	/* true if hardware state needs flushing */
	bool dirty;
};

struct dpm_bp {
	struct breakpoint *bp;
	struct dpm_bpwp bpwp;
};

struct dpm_wp {
	struct watchpoint *wp;
	struct dpm_bpwp bpwp;
};

/**
 * This wraps an implementation of DPM primitives.  Each interface
 * provider supplies a structure like this, which is the glue between
 * upper level code and the lower level hardware access.
 *
 * It is a PRELIMINARY AND INCOMPLETE set of primitives, starting with
 * support for CPU register access.
 */
struct arm_dpm {
	struct arm *arm;

	/** Cache of DIDR */
	uint32_t didr;

	/** Invoke before a series of instruction operations */
	int (*prepare)(struct arm_dpm *);

	/** Invoke after a series of instruction operations */
	int (*finish)(struct arm_dpm *);

	/* WRITE TO CPU */

	/** Runs one instruction, writing data to DCC before execution. */
	int (*instr_write_data_dcc)(struct arm_dpm *,
			uint32_t opcode, uint32_t data);

	/** Runs one instruction, writing data to R0 before execution. */
	int (*instr_write_data_r0)(struct arm_dpm *,
			uint32_t opcode, uint32_t data);

	/** Optional core-specific operation invoked after CPSR writes. */
	int (*instr_cpsr_sync)(struct arm_dpm *dpm);

	/* READ FROM CPU */

	/** Runs one instruction, reading data from dcc after execution. */
	int (*instr_read_data_dcc)(struct arm_dpm *,
			uint32_t opcode, uint32_t *data);

	/** Runs one instruction, reading data from r0 after execution. */
	int (*instr_read_data_r0)(struct arm_dpm *,
			uint32_t opcode, uint32_t *data);

	/* BREAKPOINT/WATCHPOINT SUPPORT */

	/**
	 * Enables one breakpoint or watchpoint by writing to the
	 * hardware registers.  The specified breakpoint/watchpoint
	 * must currently be disabled.  Indices 0..15 are used for
	 * breakpoints; indices 16..31 are for watchpoints.
	 */
	int (*bpwp_enable)(struct arm_dpm *, unsigned index_value,
			uint32_t addr, uint32_t control);

	/**
	 * Disables one breakpoint or watchpoint by clearing its
	 * hardware control registers.  Indices are the same ones
	 * accepted by bpwp_enable().
	 */
	int (*bpwp_disable)(struct arm_dpm *, unsigned index_value);

	/* The breakpoint and watchpoint arrays are private to the
	 * DPM infrastructure.  There are nbp indices in the dbp
	 * array.  There are nwp indices in the dwp array.
	 */

	unsigned nbp;
	unsigned nwp;
	struct dpm_bp *dbp;
	struct dpm_wp *dwp;

	/** Address of the instruction which triggered a watchpoint. */
	uint32_t wp_pc;

	/** Recent value of DSCR. */
	uint32_t dscr;

	/* FIXME -- read/write DCSR methods and symbols */
};

int arm_dpm_setup(struct arm_dpm *dpm);
int arm_dpm_initialize(struct arm_dpm *dpm);

int arm_dpm_read_current_registers(struct arm_dpm *);
int dpm_modeswitch(struct arm_dpm *dpm, enum arm_mode mode);


int arm_dpm_write_dirty_registers(struct arm_dpm *, bool bpwp);

void arm_dpm_report_wfar(struct arm_dpm *, uint32_t wfar);

/* Subset of DSCR bits; see ARMv7a arch spec section C10.3.1.
 * Not all v7 bits are valid in v6.
 */
#define DSCR_CORE_HALTED	(1 << 0)
#define DSCR_CORE_RESTARTED	(1 << 1)
#define DSCR_INT_DIS		(1 << 11)
#define DSCR_ITR_EN			(1 << 13)
#define DSCR_HALT_DBG_MODE	(1 << 14)
#define DSCR_MON_DBG_MODE	(1 << 15)
#define DSCR_INSTR_COMP		(1 << 24)
#define DSCR_DTR_TX_FULL	(1 << 29)
#define DSCR_DTR_RX_FULL	(1 << 30)

#define DSCR_ENTRY(dscr) 	(((dscr) >> 2) & 0xf)
#define DSCR_RUN_MODE(dscr)	((dscr) & (DSCR_CORE_HALTED | DSCR_CORE_RESTARTED))

/* DRCR (debug run control register) bits */
#define DRCR_HALT				(1 << 0)
#define DRCR_RESTART			(1 << 1)
#define DRCR_CLEAR_EXCEPTIONS	(1 << 2)

void arm_dpm_report_dscr(struct arm_dpm *dpm, uint32_t dcsr);

#endif /* __ARM_DPM_H */
