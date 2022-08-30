// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Generic Xtensa target API for OpenOCD                                 *
 *   Copyright (C) 2020-2022 Cadence Design Systems, Inc.                  *
 *   Copyright (C) 2016-2019 Espressif Systems Ltd.                        *
 *   Derived from esp108.c                                                 *
 *   Author: Angus Gratton gus@projectgus.com                              *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <helper/time_support.h>
#include <helper/align.h>
#include <target/register.h>

#include "xtensa_chip.h"
#include "xtensa.h"

/* Swap 4-bit Xtensa opcodes and fields */
#define XT_NIBSWAP8(V)									\
	((((V) & 0x0F) << 4)								\
		| (((V) & 0xF0) >> 4))

#define XT_NIBSWAP16(V)									\
	((((V) & 0x000F) << 12)								\
		| (((V) & 0x00F0) << 4)							\
		| (((V) & 0x0F00) >> 4)							\
		| (((V) & 0xF000) >> 12))

#define XT_NIBSWAP24(V)									\
	((((V) & 0x00000F) << 20)							\
		| (((V) & 0x0000F0) << 12)						\
		| (((V) & 0x000F00) << 4)						\
		| (((V) & 0x00F000) >> 4)						\
		| (((V) & 0x0F0000) >> 12)						\
		| (((V) & 0xF00000) >> 20))

/* _XT_INS_FORMAT_*()
 * Instruction formatting converted from little-endian inputs
 * and shifted to the MSB-side of DIR for BE systems.
 */
#define _XT_INS_FORMAT_RSR(X, OPCODE, SR, T)			\
	(XT_ISBE(X) ? (XT_NIBSWAP24(OPCODE)					\
			| (((T) & 0x0F) << 16)						\
			| (((SR) & 0xFF) << 8)) << 8				\
		: (OPCODE)										\
		| (((SR) & 0xFF) << 8)							\
		| (((T) & 0x0F) << 4))

#define _XT_INS_FORMAT_RRR(X, OPCODE, ST, R)			\
	(XT_ISBE(X) ? (XT_NIBSWAP24(OPCODE)					\
			| ((XT_NIBSWAP8((ST) & 0xFF)) << 12)		\
			| (((R) & 0x0F) << 8)) << 8					\
		: (OPCODE)										\
		| (((ST) & 0xFF) << 4)							\
		| (((R) & 0x0F) << 12))

#define _XT_INS_FORMAT_RRRN(X, OPCODE, S, T, IMM4)		\
	(XT_ISBE(X) ? (XT_NIBSWAP16(OPCODE)					\
			| (((T) & 0x0F) << 8)						\
			| (((S) & 0x0F) << 4)						\
			| ((IMM4) & 0x0F)) << 16					\
		: (OPCODE)										\
		| (((T) & 0x0F) << 4)							\
		| (((S) & 0x0F) << 8)							\
		| (((IMM4) & 0x0F) << 12))

#define _XT_INS_FORMAT_RRI8(X, OPCODE, R, S, T, IMM8)	\
	(XT_ISBE(X) ? (XT_NIBSWAP24(OPCODE)					\
			| (((T) & 0x0F) << 16)						\
			| (((S) & 0x0F) << 12)						\
			| (((R) & 0x0F) << 8)						\
			| ((IMM8) & 0xFF)) << 8						\
		: (OPCODE)										\
		| (((IMM8) & 0xFF) << 16)						\
		| (((R) & 0x0F) << 12)							\
		| (((S) & 0x0F) << 8)							\
		| (((T) & 0x0F) << 4))

#define _XT_INS_FORMAT_RRI4(X, OPCODE, IMM4, R, S, T)	\
	(XT_ISBE(X) ? (XT_NIBSWAP24(OPCODE)					\
			| (((T) & 0x0F) << 16)						\
			| (((S) & 0x0F) << 12)						\
			| (((R) & 0x0F) << 8)) << 8					\
		| ((IMM4) & 0x0F)								\
		: (OPCODE)										\
		| (((IMM4) & 0x0F) << 20)						\
		| (((R) & 0x0F) << 12)							\
		| (((S) & 0x0F) << 8)							\
		| (((T) & 0x0F) << 4))

/* Xtensa processor instruction opcodes
*/
/* "Return From Debug Operation" to Normal */
#define XT_INS_RFDO(X) (XT_ISBE(X) ? 0x000e1f << 8 : 0xf1e000)
/* "Return From Debug and Dispatch" - allow sw debugging stuff to take over */
#define XT_INS_RFDD(X) (XT_ISBE(X) ? 0x010e1f << 8 : 0xf1e010)

/* Load to DDR register, increase addr register */
#define XT_INS_LDDR32P(X, S) (XT_ISBE(X) ? (0x0E0700 | ((S) << 12)) << 8 : (0x0070E0 | ((S) << 8)))
/* Store from DDR register, increase addr register */
#define XT_INS_SDDR32P(X, S) (XT_ISBE(X) ? (0x0F0700 | ((S) << 12)) << 8 : (0x0070F0 | ((S) << 8)))

/* Load 32-bit Indirect from A(S)+4*IMM8 to A(T) */
#define XT_INS_L32I(X, S, T, IMM8)  _XT_INS_FORMAT_RRI8(X, 0x002002, 0, S, T, IMM8)
/* Load 16-bit Unsigned from A(S)+2*IMM8 to A(T) */
#define XT_INS_L16UI(X, S, T, IMM8) _XT_INS_FORMAT_RRI8(X, 0x001002, 0, S, T, IMM8)
/* Load 8-bit Unsigned from A(S)+IMM8 to A(T) */
#define XT_INS_L8UI(X, S, T, IMM8)  _XT_INS_FORMAT_RRI8(X, 0x000002, 0, S, T, IMM8)

/* Store 32-bit Indirect to A(S)+4*IMM8 from A(T) */
#define XT_INS_S32I(X, S, T, IMM8) _XT_INS_FORMAT_RRI8(X, 0x006002, 0, S, T, IMM8)
/* Store 16-bit to A(S)+2*IMM8 from A(T) */
#define XT_INS_S16I(X, S, T, IMM8) _XT_INS_FORMAT_RRI8(X, 0x005002, 0, S, T, IMM8)
/* Store 8-bit to A(S)+IMM8 from A(T) */
#define XT_INS_S8I(X, S, T, IMM8)  _XT_INS_FORMAT_RRI8(X, 0x004002, 0, S, T, IMM8)

/* Cache Instructions */
#define XT_INS_IHI(X, S, IMM8) _XT_INS_FORMAT_RRI8(X, 0x0070E2, 0, S, 0, IMM8)
#define XT_INS_DHWBI(X, S, IMM8) _XT_INS_FORMAT_RRI8(X, 0x007052, 0, S, 0, IMM8)
#define XT_INS_DHWB(X, S, IMM8) _XT_INS_FORMAT_RRI8(X, 0x007042, 0, S, 0, IMM8)
#define XT_INS_ISYNC(X) (XT_ISBE(X) ? 0x000200 << 8 : 0x002000)

/* Control Instructions */
#define XT_INS_JX(X, S) (XT_ISBE(X) ? (0x050000 | ((S) << 12)) : (0x0000a0 | ((S) << 8)))
#define XT_INS_CALL0(X, IMM18) (XT_ISBE(X) ? (0x500000 | ((IMM18) & 0x3ffff)) : (0x000005 | (((IMM18) & 0x3ffff) << 6)))

/* Read Special Register */
#define XT_INS_RSR(X, SR, T) _XT_INS_FORMAT_RSR(X, 0x030000, SR, T)
/* Write Special Register */
#define XT_INS_WSR(X, SR, T) _XT_INS_FORMAT_RSR(X, 0x130000, SR, T)
/* Swap Special Register */
#define XT_INS_XSR(X, SR, T) _XT_INS_FORMAT_RSR(X, 0x610000, SR, T)

/* Rotate Window by (-8..7) */
#define XT_INS_ROTW(X, N) (XT_ISBE(X) ? ((0x000804) | (((N) & 15) << 16)) << 8 : ((0x408000) | (((N) & 15) << 4)))

/* Read User Register */
#define XT_INS_RUR(X, UR, T) _XT_INS_FORMAT_RRR(X, 0xE30000, UR, T)
/* Write User Register */
#define XT_INS_WUR(X, UR, T) _XT_INS_FORMAT_RSR(X, 0xF30000, UR, T)

/* Read Floating-Point Register */
#define XT_INS_RFR(X, FR, T) _XT_INS_FORMAT_RRR(X, 0xFA0000, ((FR << 4) | 0x4), T)
/* Write Floating-Point Register */
#define XT_INS_WFR(X, FR, T) _XT_INS_FORMAT_RRR(X, 0xFA0000, ((T << 4) | 0x5), FR)

#define XT_INS_L32E(X, R, S, T) _XT_INS_FORMAT_RRI4(X, 0x090000, 0, R, S, T)
#define XT_INS_S32E(X, R, S, T) _XT_INS_FORMAT_RRI4(X, 0x490000, 0, R, S, T)
#define XT_INS_L32E_S32E_MASK(X)   (XT_ISBE(X) ? 0xF000FF << 8 : 0xFF000F)

#define XT_INS_RFWO(X) (XT_ISBE(X) ? 0x004300 << 8 : 0x003400)
#define XT_INS_RFWU(X) (XT_ISBE(X) ? 0x005300 << 8 : 0x003500)
#define XT_INS_RFWO_RFWU_MASK(X)   (XT_ISBE(X) ? 0xFFFFFF << 8 : 0xFFFFFF)

#define XT_WATCHPOINTS_NUM_MAX  2

/* Special register number macro for DDR, PS, WB, A3, A4 registers.
 * These get used a lot so making a shortcut is useful.
 */
#define XT_SR_DDR         (xtensa_regs[XT_REG_IDX_DDR].reg_num)
#define XT_SR_PS          (xtensa_regs[XT_REG_IDX_PS].reg_num)
#define XT_SR_WB          (xtensa_regs[XT_REG_IDX_WINDOWBASE].reg_num)
#define XT_REG_A3         (xtensa_regs[XT_REG_IDX_AR3].reg_num)
#define XT_REG_A4         (xtensa_regs[XT_REG_IDX_AR4].reg_num)

#define XT_PS_REG_NUM_BASE          (0xc0U)	/* (EPS2 - 2), for adding DBGLEVEL */
#define XT_PC_REG_NUM_BASE          (0xb0U)	/* (EPC1 - 1), for adding DBGLEVEL */
#define XT_PC_REG_NUM_VIRTUAL       (0xffU)	/* Marker for computing PC (EPC[DBGLEVEL) */
#define XT_PC_DBREG_NUM_BASE        (0x20U)	/* External (i.e., GDB) access */

#define XT_SW_BREAKPOINTS_MAX_NUM       32
#define XT_HW_IBREAK_MAX_NUM            2
#define XT_HW_DBREAK_MAX_NUM            2

struct xtensa_reg_desc xtensa_regs[XT_NUM_REGS] = {
	XT_MK_REG_DESC("pc", XT_PC_REG_NUM_VIRTUAL, XT_REG_SPECIAL, 0),
	XT_MK_REG_DESC("ar0", 0x00, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar1", 0x01, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar2", 0x02, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar3", 0x03, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar4", 0x04, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar5", 0x05, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar6", 0x06, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar7", 0x07, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar8", 0x08, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar9", 0x09, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar10", 0x0A, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar11", 0x0B, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar12", 0x0C, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar13", 0x0D, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar14", 0x0E, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar15", 0x0F, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar16", 0x10, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar17", 0x11, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar18", 0x12, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar19", 0x13, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar20", 0x14, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar21", 0x15, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar22", 0x16, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar23", 0x17, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar24", 0x18, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar25", 0x19, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar26", 0x1A, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar27", 0x1B, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar28", 0x1C, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar29", 0x1D, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar30", 0x1E, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar31", 0x1F, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar32", 0x20, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar33", 0x21, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar34", 0x22, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar35", 0x23, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar36", 0x24, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar37", 0x25, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar38", 0x26, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar39", 0x27, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar40", 0x28, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar41", 0x29, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar42", 0x2A, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar43", 0x2B, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar44", 0x2C, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar45", 0x2D, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar46", 0x2E, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar47", 0x2F, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar48", 0x30, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar49", 0x31, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar50", 0x32, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar51", 0x33, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar52", 0x34, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar53", 0x35, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar54", 0x36, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar55", 0x37, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar56", 0x38, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar57", 0x39, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar58", 0x3A, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar59", 0x3B, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar60", 0x3C, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar61", 0x3D, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar62", 0x3E, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("ar63", 0x3F, XT_REG_GENERAL, 0),
	XT_MK_REG_DESC("windowbase", 0x48, XT_REG_SPECIAL, 0),
	XT_MK_REG_DESC("windowstart", 0x49, XT_REG_SPECIAL, 0),
	XT_MK_REG_DESC("ps", 0xE6, XT_REG_SPECIAL, 0),	/* PS (not mapped through EPS[]) */
	XT_MK_REG_DESC("ibreakenable", 0x60, XT_REG_SPECIAL, 0),
	XT_MK_REG_DESC("ddr", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD),
	XT_MK_REG_DESC("ibreaka0", 0x80, XT_REG_SPECIAL, 0),
	XT_MK_REG_DESC("ibreaka1", 0x81, XT_REG_SPECIAL, 0),
	XT_MK_REG_DESC("dbreaka0", 0x90, XT_REG_SPECIAL, 0),
	XT_MK_REG_DESC("dbreaka1", 0x91, XT_REG_SPECIAL, 0),
	XT_MK_REG_DESC("dbreakc0", 0xA0, XT_REG_SPECIAL, 0),
	XT_MK_REG_DESC("dbreakc1", 0xA1, XT_REG_SPECIAL, 0),
	XT_MK_REG_DESC("cpenable", 0xE0, XT_REG_SPECIAL, 0),
	XT_MK_REG_DESC("exccause", 0xE8, XT_REG_SPECIAL, 0),
	XT_MK_REG_DESC("debugcause", 0xE9, XT_REG_SPECIAL, 0),
	XT_MK_REG_DESC("icount", 0xEC, XT_REG_SPECIAL, 0),
	XT_MK_REG_DESC("icountlevel", 0xED, XT_REG_SPECIAL, 0),

	/* WARNING: For these registers, regnum points to the
	 * index of the corresponding ARx registers, NOT to
	 * the processor register number! */
	XT_MK_REG_DESC("a0", XT_REG_IDX_AR0, XT_REG_RELGEN, 0),
	XT_MK_REG_DESC("a1", XT_REG_IDX_AR1, XT_REG_RELGEN, 0),
	XT_MK_REG_DESC("a2", XT_REG_IDX_AR2, XT_REG_RELGEN, 0),
	XT_MK_REG_DESC("a3", XT_REG_IDX_AR3, XT_REG_RELGEN, 0),
	XT_MK_REG_DESC("a4", XT_REG_IDX_AR4, XT_REG_RELGEN, 0),
	XT_MK_REG_DESC("a5", XT_REG_IDX_AR5, XT_REG_RELGEN, 0),
	XT_MK_REG_DESC("a6", XT_REG_IDX_AR6, XT_REG_RELGEN, 0),
	XT_MK_REG_DESC("a7", XT_REG_IDX_AR7, XT_REG_RELGEN, 0),
	XT_MK_REG_DESC("a8", XT_REG_IDX_AR8, XT_REG_RELGEN, 0),
	XT_MK_REG_DESC("a9", XT_REG_IDX_AR9, XT_REG_RELGEN, 0),
	XT_MK_REG_DESC("a10", XT_REG_IDX_AR10, XT_REG_RELGEN, 0),
	XT_MK_REG_DESC("a11", XT_REG_IDX_AR11, XT_REG_RELGEN, 0),
	XT_MK_REG_DESC("a12", XT_REG_IDX_AR12, XT_REG_RELGEN, 0),
	XT_MK_REG_DESC("a13", XT_REG_IDX_AR13, XT_REG_RELGEN, 0),
	XT_MK_REG_DESC("a14", XT_REG_IDX_AR14, XT_REG_RELGEN, 0),
	XT_MK_REG_DESC("a15", XT_REG_IDX_AR15, XT_REG_RELGEN, 0),
};

/**
 * Types of memory used at xtensa target
 */
enum xtensa_mem_region_type {
	XTENSA_MEM_REG_IROM = 0x0,
	XTENSA_MEM_REG_IRAM,
	XTENSA_MEM_REG_DROM,
	XTENSA_MEM_REG_DRAM,
	XTENSA_MEM_REG_SRAM,
	XTENSA_MEM_REG_SROM,
	XTENSA_MEM_REGS_NUM
};

/* Register definition as union for list allocation */
union xtensa_reg_val_u {
	xtensa_reg_val_t val;
	uint8_t buf[4];
};

static const struct xtensa_keyval_info_s xt_qerr[XT_QERR_NUM] = {
	{ .chrval = "E00", .intval = ERROR_FAIL },
	{ .chrval = "E01", .intval = ERROR_FAIL },
	{ .chrval = "E02", .intval = ERROR_COMMAND_ARGUMENT_INVALID },
	{ .chrval = "E03", .intval = ERROR_FAIL },
};

/* Set to true for extra debug logging */
static const bool xtensa_extra_debug_log;

/**
 * Gets a config for the specific mem type
 */
static inline const struct xtensa_local_mem_config *xtensa_get_mem_config(
	struct xtensa *xtensa,
	enum xtensa_mem_region_type type)
{
	switch (type) {
	case XTENSA_MEM_REG_IROM:
		return &xtensa->core_config->irom;
	case XTENSA_MEM_REG_IRAM:
		return &xtensa->core_config->iram;
	case XTENSA_MEM_REG_DROM:
		return &xtensa->core_config->drom;
	case XTENSA_MEM_REG_DRAM:
		return &xtensa->core_config->dram;
	case XTENSA_MEM_REG_SRAM:
		return &xtensa->core_config->sram;
	case XTENSA_MEM_REG_SROM:
		return &xtensa->core_config->srom;
	default:
		return NULL;
	}
}

/**
 * Extracts an exact xtensa_local_mem_region_config from xtensa_local_mem_config
 * for a given address
 * Returns NULL if nothing found
 */
static inline const struct xtensa_local_mem_region_config *xtensa_memory_region_find(
	const struct xtensa_local_mem_config *mem,
	target_addr_t address)
{
	for (unsigned int i = 0; i < mem->count; i++) {
		const struct xtensa_local_mem_region_config *region = &mem->regions[i];
		if (address >= region->base && address < (region->base + region->size))
			return region;
	}
	return NULL;
}

/**
 * Returns a corresponding xtensa_local_mem_region_config from the xtensa target
 * for a given address
 * Returns NULL if nothing found
 */
static inline const struct xtensa_local_mem_region_config *xtensa_target_memory_region_find(
	struct xtensa *xtensa,
	target_addr_t address)
{
	const struct xtensa_local_mem_region_config *result;
	const struct xtensa_local_mem_config *mcgf;
	for (unsigned int mtype = 0; mtype < XTENSA_MEM_REGS_NUM; mtype++) {
		mcgf = xtensa_get_mem_config(xtensa, mtype);
		result = xtensa_memory_region_find(mcgf, address);
		if (result)
			return result;
	}
	return NULL;
}

static inline bool xtensa_is_cacheable(const struct xtensa_cache_config *cache,
	const struct xtensa_local_mem_config *mem,
	target_addr_t address)
{
	if (!cache->size)
		return false;
	return xtensa_memory_region_find(mem, address);
}

static inline bool xtensa_is_icacheable(struct xtensa *xtensa, target_addr_t address)
{
	return xtensa_is_cacheable(&xtensa->core_config->icache, &xtensa->core_config->iram, address) ||
	       xtensa_is_cacheable(&xtensa->core_config->icache, &xtensa->core_config->irom, address) ||
	       xtensa_is_cacheable(&xtensa->core_config->icache, &xtensa->core_config->sram, address) ||
	       xtensa_is_cacheable(&xtensa->core_config->icache, &xtensa->core_config->srom, address);
}

static inline bool xtensa_is_dcacheable(struct xtensa *xtensa, target_addr_t address)
{
	return xtensa_is_cacheable(&xtensa->core_config->dcache, &xtensa->core_config->dram, address) ||
	       xtensa_is_cacheable(&xtensa->core_config->dcache, &xtensa->core_config->drom, address) ||
	       xtensa_is_cacheable(&xtensa->core_config->dcache, &xtensa->core_config->sram, address) ||
	       xtensa_is_cacheable(&xtensa->core_config->dcache, &xtensa->core_config->srom, address);
}

static int xtensa_core_reg_get(struct reg *reg)
{
	/* We don't need this because we read all registers on halt anyway. */
	struct xtensa *xtensa = (struct xtensa *)reg->arch_info;
	struct target *target = xtensa->target;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;
	if (!reg->exist) {
		if (strncmp(reg->name, "?0x", 3) == 0) {
			unsigned int regnum = strtoul(reg->name + 1, 0, 0);
			LOG_WARNING("Read unknown register 0x%04x ignored", regnum);
			return ERROR_OK;
		}
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}
	return ERROR_OK;
}

static int xtensa_core_reg_set(struct reg *reg, uint8_t *buf)
{
	struct xtensa *xtensa = (struct xtensa *)reg->arch_info;
	struct target *target = xtensa->target;

	assert(reg->size <= 64 && "up to 64-bit regs are supported only!");
	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	if (!reg->exist) {
		if (strncmp(reg->name, "?0x", 3) == 0) {
			unsigned int regnum = strtoul(reg->name + 1, 0, 0);
			LOG_WARNING("Write unknown register 0x%04x ignored", regnum);
			return ERROR_OK;
		}
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	buf_cpy(buf, reg->value, reg->size);

	if (xtensa->core_config->windowed) {
		/* If the user updates a potential scratch register, track for conflicts */
		for (enum xtensa_ar_scratch_set_e s = 0; s < XT_AR_SCRATCH_NUM; s++) {
			if (strcmp(reg->name, xtensa->scratch_ars[s].chrval) == 0) {
				LOG_DEBUG("Scratch reg %s [0x%08" PRIx32 "] set from gdb", reg->name,
					buf_get_u32(reg->value, 0, 32));
				LOG_DEBUG("scratch_ars mapping: a3/%s, a4/%s",
					xtensa->scratch_ars[XT_AR_SCRATCH_AR3].chrval,
					xtensa->scratch_ars[XT_AR_SCRATCH_AR4].chrval);
				xtensa->scratch_ars[s].intval = true;
				break;
			}
		}
	}
	reg->dirty = true;
	reg->valid = true;

	return ERROR_OK;
}

static const struct reg_arch_type xtensa_reg_type = {
	.get = xtensa_core_reg_get,
	.set = xtensa_core_reg_set,
};

/* Convert a register index that's indexed relative to windowbase, to the real address. */
static enum xtensa_reg_id xtensa_windowbase_offset_to_canonical(struct xtensa *xtensa,
	enum xtensa_reg_id reg_idx,
	int windowbase)
{
	unsigned int idx;
	if (reg_idx >= XT_REG_IDX_AR0 && reg_idx <= XT_REG_IDX_ARLAST) {
		idx = reg_idx - XT_REG_IDX_AR0;
	} else if (reg_idx >= XT_REG_IDX_A0 && reg_idx <= XT_REG_IDX_A15) {
		idx = reg_idx - XT_REG_IDX_A0;
	} else {
		LOG_ERROR("Error: can't convert register %d to non-windowbased register!", reg_idx);
		return -1;
	}
	return ((idx + windowbase * 4) & (xtensa->core_config->aregs_num - 1)) + XT_REG_IDX_AR0;
}

static enum xtensa_reg_id xtensa_canonical_to_windowbase_offset(struct xtensa *xtensa,
	enum xtensa_reg_id reg_idx,
	int windowbase)
{
	return xtensa_windowbase_offset_to_canonical(xtensa, reg_idx, -windowbase);
}

static void xtensa_mark_register_dirty(struct xtensa *xtensa, enum xtensa_reg_id reg_idx)
{
	struct reg *reg_list = xtensa->core_cache->reg_list;
	reg_list[reg_idx].dirty = true;
}

static void xtensa_queue_exec_ins(struct xtensa *xtensa, uint32_t ins)
{
	xtensa_queue_dbg_reg_write(xtensa, XDMREG_DIR0EXEC, ins);
}

static void xtensa_queue_exec_ins_wide(struct xtensa *xtensa, uint8_t *ops, uint8_t oplen)
{
	const int max_oplen = 64;	/* 8 DIRx regs: max width 64B */
	if ((oplen > 0) && (oplen <= max_oplen)) {
		uint8_t ops_padded[max_oplen];
		memcpy(ops_padded, ops, oplen);
		memset(ops_padded + oplen, 0, max_oplen - oplen);
		unsigned int oplenw = DIV_ROUND_UP(oplen, sizeof(uint32_t));
		for (int32_t i = oplenw - 1; i > 0; i--)
			xtensa_queue_dbg_reg_write(xtensa,
				XDMREG_DIR0 + i,
				target_buffer_get_u32(xtensa->target, &ops_padded[sizeof(uint32_t)*i]));
		/* Write DIR0EXEC last */
		xtensa_queue_dbg_reg_write(xtensa,
			XDMREG_DIR0EXEC,
			target_buffer_get_u32(xtensa->target, &ops_padded[0]));
	}
}

static int xtensa_queue_pwr_reg_write(struct xtensa *xtensa, unsigned int reg, uint32_t data)
{
	struct xtensa_debug_module *dm = &xtensa->dbg_mod;
	return dm->pwr_ops->queue_reg_write(dm, reg, data);
}

/* NOTE: Assumes A3 has already been saved */
static int xtensa_window_state_save(struct target *target, uint32_t *woe)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int woe_dis;
	uint8_t woe_buf[4];

	if (xtensa->core_config->windowed) {
		/* Save PS (LX) and disable window overflow exceptions prior to AR save */
		xtensa_queue_exec_ins(xtensa, XT_INS_RSR(xtensa, XT_SR_PS, XT_REG_A3));
		xtensa_queue_exec_ins(xtensa, XT_INS_WSR(xtensa, XT_SR_DDR, XT_REG_A3));
		xtensa_queue_dbg_reg_read(xtensa, XDMREG_DDR, woe_buf);
		int res = xtensa_dm_queue_execute(&xtensa->dbg_mod);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to read PS (%d)!", res);
			return res;
		}
		xtensa_core_status_check(target);
		*woe = buf_get_u32(woe_buf, 0, 32);
		woe_dis = *woe & ~XT_PS_WOE_MSK;
		LOG_DEBUG("Clearing PS.WOE (0x%08" PRIx32 " -> 0x%08" PRIx32 ")", *woe, woe_dis);
		xtensa_queue_dbg_reg_write(xtensa, XDMREG_DDR, woe_dis);
		xtensa_queue_exec_ins(xtensa, XT_INS_RSR(xtensa, XT_SR_DDR, XT_REG_A3));
		xtensa_queue_exec_ins(xtensa, XT_INS_WSR(xtensa, XT_SR_PS, XT_REG_A3));
	}
	return ERROR_OK;
}

/* NOTE: Assumes A3 has already been saved */
static void xtensa_window_state_restore(struct target *target, uint32_t woe)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	if (xtensa->core_config->windowed) {
		/* Restore window overflow exception state */
		xtensa_queue_dbg_reg_write(xtensa, XDMREG_DDR, woe);
		xtensa_queue_exec_ins(xtensa, XT_INS_RSR(xtensa, XT_SR_DDR, XT_REG_A3));
		xtensa_queue_exec_ins(xtensa, XT_INS_WSR(xtensa, XT_SR_PS, XT_REG_A3));
		LOG_DEBUG("Restored PS.WOE (0x%08" PRIx32 ")", woe);
	}
}

static bool xtensa_reg_is_readable(int flags, int cpenable)
{
	if (flags & XT_REGF_NOREAD)
		return false;
	if ((flags & XT_REGF_COPROC0) && (cpenable & BIT(0)) == 0)
		return false;
	return true;
}

static bool xtensa_scratch_regs_fixup(struct xtensa *xtensa, struct reg *reg_list, int i, int j, int a_idx, int ar_idx)
{
	int a_name = (a_idx == XT_AR_SCRATCH_A3) ? 3 : 4;
	if (xtensa->scratch_ars[a_idx].intval && !xtensa->scratch_ars[ar_idx].intval) {
		LOG_DEBUG("AR conflict: a%d -> ar%d", a_name, j - XT_REG_IDX_AR0);
		memcpy(reg_list[j].value, reg_list[i].value, sizeof(xtensa_reg_val_t));
	} else {
		LOG_DEBUG("AR conflict: ar%d -> a%d", j - XT_REG_IDX_AR0, a_name);
		memcpy(reg_list[i].value, reg_list[j].value, sizeof(xtensa_reg_val_t));
	}
	return xtensa->scratch_ars[a_idx].intval && xtensa->scratch_ars[ar_idx].intval;
}

static int xtensa_write_dirty_registers(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int res;
	xtensa_reg_val_t regval, windowbase = 0;
	bool scratch_reg_dirty = false, delay_cpenable = false;
	struct reg *reg_list = xtensa->core_cache->reg_list;
	unsigned int reg_list_size = xtensa->core_cache->num_regs;
	bool preserve_a3 = false;
	uint8_t a3_buf[4];
	xtensa_reg_val_t a3 = 0, woe;

	LOG_TARGET_DEBUG(target, "start");

	/* We need to write the dirty registers in the cache list back to the processor.
	 * Start by writing the SFR/user registers. */
	for (unsigned int i = 0; i < reg_list_size; i++) {
		struct xtensa_reg_desc *rlist = (i < XT_NUM_REGS) ? xtensa_regs : xtensa->optregs;
		unsigned int ridx = (i < XT_NUM_REGS) ? i : i - XT_NUM_REGS;
		if (reg_list[i].dirty) {
			if (rlist[ridx].type == XT_REG_SPECIAL ||
				rlist[ridx].type == XT_REG_USER ||
				rlist[ridx].type == XT_REG_FR) {
				scratch_reg_dirty = true;
				if (i == XT_REG_IDX_CPENABLE) {
					delay_cpenable = true;
					continue;
				}
				regval = xtensa_reg_get(target, i);
				LOG_TARGET_DEBUG(target, "Writing back reg %s (%d) val %08" PRIX32,
					reg_list[i].name,
					rlist[ridx].reg_num,
					regval);
				xtensa_queue_dbg_reg_write(xtensa, XDMREG_DDR, regval);
				xtensa_queue_exec_ins(xtensa, XT_INS_RSR(xtensa, XT_SR_DDR, XT_REG_A3));
				if (reg_list[i].exist) {
					unsigned int reg_num = rlist[ridx].reg_num;
					if (rlist[ridx].type == XT_REG_USER) {
						xtensa_queue_exec_ins(xtensa, XT_INS_WUR(xtensa, reg_num, XT_REG_A3));
					} else if (rlist[ridx].type == XT_REG_FR) {
						xtensa_queue_exec_ins(xtensa, XT_INS_WFR(xtensa, reg_num, XT_REG_A3));
					} else {/*SFR */
						if (reg_num == XT_PC_REG_NUM_VIRTUAL)
							/* reg number of PC for debug interrupt depends on NDEBUGLEVEL
							 **/
							reg_num =
								(XT_PC_REG_NUM_BASE +
								xtensa->core_config->debug.irq_level);
						xtensa_queue_exec_ins(xtensa, XT_INS_WSR(xtensa, reg_num, XT_REG_A3));
					}
				}
				reg_list[i].dirty = false;
			}
		}
	}
	if (scratch_reg_dirty)
		xtensa_mark_register_dirty(xtensa, XT_REG_IDX_A3);
	if (delay_cpenable) {
		regval = xtensa_reg_get(target, XT_REG_IDX_CPENABLE);
		LOG_TARGET_DEBUG(target, "Writing back reg cpenable (224) val %08" PRIX32, regval);
		xtensa_queue_dbg_reg_write(xtensa, XDMREG_DDR, regval);
		xtensa_queue_exec_ins(xtensa, XT_INS_RSR(xtensa, XT_SR_DDR, XT_REG_A3));
		xtensa_queue_exec_ins(xtensa, XT_INS_WSR(xtensa,
				xtensa_regs[XT_REG_IDX_CPENABLE].reg_num,
				XT_REG_A3));
		reg_list[XT_REG_IDX_CPENABLE].dirty = false;
	}

	preserve_a3 = (xtensa->core_config->windowed);
	if (preserve_a3) {
		/* Save (windowed) A3 for scratch use */
		xtensa_queue_exec_ins(xtensa, XT_INS_WSR(xtensa, XT_SR_DDR, XT_REG_A3));
		xtensa_queue_dbg_reg_read(xtensa, XDMREG_DDR, a3_buf);
		res = xtensa_dm_queue_execute(&xtensa->dbg_mod);
		if (res != ERROR_OK)
			return res;
		xtensa_core_status_check(target);
		a3 = buf_get_u32(a3_buf, 0, 32);
	}

	if (xtensa->core_config->windowed) {
		res = xtensa_window_state_save(target, &woe);
		if (res != ERROR_OK)
			return res;
		/* Grab the windowbase, we need it. */
		windowbase = xtensa_reg_get(target, XT_REG_IDX_WINDOWBASE);
		/* Check if there are mismatches between the ARx and corresponding Ax registers.
		 * When the user sets a register on a windowed config, xt-gdb may set the ARx
		 * register directly.  Thus we take ARx as priority over Ax if both are dirty
		 * and it's unclear if the user set one over the other explicitly.
		 */
		for (unsigned int i = XT_REG_IDX_A0; i <= XT_REG_IDX_A15; i++) {
			unsigned int j = xtensa_windowbase_offset_to_canonical(xtensa, i, windowbase);
			if (reg_list[i].dirty && reg_list[j].dirty) {
				if (memcmp(reg_list[i].value, reg_list[j].value, sizeof(xtensa_reg_val_t)) != 0) {
					bool show_warning = true;
					if (i == XT_REG_IDX_A3)
						show_warning = xtensa_scratch_regs_fixup(xtensa,
							reg_list, i, j, XT_AR_SCRATCH_A3, XT_AR_SCRATCH_AR3);
					else if (i == XT_REG_IDX_A4)
						show_warning = xtensa_scratch_regs_fixup(xtensa,
							reg_list, i, j, XT_AR_SCRATCH_A4, XT_AR_SCRATCH_AR4);
					if (show_warning)
						LOG_WARNING(
							"Warning: Both A%d [0x%08" PRIx32
							"] as well as its underlying physical register "
							"(AR%d) [0x%08" PRIx32 "] are dirty and differ in value",
							i - XT_REG_IDX_A0,
							buf_get_u32(reg_list[i].value, 0, 32),
							j - XT_REG_IDX_AR0,
							buf_get_u32(reg_list[j].value, 0, 32));
				}
			}
		}
	}

	/* Write A0-A16. */
	for (unsigned int i = 0; i < 16; i++) {
		if (reg_list[XT_REG_IDX_A0 + i].dirty) {
			regval = xtensa_reg_get(target, XT_REG_IDX_A0 + i);
			LOG_TARGET_DEBUG(target, "Writing back reg %s value %08" PRIX32 ", num =%i",
				xtensa_regs[XT_REG_IDX_A0 + i].name,
				regval,
				xtensa_regs[XT_REG_IDX_A0 + i].reg_num);
			xtensa_queue_dbg_reg_write(xtensa, XDMREG_DDR, regval);
			xtensa_queue_exec_ins(xtensa, XT_INS_RSR(xtensa, XT_SR_DDR, i));
			reg_list[XT_REG_IDX_A0 + i].dirty = false;
			if (i == 3) {
				/* Avoid stomping A3 during restore at end of function */
				a3 = regval;
			}
		}
	}

	if (xtensa->core_config->windowed) {
		/* Now write AR registers */
		for (unsigned int j = 0; j < XT_REG_IDX_ARLAST; j += 16) {
			/* Write the 16 registers we can see */
			for (unsigned int i = 0; i < 16; i++) {
				if (i + j < xtensa->core_config->aregs_num) {
					enum xtensa_reg_id realadr =
						xtensa_windowbase_offset_to_canonical(xtensa, XT_REG_IDX_AR0 + i + j,
						windowbase);
					/* Write back any dirty un-windowed registers */
					if (reg_list[realadr].dirty) {
						regval = xtensa_reg_get(target, realadr);
						LOG_TARGET_DEBUG(
							target,
							"Writing back reg %s value %08" PRIX32 ", num =%i",
							xtensa_regs[realadr].name,
							regval,
							xtensa_regs[realadr].reg_num);
						xtensa_queue_dbg_reg_write(xtensa, XDMREG_DDR, regval);
						xtensa_queue_exec_ins(xtensa,
							XT_INS_RSR(xtensa, XT_SR_DDR,
								xtensa_regs[XT_REG_IDX_AR0 + i].reg_num));
						reg_list[realadr].dirty = false;
						if ((i + j) == 3)
							/* Avoid stomping AR during A3 restore at end of function */
							a3 = regval;
					}
				}
			}
			/*Now rotate the window so we'll see the next 16 registers. The final rotate
			 * will wraparound, */
			/*leaving us in the state we were. */
			xtensa_queue_exec_ins(xtensa, XT_INS_ROTW(xtensa, 4));
		}

		xtensa_window_state_restore(target, woe);

		for (enum xtensa_ar_scratch_set_e s = 0; s < XT_AR_SCRATCH_NUM; s++)
			xtensa->scratch_ars[s].intval = false;
	}

	if (preserve_a3) {
		xtensa_queue_dbg_reg_write(xtensa, XDMREG_DDR, a3);
		xtensa_queue_exec_ins(xtensa, XT_INS_RSR(xtensa, XT_SR_DDR, XT_REG_A3));
	}

	res = xtensa_dm_queue_execute(&xtensa->dbg_mod);
	xtensa_core_status_check(target);

	return res;
}

static inline bool xtensa_is_stopped(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	return xtensa->dbg_mod.core_status.dsr & OCDDSR_STOPPED;
}

int xtensa_examine(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	unsigned int cmd = PWRCTL_DEBUGWAKEUP(xtensa) | PWRCTL_MEMWAKEUP(xtensa) | PWRCTL_COREWAKEUP(xtensa);

	LOG_DEBUG("coreid = %d", target->coreid);

	if (xtensa->core_config->core_type == XT_UNDEF) {
		LOG_ERROR("XTensa core not configured; is xtensa-core-openocd.cfg missing?");
		return ERROR_FAIL;
	}

	xtensa_queue_pwr_reg_write(xtensa, XDMREG_PWRCTL, cmd);
	xtensa_queue_pwr_reg_write(xtensa, XDMREG_PWRCTL, cmd | PWRCTL_JTAGDEBUGUSE(xtensa));
	xtensa_dm_queue_enable(&xtensa->dbg_mod);
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	int res = xtensa_dm_queue_execute(&xtensa->dbg_mod);
	if (res != ERROR_OK)
		return res;
	if (!xtensa_dm_is_online(&xtensa->dbg_mod)) {
		LOG_ERROR("Unexpected OCD_ID = %08" PRIx32, xtensa->dbg_mod.device_id);
		return ERROR_TARGET_FAILURE;
	}
	LOG_DEBUG("OCD_ID = %08" PRIx32, xtensa->dbg_mod.device_id);
	if (!target_was_examined(target))
		target_set_examined(target);
	xtensa_smpbreak_write(xtensa, xtensa->smp_break);
	return ERROR_OK;
}

int xtensa_wakeup(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	unsigned int cmd = PWRCTL_DEBUGWAKEUP(xtensa) | PWRCTL_MEMWAKEUP(xtensa) | PWRCTL_COREWAKEUP(xtensa);

	if (xtensa->reset_asserted)
		cmd |= PWRCTL_CORERESET(xtensa);
	xtensa_queue_pwr_reg_write(xtensa, XDMREG_PWRCTL, cmd);
	/* TODO: can we join this with the write above? */
	xtensa_queue_pwr_reg_write(xtensa, XDMREG_PWRCTL, cmd | PWRCTL_JTAGDEBUGUSE(xtensa));
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	return xtensa_dm_queue_execute(&xtensa->dbg_mod);
}

int xtensa_smpbreak_write(struct xtensa *xtensa, uint32_t set)
{
	uint32_t dsr_data = 0x00110000;
	uint32_t clear = (set | OCDDCR_ENABLEOCD) ^
		(OCDDCR_BREAKINEN | OCDDCR_BREAKOUTEN | OCDDCR_RUNSTALLINEN |
		OCDDCR_DEBUGMODEOUTEN | OCDDCR_ENABLEOCD);

	LOG_TARGET_DEBUG(xtensa->target, "write smpbreak set=0x%" PRIx32 " clear=0x%" PRIx32, set, clear);
	xtensa_queue_dbg_reg_write(xtensa, XDMREG_DCRSET, set | OCDDCR_ENABLEOCD);
	xtensa_queue_dbg_reg_write(xtensa, XDMREG_DCRCLR, clear);
	xtensa_queue_dbg_reg_write(xtensa, XDMREG_DSR, dsr_data);
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	return xtensa_dm_queue_execute(&xtensa->dbg_mod);
}

int xtensa_smpbreak_set(struct target *target, uint32_t set)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int res = ERROR_OK;

	xtensa->smp_break = set;
	if (target_was_examined(target))
		res = xtensa_smpbreak_write(xtensa, xtensa->smp_break);
	LOG_TARGET_DEBUG(target, "set smpbreak=%" PRIx32 ", state=%i", set, target->state);
	return res;
}

int xtensa_smpbreak_read(struct xtensa *xtensa, uint32_t *val)
{
	uint8_t dcr_buf[sizeof(uint32_t)];

	xtensa_queue_dbg_reg_read(xtensa, XDMREG_DCRSET, dcr_buf);
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	int res = xtensa_dm_queue_execute(&xtensa->dbg_mod);
	*val = buf_get_u32(dcr_buf, 0, 32);

	return res;
}

int xtensa_smpbreak_get(struct target *target, uint32_t *val)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	*val = xtensa->smp_break;
	return ERROR_OK;
}

static inline xtensa_reg_val_t xtensa_reg_get_value(struct reg *reg)
{
	return buf_get_u32(reg->value, 0, 32);
}

static inline void xtensa_reg_set_value(struct reg *reg, xtensa_reg_val_t value)
{
	buf_set_u32(reg->value, 0, 32, value);
	reg->dirty = true;
}

int xtensa_core_status_check(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int res, needclear = 0;

	xtensa_dm_core_status_read(&xtensa->dbg_mod);
	xtensa_dsr_t dsr = xtensa_dm_core_status_get(&xtensa->dbg_mod);
	LOG_TARGET_DEBUG(target, "DSR (%08" PRIX32 ")", dsr);
	if (dsr & OCDDSR_EXECBUSY) {
		if (!xtensa->suppress_dsr_errors)
			LOG_TARGET_ERROR(target, "DSR (%08" PRIX32 ") indicates target still busy!", dsr);
		needclear = 1;
	}
	if (dsr & OCDDSR_EXECEXCEPTION) {
		if (!xtensa->suppress_dsr_errors)
			LOG_TARGET_ERROR(target,
				"DSR (%08" PRIX32 ") indicates DIR instruction generated an exception!",
				dsr);
		needclear = 1;
	}
	if (dsr & OCDDSR_EXECOVERRUN) {
		if (!xtensa->suppress_dsr_errors)
			LOG_TARGET_ERROR(target,
				"DSR (%08" PRIX32 ") indicates DIR instruction generated an overrun!",
				dsr);
		needclear = 1;
	}
	if (needclear) {
		res = xtensa_dm_core_status_clear(&xtensa->dbg_mod,
			OCDDSR_EXECEXCEPTION | OCDDSR_EXECOVERRUN);
		if (res != ERROR_OK && !xtensa->suppress_dsr_errors)
			LOG_TARGET_ERROR(target, "clearing DSR failed!");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

xtensa_reg_val_t xtensa_reg_get(struct target *target, enum xtensa_reg_id reg_id)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	struct reg *reg = &xtensa->core_cache->reg_list[reg_id];
	return xtensa_reg_get_value(reg);
}

void xtensa_reg_set(struct target *target, enum xtensa_reg_id reg_id, xtensa_reg_val_t value)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	struct reg *reg = &xtensa->core_cache->reg_list[reg_id];
	if (xtensa_reg_get_value(reg) == value)
		return;
	xtensa_reg_set_value(reg, value);
}

/* Set Ax (XT_REG_RELGEN) register along with its underlying ARx (XT_REG_GENERAL) */
void xtensa_reg_set_deep_relgen(struct target *target, enum xtensa_reg_id a_idx, xtensa_reg_val_t value)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	uint32_t windowbase = (xtensa->core_config->windowed ?
		xtensa_reg_get(target, XT_REG_IDX_WINDOWBASE) : 0);
	int ar_idx = xtensa_windowbase_offset_to_canonical(xtensa, a_idx, windowbase);
	xtensa_reg_set(target, a_idx, value);
	xtensa_reg_set(target, ar_idx, value);
}

/* Read cause for entering halted state; return bitmask in DEBUGCAUSE_* format */
uint32_t xtensa_cause_get(struct target *target)
{
	return xtensa_reg_get(target, XT_REG_IDX_DEBUGCAUSE);
}

void xtensa_cause_clear(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	xtensa_reg_set(target, XT_REG_IDX_DEBUGCAUSE, 0);
	xtensa->core_cache->reg_list[XT_REG_IDX_DEBUGCAUSE].dirty = false;
}

int xtensa_assert_reset(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);

	LOG_TARGET_DEBUG(target, "target_number=%i, begin", target->target_number);
	xtensa_queue_pwr_reg_write(xtensa,
		XDMREG_PWRCTL,
		PWRCTL_JTAGDEBUGUSE(xtensa) | PWRCTL_DEBUGWAKEUP(xtensa) | PWRCTL_MEMWAKEUP(xtensa) |
		PWRCTL_COREWAKEUP(xtensa) | PWRCTL_CORERESET(xtensa));
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	int res = xtensa_dm_queue_execute(&xtensa->dbg_mod);
	if (res != ERROR_OK)
		return res;

	/* registers are now invalid */
	xtensa->reset_asserted = true;
	register_cache_invalidate(xtensa->core_cache);
	target->state = TARGET_RESET;
	return ERROR_OK;
}

int xtensa_deassert_reset(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);

	LOG_TARGET_DEBUG(target, "halt=%d", target->reset_halt);
	if (target->reset_halt)
		xtensa_queue_dbg_reg_write(xtensa,
			XDMREG_DCRSET,
			OCDDCR_ENABLEOCD | OCDDCR_DEBUGINTERRUPT);
	xtensa_queue_pwr_reg_write(xtensa,
		XDMREG_PWRCTL,
		PWRCTL_JTAGDEBUGUSE(xtensa) | PWRCTL_DEBUGWAKEUP(xtensa) | PWRCTL_MEMWAKEUP(xtensa) |
		PWRCTL_COREWAKEUP(xtensa));
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	int res = xtensa_dm_queue_execute(&xtensa->dbg_mod);
	if (res != ERROR_OK)
		return res;
	target->state = TARGET_RUNNING;
	xtensa->reset_asserted = false;
	return res;
}

int xtensa_soft_reset_halt(struct target *target)
{
	LOG_TARGET_DEBUG(target, "begin");
	return xtensa_assert_reset(target);
}

int xtensa_fetch_all_regs(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	struct reg *reg_list = xtensa->core_cache->reg_list;
	unsigned int reg_list_size = xtensa->core_cache->num_regs;
	xtensa_reg_val_t cpenable = 0, windowbase = 0, a3;
	uint32_t woe;
	uint8_t a3_buf[4];
	bool debug_dsrs = !xtensa->regs_fetched || LOG_LEVEL_IS(LOG_LVL_DEBUG);

	union xtensa_reg_val_u *regvals = calloc(reg_list_size, sizeof(*regvals));
	if (!regvals) {
		LOG_TARGET_ERROR(target, "unable to allocate memory for regvals!");
		return ERROR_FAIL;
	}
	union xtensa_reg_val_u *dsrs = calloc(reg_list_size, sizeof(*dsrs));
	if (!dsrs) {
		LOG_TARGET_ERROR(target, "unable to allocate memory for dsrs!");
		free(regvals);
		return ERROR_FAIL;
	}

	LOG_TARGET_DEBUG(target, "start");

	/* Save (windowed) A3 so cache matches physical AR3; A3 usable as scratch */
	xtensa_queue_exec_ins(xtensa, XT_INS_WSR(xtensa, XT_SR_DDR, XT_REG_A3));
	xtensa_queue_dbg_reg_read(xtensa, XDMREG_DDR, a3_buf);
	int res = xtensa_window_state_save(target, &woe);
	if (res != ERROR_OK)
		goto xtensa_fetch_all_regs_done;

	/* Assume the CPU has just halted. We now want to fill the register cache with all the
	 * register contents GDB needs. For speed, we pipeline all the read operations, execute them
	 * in one go, then sort everything out from the regvals variable. */

	/* Start out with AREGS; we can reach those immediately. Grab them per 16 registers. */
	for (unsigned int j = 0; j < XT_AREGS_NUM_MAX; j += 16) {
		/*Grab the 16 registers we can see */
		for (unsigned int i = 0; i < 16; i++) {
			if (i + j < xtensa->core_config->aregs_num) {
				xtensa_queue_exec_ins(xtensa,
					XT_INS_WSR(xtensa, XT_SR_DDR, xtensa_regs[XT_REG_IDX_AR0 + i].reg_num));
				xtensa_queue_dbg_reg_read(xtensa, XDMREG_DDR,
					regvals[XT_REG_IDX_AR0 + i + j].buf);
				if (debug_dsrs)
					xtensa_queue_dbg_reg_read(xtensa, XDMREG_DSR,
						dsrs[XT_REG_IDX_AR0 + i + j].buf);
			}
		}
		if (xtensa->core_config->windowed)
			/* Now rotate the window so we'll see the next 16 registers. The final rotate
			 * will wraparound, */
			/* leaving us in the state we were. */
			xtensa_queue_exec_ins(xtensa, XT_INS_ROTW(xtensa, 4));
	}
	xtensa_window_state_restore(target, woe);

	if (xtensa->core_config->coproc) {
		/* As the very first thing after AREGS, go grab CPENABLE */
		xtensa_queue_exec_ins(xtensa, XT_INS_RSR(xtensa, xtensa_regs[XT_REG_IDX_CPENABLE].reg_num, XT_REG_A3));
		xtensa_queue_exec_ins(xtensa, XT_INS_WSR(xtensa, XT_SR_DDR, XT_REG_A3));
		xtensa_queue_dbg_reg_read(xtensa, XDMREG_DDR, regvals[XT_REG_IDX_CPENABLE].buf);
	}
	res = xtensa_dm_queue_execute(&xtensa->dbg_mod);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to read ARs (%d)!", res);
		goto xtensa_fetch_all_regs_done;
	}
	xtensa_core_status_check(target);

	a3 = buf_get_u32(a3_buf, 0, 32);

	if (xtensa->core_config->coproc) {
		cpenable = buf_get_u32(regvals[XT_REG_IDX_CPENABLE].buf, 0, 32);

		/* Enable all coprocessors (by setting all bits in CPENABLE) so we can read FP and user registers. */
		xtensa_queue_dbg_reg_write(xtensa, XDMREG_DDR, 0xffffffff);
		xtensa_queue_exec_ins(xtensa, XT_INS_RSR(xtensa, XT_SR_DDR, XT_REG_A3));
		xtensa_queue_exec_ins(xtensa, XT_INS_WSR(xtensa, xtensa_regs[XT_REG_IDX_CPENABLE].reg_num, XT_REG_A3));

		/* Save CPENABLE; flag dirty later (when regcache updated) so original value is always restored */
		LOG_TARGET_DEBUG(target, "CPENABLE: was 0x%" PRIx32 ", all enabled", cpenable);
		xtensa_reg_set(target, XT_REG_IDX_CPENABLE, cpenable);
	}
	/* We're now free to use any of A0-A15 as scratch registers
	 * Grab the SFRs and user registers first. We use A3 as a scratch register. */
	for (unsigned int i = 0; i < reg_list_size; i++) {
		struct xtensa_reg_desc *rlist = (i < XT_NUM_REGS) ? xtensa_regs : xtensa->optregs;
		unsigned int ridx = (i < XT_NUM_REGS) ? i : i - XT_NUM_REGS;
		if (xtensa_reg_is_readable(rlist[ridx].flags, cpenable) && rlist[ridx].exist) {
			bool reg_fetched = true;
			unsigned int reg_num = rlist[ridx].reg_num;
			switch (rlist[ridx].type) {
			case XT_REG_USER:
				xtensa_queue_exec_ins(xtensa, XT_INS_RUR(xtensa, reg_num, XT_REG_A3));
				break;
			case XT_REG_FR:
				xtensa_queue_exec_ins(xtensa, XT_INS_RFR(xtensa, reg_num, XT_REG_A3));
				break;
			case XT_REG_SPECIAL:
				if (reg_num == XT_PC_REG_NUM_VIRTUAL) {
					/* reg number of PC for debug interrupt depends on NDEBUGLEVEL */
					reg_num = (XT_PC_REG_NUM_BASE + xtensa->core_config->debug.irq_level);
				} else if (reg_num == xtensa_regs[XT_REG_IDX_PS].reg_num) {
					/* reg number of PS for debug interrupt depends on NDEBUGLEVEL */
					reg_num = (XT_PS_REG_NUM_BASE + xtensa->core_config->debug.irq_level);
				} else if (reg_num == xtensa_regs[XT_REG_IDX_CPENABLE].reg_num) {
					/* CPENABLE already read/updated; don't re-read */
					reg_fetched = false;
					break;
				}
				xtensa_queue_exec_ins(xtensa, XT_INS_RSR(xtensa, reg_num, XT_REG_A3));
				break;
			default:
				reg_fetched = false;
			}
			if (reg_fetched) {
				xtensa_queue_exec_ins(xtensa, XT_INS_WSR(xtensa, XT_SR_DDR, XT_REG_A3));
				xtensa_queue_dbg_reg_read(xtensa, XDMREG_DDR, regvals[i].buf);
				if (debug_dsrs)
					xtensa_queue_dbg_reg_read(xtensa, XDMREG_DSR, dsrs[i].buf);
			}
		}
	}
	/* Ok, send the whole mess to the CPU. */
	res = xtensa_dm_queue_execute(&xtensa->dbg_mod);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to fetch AR regs!");
		goto xtensa_fetch_all_regs_done;
	}
	xtensa_core_status_check(target);

	if (debug_dsrs) {
		/* DSR checking: follows order in which registers are requested. */
		for (unsigned int i = 0; i < reg_list_size; i++) {
			struct xtensa_reg_desc *rlist = (i < XT_NUM_REGS) ? xtensa_regs : xtensa->optregs;
			unsigned int ridx = (i < XT_NUM_REGS) ? i : i - XT_NUM_REGS;
			if (xtensa_reg_is_readable(rlist[ridx].flags, cpenable) && rlist[ridx].exist &&
				(rlist[ridx].type != XT_REG_DEBUG) &&
				(rlist[ridx].type != XT_REG_RELGEN) &&
				(rlist[ridx].type != XT_REG_TIE) &&
				(rlist[ridx].type != XT_REG_OTHER)) {
				if (buf_get_u32(dsrs[i].buf, 0, 32) & OCDDSR_EXECEXCEPTION) {
					LOG_ERROR("Exception reading %s!", reg_list[i].name);
					res = ERROR_FAIL;
					goto xtensa_fetch_all_regs_done;
				}
			}
		}
	}

	if (xtensa->core_config->windowed)
		/* We need the windowbase to decode the general addresses. */
		windowbase = buf_get_u32(regvals[XT_REG_IDX_WINDOWBASE].buf, 0, 32);
	/* Decode the result and update the cache. */
	for (unsigned int i = 0; i < reg_list_size; i++) {
		struct xtensa_reg_desc *rlist = (i < XT_NUM_REGS) ? xtensa_regs : xtensa->optregs;
		unsigned int ridx = (i < XT_NUM_REGS) ? i : i - XT_NUM_REGS;
		if (xtensa_reg_is_readable(rlist[ridx].flags, cpenable) && rlist[ridx].exist) {
			if ((xtensa->core_config->windowed) && (rlist[ridx].type == XT_REG_GENERAL)) {
				/* The 64-value general register set is read from (windowbase) on down.
				 * We need to get the real register address by subtracting windowbase and
				 * wrapping around. */
				enum xtensa_reg_id realadr = xtensa_canonical_to_windowbase_offset(xtensa, i,
					windowbase);
				buf_cpy(regvals[realadr].buf, reg_list[i].value, reg_list[i].size);
			} else if (rlist[ridx].type == XT_REG_RELGEN) {
				buf_cpy(regvals[rlist[ridx].reg_num].buf, reg_list[i].value, reg_list[i].size);
				if (xtensa_extra_debug_log) {
					xtensa_reg_val_t regval = buf_get_u32(regvals[rlist[ridx].reg_num].buf, 0, 32);
					LOG_DEBUG("%s = 0x%x", rlist[ridx].name, regval);
				}
			} else {
				xtensa_reg_val_t regval = buf_get_u32(regvals[i].buf, 0, 32);
				bool is_dirty = (i == XT_REG_IDX_CPENABLE);
				if (xtensa_extra_debug_log)
					LOG_INFO("Register %s: 0x%X", reg_list[i].name, regval);
				xtensa_reg_set(target, i, regval);
				reg_list[i].dirty = is_dirty;	/*always do this _after_ xtensa_reg_set! */
			}
			reg_list[i].valid = true;
		} else {
			if ((rlist[ridx].flags & XT_REGF_MASK) == XT_REGF_NOREAD) {
				/* Report read-only registers all-zero but valid */
				reg_list[i].valid = true;
				xtensa_reg_set(target, i, 0);
			} else {
				reg_list[i].valid = false;
			}
		}
	}

	if (xtensa->core_config->windowed) {
		/* We have used A3 as a scratch register.
		 * Windowed configs: restore A3's AR (XT_REG_GENERAL) and and flag for write-back.
		 */
		enum xtensa_reg_id ar3_idx = xtensa_windowbase_offset_to_canonical(xtensa, XT_REG_IDX_A3, windowbase);
		xtensa_reg_set(target, ar3_idx, a3);
		xtensa_mark_register_dirty(xtensa, ar3_idx);

		/* Reset scratch_ars[] on fetch.  .chrval tracks AR mapping and changes w/ window */
		sprintf(xtensa->scratch_ars[XT_AR_SCRATCH_AR3].chrval, "ar%d", ar3_idx - XT_REG_IDX_AR0);
		enum xtensa_reg_id ar4_idx = xtensa_windowbase_offset_to_canonical(xtensa, XT_REG_IDX_A4, windowbase);
		sprintf(xtensa->scratch_ars[XT_AR_SCRATCH_AR4].chrval, "ar%d", ar4_idx - XT_REG_IDX_AR0);
		for (enum xtensa_ar_scratch_set_e s = 0; s < XT_AR_SCRATCH_NUM; s++)
			xtensa->scratch_ars[s].intval = false;
	}

	/* We have used A3 (XT_REG_RELGEN) as a scratch register.  Restore and flag for write-back. */
	xtensa_reg_set(target, XT_REG_IDX_A3, a3);
	xtensa_mark_register_dirty(xtensa, XT_REG_IDX_A3);
	xtensa->regs_fetched = true;
xtensa_fetch_all_regs_done:
	free(regvals);
	free(dsrs);
	return res;
}

int xtensa_get_gdb_reg_list(struct target *target,
	struct reg **reg_list[],
	int *reg_list_size,
	enum target_register_class reg_class)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	unsigned int num_regs;

	if (reg_class == REG_CLASS_GENERAL) {
		if ((xtensa->genpkt_regs_num == 0) || !xtensa->contiguous_regs_list) {
			LOG_ERROR("reg_class %d unhandled; 'xtgregs' not found", reg_class);
			return ERROR_FAIL;
		}
		num_regs = xtensa->genpkt_regs_num;
	} else {
		/* Determine whether to return a contiguous or sparse register map */
		num_regs = xtensa->regmap_contiguous ? xtensa->total_regs_num : xtensa->dbregs_num;
	}

	LOG_DEBUG("reg_class=%i, num_regs=%d", (int)reg_class, num_regs);

	*reg_list = calloc(num_regs, sizeof(struct reg *));
	if (!*reg_list)
		return ERROR_FAIL;

	*reg_list_size = num_regs;
	if (xtensa->regmap_contiguous) {
		assert((num_regs <= xtensa->total_regs_num) && "contiguous regmap size internal error!");
		for (unsigned int i = 0; i < num_regs; i++)
			(*reg_list)[i] = xtensa->contiguous_regs_list[i];
		return ERROR_OK;
	}

	for (unsigned int i = 0; i < num_regs; i++)
		(*reg_list)[i] = (struct reg *)&xtensa->empty_regs[i];
	unsigned int k = 0;
	for (unsigned int i = 0; i < xtensa->core_cache->num_regs && k < num_regs; i++) {
		if (xtensa->core_cache->reg_list[i].exist) {
			struct xtensa_reg_desc *rlist = (i < XT_NUM_REGS) ? xtensa_regs : xtensa->optregs;
			unsigned int ridx = (i < XT_NUM_REGS) ? i : i - XT_NUM_REGS;
			int sparse_idx = rlist[ridx].dbreg_num;
			if (i == XT_REG_IDX_PS) {
				if (xtensa->eps_dbglevel_idx == 0) {
					LOG_ERROR("eps_dbglevel_idx not set\n");
					return ERROR_FAIL;
				}
				(*reg_list)[sparse_idx] = &xtensa->core_cache->reg_list[xtensa->eps_dbglevel_idx];
				if (xtensa_extra_debug_log)
					LOG_DEBUG("SPARSE GDB reg 0x%x getting EPS%d 0x%x",
						sparse_idx, xtensa->core_config->debug.irq_level,
						xtensa_reg_get_value((*reg_list)[sparse_idx]));
			} else if (rlist[ridx].type == XT_REG_RELGEN) {
				(*reg_list)[sparse_idx - XT_REG_IDX_ARFIRST] = &xtensa->core_cache->reg_list[i];
			} else {
				(*reg_list)[sparse_idx] = &xtensa->core_cache->reg_list[i];
			}
			if (i == XT_REG_IDX_PC)
				/* Make a duplicate copy of PC for external access */
				(*reg_list)[XT_PC_DBREG_NUM_BASE] = &xtensa->core_cache->reg_list[i];
			k++;
		}
	}

	if (k == num_regs)
		LOG_ERROR("SPARSE GDB reg list full (size %d)", k);

	return ERROR_OK;
}

int xtensa_mmu_is_enabled(struct target *target, int *enabled)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	*enabled = xtensa->core_config->mmu.itlb_entries_count > 0 ||
		xtensa->core_config->mmu.dtlb_entries_count > 0;
	return ERROR_OK;
}

int xtensa_halt(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);

	LOG_TARGET_DEBUG(target, "start");
	if (target->state == TARGET_HALTED) {
		LOG_TARGET_DEBUG(target, "target was already halted");
		return ERROR_OK;
	}
	/* First we have to read dsr and check if the target stopped */
	int res = xtensa_dm_core_status_read(&xtensa->dbg_mod);
	if (res != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to read core status!");
		return res;
	}
	LOG_TARGET_DEBUG(target, "Core status 0x%" PRIx32, xtensa_dm_core_status_get(&xtensa->dbg_mod));
	if (!xtensa_is_stopped(target)) {
		xtensa_queue_dbg_reg_write(xtensa, XDMREG_DCRSET, OCDDCR_ENABLEOCD | OCDDCR_DEBUGINTERRUPT);
		xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
		res = xtensa_dm_queue_execute(&xtensa->dbg_mod);
		if (res != ERROR_OK)
			LOG_TARGET_ERROR(target, "Failed to set OCDDCR_DEBUGINTERRUPT. Can't halt.");
	}

	return res;
}

int xtensa_prepare_resume(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints,
	int debug_execution)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	uint32_t bpena = 0;

	LOG_TARGET_DEBUG(target,
		"current=%d address=" TARGET_ADDR_FMT ", handle_breakpoints=%i, debug_execution=%i)",
		current,
		address,
		handle_breakpoints,
		debug_execution);

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_WARNING(target, "target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (address && !current) {
		xtensa_reg_set(target, XT_REG_IDX_PC, address);
	} else {
		uint32_t cause = xtensa_cause_get(target);
		LOG_TARGET_DEBUG(target, "DEBUGCAUSE 0x%x (watchpoint %lu) (break %lu)",
			cause, (cause & DEBUGCAUSE_DB), (cause & (DEBUGCAUSE_BI | DEBUGCAUSE_BN)));
		if (cause & DEBUGCAUSE_DB)
			/* We stopped due to a watchpoint. We can't just resume executing the
			 * instruction again because */
			/* that would trigger the watchpoint again. To fix this, we single-step,
			 * which ignores watchpoints. */
			xtensa_do_step(target, current, address, handle_breakpoints);
		if (cause & (DEBUGCAUSE_BI | DEBUGCAUSE_BN))
			/* We stopped due to a break instruction. We can't just resume executing the
			 * instruction again because */
			/* that would trigger the break again. To fix this, we single-step, which
			 * ignores break. */
			xtensa_do_step(target, current, address, handle_breakpoints);
	}

	/* Write back hw breakpoints. Current FreeRTOS SMP code can set a hw breakpoint on an
	 * exception; we need to clear that and return to the breakpoints gdb has set on resume. */
	for (unsigned int slot = 0; slot < xtensa->core_config->debug.ibreaks_num; slot++) {
		if (xtensa->hw_brps[slot]) {
			/* Write IBREAKA[slot] and set bit #slot in IBREAKENABLE */
			xtensa_reg_set(target, XT_REG_IDX_IBREAKA0 + slot, xtensa->hw_brps[slot]->address);
			bpena |= BIT(slot);
		}
	}
	xtensa_reg_set(target, XT_REG_IDX_IBREAKENABLE, bpena);

	/* Here we write all registers to the targets */
	int res = xtensa_write_dirty_registers(target);
	if (res != ERROR_OK)
		LOG_TARGET_ERROR(target, "Failed to write back register cache.");
	return res;
}

int xtensa_do_resume(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);

	LOG_TARGET_DEBUG(target, "start");

	xtensa_queue_exec_ins(xtensa, XT_INS_RFDO(xtensa));
	int res = xtensa_dm_queue_execute(&xtensa->dbg_mod);
	if (res != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to exec RFDO %d!", res);
		return res;
	}
	xtensa_core_status_check(target);
	return ERROR_OK;
}

int xtensa_resume(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints,
	int debug_execution)
{
	LOG_TARGET_DEBUG(target, "start");
	int res = xtensa_prepare_resume(target, current, address, handle_breakpoints, debug_execution);
	if (res != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to prepare for resume!");
		return res;
	}
	res = xtensa_do_resume(target);
	if (res != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to resume!");
		return res;
	}

	target->debug_reason = DBG_REASON_NOTHALTED;
	if (!debug_execution)
		target->state = TARGET_RUNNING;
	else
		target->state = TARGET_DEBUG_RUNNING;

	target_call_event_callbacks(target, TARGET_EVENT_RESUMED);

	return ERROR_OK;
}

static bool xtensa_pc_in_winexc(struct target *target, target_addr_t pc)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	uint8_t insn_buf[XT_ISNS_SZ_MAX];
	int err = xtensa_read_buffer(target, pc, sizeof(insn_buf), insn_buf);
	if (err != ERROR_OK)
		return false;

	xtensa_insn_t insn = buf_get_u32(insn_buf, 0, 24);
	xtensa_insn_t masked = insn & XT_INS_L32E_S32E_MASK(xtensa);
	if (masked == XT_INS_L32E(xtensa, 0, 0, 0) || masked == XT_INS_S32E(xtensa, 0, 0, 0))
		return true;

	masked = insn & XT_INS_RFWO_RFWU_MASK(xtensa);
	if (masked == XT_INS_RFWO(xtensa) || masked == XT_INS_RFWU(xtensa))
		return true;

	return false;
}

int xtensa_do_step(struct target *target, int current, target_addr_t address, int handle_breakpoints)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int res;
	const uint32_t icount_val = -2;	/* ICOUNT value to load for 1 step */
	xtensa_reg_val_t dbreakc[XT_WATCHPOINTS_NUM_MAX];
	xtensa_reg_val_t icountlvl, cause;
	xtensa_reg_val_t oldps, oldpc, cur_pc;
	bool ps_lowered = false;

	LOG_TARGET_DEBUG(target, "current=%d, address=" TARGET_ADDR_FMT ", handle_breakpoints=%i",
		current, address, handle_breakpoints);

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_WARNING(target, "target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (xtensa->eps_dbglevel_idx == 0) {
		LOG_ERROR("eps_dbglevel_idx not set\n");
		return ERROR_FAIL;
	}

	/* Save old ps (EPS[dbglvl] on LX), pc */
	oldps = xtensa_reg_get(target, xtensa->eps_dbglevel_idx);
	oldpc = xtensa_reg_get(target, XT_REG_IDX_PC);

	cause = xtensa_cause_get(target);
	LOG_TARGET_DEBUG(target, "oldps=%" PRIx32 ", oldpc=%" PRIx32 " dbg_cause=%" PRIx32 " exc_cause=%" PRIx32,
		oldps,
		oldpc,
		cause,
		xtensa_reg_get(target, XT_REG_IDX_EXCCAUSE));
	if (handle_breakpoints && (cause & (DEBUGCAUSE_BI | DEBUGCAUSE_BN))) {
		/* handle hard-coded SW breakpoints (e.g. syscalls) */
		LOG_TARGET_DEBUG(target, "Increment PC to pass break instruction...");
		xtensa_cause_clear(target);	/* so we don't recurse into the same routine */
		/* pretend that we have stepped */
		if (cause & DEBUGCAUSE_BI)
			xtensa_reg_set(target, XT_REG_IDX_PC, oldpc + 3);	/* PC = PC+3 */
		else
			xtensa_reg_set(target, XT_REG_IDX_PC, oldpc + 2);	/* PC = PC+2 */
		return ERROR_OK;
	}

	/* Xtensa LX has an ICOUNTLEVEL register which sets the maximum interrupt level
	 * at which the instructions are to be counted while stepping.
	 *
	 * For example, if we need to step by 2 instructions, and an interrupt occurs
	 * in between, the processor will trigger the interrupt and halt after the 2nd
	 * instruction within the interrupt vector and/or handler.
	 *
	 * However, sometimes we don't want the interrupt handlers to be executed at all
	 * while stepping through the code. In this case (XT_STEPPING_ISR_OFF),
	 * ICOUNTLEVEL can be lowered to the executing code's (level + 1) to prevent ISR
	 * code from being counted during stepping.  Note that C exception handlers must
	 * run at level 0 and hence will be counted and stepped into, should one occur.
	 *
	 * TODO: Certain instructions should never be single-stepped and should instead
	 * be emulated (per DUG): RSIL >= DBGLEVEL, RSR/WSR [ICOUNT|ICOUNTLEVEL], and
	 * RFI >= DBGLEVEL.
	 */
	if (xtensa->stepping_isr_mode == XT_STEPPING_ISR_OFF) {
		if (!xtensa->core_config->high_irq.enabled) {
			LOG_TARGET_WARNING(
				target,
				"disabling IRQs while stepping is not implemented w/o high prio IRQs option!");
			return ERROR_FAIL;
		}
		/* Update ICOUNTLEVEL accordingly */
		icountlvl = MIN((oldps & 0xF) + 1, xtensa->core_config->debug.irq_level);
	} else {
		icountlvl = xtensa->core_config->debug.irq_level;
	}

	if (cause & DEBUGCAUSE_DB) {
		/* We stopped due to a watchpoint. We can't just resume executing the instruction again because
		 * that would trigger the watchpoint again. To fix this, we remove watchpoints,single-step and
		 * re-enable the watchpoint. */
		LOG_TARGET_DEBUG(
			target,
			"Single-stepping to get past instruction that triggered the watchpoint...");
		xtensa_cause_clear(target);	/* so we don't recurse into the same routine */
		/* Save all DBREAKCx registers and set to 0 to disable watchpoints */
		for (unsigned int slot = 0; slot < xtensa->core_config->debug.dbreaks_num; slot++) {
			dbreakc[slot] = xtensa_reg_get(target, XT_REG_IDX_DBREAKC0 + slot);
			xtensa_reg_set(target, XT_REG_IDX_DBREAKC0 + slot, 0);
		}
	}

	if (!handle_breakpoints && (cause & (DEBUGCAUSE_BI | DEBUGCAUSE_BN)))
		/* handle normal SW breakpoint */
		xtensa_cause_clear(target);	/* so we don't recurse into the same routine */
	if ((oldps & 0xf) >= icountlvl) {
		/* Lower interrupt level to allow stepping, but flag eps[dbglvl] to be restored */
		ps_lowered = true;
		uint32_t newps = (oldps & ~0xf) | (icountlvl - 1);
		xtensa_reg_set(target, xtensa->eps_dbglevel_idx, newps);
		LOG_TARGET_DEBUG(target,
			"Lowering PS.INTLEVEL to allow stepping: %s <- 0x%08" PRIx32 " (was 0x%08" PRIx32 ")",
			xtensa->core_cache->reg_list[xtensa->eps_dbglevel_idx].name,
			newps,
			oldps);
	}
	do {
		xtensa_reg_set(target, XT_REG_IDX_ICOUNTLEVEL, icountlvl);
		xtensa_reg_set(target, XT_REG_IDX_ICOUNT, icount_val);

		/* Now ICOUNT is set, we can resume as if we were going to run */
		res = xtensa_prepare_resume(target, current, address, 0, 0);
		if (res != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Failed to prepare resume for single step");
			return res;
		}
		res = xtensa_do_resume(target);
		if (res != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Failed to resume after setting up single step");
			return res;
		}

		/* Wait for stepping to complete */
		long long start = timeval_ms();
		while (timeval_ms() < start + 500) {
			/* Do not use target_poll here, it also triggers other things... just manually read the DSR
			 *until stepping is complete. */
			usleep(1000);
			res = xtensa_dm_core_status_read(&xtensa->dbg_mod);
			if (res != ERROR_OK) {
				LOG_TARGET_ERROR(target, "Failed to read core status!");
				return res;
			}
			if (xtensa_is_stopped(target))
				break;
			usleep(1000);
		}
		LOG_TARGET_DEBUG(target, "Finish stepping. dsr=0x%08" PRIx32,
			xtensa_dm_core_status_get(&xtensa->dbg_mod));
		if (!xtensa_is_stopped(target)) {
			LOG_TARGET_WARNING(
				target,
				"Timed out waiting for target to finish stepping. dsr=0x%08" PRIx32,
				xtensa_dm_core_status_get(&xtensa->dbg_mod));
			target->debug_reason = DBG_REASON_NOTHALTED;
			target->state = TARGET_RUNNING;
			return ERROR_FAIL;
		}

		xtensa_fetch_all_regs(target);
		cur_pc = xtensa_reg_get(target, XT_REG_IDX_PC);

		LOG_TARGET_DEBUG(target,
			"cur_ps=%" PRIx32 ", cur_pc=%" PRIx32 " dbg_cause=%" PRIx32 " exc_cause=%" PRIx32,
			xtensa_reg_get(target, XT_REG_IDX_PS),
			cur_pc,
			xtensa_cause_get(target),
			xtensa_reg_get(target, XT_REG_IDX_EXCCAUSE));

		/* Do not step into WindowOverflow if ISRs are masked.
		   If we stop in WindowOverflow at breakpoint with masked ISRs and
		   try to do a step it will get us out of that handler */
		if (xtensa->core_config->windowed &&
			xtensa->stepping_isr_mode == XT_STEPPING_ISR_OFF &&
			xtensa_pc_in_winexc(target, cur_pc)) {
			/* isrmask = on, need to step out of the window exception handler */
			LOG_DEBUG("Stepping out of window exception, PC=%" PRIX32, cur_pc);
			oldpc = cur_pc;
			address = oldpc + 3;
			continue;
		}

		if (oldpc == cur_pc)
			LOG_TARGET_WARNING(target, "Stepping doesn't seem to change PC! dsr=0x%08" PRIx32,
				xtensa_dm_core_status_get(&xtensa->dbg_mod));
		else
			LOG_DEBUG("Stepped from %" PRIX32 " to %" PRIX32, oldpc, cur_pc);
		break;
	} while (true);

	target->debug_reason = DBG_REASON_SINGLESTEP;
	target->state = TARGET_HALTED;
	target_call_event_callbacks(target, TARGET_EVENT_HALTED);
	LOG_DEBUG("Done stepping, PC=%" PRIX32, cur_pc);

	if (cause & DEBUGCAUSE_DB) {
		LOG_TARGET_DEBUG(target, "...Done, re-installing watchpoints.");
		/* Restore the DBREAKCx registers */
		for (unsigned int slot = 0; slot < xtensa->core_config->debug.dbreaks_num; slot++)
			xtensa_reg_set(target, XT_REG_IDX_DBREAKC0 + slot, dbreakc[slot]);
	}

	/* Restore int level */
	if (ps_lowered) {
		LOG_DEBUG("Restoring %s after stepping: 0x%08" PRIx32,
			xtensa->core_cache->reg_list[xtensa->eps_dbglevel_idx].name,
			oldps);
		xtensa_reg_set(target, xtensa->eps_dbglevel_idx, oldps);
	}

	/* write ICOUNTLEVEL back to zero */
	xtensa_reg_set(target, XT_REG_IDX_ICOUNTLEVEL, 0);
	/* TODO: can we skip writing dirty registers and re-fetching them? */
	res = xtensa_write_dirty_registers(target);
	xtensa_fetch_all_regs(target);
	return res;
}

int xtensa_step(struct target *target, int current, target_addr_t address, int handle_breakpoints)
{
	return xtensa_do_step(target, current, address, handle_breakpoints);
}

/**
 * Returns true if two ranges are overlapping
 */
static inline bool xtensa_memory_regions_overlap(target_addr_t r1_start,
	target_addr_t r1_end,
	target_addr_t r2_start,
	target_addr_t r2_end)
{
	if ((r2_start >= r1_start) && (r2_start < r1_end))
		return true;	/* r2_start is in r1 region */
	if ((r2_end > r1_start) && (r2_end <= r1_end))
		return true;	/* r2_end is in r1 region */
	return false;
}

/**
 * Returns a size of overlapped region of two ranges.
 */
static inline target_addr_t xtensa_get_overlap_size(target_addr_t r1_start,
	target_addr_t r1_end,
	target_addr_t r2_start,
	target_addr_t r2_end)
{
	if (xtensa_memory_regions_overlap(r1_start, r1_end, r2_start, r2_end)) {
		target_addr_t ov_start = r1_start < r2_start ? r2_start : r1_start;
		target_addr_t ov_end = r1_end > r2_end ? r2_end : r1_end;
		return ov_end - ov_start;
	}
	return 0;
}

/**
 * Check if the address gets to memory regions, and its access mode
 */
static bool xtensa_memory_op_validate_range(struct xtensa *xtensa, target_addr_t address, size_t size, int access)
{
	target_addr_t adr_pos = address;	/* address cursor set to the beginning start */
	target_addr_t adr_end = address + size;	/* region end */
	target_addr_t overlap_size;
	const struct xtensa_local_mem_region_config *cm;	/* current mem region */

	while (adr_pos < adr_end) {
		cm = xtensa_target_memory_region_find(xtensa, adr_pos);
		if (!cm)	/* address is not belong to anything */
			return false;
		if ((cm->access & access) != access)	/* access check */
			return false;
		overlap_size = xtensa_get_overlap_size(cm->base, (cm->base + cm->size), adr_pos, adr_end);
		assert(overlap_size != 0);
		adr_pos += overlap_size;
	}
	return true;
}

int xtensa_read_memory(struct target *target, target_addr_t address, uint32_t size, uint32_t count, uint8_t *buffer)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	/* We are going to read memory in 32-bit increments. This may not be what the calling
	 * function expects, so we may need to allocate a temp buffer and read into that first. */
	target_addr_t addrstart_al = ALIGN_DOWN(address, 4);
	target_addr_t addrend_al = ALIGN_UP(address + size * count, 4);
	target_addr_t adr = addrstart_al;
	uint8_t *albuff;
	bool bswap = xtensa->target->endianness == TARGET_BIG_ENDIAN;

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_WARNING(target, "target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!xtensa->permissive_mode) {
		if (!xtensa_memory_op_validate_range(xtensa, address, (size * count),
				XT_MEM_ACCESS_READ)) {
			LOG_DEBUG("address " TARGET_ADDR_FMT " not readable", address);
			return ERROR_FAIL;
		}
	}

	if (addrstart_al == address && addrend_al == address + (size * count)) {
		albuff = buffer;
	} else {
		albuff = malloc(addrend_al - addrstart_al);
		if (!albuff) {
			LOG_TARGET_ERROR(target, "Out of memory allocating %" TARGET_PRIdADDR " bytes!",
				addrend_al - addrstart_al);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	/* We're going to use A3 here */
	xtensa_mark_register_dirty(xtensa, XT_REG_IDX_A3);
	/* Write start address to A3 */
	xtensa_queue_dbg_reg_write(xtensa, XDMREG_DDR, addrstart_al);
	xtensa_queue_exec_ins(xtensa, XT_INS_RSR(xtensa, XT_SR_DDR, XT_REG_A3));
	/* Now we can safely read data from addrstart_al up to addrend_al into albuff */
	if (xtensa->probe_lsddr32p != 0) {
		xtensa_queue_exec_ins(xtensa, XT_INS_LDDR32P(xtensa, XT_REG_A3));
		for (unsigned int i = 0; adr != addrend_al; i += sizeof(uint32_t), adr += sizeof(uint32_t))
			xtensa_queue_dbg_reg_read(xtensa,
				(adr + sizeof(uint32_t) == addrend_al) ? XDMREG_DDR : XDMREG_DDREXEC,
				&albuff[i]);
	} else {
		xtensa_mark_register_dirty(xtensa, XT_REG_IDX_A4);
		for (unsigned int i = 0; adr != addrend_al; i += sizeof(uint32_t), adr += sizeof(uint32_t)) {
			xtensa_queue_exec_ins(xtensa, XT_INS_L32I(xtensa, XT_REG_A3, XT_REG_A4, 0));
			xtensa_queue_exec_ins(xtensa, XT_INS_WSR(xtensa, XT_SR_DDR, XT_REG_A4));
			xtensa_queue_dbg_reg_read(xtensa, XDMREG_DDR, &albuff[i]);
			xtensa_queue_dbg_reg_write(xtensa, XDMREG_DDR, adr + sizeof(uint32_t));
			xtensa_queue_exec_ins(xtensa, XT_INS_RSR(xtensa, XT_SR_DDR, XT_REG_A3));
		}
	}
	int res = xtensa_dm_queue_execute(&xtensa->dbg_mod);
	if (res == ERROR_OK) {
		bool prev_suppress = xtensa->suppress_dsr_errors;
		xtensa->suppress_dsr_errors = true;
		res = xtensa_core_status_check(target);
		if (xtensa->probe_lsddr32p == -1)
			xtensa->probe_lsddr32p = 1;
		xtensa->suppress_dsr_errors = prev_suppress;
	}
	if (res != ERROR_OK) {
		if (xtensa->probe_lsddr32p != 0) {
			/* Disable fast memory access instructions and retry before reporting an error */
			LOG_TARGET_INFO(target, "Disabling LDDR32.P/SDDR32.P");
			xtensa->probe_lsddr32p = 0;
			res = xtensa_read_memory(target, address, size, count, buffer);
			bswap = false;
		} else {
			LOG_TARGET_WARNING(target, "Failed reading %d bytes at address "TARGET_ADDR_FMT,
				count * size, address);
		}
	}

	if (bswap)
		buf_bswap32(albuff, albuff, addrend_al - addrstart_al);
	if (albuff != buffer) {
		memcpy(buffer, albuff + (address & 3), (size * count));
		free(albuff);
	}

	return res;
}

int xtensa_read_buffer(struct target *target, target_addr_t address, uint32_t count, uint8_t *buffer)
{
	/* xtensa_read_memory can also read unaligned stuff. Just pass through to that routine. */
	return xtensa_read_memory(target, address, 1, count, buffer);
}

int xtensa_write_memory(struct target *target,
	target_addr_t address,
	uint32_t size,
	uint32_t count,
	const uint8_t *buffer)
{
	/* This memory write function can get thrown nigh everything into it, from
	 * aligned uint32 writes to unaligned uint8ths. The Xtensa memory doesn't always
	 * accept anything but aligned uint32 writes, though. That is why we convert
	 * everything into that. */
	struct xtensa *xtensa = target_to_xtensa(target);
	target_addr_t addrstart_al = ALIGN_DOWN(address, 4);
	target_addr_t addrend_al = ALIGN_UP(address + size * count, 4);
	target_addr_t adr = addrstart_al;
	int res;
	uint8_t *albuff;
	bool fill_head_tail = false;

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_WARNING(target, "target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (!xtensa->permissive_mode) {
		if (!xtensa_memory_op_validate_range(xtensa, address, (size * count), XT_MEM_ACCESS_WRITE)) {
			LOG_WARNING("address " TARGET_ADDR_FMT " not writable", address);
			return ERROR_FAIL;
		}
	}

	if (size == 0 || count == 0 || !buffer)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* Allocate a temporary buffer to put the aligned bytes in, if needed. */
	if (addrstart_al == address && addrend_al == address + (size * count)) {
		if (xtensa->target->endianness == TARGET_BIG_ENDIAN)
			/* Need a buffer for byte-swapping */
			albuff = malloc(addrend_al - addrstart_al);
		else
			/* We discard the const here because albuff can also be non-const */
			albuff = (uint8_t *)buffer;
	} else {
		fill_head_tail = true;
		albuff = malloc(addrend_al - addrstart_al);
	}
	if (!albuff) {
		LOG_TARGET_ERROR(target, "Out of memory allocating %" TARGET_PRIdADDR " bytes!",
			addrend_al - addrstart_al);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* We're going to use A3 here */
	xtensa_mark_register_dirty(xtensa, XT_REG_IDX_A3);

	/* If we're using a temp aligned buffer, we need to fill the head and/or tail bit of it. */
	if (fill_head_tail) {
		/* See if we need to read the first and/or last word. */
		if (address & 3) {
			xtensa_queue_dbg_reg_write(xtensa, XDMREG_DDR, addrstart_al);
			xtensa_queue_exec_ins(xtensa, XT_INS_RSR(xtensa, XT_SR_DDR, XT_REG_A3));
			if (xtensa->probe_lsddr32p == 1) {
				xtensa_queue_exec_ins(xtensa, XT_INS_LDDR32P(xtensa, XT_REG_A3));
			} else {
				xtensa_queue_exec_ins(xtensa, XT_INS_L32I(xtensa, XT_REG_A3, XT_REG_A3, 0));
				xtensa_queue_exec_ins(xtensa, XT_INS_WSR(xtensa, XT_SR_DDR, XT_REG_A3));
			}
			xtensa_queue_dbg_reg_read(xtensa, XDMREG_DDR, &albuff[0]);
		}
		if ((address + (size * count)) & 3) {
			xtensa_queue_dbg_reg_write(xtensa, XDMREG_DDR, addrend_al - 4);
			xtensa_queue_exec_ins(xtensa, XT_INS_RSR(xtensa, XT_SR_DDR, XT_REG_A3));
			if (xtensa->probe_lsddr32p == 1) {
				xtensa_queue_exec_ins(xtensa, XT_INS_LDDR32P(xtensa, XT_REG_A3));
			} else {
				xtensa_queue_exec_ins(xtensa, XT_INS_L32I(xtensa, XT_REG_A3, XT_REG_A3, 0));
				xtensa_queue_exec_ins(xtensa, XT_INS_WSR(xtensa, XT_SR_DDR, XT_REG_A3));
			}
			xtensa_queue_dbg_reg_read(xtensa, XDMREG_DDR,
				&albuff[addrend_al - addrstart_al - 4]);
		}
		/* Grab bytes */
		res = xtensa_dm_queue_execute(&xtensa->dbg_mod);
		if (res != ERROR_OK) {
			LOG_ERROR("Error issuing unaligned memory write context instruction(s): %d", res);
			if (albuff != buffer)
				free(albuff);
			return res;
		}
		xtensa_core_status_check(target);
		if (xtensa->target->endianness == TARGET_BIG_ENDIAN) {
			bool swapped_w0 = false;
			if (address & 3) {
				buf_bswap32(&albuff[0], &albuff[0], 4);
				swapped_w0 = true;
			}
			if ((address + (size * count)) & 3) {
				if ((addrend_al - addrstart_al - 4 == 0) && swapped_w0) {
					/* Don't double-swap if buffer start/end are within the same word */
				} else {
					buf_bswap32(&albuff[addrend_al - addrstart_al - 4],
						&albuff[addrend_al - addrstart_al - 4], 4);
				}
			}
		}
		/* Copy data to be written into the aligned buffer (in host-endianness) */
		memcpy(&albuff[address & 3], buffer, size * count);
		/* Now we can write albuff in aligned uint32s. */
	}

	if (xtensa->target->endianness == TARGET_BIG_ENDIAN)
		buf_bswap32(albuff, fill_head_tail ? albuff : buffer, addrend_al - addrstart_al);

	/* Write start address to A3 */
	xtensa_queue_dbg_reg_write(xtensa, XDMREG_DDR, addrstart_al);
	xtensa_queue_exec_ins(xtensa, XT_INS_RSR(xtensa, XT_SR_DDR, XT_REG_A3));
	/* Write the aligned buffer */
	if (xtensa->probe_lsddr32p != 0) {
		for (unsigned int i = 0; adr != addrend_al; i += sizeof(uint32_t), adr += sizeof(uint32_t)) {
			if (i == 0) {
				xtensa_queue_dbg_reg_write(xtensa, XDMREG_DDR, buf_get_u32(&albuff[i], 0, 32));
				xtensa_queue_exec_ins(xtensa, XT_INS_SDDR32P(xtensa, XT_REG_A3));
			} else {
				xtensa_queue_dbg_reg_write(xtensa, XDMREG_DDREXEC, buf_get_u32(&albuff[i], 0, 32));
			}
		}
	} else {
		xtensa_mark_register_dirty(xtensa, XT_REG_IDX_A4);
		for (unsigned int i = 0; adr != addrend_al; i += sizeof(uint32_t), adr += sizeof(uint32_t)) {
			xtensa_queue_dbg_reg_write(xtensa, XDMREG_DDR, buf_get_u32(&albuff[i], 0, 32));
			xtensa_queue_exec_ins(xtensa, XT_INS_RSR(xtensa, XT_SR_DDR, XT_REG_A4));
			xtensa_queue_exec_ins(xtensa, XT_INS_S32I(xtensa, XT_REG_A3, XT_REG_A4, 0));
			xtensa_queue_dbg_reg_write(xtensa, XDMREG_DDR, adr + sizeof(uint32_t));
			xtensa_queue_exec_ins(xtensa, XT_INS_RSR(xtensa, XT_SR_DDR, XT_REG_A3));
		}
	}

	res = xtensa_dm_queue_execute(&xtensa->dbg_mod);
	if (res == ERROR_OK) {
		bool prev_suppress = xtensa->suppress_dsr_errors;
		xtensa->suppress_dsr_errors = true;
		res = xtensa_core_status_check(target);
		if (xtensa->probe_lsddr32p == -1)
			xtensa->probe_lsddr32p = 1;
		xtensa->suppress_dsr_errors = prev_suppress;
	}
	if (res != ERROR_OK) {
		if (xtensa->probe_lsddr32p != 0) {
			/* Disable fast memory access instructions and retry before reporting an error */
			LOG_TARGET_INFO(target, "Disabling LDDR32.P/SDDR32.P");
			xtensa->probe_lsddr32p = 0;
			res = xtensa_write_memory(target, address, size, count, buffer);
		} else {
			LOG_TARGET_WARNING(target, "Failed writing %d bytes at address "TARGET_ADDR_FMT,
				count * size, address);
		}
	} else {
		/* Invalidate ICACHE, writeback DCACHE if present */
		uint32_t issue_ihi = xtensa_is_icacheable(xtensa, address);
		uint32_t issue_dhwb = xtensa_is_dcacheable(xtensa, address);
		if (issue_ihi || issue_dhwb) {
			uint32_t ilinesize = issue_ihi ?  xtensa->core_config->icache.line_size : UINT32_MAX;
			uint32_t dlinesize = issue_dhwb ? xtensa->core_config->dcache.line_size : UINT32_MAX;
			uint32_t linesize = MIN(ilinesize, dlinesize);
			uint32_t off = 0;
			adr = addrstart_al;

			while ((adr + off) < addrend_al) {
				if (off == 0) {
					/* Write start address to A3 */
					xtensa_queue_dbg_reg_write(xtensa, XDMREG_DDR, adr);
					xtensa_queue_exec_ins(xtensa, XT_INS_RSR(xtensa, XT_SR_DDR, XT_REG_A3));
				}
				if (issue_ihi)
					xtensa_queue_exec_ins(xtensa, XT_INS_IHI(xtensa, XT_REG_A3, off));
				if (issue_dhwb)
					xtensa_queue_exec_ins(xtensa, XT_INS_DHWBI(xtensa, XT_REG_A3, off));
				off += linesize;
				if (off > 1020) {
					/* IHI, DHWB have 8-bit immediate operands (0..1020) */
					adr += off;
					off = 0;
				}
			}

			/* Execute cache WB/INV instructions */
			res = xtensa_dm_queue_execute(&xtensa->dbg_mod);
			xtensa_core_status_check(target);
			if (res != ERROR_OK)
				LOG_TARGET_ERROR(target,
					"Error issuing cache writeback/invaldate instruction(s): %d",
					res);
		}
	}
	if (albuff != buffer)
		free(albuff);

	return res;
}

int xtensa_write_buffer(struct target *target, target_addr_t address, uint32_t count, const uint8_t *buffer)
{
	/* xtensa_write_memory can handle everything. Just pass on to that. */
	return xtensa_write_memory(target, address, 1, count, buffer);
}

int xtensa_checksum_memory(struct target *target, target_addr_t address, uint32_t count, uint32_t *checksum)
{
	LOG_WARNING("not implemented yet");
	return ERROR_FAIL;
}

int xtensa_poll(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	if (xtensa_dm_poll(&xtensa->dbg_mod) != ERROR_OK) {
		target->state = TARGET_UNKNOWN;
		return ERROR_TARGET_NOT_EXAMINED;
	}

	int res = xtensa_dm_power_status_read(&xtensa->dbg_mod, PWRSTAT_DEBUGWASRESET(xtensa) |
		PWRSTAT_COREWASRESET(xtensa));
	if (xtensa->dbg_mod.power_status.stat != xtensa->dbg_mod.power_status.stath)
		LOG_TARGET_DEBUG(target, "PWRSTAT: read 0x%08" PRIx32 ", clear 0x%08lx, reread 0x%08" PRIx32,
			xtensa->dbg_mod.power_status.stat,
			PWRSTAT_DEBUGWASRESET(xtensa) | PWRSTAT_COREWASRESET(xtensa),
			xtensa->dbg_mod.power_status.stath);
	if (res != ERROR_OK)
		return res;

	if (xtensa_dm_tap_was_reset(&xtensa->dbg_mod)) {
		LOG_TARGET_INFO(target, "Debug controller was reset.");
		res = xtensa_smpbreak_write(xtensa, xtensa->smp_break);
		if (res != ERROR_OK)
			return res;
	}
	if (xtensa_dm_core_was_reset(&xtensa->dbg_mod))
		LOG_TARGET_INFO(target, "Core was reset.");
	xtensa_dm_power_status_cache(&xtensa->dbg_mod);
	/* Enable JTAG, set reset if needed */
	res = xtensa_wakeup(target);
	if (res != ERROR_OK)
		return res;

	uint32_t prev_dsr = xtensa->dbg_mod.core_status.dsr;
	res = xtensa_dm_core_status_read(&xtensa->dbg_mod);
	if (res != ERROR_OK)
		return res;
	if (prev_dsr != xtensa->dbg_mod.core_status.dsr)
		LOG_TARGET_DEBUG(target,
			"DSR has changed: was 0x%08" PRIx32 " now 0x%08" PRIx32,
			prev_dsr,
			xtensa->dbg_mod.core_status.dsr);
	if (xtensa->dbg_mod.power_status.stath & PWRSTAT_COREWASRESET(xtensa)) {
		/* if RESET state is persitent  */
		target->state = TARGET_RESET;
	} else if (!xtensa_dm_is_powered(&xtensa->dbg_mod)) {
		LOG_TARGET_DEBUG(target, "not powered 0x%" PRIX32 "%ld",
			xtensa->dbg_mod.core_status.dsr,
			xtensa->dbg_mod.core_status.dsr & OCDDSR_STOPPED);
		target->state = TARGET_UNKNOWN;
		if (xtensa->come_online_probes_num == 0)
			target->examined = false;
		else
			xtensa->come_online_probes_num--;
	} else if (xtensa_is_stopped(target)) {
		if (target->state != TARGET_HALTED) {
			enum target_state oldstate = target->state;
			target->state = TARGET_HALTED;
			/* Examine why the target has been halted */
			target->debug_reason = DBG_REASON_DBGRQ;
			xtensa_fetch_all_regs(target);
			/* When setting debug reason DEBUGCAUSE events have the following
			 * priorities: watchpoint == breakpoint > single step > debug interrupt. */
			/* Watchpoint and breakpoint events at the same time results in special
			 * debug reason: DBG_REASON_WPTANDBKPT. */
			uint32_t halt_cause = xtensa_cause_get(target);
			/* TODO: Add handling of DBG_REASON_EXC_CATCH */
			if (halt_cause & DEBUGCAUSE_IC)
				target->debug_reason = DBG_REASON_SINGLESTEP;
			if (halt_cause & (DEBUGCAUSE_IB | DEBUGCAUSE_BN | DEBUGCAUSE_BI)) {
				if (halt_cause & DEBUGCAUSE_DB)
					target->debug_reason = DBG_REASON_WPTANDBKPT;
				else
					target->debug_reason = DBG_REASON_BREAKPOINT;
			} else if (halt_cause & DEBUGCAUSE_DB) {
				target->debug_reason = DBG_REASON_WATCHPOINT;
			}
			LOG_TARGET_DEBUG(target, "Target halted, pc=0x%08" PRIx32
				", debug_reason=%08" PRIx32 ", oldstate=%08" PRIx32,
				xtensa_reg_get(target, XT_REG_IDX_PC),
				target->debug_reason,
				oldstate);
			LOG_TARGET_DEBUG(target, "Halt reason=0x%08" PRIX32 ", exc_cause=%" PRId32 ", dsr=0x%08" PRIx32,
				halt_cause,
				xtensa_reg_get(target, XT_REG_IDX_EXCCAUSE),
				xtensa->dbg_mod.core_status.dsr);
			xtensa_dm_core_status_clear(
				&xtensa->dbg_mod,
				OCDDSR_DEBUGPENDBREAK | OCDDSR_DEBUGINTBREAK | OCDDSR_DEBUGPENDTRAX |
				OCDDSR_DEBUGINTTRAX |
				OCDDSR_DEBUGPENDHOST | OCDDSR_DEBUGINTHOST);
		}
	} else {
		target->debug_reason = DBG_REASON_NOTHALTED;
		if (target->state != TARGET_RUNNING && target->state != TARGET_DEBUG_RUNNING) {
			target->state = TARGET_RUNNING;
			target->debug_reason = DBG_REASON_NOTHALTED;
		}
	}
	if (xtensa->trace_active) {
		/* Detect if tracing was active but has stopped. */
		struct xtensa_trace_status trace_status;
		res = xtensa_dm_trace_status_read(&xtensa->dbg_mod, &trace_status);
		if (res == ERROR_OK) {
			if (!(trace_status.stat & TRAXSTAT_TRACT)) {
				LOG_INFO("Detected end of trace.");
				if (trace_status.stat & TRAXSTAT_PCMTG)
					LOG_TARGET_INFO(target, "Trace stop triggered by PC match");
				if (trace_status.stat & TRAXSTAT_PTITG)
					LOG_TARGET_INFO(target, "Trace stop triggered by Processor Trigger Input");
				if (trace_status.stat & TRAXSTAT_CTITG)
					LOG_TARGET_INFO(target, "Trace stop triggered by Cross-trigger Input");
				xtensa->trace_active = false;
			}
		}
	}
	return ERROR_OK;
}

static int xtensa_update_instruction(struct target *target, target_addr_t address, uint32_t size, const uint8_t *buffer)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	unsigned int issue_ihi = xtensa_is_icacheable(xtensa, address);
	unsigned int issue_dhwbi = xtensa_is_dcacheable(xtensa, address);
	uint32_t icache_line_size = issue_ihi ? xtensa->core_config->icache.line_size : UINT32_MAX;
	uint32_t dcache_line_size = issue_dhwbi ? xtensa->core_config->dcache.line_size : UINT32_MAX;
	unsigned int same_ic_line = ((address & (icache_line_size - 1)) + size) <= icache_line_size;
	unsigned int same_dc_line = ((address & (dcache_line_size - 1)) + size) <= dcache_line_size;
	int ret;

	if (size > icache_line_size)
		return ERROR_FAIL;

	if (issue_ihi || issue_dhwbi) {
		/* We're going to use A3 here */
		xtensa_mark_register_dirty(xtensa, XT_REG_IDX_A3);

		/* Write start address to A3 and invalidate */
		xtensa_queue_dbg_reg_write(xtensa, XDMREG_DDR, address);
		xtensa_queue_exec_ins(xtensa, XT_INS_RSR(xtensa, XT_SR_DDR, XT_REG_A3));
		LOG_TARGET_DEBUG(target, "DHWBI, IHI for address "TARGET_ADDR_FMT, address);
		if (issue_dhwbi) {
			xtensa_queue_exec_ins(xtensa, XT_INS_DHWBI(xtensa, XT_REG_A3, 0));
			if (!same_dc_line) {
				LOG_TARGET_DEBUG(target,
					"DHWBI second dcache line for address "TARGET_ADDR_FMT,
					address + 4);
				xtensa_queue_exec_ins(xtensa, XT_INS_DHWBI(xtensa, XT_REG_A3, 4));
			}
		}
		if (issue_ihi) {
			xtensa_queue_exec_ins(xtensa, XT_INS_IHI(xtensa, XT_REG_A3, 0));
			if (!same_ic_line) {
				LOG_TARGET_DEBUG(target,
					"IHI second icache line for address "TARGET_ADDR_FMT,
					address + 4);
				xtensa_queue_exec_ins(xtensa, XT_INS_IHI(xtensa, XT_REG_A3, 4));
			}
		}

		/* Execute invalidate instructions */
		ret = xtensa_dm_queue_execute(&xtensa->dbg_mod);
		xtensa_core_status_check(target);
		if (ret != ERROR_OK) {
			LOG_ERROR("Error issuing cache invaldate instruction(s): %d", ret);
			return ret;
		}
	}

	/* Write new instructions to memory */
	ret = target_write_buffer(target, address, size, buffer);
	if (ret != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Error writing instruction to memory: %d", ret);
		return ret;
	}

	if (issue_dhwbi) {
		/* Flush dcache so instruction propagates.  A3 may be corrupted during memory write */
		xtensa_queue_dbg_reg_write(xtensa, XDMREG_DDR, address);
		xtensa_queue_exec_ins(xtensa, XT_INS_RSR(xtensa, XT_SR_DDR, XT_REG_A3));
		xtensa_queue_exec_ins(xtensa, XT_INS_DHWB(xtensa, XT_REG_A3, 0));
		LOG_DEBUG("DHWB dcache line for address "TARGET_ADDR_FMT, address);
		if (!same_dc_line) {
			LOG_TARGET_DEBUG(target, "DHWB second dcache line for address "TARGET_ADDR_FMT, address + 4);
			xtensa_queue_exec_ins(xtensa, XT_INS_DHWB(xtensa, XT_REG_A3, 4));
		}

		/* Execute invalidate instructions */
		ret = xtensa_dm_queue_execute(&xtensa->dbg_mod);
		xtensa_core_status_check(target);
	}

	/* TODO: Handle L2 cache if present */
	return ret;
}

static int xtensa_sw_breakpoint_add(struct target *target,
	struct breakpoint *breakpoint,
	struct xtensa_sw_breakpoint *sw_bp)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int ret = target_read_buffer(target, breakpoint->address, XT_ISNS_SZ_MAX, sw_bp->insn);
	if (ret != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to read original instruction (%d)!", ret);
		return ret;
	}

	sw_bp->insn_sz = MIN(XT_ISNS_SZ_MAX, breakpoint->length);
	sw_bp->oocd_bp = breakpoint;

	uint32_t break_insn = sw_bp->insn_sz == XT_ISNS_SZ_MAX ? XT_INS_BREAK(xtensa, 0, 0) : XT_INS_BREAKN(xtensa, 0);

	/* Underlying memory write will convert instruction endianness, don't do that here */
	ret = xtensa_update_instruction(target, breakpoint->address, sw_bp->insn_sz, (uint8_t *)&break_insn);
	if (ret != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to write breakpoint instruction (%d)!", ret);
		return ret;
	}

	return ERROR_OK;
}

static int xtensa_sw_breakpoint_remove(struct target *target, struct xtensa_sw_breakpoint *sw_bp)
{
	int ret = xtensa_update_instruction(target, sw_bp->oocd_bp->address, sw_bp->insn_sz, sw_bp->insn);
	if (ret != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to write insn (%d)!", ret);
		return ret;
	}
	sw_bp->oocd_bp = NULL;
	return ERROR_OK;
}

int xtensa_breakpoint_add(struct target *target, struct breakpoint *breakpoint)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	unsigned int slot;

	if (breakpoint->type == BKPT_SOFT) {
		for (slot = 0; slot < XT_SW_BREAKPOINTS_MAX_NUM; slot++) {
			if (!xtensa->sw_brps[slot].oocd_bp ||
				xtensa->sw_brps[slot].oocd_bp == breakpoint)
				break;
		}
		if (slot == XT_SW_BREAKPOINTS_MAX_NUM) {
			LOG_TARGET_WARNING(target, "No free slots to add SW breakpoint!");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		int ret = xtensa_sw_breakpoint_add(target, breakpoint, &xtensa->sw_brps[slot]);
		if (ret != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Failed to add SW breakpoint!");
			return ret;
		}
		LOG_TARGET_DEBUG(target, "placed SW breakpoint %u @ " TARGET_ADDR_FMT,
			slot,
			breakpoint->address);
		return ERROR_OK;
	}

	for (slot = 0; slot < xtensa->core_config->debug.ibreaks_num; slot++) {
		if (!xtensa->hw_brps[slot] || xtensa->hw_brps[slot] == breakpoint)
			break;
	}
	if (slot == xtensa->core_config->debug.ibreaks_num) {
		LOG_TARGET_ERROR(target, "No free slots to add HW breakpoint!");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	xtensa->hw_brps[slot] = breakpoint;
	/* We will actually write the breakpoints when we resume the target. */
	LOG_TARGET_DEBUG(target, "placed HW breakpoint %u @ " TARGET_ADDR_FMT,
		slot,
		breakpoint->address);

	return ERROR_OK;
}

int xtensa_breakpoint_remove(struct target *target, struct breakpoint *breakpoint)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	unsigned int slot;

	if (breakpoint->type == BKPT_SOFT) {
		for (slot = 0; slot < XT_SW_BREAKPOINTS_MAX_NUM; slot++) {
			if (xtensa->sw_brps[slot].oocd_bp && xtensa->sw_brps[slot].oocd_bp == breakpoint)
				break;
		}
		if (slot == XT_SW_BREAKPOINTS_MAX_NUM) {
			LOG_TARGET_WARNING(target, "Max SW breakpoints slot reached, slot=%u!", slot);
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
		int ret = xtensa_sw_breakpoint_remove(target, &xtensa->sw_brps[slot]);
		if (ret != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Failed to remove SW breakpoint (%d)!", ret);
			return ret;
		}
		LOG_TARGET_DEBUG(target, "cleared SW breakpoint %u @ " TARGET_ADDR_FMT, slot, breakpoint->address);
		return ERROR_OK;
	}

	for (slot = 0; slot < xtensa->core_config->debug.ibreaks_num; slot++) {
		if (xtensa->hw_brps[slot] == breakpoint)
			break;
	}
	if (slot == xtensa->core_config->debug.ibreaks_num) {
		LOG_TARGET_ERROR(target, "HW breakpoint not found!");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	xtensa->hw_brps[slot] = NULL;
	LOG_TARGET_DEBUG(target, "cleared HW breakpoint %u @ " TARGET_ADDR_FMT, slot, breakpoint->address);
	return ERROR_OK;
}

int xtensa_watchpoint_add(struct target *target, struct watchpoint *watchpoint)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	unsigned int slot;
	xtensa_reg_val_t dbreakcval;

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_WARNING(target, "target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (watchpoint->mask != ~(uint32_t)0) {
		LOG_TARGET_ERROR(target, "watchpoint value masks not supported");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	for (slot = 0; slot < xtensa->core_config->debug.dbreaks_num; slot++) {
		if (!xtensa->hw_wps[slot] || xtensa->hw_wps[slot] == watchpoint)
			break;
	}
	if (slot == xtensa->core_config->debug.dbreaks_num) {
		LOG_TARGET_WARNING(target, "No free slots to add HW watchpoint!");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* Figure out value for dbreakc5..0
	 * It's basically 0x3F with an incremental bit removed from the LSB for each extra length power of 2. */
	if (watchpoint->length < 1 || watchpoint->length > 64 ||
		!IS_PWR_OF_2(watchpoint->length) ||
		!IS_ALIGNED(watchpoint->address, watchpoint->length)) {
		LOG_TARGET_WARNING(
			target,
			"Watchpoint with length %d on address " TARGET_ADDR_FMT
			" not supported by hardware.",
			watchpoint->length,
			watchpoint->address);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	dbreakcval = ALIGN_DOWN(0x3F, watchpoint->length);

	if (watchpoint->rw == WPT_READ)
		dbreakcval |= BIT(30);
	if (watchpoint->rw == WPT_WRITE)
		dbreakcval |= BIT(31);
	if (watchpoint->rw == WPT_ACCESS)
		dbreakcval |= BIT(30) | BIT(31);

	/* Write DBREAKA[slot] and DBCREAKC[slot] */
	xtensa_reg_set(target, XT_REG_IDX_DBREAKA0 + slot, watchpoint->address);
	xtensa_reg_set(target, XT_REG_IDX_DBREAKC0 + slot, dbreakcval);
	xtensa->hw_wps[slot] = watchpoint;
	LOG_TARGET_DEBUG(target, "placed HW watchpoint @ " TARGET_ADDR_FMT,
		watchpoint->address);
	return ERROR_OK;
}

int xtensa_watchpoint_remove(struct target *target, struct watchpoint *watchpoint)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	unsigned int slot;

	for (slot = 0; slot < xtensa->core_config->debug.dbreaks_num; slot++) {
		if (xtensa->hw_wps[slot] == watchpoint)
			break;
	}
	if (slot == xtensa->core_config->debug.dbreaks_num) {
		LOG_TARGET_WARNING(target, "HW watchpoint " TARGET_ADDR_FMT " not found!", watchpoint->address);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	xtensa_reg_set(target, XT_REG_IDX_DBREAKC0 + slot, 0);
	xtensa->hw_wps[slot] = NULL;
	LOG_TARGET_DEBUG(target, "cleared HW watchpoint @ " TARGET_ADDR_FMT,
		watchpoint->address);
	return ERROR_OK;
}

static int xtensa_build_reg_cache(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	struct reg_cache **cache_p = register_get_last_cache_p(&target->reg_cache);
	unsigned int last_dbreg_num = 0;

	if (xtensa->core_regs_num + xtensa->num_optregs != xtensa->total_regs_num)
		LOG_TARGET_WARNING(target, "Register count MISMATCH: %d core regs, %d extended regs; %d expected",
			xtensa->core_regs_num, xtensa->num_optregs, xtensa->total_regs_num);

	struct reg_cache *reg_cache = calloc(1, sizeof(struct reg_cache));

	if (!reg_cache) {
		LOG_ERROR("Failed to alloc reg cache!");
		return ERROR_FAIL;
	}
	reg_cache->name = "Xtensa registers";
	reg_cache->next = NULL;
	/* Init reglist */
	unsigned int reg_list_size = XT_NUM_REGS + xtensa->num_optregs;
	struct reg *reg_list = calloc(reg_list_size, sizeof(struct reg));
	if (!reg_list) {
		LOG_ERROR("Failed to alloc reg list!");
		goto fail;
	}
	xtensa->dbregs_num = 0;
	unsigned int didx = 0;
	for (unsigned int whichlist = 0; whichlist < 2; whichlist++) {
		struct xtensa_reg_desc *rlist = (whichlist == 0) ? xtensa_regs : xtensa->optregs;
		unsigned int listsize = (whichlist == 0) ? XT_NUM_REGS : xtensa->num_optregs;
		for (unsigned int i = 0; i < listsize; i++, didx++) {
			reg_list[didx].exist = rlist[i].exist;
			reg_list[didx].name = rlist[i].name;
			reg_list[didx].size = 32;
			reg_list[didx].value = calloc(1, 4 /*XT_REG_LEN*/);	/* make Clang Static Analyzer happy */
			if (!reg_list[didx].value) {
				LOG_ERROR("Failed to alloc reg list value!");
				goto fail;
			}
			reg_list[didx].dirty = false;
			reg_list[didx].valid = false;
			reg_list[didx].type = &xtensa_reg_type;
			reg_list[didx].arch_info = xtensa;
			if (rlist[i].exist && (rlist[i].dbreg_num > last_dbreg_num))
				last_dbreg_num = rlist[i].dbreg_num;

			if (xtensa_extra_debug_log) {
				LOG_TARGET_DEBUG(target,
					"POPULATE %-16s list %d exist %d, idx %d, type %d, dbreg_num 0x%04x",
					reg_list[didx].name,
					whichlist,
					reg_list[didx].exist,
					didx,
					rlist[i].type,
					rlist[i].dbreg_num);
			}
		}
	}

	xtensa->dbregs_num = last_dbreg_num + 1;
	reg_cache->reg_list = reg_list;
	reg_cache->num_regs = reg_list_size;

	LOG_TARGET_DEBUG(target, "xtensa->total_regs_num %d reg_list_size %d xtensa->dbregs_num %d",
		xtensa->total_regs_num, reg_list_size, xtensa->dbregs_num);

	/* Construct empty-register list for handling unknown register requests */
	xtensa->empty_regs = calloc(xtensa->dbregs_num, sizeof(struct reg));
	if (!xtensa->empty_regs) {
		LOG_TARGET_ERROR(target, "ERROR: Out of memory");
		goto fail;
	}
	for (unsigned int i = 0; i < xtensa->dbregs_num; i++) {
		xtensa->empty_regs[i].name = calloc(8, sizeof(char));
		if (!xtensa->empty_regs[i].name) {
			LOG_TARGET_ERROR(target, "ERROR: Out of memory");
			goto fail;
		}
		sprintf((char *)xtensa->empty_regs[i].name, "?0x%04x", i & 0x0000FFFF);
		xtensa->empty_regs[i].size = 32;
		xtensa->empty_regs[i].type = &xtensa_reg_type;
		xtensa->empty_regs[i].value = calloc(1, 4 /*XT_REG_LEN*/);	/* make Clang Static Analyzer happy */
		if (!xtensa->empty_regs[i].value) {
			LOG_ERROR("Failed to alloc empty reg list value!");
			goto fail;
		}
		xtensa->empty_regs[i].arch_info = xtensa;
	}

	/* Construct contiguous register list from contiguous descriptor list */
	if (xtensa->regmap_contiguous && xtensa->contiguous_regs_desc) {
		xtensa->contiguous_regs_list = calloc(xtensa->total_regs_num, sizeof(struct reg *));
		if (!xtensa->contiguous_regs_list) {
			LOG_TARGET_ERROR(target, "ERROR: Out of memory");
			goto fail;
		}
		for (unsigned int i = 0; i < xtensa->total_regs_num; i++) {
			unsigned int j;
			for (j = 0; j < reg_cache->num_regs; j++) {
				if (!strcmp(reg_cache->reg_list[j].name, xtensa->contiguous_regs_desc[i]->name)) {
					xtensa->contiguous_regs_list[i] = &(reg_cache->reg_list[j]);
					LOG_TARGET_DEBUG(target,
						"POPULATE contiguous regs list: %-16s, dbreg_num 0x%04x",
						xtensa->contiguous_regs_list[i]->name,
						xtensa->contiguous_regs_desc[i]->dbreg_num);
					break;
				}
			}
			if (j == reg_cache->num_regs)
				LOG_TARGET_WARNING(target, "contiguous register %s not found",
					xtensa->contiguous_regs_desc[i]->name);
		}
	}

	xtensa->algo_context_backup = calloc(reg_cache->num_regs, sizeof(void *));
	if (!xtensa->algo_context_backup) {
		LOG_ERROR("Failed to alloc mem for algorithm context backup!");
		goto fail;
	}
	for (unsigned int i = 0; i < reg_cache->num_regs; i++) {
		struct reg *reg = &reg_cache->reg_list[i];
		xtensa->algo_context_backup[i] = calloc(1, reg->size / 8);
		if (!xtensa->algo_context_backup[i]) {
			LOG_ERROR("Failed to alloc mem for algorithm context!");
			goto fail;
		}
	}
	xtensa->core_cache = reg_cache;
	if (cache_p)
		*cache_p = reg_cache;
	return ERROR_OK;

fail:
	if (reg_list) {
		for (unsigned int i = 0; i < reg_list_size; i++)
			free(reg_list[i].value);
		free(reg_list);
	}
	if (xtensa->empty_regs) {
		for (unsigned int i = 0; i < xtensa->dbregs_num; i++) {
			free((void *)xtensa->empty_regs[i].name);
			free(xtensa->empty_regs[i].value);
		}
		free(xtensa->empty_regs);
	}
	if (xtensa->algo_context_backup) {
		for (unsigned int i = 0; i < reg_cache->num_regs; i++)
			free(xtensa->algo_context_backup[i]);
		free(xtensa->algo_context_backup);
	}
	free(reg_cache);

	return ERROR_FAIL;
}

static int32_t xtensa_gdbqc_parse_exec_tie_ops(struct target *target, char *opstr)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int32_t status = ERROR_COMMAND_ARGUMENT_INVALID;
	/* Process op[] list */
	while (opstr && (*opstr == ':')) {
		uint8_t ops[32];
		unsigned int oplen = strtoul(opstr + 1, &opstr, 16);
		if (oplen > 32) {
			LOG_TARGET_ERROR(target, "TIE access instruction too long (%d)\n", oplen);
			break;
		}
		unsigned int i = 0;
		while ((i < oplen) && opstr && (*opstr == ':'))
			ops[i++] = strtoul(opstr + 1, &opstr, 16);
		if (i != oplen) {
			LOG_TARGET_ERROR(target, "TIE access instruction malformed (%d)\n", i);
			break;
		}

		char insn_buf[128];
		sprintf(insn_buf, "Exec %d-byte TIE sequence: ", oplen);
		for (i = 0; i < oplen; i++)
			sprintf(insn_buf + strlen(insn_buf), "%02x:", ops[i]);
		LOG_TARGET_DEBUG(target, "%s", insn_buf);
		xtensa_queue_exec_ins_wide(xtensa, ops, oplen);	/* Handles endian-swap */
		status = ERROR_OK;
	}
	return status;
}

static int xtensa_gdbqc_qxtreg(struct target *target, const char *packet, char **response_p)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	bool iswrite = (packet[0] == 'Q');
	enum xtensa_qerr_e error;

	/* Read/write TIE register.  Requires spill location.
	 * qxtreg<num>:<len>:<oplen>:<op[0]>:<...>[:<oplen>:<op[0]>:<...>]
	 * Qxtreg<num>:<len>:<oplen>:<op[0]>:<...>[:<oplen>:<op[0]>:<...>]=<value>
	 */
	if (!(xtensa->spill_buf)) {
		LOG_ERROR("Spill location not specified. Try 'target remote <host>:3333 &spill_location0'");
		error = XT_QERR_FAIL;
		goto xtensa_gdbqc_qxtreg_fail;
	}

	char *delim;
	uint32_t regnum = strtoul(packet + 6, &delim, 16);
	if (*delim != ':') {
		LOG_ERROR("Malformed qxtreg packet");
		error = XT_QERR_INVAL;
		goto xtensa_gdbqc_qxtreg_fail;
	}
	uint32_t reglen = strtoul(delim + 1, &delim, 16);
	if (*delim != ':') {
		LOG_ERROR("Malformed qxtreg packet");
		error = XT_QERR_INVAL;
		goto xtensa_gdbqc_qxtreg_fail;
	}
	uint8_t regbuf[XT_QUERYPKT_RESP_MAX];
	memset(regbuf, 0, XT_QUERYPKT_RESP_MAX);
	LOG_DEBUG("TIE reg 0x%08" PRIx32 " %s (%d bytes)", regnum, iswrite ? "write" : "read", reglen);
	if (reglen * 2 + 1 > XT_QUERYPKT_RESP_MAX) {
		LOG_ERROR("TIE register too large");
		error = XT_QERR_MEM;
		goto xtensa_gdbqc_qxtreg_fail;
	}

	/* (1) Save spill memory, (1.5) [if write then store value to spill location],
	 * (2) read old a4, (3) write spill address to a4.
	 * NOTE: ensure a4 is restored properly by all error handling logic
	 */
	unsigned int memop_size = (xtensa->spill_loc & 3) ? 1 : 4;
	int status = xtensa_read_memory(target, xtensa->spill_loc, memop_size,
		xtensa->spill_bytes / memop_size, xtensa->spill_buf);
	if (status != ERROR_OK) {
		LOG_ERROR("Spill memory save");
		error = XT_QERR_MEM;
		goto xtensa_gdbqc_qxtreg_fail;
	}
	if (iswrite) {
		/* Extract value and store in spill memory */
		unsigned int b = 0;
		char *valbuf = strchr(delim, '=');
		if (!(valbuf && (*valbuf == '='))) {
			LOG_ERROR("Malformed Qxtreg packet");
			error = XT_QERR_INVAL;
			goto xtensa_gdbqc_qxtreg_fail;
		}
		valbuf++;
		while (*valbuf && *(valbuf + 1)) {
			char bytestr[3] = { 0, 0, 0 };
			strncpy(bytestr, valbuf, 2);
			regbuf[b++] = strtoul(bytestr, NULL, 16);
			valbuf += 2;
		}
		if (b != reglen) {
			LOG_ERROR("Malformed Qxtreg packet");
			error = XT_QERR_INVAL;
			goto xtensa_gdbqc_qxtreg_fail;
		}
		status = xtensa_write_memory(target, xtensa->spill_loc, memop_size,
			reglen / memop_size, regbuf);
		if (status != ERROR_OK) {
			LOG_ERROR("TIE value store");
			error = XT_QERR_MEM;
			goto xtensa_gdbqc_qxtreg_fail;
		}
	}
	xtensa_reg_val_t orig_a4 = xtensa_reg_get(target, XT_REG_IDX_A4);
	xtensa_queue_dbg_reg_write(xtensa, XDMREG_DDR, xtensa->spill_loc);
	xtensa_queue_exec_ins(xtensa, XT_INS_RSR(xtensa, XT_SR_DDR, XT_REG_A4));

	int32_t tieop_status = xtensa_gdbqc_parse_exec_tie_ops(target, delim);

	/* Restore a4 but not yet spill memory.  Execute it all... */
	xtensa_queue_dbg_reg_write(xtensa, XDMREG_DDR, orig_a4);
	xtensa_queue_exec_ins(xtensa, XT_INS_RSR(xtensa, XT_SR_DDR, XT_REG_A4));
	status = xtensa_dm_queue_execute(&xtensa->dbg_mod);
	if (status != ERROR_OK) {
		LOG_TARGET_ERROR(target, "TIE queue execute: %d\n", status);
		tieop_status = status;
	}
	status = xtensa_core_status_check(target);
	if (status != ERROR_OK) {
		LOG_TARGET_ERROR(target, "TIE instr execute: %d\n", status);
		tieop_status = status;
	}

	if (tieop_status == ERROR_OK) {
		if (iswrite) {
			/* TIE write succeeded; send OK */
			strcpy(*response_p, "OK");
		} else {
			/* TIE read succeeded; copy result from spill memory */
			status = xtensa_read_memory(target, xtensa->spill_loc, memop_size, reglen, regbuf);
			if (status != ERROR_OK) {
				LOG_TARGET_ERROR(target, "TIE result read");
				tieop_status = status;
			}
			unsigned int i;
			for (i = 0; i < reglen; i++)
				sprintf(*response_p + 2 * i, "%02x", regbuf[i]);
			*(*response_p + 2 * i) = '\0';
			LOG_TARGET_DEBUG(target, "TIE response: %s", *response_p);
		}
	}

	/* Restore spill memory first, then report any previous errors */
	status = xtensa_write_memory(target, xtensa->spill_loc, memop_size,
		xtensa->spill_bytes / memop_size, xtensa->spill_buf);
	if (status != ERROR_OK) {
		LOG_ERROR("Spill memory restore");
		error = XT_QERR_MEM;
		goto xtensa_gdbqc_qxtreg_fail;
	}
	if (tieop_status != ERROR_OK) {
		LOG_ERROR("TIE execution");
		error = XT_QERR_FAIL;
		goto xtensa_gdbqc_qxtreg_fail;
	}
	return ERROR_OK;

xtensa_gdbqc_qxtreg_fail:
	strcpy(*response_p, xt_qerr[error].chrval);
	return xt_qerr[error].intval;
}

int xtensa_gdb_query_custom(struct target *target, const char *packet, char **response_p)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	enum xtensa_qerr_e error;
	if (!packet || !response_p) {
		LOG_TARGET_ERROR(target, "invalid parameter: packet %p response_p %p", packet, response_p);
		return ERROR_FAIL;
	}

	*response_p = xtensa->qpkt_resp;
	if (strncmp(packet, "qxtn", 4) == 0) {
		strcpy(*response_p, "OpenOCD");
		return ERROR_OK;
	} else if (strncasecmp(packet, "qxtgdbversion=", 14) == 0) {
		return ERROR_OK;
	} else if ((strncmp(packet, "Qxtsis=", 7) == 0) || (strncmp(packet, "Qxtsds=", 7) == 0)) {
		/* Confirm host cache params match core .cfg file */
		struct xtensa_cache_config *cachep = (packet[4] == 'i') ?
			&xtensa->core_config->icache : &xtensa->core_config->dcache;
		unsigned int line_size = 0, size = 0, way_count = 0;
		sscanf(&packet[7], "%x,%x,%x", &line_size, &size, &way_count);
		if ((cachep->line_size != line_size) ||
			(cachep->size != size) ||
			(cachep->way_count != way_count)) {
			LOG_TARGET_WARNING(target, "%cCache mismatch; check xtensa-core-XXX.cfg file",
				cachep == &xtensa->core_config->icache ? 'I' : 'D');
		}
		strcpy(*response_p, "OK");
		return ERROR_OK;
	} else if ((strncmp(packet, "Qxtiram=", 8) == 0) || (strncmp(packet, "Qxtirom=", 8) == 0)) {
		/* Confirm host IRAM/IROM params match core .cfg file */
		struct xtensa_local_mem_config *memp = (packet[5] == 'a') ?
			&xtensa->core_config->iram : &xtensa->core_config->irom;
		unsigned int base = 0, size = 0, i;
		char *pkt = (char *)&packet[7];
		do {
			pkt++;
			size = strtoul(pkt, &pkt, 16);
			pkt++;
			base = strtoul(pkt, &pkt, 16);
			LOG_TARGET_DEBUG(target, "memcheck: %dB @ 0x%08x", size, base);
			for (i = 0; i < memp->count; i++) {
				if ((memp->regions[i].base == base) && (memp->regions[i].size == size))
					break;
			}
			if (i == memp->count) {
				LOG_TARGET_WARNING(target, "%s mismatch; check xtensa-core-XXX.cfg file",
					memp == &xtensa->core_config->iram ? "IRAM" : "IROM");
				break;
			}
			for (i = 0; i < 11; i++) {
				pkt++;
				strtoul(pkt, &pkt, 16);
			}
		} while (pkt && (pkt[0] == ','));
		strcpy(*response_p, "OK");
		return ERROR_OK;
	} else if (strncmp(packet, "Qxtexcmlvl=", 11) == 0) {
		/* Confirm host EXCM_LEVEL matches core .cfg file */
		unsigned int excm_level = strtoul(&packet[11], NULL, 0);
		if (!xtensa->core_config->high_irq.enabled ||
			(excm_level != xtensa->core_config->high_irq.excm_level))
			LOG_TARGET_WARNING(target, "EXCM_LEVEL mismatch; check xtensa-core-XXX.cfg file");
		strcpy(*response_p, "OK");
		return ERROR_OK;
	} else if ((strncmp(packet, "Qxtl2cs=", 8) == 0) ||
		(strncmp(packet, "Qxtl2ca=", 8) == 0) ||
		(strncmp(packet, "Qxtdensity=", 11) == 0)) {
		strcpy(*response_p, "OK");
		return ERROR_OK;
	} else if (strncmp(packet, "Qxtspill=", 9) == 0) {
		char *delim;
		uint32_t spill_loc = strtoul(packet + 9, &delim, 16);
		if (*delim != ':') {
			LOG_ERROR("Malformed Qxtspill packet");
			error = XT_QERR_INVAL;
			goto xtensa_gdb_query_custom_fail;
		}
		xtensa->spill_loc = spill_loc;
		xtensa->spill_bytes = strtoul(delim + 1, NULL, 16);
		if (xtensa->spill_buf)
			free(xtensa->spill_buf);
		xtensa->spill_buf = calloc(1, xtensa->spill_bytes);
		if (!xtensa->spill_buf) {
			LOG_ERROR("Spill buf alloc");
			error = XT_QERR_MEM;
			goto xtensa_gdb_query_custom_fail;
		}
		LOG_TARGET_DEBUG(target, "Set spill 0x%08" PRIx32 " (%d)", xtensa->spill_loc, xtensa->spill_bytes);
		strcpy(*response_p, "OK");
		return ERROR_OK;
	} else if (strncasecmp(packet, "qxtreg", 6) == 0) {
		return xtensa_gdbqc_qxtreg(target, packet, response_p);
	} else if ((strncmp(packet, "qTStatus", 8) == 0) ||
		(strncmp(packet, "qxtftie", 7) == 0) ||
		(strncmp(packet, "qxtstie", 7) == 0)) {
		/* Return empty string to indicate trace, TIE wire debug are unsupported */
		strcpy(*response_p, "");
		return ERROR_OK;
	}

	/* Warn for all other queries, but do not return errors */
	LOG_TARGET_WARNING(target, "Unknown target-specific query packet: %s", packet);
	strcpy(*response_p, "");
	return ERROR_OK;

xtensa_gdb_query_custom_fail:
	strcpy(*response_p, xt_qerr[error].chrval);
	return xt_qerr[error].intval;
}

int xtensa_init_arch_info(struct target *target, struct xtensa *xtensa,
	const struct xtensa_debug_module_config *dm_cfg)
{
	target->arch_info = xtensa;
	xtensa->common_magic = XTENSA_COMMON_MAGIC;
	xtensa->target = target;
	xtensa->stepping_isr_mode = XT_STEPPING_ISR_ON;

	xtensa->core_config = calloc(1, sizeof(struct xtensa_config));
	if (!xtensa->core_config) {
		LOG_ERROR("Xtensa configuration alloc failed\n");
		return ERROR_FAIL;
	}

	/* Default cache settings are disabled with 1 way */
	xtensa->core_config->icache.way_count = 1;
	xtensa->core_config->dcache.way_count = 1;

	/* chrval: AR3/AR4 register names will change with window mapping.
	 * intval: tracks whether scratch register was set through gdb P packet.
	 */
	for (enum xtensa_ar_scratch_set_e s = 0; s < XT_AR_SCRATCH_NUM; s++) {
		xtensa->scratch_ars[s].chrval = calloc(8, sizeof(char));
		if (!xtensa->scratch_ars[s].chrval) {
			for (enum xtensa_ar_scratch_set_e f = 0; f < s; f++)
				free(xtensa->scratch_ars[f].chrval);
			free(xtensa->core_config);
			LOG_ERROR("Xtensa scratch AR alloc failed\n");
			return ERROR_FAIL;
		}
		xtensa->scratch_ars[s].intval = false;
		sprintf(xtensa->scratch_ars[s].chrval, "%s%d",
			((s == XT_AR_SCRATCH_A3) || (s == XT_AR_SCRATCH_A4)) ? "a" : "ar",
			((s == XT_AR_SCRATCH_A3) || (s == XT_AR_SCRATCH_AR3)) ? 3 : 4);
	}

	return xtensa_dm_init(&xtensa->dbg_mod, dm_cfg);
}

void xtensa_set_permissive_mode(struct target *target, bool state)
{
	target_to_xtensa(target)->permissive_mode = state;
}

int xtensa_target_init(struct command_context *cmd_ctx, struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);

	xtensa->come_online_probes_num = 3;
	xtensa->hw_brps = calloc(XT_HW_IBREAK_MAX_NUM, sizeof(struct breakpoint *));
	if (!xtensa->hw_brps) {
		LOG_ERROR("Failed to alloc memory for HW breakpoints!");
		return ERROR_FAIL;
	}
	xtensa->hw_wps = calloc(XT_HW_DBREAK_MAX_NUM, sizeof(struct watchpoint *));
	if (!xtensa->hw_wps) {
		free(xtensa->hw_brps);
		LOG_ERROR("Failed to alloc memory for HW watchpoints!");
		return ERROR_FAIL;
	}
	xtensa->sw_brps = calloc(XT_SW_BREAKPOINTS_MAX_NUM, sizeof(struct xtensa_sw_breakpoint));
	if (!xtensa->sw_brps) {
		free(xtensa->hw_brps);
		free(xtensa->hw_wps);
		LOG_ERROR("Failed to alloc memory for SW breakpoints!");
		return ERROR_FAIL;
	}

	xtensa->spill_loc = 0xffffffff;
	xtensa->spill_bytes = 0;
	xtensa->spill_buf = NULL;
	xtensa->probe_lsddr32p = -1;	/* Probe for fast load/store operations */

	return xtensa_build_reg_cache(target);
}

static void xtensa_free_reg_cache(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	struct reg_cache *cache = xtensa->core_cache;

	if (cache) {
		register_unlink_cache(&target->reg_cache, cache);
		for (unsigned int i = 0; i < cache->num_regs; i++) {
			free(xtensa->algo_context_backup[i]);
			free(cache->reg_list[i].value);
		}
		free(xtensa->algo_context_backup);
		free(cache->reg_list);
		free(cache);
	}
	xtensa->core_cache = NULL;
	xtensa->algo_context_backup = NULL;

	if (xtensa->empty_regs) {
		for (unsigned int i = 0; i < xtensa->dbregs_num; i++) {
			free((void *)xtensa->empty_regs[i].name);
			free(xtensa->empty_regs[i].value);
		}
		free(xtensa->empty_regs);
	}
	xtensa->empty_regs = NULL;
	if (xtensa->optregs) {
		for (unsigned int i = 0; i < xtensa->num_optregs; i++)
			free((void *)xtensa->optregs[i].name);
		free(xtensa->optregs);
	}
	xtensa->optregs = NULL;
}

void xtensa_target_deinit(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);

	LOG_DEBUG("start");

	if (target_was_examined(target)) {
		int ret = xtensa_queue_dbg_reg_write(xtensa, XDMREG_DCRCLR, OCDDCR_ENABLEOCD);
		if (ret != ERROR_OK) {
			LOG_ERROR("Failed to queue OCDDCR_ENABLEOCD clear operation!");
			return;
		}
		xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
		ret = xtensa_dm_queue_execute(&xtensa->dbg_mod);
		if (ret != ERROR_OK) {
			LOG_ERROR("Failed to clear OCDDCR_ENABLEOCD!");
			return;
		}
		xtensa_dm_deinit(&xtensa->dbg_mod);
	}
	xtensa_free_reg_cache(target);
	free(xtensa->hw_brps);
	free(xtensa->hw_wps);
	free(xtensa->sw_brps);
	if (xtensa->spill_buf) {
		free(xtensa->spill_buf);
		xtensa->spill_buf = NULL;
	}
	for (enum xtensa_ar_scratch_set_e s = 0; s < XT_AR_SCRATCH_NUM; s++)
		free(xtensa->scratch_ars[s].chrval);
	free(xtensa->core_config);
}

const char *xtensa_get_gdb_arch(struct target *target)
{
	return "xtensa";
}

/* exe <ascii-encoded hexadecimal instruction bytes> */
static COMMAND_HELPER(xtensa_cmd_exe_do, struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* Process ascii-encoded hex byte string */
	const char *parm = CMD_ARGV[0];
	unsigned int parm_len = strlen(parm);
	if ((parm_len >= 64) || (parm_len & 1)) {
		LOG_ERROR("Invalid parameter length (%d): must be even, < 64 characters", parm_len);
		return ERROR_FAIL;
	}

	uint8_t ops[32];
	memset(ops, 0, 32);
	unsigned int oplen = parm_len / 2;
	char encoded_byte[3] = { 0, 0, 0 };
	for (unsigned int i = 0; i < oplen; i++) {
		encoded_byte[0] = *parm++;
		encoded_byte[1] = *parm++;
		ops[i] = strtoul(encoded_byte, NULL, 16);
	}

	/* GDB must handle state save/restore.
	 * Flush reg cache in case spill location is in an AR
	 * Update CPENABLE only for this execution; later restore cached copy
	 * Keep a copy of exccause in case executed code triggers an exception
	 */
	int status = xtensa_write_dirty_registers(target);
	if (status != ERROR_OK) {
		LOG_ERROR("%s: Failed to write back register cache.", target_name(target));
		return ERROR_FAIL;
	}
	xtensa_reg_val_t exccause = xtensa_reg_get(target, XT_REG_IDX_EXCCAUSE);
	xtensa_reg_val_t cpenable = xtensa_reg_get(target, XT_REG_IDX_CPENABLE);
	xtensa_reg_val_t a3 = xtensa_reg_get(target, XT_REG_IDX_A3);
	xtensa_queue_dbg_reg_write(xtensa, XDMREG_DDR, 0xffffffff);
	xtensa_queue_exec_ins(xtensa, XT_INS_RSR(xtensa, XT_SR_DDR, XT_REG_A3));
	xtensa_queue_exec_ins(xtensa, XT_INS_WSR(xtensa,
			xtensa_regs[XT_REG_IDX_CPENABLE].reg_num, XT_REG_A3));
	xtensa_queue_dbg_reg_write(xtensa, XDMREG_DDR, a3);
	xtensa_queue_exec_ins(xtensa, XT_INS_RSR(xtensa, XT_SR_DDR, XT_REG_A3));

	/* Queue instruction list and execute everything */
	LOG_TARGET_DEBUG(target, "execute stub: %s", CMD_ARGV[0]);
	xtensa_queue_exec_ins_wide(xtensa, ops, oplen);	/* Handles endian-swap */
	status = xtensa_dm_queue_execute(&xtensa->dbg_mod);
	if (status != ERROR_OK)
		LOG_TARGET_ERROR(target, "TIE queue execute: %d\n", status);
	status = xtensa_core_status_check(target);
	if (status != ERROR_OK)
		LOG_TARGET_ERROR(target, "TIE instr execute: %d\n", status);

	/* Reread register cache and restore saved regs after instruction execution */
	if (xtensa_fetch_all_regs(target) != ERROR_OK)
		LOG_TARGET_ERROR(target, "%s: Failed to fetch register cache (post-exec).", target_name(target));
	xtensa_reg_set(target, XT_REG_IDX_EXCCAUSE, exccause);
	xtensa_reg_set(target, XT_REG_IDX_CPENABLE, cpenable);
	return status;
}

COMMAND_HANDLER(xtensa_cmd_exe)
{
	return CALL_COMMAND_HANDLER(xtensa_cmd_exe_do, get_current_target(CMD_CTX));
}

/* xtdef <name> */
COMMAND_HELPER(xtensa_cmd_xtdef_do, struct xtensa *xtensa)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	const char *core_name = CMD_ARGV[0];
	if (strcasecmp(core_name, "LX") == 0) {
		xtensa->core_config->core_type = XT_LX;
	} else {
		LOG_ERROR("xtdef [LX]\n");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(xtensa_cmd_xtdef)
{
	return CALL_COMMAND_HANDLER(xtensa_cmd_xtdef_do,
		target_to_xtensa(get_current_target(CMD_CTX)));
}

static inline bool xtensa_cmd_xtopt_legal_val(char *opt, int val, int min, int max)
{
	if ((val < min) || (val > max)) {
		LOG_ERROR("xtopt %s (%d) out of range [%d..%d]\n", opt, val, min, max);
		return false;
	}
	return true;
}

/* xtopt <name> <value> */
COMMAND_HELPER(xtensa_cmd_xtopt_do, struct xtensa *xtensa)
{
	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	const char *opt_name = CMD_ARGV[0];
	int opt_val = strtol(CMD_ARGV[1], NULL, 0);
	if (strcasecmp(opt_name, "arnum") == 0) {
		if (!xtensa_cmd_xtopt_legal_val("arnum", opt_val, 0, 64))
			return ERROR_COMMAND_ARGUMENT_INVALID;
		xtensa->core_config->aregs_num = opt_val;
	} else if (strcasecmp(opt_name, "windowed") == 0) {
		if (!xtensa_cmd_xtopt_legal_val("windowed", opt_val, 0, 1))
			return ERROR_COMMAND_ARGUMENT_INVALID;
		xtensa->core_config->windowed = opt_val;
	} else if (strcasecmp(opt_name, "cpenable") == 0) {
		if (!xtensa_cmd_xtopt_legal_val("cpenable", opt_val, 0, 1))
			return ERROR_COMMAND_ARGUMENT_INVALID;
		xtensa->core_config->coproc = opt_val;
	} else if (strcasecmp(opt_name, "exceptions") == 0) {
		if (!xtensa_cmd_xtopt_legal_val("exceptions", opt_val, 0, 1))
			return ERROR_COMMAND_ARGUMENT_INVALID;
		xtensa->core_config->exceptions = opt_val;
	} else if (strcasecmp(opt_name, "intnum") == 0) {
		if (!xtensa_cmd_xtopt_legal_val("intnum", opt_val, 0, 32))
			return ERROR_COMMAND_ARGUMENT_INVALID;
		xtensa->core_config->irq.enabled = (opt_val > 0);
		xtensa->core_config->irq.irq_num = opt_val;
	} else if (strcasecmp(opt_name, "hipriints") == 0) {
		if (!xtensa_cmd_xtopt_legal_val("hipriints", opt_val, 0, 1))
			return ERROR_COMMAND_ARGUMENT_INVALID;
		xtensa->core_config->high_irq.enabled = opt_val;
	} else if (strcasecmp(opt_name, "excmlevel") == 0) {
		if (!xtensa_cmd_xtopt_legal_val("excmlevel", opt_val, 1, 6))
			return ERROR_COMMAND_ARGUMENT_INVALID;
		if (!xtensa->core_config->high_irq.enabled) {
			LOG_ERROR("xtopt excmlevel requires hipriints\n");
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
		xtensa->core_config->high_irq.excm_level = opt_val;
	} else if (strcasecmp(opt_name, "intlevels") == 0) {
		if (xtensa->core_config->core_type == XT_LX) {
			if (!xtensa_cmd_xtopt_legal_val("intlevels", opt_val, 2, 6))
				return ERROR_COMMAND_ARGUMENT_INVALID;
		} else {
			if (!xtensa_cmd_xtopt_legal_val("intlevels", opt_val, 1, 255))
				return ERROR_COMMAND_ARGUMENT_INVALID;
		}
		if (!xtensa->core_config->high_irq.enabled) {
			LOG_ERROR("xtopt intlevels requires hipriints\n");
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
		xtensa->core_config->high_irq.level_num = opt_val;
	} else if (strcasecmp(opt_name, "debuglevel") == 0) {
		if (xtensa->core_config->core_type == XT_LX) {
			if (!xtensa_cmd_xtopt_legal_val("debuglevel", opt_val, 2, 6))
				return ERROR_COMMAND_ARGUMENT_INVALID;
		} else {
			if (!xtensa_cmd_xtopt_legal_val("debuglevel", opt_val, 0, 0))
				return ERROR_COMMAND_ARGUMENT_INVALID;
		}
		xtensa->core_config->debug.enabled = 1;
		xtensa->core_config->debug.irq_level = opt_val;
	} else if (strcasecmp(opt_name, "ibreaknum") == 0) {
		if (!xtensa_cmd_xtopt_legal_val("ibreaknum", opt_val, 0, 2))
			return ERROR_COMMAND_ARGUMENT_INVALID;
		xtensa->core_config->debug.ibreaks_num = opt_val;
	} else if (strcasecmp(opt_name, "dbreaknum") == 0) {
		if (!xtensa_cmd_xtopt_legal_val("dbreaknum", opt_val, 0, 2))
			return ERROR_COMMAND_ARGUMENT_INVALID;
		xtensa->core_config->debug.dbreaks_num = opt_val;
	} else if (strcasecmp(opt_name, "tracemem") == 0) {
		if (!xtensa_cmd_xtopt_legal_val("tracemem", opt_val, 0, 256 * 1024))
			return ERROR_COMMAND_ARGUMENT_INVALID;
		xtensa->core_config->trace.mem_sz = opt_val;
		xtensa->core_config->trace.enabled = (opt_val > 0);
	} else if (strcasecmp(opt_name, "tracememrev") == 0) {
		if (!xtensa_cmd_xtopt_legal_val("tracememrev", opt_val, 0, 1))
			return ERROR_COMMAND_ARGUMENT_INVALID;
		xtensa->core_config->trace.reversed_mem_access = opt_val;
	} else if (strcasecmp(opt_name, "perfcount") == 0) {
		if (!xtensa_cmd_xtopt_legal_val("perfcount", opt_val, 0, 8))
			return ERROR_COMMAND_ARGUMENT_INVALID;
		xtensa->core_config->debug.perfcount_num = opt_val;
	} else {
		LOG_WARNING("Unknown xtensa command ignored: \"xtopt %s %s\"", CMD_ARGV[0], CMD_ARGV[1]);
		return ERROR_OK;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(xtensa_cmd_xtopt)
{
	return CALL_COMMAND_HANDLER(xtensa_cmd_xtopt_do,
		target_to_xtensa(get_current_target(CMD_CTX)));
}

/* xtmem <type> [parameters] */
COMMAND_HELPER(xtensa_cmd_xtmem_do, struct xtensa *xtensa)
{
	struct xtensa_cache_config *cachep = NULL;
	struct xtensa_local_mem_config *memp = NULL;
	int mem_access = 0;
	bool is_dcache = false;

	if (CMD_ARGC == 0) {
		LOG_ERROR("xtmem <type> [parameters]\n");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	const char *mem_name = CMD_ARGV[0];
	if (strcasecmp(mem_name, "icache") == 0) {
		cachep = &xtensa->core_config->icache;
	} else if (strcasecmp(mem_name, "dcache") == 0) {
		cachep = &xtensa->core_config->dcache;
		is_dcache = true;
	} else if (strcasecmp(mem_name, "l2cache") == 0) {
		/* TODO: support L2 cache */
	} else if (strcasecmp(mem_name, "l2addr") == 0) {
		/* TODO: support L2 cache */
	} else if (strcasecmp(mem_name, "iram") == 0) {
		memp = &xtensa->core_config->iram;
		mem_access = XT_MEM_ACCESS_READ | XT_MEM_ACCESS_WRITE;
	} else if (strcasecmp(mem_name, "dram") == 0) {
		memp = &xtensa->core_config->dram;
		mem_access = XT_MEM_ACCESS_READ | XT_MEM_ACCESS_WRITE;
	} else if (strcasecmp(mem_name, "sram") == 0) {
		memp = &xtensa->core_config->sram;
		mem_access = XT_MEM_ACCESS_READ | XT_MEM_ACCESS_WRITE;
	} else if (strcasecmp(mem_name, "irom") == 0) {
		memp = &xtensa->core_config->irom;
		mem_access = XT_MEM_ACCESS_READ;
	} else if (strcasecmp(mem_name, "drom") == 0) {
		memp = &xtensa->core_config->drom;
		mem_access = XT_MEM_ACCESS_READ;
	} else if (strcasecmp(mem_name, "srom") == 0) {
		memp = &xtensa->core_config->srom;
		mem_access = XT_MEM_ACCESS_READ;
	} else {
		LOG_ERROR("xtmem types: <icache|dcache|l2cache|l2addr|iram|irom|dram|drom|sram|srom>\n");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	if (cachep) {
		if ((CMD_ARGC != 4) && (CMD_ARGC != 5)) {
			LOG_ERROR("xtmem <cachetype> <linebytes> <cachebytes> <ways> [writeback]\n");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		cachep->line_size = strtoul(CMD_ARGV[1], NULL, 0);
		cachep->size = strtoul(CMD_ARGV[2], NULL, 0);
		cachep->way_count = strtoul(CMD_ARGV[3], NULL, 0);
		cachep->writeback = ((CMD_ARGC == 5) && is_dcache) ?
			strtoul(CMD_ARGV[4], NULL, 0) : 0;
	} else if (memp) {
		if (CMD_ARGC != 3) {
			LOG_ERROR("xtmem <memtype> <baseaddr> <bytes>\n");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		struct xtensa_local_mem_region_config *memcfgp = &memp->regions[memp->count];
		memcfgp->base = strtoul(CMD_ARGV[1], NULL, 0);
		memcfgp->size = strtoul(CMD_ARGV[2], NULL, 0);
		memcfgp->access = mem_access;
		memp->count++;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(xtensa_cmd_xtmem)
{
	return CALL_COMMAND_HANDLER(xtensa_cmd_xtmem_do,
		target_to_xtensa(get_current_target(CMD_CTX)));
}

/* xtmpu <num FG seg> <min seg size> <lockable> <executeonly> */
COMMAND_HELPER(xtensa_cmd_xtmpu_do, struct xtensa *xtensa)
{
	if (CMD_ARGC != 4) {
		LOG_ERROR("xtmpu <num FG seg> <min seg size> <lockable> <executeonly>\n");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	unsigned int nfgseg = strtoul(CMD_ARGV[0], NULL, 0);
	unsigned int minsegsize = strtoul(CMD_ARGV[1], NULL, 0);
	unsigned int lockable = strtoul(CMD_ARGV[2], NULL, 0);
	unsigned int execonly = strtoul(CMD_ARGV[3], NULL, 0);

	if ((nfgseg > 32)) {
		LOG_ERROR("<nfgseg> must be within [0..32]\n");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	} else if (minsegsize & (minsegsize - 1)) {
		LOG_ERROR("<minsegsize> must be a power of 2 >= 32\n");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	} else if (lockable > 1) {
		LOG_ERROR("<lockable> must be 0 or 1\n");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	} else if (execonly > 1) {
		LOG_ERROR("<execonly> must be 0 or 1\n");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	xtensa->core_config->mpu.enabled = true;
	xtensa->core_config->mpu.nfgseg = nfgseg;
	xtensa->core_config->mpu.minsegsize = minsegsize;
	xtensa->core_config->mpu.lockable = lockable;
	xtensa->core_config->mpu.execonly = execonly;
	return ERROR_OK;
}

COMMAND_HANDLER(xtensa_cmd_xtmpu)
{
	return CALL_COMMAND_HANDLER(xtensa_cmd_xtmpu_do,
		target_to_xtensa(get_current_target(CMD_CTX)));
}

/* xtmmu <NIREFILLENTRIES> <NDREFILLENTRIES> <IVARWAY56> <DVARWAY56> */
COMMAND_HELPER(xtensa_cmd_xtmmu_do, struct xtensa *xtensa)
{
	if (CMD_ARGC != 2) {
		LOG_ERROR("xtmmu <NIREFILLENTRIES> <NDREFILLENTRIES>\n");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	unsigned int nirefillentries = strtoul(CMD_ARGV[0], NULL, 0);
	unsigned int ndrefillentries = strtoul(CMD_ARGV[1], NULL, 0);
	if ((nirefillentries != 16) && (nirefillentries != 32)) {
		LOG_ERROR("<nirefillentries> must be 16 or 32\n");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	} else if ((ndrefillentries != 16) && (ndrefillentries != 32)) {
		LOG_ERROR("<ndrefillentries> must be 16 or 32\n");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	xtensa->core_config->mmu.enabled = true;
	xtensa->core_config->mmu.itlb_entries_count = nirefillentries;
	xtensa->core_config->mmu.dtlb_entries_count = ndrefillentries;
	return ERROR_OK;
}

COMMAND_HANDLER(xtensa_cmd_xtmmu)
{
	return CALL_COMMAND_HANDLER(xtensa_cmd_xtmmu_do,
		target_to_xtensa(get_current_target(CMD_CTX)));
}

/* xtregs <numregs>
 * xtreg <regname> <regnum> */
COMMAND_HELPER(xtensa_cmd_xtreg_do, struct xtensa *xtensa)
{
	if (CMD_ARGC == 1) {
		int32_t numregs = strtoul(CMD_ARGV[0], NULL, 0);
		if ((numregs <= 0) || (numregs > UINT16_MAX)) {
			LOG_ERROR("xtreg <numregs>: Invalid 'numregs' (%d)", numregs);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		if ((xtensa->genpkt_regs_num > 0) && (numregs < (int32_t)xtensa->genpkt_regs_num)) {
			LOG_ERROR("xtregs (%d) must be larger than numgenregs (%d) (if xtregfmt specified)",
				numregs, xtensa->genpkt_regs_num);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		xtensa->total_regs_num = numregs;
		xtensa->core_regs_num = 0;
		xtensa->num_optregs = 0;
		/* A little more memory than required, but saves a second initialization pass */
		xtensa->optregs = calloc(xtensa->total_regs_num, sizeof(struct xtensa_reg_desc));
		if (!xtensa->optregs) {
			LOG_ERROR("Failed to allocate xtensa->optregs!");
			return ERROR_FAIL;
		}
		return ERROR_OK;
	} else if (CMD_ARGC != 2) {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* "xtregfmt contiguous" must be specified prior to the first "xtreg" definition
	 * if general register (g-packet) requests or contiguous register maps are supported */
	if (xtensa->regmap_contiguous && !xtensa->contiguous_regs_desc) {
		xtensa->contiguous_regs_desc = calloc(xtensa->total_regs_num, sizeof(struct xtensa_reg_desc *));
		if (!xtensa->contiguous_regs_desc) {
			LOG_ERROR("Failed to allocate xtensa->contiguous_regs_desc!");
			return ERROR_FAIL;
		}
	}

	const char *regname = CMD_ARGV[0];
	unsigned int regnum = strtoul(CMD_ARGV[1], NULL, 0);
	if (regnum > UINT16_MAX) {
		LOG_ERROR("<regnum> must be a 16-bit number");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	if ((xtensa->num_optregs + xtensa->core_regs_num) >= xtensa->total_regs_num) {
		if (xtensa->total_regs_num)
			LOG_ERROR("'xtreg %s 0x%04x': Too many registers (%d expected, %d core %d extended)",
				regname, regnum,
				xtensa->total_regs_num, xtensa->core_regs_num, xtensa->num_optregs);
		else
			LOG_ERROR("'xtreg %s 0x%04x': Number of registers unspecified",
				regname, regnum);
		return ERROR_FAIL;
	}

	/* Determine whether register belongs in xtensa_regs[] or xtensa->xtensa_spec_regs[] */
	struct xtensa_reg_desc *rptr = &xtensa->optregs[xtensa->num_optregs];
	bool is_extended_reg = true;
	unsigned int ridx;
	for (ridx = 0; ridx < XT_NUM_REGS; ridx++) {
		if (strcmp(CMD_ARGV[0], xtensa_regs[ridx].name) == 0) {
			/* Flag core register as defined */
			rptr = &xtensa_regs[ridx];
			xtensa->core_regs_num++;
			is_extended_reg = false;
			break;
		}
	}

	rptr->exist = true;
	if (is_extended_reg) {
		/* Register ID, debugger-visible register ID */
		rptr->name = strdup(CMD_ARGV[0]);
		rptr->dbreg_num = regnum;
		rptr->reg_num = (regnum & XT_REG_INDEX_MASK);
		xtensa->num_optregs++;

		/* Register type */
		if ((regnum & XT_REG_GENERAL_MASK) == XT_REG_GENERAL_VAL) {
			rptr->type = XT_REG_GENERAL;
		} else if ((regnum & XT_REG_USER_MASK) == XT_REG_USER_VAL) {
			rptr->type = XT_REG_USER;
		} else if ((regnum & XT_REG_FR_MASK) == XT_REG_FR_VAL) {
			rptr->type = XT_REG_FR;
		} else if ((regnum & XT_REG_SPECIAL_MASK) == XT_REG_SPECIAL_VAL) {
			rptr->type = XT_REG_SPECIAL;
		} else if ((regnum & XT_REG_RELGEN_MASK) == XT_REG_RELGEN_VAL) {
			/* WARNING: For these registers, regnum points to the
			 * index of the corresponding ARx registers, NOT to
			 * the processor register number! */
			rptr->type = XT_REG_RELGEN;
			rptr->reg_num += XT_REG_IDX_ARFIRST;
			rptr->dbreg_num += XT_REG_IDX_ARFIRST;
		} else if ((regnum & XT_REG_TIE_MASK) != 0) {
			rptr->type = XT_REG_TIE;
		} else {
			rptr->type = XT_REG_OTHER;
		}

		/* Register flags */
		if ((strcmp(rptr->name, "mmid") == 0) || (strcmp(rptr->name, "eraccess") == 0) ||
			(strcmp(rptr->name, "ddr") == 0) || (strcmp(rptr->name, "intset") == 0) ||
			(strcmp(rptr->name, "intclear") == 0))
			rptr->flags = XT_REGF_NOREAD;
		else
			rptr->flags = 0;

		if ((rptr->reg_num == (XT_PS_REG_NUM_BASE + xtensa->core_config->debug.irq_level)) &&
			(xtensa->core_config->core_type == XT_LX) && (rptr->type == XT_REG_SPECIAL)) {
			xtensa->eps_dbglevel_idx = XT_NUM_REGS + xtensa->num_optregs - 1;
			LOG_DEBUG("Setting PS (%s) index to %d", rptr->name, xtensa->eps_dbglevel_idx);
		}
	} else if (strcmp(rptr->name, "cpenable") == 0) {
		xtensa->core_config->coproc = true;
	}

	/* Build out list of contiguous registers in specified order */
	unsigned int running_reg_count = xtensa->num_optregs + xtensa->core_regs_num;
	if (xtensa->contiguous_regs_desc) {
		assert((running_reg_count <= xtensa->total_regs_num) && "contiguous register address internal error!");
		xtensa->contiguous_regs_desc[running_reg_count - 1] = rptr;
	}
	if (xtensa_extra_debug_log)
		LOG_DEBUG("Added %s register %-16s: 0x%04x/0x%02x t%d (%d of %d)",
			is_extended_reg ? "config-specific" : "core",
			rptr->name, rptr->dbreg_num, rptr->reg_num, rptr->type,
			is_extended_reg ? xtensa->num_optregs : ridx,
			is_extended_reg ? xtensa->total_regs_num : XT_NUM_REGS);
	return ERROR_OK;
}

COMMAND_HANDLER(xtensa_cmd_xtreg)
{
	return CALL_COMMAND_HANDLER(xtensa_cmd_xtreg_do,
		target_to_xtensa(get_current_target(CMD_CTX)));
}

/* xtregfmt <contiguous|sparse> [numgregs] */
COMMAND_HELPER(xtensa_cmd_xtregfmt_do, struct xtensa *xtensa)
{
	if ((CMD_ARGC == 1) || (CMD_ARGC == 2)) {
		if (!strcasecmp(CMD_ARGV[0], "sparse")) {
			return ERROR_OK;
		} else if (!strcasecmp(CMD_ARGV[0], "contiguous")) {
			xtensa->regmap_contiguous = true;
			if (CMD_ARGC == 2) {
				unsigned int numgregs = strtoul(CMD_ARGV[1], NULL, 0);
				if ((numgregs <= 0) ||
					((numgregs > xtensa->total_regs_num) &&
					(xtensa->total_regs_num > 0))) {
					LOG_ERROR("xtregfmt: if specified, numgregs (%d) must be <= numregs (%d)",
						numgregs, xtensa->total_regs_num);
					return ERROR_COMMAND_SYNTAX_ERROR;
				}
				xtensa->genpkt_regs_num = numgregs;
			}
			return ERROR_OK;
		}
	}
	return ERROR_COMMAND_SYNTAX_ERROR;
}

COMMAND_HANDLER(xtensa_cmd_xtregfmt)
{
	return CALL_COMMAND_HANDLER(xtensa_cmd_xtregfmt_do,
		target_to_xtensa(get_current_target(CMD_CTX)));
}

COMMAND_HELPER(xtensa_cmd_permissive_mode_do, struct xtensa *xtensa)
{
	return CALL_COMMAND_HANDLER(handle_command_parse_bool,
		&xtensa->permissive_mode, "xtensa permissive mode");
}

COMMAND_HANDLER(xtensa_cmd_permissive_mode)
{
	return CALL_COMMAND_HANDLER(xtensa_cmd_permissive_mode_do,
		target_to_xtensa(get_current_target(CMD_CTX)));
}

/* perfmon_enable <counter_id> <select> [mask] [kernelcnt] [tracelevel] */
COMMAND_HELPER(xtensa_cmd_perfmon_enable_do, struct xtensa *xtensa)
{
	struct xtensa_perfmon_config config = {
		.mask = 0xffff,
		.kernelcnt = 0,
		.tracelevel = -1	/* use DEBUGLEVEL by default */
	};

	if (CMD_ARGC < 2 || CMD_ARGC > 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	unsigned int counter_id = strtoul(CMD_ARGV[0], NULL, 0);
	if (counter_id >= XTENSA_MAX_PERF_COUNTERS) {
		command_print(CMD, "counter_id should be < %d", XTENSA_MAX_PERF_COUNTERS);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	config.select = strtoul(CMD_ARGV[1], NULL, 0);
	if (config.select > XTENSA_MAX_PERF_SELECT) {
		command_print(CMD, "select should be < %d", XTENSA_MAX_PERF_SELECT);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	if (CMD_ARGC >= 3) {
		config.mask = strtoul(CMD_ARGV[2], NULL, 0);
		if (config.mask > XTENSA_MAX_PERF_MASK) {
			command_print(CMD, "mask should be < %d", XTENSA_MAX_PERF_MASK);
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
	}

	if (CMD_ARGC >= 4) {
		config.kernelcnt = strtoul(CMD_ARGV[3], NULL, 0);
		if (config.kernelcnt > 1) {
			command_print(CMD, "kernelcnt should be 0 or 1");
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
	}

	if (CMD_ARGC >= 5) {
		config.tracelevel = strtoul(CMD_ARGV[4], NULL, 0);
		if (config.tracelevel > 7) {
			command_print(CMD, "tracelevel should be <=7");
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
	}

	if (config.tracelevel == -1)
		config.tracelevel = xtensa->core_config->debug.irq_level;

	return xtensa_dm_perfmon_enable(&xtensa->dbg_mod, counter_id, &config);
}

COMMAND_HANDLER(xtensa_cmd_perfmon_enable)
{
	return CALL_COMMAND_HANDLER(xtensa_cmd_perfmon_enable_do,
		target_to_xtensa(get_current_target(CMD_CTX)));
}

/* perfmon_dump [counter_id] */
COMMAND_HELPER(xtensa_cmd_perfmon_dump_do, struct xtensa *xtensa)
{
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	int counter_id = -1;
	if (CMD_ARGC == 1) {
		counter_id = strtol(CMD_ARGV[0], NULL, 0);
		if (counter_id > XTENSA_MAX_PERF_COUNTERS) {
			command_print(CMD, "counter_id should be < %d", XTENSA_MAX_PERF_COUNTERS);
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
	}

	unsigned int counter_start = (counter_id < 0) ? 0 : counter_id;
	unsigned int counter_end = (counter_id < 0) ? XTENSA_MAX_PERF_COUNTERS : counter_id + 1;
	for (unsigned int counter = counter_start; counter < counter_end; ++counter) {
		char result_buf[128] = { 0 };
		size_t result_pos = snprintf(result_buf, sizeof(result_buf), "Counter %d: ", counter);
		struct xtensa_perfmon_result result;
		int res = xtensa_dm_perfmon_dump(&xtensa->dbg_mod, counter, &result);
		if (res != ERROR_OK)
			return res;
		snprintf(result_buf + result_pos, sizeof(result_buf) - result_pos,
			"%-12" PRIu64 "%s",
			result.value,
			result.overflow ? " (overflow)" : "");
		LOG_INFO("%s", result_buf);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(xtensa_cmd_perfmon_dump)
{
	return CALL_COMMAND_HANDLER(xtensa_cmd_perfmon_dump_do,
		target_to_xtensa(get_current_target(CMD_CTX)));
}

COMMAND_HELPER(xtensa_cmd_mask_interrupts_do, struct xtensa *xtensa)
{
	int state = -1;

	if (CMD_ARGC < 1) {
		const char *st;
		state = xtensa->stepping_isr_mode;
		if (state == XT_STEPPING_ISR_ON)
			st = "OFF";
		else if (state == XT_STEPPING_ISR_OFF)
			st = "ON";
		else
			st = "UNKNOWN";
		command_print(CMD, "Current ISR step mode: %s", st);
		return ERROR_OK;
	}
	/* Masking is ON -> interrupts during stepping are OFF, and vice versa */
	if (!strcasecmp(CMD_ARGV[0], "off"))
		state = XT_STEPPING_ISR_ON;
	else if (!strcasecmp(CMD_ARGV[0], "on"))
		state = XT_STEPPING_ISR_OFF;

	if (state == -1) {
		command_print(CMD, "Argument unknown. Please pick one of ON, OFF");
		return ERROR_FAIL;
	}
	xtensa->stepping_isr_mode = state;
	return ERROR_OK;
}

COMMAND_HANDLER(xtensa_cmd_mask_interrupts)
{
	return CALL_COMMAND_HANDLER(xtensa_cmd_mask_interrupts_do,
		target_to_xtensa(get_current_target(CMD_CTX)));
}

COMMAND_HELPER(xtensa_cmd_smpbreak_do, struct target *target)
{
	int res;
	uint32_t val = 0;

	if (CMD_ARGC >= 1) {
		for (unsigned int i = 0; i < CMD_ARGC; i++) {
			if (!strcasecmp(CMD_ARGV[0], "none")) {
				val = 0;
			} else if (!strcasecmp(CMD_ARGV[i], "BreakIn")) {
				val |= OCDDCR_BREAKINEN;
			} else if (!strcasecmp(CMD_ARGV[i], "BreakOut")) {
				val |= OCDDCR_BREAKOUTEN;
			} else if (!strcasecmp(CMD_ARGV[i], "RunStallIn")) {
				val |= OCDDCR_RUNSTALLINEN;
			} else if (!strcasecmp(CMD_ARGV[i], "DebugModeOut")) {
				val |= OCDDCR_DEBUGMODEOUTEN;
			} else if (!strcasecmp(CMD_ARGV[i], "BreakInOut")) {
				val |= OCDDCR_BREAKINEN | OCDDCR_BREAKOUTEN;
			} else if (!strcasecmp(CMD_ARGV[i], "RunStall")) {
				val |= OCDDCR_RUNSTALLINEN | OCDDCR_DEBUGMODEOUTEN;
			} else {
				command_print(CMD, "Unknown arg %s", CMD_ARGV[i]);
				command_print(
					CMD,
					"use either BreakInOut, None or RunStall as arguments, or any combination of BreakIn, BreakOut, RunStallIn and DebugModeOut.");
				return ERROR_OK;
			}
		}
		res = xtensa_smpbreak_set(target, val);
		if (res != ERROR_OK)
			command_print(CMD, "Failed to set smpbreak config %d", res);
	} else {
		struct xtensa *xtensa = target_to_xtensa(target);
		res = xtensa_smpbreak_read(xtensa, &val);
		if (res == ERROR_OK)
			command_print(CMD, "Current bits set:%s%s%s%s",
				(val & OCDDCR_BREAKINEN) ? " BreakIn" : "",
				(val & OCDDCR_BREAKOUTEN) ? " BreakOut" : "",
				(val & OCDDCR_RUNSTALLINEN) ? " RunStallIn" : "",
				(val & OCDDCR_DEBUGMODEOUTEN) ? " DebugModeOut" : ""
				);
		else
			command_print(CMD, "Failed to get smpbreak config %d", res);
	}
	return res;
}

COMMAND_HANDLER(xtensa_cmd_smpbreak)
{
	return CALL_COMMAND_HANDLER(xtensa_cmd_smpbreak_do,
		get_current_target(CMD_CTX));
}

COMMAND_HELPER(xtensa_cmd_tracestart_do, struct xtensa *xtensa)
{
	struct xtensa_trace_status trace_status;
	struct xtensa_trace_start_config cfg = {
		.stoppc = 0,
		.stopmask = XTENSA_STOPMASK_DISABLED,
		.after = 0,
		.after_is_words = false
	};

	/* Parse arguments */
	for (unsigned int i = 0; i < CMD_ARGC; i++) {
		if ((!strcasecmp(CMD_ARGV[i], "pc")) && CMD_ARGC > i) {
			char *e;
			i++;
			cfg.stoppc = strtol(CMD_ARGV[i], &e, 0);
			cfg.stopmask = 0;
			if (*e == '/')
				cfg.stopmask = strtol(e, NULL, 0);
		} else if ((!strcasecmp(CMD_ARGV[i], "after")) && CMD_ARGC > i) {
			i++;
			cfg.after = strtol(CMD_ARGV[i], NULL, 0);
		} else if (!strcasecmp(CMD_ARGV[i], "ins")) {
			cfg.after_is_words = 0;
		} else if (!strcasecmp(CMD_ARGV[i], "words")) {
			cfg.after_is_words = 1;
		} else {
			command_print(CMD, "Did not understand %s", CMD_ARGV[i]);
			return ERROR_FAIL;
		}
	}

	int res = xtensa_dm_trace_status_read(&xtensa->dbg_mod, &trace_status);
	if (res != ERROR_OK)
		return res;
	if (trace_status.stat & TRAXSTAT_TRACT) {
		LOG_WARNING("Silently stop active tracing!");
		res = xtensa_dm_trace_stop(&xtensa->dbg_mod, false);
		if (res != ERROR_OK)
			return res;
	}

	res = xtensa_dm_trace_start(&xtensa->dbg_mod, &cfg);
	if (res != ERROR_OK)
		return res;

	xtensa->trace_active = true;
	command_print(CMD, "Trace started.");
	return ERROR_OK;
}

COMMAND_HANDLER(xtensa_cmd_tracestart)
{
	return CALL_COMMAND_HANDLER(xtensa_cmd_tracestart_do,
		target_to_xtensa(get_current_target(CMD_CTX)));
}

COMMAND_HELPER(xtensa_cmd_tracestop_do, struct xtensa *xtensa)
{
	struct xtensa_trace_status trace_status;

	int res = xtensa_dm_trace_status_read(&xtensa->dbg_mod, &trace_status);
	if (res != ERROR_OK)
		return res;

	if (!(trace_status.stat & TRAXSTAT_TRACT)) {
		command_print(CMD, "No trace is currently active.");
		return ERROR_FAIL;
	}

	res = xtensa_dm_trace_stop(&xtensa->dbg_mod, true);
	if (res != ERROR_OK)
		return res;

	xtensa->trace_active = false;
	command_print(CMD, "Trace stop triggered.");
	return ERROR_OK;
}

COMMAND_HANDLER(xtensa_cmd_tracestop)
{
	return CALL_COMMAND_HANDLER(xtensa_cmd_tracestop_do,
		target_to_xtensa(get_current_target(CMD_CTX)));
}

COMMAND_HELPER(xtensa_cmd_tracedump_do, struct xtensa *xtensa, const char *fname)
{
	struct xtensa_trace_config trace_config;
	struct xtensa_trace_status trace_status;
	uint32_t memsz, wmem;

	int res = xtensa_dm_trace_status_read(&xtensa->dbg_mod, &trace_status);
	if (res != ERROR_OK)
		return res;

	if (trace_status.stat & TRAXSTAT_TRACT) {
		command_print(CMD, "Tracing is still active. Please stop it first.");
		return ERROR_FAIL;
	}

	res = xtensa_dm_trace_config_read(&xtensa->dbg_mod, &trace_config);
	if (res != ERROR_OK)
		return res;

	if (!(trace_config.ctrl & TRAXCTRL_TREN)) {
		command_print(CMD, "No active trace found; nothing to dump.");
		return ERROR_FAIL;
	}

	memsz = trace_config.memaddr_end - trace_config.memaddr_start + 1;
	LOG_INFO("Total trace memory: %d words", memsz);
	if ((trace_config.addr &
			((TRAXADDR_TWRAP_MASK << TRAXADDR_TWRAP_SHIFT) | TRAXADDR_TWSAT)) == 0) {
		/*Memory hasn't overwritten itself yet. */
		wmem = trace_config.addr & TRAXADDR_TADDR_MASK;
		LOG_INFO("...but trace is only %d words", wmem);
		if (wmem < memsz)
			memsz = wmem;
	} else {
		if (trace_config.addr & TRAXADDR_TWSAT) {
			LOG_INFO("Real trace is many times longer than that (overflow)");
		} else {
			uint32_t trc_sz = (trace_config.addr >> TRAXADDR_TWRAP_SHIFT) & TRAXADDR_TWRAP_MASK;
			trc_sz = (trc_sz * memsz) + (trace_config.addr & TRAXADDR_TADDR_MASK);
			LOG_INFO("Real trace is %d words, but the start has been truncated.", trc_sz);
		}
	}

	uint8_t *tracemem = malloc(memsz * 4);
	if (!tracemem) {
		command_print(CMD, "Failed to alloc memory for trace data!");
		return ERROR_FAIL;
	}
	res = xtensa_dm_trace_data_read(&xtensa->dbg_mod, tracemem, memsz * 4);
	if (res != ERROR_OK) {
		free(tracemem);
		return res;
	}

	int f = open(fname, O_WRONLY | O_CREAT | O_TRUNC, 0666);
	if (f <= 0) {
		free(tracemem);
		command_print(CMD, "Unable to open file %s", fname);
		return ERROR_FAIL;
	}
	if (write(f, tracemem, memsz * 4) != (int)memsz * 4)
		command_print(CMD, "Unable to write to file %s", fname);
	else
		command_print(CMD, "Written %d bytes of trace data to %s", memsz * 4, fname);
	close(f);

	bool is_all_zeroes = true;
	for (unsigned int i = 0; i < memsz * 4; i++) {
		if (tracemem[i] != 0) {
			is_all_zeroes = false;
			break;
		}
	}
	free(tracemem);
	if (is_all_zeroes)
		command_print(
			CMD,
			"WARNING: File written is all zeroes. Are you sure you enabled trace memory?");

	return ERROR_OK;
}

COMMAND_HANDLER(xtensa_cmd_tracedump)
{
	if (CMD_ARGC != 1) {
		command_print(CMD, "Command takes exactly 1 parameter.Need filename to dump to as output!");
		return ERROR_FAIL;
	}

	return CALL_COMMAND_HANDLER(xtensa_cmd_tracedump_do,
		target_to_xtensa(get_current_target(CMD_CTX)), CMD_ARGV[0]);
}

static const struct command_registration xtensa_any_command_handlers[] = {
	{
		.name = "xtdef",
		.handler = xtensa_cmd_xtdef,
		.mode = COMMAND_CONFIG,
		.help = "Configure Xtensa core type",
		.usage = "<type>",
	},
	{
		.name = "xtopt",
		.handler = xtensa_cmd_xtopt,
		.mode = COMMAND_CONFIG,
		.help = "Configure Xtensa core option",
		.usage = "<name> <value>",
	},
	{
		.name = "xtmem",
		.handler = xtensa_cmd_xtmem,
		.mode = COMMAND_CONFIG,
		.help = "Configure Xtensa memory/cache option",
		.usage = "<type> [parameters]",
	},
	{
		.name = "xtmmu",
		.handler = xtensa_cmd_xtmmu,
		.mode = COMMAND_CONFIG,
		.help = "Configure Xtensa MMU option",
		.usage = "<NIREFILLENTRIES> <NDREFILLENTRIES> <IVARWAY56> <DVARWAY56>",
	},
	{
		.name = "xtmpu",
		.handler = xtensa_cmd_xtmpu,
		.mode = COMMAND_CONFIG,
		.help = "Configure Xtensa MPU option",
		.usage = "<num FG seg> <min seg size> <lockable> <executeonly>",
	},
	{
		.name = "xtreg",
		.handler = xtensa_cmd_xtreg,
		.mode = COMMAND_CONFIG,
		.help = "Configure Xtensa register",
		.usage = "<regname> <regnum>",
	},
	{
		.name = "xtregs",
		.handler = xtensa_cmd_xtreg,
		.mode = COMMAND_CONFIG,
		.help = "Configure number of Xtensa registers",
		.usage = "<numregs>",
	},
	{
		.name = "xtregfmt",
		.handler = xtensa_cmd_xtregfmt,
		.mode = COMMAND_CONFIG,
		.help = "Configure format of Xtensa register map",
		.usage = "<contiguous|sparse> [numgregs]",
	},
	{
		.name = "set_permissive",
		.handler = xtensa_cmd_permissive_mode,
		.mode = COMMAND_ANY,
		.help = "When set to 1, enable Xtensa permissive mode (fewer client-side checks)",
		.usage = "[0|1]",
	},
	{
		.name = "maskisr",
		.handler = xtensa_cmd_mask_interrupts,
		.mode = COMMAND_ANY,
		.help = "mask Xtensa interrupts at step",
		.usage = "['on'|'off']",
	},
	{
		.name = "smpbreak",
		.handler = xtensa_cmd_smpbreak,
		.mode = COMMAND_ANY,
		.help = "Set the way the CPU chains OCD breaks",
		.usage = "[none|breakinout|runstall] | [BreakIn] [BreakOut] [RunStallIn] [DebugModeOut]",
	},
	{
		.name = "perfmon_enable",
		.handler = xtensa_cmd_perfmon_enable,
		.mode = COMMAND_EXEC,
		.help = "Enable and start performance counter",
		.usage = "<counter_id> <select> [mask] [kernelcnt] [tracelevel]",
	},
	{
		.name = "perfmon_dump",
		.handler = xtensa_cmd_perfmon_dump,
		.mode = COMMAND_EXEC,
		.help = "Dump performance counter value. If no argument specified, dumps all counters.",
		.usage = "[counter_id]",
	},
	{
		.name = "tracestart",
		.handler = xtensa_cmd_tracestart,
		.mode = COMMAND_EXEC,
		.help =
			"Tracing: Set up and start a trace. Optionally set stop trigger address and amount of data captured after.",
		.usage = "[pc <pcval>/[maskbitcount]] [after <n> [ins|words]]",
	},
	{
		.name = "tracestop",
		.handler = xtensa_cmd_tracestop,
		.mode = COMMAND_EXEC,
		.help = "Tracing: Stop current trace as started by the tracestart command",
		.usage = "",
	},
	{
		.name = "tracedump",
		.handler = xtensa_cmd_tracedump,
		.mode = COMMAND_EXEC,
		.help = "Tracing: Dump trace memory to a files. One file per core.",
		.usage = "<outfile>",
	},
	{
		.name = "exe",
		.handler = xtensa_cmd_exe,
		.mode = COMMAND_ANY,
		.help = "Xtensa stub execution",
		.usage = "<ascii-encoded hexadecimal instruction bytes>",
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration xtensa_command_handlers[] = {
	{
		.name = "xtensa",
		.mode = COMMAND_ANY,
		.help = "Xtensa command group",
		.usage = "",
		.chain = xtensa_any_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};
