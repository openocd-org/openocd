/***************************************************************************
 *   Generic Xtensa target API for OpenOCD                                 *
 *   Copyright (C) 2016-2019 Espressif Systems Ltd.                        *
 *   Derived from esp108.c                                                 *
 *   Author: Angus Gratton gus@projectgus.com                              *
 *   Author: Jeroen Domburg <jeroen@espressif.com>                         *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
 *   Author: Andrey Gramakov <andrei.gramakov@espressif.com>               *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <helper/time_support.h>
#include <helper/align.h>
#include <target/register.h>

#include "xtensa.h"


#define _XT_INS_FORMAT_RSR(OPCODE, SR, T) ((OPCODE)	    \
		| (((SR) & 0xFF) << 8) \
		| (((T) & 0x0F) << 4))

#define _XT_INS_FORMAT_RRR(OPCODE, ST, R) ((OPCODE)	    \
		| (((ST) & 0xFF) << 4) \
		| (((R) & 0x0F) << 12))

#define _XT_INS_FORMAT_RRRN(OPCODE, S, T, IMM4) ((OPCODE)	      \
		| (((T) & 0x0F) << 4)	\
		| (((S) & 0x0F) << 8)	\
		| (((IMM4) & 0x0F) << 12))

#define _XT_INS_FORMAT_RRI8(OPCODE, R, S, T, IMM8) ((OPCODE)	    \
		| (((IMM8) & 0xFF) << 16) \
		| (((R) & 0x0F) << 12)	\
		| (((S) & 0x0F) << 8)	\
		| (((T) & 0x0F) << 4))

#define _XT_INS_FORMAT_RRI4(OPCODE, IMM4, R, S, T) ((OPCODE) \
		| (((IMM4) & 0x0F) << 20) \
		| (((R) & 0x0F) << 12) \
		| (((S) & 0x0F) << 8)	\
		| (((T) & 0x0F) << 4))

/* Xtensa processor instruction opcodes
 * "Return From Debug Operation" to Normal */
#define XT_INS_RFDO      0xf1e000
/* "Return From Debug and Dispatch" - allow sw debugging stuff to take over */
#define XT_INS_RFDD      0xf1e010

/* Load to DDR register, increase addr register */
#define XT_INS_LDDR32P(S) (0x0070E0 | ((S) << 8))
/* Store from DDR register, increase addr register */
#define XT_INS_SDDR32P(S) (0x0070F0 | ((S) << 8))

/* Load 32-bit Indirect from A(S) + 4 * IMM8 to A(T) */
#define XT_INS_L32I(S, T, IMM8)  _XT_INS_FORMAT_RRI8(0x002002, 0, S, T, IMM8)
/* Load 16-bit Unsigned from A(S) + 2 * IMM8 to A(T) */
#define XT_INS_L16UI(S, T, IMM8) _XT_INS_FORMAT_RRI8(0x001002, 0, S, T, IMM8)
/* Load 8-bit Unsigned from A(S) + IMM8 to A(T) */
#define XT_INS_L8UI(S, T, IMM8)  _XT_INS_FORMAT_RRI8(0x000002, 0, S, T, IMM8)

/* Store 32-bit Indirect to A(S) + 4 * IMM8 from A(T) */
#define XT_INS_S32I(S, T, IMM8) _XT_INS_FORMAT_RRI8(0x006002, 0, S, T, IMM8)
/* Store 16-bit to A(S) + 2 * IMM8 from A(T) */
#define XT_INS_S16I(S, T, IMM8) _XT_INS_FORMAT_RRI8(0x005002, 0, S, T, IMM8)
/* Store 8-bit to A(S) + IMM8 from A(T) */
#define XT_INS_S8I(S, T, IMM8)  _XT_INS_FORMAT_RRI8(0x004002, 0, S, T, IMM8)

/* Read Special Register */
#define XT_INS_RSR(SR, T) _XT_INS_FORMAT_RSR(0x030000, SR, T)
/* Write Special Register */
#define XT_INS_WSR(SR, T) _XT_INS_FORMAT_RSR(0x130000, SR, T)
/* Swap Special Register */
#define XT_INS_XSR(SR, T) _XT_INS_FORMAT_RSR(0x610000, SR, T)

/* Rotate Window by (-8..7) */
#define XT_INS_ROTW(N) ((0x408000) | (((N) & 15) << 4))

/* Read User Register */
#define XT_INS_RUR(UR, T) _XT_INS_FORMAT_RRR(0xE30000, UR, T)
/* Write User Register */
#define XT_INS_WUR(UR, T) _XT_INS_FORMAT_RSR(0xF30000, UR, T)

/* Read Floating-Point Register */
#define XT_INS_RFR(FR, T) _XT_INS_FORMAT_RRR(0xFA0000, (((FR) << 4) | 0x4), T)
/* Write Floating-Point Register */
#define XT_INS_WFR(FR, T) _XT_INS_FORMAT_RRR(0xFA0000, (((FR) << 4) | 0x5), T)

/* 32-bit break */
#define XT_INS_BREAK(IMM1, IMM2)  _XT_INS_FORMAT_RRR(0x000000, \
		(((IMM1) & 0x0F) << 4) | ((IMM2) & 0x0F), 0x4)
/* 16-bit break */
#define XT_INS_BREAKN(IMM4)  _XT_INS_FORMAT_RRRN(0x00000D, IMM4, 0x2, 0xF)

#define XT_INS_L32E(R, S, T) _XT_INS_FORMAT_RRI4(0x90000, 0, R, S, T)
#define XT_INS_S32E(R, S, T) _XT_INS_FORMAT_RRI4(0x490000, 0, R, S, T)
#define XT_INS_L32E_S32E_MASK   0xFF000F

#define XT_INS_RFWO 0x3400
#define XT_INS_RFWU 0x3500
#define XT_INS_RFWO_RFWU_MASK   0xFFFFFF

#define XT_WATCHPOINTS_NUM_MAX  2

/* Special register number macro for DDR register.
* this gets used a lot so making a shortcut to it is
* useful.
*/
#define XT_SR_DDR         (xtensa_regs[XT_REG_IDX_OCD_DDR].reg_num)

/*Same thing for A3/A4 */
#define XT_REG_A3         (xtensa_regs[XT_REG_IDX_AR3].reg_num)
#define XT_REG_A4         (xtensa_regs[XT_REG_IDX_AR4].reg_num)

#define XT_PC_REG_NUM_BASE          (176)
#define XT_SW_BREAKPOINTS_MAX_NUM   32

const struct xtensa_reg_desc xtensa_regs[XT_NUM_REGS] = {
	{ "pc", XT_PC_REG_NUM_BASE /*+XT_DEBUGLEVEL*/, XT_REG_SPECIAL, 0 },		/* actually epc[debuglevel] */
	{ "ar0", 0x00, XT_REG_GENERAL, 0 },
	{ "ar1", 0x01, XT_REG_GENERAL, 0 },
	{ "ar2", 0x02, XT_REG_GENERAL, 0 },
	{ "ar3", 0x03, XT_REG_GENERAL, 0 },
	{ "ar4", 0x04, XT_REG_GENERAL, 0 },
	{ "ar5", 0x05, XT_REG_GENERAL, 0 },
	{ "ar6", 0x06, XT_REG_GENERAL, 0 },
	{ "ar7", 0x07, XT_REG_GENERAL, 0 },
	{ "ar8", 0x08, XT_REG_GENERAL, 0 },
	{ "ar9", 0x09, XT_REG_GENERAL, 0 },
	{ "ar10", 0x0A, XT_REG_GENERAL, 0 },
	{ "ar11", 0x0B, XT_REG_GENERAL, 0 },
	{ "ar12", 0x0C, XT_REG_GENERAL, 0 },
	{ "ar13", 0x0D, XT_REG_GENERAL, 0 },
	{ "ar14", 0x0E, XT_REG_GENERAL, 0 },
	{ "ar15", 0x0F, XT_REG_GENERAL, 0 },
	{ "ar16", 0x10, XT_REG_GENERAL, 0 },
	{ "ar17", 0x11, XT_REG_GENERAL, 0 },
	{ "ar18", 0x12, XT_REG_GENERAL, 0 },
	{ "ar19", 0x13, XT_REG_GENERAL, 0 },
	{ "ar20", 0x14, XT_REG_GENERAL, 0 },
	{ "ar21", 0x15, XT_REG_GENERAL, 0 },
	{ "ar22", 0x16, XT_REG_GENERAL, 0 },
	{ "ar23", 0x17, XT_REG_GENERAL, 0 },
	{ "ar24", 0x18, XT_REG_GENERAL, 0 },
	{ "ar25", 0x19, XT_REG_GENERAL, 0 },
	{ "ar26", 0x1A, XT_REG_GENERAL, 0 },
	{ "ar27", 0x1B, XT_REG_GENERAL, 0 },
	{ "ar28", 0x1C, XT_REG_GENERAL, 0 },
	{ "ar29", 0x1D, XT_REG_GENERAL, 0 },
	{ "ar30", 0x1E, XT_REG_GENERAL, 0 },
	{ "ar31", 0x1F, XT_REG_GENERAL, 0 },
	{ "ar32", 0x20, XT_REG_GENERAL, 0 },
	{ "ar33", 0x21, XT_REG_GENERAL, 0 },
	{ "ar34", 0x22, XT_REG_GENERAL, 0 },
	{ "ar35", 0x23, XT_REG_GENERAL, 0 },
	{ "ar36", 0x24, XT_REG_GENERAL, 0 },
	{ "ar37", 0x25, XT_REG_GENERAL, 0 },
	{ "ar38", 0x26, XT_REG_GENERAL, 0 },
	{ "ar39", 0x27, XT_REG_GENERAL, 0 },
	{ "ar40", 0x28, XT_REG_GENERAL, 0 },
	{ "ar41", 0x29, XT_REG_GENERAL, 0 },
	{ "ar42", 0x2A, XT_REG_GENERAL, 0 },
	{ "ar43", 0x2B, XT_REG_GENERAL, 0 },
	{ "ar44", 0x2C, XT_REG_GENERAL, 0 },
	{ "ar45", 0x2D, XT_REG_GENERAL, 0 },
	{ "ar46", 0x2E, XT_REG_GENERAL, 0 },
	{ "ar47", 0x2F, XT_REG_GENERAL, 0 },
	{ "ar48", 0x30, XT_REG_GENERAL, 0 },
	{ "ar49", 0x31, XT_REG_GENERAL, 0 },
	{ "ar50", 0x32, XT_REG_GENERAL, 0 },
	{ "ar51", 0x33, XT_REG_GENERAL, 0 },
	{ "ar52", 0x34, XT_REG_GENERAL, 0 },
	{ "ar53", 0x35, XT_REG_GENERAL, 0 },
	{ "ar54", 0x36, XT_REG_GENERAL, 0 },
	{ "ar55", 0x37, XT_REG_GENERAL, 0 },
	{ "ar56", 0x38, XT_REG_GENERAL, 0 },
	{ "ar57", 0x39, XT_REG_GENERAL, 0 },
	{ "ar58", 0x3A, XT_REG_GENERAL, 0 },
	{ "ar59", 0x3B, XT_REG_GENERAL, 0 },
	{ "ar60", 0x3C, XT_REG_GENERAL, 0 },
	{ "ar61", 0x3D, XT_REG_GENERAL, 0 },
	{ "ar62", 0x3E, XT_REG_GENERAL, 0 },
	{ "ar63", 0x3F, XT_REG_GENERAL, 0 },
	{ "lbeg", 0x00, XT_REG_SPECIAL, 0 },
	{ "lend", 0x01, XT_REG_SPECIAL, 0 },
	{ "lcount", 0x02, XT_REG_SPECIAL, 0 },
	{ "sar", 0x03, XT_REG_SPECIAL, 0 },
	{ "windowbase", 0x48, XT_REG_SPECIAL, 0 },
	{ "windowstart", 0x49, XT_REG_SPECIAL, 0 },
	{ "configid0", 0xB0, XT_REG_SPECIAL, 0 },
	{ "configid1", 0xD0, XT_REG_SPECIAL, 0 },
	{ "ps", 0xC6, XT_REG_SPECIAL, 0 },			/* actually EPS[debuglevel] */
	{ "threadptr", 0xE7, XT_REG_USER, 0 },
	{ "br", 0x04, XT_REG_SPECIAL, 0 },
	{ "scompare1", 0x0C, XT_REG_SPECIAL, 0 },
	{ "acclo", 0x10, XT_REG_SPECIAL, 0 },
	{ "acchi", 0x11, XT_REG_SPECIAL, 0 },
	{ "m0", 0x20, XT_REG_SPECIAL, 0 },
	{ "m1", 0x21, XT_REG_SPECIAL, 0 },
	{ "m2", 0x22, XT_REG_SPECIAL, 0 },
	{ "m3", 0x23, XT_REG_SPECIAL, 0 },
	{ "f0", 0x00, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f1", 0x01, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f2", 0x02, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f3", 0x03, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f4", 0x04, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f5", 0x05, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f6", 0x06, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f7", 0x07, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f8", 0x08, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f9", 0x09, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f10", 0x0A, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f11", 0x0B, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f12", 0x0C, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f13", 0x0D, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f14", 0x0E, XT_REG_FR, XT_REGF_COPROC0 },
	{ "f15", 0x0F, XT_REG_FR, XT_REGF_COPROC0 },
	{ "fcr", 0xE8, XT_REG_USER, XT_REGF_COPROC0 },
	{ "fsr", 0xE9, XT_REG_USER, XT_REGF_COPROC0 },
	{ "mmid", 0x59, XT_REG_SPECIAL, XT_REGF_NOREAD },
	{ "ibreakenable", 0x60, XT_REG_SPECIAL, 0 },
	{ "memctl", 0x61, XT_REG_SPECIAL, 0 },
	{ "atomctl", 0x63, XT_REG_SPECIAL, 0 },
	{ "ibreaka0", 0x80, XT_REG_SPECIAL, 0 },
	{ "ibreaka1", 0x81, XT_REG_SPECIAL, 0 },
	{ "dbreaka0", 0x90, XT_REG_SPECIAL, 0 },
	{ "dbreaka1", 0x91, XT_REG_SPECIAL, 0 },
	{ "dbreakc0", 0xA0, XT_REG_SPECIAL, 0 },
	{ "dbreakc1", 0xA1, XT_REG_SPECIAL, 0 },
	{ "epc1", 0xB1, XT_REG_SPECIAL, 0 },
	{ "epc2", 0xB2, XT_REG_SPECIAL, 0 },
	{ "epc3", 0xB3, XT_REG_SPECIAL, 0 },
	{ "epc4", 0xB4, XT_REG_SPECIAL, 0 },
	{ "epc5", 0xB5, XT_REG_SPECIAL, 0 },
	{ "epc6", 0xB6, XT_REG_SPECIAL, 0 },
	{ "epc7", 0xB7, XT_REG_SPECIAL, 0 },
	{ "depc", 0xC0, XT_REG_SPECIAL, 0 },
	{ "eps2", 0xC2, XT_REG_SPECIAL, 0 },
	{ "eps3", 0xC3, XT_REG_SPECIAL, 0 },
	{ "eps4", 0xC4, XT_REG_SPECIAL, 0 },
	{ "eps5", 0xC5, XT_REG_SPECIAL, 0 },
	{ "eps6", 0xC6, XT_REG_SPECIAL, 0 },
	{ "eps7", 0xC7, XT_REG_SPECIAL, 0 },
	{ "excsave1", 0xD1, XT_REG_SPECIAL, 0 },
	{ "excsave2", 0xD2, XT_REG_SPECIAL, 0 },
	{ "excsave3", 0xD3, XT_REG_SPECIAL, 0 },
	{ "excsave4", 0xD4, XT_REG_SPECIAL, 0 },
	{ "excsave5", 0xD5, XT_REG_SPECIAL, 0 },
	{ "excsave6", 0xD6, XT_REG_SPECIAL, 0 },
	{ "excsave7", 0xD7, XT_REG_SPECIAL, 0 },
	{ "cpenable", 0xE0, XT_REG_SPECIAL, 0 },
	{ "interrupt", 0xE2, XT_REG_SPECIAL, 0 },
	{ "intset", 0xE2, XT_REG_SPECIAL, XT_REGF_NOREAD },
	{ "intclear", 0xE3, XT_REG_SPECIAL, XT_REGF_NOREAD },
	{ "intenable", 0xE4, XT_REG_SPECIAL, 0 },
	{ "vecbase", 0xE7, XT_REG_SPECIAL, 0 },
	{ "exccause", 0xE8, XT_REG_SPECIAL, 0 },
	{ "debugcause", 0xE9, XT_REG_SPECIAL, 0 },
	{ "ccount", 0xEA, XT_REG_SPECIAL, 0 },
	{ "prid", 0xEB, XT_REG_SPECIAL, 0 },
	{ "icount", 0xEC, XT_REG_SPECIAL, 0 },
	{ "icountlevel", 0xED, XT_REG_SPECIAL, 0 },
	{ "excvaddr", 0xEE, XT_REG_SPECIAL, 0 },
	{ "ccompare0", 0xF0, XT_REG_SPECIAL, 0 },
	{ "ccompare1", 0xF1, XT_REG_SPECIAL, 0 },
	{ "ccompare2", 0xF2, XT_REG_SPECIAL, 0 },
	{ "misc0", 0xF4, XT_REG_SPECIAL, 0 },
	{ "misc1", 0xF5, XT_REG_SPECIAL, 0 },
	{ "misc2", 0xF6, XT_REG_SPECIAL, 0 },
	{ "misc3", 0xF7, XT_REG_SPECIAL, 0 },
	{ "litbase", 0x05, XT_REG_SPECIAL, 0 },
	{ "ptevaddr", 0x53, XT_REG_SPECIAL, 0 },
	{ "rasid", 0x5A, XT_REG_SPECIAL, 0 },
	{ "itlbcfg", 0x5B, XT_REG_SPECIAL, 0 },
	{ "dtlbcfg", 0x5C, XT_REG_SPECIAL, 0 },
	{ "mepc", 0x6A, XT_REG_SPECIAL, 0 },
	{ "meps", 0x6B, XT_REG_SPECIAL, 0 },
	{ "mesave", 0x6C, XT_REG_SPECIAL, 0 },
	{ "mesr", 0x6D, XT_REG_SPECIAL, 0 },
	{ "mecr", 0x6E, XT_REG_SPECIAL, 0 },
	{ "mevaddr", 0x6F, XT_REG_SPECIAL, 0 },
	{ "a0", XT_REG_IDX_AR0, XT_REG_RELGEN, 0 },	/* WARNING: For these registers, regnum points to the */
	{ "a1", XT_REG_IDX_AR1, XT_REG_RELGEN, 0 },	/* index of the corresponding ARxregisters, NOT to */
	{ "a2", XT_REG_IDX_AR2, XT_REG_RELGEN, 0 },	/* the processor register number! */
	{ "a3", XT_REG_IDX_AR3, XT_REG_RELGEN, 0 },
	{ "a4", XT_REG_IDX_AR4, XT_REG_RELGEN, 0 },
	{ "a5", XT_REG_IDX_AR5, XT_REG_RELGEN, 0 },
	{ "a6", XT_REG_IDX_AR6, XT_REG_RELGEN, 0 },
	{ "a7", XT_REG_IDX_AR7, XT_REG_RELGEN, 0 },
	{ "a8", XT_REG_IDX_AR8, XT_REG_RELGEN, 0 },
	{ "a9", XT_REG_IDX_AR9, XT_REG_RELGEN, 0 },
	{ "a10", XT_REG_IDX_AR10, XT_REG_RELGEN, 0 },
	{ "a11", XT_REG_IDX_AR11, XT_REG_RELGEN, 0 },
	{ "a12", XT_REG_IDX_AR12, XT_REG_RELGEN, 0 },
	{ "a13", XT_REG_IDX_AR13, XT_REG_RELGEN, 0 },
	{ "a14", XT_REG_IDX_AR14, XT_REG_RELGEN, 0 },
	{ "a15", XT_REG_IDX_AR15, XT_REG_RELGEN, 0 },

	{ "pwrctl", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "pwrstat", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "eristat", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "cs_itctrl", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "cs_claimset", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "cs_claimclr", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "cs_lockaccess", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "cs_lockstatus", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "cs_authstatus", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "fault_info", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "trax_id", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "trax_ctrl", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "trax_stat", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "trax_data", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "trax_addr", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "trax_pctrigger", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "trax_pcmatch", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "trax_delay", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "trax_memstart", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "trax_memend", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "pmg", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "pmoc", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "pm0", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "pm1", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "pmctrl0", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "pmctrl1", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "pmstat0", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "pmstat1", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "ocd_id", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "ocd_dcrclr", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "ocd_dcrset", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "ocd_dsr", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
	{ "ddr", 0x68, XT_REG_DEBUG, XT_REGF_NOREAD },
};


/**
 * Types of memory used at xtensa target
 */
enum xtensa_mem_region_type {
	XTENSA_MEM_REG_IROM = 0x0,
	XTENSA_MEM_REG_IRAM,
	XTENSA_MEM_REG_DROM,
	XTENSA_MEM_REG_DRAM,
	XTENSA_MEM_REG_URAM,
	XTENSA_MEM_REG_XLMI,
	XTENSA_MEM_REGS_NUM
};

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
	case XTENSA_MEM_REG_URAM:
		return &xtensa->core_config->uram;
	case XTENSA_MEM_REG_XLMI:
		return &xtensa->core_config->xlmi;
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

static int xtensa_core_reg_get(struct reg *reg)
{
	/*We don't need this because we read all registers on halt anyway. */
	struct xtensa *xtensa = (struct xtensa *)reg->arch_info;
	struct target *target = xtensa->target;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;
	return ERROR_OK;
}

static int xtensa_core_reg_set(struct reg *reg, uint8_t *buf)
{
	struct xtensa *xtensa = (struct xtensa *)reg->arch_info;
	struct target *target = xtensa->target;

	assert(reg->size <= 64 && "up to 64-bit regs are supported only!");
	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	buf_cpy(buf, reg->value, reg->size);
	reg->dirty = true;
	reg->valid = true;

	return ERROR_OK;
}

static const struct reg_arch_type xtensa_reg_type = {
	.get = xtensa_core_reg_get,
	.set = xtensa_core_reg_set,
};

const struct reg_arch_type xtensa_user_reg_u32_type = {
	.get = xtensa_core_reg_get,
	.set = xtensa_core_reg_set,
};

const struct reg_arch_type xtensa_user_reg_u128_type = {
	.get = xtensa_core_reg_get,
	.set = xtensa_core_reg_set,
};

static inline size_t xtensa_insn_size_get(uint32_t insn)
{
	return insn & BIT(3) ? 2 : XT_ISNS_SZ_MAX;
}

/* Convert a register index that's indexed relative to windowbase, to the real address. */
static enum xtensa_reg_id xtensa_windowbase_offset_to_canonical(enum xtensa_reg_id reg_idx, int windowbase)
{
	unsigned int idx;
	if (reg_idx >= XT_REG_IDX_AR0 && reg_idx <= XT_REG_IDX_AR63) {
		idx = reg_idx - XT_REG_IDX_AR0;
	} else if (reg_idx >= XT_REG_IDX_A0 && reg_idx <= XT_REG_IDX_A15) {
		idx = reg_idx - XT_REG_IDX_A0;
	} else {
		LOG_ERROR("Error: can't convert register %d to non-windowbased register!", reg_idx);
		return -1;
	}
	return ((idx + windowbase * 4) & 63) + XT_REG_IDX_AR0;
}

static enum xtensa_reg_id xtensa_canonical_to_windowbase_offset(enum xtensa_reg_id reg_idx, int windowbase)
{
	return xtensa_windowbase_offset_to_canonical(reg_idx, -windowbase);
}

static void xtensa_mark_register_dirty(struct xtensa *xtensa, enum xtensa_reg_id reg_idx)
{
	struct reg *reg_list = xtensa->core_cache->reg_list;
	reg_list[reg_idx].dirty = true;
}

static int xtensa_queue_dbg_reg_read(struct xtensa *xtensa, unsigned int reg, uint8_t *data)
{
	struct xtensa_debug_module *dm = &xtensa->dbg_mod;

	if (!xtensa->core_config->trace.enabled &&
		(reg <= NARADR_MEMADDREND || (reg >= NARADR_PMG && reg <= NARADR_PMSTAT7))) {
		LOG_ERROR("Can not access %u reg when Trace Port option disabled!", reg);
		return ERROR_FAIL;
	}
	return dm->dbg_ops->queue_reg_read(dm, reg, data);
}

static int xtensa_queue_dbg_reg_write(struct xtensa *xtensa, unsigned int reg, uint32_t data)
{
	struct xtensa_debug_module *dm = &xtensa->dbg_mod;

	if (!xtensa->core_config->trace.enabled &&
		(reg <= NARADR_MEMADDREND || (reg >= NARADR_PMG && reg <= NARADR_PMSTAT7))) {
		LOG_ERROR("Can not access %u reg when Trace Port option disabled!", reg);
		return ERROR_FAIL;
	}
	return dm->dbg_ops->queue_reg_write(dm, reg, data);
}

static void xtensa_queue_exec_ins(struct xtensa *xtensa, uint32_t ins)
{
	xtensa_queue_dbg_reg_write(xtensa, NARADR_DIR0EXEC, ins);
}

static bool xtensa_reg_is_readable(enum xtensa_reg_flags flags, xtensa_reg_val_t cpenable)
{
	if (flags & XT_REGF_NOREAD)
		return false;
	if ((flags & XT_REGF_COPROC0) && (cpenable & BIT(0)) == 0)
		return false;
	return true;
}

static int xtensa_queue_pwr_reg_write(struct xtensa *xtensa, unsigned int reg, uint32_t data)
{
	struct xtensa_debug_module *dm = &xtensa->dbg_mod;
	return dm->pwr_ops->queue_reg_write(dm, reg, data);
}

static bool xtensa_special_reg_exists(struct xtensa *xtensa, enum xtensa_reg_id reg_idx)
{
	/* TODO: array of size XT_NUM_REGS can be used here to map special register ID to
	 * corresponding config option 'enabled' flag */
	if (reg_idx >= XT_REG_IDX_LBEG && reg_idx <= XT_REG_IDX_LCOUNT)
		return xtensa->core_config->loop;
	else if (reg_idx == XT_REG_IDX_BR)
		return xtensa->core_config->boolean;
	else if (reg_idx == XT_REG_IDX_LITBASE)
		return xtensa->core_config->ext_l32r;
	else if (reg_idx == XT_REG_IDX_SCOMPARE1 || reg_idx == XT_REG_IDX_ATOMCTL)
		return xtensa->core_config->cond_store;
	else if (reg_idx >= XT_REG_IDX_ACCLO && reg_idx <= XT_REG_IDX_M3)
		return xtensa->core_config->mac16;
	else if (reg_idx == XT_REG_IDX_WINDOWBASE || reg_idx == XT_REG_IDX_WINDOWSTART)
		return xtensa->core_config->windowed;
	else if (reg_idx >= XT_REG_IDX_PTEVADDR && reg_idx <= XT_REG_IDX_DTLBCFG)
		return xtensa->core_config->mmu.enabled;
	else if (reg_idx == XT_REG_IDX_MMID)
		return xtensa->core_config->trace.enabled;
	else if (reg_idx >= XT_REG_IDX_MEPC && reg_idx <= XT_REG_IDX_MEVADDR)
		return xtensa->core_config->mem_err_check;
	else if (reg_idx == XT_REG_IDX_CPENABLE)
		return xtensa->core_config->coproc;
	else if (reg_idx == XT_REG_IDX_VECBASE)
		return xtensa->core_config->reloc_vec;
	else if (reg_idx == XT_REG_IDX_CCOUNT)
		return xtensa->core_config->tim_irq.enabled;
	else if (reg_idx >= XT_REG_IDX_CCOMPARE0 && reg_idx <= XT_REG_IDX_CCOMPARE2)
		return xtensa->core_config->tim_irq.enabled &&
		       (reg_idx - XT_REG_IDX_CCOMPARE0 < xtensa->core_config->tim_irq.comp_num);
	else if (reg_idx == XT_REG_IDX_PRID)
		return xtensa->core_config->proc_id;
	else if (reg_idx >= XT_REG_IDX_MISC0 && reg_idx <= XT_REG_IDX_MISC3)
		return reg_idx - XT_REG_IDX_MISC0 < xtensa->core_config->miscregs_num;
	return true;
}

static bool xtensa_user_reg_exists(struct xtensa *xtensa, enum xtensa_reg_id reg_idx)
{
	if (reg_idx == XT_REG_IDX_THREADPTR)
		return xtensa->core_config->threadptr;
	if (reg_idx == XT_REG_IDX_FCR || reg_idx == XT_REG_IDX_FSR)
		return xtensa->core_config->fp_coproc;
	return false;
}

static inline bool xtensa_fp_reg_exists(struct xtensa *xtensa, enum xtensa_reg_id reg_idx)
{
	return xtensa->core_config->fp_coproc;
}

static inline bool xtensa_regular_reg_exists(struct xtensa *xtensa, enum xtensa_reg_id reg_idx)
{
	if (reg_idx >= XT_REG_IDX_AR0 && reg_idx <= XT_REG_IDX_AR63)
		return reg_idx - XT_REG_IDX_AR0 < xtensa->core_config->aregs_num;
	return true;
}

static int xtensa_write_dirty_registers(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int res;
	xtensa_reg_val_t regval, windowbase = 0;
	bool scratch_reg_dirty = false;
	struct reg *reg_list = xtensa->core_cache->reg_list;

	LOG_TARGET_DEBUG(target, "start");

	/*We need to write the dirty registers in the cache list back to the processor.
	 *Start by writing the SFR/user registers. */
	for (unsigned int i = 0; i < XT_NUM_REGS; i++) {
		if (reg_list[i].dirty) {
			if (xtensa_regs[i].type == XT_REG_SPECIAL ||
				xtensa_regs[i].type == XT_REG_USER ||
				xtensa_regs[i].type == XT_REG_FR) {
				scratch_reg_dirty = true;
				regval = xtensa_reg_get(target, i);
				LOG_TARGET_DEBUG(target, "Writing back reg %s val %08" PRIX32,
					xtensa_regs[i].name,
					regval);
				xtensa_queue_dbg_reg_write(xtensa, NARADR_DDR, regval);
				xtensa_queue_exec_ins(xtensa, XT_INS_RSR(XT_SR_DDR, XT_REG_A3));
				if (xtensa_regs[i].type == XT_REG_USER) {
					if (reg_list[i].exist)
						xtensa_queue_exec_ins(xtensa,
							XT_INS_WUR(xtensa_regs[i].reg_num,
								XT_REG_A3));
				} else if (xtensa_regs[i].type == XT_REG_FR) {
					if (reg_list[i].exist)
						xtensa_queue_exec_ins(xtensa,
							XT_INS_WFR(xtensa_regs[i].reg_num,
								XT_REG_A3));
				} else {/*SFR */
					if (reg_list[i].exist) {
						unsigned int reg_num = xtensa_regs[i].reg_num;
						if (reg_num == XT_PC_REG_NUM_BASE)
							/* reg number of PC for debug interrupt
							 * depends on NDEBUGLEVEL */
							reg_num += xtensa->core_config->debug.irq_level;

						xtensa_queue_exec_ins(xtensa,
							XT_INS_WSR(reg_num, XT_REG_A3));
					}
				}
				reg_list[i].dirty = false;
			}
		}
	}
	if (scratch_reg_dirty)
		xtensa_mark_register_dirty(xtensa, XT_REG_IDX_A3);

	if (xtensa->core_config->user_regs_num > 0 &&
		xtensa->core_config->queue_write_dirty_user_regs)
		xtensa->core_config->queue_write_dirty_user_regs(target);

	if (xtensa->core_config->windowed) {
		/*Grab the windowbase, we need it. */
		windowbase = xtensa_reg_get(target, XT_REG_IDX_WINDOWBASE);
		/*Check if there are problems with both the ARx as well as the corresponding Rx
		 * registers set and dirty. */
		/*Warn the user if this happens, not much else we can do... */
		for (unsigned int i = XT_REG_IDX_A0; i <= XT_REG_IDX_A15; i++) {
			unsigned int j = xtensa_windowbase_offset_to_canonical(i, windowbase);
			if (reg_list[i].dirty && reg_list[j].dirty) {
				if (memcmp(reg_list[i].value, reg_list[j].value,
						sizeof(xtensa_reg_val_t)) != 0)
					LOG_WARNING(
						"Warning: Both A%d as well as the physical register it points to (AR%d) are dirty and differs in value. Results are undefined!",
						i - XT_REG_IDX_A0,
						j - XT_REG_IDX_AR0);
			}
		}
	}

	/*Write A0-A16 */
	for (unsigned int i = 0; i < 16; i++) {
		if (reg_list[XT_REG_IDX_A0 + i].dirty) {
			regval = xtensa_reg_get(target, XT_REG_IDX_A0 + i);
			LOG_TARGET_DEBUG(target, "Writing back reg %s value %08" PRIX32 ", num =%i",
				xtensa_regs[XT_REG_IDX_A0 + i].name,
				regval,
				xtensa_regs[XT_REG_IDX_A0 + i].reg_num);
			xtensa_queue_dbg_reg_write(xtensa, NARADR_DDR, regval);
			xtensa_queue_exec_ins(xtensa, XT_INS_RSR(XT_SR_DDR, i));
			reg_list[XT_REG_IDX_A0 + i].dirty = false;
		}
	}

	if (xtensa->core_config->windowed) {
		/*Now write AR0-AR63. */
		for (unsigned int j = 0; j < 64; j += 16) {
			/*Write the 16 registers we can see */
			for (unsigned int i = 0; i < 16; i++) {
				if (i + j < xtensa->core_config->aregs_num) {
					enum xtensa_reg_id realadr =
						xtensa_windowbase_offset_to_canonical(XT_REG_IDX_AR0 + i + j,
						windowbase);
					/*Write back any dirty un-windowed registers */
					if (reg_list[realadr].dirty) {
						regval = xtensa_reg_get(target, realadr);
						LOG_TARGET_DEBUG(
							target,
							"Writing back reg %s value %08" PRIX32 ", num =%i",
							xtensa_regs[realadr].name,
							regval,
							xtensa_regs[realadr].reg_num);
						xtensa_queue_dbg_reg_write(xtensa, NARADR_DDR, regval);
						xtensa_queue_exec_ins(xtensa,
							XT_INS_RSR(XT_SR_DDR, xtensa_regs[XT_REG_IDX_AR0 + i].reg_num));
						reg_list[realadr].dirty = false;
					}
				}
			}
			/*Now rotate the window so we'll see the next 16 registers. The final rotate
			 * will wraparound, */
			/*leaving us in the state we were. */
			xtensa_queue_exec_ins(xtensa, XT_INS_ROTW(4));
		}
	}
	res = jtag_execute_queue();
	xtensa_core_status_check(target);

	return res;
}

int xtensa_queue_write_dirty_user_regs_u32(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	struct reg *reg_list = xtensa->core_cache->reg_list;
	xtensa_reg_val_t reg_val;
	bool scratch_reg_dirty = false;

	LOG_TARGET_DEBUG(target, "start");

	/* We need to write the dirty registers in the cache list back to the processor.
	 * Start by writing the SFR/user registers. */
	for (unsigned int i = 0; i < xtensa->core_config->user_regs_num; i++) {
		if (!reg_list[XT_USR_REG_START + i].dirty)
			continue;
		scratch_reg_dirty = true;
		reg_val = xtensa_reg_get(target, XT_USR_REG_START + i);
		LOG_TARGET_DEBUG(target, "Writing back reg %s val %08" PRIX32,
			xtensa->core_config->user_regs[i].name,
			reg_val);
		xtensa_queue_dbg_reg_write(xtensa, NARADR_DDR, reg_val);
		xtensa_queue_exec_ins(xtensa, XT_INS_RSR(XT_SR_DDR, XT_REG_A3));
		xtensa_queue_exec_ins(xtensa,
			XT_INS_WUR(xtensa->core_config->user_regs[i].reg_num,
				XT_REG_A3));
		reg_list[XT_USR_REG_START + i].dirty = false;
	}
	if (scratch_reg_dirty)
		xtensa_mark_register_dirty(xtensa, XT_REG_IDX_A3);

	return ERROR_OK;
}

static inline bool xtensa_is_stopped(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	return xtensa->dbg_mod.core_status.dsr & OCDDSR_STOPPED;
}

int xtensa_examine(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	unsigned int cmd = PWRCTL_DEBUGWAKEUP | PWRCTL_MEMWAKEUP | PWRCTL_COREWAKEUP;

	LOG_DEBUG("coreid = %d", target->coreid);
	xtensa_queue_pwr_reg_write(xtensa, DMREG_PWRCTL, cmd);
	xtensa_queue_pwr_reg_write(xtensa, DMREG_PWRCTL, cmd | PWRCTL_JTAGDEBUGUSE);
	xtensa_dm_queue_enable(&xtensa->dbg_mod);
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	int res = jtag_execute_queue();
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
	unsigned int cmd = PWRCTL_DEBUGWAKEUP | PWRCTL_MEMWAKEUP | PWRCTL_COREWAKEUP;

	if (xtensa->reset_asserted)
		cmd |= PWRCTL_CORERESET;
	xtensa_queue_pwr_reg_write(xtensa, DMREG_PWRCTL, cmd);
	/* TODO: can we join this with the write above? */
	xtensa_queue_pwr_reg_write(xtensa, DMREG_PWRCTL, cmd | PWRCTL_JTAGDEBUGUSE);
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	return jtag_execute_queue();
}

int xtensa_smpbreak_write(struct xtensa *xtensa, uint32_t set)
{
	uint32_t dsr_data = 0x00110000;
	uint32_t clear = (set | OCDDCR_ENABLEOCD) ^
		(OCDDCR_BREAKINEN | OCDDCR_BREAKOUTEN | OCDDCR_RUNSTALLINEN |
		OCDDCR_DEBUGMODEOUTEN | OCDDCR_ENABLEOCD);

	LOG_TARGET_DEBUG(xtensa->target, "write smpbreak set=0x%" PRIx32 " clear=0x%" PRIx32, set, clear);
	xtensa_queue_dbg_reg_write(xtensa, NARADR_DCRSET, set | OCDDCR_ENABLEOCD);
	xtensa_queue_dbg_reg_write(xtensa, NARADR_DCRCLR, clear);
	xtensa_queue_dbg_reg_write(xtensa, NARADR_DSR, dsr_data);
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	return jtag_execute_queue();
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

	xtensa_queue_dbg_reg_read(xtensa, NARADR_DCRSET, dcr_buf);
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	int res = jtag_execute_queue();
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
		return xtensa->suppress_dsr_errors ? ERROR_OK : ERROR_FAIL;
	}
	return ERROR_OK;
}

xtensa_reg_val_t xtensa_reg_get(struct target *target, enum xtensa_reg_id reg_id)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	struct reg *reg = &xtensa->core_cache->reg_list[reg_id];
	assert(reg_id < xtensa->core_cache->num_regs && "Attempt to access non-existing reg!");
	return xtensa_reg_get_value(reg);
}

void xtensa_reg_set(struct target *target, enum xtensa_reg_id reg_id, xtensa_reg_val_t value)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	struct reg *reg = &xtensa->core_cache->reg_list[reg_id];
	assert(reg_id < xtensa->core_cache->num_regs && "Attempt to access non-existing reg!");
	if (xtensa_reg_get_value(reg) == value)
		return;
	xtensa_reg_set_value(reg, value);
}

int xtensa_assert_reset(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);

	LOG_TARGET_DEBUG(target, "target_number=%i, begin", target->target_number);
	target->state = TARGET_RESET;
	xtensa_queue_pwr_reg_write(xtensa,
		DMREG_PWRCTL,
		PWRCTL_JTAGDEBUGUSE | PWRCTL_DEBUGWAKEUP | PWRCTL_MEMWAKEUP | PWRCTL_COREWAKEUP |
		PWRCTL_CORERESET);
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	int res = jtag_execute_queue();
	if (res != ERROR_OK)
		return res;
	xtensa->reset_asserted = true;
	return res;
}

int xtensa_deassert_reset(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);

	LOG_TARGET_DEBUG(target, "halt=%d", target->reset_halt);
	if (target->reset_halt)
		xtensa_queue_dbg_reg_write(xtensa,
			NARADR_DCRSET,
			OCDDCR_ENABLEOCD | OCDDCR_DEBUGINTERRUPT);
	xtensa_queue_pwr_reg_write(xtensa,
		DMREG_PWRCTL,
		PWRCTL_JTAGDEBUGUSE | PWRCTL_DEBUGWAKEUP | PWRCTL_MEMWAKEUP | PWRCTL_COREWAKEUP);
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	int res = jtag_execute_queue();
	if (res != ERROR_OK)
		return res;
	target->state = TARGET_RUNNING;
	xtensa->reset_asserted = false;
	return res;
}

int xtensa_fetch_all_regs(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	struct reg *reg_list = xtensa->core_cache->reg_list;
	xtensa_reg_val_t cpenable = 0, windowbase = 0;
	uint8_t regvals[XT_NUM_REGS][sizeof(xtensa_reg_val_t)];
	uint8_t dsrs[XT_NUM_REGS][sizeof(xtensa_dsr_t)];
	bool debug_dsrs = !xtensa->regs_fetched || LOG_LEVEL_IS(LOG_LVL_DEBUG);

	LOG_TARGET_DEBUG(target, "start");

	/* Assume the CPU has just halted. We now want to fill the register cache with all the
	 * register contents GDB needs. For speed, we pipeline all the read operations, execute them
	 * in one go, then sort everything out from the regvals variable. */

	/* Start out with AREGS; we can reach those immediately. Grab them per 16 registers. */
	for (unsigned int j = 0; j < XT_AREGS_NUM_MAX; j += 16) {
		/*Grab the 16 registers we can see */
		for (unsigned int i = 0; i < 16; i++) {
			if (i + j < xtensa->core_config->aregs_num) {
				xtensa_queue_exec_ins(xtensa,
					XT_INS_WSR(XT_SR_DDR, xtensa_regs[XT_REG_IDX_AR0 + i].reg_num));
				xtensa_queue_dbg_reg_read(xtensa, NARADR_DDR, regvals[XT_REG_IDX_AR0 + i + j]);
				if (debug_dsrs)
					xtensa_queue_dbg_reg_read(xtensa, NARADR_DSR, dsrs[XT_REG_IDX_AR0 + i + j]);
			}
		}
		if (xtensa->core_config->windowed) {
			/* Now rotate the window so we'll see the next 16 registers. The final rotate
			 * will wraparound, */
			/* leaving us in the state we were. */
			xtensa_queue_exec_ins(xtensa, XT_INS_ROTW(4));
		}
	}
	if (xtensa->core_config->coproc) {
		/* As the very first thing after AREGS, go grab the CPENABLE registers. It indicates
		 * if we can also grab the FP */
		/* (and theoretically other coprocessor) registers, or if this is a bad thing to do.*/
		xtensa_queue_exec_ins(xtensa, XT_INS_RSR(xtensa_regs[XT_REG_IDX_CPENABLE].reg_num, XT_REG_A3));
		xtensa_queue_exec_ins(xtensa, XT_INS_WSR(XT_SR_DDR, XT_REG_A3));
		xtensa_queue_dbg_reg_read(xtensa, NARADR_DDR, regvals[XT_REG_IDX_CPENABLE]);
	}
	int res = jtag_execute_queue();
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to read ARs (%d)!", res);
		return res;
	}
	xtensa_core_status_check(target);

	if (xtensa->core_config->coproc)
		cpenable = buf_get_u32(regvals[XT_REG_IDX_CPENABLE], 0, 32);
	/* We're now free to use any of A0-A15 as scratch registers
	 * Grab the SFRs and user registers first. We use A3 as a scratch register. */
	for (unsigned int i = 0; i < XT_NUM_REGS; i++) {
		if (xtensa_reg_is_readable(xtensa_regs[i].flags, cpenable) && reg_list[i].exist &&
			(xtensa_regs[i].type == XT_REG_SPECIAL ||
				xtensa_regs[i].type == XT_REG_USER || xtensa_regs[i].type == XT_REG_FR)) {
			if (xtensa_regs[i].type == XT_REG_USER) {
				xtensa_queue_exec_ins(xtensa, XT_INS_RUR(xtensa_regs[i].reg_num, XT_REG_A3));
			} else if (xtensa_regs[i].type == XT_REG_FR) {
				xtensa_queue_exec_ins(xtensa, XT_INS_RFR(xtensa_regs[i].reg_num, XT_REG_A3));
			} else {	/*SFR */
				unsigned int reg_num = xtensa_regs[i].reg_num;
				if (reg_num == XT_PC_REG_NUM_BASE) {
					/* reg number of PC for debug interrupt depends on NDEBUGLEVEL */
					reg_num += xtensa->core_config->debug.irq_level;
				}
				xtensa_queue_exec_ins(xtensa, XT_INS_RSR(reg_num, XT_REG_A3));
			}
			xtensa_queue_exec_ins(xtensa, XT_INS_WSR(XT_SR_DDR, XT_REG_A3));
			xtensa_queue_dbg_reg_read(xtensa, NARADR_DDR, regvals[i]);
			if (debug_dsrs)
				xtensa_queue_dbg_reg_read(xtensa, NARADR_DSR, dsrs[i]);
		}
	}
	/* Ok, send the whole mess to the CPU. */
	res = jtag_execute_queue();
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to fetch AR regs!");
		return res;
	}
	xtensa_core_status_check(target);

	if (debug_dsrs) {
		/* DSR checking: follows order in which registers are requested. */
		for (unsigned int i = 0; i < XT_NUM_REGS; i++) {
			if (xtensa_reg_is_readable(xtensa_regs[i].flags, cpenable) && reg_list[i].exist &&
				(xtensa_regs[i].type == XT_REG_SPECIAL || xtensa_regs[i].type == XT_REG_USER ||
					xtensa_regs[i].type == XT_REG_FR)) {
				if (buf_get_u32(dsrs[i], 0, 32) & OCDDSR_EXECEXCEPTION) {
					LOG_ERROR("Exception reading %s!", xtensa_regs[i].name);
					return ERROR_FAIL;
				}
			}
		}
	}

	if (xtensa->core_config->user_regs_num > 0 && xtensa->core_config->fetch_user_regs) {
		res = xtensa->core_config->fetch_user_regs(target);
		if (res != ERROR_OK)
			return res;
	}

	if (xtensa->core_config->windowed) {
		/* We need the windowbase to decode the general addresses. */
		windowbase = buf_get_u32(regvals[XT_REG_IDX_WINDOWBASE], 0, 32);
	}
	/* Decode the result and update the cache. */
	for (unsigned int i = 0; i < XT_NUM_REGS; i++) {
		if (xtensa_reg_is_readable(xtensa_regs[i].flags, cpenable) && reg_list[i].exist) {
			if (xtensa_regs[i].type == XT_REG_GENERAL) {
				/* TODO: add support for non-windowed configs */
				assert(
					xtensa->core_config->windowed &&
					"Regs fetch is not supported for non-windowed configs!");
				/* The 64-value general register set is read from (windowbase) on down.
				 * We need to get the real register address by subtracting windowbase and
				 * wrapping around. */
				int realadr = xtensa_canonical_to_windowbase_offset(i, windowbase);
				buf_cpy(regvals[realadr], reg_list[i].value, reg_list[i].size);
			} else if (xtensa_regs[i].type == XT_REG_RELGEN) {
				buf_cpy(regvals[xtensa_regs[i].reg_num], reg_list[i].value, reg_list[i].size);
			} else {
				buf_cpy(regvals[i], reg_list[i].value, reg_list[i].size);
			}
			reg_list[i].valid = true;
		} else {
			reg_list[i].valid = false;
		}
	}
	/* We have used A3 as a scratch register and we will need to write that back. */
	xtensa_mark_register_dirty(xtensa, XT_REG_IDX_A3);
	xtensa->regs_fetched = true;

	return ERROR_OK;
}

int xtensa_fetch_user_regs_u32(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	struct reg *reg_list = xtensa->core_cache->reg_list;
	xtensa_reg_val_t cpenable = 0;
	uint8_t regvals[XT_USER_REGS_NUM_MAX][sizeof(xtensa_reg_val_t)];
	uint8_t dsrs[XT_USER_REGS_NUM_MAX][sizeof(xtensa_dsr_t)];
	bool debug_dsrs = !xtensa->regs_fetched || LOG_LEVEL_IS(LOG_LVL_DEBUG);

	assert(xtensa->core_config->user_regs_num < XT_USER_REGS_NUM_MAX && "Too many user regs configured!");
	if (xtensa->core_config->coproc)
		cpenable = xtensa_reg_get(target, XT_REG_IDX_CPENABLE);

	for (unsigned int i = 0; i < xtensa->core_config->user_regs_num; i++) {
		if (!xtensa_reg_is_readable(xtensa->core_config->user_regs[i].flags, cpenable))
			continue;
		xtensa_queue_exec_ins(xtensa, XT_INS_RUR(xtensa->core_config->user_regs[i].reg_num, XT_REG_A3));
		xtensa_queue_exec_ins(xtensa, XT_INS_WSR(XT_SR_DDR, XT_REG_A3));
		xtensa_queue_dbg_reg_read(xtensa, NARADR_DDR, regvals[i]);
		if (debug_dsrs)
			xtensa_queue_dbg_reg_read(xtensa, NARADR_DSR, dsrs[i]);
	}
	/* Ok, send the whole mess to the CPU. */
	int res = jtag_execute_queue();
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to fetch AR regs!");
		return res;
	}
	xtensa_core_status_check(target);

	if (debug_dsrs) {
		/* DSR checking: follows order in which registers are requested. */
		for (unsigned int i = 0; i < xtensa->core_config->user_regs_num; i++) {
			if (!xtensa_reg_is_readable(xtensa->core_config->user_regs[i].flags, cpenable))
				continue;
			if (buf_get_u32(dsrs[i], 0, 32) & OCDDSR_EXECEXCEPTION) {
				LOG_ERROR("Exception reading %s!", xtensa->core_config->user_regs[i].name);
				return ERROR_FAIL;
			}
		}
	}

	for (unsigned int i = 0; i < xtensa->core_config->user_regs_num; i++) {
		if (xtensa_reg_is_readable(xtensa->core_config->user_regs[i].flags, cpenable)) {
			buf_cpy(regvals[i], reg_list[XT_USR_REG_START + i].value, reg_list[XT_USR_REG_START + i].size);
			reg_list[XT_USR_REG_START + i].valid = true;
		} else {
			reg_list[XT_USR_REG_START + i].valid = false;
		}
	}

	/* We have used A3 as a scratch register and we will need to write that back. */
	xtensa_mark_register_dirty(xtensa, XT_REG_IDX_A3);
	return ERROR_OK;
}

int xtensa_get_gdb_reg_list(struct target *target,
	struct reg **reg_list[],
	int *reg_list_size,
	enum target_register_class reg_class)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	unsigned int num_regs = xtensa->core_config->gdb_general_regs_num;

	if (reg_class == REG_CLASS_ALL)
		num_regs = xtensa->regs_num;

	LOG_DEBUG("reg_class=%i, num_regs=%d", reg_class, num_regs);

	*reg_list = malloc(num_regs * sizeof(struct reg *));
	if (!*reg_list)
		return ERROR_FAIL;

	for (unsigned int k = 0; k < num_regs; k++) {
		unsigned int reg_id = xtensa->core_config->gdb_regs_mapping[k];
		(*reg_list)[k] = &xtensa->core_cache->reg_list[reg_id];
	}

	*reg_list_size = num_regs;

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
		xtensa_queue_dbg_reg_write(xtensa, NARADR_DCRSET, OCDDCR_ENABLEOCD | OCDDCR_DEBUGINTERRUPT);
		xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
		res = jtag_execute_queue();
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
		xtensa_reg_val_t cause = xtensa_reg_get(target, XT_REG_IDX_DEBUGCAUSE);
		if (cause & DEBUGCAUSE_DB) {
			/* We stopped due to a watchpoint. We can't just resume executing the
			 * instruction again because */
			/* that would trigger the watchpoint again. To fix this, we single-step,
			 * which ignores watchpoints. */
			xtensa_do_step(target, current, address, handle_breakpoints);
		}
		if (cause & (DEBUGCAUSE_BI | DEBUGCAUSE_BN)) {
			/* We stopped due to a break instruction. We can't just resume executing the
			 * instruction again because */
			/* that would trigger the break again. To fix this, we single-step, which
			 * ignores break. */
			xtensa_do_step(target, current, address, handle_breakpoints);
		}
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

	xtensa_queue_exec_ins(xtensa, XT_INS_RFDO);
	int res = jtag_execute_queue();
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
	uint8_t insn_buf[XT_ISNS_SZ_MAX];
	int err = xtensa_read_buffer(target, pc, sizeof(insn_buf), insn_buf);
	if (err != ERROR_OK)
		return false;

	xtensa_insn_t insn = buf_get_u32(insn_buf, 0, 24);
	xtensa_insn_t masked = insn & XT_INS_L32E_S32E_MASK;
	if (masked == XT_INS_L32E(0, 0, 0) || masked == XT_INS_S32E(0, 0, 0))
		return true;

	masked = insn & XT_INS_RFWO_RFWU_MASK;
	if (masked == XT_INS_RFWO || masked == XT_INS_RFWU)
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
	xtensa_reg_val_t oldps, newps, oldpc, cur_pc;

	LOG_TARGET_DEBUG(target, "current=%d, address=" TARGET_ADDR_FMT ", handle_breakpoints=%i",
		current, address, handle_breakpoints);

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_WARNING(target, "target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (xtensa->core_config->debug.icount_sz != 32) {
		LOG_TARGET_WARNING(target, "stepping for ICOUNT less then 32 bits is not implemented!");
		return ERROR_FAIL;
	}

	/* Save old ps/pc */
	oldps = xtensa_reg_get(target, XT_REG_IDX_PS);
	oldpc = xtensa_reg_get(target, XT_REG_IDX_PC);

	cause = xtensa_reg_get(target, XT_REG_IDX_DEBUGCAUSE);
	LOG_TARGET_DEBUG(target, "oldps=%" PRIx32 ", oldpc=%" PRIx32 " dbg_cause=%" PRIx32 " exc_cause=%" PRIx32,
		oldps,
		oldpc,
		cause,
		xtensa_reg_get(target, XT_REG_IDX_EXCCAUSE));
	if (handle_breakpoints && (cause & (DEBUGCAUSE_BI | DEBUGCAUSE_BN))) {
		/* handle hard-coded SW breakpoints (e.g. syscalls) */
		LOG_TARGET_DEBUG(target, "Increment PC to pass break instruction...");
		xtensa_reg_set(target, XT_REG_IDX_DEBUGCAUSE, 0);	/* so we don't recurse into the same routine */
		xtensa->core_cache->reg_list[XT_REG_IDX_DEBUGCAUSE].dirty = false;
		/* pretend that we have stepped */
		if (cause & DEBUGCAUSE_BI)
			xtensa_reg_set(target, XT_REG_IDX_PC, oldpc + 3);	/* PC = PC+3 */
		else
			xtensa_reg_set(target, XT_REG_IDX_PC, oldpc + 2);	/* PC = PC+2 */
		return ERROR_OK;
	}

	/* Xtensa has an ICOUNTLEVEL register which sets the maximum interrupt level at which the
	 * instructions are to be counted while stepping.
	 * For example, if we need to step by 2 instructions, and an interrupt occurs inbetween,
	 * the processor will execute the interrupt, return, and halt after the 2nd instruction.
	 * However, sometimes we don't want the interrupt handlers to be executed at all, while
	 * stepping through the code. In this case (XT_STEPPING_ISR_OFF), PS.INTLEVEL can be raised
	 * to only allow Debug and NMI interrupts.
	 */
	if (xtensa->stepping_isr_mode == XT_STEPPING_ISR_OFF) {
		if (!xtensa->core_config->high_irq.enabled) {
			LOG_TARGET_WARNING(
				target,
				"disabling IRQs while stepping is not implemented w/o high prio IRQs option!");
			return ERROR_FAIL;
		}
		/* Mask all interrupts below Debug, i.e. PS.INTLEVEL = DEBUGLEVEL - 1 */
		xtensa_reg_val_t temp_ps = (oldps & ~0xF) | (xtensa->core_config->debug.irq_level - 1);
		xtensa_reg_set(target, XT_REG_IDX_PS, temp_ps);
	}
	/* Regardless of ISRs masking mode we need to count instructions at any CINTLEVEL during step.
	    So set `icountlvl` to DEBUGLEVEL.
	    If ISRs are masked they are disabled in PS (see above), so having `icountlvl` set to DEBUGLEVEL
	    will allow to step through any type of the code, e.g. 'high int level' ISR.
	    If ISRs are not masked With `icountlvl` set to DEBUGLEVEL, we can step into any ISR
	    which can happen (enabled in PS).
	*/
	icountlvl = xtensa->core_config->debug.irq_level;

	if (cause & DEBUGCAUSE_DB) {
		/* We stopped due to a watchpoint. We can't just resume executing the instruction again because
		 * that would trigger the watchpoint again. To fix this, we remove watchpoints,single-step and
		 * re-enable the watchpoint. */
		LOG_TARGET_DEBUG(
			target,
			"Single-stepping to get past instruction that triggered the watchpoint...");
		xtensa_reg_set(target, XT_REG_IDX_DEBUGCAUSE, 0);	/*so we don't recurse into
									 * the same routine */
		xtensa->core_cache->reg_list[XT_REG_IDX_DEBUGCAUSE].dirty = false;
		/*Save all DBREAKCx registers and set to 0 to disable watchpoints */
		for (unsigned int slot = 0; slot < xtensa->core_config->debug.dbreaks_num; slot++) {
			dbreakc[slot] = xtensa_reg_get(target, XT_REG_IDX_DBREAKC0 + slot);
			xtensa_reg_set(target, XT_REG_IDX_DBREAKC0 + slot, 0);
		}
	}

	if (!handle_breakpoints && (cause & (DEBUGCAUSE_BI | DEBUGCAUSE_BN))) {
		/* handle normal SW breakpoint */
		xtensa_reg_set(target, XT_REG_IDX_DEBUGCAUSE, 0);	/*so we don't recurse into
									 * the same routine */
		xtensa->core_cache->reg_list[XT_REG_IDX_DEBUGCAUSE].dirty = false;
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
		target->debug_reason = DBG_REASON_SINGLESTEP;
		target->state = TARGET_HALTED;

		xtensa_fetch_all_regs(target);

		cur_pc = xtensa_reg_get(target, XT_REG_IDX_PC);

		LOG_TARGET_DEBUG(target,
			"cur_ps=%" PRIx32 ", cur_pc=%" PRIx32 " dbg_cause=%" PRIx32 " exc_cause=%" PRIx32,
			xtensa_reg_get(target, XT_REG_IDX_PS),
			cur_pc,
			xtensa_reg_get(target, XT_REG_IDX_DEBUGCAUSE),
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
	LOG_DEBUG("Done stepping, PC=%" PRIX32, cur_pc);

	if (cause & DEBUGCAUSE_DB) {
		LOG_TARGET_DEBUG(target, "...Done, re-installing watchpoints.");
		/* Restore the DBREAKCx registers */
		for (unsigned int slot = 0; slot < xtensa->core_config->debug.dbreaks_num; slot++)
			xtensa_reg_set(target, XT_REG_IDX_DBREAKC0 + slot, dbreakc[slot]);
	}

	/* Restore int level */
	/* TODO: Theoretically, this can mess up stepping over an instruction that modifies
	 * ps.intlevel by itself. TODO: Look into this. */
	if (xtensa->stepping_isr_mode == XT_STEPPING_ISR_OFF) {
		newps = xtensa_reg_get(target, XT_REG_IDX_PS);
		newps = (newps & ~0xF) | (oldps & 0xf);
		xtensa_reg_set(target, XT_REG_IDX_PS, newps);
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
 * Check if the address gets to memory regions, and it's access mode
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
	xtensa_queue_dbg_reg_write(xtensa, NARADR_DDR, addrstart_al);
	xtensa_queue_exec_ins(xtensa, XT_INS_RSR(XT_SR_DDR, XT_REG_A3));
	/* Now we can safely read data from addrstart_al up to addrend_al into albuff */
	for (unsigned int i = 0; adr != addrend_al; i += sizeof(uint32_t), adr += sizeof(uint32_t)) {
		xtensa_queue_exec_ins(xtensa, XT_INS_LDDR32P(XT_REG_A3));
		xtensa_queue_dbg_reg_read(xtensa, NARADR_DDR, &albuff[i]);
	}
	int res = jtag_execute_queue();
	if (res == ERROR_OK)
		res = xtensa_core_status_check(target);
	if (res != ERROR_OK)
		LOG_TARGET_WARNING(target, "Failed reading %d bytes at address " TARGET_ADDR_FMT,
			count * size, address);

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
		/* We discard the const here because albuff can also be non-const */
		albuff = (uint8_t *)buffer;
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

	/* If we're using a temp aligned buffer, we need to fill the head and/or tail bit of it. */
	if (albuff != buffer) {
		/* See if we need to read the first and/or last word. */
		if (address & 3) {
			xtensa_queue_dbg_reg_write(xtensa, NARADR_DDR, addrstart_al);
			xtensa_queue_exec_ins(xtensa, XT_INS_RSR(XT_SR_DDR, XT_REG_A3));
			xtensa_queue_exec_ins(xtensa, XT_INS_LDDR32P(XT_REG_A3));
			xtensa_queue_dbg_reg_read(xtensa, NARADR_DDR, &albuff[0]);
		}
		if ((address + (size * count)) & 3) {
			xtensa_queue_dbg_reg_write(xtensa, NARADR_DDR, addrend_al - 4);
			xtensa_queue_exec_ins(xtensa, XT_INS_RSR(XT_SR_DDR, XT_REG_A3));
			xtensa_queue_exec_ins(xtensa, XT_INS_LDDR32P(XT_REG_A3));
			xtensa_queue_dbg_reg_read(xtensa, NARADR_DDR,
				&albuff[addrend_al - addrstart_al - 4]);
		}
		/* Grab bytes */
		res = jtag_execute_queue();
		if (res != ERROR_OK) {
			LOG_ERROR("Error issuing unaligned memory write context instruction(s): %d", res);
			if (albuff != buffer)
				free(albuff);
			return res;
		}
		xtensa_core_status_check(target);
		/* Copy data to be written into the aligned buffer */
		memcpy(&albuff[address & 3], buffer, size * count);
		/* Now we can write albuff in aligned uint32s. */
	}

	/* Write start address to A3 */
	xtensa_queue_dbg_reg_write(xtensa, NARADR_DDR, addrstart_al);
	xtensa_queue_exec_ins(xtensa, XT_INS_RSR(XT_SR_DDR, XT_REG_A3));
	/* Write the aligned buffer */
	for (unsigned int i = 0; adr != addrend_al; i += sizeof(uint32_t), adr += sizeof(uint32_t)) {
		xtensa_queue_dbg_reg_write(xtensa, NARADR_DDR, buf_get_u32(&albuff[i], 0, 32));
		xtensa_queue_exec_ins(xtensa, XT_INS_SDDR32P(XT_REG_A3));
	}
	res = jtag_execute_queue();
	if (res == ERROR_OK)
		res = xtensa_core_status_check(target);
	if (res != ERROR_OK)
		LOG_TARGET_WARNING(target, "Failed writing %d bytes at address " TARGET_ADDR_FMT, count * size, address);
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

	int res = xtensa_dm_power_status_read(&xtensa->dbg_mod, PWRSTAT_DEBUGWASRESET | PWRSTAT_COREWASRESET);
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

	res = xtensa_dm_core_status_read(&xtensa->dbg_mod);
	if (res != ERROR_OK)
		return res;
	if (xtensa->dbg_mod.power_status.stath & PWRSTAT_COREWASRESET) {
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
			xtensa_reg_val_t halt_cause = xtensa_reg_get(target, XT_REG_IDX_DEBUGCAUSE);
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
			LOG_TARGET_DEBUG(target, "Target halted, pc=0x%08" PRIX32 ", debug_reason=%08x, oldstate=%08x",
				xtensa_reg_get(target, XT_REG_IDX_PC),
				target->debug_reason,
				oldstate);
			LOG_TARGET_DEBUG(target, "Halt reason=0x%08" PRIX32 ", exc_cause=%" PRId32 ", dsr=0x%08" PRIx32,
				halt_cause,
				xtensa_reg_get(target, XT_REG_IDX_EXCCAUSE),
				xtensa->dbg_mod.core_status.dsr);
			LOG_TARGET_INFO(target, "Target halted, PC=0x%08" PRIX32 ", debug_reason=%08x",
				xtensa_reg_get(target, XT_REG_IDX_PC), target->debug_reason);
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

static int xtensa_sw_breakpoint_add(struct target *target,
	struct breakpoint *breakpoint,
	struct xtensa_sw_breakpoint *sw_bp)
{
	int ret = target_read_buffer(target, breakpoint->address, XT_ISNS_SZ_MAX, sw_bp->insn);
	if (ret != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to read original instruction (%d)!", ret);
		return ret;
	}

	sw_bp->insn_sz = xtensa_insn_size_get(buf_get_u32(sw_bp->insn, 0, 24));
	sw_bp->oocd_bp = breakpoint;

	uint32_t break_insn = sw_bp->insn_sz == XT_ISNS_SZ_MAX ? XT_INS_BREAK(0, 0) : XT_INS_BREAKN(0);
	/* convert to target endianness */
	uint8_t break_insn_buff[4];
	target_buffer_set_u32(target, break_insn_buff, break_insn);

	ret = target_write_buffer(target, breakpoint->address, sw_bp->insn_sz, break_insn_buff);
	if (ret != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to write breakpoint instruction (%d)!", ret);
		return ret;
	}

	return ERROR_OK;
}

static int xtensa_sw_breakpoint_remove(struct target *target, struct xtensa_sw_breakpoint *sw_bp)
{
	int ret = target_write_buffer(target, sw_bp->oocd_bp->address, sw_bp->insn_sz, sw_bp->insn);
	if (ret != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to read insn (%d)!", ret);
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
	LOG_TARGET_DEBUG(target, "placed HW breakpoint @ " TARGET_ADDR_FMT,
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
	struct reg_cache *reg_cache = calloc(1, sizeof(struct reg_cache));

	if (!reg_cache) {
		LOG_ERROR("Failed to alloc reg cache!");
		return ERROR_FAIL;
	}
	reg_cache->name = "Xtensa registers";
	reg_cache->next = NULL;
	reg_cache->num_regs = XT_NUM_REGS + xtensa->core_config->user_regs_num;
	/* Init reglist */
	struct reg *reg_list = calloc(reg_cache->num_regs, sizeof(struct reg));
	if (!reg_list) {
		LOG_ERROR("Failed to alloc reg list!");
		goto fail;
	}
	xtensa->regs_num = 0;

	for (unsigned int i = 0; i < XT_NUM_REGS; i++) {
		reg_list[i].exist = false;
		if (xtensa_regs[i].type == XT_REG_USER) {
			if (xtensa_user_reg_exists(xtensa, i))
				reg_list[i].exist = true;
			else
				LOG_DEBUG("User reg '%s' (%d) does not exist", xtensa_regs[i].name, i);
		} else if (xtensa_regs[i].type == XT_REG_FR) {
			if (xtensa_fp_reg_exists(xtensa, i))
				reg_list[i].exist = true;
			else
				LOG_DEBUG("FP reg '%s' (%d) does not exist", xtensa_regs[i].name, i);
		} else if (xtensa_regs[i].type == XT_REG_SPECIAL) {
			if (xtensa_special_reg_exists(xtensa, i))
				reg_list[i].exist = true;
			else
				LOG_DEBUG("Special reg '%s' (%d) does not exist", xtensa_regs[i].name, i);
		} else {
			if (xtensa_regular_reg_exists(xtensa, i))
				reg_list[i].exist = true;
			else
				LOG_DEBUG("Regular reg '%s' (%d) does not exist", xtensa_regs[i].name, i);
		}
		reg_list[i].name = xtensa_regs[i].name;
		reg_list[i].size = 32;
		reg_list[i].value = calloc(1, 4	/*XT_REG_LEN*/);/* make Clang Static Analyzer happy */
		if (!reg_list[i].value) {
			LOG_ERROR("Failed to alloc reg list value!");
			goto fail;
		}
		reg_list[i].dirty = false;
		reg_list[i].valid = false;
		reg_list[i].type = &xtensa_reg_type;
		reg_list[i].arch_info = xtensa;
		if (reg_list[i].exist)
			xtensa->regs_num++;
	}
	for (unsigned int i = 0; i < xtensa->core_config->user_regs_num; i++) {
		reg_list[XT_USR_REG_START + i].exist = true;
		reg_list[XT_USR_REG_START + i].name = xtensa->core_config->user_regs[i].name;
		reg_list[XT_USR_REG_START + i].size = xtensa->core_config->user_regs[i].size;
		reg_list[XT_USR_REG_START + i].value = calloc(1, reg_list[XT_USR_REG_START + i].size / 8);
		if (!reg_list[XT_USR_REG_START + i].value) {
			LOG_ERROR("Failed to alloc user reg list value!");
			goto fail;
		}
		reg_list[XT_USR_REG_START + i].dirty = false;
		reg_list[XT_USR_REG_START + i].valid = false;
		reg_list[XT_USR_REG_START + i].type = xtensa->core_config->user_regs[i].type;
		reg_list[XT_USR_REG_START + i].arch_info = xtensa;
		xtensa->regs_num++;
	}
	if (xtensa->core_config->gdb_general_regs_num >= xtensa->regs_num) {
		LOG_ERROR("Regs number less then GDB general regs number!");
		goto fail;
	}

	/* assign GDB reg numbers to registers */
	for (unsigned int gdb_reg_id = 0; gdb_reg_id < xtensa->regs_num; gdb_reg_id++) {
		unsigned int reg_id = xtensa->core_config->gdb_regs_mapping[gdb_reg_id];
		if (reg_id >= reg_cache->num_regs) {
			LOG_ERROR("Invalid GDB map!");
			goto fail;
		}
		if (!reg_list[reg_id].exist) {
			LOG_ERROR("Non-existing reg in GDB map!");
			goto fail;
		}
		reg_list[reg_id].number = gdb_reg_id;
	}
	reg_cache->reg_list = reg_list;

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
		for (unsigned int i = 0; i < reg_cache->num_regs; i++)
			free(reg_list[i].value);
		free(reg_list);
	}
	if (xtensa->algo_context_backup) {
		for (unsigned int i = 0; i < reg_cache->num_regs; i++)
			free(xtensa->algo_context_backup[i]);
		free(xtensa->algo_context_backup);
	}
	free(reg_cache);

	return ERROR_FAIL;
}

int xtensa_init_arch_info(struct target *target, struct xtensa *xtensa,
	const struct xtensa_config *xtensa_config,
	const struct xtensa_debug_module_config *dm_cfg)
{
	target->arch_info = xtensa;
	xtensa->common_magic = XTENSA_COMMON_MAGIC;
	xtensa->target = target;
	xtensa->core_config = xtensa_config;
	xtensa->stepping_isr_mode = XT_STEPPING_ISR_ON;

	if (!xtensa->core_config->exc.enabled || !xtensa->core_config->irq.enabled ||
		!xtensa->core_config->high_irq.enabled || !xtensa->core_config->debug.enabled) {
		LOG_ERROR("Xtensa configuration does not support debugging!");
		return ERROR_FAIL;
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
	xtensa->hw_brps = calloc(xtensa->core_config->debug.ibreaks_num, sizeof(struct breakpoint *));
	if (!xtensa->hw_brps) {
		LOG_ERROR("Failed to alloc memory for HW breakpoints!");
		return ERROR_FAIL;
	}
	xtensa->hw_wps = calloc(xtensa->core_config->debug.dbreaks_num, sizeof(struct watchpoint *));
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
}

void xtensa_target_deinit(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);

	LOG_DEBUG("start");

	if (target_was_examined(target)) {
		int ret = xtensa_queue_dbg_reg_write(xtensa, NARADR_DCRCLR, OCDDCR_ENABLEOCD);
		if (ret != ERROR_OK) {
			LOG_ERROR("Failed to queue OCDDCR_ENABLEOCD clear operation!");
			return;
		}
		xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
		ret = jtag_execute_queue();
		if (ret != ERROR_OK) {
			LOG_ERROR("Failed to clear OCDDCR_ENABLEOCD!");
			return;
		}
	}
	xtensa_free_reg_cache(target);
	free(xtensa->hw_brps);
	free(xtensa->hw_wps);
	free(xtensa->sw_brps);
}

const char *xtensa_get_gdb_arch(struct target *target)
{
	return "xtensa";
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
	int res = ERROR_OK;
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
		if (res == ERROR_OK) {
			command_print(CMD, "Current bits set:%s%s%s%s",
				(val & OCDDCR_BREAKINEN) ? " BreakIn" : "",
				(val & OCDDCR_BREAKOUTEN) ? " BreakOut" : "",
				(val & OCDDCR_RUNSTALLINEN) ? " RunStallIn" : "",
				(val & OCDDCR_DEBUGMODEOUTEN) ? " DebugModeOut" : ""
				);
		} else {
			command_print(CMD, "Failed to get smpbreak config %d", res);
		}
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

const struct command_registration xtensa_command_handlers[] = {
	{
		.name = "set_permissive",
		.handler = xtensa_cmd_permissive_mode,
		.mode = COMMAND_ANY,
		.help = "When set to 1, enable Xtensa permissive mode (less client-side checks)",
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
		.usage =
			"[none|breakinout|runstall] | [BreakIn] [BreakOut] [RunStallIn] [DebugModeOut]",
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
		.help =
			"Dump performance counter value. If no argument specified, dumps all counters.",
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
	COMMAND_REGISTRATION_DONE
};
