/***************************************************************************
 *   Copyright (C) 2011 by Julius Baxter                                   *
 *   julius@opencores.org                                                  *
 *                                                                         *
 *   Copyright (C) 2013 by Marek Czerski                                   *
 *   ma.czerski@gmail.com                                                  *
 *                                                                         *
 *   Copyright (C) 2013 by Franck Jullien                                  *
 *   elec4fun@gmail.com                                                    *
 *                                                                         *
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

#ifndef OPENOCD_TARGET_OPENRISC_OR1K_H
#define OPENOCD_TARGET_OPENRISC_OR1K_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <target/target.h>

/* SPR groups start address */
#define GROUP0		(0  << 11)
#define GROUP1		(1  << 11)
#define GROUP2		(2  << 11)
#define GROUP3		(3  << 11)
#define GROUP4		(4  << 11)
#define GROUP5		(5  << 11)
#define GROUP6		(6  << 11)
#define GROUP7		(7  << 11)
#define GROUP8		(8  << 11)
#define GROUP9		(9  << 11)
#define GROUP10		(10 << 11)

/* OR1K registers */
enum or1k_reg_nums {
	OR1K_REG_R0 = 0,
	OR1K_REG_R1,
	OR1K_REG_R2,
	OR1K_REG_R3,
	OR1K_REG_R4,
	OR1K_REG_R5,
	OR1K_REG_R6,
	OR1K_REG_R7,
	OR1K_REG_R8,
	OR1K_REG_R9,
	OR1K_REG_R10,
	OR1K_REG_R11,
	OR1K_REG_R12,
	OR1K_REG_R13,
	OR1K_REG_R14,
	OR1K_REG_R15,
	OR1K_REG_R16,
	OR1K_REG_R17,
	OR1K_REG_R18,
	OR1K_REG_R19,
	OR1K_REG_R20,
	OR1K_REG_R21,
	OR1K_REG_R22,
	OR1K_REG_R23,
	OR1K_REG_R24,
	OR1K_REG_R25,
	OR1K_REG_R26,
	OR1K_REG_R27,
	OR1K_REG_R28,
	OR1K_REG_R29,
	OR1K_REG_R30,
	OR1K_REG_R31,
	OR1K_REG_PPC,
	OR1K_REG_NPC,
	OR1K_REG_SR,
	OR1KNUMCOREREGS
};

struct or1k_jtag {
	struct jtag_tap *tap;
	int or1k_jtag_inited;
	int or1k_jtag_module_selected;
	uint8_t *current_reg_idx;
	struct or1k_tap_ip *tap_ip;
	struct or1k_du *du_core;
	struct target *target;
};

struct or1k_common {
	struct or1k_jtag jtag;
	struct reg_cache *core_cache;
	uint32_t core_regs[OR1KNUMCOREREGS];
	int nb_regs;
	struct or1k_core_reg *arch_info;
};

static inline struct or1k_common *
target_to_or1k(struct target *target)
{
	return (struct or1k_common *)target->arch_info;
}

struct or1k_core_reg {
	const char *name;
	uint32_t list_num;   /* Index in register cache */
	uint32_t spr_num;    /* Number in architecture's SPR space */
	struct target *target;
	struct or1k_common *or1k_common;
	const char *feature; /* feature name in XML tdesc file */
	const char *group;   /* register group in XML tdesc file */
};

struct or1k_core_reg_init {
	const char *name;
	uint32_t spr_num;    /* Number in architecture's SPR space */
	const char *feature; /* feature name in XML tdesc file */
	const char *group;   /* register group in XML tdesc file */
};

/* ORBIS32 Trap instruction */
#define OR1K_TRAP_INSTR  0x21000001

enum or1k_debug_reg_nums {
	OR1K_DEBUG_REG_DMR1 = 0,
	OR1K_DEBUG_REG_DMR2,
	OR1K_DEBUG_REG_DCWR0,
	OR1K_DEBUG_REG_DCWR1,
	OR1K_DEBUG_REG_DSR,
	OR1K_DEBUG_REG_DRR,
	OR1K_DEBUG_REG_NUM
};

#define NO_SINGLE_STEP		0
#define SINGLE_STEP		1

/* OR1K Debug registers and bits needed for resuming */
#define OR1K_DEBUG_REG_BASE	GROUP6                     /* Debug registers Base address */
#define OR1K_DMR1_CPU_REG_ADD	(OR1K_DEBUG_REG_BASE + 16) /* Debug Mode Register 1 0x3010 */
#define OR1K_DMR1_ST		0x00400000                 /* Single-step trace */
#define OR1K_DMR1_BT		0x00800000                 /* Branch trace */
#define OR1K_DMR2_WGB		0x003ff000                 /* Watchpoints generating breakpoint */
#define OR1K_DSR_TE		0x00002000                 /* Trap exception */

/* OR1K Instruction cache registers needed for invalidating instruction
 * memory during adding and removing breakpoints.
 */
#define OR1K_ICBIR_CPU_REG_ADD ((4 << 11) + 2)             /* IC Block Invalidate Register 0x2002 */

#endif /* OPENOCD_TARGET_OPENRISC_OR1K_H */
