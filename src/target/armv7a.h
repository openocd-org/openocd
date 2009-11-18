/***************************************************************************
 *    Copyright (C) 2009 by David Brownell                                 *
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
#ifndef ARMV7A_H
#define ARMV7A_H

#include "arm_adi_v5.h"
#include "armv4_5.h"
#include "armv4_5_mmu.h"
#include "armv4_5_cache.h"

typedef enum armv7a_mode
{
	ARMV7A_MODE_USR = 16,
	ARMV7A_MODE_FIQ = 17,
	ARMV7A_MODE_IRQ = 18,
	ARMV7A_MODE_SVC = 19,
	ARMV7A_MODE_ABT = 23,
	ARMV7A_MODE_UND = 27,
	ARMV7A_MODE_SYS = 31,
	ARMV7A_MODE_MON = 22,
	ARMV7A_MODE_ANY = -1
} armv7a_t;

typedef enum armv7a_state
{
	ARMV7A_STATE_ARM,
	ARMV7A_STATE_THUMB,
	ARMV7A_STATE_JAZELLE,
	ARMV7A_STATE_THUMBEE,
} armv7a_state_t;

enum
{
	ARM_PC  = 15,
	ARM_CPSR = 16
}
;
/* offsets into armv4_5 core register cache */
enum
{
	ARMV7A_CPSR = 31,
	ARMV7A_SPSR_FIQ = 32,
	ARMV7A_SPSR_IRQ = 33,
	ARMV7A_SPSR_SVC = 34,
	ARMV7A_SPSR_ABT = 35,
	ARMV7A_SPSR_UND = 36
};

#define ARMV7_COMMON_MAGIC 0x0A450999

/* VA to PA translation operations opc2 values*/
#define V2PCWPR  0
#define V2PCWPW  1
#define V2PCWUR  2
#define V2PCWUW  3
#define V2POWPR  4
#define V2POWPW  5
#define V2POWUR  6
#define V2POWUW  7

struct armv7a_common
{
	int common_magic;
	struct reg_cache *core_cache;
	enum armv7a_mode core_mode;
	enum armv7a_state core_state;

	/* arm adp debug port */
	struct swjdp_common swjdp_info;

	/* Core Debug Unit */
	uint32_t debug_base;
	uint8_t debug_ap;
	uint8_t memory_ap;

	/* Cache and Memory Management Unit */
	struct armv4_5_mmu_common armv4_5_mmu;
	struct arm armv4_5_common;

	int (*read_cp15)(struct target *target,
			uint32_t op1, uint32_t op2,
			uint32_t CRn, uint32_t CRm, uint32_t *value);
	int (*write_cp15)(struct target *target,
			uint32_t op1, uint32_t op2,
			uint32_t CRn, uint32_t CRm, uint32_t value);

	int (*examine_debug_reason)(struct target *target);
	void (*post_debug_entry)(struct target *target);

	void (*pre_restore_context)(struct target *target);
	void (*post_restore_context)(struct target *target);

};

static inline struct armv7a_common *
target_to_armv7a(struct target *target)
{
	return container_of(target->arch_info, struct armv7a_common,
			armv4_5_common);
}

struct armv7a_algorithm
{
	int common_magic;

	enum armv7a_mode core_mode;
	enum armv7a_state core_state;
};

struct armv7a_core_reg
{
	int num;
	enum armv7a_mode mode;
	struct target *target;
	struct armv7a_common *armv7a_common;
};

int armv7a_arch_state(struct target *target);
struct reg_cache *armv7a_build_reg_cache(struct target *target,
		struct armv7a_common *armv7a_common);
int armv7a_register_commands(struct command_context *cmd_ctx);
int armv7a_init_arch_info(struct target *target, struct armv7a_common *armv7a);

#endif /* ARMV4_5_H */
