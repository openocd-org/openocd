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

#include "register.h"
#include "target.h"
#include "log.h"
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

extern char **armv7a_mode_strings;

typedef enum armv7a_state
{
	ARMV7A_STATE_ARM,
	ARMV7A_STATE_THUMB,
	ARMV7A_STATE_JAZELLE,
	ARMV7A_STATE_THUMBEE,
} armv7a_state_t;

extern char *armv7a_state_strings[];

extern int armv7a_core_reg_map[8][17];

#define ARMV7A_CORE_REG_MODE(cache, mode, num) \
		cache->reg_list[armv7a_core_reg_map[armv7a_mode_to_number(mode)][num]]
#define ARMV7A_CORE_REG_MODENUM(cache, mode, num) \
		cache->reg_list[armv7a_core_reg_map[mode][num]]

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

#define ARMV4_5_COMMON_MAGIC 0x0A450A45
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

typedef struct armv7a_common_s
{
	int common_magic;
	reg_cache_t *core_cache;
	enum armv7a_mode core_mode;
	enum armv7a_state core_state;

	/* arm adp debug port */
	swjdp_common_t swjdp_info;

	/* Core Debug Unit */
	uint32_t debug_base;
	uint8_t debug_ap;
	uint8_t memory_ap;

	/* Cache and Memory Management Unit */
	armv4_5_mmu_common_t armv4_5_mmu;
	armv4_5_common_t armv4_5_common;
	void *arch_info;

//	int (*full_context)(struct target_s *target);
//	int (*read_core_reg)(struct target_s *target, int num, enum armv7a_mode mode);
//	int (*write_core_reg)(struct target_s *target, int num, enum armv7a_mode mode, u32 value);
	int (*read_cp15)(struct target_s *target,
			uint32_t op1, uint32_t op2,
			uint32_t CRn, uint32_t CRm, uint32_t *value);
	int (*write_cp15)(struct target_s *target,
			uint32_t op1, uint32_t op2,
			uint32_t CRn, uint32_t CRm, uint32_t value);

	int (*examine_debug_reason)(target_t *target);
	void (*post_debug_entry)(target_t *target);

	void (*pre_restore_context)(target_t *target);
	void (*post_restore_context)(target_t *target);

} armv7a_common_t;

typedef struct armv7a_algorithm_s
{
	int common_magic;

	enum armv7a_mode core_mode;
	enum armv7a_state core_state;
} armv7a_algorithm_t;

typedef struct armv7a_core_reg_s
{
	int num;
	enum armv7a_mode mode;
	target_t *target;
	armv7a_common_t *armv7a_common;
} armv7a_core_reg_t;

int armv7a_arch_state(struct target_s *target);
reg_cache_t *armv7a_build_reg_cache(target_t *target,
		armv7a_common_t *armv7a_common);
int armv7a_register_commands(struct command_context_s *cmd_ctx);
int armv7a_init_arch_info(target_t *target, armv7a_common_t *armv7a);

/* map psr mode bits to linear number */
static inline int armv7a_mode_to_number(enum armv7a_mode mode)
{
	switch (mode)
	{
		case ARMV7A_MODE_USR: return 0; break;
		case ARMV7A_MODE_FIQ: return 1; break;
		case ARMV7A_MODE_IRQ: return 2; break;
		case ARMV7A_MODE_SVC: return 3; break;
		case ARMV7A_MODE_ABT: return 4; break;
		case ARMV7A_MODE_UND: return 5; break;
		case ARMV7A_MODE_SYS: return 6; break;
		case ARMV7A_MODE_MON: return 7; break;
		case ARMV7A_MODE_ANY: return 0; break;	/* map MODE_ANY to user mode */
		default:
			LOG_ERROR("invalid mode value encountered, val %d", mode);
			return -1;
	}
}

/* map linear number to mode bits */
static inline enum armv7a_mode armv7a_number_to_mode(int number)
{
	switch(number)
	{
		case 0: return ARMV7A_MODE_USR; break;
		case 1: return ARMV7A_MODE_FIQ; break;
		case 2: return ARMV7A_MODE_IRQ; break;
		case 3: return ARMV7A_MODE_SVC; break;
		case 4: return ARMV7A_MODE_ABT; break;
		case 5: return ARMV7A_MODE_UND; break;
		case 6: return ARMV7A_MODE_SYS; break;
		case 7: return ARMV7A_MODE_MON; break;
		default:
			LOG_ERROR("mode index out of bounds");
			return ARMV7A_MODE_ANY;
	}
};


#endif /* ARMV4_5_H */
