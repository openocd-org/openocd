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
#ifndef ARMV7M_COMMON_H
#define ARMV7M_COMMON_H

#include "arm_adi_v5.h"
#include "arm.h"

/* define for enabling armv7 gdb workarounds */
#if 1
#define ARMV7_GDB_HACKS
#endif

#ifdef ARMV7_GDB_HACKS
extern uint8_t armv7m_gdb_dummy_cpsr_value[];
extern struct reg armv7m_gdb_dummy_cpsr_reg;
#endif


enum armv7m_mode
{
	ARMV7M_MODE_THREAD = 0,
	ARMV7M_MODE_USER_THREAD = 1,
	ARMV7M_MODE_HANDLER = 2,
	ARMV7M_MODE_ANY = -1
};

extern char *armv7m_mode_strings[];

enum armv7m_regtype
{
	ARMV7M_REGISTER_CORE_GP,
	ARMV7M_REGISTER_CORE_SP,
	ARMV7M_REGISTER_MEMMAP
};

char *armv7m_exception_string(int number);

/* offsets into armv7m core register cache */
enum
{
	/* for convenience, the first set of indices match
	 * the Cortex-M3 DCRSR selectors
	 */
	ARMV7M_R0,
	ARMV7M_R1,
	ARMV7M_R2,
	ARMV7M_R3,

	ARMV7M_R4,
	ARMV7M_R5,
	ARMV7M_R6,
	ARMV7M_R7,

	ARMV7M_R8,
	ARMV7M_R9,
	ARMV7M_R10,
	ARMV7M_R11,

	ARMV7M_R12,
	ARMV7M_R13,
	ARMV7M_R14,
	ARMV7M_PC = 15,

	ARMV7M_xPSR = 16,
	ARMV7M_MSP,
	ARMV7M_PSP,

	/* this next set of indices is arbitrary */
	ARMV7M_PRIMASK,
	ARMV7M_BASEPRI,
	ARMV7M_FAULTMASK,
	ARMV7M_CONTROL,
};

#define ARMV7M_COMMON_MAGIC 0x2A452A45

struct armv7m_common
{
	struct arm	arm;

	int common_magic;
	struct reg_cache *core_cache;
	enum armv7m_mode core_mode;
	int exception_number;
	struct adiv5_dap dap;

	uint32_t demcr;

	/* Direct processor core register read and writes */
	int (*load_core_reg_u32)(struct target *target,
		enum armv7m_regtype type, uint32_t num, uint32_t *value);
	int (*store_core_reg_u32)(struct target *target,
		enum armv7m_regtype type, uint32_t num, uint32_t value);

	/* register cache to processor synchronization */
	int (*read_core_reg)(struct target *target, unsigned num);
	int (*write_core_reg)(struct target *target, unsigned num);

	int (*examine_debug_reason)(struct target *target);
	int (*post_debug_entry)(struct target *target);

	void (*pre_restore_context)(struct target *target);
};

static inline struct armv7m_common *
target_to_armv7m(struct target *target)
{
	return container_of(target->arch_info, struct armv7m_common, arm);
}

static inline bool is_armv7m(struct armv7m_common *armv7m)
{
	return armv7m->common_magic == ARMV7M_COMMON_MAGIC;
}

struct armv7m_algorithm
{
	int common_magic;

	enum armv7m_mode core_mode;
};

struct armv7m_core_reg
{
	uint32_t num;
	enum armv7m_regtype type;
	struct target *target;
	struct armv7m_common *armv7m_common;
};

struct reg_cache *armv7m_build_reg_cache(struct target *target);
enum armv7m_mode armv7m_number_to_mode(int number);
int armv7m_mode_to_number(enum armv7m_mode mode);

int armv7m_arch_state(struct target *target);
int armv7m_get_gdb_reg_list(struct target *target,
		struct reg **reg_list[], int *reg_list_size);

int armv7m_init_arch_info(struct target *target, struct armv7m_common *armv7m);

int armv7m_run_algorithm(struct target *target,
		int num_mem_params, struct mem_param *mem_params,
		int num_reg_params, struct reg_param *reg_params,
		uint32_t entry_point, uint32_t exit_point,
		int timeout_ms, void *arch_info);

int armv7m_invalidate_core_regs(struct target *target);

int armv7m_restore_context(struct target *target);

int armv7m_checksum_memory(struct target *target,
		uint32_t address, uint32_t count, uint32_t* checksum);
int armv7m_blank_check_memory(struct target *target,
		uint32_t address, uint32_t count, uint32_t* blank);

int armv7m_maybe_skip_bkpt_inst(struct target *target, bool *inst_found);

extern const struct command_registration armv7m_command_handlers[];

#endif /* ARMV7M_H */
