/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ARMV7M_H
#define OPENOCD_TARGET_ARMV7M_H

#include "arm.h"
#include "armv7m_trace.h"

struct adiv5_ap;

extern const int armv7m_psp_reg_map[];
extern const int armv7m_msp_reg_map[];

const char *armv7m_exception_string(int number);

/* Cortex-M DCRSR.REGSEL selectors */
enum {
	ARMV7M_REGSEL_R0,
	ARMV7M_REGSEL_R1,
	ARMV7M_REGSEL_R2,
	ARMV7M_REGSEL_R3,

	ARMV7M_REGSEL_R4,
	ARMV7M_REGSEL_R5,
	ARMV7M_REGSEL_R6,
	ARMV7M_REGSEL_R7,

	ARMV7M_REGSEL_R8,
	ARMV7M_REGSEL_R9,
	ARMV7M_REGSEL_R10,
	ARMV7M_REGSEL_R11,

	ARMV7M_REGSEL_R12,
	ARMV7M_REGSEL_R13,
	ARMV7M_REGSEL_R14,
	ARMV7M_REGSEL_PC = 15,

	ARMV7M_REGSEL_XPSR = 16,
	ARMV7M_REGSEL_MSP,
	ARMV7M_REGSEL_PSP,

	ARMV8M_REGSEL_MSP_NS = 0x18,
	ARMV8M_REGSEL_PSP_NS,
	ARMV8M_REGSEL_MSP_S,
	ARMV8M_REGSEL_PSP_S,
	ARMV8M_REGSEL_MSPLIM_S,
	ARMV8M_REGSEL_PSPLIM_S,
	ARMV8M_REGSEL_MSPLIM_NS,
	ARMV8M_REGSEL_PSPLIM_NS,

	ARMV7M_REGSEL_PMSK_BPRI_FLTMSK_CTRL = 0x14,
	ARMV8M_REGSEL_PMSK_BPRI_FLTMSK_CTRL_S = 0x22,
	ARMV8M_REGSEL_PMSK_BPRI_FLTMSK_CTRL_NS = 0x23,
	ARMV8M_REGSEL_VPR = 0x24,
	ARMV7M_REGSEL_FPSCR = 0x21,

	/* 32bit Floating-point registers */
	ARMV7M_REGSEL_S0 = 0x40,
	ARMV7M_REGSEL_S1,
	ARMV7M_REGSEL_S2,
	ARMV7M_REGSEL_S3,
	ARMV7M_REGSEL_S4,
	ARMV7M_REGSEL_S5,
	ARMV7M_REGSEL_S6,
	ARMV7M_REGSEL_S7,
	ARMV7M_REGSEL_S8,
	ARMV7M_REGSEL_S9,
	ARMV7M_REGSEL_S10,
	ARMV7M_REGSEL_S11,
	ARMV7M_REGSEL_S12,
	ARMV7M_REGSEL_S13,
	ARMV7M_REGSEL_S14,
	ARMV7M_REGSEL_S15,
	ARMV7M_REGSEL_S16,
	ARMV7M_REGSEL_S17,
	ARMV7M_REGSEL_S18,
	ARMV7M_REGSEL_S19,
	ARMV7M_REGSEL_S20,
	ARMV7M_REGSEL_S21,
	ARMV7M_REGSEL_S22,
	ARMV7M_REGSEL_S23,
	ARMV7M_REGSEL_S24,
	ARMV7M_REGSEL_S25,
	ARMV7M_REGSEL_S26,
	ARMV7M_REGSEL_S27,
	ARMV7M_REGSEL_S28,
	ARMV7M_REGSEL_S29,
	ARMV7M_REGSEL_S30,
	ARMV7M_REGSEL_S31,
};

/* offsets into armv7m core register cache */
enum {
	/* for convenience, the first set of indices match
	 * the Cortex-M DCRSR.REGSEL selectors
	 */
	ARMV7M_R0 = ARMV7M_REGSEL_R0,
	ARMV7M_R1 = ARMV7M_REGSEL_R1,
	ARMV7M_R2 = ARMV7M_REGSEL_R2,
	ARMV7M_R3 = ARMV7M_REGSEL_R3,

	ARMV7M_R4 = ARMV7M_REGSEL_R4,
	ARMV7M_R5 = ARMV7M_REGSEL_R5,
	ARMV7M_R6 = ARMV7M_REGSEL_R6,
	ARMV7M_R7 = ARMV7M_REGSEL_R7,

	ARMV7M_R8 = ARMV7M_REGSEL_R8,
	ARMV7M_R9 = ARMV7M_REGSEL_R9,
	ARMV7M_R10 = ARMV7M_REGSEL_R10,
	ARMV7M_R11 = ARMV7M_REGSEL_R11,

	ARMV7M_R12 = ARMV7M_REGSEL_R12,
	ARMV7M_R13 = ARMV7M_REGSEL_R13,
	ARMV7M_R14 = ARMV7M_REGSEL_R14,
	ARMV7M_PC = ARMV7M_REGSEL_PC,

	ARMV7M_XPSR = ARMV7M_REGSEL_XPSR,
	ARMV7M_MSP = ARMV7M_REGSEL_MSP,
	ARMV7M_PSP = ARMV7M_REGSEL_PSP,

	/* following indices are arbitrary, do not match DCRSR.REGSEL selectors */

	/* A block of container and contained registers follows:
	 * THE ORDER IS IMPORTANT to the end of the block ! */
	/* working register for packing/unpacking special regs, hidden from gdb */
	ARMV7M_PMSK_BPRI_FLTMSK_CTRL,

	/* WARNING: If you use armv7m_write_core_reg() on one of 4 following
	 * special registers, the new data go to ARMV7M_PMSK_BPRI_FLTMSK_CTRL
	 * cache only and are not flushed to CPU HW register.
	 * To trigger write to CPU HW register, add
	 *		armv7m_write_core_reg(,,ARMV7M_PMSK_BPRI_FLTMSK_CTRL,);
	 */
	ARMV7M_PRIMASK,
	ARMV7M_BASEPRI,
	ARMV7M_FAULTMASK,
	ARMV7M_CONTROL,
	/* The end of block of container and contained registers */

	/* ARMv8-M specific registers */
	ARMV8M_MSP_NS,
	ARMV8M_PSP_NS,
	ARMV8M_MSP_S,
	ARMV8M_PSP_S,
	ARMV8M_MSPLIM_S,
	ARMV8M_PSPLIM_S,
	ARMV8M_MSPLIM_NS,
	ARMV8M_PSPLIM_NS,

	/* A block of container and contained registers follows:
	 * THE ORDER IS IMPORTANT to the end of the block ! */
	ARMV8M_PMSK_BPRI_FLTMSK_CTRL_S,
	ARMV8M_PRIMASK_S,
	ARMV8M_BASEPRI_S,
	ARMV8M_FAULTMASK_S,
	ARMV8M_CONTROL_S,
	/* The end of block of container and contained registers */

	/* A block of container and contained registers follows:
	 * THE ORDER IS IMPORTANT to the end of the block ! */
	ARMV8M_PMSK_BPRI_FLTMSK_CTRL_NS,
	ARMV8M_PRIMASK_NS,
	ARMV8M_BASEPRI_NS,
	ARMV8M_FAULTMASK_NS,
	ARMV8M_CONTROL_NS,
	/* The end of block of container and contained registers */

	/* 64bit Floating-point registers */
	ARMV7M_D0,
	ARMV7M_D1,
	ARMV7M_D2,
	ARMV7M_D3,
	ARMV7M_D4,
	ARMV7M_D5,
	ARMV7M_D6,
	ARMV7M_D7,
	ARMV7M_D8,
	ARMV7M_D9,
	ARMV7M_D10,
	ARMV7M_D11,
	ARMV7M_D12,
	ARMV7M_D13,
	ARMV7M_D14,
	ARMV7M_D15,

	/* Floating-point status register */
	ARMV7M_FPSCR,

	/* Vector Predication Status and Control Register */
	ARMV8M_VPR,

	/* for convenience add registers' block delimiters */
	ARMV7M_LAST_REG,
	ARMV7M_CORE_FIRST_REG = ARMV7M_R0,
	ARMV7M_CORE_LAST_REG = ARMV7M_XPSR,
	ARMV7M_FPU_FIRST_REG = ARMV7M_D0,
	ARMV7M_FPU_LAST_REG = ARMV8M_VPR,
	ARMV8M_FIRST_REG = ARMV8M_MSP_NS,
	ARMV8M_LAST_REG = ARMV8M_CONTROL_NS,
};

enum {
	FP_NONE = 0,
	FPV4_SP,
	FPV5_SP,
	FPV5_DP,
	FPV5_MVE_I,
	FPV5_MVE_F,
};

#define ARMV7M_NUM_CORE_REGS (ARMV7M_CORE_LAST_REG - ARMV7M_CORE_FIRST_REG + 1)

#define ARMV7M_COMMON_MAGIC 0x2A452A45U

struct armv7m_common {
	unsigned int common_magic;

	struct arm arm;

	int exception_number;

	/* AP this processor is connected to in the DAP */
	struct adiv5_ap *debug_ap;

	int fp_feature;
	uint32_t demcr;

	/* hla_target uses a high level adapter that does not support all functions */
	bool is_hla_target;

	struct armv7m_trace_config trace_config;

	/* Direct processor core register read and writes */
	int (*load_core_reg_u32)(struct target *target, uint32_t regsel, uint32_t *value);
	int (*store_core_reg_u32)(struct target *target, uint32_t regsel, uint32_t value);

	int (*examine_debug_reason)(struct target *target);
	int (*post_debug_entry)(struct target *target);

	void (*pre_restore_context)(struct target *target);
};

static inline bool is_armv7m(const struct armv7m_common *armv7m)
{
	return armv7m->common_magic == ARMV7M_COMMON_MAGIC;
}

/**
 * @returns the pointer to the target specific struct
 * without matching a magic number.
 * Use in target specific service routines, where the correct
 * type of arch_info is certain.
 */
static inline struct armv7m_common *
target_to_armv7m(struct target *target)
{
	return container_of(target->arch_info, struct armv7m_common, arm);
}

/**
 * @returns the pointer to the target specific struct
 * or NULL if the magic number does not match.
 * Use in a flash driver or any place where mismatch of the arch_info
 * type can happen.
 */
static inline struct armv7m_common *
target_to_armv7m_safe(struct target *target)
{
	if (!target)
		return NULL;

	if (!target->arch_info)
		return NULL;

	/* Check the parent type first to prevent peeking memory too far
	 * from arch_info pointer */
	if (!is_arm(target_to_arm(target)))
		return NULL;

	struct armv7m_common *armv7m = target_to_armv7m(target);
	if (!is_armv7m(armv7m))
		return NULL;

	return armv7m;
}

struct armv7m_algorithm {
	unsigned int common_magic;

	enum arm_mode core_mode;

	uint32_t context[ARMV7M_LAST_REG]; /* ARMV7M_NUM_REGS */
};

struct reg_cache *armv7m_build_reg_cache(struct target *target);
void armv7m_free_reg_cache(struct target *target);

enum armv7m_mode armv7m_number_to_mode(int number);
int armv7m_mode_to_number(enum armv7m_mode mode);

int armv7m_arch_state(struct target *target);
int armv7m_get_gdb_reg_list(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class);

int armv7m_init_arch_info(struct target *target, struct armv7m_common *armv7m);

int armv7m_run_algorithm(struct target *target,
		int num_mem_params, struct mem_param *mem_params,
		int num_reg_params, struct reg_param *reg_params,
		target_addr_t entry_point, target_addr_t exit_point,
		unsigned int timeout_ms, void *arch_info);

int armv7m_start_algorithm(struct target *target,
		int num_mem_params, struct mem_param *mem_params,
		int num_reg_params, struct reg_param *reg_params,
		target_addr_t entry_point, target_addr_t exit_point,
		void *arch_info);

int armv7m_wait_algorithm(struct target *target,
		int num_mem_params, struct mem_param *mem_params,
		int num_reg_params, struct reg_param *reg_params,
		target_addr_t exit_point, unsigned int timeout_ms,
		void *arch_info);

int armv7m_invalidate_core_regs(struct target *target);

int armv7m_restore_context(struct target *target);

uint32_t armv7m_map_id_to_regsel(unsigned int arm_reg_id);

bool armv7m_map_reg_packing(unsigned int arm_reg_id,
		unsigned int *reg32_id, uint32_t *offset);

int armv7m_checksum_memory(struct target *target,
		target_addr_t address, uint32_t count, uint32_t *checksum);
int armv7m_blank_check_memory(struct target *target,
		struct target_memory_check_block *blocks, int num_blocks, uint8_t erased_value);

int armv7m_maybe_skip_bkpt_inst(struct target *target, bool *inst_found);

extern const struct command_registration armv7m_command_handlers[];

#endif /* OPENOCD_TARGET_ARMV7M_H */
