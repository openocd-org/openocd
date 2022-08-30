// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2015 by Daniel Krebs                                    *
 *   Daniel Krebs - github@daniel-krebs.net                                *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rtos.h"
#include "target/armv7m.h"
#include "rtos_standard_stackings.h"

/* This works for the M0 and M34 stackings as xPSR is in a fixed
 * location
 */
static target_addr_t rtos_riot_cortex_m_stack_align(struct target *target,
	const uint8_t *stack_data, const struct rtos_register_stacking *stacking,
	target_addr_t stack_ptr)
{
	const int XPSR_OFFSET = 0x40;
	return rtos_cortex_m_stack_align(target, stack_data, stacking,
		stack_ptr, XPSR_OFFSET);
}

/* see thread_arch.c */
static const struct stack_register_offset rtos_riot_cortex_m0_stack_offsets[ARMV7M_NUM_CORE_REGS] = {
	{ ARMV7M_R0,    0x24, 32 },		/* r0   */
	{ ARMV7M_R1,    0x28, 32 },		/* r1   */
	{ ARMV7M_R2,    0x2c, 32 },		/* r2   */
	{ ARMV7M_R3,    0x30, 32 },		/* r3   */
	{ ARMV7M_R4,    0x14, 32 },		/* r4   */
	{ ARMV7M_R5,    0x18, 32 },		/* r5   */
	{ ARMV7M_R6,    0x1c, 32 },		/* r6   */
	{ ARMV7M_R7,    0x20, 32 },		/* r7   */
	{ ARMV7M_R8,    0x04, 32 },		/* r8   */
	{ ARMV7M_R9,    0x08, 32 },		/* r9   */
	{ ARMV7M_R10,   0x0c, 32 },		/* r10  */
	{ ARMV7M_R11,   0x10, 32 },		/* r11  */
	{ ARMV7M_R12,   0x34, 32 },		/* r12  */
	{ ARMV7M_R13,   -2,   32 },		/* sp   */
	{ ARMV7M_R14,   0x38, 32 },		/* lr   */
	{ ARMV7M_PC,    0x3c, 32 },		/* pc   */
	{ ARMV7M_XPSR,  0x40, 32 },		/* xPSR */
};

const struct rtos_register_stacking rtos_riot_cortex_m0_stacking = {
	.stack_registers_size = 0x44,
	.stack_growth_direction = -1,
	.num_output_registers = ARMV7M_NUM_CORE_REGS,
	.calculate_process_stack = rtos_riot_cortex_m_stack_align,
	.register_offsets = rtos_riot_cortex_m0_stack_offsets
};

/* see thread_arch.c */
static const struct stack_register_offset rtos_riot_cortex_m34_stack_offsets[ARMV7M_NUM_CORE_REGS] = {
	{ ARMV7M_R0,    0x24, 32 },	/* r0   */
	{ ARMV7M_R1,    0x28, 32 },	/* r1   */
	{ ARMV7M_R2,    0x2c, 32 },	/* r2   */
	{ ARMV7M_R3,    0x30, 32 },	/* r3   */
	{ ARMV7M_R4,    0x04, 32 },	/* r4   */
	{ ARMV7M_R5,    0x08, 32 },	/* r5   */
	{ ARMV7M_R6,    0x0c, 32 },	/* r6   */
	{ ARMV7M_R7,    0x10, 32 },	/* r7   */
	{ ARMV7M_R8,    0x14, 32 },	/* r8   */
	{ ARMV7M_R9,    0x18, 32 },	/* r9   */
	{ ARMV7M_R10,   0x1c, 32 },	/* r10  */
	{ ARMV7M_R11,   0x20, 32 },	/* r11  */
	{ ARMV7M_R12,   0x34, 32 },	/* r12  */
	{ ARMV7M_R13,   -2,   32 },	/* sp   */
	{ ARMV7M_R14,   0x38, 32 },	/* lr   */
	{ ARMV7M_PC,    0x3c, 32 },	/* pc   */
	{ ARMV7M_XPSR,  0x40, 32 },	/* xPSR */
};

const struct rtos_register_stacking rtos_riot_cortex_m34_stacking = {
	.stack_registers_size = 0x44,
	.stack_growth_direction = -1,
	.num_output_registers = ARMV7M_NUM_CORE_REGS,
	.calculate_process_stack = rtos_riot_cortex_m_stack_align,
	.register_offsets = rtos_riot_cortex_m34_stack_offsets
};
