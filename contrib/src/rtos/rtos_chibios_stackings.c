// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2012 by Matthias Blaicher                               *
 *   Matthias Blaicher - matthias@blaicher.com                             *
 *                                                                         *
 *   Copyright (C) 2011 by Broadcom Corporation                            *
 *   Evan Hunter - ehunter@broadcom.com                                    *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rtos.h"
#include "target/armv7m.h"

static const struct stack_register_offset rtos_chibios_arm_v7m_stack_offsets[ARMV7M_NUM_CORE_REGS] = {
	{ ARMV7M_R0,   -1,   32 },		/* r0   */
	{ ARMV7M_R1,   -1,   32 },		/* r1   */
	{ ARMV7M_R2,   -1,   32 },		/* r2   */
	{ ARMV7M_R3,   -1,   32 },		/* r3   */
	{ ARMV7M_R4,   0x00, 32 },		/* r4   */
	{ ARMV7M_R5,   0x04, 32 },		/* r5   */
	{ ARMV7M_R6,   0x08, 32 },		/* r6   */
	{ ARMV7M_R7,   0x0c, 32 },		/* r7   */
	{ ARMV7M_R8,   0x10, 32 },		/* r8   */
	{ ARMV7M_R9,   0x14, 32 },		/* r9   */
	{ ARMV7M_R10,  0x18, 32 },		/* r10  */
	{ ARMV7M_R11,  0x1c, 32 },		/* r11  */
	{ ARMV7M_R12,  -1,   32 },		/* r12  */
	{ ARMV7M_R13,  -2,   32 },		/* sp   */
	{ ARMV7M_R14,  -1,   32 },		/* lr   */
	{ ARMV7M_PC,   0x20, 32 },		/* pc   */
	{ ARMV7M_XPSR, -1,   32 },		/* xPSR */
};

const struct rtos_register_stacking rtos_chibios_arm_v7m_stacking = {
	.stack_registers_size = 0x24,
	.stack_growth_direction = -1,
	.num_output_registers = ARMV7M_NUM_CORE_REGS,
	.register_offsets = rtos_chibios_arm_v7m_stack_offsets
};

static const struct stack_register_offset rtos_chibios_arm_v7m_stack_offsets_w_fpu[ARMV7M_NUM_CORE_REGS] = {
	{ ARMV7M_R0,   -1,   32 },		/* r0   */
	{ ARMV7M_R1,   -1,   32 },		/* r1   */
	{ ARMV7M_R2,   -1,   32 },		/* r2   */
	{ ARMV7M_R3,   -1,   32 },		/* r3   */
	{ ARMV7M_R4,   0x40, 32 },		/* r4   */
	{ ARMV7M_R5,   0x44, 32 },		/* r5   */
	{ ARMV7M_R6,   0x48, 32 },		/* r6   */
	{ ARMV7M_R7,   0x4c, 32 },		/* r7   */
	{ ARMV7M_R8,   0x50, 32 },		/* r8   */
	{ ARMV7M_R9,   0x54, 32 },		/* r9   */
	{ ARMV7M_R10,  0x58, 32 },		/* r10  */
	{ ARMV7M_R11,  0x5c, 32 },		/* r11  */
	{ ARMV7M_R12,  -1,   32 },		/* r12  */
	{ ARMV7M_R13,  -2,   32 },		/* sp   */
	{ ARMV7M_R14,  -1,   32 },		/* lr   */
	{ ARMV7M_PC,   0x60, 32 },		/* pc   */
	{ ARMV7M_XPSR, -1,   32 },		/* xPSR */
};

const struct rtos_register_stacking rtos_chibios_arm_v7m_stacking_w_fpu = {
	.stack_registers_size = 0x64,
	.stack_growth_direction = -1,
	.num_output_registers = ARMV7M_NUM_CORE_REGS,
	.register_offsets = rtos_chibios_arm_v7m_stack_offsets_w_fpu
};
