// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rtos.h"
#include "rtos_standard_stackings.h"
#include "target/armv7m.h"

static const struct stack_register_offset rtos_ecos_cortex_m3_stack_offsets[ARMV7M_NUM_CORE_REGS] = {
	{ ARMV7M_R0,   0x0c, 32 },		/* r0   */
	{ ARMV7M_R1,   0x10, 32 },		/* r1   */
	{ ARMV7M_R2,   0x14, 32 },		/* r2   */
	{ ARMV7M_R3,   0x18, 32 },		/* r3   */
	{ ARMV7M_R4,   0x1c, 32 },		/* r4   */
	{ ARMV7M_R5,   0x20, 32 },		/* r5   */
	{ ARMV7M_R6,   0x24, 32 },		/* r6   */
	{ ARMV7M_R7,   0x28, 32 },		/* r7   */
	{ ARMV7M_R8,   0x2c, 32 },		/* r8   */
	{ ARMV7M_R9,   0x30, 32 },		/* r9   */
	{ ARMV7M_R10,  0x34, 32 },		/* r10  */
	{ ARMV7M_R11,  0x38, 32 },		/* r11  */
	{ ARMV7M_R12,  0x3c, 32 },		/* r12  */
	{ ARMV7M_R13,  -2,   32 },		/* sp   */
	{ ARMV7M_R14,  -1,   32 },		/* lr   */
	{ ARMV7M_PC,   0x40, 32 },		/* pc   */
	{ ARMV7M_XPSR, -1,   32 },		/* xPSR */
};

const struct rtos_register_stacking rtos_ecos_cortex_m3_stacking = {
	.stack_registers_size = 0x44,
	.stack_growth_direction = -1,
	.num_output_registers = ARMV7M_NUM_CORE_REGS,
	.calculate_process_stack = rtos_generic_stack_align8,
	.register_offsets = rtos_ecos_cortex_m3_stack_offsets
};
