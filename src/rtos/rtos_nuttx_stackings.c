// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rtos.h"
#include "target/armv7m.h"
#include "rtos_nuttx_stackings.h"
#include "rtos_standard_stackings.h"
#include <target/riscv/riscv.h>

/* see arch/arm/include/armv7-m/irq_cmnvector.h */
static const struct stack_register_offset nuttx_stack_offsets_cortex_m[] = {
	{ ARMV7M_R0, 0x28, 32 },		/* r0   */
	{ ARMV7M_R1, 0x2c, 32 },		/* r1   */
	{ ARMV7M_R2, 0x30, 32 },		/* r2   */
	{ ARMV7M_R3, 0x34, 32 },		/* r3   */
	{ ARMV7M_R4, 0x08, 32 },		/* r4   */
	{ ARMV7M_R5, 0x0c, 32 },		/* r5   */
	{ ARMV7M_R6, 0x10, 32 },		/* r6   */
	{ ARMV7M_R7, 0x14, 32 },		/* r7   */
	{ ARMV7M_R8, 0x18, 32 },		/* r8   */
	{ ARMV7M_R9, 0x1c, 32 },		/* r9   */
	{ ARMV7M_R10, 0x20, 32 },		/* r10  */
	{ ARMV7M_R11, 0x24, 32 },		/* r11  */
	{ ARMV7M_R12, 0x38, 32 },		/* r12  */
	{ ARMV7M_R13, 0, 32 },			/* sp   */
	{ ARMV7M_R14, 0x3c, 32 },		/* lr   */
	{ ARMV7M_PC, 0x40, 32 },		/* pc   */
	{ ARMV7M_XPSR, 0x44, 32 },		/* xPSR */
};

const struct rtos_register_stacking nuttx_stacking_cortex_m = {
	.stack_registers_size = 0x48,
	.stack_growth_direction = -1,
	.num_output_registers = 17,
	.register_offsets = nuttx_stack_offsets_cortex_m,
};

static const struct stack_register_offset nuttx_stack_offsets_cortex_m_fpu[] = {
	{ ARMV7M_R0, 0x6c, 32 },		/* r0   */
	{ ARMV7M_R1, 0x70, 32 },		/* r1   */
	{ ARMV7M_R2, 0x74, 32 },		/* r2   */
	{ ARMV7M_R3, 0x78, 32 },		/* r3   */
	{ ARMV7M_R4, 0x08, 32 },		/* r4   */
	{ ARMV7M_R5, 0x0c, 32 },		/* r5   */
	{ ARMV7M_R6, 0x10, 32 },		/* r6   */
	{ ARMV7M_R7, 0x14, 32 },		/* r7   */
	{ ARMV7M_R8, 0x18, 32 },		/* r8   */
	{ ARMV7M_R9, 0x1c, 32 },		/* r9   */
	{ ARMV7M_R10, 0x20, 32 },		/* r10  */
	{ ARMV7M_R11, 0x24, 32 },		/* r11  */
	{ ARMV7M_R12, 0x7c, 32 },		/* r12  */
	{ ARMV7M_R13, 0, 32 },			/* sp   */
	{ ARMV7M_R14, 0x80, 32 },		/* lr   */
	{ ARMV7M_PC, 0x84, 32 },		/* pc   */
	{ ARMV7M_XPSR, 0x88, 32 },		/* xPSR */
};

const struct rtos_register_stacking nuttx_stacking_cortex_m_fpu = {
	.stack_registers_size = 0x8c,
	.stack_growth_direction = -1,
	.num_output_registers = 17,
	.register_offsets = nuttx_stack_offsets_cortex_m_fpu,
};

static const struct stack_register_offset nuttx_stack_offsets_riscv[] = {
	{ GDB_REGNO_ZERO, -1, 32 },
	{ GDB_REGNO_RA, 0x04, 32 },
	{ GDB_REGNO_SP, 0x08, 32 },
	{ GDB_REGNO_GP, 0x0c, 32 },
	{ GDB_REGNO_TP, 0x10, 32 },
	{ GDB_REGNO_T0, 0x14, 32 },
	{ GDB_REGNO_T1, 0x18, 32 },
	{ GDB_REGNO_T2, 0x1c, 32 },
	{ GDB_REGNO_FP, 0x20, 32 },
	{ GDB_REGNO_S1, 0x24, 32 },
	{ GDB_REGNO_A0, 0x28, 32 },
	{ GDB_REGNO_A1, 0x2c, 32 },
	{ GDB_REGNO_A2, 0x30, 32 },
	{ GDB_REGNO_A3, 0x34, 32 },
	{ GDB_REGNO_A4, 0x38, 32 },
	{ GDB_REGNO_A5, 0x3c, 32 },
	{ GDB_REGNO_A6, 0x40, 32 },
	{ GDB_REGNO_A7, 0x44, 32 },
	{ GDB_REGNO_S2, 0x48, 32 },
	{ GDB_REGNO_S3, 0x4c, 32 },
	{ GDB_REGNO_S4, 0x50, 32 },
	{ GDB_REGNO_S5, 0x54, 32 },
	{ GDB_REGNO_S6, 0x58, 32 },
	{ GDB_REGNO_S7, 0x5c, 32 },
	{ GDB_REGNO_S8, 0x60, 32 },
	{ GDB_REGNO_S9, 0x64, 32 },
	{ GDB_REGNO_S10, 0x68, 32 },
	{ GDB_REGNO_S11, 0x6c, 32 },
	{ GDB_REGNO_T3, 0x70, 32 },
	{ GDB_REGNO_T4, 0x74, 32 },
	{ GDB_REGNO_T5, 0x78, 32 },
	{ GDB_REGNO_T6, 0x7c, 32 },
	{ GDB_REGNO_PC, 0x00, 32 },
};

const struct rtos_register_stacking nuttx_riscv_stacking = {
	.stack_registers_size = 33 * 4,
	.stack_growth_direction = -1,
	.num_output_registers = 33,
	.calculate_process_stack = rtos_generic_stack_align8,
	.register_offsets = nuttx_stack_offsets_riscv,
};
