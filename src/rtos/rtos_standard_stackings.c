/***************************************************************************
 *   Copyright (C) 2011 by Broadcom Corporation                            *
 *   Evan Hunter - ehunter@broadcom.com                                    *
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

#include "rtos.h"
#include "target/armv7m.h"
#include "target/riscv/riscv.h"

static const struct stack_register_offset rtos_standard_cortex_m3_stack_offsets[ARMV7M_NUM_CORE_REGS] = {
	{ ARMV7M_R0,   0x20, 32 },		/* r0   */
	{ ARMV7M_R1,   0x24, 32 },		/* r1   */
	{ ARMV7M_R2,   0x28, 32 },		/* r2   */
	{ ARMV7M_R3,   0x2c, 32 },		/* r3   */
	{ ARMV7M_R4,   0x00, 32 },		/* r4   */
	{ ARMV7M_R5,   0x04, 32 },		/* r5   */
	{ ARMV7M_R6,   0x08, 32 },		/* r6   */
	{ ARMV7M_R7,   0x0c, 32 },		/* r7   */
	{ ARMV7M_R8,   0x10, 32 },		/* r8   */
	{ ARMV7M_R9,   0x14, 32 },		/* r9   */
	{ ARMV7M_R10,  0x18, 32 },		/* r10  */
	{ ARMV7M_R11,  0x1c, 32 },		/* r11  */
	{ ARMV7M_R12,  0x30, 32 },		/* r12  */
	{ ARMV7M_R13,  -2,   32 },		/* sp   */
	{ ARMV7M_R14,  0x34, 32 },		/* lr   */
	{ ARMV7M_PC,   0x38, 32 },		/* pc   */
	{ ARMV7M_xPSR, 0x3c, 32 },		/* xPSR */
};

static const struct stack_register_offset rtos_standard_cortex_m4f_stack_offsets[] = {
	{ ARMV7M_R0,   0x24, 32 },		/* r0   */
	{ ARMV7M_R1,   0x28, 32 },		/* r1   */
	{ ARMV7M_R2,   0x2c, 32 },		/* r2   */
	{ ARMV7M_R3,   0x30, 32 },		/* r3   */
	{ ARMV7M_R4,   0x00, 32 },		/* r4   */
	{ ARMV7M_R5,   0x04, 32 },		/* r5   */
	{ ARMV7M_R6,   0x08, 32 },		/* r6   */
	{ ARMV7M_R7,   0x0c, 32 },		/* r7   */
	{ ARMV7M_R8,   0x10, 32 },		/* r8   */
	{ ARMV7M_R9,   0x14, 32 },		/* r9   */
	{ ARMV7M_R10,  0x18, 32 },		/* r10  */
	{ ARMV7M_R11,  0x1c, 32 },		/* r11  */
	{ ARMV7M_R12,  0x34, 32 },		/* r12  */
	{ ARMV7M_R13,  -2,   32 },		/* sp   */
	{ ARMV7M_R14,  0x38, 32 },		/* lr   */
	{ ARMV7M_PC,   0x3c, 32 },		/* pc   */
	{ ARMV7M_xPSR, 0x40, 32 },		/* xPSR */
};

static const struct stack_register_offset rtos_standard_cortex_m4f_fpu_stack_offsets[] = {
	{ ARMV7M_R0,   0x64, 32 },		/* r0   */
	{ ARMV7M_R1,   0x68, 32 },		/* r1   */
	{ ARMV7M_R2,   0x6c, 32 },		/* r2   */
	{ ARMV7M_R3,   0x70, 32 },		/* r3   */
	{ ARMV7M_R4,   0x00, 32 },		/* r4   */
	{ ARMV7M_R5,   0x04, 32 },		/* r5   */
	{ ARMV7M_R6,   0x08, 32 },		/* r6   */
	{ ARMV7M_R7,   0x0c, 32 },		/* r7   */
	{ ARMV7M_R8,   0x10, 32 },		/* r8   */
	{ ARMV7M_R9,   0x14, 32 },		/* r9   */
	{ ARMV7M_R10,  0x18, 32 },		/* r10  */
	{ ARMV7M_R11,  0x1c, 32 },		/* r11  */
	{ ARMV7M_R12,  0x74, 32 },		/* r12  */
	{ ARMV7M_R13,  -2,   32 },		/* sp   */
	{ ARMV7M_R14,  0x78, 32 },		/* lr   */
	{ ARMV7M_PC,   0x7c, 32 },		/* pc   */
	{ ARMV7M_xPSR, 0x80, 32 },		/* xPSR */
};


static const struct stack_register_offset rtos_standard_cortex_r4_stack_offsets[] = {
	{ 0,  0x08, 32 },		/* r0  (a1)   */
	{ 1,  0x0c, 32 },		/* r1  (a2)  */
	{ 2,  0x10, 32 },		/* r2  (a3)  */
	{ 3,  0x14, 32 },		/* r3  (a4)  */
	{ 4,  0x18, 32 },		/* r4  (v1)  */
	{ 5,  0x1c, 32 },		/* r5  (v2)  */
	{ 6,  0x20, 32 },		/* r6  (v3)  */
	{ 7,  0x24, 32 },		/* r7  (v4)  */
	{ 8,  0x28, 32 },		/* r8  (a1)  */
	{ 10, 0x2c, 32 },		/* r9  (sb)  */
	{ 11, 0x30, 32 },		/* r10 (sl) */
	{ 12, 0x34, 32 },		/* r11 (fp) */
	{ 13, 0x38, 32 },		/* r12 (ip) */
	{ 14, -2,   32 },		/* sp   */
	{ 15, 0x3c, 32 },		/* lr   */
	{ 16, 0x40, 32 },		/* pc   */
	{ 17, -1,   96 },		/* FPA1 */
	{ 18, -1,   96 },		/* FPA2 */
	{ 19, -1,   96 },		/* FPA3 */
	{ 20, -1,   96 },		/* FPA4 */
	{ 21, -1,   96 },		/* FPA5 */
	{ 22, -1,   96 },		/* FPA6 */
	{ 23, -1,   96 },		/* FPA7 */
	{ 24, -1,   96 },		/* FPA8 */
	{ 25, -1,   32 },		/* FPS  */
	{ 26, 0x04, 32 },		/* CSPR */
};

static const struct stack_register_offset rtos_standard_nds32_n1068_stack_offsets[] = {
	{ 0,  0x88, 32 },		/* R0  */
	{ 1,  0x8C, 32 },		/* R1 */
	{ 2,  0x14, 32 },		/* R2 */
	{ 3,  0x18, 32 },		/* R3 */
	{ 4,  0x1C, 32 },		/* R4 */
	{ 5,  0x20, 32 },		/* R5 */
	{ 6,  0x24, 32 },		/* R6 */
	{ 7,  0x28, 32 },		/* R7 */
	{ 8,  0x2C, 32 },		/* R8 */
	{ 9,  0x30, 32 },		/* R9 */
	{ 10, 0x34, 32 },		/* R10 */
	{ 11, 0x38, 32 },		/* R11 */
	{ 12, 0x3C, 32 },		/* R12 */
	{ 13, 0x40, 32 },		/* R13 */
	{ 14, 0x44, 32 },		/* R14 */
	{ 15, 0x48, 32 },		/* R15 */
	{ 16, 0x4C, 32 },		/* R16 */
	{ 17, 0x50, 32 },		/* R17 */
	{ 18, 0x54, 32 },		/* R18 */
	{ 19, 0x58, 32 },		/* R19 */
	{ 20, 0x5C, 32 },		/* R20 */
	{ 21, 0x60, 32 },		/* R21 */
	{ 22, 0x64, 32 },		/* R22 */
	{ 23, 0x68, 32 },		/* R23 */
	{ 24, 0x6C, 32 },		/* R24 */
	{ 25, 0x70, 32 },		/* R25 */
	{ 26, 0x74, 32 },		/* R26 */
	{ 27, 0x78, 32 },		/* R27 */
	{ 28, 0x7C, 32 },		/* R28 */
	{ 29, 0x80, 32 },		/* R29 */
	{ 30, 0x84, 32 },		/* R30 (LP) */
	{ 31, 0x00, 32 },		/* R31 (SP) */
	{ 32, 0x04, 32 },		/* PSW */
	{ 33, 0x08, 32 },		/* IPC */
	{ 34, 0x0C, 32 },		/* IPSW */
	{ 35, 0x10, 32 },		/* IFC_LP */
};

static const struct stack_register_offset rtos_metal_rv32_stack_offsets[] = {
	/* zero isn't on the stack. By making its offset -1 we leave the value at 0
	 * inside rtos_generic_stack_read(). */
	{ GDB_REGNO_ZERO,  -1, 32 },
	{ GDB_REGNO_RA,  0x04, 32 },
	{ GDB_REGNO_SP,  0x08, 32 },
	{ GDB_REGNO_GP,  0x0c, 32 },
	{ GDB_REGNO_TP,  0x10, 32 },
	{ GDB_REGNO_T0,  0x14, 32 },
	{ GDB_REGNO_T1,  0x18, 32 },
	{ GDB_REGNO_T2,  0x1c, 32 },
	{ GDB_REGNO_FP,  0x20, 32 },
	{ GDB_REGNO_S1,  0x24, 32 },
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
	{ GDB_REGNO_PC, 0x80, 32 },
	/* Registers below are on the stack, but not what gdb expects to return from
	 * a 'g' packet so are only accessible through get_reg. */
	{ GDB_REGNO_MSTATUS, 0x84, 32 },
};

static const struct stack_register_offset rtos_metal_rv64_stack_offsets[] = {
	/* zero isn't on the stack. By making its offset -1 we leave the value at 0
	 * inside rtos_generic_stack_read(). */
	{ GDB_REGNO_ZERO,  -1, 64 },
	{ GDB_REGNO_RA, 2 * 0x04, 64 },
	{ GDB_REGNO_SP, 2 * 0x08, 64 },
	{ GDB_REGNO_GP, 2 * 0x0c, 64 },
	{ GDB_REGNO_TP, 2 * 0x10, 64 },
	{ GDB_REGNO_T0, 2 * 0x14, 64 },
	{ GDB_REGNO_T1, 2 * 0x18, 64 },
	{ GDB_REGNO_T2, 2 * 0x1c, 64 },
	{ GDB_REGNO_FP, 2 * 0x20, 64 },
	{ GDB_REGNO_S1, 2 * 0x24, 64 },
	{ GDB_REGNO_A0, 2 * 0x28, 64 },
	{ GDB_REGNO_A1, 2 * 0x2c, 64 },
	{ GDB_REGNO_A2, 2 * 0x30, 64 },
	{ GDB_REGNO_A3, 2 * 0x34, 64 },
	{ GDB_REGNO_A4, 2 * 0x38, 64 },
	{ GDB_REGNO_A5, 2 * 0x3c, 64 },
	{ GDB_REGNO_A6, 2 * 0x40, 64 },
	{ GDB_REGNO_A7, 2 * 0x44, 64 },
	{ GDB_REGNO_S2, 2 * 0x48, 64 },
	{ GDB_REGNO_S3, 2 * 0x4c, 64 },
	{ GDB_REGNO_S4, 2 * 0x50, 64 },
	{ GDB_REGNO_S5, 2 * 0x54, 64 },
	{ GDB_REGNO_S6, 2 * 0x58, 64 },
	{ GDB_REGNO_S7, 2 * 0x5c, 64 },
	{ GDB_REGNO_S8, 2 * 0x60, 64 },
	{ GDB_REGNO_S9, 2 * 0x64, 64 },
	{ GDB_REGNO_S10, 2 * 0x68, 64 },
	{ GDB_REGNO_S11, 2 * 0x6c, 64 },
	{ GDB_REGNO_T3, 2 * 0x70, 64 },
	{ GDB_REGNO_T4, 2 * 0x74, 64 },
	{ GDB_REGNO_T5, 2 * 0x78, 64 },
	{ GDB_REGNO_T6, 2 * 0x7c, 64 },
	{ GDB_REGNO_PC, 2 * 0x80, 64 },
	/* Registers below are on the stack, but not what gdb expects to return from
	 * a 'g' packet so are only accessible through get_reg. */
	{ GDB_REGNO_MSTATUS, 2 * 0x84, 64 },
};

static const struct stack_register_offset rtos_standard_rv32_stack_offsets[] = {
	/* zero isn't on the stack. By making its offset -1 we leave the value at 0
	 * inside rtos_generic_stack_read(). */
	{ GDB_REGNO_ZERO,  -1, 32 },
	{ GDB_REGNO_RA,  0x04, 32 },
	{ GDB_REGNO_SP,  -2, 32 },
	{ GDB_REGNO_GP,  -2, 32 },
	{ GDB_REGNO_TP,  -2, 32 },
	{ GDB_REGNO_T0,  0x08, 32 },
	{ GDB_REGNO_T1,  0x0c, 32 },
	{ GDB_REGNO_T2,  0x10, 32 },
	{ GDB_REGNO_FP,  0x14, 32 },
	{ GDB_REGNO_S1,  0x18, 32 },
	{ GDB_REGNO_A0, 0x1c, 32 },
	{ GDB_REGNO_A1, 0x20, 32 },
	{ GDB_REGNO_A2, 0x24, 32 },
	{ GDB_REGNO_A3, 0x28, 32 },
	{ GDB_REGNO_A4, 0x2c, 32 },
	{ GDB_REGNO_A5, 0x30, 32 },
	{ GDB_REGNO_A6, 0x34, 32 },
	{ GDB_REGNO_A7, 0x38, 32 },
	{ GDB_REGNO_S2, 0x3c, 32 },
	{ GDB_REGNO_S3, 0x40, 32 },
	{ GDB_REGNO_S4, 0x44, 32 },
	{ GDB_REGNO_S5, 0x48, 32 },
	{ GDB_REGNO_S6, 0x4c, 32 },
	{ GDB_REGNO_S7, 0x50, 32 },
	{ GDB_REGNO_S8, 0x54, 32 },
	{ GDB_REGNO_S9, 0x58, 32 },
	{ GDB_REGNO_S10, 0x5c, 32 },
	{ GDB_REGNO_S11, 0x60, 32 },
	{ GDB_REGNO_T3, 0x64, 32 },
	{ GDB_REGNO_T4, 0x68, 32 },
	{ GDB_REGNO_T5, 0x6c, 32 },
	{ GDB_REGNO_T6, 0x70, 32 },
	{ GDB_REGNO_PC, 0, 32 },
	/* Registers below are on the stack, but not what gdb expects to return from
	 * a 'g' packet so are only accessible through get_reg. */
	{ GDB_REGNO_MSTATUS, 29 * 4, 32 },
};

static const struct stack_register_offset rtos_standard_rv64_stack_offsets[] = {
	/* zero isn't on the stack. By making its offset -1 we leave the value at 0
	 * inside rtos_generic_stack_read(). */
	{ GDB_REGNO_ZERO,  -1, 64 },
	{ GDB_REGNO_RA, 2 * 0x04, 64 },
	{ GDB_REGNO_SP, -2, 64 },
	{ GDB_REGNO_GP, -2, 64 },
	{ GDB_REGNO_TP, -2, 64 },
	{ GDB_REGNO_T0, 2 * 0x08, 64 },
	{ GDB_REGNO_T1, 2 * 0x0c, 64 },
	{ GDB_REGNO_T2, 2 * 0x10, 64 },
	{ GDB_REGNO_FP, 2 * 0x14, 64 },
	{ GDB_REGNO_S1, 2 * 0x18, 64 },
	{ GDB_REGNO_A0, 2 * 0x1c, 64 },
	{ GDB_REGNO_A1, 2 * 0x20, 64 },
	{ GDB_REGNO_A2, 2 * 0x24, 64 },
	{ GDB_REGNO_A3, 2 * 0x28, 64 },
	{ GDB_REGNO_A4, 2 * 0x2c, 64 },
	{ GDB_REGNO_A5, 2 * 0x30, 64 },
	{ GDB_REGNO_A6, 2 * 0x34, 64 },
	{ GDB_REGNO_A7, 2 * 0x38, 64 },
	{ GDB_REGNO_S2, 2 * 0x3c, 64 },
	{ GDB_REGNO_S3, 2 * 0x40, 64 },
	{ GDB_REGNO_S4, 2 * 0x44, 64 },
	{ GDB_REGNO_S5, 2 * 0x48, 64 },
	{ GDB_REGNO_S6, 2 * 0x4c, 64 },
	{ GDB_REGNO_S7, 2 * 0x50, 64 },
	{ GDB_REGNO_S8, 2 * 0x54, 64 },
	{ GDB_REGNO_S9, 2 * 0x58, 64 },
	{ GDB_REGNO_S10, 2 * 0x5c, 64 },
	{ GDB_REGNO_S11, 2 * 0x60, 64 },
	{ GDB_REGNO_T3, 2 * 0x64, 64 },
	{ GDB_REGNO_T4, 2 * 0x68, 64 },
	{ GDB_REGNO_T5, 2 * 0x6c, 64 },
	{ GDB_REGNO_T6, 2 * 0x70, 64 },
	{ GDB_REGNO_PC, 0, 64 },
	/* Registers below are on the stack, but not what gdb expects to return from
	 * a 'g' packet so are only accessible through get_reg. */
	{ GDB_REGNO_MSTATUS, 2 * 29 * 4, 64 },
};

static target_addr_t rtos_generic_stack_align(struct target *target,
	const uint8_t *stack_data, const struct rtos_register_stacking *stacking,
	target_addr_t stack_ptr, int align)
{
	target_addr_t new_stack_ptr = stack_ptr;
	if (stacking->stack_growth_direction > 0)
		new_stack_ptr -= stacking->stack_registers_size;
	else
		new_stack_ptr += stacking->stack_registers_size;
	target_addr_t aligned_stack_ptr = new_stack_ptr & ~((int64_t)align - 1);

	if (aligned_stack_ptr != new_stack_ptr &&
		stacking->stack_growth_direction == -1) {
		/* If we have a downward growing stack, the simple alignment code
		 * above results in a wrong result (since it rounds down to nearest
		 * alignment).  We want to round up so add an extra align.
		 */
		aligned_stack_ptr += (target_addr_t)align;
	}
	return aligned_stack_ptr;
}

target_addr_t rtos_generic_stack_align8(struct target *target,
	const uint8_t *stack_data, const struct rtos_register_stacking *stacking,
	target_addr_t stack_ptr)
{
	return rtos_generic_stack_align(target, stack_data,
			stacking, stack_ptr, 8);
}

/* The Cortex-M3 will indicate that an alignment adjustment
 * has been done on the stack by setting bit 9 of the stacked xPSR
 * register.  In this case, we can just add an extra 4 bytes to get
 * to the program stack.  Note that some places in the ARM documentation
 * make this a little unclear but the padding takes place before the
 * normal exception stacking - so xPSR is always available at a fixed
 * location.
 *
 * Relevant documentation:
 *    Cortex-M series processors -> Cortex-M3 -> Revision: xxx ->
 *        Cortex-M3 Devices Generic User Guide -> The Cortex-M3 Processor ->
 *        Exception Model -> Exception entry and return -> Exception entry
 *    Cortex-M series processors -> Cortex-M3 -> Revision: xxx ->
 *        Cortex-M3 Devices Generic User Guide -> Cortex-M3 Peripherals ->
 *        System control block -> Configuration and Control Register (STKALIGN)
 *
 * This is just a helper function for use in the calculate_process_stack
 * function for a given architecture/rtos.
 */
target_addr_t rtos_cortex_m_stack_align(struct target *target,
	const uint8_t *stack_data, const struct rtos_register_stacking *stacking,
	target_addr_t stack_ptr, size_t xpsr_offset)
{
	const uint32_t ALIGN_NEEDED = (1 << 9);
	uint32_t xpsr;
	target_addr_t new_stack_ptr;

	new_stack_ptr = stack_ptr - stacking->stack_growth_direction *
		stacking->stack_registers_size;
	xpsr = (target->endianness == TARGET_LITTLE_ENDIAN) ?
			le_to_h_u32(&stack_data[xpsr_offset]) :
			be_to_h_u32(&stack_data[xpsr_offset]);
	if ((xpsr & ALIGN_NEEDED) != 0) {
		LOG_DEBUG("XPSR(0x%08" PRIx32 ") indicated stack alignment was necessary\r\n",
			xpsr);
		new_stack_ptr -= (stacking->stack_growth_direction * 4);
	}
	return new_stack_ptr;
}

static target_addr_t rtos_standard_cortex_m3_stack_align(struct target *target,
	const uint8_t *stack_data, const struct rtos_register_stacking *stacking,
	target_addr_t stack_ptr)
{
	const int XPSR_OFFSET = 0x3c;
	return rtos_cortex_m_stack_align(target, stack_data, stacking,
		stack_ptr, XPSR_OFFSET);
}

static target_addr_t rtos_standard_cortex_m4f_stack_align(struct target *target,
	const uint8_t *stack_data, const struct rtos_register_stacking *stacking,
	target_addr_t stack_ptr)
{
	const int XPSR_OFFSET = 0x40;
	return rtos_cortex_m_stack_align(target, stack_data, stacking,
		stack_ptr, XPSR_OFFSET);
}

static target_addr_t rtos_standard_cortex_m4f_fpu_stack_align(struct target *target,
	const uint8_t *stack_data, const struct rtos_register_stacking *stacking,
	target_addr_t stack_ptr)
{
	const int XPSR_OFFSET = 0x80;
	return rtos_cortex_m_stack_align(target, stack_data, stacking,
		stack_ptr, XPSR_OFFSET);
}


const struct rtos_register_stacking rtos_standard_cortex_m3_stacking = {
	.stack_registers_size = 0x40,
	.stack_growth_direction = -1,
	.num_output_registers = ARMV7M_NUM_CORE_REGS,
	.calculate_process_stack = rtos_standard_cortex_m3_stack_align,
	.register_offsets = rtos_standard_cortex_m3_stack_offsets
};

const struct rtos_register_stacking rtos_standard_cortex_m4f_stacking = {
	.stack_registers_size = 0x44,
	.stack_growth_direction = -1,
	.num_output_registers = ARMV7M_NUM_CORE_REGS,
	.calculate_process_stack = rtos_standard_cortex_m4f_stack_align,
	.register_offsets = rtos_standard_cortex_m4f_stack_offsets
};

const struct rtos_register_stacking rtos_standard_cortex_m4f_fpu_stacking = {
	.stack_registers_size = 0xcc,
	.stack_growth_direction = -1,
	.num_output_registers = ARMV7M_NUM_CORE_REGS,
	.calculate_process_stack = rtos_standard_cortex_m4f_fpu_stack_align,
	.register_offsets = rtos_standard_cortex_m4f_fpu_stack_offsets
};

const struct rtos_register_stacking rtos_standard_cortex_r4_stacking = {
	.stack_registers_size = 0x48,
	.stack_growth_direction = -1,
	.num_output_registers = 26,
	.calculate_process_stack = rtos_generic_stack_align8,
	.register_offsets = rtos_standard_cortex_r4_stack_offsets
};

const struct rtos_register_stacking rtos_standard_nds32_n1068_stacking = {
	.stack_registers_size = 0x90,
	.stack_growth_direction = -1,
	.num_output_registers = 32,
	.calculate_process_stack = rtos_generic_stack_align8,
	.register_offsets = rtos_standard_nds32_n1068_stack_offsets
};

const struct rtos_register_stacking rtos_metal_rv32_stacking = {
	.stack_registers_size = (32 + 2) * 4,
	.stack_growth_direction = -1,
	.num_output_registers = 33,
	.calculate_process_stack = rtos_generic_stack_align8,
	.register_offsets = rtos_metal_rv32_stack_offsets,
	.total_register_count = ARRAY_SIZE(rtos_metal_rv32_stack_offsets)
};

const struct rtos_register_stacking rtos_standard_rv32_stacking = {
	.stack_registers_size = (32 + 2) * 4,
	.stack_growth_direction = -1,
	.num_output_registers = 33,
	.calculate_process_stack = rtos_generic_stack_align8,
	.register_offsets = rtos_standard_rv32_stack_offsets,
	.total_register_count = ARRAY_SIZE(rtos_standard_rv32_stack_offsets)
};

const struct rtos_register_stacking rtos_metal_rv64_stacking = {
	.stack_registers_size = (32 + 2) * 8,
	.stack_growth_direction = -1,
	.num_output_registers = 33,
	.calculate_process_stack = rtos_generic_stack_align8,
	.register_offsets = rtos_metal_rv64_stack_offsets,
	.total_register_count = ARRAY_SIZE(rtos_metal_rv64_stack_offsets)
};

const struct rtos_register_stacking rtos_standard_rv64_stacking = {
	.stack_registers_size = (32 + 2) * 8,
	.stack_growth_direction = -1,
	.num_output_registers = 33,
	.calculate_process_stack = rtos_generic_stack_align8,
	.register_offsets = rtos_standard_rv64_stack_offsets,
	.total_register_count = ARRAY_SIZE(rtos_standard_rv64_stack_offsets)
};
