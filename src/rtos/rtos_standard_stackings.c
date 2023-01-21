// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2011 by Broadcom Corporation                            *
 *   Evan Hunter - ehunter@broadcom.com                                    *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rtos.h"
#include "target/armv7m.h"
#include "rtos_standard_stackings.h"

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
	{ ARMV7M_XPSR, 0x3c, 32 },		/* xPSR */
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
	{ ARMV7M_XPSR, 0x40, 32 },		/* xPSR */
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
	{ ARMV7M_XPSR, 0x80, 32 },		/* xPSR */
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

static target_addr_t rtos_generic_stack_align(struct target *target,
	const uint8_t *stack_data, const struct rtos_register_stacking *stacking,
	target_addr_t stack_ptr, int align)
{
	target_addr_t new_stack_ptr;
	target_addr_t aligned_stack_ptr;
	new_stack_ptr = stack_ptr - stacking->stack_growth_direction *
		stacking->stack_registers_size;
	aligned_stack_ptr = new_stack_ptr & ~((target_addr_t)align - 1);
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
