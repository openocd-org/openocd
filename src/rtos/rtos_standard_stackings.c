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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rtos.h"
#include "target/armv7m.h"

static const struct stack_register_offset rtos_standard_Cortex_M3_stack_offsets[ARMV7M_NUM_CORE_REGS] = {
	{ 0x20, 32 },		/* r0   */
	{ 0x24, 32 },		/* r1   */
	{ 0x28, 32 },		/* r2   */
	{ 0x2c, 32 },		/* r3   */
	{ 0x00, 32 },		/* r4   */
	{ 0x04, 32 },		/* r5   */
	{ 0x08, 32 },		/* r6   */
	{ 0x0c, 32 },		/* r7   */
	{ 0x10, 32 },		/* r8   */
	{ 0x14, 32 },		/* r9   */
	{ 0x18, 32 },		/* r10  */
	{ 0x1c, 32 },		/* r11  */
	{ 0x30, 32 },		/* r12  */
	{ -2,   32 },		/* sp   */
	{ 0x34, 32 },		/* lr   */
	{ 0x38, 32 },		/* pc   */
	{ 0x3c, 32 },		/* xPSR */
};

static const struct stack_register_offset rtos_standard_Cortex_R4_stack_offsets[] = {
	{ 0x08, 32 },		/* r0  (a1)   */
	{ 0x0c, 32 },		/* r1  (a2)  */
	{ 0x10, 32 },		/* r2  (a3)  */
	{ 0x14, 32 },		/* r3  (a4)  */
	{ 0x18, 32 },		/* r4  (v1)  */
	{ 0x1c, 32 },		/* r5  (v2)  */
	{ 0x20, 32 },		/* r6  (v3)  */
	{ 0x24, 32 },		/* r7  (v4)  */
	{ 0x28, 32 },		/* r8  (a1)  */
	{ 0x2c, 32 },		/* r9  (sb)  */
	{ 0x30, 32 },		/* r10 (sl) */
	{ 0x34, 32 },		/* r11 (fp) */
	{ 0x38, 32 },		/* r12 (ip) */
	{ -2,   32 },		/* sp   */
	{ 0x3c, 32 },		/* lr   */
	{ 0x40, 32 },		/* pc   */
	{ -1,   96 },		/* FPA1 */
	{ -1,   96 },		/* FPA2 */
	{ -1,   96 },		/* FPA3 */
	{ -1,   96 },		/* FPA4 */
	{ -1,   96 },		/* FPA5 */
	{ -1,   96 },		/* FPA6 */
	{ -1,   96 },		/* FPA7 */
	{ -1,   96 },		/* FPA8 */
	{ -1,   32 },		/* FPS  */
	{ 0x04, 32 },		/* CSPR */
};

static const struct stack_register_offset rtos_standard_NDS32_N1068_stack_offsets[] = {
	{ 0x88, 32 },		/* R0  */
	{ 0x8C, 32 },		/* R1 */
	{ 0x14, 32 },		/* R2 */
	{ 0x18, 32 },		/* R3 */
	{ 0x1C, 32 },		/* R4 */
	{ 0x20, 32 },		/* R5 */
	{ 0x24, 32 },		/* R6 */
	{ 0x28, 32 },		/* R7 */
	{ 0x2C, 32 },		/* R8 */
	{ 0x30, 32 },		/* R9 */
	{ 0x34, 32 },		/* R10 */
	{ 0x38, 32 },		/* R11 */
	{ 0x3C, 32 },		/* R12 */
	{ 0x40, 32 },		/* R13 */
	{ 0x44, 32 },		/* R14 */
	{ 0x48, 32 },		/* R15 */
	{ 0x4C, 32 },		/* R16 */
	{ 0x50, 32 },		/* R17 */
	{ 0x54, 32 },		/* R18 */
	{ 0x58, 32 },		/* R19 */
	{ 0x5C, 32 },		/* R20 */
	{ 0x60, 32 },		/* R21 */
	{ 0x64, 32 },		/* R22 */
	{ 0x68, 32 },		/* R23 */
	{ 0x6C, 32 },		/* R24 */
	{ 0x70, 32 },		/* R25 */
	{ 0x74, 32 },		/* R26 */
	{ 0x78, 32 },		/* R27 */
	{ 0x7C, 32 },		/* R28 */
	{ 0x80, 32 },		/* R29 */
	{ 0x84, 32 },		/* R30 (LP) */
	{ 0x00, 32 },		/* R31 (SP) */
	{ 0x04, 32 },		/* PSW */
	{ 0x08, 32 },		/* IPC */
	{ 0x0C, 32 },		/* IPSW */
	{ 0x10, 32 },		/* IFC_LP */
};

const struct rtos_register_stacking rtos_standard_Cortex_M3_stacking = {
	0x40,					/* stack_registers_size */
	-1,						/* stack_growth_direction */
	ARMV7M_NUM_CORE_REGS,	/* num_output_registers */
	8,						/* stack_alignment */
	rtos_standard_Cortex_M3_stack_offsets	/* register_offsets */
};

const struct rtos_register_stacking rtos_standard_Cortex_R4_stacking = {
	0x48,				/* stack_registers_size */
	-1,					/* stack_growth_direction */
	26,					/* num_output_registers */
	8,					/* stack_alignment */
	rtos_standard_Cortex_R4_stack_offsets	/* register_offsets */
};

const struct rtos_register_stacking rtos_standard_NDS32_N1068_stacking = {
	0x90,				/* stack_registers_size */
	-1,					/* stack_growth_direction */
	32,					/* num_output_registers */
	8,					/* stack_alignment */
	rtos_standard_NDS32_N1068_stack_offsets	/* register_offsets */
};
