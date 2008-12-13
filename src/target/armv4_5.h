/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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
#ifndef ARMV4_5_H
#define ARMV4_5_H

#include "register.h"
#include "target.h"
#include "log.h"

typedef enum armv4_5_mode
{
	ARMV4_5_MODE_USR = 16, 
	ARMV4_5_MODE_FIQ = 17, 
	ARMV4_5_MODE_IRQ = 18, 
	ARMV4_5_MODE_SVC = 19, 
	ARMV4_5_MODE_ABT = 23,
	ARMV4_5_MODE_UND = 27,
	ARMV4_5_MODE_SYS = 31,
	ARMV4_5_MODE_ANY = -1
} armv4_5_mode_t;

extern char** armv4_5_mode_strings;

typedef enum armv4_5_state
{
	ARMV4_5_STATE_ARM,
	ARMV4_5_STATE_THUMB,
	ARMV4_5_STATE_JAZELLE,
} armv4_5_state_t;

extern char* armv4_5_state_strings[];

extern int armv4_5_core_reg_map[7][17];

#define ARMV4_5_CORE_REG_MODE(cache, mode, num) \
		cache->reg_list[armv4_5_core_reg_map[armv4_5_mode_to_number(mode)][num]]
#define ARMV4_5_CORE_REG_MODENUM(cache, mode, num) \
		cache->reg_list[armv4_5_core_reg_map[mode][num]]

/* offsets into armv4_5 core register cache */
enum 
{
	ARMV4_5_CPSR = 31,
	ARMV4_5_SPSR_FIQ = 32,
	ARMV4_5_SPSR_IRQ = 33,
	ARMV4_5_SPSR_SVC = 34,
	ARMV4_5_SPSR_ABT = 35,
	ARMV4_5_SPSR_UND = 36
};

#define ARMV4_5_COMMON_MAGIC 0x0A450A45

typedef struct armv4_5_common_s
{
	int common_magic;
	reg_cache_t *core_cache;
	enum armv4_5_mode core_mode;
	enum armv4_5_state core_state;
	int (*full_context)(struct target_s *target);
	int (*read_core_reg)(struct target_s *target, int num, enum armv4_5_mode mode);
	int (*write_core_reg)(struct target_s *target, int num, enum armv4_5_mode mode, u32 value);
	void *arch_info;
} armv4_5_common_t;

typedef struct armv4_5_algorithm_s
{
	int common_magic;
		
	enum armv4_5_mode core_mode;
	enum armv4_5_state core_state;
} armv4_5_algorithm_t;

typedef struct armv4_5_core_reg_s
{
	int num;
	enum armv4_5_mode mode;
	target_t *target;
	armv4_5_common_t *armv4_5_common;
} armv4_5_core_reg_t;

extern reg_cache_t* armv4_5_build_reg_cache(target_t *target, armv4_5_common_t *armv4_5_common);

/* map psr mode bits to linear number */
static __inline int armv4_5_mode_to_number(enum armv4_5_mode mode)
{
	switch (mode)
	{
		case ARMV4_5_MODE_USR: return 0; break;
		case ARMV4_5_MODE_FIQ: return 1; break;
		case ARMV4_5_MODE_IRQ: return 2; break;
		case ARMV4_5_MODE_SVC: return 3; break;
		case ARMV4_5_MODE_ABT: return 4; break;
		case ARMV4_5_MODE_UND: return 5; break;
		case ARMV4_5_MODE_SYS: return 6; break;
		case ARMV4_5_MODE_ANY: return 0; break;	/* map MODE_ANY to user mode */
		default: 
			LOG_ERROR("invalid mode value encountered");
			return -1;
	}
}

/* map linear number to mode bits */
static __inline enum armv4_5_mode armv4_5_number_to_mode(int number)
{
	switch(number)
	{
		case 0: return ARMV4_5_MODE_USR; break;
		case 1: return ARMV4_5_MODE_FIQ; break;
		case 2: return ARMV4_5_MODE_IRQ; break;
		case 3: return ARMV4_5_MODE_SVC; break;
		case 4: return ARMV4_5_MODE_ABT; break;
		case 5: return ARMV4_5_MODE_UND; break;
		case 6: return ARMV4_5_MODE_SYS; break;
		default: 
			LOG_ERROR("mode index out of bounds");
			return -1;
	}
};

extern int armv4_5_arch_state(struct target_s *target);
extern int armv4_5_get_gdb_reg_list(target_t *target, reg_t **reg_list[], int *reg_list_size);
extern int armv4_5_invalidate_core_regs(target_t *target);

extern int armv4_5_register_commands(struct command_context_s *cmd_ctx);
extern int armv4_5_init_arch_info(target_t *target, armv4_5_common_t *armv4_5);

extern int armv4_5_run_algorithm(struct target_s *target, int num_mem_params, mem_param_t *mem_params, int num_reg_params, reg_param_t *reg_params, u32 entry_point, u32 exit_point, int timeout_ms, void *arch_info);

extern int armv4_5_invalidate_core_regs(target_t *target);

/* ARM mode instructions
 */
 
/* Store multiple increment after
 * Rn: base register
 * List: for each bit in list: store register
 * S: in priviledged mode: store user-mode registers
 * W=1: update the base register. W=0: leave the base register untouched
 */
#define ARMV4_5_STMIA(Rn, List, S, W)	(0xe8800000 | ((S) << 22) | ((W) << 21) | ((Rn) << 16) | (List))

/* Load multiple increment after
 * Rn: base register
 * List: for each bit in list: store register
 * S: in priviledged mode: store user-mode registers
 * W=1: update the base register. W=0: leave the base register untouched
 */
#define ARMV4_5_LDMIA(Rn, List, S, W)	(0xe8900000 | ((S) << 22) | ((W) << 21) | ((Rn) << 16) | (List))

/* MOV r8, r8 */
#define ARMV4_5_NOP					(0xe1a08008)

/* Move PSR to general purpose register
 * R=1: SPSR R=0: CPSR
 * Rn: target register
 */
#define ARMV4_5_MRS(Rn, R)			(0xe10f0000 | ((R) << 22) | ((Rn) << 12))

/* Store register
 * Rd: register to store
 * Rn: base register
 */
#define ARMV4_5_STR(Rd, Rn)			(0xe5800000 | ((Rd) << 12) | ((Rn) << 16))

/* Load register
 * Rd: register to load
 * Rn: base register
 */
#define ARMV4_5_LDR(Rd, Rn)			(0xe5900000 | ((Rd) << 12) | ((Rn) << 16))

/* Move general purpose register to PSR
 * R=1: SPSR R=0: CPSR
 * Field: Field mask
 * 1: control field 2: extension field 4: status field 8: flags field
 * Rm: source register
 */
#define ARMV4_5_MSR_GP(Rm, Field, R)	(0xe120f000 | (Rm) | ((Field) << 16) | ((R) << 22))
#define ARMV4_5_MSR_IM(Im, Rotate, Field, R)	(0xe320f000 | (Im)  | ((Rotate) << 8) | ((Field) << 16) | ((R) << 22))

/* Load Register Halfword Immediate Post-Index
 * Rd: register to load
 * Rn: base register
 */
#define ARMV4_5_LDRH_IP(Rd, Rn)	(0xe0d000b2 | ((Rd) << 12) | ((Rn) << 16))

/* Load Register Byte Immediate Post-Index
 * Rd: register to load
 * Rn: base register
 */
#define ARMV4_5_LDRB_IP(Rd, Rn)	(0xe4d00001 | ((Rd) << 12) | ((Rn) << 16))

/* Store register Halfword Immediate Post-Index
 * Rd: register to store
 * Rn: base register
 */
#define ARMV4_5_STRH_IP(Rd, Rn)	(0xe0c000b2 | ((Rd) << 12) | ((Rn) << 16))

/* Store register Byte Immediate Post-Index
 * Rd: register to store
 * Rn: base register
 */
#define ARMV4_5_STRB_IP(Rd, Rn)	(0xe4c00001 | ((Rd) << 12) | ((Rn) << 16))

/* Branch (and Link)
 * Im: Branch target (left-shifted by 2 bits, added to PC)
 * L: 1: branch and link 0: branch only
 */
#define ARMV4_5_B(Im, L) (0xea000000 | (Im) | ((L) << 24))

/* Branch and exchange (ARM state)
 * Rm: register holding branch target address
 */
#define ARMV4_5_BX(Rm) (0xe12fff10 | (Rm))

/* Move to ARM register from coprocessor
 * CP: Coprocessor number
 * op1: Coprocessor opcode
 * Rd: destination register
 * CRn: first coprocessor operand
 * CRm: second coprocessor operand
 * op2: Second coprocessor opcode
 */
#define ARMV4_5_MRC(CP, op1, Rd, CRn, CRm, op2) (0xee100010 | (CRm) | ((op2) << 5) | ((CP) << 8) | ((Rd) << 12) | ((CRn) << 16) | ((op1) << 21)) 

/* Move to coprocessor from ARM register
 * CP: Coprocessor number
 * op1: Coprocessor opcode
 * Rd: destination register
 * CRn: first coprocessor operand
 * CRm: second coprocessor operand
 * op2: Second coprocessor opcode
 */
#define ARMV4_5_MCR(CP, op1, Rd, CRn, CRm, op2) (0xee000010 | (CRm) | ((op2) << 5) | ((CP) << 8) | ((Rd) << 12) | ((CRn) << 16) | ((op1) << 21)) 

/* Breakpoint instruction (ARMv5)
 * Im: 16-bit immediate
 */
#define ARMV5_BKPT(Im) (0xe1200070 | ((Im & 0xfff0) << 8) | (Im & 0xf))


/* Thumb mode instructions
 */
 
/* Store register (Thumb mode)
 * Rd: source register
 * Rn: base register
 */
#define ARMV4_5_T_STR(Rd, Rn)	((0x6000 | (Rd) | ((Rn) << 3)) | ((0x6000 | (Rd) | ((Rn) << 3)) << 16))

/* Load register (Thumb state)
 * Rd: destination register
 * Rn: base register
 */
#define ARMV4_5_T_LDR(Rd, Rn)	((0x6800 | ((Rn) << 3) | (Rd)) | ((0x6800 | ((Rn) << 3) | (Rd)) << 16))

/* Load multiple (Thumb state)
 * Rn: base register
 * List: for each bit in list: store register
 */
#define ARMV4_5_T_LDMIA(Rn, List) ((0xc800 | ((Rn) << 8) | (List)) | ((0xc800 | ((Rn) << 8) | List) << 16))
 
/* Load register with PC relative addressing
 * Rd: register to load
 */
#define ARMV4_5_T_LDR_PCREL(Rd)	((0x4800 | ((Rd) << 8)) | ((0x4800 | ((Rd) << 8)) << 16)) 
 
/* Move hi register (Thumb mode)
 * Rd: destination register
 * Rm: source register
 */
#define ARMV4_5_T_MOV(Rd, Rm)	((0x4600 | ((Rd) & 0x7) | (((Rd) & 0x8) << 4) | (((Rm) & 0x7) << 3) | (((Rm) & 0x8) << 3)) | ((0x4600 | ((Rd) & 0x7) | (((Rd) & 0x8) << 4) | (((Rm) & 0x7) << 3) | (((Rm) & 0x8) << 3)) << 16))

/* No operation (Thumb mode)
 */
#define ARMV4_5_T_NOP	(0x46c0 | (0x46c0 << 16))

/* Move immediate to register (Thumb state)
 * Rd: destination register
 * Im: 8-bit immediate value
 */
#define ARMV4_5_T_MOV_IM(Rd, Im)	((0x2000 | ((Rd) << 8) | (Im)) | ((0x2000 | ((Rd) << 8) | (Im)) << 16))

/* Branch and Exchange
 * Rm: register containing branch target
 */
#define ARMV4_5_T_BX(Rm)		((0x4700 | ((Rm) << 3)) | ((0x4700 | ((Rm) << 3)) << 16))

/* Branch (Thumb state)
 * Imm: Branch target
 */
#define ARMV4_5_T_B(Imm)	((0xe000 | (Imm)) | ((0xe000 | (Imm)) << 16))

/* Breakpoint instruction (ARMv5) (Thumb state)
 * Im: 8-bit immediate
 */
#define ARMV5_T_BKPT(Im) ((0xbe00 | Im) | ((0xbe00 | Im) << 16))

#endif /* ARMV4_5_H */
