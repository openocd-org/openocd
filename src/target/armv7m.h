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

#include "register.h"
#include "target.h"
#include "arm_jtag.h"

/* define for enabling armv7 gdb workarounds */
#if 1
#define ARMV7_GDB_HACKS
#endif

enum armv7m_mode
{
	ARMV7M_MODE_THREAD = 0,
	ARMV7M_MODE_USER_THREAD = 1,
	ARMV7M_MODE_HANDLER = 2,
	ARMV7M_MODE_ANY = -1
};

extern char* armv7m_mode_strings[];

enum armv7m_regtype
{
	ARMV7M_REGISTER_CORE_GP,
	ARMV7M_REGISTER_CORE_SP,
	ARMV7M_REGISTER_MEMMAP
};

extern char* armv7m_exception_strings[];

extern char *armv7m_exception_string(int number);

/* offsets into armv7m core register cache */
enum 
{
	ARMV7M_PC = 15,
	ARMV7M_xPSR = 16,
	ARMV7M_MSP,
	ARMV7M_PSP,
	ARMV7M_PRIMASK,
	ARMV7M_BASEPRI,
	ARMV7M_FAULTMASK,
	ARMV7M_CONTROL,
	ARMV7NUMCOREREGS
};

#define ARMV7M_COMMON_MAGIC 0x2A452A45

typedef struct armv7m_common_s
{
	int common_magic;
	reg_cache_t *core_cache;
	enum armv7m_mode core_mode;
	int exception_number;
	
	/* Direct processor core register read and writes */
	int (*load_core_reg_u32)(struct target_s *target, enum armv7m_regtype type, u32 num, u32 *value);
	int (*store_core_reg_u32)(struct target_s *target, enum armv7m_regtype type, u32 num, u32 value);
	/* register cache to processor synchronization */
	int (*read_core_reg)(struct target_s *target, int num);
	int (*write_core_reg)(struct target_s *target, int num);
	
	int (*examine_debug_reason)(target_t *target);
	void (*pre_debug_entry)(target_t *target);
	void (*post_debug_entry)(target_t *target);
	
	void (*pre_restore_context)(target_t *target);
	void (*post_restore_context)(target_t *target);

	void *arch_info;
} armv7m_common_t;

typedef struct armv7m_algorithm_s
{
	int common_magic;
	
	enum armv7m_mode core_mode;
} armv7m_algorithm_t;

typedef struct armv7m_core_reg_s
{
	u32 num;
	enum armv7m_regtype type;
	enum armv7m_mode mode;
	target_t *target;
	armv7m_common_t *armv7m_common;
} armv7m_core_reg_t;

extern reg_cache_t *armv7m_build_reg_cache(target_t *target);
extern enum armv7m_mode armv7m_number_to_mode(int number);
extern int armv7m_mode_to_number(enum armv7m_mode mode);

extern int armv7m_arch_state(struct target_s *target);
extern int armv7m_get_gdb_reg_list(target_t *target, reg_t **reg_list[], int *reg_list_size);
extern int armv7m_invalidate_core_regs(target_t *target);

extern int armv7m_register_commands(struct command_context_s *cmd_ctx);
extern int armv7m_init_arch_info(target_t *target, armv7m_common_t *armv7m);

extern int armv7m_run_algorithm(struct target_s *target, int num_mem_params, mem_param_t *mem_params, int num_reg_params, reg_param_t *reg_params, u32 entry_point, u32 exit_point, int timeout_ms, void *arch_info);

extern int armv7m_invalidate_core_regs(target_t *target);

extern int armv7m_restore_context(target_t *target);

extern int armv7m_checksum_memory(struct target_s *target, u32 address, u32 count, u32* checksum);
extern int armv7m_blank_check_memory(struct target_s *target, u32 address, u32 count, u32* blank);

/* Thumb mode instructions
 */
 
/* Move to Register from Special Register  (Thumb mode) 32 bit Thumb2 instruction
 * Rd: destination register
 * SYSm: source special register
 */
#define ARMV7M_T_MRS(Rd, SYSm)	((0xF3EF) | ((0x8000 | (Rd<<8) | SYSm) << 16)) 

/* Move from Register from Special Register  (Thumb mode) 32 bit Thumb2 instruction
 * Rd: source register
 * SYSm: destination special register
 */
#define ARMV7M_T_MSR(SYSm, Rn)	((0xF380 | ( Rn<<8 )) | ((0x8800 | SYSm) << 16)) 

/* Change Processor State. The instruction modifies the PRIMASK and FAULTMASK 
 * special-purpose register values  (Thumb mode) 16 bit Thumb2 instruction
 * Rd: source register
 * IF: 
 */
#define I_FLAG 2
#define F_FLAG 1  
#define ARMV7M_T_CPSID(IF)	((0xB660 | (1<<8) | (IF&0x3)) | ((0xB660 | (1<<8) | (IF&0x3)) << 16)) 
#define ARMV7M_T_CPSIE(IF)	((0xB660 | (0<<8) | (IF&0x3)) | ((0xB660 | (0<<8) | (IF&0x3)) << 16)) 

/* Breakpoint (Thumb mode) v5 onwards
 * Im: immediate value used by debugger
 */
#define ARMV7M_T_BKPT(Im)	((0xBE00 | Im ) | ((0xBE00 | Im ) << 16))

/* Store register (Thumb mode)
 * Rd: source register
 * Rn: base register
 */
#define ARMV7M_T_STR(Rd, Rn)	((0x6000 | Rd | (Rn << 3)) | ((0x6000 | Rd | (Rn << 3)) << 16))

/* Load register (Thumb state)
 * Rd: destination register
 * Rn: base register
 */
#define ARMV7M_T_LDR(Rd, Rn)	((0x6800 | (Rn << 3) | Rd) | ((0x6800 | (Rn << 3) | Rd) << 16))

/* Load multiple (Thumb state)
 * Rn: base register
 * List: for each bit in list: store register
 */
#define ARMV7M_T_LDMIA(Rn, List) ((0xc800 | (Rn << 8) | List) | ((0xc800 | (Rn << 8) | List) << 16))
 
/* Load register with PC relative addressing
 * Rd: register to load
 */
#define ARMV7M_T_LDR_PCREL(Rd)	((0x4800 | (Rd << 8)) | ((0x4800 | (Rd << 8)) << 16)) 
 
/* Move hi register (Thumb mode)
 * Rd: destination register
 * Rm: source register
 */
#define ARMV7M_T_MOV(Rd, Rm)	((0x4600 | (Rd & 0x7) | ((Rd & 0x8) << 4) | ((Rm & 0x7) << 3) | ((Rm & 0x8) << 3)) | ((0x4600 | (Rd & 0x7) | ((Rd & 0x8) << 4) | ((Rm & 0x7) << 3) | ((Rm & 0x8) << 3)) << 16))

/* No operation (Thumb mode)
 */
#define ARMV7M_T_NOP	(0x46c0 | (0x46c0 << 16))

/* Move immediate to register (Thumb state)
 * Rd: destination register
 * Im: 8-bit immediate value
 */
#define ARMV7M_T_MOV_IM(Rd, Im)	((0x2000 | (Rd << 8) | Im) | ((0x2000 | (Rd << 8) | Im) << 16))

/* Branch and Exchange
 * Rm: register containing branch target
 */
#define ARMV7M_T_BX(Rm)		((0x4700 | (Rm << 3)) | ((0x4700 | (Rm << 3)) << 16))

/* Branch (Thumb state)
 * Imm: Branch target
 */
#define ARMV7M_T_B(Imm)	((0xe000 | Imm) | ((0xe000 | Imm) << 16))

#endif /* ARMV7M_H */
