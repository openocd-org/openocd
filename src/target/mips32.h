/***************************************************************************
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2008 by David T.L. Wong                                 *
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

#ifndef MIPS32_H
#define MIPS32_H

#include "target.h"
#include "register.h"
#include "mips_ejtag.h"
#include "mips32_pracc.h"

#define MIPS32_COMMON_MAGIC		0xB320B320

/* offsets into mips32 core register cache */
enum 
{
	MIPS32_PC = 37,
	MIPS32NUMCOREREGS
};

typedef struct mips32_comparator_s
{
	int used;
	//int type;
	u32 bp_value;
	u32 reg_address;
} mips32_comparator_t;

typedef struct mips32_common_s
{
	int common_magic;
	void *arch_info;
	reg_cache_t *core_cache;
	mips_ejtag_t ejtag_info;
	u32 core_regs[MIPS32NUMCOREREGS];
	
	int bp_scanned;
	int num_inst_bpoints;
	int num_data_bpoints;
	int num_inst_bpoints_avail;
	int num_data_bpoints_avail;
	mips32_comparator_t *inst_break_list;
	mips32_comparator_t *data_break_list;
	
	/* register cache to processor synchronization */
	int (*read_core_reg)(struct target_s *target, int num);
	int (*write_core_reg)(struct target_s *target, int num);
} mips32_common_t;

typedef struct mips32_core_reg_s
{
	u32 num;
	target_t *target;
	mips32_common_t *mips32_common;
} mips32_core_reg_t;

#define MIPS32_OP_BEQ	0x04
#define MIPS32_OP_ADDI	0x08
#define MIPS32_OP_AND	0x24
#define MIPS32_OP_COP0	0x10
#define MIPS32_OP_LUI	0x0F
#define MIPS32_OP_LW	0x23
#define MIPS32_OP_LBU	0x24
#define MIPS32_OP_LHU	0x25
#define MIPS32_OP_MFHI	0x10
#define MIPS32_OP_MTHI	0x11
#define MIPS32_OP_MFLO	0x12
#define MIPS32_OP_MTLO	0x13
#define MIPS32_OP_SB	0x28
#define MIPS32_OP_SH	0x29
#define MIPS32_OP_SW	0x2B
#define MIPS32_OP_ORI	0x0D

#define MIPS32_COP0_MF	0x00
#define MIPS32_COP0_MT	0x04

#define MIPS32_R_INST(opcode, rs, rt, rd, shamt, funct)	(((opcode)<<26) |((rs)<<21)|((rt)<<16)|((rd)<<11)| ((shamt)<<5) | (funct))
#define MIPS32_I_INST(opcode, rs, rt, immd)	(((opcode)<<26) |((rs)<<21)|((rt)<<16)|(immd))
#define MIPS32_J_INST(opcode, addr)	(((opcode)<<26) |(addr))

#define MIPS32_NOP					0
#define MIPS32_ADDI(tar, src, val)	MIPS32_I_INST(MIPS32_OP_ADDI, src, tar, val)
#define MIPS32_AND(reg, off, val)	MIPS32_R_INST(0, off, val, reg, 0, MIPS32_OP_AND)
#define MIPS32_B(off)				MIPS32_BEQ(0, 0, off)
#define MIPS32_BEQ(src,tar,off)		MIPS32_I_INST(MIPS32_OP_BEQ, src, tar, off)
#define MIPS32_MFC0(gpr, cpr, sel)	MIPS32_R_INST(MIPS32_OP_COP0, MIPS32_COP0_MF, gpr, cpr, 0, sel)
#define MIPS32_MTC0(gpr,cpr, sel)	MIPS32_R_INST(MIPS32_OP_COP0, MIPS32_COP0_MT, gpr, cpr, 0, sel)
#define MIPS32_LBU(reg, off, base)	MIPS32_I_INST(MIPS32_OP_LBU, base, reg, off)
#define MIPS32_LHU(reg, off, base)	MIPS32_I_INST(MIPS32_OP_LHU, base, reg, off)
#define MIPS32_LUI(reg, val)		MIPS32_I_INST(MIPS32_OP_LUI, 0, reg, val)
#define MIPS32_LW(reg, off, base)	MIPS32_I_INST(MIPS32_OP_LW, base, reg, off)
#define MIPS32_MFLO(reg)			MIPS32_R_INST(0, 0, 0, reg, 0, MIPS32_OP_MFLO)
#define MIPS32_MFHI(reg)			MIPS32_R_INST(0, 0, 0, reg, 0, MIPS32_OP_MFHI)
#define MIPS32_MTLO(reg)			MIPS32_R_INST(0, 0, 0, reg, 0, MIPS32_OP_MTLO)
#define MIPS32_MTHI(reg)			MIPS32_R_INST(0, 0, 0, reg, 0, MIPS32_OP_MTHI)
#define MIPS32_ORI(src, tar, val)	MIPS32_I_INST(MIPS32_OP_ORI, src, tar, val)
#define MIPS32_SB(reg, off, base)	MIPS32_I_INST(MIPS32_OP_SB, base, reg, off)
#define MIPS32_SH(reg, off, base)	MIPS32_I_INST(MIPS32_OP_SH, base, reg, off)
#define MIPS32_SW(reg, off, base)	MIPS32_I_INST(MIPS32_OP_SW, base, reg, off)

/* ejtag specific instructions */
#define MIPS32_DRET					0x4200001F
#define MIPS32_SDBBP				0x7000003F
#define MIPS16_SDBBP 				0xE801

extern int mips32_arch_state(struct target_s *target);
extern int mips32_init_arch_info(target_t *target, mips32_common_t *mips32, jtag_tap_t *tap);
extern int mips32_restore_context(target_t *target);
extern int mips32_save_context(target_t *target);
extern reg_cache_t *mips32_build_reg_cache(target_t *target);
extern int mips32_run_algorithm(struct target_s *target, int num_mem_params, mem_param_t *mem_params, int num_reg_params, reg_param_t *reg_params, u32 entry_point, u32 exit_point, int timeout_ms, void *arch_info);
extern int mips32_configure_break_unit(struct target_s *target);
extern int mips32_examine(struct target_s *target);

extern int mips32_register_commands(struct command_context_s *cmd_ctx);
extern int mips32_invalidate_core_regs(target_t *target);
extern int mips32_get_gdb_reg_list(target_t *target, reg_t **reg_list[], int *reg_list_size);

#endif	/*MIPS32_H*/
