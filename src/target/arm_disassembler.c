/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arm_disassembler.h"

#include "log.h"

#include <string.h>

/* textual represenation of the condition field */
/* ALways (default) is ommitted (empty string) */
char *arm_condition_strings[] =
{
	"EQ", "NE", "CS", "CC", "MI", "PL", "VS", "VC", "HI", "LS", "GE", "LT", "GT", "LE", "", "NV"
};

/* make up for C's missing ROR */
u32 ror(u32 value, int places) 
{ 
	return (value >> places) | (value << (32 - places)); 
}

int evaluate_pld(u32 opcode, u32 address, arm_instruction_t *instruction)
{
	/* PLD */
	if ((opcode & 0x0d70f0000) == 0x0550f000)
	{
		instruction->type = ARM_PLD;
		
		snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\tPLD ...TODO...", address, opcode);
		
		return ERROR_OK;
	}
	else
	{
		instruction->type = ARM_UNDEFINED_INSTRUCTION;
		return ERROR_OK;
	}
	
	LOG_ERROR("should never reach this point");
	return -1;
}

int evaluate_swi(u32 opcode, u32 address, arm_instruction_t *instruction)
{
	instruction->type = ARM_SWI;
	
	snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\tSWI 0x%6.6x", address, opcode, (opcode & 0xffffff));
	
	return ERROR_OK;
}

int evaluate_blx_imm(u32 opcode, u32 address, arm_instruction_t *instruction)
{
	int offset;
	u32 immediate;
	u32 target_address;
	
	instruction->type = ARM_BLX;
	immediate = opcode & 0x00ffffff;
	
	/* sign extend 24-bit immediate */
	if (immediate & 0x00800000)
		offset = 0xff000000 | immediate;
	else
		offset = immediate;
	
	/* shift two bits left */
	offset <<= 2;
	
	/* odd/event halfword */
	if (opcode & 0x01000000)
		offset |= 0x2;
	
	target_address = address + 8 + offset;
	
	snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\tBLX 0x%8.8x", address, opcode, target_address);
	
	instruction->info.b_bl_bx_blx.reg_operand = -1;
	instruction->info.b_bl_bx_blx.target_address = target_address;
	
	return ERROR_OK;
}

int evaluate_b_bl(u32 opcode, u32 address, arm_instruction_t *instruction)
{
	u8 L;
	u32 immediate;
	int offset;
	u32 target_address;
	
	immediate = opcode & 0x00ffffff;
	L = (opcode & 0x01000000) >> 24;
	
	/* sign extend 24-bit immediate */
	if (immediate & 0x00800000)
		offset = 0xff000000 | immediate;
	else
		offset = immediate;
	
	/* shift two bits left */
	offset <<= 2;
	
	target_address = address + 8 + offset;

	if (L)
		instruction->type = ARM_BL;
	else
		instruction->type = ARM_B;
	
	snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\tB%s%s 0x%8.8x", address, opcode,
			 (L) ? "L" : "", COND(opcode), target_address);
	
	instruction->info.b_bl_bx_blx.reg_operand = -1;
	instruction->info.b_bl_bx_blx.target_address = target_address;
	
	return ERROR_OK;
}

/* Coprocessor load/store and double register transfers */
/* both normal and extended instruction space (condition field b1111) */
int evaluate_ldc_stc_mcrr_mrrc(u32 opcode, u32 address, arm_instruction_t *instruction)
{
	u8 cp_num = (opcode & 0xf00) >> 8;
	
	/* MCRR or MRRC */
	if (((opcode & 0x0ff00000) == 0x0c400000) || ((opcode & 0x0ff00000) == 0x0c400000))
	{
		u8 cp_opcode, Rd, Rn, CRm;
		char *mnemonic;
		
		cp_opcode = (opcode & 0xf0) >> 4;
		Rd = (opcode & 0xf000) >> 12;
		Rn = (opcode & 0xf0000) >> 16;
		CRm = (opcode & 0xf);
		
		/* MCRR */
		if ((opcode & 0x0ff00000) == 0x0c400000)
		{
			instruction->type = ARM_MCRR;
			mnemonic = "MCRR";
		}
		
		/* MRRC */
		if ((opcode & 0x0ff00000) == 0x0c500000)
		{
			instruction->type = ARM_MRRC;
			mnemonic = "MRRC";
		}
		
		snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\t%s%s p%i, %x, r%i, r%i, c%i",
				 address, opcode, mnemonic, COND(opcode), cp_num, cp_opcode, Rd, Rn, CRm);
	}
	else /* LDC or STC */
	{
		u8 CRd, Rn, offset;
		u8 U, N;
		char *mnemonic;
		char addressing_mode[32];
		
		CRd = (opcode & 0xf000) >> 12;
		Rn = (opcode & 0xf0000) >> 16;
		offset = (opcode & 0xff);
		
		/* load/store */
		if (opcode & 0x00100000)
		{
			instruction->type = ARM_LDC;
			mnemonic = "LDC";
		}
		else
		{
			instruction->type = ARM_STC;
			mnemonic = "STC";
		}
		
		U = (opcode & 0x00800000) >> 23;
		N = (opcode & 0x00400000) >> 22;
		
		/* addressing modes */
		if ((opcode & 0x01200000) == 0x01000000) /* immediate offset */
			snprintf(addressing_mode, 32, "[r%i, #%s0x%2.2x*4]", Rn, (U) ? "" : "-", offset);
		else if ((opcode & 0x01200000) == 0x01200000) /* immediate pre-indexed */
			snprintf(addressing_mode, 32, "[r%i, #%s0x%2.2x*4]!", Rn, (U) ? "" : "-", offset);
		else if ((opcode & 0x01200000) == 0x00200000) /* immediate post-indexed */
			snprintf(addressing_mode, 32, "[r%i], #%s0x%2.2x*4", Rn, (U) ? "" : "-", offset);
		else if ((opcode & 0x01200000) == 0x00000000) /* unindexed */
			snprintf(addressing_mode, 32, "[r%i], #0x%2.2x", Rn, offset);

		snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\t%s%s%s p%i, c%i, %s",
				 address, opcode, mnemonic, ((opcode & 0xf0000000) == 0xf0000000) ? COND(opcode) : "2",
				 (N) ? "L" : "",
				 cp_num, CRd, addressing_mode);
	}
	
	return ERROR_OK;
}

/* Coprocessor data processing instructions */
/* Coprocessor register transfer instructions */
/* both normal and extended instruction space (condition field b1111) */
int evaluate_cdp_mcr_mrc(u32 opcode, u32 address, arm_instruction_t *instruction)
{
	char* cond;
	char* mnemonic;
	u8 cp_num, opcode_1, CRd_Rd, CRn, CRm, opcode_2;
	
	cond = ((opcode & 0xf0000000) == 0xf0000000) ? "2" : COND(opcode);
	cp_num = (opcode & 0xf00) >> 8;
	CRd_Rd = (opcode & 0xf000) >> 12;
	CRn = (opcode & 0xf0000) >> 16;
	CRm = (opcode & 0xf);
	opcode_2 = (opcode & 0xe0) >> 5;
	
	/* CDP or MRC/MCR */
	if (opcode & 0x00000010) /* bit 4 set -> MRC/MCR */
	{
		if (opcode & 0x00100000) /* bit 20 set -> MRC */
		{
			instruction->type = ARM_MRC;
			mnemonic = "MRC";
		}
		else /* bit 20 not set -> MCR */
		{
			instruction->type = ARM_MCR;
			mnemonic = "MCR";
		}
		
		opcode_1 = (opcode & 0x00e00000) >> 21;
		
		snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\t%s%s p%i, 0x%2.2x, r%i, c%i, c%i, 0x%2.2x",
				 address, opcode, mnemonic, cond,
				 cp_num, opcode_1, CRd_Rd, CRn, CRm, opcode_2);
	}
	else /* bit 4 not set -> CDP */
	{
		instruction->type = ARM_CDP;
		mnemonic = "CDP";
		
		opcode_1 = (opcode & 0x00f00000) >> 20;
		
		snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\t%s%s p%i, 0x%2.2x, c%i, c%i, c%i, 0x%2.2x",
				 address, opcode, mnemonic, cond,
				 cp_num, opcode_1, CRd_Rd, CRn, CRm, opcode_2);
	}
	
	return ERROR_OK;
}

/* Load/store instructions */
int evaluate_load_store(u32 opcode, u32 address, arm_instruction_t *instruction)
{
	u8 I, P, U, B, W, L;
	u8 Rn, Rd;
	char *operation; /* "LDR" or "STR" */
	char *suffix; /* "", "B", "T", "BT" */
	char offset[32];
	
	/* examine flags */
	I = (opcode & 0x02000000) >> 25;
	P = (opcode & 0x01000000) >> 24;
	U = (opcode & 0x00800000) >> 23;
	B = (opcode & 0x00400000) >> 22;
	W = (opcode & 0x00200000) >> 21;
	L = (opcode & 0x00100000) >> 20;
	
	/* target register */
	Rd = (opcode & 0xf000) >> 12;
	
	/* base register */
	Rn = (opcode & 0xf0000) >> 16;
	
	instruction->info.load_store.Rd = Rd;
	instruction->info.load_store.Rn = Rn;
	instruction->info.load_store.U = U;

	/* determine operation */
	if (L)
		operation = "LDR";
	else
		operation = "STR";
	
	/* determine instruction type and suffix */
	if (B)
	{
		if ((P == 0) && (W == 1))
		{
			if (L)
				instruction->type = ARM_LDRBT;
			else
				instruction->type = ARM_STRBT;
			suffix = "BT";
		}
		else
		{
			if (L)
				instruction->type = ARM_LDRB;
			else
				instruction->type = ARM_STRB;
			suffix = "B";
		}
	}
	else
	{
		if ((P == 0) && (W == 1))
		{
			if (L)
				instruction->type = ARM_LDRT;
			else
				instruction->type = ARM_STRT;
			suffix = "T";
		}
		else
		{
			if (L)
				instruction->type = ARM_LDR;
			else
				instruction->type = ARM_STR;
			suffix = "";
		}
	}
	
	if (!I) /* #+-<offset_12> */
	{
		u32 offset_12 = (opcode & 0xfff);
		if (offset_12)
			snprintf(offset, 32, ", #%s0x%x", (U) ? "" : "-", offset_12);
		else
			snprintf(offset, 32, "%s", "");
		
		instruction->info.load_store.offset_mode = 0;
		instruction->info.load_store.offset.offset = offset_12;
	}
	else /* either +-<Rm> or +-<Rm>, <shift>, #<shift_imm> */
	{
		u8 shift_imm, shift;
		u8 Rm;
		
		shift_imm = (opcode & 0xf80) >> 7;
		shift = (opcode & 0x60) >> 5;
		Rm = (opcode & 0xf);
		
		/* LSR encodes a shift by 32 bit as 0x0 */
		if ((shift == 0x1) && (shift_imm == 0x0))
			shift_imm = 0x20;
		
		/* ASR encodes a shift by 32 bit as 0x0 */
		if ((shift == 0x2) && (shift_imm == 0x0))
			shift_imm = 0x20;

		/* ROR by 32 bit is actually a RRX */
		if ((shift == 0x3) && (shift_imm == 0x0))
			shift = 0x4;
		
		instruction->info.load_store.offset_mode = 1;
		instruction->info.load_store.offset.reg.Rm = Rm;
		instruction->info.load_store.offset.reg.shift = shift;
		instruction->info.load_store.offset.reg.shift_imm = shift_imm;

		if ((shift_imm == 0x0) && (shift == 0x0)) /* +-<Rm> */
		{
			snprintf(offset, 32, ", %sr%i", (U) ? "" : "-", Rm);
		}
		else /* +-<Rm>, <Shift>, #<shift_imm> */
		{
			switch (shift)
			{
				case 0x0: /* LSL */
					snprintf(offset, 32, ", %sr%i, LSL #0x%x", (U) ? "" : "-", Rm, shift_imm);
					break;
				case 0x1: /* LSR */
					snprintf(offset, 32, ", %sr%i, LSR #0x%x", (U) ? "" : "-", Rm, shift_imm);
					break;
				case 0x2: /* ASR */
					snprintf(offset, 32, ", %sr%i, ASR #0x%x", (U) ? "" : "-", Rm, shift_imm);
					break;
				case 0x3: /* ROR */
					snprintf(offset, 32, ", %sr%i, ROR #0x%x", (U) ? "" : "-", Rm, shift_imm);
					break;
				case 0x4: /* RRX */
					snprintf(offset, 32, ", %sr%i, RRX", (U) ? "" : "-", Rm);
					break;
			}
		}
	}
	
	if (P == 1)
	{
		if (W == 0) /* offset */
		{
			snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\t%s%s%s r%i, [r%i%s]",
					 address, opcode, operation, COND(opcode), suffix,
					 Rd, Rn, offset);
			
			instruction->info.load_store.index_mode = 0;
		}
		else /* pre-indexed */
		{
			snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\t%s%s%s r%i, [r%i%s]!",
					 address, opcode, operation, COND(opcode), suffix,
					 Rd, Rn, offset);
			
			instruction->info.load_store.index_mode = 1;
		}
	}
	else /* post-indexed */
	{
		snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\t%s%s%s r%i, [r%i]%s",
				 address, opcode, operation, COND(opcode), suffix,
				 Rd, Rn, offset);
		
		instruction->info.load_store.index_mode = 2;
	}
	
	return ERROR_OK;
}

/* Miscellaneous load/store instructions */
int evaluate_misc_load_store(u32 opcode, u32 address, arm_instruction_t *instruction)
{
	u8 P, U, I, W, L, S, H;
	u8 Rn, Rd;
	char *operation; /* "LDR" or "STR" */
	char *suffix; /* "H", "SB", "SH", "D" */
	char offset[32];
	
	/* examine flags */
	P = (opcode & 0x01000000) >> 24;
	U = (opcode & 0x00800000) >> 23;
	I = (opcode & 0x00400000) >> 22;
	W = (opcode & 0x00200000) >> 21;
	L = (opcode & 0x00100000) >> 20;
	S = (opcode & 0x00000040) >> 6;
	H = (opcode & 0x00000020) >> 5;
	
	/* target register */
	Rd = (opcode & 0xf000) >> 12;
	
	/* base register */
	Rn = (opcode & 0xf0000) >> 16;
	
	instruction->info.load_store.Rd = Rd;
	instruction->info.load_store.Rn = Rn;
	instruction->info.load_store.U = U;
	
	/* determine instruction type and suffix */
	if (S) /* signed */
	{
		if (L) /* load */
		{
			if (H)
			{
				operation = "LDR";
				instruction->type = ARM_LDRSH;
				suffix = "SH";
			}
			else
			{
				operation = "LDR";
				instruction->type = ARM_LDRSB;
				suffix = "SB";
			}
		}
		else /* there are no signed stores, so this is used to encode double-register load/stores */
		{
			suffix = "D";
			if (H)
			{
				operation = "STR";
				instruction->type = ARM_STRD;
			}
			else
			{
				operation = "LDR";
				instruction->type = ARM_LDRD;
			}
		}
	}
	else /* unsigned */
	{
		suffix = "H";
		if (L) /* load */
		{
			operation = "LDR";
			instruction->type = ARM_LDRH;
		}
		else /* store */
		{
			operation = "STR";
			instruction->type = ARM_STRH;
		}
	}
	
	if (I) /* Immediate offset/index (#+-<offset_8>)*/
	{
		u32 offset_8 = ((opcode & 0xf00) >> 4) | (opcode & 0xf);
		snprintf(offset, 32, "#%s0x%x", (U) ? "" : "-", offset_8);
		
		instruction->info.load_store.offset_mode = 0;
		instruction->info.load_store.offset.offset = offset_8;
	}
	else /* Register offset/index (+-<Rm>) */
	{
		u8 Rm;
		Rm = (opcode & 0xf);
		snprintf(offset, 32, "%sr%i", (U) ? "" : "-", Rm);
		
		instruction->info.load_store.offset_mode = 1;
		instruction->info.load_store.offset.reg.Rm = Rm;
		instruction->info.load_store.offset.reg.shift = 0x0;
		instruction->info.load_store.offset.reg.shift_imm = 0x0;
	}
	
	if (P == 1)
	{
		if (W == 0) /* offset */
		{
			snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\t%s%s%s r%i, [r%i, %s]",
					 address, opcode, operation, COND(opcode), suffix,
					 Rd, Rn, offset);
			
			instruction->info.load_store.index_mode = 0;
		}
		else /* pre-indexed */
		{
			snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\t%s%s%s r%i, [r%i, %s]!",
					 address, opcode, operation, COND(opcode), suffix,
					 Rd, Rn, offset);
		
			instruction->info.load_store.index_mode = 1;
		}
	}
	else /* post-indexed */
	{
		snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\t%s%s%s r%i, [r%i], %s",
				 address, opcode, operation, COND(opcode), suffix,
				 Rd, Rn, offset);
	
		instruction->info.load_store.index_mode = 2;
	}
	
	return ERROR_OK;
}

/* Load/store multiples instructions */
int evaluate_ldm_stm(u32 opcode, u32 address, arm_instruction_t *instruction)
{
	u8 P, U, S, W, L, Rn;
	u32 register_list;
	char *addressing_mode;
	char *mnemonic;
	char reg_list[69];
	char *reg_list_p;
	int i;
	int first_reg = 1;
	
	P = (opcode & 0x01000000) >> 24;
	U = (opcode & 0x00800000) >> 23;
	S = (opcode & 0x00400000) >> 22;
	W = (opcode & 0x00200000) >> 21;
	L = (opcode & 0x00100000) >> 20;
	register_list = (opcode & 0xffff);
	Rn = (opcode & 0xf0000) >> 16;
	
	instruction->info.load_store_multiple.Rn = Rn;
	instruction->info.load_store_multiple.register_list = register_list;
	instruction->info.load_store_multiple.S = S;
	instruction->info.load_store_multiple.W = W;
	
	if (L)
	{
		instruction->type = ARM_LDM;
		mnemonic = "LDM";
	}
	else
	{
		instruction->type = ARM_STM;
		mnemonic = "STM";
	}
	
	if (P)
	{
		if (U)
		{
			instruction->info.load_store_multiple.addressing_mode = 1;
			addressing_mode = "IB";
		}
		else
		{
			instruction->info.load_store_multiple.addressing_mode = 3;
			addressing_mode = "DB";
		}
	}
	else
	{
		if (U)
		{
			instruction->info.load_store_multiple.addressing_mode = 0;
			addressing_mode = "IA";
		}
		else
		{
			instruction->info.load_store_multiple.addressing_mode = 2;
			addressing_mode = "DA";
		}
	}
	
	reg_list_p = reg_list;
	for (i = 0; i <= 15; i++)
	{
		if ((register_list >> i) & 1)
		{
			if (first_reg)
			{
				first_reg = 0;
				reg_list_p += snprintf(reg_list_p, (reg_list + 69 - reg_list_p), "r%i", i);
			}
			else
			{
				reg_list_p += snprintf(reg_list_p, (reg_list + 69 - reg_list_p), ", r%i", i);
			}
		}
	}
	
	snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\t%s%s%s r%i%s, {%s}%s",
			 address, opcode, mnemonic, COND(opcode), addressing_mode,
			 Rn, (W) ? "!" : "", reg_list, (S) ? "^" : "");
	
	return ERROR_OK;
}

/* Multiplies, extra load/stores */
int evaluate_mul_and_extra_ld_st(u32 opcode, u32 address, arm_instruction_t *instruction)
{
	/* Multiply (accumulate) (long) and Swap/swap byte */
	if ((opcode & 0x000000f0) == 0x00000090)
	{
		/* Multiply (accumulate) */
		if ((opcode & 0x0f800000) == 0x00000000)
		{
			u8 Rm, Rs, Rn, Rd, S;
			Rm = opcode & 0xf;
			Rs = (opcode & 0xf00) >> 8;
			Rn = (opcode & 0xf000) >> 12;
			Rd = (opcode & 0xf0000) >> 16;
			S = (opcode & 0x00100000) >> 20;
			
			/* examine A bit (accumulate) */
			if (opcode & 0x00200000)
			{
				instruction->type = ARM_MLA;
				snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\tMLA%s%s r%i, r%i, r%i, r%i",
						address, opcode, COND(opcode), (S) ? "S" : "", Rd, Rm, Rs, Rn);
			}
			else
			{
				instruction->type = ARM_MUL;
				snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\tMUL%s%s r%i, r%i, r%i",
						 address, opcode, COND(opcode), (S) ? "S" : "", Rd, Rm, Rs);
			}
			
			return ERROR_OK;
		}
		
		/* Multiply (accumulate) long */
		if ((opcode & 0x0f800000) == 0x00800000)
		{
			char* mnemonic = NULL;
			u8 Rm, Rs, RdHi, RdLow, S;
			Rm = opcode & 0xf;
			Rs = (opcode & 0xf00) >> 8;
			RdHi = (opcode & 0xf000) >> 12;
			RdLow = (opcode & 0xf0000) >> 16;
			S = (opcode & 0x00100000) >> 20;
			
			switch ((opcode & 0x00600000) >> 21)
			{
				case 0x0:
					instruction->type = ARM_UMULL;
					mnemonic = "UMULL";
					break;
				case 0x1:
					instruction->type = ARM_UMLAL;
					mnemonic = "UMLAL";
					break;
				case 0x2:
					instruction->type = ARM_SMULL;
					mnemonic = "SMULL";
					break;
				case 0x3:
					instruction->type = ARM_SMLAL;
					mnemonic = "SMLAL";
					break;
			}
			
			snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\t%s%s%s r%i, r%i, r%i, r%i",
						address, opcode, mnemonic, COND(opcode), (S) ? "S" : "",
						RdLow, RdHi, Rm, Rs);
			
			return ERROR_OK;
		}
		
		/* Swap/swap byte */
		if ((opcode & 0x0f800000) == 0x01000000)
		{
			u8 Rm, Rd, Rn;
			Rm = opcode & 0xf;
			Rd = (opcode & 0xf000) >> 12;
			Rn = (opcode & 0xf0000) >> 16;
			
			/* examine B flag */
			instruction->type = (opcode & 0x00400000) ? ARM_SWPB : ARM_SWP;
			
			snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\t%s%s r%i, r%i, [r%i]",
					 address, opcode, (opcode & 0x00400000) ? "SWPB" : "SWP", COND(opcode), Rd, Rm, Rn);
			return ERROR_OK;
		}
		
	}
	
	return evaluate_misc_load_store(opcode, address, instruction);
}

int evaluate_mrs_msr(u32 opcode, u32 address, arm_instruction_t *instruction)
{
	int R = (opcode & 0x00400000) >> 22;
	char *PSR = (R) ? "SPSR" : "CPSR";
		
	/* Move register to status register (MSR) */
	if (opcode & 0x00200000)
	{
		instruction->type = ARM_MSR;
			
		/* immediate variant */
		if (opcode & 0x02000000)
		{
			u8 immediate = (opcode & 0xff);
			u8 rotate = (opcode & 0xf00);
			
			snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\tMSR%s %s_%s%s%s%s, 0x%8.8x",
					 address, opcode, COND(opcode), PSR,
					 (opcode & 0x10000) ? "c" : "",
					 (opcode & 0x20000) ? "x" : "",
					 (opcode & 0x40000) ? "s" : "",
					 (opcode & 0x80000) ? "f" : "",
					 ror(immediate, (rotate * 2))
					);
		}
		else /* register variant */
		{
			u8 Rm = opcode & 0xf;
			snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\tMSR%s %s_%s%s%s%s, r%i",
					 address, opcode, COND(opcode), PSR,
					 (opcode & 0x10000) ? "c" : "",
					 (opcode & 0x20000) ? "x" : "",
					 (opcode & 0x40000) ? "s" : "",
					 (opcode & 0x80000) ? "f" : "",
					 Rm
					);
		}
		
	}
	else /* Move status register to register (MRS) */
	{
		u8 Rd;
		
		instruction->type = ARM_MRS;
		Rd = (opcode & 0x0000f000) >> 12;
			
		snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\tMRS%s r%i, %s",
				 address, opcode, COND(opcode), Rd, PSR);
	}
	
	return ERROR_OK;
}

/* Miscellaneous instructions */
int evaluate_misc_instr(u32 opcode, u32 address, arm_instruction_t *instruction)
{
	/* MRS/MSR */
	if ((opcode & 0x000000f0) == 0x00000000)
	{
		evaluate_mrs_msr(opcode, address, instruction);
	}
	
	/* BX */
	if ((opcode & 0x006000f0) == 0x00200010)
	{
		u8 Rm;
		instruction->type = ARM_BX;
		Rm = opcode & 0xf;
		
		snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\tBX%s r%i",
				 address, opcode, COND(opcode), Rm);
		
		instruction->info.b_bl_bx_blx.reg_operand = Rm;
		instruction->info.b_bl_bx_blx.target_address = -1;
	}
	
	/* CLZ */
	if ((opcode & 0x006000f0) == 0x00600010)
	{
		u8 Rm, Rd;
		instruction->type = ARM_CLZ;
		Rm = opcode & 0xf;
		Rd = (opcode & 0xf000) >> 12;

		snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\tCLZ%s r%i, r%i",
				 address, opcode, COND(opcode), Rd, Rm);
	}
	
	/* BLX(2) */
	if ((opcode & 0x006000f0) == 0x00200030)
	{
		u8 Rm;
		instruction->type = ARM_BLX;
		Rm = opcode & 0xf;
		
		snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\tBLX%s r%i",
				 address, opcode, COND(opcode), Rm);
				 
		instruction->info.b_bl_bx_blx.reg_operand = Rm;
		instruction->info.b_bl_bx_blx.target_address = -1;
	}
	
	/* Enhanced DSP add/subtracts */
	if ((opcode & 0x0000000f0) == 0x00000050)
	{
		u8 Rm, Rd, Rn;
		char *mnemonic = NULL;
		Rm = opcode & 0xf;
		Rd = (opcode & 0xf000) >> 12;
		Rn = (opcode & 0xf0000) >> 16;
		
		switch ((opcode & 0x00600000) >> 21)
		{
			case 0x0:
				instruction->type = ARM_QADD;
				mnemonic = "QADD";
				break;
			case 0x1:
				instruction->type = ARM_QSUB;
				mnemonic = "QSUB";
				break;
			case 0x2:
				instruction->type = ARM_QDADD;
				mnemonic = "QDADD";
				break;
			case 0x3:
				instruction->type = ARM_QDSUB;
				mnemonic = "QDSUB";
				break;
		}
		
		snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\t%s%s r%i, r%i, r%i",
				 address, opcode, mnemonic, COND(opcode), Rd, Rm, Rn);
	}
	
	/* Software breakpoints */
	if ((opcode & 0x0000000f0) == 0x00000070)
	{
		u32 immediate;
		instruction->type = ARM_BKPT;
		immediate = ((opcode & 0x000fff00) >> 4) | (opcode & 0xf);
		
		snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\tBKPT 0x%4.4x",
				 address, opcode, immediate);	
	}
	
	/* Enhanced DSP multiplies */
	if ((opcode & 0x000000090) == 0x00000080)
	{
		int x = (opcode & 0x20) >> 5;
		int y = (opcode & 0x40) >> 6;
		
		/* SMLA<x><y> */
		if ((opcode & 0x00600000) == 0x00000000)
		{
			u8 Rd, Rm, Rs, Rn;
			instruction->type = ARM_SMLAxy;
			Rd = (opcode & 0xf0000) >> 16;
			Rm = (opcode & 0xf);
			Rs = (opcode & 0xf00) >> 8;
			Rn = (opcode & 0xf000) >> 12;
			
			snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\tSMLA%s%s%s r%i, r%i, r%i, r%i",
					 address, opcode, (x) ? "T" : "B", (y) ? "T" : "B", COND(opcode),
					 Rd, Rm, Rs, Rn);
		}
		
		/* SMLAL<x><y> */
		if ((opcode & 0x00600000) == 0x00400000)
		{
			u8 RdLow, RdHi, Rm, Rs;
			instruction->type = ARM_SMLAxy;
			RdHi = (opcode & 0xf0000) >> 16;
			RdLow = (opcode & 0xf000) >> 12;
			Rm = (opcode & 0xf);
			Rs = (opcode & 0xf00) >> 8;
			
			snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\tSMLA%s%s%s r%i, r%i, r%i, r%i",
					 address, opcode, (x) ? "T" : "B", (y) ? "T" : "B", COND(opcode),
					 RdLow, RdHi, Rm, Rs);
		}
		
		/* SMLAW<y> */
		if (((opcode & 0x00600000) == 0x00100000) && (x == 0))
		{
			u8 Rd, Rm, Rs, Rn;
			instruction->type = ARM_SMLAWy;
			Rd = (opcode & 0xf0000) >> 16;
			Rm = (opcode & 0xf);
			Rs = (opcode & 0xf00) >> 8;
			Rn = (opcode & 0xf000) >> 12;

			snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\tSMLAW%s%s r%i, r%i, r%i, r%i",
					 address, opcode, (y) ? "T" : "B", COND(opcode),
					 Rd, Rm, Rs, Rn);
		}
		
		/* SMUL<x><y> */
		if ((opcode & 0x00600000) == 0x00300000)
		{
			u8 Rd, Rm, Rs;
			instruction->type = ARM_SMULxy;
			Rd = (opcode & 0xf0000) >> 16;
			Rm = (opcode & 0xf);
			Rs = (opcode & 0xf00) >> 8;
			
			snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\tSMULW%s%s%s r%i, r%i, r%i",
					 address, opcode, (x) ? "T" : "B", (y) ? "T" : "B", COND(opcode),
					 Rd, Rm, Rs);
		}
		
		/* SMULW<y> */
		if (((opcode & 0x00600000) == 0x00100000) && (x == 1))
		{
			u8 Rd, Rm, Rs;
			instruction->type = ARM_SMULWy;
			Rd = (opcode & 0xf0000) >> 16;
			Rm = (opcode & 0xf);
			Rs = (opcode & 0xf00) >> 8;
			
			snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\tSMULW%s%s r%i, r%i, r%i",
					 address, opcode, (y) ? "T" : "B", COND(opcode),
					 Rd, Rm, Rs);
		}
	}
	
	return ERROR_OK;
}

int evaluate_data_proc(u32 opcode, u32 address, arm_instruction_t *instruction)
{
	u8 I, op, S, Rn, Rd;
	char *mnemonic = NULL;
	char shifter_operand[32];
	
	I = (opcode & 0x02000000) >> 25;
	op = (opcode & 0x01e00000) >> 21;
	S = (opcode & 0x00100000) >> 20;
	
	Rd = (opcode & 0xf000) >> 12;
	Rn = (opcode & 0xf0000) >> 16;
	
	instruction->info.data_proc.Rd = Rd;
	instruction->info.data_proc.Rn = Rn;
	instruction->info.data_proc.S = S;

	switch (op)
	{
		case 0x0:
			instruction->type = ARM_AND;
			mnemonic = "AND";
			break;
		case 0x1:
			instruction->type = ARM_EOR;
			mnemonic = "EOR";
			break;
		case 0x2:
			instruction->type = ARM_SUB;
			mnemonic = "SUB";
			break;
		case 0x3:
			instruction->type = ARM_RSB;
			mnemonic = "RSB";
			break;
		case 0x4:
			instruction->type = ARM_ADD;
			mnemonic = "ADD";
			break;
		case 0x5:
			instruction->type = ARM_ADC;
			mnemonic = "ADC";
			break;
		case 0x6:
			instruction->type = ARM_SBC;
			mnemonic = "SBC";
			break;
		case 0x7:
			instruction->type = ARM_RSC;
			mnemonic = "RSC";
			break;
		case 0x8:
			instruction->type = ARM_TST;
			mnemonic = "TST";
			break;
		case 0x9:
			instruction->type = ARM_TEQ;
			mnemonic = "TEQ";
			break;
		case 0xa:
			instruction->type = ARM_CMP;
			mnemonic = "CMP";
			break;
		case 0xb:
			instruction->type = ARM_CMN;
			mnemonic = "CMN";
			break;
		case 0xc:
			instruction->type = ARM_ORR;
			mnemonic = "ORR";
			break;
		case 0xd:
			instruction->type = ARM_MOV;
			mnemonic = "MOV";
			break;
		case 0xe:
			instruction->type = ARM_BIC;
			mnemonic = "BIC";
			break;
		case 0xf:
			instruction->type = ARM_MVN;
			mnemonic = "MVN";
			break;
	}
	
	if (I) /* immediate shifter operand (#<immediate>)*/
	{
		u8 immed_8 = opcode & 0xff;
		u8 rotate_imm = (opcode & 0xf00) >> 8;
		u32 immediate;
		
		immediate = ror(immed_8, rotate_imm * 2);
		
		snprintf(shifter_operand, 32, "#0x%x", immediate);
		
		instruction->info.data_proc.variant = 0;
		instruction->info.data_proc.shifter_operand.immediate.immediate = immediate;
	}
	else /* register-based shifter operand */
	{
		u8 shift, Rm;
		shift = (opcode & 0x60) >> 5;
		Rm = (opcode & 0xf);
		
		if ((opcode & 0x10) != 0x10) /* Immediate shifts ("<Rm>" or "<Rm>, <shift> #<shift_immediate>") */
		{
			u8 shift_imm;
			shift_imm = (opcode & 0xf80) >> 7;

			instruction->info.data_proc.variant = 1;
			instruction->info.data_proc.shifter_operand.immediate_shift.Rm = Rm;
			instruction->info.data_proc.shifter_operand.immediate_shift.shift_imm = shift_imm;
			instruction->info.data_proc.shifter_operand.immediate_shift.shift = shift;
		
			/* LSR encodes a shift by 32 bit as 0x0 */
			if ((shift == 0x1) && (shift_imm == 0x0))
				shift_imm = 0x20;
		
			/* ASR encodes a shift by 32 bit as 0x0 */
			if ((shift == 0x2) && (shift_imm == 0x0))
				shift_imm = 0x20;

			/* ROR by 32 bit is actually a RRX */
			if ((shift == 0x3) && (shift_imm == 0x0))
				shift = 0x4;
			
			if ((shift_imm == 0x0) && (shift == 0x0))
			{
				snprintf(shifter_operand, 32, "r%i", Rm);
			}
			else
			{
				if (shift == 0x0) /* LSL */
				{
					snprintf(shifter_operand, 32, "r%i, LSL #0x%x", Rm, shift_imm);
				}
				else if (shift == 0x1) /* LSR */
				{
					snprintf(shifter_operand, 32, "r%i, LSR #0x%x", Rm, shift_imm);
				}
				else if (shift == 0x2) /* ASR */
				{
					snprintf(shifter_operand, 32, "r%i, ASR #0x%x", Rm, shift_imm);
				}
				else if (shift == 0x3) /* ROR */
				{
					snprintf(shifter_operand, 32, "r%i, ROR #0x%x", Rm, shift_imm);
				}
				else if (shift == 0x4) /* RRX */
				{
					snprintf(shifter_operand, 32, "r%i, RRX", Rm);
				}
			}
		}
		else /* Register shifts ("<Rm>, <shift> <Rs>") */
		{
			u8 Rs = (opcode & 0xf00) >> 8;
			
			instruction->info.data_proc.variant = 2;
			instruction->info.data_proc.shifter_operand.register_shift.Rm = Rm;
			instruction->info.data_proc.shifter_operand.register_shift.Rs = Rs;
			instruction->info.data_proc.shifter_operand.register_shift.shift = shift;
			
			if (shift == 0x0) /* LSL */
			{
				snprintf(shifter_operand, 32, "r%i, LSL r%i", Rm, Rs);
			}
			else if (shift == 0x1) /* LSR */
			{
				snprintf(shifter_operand, 32, "r%i, LSR r%i", Rm, Rs);
			}
			else if (shift == 0x2) /* ASR */
			{
				snprintf(shifter_operand, 32, "r%i, ASR r%i", Rm, Rs);
			}
			else if (shift == 0x3) /* ROR */
			{
				snprintf(shifter_operand, 32, "r%i, ROR r%i", Rm, Rs);
			}
		}
	}
	
	if ((op < 0x8) || (op == 0xc) || (op == 0xe)) /* <opcode3>{<cond>}{S} <Rd>, <Rn>, <shifter_operand> */
	{
		snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\t%s%s%s r%i, r%i, %s",
				 address, opcode, mnemonic, COND(opcode),
				 (S) ? "S" : "", Rd, Rn, shifter_operand);
	}
	else if ((op == 0xd) || (op == 0xf)) /* <opcode1>{<cond>}{S} <Rd>, <shifter_operand> */
	{
		if (opcode==0xe1a00000) /* print MOV r0,r0 as NOP */
			snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\tNOP",address, opcode);
		else
			snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\t%s%s%s r%i, %s",
				 address, opcode, mnemonic, COND(opcode),
				 (S) ? "S" : "", Rd, shifter_operand);
	}
	else /* <opcode2>{<cond>} <Rn>, <shifter_operand> */
	{
		snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\t%s%s r%i, %s",
				 address, opcode, mnemonic, COND(opcode),
				 Rn, shifter_operand);
	}
	
	return ERROR_OK;
}
		
int arm_evaluate_opcode(u32 opcode, u32 address, arm_instruction_t *instruction)
{
	/* clear fields, to avoid confusion */
	memset(instruction, 0, sizeof(arm_instruction_t));
	instruction->opcode = opcode;
	
	/* catch opcodes with condition field [31:28] = b1111 */
	if ((opcode & 0xf0000000) == 0xf0000000)
	{
		/* Undefined instruction (or ARMv5E cache preload PLD) */
		if ((opcode & 0x08000000) == 0x00000000)
			return evaluate_pld(opcode, address, instruction);
		
		/* Undefined instruction */
		if ((opcode & 0x0e000000) == 0x08000000)
		{
			instruction->type = ARM_UNDEFINED_INSTRUCTION;
			snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\tUNDEFINED INSTRUCTION", address, opcode);
			return ERROR_OK;
		}
		
		/* Branch and branch with link and change to Thumb */
		if ((opcode & 0x0e000000) == 0x0a000000)
			return evaluate_blx_imm(opcode, address, instruction);
		
		/* Extended coprocessor opcode space (ARMv5 and higher )*/
		/* Coprocessor load/store and double register transfers */
		if ((opcode & 0x0e000000) == 0x0c000000)
			return evaluate_ldc_stc_mcrr_mrrc(opcode, address, instruction);
		
		/* Coprocessor data processing */
		if ((opcode & 0x0f000100) == 0x0c000000)
			return evaluate_cdp_mcr_mrc(opcode, address, instruction);
		
		/* Coprocessor register transfers */
		if ((opcode & 0x0f000010) == 0x0c000010)
			return evaluate_cdp_mcr_mrc(opcode, address, instruction);
		
		/* Undefined instruction */
		if ((opcode & 0x0f000000) == 0x0f000000)
		{
			instruction->type = ARM_UNDEFINED_INSTRUCTION;
			snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\tUNDEFINED INSTRUCTION", address, opcode);
			return ERROR_OK;
		}
	}
	
	/* catch opcodes with [27:25] = b000 */
	if ((opcode & 0x0e000000) == 0x00000000)
	{
		/* Multiplies, extra load/stores */
		if ((opcode & 0x00000090) == 0x00000090)
			return evaluate_mul_and_extra_ld_st(opcode, address, instruction);
		
		/* Miscellaneous instructions */
		if ((opcode & 0x0f900000) == 0x01000000)
			return evaluate_misc_instr(opcode, address, instruction);
		
		return evaluate_data_proc(opcode, address, instruction);
	}
	
	/* catch opcodes with [27:25] = b001 */
	if ((opcode & 0x0e000000) == 0x02000000)
	{
		/* Undefined instruction */
		if ((opcode & 0x0fb00000) == 0x03000000)
		{
			instruction->type = ARM_UNDEFINED_INSTRUCTION;
			snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\tUNDEFINED INSTRUCTION", address, opcode);
			return ERROR_OK;
		}
				
		/* Move immediate to status register */
		if ((opcode & 0x0fb00000) == 0x03200000)
			return evaluate_mrs_msr(opcode, address, instruction);
		
		return evaluate_data_proc(opcode, address, instruction);

	}
	
	/* catch opcodes with [27:25] = b010 */
	if ((opcode & 0x0e000000) == 0x04000000)
	{
		/* Load/store immediate offset */
		return evaluate_load_store(opcode, address, instruction);
	}
	
	/* catch opcodes with [27:25] = b011 */
	if ((opcode & 0x0e000000) == 0x06000000)
	{
		/* Undefined instruction */
		if ((opcode & 0x00000010) == 0x00000010)
		{
			instruction->type = ARM_UNDEFINED_INSTRUCTION;
			snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\tUNDEFINED INSTRUCTION", address, opcode);
			return ERROR_OK;
		}
		
		/* Load/store register offset */
		return evaluate_load_store(opcode, address, instruction);

	}
	
	/* catch opcodes with [27:25] = b100 */
	if ((opcode & 0x0e000000) == 0x08000000)
	{
		/* Load/store multiple */
		return evaluate_ldm_stm(opcode, address, instruction);
	}
	
	/* catch opcodes with [27:25] = b101 */
	if ((opcode & 0x0e000000) == 0x0a000000)
	{
		/* Branch and branch with link */
		return evaluate_b_bl(opcode, address, instruction);
	}
	
	/* catch opcodes with [27:25] = b110 */
	if ((opcode & 0x0e000000) == 0x0a000000)
	{
		/* Coprocessor load/store and double register transfers */
		return evaluate_ldc_stc_mcrr_mrrc(opcode, address, instruction);
	}
	
	/* catch opcodes with [27:25] = b111 */
	if ((opcode & 0x0e000000) == 0x0e000000)
	{
		/* Software interrupt */
		if ((opcode & 0x0f000000) == 0x0f000000)
			return evaluate_swi(opcode, address, instruction);
		
		/* Coprocessor data processing */
		if ((opcode & 0x0f000010) == 0x0e000000)
			return evaluate_cdp_mcr_mrc(opcode, address, instruction);
		
		/* Coprocessor register transfers */
		if ((opcode & 0x0f000010) == 0x0e000010)
			return evaluate_cdp_mcr_mrc(opcode, address, instruction);
	}
	
	LOG_ERROR("should never reach this point");
	return -1;
}

int evaluate_b_bl_blx_thumb(u16 opcode, u32 address, arm_instruction_t *instruction)
{
	u32 offset = opcode & 0x7ff;
	u32 opc = (opcode >> 11) & 0x3;
	u32 target_address;
	char *mnemonic = NULL;
	
	/* sign extend 11-bit offset */
	if (((opc==0) || (opc==2)) && (offset & 0x00000400))
		offset = 0xfffff800 | offset;
	
	target_address = address + 4 + (offset<<1);

	switch(opc)
	{
		/* unconditional branch */
		case 0:
			instruction->type = ARM_B;
			mnemonic = "B";
			break;
		/* BLX suffix */
		case 1:
			instruction->type = ARM_BLX;
			mnemonic = "BLX";
			break;
		/* BL/BLX prefix */
		case 2:
			instruction->type = ARM_UNKNOWN_INSTUCTION;
			mnemonic = "prefix";
			target_address = offset<<12;
			break;
		/* BL suffix */
		case 3:
			instruction->type = ARM_BL;
			mnemonic = "BL";
			break;
	}
	/* TODO: deals correctly with dual opcodes BL/BLX ... */

	snprintf(instruction->text, 128, "0x%8.8x\t0x%4.4x\t%s 0x%8.8x", address, opcode,mnemonic, target_address);
	
	instruction->info.b_bl_bx_blx.reg_operand = -1;
	instruction->info.b_bl_bx_blx.target_address = target_address;

	return ERROR_OK;
}

int evaluate_add_sub_thumb(u16 opcode, u32 address, arm_instruction_t *instruction)
{
	u8 Rd = (opcode >> 0) & 0x7;
	u8 Rn = (opcode >> 3) & 0x7;
	u8 Rm_imm = (opcode >> 6) & 0x7;
	u32 opc = opcode & (1<<9);
	u32 reg_imm  = opcode & (1<<10);
	char *mnemonic;
	
	if (opc)
	{
		instruction->type = ARM_SUB;
		mnemonic = "SUBS";
	}
	else
	{
		instruction->type = ARM_ADD;
		mnemonic = "ADDS";
	}
	
	instruction->info.data_proc.Rd = Rd;
	instruction->info.data_proc.Rn = Rn;
	instruction->info.data_proc.S = 1;

	if (reg_imm)
	{
		instruction->info.data_proc.variant = 0; /*immediate*/
		instruction->info.data_proc.shifter_operand.immediate.immediate = Rm_imm;
		snprintf(instruction->text, 128, "0x%8.8x\t0x%4.4x\t%s r%i, r%i, #%d",
				 address, opcode, mnemonic, Rd, Rn, Rm_imm);
	}
	else
	{
		instruction->info.data_proc.variant = 1; /*immediate shift*/
		instruction->info.data_proc.shifter_operand.immediate_shift.Rm = Rm_imm;
		snprintf(instruction->text, 128, "0x%8.8x\t0x%4.4x\t%s r%i, r%i, r%i",
				 address, opcode, mnemonic, Rd, Rn, Rm_imm);
	}

	return ERROR_OK;
}

int evaluate_shift_imm_thumb(u16 opcode, u32 address, arm_instruction_t *instruction)
{
	u8 Rd = (opcode >> 0) & 0x7;
	u8 Rm = (opcode >> 3) & 0x7;
	u8 imm = (opcode >> 6) & 0x1f;
	u8 opc = (opcode >> 11) & 0x3;
	char *mnemonic = NULL;
	
	switch(opc)
	{
		case 0:
			instruction->type = ARM_MOV;
			mnemonic = "LSLS";
			instruction->info.data_proc.shifter_operand.immediate_shift.shift = 0;
			break;
		case 1:
			instruction->type = ARM_MOV;
			mnemonic = "LSRS";
			instruction->info.data_proc.shifter_operand.immediate_shift.shift = 1;
			break;
		case 2:
			instruction->type = ARM_MOV;
			mnemonic = "ASRS";
			instruction->info.data_proc.shifter_operand.immediate_shift.shift = 2;
			break;
	}

	if ((imm==0) && (opc!=0))
		imm = 32;

	instruction->info.data_proc.Rd = Rd;
	instruction->info.data_proc.Rn = -1;
	instruction->info.data_proc.S = 1;

	instruction->info.data_proc.variant = 1; /*immediate_shift*/
	instruction->info.data_proc.shifter_operand.immediate_shift.Rm = Rm;
	instruction->info.data_proc.shifter_operand.immediate_shift.shift_imm = imm;

	snprintf(instruction->text, 128, "0x%8.8x\t0x%4.4x\t%s r%i, r%i, #0x%02x",
				 address, opcode, mnemonic, Rd, Rm, imm);

	return ERROR_OK;
}

int evaluate_data_proc_imm_thumb(u16 opcode, u32 address, arm_instruction_t *instruction)
{
	u8 imm = opcode & 0xff;
	u8 Rd = (opcode >> 8) & 0x7;
	u32 opc = (opcode >> 11) & 0x3;
	char *mnemonic = NULL;
	
	instruction->info.data_proc.Rd = Rd;
	instruction->info.data_proc.Rn = Rd;
	instruction->info.data_proc.S = 1;
	instruction->info.data_proc.variant = 0; /*immediate*/
	instruction->info.data_proc.shifter_operand.immediate.immediate = imm;
	
	switch(opc)
	{
		case 0:
			instruction->type = ARM_MOV;
			mnemonic = "MOVS";
			instruction->info.data_proc.Rn = -1;
			break;
		case 1:
			instruction->type = ARM_CMP;
			mnemonic = "CMP";
			instruction->info.data_proc.Rd = -1;
			break;
		case 2:
			instruction->type = ARM_ADD;
			mnemonic = "ADDS";
			break;
		case 3:
			instruction->type = ARM_SUB;
			mnemonic = "SUBS";
			break;
	}
	
	snprintf(instruction->text, 128, "0x%8.8x\t0x%4.4x\t%s r%i, #0x%02x",
				 address, opcode, mnemonic, Rd, imm);

	return ERROR_OK;
}

int evaluate_data_proc_thumb(u16 opcode, u32 address, arm_instruction_t *instruction)
{
	u8 high_reg, op, Rm, Rd,H1,H2;
	char *mnemonic = NULL;
	
	high_reg = (opcode & 0x0400) >> 10;
	op = (opcode & 0x03C0) >> 6;
	
	Rd = (opcode & 0x0007);
	Rm = (opcode & 0x0038) >> 3;
	H1 = (opcode & 0x0080) >> 7;
	H2 = (opcode & 0x0040) >> 6;
	
	instruction->info.data_proc.Rd = Rd;
	instruction->info.data_proc.Rn = Rd;
	instruction->info.data_proc.S = (!high_reg || (instruction->type == ARM_CMP));
	instruction->info.data_proc.variant = 1 /*immediate shift*/;
	instruction->info.data_proc.shifter_operand.immediate_shift.Rm = Rm;

	if (high_reg)
	{
		Rd |= H1 << 3;
		Rm |= H2 << 3;
		op >>= 2;
	
		switch (op)
		{
			case 0x0:
				instruction->type = ARM_ADD;
				mnemonic = "ADD";
				break;
			case 0x1:
				instruction->type = ARM_CMP;
				mnemonic = "CMP";
				break;
			case 0x2:
				instruction->type = ARM_MOV;
				mnemonic = "MOV";
				break;
			case 0x3:
				if ((opcode & 0x7) == 0x0)
				{
					instruction->info.b_bl_bx_blx.reg_operand = Rm;
					if (H1)
					{
						instruction->type = ARM_BLX;
						snprintf(instruction->text, 128, "0x%8.8x\t0x%4.4x\tBLX r%i", address, opcode, Rm);
					}
					else
					{
						instruction->type = ARM_BX;
						snprintf(instruction->text, 128, "0x%8.8x\t0x%4.4x\tBX r%i", address, opcode, Rm);
					}
				}
				else
				{
					instruction->type = ARM_UNDEFINED_INSTRUCTION;
					snprintf(instruction->text, 128, "0x%8.8x\t0x%4.4x\tUNDEFINED INSTRUCTION", address, opcode);
				}
				return ERROR_OK;	
				break;
		}
	}
	else
	{
		switch (op)
		{
			case 0x0:
				instruction->type = ARM_AND;
				mnemonic = "ANDS";
				break;
			case 0x1:
				instruction->type = ARM_EOR;
				mnemonic = "EORS";
				break;
			case 0x2:
				instruction->type = ARM_MOV;
				mnemonic = "LSLS";
				instruction->info.data_proc.variant = 2 /*register shift*/;
				instruction->info.data_proc.shifter_operand.register_shift.shift = 0;
				instruction->info.data_proc.shifter_operand.register_shift.Rm = Rd;
				instruction->info.data_proc.shifter_operand.register_shift.Rs = Rm;
				break;
			case 0x3:
				instruction->type = ARM_MOV;
				mnemonic = "LSRS";
				instruction->info.data_proc.variant = 2 /*register shift*/;
				instruction->info.data_proc.shifter_operand.register_shift.shift = 1;
				instruction->info.data_proc.shifter_operand.register_shift.Rm = Rd;
				instruction->info.data_proc.shifter_operand.register_shift.Rs = Rm;
				break;
			case 0x4:
				instruction->type = ARM_MOV;
				mnemonic = "ASRS";
				instruction->info.data_proc.variant = 2 /*register shift*/;
				instruction->info.data_proc.shifter_operand.register_shift.shift = 2;
				instruction->info.data_proc.shifter_operand.register_shift.Rm = Rd;
				instruction->info.data_proc.shifter_operand.register_shift.Rs = Rm;
				break;
			case 0x5:
				instruction->type = ARM_ADC;
				mnemonic = "ADCS";
				break;
			case 0x6:
				instruction->type = ARM_SBC;
				mnemonic = "SBCS";
				break;
			case 0x7:
				instruction->type = ARM_MOV;
				mnemonic = "RORS";
				instruction->info.data_proc.variant = 2 /*register shift*/;
				instruction->info.data_proc.shifter_operand.register_shift.shift = 3;
				instruction->info.data_proc.shifter_operand.register_shift.Rm = Rd;
				instruction->info.data_proc.shifter_operand.register_shift.Rs = Rm;
				break;
			case 0x8:
				instruction->type = ARM_TST;
				mnemonic = "TST";
				break;
			case 0x9:
				instruction->type = ARM_RSB;
				mnemonic = "NEGS";
				instruction->info.data_proc.variant = 0 /*immediate*/;
				instruction->info.data_proc.shifter_operand.immediate.immediate = 0;
				instruction->info.data_proc.Rn = Rm;
				break;
			case 0xA:
				instruction->type = ARM_CMP;
				mnemonic = "CMP";
				break;
			case 0xB:
				instruction->type = ARM_CMN;
				mnemonic = "CMN";
				break;
			case 0xC:
				instruction->type = ARM_ORR;
				mnemonic = "ORRS";
				break;
			case 0xD:
				instruction->type = ARM_MUL;
				mnemonic = "MULS";
				break;
			case 0xE:
				instruction->type = ARM_BIC;
				mnemonic = "BICS";
				break;
			case 0xF:
				instruction->type = ARM_MVN;
				mnemonic = "MVNS";
				break;
		}
	}

	snprintf(instruction->text, 128, "0x%8.8x\t0x%4.4x\t%s r%i, r%i",
				 address, opcode, mnemonic, Rd, Rm);

	return ERROR_OK;
}

int evaluate_load_literal_thumb(u16 opcode, u32 address, arm_instruction_t *instruction)
{
	u32 immediate;
	u8 Rd = (opcode >> 8) & 0x7; 

	instruction->type = ARM_LDR;
	immediate = opcode & 0x000000ff;

	snprintf(instruction->text, 128, "0x%8.8x\t0x%4.4x\tLDR r%i, [PC, #0x%x]", address, opcode, Rd, immediate*4);

	instruction->info.load_store.Rd = Rd;
	instruction->info.load_store.Rn = 15 /*PC*/;
	instruction->info.load_store.index_mode = 0; /*offset*/
	instruction->info.load_store.offset_mode = 0; /*immediate*/
	instruction->info.load_store.offset.offset = immediate*4;

	return ERROR_OK;
}

int evaluate_load_store_reg_thumb(u16 opcode, u32 address, arm_instruction_t *instruction)
{
	u8 Rd = (opcode >> 0) & 0x7; 
	u8 Rn = (opcode >> 3) & 0x7; 
	u8 Rm = (opcode >> 6) & 0x7; 
	u8 opc = (opcode >> 9) & 0x7; 
	char *mnemonic = NULL;

	switch(opc)
	{
		case 0:
			instruction->type = ARM_STR;
			mnemonic = "STR";
			break;
		case 1:
			instruction->type = ARM_STRH;
			mnemonic = "STRH";
			break;
		case 2:
			instruction->type = ARM_STRB;
			mnemonic = "STRB";
			break;
		case 3:
			instruction->type = ARM_LDRSB;
			mnemonic = "LDRSB";
			break;
		case 4:
			instruction->type = ARM_LDR;
			mnemonic = "LDR";
			break;
		case 5:
			instruction->type = ARM_LDRH;
			mnemonic = "LDRH";
			break;
		case 6:
			instruction->type = ARM_LDRB;
			mnemonic = "LDRB";
			break;
		case 7:
			instruction->type = ARM_LDRSH;
			mnemonic = "LDRSH";
			break;
	}

	snprintf(instruction->text, 128, "0x%8.8x\t0x%4.4x\t%s r%i, [r%i, r%i]", address, opcode, mnemonic, Rd, Rn, Rm);
	
	instruction->info.load_store.Rd = Rd;
	instruction->info.load_store.Rn = Rn;
	instruction->info.load_store.index_mode = 0; /*offset*/
	instruction->info.load_store.offset_mode = 1; /*register*/
	instruction->info.load_store.offset.reg.Rm = Rm;

	return ERROR_OK;
}

int evaluate_load_store_imm_thumb(u16 opcode, u32 address, arm_instruction_t *instruction)
{
	u32 offset = (opcode >> 6) & 0x1f;
	u8 Rd = (opcode >> 0) & 0x7; 
	u8 Rn = (opcode >> 3) & 0x7; 
	u32 L = opcode & (1<<11);
	u32 B = opcode & (1<<12);
	char *mnemonic;
	char suffix = ' ';
	u32 shift = 2;

	if (L)
	{
		instruction->type = ARM_LDR;
		mnemonic = "LDR";
	}
	else
	{
		instruction->type = ARM_STR;
		mnemonic = "STR";
	}

	if ((opcode&0xF000)==0x8000)
	{
		suffix = 'H';
		shift = 1;
	}
	else if (B)
	{
		suffix = 'B';
		shift = 0;
	}

	snprintf(instruction->text, 128, "0x%8.8x\t0x%4.4x\t%s%c r%i, [r%i, #0x%x]", address, opcode, mnemonic, suffix, Rd, Rn, offset<<shift);
	
	instruction->info.load_store.Rd = Rd;
	instruction->info.load_store.Rn = Rn;
	instruction->info.load_store.index_mode = 0; /*offset*/
	instruction->info.load_store.offset_mode = 0; /*immediate*/
	instruction->info.load_store.offset.offset = offset<<shift;

	return ERROR_OK;
}

int evaluate_load_store_stack_thumb(u16 opcode, u32 address, arm_instruction_t *instruction)
{
	u32 offset = opcode  & 0xff;
	u8 Rd = (opcode >> 8) & 0x7; 
	u32 L = opcode & (1<<11);
	char *mnemonic;

	if (L)
	{
		instruction->type = ARM_LDR;
		mnemonic = "LDR";
	}
	else
	{
		instruction->type = ARM_STR;
		mnemonic = "STR";
	}

	snprintf(instruction->text, 128, "0x%8.8x\t0x%4.4x\t%s r%i, [SP, #0x%x]", address, opcode, mnemonic, Rd, offset*4);
	
	instruction->info.load_store.Rd = Rd;
	instruction->info.load_store.Rn = 13 /*SP*/;
	instruction->info.load_store.index_mode = 0; /*offset*/
	instruction->info.load_store.offset_mode = 0; /*immediate*/
	instruction->info.load_store.offset.offset = offset*4;

	return ERROR_OK;
}

int evaluate_add_sp_pc_thumb(u16 opcode, u32 address, arm_instruction_t *instruction)
{
	u32 imm = opcode  & 0xff;
	u8 Rd = (opcode >> 8) & 0x7; 
	u8 Rn;
	u32 SP = opcode & (1<<11);
	char *reg_name;

	instruction->type = ARM_ADD;
	
	if (SP)
	{
		reg_name = "SP";
		Rn = 13;
	}
	else
	{
		reg_name = "PC";
		Rn = 15;
	}

	snprintf(instruction->text, 128, "0x%8.8x\t0x%4.4x\tADD r%i, %s, #0x%x", address, opcode, Rd,reg_name, imm*4);

	instruction->info.data_proc.variant = 0 /* immediate */;
	instruction->info.data_proc.Rd = Rd;
	instruction->info.data_proc.Rn = Rn;
	instruction->info.data_proc.shifter_operand.immediate.immediate = imm*4;

	return ERROR_OK;
}

int evaluate_adjust_stack_thumb(u16 opcode, u32 address, arm_instruction_t *instruction)
{
	u32 imm = opcode  & 0x7f;
	u8 opc = opcode & (1<<7);
	char *mnemonic;

	
	if (opc)
	{
		instruction->type = ARM_SUB;
		mnemonic = "SUB";
	}
	else
	{
		instruction->type = ARM_ADD;
		mnemonic = "ADD";
	}

	snprintf(instruction->text, 128, "0x%8.8x\t0x%4.4x\t%s SP, #0x%x", address, opcode, mnemonic, imm*4);

	instruction->info.data_proc.variant = 0 /* immediate */;
	instruction->info.data_proc.Rd = 13 /*SP*/;
	instruction->info.data_proc.Rn = 13 /*SP*/;
	instruction->info.data_proc.shifter_operand.immediate.immediate = imm*4;

	return ERROR_OK;
}

int evaluate_breakpoint_thumb(u16 opcode, u32 address, arm_instruction_t *instruction)
{
	u32 imm = opcode  & 0xff;
	
	instruction->type = ARM_BKPT;

	snprintf(instruction->text, 128, "0x%8.8x\t0x%4.4x\tBKPT 0x%02x", address, opcode, imm);

	return ERROR_OK;
}

int evaluate_load_store_multiple_thumb(u16 opcode, u32 address, arm_instruction_t *instruction)
{
	u32 reg_list = opcode  & 0xff;
	u32 L = opcode & (1<<11);
	u32 R = opcode & (1<<8);
	u8 Rn = (opcode >> 8) & 7;
	u8 addr_mode = 0 /* IA */;
	char reg_names[40];
	char *reg_names_p;
	char *mnemonic;
	char ptr_name[7] = "";
	int i;	

	if ((opcode & 0xf000) == 0xc000)
	{ /* generic load/store multiple */
		if (L)
		{
			instruction->type = ARM_LDM;
			mnemonic = "LDMIA";
		}
		else
		{
			instruction->type = ARM_STM;
			mnemonic = "STMIA";
		}
		snprintf(ptr_name,7,"r%i!, ",Rn);
	}
	else
	{ /* push/pop */
		Rn = 13; /* SP */
		if (L)
		{
			instruction->type = ARM_LDM;
			mnemonic = "POP";
			if (R)
				reg_list |= (1<<15) /*PC*/;
		}
		else
		{
			instruction->type = ARM_STM;
			mnemonic = "PUSH";
			addr_mode = 3; /*DB*/
			if (R)
				reg_list |= (1<<14) /*LR*/;
		}
	}

	reg_names_p = reg_names;
	for (i = 0; i <= 15; i++)
	{
		if (reg_list & (1<<i))
			reg_names_p += snprintf(reg_names_p, (reg_names + 40 - reg_names_p), "r%i, ", i);
	}
	if (reg_names_p>reg_names)
		reg_names_p[-2] = '\0';
	else /* invalid op : no registers */
    		reg_names[0] = '\0';

	snprintf(instruction->text, 128, "0x%8.8x\t0x%4.4x\t%s %s{%s}", address, opcode, mnemonic, ptr_name,reg_names);

	instruction->info.load_store_multiple.register_list = reg_list;
	instruction->info.load_store_multiple.Rn = Rn;
	instruction->info.load_store_multiple.addressing_mode = addr_mode;

	return ERROR_OK;
}

int evaluate_cond_branch_thumb(u16 opcode, u32 address, arm_instruction_t *instruction)
{
	u32 offset = opcode  & 0xff;
	u8 cond = (opcode >> 8) & 0xf;
	u32 target_address;

	if (cond == 0xf)
	{
		instruction->type = ARM_SWI;
		snprintf(instruction->text, 128, "0x%8.8x\t0x%4.4x\tSWI 0x%02x", address, opcode, offset);
		return ERROR_OK;
	}
	else if (cond == 0xe)
	{
		instruction->type = ARM_UNDEFINED_INSTRUCTION;
		snprintf(instruction->text, 128, "0x%8.8x\t0x%4.4x\tUNDEFINED INSTRUCTION", address, opcode);
		return ERROR_OK;
	}

	/* sign extend 8-bit offset */
	if (offset & 0x00000080)
		offset = 0xffffff00 | offset;
	
	target_address = address + 4 + (offset<<1);

	snprintf(instruction->text, 128, "0x%8.8x\t0x%4.4x\tB%s 0x%8.8x", address, opcode,
			 arm_condition_strings[cond], target_address);
	
	instruction->type = ARM_B;
	instruction->info.b_bl_bx_blx.reg_operand = -1;
	instruction->info.b_bl_bx_blx.target_address = target_address;

	return ERROR_OK;
}

int thumb_evaluate_opcode(u16 opcode, u32 address, arm_instruction_t *instruction)
{
	/* clear fields, to avoid confusion */
	memset(instruction, 0, sizeof(arm_instruction_t));
	instruction->opcode = opcode;
	
	if ((opcode & 0xe000) == 0x0000)
	{
		/* add/substract register or immediate */
		if ((opcode & 0x1800) == 0x1800)
			return evaluate_add_sub_thumb(opcode, address, instruction);
		/* shift by immediate */
		else
			return evaluate_shift_imm_thumb(opcode, address, instruction);
	}
	
	/* Add/substract/compare/move immediate */
	if ((opcode & 0xe000) == 0x2000)
	{
		return evaluate_data_proc_imm_thumb(opcode, address, instruction);
	}
	
	/* Data processing instructions */
	if ((opcode & 0xf800) == 0x4000)
	{
		return evaluate_data_proc_thumb(opcode, address, instruction);
	}
	
	/* Load from literal pool */
	if ((opcode & 0xf800) == 0x4800)
	{
		return evaluate_load_literal_thumb(opcode, address, instruction);
	}

	/* Load/Store register offset */
	if ((opcode & 0xf000) == 0x5000)
	{
		return evaluate_load_store_reg_thumb(opcode, address, instruction);
	}

	/* Load/Store immediate offset */
	if (((opcode & 0xe000) == 0x6000)
		||((opcode & 0xf000) == 0x8000))
	{
		return evaluate_load_store_imm_thumb(opcode, address, instruction);
	}
	
	/* Load/Store from/to stack */
	if ((opcode & 0xf000) == 0x9000)
	{
		return evaluate_load_store_stack_thumb(opcode, address, instruction);
	}

	/* Add to SP/PC */
	if ((opcode & 0xf000) == 0xa000)
	{
		return evaluate_add_sp_pc_thumb(opcode, address, instruction);
	}

	/* Misc */
	if ((opcode & 0xf000) == 0xb000)
	{
		if ((opcode & 0x0f00) == 0x0000)
			return evaluate_adjust_stack_thumb(opcode, address, instruction);
		else if ((opcode & 0x0f00) == 0x0e00)
			return evaluate_breakpoint_thumb(opcode, address, instruction);
		else if ((opcode & 0x0600) == 0x0400) /* push pop */
			return evaluate_load_store_multiple_thumb(opcode, address, instruction);
		else
		{
			instruction->type = ARM_UNDEFINED_INSTRUCTION;
			snprintf(instruction->text, 128, "0x%8.8x\t0x%4.4x\tUNDEFINED INSTRUCTION", address, opcode);
			return ERROR_OK;
		}
	}

	/* Load/Store multiple */
	if ((opcode & 0xf000) == 0xc000)
	{
		return evaluate_load_store_multiple_thumb(opcode, address, instruction);
	}

	/* Conditional branch + SWI */
	if ((opcode & 0xf000) == 0xd000)
	{
		return evaluate_cond_branch_thumb(opcode, address, instruction);
	}
	
	if ((opcode & 0xe000) == 0xe000)
	{
		/* Undefined instructions */
		if ((opcode & 0xf801) == 0xe801)
		{
			instruction->type = ARM_UNDEFINED_INSTRUCTION;
			snprintf(instruction->text, 128, "0x%8.8x\t0x%8.8x\tUNDEFINED INSTRUCTION", address, opcode);
			return ERROR_OK;
		}
		else
		{ /* Branch to offset */
			return evaluate_b_bl_blx_thumb(opcode, address, instruction);
		}
	}

	LOG_ERROR("should never reach this point (opcode=%04x)",opcode);
	return -1;
}

int arm_access_size(arm_instruction_t *instruction)
{
	if ((instruction->type == ARM_LDRB)
		|| (instruction->type == ARM_LDRBT)
		|| (instruction->type == ARM_LDRSB)
		|| (instruction->type == ARM_STRB)
		|| (instruction->type == ARM_STRBT))
	{
		return 1;
	}
	else if ((instruction->type == ARM_LDRH)
		|| (instruction->type == ARM_LDRSH)
		|| (instruction->type == ARM_STRH))
	{
		return 2;
	}
	else if ((instruction->type == ARM_LDR)
		|| (instruction->type == ARM_LDRT)
		|| (instruction->type == ARM_STR)
		|| (instruction->type == ARM_STRT))
	{
		return 4;
	}
	else if ((instruction->type == ARM_LDRD)
		|| (instruction->type == ARM_STRD))
	{
		return 8;
	}
	else
	{
		LOG_ERROR("BUG: instruction type %i isn't a load/store instruction", instruction->type);
		return 0;
	}
}
