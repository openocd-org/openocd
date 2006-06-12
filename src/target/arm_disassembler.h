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
#ifndef ARM_DISASSEMBLER_H
#define ARM_DISASSEMBLER_H

#include "types.h"

enum arm_instruction_type
{
	ARM_UNKNOWN_INSTUCTION,
	
	/* Branch instructions */
	ARM_B,
	ARM_BL,
	ARM_BX,
	ARM_BLX,
	
	/* Data processing instructions */
	ARM_AND,
	ARM_EOR,
	ARM_SUB,
	ARM_RSB,
	ARM_ADD,
	ARM_ADC,
	ARM_SBC,
	ARM_RSC,
	ARM_TST,
	ARM_TEQ,
	ARM_CMP,
	ARM_CMN,
	ARM_ORR,
	ARM_MOV,
	ARM_BIC,
	ARM_MVN,
	
	/* Load/store instructions */
	ARM_LDR,
	ARM_LDRB,
	ARM_LDRT,
	ARM_LDRBT,
	
	ARM_LDRH,
	ARM_LDRSB,
	ARM_LDRSH,
	
	ARM_LDM,

	ARM_STR,
	ARM_STRB,
	ARM_STRT,
	ARM_STRBT,
	
	ARM_STRH,
	
	ARM_STM,
	
	/* Status register access instructions */
	ARM_MRS,
	ARM_MSR,
	
	/* Multiply instructions */
	ARM_MUL,
	ARM_MLA,
	ARM_SMULL,
	ARM_SMLAL,
	ARM_UMULL,
	ARM_UMLAL,
	
	/* Miscellaneous instructions */
	ARM_CLZ,
	
	/* Exception generating instructions */
	ARM_BKPT,
	ARM_SWI,
	
	/* Coprocessor instructions */
	ARM_CDP,
	ARM_LDC,
	ARM_STC,
	ARM_MCR,
	ARM_MRC,
	
	/* Semaphore instructions */
	ARM_SWP,
	ARM_SWPB,
	
	/* Enhanced DSP extensions */
	ARM_MCRR,
	ARM_MRRC,
	ARM_PLD,
	ARM_QADD,
	ARM_QDADD,
	ARM_QSUB,
	ARM_QDSUB,
	ARM_SMLAxy,
	ARM_SMLALxy,
	ARM_SMLAWy,
	ARM_SMULxy,
	ARM_SMULWy,
	ARM_LDRD,
	ARM_STRD,

	ARM_UNDEFINED_INSTRUCTION = 0xffffffff,
};

typedef struct arm_instruction_s
{
	enum arm_instruction_type type;
	char text[128];
	u32 opcode;
	
	/* target */
	u32 target_address;
	
} arm_instruction_t;

extern int evaluate_opcode(u32 opcode, u32 address, arm_instruction_t *instruction);

#define COND(opcode) (arm_condition_strings[(opcode & 0xf0000000)>>28])

#endif /* ARM_DISASSEMBLER_H */
