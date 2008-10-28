/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Hongtao Zheng                                   *
 *   hontor@126.com                                                        *
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

#include "target.h"
#include "armv4_5.h"
#include "arm_disassembler.h"
#include "arm_simulator.h"
#include "log.h"
#include "binarybuffer.h"

#include <string.h>

u32 arm_shift(u8 shift, u32 Rm, u32 shift_amount, u8 *carry)
{
	u32 return_value = 0;
	shift_amount &= 0xff;
	
	if (shift == 0x0) /* LSL */
	{
		if ((shift_amount > 0) && (shift_amount <= 32))
		{
			return_value = Rm << shift_amount;
			*carry = Rm >> (32 - shift_amount);
		}
		else if (shift_amount > 32)
		{
			return_value = 0x0;
			*carry = 0x0;
		}
		else /* (shift_amount == 0) */
		{
			return_value = Rm;
		}
	}
	else if (shift == 0x1) /* LSR */
	{
		if ((shift_amount > 0) && (shift_amount <= 32))
		{
			return_value = Rm >> shift_amount;
			*carry = (Rm >> (shift_amount - 1)) & 1;
		}
		else if (shift_amount > 32)
		{
			return_value = 0x0;
			*carry = 0x0;
		}
		else /* (shift_amount == 0) */
		{
			return_value = Rm;
		}
	}
	else if (shift == 0x2) /* ASR */
	{
		if ((shift_amount > 0) && (shift_amount <= 32))
		{
			/* right shifts of unsigned values are guaranteed to be logical (shift in zeroes)
			 * simulate an arithmetic shift (shift in signed-bit) by adding the signed-bit manually */
			return_value = Rm >> shift_amount;
			if (Rm & 0x80000000)
				return_value |= 0xffffffff << (32 - shift_amount);
		}
		else if (shift_amount > 32)
		{
			if (Rm & 0x80000000)
			{
				return_value = 0xffffffff;
				*carry = 0x1;
			}
			else
			{
				return_value = 0x0;
				*carry = 0x0;
			}
		}
		else /* (shift_amount == 0) */
		{
			return_value = Rm;
		}
	}
	else if (shift == 0x3) /* ROR */
	{
		if (shift_amount == 0)
		{
			return_value = Rm;
		}
		else
		{
			shift_amount = shift_amount % 32;
			return_value = (Rm >> shift_amount) | (Rm << (32 - shift_amount));
			*carry = (return_value >> 31) & 0x1;
		}
	}
	else if (shift == 0x4) /* RRX */
	{
		return_value = Rm >> 1;
		if (*carry)
			Rm |= 0x80000000;
		*carry = Rm & 0x1;
	}
	
	return return_value;
}

u32 arm_shifter_operand(armv4_5_common_t *armv4_5, int variant, union arm_shifter_operand shifter_operand, u8 *shifter_carry_out)
{
	u32 return_value;
	int instruction_size;
	
	if (armv4_5->core_state == ARMV4_5_STATE_ARM)
		instruction_size = 4;
	else
		instruction_size = 2;
	
	*shifter_carry_out = buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 29, 1);
	
	if (variant == 0) /* 32-bit immediate */
	{
		return_value = shifter_operand.immediate.immediate;
	}
	else if (variant == 1) /* immediate shift */
	{
		u32 Rm = buf_get_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, shifter_operand.immediate_shift.Rm).value, 0, 32);
		
		/* adjust RM in case the PC is being read */
		if (shifter_operand.immediate_shift.Rm == 15)
			Rm += 2 * instruction_size;
		
		return_value = arm_shift(shifter_operand.immediate_shift.shift, Rm, shifter_operand.immediate_shift.shift_imm, shifter_carry_out);
	}
	else if (variant == 2) /* register shift */
	{
		u32 Rm = buf_get_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, shifter_operand.register_shift.Rm).value, 0, 32);
		u32 Rs = buf_get_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, shifter_operand.register_shift.Rs).value, 0, 32);
		
		/* adjust RM in case the PC is being read */
		if (shifter_operand.register_shift.Rm == 15)
			Rm += 2 * instruction_size;
			
		return_value = arm_shift(shifter_operand.immediate_shift.shift, Rm, Rs, shifter_carry_out);
	}
	else
	{
		LOG_ERROR("BUG: shifter_operand.variant not 0, 1 or 2");
		return_value = 0xffffffff;
	}
	
	return return_value;
}

int pass_condition(u32 cpsr, u32 opcode)
{
	switch ((opcode & 0xf0000000) >> 28)
	{
		case 0x0:	/* EQ */
			if (cpsr & 0x40000000)
				return 1;
			else
				return 0;
		case 0x1:	/* NE */
			if (!(cpsr & 0x40000000))
				return 1;
			else
				return 0;
		case 0x2:	/* CS */
			if (cpsr & 0x20000000)
				return 1;
			else
				return 0;
		case 0x3:	/* CC */
			if (!(cpsr & 0x20000000))
				return 1;
			else
				return 0;
		case 0x4:	/* MI */
			if (cpsr & 0x80000000)
				return 1;
			else
				return 0;
		case 0x5:	/* PL */
			if (!(cpsr & 0x80000000))
				return 1;
			else
				return 0;
		case 0x6:	/* VS */
			if (cpsr & 0x10000000)
				return 1;
			else
				return 0;
		case 0x7:	/* VC */
			if (!(cpsr & 0x10000000))
				return 1;
			else
				return 0;
		case 0x8:	/* HI */
			if ((cpsr & 0x20000000) && !(cpsr & 0x40000000))
				return 1;
			else
				return 0;
		case 0x9:	/* LS */
			if (!(cpsr & 0x20000000) || (cpsr & 0x40000000))
				return 1;
			else
				return 0;
		case 0xa:	/* GE */
			if (((cpsr & 0x80000000) && (cpsr & 0x10000000))
				|| (!(cpsr & 0x80000000) && !(cpsr & 0x10000000)))
				return 1;
			else
				return 0;
		case 0xb:	/* LT */
			if (((cpsr & 0x80000000) && !(cpsr & 0x10000000))
				|| (!(cpsr & 0x80000000) && (cpsr & 0x10000000)))
				return 1;
			else
				return 0;
		case 0xc:	/* GT */
			if (!(cpsr & 0x40000000) &&
				(((cpsr & 0x80000000) && (cpsr & 0x10000000))
				|| (!(cpsr & 0x80000000) && !(cpsr & 0x10000000))))
				return 1;
			else
				return 0;
		case 0xd:	/* LE */
			if ((cpsr & 0x40000000) &&
				(((cpsr & 0x80000000) && !(cpsr & 0x10000000))
				|| (!(cpsr & 0x80000000) && (cpsr & 0x10000000))))
				return 1;
			else
				return 0;
		case 0xe:
		case 0xf:
			return 1;
				
	}
	
	LOG_ERROR("BUG: should never get here");
	return 0;
}

int thumb_pass_branch_condition(u32 cpsr, u16 opcode)
{
	return pass_condition(cpsr, (opcode & 0x0f00) << 20); 
}

/* simulate a single step (if possible)
 * if the dry_run_pc argument is provided, no state is changed,
 * but the new pc is stored in the variable pointed at by the argument
 */
int arm_simulate_step(target_t *target, u32 *dry_run_pc)
{
	armv4_5_common_t *armv4_5 = target->arch_info;
	u32 current_pc = buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32);
	arm_instruction_t instruction;
	int instruction_size;
	int retval = ERROR_OK;
	
	if (armv4_5->core_state == ARMV4_5_STATE_ARM)
	{
		u32 opcode;
		
		/* get current instruction, and identify it */
		if((retval = target_read_u32(target, current_pc, &opcode)) != ERROR_OK)
		{
			return retval;
		}
		if((retval = arm_evaluate_opcode(opcode, current_pc, &instruction)) != ERROR_OK)
		{
			return retval;
		}
		instruction_size = 4;
		
		/* check condition code (for all instructions) */
		if (!pass_condition(buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 32), opcode))
		{
			if (dry_run_pc)
			{
				*dry_run_pc = current_pc + instruction_size;
			}
			else
			{
				buf_set_u32(armv4_5->core_cache->reg_list[15].value, 0, 32, current_pc + instruction_size);
			}
			
			return ERROR_OK;
		}
	}
	else
	{
		u16 opcode;
		
		if((retval = target_read_u16(target, current_pc, &opcode)) != ERROR_OK)
		{
			return retval;
		}
		if((retval = thumb_evaluate_opcode(opcode, current_pc, &instruction)) != ERROR_OK)
		{
			return retval;
			}
		instruction_size = 2;
		
		/* check condition code (only for branch instructions) */
		if ((!thumb_pass_branch_condition(buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 32), opcode)) &&
			(instruction.type == ARM_B))
		{
			if (dry_run_pc)
			{
				*dry_run_pc = current_pc + instruction_size;
			}
			else
			{
				buf_set_u32(armv4_5->core_cache->reg_list[15].value, 0, 32, current_pc + instruction_size);
			}
			
			return ERROR_OK;
		}
	}
	
	/* examine instruction type */

	/* branch instructions */
	if ((instruction.type >= ARM_B) && (instruction.type <= ARM_BLX))
	{
		u32 target;
		
		if (instruction.info.b_bl_bx_blx.reg_operand == -1)
		{
			target = instruction.info.b_bl_bx_blx.target_address;
		}
		else
		{
			target = buf_get_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, instruction.info.b_bl_bx_blx.reg_operand).value, 0, 32); 
			if(instruction.info.b_bl_bx_blx.reg_operand == 15)
			{
				target += 2 * instruction_size;
			}
		}
		
		if (dry_run_pc)
		{	
			*dry_run_pc = target;
			return ERROR_OK;
		}
		else
		{
			if (instruction.type == ARM_B)
			{
				buf_set_u32(armv4_5->core_cache->reg_list[15].value, 0, 32, target);
			}
			else if (instruction.type == ARM_BL)
			{
				u32 old_pc = buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32);
				buf_set_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 14).value, 0, 32, old_pc + 4);
				buf_set_u32(armv4_5->core_cache->reg_list[15].value, 0, 32, target);
			}
			else if (instruction.type == ARM_BX)
			{
				if (target & 0x1)
				{
					armv4_5->core_state = ARMV4_5_STATE_THUMB;
				}
				else
				{
					armv4_5->core_state = ARMV4_5_STATE_ARM;
				}
				buf_set_u32(armv4_5->core_cache->reg_list[15].value, 0, 32, target & 0xfffffffe);
			}
			else if (instruction.type == ARM_BLX)
			{
				u32 old_pc = buf_get_u32(armv4_5->core_cache->reg_list[15].value, 0, 32);
				buf_set_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 14).value, 0, 32, old_pc + 4);

				if (target & 0x1)
				{
					armv4_5->core_state = ARMV4_5_STATE_THUMB;
				}
				else
				{
					armv4_5->core_state = ARMV4_5_STATE_ARM;
				}
				buf_set_u32(armv4_5->core_cache->reg_list[15].value, 0, 32, target & 0xfffffffe);
			}
			
			return ERROR_OK;
		}
	}
	/* data processing instructions, except compare instructions (CMP, CMN, TST, TEQ) */
	else if (((instruction.type >= ARM_AND) && (instruction.type <= ARM_RSC))
			|| ((instruction.type >= ARM_ORR) && (instruction.type <= ARM_MVN)))
	{
		u32 Rd, Rn, shifter_operand;
		u8 C = buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 29, 1);
		u8 carry_out;
		
		Rd = 0x0;
		Rn = buf_get_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, instruction.info.data_proc.Rn).value, 0, 32);
		shifter_operand = arm_shifter_operand(armv4_5, instruction.info.data_proc.variant, instruction.info.data_proc.shifter_operand, &carry_out);

		/* adjust Rn in case the PC is being read */
		if (instruction.info.data_proc.Rn == 15)
			Rn += 2 * instruction_size;
				
		if (instruction.type == ARM_AND)
			Rd = Rn & shifter_operand;
		else if (instruction.type == ARM_EOR)
			Rd = Rn ^ shifter_operand;
		else if (instruction.type == ARM_SUB)
			Rd = Rn - shifter_operand;
		else if (instruction.type == ARM_RSB)
			Rd = shifter_operand - Rn;
		else if (instruction.type == ARM_ADD)
			Rd = Rn + shifter_operand;
		else if (instruction.type == ARM_ADC)
			Rd = Rn + shifter_operand + (C & 1);
		else if (instruction.type == ARM_SBC)
			Rd = Rn - shifter_operand - (C & 1) ? 0 : 1;
		else if (instruction.type == ARM_RSC)
			Rd = shifter_operand - Rn - (C & 1) ? 0 : 1;
		else if (instruction.type == ARM_ORR)
			Rd = Rn | shifter_operand;
		else if (instruction.type == ARM_BIC)
			Rd = Rn & ~(shifter_operand);
		else if (instruction.type == ARM_MOV)
			Rd = shifter_operand;
		else if (instruction.type == ARM_MVN)
			Rd = ~shifter_operand;
		
		if (dry_run_pc)
		{
			if (instruction.info.data_proc.Rd == 15)
			{
				*dry_run_pc = Rd;
				return ERROR_OK;
			}
			else
			{
				*dry_run_pc = current_pc + instruction_size;
			}
			
			return ERROR_OK;
		}
		else
		{
			buf_set_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, instruction.info.data_proc.Rd).value, 0, 32, Rd);
			LOG_WARNING("no updating of flags yet");

			if (instruction.info.data_proc.Rd == 15)  
				return ERROR_OK;
		}
	}
	/* compare instructions (CMP, CMN, TST, TEQ) */
	else if ((instruction.type >= ARM_TST) && (instruction.type <= ARM_CMN))
	{
		if (dry_run_pc)
		{
			*dry_run_pc = current_pc + instruction_size;
			return ERROR_OK;
		}
		else
		{
			LOG_WARNING("no updating of flags yet");
		}
	}
	/* load register instructions */
	else if ((instruction.type >= ARM_LDR) && (instruction.type <= ARM_LDRSH))
	{
		u32 load_address = 0, modified_address = 0, load_value;
		u32 Rn = buf_get_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, instruction.info.load_store.Rn).value, 0, 32);
		
		/* adjust Rn in case the PC is being read */
		if (instruction.info.load_store.Rn == 15)
			Rn += 2 * instruction_size;
		
		if (instruction.info.load_store.offset_mode == 0)
		{
			if (instruction.info.load_store.U)
				modified_address = Rn + instruction.info.load_store.offset.offset;
			else
				modified_address = Rn - instruction.info.load_store.offset.offset;
		}
		else if (instruction.info.load_store.offset_mode == 1)
		{
			u32 offset;
			u32 Rm = buf_get_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, instruction.info.load_store.offset.reg.Rm).value, 0, 32);
			u8 shift = instruction.info.load_store.offset.reg.shift;
			u8 shift_imm = instruction.info.load_store.offset.reg.shift_imm;
			u8 carry = buf_get_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 29, 1);
			
			offset = arm_shift(shift, Rm, shift_imm, &carry);
			
			if (instruction.info.load_store.U)
				modified_address = Rn + offset;
			else
				modified_address = Rn - offset;
		}
		else
		{
			LOG_ERROR("BUG: offset_mode neither 0 (offset) nor 1 (scaled register)");
		}
		
		if (instruction.info.load_store.index_mode == 0)
		{
			/* offset mode
			 * we load from the modified address, but don't change the base address register */
			load_address = modified_address;
			modified_address = Rn;
		}
		else if (instruction.info.load_store.index_mode == 1)
		{
			/* pre-indexed mode
			 * we load from the modified address, and write it back to the base address register */
			load_address = modified_address;
		}
		else if (instruction.info.load_store.index_mode == 2)
		{
			/* post-indexed mode
			 * we load from the unmodified address, and write the modified address back */
			 load_address = Rn;
		}
		
		if((!dry_run_pc) || (instruction.info.load_store.Rd == 15))
		{
			if((retval = target_read_u32(target, load_address, &load_value)) != ERROR_OK)
			{
				return retval;
			}
		}
		
		if (dry_run_pc)
		{
			if (instruction.info.load_store.Rd == 15)
			{
				*dry_run_pc = load_value;
				return ERROR_OK;
			}
			else
			{
				*dry_run_pc = current_pc + instruction_size;
			}
			
			return ERROR_OK;
		}
		else
		{
			if ((instruction.info.load_store.index_mode == 1) ||
				(instruction.info.load_store.index_mode == 2))
			{
				buf_set_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, instruction.info.load_store.Rn).value, 0, 32, modified_address);
			} 
			buf_set_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, instruction.info.load_store.Rd).value, 0, 32, load_value);
			
			if (instruction.info.load_store.Rd == 15)
				return ERROR_OK;
		}
	}
	/* load multiple instruction */
	else if (instruction.type == ARM_LDM)
	{
		int i;
		u32 Rn = buf_get_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, instruction.info.load_store_multiple.Rn).value, 0, 32);
		u32 load_values[16];
		int bits_set = 0;

		for (i = 0; i < 16; i++)
		{
			if (instruction.info.load_store_multiple.register_list & (1 << i))
				bits_set++;
		}
		
		switch (instruction.info.load_store_multiple.addressing_mode)
		{
			case 0: /* Increment after */
				Rn = Rn;
				break;
			case 1: /* Increment before */
				Rn = Rn + 4;
				break;
			case 2: /* Decrement after */
				Rn = Rn - (bits_set * 4) + 4; 
				break;
			case 3: /* Decrement before */
				Rn = Rn - (bits_set * 4);
				break;
		}

		for (i = 0; i < 16; i++)
		{
			if (instruction.info.load_store_multiple.register_list & (1 << i))
			{
				if((!dry_run_pc) || (i == 15))
				{
					target_read_u32(target, Rn, &load_values[i]);
				}
				Rn += 4;
			}
		}
		
		if (dry_run_pc)
		{
			if (instruction.info.load_store_multiple.register_list & 0x8000)
			{
				*dry_run_pc = load_values[15];
				return ERROR_OK;
			}
		}
		else
		{
			enum armv4_5_mode mode = armv4_5->core_mode;
			int update_cpsr = 0;

			if (instruction.info.load_store_multiple.S)
			{
				if (instruction.info.load_store_multiple.register_list & 0x8000)
					update_cpsr = 1;
				else
					mode = ARMV4_5_MODE_USR;
			}

			for (i = 0; i < 16; i++)
			{
				if (instruction.info.load_store_multiple.register_list & (1 << i))
				{
					buf_set_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, mode, i).value, 0, 32, load_values[i]);
				}
			}
			
			if (update_cpsr)
			{
				u32 spsr = buf_get_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, 16).value, 0, 32);
				buf_set_u32(armv4_5->core_cache->reg_list[ARMV4_5_CPSR].value, 0, 32, spsr);
			}
			
			/* base register writeback */
			if (instruction.info.load_store_multiple.W)
				buf_set_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, instruction.info.load_store_multiple.Rn).value, 0, 32, Rn); 
			
			if (instruction.info.load_store_multiple.register_list & 0x8000)
				return ERROR_OK;
		}
	}
	/* store multiple instruction */
	else if (instruction.type == ARM_STM)
	{
		int i;

		if (dry_run_pc)
		{
			/* STM wont affect PC (advance by instruction size */
		}
		else
		{
			u32 Rn = buf_get_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, instruction.info.load_store_multiple.Rn).value, 0, 32);
			int bits_set = 0;
			enum armv4_5_mode mode = armv4_5->core_mode;

			for (i = 0; i < 16; i++)
			{
				if (instruction.info.load_store_multiple.register_list & (1 << i))
					bits_set++;
			}
			
			if (instruction.info.load_store_multiple.S)
			{
				mode = ARMV4_5_MODE_USR;
			}
			
			switch (instruction.info.load_store_multiple.addressing_mode)
			{
				case 0: /* Increment after */
					Rn = Rn;
					break;
				case 1: /* Increment before */
					Rn = Rn + 4;
					break;
				case 2: /* Decrement after */
					Rn = Rn - (bits_set * 4) + 4; 
					break;
				case 3: /* Decrement before */
					Rn = Rn - (bits_set * 4);
					break;
			}
			
			for (i = 0; i < 16; i++)
			{
				if (instruction.info.load_store_multiple.register_list & (1 << i))
				{
					target_write_u32(target, Rn, buf_get_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, i).value, 0, 32));
					Rn += 4;
				}
			}
			
			/* base register writeback */
			if (instruction.info.load_store_multiple.W)
				buf_set_u32(ARMV4_5_CORE_REG_MODE(armv4_5->core_cache, armv4_5->core_mode, instruction.info.load_store_multiple.Rn).value, 0, 32, Rn); 
			
		}
	}
	else if (!dry_run_pc)
	{
		/* the instruction wasn't handled, but we're supposed to simulate it
		 */
		return ERROR_ARM_SIMULATOR_NOT_IMPLEMENTED;
	}
	
	if (dry_run_pc)
	{
		*dry_run_pc = current_pc + instruction_size;
		return ERROR_OK;
	}
	else
	{
		buf_set_u32(armv4_5->core_cache->reg_list[15].value, 0, 32, current_pc + instruction_size);
		return ERROR_OK;
	}
	
}
