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
#ifndef ARM9TDMI_H
#define ARM9TDMI_H

#include "embeddedice.h"

/* FIXME we don't really need a separate arm9tdmi struct any more...
 * remove it, the arm7/arm9 common struct suffices.
 */
struct arm9tdmi_common
{
	struct arm7_9_common arm7_9_common;
};

struct arm9tdmi_vector
{
	char *name;
	uint32_t value;
};

enum arm9tdmi_vector_bit
{
	ARM9TDMI_RESET_VECTOR = 0x01,
	ARM9TDMI_UNDEF_VECTOR = 0x02,
	ARM9TDMI_SWI_VECTOR = 0x04,
	ARM9TDMI_PABT_VECTOR = 0x08,
	ARM9TDMI_DABT_VECTOR = 0x10,
	/* BIT(5) reserved -- must be zero */
	ARM9TDMI_IRQ_VECTOR = 0x40,
	ARM9TDMI_FIQ_VECTOR = 0x80,
};

int arm9tdmi_init_target(struct command_context_s *cmd_ctx,
		struct target_s *target);
int arm9tdmi_examine(struct target_s *target);
int arm9tdmi_init_arch_info(target_t *target,
		struct arm9tdmi_common *arm9tdmi, struct jtag_tap *tap);
int arm9tdmi_register_commands(struct command_context_s *cmd_ctx);

int arm9tdmi_clock_out(struct arm_jtag *jtag_info,
		uint32_t instr, uint32_t out, uint32_t *in, int sysspeed);
int arm9tdmi_clock_data_in(struct arm_jtag *jtag_info, uint32_t *in);
int arm9tdmi_clock_data_in_endianness(struct arm_jtag *jtag_info,
		void *in, int size, int be);
void arm9tdmi_read_core_regs(target_t *target,
		uint32_t mask, uint32_t* core_regs[16]);
void arm9tdmi_write_core_regs(target_t *target,
		uint32_t mask, uint32_t core_regs[16]);

int arm9tdmi_examine_debug_reason(target_t *target);

void arm9tdmi_load_word_regs(target_t *target, uint32_t mask);
void arm9tdmi_load_hword_reg(target_t *target, int num);
void arm9tdmi_load_byte_reg(target_t *target, int num);
void arm9tdmi_store_word_regs(target_t *target, uint32_t mask);
void arm9tdmi_store_hword_reg(target_t *target, int num);
void arm9tdmi_store_byte_reg(target_t *target, int num);

void arm9tdmi_branch_resume(target_t *target);
void arm9tdmi_enable_single_step(target_t *target, uint32_t next_pc);
void arm9tdmi_disable_single_step(target_t *target);

#endif /* ARM9TDMI_H */
