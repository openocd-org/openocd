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

#include "target.h"
#include "register.h"
#include "armv4_5.h"
#include "embeddedice.h"
#include "arm_jtag.h"
#include "arm7_9_common.h"

#define	ARM9TDMI_COMMON_MAGIC 0x00a900a9

typedef struct arm9tdmi_common_s
{
	int common_magic;
	void *arch_info;
	arm7_9_common_t arm7_9_common;
} arm9tdmi_common_t;

typedef struct arm9tdmi_vector_s
{
	char *name;
	u32 value;
} arm9tdmi_vector_t;

enum arm9tdmi_vector
{
	ARM9TDMI_RESET_VECTOR = 0x01,
	ARM9TDMI_UNDEF_VECTOR = 0x02,
	ARM9TDMI_SWI_VECTOR = 0x04,
	ARM9TDMI_PABT_VECTOR = 0x08,
	ARM9TDMI_DABT_VECTOR = 0x10,
	ARM9TDMI_RESERVED_VECTOR = 0x20,
	ARM9TDMI_IRQ_VECTOR = 0x40,
	ARM9TDMI_FIQ_VECTOR = 0x80,
};

extern int arm9tdmi_init_target(struct command_context_s *cmd_ctx, struct target_s *target);
int arm9tdmi_examine(struct target_s *target);
extern int arm9tdmi_init_arch_info(target_t *target, arm9tdmi_common_t *arm9tdmi, jtag_tap_t *tap);
extern int arm9tdmi_register_commands(struct command_context_s *cmd_ctx);

extern int arm9tdmi_clock_out(arm_jtag_t *jtag_info, u32 instr, u32 out, u32 *in, int sysspeed);
extern int arm9tdmi_clock_data_in(arm_jtag_t *jtag_info, u32 *in);
extern int arm9tdmi_clock_data_in_endianness(arm_jtag_t *jtag_info, void *in, int size, int be); 
extern void arm9tdmi_read_core_regs(target_t *target, u32 mask, u32* core_regs[16]);
extern void arm9tdmi_write_core_regs(target_t *target, u32 mask, u32 core_regs[16]);

#endif /* ARM9TDMI_H */
