/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
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
#ifndef ARM926EJS_H
#define ARM926EJS_H

#include "target.h"
#include "register.h"
#include "embeddedice.h"
#include "arm_jtag.h"
#include "arm9tdmi.h"
#include "armv4_5_mmu.h"
#include "armv4_5_cache.h"

#define	ARM926EJS_COMMON_MAGIC 0xa926a926

typedef struct arm926ejs_common_s
{
	int common_magic;
	armv4_5_mmu_common_t armv4_5_mmu;
	arm9tdmi_common_t arm9tdmi_common;
	int (*read_cp15)(target_t *target, u32 op1, u32 op2, u32 CRn, u32 CRm, u32 *value);
	int (*write_cp15)(target_t *target, u32 op1, u32 op2, u32 CRn, u32 CRm, u32 value);
	u32 cp15_control_reg;
	u32 d_fsr;
	u32 i_fsr;
	u32 d_far;
} arm926ejs_common_t;

extern int arm926ejs_init_arch_info(target_t *target, arm926ejs_common_t *arm926ejs, jtag_tap_t *tap);
extern int arm926ejs_register_commands(struct command_context_s *cmd_ctx); 
extern int arm926ejs_arch_state(struct target_s *target); 
extern int arm926ejs_write_memory(struct target_s *target, u32 address, u32 size, u32 count, u8 *buffer); 
extern int arm926ejs_soft_reset_halt(struct target_s *target);

#endif /* ARM926EJS_H */
