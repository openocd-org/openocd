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

#ifndef MIPS_M4K_H
#define MIPS_M4K_H

#include "register.h"
#include "target.h"

#define MIPSM4K_COMMON_MAGIC	0xB321B321

typedef struct mips_m4k_common_s
{
	int common_magic;
	mips32_common_t mips32_common;
} mips_m4k_common_t;

extern int mips_m4k_bulk_write_memory(target_t *target, u32 address, u32 count, u8 *buffer);

extern void mips_m4k_enable_breakpoints(struct target_s *target);
extern int mips_m4k_set_breakpoint(struct target_s *target, breakpoint_t *breakpoint);
extern int mips_m4k_unset_breakpoint(struct target_s *target, breakpoint_t *breakpoint);
extern int mips_m4k_add_breakpoint(struct target_s *target, breakpoint_t *breakpoint);
extern int mips_m4k_remove_breakpoint(struct target_s *target, breakpoint_t *breakpoint);
extern int mips_m4k_set_watchpoint(struct target_s *target, watchpoint_t *watchpoint);
extern int mips_m4k_unset_watchpoint(struct target_s *target, watchpoint_t *watchpoint);
extern int mips_m4k_add_watchpoint(struct target_s *target, watchpoint_t *watchpoint);
extern int mips_m4k_remove_watchpoint(struct target_s *target, watchpoint_t *watchpoint);
extern void mips_m4k_enable_watchpoints(struct target_s *target);

#endif	/*MIPS_M4K_H*/
