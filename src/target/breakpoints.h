/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
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
#ifndef BREAKPOINTS_H
#define BREAKPOINTS_H

#include "target.h"

struct target_s;

enum breakpoint_type
{
	BKPT_HARD,
	BKPT_SOFT,
};

extern char *breakpoint_type_strings[];

enum watchpoint_rw
{
	WPT_READ = 0, WPT_WRITE = 1, WPT_ACCESS = 2
};

extern char *watchpoint_rw_strings[];

typedef struct breakpoint_s
{
	u32 address;
	int length;
	enum breakpoint_type type;
	int set;
	u8 *orig_instr;
	struct breakpoint_s *next;
} breakpoint_t;

typedef struct watchpoint_s
{
	u32 address;
	int length;
	u32 mask;
	u32 value;
	enum watchpoint_rw rw;
	int set;
	struct watchpoint_s *next;
} watchpoint_t;

extern void breakpoint_clear_target(struct target_s *target);
extern int breakpoint_add(struct target_s *target, u32 address, u32 length, enum breakpoint_type type);
extern void breakpoint_remove(struct target_s *target, u32 address);
extern breakpoint_t* breakpoint_find(struct target_s *target, u32 address);
extern int watchpoint_add(struct target_s *target, u32 address, u32 length, enum watchpoint_rw rw, u32 value, u32 mask);
extern void watchpoint_remove(struct target_s *target, u32 address);
extern void watchpoint_clear_target(struct target_s *target);

#endif /* BREAKPOINTS_H */
