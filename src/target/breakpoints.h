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

#include "types.h"

struct target_s;

enum breakpoint_type
{
	BKPT_HARD,
	BKPT_SOFT,
};

enum watchpoint_rw
{
	WPT_READ = 0, WPT_WRITE = 1, WPT_ACCESS = 2
};

typedef struct breakpoint_s
{
	uint32_t address;
	int length;
	enum breakpoint_type type;
	int set;
	uint8_t *orig_instr;
	struct breakpoint_s *next;
	int unique_id;
} breakpoint_t;

typedef struct watchpoint_s
{
	uint32_t address;
	int length;
	uint32_t mask;
	uint32_t value;
	enum watchpoint_rw rw;
	int set;
	struct watchpoint_s *next;
	int unique_id;
} watchpoint_t;

extern void breakpoint_clear_target(struct target_s *target);
extern int breakpoint_add(struct target_s *target, uint32_t address, uint32_t length, enum breakpoint_type type);
extern void breakpoint_remove(struct target_s *target, uint32_t address);
extern breakpoint_t* breakpoint_find(struct target_s *target, uint32_t address);
extern int watchpoint_add(struct target_s *target, uint32_t address, uint32_t length, enum watchpoint_rw rw, uint32_t value, uint32_t mask);
extern void watchpoint_remove(struct target_s *target, uint32_t address);
extern void watchpoint_clear_target(struct target_s *target);

#endif /* BREAKPOINTS_H */
