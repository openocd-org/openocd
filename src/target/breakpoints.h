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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifndef BREAKPOINTS_H
#define BREAKPOINTS_H

struct target;

enum breakpoint_type {
	BKPT_HARD,
	BKPT_SOFT,
};

enum watchpoint_rw {
	WPT_READ = 0, WPT_WRITE = 1, WPT_ACCESS = 2
};

struct breakpoint {
	uint32_t address;
	uint32_t asid;
	int length;
	enum breakpoint_type type;
	int set;
	uint8_t *orig_instr;
	struct breakpoint *next;
	uint32_t unique_id;
	int linked_BRP;
};

struct watchpoint {
	uint32_t address;
	uint32_t length;
	uint32_t mask;
	uint32_t value;
	enum watchpoint_rw rw;
	int set;
	struct watchpoint *next;
	int unique_id;
};

void breakpoint_clear_target(struct target *target);
int breakpoint_add(struct target *target,
		uint32_t address, uint32_t length, enum breakpoint_type type);
int context_breakpoint_add(struct target *target,
		uint32_t asid, uint32_t length, enum breakpoint_type type);
int hybrid_breakpoint_add(struct target *target,
		uint32_t address, uint32_t asid, uint32_t length, enum breakpoint_type type);
void breakpoint_remove(struct target *target, uint32_t address);

struct breakpoint *breakpoint_find(struct target *target, uint32_t address);

void watchpoint_clear_target(struct target *target);
int watchpoint_add(struct target *target,
		uint32_t address, uint32_t length,
		enum watchpoint_rw rw, uint32_t value, uint32_t mask);
void watchpoint_remove(struct target *target, uint32_t address);

/* report type and address of just hit watchpoint */
int watchpoint_hit(struct target *target, enum watchpoint_rw *rw, uint32_t *address);

#endif /* BREAKPOINTS_H */
