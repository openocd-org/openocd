/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_BREAKPOINTS_H
#define OPENOCD_TARGET_BREAKPOINTS_H

#include <stdint.h>

#include "helper/types.h"

struct target;

enum breakpoint_type {
	BKPT_HARD,
	BKPT_SOFT,
};

enum watchpoint_rw {
	WPT_READ = 0, WPT_WRITE = 1, WPT_ACCESS = 2
};

struct breakpoint {
	target_addr_t address;
	uint32_t asid;
	int length;
	enum breakpoint_type type;
	bool is_set;
	unsigned int number;
	uint8_t *orig_instr;
	struct breakpoint *next;
	uint32_t unique_id;
	int linked_brp;
};

#define WATCHPOINT_IGNORE_DATA_VALUE_MASK (~(uint64_t)0)

struct watchpoint {
	target_addr_t address;
	uint32_t length;
	uint64_t mask;
	uint64_t value;
	enum watchpoint_rw rw;
	bool is_set;
	unsigned int number;
	struct watchpoint *next;
	int unique_id;
};

int breakpoint_clear_target(struct target *target);
int breakpoint_add(struct target *target,
		target_addr_t address, uint32_t length, enum breakpoint_type type);
int context_breakpoint_add(struct target *target,
		uint32_t asid, uint32_t length, enum breakpoint_type type);
int hybrid_breakpoint_add(struct target *target,
		target_addr_t address, uint32_t asid, uint32_t length, enum breakpoint_type type);
int breakpoint_remove(struct target *target, target_addr_t address);
int breakpoint_remove_all(struct target *target);

struct breakpoint *breakpoint_find(struct target *target, target_addr_t address);

static inline void breakpoint_hw_set(struct breakpoint *breakpoint, unsigned int hw_number)
{
	breakpoint->is_set = true;
	breakpoint->number = hw_number;
}

int watchpoint_clear_target(struct target *target);
int watchpoint_add(struct target *target,
		target_addr_t address, uint32_t length,
		enum watchpoint_rw rw, uint64_t value, uint64_t mask);
int watchpoint_remove(struct target *target, target_addr_t address);
int watchpoint_remove_all(struct target *target);

/* report type and address of just hit watchpoint */
int watchpoint_hit(struct target *target, enum watchpoint_rw *rw,
		target_addr_t *address);

static inline void watchpoint_set(struct watchpoint *watchpoint, unsigned int number)
{
	watchpoint->is_set = true;
	watchpoint->number = number;
}

#define ERROR_BREAKPOINT_NOT_FOUND (-1600)
#define ERROR_WATCHPOINT_NOT_FOUND (-1601)

#endif /* OPENOCD_TARGET_BREAKPOINTS_H */
