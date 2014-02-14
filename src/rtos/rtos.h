/***************************************************************************
 *   Copyright (C) 2011 by Broadcom Corporation                            *
 *   Evan Hunter - ehunter@broadcom.com                                    *
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

#ifndef RTOS_H
#define RTOS_H

#include "server/server.h"
#include <jim-nvp.h>

typedef int64_t threadid_t;
typedef int64_t symbol_address_t;

struct reg;

/**
 * Table should be terminated by an element with NULL in symbol_name
 */
typedef struct symbol_table_elem_struct {
	char *symbol_name;
	symbol_address_t address;

} symbol_table_elem_t;

struct thread_detail {
	threadid_t threadid;
	bool exists;
	char *display_str;
	char *thread_name_str;
	char *extra_info_str;
};

struct rtos {
	const struct rtos_type *type;

	symbol_table_elem_t *symbols;
	struct target *target;
	/*  add a context variable instead of global variable */
	int64_t current_threadid;
	threadid_t current_thread;
	struct thread_detail *thread_details;
	int thread_count;
	int (*gdb_thread_packet)(struct connection *connection, char const *packet, int packet_size);
	void *rtos_specific_params;
};

struct rtos_type {
	char *name;
	int (*detect_rtos)(struct target *target);
	int (*create)(struct target *target);
	int (*smp_init)(struct target *target);
	int (*update_threads)(struct rtos *rtos);
	int (*get_thread_reg_list)(struct rtos *rtos, int64_t thread_id, char **hex_reg_list);
	int (*get_symbol_list_to_lookup)(symbol_table_elem_t *symbol_list[]);
	int (*clean)(struct target *target);
	char * (*ps_command)(struct target *target);
};

struct stack_register_offset {
	signed short offset;		/* offset in bytes from stack head, or -1 to indicate
					 * register is not stacked, or -2 to indicate this is the
					 * stack pointer register */
	unsigned short width_bits;
};

struct rtos_register_stacking {
	unsigned char stack_registers_size;
	signed char stack_growth_direction;
	unsigned char num_output_registers;
	unsigned char stack_alignment;
	const struct stack_register_offset *register_offsets;
};

#define GDB_THREAD_PACKET_NOT_CONSUMED (-40)

int rtos_create(Jim_GetOptInfo *goi, struct target *target);
int rtos_generic_stack_read(struct target *target,
		const struct rtos_register_stacking *stacking,
		int64_t stack_ptr,
		char **hex_reg_list);
int rtos_try_next(struct target *target);
int gdb_thread_packet(struct connection *connection, char const *packet, int packet_size);
int rtos_get_gdb_reg_list(struct connection *connection);
int rtos_update_threads(struct target *target);
void rtos_free_threadlist(struct rtos *rtos);
int rtos_smp_init(struct target *target);
/*  function for handling symbol access */
int rtos_qsymbol(struct connection *connection, char const *packet, int packet_size);

#endif	/* RTOS_H */
