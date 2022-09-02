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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef OPENOCD_RTOS_RTOS_H
#define OPENOCD_RTOS_RTOS_H

#include "server/server.h"
#include "target/breakpoints.h"
#include "target/target.h"
#include <helper/jim-nvp.h>

typedef int64_t threadid_t;
typedef int64_t symbol_address_t;

struct reg;

/**
 * Table should be terminated by an element with NULL in symbol_name
 */
struct symbol_table_elem {
	const char *symbol_name;
	symbol_address_t address;
	bool optional;
};

struct thread_detail {
	threadid_t threadid;
	bool exists;
	char *thread_name_str;
	char *extra_info_str;
};

struct rtos {
	const struct rtos_type *type;

	struct symbol_table_elem *symbols;
	struct target *target;
	/*  add a context variable instead of global variable */
	/* The thread currently selected by gdb. */
	int64_t current_threadid;
	/* The currently selected thread according to the target. */
	threadid_t current_thread;
	struct thread_detail *thread_details;
	int thread_count;
	int (*gdb_thread_packet)(struct connection *connection, char const *packet, int packet_size);
	int (*gdb_target_for_threadid)(struct connection *connection, threadid_t thread_id, struct target **p_target);
	void *rtos_specific_params;
	/* Populated in rtos.c, so that individual RTOSes can register commands. */
	struct command_context *cmd_ctx;
};

struct rtos_reg {
	uint32_t number;
	uint32_t size;
	uint8_t value[16];
	/* WARNING: rtos_get_gdb_reg() relies on the fact that value is the last
	 * element of this struct. Any new fields should be added *before* value. */
};

struct rtos_type {
	const char *name;
	bool (*detect_rtos)(struct target *target);
	int (*create)(struct target *target);
	int (*smp_init)(struct target *target);
	int (*update_threads)(struct rtos *rtos);
	/** Return a list of general registers, with their values filled out. */
	int (*get_thread_reg_list)(struct rtos *rtos, threadid_t thread_id,
			struct rtos_reg **reg_list, int *num_regs);
	/** Return the size and value of the specified reg_num. The value is
	 * allocated by the callee and freed by the caller. */
	int (*get_thread_reg_value)(struct rtos *rtos, threadid_t thread_id,
			uint32_t reg_num, uint32_t *size, uint8_t **value);
	int (*get_symbol_list_to_lookup)(struct symbol_table_elem *symbol_list[]);
	int (*clean)(struct target *target);
	char * (*ps_command)(struct target *target);
	int (*set_reg)(struct rtos *rtos, uint32_t reg_num, uint8_t *reg_value);
	/**
	 * Possibly work around an annoying gdb behaviour: when the current thread
	 * is changed in gdb, it assumes that the target can follow and also make
	 * the thread current. This is an assumption that cannot hold for a real
	 * target running a multi-threading OS. If an RTOS can do this, override
	 * needs_fake_step(). */
	bool (*needs_fake_step)(struct target *target, threadid_t thread_id);
	/* Implement these if different threads in the RTOS can see memory
	 * differently (for instance because address translation might be different
	 * for each thread). */
	int (*read_buffer)(struct rtos *rtos, target_addr_t address, uint32_t size,
			uint8_t *buffer);
	int (*write_buffer)(struct rtos *rtos, target_addr_t address, uint32_t size,
			const uint8_t *buffer);
	/* When a software breakpoint is set, it is set on only one target,
	 * because we assume memory is shared across them. By default this is the
	 * first target in the SMP group. Override this function to have
	 * breakpoint_add() use a different target. */
	struct target * (*swbp_target)(struct rtos *rtos, target_addr_t address,
				     uint32_t length, enum breakpoint_type type);
};

struct stack_register_offset {
	unsigned short number;		/* register number */
	signed short offset;		/* offset in bytes from stack head, or -1 to indicate
					 * register is not stacked, or -2 to indicate this is the
					 * stack pointer register */
	unsigned short width_bits;
};

struct rtos_register_stacking {
	unsigned stack_registers_size;
	int stack_growth_direction;
	/* The number of gdb general registers, in order. */
	unsigned char num_output_registers;
	/* Some targets require evaluating the stack to determine the
	 * actual stack pointer for a process.  If this field is NULL,
	 * just use stacking->stack_registers_size * stack_growth_direction
	 * to calculate adjustment.
	 */
	target_addr_t (*calculate_process_stack)(struct target *target,
		const uint8_t *stack_data,
		const struct rtos_register_stacking *stacking,
		target_addr_t stack_ptr);
	const struct stack_register_offset *register_offsets;
	/* Total number of registers on the stack, including the general ones. This
	 * may be 0 if there are no additional registers on the stack beyond the
	 * general ones. */
	unsigned total_register_count;
};

#define GDB_THREAD_PACKET_NOT_CONSUMED (-40)

int rtos_create(struct jim_getopt_info *goi, struct target *target);
void rtos_destroy(struct target *target);
int rtos_set_reg(struct connection *connection, int reg_num,
		uint8_t *reg_value);
int rtos_generic_stack_read(struct target *target,
		const struct rtos_register_stacking *stacking,
		target_addr_t stack_ptr,
		struct rtos_reg **reg_list,
		int *num_regs);
int rtos_generic_stack_read_reg(struct target *target,
								const struct rtos_register_stacking *stacking,
								target_addr_t stack_ptr,
								uint32_t reg_num, struct rtos_reg *reg);
int rtos_generic_stack_write_reg(struct target *target,
								const struct rtos_register_stacking *stacking,
								target_addr_t stack_ptr,
								uint32_t reg_num, uint8_t *reg_value);
int gdb_thread_packet(struct connection *connection, char const *packet, int packet_size);
int rtos_get_gdb_reg(struct connection *connection, int reg_num);
int rtos_get_gdb_reg_list(struct connection *connection);
int rtos_update_threads(struct target *target);
void rtos_free_threadlist(struct rtos *rtos);
int rtos_smp_init(struct target *target);
/*  function for handling symbol access */
int rtos_qsymbol(struct connection *connection, char const *packet, int packet_size);
bool rtos_needs_fake_step(struct target *target, threadid_t thread_id);
int rtos_read_buffer(struct target *target, target_addr_t address,
		uint32_t size, uint8_t *buffer);
int rtos_write_buffer(struct target *target, target_addr_t address,
		uint32_t size, const uint8_t *buffer);
struct target *rtos_swbp_target(struct target *target, target_addr_t address,
				uint32_t length, enum breakpoint_type type);
struct rtos *rtos_of_target(struct target *target);

#endif /* OPENOCD_RTOS_RTOS_H */
