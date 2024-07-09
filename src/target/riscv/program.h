/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_TARGET_RISCV_PROGRAM_H
#define OPENOCD_TARGET_RISCV_PROGRAM_H

#include "riscv.h"

#define RISCV_MAX_PROGBUF_SIZE 32
#define RISCV_REGISTER_COUNT 32
#define RISCV_DSCRATCH_COUNT 2

typedef enum {
	RISCV_PROGBUF_EXEC_RESULT_NOT_EXECUTED,
	RISCV_PROGBUF_EXEC_RESULT_UNKNOWN,
	RISCV_PROGBUF_EXEC_RESULT_EXCEPTION,
	RISCV_PROGBUF_EXEC_RESULT_UNKNOWN_ERROR,
	RISCV_PROGBUF_EXEC_RESULT_SUCCESS
} riscv_progbuf_exec_result_t;

/* The various RISC-V debug specifications all revolve around setting up
 * program buffers and executing them on the target.  This structure contains a
 * single program, which can then be executed on targets.  */
struct riscv_program {
	struct target *target;

	uint32_t progbuf[RISCV_MAX_PROGBUF_SIZE];

	/* Number of 32-bit instructions in the program. */
	size_t instruction_count;

	/* execution result of the program */
	/* TODO: remove this field. We should make it a parameter to riscv_program_exec */
	riscv_progbuf_exec_result_t execution_result;
};

/* Initializes a program with the header. */
int riscv_program_init(struct riscv_program *p, struct target *t);

/* Write the program to the program buffer. */
int riscv_program_write(struct riscv_program *program);

/* Executes a program, returning 0 if the program successfully executed.  Note
 * that this may cause registers to be saved or restored, which could result to
 * calls to things like riscv013_reg_save which itself could require a
 * program to execute.  That's OK, just make sure this eventually terminates.
 * */
int riscv_program_exec(struct riscv_program *p, struct target *t);

/* A lower level interface, you shouldn't use this unless you have a reason. */
int riscv_program_insert(struct riscv_program *p, riscv_insn_t i);

/* Helpers to assemble various instructions.  Return 0 on success.  These might
 * assemble into a multi-instruction sequence that overwrites some other
 * register, but those will be properly saved and restored. */
int riscv_program_ldr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno a, int o);
int riscv_program_lwr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno a, int o);
int riscv_program_lhr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno a, int o);
int riscv_program_lbr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno a, int o);
int riscv_program_load(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int o,
		unsigned int s);

int riscv_program_sdr(struct riscv_program *p, enum gdb_regno s, enum gdb_regno a, int o);
int riscv_program_swr(struct riscv_program *p, enum gdb_regno s, enum gdb_regno a, int o);
int riscv_program_shr(struct riscv_program *p, enum gdb_regno s, enum gdb_regno a, int o);
int riscv_program_sbr(struct riscv_program *p, enum gdb_regno s, enum gdb_regno a, int o);
int riscv_program_store(struct riscv_program *p, enum gdb_regno d, enum gdb_regno b, int o,
		unsigned int s);

int riscv_program_csrrsi(struct riscv_program *p, enum gdb_regno d, unsigned int z, enum gdb_regno csr);
int riscv_program_csrrci(struct riscv_program *p, enum gdb_regno d, unsigned int z, enum gdb_regno csr);
int riscv_program_csrr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno csr);
int riscv_program_csrw(struct riscv_program *p, enum gdb_regno s, enum gdb_regno csr);

int riscv_program_fence_i(struct riscv_program *p);
int riscv_program_fence_rw_rw(struct riscv_program *p);
int riscv_program_ebreak(struct riscv_program *p);

int riscv_program_addi(struct riscv_program *p, enum gdb_regno d, enum gdb_regno s, int16_t i);

#endif /* OPENOCD_TARGET_RISCV_PROGRAM_H */
