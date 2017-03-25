#ifndef TARGET__RISCV__PROGRAM_H
#define TARGET__RISCV__PROGRAM_H

#include "riscv.h"

#define RISCV_MAX_DEBUG_BUFFER_SIZE 32
#define RISCV_REGISTER_COUNT 32
#define RISCV_DSCRATCH_COUNT 2

/* The various RISC-V debug specifications all revolve around setting up
 * program buffers and executing them on the target.  This structure contains a
 * single program, which can then be executed on targets.  */
struct riscv_program {
	struct target *target;

	uint32_t debug_buffer[RISCV_MAX_DEBUG_BUFFER_SIZE];

	/* The debug buffer is allocated in two directions: instructions go at
	 * the start, while data goes at the end.  When they meet in the middle
	 * this blows up. */
	size_t instruction_count;
	size_t data_count;

	/* Side effects of executing this program.  These must be accounted for
	 * in order to maintain correct executing of the target system.  */
	bool writes_xreg[RISCV_REGISTER_COUNT];
	bool writes_memory;

	/* When a register is used it will be set in this array. */
	bool in_use[RISCV_REGISTER_COUNT];

	/* Remembers the registers that have been saved into dscratch
	 * registers.  These are restored */
	enum gdb_regno dscratch_saved[RISCV_DSCRATCH_COUNT];

	/* XLEN on the target. */
	int target_xlen;
};

/* Initializes a program with the header. */
int riscv_program_init(struct riscv_program *p, struct target *t);

/* Executes a program, returning 0 if the program successfully executed.  Note
 * that this may cause registers to be saved or restored, which could result to
 * calls to things like riscv_save_register which itself could require a
 * program to execute.  That's OK, just make sure this eventually terminates.
 * */
int riscv_program_exec(struct riscv_program *p, struct target *t);
int riscv_program_load(struct riscv_program *p, struct target *t);

/* Clears a program, removing all the state associated with it. */
int riscv_program_clear(struct riscv_program *p, struct target *t);

/* A lower level interface, you shouldn't use this unless you have a reason. */
int riscv_program_insert(struct riscv_program *p, riscv_insn_t i);

/* There is hardware support for saving at least one register.  This register
 * doesn't need to be saved/restored the usual way, which is useful during
 * early initialization when we can't save/restore arbitrary registerrs to host
 * memory. */
int riscv_program_save_to_dscratch(struct riscv_program *p, enum gdb_regno to_save);

/* Allocates data of various sizes.  Either returns the absolute physical
 * address or RISCV_PROGRAM_ALLOC_FAIL on failure. */
riscv_addr_t riscv_program_alloc_data(struct riscv_program *p, size_t bytes);
riscv_addr_t riscv_program_alloc_x(struct riscv_program *p);
riscv_addr_t riscv_program_alloc_d(struct riscv_program *p);
riscv_addr_t riscv_program_alloc_w(struct riscv_program *p);
riscv_addr_t riscv_program_alloc_h(struct riscv_program *p);
riscv_addr_t riscv_program_alloc_b(struct riscv_program *p);
#define RISCV_PROGRAM_ALLOC_FAIL ((riscv_addr_t)(-1))

/* Reads a word of memory from this program's internal view of the debug RAM.
 * This is what you want to use to get data back from the program after it
 * executes. */
riscv_insn_t riscv_program_read_ram(struct riscv_program *p, riscv_addr_t addr);
void riscv_program_write_ram(struct riscv_program *p, riscv_addr_t a, uint64_t d);

/* Helpers to assembly various instructions.  Return 0 on success.  These might
 * assembly into a multi-instruction sequence that overwrites some other
 * register, but those will be properly saved and restored. */
int riscv_program_lx(struct riscv_program *p, enum gdb_regno d, riscv_addr_t addr);
int riscv_program_ld(struct riscv_program *p, enum gdb_regno d, riscv_addr_t addr);
int riscv_program_lw(struct riscv_program *p, enum gdb_regno d, riscv_addr_t addr);
int riscv_program_lh(struct riscv_program *p, enum gdb_regno d, riscv_addr_t addr);
int riscv_program_lb(struct riscv_program *p, enum gdb_regno d, riscv_addr_t addr);

int riscv_program_sx(struct riscv_program *p, enum gdb_regno s, riscv_addr_t addr);
int riscv_program_sd(struct riscv_program *p, enum gdb_regno s, riscv_addr_t addr);
int riscv_program_sw(struct riscv_program *p, enum gdb_regno s, riscv_addr_t addr);
int riscv_program_sh(struct riscv_program *p, enum gdb_regno s, riscv_addr_t addr);
int riscv_program_sb(struct riscv_program *p, enum gdb_regno s, riscv_addr_t addr);

int riscv_program_lxr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno a, int o);
int riscv_program_ldr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno a, int o);
int riscv_program_lwr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno a, int o);
int riscv_program_lhr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno a, int o);
int riscv_program_lbr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno a, int o);

int riscv_program_sxr(struct riscv_program *p, enum gdb_regno s, enum gdb_regno a, int o);
int riscv_program_sdr(struct riscv_program *p, enum gdb_regno s, enum gdb_regno a, int o);
int riscv_program_swr(struct riscv_program *p, enum gdb_regno s, enum gdb_regno a, int o);
int riscv_program_shr(struct riscv_program *p, enum gdb_regno s, enum gdb_regno a, int o);
int riscv_program_sbr(struct riscv_program *p, enum gdb_regno s, enum gdb_regno a, int o);

int riscv_program_csrr(struct riscv_program *p, enum gdb_regno d, enum gdb_regno csr);
int riscv_program_csrw(struct riscv_program *p, enum gdb_regno s, enum gdb_regno csr);
int riscv_program_csrrw(struct riscv_program *p, enum gdb_regno d, enum gdb_regno s, enum gdb_regno csr);

int riscv_program_fence_i(struct riscv_program *p);
int riscv_program_fence(struct riscv_program *p);
int riscv_program_ebreak(struct riscv_program *p);

int riscv_program_lui(struct riscv_program *p, enum gdb_regno d, int32_t u);
int riscv_program_addi(struct riscv_program *p, enum gdb_regno d, enum gdb_regno s, int16_t i);

int riscv_program_fsd(struct riscv_program *p, enum gdb_regno s, riscv_addr_t addr);
int riscv_program_fld(struct riscv_program *p, enum gdb_regno d, riscv_addr_t addr);

/* Assembler macros. */
int riscv_program_li(struct riscv_program *p, enum gdb_regno d, riscv_reg_t c);
int riscv_program_la(struct riscv_program *p, enum gdb_regno d, riscv_addr_t a);

/* Register allocation.  The user is expected to have obtained temporary
 * registers using these fuctions.  Additionally, there is an interface for
 * reserving registers -- it's expected that this has been called as the first
 * thing in the program's execution to reserve registers that can't be touched
 * by the program's execution. */
void riscv_program_reserve_register(struct riscv_program *p, enum gdb_regno r);
enum gdb_regno riscv_program_gettemp(struct riscv_program *p);
void riscv_program_puttemp(struct riscv_program *p, enum gdb_regno r);

/* Executing a program usually causes the registers that get overwritten to be
 * saved and restored.  Calling this prevents the given register from actually
 * being restored as a result of all activity in this program. */
int riscv_program_dont_restore_register(struct riscv_program *p, enum gdb_regno r);
int riscv_program_do_restore_register(struct riscv_program *p, enum gdb_regno r);

/* Addressing functions. */
riscv_addr_t riscv_program_gah(struct riscv_program *p, riscv_addr_t addr);

#endif
