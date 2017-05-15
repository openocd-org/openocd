#ifndef RISCV_H
#define RISCV_H

struct riscv_program;

#include <stdint.h>
#include "opcodes.h"
#include "gdb_regs.h"

/* The register cache is staticly allocated. */
#define RISCV_MAX_HARTS 32
#define RISCV_MAX_REGISTERS 5000
#define RISCV_MAX_TRIGGERS 32

extern struct target_type riscv011_target;
extern struct target_type riscv013_target;

/*
 * Definitions shared by code supporting all RISC-V versions.
 */
typedef uint64_t riscv_reg_t;
typedef uint32_t riscv_insn_t;
typedef  int64_t riscv_addr_t;

enum riscv_halt_reason {
	RISCV_HALT_INTERRUPT,
	RISCV_HALT_BREAKPOINT,
	RISCV_HALT_SINGLESTEP,
};

typedef struct {
	unsigned dtm_version;
	struct command_context *cmd_ctx;
	void *version_specific;

	/* The number of harts on this system. */
	int hart_count;

	/* The hart that the RTOS thinks is currently being debugged. */
	int rtos_hartid;

	/* The hart that is currently being debugged.  Note that this is
	 * different than the hartid that the RTOS is expected to use.  This
	 * one will change all the time, it's more of a global argument to
	 * every function than an actual */
	int current_hartid;

	/* Enough space to store all the registers we might need to save. */
	/* FIXME: This should probably be a bunch of register caches. */
	uint64_t saved_registers[RISCV_MAX_HARTS][RISCV_MAX_REGISTERS];
	bool valid_saved_registers[RISCV_MAX_HARTS][RISCV_MAX_REGISTERS];

	/* The register cache points into here. */
	uint64_t reg_cache_values[RISCV_MAX_REGISTERS];
	
	/* It's possible that each core has a different supported ISA set. */
	int xlen[RISCV_MAX_HARTS];

	/* The number of triggers per hart. */
	int trigger_count[RISCV_MAX_HARTS];

	/* The address of the debug RAM buffer. */
	riscv_addr_t debug_buffer_addr[RISCV_MAX_HARTS];

	/* The number of entries in the debug buffer. */
	int debug_buffer_size[RISCV_MAX_HARTS];

	/* This avoids invalidating the register cache too often. */
	bool registers_initialized;

	/* Helper functions that target the various RISC-V debug spec
	 * implementations. */
	riscv_reg_t (*get_register)(struct target *, int, int);
	void (*set_register)(struct target *, int, int, uint64_t);
	void (*select_current_hart)(struct target *);
	bool (*is_halted)(struct target *target);
	void (*halt_current_hart)(struct target *);
	void (*resume_current_hart)(struct target *target);
	void (*step_current_hart)(struct target *target);
	void (*on_halt)(struct target *target);
	void (*on_resume)(struct target *target);
	void (*on_step)(struct target *target);
	enum riscv_halt_reason (*halt_reason)(struct target *target);
	void (*debug_buffer_enter)(struct target *target, struct riscv_program *program);
	void (*debug_buffer_leave)(struct target *target, struct riscv_program *program);
	void (*write_debug_buffer)(struct target *target, int i, riscv_insn_t d);
	riscv_insn_t (*read_debug_buffer)(struct target *target, int i);
	int (*execute_debug_buffer)(struct target *target);
	int (*dmi_write_u64_bits)(struct target *target);
	void (*fill_dmi_write_u64)(struct target *target, char *buf, int a, uint64_t d);
	void (*fill_dmi_read_u64)(struct target *target, char *buf, int a);
	void (*fill_dmi_nop_u64)(struct target *target, char *buf);
	void (*reset_current_hart)(struct target *target);
} riscv_info_t;

/* Everything needs the RISC-V specific info structure, so here's a nice macro
 * that provides that. */
static inline riscv_info_t *riscv_info(const struct target *target) __attribute__((unused));
static inline riscv_info_t *riscv_info(const struct target *target)
{ return target->arch_info; }
#define RISCV_INFO(R) riscv_info_t *R = riscv_info(target);

extern uint8_t ir_dtmcontrol[1];
extern struct scan_field select_dtmcontrol;
extern uint8_t ir_dbus[1];
extern struct scan_field select_dbus;
extern uint8_t ir_idcode[1];
extern struct scan_field select_idcode;

/*** OpenOCD Interface */
int riscv_openocd_poll(struct target *target);

int riscv_openocd_halt(struct target *target);

int riscv_openocd_resume(
	struct target *target,
	int current,
	uint32_t address,
	int handle_breakpoints, 
	int debug_execution
);

int riscv_openocd_step(
	struct target *target,
	int current,
	uint32_t address,
	int handle_breakpoints
);

int riscv_openocd_assert_reset(struct target *target);
int riscv_openocd_deassert_reset(struct target *target);

/*** RISC-V Interface ***/

/* Initializes the shared RISC-V structure. */
void riscv_info_init(riscv_info_t *r);

/* Run control, possibly for multiple harts.  The _all_harts versions resume
 * all the enabled harts, which when running in RTOS mode is all the harts on
 * the system. */
int riscv_halt_all_harts(struct target *target);
int riscv_halt_one_hart(struct target *target, int hartid);
int riscv_resume_all_harts(struct target *target);
int riscv_resume_one_hart(struct target *target, int hartid);
int riscv_reset_all_harts(struct target *target);
int riscv_reset_one_hart(struct target *target, int hartid);

/* Steps the hart that's currently selected in the RTOS, or if there is no RTOS
 * then the only hart. */
int riscv_step_rtos_hart(struct target *target);

/* Returns XLEN for the given (or current) hart. */
int riscv_xlen(const struct target *target);
int riscv_xlen_of_hart(const struct target *target, int hartid);

bool riscv_rtos_enabled(const struct target *target);

/* Sets the current hart, which is the hart that will actually be used when
 * issuing debug commands. */
void riscv_set_current_hartid(struct target *target, int hartid);
int riscv_current_hartid(const struct target *target);

/*** Support functions for the RISC-V 'RTOS', which provides multihart support
 * without requiring multiple targets.  */

/* When using the RTOS to debug, this selects the hart that is currently being
 * debugged.  This doesn't propogate to the hardware. */
void riscv_set_all_rtos_harts(struct target *target);
void riscv_set_rtos_hartid(struct target *target, int hartid);

/* Lists the number of harts in the system, which are assumed to be
 * concecutive and start with mhartid=0. */
int riscv_count_harts(struct target *target);

/* Returns TRUE if the target has the given register on the given hart.  */
bool riscv_has_register(struct target *target, int hartid, int regid);

/* Returns the value of the given register on the given hart.  32-bit registers
 * are zero extended to 64 bits.  */
void riscv_set_register(struct target *target, enum gdb_regno i, riscv_reg_t v);
void riscv_set_register_on_hart(struct target *target, int hid, enum gdb_regno rid, uint64_t v);
riscv_reg_t riscv_get_register(struct target *target, enum gdb_regno i);
riscv_reg_t riscv_get_register_on_hart(struct target *target, int hid, enum gdb_regno rid);

/* Checks the state of the current hart -- "is_halted" checks the actual
 * on-device register. */
bool riscv_is_halted(struct target *target);
enum riscv_halt_reason riscv_halt_reason(struct target *target, int hartid);

/* Returns the number of triggers availiable to either the current hart or to
 * the given hart. */
int riscv_count_triggers(struct target *target);
int riscv_count_triggers_of_hart(struct target *target, int hartid);

/* These helper functions let the generic program interface get target-specific
 * information. */
size_t riscv_debug_buffer_size(struct target *target);
riscv_addr_t riscv_debug_buffer_addr(struct target *target);

int riscv_debug_buffer_enter(struct target *target, struct riscv_program *program);
int riscv_debug_buffer_leave(struct target *target, struct riscv_program *program);

riscv_insn_t riscv_read_debug_buffer(struct target *target, int index);
riscv_addr_t riscv_read_debug_buffer_x(struct target *target, int index);
int riscv_write_debug_buffer(struct target *target, int index, riscv_insn_t insn);
int riscv_write_debug_buffer_x(struct target *target, int index, riscv_addr_t data);
int riscv_execute_debug_buffer(struct target *target);

void riscv_fill_dmi_nop_u64(struct target *target, char *buf);
void riscv_fill_dmi_write_u64(struct target *target, char *buf, int a, uint64_t d);
void riscv_fill_dmi_read_u64(struct target *target, char *buf, int a);
int riscv_dmi_write_u64_bits(struct target *target);

#endif
