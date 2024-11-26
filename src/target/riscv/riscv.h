/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_TARGET_RISCV_RISCV_H
#define OPENOCD_TARGET_RISCV_RISCV_H

struct riscv_program;

#include <stdint.h>
#include "opcodes.h"
#include "gdb_regs.h"
#include "jtag/jtag.h"
#include "target/semihosting_common.h"
#include "target/target.h"
#include "target/register.h"
#include <helper/command.h>
#include <helper/bits.h>

#define RISCV_COMMON_MAGIC	0x52495356U

#define RISCV_MAX_HARTS  ((int)BIT(20))
#define RISCV_MAX_TRIGGERS 32
#define RISCV_MAX_HWBPS 16
#define RISCV_MAX_DMS 100

#define DEFAULT_COMMAND_TIMEOUT_SEC 5

#define RISCV_SATP_MODE(xlen)  ((xlen) == 32 ? SATP32_MODE : SATP64_MODE)
#define RISCV_SATP_PPN(xlen)  ((xlen) == 32 ? SATP32_PPN : SATP64_PPN)
#define RISCV_HGATP_MODE(xlen)  ((xlen) == 32 ? HGATP32_MODE : HGATP64_MODE)
#define RISCV_HGATP_PPN(xlen)  ((xlen) == 32 ? HGATP32_PPN : HGATP64_PPN)
#define RISCV_PGSHIFT 12
#define RISCV_PGSIZE BIT(RISCV_PGSHIFT)
#define RISCV_PGBASE(addr) ((addr) & ~(RISCV_PGSIZE - 1))
#define RISCV_PGOFFSET(addr) ((addr) & (RISCV_PGSIZE - 1))

#define PG_MAX_LEVEL 5

#define RISCV_BATCH_ALLOC_SIZE 128

extern struct target_type riscv011_target;
extern struct target_type riscv013_target;

/*
 * Definitions shared by code supporting all RISC-V versions.
 */
typedef uint64_t riscv_reg_t;
typedef uint32_t riscv_insn_t;
typedef uint64_t riscv_addr_t;

typedef enum {
	YNM_MAYBE,
	YNM_YES,
	YNM_NO
} yes_no_maybe_t;

typedef enum riscv_mem_access_method {
	RISCV_MEM_ACCESS_PROGBUF,
	RISCV_MEM_ACCESS_SYSBUS,
	RISCV_MEM_ACCESS_ABSTRACT,
	RISCV_MEM_ACCESS_MAX_METHODS_NUM
} riscv_mem_access_method_t;

typedef enum riscv_virt2phys_mode {
	RISCV_VIRT2PHYS_MODE_HW,
	RISCV_VIRT2PHYS_MODE_SW,
	RISCV_VIRT2PHYS_MODE_OFF
} riscv_virt2phys_mode_t;

const char *riscv_virt2phys_mode_to_str(riscv_virt2phys_mode_t mode);

enum riscv_halt_reason {
	RISCV_HALT_INTERRUPT,
	RISCV_HALT_EBREAK,
	RISCV_HALT_SINGLESTEP,
	RISCV_HALT_TRIGGER,
	RISCV_HALT_UNKNOWN,
	RISCV_HALT_GROUP,
	RISCV_HALT_ERROR
};

enum riscv_isrmasking_mode {
	/* RISCV_ISRMASK_AUTO,	*/ /* not supported yet */
	RISCV_ISRMASK_OFF,
	/* RISCV_ISRMASK_ON,	*/ /* not supported yet */
	RISCV_ISRMASK_STEPONLY,
};

enum riscv_hart_state {
	RISCV_STATE_NON_EXISTENT,
	RISCV_STATE_RUNNING,
	RISCV_STATE_HALTED,
	RISCV_STATE_UNAVAILABLE
};

/* RISC-V-specific data assigned to a register. */
typedef struct {
	struct target *target;
	/* Abstract command's regno for a custom register. */
	unsigned int custom_number;
} riscv_reg_info_t;

#define RISCV_SAMPLE_BUF_TIMESTAMP_BEFORE	0x80
#define RISCV_SAMPLE_BUF_TIMESTAMP_AFTER	0x81
struct riscv_sample_buf {
	uint8_t *buf;
	unsigned int used;
	unsigned int size;
};

typedef struct {
	bool enabled;
	struct {
		bool enabled;
		target_addr_t address;
		uint32_t size_bytes;
	} bucket[16];
} riscv_sample_config_t;

typedef struct {
	struct list_head list;
	uint16_t low, high;
	char *name;
} range_list_t;

#define DTM_DTMCS_VERSION_UNKNOWN ((unsigned int)-1)
#define RISCV_TINFO_VERSION_UNKNOWN (-1)

#define RISCV013_DTMCS_ABITS_MIN 7
#define RISCV013_DTMCS_ABITS_MAX 32

struct reg_name_table {
	unsigned int num_entries;
	char **reg_names;
};

typedef struct riscv_mem_access_args {
	target_addr_t address;

	const uint8_t *write_buffer;
	uint8_t *read_buffer;

	uint32_t size;
	uint32_t count;
	uint32_t increment;
} riscv_mem_access_args_t;

static inline bool
riscv_mem_access_is_valid(const riscv_mem_access_args_t args)
{
	return !args.read_buffer != !args.write_buffer;
}

static inline bool
riscv_mem_access_is_read(const riscv_mem_access_args_t args)
{
	assert(riscv_mem_access_is_valid(args));
	return !args.write_buffer && args.read_buffer;
}

static inline bool
riscv_mem_access_is_write(const riscv_mem_access_args_t args)
{
	assert(riscv_mem_access_is_valid(args));
	return !args.read_buffer && args.write_buffer;
}


struct riscv_info {
	unsigned int common_magic;

	unsigned int dtm_version;

	struct command_context *cmd_ctx;
	void *version_specific;

	struct reg_name_table custom_register_names;
	char **reg_names;

	/* It's possible that each core has a different supported ISA set. */
	int xlen;
	/* TODO: use the value from the register cache instead. */
	riscv_reg_t misa;
	/* TODO: use the value from the register cache instead.
	 * Cached value of vlenb. 0 indicates there is no vector support.
	 * Note that you can have vector support without misa.V set, because
	 * Zve* extensions implement vector registers without setting misa.V. */
	unsigned int vlenb;

	/* The number of triggers per hart. */
	unsigned int trigger_count;

	/* Data structure to record known unsupported tdata1+tdata2 trigger CSR values.
	 * This is to avoid repetitive attempts to set trigger configurations that are already
	 * known to be unsupported in the HW.
	 * A separate data structure is created for each trigger. */
	struct list_head *wp_triggers_negative_cache;

	/* record the tinfo of each trigger */
	unsigned int trigger_tinfo[RISCV_MAX_TRIGGERS];

	/* Version of the implemented Sdtrig extension */
	int tinfo_version;

	/* Record if single-step is needed prior to resuming
	 * from a software breakpoint or trigger.
	 * Single-step is needed if the instruction that
	 * caused the halt was not retired. That is,
	 * when we halted "before" that instruction.
	 */
	bool need_single_step;

	/* For each physical trigger contains:
	 * -1: the hwbp is available
	 * -4: The trigger is used by the itrigger command
	 * -5: The trigger is used by the etrigger command
	 * >= 0: unique_id of the breakpoint/watchpoint that is using it.
	 * Note that in RTOS mode the triggers are the same across all harts the
	 * target controls, while otherwise only a single hart is controlled. */
	int64_t trigger_unique_id[RISCV_MAX_HWBPS];

	/* The unique id of the trigger that caused the most recent halt. If the
	 * most recent halt was not caused by a trigger, then this is -1. */
	int64_t trigger_hit;

	/* The configured approach to translate virtual addresses to physical */
	riscv_virt2phys_mode_t virt2phys_mode;

	bool triggers_enumerated;

	/* Decremented every scan, and when it reaches 0 we clear the learned
	 * delays, causing them to be relearned. Used for testing. */
	int reset_delays_wait;

	/* This target has been prepped and is ready to step/resume. */
	bool prepped;
	/* This target was selected using hasel. */
	bool selected;

	/* Used by riscv_openocd_poll(). */
	bool halted_needs_event_callback;
	enum target_event halted_callback_event;

	enum riscv_isrmasking_mode isrmask_mode;

	/* Helper functions that target the various RISC-V debug spec
	 * implementations. */
	int (*select_target)(struct target *target);
	int (*get_hart_state)(struct target *target, enum riscv_hart_state *state);
	/* Resume this target, as well as every other prepped target that can be
	 * resumed near-simultaneously. Clear the prepped flag on any target that
	 * was resumed. */
	int (*resume_go)(struct target *target);
	int (*step_current_hart)(struct target *target);

	/* These get called from riscv_poll_hart(), which is a house of cards
	 * together with openocd_poll(), so be careful not to upset things too
	 * much. */
	int (*handle_became_halted)(struct target *target,
		enum riscv_hart_state previous_riscv_state);
	int (*handle_became_running)(struct target *target,
		enum riscv_hart_state previous_riscv_state);
	int (*handle_became_unavailable)(struct target *target,
		enum riscv_hart_state previous_riscv_state);

	/* Called periodically (no guarantees about frequency), while there's
	 * nothing else going on. */
	int (*tick)(struct target *target);

	/* Get this target as ready as possible to resume, without actually
	 * resuming. */
	int (*resume_prep)(struct target *target);
	int (*halt_prep)(struct target *target);
	int (*halt_go)(struct target *target);
	int (*on_step)(struct target *target);
	enum riscv_halt_reason (*halt_reason)(struct target *target);
	int (*write_progbuf)(struct target *target, unsigned int index, riscv_insn_t d);
	riscv_insn_t (*read_progbuf)(struct target *target, unsigned int index);
	int (*execute_progbuf)(struct target *target, uint32_t *cmderr);
	int (*invalidate_cached_progbuf)(struct target *target);
	unsigned int (*get_dmi_address_bits)(const struct target *target);
	void (*fill_dmi_write)(const struct target *target, uint8_t *buf, uint32_t a, uint32_t d);
	void (*fill_dmi_read)(const struct target *target, uint8_t *buf, uint32_t a);
	void (*fill_dm_nop)(const struct target *target, uint8_t *buf);

	int (*authdata_read)(struct target *target, uint32_t *value, unsigned int index);
	int (*authdata_write)(struct target *target, uint32_t value, unsigned int index);

	int (*dmi_read)(struct target *target, uint32_t *value, uint32_t address);
	int (*dmi_write)(struct target *target, uint32_t address, uint32_t value);

	bool (*get_impebreak)(const struct target *target);
	unsigned int (*get_progbufsize)(const struct target *target);

	/* Get the DMI address of target's DM's register.
	 * The function should return the passed address
	 * if the target is not assigned a DM yet.
	 */
	uint32_t (*get_dmi_address)(const struct target *target, uint32_t dm_address);

	int (*sample_memory)(struct target *target,
						 struct riscv_sample_buf *buf,
						 riscv_sample_config_t *config,
						 int64_t until_ms);

	int (*access_memory)(struct target *target, const riscv_mem_access_args_t args);

	unsigned int (*data_bits)(struct target *target);

	COMMAND_HELPER((*print_info), struct target *target);

	/* Storage for arch_info of non-custom registers. */
	riscv_reg_info_t shared_reg_info;

	/* Storage for vector register types. */
	struct reg_data_type_vector vector_uint8;
	struct reg_data_type_vector vector_uint16;
	struct reg_data_type_vector vector_uint32;
	struct reg_data_type_vector vector_uint64;
	struct reg_data_type_vector vector_uint128;
	struct reg_data_type type_uint8_vector;
	struct reg_data_type type_uint16_vector;
	struct reg_data_type type_uint32_vector;
	struct reg_data_type type_uint64_vector;
	struct reg_data_type type_uint128_vector;
	struct reg_data_type_union_field vector_fields[5];
	struct reg_data_type_union vector_union;
	struct reg_data_type type_vector;

	bool *reserved_triggers;

	/* Memory access methods to use, ordered by priority, highest to lowest. */
	riscv_mem_access_method_t mem_access_methods[RISCV_MEM_ACCESS_MAX_METHODS_NUM];

	unsigned int num_enabled_mem_access_methods;

	/* Different memory regions may need different methods but single configuration is applied
	 * for all. Following flags are used to warn only once about failing memory access method. */
	bool mem_access_warn[RISCV_MEM_ACCESS_MAX_METHODS_NUM];

	/* In addition to the ones in the standard spec, we'll also expose additional
	 * CSRs in this list. */
	struct list_head expose_csr;
	/* Same, but for custom registers.
	 * Custom registers are for non-standard extensions and use abstract register numbers
	 * from range 0xc000 ... 0xffff. */
	struct list_head expose_custom;

	/* The list of registers to mark as "hidden". Hidden registers are available
	 * but do not appear in gdb targets description or reg command output. */
	struct list_head hide_csr;

	riscv_sample_config_t sample_config;
	struct riscv_sample_buf sample_buf;

	/* Track when we were last asked to do something substantial. */
	int64_t last_activity;

	yes_no_maybe_t vsew64_supported;

	bool range_trigger_fallback_encountered;

	bool wp_allow_equality_match_trigger;
	bool wp_allow_napot_trigger;
	bool wp_allow_ge_lt_trigger;

	bool autofence;
};

enum riscv_priv_mode {
	RISCV_MODE_M,
	RISCV_MODE_S,
	RISCV_MODE_U,
	RISCV_MODE_VS,
	RISCV_MODE_VU,
	N_RISCV_MODE
};

struct riscv_private_config {
	bool dcsr_ebreak_fields[N_RISCV_MODE];
};

static inline struct riscv_private_config
*riscv_private_config(const struct target *target)
{
	assert(target->private_config);
	return target->private_config;
}

COMMAND_HELPER(riscv_print_info_line, const char *section, const char *key,
			   unsigned int value);

typedef struct {
	uint8_t tunneled_dr_width;
	struct scan_field tunneled_dr[4];
} riscv_bscan_tunneled_scan_context_t;

typedef struct {
	const char *name;
	int level;
	unsigned int va_bits;
	/* log2(PTESIZE) */
	unsigned int pte_shift;
	unsigned int vpn_shift[PG_MAX_LEVEL];
	unsigned int vpn_mask[PG_MAX_LEVEL];
	unsigned int pte_ppn_shift[PG_MAX_LEVEL];
	unsigned int pte_ppn_mask[PG_MAX_LEVEL];
	unsigned int pa_ppn_shift[PG_MAX_LEVEL];
	unsigned int pa_ppn_mask[PG_MAX_LEVEL];
} virt2phys_info_t;

bool riscv_virt2phys_mode_is_hw(const struct target *target);
bool riscv_virt2phys_mode_is_sw(const struct target *target);

/* Wall-clock timeout for a command/access. Settable via RISC-V Target commands.*/
int riscv_get_command_timeout_sec(void);

/* Everything needs the RISC-V specific info structure, so here's a nice macro
 * that provides that. */
static inline struct riscv_info *riscv_info(const struct target *target) __attribute__((unused));
static inline struct riscv_info *riscv_info(const struct target *target)
{
	assert(target->arch_info);
	return target->arch_info;
}
#define RISCV_INFO(R) struct riscv_info *R = riscv_info(target);

static inline bool is_riscv(const struct riscv_info *riscv_info)
{
	return riscv_info->common_magic == RISCV_COMMON_MAGIC;
}

extern struct scan_field select_dtmcontrol;
extern struct scan_field select_dbus;
extern struct scan_field select_idcode;

int dtmcs_scan(struct jtag_tap *tap, uint32_t out, uint32_t *in_ptr);

extern struct scan_field *bscan_tunneled_select_dmi;
extern uint32_t bscan_tunneled_select_dmi_num_fields;
typedef enum { BSCAN_TUNNEL_NESTED_TAP, BSCAN_TUNNEL_DATA_REGISTER } bscan_tunnel_type_t;
extern uint8_t bscan_tunnel_ir_width;

void select_dmi_via_bscan(struct jtag_tap *tap);

/*** OpenOCD Interface */
int riscv_openocd_poll(struct target *target);

int riscv_halt(struct target *target);

int riscv_openocd_step(
	struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints
);

/*** RISC-V Interface ***/

bool riscv_supports_extension(const struct target *target, char letter);

/* Returns XLEN for the given (or current) hart. */
unsigned int riscv_xlen(const struct target *target);

/* Returns VLENB for the given (or current) hart. */
unsigned int riscv_vlenb(const struct target *target);

/*** Support functions for the RISC-V 'RTOS', which provides multihart support
 * without requiring multiple targets.  */

/* Checks the state of the current hart -- "is_halted" checks the actual
 * on-device register. */
int riscv_get_hart_state(struct target *target, enum riscv_hart_state *state);

/* These helper functions let the generic program interface get target-specific
 * information. */
size_t riscv_progbuf_size(struct target *target);

riscv_insn_t riscv_read_progbuf(struct target *target, int index);
int riscv_write_progbuf(struct target *target, int index, riscv_insn_t insn);
int riscv_execute_progbuf(struct target *target, uint32_t *cmderr);

void riscv_fill_dm_nop(const struct target *target, uint8_t *buf);
void riscv_fill_dmi_write(const struct target *target, uint8_t *buf, uint32_t a, uint32_t d);
void riscv_fill_dmi_read(const struct target *target, uint8_t *buf, uint32_t a);
unsigned int riscv_get_dmi_address_bits(const struct target *target);

uint32_t riscv_get_dmi_address(const struct target *target, uint32_t dm_address);

int riscv_enumerate_triggers(struct target *target);

int riscv_add_watchpoint(struct target *target, struct watchpoint *watchpoint);
int riscv_remove_watchpoint(struct target *target,
		struct watchpoint *watchpoint);

void riscv_semihosting_init(struct target *target);

enum semihosting_result riscv_semihosting(struct target *target, int *retval);

void riscv_add_bscan_tunneled_scan(struct jtag_tap *tap, const struct scan_field *field,
		riscv_bscan_tunneled_scan_context_t *ctxt);

int riscv_read_by_any_size(struct target *target, target_addr_t address, uint32_t size, uint8_t *buffer);
int riscv_write_by_any_size(struct target *target, target_addr_t address, uint32_t size, uint8_t *buffer);

int riscv_interrupts_disable(struct target *target, uint64_t ie_mask, uint64_t *old_mstatus);
int riscv_interrupts_restore(struct target *target, uint64_t old_mstatus);

#endif /* OPENOCD_TARGET_RISCV_RISCV_H */
