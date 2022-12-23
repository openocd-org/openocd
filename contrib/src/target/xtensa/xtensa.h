/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Generic Xtensa target                                                 *
 *   Copyright (C) 2020-2022 Cadence Design Systems, Inc.                  *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_XTENSA_H
#define OPENOCD_TARGET_XTENSA_H

#include "assert.h"
#include <target/target.h>
#include <target/breakpoints.h>
#include "xtensa_regs.h"
#include "xtensa_debug_module.h"

/**
 * @file
 * Holds the interface to Xtensa cores.
 */

/* Big-endian vs. little-endian detection */
#define XT_ISBE(X)                              ((X)->target->endianness == TARGET_BIG_ENDIAN)

/* 24-bit break; BE version field-swapped then byte-swapped for use in memory R/W fns */
#define XT_INS_BREAK_LE(S, T)                   (0x004000 | (((S) & 0xF) << 8) | (((T) & 0xF) << 4))
#define XT_INS_BREAK_BE(S, T)                   (0x000400 | (((S) & 0xF) << 12) | ((T) & 0xF))
#define XT_INS_BREAK(X, S, T)                   (XT_ISBE(X) ? XT_INS_BREAK_BE(S, T) : XT_INS_BREAK_LE(S, T))

/* 16-bit break; BE version field-swapped then byte-swapped for use in memory R/W fns */
#define XT_INS_BREAKN_LE(IMM4)                  (0xF02D | (((IMM4) & 0xF) << 8))
#define XT_INS_BREAKN_BE(IMM4)                  (0x0FD2 | (((IMM4) & 0xF) << 12))
#define XT_INS_BREAKN(X, IMM4)                  (XT_ISBE(X) ? XT_INS_BREAKN_BE(IMM4) : XT_INS_BREAKN_LE(IMM4))

#define XT_ISNS_SZ_MAX                          3

#define XT_PS_RING(_v_)                         ((uint32_t)((_v_) & 0x3) << 6)
#define XT_PS_RING_MSK                          (0x3 << 6)
#define XT_PS_RING_GET(_v_)                     (((_v_) >> 6) & 0x3)
#define XT_PS_CALLINC_MSK                       (0x3 << 16)
#define XT_PS_OWB_MSK                           (0xF << 8)
#define XT_PS_WOE_MSK                           BIT(18)

#define XT_LOCAL_MEM_REGIONS_NUM_MAX            8

#define XT_AREGS_NUM_MAX                        64
#define XT_USER_REGS_NUM_MAX                    256

#define XT_MEM_ACCESS_NONE                      0x0
#define XT_MEM_ACCESS_READ                      0x1
#define XT_MEM_ACCESS_WRITE                     0x2

#define XT_MAX_TIE_REG_WIDTH                    (512)	/* TIE register file max 4096 bits */
#define XT_QUERYPKT_RESP_MAX                    (XT_MAX_TIE_REG_WIDTH * 2 + 1)

enum xtensa_qerr_e {
	XT_QERR_INTERNAL = 0,
	XT_QERR_FAIL,
	XT_QERR_INVAL,
	XT_QERR_MEM,
	XT_QERR_NUM,
};

/* An and ARn registers potentially used as scratch regs */
enum xtensa_ar_scratch_set_e {
	XT_AR_SCRATCH_A3 = 0,
	XT_AR_SCRATCH_AR3,
	XT_AR_SCRATCH_A4,
	XT_AR_SCRATCH_AR4,
	XT_AR_SCRATCH_NUM
};

struct xtensa_keyval_info_s {
	char *chrval;
	int intval;
};

enum xtensa_type {
	XT_UNDEF = 0,
	XT_LX,
};

struct xtensa_cache_config {
	uint8_t way_count;
	uint32_t line_size;
	uint32_t size;
	int writeback;
};

struct xtensa_local_mem_region_config {
	target_addr_t base;
	uint32_t size;
	int access;
};

struct xtensa_local_mem_config {
	uint16_t count;
	struct xtensa_local_mem_region_config regions[XT_LOCAL_MEM_REGIONS_NUM_MAX];
};

struct xtensa_mmu_config {
	bool enabled;
	uint8_t itlb_entries_count;
	uint8_t dtlb_entries_count;
};

struct xtensa_mpu_config {
	bool enabled;
	uint8_t nfgseg;
	uint32_t minsegsize;
	bool lockable;
	bool execonly;
};

struct xtensa_irq_config {
	bool enabled;
	uint8_t irq_num;
};

struct xtensa_high_prio_irq_config {
	bool enabled;
	uint8_t level_num;
	uint8_t excm_level;
};

struct xtensa_debug_config {
	bool enabled;
	uint8_t irq_level;
	uint8_t ibreaks_num;
	uint8_t dbreaks_num;
	uint8_t perfcount_num;
};

struct xtensa_tracing_config {
	bool enabled;
	uint32_t mem_sz;
	bool reversed_mem_access;
};

struct xtensa_config {
	enum xtensa_type core_type;
	uint8_t aregs_num;
	bool windowed;
	bool coproc;
	bool exceptions;
	struct xtensa_irq_config irq;
	struct xtensa_high_prio_irq_config high_irq;
	struct xtensa_mmu_config mmu;
	struct xtensa_mpu_config mpu;
	struct xtensa_debug_config debug;
	struct xtensa_tracing_config trace;
	struct xtensa_cache_config icache;
	struct xtensa_cache_config dcache;
	struct xtensa_local_mem_config irom;
	struct xtensa_local_mem_config iram;
	struct xtensa_local_mem_config drom;
	struct xtensa_local_mem_config dram;
	struct xtensa_local_mem_config sram;
	struct xtensa_local_mem_config srom;
};

typedef uint32_t xtensa_insn_t;

enum xtensa_stepping_isr_mode {
	XT_STEPPING_ISR_OFF,	/* interrupts are disabled during stepping */
	XT_STEPPING_ISR_ON,		/* interrupts are enabled during stepping */
};

/* Only supported in cores with in-CPU MMU. None of Espressif chips as of now. */
enum xtensa_mode {
	XT_MODE_RING0,
	XT_MODE_RING1,
	XT_MODE_RING2,
	XT_MODE_RING3,
	XT_MODE_ANY	/* special value to run algorithm in current core mode */
};

struct xtensa_sw_breakpoint {
	struct breakpoint *oocd_bp;
	/* original insn */
	uint8_t insn[XT_ISNS_SZ_MAX];
	/* original insn size */
	uint8_t insn_sz;	/* 2 or 3 bytes */
};

#define XTENSA_COMMON_MAGIC 0x54E4E555U

/**
 * Represents a generic Xtensa core.
 */
struct xtensa {
	unsigned int common_magic;
	struct xtensa_chip_common *xtensa_chip;
	struct xtensa_config *core_config;
	struct xtensa_debug_module dbg_mod;
	struct reg_cache *core_cache;
	unsigned int total_regs_num;
	unsigned int core_regs_num;
	bool regmap_contiguous;
	unsigned int genpkt_regs_num;
	struct xtensa_reg_desc **contiguous_regs_desc;
	struct reg **contiguous_regs_list;
	/* Per-config Xtensa registers as specified via "xtreg" in xtensa-core*.cfg */
	struct xtensa_reg_desc *optregs;
	unsigned int num_optregs;
	struct reg *empty_regs;
	char qpkt_resp[XT_QUERYPKT_RESP_MAX];
	/* An array of pointers to buffers to backup registers' values while algo is run on target.
	 * Size is 'regs_num'. */
	void **algo_context_backup;
	unsigned int eps_dbglevel_idx;
	unsigned int dbregs_num;
	struct target *target;
	bool reset_asserted;
	enum xtensa_stepping_isr_mode stepping_isr_mode;
	struct breakpoint **hw_brps;
	struct watchpoint **hw_wps;
	struct xtensa_sw_breakpoint *sw_brps;
	bool trace_active;
	bool permissive_mode;	/* bypass memory checks */
	bool suppress_dsr_errors;
	uint32_t smp_break;
	uint32_t spill_loc;
	unsigned int spill_bytes;
	uint8_t *spill_buf;
	int8_t probe_lsddr32p;
	/* Sometimes debug module's 'powered' bit is cleared after reset, but get set after some
	 * time.This is the number of polling periods after which core is considered to be powered
	 * off (marked as unexamined) if the bit retains to be cleared (e.g. if core is disabled by
	 * SW running on target).*/
	uint8_t come_online_probes_num;
	bool proc_syscall;
	bool halt_request;
	struct xtensa_keyval_info_s scratch_ars[XT_AR_SCRATCH_NUM];
	bool regs_fetched;	/* true after first register fetch completed successfully */
};

static inline struct xtensa *target_to_xtensa(struct target *target)
{
	assert(target);
	struct xtensa *xtensa = target->arch_info;
	assert(xtensa->common_magic == XTENSA_COMMON_MAGIC);
	return xtensa;
}

int xtensa_init_arch_info(struct target *target,
	struct xtensa *xtensa,
	const struct xtensa_debug_module_config *dm_cfg);
int xtensa_target_init(struct command_context *cmd_ctx, struct target *target);
void xtensa_target_deinit(struct target *target);

static inline bool xtensa_addr_in_mem(const struct xtensa_local_mem_config *mem, uint32_t addr)
{
	for (unsigned int i = 0; i < mem->count; i++) {
		if (addr >= mem->regions[i].base &&
			addr < mem->regions[i].base + mem->regions[i].size)
			return true;
	}
	return false;
}

static inline bool xtensa_data_addr_valid(struct target *target, uint32_t addr)
{
	struct xtensa *xtensa = target_to_xtensa(target);

	if (xtensa_addr_in_mem(&xtensa->core_config->drom, addr))
		return true;
	if (xtensa_addr_in_mem(&xtensa->core_config->dram, addr))
		return true;
	if (xtensa_addr_in_mem(&xtensa->core_config->sram, addr))
		return true;
	return false;
}

static inline int xtensa_queue_dbg_reg_read(struct xtensa *xtensa, enum xtensa_dm_reg reg, uint8_t *data)
{
	struct xtensa_debug_module *dm = &xtensa->dbg_mod;

	if (!xtensa->core_config->trace.enabled &&
		(reg <= XDMREG_MEMADDREND || (reg >= XDMREG_PMG && reg <= XDMREG_PMSTAT7))) {
		LOG_ERROR("Can not access %u reg when Trace Port option disabled!", reg);
		return ERROR_FAIL;
	}
	return dm->dbg_ops->queue_reg_read(dm, reg, data);
}

static inline int xtensa_queue_dbg_reg_write(struct xtensa *xtensa, enum xtensa_dm_reg reg, uint32_t data)
{
	struct xtensa_debug_module *dm = &xtensa->dbg_mod;

	if (!xtensa->core_config->trace.enabled &&
		(reg <= XDMREG_MEMADDREND || (reg >= XDMREG_PMG && reg <= XDMREG_PMSTAT7))) {
		LOG_ERROR("Can not access %u reg when Trace Port option disabled!", reg);
		return ERROR_FAIL;
	}
	return dm->dbg_ops->queue_reg_write(dm, reg, data);
}

static inline int xtensa_core_status_clear(struct target *target, uint32_t bits)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	return xtensa_dm_core_status_clear(&xtensa->dbg_mod, bits);
}

int xtensa_core_status_check(struct target *target);

int xtensa_examine(struct target *target);
int xtensa_wakeup(struct target *target);
int xtensa_smpbreak_set(struct target *target, uint32_t set);
int xtensa_smpbreak_get(struct target *target, uint32_t *val);
int xtensa_smpbreak_write(struct xtensa *xtensa, uint32_t set);
int xtensa_smpbreak_read(struct xtensa *xtensa, uint32_t *val);
xtensa_reg_val_t xtensa_reg_get(struct target *target, enum xtensa_reg_id reg_id);
void xtensa_reg_set(struct target *target, enum xtensa_reg_id reg_id, xtensa_reg_val_t value);
void xtensa_reg_set_deep_relgen(struct target *target, enum xtensa_reg_id a_idx, xtensa_reg_val_t value);
int xtensa_fetch_all_regs(struct target *target);
int xtensa_get_gdb_reg_list(struct target *target,
	struct reg **reg_list[],
	int *reg_list_size,
	enum target_register_class reg_class);
uint32_t xtensa_cause_get(struct target *target);
void xtensa_cause_clear(struct target *target);
void xtensa_cause_reset(struct target *target);
int xtensa_poll(struct target *target);
void xtensa_on_poll(struct target *target);
int xtensa_halt(struct target *target);
int xtensa_resume(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints,
	int debug_execution);
int xtensa_prepare_resume(struct target *target,
	int current,
	target_addr_t address,
	int handle_breakpoints,
	int debug_execution);
int xtensa_do_resume(struct target *target);
int xtensa_step(struct target *target, int current, target_addr_t address, int handle_breakpoints);
int xtensa_do_step(struct target *target, int current, target_addr_t address, int handle_breakpoints);
int xtensa_mmu_is_enabled(struct target *target, int *enabled);
int xtensa_read_memory(struct target *target, target_addr_t address, uint32_t size, uint32_t count, uint8_t *buffer);
int xtensa_read_buffer(struct target *target, target_addr_t address, uint32_t count, uint8_t *buffer);
int xtensa_write_memory(struct target *target,
	target_addr_t address,
	uint32_t size,
	uint32_t count,
	const uint8_t *buffer);
int xtensa_write_buffer(struct target *target, target_addr_t address, uint32_t count, const uint8_t *buffer);
int xtensa_checksum_memory(struct target *target, target_addr_t address, uint32_t count, uint32_t *checksum);
int xtensa_assert_reset(struct target *target);
int xtensa_deassert_reset(struct target *target);
int xtensa_soft_reset_halt(struct target *target);
int xtensa_breakpoint_add(struct target *target, struct breakpoint *breakpoint);
int xtensa_breakpoint_remove(struct target *target, struct breakpoint *breakpoint);
int xtensa_watchpoint_add(struct target *target, struct watchpoint *watchpoint);
int xtensa_watchpoint_remove(struct target *target, struct watchpoint *watchpoint);
void xtensa_set_permissive_mode(struct target *target, bool state);
const char *xtensa_get_gdb_arch(struct target *target);
int xtensa_gdb_query_custom(struct target *target, const char *packet, char **response_p);

COMMAND_HELPER(xtensa_cmd_xtdef_do, struct xtensa *xtensa);
COMMAND_HELPER(xtensa_cmd_xtopt_do, struct xtensa *xtensa);
COMMAND_HELPER(xtensa_cmd_xtmem_do, struct xtensa *xtensa);
COMMAND_HELPER(xtensa_cmd_xtmpu_do, struct xtensa *xtensa);
COMMAND_HELPER(xtensa_cmd_xtmmu_do, struct xtensa *xtensa);
COMMAND_HELPER(xtensa_cmd_xtreg_do, struct xtensa *xtensa);
COMMAND_HELPER(xtensa_cmd_xtregfmt_do, struct xtensa *xtensa);
COMMAND_HELPER(xtensa_cmd_permissive_mode_do, struct xtensa *xtensa);
COMMAND_HELPER(xtensa_cmd_mask_interrupts_do, struct xtensa *xtensa);
COMMAND_HELPER(xtensa_cmd_smpbreak_do, struct target *target);
COMMAND_HELPER(xtensa_cmd_perfmon_dump_do, struct xtensa *xtensa);
COMMAND_HELPER(xtensa_cmd_perfmon_enable_do, struct xtensa *xtensa);
COMMAND_HELPER(xtensa_cmd_tracestart_do, struct xtensa *xtensa);
COMMAND_HELPER(xtensa_cmd_tracestop_do, struct xtensa *xtensa);
COMMAND_HELPER(xtensa_cmd_tracedump_do, struct xtensa *xtensa, const char *fname);

extern const struct command_registration xtensa_command_handlers[];

#endif	/* OPENOCD_TARGET_XTENSA_H */
