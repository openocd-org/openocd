/***************************************************************************
 *   Generic Xtensa target                                                 *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
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

#define XT_ISNS_SZ_MAX                  3

#define XT_PS_RING(_v_)                 ((uint32_t)((_v_) & 0x3) << 6)
#define XT_PS_RING_MSK                  (0x3 << 6)
#define XT_PS_RING_GET(_v_)             (((_v_) >> 6) & 0x3)
#define XT_PS_CALLINC_MSK               (0x3 << 16)
#define XT_PS_OWB_MSK                   (0xF << 8)

#define XT_LOCAL_MEM_REGIONS_NUM_MAX    8

#define XT_AREGS_NUM_MAX                64
#define XT_USER_REGS_NUM_MAX            256

#define XT_MEM_ACCESS_NONE              0x0
#define XT_MEM_ACCESS_READ              0x1
#define XT_MEM_ACCESS_WRITE             0x2

enum xtensa_mem_err_detect {
	XT_MEM_ERR_DETECT_NONE,
	XT_MEM_ERR_DETECT_PARITY,
	XT_MEM_ERR_DETECT_ECC,
};

struct xtensa_cache_config {
	uint8_t way_count;
	uint8_t line_size;
	uint16_t size;
	bool writeback;
	enum xtensa_mem_err_detect mem_err_check;
};

struct xtensa_local_mem_region_config {
	target_addr_t base;
	uint32_t size;
	enum xtensa_mem_err_detect mem_err_check;
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
	bool ivarway56;
	bool dvarway56;
};

struct xtensa_exception_config {
	bool enabled;
	uint8_t depc_num;
};

struct xtensa_irq_config {
	bool enabled;
	uint8_t irq_num;
};

struct xtensa_high_prio_irq_config {
	bool enabled;
	uint8_t excm_level;
	uint8_t nmi_num;
};

struct xtensa_debug_config {
	bool enabled;
	uint8_t irq_level;
	uint8_t ibreaks_num;
	uint8_t dbreaks_num;
	uint8_t icount_sz;
};

struct xtensa_tracing_config {
	bool enabled;
	uint32_t mem_sz;
	bool reversed_mem_access;
};

struct xtensa_timer_irq_config {
	bool enabled;
	uint8_t comp_num;
};

struct xtensa_config {
	bool density;
	uint8_t aregs_num;
	bool windowed;
	bool coproc;
	bool fp_coproc;
	bool loop;
	uint8_t miscregs_num;
	bool threadptr;
	bool boolean;
	bool cond_store;
	bool ext_l32r;
	bool mac16;
	bool reloc_vec;
	bool proc_id;
	bool mem_err_check;
	uint16_t user_regs_num;
	const struct xtensa_user_reg_desc *user_regs;
	int (*fetch_user_regs)(struct target *target);
	int (*queue_write_dirty_user_regs)(struct target *target);
	struct xtensa_cache_config icache;
	struct xtensa_cache_config dcache;
	struct xtensa_local_mem_config irom;
	struct xtensa_local_mem_config iram;
	struct xtensa_local_mem_config drom;
	struct xtensa_local_mem_config dram;
	struct xtensa_local_mem_config uram;
	struct xtensa_local_mem_config xlmi;
	struct xtensa_mmu_config mmu;
	struct xtensa_exception_config exc;
	struct xtensa_irq_config irq;
	struct xtensa_high_prio_irq_config high_irq;
	struct xtensa_timer_irq_config tim_irq;
	struct xtensa_debug_config debug;
	struct xtensa_tracing_config trace;
	unsigned int gdb_general_regs_num;
	const unsigned int *gdb_regs_mapping;
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
	const struct xtensa_config *core_config;
	struct xtensa_debug_module dbg_mod;
	struct reg_cache *core_cache;
	unsigned int regs_num;
	/* An array of pointers to buffers to backup registers' values while algo is run on target.
	 * Size is 'regs_num'. */
	void **algo_context_backup;
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
	/* Sometimes debug module's 'powered' bit is cleared after reset, but get set after some
	 * time.This is the number of polling periods after which core is considered to be powered
	 * off (marked as unexamined) if the bit retains to be cleared (e.g. if core is disabled by
	 * SW running on target).*/
	uint8_t come_online_probes_num;
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
	const struct xtensa_config *cfg,
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
	if (xtensa_addr_in_mem(&xtensa->core_config->uram, addr))
		return true;
	return false;
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
int xtensa_fetch_all_regs(struct target *target);
int xtensa_get_gdb_reg_list(struct target *target,
	struct reg **reg_list[],
	int *reg_list_size,
	enum target_register_class reg_class);
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
int xtensa_breakpoint_add(struct target *target, struct breakpoint *breakpoint);
int xtensa_breakpoint_remove(struct target *target, struct breakpoint *breakpoint);
int xtensa_watchpoint_add(struct target *target, struct watchpoint *watchpoint);
int xtensa_watchpoint_remove(struct target *target, struct watchpoint *watchpoint);
void xtensa_set_permissive_mode(struct target *target, bool state);
int xtensa_fetch_user_regs_u32(struct target *target);
int xtensa_queue_write_dirty_user_regs_u32(struct target *target);
const char *xtensa_get_gdb_arch(struct target *target);


COMMAND_HELPER(xtensa_cmd_permissive_mode_do, struct xtensa *xtensa);
COMMAND_HELPER(xtensa_cmd_mask_interrupts_do, struct xtensa *xtensa);
COMMAND_HELPER(xtensa_cmd_smpbreak_do, struct target *target);
COMMAND_HELPER(xtensa_cmd_perfmon_dump_do, struct xtensa *xtensa);
COMMAND_HELPER(xtensa_cmd_perfmon_enable_do, struct xtensa *xtensa);
COMMAND_HELPER(xtensa_cmd_tracestart_do, struct xtensa *xtensa);
COMMAND_HELPER(xtensa_cmd_tracestop_do, struct xtensa *xtensa);
COMMAND_HELPER(xtensa_cmd_tracedump_do, struct xtensa *xtensa, const char *fname);

extern const struct reg_arch_type xtensa_user_reg_u32_type;
extern const struct reg_arch_type xtensa_user_reg_u128_type;
extern const struct command_registration xtensa_command_handlers[];

#endif	/* OPENOCD_TARGET_XTENSA_H */
