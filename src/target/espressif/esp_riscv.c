/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP RISCV common definitions for OpenOCD                              *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif


#include <stdbool.h>
#include <stdint.h>

#include <helper/bits.h>
#include <helper/align.h>
#include <target/target.h>
#include <target/target_type.h>
#include <target/smp.h>
#include <target/semihosting_common.h>
#include <rtos/rtos.h>

#include "esp_riscv.h"
#include "esp_semihosting.h"

/* Argument indexes for ESP_SEMIHOSTING_SYS_BREAKPOINT_SET */
enum {
	ESP_RISCV_SET_BREAKPOINT_ARG_SET,
	ESP_RISCV_SET_BREAKPOINT_ARG_ID,
	ESP_RISCV_SET_BREAKPOINT_ARG_ADDR,	/* missed if `set` is false */
	ESP_RISCV_SET_BREAKPOINT_ARG_MAX
};

/* Argument indexes for ESP_SEMIHOSTING_SYS_WATCHPOINT_SET */
enum {
	ESP_RISCV_SET_WATCHPOINT_ARG_SET,
	ESP_RISCV_SET_WATCHPOINT_ARG_ID,
	ESP_RISCV_SET_WATCHPOINT_ARG_ADDR,	/* missed if `set` is false */
	ESP_RISCV_SET_WATCHPOINT_ARG_SIZE,	/* missed if `set` is false */
	ESP_RISCV_SET_WATCHPOINT_ARG_FLAGS,	/* missed if `set` is false */
	ESP_RISCV_SET_WATCHPOINT_ARG_MAX
};

enum esp_riscv_exception_cause {
	INSTR_ADDR_MISALIGNED = 0x0,
	PMP_INSTRUCTION_ACCESS_FAULT = 0x1,
	ILLEGAL_INSTRUCTION = 0x2,
	HARDWARE_BREAKPOINT = 0x3,
	LOAD_ADDR_MISALIGNED = 0x4,
	PMP_LOAD_ACCESS_FAULT = 0x5,
	STORE_ADDR_MISALIGNED = 0x6,
	PMP_STORE_ACCESS_FAULT = 0x7,
	ECALL_FROM_U_MODE = 0x8,
	ECALL_FROM_S_MODE = 0x9,
	ECALL_FROM_M_MODE = 0xb,
	INSTR_PAGE_FAULT = 0xc,
	LOAD_PAGE_FAULT = 0xd,
	STORE_PAGE_FAULT = 0xf,
};

#define ESP_RISCV_EXCEPTION_CAUSE(reg_val)  ((reg_val) & 0x1F)

#define ESP_RISCV_LOAD_FP     0x07
#define ESP_RISCV_STORE_FP    0x27
#define ESP_RISCV_OP_FP       0x53

#define ESP_SEMIHOSTING_WP_FLG_RD   (1UL << 0)
#define ESP_SEMIHOSTING_WP_FLG_WR   (1UL << 1)

#define ESP_RISCV_DBGSTUBS_UPDATE_DATA_ENTRY(_e_) \
	do { \
		(_e_) = buf_get_u32((uint8_t *)&(_e_), 0, 32); \
		if ((_e_) == 0) { \
			LOG_WARNING("No valid stub data entry found (0x%x)!", (uint32_t)(_e_));	\
		} \
	} while (0)

#define ESP_RISCV_DBGSTUBS_UPDATE_CODE_ENTRY(_e_) \
	do { \
		(_e_) = buf_get_u32((uint8_t *)&(_e_), 0, 32); \
		if ((_e_) == 0) { \
			LOG_WARNING("No valid stub code entry found (0x%x)!", (uint32_t)(_e_));	\
		} \
	} while (0)

static int esp_riscv_debug_stubs_info_init(struct target *target,
	target_addr_t ctrl_addr);

static const char *esp_riscv_get_exception_reason(enum esp_riscv_exception_cause exception_code)
{
	switch (ESP_RISCV_EXCEPTION_CAUSE(exception_code)) {
	case INSTR_ADDR_MISALIGNED:
		return "Instruction address misaligned";
	case PMP_INSTRUCTION_ACCESS_FAULT:
		return "PMP Instruction access fault";
	case ILLEGAL_INSTRUCTION:
		return "Illegal Instruction";
	case HARDWARE_BREAKPOINT:
		return "Hardware Breakpoint/Watchpoint or EBREAK";
	case LOAD_ADDR_MISALIGNED:
		return "Load address misaligned";
	case PMP_LOAD_ACCESS_FAULT:
		return "PMP Load access fault";
	case PMP_STORE_ACCESS_FAULT:
		return "PMP Store access fault";
	case ECALL_FROM_U_MODE:
		return "ECALL from U mode";
	case ECALL_FROM_S_MODE:
		return "ECALL from S-mode";
	case ECALL_FROM_M_MODE:
		return "ECALL from M mode";
	case INSTR_PAGE_FAULT:
		return "Instruction page fault";
	case LOAD_PAGE_FAULT:
		return "Load page fault";
	case STORE_PAGE_FAULT:
		return "Store page fault";
	}
	return "Unknown exception cause";
}

static void esp_riscv_print_exception_reason(struct target *target)
{
	if (target->state != TARGET_HALTED)
		return;

	if (target->halt_issued)
		/* halted upon `halt` request. This is not an exception */
		return;

	if (esp_common_read_pseudo_ex_reason(target) == ERROR_OK)
		return;

	riscv_reg_t mcause;
	int result = riscv_get_register(target, &mcause, GDB_REGNO_MCAUSE);
	if (result != ERROR_OK) {
		LOG_ERROR("Failed to read mcause register. Unknown exception reason!");
		return;
	}

	/* Exception ID 0x0 (instruction access misaligned) is not present because CPU always masks the lowest
	* bit of the address during instruction fetch.
	* And (mcause(31) is 1 for interrupts and 0 for exceptions). We will print only exception reasons */
	LOG_TARGET_DEBUG(target, "mcause=0x%" PRIx64, mcause);
	if (mcause & BIT(31) || ESP_RISCV_EXCEPTION_CAUSE(mcause) == 0)
		return;

	if (ESP_RISCV_EXCEPTION_CAUSE(mcause) == ILLEGAL_INSTRUCTION) {
		riscv_reg_t mtval;
		result = riscv_get_register(target, &mtval, CSR_MTVAL + GDB_REGNO_CSR0);
		if (result != ERROR_OK) {
			LOG_ERROR("Failed to read mtval register!");
			return;
		}
		uint32_t opcode = (mtval >> 0) & ((1U << 7) - 1);
		LOG_TARGET_DEBUG(target, "mtval=0x%" PRIx64 " opcode=0x%" PRIx32, mtval, opcode);
		/*
			These floating point instruction faults are handled in the idf _panic_handler and returned
			without terminating the program.
			Therefore, printing the exception cause here could provide incorrect information to users.
		*/
		if (opcode == ESP_RISCV_LOAD_FP || opcode == ESP_RISCV_STORE_FP || opcode == ESP_RISCV_OP_FP)
			return;
	}

	LOG_TARGET_INFO(target, "Halt cause (%d) - (%s)", (int)ESP_RISCV_EXCEPTION_CAUSE(mcause),
		esp_riscv_get_exception_reason(mcause));
}

static bool esp_riscv_is_wp_set_by_program(struct target *target)
{
	if (target->debug_reason != DBG_REASON_WATCHPOINT)
		return false;

	RISCV_INFO(r);

	for (struct watchpoint *wp = target->watchpoints; wp; wp = wp->next) {
		if (wp->unique_id == r->trigger_hit) {
			struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);
			/* check if wp set via semihosting call by target application */
			for (unsigned int id = 0; id < esp_riscv->max_wp_num; ++id) {
				if (esp_riscv->target_wp_addr[id] == wp->address) {
					LOG_TARGET_DEBUG(target, "wp[%d] set by program", id);
					return true;
				}
			}
		}
	}

	return false;
}

static bool esp_riscv_is_bp_set_by_program(struct target *target)
{
	if (target->debug_reason != DBG_REASON_BREAKPOINT)
		return false;

	RISCV_INFO(r);

	for (struct breakpoint *bp = target->breakpoints; bp; bp = bp->next) {
		if (bp->unique_id == r->trigger_hit) {
			struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);
			/* check if bp set via semihosting call by target application */
			for (unsigned int id = 0; id < esp_riscv->max_bp_num; ++id) {
				if (esp_riscv->target_bp_addr[id] == bp->address) {
					LOG_TARGET_DEBUG(target, "bp[%d] set by program", id);
					return true;
				}
			}
		}
	}

	return false;
}

/* General purpose registers */
static const char *esp_riscv_gprs[] = {
	"zero", "ra", "sp", "gp", "tp", "t0", "t1", "t2", "fp", "s1",
	"a0", "a1", "a2", "a3", "a4", "a5", "a6", "a7", "s2", "s3", "s4",
	"s5", "s6", "s7", "s8", "s9", "s10", "s11", "t3", "t4", "t5", "t6",
	"pc", "priv",
};

/* Floating point registers */
static const char *esp_riscv_fprs[] = {
	"ft0", "ft1", "ft2", "ft3", "ft4", "ft5", "ft6", "ft7", "fs0", "fs1",
	"fa0", "fa1", "fa2", "fa3", "fa4", "fa5", "fa6", "fa7", "fs2", "fs3",
	"fs4", "fs5", "fs6", "fs7", "fs8", "fs9", "fs10", "fs11", "ft8", "ft9",
	"ft10", "ft11",
};

/* Common CSRs for all chips */
static const char *esp_riscv_csrs[] = {
	"mstatus", "misa", "mtvec", "mscratch", "mepc", "mcause", "mtval",
	"pmpcfg0", "pmpcfg1", "pmpcfg2", "pmpcfg3",
	"pmpaddr0", "pmpaddr1", "pmpaddr2", "pmpaddr3", "pmpaddr4", "pmpaddr5", "pmpaddr6", "pmpaddr7",
	"pmpaddr8", "pmpaddr9", "pmpaddr10", "pmpaddr11", "pmpaddr12", "pmpaddr13", "pmpaddr14", "pmpaddr15",
	"tselect", "tdata1", "tdata2", "tcontrol",
	"dcsr", "dpc", "dscratch0", "dscratch1",
	"csr_mpcer",  "csr_mpcmr", "csr_mpccr",
	"csr_cpu_gpio_oen", "csr_cpu_gpio_in", "csr_cpu_gpio_out",
};

/* Read only registers */
static const char *esp_riscv_ro_csrs[] = {
	"mvendorid", "marchid", "mimpid", "mhartid",
};

int esp_riscv_examine(struct target *target)
{
	int ret = riscv_target.examine(target);
	if (ret != ERROR_OK)
		return ret;

	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);

	/*
		RISCV code initializes all registers upon target examination.
		Espressif chips don't support all of them.
		Disable not supported registers and avoid writing to read only registers during algorithm run
	*/

	struct {
		const char **reg_array;
		size_t reg_array_size;
		bool save_restore;
	} esp_riscv_registers[] = {
		{ esp_riscv_gprs, ARRAY_SIZE(esp_riscv_gprs), true },
		/* FPRs can't be read via abstract command in ESP32-P4 ECO version */
		{ esp_riscv_fprs, ARRAY_SIZE(esp_riscv_fprs), false },
		{ esp_riscv_csrs, ARRAY_SIZE(esp_riscv_csrs), true },
		{ esp_riscv_ro_csrs, ARRAY_SIZE(esp_riscv_ro_csrs), false },
		{ esp_riscv->existent_ro_csrs, esp_riscv->existent_ro_csr_size, false }, /* chip specific RO CSRs */
		{ esp_riscv->existent_csrs, esp_riscv->existent_csr_size, true } /* chip specific CSRs */
	};

	for (unsigned int i = 0; i < target->reg_cache->num_regs; i++) {
		if (target->reg_cache->reg_list[i].exist) {
			target->reg_cache->reg_list[i].exist = false;
			for (unsigned int j = 0; j < ARRAY_SIZE(esp_riscv_registers); j++) {
				for (unsigned int k = 0; k < esp_riscv_registers[j].reg_array_size; k++) {
					if (!strcmp(target->reg_cache->reg_list[i].name, esp_riscv_registers[j].reg_array[k])) {
						target->reg_cache->reg_list[i].exist = true;
						target->reg_cache->reg_list[i].caller_save = esp_riscv_registers[j].save_restore;
						break;
					}
				}
				if (target->reg_cache->reg_list[i].exist)
					break;
			}
		}
	}

	return ERROR_OK;
}

int esp_riscv_poll(struct target *target)
{
	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);
	int res = ERROR_OK;

	if (target->state == TARGET_HALTED && target->smp && target->gdb_service && !target->gdb_service->target) {
		target->gdb_service->target = esp_common_get_halted_target(target, target->gdb_service->core[1]);
		LOG_INFO("Switch GDB target to '%s'", target_name(target->gdb_service->target));
		target_call_event_callbacks(target, TARGET_EVENT_HALTED);
		return ERROR_OK;
	}

	if (esp_riscv->was_reset) {
		if (esp_riscv->print_reset_reason) {
			uint32_t reset_reason_reg_val = 0;
			res = target_read_u32(target, esp_riscv->rtccntl_reset_state_reg, &reset_reason_reg_val);
			if (res != ERROR_OK)
				LOG_TARGET_WARNING(target, "Failed to read reset cause register (%d)!", res);
			else
				esp_riscv->print_reset_reason(target, reset_reason_reg_val);
		}

		if (esp_riscv->semi_ops->post_reset)
			esp_riscv->semi_ops->post_reset(target);

		/* Clear memory which is used by RTOS layer to get the task count */
		if (target->rtos && target->rtos->type->post_reset_cleanup) {
			res = (*target->rtos->type->post_reset_cleanup)(target);
			if (res != ERROR_OK)
				LOG_WARNING("Failed to do rtos-specific cleanup (%d)", res);
		}
		esp_riscv->was_reset = false;
	}

	return riscv_openocd_poll(target);
}

int esp_riscv_alloc_trigger_addr(struct target *target)
{
	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);

	esp_riscv->target_bp_addr = calloc(esp_riscv->max_bp_num, sizeof(*esp_riscv->target_bp_addr));
	if (!esp_riscv->target_bp_addr)
		return ERROR_FAIL;

	esp_riscv->target_wp_addr = calloc(esp_riscv->max_wp_num, sizeof(*esp_riscv->target_wp_addr));
	if (!esp_riscv->target_wp_addr)
		return ERROR_FAIL;

	return ERROR_OK;
}

int esp_riscv_semihosting(struct target *target)
{
	int res = ERROR_OK;
	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);
	struct semihosting *semihosting = target->semihosting;

	/*
		If a bp/wp set request comes from Core1, the other cores may continue running.
		We need to ensure that all harts are in a halted state.
	*/
	if (target->smp && (semihosting->op == ESP_SEMIHOSTING_SYS_BREAKPOINT_SET ||
		semihosting->op == ESP_SEMIHOSTING_SYS_WATCHPOINT_SET)) {
		/* Do not report internal halt, set flag for target with active GDB service (keep in mind for OCD-1132) */
		struct riscv_info *info = riscv_info(target->gdb_service ? target->gdb_service->target : target);
		info->pause_gdb_callbacks = true;
		/* Halt all harts in the SMP group. Resume-all will be handled in riscv_semihosting() return */
		res = riscv_halt(target);
		info->pause_gdb_callbacks = false;
		if (res != ERROR_OK)
			return res;
	}

	switch (semihosting->op) {
	case ESP_SEMIHOSTING_SYS_APPTRACE_INIT:
		res = esp_riscv_apptrace_info_init(target, semihosting->param, NULL);
		if (res != ERROR_OK)
			return res;
		break;
	case ESP_SEMIHOSTING_SYS_DEBUG_STUBS_INIT:
		res = esp_riscv_debug_stubs_info_init(target, semihosting->param);
		if (res != ERROR_OK)
			return res;
		break;
	case ESP_SEMIHOSTING_SYS_BREAKPOINT_SET:
	{
		/* Enough space to hold 3 long words for both riscv32 and riscv64 archs. */
		uint8_t fields[ESP_RISCV_SET_BREAKPOINT_ARG_MAX * sizeof(uint64_t)];
		res = semihosting_read_fields(target,
				ESP_RISCV_SET_BREAKPOINT_ARG_MAX,
				fields);
		if (res != ERROR_OK)
			return res;
		int id = semihosting_get_field(target,
				ESP_RISCV_SET_BREAKPOINT_ARG_ID,
				fields);
		if (id >= esp_riscv->max_bp_num) {
			LOG_ERROR("Unsupported breakpoint ID (%d)!", id);
			return ERROR_FAIL;
		}
		int set = semihosting_get_field(target,
				ESP_RISCV_SET_BREAKPOINT_ARG_SET,
				fields);
		if (set) {
			if (esp_riscv->target_bp_addr[id]) {
				breakpoint_remove(target, esp_riscv->target_bp_addr[id]);
			}
			esp_riscv->target_bp_addr[id] = semihosting_get_field(target,
					ESP_RISCV_SET_BREAKPOINT_ARG_ADDR,
					fields);
			res = breakpoint_add(target,
					esp_riscv->target_bp_addr[id],
					2,
					BKPT_HARD);
			if (res != ERROR_OK)
				return res;
		} else {
			breakpoint_remove(target, esp_riscv->target_bp_addr[id]);
			esp_riscv->target_bp_addr[id] = 0;
		}
		break;
	}
	case ESP_SEMIHOSTING_SYS_WATCHPOINT_SET:
	{
		/* Enough space to hold 5 long words for both riscv32 and riscv64 archs. */
		uint8_t fields[ESP_RISCV_SET_WATCHPOINT_ARG_MAX * sizeof(uint64_t)];
		res = semihosting_read_fields(target,
				ESP_RISCV_SET_WATCHPOINT_ARG_MAX,
				fields);
		if (res != ERROR_OK)
			return res;
		int id = semihosting_get_field(target,
				ESP_RISCV_SET_WATCHPOINT_ARG_ID,
				fields);
		if (id >= esp_riscv->max_wp_num) {
			LOG_ERROR("Unsupported watchpoint ID (%d)!", id);
			return ERROR_FAIL;
		}
		int set = semihosting_get_field(target,
				ESP_RISCV_SET_WATCHPOINT_ARG_SET,
				fields);
		if (set) {
			if (esp_riscv->target_wp_addr[id]) {
				watchpoint_remove(target, esp_riscv->target_wp_addr[id]);
			}
			esp_riscv->target_wp_addr[id] = semihosting_get_field(target,
					ESP_RISCV_SET_WATCHPOINT_ARG_ADDR,
					fields);
			int size = semihosting_get_field(target,
					ESP_RISCV_SET_WATCHPOINT_ARG_SIZE,
					fields);
			int flags = semihosting_get_field(target,
					ESP_RISCV_SET_WATCHPOINT_ARG_FLAGS,
					fields);
			enum watchpoint_rw wp_type;
			switch (flags &
					(ESP_SEMIHOSTING_WP_FLG_RD | ESP_SEMIHOSTING_WP_FLG_WR)) {
			case ESP_SEMIHOSTING_WP_FLG_RD:
				wp_type = WPT_READ;
				break;
			case ESP_SEMIHOSTING_WP_FLG_WR:
				wp_type = WPT_WRITE;
				break;
			case ESP_SEMIHOSTING_WP_FLG_RD | ESP_SEMIHOSTING_WP_FLG_WR:
				wp_type = WPT_ACCESS;
				break;
			default:
				LOG_ERROR("Unsupported watchpoint type (0x%x)!",
						flags);
				return ERROR_FAIL;
			}
			res = watchpoint_add(target,
					esp_riscv->target_wp_addr[id],
					size,
					wp_type,
					0,
					WATCHPOINT_IGNORE_DATA_VALUE_MASK);
			if (res != ERROR_OK)
				return res;
		} else {
			watchpoint_remove(target, esp_riscv->target_wp_addr[id]);
			esp_riscv->target_wp_addr[id] = 0;
		}
		break;
	}
	default:
		return ERROR_FAIL;
	}

	semihosting->result = res == ERROR_OK ? 0 : -1;
	semihosting->is_resumable = true;

	return res;
}

static int esp_riscv_debug_stubs_info_init(struct target *target,
	target_addr_t vec_addr)
{
	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);

	LOG_TARGET_INFO(target, "Detected debug stubs entry @ " TARGET_ADDR_FMT, vec_addr);

	memset(&esp_riscv->esp.dbg_stubs, 0, sizeof(esp_riscv->esp.dbg_stubs));

	esp_riscv->esp.dbg_stubs.base = vec_addr;
	int res = esp_dbgstubs_table_read(target, &esp_riscv->esp.dbg_stubs);
	if (res != ERROR_OK)
		return res;
	if (esp_riscv->esp.dbg_stubs.entries_count == 0)
		return ERROR_OK;

	/* read debug stubs descriptor */
	ESP_RISCV_DBGSTUBS_UPDATE_DATA_ENTRY(esp_riscv->esp.dbg_stubs.entries[ESP_DBG_STUB_CONTROL_DATA]);
	res = target_read_buffer(target, esp_riscv->esp.dbg_stubs.entries[ESP_DBG_STUB_CONTROL_DATA],
		sizeof(struct esp_dbg_stubs_ctl_data), (uint8_t *)&esp_riscv->esp.dbg_stubs.ctl_data);
	if (res != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to read debug stubs descriptor (%d)!", res);
		return res;
	}
	ESP_RISCV_DBGSTUBS_UPDATE_CODE_ENTRY(esp_riscv->esp.dbg_stubs.ctl_data.tramp_addr);
	ESP_RISCV_DBGSTUBS_UPDATE_DATA_ENTRY(esp_riscv->esp.dbg_stubs.ctl_data.min_stack_addr);
	ESP_RISCV_DBGSTUBS_UPDATE_CODE_ENTRY(esp_riscv->esp.dbg_stubs.ctl_data.data_alloc);
	ESP_RISCV_DBGSTUBS_UPDATE_CODE_ENTRY(esp_riscv->esp.dbg_stubs.ctl_data.data_free);

	return ERROR_OK;
}

int esp_riscv_breakpoint_add(struct target *target, struct breakpoint *breakpoint)
{
	struct esp_riscv_common *esp_riscv;

	int res = riscv_add_breakpoint(target, breakpoint);

	if (breakpoint->type == BKPT_HARD) {
		if (res == ERROR_OK) {
			RISCV_INFO(info);
			/* manual_hwbp_set is required to be set for all harts.
			 * Otherwise bps coming from hart1 will not be handled properly during step.
			 * TODO: Check if needs to be set earlier in the first TDATA1 or TDATA2 modification.
			*/
			info->manual_hwbp_set = true;
		} else if (res == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
			/* For SMP target return OK if SW flash breakpoint is already set using another
			*core; GDB causes call to esp_algo_flash_breakpoint_add() for every core, since it
			*treats flash breakpoints as HW ones */
			if (target->smp) {
				struct target_list *curr;
				foreach_smp_target(curr, target->smp_targets) {
					esp_riscv = target_to_esp_riscv(curr->target);
					if (esp_common_flash_breakpoint_exists(&esp_riscv->esp, breakpoint))
						return ERROR_OK;
				}
			}
			esp_riscv = target_to_esp_riscv(target);
			return esp_common_flash_breakpoint_add(target, &esp_riscv->esp, breakpoint);
		}
	}

	return res;
}

int esp_riscv_breakpoint_remove(struct target *target, struct breakpoint *breakpoint)
{
	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);

	int res = riscv_remove_breakpoint(target, breakpoint);
	if (res == ERROR_TARGET_RESOURCE_NOT_AVAILABLE && breakpoint->type == BKPT_HARD) {
		res = esp_common_flash_breakpoint_remove(target, &esp_riscv->esp, breakpoint);
		if (res == ERROR_TARGET_RESOURCE_NOT_AVAILABLE && target->smp) {
			/* For SMP target return OK always, because SW flash breakpoint are set only
			 *using one core, but GDB causes call to esp_algo_flash_breakpoint_remove() for
			 *every core, since it treats flash breakpoints as HW ones */
			res = ERROR_OK;
		}
	}

	return res;
}

int esp_riscv_smp_watchpoint_add(struct target *target, struct watchpoint *watchpoint)
{
	int res = riscv_add_watchpoint(target, watchpoint);
	if (res != ERROR_OK)
		return res;

	if (!target->smp)
		return ERROR_OK;

	struct target_list *head;
	foreach_smp_target(head, target->smp_targets) {
		struct target *curr = head->target;
		if (curr == target || !target_was_examined(curr))
			continue;
		/* Need to use high level API here because every target for core contains list of watchpoints.
		 * GDB works with active core only, so we need to duplicate every watchpoint on other cores,
		 * otherwise watchpoint_free() on active core can fail if WP has been initially added on another core. */
		unsigned int tmp_smp = curr->smp;
		curr->smp = 0;
		res = watchpoint_add(curr, watchpoint->address, watchpoint->length,
			watchpoint->rw, watchpoint->value, watchpoint->mask);
		curr->smp = tmp_smp;
		if (res != ERROR_OK)
			return res;
	}
	return ERROR_OK;
}

int esp_riscv_smp_watchpoint_remove(struct target *target, struct watchpoint *watchpoint)
{
	int res = riscv_remove_watchpoint(target, watchpoint);
	if (res != ERROR_OK)
		return res;

	if (!target->smp)
		return ERROR_OK;

	struct target_list *head;
	foreach_smp_target(head, target->smp_targets) {
		struct target *curr = head->target;
		if (curr == target)
			continue;
		/* see big comment in esp_riscv_watchpoint_add() */
		unsigned int tmp_smp = curr->smp;
		curr->smp = 0;
		res = watchpoint_remove(curr, watchpoint->address);
		curr->smp = tmp_smp;
		if (res != ERROR_OK)
			return res;
	}
	return ERROR_OK;
}

int esp_riscv_hit_watchpoint(struct target *target, struct watchpoint **hit_watchpoint)
{
	/* Do not send watchpoint info if it is set by program.
	 * Otherwise GDB will ignore T05 msg and will not halt at the watchpoint.
	*/
	if (esp_riscv_is_wp_set_by_program(target))
		return ERROR_FAIL;

	return riscv_hit_watchpoint(target, hit_watchpoint);
}

int esp_riscv_resume(struct target *target, bool current, target_addr_t address,
		bool handle_breakpoints, bool debug_execution)
{
	/* On Riscv targets we change gdb service target only for gdb fileio requests
	 * After getting the fileio response it is ok to switch it to the default target which is core0
	 * Resume request is sent in the gdb_fileio_response_packet() after fileio command processed
	*/
	if (target->smp && target->gdb_service) {
		struct target_list *head;
		head = list_first_entry(target->smp_targets, struct target_list, lh);
		target->gdb_service->target = head->target;
	}

	/* If one of the target stopped due to breakpoint/watchpoint set by program,
	 * we need to handle_breakpoints to make single step
	 */
	if (!handle_breakpoints) {
		if (target->smp) {
			struct target_list *head;
			foreach_smp_target(head, target->smp_targets) {
				if (esp_riscv_is_bp_set_by_program(head->target) || esp_riscv_is_wp_set_by_program(head->target)) {
					handle_breakpoints = true;
					break;
				}
			}
		} else {
			handle_breakpoints = esp_riscv_is_bp_set_by_program(target) || esp_riscv_is_wp_set_by_program(target);
		}
	}

	return riscv_target_resume(target, current, address, handle_breakpoints, debug_execution);
}

static int esp_riscv_on_halt(struct target *target)
{
	riscv_reg_t reg_value;
	if (riscv_get_register(target, &reg_value, GDB_REGNO_DPC) == ERROR_OK)
		LOG_TARGET_INFO(target, "Target halted, PC=0x%08" PRIX64 ", debug_reason=%08x",
			reg_value, target->debug_reason);
	esp_riscv_print_exception_reason(target);
	return ERROR_OK;
}

static int esp_riscv_single_hart_poll(struct target *target)
{
	int smp = target->smp;
	target->smp = 0;
	int res = riscv_openocd_poll(target);
	target->smp = smp;
	return res;
}

int esp_riscv_start_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t entry_point, target_addr_t exit_point,
	void *arch_info)
{
	struct esp_riscv_algorithm *algorithm_info = arch_info;
	size_t max_saved_reg = algorithm_info->max_saved_reg;

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_ERROR(target, "Not halted. Can not start target algo!");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Save registers */
	for (uint32_t number = GDB_REGNO_ZERO + 1;
		number <= max_saved_reg && number < target->reg_cache->num_regs; number++) {
		struct reg *r = &target->reg_cache->reg_list[number];

		algorithm_info->valid_saved_registers[r->number] = r->exist;
		if (!r->exist || !r->caller_save)
			continue;

		LOG_TARGET_DEBUG(target, "save %s", r->name);

		if (r->size > 64) {
			LOG_TARGET_ERROR(target, "Register %s is %d bits! Max 64-bits are supported.",
				r->name,
				r->size);
			return ERROR_FAIL;
		}

		if (r->type->get(r) != ERROR_OK) {
			LOG_TARGET_ERROR(target, "get(%s) failed", r->name);
			r->exist = false;
			return ERROR_FAIL;
		}
		algorithm_info->saved_registers[r->number] = buf_get_u64(r->value, 0, r->size);
	}

	/* write mem params */
	for (int i = 0; i < num_mem_params; i++) {
		if (mem_params[i].direction != PARAM_IN) {
			int retval = target_write_buffer(target, mem_params[i].address,
				mem_params[i].size,
				mem_params[i].value);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	for (int i = 0; i < num_reg_params; i++) {
		LOG_TARGET_DEBUG(target, "set %s", reg_params[i].reg_name);
		struct reg *r = register_get_by_name(target->reg_cache, reg_params[i].reg_name, false);
		if (!r) {
			LOG_TARGET_ERROR(target, "Couldn't find register named '%s'", reg_params[i].reg_name);
			return ERROR_FAIL;
		}

		if (r->size != reg_params[i].size) {
			LOG_TARGET_ERROR(target, "Register %s is %d bits instead of %d bits.",
				reg_params[i].reg_name, r->size, reg_params[i].size);
			return ERROR_FAIL;
		}

		if (r->number > GDB_REGNO_XPR31) {
			LOG_TARGET_ERROR(target, "Only GPRs can be use as argument registers.");
			return ERROR_FAIL;
		}

		if (reg_params[i].direction == PARAM_OUT || reg_params[i].direction == PARAM_IN_OUT) {
			if (r->type->set(r, reg_params[i].value) != ERROR_OK) {
				LOG_TARGET_ERROR(target, "set(%s) failed", reg_params[i].reg_name);
				return ERROR_FAIL;
			}
		}
	}

	/* Disable Interrupts before attempting to run the algorithm. */
	int retval = riscv_interrupts_disable(target,
		MSTATUS_MIE | MSTATUS_HIE | MSTATUS_SIE | MSTATUS_UIE,
		&algorithm_info->masked_mstatus);
	if (retval != ERROR_OK)
		return retval;

	/* Run algorithm */
	LOG_TARGET_DEBUG(target, "resume at 0x%" TARGET_PRIxADDR, entry_point);
	return riscv_resume(target, 0, entry_point, 0, 1, true);
}

/* Algorithm must end with a software breakpoint instruction. */
int esp_riscv_wait_algorithm(struct target *target,
	int num_mem_params, struct mem_param *mem_params,
	int num_reg_params, struct reg_param *reg_params,
	target_addr_t exit_point, unsigned int timeout_ms,
	void *arch_info)
{
	RISCV_INFO(info);
	struct esp_riscv_algorithm *algorithm_info = arch_info;
	size_t max_saved_reg = algorithm_info->max_saved_reg;

	int64_t start = timeval_ms();
	while (target->state != TARGET_HALTED) {
		LOG_DEBUG_IO("poll()");
		int64_t now = timeval_ms();
		if (now - start > timeout_ms) {
			LOG_TARGET_ERROR(target, "Algorithm timed out after %" PRId64 " ms.", now - start);
			riscv_halt(target);
			esp_riscv_single_hart_poll(target);
			enum gdb_regno regnums[] = {
				GDB_REGNO_RA, GDB_REGNO_SP, GDB_REGNO_GP, GDB_REGNO_TP,
				GDB_REGNO_T0, GDB_REGNO_T1, GDB_REGNO_T2, GDB_REGNO_FP,
				GDB_REGNO_S1, GDB_REGNO_A0, GDB_REGNO_A1, GDB_REGNO_A2,
				GDB_REGNO_A3, GDB_REGNO_A4, GDB_REGNO_A5, GDB_REGNO_A6,
				GDB_REGNO_A7, GDB_REGNO_S2, GDB_REGNO_S3, GDB_REGNO_S4,
				GDB_REGNO_S5, GDB_REGNO_S6, GDB_REGNO_S7, GDB_REGNO_S8,
				GDB_REGNO_S9, GDB_REGNO_S10, GDB_REGNO_S11, GDB_REGNO_T3,
				GDB_REGNO_T4, GDB_REGNO_T5, GDB_REGNO_T6,
				GDB_REGNO_PC,
				GDB_REGNO_MSTATUS, GDB_REGNO_MEPC, GDB_REGNO_MCAUSE,
			};
			for (unsigned int i = 0; i < ARRAY_SIZE(regnums); i++) {
				enum gdb_regno regno = regnums[i];
				riscv_reg_t reg_value;
				if (riscv_get_register(target, &reg_value, regno) != ERROR_OK)
					break;
				LOG_TARGET_ERROR(target, "%s = 0x%" PRIx64, gdb_regno_name(target, regno), reg_value);
			}
			return ERROR_TARGET_TIMEOUT;
		}

		int result = esp_riscv_single_hart_poll(target);
		if (result != ERROR_OK)
			return result;
	}

	struct reg *reg_pc = register_get_by_name(target->reg_cache, "pc", true);
	if (reg_pc->type->get(reg_pc) != ERROR_OK)
		return ERROR_FAIL;
	uint64_t final_pc = buf_get_u64(reg_pc->value, 0, reg_pc->size);
	if (exit_point && final_pc != exit_point) {
		LOG_TARGET_ERROR(target, "PC ended up at 0x%" PRIx64 " instead of 0x%"
			TARGET_PRIxADDR, final_pc, exit_point);
		return ERROR_FAIL;
	}

	for (int i = 0; i < num_reg_params; i++) {
		if (reg_params[i].direction == PARAM_IN ||
			reg_params[i].direction == PARAM_IN_OUT) {
			struct reg *r = register_get_by_name(target->reg_cache,
				reg_params[i].reg_name,
				false);
			if (r->type->get(r) != ERROR_OK) {
				LOG_TARGET_ERROR(target, "get(%s) failed", r->name);
				return ERROR_FAIL;
			}
			buf_cpy(r->value, reg_params[i].value, reg_params[i].size);
		}
	}
	/* Read memory values to mem_params */
	LOG_TARGET_DEBUG(target, "Read mem params");
	for (int i = 0; i < num_mem_params; i++) {
		LOG_TARGET_DEBUG(target, "Check mem param @ " TARGET_ADDR_FMT, mem_params[i].address);
		if (mem_params[i].direction != PARAM_OUT) {
			LOG_TARGET_DEBUG(target, "Read mem param @ " TARGET_ADDR_FMT, mem_params[i].address);
			int retval = target_read_buffer(target, mem_params[i].address,
				mem_params[i].size,
				mem_params[i].value);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	/* Restore registers */
	for (uint32_t number = GDB_REGNO_ZERO + 1;
		number <= max_saved_reg && number < target->reg_cache->num_regs; number++) {
		struct reg *r = &target->reg_cache->reg_list[number];

		if (!algorithm_info->valid_saved_registers[r->number] || !r->caller_save)
			continue;

		LOG_TARGET_DEBUG(target, "restore %s", r->name);
		uint8_t buf[8];
		buf_set_u64(buf, 0, info->xlen, algorithm_info->saved_registers[r->number]);
		if (r->type->set(r, buf) != ERROR_OK) {
			LOG_TARGET_ERROR(target, "set(%s) failed", r->name);
			return ERROR_FAIL;
		}
	}

	/* We restore mstatus above, but writing to other m registers can changes it's original value */
	if (riscv_interrupts_restore(target, algorithm_info->masked_mstatus) != ERROR_OK)
		return ERROR_FAIL;

	if (riscv_flush_registers(target) != ERROR_OK)
		return ERROR_FAIL;
	return ERROR_OK;
}

int esp_riscv_run_algorithm(struct target *target, int num_mem_params,
	struct mem_param *mem_params, int num_reg_params,
	struct reg_param *reg_params, target_addr_t entry_point,
	target_addr_t exit_point, unsigned int timeout_ms, void *arch_info)
{
	int retval;

	retval = esp_riscv_start_algorithm(target,
		num_mem_params, mem_params,
		num_reg_params, reg_params,
		entry_point, exit_point,
		arch_info);

	if (retval == ERROR_OK) {
		retval = esp_riscv_wait_algorithm(target,
			num_mem_params, mem_params,
			num_reg_params, reg_params,
			exit_point, timeout_ms,
			arch_info);
	}

	return retval;
}

int esp_riscv_smp_run_func_image(struct target *target, struct esp_algorithm_run_data *run, uint32_t num_args, ...)
{
	struct target *run_target = target;
	struct target_list *head;
	va_list ap;

	if (target->smp) {
		head = list_first_entry(target->smp_targets, struct target_list, lh);
		run_target = head->target;
	}

	if (run_target->coreid != 0 || !target_was_examined(run_target) || run_target->state != TARGET_HALTED) {
		LOG_ERROR("Algorithm only can be run on examined and halted Core0");
		return ERROR_FAIL;
	}

	va_start(ap, num_args);
	int res = esp_algorithm_run_func_image_va(run_target, run, num_args, ap);
	va_end(ap);
	return res;
}

int esp_riscv_read_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, uint8_t *buffer)
{
	RISCV_INFO(r);
	uint32_t sba_access_size = r->data_bits(target) / 8;

	if (size < sba_access_size || !IS_ALIGNED(address, sba_access_size)) {
		LOG_DEBUG("Use %d-bit access: size: %d\tcount:%d\tstart address: 0x%08"
			TARGET_PRIxADDR, sba_access_size * 8, size, count, address);
		target_addr_t al_addr = ALIGN_DOWN(address, sba_access_size);
		uint32_t al_len = (size * count) + address - al_addr;
		uint32_t al_cnt = ALIGN_UP(al_len, sba_access_size);
		uint8_t al_buf[al_cnt];
		int ret = riscv_target.read_memory(target, al_addr, sba_access_size, al_cnt / sba_access_size, al_buf);
		if (ret == ERROR_OK)
			memcpy(buffer, &al_buf[address & (sba_access_size - 1)], size * count);
		return ret;
	}

	return riscv_target.read_memory(target, address, size, count, buffer);
}

int esp_riscv_write_memory(struct target *target, target_addr_t address,
	uint32_t size, uint32_t count, const uint8_t *buffer)
{
	RISCV_INFO(r);
	uint32_t sba_access_size = r->data_bits(target) / 8;

	if (size < sba_access_size || !IS_ALIGNED(address, sba_access_size)) {
		LOG_DEBUG("Use %d-bit access: size: %d\tcount:%d\tstart address: 0x%08"
			TARGET_PRIxADDR, sba_access_size * 8, size, count, address);
		target_addr_t al_addr = ALIGN_DOWN(address, sba_access_size);
		uint32_t al_len = (size * count) + address - al_addr;
		uint32_t al_cnt = ALIGN_UP(al_len, sba_access_size);
		uint8_t al_buf[al_cnt];
		int ret = riscv_target.read_memory(target, al_addr, sba_access_size, al_cnt / sba_access_size, al_buf);
		if (ret == ERROR_OK) {
			memcpy(&al_buf[address & (sba_access_size - 1)], buffer, size * count);
			ret = riscv_target.write_memory(target, al_addr, sba_access_size, al_cnt / sba_access_size, al_buf);
		}
		return ret;
	}
	return riscv_target.write_memory(target, address, size, count, buffer);
}

void esp_riscv_deinit_target(struct target *target)
{
	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);
	if (esp_riscv->semi_ops->post_reset)
		esp_riscv->semi_ops->post_reset(target);

	free(esp_riscv->target_bp_addr);
	free(esp_riscv->target_wp_addr);

	riscv_target.deinit_target(target);
}

int esp_riscv_assert_reset(struct target *target)
{
	struct esp_riscv_common *esp_riscv = target_to_esp_riscv(target);
	/* clear previous apptrace ctrl_addr to avoid invalid tracing control block usage during/after reset */
	esp_riscv->apptrace.ctrl_addr = 0;
	return riscv_assert_reset(target);
}

int esp_riscv_get_gdb_reg_list_noread(struct target *target,
		struct reg **reg_list[], int *reg_list_size,
		enum target_register_class reg_class)
{
	if (target->state == TARGET_HALTED) {
		return riscv_get_gdb_reg_list(target, reg_list, reg_list_size, reg_class);
	} else if (target->state == TARGET_RUNNING) {
		/* GDB can send 'g' packet when target is running. This is unexpected behavior explained in the OCD-749 */
		return riscv_get_gdb_reg_list_noread(target, reg_list, reg_list_size, reg_class);
	}

	LOG_TARGET_ERROR(target, "Unexpected target state! (%d)", target->state);
	return ERROR_FAIL;
}

COMMAND_HANDLER(esp_riscv_halted_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);
	if (!target) {
		LOG_ERROR("No target selected");
		return ERROR_FAIL;
	}

	return esp_riscv_on_halt(target);
}

const struct command_registration esp_riscv_command_handlers[] = {
	{
		.name = "process_lazy_breakpoints",
		.handler = esp_common_process_flash_breakpoints_command,
		.mode = COMMAND_ANY,
		.help = "Set/clear all pending flash breakpoints",
		.usage = "",
	},
	{
		.name = "disable_lazy_breakpoints",
		.handler = esp_common_disable_lazy_breakpoints_command,
		.mode = COMMAND_ANY,
		.help = "Process flash breakpoints on time",
		.usage = "",
	},
	{
		.name = "halted_event_handler",
		.handler = esp_riscv_halted_command,
		.mode = COMMAND_ANY,
		.help = "Handles halted event and prints exception reason",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};
