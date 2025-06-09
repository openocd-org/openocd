/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Espressif chips common target API for OpenOCD                         *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>
#include <helper/binarybuffer.h>
#include <target/smp.h>
#include <target/target.h>
#include "esp_riscv.h"
#include "esp_xtensa.h"
#include "esp.h"

#define ESP_FLASH_BREAKPOINTS_MAX_NUM  32
#define ESP_ASSIST_DEBUG_INVALID_VALUE 0xFFFFFFFF

static int esp_callback_event_handler(struct target *target, enum target_event event, void *priv);

struct esp_common *target_to_esp_common(struct target *target)
{
	struct xtensa *xtensa = target->arch_info;
	if (xtensa->common_magic == RISCV_COMMON_MAGIC)
		return &(target_to_esp_riscv(target)->esp);
	else if (xtensa->common_magic == XTENSA_COMMON_MAGIC)
		return &(target_to_esp_xtensa(target)->esp);
	LOG_ERROR("Unknown target arch!");
	return NULL;
}

int esp_common_init(struct target *target, struct esp_common *esp,
	const struct esp_flash_breakpoint_ops *flash_brps_ops,
	const struct esp_algorithm_hw *algo_hw)
{
	esp->algo_hw = algo_hw;
	esp->flash_brps.ops = flash_brps_ops;
	esp->flash_brps.brps = calloc(ESP_FLASH_BREAKPOINTS_MAX_NUM, sizeof(struct esp_flash_breakpoint));
	if (!esp->flash_brps.brps)
		return ERROR_FAIL;
	esp->breakpoint_lazy_process = true;

	if (target->coreid == 0)
		target_register_event_callback(esp_callback_event_handler, esp);

	return ERROR_OK;
}

int esp_dbgstubs_table_read(struct target *target, struct esp_dbg_stubs *dbg_stubs)
{
	uint8_t entry_buff[sizeof(uint32_t) * ESP_DBG_STUB_ENTRY_MAX] = {0}; /* to avoid endiannes issues */

	LOG_TARGET_DEBUG(target, "Read debug stubs info %" PRIx32 " / %d", dbg_stubs->base, dbg_stubs->entries_count);

	/* Read all entries */
	int res = target_read_buffer(target, dbg_stubs->base, sizeof(entry_buff), entry_buff);
	if (res != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Failed to read first debug stub entry!");
		return res;
	}

	dbg_stubs->entries[0] = target_buffer_get_u32(target, entry_buff);
	if (dbg_stubs->entries[0] != ESP_DBG_STUB_MAGIC_NUM_VAL) {
		LOG_TARGET_ERROR(target, "Invalid stub magic num (%" PRIx32 ")", dbg_stubs->entries[0]);
		return ERROR_FAIL;
	}

	dbg_stubs->entries[1] = target_buffer_get_u32(target, entry_buff + sizeof(uint32_t));
	uint32_t table_size = dbg_stubs->entries[1];

	/* discard unsupported entries */
	if (table_size < 2) {
		LOG_ERROR("Invalid stub table entry size (%x)", table_size);
		return ERROR_FAIL;
	}
	if (table_size > ESP_DBG_STUB_ENTRY_MAX)
		table_size = ESP_DBG_STUB_ENTRY_MAX;

	for (unsigned int i = 2; i < table_size; ++i)
		dbg_stubs->entries[i] = target_buffer_get_u32(target, entry_buff + sizeof(uint32_t) * i);

	for (enum esp_dbg_stub_id i = ESP_DBG_STUB_CONTROL_DATA; i < ESP_DBG_STUB_ENTRY_MAX; i++) {
		LOG_DEBUG("Check dbg stub %d - %x", i, dbg_stubs->entries[i]);
		if (dbg_stubs->entries[i]) {
			LOG_DEBUG("New dbg stub %d at %x", dbg_stubs->entries_count, dbg_stubs->entries[i]);
			dbg_stubs->entries_count++;
		}
	}
	if (dbg_stubs->entries_count < table_size - ESP_DBG_STUB_CONTROL_DATA)
		LOG_WARNING("Not full dbg stub table %d of %d", dbg_stubs->entries_count,
			table_size - ESP_DBG_STUB_CONTROL_DATA);

	return ERROR_OK;
}

struct target *esp_common_get_halted_target(struct target *target, int32_t coreid)
{
	struct target_list *head;
	struct target *curr;

	foreach_smp_target(head, target->smp_targets) {
		curr = head->target;
		if (curr->coreid == coreid && curr->state == TARGET_HALTED)
			return curr;
	}

	return target;
}

static void esp_common_dump_bp_slot(const char *caption, struct esp_flash_breakpoints *bps, size_t slot)
{
	if (!LOG_LEVEL_IS(LOG_LVL_DEBUG))
		return;

	struct breakpoint *curr = bps->brps[slot].oocd_bp;

	if (curr) {
		if (caption && strlen(caption))
			LOG_OUTPUT("=========== %s ===============\n", caption);
		LOG_OUTPUT("Slot: %zu\n", slot);
		LOG_OUTPUT(" \toriginal inst : ");
		struct esp_flash_breakpoint *brps = &bps->brps[slot];
		for (int i = 0; i < brps->insn_sz; i++)
			LOG_OUTPUT("%02X", brps->insn[i]);
		LOG_OUTPUT("\n\tstatus        : %s\n", brps->status == ESP_BP_STAT_DONE ? "DONE" : "PENDING");
		LOG_OUTPUT("\taction        : %s\n", brps->action == ESP_BP_ACT_ADD ? "ADD" : "REMOVE");
		LOG_OUTPUT("\tvirt addr     : " TARGET_ADDR_FMT "\n", brps->bp_address);
		LOG_OUTPUT("\tflash addr    : 0x%" PRIx32 "\n", brps->bp_flash_addr);
	}
}

static int esp_common_flash_breakpoints_clear(struct target *target)
{
	struct esp_common *esp = target_to_esp_common(target);

	for (size_t slot = 0; slot < ESP_FLASH_BREAKPOINTS_MAX_NUM; slot++) {
		struct esp_flash_breakpoint *flash_bp = &esp->flash_brps.brps[slot];
		if (flash_bp->insn_sz > 0) {
			int ret = esp->flash_brps.ops->breakpoint_remove(target, flash_bp, 1);
			if (ret != ERROR_OK) {
				LOG_TARGET_ERROR(target,
					"Failed to remove SW flash BP @ "
					TARGET_ADDR_FMT " (%d)!",
					flash_bp->bp_address,
					ret);
				return ret;
			}
		}
	}
	memset(esp->flash_brps.brps, 0, ESP_FLASH_BREAKPOINTS_MAX_NUM * sizeof(struct esp_flash_breakpoint));
	return ERROR_OK;
}

static bool esp_common_any_pending_flash_breakpoint(struct esp_common *esp)
{
	for (uint32_t slot = 0; slot < ESP_FLASH_BREAKPOINTS_MAX_NUM; slot++) {
		if (esp->flash_brps.brps[slot].status == ESP_BP_STAT_PEND)
			return true;
	}
	return false;
}

static bool esp_common_any_added_flash_breakpoint(struct esp_common *esp)
{
	for (uint32_t slot = 0; slot < ESP_FLASH_BREAKPOINTS_MAX_NUM; slot++) {
		if (esp->flash_brps.brps[slot].insn_sz > 0)
			return true;
	}
	return false;
}

static void esp_common_flash_breakpoints_get_ready_to_remove(struct esp_common *esp)
{
	for (uint32_t slot = 0; slot < ESP_FLASH_BREAKPOINTS_MAX_NUM; slot++) {
		if (esp->flash_brps.brps[slot].insn_sz > 0) {
			esp->flash_brps.brps[slot].action = ESP_BP_ACT_REM;
			esp->flash_brps.brps[slot].status = ESP_BP_STAT_PEND;
		}
	}
}

bool esp_common_flash_breakpoint_exists(struct esp_common *esp, struct breakpoint *breakpoint)
{
	for (uint32_t slot = 0; slot < ESP_FLASH_BREAKPOINTS_MAX_NUM; slot++) {
		if (esp->flash_brps.brps[slot].bp_address == breakpoint->address)
			return true;
	}
	return false;
}

int esp_common_flash_breakpoint_add(struct target *target, struct esp_common *esp, struct breakpoint *breakpoint)
{
	size_t slot;
	struct esp_flash_breakpoints *flash_bps = &esp->flash_brps;

	/*
	If the same address has remove-pending state, that means breakpoint already inserted and not removed from flash
	It doesn't need to be added again. We will change it's status as add-done
	*/
	for (slot = 0; slot < ESP_FLASH_BREAKPOINTS_MAX_NUM; slot++) {
		/* Do not check ocd_bp here since it could be freed before we complete the lazy process */
		if (flash_bps->brps[slot].bp_address == breakpoint->address) {
			if (flash_bps->brps[slot].action == ESP_BP_ACT_REM && flash_bps->brps[slot].status == ESP_BP_STAT_PEND) {
				flash_bps->brps[slot].action = ESP_BP_ACT_ADD;
				flash_bps->brps[slot].status = ESP_BP_STAT_DONE;
				esp_common_dump_bp_slot("BP-ADD(fake)", flash_bps, slot);
				return ERROR_OK;
			}
		}
	}

	/* Find an empty slot and add the new breakpoint as add-pending */
	for (slot = 0; slot < ESP_FLASH_BREAKPOINTS_MAX_NUM; slot++) {
		if (flash_bps->brps[slot].action == ESP_BP_ACT_REM && flash_bps->brps[slot].status == ESP_BP_STAT_DONE)
			break;
	}
	if (slot == ESP_FLASH_BREAKPOINTS_MAX_NUM) {
		LOG_TARGET_WARNING(target, "max SW flash slot reached, slot=%zu", slot);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	int ret = flash_bps->ops->breakpoint_prepare(target, breakpoint, &flash_bps->brps[slot]);
	if (ret != ERROR_OK)
		return ret;

	if (esp->breakpoint_lazy_process) {
		flash_bps->brps[slot].action = ESP_BP_ACT_ADD;
		flash_bps->brps[slot].status = ESP_BP_STAT_PEND;
		esp_common_dump_bp_slot("BP-ADD(new)", flash_bps, slot);
		return ERROR_OK;
	}

	return flash_bps->ops->breakpoint_add(target, &flash_bps->brps[slot], 1);
}

int esp_common_flash_breakpoint_remove(struct target *target, struct esp_common *esp, struct breakpoint *breakpoint)
{
	struct esp_flash_breakpoint *flash_bps = esp->flash_brps.brps;
	size_t slot;

	for (slot = 0; slot < ESP_FLASH_BREAKPOINTS_MAX_NUM; slot++) {
		if (flash_bps[slot].action == ESP_BP_ACT_ADD && flash_bps[slot].status == ESP_BP_STAT_DONE &&
			flash_bps[slot].bp_address == breakpoint->address)
			break;
	}

	if (slot == ESP_FLASH_BREAKPOINTS_MAX_NUM) {
		if (esp->breakpoint_lazy_process) {
			/* This is not an error since breakpoints are already removed inside qxfer-thread-read-end event */
			return ERROR_OK;
		}
		LOG_TARGET_DEBUG(target, "max SW flash slot reached, slot=%zu", slot);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* Mark that z1 package received for this slot */
	if (esp->breakpoint_lazy_process) {
		flash_bps[slot].action = ESP_BP_ACT_REM;
		flash_bps[slot].status = ESP_BP_STAT_PEND;
		esp_common_dump_bp_slot("BP-REMOVE", &esp->flash_brps, slot);
		return ERROR_OK;
	}

	return esp->flash_brps.ops->breakpoint_remove(target, &esp->flash_brps.brps[slot], 1);
}

static int esp_common_process_lazy_flash_breakpoints(struct target *target)
{
	struct esp_common *esp = target_to_esp_common(target);
	struct esp_flash_breakpoint *flash_bps = esp->flash_brps.brps;
	int ret;

	for (size_t i = 0; i < ESP_FLASH_BREAKPOINTS_MAX_NUM; ++i)
		esp_common_dump_bp_slot("BP-PROCESS", &esp->flash_brps, i);

	size_t add_num_bps = 0;
	size_t remove_num_bps = 0;
	size_t first_pending_off = ESP_FLASH_BREAKPOINTS_MAX_NUM;
	for (size_t i = 0; i < ESP_FLASH_BREAKPOINTS_MAX_NUM; ++i) {
		if (flash_bps[i].status != ESP_BP_STAT_PEND)
			continue;
		if (first_pending_off == ESP_FLASH_BREAKPOINTS_MAX_NUM)
			first_pending_off = i;
		flash_bps[i].action == ESP_BP_ACT_ADD ?  ++add_num_bps : ++remove_num_bps;
	}

	if (add_num_bps + remove_num_bps == 0) {
		/* No breakpoints in the cache. Nothing to do */
		return ERROR_OK;
	}

	LOG_TARGET_DEBUG(target, "BP num in the cache: add(%zu) + rem(%zu) from off(%zu)",
		add_num_bps, remove_num_bps, first_pending_off);

	/* If both possible pending states in the cache, add them one by one in the order they were added */
	if (add_num_bps && remove_num_bps) {
		for (size_t slot = 0; slot < ESP_FLASH_BREAKPOINTS_MAX_NUM; ++slot) {
			if (flash_bps[slot].status == ESP_BP_STAT_PEND) {
				if (flash_bps[slot].action == ESP_BP_ACT_ADD)
					ret = esp->flash_brps.ops->breakpoint_add(target, &flash_bps[slot], 1);
				else
					ret = esp->flash_brps.ops->breakpoint_remove(target, &flash_bps[slot], 1);
				if (ret != ERROR_OK) {
					LOG_TARGET_ERROR(target, "Breakpoints couldn't be processed at slot (%zu)", slot);
					return ret;
				}
			}
		}

		return ERROR_OK;
	}

	/* If all bps have same add or remove pending state, add all in one shot from the first pending index */
	flash_bps = &esp->flash_brps.brps[first_pending_off];
	if (add_num_bps)
		ret = esp->flash_brps.ops->breakpoint_add(target, flash_bps, add_num_bps);
	else
		ret = esp->flash_brps.ops->breakpoint_remove(target, flash_bps, remove_num_bps);

	if (ret != ERROR_OK)
		LOG_TARGET_ERROR(target, "Breakpoints couldn't be processed");

	return ret;
}

static int esp_common_halt_target(struct target *target, enum target_state *old_state)
{
	*old_state = target->state;
	if (target->state != TARGET_HALTED) {
		int ret = target_halt(target);
		if (ret != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Failed to halt target (%d)!", ret);
			return ret;
		}
		ret = target_wait_state(target, TARGET_HALTED, 3000);
		if (ret != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Failed to wait halted target (%d)!", ret);
			return ret;
		}
	}

	return ERROR_OK;
}

/* TODO: Prefer keeping OpenOCD default and not change it in the C code.
 * If for any reason a user wants a different behavior, he/she can use a TCL event handler
 */
int esp_common_handle_gdb_detach(struct target *target)
{
	struct esp_common *esp = target_to_esp_common(target);

	if (!esp_common_any_added_flash_breakpoint(esp))
		return ERROR_OK;

	enum target_state old_state;

	int ret = esp_common_halt_target(target, &old_state);
	if (ret != ERROR_OK)
		return ret;

	if (esp->breakpoint_lazy_process) {
		esp_common_flash_breakpoints_get_ready_to_remove(esp);
		ret = esp_common_process_lazy_flash_breakpoints(target);
	} else {
		ret = esp_common_flash_breakpoints_clear(target);
	}
	if (ret != ERROR_OK)
		return ret;

	if (old_state == TARGET_RUNNING) {
		ret = target_resume(target, 1, 0, 1, 0);
		if (ret != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Failed to resume target after flash BPs removal (%d)!", ret);
			return ret;
		}
	}
	return ERROR_OK;
}

static int esp_common_handle_flash_breakpoints(struct target *target)
{
	struct esp_common *esp = target_to_esp_common(target);

	if (!esp->breakpoint_lazy_process)
		return ERROR_OK;

	if (!esp_common_any_pending_flash_breakpoint(esp))
		return ERROR_OK;

	enum target_state old_state;

	int ret = esp_common_halt_target(target, &old_state);
	if (ret != ERROR_OK)
		return ret;

	ret = esp_common_process_lazy_flash_breakpoints(target);
	if (ret != ERROR_OK)
		return ret;

	if (old_state == TARGET_RUNNING) {
		ret = target_resume(target, 1, 0, 1, 0);
		if (ret != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Failed to resume target after processing flash BPs (%d)!", ret);
			return ret;
		}
	}
	return ERROR_OK;
}

void esp_common_assist_debug_monitor_disable(struct target *target, uint32_t address, uint32_t *value)
{
	LOG_TARGET_DEBUG(target, "addr 0x%08" PRIx32, address);

	if (address == ESP_ASSIST_DEBUG_INVALID_VALUE) {
		*value = ESP_ASSIST_DEBUG_INVALID_VALUE;
		LOG_TARGET_DEBUG(target, "assist debug monitor feature is not supported yet!");
		return;
	}

	int res = target_read_u32(target, address, value);
	if (res != ERROR_OK) {
		LOG_ERROR("Can not read assist_debug register (%d)!", res);
		*value = ESP_ASSIST_DEBUG_INVALID_VALUE;
		return;
	}
	LOG_TARGET_DEBUG(target, "Saved register value 0x%08" PRIx32, *value);

	res = target_write_u32(target, address, 0);
	if (res != ERROR_OK) {
		LOG_ERROR("Can not write assist_debug register (%d)!", res);
		*value = ESP_ASSIST_DEBUG_INVALID_VALUE;
	}
}

void esp_common_assist_debug_monitor_restore(struct target *target, uint32_t address, uint32_t value)
{
	LOG_TARGET_DEBUG(target, "value 0x%08" PRIx32 " addr 0x%08" PRIx32, value, address);

	/* value was not set by disable function */
	if (value == ESP_ASSIST_DEBUG_INVALID_VALUE)
		return;

	int res = target_write_u32(target, address, value);
	if (res != ERROR_OK)
		LOG_ERROR("Can not restore assist_debug register (%d)!", res);
}

int esp_common_read_pseudo_ex_reason(struct target *target)
{
	struct esp_common *esp = target_to_esp_common(target);
	if (esp && esp->panic_reason.addr) {
		uint8_t str[esp->panic_reason.len + 1];
		memset(str, 0x00, sizeof(str));
		int retval = target_read_memory(target, esp->panic_reason.addr, 1, esp->panic_reason.len, str);
		if (retval == ERROR_OK)
			LOG_TARGET_INFO(target, "Halt cause (%s)", str);
		else
			LOG_TARGET_ERROR(target, "Pseudo exception reason read failed (%d)", retval);
		esp->panic_reason.addr = 0;
		return ERROR_OK;
	}

	return ERROR_FAIL;
}

/* Generic commands for xtensa and riscv */

static int esp_common_gdb_detach_handler(struct target *target)
{
	if (target->smp) {
		struct target_list *head;
		foreach_smp_target(head, target->smp_targets) {
			int ret = esp_common_handle_gdb_detach(head->target);
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return esp_common_handle_gdb_detach(target);
}

static int esp_common_process_flash_breakpoints_handler(struct target *target)
{
	if (target->smp) {
		struct target_list *head;
		foreach_smp_target(head, target->smp_targets) {
			int ret = esp_common_handle_flash_breakpoints(head->target);
			if (ret != ERROR_OK)
				return ret;
		}
		return ERROR_OK;
	}
	return esp_common_handle_flash_breakpoints(target);
}

static int esp_common_disable_lazy_breakpoints_handler(struct target *target)
{
	/* Before disabling, add/remove pending breakpoints */
	int ret = esp_common_process_flash_breakpoints_handler(target);
	if (ret != ERROR_OK)
		return ret;

	if (target->smp) {
		struct target_list *head;
		foreach_smp_target(head, target->smp_targets) {
			target_to_esp_common(target)->breakpoint_lazy_process = false;
		}
		return ERROR_OK;
	}

	target_to_esp_common(target)->breakpoint_lazy_process = false;

	return ERROR_OK;
}

int esp_common_process_flash_breakpoints_command(struct command_invocation *cmd)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);

	return esp_common_process_flash_breakpoints_handler(target);
}

int esp_common_disable_lazy_breakpoints_command(struct command_invocation *cmd)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);

	int ret = esp_common_disable_lazy_breakpoints_handler(target);

	if (ret == ERROR_OK)
		command_print(CMD, "disabled");
	else
		command_print(CMD, "failed");

	return ret;
}

static int esp_callback_event_handler(struct target *target, enum target_event event, void *priv)
{
	switch (event) {
		case TARGET_EVENT_STEP_START:
		case TARGET_EVENT_RESUME_START:
		case TARGET_EVENT_QXFER_THREAD_READ_END:
			return esp_common_process_flash_breakpoints_handler(target);
		case TARGET_EVENT_GDB_DETACH:
			return esp_common_gdb_detach_handler(target);
#if IS_ESPIDF
		case TARGET_EVENT_EXAMINE_FAIL:
			extern int examine_failed_ui_handler(struct command_invocation *cmd);
			return examine_failed_ui_handler;
#endif
		default:
			break;
	}

	return ERROR_OK;
}
