// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Espressif Xtensa target API for OpenOCD                               *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <stdbool.h>
#include <stdint.h>
#include <target/smp.h>
#include <target/register.h>
#include "esp.h"
#include "esp_xtensa.h"
#include "esp_xtensa_apptrace.h"
#include "esp_semihosting.h"
#include "esp_xtensa_algorithm.h"

#define ESP_XTENSA_DBGSTUBS_UPDATE_DATA_ENTRY(_e_) \
	do { \
		uint32_t __internal_val = (_e_); \
		if (!xtensa_data_addr_valid(target, __internal_val)) { \
			LOG_ERROR("No valid stub data entry found (0x%" PRIx32 ")!", __internal_val); \
			return;	\
		} \
	} while (0)

#define ESP_XTENSA_DBGSTUBS_UPDATE_CODE_ENTRY(_e_) \
	do { \
		uint32_t __internal_val = (_e_); \
		if (__internal_val == 0) { \
			LOG_ERROR("No valid stub code entry found (0x%" PRIx32 ")!", __internal_val); \
			return;	\
		} \
	} while (0)

static void esp_xtensa_dbgstubs_info_update(struct target *target);
static void esp_xtensa_dbgstubs_addr_check(struct target *target);

static int esp_xtensa_dbgstubs_restore(struct target *target)
{
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);

	if (esp_xtensa->esp.dbg_stubs.base == 0)
		return ERROR_OK;

	LOG_TARGET_INFO(target, "Restore debug stubs address %" PRIx32, esp_xtensa->esp.dbg_stubs.base);
	int res = esp_xtensa_apptrace_status_reg_write(target, esp_xtensa->esp.dbg_stubs.base);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to write trace status (%d)!", res);
		return res;
	}
	return ERROR_OK;
}
int esp_xtensa_on_halt(struct target *target)
{
	/* debug stubs can be used in HALTED state only, so it is OK to get info about them here */
	esp_xtensa_dbgstubs_info_update(target);
	return ERROR_OK;
}

int esp_xtensa_init_arch_info(struct target *target,
	struct esp_xtensa_common *esp_xtensa,
	struct xtensa_debug_module_config *dm_cfg,
	const struct esp_semihost_ops *semihost_ops)
{
	int ret = xtensa_init_arch_info(target, &esp_xtensa->xtensa, dm_cfg);
	if (ret != ERROR_OK)
		return ret;
	ret = esp_common_init(&esp_xtensa->esp, &xtensa_algo_hw);
	if (ret != ERROR_OK)
		return ret;

	esp_xtensa->semihost.ops = (struct esp_semihost_ops *)semihost_ops;
	esp_xtensa->apptrace.hw = &esp_xtensa_apptrace_hw;
	return ERROR_OK;
}

int esp_xtensa_target_init(struct command_context *cmd_ctx, struct target *target)
{
	return xtensa_target_init(cmd_ctx, target);
}

void esp_xtensa_target_deinit(struct target *target)
{
	LOG_DEBUG("start");

	if (target_was_examined(target)) {
		int ret = esp_xtensa_dbgstubs_restore(target);
		if (ret != ERROR_OK)
			return;
	}
	xtensa_target_deinit(target);
	free(target_to_esp_xtensa(target));	/* same as free(xtensa) */
}

int esp_xtensa_arch_state(struct target *target)
{
	return ERROR_OK;
}

int esp_xtensa_poll(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	struct esp_xtensa_common *esp_xtensa_common = target_to_esp_xtensa(target);

	int ret = xtensa_poll(target);

	if (xtensa_dm_power_status_get(&xtensa->dbg_mod) & PWRSTAT_COREWASRESET(xtensa)) {
		LOG_TARGET_DEBUG(target, "Clear debug stubs info");
		memset(&esp_xtensa_common->esp.dbg_stubs, 0, sizeof(esp_xtensa_common->esp.dbg_stubs));
	}
	if (target->state != TARGET_DEBUG_RUNNING)
		esp_xtensa_dbgstubs_addr_check(target);
	return ret;
}

static void esp_xtensa_dbgstubs_addr_check(struct target *target)
{
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);
	uint32_t vec_addr = 0;

	if (esp_xtensa->esp.dbg_stubs.base != 0)
		return;

	int res = esp_xtensa_apptrace_status_reg_read(target, &vec_addr);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to read debug stubs address location (%d)!", res);
		return;
	}
	if (xtensa_data_addr_valid(target, vec_addr)) {
		LOG_TARGET_INFO(target, "Detected debug stubs @ %" PRIx32, vec_addr);
		res = esp_xtensa_apptrace_status_reg_write(target, 0);
		if (res != ERROR_OK)
			LOG_ERROR("Failed to clear debug stubs address location (%d)!", res);
		esp_xtensa->esp.dbg_stubs.base = vec_addr;
	}
}

static void esp_xtensa_dbgstubs_info_update(struct target *target)
{
	struct esp_xtensa_common *esp_xtensa = target_to_esp_xtensa(target);

	if (esp_xtensa->esp.dbg_stubs.base == 0 || esp_xtensa->esp.dbg_stubs.entries_count != 0)
		return;

	int res = esp_dbgstubs_table_read(target, &esp_xtensa->esp.dbg_stubs);
	if (res != ERROR_OK)
		return;
	if (esp_xtensa->esp.dbg_stubs.entries_count == 0)
		return;

	/* read debug stubs descriptor */
	ESP_XTENSA_DBGSTUBS_UPDATE_DATA_ENTRY(esp_xtensa->esp.dbg_stubs.entries[ESP_DBG_STUB_DESC]);
	res = target_read_buffer(target, esp_xtensa->esp.dbg_stubs.entries[ESP_DBG_STUB_DESC],
		sizeof(struct esp_dbg_stubs_desc),
		(uint8_t *)&esp_xtensa->esp.dbg_stubs.desc);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to read debug stubs descriptor (%d)!", res);
		return;
	}
	ESP_XTENSA_DBGSTUBS_UPDATE_CODE_ENTRY(esp_xtensa->esp.dbg_stubs.desc.tramp_addr);
	ESP_XTENSA_DBGSTUBS_UPDATE_DATA_ENTRY(esp_xtensa->esp.dbg_stubs.desc.min_stack_addr);
	ESP_XTENSA_DBGSTUBS_UPDATE_CODE_ENTRY(esp_xtensa->esp.dbg_stubs.desc.data_alloc);
	ESP_XTENSA_DBGSTUBS_UPDATE_CODE_ENTRY(esp_xtensa->esp.dbg_stubs.desc.data_free);
}

int esp_xtensa_breakpoint_add(struct target *target, struct breakpoint *breakpoint)
{
	return xtensa_breakpoint_add(target, breakpoint);
	/* flash breakpoints will be handled in another patch */
}

int esp_xtensa_breakpoint_remove(struct target *target, struct breakpoint *breakpoint)
{
	return xtensa_breakpoint_remove(target, breakpoint);
	/* flash breakpoints will be handled in another patch */
}

int esp_xtensa_profiling(struct target *target, uint32_t *samples,
	uint32_t max_num_samples, uint32_t *num_samples, uint32_t seconds)
{
	struct timeval timeout, now;
	struct xtensa *xtensa = target_to_xtensa(target);
	int retval = ERROR_OK;
	int res;

	/* Vary samples per pass to avoid sampling a periodic function periodically */
	#define MIN_PASS 200
	#define MAX_PASS 1000

	gettimeofday(&timeout, NULL);
	timeval_add_time(&timeout, seconds, 0);

	uint8_t buf[sizeof(uint32_t) * MAX_PASS];

	/* Capture one sample to verify the register is present and working */
	xtensa_queue_dbg_reg_read(xtensa, XDMREG_DEBUGPC, buf);
	res = xtensa_dm_queue_execute(&xtensa->dbg_mod);
	if (res != ERROR_OK) {
		LOG_TARGET_INFO(target, "Failed to read DEBUGPC, fallback to stop-and-go");
		return target_profiling_default(target, samples, max_num_samples, num_samples, seconds);
	} else if (buf[0] == 0 && buf[1] == 0 && buf[2] == 0 && buf[3] == 0) {
		LOG_TARGET_INFO(target, "NULL DEBUGPC, fallback to stop-and-go");
		return target_profiling_default(target, samples, max_num_samples, num_samples, seconds);
	}

	LOG_TARGET_INFO(target, "Starting XTENSA DEBUGPC profiling. Sampling as fast as we can...");

	/* Make sure the target is running */
	target_poll(target);
	if (target->state == TARGET_HALTED)
		retval = target_resume(target, true, 0, false, false);

	if (retval != ERROR_OK) {
		LOG_TARGET_ERROR(target, "Error while resuming target");
		return retval;
	}

	uint32_t sample_count = 0;

	for (;;) {
		uint32_t remaining = max_num_samples - sample_count;
		uint32_t this_pass = rand() % (MAX_PASS - MIN_PASS) + MIN_PASS;
		this_pass = this_pass > remaining ? remaining : this_pass;
		for (uint32_t i = 0; i < this_pass; ++i)
			xtensa_queue_dbg_reg_read(xtensa, XDMREG_DEBUGPC, buf + i * sizeof(uint32_t));
		res = xtensa_dm_queue_execute(&xtensa->dbg_mod);
		if (res != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Failed to read DEBUGPC!");
			return res;
		}

		for (uint32_t i = 0; i < this_pass; ++i) {
			uint32_t sample32 = buf_get_u32(buf + i * sizeof(uint32_t), 0, 32);
			samples[sample_count++] = sample32;
		}
		gettimeofday(&now, NULL);
		if (sample_count >= max_num_samples || timeval_compare(&now, &timeout) > 0) {
			LOG_TARGET_INFO(target, "Profiling completed. %" PRIu32 " samples.", sample_count);
			break;
		}
	}

	*num_samples = sample_count;
	return retval;

	#undef MIN_PASS
	#undef MAX_PASS
}
