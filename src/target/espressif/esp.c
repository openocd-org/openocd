// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Espressif chips common target API for OpenOCD                         *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>
#include <helper/binarybuffer.h>
#include "target/target.h"
#include "esp.h"

int esp_dbgstubs_table_read(struct target *target, struct esp_dbg_stubs *dbg_stubs)
{
	uint32_t table_size, table_start_id, desc_entry_id, gcov_entry_id;
	uint32_t entries[ESP_DBG_STUB_ENTRY_MAX] = {0};
	uint8_t entry_buff[sizeof(entries)] = {0}; /* to avoid endiannes issues */

	LOG_TARGET_DEBUG(target, "Read debug stubs info %" PRIx32 " / %d", dbg_stubs->base, dbg_stubs->entries_count);

	/* First of, read 2 entries to get magic num and table size */
	int res = target_read_buffer(target, dbg_stubs->base, sizeof(uint32_t) * 2, entry_buff);
	if (res != ERROR_OK) {
		LOG_ERROR("%s: Failed to read first debug stub entry!", target_name(target));
		return res;
	}
	entries[0] = target_buffer_get_u32(target, entry_buff);
	entries[1] = target_buffer_get_u32(target, entry_buff + sizeof(uint32_t));

	if (entries[0] != ESP_DBG_STUB_MAGIC_NUM_VAL) {
		/* idf with the old table entry structure */
		table_size = 2;
		table_start_id = 0;
		desc_entry_id = 0;
		gcov_entry_id = 1;
	} else {
		table_size = entries[1];
		table_start_id = ESP_DBG_STUB_TABLE_START;
		desc_entry_id = ESP_DBG_STUB_TABLE_START;
		gcov_entry_id = ESP_DBG_STUB_ENTRY_FIRST;

		/* discard unsupported entries */
		if (table_size > ESP_DBG_STUB_ENTRY_MAX)
			table_size = ESP_DBG_STUB_ENTRY_MAX;

		/* now read the remaining entries */
		res = target_read_buffer(target, dbg_stubs->base + 2 * sizeof(uint32_t), sizeof(uint32_t) * table_size - 2,
			entry_buff + sizeof(uint32_t) * 2);
		if (res != ERROR_OK) {
			LOG_TARGET_ERROR(target, "Failed to read debug stubs info!");
			return res;
		}
		for (unsigned int i = 2; i < table_size; ++i)
			entries[i] = target_buffer_get_u32(target, entry_buff + sizeof(uint32_t) * i);

		dbg_stubs->entries[ESP_DBG_STUB_CAPABILITIES] = entries[ESP_DBG_STUB_CAPABILITIES];
	}

	dbg_stubs->entries[ESP_DBG_STUB_DESC] = entries[desc_entry_id];
	dbg_stubs->entries[ESP_DBG_STUB_ENTRY_GCOV] = entries[gcov_entry_id];

	for (enum esp_dbg_stub_id i = ESP_DBG_STUB_DESC; i < ESP_DBG_STUB_ENTRY_MAX; i++) {
		LOG_DEBUG("Check dbg stub %d - %x", i, dbg_stubs->entries[i]);
		if (dbg_stubs->entries[i]) {
			LOG_DEBUG("New dbg stub %d at %x", dbg_stubs->entries_count, dbg_stubs->entries[i]);
			dbg_stubs->entries_count++;
		}
	}
	if (dbg_stubs->entries_count < table_size - table_start_id)
		LOG_WARNING("Not full dbg stub table %d of %d", dbg_stubs->entries_count, table_size - table_start_id);

	return ERROR_OK;
}
