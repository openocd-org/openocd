/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Xtensa application tracing module for OpenOCD                         *
 *   Copyright (C) 2017 Espressif Systems Ltd.                             *
 ***************************************************************************/

/*
    How it works?
    https://github.com/espressif/esp-idf/blob/master/components/app_trace/port/xtensa/port.c#L8
*/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/align.h>
#include <helper/crc16.h>
#include <target/xtensa/xtensa.h>
#include <target/xtensa/xtensa_debug_module.h>
#include "esp_xtensa_apptrace.h"
#include <target/target_type.h>

/* TRAX is disabled, so we use its registers for our own purposes
 * | 31..XXXXXX..24 | 23 .(host_connect). 23 | 22 .(host_data). 22| 21..(block_id)..15 | 14..(block_len)..0 |
 */
#define XTENSA_APPTRACE_CTRL_REG                XDMREG_DELAYCNT
#define XTENSA_APPTRACE_BLOCK_ID_MSK            0x7FUL
#define XTENSA_APPTRACE_BLOCK_ID_MAX            XTENSA_APPTRACE_BLOCK_ID_MSK
/* if non-zero then apptrace code entered the critical section and the value is an address of the
 * critical section's exit point */
#define XTENSA_APPTRACE_STAT_REG                XDMREG_TRIGGERPC
#define XTENSA_APPTRACE_CRC_REG                 XDMREG_PM1

#define XTENSA_APPTRACE_CRC_MSK                 0xFFFF
#define XTENSA_APPTRACE_CRC_INDICATOR           (0xA55AU << 16)

#define XTENSA_APPTRACE_BLOCK_LEN_MSK           0x7FFFUL
#define XTENSA_APPTRACE_BLOCK_LEN(_l_)          ((_l_) & XTENSA_APPTRACE_BLOCK_LEN_MSK)
#define XTENSA_APPTRACE_BLOCK_LEN_GET(_v_)      ((_v_) & XTENSA_APPTRACE_BLOCK_LEN_MSK)
#define XTENSA_APPTRACE_BLOCK_ID(_id_)          (((_id_) & XTENSA_APPTRACE_BLOCK_ID_MSK) << 15)
#define XTENSA_APPTRACE_BLOCK_ID_GET(_v_)       (((_v_) >> 15) & XTENSA_APPTRACE_BLOCK_ID_MSK)
#define XTENSA_APPTRACE_HOST_DATA               BIT(22)
#define XTENSA_APPTRACE_HOST_CONNECT            BIT(23)

static int esp_xtensa_apptrace_leave_crit_section_start(struct target *target);
static int esp_xtensa_apptrace_leave_crit_section_stop(struct target *target);
static int esp_xtensa_apptrace_buffs_write(struct target *target,
	uint32_t bufs_num,
	uint32_t buf_sz[],
	const uint8_t *bufs[],
	uint32_t block_id,
	bool ack,
	bool data);

struct esp32_apptrace_hw esp_xtensa_apptrace_hw = {
	.max_block_id = XTENSA_APPTRACE_BLOCK_ID_MAX,
	.max_block_size_get = esp_xtensa_apptrace_block_max_size_get,
	.status_reg_read = esp_xtensa_apptrace_status_reg_read,
	.ctrl_reg_write = esp_xtensa_apptrace_ctrl_reg_write,
	.ctrl_reg_read = esp_xtensa_apptrace_ctrl_reg_read,
	.data_len_read = esp_xtensa_apptrace_data_len_read,
	.data_read = esp_xtensa_apptrace_data_read,
	.usr_block_max_size_get = esp_xtensa_apptrace_usr_block_max_size_get,
	.buffs_write = esp_xtensa_apptrace_buffs_write,
	.leave_trace_crit_section_start = esp_xtensa_apptrace_leave_crit_section_start,
	.leave_trace_crit_section_stop = esp_xtensa_apptrace_leave_crit_section_stop,
	.apptrace_is_inited = NULL
};

static int esp_xtensa_apptrace_debug_reg_read(struct target *target, uint32_t reg, uint32_t *val)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	uint8_t tmp[4];

	int res = xtensa_queue_dbg_reg_read(xtensa, reg, tmp);
	if (res != ERROR_OK)
		return res;
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	res = xtensa_dm_queue_execute(&xtensa->dbg_mod);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to exec JTAG queue!");
		return res;
	}
	*val = buf_get_u32(tmp, 0, 32);
	return ERROR_OK;
}

static int esp_xtensa_apptrace_debug_reg_write(struct target *target, uint32_t reg, uint32_t val)
{
	struct xtensa *xtensa = target_to_xtensa(target);

	xtensa_queue_dbg_reg_write(xtensa, reg, val);
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	int res = xtensa_dm_queue_execute(&xtensa->dbg_mod);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to exec JTAG queue!");
		return res;
	}
	return ERROR_OK;
}

uint32_t esp_xtensa_apptrace_block_max_size_get(struct target *target)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	struct xtensa_trace_status trace_status;
	struct xtensa_trace_config trace_config;
	uint32_t max_trace_block_sz;

	int res = xtensa_dm_trace_status_read(&xtensa->dbg_mod, &trace_status);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to read TRAX status (%d)!", res);
		return 0;
	}

	max_trace_block_sz = BIT(((trace_status.stat >> 8) & 0x1f) - 2) * 4;
	res = xtensa_dm_trace_config_read(&xtensa->dbg_mod, &trace_config);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to read TRAX config (%d)!", res);
		return 0;
	}
	LOG_DEBUG("ctrl=0x%" PRIx32 " memadrstart=0x%" PRIx32 " memadrend=0x%" PRIx32 " traxadr=0x%" PRIx32,
		trace_config.ctrl,
		trace_config.memaddr_start,
		trace_config.memaddr_end,
		trace_config.addr);

	return max_trace_block_sz;
}

uint32_t esp_xtensa_apptrace_usr_block_max_size_get(struct target *target)
{
	return esp_xtensa_apptrace_block_max_size_get(target) - sizeof(struct esp_apptrace_host2target_hdr);
}

int esp_xtensa_apptrace_data_len_read(struct target *target,
	uint32_t *block_id,
	uint32_t *len)
{
	return esp_xtensa_apptrace_ctrl_reg_read(target, block_id, len, NULL);
}

int esp_xtensa_apptrace_usr_block_write(struct target *target,
	uint32_t block_id,
	const uint8_t *data,
	uint32_t size)
{
	return esp_apptrace_usr_block_write(&esp_xtensa_apptrace_hw, target, block_id, data, size);
}

static int esp_xtensa_apptrace_data_reverse_read(struct xtensa *xtensa,
	uint32_t size,
	uint8_t *buffer,
	uint8_t *unal_bytes)
{
	int res = 0;
	uint32_t rd_sz = ALIGN_UP(size, 4);

	res = xtensa_queue_dbg_reg_write(xtensa, XDMREG_TRAXADDR, (xtensa->core_config->trace.mem_sz - rd_sz) / 4);
	if (res != ERROR_OK)
		return res;
	if (!IS_ALIGNED(size, 4)) {
		res = xtensa_queue_dbg_reg_read(xtensa, XDMREG_TRAXDATA, unal_bytes);
		if (res != ERROR_OK)
			return res;
	}
	for (unsigned int i = size / 4; i != 0; i--) {
		res = xtensa_queue_dbg_reg_read(xtensa, XDMREG_TRAXDATA, &buffer[(i - 1) * 4]);
		if (res != ERROR_OK)
			return res;
	}

	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	res = xtensa_dm_queue_execute(&xtensa->dbg_mod);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to exec JTAG queue!");
		return res;
	}

	return ERROR_OK;
}

static int esp_xtensa_apptrace_data_normal_read(struct xtensa *xtensa,
	uint32_t size,
	uint8_t *buffer,
	uint8_t *unal_bytes,
	bool wait)
{
	int res = xtensa_queue_dbg_reg_write(xtensa, XDMREG_TRAXADDR, 0);
	if (res != ERROR_OK)
		return res;
	for (unsigned int i = 0; i < size / 4; i++) {
		res = xtensa_queue_dbg_reg_read(xtensa, XDMREG_TRAXDATA, &buffer[i * 4]);
		if (res != ERROR_OK)
			return res;
	}
	if (!IS_ALIGNED(size, 4)) {
		res = xtensa_queue_dbg_reg_read(xtensa, XDMREG_TRAXDATA, unal_bytes);
		if (res != ERROR_OK)
			return res;
	}

	if (wait) {
		/* Workaround for esp32s3 data corruption while reading the high volume of data */
		/* Don't have clear reason for this, might fix the priority issue with the target memory access and tracing */
		usleep(5000 * (size / 256));
	}

	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	res = xtensa_dm_queue_execute(&xtensa->dbg_mod);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to exec JTAG queue!");
		return res;
	}

	return ERROR_OK;
}

int esp_xtensa_apptrace_data_read(struct target *target,
	uint32_t size,
	uint8_t *buffer,
	uint32_t block_id,
	bool ack)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	uint32_t tmp = XTENSA_APPTRACE_HOST_CONNECT | XTENSA_APPTRACE_BLOCK_ID(block_id) | XTENSA_APPTRACE_BLOCK_LEN(0);
	uint8_t unal_bytes[4];
	uint32_t target_crc16;
	const int MAX_TRIES = 10;
	bool wait = strcmp(target->type->name, "esp32s3") == 0;

	int res = esp_xtensa_apptrace_debug_reg_read(target, XTENSA_APPTRACE_CRC_REG, &target_crc16);
	if (res != ERROR_OK)
		return res;

	bool check_crc = (target_crc16 & XTENSA_APPTRACE_CRC_INDICATOR) == XTENSA_APPTRACE_CRC_INDICATOR ? true : false;

	target_crc16 &= XTENSA_APPTRACE_CRC_MSK; /* clear the CRC indicator bits */
	/* Sanity check */
	if (target_crc16 == 0)
		check_crc = false;

	for (int i = 1; i <= MAX_TRIES; ++i) {
		LOG_TARGET_DEBUG(target, "Read data from block %" PRIu32 " size %" PRIu32, block_id, size);
		if (xtensa->core_config->trace.reversed_mem_access)
			res = esp_xtensa_apptrace_data_reverse_read(xtensa, size, buffer, unal_bytes);
		else
			res = esp_xtensa_apptrace_data_normal_read(xtensa, size, buffer, unal_bytes, wait);
		if (res != ERROR_OK)
			return res;

		if (!IS_ALIGNED(size, 4)) {
			/* copy the last unaligned bytes */
			memcpy(buffer + ALIGN_DOWN(size, 4), unal_bytes, size & 0x3UL);
		}

		LOG_TARGET_DEBUG(target, "target crc: %" PRIx32 " check_crc: %d", target_crc16, check_crc);

		if (!check_crc)
			break;

		uint16_t crc16 = crc16_le(0, buffer, size);
		if (crc16 == target_crc16)
			break;

		LOG_WARNING("[%d/%d] CRC mismatch! calculated: 0x%" PRIx16 " read: 0x%" PRIx32,
			i, MAX_TRIES, crc16, target_crc16);
	}

	if (ack) {
		LOG_TARGET_DEBUG(target, "Ack block %" PRIu32 " write 0x%" PRIx32 " to control reg", block_id, tmp);
		res = esp_xtensa_apptrace_debug_reg_write(target, XTENSA_APPTRACE_CTRL_REG, tmp);
	}

	return res;
}

int esp_xtensa_apptrace_ctrl_reg_write(struct target *target,
	uint32_t block_id,
	uint32_t len,
	bool conn,
	bool data)
{
	uint32_t tmp = (conn ? XTENSA_APPTRACE_HOST_CONNECT : 0) |
		(data ? XTENSA_APPTRACE_HOST_DATA : 0) | XTENSA_APPTRACE_BLOCK_ID(block_id) |
		XTENSA_APPTRACE_BLOCK_LEN(len);

	return esp_xtensa_apptrace_debug_reg_write(target, XTENSA_APPTRACE_CTRL_REG, tmp);
}

int esp_xtensa_apptrace_ctrl_reg_read(struct target *target,
	uint32_t *block_id,
	uint32_t *len,
	bool *conn)
{
	uint32_t val;
	int res = esp_xtensa_apptrace_debug_reg_read(target, XTENSA_APPTRACE_CTRL_REG, &val);
	if (res != ERROR_OK)
		return res;
	if (block_id)
		*block_id = XTENSA_APPTRACE_BLOCK_ID_GET(val);
	if (len)
		*len = XTENSA_APPTRACE_BLOCK_LEN_GET(val);
	if (conn)
		*conn = val & XTENSA_APPTRACE_HOST_CONNECT;
	return ERROR_OK;
}

int esp_xtensa_apptrace_status_reg_read(struct target *target, uint32_t *stat)
{
	return esp_xtensa_apptrace_debug_reg_read(target, XTENSA_APPTRACE_STAT_REG, stat);
}

int esp_xtensa_apptrace_status_reg_write(struct target *target, uint32_t stat)
{
	return esp_xtensa_apptrace_debug_reg_write(target, XTENSA_APPTRACE_STAT_REG, stat);
}

static int esp_xtensa_swdbg_activate(struct target *target, int enab)
{
	int res = esp_xtensa_apptrace_debug_reg_write(target, enab ? XDMREG_DCRSET : XDMREG_DCRCLR, OCDDCR_DEBUGSWACTIVE);
	if (res != ERROR_OK) {
		LOG_TARGET_ERROR(target, "writing DCR failed");
		return res;
	}

	return ERROR_OK;
}

static int esp_xtensa_apptrace_leave_crit_section_start(struct target *target)
{
	/* TODO: not sure that we need this, but it seems that we fail to leave tracing critical
	 *section w/o this */
	int res = esp_xtensa_swdbg_activate(target, 1 /*enable*/);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to activate SW debug (%d)!", res);
		return res;
	}
	return ERROR_OK;
}

static int esp_xtensa_apptrace_leave_crit_section_stop(struct target *target)
{
	int res = esp_xtensa_swdbg_activate(target, 0 /*disable*/);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to activate SW debug (%d)!", res);
		return res;
	}
	return ERROR_OK;
}

static int esp_xtensa_apptrace_queue_reverse_write(struct target *target, uint32_t bufs_num,
	uint32_t buf_sz[], const uint8_t *bufs[])
{
	int res = ERROR_OK;
	uint32_t cached_bytes = 0, total_sz = 0;
	uint8_t cached_data8[sizeof(uint32_t)] = { 0 };
	uint32_t cached_data32 = 0;

	struct xtensa *xtensa = target_to_xtensa(target);

	for (uint32_t i = 0; i < bufs_num; i++)
		total_sz += buf_sz[i];
	if (!IS_ALIGNED(total_sz, 4)) {
		cached_bytes = sizeof(uint32_t) - (total_sz & 0x3UL);
		total_sz = ALIGN_UP(total_sz, 4);
	}
	xtensa_queue_dbg_reg_write(xtensa, XDMREG_TRAXADDR, (xtensa->core_config->trace.mem_sz - total_sz) / 4);
	for (uint32_t i = bufs_num; i > 0; i--) {
		uint32_t bsz = buf_sz[i - 1];
		const uint8_t *cur_buf = &bufs[i - 1][bsz];
		uint32_t bytes_to_cache;
		/* if there are cached bytes from the previous buffer, combine them with the last
		 * from the current buffer */
		if (cached_bytes) {
			if ((cached_bytes + bsz) < sizeof(uint32_t))
				bytes_to_cache = bsz;
			else
				bytes_to_cache = sizeof(uint32_t) - cached_bytes;
			memcpy(&cached_data8[sizeof(uint32_t) - cached_bytes - bytes_to_cache],
				cur_buf - bytes_to_cache,
				bytes_to_cache);
			cached_data32 = target_buffer_get_u32(target, cached_data8);
			cached_bytes += bytes_to_cache;
			if (cached_bytes < sizeof(uint32_t))
				continue;
			res = xtensa_queue_dbg_reg_write(xtensa, XDMREG_TRAXDATA, cached_data32);
			if (res != ERROR_OK)
				return res;
			bsz -= bytes_to_cache;
			cur_buf -= bytes_to_cache;
			memset(cached_data8, 0x00, sizeof(cached_data8));
			cached_bytes = 0;
		}
		/* write full dwords */
		for (unsigned int k = bsz; k >= sizeof(uint32_t); k -= sizeof(uint32_t)) {
			uint32_t temp = target_buffer_get_u32(target, cur_buf - sizeof(uint32_t));
			res = xtensa_queue_dbg_reg_write(xtensa, XDMREG_TRAXDATA, temp);
			if (res != ERROR_OK)
				return res;
			cur_buf -= sizeof(uint32_t);
		}
		/* if there are bytes to be cached (1..3) */
		bytes_to_cache = bsz & 0x3UL;
		if (bytes_to_cache > 0) {
			if (bytes_to_cache + cached_bytes >= sizeof(uint32_t)) {
				/* filling the cache buffer from the end to beginning */
				uint32_t to_copy = sizeof(uint32_t) - cached_bytes;
				memcpy(&cached_data8[0], cur_buf - to_copy, to_copy);
				cached_data32 = target_buffer_get_u32(target, cached_data8);
				/* write full word of cached bytes */
				res = xtensa_queue_dbg_reg_write(xtensa, XDMREG_TRAXDATA, cached_data32);
				if (res != ERROR_OK)
					return res;
				/* cache remaining bytes */
				memset(cached_data8, 0x00, sizeof(cached_data8));
				cur_buf -= to_copy;
				to_copy = bytes_to_cache + cached_bytes - sizeof(uint32_t);
				memcpy(&cached_data8[sizeof(uint32_t) - to_copy], cur_buf - to_copy, to_copy);
				cached_bytes = to_copy;
			} else {
				/* filling the cache buffer from the end to beginning */
				memcpy(&cached_data8[sizeof(uint32_t) - cached_bytes - bytes_to_cache],
					cur_buf - bytes_to_cache,
					bytes_to_cache);
				cached_bytes += bytes_to_cache;
			}
		}
	}
	return ERROR_OK;
}

static int esp_xtensa_apptrace_queue_normal_write(struct target *target, uint32_t bufs_num,
	uint32_t buf_sz[], const uint8_t *bufs[])
{
	int res = ERROR_OK;
	uint32_t cached_bytes = 0;
	uint8_t cached_data8[4] = { 0 };
	uint32_t cached_data32 = 0;

	struct xtensa *xtensa = target_to_xtensa(target);

	/* | 1 |   2   | 1 | 2     |       4       |.......|
	 * |       4       |       4       |       4       | */
	xtensa_queue_dbg_reg_write(xtensa, XDMREG_TRAXADDR, 0);
	for (unsigned int i = 0; i < bufs_num; i++) {
		uint32_t bsz = buf_sz[i];
		const uint8_t *cur_buf = bufs[i];
		uint32_t bytes_to_cache;
		/* if there are cached bytes from the previous buffer, combine them with the last
		 * from the current buffer */
		if (cached_bytes) {
			if ((cached_bytes + bsz) < sizeof(uint32_t))
				bytes_to_cache = bsz;
			else
				bytes_to_cache = sizeof(uint32_t) - cached_bytes;
			memcpy(&cached_data8[cached_bytes], cur_buf, bytes_to_cache);
			cached_bytes += bytes_to_cache;
			if (cached_bytes < sizeof(uint32_t))
				continue;
			cached_data32 = target_buffer_get_u32(target, cached_data8);
			res = xtensa_queue_dbg_reg_write(xtensa, XDMREG_TRAXDATA, cached_data32);
			if (res != ERROR_OK)
				return res;
			bsz -= bytes_to_cache;
			cur_buf += bytes_to_cache;
			memset(cached_data8, 0x00, sizeof(cached_data8));
			cached_bytes = 0;
		}
		/* write full dwords */
		for (unsigned int k = 0; (k + sizeof(uint32_t)) <= bsz; k += sizeof(uint32_t)) {
			uint32_t temp = target_buffer_get_u32(target, cur_buf);
			res = xtensa_queue_dbg_reg_write(xtensa, XDMREG_TRAXDATA, temp);
			if (res != ERROR_OK)
				return res;
			cur_buf += sizeof(uint32_t);
		}
		/* if there are bytes to be cached (1..3) */
		bytes_to_cache = bsz & 0x3UL;
		if (bytes_to_cache > 0) {
			if (bytes_to_cache + cached_bytes >= sizeof(uint32_t)) {
				memcpy(&cached_data8[0], cur_buf, sizeof(uint32_t) - cached_bytes);
				cached_data32 = target_buffer_get_u32(target, cached_data8);
				/* write full word of cached bytes */
				res = xtensa_queue_dbg_reg_write(xtensa, XDMREG_TRAXDATA, cached_data32);
				if (res != ERROR_OK)
					return res;
				/* cache remaining bytes */
				memset(cached_data8, 0x00, sizeof(cached_data8));
				cur_buf += sizeof(uint32_t) - cached_bytes;
				cached_bytes = bytes_to_cache + cached_bytes - sizeof(uint32_t);
				memcpy(&cached_data8[0], cur_buf, cached_bytes);
			} else {
				memcpy(&cached_data8[cached_bytes], cur_buf, bytes_to_cache);
				cached_bytes += bytes_to_cache;
			}
		}
	}
	if (cached_bytes) {
		/* write remaining cached bytes */
		cached_data32 = target_buffer_get_u32(target, cached_data8);
		res = xtensa_queue_dbg_reg_write(xtensa, XDMREG_TRAXDATA, cached_data32);
		if (res != ERROR_OK)
			return res;
	}
	return ERROR_OK;
}

static int esp_xtensa_apptrace_buffs_write(struct target *target,
	uint32_t bufs_num,
	uint32_t buf_sz[],
	const uint8_t *bufs[],
	uint32_t block_id,
	bool ack,
	bool data)
{
	struct xtensa *xtensa = target_to_xtensa(target);
	int res = ERROR_OK;
	uint32_t tmp = XTENSA_APPTRACE_HOST_CONNECT |
		(data ? XTENSA_APPTRACE_HOST_DATA : 0) | XTENSA_APPTRACE_BLOCK_ID(block_id) |
		XTENSA_APPTRACE_BLOCK_LEN(0);

	if (xtensa->core_config->trace.reversed_mem_access)
		res = esp_xtensa_apptrace_queue_reverse_write(target, bufs_num, buf_sz, bufs);
	else
		res = esp_xtensa_apptrace_queue_normal_write(target, bufs_num, buf_sz, bufs);
	if (res != ERROR_OK)
		return res;
	if (ack) {
		LOG_DEBUG("Ack block %" PRId32 " on target (%s)!", block_id, target_name(target));
		res = xtensa_queue_dbg_reg_write(xtensa, XTENSA_APPTRACE_CTRL_REG, tmp);
		if (res != ERROR_OK)
			return res;
	}
	xtensa_dm_queue_tdi_idle(&xtensa->dbg_mod);
	res = xtensa_dm_queue_execute(&xtensa->dbg_mod);
	if (res != ERROR_OK) {
		LOG_ERROR("Failed to exec JTAG queue!");
		return res;
	}
	return ERROR_OK;
}
