// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Copyright (C) 2016-2020 by Marc Schink <dev@zapb.de>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stddef.h>
#include <stdint.h>
#include <helper/log.h>
#include <helper/binarybuffer.h>
#include <helper/command.h>
#include <rtt/rtt.h>
#include <target/rtt.h>

#include "target.h"

// Offsets for RTT control block parameters.
struct rtt_control_params {
	unsigned int channel_size;
	unsigned int buffer_addr_offset;
	unsigned int size_offset;
	unsigned int write_pos_offset;
	unsigned int read_pos_offset;
	unsigned int flags_offset;
};

// Offsets for 32-bit architecture.
static const struct rtt_control_params rtt_params_32 = {
	.channel_size    = RTT_CHANNEL_SIZE_32,
	.buffer_addr_offset = 4,
	.size_offset        = 8,
	.write_pos_offset   = 12,
	.read_pos_offset    = 16,
	.flags_offset       = 20
};

// Offsets for 64-bit architecture.
static const struct rtt_control_params rtt_params_64 = {
	.channel_size    = RTT_CHANNEL_SIZE_64,
	.buffer_addr_offset = 8,
	.size_offset        = 16,
	.write_pos_offset   = 20,
	.read_pos_offset    = 24,
	.flags_offset       = 28
};

static const struct rtt_control_params *get_rtt_params(struct target *target)
{
	if (target_address_bits(target) == 64)
		return &rtt_params_64;
	return &rtt_params_32;
}

static int read_rtt_channel(struct target *target,
		const struct rtt_control *ctrl, unsigned int channel_index,
		enum rtt_channel_type type, struct rtt_channel *channel)
{
	int ret;
	uint8_t buf[RTT_CHANNEL_SIZE_64];
	target_addr_t address;
	const struct rtt_control_params *params = get_rtt_params(target);

	address = ctrl->address + RTT_CB_SIZE + (channel_index * params->channel_size);

	if (type == RTT_CHANNEL_TYPE_DOWN)
		address += ctrl->num_up_channels * params->channel_size;

	ret = target_read_buffer(target, address, params->channel_size, buf);

	if (ret != ERROR_OK)
		return ret;

	channel->address = address;
	if (target_address_bits(target) == 64) {
		channel->name_addr = target_buffer_get_u64(target, buf + 0);
		channel->buffer_addr = target_buffer_get_u64(target, buf + params->buffer_addr_offset);
	} else {
		channel->name_addr = target_buffer_get_u32(target, buf + 0);
		channel->buffer_addr = target_buffer_get_u32(target, buf + params->buffer_addr_offset);
	}
	channel->size = target_buffer_get_u32(target, buf + params->size_offset);
	channel->write_pos = target_buffer_get_u32(target, buf + params->write_pos_offset);
	channel->read_pos = target_buffer_get_u32(target, buf + params->read_pos_offset);
	channel->flags = target_buffer_get_u32(target, buf + params->flags_offset);
	return ERROR_OK;
}

int target_rtt_start(struct target *target, const struct rtt_control *ctrl,
		void *user_data)
{
	return ERROR_OK;
}

int target_rtt_stop(struct target *target, void *user_data)
{
	return ERROR_OK;
}

static int read_channel_name(struct target *target, target_addr_t address,
		char *name, size_t length)
{
	size_t offset;

	offset = 0;

	while (offset < length) {
		int ret;
		size_t read_length;

		read_length = MIN(32, length - offset);
		ret = target_read_buffer(target, address + offset, read_length,
			(uint8_t *)name + offset);

		if (ret != ERROR_OK)
			return ret;

		if (memchr(name + offset, '\0', read_length))
			return ERROR_OK;

		offset += read_length;
	}

	name[length - 1] = '\0';

	return ERROR_OK;
}

static int write_to_channel(struct target *target,
		const struct rtt_channel *channel, const uint8_t *buffer,
		size_t *length)
{
	int ret;
	uint32_t len;

	if (!*length)
		return ERROR_OK;

	if (channel->write_pos == channel->read_pos) {
		uint32_t first_length;

		len = MIN(*length, channel->size - 1);
		first_length = MIN(len, channel->size - channel->write_pos);

		ret = target_write_buffer(target,
			channel->buffer_addr + channel->write_pos, first_length,
			buffer);

		if (ret != ERROR_OK)
			return ret;

		ret = target_write_buffer(target, channel->buffer_addr,
			len - first_length, buffer + first_length);

		if (ret != ERROR_OK)
			return ret;
	} else if (channel->write_pos < channel->read_pos) {
		len = MIN(*length, channel->read_pos - channel->write_pos - 1);

		if (!len) {
			*length = 0;
			return ERROR_OK;
		}

		ret = target_write_buffer(target,
			channel->buffer_addr + channel->write_pos, len, buffer);

		if (ret != ERROR_OK)
			return ret;
	} else {
		uint32_t first_length;

		len = MIN(*length,
			channel->size - channel->write_pos + channel->read_pos - 1);

		if (!len) {
			*length = 0;
			return ERROR_OK;
		}

		first_length = MIN(len, channel->size - channel->write_pos);

		ret = target_write_buffer(target,
			channel->buffer_addr + channel->write_pos, first_length,
			buffer);

		if (ret != ERROR_OK)
			return ret;

		buffer = buffer + first_length;

		ret = target_write_buffer(target, channel->buffer_addr,
			len - first_length, buffer);

		if (ret != ERROR_OK)
			return ret;
	}

	const struct rtt_control_params *params = get_rtt_params(target);
	ret = target_write_u32(target, channel->address + params->write_pos_offset,
		(channel->write_pos + len) % channel->size);

	if (ret != ERROR_OK)
		return ret;

	*length = len;

	return ERROR_OK;
}

static bool channel_is_active(const struct rtt_channel *channel)
{
	if (!channel)
		return false;

	if (!channel->size)
		return false;

	return true;
}

int target_rtt_write_callback(struct target *target, struct rtt_control *ctrl,
		unsigned int channel_index, const uint8_t *buffer, size_t *length,
		void *user_data)
{
	int ret;
	struct rtt_channel channel;

	ret = read_rtt_channel(target, ctrl, channel_index,
		RTT_CHANNEL_TYPE_DOWN, &channel);

	if (ret != ERROR_OK) {
		LOG_ERROR("rtt: Failed to read down-channel %u description",
			channel_index);
		return ret;
	}

	if (!channel_is_active(&channel)) {
		LOG_WARNING("rtt: Down-channel %u is not active", channel_index);
		return ERROR_OK;
	}

	if (channel.size < RTT_CHANNEL_BUFFER_MIN_SIZE) {
		LOG_WARNING("rtt: Down-channel %u is not large enough",
			channel_index);
		return ERROR_OK;
	}

	ret = write_to_channel(target, &channel, buffer, length);

	if (ret != ERROR_OK)
		return ret;

	LOG_DEBUG("rtt: Wrote %zu bytes into down-channel %u", *length,
		channel_index);

	return ERROR_OK;
}

int target_rtt_read_control_block(struct target *target,
		target_addr_t address, struct rtt_control *ctrl, void *user_data)
{
	int ret;
	uint8_t buf[RTT_CB_SIZE];

	ret = target_read_buffer(target, address, RTT_CB_SIZE, buf);

	if (ret != ERROR_OK)
		return ret;

	memcpy(ctrl->id, buf, RTT_CB_MAX_ID_LENGTH);
	ctrl->id[RTT_CB_MAX_ID_LENGTH - 1] = '\0';
	ctrl->num_up_channels = target_buffer_get_u32(target, buf + RTT_CB_MAX_ID_LENGTH + 0);
	ctrl->num_down_channels = target_buffer_get_u32(target, buf + RTT_CB_MAX_ID_LENGTH + 4);

	return ERROR_OK;
}

int target_rtt_find_control_block(struct target *target,
		target_addr_t *address, size_t size, const char *id, bool *found,
		void *user_data)
{
	target_addr_t address_end = *address + size;
	uint8_t buf[1024];

	*found = false;

	size_t id_matched_length = 0;
	const size_t id_length = strlen(id);

	LOG_INFO("rtt: Searching for control block '%s'", id);

	for (target_addr_t addr = *address; addr < address_end; addr += sizeof(buf)) {
		int ret;

		const size_t buf_size = MIN(sizeof(buf), address_end - addr);
		ret = target_read_buffer(target, addr, buf_size, buf);

		if (ret != ERROR_OK)
			return ret;

		for (size_t buf_off = 0; buf_off < buf_size; buf_off++) {
			if (id_matched_length > 0 &&
			    buf[buf_off] != id[id_matched_length]) {
				/* Start from beginning */
				id_matched_length = 0;
			}

			if (buf[buf_off] == id[id_matched_length])
				id_matched_length++;

			if (id_matched_length == id_length) {
				*address = addr + buf_off + 1 - id_length;
				*found = true;
				return ERROR_OK;
			}
		}
	}

	return ERROR_OK;
}

int target_rtt_read_channel_info(struct target *target,
		const struct rtt_control *ctrl, unsigned int channel_index,
		enum rtt_channel_type type, struct rtt_channel_info *info,
		void *user_data)
{
	int ret;
	struct rtt_channel channel;

	ret = read_rtt_channel(target, ctrl, channel_index, type, &channel);

	if (ret != ERROR_OK) {
		LOG_ERROR("rtt: Failed to read channel %u description",
			channel_index);
		return ret;
	}

	info->size = channel.size;
	info->flags = channel.flags;

	if (!channel_is_active(&channel))
		return ERROR_OK;

	ret = read_channel_name(target, channel.name_addr, info->name,
		info->name_length);

	if (ret != ERROR_OK)
		return ret;

	return ERROR_OK;
}

static int read_from_channel(struct target *target,
		const struct rtt_channel *channel, uint8_t *buffer,
		size_t *length)
{
	int ret;
	uint32_t len;

	if (!*length)
		return ERROR_OK;

	if (channel->read_pos == channel->write_pos) {
		len = 0;
	} else if (channel->read_pos < channel->write_pos) {
		len = MIN(*length, channel->write_pos - channel->read_pos);

		ret = target_read_buffer(target,
			channel->buffer_addr + channel->read_pos, len, buffer);

		if (ret != ERROR_OK)
			return ret;
	} else {
		uint32_t first_length;

		len = MIN(*length,
			channel->size - channel->read_pos + channel->write_pos);
		first_length = MIN(len, channel->size - channel->read_pos);

		ret = target_read_buffer(target,
			channel->buffer_addr + channel->read_pos, first_length, buffer);

		if (ret != ERROR_OK)
			return ret;

		ret = target_read_buffer(target, channel->buffer_addr,
			len - first_length, buffer + first_length);

		if (ret != ERROR_OK)
			return ret;
	}

	if (len > 0) {
		const struct rtt_control_params *params = get_rtt_params(target);
		ret = target_write_u32(target, channel->address + params->read_pos_offset,
			(channel->read_pos + len) % channel->size);

		if (ret != ERROR_OK)
			return ret;
	}

	*length = len;

	return ERROR_OK;
}

int target_rtt_read_callback(struct target *target,
		const struct rtt_control *ctrl, struct rtt_sink_list **sinks,
		size_t num_channels, void *user_data)
{
	num_channels = MIN(num_channels, ctrl->num_up_channels);

	for (size_t i = 0; i < num_channels; i++) {
		int ret;
		struct rtt_channel channel;
		uint8_t buffer[1024];
		size_t length;

		if (!sinks[i])
			continue;

		ret = read_rtt_channel(target, ctrl, i, RTT_CHANNEL_TYPE_UP,
			&channel);

		if (ret != ERROR_OK) {
			LOG_ERROR("rtt: Failed to read up-channel %zu description", i);
			return ret;
		}

		if (!channel_is_active(&channel)) {
			LOG_WARNING("rtt: Up-channel %zu is not active", i);
			continue;
		}

		if (channel.size < RTT_CHANNEL_BUFFER_MIN_SIZE) {
			LOG_WARNING("rtt: Up-channel %zu is not large enough", i);
			continue;
		}

		length = sizeof(buffer);
		ret = read_from_channel(target, &channel, buffer, &length);

		if (ret != ERROR_OK) {
			LOG_ERROR("rtt: Failed to read from up-channel %zu", i);
			return ret;
		}

		for (struct rtt_sink_list *sink = sinks[i]; sink; sink = sink->next)
			sink->read(i, buffer, length, sink->user_data);
	}

	return ERROR_OK;
}
