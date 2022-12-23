/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Copyright (C) 2016-2020 by Marc Schink <dev@zapb.de>
 */

#ifndef OPENOCD_TARGET_RTT_H
#define OPENOCD_TARGET_RTT_H

#include <stdint.h>
#include <stdbool.h>

#include <target/target.h>
#include <rtt/rtt.h>

int target_rtt_start(struct target *target, const struct rtt_control *ctrl,
		void *user_data);
int target_rtt_stop(struct target *target, void *user_data);
int target_rtt_find_control_block(struct target *target,
		target_addr_t *address, size_t size, const char *id, bool *found,
		void *user_data);
int target_rtt_read_control_block(struct target *target,
		target_addr_t address, struct rtt_control *ctrl, void *user_data);
int target_rtt_write_callback(struct target *target,
		struct rtt_control *ctrl, unsigned int channel_index,
		const uint8_t *buffer, size_t *length, void *user_data);
int target_rtt_read_callback(struct target *target,
		const struct rtt_control *ctrl, struct rtt_sink_list **sinks,
		size_t length, void *user_data);
int target_rtt_read_channel_info(struct target *target,
		const struct rtt_control *ctrl, unsigned int channel_index,
		enum rtt_channel_type type, struct rtt_channel_info *info,
		void *user_data);

#endif /* OPENOCD_TARGET_RTT_H */
