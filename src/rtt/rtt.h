/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Copyright (C) 2016-2020 by Marc Schink <dev@zapb.de>
 */

#ifndef OPENOCD_RTT_RTT_H
#define OPENOCD_RTT_RTT_H

#include <stdint.h>
#include <stdbool.h>

#include <helper/command.h>
#include <target/target.h>

/**
 * Control block ID length in bytes, including the trailing null-terminator.
 */
#define RTT_CB_MAX_ID_LENGTH	16

/* Control block size in bytes. */
#define RTT_CB_SIZE		(RTT_CB_MAX_ID_LENGTH + 2 * sizeof(uint32_t))

/* Channel structure size in bytes. */
#define RTT_CHANNEL_SIZE	24

/* Minimal channel buffer size in bytes. */
#define RTT_CHANNEL_BUFFER_MIN_SIZE	2

/** RTT control block. */
struct rtt_control {
	/** Control block address on the target. */
	target_addr_t address;
	/** Control block identifier, including trailing null-terminator. */
	char id[RTT_CB_MAX_ID_LENGTH];
	/** Maximum number of up-channels. */
	uint32_t num_up_channels;
	/** Maximum number of down-channels. */
	uint32_t num_down_channels;
};

/** RTT channel. */
struct rtt_channel {
	/** Channel structure address on the target. */
	target_addr_t address;
	/** Channel name address on the target. */
	uint32_t name_addr;
	/** Buffer address on the target. */
	uint32_t buffer_addr;
	/** Channel buffer size in bytes. */
	uint32_t size;
	/**  Write position within the buffer in bytes. */
	uint32_t write_pos;
	/** Read position within the buffer in bytes. */
	uint32_t read_pos;
	/**
	 * Buffer flags.
	 *
	 * @note: Not used at the moment.
	 */
	uint32_t flags;
};

/** RTT channel information. */
struct rtt_channel_info {
	/** Channel name. */
	char *name;
	/** Length of the name in bytes, including the trailing null-terminator. */
	size_t name_length;
	/** Buffer size in bytes. */
	uint32_t size;
	/**
	 * Buffer flags.
	 *
	 * @note: Not used at the moment.
	 */
	uint32_t flags;
};

typedef int (*rtt_sink_read)(unsigned int channel, const uint8_t *buffer,
		size_t length, void *user_data);

struct rtt_sink_list {
	rtt_sink_read read;
	void *user_data;

	struct rtt_sink_list *next;
};

/** Channel type. */
enum rtt_channel_type {
	/** Up channel (target to host). */
	RTT_CHANNEL_TYPE_UP,
	/** Down channel (host to target). */
	RTT_CHANNEL_TYPE_DOWN
};

/** RTT source. */
struct rtt_source {
	int (*find_cb)(struct target *target,
		target_addr_t *address, size_t size, const char *id, bool *found,
		void *user_data);
	int (*read_cb)(struct target *target,
		target_addr_t address, struct rtt_control *ctrl_block,
		void *user_data);
	int (*read_channel_info)(struct target *target,
		const struct rtt_control *ctrl, unsigned int channel,
		enum rtt_channel_type type, struct rtt_channel_info *info,
		void *user_data);
	int (*start)(struct target *target,
		const struct rtt_control *ctrl, void *user_data);
	int (*stop)(struct target *target, void *user_data);
	int (*read)(struct target *target,
		const struct rtt_control *ctrl, struct rtt_sink_list **sinks,
		size_t num_channels, void *user_data);
	int (*write)(struct target *target,
		struct rtt_control *ctrl, unsigned int channel,
		const uint8_t *buffer, size_t *length, void *user_data);
};

/**
 * Initialize Real-Time Transfer (RTT).
 *
 * @returns ERROR_OK on success, an error code on failure.
 */
int rtt_init(void);

/**
 * Shutdown Real-Time Transfer (RTT).
 *
 * @returns ERROR_OK on success, an error code on failure.
 */
int rtt_exit(void);

/**
 * Register an RTT source for a target.
 *
 * @param[in] source RTT source.
 * @param[in,out] target Target.
 *
 * @returns ERROR_OK on success, an error code on failure.
 */
int rtt_register_source(const struct rtt_source source,
		struct target *target);

/**
 * Setup RTT.
 *
 * @param[in] address Start address to search for the control block.
 * @param[in] size Size of the control block search area.
 * @param[in] id Identifier of the control block. Must be null-terminated.
 *
 * @returns ERROR_OK on success, an error code on failure.
 */
int rtt_setup(target_addr_t address, size_t size, const char *id);

/**
 * Start Real-Time Transfer (RTT).
 *
 * @returns ERROR_OK on success, an error code on failure.
 */
int rtt_start(void);

/**
 * Stop Real-Time Transfer (RTT).
 *
 * @returns ERROR_OK on success, an error code on failure.
 */
int rtt_stop(void);

/**
 * Get the polling interval.
 *
 * @param[out] interval Polling interval in milliseconds.
 *
 * @returns ERROR_OK on success, an error code on failure.
 */
int rtt_get_polling_interval(unsigned int *interval);

/**
 * Set the polling interval.
 *
 * @param[in] interval Polling interval in milliseconds.
 *
 * @returns ERROR_OK on success, an error code on failure.
 */
int rtt_set_polling_interval(unsigned int interval);

/**
 * Get whether RTT is configured.
 *
 * @returns Whether RTT is configured.
 */
bool rtt_configured(void);

/**
 * Get whether RTT control block was found.
 *
 * @returns Whether RTT was found.
 */
bool rtt_found_cb(void);

/**
 * Get the RTT control block.
 *
 * @returns The RTT control block.
 */
const struct rtt_control *rtt_get_control(void);

/**
 * Read channel information.
 *
 * @param[in] channel_index Channel index.
 * @param[in] type Channel type.
 * @param[out] info Channel information.
 *
 * @returns ERROR_OK on success, an error code on failure.
 */
int rtt_read_channel_info(unsigned int channel_index,
	enum rtt_channel_type type, struct rtt_channel_info *info);

/**
 * Register an RTT sink.
 *
 * @param[in] channel_index Channel index.
 * @param[in] read Read callback function.
 * @param[in,out] user_data User data to be passed to the callback function.
 *
 * @returns ERROR_OK on success, an error code on failure.
 */
int rtt_register_sink(unsigned int channel_index, rtt_sink_read read,
		void *user_data);

/**
 * Unregister an RTT sink.
 *
 * @param[in] channel_index Channel index.
 * @param[in] read Read callback function.
 * @param[in,out] user_data User data to be passed to the callback function.
 *
 * @returns ERROR_OK on success, an error code on failure.
 */
int rtt_unregister_sink(unsigned int channel_index, rtt_sink_read read,
		void *user_data);

/**
 * Write to an RTT channel.
 *
 * @param[in] channel_index Channel index.
 * @param[in] buffer Buffer with data that should be written to the channel.
 * @param[in,out] length Number of bytes to write. On success, the argument gets
 *                       updated with the actual number of written bytes.
 *
 * @returns ERROR_OK on success, an error code on failure.
 */
int rtt_write_channel(unsigned int channel_index, const uint8_t *buffer,
		size_t *length);

extern const struct command_registration rtt_target_command_handlers[];

#endif /* OPENOCD_RTT_RTT_H */
