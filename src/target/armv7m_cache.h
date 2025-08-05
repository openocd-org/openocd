/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * Copyright (C) 2025 by STMicroelectronics
 * Copyright (C) 2025 by Antonio Borneo <borneo.antonio@gmail.com>
 */

#ifndef OPENOCD_TARGET_ARMV7M_CACHE_H
#define OPENOCD_TARGET_ARMV7M_CACHE_H

#include <stdbool.h>
#include <stdint.h>

#include <helper/types.h>

struct target;

struct armv7m_cache_size {
	// cache dimensioning
	uint32_t line_len;
	uint32_t associativity;
	uint32_t num_sets;
	uint32_t cache_size;
	// info for set way operation on cache
	uint32_t index;
	uint32_t index_shift;
	uint32_t way;
	uint32_t way_shift;
};

// information about one architecture cache at any level
struct armv7m_arch_cache {
	unsigned int ctype;					// cache type, CLIDR encoding
	struct armv7m_cache_size d_u_size;	// data cache
	struct armv7m_cache_size i_size;	// instruction cache
};

// common cache information
struct armv7m_cache_common {
	bool info_valid;
	bool has_i_cache;
	bool has_d_u_cache;
	unsigned int loc;					// level of coherency
	uint32_t d_min_line_len;			// minimum d-cache line_len
	uint32_t i_min_line_len;			// minimum i-cache line_len
	struct armv7m_arch_cache arch[6];	// cache info, L1 - L7
};

int armv7m_identify_cache(struct target *target);
int armv7m_d_cache_flush(struct target *target, uint32_t address,
	unsigned int length);
int armv7m_i_cache_inval(struct target *target, uint32_t address,
	unsigned int length);
int armv7m_handle_cache_info_command(struct command_invocation *cmd,
	struct target *target);

#endif /* OPENOCD_TARGET_ARMV7M_CACHE_H */
