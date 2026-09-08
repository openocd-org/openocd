/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2010 by Øyvind Harboe                                   *
 ***************************************************************************/

#ifndef OPENOCD_HELPER_UTIL_H
#define OPENOCD_HELPER_UTIL_H

#include "types.h"

struct command_context;

int util_init(struct command_context *cmd_ctx);
bool is_memory_regions_overlap(target_addr_t start1, unsigned int size1,
		target_addr_t start2, unsigned int size2);

#endif /* OPENOCD_HELPER_UTIL_H */
