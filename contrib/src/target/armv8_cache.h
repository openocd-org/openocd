/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2016 by Matthias Welwarsky                              *
 *   matthias.welwarsky@sysgo.com                                          *
 ***************************************************************************/
#ifndef OPENOCD_TARGET_ARMV8_CACHE_H_
#define OPENOCD_TARGET_ARMV8_CACHE_H_

#include "armv8.h"

extern int armv8_cache_d_inner_flush_virt(struct armv8_common *armv8, target_addr_t va, size_t size);
extern int armv8_cache_i_inner_inval_virt(struct armv8_common *armv8, target_addr_t va, size_t size);

#endif /* OPENOCD_TARGET_ARMV8_CACHE_H_ */
