/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2013 Andes Technology                                   *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_NDS32_V2_H
#define OPENOCD_TARGET_NDS32_V2_H

#include "nds32.h"

struct nds32_v2_common {
	struct nds32 nds32;

	uint32_t backup_ir0;

	/** number of hardware breakpoints */
	int32_t n_hbr;

	/** next hardware breakpoint index */
	/** increase from low index to high index */
	int32_t next_hbr_index;
};

static inline struct nds32_v2_common *target_to_nds32_v2(struct target *target)
{
	return container_of(target->arch_info, struct nds32_v2_common, nds32);
}

#endif /* OPENOCD_TARGET_NDS32_V2_H */
