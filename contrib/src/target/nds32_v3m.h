/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2013 Andes Technology                                   *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_NDS32_V3M_H
#define OPENOCD_TARGET_NDS32_V3M_H

#include "nds32.h"

struct nds32_v3m_common {
	struct nds32 nds32;

	/** number of hardware breakpoints */
	int32_t n_hbr;

	/** number of hardware watchpoints */
	int32_t n_hwp;

	/** number of used hardware watchpoints */
	int32_t used_n_wp;

	/** next hardware breakpoint index */
	/** for simple breakpoints, hardware breakpoints are inserted
	 * from high index to low index */
	int32_t next_hbr_index;

	/** next hardware watchpoint index */
	/** increase from low index to high index */
	int32_t next_hwp_index;
};

static inline struct nds32_v3m_common *target_to_nds32_v3m(struct target *target)
{
	return container_of(target->arch_info, struct nds32_v3m_common, nds32);
}

#endif /* OPENOCD_TARGET_NDS32_V3M_H */
