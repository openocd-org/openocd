/***************************************************************************
 *   Copyright (C) 2013 Andes Technology                                   *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
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
