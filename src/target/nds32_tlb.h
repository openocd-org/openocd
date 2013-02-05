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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/
#ifndef __NDS32_TLB_H__
#define __NDS32_TLB_H__

#include "nds32.h"

enum {
	PAGE_SIZE_4K = 0,
	PAGE_SIZE_8K,
	PAGE_SIZE_NUM,
};

struct page_table_walker_info_s {

	uint32_t L1_offset_mask;
	uint32_t L1_offset_shift;
	uint32_t L2_offset_mask;
	uint32_t L2_offset_shift;
	uint32_t va_offset_mask;
	uint32_t L1_base_mask;
	uint32_t L2_base_mask;
	uint32_t ppn_mask;
};

extern int nds32_probe_tlb(struct nds32 *nds32, const uint32_t virtual_address,
		uint32_t *physical_address);
extern int nds32_walk_page_table(struct nds32 *nds32, const uint32_t virtual_address,
		uint32_t *physical_address);

#endif
