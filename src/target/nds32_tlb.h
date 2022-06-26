/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2013 Andes Technology                                   *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_NDS32_TLB_H
#define OPENOCD_TARGET_NDS32_TLB_H

#include "nds32.h"

enum {
	PAGE_SIZE_4K = 0,
	PAGE_SIZE_8K,
	PAGE_SIZE_NUM,
};

struct page_table_walker_info_s {

	uint32_t l1_offset_mask;
	uint32_t l1_offset_shift;
	uint32_t l2_offset_mask;
	uint32_t l2_offset_shift;
	uint32_t va_offset_mask;
	uint32_t l1_base_mask;
	uint32_t l2_base_mask;
	uint32_t ppn_mask;
};

extern int nds32_probe_tlb(struct nds32 *nds32, const target_addr_t virtual_address,
		target_addr_t *physical_address);
extern int nds32_walk_page_table(struct nds32 *nds32, const target_addr_t virtual_address,
		target_addr_t *physical_address);

#endif /* OPENOCD_TARGET_NDS32_TLB_H */
