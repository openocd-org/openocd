/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 ***************************************************************************/

#ifndef OPENOCD_FLASH_NOR_NON_CFI_H
#define OPENOCD_FLASH_NOR_NON_CFI_H

struct non_cfi {
	uint16_t mfr;
	uint16_t id;
	uint16_t pri_id;
	uint32_t dev_size;
	uint16_t interface_desc;
	uint16_t max_buf_write_size;
	uint8_t num_erase_regions;
	uint32_t erase_region_info[6];
	uint8_t  status_poll_mask;
};

void cfi_fixup_non_cfi(struct flash_bank *bank);

#endif /* OPENOCD_FLASH_NOR_NON_CFI_H */
