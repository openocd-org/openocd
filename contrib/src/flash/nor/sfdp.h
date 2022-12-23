/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2019 by Andreas Bolsch <andreas.bolsch@mni.thm.de	   *
 ***************************************************************************/

#ifndef OPENOCD_FLASH_NOR_SFDP_H
#define OPENOCD_FLASH_NOR_SFDP_H

/* per JESD216D 'addr' is *byte* based but must be word aligned,
 * 'buffer' is word based, word aligned and always little-endian encoded,
 * in the flash, 'addr_len' is 3 or 4, 'dummy' ***usually*** 8
 *
 * the actual number of dummy clocks should be worked out by this function
 * dynamically, i.e. by scanning the first few bytes for the SFDP signature
 *
 * buffer contents is supposed to be returned in ***host*** endianness */
typedef int (*read_sfdp_block_t)(struct flash_bank *bank, uint32_t addr,
	uint32_t words, uint32_t *buffer);

extern int spi_sfdp(struct flash_bank *bank, struct flash_device *dev,
	read_sfdp_block_t read_sfdp_block);

#endif /* OPENOCD_FLASH_NOR_SFDP_H */
