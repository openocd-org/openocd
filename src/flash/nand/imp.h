/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
 ***************************************************************************/

#ifndef OPENOCD_FLASH_NAND_IMP_H
#define OPENOCD_FLASH_NAND_IMP_H

#include "core.h"
#include "driver.h"

void nand_device_add(struct nand_device *c);

int nand_write_page(struct nand_device *nand,
		uint32_t page, uint8_t *data, uint32_t data_size,
		uint8_t *oob, uint32_t oob_size);

int nand_read_page(struct nand_device *nand, uint32_t page,
		uint8_t *data, uint32_t data_size,
		uint8_t *oob, uint32_t oob_size);

int nand_probe(struct nand_device *nand);
int nand_erase(struct nand_device *nand, int first_block, int last_block);
int nand_build_bbt(struct nand_device *nand, int first, int last);

#endif /* OPENOCD_FLASH_NAND_IMP_H */
