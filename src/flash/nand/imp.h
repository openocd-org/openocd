/***************************************************************************
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
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

#ifndef FLASH_NAND_IMP_H
#define FLASH_NAND_IMP_H

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

#endif	/* FLASH_NAND_IMP_H */
