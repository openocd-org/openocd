/***************************************************************************
 *   Copyright (C) 2017 by Texas Instruments, Inc.                         *
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

#ifndef OPENOCD_FLASH_NOR_CC3220SF_H
#define OPENOCD_FLASH_NOR_CC3220SF_H

/* CC3220SF device types */
#define CC3220_NO_TYPE 0 /* Device type not determined yet */
#define CC3220_OTHER   1 /* CC3220 variant without flash */
#define CC3220SF       2 /* CC3220SF variant with flash */

/* Flash parameters */
#define FLASH_BASE_ADDR   0x01000000
#define FLASH_SECTOR_SIZE 2048
#define FLASH_NUM_SECTORS 512

/* CC2200SF flash registers */
#define FMA_REGISTER_ADDR 0x400FD000
#define FMC_REGISTER_ADDR 0x400FD008
#define FMC_DEFAULT_VALUE 0xA4420000
#define FMC_ERASE_BIT     0x00000002
#define FMC_MERASE_BIT    0x00000004
#define FMC_ERASE_VALUE   (FMC_DEFAULT_VALUE | FMC_ERASE_BIT)
#define FMC_MERASE_VALUE  (FMC_DEFAULT_VALUE | FMC_MERASE_BIT)

/* Flash helper algorithm for CC3220SF */
const uint8_t cc3220sf_algo[] = {
#include "../../../contrib/loaders/flash/cc3220sf/cc3220sf.inc"
};

#endif /* OPENOCD_FLASH_NOR_CC3220SF_H */
