/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2017 by Texas Instruments, Inc.                         *
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

#endif /* OPENOCD_FLASH_NOR_CC3220SF_H */
