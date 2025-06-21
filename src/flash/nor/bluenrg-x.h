/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2019 by STMicroelectronics.                             *
 ***************************************************************************/

#ifndef OPENOCD_FLASH_NOR_BLUENRGX_H
#define OPENOCD_FLASH_NOR_BLUENRGX_H

/* Flash Controller registers offsets */
#define FLASH_REG_COMMAND 0x00
#define FLASH_REG_CONFIG  0x04
#define FLASH_REG_IRQSTAT 0x08
#define FLASH_REG_IRQMASK 0x0C
#define FLASH_REG_IRQRAW  0x10
#define FLASH_REG_ADDRESS 0x18
#define FLASH_REG_UNLOCKM 0x1C
#define FLASH_REG_UNLOCKL 0x20
#define FLASH_REG_DATA0   0x40
#define FLASH_REG_DATA1   0x44
#define FLASH_REG_DATA2   0x48
#define FLASH_REG_DATA3   0x4C
#define FLASH_SIZE_REG    0x14

/* Flash Controller commands */
#define FLASH_CMD_ERASE_PAGE 0x11
#define FLASH_CMD_MASSERASE  0x22
#define FLASH_CMD_WRITE      0x33
#define FLASH_CMD_BURSTWRITE 0xCC
#define FLASH_INT_CMDDONE    0x01
#define FLASH_INT_CMDSTART   0x02

/* Flash Controller constants */
#define FLASH_WORD_LEN       4
#define FLASH_DATA_WIDTH_W   4
#define FLASH_DATA_WIDTH     16


#endif /* OPENOCD_FLASH_NOR_BLUENRGX_H */
