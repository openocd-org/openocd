/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2017 by Texas Instruments, Inc.                         *
 ***************************************************************************/

#ifndef OPENOCD_FLASH_NOR_CC26XX_H
#define OPENOCD_FLASH_NOR_CC26XX_H

/* Addresses of FCFG1 registers to access ICEPick Device ID and User ID */
#define FCFG1_ICEPICK_ID 0x50001318
#define FCFG1_USER_ID    0x50001294

/* ICEPick device ID mask and values */
#define ICEPICK_ID_MASK          0x0fffffff
#define ICEPICK_REV_MASK         0xf0000000
#define CC26X0_ICEPICK_ID        0x0b99a02f
#define CC26X1_ICEPICK_ID        0x0b9bd02f
#define CC13X0_ICEPICK_ID        0x0b9be02f
#define CC13X2_CC26X2_ICEPICK_ID 0x0bb4102f

/* User ID mask for Agama CC13x2 vs CC26x2 */
#define USER_ID_CC13_MASK 0x00800000

/* Common CC26xx/CC13xx flash and memory parameters */
#define CC26XX_FLASH_BASE_ADDR   0x00000000
#define CC26XX_FLASH_SIZE_INFO   0x4003002c
#define CC26XX_SRAM_SIZE_INFO    0x40082250
#define CC26XX_ALGO_BASE_ADDRESS 0x20000000

/* Chameleon CC26x0/CC13x0 specific parameters */
#define CC26X0_MAX_SECTORS   32
#define CC26X0_SECTOR_LENGTH 0x1000
#define CC26X0_ALGO_BUFFER_0 0x20001c00
#define CC26X0_ALGO_BUFFER_1 0x20002c00
#define CC26X0_ALGO_PARAMS_0 0x20001bd8
#define CC26X0_ALGO_PARAMS_1 0x20001bec
#define CC26X0_WORKING_SIZE  (CC26X0_ALGO_BUFFER_1 + CC26X0_SECTOR_LENGTH - \
							 CC26XX_ALGO_BASE_ADDRESS)

/* Agama CC26x2/CC13x2 specific parameters */
#define CC26X2_MAX_SECTORS   128
#define CC26X2_SECTOR_LENGTH 0x2000
#define CC26X2_ALGO_BUFFER_0 0x20002000
#define CC26X2_ALGO_BUFFER_1 0x20004000
#define CC26X2_ALGO_PARAMS_0 0x20001fd8
#define CC26X2_ALGO_PARAMS_1 0x20001fec
#define CC26X2_WORKING_SIZE  (CC26X2_ALGO_BUFFER_1 + CC26X2_SECTOR_LENGTH - \
							 CC26XX_ALGO_BASE_ADDRESS)

/* CC26xx flash helper algorithm buffer flags */
#define CC26XX_BUFFER_EMPTY 0x00000000
#define CC26XX_BUFFER_FULL  0xffffffff

/* CC26XX flash helper algorithm commands */
#define CC26XX_CMD_NO_ACTION                     0
#define CC26XX_CMD_ERASE_ALL                     1
#define CC26XX_CMD_PROGRAM                       2
#define CC26XX_CMD_ERASE_AND_PROGRAM             3
#define CC26XX_CMD_ERASE_AND_PROGRAM_WITH_RETAIN 4
#define CC26XX_CMD_ERASE_SECTORS                 5

/* CC26xx and CC13xx device types */
#define CC26XX_NO_TYPE 0 /* Device type not determined yet */
#define CC26X0_TYPE    1 /* CC26x0 Chameleon device */
#define CC26X1_TYPE    2 /* CC26x1 Chameleon device */
#define CC26X2_TYPE    3 /* CC26x2 Agama device */
#define CC13X0_TYPE    4 /* CC13x0 Chameleon device */
#define CC13X2_TYPE    5 /* CC13x2 Agama device */

/* Flash helper algorithm parameter block struct */
#define CC26XX_STATUS_OFFSET 0x0c
struct cc26xx_algo_params {
	uint8_t address[4];
	uint8_t length[4];
	uint8_t command[4];
	uint8_t status[4];
};

#endif /* OPENOCD_FLASH_NOR_CC26XX_H */
