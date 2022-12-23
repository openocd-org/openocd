/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2007, 2008 by Ben Dooks                                 *
 *   ben@fluff.org                                                         *
 ***************************************************************************/

#ifndef OPENOCD_FLASH_NAND_S3C24XX_H
#define OPENOCD_FLASH_NAND_S3C24XX_H

/*
 * S3C24XX Series OpenOCD NAND Flash controller support.
 *
 * Many thanks to Simtec Electronics for sponsoring this work.
 */

#include "imp.h"
#include "s3c24xx_regs.h"
#include <target/target.h>

struct s3c24xx_nand_controller {
	/* register addresses */
	uint32_t		 cmd;
	uint32_t		 addr;
	uint32_t		 data;
	uint32_t		 nfstat;
};

/* Default to using the un-translated NAND register based address */
#undef S3C2410_NFREG
#define S3C2410_NFREG(x) ((x) + 0x4e000000)

#define S3C24XX_DEVICE_COMMAND() \
		COMMAND_HELPER(s3c24xx_nand_device_command, \
				struct nand_device *nand, \
				struct s3c24xx_nand_controller **info)

S3C24XX_DEVICE_COMMAND();

#define CALL_S3C24XX_DEVICE_COMMAND(d, i) \
	do { \
		int retval = CALL_COMMAND_HANDLER(s3c24xx_nand_device_command, d, i); \
		if (retval != ERROR_OK) \
			return retval; \
	} while (0)

int s3c24xx_reset(struct nand_device *nand);

int s3c24xx_command(struct nand_device *nand, uint8_t command);
int s3c24xx_address(struct nand_device *nand, uint8_t address);

int s3c24xx_write_data(struct nand_device *nand, uint16_t data);
int s3c24xx_read_data(struct nand_device *nand, void *data);

#define s3c24xx_write_page NULL
#define s3c24xx_read_page NULL

/* code shared between different controllers */

int s3c2440_nand_ready(struct nand_device *nand, int timeout);

int s3c2440_read_block_data(struct nand_device *nand,
		uint8_t *data, int data_size);
int s3c2440_write_block_data(struct nand_device *nand,
		uint8_t *data, int data_size);

#endif /* OPENOCD_FLASH_NAND_S3C24XX_H */
