/***************************************************************************
 *   Copyright (C) 2007, 2008 by Ben Dooks                                 *
 *   ben@fluff.org                                                         *
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

/*
 * S3C2410 OpenOCD NAND Flash controller support.
 *
 * Many thanks to Simtec Electronics for sponsoring this work.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "s3c24xx.h"

NAND_DEVICE_COMMAND_HANDLER(s3c2410_nand_device_command)
{
	struct s3c24xx_nand_controller *info;
	CALL_S3C24XX_DEVICE_COMMAND(nand, &info);

	/* fill in the address fields for the core device */
	info->cmd = S3C2410_NFCMD;
	info->addr = S3C2410_NFADDR;
	info->data = S3C2410_NFDATA;
	info->nfstat = S3C2410_NFSTAT;

	return ERROR_OK;
}

static int s3c2410_init(struct nand_device *nand)
{
	struct target *target = nand->target;

	target_write_u32(target, S3C2410_NFCONF,
			 S3C2410_NFCONF_EN | S3C2410_NFCONF_TACLS(3) |
			 S3C2410_NFCONF_TWRPH0(5) | S3C2410_NFCONF_TWRPH1(3));

	return ERROR_OK;
}

static int s3c2410_write_data(struct nand_device *nand, uint16_t data)
{
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use S3C24XX NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	target_write_u32(target, S3C2410_NFDATA, data);
	return ERROR_OK;
}

static int s3c2410_read_data(struct nand_device *nand, void *data)
{
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use S3C24XX NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	target_read_u8(target, S3C2410_NFDATA, data);
	return ERROR_OK;
}

static int s3c2410_nand_ready(struct nand_device *nand, int timeout)
{
	struct target *target = nand->target;
	uint8_t status;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use S3C24XX NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	do {
		target_read_u8(target, S3C2410_NFSTAT, &status);

		if (status & S3C2410_NFSTAT_BUSY)
			return 1;

		alive_sleep(1);
	} while (timeout-- > 0);

	return 0;
}

struct nand_flash_controller s3c2410_nand_controller = {
	.name = "s3c2410",
	.nand_device_command = &s3c2410_nand_device_command,
	.init = &s3c2410_init,
	.reset = &s3c24xx_reset,
	.command = &s3c24xx_command,
	.address = &s3c24xx_address,
	.write_data = &s3c2410_write_data,
	.read_data = &s3c2410_read_data,
	.write_page = s3c24xx_write_page,
	.read_page = s3c24xx_read_page,
	.nand_ready = &s3c2410_nand_ready,
};
