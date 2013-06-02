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
 * S3C2440 OpenOCD NAND Flash controller support.
 *
 * Many thanks to Simtec Electronics for sponsoring this work.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "s3c24xx.h"

NAND_DEVICE_COMMAND_HANDLER(s3c2440_nand_device_command)
{
	struct s3c24xx_nand_controller *info;
	CALL_S3C24XX_DEVICE_COMMAND(nand, &info);

	/* fill in the address fields for the core device */
	info->cmd = S3C2440_NFCMD;
	info->addr = S3C2440_NFADDR;
	info->data = S3C2440_NFDATA;
	info->nfstat = S3C2440_NFSTAT;

	return ERROR_OK;
}

static int s3c2440_init(struct nand_device *nand)
{
	struct target *target = nand->target;

	target_write_u32(target, S3C2410_NFCONF,
			 S3C2440_NFCONF_TACLS(3) |
			 S3C2440_NFCONF_TWRPH0(7) |
			 S3C2440_NFCONF_TWRPH1(7));

	target_write_u32(target, S3C2440_NFCONT,
			 S3C2440_NFCONT_INITECC | S3C2440_NFCONT_ENABLE);

	return ERROR_OK;
}

int s3c2440_nand_ready(struct nand_device *nand, int timeout)
{
	struct s3c24xx_nand_controller *s3c24xx_info = nand->controller_priv;
	struct target *target = nand->target;
	uint8_t status;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use S3C24XX NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	do {
		target_read_u8(target, s3c24xx_info->nfstat, &status);

		if (status & S3C2440_NFSTAT_READY)
			return 1;

		alive_sleep(1);
	} while (timeout-- > 0);


	return 0;
}

/* use the fact we can read/write 4 bytes in one go via a single 32bit op */

int s3c2440_read_block_data(struct nand_device *nand, uint8_t *data, int data_size)
{
	struct s3c24xx_nand_controller *s3c24xx_info = nand->controller_priv;
	struct target *target = nand->target;
	uint32_t nfdata = s3c24xx_info->data;
	uint32_t tmp;

	LOG_INFO("%s: reading data: %p, %p, %d", __func__, nand, data, data_size);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use S3C24XX NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	while (data_size >= 4) {
		target_read_u32(target, nfdata, &tmp);

		data[0] = tmp;
		data[1] = tmp >> 8;
		data[2] = tmp >> 16;
		data[3] = tmp >> 24;

		data_size -= 4;
		data += 4;
	}

	while (data_size > 0) {
		target_read_u8(target, nfdata, data);

		data_size -= 1;
		data += 1;
	}

	return ERROR_OK;
}

int s3c2440_write_block_data(struct nand_device *nand, uint8_t *data, int data_size)
{
	struct s3c24xx_nand_controller *s3c24xx_info = nand->controller_priv;
	struct target *target = nand->target;
	uint32_t nfdata = s3c24xx_info->data;
	uint32_t tmp;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use S3C24XX NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	while (data_size >= 4) {
		tmp = le_to_h_u32(data);
		target_write_u32(target, nfdata, tmp);

		data_size -= 4;
		data += 4;
	}

	while (data_size > 0) {
		target_write_u8(target, nfdata, *data);

		data_size -= 1;
		data += 1;
	}

	return ERROR_OK;
}

struct nand_flash_controller s3c2440_nand_controller = {
	.name = "s3c2440",
	.nand_device_command = &s3c2440_nand_device_command,
	.init = &s3c2440_init,
	.reset = &s3c24xx_reset,
	.command = &s3c24xx_command,
	.address = &s3c24xx_address,
	.write_data = &s3c24xx_write_data,
	.read_data = &s3c24xx_read_data,
	.write_page = s3c24xx_write_page,
	.read_page = s3c24xx_read_page,
	.write_block_data = &s3c2440_write_block_data,
	.read_block_data = &s3c2440_read_block_data,
	.nand_ready = &s3c2440_nand_ready,
};
