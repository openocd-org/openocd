// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2007, 2008 by Ben Dooks                                 *
 *   ben@fluff.org                                                         *
 ***************************************************************************/

/*
 * S3C24XX Series OpenOCD NAND Flash controller support.
 *
 * Many thanks to Simtec Electronics for sponsoring this work.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "s3c24xx.h"

S3C24XX_DEVICE_COMMAND()
{
	*info = NULL;

	struct s3c24xx_nand_controller *s3c24xx_info;
	s3c24xx_info = malloc(sizeof(struct s3c24xx_nand_controller));
	if (!s3c24xx_info) {
		LOG_ERROR("no memory for nand controller");
		return -ENOMEM;
	}

	nand->controller_priv = s3c24xx_info;
	*info = s3c24xx_info;

	return ERROR_OK;
}

int s3c24xx_reset(struct nand_device *nand)
{
	struct s3c24xx_nand_controller *s3c24xx_info = nand->controller_priv;
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use S3C24XX NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	target_write_u32(target, s3c24xx_info->cmd, 0xff);

	return ERROR_OK;
}

int s3c24xx_command(struct nand_device *nand, uint8_t command)
{
	struct s3c24xx_nand_controller *s3c24xx_info = nand->controller_priv;
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use S3C24XX NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	target_write_u16(target, s3c24xx_info->cmd, command);
	return ERROR_OK;
}

int s3c24xx_address(struct nand_device *nand, uint8_t address)
{
	struct s3c24xx_nand_controller *s3c24xx_info = nand->controller_priv;
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use S3C24XX NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	target_write_u16(target, s3c24xx_info->addr, address);
	return ERROR_OK;
}

int s3c24xx_write_data(struct nand_device *nand, uint16_t data)
{
	struct s3c24xx_nand_controller *s3c24xx_info = nand->controller_priv;
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use S3C24XX NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	target_write_u8(target, s3c24xx_info->data, data);
	return ERROR_OK;
}

int s3c24xx_read_data(struct nand_device *nand, void *data)
{
	struct s3c24xx_nand_controller *s3c24xx_info = nand->controller_priv;
	struct target *target = nand->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use S3C24XX NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	target_read_u8(target, s3c24xx_info->data, data);
	return ERROR_OK;
}
