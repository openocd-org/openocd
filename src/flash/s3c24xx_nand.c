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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

/*
 * S3C24XX Series OpenOCD NAND Flash controller support.
 *
 * Many thanks to Simtec Electronics for sponsoring this work.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "replacements.h"
#include "log.h"

#include <stdlib.h>
#include <string.h>

#include "nand.h"
#include "s3c24xx_nand.h"
#include "target.h"

s3c24xx_nand_controller_t *
s3c24xx_nand_device_command(struct command_context_s *cmd_ctx, char *cmd,
			    char **args, int argc,
			    struct nand_device_s *device)
{
	s3c24xx_nand_controller_t *s3c24xx_info;
	
	s3c24xx_info = malloc(sizeof(s3c24xx_nand_controller_t));
	if (s3c24xx_info == NULL) {
		LOG_ERROR("no memory for nand controller\n");
		return NULL;
	}

	device->controller_priv = s3c24xx_info;

	s3c24xx_info->target = get_target_by_num(strtoul(args[1], NULL, 0));
	if (s3c24xx_info->target == NULL) {
		LOG_ERROR("no target '%s' configured", args[1]);
		return NULL;
	}
		
	return s3c24xx_info;
}

int s3c24xx_register_commands(struct command_context_s *cmd_ctx)
{
	return ERROR_OK;
}

int s3c24xx_reset(struct nand_device_s *device)
{
	s3c24xx_nand_controller_t *s3c24xx_info = device->controller_priv;
	target_t *target = s3c24xx_info->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use S3C24XX NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}
	
	target_write_u32(target, s3c24xx_info->cmd, 0xff);
	
	return ERROR_OK;
}

int s3c24xx_command(struct nand_device_s *device, u8 command)
{
	s3c24xx_nand_controller_t *s3c24xx_info = device->controller_priv;
	target_t *target = s3c24xx_info->target;
	
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use S3C24XX NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	target_write_u16(target, s3c24xx_info->cmd, command);
	return ERROR_OK;
}


int s3c24xx_address(struct nand_device_s *device, u8 address)
{
	s3c24xx_nand_controller_t *s3c24xx_info = device->controller_priv;
	target_t *target = s3c24xx_info->target;
	
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use S3C24XX NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}
	
	target_write_u16(target, s3c24xx_info->addr, address);
	return ERROR_OK;
}

int s3c24xx_write_data(struct nand_device_s *device, u16 data)
{
	s3c24xx_nand_controller_t *s3c24xx_info = device->controller_priv;
	target_t *target = s3c24xx_info->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use S3C24XX NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}
	
	target_write_u8(target, s3c24xx_info->data, data);
	return ERROR_OK;
}

int s3c24xx_read_data(struct nand_device_s *device, void *data)
{
	s3c24xx_nand_controller_t *s3c24xx_info = device->controller_priv;
	target_t *target = s3c24xx_info->target;
	
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("target must be halted to use S3C24XX NAND flash controller");
		return ERROR_NAND_OPERATION_FAILED;
	}

	target_read_u8(target, s3c24xx_info->data, data);
	return ERROR_OK;
}

int s3c24xx_controller_ready(struct nand_device_s *device, int timeout)
{
	return 1;
}
