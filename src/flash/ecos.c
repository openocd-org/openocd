/***************************************************************************
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string.h>

#include "replacements.h"

#include "flash.h"

#include "target.h"

#include "flash.h"
#include "target.h"
#include "log.h"
#include "binarybuffer.h"
#include "../target/embeddedice.h"
#include "types.h"

int ecosflash_register_commands(struct command_context_s *cmd_ctx);
int ecosflash_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank);
int ecosflash_erase(struct flash_bank_s *bank, int first, int last);
int ecosflash_protect(struct flash_bank_s *bank, int set, int first, int last);
int ecosflash_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count);
int ecosflash_probe(struct flash_bank_s *bank);
int ecosflash_protect_check(struct flash_bank_s *bank);
int ecosflash_info(struct flash_bank_s *bank, char *buf, int buf_size);

u32 ecosflash_get_flash_status(flash_bank_t *bank);
void ecosflash_set_flash_mode(flash_bank_t *bank,int mode);
u32 ecosflash_wait_status_busy(flash_bank_t *bank, u32 waitbits, int timeout);
int ecosflash_handle_gpnvm_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

flash_driver_t ecosflash_flash =
{
	.name = "ecosflash",
	.register_commands = ecosflash_register_commands,
	.flash_bank_command = ecosflash_flash_bank_command,
	.erase = ecosflash_erase,
	.protect = ecosflash_protect,
	.write = ecosflash_write,
	.probe = ecosflash_probe,
	.auto_probe = ecosflash_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = ecosflash_protect_check,
	.info = ecosflash_info
};

typedef struct ecosflash_flash_bank_s
{
	struct target_s *target;
	working_area_t *write_algorithm;
	working_area_t *erase_check_algorithm;
	char *driverPath;
	u32 start_address;
} ecosflash_flash_bank_t;

static const int sectorSize=0x10000;

char *
flash_errmsg(int err);

#ifndef __ECOS
#define FLASH_ERR_OK              0x00  /* No error - operation complete */
#define FLASH_ERR_INVALID         0x01  /* Invalid FLASH address */
#define FLASH_ERR_ERASE           0x02  /* Error trying to erase */
#define FLASH_ERR_LOCK            0x03  /* Error trying to lock/unlock */
#define FLASH_ERR_PROGRAM         0x04  /* Error trying to program */
#define FLASH_ERR_PROTOCOL        0x05  /* Generic error */
#define FLASH_ERR_PROTECT         0x06  /* Device/region is write-protected */
#define FLASH_ERR_NOT_INIT        0x07  /* FLASH info not yet initialized */
#define FLASH_ERR_HWR             0x08  /* Hardware (configuration?) problem */
#define FLASH_ERR_ERASE_SUSPEND   0x09  /* Device is in erase suspend mode */
#define FLASH_ERR_PROGRAM_SUSPEND 0x0a  /* Device is in in program suspend mode */
#define FLASH_ERR_DRV_VERIFY      0x0b  /* Driver failed to verify data */
#define FLASH_ERR_DRV_TIMEOUT     0x0c  /* Driver timed out waiting for device */
#define FLASH_ERR_DRV_WRONG_PART  0x0d  /* Driver does not support device */
#define FLASH_ERR_LOW_VOLTAGE     0x0e  /* Not enough juice to complete job */

char *
flash_errmsg(int err)
{
	switch (err) {
	case FLASH_ERR_OK:
		return "No error - operation complete";
	case FLASH_ERR_ERASE_SUSPEND:
		return "Device is in erase suspend state";
	case FLASH_ERR_PROGRAM_SUSPEND:
		return "Device is in program suspend state";
	case FLASH_ERR_INVALID:
		return "Invalid FLASH address";
	case FLASH_ERR_ERASE:
		return "Error trying to erase";
	case FLASH_ERR_LOCK:
		return "Error trying to lock/unlock";
	case FLASH_ERR_PROGRAM:
		return "Error trying to program";
	case FLASH_ERR_PROTOCOL:
		return "Generic error";
	case FLASH_ERR_PROTECT:
		return "Device/region is write-protected";
	case FLASH_ERR_NOT_INIT:
		return "FLASH sub-system not initialized";
	case FLASH_ERR_DRV_VERIFY:
		return "Data verify failed after operation";
	case FLASH_ERR_DRV_TIMEOUT:
		return "Driver timed out waiting for device";
	case FLASH_ERR_DRV_WRONG_PART:
		return "Driver does not support device";
	case FLASH_ERR_LOW_VOLTAGE:
		return "Device reports low voltage";
	default:
		return "Unknown error";
	}
}
#endif

/* flash bank ecosflash <base> <size> <chip_width> <bus_width> <target#> <driverPath>
 */
int ecosflash_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank)
{
	ecosflash_flash_bank_t *info;

	if (argc < 7)
	{
		LOG_WARNING("incomplete flash_bank ecosflash configuration");
		return ERROR_FLASH_BANK_INVALID;
	}

	info = malloc(sizeof(ecosflash_flash_bank_t));
	if(info == NULL)
	{
		LOG_ERROR("no memory for flash bank info");
		exit(-1);
	}
	bank->driver_priv = info;
	info->driverPath=strdup(args[6]);

	/* eCos flash sector sizes are not exposed to OpenOCD, use 0x10000 as
	 * a way to improve impeadance matach between OpenOCD and eCos flash
	 * driver.
	 */
	int i = 0;
	u32 offset = 0;
	bank->num_sectors=bank->size/sectorSize;
	bank->sectors = malloc(sizeof(flash_sector_t) * bank->num_sectors);
	for (i = 0; i < bank->num_sectors; i++)
	{
		bank->sectors[i].offset = offset;
		bank->sectors[i].size = sectorSize;
		offset += bank->sectors[i].size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 0;
	}

	info->target = get_target_by_num(strtoul(args[5], NULL, 0));
	if (info->target == NULL)
	{
		LOG_ERROR("no target '%i' configured", (int)strtoul(args[5], NULL, 0));
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

int loadDriver(ecosflash_flash_bank_t *info)
{
	u32 buf_cnt;
	u32 image_size;
	image_t image;

	image.base_address_set = 0;
	image.start_address_set = 0;
	target_t *target=info->target;
	int retval;

	if ((retval=image_open(&image, info->driverPath, NULL)) != ERROR_OK)
	{
		return retval;
	}

	info->start_address=image.start_address;

	image_size = 0x0;
	int i;
	for (i = 0; i < image.num_sections; i++)
	{
		void *buffer = malloc(image.sections[i].size);
		int retval;
		if ((retval = image_read_section(&image, i, 0x0, image.sections[i].size, buffer, &buf_cnt)) != ERROR_OK)
		{
			free(buffer);
			image_close(&image);
			return retval;
		}
		target_write_buffer(target, image.sections[i].base_address, buf_cnt, buffer);
		image_size += buf_cnt;
		LOG_DEBUG("%u byte written at address 0x%8.8x", buf_cnt, image.sections[i].base_address);

		free(buffer);
	}

	image_close(&image);

	return ERROR_OK;
}

static int const OFFSET_ERASE=0x0;
static int const OFFSET_ERASE_SIZE=0x8;
static int const OFFSET_FLASH=0xc;
static int const OFFSET_FLASH_SIZE=0x8;
static int const OFFSET_GET_WORKAREA=0x18;
static int const OFFSET_GET_WORKAREA_SIZE=0x4;

int runCode(ecosflash_flash_bank_t *info,
		u32 codeStart, u32 codeStop, u32 r0, u32 r1, u32 r2,
		u32 *result,
		/* timeout in ms */
		int timeout)
{
	target_t *target=info->target;

	reg_param_t reg_params[3];
	armv4_5_algorithm_t armv4_5_info;
	armv4_5_info.common_magic = ARMV4_5_COMMON_MAGIC;
	armv4_5_info.core_mode = ARMV4_5_MODE_SVC;
	armv4_5_info.core_state = ARMV4_5_STATE_ARM;

	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);

	buf_set_u32(reg_params[0].value, 0, 32, r0);
	buf_set_u32(reg_params[1].value, 0, 32, r1);
	buf_set_u32(reg_params[2].value, 0, 32, r2);

	int retval;
	if ((retval = target->type->run_algorithm(target, 0, NULL, 3, reg_params,
			codeStart,
			codeStop, timeout,
			&armv4_5_info)) != ERROR_OK)
	{
		LOG_ERROR("error executing eCos flash algorithm");
		return retval;
	}

	*result=buf_get_u32(reg_params[0].value, 0, 32);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);

	return ERROR_OK;
}

int eCosBoard_erase(ecosflash_flash_bank_t *info, u32 address, u32 len)
{
	int retval;
	int timeout = (len / 20480 + 1) * 1000; /*asume 20 KB/s*/

	retval=loadDriver(info);
	if (retval!=ERROR_OK)
		return retval;

	u32 flashErr;
	retval=runCode(info,
			info->start_address+OFFSET_ERASE,
			info->start_address+OFFSET_ERASE+OFFSET_ERASE_SIZE,
			address,
			len,
			0,
			&flashErr,
			timeout
			);
	if (retval!=ERROR_OK)
		return retval;

	if (flashErr != 0x0)
	{
		LOG_ERROR("Flash erase failed with %d (%s)\n", flashErr, flash_errmsg(flashErr));
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

int eCosBoard_flash(ecosflash_flash_bank_t *info, void *data, u32 address, u32 len)
{
	target_t *target=info->target;
	const int chunk=8192;
	int retval=ERROR_OK;
	int timeout = (chunk / 20480 + 1) * 1000; /*asume 20 KB/s + 1 second*/

	retval=loadDriver(info);
	if (retval!=ERROR_OK)
		return retval;

	u32 buffer;
	retval=runCode(info,
			info->start_address+OFFSET_GET_WORKAREA,
			info->start_address+OFFSET_GET_WORKAREA+OFFSET_GET_WORKAREA_SIZE,
			0,
			0,
			0,
			&buffer,
			1000);
	if (retval!=ERROR_OK)
		return retval;


	int i;
	for (i=0; i<len; i+=chunk)
	{
		int t=len-i;
		if (t>chunk)
		{
			t=chunk;
		}

		int retval;
		retval=target_write_buffer(target, buffer, t, ((u8 *)data)+i);
		if (retval != ERROR_OK)
			return retval;

		u32 flashErr;
		retval=runCode(info,
				info->start_address+OFFSET_FLASH,
				info->start_address+OFFSET_FLASH+OFFSET_FLASH_SIZE,
				buffer,
				address+i,
				t,
				&flashErr,
				timeout);
		if (retval != ERROR_OK)
			return retval;

		if (flashErr != 0x0)
		{
			LOG_ERROR("Flash prog failed with %d (%s)\n", flashErr, flash_errmsg(flashErr));
			return ERROR_FAIL;
		}
	}
	return ERROR_OK;
}

int ecosflash_probe(struct flash_bank_s *bank)
{
	return ERROR_OK;
}

int ecosflash_register_commands(struct command_context_s *cmd_ctx)
{
	register_command(cmd_ctx, NULL, "ecosflash", NULL, COMMAND_ANY, NULL);

	return ERROR_OK;
}

#if 0
static void command(flash_bank_t *bank, u8 cmd, u8 *cmd_buf)
{
	ecosflash_flash_bank_t *info = bank->driver_priv;
	int i;

	if (info->target->endianness == TARGET_LITTLE_ENDIAN)
	{
		for (i = bank->bus_width; i > 0; i--)
		{
			*cmd_buf++ = (i & (bank->chip_width - 1)) ? 0x0 : cmd;
		}
	}
	else
	{
		for (i = 1; i <= bank->bus_width; i++)
		{
			*cmd_buf++ = (i & (bank->chip_width - 1)) ? 0x0 : cmd;
		}
	}
}
#endif

u32 ecosflash_address(struct flash_bank_s *bank, u32 address)
{
	u32 retval = 0;
	switch(bank->bus_width)
	{
		case 4:
			retval = address & 0xfffffffc;
		case 2:
			retval = address & 0xfffffffe;
		case 1:
			retval = address;
	}

	return retval + bank->base;
}

int ecosflash_erase(struct flash_bank_s *bank, int first, int last)
{
	struct flash_bank_s *c=bank;
	ecosflash_flash_bank_t *info = bank->driver_priv;
	return eCosBoard_erase(info, c->base+first*sectorSize, sectorSize*(last-first+1));
}

int ecosflash_protect(struct flash_bank_s *bank, int set, int first, int last)
{
	return ERROR_OK;
}

int ecosflash_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count)
{
	ecosflash_flash_bank_t *info = bank->driver_priv;
	struct flash_bank_s *c=bank;
	return eCosBoard_flash(info, buffer, c->base+offset, count);
}

int ecosflash_protect_check(struct flash_bank_s *bank)
{
	return ERROR_OK;
}

int ecosflash_info(struct flash_bank_s *bank, char *buf, int buf_size)
{
	ecosflash_flash_bank_t *info = bank->driver_priv;
	snprintf(buf, buf_size, "eCos flash driver: %s", info->driverPath);
	return ERROR_OK;
}

u32 ecosflash_get_flash_status(flash_bank_t *bank)
{
	return ERROR_OK;
}

void ecosflash_set_flash_mode(flash_bank_t *bank,int mode)
{

}

u32 ecosflash_wait_status_busy(flash_bank_t *bank, u32 waitbits, int timeout)
{
	return ERROR_OK;
}

int ecosflash_handle_gpnvm_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	return ERROR_OK;
}
