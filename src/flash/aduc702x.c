/***************************************************************************
 *   Copyright (C) 2008 by Kevin McGuire                                   *
 *   Copyright (C) 2008 by Marcel Wijlaars                                 *
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

#include "replacements.h"

#include "flash.h"
#include "target.h"
#include "log.h"
#include "armv4_5.h"
#include "algorithm.h"
#include "binarybuffer.h"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

int aduc702x_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank);
int aduc702x_register_commands(struct command_context_s *cmd_ctx);
int aduc702x_erase(struct flash_bank_s *bank, int first, int last);
int aduc702x_protect(struct flash_bank_s *bank, int set, int first, int last);
int aduc702x_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count);
int aduc702x_probe(struct flash_bank_s *bank);
int aduc702x_info(struct flash_bank_s *bank, char *buf, int buf_size);
int aduc702x_protect_check(struct flash_bank_s *bank);

#define ADUC702x_FLASH				0xfffff800
#define ADUC702x_FLASH_FEESTA		(0*4)
#define ADUC702x_FLASH_FEEMOD		(1*4)
#define ADUC702x_FLASH_FEECON		(2*4)
#define ADUC702x_FLASH_FEEDAT		(3*4)
#define ADUC702x_FLASH_FEEADR		(4*4)
#define ADUC702x_FLASH_FEESIGN		(5*4)
#define ADUC702x_FLASH_FEEPRO		(6*4)
#define ADUC702x_FLASH_FEEHIDE		(7*4)

typedef struct {
	u32 feesta;
	u32 feemod;
	u32 feecon;
	u32 feedat;
	u32 feeadr;
	u32 feesign;
	u32 feepro;
	u32 feehide;
} ADUC702x_FLASH_MMIO;

typedef struct
{
	unsigned char tmp;
} aduc702x_bank;

flash_driver_t aduc702x_flash =
{
	.name = "aduc702x",
	.register_commands = aduc702x_register_commands,
	.flash_bank_command = aduc702x_flash_bank_command,
	.erase = aduc702x_erase,
	.protect = aduc702x_protect,
	.write = aduc702x_write,
	.probe = aduc702x_probe,
	.auto_probe = aduc702x_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = aduc702x_protect_check,
	.info = aduc702x_info
};

int aduc702x_register_commands(struct command_context_s *cmd_ctx)
{
	return ERROR_OK;
}

/* flash bank aduc702x <base> <size> 0 0 <target#>  */
int aduc702x_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank)
{
	aduc702x_bank *nbank;

	if (argc < 6)
	{
		LOG_WARNING("incomplete flash_bank aduc702x configuration");
		return ERROR_FLASH_BANK_INVALID;
	}

	nbank = malloc(sizeof(aduc702x_bank));
	/* just warn that we are used to normally using 0x80000 */
	if (bank->base != 0x80000)
	{
		LOG_WARNING("Default base address is 0x80000 on the ADuC7026!");
	}
	
	nbank->tmp = 1;
	bank->driver_priv = nbank;
	return ERROR_OK;
}

int aduc702x_protect_check(struct flash_bank_s *bank)
{
	printf("aduc702x_protect_check not implemented yet.\n");
	return ERROR_OK;
}

int aduc702x_erase(struct flash_bank_s *bank, int first, int last)
{
	int x;
	int count;
	u32 v;
	target_t *target = bank->target;

	/* mass erase */
	if ((first | last) == 0)
	{
		LOG_DEBUG("performing mass erase.\n");
		target_write_u32(target, ADUC702x_FLASH + ADUC702x_FLASH_FEEDAT, 0x3cff);
		target_write_u32(target, ADUC702x_FLASH + ADUC702x_FLASH_FEEADR, 0xffc3);
		target_read_u32(target, ADUC702x_FLASH + ADUC702x_FLASH_FEEMOD, &v);
		target_write_u32(target, ADUC702x_FLASH + ADUC702x_FLASH_FEEMOD, v | 0x8);
		target_write_u32(target, ADUC702x_FLASH + ADUC702x_FLASH_FEECON, 0x06);
		for (v = 0x4; v & 0x4;
			target_read_u32(target, ADUC702x_FLASH + ADUC702x_FLASH_FEESTA, &v));

		if (!(v & 0x1))
		{
			LOG_ERROR("mass erase failed.\n");
			return ERROR_FLASH_SECTOR_NOT_ERASED;
		}
		LOG_DEBUG("mass erase successful.\n");
		return ERROR_OK;
	}

	count = last - first;
	for (x = 0; x < count; ++x)
	{
		target_write_u32(target, ADUC702x_FLASH + ADUC702x_FLASH_FEEADR, bank->base + first * 512 + x * 512);
		target_write_u32(target, ADUC702x_FLASH + ADUC702x_FLASH_FEECON, 0x05);
		for (v = 0x4; v & 0x4; target_read_u32(target, ADUC702x_FLASH + ADUC702x_FLASH_FEESTA, &v));
		if (!(v & 0x1))
		{
			LOG_ERROR("erase failed for page address %x\n", bank->base + first * 512 + x * 512);
			return ERROR_FLASH_SECTOR_NOT_ERASED;
		}
		LOG_DEBUG("erased page address %x\n", bank->base + first * 512 + x * 512);
	}
	return ERROR_OK;
}

int aduc702x_protect(struct flash_bank_s *bank, int set, int first, int last)
{
	printf("aduc702x_protect not implemented yet.\n");
	return ERROR_FLASH_OPERATION_FAILED;
}

int aduc702x_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count)
{
	int x;
	u32	v;
	target_t *target = bank->target;
	
	for (x = 0; x < count; x += 2)
	{
		target_write_u32(target, ADUC702x_FLASH + ADUC702x_FLASH_FEEADR, offset + x);
		/* if we able to encounter a single byte instead of a word */
		if ((x + 1) == count) {
			target_write_u32(target, ADUC702x_FLASH + ADUC702x_FLASH_FEEDAT, buffer[x]);
		}
		else {
			target_write_u32(target, ADUC702x_FLASH + ADUC702x_FLASH_FEEDAT, buffer[x] | (buffer[x+1] << 8));
		}
		target_write_u32(target, ADUC702x_FLASH + ADUC702x_FLASH_FEECON, 0x02);
		for (v = 0x4; v & 0x4; target_read_u32(target, ADUC702x_FLASH + ADUC702x_FLASH_FEESTA, &v));
		if (!(v & 0x1))
		{
			LOG_ERROR("single write failed for address %x.\n", offset + x);
			return ERROR_FLASH_OPERATION_FAILED;
		}
		LOG_DEBUG("single write for address %x.\n", offset + x);
	}
	return ERROR_OK;
}

int aduc702x_probe(struct flash_bank_s *bank)
{
	return ERROR_OK;
}

int aduc702x_info(struct flash_bank_s *bank, char *buf, int buf_size)
{
	snprintf(buf, buf_size, "aduc702x flash driver info" );
	return ERROR_OK;
}
