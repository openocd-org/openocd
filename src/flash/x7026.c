/***************************************************************************
 *   Copyright (C) 2008 by Kevin McGuire                                   *
 *   Copyright (C) 2008 by Marcel Wijlaars                                 *
 *
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

int x7026_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank);
int x7026_register_commands(struct command_context_s *cmd_ctx);
int x7026_erase(struct flash_bank_s *bank, int first, int last);
int x7026_protect(struct flash_bank_s *bank, int set, int first, int last);
int x7026_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count);
int x7026_probe(struct flash_bank_s *bank);
int x7026_info(struct flash_bank_s *bank, char *buf, int buf_size);
int x7026_protect_check(struct flash_bank_s *bank);
int x7026_erase_check(struct flash_bank_s *bank);

#define ADUC7026_FLASH				0xfffff800
#define ADUC7026_FLASH_FEESTA			(0*4)
#define ADUC7026_FLASH_FEEMOD			(1*4)
#define ADUC7026_FLASH_FEECON			(2*4)
#define ADUC7026_FLASH_FEEDAT			(3*4)
#define ADUC7026_FLASH_FEEADR			(4*4)
#define ADUC7026_FLASH_FEESIGN		(5*4)
#define ADUC7026_FLASH_FEEPRO			(6*4)
#define ADUC7026_FLASH_FEEHIDE		(7*4)

typedef struct{
	uint32_t	feesta;
	uint32_t	feemod;
	uint32_t	feecon;
	uint32_t	feedat;
	uint32_t	feeadr;
	uint32_t	feesign;
	uint32_t	feepro;
	uint32_t	feehide;
} ADUC7026_FLASH_MMIO;

typedef struct
{
	unsigned char tmp;
} X7026_BANK;

flash_driver_t x7026_flash =
{
	.name = "x7026",
	.register_commands = x7026_register_commands,
	.flash_bank_command = x7026_flash_bank_command,
	.erase = x7026_erase,
	.protect = x7026_protect,
	.write = x7026_write,
	.probe = x7026_probe,
	.auto_probe = x7026_probe,
	.erase_check = x7026_erase_check,
	.protect_check = x7026_protect_check,
	.info = x7026_info
};

int x7026_register_commands(struct command_context_s *cmd_ctx)
{
	printf("x7026_register_commands not implemented yet.\n");
	return ERROR_OK;
}

/* flash bank str7x <base> <size> 0 0 <target#>  */
int x7026_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank)
{
	X7026_BANK	*nbank;

	if (argc < 6)
	{
		LOG_WARNING("incomplete flash_bank x7026 configuration");
		return ERROR_FLASH_BANK_INVALID;
	}

	nbank = malloc(sizeof(X7026_BANK));
	/* just warn that we are used to normally using 0x80000 */
	if(bank->base != 0x80000)
	{
		LOG_WARNING("Default base address is 0x80000 on the ADuC7026!");
	}
	nbank->tmp = 1;
	bank->driver_priv = nbank;
	return ERROR_OK;
}

int x7026_protect_check(struct flash_bank_s *bank)
{
	printf("x7026_protect_check not implemented yet.\n");
	return ERROR_OK;
}

int x7026_erase(struct flash_bank_s *bank, int first, int last)
{
	unsigned int 	x;
	int 			count;
	u32 			v;
	target_t *target = bank->target;

	/* mass erase */
	if((first | last) == 0)
	{
		printf("performing mass erase.\n");
		target_write_u32(target, ADUC7026_FLASH + ADUC7026_FLASH_FEEDAT, 0x3cff);
		target_write_u32(target, ADUC7026_FLASH + ADUC7026_FLASH_FEEADR, 0xffc3);
		target_read_u32(target, ADUC7026_FLASH + ADUC7026_FLASH_FEEMOD, &v);
		target_write_u32(target, ADUC7026_FLASH + ADUC7026_FLASH_FEEMOD, v | 0x8);
		target_write_u32(target, ADUC7026_FLASH + ADUC7026_FLASH_FEECON, 0x06);
		for(v = 0x4; v & 0x4;
			target_read_u32(target, ADUC7026_FLASH + ADUC7026_FLASH_FEESTA, &v));

		if(!(v & 0x1))
		{
			printf("mass erase failed.\n");
			return -1;
		}
		printf("mass erase successful.\n");
		return ERROR_OK;
	}

	count = last - first;
	for(x = 0; x < (unsigned int)count; ++x)
	{
		target_write_u32(target, ADUC7026_FLASH + ADUC7026_FLASH_FEEADR, bank->base + first * 512 + x * 512);
		target_write_u32(target, ADUC7026_FLASH + ADUC7026_FLASH_FEECON, 0x05);
		for(v = 0x4; v & 0x4; target_read_u32(target, ADUC7026_FLASH + ADUC7026_FLASH_FEESTA, &v));
		if(!(v & 0x1))
		{
			printf("erase failed for page address %x\n", bank->base + first * 512 + x * 512);
			return -1;
		}
		printf("erased page address %x\n", bank->base + first * 512 + x * 512);
	}
	return ERROR_OK;
}

int x7026_protect(struct flash_bank_s *bank, int set, int first, int last)
{
	printf("x7026_protect not implemented yet.\n");
	return -1;
}

int x7026_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count)
{
	unsigned int 	x;
	u32			v;
	target_t 		*target = bank->target;
	for(x = 0; x < count; x += 2)
	{
		target_write_u32(target, ADUC7026_FLASH + ADUC7026_FLASH_FEEADR, offset + x);
		/* if we able to encounter a single byte instead of a word */
		if((x + 1) == count)
		{
			target_write_u32(target, ADUC7026_FLASH + ADUC7026_FLASH_FEEDAT, buffer[x]);
		}else{
			target_write_u32(target, ADUC7026_FLASH + ADUC7026_FLASH_FEEDAT, buffer[x] | (buffer[x+1] << 8));
		}
		target_write_u32(target, ADUC7026_FLASH + ADUC7026_FLASH_FEECON, 0x02);
		for(v = 0x4; v & 0x4; target_read_u32(target, ADUC7026_FLASH + ADUC7026_FLASH_FEESTA, &v));
		if(!(v & 0x1))
		{
			printf("single write failed for address %x.\n", offset + x);
			return -1;
		}
		printf("single write for address %x.\n", offset + x);
	}
	return ERROR_OK;
}

int x7026_probe(struct flash_bank_s *bank)
{
	printf("x7026_probe not implemented yet.\n");
	return ERROR_OK;
}

int x7026_info(struct flash_bank_s *bank, char *buf, int buf_size)
{
	snprintf(buf, buf_size, "x7026 flash driver info" );
	return -1;
}

int x7026_erase_check(struct flash_bank_s *bank)
{
	printf("x7026_erase_check not implemented yet.\n");
	return -1;
}
