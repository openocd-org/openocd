/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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

#include "str7x.h"
#include "flash.h"
#include "target.h"
#include "log.h"
#include "armv4_5.h"
#include "algorithm.h"
#include "binarybuffer.h"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

str7x_mem_layout_t mem_layout[] = {
	{0x00000000, 0x02000, 0x01},
	{0x00002000, 0x02000, 0x01},
	{0x00004000, 0x02000, 0x01},
	{0x00006000, 0x02000, 0x01},
	{0x00008000, 0x08000, 0x01},
	{0x00010000, 0x10000, 0x01},
	{0x00020000, 0x10000, 0x01},
	{0x00030000, 0x10000, 0x01},
	{0x000C0000, 0x02000, 0x10},
	{0x000C2000, 0x02000, 0x10},
	{0,0},
};

int str7x_register_commands(struct command_context_s *cmd_ctx);
int str7x_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank);
int str7x_erase(struct flash_bank_s *bank, int first, int last);
int str7x_protect(struct flash_bank_s *bank, int set, int first, int last);
int str7x_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count);
int str7x_probe(struct flash_bank_s *bank);
int str7x_handle_part_id_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int str7x_protect_check(struct flash_bank_s *bank);
int str7x_erase_check(struct flash_bank_s *bank);
int str7x_info(struct flash_bank_s *bank, char *buf, int buf_size);

flash_driver_t str7x_flash =
{
	.name = "str7x",
	.register_commands = str7x_register_commands,
	.flash_bank_command = str7x_flash_bank_command,
	.erase = str7x_erase,
	.protect = str7x_protect,
	.write = str7x_write,
	.probe = str7x_probe,
	.erase_check = str7x_erase_check,
	.protect_check = str7x_protect_check,
	.info = str7x_info
};

int str7x_register_commands(struct command_context_s *cmd_ctx)
{

	return ERROR_OK;
}

int str7x_get_flash_adr(struct flash_bank_s *bank, u32 reg)
{
	str7x_flash_bank_t *str7x_info = bank->driver_priv;
	return (str7x_info->flash_base|reg);
}

int str7x_build_block_list(struct flash_bank_s *bank)
{
	str7x_flash_bank_t *str7x_info = bank->driver_priv;

	int i;
	int num_sectors;
		
	switch (bank->size)
	{
		case 16 * 1024:
			num_sectors = 2;
			break;
		case 64 * 1024:
			num_sectors = 5;
			break;
		case 128 * 1024:
			num_sectors = 6;
			break;
		case 256 * 1024:
			num_sectors = 8;
			break;
		default:
			ERROR("BUG: unknown bank->size encountered");
			exit(-1);
	}
	
	if( str7x_info->bank1 == 1 )
	{
		num_sectors += 2;
	}
	
	bank->num_sectors = num_sectors;
	bank->sectors = malloc(sizeof(flash_sector_t) * num_sectors);
	
	for (i = 0; i < num_sectors; i++)
	{
		bank->sectors[i].offset = mem_layout[i].sector_start;
		bank->sectors[i].size = mem_layout[i].sector_size;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}

	return ERROR_OK;
}

/* flash bank str7x <base> <size> 0 0 <str71_variant> <target#>
 */
int str7x_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank)
{
	str7x_flash_bank_t *str7x_info;
	
	if (argc < 7)
	{
		WARNING("incomplete flash_bank str7x configuration");
		return ERROR_FLASH_BANK_INVALID;
	}
	
	str7x_info = malloc(sizeof(str7x_flash_bank_t));
	bank->driver_priv = str7x_info;
	
	if (strcmp(args[5], "STR71x") == 0)
	{
		str7x_info->bank1 = 1;
		str7x_info->flash_base = 0x40000000;
	}
	else if (strcmp(args[5], "STR73x") == 0)
	{
		str7x_info->bank1 = 0;
		str7x_info->flash_base = 0x80000000;
	}
	else
	{
		ERROR("unknown STR7x variant");
		free(str7x_info);
		return ERROR_FLASH_BANK_INVALID;
	}
	
	str7x_info->target = get_target_by_num(strtoul(args[6], NULL, 0));
	if (!str7x_info->target)
	{
		ERROR("no target '%s' configured", args[6]);
		exit(-1);
	}

	str7x_build_block_list(bank);
	
	return ERROR_OK;
}

u32 str7x_status(struct flash_bank_s *bank)
{
	str7x_flash_bank_t *str7x_info = bank->driver_priv;
	target_t *target = str7x_info->target;
	u32 retval;

	target->type->read_memory(target, str7x_get_flash_adr(bank, FLASH_CR0), 4, 1, (u8*)&retval);

	return retval;
}

u32 str7x_result(struct flash_bank_s *bank)
{
	str7x_flash_bank_t *str7x_info = bank->driver_priv;
	target_t *target = str7x_info->target;
	u32 retval;

	target->type->read_memory(target, str7x_get_flash_adr(bank, FLASH_ER), 4, 1, (u8*)&retval);
	
	return retval;
}

int str7x_blank_check(struct flash_bank_s *bank, int first, int last)
{
	str7x_flash_bank_t *str7x_info = bank->driver_priv;
	target_t *target = str7x_info->target;
	u8 *buffer;
	int i;
	int nBytes;
	
	if ((first < 0) || (last > bank->num_sectors))
		return ERROR_FLASH_SECTOR_INVALID;

	if (str7x_info->target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}
	
	buffer = malloc(256);
	
	for (i = first; i <= last; i++)
	{
		bank->sectors[i].is_erased = 1;

		target->type->read_memory(target, bank->base + bank->sectors[i].offset, 4, 256/4, buffer);
		
		for (nBytes = 0; nBytes < 256; nBytes++)
		{
			if (buffer[nBytes] != 0xFF)
			{
				bank->sectors[i].is_erased = 0;
				break;
			}
		}	
	}
	
	free(buffer);

	return ERROR_OK;
}

int str7x_protect_check(struct flash_bank_s *bank)
{
	str7x_flash_bank_t *str7x_info = bank->driver_priv;
	target_t *target = str7x_info->target;
	
	int i;
	int retval;

	if (str7x_info->target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}

	target->type->read_memory(target, str7x_get_flash_adr(bank, FLASH_NVWPAR), 4, 1, (u8*)&retval);

	for (i = 0; i < bank->num_sectors; i++)
	{
		if (retval & (mem_layout[i].reg_offset << i))
			bank->sectors[i].is_protected = 0;
		else
			bank->sectors[i].is_protected = 1;
	}

	return ERROR_OK;
}

int str7x_erase(struct flash_bank_s *bank, int first, int last)
{
	str7x_flash_bank_t *str7x_info = bank->driver_priv;
	target_t *target = str7x_info->target;
	
	int i;
	u32 cmd;
	u32 retval;
	u32 erase_blocks;
	
	if (str7x_info->target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}
	
	erase_blocks = 0;
	
	for (i = first; i <= last; i++)
		erase_blocks |= (mem_layout[i].reg_offset << i);
	
	cmd = FLASH_SER;
	target->type->write_memory(target, str7x_get_flash_adr(bank, FLASH_CR0), 4, 1, (u8*)&cmd);
	
	cmd = erase_blocks;
	target->type->write_memory(target, str7x_get_flash_adr(bank, FLASH_CR1), 4, 1, (u8*)&cmd);
	
	cmd = FLASH_SER|FLASH_WMS;
	target->type->write_memory(target, str7x_get_flash_adr(bank, FLASH_CR0), 4, 1, (u8*)&cmd);
	
	while (((retval = str7x_status(bank)) & (FLASH_BSYA1|FLASH_BSYA2))){
		usleep(1000);
	}
	
	retval = str7x_result(bank);
	
	if (retval & FLASH_ERER)
		return ERROR_FLASH_SECTOR_NOT_ERASED;
	else if (retval & FLASH_WPF)
		return ERROR_FLASH_OPERATION_FAILED;
	
	for (i = first; i <= last; i++)
		bank->sectors[i].is_erased = 1;

	return ERROR_OK;
}

int str7x_protect(struct flash_bank_s *bank, int set, int first, int last)
{
	str7x_flash_bank_t *str7x_info = bank->driver_priv;
	target_t *target = str7x_info->target;
	int i;
	u32 cmd;
	u32 retval;
	u32 protect_blocks;
	
	if (str7x_info->target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}
	
	protect_blocks = 0xFFFFFFFF;

	if( set )
	{
		for (i = first; i <= last; i++)
			protect_blocks &= ~(mem_layout[i].reg_offset << i);
	}

	cmd = FLASH_SPR;
	target->type->write_memory(target, str7x_get_flash_adr(bank, FLASH_CR0), 4, 1, (u8*)&cmd);
	
	cmd = str7x_get_flash_adr(bank, FLASH_NVWPAR);
	target->type->write_memory(target, str7x_get_flash_adr(bank, FLASH_AR), 4, 1, (u8*)&cmd);
	
	cmd = protect_blocks;
	target->type->write_memory(target, str7x_get_flash_adr(bank, FLASH_DR0), 4, 1, (u8*)&cmd);
	
	cmd = FLASH_SPR|FLASH_WMS;
	target->type->write_memory(target, str7x_get_flash_adr(bank, FLASH_CR0), 4, 1, (u8*)&cmd);
	
	while (((retval = str7x_status(bank)) & (FLASH_BSYA1|FLASH_BSYA2))){
		usleep(1000);
	}
	
	retval = str7x_result(bank);
	
	if (retval & FLASH_ERER)
		return ERROR_FLASH_SECTOR_NOT_ERASED;
	else if (retval & FLASH_WPF)
		return ERROR_FLASH_OPERATION_FAILED;

	return ERROR_OK;
}

int str7x_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count)
{
	str7x_flash_bank_t *str7x_info = bank->driver_priv;
	target_t *target = str7x_info->target;
	u32 dwords_remaining = (count / 8);
	u32 bytes_remaining = (count & 0x00000007);
	u32 address = bank->base + offset;
	u32 *wordbuffer = (u32*)buffer;
	u32 bytes_written = 0;
	u32 cmd;
	u32 retval;
	
	if (str7x_info->target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;
	
	while (dwords_remaining > 0)
	{
		// command
		cmd = FLASH_DWPG;
		target->type->write_memory(target, str7x_get_flash_adr(bank, FLASH_CR0), 4, 1, (u8*)&cmd);
		
		// address
		cmd = address;
		target->type->write_memory(target, str7x_get_flash_adr(bank, FLASH_AR), 4, 1, (u8*)&cmd);
		
		// data byte 1
		cmd = wordbuffer[bytes_written/4];
		target->type->write_memory(target, str7x_get_flash_adr(bank, FLASH_DR0), 4, 1, (u8*)&cmd);
		bytes_written += 4;
		
		// data byte 2
		cmd = wordbuffer[bytes_written/4];
		target->type->write_memory(target, str7x_get_flash_adr(bank, FLASH_DR1), 4, 1, (u8*)&cmd);
		bytes_written += 4;
		
		/* start programming cycle */
		cmd = FLASH_DWPG|FLASH_WMS;
		target->type->write_memory(target, str7x_get_flash_adr(bank, FLASH_CR0), 4, 1, (u8*)&cmd);
		
		while (((retval = str7x_status(bank)) & (FLASH_BSYA1|FLASH_BSYA2))){
			usleep(1000);
		}
		
		retval = str7x_result(bank);
		
		if (retval & FLASH_PGER)
			return ERROR_FLASH_OPERATION_FAILED;
		else if (retval & FLASH_WPF)
			return ERROR_FLASH_OPERATION_FAILED;

		dwords_remaining--;
		address += 8;
	}
	
	while( bytes_remaining > 0 )
	{
		// command
		cmd = FLASH_WPG;
		target->type->write_memory(target, str7x_get_flash_adr(bank, FLASH_CR0), 4, 1, (u8*)&cmd);
		
		// address
		cmd = address;
		target->type->write_memory(target, str7x_get_flash_adr(bank, FLASH_AR), 4, 1, (u8*)&cmd);
		
		// data byte
		cmd = buffer[bytes_written];
		target->type->write_memory(target, str7x_get_flash_adr(bank, FLASH_DR0), 4, 1, (u8*)&cmd);
		
		/* start programming cycle */
		cmd = FLASH_WPG|FLASH_WMS;
		target->type->write_memory(target, str7x_get_flash_adr(bank, FLASH_CR0), 4, 1, (u8*)&cmd);
		
		while (((retval = str7x_status(bank)) & (FLASH_BSYA1|FLASH_BSYA2))){
			usleep(1000);
		}
		
		retval = str7x_result(bank);
		
		if (retval & FLASH_PGER)
			return ERROR_FLASH_OPERATION_FAILED;
		else if (retval & FLASH_WPF)
			return ERROR_FLASH_OPERATION_FAILED;

		address++;
		bytes_remaining--;
		bytes_written++;
	}
	
	return ERROR_OK;
}

int str7x_probe(struct flash_bank_s *bank)
{
	return ERROR_OK;
}

int str7x_handle_part_id_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	return ERROR_OK;
}

int str7x_erase_check(struct flash_bank_s *bank)
{
	return str7x_blank_check(bank, 0, bank->num_sectors - 1);
}

int str7x_info(struct flash_bank_s *bank, char *buf, int buf_size)
{
	snprintf(buf, buf_size, "str7x flash driver info" );
	return ERROR_OK;
}
