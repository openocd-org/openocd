/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *
 *   Copyright (C) 2008 by Oyvind Harboe                                   *
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

#include "replacements.h"

#include "str9x.h"
#include "flash.h"
#include "target.h"
#include "log.h"
#include "armv4_5.h"
#include "arm966e.h"
#include "algorithm.h"
#include "binarybuffer.h"

#include <stdlib.h>
#include <string.h>
#include <unistd.h>

static u32 bank1start = 0x00080000;

int str9x_register_commands(struct command_context_s *cmd_ctx);
int str9x_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank);
int str9x_erase(struct flash_bank_s *bank, int first, int last);
int str9x_protect(struct flash_bank_s *bank, int set, int first, int last);
int str9x_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count);
int str9x_probe(struct flash_bank_s *bank);
int str9x_handle_part_id_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int str9x_protect_check(struct flash_bank_s *bank);
int str9x_info(struct flash_bank_s *bank, char *buf, int buf_size);

int str9x_handle_flash_config_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

flash_driver_t str9x_flash =
{
	.name = "str9x",
	.register_commands = str9x_register_commands,
	.flash_bank_command = str9x_flash_bank_command,
	.erase = str9x_erase,
	.protect = str9x_protect,
	.write = str9x_write,
	.probe = str9x_probe,
	.auto_probe = str9x_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = str9x_protect_check,
	.info = str9x_info
};

int str9x_register_commands(struct command_context_s *cmd_ctx)
{
	command_t *str9x_cmd = register_command(cmd_ctx, NULL, "str9x", NULL, COMMAND_ANY, NULL);
	
	register_command(cmd_ctx, str9x_cmd, "flash_config", str9x_handle_flash_config_command, COMMAND_EXEC,
					 "configure str9 flash controller");
					 
	return ERROR_OK;
}

int str9x_build_block_list(struct flash_bank_s *bank)
{
	str9x_flash_bank_t *str9x_info = bank->driver_priv;
	
	int i;
	int num_sectors;
	int b0_sectors = 0, b1_sectors = 0;
	u32 offset = 0;
	
	/* set if we have large flash str9 */
	str9x_info->variant = 0;
	str9x_info->bank1 = 0;
	
	switch (bank->size)
	{
		case (256 * 1024):
			b0_sectors = 4;
			break;
		case (512 * 1024):
			b0_sectors = 8;
			break;
		case (1024 * 1024):
			bank1start = 0x00100000;
			str9x_info->variant = 1;
			b0_sectors = 16;
			break;
		case (2048 * 1024):
			bank1start = 0x00200000;
			str9x_info->variant = 1;
			b0_sectors = 32;
			break;
		case (128 * 1024):
			str9x_info->variant = 1;
			str9x_info->bank1 = 1;
			b1_sectors = 8;
			bank1start = bank->base;
			break;
		case (32 * 1024):
			str9x_info->bank1 = 1;
			b1_sectors = 4;
			bank1start = bank->base;
			break;
		default:
			LOG_ERROR("BUG: unknown bank->size encountered");
			exit(-1);
	}
		
	num_sectors = b0_sectors + b1_sectors;
	
	bank->num_sectors = num_sectors;
	bank->sectors = malloc(sizeof(flash_sector_t) * num_sectors);
	str9x_info->sector_bits = malloc(sizeof(u32) * num_sectors);
	
	num_sectors = 0;
	
	for (i = 0; i < b0_sectors; i++)
	{
		bank->sectors[num_sectors].offset = offset;
		bank->sectors[num_sectors].size = 0x10000;
		offset += bank->sectors[i].size;
		bank->sectors[num_sectors].is_erased = -1;
		bank->sectors[num_sectors].is_protected = 1;
		str9x_info->sector_bits[num_sectors++] = (1<<i);
	}

	for (i = 0; i < b1_sectors; i++)
	{
		bank->sectors[num_sectors].offset = offset;
		bank->sectors[num_sectors].size = str9x_info->variant == 0 ? 0x2000 : 0x4000;
		offset += bank->sectors[i].size;
		bank->sectors[num_sectors].is_erased = -1;
		bank->sectors[num_sectors].is_protected = 1;
		if (str9x_info->variant)
			str9x_info->sector_bits[num_sectors++] = (1<<i);
		else
			str9x_info->sector_bits[num_sectors++] = (1<<(i+8));
	}
	
	return ERROR_OK;
}

/* flash bank str9x <base> <size> 0 0 <target#>
 */
int str9x_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank)
{
	str9x_flash_bank_t *str9x_info;
	
	if (argc < 6)
	{
		LOG_WARNING("incomplete flash_bank str9x configuration");
		return ERROR_FLASH_BANK_INVALID;
	}
	
	str9x_info = malloc(sizeof(str9x_flash_bank_t));
	bank->driver_priv = str9x_info;
	
	str9x_build_block_list(bank);
	
	str9x_info->write_algorithm = NULL;
	
	return ERROR_OK;
}

int str9x_protect_check(struct flash_bank_s *bank)
{
	int retval;
	str9x_flash_bank_t *str9x_info = bank->driver_priv;
	target_t *target = bank->target;
	
	int i;
	u32 adr;
	u32 status = 0;

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* read level one protection */
	
	if (str9x_info->variant)
	{
		if (str9x_info->bank1)
		{
			adr = bank1start + 0x18;
			if ((retval=target_write_u16(target, adr, 0x90))!=ERROR_OK)
			{
				return retval;
			}
			if ((retval=target_read_u16(target, adr, (u16*)&status))!=ERROR_OK)
			{
				return retval;
			}
		}
		else
		{
			adr = bank1start + 0x14;
			if ((retval=target_write_u16(target, adr, 0x90))!=ERROR_OK)
			{
				return retval;
			}
			if ((retval=target_read_u32(target, adr, &status))!=ERROR_OK)
			{
				return retval;
			}
		}
	}
	else
	{
		adr = bank1start + 0x10;
		if ((retval=target_write_u16(target, adr, 0x90))!=ERROR_OK)
		{
			return retval;
		}
		if ((retval=target_read_u16(target, adr, (u16*)&status))!=ERROR_OK)
		{
			return retval;
		}
	}
	
	/* read array command */
	if ((retval=target_write_u16(target, adr, 0xFF))!=ERROR_OK)
	{
		return retval;
	}
	
	for (i = 0; i < bank->num_sectors; i++)
	{
		if (status & str9x_info->sector_bits[i])
			bank->sectors[i].is_protected = 1;
		else
			bank->sectors[i].is_protected = 0;
	}
	
	return ERROR_OK;
}

int str9x_erase(struct flash_bank_s *bank, int first, int last)
{
	target_t *target = bank->target;
	int i;
	u32 adr;
	u8 status;
	u8 erase_cmd;
	
	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	/* Check if we erase whole bank */
	if ((first == 0) && (last == (bank->num_sectors - 1)))
	{
		/* Optimize to run erase bank command instead of sector */
		erase_cmd = 0x80;
	}
	else
	{
		/* Erase sector command */
		erase_cmd = 0x20;
	}
	
	for (i = first; i <= last; i++)
	{
		int retval;
		adr = bank->base + bank->sectors[i].offset;
		
		/* erase sectors */
		if ((retval=target_write_u16(target, adr, erase_cmd))!=ERROR_OK)
		{
			return retval;
		}
		if ((retval=target_write_u16(target, adr, 0xD0))!=ERROR_OK)
		{
			return retval;
		}
		
		/* get status */
		if ((retval=target_write_u16(target, adr, 0x70))!=ERROR_OK)
		{
			return retval;
		}

		int timeout;		
		for (timeout=0; timeout<1000; timeout++) {
			if ((retval=target_read_u8(target, adr, &status))!=ERROR_OK)
			{
				return retval;
			}
			if( status & 0x80 )
				break;
			alive_sleep(1);
		}
		if (timeout==1000)
		{
			LOG_ERROR("erase timed out");
			return ERROR_FAIL;
		}
		
		/* clear status, also clear read array */
		if ((retval=target_write_u16(target, adr, 0x50))!=ERROR_OK)
		{
			return retval;
		}
		
		/* read array command */
		if ((retval=target_write_u16(target, adr, 0xFF))!=ERROR_OK)
		{
			return retval;
		}
		
		if( status & 0x22 )
		{
			LOG_ERROR("error erasing flash bank, status: 0x%x", status);
			return ERROR_FLASH_OPERATION_FAILED;
		}
		
		/* If we ran erase bank command, we are finished */
		if (erase_cmd == 0x80)
			break;
	}
	
	for (i = first; i <= last; i++)
		bank->sectors[i].is_erased = 1;

	return ERROR_OK;
}

int str9x_protect(struct flash_bank_s *bank, int set, int first, int last)
{
	target_t *target = bank->target;
	int i;
	u32 adr;
	u8 status;
	
	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	for (i = first; i <= last; i++)
	{
		/* Level One Protection */
	
		adr = bank->base + bank->sectors[i].offset;
		
		target_write_u16(target, adr, 0x60);
		if( set )
			target_write_u16(target, adr, 0x01);
		else
			target_write_u16(target, adr, 0xD0);
		
		/* query status */
		target_read_u8(target, adr, &status);
		
		/* clear status, also clear read array */
		target_write_u16(target, adr, 0x50);
		
		/* read array command */
		target_write_u16(target, adr, 0xFF);
	}
	
	return ERROR_OK;
}

int str9x_write_block(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count)
{
	str9x_flash_bank_t *str9x_info = bank->driver_priv;
	target_t *target = bank->target;
	u32 buffer_size = 8192;
	working_area_t *source;
	u32 address = bank->base + offset;
	reg_param_t reg_params[4];
	armv4_5_algorithm_t armv4_5_info;
	int retval = ERROR_OK;
	
	u32 str9x_flash_write_code[] = {
					/* write:				*/
		0xe3c14003,	/*	bic	r4, r1, #3		*/
		0xe3a03040,	/*	mov	r3, #0x40		*/
		0xe1c430b0,	/*	strh r3, [r4, #0]	*/
		0xe0d030b2,	/*	ldrh r3, [r0], #2	*/
		0xe0c130b2,	/*	strh r3, [r1], #2	*/
		0xe3a03070,	/*	mov r3, #0x70		*/
		0xe1c430b0,	/*	strh r3, [r4, #0]	*/
					/* busy:				*/
		0xe5d43000,	/*	ldrb r3, [r4, #0]	*/
		0xe3130080,	/*	tst r3, #0x80		*/
		0x0afffffc,	/*	beq busy			*/
		0xe3a05050,	/*	mov	r5, #0x50		*/
		0xe1c450b0,	/*	strh r5, [r4, #0]	*/
		0xe3a050ff,	/*	mov	r5, #0xFF		*/
		0xe1c450b0,	/*	strh r5, [r4, #0]	*/
		0xe3130012,	/*	tst	r3, #0x12		*/
		0x1a000001,	/*	bne exit			*/
		0xe2522001,	/*	subs r2, r2, #1		*/
		0x1affffed,	/*	bne write			*/
					/* exit:				*/
		0xeafffffe,	/*	b exit				*/
	};
	
	/* flash write code */
	if (target_alloc_working_area(target, 4 * 19, &str9x_info->write_algorithm) != ERROR_OK)
	{
		LOG_WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};
		
	target_write_buffer(target, str9x_info->write_algorithm->address, 19 * 4, (u8*)str9x_flash_write_code);

	/* memory buffer */
	while (target_alloc_working_area(target, buffer_size, &source) != ERROR_OK)
	{
		buffer_size /= 2;
		if (buffer_size <= 256)
		{
			/* if we already allocated the writing code, but failed to get a buffer, free the algorithm */
			if (str9x_info->write_algorithm)
				target_free_working_area(target, str9x_info->write_algorithm);
			
			LOG_WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}
	
	armv4_5_info.common_magic = ARMV4_5_COMMON_MAGIC;
	armv4_5_info.core_mode = ARMV4_5_MODE_SVC;
	armv4_5_info.core_state = ARMV4_5_STATE_ARM;
	
	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_IN);
	
	while (count > 0)
	{
		u32 thisrun_count = (count > (buffer_size / 2)) ? (buffer_size / 2) : count;
		
		target_write_buffer(target, source->address, thisrun_count * 2, buffer);
		
		buf_set_u32(reg_params[0].value, 0, 32, source->address);
		buf_set_u32(reg_params[1].value, 0, 32, address);
		buf_set_u32(reg_params[2].value, 0, 32, thisrun_count);

		if ((retval = target->type->run_algorithm(target, 0, NULL, 4, reg_params, str9x_info->write_algorithm->address, str9x_info->write_algorithm->address + (18 * 4), 10000, &armv4_5_info)) != ERROR_OK)
		{
			LOG_ERROR("error executing str9x flash write algorithm");
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}
	
		if (buf_get_u32(reg_params[3].value, 0, 32) != 0x80)
		{
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}
		
		buffer += thisrun_count * 2;
		address += thisrun_count * 2;
		count -= thisrun_count;
	}
	
	target_free_working_area(target, source);
	target_free_working_area(target, str9x_info->write_algorithm);
	
	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	
	return retval;
}

int str9x_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count)
{
	target_t *target = bank->target;
	u32 words_remaining = (count / 2);
	u32 bytes_remaining = (count & 0x00000001);
	u32 address = bank->base + offset;
	u32 bytes_written = 0;
	u8 status;
	u32 retval;
	u32 check_address = offset;
	u32 bank_adr;
	int i;
	
	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	if (offset & 0x1)
	{
		LOG_WARNING("offset 0x%x breaks required 2-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}
	
	for (i = 0; i < bank->num_sectors; i++)
	{
		u32 sec_start = bank->sectors[i].offset;
		u32 sec_end = sec_start + bank->sectors[i].size;
		
		/* check if destination falls within the current sector */
		if ((check_address >= sec_start) && (check_address < sec_end))
		{
			/* check if destination ends in the current sector */
			if (offset + count < sec_end)
				check_address = offset + count;
			else
				check_address = sec_end;
		}
	}
	
	if (check_address != offset + count)
		return ERROR_FLASH_DST_OUT_OF_BANK;
	
	/* multiple half words (2-byte) to be programmed? */
	if (words_remaining > 0) 
	{
		/* try using a block write */
		if ((retval = str9x_write_block(bank, buffer, offset, words_remaining)) != ERROR_OK)
		{
			if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			{
				/* if block write failed (no sufficient working area),
				 * we use normal (slow) single dword accesses */ 
				LOG_WARNING("couldn't use block writes, falling back to single memory accesses");
			}
			else if (retval == ERROR_FLASH_OPERATION_FAILED)
			{
				LOG_ERROR("flash writing failed with error code: 0x%x", retval);
				return ERROR_FLASH_OPERATION_FAILED;
			}
		}
		else
		{
			buffer += words_remaining * 2;
			address += words_remaining * 2;
			words_remaining = 0;
		}
	}

	while (words_remaining > 0)
	{
		bank_adr = address & ~0x03;
		
		/* write data command */
		target_write_u16(target, bank_adr, 0x40);
		target->type->write_memory(target, address, 2, 1, buffer + bytes_written);
		
		/* get status command */
		target_write_u16(target, bank_adr, 0x70);
	
		int timeout;
		for (timeout=0; timeout<1000; timeout++)
		{	
			target_read_u8(target, bank_adr, &status);
			if( status & 0x80 )
				break;
			alive_sleep(1);
		}
		if (timeout==1000)
		{
			LOG_ERROR("write timed out");
			return ERROR_FAIL;
		}
		
		/* clear status reg and read array */
		target_write_u16(target, bank_adr, 0x50);
		target_write_u16(target, bank_adr, 0xFF);
		
		if (status & 0x10)
			return ERROR_FLASH_OPERATION_FAILED;
		else if (status & 0x02)
			return ERROR_FLASH_OPERATION_FAILED;

		bytes_written += 2;
		words_remaining--;
		address += 2;
	}
	
	if (bytes_remaining)
	{
		u8 last_halfword[2] = {0xff, 0xff};
		int i = 0;
				
		while(bytes_remaining > 0)
		{
			last_halfword[i++] = *(buffer + bytes_written); 
			bytes_remaining--;
			bytes_written++;
		}
		
		bank_adr = address & ~0x03;
		
		/* write data comamnd */
		target_write_u16(target, bank_adr, 0x40);
		target->type->write_memory(target, address, 2, 1, last_halfword);
		
		/* query status command */
		target_write_u16(target, bank_adr, 0x70);
		
		int timeout;
		for (timeout=0; timeout<1000; timeout++)
		{	
			target_read_u8(target, bank_adr, &status);
			if( status & 0x80 )
				break;
			alive_sleep(1);
		}
		if (timeout==1000)
		{
			LOG_ERROR("write timed out");
			return ERROR_FAIL;
		}
		
		/* clear status reg and read array */
		target_write_u16(target, bank_adr, 0x50);
		target_write_u16(target, bank_adr, 0xFF);
		
		if (status & 0x10)
			return ERROR_FLASH_OPERATION_FAILED;
		else if (status & 0x02)
			return ERROR_FLASH_OPERATION_FAILED;
	}
		
	return ERROR_OK;
}

int str9x_probe(struct flash_bank_s *bank)
{
	return ERROR_OK;
}

int str9x_handle_part_id_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	return ERROR_OK;
}

int str9x_info(struct flash_bank_s *bank, char *buf, int buf_size)
{
	snprintf(buf, buf_size, "str9x flash driver info" );
	return ERROR_OK;
}

int str9x_handle_flash_config_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	str9x_flash_bank_t *str9x_info;
	flash_bank_t *bank;
	target_t *target = NULL;
	
	if (argc < 5)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	
	bank = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	if (!bank)
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
		return ERROR_OK;
	}
	
	str9x_info = bank->driver_priv;
	
	target = bank->target;
	
	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	/* config flash controller */
	target_write_u32(target, FLASH_BBSR, strtoul(args[1], NULL, 0));
	target_write_u32(target, FLASH_NBBSR, strtoul(args[2], NULL, 0));
	target_write_u32(target, FLASH_BBADR, (strtoul(args[3], NULL, 0) >> 2));
	target_write_u32(target, FLASH_NBBADR, (strtoul(args[4], NULL, 0) >> 2));

	/* set bit 18 instruction TCM order as per flash programming manual */
	arm966e_write_cp15(target, 62, 0x40000);
	
	/* enable flash bank 1 */
	target_write_u32(target, FLASH_CR, 0x18);
	return ERROR_OK;
}
