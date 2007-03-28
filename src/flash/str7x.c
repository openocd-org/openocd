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
	{0x00002000, 0x02000, 0x02},
	{0x00004000, 0x02000, 0x04},
	{0x00006000, 0x02000, 0x08},
	{0x00008000, 0x08000, 0x10},
	{0x00010000, 0x10000, 0x20},
	{0x00020000, 0x10000, 0x40},
	{0x00030000, 0x10000, 0x80},
	{0x000C0000, 0x02000, 0x10000},
	{0x000C2000, 0x02000, 0x20000},
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
	return (bank->base | reg);
}

int str7x_build_block_list(struct flash_bank_s *bank)
{
	str7x_flash_bank_t *str7x_info = bank->driver_priv;

	int i;
	int num_sectors = 0, b0_sectors = 0, b1_sectors = 0;
		
	switch (bank->size)
	{
		case 16 * 1024:
			b0_sectors = 2;
			break;
		case 64 * 1024:
			b0_sectors = 5;
			break;
		case 128 * 1024:
			b0_sectors = 6;
			break;
		case 256 * 1024:
			b0_sectors = 8;
			break;
		default:
			ERROR("BUG: unknown bank->size encountered");
			exit(-1);
	}
	
	if( str7x_info->bank1 == 1 )
	{
		b1_sectors += 2;
	}
	
	num_sectors = b0_sectors + b1_sectors;
	
	bank->num_sectors = num_sectors;
	bank->sectors = malloc(sizeof(flash_sector_t) * num_sectors);
	str7x_info->sector_bits = malloc(sizeof(u32) * num_sectors);
	str7x_info->sector_bank = malloc(sizeof(u32) * num_sectors);
	
	num_sectors = 0;
	
	for (i = 0; i < b0_sectors; i++)
	{
		bank->sectors[num_sectors].offset = mem_layout[i].sector_start;
		bank->sectors[num_sectors].size = mem_layout[i].sector_size;
		bank->sectors[num_sectors].is_erased = -1;
		bank->sectors[num_sectors].is_protected = 1;
		str7x_info->sector_bank[num_sectors] = 0;
		str7x_info->sector_bits[num_sectors++] = mem_layout[i].sector_bit;
	}
	
	if (b1_sectors)
	{
		for (i = 8; i < 10; i++)
		{
			bank->sectors[num_sectors].offset = mem_layout[i].sector_start;
			bank->sectors[num_sectors].size = mem_layout[i].sector_size;
			bank->sectors[num_sectors].is_erased = -1;
			bank->sectors[num_sectors].is_protected = 1;
			str7x_info->sector_bank[num_sectors] = 1;
			str7x_info->sector_bits[num_sectors++] = mem_layout[i].sector_bit;
		}
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
		if (bank->base != 0x40000000)
		{
			WARNING("overriding flash base address for STR71x device with 0x40000000");
			bank->base = 0x40000000;
		}
	}
	else if (strcmp(args[5], "STR73x") == 0)
	{
		str7x_info->bank1 = 0;
		if (bank->base != 0x80000000)
		{
			WARNING("overriding flash base address for STR73x device with 0x80000000");
			bank->base = 0x80000000;
		}
	}
	else if (strcmp(args[5], "STR75x") == 0)
	{
		str7x_info->bank1 = 1;
		if (bank->base != 0x20000000)
		{
			WARNING("overriding flash base address for STR75x device with 0x20000000");
			bank->base = 0x20000000;
		}
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
	
	str7x_info->write_algorithm = NULL;
	
	return ERROR_OK;
}

u32 str7x_status(struct flash_bank_s *bank)
{
	str7x_flash_bank_t *str7x_info = bank->driver_priv;
	target_t *target = str7x_info->target;
	u32 retval;

	target_read_u32(target, str7x_get_flash_adr(bank, FLASH_CR0), &retval);

	return retval;
}

u32 str7x_result(struct flash_bank_s *bank)
{
	str7x_flash_bank_t *str7x_info = bank->driver_priv;
	target_t *target = str7x_info->target;
	u32 retval;

	target_read_u32(target, str7x_get_flash_adr(bank, FLASH_ER), &retval);
	
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
	u32 retval;

	if (str7x_info->target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}

	target_read_u32(target, str7x_get_flash_adr(bank, FLASH_NVWPAR), &retval);

	for (i = 0; i < bank->num_sectors; i++)
	{
		if (retval & str7x_info->sector_bits[i])
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
	u32 b0_sectors = 0, b1_sectors = 0;
	
	if (str7x_info->target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}
	
	for (i = first; i <= last; i++)
	{
		if (str7x_info->sector_bank[i] == 0)
			b0_sectors |= str7x_info->sector_bits[i];
		else if (str7x_info->sector_bank[i] == 1)
			b1_sectors |= str7x_info->sector_bits[i];
		else
			ERROR("BUG: str7x_info->sector_bank[i] neither 0 nor 1 (%i)", str7x_info->sector_bank[i]);
	}
	
	if (b0_sectors)
	{
		DEBUG("b0_sectors: 0x%x", b0_sectors);
		
		/* clear FLASH_ER register */	
		target_write_u32(target, str7x_get_flash_adr(bank, FLASH_ER), 0x0);
		
		cmd = FLASH_SER;
		target_write_u32(target, str7x_get_flash_adr(bank, FLASH_CR0), cmd);
		
		cmd = b0_sectors;
		target_write_u32(target, str7x_get_flash_adr(bank, FLASH_CR1), cmd);
		
		cmd = FLASH_SER|FLASH_WMS;
		target_write_u32(target, str7x_get_flash_adr(bank, FLASH_CR0), cmd);
		
		while (((retval = str7x_status(bank)) & (FLASH_BSYA1|FLASH_BSYA2))){
			usleep(1000);
		}
		
		retval = str7x_result(bank);
		
		if (retval)
		{
			ERROR("error erasing flash bank, FLASH_ER: 0x%x", retval);
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}
	
	if (b1_sectors)
	{
		DEBUG("b1_sectors: 0x%x", b1_sectors);
		
		/* clear FLASH_ER register */	
		target_write_u32(target, str7x_get_flash_adr(bank, FLASH_ER), 0x0);
		
		cmd = FLASH_SER;
		target_write_u32(target, str7x_get_flash_adr(bank, FLASH_CR0), cmd);
		
		cmd = b1_sectors;
		target_write_u32(target, str7x_get_flash_adr(bank, FLASH_CR1), cmd);
		
		cmd = FLASH_SER|FLASH_WMS;
		target_write_u32(target, str7x_get_flash_adr(bank, FLASH_CR0), cmd);
		
		while (((retval = str7x_status(bank)) & (FLASH_BSYA1|FLASH_BSYA2))){
			usleep(1000);
		}
		
		retval = str7x_result(bank);
		
		if (retval)
		{
			ERROR("error erasing flash bank, FLASH_ER: 0x%x", retval);
			return ERROR_FLASH_OPERATION_FAILED;
		}
	}
	
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

	if (set)
	{
		for (i = first; i <= last; i++)
			protect_blocks &= ~(str7x_info->sector_bits[i]);
	}
	
	/* clear FLASH_ER register */	
	target_write_u32(target, str7x_get_flash_adr(bank, FLASH_ER), 0x0);

	cmd = FLASH_SPR;
	target_write_u32(target, str7x_get_flash_adr(bank, FLASH_CR0), cmd);
	
	cmd = str7x_get_flash_adr(bank, FLASH_NVWPAR);
	target_write_u32(target, str7x_get_flash_adr(bank, FLASH_AR), cmd);
	
	cmd = protect_blocks;
	target_write_u32(target, str7x_get_flash_adr(bank, FLASH_DR0), cmd);
	
	cmd = FLASH_SPR|FLASH_WMS;
	target_write_u32(target, str7x_get_flash_adr(bank, FLASH_CR0), cmd);
	
	while (((retval = str7x_status(bank)) & (FLASH_BSYA1|FLASH_BSYA2))){
		usleep(1000);
	}
	
	retval = str7x_result(bank);
	
	DEBUG("retval: 0x%8.8x", retval);
	
	if (retval & FLASH_ERER)
		return ERROR_FLASH_SECTOR_NOT_ERASED;
	else if (retval & FLASH_WPF)
		return ERROR_FLASH_OPERATION_FAILED;

	return ERROR_OK;
}

int str7x_write_block(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count)
{
	str7x_flash_bank_t *str7x_info = bank->driver_priv;
	target_t *target = str7x_info->target;
	u32 buffer_size = 8192;
	working_area_t *source;
	u32 address = bank->base + offset;
	reg_param_t reg_params[5];
	armv4_5_algorithm_t armv4_5_info;
	int retval = ERROR_OK;
	
	u32 str7x_flash_write_code[] = {
					/* write:				*/
		0xe3a04201, /*	mov r4, #0x10000000	*/
		0xe5824000, /*	str r4, [r2, #0x0]	*/
		0xe5821010, /*	str r1, [r2, #0x10]	*/
		0xe4904004, /*	ldr r4, [r0], #4	*/
		0xe5824008, /*	str r4, [r2, #0x8]	*/
		0xe4904004, /*	ldr r4, [r0], #4	*/
		0xe582400c, /*	str r4, [r2, #0xc]	*/
		0xe3a04209, /*	mov r4, #0x90000000	*/
		0xe5824000, /*	str r4, [r2, #0x0]	*/
		            /* busy:				*/
		0xe5924000, /*	ldr r4, [r2, #0x0]	*/
		0xe3140016, /*	tst r4, #0x16		*/
		0x1afffffc, /*	bne busy			*/
		0xe5924014, /*	ldr r4, [r2, #0x14]	*/
		0xe31400ff, /*	tst r4, #0xff		*/
		0x03140c01, /*	tsteq r4, #0x100	*/
		0x1a000002, /*	bne exit			*/
		0xe2811008, /*	add r1, r1, #0x8	*/
		0xe2533001, /*	subs r3, r3, #1		*/
		0x1affffec, /*	bne write			*/
					/* exit:				*/
		0xeafffffe, /*	b exit				*/
	};
	
	u8 str7x_flash_write_code_buf[80];
	int i;
	
	/* flash write code */
	if (!str7x_info->write_algorithm)
	{
		if (target_alloc_working_area(target, 4 * 20, &str7x_info->write_algorithm) != ERROR_OK)
		{
			WARNING("no working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		};

		/* convert flash writing code into a buffer in target endianness */
		for (i = 0; i < 20; i++)
			target_buffer_set_u32(target, str7x_flash_write_code_buf + i*4, str7x_flash_write_code[i]);
			
		target_write_buffer(target, str7x_info->write_algorithm->address, 20 * 4, str7x_flash_write_code_buf);
	}

	/* memory buffer */
	while (target_alloc_working_area(target, buffer_size, &source) != ERROR_OK)
	{
		buffer_size /= 2;
		if (buffer_size <= 256)
		{
			/* if we already allocated the writing code, but failed to get a buffer, free the algorithm */
			if (str7x_info->write_algorithm)
				target_free_working_area(target, str7x_info->write_algorithm);
			
			WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	};
	
	armv4_5_info.common_magic = ARMV4_5_COMMON_MAGIC;
	armv4_5_info.core_mode = ARMV4_5_MODE_SVC;
	armv4_5_info.core_state = ARMV4_5_STATE_ARM;
	
	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);
	init_reg_param(&reg_params[4], "r4", 32, PARAM_IN);
	
	while (count > 0)
	{
		u32 thisrun_count = (count > (buffer_size / 8)) ? (buffer_size / 8) : count;
		
		target_write_buffer(target, source->address, thisrun_count * 8, buffer);
		
		buf_set_u32(reg_params[0].value, 0, 32, source->address);
		buf_set_u32(reg_params[1].value, 0, 32, address);
		buf_set_u32(reg_params[2].value, 0, 32, str7x_get_flash_adr(bank, FLASH_CR0));
		buf_set_u32(reg_params[3].value, 0, 32, thisrun_count);
	
		if ((retval = target->type->run_algorithm(target, 0, NULL, 5, reg_params, str7x_info->write_algorithm->address, str7x_info->write_algorithm->address + (19 * 4), 10000, &armv4_5_info)) != ERROR_OK)
		{
			ERROR("error executing str7x flash write algorithm");
			break;
		}
	
		if (buf_get_u32(reg_params[4].value, 0, 32) != 0x00)
		{
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}
		
		buffer += thisrun_count * 8;
		address += thisrun_count * 8;
		count -= thisrun_count;
	}
	
	target_free_working_area(target, source);
	
	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	
	return retval;
}

int str7x_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count)
{
	str7x_flash_bank_t *str7x_info = bank->driver_priv;
	target_t *target = str7x_info->target;
	u32 dwords_remaining = (count / 8);
	u32 bytes_remaining = (count & 0x00000007);
	u32 address = bank->base + offset;
	u32 bytes_written = 0;
	u32 cmd;
	u32 retval;
	u32 check_address = offset;
	int i;
	
	if (str7x_info->target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (offset & 0x7)
	{
		WARNING("offset 0x%x breaks required 8-byte alignment", offset);
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
		
	/* clear FLASH_ER register */	
	target_write_u32(target, str7x_get_flash_adr(bank, FLASH_ER), 0x0);

	/* multiple dwords (8-byte) to be programmed? */
	if (dwords_remaining > 0) 
	{
		/* try using a block write */
		if ((retval = str7x_write_block(bank, buffer, offset, dwords_remaining)) != ERROR_OK)
		{
			if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			{
				/* if block write failed (no sufficient working area),
				 * we use normal (slow) single dword accesses */ 
				WARNING("couldn't use block writes, falling back to single memory accesses");
			}
			else if (retval == ERROR_FLASH_OPERATION_FAILED)
			{
				/* if an error occured, we examine the reason, and quit */
				retval = str7x_result(bank);
				
				ERROR("flash writing failed with error code: 0x%x", retval);
				return ERROR_FLASH_OPERATION_FAILED;
			}
		}
		else
		{
			buffer += dwords_remaining * 8;
			address += dwords_remaining * 8;
			dwords_remaining = 0;
		}
	}

	while (dwords_remaining > 0)
	{
		// command
		cmd = FLASH_DWPG;
		target_write_u32(target, str7x_get_flash_adr(bank, FLASH_CR0), cmd);
		
		// address
		target_write_u32(target, str7x_get_flash_adr(bank, FLASH_AR), address);
		
		// data word 1
		target->type->write_memory(target, str7x_get_flash_adr(bank, FLASH_DR0), 4, 1, buffer + bytes_written);
		bytes_written += 4;
		
		// data word 2
		target->type->write_memory(target, str7x_get_flash_adr(bank, FLASH_DR1), 4, 1, buffer + bytes_written);
		bytes_written += 4;
		
		/* start programming cycle */
		cmd = FLASH_DWPG | FLASH_WMS;
		target_write_u32(target, str7x_get_flash_adr(bank, FLASH_CR0), cmd);
		
		while (((retval = str7x_status(bank)) & (FLASH_BSYA1 | FLASH_BSYA2)))
		{
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
	
	if (bytes_remaining)
	{
		u8 last_dword[8] = {0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff};
		int i = 0;
				
		while(bytes_remaining > 0)
		{
			last_dword[i++] = *(buffer + bytes_written); 
			bytes_remaining--;
			bytes_written++;
		}
		
		// command
		cmd = FLASH_DWPG;
		target_write_u32(target, str7x_get_flash_adr(bank, FLASH_CR0), cmd);
		
		// address
		target_write_u32(target, str7x_get_flash_adr(bank, FLASH_AR), address);
		
		// data word 1
		target->type->write_memory(target, str7x_get_flash_adr(bank, FLASH_DR0), 4, 1, last_dword);
		bytes_written += 4;
		
		// data word 2
		target->type->write_memory(target, str7x_get_flash_adr(bank, FLASH_DR1), 4, 1, last_dword + 4);
		bytes_written += 4;
		
		/* start programming cycle */
		cmd = FLASH_DWPG | FLASH_WMS;
		target_write_u32(target, str7x_get_flash_adr(bank, FLASH_CR0), cmd);
		
		while (((retval = str7x_status(bank)) & (FLASH_BSYA1 | FLASH_BSYA2)))
		{
			usleep(1000);
		}
		
		retval = str7x_result(bank);
		
		if (retval & FLASH_PGER)
			return ERROR_FLASH_OPERATION_FAILED;
		else if (retval & FLASH_WPF)
			return ERROR_FLASH_OPERATION_FAILED;
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
