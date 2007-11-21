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

#include "stm32x.h"
#include "flash.h"
#include "target.h"
#include "log.h"
#include "armv7m.h"
#include "algorithm.h"
#include "binarybuffer.h"

#include <stdlib.h>
#include <string.h>

int stm32x_register_commands(struct command_context_s *cmd_ctx);
int stm32x_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank);
int stm32x_erase(struct flash_bank_s *bank, int first, int last);
int stm32x_protect(struct flash_bank_s *bank, int set, int first, int last);
int stm32x_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count);
int stm32x_probe(struct flash_bank_s *bank);
int stm32x_handle_part_id_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int stm32x_protect_check(struct flash_bank_s *bank);
int stm32x_erase_check(struct flash_bank_s *bank);
int stm32x_info(struct flash_bank_s *bank, char *buf, int buf_size);

int stm32x_handle_lock_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int stm32x_handle_unlock_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int stm32x_handle_options_read_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int stm32x_handle_options_write_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int stm32x_handle_mass_erase_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

flash_driver_t stm32x_flash =
{
	.name = "stm32x",
	.register_commands = stm32x_register_commands,
	.flash_bank_command = stm32x_flash_bank_command,
	.erase = stm32x_erase,
	.protect = stm32x_protect,
	.write = stm32x_write,
	.probe = stm32x_probe,
	.erase_check = stm32x_erase_check,
	.protect_check = stm32x_protect_check,
	.info = stm32x_info
};

int stm32x_register_commands(struct command_context_s *cmd_ctx)
{
	command_t *stm32x_cmd = register_command(cmd_ctx, NULL, "stm32x", NULL, COMMAND_ANY, "stm32x flash specific commands");
	
	register_command(cmd_ctx, stm32x_cmd, "lock", stm32x_handle_lock_command, COMMAND_EXEC,
					 "lock device");
	register_command(cmd_ctx, stm32x_cmd, "unlock", stm32x_handle_unlock_command, COMMAND_EXEC,
					 "unlock protected device");
	register_command(cmd_ctx, stm32x_cmd, "mass_erase", stm32x_handle_mass_erase_command, COMMAND_EXEC,
					 "mass erase device");
	register_command(cmd_ctx, stm32x_cmd, "options_read", stm32x_handle_options_read_command, COMMAND_EXEC,
					 "read device option bytes");
	register_command(cmd_ctx, stm32x_cmd, "options_write", stm32x_handle_options_write_command, COMMAND_EXEC,
					 "write device option bytes");
	return ERROR_OK;
}

int stm32x_build_block_list(struct flash_bank_s *bank)
{
	int i;
	int num_sectors = 0;
		
	switch (bank->size)
	{
		case 32 * 1024:
			num_sectors = 32;
			break;
		case 64 * 1024:
			num_sectors = 64;
			break;
		case 128 * 1024:
			num_sectors = 128;
			break;
		default:
			ERROR("BUG: unknown bank->size encountered");
			exit(-1);
	}
	
	bank->num_sectors = num_sectors;
	bank->sectors = malloc(sizeof(flash_sector_t) * num_sectors);
	
	for (i = 0; i < num_sectors; i++)
	{
		bank->sectors[i].offset = i * 1024;
		bank->sectors[i].size = 1024;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = 1;
	}
	
	return ERROR_OK;
}

/* flash bank stm32x <base> <size> 0 0 <target#>
 */
int stm32x_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank)
{
	stm32x_flash_bank_t *stm32x_info;
	
	if (argc < 6)
	{
		WARNING("incomplete flash_bank stm32x configuration");
		return ERROR_FLASH_BANK_INVALID;
	}
	
	stm32x_info = malloc(sizeof(stm32x_flash_bank_t));
	bank->driver_priv = stm32x_info;
	
	if (bank->base != 0x08000000)
	{
		WARNING("overriding flash base address for STM32x device with 0x08000000");
		bank->base = 0x08000000;
	}

	stm32x_build_block_list(bank);
	
	stm32x_info->write_algorithm = NULL;
	
	return ERROR_OK;
}

u32 stm32x_get_flash_status(flash_bank_t *bank)
{
	target_t *target = bank->target;
	u32 status;
	
	target_read_u32(target, STM32_FLASH_SR, &status);
	
	return status;
}

u32 stm32x_wait_status_busy(flash_bank_t *bank, int timeout)
{
	u32 status;
	
	/* wait for busy to clear */
	while (((status = stm32x_get_flash_status(bank)) & FLASH_BSY) && (timeout-- > 0))
	{
		DEBUG("status: 0x%x", status);
		usleep(1000);
	}
	
	return status;
}

int stm32x_read_options(struct flash_bank_s *bank)
{
	u32 optiondata;
	stm32x_flash_bank_t *stm32x_info = NULL;
	target_t *target = bank->target;
	
	stm32x_info = bank->driver_priv;
	
	/* read current option bytes */
	target_read_u32(target, STM32_FLASH_OBR, &optiondata);
	
	stm32x_info->option_bytes.user_options = (u16)0xFFF8|((optiondata >> 2) & 0x07);
	stm32x_info->option_bytes.RDP = (optiondata & (1 << OPT_READOUT)) ? 0xFFFF : 0x5AA5;
	
	if (optiondata & (1 << OPT_READOUT))
		INFO("Device Security Bit Set");
	
	/* each bit refers to a 4bank protection */
	target_read_u32(target, STM32_FLASH_WRPR, &optiondata);
	
	stm32x_info->option_bytes.protection[0] = (u16)optiondata;
	stm32x_info->option_bytes.protection[1] = (u16)(optiondata >> 8);
	stm32x_info->option_bytes.protection[2] = (u16)(optiondata >> 16);
	stm32x_info->option_bytes.protection[3] = (u16)(optiondata >> 24);
		
	return ERROR_OK;
}

int stm32x_erase_options(struct flash_bank_s *bank)
{
	stm32x_flash_bank_t *stm32x_info = NULL;
	target_t *target = bank->target;
	u32 status;
	
	stm32x_info = bank->driver_priv;
	
	/* read current options */
	stm32x_read_options(bank);
	
	/* unlock flash registers */
	target_write_u32(target, STM32_FLASH_KEYR, KEY1);
	target_write_u32(target, STM32_FLASH_KEYR, KEY2);
	
	/* unlock option flash registers */
	target_write_u32(target, STM32_FLASH_OPTKEYR, KEY1);
	target_write_u32(target, STM32_FLASH_OPTKEYR, KEY2);
	
	/* erase option bytes */
	target_write_u32(target, STM32_FLASH_CR, FLASH_OPTER|FLASH_OPTWRE);
	target_write_u32(target, STM32_FLASH_CR, FLASH_OPTER|FLASH_STRT|FLASH_OPTWRE);
	
	status = stm32x_wait_status_busy(bank, 10);
	
	if( status & FLASH_WRPRTERR )
		return ERROR_FLASH_OPERATION_FAILED;
	if( status & FLASH_PGERR )
		return ERROR_FLASH_OPERATION_FAILED;
	
	/* clear readout protection and complementary option bytes
	 * this will also force a device unlock if set */
	stm32x_info->option_bytes.RDP = 0x5AA5;
	
	return ERROR_OK;
}

int stm32x_write_options(struct flash_bank_s *bank)
{
	stm32x_flash_bank_t *stm32x_info = NULL;
	target_t *target = bank->target;
	u32 status;
	
	stm32x_info = bank->driver_priv;
	
	/* unlock flash registers */
	target_write_u32(target, STM32_FLASH_KEYR, KEY1);
	target_write_u32(target, STM32_FLASH_KEYR, KEY2);
	
	/* unlock option flash registers */
	target_write_u32(target, STM32_FLASH_OPTKEYR, KEY1);
	target_write_u32(target, STM32_FLASH_OPTKEYR, KEY2);
	
	/* program option bytes */
	target_write_u32(target, STM32_FLASH_CR, FLASH_OPTPG|FLASH_OPTWRE);
		
	/* write user option byte */
	target_write_u16(target, STM32_OB_USER, stm32x_info->option_bytes.user_options);
	
	status = stm32x_wait_status_busy(bank, 10);
	
	if( status & FLASH_WRPRTERR )
		return ERROR_FLASH_OPERATION_FAILED;
	if( status & FLASH_PGERR )
		return ERROR_FLASH_OPERATION_FAILED;
	
	/* write protection byte 1 */
	target_write_u16(target, STM32_OB_WRP0, stm32x_info->option_bytes.protection[0]);
	
	status = stm32x_wait_status_busy(bank, 10);
	
	if( status & FLASH_WRPRTERR )
		return ERROR_FLASH_OPERATION_FAILED;
	if( status & FLASH_PGERR )
		return ERROR_FLASH_OPERATION_FAILED;
	
	/* write protection byte 2 */
	target_write_u16(target, STM32_OB_WRP1, stm32x_info->option_bytes.protection[1]);
	
	status = stm32x_wait_status_busy(bank, 10);
	
	if( status & FLASH_WRPRTERR )
		return ERROR_FLASH_OPERATION_FAILED;
	if( status & FLASH_PGERR )
		return ERROR_FLASH_OPERATION_FAILED;
	
	/* write protection byte 3 */
	target_write_u16(target, STM32_OB_WRP2, stm32x_info->option_bytes.protection[2]);
	
	status = stm32x_wait_status_busy(bank, 10);
	
	if( status & FLASH_WRPRTERR )
		return ERROR_FLASH_OPERATION_FAILED;
	if( status & FLASH_PGERR )
		return ERROR_FLASH_OPERATION_FAILED;
	
	/* write protection byte 4 */
	target_write_u16(target, STM32_OB_WRP3, stm32x_info->option_bytes.protection[3]);
	
	status = stm32x_wait_status_busy(bank, 10);
	
	if( status & FLASH_WRPRTERR )
		return ERROR_FLASH_OPERATION_FAILED;
	if( status & FLASH_PGERR )
		return ERROR_FLASH_OPERATION_FAILED;
	
	/* write readout protection bit */
	target_write_u16(target, STM32_OB_RDP, stm32x_info->option_bytes.RDP);
	
	status = stm32x_wait_status_busy(bank, 10);
	
	if( status & FLASH_WRPRTERR )
		return ERROR_FLASH_OPERATION_FAILED;
	if( status & FLASH_PGERR )
		return ERROR_FLASH_OPERATION_FAILED;
	
	target_write_u32(target, STM32_FLASH_CR, FLASH_LOCK);
	
	return ERROR_OK;
}

int stm32x_blank_check(struct flash_bank_s *bank, int first, int last)
{
	target_t *target = bank->target;
	u8 *buffer;
	int i;
	int nBytes;
	
	if ((first < 0) || (last > bank->num_sectors))
		return ERROR_FLASH_SECTOR_INVALID;

	if (target->state != TARGET_HALTED)
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

int stm32x_protect_check(struct flash_bank_s *bank)
{
	target_t *target = bank->target;
	
	u32 protection;
	int i, s;
	int num_bits;

	if (target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}

	/* each bit refers to a 4bank protection */
	target_read_u32(target, STM32_FLASH_WRPR, &protection);
	
	/* each protection bit is for 4 1K pages */
	num_bits = (bank->num_sectors / 4);
	
	for (i = 0; i < num_bits; i++)
	{
		int set = 1;
		
		if( protection & (1 << i))
			set = 0;
		
		for (s = 0; s < 4; s++)
			bank->sectors[(i * 4) + s].is_protected = set;
	}

	return ERROR_OK;
}

int stm32x_erase(struct flash_bank_s *bank, int first, int last)
{
	target_t *target = bank->target;
	
	int i;
	u32 status;
	
	if (target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}
	
	/* unlock flash registers */
	target_write_u32(target, STM32_FLASH_KEYR, KEY1);
	target_write_u32(target, STM32_FLASH_KEYR, KEY2);
	
	for (i = first; i <= last; i++)
	{	
		target_write_u32(target, STM32_FLASH_CR, FLASH_PER);
		target_write_u32(target, STM32_FLASH_AR, bank->base + bank->sectors[i].offset);
		target_write_u32(target, STM32_FLASH_CR, FLASH_PER|FLASH_STRT);
		
		status = stm32x_wait_status_busy(bank, 10);
		
		if( status & FLASH_WRPRTERR )
			return ERROR_FLASH_OPERATION_FAILED;
		if( status & FLASH_PGERR )
			return ERROR_FLASH_OPERATION_FAILED;
		bank->sectors[i].is_erased = 1;
	}

	target_write_u32(target, STM32_FLASH_CR, FLASH_LOCK);
	
	return ERROR_OK;
}

int stm32x_protect(struct flash_bank_s *bank, int set, int first, int last)
{
	stm32x_flash_bank_t *stm32x_info = NULL;
	target_t *target = bank->target;
	u16 prot_reg[4] = {0xFFFF, 0xFFFF, 0xFFFF, 0xFFFF};
	int i, reg, bit;
	int status;
	u32 protection;
	
	stm32x_info = bank->driver_priv;
	
	if (target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if ((first && (first % 4)) || ((last + 1) && (last + 1) % 4))
	{
		WARNING("sector start/end incorrect - stm32 has 4K sector protection");
		return ERROR_FLASH_SECTOR_INVALID;
	}
	
	/* each bit refers to a 4bank protection */
	target_read_u32(target, STM32_FLASH_WRPR, &protection);
	
	prot_reg[0] = (u16)protection;
	prot_reg[1] = (u16)(protection >> 8);
	prot_reg[2] = (u16)(protection >> 16);
	prot_reg[3] = (u16)(protection >> 24);
	
	for (i = first; i <= last; i++)
	{
		reg = (i / 4) / 8;
		bit = (i / 4) - (reg * 8);
		
		if( set )
			prot_reg[reg] &= ~(1 << bit);
		else
			prot_reg[reg] |= (1 << bit);
	}
	
	if ((status = stm32x_erase_options(bank)) != ERROR_OK)
		return status;
	
	stm32x_info->option_bytes.protection[0] = prot_reg[0];
	stm32x_info->option_bytes.protection[1] = prot_reg[1];
	stm32x_info->option_bytes.protection[2] = prot_reg[2];
	stm32x_info->option_bytes.protection[3] = prot_reg[3];
	
	return stm32x_write_options(bank);
}

int stm32x_write_block(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count)
{
	stm32x_flash_bank_t *stm32x_info = bank->driver_priv;
	target_t *target = bank->target;
	u32 buffer_size = 8192;
	working_area_t *source;
	u32 address = bank->base + offset;
	reg_param_t reg_params[4];
	armv7m_algorithm_t armv7m_info;
	int retval = ERROR_OK;
	
	u8 stm32x_flash_write_code[] = {
									/* write: */
		0xDF, 0xF8, 0x24, 0x40,		/* ldr	r4, STM32_FLASH_CR */
		0x09, 0x4D,					/* ldr	r5, STM32_FLASH_SR */
		0x4F, 0xF0, 0x01, 0x03,		/* mov	r3, #1 */
		0x23, 0x60,					/* str	r3, [r4, #0] */
		0x30, 0xF8, 0x02, 0x3B,		/* ldrh r3, [r0], #2 */
		0x21, 0xF8, 0x02, 0x3B,		/* strh r3, [r1], #2 */
									/* busy: */
		0x2B, 0x68,					/* ldr 	r3, [r5, #0] */
		0x13, 0xF0, 0x01, 0x0F,		/* tst 	r3, #0x01 */
		0xFB, 0xD0,					/* beq 	busy */
		0x13, 0xF0, 0x14, 0x0F,		/* tst	r3, #0x14 */
		0x01, 0xD1,					/* bne	exit */
		0x01, 0x3A,					/* subs	r2, r2, #1 */
		0xED, 0xD1,					/* bne	write */
									/* exit: */
		0xFE, 0xE7,					/* b exit */		           	
		0x10, 0x20, 0x02, 0x40,		/* STM32_FLASH_CR:	.word 0x40022010 */
		0x0C, 0x20, 0x02, 0x40		/* STM32_FLASH_SR:	.word 0x4002200C */
	};
	
	/* flash write code */
	if (target_alloc_working_area(target, sizeof(stm32x_flash_write_code), &stm32x_info->write_algorithm) != ERROR_OK)
	{
		WARNING("no working area available, can't do block memory writes");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	};
	
	target_write_buffer(target, stm32x_info->write_algorithm->address, sizeof(stm32x_flash_write_code), stm32x_flash_write_code);

	/* memory buffer */
	while (target_alloc_working_area(target, buffer_size, &source) != ERROR_OK)
	{
		buffer_size /= 2;
		if (buffer_size <= 256)
		{
			/* if we already allocated the writing code, but failed to get a buffer, free the algorithm */
			if (stm32x_info->write_algorithm)
				target_free_working_area(target, stm32x_info->write_algorithm);
			
			WARNING("no large enough working area available, can't do block memory writes");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	};
	
	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARMV7M_MODE_ANY;
	armv7m_info.core_state = ARMV7M_STATE_THUMB;
	
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
		
		if ((retval = target->type->run_algorithm(target, 0, NULL, 4, reg_params, stm32x_info->write_algorithm->address, \
				stm32x_info->write_algorithm->address + (sizeof(stm32x_flash_write_code) - 10), 10000, &armv7m_info)) != ERROR_OK)
		{
			ERROR("error executing str7x flash write algorithm");
			break;
		}
		
		if (buf_get_u32(reg_params[3].value, 0, 32) & 0x14)
		{
			retval = ERROR_FLASH_OPERATION_FAILED;
			break;
		}
		
		buffer += thisrun_count * 2;
		address += thisrun_count * 2;
		count -= thisrun_count;
	}
	
	target_free_working_area(target, source);
	target_free_working_area(target, stm32x_info->write_algorithm);
	
	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	
	return retval;
}

int stm32x_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count)
{
	target_t *target = bank->target;
	u32 words_remaining = (count / 2);
	u32 bytes_remaining = (count & 0x00000001);
	u32 address = bank->base + offset;
	u32 bytes_written = 0;
	u8 status;
	u32 retval;
	
	if (target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (offset & 0x1)
	{
		WARNING("offset 0x%x breaks required 2-byte alignment", offset);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}
	
	/* unlock flash registers */
	target_write_u32(target, STM32_FLASH_KEYR, KEY1);
	target_write_u32(target, STM32_FLASH_KEYR, KEY2);
	
	/* multiple half words (2-byte) to be programmed? */
	if (words_remaining > 0) 
	{
		/* try using a block write */
		if ((retval = stm32x_write_block(bank, buffer, offset, words_remaining)) != ERROR_OK)
		{
			if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
			{
				/* if block write failed (no sufficient working area),
				 * we use normal (slow) single dword accesses */ 
				WARNING("couldn't use block writes, falling back to single memory accesses");
			}
			else if (retval == ERROR_FLASH_OPERATION_FAILED)
			{
				ERROR("flash writing failed with error code: 0x%x", retval);
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
		target_write_u32(target, STM32_FLASH_CR, FLASH_PG);
		target_write_u16(target, address, *(u16*)(buffer + bytes_written));
		
		status = stm32x_wait_status_busy(bank, 5);
		
		if( status & FLASH_WRPRTERR )
			return ERROR_FLASH_OPERATION_FAILED;
		if( status & FLASH_PGERR )
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
		
		target_write_u32(target, STM32_FLASH_CR, FLASH_PG);
		target_write_u16(target, address, *(u16*)last_halfword);
		
		status = stm32x_wait_status_busy(bank, 5);
		
		if( status & FLASH_WRPRTERR )
			return ERROR_FLASH_OPERATION_FAILED;
		if( status & FLASH_PGERR )
			return ERROR_FLASH_OPERATION_FAILED;
	}
	
	target_write_u32(target, STM32_FLASH_CR, FLASH_LOCK);
	
	return ERROR_OK;
}

int stm32x_probe(struct flash_bank_s *bank)
{
	return ERROR_OK;
}

int stm32x_handle_part_id_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	return ERROR_OK;
}

int stm32x_erase_check(struct flash_bank_s *bank)
{
	return stm32x_blank_check(bank, 0, bank->num_sectors - 1);
}

int stm32x_info(struct flash_bank_s *bank, char *buf, int buf_size)
{
	snprintf(buf, buf_size, "stm32x flash driver info" );
	return ERROR_OK;
}

int stm32x_handle_lock_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *bank;
	target_t *target = NULL;
	stm32x_flash_bank_t *stm32x_info = NULL;
	
	if (argc < 1)
	{
		command_print(cmd_ctx, "stm32x lock <bank>");
		return ERROR_OK;	
	}
	
	bank = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	if (!bank)
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
		return ERROR_OK;
	}
	
	stm32x_info = bank->driver_priv;
	
	target = bank->target;
	
	if (target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (stm32x_erase_options(bank) != ERROR_OK)
	{
		command_print(cmd_ctx, "stm32x failed to erase options");
		return ERROR_OK;
	}
		
	/* set readout protection */	
	stm32x_info->option_bytes.RDP = 0;
	
	if (stm32x_write_options(bank) != ERROR_OK)
	{
		command_print(cmd_ctx, "stm32x failed to lock device");
		return ERROR_OK;
	}
	
	command_print(cmd_ctx, "stm32x locked");
	
	return ERROR_OK;
}

int stm32x_handle_unlock_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *bank;
	target_t *target = NULL;
	stm32x_flash_bank_t *stm32x_info = NULL;
	
	if (argc < 1)
	{
		command_print(cmd_ctx, "stm32x unlock <bank>");
		return ERROR_OK;	
	}
	
	bank = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	if (!bank)
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
		return ERROR_OK;
	}
	
	stm32x_info = bank->driver_priv;
	
	target = bank->target;
	
	if (target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}
		
	if (stm32x_erase_options(bank) != ERROR_OK)
	{
		command_print(cmd_ctx, "stm32x failed to unlock device");
		return ERROR_OK;
	}
	
	if (stm32x_write_options(bank) != ERROR_OK)
	{
		command_print(cmd_ctx, "stm32x failed to lock device");
		return ERROR_OK;
	}
	
	command_print(cmd_ctx, "stm32x unlocked");
	
	return ERROR_OK;
}

int stm32x_handle_options_read_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *bank;
	u32 optionbyte;
	target_t *target = NULL;
	stm32x_flash_bank_t *stm32x_info = NULL;
	
	if (argc < 1)
	{
		command_print(cmd_ctx, "stm32x options_read <bank>");
		return ERROR_OK;	
	}
	
	bank = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	if (!bank)
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
		return ERROR_OK;
	}
	
	stm32x_info = bank->driver_priv;
	
	target = bank->target;
	
	if (target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}
	
	target_read_u32(target, STM32_FLASH_OBR, &optionbyte);
	command_print(cmd_ctx, "Option Byte: 0x%x", optionbyte);
	
	if (buf_get_u32((u8*)&optionbyte, OPT_ERROR, 1))
		command_print(cmd_ctx, "Option Byte Complement Error");
	
	if (buf_get_u32((u8*)&optionbyte, OPT_READOUT, 1))
		command_print(cmd_ctx, "Readout Protection On");
	else
		command_print(cmd_ctx, "Readout Protection Off");
	
	if (buf_get_u32((u8*)&optionbyte, OPT_RDWDGSW, 1))
		command_print(cmd_ctx, "Software Watchdog");
	else
		command_print(cmd_ctx, "Hardware Watchdog");
	
	if (buf_get_u32((u8*)&optionbyte, OPT_RDRSTSTOP, 1))
		command_print(cmd_ctx, "Stop: No reset generated");
	else
		command_print(cmd_ctx, "Stop: Reset generated");
	
	if (buf_get_u32((u8*)&optionbyte, OPT_RDRSTSTDBY, 1))
		command_print(cmd_ctx, "Standby: No reset generated");
	else
		command_print(cmd_ctx, "Standby: Reset generated");
	
	return ERROR_OK;
}

int stm32x_handle_options_write_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *bank;
	target_t *target = NULL;
	stm32x_flash_bank_t *stm32x_info = NULL;
	u16 optionbyte = 0xF8;
	
	if (argc < 4)
	{
		command_print(cmd_ctx, "stm32x options_write <bank> <SWWDG|HWWDG> <RSTSTNDBY|NORSTSTNDBY> <RSTSTOP|NORSTSTOP>");
		return ERROR_OK;	
	}
	
	bank = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	if (!bank)
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
		return ERROR_OK;
	}
	
	stm32x_info = bank->driver_priv;
	
	target = bank->target;
	
	if (target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (strcmp(args[1], "SWWDG") == 0)
	{
		optionbyte |= (1<<0);
	}
	else
	{
		optionbyte &= ~(1<<0);
	}
	
	if (strcmp(args[2], "NORSTSTNDBY") == 0)
	{
		optionbyte |= (1<<1);
	}
	else
	{
		optionbyte &= ~(1<<1);
	}
	
	if (strcmp(args[3], "NORSTSTOP") == 0)
	{
		optionbyte |= (1<<2);
	}
	else
	{
		optionbyte &= ~(1<<2);
	}
	
	if (stm32x_erase_options(bank) != ERROR_OK)
	{
		command_print(cmd_ctx, "stm32x failed to erase options");
		return ERROR_OK;
	}
	
	stm32x_info->option_bytes.user_options = optionbyte;
	
	if (stm32x_write_options(bank) != ERROR_OK)
	{
		command_print(cmd_ctx, "stm32x failed to write options");
		return ERROR_OK;
	}
	
	command_print(cmd_ctx, "stm32x write options complete");
	
	return ERROR_OK;
}

int stm32x_handle_mass_erase_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = NULL;
	stm32x_flash_bank_t *stm32x_info = NULL;
	flash_bank_t *bank;
	u32 status;
	
	if (argc < 1)
	{
		command_print(cmd_ctx, "stm32x mass_erase <bank>");
		return ERROR_OK;	
	}
	
	bank = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	if (!bank)
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
		return ERROR_OK;
	}
	
	stm32x_info = bank->driver_priv;
	
	target = bank->target;
	
	if (target->state != TARGET_HALTED)
	{
		return ERROR_TARGET_NOT_HALTED;
	}
	
	/* unlock option flash registers */
	target_write_u32(target, STM32_FLASH_KEYR, KEY1);
	target_write_u32(target, STM32_FLASH_KEYR, KEY2);
	
	/* mass erase flash memory */
	target_write_u32(target, STM32_FLASH_CR, FLASH_MER);
	target_write_u32(target, STM32_FLASH_CR, FLASH_MER|FLASH_STRT);
	
	status = stm32x_wait_status_busy(bank, 10);
	
	target_write_u32(target, STM32_FLASH_CR, FLASH_LOCK);
	
	if( status & FLASH_WRPRTERR )
	{
		command_print(cmd_ctx, "stm32x device protected");
		return ERROR_OK;
	}
	
	if( status & FLASH_PGERR )
	{
		command_print(cmd_ctx, "stm32x device programming failed");
		return ERROR_OK;
	}
	
	command_print(cmd_ctx, "stm32x mass erase complete");
	
	return ERROR_OK;
}
