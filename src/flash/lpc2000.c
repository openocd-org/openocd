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

#include "lpc2000.h"

#include "flash.h"
#include "target.h"
#include "log.h"
#include "armv4_5.h"
#include "algorithm.h"
#include "binarybuffer.h"

#include <stdlib.h>
#include <string.h>

/* flash programming support for Philips LPC2xxx devices
 * currently supported devices:
 * variant 1 (lpc2000_v1):
 * - 2104|5|6
 * - 2114|9
 * - 2124|9
 * - 2194
 * - 2212|4
 * - 2292|4
 *
 * variant 2 (lpc2000_v2):
 * - 213x
 * - 214x
 * - 2101|2|3
 * - 2364|6|8
 * - 2378
 */

int lpc2000_register_commands(struct command_context_s *cmd_ctx);
int lpc2000_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank);
int lpc2000_erase(struct flash_bank_s *bank, int first, int last);
int lpc2000_protect(struct flash_bank_s *bank, int set, int first, int last);
int lpc2000_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count);
int lpc2000_probe(struct flash_bank_s *bank);
int lpc2000_erase_check(struct flash_bank_s *bank);
int lpc2000_protect_check(struct flash_bank_s *bank);
int lpc2000_info(struct flash_bank_s *bank, char *buf, int buf_size);
	
int lpc2000_handle_part_id_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

flash_driver_t lpc2000_flash =
{
	.name = "lpc2000",
	.register_commands = lpc2000_register_commands,
	.flash_bank_command = lpc2000_flash_bank_command,
	.erase = lpc2000_erase,
	.protect = lpc2000_protect,
	.write = lpc2000_write,
	.probe = lpc2000_probe,
	.auto_probe = lpc2000_probe,
	.erase_check = lpc2000_erase_check,
	.protect_check = lpc2000_protect_check,
	.info = lpc2000_info
};

int lpc2000_register_commands(struct command_context_s *cmd_ctx)
{
	command_t *lpc2000_cmd = register_command(cmd_ctx, NULL, "lpc2000", NULL, COMMAND_ANY, NULL);
	
	register_command(cmd_ctx, lpc2000_cmd, "part_id", lpc2000_handle_part_id_command, COMMAND_EXEC,
					 "print part id of lpc2000 flash bank <num>");
	
	return ERROR_OK;
}

int lpc2000_build_sector_list(struct flash_bank_s *bank)
{
	lpc2000_flash_bank_t *lpc2000_info = bank->driver_priv;
	
	/* default to a 4096 write buffer */
	lpc2000_info->cmd51_max_buffer = 4096;
	
	if (lpc2000_info->variant == 1)
	{
		int i = 0;
		u32 offset = 0;
		
		/* variant 1 has different layout for 128kb and 256kb flashes */
		if (bank->size == 128 * 1024)
		{
			bank->num_sectors = 16;
			bank->sectors = malloc(sizeof(flash_sector_t) * 16);
			for (i = 0; i < 16; i++)
			{
				bank->sectors[i].offset = offset;
				bank->sectors[i].size = 8 * 1024;
				offset += bank->sectors[i].size;
				bank->sectors[i].is_erased = -1;
				bank->sectors[i].is_protected = 1;
			}
		}
		else if (bank->size == 256 * 1024)
		{
			bank->num_sectors = 18;
			bank->sectors = malloc(sizeof(flash_sector_t) * 18);
			
			for (i = 0; i < 8; i++)
			{
				bank->sectors[i].offset = offset;
				bank->sectors[i].size = 8 * 1024;
				offset += bank->sectors[i].size;
				bank->sectors[i].is_erased = -1;
				bank->sectors[i].is_protected = 1;
			}
			for (i = 8; i < 10; i++)
			{
				bank->sectors[i].offset = offset;
				bank->sectors[i].size = 64 * 1024;
				offset += bank->sectors[i].size;
				bank->sectors[i].is_erased = -1;
				bank->sectors[i].is_protected = 1;
			}
			for (i = 10; i < 18; i++)
			{
				bank->sectors[i].offset = offset;
				bank->sectors[i].size = 8 * 1024;
				offset += bank->sectors[i].size;
				bank->sectors[i].is_erased = -1;
				bank->sectors[i].is_protected = 1;
			}
		}
		else
		{
			LOG_ERROR("BUG: unknown bank->size encountered");
			exit(-1);
		}
	}
	else if (lpc2000_info->variant == 2)
	{
		int num_sectors;
		int i;
		u32 offset = 0;
	
		/* variant 2 has a uniform layout, only number of sectors differs */
		switch (bank->size)
		{
			case 4 * 1024:
				lpc2000_info->cmd51_max_buffer = 1024;
				num_sectors = 1;
				break;
			case 8 * 1024:
				lpc2000_info->cmd51_max_buffer = 1024;
				num_sectors = 2;
				break;
			case 16 * 1024:
				num_sectors = 4;
				break;
			case 32 * 1024:
				num_sectors = 8;
				break;
			case 64 * 1024:
				num_sectors = 9;
				break;
			case 128 * 1024:
				num_sectors = 11;
				break;
			case 256 * 1024:
				num_sectors = 15;
				break;
			case 512 * 1024:
			case 500 * 1024:
				num_sectors = 27;
				break;
			default:
				LOG_ERROR("BUG: unknown bank->size encountered");
				exit(-1);
				break;
		}
		
		bank->num_sectors = num_sectors;
		bank->sectors = malloc(sizeof(flash_sector_t) * num_sectors);
		
		for (i = 0; i < num_sectors; i++)
		{
			if ((i >= 0) && (i < 8))
			{
				bank->sectors[i].offset = offset;
				bank->sectors[i].size = 4 * 1024;
				offset += bank->sectors[i].size;
				bank->sectors[i].is_erased = -1;
				bank->sectors[i].is_protected = 1;
			}
			if ((i >= 8) && (i < 22))
			{
				bank->sectors[i].offset = offset;
				bank->sectors[i].size = 32 * 1024;
				offset += bank->sectors[i].size;
				bank->sectors[i].is_erased = -1;
				bank->sectors[i].is_protected = 1;
			}
			if ((i >= 22) && (i < 27))
			{
				bank->sectors[i].offset = offset;
				bank->sectors[i].size = 4 * 1024;
				offset += bank->sectors[i].size;
				bank->sectors[i].is_erased = -1;
				bank->sectors[i].is_protected = 1;
			}
		}
	}
	else
	{
		LOG_ERROR("BUG: unknown lpc2000_info->variant encountered");
		exit(-1);
	}
	
	return ERROR_OK;
}

/* call LPC2000 IAP function
 * uses 172 bytes working area
 * 0x0 to 0x7: jump gate (BX to thumb state, b -2 to wait)
 * 0x8 to 0x1f: command parameter table
 * 0x20 to 0x2b: command result table
 * 0x2c to 0xac: stack (only 128b needed)
 */
int lpc2000_iap_call(flash_bank_t *bank, int code, u32 param_table[5], u32 result_table[2])
{
	int retval;
	lpc2000_flash_bank_t *lpc2000_info = bank->driver_priv;
	target_t *target = bank->target;
	mem_param_t mem_params[2];
	reg_param_t reg_params[5];
	armv4_5_algorithm_t armv4_5_info;
	u32 status_code;
	
	/* regrab previously allocated working_area, or allocate a new one */
	if (!lpc2000_info->iap_working_area)
	{
		u8 jump_gate[8];
		
		/* make sure we have a working area */
		if (target_alloc_working_area(target, 172, &lpc2000_info->iap_working_area) != ERROR_OK)
		{
			LOG_ERROR("no working area specified, can't write LPC2000 internal flash");
			return ERROR_FLASH_OPERATION_FAILED;
		}
		
		/* write IAP code to working area */
		target_buffer_set_u32(target, jump_gate, ARMV4_5_BX(12));
		target_buffer_set_u32(target, jump_gate + 4, ARMV4_5_B(0xfffffe, 0));
		if((retval = target->type->write_memory(target, lpc2000_info->iap_working_area->address, 4, 2, jump_gate)) != ERROR_OK)
		{
			return retval;
		}
	}
	
	armv4_5_info.common_magic = ARMV4_5_COMMON_MAGIC;
	armv4_5_info.core_mode = ARMV4_5_MODE_SVC;
	armv4_5_info.core_state = ARMV4_5_STATE_ARM;
	
	/* command parameter table */
	init_mem_param(&mem_params[0], lpc2000_info->iap_working_area->address + 8, 4 * 6, PARAM_OUT);
	target_buffer_set_u32(target, mem_params[0].value, code);
	target_buffer_set_u32(target, mem_params[0].value + 0x4, param_table[0]);
	target_buffer_set_u32(target, mem_params[0].value + 0x8, param_table[1]);
	target_buffer_set_u32(target, mem_params[0].value + 0xc, param_table[2]);
	target_buffer_set_u32(target, mem_params[0].value + 0x10, param_table[3]);
	target_buffer_set_u32(target, mem_params[0].value + 0x14, param_table[4]);
	
	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	buf_set_u32(reg_params[0].value, 0, 32, lpc2000_info->iap_working_area->address + 0x8);
	
	/* command result table */
	init_mem_param(&mem_params[1], lpc2000_info->iap_working_area->address + 0x20, 4 * 3, PARAM_IN);
	
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	buf_set_u32(reg_params[1].value, 0, 32, lpc2000_info->iap_working_area->address + 0x20);
	
	/* IAP entry point */
	init_reg_param(&reg_params[2], "r12", 32, PARAM_OUT);
	buf_set_u32(reg_params[2].value, 0, 32, 0x7ffffff1);
	
	/* IAP stack */
	init_reg_param(&reg_params[3], "r13_svc", 32, PARAM_OUT);
	buf_set_u32(reg_params[3].value, 0, 32, lpc2000_info->iap_working_area->address + 0xac);

	/* return address */
	init_reg_param(&reg_params[4], "lr_svc", 32, PARAM_OUT);
	buf_set_u32(reg_params[4].value, 0, 32, lpc2000_info->iap_working_area->address + 0x4);
	
	target->type->run_algorithm(target, 2, mem_params, 5, reg_params, lpc2000_info->iap_working_area->address, lpc2000_info->iap_working_area->address + 0x4, 10000, &armv4_5_info);
	
	status_code = buf_get_u32(mem_params[1].value, 0, 32);
	result_table[0] = target_buffer_get_u32(target, mem_params[1].value);
	result_table[1] = target_buffer_get_u32(target, mem_params[1].value + 4);
	
	destroy_mem_param(&mem_params[0]);
	destroy_mem_param(&mem_params[1]);
	
	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);
	
	return status_code;
}

int lpc2000_iap_blank_check(struct flash_bank_s *bank, int first, int last)
{
	u32 param_table[5];
	u32 result_table[2];
	int status_code;
	int i;
	
	if ((first < 0) || (last > bank->num_sectors))
		return ERROR_FLASH_SECTOR_INVALID;
	
	for (i = first; i <= last; i++)
	{
		/* check single sector */
		param_table[0] = param_table[1] = i;
		status_code = lpc2000_iap_call(bank, 53, param_table, result_table);
		
		switch (status_code)
		{
			case ERROR_FLASH_OPERATION_FAILED:
				return ERROR_FLASH_OPERATION_FAILED;
			case LPC2000_CMD_SUCCESS:
				bank->sectors[i].is_erased = 1;
				break;
			case LPC2000_SECTOR_NOT_BLANK:
				bank->sectors[i].is_erased = 0;
				break;
			case LPC2000_INVALID_SECTOR:
				bank->sectors[i].is_erased = 0;
				break;
			case LPC2000_BUSY:
				return ERROR_FLASH_BUSY;
				break;
			default:
				LOG_ERROR("BUG: unknown LPC2000 status code");
				exit(-1);
		}
	}
	
	return ERROR_OK;
}

/* flash bank lpc2000 <base> <size> 0 0 <target#> <lpc_variant> <cclk> [calc_checksum]
 */
int lpc2000_flash_bank_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct flash_bank_s *bank)
{
	lpc2000_flash_bank_t *lpc2000_info;
	
	if (argc < 8)
	{
		LOG_WARNING("incomplete flash_bank lpc2000 configuration");
		return ERROR_FLASH_BANK_INVALID;
	}
	
	lpc2000_info = malloc(sizeof(lpc2000_flash_bank_t));
	bank->driver_priv = lpc2000_info;
	
	if (strcmp(args[6], "lpc2000_v1") == 0)
	{
		lpc2000_info->variant = 1;
		lpc2000_info->cmd51_dst_boundary = 512;
		lpc2000_info->cmd51_can_256b = 0;
		lpc2000_info->cmd51_can_8192b = 1;
	}
	else if (strcmp(args[6], "lpc2000_v2") == 0)
	{
		lpc2000_info->variant = 2;
		lpc2000_info->cmd51_dst_boundary = 256;
		lpc2000_info->cmd51_can_256b = 1;
		lpc2000_info->cmd51_can_8192b = 0;
	}
	else
	{
		LOG_ERROR("unknown LPC2000 variant");
		free(lpc2000_info);
		return ERROR_FLASH_BANK_INVALID;
	}
	
	lpc2000_info->iap_working_area = NULL;
	lpc2000_info->cclk = strtoul(args[7], NULL, 0);
	lpc2000_info->calc_checksum = 0;
	lpc2000_build_sector_list(bank);
		
	if (argc >= 9)
	{
		if (strcmp(args[8], "calc_checksum") == 0)
			lpc2000_info->calc_checksum = 1;
	}
	
	return ERROR_OK;
}

int lpc2000_erase(struct flash_bank_s *bank, int first, int last)
{
	lpc2000_flash_bank_t *lpc2000_info = bank->driver_priv;
	u32 param_table[5];
	u32 result_table[2];
	int status_code;
	
	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	param_table[0] = first;
	param_table[1] = last;
	param_table[2] = lpc2000_info->cclk;
	
	/* Prepare sectors */
	status_code = lpc2000_iap_call(bank, 50, param_table, result_table);
	switch (status_code)
	{
		case ERROR_FLASH_OPERATION_FAILED:
			return ERROR_FLASH_OPERATION_FAILED;
		case LPC2000_CMD_SUCCESS:
			break;
		case LPC2000_INVALID_SECTOR:
			return ERROR_FLASH_SECTOR_INVALID;
			break;
		default:
			LOG_WARNING("lpc2000 prepare sectors returned %i", status_code);
			return ERROR_FLASH_OPERATION_FAILED;
	}
	
	/* Erase sectors */
	status_code = lpc2000_iap_call(bank, 52, param_table, result_table);
	switch (status_code)
	{
		case ERROR_FLASH_OPERATION_FAILED:
			return ERROR_FLASH_OPERATION_FAILED;
		case LPC2000_CMD_SUCCESS:
			break;
		case LPC2000_INVALID_SECTOR:
			return ERROR_FLASH_SECTOR_INVALID;
			break;
		default:
			LOG_WARNING("lpc2000 erase sectors returned %i", status_code);
			return ERROR_FLASH_OPERATION_FAILED;
	}
	
	return ERROR_OK;
}

int lpc2000_protect(struct flash_bank_s *bank, int set, int first, int last)
{
	/* can't protect/unprotect on the lpc2000 */
	return ERROR_OK;
}

int lpc2000_write(struct flash_bank_s *bank, u8 *buffer, u32 offset, u32 count)
{
	lpc2000_flash_bank_t *lpc2000_info = bank->driver_priv;
	target_t *target = bank->target;
	u32 dst_min_alignment;
	u32 bytes_remaining = count;
	u32 bytes_written = 0;
	int first_sector = 0;
	int last_sector = 0;
	u32 param_table[5];
	u32 result_table[2];
	int status_code;
	int i;
	working_area_t *download_area;
	int retval = ERROR_OK;
		 
	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if (offset + count > bank->size)
		return ERROR_FLASH_DST_OUT_OF_BANK;
		
	if (lpc2000_info->cmd51_can_256b)
		dst_min_alignment = 256;
	else
		dst_min_alignment = 512;
	
	if (offset % dst_min_alignment)
	{
		LOG_WARNING("offset 0x%x breaks required alignment 0x%x", offset, dst_min_alignment);
		return ERROR_FLASH_DST_BREAKS_ALIGNMENT;
	}
	
	for (i = 0; i < bank->num_sectors; i++)
	{
		if (offset >= bank->sectors[i].offset)
			first_sector = i;
		if (offset + CEIL(count, dst_min_alignment) * dst_min_alignment > bank->sectors[i].offset)
			last_sector = i;
	}
	
	LOG_DEBUG("first_sector: %i, last_sector: %i", first_sector, last_sector);

	/* check if exception vectors should be flashed */
	if ((offset == 0) && (count >= 0x20) && lpc2000_info->calc_checksum)
	{
		u32 checksum = 0;
		int i = 0;
		for (i = 0; i < 8; i++)
		{
			LOG_DEBUG("0x%2.2x: 0x%8.8x", i * 4, buf_get_u32(buffer + (i * 4), 0, 32));
			if (i != 5)
				checksum += buf_get_u32(buffer + (i * 4), 0, 32);
		}
		checksum = 0 - checksum;
		LOG_DEBUG("checksum: 0x%8.8x", checksum);
		buf_set_u32(buffer + 0x14, 0, 32, checksum);
	}
	
	/* allocate a working area */
	if (target_alloc_working_area(target, lpc2000_info->cmd51_max_buffer, &download_area) != ERROR_OK)
	{
		LOG_ERROR("no working area specified, can't write LPC2000 internal flash");
		return ERROR_FLASH_OPERATION_FAILED;
	}
	
	while (bytes_remaining > 0)
	{
		u32 thisrun_bytes;
		if (bytes_remaining >= lpc2000_info->cmd51_max_buffer)
			thisrun_bytes = lpc2000_info->cmd51_max_buffer;
		else if (bytes_remaining >= 1024)
			thisrun_bytes = 1024;
		else if ((bytes_remaining >= 512) || (!lpc2000_info->cmd51_can_256b))
			thisrun_bytes = 512;
		else
			thisrun_bytes = 256;
		
		/* Prepare sectors */
		param_table[0] = first_sector;
		param_table[1] = last_sector;
		status_code = lpc2000_iap_call(bank, 50, param_table, result_table);
		switch (status_code)
		{
			case ERROR_FLASH_OPERATION_FAILED:
				retval = ERROR_FLASH_OPERATION_FAILED;
				break;
			case LPC2000_CMD_SUCCESS:
				break;
			case LPC2000_INVALID_SECTOR:
				retval = ERROR_FLASH_SECTOR_INVALID;
				break;
			default:
				LOG_WARNING("lpc2000 prepare sectors returned %i", status_code);
				retval = ERROR_FLASH_OPERATION_FAILED;
				break;
		}

		/* Exit if error occured */
		if (retval != ERROR_OK)
			break;
		
		if (bytes_remaining >= thisrun_bytes)
		{
			if ((retval = target_write_buffer(bank->target, download_area->address, thisrun_bytes, buffer + bytes_written)) != ERROR_OK)
			{
				retval = ERROR_FLASH_OPERATION_FAILED;
				break;
			}
		}
		else
		{
			u8 *last_buffer = malloc(thisrun_bytes);
			int i;
			memcpy(last_buffer, buffer + bytes_written, bytes_remaining);
			for (i = bytes_remaining; i < thisrun_bytes; i++)
				last_buffer[i] = 0xff;
			target_write_buffer(bank->target, download_area->address, thisrun_bytes, last_buffer);
			free(last_buffer);
		}
		
		LOG_DEBUG("writing 0x%x bytes to address 0x%x", thisrun_bytes, bank->base + offset + bytes_written);
		
		/* Write data */
		param_table[0] = bank->base + offset + bytes_written;
		param_table[1] = download_area->address;
		param_table[2] = thisrun_bytes;
		param_table[3] = lpc2000_info->cclk;
		status_code = lpc2000_iap_call(bank, 51, param_table, result_table);
		switch (status_code)
		{
			case ERROR_FLASH_OPERATION_FAILED:
				retval = ERROR_FLASH_OPERATION_FAILED;
				break;
			case LPC2000_CMD_SUCCESS:
				break;
			case LPC2000_INVALID_SECTOR:
				retval = ERROR_FLASH_SECTOR_INVALID;
				break;
			default:
				LOG_WARNING("lpc2000 returned %i", status_code);
				retval = ERROR_FLASH_OPERATION_FAILED;
				break;
		}
		
		/* Exit if error occured */
		if (retval != ERROR_OK)
			break;
		
		if (bytes_remaining > thisrun_bytes)
			bytes_remaining -= thisrun_bytes;
		else
			bytes_remaining = 0;
		bytes_written += thisrun_bytes;
	}
	
	target_free_working_area(target, download_area);
	
	return retval;
}

int lpc2000_probe(struct flash_bank_s *bank)
{
	/* we can't probe on an lpc2000 
	 * if this is an lpc2xxx, it has the configured flash
	 */
	return ERROR_OK;
}

int lpc2000_erase_check(struct flash_bank_s *bank)
{
	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	return lpc2000_iap_blank_check(bank, 0, bank->num_sectors - 1);
}

int lpc2000_protect_check(struct flash_bank_s *bank)
{
	/* sectors are always protected	*/
	return ERROR_OK;
}

int lpc2000_info(struct flash_bank_s *bank, char *buf, int buf_size)
{
	lpc2000_flash_bank_t *lpc2000_info = bank->driver_priv;

	snprintf(buf, buf_size, "lpc2000 flash driver variant: %i, clk: %i", lpc2000_info->variant, lpc2000_info->cclk);
	
	return ERROR_OK;
}

int lpc2000_handle_part_id_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	flash_bank_t *bank;
	u32 param_table[5];
	u32 result_table[2];
	int status_code;

	if (argc < 1)
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	
	bank = get_flash_bank_by_num(strtoul(args[0], NULL, 0));
	if (!bank)
	{
		command_print(cmd_ctx, "flash bank '#%s' is out of bounds", args[0]);
		return ERROR_OK;
	}

	if (bank->target->state != TARGET_HALTED)
	{
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	
	if ((status_code = lpc2000_iap_call(bank, 54, param_table, result_table)) != 0x0)
	{
		if (status_code == ERROR_FLASH_OPERATION_FAILED)
		{
			command_print(cmd_ctx, "no sufficient working area specified, can't access LPC2000 IAP interface");
			return ERROR_OK;
		}
		command_print(cmd_ctx, "lpc2000 IAP returned status code %i", status_code);
	}
	else
	{
		command_print(cmd_ctx, "lpc2000 part id: 0x%8.8x", result_table[0]);
	}
	
	return ERROR_OK;
}
