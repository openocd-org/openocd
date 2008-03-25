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

#include "arm7_9_common.h"
#include "log.h"
#include "command.h"
#include "armv4_5_mmu.h"
#include "target.h"

#include <stdlib.h>

u32 armv4mmu_translate_va(target_t *target, armv4_5_mmu_common_t *armv4_5_mmu, u32 va, int *type, u32 *cb, int *domain, u32 *ap);
int armv4_5_mmu_read_physical(target_t *target, armv4_5_mmu_common_t *armv4_5_mmu, u32 address, u32 size, u32 count, u8 *buffer);
int armv4_5_mmu_write_physical(target_t *target, armv4_5_mmu_common_t *armv4_5_mmu, u32 address, u32 size, u32 count, u8 *buffer);

char* armv4_5_mmu_page_type_names[] =
{
	"section", "large page", "small page", "tiny page"
};

u32 armv4_5_mmu_translate_va(target_t *target, armv4_5_mmu_common_t *armv4_5_mmu, u32 va, int *type, u32 *cb, int *domain, u32 *ap)
{
	u32 first_lvl_descriptor = 0x0;
	u32 second_lvl_descriptor = 0x0;
	u32 ttb = armv4_5_mmu->get_ttb(target);

	armv4_5_mmu_read_physical(target, armv4_5_mmu,
		(ttb & 0xffffc000) | ((va & 0xfff00000) >> 18),
		4, 1, (u8*)&first_lvl_descriptor);
	first_lvl_descriptor = target_buffer_get_u32(target, (u8*)&first_lvl_descriptor);

	LOG_DEBUG("1st lvl desc: %8.8x", first_lvl_descriptor);

	if ((first_lvl_descriptor & 0x3) == 0)
	{
		*type = -1;
		LOG_ERROR("Address translation failure");
		return ERROR_TARGET_TRANSLATION_FAULT;
	}

	if (!armv4_5_mmu->has_tiny_pages && ((first_lvl_descriptor & 0x3) == 3))
	{
		*type = -1;
		LOG_ERROR("Address translation failure");
		return ERROR_TARGET_TRANSLATION_FAULT;
	}

	/* domain is always specified in bits 8-5 */
	*domain = (first_lvl_descriptor & 0x1e0) >> 5;

	if ((first_lvl_descriptor & 0x3) == 2)
	{
		/* section descriptor */
		*type = ARMV4_5_SECTION;
		*cb = (first_lvl_descriptor & 0xc) >> 2;
		*ap = (first_lvl_descriptor & 0xc00) >> 10;
		return (first_lvl_descriptor & 0xfff00000) | (va & 0x000fffff);
	}

	if ((first_lvl_descriptor & 0x3) == 1)
	{
		/* coarse page table */
		armv4_5_mmu_read_physical(target, armv4_5_mmu,
			(first_lvl_descriptor & 0xfffffc00) | ((va & 0x000ff000) >> 10),
			4, 1, (u8*)&second_lvl_descriptor);
	}
	else if ((first_lvl_descriptor & 0x3) == 3)
	{
		/* fine page table */
		armv4_5_mmu_read_physical(target, armv4_5_mmu,
			(first_lvl_descriptor & 0xfffff000) | ((va & 0x000ffc00) >> 8),
			4, 1, (u8*)&second_lvl_descriptor);
	}
	
	second_lvl_descriptor = target_buffer_get_u32(target, (u8*)&second_lvl_descriptor);
	
	LOG_DEBUG("2nd lvl desc: %8.8x", second_lvl_descriptor);

	if ((second_lvl_descriptor & 0x3) == 0)
	{
		*type = -1;
		LOG_ERROR("Address translation failure");
		return ERROR_TARGET_TRANSLATION_FAULT;
	}

	/* cacheable/bufferable is always specified in bits 3-2 */
	*cb = (second_lvl_descriptor & 0xc) >> 2;

	if ((second_lvl_descriptor & 0x3) == 1)
	{
		/* large page descriptor */
		*type = ARMV4_5_LARGE_PAGE;
		*ap = (second_lvl_descriptor & 0xff0) >> 4;
		return (second_lvl_descriptor & 0xffff0000) | (va & 0x0000ffff);
	}

	if ((second_lvl_descriptor & 0x3) == 2)
	{
		/* small page descriptor */
		*type = ARMV4_5_SMALL_PAGE;
		*ap = (second_lvl_descriptor & 0xff0) >> 4;
		return (second_lvl_descriptor & 0xfffff000) | (va & 0x00000fff);
	}

	if ((second_lvl_descriptor & 0x3) == 3)
	{
		/* tiny page descriptor */
		*type = ARMV4_5_TINY_PAGE;
		*ap = (second_lvl_descriptor & 0x30) >> 4;
		return (second_lvl_descriptor & 0xfffffc00) | (va & 0x000003ff);
	}

	/* should not happen */
	*type = -1;
	LOG_ERROR("Address translation failure");
	return ERROR_TARGET_TRANSLATION_FAULT;
}

int armv4_5_mmu_read_physical(target_t *target, armv4_5_mmu_common_t *armv4_5_mmu, u32 address, u32 size, u32 count, u8 *buffer)
{
	int retval;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	/* disable MMU and data (or unified) cache */
	armv4_5_mmu->disable_mmu_caches(target, 1, 1, 0);

	retval = armv4_5_mmu->read_memory(target, address, size, count, buffer);

	/* reenable MMU / cache */
	armv4_5_mmu->enable_mmu_caches(target, armv4_5_mmu->mmu_enabled,
		armv4_5_mmu->armv4_5_cache.d_u_cache_enabled,
		armv4_5_mmu->armv4_5_cache.i_cache_enabled);

	return retval;
}

int armv4_5_mmu_write_physical(target_t *target, armv4_5_mmu_common_t *armv4_5_mmu, u32 address, u32 size, u32 count, u8 *buffer)
{
	int retval;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	/* disable MMU and data (or unified) cache */
	armv4_5_mmu->disable_mmu_caches(target, 1, 1, 0);
	
	retval = armv4_5_mmu->write_memory(target, address, size, count, buffer);

	/* reenable MMU / cache */
	armv4_5_mmu->enable_mmu_caches(target, armv4_5_mmu->mmu_enabled,
		armv4_5_mmu->armv4_5_cache.d_u_cache_enabled,
		armv4_5_mmu->armv4_5_cache.i_cache_enabled);
	
	return retval;
}

int armv4_5_mmu_handle_virt2phys_command(command_context_t *cmd_ctx, char *cmd, char **args, int argc, target_t *target, armv4_5_mmu_common_t *armv4_5_mmu)
{
	u32 va;
	u32 pa;
	int type;
	u32 cb;
	int domain;
	u32 ap;
	
	if (target->state != TARGET_HALTED)
	{
		command_print(cmd_ctx, "target must be stopped for \"virt2phys\" command");
		return ERROR_OK;
	}

	if (argc == 0)
	{
		command_print(cmd_ctx, "usage: virt2phys <virtual address>");
		return ERROR_OK;
	}

	if (argc == 1)
	{
		va = strtoul(args[0], NULL, 0);
		pa = armv4_5_mmu_translate_va(target, armv4_5_mmu, va, &type, &cb, &domain, &ap);
		if (type == -1)
		{
			switch (pa)
			{
				case ERROR_TARGET_TRANSLATION_FAULT:
					command_print(cmd_ctx, "no valid translation for 0x%8.8x", va);
					break;
				default:
					command_print(cmd_ctx, "unknown translation error");
			}
			return ERROR_OK;
		}
	
		command_print(cmd_ctx, "0x%8.8x -> 0x%8.8x, type: %s, cb: %i, domain: %i, ap: %2.2x",
			va, pa, armv4_5_mmu_page_type_names[type], cb, domain, ap);
	}			
	
	return ERROR_OK;
}

int armv4_5_mmu_handle_md_phys_command(command_context_t *cmd_ctx, char *cmd, char **args, int argc, target_t *target, armv4_5_mmu_common_t *armv4_5_mmu)
{
	int count = 1;
	int size = 4;
	u32 address = 0;
	int i;

	char output[128];
	int output_len;

	int retval;

	u8 *buffer;

	if (target->state != TARGET_HALTED)
	{
		command_print(cmd_ctx, "target must be stopped for \"%s\" command", cmd);
		return ERROR_OK;
	}

	if (argc < 1)
		return ERROR_OK;

	if (argc == 2)
		count = strtoul(args[1], NULL, 0);

	address = strtoul(args[0], NULL, 0);

	switch (cmd[2])
	{
		case 'w':
			size = 4;
			break;
		case 'h':
			size = 2;
			break;
		case 'b':
			size = 1;
			break;
		default:
			return ERROR_OK;
	}

	buffer = calloc(count, size);
	if ((retval  = armv4_5_mmu_read_physical(target, armv4_5_mmu, address, size, count, buffer)) != ERROR_OK)
	{
		switch (retval)
		{
			case ERROR_TARGET_UNALIGNED_ACCESS:
				command_print(cmd_ctx, "error: address not aligned");
				break;
			case ERROR_TARGET_NOT_HALTED:
				command_print(cmd_ctx, "error: target must be halted for memory accesses");
				break;			
			case ERROR_TARGET_DATA_ABORT:
				command_print(cmd_ctx, "error: access caused data abort, system possibly corrupted");
				break;
			default:
				command_print(cmd_ctx, "error: unknown error");
		}
	}

	output_len = 0;

	for (i = 0; i < count; i++)
	{
		if (i%8 == 0)
			output_len += snprintf(output + output_len, 128 - output_len, "0x%8.8x: ", address + (i*size));
		
		switch (size)
		{
			case 4:
				output_len += snprintf(output + output_len, 128 - output_len, "%8.8x ", target_buffer_get_u32(target, &buffer[i*4]));
				break;
			case 2:
				output_len += snprintf(output + output_len, 128 - output_len, "%4.4x ", target_buffer_get_u16(target, &buffer[i*2]));
				break;
			case 1:
				output_len += snprintf(output + output_len, 128 - output_len, "%2.2x ", buffer[i*1]);
				break;
		}

		if ((i % 8 == 7) || (i == count - 1))
		{
			command_print(cmd_ctx, output);
			output_len = 0;
		}
	}

	free(buffer);
	
	return ERROR_OK;
}

int armv4_5_mmu_handle_mw_phys_command(command_context_t *cmd_ctx, char *cmd, char **args, int argc, target_t *target, armv4_5_mmu_common_t *armv4_5_mmu)
{
	u32 address = 0;
	u32 value = 0;
	int retval;
	u8 value_buf[4];

	if (target->state != TARGET_HALTED)
	{
		command_print(cmd_ctx, "target must be stopped for \"%s\" command", cmd);
		return ERROR_OK;
	}

	if (argc < 2)
		return ERROR_OK;

	address = strtoul(args[0], NULL, 0);
	value = strtoul(args[1], NULL, 0);

	switch (cmd[2])
	{
		case 'w':
			target_buffer_set_u32(target, value_buf, value);
			retval = armv4_5_mmu_write_physical(target, armv4_5_mmu, address, 4, 1, value_buf);
			break;
		case 'h':
			target_buffer_set_u16(target, value_buf, value);
			retval = armv4_5_mmu_write_physical(target, armv4_5_mmu, address, 2, 1, value_buf);
			break;
		case 'b':
			value_buf[0] = value;
			retval = armv4_5_mmu_write_physical(target, armv4_5_mmu, address, 1, 1, value_buf);
			break;
		default:
			return ERROR_OK;
	}

	switch (retval)
	{
		case ERROR_TARGET_UNALIGNED_ACCESS:
			command_print(cmd_ctx, "error: address not aligned");
			break;
		case ERROR_TARGET_DATA_ABORT:
			command_print(cmd_ctx, "error: access caused data abort, system possibly corrupted");
			break;
		case ERROR_TARGET_NOT_HALTED:
			command_print(cmd_ctx, "error: target must be halted for memory accesses");
			break;
		case ERROR_OK:
			break;
		default:
			command_print(cmd_ctx, "error: unknown error");
	}	

	return ERROR_OK;
}
