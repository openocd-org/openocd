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

#include <helper/log.h>
#include "target.h"
#include "armv4_5_mmu.h"


uint32_t armv4mmu_translate_va(struct target *target, struct armv4_5_mmu_common *armv4_5_mmu, uint32_t va, int *type, uint32_t *cb, int *domain, uint32_t *ap);

char* armv4_5_mmu_page_type_names[] =
{
	"section", "large page", "small page", "tiny page"
};

uint32_t armv4_5_mmu_translate_va(struct target *target, struct armv4_5_mmu_common *armv4_5_mmu, uint32_t va, int *type, uint32_t *cb, int *domain, uint32_t *ap)
{
	uint32_t first_lvl_descriptor = 0x0;
	uint32_t second_lvl_descriptor = 0x0;
	uint32_t ttb = armv4_5_mmu->get_ttb(target);

	armv4_5_mmu_read_physical(target, armv4_5_mmu,
		(ttb & 0xffffc000) | ((va & 0xfff00000) >> 18),
		4, 1, (uint8_t*)&first_lvl_descriptor);
	first_lvl_descriptor = target_buffer_get_u32(target, (uint8_t*)&first_lvl_descriptor);

	LOG_DEBUG("1st lvl desc: %8.8" PRIx32 "", first_lvl_descriptor);

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
			4, 1, (uint8_t*)&second_lvl_descriptor);
	}
	else if ((first_lvl_descriptor & 0x3) == 3)
	{
		/* fine page table */
		armv4_5_mmu_read_physical(target, armv4_5_mmu,
			(first_lvl_descriptor & 0xfffff000) | ((va & 0x000ffc00) >> 8),
			4, 1, (uint8_t*)&second_lvl_descriptor);
	}

	second_lvl_descriptor = target_buffer_get_u32(target, (uint8_t*)&second_lvl_descriptor);

	LOG_DEBUG("2nd lvl desc: %8.8" PRIx32 "", second_lvl_descriptor);

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

int armv4_5_mmu_read_physical(struct target *target, struct armv4_5_mmu_common *armv4_5_mmu, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer)
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

int armv4_5_mmu_write_physical(struct target *target, struct armv4_5_mmu_common *armv4_5_mmu, uint32_t address, uint32_t size, uint32_t count, uint8_t *buffer)
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
