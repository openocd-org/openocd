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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/log.h>
#include "target.h"
#include "armv4_5_mmu.h"

int armv4_5_mmu_translate_va(struct target *target,
		struct armv4_5_mmu_common *armv4_5_mmu, uint32_t va, uint32_t *cb, uint32_t *val)
{
	uint32_t first_lvl_descriptor = 0x0;
	uint32_t second_lvl_descriptor = 0x0;
	uint32_t ttb;
	int retval;
	retval = armv4_5_mmu->get_ttb(target, &ttb);
	if (retval != ERROR_OK)
		return retval;

	retval = armv4_5_mmu_read_physical(target, armv4_5_mmu,
		(ttb & 0xffffc000) | ((va & 0xfff00000) >> 18),
		4, 1, (uint8_t *)&first_lvl_descriptor);
	if (retval != ERROR_OK)
		return retval;
	first_lvl_descriptor = target_buffer_get_u32(target, (uint8_t *)&first_lvl_descriptor);

	LOG_DEBUG("1st lvl desc: %8.8" PRIx32 "", first_lvl_descriptor);

	if ((first_lvl_descriptor & 0x3) == 0) {
		LOG_ERROR("Address translation failure");
		return ERROR_TARGET_TRANSLATION_FAULT;
	}

	if (!armv4_5_mmu->has_tiny_pages && ((first_lvl_descriptor & 0x3) == 3)) {
		LOG_ERROR("Address translation failure");
		return ERROR_TARGET_TRANSLATION_FAULT;
	}

	if ((first_lvl_descriptor & 0x3) == 2) {
		/* section descriptor */
		*cb = (first_lvl_descriptor & 0xc) >> 2;
		*val = (first_lvl_descriptor & 0xfff00000) | (va & 0x000fffff);
		return ERROR_OK;
	}

	if ((first_lvl_descriptor & 0x3) == 1) {
		/* coarse page table */
		retval = armv4_5_mmu_read_physical(target, armv4_5_mmu,
			(first_lvl_descriptor & 0xfffffc00) | ((va & 0x000ff000) >> 10),
			4, 1, (uint8_t *)&second_lvl_descriptor);
		if (retval != ERROR_OK)
			return retval;
	} else if ((first_lvl_descriptor & 0x3) == 3) {
		/* fine page table */
		retval = armv4_5_mmu_read_physical(target, armv4_5_mmu,
			(first_lvl_descriptor & 0xfffff000) | ((va & 0x000ffc00) >> 8),
			4, 1, (uint8_t *)&second_lvl_descriptor);
		if (retval != ERROR_OK)
			return retval;
	}

	second_lvl_descriptor = target_buffer_get_u32(target, (uint8_t *)&second_lvl_descriptor);

	LOG_DEBUG("2nd lvl desc: %8.8" PRIx32 "", second_lvl_descriptor);

	if ((second_lvl_descriptor & 0x3) == 0) {
		LOG_ERROR("Address translation failure");
		return ERROR_TARGET_TRANSLATION_FAULT;
	}

	/* cacheable/bufferable is always specified in bits 3-2 */
	*cb = (second_lvl_descriptor & 0xc) >> 2;

	if ((second_lvl_descriptor & 0x3) == 1) {
		/* large page descriptor */
		*val = (second_lvl_descriptor & 0xffff0000) | (va & 0x0000ffff);
		return ERROR_OK;
	}

	if ((second_lvl_descriptor & 0x3) == 2) {
		/* small page descriptor */
		*val = (second_lvl_descriptor & 0xfffff000) | (va & 0x00000fff);
		return ERROR_OK;
	}

	if ((second_lvl_descriptor & 0x3) == 3) {
		/* tiny page descriptor */
		*val = (second_lvl_descriptor & 0xfffffc00) | (va & 0x000003ff);
		return ERROR_OK;
	}

	/* should not happen */
	LOG_ERROR("Address translation failure");
	return ERROR_TARGET_TRANSLATION_FAULT;
}

int armv4_5_mmu_read_physical(struct target *target,
		struct armv4_5_mmu_common *armv4_5_mmu, uint32_t address,
		uint32_t size, uint32_t count, uint8_t *buffer)
{
	int retval;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	/* disable MMU and data (or unified) cache */
	retval = armv4_5_mmu->disable_mmu_caches(target, 1, 1, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = armv4_5_mmu->read_memory(target, address, size, count, buffer);
	if (retval != ERROR_OK)
		return retval;

	/* reenable MMU / cache */
	retval = armv4_5_mmu->enable_mmu_caches(target, armv4_5_mmu->mmu_enabled,
		armv4_5_mmu->armv4_5_cache.d_u_cache_enabled,
		armv4_5_mmu->armv4_5_cache.i_cache_enabled);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

int armv4_5_mmu_write_physical(struct target *target,
		struct armv4_5_mmu_common *armv4_5_mmu, uint32_t address,
		uint32_t size, uint32_t count, const uint8_t *buffer)
{
	int retval;

	if (target->state != TARGET_HALTED)
		return ERROR_TARGET_NOT_HALTED;

	/* disable MMU and data (or unified) cache */
	retval = armv4_5_mmu->disable_mmu_caches(target, 1, 1, 0);
	if (retval != ERROR_OK)
		return retval;

	retval = armv4_5_mmu->write_memory(target, address, size, count, buffer);
	if (retval != ERROR_OK)
		return retval;

	/* reenable MMU / cache */
	retval = armv4_5_mmu->enable_mmu_caches(target, armv4_5_mmu->mmu_enabled,
		armv4_5_mmu->armv4_5_cache.d_u_cache_enabled,
		armv4_5_mmu->armv4_5_cache.i_cache_enabled);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}
