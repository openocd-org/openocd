/***************************************************************************
 *   Copyright (C) 2016 by Matthias Welwarsky                              *
 *   matthias.welwarsky@sysgo.com                                          *
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

#include "armv8_cache.h"
#include "armv8_dpm.h"
#include "armv8_opcodes.h"

static int armv8_d_cache_sanity_check(struct armv8_common *armv8)
{
	struct armv8_cache_common *armv8_cache = &armv8->armv8_mmu.armv8_cache;

	if (armv8_cache->d_u_cache_enabled)
		return ERROR_OK;

	return ERROR_TARGET_INVALID;
}

static int armv8_i_cache_sanity_check(struct armv8_common *armv8)
{
	struct armv8_cache_common *armv8_cache = &armv8->armv8_mmu.armv8_cache;

	if (armv8_cache->i_cache_enabled)
		return ERROR_OK;

	return ERROR_TARGET_INVALID;
}

int armv8_cache_d_inner_flush_virt(struct armv8_common *armv8, target_addr_t va, size_t size)
{
	struct arm_dpm *dpm = armv8->arm.dpm;
	struct armv8_cache_common *armv8_cache = &armv8->armv8_mmu.armv8_cache;
	uint64_t linelen = armv8_cache->d_u_size.linelen;
	target_addr_t va_line, va_end;
	int retval;

	retval = armv8_d_cache_sanity_check(armv8);
	if (retval != ERROR_OK)
		return retval;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;

	va_line = va & (-linelen);
	va_end = va + size;

	while (va_line < va_end) {
		/* DC CIVAC */
		/* Aarch32: DCCIMVAC: ARMV4_5_MCR(15, 0, 0, 7, 14, 1) */
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_SYS(SYSTEM_DCCIVAC, 0), va_line);
		if (retval != ERROR_OK)
			goto done;
		va_line += linelen;
	}

	dpm->finish(dpm);
	return retval;

done:
	LOG_ERROR("d-cache invalidate failed");
	dpm->finish(dpm);

	return retval;
}

int armv8_cache_i_inner_inval_virt(struct armv8_common *armv8, target_addr_t va, size_t size)
{
	struct arm_dpm *dpm = armv8->arm.dpm;
	struct armv8_cache_common *armv8_cache = &armv8->armv8_mmu.armv8_cache;
	uint64_t linelen = armv8_cache->i_size.linelen;
	target_addr_t va_line, va_end;
	int retval;

	retval = armv8_i_cache_sanity_check(armv8);
	if (retval != ERROR_OK)
		return retval;

	retval = dpm->prepare(dpm);
	if (retval != ERROR_OK)
		goto done;

	va_line = va & (-linelen);
	va_end = va + size;

	while (va_line < va_end) {
		/* IC IVAU - Invalidate instruction cache by VA to PoU. */
		retval = dpm->instr_write_data_r0_64(dpm,
				ARMV8_SYS(SYSTEM_ICIVAU, 0), va_line);
		if (retval != ERROR_OK)
			goto done;
		va_line += linelen;
	}

	dpm->finish(dpm);
	return retval;

done:
	LOG_ERROR("d-cache invalidate failed");
	dpm->finish(dpm);

	return retval;
}
