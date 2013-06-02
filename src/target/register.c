/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "register.h"
#include <helper/log.h>

/**
 * @file
 * Holds utilities to work with register caches.
 *
 * OpenOCD uses machine registers internally, and exposes them by name
 * to Tcl scripts.  Sets of related registers are grouped into caches.
 * For example, a CPU core will expose a set of registers, and there
 * may be separate registers associated with debug or trace modules.
 */

struct reg *register_get_by_name(struct reg_cache *first,
		const char *name, bool search_all)
{
	unsigned i;
	struct reg_cache *cache = first;

	while (cache) {
		for (i = 0; i < cache->num_regs; i++) {
			if (strcmp(cache->reg_list[i].name, name) == 0)
				return &(cache->reg_list[i]);
		}

		if (search_all)
			cache = cache->next;
		else
			break;
	}

	return NULL;
}

struct reg_cache **register_get_last_cache_p(struct reg_cache **first)
{
	struct reg_cache **cache_p = first;

	if (*cache_p)
		while (*cache_p)
			cache_p = &((*cache_p)->next);
	else
		return first;

	return cache_p;
}

/** Marks the contents of the register cache as invalid (and clean). */
void register_cache_invalidate(struct reg_cache *cache)
{
	struct reg *reg = cache->reg_list;

	for (unsigned n = cache->num_regs; n != 0; n--, reg++) {
		reg->valid = 0;
		reg->dirty = 0;
	}
}

static int register_get_dummy_core_reg(struct reg *reg)
{
	return ERROR_OK;
}

static int register_set_dummy_core_reg(struct reg *reg, uint8_t *buf)
{
	reg->dirty = 1;
	reg->valid = 1;

	return ERROR_OK;
}

static const struct reg_arch_type dummy_type = {
	.get = register_get_dummy_core_reg,
	.set = register_set_dummy_core_reg,
};

void register_init_dummy(struct reg *reg)
{
	reg->type = &dummy_type;
}
