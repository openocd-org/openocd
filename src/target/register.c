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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "register.h"
#include "log.h"


struct reg_arch_type *reg_arch_types = NULL;

struct reg* register_get_by_name(struct reg_cache *first,
		const char *name, bool search_all)
{
	int i;
	struct reg_cache *cache = first;

	while (cache)
	{
		for (i = 0; i < cache->num_regs; i++)
		{
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

struct reg_cache** register_get_last_cache_p(struct reg_cache **first)
{
	struct reg_cache **cache_p = first;

	if (*cache_p)
		while (*cache_p)
			cache_p = &((*cache_p)->next);
	else
		return first;

	return cache_p;
}

int register_reg_arch_type(int (*get)(struct reg *reg), int (*set)(struct reg *reg, uint8_t *buf))
{
	struct reg_arch_type** arch_type_p = &reg_arch_types;
	int id = 0;

	if (*arch_type_p)
	{
		while (*arch_type_p)
		{
			id = (*arch_type_p)->id;
			arch_type_p = &((*arch_type_p)->next);
		}
	}

	(*arch_type_p) = malloc(sizeof(struct reg_arch_type));
	(*arch_type_p)->id = id + 1;
	(*arch_type_p)->set = set;
	(*arch_type_p)->get = get;
	(*arch_type_p)->next = NULL;

	return id + 1;
}

struct reg_arch_type* register_get_arch_type(int id)
{
	struct reg_arch_type *arch_type = reg_arch_types;

	while (arch_type)
	{
		if (arch_type->id == id)
			return arch_type;
		arch_type = arch_type->next;
	}
	LOG_ERROR("BUG: encountered unregistered arch type 0x%08x", id);
	exit(-1);
	return NULL;
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

void register_init_dummy(struct reg *reg)
{
	static int dummy_arch_type = -1;
	if (dummy_arch_type == -1)
		dummy_arch_type = register_reg_arch_type(register_get_dummy_core_reg, register_set_dummy_core_reg);

	reg->arch_type = dummy_arch_type;
}
