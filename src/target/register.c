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

#include "register.h"

#include "log.h"
#include "command.h"

#include <string.h>
#include <stdlib.h>

reg_arch_type_t *reg_arch_types = NULL;

reg_t* register_get_by_name(reg_cache_t *first, char *name, int search_all)
{
	int i;
	reg_cache_t *cache = first;
	
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

reg_cache_t** register_get_last_cache_p(reg_cache_t **first)
{
	reg_cache_t **cache_p = first;
	
	if (*cache_p)
		while (*cache_p)
			cache_p = &((*cache_p)->next);
	else
		return first;
	
	return cache_p;
}

int register_reg_arch_type(int (*get)(reg_t *reg), int (*set)(reg_t *reg, u8 *buf))
{
	reg_arch_type_t** arch_type_p = &reg_arch_types;
	int id = 0;
	
	if (*arch_type_p)
	{
		while (*arch_type_p)
		{
			id = (*arch_type_p)->id;
			arch_type_p = &((*arch_type_p)->next);
		}
	}
	
	(*arch_type_p) = malloc(sizeof(reg_arch_type_t));
	(*arch_type_p)->id = id + 1;
	(*arch_type_p)->set = set;
	(*arch_type_p)->get = get;
	(*arch_type_p)->next = NULL;
			
	return id + 1;
}

reg_arch_type_t* register_get_arch_type(int id)
{
	reg_arch_type_t *arch_type = reg_arch_types;
	
	while (arch_type)
	{
		if (arch_type->id == id)
			return arch_type;
		arch_type = arch_type->next;
	}
	
	return NULL;
}
