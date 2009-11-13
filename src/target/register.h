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
#ifndef REGISTER_H
#define REGISTER_H

#include "types.h"

struct target;

struct bitfield_desc
{
	char *name;
	int num_bits;
};

struct reg
{
	char *name;
	void *value;
	int dirty;
	int valid;
	uint32_t size;
	struct bitfield_desc *bitfield_desc;
	int num_bitfields;
	void *arch_info;
	int arch_type;
};

struct reg_cache
{
	char *name;
	struct reg_cache *next;
	struct reg *reg_list;
	int num_regs;
};

struct reg_arch_type
{
	int id;
	int (*get)(struct reg *reg);
	int (*set)(struct reg *reg, uint8_t *buf);
	struct reg_arch_type *next;
};

struct reg* register_get_by_name(struct reg_cache *first,
		const char *name, bool search_all);
struct reg_cache** register_get_last_cache_p(struct reg_cache **first);

int register_reg_arch_type(int (*get)(struct reg *reg),
		int (*set)(struct reg *reg, uint8_t *buf));
struct reg_arch_type* register_get_arch_type(int id);

void register_init_dummy(struct reg *reg);

#endif /* REGISTER_H */
