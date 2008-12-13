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
#include "target.h"

struct target_s;

typedef struct bitfield_desc_s
{
	char *name;
	int num_bits;
} bitfield_desc_t;

typedef struct reg_s
{
	char *name;
	u8 *value;
	int dirty;
	int valid;
	int size;
	bitfield_desc_t *bitfield_desc;
	int num_bitfields;
	void *arch_info;
	int arch_type;
} reg_t;

typedef struct reg_cache_s
{
	char *name;
	struct reg_cache_s *next;
	reg_t *reg_list;
	int num_regs;
} reg_cache_t;

typedef struct reg_arch_type_s
{
	int id;
	int (*get)(reg_t *reg);
	int (*set)(reg_t *reg, u8 *buf);
	struct reg_arch_type_s *next;
} reg_arch_type_t;

extern reg_t* register_get_by_name(reg_cache_t *first, char *name, int search_all);
extern reg_cache_t** register_get_last_cache_p(reg_cache_t **first);
extern int register_reg_arch_type(int (*get)(reg_t *reg), int (*set)(reg_t *reg, u8 *buf));
extern reg_arch_type_t* register_get_arch_type(int id);
extern void register_init_dummy(reg_t *reg);

#endif /* REGISTER_H */
