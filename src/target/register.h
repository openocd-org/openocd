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

#ifndef REGISTER_H
#define REGISTER_H

struct target;

enum reg_type {
	REG_TYPE_INT,
	REG_TYPE_INT8,
	REG_TYPE_INT16,
	REG_TYPE_INT32,
	REG_TYPE_INT64,
	REG_TYPE_INT128,
	REG_TYPE_UINT8,
	REG_TYPE_UINT16,
	REG_TYPE_UINT32,
	REG_TYPE_UINT64,
	REG_TYPE_UINT128,
	REG_TYPE_CODE_PTR,
	REG_TYPE_DATA_PTR,
	REG_TYPE_FLOAT,
	REG_TYPE_IEEE_SINGLE,
	REG_TYPE_IEEE_DOUBLE,
	REG_TYPE_ARCH_DEFINED,
};

struct reg_feature {
	const char *name;
};

struct reg_data_type_vector {
	struct reg_data_type *type;
	uint32_t count;
};

struct reg_data_type_union_field {
	const char *name;
	struct reg_data_type *type;
	struct reg_data_type_union_field *next;
};

struct reg_data_type_union {
	struct reg_data_type_union_field *fields;
};

struct reg_data_type_bitfield {
	uint32_t start;
	uint32_t end;
};

struct reg_data_type_struct_field {
	const char *name;
	bool use_bitfields;
	union {
		struct reg_data_type_bitfield *bitfield;
		struct reg_data_type *type;
	};
	struct reg_data_type_struct_field *next;
};

struct reg_data_type_struct {
	uint32_t size;
	struct reg_data_type_struct_field *fields;
};

struct reg_data_type_flags_field {
	const char *name;
	struct reg_data_type_bitfield *bitfield;
	struct reg_data_type_flags_field *next;
};

struct reg_data_type_flags {
	uint32_t size;
	struct reg_data_type_flags_field *fields;
};

enum reg_data_type_class {
	REG_TYPE_CLASS_VECTOR,
	REG_TYPE_CLASS_UNION,
	REG_TYPE_CLASS_STRUCT,
	REG_TYPE_CLASS_FLAGS,
};

struct reg_data_type {
	enum reg_type type;
	const char *id;
	enum reg_data_type_class type_class;
	union {
		struct reg_data_type_vector *reg_type_vector;
		struct reg_data_type_union *reg_type_union;
		struct reg_data_type_struct *reg_type_struct;
		struct reg_data_type_flags *reg_type_flags;
	};
};

struct reg {
	const char *name;
	uint32_t number;
	struct reg_feature *feature;
	bool caller_save;
	void *value;
	bool dirty;
	bool valid;
	bool exist;
	uint32_t size;
	struct reg_data_type *reg_data_type;
	const char *group;
	void *arch_info;
	const struct reg_arch_type *type;
};

struct reg_cache {
	const char *name;
	struct reg_cache *next;
	struct reg *reg_list;
	unsigned num_regs;
};

struct reg_arch_type {
	int (*get)(struct reg *reg);
	int (*set)(struct reg *reg, uint8_t *buf);
};

struct reg *register_get_by_name(struct reg_cache *first,
		const char *name, bool search_all);
struct reg_cache **register_get_last_cache_p(struct reg_cache **first);
void register_cache_invalidate(struct reg_cache *cache);

void register_init_dummy(struct reg *reg);

#endif /* REGISTER_H */
