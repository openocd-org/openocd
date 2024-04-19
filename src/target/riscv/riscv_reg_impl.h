/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_TARGET_RISCV_RISCV_REG_IMPL_H
#define OPENOCD_TARGET_RISCV_RISCV_REG_IMPL_H

#include "gdb_regs.h"
#include "riscv.h"
#include "target/target.h"
#include "target/register.h"
#include <assert.h>

/**
 * This file describes the helpers to use during register cache initialization
 * of a RISC-V target. Each cache entry proceedes through the following stages:
 *  - not allocated before `riscv_reg_impl_init_cache()`
 *  - not initialized before the call to `riscv_reg_impl_init_one()` with appropriate regno.
 *  - initialized until `riscv_reg_free_all()` is called.
 */
static inline bool riscv_reg_impl_is_initialized(const struct reg *reg)
{
	assert(reg);
	if (!reg->feature) {
		const struct reg default_reg = {0};
		assert(!memcmp(&default_reg, reg, sizeof(*reg)));
		return false;
	}
	assert(reg->arch_info);
	assert(((riscv_reg_info_t *)reg->arch_info)->target);
	assert((!reg->exist && !reg->value) || (reg->exist && reg->value));
	assert(reg->valid || !reg->dirty);
	return true;
}
/**
 * Initialize register cache. Note, that each specific register cache entry is
 * not initialized by this function.
 */
int riscv_reg_impl_init_cache(struct target *target);

/** Initialize register. */
int riscv_reg_impl_init_one(struct target *target, uint32_t regno,
		const struct reg_arch_type *reg_type);

/** Return the entry in the register cache of the target. */
struct reg *riscv_reg_impl_cache_entry(const struct target *target,
		uint32_t number);

/** Return the target that owns the cache entry. */
struct target *riscv_reg_impl_get_target(const struct reg *reg);

static inline void init_shared_reg_info(struct target *target)
{
	RISCV_INFO(info);
	info->shared_reg_info.target = target;
	info->shared_reg_info.custom_number = 0;
}

/** TODO: vector register type description can be moved into `riscv013_info_t`,
 * since 0.11 targets do not support access to vector registers. */
static inline void riscv_reg_impl_init_vector_reg_type(const struct target *target)
{
	RISCV_INFO(info);
	static struct reg_data_type type_uint8 = { .type = REG_TYPE_UINT8, .id = "uint8" };
	static struct reg_data_type type_uint16 = { .type = REG_TYPE_UINT16, .id = "uint16" };
	static struct reg_data_type type_uint32 = { .type = REG_TYPE_UINT32, .id = "uint32" };
	static struct reg_data_type type_uint64 = { .type = REG_TYPE_UINT64, .id = "uint64" };
	static struct reg_data_type type_uint128 = { .type = REG_TYPE_UINT128, .id = "uint128" };

	/* This is roughly the XML we want:
	 * <vector id="bytes" type="uint8" count="16"/>
	 * <vector id="shorts" type="uint16" count="8"/>
	 * <vector id="words" type="uint32" count="4"/>
	 * <vector id="longs" type="uint64" count="2"/>
	 * <vector id="quads" type="uint128" count="1"/>
	 * <union id="riscv_vector_type">
	 *   <field name="b" type="bytes"/>
	 *   <field name="s" type="shorts"/>
	 *   <field name="w" type="words"/>
	 *   <field name="l" type="longs"/>
	 *   <field name="q" type="quads"/>
	 * </union>
	 */

	info->vector_uint8.type = &type_uint8;
	info->vector_uint8.count = riscv_vlenb(target);
	info->type_uint8_vector.type = REG_TYPE_ARCH_DEFINED;
	info->type_uint8_vector.id = "bytes";
	info->type_uint8_vector.type_class = REG_TYPE_CLASS_VECTOR;
	info->type_uint8_vector.reg_type_vector = &info->vector_uint8;

	info->vector_uint16.type = &type_uint16;
	info->vector_uint16.count = riscv_vlenb(target) / 2;
	info->type_uint16_vector.type = REG_TYPE_ARCH_DEFINED;
	info->type_uint16_vector.id = "shorts";
	info->type_uint16_vector.type_class = REG_TYPE_CLASS_VECTOR;
	info->type_uint16_vector.reg_type_vector = &info->vector_uint16;

	info->vector_uint32.type = &type_uint32;
	info->vector_uint32.count = riscv_vlenb(target) / 4;
	info->type_uint32_vector.type = REG_TYPE_ARCH_DEFINED;
	info->type_uint32_vector.id = "words";
	info->type_uint32_vector.type_class = REG_TYPE_CLASS_VECTOR;
	info->type_uint32_vector.reg_type_vector = &info->vector_uint32;

	info->vector_uint64.type = &type_uint64;
	info->vector_uint64.count = riscv_vlenb(target) / 8;
	info->type_uint64_vector.type = REG_TYPE_ARCH_DEFINED;
	info->type_uint64_vector.id = "longs";
	info->type_uint64_vector.type_class = REG_TYPE_CLASS_VECTOR;
	info->type_uint64_vector.reg_type_vector = &info->vector_uint64;

	info->vector_uint128.type = &type_uint128;
	info->vector_uint128.count = riscv_vlenb(target) / 16;
	info->type_uint128_vector.type = REG_TYPE_ARCH_DEFINED;
	info->type_uint128_vector.id = "quads";
	info->type_uint128_vector.type_class = REG_TYPE_CLASS_VECTOR;
	info->type_uint128_vector.reg_type_vector = &info->vector_uint128;

	info->vector_fields[0].name = "b";
	info->vector_fields[0].type = &info->type_uint8_vector;
	if (riscv_vlenb(target) >= 2) {
		info->vector_fields[0].next = info->vector_fields + 1;
		info->vector_fields[1].name = "s";
		info->vector_fields[1].type = &info->type_uint16_vector;
	} else {
		info->vector_fields[0].next = NULL;
	}
	if (riscv_vlenb(target) >= 4) {
		info->vector_fields[1].next = info->vector_fields + 2;
		info->vector_fields[2].name = "w";
		info->vector_fields[2].type = &info->type_uint32_vector;
	} else {
		info->vector_fields[1].next = NULL;
	}
	if (riscv_vlenb(target) >= 8) {
		info->vector_fields[2].next = info->vector_fields + 3;
		info->vector_fields[3].name = "l";
		info->vector_fields[3].type = &info->type_uint64_vector;
	} else {
		info->vector_fields[2].next = NULL;
	}
	if (riscv_vlenb(target) >= 16) {
		info->vector_fields[3].next = info->vector_fields + 4;
		info->vector_fields[4].name = "q";
		info->vector_fields[4].type = &info->type_uint128_vector;
	} else {
		info->vector_fields[3].next = NULL;
	}
	info->vector_fields[4].next = NULL;

	info->vector_union.fields = info->vector_fields;

	info->type_vector.type = REG_TYPE_ARCH_DEFINED;
	info->type_vector.id = "riscv_vector";
	info->type_vector.type_class = REG_TYPE_CLASS_UNION;
	info->type_vector.reg_type_union = &info->vector_union;
}

/** Expose additional CSRs, as specified by `riscv_info_t::expose_csr` list. */
int riscv_reg_impl_expose_csrs(const struct target *target);

/** Hide additional CSRs, as specified by `riscv_info_t::hide_csr` list. */
void riscv_reg_impl_hide_csrs(const struct target *target);

/**
 * If write is true:
 *   return true iff we are guaranteed that the register will contain exactly
 *       the value we just wrote when it's read.
 * If write is false:
 *   return true iff we are guaranteed that the register will read the same
 *       value in the future as the value we just read.
 */
static inline bool riscv_reg_impl_gdb_regno_cacheable(enum gdb_regno regno,
		bool is_write)
{
	if (regno == GDB_REGNO_ZERO)
		return !is_write;

	/* GPRs, FPRs, vector registers are just normal data stores. */
	if (regno <= GDB_REGNO_XPR31 ||
			(regno >= GDB_REGNO_FPR0 && regno <= GDB_REGNO_FPR31) ||
			(regno >= GDB_REGNO_V0 && regno <= GDB_REGNO_V31))
		return true;

	/* Most CSRs won't change value on us, but we can't assume it about arbitrary
	 * CSRs. */
	switch (regno) {
		case GDB_REGNO_DPC:
		case GDB_REGNO_VSTART:
		case GDB_REGNO_VXSAT:
		case GDB_REGNO_VXRM:
		case GDB_REGNO_VLENB:
		case GDB_REGNO_VL:
		case GDB_REGNO_VTYPE:
		case GDB_REGNO_MISA:
		case GDB_REGNO_DCSR:
		case GDB_REGNO_DSCRATCH0:
		case GDB_REGNO_MSTATUS:
		case GDB_REGNO_MEPC:
		case GDB_REGNO_MCAUSE:
		case GDB_REGNO_SATP:
			/*
			 * WARL registers might not contain the value we just wrote, but
			 * these ones won't spontaneously change their value either. *
			 */
			return !is_write;

		case GDB_REGNO_TSELECT:	/* I think this should be above, but then it doesn't work. */
		case GDB_REGNO_TDATA1:	/* Changes value when tselect is changed. */
		case GDB_REGNO_TDATA2:  /* Changes value when tselect is changed. */
		default:
			return false;
	}
}
#endif /* OPENOCD_TARGET_RISCV_RISCV_REG_IMPL_H */
