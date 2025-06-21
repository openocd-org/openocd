// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "riscv-011_reg.h"

#include "riscv_reg_impl.h"
#include "riscv-011.h"

static int riscv011_reg_get(struct reg *reg)
{
	struct target * const target = riscv_reg_impl_get_target(reg);
	riscv_reg_t value;
	const int result = riscv011_get_register(target, &value, reg->number);
	if (result != ERROR_OK)
		return result;
	buf_set_u64(reg->value, 0, reg->size, value);
	return ERROR_OK;
}

static int riscv011_reg_set(struct reg *reg, uint8_t *buf)
{
	const riscv_reg_t value = buf_get_u64(buf, 0, reg->size);
	struct target * const target = riscv_reg_impl_get_target(reg);
	return riscv011_set_register(target, reg->number, value);
}

static const struct reg_arch_type *riscv011_gdb_regno_reg_type(uint32_t regno)
{
	static const struct reg_arch_type riscv011_reg_type = {
		.get = riscv011_reg_get,
		.set = riscv011_reg_set
	};
	return &riscv011_reg_type;
}


int riscv011_reg_init_all(struct target *target)
{
	int res = riscv_reg_impl_init_cache(target);
	if (res != ERROR_OK)
		return res;

	init_shared_reg_info(target);

	RISCV_INFO(r);
	assert(!r->vlenb
			&& "VLENB discovery is not supported on RISC-V 0.11 targets");
	/* Existence of some registers depends on others.
	 * E.g. the presence of "v0-31" registers is infered from "vlenb" being
	 * non-zero.
	 * Currently, discovery of the following registers is not supported on
	 * RISC-V 0.11 targets. */
	uint32_t non_discoverable_regs[] = {
		GDB_REGNO_VLENB,
		GDB_REGNO_MTOPI,
		GDB_REGNO_MTOPEI
	};
	for (unsigned int i = 0; i < ARRAY_SIZE(non_discoverable_regs); ++i) {
		const uint32_t regno = non_discoverable_regs[i];
		res = riscv_reg_impl_init_cache_entry(target, regno,
				/*exist*/ false, riscv011_gdb_regno_reg_type(regno));
		if (res != ERROR_OK)
			return res;
	}

	for (uint32_t regno = 0; regno < target->reg_cache->num_regs; ++regno) {
		const struct reg * const reg = riscv_reg_impl_cache_entry(target, regno);
		if (riscv_reg_impl_is_initialized(reg))
			continue;
		res = riscv_reg_impl_init_cache_entry(target, regno,
				riscv_reg_impl_gdb_regno_exist(target, regno),
				riscv011_gdb_regno_reg_type(regno));
		if (res != ERROR_OK)
			return res;
	}

	if (riscv_reg_impl_expose_csrs(target) != ERROR_OK)
		return ERROR_FAIL;

	riscv_reg_impl_hide_csrs(target);

	return ERROR_OK;
}
