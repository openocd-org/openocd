// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "riscv-013_reg.h"

#include "riscv_reg.h"
#include "riscv_reg_impl.h"
#include "riscv-013.h"
#include <helper/time_support.h>

static int riscv013_reg_get(struct reg *reg)
{
	struct target *target = riscv_reg_impl_get_target(reg);

	/* TODO: Hack to deal with gdb that thinks these registers still exist. */
	if (reg->number > GDB_REGNO_XPR15 && reg->number <= GDB_REGNO_XPR31 &&
			riscv_supports_extension(target, 'E')) {
		buf_set_u64(reg->value, 0, reg->size, 0);
		return ERROR_OK;
	}

	if (reg->number >= GDB_REGNO_V0 && reg->number <= GDB_REGNO_V31) {
		if (riscv013_get_register_buf(target, reg->value, reg->number) != ERROR_OK)
			return ERROR_FAIL;

		reg->valid = riscv_reg_impl_gdb_regno_cacheable(reg->number, /* is write? */ false);
	} else {
		uint64_t value;
		int result = riscv_reg_get(target, &value, reg->number);
		if (result != ERROR_OK)
			return result;
		buf_set_u64(reg->value, 0, reg->size, value);
	}
	char *str = buf_to_hex_str(reg->value, reg->size);
	LOG_TARGET_DEBUG(target, "Read 0x%s from %s (valid=%d).", str, reg->name,
			reg->valid);
	free(str);
	return ERROR_OK;
}

static int riscv013_reg_set(struct reg *reg, uint8_t *buf)
{
	struct target *target = riscv_reg_impl_get_target(reg);
	RISCV_INFO(r);

	char *str = buf_to_hex_str(buf, reg->size);
	LOG_TARGET_DEBUG(target, "Write 0x%s to %s (valid=%d).", str, reg->name,
			reg->valid);
	free(str);

	/* TODO: Hack to deal with gdb that thinks these registers still exist. */
	if (reg->number > GDB_REGNO_XPR15 && reg->number <= GDB_REGNO_XPR31 &&
			riscv_supports_extension(target, 'E') &&
			buf_get_u64(buf, 0, reg->size) == 0)
		return ERROR_OK;

	if (reg->number == GDB_REGNO_TDATA1 ||
			reg->number == GDB_REGNO_TDATA2) {
		r->manual_hwbp_set = true;
		/* When enumerating triggers, we clear any triggers with DMODE set,
		 * assuming they were left over from a previous debug session. So make
		 * sure that is done before a user might be setting their own triggers.
		 */
		if (riscv_enumerate_triggers(target) != ERROR_OK)
			return ERROR_FAIL;
	}

	if (reg->number >= GDB_REGNO_V0 && reg->number <= GDB_REGNO_V31) {
		if (riscv013_set_register_buf(target, reg->number, buf) != ERROR_OK)
			return ERROR_FAIL;

		memcpy(reg->value, buf, DIV_ROUND_UP(reg->size, 8));
		reg->valid = riscv_reg_impl_gdb_regno_cacheable(reg->number, /* is write? */ true);
	} else {
		const riscv_reg_t value = buf_get_u64(buf, 0, reg->size);
		if (riscv_reg_set(target, reg->number, value) != ERROR_OK)
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static const struct reg_arch_type *riscv013_gdb_regno_reg_type(uint32_t regno)
{
	static const struct reg_arch_type riscv011_reg_type = {
		.get = riscv013_reg_get,
		.set = riscv013_reg_set
	};
	return &riscv011_reg_type;
}

static int riscv013_init_reg(struct target *target, uint32_t regno)
{
	return riscv_reg_impl_init_one(target, regno, riscv013_gdb_regno_reg_type(regno));
}

int riscv013_reg_init_all(struct target *target)
{
	if (riscv_reg_impl_init_cache(target) != ERROR_OK)
		return ERROR_FAIL;

	init_shared_reg_info(target);

	riscv_reg_impl_init_vector_reg_type(target);

	for (uint32_t regno = 0; regno < target->reg_cache->num_regs; ++regno)
		if (riscv013_init_reg(target, regno) != ERROR_OK)
			return ERROR_FAIL;

	if (riscv_reg_impl_expose_csrs(target) != ERROR_OK)
		return ERROR_FAIL;

	riscv_reg_impl_hide_csrs(target);

	return ERROR_OK;
}

/**
 * This function is used to save the value of a register in cache. The register
 * is marked as dirty, and writeback is delayed for as long as possible.
 */
int riscv013_reg_save(struct target *target, enum gdb_regno regid)
{
	if (target->state != TARGET_HALTED) {
		LOG_TARGET_ERROR(target, "Can't save register %s on a hart that is not halted.",
				 riscv_reg_gdb_regno_name(target, regid));
		return ERROR_FAIL;
	}
	assert(riscv_reg_impl_gdb_regno_cacheable(regid, /* is write? */ false) &&
			"Only cacheable registers can be saved.");

	RISCV_INFO(r);
	riscv_reg_t value;
	if (!target->reg_cache) {
		assert(!target_was_examined(target));
		/* To create register cache it is needed to examine the target first,
		 * therefore during examine, any changed register needs to be saved
		 * and restored manually.
		 */
		return ERROR_OK;
	}

	struct reg *reg = riscv_reg_impl_cache_entry(target, regid);

	LOG_TARGET_DEBUG(target, "Saving %s", reg->name);
	if (riscv_reg_get(target, &value, regid) != ERROR_OK)
		return ERROR_FAIL;

	assert(reg->valid &&
			"The register is cacheable, so the cache entry must be valid now.");
	/* Mark the register dirty. We assume that this function is called
	 * because the caller is about to mess with the underlying value of the
	 * register. */
	reg->dirty = true;

	r->last_activity = timeval_ms();

	return ERROR_OK;
}
