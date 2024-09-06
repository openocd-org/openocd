// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "riscv-013_reg.h"

#include "riscv_reg.h"
#include "riscv_reg_impl.h"
#include "riscv-013.h"
#include "debug_defines.h"
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

	char *str = buf_to_hex_str(buf, reg->size);
	LOG_TARGET_DEBUG(target, "Write 0x%s to %s (valid=%d).", str, reg->name,
			reg->valid);
	free(str);

	/* TODO: Hack to deal with gdb that thinks these registers still exist. */
	if (reg->number > GDB_REGNO_XPR15 && reg->number <= GDB_REGNO_XPR31 &&
			riscv_supports_extension(target, 'E') &&
			buf_get_u64(buf, 0, reg->size) == 0)
		return ERROR_OK;

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
	static const struct reg_arch_type riscv013_reg_type = {
		.get = riscv013_reg_get,
		.set = riscv013_reg_set
	};
	return &riscv013_reg_type;
}

static int init_cache_entry(struct target *target, uint32_t regno)
{
	struct reg * const reg = riscv_reg_impl_cache_entry(target, regno);
	if (riscv_reg_impl_is_initialized(reg))
		return ERROR_OK;
	return riscv_reg_impl_init_cache_entry(target, regno,
			riscv_reg_impl_gdb_regno_exist(target, regno),
			riscv013_gdb_regno_reg_type(regno));
}

/**
 * Some registers are optional (e.g. "misa"). For such registers it is first
 * assumed they exist (via "assume_reg_exist()"), then the read is attempted
 * (via the usual "riscv_reg_get()") and if the read fails, the register is
 * marked as non-existing (via "riscv_reg_impl_set_exist()").
 */
static int assume_reg_exist(struct target *target, uint32_t regno)
{
	return riscv_reg_impl_init_cache_entry(target, regno,
			/* exist */ true, riscv013_gdb_regno_reg_type(regno));
}

static int examine_xlen(struct target *target)
{
	RISCV_INFO(r);
	unsigned int cmderr;

	const uint32_t command = riscv013_access_register_command(target,
			GDB_REGNO_S0, /* size */ 64, AC_ACCESS_REGISTER_TRANSFER);
	int res = riscv013_execute_abstract_command(target, command, &cmderr);
	if (res == ERROR_OK) {
		r->xlen = 64;
		return ERROR_OK;
	}
	if (res == ERROR_TIMEOUT_REACHED)
		return ERROR_FAIL;
	r->xlen = 32;

	return ERROR_OK;
}

static int examine_vlenb(struct target *target)
{
	RISCV_INFO(r);

	/* Reading "vlenb" requires "mstatus.vs" to be set, so "mstatus" should
	 * be accessible.*/
	int res = init_cache_entry(target, GDB_REGNO_MSTATUS);
	if (res != ERROR_OK)
		return res;

	res = assume_reg_exist(target, GDB_REGNO_VLENB);
	if (res != ERROR_OK)
		return res;

	riscv_reg_t vlenb_val;
	if (riscv_reg_get(target, &vlenb_val, GDB_REGNO_VLENB) != ERROR_OK) {
		if (riscv_supports_extension(target, 'V'))
			LOG_TARGET_WARNING(target, "Couldn't read vlenb; vector register access won't work.");
		r->vlenb = 0;
		return riscv_reg_impl_set_exist(target, GDB_REGNO_VLENB, false);
	}
	/* As defined by RISC-V V extension specification:
	 * https://github.com/riscv/riscv-v-spec/blob/2f68ef7256d6ec53e4d2bd7cb12862f406d64e34/v-spec.adoc?plain=1#L67-L72 */
	const unsigned int vlen_max = 65536;
	const unsigned int vlenb_max = vlen_max / 8;
	if (vlenb_val > vlenb_max) {
		LOG_TARGET_WARNING(target, "'vlenb == %" PRIu64
				"' is greater than maximum allowed by specification (%u); vector register access won't work.",
				vlenb_val, vlenb_max);
		r->vlenb = 0;
		return ERROR_OK;
	}
	assert(vlenb_max <= UINT_MAX);
	r->vlenb = (unsigned int)vlenb_val;

	LOG_TARGET_INFO(target, "Vector support with vlenb=%u", r->vlenb);
	return ERROR_OK;
}

static int examine_misa(struct target *target)
{
	RISCV_INFO(r);

	int res = init_cache_entry(target, GDB_REGNO_MISA);
	if (res != ERROR_OK)
		return res;

	res = riscv_reg_get(target, &r->misa, GDB_REGNO_MISA);
	if (res != ERROR_OK)
		return res;

	return ERROR_OK;
}

static int examine_mtopi(struct target *target)
{
	RISCV_INFO(r);

	/* Assume the registers exist */
	r->mtopi_readable = true;
	r->mtopei_readable = true;

	int res = assume_reg_exist(target, GDB_REGNO_MTOPI);
	if (res != ERROR_OK)
		return res;
	res = assume_reg_exist(target, GDB_REGNO_MTOPEI);
	if (res != ERROR_OK)
		return res;

	riscv_reg_t value;
	if (riscv_reg_get(target, &value, GDB_REGNO_MTOPI) != ERROR_OK) {
		r->mtopi_readable = false;
		r->mtopei_readable = false;
	} else if (riscv_reg_get(target, &value, GDB_REGNO_MTOPEI) != ERROR_OK) {
		LOG_TARGET_INFO(target, "S?aia detected without IMSIC");
		r->mtopei_readable = false;
	} else {
		LOG_TARGET_INFO(target, "S?aia detected with IMSIC");
	}
	res = riscv_reg_impl_set_exist(target, GDB_REGNO_MTOPI, r->mtopi_readable);
	if (res != ERROR_OK)
		return res;

	return riscv_reg_impl_set_exist(target, GDB_REGNO_MTOPEI, r->mtopei_readable);
}

/**
 * This function assumes target's DM to be initialized (target is able to
 * access DMs registers, execute program buffer, etc.)
 */
int riscv013_reg_examine_all(struct target *target)
{
	RISCV_INFO(r);

	int res = riscv_reg_impl_init_cache(target);
	if (res != ERROR_OK)
		return res;

	init_shared_reg_info(target);

	assert(target->state == TARGET_HALTED);

	res = examine_xlen(target);
	if (res != ERROR_OK)
		return res;

	/* Reading CSRs may clobber "s0", "s1", so it should be possible to
	 * save them in cache. */
	res = init_cache_entry(target, GDB_REGNO_S0);
	if (res != ERROR_OK)
		return res;
	res = init_cache_entry(target, GDB_REGNO_S1);
	if (res != ERROR_OK)
		return res;

	res = examine_misa(target);
	if (res != ERROR_OK)
		return res;

	/* Display this as early as possible to help people who are using
	 * really slow simulators. */
	LOG_TARGET_DEBUG(target, " XLEN=%d, misa=0x%" PRIx64, riscv_xlen(target), r->misa);

	res = examine_vlenb(target);
	if (res != ERROR_OK)
		return res;

	riscv_reg_impl_init_vector_reg_type(target);

	res = examine_mtopi(target);
	if (res != ERROR_OK)
		return res;

	for (uint32_t regno = 0; regno < target->reg_cache->num_regs; ++regno) {
		res = init_cache_entry(target, regno);
		if (res != ERROR_OK)
			return res;
	}

	res = riscv_reg_impl_expose_csrs(target);
	if (res != ERROR_OK)
		return res;

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
