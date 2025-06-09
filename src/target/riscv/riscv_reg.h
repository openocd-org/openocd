/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_TARGET_RISCV_RISCV_REG_H
#define OPENOCD_TARGET_RISCV_RISCV_REG_H

#include "target/target.h"
#include "target/register.h"

/**
 * This file describes the register cache interface available to the RISC-V
 * target. Functions declared here should be safe to use once register cache is
 * completely initialized and may be used with caution during register cache
 * initialization.
 */

/** Return the name of the register by it's number in register cache. */
const char *riscv_reg_gdb_regno_name(const struct target *target, enum gdb_regno regno);

/** Free register cache and associated structures. */
void riscv_reg_free_all(struct target *target);

/** Write all dirty registers to the target. */
int riscv_reg_flush_all(struct target *target);
/**
 * Check whether there are any dirty registers in the OpenOCD's register cache.
 * In addition, all dirty registers will be reported to the log using the
 * supplied "log_level".
 */
bool riscv_reg_cache_any_dirty(const struct target *target, int log_level);
/**
 * Invalidate all registers - forget their cached register values.
 * WARNING: If a register was dirty, its walue will be silently lost!
 */
void riscv_reg_cache_invalidate_all(struct target *target);
/**
 * Set the register value. For cacheable registers, only the cache is updated
 * (write-back mode).
 */
int riscv_reg_set(struct target *target, enum gdb_regno i, riscv_reg_t v);
/**
 * Set the register value and immediately write it to the target
 * (write-through mode).
 */
int riscv_reg_write(struct target *target, enum gdb_regno i, riscv_reg_t v);
/** Get register, from the cache if it's in there. */
int riscv_reg_get(struct target *target, riscv_reg_t *value,
		enum gdb_regno r);

#endif /* OPENOCD_TARGET_RISCV_RISCV_REG_H */
