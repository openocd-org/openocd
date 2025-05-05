/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_TARGET_RISCV_DEBUG_REG_PRINTER_H
#define OPENOCD_TARGET_RISCV_DEBUG_REG_PRINTER_H

#include "debug_defines.h"

enum riscv_debug_reg_show {
	RISCV_DEBUG_REG_SHOW_ALL,
	RISCV_DEBUG_REG_HIDE_ALL_0,
	RISCV_DEBUG_REG_HIDE_UNNAMED_0,
};

/**
 * This function is used to fill a buffer with a decoded string representation
 * of register's value.
 * @param buf      If non-NULL, the buffer to write the string into.
 *                 Otherwise, the function does not perform any writes,
 *                 it just calculates the number of characters used.
 * @param reg_ordinal
 *                 The ordinal of the register.
 * @param context  The structure, containing the information about the target,
 *                 necessary to decode the register.
 * @param value    The value to be decoded.
 *
 * Returns the number of characters used by the string representation
 * (excluding '\0').
 *
 * Example:
 * const struct riscv_debug_reg_ctx_t context = {
 *     .abits = { .value = <abits value>, .is_set = true }
 * };
 * char buf[riscv_debug_reg_to_s(NULL, DTM_DMI_ORDINAL, context, <dmi value>) + 1]
 * riscv_debug_reg_to_s(buf, DTM_DMI_ORDINAL, context, <dmi value>);
 */
unsigned int riscv_debug_reg_to_s(char *buf, enum riscv_debug_reg_ordinal reg_ordinal,
		riscv_debug_reg_ctx_t context, uint64_t value,
		enum riscv_debug_reg_show show);

#endif /* OPENOCD_TARGET_RISCV_DEBUG_REG_PRINTER_H */
