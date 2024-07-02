/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_TARGET_RISCV_RISCV_013_H
#define OPENOCD_TARGET_RISCV_RISCV_013_H

#include "riscv.h"

/* TODO: These functions should be replaced here by access methods that can be
 * reused by other modules (e.g. a function writing an abstract commands, a
 * function filling/executing program buffer, etc.), while the specifics on how
 * to use these general-purpose version-specific methods to get a register's
 * value will be in `riscv-013_reg.c`.
 */
int riscv013_get_register(struct target *target,
		riscv_reg_t *value, enum gdb_regno rid);
int riscv013_get_register_buf(struct target *target, uint8_t *value,
		enum gdb_regno regno);
int riscv013_set_register(struct target *target, enum gdb_regno rid,
		riscv_reg_t value);
int riscv013_set_register_buf(struct target *target, enum gdb_regno regno,
		const uint8_t *value);
uint32_t riscv013_access_register_command(struct target *target, uint32_t number,
		unsigned int size, uint32_t flags);
int riscv013_execute_abstract_command(struct target *target, uint32_t command,
		uint32_t *cmderr);

#endif /* OPENOCD_TARGET_RISCV_RISCV_013_H */
