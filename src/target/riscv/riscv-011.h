/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_TARGET_RISCV_RISCV_011_H
#define OPENOCD_TARGET_RISCV_RISCV_011_H

#include "riscv.h"
#include "gdb_regs.h"
#include "target/target.h"

int riscv011_get_register(struct target *target, riscv_reg_t *value,
		enum gdb_regno regid);
int riscv011_set_register(struct target *target, enum gdb_regno regid,
		riscv_reg_t value);

#endif /* OPENOCD_TARGET_RISCV_RISCV_011_H */
