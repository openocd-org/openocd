/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_TARGET_RISCV_RISCV_011_REG_H
#define OPENOCD_TARGET_RISCV_RISCV_011_REG_H

#include "target/target.h"

/**
 * This file describes additional register cache interface available to the
 * RISC-V Debug Specification v0.11 targets.
 */

/**
 * Initialize register cache. After this function all registers can be
 * safely accessed via functions described here and in `riscv_reg.h`.
 */
int riscv011_reg_init_all(struct target *target);

#endif /* OPENOCD_TARGET_RISCV_RISCV_011_REG_H */
