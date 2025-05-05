/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_TARGET_RISCV_RISCV_013_REG_H
#define OPENOCD_TARGET_RISCV_RISCV_013_REG_H

#include "target/target.h"
#include "gdb_regs.h"
#include "riscv.h"

/**
 * This file describes additional register cache interface available to the
 * RISC-V Debug Specification v0.13+ targets.
 */

/**
 * This function assumes target is halted.
 * After this function all registers can be safely accessed via functions
 * described here and in `riscv_reg.h`.
 */
int riscv013_reg_examine_all(struct target *target);

/**
 * This function is used to save the value of a register in cache. The register
 * is marked as dirty, and writeback is delayed for as long as possible.
 * Generally used to save registers before program buffer execution.
 *
 * TODO: The interface should be restricted in such a way that only GPRs can be
 * saved.
 */
int riscv013_reg_save(struct target *target, enum gdb_regno regid);

#endif /* OPENOCD_TARGET_RISCV_RISCV_013_REG_H */
