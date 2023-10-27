/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Xtensa Target File-I/O Support for OpenOCD                            *
 *   Copyright (C) 2020-2023 Cadence Design Systems, Inc.                  *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_XTENSA_FILEIO_H
#define OPENOCD_TARGET_XTENSA_FILEIO_H

#include <target/target.h>
#include <helper/command.h>
#include "xtensa.h"

#define XTENSA_SYSCALL_OP_REG		XT_REG_IDX_A2
#define XTENSA_SYSCALL_RETVAL_REG	XT_REG_IDX_A2
#define XTENSA_SYSCALL_ERRNO_REG	XT_REG_IDX_A3

#define XTENSA_SYSCALL_OPEN			(-2)
#define XTENSA_SYSCALL_CLOSE		(-3)
#define XTENSA_SYSCALL_READ			(-4)
#define XTENSA_SYSCALL_WRITE		(-5)
#define XTENSA_SYSCALL_LSEEK		(-6)
#define XTENSA_SYSCALL_RENAME		(-7)
#define XTENSA_SYSCALL_UNLINK		(-8)
#define XTENSA_SYSCALL_STAT			(-9)
#define XTENSA_SYSCALL_FSTAT		(-10)
#define XTENSA_SYSCALL_GETTIMEOFDAY	(-11)
#define XTENSA_SYSCALL_ISATTY		(-12)
#define XTENSA_SYSCALL_SYSTEM		(-13)

int xtensa_fileio_init(struct target *target);
int xtensa_fileio_detect_proc(struct target *target);
int xtensa_get_gdb_fileio_info(struct target *target, struct gdb_fileio_info *fileio_info);
int xtensa_gdb_fileio_end(struct target *target, int retcode, int fileio_errno, bool ctrl_c);

#endif	/* OPENOCD_TARGET_XTENSA_FILEIO_H */
