/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2019 by Mete Balci                                      *
 *   metebalci@gmail.com                                                   *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_AARCH64_DISASSEMBLER_H
#define OPENOCD_TARGET_AARCH64_DISASSEMBLER_H

#include "target.h"

int a64_disassemble(
		struct command_invocation *cmd,
		struct target *target,
		target_addr_t address,
		size_t count);

#endif /* OPENOCD_TARGET_AARCH64_DISASSEMBLER_H */
