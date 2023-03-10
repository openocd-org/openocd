/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2023 by Rapid Silicon                                   *
 *   chernyee.kok@rapidsilicon.com                                         *
 ***************************************************************************/

#ifndef OPENOCD_PLD_GEMINI_H
#define OPENOCD_PLD_GEMINI_H

#include <jtag/jtag.h>
#include <target/target.h>
#include <target/riscv/riscv.h>

struct gemini_pld_device {
	struct target *target;
	struct jtag_tap *tap;
};

#endif /* OPENOCD_PLD_GEMINI_H */
