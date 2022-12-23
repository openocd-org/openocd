/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2010 by Oleksandr Tymoshenko <gonzo@bluezbox.com>       *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_AVR32_REGS_H
#define OPENOCD_TARGET_AVR32_REGS_H

enum avr32_reg_nums {
	AVR32_REG_R0 = 0,
	AVR32_REG_R1,
	AVR32_REG_R2,
	AVR32_REG_R3,
	AVR32_REG_R4,
	AVR32_REG_R5,
	AVR32_REG_R6,
	AVR32_REG_R7,
	AVR32_REG_R8,
	AVR32_REG_R9,
	AVR32_REG_R10,
	AVR32_REG_R11,
	AVR32_REG_R12,
	AVR32_REG_SP,
	AVR32_REG_LR,
	AVR32_REG_PC,
	AVR32_REG_SR,
};

int avr32_jtag_read_regs(struct avr32_jtag *jtag_info, uint32_t *regs);
int avr32_jtag_write_regs(struct avr32_jtag *jtag_info, uint32_t *regs);

#endif /* OPENOCD_TARGET_AVR32_REGS_H */
