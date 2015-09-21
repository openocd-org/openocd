/***************************************************************************
 *   Copyright (C) 2010 by Oleksandr Tymoshenko <gonzo@bluezbox.com>       *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
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
