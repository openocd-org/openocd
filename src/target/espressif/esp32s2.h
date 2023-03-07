/***************************************************************************
 *   ESP32-S2 target for OpenOCD                                           *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
 *   Author: Alexey Gerenkov <alexey@espressif.com>                        *
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

#ifndef OPENOCD_TARGET_ESP32S2_H
#define OPENOCD_TARGET_ESP32S2_H

#include <target/xtensa/xtensa_regs.h>

#define ESP32_S2_DROM_LOW   0x3f000000
#define ESP32_S2_DROM_HIGH  0x3ff80000
#define ESP32_S2_IROM_LOW   0x40080000
#define ESP32_S2_IROM_HIGH  0x40800000

/* Number of registers returned directly by the G command
 * Corresponds to the amount of regs listed in regformats/reg-xtensa.dat in the gdb source */
#define ESP32_S2_NUM_REGS_G_COMMAND   72

enum esp32s2_reg_id {
	/* chip specific registers that extend ISA go after ISA-defined ones */
	ESP32_S2_REG_IDX_GPIOOUT = XT_USR_REG_START,
	ESP32_S2_NUM_REGS,
};

#endif	/* OPENOCD_TARGET_ESP32S2_H */
