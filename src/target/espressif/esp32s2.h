/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP32-S2 target for OpenOCD                                           *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
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
