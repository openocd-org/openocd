/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   ESP32 target for OpenOCD                                              *
 *   Copyright (C) 2017 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ESP32_H
#define OPENOCD_TARGET_ESP32_H

#include <target/xtensa/xtensa_regs.h>

#define ESP32_DROM_LOW             0x3F400000
#define ESP32_DROM_HIGH            0x3F800000
#define ESP32_IROM_LOW             0x400D0000
#define ESP32_IROM_HIGH            0x40400000

/* Number of registers returned directly by the G command
 * Corresponds to the amount of regs listed in regformats/reg-xtensa.dat in the gdb source */
#define ESP32_NUM_REGS_G_COMMAND   105

enum esp32_reg_id {
	/* chip specific registers that extend ISA go after ISA-defined ones */
	ESP32_REG_IDX_EXPSTATE = XT_USR_REG_START,
	ESP32_REG_IDX_F64R_LO,
	ESP32_REG_IDX_F64R_HI,
	ESP32_REG_IDX_F64S,
	ESP32_NUM_REGS,
};

#endif	/* OPENOCD_TARGET_ESP32_H */
