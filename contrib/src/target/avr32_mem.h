/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2010 by Oleksandr Tymoshenko <gonzo@bluezbox.com>       *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_AVR32_MEM_H
#define OPENOCD_TARGET_AVR32_MEM_H

int avr32_jtag_read_memory32(struct avr32_jtag *jtag_info,
		uint32_t addr, int count, uint32_t *buffer);
int avr32_jtag_read_memory16(struct avr32_jtag *jtag_info,
		uint32_t addr, int count, uint16_t *buffer);
int avr32_jtag_read_memory8(struct avr32_jtag *jtag_info,
		uint32_t addr, int count, uint8_t *buffer);

int avr32_jtag_write_memory32(struct avr32_jtag *jtag_info,
		uint32_t addr, int count, const uint32_t *buffer);
int avr32_jtag_write_memory16(struct avr32_jtag *jtag_info,
		uint32_t addr, int count, const uint16_t *buffer);
int avr32_jtag_write_memory8(struct avr32_jtag *jtag_info,
		uint32_t addr, int count, const uint8_t *buffer);

#endif /* OPENOCD_TARGET_AVR32_MEM_H */
