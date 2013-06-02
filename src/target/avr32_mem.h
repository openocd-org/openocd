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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifndef AVR32_MEM
#define AVR32_MEM

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

#endif /* AVR32_MEM */
