/***************************************************************************
 *   Copyright (C) 2011 by Martin Schmoelzer                               *
 *   <martin.schmoelzer@student.tuwien.ac.at>                              *
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

#ifndef __JTAG_H
#define __JTAG_H

#include <stdint.h>

#define NOP { __asm nop __endasm; }

void jtag_scan_in(uint8_t out_offset, uint8_t in_offset);
void jtag_slow_scan_in(uint8_t out_offset, uint8_t in_offset);

void jtag_scan_out(uint8_t out_offset);
void jtag_slow_scan_out(uint8_t out_offset);

void jtag_scan_io(uint8_t out_offset, uint8_t in_offset);
void jtag_slow_scan_io(uint8_t out_offset, uint8_t in_offset);

void jtag_clock_tck(uint16_t count);
void jtag_slow_clock_tck(uint16_t count);
void jtag_clock_tms(uint8_t count, uint8_t sequence);
void jtag_slow_clock_tms(uint8_t count, uint8_t sequence);

uint16_t  jtag_get_signals(void);
void jtag_set_signals(uint8_t low, uint8_t high);

void jtag_configure_tck_delay(uint8_t scan_in, uint8_t scan_out,
		uint8_t scan_io, uint8_t tck, uint8_t tms);

#endif
