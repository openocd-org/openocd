/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2011 by Martin Schmoelzer                               *
 *   <martin.schmoelzer@student.tuwien.ac.at>                              *
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
