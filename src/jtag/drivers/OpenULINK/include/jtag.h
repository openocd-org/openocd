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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef __JTAG_H
#define __JTAG_H

#include "shorttypes.h"

#define NOP {__asm nop __endasm;}

void jtag_scan_in(u8 out_offset, u8 in_offset);
void jtag_scan_out(u8 out_offset);
void jtag_scan_io(u8 out_offset, u8 in_offset);

void jtag_slow_scan_in(u8 scan_size_bytes, u8 tdo_index, u8 scan_options);
void jtag_slow_scan_out(u8 scan_size_bytes, u8 tdi_index, u8 scan_options);
void jtag_slow_scan_io(u8 scan_size_bytes, u8 tdi_index, u8 tdo_index,
    u8 scan_options);

void jtag_clock_tck(u16 count);
void jtag_clock_tms(u8 count, u8 sequence);
void jtag_slow_clock_tms(u8 count, u8 sequence);

u16  jtag_get_signals(void);
void jtag_set_signals(u8 low, u8 high);

void jtag_configure_tck_delay(u8 scan, u8 tck, u8 tms);

#endif
