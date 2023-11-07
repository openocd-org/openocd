/* SPDX-License-Identifier: GPL-2.0-or-later */
/****************************************************************************
	File : jtag.h															*
	Contents : Jtag handling functions header file for NanoXplore			*
	USB-JTAG ANGIE adapter hardware.										*
	Based on openULINK project code by: Martin Schmoelzer.					*
	Copyright 2023, Ahmed Errached BOUDJELIDA, NanoXplore SAS.				*
	<aboudjelida@nanoxplore.com>											*
	<ahmederrachedbjld@gmail.com>											*
*****************************************************************************/

#ifndef __JTAG_H
#define __JTAG_H

#include <stdint.h>

uint16_t jtag_get_signals(void);
void jtag_configure_tck_delay(uint8_t scan_in, uint8_t scan_out,
	uint8_t scan_io, uint8_t tck, uint8_t tms);
void jtag_clock_tms(uint8_t count, uint8_t sequence);
void jtag_slow_clock_tms(uint8_t count, uint8_t sequence);
void jtag_set_signals(uint8_t low, uint8_t high);
void jtag_clock_tck(uint16_t count);
void jtag_slow_clock_tck(uint16_t count);
void jtag_scan_in(uint8_t out_offset, uint8_t in_offset);
void jtag_scan_out(uint8_t out_offset);
void jtag_scan_io(uint8_t out_offset, uint8_t in_offset);
void jtag_slow_scan_in(uint8_t out_offset, uint8_t in_offset);
void jtag_slow_scan_out(uint8_t out_offset);
void jtag_slow_scan_io(uint8_t out_offset, uint8_t in_offset);
#endif
