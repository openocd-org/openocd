/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2009 Zachary T Welch                                    *
 *   zw@superlucidity.net                                                  *
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
#ifndef MINIDRIVER_H
#define MINIDRIVER_H

/* @page jtagminidriver JTAG Mini-Driver
 *
 * The JTAG minidriver interface allows the definition of alternate
 * interface functions, instead of the built-in asynchronous driver
 * module that is used by the standard JTAG interface drivers.
 *
 * In addtion to the functions defined in the c minidriver.h file, the
 * @c jtag_minidriver.h file must declare the following functions (or
 * define static inline versions of them):
 * - jtag_add_callback
 * - jtag_add_callback4
 * - interface_jtag_add_dr_out
 *  
 * The following core functions are declared in this file for use by
 * the minidriver and do @b not need to be defined by an implementation:
 * - default_interface_jtag_execute_queue()
 */

#ifdef HAVE_JTAG_MINIDRIVER_H

#include "jtag_minidriver.h"

static inline void interface_jtag_alloc_in_value32(scan_field_t *field)
{
	field->in_value = field->intmp;
}

static inline void interface_jtag_add_scan_check_alloc(scan_field_t *field)
{
	/* We're executing this synchronously, so try to use local storage. */
	if (field->num_bits > 32)
	{
		unsigned num_bytes = TAP_SCAN_BYTES(field->num_bits);
		field->in_value = (u8 *)malloc(num_bytes);
		field->allocated = 1;
	}
	else
		field->in_value = field->intmp;
}

#else

static inline void interface_jtag_alloc_in_value32(scan_field_t *field)
{
	field->in_value = (u8 *)cmd_queue_alloc(4);
}

static inline void interface_jtag_add_scan_check_alloc(scan_field_t *field)
{
	unsigned num_bytes = TAP_SCAN_BYTES(field->num_bits);
	field->in_value = (u8 *)cmd_queue_alloc(num_bytes);
}

extern void interface_jtag_add_dr_out(jtag_tap_t* tap,
		int num_fields, const int* num_bits, const u32* value,
		tap_state_t end_state);

extern void interface_jtag_add_callback(jtag_callback1_t f, u8 *in);

extern void interface_jtag_add_callback4(jtag_callback_t f, u8 *in,
		jtag_callback_data_t data1, jtag_callback_data_t data2,
		jtag_callback_data_t data3);

#endif

extern int interface_jtag_add_ir_scan(
		int num_fields, const scan_field_t* fields,
		tap_state_t endstate);
extern int interface_jtag_add_plain_ir_scan(
		int num_fields, const scan_field_t* fields,
		tap_state_t endstate);

extern int interface_jtag_add_dr_scan(
		int num_fields, const scan_field_t* fields,
		tap_state_t endstate);
extern int interface_jtag_add_plain_dr_scan(
		int num_fields, const scan_field_t* fields,
		tap_state_t endstate);

extern int interface_jtag_add_tlr(void);
extern int interface_jtag_add_pathmove(int num_states, const tap_state_t* path);
extern int interface_jtag_add_runtest(int num_cycles, tap_state_t endstate);

/**
 * This drives the actual srst and trst pins. srst will always be 0
 * if jtag_reset_config & RESET_SRST_PULLS_TRST != 0 and ditto for
 * trst.
 *
 * the higher level jtag_add_reset will invoke jtag_add_tlr() if
 * approperiate
 */
extern int interface_jtag_add_reset(int trst, int srst);
extern int interface_jtag_add_end_state(tap_state_t endstate);
extern int interface_jtag_add_sleep(u32 us);
extern int interface_jtag_add_clocks(int num_cycles);
extern int interface_jtag_execute_queue(void);

/**
 * Calls the interface callback to execute the queue.  This routine
 * is used by the JTAG driver layer and should not be called directly.
 */
extern int default_interface_jtag_execute_queue(void);


#endif // MINIDRIVER_H
