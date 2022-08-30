// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *   Copyright (C) 2008 by Frederik Kriewtz                                *
 *   frederik@kriewitz.eu                                                  *
 ***************************************************************************/

#include "dcc_stdio.h"

/* enable openocd debugmsg at the gdb prompt:
 * monitor target_request debugmsgs enable
 *
 * create a trace point:
 * monitor trace point 1
 *
 * to show how often the trace point was hit:
 * monitor trace point
*/

int main(void)
{
	dbg_write_str("hello world");

	dbg_write_char('t');
	dbg_write_char('e');
	dbg_write_char('s');
	dbg_write_char('t');
	dbg_write_char('\n');

	unsigned long test_u32 = 0x01234567;
	dbg_write_u32(&test_u32, 1);

	static const unsigned short test_u16[] = {0x0123, 0x4567, 0x89AB, 0xCDEF, 0x0123, 0x4567, 0x89AB, 0xCDEF};
	dbg_write_u16(test_u16, 8);

	static const unsigned char test_u8[] = {0x00, 0x11, 0x22, 0x33, 0x44, 0x55, 0x66, 0x77, 0x88, 0x99, 0xAA, 0xBB, 0xCC, 0XDD, 0xEE, 0xFF};
	dbg_write_u8(test_u8, 16);

	while(1)
	{
		dbg_trace_point(0);
	}
}
