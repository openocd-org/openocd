// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2007 by Pavel Chromy                                    *
 *   chromy@asix.cz                                                        *
 ***************************************************************************/
#include "dcc.h"


/* debug channel read (debugger->MCU) */
uint32 dcc_rd(void)
{
	volatile uint32 dcc_reg;

	do {
		asm volatile ("mrc p14, 0, %0, C0, C0" : "=r" (dcc_reg) :);
	} while ((dcc_reg&1) == 0);

	asm volatile ("mrc p14, 0, %0, C1, C0" : "=r" (dcc_reg) :);
	return dcc_reg;
}


/* debug channel write (MCU->debugger) */
int dcc_wr(uint32 data)
{
	volatile uint32 dcc_reg;

	do {
		asm volatile ("mrc p14, 0, %0, C0, C0" : "=r" (dcc_reg) :);
		/* operation controlled by master, cancel operation
			 upon reception of data for immediate response */
		if (dcc_reg&1) return -1;
	} while (dcc_reg&2);

	asm volatile ("mcr p14, 0, %0, C1, C0" : : "r" (data));
	return 0;
}
