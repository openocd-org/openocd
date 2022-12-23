// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2011 by Martin Schmoelzer                               *
 *   <martin.schmoelzer@student.tuwien.ac.at>                              *
 ***************************************************************************/

#include "delay.h"

void delay_5us(void)
{
	NOP;
}

void delay_1ms(void)
{
	uint16_t i;

	for (i = 0; i < 598; i++)
		;
}

void delay_us(uint16_t delay)
{
	uint16_t i;
	uint16_t maxcount = (delay / 5);

	for (i = 0; i < maxcount; i++)
		delay_5us();
}

void delay_ms(uint16_t delay)
{
	uint16_t i;

	for (i = 0; i < delay; i++)
		delay_1ms();
}
