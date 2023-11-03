// SPDX-License-Identifier: GPL-2.0-or-later
/****************************************************************************
	File : delay.c															*
	Contents : Delays handling fucntions code for NanoXplore				*
	USB-JTAG ANGIE adapter hardware.										*
	Based on openULINK project code by: Martin Schmoelzer.					*
	Copyright 2023, Ahmed Errached BOUDJELIDA, NanoXplore SAS.				*
	<aboudjelida@nanoxplore.com>											*
	<ahmederrachedbjld@gmail.com>											*
*****************************************************************************/

#include "delay.h"
#include <mcs51/compiler.h>

void syncdelay(uint8_t count)
{
	for (uint8_t i = 0; i < count; i++)
		NOP();
}

void delay_5us(void)
{
	NOP();
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
