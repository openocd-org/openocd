// SPDX-License-Identifier: LGPL-2.1-or-later

/*
 * This code was taken from the fx2lib project from this link:
 * https://github.com/djmuhlestein/fx2lib
 *
 * Copyright (C) 2009 Ubixum, Inc.
*/

#include <reg_ezusb.h>
#include <fx2macros.h>
#include <serial.h>
#include <stdint.h>
/**
 * using the comp port implies that timer 2 will be used as
 * a baud rate generator.  (Don't use timer 2)
 **/
void sio0_init(uint32_t baud_rate) __critical
{
	uint16_t hl;     /* hl value for reload */
	uint8_t mult;   /* multiplier for clock speed */
	uint32_t tmp;   /* scratch for mult/divide */

	mult = (CPUFREQ == CLK_12M) ? 1 : ((CPUFREQ == CLK_24M) ? 2 : 4);

	/* set the clock rate */
	/* use clock 2 */
	RCLK = 1; TCLK = 1;
	tmp = mult * 375000L * 2;
	tmp /= baud_rate;
	tmp += 1;
	tmp /= 2;
	hl = 0xFFFF - (uint16_t)tmp;
	RCAP2H = (uint8_t)(((uint16_t)(hl) >> 8) & 0xff);

	/* seems that the 24/48mhz calculations are always one less than suggested values */
  /* trm table 14-16 */
	RCAP2L = ((uint8_t)((uint16_t)(hl) & 0xff)) + (mult > 0 ? 1 : 0);

  /* start the timer */
	TR2 = 1;

	/* set up the serial port	*/
	SM0 = 0; SM1 = 1; /* serial mode 1 (asyncronous)	*/
	SM2 = 0 ;       /* has to do with receiving */
	REN = 1 ;       /* to enable receiving */
	PCON |= 0x80;    /* SET SMOD0, baud rate doubler */
	TI = 1;         /* we send initial byte */
}

int getchar(void)
{
	char c;
	while (!RI)
		;
	c = SBUF0;
	RI = 0;
	return c;
}

void _transchar(char c)
{
	while (!TI)
		; /* wait for TI=1 */
	TI = 0;
	SBUF0 = c;
}

int putchar (char c)
{
	if (c == '\n')
		_transchar('\r'); /* transmit \r\n */
	_transchar(c);
	if (c == '\r')
		_transchar('\n'); /* transmit \r\n */
	return c;
}
