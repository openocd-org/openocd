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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#include "main.h"

#include "io.h"
#include "usb.h"
#include "protocol.h"

extern void sudav_isr(void)    __interrupt SUDAV_ISR;
extern void sof_isr(void)      __interrupt;
extern void sutok_isr(void)    __interrupt;
extern void suspend_isr(void)  __interrupt;
extern void usbreset_isr(void) __interrupt;
extern void ibn_isr(void)      __interrupt;
extern void ep0in_isr(void)    __interrupt;
extern void ep0out_isr(void)   __interrupt;
extern void ep1in_isr(void)    __interrupt;
extern void ep1out_isr(void)   __interrupt;
extern void ep2in_isr(void)    __interrupt;
extern void ep2out_isr(void)   __interrupt;
extern void ep3in_isr(void)    __interrupt;
extern void ep3out_isr(void)   __interrupt;
extern void ep4in_isr(void)    __interrupt;
extern void ep4out_isr(void)   __interrupt;
extern void ep5in_isr(void)    __interrupt;
extern void ep5out_isr(void)   __interrupt;
extern void ep6in_isr(void)    __interrupt;
extern void ep6out_isr(void)   __interrupt;
extern void ep7in_isr(void)    __interrupt;
extern void ep7out_isr(void)   __interrupt;

void io_init(void)
{
	/* PORTxCFG register bits select alternate functions (1 == alternate function,
	 *                                                    0 == standard I/O)
	 * OEx register bits turn on/off output buffer (1 == output, 0 == input)
	 * OUTx register bits determine pin state of output
	 * PINx register bits reflect pin state (high == 1, low == 0) */

	/* PORT A */
	PORTACFG = PIN_OE;
	OEA = PIN_U_OE | PIN_OE | PIN_RUN_LED | PIN_COM_LED;
	OUTA = PIN_RUN_LED | PIN_COM_LED;

	/* PORT B */
	PORTBCFG = 0x00;
	OEB = PIN_TDI | PIN_TMS | PIN_TCK | PIN_TRST | PIN_BRKIN | PIN_RESET
		| PIN_OCDSE;

	/* TRST and RESET signals are low-active but inverted by hardware, so we clear
	 * these signals here! */
	OUTB = 0x00;

	/* PORT C */
	PORTCCFG = PIN_WR;
	OEC = PIN_TXD0 | PIN_WR;
	OUTC = 0x00;
}

int main(void)
{
	io_init();
	usb_init();

	/* Enable Interrupts */
	EA = 1;

	/* Begin executing command(s). This function never returns. */
	command_loop();

	/* Never reached, but SDCC complains about missing return statement */
	return 0;
}
