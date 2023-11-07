// SPDX-License-Identifier: GPL-2.0-or-later

/****************************************************************************
	File : main.c															*
	Contents : main code for NanoXplore USB-JTAG ANGIE adapter				*
	hardware.																*
	Based on openULINK project code by: Martin Schmoelzer.					*
	Copyright 2023, Ahmed Errached BOUDJELIDA, NanoXplore SAS.				*
	<aboudjelida@nanoxplore.com>											*
	<ahmederrachedbjld@gmail.com>											*
*****************************************************************************/

#include "usb.h"
#include "delay.h"
#include "protocol.h"
#include "reg_ezusb.h"
#include <serial.h>
#include <stdio.h>

extern void sudav_isr(void)__interrupt	SUDAV_ISR;
extern void sof_isr(void)__interrupt;
extern void sutok_isr(void)__interrupt;
extern void suspend_isr(void)__interrupt;
extern void usbreset_isr(void)__interrupt;
extern void highspeed_isr(void)__interrupt;
extern void ep0ack_isr(void)__interrupt;
extern void stub_isr(void)__interrupt;
extern void ep0in_isr(void)__interrupt;
extern void ep0out_isr(void)__interrupt;
extern void ep1in_isr(void)__interrupt;
extern void ep1out_isr(void)__interrupt;
extern void ep2_isr(void)__interrupt;
extern void ep4_isr(void)__interrupt;
extern void ep6_isr(void)__interrupt;
extern void ep8_isr(void)__interrupt;
extern void ibn_isr(void)__interrupt;
extern void ep0pingnak_isr(void)__interrupt;
extern void ep1pingnak_isr(void)__interrupt;
extern void ep2pingnak_isr(void)__interrupt;
extern void ep4pingnak_isr(void)__interrupt;
extern void ep6pingnak_isr(void)__interrupt;
extern void ep8pingnak_isr(void)__interrupt;
extern void errorlimit_isr(void)__interrupt;
extern void ep2piderror_isr(void)__interrupt;
extern void ep4piderror_isr(void)__interrupt;
extern void ep6piderror_isr(void)__interrupt;
extern void ep8piderror_isr(void)__interrupt;
extern void ep2pflag_isr(void)__interrupt;
extern void ep4pflag_isr(void)__interrupt;
extern void ep6pflag_isr(void)__interrupt;
extern void ep8pflag_isr(void)__interrupt;
extern void ep2eflag_isr(void)__interrupt;
extern void ep4eflag_isr(void)__interrupt;
extern void ep6eflag_isr(void)__interrupt;
extern void ep8eflag_isr(void)__interrupt;
extern void ep2fflag_isr(void)__interrupt;
extern void ep4fflag_isr(void)__interrupt;
extern void ep6fflag_isr(void)__interrupt;
extern void ep8fflag_isr(void)__interrupt;
extern void gpifcomplete_isr(void)__interrupt;
extern void gpifwaveform_isr(void)__interrupt;

void gpif_init(void);

int main(void)
{
	CPUCS = ((CPUCS & ~bmclkspd) | (CLK_48M << 3) | CLKOE);	/* required for sio0_init */
	sio0_init(57600);	/* needed for printf */

	ep_init();
	gpif_init();
	interrupt_init();
	io_init();

	/* Perform ReNumeration */
	USBCS |= (DISCON | RENUM);
	delay_ms(250);
	USBCS &= ~DISCON;

	/* Begin executing command(s). This function never returns. */
	command_loop();

	/* Never reached, but SDCC complains about missing return statement */
	return 0;
}
