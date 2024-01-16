// SPDX-License-Identifier: GPL-2.0-or-later

/****************************************************************************
	File : protocol.c														*
	Contents : Jtag commands handling protocol code for NanoXplore			*
	USB-JTAG ANGIE adapter hardware.										*
	Based on openULINK project code by: Martin Schmoelzer.					*
	Copyright 2023, Ahmed Errached BOUDJELIDA, NanoXplore SAS.				*
	<aboudjelida@nanoxplore.com>											*
	<ahmederrachedbjld@gmail.com>											*
*****************************************************************************/

#include "usb.h"
#include "protocol.h"
#include "jtag.h"
#include "delay.h"
#include "io.h"
#include "msgtypes.h"
#include "reg_ezusb.h"
#include <serial.h>
#include <stdio.h>

/** Index in EP1 Bulk-OUT data buffer that contains the current command ID */
volatile uint8_t cmd_id_index;

/** Number of data bytes already in EP1 Bulk-IN buffer */
volatile uint8_t payload_index_in;

/**
 * Executes one command and updates global command indexes.
 *
 * @return true if this command was the last command.
 * @return false if there are more commands within the current contents of the
 * Bulk EP1-OUT data buffer.
 */
bool execute_command(void)
{
	uint8_t usb_out_bytecount, usb_in_bytecount;
	uint16_t signal_state = 0;
	uint16_t count;

	/* Most commands do not transfer IN data. To save code space, we write 0 to
	 * usb_in_bytecount here, then modify it in the switch statement below where
	 * necessary */
	usb_in_bytecount = 0;

	switch (EP1OUTBUF[cmd_id_index] /* Command ID */) {
	case CMD_SCAN_IN:
		usb_out_bytecount = 5;
		usb_in_bytecount = EP1OUTBUF[cmd_id_index + 1];
		jtag_scan_in((cmd_id_index + 1), payload_index_in);
		break;
	case CMD_SCAN_OUT:
		usb_out_bytecount = EP1OUTBUF[cmd_id_index + 1] + 5;
		jtag_scan_out(cmd_id_index + 1);
		break;
	case CMD_SCAN_IO:
		usb_in_bytecount = EP1OUTBUF[cmd_id_index + 1];
		usb_out_bytecount = usb_in_bytecount + 5;
		jtag_scan_io((cmd_id_index + 1), payload_index_in);
		break;
	case CMD_CLOCK_TMS:
		usb_out_bytecount = 2;
		jtag_clock_tms(EP1OUTBUF[cmd_id_index + 1], EP1OUTBUF[cmd_id_index + 2]);
		break;
	case CMD_CLOCK_TCK:
		usb_out_bytecount = 2;
		count = (uint16_t)EP1OUTBUF[cmd_id_index + 1];
		count |= ((uint16_t)EP1OUTBUF[cmd_id_index + 2]) << 8;
		jtag_clock_tck(count);
		break;
	case CMD_SLOW_SCAN_IN:
		usb_out_bytecount = 5;
		usb_in_bytecount = EP1OUTBUF[cmd_id_index + 1];
		jtag_slow_scan_in(cmd_id_index + 1, payload_index_in);
		break;
	case CMD_SLOW_SCAN_OUT:
		usb_out_bytecount = EP1OUTBUF[cmd_id_index + 1] + 5;
		jtag_slow_scan_out(cmd_id_index + 1);
		break;
	case CMD_SLOW_SCAN_IO:
		usb_in_bytecount = EP1OUTBUF[cmd_id_index + 1];
		usb_out_bytecount = usb_in_bytecount + 5;
		jtag_slow_scan_io(cmd_id_index + 1, payload_index_in);
		break;
	case CMD_SLOW_CLOCK_TMS:
		usb_out_bytecount = 2;
		jtag_slow_clock_tms(EP1OUTBUF[cmd_id_index + 1], EP1OUTBUF[cmd_id_index + 2]);
		break;
	case CMD_SLOW_CLOCK_TCK:
		usb_out_bytecount = 2;
		count = (uint16_t)EP1OUTBUF[cmd_id_index + 1];
		count |= ((uint16_t)EP1OUTBUF[cmd_id_index + 2]) << 8;
		jtag_slow_clock_tck(count);
		break;
	case CMD_SLEEP_US:
		usb_out_bytecount = 2;
		count = (uint16_t)EP1OUTBUF[cmd_id_index + 1];
		count |= ((uint16_t)EP1OUTBUF[cmd_id_index + 2]) << 8;
		delay_us(count);
		break;
	case CMD_SLEEP_MS:
		usb_out_bytecount = 2;
		count = (uint16_t)EP1OUTBUF[cmd_id_index + 1];
		count |= ((uint16_t)EP1OUTBUF[cmd_id_index + 2]) << 8;
		delay_ms(count);
		break;
	case CMD_GET_SIGNALS:
		usb_out_bytecount = 0;
		usb_in_bytecount = 2;
		signal_state = jtag_get_signals();
		EP1INBUF[payload_index_in] = (signal_state >> 8);
		EP1INBUF[payload_index_in + 1] = (signal_state & 0xFF);
		break;
	case CMD_SET_SIGNALS:
		usb_out_bytecount = 2;
		jtag_set_signals(EP1OUTBUF[cmd_id_index + 1], EP1OUTBUF[cmd_id_index + 2]);
		break;
	case CMD_CONFIGURE_TCK_FREQ:
		usb_out_bytecount = 5;
		jtag_configure_tck_delay(EP1OUTBUF[cmd_id_index + 1],	/* scan_in */
		EP1OUTBUF[cmd_id_index + 2],	/* scan_out */
		EP1OUTBUF[cmd_id_index + 3],	/* scan_io */
		EP1OUTBUF[cmd_id_index + 4],	/* clock_tck */
		EP1OUTBUF[cmd_id_index + 5]);	/* clock_tms */
		break;
	case CMD_TEST:
		usb_out_bytecount = 1;
		/* Do nothing... This command is only used to test if the device is ready
		 * to accept new commands */
		break;
	default:
		/* Should never be reached */
		usb_out_bytecount = 0;
		break;
	}

	/* Update EP1 Bulk-IN data byte count */
	payload_index_in += usb_in_bytecount;

	/* Determine if this was the last command */
	if ((cmd_id_index + usb_out_bytecount + 1) >= EP1OUTBC)
		return true;

	/* Not the last command, update cmd_id_index */
	cmd_id_index += (usb_out_bytecount + 1);
	return false;
}

/**
 * Forever wait for commands and execute them as they arrive.
 */
void command_loop(void)
{
	bool last_command;
	while (1) {
		cmd_id_index = 0;
		payload_index_in = 0;

		/* Wait until host sends Bulk-OUT packet */
		while ((!ep1_out) && (!ep6_out))
			;
		if (ep6_out) {
			/* Execute I2C command */
			i2c_recieve();
			ep6_out = false;
		}
		if (ep1_out) {
			ep1_out = false;
			/* Execute the commands */
			last_command = false;
			while (!last_command)
				last_command = execute_command();

			/* Send back EP1 Bulk-IN packet if required */
			if (payload_index_in > 0) {
				EP1INBC = payload_index_in;
				syncdelay(3);

				while (!ep1_in)
					;
				ep1_in = false;
			}

			/* Re-arm EP1-OUT after command execution */
			EP1OUTBC = 0;
			syncdelay(3);
			EP1OUTBC = 0;
			syncdelay(3);
		}
	}
}
