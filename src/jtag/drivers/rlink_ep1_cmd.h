/***************************************************************************
 *   Copyright (C) 2008 Lou Deluxe                                         *
 *   lou.openocd012@fixit.nospammail.net                                   *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef OPENOCD_JTAG_DRIVERS_RLINK_EP1_CMD_H
#define OPENOCD_JTAG_DRIVERS_RLINK_EP1_CMD_H

/*
 * Command opcodes that can be sent over endpoint 1.
 * This codifies information provided by Rob Brown <rob@cobbleware.com>.
 * The buffer can contain several of these, but only one which returns data.
 * Some of these opcodes have arguments, which follow immediately.
 * If shorter than the packet size, trailing positions should be zero-filled.
 */

/* LED update enables:
 *  When enabled, each LED is updated automatically.
 *  When not enabled, each LED can be controlled manually with EP1_CMD_SET_PORTD_LEDS.
 */
#define EP1_CMD_LEDUE_BOTH				(0x05)
/* EP1_CMD_LEDUE_NONE has the side effect of turning the LEDs on */
#define EP1_CMD_LEDUE_NONE				(0x06)
#define EP1_CMD_LEDUE_ERROR				(0x17)
#define EP1_CMD_LEDUE_BUSY				(0x18)

#define EP1_CMD_DTC_STOP				(0x0b)
#define EP1_CMD_DTC_LOAD				(0x0c)
#define EP1_CMD_DTC_CALL				(0x0d)
#define EP1_CMD_SET_UPLOAD				(0x0f)
#define EP1_CMD_SET_DOWNLOAD			(0x10)
#define EP1_CMD_DTC_WAIT				(0x12)
#define EP1_CMD_DTC_GET_STATUS			(0x15)
/* a quick way to just read back one byte */
#define EP1_CMD_DTC_GET_CACHED_STATUS	(0x16)

/* Writes upper 2 bits (SHDN and SEL) of port D with argument */
#define EP1_CMD_SET_PORTD_VPP			(0x19)
/* Writes lower 2 bits (BUSY and ERROR) of port D with argument */
#define EP1_CMD_SET_PORTD_LEDS			(0x1a)

#define EP1_CMD_MEMORY_READ				(0x28)
#define EP1_CMD_MEMORY_WRITE			(0x29)
#define EP1_CMD_GET_FWREV				(0xfe)
#define EP1_CMD_GET_SERIAL				(0xff)

#endif /* OPENOCD_JTAG_DRIVERS_RLINK_EP1_CMD_H */
