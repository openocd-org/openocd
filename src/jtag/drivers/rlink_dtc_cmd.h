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

#ifndef OPENOCD_JTAG_DRIVERS_RLINK_DTC_CMD_H
#define OPENOCD_JTAG_DRIVERS_RLINK_DTC_CMD_H

/* A command position with the high nybble of 0x0 is reserved for an error condition.
 * If executed, it stops the DTC and raises the ERROR flag */

#define DTC_CMD_SHIFT_TMS_BYTES(bytes)	((0x1 << 4) | ((bytes) - 1))
/* Shift 1-16 bytes out TMS. TDI is 0. */
/* Bytes to shift follow. */

#define DTC_CMD_SHIFT_TDI_BYTES(bytes)	((0x2 << 4) | ((bytes) - 1))
/* Shift 1-16 bytes out TDI. TMS is 0. */
/* Bytes to shift follow. */

#define DTC_CMD_SHIFT_TDI_AND_TMS_BYTES(bytes)	((0x3 << 4) | ((bytes) - 1))
/* Shift 1-16 byte pairs out TDI and TMS. */
/* Byte pairs to shift follow in TDI, TMS order. */

#define DTC_CMD_SHIFT_TDO_BYTES(bytes)	((0x4 << 4) | ((bytes) - 1))
/* Shift 1-16 bytes in TDO. TMS is unaffected. */
/* Reply buffer contains bytes shifted in. */

#define DTC_CMD_SHIFT_TDIO_BYTES(bytes)	((0x6 << 4) | ((bytes) - 1))
/* Shift 1-16 bytes out TDI and in TDO. TMS is unaffected. */

#define DTC_CMD_SHIFT_TMS_TDI_BIT_PAIR(tms, tdi, tdo)	((0x8 << 4) | (\
		(tms) ? (1 << 0) : 0	\
) | (\
		(tdi) ? (1 << 1) : 0	\
) | (\
		(tdo) ? (1 << 3) : 0	\
))
/* Single bit shift.
 * tms and tdi are the levels shifted out on TMS and TDI, respectively.
 * tdo indicates whether a byte will be returned in the reply buffer with its
 * least significant bit set to reflect TDO
 * Care should be taken when tdo is zero, as the underlying code actually does put
 * that byte in the reply buffer. Setting tdo to zero just moves the pointer back.
 * The result is that if this command is executed when the reply buffer is already full,
 * a byte will be written erroneously to memory not belonging to the reply buffer.
 * This could be worked around at the expense of DTC code space and speed. */

#define DTC_CMD_SHIFT_TMS_BITS(bits)	((0x9 << 4) | ((bits) - 1))
/* Shift 1-8 bits out TMS. */
/* Bits to be shifted out are left justified in the following byte. */

#define DTC_CMD_SHIFT_TDIO_BITS(bits)	((0xe << 4) | ((bits) - 1))
/* Shift 1-8 bits out TDI and in TDO, TMS is unaffected. */
/* Bits to be shifted out are left justified in the following byte. */
/* Bits shifted in are right justified in the byte placed in the reply buffer. */

#define DTC_CMD_STOP			(0xf << 4)
/* Stop processing the command buffer and wait for the next one. */
/* A shared status byte is updated with bit 0 set when this has happened,
 * and it is cleared when a new command buffer becomes ready.
 * The host can poll that byte to see when it is safe to read a reply. */

#endif /* OPENOCD_JTAG_DRIVERS_RLINK_DTC_CMD_H */
