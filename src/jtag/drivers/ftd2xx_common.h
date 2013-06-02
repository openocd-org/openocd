/***************************************************************************
 *   Copyright (C) 2011 by Spencer Oliver <spen@spen-soft.co.uk>           *
 *                                                                         *
 *   Written by Arnim Laeuger, 2008 (from urjtag)                          *
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

#ifndef _FTD2XX_COMMON_H
#define _FTD2XX_COMMON_H

#if ((BUILD_FT2232_FTD2XX == 1) || (BUILD_PRESTO_FTD2XX == 1) || (BUILD_USB_BLASTER_FTD2XX == 1))
#include <ftd2xx.h>

static const char *ftd2xx_status_string(FT_STATUS status)
{
	switch (status) {
		case FT_OK:							return "OK";
		case FT_INVALID_HANDLE:				return "invalid handle";
		case FT_DEVICE_NOT_FOUND:			return "device not found";
		case FT_DEVICE_NOT_OPENED:			return "device not opened";
		case FT_IO_ERROR:					return "io error";
		case FT_INSUFFICIENT_RESOURCES:		return "insufficient resources";
		case FT_INVALID_PARAMETER:			return "invalid parameter";
		case FT_INVALID_BAUD_RATE:			return "invalid baud rate";

		case FT_DEVICE_NOT_OPENED_FOR_ERASE: return "device not opened for erase";
		case FT_DEVICE_NOT_OPENED_FOR_WRITE: return "device not opened for write";
		case FT_FAILED_TO_WRITE_DEVICE:		return "failed to write device";
		case FT_EEPROM_READ_FAILED:			return "eeprom read failed";
		case FT_EEPROM_WRITE_FAILED:		return "eeprom write failed";
		case FT_EEPROM_ERASE_FAILED:		return "eeprom erase failed";
		case FT_EEPROM_NOT_PRESENT:			return "eeprom not present";
		case FT_EEPROM_NOT_PROGRAMMED:		return "eeprom not programmed";
		case FT_INVALID_ARGS:				return "invalid args";
		case FT_NOT_SUPPORTED:				return "not supported";
		case FT_OTHER_ERROR:				return "other error";
	}

	return "undefined FTD2xx error";
}

#endif
#endif /* _FTD2XX_COMMON_H */
