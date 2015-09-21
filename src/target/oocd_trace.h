/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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

#ifndef OPENOCD_TARGET_OOCD_TRACE_H
#define OPENOCD_TARGET_OOCD_TRACE_H

#include <termios.h>

/* registers */
enum {
	OOCD_TRACE_ID = 0x7,
	OOCD_TRACE_ADDRESS = 0x0,
	OOCD_TRACE_TRIGGER_COUNTER = 0x01,
	OOCD_TRACE_CONTROL = 0x2,
	OOCD_TRACE_STATUS = 0x3,
	OOCD_TRACE_SDRAM_COUNTER = 0x4,
};

/* commands */
enum {
	OOCD_TRACE_NOP = 0x0,
	OOCD_TRACE_READ_REG = 0x10,
	OOCD_TRACE_WRITE_REG = 0x18,
	OOCD_TRACE_READ_RAM = 0x20,
/*	OOCD_TRACE_WRITE_RAM = 0x28, */
	OOCD_TRACE_RESYNC = 0xf0,
};

struct oocd_trace {
	struct etm_context *etm_ctx;
	char *tty;
	int tty_fd;
	struct termios oldtio, newtio;
};

extern struct etm_capture_driver oocd_trace_capture_driver;

#endif /* OPENOCD_TARGET_OOCD_TRACE_H */
