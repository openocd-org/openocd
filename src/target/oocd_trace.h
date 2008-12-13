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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef OOCD_TRACE_H
#define OOCD_TRACE_H

#include "command.h"

#include "etm.h"

#include <termios.h>
#include <unistd.h>

/* registers */
enum
{
	OOCD_TRACE_ID = 0x7,
	OOCD_TRACE_ADDRESS = 0x0,
	OOCD_TRACE_TRIGGER_COUNTER = 0x01,
	OOCD_TRACE_CONTROL = 0x2,
	OOCD_TRACE_STATUS = 0x3,
	OOCD_TRACE_SDRAM_COUNTER = 0x4,
};

/* commands */
enum
{
	OOCD_TRACE_NOP = 0x0,
	OOCD_TRACE_READ_REG = 0x10,
	OOCD_TRACE_WRITE_REG = 0x18,
	OOCD_TRACE_READ_RAM = 0x20,
/*	OOCD_TRACE_WRITE_RAM = 0x28, */
	OOCD_TRACE_RESYNC = 0xf0,
};

typedef struct oocd_trace_s
{
	etm_context_t *etm_ctx;
	char *tty;
	int tty_fd;
	struct termios oldtio, newtio;
} oocd_trace_t;

extern etm_capture_driver_t oocd_trace_capture_driver;

extern int oocd_trace_register_commands(struct command_context_s *cmd_ctx);

#endif /* OOCD_TRACE_TRACE_H */
