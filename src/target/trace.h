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
#ifndef TRACE_H
#define TRACE_H

#include "target.h"
#include "command.h"
#include "types.h"

typedef struct trace_point_s
{
	u32 address;
	u64 hit_counter;
} trace_point_t;

typedef struct trace_s
{
	int num_trace_points;
	int trace_points_size;
	trace_point_t *trace_points;
	int trace_history_size;
	u32 *trace_history;
	int trace_history_pos;
	int trace_history_overflowed;
} trace_t;

typedef enum trace_status
{
	TRACE_IDLE = 0x0,
	TRACE_RUNNING = 0x1,
	TRACE_TRIGGERED = 0x2,
	TRACE_COMPLETED = 0x4,
	TRACE_OVERFLOWED = 0x8,
} trace_status_t;

extern int trace_point(struct target_s *target, int number);
extern int trace_register_commands(struct command_context_s *cmd_ctx);

#define ERROR_TRACE_IMAGE_UNAVAILABLE		-(1500)
#define ERROR_TRACE_INSTRUCTION_UNAVAILABLE	-(1501)

#endif /* TRACE_H */
