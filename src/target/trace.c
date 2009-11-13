/***************************************************************************
 *   Copyright (C) 2005, 2007 by Dominic Rath                              *
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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "log.h"
#include "trace.h"
#include "target.h"

int trace_point(struct target *target, uint32_t number)
{
	struct trace *trace = target->trace_info;

	LOG_DEBUG("tracepoint: %i", (int)number);

	if (number < trace->num_trace_points)
		trace->trace_points[number].hit_counter++;

	if (trace->trace_history_size)
	{
		trace->trace_history[trace->trace_history_pos++] = number;
		if (trace->trace_history_pos == trace->trace_history_size)
		{
			trace->trace_history_pos = 0;
			trace->trace_history_overflowed = 1;
		}
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_trace_point_command)
{
	struct target *target = get_current_target(cmd_ctx);
	struct trace *trace = target->trace_info;

	if (argc == 0)
	{
		uint32_t i;

		for (i = 0; i < trace->num_trace_points; i++)
		{
			command_print(cmd_ctx, "trace point 0x%8.8" PRIx32 " (%lld times hit)",
					trace->trace_points[i].address,
					(long long)trace->trace_points[i].hit_counter);
		}

		return ERROR_OK;
	}

	if (!strcmp(args[0], "clear"))
	{
		if (trace->trace_points)
		{
			free(trace->trace_points);
			trace->trace_points = NULL;
		}
		trace->num_trace_points = 0;
		trace->trace_points_size = 0;

		return ERROR_OK;
	}

	/* resize array if necessary */
	if (!trace->trace_points || (trace->trace_points_size == trace->num_trace_points))
	{
		trace->trace_points = realloc(trace->trace_points, sizeof(struct trace_point) * (trace->trace_points_size + 32));
		trace->trace_points_size += 32;
	}

	uint32_t address;
	COMMAND_PARSE_NUMBER(u32, args[0], address);
	trace->trace_points[trace->num_trace_points].address = address;
	trace->trace_points[trace->num_trace_points].hit_counter = 0;
	trace->num_trace_points++;

	return ERROR_OK;
}

COMMAND_HANDLER(handle_trace_history_command)
{
	struct target *target = get_current_target(cmd_ctx);
	struct trace *trace = target->trace_info;

	if (argc > 0)
	{
		trace->trace_history_pos = 0;
		trace->trace_history_overflowed = 0;

		if (!strcmp(args[0], "clear"))
		{
			/* clearing is implicit, we've just reset position anyway */
			return ERROR_OK;
		}

		if (trace->trace_history)
			free(trace->trace_history);

		COMMAND_PARSE_NUMBER(u32, args[0], trace->trace_history_size);
		trace->trace_history = malloc(sizeof(uint32_t) * trace->trace_history_size);

		command_print(cmd_ctx, "new trace history size: %i", (int)(trace->trace_history_size));
	}
	else
	{
		uint32_t i;
		uint32_t first = 0;
		uint32_t last = trace->trace_history_pos;

		if (!trace->trace_history_size) {
			command_print(cmd_ctx, "trace history buffer is not allocated");
			return ERROR_OK;
		}
		if (trace->trace_history_overflowed)
		{
			first = trace->trace_history_pos;
			last = trace->trace_history_pos - 1;
		}

		for (i = first; (i % trace->trace_history_size) != last; i++)
		{
			if (trace->trace_history[i % trace->trace_history_size] < trace->num_trace_points)
			{
				uint32_t address;
				address = trace->trace_points[trace->trace_history[i % trace->trace_history_size]].address;
				command_print(cmd_ctx, "trace point %i: 0x%8.8" PRIx32 "",
					      (int)(trace->trace_history[i % trace->trace_history_size]),
					      address);
			}

			else
			{
				command_print(cmd_ctx, "trace point %i: -not defined-", (int)(trace->trace_history[i % trace->trace_history_size]));
			}
		}
	}

	return ERROR_OK;
}

int trace_register_commands(struct command_context_s *cmd_ctx)
{
	command_t *trace_cmd =
		register_command(cmd_ctx, NULL, "trace", NULL, COMMAND_ANY, "trace commands");

	register_command(cmd_ctx, trace_cmd, "history", handle_trace_history_command,
		COMMAND_EXEC, "display trace history, ['clear'] history or set [size]");

	register_command(cmd_ctx, trace_cmd, "point", handle_trace_point_command,
		COMMAND_EXEC, "display trace points, ['clear'] list of trace points, or add new tracepoint at [address]");

	return ERROR_OK;
}
