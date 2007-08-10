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

#include "replacements.h"
#include "log.h"
#include "trace.h"
#include "target.h"

#include <stdlib.h>

int handle_trace_history_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target = get_current_target(cmd_ctx);
	trace_t *trace = target->trace_info;
	
	if (argc > 0)
	{
		if (trace->trace_history)
			free(trace->trace_history);
		
		trace->trace_history_size = strtoul(args[0], NULL, 0);
		trace->trace_history = malloc(sizeof(u32) * trace->trace_history_size);
		
		command_print(cmd_ctx, "new trace history size: %i", trace->trace_history_size);
	}
	else
	{
		int i;
		
		for (i = 0; i < trace->trace_history_size; i++)
		{
			if (trace->trace_history[i] < trace->num_trace_points)
			{
				u32 address;
				address = trace->trace_points[trace->trace_history[i]].address;
				command_print(cmd_ctx, "trace point %i: 0x%8.8x",
					trace->trace_history[i],
					address);
			}

			else
			{
				command_print(cmd_ctx, "trace point %i: -not defined-", trace->trace_history[i]);
			}
		}
	}

	return ERROR_OK;
}
