/***************************************************************************
 *   Copyright (C) 2018 by Square, Inc.                                    *
 *   Steven Stallion <stallion@squareup.com>                               *
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

#ifndef OPENOCD_TARGET_ESIRISC_TRACE_H
#define OPENOCD_TARGET_ESIRISC_TRACE_H

#include <helper/command.h>
#include <helper/types.h>
#include <target/target.h>

enum esirisc_trace_delay {
	ESIRISC_TRACE_DELAY_NONE,
	ESIRISC_TRACE_DELAY_START,
	ESIRISC_TRACE_DELAY_STOP,
	ESIRISC_TRACE_DELAY_BOTH,
};

enum esirisc_trace_format {
	ESIRISC_TRACE_FORMAT_FULL,
	ESIRISC_TRACE_FORMAT_BRANCH,
	ESIRISC_TRACE_FORMAT_ICACHE,
};

enum esirisc_trace_id {
	ESIRISC_TRACE_ID_EXECUTE,
	ESIRISC_TRACE_ID_STALL,
	ESIRISC_TRACE_ID_BRANCH,
	ESIRISC_TRACE_ID_EXTENDED,
};

enum esirisc_trace_ext_id {
	ESIRISC_TRACE_EXT_ID_EXCEPTION = 1,
	ESIRISC_TRACE_EXT_ID_ERET,
	ESIRISC_TRACE_EXT_ID_STOP,
	ESIRISC_TRACE_EXT_ID_WAIT,
	ESIRISC_TRACE_EXT_ID_MULTICYCLE,
	ESIRISC_TRACE_EXT_ID_COUNT,
	ESIRISC_TRACE_EXT_ID_PC,
	ESIRISC_TRACE_EXT_ID_INDIRECT,
	ESIRISC_TRACE_EXT_ID_END,
	ESIRISC_TRACE_EXT_ID_END_PC,
};

enum esirisc_trace_trigger {
	ESIRISC_TRACE_TRIGGER_NONE,
	ESIRISC_TRACE_TRIGGER_PC,
	ESIRISC_TRACE_TRIGGER_LOAD,
	ESIRISC_TRACE_TRIGGER_STORE,
	ESIRISC_TRACE_TRIGGER_EXCEPTION,
	ESIRISC_TRACE_TRIGGER_ERET,
	ESIRISC_TRACE_TRIGGER_WAIT,
	ESIRISC_TRACE_TRIGGER_STOP,
	ESIRISC_TRACE_TRIGGER_HIGH,
	ESIRISC_TRACE_TRIGGER_LOW,
};

struct esirisc_trace {
	target_addr_t buffer_start;
	target_addr_t buffer_end;
	bool buffer_wrap;
	bool flow_control;

	enum esirisc_trace_format format;
	int pc_bits;

	enum esirisc_trace_trigger start_trigger;
	uint32_t start_data;
	uint32_t start_mask;

	enum esirisc_trace_trigger stop_trigger;
	uint32_t stop_data;
	uint32_t stop_mask;

	enum esirisc_trace_delay delay;
	uint32_t delay_cycles;
};

extern const struct command_registration esirisc_trace_command_handlers[];

static inline uint32_t esirisc_trace_buffer_size(struct esirisc_trace *trace_info)
{
	return trace_info->buffer_end - trace_info->buffer_start;
}

static inline bool esirisc_trace_is_fifo(struct esirisc_trace *trace_info)
{
	return trace_info->buffer_start == trace_info->buffer_end;
}

#endif /* OPENOCD_TARGET_ESIRISC_TRACE_H */
