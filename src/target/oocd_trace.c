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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#define _GNU_SOURCE
#include <string.h>
#include <errno.h>

#include "oocd_trace.h"
#include "etm.h"

#include "log.h"
#include "types.h"
#include "binarybuffer.h"
#include "target.h"
#include "register.h"
#include "jtag.h"
#include "arm7_9_common.h"
#include "replacements.h"

#include <stdlib.h>

int oocd_trace_read_reg(oocd_trace_t *oocd_trace, int reg, u32 *value)
{
	size_t bytes_written, bytes_read, bytes_to_read;
	u8 cmd;

	cmd = 0x10 | (reg & 0x7);
	bytes_written = write(oocd_trace->tty_fd, &cmd, 1);

	bytes_to_read = 4;
	while (bytes_to_read > 0)
	{
		bytes_read = read(oocd_trace->tty_fd, ((u8*)value) + 4 - bytes_to_read, bytes_to_read);
		bytes_to_read -= bytes_read;
	}

	LOG_DEBUG("reg #%i: 0x%8.8x\n", reg, *value);
	
	return ERROR_OK;
}

int oocd_trace_write_reg(oocd_trace_t *oocd_trace, int reg, u32 value)
{
	size_t bytes_written;
	u8 data[5];

	data[0] = 0x18 | (reg & 0x7);
	data[1] = value & 0xff;
	data[2] = (value & 0xff00) >> 8;
	data[3] = (value & 0xff0000) >> 16;
	data[4] = (value & 0xff000000) >> 24;

	bytes_written = write(oocd_trace->tty_fd, data, 5);
	LOG_DEBUG("reg #%i: 0x%8.8x\n", reg, value);

	return ERROR_OK;
}

int oocd_trace_read_memory(oocd_trace_t *oocd_trace, u8 *data, u32 address, u32 size)
{
	size_t bytes_written, bytes_read, bytes_to_read;
	u8 cmd;

	oocd_trace_write_reg(oocd_trace, OOCD_TRACE_ADDRESS, address);
	oocd_trace_write_reg(oocd_trace, OOCD_TRACE_SDRAM_COUNTER, size);

	cmd = 0x20;
	bytes_written = write(oocd_trace->tty_fd, &cmd, 1);

	bytes_to_read = size * 16;
	while (bytes_to_read > 0)
	{
		if ((bytes_read = read(oocd_trace->tty_fd,
				((u8*)data) + (size * 16) - bytes_to_read, bytes_to_read)) < 0)
		{
			LOG_DEBUG("read() returned %i (%s)", bytes_read, strerror(errno));
		}
		else
			bytes_to_read -= bytes_read;
	}
	
	return ERROR_OK;
}

int oocd_trace_init(etm_context_t *etm_ctx)
{
	u8 trash[256];
	oocd_trace_t *oocd_trace = etm_ctx->capture_driver_priv;
	size_t bytes_read;
	
	oocd_trace->tty_fd = open(oocd_trace->tty, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if(oocd_trace->tty_fd < 0)
	{
		LOG_ERROR("can't open tty");
		return ERROR_ETM_CAPTURE_INIT_FAILED;
	}

	/* clear input & output buffers, then switch to "blocking mode" */
	tcflush(oocd_trace->tty_fd, TCOFLUSH);
	tcflush(oocd_trace->tty_fd, TCIFLUSH);
	fcntl(oocd_trace->tty_fd, F_SETFL, fcntl(oocd_trace->tty_fd, F_GETFL) & ~O_NONBLOCK);
	
	tcgetattr(oocd_trace->tty_fd, &oocd_trace->oldtio); /* save current port settings */
	
	bzero(&oocd_trace->newtio, sizeof(oocd_trace->newtio));
	oocd_trace->newtio.c_cflag = CS8 | CLOCAL | CREAD | B2500000;
	
	oocd_trace->newtio.c_iflag = IGNPAR | IGNBRK | IXON | IXOFF;
	oocd_trace->newtio.c_oflag = 0;
	
	/* set input mode (non-canonical, no echo,...) */
	oocd_trace->newtio.c_lflag = 0;
	
	cfmakeraw(&oocd_trace->newtio);
	oocd_trace->newtio.c_cc[VTIME] = 1;   /* inter-character timer used */
	oocd_trace->newtio.c_cc[VMIN] = 0;   /* blocking read until 0 chars received */
	
	tcflush(oocd_trace->tty_fd, TCIFLUSH);
	tcsetattr(oocd_trace->tty_fd, TCSANOW, &oocd_trace->newtio);
	
	/* occasionally one bogus character is left in the input buffer
	 * read up any leftover characters to ensure communication is in sync */
	while ((bytes_read = read(oocd_trace->tty_fd, trash, sizeof(trash))) > 0)
	{
		LOG_DEBUG("%i bytes read\n", bytes_read);
	};
	
	return ERROR_OK;
}

trace_status_t oocd_trace_status(etm_context_t *etm_ctx)
{
	oocd_trace_t *oocd_trace = etm_ctx->capture_driver_priv;
	u32 status;
	
	oocd_trace_read_reg(oocd_trace, OOCD_TRACE_STATUS, &status);
	
	/* if tracing is currently idle, return this information */
	if (etm_ctx->capture_status == TRACE_IDLE)
	{
		return etm_ctx->capture_status;
	}
	else if (etm_ctx->capture_status & TRACE_RUNNING)
	{
		/* check Full bit to identify an overflow */
		if (status & 0x4)
			etm_ctx->capture_status |= TRACE_OVERFLOWED;
		
		/* check Triggered bit to identify trigger condition */
		if (status & 0x2)
			etm_ctx->capture_status |= TRACE_TRIGGERED;
		
		if (status & 0x1)
		{
			etm_ctx->capture_status &= ~TRACE_RUNNING;
			etm_ctx->capture_status |= TRACE_COMPLETED;
		}
	}
	
	return etm_ctx->capture_status;
}

int oocd_trace_read_trace(etm_context_t *etm_ctx)
{
	oocd_trace_t *oocd_trace = etm_ctx->capture_driver_priv;
	u32 status, address;
	u32 first_frame = 0x0;
	u32 num_frames = 1048576;
	u8 *trace_data;
	int i;

	oocd_trace_read_reg(oocd_trace, OOCD_TRACE_STATUS, &status);
	oocd_trace_read_reg(oocd_trace, OOCD_TRACE_ADDRESS, &address);

	/* check if we overflowed, and adjust first frame of the trace accordingly
	 * if we didn't overflow, read only up to the frame that would be written next,
	 * i.e. don't read invalid entries
	 */
	if (status & 0x4)
		first_frame = address;
	else
		num_frames = address;

	/* read data into temporary array for unpacking 
	 * one frame from OpenOCD+trace corresponds to 16 trace cycles
	 */
	trace_data = malloc(sizeof(u8) * num_frames * 16);
	oocd_trace_read_memory(oocd_trace, trace_data, first_frame, num_frames);

	if (etm_ctx->trace_depth > 0)
	{
		free(etm_ctx->trace_data);
	}

	etm_ctx->trace_depth = num_frames * 16;
	etm_ctx->trace_data = malloc(sizeof(etmv1_trace_data_t) * etm_ctx->trace_depth);
	
	for (i = 0; i < num_frames * 16; i++)
	{
		etm_ctx->trace_data[i].pipestat = (trace_data[i] & 0x7);
		etm_ctx->trace_data[i].packet = (trace_data[i] & 0x78) >> 3;
		etm_ctx->trace_data[i].flags = 0;
		
		if ((trace_data[i] & 0x80) >> 7)
		{
			etm_ctx->trace_data[i].flags |= ETMV1_TRACESYNC_CYCLE;
		}

		if (etm_ctx->trace_data[i].pipestat == STAT_TR)
		{
			etm_ctx->trace_data[i].pipestat = etm_ctx->trace_data[i].packet & 0x7;
			etm_ctx->trace_data[i].flags |= ETMV1_TRIGGER_CYCLE;
		}
	}
	
	free(trace_data);

	return ERROR_OK;
}

int oocd_trace_start_capture(etm_context_t *etm_ctx)
{
	oocd_trace_t *oocd_trace = etm_ctx->capture_driver_priv;
	u32 control = 0x1;	/* 0x1: enabled */
	u32 trigger_count;
	
	if (((etm_ctx->portmode & ETM_PORT_MODE_MASK) != ETM_PORT_NORMAL)
		|| ((etm_ctx->portmode & ETM_PORT_WIDTH_MASK) != ETM_PORT_4BIT))
	{
		LOG_DEBUG("OpenOCD+trace only supports normal 4-bit ETM mode");
		return ERROR_ETM_PORTMODE_NOT_SUPPORTED;
	}
	
	if ((etm_ctx->portmode & ETM_PORT_CLOCK_MASK) == ETM_PORT_HALF_CLOCK)
	{
		control |= 0x2;	/* half rate clock, capture at twice the clock rate */
	}
	
	/* OpenOCD+trace holds up to 16 million samples,
	 * but trigger counts is set in multiples of 16 */
	trigger_count = (1048576 * etm_ctx->trigger_percent) / 100;

	/* capturing always starts at address zero */
	oocd_trace_write_reg(oocd_trace, OOCD_TRACE_ADDRESS, 0x0);
	oocd_trace_write_reg(oocd_trace, OOCD_TRACE_TRIGGER_COUNTER, trigger_count);
	oocd_trace_write_reg(oocd_trace, OOCD_TRACE_CONTROL, control);
	
	/* we're starting a new trace, initialize capture status */
	etm_ctx->capture_status = TRACE_RUNNING;
	
	return ERROR_OK; 
}

int oocd_trace_stop_capture(etm_context_t *etm_ctx)
{
	oocd_trace_t *oocd_trace = etm_ctx->capture_driver_priv;

	/* trace stopped, just clear running flag, but preserve others */ 
	etm_ctx->capture_status &= ~TRACE_RUNNING;
	
	oocd_trace_write_reg(oocd_trace, OOCD_TRACE_CONTROL, 0x0);
	
	return ERROR_OK;
}

etm_capture_driver_t oocd_trace_capture_driver =
{
	.name = "oocd_trace",
	.register_commands = oocd_trace_register_commands,
	.init = oocd_trace_init,
	.status = oocd_trace_status,
	.start_capture = oocd_trace_start_capture,
	.stop_capture = oocd_trace_stop_capture,
	.read_trace = oocd_trace_read_trace,
};

int handle_oocd_trace_config_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	
	if (argc != 2)
	{
		LOG_ERROR("incomplete 'oocd_trace config <target> <tty>' command");
		exit(-1);
	}
	
	target = get_current_target(cmd_ctx);
	
	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}
	
	if (arm7_9->etm_ctx)
	{
		oocd_trace_t *oocd_trace = malloc(sizeof(oocd_trace_t));
		
		arm7_9->etm_ctx->capture_driver_priv = oocd_trace;
		oocd_trace->etm_ctx = arm7_9->etm_ctx;
		
		/* copy name of TTY device used to communicate with OpenOCD+trace */
		oocd_trace->tty = strndup(args[1], 256);
	}
	else
	{
		LOG_ERROR("target has no ETM defined, OpenOCD+trace left unconfigured");
	}

	return ERROR_OK;
}

int handle_oocd_trace_status_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	oocd_trace_t *oocd_trace;
	u32 status;
	
	target = get_current_target(cmd_ctx);
	
	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}
	
	if (!arm7_9->etm_ctx)
	{
		command_print(cmd_ctx, "current target doesn't have an ETM configured");
		return ERROR_OK;
	}
	
	if (strcmp(arm7_9->etm_ctx->capture_driver->name, "oocd_trace") != 0)
	{
		command_print(cmd_ctx, "current target's ETM capture driver isn't 'oocd_trace'");
		return ERROR_OK;
	}
	
	oocd_trace = (oocd_trace_t*)arm7_9->etm_ctx->capture_driver_priv;
	
	oocd_trace_read_reg(oocd_trace, OOCD_TRACE_STATUS, &status);
	
	if (status & 0x8)
		command_print(cmd_ctx, "trace clock locked");
	else
		command_print(cmd_ctx, "no trace clock");
		
	return ERROR_OK;
}

int handle_oocd_trace_resync_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	target_t *target;
	armv4_5_common_t *armv4_5;
	arm7_9_common_t *arm7_9;
	oocd_trace_t *oocd_trace;
	size_t bytes_written;
	u8 cmd_array[1];
	
	target = get_current_target(cmd_ctx);
	
	if (arm7_9_get_arch_pointers(target, &armv4_5, &arm7_9) != ERROR_OK)
	{
		command_print(cmd_ctx, "current target isn't an ARM7/ARM9 target");
		return ERROR_OK;
	}
	
	if (!arm7_9->etm_ctx)
	{
		command_print(cmd_ctx, "current target doesn't have an ETM configured");
		return ERROR_OK;
	}
	
	if (strcmp(arm7_9->etm_ctx->capture_driver->name, "oocd_trace") != 0)
	{
		command_print(cmd_ctx, "current target's ETM capture driver isn't 'oocd_trace'");
		return ERROR_OK;
	}
	
	oocd_trace = (oocd_trace_t*)arm7_9->etm_ctx->capture_driver_priv;
	
	cmd_array[0] = 0xf0;

	bytes_written = write(oocd_trace->tty_fd, cmd_array, 1);
	
	command_print(cmd_ctx, "requesting traceclock resync");
	LOG_DEBUG("resyncing traceclk pll");

	return ERROR_OK;
}

int oocd_trace_register_commands(struct command_context_s *cmd_ctx)
{
	command_t *oocd_trace_cmd;
	
	oocd_trace_cmd = register_command(cmd_ctx, NULL, "oocd_trace", NULL, COMMAND_ANY, "OpenOCD+trace");
	
	register_command(cmd_ctx, oocd_trace_cmd, "config", handle_oocd_trace_config_command, COMMAND_CONFIG, NULL);

	register_command(cmd_ctx, oocd_trace_cmd, "status", handle_oocd_trace_status_command, COMMAND_EXEC, "display OpenOCD+trace status");
	register_command(cmd_ctx, oocd_trace_cmd, "resync", handle_oocd_trace_resync_command, COMMAND_EXEC, "resync OpenOCD+trace capture clock");

	return ERROR_OK;
}
