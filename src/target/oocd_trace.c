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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arm.h"
#include "etm.h"
#include "oocd_trace.h"

/*
 * This is "proof of concept" code, for prototype hardware:
 * https://lists.berlios.de/pipermail/openocd-development/2007-September/000336.html
 */

static int oocd_trace_read_reg(struct oocd_trace *oocd_trace, int reg, uint32_t *value)
{
	size_t bytes_written, bytes_read, bytes_to_read;
	uint8_t cmd;

	cmd = 0x10 | (reg & 0x7);
	bytes_written = write(oocd_trace->tty_fd, &cmd, 1);
	if (bytes_written < 1)
		return ERROR_FAIL;

	bytes_to_read = 4;
	while (bytes_to_read > 0) {
		bytes_read = read(oocd_trace->tty_fd, ((uint8_t *)value) + 4 - bytes_to_read, bytes_to_read);
		bytes_to_read -= bytes_read;
	}

	LOG_DEBUG("reg #%i: 0x%8.8x", reg, *value);

	return ERROR_OK;
}

static int oocd_trace_write_reg(struct oocd_trace *oocd_trace, int reg, uint32_t value)
{
	size_t bytes_written;
	uint8_t data[5];

	data[0] = 0x18 | (reg & 0x7);
	data[1] = value & 0xff;
	data[2] = (value & 0xff00) >> 8;
	data[3] = (value & 0xff0000) >> 16;
	data[4] = (value & 0xff000000) >> 24;

	bytes_written = write(oocd_trace->tty_fd, data, 5);
	if (bytes_written < 5)
		return ERROR_FAIL;

	LOG_DEBUG("reg #%i: 0x%8.8x", reg, value);

	return ERROR_OK;
}

static int oocd_trace_read_memory(struct oocd_trace *oocd_trace, uint8_t *data, uint32_t address, uint32_t size)
{
	size_t bytes_written, bytes_to_read;
	ssize_t bytes_read;
	uint8_t cmd;

	oocd_trace_write_reg(oocd_trace, OOCD_TRACE_ADDRESS, address);
	oocd_trace_write_reg(oocd_trace, OOCD_TRACE_SDRAM_COUNTER, size);

	cmd = 0x20;
	bytes_written = write(oocd_trace->tty_fd, &cmd, 1);
	if (bytes_written < 1)
		return ERROR_FAIL;

	bytes_to_read = size * 16;
	while (bytes_to_read > 0) {
		bytes_read = read(oocd_trace->tty_fd,
				((uint8_t *)data) + (size * 16) - bytes_to_read, bytes_to_read);
		if (bytes_read < 0)
			LOG_DEBUG("read() returned %zi (%s)", bytes_read, strerror(errno));
		else
			bytes_to_read -= bytes_read;
	}

	return ERROR_OK;
}

static int oocd_trace_init(struct etm_context *etm_ctx)
{
	uint8_t trash[256];
	struct oocd_trace *oocd_trace = etm_ctx->capture_driver_priv;
	size_t bytes_read;

	oocd_trace->tty_fd = open(oocd_trace->tty, O_RDWR | O_NOCTTY | O_NONBLOCK);

	if (oocd_trace->tty_fd < 0) {
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
	do {
		bytes_read = read(oocd_trace->tty_fd, trash, sizeof(trash));
		if (bytes_read)
			LOG_DEBUG("%zi bytes read", bytes_read);
	} while (bytes_read > 0);

	return ERROR_OK;
}

static trace_status_t oocd_trace_status(struct etm_context *etm_ctx)
{
	struct oocd_trace *oocd_trace = etm_ctx->capture_driver_priv;
	uint32_t status;

	oocd_trace_read_reg(oocd_trace, OOCD_TRACE_STATUS, &status);

	/* if tracing is currently idle, return this information */
	if (etm_ctx->capture_status == TRACE_IDLE)
		return etm_ctx->capture_status;
	else if (etm_ctx->capture_status & TRACE_RUNNING) {
		/* check Full bit to identify an overflow */
		if (status & 0x4)
			etm_ctx->capture_status |= TRACE_OVERFLOWED;

		/* check Triggered bit to identify trigger condition */
		if (status & 0x2)
			etm_ctx->capture_status |= TRACE_TRIGGERED;

		if (status & 0x1) {
			etm_ctx->capture_status &= ~TRACE_RUNNING;
			etm_ctx->capture_status |= TRACE_COMPLETED;
		}
	}

	return etm_ctx->capture_status;
}

static int oocd_trace_read_trace(struct etm_context *etm_ctx)
{
	struct oocd_trace *oocd_trace = etm_ctx->capture_driver_priv;
	uint32_t status, address;
	uint32_t first_frame = 0x0;
	uint32_t num_frames = 1048576;
	uint8_t *trace_data;
	uint32_t i;

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
	 * one frame from OpenOCD + trace corresponds to 16 trace cycles
	 */
	trace_data = malloc(sizeof(uint8_t) * num_frames * 16);
	oocd_trace_read_memory(oocd_trace, trace_data, first_frame, num_frames);

	if (etm_ctx->trace_depth > 0)
		free(etm_ctx->trace_data);

	etm_ctx->trace_depth = num_frames * 16;
	etm_ctx->trace_data = malloc(sizeof(struct etmv1_trace_data) * etm_ctx->trace_depth);

	for (i = 0; i < num_frames * 16; i++) {
		etm_ctx->trace_data[i].pipestat = (trace_data[i] & 0x7);
		etm_ctx->trace_data[i].packet = (trace_data[i] & 0x78) >> 3;
		etm_ctx->trace_data[i].flags = 0;

		if ((trace_data[i] & 0x80) >> 7)
			etm_ctx->trace_data[i].flags |= ETMV1_TRACESYNC_CYCLE;

		if (etm_ctx->trace_data[i].pipestat == STAT_TR) {
			etm_ctx->trace_data[i].pipestat = etm_ctx->trace_data[i].packet & 0x7;
			etm_ctx->trace_data[i].flags |= ETMV1_TRIGGER_CYCLE;
		}
	}

	free(trace_data);

	return ERROR_OK;
}

static int oocd_trace_start_capture(struct etm_context *etm_ctx)
{
	struct oocd_trace *oocd_trace = etm_ctx->capture_driver_priv;
	uint32_t control = 0x1;	/* 0x1: enabled */
	uint32_t trigger_count;

	if (((etm_ctx->control & ETM_PORT_MODE_MASK) != ETM_PORT_NORMAL)
		|| ((etm_ctx->control & ETM_PORT_WIDTH_MASK) != ETM_PORT_4BIT)) {
		LOG_DEBUG("OpenOCD + trace only supports normal 4-bit ETM mode");
		return ERROR_ETM_PORTMODE_NOT_SUPPORTED;
	}

	if ((etm_ctx->control & ETM_PORT_CLOCK_MASK) == ETM_PORT_HALF_CLOCK)
		control |= 0x2;	/* half rate clock, capture at twice the clock rate */

	/* OpenOCD + trace holds up to 16 million samples,
	 * but trigger counts is set in multiples of 16 */
	trigger_count = (1048576 * /* trigger_percent */ 50) / 100;

	/* capturing always starts at address zero */
	oocd_trace_write_reg(oocd_trace, OOCD_TRACE_ADDRESS, 0x0);
	oocd_trace_write_reg(oocd_trace, OOCD_TRACE_TRIGGER_COUNTER, trigger_count);
	oocd_trace_write_reg(oocd_trace, OOCD_TRACE_CONTROL, control);

	/* we're starting a new trace, initialize capture status */
	etm_ctx->capture_status = TRACE_RUNNING;

	return ERROR_OK;
}

static int oocd_trace_stop_capture(struct etm_context *etm_ctx)
{
	struct oocd_trace *oocd_trace = etm_ctx->capture_driver_priv;

	/* trace stopped, just clear running flag, but preserve others */
	etm_ctx->capture_status &= ~TRACE_RUNNING;

	oocd_trace_write_reg(oocd_trace, OOCD_TRACE_CONTROL, 0x0);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_oocd_trace_config_command)
{
	struct target *target;
	struct arm *arm;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	target = get_current_target(CMD_CTX);
	arm = target_to_arm(target);
	if (!is_arm(arm)) {
		command_print(CMD, "current target isn't an ARM");
		return ERROR_FAIL;
	}

	if (arm->etm) {
		struct oocd_trace *oocd_trace = malloc(sizeof(struct oocd_trace));

		arm->etm->capture_driver_priv = oocd_trace;
		oocd_trace->etm_ctx = arm->etm;

		/* copy name of TTY device used to communicate with OpenOCD + trace */
		oocd_trace->tty = strndup(CMD_ARGV[1], 256);
	} else
		LOG_ERROR("target has no ETM defined, OpenOCD + trace left unconfigured");

	return ERROR_OK;
}

COMMAND_HANDLER(handle_oocd_trace_status_command)
{
	struct target *target;
	struct arm *arm;
	struct oocd_trace *oocd_trace;
	uint32_t status;

	target = get_current_target(CMD_CTX);

	arm = target_to_arm(target);
	if (!is_arm(arm)) {
		command_print(CMD, "current target isn't an ARM");
		return ERROR_FAIL;
	}

	if (!arm->etm) {
		command_print(CMD, "current target doesn't have an ETM configured");
		return ERROR_FAIL;
	}

	if (strcmp(arm->etm->capture_driver->name, "oocd_trace") != 0) {
		command_print(CMD, "current target's ETM capture driver isn't 'oocd_trace'");
		return ERROR_FAIL;
	}

	oocd_trace = (struct oocd_trace *)arm->etm->capture_driver_priv;

	oocd_trace_read_reg(oocd_trace, OOCD_TRACE_STATUS, &status);

	if (status & 0x8)
		command_print(CMD, "trace clock locked");
	else
		command_print(CMD, "no trace clock");

	return ERROR_OK;
}

COMMAND_HANDLER(handle_oocd_trace_resync_command)
{
	struct target *target;
	struct arm *arm;
	struct oocd_trace *oocd_trace;
	size_t bytes_written;
	uint8_t cmd_array[1];

	target = get_current_target(CMD_CTX);

	arm = target_to_arm(target);
	if (!is_arm(arm)) {
		command_print(CMD, "current target isn't an ARM");
		return ERROR_FAIL;
	}

	if (!arm->etm) {
		command_print(CMD, "current target doesn't have an ETM configured");
		return ERROR_FAIL;
	}

	if (strcmp(arm->etm->capture_driver->name, "oocd_trace") != 0) {
		command_print(CMD, "current target's ETM capture driver isn't 'oocd_trace'");
		return ERROR_FAIL;
	}

	oocd_trace = (struct oocd_trace *)arm->etm->capture_driver_priv;

	cmd_array[0] = 0xf0;

	bytes_written = write(oocd_trace->tty_fd, cmd_array, 1);
	if (bytes_written < 1)
		return ERROR_FAIL;

	command_print(CMD, "requesting traceclock resync");
	LOG_DEBUG("resyncing traceclk pll");

	return ERROR_OK;
}

static const struct command_registration oocd_trace_all_command_handlers[] = {
	{
		.name = "config",
		.handler = handle_oocd_trace_config_command,
		.mode = COMMAND_CONFIG,
		.usage = "<target> <tty>",
	},
	{
		.name = "status",
		.handler = handle_oocd_trace_status_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "display OpenOCD + trace status",
	},
	{
		.name = "resync",
		.handler = handle_oocd_trace_resync_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "resync OpenOCD + trace capture clock",
	},
	COMMAND_REGISTRATION_DONE
};
static const struct command_registration oocd_trace_command_handlers[] = {
	{
		.name = "oocd_trace",
		.mode = COMMAND_ANY,
		.help = "OpenOCD trace capture driver command group",
		.usage = "",
		.chain = oocd_trace_all_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct etm_capture_driver oocd_trace_capture_driver = {
	.name = "oocd_trace",
	.commands = oocd_trace_command_handlers,
	.init = oocd_trace_init,
	.status = oocd_trace_status,
	.start_capture = oocd_trace_start_capture,
	.stop_capture = oocd_trace_stop_capture,
	.read_trace = oocd_trace_read_trace,
};
