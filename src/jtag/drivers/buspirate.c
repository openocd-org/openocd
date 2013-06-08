/***************************************************************************
 *   Copyright (C) 2010 by Michal Demin                                    *
 *   based on usbprog.c and arm-jtag-ew.c                                  *
 *   Several fixes by R. Diez in 2013.                                     *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include <jtag/commands.h>

#include <termios.h>
#include <fcntl.h>
#include <sys/ioctl.h>

#undef DEBUG_SERIAL
/*#define DEBUG_SERIAL */
static int buspirate_execute_queue(void);
static int buspirate_init(void);
static int buspirate_quit(void);

static void buspirate_end_state(tap_state_t state);
static void buspirate_state_move(void);
static void buspirate_path_move(int num_states, tap_state_t *path);
static void buspirate_runtest(int num_cycles);
static void buspirate_scan(bool ir_scan, enum scan_type type,
	uint8_t *buffer, int scan_size, struct scan_command *command);

#define CMD_UNKNOWN       0x00
#define CMD_PORT_MODE     0x01
#define CMD_FEATURE       0x02
#define CMD_READ_ADCS     0x03
/*#define CMD_TAP_SHIFT     0x04 // old protocol */
#define CMD_TAP_SHIFT     0x05
#define CMD_ENTER_OOCD    0x06
#define CMD_UART_SPEED    0x07
#define CMD_JTAG_SPEED    0x08

/* Not all OSes have this speed defined */
#if !defined(B1000000)
#define  B1000000 0010010
#endif

enum {
	MODE_HIZ = 0,
	MODE_JTAG = 1,		/* push-pull outputs */
	MODE_JTAG_OD = 2,	/* open-drain outputs */
};

enum {
	FEATURE_LED = 0x01,
	FEATURE_VREG = 0x02,
	FEATURE_TRST = 0x04,
	FEATURE_SRST = 0x08,
	FEATURE_PULLUP = 0x10
};

enum {
	ACTION_DISABLE = 0,
	ACTION_ENABLE = 1
};

enum {
	SERIAL_NORMAL = 0,
	SERIAL_FAST = 1
};

static const cc_t SHORT_TIMEOUT  = 1; /* Must be at least 1. */
static const cc_t NORMAL_TIMEOUT = 10;

static int buspirate_fd = -1;
static int buspirate_pinmode = MODE_JTAG_OD;
static int buspirate_baudrate = SERIAL_NORMAL;
static int buspirate_vreg;
static int buspirate_pullup;
static char *buspirate_port;

static enum tap_state last_tap_state = TAP_RESET;


/* TAP interface */
static void buspirate_tap_init(void);
static int buspirate_tap_execute(void);
static void buspirate_tap_append(int tms, int tdi);
static void buspirate_tap_append_scan(int length, uint8_t *buffer,
		struct scan_command *command);
static void buspirate_tap_make_space(int scan, int bits);

static void buspirate_reset(int trst, int srst);

/* low level interface */
static void buspirate_jtag_reset(int);
static void buspirate_jtag_enable(int);
static unsigned char buspirate_jtag_command(int, char *, int);
static void buspirate_jtag_set_speed(int, char);
static void buspirate_jtag_set_mode(int, char);
static void buspirate_jtag_set_feature(int, char, char);
static void buspirate_jtag_get_adcs(int);

/* low level HW communication interface */
static int buspirate_serial_open(char *port);
static int buspirate_serial_setspeed(int fd, char speed, cc_t timeout);
static int buspirate_serial_write(int fd, char *buf, int size);
static int buspirate_serial_read(int fd, char *buf, int size);
static void buspirate_serial_close(int fd);
static void buspirate_print_buffer(char *buf, int size);

static int buspirate_execute_queue(void)
{
	/* currently processed command */
	struct jtag_command *cmd = jtag_command_queue;
	int scan_size;
	enum scan_type type;
	uint8_t *buffer;

	while (cmd) {
		switch (cmd->type) {
		case JTAG_RUNTEST:
			DEBUG_JTAG_IO("runtest %i cycles, end in %s",
				cmd->cmd.runtest->num_cycles,
				tap_state_name(cmd->cmd.runtest
					->end_state));
			buspirate_end_state(cmd->cmd.runtest
					->end_state);
			buspirate_runtest(cmd->cmd.runtest
					->num_cycles);
			break;
		case JTAG_TLR_RESET:
			DEBUG_JTAG_IO("statemove end in %s",
				tap_state_name(cmd->cmd.statemove
						->end_state));
			buspirate_end_state(cmd->cmd.statemove
					->end_state);
			buspirate_state_move();
			break;
		case JTAG_PATHMOVE:
			DEBUG_JTAG_IO("pathmove: %i states, end in %s",
				cmd->cmd.pathmove->num_states,
				tap_state_name(cmd->cmd.pathmove
					->path[cmd->cmd.pathmove
						->num_states - 1]));
			buspirate_path_move(cmd->cmd.pathmove
					->num_states,
					cmd->cmd.pathmove->path);
			break;
		case JTAG_SCAN:
			DEBUG_JTAG_IO("scan end in %s",
				tap_state_name(cmd->cmd.scan
					->end_state));

			buspirate_end_state(cmd->cmd.scan
					->end_state);

			scan_size = jtag_build_buffer(cmd->cmd.scan,
					&buffer);
			type = jtag_scan_type(cmd->cmd.scan);
			buspirate_scan(cmd->cmd.scan->ir_scan, type,
				buffer, scan_size, cmd->cmd.scan);

			break;
		case JTAG_RESET:
			DEBUG_JTAG_IO("reset trst: %i srst %i",
				cmd->cmd.reset->trst, cmd->cmd.reset->srst);

			/* flush buffers, so we can reset */
			buspirate_tap_execute();

			if (cmd->cmd.reset->trst == 1)
				tap_set_state(TAP_RESET);
			buspirate_reset(cmd->cmd.reset->trst,
					cmd->cmd.reset->srst);
			break;
		case JTAG_SLEEP:
			DEBUG_JTAG_IO("sleep %i", cmd->cmd.sleep->us);
			buspirate_tap_execute();
			jtag_sleep(cmd->cmd.sleep->us);
				break;
		default:
			LOG_ERROR("BUG: unknown JTAG command type encountered");
			exit(-1);
		}

		cmd = cmd->next;
	}

	return buspirate_tap_execute();
}


/* Returns true if successful, false if error. */

static bool read_and_discard_all_data(const int fd)
{
	/* LOG_INFO("Discarding any stale data from a previous connection..."); */

	bool was_msg_already_printed = false;

	for ( ; ; ) {
		char buffer[1024];  /* Any size will do, it's a trade-off between stack size and performance. */

		const ssize_t read_count = read(fd, buffer, sizeof(buffer));

		if (read_count == 0) {
			/* This is the "end of file" or "connection closed at the other end" condition. */
			return true;
		}

		if (read_count > 0) {
			if (!was_msg_already_printed)	{
				LOG_INFO("Some stale data from a previous connection was discarded.");
				was_msg_already_printed = true;
			}

			continue;
		}

		assert(read_count == -1);  /* According to the specification. */

		const int errno_code = errno;

		if (errno_code == EINTR)
			continue;

		if (errno_code == EAGAIN ||
			errno_code == EWOULDBLOCK) {
			/* We know that the file descriptor has been opened with O_NONBLOCK or O_NDELAY,
			   and these codes mean that there is no data to read at present. */
			return true;
		}

		/* Some other error has occurred. */
		return false;
	}
}


static int buspirate_init(void)
{
	if (buspirate_port == NULL) {
		LOG_ERROR("You need to specify the serial port!");
		return ERROR_JTAG_INIT_FAILED;
	}

	buspirate_fd = buspirate_serial_open(buspirate_port);
	if (buspirate_fd == -1) {
		LOG_ERROR("Could not open serial port");
		return ERROR_JTAG_INIT_FAILED;
	}

	/* The Operating System or the device itself may deliver stale data from the last connection,
	   so discard all available bytes right after the new connection has been established.
	   After all, we are implementing here a master/slave protocol, so the slave should have nothing
	   to say until the master sends the first command.

	   In the past, there was a tcflush() call in buspirate_serial_setspeed(), but that
	   was not enough. I guess you must actively read from the serial port to trigger any
	   data collection from the device and/or lower USB layers. If you disable the serial port
	   read timeout (if you set SHORT_TIMEOUT to 0), then the discarding does not work any more.

	   Note that we are lowering the serial port timeout for this first read operation,
	   otherwise the normal initialisation would be delayed for too long. */

	if (-1 == buspirate_serial_setspeed(buspirate_fd, SERIAL_NORMAL, SHORT_TIMEOUT)) {
		LOG_ERROR("Error configuring the serial port.");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (!read_and_discard_all_data(buspirate_fd)) {
		LOG_ERROR("Error while attempting to discard any stale data right after establishing the connection.");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (-1 == buspirate_serial_setspeed(buspirate_fd, SERIAL_NORMAL, NORMAL_TIMEOUT)) {
		LOG_ERROR("Error configuring the serial port.");
		return ERROR_JTAG_INIT_FAILED;
	}

	buspirate_jtag_enable(buspirate_fd);

	if (buspirate_baudrate != SERIAL_NORMAL)
		buspirate_jtag_set_speed(buspirate_fd, SERIAL_FAST);

	LOG_INFO("Buspirate Interface ready!");

	buspirate_tap_init();
	buspirate_jtag_set_mode(buspirate_fd, buspirate_pinmode);
	buspirate_jtag_set_feature(buspirate_fd, FEATURE_VREG,
		(buspirate_vreg == 1) ? ACTION_ENABLE : ACTION_DISABLE);
	buspirate_jtag_set_feature(buspirate_fd, FEATURE_PULLUP,
		(buspirate_pullup == 1) ? ACTION_ENABLE : ACTION_DISABLE);
	buspirate_reset(0, 0);

	return ERROR_OK;
}

static int buspirate_quit(void)
{
	LOG_INFO("Shutting down buspirate.");
	buspirate_jtag_set_mode(buspirate_fd, MODE_HIZ);

	buspirate_jtag_set_speed(buspirate_fd, SERIAL_NORMAL);
	buspirate_jtag_reset(buspirate_fd);

	buspirate_serial_close(buspirate_fd);

	if (buspirate_port) {
		free(buspirate_port);
		buspirate_port = NULL;
	}
	return ERROR_OK;
}

/* openocd command interface */
COMMAND_HANDLER(buspirate_handle_adc_command)
{
	if (buspirate_fd == -1)
		return ERROR_OK;

	/* send the command */
	buspirate_jtag_get_adcs(buspirate_fd);

	return ERROR_OK;

}

COMMAND_HANDLER(buspirate_handle_vreg_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (atoi(CMD_ARGV[0]) == 1)
		buspirate_vreg = 1;
	else if (atoi(CMD_ARGV[0]) == 0)
		buspirate_vreg = 0;
	else
		LOG_ERROR("usage: buspirate_vreg <1|0>");

	return ERROR_OK;

}

COMMAND_HANDLER(buspirate_handle_pullup_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (atoi(CMD_ARGV[0]) == 1)
		buspirate_pullup = 1;
	else if (atoi(CMD_ARGV[0]) == 0)
		buspirate_pullup = 0;
	else
		LOG_ERROR("usage: buspirate_pullup <1|0>");

	return ERROR_OK;

}

COMMAND_HANDLER(buspirate_handle_led_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (atoi(CMD_ARGV[0]) == 1) {
		/* enable led */
		buspirate_jtag_set_feature(buspirate_fd, FEATURE_LED,
				ACTION_ENABLE);
	} else if (atoi(CMD_ARGV[0]) == 0) {
		/* disable led */
		buspirate_jtag_set_feature(buspirate_fd, FEATURE_LED,
				ACTION_DISABLE);
	} else {
		LOG_ERROR("usage: buspirate_led <1|0>");
	}

	return ERROR_OK;

}

COMMAND_HANDLER(buspirate_handle_mode_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGV[0][0] == 'n')
		buspirate_pinmode = MODE_JTAG;
	else if (CMD_ARGV[0][0] == 'o')
		buspirate_pinmode = MODE_JTAG_OD;
	else
		LOG_ERROR("usage: buspirate_mode <normal|open-drain>");

	return ERROR_OK;

}

COMMAND_HANDLER(buspirate_handle_speed_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGV[0][0] == 'n')
		buspirate_baudrate = SERIAL_NORMAL;
	else if (CMD_ARGV[0][0] == 'f')
		buspirate_baudrate = SERIAL_FAST;
	else
		LOG_ERROR("usage: buspirate_speed <normal|fast>");

	return ERROR_OK;

}

COMMAND_HANDLER(buspirate_handle_port_command)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (buspirate_port == NULL)
		buspirate_port = strdup(CMD_ARGV[0]);

	return ERROR_OK;

}

static const struct command_registration buspirate_command_handlers[] = {
	{
		.name = "buspirate_adc",
		.handler = &buspirate_handle_adc_command,
		.mode = COMMAND_EXEC,
		.help = "reads voltages on adc pins",
	},
	{
		.name = "buspirate_vreg",
		.usage = "<1|0>",
		.handler = &buspirate_handle_vreg_command,
		.mode = COMMAND_CONFIG,
		.help = "changes the state of voltage regulators",
	},
	{
		.name = "buspirate_pullup",
		.usage = "<1|0>",
		.handler = &buspirate_handle_pullup_command,
		.mode = COMMAND_CONFIG,
		.help = "changes the state of pullup",
	},
	{
		.name = "buspirate_led",
		.usage = "<1|0>",
		.handler = &buspirate_handle_led_command,
		.mode = COMMAND_EXEC,
		.help = "changes the state of led",
	},
	{
		.name = "buspirate_speed",
		.usage = "<normal|fast>",
		.handler = &buspirate_handle_speed_command,
		.mode = COMMAND_CONFIG,
		.help = "speed of the interface",
	},
	{
		.name = "buspirate_mode",
		.usage = "<normal|open-drain>",
		.handler = &buspirate_handle_mode_command,
		.mode = COMMAND_CONFIG,
		.help = "pin mode of the interface",
	},
	{
		.name = "buspirate_port",
		.usage = "/dev/ttyUSB0",
		.handler = &buspirate_handle_port_command,
		.mode =	COMMAND_CONFIG,
		.help = "name of the serial port to open",
	},
	COMMAND_REGISTRATION_DONE
};

struct jtag_interface buspirate_interface = {
	.name = "buspirate",
	.execute_queue = buspirate_execute_queue,
	.commands = buspirate_command_handlers,
	.init = buspirate_init,
	.quit = buspirate_quit
};

/*************** jtag execute commands **********************/
static void buspirate_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else {
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

static void buspirate_state_move(void)
{
	int i = 0, tms = 0;
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(),
			tap_get_end_state());
	int tms_count = tap_get_tms_path_len(tap_get_state(),
			tap_get_end_state());

	for (i = 0; i < tms_count; i++) {
		tms = (tms_scan >> i) & 1;
		buspirate_tap_append(tms, 0);
	}

	tap_set_state(tap_get_end_state());
}

static void buspirate_path_move(int num_states, tap_state_t *path)
{
	int i;

	for (i = 0; i < num_states; i++) {
		if (tap_state_transition(tap_get_state(), false) == path[i]) {
			buspirate_tap_append(0, 0);
		} else if (tap_state_transition(tap_get_state(), true)
				== path[i]) {
			buspirate_tap_append(1, 0);
		} else {
			LOG_ERROR("BUG: %s -> %s isn't a valid "
				"TAP transition",
				tap_state_name(tap_get_state()),
				tap_state_name(path[i]));
			exit(-1);
		}

		tap_set_state(path[i]);
	}

	tap_set_end_state(tap_get_state());
}

static void buspirate_runtest(int num_cycles)
{
	int i;

	tap_state_t saved_end_state = tap_get_end_state();

	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE) {
		buspirate_end_state(TAP_IDLE);
		buspirate_state_move();
	}

	for (i = 0; i < num_cycles; i++)
		buspirate_tap_append(0, 0);

	DEBUG_JTAG_IO("runtest: cur_state %s end_state %s",
			tap_state_name(tap_get_state()),
			tap_state_name(tap_get_end_state()));

	/* finish in end_state */
	buspirate_end_state(saved_end_state);
	if (tap_get_state() != tap_get_end_state())
		buspirate_state_move();
}

static void buspirate_scan(bool ir_scan, enum scan_type type,
	uint8_t *buffer, int scan_size, struct scan_command *command)
{
	tap_state_t saved_end_state;

	buspirate_tap_make_space(1, scan_size+8);
	/* is 8 correct ? (2 moves = 16) */

	saved_end_state = tap_get_end_state();

	buspirate_end_state(ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT);

	/* Only move if we're not already there */
	if (tap_get_state() != tap_get_end_state())
		buspirate_state_move();

	buspirate_tap_append_scan(scan_size, buffer, command);

	/* move to PAUSE */
	buspirate_tap_append(0, 0);

	/* restore the saved state */
	buspirate_end_state(saved_end_state);
	tap_set_state(ir_scan ? TAP_IRPAUSE : TAP_DRPAUSE);

	if (tap_get_state() != tap_get_end_state())
		buspirate_state_move();
}


/************************* TAP related stuff **********/

/* This buffer size matches the maximum CMD_TAP_SHIFT bit length in the Bus Pirate firmware,
   look for constant 0x2000 in OpenOCD.c . */
#define BUSPIRATE_BUFFER_SIZE 1024

/* The old value of 32 scans was not enough to achieve near 100% utilisation ratio
   for the current BUSPIRATE_BUFFER_SIZE value of 1024.
   With 128 scans I am getting full USB 2.0 high speed packets (512 bytes long) when
   using the JtagDue firmware on the Arduino Due instead of the Bus Pirate, which
   amounts approximately to a 10% overall speed gain. Bigger packets should also
   benefit the Bus Pirate, but the speed difference is much smaller.
   Unfortunately, each 512-byte packet is followed by a 329-byte one, which is not ideal.
   However, increasing BUSPIRATE_BUFFER_SIZE for the benefit of the JtagDue would
   make it incompatible with the Bus Pirate firmware. */
#define BUSPIRATE_MAX_PENDING_SCANS 128

static char tms_chain[BUSPIRATE_BUFFER_SIZE]; /* send */
static char tdi_chain[BUSPIRATE_BUFFER_SIZE]; /* send */
static int tap_chain_index;

struct pending_scan_result /* this was stolen from arm-jtag-ew */
{
	int first; /* First bit position in tdo_buffer to read */
	int length; /* Number of bits to read */
	struct scan_command *command; /* Corresponding scan command */
	uint8_t *buffer;
};

static struct pending_scan_result
tap_pending_scans[BUSPIRATE_MAX_PENDING_SCANS];
static int tap_pending_scans_num;

static void buspirate_tap_init(void)
{
	tap_chain_index = 0;
	tap_pending_scans_num = 0;
}

static int buspirate_tap_execute(void)
{
	static const int CMD_TAP_SHIFT_HEADER_LEN = 3;

	char tmp[4096];
	uint8_t *in_buf;
	int i;
	int fill_index = 0;
	int ret;
	int bytes_to_send;

	if (tap_chain_index <= 0)
		return ERROR_OK;

	LOG_DEBUG("executing tap num bits = %i scans = %i",
			tap_chain_index, tap_pending_scans_num);

	bytes_to_send = DIV_ROUND_UP(tap_chain_index, 8);

	tmp[0] = CMD_TAP_SHIFT; /* this command expects number of bits */
	tmp[1] = (char)(tap_chain_index >> 8);  /* high */
	tmp[2] = (char)(tap_chain_index);  /* low */

	fill_index = CMD_TAP_SHIFT_HEADER_LEN;
	for (i = 0; i < bytes_to_send; i++) {
		tmp[fill_index] = tdi_chain[i];
		fill_index++;
		tmp[fill_index] = tms_chain[i];
		fill_index++;
	}

	/* jlink.c calls the routine below, which may be useful for debugging purposes.
	   For example, enabling this allows you to compare the log outputs from jlink.c
	   and from this module for JTAG development or troubleshooting purposes. */
	if (false) {
		last_tap_state = jtag_debug_state_machine(tms_chain, tdi_chain,
												  tap_chain_index, last_tap_state);
	}

	ret = buspirate_serial_write(buspirate_fd, tmp, CMD_TAP_SHIFT_HEADER_LEN + bytes_to_send*2);
	if (ret != bytes_to_send*2+CMD_TAP_SHIFT_HEADER_LEN) {
		LOG_ERROR("error writing :(");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	ret = buspirate_serial_read(buspirate_fd, tmp, bytes_to_send + CMD_TAP_SHIFT_HEADER_LEN);
	if (ret != bytes_to_send + CMD_TAP_SHIFT_HEADER_LEN) {
		LOG_ERROR("error reading");
		return ERROR_FAIL;
	}
	in_buf = (uint8_t *)(&tmp[CMD_TAP_SHIFT_HEADER_LEN]);

	/* parse the scans */
	for (i = 0; i < tap_pending_scans_num; i++) {
		uint8_t *buffer = tap_pending_scans[i].buffer;
		int length = tap_pending_scans[i].length;
		int first = tap_pending_scans[i].first;
		struct scan_command *command = tap_pending_scans[i].command;

		/* copy bits from buffer */
		buf_set_buf(in_buf, first, buffer, 0, length);

		/* return buffer to higher level */
		if (jtag_read_buffer(buffer, command) != ERROR_OK) {
			buspirate_tap_init();
			return ERROR_JTAG_QUEUE_FAILED;
		}

		free(buffer);
	}
	buspirate_tap_init();
	return ERROR_OK;
}

static void buspirate_tap_make_space(int scans, int bits)
{
	int have_scans = BUSPIRATE_MAX_PENDING_SCANS - tap_pending_scans_num;
	int have_bits = BUSPIRATE_BUFFER_SIZE * 8 - tap_chain_index;

	if ((have_scans < scans) || (have_bits < bits))
		buspirate_tap_execute();
}

static void buspirate_tap_append(int tms, int tdi)
{
	int chain_index;

	buspirate_tap_make_space(0, 1);
	chain_index = tap_chain_index / 8;

	if (chain_index < BUSPIRATE_BUFFER_SIZE) {
		int bit_index = tap_chain_index % 8;
		uint8_t bit = 1 << bit_index;

		if (0 == bit_index) {
			/* Let's say that the TAP shift operation wants to shift 9 bits,
			   so we will be sending to the Bus Pirate a bit count of 9 but still
			   full 16 bits (2 bytes) of shift data.
			   If we don't clear all bits at this point, the last 7 bits will contain
			   random data from the last buffer contents, which is not pleasant to the eye.
			   Besides, the Bus Pirate (or some clone) may want to assert in debug builds
			   that, after consuming all significant data bits, the rest of them are zero.
			   Therefore, for aesthetic and for assert purposes, we clear all bits below. */
			tms_chain[chain_index] = 0;
			tdi_chain[chain_index] = 0;
		}

		if (tms)
			tms_chain[chain_index] |= bit;
		else
			tms_chain[chain_index] &= ~bit;

		if (tdi)
			tdi_chain[chain_index] |= bit;
		else
			tdi_chain[chain_index] &= ~bit;

		tap_chain_index++;
	} else {
		LOG_ERROR("tap_chain overflow, bad things will happen");
		/* Exit abruptly, like jlink.c does. After a buffer overflow we don't want
		   to carry on, as data will be corrupt. Another option would be to return
		   some error code at this point. */
		exit(-1);
	}
}

static void buspirate_tap_append_scan(int length, uint8_t *buffer,
		struct scan_command *command)
{
	int i;
	tap_pending_scans[tap_pending_scans_num].length = length;
	tap_pending_scans[tap_pending_scans_num].buffer = buffer;
	tap_pending_scans[tap_pending_scans_num].command = command;
	tap_pending_scans[tap_pending_scans_num].first = tap_chain_index;

	for (i = 0; i < length; i++) {
		int tms = (i < length-1 ? 0 : 1);
		int tdi = (buffer[i/8] >> (i%8)) & 1;
		buspirate_tap_append(tms, tdi);
	}
	tap_pending_scans_num++;
}

/*************** jtag wrapper functions *********************/

/* (1) assert or (0) deassert reset lines */
static void buspirate_reset(int trst, int srst)
{
	LOG_DEBUG("trst: %i, srst: %i", trst, srst);

	if (trst)
		buspirate_jtag_set_feature(buspirate_fd,
				FEATURE_TRST, ACTION_DISABLE);
	else
		buspirate_jtag_set_feature(buspirate_fd,
				FEATURE_TRST, ACTION_ENABLE);

	if (srst)
		buspirate_jtag_set_feature(buspirate_fd,
				FEATURE_SRST, ACTION_DISABLE);
	else
		buspirate_jtag_set_feature(buspirate_fd,
				FEATURE_SRST, ACTION_ENABLE);
}

/*************** jtag lowlevel functions ********************/
static void buspirate_jtag_enable(int fd)
{
	int ret;
	char tmp[21] = { [0 ... 20] = 0x00 };
	int done = 0;
	int cmd_sent = 0;

	LOG_DEBUG("Entering binary mode");
	buspirate_serial_write(fd, tmp, 20);
	usleep(10000);

	/* reads 1 to n "BBIO1"s and one "OCD1" */
	while (!done) {
		ret = buspirate_serial_read(fd, tmp, 4);
		if (ret != 4) {
			LOG_ERROR("Buspirate error. Is binary"
				"/OpenOCD support enabled?");
			exit(-1);
		}
		if (strncmp(tmp, "BBIO", 4) == 0) {
			ret = buspirate_serial_read(fd, tmp, 1);
			if (ret != 1) {
				LOG_ERROR("Buspirate did not answer correctly! "
					"Do you have correct firmware?");
				exit(-1);
			}
			if (tmp[0] != '1') {
				LOG_ERROR("Unsupported binary protocol");
				exit(-1);
			}
			if (cmd_sent == 0) {
				cmd_sent = 1;
				tmp[0] = CMD_ENTER_OOCD;
				ret = buspirate_serial_write(fd, tmp, 1);
				if (ret != 1) {
					LOG_ERROR("error reading");
					exit(-1);
				}
			}
		} else if (strncmp(tmp, "OCD1", 4) == 0)
			done = 1;
		else {
			LOG_ERROR("Buspirate did not answer correctly! "
				"Do you have correct firmware?");
			exit(-1);
		}
	}

}

static void buspirate_jtag_reset(int fd)
{
	char tmp[5];

	tmp[0] = 0x00; /* exit OCD1 mode */
	buspirate_serial_write(fd, tmp, 1);
	usleep(10000);
	/* We ignore the return value here purposly, nothing we can do */
	buspirate_serial_read(fd, tmp, 5);
	if (strncmp(tmp, "BBIO1", 5) == 0) {
		tmp[0] = 0x0F; /*  reset BP */
		buspirate_serial_write(fd, tmp, 1);
	} else
		LOG_ERROR("Unable to restart buspirate!");
}

static void buspirate_jtag_set_speed(int fd, char speed)
{
	int ret;
	char tmp[2];
	char ack[2];

	ack[0] = 0xAA;
	ack[1] = 0x55;

	tmp[0] = CMD_UART_SPEED;
	tmp[1] = speed;
	buspirate_jtag_command(fd, tmp, 2);

	/* here the adapter changes speed, we need follow */
	if (-1 == buspirate_serial_setspeed(fd, speed, NORMAL_TIMEOUT)) {
		LOG_ERROR("Error configuring the serial port.");
		exit(-1);
	}

	buspirate_serial_write(fd, ack, 2);
	ret = buspirate_serial_read(fd, tmp, 2);
	if (ret != 2) {
		LOG_ERROR("Buspirate did not ack speed change");
		exit(-1);
	}
	if ((tmp[0] != CMD_UART_SPEED) || (tmp[1] != speed)) {
		LOG_ERROR("Buspirate did not reply as expected to the speed change command");
		exit(-1);
	}
	LOG_INFO("Buspirate switched to %s mode",
		(speed == SERIAL_NORMAL) ? "normal" : "FAST");
}


static void buspirate_jtag_set_mode(int fd, char mode)
{
	char tmp[2];
	tmp[0] = CMD_PORT_MODE;
	tmp[1] = mode;
	buspirate_jtag_command(fd, tmp, 2);
}

static void buspirate_jtag_set_feature(int fd, char feat, char action)
{
	char tmp[3];
	tmp[0] = CMD_FEATURE;
	tmp[1] = feat;   /* what */
	tmp[2] = action; /* action */
	buspirate_jtag_command(fd, tmp, 3);
}

static void buspirate_jtag_get_adcs(int fd)
{
	uint8_t tmp[10];
	uint16_t a, b, c, d;
	tmp[0] = CMD_READ_ADCS;
	buspirate_jtag_command(fd, (char *)tmp, 1);
	a = tmp[2] << 8 | tmp[3];
	b = tmp[4] << 8 | tmp[5];
	c = tmp[6] << 8 | tmp[7];
	d = tmp[8] << 8 | tmp[9];

	LOG_INFO("ADC: ADC_Pin = %.02f VPullup = %.02f V33 = %.02f "
		"V50 = %.02f",
		((float)a)/155.1515, ((float)b)/155.1515,
		((float)c)/155.1515, ((float)d)/155.1515);
}

static unsigned char buspirate_jtag_command(int fd,
		char *cmd, int cmdlen)
{
	int res;
	int len = 0;

	res = buspirate_serial_write(fd, cmd, cmdlen);

	if ((cmd[0] == CMD_UART_SPEED)
				|| (cmd[0] == CMD_PORT_MODE)
				|| (cmd[0] == CMD_FEATURE)
				|| (cmd[0] == CMD_JTAG_SPEED))
		return 1;

	if (res == cmdlen) {
		switch (cmd[0]) {
		case CMD_READ_ADCS:
			len = 10; /* 2*sizeof(char)+4*sizeof(uint16_t) */
			break;
		case CMD_TAP_SHIFT:
			len = cmdlen;
			break;
		default:
			LOG_INFO("Wrong !");
		}
		res =  buspirate_serial_read(fd, cmd, len);
		if (res > 0)
			return (unsigned char)cmd[1];
		else
			return -1;
	} else
		return -1;
	return 0;
}

/* low level serial port */
/* TODO add support for WIN32 and others ! */
static int buspirate_serial_open(char *port)
{
	int fd;
	fd = open(buspirate_port, O_RDWR | O_NOCTTY | O_NDELAY);
	return fd;
}


/* Returns -1 on error. */

static int buspirate_serial_setspeed(int fd, char speed, cc_t timeout)
{
	struct termios t_opt;
	speed_t baud = (speed == SERIAL_FAST) ? B1000000 : B115200;

	/* set the serial port parameters */
	fcntl(fd, F_SETFL, 0);
	if (0 != tcgetattr(fd, &t_opt))
		return -1;

	if (0 != cfsetispeed(&t_opt, baud))
		return -1;

	if (0 != cfsetospeed(&t_opt, baud))
		return -1;

	t_opt.c_cflag |= (CLOCAL | CREAD);
	t_opt.c_cflag &= ~PARENB;
	t_opt.c_cflag &= ~CSTOPB;
	t_opt.c_cflag &= ~CSIZE;
	t_opt.c_cflag |= CS8;
	t_opt.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);

	/* The serial port may have been configured for human interaction with
	   the Bus Pirate console, but OpenOCD is going to use a binary protocol,
	   so make sure to turn off any CR/LF translation and the like. */
	t_opt.c_iflag &= ~(IXON | IXOFF | IXANY | INLCR | ICRNL);

	t_opt.c_oflag &= ~OPOST;
	t_opt.c_cc[VMIN] = 0;
	t_opt.c_cc[VTIME] = timeout;

	/* Note that, in the past, TCSANOW was used below instead of TCSADRAIN,
	   and CMD_UART_SPEED did not work properly then, at least with
	   the Bus Pirate v3.5 (USB). */
	if (0 != tcsetattr(fd, TCSADRAIN, &t_opt)) {
		/* According to the Linux documentation, this is actually not enough
		   to detect errors, you need to call tcgetattr() and check that
		   all changes have been performed successfully. */
		return -1;
	}

	return 0;
}

static int buspirate_serial_write(int fd, char *buf, int size)
{
	int ret = 0;

	ret = write(fd, buf, size);

	LOG_DEBUG("size = %d ret = %d", size, ret);
	buspirate_print_buffer(buf, size);

	if (ret != size)
		LOG_ERROR("Error sending data");

	return ret;
}

static int buspirate_serial_read(int fd, char *buf, int size)
{
	int len = 0;
	int ret = 0;
	int timeout = 0;

	while (len < size) {
		ret = read(fd, buf+len, size-len);
		if (ret == -1)
			return -1;

		if (ret == 0) {
			timeout++;

			if (timeout >= 10)
				break;

			continue;
		}

		len += ret;
	}

	LOG_DEBUG("should have read = %d actual size = %d", size, len);
	buspirate_print_buffer(buf, len);

	if (len != size)
		LOG_ERROR("Error reading data");

	return len;
}

static void buspirate_serial_close(int fd)
{
	close(fd);
}

#define LINE_SIZE      81
#define BYTES_PER_LINE 16
static void buspirate_print_buffer(char *buf, int size)
{
	char line[LINE_SIZE];
	char tmp[10];
	int offset = 0;

	line[0] = 0;
	while (offset < size) {
		snprintf(tmp, 5, "%02x ", (uint8_t)buf[offset]);
		offset++;

		strcat(line, tmp);

		if (offset % BYTES_PER_LINE == 0) {
			LOG_DEBUG("%s", line);
			line[0] = 0;
		}
	}

	if (line[0] != 0)
		LOG_DEBUG("%s", line);
}
