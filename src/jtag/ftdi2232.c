/***************************************************************************
 *   Copyright (C) 2004 by Dominic Rath                                    *
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

/* project specific includes */
#include "log.h"
#include "types.h"
#include "jtag.h"
#include "configuration.h"
#include "command.h"

/* system includes */
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <usb.h>
#include <ftdi.h>

#include <sys/time.h>
#include <time.h>

/* enable this to debug io latency
 */
#if 0
#define _DEBUG_USB_IO_
#endif

int ftdi2232_execute_queue(void);

int ftdi2232_speed(int speed);
int ftdi2232_register_commands(struct command_context_s *cmd_ctx);
int ftdi2232_init(void);
int ftdi2232_quit(void);

enum { FTDI2232_TRST = 0x10, FTDI2232_SRST = 0x40 };
static u8 discrete_output = 0x0 | FTDI2232_TRST | FTDI2232_SRST;
static struct ftdi_context ftdic;

static u8 *ftdi2232_buffer = NULL;
static int ftdi2232_buffer_size = 0;
static int ftdi2232_read_pointer = 0;
static int ftdi2232_expect_read = 0;
#define FTDI2232_BUFFER_SIZE	131072
#define BUFFER_ADD ftdi2232_buffer[ftdi2232_buffer_size++]
#define BUFFER_READ ftdi2232_buffer[ftdi2232_read_pointer++]

#define FTDI2232_SAVE_SIZE	1024

int ftdi2232_handle_vid_pid_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

static u16 ftdi2232_vid = 0x0403;
static u16 ftdi2232_pid = 0x6010;

jtag_interface_t ftdi2232_interface = 
{
	
	.name = "ftdi2232",
	
	.execute_queue = ftdi2232_execute_queue,
	
	.support_statemove = 1,
	
	.speed = ftdi2232_speed,
	.register_commands = ftdi2232_register_commands,
	.init = ftdi2232_init,
	.quit = ftdi2232_quit,
};

int ftdi2232_speed(int speed)
{
	u8 buf[3];

	buf[0] = 0x86; /* command "set divisor" */
	buf[1] = speed & 0xff; /* valueL (0=6MHz, 1=3MHz, 2=1.5MHz, ...*/
	buf[2] = (speed >> 8) & 0xff; /* valueH */
	
	DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);
	ftdi_write_data(&ftdic, buf, 3);

	return ERROR_OK;
}

int ftdi2232_register_commands(struct command_context_s *cmd_ctx)
{
	register_command(cmd_ctx, NULL, "ftdi2232_vid_pid", ftdi2232_handle_vid_pid_command,
		COMMAND_CONFIG, NULL);
	
	return ERROR_OK;
}

void ftdi2232_end_state(state)
{
	if (tap_move_map[state] != -1)
		end_state = state;
	else
	{
		ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

void ftdi2232_read_scan(enum scan_type type, u8* buffer, int scan_size)
{
	int num_bytes = ((scan_size + 7) / 8);
	int bits_left = scan_size;
	int cur_byte = 0;

	while(num_bytes-- > 1)
	{
		buffer[cur_byte] = BUFFER_READ;
		cur_byte++;
		bits_left -= 8;
	}

	buffer[cur_byte] = 0x0;

	if (bits_left > 1)
	{
		buffer[cur_byte] = BUFFER_READ >> 1;
	}

	buffer[cur_byte] = (buffer[cur_byte] | ((BUFFER_READ & 0x02) << 6)) >> (8 - bits_left);

}

void ftdi2232_debug_dump_buffer(void)
{
	int i;
	for (i = 0; i < ftdi2232_buffer_size; i++)
	{
		printf("%2.2x ", ftdi2232_buffer[i]);
		if (i % 16 == 15)
			printf("\n");
	}
	printf("\n");
	fflush(stdout);
}

int ftdi2232_send_and_recv(jtag_command_t *first, jtag_command_t *last)
{
	jtag_command_t *cmd;
	u8 *buffer;
	int scan_size;
	enum scan_type type;
	int retval;

	BUFFER_ADD = 0x87;	/* send immediate command */
	
	if (ftdi2232_buffer_size > FTDI2232_SAVE_SIZE)
	{
		ERROR("BUG: ftdi2232_buffer grew beyond %i byte (%i) - this is going to fail", FTDI2232_SAVE_SIZE,  ftdi2232_buffer_size);
	}

#ifdef _DEBUG_USB_IO_
	DEBUG("write buffer (size %i):", ftdi2232_buffer_size);
	ftdi2232_debug_dump_buffer();
#endif

	if ((retval = ftdi_write_data(&ftdic, ftdi2232_buffer, ftdi2232_buffer_size)) < 0)
	{
		ERROR("ftdi_write_data returned %i", retval);
		exit(-1);
	}

	if (ftdi2232_expect_read)
	{
		int timeout = 100;
		ftdi2232_buffer_size = 0;
		
		while ((ftdi2232_buffer_size < ftdi2232_expect_read) && timeout)
		{
			ftdi2232_buffer_size += ftdi_read_data(&ftdic, ftdi2232_buffer + ftdi2232_buffer_size, FTDI2232_BUFFER_SIZE - ftdi2232_buffer_size);
			timeout--;
		}

		if (ftdi2232_expect_read != ftdi2232_buffer_size)
		{
			ERROR("ftdi2232_expect_read (%i) != ftdi2232_buffer_size (%i) (%i retries)", ftdi2232_expect_read, ftdi2232_buffer_size, 100 - timeout);
			ftdi2232_debug_dump_buffer();

			exit(-1);
		}

#ifdef _DEBUG_USB_IO_
		DEBUG("read buffer (%i retries): %i bytes", 100 - timeout, ftdi2232_buffer_size);
		ftdi2232_debug_dump_buffer();	
#endif
	}

	ftdi2232_expect_read = 0;
	ftdi2232_read_pointer = 0;

	cmd = first;
	while (cmd != last)
	{
		switch (cmd->type)
		{
			case JTAG_SCAN:
				type = jtag_scan_type(cmd->cmd.scan);
				if (type != SCAN_OUT)
				{
					scan_size = jtag_scan_size(cmd->cmd.scan);
					buffer = calloc(CEIL(scan_size, 8), 1);
					ftdi2232_read_scan(type, buffer, scan_size);
					jtag_read_buffer(buffer, cmd->cmd.scan);
					free(buffer);
				}
				break;
			default:
				break;
		}
		cmd = cmd->next;
	}
	
	ftdi2232_buffer_size = 0;

	return ERROR_OK;
}

void ftdi2232_add_scan(int ir_scan, enum scan_type type, u8 *buffer, int scan_size)
{
	int num_bytes = (scan_size + 7) / 8;
	int bits_left = scan_size;
	int cur_byte = 0;
	int last_bit;

	/* command "Clock Data to TMS/CS Pin (no Read)" */
	BUFFER_ADD = 0x4b;
	/* scan 7 bit */
	BUFFER_ADD = 0x6;
	/* TMS data bits */
	if (ir_scan)
	{
		BUFFER_ADD = TAP_MOVE(cur_state, TAP_SI);
		cur_state = TAP_SI;
	}
	else
	{
		BUFFER_ADD = TAP_MOVE(cur_state, TAP_SD);
		cur_state = TAP_SD;
	}
	//DEBUG("added TMS scan (no read)");

	/* add command for complete bytes */
	if (num_bytes > 1)
	{
		if (type == SCAN_IO)
		{
			/* Clock Data Bytes In and Out LSB First */
			BUFFER_ADD = 0x39;
			//DEBUG("added TDI bytes (io %i)", num_bytes);
		}
		else if (type == SCAN_OUT)
		{
			/* Clock Data Bytes Out on -ve Clock Edge LSB First (no Read) */
			BUFFER_ADD = 0x19;
			//DEBUG("added TDI bytes (o)");
		}
		else if (type == SCAN_IN)
		{
			/* Clock Data Bytes In on +ve Clock Edge LSB First (no Write) */
			BUFFER_ADD = 0x28;
			//DEBUG("added TDI bytes (i %i)", num_bytes);
		}
		BUFFER_ADD = (num_bytes-2) & 0xff;
		BUFFER_ADD = ((num_bytes-2) >> 8) & 0xff;
	}
	if (type != SCAN_IN)
	{
		/* add complete bytes */
		while(num_bytes-- > 1)
		{
			BUFFER_ADD = buffer[cur_byte];
			cur_byte++;
			bits_left -= 8;
		}
	}
	if (type == SCAN_IN)
	{
		bits_left -= 8 * (num_bytes - 1);
	}

	/* the most signifcant bit is scanned during TAP movement */
	if (type != SCAN_IN)
		last_bit = (buffer[cur_byte] >> (bits_left - 1)) & 0x1;
	else
		last_bit = 0;

	/* process remaining bits but the last one */
	if (bits_left > 1)
	{
		if (type == SCAN_IO)
		{
			/* Clock Data Bits In and Out LSB First */
			BUFFER_ADD = 0x3b;
			//DEBUG("added TDI bits (io) %i", bits_left - 1);
		}
		else if (type == SCAN_OUT)
		{
			/* Clock Data Bits Out on -ve Clock Edge LSB First (no Read) */
			BUFFER_ADD = 0x1b;
			//DEBUG("added TDI bits (o)");
		}
		else if (type == SCAN_IN)
		{
			/* Clock Data Bits In on +ve Clock Edge LSB First (no Write) */
			BUFFER_ADD = 0x2a;
			//DEBUG("added TDI bits (i %i)", bits_left - 1);
		}
		BUFFER_ADD = bits_left - 2;
		if (type != SCAN_IN)
			BUFFER_ADD = buffer[cur_byte];
	}

	/* move from Shift-IR/DR to end state */
	if (type != SCAN_OUT)
	{
		/* Clock Data to TMS/CS Pin with Read */
		BUFFER_ADD = 0x6b;
		//DEBUG("added TMS scan (read)");
	}
	else
	{
		/* Clock Data to TMS/CS Pin (no Read) */
		BUFFER_ADD = 0x4b;
		//DEBUG("added TMS scan (no read)");
	}
	BUFFER_ADD = 0x6;
	BUFFER_ADD = TAP_MOVE(cur_state, end_state) | (last_bit << 7);
	cur_state = end_state;

}

int ftdi2232_predict_scan_out(int scan_size, enum scan_type type)
{
	int predicted_size = 6;
	if (type == SCAN_IN)	/* only from device to host */
	{
		predicted_size += (CEIL(scan_size, 8) > 1) ? 3 : 0;
		predicted_size += ((scan_size - 1) % 8) ? 2 : 0;
	}
	else					/* host to device, or bidirectional */
	{
		predicted_size += (CEIL(scan_size, 8) > 1) ? (CEIL(scan_size, 8) + 3 - 1) : 0;
		predicted_size += ((scan_size - 1) % 8) ? 3 : 0;
	}

	return predicted_size;
}

int ftdi2232_predict_scan_in(int scan_size, enum scan_type type)
{
	int predicted_size = 0;
	
	if (type != SCAN_OUT)
	{
		/* complete bytes */
		predicted_size += (CEIL(scan_size, 8) > 1) ? (CEIL(scan_size, 8) - 1) : 0;
		/* remaining bits - 1 */
		predicted_size += ((scan_size - 1) % 8) ? 1 : 0;
		/* last bit (from TMS scan) */
		predicted_size += 1;
	}
	
	//DEBUG("scan_size: %i, predicted_size: %i", scan_size, predicted_size);

	return predicted_size;
}

int ftdi2232_execute_queue()
{
	jtag_command_t *cmd = jtag_command_queue; /* currently processed command */
	jtag_command_t *first_unsent = cmd;	/* next command that has to be sent */
	u8 *buffer;
	int scan_size;	/* size of IR or DR scan */
	enum scan_type type;
	int i;
	int predicted_size = 0;
	int require_send = 0;

	ftdi2232_buffer_size = 0;
	ftdi2232_expect_read = 0;

	while (cmd)
	{
		switch(cmd->type)
		{
			case JTAG_END_STATE:
				if (cmd->cmd.end_state->end_state != -1)
					ftdi2232_end_state(cmd->cmd.end_state->end_state);
				break;
			case JTAG_RESET:
				/* only send the maximum buffer size that FT2232C can handle */
				predicted_size = 3;
				if (ftdi2232_buffer_size + predicted_size + 1 > FTDI2232_SAVE_SIZE)
				{
					ftdi2232_send_and_recv(first_unsent, cmd);
					require_send = 0;
					first_unsent = cmd;
				}

				if (cmd->cmd.reset->trst == 1)
				{
					cur_state = TAP_TLR;
					discrete_output &= ~FTDI2232_TRST;
				}
				else if (cmd->cmd.reset->trst == 0)
				{
					discrete_output |= FTDI2232_TRST;
				}

				if (cmd->cmd.reset->srst == 1)
					discrete_output &= ~FTDI2232_SRST;
				else if (cmd->cmd.reset->srst == 0)
					discrete_output |= FTDI2232_SRST;
				/* command "set data bits low byte" */
				BUFFER_ADD = 0x80;
				/* value (TMS=1,TCK=0, TDI=0, TRST/SRST */
				BUFFER_ADD = 0x08 | discrete_output;
				/* dir (output=1), TCK/TDI/TMS=out, TDO=in, TRST/SRST=out */
				BUFFER_ADD = 0x0b | FTDI2232_SRST | FTDI2232_TRST;
				require_send = 1;
				break;
			case JTAG_RUNTEST:
				/* only send the maximum buffer size that FT2232C can handle */
				predicted_size = 0;
				if (cur_state != TAP_RTI)
					predicted_size += 3;
				predicted_size += 3 * CEIL(cmd->cmd.runtest->num_cycles, 7);
				if ((cmd->cmd.runtest->end_state != -1) && (cmd->cmd.runtest->end_state != TAP_RTI))
					predicted_size += 3;
				if ((cmd->cmd.runtest->end_state == -1) && (end_state != TAP_RTI))
					predicted_size += 3;
				if (ftdi2232_buffer_size + predicted_size + 1 > FTDI2232_SAVE_SIZE)
				{
					ftdi2232_send_and_recv(first_unsent, cmd);
					require_send = 0;
					first_unsent = cmd;
				}
				if (cur_state != TAP_RTI)
				{
					/* command "Clock Data to TMS/CS Pin (no Read)" */
					BUFFER_ADD = 0x4b;
					/* scan 7 bit */
					BUFFER_ADD = 0x6;
					/* TMS data bits */
					BUFFER_ADD = TAP_MOVE(cur_state, TAP_RTI);
					cur_state = TAP_RTI;
					require_send = 1;
				}
				i = cmd->cmd.runtest->num_cycles;
				while (i > 0)
				{
					/* command "Clock Data to TMS/CS Pin (no Read)" */
					BUFFER_ADD = 0x4b;
					/* scan 7 bit */
					BUFFER_ADD = (i > 7) ? 6 : (i - 1);
					/* TMS data bits */
					BUFFER_ADD = 0x0;
					cur_state = TAP_RTI;
					i -= (i > 7) ? 7 : i;
					//DEBUG("added TMS scan (no read)");
				}
				if (cmd->cmd.runtest->end_state != -1)
					ftdi2232_end_state(cmd->cmd.runtest->end_state);
				if (cur_state != end_state)
				{
					/* command "Clock Data to TMS/CS Pin (no Read)" */
					BUFFER_ADD = 0x4b;
					/* scan 7 bit */
					BUFFER_ADD = 0x6;
					/* TMS data bits */
					BUFFER_ADD = TAP_MOVE(cur_state, end_state);
					cur_state = end_state;
					//DEBUG("added TMS scan (no read)");
				}
				require_send = 1;
				break;
			case JTAG_STATEMOVE:
				/* only send the maximum buffer size that FT2232C can handle */
				predicted_size = 3;
				if (ftdi2232_buffer_size + predicted_size + 1 > FTDI2232_SAVE_SIZE)
				{
					ftdi2232_send_and_recv(first_unsent, cmd);
					require_send = 0;
					first_unsent = cmd;
				}
				if (cmd->cmd.statemove->end_state != -1)
					ftdi2232_end_state(cmd->cmd.statemove->end_state);
				/* command "Clock Data to TMS/CS Pin (no Read)" */
				BUFFER_ADD = 0x4b;
				/* scan 7 bit */
				BUFFER_ADD = 0x6;
				/* TMS data bits */
				BUFFER_ADD = TAP_MOVE(cur_state, end_state);
				//DEBUG("added TMS scan (no read)");
				cur_state = end_state;
				require_send = 1;
				break;
			case JTAG_SCAN:
				scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
				type = jtag_scan_type(cmd->cmd.scan);
				predicted_size = ftdi2232_predict_scan_out(scan_size, type);
				if (ftdi2232_buffer_size + predicted_size + 1 > FTDI2232_SAVE_SIZE)
				{
					ftdi2232_send_and_recv(first_unsent, cmd);
					require_send = 0;
					first_unsent = cmd;
				}
				ftdi2232_expect_read += ftdi2232_predict_scan_in(scan_size, type);
				//DEBUG("new read size: %i", ftdi2232_expect_read);
				if (cmd->cmd.scan->end_state != -1)
					ftdi2232_end_state(cmd->cmd.scan->end_state);
				ftdi2232_add_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size);
				require_send = 1;
				if (buffer)
					free(buffer);
				break;
			case JTAG_SLEEP:
				jtag_sleep(cmd->cmd.sleep->us);
				break;
			default:
				ERROR("BUG: unknown JTAG command type encountered");
				exit(-1);
		}
		cmd = cmd->next;
	}

	if (require_send > 0)
		ftdi2232_send_and_recv(first_unsent, cmd);

	return ERROR_OK;
}

int ftdi2232_init(void)
{
	if (ftdi_init(&ftdic) < 0)
		return ERROR_JTAG_INIT_FAILED;

	/* context, vendor id, product id */
	if (ftdi_usb_open(&ftdic, ftdi2232_vid, ftdi2232_pid) < 0)
	{
		ERROR("unable to open ftdi device: %s", ftdic.error_str);
		return ERROR_JTAG_INIT_FAILED;
	}

	if (ftdi_usb_reset(&ftdic) < 0)
	{
		ERROR("unable to reset ftdi device");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (ftdi_set_latency_timer(&ftdic, 1) < 0)
	{
		ERROR("unable to set latency timer");
		return ERROR_JTAG_INIT_FAILED;
	}

	ftdi2232_buffer_size = 0;
	ftdi2232_buffer = malloc(FTDI2232_BUFFER_SIZE);

	ftdic.bitbang_mode = 0; /* Reset controller */
	ftdi_enable_bitbang(&ftdic, 0x0b | FTDI2232_SRST | FTDI2232_TRST); /* ctx, i/o mask (out=1, in=0) */

	ftdic.bitbang_mode = 2; /* MPSSE mode */
	ftdi_enable_bitbang(&ftdic, 0x0b | FTDI2232_SRST | FTDI2232_TRST); /* ctx, i/o mask (out=1, in=0) */
	
	if (ftdi_usb_purge_buffers(&ftdic) < 0)
	{
		ERROR("ftdi_purge_buffers: %s", ftdic.error_str);
		return ERROR_JTAG_INIT_FAILED;
	}

	/* initialize low byte for jtag */
	BUFFER_ADD = 0x80; /* command "set data bits low byte" */
	BUFFER_ADD = 0x08 | FTDI2232_SRST | FTDI2232_TRST; /* value (TMS=1,TCK=0, TDI=0, xRST high) */
	BUFFER_ADD = 0x0b | FTDI2232_SRST | FTDI2232_TRST; /* dir (output=1), TCK/TDI/TMS=out, TDO=in */
	BUFFER_ADD = 0x85; /* command "Disconnect TDI/DO to TDO/DI for Loopback" */
	ftdi2232_debug_dump_buffer();
	if (ftdi_write_data(&ftdic, ftdi2232_buffer, ftdi2232_buffer_size) != 4)
		return ERROR_JTAG_INIT_FAILED;

	ftdi2232_speed(jtag_speed);

	return ERROR_OK;
}

int ftdi2232_quit(void)
{
	ftdi_disable_bitbang(&ftdic);
	
	ftdi_usb_close(&ftdic);
	
	ftdi_deinit(&ftdic);

	free(ftdi2232_buffer);

	return ERROR_OK;
}

int ftdi2232_handle_vid_pid_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc >= 2)
	{
		ftdi2232_vid = strtol(args[0], NULL, 0);
		ftdi2232_pid = strtol(args[1], NULL, 0);
	}
	else
	{
		WARNING("incomplete ftdi2232_vid_pid configuration directive");
	}
	
	return ERROR_OK;
}
