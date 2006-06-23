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
#include "config.h"
#if IS_CYGWIN == 1
#include "windows.h"
#undef ERROR
#endif

/* project specific includes */
#include "log.h"
#include "types.h"
#include "jtag.h"
#include "configuration.h"

/* system includes */
#include <string.h>
#include <stdlib.h>
#include <unistd.h>
#include <ftd2xx.h>

#include <sys/time.h>
#include <time.h>

/* enable this to debug io latency
 */
#if 0
#define _DEBUG_USB_IO_
#endif

/* enable this to debug communication
 */
#if 0
#define _DEBUG_USB_COMMS_
#endif

/* enable this to work around ftd2xx deadlock
 */
#if 0
#define _FTD2XX_QUEUE_DELAY_
#endif

int ftd2xx_execute_queue(void);

int ftd2xx_speed(int speed);
int ftd2xx_register_commands(struct command_context_s *cmd_ctx);
int ftd2xx_init(void);
int ftd2xx_quit(void);

int ftd2xx_handle_device_desc_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int ftd2xx_handle_layout_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int ftd2xx_handle_vid_pid_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

char *ftd2xx_device_desc = NULL;
char *ftd2xx_layout = NULL;
u16 ftd2xx_vid = 0x0403;
u16 ftd2xx_pid = 0x6010;

typedef struct ftd2xx_layout_s
{
	char* name;
	int(*init)(void);
	void(*reset)(int trst, int srst);
} ftd2xx_layout_t;

int usbjtag_init(void);
int jtagkey_init(void);
void usbjtag_reset(int trst, int srst);
void jtagkey_reset(int trst, int srst);

ftd2xx_layout_t ftd2xx_layouts[] =
{
	{"usbjtag", usbjtag_init, usbjtag_reset},
	{"jtagkey", jtagkey_init, jtagkey_reset},
	{"jtagkey_prototype_v1", jtagkey_init, jtagkey_reset},
	{NULL, NULL, NULL},
};

static u8 nTRST, nTRSTnOE, nSRST, nSRSTnOE;

static ftd2xx_layout_t *layout;
static u8 low_output = 0x0;
static u8 low_direction = 0x0;
static u8 high_output = 0x0;
static u8 high_direction = 0x0;
static FT_HANDLE ftdih = NULL;

static u8 *ftd2xx_buffer = NULL;
static int ftd2xx_buffer_size = 0;
static int ftd2xx_read_pointer = 0;
static int ftd2xx_expect_read = 0;
#define FTD2XX_BUFFER_SIZE	131072
#define BUFFER_ADD ftd2xx_buffer[ftd2xx_buffer_size++]
#define BUFFER_READ ftd2xx_buffer[ftd2xx_read_pointer++]

jtag_interface_t ftd2xx_interface = 
{
	
	.name = "ftd2xx",
	
	.execute_queue = ftd2xx_execute_queue,
	
	.support_statemove = 1,
	
	.speed = ftd2xx_speed,
	.register_commands = ftd2xx_register_commands,
	.init = ftd2xx_init,
	.quit = ftd2xx_quit,
};

int ftd2xx_speed(int speed)
{
	u8 buf[3];
	FT_STATUS status;
	DWORD bytes_written;

	buf[0] = 0x86; /* command "set divisor" */
	buf[1] = speed & 0xff; /* valueL (0=6MHz, 1=3MHz, 2=1.5MHz, ...*/
	buf[2] = (speed >> 8) & 0xff; /* valueH */
	
	DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);
	if (((status = FT_Write(ftdih, buf, 3, &bytes_written)) != FT_OK) || (bytes_written != 3))
	{
		ERROR("couldn't write to ftdi device: %i", status);
		return status;
	}
	
	return ERROR_OK;
}

int ftd2xx_register_commands(struct command_context_s *cmd_ctx)
{
	register_command(cmd_ctx, NULL, "ftd2xx_device_desc", ftd2xx_handle_device_desc_command,
		COMMAND_CONFIG, NULL);
	register_command(cmd_ctx, NULL, "ftd2xx_layout", ftd2xx_handle_layout_command,
		COMMAND_CONFIG, NULL);
	register_command(cmd_ctx, NULL, "ftd2xx_vid_pid", ftd2xx_handle_vid_pid_command,
					 COMMAND_CONFIG, NULL);
	return ERROR_OK;
}

void ftd2xx_end_state(state)
{
	if (tap_move_map[state] != -1)
		end_state = state;
	else
	{
		ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

void ftd2xx_read_scan(enum scan_type type, u8* buffer, int scan_size)
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

void ftd2xx_debug_dump_buffer(void)
{
	int i;
	char line[256];
	char *line_p = line;
	
	for (i = 0; i < ftd2xx_buffer_size; i++)
	{
		line_p += snprintf(line_p, 256 - (line_p - line), "%2.2x ", ftd2xx_buffer[i]);
		if (i % 16 == 15)
		{
			DEBUG("%s", line);
			line_p = line;
		}
	}
	
	if (line_p != line)
		DEBUG("%s", line);
}

int ftd2xx_send_and_recv(jtag_command_t *first, jtag_command_t *last)
{
	jtag_command_t *cmd;
	u8 *buffer;
	int scan_size;
	enum scan_type type;
	FT_STATUS status;
	DWORD bytes_written;
	DWORD bytes_read;
	
#ifdef _DEBUG_USB_IO_
	struct timeval start, inter, inter2, end;
#endif

#ifdef _DEBUG_USB_COMMS_
	DEBUG("write buffer (size %i):", ftd2xx_buffer_size);
	ftd2xx_debug_dump_buffer();
#endif

#ifdef _DEBUG_USB_IO_
	gettimeofday(&start, NULL);	
#endif

	if ((status = FT_Write(ftdih, ftd2xx_buffer, ftd2xx_buffer_size, &bytes_written)) != FT_OK)
	{
		ERROR("couldn't write to ftdi device: %i", status);
		exit(-1);
	}
	
#ifdef _DEBUG_USB_IO_
	gettimeofday(&inter, NULL);	
#endif
	
	if (ftd2xx_expect_read)
	{
		int timeout = 100;
		ftd2xx_buffer_size = 0;
		
#ifdef _FTD2XX_QUEUE_DELAY_
		DWORD inrxqueue = 0;
		while (inrxqueue < ftd2xx_expect_read)
		{
			FT_GetQueueStatus(ftdih, &inrxqueue);
			if (inrxqueue >= ftd2xx_expect_read)
				break;
			usleep(1000);
		};
#endif
		
#ifdef _DEBUG_USB_IO_
	gettimeofday(&inter2, NULL);	
#endif
			
		if ((status = FT_Read(ftdih, ftd2xx_buffer, ftd2xx_expect_read, &bytes_read)) != FT_OK)
		{
			ERROR("couldn't read from ftdi device: %i", status);
			exit(-1);
		}

#ifdef _DEBUG_USB_IO_
		gettimeofday(&end, NULL);	

		INFO("inter: %i.%i, inter2: %i.%i end: %i.%i", inter.tv_sec - start.tv_sec, inter.tv_usec - start.tv_usec,
			inter2.tv_sec - start.tv_sec, inter2.tv_usec - start.tv_usec,
			end.tv_sec - start.tv_sec, end.tv_usec - start.tv_usec);
#endif
	
		
		ftd2xx_buffer_size = bytes_read;
		
		if (ftd2xx_expect_read != ftd2xx_buffer_size)
		{
			ERROR("ftd2xx_expect_read (%i) != ftd2xx_buffer_size (%i) (%i retries)", ftd2xx_expect_read, ftd2xx_buffer_size, 100 - timeout);
			ftd2xx_debug_dump_buffer();	

			exit(-1);
		}

#ifdef _DEBUG_USB_COMMS_
		DEBUG("read buffer (%i retries): %i bytes", 100 - timeout, ftd2xx_buffer_size);
		ftd2xx_debug_dump_buffer();
#endif
	}

	ftd2xx_expect_read = 0;
	ftd2xx_read_pointer = 0;

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
					ftd2xx_read_scan(type, buffer, scan_size);
					jtag_read_buffer(buffer, cmd->cmd.scan);
					free(buffer);
				}
				break;
			default:
				break;
		}
		cmd = cmd->next;
	}
	
	ftd2xx_buffer_size = 0;

	return ERROR_OK;
}

void ftd2xx_add_scan(int ir_scan, enum scan_type type, u8 *buffer, int scan_size)
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
		BUFFER_ADD = (num_bytes >> 8) & 0xff;
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

int ftd2xx_predict_scan_out(int scan_size, enum scan_type type)
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

int ftd2xx_predict_scan_in(int scan_size, enum scan_type type)
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

void usbjtag_reset(int trst, int srst)
{
	if (trst == 1)
	{
		cur_state = TAP_TLR;
		if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
			low_direction |= nTRSTnOE;	/* switch to output pin (output is low) */
		else
			low_output &= ~nTRST;	/* switch output low */
	}
	else if (trst == 0)
	{
		if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
			low_direction &= ~nTRSTnOE; /* switch to input pin (high-Z + internal and external pullup) */
		else
			low_output |= nTRST; /* switch output high */
	}

	if (srst == 1)
	{
		if (jtag_reset_config & RESET_SRST_PUSH_PULL)
			low_output &= ~nSRST;	/* switch output low */
		else
			low_direction |= nSRSTnOE;	/* switch to output pin (output is low) */
	}
	else if (srst == 0)
	{
		if (jtag_reset_config & RESET_SRST_PUSH_PULL)
			low_output |= nSRST;	/* switch output high */
		else
			low_direction &= ~nSRSTnOE;	/* switch to input pin (high-Z) */
	}
	
	/* command "set data bits low byte" */
	BUFFER_ADD = 0x80;
	BUFFER_ADD = low_output;
	BUFFER_ADD = low_direction;

}

void jtagkey_reset(int trst, int srst)
{
	if (trst == 1)
	{
		cur_state = TAP_TLR;
		if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
			high_output &= ~nTRSTnOE;
		else
			high_output &= ~nTRST;
	}
	else if (trst == 0)
	{
		if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
			high_output |= nTRSTnOE;
		else
			high_output |= nTRST;
	}

	if (srst == 1)
	{
		if (jtag_reset_config & RESET_SRST_PUSH_PULL)
			high_output &= ~nSRST;
		else
			high_output &= ~nSRSTnOE;
	}
	else if (srst == 0)
	{
		if (jtag_reset_config & RESET_SRST_PUSH_PULL)
			high_output |= nSRST;
		else
			high_output |= nSRSTnOE;
	}
	
	/* command "set data bits high byte" */
	BUFFER_ADD = 0x82;
	BUFFER_ADD = high_output;
	BUFFER_ADD = high_direction;
	DEBUG("trst: %i, srst: %i, high_output: 0x%2.2x, high_direction: 0x%2.2x", trst, srst, high_output, high_direction);
}

int ftd2xx_execute_queue()
{
	jtag_command_t *cmd = jtag_command_queue; /* currently processed command */
	jtag_command_t *first_unsent = cmd;	/* next command that has to be sent */
	u8 *buffer;
	int scan_size;	/* size of IR or DR scan */
	enum scan_type type;
	int i;
	int predicted_size = 0;
	int require_send = 0;

	ftd2xx_buffer_size = 0;
	ftd2xx_expect_read = 0;

	while (cmd)
	{
		switch(cmd->type)
		{
			case JTAG_END_STATE:
				if (cmd->cmd.end_state->end_state != -1)
					ftd2xx_end_state(cmd->cmd.end_state->end_state);
				break;
			case JTAG_RESET:
				/* only send the maximum buffer size that FT2232C can handle */
				predicted_size = 3;
				if (ftd2xx_buffer_size + predicted_size + 1 > FTD2XX_BUFFER_SIZE)
				{
					ftd2xx_send_and_recv(first_unsent, cmd);
					require_send = 0;
					first_unsent = cmd;
				}

				layout->reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
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
				if (ftd2xx_buffer_size + predicted_size + 1 > FTD2XX_BUFFER_SIZE)
				{
					ftd2xx_send_and_recv(first_unsent, cmd);
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
					ftd2xx_end_state(cmd->cmd.runtest->end_state);
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
				if (ftd2xx_buffer_size + predicted_size + 1 > FTD2XX_BUFFER_SIZE)
				{
					ftd2xx_send_and_recv(first_unsent, cmd);
					require_send = 0;
					first_unsent = cmd;
				}
				if (cmd->cmd.statemove->end_state != -1)
					ftd2xx_end_state(cmd->cmd.statemove->end_state);
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
				predicted_size = ftd2xx_predict_scan_out(scan_size, type);
				if (ftd2xx_buffer_size + predicted_size + 1 > FTD2XX_BUFFER_SIZE)
				{
					DEBUG("ftd2xx buffer size reached, sending queued commands (first_unsent: %x, cmd: %x)", first_unsent, cmd);
					ftd2xx_send_and_recv(first_unsent, cmd);
					require_send = 0;
					first_unsent = cmd;
				}
				ftd2xx_expect_read += ftd2xx_predict_scan_in(scan_size, type);
				//DEBUG("new read size: %i", ftd2xx_expect_read);
				if (cmd->cmd.scan->end_state != -1)
					ftd2xx_end_state(cmd->cmd.scan->end_state);
				ftd2xx_add_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size);
				require_send = 1;
				if (buffer)
					free(buffer);
				break;
			case JTAG_SLEEP:
				ftd2xx_send_and_recv(first_unsent, cmd);
				first_unsent = cmd->next;
				jtag_sleep(cmd->cmd.sleep->us);
				break;
			default:
				ERROR("BUG: unknown JTAG command type encountered");
				exit(-1);
		}
		cmd = cmd->next;
	}

	if (require_send > 0)
		ftd2xx_send_and_recv(first_unsent, cmd);

	return ERROR_OK;
}

int ftd2xx_init(void)
{
	u8 latency_timer;
	FT_STATUS status;
	DWORD num_devices;
	
	ftd2xx_layout_t *cur_layout = ftd2xx_layouts;
	
	if ((ftd2xx_layout == NULL) || (ftd2xx_layout[0] == 0))
	{
		ftd2xx_layout = "usbjtag";
		WARNING("No ftd2xx layout specified, using default 'usbjtag'");
	}
	
	while (cur_layout->name)
	{
		if (strcmp(cur_layout->name, ftd2xx_layout) == 0)
		{
			layout = cur_layout;
			break;
		}
		cur_layout++;
	}

	if (!layout)
	{
		ERROR("No matching layout found for %s", ftd2xx_layout);
		return ERROR_JTAG_INIT_FAILED;
	}
	
	if (ftd2xx_device_desc == NULL)
	{
		WARNING("no ftd2xx device description specified, using default 'Dual RS232'");
		ftd2xx_device_desc = "Dual RS232";
	}
	
#if IS_CYGWIN != 1
	/* Add JTAGkey Vid/Pid to the linux driver */
	if ((status = FT_SetVIDPID(ftd2xx_vid, ftd2xx_pid)) != FT_OK)
	{
		WARNING("couldn't add %4.4x:%4.4x", ftd2xx_vid, ftd2xx_pid);
	}
#endif

	if ((status = FT_OpenEx(ftd2xx_device_desc, FT_OPEN_BY_DESCRIPTION, &ftdih)) != FT_OK)
	{
		ERROR("unable to open ftdi device: %i", status);
		status = FT_ListDevices(&num_devices, NULL, FT_LIST_NUMBER_ONLY);
		if (status == FT_OK)
		{
			char **desc_array = malloc(sizeof(char*) * (num_devices + 1));
			int i;

			for (i = 0; i < num_devices; i++)
				desc_array[i] = malloc(64);
			desc_array[num_devices] = NULL;

			status = FT_ListDevices(desc_array, &num_devices, FT_LIST_ALL | FT_OPEN_BY_DESCRIPTION);

			if (status == FT_OK)
			{
				ERROR("ListDevices: %d\n", num_devices);
				for (i = 0; i < num_devices; i++)
					ERROR("%i: %s", i, desc_array[i]);
			}
			
			for (i = 0; i < num_devices; i++)
				free(desc_array[i]);
			free(desc_array);
		}
		else
		{
			printf("ListDevices: NONE\n");
		}
		return ERROR_JTAG_INIT_FAILED;
	}

	if ((status = FT_SetLatencyTimer(ftdih, 2)) != FT_OK)
	{
		ERROR("unable to set latency timer: %i", status);
		return ERROR_JTAG_INIT_FAILED;
	}
	
	if ((status = FT_GetLatencyTimer(ftdih, &latency_timer)) != FT_OK)
	{
		ERROR("unable to get latency timer: %i", status);
		return ERROR_JTAG_INIT_FAILED;
	}
	else
	{
		DEBUG("current latency timer: %i", latency_timer);
	}

	if ((status = FT_SetBitMode(ftdih, 0x0b, 2)) != FT_OK)
	{
		ERROR("unable to enable bit i/o mode: %i", status);
		return ERROR_JTAG_INIT_FAILED;
	}

	ftd2xx_buffer_size = 0;
	ftd2xx_buffer = malloc(FTD2XX_BUFFER_SIZE);

	if (layout->init() != ERROR_OK)
		return ERROR_JTAG_INIT_FAILED;

	ftd2xx_speed(jtag_speed);
	
	if ((status = FT_Purge(ftdih, FT_PURGE_RX | FT_PURGE_TX)) != FT_OK)
	{
		ERROR("error purging ftd2xx device: %i", status);
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

int usbjtag_init(void)
{
	u8 buf[3];
	FT_STATUS status;
	DWORD bytes_written;
	
	low_output = 0x08;
	low_direction = 0x0b;
	
	nTRST = 0x10;
	nTRSTnOE = 0x10;
	nSRST = 0x40;
	nSRSTnOE = 0x40;
	
	if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
	{
		low_direction &= ~nTRSTnOE; /* nTRST input */
		low_output &= ~nTRST; /* nTRST = 0 */
	}
	else
	{
		low_direction |= nTRSTnOE; /* nTRST output */
		low_output |= nTRST; /* nTRST = 1 */
	}
	
	if (jtag_reset_config & RESET_SRST_PUSH_PULL)
	{
		low_direction |= nSRSTnOE; /* nSRST output */
		low_output |= nSRST; /* nSRST = 1 */
	}
	else
	{
		low_direction &= ~nSRSTnOE; /* nSRST input */
		low_output &= ~nSRST; /* nSRST = 0 */
	}
	
	/* initialize low byte for jtag */
	buf[0] = 0x80; /* command "set data bits low byte" */
	buf[1] = low_output; /* value (TMS=1,TCK=0, TDI=0, xRST high) */
	buf[2] = low_direction; /* dir (output=1), TCK/TDI/TMS=out, TDO=in */
	DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);
	
	if (((FT_Write(ftdih, buf, 3, &bytes_written)) != FT_OK) || (bytes_written != 3))
	{
		ERROR("couldn't write to ftdi device: %i", status);
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

int jtagkey_init(void)
{
	u8 buf[3];
	FT_STATUS status;
	DWORD bytes_written;
	
	low_output = 0x08;
	low_direction = 0x1b;
	
	/* initialize low byte for jtag */
	buf[0] = 0x80; /* command "set data bits low byte" */
	buf[1] = low_output; /* value (TMS=1,TCK=0, TDI=0, nOE=0) */
	buf[2] = low_direction; /* dir (output=1), TCK/TDI/TMS=out, TDO=in, nOE=out */
	DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);
	
	if (((FT_Write(ftdih, buf, 3, &bytes_written)) != FT_OK) || (bytes_written != 3))
	{
		ERROR("couldn't write to ftdi device: %i", status);
		return ERROR_JTAG_INIT_FAILED;
	}
	
	if (strcmp(layout->name, "jtagkey") == 0)
	{
		nTRST = 0x01;
		nTRSTnOE = 0x4;
		nSRST = 0x02;
		nSRSTnOE = 0x08;
	}
	else if (strcmp(layout->name, "jtagkey_prototype_v1") == 0)
	{
		nTRST = 0x02;
		nTRSTnOE = 0x1;
		nSRST = 0x08;
		nSRSTnOE = 0x04;
	}
	else
	{
		ERROR("BUG: jtagkey_init called for non jtagkey layout");
		exit(-1);
	}
	
	high_output = 0x0;
	high_direction = 0x0f;

	if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
	{
		high_output |= nTRSTnOE;
		high_output &= ~nTRST;
	}
	else
	{
		high_output &= ~nTRSTnOE;
		high_output |= nTRST;
	}
	
	if (jtag_reset_config & RESET_SRST_PUSH_PULL)
	{
		high_output &= ~nSRSTnOE;
		high_output |= nSRST;
	}
	else
	{
		high_output |= nSRSTnOE;
		high_output &= ~nSRST;
	}
	
	/* initialize high port */
	buf[0] = 0x82; /* command "set data bits low byte" */
	buf[1] = high_output; /* value */
	buf[2] = high_direction;   /* all outputs (xRST and xRSTnOE) */
	DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);
	
	if (((FT_Write(ftdih, buf, 3, &bytes_written)) != FT_OK) || (bytes_written != 3))
	{
		ERROR("couldn't write to ftdi device: %i", status);
		return ERROR_JTAG_INIT_FAILED;
	}
	
	return ERROR_OK;
}

int ftd2xx_quit(void)
{
	FT_STATUS status;

	status = FT_Close(ftdih);

	free(ftd2xx_buffer);

	return ERROR_OK;
}

int ftd2xx_handle_device_desc_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc == 1)
	{
		ftd2xx_device_desc = strdup(args[0]);
	}
	else
	{
		ERROR("expected exactly one argument to ftd2xx_device_desc <description>");
	}
	
	return ERROR_OK;
}

int ftd2xx_handle_layout_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc == 0)
		return ERROR_OK;

	ftd2xx_layout = malloc(strlen(args[0]) + 1);
	strcpy(ftd2xx_layout, args[0]);

	return ERROR_OK;
}

int ftd2xx_handle_vid_pid_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc >= 2)
	{
		ftd2xx_vid = strtol(args[0], NULL, 0);
		ftd2xx_pid = strtol(args[1], NULL, 0);
	}
	else
	{
		WARNING("incomplete ftd2xx_vid_pid configuration directive");
	}
	
	return ERROR_OK;
}
