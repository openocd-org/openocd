/***************************************************************************
*   Copyright (C) 2004, 2006 by Dominic Rath                              *
*   Dominic.Rath@gmx.de                                                   *
*                                                                         *
*   Copyright (C) 2008 by Spencer Oliver                                  *
*   spen@spen-soft.co.uk                                                  *
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


/* This code uses information contained in the MPSSE specification which was
 * found here:
 * http://www.ftdichip.com/Documents/AppNotes/AN2232C-01_MPSSE_Cmnd.pdf
 * Hereafter this is called the "MPSSE Spec".
 */


#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#if IS_CYGWIN == 1
#include "windows.h"
#endif

#include "replacements.h"

/* project specific includes */
#include "log.h"
#include "types.h"
#include "jtag.h"
#include "configuration.h"
#include "time_support.h"

/* system includes */
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

/* FT2232 access library includes */
#if BUILD_FT2232_FTD2XX == 1
#include <ftd2xx.h>
#elif BUILD_FT2232_LIBFTDI == 1
#include <ftdi.h>
#endif

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

int ft2232_execute_queue(void);

int ft2232_speed(int speed);
int ft2232_speed_div(int speed, int* khz);
int ft2232_khz(int khz, int* jtag_speed);
int ft2232_register_commands(struct command_context_s* cmd_ctx);
int ft2232_init(void);
int ft2232_quit(void);

int ft2232_handle_device_desc_command(struct command_context_s* cmd_ctx, char* cmd, char** args, int argc);
int ft2232_handle_serial_command(struct command_context_s* cmd_ctx, char* cmd, char** args, int argc);
int ft2232_handle_layout_command(struct command_context_s* cmd_ctx, char* cmd, char** args, int argc);
int ft2232_handle_vid_pid_command(struct command_context_s* cmd_ctx, char* cmd, char** args, int argc);
int ft2232_handle_latency_command(struct command_context_s* cmd_ctx, char* cmd, char** args, int argc);


/**
 * Function ft2232_stableclocks
 * will send out \a num_cycles on the TCK line while the TAP(s)
 * are in a stable state.  Calling code must ensure that current state is
 * stable, that verification is not done in here.
 * @param num_cycles is the count of clocks cycles to send.
 * @return int - ERROR_OK or ERROR_JTAG_QUEUE_FAILED
 */
static int ft2232_stableclocks(int num_cycles, jtag_command_t* cmd);


char *        ft2232_device_desc_A = NULL;
char*         ft2232_device_desc = NULL;
char*         ft2232_serial  = NULL;
char*         ft2232_layout  = NULL;
unsigned char ft2232_latency = 2;

#define MAX_USB_IDS 8
/* vid = pid = 0 marks the end of the list */
static u16 ft2232_vid[MAX_USB_IDS + 1] = { 0x0403, 0 };
static u16 ft2232_pid[MAX_USB_IDS + 1] = { 0x6010, 0 };

typedef struct ft2232_layout_s
{
	char* name;
	int (*init)(void);
	void (*reset)(int trst, int srst);
	void (*blink)(void);
} ft2232_layout_t;

/* init procedures for supported layouts */
int  usbjtag_init(void);
int  jtagkey_init(void);
int  olimex_jtag_init(void);
int  flyswatter_init(void);
int  turtle_init(void);
int  comstick_init(void);
int  stm32stick_init(void);
int  axm0432_jtag_init(void);
int sheevaplug_init(void);

/* reset procedures for supported layouts */
void usbjtag_reset(int trst, int srst);
void jtagkey_reset(int trst, int srst);
void olimex_jtag_reset(int trst, int srst);
void flyswatter_reset(int trst, int srst);
void turtle_reset(int trst, int srst);
void comstick_reset(int trst, int srst);
void stm32stick_reset(int trst, int srst);
void axm0432_jtag_reset(int trst, int srst);
void sheevaplug_reset(int trst, int srst);

/* blink procedures for layouts that support a blinking led */
void olimex_jtag_blink(void);
void flyswatter_jtag_blink(void);
void turtle_jtag_blink(void);

ft2232_layout_t            ft2232_layouts[] =
{
	{ "usbjtag",              usbjtag_init,              usbjtag_reset,      NULL                    },
	{ "jtagkey",              jtagkey_init,              jtagkey_reset,      NULL                    },
	{ "jtagkey_prototype_v1", jtagkey_init,              jtagkey_reset,      NULL                    },
	{ "oocdlink",             jtagkey_init,              jtagkey_reset,      NULL                    },
	{ "signalyzer",           usbjtag_init,              usbjtag_reset,      NULL                    },
	{ "evb_lm3s811",          usbjtag_init,              usbjtag_reset,      NULL                    },
	{ "olimex-jtag",          olimex_jtag_init,          olimex_jtag_reset,  olimex_jtag_blink       },
	{ "flyswatter",           flyswatter_init,           flyswatter_reset,   flyswatter_jtag_blink   },
	{ "turtelizer2",          turtle_init,               turtle_reset,       turtle_jtag_blink       },
	{ "comstick",             comstick_init,             comstick_reset,     NULL                    },
	{ "stm32stick",           stm32stick_init,           stm32stick_reset,   NULL                    },
	{ "axm0432_jtag",         axm0432_jtag_init,         axm0432_jtag_reset, NULL                    },
	{"sheevaplug",            sheevaplug_init,           sheevaplug_reset,   NULL                    },
	{ NULL,                   NULL,                      NULL },
};

static u8                  nTRST, nTRSTnOE, nSRST, nSRSTnOE;

static ft2232_layout_t*    layout;
static u8                  low_output     = 0x0;
static u8                  low_direction  = 0x0;
static u8                  high_output    = 0x0;
static u8                  high_direction = 0x0;

#if BUILD_FT2232_FTD2XX == 1
static FT_HANDLE           ftdih = NULL;
#elif BUILD_FT2232_LIBFTDI == 1
static struct ftdi_context ftdic;
#endif


static jtag_command_t* first_unsent;        /* next command that has to be sent */
static int             require_send;

static u8*             ft2232_buffer = NULL;
static int             ft2232_buffer_size  = 0;
static int             ft2232_read_pointer = 0;
static int             ft2232_expect_read  = 0;

#define FT2232_BUFFER_SIZE 131072
#define BUFFER_ADD         ft2232_buffer[ft2232_buffer_size++]
#define BUFFER_READ        ft2232_buffer[ft2232_read_pointer++]

jtag_interface_t ft2232_interface =
{
	.name               = "ft2232",
	.execute_queue = ft2232_execute_queue,
	.speed     = ft2232_speed,
	.speed_div = ft2232_speed_div,
	.khz                = ft2232_khz,
	.register_commands  = ft2232_register_commands,
	.init = ft2232_init,
	.quit = ft2232_quit,
};

int ft2232_write(u8* buf, int size, u32* bytes_written)
{
#if BUILD_FT2232_FTD2XX == 1
	FT_STATUS status;
	DWORD     dw_bytes_written;
	if ( ( status = FT_Write(ftdih, buf, size, &dw_bytes_written) ) != FT_OK )
	{
		*bytes_written = dw_bytes_written;
		LOG_ERROR("FT_Write returned: %lu", status);
		return ERROR_JTAG_DEVICE_ERROR;
	}
	else
	{
		*bytes_written = dw_bytes_written;
		return ERROR_OK;
	}
#elif BUILD_FT2232_LIBFTDI == 1
	int retval;
	if ( ( retval = ftdi_write_data(&ftdic, buf, size) ) < 0 )
	{
		*bytes_written = 0;
		LOG_ERROR( "ftdi_write_data: %s", ftdi_get_error_string(&ftdic) );
		return ERROR_JTAG_DEVICE_ERROR;
	}
	else
	{
		*bytes_written = retval;
		return ERROR_OK;
	}
#endif
}


int ft2232_read(u8* buf, int size, u32* bytes_read)
{
#if BUILD_FT2232_FTD2XX == 1
	DWORD     dw_bytes_read;
	FT_STATUS status;
	int       timeout = 5;
	*bytes_read = 0;

	while ( (*bytes_read < size) && timeout-- )
	{
		if ( ( status = FT_Read(ftdih, buf + *bytes_read, size -
					  *bytes_read, &dw_bytes_read) ) != FT_OK )
		{
			*bytes_read = 0;
			LOG_ERROR("FT_Read returned: %lu", status);
			return ERROR_JTAG_DEVICE_ERROR;
		}
		*bytes_read += dw_bytes_read;
	}

#elif BUILD_FT2232_LIBFTDI == 1
	int retval;
	int timeout = 100;
	*bytes_read = 0;

	while ( (*bytes_read < size) && timeout-- )
	{
		if ( ( retval = ftdi_read_data(&ftdic, buf + *bytes_read, size - *bytes_read) ) < 0 )
		{
			*bytes_read = 0;
			LOG_ERROR( "ftdi_read_data: %s", ftdi_get_error_string(&ftdic) );
			return ERROR_JTAG_DEVICE_ERROR;
		}
		*bytes_read += retval;
	}

#endif

	if (*bytes_read < size)
	{
		LOG_ERROR("couldn't read the requested number of bytes from FT2232 device (%i < %i)", *bytes_read, size);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}


int ft2232_speed(int speed)
{
	u8  buf[3];
	int retval;
	u32 bytes_written;

	buf[0] = 0x86;                  /* command "set divisor" */
	buf[1] = speed & 0xff;          /* valueL (0=6MHz, 1=3MHz, 2=2.0MHz, ...*/
	buf[2] = (speed >> 8) & 0xff;   /* valueH */

	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);
	if ( ( ( retval = ft2232_write(buf, 3, &bytes_written) ) != ERROR_OK ) || (bytes_written != 3) )
	{
		LOG_ERROR("couldn't set FT2232 TCK speed");
		return retval;
	}

	return ERROR_OK;
}


int ft2232_speed_div(int speed, int* khz)
{
	/* Take a look in the FT2232 manual,
	 * AN2232C-01 Command Processor for
	 * MPSSE and MCU Host Bus. Chapter 3.8 */

	*khz = 6000 / (1 + speed);

	return ERROR_OK;
}


int ft2232_khz(int khz, int* jtag_speed)
{
	if (khz==0)
	{
		LOG_ERROR("RCLK not supported");
		return ERROR_FAIL;
	}

	/* Take a look in the FT2232 manual,
	 * AN2232C-01 Command Processor for
	 * MPSSE and MCU Host Bus. Chapter 3.8
	 *
	 * We will calc here with a multiplier
	 * of 10 for better rounding later. */

	/* Calc speed, (6000 / khz) - 1 */
	/* Use 65000 for better rounding */
	*jtag_speed = (60000 / khz) - 10;

	/* Add 0.9 for rounding */
	*jtag_speed += 9;

	/* Calc real speed */
	*jtag_speed = *jtag_speed / 10;

	/* Check if speed is greater than 0 */
	if (*jtag_speed < 0)
	{
		*jtag_speed = 0;
	}

	/* Check max value */
	if (*jtag_speed > 0xFFFF)
	{
		*jtag_speed = 0xFFFF;
	}

	return ERROR_OK;
}


int ft2232_register_commands(struct command_context_s* cmd_ctx)
{
	register_command(cmd_ctx, NULL, "ft2232_device_desc", ft2232_handle_device_desc_command,
			COMMAND_CONFIG, "the USB device description of the FTDI FT2232 device");
	register_command(cmd_ctx, NULL, "ft2232_serial", ft2232_handle_serial_command,
			COMMAND_CONFIG, "the serial number of the FTDI FT2232 device");
	register_command(cmd_ctx, NULL, "ft2232_layout", ft2232_handle_layout_command,
			COMMAND_CONFIG, "the layout of the FT2232 GPIO signals used to control output-enables and reset signals");
	register_command(cmd_ctx, NULL, "ft2232_vid_pid", ft2232_handle_vid_pid_command,
			COMMAND_CONFIG, "the vendor ID and product ID of the FTDI FT2232 device");
	register_command(cmd_ctx, NULL, "ft2232_latency", ft2232_handle_latency_command,
			COMMAND_CONFIG, "set the FT2232 latency timer to a new value");
	return ERROR_OK;
}


void ft2232_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else
	{
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}


void ft2232_read_scan(enum scan_type type, u8* buffer, int scan_size)
{
	int num_bytes = (scan_size + 7) / 8;
	int bits_left = scan_size;
	int cur_byte  = 0;

	while (num_bytes-- > 1)
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

	buffer[cur_byte] = ( buffer[cur_byte] | ( (BUFFER_READ & 0x02) << 6 ) ) >> (8 - bits_left);
}


void ft2232_debug_dump_buffer(void)
{
	int   i;
	char  line[256];
	char* line_p = line;

	for (i = 0; i < ft2232_buffer_size; i++)
	{
		line_p += snprintf(line_p, 256 - (line_p - line), "%2.2x ", ft2232_buffer[i]);
		if (i % 16 == 15)
		{
			LOG_DEBUG("%s", line);
			line_p = line;
		}
	}

	if (line_p != line)
		LOG_DEBUG("%s", line);
}


int ft2232_send_and_recv(jtag_command_t* first, jtag_command_t* last)
{
	jtag_command_t* cmd;
	u8*             buffer;
	int             scan_size;
	enum scan_type  type;
	int             retval;
	u32             bytes_written;
	u32             bytes_read;

#ifdef _DEBUG_USB_IO_
	struct timeval  start, inter, inter2, end;
	struct timeval  d_inter, d_inter2, d_end;
#endif

#ifdef _DEBUG_USB_COMMS_
	LOG_DEBUG("write buffer (size %i):", ft2232_buffer_size);
	ft2232_debug_dump_buffer();
#endif

#ifdef _DEBUG_USB_IO_
	gettimeofday(&start, NULL);
#endif

	if ( ( retval = ft2232_write(ft2232_buffer, ft2232_buffer_size, &bytes_written) ) != ERROR_OK )
	{
		LOG_ERROR("couldn't write MPSSE commands to FT2232");
		return retval;
	}

#ifdef _DEBUG_USB_IO_
	gettimeofday(&inter, NULL);
#endif

	if (ft2232_expect_read)
	{
		int timeout = 100;
		ft2232_buffer_size = 0;

#ifdef _DEBUG_USB_IO_
		gettimeofday(&inter2, NULL);
#endif

		if ( ( retval = ft2232_read(ft2232_buffer, ft2232_expect_read, &bytes_read) ) != ERROR_OK )
		{
			LOG_ERROR("couldn't read from FT2232");
			return retval;
		}

#ifdef _DEBUG_USB_IO_
		gettimeofday(&end, NULL);

		timeval_subtract(&d_inter, &inter, &start);
		timeval_subtract(&d_inter2, &inter2, &start);
		timeval_subtract(&d_end, &end, &start);

		LOG_INFO("inter: %i.%06i, inter2: %i.%06i end: %i.%06i", d_inter.tv_sec, d_inter.tv_usec, d_inter2.tv_sec,
				d_inter2.tv_usec, d_end.tv_sec,
				d_end.tv_usec);
#endif

		ft2232_buffer_size = bytes_read;

		if (ft2232_expect_read != ft2232_buffer_size)
		{
			LOG_ERROR("ft2232_expect_read (%i) != ft2232_buffer_size (%i) (%i retries)", ft2232_expect_read,
					ft2232_buffer_size,
					100 - timeout);
			ft2232_debug_dump_buffer();

			exit(-1);
		}

#ifdef _DEBUG_USB_COMMS_
		LOG_DEBUG("read buffer (%i retries): %i bytes", 100 - timeout, ft2232_buffer_size);
		ft2232_debug_dump_buffer();
#endif
	}

	ft2232_expect_read  = 0;
	ft2232_read_pointer = 0;

	/* return ERROR_OK, unless a jtag_read_buffer returns a failed check
	 * that wasn't handled by a caller-provided error handler
	 */
	retval = ERROR_OK;

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
				buffer    = calloc(CEIL(scan_size, 8), 1);
				ft2232_read_scan(type, buffer, scan_size);
				if (jtag_read_buffer(buffer, cmd->cmd.scan) != ERROR_OK)
					retval = ERROR_JTAG_QUEUE_FAILED;
				free(buffer);
			}
			break;

		default:
			break;
		}

		cmd = cmd->next;
	}

	ft2232_buffer_size = 0;

	return retval;
}


void ft2232_add_pathmove(pathmove_command_t* cmd)
{
	int num_states = cmd->num_states;
	int state_count = 0;

	while (num_states)
	{
		u8  tms_byte = 0;       /* zero this on each MPSSE batch */

		int bit_count = 0;

		int num_states_batch = num_states > 7 ? 7 : num_states;

		/* command "Clock Data to TMS/CS Pin (no Read)" */
		BUFFER_ADD = 0x4b;

		/* number of states remaining */
		BUFFER_ADD = num_states_batch - 1;

		while (num_states_batch--)
		{
			if (tap_state_transition(tap_get_state(), false) == cmd->path[state_count])
				buf_set_u32(&tms_byte, bit_count++, 1, 0x0);
			else if (tap_state_transition(tap_get_state(), true) == cmd->path[state_count])
				buf_set_u32(&tms_byte, bit_count++, 1, 0x1);
			else
			{
				LOG_ERROR( "BUG: %s -> %s isn't a valid TAP transition", tap_state_name(
								 tap_get_state() ), tap_state_name(cmd->path[state_count]) );
				exit(-1);
			}

			tap_set_state(cmd->path[state_count]);
			state_count++;
			num_states--;
		}

		BUFFER_ADD = tms_byte;
	}
	
	tap_set_end_state(tap_get_state());
}


void ft2232_add_scan(int ir_scan, enum scan_type type, u8* buffer, int scan_size)
{
	int num_bytes = (scan_size + 7) / 8;
	int bits_left = scan_size;
	int cur_byte  = 0;
	int last_bit;

	if ( !( ( !ir_scan && (tap_get_state() == TAP_DRSHIFT) )
	   || (    ir_scan && (tap_get_state() == TAP_IRSHIFT) ) ) )
	{
		/* command "Clock Data to TMS/CS Pin (no Read)" */
		BUFFER_ADD = 0x4b;

		BUFFER_ADD = 0x6;       /* scan 7 bits */

		/* TMS data bits */
		if (ir_scan)
		{
			BUFFER_ADD = tap_get_tms_path(tap_get_state(), TAP_IRSHIFT);
			tap_set_state(TAP_IRSHIFT);
		}
		else
		{
			BUFFER_ADD = tap_get_tms_path(tap_get_state(), TAP_DRSHIFT);
			tap_set_state(TAP_DRSHIFT);
		}
		/* LOG_DEBUG("added TMS scan (no read)"); */
	}

	/* add command for complete bytes */
	while (num_bytes > 1)
	{
		int thisrun_bytes;
		if (type == SCAN_IO)
		{
			/* Clock Data Bytes In and Out LSB First */
			BUFFER_ADD = 0x39;
			/* LOG_DEBUG("added TDI bytes (io %i)", num_bytes); */
		}
		else if (type == SCAN_OUT)
		{
			/* Clock Data Bytes Out on -ve Clock Edge LSB First (no Read) */
			BUFFER_ADD = 0x19;
			/* LOG_DEBUG("added TDI bytes (o)"); */
		}
		else if (type == SCAN_IN)
		{
			/* Clock Data Bytes In on +ve Clock Edge LSB First (no Write) */
			BUFFER_ADD = 0x28;
			/* LOG_DEBUG("added TDI bytes (i %i)", num_bytes); */
		}

		thisrun_bytes = (num_bytes > 65537) ? 65536 : (num_bytes - 1);
		num_bytes    -= thisrun_bytes;
		BUFFER_ADD    = (thisrun_bytes - 1) & 0xff;
		BUFFER_ADD    = ( (thisrun_bytes - 1) >> 8 ) & 0xff;

		if (type != SCAN_IN)
		{
			/* add complete bytes */
			while (thisrun_bytes-- > 0)
			{
				BUFFER_ADD = buffer[cur_byte];
				cur_byte++;
				bits_left -= 8;
			}
		}
		else /* (type == SCAN_IN) */
		{
			bits_left -= 8 * (thisrun_bytes);
		}
	}

	/* the most signifcant bit is scanned during TAP movement */
	if (type != SCAN_IN)
		last_bit = ( buffer[cur_byte] >> (bits_left - 1) ) & 0x1;
	else
		last_bit = 0;

	/* process remaining bits but the last one */
	if (bits_left > 1)
	{
		if (type == SCAN_IO)
		{
			/* Clock Data Bits In and Out LSB First */
			BUFFER_ADD = 0x3b;
			/* LOG_DEBUG("added TDI bits (io) %i", bits_left - 1); */
		}
		else if (type == SCAN_OUT)
		{
			/* Clock Data Bits Out on -ve Clock Edge LSB First (no Read) */
			BUFFER_ADD = 0x1b;
			/* LOG_DEBUG("added TDI bits (o)"); */
		}
		else if (type == SCAN_IN)
		{
			/* Clock Data Bits In on +ve Clock Edge LSB First (no Write) */
			BUFFER_ADD = 0x2a;
			/* LOG_DEBUG("added TDI bits (i %i)", bits_left - 1); */
		}
		BUFFER_ADD = bits_left - 2;
		if (type != SCAN_IN)
			BUFFER_ADD = buffer[cur_byte];
	}

	if ( (  ir_scan && (tap_get_end_state() == TAP_IRSHIFT) )
	  || ( !ir_scan && (tap_get_end_state() == TAP_DRSHIFT) ) )
	{
		if (type == SCAN_IO)
		{
			/* Clock Data Bits In and Out LSB First */
			BUFFER_ADD = 0x3b;
			/* LOG_DEBUG("added TDI bits (io) %i", bits_left - 1); */
		}
		else if (type == SCAN_OUT)
		{
			/* Clock Data Bits Out on -ve Clock Edge LSB First (no Read) */
			BUFFER_ADD = 0x1b;
			/* LOG_DEBUG("added TDI bits (o)"); */
		}
		else if (type == SCAN_IN)
		{
			/* Clock Data Bits In on +ve Clock Edge LSB First (no Write) */
			BUFFER_ADD = 0x2a;
			/* LOG_DEBUG("added TDI bits (i %i)", bits_left - 1); */
		}
		BUFFER_ADD = 0x0;
		BUFFER_ADD = last_bit;
	}
	else
	{
		/* move from Shift-IR/DR to end state */
		if (type != SCAN_OUT)
		{
			/* Clock Data to TMS/CS Pin with Read */
			BUFFER_ADD = 0x6b;
			/* LOG_DEBUG("added TMS scan (read)"); */
		}
		else
		{
			/* Clock Data to TMS/CS Pin (no Read) */
			BUFFER_ADD = 0x4b;
			/* LOG_DEBUG("added TMS scan (no read)"); */
		}
		BUFFER_ADD = 0x6;   /* scan 7 bits */

		BUFFER_ADD = tap_get_tms_path( tap_get_state(), tap_get_end_state() ) | (last_bit << 7);
		tap_set_state( tap_get_end_state() );
	}
}


int ft2232_large_scan(scan_command_t* cmd, enum scan_type type, u8* buffer, int scan_size)
{
	int num_bytes = (scan_size + 7) / 8;
	int bits_left = scan_size;
	int cur_byte  = 0;
	int last_bit;
	u8* receive_buffer  = malloc( CEIL(scan_size, 8) );
	u8* receive_pointer = receive_buffer;
	u32 bytes_written;
	u32 bytes_read;
	int retval;
	int thisrun_read = 0;

	if (cmd->ir_scan)
	{
		LOG_ERROR("BUG: large IR scans are not supported");
		exit(-1);
	}

	if (tap_get_state() != TAP_DRSHIFT)
	{
		/* command "Clock Data to TMS/CS Pin (no Read)" */
		BUFFER_ADD = 0x4b;

		BUFFER_ADD = 0x6;       /* scan 7 bits */

		/* TMS data bits */
		BUFFER_ADD = tap_get_tms_path(tap_get_state(), TAP_DRSHIFT);
		tap_set_state(TAP_DRSHIFT);
	}

	if ( ( retval = ft2232_write(ft2232_buffer, ft2232_buffer_size, &bytes_written) ) != ERROR_OK )
	{
		LOG_ERROR("couldn't write MPSSE commands to FT2232");
		exit(-1);
	}
	LOG_DEBUG("ft2232_buffer_size: %i, bytes_written: %i", ft2232_buffer_size, bytes_written);
	ft2232_buffer_size = 0;

	/* add command for complete bytes */
	while (num_bytes > 1)
	{
		int thisrun_bytes;

		if (type == SCAN_IO)
		{
			/* Clock Data Bytes In and Out LSB First */
			BUFFER_ADD = 0x39;
			/* LOG_DEBUG("added TDI bytes (io %i)", num_bytes); */
		}
		else if (type == SCAN_OUT)
		{
			/* Clock Data Bytes Out on -ve Clock Edge LSB First (no Read) */
			BUFFER_ADD = 0x19;
			/* LOG_DEBUG("added TDI bytes (o)"); */
		}
		else if (type == SCAN_IN)
		{
			/* Clock Data Bytes In on +ve Clock Edge LSB First (no Write) */
			BUFFER_ADD = 0x28;
			/* LOG_DEBUG("added TDI bytes (i %i)", num_bytes); */
		}

		thisrun_bytes = (num_bytes > 65537) ? 65536 : (num_bytes - 1);
		thisrun_read  = thisrun_bytes;
		num_bytes    -= thisrun_bytes;
		BUFFER_ADD    = (thisrun_bytes - 1) & 0xff;
		BUFFER_ADD    = ( (thisrun_bytes - 1) >> 8 ) & 0xff;

		if (type != SCAN_IN)
		{
			/* add complete bytes */
			while (thisrun_bytes-- > 0)
			{
				BUFFER_ADD = buffer[cur_byte];
				cur_byte++;
				bits_left -= 8;
			}
		}
		else /* (type == SCAN_IN) */
		{
			bits_left -= 8 * (thisrun_bytes);
		}

		if ( ( retval = ft2232_write(ft2232_buffer, ft2232_buffer_size, &bytes_written) ) != ERROR_OK )
		{
			LOG_ERROR("couldn't write MPSSE commands to FT2232");
			exit(-1);
		}
		LOG_DEBUG("ft2232_buffer_size: %i, bytes_written: %i", ft2232_buffer_size, bytes_written);
		ft2232_buffer_size = 0;

		if (type != SCAN_OUT)
		{
			if ( ( retval = ft2232_read(receive_pointer, thisrun_read, &bytes_read) ) != ERROR_OK )
			{
				LOG_ERROR("couldn't read from FT2232");
				exit(-1);
			}
			LOG_DEBUG("thisrun_read: %i, bytes_read: %i", thisrun_read, bytes_read);
			receive_pointer += bytes_read;
		}
	}

	thisrun_read = 0;

	/* the most signifcant bit is scanned during TAP movement */
	if (type != SCAN_IN)
		last_bit = ( buffer[cur_byte] >> (bits_left - 1) ) & 0x1;
	else
		last_bit = 0;

	/* process remaining bits but the last one */
	if (bits_left > 1)
	{
		if (type == SCAN_IO)
		{
			/* Clock Data Bits In and Out LSB First */
			BUFFER_ADD = 0x3b;
			/* LOG_DEBUG("added TDI bits (io) %i", bits_left - 1); */
		}
		else if (type == SCAN_OUT)
		{
			/* Clock Data Bits Out on -ve Clock Edge LSB First (no Read) */
			BUFFER_ADD = 0x1b;
			/* LOG_DEBUG("added TDI bits (o)"); */
		}
		else if (type == SCAN_IN)
		{
			/* Clock Data Bits In on +ve Clock Edge LSB First (no Write) */
			BUFFER_ADD = 0x2a;
			/* LOG_DEBUG("added TDI bits (i %i)", bits_left - 1); */
		}
		BUFFER_ADD = bits_left - 2;
		if (type != SCAN_IN)
			BUFFER_ADD = buffer[cur_byte];

		if (type != SCAN_OUT)
			thisrun_read += 2;
	}

	if (tap_get_end_state() == TAP_DRSHIFT)
	{
		if (type == SCAN_IO)
		{
			/* Clock Data Bits In and Out LSB First */
			BUFFER_ADD = 0x3b;
			/* LOG_DEBUG("added TDI bits (io) %i", bits_left - 1); */
		}
		else if (type == SCAN_OUT)
		{
			/* Clock Data Bits Out on -ve Clock Edge LSB First (no Read) */
			BUFFER_ADD = 0x1b;
			/* LOG_DEBUG("added TDI bits (o)"); */
		}
		else if (type == SCAN_IN)
		{
			/* Clock Data Bits In on +ve Clock Edge LSB First (no Write) */
			BUFFER_ADD = 0x2a;
			/* LOG_DEBUG("added TDI bits (i %i)", bits_left - 1); */
		}
		BUFFER_ADD = 0x0;
		BUFFER_ADD = last_bit;
	}
	else
	{
		/* move from Shift-IR/DR to end state */
		if (type != SCAN_OUT)
		{
			/* Clock Data to TMS/CS Pin with Read */
			BUFFER_ADD = 0x6b;
			/* LOG_DEBUG("added TMS scan (read)"); */
		}
		else
		{
			/* Clock Data to TMS/CS Pin (no Read) */
			BUFFER_ADD = 0x4b;
			/* LOG_DEBUG("added TMS scan (no read)"); */
		}
		BUFFER_ADD = 0x6;
		BUFFER_ADD = tap_get_tms_path( tap_get_state(), tap_get_end_state() ) | (last_bit << 7);
		tap_set_state( tap_get_end_state() );
	}

	if (type != SCAN_OUT)
		thisrun_read += 1;

	if ( ( retval = ft2232_write(ft2232_buffer, ft2232_buffer_size, &bytes_written) ) != ERROR_OK )
	{
		LOG_ERROR("couldn't write MPSSE commands to FT2232");
		exit(-1);
	}
	LOG_DEBUG("ft2232_buffer_size: %i, bytes_written: %i", ft2232_buffer_size, bytes_written);
	ft2232_buffer_size = 0;

	if (type != SCAN_OUT)
	{
		if ( ( retval = ft2232_read(receive_pointer, thisrun_read, &bytes_read) ) != ERROR_OK )
		{
			LOG_ERROR("couldn't read from FT2232");
			exit(-1);
		}
		LOG_DEBUG("thisrun_read: %i, bytes_read: %i", thisrun_read, bytes_read);
		receive_pointer += bytes_read;
	}

	return ERROR_OK;
}


int ft2232_predict_scan_out(int scan_size, enum scan_type type)
{
	int predicted_size = 3;
	int num_bytes = (scan_size - 1) / 8;

	if (tap_get_state() != TAP_DRSHIFT)
		predicted_size += 3;

	if (type == SCAN_IN)    /* only from device to host */
	{
		/* complete bytes */
		predicted_size += CEIL(num_bytes, 65536) * 3;
		/* remaining bits - 1 (up to 7) */
		predicted_size += ( (scan_size - 1) % 8 ) ? 2 : 0;
	}
	else                    /* host to device, or bidirectional */
	{
		/* complete bytes */
		predicted_size += num_bytes + CEIL(num_bytes, 65536) * 3;
		/* remaining bits -1 (up to 7) */
		predicted_size += ( (scan_size - 1) % 8 ) ? 3 : 0;
	}

	return predicted_size;
}


int ft2232_predict_scan_in(int scan_size, enum scan_type type)
{
	int predicted_size = 0;

	if (type != SCAN_OUT)
	{
		/* complete bytes */
		predicted_size += (CEIL(scan_size, 8) > 1) ? (CEIL(scan_size, 8) - 1) : 0;

		/* remaining bits - 1 */
		predicted_size += ( (scan_size - 1) % 8 ) ? 1 : 0;

		/* last bit (from TMS scan) */
		predicted_size += 1;
	}

	/* LOG_DEBUG("scan_size: %i, predicted_size: %i", scan_size, predicted_size); */

	return predicted_size;
}


void usbjtag_reset(int trst, int srst)
{
	if (trst == 1)
	{
		if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
			low_direction |= nTRSTnOE;  /* switch to output pin (output is low) */
		else
			low_output &= ~nTRST;       /* switch output low */
	}
	else if (trst == 0)
	{
		if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
			low_direction &= ~nTRSTnOE; /* switch to input pin (high-Z + internal and external pullup) */
		else
			low_output |= nTRST;        /* switch output high */
	}

	if (srst == 1)
	{
		if (jtag_reset_config & RESET_SRST_PUSH_PULL)
			low_output &= ~nSRST;       /* switch output low */
		else
			low_direction |= nSRSTnOE;  /* switch to output pin (output is low) */
	}
	else if (srst == 0)
	{
		if (jtag_reset_config & RESET_SRST_PUSH_PULL)
			low_output |= nSRST;        /* switch output high */
		else
			low_direction &= ~nSRSTnOE; /* switch to input pin (high-Z) */
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
	LOG_DEBUG("trst: %i, srst: %i, high_output: 0x%2.2x, high_direction: 0x%2.2x", trst, srst, high_output,
			high_direction);
}


void olimex_jtag_reset(int trst, int srst)
{
	if (trst == 1)
	{
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
		high_output |= nSRST;
	}
	else if (srst == 0)
	{
		high_output &= ~nSRST;
	}

	/* command "set data bits high byte" */
	BUFFER_ADD = 0x82;
	BUFFER_ADD = high_output;
	BUFFER_ADD = high_direction;
	LOG_DEBUG("trst: %i, srst: %i, high_output: 0x%2.2x, high_direction: 0x%2.2x", trst, srst, high_output,
			high_direction);
}


void axm0432_jtag_reset(int trst, int srst)
{
	if (trst == 1)
	{
		tap_set_state(TAP_RESET);
		high_output &= ~nTRST;
	}
	else if (trst == 0)
	{
		high_output |= nTRST;
	}

	if (srst == 1)
	{
		high_output &= ~nSRST;
	}
	else if (srst == 0)
	{
		high_output |= nSRST;
	}

	/* command "set data bits low byte" */
	BUFFER_ADD = 0x82;
	BUFFER_ADD = high_output;
	BUFFER_ADD = high_direction;
	LOG_DEBUG("trst: %i, srst: %i, high_output: 0x%2.2x, high_direction: 0x%2.2x", trst, srst, high_output,
			high_direction);
}


void flyswatter_reset(int trst, int srst)
{
	if (trst == 1)
	{
		low_output &= ~nTRST;
	}
	else if (trst == 0)
	{
		low_output |= nTRST;
	}

	if (srst == 1)
	{
		low_output |= nSRST;
	}
	else if (srst == 0)
	{
		low_output &= ~nSRST;
	}

	/* command "set data bits low byte" */
	BUFFER_ADD = 0x80;
	BUFFER_ADD = low_output;
	BUFFER_ADD = low_direction;
	LOG_DEBUG("trst: %i, srst: %i, low_output: 0x%2.2x, low_direction: 0x%2.2x", trst, srst, low_output, low_direction);
}


void turtle_reset(int trst, int srst)
{
	trst = trst;

	if (srst == 1)
	{
		low_output |= nSRST;
	}
	else if (srst == 0)
	{
		low_output &= ~nSRST;
	}

	/* command "set data bits low byte" */
	BUFFER_ADD = 0x80;
	BUFFER_ADD = low_output;
	BUFFER_ADD = low_direction;
	LOG_DEBUG("srst: %i, low_output: 0x%2.2x, low_direction: 0x%2.2x", srst, low_output, low_direction);
}


void comstick_reset(int trst, int srst)
{
	if (trst == 1)
	{
		high_output &= ~nTRST;
	}
	else if (trst == 0)
	{
		high_output |= nTRST;
	}

	if (srst == 1)
	{
		high_output &= ~nSRST;
	}
	else if (srst == 0)
	{
		high_output |= nSRST;
	}

	/* command "set data bits high byte" */
	BUFFER_ADD = 0x82;
	BUFFER_ADD = high_output;
	BUFFER_ADD = high_direction;
	LOG_DEBUG("trst: %i, srst: %i, high_output: 0x%2.2x, high_direction: 0x%2.2x", trst, srst, high_output,
			high_direction);
}


void stm32stick_reset(int trst, int srst)
{
	if (trst == 1)
	{
		high_output &= ~nTRST;
	}
	else if (trst == 0)
	{
		high_output |= nTRST;
	}

	if (srst == 1)
	{
		low_output &= ~nSRST;
	}
	else if (srst == 0)
	{
		low_output |= nSRST;
	}

	/* command "set data bits low byte" */
	BUFFER_ADD = 0x80;
	BUFFER_ADD = low_output;
	BUFFER_ADD = low_direction;

	/* command "set data bits high byte" */
	BUFFER_ADD = 0x82;
	BUFFER_ADD = high_output;
	BUFFER_ADD = high_direction;
	LOG_DEBUG("trst: %i, srst: %i, high_output: 0x%2.2x, high_direction: 0x%2.2x", trst, srst, high_output,
			high_direction);
}



void sheevaplug_reset(int trst, int srst)
{
	if (trst == 1)
		high_output &= ~nTRST;
	else if (trst == 0)
		high_output |= nTRST;

	if (srst == 1)
		high_output &= ~nSRSTnOE;
	else if (srst == 0)
		high_output |= nSRSTnOE;

	/* command "set data bits high byte" */
	BUFFER_ADD = 0x82;
	BUFFER_ADD = high_output;
	BUFFER_ADD = high_direction;
	LOG_DEBUG("trst: %i, srst: %i, high_output: 0x%2.2x, high_direction: 0x%2.2x", trst, srst, high_output, high_direction);
}

int ft2232_execute_queue()
{
	jtag_command_t* cmd = jtag_command_queue;   /* currently processed command */
	u8*             buffer;
	int             scan_size;                  /* size of IR or DR scan */
	enum scan_type  type;
	int             i;
	int             predicted_size = 0;
	int             retval;

	first_unsent = cmd;         /* next command that has to be sent */
	require_send = 0;

	/* return ERROR_OK, unless ft2232_send_and_recv reports a failed check
	 * that wasn't handled by a caller-provided error handler
	 */
	retval = ERROR_OK;

	ft2232_buffer_size = 0;
	ft2232_expect_read = 0;

	/* blink, if the current layout has that feature */
	if (layout->blink)
		layout->blink();

	while (cmd)
	{
		switch (cmd->type)
		{
		case JTAG_END_STATE:
			if (cmd->cmd.end_state->end_state != -1)
				ft2232_end_state(cmd->cmd.end_state->end_state);
			break;

		case JTAG_RESET:
			/* only send the maximum buffer size that FT2232C can handle */
			predicted_size = 3;
			if (ft2232_buffer_size + predicted_size + 1 > FT2232_BUFFER_SIZE)
			{
				if (ft2232_send_and_recv(first_unsent, cmd) != ERROR_OK)
					retval = ERROR_JTAG_QUEUE_FAILED;
				require_send = 0;
				first_unsent = cmd;
			}

			if ( (cmd->cmd.reset->trst == 1) || ( cmd->cmd.reset->srst && (jtag_reset_config & RESET_SRST_PULLS_TRST) ) )
			{
				tap_set_state(TAP_RESET);
			}
			layout->reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
			require_send = 1;

#ifdef _DEBUG_JTAG_IO_
			LOG_DEBUG("trst: %i, srst: %i", cmd->cmd.reset->trst, cmd->cmd.reset->srst);
#endif
			break;

		case JTAG_RUNTEST:
			/* only send the maximum buffer size that FT2232C can handle */
			predicted_size = 0;
			if (tap_get_state() != TAP_IDLE)
				predicted_size += 3;
			predicted_size += 3 * CEIL(cmd->cmd.runtest->num_cycles, 7);
			if ( (cmd->cmd.runtest->end_state != -1) && (cmd->cmd.runtest->end_state != TAP_IDLE) )
				predicted_size += 3;
			if ( (cmd->cmd.runtest->end_state == -1) && (tap_get_end_state() != TAP_IDLE) )
				predicted_size += 3;
			if (ft2232_buffer_size + predicted_size + 1 > FT2232_BUFFER_SIZE)
			{
				if (ft2232_send_and_recv(first_unsent, cmd) != ERROR_OK)
					retval = ERROR_JTAG_QUEUE_FAILED;
				require_send = 0;
				first_unsent = cmd;
			}
			if (tap_get_state() != TAP_IDLE)
			{
				/* command "Clock Data to TMS/CS Pin (no Read)" */
				BUFFER_ADD = 0x4b;
				BUFFER_ADD = 0x6;    /* scan 7 bits */

				/* TMS data bits */
				BUFFER_ADD = tap_get_tms_path(tap_get_state(), TAP_IDLE);
				tap_set_state(TAP_IDLE);
				require_send = 1;
			}
			i = cmd->cmd.runtest->num_cycles;
			while (i > 0)
			{
				/* command "Clock Data to TMS/CS Pin (no Read)" */
				BUFFER_ADD = 0x4b;

				/* scan 7 bits */
				BUFFER_ADD = (i > 7) ? 6 : (i - 1);

				/* TMS data bits */
				BUFFER_ADD = 0x0;
				tap_set_state(TAP_IDLE);
				i -= (i > 7) ? 7 : i;
				/* LOG_DEBUG("added TMS scan (no read)"); */
			}

			if (cmd->cmd.runtest->end_state != -1)
				ft2232_end_state(cmd->cmd.runtest->end_state);

			if ( tap_get_state() != tap_get_end_state() )
			{
				/* command "Clock Data to TMS/CS Pin (no Read)" */
				BUFFER_ADD = 0x4b;
				/* scan 7 bit */
				BUFFER_ADD = 0x6;
				/* TMS data bits */
				BUFFER_ADD = tap_get_tms_path( tap_get_state(), tap_get_end_state() );
				tap_set_state( tap_get_end_state() );
				/* LOG_DEBUG("added TMS scan (no read)"); */
			}
			require_send = 1;
#ifdef _DEBUG_JTAG_IO_
			LOG_DEBUG( "runtest: %i, end in %s", cmd->cmd.runtest->num_cycles, tap_state_name( tap_get_end_state() ) );
#endif
			break;

		case JTAG_STATEMOVE:
			/* only send the maximum buffer size that FT2232C can handle */
			predicted_size = 3;
			if (ft2232_buffer_size + predicted_size + 1 > FT2232_BUFFER_SIZE)
			{
				if (ft2232_send_and_recv(first_unsent, cmd) != ERROR_OK)
					retval = ERROR_JTAG_QUEUE_FAILED;
				require_send = 0;
				first_unsent = cmd;
			}
			if (cmd->cmd.statemove->end_state != -1)
				ft2232_end_state(cmd->cmd.statemove->end_state);

			/* command "Clock Data to TMS/CS Pin (no Read)" */
			BUFFER_ADD = 0x4b;

			BUFFER_ADD = 0x6;       /* scan 7 bits */

			/* TMS data bits */
			BUFFER_ADD = tap_get_tms_path( tap_get_state(), tap_get_end_state() );
			/* LOG_DEBUG("added TMS scan (no read)"); */
			tap_set_state( tap_get_end_state() );
			require_send = 1;
#ifdef _DEBUG_JTAG_IO_
			LOG_DEBUG( "statemove: %s", tap_state_name( tap_get_end_state() ) );
#endif
			break;

		case JTAG_PATHMOVE:
			/* only send the maximum buffer size that FT2232C can handle */
			predicted_size = 3 * CEIL(cmd->cmd.pathmove->num_states, 7);
			if (ft2232_buffer_size + predicted_size + 1 > FT2232_BUFFER_SIZE)
			{
				if (ft2232_send_and_recv(first_unsent, cmd) != ERROR_OK)
					retval = ERROR_JTAG_QUEUE_FAILED;
				require_send = 0;
				first_unsent = cmd;
			}
			ft2232_add_pathmove(cmd->cmd.pathmove);
			require_send = 1;
#ifdef _DEBUG_JTAG_IO_
			LOG_DEBUG( "pathmove: %i states, end in %s", cmd->cmd.pathmove->num_states,
					tap_state_name(cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]) );
#endif
			break;

		case JTAG_SCAN:
			scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
			type = jtag_scan_type(cmd->cmd.scan);
			predicted_size = ft2232_predict_scan_out(scan_size, type);
			if ( (predicted_size + 1) > FT2232_BUFFER_SIZE )
			{
				LOG_DEBUG("oversized ft2232 scan (predicted_size > FT2232_BUFFER_SIZE)");
				/* unsent commands before this */
				if (first_unsent != cmd)
					if (ft2232_send_and_recv(first_unsent, cmd) != ERROR_OK)
						retval = ERROR_JTAG_QUEUE_FAILED;

				/* current command */
				if (cmd->cmd.scan->end_state != -1)
					ft2232_end_state(cmd->cmd.scan->end_state);
				ft2232_large_scan(cmd->cmd.scan, type, buffer, scan_size);
				require_send = 0;
				first_unsent = cmd->next;
				if (buffer)
					free(buffer);
				break;
			}
			else if (ft2232_buffer_size + predicted_size + 1 > FT2232_BUFFER_SIZE)
			{
				LOG_DEBUG("ft2232 buffer size reached, sending queued commands (first_unsent: %p, cmd: %p)",
						first_unsent,
						cmd);
				if (ft2232_send_and_recv(first_unsent, cmd) != ERROR_OK)
					retval = ERROR_JTAG_QUEUE_FAILED;
				require_send = 0;
				first_unsent = cmd;
			}
			ft2232_expect_read += ft2232_predict_scan_in(scan_size, type);
			/* LOG_DEBUG("new read size: %i", ft2232_expect_read); */
			if (cmd->cmd.scan->end_state != -1)
				ft2232_end_state(cmd->cmd.scan->end_state);
			ft2232_add_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size);
			require_send = 1;
			if (buffer)
				free(buffer);
#ifdef _DEBUG_JTAG_IO_
			LOG_DEBUG( "%s scan, %i bits, end in %s", (cmd->cmd.scan->ir_scan) ? "IR" : "DR", scan_size,
					tap_state_name( tap_get_end_state() ) );
#endif
			break;

		case JTAG_SLEEP:
			if (ft2232_send_and_recv(first_unsent, cmd) != ERROR_OK)
				retval = ERROR_JTAG_QUEUE_FAILED;
			first_unsent = cmd->next;
			jtag_sleep(cmd->cmd.sleep->us);
#ifdef _DEBUG_JTAG_IO_
			LOG_DEBUG( "sleep %i usec while in %s", cmd->cmd.sleep->us, tap_state_name( tap_get_state() ) );
#endif
			break;

		case JTAG_STABLECLOCKS:

			/* this is only allowed while in a stable state.  A check for a stable
			 * state was done in jtag_add_clocks()
			 */
			if (ft2232_stableclocks(cmd->cmd.stableclocks->num_cycles, cmd) != ERROR_OK)
				retval = ERROR_JTAG_QUEUE_FAILED;
#ifdef _DEBUG_JTAG_IO_
			LOG_DEBUG( "clocks %i while in %s", cmd->cmd.stableclocks->num_cycles, tap_state_name( tap_get_state() ) );
#endif
			break;

		default:
			LOG_ERROR("BUG: unknown JTAG command type encountered");
			exit(-1);
		}

		cmd = cmd->next;
	}

	if (require_send > 0)
		if (ft2232_send_and_recv(first_unsent, cmd) != ERROR_OK)
			retval = ERROR_JTAG_QUEUE_FAILED;

	return retval;
}


#if BUILD_FT2232_FTD2XX == 1
static int ft2232_init_ftd2xx(u16 vid, u16 pid, int more, int* try_more)
{
	FT_STATUS status;
	DWORD     openex_flags  = 0;
	char*     openex_string = NULL;
	u8        latency_timer;

	LOG_DEBUG("'ft2232' interface using FTD2XX with '%s' layout (%4.4x:%4.4x)", ft2232_layout, vid, pid);

#if IS_WIN32 == 0
	/* Add non-standard Vid/Pid to the linux driver */
	if ( ( status = FT_SetVIDPID(vid, pid) ) != FT_OK )
	{
		LOG_WARNING("couldn't add %4.4x:%4.4x", vid, pid);
	}
#endif

	if (ft2232_device_desc && ft2232_serial)
	{
		LOG_WARNING("can't open by device description and serial number, giving precedence to serial");
		ft2232_device_desc = NULL;
	}

	if (ft2232_device_desc)
	{
		openex_string = ft2232_device_desc;
		openex_flags  = FT_OPEN_BY_DESCRIPTION;
	}
	else if (ft2232_serial)
	{
		openex_string = ft2232_serial;
		openex_flags  = FT_OPEN_BY_SERIAL_NUMBER;
	}
	else
	{
		LOG_ERROR("neither device description nor serial number specified");
		LOG_ERROR("please add \"ft2232_device_desc <string>\" or \"ft2232_serial <string>\" to your .cfg file");

		return ERROR_JTAG_INIT_FAILED;
	}

	status = FT_OpenEx(openex_string, openex_flags, &ftdih);
	if( status != FT_OK ){
		// under Win32, the FTD2XX driver appends an "A" to the end
		// of the description, if we tried by the desc, then
		// try by the alternate "A" description.
		if( openex_string == ft2232_device_desc ){
			// Try the alternate method.
			openex_string = ft2232_device_desc_A;
			status = FT_OpenEx(openex_string, openex_flags, &ftdih);
			if( status == FT_OK ){
				// yea, the "alternate" method worked!
			} else {
				// drat, give the user a meaningfull message.
				// telling the use we tried *BOTH* methods.
				LOG_WARNING("Unable to open FTDI Device tried: '%s' and '%s'\n",
							ft2232_device_desc,
							ft2232_device_desc_A );
			}
		}
	}

	if ( status != FT_OK )
	{
		DWORD num_devices;

		if (more)
		{
			LOG_WARNING("unable to open ftdi device (trying more): %lu", status);
			*try_more = 1;
			return ERROR_JTAG_INIT_FAILED;
		}
		LOG_ERROR("unable to open ftdi device: %lu", status);
		status = FT_ListDevices(&num_devices, NULL, FT_LIST_NUMBER_ONLY);
		if (status == FT_OK)
		{
			char** desc_array = malloc( sizeof(char*) * (num_devices + 1) );
			int    i;

			for (i = 0; i < num_devices; i++)
				desc_array[i] = malloc(64);

			desc_array[num_devices] = NULL;

			status = FT_ListDevices(desc_array, &num_devices, FT_LIST_ALL | openex_flags);

			if (status == FT_OK)
			{
				LOG_ERROR("ListDevices: %lu\n", num_devices);
				for (i = 0; i < num_devices; i++)
					LOG_ERROR("%i: \"%s\"", i, desc_array[i]);
			}

			for (i = 0; i < num_devices; i++)
				free(desc_array[i]);

			free(desc_array);
		}
		else
		{
			LOG_ERROR("ListDevices: NONE\n");
		}
		return ERROR_JTAG_INIT_FAILED;
	}

	if ( ( status = FT_SetLatencyTimer(ftdih, ft2232_latency) ) != FT_OK )
	{
		LOG_ERROR("unable to set latency timer: %lu", status);
		return ERROR_JTAG_INIT_FAILED;
	}

	if ( ( status = FT_GetLatencyTimer(ftdih, &latency_timer) ) != FT_OK )
	{
		LOG_ERROR("unable to get latency timer: %lu", status);
		return ERROR_JTAG_INIT_FAILED;
	}
	else
	{
		LOG_DEBUG("current latency timer: %i", latency_timer);
	}

	if ( ( status = FT_SetTimeouts(ftdih, 5000, 5000) ) != FT_OK )
	{
		LOG_ERROR("unable to set timeouts: %lu", status);
		return ERROR_JTAG_INIT_FAILED;
	}

	if ( ( status = FT_SetBitMode(ftdih, 0x0b, 2) ) != FT_OK )
	{
		LOG_ERROR("unable to enable bit i/o mode: %lu", status);
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}


static int ft2232_purge_ftd2xx(void)
{
	FT_STATUS status;

	if ( ( status = FT_Purge(ftdih, FT_PURGE_RX | FT_PURGE_TX) ) != FT_OK )
	{
		LOG_ERROR("error purging ftd2xx device: %lu", status);
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}


#endif /* BUILD_FT2232_FTD2XX == 1 */

#if BUILD_FT2232_LIBFTDI == 1
static int ft2232_init_libftdi(u16 vid, u16 pid, int more, int* try_more)
{
	u8 latency_timer;

	LOG_DEBUG("'ft2232' interface using libftdi with '%s' layout (%4.4x:%4.4x)",
			ft2232_layout, vid, pid);

	if (ftdi_init(&ftdic) < 0)
		return ERROR_JTAG_INIT_FAILED;

	/* context, vendor id, product id */
	if (ftdi_usb_open_desc(&ftdic, vid, pid, ft2232_device_desc,
				ft2232_serial) < 0)
	{
		if (more)
			LOG_WARNING("unable to open ftdi device (trying more): %s",
					ftdic.error_str);
		else
			LOG_ERROR("unable to open ftdi device: %s", ftdic.error_str);
		*try_more = 1;
		return ERROR_JTAG_INIT_FAILED;
	}

	if (ftdi_set_interface(&ftdic, INTERFACE_A) < 0)
	{
		LOG_ERROR("unable to select FT2232 channel A: %s", ftdic.error_str);
		return ERROR_JTAG_INIT_FAILED;
	}

	if (ftdi_usb_reset(&ftdic) < 0)
	{
		LOG_ERROR("unable to reset ftdi device");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (ftdi_set_latency_timer(&ftdic, ft2232_latency) < 0)
	{
		LOG_ERROR("unable to set latency timer");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (ftdi_get_latency_timer(&ftdic, &latency_timer) < 0)
	{
		LOG_ERROR("unable to get latency timer");
		return ERROR_JTAG_INIT_FAILED;
	}
	else
	{
		LOG_DEBUG("current latency timer: %i", latency_timer);
	}

	ftdi_set_bitmode(&ftdic, 0x0b, 2); /* ctx, JTAG I/O mask */

	return ERROR_OK;
}


static int ft2232_purge_libftdi(void)
{
	if (ftdi_usb_purge_buffers(&ftdic) < 0)
	{
		LOG_ERROR("ftdi_purge_buffers: %s", ftdic.error_str);
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}


#endif /* BUILD_FT2232_LIBFTDI == 1 */

int ft2232_init(void)
{
	u8  buf[1];
	int retval;
	u32 bytes_written;
	ft2232_layout_t* cur_layout = ft2232_layouts;
	int i;

	if ( (ft2232_layout == NULL) || (ft2232_layout[0] == 0) )
	{
		ft2232_layout = "usbjtag";
		LOG_WARNING("No ft2232 layout specified, using default 'usbjtag'");
	}

	while (cur_layout->name)
	{
		if (strcmp(cur_layout->name, ft2232_layout) == 0)
		{
			layout = cur_layout;
			break;
		}
		cur_layout++;
	}

	if (!layout)
	{
		LOG_ERROR("No matching layout found for %s", ft2232_layout);
		return ERROR_JTAG_INIT_FAILED;
	}

	for (i = 0; 1; i++)
	{
		/*
		 * "more indicates that there are more IDs to try, so we should
		 * not print an error for an ID mismatch (but for anything
		 * else, we should).
		 *
		 * try_more indicates that the error code returned indicates an
		 * ID mismatch (and nothing else) and that we should proceeed
		 * with the next ID pair.
		 */
		int more     = ft2232_vid[i + 1] || ft2232_pid[i + 1];
		int try_more = 0;

#if BUILD_FT2232_FTD2XX == 1
		retval = ft2232_init_ftd2xx(ft2232_vid[i], ft2232_pid[i],
				more, &try_more);
#elif BUILD_FT2232_LIBFTDI == 1
		retval = ft2232_init_libftdi(ft2232_vid[i], ft2232_pid[i],
				more, &try_more);
#endif
		if (retval >= 0)
			break;
		if (!more || !try_more)
			return retval;
	}

	ft2232_buffer_size = 0;
	ft2232_buffer = malloc(FT2232_BUFFER_SIZE);

	if (layout->init() != ERROR_OK)
		return ERROR_JTAG_INIT_FAILED;

	ft2232_speed(jtag_speed);

	buf[0] = 0x85; /* Disconnect TDI/DO to TDO/DI for Loopback */
	if ( ( ( retval = ft2232_write(buf, 1, &bytes_written) ) != ERROR_OK ) || (bytes_written != 1) )
	{
		LOG_ERROR("couldn't write to FT2232 to disable loopback");
		return ERROR_JTAG_INIT_FAILED;
	}

#if BUILD_FT2232_FTD2XX == 1
	return ft2232_purge_ftd2xx();
#elif BUILD_FT2232_LIBFTDI == 1
	return ft2232_purge_libftdi();
#endif

	return ERROR_OK;
}


int usbjtag_init(void)
{
	u8  buf[3];
	u32 bytes_written;

	low_output    = 0x08;
	low_direction = 0x0b;

	if (strcmp(ft2232_layout, "usbjtag") == 0)
	{
		nTRST    = 0x10;
		nTRSTnOE = 0x10;
		nSRST    = 0x40;
		nSRSTnOE = 0x40;
	}
	else if (strcmp(ft2232_layout, "signalyzer") == 0)
	{
		nTRST    = 0x10;
		nTRSTnOE = 0x10;
		nSRST    = 0x20;
		nSRSTnOE = 0x20;
	}
	else if (strcmp(ft2232_layout, "evb_lm3s811") == 0)
	{
		nTRST = 0x0;
		nTRSTnOE = 0x00;
		nSRST = 0x20;
		nSRSTnOE = 0x20;
		low_output    = 0x88;
		low_direction = 0x8b;
	}
	else
	{
		LOG_ERROR("BUG: usbjtag_init called for unknown layout '%s'", ft2232_layout);
		return ERROR_JTAG_INIT_FAILED;
	}

	if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
	{
		low_direction &= ~nTRSTnOE; /* nTRST input */
		low_output    &= ~nTRST;    /* nTRST = 0 */
	}
	else
	{
		low_direction |= nTRSTnOE;  /* nTRST output */
		low_output    |= nTRST;     /* nTRST = 1 */
	}

	if (jtag_reset_config & RESET_SRST_PUSH_PULL)
	{
		low_direction |= nSRSTnOE;  /* nSRST output */
		low_output    |= nSRST;     /* nSRST = 1 */
	}
	else
	{
		low_direction &= ~nSRSTnOE; /* nSRST input */
		low_output    &= ~nSRST;    /* nSRST = 0 */
	}

	/* initialize low byte for jtag */
	buf[0] = 0x80;          /* command "set data bits low byte" */
	buf[1] = low_output;    /* value (TMS=1,TCK=0, TDI=0, xRST high) */
	buf[2] = low_direction; /* dir (output=1), TCK/TDI/TMS=out, TDO=in */
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if ( ( ( ft2232_write(buf, 3, &bytes_written) ) != ERROR_OK ) || (bytes_written != 3) )
	{
		LOG_ERROR("couldn't initialize FT2232 with 'USBJTAG' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}


int axm0432_jtag_init(void)
{
	u8  buf[3];
	u32 bytes_written;

	low_output    = 0x08;
	low_direction = 0x2b;

	/* initialize low byte for jtag */
	buf[0] = 0x80;          /* command "set data bits low byte" */
	buf[1] = low_output;    /* value (TMS=1,TCK=0, TDI=0, nOE=0) */
	buf[2] = low_direction; /* dir (output=1), TCK/TDI/TMS=out, TDO=in, nOE=out */
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if ( ( ( ft2232_write(buf, 3, &bytes_written) ) != ERROR_OK ) || (bytes_written != 3) )
	{
		LOG_ERROR("couldn't initialize FT2232 with 'JTAGkey' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (strcmp(layout->name, "axm0432_jtag") == 0)
	{
		nTRST    = 0x08;
		nTRSTnOE = 0x0;     /* No output enable for TRST*/
		nSRST    = 0x04;
		nSRSTnOE = 0x0;     /* No output enable for SRST*/
	}
	else
	{
		LOG_ERROR("BUG: axm0432_jtag_init called for non axm0432 layout");
		exit(-1);
	}

	high_output    = 0x0;
	high_direction = 0x0c;

	if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
	{
		LOG_ERROR("can't set nTRSTOE to push-pull on the Dicarlo jtag");
	}
	else
	{
		high_output |= nTRST;
	}

	if (jtag_reset_config & RESET_SRST_PUSH_PULL)
	{
		LOG_ERROR("can't set nSRST to push-pull on the Dicarlo jtag");
	}
	else
	{
		high_output |= nSRST;
	}

	/* initialize high port */
	buf[0] = 0x82;              /* command "set data bits high byte" */
	buf[1] = high_output;       /* value */
	buf[2] = high_direction;    /* all outputs (xRST and xRSTnOE) */
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if ( ( ( ft2232_write(buf, 3, &bytes_written) ) != ERROR_OK ) || (bytes_written != 3) )
	{
		LOG_ERROR("couldn't initialize FT2232 with 'Dicarlo' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}


int jtagkey_init(void)
{
	u8  buf[3];
	u32 bytes_written;

	low_output    = 0x08;
	low_direction = 0x1b;

	/* initialize low byte for jtag */
	buf[0] = 0x80;          /* command "set data bits low byte" */
	buf[1] = low_output;    /* value (TMS=1,TCK=0, TDI=0, nOE=0) */
	buf[2] = low_direction; /* dir (output=1), TCK/TDI/TMS=out, TDO=in, nOE=out */
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if ( ( ( ft2232_write(buf, 3, &bytes_written) ) != ERROR_OK ) || (bytes_written != 3) )
	{
		LOG_ERROR("couldn't initialize FT2232 with 'JTAGkey' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (strcmp(layout->name, "jtagkey") == 0)
	{
		nTRST    = 0x01;
		nTRSTnOE = 0x4;
		nSRST    = 0x02;
		nSRSTnOE = 0x08;
	}
	else if ( (strcmp(layout->name, "jtagkey_prototype_v1") == 0)
			 || (strcmp(layout->name, "oocdlink") == 0) )
	{
		nTRST    = 0x02;
		nTRSTnOE = 0x1;
		nSRST    = 0x08;
		nSRSTnOE = 0x04;
	}
	else
	{
		LOG_ERROR("BUG: jtagkey_init called for non jtagkey layout");
		exit(-1);
	}

	high_output    = 0x0;
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
	buf[0] = 0x82;              /* command "set data bits high byte" */
	buf[1] = high_output;       /* value */
	buf[2] = high_direction;    /* all outputs (xRST and xRSTnOE) */
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if ( ( ( ft2232_write(buf, 3, &bytes_written) ) != ERROR_OK ) || (bytes_written != 3) )
	{
		LOG_ERROR("couldn't initialize FT2232 with 'JTAGkey' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}


int olimex_jtag_init(void)
{
	u8  buf[3];
	u32 bytes_written;

	low_output    = 0x08;
	low_direction = 0x1b;

	/* initialize low byte for jtag */
	buf[0] = 0x80;          /* command "set data bits low byte" */
	buf[1] = low_output;    /* value (TMS=1,TCK=0, TDI=0, nOE=0) */
	buf[2] = low_direction; /* dir (output=1), TCK/TDI/TMS=out, TDO=in, nOE=out */
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if ( ( ( ft2232_write(buf, 3, &bytes_written) ) != ERROR_OK ) || (bytes_written != 3) )
	{
		LOG_ERROR("couldn't initialize FT2232 with 'JTAGkey' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	nTRST    = 0x01;
	nTRSTnOE = 0x4;
	nSRST    = 0x02;
	nSRSTnOE = 0x00; /* no output enable for nSRST */

	high_output    = 0x0;
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
		LOG_ERROR("can't set nSRST to push-pull on the Olimex ARM-USB-OCD");
	}
	else
	{
		high_output &= ~nSRST;
	}

	/* turn red LED on */
	high_output |= 0x08;

	/* initialize high port */
	buf[0] = 0x82;              /* command "set data bits high byte" */
	buf[1] = high_output;       /* value */
	buf[2] = high_direction;    /* all outputs (xRST and xRSTnOE) */
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if ( ( ( ft2232_write(buf, 3, &bytes_written) ) != ERROR_OK ) || (bytes_written != 3) )
	{
		LOG_ERROR("couldn't initialize FT2232 with 'JTAGkey' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}


int flyswatter_init(void)
{
	u8  buf[3];
	u32 bytes_written;

	low_output    = 0x18;
	low_direction = 0xfb;

	/* initialize low byte for jtag */
	buf[0] = 0x80;          /* command "set data bits low byte" */
	buf[1] = low_output;    /* value (TMS=1,TCK=0, TDI=0, nOE=0) */
	buf[2] = low_direction; /* dir (output=1), TCK/TDI/TMS=out, TDO=in, nOE[12]=out, n[ST]srst=out */
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if ( ( ( ft2232_write(buf, 3, &bytes_written) ) != ERROR_OK ) || (bytes_written != 3) )
	{
		LOG_ERROR("couldn't initialize FT2232 with 'flyswatter' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	nTRST    = 0x10;
	nTRSTnOE = 0x0;     /* not output enable for nTRST */
	nSRST    = 0x20;
	nSRSTnOE = 0x00;    /* no output enable for nSRST */

	high_output    = 0x00;
	high_direction = 0x0c;

	/* turn red LED3 on, LED2 off */
	high_output |= 0x08;

	/* initialize high port */
	buf[0] = 0x82;              /* command "set data bits high byte" */
	buf[1] = high_output;       /* value */
	buf[2] = high_direction;    /* all outputs (xRST and xRSTnOE) */
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if ( ( ( ft2232_write(buf, 3, &bytes_written) ) != ERROR_OK ) || (bytes_written != 3) )
	{
		LOG_ERROR("couldn't initialize FT2232 with 'flyswatter' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}


int turtle_init(void)
{
	u8  buf[3];
	u32 bytes_written;

	low_output    = 0x08;
	low_direction = 0x5b;

	/* initialize low byte for jtag */
	buf[0] = 0x80;          /* command "set data bits low byte" */
	buf[1] = low_output;    /* value (TMS=1,TCK=0, TDI=0, nOE=0) */
	buf[2] = low_direction; /* dir (output=1), TCK/TDI/TMS=out, TDO=in, nOE=out */
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if ( ( ( ft2232_write(buf, 3, &bytes_written) ) != ERROR_OK ) || (bytes_written != 3) )
	{
		LOG_ERROR("couldn't initialize FT2232 with 'turtelizer2' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	nSRST = 0x40;

	high_output    = 0x00;
	high_direction = 0x0C;

	/* initialize high port */
	buf[0] = 0x82; /* command "set data bits high byte" */
	buf[1] = high_output;
	buf[2] = high_direction;
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if ( ( ( ft2232_write(buf, 3, &bytes_written) ) != ERROR_OK ) || (bytes_written != 3) )
	{
		LOG_ERROR("couldn't initialize FT2232 with 'turtelizer2' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}


int comstick_init(void)
{
	u8  buf[3];
	u32 bytes_written;

	low_output    = 0x08;
	low_direction = 0x0b;

	/* initialize low byte for jtag */
	buf[0] = 0x80;          /* command "set data bits low byte" */
	buf[1] = low_output;    /* value (TMS=1,TCK=0, TDI=0, nOE=0) */
	buf[2] = low_direction; /* dir (output=1), TCK/TDI/TMS=out, TDO=in, nOE=out */
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if ( ( ( ft2232_write(buf, 3, &bytes_written) ) != ERROR_OK ) || (bytes_written != 3) )
	{
		LOG_ERROR("couldn't initialize FT2232 with 'comstick' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	nTRST    = 0x01;
	nTRSTnOE = 0x00;    /* no output enable for nTRST */
	nSRST    = 0x02;
	nSRSTnOE = 0x00;    /* no output enable for nSRST */

	high_output    = 0x03;
	high_direction = 0x03;

	/* initialize high port */
	buf[0] = 0x82; /* command "set data bits high byte" */
	buf[1] = high_output;
	buf[2] = high_direction;
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if ( ( ( ft2232_write(buf, 3, &bytes_written) ) != ERROR_OK ) || (bytes_written != 3) )
	{
		LOG_ERROR("couldn't initialize FT2232 with 'comstick' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}


int stm32stick_init(void)
{
	u8  buf[3];
	u32 bytes_written;

	low_output    = 0x88;
	low_direction = 0x8b;

	/* initialize low byte for jtag */
	buf[0] = 0x80;          /* command "set data bits low byte" */
	buf[1] = low_output;    /* value (TMS=1,TCK=0, TDI=0, nOE=0) */
	buf[2] = low_direction; /* dir (output=1), TCK/TDI/TMS=out, TDO=in, nOE=out */
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if ( ( ( ft2232_write(buf, 3, &bytes_written) ) != ERROR_OK ) || (bytes_written != 3) )
	{
		LOG_ERROR("couldn't initialize FT2232 with 'stm32stick' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	nTRST    = 0x01;
	nTRSTnOE = 0x00;    /* no output enable for nTRST */
	nSRST    = 0x80;
	nSRSTnOE = 0x00;    /* no output enable for nSRST */

	high_output    = 0x01;
	high_direction = 0x03;

	/* initialize high port */
	buf[0] = 0x82; /* command "set data bits high byte" */
	buf[1] = high_output;
	buf[2] = high_direction;
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if ( ( ( ft2232_write(buf, 3, &bytes_written) ) != ERROR_OK ) || (bytes_written != 3) )
	{
		LOG_ERROR("couldn't initialize FT2232 with 'stm32stick' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}


int sheevaplug_init(void)
{
	u8 buf[3];
	u32 bytes_written;

	low_output = 0x08;
	low_direction = 0x1b;

	/* initialize low byte for jtag */
	buf[0] = 0x80; /* command "set data bits low byte" */
	buf[1] = low_output; /* value (TMS=1,TCK=0, TDI=0, nOE=0) */
	buf[2] = low_direction; /* dir (output=1), TCK/TDI/TMS=out, TDO=in */
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if (((ft2232_write(buf, 3, &bytes_written)) != ERROR_OK) || (bytes_written != 3))
	{
		LOG_ERROR("couldn't initialize FT2232 with 'sheevaplug' layout"); 
		return ERROR_JTAG_INIT_FAILED;
	}

	nTRSTnOE = 0x1;
	nTRST = 0x02;
	nSRSTnOE = 0x4;
	nSRST = 0x08;

	high_output = 0x0;
	high_direction = 0x0f;

	/* nTRST is always push-pull */
	high_output &= ~nTRSTnOE;
	high_output |= nTRST;

	/* nSRST is always open-drain */
	high_output |= nSRSTnOE;
	high_output &= ~nSRST;

	/* initialize high port */
	buf[0] = 0x82; /* command "set data bits high byte" */
	buf[1] = high_output; /* value */
	buf[2] = high_direction;   /* all outputs - xRST */
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if (((ft2232_write(buf, 3, &bytes_written)) != ERROR_OK) || (bytes_written != 3))
	{
		LOG_ERROR("couldn't initialize FT2232 with 'sheevaplug' layout"); 
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

void olimex_jtag_blink(void)
{
	/* Olimex ARM-USB-OCD has a LED connected to ACBUS3
	 * ACBUS3 is bit 3 of the GPIOH port
	 */
	if (high_output & 0x08)
	{
		/* set port pin high */
		high_output &= 0x07;
	}
	else
	{
		/* set port pin low */
		high_output |= 0x08;
	}

	BUFFER_ADD = 0x82;
	BUFFER_ADD = high_output;
	BUFFER_ADD = high_direction;
}


void flyswatter_jtag_blink(void)
{
	/*
	 * Flyswatter has two LEDs connected to ACBUS2 and ACBUS3
	 */
	high_output ^= 0x0c;

	BUFFER_ADD = 0x82;
	BUFFER_ADD = high_output;
	BUFFER_ADD = high_direction;
}


void turtle_jtag_blink(void)
{
	/*
	 * Turtelizer2 has two LEDs connected to ACBUS2 and ACBUS3
	 */
	if (high_output & 0x08)
	{
		high_output = 0x04;
	}
	else
	{
		high_output = 0x08;
	}

	BUFFER_ADD = 0x82;
	BUFFER_ADD = high_output;
	BUFFER_ADD = high_direction;
}


int ft2232_quit(void)
{
#if BUILD_FT2232_FTD2XX == 1
	FT_STATUS status;

	status = FT_Close(ftdih);
#elif BUILD_FT2232_LIBFTDI == 1
	ftdi_disable_bitbang(&ftdic);

	ftdi_usb_close(&ftdic);

	ftdi_deinit(&ftdic);
#endif

	free(ft2232_buffer);
	ft2232_buffer = NULL;

	return ERROR_OK;
}


int ft2232_handle_device_desc_command(struct command_context_s* cmd_ctx, char* cmd, char** args, int argc)
{
	char *cp;
	char buf[200];
	if (argc == 1)
	{
		ft2232_device_desc = strdup(args[0]);
		cp = strchr( ft2232_device_desc, 0 );
		// under Win32, the FTD2XX driver appends an "A" to the end
		// of the description, this examines the given desc
		// and creates the 'missing' _A or non_A variable.
		if( (cp[-1] == 'A') && (cp[-2]==' ') ){
			// it was, so make this the "A" version.
			ft2232_device_desc_A = ft2232_device_desc;
			// and *CREATE* the non-A version.
			strcpy( buf, ft2232_device_desc );
			cp = strchr( buf, 0 );
			cp[-2] = 0;
			ft2232_device_desc =  strdup( buf );
		} else {
			// <space>A not defined
			// so create it
			sprintf( buf, "%s A", ft2232_device_desc );
			ft2232_device_desc_A = strdup( buf );
		}
	}
	else
	{
		LOG_ERROR("expected exactly one argument to ft2232_device_desc <description>");
	}

	return ERROR_OK;
}


int ft2232_handle_serial_command(struct command_context_s* cmd_ctx, char* cmd, char** args, int argc)
{
	if (argc == 1)
	{
		ft2232_serial = strdup(args[0]);
	}
	else
	{
		LOG_ERROR("expected exactly one argument to ft2232_serial <serial-number>");
	}

	return ERROR_OK;
}


int ft2232_handle_layout_command(struct command_context_s* cmd_ctx, char* cmd, char** args, int argc)
{
	if (argc == 0)
		return ERROR_OK;

	ft2232_layout = malloc(strlen(args[0]) + 1);
	strcpy(ft2232_layout, args[0]);

	return ERROR_OK;
}


int ft2232_handle_vid_pid_command(struct command_context_s* cmd_ctx, char* cmd, char** args, int argc)
{
	int i;

	if (argc > MAX_USB_IDS * 2)
	{
		LOG_WARNING("ignoring extra IDs in ft2232_vid_pid "
					"(maximum is %d pairs)", MAX_USB_IDS);
		argc = MAX_USB_IDS * 2;
	}
	if ( argc < 2 || (argc & 1) )
	{
		LOG_WARNING("incomplete ft2232_vid_pid configuration directive");
		if (argc < 2)
			return ERROR_OK;
	}

	for (i = 0; i + 1 < argc; i += 2)
	{
		ft2232_vid[i >> 1] = strtol(args[i], NULL, 0);
		ft2232_pid[i >> 1] = strtol(args[i + 1], NULL, 0);
	}

	/*
	 * Explicitly terminate, in case there are multiples instances of
	 * ft2232_vid_pid.
	 */
	ft2232_vid[i >> 1] = ft2232_pid[i >> 1] = 0;

	return ERROR_OK;
}


int ft2232_handle_latency_command(struct command_context_s* cmd_ctx, char* cmd, char** args, int argc)
{
	if (argc == 1)
	{
		ft2232_latency = atoi(args[0]);
	}
	else
	{
		LOG_ERROR("expected exactly one argument to ft2232_latency <ms>");
	}

	return ERROR_OK;
}


static int ft2232_stableclocks(int num_cycles, jtag_command_t* cmd)
{
	int retval = 0;

	/* 7 bits of either ones or zeros. */
	u8  tms = (tap_get_state() == TAP_RESET ? 0x7F : 0x00);

	while (num_cycles > 0)
	{
		/* the command 0x4b, "Clock Data to TMS/CS Pin (no Read)" handles
		 * at most 7 bits per invocation.  Here we invoke it potentially
		 * several times.
		 */
		int bitcount_per_command = (num_cycles > 7) ? 7 : num_cycles;

		if (ft2232_buffer_size + 3 >= FT2232_BUFFER_SIZE)
		{
			if (ft2232_send_and_recv(first_unsent, cmd) != ERROR_OK)
				retval = ERROR_JTAG_QUEUE_FAILED;

			first_unsent = cmd;
		}

		/* command "Clock Data to TMS/CS Pin (no Read)" */
		BUFFER_ADD = 0x4b;

		/* scan 7 bit */
		BUFFER_ADD = bitcount_per_command - 1;

		/* TMS data bits are either all zeros or ones to stay in the current stable state */
		BUFFER_ADD = tms;

		require_send = 1;

		num_cycles -= bitcount_per_command;
	}

	return retval;
}
