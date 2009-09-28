/***************************************************************************
*   Copyright (C) 2009 by Øyvind Harboe                                   *
*	Øyvind Harboe <oyvind.harboe@zylin.com>                               *
*                                                                         *
*   Copyright (C) 2009 by SoftPLC Corporation.  http://softplc.com        *
*	Dick Hollenbeck <dick@softplc.com>                                    *
*                                                                         *
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
 *
 * The datasheet for the ftdichip.com's FT2232D part is here:
 * http://www.ftdichip.com/Documents/DataSheets/DS_FT2232D.pdf
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/* project specific includes */
#include "interface.h"
#include "commands.h"
#include "time_support.h"

#if IS_CYGWIN == 1
#include <windows.h>
#endif

#include <assert.h>

#if (BUILD_FT2232_FTD2XX == 1 && BUILD_FT2232_LIBFTDI == 1)
#error "BUILD_FT2232_FTD2XX && BUILD_FT2232_LIBFTDI are mutually exclusive"
#elif (BUILD_FT2232_FTD2XX != 1 && BUILD_FT2232_LIBFTDI != 1)
#error "BUILD_FT2232_FTD2XX || BUILD_FT2232_LIBFTDI must be chosen"
#endif

/* FT2232 access library includes */
#if BUILD_FT2232_FTD2XX == 1
#include <ftd2xx.h>
#elif BUILD_FT2232_LIBFTDI == 1
#include <ftdi.h>
#endif

/* max TCK for the high speed devices 30000 kHz */
#define	FTDI_2232H_4232H_MAX_TCK	30000
/* max TCK for the full speed devices 6000 kHz */
#define	FTDI_2232C_MAX_TCK 6000
/* this speed value tells that RTCK is requested */
#define RTCK_SPEED -1

#ifndef BUILD_FT2232_HIGHSPEED
 #if BUILD_FT2232_FTD2XX == 1
	enum { FT_DEVICE_2232H = 6, FT_DEVICE_4232H };
 #elif BUILD_FT2232_LIBFTDI == 1
	enum { TYPE_2232H = 4, TYPE_4232H = 5 };
 #endif
#endif

static int ft2232_execute_queue(void);
static int ft2232_speed(int speed);
static int ft2232_speed_div(int speed, int* khz);
static int ft2232_khz(int khz, int* jtag_speed);
static int ft2232_register_commands(struct command_context_s* cmd_ctx);
static int ft2232_init(void);
static int ft2232_quit(void);

static int ft2232_handle_device_desc_command(struct command_context_s* cmd_ctx, char* cmd, char** args, int argc);
static int ft2232_handle_serial_command(struct command_context_s* cmd_ctx, char* cmd, char** args, int argc);
static int ft2232_handle_layout_command(struct command_context_s* cmd_ctx, char* cmd, char** args, int argc);
static int ft2232_handle_vid_pid_command(struct command_context_s* cmd_ctx, char* cmd, char** args, int argc);
static int ft2232_handle_latency_command(struct command_context_s* cmd_ctx, char* cmd, char** args, int argc);

/**
 * Send out \a num_cycles on the TCK line while the TAP(s) are in a
 * stable state.  Calling code must ensure that current state is stable,
 * that verification is not done in here.
 *
 * @param num_cycles The number of clocks cycles to send.
 * @param cmd The command to send.
 *
 * @returns ERROR_OK on success, or ERROR_JTAG_QUEUE_FAILED on failure.
 */
static int ft2232_stableclocks(int num_cycles, jtag_command_t* cmd);

static char *       ft2232_device_desc_A = NULL;
static char*        ft2232_device_desc = NULL;
static char*        ft2232_serial  = NULL;
static char*        ft2232_layout  = NULL;
static uint8_t		ft2232_latency = 2;
static unsigned		ft2232_max_tck = FTDI_2232C_MAX_TCK;

#define MAX_USB_IDS 8
/* vid = pid = 0 marks the end of the list */
static uint16_t ft2232_vid[MAX_USB_IDS + 1] = { 0x0403, 0 };
static uint16_t ft2232_pid[MAX_USB_IDS + 1] = { 0x6010, 0 };

typedef struct ft2232_layout_s
{
	char* name;
	int (*init)(void);
	void (*reset)(int trst, int srst);
	void (*blink)(void);
} ft2232_layout_t;

/* init procedures for supported layouts */
static int usbjtag_init(void);
static int jtagkey_init(void);
static int olimex_jtag_init(void);
static int flyswatter_init(void);
static int turtle_init(void);
static int comstick_init(void);
static int stm32stick_init(void);
static int axm0432_jtag_init(void);
static int sheevaplug_init(void);
static int icebear_jtag_init(void);
static int cortino_jtag_init(void);

/* reset procedures for supported layouts */
static void usbjtag_reset(int trst, int srst);
static void jtagkey_reset(int trst, int srst);
static void olimex_jtag_reset(int trst, int srst);
static void flyswatter_reset(int trst, int srst);
static void turtle_reset(int trst, int srst);
static void comstick_reset(int trst, int srst);
static void stm32stick_reset(int trst, int srst);
static void axm0432_jtag_reset(int trst, int srst);
static void sheevaplug_reset(int trst, int srst);
static void icebear_jtag_reset(int trst, int srst);

/* blink procedures for layouts that support a blinking led */
static void olimex_jtag_blink(void);
static void flyswatter_jtag_blink(void);
static void turtle_jtag_blink(void);

static const ft2232_layout_t  ft2232_layouts[] =
{
	{ "usbjtag",              usbjtag_init,              usbjtag_reset,      NULL                    },
	{ "jtagkey",              jtagkey_init,              jtagkey_reset,      NULL                    },
	{ "jtagkey_prototype_v1", jtagkey_init,              jtagkey_reset,      NULL                    },
	{ "oocdlink",             jtagkey_init,              jtagkey_reset,      NULL                    },
	{ "signalyzer",           usbjtag_init,              usbjtag_reset,      NULL                    },
	{ "evb_lm3s811",          usbjtag_init,              usbjtag_reset,      NULL                    },
	{ "luminary_icdi",        usbjtag_init,              usbjtag_reset,      NULL                    },
	{ "olimex-jtag",          olimex_jtag_init,          olimex_jtag_reset,  olimex_jtag_blink       },
	{ "flyswatter",           flyswatter_init,           flyswatter_reset,   flyswatter_jtag_blink   },
	{ "turtelizer2",          turtle_init,               turtle_reset,       turtle_jtag_blink       },
	{ "comstick",             comstick_init,             comstick_reset,     NULL                    },
	{ "stm32stick",           stm32stick_init,           stm32stick_reset,   NULL                    },
	{ "axm0432_jtag",         axm0432_jtag_init,         axm0432_jtag_reset, NULL                    },
	{ "sheevaplug",           sheevaplug_init,           sheevaplug_reset,   NULL                    },
	{ "icebear",              icebear_jtag_init,         icebear_jtag_reset, NULL                    },
	{ "cortino",              cortino_jtag_init,         comstick_reset, NULL                        },
	{ NULL,                   NULL,                      NULL,               NULL                    },
};

static uint8_t                  nTRST, nTRSTnOE, nSRST, nSRSTnOE;

static const ft2232_layout_t *layout;
static uint8_t                  low_output     = 0x0;
static uint8_t                  low_direction  = 0x0;
static uint8_t                  high_output    = 0x0;
static uint8_t                  high_direction = 0x0;

#if BUILD_FT2232_FTD2XX == 1
static FT_HANDLE	ftdih = NULL;
static FT_DEVICE	ftdi_device = 0;
#elif BUILD_FT2232_LIBFTDI == 1
static struct ftdi_context ftdic;
static enum ftdi_chip_type ftdi_device;
#endif

static jtag_command_t* first_unsent;        /* next command that has to be sent */
static int             require_send;

/*	http://urjtag.wiki.sourceforge.net/Cable + FT2232 says:

	"There is a significant difference between libftdi and libftd2xx. The latter
	one allows to schedule up to 64*64 bytes of result data while libftdi fails
	with more than 4*64. As a consequence, the FT2232 driver is forced to
	perform around 16x more USB transactions for long command streams with TDO
	capture when running with libftdi."

	No idea how we get
	#define FT2232_BUFFER_SIZE 131072
	a comment would have been nice.
*/

#define FT2232_BUFFER_SIZE 131072

static uint8_t*             ft2232_buffer = NULL;
static int             ft2232_buffer_size  = 0;
static int             ft2232_read_pointer = 0;
static int             ft2232_expect_read  = 0;

/**
 * Function buffer_write
 * writes a byte into the byte buffer, "ft2232_buffer", which must be sent later.
 * @param val is the byte to send.
 */
static inline void buffer_write(uint8_t val)
{
	assert(ft2232_buffer);
	assert((unsigned) ft2232_buffer_size < (unsigned) FT2232_BUFFER_SIZE);
	ft2232_buffer[ft2232_buffer_size++] = val;
}

/**
 * Function buffer_read
 * returns a byte from the byte buffer.
 */
static inline uint8_t buffer_read(void)
{
	assert(ft2232_buffer);
	assert(ft2232_read_pointer < ft2232_buffer_size);
	return ft2232_buffer[ft2232_read_pointer++];
}

/**
 * Clocks out \a bit_count bits on the TMS line, starting with the least
 * significant bit of tms_bits and progressing to more significant bits.
 * Rigorous state transition logging is done here via tap_set_state().
 *
 * @param mpsse_cmd One of the MPSSE TMS oriented commands such as
 *	0x4b or 0x6b.  See the MPSSE spec referenced above for their
 *	functionality. The MPSSE command "Clock Data to TMS/CS Pin (no Read)"
 *	is often used for this, 0x4b.
 *
 * @param tms_bits Holds the sequence of bits to send.
 * @param tms_count Tells how many bits in the sequence.
 * @param tdi_bit A single bit to pass on to TDI before the first TCK
 *	cycle and held static for the duration of TMS clocking.
 *
 * See the MPSSE spec referenced above.
 */
static void clock_tms(uint8_t mpsse_cmd, int tms_bits, int tms_count, bool tdi_bit)
{
	uint8_t	tms_byte;
	int	i;
	int	tms_ndx;				/* bit index into tms_byte */

	assert(tms_count > 0);

#if 0
	LOG_DEBUG("mpsse cmd=%02x, tms_bits = 0x%08x, bit_count=%d", mpsse_cmd, tms_bits, tms_count);
#endif

	for (tms_byte = tms_ndx = i = 0;   i < tms_count;   ++i, tms_bits>>=1)
	{
		bool bit = tms_bits & 1;

		if (bit)
			tms_byte |= (1 << tms_ndx);

		/* always do state transitions in public view */
		tap_set_state(tap_state_transition(tap_get_state(), bit));

		/*	we wrote a bit to tms_byte just above, increment bit index.  if bit was zero
			also increment.
		*/
		++tms_ndx;

		if (tms_ndx == 7  || i == tms_count-1)
		{
			buffer_write(mpsse_cmd);
			buffer_write(tms_ndx - 1);

			/*	Bit 7 of the byte is passed on to TDI/DO before the first TCK/SK of
				TMS/CS and is held static for the duration of TMS/CS clocking.
			*/
			buffer_write(tms_byte | (tdi_bit << 7));
		}
	}
}

/**
 * Function get_tms_buffer_requirements
 * returns what clock_tms() will consume if called with
 * same \a bit_count.
 */
static inline int get_tms_buffer_requirements(int bit_count)
{
	return ((bit_count + 6)/7) * 3;
}

/**
 * Function move_to_state
 * moves the TAP controller from the current state to a
 * \a goal_state through a path given by tap_get_tms_path().  State transition
 * logging is performed by delegation to clock_tms().
 *
 * @param goal_state is the destination state for the move.
 */
static void move_to_state(tap_state_t goal_state)
{
	tap_state_t	start_state = tap_get_state();

	/*	goal_state is 1/2 of a tuple/pair of states which allow convenient
		lookup of the required TMS pattern to move to this state from the
		start state.
	*/

	/* do the 2 lookups */
	int tms_bits  = tap_get_tms_path(start_state, goal_state);
	int tms_count = tap_get_tms_path_len(start_state, goal_state);

	DEBUG_JTAG_IO("start=%s goal=%s", tap_state_name(start_state), tap_state_name(goal_state));

	clock_tms(0x4b,  tms_bits, tms_count, 0);
}

jtag_interface_t ft2232_interface =
{
	.name			= "ft2232",
	.execute_queue		= ft2232_execute_queue,
	.speed			= ft2232_speed,
	.speed_div		= ft2232_speed_div,
	.khz			= ft2232_khz,
	.register_commands	= ft2232_register_commands,
	.init			= ft2232_init,
	.quit			= ft2232_quit,
};

static int ft2232_write(uint8_t* buf, int size, uint32_t* bytes_written)
{
#if BUILD_FT2232_FTD2XX == 1
	FT_STATUS status;
	DWORD dw_bytes_written;
	if ((status = FT_Write(ftdih, buf, size, &dw_bytes_written)) != FT_OK)
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
	if ((retval = ftdi_write_data(&ftdic, buf, size)) < 0)
	{
		*bytes_written = 0;
		LOG_ERROR("ftdi_write_data: %s", ftdi_get_error_string(&ftdic));
		return ERROR_JTAG_DEVICE_ERROR;
	}
	else
	{
		*bytes_written = retval;
		return ERROR_OK;
	}
#endif
}

static int ft2232_read(uint8_t* buf, uint32_t size, uint32_t* bytes_read)
{
#if BUILD_FT2232_FTD2XX == 1
	DWORD dw_bytes_read;
	FT_STATUS status;
	int timeout = 5;
	*bytes_read = 0;

	while ((*bytes_read < size) && timeout--)
	{
		if ((status = FT_Read(ftdih, buf + *bytes_read, size -
					  *bytes_read, &dw_bytes_read)) != FT_OK)
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

	while ((*bytes_read < size) && timeout--)
	{
		if ((retval = ftdi_read_data(&ftdic, buf + *bytes_read, size - *bytes_read)) < 0)
		{
			*bytes_read = 0;
			LOG_ERROR("ftdi_read_data: %s", ftdi_get_error_string(&ftdic));
			return ERROR_JTAG_DEVICE_ERROR;
		}
		*bytes_read += retval;
	}

#endif

	if (*bytes_read < size)
	{
		LOG_ERROR("couldn't read the requested number of bytes from FT2232 device (%i < %i)",
			  (unsigned int)(*bytes_read),
			  (unsigned int)size);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static bool ft2232_device_is_highspeed(void)
{
#if BUILD_FT2232_FTD2XX == 1
	return (ftdi_device == FT_DEVICE_2232H) || (ftdi_device == FT_DEVICE_4232H);
#elif BUILD_FT2232_LIBFTDI == 1
	return (ftdi_device == TYPE_2232H || ftdi_device == TYPE_4232H);
#endif
}

/*
 * Commands that only apply to the FT2232H and FT4232H devices.
 * See chapter 6 in http://www.ftdichip.com/Documents/AppNotes/
 * AN_108_Command_Processor_for_MPSSE_and_MCU_Host_Bus_Emulation_Modes.pdf
 */

static int ft2232h_ft4232h_adaptive_clocking(bool enable)
{
	uint8_t buf = enable ? 0x96 : 0x97;
	LOG_DEBUG("%2.2x", buf);

	uint32_t bytes_written;
	int retval = ft2232_write(&buf, 1, &bytes_written);
	if ((ERROR_OK != retval) || (bytes_written != 1))
	{
		LOG_ERROR("couldn't write command to %s adaptive clocking"
			, enable ? "enable" : "disable");
		return retval;
	}

	return ERROR_OK;
}

/**
 * Enable/disable the clk divide by 5 of the 60MHz master clock.
 * This result in a JTAG clock speed range of 91.553Hz-6MHz
 * respective 457.763Hz-30MHz.
 */
static int ft2232h_ft4232h_clk_divide_by_5(bool enable)
{
	uint32_t bytes_written;
	uint8_t buf = enable ?  0x8b : 0x8a;
	int retval = ft2232_write(&buf, 1, &bytes_written);
	if ((ERROR_OK != retval) || (bytes_written != 1))
	{
		LOG_ERROR("couldn't write command to %s clk divide by 5"
			, enable ? "enable" : "disable");
		return ERROR_JTAG_INIT_FAILED;
	}
	ft2232_max_tck = enable ? FTDI_2232C_MAX_TCK : FTDI_2232H_4232H_MAX_TCK;
	LOG_INFO("max TCK change to: %u kHz", ft2232_max_tck);

	return ERROR_OK;
}

static int ft2232_speed(int speed)
{
	uint8_t buf[3];
	int retval;
	uint32_t bytes_written;

	retval = ERROR_OK;
	bool enable_adaptive_clocking = (RTCK_SPEED == speed);
	if (ft2232_device_is_highspeed())
		retval = ft2232h_ft4232h_adaptive_clocking(enable_adaptive_clocking);
	else if (enable_adaptive_clocking)
	{
		LOG_ERROR("ft2232 device %lu does not support RTCK"
			, (long unsigned int)ftdi_device);
		return ERROR_FAIL;
	}

	if ((enable_adaptive_clocking) || (ERROR_OK != retval))
		return retval;

	buf[0] = 0x86;					/* command "set divisor" */
	buf[1] = speed & 0xff;			/* valueL (0 = 6MHz, 1 = 3MHz, 2 = 2.0MHz, ...*/
	buf[2] = (speed >> 8) & 0xff;	/* valueH */

	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);
	if (((retval = ft2232_write(buf, 3, &bytes_written)) != ERROR_OK) || (bytes_written != 3))
	{
		LOG_ERROR("couldn't set FT2232 TCK speed");
		return retval;
	}

	return ERROR_OK;
}

static int ft2232_speed_div(int speed, int* khz)
{
	/* Take a look in the FT2232 manual,
	 * AN2232C-01 Command Processor for
	 * MPSSE and MCU Host Bus. Chapter 3.8 */

	*khz = (RTCK_SPEED == speed) ? 0 : ft2232_max_tck / (1 + speed);

	return ERROR_OK;
}

static int ft2232_khz(int khz, int* jtag_speed)
{
	if (khz == 0)
	{
		if (ft2232_device_is_highspeed())
		{
			*jtag_speed = RTCK_SPEED;
			return ERROR_OK;
		}
		else
		{
			LOG_DEBUG("RCLK not supported");
			return ERROR_FAIL;
		}
	}

	/* Take a look in the FT2232 manual,
	 * AN2232C-01 Command Processor for
	 * MPSSE and MCU Host Bus. Chapter 3.8
	 *
	 * We will calc here with a multiplier
	 * of 10 for better rounding later. */

	/* Calc speed, (ft2232_max_tck / khz) - 1 */
	/* Use 65000 for better rounding */
	*jtag_speed = ((ft2232_max_tck*10) / khz) - 10;

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

static int ft2232_register_commands(struct command_context_s* cmd_ctx)
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

static void ft2232_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else
	{
		LOG_ERROR("BUG: %s is not a stable end state", tap_state_name(state));
		exit(-1);
	}
}

static void ft2232_read_scan(enum scan_type type, uint8_t* buffer, int scan_size)
{
	int num_bytes = (scan_size + 7) / 8;
	int bits_left = scan_size;
	int cur_byte  = 0;

	while (num_bytes-- > 1)
	{
		buffer[cur_byte++] = buffer_read();
		bits_left -= 8;
	}

	buffer[cur_byte] = 0x0;

	/* There is one more partial byte left from the clock data in/out instructions */
	if (bits_left > 1)
	{
		buffer[cur_byte] = buffer_read() >> 1;
	}
	/* This shift depends on the length of the clock data to tms instruction, insterted at end of the scan, now fixed to a two step transition in ft2232_add_scan */
	buffer[cur_byte] = (buffer[cur_byte] | (((buffer_read()) << 1) & 0x80)) >> (8 - bits_left);
}

static void ft2232_debug_dump_buffer(void)
{
	int i;
	char line[256];
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

static int ft2232_send_and_recv(jtag_command_t* first, jtag_command_t* last)
{
	jtag_command_t* cmd;
	uint8_t* buffer;
	int scan_size;
	enum scan_type  type;
	int retval;
	uint32_t bytes_written = 0;
	uint32_t bytes_read = 0;

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

	if ((retval = ft2232_write(ft2232_buffer, ft2232_buffer_size, &bytes_written)) != ERROR_OK)
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

		if ((retval = ft2232_read(ft2232_buffer, ft2232_expect_read, &bytes_read)) != ERROR_OK)
		{
			LOG_ERROR("couldn't read from FT2232");
			return retval;
		}

#ifdef _DEBUG_USB_IO_
		gettimeofday(&end, NULL);

		timeval_subtract(&d_inter, &inter, &start);
		timeval_subtract(&d_inter2, &inter2, &start);
		timeval_subtract(&d_end, &end, &start);

		LOG_INFO("inter: %u.%06u, inter2: %u.%06u end: %u.%06u",
			(unsigned)d_inter.tv_sec, (unsigned)d_inter.tv_usec,
			(unsigned)d_inter2.tv_sec, (unsigned)d_inter2.tv_usec,
			(unsigned)d_end.tv_sec, (unsigned)d_end.tv_usec);
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

/**
 * Function ft2232_add_pathmove
 * moves the TAP controller from the current state to a new state through the
 * given path, where path is an array of tap_state_t's.
 *
 * @param path is an array of tap_stat_t which gives the states to traverse through
 *   ending with the last state at path[num_states-1]
 * @param num_states is the count of state steps to move through
 */
static void ft2232_add_pathmove(tap_state_t* path, int num_states)
{
	int state_count = 0;

	assert((unsigned) num_states <= 32u);		/* tms_bits only holds 32 bits */

	/* this loop verifies that the path is legal and logs each state in the path */
	while (num_states)
	{
		unsigned char	tms_byte = 0;       /* zero this on each MPSSE batch */
		int		bit_count = 0;
		int		num_states_batch = num_states > 7 ? 7 : num_states;

		/* command "Clock Data to TMS/CS Pin (no Read)" */
		buffer_write(0x4b);

		/* number of states remaining */
		buffer_write(num_states_batch - 1);

		while (num_states_batch--) {
			/* either TMS=0 or TMS=1 must work ... */
			if (tap_state_transition(tap_get_state(), false)
						== path[state_count])
				buf_set_u32(&tms_byte, bit_count++, 1, 0x0);
			else if (tap_state_transition(tap_get_state(), true)
						== path[state_count])
				buf_set_u32(&tms_byte, bit_count++, 1, 0x1);

			/* ... or else the caller goofed BADLY */
			else {
				LOG_ERROR("BUG: %s -> %s isn't a valid "
						"TAP state transition",
					tap_state_name(tap_get_state()),
					tap_state_name(path[state_count]));
				exit(-1);
			}

			tap_set_state(path[state_count]);
			state_count++;
			num_states--;
		}

		buffer_write(tms_byte);
	}
	tap_set_end_state(tap_get_state());
}

static void ft2232_add_scan(bool ir_scan, enum scan_type type, uint8_t* buffer, int scan_size)
{
	int num_bytes = (scan_size + 7) / 8;
	int bits_left = scan_size;
	int cur_byte  = 0;
	int last_bit;

	if (!ir_scan)
	{
		if (tap_get_state() != TAP_DRSHIFT)
		{
			move_to_state(TAP_DRSHIFT);
		}
	}
	else
	{
		if (tap_get_state() != TAP_IRSHIFT)
		{
			move_to_state(TAP_IRSHIFT);
		}
	}

	/* add command for complete bytes */
	while (num_bytes > 1)
	{
		int thisrun_bytes;
		if (type == SCAN_IO)
		{
			/* Clock Data Bytes In and Out LSB First */
			buffer_write(0x39);
			/* LOG_DEBUG("added TDI bytes (io %i)", num_bytes); */
		}
		else if (type == SCAN_OUT)
		{
			/* Clock Data Bytes Out on -ve Clock Edge LSB First (no Read) */
			buffer_write(0x19);
			/* LOG_DEBUG("added TDI bytes (o)"); */
		}
		else if (type == SCAN_IN)
		{
			/* Clock Data Bytes In on +ve Clock Edge LSB First (no Write) */
			buffer_write(0x28);
			/* LOG_DEBUG("added TDI bytes (i %i)", num_bytes); */
		}

		thisrun_bytes = (num_bytes > 65537) ? 65536 : (num_bytes - 1);
		num_bytes    -= thisrun_bytes;

		buffer_write((uint8_t) (thisrun_bytes - 1));
		buffer_write((uint8_t) ((thisrun_bytes - 1) >> 8));

		if (type != SCAN_IN)
		{
			/* add complete bytes */
			while (thisrun_bytes-- > 0)
			{
				buffer_write(buffer[cur_byte++]);
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
		last_bit = (buffer[cur_byte] >> (bits_left - 1)) & 0x1;
	else
		last_bit = 0;

	/* process remaining bits but the last one */
	if (bits_left > 1)
	{
		if (type == SCAN_IO)
		{
			/* Clock Data Bits In and Out LSB First */
			buffer_write(0x3b);
			/* LOG_DEBUG("added TDI bits (io) %i", bits_left - 1); */
		}
		else if (type == SCAN_OUT)
		{
			/* Clock Data Bits Out on -ve Clock Edge LSB First (no Read) */
			buffer_write(0x1b);
			/* LOG_DEBUG("added TDI bits (o)"); */
		}
		else if (type == SCAN_IN)
		{
			/* Clock Data Bits In on +ve Clock Edge LSB First (no Write) */
			buffer_write(0x2a);
			/* LOG_DEBUG("added TDI bits (i %i)", bits_left - 1); */
		}

		buffer_write(bits_left - 2);
		if (type != SCAN_IN)
			buffer_write(buffer[cur_byte]);
	}

	if ((ir_scan && (tap_get_end_state() == TAP_IRSHIFT))
	  || (!ir_scan && (tap_get_end_state() == TAP_DRSHIFT)))
	{
		if (type == SCAN_IO)
		{
			/* Clock Data Bits In and Out LSB First */
			buffer_write(0x3b);
			/* LOG_DEBUG("added TDI bits (io) %i", bits_left - 1); */
		}
		else if (type == SCAN_OUT)
		{
			/* Clock Data Bits Out on -ve Clock Edge LSB First (no Read) */
			buffer_write(0x1b);
			/* LOG_DEBUG("added TDI bits (o)"); */
		}
		else if (type == SCAN_IN)
		{
			/* Clock Data Bits In on +ve Clock Edge LSB First (no Write) */
			buffer_write(0x2a);
			/* LOG_DEBUG("added TDI bits (i %i)", bits_left - 1); */
		}
		buffer_write(0x0);
		buffer_write(last_bit);
	}
	else
	{
		int tms_bits;
		int tms_count;
		uint8_t	mpsse_cmd;

		/* move from Shift-IR/DR to end state */
		if (type != SCAN_OUT)
		{
			/* We always go to the PAUSE state in two step at the end of an IN or IO scan */
			/* This must be coordinated with the bit shifts in ft2232_read_scan    */
			tms_bits  = 0x01;
			tms_count = 2;
			/* Clock Data to TMS/CS Pin with Read */
			mpsse_cmd = 0x6b;
			/* LOG_DEBUG("added TMS scan (read)"); */
		}
		else
		{
			tms_bits  = tap_get_tms_path(tap_get_state(), tap_get_end_state());
			tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());
			/* Clock Data to TMS/CS Pin (no Read) */
			mpsse_cmd = 0x4b;
			/* LOG_DEBUG("added TMS scan (no read)"); */
		}

		clock_tms(mpsse_cmd, tms_bits, tms_count, last_bit);
	}

	if (tap_get_state() != tap_get_end_state())
	{
		move_to_state(tap_get_end_state());
	}
}

static int ft2232_large_scan(scan_command_t* cmd, enum scan_type type, uint8_t* buffer, int scan_size)
{
	int num_bytes = (scan_size + 7) / 8;
	int bits_left = scan_size;
	int cur_byte  = 0;
	int last_bit;
	uint8_t* receive_buffer  = malloc(CEIL(scan_size, 8));
	uint8_t* receive_pointer = receive_buffer;
	uint32_t bytes_written;
	uint32_t bytes_read;
	int retval;
	int thisrun_read = 0;

	if (cmd->ir_scan)
	{
		LOG_ERROR("BUG: large IR scans are not supported");
		exit(-1);
	}

	if (tap_get_state() != TAP_DRSHIFT)
	{
		move_to_state(TAP_DRSHIFT);
	}

	if ((retval = ft2232_write(ft2232_buffer, ft2232_buffer_size, &bytes_written)) != ERROR_OK)
	{
		LOG_ERROR("couldn't write MPSSE commands to FT2232");
		exit(-1);
	}
	LOG_DEBUG("ft2232_buffer_size: %i, bytes_written: %i",
		  ft2232_buffer_size, (int)bytes_written);
	ft2232_buffer_size = 0;

	/* add command for complete bytes */
	while (num_bytes > 1)
	{
		int thisrun_bytes;

		if (type == SCAN_IO)
		{
			/* Clock Data Bytes In and Out LSB First */
			buffer_write(0x39);
			/* LOG_DEBUG("added TDI bytes (io %i)", num_bytes); */
		}
		else if (type == SCAN_OUT)
		{
			/* Clock Data Bytes Out on -ve Clock Edge LSB First (no Read) */
			buffer_write(0x19);
			/* LOG_DEBUG("added TDI bytes (o)"); */
		}
		else if (type == SCAN_IN)
		{
			/* Clock Data Bytes In on +ve Clock Edge LSB First (no Write) */
			buffer_write(0x28);
			/* LOG_DEBUG("added TDI bytes (i %i)", num_bytes); */
		}

		thisrun_bytes = (num_bytes > 65537) ? 65536 : (num_bytes - 1);
		thisrun_read  = thisrun_bytes;
		num_bytes    -= thisrun_bytes;
		buffer_write((uint8_t) (thisrun_bytes - 1));
		buffer_write((uint8_t) ((thisrun_bytes - 1) >> 8));

		if (type != SCAN_IN)
		{
			/* add complete bytes */
			while (thisrun_bytes-- > 0)
			{
				buffer_write(buffer[cur_byte]);
				cur_byte++;
				bits_left -= 8;
			}
		}
		else /* (type == SCAN_IN) */
		{
			bits_left -= 8 * (thisrun_bytes);
		}

		if ((retval = ft2232_write(ft2232_buffer, ft2232_buffer_size, &bytes_written)) != ERROR_OK)
		{
			LOG_ERROR("couldn't write MPSSE commands to FT2232");
			exit(-1);
		}
		LOG_DEBUG("ft2232_buffer_size: %i, bytes_written: %i",
			  ft2232_buffer_size,
			  (int)bytes_written);
		ft2232_buffer_size = 0;

		if (type != SCAN_OUT)
		{
			if ((retval = ft2232_read(receive_pointer, thisrun_read, &bytes_read)) != ERROR_OK)
			{
				LOG_ERROR("couldn't read from FT2232");
				exit(-1);
			}
			LOG_DEBUG("thisrun_read: %i, bytes_read: %i",
				  thisrun_read,
				  (int)bytes_read);
			receive_pointer += bytes_read;
		}
	}

	thisrun_read = 0;

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
			buffer_write(0x3b);
			/* LOG_DEBUG("added TDI bits (io) %i", bits_left - 1); */
		}
		else if (type == SCAN_OUT)
		{
			/* Clock Data Bits Out on -ve Clock Edge LSB First (no Read) */
			buffer_write(0x1b);
			/* LOG_DEBUG("added TDI bits (o)"); */
		}
		else if (type == SCAN_IN)
		{
			/* Clock Data Bits In on +ve Clock Edge LSB First (no Write) */
			buffer_write(0x2a);
			/* LOG_DEBUG("added TDI bits (i %i)", bits_left - 1); */
		}
		buffer_write(bits_left - 2);
		if (type != SCAN_IN)
			buffer_write(buffer[cur_byte]);

		if (type != SCAN_OUT)
			thisrun_read += 2;
	}

	if (tap_get_end_state() == TAP_DRSHIFT)
	{
		if (type == SCAN_IO)
		{
			/* Clock Data Bits In and Out LSB First */
			buffer_write(0x3b);
			/* LOG_DEBUG("added TDI bits (io) %i", bits_left - 1); */
		}
		else if (type == SCAN_OUT)
		{
			/* Clock Data Bits Out on -ve Clock Edge LSB First (no Read) */
			buffer_write(0x1b);
			/* LOG_DEBUG("added TDI bits (o)"); */
		}
		else if (type == SCAN_IN)
		{
			/* Clock Data Bits In on +ve Clock Edge LSB First (no Write) */
			buffer_write(0x2a);
			/* LOG_DEBUG("added TDI bits (i %i)", bits_left - 1); */
		}
		buffer_write(0x0);
		buffer_write(last_bit);
	}
	else
	{
		int tms_bits  = tap_get_tms_path(tap_get_state(), tap_get_end_state());
		int tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());
		uint8_t	mpsse_cmd;

		/* move from Shift-IR/DR to end state */
		if (type != SCAN_OUT)
		{
			/* Clock Data to TMS/CS Pin with Read */
			mpsse_cmd = 0x6b;
			/* LOG_DEBUG("added TMS scan (read)"); */
		}
		else
		{
			/* Clock Data to TMS/CS Pin (no Read) */
			mpsse_cmd = 0x4b;
			/* LOG_DEBUG("added TMS scan (no read)"); */
		}

		clock_tms(mpsse_cmd, tms_bits, tms_count, last_bit);
	}

	if (type != SCAN_OUT)
		thisrun_read += 1;

	if ((retval = ft2232_write(ft2232_buffer, ft2232_buffer_size, &bytes_written)) != ERROR_OK)
	{
		LOG_ERROR("couldn't write MPSSE commands to FT2232");
		exit(-1);
	}
	LOG_DEBUG("ft2232_buffer_size: %i, bytes_written: %i",
		  ft2232_buffer_size,
		  (int)bytes_written);
	ft2232_buffer_size = 0;

	if (type != SCAN_OUT)
	{
		if ((retval = ft2232_read(receive_pointer, thisrun_read, &bytes_read)) != ERROR_OK)
		{
			LOG_ERROR("couldn't read from FT2232");
			exit(-1);
		}
		LOG_DEBUG("thisrun_read: %i, bytes_read: %i",
			  thisrun_read,
			  (int)bytes_read);
		receive_pointer += bytes_read;
	}

	return ERROR_OK;
}

static int ft2232_predict_scan_out(int scan_size, enum scan_type type)
{
	int predicted_size = 3;
	int num_bytes = (scan_size - 1) / 8;

	if (tap_get_state() != TAP_DRSHIFT)
		predicted_size += get_tms_buffer_requirements(tap_get_tms_path_len(tap_get_state(), TAP_DRSHIFT));

	if (type == SCAN_IN)	/* only from device to host */
	{
		/* complete bytes */
		predicted_size += CEIL(num_bytes, 65536) * 3;

		/* remaining bits - 1 (up to 7) */
		predicted_size += ((scan_size - 1) % 8) ? 2 : 0;
	}
	else	/* host to device, or bidirectional */
	{
		/* complete bytes */
		predicted_size += num_bytes + CEIL(num_bytes, 65536) * 3;

		/* remaining bits -1 (up to 7) */
		predicted_size += ((scan_size - 1) % 8) ? 3 : 0;
	}

	return predicted_size;
}

static int ft2232_predict_scan_in(int scan_size, enum scan_type type)
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

	/* LOG_DEBUG("scan_size: %i, predicted_size: %i", scan_size, predicted_size); */

	return predicted_size;
}

static void usbjtag_reset(int trst, int srst)
{
	enum reset_types jtag_reset_config = jtag_get_reset_config();
	if (trst == 1)
	{
		if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
			low_direction |= nTRSTnOE;	/* switch to output pin (output is low) */
		else
			low_output &= ~nTRST;		/* switch output low */
	}
	else if (trst == 0)
	{
		if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
			low_direction &= ~nTRSTnOE;	/* switch to input pin (high-Z + internal and external pullup) */
		else
			low_output |= nTRST;		/* switch output high */
	}

	if (srst == 1)
	{
		if (jtag_reset_config & RESET_SRST_PUSH_PULL)
			low_output &= ~nSRST;		/* switch output low */
		else
			low_direction |= nSRSTnOE;	/* switch to output pin (output is low) */
	}
	else if (srst == 0)
	{
		if (jtag_reset_config & RESET_SRST_PUSH_PULL)
			low_output |= nSRST;		/* switch output high */
		else
			low_direction &= ~nSRSTnOE;	/* switch to input pin (high-Z) */
	}

	/* command "set data bits low byte" */
	buffer_write(0x80);
	buffer_write(low_output);
	buffer_write(low_direction);
}

static void jtagkey_reset(int trst, int srst)
{
	enum reset_types jtag_reset_config = jtag_get_reset_config();
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
	buffer_write(0x82);
	buffer_write(high_output);
	buffer_write(high_direction);
	LOG_DEBUG("trst: %i, srst: %i, high_output: 0x%2.2x, high_direction: 0x%2.2x", trst, srst, high_output,
			high_direction);
}

static void olimex_jtag_reset(int trst, int srst)
{
	enum reset_types jtag_reset_config = jtag_get_reset_config();
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
	buffer_write(0x82);
	buffer_write(high_output);
	buffer_write(high_direction);
	LOG_DEBUG("trst: %i, srst: %i, high_output: 0x%2.2x, high_direction: 0x%2.2x", trst, srst, high_output,
			high_direction);
}

static void axm0432_jtag_reset(int trst, int srst)
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
	buffer_write(0x82);
	buffer_write(high_output);
	buffer_write(high_direction);
	LOG_DEBUG("trst: %i, srst: %i, high_output: 0x%2.2x, high_direction: 0x%2.2x", trst, srst, high_output,
			high_direction);
}

static void flyswatter_reset(int trst, int srst)
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
	buffer_write(0x80);
	buffer_write(low_output);
	buffer_write(low_direction);
	LOG_DEBUG("trst: %i, srst: %i, low_output: 0x%2.2x, low_direction: 0x%2.2x", trst, srst, low_output, low_direction);
}

static void turtle_reset(int trst, int srst)
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
	buffer_write(0x80);
	buffer_write(low_output);
	buffer_write(low_direction);
	LOG_DEBUG("srst: %i, low_output: 0x%2.2x, low_direction: 0x%2.2x", srst, low_output, low_direction);
}

static void comstick_reset(int trst, int srst)
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
	buffer_write(0x82);
	buffer_write(high_output);
	buffer_write(high_direction);
	LOG_DEBUG("trst: %i, srst: %i, high_output: 0x%2.2x, high_direction: 0x%2.2x", trst, srst, high_output,
			high_direction);
}

static void stm32stick_reset(int trst, int srst)
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
	buffer_write(0x80);
	buffer_write(low_output);
	buffer_write(low_direction);

	/* command "set data bits high byte" */
	buffer_write(0x82);
	buffer_write(high_output);
	buffer_write(high_direction);
	LOG_DEBUG("trst: %i, srst: %i, high_output: 0x%2.2x, high_direction: 0x%2.2x", trst, srst, high_output,
			high_direction);
}

static void sheevaplug_reset(int trst, int srst)
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
	buffer_write(0x82);
	buffer_write(high_output);
	buffer_write(high_direction);
	LOG_DEBUG("trst: %i, srst: %i, high_output: 0x%2.2x, high_direction: 0x%2.2x", trst, srst, high_output, high_direction);
}

static int ft2232_execute_runtest(jtag_command_t *cmd)
{
	int retval;
	int i;
	int predicted_size = 0;
	retval = ERROR_OK;

	DEBUG_JTAG_IO("runtest %i cycles, end in %s",
			cmd->cmd.runtest->num_cycles,
			tap_state_name(cmd->cmd.runtest->end_state));

	/* only send the maximum buffer size that FT2232C can handle */
	predicted_size = 0;
	if (tap_get_state() != TAP_IDLE)
		predicted_size += 3;
	predicted_size += 3 * CEIL(cmd->cmd.runtest->num_cycles, 7);
	if (cmd->cmd.runtest->end_state != TAP_IDLE)
		predicted_size += 3;
	if (tap_get_end_state() != TAP_IDLE)
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
		move_to_state(TAP_IDLE);
		require_send = 1;
	}
	i = cmd->cmd.runtest->num_cycles;
	while (i > 0)
	{
		/* there are no state transitions in this code, so omit state tracking */

		/* command "Clock Data to TMS/CS Pin (no Read)" */
		buffer_write(0x4b);

		/* scan 7 bits */
		buffer_write((i > 7) ? 6 : (i - 1));

		/* TMS data bits */
		buffer_write(0x0);
		tap_set_state(TAP_IDLE);

		i -= (i > 7) ? 7 : i;
		/* LOG_DEBUG("added TMS scan (no read)"); */
	}

	ft2232_end_state(cmd->cmd.runtest->end_state);

	if (tap_get_state() != tap_get_end_state())
	{
		move_to_state(tap_get_end_state());
	}

	require_send = 1;
#ifdef _DEBUG_JTAG_IO_
	LOG_DEBUG("runtest: %i, end in %s", cmd->cmd.runtest->num_cycles, tap_state_name(tap_get_end_state()));
#endif

	return retval;
}

static int ft2232_execute_statemove(jtag_command_t *cmd)
{
	int	predicted_size = 0;
	int	retval = ERROR_OK;

	DEBUG_JTAG_IO("statemove end in %i", cmd->cmd.statemove->end_state);

	/* only send the maximum buffer size that FT2232C can handle */
	predicted_size = 3;
	if (ft2232_buffer_size + predicted_size + 1 > FT2232_BUFFER_SIZE)
	{
		if (ft2232_send_and_recv(first_unsent, cmd) != ERROR_OK)
			retval = ERROR_JTAG_QUEUE_FAILED;
		require_send = 0;
		first_unsent = cmd;
	}
	ft2232_end_state(cmd->cmd.statemove->end_state);

	/* For TAP_RESET, ignore the current recorded state.  It's often
	 * wrong at server startup, and this transation is critical whenever
	 * it's requested.
	 */
	if (tap_get_end_state() == TAP_RESET) {
		clock_tms(0x4b,  0xff, 5, 0);
		require_send = 1;

	/* shortest-path move to desired end state */
	} else if (tap_get_state() != tap_get_end_state())
	{
		move_to_state(tap_get_end_state());
		require_send = 1;
	}

	return retval;
}

static int ft2232_execute_pathmove(jtag_command_t *cmd)
{
	int	predicted_size = 0;
	int	retval = ERROR_OK;

	tap_state_t*	 path = cmd->cmd.pathmove->path;
	int	num_states    = cmd->cmd.pathmove->num_states;

	DEBUG_JTAG_IO("pathmove: %i states, current: %s  end: %s", num_states,
			tap_state_name(tap_get_state()),
			tap_state_name(path[num_states-1]));

	/* only send the maximum buffer size that FT2232C can handle */
	predicted_size = 3 * CEIL(num_states, 7);
	if (ft2232_buffer_size + predicted_size + 1 > FT2232_BUFFER_SIZE)
	{
		if (ft2232_send_and_recv(first_unsent, cmd) != ERROR_OK)
			retval = ERROR_JTAG_QUEUE_FAILED;

		require_send = 0;
		first_unsent = cmd;
	}

	ft2232_add_pathmove(path, num_states);
	require_send = 1;

	return retval;
}

static int ft2232_execute_scan(jtag_command_t *cmd)
{
	uint8_t* buffer;
	int scan_size;				/* size of IR or DR scan */
	int predicted_size = 0;
	int retval = ERROR_OK;

	enum scan_type  type = jtag_scan_type(cmd->cmd.scan);

	DEBUG_JTAG_IO("%s type:%d", cmd->cmd.scan->ir_scan ? "IRSCAN" : "DRSCAN", type);

	scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);

	predicted_size = ft2232_predict_scan_out(scan_size, type);
	if ((predicted_size + 1) > FT2232_BUFFER_SIZE)
	{
		LOG_DEBUG("oversized ft2232 scan (predicted_size > FT2232_BUFFER_SIZE)");
		/* unsent commands before this */
		if (first_unsent != cmd)
			if (ft2232_send_and_recv(first_unsent, cmd) != ERROR_OK)
				retval = ERROR_JTAG_QUEUE_FAILED;

		/* current command */
		ft2232_end_state(cmd->cmd.scan->end_state);
		ft2232_large_scan(cmd->cmd.scan, type, buffer, scan_size);
		require_send = 0;
		first_unsent = cmd->next;
		if (buffer)
			free(buffer);
		return retval;
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
	ft2232_end_state(cmd->cmd.scan->end_state);
	ft2232_add_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size);
	require_send = 1;
	if (buffer)
		free(buffer);
#ifdef _DEBUG_JTAG_IO_
	LOG_DEBUG("%s scan, %i bits, end in %s", (cmd->cmd.scan->ir_scan) ? "IR" : "DR", scan_size,
			tap_state_name(tap_get_end_state()));
#endif
	return retval;

}

static int ft2232_execute_reset(jtag_command_t *cmd)
{
	int retval;
	int predicted_size = 0;
	retval = ERROR_OK;

	DEBUG_JTAG_IO("reset trst: %i srst %i",
			cmd->cmd.reset->trst, cmd->cmd.reset->srst);

	/* only send the maximum buffer size that FT2232C can handle */
	predicted_size = 3;
	if (ft2232_buffer_size + predicted_size + 1 > FT2232_BUFFER_SIZE)
	{
		if (ft2232_send_and_recv(first_unsent, cmd) != ERROR_OK)
			retval = ERROR_JTAG_QUEUE_FAILED;
		require_send = 0;
		first_unsent = cmd;
	}

	if ((cmd->cmd.reset->trst == 1) || (cmd->cmd.reset->srst && (jtag_get_reset_config() & RESET_SRST_PULLS_TRST)))
	{
		tap_set_state(TAP_RESET);
	}

	layout->reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
	require_send = 1;

#ifdef _DEBUG_JTAG_IO_
	LOG_DEBUG("trst: %i, srst: %i", cmd->cmd.reset->trst, cmd->cmd.reset->srst);
#endif
	return retval;
}

static int ft2232_execute_sleep(jtag_command_t *cmd)
{
	int retval;
	retval = ERROR_OK;

	DEBUG_JTAG_IO("sleep %i", cmd->cmd.sleep->us);

	if (ft2232_send_and_recv(first_unsent, cmd) != ERROR_OK)
				retval = ERROR_JTAG_QUEUE_FAILED;
	first_unsent = cmd->next;
	jtag_sleep(cmd->cmd.sleep->us);
#ifdef _DEBUG_JTAG_IO_
			LOG_DEBUG("sleep %i usec while in %s", cmd->cmd.sleep->us, tap_state_name(tap_get_state()));
#endif

	return retval;
}

static int ft2232_execute_stableclocks(jtag_command_t *cmd)
{
	int retval;
	retval = ERROR_OK;

	/* this is only allowed while in a stable state.  A check for a stable
	 * state was done in jtag_add_clocks()
	 */
	if (ft2232_stableclocks(cmd->cmd.stableclocks->num_cycles, cmd) != ERROR_OK)
		retval = ERROR_JTAG_QUEUE_FAILED;
#ifdef _DEBUG_JTAG_IO_
	LOG_DEBUG("clocks %i while in %s", cmd->cmd.stableclocks->num_cycles, tap_state_name(tap_get_state()));
#endif

	return retval;
}

static int ft2232_execute_command(jtag_command_t *cmd)
{
	int retval;
	retval = ERROR_OK;

	switch (cmd->type)
	{
	case JTAG_RESET:	retval = ft2232_execute_reset(cmd); break;
	case JTAG_RUNTEST:	retval = ft2232_execute_runtest(cmd); break;
	case JTAG_STATEMOVE: retval = ft2232_execute_statemove(cmd); break;
	case JTAG_PATHMOVE:	retval = ft2232_execute_pathmove(cmd); break;
	case JTAG_SCAN:		retval = ft2232_execute_scan(cmd); break;
	case JTAG_SLEEP:	retval = ft2232_execute_sleep(cmd); break;
	case JTAG_STABLECLOCKS:	retval = ft2232_execute_stableclocks(cmd); break;
	default:
		LOG_ERROR("BUG: unknown JTAG command type encountered");
		exit(-1);
	}
	return retval;
}

static int ft2232_execute_queue()
{
	jtag_command_t* cmd = jtag_command_queue;	/* currently processed command */
	int retval;

	first_unsent = cmd;		/* next command that has to be sent */
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
		if (ft2232_execute_command(cmd) != ERROR_OK)
			retval = ERROR_JTAG_QUEUE_FAILED;
		/* Start reading input before FT2232 TX buffer fills up */
		cmd = cmd->next;
		if (ft2232_expect_read > 256)
		{
			if (ft2232_send_and_recv(first_unsent, cmd) != ERROR_OK)
				retval = ERROR_JTAG_QUEUE_FAILED;
			first_unsent = cmd;
		}
	}

	if (require_send > 0)
		if (ft2232_send_and_recv(first_unsent, cmd) != ERROR_OK)
			retval = ERROR_JTAG_QUEUE_FAILED;

	return retval;
}

#if BUILD_FT2232_FTD2XX == 1
static int ft2232_init_ftd2xx(uint16_t vid, uint16_t pid, int more, int* try_more)
{
	FT_STATUS	status;
	DWORD		deviceID;
	char		SerialNumber[16];
	char		Description[64];
	DWORD	openex_flags  = 0;
	char*	openex_string = NULL;
	uint8_t	latency_timer;

	LOG_DEBUG("'ft2232' interface using FTD2XX with '%s' layout (%4.4x:%4.4x)", ft2232_layout, vid, pid);

#if IS_WIN32 == 0
	/* Add non-standard Vid/Pid to the linux driver */
	if ((status = FT_SetVIDPID(vid, pid)) != FT_OK)
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
	if (status != FT_OK) {
		/* under Win32, the FTD2XX driver appends an "A" to the end
		 * of the description, if we tried by the desc, then
		 * try by the alternate "A" description. */
		if (openex_string == ft2232_device_desc) {
			/* Try the alternate method. */
			openex_string = ft2232_device_desc_A;
			status = FT_OpenEx(openex_string, openex_flags, &ftdih);
			if (status == FT_OK) {
				/* yea, the "alternate" method worked! */
			} else {
				/* drat, give the user a meaningfull message.
				 * telling the use we tried *BOTH* methods. */
				LOG_WARNING("Unable to open FTDI Device tried: '%s' and '%s'\n",
							ft2232_device_desc,
							ft2232_device_desc_A);
			}
		}
	}

	if (status != FT_OK)
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
			char** desc_array = malloc(sizeof(char*) * (num_devices + 1));
			uint32_t i;

			for (i = 0; i < num_devices; i++)
				desc_array[i] = malloc(64);

			desc_array[num_devices] = NULL;

			status = FT_ListDevices(desc_array, &num_devices, FT_LIST_ALL | openex_flags);

			if (status == FT_OK)
			{
				LOG_ERROR("ListDevices: %lu\n", num_devices);
				for (i = 0; i < num_devices; i++)
					LOG_ERROR("%" PRIu32 ": \"%s\"", i, desc_array[i]);
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

	if ((status = FT_SetLatencyTimer(ftdih, ft2232_latency)) != FT_OK)
	{
		LOG_ERROR("unable to set latency timer: %lu", status);
		return ERROR_JTAG_INIT_FAILED;
	}

	if ((status = FT_GetLatencyTimer(ftdih, &latency_timer)) != FT_OK)
	{
		LOG_ERROR("unable to get latency timer: %lu", status);
		return ERROR_JTAG_INIT_FAILED;
	}
	else
	{
		LOG_DEBUG("current latency timer: %i", latency_timer);
	}

	if ((status = FT_SetTimeouts(ftdih, 5000, 5000)) != FT_OK)
	{
		LOG_ERROR("unable to set timeouts: %lu", status);
		return ERROR_JTAG_INIT_FAILED;
	}

	if ((status = FT_SetBitMode(ftdih, 0x0b, 2)) != FT_OK)
	{
		LOG_ERROR("unable to enable bit i/o mode: %lu", status);
		return ERROR_JTAG_INIT_FAILED;
	}

	if ((status = FT_GetDeviceInfo(ftdih, &ftdi_device, &deviceID, SerialNumber, Description, NULL)) != FT_OK)
	{
		LOG_ERROR("unable to get FT_GetDeviceInfo: %lu", status);
		return ERROR_JTAG_INIT_FAILED;
	}
	else
	{
		static const char* type_str[] =
			{"BM", "AM", "100AX", "UNKNOWN", "2232C", "232R", "2232H", "4232H"};
		unsigned no_of_known_types = sizeof(type_str) / sizeof(type_str[0]) - 1;
		unsigned type_index = ((unsigned)ftdi_device <= no_of_known_types)
			? ftdi_device : FT_DEVICE_UNKNOWN;
		LOG_INFO("device: %lu \"%s\"", ftdi_device, type_str[type_index]);
		LOG_INFO("deviceID: %lu", deviceID);
		LOG_INFO("SerialNumber: %s", SerialNumber);
		LOG_INFO("Description: %s", Description);
	}

	return ERROR_OK;
}

static int ft2232_purge_ftd2xx(void)
{
	FT_STATUS status;

	if ((status = FT_Purge(ftdih, FT_PURGE_RX | FT_PURGE_TX)) != FT_OK)
	{
		LOG_ERROR("error purging ftd2xx device: %lu", status);
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

#endif /* BUILD_FT2232_FTD2XX == 1 */

#if BUILD_FT2232_LIBFTDI == 1
static int ft2232_init_libftdi(uint16_t vid, uint16_t pid, int more, int* try_more)
{
	uint8_t latency_timer;

	LOG_DEBUG("'ft2232' interface using libftdi with '%s' layout (%4.4x:%4.4x)",
			ft2232_layout, vid, pid);

	if (ftdi_init(&ftdic) < 0)
		return ERROR_JTAG_INIT_FAILED;

	if (ftdi_set_interface(&ftdic, INTERFACE_A) < 0)
	{
		LOG_ERROR("unable to select FT2232 channel A: %s", ftdic.error_str);
		return ERROR_JTAG_INIT_FAILED;
	}

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

	/* There is already a reset in ftdi_usb_open_desc, this should be redundant */
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

	ftdi_device = ftdic.type;
	static const char* type_str[] =
		{"AM", "BM", "2232C", "R", "2232H", "4232H", "Unknown"};
	unsigned no_of_known_types = sizeof(type_str) / sizeof(type_str[0]) - 1;
	unsigned type_index = ((unsigned)ftdi_device < no_of_known_types)
		? ftdi_device : no_of_known_types;
	LOG_DEBUG("FTDI chip type: %i \"%s\"", (int)ftdi_device, type_str[type_index]);
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

static int ft2232_init(void)
{
	uint8_t  buf[1];
	int retval;
	uint32_t bytes_written;
	const ft2232_layout_t* cur_layout = ft2232_layouts;
	int i;

	if (tap_get_tms_path_len(TAP_IRPAUSE,TAP_IRPAUSE) == 7)
	{
		LOG_DEBUG("ft2232 interface using 7 step jtag state transitions");
	}
	else
	{
		LOG_DEBUG("ft2232 interface using shortest path jtag state transitions");

	}
	if ((ft2232_layout == NULL) || (ft2232_layout[0] == 0))
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

	if (ft2232_device_is_highspeed())
	{
#ifndef BUILD_FT2232_HIGHSPEED
 #if BUILD_FT2232_FTD2XX == 1
		LOG_WARNING("High Speed device found - You need a newer FTD2XX driver (version 2.04.16 or later)");
 #elif BUILD_FT2232_LIBFTDI == 1
		LOG_WARNING("High Speed device found - You need a newer libftdi version (0.16 or later)");
 #endif
#endif
		/* make sure the legacy mode is disabled */
		if (ft2232h_ft4232h_clk_divide_by_5(false) != ERROR_OK)
			return ERROR_JTAG_INIT_FAILED;
	}

	ft2232_speed(jtag_get_speed());

	buf[0] = 0x85; /* Disconnect TDI/DO to TDO/DI for Loopback */
	if (((retval = ft2232_write(buf, 1, &bytes_written)) != ERROR_OK) || (bytes_written != 1))
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

static int usbjtag_init(void)
{
	uint8_t  buf[3];
	uint32_t bytes_written;

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
	else if (strcmp(ft2232_layout, "luminary_icdi") == 0)
	{
		nTRST = 0x0;
		nTRSTnOE = 0x00;
		nSRST = 0x20;
		nSRSTnOE = 0x20;
		low_output    = 0x88;
		low_direction = 0xcb;
	}
	else
	{
		LOG_ERROR("BUG: usbjtag_init called for unknown layout '%s'", ft2232_layout);
		return ERROR_JTAG_INIT_FAILED;
	}

	enum reset_types jtag_reset_config = jtag_get_reset_config();
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
	buf[1] = low_output;    /* value (TMS = 1,TCK = 0, TDI = 0, xRST high) */
	buf[2] = low_direction; /* dir (output = 1), TCK/TDI/TMS = out, TDO = in */
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if (((ft2232_write(buf, 3, &bytes_written)) != ERROR_OK) || (bytes_written != 3))
	{
		LOG_ERROR("couldn't initialize FT2232 with 'USBJTAG' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int axm0432_jtag_init(void)
{
	uint8_t  buf[3];
	uint32_t bytes_written;

	low_output    = 0x08;
	low_direction = 0x2b;

	/* initialize low byte for jtag */
	buf[0] = 0x80;          /* command "set data bits low byte" */
	buf[1] = low_output;    /* value (TMS = 1,TCK = 0, TDI = 0, nOE = 0) */
	buf[2] = low_direction; /* dir (output = 1), TCK/TDI/TMS = out, TDO = in, nOE = out */
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if (((ft2232_write(buf, 3, &bytes_written)) != ERROR_OK) || (bytes_written != 3))
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

	enum reset_types jtag_reset_config = jtag_get_reset_config();
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

	if (((ft2232_write(buf, 3, &bytes_written)) != ERROR_OK) || (bytes_written != 3))
	{
		LOG_ERROR("couldn't initialize FT2232 with 'Dicarlo' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int jtagkey_init(void)
{
	uint8_t  buf[3];
	uint32_t bytes_written;

	low_output    = 0x08;
	low_direction = 0x1b;

	/* initialize low byte for jtag */
	buf[0] = 0x80;          /* command "set data bits low byte" */
	buf[1] = low_output;    /* value (TMS = 1,TCK = 0, TDI = 0, nOE = 0) */
	buf[2] = low_direction; /* dir (output = 1), TCK/TDI/TMS = out, TDO = in, nOE = out */
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if (((ft2232_write(buf, 3, &bytes_written)) != ERROR_OK) || (bytes_written != 3))
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
	else if ((strcmp(layout->name, "jtagkey_prototype_v1") == 0)
			 || (strcmp(layout->name, "oocdlink") == 0))
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

	enum reset_types jtag_reset_config = jtag_get_reset_config();
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

	if (((ft2232_write(buf, 3, &bytes_written)) != ERROR_OK) || (bytes_written != 3))
	{
		LOG_ERROR("couldn't initialize FT2232 with 'JTAGkey' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int olimex_jtag_init(void)
{
	uint8_t  buf[3];
	uint32_t bytes_written;

	low_output    = 0x08;
	low_direction = 0x1b;

	/* initialize low byte for jtag */
	buf[0] = 0x80;          /* command "set data bits low byte" */
	buf[1] = low_output;    /* value (TMS = 1,TCK = 0, TDI = 0, nOE = 0) */
	buf[2] = low_direction; /* dir (output = 1), TCK/TDI/TMS = out, TDO = in, nOE = out */
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if (((ft2232_write(buf, 3, &bytes_written)) != ERROR_OK) || (bytes_written != 3))
	{
		LOG_ERROR("couldn't initialize FT2232 with 'Olimex' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	nTRST    = 0x01;
	nTRSTnOE = 0x4;
	nSRST    = 0x02;
	nSRSTnOE = 0x00; /* no output enable for nSRST */

	high_output    = 0x0;
	high_direction = 0x0f;

	enum reset_types jtag_reset_config = jtag_get_reset_config();
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

	if ((ft2232_write(buf, 3, &bytes_written) != ERROR_OK) || (bytes_written != 3))
	{
		LOG_ERROR("couldn't initialize FT2232 with 'Olimex' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int flyswatter_init(void)
{
	uint8_t  buf[3];
	uint32_t bytes_written;

	low_output    = 0x18;
	low_direction = 0xfb;

	/* initialize low byte for jtag */
	buf[0] = 0x80;          /* command "set data bits low byte" */
	buf[1] = low_output;    /* value (TMS = 1,TCK = 0, TDI = 0, nOE = 0) */
	buf[2] = low_direction; /* dir (output = 1), TCK/TDI/TMS = out, TDO = in, nOE[12]=out, n[ST]srst = out */
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if (((ft2232_write(buf, 3, &bytes_written)) != ERROR_OK) || (bytes_written != 3))
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

	if (((ft2232_write(buf, 3, &bytes_written)) != ERROR_OK) || (bytes_written != 3))
	{
		LOG_ERROR("couldn't initialize FT2232 with 'flyswatter' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int turtle_init(void)
{
	uint8_t  buf[3];
	uint32_t bytes_written;

	low_output    = 0x08;
	low_direction = 0x5b;

	/* initialize low byte for jtag */
	buf[0] = 0x80;          /* command "set data bits low byte" */
	buf[1] = low_output;    /* value (TMS = 1,TCK = 0, TDI = 0, nOE = 0) */
	buf[2] = low_direction; /* dir (output = 1), TCK/TDI/TMS = out, TDO = in, nOE = out */
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if (((ft2232_write(buf, 3, &bytes_written)) != ERROR_OK) || (bytes_written != 3))
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

	if (((ft2232_write(buf, 3, &bytes_written)) != ERROR_OK) || (bytes_written != 3))
	{
		LOG_ERROR("couldn't initialize FT2232 with 'turtelizer2' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int comstick_init(void)
{
	uint8_t  buf[3];
	uint32_t bytes_written;

	low_output    = 0x08;
	low_direction = 0x0b;

	/* initialize low byte for jtag */
	buf[0] = 0x80;          /* command "set data bits low byte" */
	buf[1] = low_output;    /* value (TMS = 1,TCK = 0, TDI = 0, nOE = 0) */
	buf[2] = low_direction; /* dir (output = 1), TCK/TDI/TMS = out, TDO = in, nOE = out */
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if (((ft2232_write(buf, 3, &bytes_written)) != ERROR_OK) || (bytes_written != 3))
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

	if (((ft2232_write(buf, 3, &bytes_written)) != ERROR_OK) || (bytes_written != 3))
	{
		LOG_ERROR("couldn't initialize FT2232 with 'comstick' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int stm32stick_init(void)
{
	uint8_t  buf[3];
	uint32_t bytes_written;

	low_output    = 0x88;
	low_direction = 0x8b;

	/* initialize low byte for jtag */
	buf[0] = 0x80;          /* command "set data bits low byte" */
	buf[1] = low_output;    /* value (TMS = 1,TCK = 0, TDI = 0, nOE = 0) */
	buf[2] = low_direction; /* dir (output = 1), TCK/TDI/TMS = out, TDO = in, nOE = out */
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if (((ft2232_write(buf, 3, &bytes_written)) != ERROR_OK) || (bytes_written != 3))
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

	if (((ft2232_write(buf, 3, &bytes_written)) != ERROR_OK) || (bytes_written != 3))
	{
		LOG_ERROR("couldn't initialize FT2232 with 'stm32stick' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int sheevaplug_init(void)
{
	uint8_t buf[3];
	uint32_t bytes_written;

	low_output = 0x08;
	low_direction = 0x1b;

	/* initialize low byte for jtag */
	buf[0] = 0x80; /* command "set data bits low byte" */
	buf[1] = low_output; /* value (TMS = 1,TCK = 0, TDI = 0, nOE = 0) */
	buf[2] = low_direction; /* dir (output = 1), TCK/TDI/TMS = out, TDO = in */
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

static int cortino_jtag_init(void)
{
	uint8_t  buf[3];
	uint32_t bytes_written;

	low_output    = 0x08;
	low_direction = 0x1b;

	/* initialize low byte for jtag */
	buf[0] = 0x80;          /* command "set data bits low byte" */
	buf[1] = low_output;    /* value (TMS = 1,TCK = 0, TDI = 0, nOE = 0) */
	buf[2] = low_direction; /* dir (output = 1), TCK/TDI/TMS = out, TDO = in, nOE = out */
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if (((ft2232_write(buf, 3, &bytes_written)) != ERROR_OK) || (bytes_written != 3))
	{
		LOG_ERROR("couldn't initialize FT2232 with 'cortino' layout");
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

	if (((ft2232_write(buf, 3, &bytes_written)) != ERROR_OK) || (bytes_written != 3))
	{
		LOG_ERROR("couldn't initialize FT2232 with 'stm32stick' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static void olimex_jtag_blink(void)
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

	buffer_write(0x82);
	buffer_write(high_output);
	buffer_write(high_direction);
}

static void flyswatter_jtag_blink(void)
{
	/*
	 * Flyswatter has two LEDs connected to ACBUS2 and ACBUS3
	 */
	high_output ^= 0x0c;

	buffer_write(0x82);
	buffer_write(high_output);
	buffer_write(high_direction);
}

static void turtle_jtag_blink(void)
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

	buffer_write(0x82);
	buffer_write(high_output);
	buffer_write(high_direction);
}

static int ft2232_quit(void)
{
#if BUILD_FT2232_FTD2XX == 1
	FT_STATUS status;

	status = FT_Close(ftdih);
#elif BUILD_FT2232_LIBFTDI == 1
	ftdi_usb_close(&ftdic);

	ftdi_deinit(&ftdic);
#endif

	free(ft2232_buffer);
	ft2232_buffer = NULL;

	return ERROR_OK;
}

static int ft2232_handle_device_desc_command(struct command_context_s* cmd_ctx, char* cmd, char** args, int argc)
{
	char *cp;
	char buf[200];
	if (argc == 1)
	{
		ft2232_device_desc = strdup(args[0]);
		cp = strchr(ft2232_device_desc, 0);
		/* under Win32, the FTD2XX driver appends an "A" to the end
		 * of the description, this examines the given desc
		 * and creates the 'missing' _A or non_A variable. */
		if ((cp[-1] == 'A') && (cp[-2]==' ')) {
			/* it was, so make this the "A" version. */
			ft2232_device_desc_A = ft2232_device_desc;
			/* and *CREATE* the non-A version. */
			strcpy(buf, ft2232_device_desc);
			cp = strchr(buf, 0);
			cp[-2] = 0;
			ft2232_device_desc =  strdup(buf);
		} else {
			/* <space > A not defined
			 * so create it */
			sprintf(buf, "%s A", ft2232_device_desc);
			ft2232_device_desc_A = strdup(buf);
		}
	}
	else
	{
		LOG_ERROR("expected exactly one argument to ft2232_device_desc <description>");
	}

	return ERROR_OK;
}

static int ft2232_handle_serial_command(struct command_context_s* cmd_ctx, char* cmd, char** args, int argc)
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

static int ft2232_handle_layout_command(struct command_context_s* cmd_ctx, char* cmd, char** args, int argc)
{
	if (argc == 0)
		return ERROR_OK;

	ft2232_layout = malloc(strlen(args[0]) + 1);
	strcpy(ft2232_layout, args[0]);

	return ERROR_OK;
}

static int ft2232_handle_vid_pid_command(struct command_context_s* cmd_ctx, char* cmd, char** args, int argc)
{
	if (argc > MAX_USB_IDS * 2)
	{
		LOG_WARNING("ignoring extra IDs in ft2232_vid_pid "
					"(maximum is %d pairs)", MAX_USB_IDS);
		argc = MAX_USB_IDS * 2;
	}
	if (argc < 2 || (argc & 1))
	{
		LOG_WARNING("incomplete ft2232_vid_pid configuration directive");
		if (argc < 2)
			return ERROR_COMMAND_SYNTAX_ERROR;
		/* remove the incomplete trailing id */
		argc -= 1;
	}

	int i;
	int retval = ERROR_OK;
	for (i = 0; i < argc; i += 2)
	{
		retval = parse_u16(args[i], &ft2232_vid[i >> 1]);
		if (ERROR_OK != retval)
			break;
		retval = parse_u16(args[i + 1], &ft2232_pid[i >> 1]);
		if (ERROR_OK != retval)
			break;
	}

	/*
	 * Explicitly terminate, in case there are multiples instances of
	 * ft2232_vid_pid.
	 */
	ft2232_vid[i >> 1] = ft2232_pid[i >> 1] = 0;

	return retval;
}

static int ft2232_handle_latency_command(struct command_context_s* cmd_ctx, char* cmd, char** args, int argc)
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
	uint8_t  tms = (tap_get_state() == TAP_RESET ? 0x7F : 0x00);

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

		/* there are no state transitions in this code, so omit state tracking */

		/* command "Clock Data to TMS/CS Pin (no Read)" */
		buffer_write(0x4b);

		/* scan 7 bit */
		buffer_write(bitcount_per_command - 1);

		/* TMS data bits are either all zeros or ones to stay in the current stable state */
		buffer_write(tms);

		require_send = 1;

		num_cycles -= bitcount_per_command;
	}

	return retval;
}

/* ---------------------------------------------------------------------
 * Support for IceBear JTAG adapter from Section5:
 *	http://section5.ch/icebear
 *
 * Author: Sten, debian@sansys-electronic.com
 */

/* Icebear pin layout
 *
 * ADBUS5 (nEMU) nSRST	| 2   1|	GND (10k->VCC)
 * GND GND		| 4   3|	n.c.
 * ADBUS3 TMS		| 6   5|	ADBUS6 VCC
 * ADBUS0 TCK		| 8   7|	ADBUS7 (GND)
 * ADBUS4 nTRST		|10   9|	ACBUS0 (GND)
 * ADBUS1 TDI		|12  11|	ACBUS1 (GND)
 * ADBUS2 TDO		|14  13|	GND GND
 *
 * ADBUS0 O L TCK		ACBUS0 GND
 * ADBUS1 O L TDI		ACBUS1 GND
 * ADBUS2 I   TDO		ACBUS2 n.c.
 * ADBUS3 O H TMS		ACBUS3 n.c.
 * ADBUS4 O H nTRST
 * ADBUS5 O H nSRST
 * ADBUS6 -   VCC
 * ADBUS7 -   GND
 */
static int icebear_jtag_init(void) {
	uint8_t  buf[3];
	uint32_t bytes_written;

	low_direction	= 0x0b;	/* output: TCK TDI TMS; input: TDO */
	low_output	= 0x08;	/* high: TMS; low: TCK TDI */
	nTRST		= 0x10;
	nSRST		= 0x20;

	enum reset_types jtag_reset_config = jtag_get_reset_config();
	if ((jtag_reset_config & RESET_TRST_OPEN_DRAIN) != 0) {
		low_direction	&= ~nTRST;	/* nTRST high impedance */
	}
	else {
		low_direction	|= nTRST;
		low_output	|= nTRST;
	}

	low_direction	|= nSRST;
	low_output	|= nSRST;

	/* initialize low byte for jtag */
	buf[0] = 0x80;          /* command "set data bits low byte" */
	buf[1] = low_output;
	buf[2] = low_direction;
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if (((ft2232_write(buf, 3, &bytes_written)) != ERROR_OK) || (bytes_written != 3)) {
		LOG_ERROR("couldn't initialize FT2232 with 'IceBear' layout (low)");
		return ERROR_JTAG_INIT_FAILED;
	}

	high_output    = 0x0;
	high_direction = 0x00;


	/* initialize high port */
	buf[0] = 0x82;              /* command "set data bits high byte" */
	buf[1] = high_output;       /* value */
	buf[2] = high_direction;    /* all outputs (xRST and xRSTnOE) */
	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if (((ft2232_write(buf, 3, &bytes_written)) != ERROR_OK) || (bytes_written != 3)) {
		LOG_ERROR("couldn't initialize FT2232 with 'IceBear' layout (high)");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static void icebear_jtag_reset(int trst, int srst) {

	if (trst == 1) {
		low_direction	|= nTRST;
		low_output	&= ~nTRST;
	}
	else if (trst == 0) {
		enum reset_types jtag_reset_config = jtag_get_reset_config();
		if ((jtag_reset_config & RESET_TRST_OPEN_DRAIN) != 0)
			low_direction	&= ~nTRST;
		else
			low_output	|= nTRST;
	}

	if (srst == 1) {
		low_output &= ~nSRST;
	}
	else if (srst == 0) {
		low_output |= nSRST;
	}

	/* command "set data bits low byte" */
	buffer_write(0x80);
	buffer_write(low_output);
	buffer_write(low_direction);

	LOG_DEBUG("trst: %i, srst: %i, low_output: 0x%2.2x, low_direction: 0x%2.2x", trst, srst, low_output, low_direction);
}
