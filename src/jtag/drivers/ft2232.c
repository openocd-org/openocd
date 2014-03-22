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
*   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
***************************************************************************/

/**
 * @file
 * JTAG adapters based on the FT2232 full and high speed USB parts are
 * popular low cost JTAG debug solutions.  Many FT2232 based JTAG adapters
 * are discrete, but development boards may integrate them as alternatives
 * to more capable (and expensive) third party JTAG pods.
 *
 * JTAG uses only one of the two communications channels ("MPSSE engines")
 * on these devices.  Adapters based on FT4232 parts have four ports/channels
 * (A/B/C/D), instead of just two (A/B).
 *
 * Especially on development boards integrating one of these chips (as
 * opposed to discrete pods/dongles), the additional channels can be used
 * for a variety of purposes, but OpenOCD only uses one channel at a time.
 *
 *  - As a USB-to-serial adapter for the target's console UART ...
 *    which may be able to support ROM boot loaders that load initial
 *    firmware images to flash (or SRAM).
 *
 *  - On systems which support ARM's SWD in addition to JTAG, or instead
 *    of it, that second port can be used for reading SWV/SWO trace data.
 *
 *  - Additional JTAG links, e.g. to a CPLD or * FPGA.
 *
 * FT2232 based JTAG adapters are "dumb" not "smart", because most JTAG
 * request/response interactions involve round trips over the USB link.
 * A "smart" JTAG adapter has intelligence close to the scan chain, so it
 * can for example poll quickly for a status change (usually taking on the
 * order of microseconds not milliseconds) before beginning a queued
 * transaction which require the previous one to have completed.
 *
 * There are dozens of adapters of this type, differing in details which
 * this driver needs to understand.  Those "layout" details are required
 * as part of FT2232 driver configuration.
 *
 * This code uses information contained in the MPSSE specification which was
 * found here:
 * http://www.ftdichip.com/Documents/AppNotes/AN2232C-01_MPSSE_Cmnd.pdf
 * Hereafter this is called the "MPSSE Spec".
 *
 * The datasheet for the ftdichip.com's FT2232D part is here:
 * http://www.ftdichip.com/Documents/DataSheets/DS_FT2232D.pdf
 *
 * Also note the issue with code 0x4b (clock data to TMS) noted in
 * http://developer.intra2net.com/mailarchive/html/libftdi/2009/msg00292.html
 * which can affect longer JTAG state paths.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/* project specific includes */
#include <jtag/interface.h>
#include <transport/transport.h>
#include <helper/time_support.h>

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
#include "ftd2xx_common.h"

enum ftdi_interface {
	INTERFACE_ANY = 0,
	INTERFACE_A   = 1,
	INTERFACE_B   = 2,
	INTERFACE_C   = 3,
	INTERFACE_D   = 4
};

#elif BUILD_FT2232_LIBFTDI == 1
#include <ftdi.h>
#endif

/* max TCK for the high speed devices 30000 kHz */
#define	FTDI_x232H_MAX_TCK	30000
/* max TCK for the full speed devices 6000 kHz */
#define FTDI_2232C_MAX_TCK 6000
/* this speed value tells that RTCK is requested */
#define RTCK_SPEED -1

/*
 * On my Athlon XP 1900+ EHCI host with FT2232H JTAG dongle I get read timeout
 * errors with a retry count of 100. Increasing it solves the problem for me.
 *	- Dimitar
 *
 * FIXME There's likely an issue with the usb_read_timeout from libftdi.
 * Fix that (libusb? kernel? libftdi? here?) and restore the retry count
 * to something sane.
 */
#define LIBFTDI_READ_RETRY_COUNT		2000

#ifndef BUILD_FT2232_HIGHSPEED
 #if BUILD_FT2232_FTD2XX == 1
	enum { FT_DEVICE_2232H = 6, FT_DEVICE_4232H, FT_DEVICE_232H };
 #elif BUILD_FT2232_LIBFTDI == 1
	enum ftdi_chip_type { TYPE_2232H = 4, TYPE_4232H = 5, TYPE_232H = 6 };
 #endif
#endif

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
static int ft2232_stableclocks(int num_cycles, struct jtag_command *cmd);

static char *ft2232_device_desc_A;
static char *ft2232_device_desc;
static char *ft2232_serial;
static uint8_t ft2232_latency = 2;
static unsigned ft2232_max_tck = FTDI_2232C_MAX_TCK;
static int ft2232_channel = INTERFACE_ANY;

#define MAX_USB_IDS 8
/* vid = pid = 0 marks the end of the list */
static uint16_t ft2232_vid[MAX_USB_IDS + 1] = { 0x0403, 0 };
static uint16_t ft2232_pid[MAX_USB_IDS + 1] = { 0x6010, 0 };

struct ft2232_layout {
	char *name;
	int (*init)(void);
	void (*reset)(int trst, int srst);
	void (*blink)(void);
	int channel;
};

/* init procedures for supported layouts */
static int usbjtag_init(void);
static int jtagkey_init(void);
static int lm3s811_jtag_init(void);
static int icdi_jtag_init(void);
static int olimex_jtag_init(void);
static int flyswatter1_init(void);
static int flyswatter2_init(void);
static int minimodule_init(void);
static int turtle_init(void);
static int comstick_init(void);
static int stm32stick_init(void);
static int axm0432_jtag_init(void);
static int sheevaplug_init(void);
static int icebear_jtag_init(void);
static int cortino_jtag_init(void);
static int signalyzer_init(void);
static int signalyzer_h_init(void);
static int ktlink_init(void);
static int redbee_init(void);
static int lisa_l_init(void);
static int flossjtag_init(void);
static int xds100v2_init(void);
static int digilent_hs1_init(void);

/* reset procedures for supported layouts */
static void ftx23_reset(int trst, int srst);
static void jtagkey_reset(int trst, int srst);
static void olimex_jtag_reset(int trst, int srst);
static void flyswatter1_reset(int trst, int srst);
static void flyswatter2_reset(int trst, int srst);
static void minimodule_reset(int trst, int srst);
static void turtle_reset(int trst, int srst);
static void comstick_reset(int trst, int srst);
static void stm32stick_reset(int trst, int srst);
static void axm0432_jtag_reset(int trst, int srst);
static void sheevaplug_reset(int trst, int srst);
static void icebear_jtag_reset(int trst, int srst);
static void signalyzer_h_reset(int trst, int srst);
static void ktlink_reset(int trst, int srst);
static void redbee_reset(int trst, int srst);
static void xds100v2_reset(int trst, int srst);
static void digilent_hs1_reset(int trst, int srst);

/* blink procedures for layouts that support a blinking led */
static void olimex_jtag_blink(void);
static void flyswatter1_jtag_blink(void);
static void flyswatter2_jtag_blink(void);
static void turtle_jtag_blink(void);
static void signalyzer_h_blink(void);
static void ktlink_blink(void);
static void lisa_l_blink(void);
static void flossjtag_blink(void);

/* common transport support options */

/* static const char *jtag_and_swd[] = { "jtag", "swd", NULL }; */

static const struct ft2232_layout  ft2232_layouts[] = {
	{ .name = "usbjtag",
		.init = usbjtag_init,
		.reset = ftx23_reset,
	},
	{ .name = "jtagkey",
		.init = jtagkey_init,
		.reset = jtagkey_reset,
	},
	{ .name = "jtagkey_prototype_v1",
		.init = jtagkey_init,
		.reset = jtagkey_reset,
	},
	{ .name = "oocdlink",
		.init = jtagkey_init,
		.reset = jtagkey_reset,
	},
	{ .name = "signalyzer",
		.init = signalyzer_init,
		.reset = ftx23_reset,
	},
	{ .name = "evb_lm3s811",
		.init = lm3s811_jtag_init,
		.reset = ftx23_reset,
	},
	{ .name = "luminary_icdi",
		.init = icdi_jtag_init,
		.reset = ftx23_reset,
	},
	{ .name = "olimex-jtag",
		.init = olimex_jtag_init,
		.reset = olimex_jtag_reset,
		.blink = olimex_jtag_blink
	},
	{ .name = "flyswatter",
		.init = flyswatter1_init,
		.reset = flyswatter1_reset,
		.blink = flyswatter1_jtag_blink
	},
	{ .name = "flyswatter2",
		.init = flyswatter2_init,
		.reset = flyswatter2_reset,
		.blink = flyswatter2_jtag_blink
	},
	{ .name = "minimodule",
		.init = minimodule_init,
		.reset = minimodule_reset,
	},
	{ .name = "turtelizer2",
		.init = turtle_init,
		.reset = turtle_reset,
		.blink = turtle_jtag_blink
	},
	{ .name = "comstick",
		.init = comstick_init,
		.reset = comstick_reset,
	},
	{ .name = "stm32stick",
		.init = stm32stick_init,
		.reset = stm32stick_reset,
	},
	{ .name = "axm0432_jtag",
		.init = axm0432_jtag_init,
		.reset = axm0432_jtag_reset,
	},
	{ .name = "sheevaplug",
		.init = sheevaplug_init,
		.reset = sheevaplug_reset,
	},
	{ .name = "icebear",
		.init = icebear_jtag_init,
		.reset = icebear_jtag_reset,
	},
	{ .name = "cortino",
		.init = cortino_jtag_init,
		.reset = comstick_reset,
	},
	{ .name = "signalyzer-h",
		.init = signalyzer_h_init,
		.reset = signalyzer_h_reset,
		.blink = signalyzer_h_blink
	},
	{ .name = "ktlink",
		.init = ktlink_init,
		.reset = ktlink_reset,
		.blink = ktlink_blink
	},
	{ .name = "redbee-econotag",
		.init = redbee_init,
		.reset = redbee_reset,
	},
	{ .name = "redbee-usb",
		.init = redbee_init,
		.reset = redbee_reset,
		.channel = INTERFACE_B,
	},
	{ .name = "lisa-l",
		.init = lisa_l_init,
		.reset = ftx23_reset,
		.blink = lisa_l_blink,
		.channel = INTERFACE_B,
	},
	{ .name = "flossjtag",
		.init = flossjtag_init,
		.reset = ftx23_reset,
		.blink = flossjtag_blink,
	},
	{ .name = "xds100v2",
		.init = xds100v2_init,
		.reset = xds100v2_reset,
	},
	{ .name = "digilent-hs1",
		.init = digilent_hs1_init,
		.reset = digilent_hs1_reset,
		.channel = INTERFACE_A,
	},
	{ .name = NULL, /* END OF TABLE */ },
};

/* bitmask used to drive nTRST; usually a GPIOLx signal */
static uint8_t nTRST;
static uint8_t nTRSTnOE;
/* bitmask used to drive nSRST; usually a GPIOLx signal */
static uint8_t nSRST;
static uint8_t nSRSTnOE;

/** the layout being used with this debug session */
static const struct ft2232_layout *layout;

/** default bitmask values driven on DBUS: TCK/TDI/TDO/TMS and GPIOL(0..4) */
static uint8_t low_output;

/* note that direction bit == 1 means that signal is an output */

/** default direction bitmask for DBUS: TCK/TDI/TDO/TMS and GPIOL(0..4) */
static uint8_t low_direction;
/** default value bitmask for CBUS GPIOH(0..4) */
static uint8_t high_output;
/** default direction bitmask for CBUS GPIOH(0..4) */
static uint8_t high_direction;

#if BUILD_FT2232_FTD2XX == 1
static FT_HANDLE ftdih;
static FT_DEVICE ftdi_device;
#elif BUILD_FT2232_LIBFTDI == 1
static struct ftdi_context ftdic;
static enum ftdi_chip_type ftdi_device;
#endif

static struct jtag_command *first_unsent;	/* next command that has to be sent */
static int require_send;

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

#if BUILD_FT2232_FTD2XX == 1
#define FT2232_BUFFER_READ_QUEUE_SIZE (64*64)
#else
#define FT2232_BUFFER_READ_QUEUE_SIZE (64*4)
#endif

#define FT2232_BUFFER_SIZE 131072

static uint8_t *ft2232_buffer;
static int ft2232_buffer_size;
static int ft2232_read_pointer;
static int ft2232_expect_read;

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
	uint8_t tms_byte;
	int i;
	int tms_ndx;	/* bit index into tms_byte */

	assert(tms_count > 0);

	DEBUG_JTAG_IO("mpsse cmd=%02x, tms_bits = 0x%08x, bit_count=%d",
		mpsse_cmd, tms_bits, tms_count);

	for (tms_byte = tms_ndx = i = 0; i < tms_count; ++i, tms_bits >>= 1) {
		bool bit = tms_bits & 1;

		if (bit)
			tms_byte |= (1 << tms_ndx);

		/* always do state transitions in public view */
		tap_set_state(tap_state_transition(tap_get_state(), bit));

		/*	we wrote a bit to tms_byte just above, increment bit index.  if bit was zero
		 * also increment.
		*/
		++tms_ndx;

		if (tms_ndx == 7 || i == tms_count-1) {
			buffer_write(mpsse_cmd);
			buffer_write(tms_ndx - 1);

			/*	Bit 7 of the byte is passed on to TDI/DO before the first TCK/SK of
			 * TMS/CS and is held static for the duration of TMS/CS clocking.
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
	tap_state_t start_state = tap_get_state();

	/*	goal_state is 1/2 of a tuple/pair of states which allow convenient
	 * lookup of the required TMS pattern to move to this state from the start state.
	*/

	/* do the 2 lookups */
	int tms_bits  = tap_get_tms_path(start_state, goal_state);
	int tms_count = tap_get_tms_path_len(start_state, goal_state);

	DEBUG_JTAG_IO("start=%s goal=%s", tap_state_name(start_state), tap_state_name(goal_state));

	clock_tms(0x4b,  tms_bits, tms_count, 0);
}

static int ft2232_write(uint8_t *buf, int size, uint32_t *bytes_written)
{
#if BUILD_FT2232_FTD2XX == 1
	FT_STATUS status;
	DWORD dw_bytes_written = 0;
	status = FT_Write(ftdih, buf, size, &dw_bytes_written);
	if (status != FT_OK) {
		*bytes_written = dw_bytes_written;
		LOG_ERROR("FT_Write returned: %s", ftd2xx_status_string(status));
		return ERROR_JTAG_DEVICE_ERROR;
	} else
		*bytes_written = dw_bytes_written;

#elif BUILD_FT2232_LIBFTDI == 1
	int retval = ftdi_write_data(&ftdic, buf, size);
	if (retval < 0) {
		*bytes_written = 0;
		LOG_ERROR("ftdi_write_data: %s", ftdi_get_error_string(&ftdic));
		return ERROR_JTAG_DEVICE_ERROR;
	} else
		*bytes_written = retval;

#endif

	if (*bytes_written != (uint32_t)size)
		return ERROR_JTAG_DEVICE_ERROR;

	return ERROR_OK;
}

static int ft2232_read(uint8_t *buf, uint32_t size, uint32_t *bytes_read)
{
#if BUILD_FT2232_FTD2XX == 1
	DWORD dw_bytes_read;
	FT_STATUS status;
	int timeout = 5;
	*bytes_read = 0;

	while ((*bytes_read < size) && timeout--) {
		status = FT_Read(ftdih, buf + *bytes_read, size -
				*bytes_read, &dw_bytes_read);
		if (status != FT_OK) {
			*bytes_read = 0;
			LOG_ERROR("FT_Read returned: %s", ftd2xx_status_string(status));
			return ERROR_JTAG_DEVICE_ERROR;
		}
		*bytes_read += dw_bytes_read;
	}

#elif BUILD_FT2232_LIBFTDI == 1
	int retval;
	int timeout = LIBFTDI_READ_RETRY_COUNT;
	*bytes_read = 0;

	while ((*bytes_read < size) && timeout--) {
		retval = ftdi_read_data(&ftdic, buf + *bytes_read, size - *bytes_read);
		if (retval < 0) {
			*bytes_read = 0;
			LOG_ERROR("ftdi_read_data: %s", ftdi_get_error_string(&ftdic));
			return ERROR_JTAG_DEVICE_ERROR;
		}
		*bytes_read += retval;
	}

#endif

	if (*bytes_read < size) {
		LOG_ERROR("couldn't read enough bytes from "
			"FT2232 device (%i < %i)",
			(unsigned)*bytes_read,
			(unsigned)size);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static bool ft2232_device_is_highspeed(void)
{
#if BUILD_FT2232_FTD2XX == 1
	return (ftdi_device == FT_DEVICE_2232H) || (ftdi_device == FT_DEVICE_4232H)
 #ifdef HAS_ENUM_FT232H
		|| (ftdi_device == FT_DEVICE_232H)
 #endif
	;
#elif BUILD_FT2232_LIBFTDI == 1
	return (ftdi_device == TYPE_2232H || ftdi_device == TYPE_4232H
 #ifdef HAS_ENUM_FT232H
		|| ftdi_device == TYPE_232H
 #endif
	);
#endif
}

/*
 * Commands that only apply to the highspeed FTx232H devices (FT2232H, FT4232H, FT232H).
 * See chapter 6 in http://www.ftdichip.com/Documents/AppNotes/
 * AN_108_Command_Processor_for_MPSSE_and_MCU_Host_Bus_Emulation_Modes.pdf
 */

static int ftx232h_adaptive_clocking(bool enable)
{
	uint8_t buf = enable ? 0x96 : 0x97;
	LOG_DEBUG("%2.2x", buf);

	uint32_t bytes_written;
	int retval;

	retval = ft2232_write(&buf, sizeof(buf), &bytes_written);
	if (retval != ERROR_OK) {
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
static int ftx232h_clk_divide_by_5(bool enable)
{
	uint32_t bytes_written;
	uint8_t buf = enable ?  0x8b : 0x8a;

	if (ft2232_write(&buf, sizeof(buf), &bytes_written) != ERROR_OK) {
		LOG_ERROR("couldn't write command to %s clk divide by 5"
			, enable ? "enable" : "disable");
		return ERROR_JTAG_INIT_FAILED;
	}
	ft2232_max_tck = enable ? FTDI_2232C_MAX_TCK : FTDI_x232H_MAX_TCK;
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
		retval = ftx232h_adaptive_clocking(enable_adaptive_clocking);
	else if (enable_adaptive_clocking) {
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
	retval = ft2232_write(buf, sizeof(buf), &bytes_written);
	if (retval != ERROR_OK) {
		LOG_ERROR("couldn't set FT2232 TCK speed");
		return retval;
	}

	return ERROR_OK;
}

static int ft2232_speed_div(int speed, int *khz)
{
	/* Take a look in the FT2232 manual,
	 * AN2232C-01 Command Processor for
	 * MPSSE and MCU Host Bus. Chapter 3.8 */

	*khz = (RTCK_SPEED == speed) ? 0 : ft2232_max_tck / (1 + speed);

	return ERROR_OK;
}

static int ft2232_khz(int khz, int *jtag_speed)
{
	if (khz == 0) {
		if (ft2232_device_is_highspeed()) {
			*jtag_speed = RTCK_SPEED;
			return ERROR_OK;
		} else {
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

	/* Calc speed, (ft2232_max_tck / khz) - 1
	 * Use 65000 for better rounding */
	*jtag_speed = ((ft2232_max_tck*10) / khz) - 10;

	/* Add 0.9 for rounding */
	*jtag_speed += 9;

	/* Calc real speed */
	*jtag_speed = *jtag_speed / 10;

	/* Check if speed is greater than 0 */
	if (*jtag_speed < 0)
		*jtag_speed = 0;

	/* Check max value */
	if (*jtag_speed > 0xFFFF)
		*jtag_speed = 0xFFFF;

	return ERROR_OK;
}

static void ft2232_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else {
		LOG_ERROR("BUG: %s is not a stable end state", tap_state_name(state));
		exit(-1);
	}
}

static void ft2232_read_scan(enum scan_type type, uint8_t *buffer, int scan_size)
{
	int num_bytes = (scan_size + 7) / 8;
	int bits_left = scan_size;
	int cur_byte  = 0;

	while (num_bytes-- > 1) {
		buffer[cur_byte++] = buffer_read();
		bits_left -= 8;
	}

	buffer[cur_byte] = 0x0;

	/* There is one more partial byte left from the clock data in/out instructions */
	if (bits_left > 1)
		buffer[cur_byte] = buffer_read() >> 1;
	/* This shift depends on the length of the
	 *clock data to tms instruction, insterted
	 *at end of the scan, now fixed to a two
	 *step transition in ft2232_add_scan */
	buffer[cur_byte] = (buffer[cur_byte] | (((buffer_read()) << 1) & 0x80)) >> (8 - bits_left);
}

static void ft2232_debug_dump_buffer(void)
{
	int i;
	char line[256];
	char *line_p = line;

	for (i = 0; i < ft2232_buffer_size; i++) {
		line_p += snprintf(line_p,
				sizeof(line) - (line_p - line),
				"%2.2x ",
				ft2232_buffer[i]);
		if (i % 16 == 15) {
			LOG_DEBUG("%s", line);
			line_p = line;
		}
	}

	if (line_p != line)
		LOG_DEBUG("%s", line);
}

static int ft2232_send_and_recv(struct jtag_command *first, struct jtag_command *last)
{
	struct jtag_command *cmd;
	uint8_t *buffer;
	int scan_size;
	enum scan_type type;
	int retval;
	uint32_t bytes_written = 0;
	uint32_t bytes_read = 0;

#ifdef _DEBUG_USB_IO_
	struct timeval start, inter, inter2, end;
	struct timeval d_inter, d_inter2, d_end;
#endif

#ifdef _DEBUG_USB_COMMS_
	LOG_DEBUG("write buffer (size %i):", ft2232_buffer_size);
	ft2232_debug_dump_buffer();
#endif

#ifdef _DEBUG_USB_IO_
	gettimeofday(&start, NULL);
#endif

	retval = ft2232_write(ft2232_buffer, ft2232_buffer_size, &bytes_written);
	if (retval != ERROR_OK) {
		LOG_ERROR("couldn't write MPSSE commands to FT2232");
		return retval;
	}

#ifdef _DEBUG_USB_IO_
	gettimeofday(&inter, NULL);
#endif

	if (ft2232_expect_read) {
		/* FIXME this "timeout" is never changed ... */
		int timeout = LIBFTDI_READ_RETRY_COUNT;
		ft2232_buffer_size = 0;

#ifdef _DEBUG_USB_IO_
		gettimeofday(&inter2, NULL);
#endif

		retval = ft2232_read(ft2232_buffer, ft2232_expect_read, &bytes_read);
		if (retval != ERROR_OK) {
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

		if (ft2232_expect_read != ft2232_buffer_size) {
			LOG_ERROR("ft2232_expect_read (%i) != "
				"ft2232_buffer_size (%i) "
				"(%i retries)",
				ft2232_expect_read,
				ft2232_buffer_size,
				LIBFTDI_READ_RETRY_COUNT - timeout);
			ft2232_debug_dump_buffer();

			exit(-1);
		}

#ifdef _DEBUG_USB_COMMS_
		LOG_DEBUG("read buffer (%i retries): %i bytes",
			LIBFTDI_READ_RETRY_COUNT - timeout,
			ft2232_buffer_size);
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
	while (cmd != last) {
		switch (cmd->type) {
			case JTAG_SCAN:
				type = jtag_scan_type(cmd->cmd.scan);
				if (type != SCAN_OUT) {
					scan_size = jtag_scan_size(cmd->cmd.scan);
					buffer = calloc(DIV_ROUND_UP(scan_size, 8), 1);
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
static void ft2232_add_pathmove(tap_state_t *path, int num_states)
{
	int state_count = 0;

	assert((unsigned) num_states <= 32u);		/* tms_bits only holds 32 bits */

	DEBUG_JTAG_IO("-");

	/* this loop verifies that the path is legal and logs each state in the path */
	while (num_states) {
		unsigned char tms_byte = 0;		/* zero this on each MPSSE batch */
		int bit_count = 0;
		int num_states_batch = num_states > 7 ? 7 : num_states;

		/* command "Clock Data to TMS/CS Pin (no Read)" */
		buffer_write(0x4b);

		/* number of states remaining */
		buffer_write(num_states_batch - 1);

		while (num_states_batch--) {
			/* either TMS=0 or TMS=1 must work ... */
			if (tap_state_transition(tap_get_state(), false) == path[state_count])
				buf_set_u32(&tms_byte, bit_count++, 1, 0x0);
			else if (tap_state_transition(tap_get_state(), true) == path[state_count])
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

static void ft2232_add_scan(bool ir_scan, enum scan_type type, uint8_t *buffer, int scan_size)
{
	int num_bytes = (scan_size + 7) / 8;
	int bits_left = scan_size;
	int cur_byte  = 0;
	int last_bit;

	if (!ir_scan) {
		if (tap_get_state() != TAP_DRSHIFT)
			move_to_state(TAP_DRSHIFT);
	} else {
		if (tap_get_state() != TAP_IRSHIFT)
			move_to_state(TAP_IRSHIFT);
	}

	/* add command for complete bytes */
	while (num_bytes > 1) {
		int thisrun_bytes;
		if (type == SCAN_IO) {
			/* Clock Data Bytes In and Out LSB First */
			buffer_write(0x39);
			/* LOG_DEBUG("added TDI bytes (io %i)", num_bytes); */
		} else if (type == SCAN_OUT) {
			/* Clock Data Bytes Out on -ve Clock Edge LSB First (no Read) */
			buffer_write(0x19);
			/* LOG_DEBUG("added TDI bytes (o)"); */
		} else if (type == SCAN_IN) {
			/* Clock Data Bytes In on +ve Clock Edge LSB First (no Write) */
			buffer_write(0x28);
			/* LOG_DEBUG("added TDI bytes (i %i)", num_bytes); */
		}

		thisrun_bytes = (num_bytes > 65537) ? 65536 : (num_bytes - 1);
		num_bytes -= thisrun_bytes;

		buffer_write((uint8_t) (thisrun_bytes - 1));
		buffer_write((uint8_t) ((thisrun_bytes - 1) >> 8));

		if (type != SCAN_IN) {
			/* add complete bytes */
			while (thisrun_bytes-- > 0) {
				buffer_write(buffer[cur_byte++]);
				bits_left -= 8;
			}
		} else /* (type == SCAN_IN) */
			bits_left -= 8 * (thisrun_bytes);
	}

	/* the most signifcant bit is scanned during TAP movement */
	if (type != SCAN_IN)
		last_bit = (buffer[cur_byte] >> (bits_left - 1)) & 0x1;
	else
		last_bit = 0;

	/* process remaining bits but the last one */
	if (bits_left > 1) {
		if (type == SCAN_IO) {
			/* Clock Data Bits In and Out LSB First */
			buffer_write(0x3b);
			/* LOG_DEBUG("added TDI bits (io) %i", bits_left - 1); */
		} else if (type == SCAN_OUT) {
			/* Clock Data Bits Out on -ve Clock Edge LSB First (no Read) */
			buffer_write(0x1b);
			/* LOG_DEBUG("added TDI bits (o)"); */
		} else if (type == SCAN_IN) {
			/* Clock Data Bits In on +ve Clock Edge LSB First (no Write) */
			buffer_write(0x2a);
			/* LOG_DEBUG("added TDI bits (i %i)", bits_left - 1); */
		}

		buffer_write(bits_left - 2);
		if (type != SCAN_IN)
			buffer_write(buffer[cur_byte]);
	}

	if ((ir_scan && (tap_get_end_state() == TAP_IRSHIFT))
			|| (!ir_scan && (tap_get_end_state() == TAP_DRSHIFT))) {
		if (type == SCAN_IO) {
			/* Clock Data Bits In and Out LSB First */
			buffer_write(0x3b);
			/* LOG_DEBUG("added TDI bits (io) %i", bits_left - 1); */
		} else if (type == SCAN_OUT) {
			/* Clock Data Bits Out on -ve Clock Edge LSB First (no Read) */
			buffer_write(0x1b);
			/* LOG_DEBUG("added TDI bits (o)"); */
		} else if (type == SCAN_IN) {
			/* Clock Data Bits In on +ve Clock Edge LSB First (no Write) */
			buffer_write(0x2a);
			/* LOG_DEBUG("added TDI bits (i %i)", bits_left - 1); */
		}
		buffer_write(0x0);
		if (type != SCAN_IN)
			buffer_write(last_bit);
	} else {
		int tms_bits;
		int tms_count;
		uint8_t mpsse_cmd;

		/* move from Shift-IR/DR to end state */
		if (type != SCAN_OUT) {
			/* We always go to the PAUSE state in two step at the end of an IN or IO
			 *scan
			 * This must be coordinated with the bit shifts in ft2232_read_scan    */
			tms_bits  = 0x01;
			tms_count = 2;
			/* Clock Data to TMS/CS Pin with Read */
			mpsse_cmd = 0x6b;
		} else {
			tms_bits  = tap_get_tms_path(tap_get_state(), tap_get_end_state());
			tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());
			/* Clock Data to TMS/CS Pin (no Read) */
			mpsse_cmd = 0x4b;
		}

		DEBUG_JTAG_IO("finish %s", (type == SCAN_OUT) ? "without read" : "via PAUSE");
		clock_tms(mpsse_cmd, tms_bits, tms_count, last_bit);
	}

	if (tap_get_state() != tap_get_end_state())
		move_to_state(tap_get_end_state());
}

static int ft2232_large_scan(struct scan_command *cmd,
	enum scan_type type,
	uint8_t *buffer,
	int scan_size)
{
	int num_bytes = (scan_size + 7) / 8;
	int bits_left = scan_size;
	int cur_byte  = 0;
	int last_bit;
	uint8_t *receive_buffer  = malloc(DIV_ROUND_UP(scan_size, 8));
	uint8_t *receive_pointer = receive_buffer;
	uint32_t bytes_written;
	uint32_t bytes_read;
	int retval;
	int thisrun_read = 0;

	if (!receive_buffer) {
		LOG_ERROR("failed to allocate memory");
		exit(-1);
	}

	if (cmd->ir_scan) {
		LOG_ERROR("BUG: large IR scans are not supported");
		exit(-1);
	}

	if (tap_get_state() != TAP_DRSHIFT)
		move_to_state(TAP_DRSHIFT);

	retval = ft2232_write(ft2232_buffer, ft2232_buffer_size, &bytes_written);
	if (retval != ERROR_OK) {
		LOG_ERROR("couldn't write MPSSE commands to FT2232");
		exit(-1);
	}
	LOG_DEBUG("ft2232_buffer_size: %i, bytes_written: %i",
		ft2232_buffer_size, (int)bytes_written);
	ft2232_buffer_size = 0;

	/* add command for complete bytes */
	while (num_bytes > 1) {
		int thisrun_bytes;

		if (type == SCAN_IO) {
			/* Clock Data Bytes In and Out LSB First */
			buffer_write(0x39);
			/* LOG_DEBUG("added TDI bytes (io %i)", num_bytes); */
		} else if (type == SCAN_OUT) {
			/* Clock Data Bytes Out on -ve Clock Edge LSB First (no Read) */
			buffer_write(0x19);
			/* LOG_DEBUG("added TDI bytes (o)"); */
		} else if (type == SCAN_IN) {
			/* Clock Data Bytes In on +ve Clock Edge LSB First (no Write) */
			buffer_write(0x28);
			/* LOG_DEBUG("added TDI bytes (i %i)", num_bytes); */
		}

		thisrun_bytes = (num_bytes > 65537) ? 65536 : (num_bytes - 1);
		thisrun_read  = thisrun_bytes;
		num_bytes    -= thisrun_bytes;
		buffer_write((uint8_t) (thisrun_bytes - 1));
		buffer_write((uint8_t) ((thisrun_bytes - 1) >> 8));

		if (type != SCAN_IN) {
			/* add complete bytes */
			while (thisrun_bytes-- > 0) {
				buffer_write(buffer[cur_byte]);
				cur_byte++;
				bits_left -= 8;
			}
		} else /* (type == SCAN_IN) */
			bits_left -= 8 * (thisrun_bytes);

		retval = ft2232_write(ft2232_buffer, ft2232_buffer_size, &bytes_written);
		if (retval != ERROR_OK) {
			LOG_ERROR("couldn't write MPSSE commands to FT2232");
			exit(-1);
		}
		LOG_DEBUG("ft2232_buffer_size: %i, bytes_written: %i",
			ft2232_buffer_size,
			(int)bytes_written);
		ft2232_buffer_size = 0;

		if (type != SCAN_OUT) {
			retval = ft2232_read(receive_pointer, thisrun_read, &bytes_read);
			if (retval != ERROR_OK) {
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
	if (bits_left > 1) {
		if (type == SCAN_IO) {
			/* Clock Data Bits In and Out LSB First */
			buffer_write(0x3b);
			/* LOG_DEBUG("added TDI bits (io) %i", bits_left - 1); */
		} else if (type == SCAN_OUT) {
			/* Clock Data Bits Out on -ve Clock Edge LSB First (no Read) */
			buffer_write(0x1b);
			/* LOG_DEBUG("added TDI bits (o)"); */
		} else if (type == SCAN_IN) {
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

	if (tap_get_end_state() == TAP_DRSHIFT) {
		if (type == SCAN_IO) {
			/* Clock Data Bits In and Out LSB First */
			buffer_write(0x3b);
			/* LOG_DEBUG("added TDI bits (io) %i", bits_left - 1); */
		} else if (type == SCAN_OUT) {
			/* Clock Data Bits Out on -ve Clock Edge LSB First (no Read) */
			buffer_write(0x1b);
			/* LOG_DEBUG("added TDI bits (o)"); */
		} else if (type == SCAN_IN) {
			/* Clock Data Bits In on +ve Clock Edge LSB First (no Write) */
			buffer_write(0x2a);
			/* LOG_DEBUG("added TDI bits (i %i)", bits_left - 1); */
		}
		buffer_write(0x0);
		buffer_write(last_bit);
	} else {
		int tms_bits  = tap_get_tms_path(tap_get_state(), tap_get_end_state());
		int tms_count = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());
		uint8_t mpsse_cmd;

		/* move from Shift-IR/DR to end state */
		if (type != SCAN_OUT) {
			/* Clock Data to TMS/CS Pin with Read */
			mpsse_cmd = 0x6b;
			/* LOG_DEBUG("added TMS scan (read)"); */
		} else {
			/* Clock Data to TMS/CS Pin (no Read) */
			mpsse_cmd = 0x4b;
			/* LOG_DEBUG("added TMS scan (no read)"); */
		}

		DEBUG_JTAG_IO("finish, %s", (type == SCAN_OUT) ? "no read" : "read");
		clock_tms(mpsse_cmd, tms_bits, tms_count, last_bit);
	}

	if (type != SCAN_OUT)
		thisrun_read += 1;

	retval = ft2232_write(ft2232_buffer, ft2232_buffer_size, &bytes_written);
	if (retval != ERROR_OK) {
		LOG_ERROR("couldn't write MPSSE commands to FT2232");
		exit(-1);
	}
	LOG_DEBUG("ft2232_buffer_size: %i, bytes_written: %i",
		ft2232_buffer_size,
		(int)bytes_written);
	ft2232_buffer_size = 0;

	if (type != SCAN_OUT) {
		retval = ft2232_read(receive_pointer, thisrun_read, &bytes_read);
		if (retval != ERROR_OK) {
			LOG_ERROR("couldn't read from FT2232");
			exit(-1);
		}
		LOG_DEBUG("thisrun_read: %i, bytes_read: %i",
			thisrun_read,
			(int)bytes_read);
	}

	free(receive_buffer);

	return ERROR_OK;
}

static int ft2232_predict_scan_out(int scan_size, enum scan_type type)
{
	int predicted_size = 3;
	int num_bytes = (scan_size - 1) / 8;

	if (tap_get_state() != TAP_DRSHIFT)
		predicted_size += get_tms_buffer_requirements(
				tap_get_tms_path_len(tap_get_state(), TAP_DRSHIFT));

	if (type == SCAN_IN) {	/* only from device to host */
		/* complete bytes */
		predicted_size += DIV_ROUND_UP(num_bytes, 65536) * 3;

		/* remaining bits - 1 (up to 7) */
		predicted_size += ((scan_size - 1) % 8) ? 2 : 0;
	} else {/* host to device, or bidirectional
		 * complete bytes */
		predicted_size += num_bytes + DIV_ROUND_UP(num_bytes, 65536) * 3;

		/* remaining bits -1 (up to 7) */
		predicted_size += ((scan_size - 1) % 8) ? 3 : 0;
	}

	return predicted_size;
}

static int ft2232_predict_scan_in(int scan_size, enum scan_type type)
{
	int predicted_size = 0;

	if (type != SCAN_OUT) {
		/* complete bytes */
		predicted_size +=
			(DIV_ROUND_UP(scan_size, 8) > 1) ? (DIV_ROUND_UP(scan_size, 8) - 1) : 0;

		/* remaining bits - 1 */
		predicted_size += ((scan_size - 1) % 8) ? 1 : 0;

		/* last bit (from TMS scan) */
		predicted_size += 1;
	}

	/* LOG_DEBUG("scan_size: %i, predicted_size: %i", scan_size, predicted_size); */

	return predicted_size;
}

/* semi-generic FT2232/FT4232 reset code */
static void ftx23_reset(int trst, int srst)
{
	enum reset_types jtag_reset_config = jtag_get_reset_config();
	if (trst == 1) {
		if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
			low_direction |= nTRSTnOE;	/* switch to output pin (output is low) */
		else
			low_output &= ~nTRST;		/* switch output low */
	} else if (trst == 0) {
		if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
			low_direction &= ~nTRSTnOE;	/* switch to input pin (high-Z + internal
							 *and external pullup) */
		else
			low_output |= nTRST;		/* switch output high */
	}

	if (srst == 1) {
		if (jtag_reset_config & RESET_SRST_PUSH_PULL)
			low_output &= ~nSRST;		/* switch output low */
		else
			low_direction |= nSRSTnOE;	/* switch to output pin (output is low) */
	} else if (srst == 0) {
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
	if (trst == 1) {
		if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
			high_output &= ~nTRSTnOE;
		else
			high_output &= ~nTRST;
	} else if (trst == 0) {
		if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
			high_output |= nTRSTnOE;
		else
			high_output |= nTRST;
	}

	if (srst == 1) {
		if (jtag_reset_config & RESET_SRST_PUSH_PULL)
			high_output &= ~nSRST;
		else
			high_output &= ~nSRSTnOE;
	} else if (srst == 0) {
		if (jtag_reset_config & RESET_SRST_PUSH_PULL)
			high_output |= nSRST;
		else
			high_output |= nSRSTnOE;
	}

	/* command "set data bits high byte" */
	buffer_write(0x82);
	buffer_write(high_output);
	buffer_write(high_direction);
	LOG_DEBUG("trst: %i, srst: %i, high_output: 0x%2.2x, high_direction: 0x%2.2x",
		trst,
		srst,
		high_output,
		high_direction);
}

static void olimex_jtag_reset(int trst, int srst)
{
	enum reset_types jtag_reset_config = jtag_get_reset_config();
	if (trst == 1) {
		if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
			high_output &= ~nTRSTnOE;
		else
			high_output &= ~nTRST;
	} else if (trst == 0) {
		if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
			high_output |= nTRSTnOE;
		else
			high_output |= nTRST;
	}

	if (srst == 1)
		high_output |= nSRST;
	else if (srst == 0)
		high_output &= ~nSRST;

	/* command "set data bits high byte" */
	buffer_write(0x82);
	buffer_write(high_output);
	buffer_write(high_direction);
	LOG_DEBUG("trst: %i, srst: %i, high_output: 0x%2.2x, high_direction: 0x%2.2x",
		trst,
		srst,
		high_output,
		high_direction);
}

static void axm0432_jtag_reset(int trst, int srst)
{
	if (trst == 1) {
		tap_set_state(TAP_RESET);
		high_output &= ~nTRST;
	} else if (trst == 0)
		high_output |= nTRST;

	if (srst == 1)
		high_output &= ~nSRST;
	else if (srst == 0)
		high_output |= nSRST;

	/* command "set data bits low byte" */
	buffer_write(0x82);
	buffer_write(high_output);
	buffer_write(high_direction);
	LOG_DEBUG("trst: %i, srst: %i, high_output: 0x%2.2x, high_direction: 0x%2.2x",
		trst,
		srst,
		high_output,
		high_direction);
}

static void flyswatter_reset(int trst, int srst)
{
	if (trst == 1)
		low_output &= ~nTRST;
	else if (trst == 0)
		low_output |= nTRST;

	if (srst == 1)
		low_output |= nSRST;
	else if (srst == 0)
		low_output &= ~nSRST;

	/* command "set data bits low byte" */
	buffer_write(0x80);
	buffer_write(low_output);
	buffer_write(low_direction);
	LOG_DEBUG("trst: %i, srst: %i, low_output: 0x%2.2x, low_direction: 0x%2.2x",
		trst,
		srst,
		low_output,
		low_direction);
}

static void flyswatter1_reset(int trst, int srst)
{
	flyswatter_reset(trst, srst);
}

static void flyswatter2_reset(int trst, int srst)
{
	flyswatter_reset(trst, !srst);
}

static void minimodule_reset(int trst, int srst)
{
	if (srst == 1)
		low_output &= ~nSRST;
	else if (srst == 0)
		low_output |= nSRST;

	/* command "set data bits low byte" */
	buffer_write(0x80);
	buffer_write(low_output);
	buffer_write(low_direction);
	LOG_DEBUG("trst: %i, srst: %i, low_output: 0x%2.2x, low_direction: 0x%2.2x",
		trst,
		srst,
		low_output,
		low_direction);
}

static void turtle_reset(int trst, int srst)
{
	if (trst == 1)
		LOG_ERROR("Can't assert TRST: the adapter lacks this signal");

	if (srst == 1)
		low_output |= nSRST;
	else if (srst == 0)
		low_output &= ~nSRST;

	/* command "set data bits low byte" */
	buffer_write(0x80);
	buffer_write(low_output);
	buffer_write(low_direction);
	LOG_DEBUG("srst: %i, low_output: 0x%2.2x, low_direction: 0x%2.2x",
		srst,
		low_output,
		low_direction);
}

static void comstick_reset(int trst, int srst)
{
	if (trst == 1)
		high_output &= ~nTRST;
	else if (trst == 0)
		high_output |= nTRST;

	if (srst == 1)
		high_output &= ~nSRST;
	else if (srst == 0)
		high_output |= nSRST;

	/* command "set data bits high byte" */
	buffer_write(0x82);
	buffer_write(high_output);
	buffer_write(high_direction);
	LOG_DEBUG("trst: %i, srst: %i, high_output: 0x%2.2x, high_direction: 0x%2.2x",
		trst,
		srst,
		high_output,
		high_direction);
}

static void stm32stick_reset(int trst, int srst)
{
	if (trst == 1)
		high_output &= ~nTRST;
	else if (trst == 0)
		high_output |= nTRST;

	if (srst == 1)
		low_output &= ~nSRST;
	else if (srst == 0)
		low_output |= nSRST;

	/* command "set data bits low byte" */
	buffer_write(0x80);
	buffer_write(low_output);
	buffer_write(low_direction);

	/* command "set data bits high byte" */
	buffer_write(0x82);
	buffer_write(high_output);
	buffer_write(high_direction);
	LOG_DEBUG("trst: %i, srst: %i, high_output: 0x%2.2x, high_direction: 0x%2.2x",
		trst,
		srst,
		high_output,
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
	LOG_DEBUG("trst: %i, srst: %i, high_output: 0x%2.2x, high_direction: 0x%2.2x",
		trst,
		srst,
		high_output,
		high_direction);
}

static void redbee_reset(int trst, int srst)
{
	if (trst == 1) {
		tap_set_state(TAP_RESET);
		high_output &= ~nTRST;
	} else if (trst == 0)
		high_output |= nTRST;

	if (srst == 1)
		high_output &= ~nSRST;
	else if (srst == 0)
		high_output |= nSRST;

	/* command "set data bits low byte" */
	buffer_write(0x82);
	buffer_write(high_output);
	buffer_write(high_direction);
	LOG_DEBUG("trst: %i, srst: %i, high_output: 0x%2.2x, "
		"high_direction: 0x%2.2x", trst, srst, high_output,
		high_direction);
}

static void xds100v2_reset(int trst, int srst)
{
	if (trst == 1) {
		tap_set_state(TAP_RESET);
		high_output &= ~nTRST;
	} else if (trst == 0)
		high_output |= nTRST;

	if (srst == 1)
		high_output |= nSRST;
	else if (srst == 0)
		high_output &= ~nSRST;

	/* command "set data bits low byte" */
	buffer_write(0x82);
	buffer_write(high_output);
	buffer_write(high_direction);
	LOG_DEBUG("trst: %i, srst: %i, high_output: 0x%2.2x, "
		"high_direction: 0x%2.2x", trst, srst, high_output,
		high_direction);
}

static int ft2232_execute_runtest(struct jtag_command *cmd)
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
	predicted_size += 3 * DIV_ROUND_UP(cmd->cmd.runtest->num_cycles, 7);
	if (cmd->cmd.runtest->end_state != TAP_IDLE)
		predicted_size += 3;
	if (tap_get_end_state() != TAP_IDLE)
		predicted_size += 3;
	if (ft2232_buffer_size + predicted_size + 1 > FT2232_BUFFER_SIZE) {
		if (ft2232_send_and_recv(first_unsent, cmd) != ERROR_OK)
			retval = ERROR_JTAG_QUEUE_FAILED;
		require_send = 0;
		first_unsent = cmd;
	}
	if (tap_get_state() != TAP_IDLE) {
		move_to_state(TAP_IDLE);
		require_send = 1;
	}
	i = cmd->cmd.runtest->num_cycles;
	while (i > 0) {
		/* there are no state transitions in this code, so omit state tracking */

		/* command "Clock Data to TMS/CS Pin (no Read)" */
		buffer_write(0x4b);

		/* scan 7 bits */
		buffer_write((i > 7) ? 6 : (i - 1));

		/* TMS data bits */
		buffer_write(0x0);

		i -= (i > 7) ? 7 : i;
		/* LOG_DEBUG("added TMS scan (no read)"); */
	}

	ft2232_end_state(cmd->cmd.runtest->end_state);

	if (tap_get_state() != tap_get_end_state())
		move_to_state(tap_get_end_state());

	require_send = 1;
	DEBUG_JTAG_IO("runtest: %i, end in %s",
		cmd->cmd.runtest->num_cycles,
		tap_state_name(tap_get_end_state()));
	return retval;
}

static int ft2232_execute_statemove(struct jtag_command *cmd)
{
	int predicted_size = 0;
	int retval = ERROR_OK;

	DEBUG_JTAG_IO("statemove end in %s",
		tap_state_name(cmd->cmd.statemove->end_state));

	/* only send the maximum buffer size that FT2232C can handle */
	predicted_size = 3;
	if (ft2232_buffer_size + predicted_size + 1 > FT2232_BUFFER_SIZE) {
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
	} else if (tap_get_state() != tap_get_end_state()) {
		move_to_state(tap_get_end_state());
		require_send = 1;
	}

	return retval;
}

/**
 * Clock a bunch of TMS (or SWDIO) transitions, to change the JTAG
 * (or SWD) state machine.
 */
static int ft2232_execute_tms(struct jtag_command *cmd)
{
	int retval = ERROR_OK;
	unsigned num_bits = cmd->cmd.tms->num_bits;
	const uint8_t *bits = cmd->cmd.tms->bits;
	unsigned count;

	DEBUG_JTAG_IO("TMS: %d bits", num_bits);

	/* only send the maximum buffer size that FT2232C can handle */
	count = 3 * DIV_ROUND_UP(num_bits, 4);
	if (ft2232_buffer_size + 3*count + 1 > FT2232_BUFFER_SIZE) {
		if (ft2232_send_and_recv(first_unsent, cmd) != ERROR_OK)
			retval = ERROR_JTAG_QUEUE_FAILED;

		require_send = 0;
		first_unsent = cmd;
	}

	/* Shift out in batches of at most 6 bits; there's a report of an
	 * FT2232 bug in this area, where shifting exactly 7 bits can make
	 * problems with TMS signaling for the last clock cycle:
	 *
	 *    http://developer.intra2net.com/mailarchive/html/
	 *		libftdi/2009/msg00292.html
	 *
	 * Command 0x4b is: "Clock Data to TMS/CS Pin (no Read)"
	 *
	 * Note that pathmoves in JTAG are not often seven bits, so that
	 * isn't a particularly likely situation outside of "special"
	 * signaling such as switching between JTAG and SWD modes.
	 */
	while (num_bits) {
		if (num_bits <= 6) {
			buffer_write(0x4b);
			buffer_write(num_bits - 1);
			buffer_write(*bits & 0x3f);
			break;
		}

		/* Yes, this is lazy ... we COULD shift out more data
		 * bits per operation, but doing it in nybbles is easy
		 */
		buffer_write(0x4b);
		buffer_write(3);
		buffer_write(*bits & 0xf);
		num_bits -= 4;

		count  = (num_bits > 4) ? 4 : num_bits;

		buffer_write(0x4b);
		buffer_write(count - 1);
		buffer_write((*bits >> 4) & 0xf);
		num_bits -= count;

		bits++;
	}

	require_send = 1;
	return retval;
}

static int ft2232_execute_pathmove(struct jtag_command *cmd)
{
	int predicted_size = 0;
	int retval = ERROR_OK;

	tap_state_t *path = cmd->cmd.pathmove->path;
	int num_states    = cmd->cmd.pathmove->num_states;

	DEBUG_JTAG_IO("pathmove: %i states, current: %s  end: %s", num_states,
		tap_state_name(tap_get_state()),
		tap_state_name(path[num_states-1]));

	/* only send the maximum buffer size that FT2232C can handle */
	predicted_size = 3 * DIV_ROUND_UP(num_states, 7);
	if (ft2232_buffer_size + predicted_size + 1 > FT2232_BUFFER_SIZE) {
		if (ft2232_send_and_recv(first_unsent, cmd) != ERROR_OK)
			retval = ERROR_JTAG_QUEUE_FAILED;

		require_send = 0;
		first_unsent = cmd;
	}

	ft2232_add_pathmove(path, num_states);
	require_send = 1;

	return retval;
}

static int ft2232_execute_scan(struct jtag_command *cmd)
{
	uint8_t *buffer;
	int scan_size;				/* size of IR or DR scan */
	int predicted_size = 0;
	int retval = ERROR_OK;

	enum scan_type type = jtag_scan_type(cmd->cmd.scan);

	DEBUG_JTAG_IO("%s type:%d", cmd->cmd.scan->ir_scan ? "IRSCAN" : "DRSCAN", type);

	scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);

	predicted_size = ft2232_predict_scan_out(scan_size, type);
	if ((predicted_size + 1) > FT2232_BUFFER_SIZE) {
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
	} else if (ft2232_buffer_size + predicted_size + 1 > FT2232_BUFFER_SIZE) {
		LOG_DEBUG(
			"ft2232 buffer size reached, sending queued commands (first_unsent: %p, cmd: %p)",
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
	DEBUG_JTAG_IO("%s scan, %i bits, end in %s",
		(cmd->cmd.scan->ir_scan) ? "IR" : "DR", scan_size,
		tap_state_name(tap_get_end_state()));
	return retval;

}

static int ft2232_execute_reset(struct jtag_command *cmd)
{
	int retval;
	int predicted_size = 0;
	retval = ERROR_OK;

	DEBUG_JTAG_IO("reset trst: %i srst %i",
		cmd->cmd.reset->trst, cmd->cmd.reset->srst);

	/* only send the maximum buffer size that FT2232C can handle */
	predicted_size = 3;
	if (ft2232_buffer_size + predicted_size + 1 > FT2232_BUFFER_SIZE) {
		if (ft2232_send_and_recv(first_unsent, cmd) != ERROR_OK)
			retval = ERROR_JTAG_QUEUE_FAILED;
		require_send = 0;
		first_unsent = cmd;
	}

	if ((cmd->cmd.reset->trst == 1) ||
	    (cmd->cmd.reset->srst && (jtag_get_reset_config() & RESET_SRST_PULLS_TRST)))
		tap_set_state(TAP_RESET);

	layout->reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
	require_send = 1;

	DEBUG_JTAG_IO("trst: %i, srst: %i",
		cmd->cmd.reset->trst, cmd->cmd.reset->srst);
	return retval;
}

static int ft2232_execute_sleep(struct jtag_command *cmd)
{
	int retval;
	retval = ERROR_OK;

	DEBUG_JTAG_IO("sleep %" PRIi32, cmd->cmd.sleep->us);

	if (ft2232_send_and_recv(first_unsent, cmd) != ERROR_OK)
		retval = ERROR_JTAG_QUEUE_FAILED;
	first_unsent = cmd->next;
	jtag_sleep(cmd->cmd.sleep->us);
	DEBUG_JTAG_IO("sleep %" PRIi32 " usec while in %s",
		cmd->cmd.sleep->us,
		tap_state_name(tap_get_state()));
	return retval;
}

static int ft2232_execute_stableclocks(struct jtag_command *cmd)
{
	int retval;
	retval = ERROR_OK;

	/* this is only allowed while in a stable state.  A check for a stable
	 * state was done in jtag_add_clocks()
	 */
	if (ft2232_stableclocks(cmd->cmd.stableclocks->num_cycles, cmd) != ERROR_OK)
		retval = ERROR_JTAG_QUEUE_FAILED;
	DEBUG_JTAG_IO("clocks %i while in %s",
		cmd->cmd.stableclocks->num_cycles,
		tap_state_name(tap_get_state()));
	return retval;
}

static int ft2232_execute_command(struct jtag_command *cmd)
{
	int retval;

	switch (cmd->type) {
		case JTAG_RESET:
			retval = ft2232_execute_reset(cmd);
			break;
		case JTAG_RUNTEST:
			retval = ft2232_execute_runtest(cmd);
			break;
		case JTAG_TLR_RESET:
			retval = ft2232_execute_statemove(cmd);
			break;
		case JTAG_PATHMOVE:
			retval = ft2232_execute_pathmove(cmd);
			break;
		case JTAG_SCAN:
			retval = ft2232_execute_scan(cmd);
			break;
		case JTAG_SLEEP:
			retval = ft2232_execute_sleep(cmd);
			break;
		case JTAG_STABLECLOCKS:
			retval = ft2232_execute_stableclocks(cmd);
			break;
		case JTAG_TMS:
			retval = ft2232_execute_tms(cmd);
			break;
		default:
			LOG_ERROR("BUG: unknown JTAG command type encountered");
			retval = ERROR_JTAG_QUEUE_FAILED;
			break;
	}
	return retval;
}

static int ft2232_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;	/* currently processed command */
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

	while (cmd) {
		/* fill the write buffer with the desired command */
		if (ft2232_execute_command(cmd) != ERROR_OK)
			retval = ERROR_JTAG_QUEUE_FAILED;
		/* Start reading input before FT2232 TX buffer fills up.
		 * Sometimes this happens because we don't know the
		 * length of the last command before we execute it. So
		 * we simple inform the user.
		 */
		cmd = cmd->next;

		if (ft2232_expect_read >= FT2232_BUFFER_READ_QUEUE_SIZE) {
			if (ft2232_expect_read > (FT2232_BUFFER_READ_QUEUE_SIZE+1))
				LOG_DEBUG("read buffer size looks too high %d/%d",
					ft2232_expect_read,
					(FT2232_BUFFER_READ_QUEUE_SIZE+1));
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
static int ft2232_init_ftd2xx(uint16_t vid, uint16_t pid, int more, int *try_more)
{
	FT_STATUS status;
	DWORD deviceID;
	char SerialNumber[16];
	char Description[64];
	DWORD openex_flags  = 0;
	char *openex_string = NULL;
	uint8_t latency_timer;

	if (layout == NULL) {
		LOG_WARNING("No ft2232 layout specified'");
		return ERROR_JTAG_INIT_FAILED;
	}

	LOG_DEBUG("'ft2232' interface using FTD2XX with '%s' layout (%4.4x:%4.4x)",
			layout->name, vid, pid);

#if IS_WIN32 == 0
	/* Add non-standard Vid/Pid to the linux driver */
	status = FT_SetVIDPID(vid, pid);
	if (status != FT_OK)
		LOG_WARNING("couldn't add %4.4x:%4.4x", vid, pid);

#endif

	if (ft2232_device_desc && ft2232_serial) {
		LOG_WARNING(
			"can't open by device description and serial number, giving precedence to serial");
		ft2232_device_desc = NULL;
	}

	if (ft2232_device_desc) {
		openex_string = ft2232_device_desc;
		openex_flags  = FT_OPEN_BY_DESCRIPTION;
	} else if (ft2232_serial) {
		openex_string = ft2232_serial;
		openex_flags  = FT_OPEN_BY_SERIAL_NUMBER;
	} else {
		LOG_ERROR("neither device description nor serial number specified");
		LOG_ERROR(
			"please add \"ft2232_device_desc <string>\" or \"ft2232_serial <string>\" to your .cfg file");

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
				LOG_WARNING("Unable to open FTDI Device tried: '%s' and '%s'",
					ft2232_device_desc,
					ft2232_device_desc_A);
			}
		}
	}

	if (status != FT_OK) {
		DWORD num_devices;

		if (more) {
			LOG_WARNING("unable to open ftdi device (trying more): %s",
				ftd2xx_status_string(status));
			*try_more = 1;
			return ERROR_JTAG_INIT_FAILED;
		}
		LOG_ERROR("unable to open ftdi device: %s",
			ftd2xx_status_string(status));
		status = FT_ListDevices(&num_devices, NULL, FT_LIST_NUMBER_ONLY);
		if (status == FT_OK) {
			char **desc_array = malloc(sizeof(char *) * (num_devices + 1));
			uint32_t i;

			for (i = 0; i < num_devices; i++)
				desc_array[i] = malloc(64);

			desc_array[num_devices] = NULL;

			status = FT_ListDevices(desc_array, &num_devices, FT_LIST_ALL | openex_flags);

			if (status == FT_OK) {
				LOG_ERROR("ListDevices: %" PRIu32, (uint32_t)num_devices);
				for (i = 0; i < num_devices; i++)
					LOG_ERROR("%" PRIu32 ": \"%s\"", i, desc_array[i]);
			}

			for (i = 0; i < num_devices; i++)
				free(desc_array[i]);

			free(desc_array);
		} else
			LOG_ERROR("ListDevices: NONE");
		return ERROR_JTAG_INIT_FAILED;
	}

	status = FT_SetLatencyTimer(ftdih, ft2232_latency);
	if (status != FT_OK) {
		LOG_ERROR("unable to set latency timer: %s",
			ftd2xx_status_string(status));
		return ERROR_JTAG_INIT_FAILED;
	}

	status = FT_GetLatencyTimer(ftdih, &latency_timer);
	if (status != FT_OK) {
		/* ftd2xx 1.04 (linux) has a bug when calling FT_GetLatencyTimer
		 * so ignore errors if using this driver version */
		DWORD dw_version;

		status = FT_GetDriverVersion(ftdih, &dw_version);
		LOG_ERROR("unable to get latency timer: %s",
			ftd2xx_status_string(status));

		if ((status == FT_OK) && (dw_version == 0x10004)) {
			LOG_ERROR("ftd2xx 1.04 detected - this has known issues " \
				"with FT_GetLatencyTimer, upgrade to a newer version");
		} else
			return ERROR_JTAG_INIT_FAILED;
	} else
		LOG_DEBUG("current latency timer: %i", latency_timer);

	status = FT_SetTimeouts(ftdih, 5000, 5000);
	if (status != FT_OK) {
		LOG_ERROR("unable to set timeouts: %s",
			ftd2xx_status_string(status));
		return ERROR_JTAG_INIT_FAILED;
	}

	status = FT_SetBitMode(ftdih, 0x0b, 2);
	if (status != FT_OK) {
		LOG_ERROR("unable to enable bit i/o mode: %s",
			ftd2xx_status_string(status));
		return ERROR_JTAG_INIT_FAILED;
	}

	status = FT_GetDeviceInfo(ftdih, &ftdi_device, &deviceID,
			SerialNumber, Description, NULL);
	if (status != FT_OK) {
		LOG_ERROR("unable to get FT_GetDeviceInfo: %s",
			ftd2xx_status_string(status));
		return ERROR_JTAG_INIT_FAILED;
	} else {
		static const char *type_str[] = {
			"BM", "AM", "100AX", "UNKNOWN", "2232C", "232R", "2232H", "4232H", "232H"
		};
		unsigned no_of_known_types = ARRAY_SIZE(type_str) - 1;
		unsigned type_index = ((unsigned)ftdi_device <= no_of_known_types)
			? ftdi_device : FT_DEVICE_UNKNOWN;
		LOG_INFO("device: %" PRIu32 " \"%s\"", (uint32_t)ftdi_device, type_str[type_index]);
		LOG_INFO("deviceID: %" PRIu32, (uint32_t)deviceID);
		LOG_INFO("SerialNumber: %s", SerialNumber);
		LOG_INFO("Description: %s", Description);
	}

	return ERROR_OK;
}

static int ft2232_purge_ftd2xx(void)
{
	FT_STATUS status;

	status = FT_Purge(ftdih, FT_PURGE_RX | FT_PURGE_TX);
	if (status != FT_OK) {
		LOG_ERROR("error purging ftd2xx device: %s",
			ftd2xx_status_string(status));
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

#endif	/* BUILD_FT2232_FTD2XX == 1 */

#if BUILD_FT2232_LIBFTDI == 1
static int ft2232_init_libftdi(uint16_t vid, uint16_t pid, int more, int *try_more, int channel)
{
	uint8_t latency_timer;

	if (layout == NULL) {
		LOG_WARNING("No ft2232 layout specified'");
		return ERROR_JTAG_INIT_FAILED;
	}

	LOG_DEBUG("'ft2232' interface using libftdi with '%s' layout (%4.4x:%4.4x)",
		layout->name, vid, pid);

	if (ftdi_init(&ftdic) < 0)
		return ERROR_JTAG_INIT_FAILED;

	/* default to INTERFACE_A */
	if (channel == INTERFACE_ANY)
		channel = INTERFACE_A;
	if (ftdi_set_interface(&ftdic, channel) < 0) {
		LOG_ERROR("unable to select FT2232 channel A: %s", ftdic.error_str);
		return ERROR_JTAG_INIT_FAILED;
	}

	/* context, vendor id, product id */
	if (ftdi_usb_open_desc(&ftdic, vid, pid, ft2232_device_desc, ft2232_serial) < 0) {
		if (more)
			LOG_WARNING("unable to open ftdi device (trying more): %s",
				ftdic.error_str);
		else
			LOG_ERROR("unable to open ftdi device: %s", ftdic.error_str);
		*try_more = 1;
		return ERROR_JTAG_INIT_FAILED;
	}

	/* There is already a reset in ftdi_usb_open_desc, this should be redundant */
	if (ftdi_usb_reset(&ftdic) < 0) {
		LOG_ERROR("unable to reset ftdi device");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (ftdi_set_latency_timer(&ftdic, ft2232_latency) < 0) {
		LOG_ERROR("unable to set latency timer");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (ftdi_get_latency_timer(&ftdic, &latency_timer) < 0) {
		LOG_ERROR("unable to get latency timer");
		return ERROR_JTAG_INIT_FAILED;
	} else
		LOG_DEBUG("current latency timer: %i", latency_timer);

	ftdi_set_bitmode(&ftdic, 0x0b, 2);	/* ctx, JTAG I/O mask */

	ftdi_device = ftdic.type;
	static const char *type_str[] = {
		"AM", "BM", "2232C", "R", "2232H", "4232H", "232H", "Unknown"
	};
	unsigned no_of_known_types = ARRAY_SIZE(type_str) - 1;
	unsigned type_index = ((unsigned)ftdi_device < no_of_known_types)
		? ftdi_device : no_of_known_types;
	LOG_DEBUG("FTDI chip type: %i \"%s\"", (int)ftdi_device, type_str[type_index]);
	return ERROR_OK;
}

static int ft2232_purge_libftdi(void)
{
	if (ftdi_usb_purge_buffers(&ftdic) < 0) {
		LOG_ERROR("ftdi_purge_buffers: %s", ftdic.error_str);
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

#endif	/* BUILD_FT2232_LIBFTDI == 1 */

static int ft2232_set_data_bits_low_byte(uint8_t value, uint8_t direction)
{
	uint8_t buf[3];
	uint32_t bytes_written;

	buf[0] = 0x80;		/* command "set data bits low byte" */
	buf[1] = value;		/* value */
	buf[2] = direction;	/* direction */

	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if (ft2232_write(buf, sizeof(buf), &bytes_written) != ERROR_OK) {
		LOG_ERROR("couldn't initialize data bits low byte");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int ft2232_set_data_bits_high_byte(uint8_t value, uint8_t direction)
{
	uint8_t buf[3];
	uint32_t bytes_written;

	buf[0] = 0x82;		/* command "set data bits high byte" */
	buf[1] = value;		/* value */
	buf[2] = direction;	/* direction */

	LOG_DEBUG("%2.2x %2.2x %2.2x", buf[0], buf[1], buf[2]);

	if (ft2232_write(buf, sizeof(buf), &bytes_written) != ERROR_OK) {
		LOG_ERROR("couldn't initialize data bits high byte");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int ft2232_init(void)
{
	uint8_t buf[1];
	int retval;
	uint32_t bytes_written;

	LOG_WARNING("Using DEPRECATED interface driver 'ft2232'");
#if BUILD_FTDI
	LOG_INFO("Consider using the 'ftdi' interface driver, with configuration files in interface/ftdi/...");
#endif

	if (tap_get_tms_path_len(TAP_IRPAUSE, TAP_IRPAUSE) == 7)
		LOG_DEBUG("ft2232 interface using 7 step jtag state transitions");
	else
		LOG_DEBUG("ft2232 interface using shortest path jtag state transitions");
	if (layout == NULL) {
		LOG_WARNING("No ft2232 layout specified'");
		return ERROR_JTAG_INIT_FAILED;
	}

	for (int i = 0; 1; i++) {
		/*
		 * "more indicates that there are more IDs to try, so we should
		 * not print an error for an ID mismatch (but for anything
		 * else, we should).
		 *
		 * try_more indicates that the error code returned indicates an
		 * ID mismatch (and nothing else) and that we should proceeed
		 * with the next ID pair.
		 */
		int more = ft2232_vid[i + 1] || ft2232_pid[i + 1];
		int try_more = 0;

#if BUILD_FT2232_FTD2XX == 1
		retval = ft2232_init_ftd2xx(ft2232_vid[i], ft2232_pid[i],
				more, &try_more);
#elif BUILD_FT2232_LIBFTDI == 1
		retval = ft2232_init_libftdi(ft2232_vid[i], ft2232_pid[i],
				more, &try_more, ft2232_channel);
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

	if (ft2232_device_is_highspeed()) {
#ifndef BUILD_FT2232_HIGHSPEED
 #if BUILD_FT2232_FTD2XX == 1
		LOG_WARNING(
			"High Speed device found - You need a newer FTD2XX driver (version 2.04.16 or later)");
 #elif BUILD_FT2232_LIBFTDI == 1
		LOG_WARNING(
			"High Speed device found - You need a newer libftdi version (0.16 or later)");
 #endif
#endif
		/* make sure the legacy mode is disabled */
		if (ftx232h_clk_divide_by_5(false) != ERROR_OK)
			return ERROR_JTAG_INIT_FAILED;
	}

	buf[0] = 0x85;	/* Disconnect TDI/DO to TDO/DI for Loopback */
	retval = ft2232_write(buf, 1, &bytes_written);
	if (retval != ERROR_OK) {
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

/** Updates defaults for DBUS signals:  the four JTAG signals
 * (TCK, TDI, TDO, TMS) and * the four GPIOL signals.
 */
static inline void ftx232_dbus_init(void)
{
	low_output    = 0x08;
	low_direction = 0x0b;
}

/** Initializes DBUS signals:  the four JTAG signals (TCK, TDI, TDO, TMS),
 * the four GPIOL signals.  Initialization covers value and direction,
 * as customized for each layout.
 */
static int ftx232_dbus_write(void)
{
	enum reset_types jtag_reset_config = jtag_get_reset_config();
	if (jtag_reset_config & RESET_TRST_OPEN_DRAIN) {
		low_direction &= ~nTRSTnOE;	/* nTRST input */
		low_output &= ~nTRST;	/* nTRST = 0 */
	} else {
		low_direction |= nTRSTnOE;	/* nTRST output */
		low_output |= nTRST;		/* nTRST = 1 */
	}

	if (jtag_reset_config & RESET_SRST_PUSH_PULL) {
		low_direction |= nSRSTnOE;	/* nSRST output */
		low_output |= nSRST;		/* nSRST = 1 */
	} else {
		low_direction &= ~nSRSTnOE;	/* nSRST input */
		low_output &= ~nSRST;	/* nSRST = 0 */
	}

	/* initialize low byte for jtag */
	if (ft2232_set_data_bits_low_byte(low_output, low_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 DBUS");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int usbjtag_init(void)
{
	/*
	 * NOTE:  This is now _specific_ to the "usbjtag" layout.
	 * Don't try cram any more layouts into this.
	 */
	ftx232_dbus_init();

	nTRST    = 0x10;
	nTRSTnOE = 0x10;
	nSRST    = 0x40;
	nSRSTnOE = 0x40;

	return ftx232_dbus_write();
}

static int lm3s811_jtag_init(void)
{
	ftx232_dbus_init();

	/* There are multiple revisions of LM3S811 eval boards:
	 * - Rev B (and older?) boards have no SWO trace support.
	 * - Rev C boards add ADBUS_6 DBG_ENn and BDBUS_4 SWO_EN;
	 *   they should use the "luminary_icdi" layout instead.
	 */
	nTRST = 0x0;
	nTRSTnOE = 0x00;
	nSRST = 0x20;
	nSRSTnOE = 0x20;
	low_output    = 0x88;
	low_direction = 0x8b;

	return ftx232_dbus_write();
}

static int icdi_jtag_init(void)
{
	ftx232_dbus_init();

	/* Most Luminary eval boards support SWO trace output,
	 * and should use this "luminary_icdi" layout.
	 *
	 * ADBUS 0..3 are used for JTAG as usual.  GPIOs are used
	 * to switch between JTAG and SWD, or switch the ft2232 UART
	 * on the second MPSSE channel/interface (BDBUS)
	 * between (i) the stellaris UART (on Luminary boards)
	 * or (ii) SWO trace data (generic).
	 *
	 * We come up in JTAG mode and may switch to SWD later (with
	 * SWO/trace option if SWD is active).
	 *
	 * DBUS == GPIO-Lx
	 * CBUS == GPIO-Hx
	 */


#define ICDI_JTAG_EN (1 << 7)		/* ADBUS 7 (a.k.a. DBGMOD) */
#define ICDI_DBG_ENn (1 << 6)		/* ADBUS 6 */
#define ICDI_SRST (1 << 5)		/* ADBUS 5 */


	/* GPIOs on second channel/interface (UART) ... */
#define ICDI_SWO_EN (1 << 4)		/* BDBUS 4 */
#define ICDI_TX_SWO (1 << 1)		/* BDBUS 1 */
#define ICDI_VCP_RX (1 << 0)		/* BDBUS 0 (to stellaris UART) */

	nTRST = 0x0;
	nTRSTnOE = 0x00;
	nSRST = ICDI_SRST;
	nSRSTnOE = ICDI_SRST;

	low_direction |= ICDI_JTAG_EN | ICDI_DBG_ENn;
	low_output    |= ICDI_JTAG_EN;
	low_output    &= ~ICDI_DBG_ENn;

	return ftx232_dbus_write();
}

static int signalyzer_init(void)
{
	ftx232_dbus_init();

	nTRST    = 0x10;
	nTRSTnOE = 0x10;
	nSRST    = 0x20;
	nSRSTnOE = 0x20;
	return ftx232_dbus_write();
}

static int axm0432_jtag_init(void)
{
	low_output    = 0x08;
	low_direction = 0x2b;

	/* initialize low byte for jtag */
	if (ft2232_set_data_bits_low_byte(low_output, low_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'JTAGkey' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (strcmp(layout->name, "axm0432_jtag") == 0) {
		nTRST    = 0x08;
		nTRSTnOE = 0x0;		/* No output enable for TRST*/
		nSRST    = 0x04;
		nSRSTnOE = 0x0;		/* No output enable for SRST*/
	} else {
		LOG_ERROR("BUG: axm0432_jtag_init called for non axm0432 layout");
		exit(-1);
	}

	high_output    = 0x0;
	high_direction = 0x0c;

	enum reset_types jtag_reset_config = jtag_get_reset_config();
	if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
		LOG_ERROR("can't set nTRSTOE to push-pull on the Dicarlo jtag");
	else
		high_output |= nTRST;

	if (jtag_reset_config & RESET_SRST_PUSH_PULL)
		LOG_ERROR("can't set nSRST to push-pull on the Dicarlo jtag");
	else
		high_output |= nSRST;

	/* initialize high byte for jtag */
	if (ft2232_set_data_bits_high_byte(high_output, high_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'Dicarlo' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int redbee_init(void)
{
	low_output    = 0x08;
	low_direction = 0x2b;

	/* initialize low byte for jtag */
	if (ft2232_set_data_bits_low_byte(low_output, low_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'redbee' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	nTRST    = 0x08;
	nTRSTnOE = 0x0;		/* No output enable for TRST*/
	nSRST    = 0x04;
	nSRSTnOE = 0x0;		/* No output enable for SRST*/

	high_output    = 0x0;
	high_direction = 0x0c;

	enum reset_types jtag_reset_config = jtag_get_reset_config();
	if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
		LOG_ERROR("can't set nTRSTOE to push-pull on redbee");
	else
		high_output |= nTRST;

	if (jtag_reset_config & RESET_SRST_PUSH_PULL)
		LOG_ERROR("can't set nSRST to push-pull on redbee");
	else
		high_output |= nSRST;

	/* initialize high byte for jtag */
	if (ft2232_set_data_bits_high_byte(high_output, high_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'redbee' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int jtagkey_init(void)
{
	low_output    = 0x08;
	low_direction = 0x1b;

	/* initialize low byte for jtag */
	if (ft2232_set_data_bits_low_byte(low_output, low_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'JTAGkey' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (strcmp(layout->name, "jtagkey") == 0) {
		nTRST    = 0x01;
		nTRSTnOE = 0x4;
		nSRST    = 0x02;
		nSRSTnOE = 0x08;
	} else if ((strcmp(layout->name, "jtagkey_prototype_v1") == 0)
		   || (strcmp(layout->name, "oocdlink") == 0)) {
		nTRST    = 0x02;
		nTRSTnOE = 0x1;
		nSRST    = 0x08;
		nSRSTnOE = 0x04;
	} else {
		LOG_ERROR("BUG: jtagkey_init called for non jtagkey layout");
		exit(-1);
	}

	high_output    = 0x0;
	high_direction = 0x0f;

	enum reset_types jtag_reset_config = jtag_get_reset_config();
	if (jtag_reset_config & RESET_TRST_OPEN_DRAIN) {
		high_output |= nTRSTnOE;
		high_output &= ~nTRST;
	} else {
		high_output &= ~nTRSTnOE;
		high_output |= nTRST;
	}

	if (jtag_reset_config & RESET_SRST_PUSH_PULL) {
		high_output &= ~nSRSTnOE;
		high_output |= nSRST;
	} else {
		high_output |= nSRSTnOE;
		high_output &= ~nSRST;
	}

	/* initialize high byte for jtag */
	if (ft2232_set_data_bits_high_byte(high_output, high_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'JTAGkey' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int olimex_jtag_init(void)
{
	low_output    = 0x08;
	low_direction = 0x1b;

	/* initialize low byte for jtag */
	if (ft2232_set_data_bits_low_byte(low_output, low_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'Olimex' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	nTRST    = 0x01;
	nTRSTnOE = 0x4;
	nSRST    = 0x02;
	nSRSTnOE = 0x00;/* no output enable for nSRST */

	high_output    = 0x0;
	high_direction = 0x0f;

	enum reset_types jtag_reset_config = jtag_get_reset_config();
	if (jtag_reset_config & RESET_TRST_OPEN_DRAIN) {
		high_output |= nTRSTnOE;
		high_output &= ~nTRST;
	} else {
		high_output &= ~nTRSTnOE;
		high_output |= nTRST;
	}

	if (jtag_reset_config & RESET_SRST_PUSH_PULL)
		LOG_ERROR("can't set nSRST to push-pull on the Olimex ARM-USB-OCD");
	else
		high_output &= ~nSRST;

	/* turn red LED on */
	high_output |= 0x08;

	/* initialize high byte for jtag */
	if (ft2232_set_data_bits_high_byte(high_output, high_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'Olimex' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int flyswatter_init(int rev)
{
	low_output    = 0x18;
	low_direction = 0x7b;

	if ((rev < 0) || (rev > 3)) {
		LOG_ERROR("bogus 'flyswatter' revision supplied (%i)", rev);
		return ERROR_JTAG_INIT_FAILED;
	}

	if (rev == 1)
		low_direction |= 1 << 7;

	/* initialize low byte for jtag */
	if (ft2232_set_data_bits_low_byte(low_output, low_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'flyswatter' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	nTRST    = 0x10;
	nTRSTnOE = 0x0;		/* not output enable for nTRST */
	nSRST    = 0x20;
	nSRSTnOE = 0x00;	/* no output enable for nSRST */

	high_output    = 0x00;

	if (rev == 1)
		high_direction = 0x0c;
	else
		high_direction = 0x01;

	/* turn red LED3 on, LED2 off */
	high_output |= 0x08;

	/* initialize high byte for jtag */
	if (ft2232_set_data_bits_high_byte(high_output, high_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'flyswatter' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int flyswatter1_init(void)
{
	return flyswatter_init(1);
}

static int flyswatter2_init(void)
{
	return flyswatter_init(2);
}

static int minimodule_init(void)
{
	low_output    = 0x18;	/* check if srst should be 1 or 0 initially. (0x08) (flyswatter was
				 * 0x18) */
	low_direction = 0xfb;	/* 0xfb; */

	/* initialize low byte for jtag */
	if (ft2232_set_data_bits_low_byte(low_output, low_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'minimodule' layout");
		return ERROR_JTAG_INIT_FAILED;
	}


	nSRST    = 0x20;

	high_output    = 0x00;
	high_direction = 0x05;

	/* turn red LED3 on, LED2 off */
	/* high_output |= 0x08; */

	/* initialize high byte for jtag */
	if (ft2232_set_data_bits_high_byte(high_output, high_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'minimodule' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int turtle_init(void)
{
	low_output    = 0x08;
	low_direction = 0x5b;

	/* initialize low byte for jtag */
	if (ft2232_set_data_bits_low_byte(low_output, low_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'turtelizer2' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	nSRST = 0x40;

	high_output    = 0x00;
	high_direction = 0x0C;

	/* initialize high byte for jtag */
	if (ft2232_set_data_bits_high_byte(high_output, high_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'turtelizer2' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int comstick_init(void)
{
	low_output    = 0x08;
	low_direction = 0x0b;

	/* initialize low byte for jtag */
	if (ft2232_set_data_bits_low_byte(low_output, low_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'comstick' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	nTRST    = 0x01;
	nTRSTnOE = 0x00;	/* no output enable for nTRST */
	nSRST    = 0x02;
	nSRSTnOE = 0x00;	/* no output enable for nSRST */

	high_output    = 0x03;
	high_direction = 0x03;

	/* initialize high byte for jtag */
	if (ft2232_set_data_bits_high_byte(high_output, high_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'comstick' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int stm32stick_init(void)
{
	low_output    = 0x88;
	low_direction = 0x8b;

	/* initialize low byte for jtag */
	if (ft2232_set_data_bits_low_byte(low_output, low_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'stm32stick' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	nTRST    = 0x01;
	nTRSTnOE = 0x00;	/* no output enable for nTRST */
	nSRST    = 0x80;
	nSRSTnOE = 0x00;	/* no output enable for nSRST */

	high_output    = 0x01;
	high_direction = 0x03;

	/* initialize high byte for jtag */
	if (ft2232_set_data_bits_high_byte(high_output, high_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'stm32stick' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int sheevaplug_init(void)
{
	low_output = 0x08;
	low_direction = 0x1b;

	/* initialize low byte for jtag */
	if (ft2232_set_data_bits_low_byte(low_output, low_direction) != ERROR_OK) {
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

	/* initialize high byte for jtag */
	if (ft2232_set_data_bits_high_byte(high_output, high_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'sheevaplug' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int cortino_jtag_init(void)
{
	low_output    = 0x08;
	low_direction = 0x1b;

	/* initialize low byte for jtag */
	if (ft2232_set_data_bits_low_byte(low_output, low_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'cortino' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	nTRST    = 0x01;
	nTRSTnOE = 0x00;	/* no output enable for nTRST */
	nSRST    = 0x02;
	nSRSTnOE = 0x00;	/* no output enable for nSRST */

	high_output    = 0x03;
	high_direction = 0x03;

	/* initialize high byte for jtag */
	if (ft2232_set_data_bits_high_byte(high_output, high_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'cortino' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int lisa_l_init(void)
{
	ftx232_dbus_init();

	nTRST    = 0x10;
	nTRSTnOE = 0x10;
	nSRST    = 0x40;
	nSRSTnOE = 0x40;

	high_output = 0x00;
	high_direction = 0x18;

	/* initialize high byte for jtag */
	if (ft2232_set_data_bits_high_byte(high_output, high_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'lisa_l' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ftx232_dbus_write();
}

static int flossjtag_init(void)
{
	ftx232_dbus_init();

	nTRST    = 0x10;
	nTRSTnOE = 0x10;
	nSRST    = 0x40;
	nSRSTnOE = 0x40;

	high_output = 0x00;
	high_direction = 0x18;

	/* initialize high byte for jtag */
	if (ft2232_set_data_bits_high_byte(high_output, high_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'Floss-JTAG' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ftx232_dbus_write();
}

/*
 * The reference schematic from TI for the XDS100v2 has a CPLD on which opens
 * the door for a number of different configurations
 *
 * Known Implementations:
 * http://processors.wiki.ti.com/images/9/93/TMS570LS20216_USB_STICK_Schematic.pdf
 *
 * http://processors.wiki.ti.com/index.php/XDS100 (rev2)
 *	* CLPD logic: Rising edge to enable outputs (XDS100_PWR_RST)
 *		* ACBUS3 to transition 0->1 (OE rising edge)
 *	* CPLD logic: Put the EMU0/1 pins in Hi-Z:
 *		* ADBUS5/GPIOL1 = EMU_EN = 1
 *		* ADBUS6/GPIOL2 = EMU0 = 0
 *		* ACBUS4/SPARE0 = EMU1 = 0
 *	* CPLD logic: Disable loopback
 *		* ACBUS6/SPARE2 = LOOPBACK = 0
 */
#define XDS100_nEMU_EN  (1<<5)
#define XDS100_nEMU0    (1<<6)

#define XDS100_PWR_RST  (1<<3)
#define XDS100_nEMU1    (1<<4)
#define XDS100_LOOPBACK (1<<6)
static int xds100v2_init(void)
{
	/* These are in the lower byte */
	nTRST    = 0x10;
	nTRSTnOE = 0x10;

	/* These aren't actually used on 14 pin connectors
	 * These are in the upper byte */
	nSRST    = 0x01;
	nSRSTnOE = 0x01;

	low_output    = 0x08 | nTRST | XDS100_nEMU_EN;
	low_direction = 0x0b | nTRSTnOE | XDS100_nEMU_EN | XDS100_nEMU0;

	if (ft2232_set_data_bits_low_byte(low_output, low_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'xds100v2' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	high_output = 0;
	high_direction = nSRSTnOE | XDS100_LOOPBACK | XDS100_PWR_RST | XDS100_nEMU1;

	/* initialize high byte for jtag */
	if (ft2232_set_data_bits_high_byte(high_output, high_direction) != ERROR_OK) {
		LOG_ERROR("couldn't put CPLD in to reset with 'xds100v2' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	high_output |= XDS100_PWR_RST;

	/* initialize high byte for jtag */
	if (ft2232_set_data_bits_high_byte(high_output, high_direction) != ERROR_OK) {
		LOG_ERROR("couldn't bring CPLD out of reset with 'xds100v2' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static void olimex_jtag_blink(void)
{
	/* Olimex ARM-USB-OCD has a LED connected to ACBUS3
	 * ACBUS3 is bit 3 of the GPIOH port
	 */
	high_output ^= 0x08;

	buffer_write(0x82);
	buffer_write(high_output);
	buffer_write(high_direction);
}

static void flyswatter_jtag_blink(unsigned char led)
{
	buffer_write(0x82);
	buffer_write(high_output ^ led);
	buffer_write(high_direction);
}

static void flyswatter1_jtag_blink(void)
{
	/*
	 * Flyswatter has two LEDs connected to ACBUS2 and ACBUS3
	 */
	flyswatter_jtag_blink(0xc);
}

static void flyswatter2_jtag_blink(void)
{
	/*
	 * Flyswatter2 only has one LED connected to ACBUS2
	 */
	flyswatter_jtag_blink(0x4);
}

static void turtle_jtag_blink(void)
{
	/*
	 * Turtelizer2 has two LEDs connected to ACBUS2 and ACBUS3
	 */
	if (high_output & 0x08)
		high_output = 0x04;
	else
		high_output = 0x08;

	buffer_write(0x82);
	buffer_write(high_output);
	buffer_write(high_direction);
}

static void lisa_l_blink(void)
{
	/*
	 * Lisa/L has two LEDs connected to BCBUS3 and BCBUS4
	 */
	if (high_output & 0x10)
		high_output = 0x08;
	else
		high_output = 0x10;

	buffer_write(0x82);
	buffer_write(high_output);
	buffer_write(high_direction);
}

static void flossjtag_blink(void)
{
	/*
	 * Floss-JTAG has two LEDs connected to ACBUS3 and ACBUS4
	 */
	if (high_output & 0x10)
		high_output = 0x08;
	else
		high_output = 0x10;

	buffer_write(0x82);
	buffer_write(high_output);
	buffer_write(high_direction);
}

static int ft2232_quit(void)
{
#if BUILD_FT2232_FTD2XX == 1

	FT_Close(ftdih);
#elif BUILD_FT2232_LIBFTDI == 1
	ftdi_usb_close(&ftdic);

	ftdi_deinit(&ftdic);
#endif

	free(ft2232_buffer);
	ft2232_buffer = NULL;

	return ERROR_OK;
}

COMMAND_HANDLER(ft2232_handle_device_desc_command)
{
	char *cp;
	char buf[200];
	if (CMD_ARGC == 1) {
		ft2232_device_desc = strdup(CMD_ARGV[0]);
		cp = strchr(ft2232_device_desc, 0);
		/* under Win32, the FTD2XX driver appends an "A" to the end
		 * of the description, this examines the given desc
		 * and creates the 'missing' _A or non_A variable. */
		if ((cp[-1] == 'A') && (cp[-2] == ' ')) {
			/* it was, so make this the "A" version. */
			ft2232_device_desc_A = ft2232_device_desc;
			/* and *CREATE* the non-A version. */
			strcpy(buf, ft2232_device_desc);
			cp = strchr(buf, 0);
			cp[-2] = 0;
			ft2232_device_desc = strdup(buf);
		} else {
			/* <space > A not defined
			 * so create it */
			sprintf(buf, "%s A", ft2232_device_desc);
			ft2232_device_desc_A = strdup(buf);
		}
	} else
		LOG_ERROR("expected exactly one argument to ft2232_device_desc <description>");

	return ERROR_OK;
}

COMMAND_HANDLER(ft2232_handle_serial_command)
{
	if (CMD_ARGC == 1)
		ft2232_serial = strdup(CMD_ARGV[0]);
	else
		return ERROR_COMMAND_SYNTAX_ERROR;

	return ERROR_OK;
}

COMMAND_HANDLER(ft2232_handle_layout_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (layout) {
		LOG_ERROR("already specified ft2232_layout %s",
			layout->name);
		return (strcmp(layout->name, CMD_ARGV[0]) != 0)
		       ? ERROR_FAIL
		       : ERROR_OK;
	}

	for (const struct ft2232_layout *l = ft2232_layouts; l->name; l++) {
		if (strcmp(l->name, CMD_ARGV[0]) == 0) {
			layout = l;
			ft2232_channel = l->channel;
			return ERROR_OK;
		}
	}

	LOG_ERROR("No FT2232 layout '%s' found", CMD_ARGV[0]);
	return ERROR_FAIL;
}

COMMAND_HANDLER(ft2232_handle_vid_pid_command)
{
	if (CMD_ARGC > MAX_USB_IDS * 2) {
		LOG_WARNING("ignoring extra IDs in ft2232_vid_pid "
			"(maximum is %d pairs)", MAX_USB_IDS);
		CMD_ARGC = MAX_USB_IDS * 2;
	}
	if (CMD_ARGC < 2 || (CMD_ARGC & 1)) {
		LOG_WARNING("incomplete ft2232_vid_pid configuration directive");
		if (CMD_ARGC < 2)
			return ERROR_COMMAND_SYNTAX_ERROR;
		/* remove the incomplete trailing id */
		CMD_ARGC -= 1;
	}

	unsigned i;
	for (i = 0; i < CMD_ARGC; i += 2) {
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i], ft2232_vid[i >> 1]);
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1], ft2232_pid[i >> 1]);
	}

	/*
	 * Explicitly terminate, in case there are multiples instances of
	 * ft2232_vid_pid.
	 */
	ft2232_vid[i >> 1] = ft2232_pid[i >> 1] = 0;

	return ERROR_OK;
}

COMMAND_HANDLER(ft2232_handle_latency_command)
{
	if (CMD_ARGC == 1)
		ft2232_latency = atoi(CMD_ARGV[0]);
	else
		return ERROR_COMMAND_SYNTAX_ERROR;

	return ERROR_OK;
}

COMMAND_HANDLER(ft2232_handle_channel_command)
{
	if (CMD_ARGC == 1) {
		ft2232_channel = atoi(CMD_ARGV[0]);
		if (ft2232_channel < 0 || ft2232_channel > 4)
			LOG_ERROR("ft2232_channel must be in the 0 to 4 range");
	} else
		LOG_ERROR("expected exactly one argument to ft2232_channel <ch>");

	return ERROR_OK;
}

static int ft2232_stableclocks(int num_cycles, struct jtag_command *cmd)
{
	int retval = 0;

	/* 7 bits of either ones or zeros. */
	uint8_t tms = (tap_get_state() == TAP_RESET ? 0x7F : 0x00);

	while (num_cycles > 0) {
		/* the command 0x4b, "Clock Data to TMS/CS Pin (no Read)" handles
		 * at most 7 bits per invocation.  Here we invoke it potentially
		 * several times.
		 */
		int bitcount_per_command = (num_cycles > 7) ? 7 : num_cycles;

		if (ft2232_buffer_size + 3 >= FT2232_BUFFER_SIZE) {
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
static int icebear_jtag_init(void)
{
	low_direction   = 0x0b;	/* output: TCK TDI TMS; input: TDO */
	low_output      = 0x08;	/* high: TMS; low: TCK TDI */
	nTRST           = 0x10;
	nSRST           = 0x20;

	enum reset_types jtag_reset_config = jtag_get_reset_config();
	if ((jtag_reset_config & RESET_TRST_OPEN_DRAIN) != 0)
		low_direction   &= ~nTRST;	/* nTRST high impedance */
	else {
		low_direction   |= nTRST;
		low_output      |= nTRST;
	}

	low_direction   |= nSRST;
	low_output      |= nSRST;

	/* initialize low byte for jtag */
	if (ft2232_set_data_bits_low_byte(low_output, low_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'IceBear' layout (low)");
		return ERROR_JTAG_INIT_FAILED;
	}

	high_output    = 0x0;
	high_direction = 0x00;

	/* initialize high byte for jtag */
	if (ft2232_set_data_bits_high_byte(high_output, high_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'IceBear' layout (high)");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static void icebear_jtag_reset(int trst, int srst)
{
	if (trst == 1) {
		low_direction   |= nTRST;
		low_output      &= ~nTRST;
	} else if (trst == 0) {
		enum reset_types jtag_reset_config = jtag_get_reset_config();
		if ((jtag_reset_config & RESET_TRST_OPEN_DRAIN) != 0)
			low_direction   &= ~nTRST;
		else
			low_output      |= nTRST;
	}

	if (srst == 1)
		low_output &= ~nSRST;
	else if (srst == 0)
		low_output |= nSRST;

	/* command "set data bits low byte" */
	buffer_write(0x80);
	buffer_write(low_output);
	buffer_write(low_direction);

	LOG_DEBUG("trst: %i, srst: %i, low_output: 0x%2.2x, low_direction: 0x%2.2x",
		trst,
		srst,
		low_output,
		low_direction);
}

/* ---------------------------------------------------------------------
 * Support for Signalyzer H2 and Signalyzer H4
 * JTAG adapter from Xverve Technologies Inc.
 * http://www.signalyzer.com or http://www.xverve.com
 *
 * Author: Oleg Seiljus, oleg@signalyzer.com
 */
static unsigned char signalyzer_h_side;
static unsigned int signalyzer_h_adapter_type;

static int signalyzer_h_ctrl_write(int address, unsigned short value);

#if BUILD_FT2232_FTD2XX == 1
static int signalyzer_h_ctrl_read(int address, unsigned short *value);
#endif

#define SIGNALYZER_COMMAND_ADDR					128
#define SIGNALYZER_DATA_BUFFER_ADDR				129

#define SIGNALYZER_COMMAND_VERSION				0x41
#define SIGNALYZER_COMMAND_RESET				0x42
#define SIGNALYZER_COMMAND_POWERCONTROL_GET		0x50
#define SIGNALYZER_COMMAND_POWERCONTROL_SET		0x51
#define SIGNALYZER_COMMAND_PWM_SET				0x52
#define SIGNALYZER_COMMAND_LED_SET				0x53
#define SIGNALYZER_COMMAND_ADC					0x54
#define SIGNALYZER_COMMAND_GPIO_STATE			0x55
#define SIGNALYZER_COMMAND_GPIO_MODE			0x56
#define SIGNALYZER_COMMAND_GPIO_PORT			0x57
#define SIGNALYZER_COMMAND_I2C					0x58

#define SIGNALYZER_CHAN_A						1
#define SIGNALYZER_CHAN_B						2
/* LEDS use channel C */
#define SIGNALYZER_CHAN_C						4

#define SIGNALYZER_LED_GREEN					1
#define SIGNALYZER_LED_RED						2

#define SIGNALYZER_MODULE_TYPE_EM_LT16_A		0x0301
#define SIGNALYZER_MODULE_TYPE_EM_ARM_JTAG		0x0302
#define SIGNALYZER_MODULE_TYPE_EM_JTAG			0x0303
#define SIGNALYZER_MODULE_TYPE_EM_ARM_JTAG_P	0x0304
#define SIGNALYZER_MODULE_TYPE_EM_JTAG_P		0x0305


static int signalyzer_h_ctrl_write(int address, unsigned short value)
{
#if BUILD_FT2232_FTD2XX == 1
	return FT_WriteEE(ftdih, address, value);
#elif BUILD_FT2232_LIBFTDI == 1
	return 0;
#endif
}

#if BUILD_FT2232_FTD2XX == 1
static int signalyzer_h_ctrl_read(int address, unsigned short *value)
{
	return FT_ReadEE(ftdih, address, value);
}
#endif

static int signalyzer_h_led_set(unsigned char channel, unsigned char led,
	int on_time_ms, int off_time_ms, unsigned char cycles)
{
	unsigned char on_time;
	unsigned char off_time;

	if (on_time_ms < 0xFFFF)
		on_time = (unsigned char)(on_time_ms / 62);
	else
		on_time = 0xFF;

	off_time = (unsigned char)(off_time_ms / 62);

#if BUILD_FT2232_FTD2XX == 1
	FT_STATUS status;

	status = signalyzer_h_ctrl_write(SIGNALYZER_DATA_BUFFER_ADDR,
			((uint32_t)(channel << 8) | led));
	if (status != FT_OK) {
		LOG_ERROR("signalyzer_h_ctrl_write  returned: %s",
			ftd2xx_status_string(status));
		return ERROR_JTAG_DEVICE_ERROR;
	}

	status = signalyzer_h_ctrl_write((SIGNALYZER_DATA_BUFFER_ADDR + 1),
			((uint32_t)(on_time << 8) | off_time));
	if (status != FT_OK) {
		LOG_ERROR("signalyzer_h_ctrl_write  returned: %s",
			ftd2xx_status_string(status));
		return ERROR_JTAG_DEVICE_ERROR;
	}

	status = signalyzer_h_ctrl_write((SIGNALYZER_DATA_BUFFER_ADDR + 2),
			((uint32_t)cycles));
	if (status != FT_OK) {
		LOG_ERROR("signalyzer_h_ctrl_write  returned: %s",
			ftd2xx_status_string(status));
		return ERROR_JTAG_DEVICE_ERROR;
	}

	status = signalyzer_h_ctrl_write(SIGNALYZER_COMMAND_ADDR,
				SIGNALYZER_COMMAND_LED_SET);
	if (status != FT_OK) {
		LOG_ERROR("signalyzer_h_ctrl_write  returned: %s",
			ftd2xx_status_string(status));
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
#elif BUILD_FT2232_LIBFTDI == 1
	int retval;

	retval = signalyzer_h_ctrl_write(SIGNALYZER_DATA_BUFFER_ADDR,
				((uint32_t)(channel << 8) | led));
	if (retval < 0) {
		LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
			ftdi_get_error_string(&ftdic));
		return ERROR_JTAG_DEVICE_ERROR;
	}

	retval = signalyzer_h_ctrl_write((SIGNALYZER_DATA_BUFFER_ADDR + 1),
			((uint32_t)(on_time << 8) | off_time));
	if (retval < 0) {
		LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
			ftdi_get_error_string(&ftdic));
		return ERROR_JTAG_DEVICE_ERROR;
	}

	retval = signalyzer_h_ctrl_write((SIGNALYZER_DATA_BUFFER_ADDR + 2),
			(uint32_t)cycles);
	if (retval < 0) {
		LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
			ftdi_get_error_string(&ftdic));
		return ERROR_JTAG_DEVICE_ERROR;
	}

	retval = signalyzer_h_ctrl_write(SIGNALYZER_COMMAND_ADDR,
			SIGNALYZER_COMMAND_LED_SET);
	if (retval < 0) {
		LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
			ftdi_get_error_string(&ftdic));
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
#endif
}

static int signalyzer_h_init(void)
{
#if BUILD_FT2232_FTD2XX == 1
	FT_STATUS status;
	int i;
#endif

	char *end_of_desc;

	uint16_t read_buf[12] = { 0 };

	/* turn on center green led */
	signalyzer_h_led_set(SIGNALYZER_CHAN_C, SIGNALYZER_LED_GREEN,
		0xFFFF, 0x00, 0x00);

	/* determine what channel config wants to open
	 * TODO: change me... current implementation is made to work
	 * with openocd description parsing.
	 */
	end_of_desc = strrchr(ft2232_device_desc, 0x00);

	if (end_of_desc) {
		signalyzer_h_side = *(end_of_desc - 1);
		if (signalyzer_h_side == 'B')
			signalyzer_h_side = SIGNALYZER_CHAN_B;
		else
			signalyzer_h_side = SIGNALYZER_CHAN_A;
	} else {
		LOG_ERROR("No Channel was specified");
		return ERROR_FAIL;
	}

	signalyzer_h_led_set(signalyzer_h_side, SIGNALYZER_LED_GREEN,
		1000, 1000, 0xFF);

#if BUILD_FT2232_FTD2XX == 1
	/* read signalyzer versionining information */
	status = signalyzer_h_ctrl_write(SIGNALYZER_COMMAND_ADDR,
			SIGNALYZER_COMMAND_VERSION);
	if (status != FT_OK) {
		LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
			ftd2xx_status_string(status));
		return ERROR_JTAG_DEVICE_ERROR;
	}

	for (i = 0; i < 10; i++) {
		status = signalyzer_h_ctrl_read((SIGNALYZER_DATA_BUFFER_ADDR + i),
				&read_buf[i]);
		if (status != FT_OK) {
			LOG_ERROR("signalyzer_h_ctrl_read returned: %s",
				ftd2xx_status_string(status));
			return ERROR_JTAG_DEVICE_ERROR;
		}
	}

	LOG_INFO("Signalyzer: ID info: { %.4x %.4x %.4x %.4x %.4x %.4x %.4x }",
		read_buf[0], read_buf[1], read_buf[2], read_buf[3],
		read_buf[4], read_buf[5], read_buf[6]);

	/* set gpio register */
	status = signalyzer_h_ctrl_write(SIGNALYZER_DATA_BUFFER_ADDR,
			(uint32_t)(signalyzer_h_side << 8));
	if (status != FT_OK) {
		LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
			ftd2xx_status_string(status));
		return ERROR_JTAG_DEVICE_ERROR;
	}

	status = signalyzer_h_ctrl_write(SIGNALYZER_DATA_BUFFER_ADDR + 1, 0x0404);
	if (status != FT_OK) {
		LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
			ftd2xx_status_string(status));
		return ERROR_JTAG_DEVICE_ERROR;
	}

	status = signalyzer_h_ctrl_write(SIGNALYZER_COMMAND_ADDR,
			SIGNALYZER_COMMAND_GPIO_STATE);
	if (status != FT_OK) {
		LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
			ftd2xx_status_string(status));
		return ERROR_JTAG_DEVICE_ERROR;
	}

	/* read adapter type information */
	status = signalyzer_h_ctrl_write(SIGNALYZER_DATA_BUFFER_ADDR,
			((uint32_t)(signalyzer_h_side << 8) | 0x01));
	if (status != FT_OK) {
		LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
			ftd2xx_status_string(status));
		return ERROR_JTAG_DEVICE_ERROR;
	}

	status = signalyzer_h_ctrl_write(
			(SIGNALYZER_DATA_BUFFER_ADDR + 1), 0xA000);
	if (status != FT_OK) {
		LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
			ftd2xx_status_string(status));
		return ERROR_JTAG_DEVICE_ERROR;
	}

	status = signalyzer_h_ctrl_write(
			(SIGNALYZER_DATA_BUFFER_ADDR + 2), 0x0008);
	if (status != FT_OK) {
		LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
			ftd2xx_status_string(status));
		return ERROR_JTAG_DEVICE_ERROR;
	}

	status = signalyzer_h_ctrl_write(SIGNALYZER_COMMAND_ADDR,
			SIGNALYZER_COMMAND_I2C);
	if (status != FT_OK) {
		LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
			ftd2xx_status_string(status));
		return ERROR_JTAG_DEVICE_ERROR;
	}

	usleep(100000);

	status = signalyzer_h_ctrl_read(SIGNALYZER_COMMAND_ADDR, &read_buf[0]);
	if (status != FT_OK) {
		LOG_ERROR("signalyzer_h_ctrl_read returned: %s",
			ftd2xx_status_string(status));
		return ERROR_JTAG_DEVICE_ERROR;
	}

	if (read_buf[0] != 0x0498)
		signalyzer_h_adapter_type = 0x0000;
	else {
		for (i = 0; i < 4; i++) {
			status = signalyzer_h_ctrl_read((SIGNALYZER_DATA_BUFFER_ADDR + i), &read_buf[i]);
			if (status != FT_OK) {
				LOG_ERROR("signalyzer_h_ctrl_read returned: %s",
					ftd2xx_status_string(status));
				return ERROR_JTAG_DEVICE_ERROR;
			}
		}

		signalyzer_h_adapter_type = read_buf[0];
	}

#elif BUILD_FT2232_LIBFTDI == 1
	/* currently libftdi does not allow reading individual eeprom
	 * locations, therefore adapter type cannot be detected.
	 * override with most common type
	 */
	signalyzer_h_adapter_type = SIGNALYZER_MODULE_TYPE_EM_ARM_JTAG;
#endif

	enum reset_types jtag_reset_config = jtag_get_reset_config();

	/* ADAPTOR: EM_LT16_A */
	if (signalyzer_h_adapter_type == SIGNALYZER_MODULE_TYPE_EM_LT16_A) {
		LOG_INFO("Signalyzer: EM-LT (16-channel level translator) "
			"detected. (HW: %2x).", (read_buf[1] >> 8));

		nTRST    = 0x10;
		nTRSTnOE = 0x10;
		nSRST    = 0x20;
		nSRSTnOE = 0x20;

		low_output     = 0x08;
		low_direction  = 0x1b;

		high_output    = 0x0;
		high_direction = 0x0;

		if (jtag_reset_config & RESET_TRST_OPEN_DRAIN) {
			low_direction &= ~nTRSTnOE;	/* nTRST input */
			low_output    &= ~nTRST;	/* nTRST = 0 */
		} else {
			low_direction |= nTRSTnOE;	/* nTRST output */
			low_output    |= nTRST;		/* nTRST = 1 */
		}

		if (jtag_reset_config & RESET_SRST_PUSH_PULL) {
			low_direction |= nSRSTnOE;	/* nSRST output */
			low_output    |= nSRST;		/* nSRST = 1 */
		} else {
			low_direction &= ~nSRSTnOE;	/* nSRST input */
			low_output    &= ~nSRST;	/* nSRST = 0 */
		}

#if BUILD_FT2232_FTD2XX == 1
		/* enable power to the module */
		status = signalyzer_h_ctrl_write(SIGNALYZER_DATA_BUFFER_ADDR,
				((uint32_t)(signalyzer_h_side << 8) | 0x01));
		if (status != FT_OK) {
			LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
				ftd2xx_status_string(status));
			return ERROR_JTAG_DEVICE_ERROR;
		}

		status = signalyzer_h_ctrl_write(SIGNALYZER_COMMAND_ADDR,
				SIGNALYZER_COMMAND_POWERCONTROL_SET);
		if (status != FT_OK) {
			LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
				ftd2xx_status_string(status));
			return ERROR_JTAG_DEVICE_ERROR;
		}

		/* set gpio mode register */
		status = signalyzer_h_ctrl_write(SIGNALYZER_DATA_BUFFER_ADDR,
				(uint32_t)(signalyzer_h_side << 8));
		if (status != FT_OK) {
			LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
				ftd2xx_status_string(status));
			return ERROR_JTAG_DEVICE_ERROR;
		}

		status = signalyzer_h_ctrl_write(SIGNALYZER_DATA_BUFFER_ADDR + 1, 0x0000);
		if (status != FT_OK) {
			LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
				ftd2xx_status_string(status));
			return ERROR_JTAG_DEVICE_ERROR;
		}

		status = signalyzer_h_ctrl_write(SIGNALYZER_COMMAND_ADDR, SIGNALYZER_COMMAND_GPIO_MODE);
		if (status != FT_OK) {
			LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
				ftd2xx_status_string(status));
			return ERROR_JTAG_DEVICE_ERROR;
		}

		/* set gpio register */
		status = signalyzer_h_ctrl_write(SIGNALYZER_DATA_BUFFER_ADDR,
				(uint32_t)(signalyzer_h_side << 8));
		if (status != FT_OK) {
			LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
				ftd2xx_status_string(status));
			return ERROR_JTAG_DEVICE_ERROR;
		}

		status = signalyzer_h_ctrl_write(SIGNALYZER_DATA_BUFFER_ADDR + 1, 0x4040);
		if (status != FT_OK) {
			LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
				ftd2xx_status_string(status));
			return ERROR_JTAG_DEVICE_ERROR;
		}

		status = signalyzer_h_ctrl_write(SIGNALYZER_COMMAND_ADDR,
				SIGNALYZER_COMMAND_GPIO_STATE);
		if (status != FT_OK) {
			LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
				ftd2xx_status_string(status));
			return ERROR_JTAG_DEVICE_ERROR;
		}
#endif
	}
	/* ADAPTOR: EM_ARM_JTAG, EM_ARM_JTAG_P, EM_JTAG, EM_JTAG_P */
	else if ((signalyzer_h_adapter_type == SIGNALYZER_MODULE_TYPE_EM_ARM_JTAG) ||
		 (signalyzer_h_adapter_type == SIGNALYZER_MODULE_TYPE_EM_ARM_JTAG_P) ||
		 (signalyzer_h_adapter_type == SIGNALYZER_MODULE_TYPE_EM_JTAG)  ||
		 (signalyzer_h_adapter_type == SIGNALYZER_MODULE_TYPE_EM_JTAG_P)) {
		if (signalyzer_h_adapter_type
		    == SIGNALYZER_MODULE_TYPE_EM_ARM_JTAG)
			LOG_INFO("Signalyzer: EM-ARM-JTAG (ARM JTAG) "
				"detected. (HW: %2x).", (read_buf[1] >> 8));
		else if (signalyzer_h_adapter_type
			 == SIGNALYZER_MODULE_TYPE_EM_ARM_JTAG_P)
			LOG_INFO("Signalyzer: EM-ARM-JTAG_P "
				"(ARM JTAG with PSU) detected. (HW: %2x).",
				(read_buf[1] >> 8));
		else if (signalyzer_h_adapter_type
			 == SIGNALYZER_MODULE_TYPE_EM_JTAG)
			LOG_INFO("Signalyzer: EM-JTAG (Generic JTAG) "
				"detected. (HW: %2x).", (read_buf[1] >> 8));
		else if (signalyzer_h_adapter_type
			 == SIGNALYZER_MODULE_TYPE_EM_JTAG_P)
			LOG_INFO("Signalyzer: EM-JTAG-P "
				"(Generic JTAG with PSU) detected. (HW: %2x).",
				(read_buf[1] >> 8));

		nTRST          = 0x02;
		nTRSTnOE       = 0x04;
		nSRST          = 0x08;
		nSRSTnOE       = 0x10;

		low_output     = 0x08;
		low_direction  = 0x1b;

		high_output    = 0x0;
		high_direction = 0x1f;

		if (jtag_reset_config & RESET_TRST_OPEN_DRAIN) {
			high_output |= nTRSTnOE;
			high_output &= ~nTRST;
		} else {
			high_output &= ~nTRSTnOE;
			high_output |= nTRST;
		}

		if (jtag_reset_config & RESET_SRST_PUSH_PULL) {
			high_output &= ~nSRSTnOE;
			high_output |= nSRST;
		} else {
			high_output |= nSRSTnOE;
			high_output &= ~nSRST;
		}

#if BUILD_FT2232_FTD2XX == 1
		/* enable power to the module */
		status = signalyzer_h_ctrl_write(SIGNALYZER_DATA_BUFFER_ADDR,
				((uint32_t)(signalyzer_h_side << 8) | 0x01));
		if (status != FT_OK) {
			LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
				ftd2xx_status_string(status));
			return ERROR_JTAG_DEVICE_ERROR;
		}

		status = signalyzer_h_ctrl_write(SIGNALYZER_COMMAND_ADDR,
				SIGNALYZER_COMMAND_POWERCONTROL_SET);
		if (status != FT_OK) {
			LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
				ftd2xx_status_string(status));
			return ERROR_JTAG_DEVICE_ERROR;
		}

		/* set gpio mode register (IO_16 and IO_17 set as analog
		 * inputs, other is gpio)
		 */
		status = signalyzer_h_ctrl_write(SIGNALYZER_DATA_BUFFER_ADDR,
				(uint32_t)(signalyzer_h_side << 8));
		if (status != FT_OK) {
			LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
				ftd2xx_status_string(status));
			return ERROR_JTAG_DEVICE_ERROR;
		}

		status = signalyzer_h_ctrl_write(SIGNALYZER_DATA_BUFFER_ADDR + 1, 0x0060);
		if (status != FT_OK) {
			LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
				ftd2xx_status_string(status));
			return ERROR_JTAG_DEVICE_ERROR;
		}

		status = signalyzer_h_ctrl_write(SIGNALYZER_COMMAND_ADDR, SIGNALYZER_COMMAND_GPIO_MODE);
		if (status != FT_OK) {
			LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
				ftd2xx_status_string(status));
			return ERROR_JTAG_DEVICE_ERROR;
		}

		/* set gpio register (all inputs, for -P modules,
		 * PSU will be turned off)
		 */
		status = signalyzer_h_ctrl_write(SIGNALYZER_DATA_BUFFER_ADDR,
				(uint32_t)(signalyzer_h_side << 8));
		if (status != FT_OK) {
			LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
				ftd2xx_status_string(status));
			return ERROR_JTAG_DEVICE_ERROR;
		}

		status = signalyzer_h_ctrl_write(SIGNALYZER_DATA_BUFFER_ADDR + 1, 0x0000);
		if (status != FT_OK) {
			LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
				ftd2xx_status_string(status));
			return ERROR_JTAG_DEVICE_ERROR;
		}

		status = signalyzer_h_ctrl_write(SIGNALYZER_COMMAND_ADDR, SIGNALYZER_COMMAND_GPIO_STATE);
		if (status != FT_OK) {
			LOG_ERROR("signalyzer_h_ctrl_write returned: %s",
				ftd2xx_status_string(status));
			return ERROR_JTAG_DEVICE_ERROR;
		}
#endif
	} else if (signalyzer_h_adapter_type == 0x0000) {
		LOG_INFO("Signalyzer: No external modules were detected.");

		nTRST    = 0x10;
		nTRSTnOE = 0x10;
		nSRST    = 0x20;
		nSRSTnOE = 0x20;

		low_output     = 0x08;
		low_direction  = 0x1b;

		high_output    = 0x0;
		high_direction = 0x0;

		if (jtag_reset_config & RESET_TRST_OPEN_DRAIN) {
			low_direction &= ~nTRSTnOE;	/* nTRST input */
			low_output    &= ~nTRST;	/* nTRST = 0 */
		} else {
			low_direction |= nTRSTnOE;	/* nTRST output */
			low_output    |= nTRST;		/* nTRST = 1 */
		}

		if (jtag_reset_config & RESET_SRST_PUSH_PULL) {
			low_direction |= nSRSTnOE;	/* nSRST output */
			low_output    |= nSRST;		/* nSRST = 1 */
		} else {
			low_direction &= ~nSRSTnOE;	/* nSRST input */
			low_output    &= ~nSRST;	/* nSRST = 0 */
		}
	} else {
		LOG_ERROR("Unknown module type is detected: %.4x",
			signalyzer_h_adapter_type);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	/* initialize low byte of controller for jtag operation */
	if (ft2232_set_data_bits_low_byte(low_output, low_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize Signalyzer-H layout");
		return ERROR_JTAG_INIT_FAILED;
	}

#if BUILD_FT2232_FTD2XX == 1
	if (ftdi_device == FT_DEVICE_2232H) {
		/* initialize high byte of controller for jtag operation */
		if (ft2232_set_data_bits_high_byte(high_output, high_direction) != ERROR_OK) {
			LOG_ERROR("couldn't initialize Signalyzer-H layout");
			return ERROR_JTAG_INIT_FAILED;
		}
	}
#elif BUILD_FT2232_LIBFTDI == 1
	if (ftdi_device == TYPE_2232H) {
		/* initialize high byte of controller for jtag operation */
		if (ft2232_set_data_bits_high_byte(high_output, high_direction) != ERROR_OK) {
			LOG_ERROR("couldn't initialize Signalyzer-H layout");
			return ERROR_JTAG_INIT_FAILED;
		}
	}
#endif
	return ERROR_OK;
}

static void signalyzer_h_reset(int trst, int srst)
{
	enum reset_types jtag_reset_config = jtag_get_reset_config();

	/* ADAPTOR: EM_LT16_A */
	if (signalyzer_h_adapter_type == SIGNALYZER_MODULE_TYPE_EM_LT16_A) {
		if (trst == 1) {
			if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
				/* switch to output pin (output is low) */
				low_direction |= nTRSTnOE;
			else
				/* switch output low */
				low_output &= ~nTRST;
		} else if (trst == 0) {
			if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
				/* switch to input pin (high-Z + internal
				 * and external pullup) */
				low_direction &= ~nTRSTnOE;
			else
				/* switch output high */
				low_output |= nTRST;
		}

		if (srst == 1) {
			if (jtag_reset_config & RESET_SRST_PUSH_PULL)
				/* switch output low */
				low_output &= ~nSRST;
			else
				/* switch to output pin (output is low) */
				low_direction |= nSRSTnOE;
		} else if (srst == 0) {
			if (jtag_reset_config & RESET_SRST_PUSH_PULL)
				/* switch output high */
				low_output |= nSRST;
			else
				/* switch to input pin (high-Z) */
				low_direction &= ~nSRSTnOE;
		}

		/* command "set data bits low byte" */
		buffer_write(0x80);
		buffer_write(low_output);
		buffer_write(low_direction);
		LOG_DEBUG("trst: %i, srst: %i, low_output: 0x%2.2x, "
			"low_direction: 0x%2.2x",
			trst, srst, low_output, low_direction);
	}
	/* ADAPTOR: EM_ARM_JTAG,  EM_ARM_JTAG_P, EM_JTAG, EM_JTAG_P */
	else if ((signalyzer_h_adapter_type == SIGNALYZER_MODULE_TYPE_EM_ARM_JTAG) ||
		 (signalyzer_h_adapter_type == SIGNALYZER_MODULE_TYPE_EM_ARM_JTAG_P) ||
		 (signalyzer_h_adapter_type == SIGNALYZER_MODULE_TYPE_EM_JTAG)  ||
		 (signalyzer_h_adapter_type == SIGNALYZER_MODULE_TYPE_EM_JTAG_P)) {
		if (trst == 1) {
			if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
				high_output &= ~nTRSTnOE;
			else
				high_output &= ~nTRST;
		} else if (trst == 0) {
			if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
				high_output |= nTRSTnOE;
			else
				high_output |= nTRST;
		}

		if (srst == 1) {
			if (jtag_reset_config & RESET_SRST_PUSH_PULL)
				high_output &= ~nSRST;
			else
				high_output &= ~nSRSTnOE;
		} else if (srst == 0) {
			if (jtag_reset_config & RESET_SRST_PUSH_PULL)
				high_output |= nSRST;
			else
				high_output |= nSRSTnOE;
		}

		/* command "set data bits high byte" */
		buffer_write(0x82);
		buffer_write(high_output);
		buffer_write(high_direction);
		LOG_INFO("trst: %i, srst: %i, high_output: 0x%2.2x, "
			"high_direction: 0x%2.2x",
			trst, srst, high_output, high_direction);
	} else if (signalyzer_h_adapter_type == 0x0000) {
		if (trst == 1) {
			if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
				/* switch to output pin (output is low) */
				low_direction |= nTRSTnOE;
			else
				/* switch output low */
				low_output &= ~nTRST;
		} else if (trst == 0) {
			if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
				/* switch to input pin (high-Z + internal
				 * and external pullup) */
				low_direction &= ~nTRSTnOE;
			else
				/* switch output high */
				low_output |= nTRST;
		}

		if (srst == 1) {
			if (jtag_reset_config & RESET_SRST_PUSH_PULL)
				/* switch output low */
				low_output &= ~nSRST;
			else
				/* switch to output pin (output is low) */
				low_direction |= nSRSTnOE;
		} else if (srst == 0) {
			if (jtag_reset_config & RESET_SRST_PUSH_PULL)
				/* switch output high */
				low_output |= nSRST;
			else
				/* switch to input pin (high-Z) */
				low_direction &= ~nSRSTnOE;
		}

		/* command "set data bits low byte" */
		buffer_write(0x80);
		buffer_write(low_output);
		buffer_write(low_direction);
		LOG_DEBUG("trst: %i, srst: %i, low_output: 0x%2.2x, "
			"low_direction: 0x%2.2x",
			trst, srst, low_output, low_direction);
	}
}

static void signalyzer_h_blink(void)
{
	signalyzer_h_led_set(signalyzer_h_side, SIGNALYZER_LED_RED, 100, 0, 1);
}

/********************************************************************
 * Support for KT-LINK
 * JTAG adapter from KRISTECH
 * http://www.kristech.eu
 *******************************************************************/
static int ktlink_init(void)
{
	uint8_t swd_en = 0x20;	/* 0x20 SWD disable, 0x00 SWD enable (ADBUS5) */

	low_output    = 0x08 | swd_en;	/* value; TMS=1,TCK=0,TDI=0,SWD=swd_en */
	low_direction = 0x3B;		/* out=1; TCK/TDI/TMS=out,TDO=in,SWD=out,RTCK=in,SRSTIN=in */

	/* initialize low byte for jtag */
	if (ft2232_set_data_bits_low_byte(low_output, low_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'ktlink' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	nTRST    = 0x01;
	nSRST    = 0x02;
	nTRSTnOE = 0x04;
	nSRSTnOE = 0x08;

	high_output    = 0x80;	/* turn LED on */
	high_direction = 0xFF;	/* all outputs */

	enum reset_types jtag_reset_config = jtag_get_reset_config();

	if (jtag_reset_config & RESET_TRST_OPEN_DRAIN) {
		high_output |= nTRSTnOE;
		high_output &= ~nTRST;
	} else {
		high_output &= ~nTRSTnOE;
		high_output |= nTRST;
	}

	if (jtag_reset_config & RESET_SRST_PUSH_PULL) {
		high_output &= ~nSRSTnOE;
		high_output |= nSRST;
	} else {
		high_output |= nSRSTnOE;
		high_output &= ~nSRST;
	}

	/* initialize high byte for jtag */
	if (ft2232_set_data_bits_high_byte(high_output, high_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'ktlink' layout");
		return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static void ktlink_reset(int trst, int srst)
{
	enum reset_types jtag_reset_config = jtag_get_reset_config();

	if (trst == 1) {
		if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
			high_output &= ~nTRSTnOE;
		else
			high_output &= ~nTRST;
	} else if (trst == 0) {
		if (jtag_reset_config & RESET_TRST_OPEN_DRAIN)
			high_output |= nTRSTnOE;
		else
			high_output |= nTRST;
	}

	if (srst == 1) {
		if (jtag_reset_config & RESET_SRST_PUSH_PULL)
			high_output &= ~nSRST;
		else
			high_output &= ~nSRSTnOE;
	} else if (srst == 0) {
		if (jtag_reset_config & RESET_SRST_PUSH_PULL)
			high_output |= nSRST;
		else
			high_output |= nSRSTnOE;
	}

	buffer_write(0x82);	/* command "set data bits high byte" */
	buffer_write(high_output);
	buffer_write(high_direction);
	LOG_DEBUG("trst: %i, srst: %i, high_output: 0x%2.2x, high_direction: 0x%2.2x",
		trst,
		srst,
		high_output,
		high_direction);
}

static void ktlink_blink(void)
{
	/* LED connected to ACBUS7 */
	high_output ^= 0x80;

	buffer_write(0x82);	/* command "set data bits high byte" */
	buffer_write(high_output);
	buffer_write(high_direction);
}

/********************************************************************
 * Support for Digilent HS-1
 * JTAG adapter from Digilent
 * http://www.digilent.com
 * Author: Stephane Bonnet bonnetst@hds.utc.fr
 *******************************************************************/

static int digilent_hs1_init(void)
{
	/* the adapter only supports the base JTAG signals, no nTRST
	   nor nSRST */
	low_output	= 0x88;
	low_direction	= 0x8b;

	/* initialize low byte for jtag */
	if (ft2232_set_data_bits_low_byte(low_output, low_direction) != ERROR_OK) {
		LOG_ERROR("couldn't initialize FT2232 with 'digilent_hs1' layout");
		return ERROR_JTAG_INIT_FAILED;
	}
	return ERROR_OK;
}

static void digilent_hs1_reset(int trst, int srst)
{
	/* Dummy function, no reset signals supported. */
}

static const struct command_registration ft2232_command_handlers[] = {
	{
		.name = "ft2232_device_desc",
		.handler = &ft2232_handle_device_desc_command,
		.mode = COMMAND_CONFIG,
		.help = "set the USB device description of the FTDI FT2232 device",
		.usage = "description_string",
	},
	{
		.name = "ft2232_serial",
		.handler = &ft2232_handle_serial_command,
		.mode = COMMAND_CONFIG,
		.help = "set the serial number of the FTDI FT2232 device",
		.usage = "serial_string",
	},
	{
		.name = "ft2232_layout",
		.handler = &ft2232_handle_layout_command,
		.mode = COMMAND_CONFIG,
		.help = "set the layout of the FT2232 GPIO signals used "
			"to control output-enables and reset signals",
		.usage = "layout_name",
	},
	{
		.name = "ft2232_vid_pid",
		.handler = &ft2232_handle_vid_pid_command,
		.mode = COMMAND_CONFIG,
		.help = "the vendor ID and product ID of the FTDI FT2232 device",
		.usage = "(vid pid)* ",
	},
	{
		.name = "ft2232_latency",
		.handler = &ft2232_handle_latency_command,
		.mode = COMMAND_CONFIG,
		.help = "set the FT2232 latency timer to a new value",
		.usage = "value",
	},
	{
		.name = "ft2232_channel",
		.handler = &ft2232_handle_channel_command,
		.mode = COMMAND_CONFIG,
		.help = "set the FT2232 channel to a new value",
		.usage = "value",
	},
	COMMAND_REGISTRATION_DONE
};

struct jtag_interface ft2232_interface = {
	.name = "ft2232",
	.supported = DEBUG_CAP_TMS_SEQ,
	.commands = ft2232_command_handlers,
	.transports = jtag_only,

	.init = ft2232_init,
	.quit = ft2232_quit,
	.speed = ft2232_speed,
	.speed_div = ft2232_speed_div,
	.khz = ft2232_khz,
	.execute_queue = ft2232_execute_queue,
};
