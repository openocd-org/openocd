/***************************************************************************
 *   Driver for USB-JTAG, Altera USB-Blaster and compatibles               *
 *   Original code from Kolja Waschk's USB-JTAG project                    *
 *     (http://www.ixo.de/info/usb_jtag/).                                 *
 *   Some updates by Anthony Liu (2006).                                   *
 *   Minor updates and cleanup by Catalin Patulea (2009).                  *
 *   Speed updates by Ali Lown (2011).                                     *
 *                                                                         *
 *   Copyright (C) 2011 Ali Lown                                           *
 *   ali@lown.me.uk                                                        *
 *                                                                         *
 *   Copyright (C) 2009 Catalin Patulea                                    *
 *   cat@vv.carleton.ca                                                    *
 *                                                                         *
 *   Copyright (C) 2006 Kolja Waschk                                       *
 *   usbjtag@ixo.de                                                        *
 *                                                                         *
 *   Based on ft2232.c and bitbang.c,                                      *
 *   Copyright (C) 2004,2006 by Dominic Rath                               *
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

/*
 * The following information is originally from Kolja Waschk's USB-JTAG,
 * where it was obtained by reverse engineering an Altera USB-Blaster.
 * See http://www.ixo.de/info/usb_jtag/ for USB-Blaster block diagram and
 * usb_jtag-20080705-1200.zip#usb_jtag/host/openocd for protocol.
 *
 * The same information is also on the UrJTAG mediawiki, with some additional
 * notes on bits marked as "unknown" by usb_jtag.
 * (http://sourceforge.net/apps/mediawiki/urjtag/index.php?
 *    title=Cable_Altera_USB-Blaster)
 *
 * USB-JTAG, Altera USB-Blaster and compatibles are typically implemented as
 * an FTDIChip FT245 followed by a CPLD which handles a two-mode protocol:
 *
 *            _________
 *           |         |
 *           | AT93C46 |
 *           |_________|
 *            __|__________    _________
 *           |             |  |         |
 *      USB__| FTDI 245BM  |__| EPM7064 |__JTAG (B_TDO,B_TDI,B_TMS,B_TCK)
 *           |_____________|  |_________|
 *            __|__________    _|___________
 *           |             |  |             |
 *           | 6 MHz XTAL  |  | 24 MHz Osc. |
 *           |_____________|  |_____________|
 *
 * Protocol details are given in the code below.
 *
 * It is also possible to emulate this configuration using a single-chip USB
 * controller like the Cypress FX2 (again, see usb_jtag for details).
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#if IS_CYGWIN == 1
#include "windows.h"
#undef LOG_ERROR
#endif

/* project specific includes */
#include <jtag/interface.h>
#include <jtag/commands.h>
#include <helper/time_support.h>

/* system includes */
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

#include "bitbang.h"

#if (BUILD_USB_BLASTER_FTD2XX == 1 && BUILD_USB_BLASTER_LIBFTDI == 1)
#error "BUILD_USB_BLASTER_FTD2XX && BUILD_USB_BLASTER_LIBFTDI "
"are mutually exclusive"
#elif (BUILD_USB_BLASTER_FTD2XX != 1 && BUILD_USB_BLASTER_LIBFTDI != 1)
#error "BUILD_USB_BLASTER_FTD2XX || BUILD_USB_BLASTER_LIBFTDI must be chosen"
#endif

/* USB_BLASTER access library includes */
#if BUILD_USB_BLASTER_FTD2XX == 1
#include <ftd2xx.h>
#include "ftd2xx_common.h"
#elif BUILD_USB_BLASTER_LIBFTDI == 1
#include <ftdi.h>
#endif

#include <sys/time.h>
#include <time.h>

static char *usb_blaster_device_desc;
static uint16_t usb_blaster_vid = 0x09fb;	/* Altera */
static uint16_t usb_blaster_pid = 0x6001;	/* USB-Blaster */

/* last output byte in simple bit banging (legacy) mode */
static uint8_t out_value;
/* global output buffer for bit banging */
#define BUF_LEN 64	/* Size of EP1 */
static uint8_t out_buffer[BUF_LEN];
static uint16_t out_count;

#if BUILD_USB_BLASTER_FTD2XX == 1
static FT_HANDLE ftdih;
#elif BUILD_USB_BLASTER_LIBFTDI == 1
static struct ftdi_context ftdic;
#endif

static int usb_blaster_buf_write(
	uint8_t *buf, int size, uint32_t *bytes_written)
{
#if BUILD_USB_BLASTER_FTD2XX == 1
	FT_STATUS status;
	DWORD dw_bytes_written;

#ifdef _DEBUG_JTAG_IO_
	LOG_DEBUG("usb_blaster_buf_write %02X (%d)", buf[0], size);
#endif
	status = FT_Write(ftdih, buf, size, &dw_bytes_written);
	if (status != FT_OK) {
		*bytes_written = dw_bytes_written;
		LOG_ERROR("FT_Write returned: %s", ftd2xx_status_string(status));
		return ERROR_JTAG_DEVICE_ERROR;
	}
	*bytes_written = dw_bytes_written;
	return ERROR_OK;
#elif BUILD_USB_BLASTER_LIBFTDI == 1
	int retval;
#ifdef _DEBUG_JTAG_IO_
	LOG_DEBUG("usb_blaster_buf_write %02X (%d)", buf[0], size);
#endif
	retval = ftdi_write_data(&ftdic, buf, size);
	if (retval < 0) {
		*bytes_written = 0;
		LOG_ERROR("ftdi_write_data: %s", ftdi_get_error_string(&ftdic));
		return ERROR_JTAG_DEVICE_ERROR;
	}
	*bytes_written = retval;
	return ERROR_OK;
#endif
}

static int usb_blaster_buf_read(uint8_t *buf, unsigned size, uint32_t *bytes_read)
{
#if BUILD_USB_BLASTER_FTD2XX == 1
	DWORD dw_bytes_read;
	FT_STATUS status;

	status = FT_Read(ftdih, buf, size, &dw_bytes_read);
	if (status != FT_OK) {
		*bytes_read = dw_bytes_read;
		LOG_ERROR("FT_Read returned: %s", ftd2xx_status_string(status));
		return ERROR_JTAG_DEVICE_ERROR;
	}
#ifdef _DEBUG_JTAG_IO_
	LOG_DEBUG("usb_blaster_buf_read %02X (%" PRIu32 ")", buf[0], dw_bytes_read);
#endif
	*bytes_read = dw_bytes_read;
	return ERROR_OK;

#elif BUILD_USB_BLASTER_LIBFTDI == 1
	int retval;
	int timeout = 100;

	*bytes_read = 0;
	while ((*bytes_read < size) && timeout--) {
		retval = ftdi_read_data(&ftdic, buf + *bytes_read,
				size - *bytes_read);
		if (retval < 0) {
			*bytes_read = 0;
			LOG_ERROR("ftdi_read_data: %s",
				ftdi_get_error_string(&ftdic));
			return ERROR_JTAG_DEVICE_ERROR;
		}
		*bytes_read += retval;
	}
#ifdef _DEBUG_JTAG_IO_
	LOG_DEBUG("usb_blaster_buf_read %02X (%d)", buf[0], *bytes_read);
#endif
	return ERROR_OK;
#endif
}

/* The following code doesn't fully utilize the possibilities of the
 * USB-Blaster. It only buffers data up to the maximum packet size of 64 bytes.
 *
 * Actually, the USB-Blaster offers a byte-shift mode to transmit up to 504 data
 * bits (bidirectional) in a single USB packet. A header byte has to be sent as
 * the first byte in a packet with the following meaning:
 *
 *   Bit 7 (0x80): Must be set to indicate byte-shift mode.
 *   Bit 6 (0x40): If set, the USB-Blaster will also read data, not just write.
 *   Bit 5..0:     Define the number N of following bytes
 *
 * All N following bytes will then be clocked out serially on TDI. If Bit 6 was
 * set, it will afterwards return N bytes with TDO data read while clocking out
 * the TDI data. LSB of the first byte after the header byte will appear first
 * on TDI.
 */

/* Simple bit banging mode:
 *
 *   Bit 7 (0x80): Must be zero (see byte-shift mode above)
 *   Bit 6 (0x40): If set, you will receive a byte indicating the state of TDO
 *                 in return.
 *   Bit 5 (0x20): Output Enable/LED.
 *   Bit 4 (0x10): TDI Output.
 *   Bit 3 (0x08): nCS Output (not used in JTAG mode).
 *   Bit 2 (0x04): nCE Output (not used in JTAG mode).
 *   Bit 1 (0x02): TMS Output.
 *   Bit 0 (0x01): TCK Output.
 *
 * For transmitting a single data bit, you need to write two bytes. Up to 64
 * bytes can be combined in a single USB packet.
 * It isn't possible to read a data without transmitting data.
 */

#define TCK			(1 << 0)
#define TMS			(1 << 1)
#define NCE			(1 << 2)
#define NCS			(1 << 3)
#define TDI			(1 << 4)
#define LED			(1 << 5)
#define READ		(1 << 6)
#define SHMODE		(1 << 7)
#define OTHERS		((1 << 2) | (1 << 3) | (1 << 5))

#define READ_TDO	(1 << 0)

static void usb_blaster_write_databuffer(uint8_t *buf, uint16_t len)
{
	uint32_t bytes_written;
	usb_blaster_buf_write(buf, len, &bytes_written);
	out_count = 0;
#ifdef _DEBUG_JTAG_IO_
	LOG_DEBUG("---- WROTE %d", bytes_written);
#endif
}

static void usb_blaster_addtowritebuffer(uint8_t value, bool forcewrite)
{
	out_buffer[out_count] = value;
	out_count += 1;
	if (out_count == BUF_LEN || forcewrite)
		usb_blaster_write_databuffer(out_buffer, out_count);
}

static int usb_blaster_read_data(void)
{
	int status;
	uint8_t buf[1];
	uint32_t bytes_read;

	if (out_count > 0)
		usb_blaster_write_databuffer(out_buffer, out_count);

	out_value |= READ;
	usb_blaster_addtowritebuffer(out_value, true);
	out_value &= ~READ;

	status = usb_blaster_buf_read(buf, 1, &bytes_read);
	if (status < 0)
		return 0;

	return !!(buf[0] & READ_TDO);
}

static void usb_blaster_write(int tck, int tms, int tdi)
{
#ifdef _DEBUG_JTAG_IO_
	LOG_DEBUG("---- usb_blaster_write(%d,%d,%d)", tck, tms, tdi);
#endif
	out_value &= ~(TCK | TMS | TDI);
	if (tck)
		out_value |= TCK;
	if (tms)
		out_value |= TMS;
	if (tdi)
		out_value |= TDI;

	usb_blaster_addtowritebuffer(out_value, false);
}

static int usb_blaster_speed(int speed)
{
#if BUILD_USB_BLASTER_FTD2XX == 1
	LOG_DEBUG("TODO: usb_blaster_speed() isn't implemented for libftd2xx!");
#elif BUILD_USB_BLASTER_LIBFTDI == 1
	LOG_DEBUG("TODO: usb_blaster_speed() isn't optimally implemented!");

	/* TODO: libftdi's ftdi_set_baudrate chokes on high rates, use lowlevel
	 * usb function instead! And additionally allow user to throttle.
	 */
	if (ftdi_set_baudrate(&ftdic, 3000000 / 4) < 0) {
		LOG_ERROR("Can't set baud rate to max: %s",
			ftdi_get_error_string(&ftdic));
		return ERROR_JTAG_DEVICE_ERROR;
	}
	;
#endif

	return ERROR_OK;
}

static void usb_blaster_reset(int trst, int srst)
{
	LOG_DEBUG("TODO: usb_blaster_reset(%d,%d) isn't implemented!",
		trst, srst);
}

static void usb_blaster_blink(int state)
{
	out_value = 0x00;
	if (state)
		out_value |= LED;

	usb_blaster_addtowritebuffer(out_value, true);
}

static struct bitbang_interface usb_blaster_bitbang = {
	.read = usb_blaster_read_data,
	.write = usb_blaster_write,
	.reset = usb_blaster_reset,
	.blink = usb_blaster_blink,
};

static int usb_blaster_init(void)
{
	uint8_t latency_timer;

#if BUILD_USB_BLASTER_FTD2XX == 1
	FT_STATUS status;
#endif

#if BUILD_USB_BLASTER_FTD2XX == 1
	LOG_DEBUG("'usb_blaster' interface using FTD2XX");
#elif BUILD_USB_BLASTER_LIBFTDI == 1
	LOG_DEBUG("'usb_blaster' interface using libftdi");
#endif

#if BUILD_USB_BLASTER_FTD2XX == 1
	/* Open by device description */
	if (usb_blaster_device_desc == NULL) {
		LOG_WARNING("no usb_blaster device description specified, "
			"using default 'USB-Blaster'");
		usb_blaster_device_desc = "USB-Blaster";
	}

#if IS_WIN32 == 0
	/* Add non-standard Vid/Pid to the linux driver */
	status = FT_SetVIDPID(usb_blaster_vid, usb_blaster_pid);
	if (status != FT_OK) {
		LOG_WARNING("couldn't add %4.4x:%4.4x",
			usb_blaster_vid, usb_blaster_pid);
	}
#endif

	status = FT_OpenEx(usb_blaster_device_desc, FT_OPEN_BY_DESCRIPTION,
			&ftdih);
	if (status != FT_OK) {
		DWORD num_devices;

		LOG_ERROR("unable to open ftdi device: %s",
			ftd2xx_status_string(status));
		status = FT_ListDevices(&num_devices, NULL,
				FT_LIST_NUMBER_ONLY);
		if (status == FT_OK) {
			char **desc_array = malloc(sizeof(char *)
					* (num_devices + 1));
			unsigned int i;

			for (i = 0; i < num_devices; i++)
				desc_array[i] = malloc(64);
			desc_array[num_devices] = NULL;

			status = FT_ListDevices(desc_array, &num_devices,
					FT_LIST_ALL | FT_OPEN_BY_DESCRIPTION);

			if (status == FT_OK) {
				LOG_ERROR("ListDevices: %" PRIu32, (uint32_t)num_devices);
				for (i = 0; i < num_devices; i++)
					LOG_ERROR("%i: %s", i, desc_array[i]);
			}

			for (i = 0; i < num_devices; i++)
				free(desc_array[i]);
			free(desc_array);
		} else
			printf("ListDevices: NONE\n");
		return ERROR_JTAG_INIT_FAILED;
	}

	status = FT_SetLatencyTimer(ftdih, 2);
	if (status != FT_OK) {
		LOG_ERROR("unable to set latency timer: %s",
			ftd2xx_status_string(status));
		return ERROR_JTAG_INIT_FAILED;
	}

	status = FT_GetLatencyTimer(ftdih, &latency_timer);
	if (status != FT_OK) {
		LOG_ERROR("unable to get latency timer: %s",
			ftd2xx_status_string(status));
		return ERROR_JTAG_INIT_FAILED;
	}
	LOG_DEBUG("current latency timer: %i", latency_timer);

	status = FT_SetBitMode(ftdih, 0x00, 0);
	if (status != FT_OK) {
		LOG_ERROR("unable to disable bit i/o mode: %s",
			ftd2xx_status_string(status));
		return ERROR_JTAG_INIT_FAILED;
	}
#elif BUILD_USB_BLASTER_LIBFTDI == 1
	if (ftdi_init(&ftdic) < 0)
		return ERROR_JTAG_INIT_FAILED;

	/* context, vendor id, product id */
	if (ftdi_usb_open(&ftdic, usb_blaster_vid, usb_blaster_pid) < 0) {
		LOG_ERROR("unable to open ftdi device: %s", ftdic.error_str);
		return ERROR_JTAG_INIT_FAILED;
	}

	if (ftdi_usb_reset(&ftdic) < 0) {
		LOG_ERROR("unable to reset ftdi device");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (ftdi_set_latency_timer(&ftdic, 2) < 0) {
		LOG_ERROR("unable to set latency timer");
		return ERROR_JTAG_INIT_FAILED;
	}

	if (ftdi_get_latency_timer(&ftdic, &latency_timer) < 0) {
		LOG_ERROR("unable to get latency timer");
		return ERROR_JTAG_INIT_FAILED;
	}
	LOG_DEBUG("current latency timer: %u", latency_timer);

	ftdi_disable_bitbang(&ftdic);
#endif

	bitbang_interface = &usb_blaster_bitbang;

#if 0
#if BUILD_USB_BLASTER_FTD2XX == 1
	status = FT_Purge(ftdih, FT_PURGE_RX | FT_PURGE_TX);
	if (status != FT_OK) {
		LOG_ERROR("error purging ftd2xx device: %i", status);
		return ERROR_JTAG_INIT_FAILED;
	}
#elif BUILD_USB_BLASTER_LIBFTDI == 1
	if (ftdi_usb_purge_buffers(&ftdic) < 0) {
		LOG_ERROR("ftdi_purge_buffers: %s", ftdic.error_str);
		return ERROR_JTAG_INIT_FAILED;
	}
#endif
#endif

	return ERROR_OK;
}

static int usb_blaster_quit(void)
{
	if (out_count > 0)
		usb_blaster_write_databuffer(out_buffer, out_count);

#if BUILD_USB_BLASTER_FTD2XX == 1
	FT_STATUS status;

	status = FT_Close(ftdih);
#elif BUILD_USB_BLASTER_LIBFTDI == 1
	ftdi_usb_close(&ftdic);
	ftdi_deinit(&ftdic);
#endif

	return ERROR_OK;
}

COMMAND_HANDLER(usb_blaster_handle_device_desc_command)
{
	if (CMD_ARGC == 1)
		usb_blaster_device_desc = strdup(CMD_ARGV[0]);
	else
		LOG_ERROR("require exactly one argument to "
			"usb_blaster_device_desc <description>");

	return ERROR_OK;
}

COMMAND_HANDLER(usb_blaster_handle_vid_pid_command)
{
	if (CMD_ARGC > 2) {
		LOG_WARNING("ignoring extra IDs in usb_blaster_vid_pid "
			"(maximum is 1 pair)");
		CMD_ARGC = 2;
	}
	if (CMD_ARGC == 2) {
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], usb_blaster_vid);
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[1], usb_blaster_pid);
	} else
		LOG_WARNING("incomplete usb_blaster_vid_pid configuration");

	return ERROR_OK;
}

COMMAND_HANDLER(usb_blaster_handle_pin_command)
{
	if (CMD_ARGC == 2) {
		const char *const pin_name = CMD_ARGV[0];
		uint8_t mask;
		unsigned int state;

		if (!strcmp(pin_name, "pin6"))
			mask = NCE;
		else if (!strcmp(pin_name, "pin8"))
			mask = NCS;
		else {
			LOG_ERROR("%s: pin name must be \"pin6\" or \"pin8\"",
				CMD_NAME);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[1], state);
		if (state == 0) {
			out_value &= ~mask;
			usb_blaster_addtowritebuffer(out_value, true);
		} else if (state == 1)   {
			out_value |= mask;
			usb_blaster_addtowritebuffer(out_value, true);
		} else {
			LOG_ERROR("%s: pin state must be 0 or 1", CMD_NAME);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}

		return ERROR_OK;
	} else {
		LOG_ERROR("%s takes exactly two arguments", CMD_NAME);
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
}

static const struct command_registration usb_blaster_command_handlers[] = {
	{
		.name = "usb_blaster_device_desc",
		.handler = usb_blaster_handle_device_desc_command,
		.mode = COMMAND_CONFIG,
		.help = "set the USB device description of the USB-Blaster",
		.usage = "description-string",
	},
	{
		.name = "usb_blaster_vid_pid",
		.handler = usb_blaster_handle_vid_pid_command,
		.mode = COMMAND_CONFIG,
		.help = "the vendor ID and product ID of the USB-Blaster",
		.usage = "vid pid",
	},
	{
		.name = "usb_blaster",
		.handler = usb_blaster_handle_pin_command,
		.mode = COMMAND_ANY,
		.help = "set pin state for the unused GPIO pins",
		.usage = "(pin6|pin8) (0|1)",
	},
	COMMAND_REGISTRATION_DONE
};

struct jtag_interface usb_blaster_interface = {
	.name = "usb_blaster",
	.commands = usb_blaster_command_handlers,
	.supported = DEBUG_CAP_TMS_SEQ,

	.execute_queue = bitbang_execute_queue,

	.speed = usb_blaster_speed,
	.init = usb_blaster_init,
	.quit = usb_blaster_quit,
};
