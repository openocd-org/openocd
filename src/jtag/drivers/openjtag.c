/*******************************************************************************
 *   Driver for OpenJTAG Project (www.openjtag.org)                            *
 *   Compatible with libftdi and ftd2xx drivers.                               *
 *                                                                             *
 *   Copyright (C) 2010 by Ivan Meleca <mileca@gmail.com>                      *
 *                                                                             *
 *   Copyright (C) 2013 by Ryan Corbin, GlueLogix Inc. <corbin.ryan@gmail.com> *
 *   Updated to work with OpenOCD v0.7.0. Fixed libftdi read speed issue.      *
 *                                                                             *
 *   Based on usb_blaster.c                                                    *
 *   Copyright (C) 2009 Catalin Patulea                                        *
 *   Copyright (C) 2006 Kolja Waschk                                           *
 *                                                                             *
 *   And jlink.c                                                               *
 *   Copyright (C) 2008 by Spencer Oliver                                      *
 *   spen@spen-soft.co.uk                                                      *
 *                                                                             *
 *   This program is free software; you can redistribute it and/or modify      *
 *   it under the terms of the GNU General Public License as published by      *
 *   the Free Software Foundation; either version 2 of the License, or         *
 *   (at your option) any later version.                                       *
 *                                                                             *
 *   This program is distributed in the hope that it will be useful,           *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of            *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the             *
 *   GNU General Public License for more details.                              *
 *                                                                             *
 *   You should have received a copy of the GNU General Public License         *
 *   along with this program; if not, write to the                             *
 *   Free Software Foundation, Inc.,                                           *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.               *
 ***************************************************************************/

/***************************************************************************
 * Version 1.0  Tested on a MCBSTM32 board using a Cortex M3 (stm32f103x), *
 *              GDB and Eclipse under Linux (Ubuntu 10.04)                 *
 *                                                                         *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/interface.h>
#include <jtag/commands.h>
#include "usb_common.h"

/*
 * OpenJTAG-OpenOCD state conversion
 */
typedef enum openjtag_tap_state {
	OPENJTAG_TAP_INVALID    = -1,
	OPENJTAG_TAP_RESET  = 0,
	OPENJTAG_TAP_IDLE   = 1,
	OPENJTAG_TAP_SELECT_DR  = 2,
	OPENJTAG_TAP_CAPTURE_DR = 3,
	OPENJTAG_TAP_SHIFT_DR   = 4,
	OPENJTAG_TAP_EXIT1_DR   = 5,
	OPENJTAG_TAP_PAUSE_DR   = 6,
	OPENJTAG_TAP_EXIT2_DR   = 7,
	OPENJTAG_TAP_UPDATE_DR  = 8,
	OPENJTAG_TAP_SELECT_IR  = 9,
	OPENJTAG_TAP_CAPURE_IR  = 10,
	OPENJTAG_TAP_SHIFT_IR   = 11,
	OPENJTAG_TAP_EXIT1_IR   = 12,
	OPENJTAG_TAP_PAUSE_IR   = 13,
	OPENJTAG_TAP_EXIT2_IR   = 14,
	OPENJTAG_TAP_UPDATE_IR  = 15,
} openjtag_tap_state_t;

#if (BUILD_OPENJTAG_FTD2XX == 1 && BUILD_OPENJTAG_LIBFTDI == 1)
#error "BUILD_OPENJTAG_FTD2XX && BUILD_OPENJTAG_LIBFTDI "
	   "are mutually exclusive"
#elif (BUILD_OPENJTAG_FTD2XX != 1 && BUILD_OPENJTAG_LIBFTDI != 1)
#error "BUILD_OPENJTAG_FTD2XX || BUILD_OPENJTAG_LIBFTDI must be chosen"
#endif

/* OPENJTAG access library includes */
#if BUILD_OPENJTAG_FTD2XX == 1
#include <ftd2xx.h>
#elif BUILD_OPENJTAG_LIBFTDI == 1
#include <ftdi.h>
#endif

/* OpenJTAG vid/pid */
static uint16_t openjtag_vid = 0x0403;
static uint16_t openjtag_pid = 0x6001;

static char *openjtag_device_desc;

#if BUILD_OPENJTAG_FTD2XX == 1
static FT_HANDLE ftdih;

#elif BUILD_OPENJTAG_LIBFTDI == 1
static struct ftdi_context ftdic;
#endif

#define OPENJTAG_BUFFER_SIZE        504
#define OPENJTAG_MAX_PENDING_RESULTS    256

struct openjtag_scan_result {
	uint32_t bits;          /* Length in bits*/
	struct scan_command *command;   /* Corresponding scan command */
	uint8_t *buffer;
};

/* USB RX/TX buffers */
static int usb_tx_buf_offs;
static uint8_t usb_tx_buf[OPENJTAG_BUFFER_SIZE];
static uint32_t usb_rx_buf_len;
static uint8_t usb_rx_buf[OPENJTAG_BUFFER_SIZE];

/* Pending readings */
static struct openjtag_scan_result openjtag_scan_result_buffer[OPENJTAG_MAX_PENDING_RESULTS];
static int openjtag_scan_result_count;

/* Openocd usb handler */
struct openocd {
	struct usb_dev_handle *usb_handle;
};

#ifdef _DEBUG_USB_COMMS_

#define DEBUG_TYPE_READ     0
#define DEBUG_TYPE_WRITE    1
#define DEBUG_TYPE_OCD_READ 2
#define DEBUG_TYPE_BUFFER   3

#define LINE_LEN  16
static void openjtag_debug_buffer(uint8_t *buffer, int length, uint8_t type)
{
	char line[128];
	char s[4];
	int i;
	int j;

	switch (type) {
		case DEBUG_TYPE_READ:
			sprintf(line, "USB READ %d bytes", length);
			break;
		case DEBUG_TYPE_WRITE:
			sprintf(line, "USB WRITE %d bytes", length);
			break;
		case DEBUG_TYPE_OCD_READ:
			sprintf(line, "TO OpenOCD %d bytes", length);
			break;
		case DEBUG_TYPE_BUFFER:
			sprintf(line, "Buffer %d bytes", length);
			break;
	}

	LOG_DEBUG("%s", line);

	for (i = 0; i < length; i += LINE_LEN) {
		switch (type) {
			case DEBUG_TYPE_READ:
				sprintf(line, "USB READ: %04x", i);
				break;
			case DEBUG_TYPE_WRITE:
				sprintf(line, "USB WRITE: %04x", i);
				break;
			case DEBUG_TYPE_OCD_READ:
				sprintf(line, "TO OpenOCD: %04x", i);
				break;
			case DEBUG_TYPE_BUFFER:
				sprintf(line, "BUFFER: %04x", i);
				break;
		}

		for (j = i; j < i + LINE_LEN && j < length; j++) {
			sprintf(s, " %02x", buffer[j]);
			strcat(line, s);
		}
		LOG_DEBUG("%s", line);
	}

}

#endif

static int8_t openjtag_get_tap_state(int8_t state)
{

	switch (state) {
		case TAP_DREXIT2:   return OPENJTAG_TAP_EXIT2_DR;
		case TAP_DREXIT1:   return OPENJTAG_TAP_EXIT1_DR;
		case TAP_DRSHIFT:   return OPENJTAG_TAP_SHIFT_DR;
		case TAP_DRPAUSE:   return OPENJTAG_TAP_PAUSE_DR;
		case TAP_IRSELECT:  return OPENJTAG_TAP_SELECT_IR;
		case TAP_DRUPDATE:  return OPENJTAG_TAP_UPDATE_DR;
		case TAP_DRCAPTURE: return OPENJTAG_TAP_CAPTURE_DR;
		case TAP_DRSELECT:  return OPENJTAG_TAP_SELECT_DR;
		case TAP_IREXIT2:   return OPENJTAG_TAP_EXIT2_IR;
		case TAP_IREXIT1:   return OPENJTAG_TAP_EXIT1_IR;
		case TAP_IRSHIFT:   return OPENJTAG_TAP_SHIFT_IR;
		case TAP_IRPAUSE:   return OPENJTAG_TAP_PAUSE_IR;
		case TAP_IDLE:      return OPENJTAG_TAP_IDLE;
		case TAP_IRUPDATE:  return OPENJTAG_TAP_UPDATE_IR;
		case TAP_IRCAPTURE: return OPENJTAG_TAP_CAPURE_IR;
		case TAP_RESET:     return OPENJTAG_TAP_RESET;
		case TAP_INVALID:
		default:            return OPENJTAG_TAP_INVALID;
	}
}

static int openjtag_buf_write(
	uint8_t *buf, int size, uint32_t *bytes_written)
{
#if BUILD_OPENJTAG_FTD2XX == 1
	FT_STATUS status;
	DWORD dw_bytes_written;

#ifdef _DEBUG_USB_COMMS_
	openjtag_debug_buffer(buf, size, DEBUG_TYPE_WRITE);
#endif

	status = FT_Write(ftdih, buf, size, &dw_bytes_written);
	if (status != FT_OK) {
		*bytes_written = dw_bytes_written;
		LOG_ERROR("FT_Write returned: %u", status);
		return ERROR_JTAG_DEVICE_ERROR;
	}
	*bytes_written = dw_bytes_written;
	return ERROR_OK;
#elif BUILD_OPENJTAG_LIBFTDI == 1
	int retval;
#ifdef _DEBUG_USB_COMMS_
	openjtag_debug_buffer(buf, size, DEBUG_TYPE_WRITE);
#endif

	retval = ftdi_write_data(&ftdic, buf, size);
	if (retval < 0) {
		*bytes_written = 0;
		LOG_ERROR("ftdi_write_data: %s", ftdi_get_error_string(&ftdic));
		return ERROR_JTAG_DEVICE_ERROR;
	}

	*bytes_written += retval;

	return ERROR_OK;
#endif
}

static int openjtag_buf_read(uint8_t *buf, uint32_t qty, uint32_t *bytes_read)
{

#if BUILD_OPENJTAG_FTD2XX == 1
	DWORD dw_bytes_read;
	FT_STATUS status;
	int timeout = 50;

	*bytes_read = 0;
	while (qty && (*bytes_read < qty) && timeout--) {

		status = FT_Read(ftdih, buf + *bytes_read,
				qty - *bytes_read, &dw_bytes_read);
		if (status != FT_OK) {
			*bytes_read = dw_bytes_read;
			LOG_ERROR("FT_Read returned: %u", status);
			return ERROR_JTAG_DEVICE_ERROR;
		}
		*bytes_read += dw_bytes_read;
	}

#ifdef _DEBUG_USB_COMMS_
	openjtag_debug_buffer(buf, *bytes_read, DEBUG_TYPE_READ);
#endif

	return ERROR_OK;
#elif BUILD_OPENJTAG_LIBFTDI == 1
	int retval;
	int timeout = 5;

	*bytes_read = 0;

	while ((*bytes_read < qty) && timeout--) {
		retval = ftdi_read_data(&ftdic, buf + *bytes_read,
				qty - *bytes_read);
		if (retval < 0) {
			*bytes_read = 0;
			DEBUG_JTAG_IO("ftdi_read_data: %s",
					ftdi_get_error_string(&ftdic));
			return ERROR_JTAG_DEVICE_ERROR;
		}
		*bytes_read += retval;
	}

#ifdef _DEBUG_USB_COMMS_
	openjtag_debug_buffer(buf, *bytes_read, DEBUG_TYPE_READ);
#endif

#endif
	return ERROR_OK;
}

static int openjtag_sendcommand(uint8_t cmd)
{
	uint32_t written;
	return openjtag_buf_write(&cmd, 1, &written);
}

static int openjtag_speed(int speed)
{
	int clockcmd;
	switch (speed) {
		case 48000:
			clockcmd = 0x00;
			break;
		case 24000:
			clockcmd = 0x20;
			break;
		case 12000:
			clockcmd = 0x40;
			break;
		case 6000:
			clockcmd = 0x60;
			break;
		case 3000:
			clockcmd = 0x80;
			break;
		case 1500:
			clockcmd = 0xA0;
			break;
		case 750:
			clockcmd = 0xC0;
			break;
		case 375:
			clockcmd = 0xE0;
			break;
		default:
			clockcmd = 0xE0;
			LOG_WARNING("adapter speed not recognized, reverting to 375 kHz");
			break;
	}
	openjtag_sendcommand(clockcmd);

	return ERROR_OK;
}

static int openjtag_init(void)
{
	uint8_t latency_timer;

#if BUILD_OPENJTAG_FTD2XX == 1
	FT_STATUS status;
#endif

usb_tx_buf_offs = 0;
usb_rx_buf_len = 0;
openjtag_scan_result_count = 0;

#if BUILD_OPENJTAG_FTD2XX == 1
	LOG_DEBUG("'openjtag' interface using FTD2XX");
#elif BUILD_OPENJTAG_LIBFTDI == 1
	LOG_DEBUG("'openjtag' interface using libftdi");
#endif

/* Open by device description */
if (openjtag_device_desc == NULL) {
	LOG_WARNING("no openjtag device description specified, "
				"using default 'Open JTAG Project'");
	openjtag_device_desc = "Open JTAG Project";
}

#if BUILD_OPENJTAG_FTD2XX == 1

#if IS_WIN32 == 0
	/* Add non-standard Vid/Pid to the linux driver */
	status = FT_SetVIDPID(openjtag_vid, openjtag_pid);
	if (status != FT_OK) {
		LOG_WARNING("couldn't add %4.4x:%4.4x",
			openjtag_vid, openjtag_pid);
	}
#endif

	status = FT_OpenEx(openjtag_device_desc, FT_OPEN_BY_DESCRIPTION,
			&ftdih);
	if (status != FT_OK) {
		DWORD num_devices;

		LOG_ERROR("unable to open ftdi device: %u", status);
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
				LOG_ERROR("ListDevices: %u\n", num_devices);
				for (i = 0; i < num_devices; i++)
					LOG_ERROR("%i: %s", i, desc_array[i]);
			}

			for (i = 0; i < num_devices; i++)
				free(desc_array[i]);
			free(desc_array);
		} else {
			LOG_ERROR("ListDevices: NONE\n");
		}
		return ERROR_JTAG_INIT_FAILED;
	}

	status = FT_SetLatencyTimer(ftdih, 2);
	if (status != FT_OK) {
		LOG_ERROR("unable to set latency timer: %u", status);
		return ERROR_JTAG_INIT_FAILED;
	}

	status = FT_GetLatencyTimer(ftdih, &latency_timer);
	if (status != FT_OK) {
		LOG_ERROR("unable to get latency timer: %u", status);
		return ERROR_JTAG_INIT_FAILED;
	}
	LOG_DEBUG("current latency timer: %i", latency_timer);

	status = FT_SetBitMode(ftdih, 0x00, 0x40);
	if (status != FT_OK) {
		LOG_ERROR("unable to disable bit i/o mode: %u", status);
		return ERROR_JTAG_INIT_FAILED;
	}

	status = FT_SetTimeouts(ftdih, 50, 0);
	if (status != FT_OK) {
		LOG_ERROR("unable to set timeouts: %u", status);
		return ERROR_JTAG_INIT_FAILED;
	}

	status = FT_Purge(ftdih, FT_PURGE_RX | FT_PURGE_TX);
	if (status != FT_OK) {
		LOG_ERROR("unable to FT_Purge() %u", status);
		return ERROR_JTAG_INIT_FAILED;
	}

#elif BUILD_OPENJTAG_LIBFTDI == 1
	if (ftdi_init(&ftdic) < 0)
		return ERROR_JTAG_INIT_FAILED;

	/* context, vendor id, product id, description, serial id */
	if (ftdi_usb_open_desc(&ftdic, openjtag_vid, openjtag_pid, openjtag_device_desc, NULL) < 0) {
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
	/* was (3000000 / 4) with a comment about a bug in libftdi when using high baudrate */
	if (ftdi_set_baudrate(&ftdic, 3000000) < 0) {
		LOG_ERROR("Can't set baud rate to max: %s",
			ftdi_get_error_string(&ftdic));
		return ERROR_JTAG_DEVICE_ERROR;
	};
#endif

#if BUILD_OPENJTAG_FTD2XX == 1
	status = FT_Purge(ftdih, FT_PURGE_RX | FT_PURGE_TX);
	if (status != FT_OK)
		return ERROR_JTAG_INIT_FAILED;
#elif BUILD_OPENJTAG_LIBFTDI == 1
	if (ftdi_usb_purge_buffers(&ftdic) < 0) {
		LOG_ERROR("ftdi_purge_buffers: %s", ftdic.error_str);
		return ERROR_JTAG_INIT_FAILED;
	}
#endif

	/* OpenJTAG speed */
	openjtag_sendcommand(0xE0); /*Start at slowest adapter speed*/

	/* MSB */
	openjtag_sendcommand(0x75);

	return ERROR_OK;
}

static int openjtag_quit(void)
{
#if BUILD_OPENJTAG_FTD2XX == 1
	FT_Close(ftdih);
#elif BUILD_OPENJTAG_LIBFTDI == 1
	ftdi_usb_close(&ftdic);
	ftdi_deinit(&ftdic);
#endif

	return ERROR_OK;
}

static void openjtag_write_tap_buffer(void)
{
	uint32_t written;

	openjtag_buf_write(usb_tx_buf, usb_tx_buf_offs, &written);
	openjtag_buf_read(usb_rx_buf, usb_tx_buf_offs, &usb_rx_buf_len);

	usb_tx_buf_offs = 0;
}

static int openjtag_execute_tap_queue(void)
{
	openjtag_write_tap_buffer();

	int res_count = 0;

	if (openjtag_scan_result_count && usb_rx_buf_len) {

		int count;
		int rx_offs = 0;
		int len;

		/* for every pending result */
		while (res_count < openjtag_scan_result_count) {

			/* get sent bits */
			len = openjtag_scan_result_buffer[res_count].bits;

			count = 0;

			uint8_t *buffer = openjtag_scan_result_buffer[res_count].buffer;

			while (len) {
				if (len <= 8) {
					DEBUG_JTAG_IO("bits < 8 buf = 0x%X, will be 0x%X",
						usb_rx_buf[rx_offs], usb_rx_buf[rx_offs] >> (8 - len));
					buffer[count] = usb_rx_buf[rx_offs] >> (8 - len);
					len = 0;
				} else {
					buffer[count] = usb_rx_buf[rx_offs];
					len -= 8;
				}

				rx_offs++;
				count++;
			}

#ifdef _DEBUG_USB_COMMS_
			openjtag_debug_buffer(buffer,
				DIV_ROUND_UP(openjtag_scan_result_buffer[res_count].bits, 8), DEBUG_TYPE_OCD_READ);
#endif
			jtag_read_buffer(buffer, openjtag_scan_result_buffer[res_count].command);

			if (openjtag_scan_result_buffer[res_count].buffer)
				free(openjtag_scan_result_buffer[res_count].buffer);

			res_count++;
		}
	}

	openjtag_scan_result_count = 0;

	return ERROR_OK;
}

static void openjtag_add_byte(char buf)
{

	if (usb_tx_buf_offs == OPENJTAG_BUFFER_SIZE) {
		DEBUG_JTAG_IO("Forcing execute_tap_queue");
		DEBUG_JTAG_IO("TX Buff offs=%d", usb_tx_buf_offs);
		openjtag_execute_tap_queue();
	}

	usb_tx_buf[usb_tx_buf_offs] = buf;
	usb_tx_buf_offs++;
}

static void openjtag_add_scan(uint8_t *buffer, int length, struct scan_command *scan_cmd)
{

	/* Ensure space to send long chains */
	/* We add two byte for each eight (or less) bits, one for command, one for data */
	if (usb_tx_buf_offs + (DIV_ROUND_UP(length, 8) * 2) >= OPENJTAG_BUFFER_SIZE) {
		DEBUG_JTAG_IO("Forcing execute_tap_queue from scan");
		DEBUG_JTAG_IO("TX Buff offs=%d len=%d", usb_tx_buf_offs, DIV_ROUND_UP(length, 8) * 2);
		openjtag_execute_tap_queue();
	}

	openjtag_scan_result_buffer[openjtag_scan_result_count].bits = length;
	openjtag_scan_result_buffer[openjtag_scan_result_count].command = scan_cmd;
	openjtag_scan_result_buffer[openjtag_scan_result_count].buffer = buffer;

	uint8_t command;
	uint8_t bits;
	int count = 0;
	while (length) {

		/* write command */
		command = 6;

		/* last bits? */
		if (length <= 8) {
			/* tms high */
			command |= (1 << 4);

			/* bits to transfer */
			bits = (length - 1);
			command |= bits << 5;
			length = 0;
		} else {
			/* whole byte */

			/* bits to transfer */
			bits = 7;
			command |= (7 << 5);
			length -= 8;
		}

		openjtag_add_byte(command);
		openjtag_add_byte(buffer[count]);
		count++;
	}

	openjtag_scan_result_count++;
}

static void openjtag_execute_reset(struct jtag_command *cmd)
{

	DEBUG_JTAG_IO("reset trst: %i srst %i",
			cmd->cmd.reset->trst, cmd->cmd.reset->srst);

	uint8_t buf = 0x00;

	if (cmd->cmd.reset->trst) {
		buf = 0x03;
	} else {
		buf |= 0x04;
		buf |= 0x05 << 4;
	}

	openjtag_add_byte(buf);
}

static void openjtag_execute_sleep(struct jtag_command *cmd)
{
	jtag_sleep(cmd->cmd.sleep->us);
}

static void openjtag_set_state(uint8_t openocd_state)
{
	int8_t state = openjtag_get_tap_state(openocd_state);

	uint8_t buf = 0;
	buf = 0x01;
	buf |= state << 4;

	openjtag_add_byte(buf);
}

static void openjtag_execute_statemove(struct jtag_command *cmd)
{
	DEBUG_JTAG_IO("state move to %i", cmd->cmd.statemove->end_state);

	tap_set_end_state(cmd->cmd.statemove->end_state);

	openjtag_set_state(cmd->cmd.statemove->end_state);

	tap_set_state(tap_get_end_state());
}


static void openjtag_execute_scan(struct jtag_command *cmd)
{

	int scan_size, old_state;
	uint8_t *buffer;

	DEBUG_JTAG_IO("scan ends in %s", tap_state_name(cmd->cmd.scan->end_state));

	/* get scan info */
	tap_set_end_state(cmd->cmd.scan->end_state);
	scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);

#ifdef _DEBUG_USB_COMMS_
	openjtag_debug_buffer(buffer, (scan_size + 7) / 8, DEBUG_TYPE_BUFFER);
#endif
	/* set state */
	old_state = tap_get_end_state();
	openjtag_set_state(cmd->cmd.scan->ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT);
	tap_set_state(cmd->cmd.scan->ir_scan ? TAP_IRSHIFT : TAP_DRSHIFT);
	tap_set_end_state(old_state);

	openjtag_add_scan(buffer, scan_size, cmd->cmd.scan);

	openjtag_set_state(cmd->cmd.scan->ir_scan ? TAP_IRPAUSE : TAP_DRPAUSE);
	tap_set_state(cmd->cmd.scan->ir_scan ? TAP_IRPAUSE : TAP_DRPAUSE);

	if (tap_get_state() != tap_get_end_state()) {
		openjtag_set_state(tap_get_end_state());
		tap_set_state(tap_get_end_state());
	}
}

static void openjtag_execute_runtest(struct jtag_command *cmd)
{

	tap_state_t end_state = cmd->cmd.runtest->end_state;
	tap_set_end_state(end_state);

	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE) {
		openjtag_set_state(TAP_IDLE);
		tap_set_state(TAP_IDLE);
	}

	if (cmd->cmd.runtest->num_cycles > 16)
		LOG_WARNING("num_cycles > 16 on run test");

	uint8_t command;
	command = 7;
	command |= ((cmd->cmd.runtest->num_cycles - 1) & 0x0F) << 4;

	openjtag_add_byte(command);

	tap_set_end_state(end_state);
	if (tap_get_end_state() != tap_get_state()) {
		openjtag_set_state(end_state);
		tap_set_state(end_state);
	}
}

static void openjtag_execute_command(struct jtag_command *cmd)
{
	DEBUG_JTAG_IO("openjtag_execute_command %i", cmd->type);
	switch (cmd->type) {
	case JTAG_RESET:
			openjtag_execute_reset(cmd);
			break;
	case JTAG_SLEEP:
			openjtag_execute_sleep(cmd);
			break;
	case JTAG_TLR_RESET:
			openjtag_execute_statemove(cmd);
			break;
	case JTAG_SCAN:
			openjtag_execute_scan(cmd);
			break;
	case JTAG_RUNTEST:
			openjtag_execute_runtest(cmd);
			break;
	case JTAG_PATHMOVE:
		/* jlink_execute_pathmove(cmd); break; */
	default:
		LOG_ERROR("BUG: unknown Open JTAG command type encountered");
		exit(-1);
	}
}

static int openjtag_execute_queue(void)
{
	struct jtag_command *cmd = jtag_command_queue;

	while (cmd != NULL) {
		openjtag_execute_command(cmd);
		cmd = cmd->next;
	}

	return openjtag_execute_tap_queue();
}

static int openjtag_speed_div(int speed, int *khz)
{
	*khz = speed;

	return ERROR_OK;
}

static int openjtag_khz(int khz, int *jtag_speed)
{

	if (khz >= 48000)
		*jtag_speed = 48000;
	else if (khz >= 24000)
		*jtag_speed = 24000;
	else if (khz >= 12000)
		*jtag_speed = 12000;
	else if (khz >= 6000)
		*jtag_speed = 6000;
	else if (khz >= 3000)
		*jtag_speed = 3000;
	else if (khz >= 1500)
		*jtag_speed = 1500;
	else if (khz >= 750)
		*jtag_speed = 750;
	else
		*jtag_speed = 375;

	return ERROR_OK;
}

COMMAND_HANDLER(openjtag_handle_device_desc_command)
{
	if (CMD_ARGC == 1)
		openjtag_device_desc = strdup(CMD_ARGV[0]);
	else
		LOG_ERROR("require exactly one argument to "
				  "openjtag_device_desc <description>");
	return ERROR_OK;
}


static const struct command_registration openjtag_command_handlers[] = {
	{
		.name = "openjtag_device_desc",
		.handler = openjtag_handle_device_desc_command,
		.mode = COMMAND_CONFIG,
		.help = "set the USB device description of the OpenJTAG",
		.usage = "description-string",
	},
	COMMAND_REGISTRATION_DONE
};

struct jtag_interface openjtag_interface = {
	.name = "openjtag",
	.commands = openjtag_command_handlers,

	.execute_queue = openjtag_execute_queue,
	.speed = openjtag_speed,
	.speed_div = openjtag_speed_div,
	.khz = openjtag_khz,
	.init = openjtag_init,
	.quit = openjtag_quit,
};


