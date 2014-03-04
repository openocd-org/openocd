/***************************************************************************
 *   Copyright (C) 2011-2012 by Mathias Kuester                            *
 *   Mathias Kuester <kesmtp@freenet.de>                                   *
 *                                                                         *
 *   Copyright (C) 2012 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   This code is based on https://github.com/texane/stlink                *
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

/* project specific includes */
#include <helper/binarybuffer.h>
#include <jtag/interface.h>
#include <jtag/hla/hla_layout.h>
#include <jtag/hla/hla_transport.h>
#include <jtag/hla/hla_interface.h>
#include <target/target.h>

#include <target/cortex_m.h>

#include "libusb_common.h"

#define ENDPOINT_IN  0x80
#define ENDPOINT_OUT 0x00

#define STLINK_WRITE_TIMEOUT 1000
#define STLINK_READ_TIMEOUT 1000

#define STLINK_NULL_EP        0
#define STLINK_RX_EP          (1|ENDPOINT_IN)
#define STLINK_TX_EP          (2|ENDPOINT_OUT)
#define STLINK_TRACE_EP       (3|ENDPOINT_IN)

#define STLINK_V2_1_TX_EP     (1|ENDPOINT_OUT)
#define STLINK_V2_1_TRACE_EP  (2|ENDPOINT_IN)

#define STLINK_SG_SIZE        (31)
#define STLINK_DATA_SIZE      (4096)
#define STLINK_CMD_SIZE_V2    (16)
#define STLINK_CMD_SIZE_V1    (10)

#define STLINK_V1_PID         (0x3744)
#define STLINK_V2_PID         (0x3748)
#define STLINK_V2_1_PID       (0x374B)

/* the current implementation of the stlink limits
 * 8bit read/writes to max 64 bytes. */
#define STLINK_MAX_RW8		(64)

enum stlink_jtag_api_version {
	STLINK_JTAG_API_V1 = 1,
	STLINK_JTAG_API_V2,
};

/** */
struct stlink_usb_version {
	/** */
	int stlink;
	/** */
	int jtag;
	/** */
	int swim;
	/** highest supported jtag api version */
	enum stlink_jtag_api_version jtag_api_max;
};

/** */
struct stlink_usb_handle_s {
	/** */
	struct jtag_libusb_device_handle *fd;
	/** */
	struct libusb_transfer *trans;
	/** */
	uint8_t rx_ep;
	/** */
	uint8_t tx_ep;
	/** */
	uint8_t trace_ep;
	/** */
	uint8_t cmdbuf[STLINK_SG_SIZE];
	/** */
	uint8_t cmdidx;
	/** */
	uint8_t direction;
	/** */
	uint8_t databuf[STLINK_DATA_SIZE];
	/** */
	uint32_t max_mem_packet;
	/** */
	enum hl_transports transport;
	/** */
	struct stlink_usb_version version;
	/** */
	uint16_t vid;
	/** */
	uint16_t pid;
	/** this is the currently used jtag api */
	enum stlink_jtag_api_version jtag_api;
	/** */
	struct {
		/** whether SWO tracing is enabled or not */
		bool enabled;
		/** trace data destination file */
		FILE *output_f;
		/** trace module source clock (for prescaler) */
		uint32_t source_hz;
		/** trace module clock prescaler */
		uint32_t prescale;
	} trace;
};

#define STLINK_DEBUG_ERR_OK            0x80
#define STLINK_DEBUG_ERR_FAULT         0x81
#define STLINK_SWD_AP_WAIT             0x10
#define STLINK_SWD_DP_WAIT             0x14

#define STLINK_CORE_RUNNING            0x80
#define STLINK_CORE_HALTED             0x81
#define STLINK_CORE_STAT_UNKNOWN       -1

#define STLINK_GET_VERSION             0xF1
#define STLINK_DEBUG_COMMAND           0xF2
#define STLINK_DFU_COMMAND             0xF3
#define STLINK_SWIM_COMMAND            0xF4
#define STLINK_GET_CURRENT_MODE        0xF5
#define STLINK_GET_TARGET_VOLTAGE      0xF7

#define STLINK_DEV_DFU_MODE            0x00
#define STLINK_DEV_MASS_MODE           0x01
#define STLINK_DEV_DEBUG_MODE          0x02
#define STLINK_DEV_SWIM_MODE           0x03
#define STLINK_DEV_BOOTLOADER_MODE     0x04
#define STLINK_DEV_UNKNOWN_MODE        -1

#define STLINK_DFU_EXIT                0x07

#define STLINK_SWIM_ENTER              0x00
#define STLINK_SWIM_EXIT               0x01

#define STLINK_DEBUG_ENTER_JTAG            0x00
#define STLINK_DEBUG_GETSTATUS             0x01
#define STLINK_DEBUG_FORCEDEBUG            0x02
#define STLINK_DEBUG_APIV1_RESETSYS        0x03
#define STLINK_DEBUG_APIV1_READALLREGS     0x04
#define STLINK_DEBUG_APIV1_READREG         0x05
#define STLINK_DEBUG_APIV1_WRITEREG        0x06
#define STLINK_DEBUG_READMEM_32BIT         0x07
#define STLINK_DEBUG_WRITEMEM_32BIT        0x08
#define STLINK_DEBUG_RUNCORE               0x09
#define STLINK_DEBUG_STEPCORE              0x0a
#define STLINK_DEBUG_APIV1_SETFP           0x0b
#define STLINK_DEBUG_READMEM_8BIT          0x0c
#define STLINK_DEBUG_WRITEMEM_8BIT         0x0d
#define STLINK_DEBUG_APIV1_CLEARFP         0x0e
#define STLINK_DEBUG_APIV1_WRITEDEBUGREG   0x0f
#define STLINK_DEBUG_APIV1_SETWATCHPOINT   0x10

#define STLINK_DEBUG_ENTER_JTAG            0x00
#define STLINK_DEBUG_ENTER_SWD             0xa3

#define STLINK_DEBUG_APIV1_ENTER           0x20
#define STLINK_DEBUG_EXIT                  0x21
#define STLINK_DEBUG_READCOREID            0x22

#define STLINK_DEBUG_APIV2_ENTER           0x30
#define STLINK_DEBUG_APIV2_READ_IDCODES    0x31
#define STLINK_DEBUG_APIV2_RESETSYS        0x32
#define STLINK_DEBUG_APIV2_READREG         0x33
#define STLINK_DEBUG_APIV2_WRITEREG        0x34
#define STLINK_DEBUG_APIV2_WRITEDEBUGREG   0x35
#define STLINK_DEBUG_APIV2_READDEBUGREG    0x36

#define STLINK_DEBUG_APIV2_READALLREGS     0x3A
#define STLINK_DEBUG_APIV2_GETLASTRWSTATUS 0x3B
#define STLINK_DEBUG_APIV2_DRIVE_NRST      0x3C

#define STLINK_DEBUG_APIV2_START_TRACE_RX  0x40
#define STLINK_DEBUG_APIV2_STOP_TRACE_RX   0x41
#define STLINK_DEBUG_APIV2_GET_TRACE_NB    0x42

#define STLINK_DEBUG_APIV2_DRIVE_NRST_LOW   0x00
#define STLINK_DEBUG_APIV2_DRIVE_NRST_HIGH  0x01
#define STLINK_DEBUG_APIV2_DRIVE_NRST_PULSE 0x02

#define STLINK_TRACE_SIZE               1024
#define STLINK_TRACE_MAX_HZ             2000000
#define STLINK_TRACE_MIN_VERSION        13

/** */
enum stlink_mode {
	STLINK_MODE_UNKNOWN = 0,
	STLINK_MODE_DFU,
	STLINK_MODE_MASS,
	STLINK_MODE_DEBUG_JTAG,
	STLINK_MODE_DEBUG_SWD,
	STLINK_MODE_DEBUG_SWIM
};

#define REQUEST_SENSE        0x03
#define REQUEST_SENSE_LENGTH 18

static void stlink_usb_init_buffer(void *handle, uint8_t direction, uint32_t size);

/** */
static int stlink_usb_xfer_v1_get_status(void *handle)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* read status */
	memset(h->cmdbuf, 0, STLINK_SG_SIZE);

	if (jtag_libusb_bulk_read(h->fd, h->rx_ep, (char *)h->cmdbuf,
			13, STLINK_READ_TIMEOUT) != 13)
		return ERROR_FAIL;

	uint32_t t1;

	t1 = buf_get_u32(h->cmdbuf, 0, 32);

	/* check for USBS */
	if (t1 != 0x53425355)
		return ERROR_FAIL;
	/*
	 * CSW status:
	 * 0 success
	 * 1 command failure
	 * 2 phase error
	 */
	if (h->cmdbuf[12] != 0)
		return ERROR_FAIL;

	return ERROR_OK;
}

/** */
static int stlink_usb_xfer_rw(void *handle, int cmdsize, const uint8_t *buf, int size)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (jtag_libusb_bulk_write(h->fd, h->tx_ep, (char *)h->cmdbuf, cmdsize,
			STLINK_WRITE_TIMEOUT) != cmdsize) {
		return ERROR_FAIL;
	}

	if (h->direction == h->tx_ep && size) {
		if (jtag_libusb_bulk_write(h->fd, h->tx_ep, (char *)buf,
				size, STLINK_WRITE_TIMEOUT) != size) {
			LOG_DEBUG("bulk write failed");
			return ERROR_FAIL;
		}
	} else if (h->direction == h->rx_ep && size) {
		if (jtag_libusb_bulk_read(h->fd, h->rx_ep, (char *)buf,
				size, STLINK_READ_TIMEOUT) != size) {
			LOG_DEBUG("bulk read failed");
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

/** */
static int stlink_usb_xfer_v1_get_sense(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, h->rx_ep, 16);

	h->cmdbuf[h->cmdidx++] = REQUEST_SENSE;
	h->cmdbuf[h->cmdidx++] = 0;
	h->cmdbuf[h->cmdidx++] = 0;
	h->cmdbuf[h->cmdidx++] = 0;
	h->cmdbuf[h->cmdidx++] = REQUEST_SENSE_LENGTH;

	res = stlink_usb_xfer_rw(handle, REQUEST_SENSE_LENGTH, h->databuf, 16);

	if (res != ERROR_OK)
		return res;

	if (stlink_usb_xfer_v1_get_status(handle) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

/** */
static int stlink_usb_xfer(void *handle, const uint8_t *buf, int size)
{
	int err, cmdsize = STLINK_CMD_SIZE_V2;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->version.stlink == 1)
		cmdsize = STLINK_SG_SIZE;

	err = stlink_usb_xfer_rw(handle, cmdsize, buf, size);

	if (err != ERROR_OK)
		return err;

	if (h->version.stlink == 1) {
		if (stlink_usb_xfer_v1_get_status(handle) != ERROR_OK) {
			/* check csw status */
			if (h->cmdbuf[12] == 1) {
				LOG_DEBUG("get sense");
				if (stlink_usb_xfer_v1_get_sense(handle) != ERROR_OK)
					return ERROR_FAIL;
			}
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

/** */
static int stlink_usb_read_trace(void *handle, const uint8_t *buf, int size)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	assert(h->version.stlink >= 2);

	if (jtag_libusb_bulk_read(h->fd, h->trace_ep, (char *)buf,
			size, STLINK_READ_TIMEOUT) != size) {
		LOG_ERROR("bulk trace read failed");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/** */
static void stlink_usb_xfer_v1_create_cmd(void *handle, uint8_t direction, uint32_t size)
{
	struct stlink_usb_handle_s *h = handle;

	/* fill the send buffer */
	strcpy((char *)h->cmdbuf, "USBC");
	h->cmdidx += 4;
	/* csw tag not used */
	h->cmdidx += 4;
	buf_set_u32(h->cmdbuf+h->cmdidx, 0, 32, size);
	h->cmdidx += 4;
	h->cmdbuf[h->cmdidx++] = (direction == h->rx_ep ? ENDPOINT_IN : ENDPOINT_OUT);
	h->cmdbuf[h->cmdidx++] = 0; /* lun */
	h->cmdbuf[h->cmdidx++] = STLINK_CMD_SIZE_V1;
}

/** */
static void stlink_usb_init_buffer(void *handle, uint8_t direction, uint32_t size)
{
	struct stlink_usb_handle_s *h = handle;

	h->direction = direction;

	h->cmdidx = 0;

	memset(h->cmdbuf, 0, STLINK_SG_SIZE);
	memset(h->databuf, 0, STLINK_DATA_SIZE);

	if (h->version.stlink == 1)
		stlink_usb_xfer_v1_create_cmd(handle, direction, size);
}

static const char * const stlink_usb_error_msg[] = {
	"unknown"
};

/** */
static int stlink_usb_error_check(void *handle)
{
	int res;
	const char *err_msg = 0;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* TODO: no error checking yet on api V1 */
	if (h->jtag_api == STLINK_JTAG_API_V1)
		h->databuf[0] = STLINK_DEBUG_ERR_OK;

	switch (h->databuf[0]) {
		case STLINK_DEBUG_ERR_OK:
			res = ERROR_OK;
			break;
		case STLINK_DEBUG_ERR_FAULT:
		default:
			err_msg = stlink_usb_error_msg[0];
			res = ERROR_FAIL;
			break;
	}

	if (res != ERROR_OK)
		LOG_DEBUG("status error: %d ('%s')", h->databuf[0], err_msg);

	return res;
}

/** */
static int stlink_usb_version(void *handle)
{
	int res;
	uint16_t v;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, h->rx_ep, 6);

	h->cmdbuf[h->cmdidx++] = STLINK_GET_VERSION;

	res = stlink_usb_xfer(handle, h->databuf, 6);

	if (res != ERROR_OK)
		return res;

	v = (h->databuf[0] << 8) | h->databuf[1];

	h->version.stlink = (v >> 12) & 0x0f;
	h->version.jtag = (v >> 6) & 0x3f;
	h->version.swim = v & 0x3f;
	h->vid = buf_get_u32(h->databuf, 16, 16);
	h->pid = buf_get_u32(h->databuf, 32, 16);

	/* set the supported jtag api version
	 * API V2 is supported since JTAG V11
	 */
	if (h->version.jtag >= 11)
		h->version.jtag_api_max = STLINK_JTAG_API_V2;
	else
		h->version.jtag_api_max = STLINK_JTAG_API_V1;

	LOG_INFO("STLINK v%d JTAG v%d API v%d SWIM v%d VID 0x%04X PID 0x%04X",
		h->version.stlink,
		h->version.jtag,
		(h->version.jtag_api_max == STLINK_JTAG_API_V1) ? 1 : 2,
		h->version.swim,
		h->vid,
		h->pid);

	return ERROR_OK;
}

static int stlink_usb_check_voltage(void *handle, float *target_voltage)
{
	struct stlink_usb_handle_s *h = handle;
	uint32_t adc_results[2];

	/* only supported by stlink/v2 and for firmware >= 13 */
	if (h->version.stlink == 1 || h->version.jtag < 13)
		return ERROR_COMMAND_NOTFOUND;

	stlink_usb_init_buffer(handle, h->rx_ep, 8);

	h->cmdbuf[h->cmdidx++] = STLINK_GET_TARGET_VOLTAGE;

	int result = stlink_usb_xfer(handle, h->databuf, 8);

	if (result != ERROR_OK)
		return result;

	/* convert result */
	adc_results[0] = le_to_h_u32(h->databuf);
	adc_results[1] = le_to_h_u32(h->databuf + 4);

	*target_voltage = 0;

	if (adc_results[0])
		*target_voltage = 2 * ((float)adc_results[1]) * (float)(1.2 / adc_results[0]);

	LOG_INFO("Target voltage: %f", (double)*target_voltage);

	return ERROR_OK;
}

/** */
static int stlink_usb_current_mode(void *handle, uint8_t *mode)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_GET_CURRENT_MODE;

	res = stlink_usb_xfer(handle, h->databuf, 2);

	if (res != ERROR_OK)
		return res;

	*mode = h->databuf[0];

	return ERROR_OK;
}

/** */
static int stlink_usb_mode_enter(void *handle, enum stlink_mode type)
{
	int res;
	int rx_size = 0;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* on api V2 we are able the read the latest command
	 * status
	 * TODO: we need the test on api V1 too
	 */
	if (h->jtag_api == STLINK_JTAG_API_V2)
		rx_size = 2;

	stlink_usb_init_buffer(handle, h->rx_ep, rx_size);

	switch (type) {
		case STLINK_MODE_DEBUG_JTAG:
			h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
			if (h->jtag_api == STLINK_JTAG_API_V1)
				h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV1_ENTER;
			else
				h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_ENTER;
			h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_ENTER_JTAG;
			break;
		case STLINK_MODE_DEBUG_SWD:
			h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
			if (h->jtag_api == STLINK_JTAG_API_V1)
				h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV1_ENTER;
			else
				h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_ENTER;
			h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_ENTER_SWD;
			break;
		case STLINK_MODE_DEBUG_SWIM:
			h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
			h->cmdbuf[h->cmdidx++] = STLINK_SWIM_ENTER;
			break;
		case STLINK_MODE_DFU:
		case STLINK_MODE_MASS:
		default:
			return ERROR_FAIL;
	}

	res = stlink_usb_xfer(handle, h->databuf, rx_size);

	if (res != ERROR_OK)
		return res;

	res = stlink_usb_error_check(h);

	return res;
}

/** */
static int stlink_usb_mode_leave(void *handle, enum stlink_mode type)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, STLINK_NULL_EP, 0);

	switch (type) {
		case STLINK_MODE_DEBUG_JTAG:
		case STLINK_MODE_DEBUG_SWD:
			h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
			h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_EXIT;
			break;
		case STLINK_MODE_DEBUG_SWIM:
			h->cmdbuf[h->cmdidx++] = STLINK_SWIM_COMMAND;
			h->cmdbuf[h->cmdidx++] = STLINK_SWIM_EXIT;
			break;
		case STLINK_MODE_DFU:
			h->cmdbuf[h->cmdidx++] = STLINK_DFU_COMMAND;
			h->cmdbuf[h->cmdidx++] = STLINK_DFU_EXIT;
			break;
		case STLINK_MODE_MASS:
		default:
			return ERROR_FAIL;
	}

	res = stlink_usb_xfer(handle, 0, 0);

	if (res != ERROR_OK)
		return res;

	return ERROR_OK;
}

static int stlink_usb_assert_srst(void *handle, int srst);

/** */
static int stlink_usb_init_mode(void *handle, bool connect_under_reset)
{
	int res;
	uint8_t mode;
	enum stlink_mode emode;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	res = stlink_usb_current_mode(handle, &mode);

	if (res != ERROR_OK)
		return res;

	LOG_DEBUG("MODE: 0x%02X", mode);

	/* try to exit current mode */
	switch (mode) {
		case STLINK_DEV_DFU_MODE:
			emode = STLINK_MODE_DFU;
			break;
		case STLINK_DEV_DEBUG_MODE:
			emode = STLINK_MODE_DEBUG_SWD;
			break;
		case STLINK_DEV_SWIM_MODE:
			emode = STLINK_MODE_DEBUG_SWIM;
			break;
		case STLINK_DEV_BOOTLOADER_MODE:
		case STLINK_DEV_MASS_MODE:
		default:
			emode = STLINK_MODE_UNKNOWN;
			break;
	}

	if (emode != STLINK_MODE_UNKNOWN) {
		res = stlink_usb_mode_leave(handle, emode);

		if (res != ERROR_OK)
			return res;
	}

	res = stlink_usb_current_mode(handle, &mode);

	if (res != ERROR_OK)
		return res;

	/* we check the target voltage here as an aid to debugging connection problems.
	 * the stlink requires the target Vdd to be connected for reliable debugging.
	 * this cmd is supported in all modes except DFU
	 */
	if (mode != STLINK_DEV_DFU_MODE) {

		float target_voltage;

		/* check target voltage (if supported) */
		res = stlink_usb_check_voltage(h, &target_voltage);

		if (res != ERROR_OK) {
			if (res != ERROR_COMMAND_NOTFOUND)
				LOG_ERROR("voltage check failed");
			/* attempt to continue as it is not a catastrophic failure */
		} else {
			/* check for a sensible target voltage, operating range is 1.65-5.5v
			 * according to datasheet */
			if (target_voltage < 1.5)
				LOG_ERROR("target voltage may be too low for reliable debugging");
		}
	}

	LOG_DEBUG("MODE: 0x%02X", mode);

	/* set selected mode */
	switch (h->transport) {
		case HL_TRANSPORT_SWD:
			emode = STLINK_MODE_DEBUG_SWD;
			break;
		case HL_TRANSPORT_JTAG:
			emode = STLINK_MODE_DEBUG_JTAG;
			break;
		case HL_TRANSPORT_SWIM:
			emode = STLINK_MODE_DEBUG_SWIM;
			break;
		default:
			emode = STLINK_MODE_UNKNOWN;
			break;
	}

	if (emode == STLINK_MODE_UNKNOWN) {
		LOG_ERROR("selected mode (transport) not supported");
		return ERROR_FAIL;
	}

	if (connect_under_reset) {
		res = stlink_usb_assert_srst(handle, 0);
		if (res != ERROR_OK)
			return res;
	}

	res = stlink_usb_mode_enter(handle, emode);

	if (res != ERROR_OK)
		return res;

	res = stlink_usb_current_mode(handle, &mode);

	if (res != ERROR_OK)
		return res;

	LOG_DEBUG("MODE: 0x%02X", mode);

	return ERROR_OK;
}

/** */
static int stlink_usb_idcode(void *handle, uint32_t *idcode)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, h->rx_ep, 4);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_READCOREID;

	res = stlink_usb_xfer(handle, h->databuf, 4);

	if (res != ERROR_OK)
		return res;

	*idcode = le_to_h_u32(h->databuf);

	LOG_DEBUG("IDCODE: 0x%08" PRIX32, *idcode);

	return ERROR_OK;
}

static int stlink_usb_v2_read_debug_reg(void *handle, uint32_t addr, uint32_t *val)
{
	struct stlink_usb_handle_s *h = handle;
	int res;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, h->rx_ep, 8);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_READDEBUGREG;
	h_u32_to_le(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;

	res = stlink_usb_xfer(handle, h->databuf, 8);

	if (res != ERROR_OK)
		return res;

	*val = le_to_h_u32(h->databuf + 4);

	return h->databuf[0] == STLINK_DEBUG_ERR_OK ? ERROR_OK : ERROR_FAIL;
}

static int stlink_usb_write_debug_reg(void *handle, uint32_t addr, uint32_t val)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	if (h->jtag_api == STLINK_JTAG_API_V1)
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV1_WRITEDEBUGREG;
	else
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_WRITEDEBUGREG;
	h_u32_to_le(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;
	h_u32_to_le(h->cmdbuf+h->cmdidx, val);
	h->cmdidx += 4;

	res = stlink_usb_xfer(handle, h->databuf, 2);

	if (res != ERROR_OK)
		return res;

	return h->databuf[0] == STLINK_DEBUG_ERR_OK ? ERROR_OK : ERROR_FAIL;
}

/** */
static void stlink_usb_trace_read(void *handle)
{
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->trace.enabled && h->version.jtag >= STLINK_TRACE_MIN_VERSION) {
		int res;

		stlink_usb_init_buffer(handle, h->rx_ep, 10);

		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_GET_TRACE_NB;

		res = stlink_usb_xfer(handle, h->databuf, 2);
		if (res == ERROR_OK) {
			uint8_t buf[STLINK_TRACE_SIZE];
			size_t size = le_to_h_u16(h->databuf);

			if (size > 0) {
				size = size < sizeof(buf) ? size : sizeof(buf) - 1;

				res = stlink_usb_read_trace(handle, buf, size);
				if (res == ERROR_OK) {
					if (h->trace.output_f) {
						/* Log retrieved trace output */
						if (fwrite(buf, 1, size, h->trace.output_f) > 0)
							fflush(h->trace.output_f);
					}
				}
			}
		}
	}
}

static int stlink_usb_trace_read_callback(void *handle)
{
	stlink_usb_trace_read(handle);
	return ERROR_OK;
}

static enum target_state stlink_usb_v2_get_status(void *handle)
{
	int result;
	uint32_t status;

	result = stlink_usb_v2_read_debug_reg(handle, DCB_DHCSR, &status);
	if  (result != ERROR_OK)
		return TARGET_UNKNOWN;

	if (status & S_HALT)
		return TARGET_HALTED;
	else if (status & S_RESET_ST)
		return TARGET_RESET;

	stlink_usb_trace_read(handle);

	return TARGET_RUNNING;
}

/** */
static enum target_state stlink_usb_state(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->jtag_api == STLINK_JTAG_API_V2)
		return stlink_usb_v2_get_status(handle);

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_GETSTATUS;

	res = stlink_usb_xfer(handle, h->databuf, 2);

	if (res != ERROR_OK)
		return TARGET_UNKNOWN;

	if (h->databuf[0] == STLINK_CORE_RUNNING)
		return TARGET_RUNNING;
	if (h->databuf[0] == STLINK_CORE_HALTED)
		return TARGET_HALTED;

	return TARGET_UNKNOWN;
}

/** */
static int stlink_usb_reset(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;

	if (h->jtag_api == STLINK_JTAG_API_V1)
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV1_RESETSYS;
	else
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_RESETSYS;

	res = stlink_usb_xfer(handle, h->databuf, 2);

	if (res != ERROR_OK)
		return res;

	LOG_DEBUG("RESET: 0x%08X", h->databuf[0]);

	/* the following is not a error under swd (using hardware srst), so return success */
	if (h->databuf[0] == STLINK_SWD_AP_WAIT || h->databuf[0] == STLINK_SWD_DP_WAIT)
		return ERROR_OK;

	return h->databuf[0] == STLINK_DEBUG_ERR_OK ? ERROR_OK : ERROR_FAIL;
}

static int stlink_usb_assert_srst(void *handle, int srst)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->jtag_api == STLINK_JTAG_API_V1)
		return ERROR_COMMAND_NOTFOUND;

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_DRIVE_NRST;
	h->cmdbuf[h->cmdidx++] = srst;

	res = stlink_usb_xfer(handle, h->databuf, 2);

	if (res != ERROR_OK)
		return res;

	return h->databuf[0] == STLINK_DEBUG_ERR_OK ? ERROR_OK : ERROR_FAIL;
}

/** */
static int stlink_configure_target_trace_port(void *handle)
{
	int res;
	uint32_t reg;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* configure the TPI */

	/* enable the trace subsystem */
	res = stlink_usb_v2_read_debug_reg(handle, DCB_DEMCR, &reg);
	if (res != ERROR_OK)
		goto out;
	res = stlink_usb_write_debug_reg(handle, DCB_DEMCR, TRCENA|reg);
	if (res != ERROR_OK)
		goto out;
	/* set the TPI clock prescaler */
	res = stlink_usb_write_debug_reg(handle, TPI_ACPR, h->trace.prescale);
	if (res != ERROR_OK)
		goto out;
	/* select the pin protocol.  The STLinkv2 only supports asynchronous
	 * UART emulation (NRZ) mode, so that's what we pick. */
	res = stlink_usb_write_debug_reg(handle, TPI_SPPR, 0x02);
	if (res != ERROR_OK)
		goto out;
	/* disable continuous formatting */
	res = stlink_usb_write_debug_reg(handle, TPI_FFCR, (1<<8));
	if (res != ERROR_OK)
		goto out;

	/* configure the ITM */

	/* unlock access to the ITM registers */
	res = stlink_usb_write_debug_reg(handle, ITM_LAR, 0xC5ACCE55);
	if (res != ERROR_OK)
		goto out;
	/* enable trace with ATB ID 1 */
	res = stlink_usb_write_debug_reg(handle, ITM_TCR, (1<<16)|(1<<0)|(1<<2));
	if (res != ERROR_OK)
		goto out;
	/* trace privilege */
	res = stlink_usb_write_debug_reg(handle, ITM_TPR, 1);
	if (res != ERROR_OK)
		goto out;
	/* trace port enable (port 0) */
	res = stlink_usb_write_debug_reg(handle, ITM_TER, (1<<0));
	if (res != ERROR_OK)
		goto out;

	res = ERROR_OK;
out:
	return res;
}

/** */
static void stlink_usb_trace_disable(void *handle)
{
	int res = ERROR_OK;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	assert(h->version.jtag >= STLINK_TRACE_MIN_VERSION);

	LOG_DEBUG("Tracing: disable\n");

	stlink_usb_init_buffer(handle, h->rx_ep, 2);
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_STOP_TRACE_RX;
	res = stlink_usb_xfer(handle, h->databuf, 2);

	if (res == ERROR_OK) {
		h->trace.enabled = false;
		target_unregister_timer_callback(stlink_usb_trace_read_callback, handle);
	}
}


/** */
static int stlink_usb_trace_enable(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->version.jtag >= STLINK_TRACE_MIN_VERSION) {
		uint32_t trace_hz;

		res = stlink_configure_target_trace_port(handle);
		if (res != ERROR_OK)
			LOG_ERROR("Unable to configure tracing on target\n");

		trace_hz = h->trace.prescale > 0 ?
			h->trace.source_hz / (h->trace.prescale + 1) :
			h->trace.source_hz;

		stlink_usb_init_buffer(handle, h->rx_ep, 10);

		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_START_TRACE_RX;
		h_u16_to_le(h->cmdbuf+h->cmdidx, (uint16_t)STLINK_TRACE_SIZE);
		h->cmdidx += 2;
		h_u32_to_le(h->cmdbuf+h->cmdidx, trace_hz);
		h->cmdidx += 4;

		res = stlink_usb_xfer(handle, h->databuf, 2);

		if (res == ERROR_OK)  {
			h->trace.enabled = true;
			LOG_DEBUG("Tracing: recording at %" PRIu32 "Hz\n", trace_hz);
			/* We need the trace read function to be called at a
			 * high-enough frequency to ensure reasonable
			 * "timeliness" in processing ITM/DWT data.
			 * TODO: An alternative could be using the asynchronous
			 * features of the libusb-1.0 API to queue up one or more
			 * reads in advance and requeue them once they are
			 * completed. */
			target_register_timer_callback(stlink_usb_trace_read_callback, 1, 1, handle);
		}
	} else {
		LOG_ERROR("Tracing is not supported by this version.");
		res = ERROR_FAIL;
	}

	return res;
}

/** */
static int stlink_usb_run(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->jtag_api == STLINK_JTAG_API_V2) {
		res = stlink_usb_write_debug_reg(handle, DCB_DHCSR, DBGKEY|C_DEBUGEN);

		/* Try to start tracing, if requested */
		if (res == ERROR_OK && h->trace.source_hz && !h->trace.enabled) {
			if (stlink_usb_trace_enable(handle) == ERROR_OK)
				LOG_DEBUG("Tracing: enabled\n");
			else
				LOG_ERROR("Tracing: enable failed\n");
		}

		return res;
	}

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_RUNCORE;

	res = stlink_usb_xfer(handle, h->databuf, 2);

	if (res != ERROR_OK)
		return res;

	return h->databuf[0] == STLINK_DEBUG_ERR_OK ? ERROR_OK : ERROR_FAIL;
}

/** */
static int stlink_usb_halt(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->jtag_api == STLINK_JTAG_API_V2) {
		res = stlink_usb_write_debug_reg(handle, DCB_DHCSR, DBGKEY|C_HALT|C_DEBUGEN);

		if (res == ERROR_OK && h->trace.enabled)
			stlink_usb_trace_disable(handle);

		return res;
	}

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_FORCEDEBUG;

	res = stlink_usb_xfer(handle, h->databuf, 2);

	if (res != ERROR_OK)
		return res;

	return h->databuf[0] == STLINK_DEBUG_ERR_OK ? ERROR_OK : ERROR_FAIL;
}

/** */
static int stlink_usb_step(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->jtag_api == STLINK_JTAG_API_V2) {
		/* TODO: this emulates the v1 api, it should really use a similar auto mask isr
		 * that the cortex-m3 currently does. */
		stlink_usb_write_debug_reg(handle, DCB_DHCSR, DBGKEY|C_HALT|C_MASKINTS|C_DEBUGEN);
		stlink_usb_write_debug_reg(handle, DCB_DHCSR, DBGKEY|C_STEP|C_MASKINTS|C_DEBUGEN);
		return stlink_usb_write_debug_reg(handle, DCB_DHCSR, DBGKEY|C_HALT|C_DEBUGEN);
	}

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_STEPCORE;

	res = stlink_usb_xfer(handle, h->databuf, 2);

	if (res != ERROR_OK)
		return res;

	return h->databuf[0] == STLINK_DEBUG_ERR_OK ? ERROR_OK : ERROR_FAIL;
}

/** */
static int stlink_usb_read_regs(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, h->rx_ep, 84);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	if (h->jtag_api == STLINK_JTAG_API_V1)
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV1_READALLREGS;
	else
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_READALLREGS;

	res = stlink_usb_xfer(handle, h->databuf, 84);

	if (res != ERROR_OK)
		return res;

	return ERROR_OK;
}

/** */
static int stlink_usb_read_reg(void *handle, int num, uint32_t *val)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, h->rx_ep, h->jtag_api == STLINK_JTAG_API_V1 ? 4 : 8);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	if (h->jtag_api == STLINK_JTAG_API_V1)
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV1_READREG;
	else
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_READREG;
	h->cmdbuf[h->cmdidx++] = num;

	res = stlink_usb_xfer(handle, h->databuf, h->jtag_api == STLINK_JTAG_API_V1 ? 4 : 8);

	if (res != ERROR_OK)
		return res;

	if (h->jtag_api == STLINK_JTAG_API_V1)
		*val = le_to_h_u32(h->databuf);
	else {
		*val = le_to_h_u32(h->databuf + 4);
		return h->databuf[0] == STLINK_DEBUG_ERR_OK ? ERROR_OK : ERROR_FAIL;
	}

	return ERROR_OK;
}

/** */
static int stlink_usb_write_reg(void *handle, int num, uint32_t val)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	if (h->jtag_api == STLINK_JTAG_API_V1)
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV1_WRITEREG;
	else
		h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_WRITEREG;
	h->cmdbuf[h->cmdidx++] = num;
	h_u32_to_le(h->cmdbuf+h->cmdidx, val);
	h->cmdidx += 4;

	res = stlink_usb_xfer(handle, h->databuf, 2);

	if (res != ERROR_OK)
		return res;

	return h->databuf[0] == STLINK_DEBUG_ERR_OK ? ERROR_OK : ERROR_FAIL;
}

static int stlink_usb_get_rw_status(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	if (h->jtag_api == STLINK_JTAG_API_V1)
		return ERROR_OK;

	stlink_usb_init_buffer(handle, h->rx_ep, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_GETLASTRWSTATUS;

	res = stlink_usb_xfer(handle, h->databuf, 2);

	if (res != ERROR_OK)
		return res;

	return h->databuf[0] == STLINK_DEBUG_ERR_OK ? ERROR_OK : res;
}

/** */
static int stlink_usb_read_mem8(void *handle, uint32_t addr, uint16_t len,
			  uint8_t *buffer)
{
	int res;
	uint16_t read_len = len;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* max 8bit read/write is 64bytes */
	if (len > STLINK_MAX_RW8) {
		LOG_DEBUG("max buffer length exceeded");
		return ERROR_FAIL;
	}

	stlink_usb_init_buffer(handle, h->rx_ep, read_len);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_READMEM_8BIT;
	h_u32_to_le(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;
	h_u16_to_le(h->cmdbuf+h->cmdidx, len);
	h->cmdidx += 2;

	/* we need to fix read length for single bytes */
	if (read_len == 1)
		read_len++;

	res = stlink_usb_xfer(handle, h->databuf, read_len);

	if (res != ERROR_OK)
		return res;

	memcpy(buffer, h->databuf, len);

	return stlink_usb_get_rw_status(handle);
}

/** */
static int stlink_usb_write_mem8(void *handle, uint32_t addr, uint16_t len,
			   const uint8_t *buffer)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* max 8bit read/write is 64bytes */
	if (len > STLINK_MAX_RW8) {
		LOG_DEBUG("max buffer length exceeded");
		return ERROR_FAIL;
	}

	stlink_usb_init_buffer(handle, h->tx_ep, len);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_WRITEMEM_8BIT;
	h_u32_to_le(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;
	h_u16_to_le(h->cmdbuf+h->cmdidx, len);
	h->cmdidx += 2;

	res = stlink_usb_xfer(handle, buffer, len);

	if (res != ERROR_OK)
		return res;

	return stlink_usb_get_rw_status(handle);
}

/** */
static int stlink_usb_read_mem32(void *handle, uint32_t addr, uint16_t len,
			  uint8_t *buffer)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* data must be a multiple of 4 and word aligned */
	if (len % 4 || addr % 4) {
		LOG_DEBUG("Invalid data alignment");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	stlink_usb_init_buffer(handle, h->rx_ep, len);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_READMEM_32BIT;
	h_u32_to_le(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;
	h_u16_to_le(h->cmdbuf+h->cmdidx, len);
	h->cmdidx += 2;

	res = stlink_usb_xfer(handle, h->databuf, len);

	if (res != ERROR_OK)
		return res;

	memcpy(buffer, h->databuf, len);

	return stlink_usb_get_rw_status(handle);
}

/** */
static int stlink_usb_write_mem32(void *handle, uint32_t addr, uint16_t len,
			   const uint8_t *buffer)
{
	int res;
	struct stlink_usb_handle_s *h = handle;

	assert(handle != NULL);

	/* data must be a multiple of 4 and word aligned */
	if (len % 4 || addr % 4) {
		LOG_DEBUG("Invalid data alignment");
		return ERROR_TARGET_UNALIGNED_ACCESS;
	}

	stlink_usb_init_buffer(handle, h->tx_ep, len);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_WRITEMEM_32BIT;
	h_u32_to_le(h->cmdbuf+h->cmdidx, addr);
	h->cmdidx += 4;
	h_u16_to_le(h->cmdbuf+h->cmdidx, len);
	h->cmdidx += 2;

	res = stlink_usb_xfer(handle, buffer, len);

	if (res != ERROR_OK)
		return res;

	return stlink_usb_get_rw_status(handle);
}

static uint32_t stlink_max_block_size(uint32_t tar_autoincr_block, uint32_t address)
{
	uint32_t max_tar_block = (tar_autoincr_block - ((tar_autoincr_block - 1) & address));
	if (max_tar_block == 0)
		max_tar_block = 4;
	return max_tar_block;
}

static int stlink_usb_read_mem(void *handle, uint32_t addr, uint32_t size,
		uint32_t count, uint8_t *buffer)
{
	int retval = ERROR_OK;
	uint32_t bytes_remaining;
	struct stlink_usb_handle_s *h = handle;

	/* calculate byte count */
	count *= size;

	while (count) {

		bytes_remaining = (size == 4) ? \
				stlink_max_block_size(h->max_mem_packet, addr) : STLINK_MAX_RW8;

		if (count < bytes_remaining)
			bytes_remaining = count;

		/* the stlink only supports 8/32bit memory read/writes
		 * honour 32bit, all others will be handled as 8bit access */
		if (size == 4) {

			/* When in jtag mode the stlink uses the auto-increment functinality.
			 * However it expects us to pass the data correctly, this includes
			 * alignment and any page boundaries. We already do this as part of the
			 * adi_v5 implementation, but the stlink is a hla adapter and so this
			 * needs implementiong manually.
			 * currently this only affects jtag mode, according to ST they do single
			 * access in SWD mode - but this may change and so we do it for both modes */

			/* we first need to check for any unaligned bytes */
			if (addr % 4) {

				uint32_t head_bytes = 4 - (addr % 4);
				retval = stlink_usb_read_mem8(handle, addr, head_bytes, buffer);
				if (retval != ERROR_OK)
					return retval;
				buffer += head_bytes;
				addr += head_bytes;
				count -= head_bytes;
				bytes_remaining -= head_bytes;
			}

			if (bytes_remaining % 4)
				retval = stlink_usb_read_mem(handle, addr, 1, bytes_remaining, buffer);
			else
				retval = stlink_usb_read_mem32(handle, addr, bytes_remaining, buffer);
		} else
			retval = stlink_usb_read_mem8(handle, addr, bytes_remaining, buffer);

		if (retval != ERROR_OK)
			return retval;

		buffer += bytes_remaining;
		addr += bytes_remaining;
		count -= bytes_remaining;
	}

	return retval;
}

static int stlink_usb_write_mem(void *handle, uint32_t addr, uint32_t size,
		uint32_t count, const uint8_t *buffer)
{
	int retval = ERROR_OK;
	uint32_t bytes_remaining;
	struct stlink_usb_handle_s *h = handle;

	/* calculate byte count */
	count *= size;

	while (count) {

		bytes_remaining = (size == 4) ? \
				stlink_max_block_size(h->max_mem_packet, addr) : STLINK_MAX_RW8;

		if (count < bytes_remaining)
			bytes_remaining = count;

		/* the stlink only supports 8/32bit memory read/writes
		 * honour 32bit, all others will be handled as 8bit access */
		if (size == 4) {

			/* When in jtag mode the stlink uses the auto-increment functinality.
			 * However it expects us to pass the data correctly, this includes
			 * alignment and any page boundaries. We already do this as part of the
			 * adi_v5 implementation, but the stlink is a hla adapter and so this
			 * needs implementiong manually.
			 * currently this only affects jtag mode, according to ST they do single
			 * access in SWD mode - but this may change and so we do it for both modes */

			/* we first need to check for any unaligned bytes */
			if (addr % 4) {

				uint32_t head_bytes = 4 - (addr % 4);
				retval = stlink_usb_write_mem8(handle, addr, head_bytes, buffer);
				if (retval != ERROR_OK)
					return retval;
				buffer += head_bytes;
				addr += head_bytes;
				count -= head_bytes;
				bytes_remaining -= head_bytes;
			}

			if (bytes_remaining % 4)
				retval = stlink_usb_write_mem(handle, addr, 1, bytes_remaining, buffer);
			else
				retval = stlink_usb_write_mem32(handle, addr, bytes_remaining, buffer);

		} else
			retval = stlink_usb_write_mem8(handle, addr, bytes_remaining, buffer);
		if (retval != ERROR_OK)
			return retval;

		buffer += bytes_remaining;
		addr += bytes_remaining;
		count -= bytes_remaining;
	}

	return retval;
}

/** */
static int stlink_usb_close(void *fd)
{
	struct stlink_usb_handle_s *h = fd;

	if (h->fd)
		jtag_libusb_close(h->fd);

	free(fd);

	return ERROR_OK;
}

/** */
static int stlink_usb_open(struct hl_interface_param_s *param, void **fd)
{
	int err, retry_count = 1;
	struct stlink_usb_handle_s *h;
	enum stlink_jtag_api_version api;

	LOG_DEBUG("stlink_usb_open");

	h = calloc(1, sizeof(struct stlink_usb_handle_s));

	if (h == 0) {
		LOG_DEBUG("malloc failed");
		return ERROR_FAIL;
	}

	h->transport = param->transport;

	const uint16_t vids[] = { param->vid, 0 };
	const uint16_t pids[] = { param->pid, 0 };

	LOG_DEBUG("transport: %d vid: 0x%04x pid: 0x%04x", param->transport,
		param->vid, param->pid);

	/*
	  On certain host USB configurations(e.g. MacBook Air)
	  STLINKv2 dongle seems to have its FW in a funky state if,
	  after plugging it in, you try to use openocd with it more
	  then once (by launching and closing openocd). In cases like
	  that initial attempt to read the FW info via
	  stlink_usb_version will fail and the device has to be reset
	  in order to become operational.
	 */
	do {
		if (jtag_libusb_open(vids, pids, &h->fd) != ERROR_OK) {
			LOG_ERROR("open failed");
			goto error_open;
		}

		jtag_libusb_set_configuration(h->fd, 0);

		if (jtag_libusb_claim_interface(h->fd, 0) != ERROR_OK) {
			LOG_DEBUG("claim interface failed");
			goto error_open;
		}

		/* RX EP is common for all versions */
		h->rx_ep = STLINK_RX_EP;

		/* wrap version for first read */
		switch (param->pid) {
			case STLINK_V1_PID:
				h->version.stlink = 1;
				h->tx_ep = STLINK_TX_EP;
				h->trace_ep = STLINK_TRACE_EP;
				break;
			case STLINK_V2_1_PID:
				h->version.stlink = 2;
				h->tx_ep = STLINK_V2_1_TX_EP;
				h->trace_ep = STLINK_V2_1_TRACE_EP;
				break;
			default:
			/* fall through - we assume V2 to be the default version*/
			case STLINK_V2_PID:
				h->version.stlink = 2;
				h->tx_ep = STLINK_TX_EP;
				h->trace_ep = STLINK_TRACE_EP;
				break;
		}

		/* get the device version */
		err = stlink_usb_version(h);

		if (err == ERROR_OK) {
			break;
		} else if (h->version.stlink == 1 ||
			   retry_count == 0) {
			LOG_ERROR("read version failed");
			goto error_open;
		} else {
			err = jtag_libusb_release_interface(h->fd, 0);
			if (err != ERROR_OK) {
				LOG_ERROR("release interface failed");
				goto error_open;
			}

			err = jtag_libusb_reset_device(h->fd);
			if (err != ERROR_OK) {
				LOG_ERROR("reset device failed");
				goto error_open;
			}

			jtag_libusb_close(h->fd);
			/*
			  Give the device one second to settle down and
			  reenumerate.
			 */
			usleep(1 * 1000 * 1000);
			retry_count--;
		}
	} while (1);

	/* compare usb vid/pid */
	if ((param->vid != h->vid) || (param->pid != h->pid))
		LOG_INFO("vid/pid are not identical: 0x%04X/0x%04X 0x%04X/0x%04X",
			param->vid, param->pid,
			h->vid, h->pid);

	/* check if mode is supported */
	err = ERROR_OK;

	switch (h->transport) {
		case HL_TRANSPORT_SWD:
		case HL_TRANSPORT_JTAG:
			if (h->version.jtag == 0)
				err = ERROR_FAIL;
			break;
		case HL_TRANSPORT_SWIM:
			if (h->version.swim == 0)
				err = ERROR_FAIL;
			break;
		default:
			err = ERROR_FAIL;
			break;
	}

	if (err != ERROR_OK) {
		LOG_ERROR("mode (transport) not supported by device");
		goto error_open;
	}

	api = h->version.jtag_api_max;

	LOG_INFO("using stlink api v%d", api);

	/* set the used jtag api, this will default to the newest supported version */
	h->jtag_api = api;

	if (h->jtag_api >= 2 && param->trace_source_hz > 0) {
		uint32_t prescale;

		prescale = param->trace_source_hz > STLINK_TRACE_MAX_HZ ?
			(param->trace_source_hz / STLINK_TRACE_MAX_HZ) - 1 : 0;

		h->trace.output_f = param->trace_f;
		h->trace.source_hz = param->trace_source_hz;
		h->trace.prescale = prescale;
	}

	/* initialize the debug hardware */
	err = stlink_usb_init_mode(h, param->connect_under_reset);

	if (err != ERROR_OK) {
		LOG_ERROR("init mode failed");
		goto error_open;
	}

	/* get cpuid, so we can determine the max page size
	 * start with a safe default */
	h->max_mem_packet = (1 << 10);

	uint8_t buffer[4];
	err = stlink_usb_read_mem32(h, CPUID, 4, buffer);
	if (err == ERROR_OK) {
		uint32_t cpuid = le_to_h_u32(buffer);
		int i = (cpuid >> 4) & 0xf;
		if (i == 4 || i == 3) {
			/* Cortex-M3/M4 has 4096 bytes autoincrement range */
			h->max_mem_packet = (1 << 12);
		}
	}

	LOG_DEBUG("Using TAR autoincrement: %" PRIu32, h->max_mem_packet);

	*fd = h;

	return ERROR_OK;

error_open:
	stlink_usb_close(h);

	return ERROR_FAIL;
}

/** */
struct hl_layout_api_s stlink_usb_layout_api = {
	/** */
	.open = stlink_usb_open,
	/** */
	.close = stlink_usb_close,
	/** */
	.idcode = stlink_usb_idcode,
	/** */
	.state = stlink_usb_state,
	/** */
	.reset = stlink_usb_reset,
	/** */
	.assert_srst = stlink_usb_assert_srst,
	/** */
	.run = stlink_usb_run,
	/** */
	.halt = stlink_usb_halt,
	/** */
	.step = stlink_usb_step,
	/** */
	.read_regs = stlink_usb_read_regs,
	/** */
	.read_reg = stlink_usb_read_reg,
	/** */
	.write_reg = stlink_usb_write_reg,
	/** */
	.read_mem = stlink_usb_read_mem,
	/** */
	.write_mem = stlink_usb_write_mem,
	/** */
	.write_debug_reg = stlink_usb_write_debug_reg
};
