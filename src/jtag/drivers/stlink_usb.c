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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/* project specific includes */
#include <helper/binarybuffer.h>
#include <jtag/interface.h>
#include <jtag/stlink/stlink_layout.h>
#include <jtag/stlink/stlink_transport.h>
#include <jtag/stlink/stlink_interface.h>
#include <target/target.h>

#include <target/cortex_m.h>

#include "libusb_common.h"

#define ENDPOINT_IN	0x80
#define ENDPOINT_OUT	0x00

#define STLINK_NULL_EP		0
#define STLINK_RX_EP		(1|ENDPOINT_IN)
#define STLINK_TX_EP		(2|ENDPOINT_OUT)
#define STLINK_SG_SIZE		(31)
#define STLINK_DATA_SIZE	(4*128)
#define STLINK_CMD_SIZE_V2	(16)
#define STLINK_CMD_SIZE_V1	(10)

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
	uint8_t cmdbuf[STLINK_SG_SIZE];
	/** */
	uint8_t cmdidx;
	/** */
	uint8_t direction;
	/** */
	uint8_t databuf[STLINK_DATA_SIZE];
	/** */
	enum stlink_transports transport;
	/** */
	struct stlink_usb_version version;
	/** */
	uint16_t vid;
	/** */
	uint16_t pid;
	/** this is the currently used jtag api */
	enum stlink_jtag_api_version jtag_api;
};

#define STLINK_DEBUG_ERR_OK			0x80
#define STLINK_DEBUG_ERR_FAULT		0x81
#define STLINK_SWD_AP_WAIT			0x10
#define STLINK_SWD_DP_WAIT			0x14

#define STLINK_CORE_RUNNING			0x80
#define STLINK_CORE_HALTED			0x81
#define STLINK_CORE_STAT_UNKNOWN		-1

#define STLINK_GET_VERSION			0xF1
#define STLINK_DEBUG_COMMAND			0xF2
#define STLINK_DFU_COMMAND			0xF3
#define STLINK_SWIM_COMMAND			0xF4
#define STLINK_GET_CURRENT_MODE			0xF5

#define STLINK_DEV_DFU_MODE			0x00
#define STLINK_DEV_MASS_MODE			0x01
#define STLINK_DEV_DEBUG_MODE			0x02
#define STLINK_DEV_SWIM_MODE			0x03
#define STLINK_DEV_BOOTLOADER_MODE		0x04
#define STLINK_DEV_UNKNOWN_MODE			-1

#define STLINK_DFU_EXIT				0x07

#define STLINK_SWIM_ENTER			0x00
#define STLINK_SWIM_EXIT			0x01

#define STLINK_DEBUG_ENTER_JTAG			0x00
#define STLINK_DEBUG_GETSTATUS			0x01
#define STLINK_DEBUG_FORCEDEBUG			0x02
#define STLINK_DEBUG_APIV1_RESETSYS		0x03
#define STLINK_DEBUG_APIV1_READALLREGS		0x04
#define STLINK_DEBUG_APIV1_READREG		0x05
#define STLINK_DEBUG_APIV1_WRITEREG		0x06
#define STLINK_DEBUG_READMEM_32BIT		0x07
#define STLINK_DEBUG_WRITEMEM_32BIT		0x08
#define STLINK_DEBUG_RUNCORE			0x09
#define STLINK_DEBUG_STEPCORE			0x0a
#define STLINK_DEBUG_APIV1_SETFP		0x0b
#define STLINK_DEBUG_READMEM_8BIT		0x0c
#define STLINK_DEBUG_WRITEMEM_8BIT		0x0d
#define STLINK_DEBUG_APIV1_CLEARFP		0x0e
#define STLINK_DEBUG_APIV1_WRITEDEBUGREG	0x0f
#define STLINK_DEBUG_APIV1_SETWATCHPOINT	0x10

#define STLINK_DEBUG_ENTER_JTAG			0x00
#define STLINK_DEBUG_ENTER_SWD			0xa3

#define STLINK_DEBUG_APIV1_ENTER		0x20
#define STLINK_DEBUG_EXIT			0x21
#define STLINK_DEBUG_READCOREID			0x22

#define STLINK_DEBUG_APIV2_ENTER		0x30
#define STLINK_DEBUG_APIV2_READ_IDCODES		0x31
#define STLINK_DEBUG_APIV2_RESETSYS		0x32
#define STLINK_DEBUG_APIV2_READREG		0x33
#define STLINK_DEBUG_APIV2_WRITEREG		0x34
#define STLINK_DEBUG_APIV2_WRITEDEBUGREG	0x35
#define STLINK_DEBUG_APIV2_READDEBUGREG	0x36

#define STLINK_DEBUG_APIV2_READALLREGS		0x3A
#define STLINK_DEBUG_APIV2_GETLASTRWSTATUS	0x3B
#define STLINK_DEBUG_APIV2_DRIVE_NRST		0x3C

#define STLINK_DEBUG_APIV2_DRIVE_NRST_LOW	0x00
#define STLINK_DEBUG_APIV2_DRIVE_NRST_HIGH	0x01
#define STLINK_DEBUG_APIV2_DRIVE_NRST_PULSE	0x02

/** */
enum stlink_mode {
	STLINK_MODE_UNKNOWN = 0,
	STLINK_MODE_DFU,
	STLINK_MODE_MASS,
	STLINK_MODE_DEBUG_JTAG,
	STLINK_MODE_DEBUG_SWD,
	STLINK_MODE_DEBUG_SWIM
};

#define REQUEST_SENSE		0x03
#define REQUEST_SENSE_LENGTH	18

static void stlink_usb_init_buffer(void *handle, uint8_t direction, uint32_t size);

/** */
static int stlink_usb_xfer_v1_get_status(void *handle)
{
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	/* read status */
	memset(h->cmdbuf, 0, STLINK_SG_SIZE);

	if (jtag_libusb_bulk_read(h->fd, STLINK_RX_EP, (char *)h->cmdbuf,
				13, 1000) != 13)
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
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	if (jtag_libusb_bulk_write(h->fd, STLINK_TX_EP, (char *)h->cmdbuf, cmdsize,
				   1000) != cmdsize) {
		return ERROR_FAIL;
	}

	if (h->direction == STLINK_TX_EP && size) {
		if (jtag_libusb_bulk_write(h->fd, STLINK_TX_EP, (char *)buf,
					  size, 1000) != size) {
			LOG_DEBUG("bulk write failed");
			return ERROR_FAIL;
		}
	} else if (h->direction == STLINK_RX_EP && size) {
		if (jtag_libusb_bulk_read(h->fd, STLINK_RX_EP, (char *)buf,
					  size, 1000) != size) {
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
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle, STLINK_RX_EP, 16);

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
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

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
static void stlink_usb_xfer_v1_create_cmd(void *handle, uint8_t direction, uint32_t size)
{
	struct stlink_usb_handle_s *h;

	h = (struct stlink_usb_handle_s *)handle;

	/* fill the send buffer */
	strcpy((char *)h->cmdbuf, "USBC");
	h->cmdidx += 4;
	/* csw tag not used */
	h->cmdidx += 4;
	buf_set_u32(h->cmdbuf+h->cmdidx, 0, 32, size);
	h->cmdidx += 4;
	h->cmdbuf[h->cmdidx++] = (direction == STLINK_RX_EP ? ENDPOINT_IN : ENDPOINT_OUT);
	h->cmdbuf[h->cmdidx++] = 0; /* lun */
	h->cmdbuf[h->cmdidx++] = STLINK_CMD_SIZE_V1;
}

/** */
static void stlink_usb_init_buffer(void *handle, uint8_t direction, uint32_t size)
{
	struct stlink_usb_handle_s *h;

	h = (struct stlink_usb_handle_s *)handle;

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
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

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
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle, STLINK_RX_EP, 6);

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

	LOG_DEBUG("STLINK v%d JTAG v%d API v%d SWIM v%d VID 0x%04X PID 0x%04X",
		h->version.stlink,
		h->version.jtag,
		(h->version.jtag_api_max == STLINK_JTAG_API_V1) ? 1 : 2,
		h->version.swim,
		h->vid,
		h->pid);

	return ERROR_OK;
}

/** */
static int stlink_usb_current_mode(void *handle, uint8_t *mode)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle, STLINK_RX_EP, 2);

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
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	/* on api V2 we are able the read the latest command
	 * status
	 * TODO: we need the test on api V1 too
	 */
	if (h->jtag_api == STLINK_JTAG_API_V2)
		rx_size = 2;

	stlink_usb_init_buffer(handle, STLINK_RX_EP, rx_size);

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
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

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

/** */
static int stlink_usb_init_mode(void *handle)
{
	int res;
	uint8_t mode;
	enum stlink_mode emode;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

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

	LOG_DEBUG("MODE: 0x%02X", mode);

	/* set selected mode */
	switch (h->transport) {
		case STLINK_TRANSPORT_SWD:
			emode = STLINK_MODE_DEBUG_SWD;
			break;
		case STLINK_TRANSPORT_JTAG:
			emode = STLINK_MODE_DEBUG_JTAG;
			break;
		case STLINK_TRANSPORT_SWIM:
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
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle, STLINK_RX_EP, 4);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_READCOREID;

	res = stlink_usb_xfer(handle, h->databuf, 4);

	if (res != ERROR_OK)
		return res;

	*idcode = le_to_h_u32(h->databuf);

	LOG_DEBUG("IDCODE: 0x%08X", *idcode);

	return ERROR_OK;
}

static int stlink_usb_v2_read_debug_reg(void *handle, uint32_t addr, uint32_t *val)
{
	struct stlink_usb_handle_s *h;
	int res;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle, STLINK_RX_EP, 8);

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
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle, STLINK_RX_EP, 2);

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

	return TARGET_RUNNING;
}

/** */
static enum target_state stlink_usb_state(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	if (h->jtag_api == STLINK_JTAG_API_V2)
		return stlink_usb_v2_get_status(handle);

	stlink_usb_init_buffer(handle, STLINK_RX_EP, 2);

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
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle, STLINK_RX_EP, 2);

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
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	if (h->jtag_api == STLINK_JTAG_API_V1)
		return ERROR_COMMAND_NOTFOUND;

	stlink_usb_init_buffer(handle, STLINK_RX_EP, 2);

	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_COMMAND;
	h->cmdbuf[h->cmdidx++] = STLINK_DEBUG_APIV2_DRIVE_NRST;
	h->cmdbuf[h->cmdidx++] = srst;

	res = stlink_usb_xfer(handle, h->databuf, 2);

	if (res != ERROR_OK)
		return res;

	return h->databuf[0] == STLINK_DEBUG_ERR_OK ? ERROR_OK : ERROR_FAIL;
}

/** */
static int stlink_usb_run(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	if (h->jtag_api == STLINK_JTAG_API_V2)
		return stlink_usb_write_debug_reg(handle, DCB_DHCSR, DBGKEY|C_DEBUGEN);

	stlink_usb_init_buffer(handle, STLINK_RX_EP, 2);

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
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	if (h->jtag_api == STLINK_JTAG_API_V2)
		return stlink_usb_write_debug_reg(handle, DCB_DHCSR, DBGKEY|C_HALT|C_DEBUGEN);

	stlink_usb_init_buffer(handle, STLINK_RX_EP, 2);

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
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	if (h->jtag_api == STLINK_JTAG_API_V2) {
		/* TODO: this emulates the v1 api, it should really use a similar auto mask isr
		 * that the cortex-m3 currently does. */
		stlink_usb_write_debug_reg(handle, DCB_DHCSR, DBGKEY|C_HALT|C_MASKINTS|C_DEBUGEN);
		stlink_usb_write_debug_reg(handle, DCB_DHCSR, DBGKEY|C_STEP|C_MASKINTS|C_DEBUGEN);
		return stlink_usb_write_debug_reg(handle, DCB_DHCSR, DBGKEY|C_HALT|C_DEBUGEN);
	}

	stlink_usb_init_buffer(handle, STLINK_RX_EP, 2);

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
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle, STLINK_RX_EP, 84);

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
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle, STLINK_RX_EP, h->jtag_api == STLINK_JTAG_API_V1 ? 4 : 8);

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
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle, STLINK_RX_EP, 2);

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
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	if (h->jtag_api == STLINK_JTAG_API_V1)
		return ERROR_OK;

	stlink_usb_init_buffer(handle, STLINK_RX_EP, 2);

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
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle, STLINK_RX_EP, read_len);

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
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle, STLINK_TX_EP, len);

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
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	len *= 4;

	stlink_usb_init_buffer(handle, STLINK_RX_EP, len);

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
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	len *= 4;

	stlink_usb_init_buffer(handle, STLINK_TX_EP, len);

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

/** */
static int stlink_usb_open(struct stlink_interface_param_s *param, void **fd)
{
	int err;
	struct stlink_usb_handle_s *h;
	enum stlink_jtag_api_version api;

	LOG_DEBUG("stlink_usb_open");

	h = malloc(sizeof(struct stlink_usb_handle_s));

	if (h == 0) {
		LOG_DEBUG("malloc failed");
		return ERROR_FAIL;
	}

	h->transport = param->transport;

	const uint16_t vids[] = { param->vid, 0 };
	const uint16_t pids[] = { param->pid, 0 };

	LOG_DEBUG("transport: %d vid: 0x%04x pid: 0x%04x", param->transport,
		param->vid, param->pid);

	if (jtag_libusb_open(vids, pids, &h->fd) != ERROR_OK) {
		LOG_ERROR("open failed");
		return ERROR_FAIL;
	}

	jtag_libusb_set_configuration(h->fd, 0);

	if (jtag_libusb_claim_interface(h->fd, 0) != ERROR_OK) {
		LOG_DEBUG("claim interface failed");
		return ERROR_FAIL;
	}

	/* wrap version for first read */
	switch (param->pid) {
		case 0x3744:
			h->version.stlink = 1;
			break;
		default:
			h->version.stlink = 2;
			break;
	}

	/* get the device version */
	err = stlink_usb_version(h);

	if (err != ERROR_OK) {
		LOG_ERROR("read version failed");
		jtag_libusb_close(h->fd);
		free(h);
		return err;
	}

	/* compare usb vid/pid */
	if ((param->vid != h->vid) || (param->pid != h->pid))
		LOG_INFO("vid/pid are not identical: 0x%04X/0x%04X 0x%04X/0x%04X",
			param->vid, param->pid,
			h->vid, h->pid);

	/* check if mode is supported */
	err = ERROR_OK;

	switch (h->transport) {
		case STLINK_TRANSPORT_SWD:
		case STLINK_TRANSPORT_JTAG:
			if (h->version.jtag == 0)
				err = ERROR_FAIL;
			break;
		case STLINK_TRANSPORT_SWIM:
			if (h->version.swim == 0)
				err = ERROR_FAIL;
			break;
		default:
			err = ERROR_FAIL;
			break;
	}

	if (err != ERROR_OK) {
		LOG_ERROR("mode (transport) not supported by device");
		jtag_libusb_close(h->fd);
		free(h);
		return err;
	}

	api = h->version.jtag_api_max;

	/* check that user has not requested certain api version
	 * and if they have check it is supported */
	if ((param->api != 0) && (param->api <= h->version.jtag_api_max)) {
		api = param->api;
		LOG_INFO("using stlink api v%d", api);
	}

	/* set the used jtag api, this will default to the newest supported version */
	h->jtag_api = api;

	/* initialize the debug hardware */
	err = stlink_usb_init_mode(h);

	if (err != ERROR_OK) {
		LOG_ERROR("init mode failed");
		jtag_libusb_close(h->fd);
		free(h);
		return err;
	}

	*fd = h;

	return ERROR_OK;
}

/** */
static int stlink_usb_close(void *fd)
{
	return ERROR_OK;
}

/** */
struct stlink_layout_api_s stlink_usb_layout_api = {
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
	.read_mem8 = stlink_usb_read_mem8,
	/** */
	.write_mem8 = stlink_usb_write_mem8,
	/** */
	.read_mem32 = stlink_usb_read_mem32,
	/** */
	.write_mem32 = stlink_usb_write_mem32,
	/** */
	.write_debug_reg = stlink_usb_write_debug_reg
};
