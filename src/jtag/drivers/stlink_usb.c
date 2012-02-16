/***************************************************************************
 *   Copyright (C) 2011-2012 by Mathias Kuester                            *
 *   Mathias Kuester <kesmtp@freenet.de>                                   *
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

#include "libusb_common.h"

#define ENDPOINT_IN	0x80
#define ENDPOINT_OUT	0x00

#define STLINK_RX_EP	(1|ENDPOINT_IN)
#define STLINK_TX_EP	(2|ENDPOINT_OUT)
#define STLINK_CMD_SIZE	(16)
#define STLINK_TX_SIZE	(4*128)
#define STLINK_RX_SIZE	(4*128)

enum stlink_jtag_api_version {
	STLINK_JTAG_API_V1 = 0,
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
	uint8_t txbuf[STLINK_TX_SIZE];
	/** */
	uint8_t rxbuf[STLINK_RX_SIZE];
	/** */
	enum stlink_transports transport;
	/** */
	struct stlink_usb_version version;
	/** */
	uint16_t vid;
	/** */
	uint16_t pid;
	/** */
	uint32_t sg_tag;
	/** this is the currently used jtag api */
	enum stlink_jtag_api_version jtag_api;
};

#define STLINK_OK				0x80
#define STLINK_FALSE				0x81
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

#define STLINK_DEBUG_APIV2_RESETSYS		0x32
#define STLINK_DEBUG_APIV2_READREG		0x33
#define STLINK_DEBUG_APIV2_WRITEREG		0x34

#define STLINK_DEBUG_APIV2_READALLREGS		0x3A
/** */
enum stlink_mode {
	STLINK_MODE_UNKNOWN = 0,
	STLINK_MODE_DFU,
	STLINK_MODE_MASS,
	STLINK_MODE_DEBUG_JTAG,
	STLINK_MODE_DEBUG_SWD,
	STLINK_MODE_DEBUG_SWIM
};

/** */
static void stlink_usb_recv_v1_create_cmd(char *b, int s, uint32_t tag, uint32_t rxsize,
			uint8_t flag, uint8_t lun, uint8_t length)
{
	int i = 0;

	memset(b, 0x00, s);

	/* fill the send buffer */
	strcpy(b, "USBC");
	i += 4;

	buf_set_u32(b+i, 0, 32, tag);
	i += 4;
	buf_set_u32(b+i, 0, 32, rxsize);
	i += 4;
	b[i++] = flag;
	b[i++] = lun;
	b[i++] = length;
}

/** */
static int stlink_usb_recv_v1_mass_storage_cmd(void *handle, const uint8_t *txbuf, int txsize, uint8_t *rxbuf,
		    int rxsize)
{
	char sg_buffer[31];
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;
	h->sg_tag = (h->sg_tag + 1) & 1;

	stlink_usb_recv_v1_create_cmd(sg_buffer, 31, h->sg_tag, rxsize, STLINK_TX_EP, 0x00, txsize);

	memcpy(sg_buffer+15, txbuf, 10);

	if (jtag_libusb_bulk_write(h->fd, STLINK_TX_EP, (char *)sg_buffer, 31,
				   1000) != 31) {
		printf("send failed\n");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

#define REQUEST_SENSE		0x03
#define REQUEST_SENSE_LENGTH	18

/** */
static int stlink_usb_recv_v1_get_status(void *handle, char *sg_buffer, int len)
{
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	/* read status */
	memset(sg_buffer, 0x00, len);

	if (jtag_libusb_bulk_read(h->fd, STLINK_RX_EP, (char *)sg_buffer,
				len, 1000) != len)
		return ERROR_FAIL;

	uint32_t t1, t2;

	t1 = buf_get_u32(sg_buffer+0, 0, 32);
	t2 = buf_get_u32(sg_buffer+4, 0, 32);

	/* check for USBS */
	if (t1 != 0x53425355)
		return ERROR_FAIL;

	return ERROR_OK;
}

/** */
static int stlink_usb_recv_v1_get_sense(void *handle)
{
	struct stlink_usb_handle_s *h;
	char cdb[16];
	char sg_buffer[31];

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;
	h->sg_tag = (h->sg_tag + 1) & 1;

	cdb[0] = REQUEST_SENSE;
	cdb[4] = REQUEST_SENSE_LENGTH;

	stlink_usb_recv_v1_create_cmd(sg_buffer, 31, h->sg_tag, REQUEST_SENSE_LENGTH, STLINK_TX_EP,
			0x00, 16);

	memcpy(sg_buffer+15, cdb, 16);

	if (jtag_libusb_bulk_write(h->fd, STLINK_TX_EP, (char *)sg_buffer, 16,
				   1000) != 16)
		return ERROR_FAIL;

	if (jtag_libusb_bulk_read(h->fd, STLINK_RX_EP, (char *)cdb,
				  16, 1000) != 16)
		return ERROR_FAIL;

	if (stlink_usb_recv_v1_get_status(handle, sg_buffer, 13) != ERROR_OK)
		return ERROR_FAIL;
	/* check for sense */
	if (sg_buffer[12] != 0)
		return ERROR_FAIL;

	/* if (sense[0] != 0x70 && sense[0] != 0x71) */

	return ERROR_OK;
}

/** */
static int stlink_usb_recv_v1(void *handle, const uint8_t *txbuf, int txsize, uint8_t *rxbuf,
		    int rxsize)
{
	int err;
	char sg_buffer[31];
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	err = stlink_usb_recv_v1_mass_storage_cmd(handle, txbuf, txsize, rxbuf, rxsize);

	if (err != ERROR_OK)
		return err;

	if (rxsize && rxbuf) {
		if (jtag_libusb_bulk_read(h->fd, STLINK_RX_EP, (char *)rxbuf,
					  rxsize, 1000) != rxsize) {
			LOG_DEBUG("jtag_libusb_bulk_read");
			return ERROR_FAIL;
		}
	}

	if (stlink_usb_recv_v1_get_status(handle, sg_buffer, 13) != ERROR_OK)
		return ERROR_FAIL;
	/* check for sense */
	if (sg_buffer[12] == 1) {
		LOG_DEBUG("get sense");
		err = stlink_usb_recv_v1_get_sense(handle);
	}
	return err;
}

/** */
static int stlink_usb_recv_v2(void *handle, const uint8_t *txbuf, int txsize, uint8_t *rxbuf,
		    int rxsize)
{
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	if (jtag_libusb_bulk_write(h->fd, STLINK_TX_EP, (char *)txbuf, txsize,
				   1000) != txsize) {
		return ERROR_FAIL;
	}
	if (rxsize && rxbuf) {
		if (jtag_libusb_bulk_read(h->fd, STLINK_RX_EP, (char *)rxbuf,
					  rxsize, 1000) != rxsize) {
			return ERROR_FAIL;
		}
	}
	return ERROR_OK;
}

/** */
static int stlink_usb_recv(void *handle, const uint8_t *txbuf, int txsize, uint8_t *rxbuf,
		    int rxsize)
{
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	if (h->version.stlink == 1) {
		return stlink_usb_recv_v1(handle, txbuf, txsize, rxbuf, rxsize);
	} else {
		if (txsize < STLINK_CMD_SIZE)
			txsize = STLINK_CMD_SIZE;
		return stlink_usb_recv_v2(handle, txbuf, txsize, rxbuf, rxsize);
	}
}

/** */
static void stlink_usb_init_buffer(void *handle)
{
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	memset(h->txbuf, 0, STLINK_CMD_SIZE);
}

/** */
static int stlink_usb_version(void *handle)
{
	int res;
	uint16_t v;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle);

	h->txbuf[0] = STLINK_GET_VERSION;

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, h->rxbuf, 6);

	if (res != ERROR_OK)
		return res;

	v = (h->rxbuf[0] << 8) | h->rxbuf[1];

	h->version.stlink = (v >> 12) & 0x0f;
	h->version.jtag = (v >> 6) & 0x3f;
	h->version.swim = v & 0x3f;
	h->vid = buf_get_u32(h->rxbuf, 16, 16);
	h->pid = buf_get_u32(h->rxbuf, 32, 16);

	/* set the supported jtag api version
	 * V1 doesn't support API V2 at all
	 * V2 support API V2 since JTAG V13
	 */
	if ((h->version.stlink == 2) && (h->version.jtag > 12))
		h->version.jtag_api_max = STLINK_JTAG_API_V2;
	else
		h->version.jtag_api_max = STLINK_JTAG_API_V1;

	LOG_DEBUG("STLINK v%d JTAG v%d API v%d SWIM v%d VID %04X PID %04X",
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

	stlink_usb_init_buffer(handle);

	h->txbuf[0] = STLINK_GET_CURRENT_MODE;

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, h->rxbuf, 2);

	if (res != ERROR_OK)
		return res;

	*mode = h->rxbuf[0];

	return ERROR_OK;
}

/** */
static int stlink_usb_mode_enter(void *handle, enum stlink_mode type)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle);

	switch (type) {
		case STLINK_MODE_DEBUG_JTAG:
			h->txbuf[0] = STLINK_DEBUG_COMMAND;
			if (h->jtag_api == STLINK_JTAG_API_V1)
				h->txbuf[1] = STLINK_DEBUG_APIV1_ENTER;
			else
				h->txbuf[1] = STLINK_DEBUG_APIV2_ENTER;
			h->txbuf[2] = STLINK_DEBUG_ENTER_JTAG;
			break;
		case STLINK_MODE_DEBUG_SWD:
			h->txbuf[0] = STLINK_DEBUG_COMMAND;
			if (h->jtag_api == STLINK_JTAG_API_V1)
				h->txbuf[1] = STLINK_DEBUG_APIV1_ENTER;
			else
				h->txbuf[1] = STLINK_DEBUG_APIV2_ENTER;
			h->txbuf[2] = STLINK_DEBUG_ENTER_SWD;
			break;
		case STLINK_MODE_DEBUG_SWIM:
			h->txbuf[0] = STLINK_SWIM_COMMAND;
			h->txbuf[1] = STLINK_SWIM_ENTER;
			break;
		case STLINK_MODE_DFU:
		case STLINK_MODE_MASS:
		default:
			return ERROR_FAIL;
	}

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, 0, 0);
	if (res != ERROR_OK)
		return res;

	return ERROR_OK;
}

/** */
static int stlink_usb_mode_leave(void *handle, enum stlink_mode type)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle);

	switch (type) {
		case STLINK_MODE_DEBUG_JTAG:
		case STLINK_MODE_DEBUG_SWD:
			h->txbuf[0] = STLINK_DEBUG_COMMAND;
			h->txbuf[1] = STLINK_DEBUG_EXIT;
			break;
		case STLINK_MODE_DEBUG_SWIM:
			h->txbuf[0] = STLINK_SWIM_COMMAND;
			h->txbuf[1] = STLINK_SWIM_EXIT;
			break;
		case STLINK_MODE_DFU:
			h->txbuf[0] = STLINK_DFU_COMMAND;
			h->txbuf[1] = STLINK_DFU_EXIT;
			break;
		case STLINK_MODE_MASS:
		default:
			return ERROR_FAIL;
	}

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, 0, 0);
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

	LOG_DEBUG("MODE: %02X", mode);

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

	LOG_DEBUG("MODE: %02X", mode);

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

	LOG_DEBUG("MODE: %02X", mode);

	return ERROR_OK;
}

/** */
static int stlink_usb_idcode(void *handle, uint32_t *idcode)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle);

	h->txbuf[0] = STLINK_DEBUG_COMMAND;
	h->txbuf[1] = STLINK_DEBUG_READCOREID;

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, h->rxbuf, 4);

	if (res != ERROR_OK)
		return res;

	*idcode = le_to_h_u32(h->rxbuf);

	LOG_DEBUG("IDCODE: %08X", *idcode);

	return ERROR_OK;
}

/** */
static enum target_state stlink_usb_state(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	if (h->jtag_api == STLINK_JTAG_API_V2)
		return TARGET_UNKNOWN;

	stlink_usb_init_buffer(handle);

	h->txbuf[0] = STLINK_DEBUG_COMMAND;
	h->txbuf[1] = STLINK_DEBUG_GETSTATUS;

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, h->rxbuf, 2);

	if (res != ERROR_OK)
		return TARGET_UNKNOWN;

	if (h->rxbuf[0] == STLINK_CORE_RUNNING)
		return TARGET_RUNNING;
	if (h->rxbuf[0] == STLINK_CORE_HALTED)
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

	stlink_usb_init_buffer(handle);

	h->txbuf[0] = STLINK_DEBUG_COMMAND;

	if (h->jtag_api == STLINK_JTAG_API_V1)
		h->txbuf[1] = STLINK_DEBUG_APIV1_RESETSYS;
	else
		h->txbuf[1] = STLINK_DEBUG_APIV2_RESETSYS;

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, h->rxbuf, 2);

	if (res != ERROR_OK)
		return res;

	LOG_DEBUG("RESET: %08X", h->rxbuf[0]);

	return ERROR_OK;
}

/** */
static int stlink_usb_run(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	if (h->jtag_api == STLINK_JTAG_API_V2)
		return ERROR_FAIL;

	stlink_usb_init_buffer(handle);

	h->txbuf[0] = STLINK_DEBUG_COMMAND;
	h->txbuf[1] = STLINK_DEBUG_RUNCORE;

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, h->rxbuf, 2);

	if (res != ERROR_OK)
		return res;

	return ERROR_OK;
}

/** */
static int stlink_usb_halt(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	if (h->jtag_api == STLINK_JTAG_API_V2)
		return ERROR_FAIL;

	stlink_usb_init_buffer(handle);

	h->txbuf[0] = STLINK_DEBUG_COMMAND;
	h->txbuf[1] = STLINK_DEBUG_FORCEDEBUG;

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, h->rxbuf, 2);

	if (res != ERROR_OK)
		return res;

	return ERROR_OK;
}

/** */
static int stlink_usb_step(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	if (h->jtag_api == STLINK_JTAG_API_V2)
		return ERROR_FAIL;

	stlink_usb_init_buffer(handle);

	h->txbuf[0] = STLINK_DEBUG_COMMAND;
	h->txbuf[1] = STLINK_DEBUG_STEPCORE;

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, h->rxbuf, 2);

	if (res != ERROR_OK)
		return res;

	return ERROR_OK;
}

/** */
static int stlink_usb_read_regs(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle);

	h->txbuf[0] = STLINK_DEBUG_COMMAND;
	if (h->jtag_api == STLINK_JTAG_API_V1)
		h->txbuf[1] = STLINK_DEBUG_APIV1_READALLREGS;
	else
		h->txbuf[1] = STLINK_DEBUG_APIV2_READALLREGS;

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, h->rxbuf, 84);

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

	stlink_usb_init_buffer(handle);

	h->txbuf[0] = STLINK_DEBUG_COMMAND;
	if (h->jtag_api == STLINK_JTAG_API_V1)
		h->txbuf[1] = STLINK_DEBUG_APIV1_READREG;
	else
		h->txbuf[1] = STLINK_DEBUG_APIV2_READREG;
	h->txbuf[2] = num;

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, h->rxbuf, 4);

	if (res != ERROR_OK)
		return res;

	*val = le_to_h_u32(h->rxbuf);

	return ERROR_OK;
}

/** */
static int stlink_usb_write_reg(void *handle, int num, uint32_t val)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle);

	h->txbuf[0] = STLINK_DEBUG_COMMAND;
	if (h->jtag_api == STLINK_JTAG_API_V1)
		h->txbuf[1] = STLINK_DEBUG_APIV1_WRITEREG;
	else
		h->txbuf[1] = STLINK_DEBUG_APIV2_WRITEREG;
	h->txbuf[2] = num;
	h_u32_to_le(h->txbuf + 3, val);

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, h->rxbuf, 2);

	if (res != ERROR_OK)
		return res;

	return ERROR_OK;
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

	stlink_usb_init_buffer(handle);

	h->txbuf[0] = STLINK_DEBUG_COMMAND;
	h->txbuf[1] = STLINK_DEBUG_READMEM_8BIT;
	h_u32_to_le(h->txbuf + 2, addr);
	h_u16_to_le(h->txbuf + 2 + 4, len);

	/* we need to fix read length for single bytes */
	if (read_len == 1)
		read_len++;

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, h->rxbuf, read_len);

	if (res != ERROR_OK)
		return res;

	memcpy(buffer, h->rxbuf, len);

	return ERROR_OK;
}

/** */
static int stlink_usb_write_mem8(void *handle, uint32_t addr, uint16_t len,
			   const uint8_t *buffer)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle);

	h->txbuf[0] = STLINK_DEBUG_COMMAND;
	h->txbuf[1] = STLINK_DEBUG_WRITEMEM_8BIT;
	h_u32_to_le(h->txbuf + 2, addr);
	h_u16_to_le(h->txbuf + 2 + 4, len);

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, 0, 0);

	if (res != ERROR_OK)
		return res;

	res = stlink_usb_recv(handle, (uint8_t *) buffer, len, 0, 0);

	if (res != ERROR_OK)
		return res;

	return ERROR_OK;
}

/** */
static int stlink_usb_read_mem32(void *handle, uint32_t addr, uint16_t len,
			  uint32_t *buffer)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle);

	len *= 4;

	h->txbuf[0] = STLINK_DEBUG_COMMAND;
	h->txbuf[1] = STLINK_DEBUG_READMEM_32BIT;
	h_u32_to_le(h->txbuf + 2, addr);
	h_u16_to_le(h->txbuf + 2 + 4, len);

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, h->rxbuf, len);

	if (res != ERROR_OK)
		return res;

	memcpy(buffer, h->rxbuf, len);

	return ERROR_OK;
}

/** */
static int stlink_usb_write_mem32(void *handle, uint32_t addr, uint16_t len,
			   const uint32_t *buffer)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle);

	len *= 4;

	h->txbuf[0] = STLINK_DEBUG_COMMAND;
	h->txbuf[1] = STLINK_DEBUG_WRITEMEM_32BIT;
	h_u32_to_le(h->txbuf + 2, addr);
	h_u16_to_le(h->txbuf + 2 + 4, len);

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, 0, 0);

	if (res != ERROR_OK)
		return res;

	res = stlink_usb_recv(handle, (uint8_t *) buffer, len, 0, 0);

	if (res != ERROR_OK)
		return res;

	return ERROR_OK;
}

/** */
static int stlink_usb_open(struct stlink_interface_param_s *param, void **fd)
{
	int err;
	struct stlink_usb_handle_s *h;

	LOG_DEBUG("stlink_usb_open");

	h = malloc(sizeof(struct stlink_usb_handle_s));

	if (h == 0) {
		LOG_DEBUG("malloc failed");
		return ERROR_FAIL;
	}

	h->transport = param->transport;

	const uint16_t vids[] = { param->vid, 0 };
	const uint16_t pids[] = { param->pid, 0 };

	LOG_DEBUG("transport: %d vid: %04x pid: %04x", param->transport,
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
		LOG_INFO("vid/pid are not identical: %04X/%04X %04X/%04X",
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

	/* set the used jtag api */
	h->jtag_api = STLINK_JTAG_API_V1;

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
};
