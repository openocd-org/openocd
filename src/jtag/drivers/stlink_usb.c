/***************************************************************************
 *   Copyright (C) 2011 by Mathias Kuester                                 *
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
};

#define STLINK_OK			0x80
#define STLINK_FALSE			0x81
#define STLINK_CORE_RUNNING		0x80
#define STLINK_CORE_HALTED		0x81
#define STLINK_CORE_STAT_UNKNOWN	-1

#define STLINK_GET_VERSION		0xf1
#define STLINK_GET_CURRENT_MODE		0xf5

#define STLINK_DEBUG_COMMAND		0xF2
#define STLINK_DFU_COMMAND		0xF3
#define STLINK_DFU_EXIT			0x07

#define STLINK_DEV_DFU_MODE		0x00
#define STLINK_DEV_MASS_MODE		0x01
#define STLINK_DEV_DEBUG_MODE		0x02
#define STLINK_DEV_UNKNOWN_MODE		-1

#define STLINK_DEBUG_ENTER		0x20
#define STLINK_DEBUG_EXIT		0x21
#define STLINK_DEBUG_READCOREID		0x22

#define STLINK_DEBUG_GETSTATUS		0x01
#define STLINK_DEBUG_FORCEDEBUG		0x02
#define STLINK_DEBUG_RESETSYS		0x03
#define STLINK_DEBUG_READALLREGS	0x04
#define STLINK_DEBUG_READREG		0x05
#define STLINK_DEBUG_WRITEREG		0x06
#define STLINK_DEBUG_READMEM_32BIT	0x07
#define STLINK_DEBUG_WRITEMEM_32BIT	0x08
#define STLINK_DEBUG_RUNCORE		0x09
#define STLINK_DEBUG_STEPCORE		0x0a
#define STLINK_DEBUG_SETFP		0x0b
#define STLINK_DEBUG_WRITEMEM_8BIT	0x0d
#define STLINK_DEBUG_CLEARFP		0x0e
#define STLINK_DEBUG_WRITEDEBUGREG	0x0f
#define STLINK_DEBUG_ENTER_SWD		0xa3
#define STLINK_DEBUG_ENTER_JTAG		0x00

#define STLINK_SWD_ENTER		0x30
#define STLINK_SWD_READCOREID		0x32

/** */
int stlink_usb_recv(void *handle, uint8_t *txbuf, int txsize, uint8_t *rxbuf,
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
void stlink_usb_init_buffer(void *handle)
{
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	memset(h->txbuf, 0, STLINK_CMD_SIZE);
}

/** */
int stlink_usb_version(void *handle)
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

	LOG_DEBUG("STLINK v%d", (v >> 12) & 0x0f);
	LOG_DEBUG("JTAG   v%d", (v >> 6) & 0x3f);
	LOG_DEBUG("SWIM   v%d", v & 0x3f);
	LOG_DEBUG("VID    %04X", buf_get_u32(h->rxbuf, 16, 16));
	LOG_DEBUG("PID    %04X", buf_get_u32(h->rxbuf, 32, 16));

	return ERROR_OK;
}

/** */
int stlink_usb_current_mode(void *handle, uint8_t *mode)
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
int stlink_usb_dfu_mode_leave(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle);

	h->txbuf[0] = STLINK_DFU_COMMAND;
	h->txbuf[1] = STLINK_DFU_EXIT;

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, 0, 0);

	if (res != ERROR_OK)
		return res;

	return ERROR_OK;
}

/** */
int stlink_usb_swd_mode_enter(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle);

	h->txbuf[0] = STLINK_DEBUG_COMMAND;
	h->txbuf[1] = STLINK_DEBUG_ENTER;
	h->txbuf[2] = STLINK_DEBUG_ENTER_SWD;

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, 0, 0);
	if (res != ERROR_OK)
		return res;

	return ERROR_OK;
}

/** */
int stlink_usb_debug_mode_leave(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle);

	h->txbuf[0] = STLINK_DEBUG_COMMAND;
	h->txbuf[1] = STLINK_DEBUG_EXIT;

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, 0, 0);
	if (res != ERROR_OK)
		return res;

	return ERROR_OK;
}

/** */
int stlink_usb_init_mode(void *handle)
{
	int res;
	uint8_t mode;

	assert(handle != NULL);

	res = stlink_usb_current_mode(handle, &mode);

	if (res != ERROR_OK)
		return res;

	LOG_DEBUG("MODE: %02X", mode);

	if (mode == STLINK_DEV_DFU_MODE) {
		res = stlink_usb_dfu_mode_leave(handle);

		if (res != ERROR_OK)
			return res;
	}

	res = stlink_usb_current_mode(handle, &mode);

	if (res != ERROR_OK)
		return res;

	LOG_DEBUG("MODE: %02X", mode);

	if (mode != STLINK_DEV_DEBUG_MODE) {
		res = stlink_usb_swd_mode_enter(handle);

		if (res != ERROR_OK)
			return res;
	}

	res = stlink_usb_current_mode(handle, &mode);

	if (res != ERROR_OK)
		return res;

	LOG_DEBUG("MODE: %02X", mode);

	return ERROR_OK;
}

/** */
int stlink_usb_idcode(void *handle, uint32_t *idcode)
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
enum target_state stlink_usb_state(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

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
int stlink_usb_reset(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle);

	h->txbuf[0] = STLINK_DEBUG_COMMAND;
	h->txbuf[1] = STLINK_DEBUG_RESETSYS;

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, h->rxbuf, 2);

	if (res != ERROR_OK)
		return res;

	LOG_DEBUG("RESET: %08X", h->rxbuf[0]);

	return ERROR_OK;
}

/** */
int stlink_usb_run(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle);

	h->txbuf[0] = STLINK_DEBUG_COMMAND;
	h->txbuf[1] = STLINK_DEBUG_RUNCORE;

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, h->rxbuf, 2);

	if (res != ERROR_OK)
		return res;

	return ERROR_OK;
}

/** */
int stlink_usb_halt(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle);

	h->txbuf[0] = STLINK_DEBUG_COMMAND;
	h->txbuf[1] = STLINK_DEBUG_FORCEDEBUG;

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, h->rxbuf, 2);

	if (res != ERROR_OK)
		return res;

	return ERROR_OK;
}

/** */
int stlink_usb_step(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle);

	h->txbuf[0] = STLINK_DEBUG_COMMAND;
	h->txbuf[1] = STLINK_DEBUG_STEPCORE;

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, h->rxbuf, 2);

	if (res != ERROR_OK)
		return res;

	return ERROR_OK;
}

/** */
int stlink_usb_read_regs(void *handle)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle);

	h->txbuf[0] = STLINK_DEBUG_COMMAND;
	h->txbuf[1] = STLINK_DEBUG_READALLREGS;

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, h->rxbuf, 84);

	if (res != ERROR_OK)
		return res;

	return ERROR_OK;
}

/** */
int stlink_usb_read_reg(void *handle, int num, uint32_t *val)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle);

	h->txbuf[0] = STLINK_DEBUG_COMMAND;
	h->txbuf[1] = STLINK_DEBUG_READREG;
	h->txbuf[2] = num;

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, h->rxbuf, 4);

	if (res != ERROR_OK)
		return res;

	*val = le_to_h_u32(h->rxbuf);

	return ERROR_OK;
}

/** */
int stlink_usb_write_reg(void *handle, int num, uint32_t val)
{
	int res;
	struct stlink_usb_handle_s *h;

	assert(handle != NULL);

	h = (struct stlink_usb_handle_s *)handle;

	stlink_usb_init_buffer(handle);

	h->txbuf[0] = STLINK_DEBUG_COMMAND;
	h->txbuf[1] = STLINK_DEBUG_WRITEREG;
	h->txbuf[2] = num;
	h_u32_to_le(h->txbuf + 3, val);

	res = stlink_usb_recv(handle, h->txbuf, STLINK_CMD_SIZE, h->rxbuf, 2);

	if (res != ERROR_OK)
		return res;

	return ERROR_OK;
}

/** */
int stlink_usb_read_mem32(void *handle, uint32_t addr, uint16_t len,
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
int stlink_usb_write_mem32(void *handle, uint32_t addr, uint16_t len,
			   uint32_t *buffer)
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

	memcpy(buffer, h->rxbuf, len);

	return ERROR_OK;
}

/** */
int stlink_usb_open(struct stlink_interface_param_s *param, void **fd)
{
	struct stlink_usb_handle_s *h;

	LOG_DEBUG("stlink_usb_open");

	h = malloc(sizeof(struct stlink_usb_handle_s));

	if (h == 0) {
		LOG_DEBUG("stlink_open_usb: malloc failed");
		return ERROR_FAIL;
	}

	const uint16_t vids[] = { param->vid, 0 };
	const uint16_t pids[] = { param->pid, 0 };

	LOG_DEBUG("stlink_open_usb: vid: %04x pid: %04x", param->vid,
		  param->pid);

	if (jtag_libusb_open(vids, pids, &h->fd) != ERROR_OK) {
		LOG_DEBUG("stlink_open_usb: open failed");
		return ERROR_FAIL;
	}

	jtag_libusb_set_configuration(h->fd, 0);

	if (jtag_libusb_claim_interface(h->fd, 0) != ERROR_OK) {
		LOG_DEBUG("stlink_open_usb: claim failed");
		return ERROR_FAIL;
	}

	stlink_usb_init_mode(h);

	stlink_usb_version(h);

	*fd = h;

	return ERROR_OK;
}

/** */
struct stlink_layout_api_s stlink_layout_api = {
	/** */
	.open = stlink_usb_open,
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
	.read_mem32 = stlink_usb_read_mem32,
	/** */
	.write_mem32 = stlink_usb_write_mem32,
};
