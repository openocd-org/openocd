/***************************************************************************
 *   Copyright (C) 2013 by Andes Technology                                *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
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

#include <jtag/drivers/libusb_common.h>
#include <helper/log.h>
#include <helper/time_support.h>
#include <target/target.h>
#include <jtag/jtag.h>
#include <target/nds32_insn.h>
#include <target/nds32_reg.h>
#include "aice_usb.h"


/* Global USB buffers */
static uint8_t usb_in_buffer[AICE_IN_BUFFER_SIZE];
static uint8_t usb_out_buffer[AICE_OUT_BUFFER_SIZE];
static uint8_t current_target_id;
static uint32_t jtag_clock;
static struct aice_usb_handler_s aice_handler;
/* AICE max retry times. If AICE command timeout, retry it. */
static int aice_max_retry_times = 10;
/* Default endian is little endian. */
static enum aice_target_endian data_endian;


/***************************************************************************/
/* AICE commands' pack/unpack functions */
static void aice_pack_htda(uint8_t cmd_code, uint8_t extra_word_length,
		uint32_t address)
{
	usb_out_buffer[0] = cmd_code;
	usb_out_buffer[1] = extra_word_length;
	usb_out_buffer[2] = (uint8_t)(address & 0xFF);
}

static void aice_pack_htdc(uint8_t cmd_code, uint8_t extra_word_length,
		uint32_t address, uint32_t word, enum aice_target_endian access_endian)
{
	usb_out_buffer[0] = cmd_code;
	usb_out_buffer[1] = extra_word_length;
	usb_out_buffer[2] = (uint8_t)(address & 0xFF);
	if (access_endian == AICE_BIG_ENDIAN) {
		usb_out_buffer[6] = (uint8_t)((word >> 24) & 0xFF);
		usb_out_buffer[5] = (uint8_t)((word >> 16) & 0xFF);
		usb_out_buffer[4] = (uint8_t)((word >> 8) & 0xFF);
		usb_out_buffer[3] = (uint8_t)(word & 0xFF);
	} else {
		usb_out_buffer[3] = (uint8_t)((word >> 24) & 0xFF);
		usb_out_buffer[4] = (uint8_t)((word >> 16) & 0xFF);
		usb_out_buffer[5] = (uint8_t)((word >> 8) & 0xFF);
		usb_out_buffer[6] = (uint8_t)(word & 0xFF);
	}
}

static void aice_pack_htdma(uint8_t cmd_code, uint8_t target_id,
		uint8_t extra_word_length, uint32_t address)
{
	usb_out_buffer[0] = cmd_code;
	usb_out_buffer[1] = target_id;
	usb_out_buffer[2] = extra_word_length;
	usb_out_buffer[3] = (uint8_t)(address & 0xFF);
}

static void aice_pack_htdmb(uint8_t cmd_code, uint8_t target_id,
		uint8_t extra_word_length, uint32_t address)
{
	usb_out_buffer[0] = cmd_code;
	usb_out_buffer[1] = target_id;
	usb_out_buffer[2] = extra_word_length;
	usb_out_buffer[3] = 0;
	usb_out_buffer[4] = (uint8_t)((address >> 24) & 0xFF);
	usb_out_buffer[5] = (uint8_t)((address >> 16) & 0xFF);
	usb_out_buffer[6] = (uint8_t)((address >> 8) & 0xFF);
	usb_out_buffer[7] = (uint8_t)(address & 0xFF);
}

static void aice_pack_htdmc(uint8_t cmd_code, uint8_t target_id,
		uint8_t extra_word_length, uint32_t address, uint32_t word,
		enum aice_target_endian access_endian)
{
	usb_out_buffer[0] = cmd_code;
	usb_out_buffer[1] = target_id;
	usb_out_buffer[2] = extra_word_length;
	usb_out_buffer[3] = (uint8_t)(address & 0xFF);
	if (access_endian == AICE_BIG_ENDIAN) {
		usb_out_buffer[7] = (uint8_t)((word >> 24) & 0xFF);
		usb_out_buffer[6] = (uint8_t)((word >> 16) & 0xFF);
		usb_out_buffer[5] = (uint8_t)((word >> 8) & 0xFF);
		usb_out_buffer[4] = (uint8_t)(word & 0xFF);
	} else {
		usb_out_buffer[4] = (uint8_t)((word >> 24) & 0xFF);
		usb_out_buffer[5] = (uint8_t)((word >> 16) & 0xFF);
		usb_out_buffer[6] = (uint8_t)((word >> 8) & 0xFF);
		usb_out_buffer[7] = (uint8_t)(word & 0xFF);
	}
}

static void aice_pack_htdmc_multiple_data(uint8_t cmd_code, uint8_t target_id,
		uint8_t extra_word_length, uint32_t address, uint32_t *word,
		uint8_t num_of_words, enum aice_target_endian access_endian)
{
	usb_out_buffer[0] = cmd_code;
	usb_out_buffer[1] = target_id;
	usb_out_buffer[2] = extra_word_length;
	usb_out_buffer[3] = (uint8_t)(address & 0xFF);

	uint8_t i;
	for (i = 0 ; i < num_of_words ; i++, word++) {
		if (access_endian == AICE_BIG_ENDIAN) {
			usb_out_buffer[7 + i * 4] = (uint8_t)((*word >> 24) & 0xFF);
			usb_out_buffer[6 + i * 4] = (uint8_t)((*word >> 16) & 0xFF);
			usb_out_buffer[5 + i * 4] = (uint8_t)((*word >> 8) & 0xFF);
			usb_out_buffer[4 + i * 4] = (uint8_t)(*word & 0xFF);
		} else {
			usb_out_buffer[4 + i * 4] = (uint8_t)((*word >> 24) & 0xFF);
			usb_out_buffer[5 + i * 4] = (uint8_t)((*word >> 16) & 0xFF);
			usb_out_buffer[6 + i * 4] = (uint8_t)((*word >> 8) & 0xFF);
			usb_out_buffer[7 + i * 4] = (uint8_t)(*word & 0xFF);
		}
	}
}

static void aice_pack_htdmd(uint8_t cmd_code, uint8_t target_id,
		uint8_t extra_word_length, uint32_t address, uint32_t word,
		enum aice_target_endian access_endian)
{
	usb_out_buffer[0] = cmd_code;
	usb_out_buffer[1] = target_id;
	usb_out_buffer[2] = extra_word_length;
	usb_out_buffer[3] = 0;
	usb_out_buffer[4] = (uint8_t)((address >> 24) & 0xFF);
	usb_out_buffer[5] = (uint8_t)((address >> 16) & 0xFF);
	usb_out_buffer[6] = (uint8_t)((address >> 8) & 0xFF);
	usb_out_buffer[7] = (uint8_t)(address & 0xFF);
	if (access_endian == AICE_BIG_ENDIAN) {
		usb_out_buffer[11] = (uint8_t)((word >> 24) & 0xFF);
		usb_out_buffer[10] = (uint8_t)((word >> 16) & 0xFF);
		usb_out_buffer[9] = (uint8_t)((word >> 8) & 0xFF);
		usb_out_buffer[8] = (uint8_t)(word & 0xFF);
	} else {
		usb_out_buffer[8] = (uint8_t)((word >> 24) & 0xFF);
		usb_out_buffer[9] = (uint8_t)((word >> 16) & 0xFF);
		usb_out_buffer[10] = (uint8_t)((word >> 8) & 0xFF);
		usb_out_buffer[11] = (uint8_t)(word & 0xFF);
	}
}

static void aice_pack_htdmd_multiple_data(uint8_t cmd_code, uint8_t target_id,
		uint8_t extra_word_length, uint32_t address, const uint8_t *word,
		enum aice_target_endian access_endian)
{
	usb_out_buffer[0] = cmd_code;
	usb_out_buffer[1] = target_id;
	usb_out_buffer[2] = extra_word_length;
	usb_out_buffer[3] = 0;
	usb_out_buffer[4] = (uint8_t)((address >> 24) & 0xFF);
	usb_out_buffer[5] = (uint8_t)((address >> 16) & 0xFF);
	usb_out_buffer[6] = (uint8_t)((address >> 8) & 0xFF);
	usb_out_buffer[7] = (uint8_t)(address & 0xFF);

	uint32_t i;
	/* num_of_words may be over 0xFF, so use uint32_t */
	uint32_t num_of_words = extra_word_length + 1;

	for (i = 0 ; i < num_of_words ; i++, word += 4) {
		if (access_endian == AICE_BIG_ENDIAN) {
			usb_out_buffer[11 + i * 4] = word[3];
			usb_out_buffer[10 + i * 4] = word[2];
			usb_out_buffer[9 + i * 4] = word[1];
			usb_out_buffer[8 + i * 4] = word[0];
		} else {
			usb_out_buffer[8 + i * 4] = word[3];
			usb_out_buffer[9 + i * 4] = word[2];
			usb_out_buffer[10 + i * 4] = word[1];
			usb_out_buffer[11 + i * 4] = word[0];
		}
	}
}

static void aice_unpack_dtha(uint8_t *cmd_ack_code, uint8_t *extra_word_length,
		uint32_t *word, enum aice_target_endian access_endian)
{
	*cmd_ack_code = usb_in_buffer[0];
	*extra_word_length = usb_in_buffer[1];

	if (access_endian == AICE_BIG_ENDIAN) {
		*word = (usb_in_buffer[5] << 24) |
			(usb_in_buffer[4] << 16) |
			(usb_in_buffer[3] << 8) |
			(usb_in_buffer[2]);
	} else {
		*word = (usb_in_buffer[2] << 24) |
			(usb_in_buffer[3] << 16) |
			(usb_in_buffer[4] << 8) |
			(usb_in_buffer[5]);
	}
}

static void aice_unpack_dtha_multiple_data(uint8_t *cmd_ack_code,
		uint8_t *extra_word_length, uint32_t *word, uint8_t num_of_words,
		enum aice_target_endian access_endian)
{
	*cmd_ack_code = usb_in_buffer[0];
	*extra_word_length = usb_in_buffer[1];

	uint8_t i;
	for (i = 0 ; i < num_of_words ; i++, word++) {
		if (access_endian == AICE_BIG_ENDIAN) {
			*word = (usb_in_buffer[5 + i * 4] << 24) |
				(usb_in_buffer[4 + i * 4] << 16) |
				(usb_in_buffer[3 + i * 4] << 8) |
				(usb_in_buffer[2 + i * 4]);
		} else {
			*word = (usb_in_buffer[2 + i * 4] << 24) |
				(usb_in_buffer[3 + i * 4] << 16) |
				(usb_in_buffer[4 + i * 4] << 8) |
				(usb_in_buffer[5 + i * 4]);
		}
	}
}

static void aice_unpack_dthb(uint8_t *cmd_ack_code, uint8_t *extra_word_length)
{
	*cmd_ack_code = usb_in_buffer[0];
	*extra_word_length = usb_in_buffer[1];
}

static void aice_unpack_dthma(uint8_t *cmd_ack_code, uint8_t *target_id,
		uint8_t *extra_word_length, uint32_t *word,
		enum aice_target_endian access_endian)
{
	*cmd_ack_code = usb_in_buffer[0];
	*target_id = usb_in_buffer[1];
	*extra_word_length = usb_in_buffer[2];
	if (access_endian == AICE_BIG_ENDIAN) {
		*word = (usb_in_buffer[7] << 24) |
			(usb_in_buffer[6] << 16) |
			(usb_in_buffer[5] << 8) |
			(usb_in_buffer[4]);
	} else {
		*word = (usb_in_buffer[4] << 24) |
			(usb_in_buffer[5] << 16) |
			(usb_in_buffer[6] << 8) |
			(usb_in_buffer[7]);
	}
}

static void aice_unpack_dthma_multiple_data(uint8_t *cmd_ack_code,
		uint8_t *target_id, uint8_t *extra_word_length, uint8_t *word,
		enum aice_target_endian access_endian)
{
	*cmd_ack_code = usb_in_buffer[0];
	*target_id = usb_in_buffer[1];
	*extra_word_length = usb_in_buffer[2];
	if (access_endian == AICE_BIG_ENDIAN) {
		word[0] = usb_in_buffer[4];
		word[1] = usb_in_buffer[5];
		word[2] = usb_in_buffer[6];
		word[3] = usb_in_buffer[7];
	} else {
		word[0] = usb_in_buffer[7];
		word[1] = usb_in_buffer[6];
		word[2] = usb_in_buffer[5];
		word[3] = usb_in_buffer[4];
	}
	word += 4;

	uint8_t i;
	for (i = 0; i < *extra_word_length; i++) {
		if (access_endian == AICE_BIG_ENDIAN) {
			word[0] = usb_in_buffer[8 + i * 4];
			word[1] = usb_in_buffer[9 + i * 4];
			word[2] = usb_in_buffer[10 + i * 4];
			word[3] = usb_in_buffer[11 + i * 4];
		} else {
			word[0] = usb_in_buffer[11 + i * 4];
			word[1] = usb_in_buffer[10 + i * 4];
			word[2] = usb_in_buffer[9 + i * 4];
			word[3] = usb_in_buffer[8 + i * 4];
		}
		word += 4;
	}
}

static void aice_unpack_dthmb(uint8_t *cmd_ack_code, uint8_t *target_id,
		uint8_t *extra_word_length)
{
	*cmd_ack_code = usb_in_buffer[0];
	*target_id = usb_in_buffer[1];
	*extra_word_length = usb_in_buffer[2];
}

/***************************************************************************/
/* End of AICE commands' pack/unpack functions */

/* calls the given usb_bulk_* function, allowing for the data to
 * trickle in with some timeouts  */
static int usb_bulk_with_retries(
			int (*f)(jtag_libusb_device_handle *, int, char *, int, int),
			jtag_libusb_device_handle *dev, int ep,
			char *bytes, int size, int timeout)
{
	int tries = 3, count = 0;

	while (tries && (count < size)) {
		int result = f(dev, ep, bytes + count, size - count, timeout);
		if (result > 0)
			count += result;
		else if ((-ETIMEDOUT != result) || !--tries)
			return result;
	}
	return count;
}

static int wrap_usb_bulk_write(jtag_libusb_device_handle *dev, int ep,
		char *buff, int size, int timeout)
{
	/* usb_bulk_write() takes const char *buff */
	return jtag_libusb_bulk_write(dev, ep, buff, size, timeout);
}

static inline int usb_bulk_write_ex(jtag_libusb_device_handle *dev, int ep,
		char *bytes, int size, int timeout)
{
	return usb_bulk_with_retries(&wrap_usb_bulk_write,
			dev, ep, bytes, size, timeout);
}

static inline int usb_bulk_read_ex(jtag_libusb_device_handle *dev, int ep,
		char *bytes, int size, int timeout)
{
	return usb_bulk_with_retries(&jtag_libusb_bulk_read,
			dev, ep, bytes, size, timeout);
}

/* Write data from out_buffer to USB. */
static int aice_usb_write(uint8_t *out_buffer, int out_length)
{
	int result;

	if (out_length > AICE_OUT_BUFFER_SIZE) {
		LOG_ERROR("aice_write illegal out_length=%d (max=%d)",
				out_length, AICE_OUT_BUFFER_SIZE);
		return -1;
	}

	result = usb_bulk_write_ex(aice_handler.usb_handle, aice_handler.usb_write_ep,
			(char *)out_buffer, out_length, AICE_USB_TIMEOUT);

	DEBUG_JTAG_IO("aice_usb_write, out_length = %d, result = %d",
			out_length, result);

	return result;
}

/* Read data from USB into in_buffer. */
static int aice_usb_read(uint8_t *in_buffer, int expected_size)
{
	int result = usb_bulk_read_ex(aice_handler.usb_handle, aice_handler.usb_read_ep,
			(char *)in_buffer, expected_size, AICE_USB_TIMEOUT);

	DEBUG_JTAG_IO("aice_usb_read, result = %d", result);

	return result;
}

static uint8_t usb_out_packets_buffer[AICE_OUT_PACKETS_BUFFER_SIZE];
static uint8_t usb_in_packets_buffer[AICE_IN_PACKETS_BUFFER_SIZE];
static uint32_t usb_out_packets_buffer_length;
static uint32_t usb_in_packets_buffer_length;
static bool usb_pack_command;

static int aice_usb_packet_flush(void)
{
	if (usb_out_packets_buffer_length == 0)
		return 0;

	LOG_DEBUG("Flush usb packets");

	int result;

	aice_usb_write(usb_out_packets_buffer, usb_out_packets_buffer_length);
	result = aice_usb_read(usb_in_packets_buffer, usb_in_packets_buffer_length);

	usb_out_packets_buffer_length = 0;
	usb_in_packets_buffer_length = 0;

	return result;
}

static void aice_usb_packet_append(uint8_t *out_buffer, int out_length,
		int in_length)
{
	if (usb_out_packets_buffer_length + out_length > AICE_OUT_PACKETS_BUFFER_SIZE)
		aice_usb_packet_flush();

	LOG_DEBUG("Append usb packets 0x%02x", out_buffer[0]);

	memcpy(usb_out_packets_buffer + usb_out_packets_buffer_length,
			out_buffer,
			out_length);
	usb_out_packets_buffer_length += out_length;
	usb_in_packets_buffer_length += in_length;
}

/***************************************************************************/
/* AICE commands */
static int aice_scan_chain(uint32_t *id_codes, uint8_t *num_of_ids)
{
	int result;
	int retry_times = 0;

	if (usb_pack_command)
		aice_usb_packet_flush();

	do {
		aice_pack_htda(AICE_CMD_SCAN_CHAIN, 0x0F, 0x0);

		aice_usb_write(usb_out_buffer, AICE_FORMAT_HTDA);

		LOG_DEBUG("SCAN_CHAIN, length: 0x0F");

		/** TODO: modify receive length */
		result = aice_usb_read(usb_in_buffer, AICE_FORMAT_DTHA);
		if (AICE_FORMAT_DTHA != result) {
			LOG_ERROR("aice_usb_read failed (requested=%d, result=%d)",
					AICE_FORMAT_DTHA, result);
			return ERROR_FAIL;
		}

		uint8_t cmd_ack_code;
		aice_unpack_dtha_multiple_data(&cmd_ack_code, num_of_ids, id_codes,
				0x10, AICE_LITTLE_ENDIAN);

		LOG_DEBUG("SCAN_CHAIN response, # of IDs: %d", *num_of_ids);

		if (cmd_ack_code != AICE_CMD_SCAN_CHAIN) {
			LOG_ERROR("aice command timeout (command=0x%x, response=0x%x)",
					AICE_CMD_SCAN_CHAIN, cmd_ack_code);

			if (retry_times > aice_max_retry_times)
				return ERROR_FAIL;

			/* clear timeout and retry */
			if (aice_write_ctrl(AICE_WRITE_CTRL_CLEAR_TIMEOUT_STATUS, 0x1) != ERROR_OK)
				return ERROR_FAIL;

			retry_times++;
			continue;
		}

		if (*num_of_ids == 0xFF) {
			LOG_ERROR("No target connected");
			return ERROR_FAIL;
		} else if (*num_of_ids == 0x10) {
			LOG_INFO("The ice chain over 16 targets");
		} else {
			(*num_of_ids)++;
		}
		break;
	} while (1);

	return ERROR_OK;
}

int aice_read_ctrl(uint32_t address, uint32_t *data)
{
	int result;

	if (usb_pack_command)
		aice_usb_packet_flush();

	aice_pack_htda(AICE_CMD_READ_CTRL, 0, address);

	aice_usb_write(usb_out_buffer, AICE_FORMAT_HTDA);

	LOG_DEBUG("READ_CTRL, address: 0x%x", address);

	result = aice_usb_read(usb_in_buffer, AICE_FORMAT_DTHA);
	if (AICE_FORMAT_DTHA != result) {
		LOG_ERROR("aice_usb_read failed (requested=%d, result=%d)",
				AICE_FORMAT_DTHA, result);
		return ERROR_FAIL;
	}

	uint8_t cmd_ack_code;
	uint8_t extra_length;
	aice_unpack_dtha(&cmd_ack_code, &extra_length, data, AICE_LITTLE_ENDIAN);

	LOG_DEBUG("READ_CTRL response, data: 0x%x", *data);

	if (cmd_ack_code != AICE_CMD_READ_CTRL) {
		LOG_ERROR("aice command error (command=0x%x, response=0x%x)",
				AICE_CMD_READ_CTRL, cmd_ack_code);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

int aice_write_ctrl(uint32_t address, uint32_t data)
{
	int result;

	if (usb_pack_command)
		aice_usb_packet_flush();

	aice_pack_htdc(AICE_CMD_WRITE_CTRL, 0, address, data, AICE_LITTLE_ENDIAN);

	aice_usb_write(usb_out_buffer, AICE_FORMAT_HTDC);

	LOG_DEBUG("WRITE_CTRL, address: 0x%x, data: 0x%x", address, data);

	result = aice_usb_read(usb_in_buffer, AICE_FORMAT_DTHB);
	if (AICE_FORMAT_DTHB != result) {
		LOG_ERROR("aice_usb_read failed (requested=%d, result=%d)",
				AICE_FORMAT_DTHB, result);
		return ERROR_FAIL;
	}

	uint8_t cmd_ack_code;
	uint8_t extra_length;
	aice_unpack_dthb(&cmd_ack_code, &extra_length);

	LOG_DEBUG("WRITE_CTRL response");

	if (cmd_ack_code != AICE_CMD_WRITE_CTRL) {
		LOG_ERROR("aice command error (command=0x%x, response=0x%x)",
				AICE_CMD_WRITE_CTRL, cmd_ack_code);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

int aice_read_dtr(uint8_t target_id, uint32_t *data)
{
	int result;
	int retry_times = 0;

	if (usb_pack_command)
		aice_usb_packet_flush();

	do {
		aice_pack_htdma(AICE_CMD_T_READ_DTR, target_id, 0, 0);

		aice_usb_write(usb_out_buffer, AICE_FORMAT_HTDMA);

		LOG_DEBUG("READ_DTR");

		result = aice_usb_read(usb_in_buffer, AICE_FORMAT_DTHMA);
		if (AICE_FORMAT_DTHMA != result) {
			LOG_ERROR("aice_usb_read failed (requested=%d, result=%d)",
					AICE_FORMAT_DTHMA, result);
			return ERROR_FAIL;
		}

		uint8_t cmd_ack_code;
		uint8_t extra_length;
		uint8_t res_target_id;
		aice_unpack_dthma(&cmd_ack_code, &res_target_id, &extra_length,
				data, AICE_LITTLE_ENDIAN);

		LOG_DEBUG("READ_DTR response, data: 0x%x", *data);

		if (cmd_ack_code == AICE_CMD_T_READ_DTR) {
			break;
		} else {
			LOG_ERROR("aice command timeout (command=0x%x, response=0x%x)",
					AICE_CMD_T_READ_DTR, cmd_ack_code);

			if (retry_times > aice_max_retry_times)
				return ERROR_FAIL;

			/* clear timeout and retry */
			if (aice_write_ctrl(AICE_WRITE_CTRL_CLEAR_TIMEOUT_STATUS, 0x1) != ERROR_OK)
				return ERROR_FAIL;

			retry_times++;
		}
	} while (1);

	return ERROR_OK;
}

int aice_write_dtr(uint8_t target_id, uint32_t data)
{
	int result;
	int retry_times = 0;

	if (usb_pack_command)
		aice_usb_packet_flush();

	do {
		aice_pack_htdmc(AICE_CMD_T_WRITE_DTR, target_id, 0, 0, data,
				AICE_LITTLE_ENDIAN);

		aice_usb_write(usb_out_buffer, AICE_FORMAT_HTDMC);

		LOG_DEBUG("WRITE_DTR, data: 0x%x", data);

		result = aice_usb_read(usb_in_buffer, AICE_FORMAT_DTHMB);
		if (AICE_FORMAT_DTHMB != result) {
			LOG_ERROR("aice_usb_read failed (requested=%d, result=%d)",
					AICE_FORMAT_DTHMB, result);
			return ERROR_FAIL;
		}

		uint8_t cmd_ack_code;
		uint8_t extra_length;
		uint8_t res_target_id;
		aice_unpack_dthmb(&cmd_ack_code, &res_target_id, &extra_length);

		LOG_DEBUG("WRITE_DTR response");

		if (cmd_ack_code == AICE_CMD_T_WRITE_DTR) {
			break;
		} else {
			LOG_ERROR("aice command timeout (command=0x%x, response=0x%x)",
					AICE_CMD_T_WRITE_DTR, cmd_ack_code);

			if (retry_times > aice_max_retry_times)
				return ERROR_FAIL;

			/* clear timeout and retry */
			if (aice_write_ctrl(AICE_WRITE_CTRL_CLEAR_TIMEOUT_STATUS, 0x1) != ERROR_OK)
				return ERROR_FAIL;

			retry_times++;
		}
	} while (1);

	return ERROR_OK;
}

int aice_read_misc(uint8_t target_id, uint32_t address, uint32_t *data)
{
	int result;
	int retry_times = 0;

	if (usb_pack_command)
		aice_usb_packet_flush();

	do {
		aice_pack_htdma(AICE_CMD_T_READ_MISC, target_id, 0, address);

		aice_usb_write(usb_out_buffer, AICE_FORMAT_HTDMA);

		LOG_DEBUG("READ_MISC, address: 0x%x", address);

		result = aice_usb_read(usb_in_buffer, AICE_FORMAT_DTHMA);
		if (AICE_FORMAT_DTHMA != result) {
			LOG_ERROR("aice_usb_read failed (requested=%d, result=%d)",
					AICE_FORMAT_DTHMA, result);
			return ERROR_AICE_DISCONNECT;
		}

		uint8_t cmd_ack_code;
		uint8_t extra_length;
		uint8_t res_target_id;
		aice_unpack_dthma(&cmd_ack_code, &res_target_id, &extra_length,
				data, AICE_LITTLE_ENDIAN);

		LOG_DEBUG("READ_MISC response, data: 0x%x", *data);

		if (cmd_ack_code == AICE_CMD_T_READ_MISC) {
			break;
		} else {
			LOG_ERROR("aice command timeout (command=0x%x, response=0x%x)",
					AICE_CMD_T_READ_MISC, cmd_ack_code);

			if (retry_times > aice_max_retry_times)
				return ERROR_FAIL;

			/* clear timeout and retry */
			if (aice_write_ctrl(AICE_WRITE_CTRL_CLEAR_TIMEOUT_STATUS, 0x1) != ERROR_OK)
				return ERROR_FAIL;

			retry_times++;
		}
	} while (1);

	return ERROR_OK;
}

int aice_write_misc(uint8_t target_id, uint32_t address, uint32_t data)
{
	int result;
	int retry_times = 0;

	if (usb_pack_command)
		aice_usb_packet_flush();

	do {
		aice_pack_htdmc(AICE_CMD_T_WRITE_MISC, target_id, 0, address,
				data, AICE_LITTLE_ENDIAN);

		aice_usb_write(usb_out_buffer, AICE_FORMAT_HTDMC);

		LOG_DEBUG("WRITE_MISC, address: 0x%x, data: 0x%x", address, data);

		result = aice_usb_read(usb_in_buffer, AICE_FORMAT_DTHMB);
		if (AICE_FORMAT_DTHMB != result) {
			LOG_ERROR("aice_usb_read failed (requested=%d, result=%d)",
					AICE_FORMAT_DTHMB, result);
			return ERROR_FAIL;
		}

		uint8_t cmd_ack_code;
		uint8_t extra_length;
		uint8_t res_target_id;
		aice_unpack_dthmb(&cmd_ack_code, &res_target_id, &extra_length);

		LOG_DEBUG("WRITE_MISC response");

		if (cmd_ack_code == AICE_CMD_T_WRITE_MISC) {
			break;
		} else {
			LOG_ERROR("aice command timeout (command=0x%x, response=0x%x)",
					AICE_CMD_T_WRITE_MISC, cmd_ack_code);

			if (retry_times > aice_max_retry_times)
				return ERROR_FAIL;

			/* clear timeout and retry */
			if (aice_write_ctrl(AICE_WRITE_CTRL_CLEAR_TIMEOUT_STATUS, 0x1) != ERROR_OK)
				return ERROR_FAIL;

			retry_times++;
		}
	} while (1);

	return ERROR_OK;
}

int aice_read_edmsr(uint8_t target_id, uint32_t address, uint32_t *data)
{
	int result;
	int retry_times = 0;

	if (usb_pack_command)
		aice_usb_packet_flush();

	do {
		aice_pack_htdma(AICE_CMD_T_READ_EDMSR, target_id, 0, address);

		aice_usb_write(usb_out_buffer, AICE_FORMAT_HTDMA);

		LOG_DEBUG("READ_EDMSR, address: 0x%x", address);

		result = aice_usb_read(usb_in_buffer, AICE_FORMAT_DTHMA);
		if (AICE_FORMAT_DTHMA != result) {
			LOG_ERROR("aice_usb_read failed (requested=%d, result=%d)",
					AICE_FORMAT_DTHMA, result);
			return ERROR_FAIL;
		}

		uint8_t cmd_ack_code;
		uint8_t extra_length;
		uint8_t res_target_id;
		aice_unpack_dthma(&cmd_ack_code, &res_target_id, &extra_length,
				data, AICE_LITTLE_ENDIAN);

		LOG_DEBUG("READ_EDMSR response, data: 0x%x", *data);

		if (cmd_ack_code == AICE_CMD_T_READ_EDMSR) {
			break;
		} else {
			LOG_ERROR("aice command timeout (command=0x%x, response=0x%x)",
					AICE_CMD_T_READ_EDMSR, cmd_ack_code);

			if (retry_times > aice_max_retry_times)
				return ERROR_FAIL;

			/* clear timeout and retry */
			if (aice_write_ctrl(AICE_WRITE_CTRL_CLEAR_TIMEOUT_STATUS, 0x1) != ERROR_OK)
				return ERROR_FAIL;

			retry_times++;
		}
	} while (1);

	return ERROR_OK;
}

int aice_write_edmsr(uint8_t target_id, uint32_t address, uint32_t data)
{
	int result;
	int retry_times = 0;

	if (usb_pack_command)
		aice_usb_packet_flush();

	do {
		aice_pack_htdmc(AICE_CMD_T_WRITE_EDMSR, target_id, 0, address,
				data, AICE_LITTLE_ENDIAN);

		aice_usb_write(usb_out_buffer, AICE_FORMAT_HTDMC);

		LOG_DEBUG("WRITE_EDMSR, address: 0x%x, data: 0x%x", address, data);

		result = aice_usb_read(usb_in_buffer, AICE_FORMAT_DTHMB);
		if (AICE_FORMAT_DTHMB != result) {
			LOG_ERROR("aice_usb_read failed (requested=%d, result=%d)",
					AICE_FORMAT_DTHMB, result);
			return ERROR_FAIL;
		}

		uint8_t cmd_ack_code;
		uint8_t extra_length;
		uint8_t res_target_id;
		aice_unpack_dthmb(&cmd_ack_code, &res_target_id, &extra_length);

		LOG_DEBUG("WRITE_EDMSR response");

		if (cmd_ack_code == AICE_CMD_T_WRITE_EDMSR) {
			break;
		} else {
			LOG_ERROR("aice command timeout (command=0x%x, response=0x%x)",
					AICE_CMD_T_WRITE_EDMSR, cmd_ack_code);

			if (retry_times > aice_max_retry_times)
				return ERROR_FAIL;

			/* clear timeout and retry */
			if (aice_write_ctrl(AICE_WRITE_CTRL_CLEAR_TIMEOUT_STATUS, 0x1) != ERROR_OK)
				return ERROR_FAIL;

			retry_times++;
		}
	} while (1);

	return ERROR_OK;
}

static int aice_switch_to_big_endian(uint32_t *word, uint8_t num_of_words)
{
	uint32_t tmp;

	for (uint8_t i = 0 ; i < num_of_words ; i++) {
		tmp = ((word[i] >> 24) & 0x000000FF) |
			((word[i] >>  8) & 0x0000FF00) |
			((word[i] <<  8) & 0x00FF0000) |
			((word[i] << 24) & 0xFF000000);
		word[i] = tmp;
	}

	return ERROR_OK;
}

static int aice_write_dim(uint8_t target_id, uint32_t *word, uint8_t num_of_words)
{
	int result;
	uint32_t big_endian_word[4];
	int retry_times = 0;

	if (usb_pack_command)
		aice_usb_packet_flush();

	memcpy(big_endian_word, word, sizeof(big_endian_word));

	/** instruction is big-endian */
	aice_switch_to_big_endian(big_endian_word, num_of_words);

	do {
		aice_pack_htdmc_multiple_data(AICE_CMD_T_WRITE_DIM, target_id,
				num_of_words - 1, 0,
				big_endian_word, num_of_words, AICE_LITTLE_ENDIAN);

		aice_usb_write(usb_out_buffer, AICE_FORMAT_HTDMC + (num_of_words - 1) * 4);

		LOG_DEBUG("WRITE_DIM, data: 0x%08x, 0x%08x, 0x%08x, 0x%08x",
				big_endian_word[0],
				big_endian_word[1],
				big_endian_word[2],
				big_endian_word[3]);

		result = aice_usb_read(usb_in_buffer, AICE_FORMAT_DTHMB);
		if (AICE_FORMAT_DTHMB != result) {
			LOG_ERROR("aice_usb_read failed (requested=%d, result=%d)",
					AICE_FORMAT_DTHMB, result);
			return ERROR_FAIL;
		}

		uint8_t cmd_ack_code;
		uint8_t extra_length;
		uint8_t res_target_id;
		aice_unpack_dthmb(&cmd_ack_code, &res_target_id, &extra_length);

		LOG_DEBUG("WRITE_DIM response");

		if (cmd_ack_code == AICE_CMD_T_WRITE_DIM) {
			break;
		} else {
			LOG_ERROR("aice command timeout (command=0x%x, response=0x%x)",
					AICE_CMD_T_WRITE_DIM, cmd_ack_code);

			if (retry_times > aice_max_retry_times)
				return ERROR_FAIL;

			/* clear timeout and retry */
			if (aice_write_ctrl(AICE_WRITE_CTRL_CLEAR_TIMEOUT_STATUS, 0x1) != ERROR_OK)
				return ERROR_FAIL;

			retry_times++;
		}
	} while (1);

	return ERROR_OK;
}

static int aice_do_execute(uint8_t target_id)
{
	int result;
	int retry_times = 0;

	if (usb_pack_command)
		aice_usb_packet_flush();

	do {
		aice_pack_htdmc(AICE_CMD_T_EXECUTE, target_id, 0, 0, 0, AICE_LITTLE_ENDIAN);

		aice_usb_write(usb_out_buffer, AICE_FORMAT_HTDMC);

		LOG_DEBUG("EXECUTE");

		result = aice_usb_read(usb_in_buffer, AICE_FORMAT_DTHMB);
		if (AICE_FORMAT_DTHMB != result) {
			LOG_ERROR("aice_usb_read failed (requested=%d, result=%d)",
					AICE_FORMAT_DTHMB, result);
			return ERROR_FAIL;
		}

		uint8_t cmd_ack_code;
		uint8_t extra_length;
		uint8_t res_target_id;
		aice_unpack_dthmb(&cmd_ack_code, &res_target_id, &extra_length);

		LOG_DEBUG("EXECUTE response");

		if (cmd_ack_code == AICE_CMD_T_EXECUTE) {
			break;
		} else {
			LOG_ERROR("aice command timeout (command=0x%x, response=0x%x)",
					AICE_CMD_T_EXECUTE, cmd_ack_code);

			if (retry_times > aice_max_retry_times)
				return ERROR_FAIL;

			/* clear timeout and retry */
			if (aice_write_ctrl(AICE_WRITE_CTRL_CLEAR_TIMEOUT_STATUS, 0x1) != ERROR_OK)
				return ERROR_FAIL;

			retry_times++;
		}
	} while (1);

	return ERROR_OK;
}

int aice_write_mem_b(uint8_t target_id, uint32_t address, uint32_t data)
{
	int result;
	int retry_times = 0;

	LOG_DEBUG("WRITE_MEM_B, ADDRESS %08" PRIx32 "  VALUE %08" PRIx32,
			address,
			data);

	if (usb_pack_command) {
		aice_pack_htdmd(AICE_CMD_T_WRITE_MEM_B, target_id, 0, address,
				data & 0x000000FF, data_endian);
		aice_usb_packet_append(usb_out_buffer, AICE_FORMAT_HTDMD, AICE_FORMAT_DTHMB);
	} else {
		do {
			aice_pack_htdmd(AICE_CMD_T_WRITE_MEM_B, target_id, 0,
					address, data & 0x000000FF, data_endian);
			aice_usb_write(usb_out_buffer, AICE_FORMAT_HTDMD);

			result = aice_usb_read(usb_in_buffer, AICE_FORMAT_DTHMB);
			if (AICE_FORMAT_DTHMB != result) {
				LOG_ERROR("aice_usb_read failed (requested=%d, result=%d)",
						AICE_FORMAT_DTHMB, result);
				return ERROR_FAIL;
			}

			uint8_t cmd_ack_code;
			uint8_t extra_length;
			uint8_t res_target_id;
			aice_unpack_dthmb(&cmd_ack_code, &res_target_id, &extra_length);

			if (cmd_ack_code == AICE_CMD_T_WRITE_MEM_B) {
				break;
			} else {
				LOG_ERROR("aice command timeout (command=0x%x, response=0x%x)",
						AICE_CMD_T_WRITE_MEM_B, cmd_ack_code);

				if (retry_times > aice_max_retry_times)
					return ERROR_FAIL;

				/* clear timeout and retry */
				if (aice_write_ctrl(AICE_WRITE_CTRL_CLEAR_TIMEOUT_STATUS, 0x1) != ERROR_OK)
					return ERROR_FAIL;

				retry_times++;
			}
		} while (1);
	}

	return ERROR_OK;
}

int aice_write_mem_h(uint8_t target_id, uint32_t address, uint32_t data)
{
	int result;
	int retry_times = 0;

	LOG_DEBUG("WRITE_MEM_H, ADDRESS %08" PRIx32 "  VALUE %08" PRIx32,
			address,
			data);

	if (usb_pack_command) {
		aice_pack_htdmd(AICE_CMD_T_WRITE_MEM_H, target_id, 0,
				(address >> 1) & 0x7FFFFFFF, data & 0x0000FFFF, data_endian);
		aice_usb_packet_append(usb_out_buffer, AICE_FORMAT_HTDMD, AICE_FORMAT_DTHMB);
	} else {
		do {
			aice_pack_htdmd(AICE_CMD_T_WRITE_MEM_H, target_id, 0,
					(address >> 1) & 0x7FFFFFFF, data & 0x0000FFFF, data_endian);
			aice_usb_write(usb_out_buffer, AICE_FORMAT_HTDMD);

			result = aice_usb_read(usb_in_buffer, AICE_FORMAT_DTHMB);
			if (AICE_FORMAT_DTHMB != result) {
				LOG_ERROR("aice_usb_read failed (requested=%d, result=%d)",
						AICE_FORMAT_DTHMB, result);
				return ERROR_FAIL;
			}

			uint8_t cmd_ack_code;
			uint8_t extra_length;
			uint8_t res_target_id;
			aice_unpack_dthmb(&cmd_ack_code, &res_target_id, &extra_length);

			if (cmd_ack_code == AICE_CMD_T_WRITE_MEM_H) {
				break;
			} else {
				LOG_ERROR("aice command timeout (command=0x%x, response=0x%x)",
						AICE_CMD_T_WRITE_MEM_H, cmd_ack_code);

				if (retry_times > aice_max_retry_times)
					return ERROR_FAIL;

				/* clear timeout and retry */
				if (aice_write_ctrl(AICE_WRITE_CTRL_CLEAR_TIMEOUT_STATUS, 0x1) != ERROR_OK)
					return ERROR_FAIL;

				retry_times++;
			}
		} while (1);
	}

	return ERROR_OK;
}

int aice_write_mem(uint8_t target_id, uint32_t address, uint32_t data)
{
	int result;
	int retry_times = 0;

	LOG_DEBUG("WRITE_MEM, ADDRESS %08" PRIx32 "  VALUE %08" PRIx32,
			address,
			data);

	if (usb_pack_command) {
		aice_pack_htdmd(AICE_CMD_T_WRITE_MEM, target_id, 0,
				(address >> 2) & 0x3FFFFFFF, data, data_endian);
		aice_usb_packet_append(usb_out_buffer, AICE_FORMAT_HTDMD, AICE_FORMAT_DTHMB);
	} else {
		do {
			aice_pack_htdmd(AICE_CMD_T_WRITE_MEM, target_id, 0,
					(address >> 2) & 0x3FFFFFFF, data, data_endian);
			aice_usb_write(usb_out_buffer, AICE_FORMAT_HTDMD);

			result = aice_usb_read(usb_in_buffer, AICE_FORMAT_DTHMB);
			if (AICE_FORMAT_DTHMB != result) {
				LOG_ERROR("aice_usb_read failed (requested=%d, result=%d)",
						AICE_FORMAT_DTHMB, result);
				return ERROR_FAIL;
			}

			uint8_t cmd_ack_code;
			uint8_t extra_length;
			uint8_t res_target_id;
			aice_unpack_dthmb(&cmd_ack_code, &res_target_id, &extra_length);

			if (cmd_ack_code == AICE_CMD_T_WRITE_MEM) {
				break;
			} else {
				LOG_ERROR("aice command timeout (command=0x%x, response=0x%x)",
						AICE_CMD_T_WRITE_MEM, cmd_ack_code);

				if (retry_times > aice_max_retry_times)
					return ERROR_FAIL;

				/* clear timeout and retry */
				if (aice_write_ctrl(AICE_WRITE_CTRL_CLEAR_TIMEOUT_STATUS, 0x1) != ERROR_OK)
					return ERROR_FAIL;

				retry_times++;
			}
		} while (1);
	}

	return ERROR_OK;
}

int aice_fastread_mem(uint8_t target_id, uint8_t *word, uint32_t num_of_words)
{
	int result;
	int retry_times = 0;

	if (usb_pack_command)
		aice_usb_packet_flush();

	do {
		aice_pack_htdmb(AICE_CMD_T_FASTREAD_MEM, target_id, num_of_words - 1, 0);

		aice_usb_write(usb_out_buffer, AICE_FORMAT_HTDMB);

		LOG_DEBUG("FASTREAD_MEM, # of DATA %08" PRIx32, num_of_words);

		result = aice_usb_read(usb_in_buffer, AICE_FORMAT_DTHMA + (num_of_words - 1) * 4);
		if (result < 0) {
			LOG_ERROR("aice_usb_read failed (requested=%d, result=%d)",
					AICE_FORMAT_DTHMA + (num_of_words - 1) * 4, result);
			return ERROR_FAIL;
		}

		uint8_t cmd_ack_code;
		uint8_t extra_length;
		uint8_t res_target_id;
		aice_unpack_dthma_multiple_data(&cmd_ack_code, &res_target_id,
				&extra_length, word, data_endian);

		if (cmd_ack_code == AICE_CMD_T_FASTREAD_MEM) {
			break;
		} else {
			LOG_ERROR("aice command timeout (command=0x%x, response=0x%x)",
					AICE_CMD_T_FASTREAD_MEM, cmd_ack_code);

			if (retry_times > aice_max_retry_times)
				return ERROR_FAIL;

			/* clear timeout and retry */
			if (aice_write_ctrl(AICE_WRITE_CTRL_CLEAR_TIMEOUT_STATUS, 0x1) != ERROR_OK)
				return ERROR_FAIL;

			retry_times++;
		}
	} while (1);

	return ERROR_OK;
}

int aice_fastwrite_mem(uint8_t target_id, const uint8_t *word, uint32_t num_of_words)
{
	int result;
	int retry_times = 0;

	if (usb_pack_command)
		aice_usb_packet_flush();

	do {
		aice_pack_htdmd_multiple_data(AICE_CMD_T_FASTWRITE_MEM, target_id,
				num_of_words - 1, 0, word, data_endian);

		aice_usb_write(usb_out_buffer, AICE_FORMAT_HTDMD + (num_of_words - 1) * 4);

		LOG_DEBUG("FASTWRITE_MEM, # of DATA %08" PRIx32, num_of_words);

		result = aice_usb_read(usb_in_buffer, AICE_FORMAT_DTHMB);
		if (AICE_FORMAT_DTHMB != result) {
			LOG_ERROR("aice_usb_read failed (requested=%d, result=%d)",
					AICE_FORMAT_DTHMB, result);
			return ERROR_FAIL;
		}

		uint8_t cmd_ack_code;
		uint8_t extra_length;
		uint8_t res_target_id;
		aice_unpack_dthmb(&cmd_ack_code, &res_target_id, &extra_length);

		if (cmd_ack_code == AICE_CMD_T_FASTWRITE_MEM) {
			break;
		} else {
			LOG_ERROR("aice command timeout (command=0x%x, response=0x%x)",
					AICE_CMD_T_FASTWRITE_MEM, cmd_ack_code);

			if (retry_times > aice_max_retry_times)
				return ERROR_FAIL;

			/* clear timeout and retry */
			if (aice_write_ctrl(AICE_WRITE_CTRL_CLEAR_TIMEOUT_STATUS, 0x1) != ERROR_OK)
				return ERROR_FAIL;

			retry_times++;
		}
	} while (1);

	return ERROR_OK;
}

int aice_read_mem_b(uint8_t target_id, uint32_t address, uint32_t *data)
{
	int result;
	int retry_times = 0;

	if (usb_pack_command)
		aice_usb_packet_flush();

	do {
		aice_pack_htdmb(AICE_CMD_T_READ_MEM_B, target_id, 0, address);

		aice_usb_write(usb_out_buffer, AICE_FORMAT_HTDMB);

		LOG_DEBUG("READ_MEM_B");

		result = aice_usb_read(usb_in_buffer, AICE_FORMAT_DTHMA);
		if (AICE_FORMAT_DTHMA != result) {
			LOG_ERROR("aice_usb_read failed (requested=%d, result=%d)",
					AICE_FORMAT_DTHMA, result);
			return ERROR_FAIL;
		}

		uint8_t cmd_ack_code;
		uint8_t extra_length;
		uint8_t res_target_id;
		aice_unpack_dthma(&cmd_ack_code, &res_target_id, &extra_length,
				data, data_endian);

		LOG_DEBUG("READ_MEM_B response, data: 0x%x", *data);

		if (cmd_ack_code == AICE_CMD_T_READ_MEM_B) {
			break;
		} else {
			LOG_ERROR("aice command timeout (command=0x%x, response=0x%x)",
					AICE_CMD_T_READ_MEM_B, cmd_ack_code);

			if (retry_times > aice_max_retry_times)
				return ERROR_FAIL;

			/* clear timeout and retry */
			if (aice_write_ctrl(AICE_WRITE_CTRL_CLEAR_TIMEOUT_STATUS, 0x1) != ERROR_OK)
				return ERROR_FAIL;

			retry_times++;
		}
	} while (1);

	return ERROR_OK;
}

int aice_read_mem_h(uint8_t target_id, uint32_t address, uint32_t *data)
{
	int result;
	int retry_times = 0;

	if (usb_pack_command)
		aice_usb_packet_flush();

	do {
		aice_pack_htdmb(AICE_CMD_T_READ_MEM_H, target_id, 0, (address >> 1) & 0x7FFFFFFF);

		aice_usb_write(usb_out_buffer, AICE_FORMAT_HTDMB);

		LOG_DEBUG("READ_MEM_H");

		result = aice_usb_read(usb_in_buffer, AICE_FORMAT_DTHMA);
		if (AICE_FORMAT_DTHMA != result) {
			LOG_ERROR("aice_usb_read failed (requested=%d, result=%d)",
					AICE_FORMAT_DTHMA, result);
			return ERROR_FAIL;
		}

		uint8_t cmd_ack_code;
		uint8_t extra_length;
		uint8_t res_target_id;
		aice_unpack_dthma(&cmd_ack_code, &res_target_id, &extra_length,
				data, data_endian);

		LOG_DEBUG("READ_MEM_H response, data: 0x%x", *data);

		if (cmd_ack_code == AICE_CMD_T_READ_MEM_H) {
			break;
		} else {
			LOG_ERROR("aice command timeout (command=0x%x, response=0x%x)",
					AICE_CMD_T_READ_MEM_H, cmd_ack_code);

			if (retry_times > aice_max_retry_times)
				return ERROR_FAIL;

			/* clear timeout and retry */
			if (aice_write_ctrl(AICE_WRITE_CTRL_CLEAR_TIMEOUT_STATUS, 0x1) != ERROR_OK)
				return ERROR_FAIL;

			retry_times++;
		}
	} while (1);

	return ERROR_OK;
}

int aice_read_mem(uint8_t target_id, uint32_t address, uint32_t *data)
{
	int result;
	int retry_times = 0;

	if (usb_pack_command)
		aice_usb_packet_flush();

	do {
		aice_pack_htdmb(AICE_CMD_T_READ_MEM, target_id, 0,
				(address >> 2) & 0x3FFFFFFF);

		aice_usb_write(usb_out_buffer, AICE_FORMAT_HTDMB);

		LOG_DEBUG("READ_MEM");

		result = aice_usb_read(usb_in_buffer, AICE_FORMAT_DTHMA);
		if (AICE_FORMAT_DTHMA != result) {
			LOG_ERROR("aice_usb_read failed (requested=%d, result=%d)",
					AICE_FORMAT_DTHMA, result);
			return ERROR_FAIL;
		}

		uint8_t cmd_ack_code;
		uint8_t extra_length;
		uint8_t res_target_id;
		aice_unpack_dthma(&cmd_ack_code, &res_target_id, &extra_length,
				data, data_endian);

		LOG_DEBUG("READ_MEM response, data: 0x%x", *data);

		if (cmd_ack_code == AICE_CMD_T_READ_MEM) {
			break;
		} else {
			LOG_ERROR("aice command timeout (command=0x%x, response=0x%x)",
					AICE_CMD_T_READ_MEM, cmd_ack_code);

			if (retry_times > aice_max_retry_times)
				return ERROR_FAIL;

			/* clear timeout and retry */
			if (aice_write_ctrl(AICE_WRITE_CTRL_CLEAR_TIMEOUT_STATUS, 0x1) != ERROR_OK)
				return ERROR_FAIL;

			retry_times++;
		}
	} while (1);

	return ERROR_OK;
}

/***************************************************************************/
/* End of AICE commands */

typedef int (*read_mem_func_t)(uint32_t address, uint32_t *data);
typedef int (*write_mem_func_t)(uint32_t address, uint32_t data);
struct cache_info {
	uint32_t set;
	uint32_t way;
	uint32_t line_size;

	uint32_t log2_set;
	uint32_t log2_line_size;
};

static uint32_t r0_backup;
static uint32_t r1_backup;
static uint32_t host_dtr_backup;
static uint32_t target_dtr_backup;
static uint32_t edmsw_backup;
static uint32_t edm_ctl_backup;
static bool debug_under_dex_on;
static bool dex_use_psw_on;
static bool host_dtr_valid;
static bool target_dtr_valid;
static enum nds_memory_access access_channel = NDS_MEMORY_ACC_CPU;
static enum nds_memory_select memory_select = NDS_MEMORY_SELECT_AUTO;
static enum aice_target_state_s core_state = AICE_TARGET_UNKNOWN;
static uint32_t edm_version;
static struct cache_info icache = {0, 0, 0, 0, 0};
static struct cache_info dcache = {0, 0, 0, 0, 0};
static bool cache_init;
static char *custom_srst_script;
static char *custom_trst_script;
static char *custom_restart_script;
static uint32_t aice_count_to_check_dbger = 30;

static int aice_read_reg(uint32_t num, uint32_t *val);
static int aice_write_reg(uint32_t num, uint32_t val);

static int check_suppressed_exception(uint32_t dbger_value)
{
	uint32_t ir4_value;
	uint32_t ir6_value;
	/* the default value of handling_suppressed_exception is false */
	static bool handling_suppressed_exception;

	if (handling_suppressed_exception)
		return ERROR_OK;

	if ((dbger_value & NDS_DBGER_ALL_SUPRS_EX) == NDS_DBGER_ALL_SUPRS_EX) {
		LOG_ERROR("<-- TARGET WARNING! Exception is detected and suppressed. -->");
		handling_suppressed_exception = true;

		aice_read_reg(IR4, &ir4_value);
		/* Clear IR6.SUPRS_EXC, IR6.IMP_EXC */
		aice_read_reg(IR6, &ir6_value);
		/*
		 * For MCU version(MSC_CFG.MCU == 1) like V3m
		 *  | SWID[30:16] | Reserved[15:10] | SUPRS_EXC[9]  | IMP_EXC[8]
		 *  |VECTOR[7:5] | INST[4] | Exc Type[3:0] |
		 *
		 * For non-MCU version(MSC_CFG.MCU == 0) like V3
		 *  | SWID[30:16] | Reserved[15:14] | SUPRS_EXC[13] | IMP_EXC[12]
		 *  | VECTOR[11:5] | INST[4] | Exc Type[3:0] |
		 */
		LOG_INFO("EVA: 0x%08x", ir4_value);
		LOG_INFO("ITYPE: 0x%08x", ir6_value);

		ir6_value = ir6_value & (~0x300); /* for MCU */
		ir6_value = ir6_value & (~0x3000); /* for non-MCU */
		aice_write_reg(IR6, ir6_value);

		handling_suppressed_exception = false;
	}

	return ERROR_OK;
}

static int check_privilege(uint32_t dbger_value)
{
	if ((dbger_value & NDS_DBGER_ILL_SEC_ACC) == NDS_DBGER_ILL_SEC_ACC) {
		LOG_ERROR("<-- TARGET ERROR! Insufficient security privilege "
				"to execute the debug operations. -->");

		/* Clear DBGER.ILL_SEC_ACC */
		if (aice_write_misc(current_target_id, NDS_EDM_MISC_DBGER,
					NDS_DBGER_ILL_SEC_ACC) != ERROR_OK)
			return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int aice_check_dbger(uint32_t expect_status)
{
	uint32_t i = 0;
	uint32_t value_dbger;

	while (1) {
		aice_read_misc(current_target_id, NDS_EDM_MISC_DBGER, &value_dbger);

		if ((value_dbger & expect_status) == expect_status) {
			if (ERROR_OK != check_suppressed_exception(value_dbger))
				return ERROR_FAIL;
			if (ERROR_OK != check_privilege(value_dbger))
				return ERROR_FAIL;
			return ERROR_OK;
		}

		long long then = 0;
		if (i == aice_count_to_check_dbger)
			then = timeval_ms();
		if (i >= aice_count_to_check_dbger) {
			if ((timeval_ms() - then) > 1000) {
				LOG_ERROR("Timeout (1000ms) waiting for $DBGER status "
						"being 0x%08x", expect_status);
				return ERROR_FAIL;
			}
		}
		i++;
	}

	return ERROR_FAIL;
}

static int aice_execute_dim(uint32_t *insts, uint8_t n_inst)
{
	/** fill DIM */
	if (aice_write_dim(current_target_id, insts, n_inst) != ERROR_OK)
		return ERROR_FAIL;

	/** clear DBGER.DPED */
	if (aice_write_misc(current_target_id, NDS_EDM_MISC_DBGER, NDS_DBGER_DPED) != ERROR_OK)
		return ERROR_FAIL;

	/** execute DIM */
	if (aice_do_execute(current_target_id) != ERROR_OK)
		return ERROR_FAIL;

	/** read DBGER.DPED */
	if (aice_check_dbger(NDS_DBGER_DPED) != ERROR_OK) {
		LOG_ERROR("<-- TARGET ERROR! Debug operations do not finish properly: "
				"0x%08x 0x%08x 0x%08x 0x%08x. -->",
				insts[0],
				insts[1],
				insts[2],
				insts[3]);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int aice_read_reg(uint32_t num, uint32_t *val)
{
	LOG_DEBUG("aice_read_reg, reg_no: 0x%08x", num);

	uint32_t instructions[4]; /** execute instructions in DIM */

	if (NDS32_REG_TYPE_GPR == nds32_reg_type(num)) { /* general registers */
		instructions[0] = MTSR_DTR(num);
		instructions[1] = DSB;
		instructions[2] = NOP;
		instructions[3] = BEQ_MINUS_12;
	} else if (NDS32_REG_TYPE_SPR == nds32_reg_type(num)) { /* user special registers */
		instructions[0] = MFUSR_G0(0, nds32_reg_sr_index(num));
		instructions[1] = MTSR_DTR(0);
		instructions[2] = DSB;
		instructions[3] = BEQ_MINUS_12;
	} else if (NDS32_REG_TYPE_AUMR == nds32_reg_type(num)) { /* audio registers */
		if ((CB_CTL <= num) && (num <= CBE3)) {
			instructions[0] = AMFAR2(0, nds32_reg_sr_index(num));
			instructions[1] = MTSR_DTR(0);
			instructions[2] = DSB;
			instructions[3] = BEQ_MINUS_12;
		} else {
			instructions[0] = AMFAR(0, nds32_reg_sr_index(num));
			instructions[1] = MTSR_DTR(0);
			instructions[2] = DSB;
			instructions[3] = BEQ_MINUS_12;
		}
	} else if (NDS32_REG_TYPE_FPU == nds32_reg_type(num)) { /* fpu registers */
		if (FPCSR == num) {
			instructions[0] = FMFCSR;
			instructions[1] = MTSR_DTR(0);
			instructions[2] = DSB;
			instructions[3] = BEQ_MINUS_12;
		} else if (FPCFG == num) {
			instructions[0] = FMFCFG;
			instructions[1] = MTSR_DTR(0);
			instructions[2] = DSB;
			instructions[3] = BEQ_MINUS_12;
		} else {
			if (FS0 <= num && num <= FS31) { /* single precision */
				instructions[0] = FMFSR(0, nds32_reg_sr_index(num));
				instructions[1] = MTSR_DTR(0);
				instructions[2] = DSB;
				instructions[3] = BEQ_MINUS_12;
			} else if (FD0 <= num && num <= FD31) { /* double precision */
				instructions[0] = FMFDR(0, nds32_reg_sr_index(num));
				instructions[1] = MTSR_DTR(0);
				instructions[2] = DSB;
				instructions[3] = BEQ_MINUS_12;
			}
		}
	} else { /* system registers */
		instructions[0] = MFSR(0, nds32_reg_sr_index(num));
		instructions[1] = MTSR_DTR(0);
		instructions[2] = DSB;
		instructions[3] = BEQ_MINUS_12;
	}

	aice_execute_dim(instructions, 4);

	uint32_t value_edmsw;
	aice_read_edmsr(current_target_id, NDS_EDM_SR_EDMSW, &value_edmsw);
	if (value_edmsw & NDS_EDMSW_WDV)
		aice_read_dtr(current_target_id, val);
	else {
		LOG_ERROR("<-- TARGET ERROR! The debug target failed to update "
				"the DTR register. -->");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int aice_usb_read_reg(uint32_t num, uint32_t *val)
{
	LOG_DEBUG("aice_usb_read_reg");

	if (num == R0) {
		*val = r0_backup;
	} else if (num == R1) {
		*val = r1_backup;
	} else if (num == DR41) {
		/* As target is halted, OpenOCD will backup DR41/DR42/DR43.
		 * As user wants to read these registers, OpenOCD should return
		 * the backup values, instead of reading the real values.
		 * As user wants to write these registers, OpenOCD should write
		 * to the backup values, instead of writing to real registers. */
		*val = edmsw_backup;
	} else if (num == DR42) {
		*val = edm_ctl_backup;
	} else if ((target_dtr_valid == true) && (num == DR43)) {
		*val = target_dtr_backup;
	} else {
		if (ERROR_OK != aice_read_reg(num, val))
			*val = 0xBBADBEEF;
	}

	return ERROR_OK;
}

static int aice_write_reg(uint32_t num, uint32_t val)
{
	LOG_DEBUG("aice_write_reg, reg_no: 0x%08x, value: 0x%08x", num, val);

	uint32_t instructions[4]; /** execute instructions in DIM */
	uint32_t value_edmsw;

	aice_write_dtr(current_target_id, val);
	aice_read_edmsr(current_target_id, NDS_EDM_SR_EDMSW, &value_edmsw);
	if (0 == (value_edmsw & NDS_EDMSW_RDV)) {
		LOG_ERROR("<-- TARGET ERROR! AICE failed to write to the DTR register. -->");
		return ERROR_FAIL;
	}

	if (NDS32_REG_TYPE_GPR == nds32_reg_type(num)) { /* general registers */
		instructions[0] = MFSR_DTR(num);
		instructions[1] = DSB;
		instructions[2] = NOP;
		instructions[3] = BEQ_MINUS_12;
	} else if (NDS32_REG_TYPE_SPR == nds32_reg_type(num)) { /* user special registers */
		instructions[0] = MFSR_DTR(0);
		instructions[1] = MTUSR_G0(0, nds32_reg_sr_index(num));
		instructions[2] = DSB;
		instructions[3] = BEQ_MINUS_12;
	} else if (NDS32_REG_TYPE_AUMR == nds32_reg_type(num)) { /* audio registers */
		if ((CB_CTL <= num) && (num <= CBE3)) {
			instructions[0] = MFSR_DTR(0);
			instructions[1] = AMTAR2(0, nds32_reg_sr_index(num));
			instructions[2] = DSB;
			instructions[3] = BEQ_MINUS_12;
		} else {
			instructions[0] = MFSR_DTR(0);
			instructions[1] = AMTAR(0, nds32_reg_sr_index(num));
			instructions[2] = DSB;
			instructions[3] = BEQ_MINUS_12;
		}
	} else if (NDS32_REG_TYPE_FPU == nds32_reg_type(num)) { /* fpu registers */
		if (FPCSR == num) {
			instructions[0] = MFSR_DTR(0);
			instructions[1] = FMTCSR;
			instructions[2] = DSB;
			instructions[3] = BEQ_MINUS_12;
		} else if (FPCFG == num) {
			/* FPCFG is readonly */
		} else {
			if (FS0 <= num && num <= FS31) { /* single precision */
				instructions[0] = MFSR_DTR(0);
				instructions[1] = FMTSR(0, nds32_reg_sr_index(num));
				instructions[2] = DSB;
				instructions[3] = BEQ_MINUS_12;
			} else if (FD0 <= num && num <= FD31) { /* double precision */
				instructions[0] = MFSR_DTR(0);
				instructions[1] = FMTDR(0, nds32_reg_sr_index(num));
				instructions[2] = DSB;
				instructions[3] = BEQ_MINUS_12;
			}
		}
	} else {
		instructions[0] = MFSR_DTR(0);
		instructions[1] = MTSR(0, nds32_reg_sr_index(num));
		instructions[2] = DSB;
		instructions[3] = BEQ_MINUS_12;
	}

	return aice_execute_dim(instructions, 4);
}

static int aice_usb_write_reg(uint32_t num, uint32_t val)
{
	LOG_DEBUG("aice_usb_write_reg");

	if (num == R0)
		r0_backup = val;
	else if (num == R1)
		r1_backup = val;
	else if (num == DR42)
		/* As target is halted, OpenOCD will backup DR41/DR42/DR43.
		 * As user wants to read these registers, OpenOCD should return
		 * the backup values, instead of reading the real values.
		 * As user wants to write these registers, OpenOCD should write
		 * to the backup values, instead of writing to real registers. */
		edm_ctl_backup = val;
	else if ((target_dtr_valid == true) && (num == DR43))
		target_dtr_backup = val;
	else
		return aice_write_reg(num, val);

	return ERROR_OK;
}

static int aice_usb_open(struct aice_port_param_s *param)
{
	const uint16_t vids[] = { param->vid, 0 };
	const uint16_t pids[] = { param->pid, 0 };
	struct jtag_libusb_device_handle *devh;

	if (jtag_libusb_open(vids, pids, &devh) != ERROR_OK)
		return ERROR_FAIL;

	/* BE ***VERY CAREFUL*** ABOUT MAKING CHANGES IN THIS
	 * AREA!!!!!!!!!!!  The behavior of libusb is not completely
	 * consistent across Windows, Linux, and Mac OS X platforms.
	 * The actions taken in the following compiler conditionals may
	 * not agree with published documentation for libusb, but were
	 * found to be necessary through trials and tribulations.  Even
	 * little tweaks can break one or more platforms, so if you do
	 * make changes test them carefully on all platforms before
	 * committing them!
	 */

#if IS_WIN32 == 0

	jtag_libusb_reset_device(devh);

#if IS_DARWIN == 0

	int timeout = 5;
	/* reopen jlink after usb_reset
	 * on win32 this may take a second or two to re-enumerate */
	int retval;
	while ((retval = jtag_libusb_open(vids, pids, &devh)) != ERROR_OK) {
		usleep(1000);
		timeout--;
		if (!timeout)
			break;
	}
	if (ERROR_OK != retval)
		return ERROR_FAIL;
#endif

#endif

	/* usb_set_configuration required under win32 */
	struct jtag_libusb_device *udev = jtag_libusb_get_device(devh);
	jtag_libusb_set_configuration(devh, 0);
	jtag_libusb_claim_interface(devh, 0);

	unsigned int aice_read_ep;
	unsigned int aice_write_ep;
	jtag_libusb_get_endpoints(udev, &aice_read_ep, &aice_write_ep);

	aice_handler.usb_read_ep = aice_read_ep;
	aice_handler.usb_write_ep = aice_write_ep;
	aice_handler.usb_handle = devh;

	return ERROR_OK;
}

static int aice_usb_read_reg_64(uint32_t num, uint64_t *val)
{
	LOG_DEBUG("aice_usb_read_reg_64, %s", nds32_reg_simple_name(num));

	uint32_t value;
	uint32_t high_value;

	if (ERROR_OK != aice_read_reg(num, &value))
		value = 0xBBADBEEF;

	aice_read_reg(R1, &high_value);

	LOG_DEBUG("low: 0x%08x, high: 0x%08x\n", value, high_value);

	if (data_endian == AICE_BIG_ENDIAN)
		*val = (((uint64_t)high_value) << 32) | value;
	else
		*val = (((uint64_t)value) << 32) | high_value;

	return ERROR_OK;
}

static int aice_usb_write_reg_64(uint32_t num, uint64_t val)
{
	uint32_t value;
	uint32_t high_value;

	if (data_endian == AICE_BIG_ENDIAN) {
		value = val & 0xFFFFFFFF;
		high_value = (val >> 32) & 0xFFFFFFFF;
	} else {
		high_value = val & 0xFFFFFFFF;
		value = (val >> 32) & 0xFFFFFFFF;
	}

	LOG_DEBUG("aice_usb_write_reg_64, %s, low: 0x%08x, high: 0x%08x\n",
			nds32_reg_simple_name(num), value, high_value);

	aice_write_reg(R1, high_value);
	return aice_write_reg(num, value);
}

static int aice_get_version_info(void)
{
	uint32_t hardware_version;
	uint32_t firmware_version;
	uint32_t fpga_version;

	if (aice_read_ctrl(AICE_READ_CTRL_GET_HARDWARE_VERSION, &hardware_version) != ERROR_OK)
		return ERROR_FAIL;

	if (aice_read_ctrl(AICE_READ_CTRL_GET_FIRMWARE_VERSION, &firmware_version) != ERROR_OK)
		return ERROR_FAIL;

	if (aice_read_ctrl(AICE_READ_CTRL_GET_FPGA_VERSION, &fpga_version) != ERROR_OK)
		return ERROR_FAIL;

	LOG_INFO("AICE version: hw_ver = 0x%x, fw_ver = 0x%x, fpga_ver = 0x%x",
			hardware_version, firmware_version, fpga_version);

	return ERROR_OK;
}

#define LINE_BUFFER_SIZE 1024

static int aice_execute_custom_script(const char *script)
{
	FILE *script_fd;
	char line_buffer[LINE_BUFFER_SIZE];
	char *op_str;
	char *reset_str;
	uint32_t delay;
	uint32_t write_ctrl_value;
	bool set_op;

	script_fd = fopen(script, "r");
	if (script_fd == NULL) {
		return ERROR_FAIL;
	} else {
		while (fgets(line_buffer, LINE_BUFFER_SIZE, script_fd) != NULL) {
			/* execute operations */
			set_op = false;
			op_str = strstr(line_buffer, "set");
			if (op_str != NULL) {
				set_op = true;
				goto get_reset_type;
			}

			op_str = strstr(line_buffer, "clear");
			if (op_str == NULL)
				continue;
get_reset_type:
			reset_str = strstr(op_str, "srst");
			if (reset_str != NULL) {
				if (set_op)
					write_ctrl_value = AICE_CUSTOM_DELAY_SET_SRST;
				else
					write_ctrl_value = AICE_CUSTOM_DELAY_CLEAN_SRST;
				goto get_delay;
			}
			reset_str = strstr(op_str, "dbgi");
			if (reset_str != NULL) {
				if (set_op)
					write_ctrl_value = AICE_CUSTOM_DELAY_SET_DBGI;
				else
					write_ctrl_value = AICE_CUSTOM_DELAY_CLEAN_DBGI;
				goto get_delay;
			}
			reset_str = strstr(op_str, "trst");
			if (reset_str != NULL) {
				if (set_op)
					write_ctrl_value = AICE_CUSTOM_DELAY_SET_TRST;
				else
					write_ctrl_value = AICE_CUSTOM_DELAY_CLEAN_TRST;
				goto get_delay;
			}
			continue;
get_delay:
			/* get delay */
			delay = strtoul(reset_str + 4, NULL, 0);
			write_ctrl_value |= (delay << 16);

			if (aice_write_ctrl(AICE_WRITE_CTRL_CUSTOM_DELAY,
						write_ctrl_value) != ERROR_OK) {
				fclose(script_fd);
				return ERROR_FAIL;
			}
		}
		fclose(script_fd);
	}

	return ERROR_OK;
}

static int aice_edm_reset(void)
{
	if (aice_write_ctrl(AICE_WRITE_CTRL_CLEAR_TIMEOUT_STATUS, 0x1) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int aice_usb_set_clock(int set_clock)
{
	if (aice_write_ctrl(AICE_WRITE_CTRL_TCK_CONTROL,
				AICE_TCK_CONTROL_TCK_SCAN) != ERROR_OK)
		return ERROR_FAIL;

	/* Read out TCK_SCAN clock value */
	uint32_t scan_clock;
	if (aice_read_ctrl(AICE_READ_CTRL_GET_ICE_STATE, &scan_clock) != ERROR_OK)
		return ERROR_FAIL;

	scan_clock &= 0x0F;

	uint32_t scan_base_freq;
	if (scan_clock & 0x8)
		scan_base_freq = 48000; /* 48 MHz */
	else
		scan_base_freq = 30000; /* 30 MHz */

	uint32_t set_base_freq;
	if (set_clock & 0x8)
		set_base_freq = 48000;
	else
		set_base_freq = 30000;

	uint32_t set_freq;
	uint32_t scan_freq;
	set_freq = set_base_freq >> (set_clock & 0x7);
	scan_freq = scan_base_freq >> (scan_clock & 0x7);

	if (scan_freq < set_freq) {
		LOG_ERROR("User specifies higher jtag clock than TCK_SCAN clock");
		return ERROR_FAIL;
	}

	if (aice_write_ctrl(AICE_WRITE_CTRL_TCK_CONTROL, set_clock) != ERROR_OK)
		return ERROR_FAIL;

	uint32_t check_speed;
	if (aice_read_ctrl(AICE_READ_CTRL_GET_ICE_STATE, &check_speed) != ERROR_OK)
		return ERROR_FAIL;

	if (((int)check_speed & 0x0F) != set_clock) {
		LOG_ERROR("Set jtag clock failed");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int aice_edm_init(void)
{
	aice_write_edmsr(current_target_id, NDS_EDM_SR_DIMBR, 0xFFFF0000);

	/* unconditionally try to turn on V3_EDM_MODE */
	uint32_t edm_ctl_value;
	aice_read_edmsr(current_target_id, NDS_EDM_SR_EDM_CTL, &edm_ctl_value);
	aice_write_edmsr(current_target_id, NDS_EDM_SR_EDM_CTL, edm_ctl_value | 0x00000040);

	aice_write_misc(current_target_id, NDS_EDM_MISC_DBGER,
			NDS_DBGER_DPED | NDS_DBGER_CRST | NDS_DBGER_AT_MAX);
	aice_write_misc(current_target_id, NDS_EDM_MISC_DIMIR, 0);

	/* get EDM version */
	uint32_t value_edmcfg;
	aice_read_edmsr(current_target_id, NDS_EDM_SR_EDM_CFG, &value_edmcfg);
	edm_version = (value_edmcfg >> 16) & 0xFFFF;

	return ERROR_OK;
}

static bool is_v2_edm(void)
{
	if ((edm_version & 0x1000) == 0)
		return true;
	else
		return false;
}

static int aice_init_edm_registers(bool clear_dex_use_psw)
{
	/* enable DEH_SEL & MAX_STOP & V3_EDM_MODE & DBGI_MASK */
	uint32_t host_edm_ctl = edm_ctl_backup | 0xA000004F;
	if (clear_dex_use_psw)
		/* After entering debug mode, OpenOCD may set
		 * DEX_USE_PSW accidentally through backup value
		 * of target EDM_CTL.
		 * So, clear DEX_USE_PSW by force. */
		host_edm_ctl &= ~(0x40000000);

	LOG_DEBUG("aice_init_edm_registers - EDM_CTL: 0x%08x", host_edm_ctl);

	int result = aice_write_edmsr(current_target_id, NDS_EDM_SR_EDM_CTL, host_edm_ctl);

	return result;
}

/**
 * EDM_CTL will be modified by OpenOCD as debugging. OpenOCD has the
 * responsibility to keep EDM_CTL untouched after debugging.
 *
 * There are two scenarios to consider:
 * 1. single step/running as debugging (running under debug session)
 * 2. detached from gdb (exit debug session)
 *
 * So, we need to bakcup EDM_CTL before halted and restore it after
 * running. The difference of these two scenarios is EDM_CTL.DEH_SEL
 * is on for scenario 1, and off for scenario 2.
 */
static int aice_backup_edm_registers(void)
{
	int result = aice_read_edmsr(current_target_id, NDS_EDM_SR_EDM_CTL, &edm_ctl_backup);

	/* To call aice_backup_edm_registers() after DEX on, DEX_USE_PSW
	 * may be not correct.  (For example, hit breakpoint, then backup
	 * EDM_CTL. EDM_CTL.DEX_USE_PSW will be cleared.)  Because debug
	 * interrupt will clear DEX_USE_PSW, DEX_USE_PSW is always off after
	 * DEX is on.  It only backups correct value before OpenOCD issues DBGI.
	 * (Backup EDM_CTL, then issue DBGI actively (refer aice_usb_halt())) */
	if (edm_ctl_backup & 0x40000000)
		dex_use_psw_on = true;
	else
		dex_use_psw_on = false;

	LOG_DEBUG("aice_backup_edm_registers - EDM_CTL: 0x%08x, DEX_USE_PSW: %s",
			edm_ctl_backup, dex_use_psw_on ? "on" : "off");

	return result;
}

static int aice_restore_edm_registers(void)
{
	LOG_DEBUG("aice_restore_edm_registers -");

	/* set DEH_SEL, because target still under EDM control */
	int result = aice_write_edmsr(current_target_id, NDS_EDM_SR_EDM_CTL,
			edm_ctl_backup | 0x80000000);

	return result;
}

static int aice_backup_tmp_registers(void)
{
	LOG_DEBUG("backup_tmp_registers -");

	/* backup target DTR first(if the target DTR is valid) */
	uint32_t value_edmsw;
	aice_read_edmsr(current_target_id, NDS_EDM_SR_EDMSW, &value_edmsw);
	edmsw_backup = value_edmsw;
	if (value_edmsw & 0x1) { /* EDMSW.WDV == 1 */
		aice_read_dtr(current_target_id, &target_dtr_backup);
		target_dtr_valid = true;

		LOG_DEBUG("Backup target DTR: 0x%08x", target_dtr_backup);
	} else {
		target_dtr_valid = false;
	}

	/* Target DTR has been backup, then backup $R0 and $R1 */
	aice_read_reg(R0, &r0_backup);
	aice_read_reg(R1, &r1_backup);

	/* backup host DTR(if the host DTR is valid) */
	if (value_edmsw & 0x2) { /* EDMSW.RDV == 1*/
		/* read out host DTR and write into target DTR, then use aice_read_edmsr to
		 * read out */
		uint32_t instructions[4] = {
			MFSR_DTR(R0), /* R0 has already been backup */
			DSB,
			MTSR_DTR(R0),
			BEQ_MINUS_12
		};
		aice_execute_dim(instructions, 4);

		aice_read_dtr(current_target_id, &host_dtr_backup);
		host_dtr_valid = true;

		LOG_DEBUG("Backup host DTR: 0x%08x", host_dtr_backup);
	} else {
		host_dtr_valid = false;
	}

	LOG_DEBUG("r0: 0x%08x, r1: 0x%08x", r0_backup, r1_backup);

	return ERROR_OK;
}

static int aice_restore_tmp_registers(void)
{
	LOG_DEBUG("restore_tmp_registers - r0: 0x%08x, r1: 0x%08x", r0_backup, r1_backup);

	if (target_dtr_valid) {
		uint32_t instructions[4] = {
			SETHI(R0, target_dtr_backup >> 12),
			ORI(R0, R0, target_dtr_backup & 0x00000FFF),
			NOP,
			BEQ_MINUS_12
		};
		aice_execute_dim(instructions, 4);

		instructions[0] = MTSR_DTR(R0);
		instructions[1] = DSB;
		instructions[2] = NOP;
		instructions[3] = BEQ_MINUS_12;
		aice_execute_dim(instructions, 4);

		LOG_DEBUG("Restore target DTR: 0x%08x", target_dtr_backup);
	}

	aice_write_reg(R0, r0_backup);
	aice_write_reg(R1, r1_backup);

	if (host_dtr_valid) {
		aice_write_dtr(current_target_id, host_dtr_backup);

		LOG_DEBUG("Restore host DTR: 0x%08x", host_dtr_backup);
	}

	return ERROR_OK;
}

static int aice_open_device(struct aice_port_param_s *param)
{
	if (ERROR_OK != aice_usb_open(param))
		return ERROR_FAIL;

	if (ERROR_FAIL == aice_get_version_info()) {
		LOG_ERROR("Cannot get AICE version!");
		return ERROR_FAIL;
	}

	LOG_INFO("AICE initialization started");

	/* attempt to reset Andes EDM */
	if (ERROR_FAIL == aice_edm_reset()) {
		LOG_ERROR("Cannot initial AICE Interface!");
		return ERROR_FAIL;
	}

	if (ERROR_OK != aice_edm_init()) {
		LOG_ERROR("Cannot initial EDM!");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int aice_usb_set_jtag_clock(uint32_t a_clock)
{
	jtag_clock = a_clock;

	if (ERROR_OK != aice_usb_set_clock(a_clock)) {
		LOG_ERROR("Cannot set AICE JTAG clock!");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int aice_usb_close(void)
{
	jtag_libusb_close(aice_handler.usb_handle);

	if (custom_srst_script)
		free(custom_srst_script);

	if (custom_trst_script)
		free(custom_trst_script);

	if (custom_restart_script)
		free(custom_restart_script);

	return ERROR_OK;
}

static int aice_usb_idcode(uint32_t *idcode, uint8_t *num_of_idcode)
{
	return aice_scan_chain(idcode, num_of_idcode);
}

static int aice_usb_halt(void)
{
	if (core_state == AICE_TARGET_HALTED) {
		LOG_DEBUG("aice_usb_halt check halted");
		return ERROR_OK;
	}

	LOG_DEBUG("aice_usb_halt");

	/** backup EDM registers */
	aice_backup_edm_registers();
	/** init EDM for host debugging */
	/** no need to clear dex_use_psw, because dbgi will clear it */
	aice_init_edm_registers(false);

	/** Clear EDM_CTL.DBGIM & EDM_CTL.DBGACKM */
	uint32_t edm_ctl_value;
	aice_read_edmsr(current_target_id, NDS_EDM_SR_EDM_CTL, &edm_ctl_value);
	if (edm_ctl_value & 0x3)
		aice_write_edmsr(current_target_id, NDS_EDM_SR_EDM_CTL, edm_ctl_value & ~(0x3));

	uint32_t dbger;
	uint32_t acc_ctl_value;

	debug_under_dex_on = false;
	aice_read_misc(current_target_id, NDS_EDM_MISC_DBGER, &dbger);

	if (dbger & NDS_DBGER_AT_MAX)
		LOG_ERROR("<-- TARGET ERROR! Reaching the max interrupt stack level. -->");

	if (dbger & NDS_DBGER_DEX) {
		if (is_v2_edm() == false) {
			/** debug 'debug mode'. use force_debug to issue dbgi */
			aice_read_misc(current_target_id, NDS_EDM_MISC_ACC_CTL, &acc_ctl_value);
			acc_ctl_value |= 0x8;
			aice_write_misc(current_target_id, NDS_EDM_MISC_ACC_CTL, acc_ctl_value);
			debug_under_dex_on = true;

			aice_write_misc(current_target_id, NDS_EDM_MISC_EDM_CMDR, 0);
			/* If CPU stalled due to AT_MAX, clear AT_MAX status. */
			if (dbger & NDS_DBGER_AT_MAX)
				aice_write_misc(current_target_id, NDS_EDM_MISC_DBGER, NDS_DBGER_AT_MAX);
		}
	} else {
		/** Issue DBGI normally */
		aice_write_misc(current_target_id, NDS_EDM_MISC_EDM_CMDR, 0);
		/* If CPU stalled due to AT_MAX, clear AT_MAX status. */
		if (dbger & NDS_DBGER_AT_MAX)
			aice_write_misc(current_target_id, NDS_EDM_MISC_DBGER, NDS_DBGER_AT_MAX);
	}

	if (aice_check_dbger(NDS_DBGER_DEX) != ERROR_OK) {
		LOG_ERROR("<-- TARGET ERROR! Unable to stop the debug target through DBGI. -->");
		return ERROR_FAIL;
	}

	if (debug_under_dex_on) {
		if (dex_use_psw_on == false) {
			/* under debug 'debug mode', force $psw to 'debug mode' bahavior */
			/* !!!NOTICE!!! this is workaround for debug 'debug mode'.
			 * it is only for debugging 'debug exception handler' purpose.
			 * after openocd detaches from target, target behavior is
			 * undefined. */
			uint32_t ir0_value;
			uint32_t debug_mode_ir0_value;
			aice_read_reg(IR0, &ir0_value);
			debug_mode_ir0_value = ir0_value | 0x408; /* turn on DEX, set POM = 1 */
			debug_mode_ir0_value &= ~(0x000000C1); /* turn off DT/IT/GIE */
			aice_write_reg(IR0, debug_mode_ir0_value);
		}
	}

	/** set EDM_CTL.DBGIM & EDM_CTL.DBGACKM after halt */
	if (edm_ctl_value & 0x3)
		aice_write_edmsr(current_target_id, NDS_EDM_SR_EDM_CTL, edm_ctl_value);

	/* backup r0 & r1 */
	aice_backup_tmp_registers();
	core_state = AICE_TARGET_HALTED;

	return ERROR_OK;
}

static int aice_usb_state(enum aice_target_state_s *state)
{
	uint32_t dbger_value;
	uint32_t ice_state;

	int result = aice_read_misc(current_target_id, NDS_EDM_MISC_DBGER, &dbger_value);

	if (ERROR_AICE_TIMEOUT == result) {
		if (aice_read_ctrl(AICE_READ_CTRL_GET_ICE_STATE, &ice_state) != ERROR_OK) {
			LOG_ERROR("<-- AICE ERROR! AICE is unplugged. -->");
			return ERROR_FAIL;
		}

		if ((ice_state & 0x20) == 0) {
			LOG_ERROR("<-- TARGET ERROR! Target is disconnected with AICE. -->");
			return ERROR_FAIL;
		} else {
			return ERROR_FAIL;
		}
	} else if (ERROR_AICE_DISCONNECT == result) {
		LOG_ERROR("<-- AICE ERROR! AICE is unplugged. -->");
		return ERROR_FAIL;
	}

	if ((dbger_value & NDS_DBGER_ILL_SEC_ACC) == NDS_DBGER_ILL_SEC_ACC) {
		LOG_ERROR("<-- TARGET ERROR! Insufficient security privilege. -->");

		/* Clear ILL_SEC_ACC */
		aice_write_misc(current_target_id, NDS_EDM_MISC_DBGER, NDS_DBGER_ILL_SEC_ACC);

		*state = AICE_TARGET_RUNNING;
		core_state = AICE_TARGET_RUNNING;
	} else if ((dbger_value & NDS_DBGER_AT_MAX) == NDS_DBGER_AT_MAX) {
		/* Issue DBGI to exit cpu stall */
		aice_usb_halt();

		/* Read OIPC to find out the trigger point */
		uint32_t ir11_value;
		aice_read_reg(IR11, &ir11_value);

		LOG_ERROR("<-- TARGET ERROR! Reaching the max interrupt stack level; "
				"CPU is stalled at 0x%08x for debugging. -->", ir11_value);

		*state = AICE_TARGET_HALTED;
	} else if ((dbger_value & NDS_DBGER_CRST) == NDS_DBGER_CRST) {
		LOG_DEBUG("DBGER.CRST is on.");

		*state = AICE_TARGET_RESET;
		core_state = AICE_TARGET_RUNNING;

		/* Clear CRST */
		aice_write_misc(current_target_id, NDS_EDM_MISC_DBGER, NDS_DBGER_CRST);
	} else if ((dbger_value & NDS_DBGER_DEX) == NDS_DBGER_DEX) {
		if (AICE_TARGET_RUNNING == core_state) {
			/* enter debug mode, init EDM registers */
			/* backup EDM registers */
			aice_backup_edm_registers();
			/* init EDM for host debugging */
			aice_init_edm_registers(true);
			aice_backup_tmp_registers();
			core_state = AICE_TARGET_HALTED;
		} else if (AICE_TARGET_UNKNOWN == core_state) {
			/* debug 'debug mode', use force debug to halt core */
			aice_usb_halt();
		}
		*state = AICE_TARGET_HALTED;
	} else {
		*state = AICE_TARGET_RUNNING;
		core_state = AICE_TARGET_RUNNING;
	}

	return ERROR_OK;
}

static int aice_usb_reset(void)
{
	if (aice_write_ctrl(AICE_WRITE_CTRL_CLEAR_TIMEOUT_STATUS, 0x1) != ERROR_OK)
		return ERROR_FAIL;

	if (custom_trst_script == NULL) {
		if (aice_write_ctrl(AICE_WRITE_CTRL_JTAG_PIN_CONTROL,
					AICE_JTAG_PIN_CONTROL_TRST) != ERROR_OK)
			return ERROR_FAIL;
	} else {
		/* custom trst operations */
		if (aice_execute_custom_script(custom_trst_script) != ERROR_OK)
			return ERROR_FAIL;
	}

	if (aice_usb_set_clock(jtag_clock) != ERROR_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int aice_issue_srst(void)
{
	LOG_DEBUG("aice_issue_srst");

	/* After issuing srst, target will be running. So we need to restore EDM_CTL. */
	aice_restore_edm_registers();

	if (custom_srst_script == NULL) {
		if (aice_write_ctrl(AICE_WRITE_CTRL_JTAG_PIN_CONTROL,
					AICE_JTAG_PIN_CONTROL_SRST) != ERROR_OK)
			return ERROR_FAIL;
	} else {
		/* custom srst operations */
		if (aice_execute_custom_script(custom_srst_script) != ERROR_OK)
			return ERROR_FAIL;
	}

	/* wait CRST infinitely */
	uint32_t dbger_value;
	int i = 0;
	while (1) {
		if (aice_read_misc(current_target_id,
					NDS_EDM_MISC_DBGER, &dbger_value) != ERROR_OK)
			return ERROR_FAIL;

		if (dbger_value & NDS_DBGER_CRST)
			break;

		if ((i % 30) == 0)
			keep_alive();
		i++;
	}

	host_dtr_valid = false;
	target_dtr_valid = false;

	core_state = AICE_TARGET_RUNNING;
	return ERROR_OK;
}

static int aice_issue_reset_hold(void)
{
	LOG_DEBUG("aice_issue_reset_hold");

	/* set no_dbgi_pin to 0 */
	uint32_t pin_status;
	aice_read_ctrl(AICE_READ_CTRL_GET_JTAG_PIN_STATUS, &pin_status);
	if (pin_status | 0x4)
		aice_write_ctrl(AICE_WRITE_CTRL_JTAG_PIN_STATUS, pin_status & (~0x4));

	/* issue restart */
	if (custom_restart_script == NULL) {
		if (aice_write_ctrl(AICE_WRITE_CTRL_JTAG_PIN_CONTROL,
					AICE_JTAG_PIN_CONTROL_RESTART) != ERROR_OK)
			return ERROR_FAIL;
	} else {
		/* custom restart operations */
		if (aice_execute_custom_script(custom_restart_script) != ERROR_OK)
			return ERROR_FAIL;
	}

	if (aice_check_dbger(NDS_DBGER_CRST | NDS_DBGER_DEX) == ERROR_OK) {
		aice_backup_tmp_registers();
		core_state = AICE_TARGET_HALTED;

		return ERROR_OK;
	} else {
		/* set no_dbgi_pin to 1 */
		aice_write_ctrl(AICE_WRITE_CTRL_JTAG_PIN_STATUS, pin_status | 0x4);

		/* issue restart again */
		if (custom_restart_script == NULL) {
			if (aice_write_ctrl(AICE_WRITE_CTRL_JTAG_PIN_CONTROL,
						AICE_JTAG_PIN_CONTROL_RESTART) != ERROR_OK)
				return ERROR_FAIL;
		} else {
			/* custom restart operations */
			if (aice_execute_custom_script(custom_restart_script) != ERROR_OK)
				return ERROR_FAIL;
		}

		if (aice_check_dbger(NDS_DBGER_CRST | NDS_DBGER_DEX) == ERROR_OK) {
			aice_backup_tmp_registers();
			core_state = AICE_TARGET_HALTED;

			return ERROR_OK;
		}

		/* do software reset-and-hold */
		aice_issue_srst();
		aice_usb_halt();

		uint32_t value_ir3;
		aice_read_reg(IR3, &value_ir3);
		aice_write_reg(PC, value_ir3 & 0xFFFF0000);
	}

	return ERROR_FAIL;
}

static int aice_usb_assert_srst(enum aice_srst_type_s srst)
{
	if ((AICE_SRST != srst) && (AICE_RESET_HOLD != srst))
		return ERROR_FAIL;

	/* clear DBGER */
	if (aice_write_misc(current_target_id, NDS_EDM_MISC_DBGER,
				NDS_DBGER_CLEAR_ALL) != ERROR_OK)
		return ERROR_FAIL;

	int result = ERROR_OK;
	if (AICE_SRST == srst)
		result = aice_issue_srst();
	else
		result = aice_issue_reset_hold();

	/* Clear DBGER.CRST after reset to avoid 'core-reset checking' errors.
	 * assert_srst is user-intentional reset behavior, so we could
	 * clear DBGER.CRST safely.
	 */
	if (aice_write_misc(current_target_id,
				NDS_EDM_MISC_DBGER, NDS_DBGER_CRST) != ERROR_OK)
		return ERROR_FAIL;

	return result;
}

static int aice_usb_run(void)
{
	LOG_DEBUG("aice_usb_run");

	uint32_t dbger_value;
	if (aice_read_misc(current_target_id,
				NDS_EDM_MISC_DBGER, &dbger_value) != ERROR_OK)
		return ERROR_FAIL;

	if ((dbger_value & NDS_DBGER_DEX) != NDS_DBGER_DEX) {
		LOG_WARNING("<-- TARGET WARNING! The debug target exited "
				"the debug mode unexpectedly. -->");
		return ERROR_FAIL;
	}

	/* restore r0 & r1 before free run */
	aice_restore_tmp_registers();
	core_state = AICE_TARGET_RUNNING;

	/* clear DBGER */
	aice_write_misc(current_target_id, NDS_EDM_MISC_DBGER,
			NDS_DBGER_CLEAR_ALL);

	/** restore EDM registers */
	/** OpenOCD should restore EDM_CTL **before** to exit debug state.
	 *  Otherwise, following instruction will read wrong EDM_CTL value.
	 *
	 *  pc -> mfsr $p0, EDM_CTL (single step)
	 *        slli $p0, $p0, 1
	 *        slri $p0, $p0, 31
	 */
	aice_restore_edm_registers();

	/** execute instructions in DIM */
	uint32_t instructions[4] = {
		NOP,
		NOP,
		NOP,
		IRET
	};
	int result = aice_execute_dim(instructions, 4);

	return result;
}

static int aice_usb_step(void)
{
	LOG_DEBUG("aice_usb_step");

	uint32_t ir0_value;
	uint32_t ir0_reg_num;

	if (is_v2_edm() == true)
		/* V2 EDM will push interrupt stack as debug exception */
		ir0_reg_num = IR1;
	else
		ir0_reg_num = IR0;

	/** enable HSS */
	aice_read_reg(ir0_reg_num, &ir0_value);
	if ((ir0_value & 0x800) == 0) {
		/** set PSW.HSS */
		ir0_value |= (0x01 << 11);
		aice_write_reg(ir0_reg_num, ir0_value);
	}

	if (ERROR_FAIL == aice_usb_run())
		return ERROR_FAIL;

	int i = 0;
	enum aice_target_state_s state;
	while (1) {
		/* read DBGER */
		if (aice_usb_state(&state) != ERROR_OK)
			return ERROR_FAIL;

		if (AICE_TARGET_HALTED == state)
			break;

		long long then = 0;
		if (i == 30)
			then = timeval_ms();

		if (i >= 30) {
			if ((timeval_ms() - then) > 1000)
				LOG_WARNING("Timeout (1000ms) waiting for halt to complete");

			return ERROR_FAIL;
		}
		i++;
	}

	/** disable HSS */
	aice_read_reg(ir0_reg_num, &ir0_value);
	ir0_value &= ~(0x01 << 11);
	aice_write_reg(ir0_reg_num, ir0_value);

	return ERROR_OK;
}

static int aice_usb_read_mem_b_bus(uint32_t address, uint32_t *data)
{
	return aice_read_mem_b(current_target_id, address, data);
}

static int aice_usb_read_mem_h_bus(uint32_t address, uint32_t *data)
{
	return aice_read_mem_h(current_target_id, address, data);
}

static int aice_usb_read_mem_w_bus(uint32_t address, uint32_t *data)
{
	return aice_read_mem(current_target_id, address, data);
}

static int aice_usb_read_mem_b_dim(uint32_t address, uint32_t *data)
{
	uint32_t value;
	uint32_t instructions[4] = {
		LBI_BI(R1, R0),
		MTSR_DTR(R1),
		DSB,
		BEQ_MINUS_12
	};

	aice_execute_dim(instructions, 4);

	aice_read_dtr(current_target_id, &value);
	*data = value & 0xFF;

	return ERROR_OK;
}

static int aice_usb_read_mem_h_dim(uint32_t address, uint32_t *data)
{
	uint32_t value;
	uint32_t instructions[4] = {
		LHI_BI(R1, R0),
		MTSR_DTR(R1),
		DSB,
		BEQ_MINUS_12
	};

	aice_execute_dim(instructions, 4);

	aice_read_dtr(current_target_id, &value);
	*data = value & 0xFFFF;

	return ERROR_OK;
}

static int aice_usb_read_mem_w_dim(uint32_t address, uint32_t *data)
{
	uint32_t instructions[4] = {
		LWI_BI(R1, R0),
		MTSR_DTR(R1),
		DSB,
		BEQ_MINUS_12
	};

	aice_execute_dim(instructions, 4);

	aice_read_dtr(current_target_id, data);

	return ERROR_OK;
}

static int aice_usb_set_address_dim(uint32_t address)
{
	uint32_t instructions[4] = {
		SETHI(R0, address >> 12),
		ORI(R0, R0, address & 0x00000FFF),
		NOP,
		BEQ_MINUS_12
	};

	return aice_execute_dim(instructions, 4);
}

static int aice_usb_read_memory_unit(uint32_t addr, uint32_t size,
		uint32_t count, uint8_t *buffer)
{
	LOG_DEBUG("aice_usb_read_memory_unit, addr: 0x%08x, size: %d, count: %d",
			addr, size, count);

	if (NDS_MEMORY_ACC_CPU == access_channel)
		aice_usb_set_address_dim(addr);

	uint32_t value;
	size_t i;
	read_mem_func_t read_mem_func;

	switch (size) {
		case 1:
			if (NDS_MEMORY_ACC_BUS == access_channel)
				read_mem_func = aice_usb_read_mem_b_bus;
			else
				read_mem_func = aice_usb_read_mem_b_dim;

			for (i = 0; i < count; i++) {
				read_mem_func(addr, &value);
				*buffer++ = (uint8_t)value;
				addr++;
			}
			break;
		case 2:
			if (NDS_MEMORY_ACC_BUS == access_channel)
				read_mem_func = aice_usb_read_mem_h_bus;
			else
				read_mem_func = aice_usb_read_mem_h_dim;

			for (i = 0; i < count; i++) {
				read_mem_func(addr, &value);
				uint16_t svalue = value;
				memcpy(buffer, &svalue, sizeof(uint16_t));
				buffer += 2;
				addr += 2;
			}
			break;
		case 4:
			if (NDS_MEMORY_ACC_BUS == access_channel)
				read_mem_func = aice_usb_read_mem_w_bus;
			else
				read_mem_func = aice_usb_read_mem_w_dim;

			for (i = 0; i < count; i++) {
				read_mem_func(addr, &value);
				memcpy(buffer, &value, sizeof(uint32_t));
				buffer += 4;
				addr += 4;
			}
			break;
	}

	return ERROR_OK;
}

static int aice_usb_write_mem_b_bus(uint32_t address, uint32_t data)
{
	return aice_write_mem_b(current_target_id, address, data);
}

static int aice_usb_write_mem_h_bus(uint32_t address, uint32_t data)
{
	return aice_write_mem_h(current_target_id, address, data);
}

static int aice_usb_write_mem_w_bus(uint32_t address, uint32_t data)
{
	return aice_write_mem(current_target_id, address, data);
}

static int aice_usb_write_mem_b_dim(uint32_t address, uint32_t data)
{
	uint32_t instructions[4] = {
		MFSR_DTR(R1),
		SBI_BI(R1, R0),
		DSB,
		BEQ_MINUS_12
	};

	aice_write_dtr(current_target_id, data & 0xFF);
	aice_execute_dim(instructions, 4);

	return ERROR_OK;
}

static int aice_usb_write_mem_h_dim(uint32_t address, uint32_t data)
{
	uint32_t instructions[4] = {
		MFSR_DTR(R1),
		SHI_BI(R1, R0),
		DSB,
		BEQ_MINUS_12
	};

	aice_write_dtr(current_target_id, data & 0xFFFF);
	aice_execute_dim(instructions, 4);

	return ERROR_OK;
}

static int aice_usb_write_mem_w_dim(uint32_t address, uint32_t data)
{
	uint32_t instructions[4] = {
		MFSR_DTR(R1),
		SWI_BI(R1, R0),
		DSB,
		BEQ_MINUS_12
	};

	aice_write_dtr(current_target_id, data);
	aice_execute_dim(instructions, 4);

	return ERROR_OK;
}

static int aice_usb_write_memory_unit(uint32_t addr, uint32_t size,
		uint32_t count, const uint8_t *buffer)
{
	LOG_DEBUG("aice_usb_write_memory_unit, addr: 0x%08x, size: %d, count: %d",
			addr, size, count);

	if (NDS_MEMORY_ACC_CPU == access_channel)
		aice_usb_set_address_dim(addr);

	size_t i;
	write_mem_func_t write_mem_func;

	switch (size) {
		case 1:
			if (NDS_MEMORY_ACC_BUS == access_channel)
				write_mem_func = aice_usb_write_mem_b_bus;
			else
				write_mem_func = aice_usb_write_mem_b_dim;

			for (i = 0; i < count; i++) {
				write_mem_func(addr, *buffer);
				buffer++;
				addr++;
			}
			break;
		case 2:
			if (NDS_MEMORY_ACC_BUS == access_channel)
				write_mem_func = aice_usb_write_mem_h_bus;
			else
				write_mem_func = aice_usb_write_mem_h_dim;

			for (i = 0; i < count; i++) {
				uint16_t value;
				memcpy(&value, buffer, sizeof(uint16_t));

				write_mem_func(addr, value);
				buffer += 2;
				addr += 2;
			}
			break;
		case 4:
			if (NDS_MEMORY_ACC_BUS == access_channel)
				write_mem_func = aice_usb_write_mem_w_bus;
			else
				write_mem_func = aice_usb_write_mem_w_dim;

			for (i = 0; i < count; i++) {
				uint32_t value;
				memcpy(&value, buffer, sizeof(uint32_t));

				write_mem_func(addr, value);
				buffer += 4;
				addr += 4;
			}
			break;
	}

	return ERROR_OK;
}

static int aice_bulk_read_mem(uint32_t addr, uint32_t count, uint8_t *buffer)
{
	uint32_t packet_size;

	while (count > 0) {
		packet_size = (count >= 0x100) ? 0x100 : count;

		/** set address */
		addr &= 0xFFFFFFFC;
		if (aice_write_misc(current_target_id, NDS_EDM_MISC_SBAR, addr) != ERROR_OK)
			return ERROR_FAIL;

		if (aice_fastread_mem(current_target_id, buffer,
					packet_size) != ERROR_OK)
			return ERROR_FAIL;

		buffer += (packet_size * 4);
		addr += (packet_size * 4);
		count -= packet_size;
	}

	return ERROR_OK;
}

static int aice_bulk_write_mem(uint32_t addr, uint32_t count, const uint8_t *buffer)
{
	uint32_t packet_size;

	while (count > 0) {
		packet_size = (count >= 0x100) ? 0x100 : count;

		/** set address */
		addr &= 0xFFFFFFFC;
		if (aice_write_misc(current_target_id, NDS_EDM_MISC_SBAR, addr | 1) != ERROR_OK)
			return ERROR_FAIL;

		if (aice_fastwrite_mem(current_target_id, buffer,
					packet_size) != ERROR_OK)
			return ERROR_FAIL;

		buffer += (packet_size * 4);
		addr += (packet_size * 4);
		count -= packet_size;
	}

	return ERROR_OK;
}

static int aice_usb_bulk_read_mem(uint32_t addr, uint32_t length, uint8_t *buffer)
{
	LOG_DEBUG("aice_usb_bulk_read_mem, addr: 0x%08x, length: 0x%08x", addr, length);

	int retval;

	if (NDS_MEMORY_ACC_CPU == access_channel)
		aice_usb_set_address_dim(addr);

	if (NDS_MEMORY_ACC_CPU == access_channel)
		retval = aice_usb_read_memory_unit(addr, 4, length / 4, buffer);
	else
		retval = aice_bulk_read_mem(addr, length / 4, buffer);

	return retval;
}

static int aice_usb_bulk_write_mem(uint32_t addr, uint32_t length, const uint8_t *buffer)
{
	LOG_DEBUG("aice_usb_bulk_write_mem, addr: 0x%08x, length: 0x%08x", addr, length);

	int retval;

	if (NDS_MEMORY_ACC_CPU == access_channel)
		aice_usb_set_address_dim(addr);

	if (NDS_MEMORY_ACC_CPU == access_channel)
		retval = aice_usb_write_memory_unit(addr, 4, length / 4, buffer);
	else
		retval = aice_bulk_write_mem(addr, length / 4, buffer);

	return retval;
}

static int aice_usb_read_debug_reg(uint32_t addr, uint32_t *val)
{
	if (AICE_TARGET_HALTED == core_state) {
		if (NDS_EDM_SR_EDMSW == addr) {
			*val = edmsw_backup;
		} else if (NDS_EDM_SR_EDM_DTR == addr) {
			if (target_dtr_valid) {
				/* if EDM_DTR has read out, clear it. */
				*val = target_dtr_backup;
				edmsw_backup &= (~0x1);
				target_dtr_valid = false;
			} else {
				*val = 0;
			}
		}
	}

	return aice_read_edmsr(current_target_id, addr, val);
}

static int aice_usb_write_debug_reg(uint32_t addr, const uint32_t val)
{
	if (AICE_TARGET_HALTED == core_state) {
		if (NDS_EDM_SR_EDM_DTR == addr) {
			host_dtr_backup = val;
			edmsw_backup |= 0x2;
			host_dtr_valid = true;
		}
	}

	return aice_write_edmsr(current_target_id, addr, val);
}

static int aice_usb_select_target(uint32_t target_id)
{
	current_target_id = target_id;

	return ERROR_OK;
}

static int aice_usb_memory_access(enum nds_memory_access channel)
{
	LOG_DEBUG("aice_usb_memory_access, access channel: %d", channel);

	access_channel = channel;

	return ERROR_OK;
}

static int aice_usb_memory_mode(enum nds_memory_select mem_select)
{
	if (memory_select == mem_select)
		return ERROR_OK;

	LOG_DEBUG("aice_usb_memory_mode, memory select: %d", mem_select);

	memory_select = mem_select;

	if (NDS_MEMORY_SELECT_AUTO != memory_select)
		aice_write_misc(current_target_id, NDS_EDM_MISC_ACC_CTL,
				memory_select - 1);
	else
		aice_write_misc(current_target_id, NDS_EDM_MISC_ACC_CTL,
				NDS_MEMORY_SELECT_MEM - 1);

	return ERROR_OK;
}

static int aice_usb_read_tlb(uint32_t virtual_address, uint32_t *physical_address)
{
	LOG_DEBUG("aice_usb_read_tlb, virtual address: 0x%08x", virtual_address);

	uint32_t instructions[4];
	uint32_t probe_result;
	uint32_t value_mr3;
	uint32_t value_mr4;
	uint32_t access_page_size;
	uint32_t virtual_offset;
	uint32_t physical_page_number;

	aice_write_dtr(current_target_id, virtual_address);

	/* probe TLB first */
	instructions[0] = MFSR_DTR(R0);
	instructions[1] = TLBOP_TARGET_PROBE(R1, R0);
	instructions[2] = DSB;
	instructions[3] = BEQ_MINUS_12;
	aice_execute_dim(instructions, 4);

	aice_read_reg(R1, &probe_result);

	if (probe_result & 0x80000000)
		return ERROR_FAIL;

	/* read TLB entry */
	aice_write_dtr(current_target_id, probe_result & 0x7FF);

	/* probe TLB first */
	instructions[0] = MFSR_DTR(R0);
	instructions[1] = TLBOP_TARGET_READ(R0);
	instructions[2] = DSB;
	instructions[3] = BEQ_MINUS_12;
	aice_execute_dim(instructions, 4);

	/* TODO: it should backup mr3, mr4 */
	aice_read_reg(MR3, &value_mr3);
	aice_read_reg(MR4, &value_mr4);

	access_page_size = value_mr4 & 0xF;
	if (0 == access_page_size) { /* 4K page */
		virtual_offset = virtual_address & 0x00000FFF;
		physical_page_number = value_mr3 & 0xFFFFF000;
	} else if (1 == access_page_size) { /* 8K page */
		virtual_offset = virtual_address & 0x00001FFF;
		physical_page_number = value_mr3 & 0xFFFFE000;
	} else if (5 == access_page_size) { /* 1M page */
		virtual_offset = virtual_address & 0x000FFFFF;
		physical_page_number = value_mr3 & 0xFFF00000;
	} else {
		return ERROR_FAIL;
	}

	*physical_address = physical_page_number | virtual_offset;

	return ERROR_OK;
}

static int aice_usb_init_cache(void)
{
	LOG_DEBUG("aice_usb_init_cache");

	uint32_t value_cr1;
	uint32_t value_cr2;

	aice_read_reg(CR1, &value_cr1);
	aice_read_reg(CR2, &value_cr2);

	icache.set = value_cr1 & 0x7;
	icache.log2_set = icache.set + 6;
	icache.set = 64 << icache.set;
	icache.way = ((value_cr1 >> 3) & 0x7) + 1;
	icache.line_size = (value_cr1 >> 6) & 0x7;
	if (icache.line_size != 0) {
		icache.log2_line_size = icache.line_size + 2;
		icache.line_size = 8 << (icache.line_size - 1);
	} else {
		icache.log2_line_size = 0;
	}

	LOG_DEBUG("\ticache set: %d, way: %d, line size: %d, "
			"log2(set): %d, log2(line_size): %d",
			icache.set, icache.way, icache.line_size,
			icache.log2_set, icache.log2_line_size);

	dcache.set = value_cr2 & 0x7;
	dcache.log2_set = dcache.set + 6;
	dcache.set = 64 << dcache.set;
	dcache.way = ((value_cr2 >> 3) & 0x7) + 1;
	dcache.line_size = (value_cr2 >> 6) & 0x7;
	if (dcache.line_size != 0) {
		dcache.log2_line_size = dcache.line_size + 2;
		dcache.line_size = 8 << (dcache.line_size - 1);
	} else {
		dcache.log2_line_size = 0;
	}

	LOG_DEBUG("\tdcache set: %d, way: %d, line size: %d, "
			"log2(set): %d, log2(line_size): %d",
			dcache.set, dcache.way, dcache.line_size,
			dcache.log2_set, dcache.log2_line_size);

	cache_init = true;

	return ERROR_OK;
}

static int aice_usb_dcache_inval_all(void)
{
	LOG_DEBUG("aice_usb_dcache_inval_all");

	uint32_t set_index;
	uint32_t way_index;
	uint32_t cache_index;
	uint32_t instructions[4];

	instructions[0] = MFSR_DTR(R0);
	instructions[1] = L1D_IX_INVAL(R0);
	instructions[2] = DSB;
	instructions[3] = BEQ_MINUS_12;

	for (set_index = 0; set_index < dcache.set; set_index++) {
		for (way_index = 0; way_index < dcache.way; way_index++) {
			cache_index = (way_index << (dcache.log2_set + dcache.log2_line_size)) |
				(set_index << dcache.log2_line_size);

			if (ERROR_OK != aice_write_dtr(current_target_id, cache_index))
				return ERROR_FAIL;

			if (ERROR_OK != aice_execute_dim(instructions, 4))
				return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int aice_usb_dcache_va_inval(uint32_t address)
{
	LOG_DEBUG("aice_usb_dcache_va_inval");

	uint32_t instructions[4];

	aice_write_dtr(current_target_id, address);

	instructions[0] = MFSR_DTR(R0);
	instructions[1] = L1D_VA_INVAL(R0);
	instructions[2] = DSB;
	instructions[3] = BEQ_MINUS_12;

	return aice_execute_dim(instructions, 4);
}

static int aice_usb_dcache_wb_all(void)
{
	LOG_DEBUG("aice_usb_dcache_wb_all");

	uint32_t set_index;
	uint32_t way_index;
	uint32_t cache_index;
	uint32_t instructions[4];

	instructions[0] = MFSR_DTR(R0);
	instructions[1] = L1D_IX_WB(R0);
	instructions[2] = DSB;
	instructions[3] = BEQ_MINUS_12;

	for (set_index = 0; set_index < dcache.set; set_index++) {
		for (way_index = 0; way_index < dcache.way; way_index++) {
			cache_index = (way_index << (dcache.log2_set + dcache.log2_line_size)) |
				(set_index << dcache.log2_line_size);

			if (ERROR_OK != aice_write_dtr(current_target_id, cache_index))
				return ERROR_FAIL;

			if (ERROR_OK != aice_execute_dim(instructions, 4))
				return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int aice_usb_dcache_va_wb(uint32_t address)
{
	LOG_DEBUG("aice_usb_dcache_va_wb");

	uint32_t instructions[4];

	aice_write_dtr(current_target_id, address);

	instructions[0] = MFSR_DTR(R0);
	instructions[1] = L1D_VA_WB(R0);
	instructions[2] = DSB;
	instructions[3] = BEQ_MINUS_12;

	return aice_execute_dim(instructions, 4);
}

static int aice_usb_icache_inval_all(void)
{
	LOG_DEBUG("aice_usb_icache_inval_all");

	uint32_t set_index;
	uint32_t way_index;
	uint32_t cache_index;
	uint32_t instructions[4];

	instructions[0] = MFSR_DTR(R0);
	instructions[1] = L1I_IX_INVAL(R0);
	instructions[2] = ISB;
	instructions[3] = BEQ_MINUS_12;

	for (set_index = 0; set_index < icache.set; set_index++) {
		for (way_index = 0; way_index < icache.way; way_index++) {
			cache_index = (way_index << (icache.log2_set + icache.log2_line_size)) |
				(set_index << icache.log2_line_size);

			if (ERROR_OK != aice_write_dtr(current_target_id, cache_index))
				return ERROR_FAIL;

			if (ERROR_OK != aice_execute_dim(instructions, 4))
				return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int aice_usb_icache_va_inval(uint32_t address)
{
	LOG_DEBUG("aice_usb_icache_va_inval");

	uint32_t instructions[4];

	aice_write_dtr(current_target_id, address);

	instructions[0] = MFSR_DTR(R0);
	instructions[1] = L1I_VA_INVAL(R0);
	instructions[2] = ISB;
	instructions[3] = BEQ_MINUS_12;

	return aice_execute_dim(instructions, 4);
}

static int aice_usb_cache_ctl(uint32_t subtype, uint32_t address)
{
	LOG_DEBUG("aice_usb_cache_ctl");

	int result;

	if (cache_init == false)
		aice_usb_init_cache();

	switch (subtype) {
		case AICE_CACHE_CTL_L1D_INVALALL:
			result = aice_usb_dcache_inval_all();
			break;
		case AICE_CACHE_CTL_L1D_VA_INVAL:
			result = aice_usb_dcache_va_inval(address);
			break;
		case AICE_CACHE_CTL_L1D_WBALL:
			result = aice_usb_dcache_wb_all();
			break;
		case AICE_CACHE_CTL_L1D_VA_WB:
			result = aice_usb_dcache_va_wb(address);
			break;
		case AICE_CACHE_CTL_L1I_INVALALL:
			result = aice_usb_icache_inval_all();
			break;
		case AICE_CACHE_CTL_L1I_VA_INVAL:
			result = aice_usb_icache_va_inval(address);
			break;
		default:
			result = ERROR_FAIL;
			break;
	}

	return result;
}

static int aice_usb_set_retry_times(uint32_t a_retry_times)
{
	aice_max_retry_times = a_retry_times;
	return ERROR_OK;
}

static int aice_usb_program_edm(char *command_sequence)
{
	char *command_str;
	char *reg_name_0;
	char *reg_name_1;
	uint32_t data_value;
	int i;

	/* init strtok() */
	command_str = strtok(command_sequence, ";");
	if (command_str == NULL)
		return ERROR_OK;

	do {
		i = 0;
		/* process one command */
		while (command_str[i] == ' ' ||
				command_str[i] == '\n' ||
				command_str[i] == '\r' ||
				command_str[i] == '\t')
			i++;

		/* skip ' ', '\r', '\n', '\t' */
		command_str = command_str + i;

		if (strncmp(command_str, "write_misc", 10) == 0) {
			reg_name_0 = strstr(command_str, "gen_port0");
			reg_name_1 = strstr(command_str, "gen_port1");

			if (reg_name_0 != NULL) {
				data_value = strtoul(reg_name_0 + 9, NULL, 0);

				if (aice_write_misc(current_target_id,
							NDS_EDM_MISC_GEN_PORT0, data_value) != ERROR_OK)
					return ERROR_FAIL;

			} else if (reg_name_1 != NULL) {
				data_value = strtoul(reg_name_1 + 9, NULL, 0);

				if (aice_write_misc(current_target_id,
							NDS_EDM_MISC_GEN_PORT1, data_value) != ERROR_OK)
					return ERROR_FAIL;
			} else {
				LOG_ERROR("program EDM, unsupported misc register: %s", command_str);
			}
		} else {
			LOG_ERROR("program EDM, unsupported command: %s", command_str);
		}

		/* update command_str */
		command_str = strtok(NULL, ";");

	} while (command_str != NULL);

	return ERROR_OK;
}

static int aice_usb_pack_command(bool enable_pack_command)
{
	if (enable_pack_command == false) {
		/* turn off usb_pack_command, flush usb_packets_buffer */
		aice_usb_packet_flush();
	}

	usb_pack_command = enable_pack_command;

	return ERROR_OK;
}

static int aice_usb_execute(uint32_t *instructions, uint32_t instruction_num)
{
	uint32_t i, j;
	uint8_t current_instruction_num;
	uint32_t dim_instructions[4] = {NOP, NOP, NOP, BEQ_MINUS_12};

	/* To execute 4 instructions as a special case */
	if (instruction_num == 4)
		return aice_execute_dim(instructions, 4);

	for (i = 0 ; i < instruction_num ; i += 3) {
		if (instruction_num - i < 3) {
			current_instruction_num = instruction_num - i;
			for (j = current_instruction_num ; j < 3 ; j++)
				dim_instructions[j] = NOP;
		} else {
			current_instruction_num = 3;
		}

		memcpy(dim_instructions, instructions + i,
				current_instruction_num * sizeof(uint32_t));

		/** fill DIM */
		if (aice_write_dim(current_target_id,
					dim_instructions,
					4) != ERROR_OK)
			return ERROR_FAIL;

		/** clear DBGER.DPED */
		if (aice_write_misc(current_target_id,
					NDS_EDM_MISC_DBGER, NDS_DBGER_DPED) != ERROR_OK)
			return ERROR_FAIL;

		/** execute DIM */
		if (aice_do_execute(current_target_id) != ERROR_OK)
			return ERROR_FAIL;

		/** check DBGER.DPED */
		if (aice_check_dbger(NDS_DBGER_DPED) != ERROR_OK) {

			LOG_ERROR("<-- TARGET ERROR! Debug operations do not finish properly:"
					"0x%08x 0x%08x 0x%08x 0x%08x. -->",
					dim_instructions[0],
					dim_instructions[1],
					dim_instructions[2],
					dim_instructions[3]);
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int aice_usb_set_custom_srst_script(const char *script)
{
	custom_srst_script = strdup(script);

	return ERROR_OK;
}

static int aice_usb_set_custom_trst_script(const char *script)
{
	custom_trst_script = strdup(script);

	return ERROR_OK;
}

static int aice_usb_set_custom_restart_script(const char *script)
{
	custom_restart_script = strdup(script);

	return ERROR_OK;
}

static int aice_usb_set_count_to_check_dbger(uint32_t count_to_check)
{
	aice_count_to_check_dbger = count_to_check;

	return ERROR_OK;
}

static int aice_usb_set_data_endian(enum aice_target_endian target_data_endian)
{
	data_endian = target_data_endian;

	return ERROR_OK;
}

/** */
struct aice_port_api_s aice_usb_api = {
	/** */
	.open = aice_open_device,
	/** */
	.close = aice_usb_close,
	/** */
	.idcode = aice_usb_idcode,
	/** */
	.state = aice_usb_state,
	/** */
	.reset = aice_usb_reset,
	/** */
	.assert_srst = aice_usb_assert_srst,
	/** */
	.run = aice_usb_run,
	/** */
	.halt = aice_usb_halt,
	/** */
	.step = aice_usb_step,
	/** */
	.read_reg = aice_usb_read_reg,
	/** */
	.write_reg = aice_usb_write_reg,
	/** */
	.read_reg_64 = aice_usb_read_reg_64,
	/** */
	.write_reg_64 = aice_usb_write_reg_64,
	/** */
	.read_mem_unit = aice_usb_read_memory_unit,
	/** */
	.write_mem_unit = aice_usb_write_memory_unit,
	/** */
	.read_mem_bulk = aice_usb_bulk_read_mem,
	/** */
	.write_mem_bulk = aice_usb_bulk_write_mem,
	/** */
	.read_debug_reg = aice_usb_read_debug_reg,
	/** */
	.write_debug_reg = aice_usb_write_debug_reg,
	/** */
	.set_jtag_clock = aice_usb_set_jtag_clock,
	/** */
	.select_target = aice_usb_select_target,
	/** */
	.memory_access = aice_usb_memory_access,
	/** */
	.memory_mode = aice_usb_memory_mode,
	/** */
	.read_tlb = aice_usb_read_tlb,
	/** */
	.cache_ctl = aice_usb_cache_ctl,
	/** */
	.set_retry_times = aice_usb_set_retry_times,
	/** */
	.program_edm = aice_usb_program_edm,
	/** */
	.pack_command = aice_usb_pack_command,
	/** */
	.execute = aice_usb_execute,
	/** */
	.set_custom_srst_script = aice_usb_set_custom_srst_script,
	/** */
	.set_custom_trst_script = aice_usb_set_custom_trst_script,
	/** */
	.set_custom_restart_script = aice_usb_set_custom_restart_script,
	/** */
	.set_count_to_check_dbger = aice_usb_set_count_to_check_dbger,
	/** */
	.set_data_endian = aice_usb_set_data_endian,
};
