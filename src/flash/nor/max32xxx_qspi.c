// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2016 - 2019 by Andreas Bolsch                           *
 *   andreas.bolsch@mni.thm.de                                             *
 *                                                                         *
 *   Copyright (C) 2010 by Antonio Borneo                                  *
 *   borneo.antonio@gmail.com                                              *
 *                                                                         *
 *   Portions Copyright (C) 2025-2026 Analog Devices, Inc.                 *
 *   pete.johanson@analog.com                                              *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or	   *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include "max32xxx_options.h"
#include "max32xxx_qspi_common_defs.h"
#include <helper/binarybuffer.h>
#include <helper/bits.h>
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include <target/image.h>
#include "spi.h"
#include "sfdp.h"

#define SPIXFC_FIFO_DEPTH 16
// 16-bit filler magic word indicating this isn't a header
#define SPIXFC_HEADER_NULL 0xF000

#define SPIXF_BASE 0x40026000
#define SPIXF_CFG (SPIXF_BASE | 0x00)
#define SPIXF_FETCH_CTRL (SPIXF_BASE | 0x04)
#define SPIXF_MODE_CTRL (SPIXF_BASE | 0x08)
#define SPIXF_MODE_DATA (SPIXF_BASE | 0x0C)
#define SPIXF_SCLK_FB_CTRL (SPIXF_BASE | 0x10)
#define SPIXF_IO_CTRL (SPIXF_BASE | 0x1C)
#define SPIXF_MEMSECCN (SPIXF_BASE | 0x20)
#define SPIXF_BUS_IDLE (SPIXF_BASE | 0x24)

#define SPIXF_MEMSECCN_ENC_ENABLE 0x1
#define SPIXF_MEMSECCN_AUTH_DISABLE 0x2

#define SPI_ICC_BASE 0x4002F000
#define SPI_ICC_CTRL (SPI_ICC_BASE | 0x100)
#define SPI_ICC_INV (SPI_ICC_BASE | 0x700)

#define SPI_ICC_CTRL_EN_POS 0
#define SPI_ICC_CTRL_EN (0x1UL << SPI_ICC_CTRL_EN_POS)
#define SPI_ICC_CTRL_RDY_POS 16
#define SPI_ICC_CTRL_RDY (0x1UL << SPI_ICC_CTRL_RDY_POS)

#define GCR_BASE 0x40000000
#define GCR_SCON (GCR_BASE | 0x00)
#define GCR_RST1 (GCR_BASE | 0x44)
#define GCR_RST1_XSPIM (0x1 << 4)
#define GCR_RST1_SPIXIP (0x1 << 5)

// Set the number of system clocks per low/high period of the SPI clock
#define SPI_CLOCK_PERIOD 2

// Flash timeout values in milliseconds.
#define MAX32XXX_QSPI_MASS_ERASE_TIMEOUT_MS 100000
#define MAX32XXX_QSPI_ERASE_TIMEOUT_MS 10000
#define MAX32XXX_QSPI_WE_TIMEOUT_MS 100
#define MAX32XXX_QSPI_FIFO_TIMEOUT_MS 100

// Set this to 1 to enable dual (1-2-2) reads if available from SFDP
#ifndef SPI_DUAL_MODE
#define SPI_DUAL_MODE 0
#endif

static const uint8_t write_code[] = {
#include "contrib/loaders/flash/max32xxx_qspi/max32xxx_qspi.inc"
};

struct max32xxx_qspi_flash_bank {
	bool probed;
	char devname[32];
	struct flash_device dev;
	unsigned int options;
};

FLASH_BANK_COMMAND_HANDLER(max32xxx_qspi_flash_bank_command)
{
	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC < 7 || CMD_ARGC > 7) {
		LOG_ERROR("incorrect flash bank max32xxx_qspi configuration: "
				  "<flash_addr_base> <flash_addr_size> 0 0 <target> <opitons>");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	struct max32xxx_qspi_flash_bank *info;
	info = malloc(sizeof(struct max32xxx_qspi_flash_bank));
	if (!info) {
		LOG_ERROR("not enough memory");
		return ERROR_FAIL;
	}

	bank->driver_priv = info;

	info->probed = false;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[6], info->options);
	bank->driver_priv = info;

	return ERROR_OK;
}

static int max32xxx_qspi_pre_op(struct flash_bank *bank)
{
	struct target *target = bank->target;
	// Set the number of system clocks for the SPI clock low and high period
	int retval = target_write_u32(target, SPIXFC_CFG, (SPI_CLOCK_PERIOD << 8) | (SPI_CLOCK_PERIOD << 12));
	if (retval != ERROR_OK)
		return retval;

	// Enable the peripheral, FIFOs and SCK feedback
	retval = target_write_u32(target, SPIXFC_GEN_CTRL, (0x7 << 0) | (0x1 << 5) | (0x1 << 24));
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int max32xxx_qspi_post_op(struct flash_bank *bank)
{
	struct target *target = bank->target;
	// Disable the SPI ICC
	int retval = target_write_u32(target, SPI_ICC_CTRL, 0);
	if (retval != ERROR_OK)
		return retval;

	// Disable SPIXFC
	retval = target_write_u32(target, SPIXFC_GEN_CTRL, 0);
	if (retval != ERROR_OK)
		return retval;

	// Reset SPI peripherals
	retval = target_write_u32(target, GCR_RST1, GCR_RST1_XSPIM | GCR_RST1_SPIXIP);
	if (retval != ERROR_OK)
		return retval;

	// Wait for reset to complete
	uint32_t temp32 = 1;
	while (temp32) {
		retval = target_read_u32(target, GCR_RST1, &temp32);
		if (retval != ERROR_OK)
			return retval;
	}

	// Invalidate cache
	retval = target_write_u32(target, SPI_ICC_INV, 1);
	if (retval != ERROR_OK)
		return retval;

	// Wait for the ready bit
	temp32 = 0;
	while (!temp32) {
		retval = target_read_u32(target, SPI_ICC_CTRL, &temp32);
		if (retval != ERROR_OK)
			return retval;
		temp32 &= SPI_ICC_CTRL_RDY;
	}

	// Set the number of system clocks for the SPI clock low and high period
	retval = target_write_u32(target, SPIXF_CFG, (SPI_CLOCK_PERIOD << 8) | (SPI_CLOCK_PERIOD << 12) | (0x1 << 2));
	if (retval != ERROR_OK)
		return retval;

	struct max32xxx_qspi_flash_bank *max32xxx_qspi_info = bank->driver_priv;
	// Enter 1-2-2 mode
	if (SPI_DUAL_MODE && max32xxx_qspi_info->dev.dread_cmd != 0x0) {
		LOG_DEBUG("Entering 1-2-2 read mode");

		// Set the read command
		retval = target_write_u32(target, SPIXF_FETCH_CTRL,
				(0x1 << 10) | (0x1 << 12) | max32xxx_qspi_info->dev.dread_cmd);
		if (retval != ERROR_OK)
			return retval;

		// Set mode control
		retval = target_write_u32(target, SPIXF_MODE_CTRL,
				max32xxx_qspi_info->dev.dread_mode + max32xxx_qspi_info->dev.dread_dclk);
		if (retval != ERROR_OK)
			return retval;
	} else {
		// Set the read command
		temp32 = max32xxx_qspi_info->dev.read_cmd;
		// Enable 4 byte addresses if using read command 0x13
		if (max32xxx_qspi_info->dev.read_cmd == 0x13)
			temp32 |= 0x1 << 16;
		retval = target_write_u32(target, SPIXF_FETCH_CTRL, temp32);
		if (retval != ERROR_OK)
			return retval;

		// Set mode control
		retval = target_write_u32(target, SPIXF_MODE_CTRL, 0);
		if (retval != ERROR_OK)
			return retval;
	}

	// Setup the encryption options
	if (max32xxx_qspi_info->options & OPTIONS_ENC) {
		temp32 = SPIXF_MEMSECCN_ENC_ENABLE;

		if (!(max32xxx_qspi_info->options & OPTIONS_AUTH))
			temp32 |= SPIXF_MEMSECCN_AUTH_DISABLE;
		retval = target_write_u32(target, SPIXF_MEMSECCN, temp32);
		if (retval != ERROR_OK)
			return retval;
	} else {
		retval = target_write_u32(target, SPIXF_MEMSECCN, 0);
		if (retval != ERROR_OK)
			return retval;
	}

	// Enable feedback mode
	retval = target_write_u32(target, SPIXF_SCLK_FB_CTRL, 1);
	if (retval != ERROR_OK)
		return retval;

	// Bus idle timeout
	retval = target_write_u32(target, SPIXF_BUS_IDLE, 1);
	if (retval != ERROR_OK)
		return retval;

	// Enable cache
	retval = target_write_u32(target, SPI_ICC_CTRL, SPI_ICC_CTRL_EN);
	if (retval != ERROR_OK)
		return retval;

	// Clear the code cache
	retval = target_read_u32(target, GCR_SCON, &temp32);
	if (retval != ERROR_OK)
		return retval;
	temp32 |= (0x1 << 6);
	retval = target_write_u32(target, GCR_SCON, temp32);
	if (retval != ERROR_OK)
		return retval;

	return retval;
}

static int max32xxx_qspi_write_txfifo(struct target *target,
		const uint8_t *data, unsigned int len)
{
	int retval = ERROR_OK;
	unsigned int data_i = 0;

	while (len - data_i) {
		unsigned int write_len;

		// Calculate how many bytes we can write on this round
		if (len - data_i > SPIXFC_FIFO_DEPTH)
			write_len = SPIXFC_FIFO_DEPTH;
		else
			write_len = len - data_i;

		// Wait for there to be room in the TX FIFO
		const int64_t start_time = timeval_ms();
		unsigned int tx_fifo_avail;
		do {
			uint32_t temp32;
			retval = target_read_u32(target, SPIXFC_FIFO_CTRL, &temp32);
			if (retval != ERROR_OK)
				return retval;
			tx_fifo_avail = SPIXFC_FIFO_DEPTH -
							((temp32 & SPIXFC_FIFO_CTRL_TX_FIFO_CNT) >>
									SPIXFC_FIFO_CTRL_TX_FIFO_CNT_POS);

			if (timeval_ms() - start_time > MAX32XXX_QSPI_FIFO_TIMEOUT_MS)
				return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

			keep_alive();
		} while (tx_fifo_avail < write_len);

		while (write_len) {
			uint16_t write_data = data[data_i++];

			if (data_i < len) {
				write_data |= (data[data_i++] << 8);
				write_len -= 2;
			} else {
				write_data |= SPIXFC_HEADER_NULL;
				write_len -= 1;
			}
			retval = target_write_u16(target, SPIXFC_FIFO_TX, write_data);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	return retval;
}

static int max32xxx_qspi_read_rxfifo(struct target *target, uint8_t *data,
		unsigned int len)
{
	int retval = ERROR_OK;
	unsigned int data_i = 0;

	while (len - data_i) {
		unsigned int read_len;

		// Wait for there to be data in the RX FIFO
		const int64_t start_time = timeval_ms();
		unsigned int rx_fifo_avail;
		do {
			uint32_t temp32;
			retval = target_read_u32(target, SPIXFC_FIFO_CTRL, &temp32);
			if (retval != ERROR_OK)
				return retval;
			rx_fifo_avail = (temp32 & SPIXFC_FIFO_CTRL_RX_FIFO_CNT) >>
							SPIXFC_FIFO_CTRL_RX_FIFO_CNT_POS;

			if (timeval_ms() - start_time > MAX32XXX_QSPI_FIFO_TIMEOUT_MS)
				return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

			keep_alive();
		} while (!rx_fifo_avail);

		// Calculate how many bytes we can write on this round
		if (len - data_i > rx_fifo_avail)
			read_len = rx_fifo_avail;
		else
			read_len = len - data_i;

		while (read_len) {
			retval = target_read_u8(target, SPIXFC_FIFO_RX, &data[data_i++]);
			if (retval != ERROR_OK)
				return retval;
			read_len--;
		}
	}

	return retval;
}

static int max32xxx_qspi_write_bytes(struct target *target, const uint8_t *data,
		unsigned int len, bool deass)
{
	int retval = ERROR_OK;
	unsigned int data_i = 0;

	// Wrap the length
	while (len - data_i) {
		unsigned int chunk_len;
		// Max transaction length is 32 units
		if (len - data_i > 32)
			chunk_len = 32;
		else
			chunk_len = len - data_i;

		// Setup the SPI header, 32 maps to 0 in the size field
		uint16_t header = SPIXFC_HEADER_TX | SPIXFC_HEADER_BYTE;
		if (chunk_len != 32)
			header |= chunk_len << SPIXFC_HEADER_SIZE_POS;

		// If we de-asserting and this is the final chunk
		if (deass && (len - data_i - chunk_len == 0))
			header |= SPIXFC_HEADER_SS_DEASS;

		// Write the header to the TX FIFO
		retval = max32xxx_qspi_write_txfifo(target, (uint8_t *)&header,
				sizeof(header));
		if (retval != ERROR_OK)
			return retval;

		// Write the data to the TX FIFO
		retval = max32xxx_qspi_write_txfifo(target, &data[data_i], chunk_len);
		if (retval != ERROR_OK)
			return retval;
		data_i += chunk_len;
	}

	return retval;
}

static int max32xxx_qspi_read_bytes(struct target *target, uint8_t *data,
		unsigned int len, bool deass)
{
	int retval = ERROR_OK;
	unsigned int data_i = 0;

	// Wrap the length
	while (len - data_i) {
		unsigned int chunk_len;
		// Max transaction length is 32 units
		if (len - data_i > 32)
			chunk_len = 32;
		else
			chunk_len = len - data_i;

		// Setup the SPI header, 32 maps to 0 in the size field
		uint16_t header = SPIXFC_HEADER_RX | SPIXFC_HEADER_BYTE;
		if (chunk_len != 32)
			header |= chunk_len << SPIXFC_HEADER_SIZE_POS;

		// If we de-asserting and this is the final chunk
		if (deass && (len - data_i - chunk_len == 0))
			header |= SPIXFC_HEADER_SS_DEASS;

		// Write the header to the TX FIFO
		retval = max32xxx_qspi_write_txfifo(target, (uint8_t *)&header,
				sizeof(header));
		if (retval != ERROR_OK)
			return retval;

		// Read the data to the TX FIFO, convert to number of bytes
		retval = max32xxx_qspi_read_rxfifo(target, (uint8_t *)&data[data_i],
				chunk_len);
		if (retval != ERROR_OK)
			return retval;
		data_i += chunk_len;
	}

	return retval;
}

static int max32xxx_qspi_read_words(struct target *target, uint32_t *data,
		unsigned int len, bool deass)
{
	uint32_t temp32;
	unsigned int data_i = 0;

	// Configure the page size
	int retval = target_read_u32(target, SPIXFC_CFG, &temp32);
	if (retval != ERROR_OK)
		return retval;
	temp32 = (temp32 & ~(SPIXFC_CONFIG_PAGE_SIZE)) |
			 SPIXFC_CONFIG_PAGE_SIZE_4_BYTES;
	retval = target_write_u32(target, SPIXFC_CFG, temp32);
	if (retval != ERROR_OK)
		return retval;

	while (len - data_i) {
		unsigned int chunk_len;
		// Max transaction length is 32 units
		if (len - data_i > 32)
			chunk_len = 32;
		else
			chunk_len = len - data_i;

		// Setup the SPI header
		uint16_t header = SPIXFC_HEADER_RX | SPIXFC_HEADER_PAGE;
		if (chunk_len != 32)
			header |= chunk_len << SPIXFC_HEADER_SIZE_POS;

		// If we de-asserting and this is the final chunk
		if (deass && (len - data_i - chunk_len == 0))
			header |= SPIXFC_HEADER_SS_DEASS;

		// Write the header to the TX FIFO
		retval = max32xxx_qspi_write_txfifo(target, (uint8_t *)&header,
				sizeof(header));
		if (retval != ERROR_OK)
			return retval;

		// Read the data to the TX FIFO, convert to number of bytes
		uint8_t *data8 = (uint8_t *)data;
		retval = max32xxx_qspi_read_rxfifo(target,
				(uint8_t *)&data8[data_i * 4], chunk_len * 4);
		if (retval != ERROR_OK)
			return retval;
		data_i += chunk_len;
	}

	return retval;
}

static int max32xxx_qspi_poll_wip(struct target *target, unsigned int timeout)
{
	const int64_t start_time = timeval_ms();
	uint8_t read_data;
	int retval;

	do {
		uint8_t cmd_data = SPIFLASH_READ_STATUS;
		retval = max32xxx_qspi_write_bytes(target, &cmd_data, 1, false);
		if (retval != ERROR_OK)
			return retval;

		read_data = SPIFLASH_BSY_BIT;
		retval = max32xxx_qspi_read_bytes(target, &read_data, 1, true);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to read the status bytes");
			return retval;
		}

		// Prevent GDB warnings
		keep_alive();

		if (timeval_ms() - start_time > timeout) {
			LOG_ERROR("Timed out waiting for flash");
			return ERROR_TIMEOUT_REACHED;
		}

	} while (read_data & SPIFLASH_BSY_BIT);

	return ERROR_OK;
}

static int max32xxx_qspi_poll_we(struct target *target)
{
	const int64_t start_time = timeval_ms();
	uint8_t read_data;
	int retval;

	do {
		uint8_t cmd_data = SPIFLASH_READ_STATUS;
		retval = max32xxx_qspi_write_bytes(target, &cmd_data, 1, false);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to write the read status bytes");
			return retval;
		}

		read_data = 0;
		retval = max32xxx_qspi_read_bytes(target, &read_data, 1, true);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to read the status bytes");
			return retval;
		}

		// Prevent GDB warnings
		keep_alive();

		if (timeval_ms() - start_time > MAX32XXX_QSPI_WE_TIMEOUT_MS) {
			LOG_ERROR("Timed out waiting for write enable");
			return ERROR_TIMEOUT_REACHED;
		}

	} while (!(read_data & SPIFLASH_WE_BIT));

	return ERROR_OK;
}

static int max32xxx_qspi_set_we(struct target *target)
{
	uint8_t cmd_data = SPIFLASH_WRITE_ENABLE;

	// TODO: Could also be instruction 0x50
	int retval = max32xxx_qspi_write_bytes(target, &cmd_data, 1, true);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write the write-enable bytes");
		return retval;
	}

	return max32xxx_qspi_poll_we(target);
}

static int max32xxx_qspi_erase(struct flash_bank *bank, unsigned int first,
		unsigned int last)
{
	struct target *target = bank->target;
	struct max32xxx_qspi_flash_bank *max32xxx_qspi_info = bank->driver_priv;

	LOG_DEBUG("%s: first = %d last = %d\n", __func__, first, last);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int retval = max32xxx_qspi_pre_op(bank);
	if (retval != ERROR_OK)
		return retval;

	while (first <= last) {
		// Set the write enable
		retval = max32xxx_qspi_set_we(target);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to set the write-enable bit!");
			goto exit;
		}

		uint8_t cmd_data[5];
		// Send the erase command
		cmd_data[0] = max32xxx_qspi_info->dev.erase_cmd;
		// Address is MSB first
		uint32_t addr = (first++) * ((max32xxx_qspi_info)->dev.sectorsize);

		if (max32xxx_qspi_info->dev.erase_cmd == 0xdc) {
			cmd_data[4] = (addr & 0x000000FF) >> 0;
			cmd_data[3] = (addr & 0x0000FF00) >> 8;
			cmd_data[2] = (addr & 0x00FF0000) >> 16;
			cmd_data[1] = (addr & 0xFF000000) >> 24;
			retval = max32xxx_qspi_write_bytes(target, cmd_data, 5, true);
		} else {
			cmd_data[3] = (addr & 0x0000FF) >> 0;
			cmd_data[2] = (addr & 0x00FF00) >> 8;
			cmd_data[1] = (addr & 0xFF0000) >> 16;
			retval = max32xxx_qspi_write_bytes(target, cmd_data, 4, true);
		}

		if (retval != ERROR_OK)
			goto exit;

		// Poll WIP until erase is complete
		retval = max32xxx_qspi_poll_wip(target, MAX32XXX_QSPI_ERASE_TIMEOUT_MS);
		if (retval != ERROR_OK) {
			LOG_ERROR("Flash bank didn't respond to status request after erase");
			goto exit;
		}
	}

exit:
	{
		int post_retval = max32xxx_qspi_post_op(bank);
		if (retval == ERROR_OK)
			retval = post_retval;
	}

	return retval;
}

static int max32xxx_qspi_write_block(struct flash_bank *bank,
		const uint8_t *buffer, uint32_t offset, uint32_t len)
{
	struct target *target = bank->target;
	struct working_area *write_algorithm;

	LOG_DEBUG("max32xxx_write_block bank=%p buffer=%p offset=%08" PRIx32
			  " len=%08" PRIx32 "",
			bank, buffer, offset, len);

	// flash write code
	int retval = target_alloc_working_area(target, sizeof(write_code),
			&write_algorithm);
	if (retval != ERROR_OK)
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;

	// memory buffer
	uint32_t buffer_size = target_get_working_area_avail(target);
	struct working_area *source;
	const uint32_t algo_overhead = SPIX_ALGO_TAIL_SIZE;
	const uint32_t minimal_buffer_size = algo_overhead + 256;

	if (buffer_size > 16384)
		buffer_size = 16384;

	if (buffer_size <= minimal_buffer_size) {
		target_free_working_area(target, write_algorithm);
		LOG_WARNING("Insufficient working area for block write algorithm");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_alloc_working_area(target, buffer_size, &source);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	retval = target_write_buffer(target, write_algorithm->address,
			sizeof(write_code), write_code);
	if (retval != ERROR_OK) {
		target_free_working_area(target, write_algorithm);
		target_free_working_area(target, source);
		return retval;
	}

	struct armv7m_algorithm armv7m_info;
	armv7m_info.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_info.core_mode = ARM_MODE_THREAD;

	struct reg_param reg_params[5];
	init_reg_param(&reg_params[0], "r0", 32, PARAM_OUT);
	init_reg_param(&reg_params[1], "r1", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "r2", 32, PARAM_OUT);
	init_reg_param(&reg_params[3], "r3", 32, PARAM_OUT);
	init_reg_param(&reg_params[4], "sp", 32, PARAM_OUT);

	buf_set_u32(reg_params[0].value, 0, 32, source->address);
	buf_set_u32(reg_params[1].value, 0, 32, source->address + source->size);
	buf_set_u32(reg_params[2].value, 0, 32, len);
	buf_set_u32(reg_params[3].value, 0, 32, offset);
	buf_set_u32(reg_params[4].value, 0, 32, source->address + source->size);

	/* mem_params for options and command live in the shared tail layout. */
	struct mem_param mem_param[2];
	const uint32_t tail_base = source->address + source->size - SPIX_ALGO_TAIL_SIZE;
	init_mem_param(&mem_param[0],
			tail_base + SPIX_ALGO_OPTIONS_OFFSET, 4,
			PARAM_OUT);
	init_mem_param(&mem_param[1],
			tail_base + SPIX_ALGO_SPI_WRITE_CMD_OFFSET, 4,
			PARAM_OUT);
	struct max32xxx_qspi_flash_bank *max32xxx_qspi_info = bank->driver_priv;
	buf_set_u32(mem_param[0].value, 0, 32, max32xxx_qspi_info->options);
	buf_set_u32(mem_param[1].value, 0, 32, max32xxx_qspi_info->dev.pprog_cmd);

	LOG_DEBUG("max32xxx_write_block source->address=" TARGET_ADDR_FMT
			  " source->size=0x%08" PRIx32,
			source->address, source->size);

	const uint32_t source_data_size = source->size - algo_overhead;
	const uint32_t algo_end = write_algorithm->address + sizeof(write_code) - 2;

	retval = target_run_flash_async_algorithm(target,
			buffer, len, 1,
			ARRAY_SIZE(mem_param), mem_param,
			ARRAY_SIZE(reg_params), reg_params,
			source->address, source_data_size,
			write_algorithm->address, algo_end,
			&armv7m_info);

	if (retval == ERROR_FLASH_OPERATION_FAILED)
		LOG_ERROR("error executing max32xxx qspi write algorithm at " TARGET_ADDR_FMT,
				write_algorithm->address);

	target_free_working_area(target, write_algorithm);
	target_free_working_area(target, source);
	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);
	destroy_reg_param(&reg_params[3]);
	destroy_reg_param(&reg_params[4]);

	return retval;
}

static int max32xxx_qspi_write(struct flash_bank *bank, const uint8_t *buffer,
		uint32_t offset, uint32_t count)
{
	struct target *target = bank->target;

	LOG_DEBUG("%s: offset=0x%08" PRIx32 " count=0x%08" PRIx32, __func__, offset,
			count);

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	struct max32xxx_qspi_flash_bank *max32xxx_qspi_info = bank->driver_priv;

	if (offset + count > bank->size) {
		LOG_ERROR("Write beyond end of flash.");
		return ERROR_FLASH_DST_OUT_OF_BANK;
	}

	int retval = ERROR_OK;
	// Determine if we want to use the on-chip algorithm
	if (max32xxx_qspi_info->options & OPTIONS_ENC || count > 16) {
		if (max32xxx_qspi_info->options & OPTIONS_AUTH) {
			/* Need to erase extra length if we're writing authentication data
			 */
			uint32_t max_sector_plain =
					(offset + count) / max32xxx_qspi_info->dev.sectorsize;
			uint32_t max_sector_auth = (offset + count * 10 / 8) /
									   max32xxx_qspi_info->dev.sectorsize;
			if (max_sector_auth > max_sector_plain) {
				LOG_WARNING("Erasing extra flash for authentication data");
				retval = max32xxx_qspi_erase(bank, max_sector_plain,
						max_sector_auth);
				if (retval != ERROR_OK)
					return retval;
			}
		}

		retval = max32xxx_qspi_pre_op(bank);
		if (retval != ERROR_OK)
			return retval;

		retval = max32xxx_qspi_write_block(bank, buffer, offset, count);

		if (retval != ERROR_OK) {
			if (retval == ERROR_TARGET_RESOURCE_NOT_AVAILABLE) {
				if (max32xxx_qspi_info->options & OPTIONS_ENC) {
					LOG_ERROR("Must use algorithm in working area for "
							  "encryption");
					goto exit;
				}
				LOG_DEBUG("working area algorithm not available");
			} else {
				LOG_ERROR("Error with flash algorithm");
				goto exit;
			}
		} else {
			goto exit;
		}
	} else {
		retval = max32xxx_qspi_pre_op(bank);
		if (retval != ERROR_OK)
			return retval;
	}

	// Send the page program command
	uint8_t cmd_data[5];
	cmd_data[0] = max32xxx_qspi_info->dev.pprog_cmd;

	unsigned int buffer_i = 0;

	while (count - buffer_i) {
		// Set the write enable
		retval = max32xxx_qspi_set_we(target);
		if (retval != ERROR_OK)
			goto exit;

		unsigned int write_len = SPI_WRITE_BOUNDARY - (offset % SPI_WRITE_BOUNDARY);

		if (write_len > count - buffer_i)
			write_len = count - buffer_i;

		if (max32xxx_qspi_info->dev.pprog_cmd == 0x12) {
			cmd_data[4] = (offset & 0x000000FF) >> 0;
			cmd_data[3] = (offset & 0x0000FF00) >> 8;
			cmd_data[2] = (offset & 0x00FF0000) >> 16;
			cmd_data[1] = (offset & 0xFF000000) >> 24;
			// Write the command
			retval = max32xxx_qspi_write_bytes(target, cmd_data, 5, false);
		} else {
			// Address is MSB first
			cmd_data[3] = (offset & 0x0000FF) >> 0;
			cmd_data[2] = (offset & 0x00FF00) >> 8;
			cmd_data[1] = (offset & 0xFF0000) >> 16;
			// Write the command
			retval = max32xxx_qspi_write_bytes(target, cmd_data, 4, false);
		}

		if (retval != ERROR_OK)
			goto exit;

		// Write the data
		retval = max32xxx_qspi_write_bytes(target,
				&buffer[buffer_i], write_len, true);
		if (retval != ERROR_OK)
			goto exit;

		// Increment the pointers
		buffer_i += write_len;
		offset += write_len;
	}

exit:
	{
		int post_retval = max32xxx_qspi_post_op(bank);
		if (retval == ERROR_OK)
			retval = post_retval;
	}

	return retval;
}

static int read_sfdp_block(struct flash_bank *bank, uint32_t addr,
		uint32_t words, uint32_t *buffer)
{
	struct target *target = bank->target;
	uint8_t cmd_data[5];

	// Write the command
	cmd_data[0] = SPIFLASH_READ_SFDP;

	// Address is MSB first
	cmd_data[3] = (addr & 0x0000FF) >> 0;
	cmd_data[2] = (addr & 0x00FF00) >> 8;
	cmd_data[1] = (addr & 0xFF0000) >> 16;

	// 1 dummy bytes
	cmd_data[4] = 0;

	int retval = max32xxx_qspi_write_bytes(target, cmd_data, sizeof(cmd_data), false);
	if (retval != ERROR_OK)
		return retval;

	// Read the response, convert words to number of bytes
	retval = max32xxx_qspi_read_words(target, buffer, words, true);

	return retval;
}

static int max32xxx_qspi_probe(struct flash_bank *bank)
{
	struct max32xxx_qspi_flash_bank *max32xxx_qspi_info = bank->driver_priv;
	bool pre_op_done = false;

	LOG_DEBUG("%s", __func__);

	if (max32xxx_qspi_info->probed) {
		bank->size = 0;
		bank->num_sectors = 0;
		free(bank->sectors);
		bank->sectors = NULL;
		memset(&max32xxx_qspi_info->dev, 0, sizeof(max32xxx_qspi_info->dev));
		max32xxx_qspi_info->probed = false;
	}

	int retval = max32xxx_qspi_pre_op(bank);
	if (retval != ERROR_OK)
		return retval;
	pre_op_done = true;

	struct target *target = bank->target;
	uint32_t temp32;
	retval = target_read_u32(target, SPIXFC_CFG, &temp32);
	if (retval != ERROR_OK)
		goto exit;
	LOG_DEBUG("SPIXFC_CFG       = 0x%08X", temp32);

	retval = target_read_u32(target, SPIXFC_SS_POL, &temp32);
	if (retval != ERROR_OK)
		goto exit;
	LOG_DEBUG("SPIXFC_SS_POL    = 0x%08X", temp32);

	retval = target_read_u32(target, SPIXFC_GEN_CTRL, &temp32);
	if (retval != ERROR_OK)
		goto exit;
	LOG_DEBUG("SPIXFC_GEN_CTRL  = 0x%08X", temp32);

	retval = target_read_u32(target, SPIXFC_FIFO_CTRL, &temp32);
	if (retval != ERROR_OK)
		goto exit;
	LOG_DEBUG("SPIXFC_FIFO_CTRL = 0x%08X", temp32);

	// Read the SFDP settings from the flash device
	struct flash_device temp_flash_device;
	retval = spi_sfdp(bank, &temp_flash_device, &read_sfdp_block);
	if (retval != ERROR_OK)
		goto exit;
	LOG_INFO("max32xxx flash \'%s\' size = %" PRIu32 " kbytes",
			temp_flash_device.name, temp_flash_device.size_in_bytes / 1024);

	max32xxx_qspi_info->dev = temp_flash_device;

	// Read the device ID
	uint8_t cmd = SPIFLASH_READ_ID;
	retval = max32xxx_qspi_write_bytes(target, &cmd, 1, false);
	if (retval != ERROR_OK)
		goto exit;

	retval = max32xxx_qspi_read_bytes(target,
			(uint8_t *)&max32xxx_qspi_info->dev.device_id, 3, true);
	if (retval != ERROR_OK)
		goto exit;

	// Set correct size value
	bank->size = max32xxx_qspi_info->dev.size_in_bytes;
	bank->write_start_alignment =
			(max32xxx_qspi_info->options & OPTIONS_AUTH) ? 0x80 : 16;
	bank->write_end_alignment = bank->write_start_alignment;

	// TODO: Get more than 1 erase command

	// Create and fill sectors array
	bank->num_sectors = max32xxx_qspi_info->dev.size_in_bytes /
						max32xxx_qspi_info->dev.sectorsize;
	struct flash_sector *sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
	if (!sectors) {
		LOG_ERROR("not enough memory");
		retval = ERROR_FAIL;
		goto exit;
	}

	for (unsigned int sector = 0; sector < bank->num_sectors; sector++) {
		sectors[sector].offset = sector * max32xxx_qspi_info->dev.sectorsize;
		sectors[sector].size = max32xxx_qspi_info->dev.sectorsize;
		sectors[sector].is_erased = -1;
		sectors[sector].is_protected = 0;
	}

	bank->sectors = sectors;

	max32xxx_qspi_info->probed = true;

	// Setup memory mapped mode
	retval = max32xxx_qspi_post_op(bank);
	if (retval != ERROR_OK)
		goto exit;
	pre_op_done = false;

	retval = target_read_u32(target, SPIXF_CFG, &temp32);
	if (retval != ERROR_OK)
		goto exit;
	LOG_DEBUG("SPIXF_CFG			= 0x%08X", temp32);
	retval = target_read_u32(target, SPIXF_FETCH_CTRL, &temp32);
	if (retval != ERROR_OK)
		goto exit;
	LOG_DEBUG("SPIXF_FETCH_CTRL		= 0x%08X", temp32);
	retval = target_read_u32(target, SPIXF_MODE_CTRL, &temp32);
	if (retval != ERROR_OK)
		goto exit;
	LOG_DEBUG("SPIXF_MODE_CTRL		= 0x%08X", temp32);
	retval = target_read_u32(target, SPIXF_MODE_DATA, &temp32);
	if (retval != ERROR_OK)
		goto exit;
	LOG_DEBUG("SPIXF_MODE_DATA		= 0x%08X", temp32);
	retval = target_read_u32(target, SPIXF_SCLK_FB_CTRL, &temp32);
	if (retval != ERROR_OK)
		goto exit;
	LOG_DEBUG("SPIXF_SCLK_FB_CTRL	= 0x%08X", temp32);
	retval = target_read_u32(target, SPIXF_IO_CTRL, &temp32);
	if (retval != ERROR_OK)
		goto exit;
	LOG_DEBUG("SPIXF_IO_CTRL		= 0x%08X", temp32);
	retval = target_read_u32(target, SPIXF_MEMSECCN, &temp32);
	if (retval != ERROR_OK)
		goto exit;
	LOG_DEBUG("SPIXF_MEMSECCN		= 0x%08X", temp32);
	retval = target_read_u32(target, SPIXF_BUS_IDLE, &temp32);
	if (retval != ERROR_OK)
		goto exit;
	LOG_DEBUG("SPIXF_BUS_IDLE		= 0x%08X", temp32);

exit:
	if (pre_op_done) {
		int post_retval = max32xxx_qspi_post_op(bank);
		if (retval == ERROR_OK)
			retval = post_retval;
	}

	return retval;
}

static int max32xxx_qspi_auto_probe(struct flash_bank *bank)
{
	struct max32xxx_qspi_flash_bank *max32xxx_qspi_info = bank->driver_priv;

	if (max32xxx_qspi_info->probed)
		return ERROR_OK;
	return max32xxx_qspi_probe(bank);
}

COMMAND_HANDLER(max32xxx_qspi_handle_mem_mapping_setup_command)
{
	int retval;

	struct flash_bank *bank;
	retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	return max32xxx_qspi_post_op(bank);
}

COMMAND_HANDLER(max32xxx_qspi_handle_mass_erase_command)
{
	struct flash_bank *bank;

	LOG_DEBUG("%s", __func__);

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	int retval = CALL_COMMAND_HANDLER(flash_command_get_bank, 0, &bank);
	if (retval != ERROR_OK)
		return retval;

	struct target *target = bank->target;
	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	struct max32xxx_qspi_flash_bank *max32xxx_qspi_info = bank->driver_priv;
	if (!(max32xxx_qspi_info->probed)) {
		LOG_ERROR("Flash bank not probed");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	if (max32xxx_qspi_info->dev.chip_erase_cmd == 0x00) {
		LOG_ERROR("Mass erase not available for this device");
		return ERROR_FLASH_OPER_UNSUPPORTED;
	}

	retval = max32xxx_qspi_pre_op(bank);
	if (retval != ERROR_OK)
		return retval;

	// Set the write enable
	retval = max32xxx_qspi_set_we(target);
	if (retval != ERROR_OK)
		goto exit;

	// Send the mass erase command
	uint8_t cmd_data[1];
	cmd_data[0] = max32xxx_qspi_info->dev.chip_erase_cmd;

	retval = max32xxx_qspi_write_bytes(target, cmd_data, 1, true);
	if (retval != ERROR_OK)
		goto exit;

	// Poll WIP until erase is complete
	retval = max32xxx_qspi_poll_wip(target, MAX32XXX_QSPI_MASS_ERASE_TIMEOUT_MS);

exit:
	{
		int post_retval = max32xxx_qspi_post_op(bank);
		if (retval == ERROR_OK)
			retval = post_retval;
	}

	return retval;
}

static int get_max32xxx_qspi_info(struct flash_bank *bank,
		struct command_invocation *cmd)
{
	struct max32xxx_qspi_flash_bank *max32xxx_qspi_info = bank->driver_priv;

	if (!(max32xxx_qspi_info->probed)) {
		command_print_sameline(cmd, "\nQSPI flash bank not probed yet\n");
		return ERROR_FLASH_BANK_NOT_PROBED;
	}

	command_print_sameline(cmd, "\nQSPI flash:\n");

	command_print_sameline(cmd, "  name          : \'%s\'\n",
			max32xxx_qspi_info->dev.name);
	command_print_sameline(cmd, "  ID            : 0x%06" PRIx32 "\n",
			max32xxx_qspi_info->dev.device_id);
	command_print_sameline(cmd, "  size          : 0x%08" PRIx32 " B\n",
			max32xxx_qspi_info->dev.size_in_bytes);
	command_print_sameline(cmd, "  page size     : 0x%08" PRIx32 " B\n",
			max32xxx_qspi_info->dev.pagesize);
	command_print_sameline(cmd, "  sector size   : 0x%08" PRIx32 " B\n",
			max32xxx_qspi_info->dev.sectorsize);
	command_print_sameline(cmd, "  read cmd      : 0x%02" PRIx32 "\n",
			max32xxx_qspi_info->dev.read_cmd);
	command_print_sameline(cmd, "  dread cmd     : 0x%02" PRIx32 "\n",
			max32xxx_qspi_info->dev.dread_cmd);
	command_print_sameline(cmd, "  dread mode    : 0x%02" PRIx32 "\n",
			max32xxx_qspi_info->dev.dread_mode);
	command_print_sameline(cmd, "  dread dclk    : 0x%02" PRIx32 "\n",
			max32xxx_qspi_info->dev.dread_dclk);
	command_print_sameline(cmd, "  qread cmd     : 0x%02" PRIx32 "\n",
			max32xxx_qspi_info->dev.qread_cmd);
	command_print_sameline(cmd, "  pprog cmd     : 0x%02" PRIx32 "\n",
			max32xxx_qspi_info->dev.pprog_cmd);
	command_print_sameline(cmd, "  erase cmd     : 0x%02" PRIx32 "\n",
			max32xxx_qspi_info->dev.erase_cmd);
	command_print_sameline(cmd, "  chip_erase cmd: 0x%02" PRIx32 "\n",
			max32xxx_qspi_info->dev.chip_erase_cmd);

	return ERROR_OK;
}

static const struct command_registration max32xxx_qspi_exec_command_handlers[] = {
	{
		.name = "mem_mapping_setup",
		.handler = max32xxx_qspi_handle_mem_mapping_setup_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "Setup memory-mapped QSPI access.",
	},
	{
		.name = "mass_erase",
		.handler = max32xxx_qspi_handle_mass_erase_command,
		.mode = COMMAND_EXEC,
		.usage = "bank_id",
		.help = "Mass erase entire flash device.",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration max32xxx_qspi_command_handlers[] = {
	{
		.name = "max32xxx_qspi",
		.mode = COMMAND_ANY,
		.help = "max32xxx_qspi flash command group",
		.usage = "",
		.chain = max32xxx_qspi_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver max32xxx_qspi_flash = {
	.name = "max32xxx_qspi",
	.commands = max32xxx_qspi_command_handlers,
	.flash_bank_command = max32xxx_qspi_flash_bank_command,
	.erase = max32xxx_qspi_erase,
	.protect = NULL,
	.write = max32xxx_qspi_write,
	.read = default_flash_read,
	.verify = default_flash_verify,
	.probe = max32xxx_qspi_probe,
	.auto_probe = max32xxx_qspi_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = NULL,
	.info = get_max32xxx_qspi_info,
	.free_driver_priv = default_flash_free_driver_priv,
};
