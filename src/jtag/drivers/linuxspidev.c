// SPDX-License-Identifier: GPL-2.0-or-later

/* Copyright (C) 2020 by Lup Yuen Lee <luppy@appkaki.com>
 * Copyright (C) 2024 by Richard Pasek <rpasek@google.com>
 */

/**
 * @file
 * Implementation of SWD protocol with a Linux SPI device.
 */

/* Uncomment to log SPI exchanges (very verbose, slows things down a lot)
 *
 * A quick note on interpreting SPI exchange messages:
 *
 * This implementation works by performing SPI exchanges with MOSI and MISO
 * tied together with a 1K resistor. This combined signal becomes SWDIO.
 *
 * Since we are performing SPI exchanges, (reading and writing at the same
 * time) this means when the target isn't driving SWDIO, what is written by
 * the host should also be read by the host.
 *
 * On SWD writes:
 *   The TX and RX data should match except for the ACK bits from the target
 *     swd write reg exchange: len=6
 *      tx_buf=C5 02 40 00 02 2C
 *      rx_buf=C5 42 40 00 02 2C
 *                ^
 *                |
 *          ACK from target
 *
 * On SWD reads:
 *   Only the command byte should match
 *     swd read reg exchange: len=6
 *      tx_buf=B1 00 00 00 00 00
 *      rx_buf=B1 40 20 00 00 F8
 *             ^^
 *             ||
 *     Command packet from host
 *
 */
// #define LOG_SPI_EXCHANGE

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdint.h>
#include <unistd.h>
#include <stdlib.h>
#include <string.h>
#include <fcntl.h>
#include <sys/ioctl.h>
#include <linux/spi/spidev.h>
#include <jtag/adapter.h>
#include <jtag/swd.h>
#include <jtag/interface.h>
#include <jtag/commands.h>

// Number of bits per SPI exchange
#define SPI_BITS 8

// Time in uS after the last bit of transfer before deselecting the device
#define SPI_DESELECT_DELAY 0

// Maximum number of SWD transactions to queue together in a SPI exchange
#define MAX_QUEUE_ENTRIES 64

#define CMD_BITS 8
#define TURN_BITS 1
#define ACK_BITS 3
#define DATA_BITS 32
#define PARITY_BITS 1

#define SWD_WR_BITS (CMD_BITS + TURN_BITS + ACK_BITS + TURN_BITS + DATA_BITS + PARITY_BITS)
#define SWD_RD_BITS (CMD_BITS + TURN_BITS + ACK_BITS + DATA_BITS + PARITY_BITS + TURN_BITS)
#define SWD_OP_BITS (MAX(SWD_WR_BITS, SWD_RD_BITS))
#define SWD_OP_BYTES (DIV_ROUND_UP(SWD_OP_BITS, SPI_BITS))

#define AP_DELAY_CLOCKS 8
#define AP_DELAY_BYTES (DIV_ROUND_UP(AP_DELAY_CLOCKS, SPI_BITS))

#define END_IDLE_CLOCKS 8
#define END_IDLE_BYTES (DIV_ROUND_UP(END_IDLE_CLOCKS, SPI_BITS))

// File descriptor for SPI device
static int spi_fd = -1;

// SPI Configuration
static char *spi_path;
static uint32_t spi_mode = SPI_MODE_3; // Note: SPI in LSB mode is not often supported. We'll flip LSB to MSB ourselves.
static uint32_t spi_speed;

struct queue_info {
	unsigned int buf_idx;
	uint32_t *rx_ptr;
};

static int queue_retval;
static unsigned int max_queue_entries;
static unsigned int queue_fill;
static unsigned int queue_buf_fill;
static unsigned int queue_buf_size;
static struct queue_info *queue_infos;
static uint8_t *queue_tx_buf;
static uint8_t *queue_rx_buf;
static uint8_t *tx_flip_buf;

static int spidev_swd_switch_seq(enum swd_special_seq seq);
static void spidev_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk);

static void spi_exchange(const uint8_t *tx_data, uint8_t *rx_data, unsigned int len)
{
#ifdef LOG_SPI_EXCHANGE
	LOG_OUTPUT("exchange: len=%u\n", len);
#endif // LOG_SPI_EXCHANGE

	if (len == 0) {
		LOG_DEBUG("exchange with no length");
		return;
	}

	if (!tx_data && !rx_data) {
		LOG_DEBUG("exchange with no valid tx or rx pointers");
		return;
	}

	if (len > queue_buf_size) {
		LOG_ERROR("exchange too large len=%u ", len);
		return;
	}

	if (tx_data) {
		// Reverse LSB to MSB
		for (unsigned int i = 0; i < len; i++)
			tx_flip_buf[i] = flip_u32(tx_data[i], 8);

#ifdef LOG_SPI_EXCHANGE
		if (len != 0) {
			LOG_OUTPUT(" tx_buf=");
			for (unsigned int i = 0; i < len; i++)
				LOG_OUTPUT("%.2" PRIx8 " ", tx_flip_buf[i]);
		}
		LOG_OUTPUT("\n");
#endif // LOG_SPI_EXCHANGE
	}
	// Transmit the MSB buffer to SPI device.
	struct spi_ioc_transfer tr = {
		/* The following must be cast to unsigned long to compile correctly on
		 * 32 and 64 bit machines (as is done in the spidev examples in Linux
		 * kernel code in tools/spi/).
		 */
		.tx_buf = (unsigned long)(tx_data ? tx_flip_buf : NULL),
		.rx_buf = (unsigned long)rx_data,
		.len = len,
		.delay_usecs = SPI_DESELECT_DELAY,
		.speed_hz = spi_speed,
		.bits_per_word = SPI_BITS,
	};
	int ret = ioctl(spi_fd, SPI_IOC_MESSAGE(1), &tr);

	if (ret < 1) {
		LOG_ERROR("exchange failed");
		return;
	}

	if (rx_data) {
#ifdef LOG_SPI_EXCHANGE
		if (len != 0) {
			LOG_OUTPUT(" rx_buf=");
			for (unsigned int i = 0; i < len; i++)
				LOG_OUTPUT("%.2" PRIx8 " ", rx_data[i]);
		}
		LOG_OUTPUT("\n");
#endif // LOG_SPI_EXCHANGE

		// Reverse MSB to LSB
		for (unsigned int i = 0; i < len; i++)
			rx_data[i] = flip_u32(rx_data[i], 8);
	}
}

static int spidev_speed(int speed)
{
	uint32_t tmp_speed = speed;

	int ret = ioctl(spi_fd, SPI_IOC_WR_MAX_SPEED_HZ, &tmp_speed);
	if (ret == -1) {
		LOG_ERROR("Failed to set SPI %d speed", speed);
		return ERROR_FAIL;
	}

	spi_speed = speed;

	return ERROR_OK;
}

static int spidev_speed_div(int speed, int *khz)
{
	*khz = speed / 1000;
	return ERROR_OK;
}

static int spidev_khz(int khz, int *jtag_speed)
{
	if (khz == 0) {
		LOG_DEBUG("RCLK not supported");
		return ERROR_FAIL;
	}

	*jtag_speed = khz * 1000;
	return ERROR_OK;
}

static void spidev_free_queue(void)
{
	max_queue_entries = 0;
	queue_buf_size = 0;

	free(queue_infos);
	queue_infos = NULL;

	free(queue_tx_buf);
	queue_tx_buf = NULL;

	free(queue_rx_buf);
	queue_rx_buf = NULL;

	free(tx_flip_buf);
	tx_flip_buf = NULL;
}

static int spidev_alloc_queue(unsigned int new_queue_entries)
{
	if (queue_fill || queue_buf_fill) {
		LOG_ERROR("Can't realloc allocate queue when queue is in use");
		return ERROR_FAIL;
	}

	unsigned int new_queue_buf_size =
		(new_queue_entries * (SWD_OP_BYTES + AP_DELAY_BYTES)) + END_IDLE_BYTES;

	queue_infos = realloc(queue_infos, sizeof(struct queue_info) * new_queue_entries);
	if (!queue_infos)
		goto realloc_fail;

	queue_tx_buf = realloc(queue_tx_buf, new_queue_buf_size);
	if (!queue_tx_buf)
		goto realloc_fail;

	queue_rx_buf = realloc(queue_rx_buf, new_queue_buf_size);
	if (!queue_rx_buf)
		goto realloc_fail;

	tx_flip_buf = realloc(tx_flip_buf, new_queue_buf_size);
	if (!tx_flip_buf)
		goto realloc_fail;

	max_queue_entries = new_queue_entries;
	queue_buf_size = new_queue_buf_size;

	LOG_DEBUG("Set queue entries to %u (buffers %u bytes)", max_queue_entries, queue_buf_size);

	return ERROR_OK;

realloc_fail:
	spidev_free_queue();

	LOG_ERROR("Couldn't allocate queue. Out of memory.");
	return ERROR_FAIL;
}

static int spidev_init(void)
{
	LOG_INFO("SPI SWD driver");

	if (spi_fd >= 0)
		return ERROR_OK;

	if (!spi_path) {
		LOG_ERROR("Path to spidev not set");
		return ERROR_JTAG_INIT_FAILED;
	}

	// Open SPI device.
	spi_fd = open(spi_path, O_RDWR);
	if (spi_fd < 0) {
		LOG_ERROR("Failed to open SPI port at %s", spi_path);
		return ERROR_JTAG_INIT_FAILED;
	}

	// Set SPI mode.
	int ret = ioctl(spi_fd, SPI_IOC_WR_MODE32, &spi_mode);
	if (ret == -1) {
		LOG_ERROR("Failed to set SPI mode 0x%" PRIx32, spi_mode);
		return ERROR_JTAG_INIT_FAILED;
	}

	// Set SPI bits per word.
	uint32_t spi_bits = SPI_BITS;
	ret = ioctl(spi_fd, SPI_IOC_WR_BITS_PER_WORD, &spi_bits);
	if (ret == -1) {
		LOG_ERROR("Failed to set SPI %" PRIu8 " bits per transfer", spi_bits);
		return ERROR_JTAG_INIT_FAILED;
	}

	LOG_INFO("Opened SPI device at %s in mode 0x%" PRIx32 " with %" PRIu8 " bits ",
		spi_path, spi_mode, spi_bits);

	// Set SPI read and write max speed.
	int speed;
	ret = adapter_get_speed(&speed);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to get adapter speed");
		return ERROR_JTAG_INIT_FAILED;
	}

	ret = spidev_speed(speed);
	if (ret != ERROR_OK)
		return ERROR_JTAG_INIT_FAILED;

	if (max_queue_entries == 0) {
		ret = spidev_alloc_queue(MAX_QUEUE_ENTRIES);
		if (ret != ERROR_OK)
			return ERROR_JTAG_INIT_FAILED;
	}

	return ERROR_OK;
}

static int spidev_quit(void)
{
	spidev_free_queue();

	if (spi_fd < 0)
		return ERROR_OK;

	close(spi_fd);
	spi_fd = -1;

	free(spi_path);
	spi_path = NULL;

	return ERROR_OK;
}

static int spidev_swd_init(void)
{
	LOG_DEBUG("spidev_swd_init");
	return ERROR_OK;
}

static int spidev_swd_execute_queue(unsigned int end_idle_bytes)
{
	LOG_DEBUG_IO("Executing %u queued transactions", queue_fill);

	if (queue_retval != ERROR_OK) {
		LOG_DEBUG_IO("Skipping due to previous errors: %d", queue_retval);
		goto skip;
	}

	/* A transaction must be followed by another transaction or at least 8 idle
	 * cycles to ensure that data is clocked through the AP. Since the tx
	 * buffer is zeroed after each queue run, every byte added to the buffer
	 * fill will add on an additional 8 idle cycles.
	 */
	queue_buf_fill += end_idle_bytes;

	spi_exchange(queue_tx_buf, queue_rx_buf, queue_buf_fill);

	for (unsigned int queue_idx = 0; queue_idx < queue_fill; queue_idx++) {
		unsigned int buf_idx = queue_infos[queue_idx].buf_idx;
		uint8_t *tx_ptr = &queue_tx_buf[buf_idx];
		uint8_t *rx_ptr = &queue_rx_buf[buf_idx];
		uint8_t cmd = buf_get_u32(tx_ptr, 0, CMD_BITS);
		bool read = cmd & SWD_CMD_RNW ? true : false;
		int ack = buf_get_u32(rx_ptr, CMD_BITS + TURN_BITS, ACK_BITS);
		uint32_t data = read ?
			buf_get_u32(rx_ptr, CMD_BITS + TURN_BITS + ACK_BITS, DATA_BITS) :
			buf_get_u32(tx_ptr, CMD_BITS + TURN_BITS + ACK_BITS + TURN_BITS, DATA_BITS);

		// Devices do not reply to DP_TARGETSEL write cmd, ignore received ack
		bool check_ack = swd_cmd_returns_ack(cmd);

		LOG_CUSTOM_LEVEL((check_ack && ack != SWD_ACK_OK) ? LOG_LVL_DEBUG : LOG_LVL_DEBUG_IO,
				"%s%s %s %s reg %X = %08" PRIx32,
				check_ack ? "" : "ack ignored ",
				ack == SWD_ACK_OK ? "OK" :
				ack == SWD_ACK_WAIT ? "WAIT" :
				ack == SWD_ACK_FAULT ? "FAULT" : "JUNK",
				cmd & SWD_CMD_APNDP ? "AP" : "DP",
				read ? "read" : "write",
				(cmd & SWD_CMD_A32) >> 1,
				data);

		if (ack != SWD_ACK_OK && check_ack) {
			queue_retval = swd_ack_to_error_code(ack);
			goto skip;

		} else if (read) {
			int parity = buf_get_u32(rx_ptr, CMD_BITS + TURN_BITS + ACK_BITS + DATA_BITS, PARITY_BITS);

			if (parity != parity_u32(data)) {
				LOG_ERROR("SWD Read data parity mismatch");
				queue_retval = ERROR_FAIL;
				goto skip;
			}

			if (queue_infos[queue_idx].rx_ptr)
				*queue_infos[queue_idx].rx_ptr = data;
		}
	}

skip:
	// Clear everything in the queue
	queue_fill = 0;
	queue_buf_fill = 0;
	memset(queue_infos, 0, sizeof(queue_infos[0]) * max_queue_entries);
	memset(queue_tx_buf, 0, queue_buf_size);
	memset(queue_rx_buf, 0, queue_buf_size);

	int retval = queue_retval;
	queue_retval = ERROR_OK;

	return retval;
}

static int spidev_swd_run_queue(void)
{
	/* Since we are unsure if another SWD transaction will follow the
	 * transactions we are just about to execute, we need to add on 8 idle
	 * cycles.
	 */
	return spidev_swd_execute_queue(END_IDLE_BYTES);
}

static void spidev_swd_queue_cmd(uint8_t cmd, uint32_t *dst, uint32_t data, uint32_t ap_delay_clk)
{
	unsigned int swd_op_bytes = DIV_ROUND_UP(SWD_OP_BITS + ap_delay_clk, SPI_BITS);

	if (queue_fill >= max_queue_entries ||
	    queue_buf_fill + swd_op_bytes + END_IDLE_BYTES > queue_buf_size) {
		/* Not enough room in the queue. Run the queue. No idle bytes are
		 * needed because we are going to execute transactions right after
		 * the queue is emptied.
		 */
		queue_retval = spidev_swd_execute_queue(0);
	}

	if (queue_retval != ERROR_OK)
		return;

	uint8_t *tx_ptr = &queue_tx_buf[queue_buf_fill];

	cmd |= SWD_CMD_START | SWD_CMD_PARK;

	buf_set_u32(tx_ptr, 0, CMD_BITS, cmd);

	if (cmd & SWD_CMD_RNW) {
		// Queue a read transaction
		queue_infos[queue_fill].rx_ptr = dst;
	} else {
		// Queue a write transaction
		buf_set_u32(tx_ptr,
			CMD_BITS + TURN_BITS + ACK_BITS + TURN_BITS, DATA_BITS, data);
		buf_set_u32(tx_ptr,
			CMD_BITS + TURN_BITS + ACK_BITS + TURN_BITS + DATA_BITS, PARITY_BITS, parity_u32(data));
	}

	queue_infos[queue_fill].buf_idx = queue_buf_fill;

	/* Add idle cycles after AP accesses to avoid WAIT. Buffer is already
	 * zeroed so we just need to advance the pointer to add idle cycles.
	 */
	queue_buf_fill += swd_op_bytes;

	queue_fill++;
}

static void spidev_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
	assert(cmd & SWD_CMD_RNW);
	spidev_swd_queue_cmd(cmd, value, 0, ap_delay_clk);
}

static void spidev_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
	assert(!(cmd & SWD_CMD_RNW));
	spidev_swd_queue_cmd(cmd, NULL, value, ap_delay_clk);
}

static int spidev_swd_switch_seq(enum swd_special_seq seq)
{
	switch (seq) {
	case LINE_RESET:
		LOG_DEBUG_IO("SWD line reset");
		spi_exchange(swd_seq_line_reset, NULL, swd_seq_line_reset_len / SPI_BITS);
		break;
	case JTAG_TO_SWD:
		LOG_DEBUG("JTAG-to-SWD");
		spi_exchange(swd_seq_jtag_to_swd, NULL, swd_seq_jtag_to_swd_len / SPI_BITS);
		break;
	case JTAG_TO_DORMANT:
		LOG_DEBUG("JTAG-to-DORMANT");
		spi_exchange(swd_seq_jtag_to_dormant, NULL, swd_seq_jtag_to_dormant_len / SPI_BITS);
		break;
	case SWD_TO_JTAG:
		LOG_DEBUG("SWD-to-JTAG");
		spi_exchange(swd_seq_swd_to_jtag, NULL, swd_seq_swd_to_jtag_len / SPI_BITS);
		break;
	case SWD_TO_DORMANT:
		LOG_DEBUG("SWD-to-DORMANT");
		spi_exchange(swd_seq_swd_to_dormant, NULL, swd_seq_swd_to_dormant_len / SPI_BITS);
		break;
	case DORMANT_TO_SWD:
		LOG_DEBUG("DORMANT-to-SWD");
		spi_exchange(swd_seq_dormant_to_swd, NULL, swd_seq_dormant_to_swd_len / SPI_BITS);
		break;
	case DORMANT_TO_JTAG:
		LOG_DEBUG("DORMANT-to-JTAG");
		spi_exchange(swd_seq_dormant_to_jtag, NULL, swd_seq_dormant_to_jtag_len / SPI_BITS);
		break;
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(spidev_handle_path_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	free(spi_path);
	spi_path = strdup(CMD_ARGV[0]);
	if (!spi_path) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(spidev_handle_mode_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], spi_mode);
	return ERROR_OK;
}

COMMAND_HANDLER(spidev_handle_queue_entries_command)
{
	uint32_t new_queue_entries;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], new_queue_entries);

	return spidev_alloc_queue(new_queue_entries);
}

const struct swd_driver spidev_swd = {
	.init = spidev_swd_init,
	.switch_seq = spidev_swd_switch_seq,
	.read_reg = spidev_swd_read_reg,
	.write_reg = spidev_swd_write_reg,
	.run = spidev_swd_run_queue,
};

static const struct command_registration spidev_subcommand_handlers[] = {
	{
		.name = "path",
		.handler = &spidev_handle_path_command,
		.mode = COMMAND_CONFIG,
		.help = "set the path to the spidev device",
		.usage = "path_to_spidev",
	},
	{
		.name = "mode",
		.handler = &spidev_handle_mode_command,
		.mode = COMMAND_CONFIG,
		.help = "set the mode of the spi port with optional bit flags (default=3)",
		.usage = "mode",
	},
	{
		.name = "queue_entries",
		.handler = &spidev_handle_queue_entries_command,
		.mode = COMMAND_CONFIG,
		.help = "set the queue entry size (default=64)",
		.usage = "queue_entries",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration spidev_command_handlers[] = {
	{
		.name = "spidev",
		.mode = COMMAND_ANY,
		.help = "perform spidev management",
		.chain = spidev_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

// Only SWD transport supported
static const char *const spidev_transports[] = { "swd", NULL };

struct adapter_driver linuxspidev_adapter_driver = {
	.name = "linuxspidev",
	.transports = spidev_transports,
	.commands = spidev_command_handlers,

	.init = spidev_init,
	.quit = spidev_quit,
	.speed = spidev_speed,
	.khz = spidev_khz,
	.speed_div = spidev_speed_div,

	.swd_ops = &spidev_swd,
};
