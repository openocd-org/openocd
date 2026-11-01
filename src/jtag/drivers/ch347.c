// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Driver for CH347-JTAG interface V1.1                                  *
 *                                                                         *
 *   Copyright (C) 2022 by oidcat.                                         *
 *   Author: oidcatiot@163.com                                             *
 *                                                                         *
 *   Enhancements by EasyDevKits - info@easydevkits.com                    *
 *                                                                         *
 *   CH347 is a high-speed USB bus converter chip that provides UART, I2C  *
 *   and SPI synchronous serial ports and JTAG interface through USB bus.  *
 *                                                                         *
 *   The JTAG interface by CH347 can supports transmission frequency       *
 *   configuration up to 60MHz.                                            *
 *                                                                         *
 *   The USB2.0 to JTAG scheme based on CH347 can be used to build         *
 *   customized USB high-speed JTAG debugger and other products.           *
 *                                                                         *
 *            _____________                                                *
 *           |             |____JTAG/SWD (TDO,TDI,TMS,TCK,TRST)            *
 *      USB__|   CH347T/F  |                                               *
 *           |_____________|____UART(TXD1,RXD1,RTS1,CTS1,DTR1)             *
 *            ______|______                                                *
 *           |             |                                               *
 *           | 8 MHz XTAL  |                                               *
 *           |_____________|                                               *
 *                                                                         *
 *   This CH347 driver is tested with the CH347T chip in mode 3 and        *
 *   with the CH347F chip.                                                 *
 *                                                                         *
 *   The datasheet for the wch-ic.com's CH347 part is here:	           *
 *   https://www.wch-ic.com/downloads/CH347DS1_PDF.html                    *
 *                                                                         *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#if IS_CYGWIN == 1
#include "windows.h"
#undef LOG_ERROR
#endif

// project specific includes
#include <jtag/interface.h>
#include <jtag/commands.h>
#include <jtag/swd.h>
#include <jtag/adapter.h>
#include <helper/time_support.h>
#include <helper/replacements.h>
#include <helper/list.h>
#include <helper/binarybuffer.h>
#include <helper/align.h>
#include "libusb_helper.h"

// system includes
#include <stdlib.h>
#include <string.h>
#include <sys/time.h>
#include <time.h>
#include <unistd.h>

#define TDI_H	BIT(4)
#define TDI_L	0
#define TMS_H	BIT(1)
#define TMS_L	0
#define TCK_H	BIT(0)
#define TCK_L	0
#define TRST_H BIT(5)
#define TRST_L 0
#define SRST_H BIT(6)
#define SRST_L 0

#define GPIO_CNT	8 // the CH347 has 8 GPIO's
// SRST pin can be configured as CH347F_SRST_FAKE_GPIO
#define GPIO_CNT_EXT (GPIO_CNT + 1)
#define CH347F_SRST_FAKE_GPIO GPIO_CNT
#define CH347T_TRST_GPIO	5
#define CH347F_TRST_GPIO	3

/* For GPIO command: always set bits 7 and 6 for GPIO enable
	bits 5 and 4 for pin direction output bit 3 is the data bit */
#define GPIO_SET_L	(BIT(4) | BIT(5) | BIT(6) | BIT(7)) // value for setting a GPIO to low
#define GPIO_SET_H	(BIT(3) | BIT(4) | BIT(5) | BIT(6) | BIT(7)) // value for setting a GPIO to high
// enable the pin and set the direction to input, so the pin gets released
#define GPIO_SET_INPUT	(BIT(6) | BIT(7))

#define VENDOR_VERSION	0x5F // for getting the chip version

// maybe the hardware of the CH347 chip can	capture only this amount of TDO bits
#define HW_TDO_BUF_SIZE	4096
// Don't send more than this amount of bytes via libusb in one packet. Also that is the limit for LARGER_PACK mode.
#define LARGER_PACK_MAX_SIZE	51200
// The data length contained in each command packet	during USB high-speed operation
// REVISIT: could be USBC_PACKET_USBHS - CH347_CMD_HEADER ?
#define UCMDPKT_DATA_MAX_BYTES_USBHS	507
#define USBC_PACKET_USBHS	512 // Maximum data length per packet at USB high speed
#define CH347_CMD_HEADER	3 // Protocol header length (1 byte command type + 2 bytes data length)
#define CH347_CMD_INIT_READ_LEN	1 // JTAG_INIT/SWD_INIT/PIN_SET read one data byte
// if we send 9 in the init command we get the STANDARD mode or LARGER_PACK mode flag
#define CH347_CMD_INIT_GET_MODE_CLOCK_INDEX_VALUE 9
// No more bits are allowed per CH347_CMD_JTAG_BIT_OP command; this should be dividable by 8
#define MAX_BITS_PER_BIT_OP	248

/* Protocol transmission format: CMD (1 byte) + Length (2 bytes) + Data
	Parameter acquisition, used to obtain firmware version, JTAG interface related parameters, etc */
#define CH347_CMD_INFO_RD	0xCA
#define CH347_CMD_GPIO	0xCC // GPIO Command
#define CH347_CMD_JTAG_INIT	0xD0 // JTAG Interface initialization command
#define CH347_CMD_JTAG_BIT_OP	0xD1 // JTAG interface pin bit control command
#define CH347_CMD_JTAG_BIT_OP_RD	0xD2 // JTAG interface pin bit control and read commands
#define CH347_CMD_JTAG_DATA_SHIFT	0xD3 // JTAG interface data shift command
#define CH347_CMD_JTAG_DATA_SHIFT_RD	0xD4 // JTAG interface data shift and read command
#define CH347_CMD_PIN_SET	0xE2 // set pin functions (CH347F)

// for SWD
#define CH347_CMD_SWD_INIT	0xE5 // SWD Interface Initialization Command
#define CH347_CMD_SWD	0xE8 // SWD Command group header
#define CH347_CMD_SWD_REG_W	0xA0 // SWD Interface write reg
#define CH347_CMD_SWD_SEQ_W	0xA1 // SWD Interface write spec seq
#define CH347_CMD_SWD_REG_R	0xA2 // SWD Interface read  reg
#define CH347_MAX_PROCESSING_US 7000 // max time in us for packet processing
			  // USB hosts disconnect the adapter if SWD processing takes more!
#define CH347_MAX_SEND_BUF	UCMDPKT_DATA_MAX_BYTES_USBHS
#define CH347_MAX_RECV_BUF	(USBC_PACKET_USBHS - CH347_CMD_HEADER)
#define CH347_MAX_CMD_BUF	128
#define CH347_SWD_CLOCK_MAX 5000
#define CH347_SWD_CLOCK_BASE 1000
// limited by the longest sequence processing time
#define CH347_SWD_CLOCK_MAX_DIVISOR (CH347_MAX_PROCESSING_US / swd_seq_dormant_to_swd_len)

#define CH347_EPOUT	0x06u // the usb endpoint number for writing
#define CH347_EPIN	0x86u // the usb endpoint number for reading
#define CH347T_MPHSI_INTERFACE	2	// the CH347T JTAG interface is number 2
#define CH347F_MPHSI_INTERFACE	4	// the CH347F JTAG interface is number 4
#define USB_WRITE_TIMEOUT	500	// write timeout in milliseconds
#define USB_READ_TIMEOUT	500 // read timeout in milliseconds
// BCD version for devices that can use bytewise mode	below this version only bitwise mode can be used
#define BYTEWISE_MODE_VERSION	0x241
/* if the configuration is not setting the vendor id / product id we search for vendor id 1a86
	and product id's 0x55dd (CH347T chip in mode 3), 0x55de (CH347F chip) and 0x55e7 (other chip) */
#define DEFAULT_VENDOR_ID	0x1a86
#define DEFAULT_CH347T_PRODUCT_ID	0x55dd
#define DEFAULT_CH347F_PRODUCT_ID	0x55de
#define DEFAULT_OTHER_PRODUCT_ID	0x55e7

/* There are 3 different CH347 variants. The datasheet mentions the CH347T chip which has the product id 0x55dd
	as default when it's configured to mode 3. The CH347F chip is also mentioned in the datasheet and has the
	default product id 0x55de. The 3rd variant is not mentioned in a datasheet but was found here
	https://github.com/WCHSoftGroup/ch347
	The code from WCHSoftGroup is also searching for this product id */
enum ch347_variant {
	CH347T = 0,	// The CH347T chip
	CH347F = 1,	// The CH347F chip
	OTHER_PRODUCT_ID = 2, // other product id not mentioned in the datasheet
};

/* STANDARD_PACK means that we can send only one USBC_PACKET_USBHS-sized USB packet
	and then read data back. LARGER_PACK means, we can send packets as large as
	LARGER_PACK_MAX_SIZE. libusb splits there large packets into smaller USB packets and
	transmit the data. Then we read back the data in a bigger packet. */
enum pack_size {
	UNSET = -1,
	STANDARD_PACK = 0,
	LARGER_PACK = 1,
};

// for STANDARD_PACK mode: these are the 6 possible speeds; values in kHz
static const int ch347_standard_pack_clock_speeds[] = {
	1875, // 1.875 MHz (60000 : 32)
	3750, // 3.75 MHz (60000 : 16)
	7500, // 7.5 MHz (60000 : 8)
	15000, // 15 MHz (60000 : 4)
	30000, // 30 MHz (60000 : 2)
	60000 // 60 MHz
};

// for LARGER_PACK mode: these are the 8 possible speeds; values in kHz
static const int ch347_larger_pack_clock_speeds[] = {
	469, // 468.75 kHz (60000 : 128)
	938, // 937.5 kHz (60000 : 64)
	1875, // 1.875 MHz (60000 : 32)
	3750, // 3.75 MHz (60000 : 16)
	7500, // 7.5 MHz (60000 : 8)
	15000, // 15 MHz (60000 : 4)
	30000, // 30 MHz (60000 : 2)
	60000 // 60 MHz
};

struct ch347_cmd {
	uint8_t type; // the command type
	uint8_t *write_data; // data bytes for write
	uint16_t write_data_len; // count of data bytes in the write_data buffer
	uint16_t read_len; // if >0 a read is needed after this command
	uint16_t tdo_bit_count; // how many TDO bits are needed to shift in by this read
	struct list_head queue; // for handling a queue (list)
};

struct ch347_scan {
	struct scan_field *fields; // array of scan_field's for data from the device
	int fields_len; // scan_fields array length
	struct list_head queue; // for handling a queue (list)
};

struct ch347_info {
	// Record the CH347 pin status
	int tms_pin;
	int tdi_pin;
	int tck_pin;
	int trst_pin;
	int srst_pin;

	enum ch347_variant chip_variant; // ch347_variant for explanation
	// if true then we can't us the bytewise commands	due to a bug of the chip; depends on BYTEWISE_MODE_VERSION
	bool use_bitwise_mode;
	enum pack_size pack_size; // see: pack_size for explanation
	int max_len; // in STANDARD_PACK or bitwise mode we can send only one USBC_PACKET_USBHS sized package
	bool swclk_5mhz_supported;
	bool cmd_gpio_allowed;
	unsigned int srst_default_gpio, trst_default_gpio;
	bool led_is_on;
	bool led_log_once;
	uint8_t bit_op_log_once;

	// a "scratchpad" where we record all bytes for one command
	uint8_t scratchpad_cmd_type; // command type
	uint8_t scratchpad[UCMDPKT_DATA_MAX_BYTES_USBHS]; // scratchpad buffer
	int scratchpad_idx; // current index in scratchpad

	// after a command is complete it will be stored for later processing
	struct list_head cmd_queue;
	// all data input scan fields are queued here
	struct list_head scan_queue;
	// read buffer for the single commands like CH347_CMD_JTAG_INIT
	uint8_t init_result_buf[CH347_CMD_INIT_READ_LEN];
	int init_result_len;
	// GPIO status returned by CH347_CMD_GPIO
	uint8_t gpio_status_buf[GPIO_CNT];
};

struct ch347_swd_io {
	uint8_t usb_cmd; // 0xA0, 0xA1, 0xA2
	uint8_t cmd;
	uint32_t *dst;
	uint32_t value;
	struct list_head list_entry;
};

struct ch347_swd_context {
	uint8_t send_buf[CH347_MAX_SEND_BUF];
	uint8_t recv_buf[CH347_MAX_RECV_BUF];
	int send_len;
	int recv_len;
	int need_recv_len;
	int sent_cmd_count;
	unsigned int clk_divisor;
	unsigned int total_swd_clk;
	struct list_head send_cmd_head;
	struct list_head free_cmd_head;
	struct ch347_swd_io ch347_cmd_buf[CH347_MAX_CMD_BUF];
};

static struct ch347_swd_context ch347_swd_context;
static bool swd_mode;

static const uint16_t default_ch347_vids[] = {DEFAULT_VENDOR_ID, DEFAULT_VENDOR_ID, DEFAULT_VENDOR_ID, 0};
static const uint16_t default_ch347_pids[] = {DEFAULT_CH347T_PRODUCT_ID,
	DEFAULT_CH347F_PRODUCT_ID, DEFAULT_OTHER_PRODUCT_ID, 0};

/* Set after the CH347F dedicated SRST output is enabled and its pin byte
 * has been transmitted. JTAG pin bytes carry bit 6 only then. SWD leaves
 * this clear. */
static bool ch347_dedicated_srst_enabled;
static struct ch347_info ch347;
static struct libusb_device_handle *ch347_handle;

struct ch347_bit_op {
	uint8_t bit;
	bool jtag_use, swd_use;
	int *ptr;
};
static const struct ch347_bit_op ch347t_gpio_to_bit_op[GPIO_CNT_EXT] = {
	[0] = { .bit = TCK_H, .jtag_use = true, .swd_use = true, .ptr = &ch347.tck_pin },
	[1] = { .jtag_use = true },
	[2] = { .bit = TMS_H, .jtag_use = true, .swd_use = true, .ptr = &ch347.tms_pin },
	[CH347T_TRST_GPIO] = { .bit = TRST_H, .ptr = &ch347.trst_pin }, // GPIO 5
};

static const struct ch347_bit_op ch347f_gpio_to_bit_op[GPIO_CNT_EXT] = {
	[CH347F_TRST_GPIO] = { .bit = TRST_H, .ptr = &ch347.trst_pin }, // GPIO 3
	[4] = { .bit = TCK_H, .jtag_use = true, .swd_use = true, .ptr = &ch347.tck_pin },
	[5] = { .jtag_use = true },
	[6] = { .bit = TDI_H, .jtag_use = true, .ptr = &ch347.tdi_pin },
	[7] = { .bit = TMS_H, .jtag_use = true, .swd_use = true, .ptr = &ch347.tms_pin },
	[CH347F_SRST_FAKE_GPIO] = { .bit = SRST_H, .ptr = &ch347.srst_pin},
};

/* there are "single" commands. These commands can't be chained together and
	need to be send as single command and need a read after write */
static inline bool ch347_is_single_cmd_type(uint8_t type)
{
	return type == CH347_CMD_JTAG_INIT || type == CH347_CMD_SWD_INIT
			|| type == CH347_CMD_PIN_SET;
}

/* The commands with variable data size can be concatenated in the scratchpad
 * and split if the scratchpad overflows */
static inline bool ch347_cmd_has_variable_data_size(uint8_t type)
{
	return type == CH347_CMD_JTAG_BIT_OP || type == CH347_CMD_JTAG_BIT_OP_RD
			|| type == CH347_CMD_JTAG_DATA_SHIFT || type == CH347_CMD_JTAG_DATA_SHIFT_RD;
}

static void log_buf_dump(const uint8_t *data, unsigned int size, bool recv)
{
	unsigned int i = 0, j = 0;
	unsigned int n = size * 3 + 1;
	char *str = malloc(n);
	if (!str)
		return;

	while (i < size && j < n) {
		uint8_t cmd = data[i++];
		hexify(str + j, &cmd, 1, n - j);
		j += 2;
		str[j++] = ' ';

		unsigned int cmd_payload_size = le_to_h_u16(data + i);
		if (i + 2 <= size && j + 4 < n) {
			hexify(str + j, data + i, 2, n - j);
			i += 2;
			j += 4;
			str[j++] = ' ';
		}

		if (cmd == CH347_CMD_SWD) {
			// nested SWD commands
			str[j] = '\0';
			if (i + cmd_payload_size > size) {
				LOG_DEBUG_IO("%s - bad size", str);
				cmd_payload_size = size - i;
			} else {
				LOG_DEBUG_IO("%s", str);
			}
			j = 0;
			unsigned int swd_base_i = i;

			while (i < swd_base_i + cmd_payload_size) {
				uint8_t swd_cmd = data[i++];
				hexify(str + j, &swd_cmd, 1, n - j);
				j += 2;
				str[j++] = ' ';

				unsigned int swd_bits = 0;
				unsigned int swd_payload_size = 0;
				if (!recv) {
					if (i + 2 <= size) {
						swd_bits = le_to_h_u16(data + i);
						hexify(str + j, data + i, 2, n - j);
						i += 2;
						j += 4;
						str[j++] = ' ';
					}
				}

				switch (swd_cmd) {
				case CH347_CMD_SWD_REG_W:
					if (recv)
						swd_payload_size = 1;
					else
						swd_payload_size = DIV_ROUND_UP(swd_bits, 8);
					break;
				case CH347_CMD_SWD_SEQ_W:
					if (!recv)
						swd_payload_size = DIV_ROUND_UP(swd_bits, 8);
					break;
				case CH347_CMD_SWD_REG_R:
					if (recv)
						swd_payload_size = 1 + 4 + 1;
					else
						swd_payload_size = 1;
					break;
				}

				hexify(str + j, data + i, MIN(swd_payload_size, size - i), n - j);
				i += swd_payload_size;
				j += 2 * swd_payload_size;
				str[j] = '\0';
				if (i > size)
					LOG_DEBUG_IO("    %s - bad size", str);
				else
					LOG_DEBUG_IO("    %s", str);
				j = 0;
			}
		} else {
			hexify(str + j, data + i, MIN(cmd_payload_size, size - i), n - j);
			i += cmd_payload_size;
			j += 2 * cmd_payload_size;
			str[j] = '\0';
			if (i > size)
				LOG_DEBUG_IO("%s - bad size", str);
			else
				LOG_DEBUG_IO("%s", str);
			j = 0;
		}
	}
	free(str);
}

/**
 * @brief writes data to the CH347 via libusb driver
 *
 * @param data Point to the data buffer
 * @param length Data length in and out
 * @return ERROR_OK at success
 */
static int ch347_write_data(uint8_t *data, int *length)
{
	int write_len = *length;
	int i = 0;
	int transferred = 0;

	while (true) {
		int retval = jtag_libusb_bulk_write(ch347_handle, CH347_EPOUT, (char *)&data[i],
			write_len, USB_WRITE_TIMEOUT, &transferred);
		if (retval != ERROR_OK) {
			LOG_ERROR("CH347 write fail (size %d)", write_len);
			log_buf_dump(data, *length, false);
			*length = 0;
			return retval;
		}
		i += transferred;
		if (i >= *length)
			break;
		write_len = *length - i;
	}

	if (LOG_LEVEL_IS(LOG_LVL_DEBUG_IO)) {
		LOG_DEBUG_IO("size=%d, buf=", i);
		log_buf_dump(data, i, false);
	}

	*length = i;
	return ERROR_OK;
}

/**
 * @brief reads data from the CH347 via libusb driver
 * See https://libusb.sourceforge.io/api-1.0/libusb_packetoverflow.html
 * for explanation of avoiding LIBUSB_TRANSFER_OVERFLOW
 *
 * @param data	Point to the data buffer
 * @param buf_length Maximal length, aligned up to USB packet size
 * @param length Minimal requested data length in and real read length out
 * @return ERROR_OK at success
 */
static int ch347_read_data(uint8_t *data, int buf_length, int *length)
{
	int done = 0;

	do {
		int transferred = 0;
		int read_len = buf_length - done;
		int retval = jtag_libusb_bulk_read(ch347_handle, CH347_EPIN, (char *)&data[done],
			read_len, USB_READ_TIMEOUT, &transferred);
		if (retval != ERROR_OK) {
			LOG_ERROR("CH347 read fail");
			*length = 0;
			return retval;
		}

		done += transferred;
	} while (done < *length);

	if (LOG_LEVEL_IS(LOG_LVL_DEBUG_IO)) {
		LOG_DEBUG_IO("size=%d, buf=", done);
		log_buf_dump(data, done, true);
	}

	*length = done;
	return ERROR_OK;
}

/**
 * @brief calculates the amount of bits and bytes that should be read
 * for this command
 *
 * @param cmd command for the calculation
 */
static void ch347_cmd_calc_reads(struct ch347_cmd *cmd)
{
	cmd->read_len = 0;
	cmd->tdo_bit_count = 0;

	switch (cmd->type) {
	case CH347_CMD_GPIO:
		// for GPIO we need to read back the same amount of data that we had send
		cmd->read_len = cmd->write_data_len;
		break;
	case CH347_CMD_SWD:
		cmd->read_len = ch347_swd_context.need_recv_len;
		break;
	case CH347_CMD_JTAG_INIT:
	case CH347_CMD_SWD_INIT:
	case CH347_CMD_PIN_SET:
		// for JTAG_INIT/SWD_INIT/PIN_SET the amount is fixed
		cmd->read_len = CH347_CMD_INIT_READ_LEN;
		break;
	case CH347_CMD_JTAG_BIT_OP_RD:
		// for bit operations we need to count the TCK high edges
		for (int i = 0; i < cmd->write_data_len; i++) {
			if ((cmd->write_data[i] & TCK_H) == TCK_H) {
				cmd->read_len++;
				cmd->tdo_bit_count++;
			}
		}
		break;
	case CH347_CMD_JTAG_DATA_SHIFT_RD:
		// for byte operations: need to read one byte back for each data byte
		cmd->read_len = cmd->write_data_len;
		// we occupy 8 bits per byte in the TDO hardware buffer
		cmd->tdo_bit_count = cmd->read_len * 8;
		break;
	}
}

/**
 * @brief copy the scratchpad content into a new command in the command queue
 *
 * @param scan_fields array of scan_field's for data from the device
 * @param scan_fields_len array length
 * @return ERROR_OK at success
 */
static int ch347_cmd_from_scratchpad(void)
{
	// nothing to do if no bytes are recorded
	if (!ch347.scratchpad_idx)
		return ERROR_OK;

	// malloc for the command and data bytes
	struct ch347_cmd *cmd = malloc(sizeof(struct ch347_cmd));
	if (cmd)
		cmd->write_data = malloc(ch347.scratchpad_idx);
	if (!cmd || !cmd->write_data) {
		LOG_ERROR("malloc failed");
		free(cmd);
		return ERROR_FAIL;
	}

	// copy data, calculate the reads and add to the command queue
	cmd->type = ch347.scratchpad_cmd_type;
	cmd->write_data_len = ch347.scratchpad_idx;
	memcpy(cmd->write_data, ch347.scratchpad, ch347.scratchpad_idx);
	ch347_cmd_calc_reads(cmd);
	list_add_tail(&cmd->queue, &ch347.cmd_queue);

	// cleanup the scratchpad for the next command
	ch347.scratchpad_cmd_type = 0;
	ch347.scratchpad_idx = 0;
	return ERROR_OK;
}

/**
 * @brief Reads data back from CH347 and decode it byte- and bitwise into the buffer
 *
 * @param decoded_buf Point to a buffer to place the data to be decoded; need to be sized
 * to the decoded size length; not the raw_read_len
 * @param decoded_buf_len length of the decoded_buf
 * @param raw_read_len Data length in bytes that should be read via libusb; the decoded length can be shorter
 * @return ERROR_OK at success
 */
static int ch347_read_scan(uint8_t *decoded_buf, int decoded_buf_len, int raw_read_len)
{
	int read_bf_len = ALIGN_UP(raw_read_len, USBC_PACKET_USBHS);
	uint8_t *read_buf = malloc(read_bf_len);
	if (!read_buf) {
		LOG_ERROR("malloc failed");
		return ERROR_FAIL;
	}

	int read_len = raw_read_len;
	int retval = ch347_read_data(read_buf, read_bf_len, &read_len);
	if (retval != ERROR_OK)	{
		free(read_buf);
		return retval;
	}

	int rd_idx = 0;
	int decoded_buf_idx = 0;

	while (rd_idx < read_len) {
		unsigned int type = read_buf[rd_idx++];
		uint16_t data_len = le_to_h_u16(&read_buf[rd_idx]);
		rd_idx += 2;

		switch (type) {
		case CH347_CMD_JTAG_INIT:
		case CH347_CMD_SWD_INIT:
		case CH347_CMD_PIN_SET:
			if (data_len != CH347_CMD_INIT_READ_LEN) {
				LOG_ERROR("CH347 unexpected returned data length from init CMD");
				free(read_buf);
				return ERROR_FAIL;
			}
			memcpy(ch347.init_result_buf, &read_buf[rd_idx], data_len);
			ch347.init_result_len = data_len;
			rd_idx += data_len;
			break;
		case CH347_CMD_GPIO:
			if (data_len != GPIO_CNT) {
				LOG_ERROR("CH347 unexpected returned data length from CMD GPIO");
				free(read_buf);
				return ERROR_FAIL;
			}
			memcpy(ch347.gpio_status_buf, &read_buf[rd_idx], data_len);
			rd_idx += data_len;
			break;
		case CH347_CMD_JTAG_DATA_SHIFT_RD:
			// for bytewise commands: copy the data bytes
			if (!decoded_buf || decoded_buf_idx + data_len > decoded_buf_len) {
				LOG_ERROR("CH347 no decoded_buf or too small");
				free(read_buf);
				return ERROR_FAIL;
			}
			memcpy(&decoded_buf[decoded_buf_idx], &read_buf[rd_idx], data_len);
			decoded_buf_idx += data_len;
			rd_idx += data_len;
			break;
		case CH347_CMD_JTAG_BIT_OP_RD:
			// for CH347_CMD_JTAG_BIT_OP_RD we need to copy bit by bit
			if (!decoded_buf || decoded_buf_idx + DIV_ROUND_UP(data_len, 8) > decoded_buf_len) {
				LOG_ERROR("CH347 no decoded_buf or too small");
				free(read_buf);
				return ERROR_FAIL;
			}
			for (int i = 0; i < data_len; i++) {
				if (read_buf[rd_idx + i] & BIT(0))
					decoded_buf[decoded_buf_idx + i / 8] |= BIT(i % 8);
				else
					decoded_buf[decoded_buf_idx + i / 8] &= ~(BIT(i % 8));
			}
			decoded_buf_idx += DIV_ROUND_UP(data_len, 8);
			rd_idx += data_len;
			break;
		case CH347_CMD_SWD:
			if (data_len > sizeof(ch347_swd_context.recv_buf)) {
				LOG_ERROR("CH347 SWD recv buf too small");
				free(read_buf);
				return ERROR_FAIL;
			}
			memcpy(ch347_swd_context.recv_buf, &read_buf[rd_idx], data_len);
			ch347_swd_context.recv_len = data_len;
			rd_idx += data_len;
			break;
		default:
			LOG_ERROR("CH347 read command fail");
			free(read_buf);
			return ERROR_FAIL;
		}
	}

	free(read_buf);
	return ERROR_OK;
}

/**
 * @brief Used to put the data from the decoded buffer into the scan command fields
 *
 * @param decoded_buf Point to a buffer for the decoded data
 * @param decoded_buf_len length of the decoded_buf
 * @return ERROR_OK at success
 */
static int ch347_scan_data_to_fields(uint8_t *decoded_buf, int decoded_buf_len)
{
	int byte_offset = 0;
	struct ch347_scan *scan;
	struct ch347_scan *tmp;
	int bit_offset = 0;
	list_for_each_entry_safe(scan, tmp, &ch347.scan_queue, queue) {
		for (int i = 0; i < scan->fields_len; i++) {
			int num_bits = scan->fields[i].num_bits;
			LOG_DEBUG_IO("fields[%d].in_value[%d], read from bit offset: %d", i, num_bits, bit_offset);
			// only if we need the value
			if (scan->fields[i].in_value) {
				uint8_t *capture_buf = malloc(DIV_ROUND_UP(num_bits, 8));
				if (!capture_buf) {
					LOG_ERROR("malloc failed");
					return ERROR_FAIL;
				}
				uint8_t *captured = buf_set_buf(decoded_buf, bit_offset, capture_buf, 0, num_bits);

				if (LOG_LEVEL_IS(LOG_LVL_DEBUG_IO)) {
					char *str = buf_to_hex_str(captured, num_bits);
					LOG_DEBUG_IO("size=%d, buf=[%s]", num_bits, str);
					free(str);
				}

				buf_cpy(captured, scan->fields[i].in_value, num_bits);
				free(capture_buf);
			} else {
				LOG_DEBUG_IO("field skipped");
			}
			bit_offset += num_bits;
		}
		list_del(&scan->queue);
		free(scan);
		/* after one round of scan field processing the
			next data bits are read from the next data byte
			=> round up and calculate the next start bit */
		byte_offset = DIV_ROUND_UP(bit_offset, 8);
		bit_offset = byte_offset * 8;
	}

	if (byte_offset < decoded_buf_len)
		LOG_DEBUG("decoded %d of %d bytes", byte_offset, decoded_buf_len);

	return ERROR_OK;
}

/**
 * @brief Drops everything queued but not transmitted yet: the scratchpad,
 * the command queue and the scan fields waiting for data.
 * Used in error paths, so stale commands are not sent with the next queue
 * and stale scan fields (owned by the already freed jtag_command queue)
 * are not filled in later.
 */
static void ch347_queues_discard(void)
{
	struct ch347_cmd *cmd, *tmp_cmd;
	list_for_each_entry_safe(cmd, tmp_cmd, &ch347.cmd_queue, queue) {
		list_del(&cmd->queue);
		free(cmd->write_data);
		free(cmd);
	}

	ch347.scratchpad_cmd_type = 0;
	ch347.scratchpad_idx = 0;

	struct ch347_scan *scan, *tmp;
	list_for_each_entry_safe(scan, tmp, &ch347.scan_queue, queue) {
		list_del(&scan->queue);
		free(scan);
	}
}

/**
 * @brief Sends the write buffer via libusb
 * and if LARGER_PACK mode is active read also data back
 *
 * @return ERROR_OK at success
 */
static int ch347_cmd_transmit_queue_inner(void)
{
	// queue last command
	int retval = ch347_cmd_from_scratchpad();
	if (retval != ERROR_OK)
		return retval;

	// nothing to do! => done here
	if (list_empty(&ch347.cmd_queue))
		return ERROR_OK;

	// calculate the needed buffer length for all decoded bytes
	struct ch347_cmd *cmd;
	int decoded_buf_len = 0;
	list_for_each_entry(cmd, &ch347.cmd_queue, queue)
		if (cmd->read_len > 0 && ch347_cmd_has_variable_data_size(cmd->type))
			decoded_buf_len += DIV_ROUND_UP(cmd->tdo_bit_count, 8);

	// create the buffer for all decoded bytes
	uint8_t *decoded_buf = NULL;
	int decoded_buf_idx = 0;
	if (decoded_buf_len > 0) {
		decoded_buf = malloc(decoded_buf_len);
		if (!decoded_buf) {
			LOG_ERROR("malloc failed");
			return ERROR_FAIL;
		}
	}

	while (!list_empty(&ch347.cmd_queue)) {
		struct ch347_cmd *last_cmd = NULL;
		int total_len = 0;
		int total_tdo_count = 0;
		int bytes_to_write = 0;
		int max_len = ch347.max_len;

		list_for_each_entry(cmd, &ch347.cmd_queue, queue) {
			total_len += CH347_CMD_HEADER + cmd->write_data_len;
			total_tdo_count += cmd->tdo_bit_count;
			if (cmd->type == CH347_CMD_GPIO)
				max_len = USBC_PACKET_USBHS;
			// don't exceed max length or max TDO bit count
			if (total_len >= max_len || total_tdo_count >= HW_TDO_BUF_SIZE)
				break;
			// remember the last cmd to send and bytes to send
			last_cmd = cmd;
			bytes_to_write = total_len;
		}

		// sanity checks
		if (!last_cmd || bytes_to_write == 0) {
			LOG_ERROR("Nothing to send!");
			free(decoded_buf);
			return ERROR_FAIL;
		}

		// create the write buffer
		uint8_t *write_buf = malloc(bytes_to_write);
		if (!write_buf) {
			LOG_ERROR("malloc failed");
			free(decoded_buf);
			return ERROR_FAIL;
		}

		int idx = 0;
		int bytes_to_read = 0;
		int current_decoded_buf_len = 0;
		struct ch347_cmd *tmp;

		list_for_each_entry_safe(cmd, tmp, &ch347.cmd_queue, queue) {
			// copy command to buffer
			write_buf[idx++] = cmd->type;
			h_u16_to_le(&write_buf[idx], cmd->write_data_len);
			idx += 2;
			memcpy(&write_buf[idx], cmd->write_data, cmd->write_data_len);
			idx += cmd->write_data_len;
			// need to read something back?
			if (cmd->read_len > 0) {
				bytes_to_read += CH347_CMD_HEADER + cmd->read_len;
				if (ch347_cmd_has_variable_data_size(cmd->type))
					current_decoded_buf_len += DIV_ROUND_UP(cmd->tdo_bit_count, 8);
			}

			// cmd data no longer needed
			list_del(&cmd->queue);
			free(cmd->write_data);
			free(cmd);

			if (cmd == last_cmd)
				break;
		}

		// write data to device
		retval = ch347_write_data(write_buf, &idx);
		free(write_buf);
		if (retval != ERROR_OK)	{
			free(decoded_buf);
			return retval;
		}

		if (!bytes_to_read)
			continue;

		// Need only to execute a read without decoding the data to make the CH347 happy?
		if (!current_decoded_buf_len) {
			// read but don't decode anything
			retval = ch347_read_scan(NULL, 0, bytes_to_read);
		} else {
			retval = ch347_read_scan(&decoded_buf[decoded_buf_idx], current_decoded_buf_len, bytes_to_read);
			decoded_buf_idx += current_decoded_buf_len;
		}

		if (retval != ERROR_OK) {
			free(decoded_buf);
			return retval;
		}
	}

	// something decoded from the data read back from CH347?
	if (decoded_buf) {
		// put the decoded data into the scan fields
		retval = ch347_scan_data_to_fields(decoded_buf, decoded_buf_len);
		free(decoded_buf);
	}

	return retval;
}

/**
 * @brief Sends all queued commands and puts the data read back into the scan
 * fields. On error, everything not transmitted yet is discarded
 *
 * @return ERROR_OK at success
 */
static int ch347_cmd_transmit_queue(void)
{
	int retval = ch347_cmd_transmit_queue_inner();
	if (retval != ERROR_OK)
		ch347_queues_discard();

	return retval;
}

/**
 * @brief starts the next command in the scratchpad. If it's the same command type
 * it can concat the data bytes. no need to make a new command for this case
 *
 * @param type command type
 */
static int ch347_cmd_start_next(uint8_t type)
{
	// different command type or non chainable command?
	uint8_t prev_type = ch347.scratchpad_cmd_type;
	if (prev_type != type || !ch347_cmd_has_variable_data_size(type)) {
		// something written in the scratchpad? => store it as command
		if (prev_type != 0 && ch347.scratchpad_idx > 0) {
			int retval = ch347_cmd_from_scratchpad();
			if (retval != ERROR_OK)
				return retval;

			/* if the last queued command is not chainable we should send it immediately
				because e.g. the init commands can't be combined with any other command */
			if (ch347_is_single_cmd_type(prev_type)) {
				retval = ch347_cmd_transmit_queue();
				if (retval != ERROR_OK)
					return retval;
			}
		}

		/* before we can send non chainable command we should send all
			other commands because we can't send it together with other commands */
		if (ch347_is_single_cmd_type(type)) {
			int retval = ch347_cmd_transmit_queue();
			if (retval != ERROR_OK)
				return retval;
		}

		// store the next command type
		ch347.scratchpad_cmd_type = type;
	}

	return ERROR_OK;
}

/**
 * @brief queue the scan fields into the scan queue
 *
 * @param scan_fields array of scan field's for data from the device
 * @param scan_fields_len array length
 * @return ERROR_OK at success
 */
static int ch347_scan_queue_fields(struct scan_field *scan_fields, int scan_fields_len)
{
	// malloc for the scan struct
	struct ch347_scan *scan = malloc(sizeof(struct ch347_scan));
	if (!scan) {
		LOG_ERROR("malloc failed");
		return ERROR_FAIL;
	}

	scan->fields = scan_fields;
	scan->fields_len = scan_fields_len;
	list_add_tail(&scan->queue, &ch347.scan_queue);
	return ERROR_OK;
}

/**
 * @brief Function executes the single command and deliver one byte from the buffer
 * that's read back from USB
 *
 * @param byte returns result byte
 * @return ERROR_OK at success
 */
static int ch347_single_read_get_byte(uint8_t *byte)
{
	ch347.init_result_len = 0;
	int retval = ch347_cmd_transmit_queue();
	if (retval != ERROR_OK)
		return retval;

	if (ch347.init_result_len < 1) {
		LOG_ERROR("no init cmd result");
		return ERROR_FAIL;
	}

	*byte = ch347.init_result_buf[0];
	return ERROR_OK;
}

/**
 * @brief checks if the scratchpad is full. If it's full the function creates
 * a command from it and puts it into the command queue
 */
static void ch347_scratchpad_check_full(void)
{
	// if full create a new command in the queue
	if (ch347.scratchpad_idx == UCMDPKT_DATA_MAX_BYTES_USBHS) {
		uint8_t type = ch347.scratchpad_cmd_type;
		ch347_cmd_from_scratchpad();
		ch347.scratchpad_cmd_type = type;
	}
}

/**
 * @brief adds one byte to the scratchpad
 * if scratchpad is full after this byte the command will be created from the
 * scratchpad and the scratchpad is cleared for the next command
 *
 * @param byte add this byte
 * @return ERROR_OK at success
 */
static int ch347_scratchpad_add_byte(uint8_t byte)
{
	if (ch347.scratchpad_cmd_type == 0) {
		LOG_ERROR("call ch347_next_cmd first!");
		return ERROR_FAIL;
	}

	ch347.scratchpad[ch347.scratchpad_idx++] = byte;
	ch347_scratchpad_check_full();
	return ERROR_OK;
}

/**
 * @brief adds the output pin byte to the scratchpad
 * if scratchpad is full after this byte the command will be created from the
 * scratchpad and the scratchpad is cleared for the next command
 *
 * @return ERROR_OK at success
 */
static int ch347_scratchpad_add_pin_byte(void)
{
	uint8_t pins = ch347.tms_pin | ch347.tdi_pin | ch347.tck_pin | ch347.trst_pin;

	if (ch347_dedicated_srst_enabled)
		pins |= ch347.srst_pin;

	return ch347_scratchpad_add_byte(pins);
}

/**
 * @brief adds bytes from a buffer to the scratchpad
 * if scratchpad is full after this byte the command will be created from the
 * scratchpad and the scratchpad is cleared for the next command
 *
 * @param bytes add this bytes
 * @param count byte count
 *
 * @return ERROR_OK at success
 */
static int ch347_scratchpad_add_bytes(uint8_t *bytes, int count)
{
	if (ch347.scratchpad_cmd_type == 0) {
		LOG_ERROR("call ch347_next_cmd first!");
		return ERROR_FAIL;
	}

	if (!ch347_cmd_has_variable_data_size(ch347.scratchpad_cmd_type)
		&& count > UCMDPKT_DATA_MAX_BYTES_USBHS - ch347.scratchpad_idx) {
		LOG_ERROR("BUG: CH347 command 0x%02" PRIx8
				  " is not variable data size type and payload is too long",
				  ch347.scratchpad_cmd_type);
		return ERROR_FAIL;
	}

	int remaining = count;
	int bytes_idx = 0;
	while (remaining > 0) {
		int bytes_to_store = ch347.scratchpad_idx + remaining <= UCMDPKT_DATA_MAX_BYTES_USBHS ?
			remaining :	UCMDPKT_DATA_MAX_BYTES_USBHS - ch347.scratchpad_idx;

		if (bytes)
			memcpy(&ch347.scratchpad[ch347.scratchpad_idx], &bytes[bytes_idx], bytes_to_store);
		else
			memset(&ch347.scratchpad[ch347.scratchpad_idx], 0, bytes_to_store);

		ch347.scratchpad_idx += bytes_to_store;
		bytes_idx += bytes_to_store;
		remaining -= bytes_to_store;
		ch347_scratchpad_check_full();
	}

	return ERROR_OK;
}

/**
 * @brief Enable the dedicated SRST output on CH347F devices.
 *
 * The 0xE2 command and its payload come from WCH's fork, commit 4fd5026aabb6
 * ("feat: Add CH347F SRST configuration"):
 * https://github.com/WCHSoftGroup/ch347/commit/4fd5026aabb62b941317b5d156d0eeaef4c43108
 * That packet is {0xe2, 0x08, 0x00, 0x00, 0x81, ...}. The command queue adds
 * the command and the length, so only the eight payload bytes are sent here.
 * Bit 0 of the second payload byte selects the mode: 0x81 enables it and
 * 0x80 disables it. Disabling does not tristate pin 8, so 0x80 is never sent.
 *
 * The byte read back from PIN_SET has no documented meaning, so it is logged
 * and does not decide whether initialization succeeded.
 *
 * Pin 8 strongly drives low after the adapter powers up. PIN_SET only selects
 * the dedicated output; a JTAG pin byte should follow to set the level.
 *
 * @return ERROR_OK on success, or the original error code when the command
 * cannot be transmitted
 */
static int ch347_srst_enable(void)
{
	int retval = ch347_cmd_start_next(CH347_CMD_PIN_SET);
	if (retval != ERROR_OK)
		return retval;

	uint8_t cmd_data[] = {0x00, 0x81, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00};
	retval = ch347_scratchpad_add_bytes(cmd_data, ARRAY_SIZE(cmd_data));
	if (retval != ERROR_OK)
		return retval;

	uint8_t init_result;
	retval = ch347_single_read_get_byte(&init_result);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("CH347F PIN_SET result %02" PRIx8, init_result);

	ch347_dedicated_srst_enabled = true;
	return ERROR_OK;
}

/**
 * @brief Function used to change the TMS value at the
 * rising edge of TCK to switch its TAP state
 *
 * @param tms TMS value to be changed; true = output TMS high; false = output TMS low
 * @return ERROR_OK at success
 */
static int ch347_scratchpad_add_clock_tms(bool tms)
{
	ch347.tms_pin = tms ? TMS_H : TMS_L;
	ch347.tck_pin = TCK_L;
	int retval = ch347_scratchpad_add_pin_byte();
	if (retval != ERROR_OK)
		return retval;

	ch347.tck_pin = TCK_H;
	return ch347_scratchpad_add_pin_byte();
}

/**
 * @brief Function adds a certain amount of TCK pulses without changing the TMS pin
 *
 * @param count Amount of L/H TCK pulses to add
 * @return ERROR_OK at success
 */
static int ch347_scratchpad_add_stableclocks(int count)
{
	int retval = ch347_cmd_start_next(CH347_CMD_JTAG_BIT_OP);
	if (retval != ERROR_OK)
		return retval;

	bool tms = ch347.tms_pin == TMS_H;
	for (int i = 0; i < count; i++) {
		retval = ch347_scratchpad_add_clock_tms(tms);
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

/**
 * @brief Return TCK to its low idle level, if it isn't there already.
 * Does not generate a clock cycle.
 *
 * @return ERROR_OK at success
 */
static int ch347_scratchpad_add_tck_idle(void)
{
	if (ch347.tck_pin == TCK_L)
		return ERROR_OK;

	int retval = ch347_cmd_start_next(CH347_CMD_JTAG_BIT_OP);
	if (retval != ERROR_OK)
		return retval;

	ch347.tck_pin = TCK_L;
	return ch347_scratchpad_add_pin_byte();
}

/**
 * @brief Function that performs state switching by changing the value of TMS
 *
 * @param tms_value The TMS values that need to be switched form one byte of data in the switching order
 * @param step The number of bit values that need to be read from the tms_value value
 * @param skip Count from the skip bit of tms_value to step
 * @return ERROR_OK at success
 */
static int ch347_scratchpad_add_tms_change(const uint8_t *tms_value, int step, int skip)
{
	LOG_DEBUG_IO("TMS Value: %02x..., step = %d, skip = %d", tms_value[0], step, skip);

	int retval = ch347_cmd_start_next(CH347_CMD_JTAG_BIT_OP);
	if (retval != ERROR_OK)
		return retval;

	for (int i = skip; i < step; i++) {
		retval = ch347_scratchpad_add_clock_tms((tms_value[i / 8] >> (i % 8)) & BIT(0));
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

/**
 * @brief Obtain the current Tap status and switch to the status TMS value passed down by cmd
 *
 * @param cmd Upper layer transfer command parameters
 * @return ERROR_OK at success; ERROR_JTAG_TRANSITION_INVALID if no transition is possible
 */
static int ch347_scratchpad_add_move_path(struct pathmove_command *cmd)
{
	LOG_DEBUG_IO("num_states=%d, last_state=%d", cmd->num_states, cmd->path[cmd->num_states - 1]);

	int retval = ch347_cmd_start_next(CH347_CMD_JTAG_BIT_OP);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int i = 0; i < cmd->num_states; i++) {
		if (tap_state_transition(tap_get_state(), false) == cmd->path[i]) {
			retval = ch347_scratchpad_add_clock_tms(0);
			if (retval != ERROR_OK)
				return retval;
		} else if (tap_state_transition(tap_get_state(), true) == cmd->path[i]) {
			retval = ch347_scratchpad_add_clock_tms(1);
			if (retval != ERROR_OK)
				return retval;
		} else {
			LOG_ERROR("No transition possible!");
			return ERROR_JTAG_TRANSITION_INVALID;
		}
		tap_set_state(cmd->path[i]);
	}

	return ERROR_OK;
}

/**
 * @brief Toggle the tap state to the target state
 *
 * @param state Pre switch target path
 * @param skip Number of digits to skip
 * @return ERROR_OK at success
 */
static int ch347_scratchpad_add_move_state(enum tap_state state, int skip)
{
	uint8_t tms_scan;
	int tms_len;

	LOG_DEBUG_IO("from %s to %s", tap_state_name(tap_get_state()), tap_state_name(state));
	// don't do anything if we are already in the right state; but do execute always the TAP_RESET
	if (tap_get_state() == state && state != TAP_RESET)
		return ERROR_OK;

	tms_scan = tap_get_tms_path(tap_get_state(), state);
	tms_len = tap_get_tms_path_len(tap_get_state(), state);
	int retval = ch347_scratchpad_add_tms_change(&tms_scan, tms_len, skip);
	tap_set_state(state);
	return retval;
}

/**
 * @brief CH347 Batch read/write function
 *
 * @param cmd The scan command
 * @param bits Read and write data this time
 * @param bits_len Incoming data length in bits
 * @param scan The transmission method of incoming data to determine whether to perform data reading
 * @return ERROR_OK at success
 */
static int ch347_scratchpad_add_write_read(struct scan_command *cmd, uint8_t *bits, int bits_len, enum scan_type scan)
{
	// the bits and bytes to transfer
	int byte_count = bits_len / 8;
	int bit_count = bits_len % 8;

	// only bytes are not possible because we need to set TMS high for the last bit
	if (byte_count > 0 && bit_count == 0) {
		// make one byte to eight bits
		byte_count--;
		bit_count = 8;
	}

	// in bitwise mode only bits are allowed
	if (ch347.use_bitwise_mode) {
		byte_count = 0;
		bit_count = bits_len;
	}

	bool is_read = (scan == SCAN_IN || scan == SCAN_IO);

	// if we need to send bytes
	if (byte_count > 0) {
		// a byte shift started with TCK high corrupts the scan
		int retval = ch347_scratchpad_add_tck_idle();
		if (retval != ERROR_OK)
			return retval;

		// start the next cmd and copy the data out bytes to it
		retval = ch347_cmd_start_next(is_read ? CH347_CMD_JTAG_DATA_SHIFT_RD : CH347_CMD_JTAG_DATA_SHIFT);
		if (retval != ERROR_OK)
			return retval;

		if (bits)
			retval = ch347_scratchpad_add_bytes(bits, byte_count);
		else
			retval = ch347_scratchpad_add_bytes(NULL, byte_count);

		if (retval != ERROR_OK)
			return retval;
	}

	// bits are always need to send; no possibility to not send bits
	int retval = ch347_cmd_start_next(is_read ? CH347_CMD_JTAG_BIT_OP_RD : CH347_CMD_JTAG_BIT_OP);
	if (retval != ERROR_OK)
		return retval;

	ch347.tms_pin = TMS_L;
	ch347.tdi_pin = TDI_L;

	for (int i = 0; i < bit_count; i++) {
		if (bits)
			ch347.tdi_pin = ((bits[byte_count + i / 8] >> i % 8) & BIT(0)) ? TDI_H : TDI_L;

		// for the last bit set TMS high to exit the shift state
		if (i + 1 == bit_count)
			ch347.tms_pin = TMS_H;

		ch347.tck_pin = TCK_L;
		retval = ch347_scratchpad_add_pin_byte();
		if (retval != ERROR_OK)
			return retval;

		ch347.tck_pin = TCK_H;
		retval = ch347_scratchpad_add_pin_byte();
		if (retval != ERROR_OK)
			return retval;

		/* cut the package after each MAX_BITS_PER_BIT_OP bits because it
			needs a dividable by 8 bits package */
		if (i > 0 && (i + 1) % MAX_BITS_PER_BIT_OP == 0) {
			retval = ch347_cmd_from_scratchpad();
			if (retval != ERROR_OK)
				return retval;

			retval = ch347_cmd_start_next(is_read ? CH347_CMD_JTAG_BIT_OP_RD : CH347_CMD_JTAG_BIT_OP);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	// if read is involved we need to queue the scan fields
	if (is_read)
		return ch347_scan_queue_fields(cmd->fields, cmd->num_fields);

	return ERROR_OK;
}

/**
 * @brief Toggle the Tap state to run test/idle
 *
 * @param cycles how many clock cycles for output
 * @param state JTAG end state
 * @return ERROR_OK at success
 */
static int ch347_scratchpad_add_run_test(int cycles, enum tap_state state)
{
	LOG_DEBUG_IO("cycles=%d, end_state=%d", cycles, state);
	int retval;
	if (tap_get_state() != TAP_IDLE) {
		retval = ch347_scratchpad_add_move_state(TAP_IDLE, 0);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = ch347_scratchpad_add_stableclocks(cycles);
	if (retval != ERROR_OK)
		return retval;

	return ch347_scratchpad_add_move_state(state, 0);
}

/**
 * @brief Switch to SHIFT-DR or SHIFT-IR status for scanning
 *
 * @param cmd Upper layer transfer command parameters
 * @return ERROR_OK at success
 */
static int ch347_scratchpad_add_scan(struct scan_command *cmd)
{
	static const char *const type2str[] = {"", "SCAN_IN", "SCAN_OUT", "SCAN_IO"};

	enum scan_type type = jtag_scan_type(cmd);
	uint8_t *buf = NULL;
	int scan_bits = jtag_build_buffer(cmd, &buf);

	// add a move to IRSHIFT or DRSHIFT state
	int retval;
	if (cmd->ir_scan)
		retval = ch347_scratchpad_add_move_state(TAP_IRSHIFT, 0);
	else
		retval = ch347_scratchpad_add_move_state(TAP_DRSHIFT, 0);

	if (retval != ERROR_OK)	{
		free(buf);
		return retval;
	}

	if (LOG_LEVEL_IS(LOG_LVL_DEBUG_IO)) {
		char *log_buf = buf_to_hex_str(buf, scan_bits);
		LOG_DEBUG_IO("scan=%s, type=%s, bits=%d, buf=[%s], end_state=%d",
			 cmd->ir_scan ? "IRSCAN" : "DRSCAN",
			 type2str[type],
			 scan_bits, log_buf, cmd->end_state);
		free(log_buf);
	}

	retval = ch347_scratchpad_add_write_read(cmd, buf, scan_bits, type);
	free(buf);
	if (retval != ERROR_OK)
		return retval;

	// add a move to the final state
	return ch347_scratchpad_add_move_state(cmd->end_state, 1);
}

/**
 * @brief GPIO number of the TRST pin
 *
 * @return GPIO number of the TRST pin of the chip variant
 */
static inline unsigned int ch347_trst_gpio(void)
{
	return ch347.chip_variant == CH347F ? CH347F_TRST_GPIO : CH347T_TRST_GPIO;
}

/**
 * @brief Translates an output level into a GPIO command byte for a drive mode
 *
 * The CH347 has no open drain output driver, so it gets emulated by switching
 * the pin to input instead of driving the inactive level. The line then relies
 * on an external pull resistor or on the target driving it.
 *
 * @param level true to output a high level, false for a low level
 * @param drive output drive mode to emulate
 * @return one of GPIO_SET_L, GPIO_SET_H or GPIO_SET_INPUT
 */
static uint8_t ch347_gpio_drive_value(bool level, enum adapter_gpio_drive_mode drive)
{
	switch (drive) {
	case ADAPTER_GPIO_DRIVE_MODE_OPEN_DRAIN:
		return level ? GPIO_SET_INPUT : GPIO_SET_L;
	case ADAPTER_GPIO_DRIVE_MODE_OPEN_SOURCE:
		return level ? GPIO_SET_H : GPIO_SET_INPUT;
	case ADAPTER_GPIO_DRIVE_MODE_PUSH_PULL:
		break;
	}

	return level ? GPIO_SET_H : GPIO_SET_L;
}

static int ch347_check_gpio_unique(enum adapter_gpio_config_index gpio_idx,
								   unsigned int *gpios_used)
{
	const struct adapter_gpio_config *gpio_config = &adapter_gpio_get_config()[gpio_idx];
	if (gpio_config->gpio_num == ADAPTER_GPIO_NOT_SET)
		return ERROR_OK;

	if (gpio_config->gpio_num >= GPIO_CNT_EXT)
		return ERROR_OK;	// invalid GPIO num will be reported later

	if (BIT(gpio_config->gpio_num) & *gpios_used) {
		LOG_ERROR("%s GPIO %u already used",
				  adapter_gpio_get_name(gpio_idx),
				  gpio_config->gpio_num);
		return ERROR_FAIL;
	}

	*gpios_used |= BIT(gpio_config->gpio_num);
	return ERROR_OK;
}

static unsigned int ch347_get_gpio_num_or_default(enum adapter_gpio_config_index gpio_idx)
{
	const struct adapter_gpio_config *gpio_config = &adapter_gpio_get_config()[gpio_idx];
	if (gpio_config->gpio_num == ADAPTER_GPIO_NOT_SET) {
		if (gpio_idx == ADAPTER_GPIO_IDX_SRST)
			return ch347.srst_default_gpio;

		if (gpio_idx == ADAPTER_GPIO_IDX_TRST)
			return ch347.trst_default_gpio;
	}
	return gpio_config->gpio_num;
}

static int ch347_set_gpio(enum adapter_gpio_config_index gpio_idx, bool active,
						   uint8_t *gpios_data, uint8_t *bit_op)
{
	const struct adapter_gpio_config *gpio_config = &adapter_gpio_get_config()[gpio_idx];
	unsigned int gpio_num = ch347_get_gpio_num_or_default(gpio_idx);
	const struct ch347_bit_op *gpio_to_bit_op = ch347.chip_variant == CH347F ?
					ch347f_gpio_to_bit_op : ch347t_gpio_to_bit_op;
	bool value = active ^ gpio_config->active_low;
	uint8_t gpio_val = ch347_gpio_drive_value(value, gpio_config->drive);

	/* Using both CH347_CMD_GPIO and CH347_CMD_JTAG_BIT_OP is tricky
	 * but the only way to use GPIO capabilities (simulated open-drain)
	 * together with JTAG and/or with the CH347F dedicated SRST pin.
	 *
	 * CH347T Chip version=2.41 does not support CH347_CMD_GPIO at all
	 * - GPIO support blocked by cmd_gpio_allowed = false.
	 *
	 * CH347T Chip version=4.41 honours CH347_CMD_GPIO setting
	 * to input and prevents CH347_CMD_JTAG_BIT_OP from changing the same
	 * pin to output. Pin previously set by CH347_CMD_GPIO to output
	 * gets updated by CH347_CMD_JTAG_BIT_OP.
	 *
	 * CH347T Chip version=5.44 and CH347F Chip version=1.20:
	 * The pin previously set by CH347_CMD_GPIO to input works as open-drain
	 * with respect to CH347_CMD_JTAG_BIT_OP. Bit op setting to L propagates
	 * low level to the pin, setting to H switches the pin to Hi-Z.
	 * Pin previously set by CH347_CMD_GPIO to output gets updated
	 * by CH347_CMD_JTAG_BIT_OP.
	 *
	 * It means for all pins controlled by both CH347_CMD_JTAG_BIT_OP
	 * and CH347_CMD_GPIO: the bit keeping the pin state
	 * for CH347_CMD_JTAG_BIT_OP must follow CH347_CMD_GPIO output state
	 * and must be set to H if GPIO is set to input. */
	if (gpio_num < GPIO_CNT && ch347.cmd_gpio_allowed) {
		gpios_data[gpio_num] = gpio_val;

		// set the corresponding bit for CH347_CMD_JTAG_BIT_OP
		if (gpio_to_bit_op[gpio_num].ptr) {
			if (gpio_val == GPIO_SET_INPUT)
				value = true;	/* input setting by CH347_CMD_GPIO keeps
								 * precedence while CH347_CMD_JTAG_BIT_OP
								 * sets the same pin to output H */
			*gpio_to_bit_op[gpio_num].ptr = value ? gpio_to_bit_op[gpio_num].bit : 0;
		}

		return ERROR_OK;
	}

	if (gpio_num < GPIO_CNT_EXT) {
		if (gpio_to_bit_op[gpio_num].ptr) {
			*gpio_to_bit_op[gpio_num].ptr = value ? gpio_to_bit_op[gpio_num].bit : 0;
			*bit_op |= gpio_to_bit_op[gpio_num].bit;

			if (gpio_val == GPIO_SET_INPUT && gpio_to_bit_op[gpio_num].bit & ~ch347.bit_op_log_once) {
				LOG_WARNING("%s drive mode ignored, GPIO %u is push-pull only",
							adapter_gpio_get_name(gpio_idx),
							gpio_num);
				ch347.bit_op_log_once |= gpio_to_bit_op[gpio_num].bit;
			}
			return ERROR_OK;
		}
	}

	// Settings failed, reaction is GPIO specific
	if (gpio_idx == ADAPTER_GPIO_IDX_LED) {
		if (gpio_num == ADAPTER_GPIO_NOT_SET)
			return ERROR_OK;
		else if (ch347.led_log_once)
			return ERROR_FAIL;

		ch347.led_log_once = true;
	} else if (!active) {
		return ERROR_OK;
	}
	if (gpio_num == ADAPTER_GPIO_NOT_SET)
		LOG_ERROR("%s GPIO not configured", adapter_gpio_get_name(gpio_idx));
	else if (gpio_num < GPIO_CNT && !ch347.cmd_gpio_allowed)
		LOG_ERROR("Too old CH347 version, %s GPIO configuration not supported",
				adapter_gpio_get_name(gpio_idx));
	else
		LOG_ERROR("%s GPIO configuration invalid", adapter_gpio_get_name(gpio_idx));

	return ERROR_FAIL;
}

static int ch347_init_gpio(enum adapter_gpio_config_index gpio_idx,
							uint8_t *gpios_data, uint8_t *bit_op)
{
	const struct adapter_gpio_config *gpio_config = &adapter_gpio_get_config()[gpio_idx];
	if (gpio_config->init_state == ADAPTER_GPIO_INIT_STATE_INPUT) {
		unsigned int gpio_num = ch347_get_gpio_num_or_default(gpio_idx);
		if (gpio_num < GPIO_CNT && ch347.cmd_gpio_allowed) {
			gpios_data[gpio_num] = GPIO_SET_INPUT;
			return ERROR_OK;
		}
		LOG_WARNING("Cannot set %s GPIO to initial input state",
					adapter_gpio_get_name(gpio_idx));
	}
	bool active = gpio_config->init_state == ADAPTER_GPIO_INIT_STATE_ACTIVE;
	return ch347_set_gpio(gpio_idx, active, gpios_data, bit_op);
}

static int ch347_exit_gpio(enum adapter_gpio_config_index gpio_idx,
							uint8_t *gpios_data, uint8_t *bit_op)
{
	const struct adapter_gpio_config *gpio_config = &adapter_gpio_get_config()[gpio_idx];
	if (gpio_config->exit_state == ADAPTER_GPIO_EXIT_STATE_NO_CHANGE)
		return ERROR_OK;

	if (gpio_config->exit_state == ADAPTER_GPIO_EXIT_STATE_INPUT) {
		unsigned int gpio_num = ch347_get_gpio_num_or_default(gpio_idx);
		if (gpio_num < GPIO_CNT && ch347.cmd_gpio_allowed) {
			gpios_data[gpio_num] = GPIO_SET_INPUT;
			return ERROR_OK;
		}
		LOG_WARNING("Cannot set %s GPIO to exit input state",
					adapter_gpio_get_name(gpio_idx));
	}
	bool active = gpio_config->exit_state == ADAPTER_GPIO_EXIT_STATE_ACTIVE;
	return ch347_set_gpio(gpio_idx, active, gpios_data, bit_op);
}

/**
 * @brief Update GPIOS or JTAG_BIT_OP controlled outputs
 * Avoid failing on the first error - the second part may succeed
 *
 * @return ERROR_OK at success or the first error
 */
static int ch347_gpios_update(uint8_t *gpios_data, bool bit_op, bool flush_now)
{
	int retval1 = ERROR_OK;
	if (bit_op) {
		retval1 = ch347_cmd_start_next(CH347_CMD_JTAG_BIT_OP);
		if (retval1 == ERROR_OK)
			retval1 = ch347_scratchpad_add_pin_byte();
	}

	unsigned int i;
	for (i = 0; i < GPIO_CNT; i++)
		if (gpios_data[i])
			break;

	bool any_gpio_set = i < GPIO_CNT;
	int retval2 = ERROR_OK;
	if (any_gpio_set) {
		retval2 = ch347_cmd_start_next(CH347_CMD_GPIO);
		if (retval2 == ERROR_OK)
			retval2 = ch347_scratchpad_add_bytes(gpios_data, GPIO_CNT);
	}
	if (retval1 == ERROR_OK)
		retval1 = retval2;

	if (flush_now && (bit_op || any_gpio_set))
		retval2 = ch347_cmd_transmit_queue();

	return retval1 == ERROR_OK ? retval2 : retval1;
}

/**
 * @brief Turn the activity LED on or off
 * Errors are intentionally ignored.
 */
static void ch347_activity_led_set(bool led_state, bool flush_now)
{
	uint8_t gpios[GPIO_CNT] = { 0 };
	uint8_t bit_op = 0;

	ch347_set_gpio(ADAPTER_GPIO_IDX_LED, led_state, gpios, &bit_op);
	int retval = ch347_gpios_update(gpios, bit_op, flush_now);
	if (retval != ERROR_OK)
		ch347.led_is_on = false;
	else
		ch347.led_is_on = led_state;
}

/**
 * @brief Control (assert/deassert) the signals SRST and TRST on the interface.
 *
 * Employs 'adapter gpio' configuration for both signals.
 * Avoid failing on the first error - the second part may succeed
 *
 * @param trst 1 to assert TRST, 0 to deassert TRST.
 * @param srst 1 to assert SRST, 0 to deassert SRST.
 * @return error code propagated from CH347_CMD_JTAG_BIT_OP or CH347_CMD_GPIO
 * or ERROR_OK
 */
static int ch347_reset(int trst, int srst)
{
	LOG_DEBUG_IO("reset trst: %i srst %i", trst, srst);

	uint8_t gpios[GPIO_CNT] = { 0 };
	uint8_t bit_op = 0;
	int retval1 = ch347_set_gpio(ADAPTER_GPIO_IDX_SRST, srst, gpios, &bit_op);
	int retval2 = ch347_set_gpio(ADAPTER_GPIO_IDX_TRST, trst, gpios, &bit_op);
	if (retval1 == ERROR_OK)
		retval1 = retval2;

	retval2 = ch347_gpios_update(gpios, bit_op, true);
	return retval1 != ERROR_OK ? retval1 : retval2;
}

/**
 * @brief Flushes the command buffer and sleeps for a specific timespan
 *
 * @param us Sleep time in microseconds
 * @return ERROR_OK at success
 */
static int ch347_sleep(int us)
{
	LOG_DEBUG_IO("us=%d", us);
	int retval = ch347_scratchpad_add_tck_idle();
	if (retval == ERROR_OK)
		retval = ch347_cmd_transmit_queue();
	jtag_sleep(us);
	return retval;
}

/**
 * @brief Executes the command queue
 *
 * @return Success returns ERROR_OK
 */
static int ch347_execute_queue(struct jtag_command *cmd_queue)
{
	struct jtag_command *cmd = cmd_queue;
	int retval = ERROR_OK;

	ch347_activity_led_set(true, false);

	while (retval == ERROR_OK && cmd) {
		switch (cmd->type) {
		case JTAG_RUNTEST:
			retval = ch347_scratchpad_add_run_test(cmd->cmd.runtest->num_cycles, cmd->cmd.runtest->end_state);
			break;
		case JTAG_STABLECLOCKS:
			retval = ch347_scratchpad_add_stableclocks(cmd->cmd.stableclocks->num_cycles);
			break;
		case JTAG_TLR_RESET:
			retval = ch347_scratchpad_add_move_state(cmd->cmd.statemove->end_state, 0);
			break;
		case JTAG_PATHMOVE:
			retval = ch347_scratchpad_add_move_path(cmd->cmd.pathmove);
			break;
		case JTAG_TMS:
			retval = ch347_scratchpad_add_tms_change(cmd->cmd.tms->bits, cmd->cmd.tms->num_bits, 0);
			break;
		case JTAG_SLEEP:
			retval = ch347_sleep(cmd->cmd.sleep->us);
			break;
		case JTAG_SCAN:
			retval = ch347_scratchpad_add_scan(cmd->cmd.scan);
			break;
		default:
			LOG_ERROR("BUG: unknown JTAG command type 0x%X", cmd->type);
			retval = ERROR_FAIL;
			break;
		}

		cmd = cmd->next;
	}

	if (retval == ERROR_OK)
		retval = ch347_scratchpad_add_tck_idle();

	/* drop the partially queued commands, the LED off goes with the next queue */
	if (retval != ERROR_OK)
		ch347_queues_discard();

	ch347_activity_led_set(false, false);

	if (retval != ERROR_OK)
		return retval;

	retval = ch347_cmd_transmit_queue();
	return retval;
}

/**
 * @brief opens the CH347 device via libusb driver
 *
 * @return ERROR_OK on success
 */
static int ch347_open_device(void)
{
	const uint16_t *ch347_vids = default_ch347_vids;
	const uint16_t *ch347_pids = default_ch347_pids;

	if (adapter_usb_get_vids()[0] != 0) {
		ch347_vids = adapter_usb_get_vids();
		ch347_pids = adapter_usb_get_pids();
	}

	int retval = jtag_libusb_open(ch347_vids, ch347_pids,
		adapter_usb_get_product_name(), &ch347_handle, NULL);
	if (retval != ERROR_OK)	{
		char error_message[256];
		snprintf(error_message, sizeof(error_message), "CH347 not found. Tried VID/PID pairs: ");
		for (int i = 0; ch347_vids[i] != 0; i++)
			snprintf(error_message + strlen(error_message), sizeof(error_message) - strlen(error_message),
				"%04x:%04x ", ch347_vids[i], ch347_pids[i]);

		LOG_ERROR("%s", error_message);
		return retval;
	}

	struct libusb_device_descriptor ch347_device_descriptor;
	libusb_device *device = libusb_get_device(ch347_handle);
	if (!device) {
		LOG_ERROR("CH347 error calling libusb_get_device");
		jtag_libusb_close(ch347_handle);
		return ERROR_FAIL;
	}

	retval = libusb_get_device_descriptor(device, &ch347_device_descriptor);
	if (retval != ERROR_OK) {
		LOG_ERROR("CH347 error getting device descriptor: %s", libusb_error_name(retval));
		jtag_libusb_close(ch347_handle);
		return retval;
	}

	// Fix for CH347F on ARM64
	// Force set configuration 1 (required on some ARM64 platforms)
	retval = libusb_set_configuration(ch347_handle, 1);
	if (retval != LIBUSB_SUCCESS && retval != LIBUSB_ERROR_BUSY)
		LOG_WARNING("CH347 set_configuration failed: %s", libusb_error_name(retval));

	// CH347T / CH347F detection
	// if we can claim interface 4 we found a CH347F chip; if we can claim interface 2 we found CH347T chip
	retval = libusb_claim_interface(ch347_handle, CH347F_MPHSI_INTERFACE);
	if (retval != ERROR_OK)	{
		retval = libusb_claim_interface(ch347_handle, CH347T_MPHSI_INTERFACE);
		if (retval != ERROR_OK)	{
			LOG_ERROR("CH347 unable to claim interface: %s", libusb_error_name(retval));
			jtag_libusb_close(ch347_handle);
			return retval;
		}
		ch347.chip_variant = CH347T;
	}	else {
		ch347.chip_variant = CH347F;
	}

	char firmware_version;
	retval = jtag_libusb_control_transfer(ch347_handle,
		LIBUSB_ENDPOINT_IN | LIBUSB_REQUEST_TYPE_VENDOR | LIBUSB_RECIPIENT_DEVICE,
		VENDOR_VERSION, 0, 0, &firmware_version, sizeof(firmware_version),
		USB_WRITE_TIMEOUT, NULL);
	if (retval != ERROR_OK) {
		LOG_ERROR("CH347 unable to get firmware version");
		jtag_libusb_close(ch347_handle);
		return retval;
	}

	char manufacturer[256 + 1];
	if (libusb_get_string_descriptor_ascii(ch347_handle, ch347_device_descriptor.iManufacturer,
		(unsigned char *)manufacturer, sizeof(manufacturer) - 1) < 0) {
		strcpy(manufacturer, "(unknown)");
	}
	char product[256 + 1];
	if (libusb_get_string_descriptor_ascii(ch347_handle, ch347_device_descriptor.iProduct,
		(unsigned char *)product, sizeof(product) - 1) < 0) {
		strcpy(product, "(unknown)");
	}
	char serial_number[256 + 1];
	if (libusb_get_string_descriptor_ascii(ch347_handle, ch347_device_descriptor.iSerialNumber,
		(unsigned char *)serial_number, sizeof(serial_number) - 1) < 0) {
		strcpy(serial_number, "(unknown)");
	}

	LOG_INFO("CH347%c %s from vendor %s with serial number %s found. (Chip version=%X.%02X, Firmware=0x%02X)",
		ch347.chip_variant == CH347T ? 'T' : ch347.chip_variant == CH347F ? 'F' : '?',
		product,
		manufacturer,
		serial_number,
		(ch347_device_descriptor.bcdDevice >> 8) & 0xFF,
		ch347_device_descriptor.bcdDevice & 0xFF,
		firmware_version);

	ch347.use_bitwise_mode = false;
	ch347.cmd_gpio_allowed = true;

	if (ch347.chip_variant == CH347T) {
		if (ch347_device_descriptor.bcdDevice < BYTEWISE_MODE_VERSION) {
			ch347.use_bitwise_mode = true;
			LOG_INFO("CH347T old version of the chip, JTAG only working in bitwise mode. For bytewise mode at least version %X.%X is needed.",
				(BYTEWISE_MODE_VERSION >> 8) & 0xFF,
				BYTEWISE_MODE_VERSION & 0xFF);
		}

		if (ch347_device_descriptor.bcdDevice < 0x441)
			ch347.cmd_gpio_allowed = false;

		if (swd_mode) {
			ch347.swclk_5mhz_supported = ch347_device_descriptor.bcdDevice >= 0x544;

			if (ch347_device_descriptor.bcdDevice < 0x441)
				LOG_WARNING("CH347T version older than 4.41 probably does not support SWD transport");

			if (ch347_device_descriptor.bcdDevice == 0x441)
				LOG_WARNING("If CH347T version 4.41 cannot connect or SWD fails often, insert a resistor to SWDIO circuit");

		} else if (ch347_device_descriptor.bcdDevice == 0x241) {
			LOG_WARNING("CH347T version 2.41 has very weird clock timing, may not work with a slower JTAG device");
		}

		if (ch347_device_descriptor.bcdDevice < 0x544)
			LOG_INFO("Please upgrade CH347T firmware to a production version >= 5.44");

	} else if (ch347.chip_variant == CH347F) {
		ch347.swclk_5mhz_supported = ch347_device_descriptor.bcdDevice >= 0x101;

		if (ch347_device_descriptor.bcdDevice < 0x101)
			LOG_INFO("Please upgrade CH347F firmware to a production version >= 1.1");
	}

	return ERROR_OK;
}

/**
 * @brief CH347 Device Release Function
 *
 * @return ERROR_OK on success
 */
static int ch347_quit(void)
{
	int retval;
	uint8_t gpios[GPIO_CNT] = { 0 };
	uint8_t bit_op = 0;
	ch347_exit_gpio(ADAPTER_GPIO_IDX_SRST, gpios, &bit_op);
	ch347_exit_gpio(ADAPTER_GPIO_IDX_TRST, gpios, &bit_op);
	ch347_exit_gpio(ADAPTER_GPIO_IDX_LED, gpios, &bit_op);

	retval = ch347_gpios_update(gpios, bit_op, true);

	jtag_libusb_close(ch347_handle);
	LOG_DEBUG("CH347 close");
	return retval;
}

/**
 * @brief Sends the CH347_CMD_JTAG_INIT (D0) command to get the JTAG
 * interface initialized with the speed index or asking with special clock index 9
 * for the LARGER_PACK mode support
 *
 * @param clock_index Clock index; special clock index 9 is used for checking
 * if the device supports the LARGER_PACK mode
 * @param supports_larger_pack_mode returns true if the device supports the LARGER_PACK mode
 * for newer CH347 devices; false if the device supports only the older STANDARD_PACK mode
 * @return ERROR_OK on success
 */
static int ch347_adapter_init(uint8_t clock_index, bool *supports_larger_pack_mode)
{
	int retval = ch347_cmd_start_next(CH347_CMD_JTAG_INIT);
	if (retval != ERROR_OK)
		return retval;

	retval = ch347_scratchpad_add_byte(0);
	if (retval != ERROR_OK)
		return retval;

	retval = ch347_scratchpad_add_byte(clock_index);
	if (retval != ERROR_OK)
		return retval;

	for (int i = 0; i < 4; i++)	{
		retval = ch347_scratchpad_add_pin_byte();
		if (retval != ERROR_OK)
			return retval;
	}

	uint8_t mode;
	retval = ch347_single_read_get_byte(&mode);
	if (retval != ERROR_OK)
		return retval;

	*supports_larger_pack_mode = mode != 0;
	return ERROR_OK;
}

/**
 * @brief Sends the CH347_CMD_JTAG_INIT (D0) command to ask the JTAG
 * interface with special clock index 9 for the LARGER_PACK mode support
 *
 * @param supports_larger_pack_mode returns true if the device supports the LARGER_PACK mode
 * for newer CH347 devices; false if the device supports only the older STANDARD_PACK mode
 * @return ERROR_OK on success
 */
static int ch347_adapter_supports_larger_pack_mode(bool *supports_larger_pack_mode)
{
	return ch347_adapter_init(CH347_CMD_INIT_GET_MODE_CLOCK_INDEX_VALUE, supports_larger_pack_mode);
}

/**
 * @brief Sends the CH347_CMD_JTAG_INIT (D0) command to get the JTAG
 * interface initialized with the speed index
 *
 * @param clock_index Clock index
 * @return ERROR_OK on success
 */
static int ch347_adapter_set_speed(uint8_t clock_index)
{
	bool unused;
	return ch347_adapter_init(clock_index, &unused);
}

/**
 * @brief swd init function
 * @param clock_divisor Divisor of base SWD frequency 1 MHz or 0 for fast 5 MHz clock
 *
 * @return ERROR_OK on success
 */
static int ch347_swd_init_cmd(uint8_t clock_divisor)
{
	int retval = ch347_cmd_start_next(CH347_CMD_SWD_INIT);
	if (retval != ERROR_OK)
		return retval;

	uint8_t cmd_data[] = {0x40, 0x42, 0x0f, 0x00, clock_divisor, 0x00, 0x00, 0x00 };
	retval = ch347_scratchpad_add_bytes(cmd_data, ARRAY_SIZE(cmd_data));
	if (retval != ERROR_OK)
		return retval;

	/* TODO: CH347_CMD_SWD_INIT reads one data byte.
			But how can we decide if SWD init was successfully executed?
			Return an error code if init was failed */
	uint8_t init_result = 0;
	retval = ch347_single_read_get_byte(&init_result);
	LOG_DEBUG("SWD init clk div %" PRIu8 ", result %02" PRIx8,
			  clock_divisor, init_result);
	ch347_swd_context.clk_divisor = clock_divisor;
	return retval;
}

/**
 * @brief Initializes the JTAG interface and set CH347 TCK frequency
 *
 * @param speed_index speed index for JTAG_INIT command
 * @return Success returns ERROR_OK, failed returns ERROR_FAIL
 */
static int ch347_speed_set(int speed_index)
{
	if (swd_mode)
		return ch347_swd_init_cmd(speed_index);

	int retval = ch347_adapter_set_speed(speed_index);
	if (retval != ERROR_OK) {
		LOG_ERROR("Couldn't set CH347 speed");
		return retval;
	}

	return ERROR_OK;
}

/**
 * @brief inits ch347.pack_size and ch347.max_len
 *
 * @return ERROR_OK on success
 */
static int ch347_init_pack_size(void)
{
	// already set?
	if (ch347.pack_size != UNSET)
		return ERROR_OK;

	// set the lower limit for starting
	ch347.max_len = MIN(USBC_PACKET_USBHS, LARGER_PACK_MAX_SIZE);
	bool supports_larger_pack_mode;
	int retval = ch347_adapter_supports_larger_pack_mode(&supports_larger_pack_mode);
	if (retval != ERROR_OK)
		return retval;

	ch347.pack_size = supports_larger_pack_mode ? LARGER_PACK : STANDARD_PACK;
	ch347.max_len = ch347.pack_size == STANDARD_PACK || ch347.use_bitwise_mode ?
		USBC_PACKET_USBHS : LARGER_PACK_MAX_SIZE;
	return ERROR_OK;
}

/**
 * @brief returns the speed in kHz for the given speed index
 *
 * @param speed_idx CH347 speed index (JTAG) or clock divisor (SWD)
 * @param khz Output of the speed in kHz
 * @return ERROR_OK on success
 */
static int ch347_speed_get(int speed_idx, int *khz)
{
	if (swd_mode) {
		if (speed_idx)
			*khz = DIV_ROUND_UP(CH347_SWD_CLOCK_BASE, speed_idx);
		else
			*khz = CH347_SWD_CLOCK_MAX;
		return ERROR_OK;
	}

	int retval = ch347_init_pack_size();
	if (retval != ERROR_OK)
		return retval;
	const int *speeds = ch347.pack_size == STANDARD_PACK ?
		ch347_standard_pack_clock_speeds : ch347_larger_pack_clock_speeds;
	*khz = speeds[speed_idx];
	return ERROR_OK;
}

/**
 * @brief converts the requested speed to a CH347 speed index
 *
 * If possible, selects the highest supported speed not exceeding the requested one.
 *
 * @param khz Speed in kHz
 * @param speed_idx Output of the CH347 speed index (JTAG) or clock divisor (SWD)
 * @return ERROR_OK at success; ERROR_FAIL if khz is zero
 */
static int ch347_speed_get_index(int khz, int *speed_idx)
{
	if (khz == 0) {
		LOG_ERROR("Adaptive clocking not supported");
		return ERROR_FAIL;
	}

	if (swd_mode) {
		if (khz >= CH347_SWD_CLOCK_MAX && ch347.swclk_5mhz_supported) {
			*speed_idx = 0;
		} else {
			// Don't allow too low clk speeds: packet processing is limited to ~8 msec
			// or triggers host USB disconnect
			*speed_idx = MIN(DIV_ROUND_UP(CH347_SWD_CLOCK_BASE, khz),
							 (int)CH347_SWD_CLOCK_MAX_DIVISOR);
		}
		return ERROR_OK;
	}

	// the available clock speeds depend on STANDARD_PACK or LARGER_PACK mode support
	int retval = ch347_init_pack_size();
	if (retval != ERROR_OK)
		return retval;
	// depending on pack size there are different fixed clock speeds possible
	const int *speeds = ch347.pack_size == STANDARD_PACK ?
		ch347_standard_pack_clock_speeds : ch347_larger_pack_clock_speeds;
	int length = ch347.pack_size == STANDARD_PACK ?
		ARRAY_SIZE(ch347_standard_pack_clock_speeds) : ARRAY_SIZE(ch347_larger_pack_clock_speeds);
	// find the suitable speed index
	int idx = length - 1;
	while (idx > 0 && speeds[idx] > khz)
		idx--;

	if (speeds[idx] > khz) {
		LOG_WARNING("Requested speed of %d kHz is not possible. Using %d kHz",
			khz, speeds[idx]);
	}
	*speed_idx = idx;
	return ERROR_OK;
}

static const struct command_registration ch347_command_handlers[] = {
	{
		.name = "ch347",
		.mode = COMMAND_ANY,
		.help = "perform ch347 management",
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

/**
 * @brief CH347 Initialization function
 *
 * @return ERROR_OK on success
 */
static int ch347_init(void)
{
	int retval = ch347_open_device();

	if (retval != ERROR_OK) {
		LOG_ERROR("CH347 open error");
		return retval;
	}

	LOG_DEBUG("CH347 open success");

	if (swd_mode) {
		/* Keep idle SWCLK and SWDIO state if CH347_CMD_JTAG_BIT_OP
		 * is used for SRST handling */
		ch347.tck_pin = TCK_H;
		ch347.tms_pin = TMS_L;
	} else {
		ch347.tck_pin = TCK_L;
		ch347.tms_pin = TMS_H;
	}
	ch347.tdi_pin = TDI_L;
	ch347.trst_pin = TRST_H;
	ch347.srst_pin = SRST_H;
	ch347_dedicated_srst_enabled = false;
	INIT_LIST_HEAD(&ch347.cmd_queue);
	INIT_LIST_HEAD(&ch347.scan_queue);

	unsigned int gpios_used = 0;
	const struct ch347_bit_op *gpio_to_bit_op = ch347.chip_variant == CH347F
				? ch347f_gpio_to_bit_op : ch347t_gpio_to_bit_op;

	for (unsigned int i = 0; i < GPIO_CNT_EXT; i++) {
		if (swd_mode ? gpio_to_bit_op[i].swd_use : gpio_to_bit_op[i].jtag_use)
			gpios_used |= BIT(i);
	}

	retval = ch347_check_gpio_unique(ADAPTER_GPIO_IDX_SRST, &gpios_used);
	if (retval == ERROR_OK)
		retval = ch347_check_gpio_unique(ADAPTER_GPIO_IDX_TRST, &gpios_used);
	if (retval == ERROR_OK)
		retval = ch347_check_gpio_unique(ADAPTER_GPIO_IDX_LED, &gpios_used);
	if (retval != ERROR_OK)
		return retval;

	ch347.trst_default_gpio = ch347_trst_gpio();
	if (gpios_used & BIT(ch347.trst_default_gpio))
		ch347.trst_default_gpio = ADAPTER_GPIO_NOT_SET;

	ch347.srst_default_gpio = ADAPTER_GPIO_NOT_SET;
	if (ch347.chip_variant == CH347F
			&& (gpios_used & BIT(CH347F_SRST_FAKE_GPIO)) == 0)
		ch347.srst_default_gpio = CH347F_SRST_FAKE_GPIO;

	/* Static initial pack_size is STANDARD_PACK (0), which would look already
	 * probed. max_len stays 0 until the probe, and PIN_SET uses this queue. */
	ch347.pack_size = UNSET;
	retval = ch347_init_pack_size();
	if (retval != ERROR_OK)
		return retval;

	uint8_t gpios[GPIO_CNT] = { 0 };
	uint8_t bit_op = 0;
	ch347_init_gpio(ADAPTER_GPIO_IDX_SRST, gpios, &bit_op);
	ch347_init_gpio(ADAPTER_GPIO_IDX_TRST, gpios, &bit_op);
	ch347_init_gpio(ADAPTER_GPIO_IDX_LED, gpios, &bit_op);

	if (bit_op & SRST_H) {
		retval = ch347_srst_enable();
		if (retval != ERROR_OK) {
			LOG_WARNING("CH347F dedicated SRST enable failed");
			// Do not fail adapter init, partial functionality possible
		}
	}

	retval = ch347_gpios_update(gpios, bit_op, true);
	if (retval != ERROR_OK) {
		LOG_WARNING("CH347 initial GPIO setting failed");
		// Do not fail adapter init, partial functionality possible
	}

	if (swd_mode)
		return ch347_swd_init_cmd(1);

	tap_set_state(TAP_RESET);
	return ERROR_OK;
}

/**
 * @brief Initialization for the swd mode
 *
 * @return Always ERROR_OK
 */
static int ch347_swd_init(void)
{
	LOG_INFO("CH347 SWD mode enabled");
	swd_mode = true;
	memset(&ch347_swd_context, 0, sizeof(ch347_swd_context));

	INIT_LIST_HEAD(&ch347_swd_context.send_cmd_head);
	INIT_LIST_HEAD(&ch347_swd_context.free_cmd_head);

	ch347_swd_context.send_len = 0;
	ch347_swd_context.need_recv_len = 0;
	struct ch347_swd_io *pswd_io = ch347_swd_context.ch347_cmd_buf;
	for (int i = 0; i < CH347_MAX_CMD_BUF; i++, pswd_io++) {
		INIT_LIST_HEAD(&pswd_io->list_entry);
		list_add_tail(&pswd_io->list_entry, &ch347_swd_context.free_cmd_head);
	}
	return ERROR_OK;
}

static struct ch347_swd_io *ch347_get_one_swd_io(void)
{
	struct ch347_swd_io *pswd_io;
	if (list_empty(&ch347_swd_context.free_cmd_head))
		return NULL;

	pswd_io = list_first_entry(&ch347_swd_context.free_cmd_head,
				   struct ch347_swd_io, list_entry);
	list_del_init(&pswd_io->list_entry);
	pswd_io->cmd = 0;
	pswd_io->usb_cmd = CH347_CMD_SWD_SEQ_W;
	pswd_io->dst = NULL;
	return pswd_io;
}

static int ch347_swd_queue_flush(void)
{
	ch347_swd_context.recv_len = 0;

	int retval = ch347_cmd_start_next(CH347_CMD_SWD);
	if (retval == ERROR_OK)
		retval = ch347_scratchpad_add_bytes(ch347_swd_context.send_buf, ch347_swd_context.send_len);
	if (retval != ERROR_OK) {
		ch347_queues_discard();
		return retval;
	}

	retval = ch347_cmd_transmit_queue();
	if (retval != ERROR_OK)
		return retval;

	if (ch347_swd_context.recv_len < ch347_swd_context.need_recv_len) {
		LOG_ERROR("write/read failed %d %d",
			  ch347_swd_context.recv_len,
			  ch347_swd_context.need_recv_len);
		retval = ERROR_FAIL;
	}

	return retval;
}

static void ch347_write_swd_reg(uint8_t cmd, const uint32_t out)
{
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = CH347_CMD_SWD_REG_W;
	// 8bit + 32bit +1bit
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = 0x29;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = 0x00;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = cmd;
	h_u32_to_le(&ch347_swd_context.send_buf[ch347_swd_context.send_len], out);
	ch347_swd_context.send_len += 4;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = parity_u32(out);
	// 0xA0 +  1 byte(3bit ACK)
	ch347_swd_context.need_recv_len += (1 + 1);
	ch347_swd_context.total_swd_clk += 46;
}

static void ch347_write_spec_seq(const uint8_t *out, uint8_t out_len)
{
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] =
		CH347_CMD_SWD_SEQ_W;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = out_len;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = 0x00;
	for (uint8_t i = 0; i < DIV_ROUND_UP(out_len, 8); i++)
		ch347_swd_context.send_buf[ch347_swd_context.send_len++] = out ? out[i] : 0x00;
	ch347_swd_context.need_recv_len += 1; // 0xA1
	ch347_swd_context.total_swd_clk += out_len;
}

static void ch347_read_swd_reg(uint8_t cmd)
{
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = CH347_CMD_SWD_REG_R;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = 0x22;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = 0x00;
	ch347_swd_context.send_buf[ch347_swd_context.send_len++] = cmd;
	// 0xA2 + 1 byte(3bit ACK) + 4 byte(data) + 1 byte(1bit parity+1bit trn)
	ch347_swd_context.need_recv_len += 1 + 1 + 4 + 1;
	ch347_swd_context.total_swd_clk += 46;
}

static int ch347_swd_switch_out(enum swd_special_seq seq, const uint8_t *out, unsigned int out_len)
{
	if ((ch347_swd_context.send_len + (1 + 2 + DIV_ROUND_UP(out_len, 8))) > CH347_MAX_SEND_BUF)
		return ERROR_FAIL;
	if ((ch347_swd_context.need_recv_len + 2) > CH347_MAX_RECV_BUF)
		return ERROR_FAIL;

	struct ch347_swd_io *pswd_io = ch347_get_one_swd_io();
	if (pswd_io) {
		ch347_write_spec_seq(out, out_len);
		list_add_tail(&pswd_io->list_entry, &ch347_swd_context.send_cmd_head);
		return ERROR_OK;
	} else {
		return ERROR_FAIL;
	}
}

// check read/write REG can fill in remaining buff
static bool ch347_chk_buf_size(uint8_t cmd, uint32_t ap_delay_clk)
{
	bool flush = false;
	int send_len = ch347_swd_context.send_len;
	int recv_len = ch347_swd_context.need_recv_len;
	int len;
	do {
		if (cmd & SWD_CMD_RNW) {
			len = 1 + 1 + 1 + 1; // 0xA2 + len + rev + cmd
			if (send_len + len > CH347_MAX_SEND_BUF)
				break;
			send_len += len;
			len = 1 + 1 + 4 + 1;
			/* 0xA2 + 1byte(3bit ack) +  4byte(data) +
			   1byte(1bit parity+1bit trn) */
			if (recv_len + len > CH347_MAX_RECV_BUF)
				break;
			recv_len += len;
		} else {                         // write reg
			len = 1 + 1 + 1 + 1 + 4 + 1;
			// 0xA0 + len + rev  + cmd +data + parity
			if (send_len + len > CH347_MAX_SEND_BUF)
				break;
			send_len += len;
			len = 1 + 1; // 0xA0 + 1byte(3bit ack)
			if (recv_len + len > CH347_MAX_RECV_BUF)
				break;
			recv_len += len;
		}
		if (cmd & SWD_CMD_APNDP) {
			len = 1 + 1 + 1 + DIV_ROUND_UP(ap_delay_clk, 8);
			// 0xA1 + Len  + rev  + n byte(delay)
			if (send_len + len > CH347_MAX_SEND_BUF)
				break;
			len = 1; // 0xA1
			if ((recv_len + len) > CH347_MAX_RECV_BUF)
				break;
		}
		// swd packet requests
		flush = true;
	} while (false);

	return flush;
}

static int ch347_swd_run_queue_inner(void);

static int ch347_swd_send_idle(uint32_t ap_delay_clk)
{
	bool run_q = false;
	struct ch347_swd_io *pswd_io = NULL;
	unsigned int max_processing_clk = ch347_swd_context.clk_divisor
		? CH347_MAX_PROCESSING_US / ch347_swd_context.clk_divisor
		: CH347_MAX_PROCESSING_US * 5;
	bool more_q_runs =
		1 + 1 + 1 + DIV_ROUND_UP(ap_delay_clk, 8) > CH347_MAX_SEND_BUF
		&& ap_delay_clk > max_processing_clk;

	if (ch347_swd_context.sent_cmd_count) {
		unsigned int expected_total_clk = ch347_swd_context.total_swd_clk + ap_delay_clk;
		unsigned int expected_send_len = ch347_swd_context.send_len
							+ 1 + 1 + 1 + DIV_ROUND_UP(ap_delay_clk, 8);
					// 0xA1 + Len  + rev  + n byte(delay)
		unsigned int expected_recv_len = ch347_swd_context.need_recv_len + 1;
					// 0xA1
		unsigned int expected_time = ch347_swd_context.clk_divisor
				? expected_total_clk * ch347_swd_context.clk_divisor
				: expected_total_clk / 5;
		if (expected_time > CH347_MAX_PROCESSING_US
				|| expected_send_len > CH347_MAX_SEND_BUF
				|| expected_recv_len > CH347_MAX_RECV_BUF) {
			int send_room = CH347_MAX_SEND_BUF - ch347_swd_context.send_len - 1 - 1 - 1;
			if (more_q_runs
					&& send_room > 0
					&& expected_recv_len <= CH347_MAX_RECV_BUF
					&& ch347_swd_context.total_swd_clk < max_processing_clk) {
				pswd_io = ch347_get_one_swd_io();
				if (pswd_io) {
					// fill the rest of queue/time by part of delay
					unsigned int this_delay_clk = MIN(ap_delay_clk, 255);
					if ((unsigned int)send_room * 8 < this_delay_clk)
						this_delay_clk = send_room * 8;
					if (max_processing_clk - ch347_swd_context.total_swd_clk < this_delay_clk)
						this_delay_clk = max_processing_clk - ch347_swd_context.total_swd_clk;
					LOG_DEBUG_IO("partial delay %u clk", this_delay_clk);
					ch347_write_spec_seq(NULL, this_delay_clk);
					list_add_tail(&pswd_io->list_entry, &ch347_swd_context.send_cmd_head);
					ap_delay_clk -= this_delay_clk;
				}
			}
			run_q = true;
		}
	}

	do {
		if (!run_q)
			pswd_io = ch347_get_one_swd_io();

		if (!pswd_io) {
			int retval = ch347_swd_run_queue_inner();
			if (retval != ERROR_OK)
				return retval;

			pswd_io = ch347_get_one_swd_io();
			if (!pswd_io) {
				LOG_ERROR("ch347 SWD queue not empty after ch347_swd_run_queue");
				return ERROR_FAIL;
			}
		}

		unsigned int send_room = CH347_MAX_SEND_BUF - 1 - 1 - 1;
		unsigned int this_delay_clk = MIN(ap_delay_clk, 255);
		if (send_room * 8 < this_delay_clk)
			this_delay_clk = send_room * 8;
		if (max_processing_clk < this_delay_clk)
			this_delay_clk = max_processing_clk;
		LOG_DEBUG_IO("delay %u clk", this_delay_clk);
		ch347_write_spec_seq(NULL, this_delay_clk);
		list_add_tail(&pswd_io->list_entry, &ch347_swd_context.send_cmd_head);
		ap_delay_clk -= this_delay_clk;
		run_q = true;
		pswd_io = NULL;
	} while (ap_delay_clk);
	return ERROR_OK;
}

static int ch347_swd_run_queue_inner(void)
{
	LOG_DEBUG_IO("Executing %u queued transactions", ch347_swd_context.sent_cmd_count);

	/* CMD_GPIO or CMD_JTAG_BIT_OP can take place before CMD_SWD
	 * in the same USB packet */
	if (!ch347.led_is_on)
		ch347_activity_led_set(true, false);

	int retval = ch347_swd_queue_flush();
	if (retval != ERROR_OK) {
		LOG_ERROR("CH347 usb write/read failed");
		goto skip;
	}

	uint8_t *recv_buf = ch347_swd_context.recv_buf;
	int recv_len = 0;
	struct list_head *tmp;
	struct list_head *pos;
	struct ch347_swd_io *pswd_io;

	list_for_each_safe(pos, tmp, &ch347_swd_context.send_cmd_head) {
		pswd_io = list_entry(pos, struct ch347_swd_io, list_entry);
		if (pswd_io->usb_cmd == CH347_CMD_SWD_SEQ_W) {
			if (recv_buf[recv_len++] != CH347_CMD_SWD_SEQ_W) {
				LOG_ERROR("CH347 usb write/read failed - not CH347_CMD_SWD_SEQ_W");
				retval = ERROR_FAIL;
				goto skip;
			}
		} else { // read/write Reg
			uint32_t ack;
			bool check_ack;
			// read  Reg
			if (recv_buf[recv_len] == CH347_CMD_SWD_REG_R) {
				recv_len++;
				ack = buf_get_u32(&recv_buf[recv_len++], 0, 3);
				/* Devices do not reply to DP_TARGETSEL write
				   cmd, ignore received ack */
				check_ack = swd_cmd_returns_ack(pswd_io->cmd);
				if (pswd_io->cmd & SWD_CMD_RNW) {
					uint32_t data = buf_get_u32(&recv_buf[recv_len], 0, 32);

					LOG_CUSTOM_LEVEL((check_ack && ack != SWD_ACK_OK)
										? LOG_LVL_DEBUG : LOG_LVL_DEBUG_IO,
									"%s%s %s read reg %X = %08" PRIx32,
									check_ack ? "" : "ack ignored ",
									ack == SWD_ACK_OK ? "OK" :
										ack == SWD_ACK_WAIT ? "WAIT" :
										ack == SWD_ACK_FAULT  ? "FAULT" : "JUNK",
									pswd_io->cmd & SWD_CMD_APNDP ? "AP" : "DP",
									(pswd_io->cmd & SWD_CMD_A32) >> 1,
									data);

					if (ack != SWD_ACK_OK && check_ack) {
						retval = swd_ack_to_error_code(ack);
						goto skip;
					}

					uint32_t parity = buf_get_u32(&recv_buf[recv_len], 32, 1);
					if (parity != (uint32_t)parity_u32(data)) {
						LOG_ERROR("SWD Read data parity mismatch");
						retval = ERROR_SWD_FAIL;
						goto skip;
					}

					if (pswd_io->dst)
						*pswd_io->dst = data;
				} else {
					retval = ERROR_FAIL;
					LOG_ERROR("CH347 usb write/read failed - not SWD_CMD_RNW");
					goto skip;
				}
				recv_len += 5;
			} else if (recv_buf[recv_len] == CH347_CMD_SWD_REG_W) {
				recv_len++;
				ack = buf_get_u32(&recv_buf[recv_len++], 0, 3);
				/* Devices do not reply to DP_TARGETSEL write
				   cmd, ignore received ack */
				check_ack = swd_cmd_returns_ack(pswd_io->cmd);

				LOG_CUSTOM_LEVEL((check_ack && ack != SWD_ACK_OK)
									? LOG_LVL_DEBUG : LOG_LVL_DEBUG_IO,
								 "%s%s %s write reg %X = %08" PRIx32,
								 check_ack ? "" : "ack ignored ",
								 ack == SWD_ACK_OK ? "OK" :
									ack == SWD_ACK_WAIT ? "WAIT" :
									ack == SWD_ACK_FAULT  ? "FAULT" : "JUNK",
								 pswd_io->cmd & SWD_CMD_APNDP ? "AP" : "DP",
								 (pswd_io->cmd & SWD_CMD_A32) >> 1,
								 pswd_io->value);

				if (ack != SWD_ACK_OK && check_ack) {
					retval = swd_ack_to_error_code(ack);
					goto skip;
				}
			} else {
				retval = ERROR_FAIL;
				LOG_ERROR("CH347 usb write/read failed recv_len = %d", recv_len);
				goto skip;
			}
		}
		list_del_init(&pswd_io->list_entry);
		list_add_tail(&pswd_io->list_entry,
				  &ch347_swd_context.free_cmd_head);
	}

skip:
	if (!list_empty(&ch347_swd_context.send_cmd_head)) {
		list_for_each_safe(pos, tmp, &ch347_swd_context.send_cmd_head) {
			pswd_io = list_entry(pos, struct ch347_swd_io, list_entry);
			list_del_init(&pswd_io->list_entry);
			list_add_tail(&pswd_io->list_entry,
					  &ch347_swd_context.free_cmd_head);
		}
	}

	ch347_swd_context.send_len = 0;
	ch347_swd_context.need_recv_len = 0;
	ch347_swd_context.recv_len = 0;
	ch347_swd_context.sent_cmd_count = 0;
	ch347_swd_context.total_swd_clk = 0;
	return retval;
}

static int ch347_swd_run_queue(void)
{
	/* A transaction must be followed by another transaction or at least 8
	   idle cycles to ensure that data is clocked through the AP. */
	int retval = ch347_swd_send_idle(8);
	if (retval != ERROR_OK)
		return retval;

	retval = ch347_swd_run_queue_inner();

	/* CMD_GPIO or CMD_JTAG_BIT_OP cannot follow CMD_SWD in the same
	 * USB packet. Flush the queue first then send eventual LED off
	 * in a separate USB packet */
	ch347_activity_led_set(false, true);

	return retval;
}

static int ch347_swd_queue_cmd(uint8_t cmd, uint32_t *dst, uint32_t data, uint32_t ap_delay_clk)
{
	int retval = ERROR_OK;
	bool run_q = false;
	if (ch347_swd_context.sent_cmd_count) {
		unsigned int expected_total_clk = ch347_swd_context.total_swd_clk
							+ 46 // SWD transaction
							+ ap_delay_clk
							+ 8; // 8 idle cycles at the end of queue
		unsigned int expected_time = ch347_swd_context.clk_divisor
				? expected_total_clk * ch347_swd_context.clk_divisor
				: expected_total_clk / 5;
		if (expected_time > CH347_MAX_PROCESSING_US) {
			LOG_DEBUG_IO("Expected queue run %u cycles, with this cmd %u",
						 ch347_swd_context.total_swd_clk, expected_total_clk);
			run_q = true;
		} else if (!ch347_chk_buf_size(cmd, ap_delay_clk)) {
			run_q = true;
		}
	}

	struct ch347_swd_io *pswd_io = NULL;
	if (!run_q)
		pswd_io = ch347_get_one_swd_io();

	if (!pswd_io) {
		retval = ch347_swd_run_queue_inner();
		if (retval != ERROR_OK)
			return retval;

		pswd_io = ch347_get_one_swd_io();
		if (!pswd_io) {
			LOG_ERROR("ch347 SWD queue not empty after ch347_swd_run_queue");
			return ERROR_FAIL;
		}
	}

	pswd_io->cmd = cmd | SWD_CMD_START | SWD_CMD_PARK;

	if (pswd_io->cmd & SWD_CMD_RNW) {
		pswd_io->usb_cmd = CH347_CMD_SWD_REG_R;
		pswd_io->dst = dst;
		ch347_read_swd_reg(pswd_io->cmd);
	} else {
		pswd_io->usb_cmd = CH347_CMD_SWD_REG_W;
		pswd_io->value = data;
		ch347_write_swd_reg(pswd_io->cmd, data);
	}

	ch347_swd_context.sent_cmd_count++;
	list_add_tail(&pswd_io->list_entry, &ch347_swd_context.send_cmd_head);

	// Insert idle cycles after AP accesses to avoid WAIT
	if (ap_delay_clk)
		retval = ch347_swd_send_idle(ap_delay_clk);

	return retval;
}

static int ch347_swd_switch_seq(enum swd_special_seq seq)
{
	switch (seq) {
	case LINE_RESET:
		LOG_DEBUG("SWD line reset");
		return ch347_swd_switch_out(seq, swd_seq_line_reset, swd_seq_line_reset_len);
	case JTAG_TO_SWD:
		LOG_DEBUG("JTAG-to-SWD");
		return ch347_swd_switch_out(seq, swd_seq_jtag_to_swd, swd_seq_jtag_to_swd_len);
	case JTAG_TO_DORMANT:
		LOG_DEBUG("JTAG-to-DORMANT");
		return ch347_swd_switch_out(seq, swd_seq_jtag_to_dormant, swd_seq_jtag_to_dormant_len);
	case SWD_TO_JTAG:
		LOG_DEBUG("SWD-to-JTAG");
		return ch347_swd_switch_out(seq, swd_seq_swd_to_jtag, swd_seq_swd_to_jtag_len);
	case SWD_TO_DORMANT:
		LOG_DEBUG("SWD-to-DORMANT");
		return ch347_swd_switch_out(seq, swd_seq_swd_to_dormant, swd_seq_swd_to_dormant_len);
	case DORMANT_TO_SWD:
		LOG_DEBUG("DORMANT-to-SWD");
		return ch347_swd_switch_out(seq, swd_seq_dormant_to_swd, swd_seq_dormant_to_swd_len);
	case DORMANT_TO_JTAG:
		LOG_DEBUG("DORMANT-to-JTAG");
		return ch347_swd_switch_out(seq, swd_seq_dormant_to_jtag, swd_seq_dormant_to_jtag_len);
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}
}

static int ch347_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
	assert(cmd & SWD_CMD_RNW);
	return ch347_swd_queue_cmd(cmd, value, 0, ap_delay_clk);
}

static int ch347_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
	assert(!(cmd & SWD_CMD_RNW));
	return ch347_swd_queue_cmd(cmd, NULL, value, ap_delay_clk);
}

static const struct swd_driver ch347_swd = {
	.init = ch347_swd_init,
	.switch_seq = ch347_swd_switch_seq,
	.read_reg = ch347_swd_read_reg,
	.write_reg = ch347_swd_write_reg,
	.run = ch347_swd_run_queue,
};

static struct jtag_interface ch347_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = ch347_execute_queue,
};

struct adapter_driver ch347_adapter_driver = {
	.name = "ch347",
	.transport_ids = TRANSPORT_JTAG | TRANSPORT_SWD,
	.transport_preferred_id = TRANSPORT_JTAG,
	.commands = ch347_command_handlers,

	.init = ch347_init,
	.quit = ch347_quit,
	.reset = ch347_reset,
	.speed = ch347_speed_set,
	.khz = ch347_speed_get_index,
	.speed_div = ch347_speed_get,

	.jtag_ops = &ch347_interface,
	.swd_ops = &ch347_swd,
};
