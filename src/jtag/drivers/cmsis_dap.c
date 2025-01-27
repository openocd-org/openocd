// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2021 by Adrian Negreanu                                 *
 *   groleo@gmail.com                                                      *
 *                                                                         *
 *   Copyright (C) 2018 by Mickaël Thomas                                  *
 *   mickael9@gmail.com                                                    *
 *                                                                         *
 *   Copyright (C) 2016 by Maksym Hilliaka                                 *
 *   oter@frozen-team.com                                                  *
 *                                                                         *
 *   Copyright (C) 2016 by Phillip Pearson                                 *
 *   pp@myelin.co.nz                                                       *
 *                                                                         *
 *   Copyright (C) 2014 by Paul Fertser                                    *
 *   fercerpav@gmail.com                                                   *
 *                                                                         *
 *   Copyright (C) 2013 by mike brown                                      *
 *   mike@theshedworks.org.uk                                              *
 *                                                                         *
 *   Copyright (C) 2013 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <transport/transport.h>
#include "helper/replacements.h"
#include <jtag/adapter.h>
#include <jtag/swd.h>
#include <jtag/interface.h>
#include <jtag/commands.h>
#include <jtag/tcl.h>
#include <target/cortex_m.h>

#include "cmsis_dap.h"
#include "libusb_helper.h"

static const struct cmsis_dap_backend *const cmsis_dap_backends[] = {
#if BUILD_CMSIS_DAP_USB == 1
	&cmsis_dap_usb_backend,
#endif

#if BUILD_CMSIS_DAP_HID == 1
	&cmsis_dap_hid_backend,
#endif
};

/* USB Config */

/* Known vid/pid pairs:
 * VID 0xc251: Keil Software
 * PID 0xf001: LPC-Link-II CMSIS_DAP
 * PID 0xf002: OPEN-SDA CMSIS_DAP (Freedom Board)
 * PID 0x2722: Keil ULINK2 CMSIS-DAP
 * PID 0x2750: Keil ULINKplus CMSIS-DAP
 *
 * VID 0x0d28: mbed Software
 * PID 0x0204: MBED CMSIS-DAP
 */

#define MAX_USB_IDS 8
/* vid = pid = 0 marks the end of the list */
static uint16_t cmsis_dap_vid[MAX_USB_IDS + 1] = { 0 };
static uint16_t cmsis_dap_pid[MAX_USB_IDS + 1] = { 0 };
static int cmsis_dap_backend = -1;
static bool swd_mode;

/* CMSIS-DAP General Commands */
#define CMD_DAP_INFO              0x00
#define CMD_DAP_LED               0x01
#define CMD_DAP_CONNECT           0x02
#define CMD_DAP_DISCONNECT        0x03
#define CMD_DAP_WRITE_ABORT       0x08
#define CMD_DAP_DELAY             0x09
#define CMD_DAP_RESET_TARGET      0x0A

/* CMD_INFO */
#define INFO_ID_VENDOR            0x01      /* string */
#define INFO_ID_PRODUCT           0x02      /* string */
#define INFO_ID_SERNUM            0x03      /* string */
#define INFO_ID_FW_VER            0x04      /* string */
#define INFO_ID_TD_VEND           0x05      /* string */
#define INFO_ID_TD_NAME           0x06      /* string */
#define INFO_ID_CAPS              0xf0      /* byte */
#define INFO_ID_PKT_CNT           0xfe      /* byte */
#define INFO_ID_PKT_SZ            0xff      /* short */
#define INFO_ID_SWO_BUF_SZ        0xfd      /* word */

#define INFO_CAPS_SWD                 BIT(0)
#define INFO_CAPS_JTAG                BIT(1)
#define INFO_CAPS_SWO_UART            BIT(2)
#define INFO_CAPS_SWO_MANCHESTER      BIT(3)
#define INFO_CAPS_ATOMIC_CMDS         BIT(4)
#define INFO_CAPS_TEST_DOMAIN_TIMER   BIT(5)
#define INFO_CAPS_SWO_STREAMING_TRACE BIT(6)
#define INFO_CAPS_UART_PORT           BIT(7)
#define INFO_CAPS_USB_COM_PORT        BIT(8)
#define INFO_CAPS__NUM_CAPS               9

/* CMD_LED */
#define LED_ID_CONNECT            0x00
#define LED_ID_RUN                0x01

#define LED_OFF                   0x00
#define LED_ON                    0x01

/* CMD_CONNECT */
#define CONNECT_DEFAULT           0x00
#define CONNECT_SWD               0x01
#define CONNECT_JTAG              0x02

/* CMSIS-DAP Common SWD/JTAG Commands */
#define CMD_DAP_DELAY             0x09
#define CMD_DAP_SWJ_PINS          0x10
#define CMD_DAP_SWJ_CLOCK         0x11
#define CMD_DAP_SWJ_SEQ           0x12

/*
 * PINS
 * Bit 0: SWCLK/TCK
 * Bit 1: SWDIO/TMS
 * Bit 2: TDI
 * Bit 3: TDO
 * Bit 5: nTRST
 * Bit 7: nRESET
 */

#define SWJ_PIN_TCK               (1<<0)
#define SWJ_PIN_TMS               (1<<1)
#define SWJ_PIN_TDI               (1<<2)
#define SWJ_PIN_TDO               (1<<3)
#define SWJ_PIN_TRST              (1<<5)
#define SWJ_PIN_SRST              (1<<7)

/* CMSIS-DAP SWD Commands */
#define CMD_DAP_SWD_CONFIGURE     0x13
#define CMD_DAP_SWD_SEQUENCE      0x1D

/* CMSIS-DAP JTAG Commands */
#define CMD_DAP_JTAG_SEQ          0x14
#define CMD_DAP_JTAG_CONFIGURE    0x15
#define CMD_DAP_JTAG_IDCODE       0x16

/* CMSIS-DAP JTAG sequence info masks */
/* Number of bits to clock through (0 means 64) */
#define DAP_JTAG_SEQ_TCK          0x3F
/* TMS will be set during the sequence if this bit is set */
#define DAP_JTAG_SEQ_TMS          0x40
/* TDO output will be captured if this bit is set */
#define DAP_JTAG_SEQ_TDO          0x80


/* CMSIS-DAP Transfer Commands */
#define CMD_DAP_TFER_CONFIGURE    0x04
#define CMD_DAP_TFER              0x05
#define CMD_DAP_TFER_BLOCK        0x06
#define CMD_DAP_TFER_ABORT        0x07

/* DAP_TransferBlock increases the sum of command/response sizes
 * (due to 16-bit Transfer Count) if used in a small packet.
 * Prevent using it until we have at least r/w operations. */
#define CMD_DAP_TFER_BLOCK_MIN_OPS 4

/* DAP Status Code */
#define DAP_OK                    0
#define DAP_ERROR                 0xFF

/* CMSIS-DAP SWO Commands */
#define CMD_DAP_SWO_TRANSPORT     0x17
#define CMD_DAP_SWO_MODE          0x18
#define CMD_DAP_SWO_BAUDRATE      0x19
#define CMD_DAP_SWO_CONTROL       0x1A
#define CMD_DAP_SWO_STATUS        0x1B
#define CMD_DAP_SWO_DATA          0x1C
#define CMD_DAP_SWO_EX_STATUS     0x1E

/* SWO transport mode for reading trace data */
#define DAP_SWO_TRANSPORT_NONE    0
#define DAP_SWO_TRANSPORT_DATA    1
#define DAP_SWO_TRANSPORT_WINUSB  2

/* SWO trace capture mode */
#define DAP_SWO_MODE_OFF          0
#define DAP_SWO_MODE_UART         1
#define DAP_SWO_MODE_MANCHESTER   2

/* SWO trace data capture */
#define DAP_SWO_CONTROL_STOP      0
#define DAP_SWO_CONTROL_START     1

/* SWO trace status */
#define DAP_SWO_STATUS_CAPTURE_INACTIVE      0
#define DAP_SWO_STATUS_CAPTURE_ACTIVE        1
#define DAP_SWO_STATUS_CAPTURE_MASK          BIT(0)
#define DAP_SWO_STATUS_STREAM_ERROR_MASK     BIT(6)
#define DAP_SWO_STATUS_BUFFER_OVERRUN_MASK   BIT(7)

/* CMSIS-DAP Vendor Commands
 * None as yet... */

static const char * const info_caps_str[INFO_CAPS__NUM_CAPS] = {
	"SWD supported",
	"JTAG supported",
	"SWO-UART supported",
	"SWO-MANCHESTER supported",
	"Atomic commands supported",
	"Test domain timer supported",
	"SWO streaming trace supported",
	"UART communication port supported",
	"UART via USB COM port supported",
};

struct pending_scan_result {
	/** Offset in bytes in the CMD_DAP_JTAG_SEQ response buffer. */
	unsigned int first;
	/** Number of bits to read. */
	unsigned int length;
	/** Location to store the result */
	uint8_t *buffer;
	/** Offset in the destination buffer */
	unsigned int buffer_offset;
};

/* Each block in FIFO can contain up to pending_queue_len transfers */
static unsigned int pending_queue_len;
static unsigned int tfer_max_command_size;
static unsigned int tfer_max_response_size;

/* pointers to buffers that will receive jtag scan results on the next flush */
#define MAX_PENDING_SCAN_RESULTS 256
static int pending_scan_result_count;
static struct pending_scan_result pending_scan_results[MAX_PENDING_SCAN_RESULTS];

/* queued JTAG sequences that will be executed on the next flush */
#define QUEUED_SEQ_BUF_LEN (cmsis_dap_handle->packet_usable_size - 3)
static int queued_seq_count;
static int queued_seq_buf_end;
static int queued_seq_tdo_ptr;
static uint8_t queued_seq_buf[1024]; /* TODO: make dynamic / move into cmsis object */

static int queued_retval;

static uint8_t output_pins = SWJ_PIN_SRST | SWJ_PIN_TRST;

static struct cmsis_dap *cmsis_dap_handle;


static int cmsis_dap_quit(void);

static int cmsis_dap_open(void)
{
	const struct cmsis_dap_backend *backend = NULL;

	struct cmsis_dap *dap = calloc(1, sizeof(struct cmsis_dap));
	if (!dap) {
		LOG_ERROR("unable to allocate memory");
		return ERROR_FAIL;
	}

	if (cmsis_dap_backend >= 0) {
		/* Use forced backend */
		backend = cmsis_dap_backends[cmsis_dap_backend];
		if (backend->open(dap, cmsis_dap_vid, cmsis_dap_pid, adapter_get_required_serial()) != ERROR_OK)
			backend = NULL;
	} else {
		/* Try all backends */
		for (unsigned int i = 0; i < ARRAY_SIZE(cmsis_dap_backends); i++) {
			backend = cmsis_dap_backends[i];
			if (backend->open(dap, cmsis_dap_vid, cmsis_dap_pid, adapter_get_required_serial()) == ERROR_OK)
				break;
			else
				backend = NULL;
		}
	}

	if (!backend) {
		LOG_ERROR("unable to find a matching CMSIS-DAP device");
		free(dap);
		return ERROR_FAIL;
	}

	dap->backend = backend;

	cmsis_dap_handle = dap;

	return ERROR_OK;
}

static void cmsis_dap_close(struct cmsis_dap *dap)
{
	if (dap->backend) {
		dap->backend->close(dap);
		dap->backend = NULL;
	}

	free(dap->packet_buffer);

	for (unsigned int i = 0; i < MAX_PENDING_REQUESTS; i++) {
		free(dap->pending_fifo[i].transfers);
		dap->pending_fifo[i].transfers = NULL;
	}

	free(cmsis_dap_handle);
	cmsis_dap_handle = NULL;
}

static void cmsis_dap_flush_read(struct cmsis_dap *dap)
{
	unsigned int i;
	/* Some CMSIS-DAP adapters keep buffered packets over
	 * USB close/open so we need to flush up to 64 old packets
	 * to be sure all buffers are empty */
	for (i = 0; i < 64; i++) {
		int retval = dap->backend->read(dap, 10, CMSIS_DAP_BLOCKING);
		if (retval == ERROR_TIMEOUT_REACHED)
			break;
	}
	if (i)
		LOG_DEBUG("Flushed %u packets", i);
}

/* Send a message and receive the reply */
static int cmsis_dap_xfer(struct cmsis_dap *dap, int txlen)
{
	if (dap->write_count + dap->read_count) {
		LOG_ERROR("internal: queue not empty before xfer");
	}
	if (dap->pending_fifo_block_count) {
		LOG_ERROR("pending %u blocks, flushing", dap->pending_fifo_block_count);
		while (dap->pending_fifo_block_count) {
			dap->backend->read(dap, 10, CMSIS_DAP_BLOCKING);
			dap->pending_fifo_block_count--;
		}
		dap->pending_fifo_put_idx = 0;
		dap->pending_fifo_get_idx = 0;
	}

	uint8_t current_cmd = dap->command[0];
	int retval = dap->backend->write(dap, txlen, LIBUSB_TIMEOUT_MS);
	if (retval < 0)
		return retval;

	/* get reply */
	retval = dap->backend->read(dap, LIBUSB_TIMEOUT_MS, CMSIS_DAP_BLOCKING);
	if (retval < 0)
		return retval;

	uint8_t *resp = dap->response;
	if (resp[0] == DAP_ERROR) {
		LOG_ERROR("CMSIS-DAP command 0x%" PRIx8 " not implemented", current_cmd);
		return ERROR_NOT_IMPLEMENTED;
	}

	if (resp[0] != current_cmd) {
		LOG_ERROR("CMSIS-DAP command mismatch. Sent 0x%" PRIx8
			 " received 0x%" PRIx8, current_cmd, resp[0]);

		dap->backend->cancel_all(dap);
		cmsis_dap_flush_read(dap);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int cmsis_dap_cmd_dap_swj_pins(uint8_t pins, uint8_t mask, uint32_t delay, uint8_t *input)
{
	uint8_t *command = cmsis_dap_handle->command;

	command[0] = CMD_DAP_SWJ_PINS;
	command[1] = pins;
	command[2] = mask;
	h_u32_to_le(&command[3], delay);

	int retval = cmsis_dap_xfer(cmsis_dap_handle, 7);
	if (retval != ERROR_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_DAP_SWJ_PINS failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	if (input)
		*input = cmsis_dap_handle->response[1];

	return ERROR_OK;
}

static int cmsis_dap_cmd_dap_swj_clock(uint32_t swj_clock)
{
	uint8_t *command = cmsis_dap_handle->command;

	/* set clock in Hz */
	swj_clock *= 1000;

	command[0] = CMD_DAP_SWJ_CLOCK;
	h_u32_to_le(&command[1], swj_clock);

	int retval = cmsis_dap_xfer(cmsis_dap_handle, 5);
	if (retval != ERROR_OK || cmsis_dap_handle->response[1] != DAP_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_DAP_SWJ_CLOCK failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

/* clock a sequence of bits out on TMS, to change JTAG states */
static int cmsis_dap_cmd_dap_swj_sequence(uint8_t s_len, const uint8_t *sequence)
{
	uint8_t *command = cmsis_dap_handle->command;

#ifdef CMSIS_DAP_JTAG_DEBUG
	LOG_DEBUG("cmsis-dap TMS sequence: len=%d", s_len);
	for (unsigned int i = 0; i < DIV_ROUND_UP(s_len, 8); ++i)
		printf("%02X ", sequence[i]);

	printf("\n");
#endif

	command[0] = CMD_DAP_SWJ_SEQ;
	command[1] = s_len;
	bit_copy(&command[2], 0, sequence, 0, s_len);

	int retval = cmsis_dap_xfer(cmsis_dap_handle, 2 + DIV_ROUND_UP(s_len, 8));
	if (retval != ERROR_OK || cmsis_dap_handle->response[1] != DAP_OK)
		return ERROR_FAIL;

	return ERROR_OK;
}

static int cmsis_dap_cmd_dap_info(uint8_t info, uint8_t **data)
{
	uint8_t *command = cmsis_dap_handle->command;

	command[0] = CMD_DAP_INFO;
	command[1] = info;

	int retval = cmsis_dap_xfer(cmsis_dap_handle, 2);
	if (retval != ERROR_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_INFO failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	*data = &cmsis_dap_handle->response[1];

	return ERROR_OK;
}

static int cmsis_dap_cmd_dap_led(uint8_t led, uint8_t state)
{
	uint8_t *command = cmsis_dap_handle->command;

	command[0] = CMD_DAP_LED;
	command[1] = led;
	command[2] = state;

	int retval = cmsis_dap_xfer(cmsis_dap_handle, 3);
	if (retval != ERROR_OK || cmsis_dap_handle->response[1] != DAP_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_LED failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int cmsis_dap_cmd_dap_connect(uint8_t mode)
{
	uint8_t *command = cmsis_dap_handle->command;

	command[0] = CMD_DAP_CONNECT;
	command[1] = mode;

	int retval = cmsis_dap_xfer(cmsis_dap_handle, 2);
	if (retval != ERROR_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_CONNECT failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	if (cmsis_dap_handle->response[1] != mode) {
		LOG_ERROR("CMSIS-DAP failed to connect in mode (%d)", mode);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int cmsis_dap_cmd_dap_disconnect(void)
{
	uint8_t *command = cmsis_dap_handle->command;

	command[0] = CMD_DAP_DISCONNECT;

	int retval = cmsis_dap_xfer(cmsis_dap_handle, 1);
	if (retval != ERROR_OK || cmsis_dap_handle->response[1] != DAP_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_DISCONNECT failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int cmsis_dap_cmd_dap_tfer_configure(uint8_t idle, uint16_t retry_count, uint16_t match_retry)
{
	uint8_t *command = cmsis_dap_handle->command;

	command[0] = CMD_DAP_TFER_CONFIGURE;
	command[1] = idle;
	h_u16_to_le(&command[2], retry_count);
	h_u16_to_le(&command[4], match_retry);

	int retval = cmsis_dap_xfer(cmsis_dap_handle, 6);
	if (retval != ERROR_OK || cmsis_dap_handle->response[1] != DAP_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_TFER_Configure failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

static int cmsis_dap_cmd_dap_swd_configure(uint8_t cfg)
{
	uint8_t *command = cmsis_dap_handle->command;

	command[0] = CMD_DAP_SWD_CONFIGURE;
	command[1] = cfg;

	int retval = cmsis_dap_xfer(cmsis_dap_handle, 2);
	if (retval != ERROR_OK || cmsis_dap_handle->response[1] != DAP_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_SWD_Configure failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

#if 0
static int cmsis_dap_cmd_dap_delay(uint16_t delay_us)
{
	uint8_t *command = cmsis_dap_handle->command;

	command[0] = CMD_DAP_DELAY;
	h_u16_to_le(&command[1], delay_us);

	int retval = cmsis_dap_xfer(cmsis_dap_handle, 3);
	if (retval != ERROR_OK || cmsis_dap_handle->response[1] != DAP_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_Delay failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}
#endif

static int cmsis_dap_metacmd_targetsel(uint32_t instance_id)
{
	uint8_t *command = cmsis_dap_handle->command;
	const uint32_t seq_rd = 0x80, seq_wr = 0x00;

	/* SWD multi-drop requires a transfer ala CMD_DAP_TFER,
	but with no expectation of an SWD ACK response.  In
	CMSIS-DAP v1.20 and v2.00, CMD_DAP_SWD_SEQUENCE was
	added to allow this special sequence to be generated.
	The purpose of this operation is to select the target
	corresponding to the instance_id that is written */

	LOG_DEBUG_IO("DP write reg TARGETSEL %" PRIx32, instance_id);

	size_t idx = 0;
	command[idx++] = CMD_DAP_SWD_SEQUENCE;
	command[idx++] = 3;	/* sequence count */

	/* sequence 0: packet request for TARGETSEL */
	command[idx++] = seq_wr | 8;
	command[idx++] = SWD_CMD_START | swd_cmd(false, false, DP_TARGETSEL) | SWD_CMD_STOP | SWD_CMD_PARK;

	/* sequence 1: read Trn ACK Trn, no expectation for target to ACK  */
	command[idx++] = seq_rd | 5;

	/* sequence 2: WDATA plus parity */
	command[idx++] = seq_wr | (32 + 1);
	h_u32_to_le(command + idx, instance_id);
	idx += 4;
	command[idx++] = parity_u32(instance_id);

	int retval = cmsis_dap_xfer(cmsis_dap_handle, idx);
	if (retval != ERROR_OK || cmsis_dap_handle->response[1] != DAP_OK) {
		LOG_ERROR("CMSIS-DAP command SWD_Sequence failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

/**
 * Sets the SWO transport mode.
 * @param[in] transport     The transport mode. Can be None, SWO_Data or
 *                          WinUSB (requires CMSIS-DAP v2).
 */
static int cmsis_dap_cmd_dap_swo_transport(uint8_t transport)
{
	uint8_t *command = cmsis_dap_handle->command;

	command[0] = CMD_DAP_SWO_TRANSPORT;
	command[1] = transport;

	int retval = cmsis_dap_xfer(cmsis_dap_handle, 2);
	if (retval != ERROR_OK || cmsis_dap_handle->response[1] != DAP_OK) {
		LOG_ERROR("CMSIS-DAP: command CMD_SWO_Transport(%d) failed.", transport);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

/**
 * Sets the SWO trace capture mode.
 * @param[in] mode          Trace capture mode. Can be UART or MANCHESTER.
 */
static int cmsis_dap_cmd_dap_swo_mode(uint8_t mode)
{
	uint8_t *command = cmsis_dap_handle->command;

	command[0] = CMD_DAP_SWO_MODE;
	command[1] = mode;

	int retval = cmsis_dap_xfer(cmsis_dap_handle, 2);
	if (retval != ERROR_OK || cmsis_dap_handle->response[1] != DAP_OK) {
		LOG_ERROR("CMSIS-DAP: command CMD_SWO_Mode(%d) failed.", mode);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

/**
 * Sets the baudrate for capturing SWO trace data.
 * Can be called iteratively to determine supported baudrates.
 * @param[in]  in_baudrate  Requested baudrate.
 * @param[out] dev_baudrate Actual baudrate or 0 (baudrate not configured).
 *                          When requested baudrate is not achievable the
 *                          closest configured baudrate can be returned or
 *                          0 which indicates that baudrate was not configured.
 */
static int cmsis_dap_cmd_dap_swo_baudrate(
					uint32_t in_baudrate,
					uint32_t *dev_baudrate)
{
	uint8_t *command = cmsis_dap_handle->command;

	command[0] = CMD_DAP_SWO_BAUDRATE;
	h_u32_to_le(&command[1], in_baudrate);

	int retval = cmsis_dap_xfer(cmsis_dap_handle, 5);
	uint32_t rvbr = le_to_h_u32(&cmsis_dap_handle->response[1]);
	if (retval != ERROR_OK || rvbr == 0) {
		LOG_ERROR("CMSIS-DAP: command CMD_SWO_Baudrate(%u) -> %u failed.", in_baudrate, rvbr);
		if (dev_baudrate)
			*dev_baudrate = 0;
		return ERROR_JTAG_DEVICE_ERROR;
	}

	if (dev_baudrate)
		*dev_baudrate = rvbr;

	return ERROR_OK;
}

/**
 * Controls the SWO trace data capture.
 * @param[in] control       Start or stop a trace. Starting capture automatically
 *                          flushes any existing trace data in buffers which has
 *                          not yet been read.
 */
static int cmsis_dap_cmd_dap_swo_control(uint8_t control)
{
	uint8_t *command = cmsis_dap_handle->command;

	command[0] = CMD_DAP_SWO_CONTROL;
	command[1] = control;

	int retval = cmsis_dap_xfer(cmsis_dap_handle, 2);
	if (retval != ERROR_OK || cmsis_dap_handle->response[1] != DAP_OK) {
		LOG_ERROR("CMSIS-DAP: command CMD_SWO_Control(%d) failed.", control);
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

/**
 * Reads the SWO trace status.
 * @param[out] trace_status The trace's status.
 *                          Bit0: Trace Capture (1 - active, 0 - inactive).
 *                          Bit6: Trace Stream Error.
 *                          Bit7: Trace Buffer Overrun.
 * @param[out] trace_count  Number of bytes in Trace Buffer (not yet read).
 */
static int cmsis_dap_cmd_dap_swo_status(
					uint8_t *trace_status,
					size_t *trace_count)
{
	uint8_t *command = cmsis_dap_handle->command;

	command[0] = CMD_DAP_SWO_STATUS;

	int retval = cmsis_dap_xfer(cmsis_dap_handle, 1);
	if (retval != ERROR_OK) {
		LOG_ERROR("CMSIS-DAP: command CMD_SWO_Status failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	if (trace_status)
		*trace_status = cmsis_dap_handle->response[1];
	if (trace_count)
		*trace_count = le_to_h_u32(&cmsis_dap_handle->response[2]);

	return ERROR_OK;
}

/**
 * Reads the captured SWO trace data from Trace Buffer.
 * @param[in]  max_trace_count Maximum number of Trace Data bytes to read.
 * @param[out] trace_status The trace's status.
 * @param[out] trace_count  Number of Trace Data bytes read.
 * @param[out] data         Trace Data bytes read.
 */
static int cmsis_dap_cmd_dap_swo_data(
					size_t max_trace_count,
					uint8_t *trace_status,
					size_t *trace_count,
					uint8_t *data)
{
	uint8_t *command = cmsis_dap_handle->command;

	command[0] = CMD_DAP_SWO_DATA;
	h_u16_to_le(&command[1], max_trace_count);

	int retval = cmsis_dap_xfer(cmsis_dap_handle, 3);
	if (retval != ERROR_OK) {
		LOG_ERROR("CMSIS-DAP: command CMD_SWO_Data failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	*trace_status = cmsis_dap_handle->response[1];
	*trace_count = le_to_h_u16(&cmsis_dap_handle->response[2]);

	if (*trace_count > 0)
		memcpy(data, &cmsis_dap_handle->response[4], *trace_count);

	return ERROR_OK;
}

static void cmsis_dap_swd_discard_all_pending(struct cmsis_dap *dap)
{
	for (unsigned int i = 0; i < MAX_PENDING_REQUESTS; i++)
		dap->pending_fifo[i].transfer_count = 0;

	dap->pending_fifo_put_idx = 0;
	dap->pending_fifo_get_idx = 0;
	dap->pending_fifo_block_count = 0;
}

static void cmsis_dap_swd_cancel_transfers(struct cmsis_dap *dap)
{
	dap->backend->cancel_all(dap);
	cmsis_dap_flush_read(dap);
	cmsis_dap_swd_discard_all_pending(dap);
}

static void cmsis_dap_swd_write_from_queue(struct cmsis_dap *dap)
{
	uint8_t *command = dap->command;
	struct pending_request_block *block = &dap->pending_fifo[dap->pending_fifo_put_idx];

	assert(dap->write_count + dap->read_count == block->transfer_count);

	/* Reset packet size check counters for the next packet */
	dap->write_count = 0;
	dap->read_count = 0;

	LOG_DEBUG_IO("Executing %d queued transactions from FIFO index %u%s",
				 block->transfer_count, dap->pending_fifo_put_idx,
				 cmsis_dap_handle->swd_cmds_differ ? "" : ", same swd ops");

	if (queued_retval != ERROR_OK) {
		LOG_DEBUG("Skipping due to previous errors: %d", queued_retval);
		goto skip;
	}

	if (block->transfer_count == 0) {
		LOG_ERROR("internal: write an empty queue?!");
		goto skip;
	}

	bool block_cmd = !cmsis_dap_handle->swd_cmds_differ
					 && block->transfer_count >= CMD_DAP_TFER_BLOCK_MIN_OPS;
	block->command = block_cmd ? CMD_DAP_TFER_BLOCK : CMD_DAP_TFER;

	command[0] = block->command;
	command[1] = 0x00;	/* DAP Index */

	unsigned int idx;
	if (block_cmd) {
		h_u16_to_le(&command[2], block->transfer_count);
		idx = 4;	/* The first transfer will store the common DAP register */
	} else {
		command[2] = block->transfer_count;
		idx = 3;
	}

	for (unsigned int i = 0; i < block->transfer_count; i++) {
		struct pending_transfer_result *transfer = &(block->transfers[i]);
		uint8_t cmd = transfer->cmd;
		uint32_t data = transfer->data;

		LOG_DEBUG_IO("%s %s reg %x %" PRIx32,
				cmd & SWD_CMD_APNDP ? "AP" : "DP",
				cmd & SWD_CMD_RNW ? "read" : "write",
			  (cmd & SWD_CMD_A32) >> 1, data);

		/* When proper WAIT handling is implemented in the
		 * common SWD framework, this kludge can be
		 * removed. However, this might lead to minor
		 * performance degradation as the adapter wouldn't be
		 * able to automatically retry anything (because ARM
		 * has forgotten to implement sticky error flags
		 * clearing). See also comments regarding
		 * cmsis_dap_cmd_dap_tfer_configure() and
		 * cmsis_dap_cmd_dap_swd_configure() in
		 * cmsis_dap_init().
		 */
		if (!(cmd & SWD_CMD_RNW) &&
		    !(cmd & SWD_CMD_APNDP) &&
		    (cmd & SWD_CMD_A32) >> 1 == DP_CTRL_STAT &&
		    (data & CORUNDETECT)) {
			LOG_DEBUG("refusing to enable sticky overrun detection");
			data &= ~CORUNDETECT;
		}

		if (!block_cmd || i == 0)
			command[idx++] = (cmd >> 1) & 0x0f;

		if (!(cmd & SWD_CMD_RNW)) {
			h_u32_to_le(&command[idx], data);
			idx += 4;
		}
	}

	int retval = dap->backend->write(dap, idx, LIBUSB_TIMEOUT_MS);
	if (retval < 0) {
		queued_retval = retval;
		goto skip;
	}

	unsigned int packet_count = dap->quirk_mode ? 1 : dap->packet_count;
	dap->pending_fifo_put_idx = (dap->pending_fifo_put_idx + 1) % packet_count;
	dap->pending_fifo_block_count++;
	if (dap->pending_fifo_block_count > packet_count)
		LOG_ERROR("internal: too much pending writes %u", dap->pending_fifo_block_count);

	return;

skip:
	block->transfer_count = 0;
}

static void cmsis_dap_swd_read_process(struct cmsis_dap *dap, enum cmsis_dap_blocking blocking)
{
	int retval;
	struct pending_request_block *block = &dap->pending_fifo[dap->pending_fifo_get_idx];

	if (dap->pending_fifo_block_count == 0) {
		LOG_ERROR("internal: no pending write when reading?!");
		return;
	}

	if (queued_retval != ERROR_OK) {
		/* keep reading blocks until the pipeline is empty */
		retval = dap->backend->read(dap, 10, CMSIS_DAP_BLOCKING);
		if (retval == ERROR_TIMEOUT_REACHED || retval == 0) {
			/* timeout means that we flushed the pipeline,
			 * we can safely discard remaining pending requests */
			cmsis_dap_swd_discard_all_pending(dap);
			return;
		}
		goto skip;
	}

	/* get reply */
	retval = dap->backend->read(dap, LIBUSB_TIMEOUT_MS, blocking);
	bool timeout = (retval == ERROR_TIMEOUT_REACHED || retval == 0);
	if (timeout && blocking == CMSIS_DAP_NON_BLOCKING)
		return;

	if (retval <= 0) {
		LOG_DEBUG("error reading adapter response");
		queued_retval = ERROR_FAIL;
		if (timeout) {
			/* timeout means that we flushed the pipeline,
			 * we can safely discard remaining pending requests */
			cmsis_dap_swd_discard_all_pending(dap);
			return;
		}
		goto skip;
	}

	uint8_t *resp = dap->response;
	if (resp[0] != block->command) {
		LOG_ERROR("CMSIS-DAP command mismatch. Expected 0x%x received 0x%" PRIx8,
			block->command, resp[0]);
		cmsis_dap_swd_cancel_transfers(dap);
		queued_retval = ERROR_FAIL;
		return;
	}

	unsigned int transfer_count;
	unsigned int idx;
	if (block->command == CMD_DAP_TFER_BLOCK) {
		transfer_count = le_to_h_u16(&resp[1]);
		idx = 3;
	} else {
		transfer_count = resp[1];
		idx = 2;
	}
	if (resp[idx] & 0x08) {
		LOG_DEBUG("CMSIS-DAP Protocol Error @ %d (wrong parity)", transfer_count);
		queued_retval = ERROR_FAIL;
		goto skip;
	}
	uint8_t ack = resp[idx++] & 0x07;
	if (ack != SWD_ACK_OK) {
		LOG_DEBUG("SWD ack not OK @ %d %s", transfer_count,
			  ack == SWD_ACK_WAIT ? "WAIT" : ack == SWD_ACK_FAULT ? "FAULT" : "JUNK");
		queued_retval = swd_ack_to_error_code(ack);
		/* TODO: use results of transfers completed before the error occurred? */
		goto skip;
	}

	if (block->transfer_count != transfer_count) {
		LOG_ERROR("CMSIS-DAP transfer count mismatch: expected %d, got %d",
			  block->transfer_count, transfer_count);
		cmsis_dap_swd_cancel_transfers(dap);
		queued_retval = ERROR_FAIL;
		return;
	}

	LOG_DEBUG_IO("Received results of %d queued transactions FIFO index %u, %s mode",
				 transfer_count, dap->pending_fifo_get_idx,
				 blocking ? "blocking" : "nonblocking");

	for (unsigned int i = 0; i < transfer_count; i++) {
		struct pending_transfer_result *transfer = &(block->transfers[i]);
		if (transfer->cmd & SWD_CMD_RNW) {
			static uint32_t last_read;
			uint32_t data = le_to_h_u32(&resp[idx]);
			uint32_t tmp = data;
			idx += 4;

			LOG_DEBUG_IO("Read result: %" PRIx32, data);

			/* Imitate posted AP reads */
			if ((transfer->cmd & SWD_CMD_APNDP) ||
			    ((transfer->cmd & SWD_CMD_A32) >> 1 == DP_RDBUFF)) {
				tmp = last_read;
				last_read = data;
			}

			if (transfer->buffer)
				*(uint32_t *)(transfer->buffer) = tmp;
		}
	}

skip:
	block->transfer_count = 0;
	if (!dap->quirk_mode && dap->packet_count > 1)
		dap->pending_fifo_get_idx = (dap->pending_fifo_get_idx + 1) % dap->packet_count;
	dap->pending_fifo_block_count--;
}

static int cmsis_dap_swd_run_queue(void)
{
	if (cmsis_dap_handle->write_count + cmsis_dap_handle->read_count) {
		if (cmsis_dap_handle->pending_fifo_block_count)
			cmsis_dap_swd_read_process(cmsis_dap_handle, CMSIS_DAP_NON_BLOCKING);

		cmsis_dap_swd_write_from_queue(cmsis_dap_handle);
	}

	while (cmsis_dap_handle->pending_fifo_block_count)
		cmsis_dap_swd_read_process(cmsis_dap_handle, CMSIS_DAP_BLOCKING);

	cmsis_dap_handle->pending_fifo_put_idx = 0;
	cmsis_dap_handle->pending_fifo_get_idx = 0;

	int retval = queued_retval;
	queued_retval = ERROR_OK;

	return retval;
}

static unsigned int cmsis_dap_tfer_cmd_size(unsigned int write_count,
							unsigned int read_count, bool block_tfer)
{
	unsigned int size;
	if (block_tfer) {
		size = 5;						/* DAP_TransferBlock header */
		size += write_count * 4;		/* data */
	} else {
		size = 3;						/* DAP_Transfer header */
		size += write_count * (1 + 4);	/* DAP register + data */
		size += read_count;				/* DAP register */
	}
	return size;
}

static unsigned int cmsis_dap_tfer_resp_size(unsigned int write_count,
							unsigned int read_count, bool block_tfer)
{
	unsigned int size;
	if (block_tfer)
		size = 4;						/* DAP_TransferBlock response header */
	else
		size = 3;						/* DAP_Transfer response header */

	size += read_count * 4;				/* data */
	return size;
}

static void cmsis_dap_swd_queue_cmd(uint8_t cmd, uint32_t *dst, uint32_t data)
{
	/* TARGETSEL register write cannot be queued */
	if (swd_cmd(false, false, DP_TARGETSEL) == cmd) {
		queued_retval = cmsis_dap_swd_run_queue();

		cmsis_dap_metacmd_targetsel(data);
		return;
	}

	/* Compute sizes of the DAP Transfer command and the expected response
	 * for all queued and this operation */
	unsigned int write_count = cmsis_dap_handle->write_count;
	unsigned int read_count = cmsis_dap_handle->read_count;
	bool block_cmd;
	if (write_count + read_count < CMD_DAP_TFER_BLOCK_MIN_OPS)
		block_cmd = false;
	else
		block_cmd = !cmsis_dap_handle->swd_cmds_differ
					&& cmd == cmsis_dap_handle->common_swd_cmd;

	if (cmd & SWD_CMD_RNW)
		read_count++;
	else
		write_count++;

	unsigned int cmd_size = cmsis_dap_tfer_cmd_size(write_count, read_count,
													block_cmd);
	unsigned int resp_size = cmsis_dap_tfer_resp_size(write_count, read_count,
													block_cmd);
	unsigned int max_transfer_count = block_cmd ? 65535 : 255;

	/* Does the DAP Transfer command and also its expected response fit into one packet? */
	if (cmd_size > tfer_max_command_size
			|| resp_size > tfer_max_response_size
			|| write_count + read_count > max_transfer_count) {
		if (cmsis_dap_handle->pending_fifo_block_count)
			cmsis_dap_swd_read_process(cmsis_dap_handle, CMSIS_DAP_NON_BLOCKING);

		/* Not enough room in the queue. Run the queue. */
		cmsis_dap_swd_write_from_queue(cmsis_dap_handle);

		unsigned int packet_count = cmsis_dap_handle->quirk_mode ? 1 : cmsis_dap_handle->packet_count;
		if (cmsis_dap_handle->pending_fifo_block_count >= packet_count)
			cmsis_dap_swd_read_process(cmsis_dap_handle, CMSIS_DAP_BLOCKING);
	}

	assert(cmsis_dap_handle->pending_fifo[cmsis_dap_handle->pending_fifo_put_idx].transfer_count < pending_queue_len);

	if (queued_retval != ERROR_OK)
		return;

	struct pending_request_block *block = &cmsis_dap_handle->pending_fifo[cmsis_dap_handle->pending_fifo_put_idx];
	struct pending_transfer_result *transfer = &(block->transfers[block->transfer_count]);
	transfer->data = data;
	transfer->cmd = cmd;
	if (block->transfer_count == 0) {
		cmsis_dap_handle->swd_cmds_differ = false;
		cmsis_dap_handle->common_swd_cmd = cmd;
	} else if (cmd != cmsis_dap_handle->common_swd_cmd) {
		cmsis_dap_handle->swd_cmds_differ = true;
	}

	if (cmd & SWD_CMD_RNW) {
		/* Queue a read transaction */
		transfer->buffer = dst;
		cmsis_dap_handle->read_count++;
	} else {
		cmsis_dap_handle->write_count++;
	}
	block->transfer_count++;
}

static void cmsis_dap_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
	assert(!(cmd & SWD_CMD_RNW));
	cmsis_dap_swd_queue_cmd(cmd, NULL, value);
}

static void cmsis_dap_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
	assert(cmd & SWD_CMD_RNW);
	cmsis_dap_swd_queue_cmd(cmd, value, 0);
}

static int cmsis_dap_get_serial_info(void)
{
	uint8_t *data;

	int retval = cmsis_dap_cmd_dap_info(INFO_ID_SERNUM, &data);
	if (retval != ERROR_OK)
		return retval;

	if (data[0]) /* strlen */
		LOG_INFO("CMSIS-DAP: Serial# = %s", &data[1]);

	return ERROR_OK;
}

static int cmsis_dap_get_version_info(void)
{
	uint8_t *data;

	/* INFO_ID_FW_VER - string */
	int retval = cmsis_dap_cmd_dap_info(INFO_ID_FW_VER, &data);
	if (retval != ERROR_OK)
		return retval;

	if (data[0]) /* strlen */
		LOG_INFO("CMSIS-DAP: FW Version = %s", &data[1]);

	return ERROR_OK;
}

static int cmsis_dap_get_caps_info(void)
{
	uint8_t *data;

	/* INFO_ID_CAPS - byte */
	int retval = cmsis_dap_cmd_dap_info(INFO_ID_CAPS, &data);
	if (retval != ERROR_OK)
		return retval;

	if (data[0] == 1 || data[0] == 2) {
		uint16_t caps = data[1];
		if (data[0] == 2)
			caps |= (uint16_t)data[2] << 8;

		cmsis_dap_handle->caps = caps;

		for (unsigned int i = 0; i < INFO_CAPS__NUM_CAPS; ++i) {
			if (caps & BIT(i))
				LOG_INFO("CMSIS-DAP: %s", info_caps_str[i]);
		}
	}

	return ERROR_OK;
}

static int cmsis_dap_get_swo_buf_sz(uint32_t *swo_buf_sz)
{
	uint8_t *data;

	/* INFO_ID_SWO_BUF_SZ - word */
	int retval = cmsis_dap_cmd_dap_info(INFO_ID_SWO_BUF_SZ, &data);
	if (retval != ERROR_OK)
		return retval;

	if (data[0] != 4)
		return ERROR_FAIL;

	*swo_buf_sz = le_to_h_u32(&data[1]);

	LOG_INFO("CMSIS-DAP: SWO Trace Buffer Size = %u bytes", *swo_buf_sz);

	return ERROR_OK;
}

static int cmsis_dap_get_status(void)
{
	uint8_t d;

	int retval = cmsis_dap_cmd_dap_swj_pins(0, 0, 0, &d);

	if (retval == ERROR_OK) {
		LOG_INFO("SWCLK/TCK = %d SWDIO/TMS = %d TDI = %d TDO = %d nTRST = %d nRESET = %d",
			(d & SWJ_PIN_TCK) ? 1 : 0,
			(d & SWJ_PIN_TMS) ? 1 : 0,
			(d & SWJ_PIN_TDI) ? 1 : 0,
			(d & SWJ_PIN_TDO) ? 1 : 0,
			(d & SWJ_PIN_TRST) ? 1 : 0,
			(d & SWJ_PIN_SRST) ? 1 : 0);
	}

	return retval;
}

static int cmsis_dap_swd_switch_seq(enum swd_special_seq seq)
{
	const uint8_t *s;
	unsigned int s_len;
	int retval;

	if (swd_mode)
		queued_retval = cmsis_dap_swd_run_queue();

	if (cmsis_dap_handle->quirk_mode && seq != LINE_RESET &&
			(output_pins & (SWJ_PIN_SRST | SWJ_PIN_TRST))
				== (SWJ_PIN_SRST | SWJ_PIN_TRST)) {
		/* Following workaround deasserts reset on most adapters.
		 * Do not reconnect if a reset line is active!
		 * Reconnecting would break connecting under reset. */

		/* First disconnect before connecting, Atmel EDBG needs it for SAMD/R/L/C */
		cmsis_dap_cmd_dap_disconnect();

		/* When we are reconnecting, DAP_Connect needs to be rerun, at
		 * least on Keil ULINK-ME */
		retval = cmsis_dap_cmd_dap_connect(CONNECT_SWD);
		if (retval != ERROR_OK)
			return retval;
	}

	switch (seq) {
	case LINE_RESET:
		LOG_DEBUG_IO("SWD line reset");
		s = swd_seq_line_reset;
		s_len = swd_seq_line_reset_len;
		break;
	case JTAG_TO_SWD:
		LOG_DEBUG("JTAG-to-SWD");
		s = swd_seq_jtag_to_swd;
		s_len = swd_seq_jtag_to_swd_len;
		break;
	case JTAG_TO_DORMANT:
		LOG_DEBUG("JTAG-to-DORMANT");
		s = swd_seq_jtag_to_dormant;
		s_len = swd_seq_jtag_to_dormant_len;
		break;
	case SWD_TO_JTAG:
		LOG_DEBUG("SWD-to-JTAG");
		s = swd_seq_swd_to_jtag;
		s_len = swd_seq_swd_to_jtag_len;
		break;
	case SWD_TO_DORMANT:
		LOG_DEBUG("SWD-to-DORMANT");
		s = swd_seq_swd_to_dormant;
		s_len = swd_seq_swd_to_dormant_len;
		break;
	case DORMANT_TO_SWD:
		LOG_DEBUG("DORMANT-to-SWD");
		s = swd_seq_dormant_to_swd;
		s_len = swd_seq_dormant_to_swd_len;
		break;
	case DORMANT_TO_JTAG:
		LOG_DEBUG("DORMANT-to-JTAG");
		s = swd_seq_dormant_to_jtag;
		s_len = swd_seq_dormant_to_jtag_len;
		break;
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}

	retval = cmsis_dap_cmd_dap_swj_sequence(s_len, s);
	if (retval != ERROR_OK)
		return retval;

	/* Atmel EDBG needs renew clock setting after SWJ_Sequence
	 * otherwise default frequency is used */
	return cmsis_dap_cmd_dap_swj_clock(adapter_get_speed_khz());
}

static int cmsis_dap_swd_open(void)
{
	if (!(cmsis_dap_handle->caps & INFO_CAPS_SWD)) {
		LOG_ERROR("CMSIS-DAP: SWD not supported");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	int retval = cmsis_dap_cmd_dap_connect(CONNECT_SWD);
	if (retval != ERROR_OK)
		return retval;

	/* Add more setup here.??... */

	LOG_INFO("CMSIS-DAP: Interface Initialised (SWD)");
	return ERROR_OK;
}

static int cmsis_dap_init(void)
{
	uint8_t *data;

	int retval = cmsis_dap_open();
	if (retval != ERROR_OK)
		return retval;

	cmsis_dap_flush_read(cmsis_dap_handle);

	retval = cmsis_dap_get_caps_info();
	if (retval != ERROR_OK)
		return retval;

	retval = cmsis_dap_get_version_info();
	if (retval != ERROR_OK)
		return retval;

	retval = cmsis_dap_get_serial_info();
	if (retval != ERROR_OK)
		return retval;

	if (swd_mode) {
		retval = cmsis_dap_swd_open();
		if (retval != ERROR_OK)
			return retval;
	} else {
		/* Connect in JTAG mode */
		if (!(cmsis_dap_handle->caps & INFO_CAPS_JTAG)) {
			LOG_ERROR("CMSIS-DAP: JTAG not supported");
			return ERROR_JTAG_DEVICE_ERROR;
		}

		retval = cmsis_dap_cmd_dap_connect(CONNECT_JTAG);
		if (retval != ERROR_OK)
			return retval;

		LOG_INFO("CMSIS-DAP: Interface Initialised (JTAG)");
	}

	/* Be conservative and suppress submitting multiple HID requests
	 * until we get packet count info from the adaptor */
	cmsis_dap_handle->packet_count = 1;

	/* INFO_ID_PKT_SZ - short */
	retval = cmsis_dap_cmd_dap_info(INFO_ID_PKT_SZ, &data);
	if (retval != ERROR_OK)
		goto init_err;

	if (data[0] == 2) {  /* short */
		uint16_t pkt_sz = data[1] + (data[2] << 8);
		if (pkt_sz != cmsis_dap_handle->packet_size) {
			cmsis_dap_handle->backend->packet_buffer_free(cmsis_dap_handle);
			retval = cmsis_dap_handle->backend->packet_buffer_alloc(cmsis_dap_handle, pkt_sz);
			if (retval != ERROR_OK)
				goto init_err;

			LOG_DEBUG("CMSIS-DAP: Packet Size = %" PRIu16, pkt_sz);
		}
	}

	/* Maximal number of transfers which fit to one packet:
	 * Limited by response size: 3 bytes of response header + 4 per read
	 * Plus writes to full command size: 3 bytes cmd header + 1 per read + 5 per write */
	tfer_max_command_size = cmsis_dap_handle->packet_usable_size;
	tfer_max_response_size = cmsis_dap_handle->packet_usable_size;
	unsigned int max_reads = tfer_max_response_size / 4;
	pending_queue_len = max_reads + (tfer_max_command_size - max_reads) / 5;
	cmsis_dap_handle->write_count = 0;
	cmsis_dap_handle->read_count = 0;

	/* INFO_ID_PKT_CNT - byte */
	retval = cmsis_dap_cmd_dap_info(INFO_ID_PKT_CNT, &data);
	if (retval != ERROR_OK)
		goto init_err;

	if (data[0] == 1) { /* byte */
		unsigned int pkt_cnt = data[1];
		if (pkt_cnt > 1)
			cmsis_dap_handle->packet_count = MIN(MAX_PENDING_REQUESTS, pkt_cnt);

		LOG_DEBUG("CMSIS-DAP: Packet Count = %u", pkt_cnt);
	}

	LOG_DEBUG("Allocating FIFO for %u pending packets", cmsis_dap_handle->packet_count);
	for (unsigned int i = 0; i < cmsis_dap_handle->packet_count; i++) {
		cmsis_dap_handle->pending_fifo[i].transfers = malloc(pending_queue_len
									 * sizeof(struct pending_transfer_result));
		if (!cmsis_dap_handle->pending_fifo[i].transfers) {
			LOG_ERROR("Unable to allocate memory for CMSIS-DAP queue");
			retval = ERROR_FAIL;
			goto init_err;
		}
	}

	/* Intentionally not checked for error, just logs an info message
	 * not vital for further debugging */
	(void)cmsis_dap_get_status();

	/* Now try to connect to the target
	 * TODO: This is all SWD only @ present */
	retval = cmsis_dap_cmd_dap_swj_clock(adapter_get_speed_khz());
	if (retval != ERROR_OK)
		goto init_err;

	/* Ask CMSIS-DAP to automatically retry on receiving WAIT for
	 * up to 64 times. This must be changed to 0 if sticky
	 * overrun detection is enabled. */
	retval = cmsis_dap_cmd_dap_tfer_configure(0, 64, 0);
	if (retval != ERROR_OK)
		goto init_err;

	if (swd_mode) {
		/* Data Phase (bit 2) must be set to 1 if sticky overrun
		 * detection is enabled */
		retval = cmsis_dap_cmd_dap_swd_configure(0);	/* 1 TRN, no Data Phase */
		if (retval != ERROR_OK)
			goto init_err;
	}
	/* Both LEDs on */
	/* Intentionally not checked for error, debugging will work
	 * without LEDs */
	(void)cmsis_dap_cmd_dap_led(LED_ID_CONNECT, LED_ON);
	(void)cmsis_dap_cmd_dap_led(LED_ID_RUN, LED_ON);

	/* support connecting with srst asserted */
	enum reset_types jtag_reset_config = jtag_get_reset_config();

	if (jtag_reset_config & RESET_CNCT_UNDER_SRST) {
		if (jtag_reset_config & RESET_SRST_NO_GATING) {
			retval = cmsis_dap_cmd_dap_swj_pins(0, SWJ_PIN_SRST, 0, NULL);
			if (retval != ERROR_OK)
				goto init_err;
			LOG_INFO("Connecting under reset");
		}
	}
	LOG_INFO("CMSIS-DAP: Interface ready");
	return ERROR_OK;

init_err:
	cmsis_dap_quit();
	return retval;
}

static int cmsis_dap_swd_init(void)
{
	swd_mode = true;
	return ERROR_OK;
}

static int cmsis_dap_quit(void)
{
	cmsis_dap_cmd_dap_disconnect();

	/* Both LEDs off */
	cmsis_dap_cmd_dap_led(LED_ID_RUN, LED_OFF);
	cmsis_dap_cmd_dap_led(LED_ID_CONNECT, LED_OFF);

	cmsis_dap_close(cmsis_dap_handle);

	return ERROR_OK;
}

static int cmsis_dap_reset(int trst, int srst)
{
	/* Set both TRST and SRST even if they're not enabled as
	 * there's no way to tristate them */

	output_pins = 0;
	if (!srst)
		output_pins |= SWJ_PIN_SRST;
	if (!trst)
		output_pins |= SWJ_PIN_TRST;

	int retval = cmsis_dap_cmd_dap_swj_pins(output_pins,
			SWJ_PIN_TRST | SWJ_PIN_SRST, 0, NULL);
	if (retval != ERROR_OK)
		LOG_ERROR("CMSIS-DAP: Interface reset failed");
	return retval;
}

static void cmsis_dap_execute_sleep(struct jtag_command *cmd)
{
#if 0
	int retval = cmsis_dap_cmd_dap_delay(cmd->cmd.sleep->us);
	if (retval != ERROR_OK)
#endif
		jtag_sleep(cmd->cmd.sleep->us);
}

/* Set TMS high for five TCK clocks, to move the TAP to the Test-Logic-Reset state */
static int cmsis_dap_execute_tlr_reset(struct jtag_command *cmd)
{
	LOG_INFO("cmsis-dap JTAG TLR_RESET");
	uint8_t seq = 0xff;

	int retval = cmsis_dap_cmd_dap_swj_sequence(8, &seq);
	if (retval == ERROR_OK)
		tap_set_state(TAP_RESET);
	return retval;
}

/* Set new end state */
static void cmsis_dap_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else {
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

#ifdef SPRINT_BINARY
static void sprint_binary(char *s, const uint8_t *buf, unsigned int offset, unsigned int len)
{
	if (!len)
		return;

	/*
	buf = { 0x18 } len=5 should result in: 11000
	buf = { 0xff 0x18 } len=13 should result in: 11111111 11000
	buf = { 0xc0 0x18 } offset=3 len=10 should result in: 11000 11000
		i=3 there means i/8 = 0 so c = 0xFF, and
	*/
	for (unsigned int i = offset; i < offset + len; ++i) {
		uint8_t c = buf[i / 8], mask = 1 << (i % 8);
		if ((i != offset) && !(i % 8))
			putchar(' ');
		*s++ = (c & mask) ? '1' : '0';
	}
	*s = 0;
}
#endif

#ifdef CMSIS_DAP_JTAG_DEBUG
static void debug_parse_cmsis_buf(const uint8_t *cmd, int cmdlen)
{
	/* cmd is a usb packet to go to the cmsis-dap interface */
	printf("cmsis-dap buffer (%d b): ", cmdlen);
	for (int i = 0; i < cmdlen; ++i)
		printf(" %02x", cmd[i]);
	printf("\n");
	switch (cmd[0]) {
		case CMD_DAP_JTAG_SEQ: {
			printf("cmsis-dap jtag sequence command %02x (n=%d)\n", cmd[0], cmd[1]);
			/*
			 * #1 = number of sequences
			 * #2 = sequence info 1
			 * #3...4+n_bytes-1 = sequence 1
			 * #4+n_bytes = sequence info 2
			 * #5+n_bytes = sequence 2 (single bit)
			 */
			int pos = 2;
			for (int seq = 0; seq < cmd[1]; ++seq) {
				uint8_t info = cmd[pos++];
				int len = info & DAP_JTAG_SEQ_TCK;
				if (len == 0)
					len = 64;
				printf("  sequence %d starting %d: info %02x (len=%d tms=%d read_tdo=%d): ",
					seq, pos, info, len, info & DAP_JTAG_SEQ_TMS, info & DAP_JTAG_SEQ_TDO);
				for (int i = 0; i < DIV_ROUND_UP(len, 8); ++i)
					printf(" %02x", cmd[pos+i]);
				pos += DIV_ROUND_UP(len, 8);
				printf("\n");
			}
			if (pos != cmdlen) {
				printf("BUFFER LENGTH MISMATCH looks like %d but %d specified", pos, cmdlen);
				exit(-1);
			}

			break;
		}
		default:
			LOG_DEBUG("unknown cmsis-dap command %02x", cmd[1]);
			break;
	}
}
#endif

static void cmsis_dap_flush(void)
{
	if (!queued_seq_count)
		return;

	LOG_DEBUG_IO("Flushing %d queued sequences (%d bytes) with %d pending scan results to capture",
		queued_seq_count, queued_seq_buf_end, pending_scan_result_count);

	/* prepare CMSIS-DAP packet */
	uint8_t *command = cmsis_dap_handle->command;
	command[0] = CMD_DAP_JTAG_SEQ;
	command[1] = queued_seq_count;
	memcpy(&command[2], queued_seq_buf, queued_seq_buf_end);

#ifdef CMSIS_DAP_JTAG_DEBUG
	debug_parse_cmsis_buf(command, queued_seq_buf_end + 2);
#endif

	/* send command to USB device */
	int retval = cmsis_dap_xfer(cmsis_dap_handle, queued_seq_buf_end + 2);

	uint8_t *resp = cmsis_dap_handle->response;
	if (retval != ERROR_OK || resp[1] != DAP_OK) {
		LOG_ERROR("CMSIS-DAP command CMD_DAP_JTAG_SEQ failed.");
		exit(-1);
	}

#ifdef CMSIS_DAP_JTAG_DEBUG
	LOG_DEBUG_IO("USB response buf:");
	for (int c = 0; c < queued_seq_buf_end + 3; ++c)
		printf("%02X ", resp[c]);
	printf("\n");
#endif

	/* copy scan results into client buffers */
	for (int i = 0; i < pending_scan_result_count; ++i) {
		struct pending_scan_result *scan = &pending_scan_results[i];
		LOG_DEBUG_IO("Copying pending_scan_result %d/%d: %d bits from byte %d -> buffer + %d bits",
			i, pending_scan_result_count, scan->length, scan->first + 2, scan->buffer_offset);
#ifdef CMSIS_DAP_JTAG_DEBUG
		for (uint32_t b = 0; b < DIV_ROUND_UP(scan->length, 8); ++b)
			printf("%02X ", resp[2+scan->first+b]);
		printf("\n");
#endif
		bit_copy(scan->buffer, scan->buffer_offset, &resp[2 + scan->first], 0, scan->length);
	}

	/* reset */
	queued_seq_count = 0;
	queued_seq_buf_end = 0;
	queued_seq_tdo_ptr = 0;
	pending_scan_result_count = 0;
}

/* queue a sequence of bits to clock out TDI / in TDO, executing if the buffer is full.
 *
 * sequence=NULL means clock out zeros on TDI
 * tdo_buffer=NULL means don't capture TDO
 */
static void cmsis_dap_add_jtag_sequence(unsigned int s_len, const uint8_t *sequence,
					unsigned int s_offset, bool tms,
					uint8_t *tdo_buffer, unsigned int tdo_buffer_offset)
{
	LOG_DEBUG_IO("[at %d] %u bits, tms %s, seq offset %u, tdo buf %p, tdo offset %u",
		queued_seq_buf_end,
		s_len, tms ? "HIGH" : "LOW", s_offset, tdo_buffer, tdo_buffer_offset);

	if (s_len == 0)
		return;

	if (s_len > 64) {
		LOG_DEBUG_IO("START JTAG SEQ SPLIT");
		for (unsigned int offset = 0; offset < s_len; offset += 64) {
			unsigned int len = s_len - offset;
			if (len > 64)
				len = 64;
			LOG_DEBUG_IO("Splitting long jtag sequence: %u-bit chunk starting at offset %u", len, offset);
			cmsis_dap_add_jtag_sequence(
				len,
				sequence,
				s_offset + offset,
				tms,
				tdo_buffer,
				!tdo_buffer ? 0 : (tdo_buffer_offset + offset)
				);
		}
		LOG_DEBUG_IO("END JTAG SEQ SPLIT");
		return;
	}

	unsigned int cmd_len = 1 + DIV_ROUND_UP(s_len, 8);
	if (queued_seq_count >= 255 || queued_seq_buf_end + cmd_len > QUEUED_SEQ_BUF_LEN)
		/* empty out the buffer */
		cmsis_dap_flush();

	++queued_seq_count;

	/* control byte */
	queued_seq_buf[queued_seq_buf_end] =
		(tms ? DAP_JTAG_SEQ_TMS : 0) |
		(tdo_buffer ? DAP_JTAG_SEQ_TDO : 0) |
		(s_len == 64 ? 0 : s_len);

	if (sequence)
		bit_copy(&queued_seq_buf[queued_seq_buf_end + 1], 0, sequence, s_offset, s_len);
	else
		memset(&queued_seq_buf[queued_seq_buf_end + 1], 0, DIV_ROUND_UP(s_len, 8));

	queued_seq_buf_end += cmd_len;

	if (tdo_buffer) {
		struct pending_scan_result *scan = &pending_scan_results[pending_scan_result_count++];
		scan->first = queued_seq_tdo_ptr;
		queued_seq_tdo_ptr += DIV_ROUND_UP(s_len, 8);
		scan->length = s_len;
		scan->buffer = tdo_buffer;
		scan->buffer_offset = tdo_buffer_offset;
	}
}

/* queue a sequence of bits to clock out TMS, executing if the buffer is full */
static void cmsis_dap_add_tms_sequence(const uint8_t *sequence, int s_len)
{
	LOG_DEBUG_IO("%d bits: %02X", s_len, *sequence);
	/* we use a series of CMD_DAP_JTAG_SEQ commands to toggle TMS,
	   because even though it seems ridiculously inefficient, it
	   allows us to combine TMS and scan sequences into the same
	   USB packet. */
	/* TODO: combine runs of the same tms value */
	for (int i = 0; i < s_len; ++i) {
		bool bit = (sequence[i / 8] & (1 << (i % 8))) != 0;
		cmsis_dap_add_jtag_sequence(1, NULL, 0, bit, NULL, 0);
	}
}

/* Move to the end state by queuing a sequence to clock into TMS */
static void cmsis_dap_state_move(void)
{
	uint8_t tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());
	uint8_t tms_scan_bits = tap_get_tms_path_len(tap_get_state(), tap_get_end_state());

	LOG_DEBUG_IO("state move from %s to %s: %d clocks, %02X on tms",
		tap_state_name(tap_get_state()), tap_state_name(tap_get_end_state()),
		tms_scan_bits, tms_scan);
	cmsis_dap_add_tms_sequence(&tms_scan, tms_scan_bits);

	tap_set_state(tap_get_end_state());
}


/* Execute a JTAG scan operation by queueing TMS and TDI/TDO sequences */
static void cmsis_dap_execute_scan(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("%s type:%d", cmd->cmd.scan->ir_scan ? "IRSCAN" : "DRSCAN",
		jtag_scan_type(cmd->cmd.scan));

	/* Make sure there are no trailing fields with num_bits == 0, or the logic below will fail. */
	while (cmd->cmd.scan->num_fields > 0
			&& cmd->cmd.scan->fields[cmd->cmd.scan->num_fields - 1].num_bits == 0) {
		cmd->cmd.scan->num_fields--;
		LOG_DEBUG("discarding trailing empty field");
	}

	if (!cmd->cmd.scan->num_fields) {
		LOG_DEBUG("empty scan, doing nothing");
		return;
	}

	if (cmd->cmd.scan->ir_scan) {
		if (tap_get_state() != TAP_IRSHIFT) {
			cmsis_dap_end_state(TAP_IRSHIFT);
			cmsis_dap_state_move();
		}
	} else {
		if (tap_get_state() != TAP_DRSHIFT) {
			cmsis_dap_end_state(TAP_DRSHIFT);
			cmsis_dap_state_move();
		}
	}

	cmsis_dap_end_state(cmd->cmd.scan->end_state);

	struct scan_field *field = cmd->cmd.scan->fields;
	unsigned int scan_size = 0;

	for (unsigned int i = 0; i < cmd->cmd.scan->num_fields; i++, field++) {
		scan_size += field->num_bits;
		LOG_DEBUG_IO("%s%s field %u/%u %u bits",
			field->in_value ? "in" : "",
			field->out_value ? "out" : "",
			i,
			cmd->cmd.scan->num_fields,
			field->num_bits);

		if (i == cmd->cmd.scan->num_fields - 1 && tap_get_state() != tap_get_end_state()) {
			LOG_DEBUG_IO("Last field and have to move out of SHIFT state");
			/* Last field, and we're leaving IRSHIFT/DRSHIFT. Clock last bit during tap
			 * movement. This last field can't have length zero, it was checked above. */
			cmsis_dap_add_jtag_sequence(
				field->num_bits - 1, /* number of bits to clock */
				field->out_value, /* output sequence */
				0, /* output offset */
				false, /* TMS low */
				field->in_value,
				0);

			/* Clock the last bit out, with TMS high */
			uint8_t last_bit = 0;
			if (field->out_value)
				bit_copy(&last_bit, 0, field->out_value, field->num_bits - 1, 1);
			cmsis_dap_add_jtag_sequence(
				1,
				&last_bit,
				0,
				true,
				field->in_value,
				field->num_bits - 1);
			tap_set_state(tap_state_transition(tap_get_state(), 1));

			/* Now clock one more cycle, with TMS low, to get us into a PAUSE state */
			cmsis_dap_add_jtag_sequence(
				1,
				&last_bit,
				0,
				false,
				NULL,
				0);
			tap_set_state(tap_state_transition(tap_get_state(), 0));
		} else {
			LOG_DEBUG_IO("Internal field, staying in SHIFT state afterwards");
			/* Clocking part of a sequence into DR or IR with TMS=0,
			   leaving TMS=0 at the end so we can continue later */
			cmsis_dap_add_jtag_sequence(
				field->num_bits,
				field->out_value,
				0,
				false,
				field->in_value,
				0);
		}
	}

	if (tap_get_state() != tap_get_end_state()) {
		cmsis_dap_end_state(tap_get_end_state());
		cmsis_dap_state_move();
	}

	LOG_DEBUG_IO("%s scan, %i bits, end in %s",
		(cmd->cmd.scan->ir_scan) ? "IR" : "DR", scan_size,
		tap_state_name(tap_get_end_state()));
}

static void cmsis_dap_pathmove(int num_states, tap_state_t *path)
{
	uint8_t tms0 = 0x00;
	uint8_t tms1 = 0xff;

	for (int i = 0; i < num_states; i++) {
		if (path[i] == tap_state_transition(tap_get_state(), false))
			cmsis_dap_add_tms_sequence(&tms0, 1);
		else if (path[i] == tap_state_transition(tap_get_state(), true))
			cmsis_dap_add_tms_sequence(&tms1, 1);
		else {
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition.",
				  tap_state_name(tap_get_state()), tap_state_name(path[i]));
			exit(-1);
		}

		tap_set_state(path[i]);
	}

	cmsis_dap_end_state(tap_get_state());
}

static void cmsis_dap_execute_pathmove(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("pathmove: %i states, end in %i",
		      cmd->cmd.pathmove->num_states,
	       cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);

	cmsis_dap_pathmove(cmd->cmd.pathmove->num_states, cmd->cmd.pathmove->path);
}

static void cmsis_dap_stableclocks(unsigned int num_cycles)
{
	uint8_t tms = tap_get_state() == TAP_RESET;
	/* TODO: Perform optimizations? */
	/* Execute num_cycles. */
	for (unsigned int i = 0; i < num_cycles; i++)
		cmsis_dap_add_tms_sequence(&tms, 1);
}

static void cmsis_dap_runtest(unsigned int num_cycles)
{
	tap_state_t saved_end_state = tap_get_end_state();

	/* Only do a state_move when we're not already in IDLE. */
	if (tap_get_state() != TAP_IDLE) {
		cmsis_dap_end_state(TAP_IDLE);
		cmsis_dap_state_move();
	}
	cmsis_dap_stableclocks(num_cycles);

	/* Finish in end_state. */
	cmsis_dap_end_state(saved_end_state);

	if (tap_get_state() != tap_get_end_state())
		cmsis_dap_state_move();
}

static void cmsis_dap_execute_runtest(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("runtest %u cycles, end in %i", cmd->cmd.runtest->num_cycles,
		      cmd->cmd.runtest->end_state);

	cmsis_dap_end_state(cmd->cmd.runtest->end_state);
	cmsis_dap_runtest(cmd->cmd.runtest->num_cycles);
}

static void cmsis_dap_execute_stableclocks(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("stableclocks %u cycles", cmd->cmd.runtest->num_cycles);
	cmsis_dap_stableclocks(cmd->cmd.runtest->num_cycles);
}

static void cmsis_dap_execute_tms(struct jtag_command *cmd)
{
	LOG_DEBUG_IO("TMS: %u bits", cmd->cmd.tms->num_bits);
	cmsis_dap_cmd_dap_swj_sequence(cmd->cmd.tms->num_bits, cmd->cmd.tms->bits);
}

/* TODO: Is there need to call cmsis_dap_flush() for the JTAG_PATHMOVE,
 * JTAG_RUNTEST, JTAG_STABLECLOCKS? */
static void cmsis_dap_execute_command(struct jtag_command *cmd)
{
	switch (cmd->type) {
		case JTAG_SLEEP:
			cmsis_dap_flush();
			cmsis_dap_execute_sleep(cmd);
			break;
		case JTAG_TLR_RESET:
			cmsis_dap_flush();
			cmsis_dap_execute_tlr_reset(cmd);
			break;
		case JTAG_SCAN:
			cmsis_dap_execute_scan(cmd);
			break;
		case JTAG_PATHMOVE:
			cmsis_dap_execute_pathmove(cmd);
			break;
		case JTAG_RUNTEST:
			cmsis_dap_execute_runtest(cmd);
			break;
		case JTAG_STABLECLOCKS:
			cmsis_dap_execute_stableclocks(cmd);
			break;
		case JTAG_TMS:
			cmsis_dap_execute_tms(cmd);
			break;
		default:
			LOG_ERROR("BUG: unknown JTAG command type 0x%X encountered", cmd->type);
			exit(-1);
	}
}

static int cmsis_dap_execute_queue(struct jtag_command *cmd_queue)
{
	struct jtag_command *cmd = cmd_queue;

	while (cmd) {
		cmsis_dap_execute_command(cmd);
		cmd = cmd->next;
	}

	cmsis_dap_flush();

	return ERROR_OK;
}

static int cmsis_dap_speed(int speed)
{
	if (speed == 0) {
		LOG_ERROR("RTCK not supported. Set nonzero \"adapter speed\".");
		return ERROR_JTAG_NOT_IMPLEMENTED;
	}

	return cmsis_dap_cmd_dap_swj_clock(speed);
}

static int cmsis_dap_speed_div(int speed, int *khz)
{
	*khz = speed;
	return ERROR_OK;
}

static int cmsis_dap_khz(int khz, int *jtag_speed)
{
	*jtag_speed = khz;
	return ERROR_OK;
}

static bool calculate_swo_prescaler(unsigned int traceclkin_freq,
		uint32_t trace_freq, uint16_t *prescaler)
{
	unsigned int presc = (traceclkin_freq + trace_freq / 2) / trace_freq;
	if (presc == 0 || presc > TPIU_ACPR_MAX_SWOSCALER + 1)
		return false;

	/* Probe's UART speed must be within 3% of the TPIU's SWO baud rate. */
	unsigned int max_deviation = (traceclkin_freq * 3) / 100;
	if (presc * trace_freq < traceclkin_freq - max_deviation ||
	    presc * trace_freq > traceclkin_freq + max_deviation)
		return false;

	*prescaler = presc;

	return true;
}

/**
 * @see adapter_driver::config_trace
 */
static int cmsis_dap_config_trace(
				bool trace_enabled,
				enum tpiu_pin_protocol pin_protocol,
				uint32_t port_size,
				unsigned int *swo_freq,
				unsigned int traceclkin_hz,
				uint16_t *swo_prescaler)
{
	int retval;

	if (!trace_enabled) {
		if (cmsis_dap_handle->trace_enabled) {
			retval = cmsis_dap_cmd_dap_swo_control(DAP_SWO_CONTROL_STOP);
			if (retval != ERROR_OK) {
				LOG_ERROR("Failed to disable the SWO-trace.");
				return retval;
			}
		}
		cmsis_dap_handle->trace_enabled = false;
		LOG_INFO("SWO-trace disabled.");
		return ERROR_OK;
	}

	if (!(cmsis_dap_handle->caps & INFO_CAPS_SWO_UART) &&
	    !(cmsis_dap_handle->caps & INFO_CAPS_SWO_MANCHESTER)) {
		LOG_ERROR("SWO-trace is not supported by the device.");
		return ERROR_FAIL;
	}

	uint8_t swo_mode;
	if (pin_protocol == TPIU_PIN_PROTOCOL_ASYNC_UART &&
	   (cmsis_dap_handle->caps & INFO_CAPS_SWO_UART)) {
		swo_mode = DAP_SWO_MODE_UART;
	} else if (pin_protocol == TPIU_PIN_PROTOCOL_ASYNC_MANCHESTER &&
		  (cmsis_dap_handle->caps & INFO_CAPS_SWO_MANCHESTER)) {
		swo_mode = DAP_SWO_MODE_MANCHESTER;
	} else {
		LOG_ERROR("Selected pin protocol is not supported.");
		return ERROR_FAIL;
	}

	if (*swo_freq == 0) {
		LOG_INFO("SWO-trace frequency autodetection not implemented.");
		return ERROR_FAIL;
	}

	retval = cmsis_dap_cmd_dap_swo_control(DAP_SWO_CONTROL_STOP);
	if (retval != ERROR_OK)
		return retval;

	cmsis_dap_handle->trace_enabled = false;

	retval = cmsis_dap_get_swo_buf_sz(&cmsis_dap_handle->swo_buf_sz);
	if (retval != ERROR_OK)
		return retval;

	retval = cmsis_dap_cmd_dap_swo_transport(DAP_SWO_TRANSPORT_DATA);
	if (retval != ERROR_OK)
		return retval;

	retval = cmsis_dap_cmd_dap_swo_mode(swo_mode);
	if (retval != ERROR_OK)
		return retval;

	retval = cmsis_dap_cmd_dap_swo_baudrate(*swo_freq, swo_freq);
	if (retval != ERROR_OK)
		return retval;

	if (!calculate_swo_prescaler(traceclkin_hz, *swo_freq,
			swo_prescaler)) {
		LOG_ERROR("SWO frequency is not suitable. Please choose a "
			"different frequency or use auto-detection.");
		return ERROR_FAIL;
	}

	LOG_INFO("SWO frequency: %u Hz.", *swo_freq);
	LOG_INFO("SWO prescaler: %u.", *swo_prescaler);

	retval = cmsis_dap_cmd_dap_swo_control(DAP_SWO_CONTROL_START);
	if (retval != ERROR_OK)
		return retval;

	cmsis_dap_handle->trace_enabled = true;

	return ERROR_OK;
}

/**
 * @see adapter_driver::poll_trace
 */
static int cmsis_dap_poll_trace(uint8_t *buf, size_t *size)
{
	uint8_t trace_status;
	size_t trace_count;

	if (!cmsis_dap_handle->trace_enabled) {
		*size = 0;
		return ERROR_OK;
	}

	int retval = cmsis_dap_cmd_dap_swo_status(&trace_status, &trace_count);
	if (retval != ERROR_OK)
		return retval;
	if ((trace_status & DAP_SWO_STATUS_CAPTURE_MASK) != DAP_SWO_STATUS_CAPTURE_ACTIVE)
		return ERROR_FAIL;

	*size = trace_count < *size ? trace_count : *size;
	size_t read_so_far = 0;
	do {
		size_t rb = 0;
		uint32_t packet_size = cmsis_dap_handle->packet_size - 4 /*data-reply*/;
		uint32_t remaining = *size - read_so_far;
		if (remaining < packet_size)
			packet_size = remaining;
		retval = cmsis_dap_cmd_dap_swo_data(
						packet_size,
						&trace_status,
						&rb,
						&buf[read_so_far]);
		if (retval != ERROR_OK)
			return retval;
		if ((trace_status & DAP_SWO_STATUS_CAPTURE_MASK) != DAP_SWO_STATUS_CAPTURE_ACTIVE)
			return ERROR_FAIL;

		read_so_far += rb;
	} while (read_so_far < *size);

	return ERROR_OK;
}

COMMAND_HANDLER(cmsis_dap_handle_info_command)
{
	if (cmsis_dap_get_version_info() == ERROR_OK)
		cmsis_dap_get_status();

	return ERROR_OK;
}

COMMAND_HANDLER(cmsis_dap_handle_cmd_command)
{
	uint8_t *command = cmsis_dap_handle->command;

	for (unsigned int i = 0; i < CMD_ARGC; i++)
		COMMAND_PARSE_NUMBER(u8, CMD_ARGV[i], command[i]);

	int retval = cmsis_dap_xfer(cmsis_dap_handle, CMD_ARGC);

	if (retval != ERROR_OK) {
		LOG_ERROR("CMSIS-DAP command failed.");
		return ERROR_JTAG_DEVICE_ERROR;
	}

	uint8_t *resp = cmsis_dap_handle->response;
	LOG_INFO("Returned data %02" PRIx8 " %02" PRIx8 " %02" PRIx8 " %02" PRIx8,
		resp[1], resp[2], resp[3], resp[4]);

	return ERROR_OK;
}

COMMAND_HANDLER(cmsis_dap_handle_vid_pid_command)
{
	if (CMD_ARGC > MAX_USB_IDS * 2) {
		LOG_WARNING("ignoring extra IDs in cmsis-dap vid_pid "
			"(maximum is %d pairs)", MAX_USB_IDS);
		CMD_ARGC = MAX_USB_IDS * 2;
	}
	if (CMD_ARGC < 2 || (CMD_ARGC & 1)) {
		LOG_WARNING("incomplete cmsis-dap vid_pid configuration directive");
		if (CMD_ARGC < 2)
			return ERROR_COMMAND_SYNTAX_ERROR;
		/* remove the incomplete trailing id */
		CMD_ARGC -= 1;
	}

	unsigned int i;
	for (i = 0; i < CMD_ARGC; i += 2) {
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i], cmsis_dap_vid[i >> 1]);
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[i + 1], cmsis_dap_pid[i >> 1]);
	}

	/*
	 * Explicitly terminate, in case there are multiples instances of
	 * cmsis_dap_vid_pid.
	 */
	cmsis_dap_vid[i >> 1] = cmsis_dap_pid[i >> 1] = 0;

	return ERROR_OK;
}

COMMAND_HANDLER(cmsis_dap_handle_backend_command)
{
	if (CMD_ARGC == 1) {
		if (strcmp(CMD_ARGV[0], "auto") == 0) {
			cmsis_dap_backend = -1; /* autoselect */
		} else {
			for (unsigned int i = 0; i < ARRAY_SIZE(cmsis_dap_backends); i++) {
				if (strcasecmp(cmsis_dap_backends[i]->name, CMD_ARGV[0]) == 0) {
					cmsis_dap_backend = i;
					return ERROR_OK;
				}
			}

			command_print(CMD, "invalid backend argument to cmsis-dap backend <backend>");
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
	} else {
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(cmsis_dap_handle_quirk_command)
{
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC == 1)
		COMMAND_PARSE_ENABLE(CMD_ARGV[0], cmsis_dap_handle->quirk_mode);

	command_print(CMD, "CMSIS-DAP quirk workarounds %s",
				  cmsis_dap_handle->quirk_mode ? "enabled" : "disabled");
	return ERROR_OK;
}

static const struct command_registration cmsis_dap_subcommand_handlers[] = {
	{
		.name = "info",
		.handler = &cmsis_dap_handle_info_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "show cmsis-dap info",
	},
	{
		.name = "cmd",
		.handler = &cmsis_dap_handle_cmd_command,
		.mode = COMMAND_EXEC,
		.usage = "",
		.help = "issue cmsis-dap command",
	},
	{
		.name = "vid_pid",
		.handler = &cmsis_dap_handle_vid_pid_command,
		.mode = COMMAND_CONFIG,
		.help = "the vendor ID and product ID of the CMSIS-DAP device",
		.usage = "(vid pid)*",
	},
	{
		.name = "backend",
		.handler = &cmsis_dap_handle_backend_command,
		.mode = COMMAND_CONFIG,
		.help = "set the communication backend to use (USB bulk or HID).",
		.usage = "(auto | usb_bulk | hid)",
	},
	{
		.name = "quirk",
		.handler = &cmsis_dap_handle_quirk_command,
		.mode = COMMAND_ANY,
		.help = "allow expensive workarounds of known adapter quirks.",
		.usage = "[enable | disable]",
	},
#if BUILD_CMSIS_DAP_USB
	{
		.name = "usb",
		.chain = cmsis_dap_usb_subcommand_handlers,
		.mode = COMMAND_ANY,
		.help = "USB bulk backend-specific commands",
		.usage = "<cmd>",
	},
#endif
	COMMAND_REGISTRATION_DONE
};


static const struct command_registration cmsis_dap_command_handlers[] = {
	{
		.name = "cmsis-dap",
		.mode = COMMAND_ANY,
		.help = "perform CMSIS-DAP management",
		.usage = "<cmd>",
		.chain = cmsis_dap_subcommand_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

static const struct swd_driver cmsis_dap_swd_driver = {
	.init = cmsis_dap_swd_init,
	.switch_seq = cmsis_dap_swd_switch_seq,
	.read_reg = cmsis_dap_swd_read_reg,
	.write_reg = cmsis_dap_swd_write_reg,
	.run = cmsis_dap_swd_run_queue,
};

static const char * const cmsis_dap_transport[] = { "swd", "jtag", NULL };

static struct jtag_interface cmsis_dap_interface = {
	.supported = DEBUG_CAP_TMS_SEQ,
	.execute_queue = cmsis_dap_execute_queue,
};

struct adapter_driver cmsis_dap_adapter_driver = {
	.name = "cmsis-dap",
	.transports = cmsis_dap_transport,
	.commands = cmsis_dap_command_handlers,

	.init = cmsis_dap_init,
	.quit = cmsis_dap_quit,
	.reset = cmsis_dap_reset,
	.speed = cmsis_dap_speed,
	.khz = cmsis_dap_khz,
	.speed_div = cmsis_dap_speed_div,
	.config_trace = cmsis_dap_config_trace,
	.poll_trace = cmsis_dap_poll_trace,

	.jtag_ops = &cmsis_dap_interface,
	.swd_ops = &cmsis_dap_swd_driver,
};
