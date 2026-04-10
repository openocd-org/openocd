// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   T-Head CK-Link JTAG adapter driver.                                   *
 *                                                                         *
 *   Supports the CK-Link Lite V2 wire protocol used by T-Head probes and  *
 *   Bouffalo Lab BL616/BL702 CK-Link clones. USB bulk with a framed       *
 *   command protocol reverse-engineered from T-Head's DebugServer.        *
 *                                                                         *
 *   Copyright (C) 2026 by William Markezana                               *
 *       <william.markezana@gmail.com>                                     *
 *                                                                         *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <assert.h>

#include <jtag/adapter.h>
#include <jtag/interface.h>
#include <jtag/commands.h>
#include <helper/binarybuffer.h>
#include <helper/bits.h>
#include <helper/time_support.h>
#include "libusb_helper.h"

// USB identification. Defaults can be overridden via `adapter usb vid_pid`.
#define CKLINK_VID_BOUFFALO 0x42bf
#define CKLINK_VID_THEAD 0x32bf
#define CKLINK_PID_LITE_V2 0xb210

#define CKLINK_USB_INTERFACE 0
#define CKLINK_EP_OUT 0x02
#define CKLINK_EP_IN 0x81
#define CKLINK_USB_TIMEOUT_MS 1000
#define CKLINK_USB_BUF_SIZE 2048

// Wire-protocol framing bytes
#define CKLINK_FRAME_START 0x68
#define CKLINK_FRAME_END 0x16

// Response frame layout: start + status + payload + checksum + end
#define CKLINK_FRAME_OVERHEAD 4
#define CKLINK_RESP_PAYLOAD_OFFSET 2

// Opcodes
#define CKLINK_OP_SELFREG_WRITE 0x06
#define CKLINK_OP_SELFREG_READ 0x87
#define CKLINK_OP_JTAG_BATCH 0x88

// Self-register indices
#define CKLINK_SR_CSR 0 // clock divider + CDI mode
#define CKLINK_SR_MTCR_WAIT 1 // target-comm wait cycles
#define CKLINK_SR_JTAG_CONFIG 8 // JTAG config word
#define CKLINK_SELFREG_BYTES 4

/*
 * Init values replayed from a captured DebugServer session.
 * sr0 (CSR): byte 0 = clock divider, byte 3 = CDI mode (0x60=5-wire JTAG,
 * 0x61=2-wire cJTAG). The 5->2->5 mode toggle at end of init acts as a
 * TAP reset: the wire driver re-initializes on each switch.
 * sr1 MTCR_WAIT = 1000 cycles. sr8 JTAG_CONFIG: opaque, known-good.
 */
#define CKLINK_SR0_SETUP_CLK_BYTE 0x17 // ~2000 kHz
#define CKLINK_SR0_RUN_CLK_BYTE 0x12 // ~2526 kHz, DebugServer default
#define CKLINK_SR0_MODE_JTAG 0x60
#define CKLINK_SR0_MODE_CJTAG 0x61
#define CKLINK_SR1_INIT_VALUE 0x000003e8
#define CKLINK_SR8_INIT_VALUE 0x00622250

// freq_kHz = 48000 / (N + 1); cap at DebugServer-validated 0x12.
#define CKLINK_CLK_SOURCE_KHZ 48000
#define CKLINK_CLK_DIV_MIN CKLINK_SR0_RUN_CLK_BYTE
#define CKLINK_CLK_DIV_MAX 0xff

#define CKLINK_CSR(clk, mode) (((uint32_t)(mode) << 24) | (clk))

// Batch opcode encodes IR/DR bit-counts in single bytes.
#define CKLINK_MAX_SCAN_BITS 255
#define CKLINK_MAX_SCAN_BYTES 32 // DIV_ROUND_UP(255, 8)

// Synthesized IR when a DR scan arrives without a cached IR.
#define CKLINK_PLACEHOLDER_IR_BITS 5
#define CKLINK_PLACEHOLDER_IR_VALUE 0x01

struct cklink_ctx {
	struct libusb_device_handle *usb_dev;
	uint8_t tx_buf[CKLINK_USB_BUF_SIZE];
	uint8_t rx_buf[CKLINK_USB_BUF_SIZE];
	uint8_t csr_clk;
	uint8_t ir_value[CKLINK_MAX_SCAN_BYTES];
	unsigned int ir_bits;
	struct jtag_command *pending_ir_cmd;
	uint8_t *scan_tdi;
	uint8_t *scan_tdo;
	size_t scan_buf_size;
	bool long_scan_warned;
	bool dropped_ir_warned;
};

static struct cklink_ctx *cklink_handle;

// Built-in VID/PID list, used when `adapter usb vid_pid` is not set.
static const uint16_t cklink_default_vids[] = {
	CKLINK_VID_BOUFFALO,
	CKLINK_VID_THEAD,
	0,
};
static const uint16_t cklink_default_pids[] = {
	CKLINK_PID_LITE_V2,
	CKLINK_PID_LITE_V2,
	0,
};

// Sum-mod-256.
static uint8_t cklink_checksum(const uint8_t *buf, unsigned int len)
{
	uint8_t sum = 0;
	for (unsigned int i = 0; i < len; i++)
		sum += buf[i];
	return sum;
}

/* Send a framed command; NULL rxlen means fire-and-forget (selfreg writes
 * are not acknowledged). @p opcode selects the response-checksum rule. */
static int cklink_usb_xfer(struct cklink_ctx *ck, uint8_t opcode,
		unsigned int txlen, unsigned int *rxlen)
{
	int actual = 0;
	int ret = jtag_libusb_bulk_write(ck->usb_dev, CKLINK_EP_OUT,
			(char *)ck->tx_buf, txlen,
			CKLINK_USB_TIMEOUT_MS, &actual);
	if (ret != ERROR_OK || (unsigned int)actual != txlen) {
		LOG_ERROR("CK-Link bulk write failed (ret=%d, actual=%d/%d)",
			ret, actual, txlen);
		return ERROR_FAIL;
	}

	if (!rxlen)
		return ERROR_OK;

	ret = jtag_libusb_bulk_read(ck->usb_dev, CKLINK_EP_IN,
			(char *)ck->rx_buf, CKLINK_USB_BUF_SIZE,
			CKLINK_USB_TIMEOUT_MS, &actual);
	if (ret != ERROR_OK || actual < (int)CKLINK_FRAME_OVERHEAD) {
		LOG_ERROR("CK-Link bulk read failed (ret=%d, actual=%d)",
			ret, actual);
		return ERROR_FAIL;
	}
	if (ck->rx_buf[0] != CKLINK_FRAME_START) {
		LOG_ERROR("CK-Link bad response start byte 0x%02x",
			ck->rx_buf[0]);
		return ERROR_FAIL;
	}
	if (ck->rx_buf[actual - 1] != CKLINK_FRAME_END) {
		LOG_ERROR("CK-Link bad response end byte 0x%02x",
			ck->rx_buf[actual - 1]);
		return ERROR_FAIL;
	}
	/*
	 * RX checksum is opcode-specific: selfreg reads sum all payload;
	 * single-entry JTAG batch sums all except the last byte of dr_tdo.
	 */
	unsigned int csum_end = actual - ((opcode == CKLINK_OP_JTAG_BATCH) ? 3 : 2);
	uint8_t computed = cklink_checksum(ck->rx_buf, csum_end);
	if (computed != ck->rx_buf[actual - 2]) {
		LOG_ERROR("CK-Link response checksum mismatch: op=0x%02x stated=0x%02x computed=0x%02x",
			opcode, ck->rx_buf[actual - 2], computed);
		return ERROR_FAIL;
	}
	// Status byte semantics undocumented; log non-zero for traces.
	if (ck->rx_buf[1] != 0) {
		LOG_DEBUG("CK-Link op=0x%02x returned status=0x%02x",
			opcode, ck->rx_buf[1]);
	}
	*rxlen = (unsigned int)actual;
	return ERROR_OK;
}

static unsigned int cklink_frame_start(struct cklink_ctx *ck, uint8_t opcode)
{
	ck->tx_buf[0] = CKLINK_FRAME_START;
	ck->tx_buf[1] = opcode;
	return 2;
}

static unsigned int cklink_frame_end(struct cklink_ctx *ck, unsigned int pos)
{
	assert(pos + 2 <= CKLINK_USB_BUF_SIZE);
	ck->tx_buf[pos] = cklink_checksum(ck->tx_buf, pos);
	ck->tx_buf[pos + 1] = CKLINK_FRAME_END;
	return pos + 2;
}

static int cklink_selfreg_write(struct cklink_ctx *ck, uint8_t reg,
		const uint8_t *value)
{
	unsigned int pos = cklink_frame_start(ck, CKLINK_OP_SELFREG_WRITE);
	ck->tx_buf[pos++] = reg;
	memcpy(&ck->tx_buf[pos], value, CKLINK_SELFREG_BYTES);
	pos += CKLINK_SELFREG_BYTES;
	pos = cklink_frame_end(ck, pos);
	return cklink_usb_xfer(ck, CKLINK_OP_SELFREG_WRITE, pos, NULL);
}

static int cklink_selfreg_read(struct cklink_ctx *ck, uint8_t reg, uint8_t *value)
{
	unsigned int pos = cklink_frame_start(ck, CKLINK_OP_SELFREG_READ);
	ck->tx_buf[pos++] = reg;
	pos = cklink_frame_end(ck, pos);

	unsigned int rxlen = 0;
	int ret = cklink_usb_xfer(ck, CKLINK_OP_SELFREG_READ, pos, &rxlen);
	if (ret != ERROR_OK)
		return ret;

	const unsigned int expected = CKLINK_FRAME_OVERHEAD + CKLINK_SELFREG_BYTES;
	if (rxlen < expected) {
		LOG_ERROR("CK-Link selfreg read response too short (%u < %u)",
			rxlen, expected);
		return ERROR_FAIL;
	}
	memcpy(value, &ck->rx_buf[CKLINK_RESP_PAYLOAD_OFFSET],
			CKLINK_SELFREG_BYTES);
	return ERROR_OK;
}

/* Single-entry batch scan. Scans > 255 bits are clamped with a one-time
 * warning: chunking across entries breaks Shift-DR continuity, and
 * erroring out here breaks OpenOCD's chain probe's -expected-id recovery. */
static int cklink_jtag_scan(struct cklink_ctx *ck,
		unsigned int ir_bits, const uint8_t *ir_tdi,
		uint8_t *ir_tdo,
		unsigned int dr_bits, const uint8_t *dr_tdi,
		uint8_t *dr_tdo)
{
	if ((ir_bits > CKLINK_MAX_SCAN_BITS || dr_bits > CKLINK_MAX_SCAN_BITS)
		&& !ck->long_scan_warned) {
		LOG_WARNING("CK-Link: scan exceeds %u-bit per-entry limit (ir=%u dr=%u); clamping",
			CKLINK_MAX_SCAN_BITS, ir_bits, dr_bits);
		ck->long_scan_warned = true;
	}
	if (ir_bits > CKLINK_MAX_SCAN_BITS)
		ir_bits = CKLINK_MAX_SCAN_BITS;
	if (dr_bits > CKLINK_MAX_SCAN_BITS)
		dr_bits = CKLINK_MAX_SCAN_BITS;

	const unsigned int ir_bytes = DIV_ROUND_UP(ir_bits, 8);
	const unsigned int dr_bytes = DIV_ROUND_UP(dr_bits, 8);

	unsigned int pos = cklink_frame_start(ck, CKLINK_OP_JTAG_BATCH);
	ck->tx_buf[pos++] = 0;		// entry_count - 1
	ck->tx_buf[pos++] = (uint8_t)ir_bits;
	if (ir_tdi)
		memcpy(&ck->tx_buf[pos], ir_tdi, ir_bytes);
	else
		memset(&ck->tx_buf[pos], 0, ir_bytes);
	pos += ir_bytes;
	ck->tx_buf[pos++] = (uint8_t)dr_bits;
	if (dr_tdi)
		memcpy(&ck->tx_buf[pos], dr_tdi, dr_bytes);
	else
		memset(&ck->tx_buf[pos], 0, dr_bytes);
	pos += dr_bytes;
	pos = cklink_frame_end(ck, pos);

	unsigned int rxlen = 0;
	int ret = cklink_usb_xfer(ck, CKLINK_OP_JTAG_BATCH, pos, &rxlen);
	if (ret != ERROR_OK)
		return ret;

	// Response: start + status + ir_tdo + dr_echo + dr_tdo + csum + end.
	const unsigned int expected =
		CKLINK_FRAME_OVERHEAD + ir_bytes + 1 + dr_bytes;
	if (rxlen < expected) {
		LOG_ERROR("CK-Link scan response truncated: got %u, need %u (ir=%u dr=%u)",
			rxlen, expected, ir_bits, dr_bits);
		return ERROR_FAIL;
	}

	if (ir_tdo && ir_bytes)
		memcpy(ir_tdo, &ck->rx_buf[CKLINK_RESP_PAYLOAD_OFFSET], ir_bytes);

	const unsigned int dr_tdo_offset =
		CKLINK_RESP_PAYLOAD_OFFSET + ir_bytes + 1;
	if (dr_tdo)
		memcpy(dr_tdo, &ck->rx_buf[dr_tdo_offset], dr_bytes);

	return ERROR_OK;
}

// Shift num_bits of idle cycles, chunked to the 255-bit protocol limit.
static int cklink_idle_cycles(struct cklink_ctx *ck, unsigned int num_bits)
{
	static const uint8_t zeros[CKLINK_MAX_SCAN_BYTES] = { 0 };

	while (num_bits > 0) {
		unsigned int chunk = num_bits > CKLINK_MAX_SCAN_BITS
			? CKLINK_MAX_SCAN_BITS : num_bits;
		int ret = cklink_jtag_scan(ck, 0, NULL, NULL,
				chunk, zeros, NULL);
		if (ret != ERROR_OK)
			return ret;
		num_bits -= chunk;
	}
	return ERROR_OK;
}

/* Flush a pending IR with a 1-bit dummy DR so its in_value gets populated
 * with real captured IR TDO. Called at end of execute_queue so Capture-IR
 * validation sees real hardware data. */
static int cklink_flush_pending_ir(struct cklink_ctx *ck)
{
	if (!ck || !ck->pending_ir_cmd)
		return ERROR_OK;

	uint8_t ir_tdo[CKLINK_MAX_SCAN_BYTES] = { 0 };
	uint8_t dummy_dr = 0;
	int ret = cklink_jtag_scan(ck, ck->ir_bits, ck->ir_value, ir_tdo,
			1, &dummy_dr, NULL);
	if (ret != ERROR_OK)
		return ret;

	struct scan_command *scan = ck->pending_ir_cmd->cmd.scan;
	unsigned int off = 0;
	for (unsigned int i = 0; i < scan->num_fields; i++) {
		struct scan_field *f = &scan->fields[i];
		if (f->in_value) {
			buf_set_buf(ir_tdo, off, f->in_value, 0,
				f->num_bits);
		}
		off += f->num_bits;
	}
	ck->pending_ir_cmd = NULL;
	// ir_bits/ir_value stay valid: hardware IR is still what we cached.
	return ERROR_OK;
}

static int cklink_execute_scan(struct cklink_ctx *ck, struct jtag_command *cmd)
{
	struct scan_command *scan = cmd->cmd.scan;

	unsigned int total_bits = 0;
	for (unsigned int i = 0; i < scan->num_fields; i++)
		total_bits += scan->fields[i].num_bits;

	if (total_bits == 0)
		return ERROR_OK;

	const unsigned int total_bytes = DIV_ROUND_UP(total_bits, 8);
	if (total_bytes > ck->scan_buf_size) {
		uint8_t *ntdi = realloc(ck->scan_tdi, total_bytes);
		uint8_t *ntdo = realloc(ck->scan_tdo, total_bytes);
		if (!ntdi || !ntdo) {
			LOG_ERROR("CK-Link: out of memory for %u-bit scan",
				total_bits);
			// Partial success is fine; freed in cklink_quit.
			if (ntdi)
				ck->scan_tdi = ntdi;
			if (ntdo)
				ck->scan_tdo = ntdo;
			return ERROR_FAIL;
		}
		ck->scan_tdi = ntdi;
		ck->scan_tdo = ntdo;
		ck->scan_buf_size = total_bytes;
	}
	memset(ck->scan_tdi, 0, total_bytes);
	memset(ck->scan_tdo, 0, total_bytes);

	unsigned int bit_offset = 0;
	for (unsigned int i = 0; i < scan->num_fields; i++) {
		struct scan_field *field = &scan->fields[i];
		if (field->out_value) {
			buf_set_buf(field->out_value, 0, ck->scan_tdi,
				bit_offset, field->num_bits);
		}
		bit_offset += field->num_bits;
	}

	if (scan->ir_scan) {
		/*
		 * Defer IR until paired with a DR (batch needs both).
		 * Back-to-back IR drops the first: mid-queue flush would
		 * cycle Capture-DR/Update-DR on the previous IR's register
		 * (e.g. DMI), triggering spurious transactions.
		 */
		if (total_bytes > sizeof(ck->ir_value)) {
			LOG_ERROR("CK-Link: IR scan of %u bits exceeds cache (%zu bytes)",
				total_bits, sizeof(ck->ir_value));
			return ERROR_JTAG_QUEUE_FAILED;
		}
		if (ck->pending_ir_cmd) {
			if (!ck->dropped_ir_warned) {
				LOG_WARNING("CK-Link: back-to-back IR scan dropped pending IR; multi-TAP chains are not supported");
				ck->dropped_ir_warned = true;
			} else {
				LOG_DEBUG("CK-Link: dropping pending IR (%u bits) before new IR scan",
					ck->ir_bits);
			}
		}
		memcpy(ck->ir_value, ck->scan_tdi, total_bytes);
		ck->ir_bits = total_bits;
		ck->pending_ir_cmd = cmd;
		return ERROR_OK;
	}

	// No cached IR yet: inject IDCODE placeholder so chain detection works.
	const uint8_t placeholder_ir = CKLINK_PLACEHOLDER_IR_VALUE;
	const unsigned int eff_ir_bits = ck->ir_bits ? ck->ir_bits
			: CKLINK_PLACEHOLDER_IR_BITS;
	const uint8_t *eff_ir_value = ck->ir_bits ? ck->ir_value : &placeholder_ir;
	if (!ck->ir_bits)
		LOG_DEBUG("CK-Link: DR scan with no cached IR; injecting IDCODE placeholder");

	uint8_t ir_tdo_buf[CKLINK_MAX_SCAN_BYTES] = { 0 };
	int ret = cklink_jtag_scan(ck, eff_ir_bits, eff_ir_value, ir_tdo_buf,
			total_bits, ck->scan_tdi, ck->scan_tdo);
	if (ret != ERROR_OK)
		return ret;

	// Scatter captured IR TDO back into the deferred IR command's fields.
	if (ck->pending_ir_cmd) {
		struct scan_command *ir_scan = ck->pending_ir_cmd->cmd.scan;
		unsigned int ir_off = 0;
		for (unsigned int i = 0; i < ir_scan->num_fields; i++) {
			struct scan_field *f = &ir_scan->fields[i];
			if (f->in_value) {
				buf_set_buf(ir_tdo_buf, ir_off,
					f->in_value, 0, f->num_bits);
			}
			ir_off += f->num_bits;
		}
		ck->pending_ir_cmd = NULL;
	}

	/*
	 * For clamped scans, fill un-shifted TDO with caller's TDI (models
	 * "no more TAPs past the limit" - a bare wire). Gives OpenOCD's
	 * bypass chain-length probe the expected end-of-chain marker.
	 */
	if (total_bits > CKLINK_MAX_SCAN_BITS) {
		buf_set_buf(ck->scan_tdi, CKLINK_MAX_SCAN_BITS,
			ck->scan_tdo, CKLINK_MAX_SCAN_BITS,
			total_bits - CKLINK_MAX_SCAN_BITS);
	}

	bit_offset = 0;
	for (unsigned int i = 0; i < scan->num_fields; i++) {
		struct scan_field *field = &scan->fields[i];
		if (field->in_value) {
			buf_set_buf(ck->scan_tdo, bit_offset,
				field->in_value, 0, field->num_bits);
		}
		bit_offset += field->num_bits;
	}

	return ERROR_OK;
}

static int cklink_write_csr(struct cklink_ctx *ck, uint8_t clk, uint8_t mode)
{
	uint8_t buf[CKLINK_SELFREG_BYTES];
	h_u32_to_le(buf, CKLINK_CSR(clk, mode));
	return cklink_selfreg_write(ck, CKLINK_SR_CSR, buf);
}

// khz -> sr0 byte 0 divider, rounded up so we never exceed the request.
static uint8_t cklink_khz_to_div(int khz)
{
	if (khz <= 0)
		return CKLINK_CLK_DIV_MAX;
	int n = ((CKLINK_CLK_SOURCE_KHZ + khz - 1) / khz) - 1;
	if (n < CKLINK_CLK_DIV_MIN)
		n = CKLINK_CLK_DIV_MIN;
	if (n > CKLINK_CLK_DIV_MAX)
		n = CKLINK_CLK_DIV_MAX;
	return (uint8_t)n;
}

static int cklink_execute_tlr_reset(struct cklink_ctx *ck)
{
	// Probe-side reset via 5->2->5 mode toggle; target TAP reset not guaranteed.
	int ret;

	ret = cklink_write_csr(ck, ck->csr_clk, CKLINK_SR0_MODE_CJTAG);
	if (ret != ERROR_OK)
		return ret;
	ret = cklink_write_csr(ck, ck->csr_clk, CKLINK_SR0_MODE_CJTAG);
	if (ret != ERROR_OK)
		return ret;
	ret = cklink_write_csr(ck, ck->csr_clk, CKLINK_SR0_MODE_JTAG);
	if (ret != ERROR_OK)
		return ret;

	ck->ir_bits = 0;
	memset(ck->ir_value, 0, sizeof(ck->ir_value));
	ck->pending_ir_cmd = NULL;
	tap_set_state(TAP_RESET);
	return ERROR_OK;
}

static int cklink_execute_runtest(struct cklink_ctx *ck, struct jtag_command *cmd)
{
	return cklink_idle_cycles(ck, cmd->cmd.runtest->num_cycles);
}

static int cklink_execute_stableclocks(struct cklink_ctx *ck, struct jtag_command *cmd)
{
	// TMS=0 shifts only hold state in RTI; other stable states would advance.
	return cklink_idle_cycles(ck, cmd->cmd.stableclocks->num_cycles);
}

static int cklink_execute_tms(struct jtag_command *cmd)
{
	// No TMS-sequence opcode: honoring JTAG_TMS is impossible.
	LOG_ERROR("CK-Link: JTAG_TMS (%u bits) is not supported by this probe",
		cmd->cmd.tms->num_bits);
	return ERROR_JTAG_NOT_IMPLEMENTED;
}

static int cklink_execute_reset(struct cklink_ctx *ck, struct jtag_command *cmd)
{
	// Only TRST is meaningful here: we have no separate SRST control.
	if (cmd->cmd.reset->trst)
		return cklink_execute_tlr_reset(ck);
	return ERROR_OK;
}

static int cklink_execute_queue(struct jtag_command *cmd_queue)
{
	struct cklink_ctx *ck = cklink_handle;
	if (!ck)
		return ERROR_JTAG_INIT_FAILED;

	for (struct jtag_command *cmd = cmd_queue; cmd; cmd = cmd->next) {
		int ret;

		switch (cmd->type) {
		case JTAG_SCAN:
			ret = cklink_execute_scan(ck, cmd);
			break;
		case JTAG_TLR_RESET:
			ret = cklink_execute_tlr_reset(ck);
			break;
		case JTAG_RUNTEST:
			ret = cklink_execute_runtest(ck, cmd);
			break;
		case JTAG_RESET:
			ret = cklink_execute_reset(ck, cmd);
			break;
		case JTAG_PATHMOVE:
			// No TMS-sequence opcode: honoring PATHMOVE is impossible.
			LOG_ERROR("CK-Link: JTAG_PATHMOVE is not supported by this probe");
			ret = ERROR_JTAG_NOT_IMPLEMENTED;
			break;
		case JTAG_STABLECLOCKS:
			ret = cklink_execute_stableclocks(ck, cmd);
			break;
		case JTAG_TMS:
			ret = cklink_execute_tms(cmd);
			break;
		case JTAG_SLEEP:
			jtag_sleep(cmd->cmd.sleep->us);
			ret = ERROR_OK;
			break;
		default:
			LOG_ERROR("CK-Link: unsupported JTAG command %d",
				cmd->type);
			return ERROR_JTAG_QUEUE_FAILED;
		}
		if (ret != ERROR_OK)
			return ret;
	}
	// Flush so any un-paired IR's in_value is populated before OpenOCD reads.
	return cklink_flush_pending_ir(ck);
}

static int cklink_probe_init(struct cklink_ctx *ck)
{
	uint8_t sr1[CKLINK_SELFREG_BYTES];
	uint8_t sr8[CKLINK_SELFREG_BYTES];
	uint8_t readback[CKLINK_SELFREG_BYTES];
	int ret;

	h_u32_to_le(sr1, CKLINK_SR1_INIT_VALUE);
	h_u32_to_le(sr8, CKLINK_SR8_INIT_VALUE);

	// Configure at slow clock for cold-boot signal-integrity margin.
	ret = cklink_write_csr(ck, CKLINK_SR0_SETUP_CLK_BYTE,
			CKLINK_SR0_MODE_JTAG);
	if (ret != ERROR_OK)
		return ret;
	ret = cklink_selfreg_write(ck, CKLINK_SR_MTCR_WAIT, sr1);
	if (ret != ERROR_OK)
		return ret;
	ret = cklink_selfreg_write(ck, CKLINK_SR_JTAG_CONFIG, sr8);
	if (ret != ERROR_OK)
		return ret;
	ret = cklink_write_csr(ck, CKLINK_SR0_RUN_CLK_BYTE,
			CKLINK_SR0_MODE_JTAG);
	if (ret != ERROR_OK)
		return ret;

	ret = cklink_selfreg_read(ck, CKLINK_SR_CSR, readback);
	if (ret != ERROR_OK)
		return ret;
	if (readback[3] != CKLINK_SR0_MODE_JTAG) {
		LOG_WARNING("CK-Link: sr0 mode byte is 0x%02x, expected 0x%02x (5-wire JTAG)",
			readback[3], CKLINK_SR0_MODE_JTAG);
	}
	ck->csr_clk = CKLINK_SR0_RUN_CLK_BYTE;

	// Pre-load IR = IDCODE; 1-bit DR is the minimum the batch opcode accepts.
	const uint8_t idcode_ir = CKLINK_PLACEHOLDER_IR_VALUE;
	const uint8_t dummy_dr = 0;
	return cklink_jtag_scan(ck, CKLINK_PLACEHOLDER_IR_BITS, &idcode_ir, NULL,
			1, &dummy_dr, NULL);
}

static int cklink_init(void)
{
	struct cklink_ctx *ck = calloc(1, sizeof(*ck));
	if (!ck) {
		LOG_ERROR("CK-Link: out of memory");
		return ERROR_JTAG_INIT_FAILED;
	}

	const uint16_t *vids = adapter_usb_get_vids();
	const uint16_t *pids = adapter_usb_get_pids();
	if (!vids[0] || !pids[0]) {
		vids = cklink_default_vids;
		pids = cklink_default_pids;
	}

	const char *serial = adapter_get_required_serial();
	if (jtag_libusb_open(vids, pids, serial,
			&ck->usb_dev, NULL) != ERROR_OK) {
		LOG_ERROR("CK-Link probe not found");
		free(ck);
		return ERROR_JTAG_INIT_FAILED;
	}

	libusb_set_auto_detach_kernel_driver(ck->usb_dev, 1);

	int claim = libusb_claim_interface(ck->usb_dev, CKLINK_USB_INTERFACE);
	if (claim != 0) {
		LOG_ERROR("CK-Link: failed to claim interface %d: %s",
			CKLINK_USB_INTERFACE, libusb_error_name(claim));
		jtag_libusb_close(ck->usb_dev);
		free(ck);
		return ERROR_JTAG_INIT_FAILED;
	}

	if (cklink_probe_init(ck) != ERROR_OK) {
		LOG_ERROR("CK-Link: probe initialization failed");
		libusb_release_interface(ck->usb_dev, CKLINK_USB_INTERFACE);
		jtag_libusb_close(ck->usb_dev);
		free(ck);
		return ERROR_JTAG_INIT_FAILED;
	}

	cklink_handle = ck;
	tap_set_state(TAP_RESET);
	LOG_INFO("CK-Link Lite V2 initialized (5-wire JTAG)");
	return ERROR_OK;
}

static int cklink_quit(void)
{
	if (!cklink_handle)
		return ERROR_OK;

	libusb_release_interface(cklink_handle->usb_dev,
			CKLINK_USB_INTERFACE);
	jtag_libusb_close(cklink_handle->usb_dev);
	free(cklink_handle->scan_tdi);
	free(cklink_handle->scan_tdo);
	free(cklink_handle);
	cklink_handle = NULL;
	return ERROR_OK;
}

static int cklink_speed(int speed)
{
	struct cklink_ctx *ck = cklink_handle;
	if (!ck)
		return ERROR_OK;	// called before init

	uint8_t div = cklink_khz_to_div(speed);
	if (div == ck->csr_clk)
		return ERROR_OK;
	int ret = cklink_write_csr(ck, div, CKLINK_SR0_MODE_JTAG);
	if (ret != ERROR_OK)
		return ret;
	ck->csr_clk = div;
	int actual = CKLINK_CLK_SOURCE_KHZ / (div + 1);
	LOG_DEBUG("CK-Link: adapter speed: requested %d kHz, programmed %d kHz (div=0x%02x)",
		speed, actual, div);
	return ERROR_OK;
}

static int cklink_khz(int khz, int *jtag_speed)
{
	if (khz == 0) {
		LOG_ERROR("CK-Link: adaptive clocking (RTCK) is not supported");
		return ERROR_JTAG_NOT_IMPLEMENTED;
	}
	uint8_t div = cklink_khz_to_div(khz);
	int achievable = CKLINK_CLK_SOURCE_KHZ / (div + 1);
	if (achievable != khz) {
		static int last_clamp_khz;
		if (khz != last_clamp_khz) {
			LOG_WARNING("CK-Link: requested %d kHz clamped to %d kHz (div=0x%02x)",
				khz, achievable, div);
			last_clamp_khz = khz;
		}
	}
	*jtag_speed = achievable;
	return ERROR_OK;
}

static int cklink_speed_div(int speed, int *khz)
{
	*khz = speed;
	return ERROR_OK;
}

static struct jtag_interface cklink_jtag_interface = {
	.execute_queue = cklink_execute_queue,
};

struct adapter_driver cklink_adapter_driver = {
	.name			= "cklink",
	.transport_ids		= TRANSPORT_JTAG,
	.transport_preferred_id	= TRANSPORT_JTAG,

	.init			= cklink_init,
	.quit			= cklink_quit,
	.speed			= cklink_speed,
	.khz			= cklink_khz,
	.speed_div		= cklink_speed_div,

	.jtag_ops		= &cklink_jtag_interface,
};
