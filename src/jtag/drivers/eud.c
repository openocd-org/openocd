// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <assert.h>
#include <inttypes.h>
#include <stdint.h>

#include <helper/binarybuffer.h>
#include <helper/bits.h>
#include <helper/bitfield.h>
#include <helper/log.h>
#include <jtag/adapter.h>
#include <jtag/interface.h>
#include <jtag/swd.h>
#include <libusb.h>
#include <target/arm_adi_v5.h>

#include "libusb_helper.h"

#define EUD_VID							0x05c6
#define EUD_CTL_PID						0x9501
#define EUD_SWD_PID						0x9504
#define EUD_INTERFACE					0
#define EUD_EP_OUT						0x02
#define EUD_EP_IN						0x81
#define EUD_USB_TIMEOUT_MS				6000
#define EUD_SWD_MAX_QUEUED_TRANSFERS	64

#define EUD_CTL_CMD_CTLOUT_SET			7
#define EUD_CTL_CMD_CTLOUT_CLR			8

#define EUD_CTL_RCTLR_SRST_N		BIT(0)
#define EUD_CTL_GPIO_SRST_N			BIT(1)
#define EUD_CTL_SWD_PERIPH_EN		BIT(2)
#define EUD_CTL_DAP_TRST_N			BIT(3)
#define EUD_CTL_JTAG_PERIPH_EN		BIT(4)
#define EUD_CTL_VDDMIN_TCK_EN		BIT(6)
#define EUD_CTL_GPIO_JTAG_SEL		BIT(7)
#define EUD_CTL_DAP_MUX_SEL			BIT(10)
#define EUD_CTL_ACC_TDO_SEL			BIT(17)
#define EUD_CTL_DAP_TDO_SEL			BIT(18)
#define EUD_CTL_MTAP_TDO_SEL		BIT(19)
#define EUD_CTL_DAP_SWDO_SEL		BIT(20)

#define EUD_CTL_SWD_ON_CLEAR		(EUD_CTL_JTAG_PERIPH_EN | \
									 EUD_CTL_GPIO_JTAG_SEL | \
									 EUD_CTL_ACC_TDO_SEL | \
									 EUD_CTL_DAP_TDO_SEL | \
									 EUD_CTL_MTAP_TDO_SEL)
#define EUD_CTL_SWD_ON_SET			(EUD_CTL_RCTLR_SRST_N | \
									 EUD_CTL_SWD_PERIPH_EN | \
									 EUD_CTL_VDDMIN_TCK_EN | \
									 EUD_CTL_DAP_MUX_SEL | \
									 EUD_CTL_DAP_SWDO_SEL)
#define EUD_SWD_BB_RCTLR_SRST_N		BIT(2)
#define EUD_SWD_BB_GPIO_SRST_N		BIT(4)
#define EUD_SWD_BB_GPIO_TRST_N		BIT(5)
#define EUD_SWD_BB_DAP_TRST_N		BIT(6)
#define EUD_SWD_BB_RESET_DEFAULT	(EUD_SWD_BB_RCTLR_SRST_N | \
									 EUD_SWD_BB_GPIO_SRST_N | \
									 EUD_SWD_BB_GPIO_TRST_N | \
									 EUD_SWD_BB_DAP_TRST_N)

#define EUD_SWD_CMD_FLUSH			1
#define EUD_SWD_CMD_FREQ			2
#define EUD_SWD_CMD_DELAY			3
 // Delay between commands = (N+1) / swd_clk_freq
#define SWD_CMD_DELAY_TIME		GENMASK(7, 0)
#define SWD_CMD_DELAY_RESERVED		GENMASK(31, 8)
#define EUD_SWD_CMD_BITBANG			4
#define EUD_SWD_CMD_DITMS			5
#define SWD_CMD_DITMS_DATA		GENMASK(15, 0)
#define SWD_CMD_DITMS_DATA_BITS		16
#define SWD_CMD_DITMS_COUNT		GENMASK(31, 16)
#define EUD_SWD_DITMS_MAX_BITS		(FIELD_GET(SWD_CMD_DITMS_COUNT, SWD_CMD_DITMS_COUNT) + 1)
#define EUD_SWD_CMD_TIMING			6
 // The number of retries the SWD Peripheral performs due to a WAIT response
 // from the DAP
#define SWD_CMD_TIMING_RETRY_COUNT	GENMASK(15, 0)
 // The number of SWD clock cycles allowed for the turn-around time minus 1
#define SWD_CMD_TIMING_TURNAROUND_TIME	GENMASK(17, 16)
 // If retry_count == 0 and SWD R/W is delayed by ACK.WAIT, wait 35 cycles
 // before issuing another SWD R/W
#define SWD_CMD_TIMING_WAIT_35_IF_ERR	BIT(18)
#define SWD_CMD_TIMING_RESERVED	GENMASK(31, 19)
#define EUD_SWD_CMD_STATUS			7
#define EUD_SWD_CMD_PERIPH_RST		8

struct eud_device {
	libusb_device_handle *handle;
	uint16_t pid;
	bool claimed;
};

struct eud_swd_transfer {
	uint8_t cmd;
	uint32_t value;
	uint32_t *read_value;
	uint32_t ap_delay_clk;
};

static libusb_context *eud_libusb_ctx;
static struct eud_device eud_ctl;
static struct eud_device eud_swd;
static struct eud_swd_transfer eud_queue[EUD_SWD_MAX_QUEUED_TRANSFERS];
static unsigned int eud_queue_len;
static int eud_queued_retval = ERROR_OK;
static int eud_speed_index = 0xa;

static const unsigned int eud_speed_khz[] = {
	120000,
	80000,
	60000,
	40000,
	30000,
	15000,
	7500,
	3750,
	1875,
	938,
	469,
	234,
	117,
};

static void eud_close(struct eud_device *dev)
{
	if (!dev->handle)
		return;

	if (dev->claimed)
		libusb_release_interface(dev->handle, EUD_INTERFACE);

	libusb_close(dev->handle);
	dev->handle = NULL;
	dev->pid = 0;
	dev->claimed = false;
}

static int eud_libusb_reinit(void)
{
	if (eud_libusb_ctx)
		libusb_exit(eud_libusb_ctx);

	int ret = libusb_init(&eud_libusb_ctx);
	if (ret != LIBUSB_SUCCESS) {
		LOG_ERROR("libusb_init() failed: %s", libusb_error_name(ret));
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int eud_open_inner(uint16_t pid, struct eud_device *dev, bool quiet)
{
	dev->handle = libusb_open_device_with_vid_pid(eud_libusb_ctx, EUD_VID, pid);
	if (!dev->handle) {
		if (!quiet)
			LOG_ERROR("unable to open EUD USB function %04x:%04x", EUD_VID, pid);
		return ERROR_FAIL;
	}

	int ret = libusb_claim_interface(dev->handle, EUD_INTERFACE);
	if (ret != LIBUSB_SUCCESS) {
		if (!quiet)
			LOG_ERROR("libusb_claim_interface() failed: %s", libusb_error_name(ret));
		eud_close(dev);
		return jtag_libusb_error(ret);
	}

	dev->pid = pid;
	dev->claimed = true;
	return ERROR_OK;
}

static int eud_open(uint16_t pid, struct eud_device *dev)
{
	return eud_open_inner(pid, dev, false);
}

static int eud_bulk_write(struct eud_device *dev, const uint8_t *buf, int len)
{
	LOG_DEBUG_IO("EUD pid 0x%04x write len %d opcode 0x%02x", dev->pid, len, buf[0]);
	int transferred;
	int ret = libusb_bulk_transfer(dev->handle, EUD_EP_OUT, (unsigned char *)buf,
								 len, &transferred, EUD_USB_TIMEOUT_MS);
	if (ret != LIBUSB_SUCCESS) {
		LOG_ERROR("EUD bulk write failed: %s", libusb_error_name(ret));
		return jtag_libusb_error(ret);
	}

	if (transferred != len) {
		LOG_ERROR("short EUD bulk write: %d of %d bytes", transferred, len);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int eud_bulk_read(struct eud_device *dev, uint8_t *buf, int len)
{
	int transferred;
	int ret = libusb_bulk_transfer(dev->handle, EUD_EP_IN, buf, len,
								 &transferred, EUD_USB_TIMEOUT_MS);
	if (ret != LIBUSB_SUCCESS) {
		LOG_ERROR("EUD bulk read failed: %s", libusb_error_name(ret));
		return jtag_libusb_error(ret);
	}

	if (transferred != len) {
		LOG_ERROR("short EUD bulk read: %d of %d bytes", transferred, len);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int eud_op(struct eud_device *dev, uint8_t opcode, uint32_t payload)
{
	uint8_t buf[5];

	buf[0] = opcode;
	h_u32_to_le(&buf[1], payload);

	return eud_bulk_write(dev, buf, sizeof(buf));
}

static int eud_op_no_payload(struct eud_device *dev, uint8_t opcode)
{
	return eud_bulk_write(dev, &opcode, sizeof(opcode));
}

static int eud_ctl_enable_swd(void)
{
	int retval = eud_open(EUD_CTL_PID, &eud_ctl);
	if (retval != ERROR_OK)
		return retval;

	retval = eud_op(&eud_ctl, EUD_CTL_CMD_CTLOUT_CLR, EUD_CTL_SWD_ON_CLEAR);
	if (retval != ERROR_OK)
		goto out;

	retval = eud_op(&eud_ctl, EUD_CTL_CMD_CTLOUT_SET, EUD_CTL_SWD_ON_SET);

out:
	eud_close(&eud_ctl);
	return retval;
}

static int eud_open_swd_after_enable(void)
{
	int retval = eud_ctl_enable_swd();
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int i = 0; i < 5000; i++) {
		alive_sleep(1);

		retval = eud_libusb_reinit();
		if (retval != ERROR_OK)
			return retval;

		retval = eud_open_inner(EUD_SWD_PID, &eud_swd, true);
		if (retval == ERROR_OK)
			return ERROR_OK;
	}

	LOG_ERROR("unable to open EUD USB function %04x:%04x", EUD_VID, EUD_SWD_PID);
	return ERROR_FAIL;
}

static int eud_reopen_swd(void)
{
	eud_close(&eud_swd);
	return eud_open_swd_after_enable();
}

static int eud_swd_set_speed_index(int speed_index)
{
	if (speed_index < 0 || speed_index >= (int)ARRAY_SIZE(eud_speed_khz)) {
		LOG_ERROR("EUD speed index %d is out of range", speed_index);
		return ERROR_FAIL;
	}

	int retval = eud_op(&eud_swd, EUD_SWD_CMD_FREQ, speed_index);
	if (retval != ERROR_OK)
		return retval;

	eud_speed_index = speed_index;
	return ERROR_OK;
}

static int eud_swd_init_defaults(void)
{
	int retval = eud_op_no_payload(&eud_swd, EUD_SWD_CMD_PERIPH_RST);
	if (retval != ERROR_OK)
		return retval;

	retval = eud_swd_set_speed_index(eud_speed_index);
	if (retval != ERROR_OK)
		return retval;

	retval = eud_op(&eud_swd, EUD_SWD_CMD_DELAY,
					FIELD_PREP(SWD_CMD_DELAY_TIME, 18));
	if (retval != ERROR_OK)
		return retval;

	return eud_op(&eud_swd, EUD_SWD_CMD_TIMING,
				  FIELD_PREP(SWD_CMD_TIMING_RETRY_COUNT, 4095));
}

static int eud_swd_bitbang(uint32_t payload)
{
	uint8_t out[6];
	uint8_t in[4];

	out[0] = EUD_SWD_CMD_BITBANG;
	h_u32_to_le(&out[1], payload);
	out[5] = EUD_SWD_CMD_FLUSH;

	int retval = eud_bulk_write(&eud_swd, out, sizeof(out));
	if (retval != ERROR_OK)
		return retval;

	return eud_bulk_read(&eud_swd, in, sizeof(in));
}

static int eud_ditms_u16(uint16_t tms, uint16_t count)
{
	int retval = eud_op(&eud_swd, EUD_SWD_CMD_DITMS,
			FIELD_PREP(SWD_CMD_DITMS_DATA, tms) |
			FIELD_PREP(SWD_CMD_DITMS_COUNT, count));
	if (retval == ERROR_OK)
		alive_sleep(1);
	return retval;
}

static int eud_swd_prepare(void)
{
	return eud_swd_init_defaults();
}

static int eud_ditms_sequence(const uint8_t *seq, unsigned int bit_len)
{
	int retval = ERROR_OK;
	unsigned int bit_pos = 0;

	// EUD can issue (N + 1) bits with a single CMD_DITMS
	// However, for N > 15 (i.e. 'send more than 16 bits') it repeats the payload[15]
	// value (N - 16) times
	while (bit_pos < bit_len) {
		unsigned int cmd_len = bit_len - bit_pos;
		if (cmd_len > SWD_CMD_DITMS_DATA_BITS) {
			bool repeated_bit = buf_get_u32(seq, bit_pos + SWD_CMD_DITMS_DATA_BITS - 1, 1);

			cmd_len = SWD_CMD_DITMS_DATA_BITS;
			while (cmd_len < EUD_SWD_DITMS_MAX_BITS && bit_pos + cmd_len < bit_len) {
				if (buf_get_u32(seq, bit_pos + cmd_len, 1) != repeated_bit)
					break;

				cmd_len++;
			}
		}

		unsigned int payload_bits = MIN(cmd_len, SWD_CMD_DITMS_DATA_BITS);
		uint16_t tms = buf_get_u32(seq, bit_pos, payload_bits);

		retval = eud_ditms_u16(tms, cmd_len - 1);
		if (retval != ERROR_OK)
			break;

		bit_pos += cmd_len;
	}

	return retval;
}

static int eud_swd_switch_seq(enum swd_special_seq seq)
{
	switch (seq) {
	case LINE_RESET:
		LOG_DEBUG("SWD line reset");
		return eud_ditms_sequence(swd_seq_line_reset, swd_seq_line_reset_len);
	case JTAG_TO_SWD:
		LOG_DEBUG("JTAG-to-SWD");
		return eud_ditms_sequence(swd_seq_jtag_to_swd, swd_seq_jtag_to_swd_len);
	case JTAG_TO_DORMANT:
		LOG_DEBUG("JTAG-to-DORMANT");
		return eud_ditms_sequence(swd_seq_jtag_to_dormant, swd_seq_jtag_to_dormant_len);
	case SWD_TO_JTAG:
		LOG_DEBUG("SWD-to-JTAG ignored");
		return ERROR_OK;
	case SWD_TO_DORMANT:
		LOG_DEBUG("SWD-to-DORMANT");
		return eud_ditms_sequence(swd_seq_swd_to_dormant, swd_seq_swd_to_dormant_len);
	case DORMANT_TO_SWD:
		LOG_DEBUG("DORMANT-to-SWD");
		return eud_ditms_sequence(swd_seq_dormant_to_swd, swd_seq_dormant_to_swd_len);
	case DORMANT_TO_JTAG:
		LOG_DEBUG("DORMANT-to-JTAG");
		return eud_ditms_sequence(swd_seq_dormant_to_jtag, swd_seq_dormant_to_jtag_len);
	default:
		LOG_ERROR("Sequence %d not supported", seq);
		return ERROR_FAIL;
	}
}

static int eud_swd_init(void)
{
	if (!eud_swd.handle)
		return ERROR_OK;

	int retval = eud_swd_prepare();
	if (retval == ERROR_OK)
		return ERROR_OK;

	retval = eud_reopen_swd();
	if (retval != ERROR_OK)
		return retval;

	return eud_swd_prepare();
}

static void eud_swd_read_reg(uint8_t cmd, uint32_t *value, uint32_t ap_delay_clk)
{
	assert(cmd & SWD_CMD_RNW);

	if (eud_queue_len >= ARRAY_SIZE(eud_queue)) {
		eud_queued_retval = ERROR_FAIL;
		return;
	}

	eud_queue[eud_queue_len++] = (struct eud_swd_transfer) {
		.cmd = cmd,
		.read_value = value,
		.ap_delay_clk = ap_delay_clk,
	};
}

static void eud_swd_write_reg(uint8_t cmd, uint32_t value, uint32_t ap_delay_clk)
{
	assert(!(cmd & SWD_CMD_RNW));

	if (eud_queue_len >= ARRAY_SIZE(eud_queue)) {
		eud_queued_retval = ERROR_FAIL;
		return;
	}

	eud_queue[eud_queue_len++] = (struct eud_swd_transfer) {
		.cmd = cmd,
		.value = value,
		.ap_delay_clk = ap_delay_clk,
	};
}

static int eud_swd_run_queue(void)
{
	int retval = eud_queued_retval;

	if (retval != ERROR_OK)
		goto out;

	for (unsigned int i = 0; i < eud_queue_len; i++) {
		struct eud_swd_transfer *transfer = &eud_queue[i];
		uint8_t out[8];
		uint8_t in[8];
		uint8_t eud_cmd = (transfer->cmd | SWD_CMD_START | SWD_CMD_PARK) & ~SWD_CMD_STOP;
		int out_len;
		int in_len;

		LOG_DEBUG_IO("%s %s reg %x %" PRIx32,
					 transfer->cmd & SWD_CMD_APNDP ? "AP" : "DP",
					 transfer->cmd & SWD_CMD_RNW ? "read" : "write",
					 (transfer->cmd & SWD_CMD_A32) >> 1,
					 transfer->value);

		out[0] = eud_cmd;
		if (transfer->cmd & SWD_CMD_RNW) {
			out[1] = EUD_SWD_CMD_STATUS;
			out[2] = EUD_SWD_CMD_FLUSH;
			out_len = 3;
			in_len = 8;
		} else {
			h_u32_to_le(&out[1], transfer->value);
			out[5] = EUD_SWD_CMD_STATUS;
			out[6] = EUD_SWD_CMD_FLUSH;
			out_len = 7;
			in_len = 4;
		}

		retval = eud_bulk_write(&eud_swd, out, out_len);
		if (retval != ERROR_OK)
			goto out;

		retval = eud_bulk_read(&eud_swd, in, in_len);
		if (retval != ERROR_OK)
			goto out;

		uint32_t status;
		if (transfer->cmd & SWD_CMD_RNW) {
			uint32_t value = le_to_h_u32(in);
			status = le_to_h_u32(&in[4]);
			if (transfer->read_value)
				*transfer->read_value = value;
			LOG_DEBUG_IO("read result: %" PRIx32, value);
		} else {
			status = le_to_h_u32(in);
		}

		uint8_t ack = status & 0x07;
		if (swd_cmd_returns_ack(transfer->cmd) && ack != SWD_ACK_OK && ack != 0) {
			LOG_DEBUG("SWD ack not OK: %u status 0x%08" PRIx32, i, status);
			retval = swd_ack_to_error_code(ack);
			goto out;
		}

		if (transfer->ap_delay_clk)
			retval = eud_ditms_u16(0, transfer->ap_delay_clk > 16 ? 16 : transfer->ap_delay_clk);

		if (retval != ERROR_OK)
			goto out;
	}

out:
	eud_queue_len = 0;
	eud_queued_retval = ERROR_OK;
	return retval;
}

static int eud_init(void)
{
	int ret = libusb_init(&eud_libusb_ctx);
	if (ret != LIBUSB_SUCCESS) {
		LOG_ERROR("libusb_init() failed: %s", libusb_error_name(ret));
		return ERROR_FAIL;
	}

	int retval = eud_open_inner(EUD_SWD_PID, &eud_swd, true);
	if (retval != ERROR_OK) {
		retval = eud_open_swd_after_enable();
		if (retval != ERROR_OK)
			goto fail;
	}

	retval = eud_swd_prepare();
	if (retval != ERROR_OK) {
		retval = eud_reopen_swd();
		if (retval != ERROR_OK)
			goto fail;

		retval = eud_swd_prepare();
		if (retval != ERROR_OK)
			goto fail;
	}

	return ERROR_OK;

fail:
	eud_close(&eud_swd);
	eud_close(&eud_ctl);
	libusb_exit(eud_libusb_ctx);
	eud_libusb_ctx = NULL;
	return retval;
}

static int eud_quit(void)
{
	eud_close(&eud_swd);
	eud_close(&eud_ctl);

	if (eud_libusb_ctx) {
		libusb_exit(eud_libusb_ctx);
		eud_libusb_ctx = NULL;
	}

	return ERROR_OK;
}

static int eud_reset(int trst, int srst)
{
	uint32_t value = EUD_SWD_BB_RESET_DEFAULT;

	if (srst)
		value &= ~(EUD_SWD_BB_RCTLR_SRST_N | EUD_SWD_BB_GPIO_SRST_N);
	if (trst)
		value &= ~EUD_SWD_BB_DAP_TRST_N;

	return eud_swd_bitbang(value);
}

static int eud_speed(int speed)
{
	if (speed < 0 || speed >= (int)ARRAY_SIZE(eud_speed_khz)) {
		LOG_ERROR("EUD speed index %d is out of range", speed);
		return ERROR_FAIL;
	}

	eud_speed_index = speed;

	if (!eud_swd.handle)
		return ERROR_OK;

	return eud_swd_set_speed_index(speed);
}

static int eud_khz(int khz, int *jtag_speed)
{
	int selected = ARRAY_SIZE(eud_speed_khz) - 1;

	for (unsigned int i = 0; i < ARRAY_SIZE(eud_speed_khz); i++) {
		if ((unsigned int)khz >= eud_speed_khz[i]) {
			selected = i;
			break;
		}
	}

	*jtag_speed = selected;
	return ERROR_OK;
}

static int eud_speed_div(int speed, int *khz)
{
	if (speed < 0 || speed >= (int)ARRAY_SIZE(eud_speed_khz))
		return ERROR_FAIL;

	*khz = eud_speed_khz[speed];
	return ERROR_OK;
}

static const struct swd_driver eud_swd_driver = {
	.init = eud_swd_init,
	.switch_seq = eud_swd_switch_seq,
	.read_reg = eud_swd_read_reg,
	.write_reg = eud_swd_write_reg,
	.run = eud_swd_run_queue,
};

struct adapter_driver eud_adapter_driver = {
	.name = "eud",
	.transport_ids = TRANSPORT_SWD,
	.transport_preferred_id = TRANSPORT_SWD,

	.init = eud_init,
	.quit = eud_quit,
	.reset = eud_reset,
	.speed = eud_speed,
	.khz = eud_khz,
	.speed_div = eud_speed_div,

	.swd_ops = &eud_swd_driver,
};
