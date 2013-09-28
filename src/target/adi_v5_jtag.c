/***************************************************************************
 *   Copyright (C) 2006 by Magnus Lundin
 *   lundin@mlu.mine.nu
 *
 *   Copyright (C) 2008 by Spencer Oliver
 *   spen@spen-soft.co.uk
 *
 *   Copyright (C) 2009 by Oyvind Harboe
 *   oyvind.harboe@zylin.com
 *
 *   Copyright (C) 2009-2010 by David Brownell
 *
 *   This program is free software; you can redistribute it and/or modify
 *   it under the terms of the GNU General Public License as published by
 *   the Free Software Foundation; either version 2 of the License, or
 *   (at your option) any later version.
 *
 *   This program is distributed in the hope that it will be useful,
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *   GNU General Public License for more details.
 *
 *   You should have received a copy of the GNU General Public License
 *   along with this program; if not, write to the
 *   Free Software Foundation, Inc.,
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.
 ***************************************************************************/

/**
 * @file
 * This file implements JTAG transport support for cores implementing
 the ARM Debug Interface version 5 (ADIv5).
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arm.h"
#include "arm_adi_v5.h"
#include <helper/time_support.h>

/* JTAG instructions/registers for JTAG-DP and SWJ-DP */
#define JTAG_DP_ABORT		0x8
#define JTAG_DP_DPACC		0xA
#define JTAG_DP_APACC		0xB
#define JTAG_DP_IDCODE		0xE

/* three-bit ACK values for DPACC and APACC reads */
#define JTAG_ACK_OK_FAULT	0x2
#define JTAG_ACK_WAIT		0x1

static int jtag_ap_q_abort(struct adiv5_dap *dap, uint8_t *ack);

/***************************************************************************
 *
 * DPACC and APACC scanchain access through JTAG-DP (or SWJ-DP)
 *
***************************************************************************/

/**
 * Scan DPACC or APACC using target ordered uint8_t buffers.  No endianness
 * conversions are performed.  See section 4.4.3 of the ADIv5 spec, which
 * discusses operations which access these registers.
 *
 * Note that only one scan is performed.  If RnW is set, a separate scan
 * will be needed to collect the data which was read; the "invalue" collects
 * the posted result of a preceding operation, not the current one.
 *
 * @param dap the DAP
 * @param instr JTAG_DP_APACC (AP access) or JTAG_DP_DPACC (DP access)
 * @param reg_addr two significant bits; A[3:2]; for APACC access, the
 *	SELECT register has more addressing bits.
 * @param RnW false iff outvalue will be written to the DP or AP
 * @param outvalue points to a 32-bit (little-endian) integer
 * @param invalue NULL, or points to a 32-bit (little-endian) integer
 * @param ack points to where the three bit JTAG_ACK_* code will be stored
 */

static int adi_jtag_dp_scan(struct adiv5_dap *dap,
		uint8_t instr, uint8_t reg_addr, uint8_t RnW,
		uint8_t *outvalue, uint8_t *invalue, uint8_t *ack)
{
	struct arm_jtag *jtag_info = dap->jtag_info;
	struct scan_field fields[2];
	uint8_t out_addr_buf;
	int retval;

	retval = arm_jtag_set_instr(jtag_info, instr, NULL, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	/* Scan out a read or write operation using some DP or AP register.
	 * For APACC access with any sticky error flag set, this is discarded.
	 */
	fields[0].num_bits = 3;
	buf_set_u32(&out_addr_buf, 0, 3, ((reg_addr >> 1) & 0x6) | (RnW & 0x1));
	fields[0].out_value = &out_addr_buf;
	fields[0].in_value = ack;

	/* NOTE: if we receive JTAG_ACK_WAIT, the previous operation did not
	 * complete; data we write is discarded, data we read is unpredictable.
	 * When overrun detect is active, STICKYORUN is set.
	 */

	fields[1].num_bits = 32;
	fields[1].out_value = outvalue;
	fields[1].in_value = invalue;

	jtag_add_dr_scan(jtag_info->tap, 2, fields, TAP_IDLE);

	/* Add specified number of tck clocks after starting memory bus
	 * access, giving the hardware time to complete the access.
	 * They provide more time for the (MEM) AP to complete the read ...
	 * See "Minimum Response Time" for JTAG-DP, in the ADIv5 spec.
	 */
	if ((instr == JTAG_DP_APACC)
			&& ((reg_addr == AP_REG_DRW)
				|| ((reg_addr & 0xF0) == AP_REG_BD0))
			&& (dap->memaccess_tck != 0))
		jtag_add_runtest(dap->memaccess_tck,
				TAP_IDLE);

	return ERROR_OK;
}

/**
 * Scan DPACC or APACC out and in from host ordered uint32_t buffers.
 * This is exactly like adi_jtag_dp_scan(), except that endianness
 * conversions are performed (so the types of invalue and outvalue
 * must be different).
 */
static int adi_jtag_dp_scan_u32(struct adiv5_dap *dap,
		uint8_t instr, uint8_t reg_addr, uint8_t RnW,
		uint32_t outvalue, uint32_t *invalue, uint8_t *ack)
{
	uint8_t out_value_buf[4];
	int retval;

	buf_set_u32(out_value_buf, 0, 32, outvalue);

	retval = adi_jtag_dp_scan(dap, instr, reg_addr, RnW,
			out_value_buf, (uint8_t *)invalue, ack);
	if (retval != ERROR_OK)
		return retval;

	if (invalue)
		jtag_add_callback(arm_le_to_h_u32,
				(jtag_callback_data_t) invalue);

	return retval;
}

/**
 * Utility to write AP registers.
 */
static inline int adi_jtag_ap_write_check(struct adiv5_dap *dap,
		uint8_t reg_addr, uint8_t *outvalue)
{
	return adi_jtag_dp_scan(dap, JTAG_DP_APACC, reg_addr, DPAP_WRITE,
			outvalue, NULL, NULL);
}

static int adi_jtag_scan_inout_check_u32(struct adiv5_dap *dap,
		uint8_t instr, uint8_t reg_addr, uint8_t RnW,
		uint32_t outvalue, uint32_t *invalue)
{
	int retval;

	/* Issue the read or write */
	retval = adi_jtag_dp_scan_u32(dap, instr, reg_addr,
			RnW, outvalue, NULL, NULL);
	if (retval != ERROR_OK)
		return retval;

	/* For reads,  collect posted value; RDBUFF has no other effect.
	 * Assumes read gets acked with OK/FAULT, and CTRL_STAT says "OK".
	 */
	if ((RnW == DPAP_READ) && (invalue != NULL))
		retval = adi_jtag_dp_scan_u32(dap, JTAG_DP_DPACC,
				DP_RDBUFF, DPAP_READ, 0, invalue, &dap->ack);
	return retval;
}

static int jtagdp_transaction_endcheck(struct adiv5_dap *dap)
{
	int retval;
	uint32_t ctrlstat;

	/* too expensive to call keep_alive() here */

	/* Here be dragons!
	 *
	 * It is easy to be in a JTAG clock range where the target
	 * is not operating in a stable fashion. This happens
	 * for a few reasons:
	 *
	 * - the user may construct a simple test case to try to see
	 * if a higher JTAG clock works to eke out more performance.
	 * This simple case may pass, but more complex situations can
	 * fail.
	 *
	 * - The mostly works JTAG clock rate and the complete failure
	 * JTAG clock rate may be as much as 2-4x apart. This seems
	 * to be especially true on RC oscillator driven parts.
	 *
	 * So: even if calling adi_jtag_scan_inout_check_u32() multiple
	 * times here seems to "make things better here", it is just
	 * hiding problems with too high a JTAG clock.
	 *
	 * Note that even if some parts have RCLK/RTCK, that doesn't
	 * mean that RCLK/RTCK is the *correct* rate to run the JTAG
	 * interface at, i.e. RCLK/RTCK rates can be "too high", especially
	 * before the RC oscillator phase is not yet complete.
	 */

	/* Post CTRL/STAT read; discard any previous posted read value
	 * but collect its ACK status.
	 */
	retval = adi_jtag_scan_inout_check_u32(dap, JTAG_DP_DPACC,
			DP_CTRL_STAT, DPAP_READ, 0, &ctrlstat);
	if (retval != ERROR_OK)
		return retval;
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	dap->ack = dap->ack & 0x7;

	/* common code path avoids calling timeval_ms() */
	if (dap->ack != JTAG_ACK_OK_FAULT) {
		long long then = timeval_ms();

		while (dap->ack != JTAG_ACK_OK_FAULT) {
			if (dap->ack == JTAG_ACK_WAIT) {
				if ((timeval_ms()-then) > 1000) {
					LOG_WARNING("Timeout (1000ms) waiting "
						"for ACK=OK/FAULT "
						"in JTAG-DP transaction - aborting");

					uint8_t ack;
					int abort_ret = jtag_ap_q_abort(dap, &ack);

					if (abort_ret != 0)
						LOG_WARNING("Abort failed : return=%d ack=%d", abort_ret, ack);

					return ERROR_JTAG_DEVICE_ERROR;
				}
			} else {
				LOG_WARNING("Invalid ACK %#x "
						"in JTAG-DP transaction",
						dap->ack);
				return ERROR_JTAG_DEVICE_ERROR;
			}

			retval = adi_jtag_scan_inout_check_u32(dap, JTAG_DP_DPACC,
					DP_CTRL_STAT, DPAP_READ, 0, &ctrlstat);
			if (retval != ERROR_OK)
				return retval;
			retval = jtag_execute_queue();
			if (retval != ERROR_OK)
				return retval;
			dap->ack = dap->ack & 0x7;
		}
	}

	/* REVISIT also STICKYCMP, for pushed comparisons (nyet used) */

	/* Check for STICKYERR and STICKYORUN */
	if (ctrlstat & (SSTICKYORUN | SSTICKYERR)) {
		LOG_DEBUG("jtag-dp: CTRL/STAT error, 0x%" PRIx32, ctrlstat);
		/* Check power to debug regions */
		if ((ctrlstat & 0xf0000000) != 0xf0000000) {
			retval = ahbap_debugport_init(dap);
			if (retval != ERROR_OK)
				return retval;
		} else {
			uint32_t mem_ap_csw, mem_ap_tar;

			/* Maybe print information about last intended
			 * MEM-AP access; but not if autoincrementing.
			 * *Real* CSW and TAR values are always shown.
			 */
			if (dap->ap_tar_value != (uint32_t) -1)
				LOG_DEBUG("MEM-AP Cached values: "
					"ap_bank 0x%" PRIx32
					", ap_csw 0x%" PRIx32
					", ap_tar 0x%" PRIx32,
					dap->ap_bank_value,
					dap->ap_csw_value,
					dap->ap_tar_value);

			if (ctrlstat & SSTICKYORUN)
				LOG_ERROR("JTAG-DP OVERRUN - check clock, "
					"memaccess, or reduce jtag speed");

			if (ctrlstat & SSTICKYERR)
				LOG_ERROR("JTAG-DP STICKY ERROR");

			/* Clear Sticky Error Bits */
			retval = adi_jtag_scan_inout_check_u32(dap, JTAG_DP_DPACC,
					DP_CTRL_STAT, DPAP_WRITE,
					dap->dp_ctrl_stat | SSTICKYORUN
						| SSTICKYERR, NULL);
			if (retval != ERROR_OK)
				return retval;
			retval = adi_jtag_scan_inout_check_u32(dap, JTAG_DP_DPACC,
					DP_CTRL_STAT, DPAP_READ, 0, &ctrlstat);
			if (retval != ERROR_OK)
				return retval;
			retval = jtag_execute_queue();
			if (retval != ERROR_OK)
				return retval;

			LOG_DEBUG("jtag-dp: CTRL/STAT 0x%" PRIx32, ctrlstat);

			retval = dap_queue_ap_read(dap,
					AP_REG_CSW, &mem_ap_csw);
			if (retval != ERROR_OK)
				return retval;

			retval = dap_queue_ap_read(dap,
					AP_REG_TAR, &mem_ap_tar);
			if (retval != ERROR_OK)
				return retval;

			retval = jtag_execute_queue();
			if (retval != ERROR_OK)
				return retval;
			LOG_ERROR("MEM_AP_CSW 0x%" PRIx32 ", MEM_AP_TAR 0x%"
					PRIx32, mem_ap_csw, mem_ap_tar);

		}
		retval = jtag_execute_queue();
		if (retval != ERROR_OK)
			return retval;
		return ERROR_JTAG_DEVICE_ERROR;
	}

	return ERROR_OK;
}

/*--------------------------------------------------------------------------*/

static int jtag_idcode_q_read(struct adiv5_dap *dap,
		uint8_t *ack, uint32_t *data)
{
	struct arm_jtag *jtag_info = dap->jtag_info;
	int retval;
	struct scan_field fields[1];

	/* This is a standard JTAG operation -- no DAP tweakage */
	retval = arm_jtag_set_instr(jtag_info, JTAG_DP_IDCODE, NULL, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;

	fields[0].num_bits = 32;
	fields[0].out_value = NULL;
	fields[0].in_value = (uint8_t *) data;

	jtag_add_dr_scan(jtag_info->tap, 1, fields, TAP_IDLE);

	jtag_add_callback(arm_le_to_h_u32,
			(jtag_callback_data_t) data);

	return ERROR_OK;
}

static int jtag_dp_q_read(struct adiv5_dap *dap, unsigned reg,
		uint32_t *data)
{
	return adi_jtag_scan_inout_check_u32(dap, JTAG_DP_DPACC,
			reg, DPAP_READ, 0, data);
}

static int jtag_dp_q_write(struct adiv5_dap *dap, unsigned reg,
		uint32_t data)
{
	return adi_jtag_scan_inout_check_u32(dap, JTAG_DP_DPACC,
			reg, DPAP_WRITE, data, NULL);
}

/** Select the AP register bank matching bits 7:4 of reg. */
static int jtag_ap_q_bankselect(struct adiv5_dap *dap, unsigned reg)
{
	uint32_t select_ap_bank = reg & 0x000000F0;

	if (select_ap_bank == dap->ap_bank_value)
		return ERROR_OK;
	dap->ap_bank_value = select_ap_bank;

	select_ap_bank |= dap->ap_current;

	return jtag_dp_q_write(dap, DP_SELECT, select_ap_bank);
}

static int jtag_ap_q_read(struct adiv5_dap *dap, unsigned reg,
		uint32_t *data)
{
	int retval = jtag_ap_q_bankselect(dap, reg);

	if (retval != ERROR_OK)
		return retval;

	return adi_jtag_scan_inout_check_u32(dap, JTAG_DP_APACC, reg,
			DPAP_READ, 0, data);
}

static int jtag_ap_q_write(struct adiv5_dap *dap, unsigned reg,
		uint32_t data)
{
	uint8_t out_value_buf[4];

	int retval = jtag_ap_q_bankselect(dap, reg);
	if (retval != ERROR_OK)
		return retval;

	buf_set_u32(out_value_buf, 0, 32, data);

	return adi_jtag_ap_write_check(dap, reg, out_value_buf);
}

static int jtag_ap_q_read_block(struct adiv5_dap *dap, unsigned reg,
		uint32_t blocksize, uint8_t *buffer)
{
	uint32_t readcount;
	int retval = ERROR_OK;

	/* Scan out first read */
	retval = adi_jtag_dp_scan(dap, JTAG_DP_APACC, reg,
			DPAP_READ, 0, NULL, NULL);
	if (retval != ERROR_OK)
		return retval;

	for (readcount = 0; readcount < blocksize - 1; readcount++) {
		/* Scan out next read; scan in posted value for the
		 * previous one.  Assumes read is acked "OK/FAULT",
		 * and CTRL_STAT says that meant "OK".
		 */
		retval = adi_jtag_dp_scan(dap, JTAG_DP_APACC, reg,
				DPAP_READ, 0, buffer + 4 * readcount,
				&dap->ack);
		if (retval != ERROR_OK)
			return retval;
	}

	/* Scan in last posted value; RDBUFF has no other effect,
	 * assuming ack is OK/FAULT and CTRL_STAT says "OK".
	 */
	retval = adi_jtag_dp_scan(dap, JTAG_DP_DPACC, DP_RDBUFF,
			DPAP_READ, 0, buffer + 4 * readcount,
			&dap->ack);

	return retval;
}

static int jtag_ap_q_abort(struct adiv5_dap *dap, uint8_t *ack)
{
	/* for JTAG, this is the only valid ABORT register operation */
	return adi_jtag_dp_scan_u32(dap, JTAG_DP_ABORT,
			0, DPAP_WRITE, 1, NULL, ack);
}

static int jtag_dp_run(struct adiv5_dap *dap)
{
	return jtagdp_transaction_endcheck(dap);
}

/* FIXME don't export ... just initialize as
 * part of DAP setup
*/
const struct dap_ops jtag_dp_ops = {
	.queue_idcode_read   = jtag_idcode_q_read,
	.queue_dp_read       = jtag_dp_q_read,
	.queue_dp_write      = jtag_dp_q_write,
	.queue_ap_read       = jtag_ap_q_read,
	.queue_ap_write      = jtag_ap_q_write,
	.queue_ap_read_block = jtag_ap_q_read_block,
	.queue_ap_abort      = jtag_ap_q_abort,
	.run                 = jtag_dp_run,
};


static const uint8_t swd2jtag_bitseq[] = {
	/* More than 50 TCK/SWCLK cycles with TMS/SWDIO high,
	 * putting both JTAG and SWD logic into reset state.
	 */
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	/* Switching equence disables SWD and enables JTAG
	 * NOTE: bits in the DP's IDCODE can expose the need for
	 * the old/deprecated sequence (0xae 0xde).
	 */
	0x3c, 0xe7,
	/* At least 50 TCK/SWCLK cycles with TMS/SWDIO high,
	 * putting both JTAG and SWD logic into reset state.
	 * NOTE:  some docs say "at least 5".
	 */
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};

/** Put the debug link into JTAG mode, if the target supports it.
 * The link's initial mode may be either SWD or JTAG.
 *
 * @param target Enters JTAG mode (if possible).
 *
 * Note that targets implemented with SW-DP do not support JTAG, and
 * that some targets which could otherwise support it may have been
 * configured to disable JTAG signaling
 *
 * @return ERROR_OK or else a fault code.
 */
int dap_to_jtag(struct target *target)
{
	int retval;

	LOG_DEBUG("Enter JTAG mode");

	/* REVISIT it's nasty to need to make calls to a "jtag"
	 * subsystem if the link isn't in JTAG mode...
	 */

	retval = jtag_add_tms_seq(8 * sizeof(swd2jtag_bitseq),
			swd2jtag_bitseq, TAP_RESET);
	if (retval == ERROR_OK)
		retval = jtag_execute_queue();

	/* REVISIT set up the DAP's ops vector for JTAG mode. */

	return retval;
}
