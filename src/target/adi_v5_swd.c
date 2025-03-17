// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *
 *   Copyright (C) 2010 by David Brownell
 ***************************************************************************/

/**
 * @file
 * Utilities to support ARM "Serial Wire Debug" (SWD), a low pin-count debug
 * link protocol used in cases where JTAG is not wanted.  This is coupled to
 * recent versions of ARM's "CoreSight" debug framework.  This specific code
 * is a transport level interface, with "target/arm_adi_v5.[hc]" code
 * understanding operation semantics, shared with the JTAG transport.
 *
 * Single DAP and multidrop-SWD support.
 *
 * for details, see "ARM IHI 0031A"
 * ARM Debug Interface v5 Architecture Specification
 * especially section 5.3 for SWD protocol
 * and "ARM IHI 0074C" ARM Debug Interface Architecture Specification ADIv6.0
 *
 * On many chips (most current Cortex-M3 parts) SWD is a run-time alternative
 * to JTAG.  Boards may support one or both.  There are also SWD-only chips,
 * (using SW-DP not SWJ-DP).
 *
 * Even boards that also support JTAG can benefit from SWD support, because
 * usually there's no way to access the SWO trace view mechanism in JTAG mode.
 * That is, trace access may require SWD support.
 *
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arm.h"
#include "arm_adi_v5.h"
#include <helper/time_support.h>

#include <transport/transport.h>
#include <jtag/interface.h>

#include <jtag/swd.h>

/* for debug, set do_sync to true to force synchronous transfers */
static bool do_sync;

static struct adiv5_dap *swd_multidrop_selected_dap;

static bool swd_multidrop_in_swd_state;


static int swd_queue_dp_write_inner(struct adiv5_dap *dap, unsigned int reg,
		uint32_t data);


static int swd_send_sequence(struct adiv5_dap *dap, enum swd_special_seq seq)
{
	const struct swd_driver *swd = adiv5_dap_swd_driver(dap);
	assert(swd);

	return swd->switch_seq(seq);
}

static void swd_finish_read(struct adiv5_dap *dap)
{
	const struct swd_driver *swd = adiv5_dap_swd_driver(dap);
	if (dap->last_read) {
		swd->read_reg(swd_cmd(true, false, DP_RDBUFF), dap->last_read, 0);
		dap->last_read = NULL;
	}
}

static void swd_clear_sticky_errors(struct adiv5_dap *dap)
{
	const struct swd_driver *swd = adiv5_dap_swd_driver(dap);
	assert(swd);

	swd->write_reg(swd_cmd(false, false, DP_ABORT),
		STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR, 0);
}

static int swd_run_inner(struct adiv5_dap *dap)
{
	const struct swd_driver *swd = adiv5_dap_swd_driver(dap);

	return swd->run();
}

static inline int check_sync(struct adiv5_dap *dap)
{
	return do_sync ? swd_run_inner(dap) : ERROR_OK;
}

/** Select the DP register bank */
static int swd_queue_dp_bankselect(struct adiv5_dap *dap, unsigned int reg)
{
	/* Only register address 0 (ADIv6 only) and 4 are banked. */
	if (is_adiv6(dap) ? (reg & 0xf) > 4 : (reg & 0xf) != 4)
		return ERROR_OK;

	uint32_t sel = (reg >> 4) & DP_SELECT_DPBANK;

	/* ADIv6 ensures DPBANKSEL = 0 after line reset */
	if ((dap->select_valid || (is_adiv6(dap) && dap->select_dpbanksel_valid))
			&& (sel == (dap->select & DP_SELECT_DPBANK)))
		return ERROR_OK;

	/* Use the AP part of dap->select regardless of dap->select_valid:
	 * if !dap->select_valid
	 * dap->select contains a speculative value likely going to be used
	 * in the following swd_queue_ap_bankselect() */
	sel |= (uint32_t)(dap->select & SELECT_AP_MASK);

	LOG_DEBUG_IO("DP BANK SELECT: %" PRIx32, sel);

	/* dap->select cache gets updated in the following call */
	return swd_queue_dp_write_inner(dap, DP_SELECT, sel);
}

static int swd_queue_dp_read_inner(struct adiv5_dap *dap, unsigned int reg,
		uint32_t *data)
{
	const struct swd_driver *swd = adiv5_dap_swd_driver(dap);
	assert(swd);

	int retval = swd_queue_dp_bankselect(dap, reg);
	if (retval != ERROR_OK)
		return retval;

	swd->read_reg(swd_cmd(true, false, reg), data, 0);

	return check_sync(dap);
}

static int swd_queue_dp_write_inner(struct adiv5_dap *dap, unsigned int reg,
		uint32_t data)
{
	int retval = ERROR_OK;
	const struct swd_driver *swd = adiv5_dap_swd_driver(dap);
	assert(swd);

	swd_finish_read(dap);

	if (reg == DP_SELECT) {
		dap->select = data | (dap->select & (0xffffffffull << 32));

		swd->write_reg(swd_cmd(false, false, reg), data, 0);

		retval = check_sync(dap);
		dap->select_valid = (retval == ERROR_OK);
		dap->select_dpbanksel_valid = dap->select_valid;

		return retval;
	}

	if (reg == DP_SELECT1)
		dap->select = ((uint64_t)data << 32) | (dap->select & 0xffffffffull);

	/* DP_ABORT write is not banked.
	 * Prevent writing DP_SELECT before as it would fail on locked up DP */
	if (reg != DP_ABORT)
		retval = swd_queue_dp_bankselect(dap, reg);

	if (retval == ERROR_OK) {
		swd->write_reg(swd_cmd(false, false, reg), data, 0);

		retval = check_sync(dap);
	}

	if (reg == DP_SELECT1)
		dap->select1_valid = (retval == ERROR_OK);

	return retval;
}


static int swd_multidrop_select_inner(struct adiv5_dap *dap, uint32_t *dpidr_ptr,
		uint32_t *dlpidr_ptr, bool clear_sticky)
{
	int retval;
	uint32_t dpidr, dlpidr;

	assert(dap_is_multidrop(dap));

	/* Send JTAG_TO_DORMANT and DORMANT_TO_SWD just once
	 * and then use shorter LINE_RESET until communication fails */
	if (!swd_multidrop_in_swd_state) {
		swd_send_sequence(dap, JTAG_TO_DORMANT);
		swd_send_sequence(dap, DORMANT_TO_SWD);
	} else {
		swd_send_sequence(dap, LINE_RESET);
	}

	/*
	 * Zero dap->select and set dap->select_dpbanksel_valid
	 * to skip the write to DP_SELECT before DPIDR read, avoiding
	 * the protocol error.
	 * Clear the other validity flags because the rest of the DP
	 * SELECT and SELECT1 registers is unknown after line reset.
	 */
	dap->select = 0;
	dap->select_dpbanksel_valid = true;
	dap->select_valid = false;
	dap->select1_valid = false;

	retval = swd_queue_dp_write_inner(dap, DP_TARGETSEL, dap->multidrop_targetsel);
	if (retval != ERROR_OK)
		return retval;

	retval = swd_queue_dp_read_inner(dap, DP_DPIDR, &dpidr);
	if (retval != ERROR_OK)
		return retval;

	if (clear_sticky) {
		/* Clear all sticky errors (including ORUN) */
		swd_clear_sticky_errors(dap);
	} else {
		/* Ideally just clear ORUN flag which is set by reset */
		retval = swd_queue_dp_write_inner(dap, DP_ABORT, ORUNERRCLR);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = swd_queue_dp_read_inner(dap, DP_DLPIDR, &dlpidr);
	if (retval != ERROR_OK)
		return retval;

	retval = swd_run_inner(dap);
	if (retval != ERROR_OK)
		return retval;

	if ((dpidr & DP_DPIDR_VERSION_MASK) < (2UL << DP_DPIDR_VERSION_SHIFT)) {
		LOG_INFO("Read DPIDR 0x%08" PRIx32
				 " has version < 2. A non multidrop capable device connected?",
				 dpidr);
		return ERROR_FAIL;
	}

	/* TODO: check TARGETID if DLIPDR is same for more than one DP */
	uint32_t expected_dlpidr = DP_DLPIDR_PROTVSN |
			(dap->multidrop_targetsel & DP_TARGETSEL_INSTANCEID_MASK);
	if (dlpidr != expected_dlpidr) {
		LOG_INFO("Read incorrect DLPIDR 0x%08" PRIx32
				 " (possibly CTRL/STAT value)",
				 dlpidr);
		return ERROR_FAIL;
	}

	LOG_DEBUG_IO("Selected DP_TARGETSEL 0x%08" PRIx32, dap->multidrop_targetsel);
	swd_multidrop_selected_dap = dap;
	swd_multidrop_in_swd_state = true;

	if (dpidr_ptr)
		*dpidr_ptr = dpidr;

	if (dlpidr_ptr)
		*dlpidr_ptr = dlpidr;

	return retval;
}

static int swd_multidrop_select(struct adiv5_dap *dap)
{
	if (!dap_is_multidrop(dap))
		return ERROR_OK;

	if (swd_multidrop_selected_dap == dap)
		return ERROR_OK;

	int retval = ERROR_OK;
	for (unsigned int retry = 0; ; retry++) {
		bool clear_sticky = retry > 0;

		retval = swd_multidrop_select_inner(dap, NULL, NULL, clear_sticky);
		if (retval == ERROR_OK)
			break;

		swd_multidrop_selected_dap = NULL;
		if (retry > 3) {
			LOG_ERROR("Failed to select multidrop %s", adiv5_dap_name(dap));
			dap->do_reconnect = true;
			return retval;
		}

		LOG_DEBUG("Failed to select multidrop %s, retrying...",
				  adiv5_dap_name(dap));
	}

	dap->do_reconnect = false;
	return retval;
}

static int swd_connect_multidrop(struct adiv5_dap *dap)
{
	int retval;
	uint32_t dpidr = 0xdeadbeef;
	uint32_t dlpidr = 0xdeadbeef;
	int64_t timeout = timeval_ms() + 500;

	do {
		/* Do not make any assumptions about SWD state in case of reconnect */
		if (dap->do_reconnect)
			swd_multidrop_in_swd_state = false;

		/* Clear link state, including the SELECT cache. */
		dap->do_reconnect = false;
		dap_invalidate_cache(dap);
		swd_multidrop_selected_dap = NULL;

		retval = swd_multidrop_select_inner(dap, &dpidr, &dlpidr, true);
		if (retval == ERROR_OK)
			break;

		swd_multidrop_in_swd_state = false;
		alive_sleep(1);

	} while (timeval_ms() < timeout);

	if (retval != ERROR_OK) {
		swd_multidrop_selected_dap = NULL;
		LOG_ERROR("Failed to connect multidrop %s", adiv5_dap_name(dap));
		return retval;
	}

	swd_multidrop_in_swd_state = true;
	LOG_INFO("SWD DPIDR 0x%08" PRIx32 ", DLPIDR 0x%08" PRIx32,
			  dpidr, dlpidr);

	return retval;
}

static int swd_connect_single(struct adiv5_dap *dap)
{
	int retval;
	uint32_t dpidr = 0xdeadbeef;
	int64_t timeout = timeval_ms() + 500;

	do {
		if (dap->switch_through_dormant) {
			swd_send_sequence(dap, JTAG_TO_DORMANT);
			swd_send_sequence(dap, DORMANT_TO_SWD);
		} else {
			swd_send_sequence(dap, JTAG_TO_SWD);
		}

		/* Clear link state, including the SELECT cache. */
		dap->do_reconnect = false;
		dap_invalidate_cache(dap);

		/* The sequences to enter in SWD (JTAG_TO_SWD and DORMANT_TO_SWD) end
		 * with a SWD line reset sequence (50 clk with SWDIO high).
		 * From ARM IHI 0031F ADIv5.2 and ARM IHI 0074C ADIv6.0,
		 * chapter B4.3.3 "Connection and line reset sequence":
		 * - DPv3 (ADIv6) only: line reset sets DP_SELECT_DPBANK to zero;
		 * - read of DP_DPIDR takes the connection out of reset;
		 * - write of DP_TARGETSEL keeps the connection in reset;
		 * - other accesses return protocol error (SWDIO not driven by target).
		 *
		 * dap_invalidate_cache() sets dap->select to zero and all validity
		 * flags to invalid. Set dap->select_dpbanksel_valid only
		 * to skip the write to DP_SELECT, avoiding the protocol error.
		 * Read DP_DPIDR to get out of reset.
		 */
		dap->select_dpbanksel_valid = true;

		retval = swd_queue_dp_read_inner(dap, DP_DPIDR, &dpidr);
		if (retval == ERROR_OK) {
			retval = swd_run_inner(dap);
			if (retval == ERROR_OK)
				break;
		}

		alive_sleep(1);

		dap->switch_through_dormant = !dap->switch_through_dormant;
	} while (timeval_ms() < timeout);

	if (retval != ERROR_OK) {
		LOG_ERROR("Error connecting DP: cannot read IDR");
		return retval;
	}

	LOG_INFO("SWD DPIDR 0x%08" PRIx32, dpidr);

	do {
		dap->do_reconnect = false;

		/* force clear all sticky faults */
		swd_clear_sticky_errors(dap);

		retval = swd_run_inner(dap);
		if (retval != ERROR_WAIT)
			break;

		alive_sleep(10);

	} while (timeval_ms() < timeout);

	return retval;
}

static int swd_pre_connect(struct adiv5_dap *dap)
{
	swd_multidrop_in_swd_state = false;

	return ERROR_OK;
}

static int swd_connect(struct adiv5_dap *dap)
{
	int status;

	/* FIXME validate transport config ... is the
	 * configured DAP present (check IDCODE)?
	 */

	/* Check if we should reset srst already when connecting, but not if reconnecting. */
	if (!dap->do_reconnect) {
		enum reset_types jtag_reset_config = jtag_get_reset_config();

		if (jtag_reset_config & RESET_CNCT_UNDER_SRST) {
			if (jtag_reset_config & RESET_SRST_NO_GATING)
				adapter_assert_reset();
			else
				LOG_WARNING("\'srst_nogate\' reset_config option is required");
		}
	}

	if (dap_is_multidrop(dap))
		status = swd_connect_multidrop(dap);
	else
		status = swd_connect_single(dap);

	/* IHI 0031E B4.3.2:
	 * "A WAIT response must not be issued to the ...
	 * ... writes to the ABORT register"
	 * swd_clear_sticky_errors() writes to the ABORT register only.
	 *
	 * Unfortunately at least Microchip SAMD51/E53/E54 returns WAIT
	 * in a corner case. Just try if ABORT resolves the problem.
	 */
	if (status == ERROR_WAIT) {
		LOG_WARNING("Connecting DP: stalled AP operation, issuing ABORT");

		dap->do_reconnect = false;

		status = swd_queue_dp_write_inner(dap, DP_ABORT,
			DAPABORT | STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR);

		if (status == ERROR_OK)
			status = swd_run_inner(dap);
	}

	if (status == ERROR_OK)
		status = dap_dp_init(dap);

	return status;
}

static int swd_check_reconnect(struct adiv5_dap *dap)
{
	if (dap->do_reconnect)
		return swd_connect(dap);

	return ERROR_OK;
}

static int swd_queue_ap_abort(struct adiv5_dap *dap, uint8_t *ack)
{
	const struct swd_driver *swd = adiv5_dap_swd_driver(dap);
	assert(swd);

	/* TODO: Send DAPABORT in swd_multidrop_select_inner()
	 * in the case the multidrop dap is not selected?
	 * swd_queue_ap_abort() is not currently used anyway...
	 */
	int retval = swd_multidrop_select(dap);
	if (retval != ERROR_OK)
		return retval;

	swd->write_reg(swd_cmd(false, false, DP_ABORT),
		DAPABORT | STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR, 0);
	return check_sync(dap);
}

static int swd_queue_dp_read(struct adiv5_dap *dap, unsigned int reg,
		uint32_t *data)
{
	int retval = swd_check_reconnect(dap);
	if (retval != ERROR_OK)
		return retval;

	retval = swd_multidrop_select(dap);
	if (retval != ERROR_OK)
		return retval;

	return swd_queue_dp_read_inner(dap, reg, data);
}

static int swd_queue_dp_write(struct adiv5_dap *dap, unsigned int reg,
		uint32_t data)
{
	int retval = swd_check_reconnect(dap);
	if (retval != ERROR_OK)
		return retval;

	retval = swd_multidrop_select(dap);
	if (retval != ERROR_OK)
		return retval;

	return swd_queue_dp_write_inner(dap, reg, data);
}

/** Select the AP register bank */
static int swd_queue_ap_bankselect(struct adiv5_ap *ap, unsigned int reg)
{
	int retval;
	struct adiv5_dap *dap = ap->dap;
	uint64_t sel;

	if (is_adiv6(dap))
		sel = ap->ap_num | (reg & 0x00000FF0);
	else
		sel = (ap->ap_num << 24) | (reg & ADIV5_DP_SELECT_APBANK);

	uint64_t sel_diff = (sel ^ dap->select) & SELECT_AP_MASK;

	bool set_select = !dap->select_valid || (sel_diff & 0xffffffffull);
	bool set_select1 = is_adiv6(dap) && dap->asize > 32
						&& (!dap->select1_valid
							|| sel_diff & (0xffffffffull << 32));

	if (set_select && set_select1) {
		/* Prepare DP bank for DP_SELECT1 now to save one write */
		sel |= (DP_SELECT1 & 0x000000f0) >> 4;
	} else {
		/* Use the DP part of dap->select regardless of dap->select_valid:
		 * if !dap->select_valid
		 * dap->select contains a speculative value likely going to be used
		 * in the following swd_queue_dp_bankselect().
		 * Moreover dap->select_valid should never be false here as a DP bank
		 * is always selected before selecting an AP bank */
		sel |= dap->select & DP_SELECT_DPBANK;
	}

	if (set_select) {
		LOG_DEBUG_IO("AP BANK SELECT: %" PRIx32, (uint32_t)sel);

		retval = swd_queue_dp_write(dap, DP_SELECT, (uint32_t)sel);
		if (retval != ERROR_OK)
			return retval;
	}

	if (set_select1) {
		LOG_DEBUG_IO("AP BANK SELECT1: %" PRIx32, (uint32_t)(sel >> 32));

		retval = swd_queue_dp_write(dap, DP_SELECT1, (uint32_t)(sel >> 32));
		if (retval != ERROR_OK)
			return retval;
	}

	return ERROR_OK;
}

static int swd_queue_ap_read(struct adiv5_ap *ap, unsigned int reg,
		uint32_t *data)
{
	struct adiv5_dap *dap = ap->dap;
	const struct swd_driver *swd = adiv5_dap_swd_driver(dap);
	assert(swd);

	int retval = swd_check_reconnect(dap);
	if (retval != ERROR_OK)
		return retval;

	retval = swd_multidrop_select(dap);
	if (retval != ERROR_OK)
		return retval;

	retval = swd_queue_ap_bankselect(ap, reg);
	if (retval != ERROR_OK)
		return retval;

	swd->read_reg(swd_cmd(true, true, reg), dap->last_read, ap->memaccess_tck);
	dap->last_read = data;

	return check_sync(dap);
}

static int swd_queue_ap_write(struct adiv5_ap *ap, unsigned int reg,
		uint32_t data)
{
	struct adiv5_dap *dap = ap->dap;
	const struct swd_driver *swd = adiv5_dap_swd_driver(dap);
	assert(swd);

	int retval = swd_check_reconnect(dap);
	if (retval != ERROR_OK)
		return retval;

	retval = swd_multidrop_select(dap);
	if (retval != ERROR_OK)
		return retval;

	swd_finish_read(dap);

	retval = swd_queue_ap_bankselect(ap, reg);
	if (retval != ERROR_OK)
		return retval;

	swd->write_reg(swd_cmd(false, true, reg), data, ap->memaccess_tck);

	return check_sync(dap);
}

/** Executes all queued DAP operations. */
static int swd_run(struct adiv5_dap *dap)
{
	int retval = swd_multidrop_select(dap);
	if (retval != ERROR_OK)
		return retval;

	swd_finish_read(dap);

	retval = swd_run_inner(dap);
	if (retval != ERROR_OK) {
		/* fault response */
		dap->do_reconnect = true;
	}

	return retval;
}

/** Put the SWJ-DP back to JTAG mode */
static void swd_quit(struct adiv5_dap *dap)
{
	const struct swd_driver *swd = adiv5_dap_swd_driver(dap);
	static bool done;

	/* There is no difference if the sequence is sent at the last
	 * or the first swd_quit() call, send it just once */
	if (done)
		return;

	done = true;
	if (dap_is_multidrop(dap)) {
		/* Emit the switch seq to dormant state regardless the state mirrored
		 * in swd_multidrop_in_swd_state. Doing so ensures robust operation
		 * in the case the variable is out of sync.
		 * Sending SWD_TO_DORMANT makes no change if the DP is already dormant. */
		swd->switch_seq(SWD_TO_DORMANT);
		swd_multidrop_in_swd_state = false;
		/* Revisit!
		 * Leaving DPs in dormant state was tested and offers some safety
		 * against DPs mismatch in case of unintentional use of non-multidrop SWD.
		 * To put SWJ-DPs to power-on state issue
		 * swd->switch_seq(DORMANT_TO_JTAG);
		 */
	} else {
		if (dap->switch_through_dormant) {
			swd->switch_seq(SWD_TO_DORMANT);
			swd->switch_seq(DORMANT_TO_JTAG);
		} else {
			swd->switch_seq(SWD_TO_JTAG);
		}
	}

	/* flush the queue to shift out the sequence before exit */
	swd->run();
}

const struct dap_ops swd_dap_ops = {
	.pre_connect_init = swd_pre_connect,
	.connect = swd_connect,
	.send_sequence = swd_send_sequence,
	.queue_dp_read = swd_queue_dp_read,
	.queue_dp_write = swd_queue_dp_write,
	.queue_ap_read = swd_queue_ap_read,
	.queue_ap_write = swd_queue_ap_write,
	.queue_ap_abort = swd_queue_ap_abort,
	.run = swd_run,
	.quit = swd_quit,
};

static const struct command_registration swd_commands[] = {
	{
		/*
		 * Set up SWD and JTAG targets identically, unless/until
		 * infrastructure improves ...  meanwhile, ignore all
		 * JTAG-specific stuff like IR length for SWD.
		 *
		 * REVISIT can we verify "just one SWD DAP" here/early?
		 */
		.name = "newdap",
		.handler = handle_jtag_newtap,
		.mode = COMMAND_CONFIG,
		.help = "declare a new SWD DAP",
		.usage = "basename dap_type ['-irlen' count] "
			"['-enable'|'-disable'] "
			"['-expected_id' number] "
			"['-ignore-version'] "
			"['-ignore-bypass'] "
			"['-ircapture' number] "
			"['-ir-bypass' number] "
			"['-mask' number]",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration swd_handlers[] = {
	{
		.name = "swd",
		.mode = COMMAND_ANY,
		.help = "SWD command group",
		.chain = swd_commands,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static int swd_select(struct command_context *ctx)
{
	/* FIXME: only place where global 'adapter_driver' is still needed */
	extern struct adapter_driver *adapter_driver;
	const struct swd_driver *swd = adapter_driver->swd_ops;
	int retval;

	retval = register_commands(ctx, NULL, swd_handlers);
	if (retval != ERROR_OK)
		return retval;

	 /* be sure driver is in SWD mode; start
	  * with hardware default TRN (1), it can be changed later
	  */
	if (!swd || !swd->read_reg || !swd->write_reg || !swd->init) {
		LOG_DEBUG("no SWD driver?");
		return ERROR_FAIL;
	}

	retval = swd->init();
	if (retval != ERROR_OK) {
		LOG_DEBUG("can't init SWD driver");
		return retval;
	}

	return retval;
}

static int swd_init(struct command_context *ctx)
{
	/* nothing done here, SWD is initialized
	 * together with the DAP */
	return ERROR_OK;
}

static struct transport swd_transport = {
	.name = "swd",
	.select = swd_select,
	.init = swd_init,
};

static void swd_constructor(void) __attribute__((constructor));
static void swd_constructor(void)
{
	transport_register(&swd_transport);
}

/** Returns true if the current debug session
 * is using SWD as its transport.
 */
bool transport_is_swd(void)
{
	return get_current_transport() == &swd_transport;
}
