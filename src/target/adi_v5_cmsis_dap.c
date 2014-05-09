/***************************************************************************
 *   Copyright (C) 2013 by mike brown                                      *
 *   mike@theshedworks.org.uk                                              *
 *                                                                         *
 *   Copyright (C) 2013 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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

/**
 * @file
 * Utilities to support ARM "CMSIS-DAP", The CoreSight Debug Access Port.
 * This is coupled to recent versions of ARM's "CoreSight" debug framework.
 * This specific code is a transport level interface, with
 * "target/arm_adi_v5.[hc]" code understanding operation semantics,
 * shared with the SWD & JTAG transports.
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

#define CMSIS_CMD_DP            (0   << 0)    /* set only for AP access */
#define CMSIS_CMD_AP            (1   << 0)    /* set only for AP access */
#define CMSIS_CMD_READ          (1   << 1)    /* set only for read access */
#define CMSIS_CMD_WRITE         (0   << 1)    /* set only for read access */
#define CMSIS_CMD_A32(n)        ((n)&0x0C)    /* bits A[3:2] of register addr */
#define CMSIS_CMD_VAL_MATCH     (1   << 4)    /* Value Match */
#define CMSIS_CMD_MATCH_MSK     (1   << 5)    /* Match Mask */

/* YUK! - but this is currently a global.... */
extern struct jtag_interface *jtag_interface;

static int cmsis_dap_clear_sticky_errors(struct adiv5_dap *dap)
{
	LOG_DEBUG(" ");

	const struct swd_driver *swd = jtag_interface->swd;
	assert(swd);

	swd->write_reg(dap, (CMSIS_CMD_DP | CMSIS_CMD_WRITE | CMSIS_CMD_A32(DP_ABORT)),
			STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR);
	return ERROR_OK;
}

static int cmsis_dap_queue_ap_abort(struct adiv5_dap *dap, uint8_t *ack)
{
	LOG_DEBUG(" ");

	const struct swd_driver *swd = jtag_interface->swd;
	assert(swd);

	swd->write_reg(dap, (CMSIS_CMD_DP | CMSIS_CMD_WRITE | CMSIS_CMD_A32(DP_ABORT)),
			DAPABORT | STKCMPCLR | STKERRCLR | WDERRCLR | ORUNERRCLR);
	return ERROR_OK;
}

static int cmsis_dap_queue_dp_read(struct adiv5_dap *dap, unsigned reg, uint32_t *data)
{
	LOG_DEBUG("reg = %d", reg);

	const struct swd_driver *swd = jtag_interface->swd;
	assert(swd);

	swd->read_reg(dap, (CMSIS_CMD_DP | CMSIS_CMD_READ | CMSIS_CMD_A32(reg)), data);
	return ERROR_OK;
}

static int (cmsis_dap_queue_dp_write)(struct adiv5_dap *dap, unsigned reg, uint32_t data)
{
	LOG_DEBUG("reg = %d, data = 0x%08" PRIx32, reg, data);

	/* setting the ORUNDETECT bit causes issues for some targets,
	 * disable until we find out why */
	if (reg == DP_CTRL_STAT) {
		LOG_DEBUG("disabling overrun detection");
		data &= ~CORUNDETECT;
	}

	const struct swd_driver *swd = jtag_interface->swd;
	assert(swd);

	swd->write_reg(dap, (CMSIS_CMD_DP | CMSIS_CMD_WRITE | CMSIS_CMD_A32(reg)), data);
	return ERROR_OK;
}

/** Select the AP register bank matching bits 7:4 of reg. */
static int cmsis_dap_ap_q_bankselect(struct adiv5_dap *dap, unsigned reg)
{
	uint32_t select_ap_bank = reg & 0x000000F0;

	if (select_ap_bank == dap->ap_bank_value)
		return ERROR_OK;

	dap->ap_bank_value = select_ap_bank;
	select_ap_bank |= dap->ap_current;

	cmsis_dap_queue_dp_write(dap, DP_SELECT, select_ap_bank);
	return ERROR_OK;
}

static int (cmsis_dap_queue_ap_read)(struct adiv5_dap *dap, unsigned reg, uint32_t *data)
{
	cmsis_dap_ap_q_bankselect(dap, reg);

	LOG_DEBUG("reg = %d", reg);

	const struct swd_driver *swd = jtag_interface->swd;
	assert(swd);

	swd->read_reg(dap, (CMSIS_CMD_AP | CMSIS_CMD_READ | CMSIS_CMD_A32(reg)), data);

	return ERROR_OK;
}

static int (cmsis_dap_queue_ap_write)(struct adiv5_dap *dap, unsigned reg, uint32_t data)
{
	/* TODO: CSW_DBGSWENABLE (bit31) causes issues for some targets
	 * disable until we find out why */
	if (reg == AP_REG_CSW)
		data &= ~CSW_DBGSWENABLE;

	cmsis_dap_ap_q_bankselect(dap, reg);

	LOG_DEBUG("reg = %d, data = 0x%08" PRIx32, reg, data);

	const struct swd_driver *swd = jtag_interface->swd;
	assert(swd);

	swd->write_reg(dap, (CMSIS_CMD_AP | CMSIS_CMD_WRITE | CMSIS_CMD_A32(reg)), data);

	return ERROR_OK;
}

/** Executes all queued DAP operations. */
static int cmsis_dap_run(struct adiv5_dap *dap)
{
	LOG_DEBUG(" ");
	/*
	  Some debug dongles do more than asked for(e.g. EDBG from
	  Atmel) behind the scene and issuing an AP write
	  may result in more than just APACC SWD transaction, which in
	  turn can possibly set sticky error bit in CTRL/STAT register
	  of the DP(an example would be writing SYSRESETREQ to AIRCR).
	  Such adapters may interpret CMSIS-DAP secification
	  differently and not guarantee to be report those failures
	  via status byte of the return USB packet from CMSIS-DAP, so
	  we need to check CTRL/STAT and if that happens to clear it.
	  Note that once the CMSIS-DAP SWD implementation starts queueing
	  transfers this will cause loss of the transfers after the
	  failed one. At least a warning is printed.
	*/
	uint32_t ctrlstat;
	cmsis_dap_queue_dp_read(dap, DP_CTRL_STAT, &ctrlstat);

	int retval = jtag_interface->swd->run(dap);

	if (retval == ERROR_OK && (ctrlstat & SSTICKYERR))
		LOG_WARNING("Adapter returned success despite SSTICKYERR being set.");

	if (retval != ERROR_OK || (ctrlstat & SSTICKYERR))
		cmsis_dap_clear_sticky_errors(dap);

	return retval;
}

const struct dap_ops cmsis_dap_ops = {
	.is_swd = true,
	.queue_dp_read       = cmsis_dap_queue_dp_read,
	.queue_dp_write      = cmsis_dap_queue_dp_write,
	.queue_ap_read       = cmsis_dap_queue_ap_read,
	.queue_ap_write      = cmsis_dap_queue_ap_write,
	.queue_ap_abort      = cmsis_dap_queue_ap_abort,
	.run = cmsis_dap_run,
};

static const struct command_registration cmsis_dap_commands[] = {
	{
	/*
	* Set up SWD and JTAG targets identically, unless/until
	* infrastructure improves ...  meanwhile, ignore all
	* JTAG-specific stuff like IR length for SWD.
	*
	* REVISIT can we verify "just one SWD DAP" here/early?
	*/
		.name = "newdap",
		.jim_handler = jim_jtag_newtap,
		.mode = COMMAND_CONFIG,
		.help = "declare a new CMSIS-DAP"
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration cmsis_dap_handlers[] = {
	{
		.name = "cmsis-dap",
		.mode = COMMAND_ANY,
		.help = "cmsis_dap command group",
		.chain = cmsis_dap_commands,
	},
	COMMAND_REGISTRATION_DONE
};

static int cmsis_dap_select(struct command_context *ctx)
{
	LOG_DEBUG("CMSIS-ADI: cmsis_dap_select");

	int retval = register_commands(ctx, NULL, cmsis_dap_handlers);

	if (retval != ERROR_OK)
		return retval;

	/* FIXME: This needs a real overhaul!! FIXME
	 * be sure driver is in SWD mode; start
	 * with hardware default TRN (1), it can be changed later
	 * we use a bogus 'swd' driver to implement cmsis-dap as it is quite similar */

	const struct swd_driver *swd = jtag_interface->swd;
	if (!swd || !swd->read_reg || !swd->write_reg || !swd->init) {
		LOG_ERROR("no SWD driver?");
		return ERROR_FAIL;
	}

	retval = swd->init();
	if (retval != ERROR_OK) {
		LOG_ERROR("unable to init CMSIS-DAP driver");
		return retval;
	}

	return retval;
}

static int cmsis_dap_init(struct command_context *ctx)
{
	struct target *target = get_current_target(ctx);
	struct arm *arm = target_to_arm(target);
	struct adiv5_dap *dap = arm->dap;
	uint32_t idcode;
	int status;

	LOG_DEBUG("CMSIS-ADI init");

	/* Force the DAP's ops vector for CMSIS-DAP mode.
	 * messy - is there a better way? */
	arm->dap->ops = &cmsis_dap_ops;

	/* FIXME validate transport config ... is the
	 * configured DAP present (check IDCODE)?
	 * Is *only* one DAP configured?
	 *
	 * MUST READ IDCODE
	 */

	/* Note, debugport_init() does setup too */

#if 0
	const struct swd_driver *swd = jtag_interface->swd;
	if (!swd || !swd->read_reg || !swd->write_reg || !swd->init) {
		LOG_ERROR("no SWD driver?");
		return ERROR_FAIL;
	}

	int retval = swd->init(1);
	if (retval != ERROR_OK) {
		LOG_ERROR("unable to init CMSIS-DAP driver");
		return retval;
	}
#endif

	status = cmsis_dap_queue_dp_read(dap, DP_IDCODE, &idcode);

	if (status == ERROR_OK)
		LOG_INFO("IDCODE 0x%08" PRIx32, idcode);

	/* force clear all sticky faults */
	cmsis_dap_clear_sticky_errors(dap);

	/* this is a workaround to get polling working */
	jtag_add_reset(0, 0);

	return status;
}

static struct transport cmsis_dap_transport = {
	.name = "cmsis-dap",
	.select = cmsis_dap_select,
	.init = cmsis_dap_init,
};

static void cmsis_dap_constructor(void) __attribute__((constructor));
static void cmsis_dap_constructor(void)
{
	transport_register(&cmsis_dap_transport);
}

/** Returns true if the current debug session
 * is using CMSIS-DAP as its transport.
 */
bool transport_is_cmsis_dap(void)
{
	return get_current_transport() == &cmsis_dap_transport;
}
