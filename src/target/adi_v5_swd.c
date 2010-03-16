/***************************************************************************
 *
 *   Copyright (C) 2010 by David Brownell
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.
 ***************************************************************************/

/**
 * @file
 * This file implements SWD transport support for cores implementing
 the ARM Debug Interface version 5 (ADIv5).
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arm.h"
#include "arm_adi_v5.h"
#include <helper/time_support.h>

/*
 * This represents the bits which must be sent out on TMS/SWDIO to
 * switch a DAP implemented using an SWJ-DP module into SWD mode.
 * These bits are stored (and transmitted) LSB-first.
 *
 * See the DAP-Lite specification, section 2.2.5 for information
 * about making the debug link select SWD or JTAG.  (Similar info
 * is in a few other ARM documents.)
 */
static const uint8_t jtag2swd_bitseq[] = {
	/* More than 50 TCK/SWCLK cycles with TMS/SWDIO high,
	 * putting both JTAG and SWD logic into reset state.
	 */
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
	/* Switching sequence enables SWD and disables JTAG
	 * NOTE: bits in the DP's IDCODE may expose the need for
	 * an old/deprecated sequence (0xb6 0xed).
	 */
	0x9e, 0xe7,
	/* More than 50 TCK/SWCLK cycles with TMS/SWDIO high,
	 * putting both JTAG and SWD logic into reset state.
	 */
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xff,
};

/**
 * Put the debug link into SWD mode, if the target supports it.
 * The link's initial mode may be either JTAG (for example,
 * with SWJ-DP after reset) or SWD.
 *
 * @param target Enters SWD mode (if possible).
 *
 * Note that targets using the JTAG-DP do not support SWD, and that
 * some targets which could otherwise support it may have have been
 * configured to disable SWD signaling
 *
 * @return ERROR_OK or else a fault code.
 */
int dap_to_swd(struct target *target)
{
	int retval;

	LOG_DEBUG("Enter SWD mode");

	/* REVISIT it's nasty to need to make calls to a "jtag"
	 * subsystem if the link isn't in JTAG mode...
	 */

	retval =  jtag_add_tms_seq(8 * sizeof(jtag2swd_bitseq),
			jtag2swd_bitseq, TAP_INVALID);
	if (retval == ERROR_OK)
		retval = jtag_execute_queue();

	/* REVISIT set up the DAP's ops vector for SWD mode. */

	return retval;
}

