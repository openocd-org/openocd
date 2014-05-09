/***************************************************************************
 *   Copyright (C) 2009-2010 by David Brownell                             *
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

#ifndef SWD_H
#define SWD_H

#include <target/arm_adi_v5.h>

/* Bits in SWD command packets, written from host to target
 * first bit on the wire is START
 */
#define SWD_CMD_START	(1 << 0)	/* always set */
#define SWD_CMD_APnDP	(1 << 1)	/* set only for AP access */
#define SWD_CMD_RnW	(1 << 2)		/* set only for read access */
#define SWD_CMD_A32	(3 << 3)		/* bits A[3:2] of register addr */
#define SWD_CMD_PARITY	(1 << 5)	/* parity of APnDP|RnW|A32 */
#define SWD_CMD_STOP	(0 << 6)	/* always clear for synch SWD */
#define SWD_CMD_PARK	(1 << 7)	/* driven high by host */
/* followed by TRN, 3-bits of ACK, TRN */

/**
 * Construct a "cmd" byte, in lSB bit order, which swd_driver.read_reg()
 * and swd_driver.write_reg() methods will use directly.
 */
static inline uint8_t swd_cmd(bool is_read, bool is_ap, uint8_t regnum)
{
	uint8_t cmd = (is_ap ? SWD_CMD_APnDP : 0)
		| (is_read ? SWD_CMD_RnW : 0)
		| ((regnum & 0xc) << 1);

	/* 8 cmd bits 4:1 may be set */
	if (parity_u32(cmd))
		cmd |= SWD_CMD_PARITY;

	/* driver handles START, STOP, and TRN */

	return cmd;
}

/* SWD_ACK_* bits are defined in <target/arm_adi_v5.h> */

struct swd_driver {
	/**
	 * Initialize the debug link so it can perform SWD operations.
	 * @param trn value from WCR: how many clocks
	 * to not drive the SWDIO line at certain points in
	 * the SWD protocol (at least 1 clock).
	 *
	 * As an example, this would switch a dual-mode debug adapter
	 * into SWD mode and out of JTAG mode.
	 *
	 * @return ERROR_OK on success, else a negative fault code.
	 */
	int (*init)(uint8_t trn);


	/**
	 * Queued read of an AP or DP register.
	 *
	 * @param dap The DAP controlled by the SWD link.
	 * @param Command byte with APnDP/RnW/addr/parity bits
	 * @param Where to store value to read from register
	 */
	void (*read_reg)(struct adiv5_dap *dap, uint8_t cmd, uint32_t *value);

	/**
	 * Queued write of an AP or DP register.
	 *
	 * @param dap The DAP controlled by the SWD link.
	 * @param Command byte with APnDP/RnW/addr/parity bits
	 * @param Value to be written to the register
	 */
	void (*write_reg)(struct adiv5_dap *dap, uint8_t cmd, uint32_t value);

	/**
	 * Execute any queued transactions and collect the result.
	 *
	 * @param dap The DAP controlled by the SWD link.
	 * @return ERROR_OK on success, Ack response code on WAIT/FAULT
	 * or negative error code on other kinds of failure.
	 */
	int (*run)(struct adiv5_dap *dap);

	/**
	 * Configures data collection from the Single-wire
	 * trace (SWO) signal.
	 * @param swo true if SWO data collection should be routed.
	 *
	 * For example,  some debug adapters include a UART which
	 * is normally connected to a microcontroller's UART TX,
	 * but which may instead be connected to SWO for use in
	 * collecting ITM (and possibly ETM) trace data.
	 *
	 * @return ERROR_OK on success, else a negative fault code.
	 */
	int *(*trace)(struct adiv5_dap *dap, bool swo);
};

int swd_init_reset(struct command_context *cmd_ctx);
void swd_add_reset(int req_srst);

bool transport_is_swd(void);
bool transport_is_cmsis_dap(void);

#endif /* SWD_H */
