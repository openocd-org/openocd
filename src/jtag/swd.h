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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

/* Bits in SWD command packets, written from host to target
 * first bit on the wire is START
 */
#define SWD_CMD_START	(1 << 0)	/* always set */
#define SWD_CMD_APnDP	(1 << 1)	/* set only for AP access */
#define SWD_CMD_RnW	(1 << 2)		/* set only for read access */
#define SWD_CMD_A32	(3 << 3)		/* bits A[3:2] of register addr */
#define SWD_CMD_PARITY	(1 << 5)	/* parity of APnDP|RnW|A32 */
#define SWD_CMD_STOP	(0 << 6)	/* always clear for synch SWD */
#define SWD_CMD_PARK	(0 << 7)	/* not driven by host (pull high) */
/* followed by TRN, 3-bits of ACK, TRN */

/* pbit16 holds precomputed parity bits for each nibble */
#define pbit(parity, nibble) (parity << nibble)

static const uint16_t pbit16 =
	pbit(0, 0) | pbit(1, 1) | pbit(1, 2) | pbit(0, 3)
	| pbit(1, 4) | pbit(0, 5) | pbit(0, 6) | pbit(1, 7)
	| pbit(1, 8) | pbit(0, 9) | pbit(0, 0xa) | pbit(1, 0xb)
	| pbit(0, 0xc) | pbit(1, 0xd) | pbit(1, 0xe) | pbit(0, 0xf);

#define nibble_parity(nibble) (pbit16 & pbit(1, nibble))

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
	if (nibble_parity(cmd >> 1))
		cmd |= SWD_CMD_PARITY;

	/* driver handles START, STOP, and TRN */

	return cmd;
}

/* SWD_ACK_* bits are defined in <target/arm_adi_v5.h> */

/*
 * FOR NOW  ... SWD driver ops are synchronous and return ACK
 * status ... no quueueing.
 *
 * Individual ops are request/response, and fast-fail permits much
 * better fault handling.  Upper layers may queue if desired.
 */

struct swd_driver {
	/**
	 * Initialize the debug link so it can perform
	 * synchronous SWD operations.
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
	  * Synchronous read of an AP or DP register.
	  *
	  * @param cmd with APnDP/RnW/addr/parity bits
	  * @param where to store value to read from register
	  *
	  * @return SWD_ACK_* code for the transaction
	  *		or (negative) fault code
	  */
	 int (*read_reg)(uint8_t cmd, uint32_t *value);

	 /**
	  * Synchronous write of an AP or DP register.
	  *
	  * @param cmd with APnDP/RnW/addr/parity bits
	  * @param value to be written to the register
	  *
	  * @return SWD_ACK_* code for the transaction
	  *		or (negative) fault code
	  */
	 int (*write_reg)(uint8_t cmd, uint32_t value);

	/* XXX START WITH enough to:
	 *	init (synch mode, WCR)
	 *		for async, TRN > 1
	 *	read IDCODE from DP
	 */

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
	int *(*trace)(bool swo);
};

bool transport_is_swd(void);
