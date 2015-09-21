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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef OPENOCD_JTAG_SWD_H
#define OPENOCD_JTAG_SWD_H

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

/**
 * Line reset.
 *
 * Line reset is at least 50 SWCLK cycles with SWDIO driven high, followed
 * by at least one idle (low) cycle.
 */
static const uint8_t swd_seq_line_reset[] = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x03
};
static const unsigned swd_seq_line_reset_len = 51;

/**
 * JTAG-to-SWD sequence.
 *
 * The JTAG-to-SWD sequence is at least 50 TCK/SWCLK cycles with TMS/SWDIO
 * high, putting either interface logic into reset state, followed by a
 * specific 16-bit sequence and finally a line reset in case the SWJ-DP was
 * already in SWD mode.
 */
static const uint8_t swd_seq_jtag_to_swd[] = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x7b, 0x9e,
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0x0f,
};
static const unsigned swd_seq_jtag_to_swd_len = 118;

/**
 * SWD-to-JTAG sequence.
 *
 * The SWD-to-JTAG sequence is at least 50 TCK/SWCLK cycles with TMS/SWDIO
 * high, putting either interface logic into reset state, followed by a
 * specific 16-bit sequence and finally at least 5 TCK cycles to put the
 * JTAG TAP in TLR.
 */
static const uint8_t swd_seq_swd_to_jtag[] = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf3, 0x9c, 0xff
};
static const unsigned swd_seq_swd_to_jtag_len = 71;

/**
 * SWD-to-dormant sequence.
 *
 * This is at least 50 SWCLK cycles with SWDIO high to put the interface
 * in reset state, followed by a specific 16-bit sequence.
 */
static const uint8_t swd_seq_swd_to_dormant[] = {
	0xff, 0xff, 0xff, 0xff, 0xff, 0xff, 0xf3, 0x8e, 0x03
};
static const unsigned swd_seq_swd_to_dormant_len = 66;

/**
 * Dormant-to-SWD sequence.
 *
 * This is at least 8 TCK/SWCLK cycles with TMS/SWDIO high to abort any ongoing
 * selection alert sequence, followed by a specific 128-bit selection alert
 * sequence, followed by 4 TCK/SWCLK cycles with TMS/SWDIO low, followed by
 * a specific protocol-dependent activation code. For SWD the activation code
 * is an 8-bit sequence. The sequence ends with a line reset.
 */
static const uint8_t swd_seq_dormant_to_swd[] = {
	0xff,
	0x92, 0xf3, 0x09, 0x62, 0x95, 0x2d, 0x85, 0x86,
	0xe9, 0xaf, 0xdd, 0xe3, 0xa2, 0x0e, 0xbc, 0x19,
	0x10, 0xfa, 0xff, 0xff, 0xff, 0xff, 0xff, 0x3f
};
static const unsigned swd_seq_dormant_to_swd_len = 199;

enum swd_special_seq {
	LINE_RESET,
	JTAG_TO_SWD,
	SWD_TO_JTAG,
	SWD_TO_DORMANT,
	DORMANT_TO_SWD,
};

struct swd_driver {
	/**
	 * Initialize the debug link so it can perform SWD operations.
	 *
	 * As an example, this would switch a dual-mode debug adapter
	 * into SWD mode and out of JTAG mode.
	 *
	 * @return ERROR_OK on success, else a negative fault code.
	 */
	int (*init)(void);

	/**
	 * Set the SWCLK frequency of the SWD link.
	 *
	 * The driver should round the desired value, downwards if possible, to
	 * the nearest supported frequency. A negative value should be ignored
	 * and can be used to query the current setting. If the driver does not
	 * support a variable frequency a fixed, nominal, value should be
	 * returned.
	 *
	 * If the frequency is increased, it must not apply before the currently
	 * queued transactions are executed. If the frequency is lowered, it may
	 * apply immediately.
	 *
	 * @param hz The desired frequency in Hz.
	 * @return The actual resulting frequency after rounding.
	 */
	int_least32_t (*frequency)(int_least32_t hz);

	/**
	 * Queue a special SWDIO sequence.
	 *
	 * @param seq The special sequence to generate.
	 * @return ERROR_OK if the sequence was queued, negative error if the
	 * sequence is unsupported.
	 */
	int (*switch_seq)(enum swd_special_seq seq);

	/**
	 * Queued read of an AP or DP register.
	 *
	 * @param Command byte with APnDP/RnW/addr/parity bits
	 * @param Where to store value to read from register
	 * @param ap_delay_hint Number of idle cycles that may be
	 * needed after an AP access to avoid WAITs
	 */
	void (*read_reg)(uint8_t cmd, uint32_t *value, uint32_t ap_delay_hint);

	/**
	 * Queued write of an AP or DP register.
	 *
	 * @param Command byte with APnDP/RnW/addr/parity bits
	 * @param Value to be written to the register
	 * @param ap_delay_hint Number of idle cycles that may be
	 * needed after an AP access to avoid WAITs
	 */
	void (*write_reg)(uint8_t cmd, uint32_t value, uint32_t ap_delay_hint);

	/**
	 * Execute any queued transactions and collect the result.
	 *
	 * @return ERROR_OK on success, Ack response code on WAIT/FAULT
	 * or negative error code on other kinds of failure.
	 */
	int (*run)(void);

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

int swd_init_reset(struct command_context *cmd_ctx);
void swd_add_reset(int req_srst);

bool transport_is_swd(void);

#endif /* OPENOCD_JTAG_SWD_H */
