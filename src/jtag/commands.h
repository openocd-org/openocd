/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2009 Zachary T Welch                                    *
 *   zw@superlucidity.net                                                  *
 ***************************************************************************/

#ifndef OPENOCD_JTAG_COMMANDS_H
#define OPENOCD_JTAG_COMMANDS_H

/**
 * The inferred type of a scan_command structure, indicating whether
 * the command has the host scan in from the device, the host scan out
 * to the device, or both.
 */
enum scan_type {
	/** From device to host, */
	SCAN_IN = 1,
	/** From host to device, */
	SCAN_OUT = 2,
	/** Full-duplex scan. */
	SCAN_IO = 3
};

/**
 * The scan_command provide a means of encapsulating a set of scan_field
 * structures that should be scanned in/out to the device.
 */
struct scan_command {
	/** instruction/not data scan */
	bool ir_scan;
	/** number of fields in *fields array */
	unsigned int num_fields;
	/** pointer to an array of data scan fields */
	struct scan_field *fields;
	/** state in which JTAG commands should finish */
	enum tap_state end_state;
};

struct statemove_command {
	/** state in which JTAG commands should finish */
	enum tap_state end_state;
};

struct pathmove_command {
	/** number of states in *path */
	unsigned int num_states;
	/** states that have to be passed */
	enum tap_state *path;
};

struct runtest_command {
	/** number of cycles to spend in Run-Test/Idle state */
	unsigned int num_cycles;
	/** state in which JTAG commands should finish */
	enum tap_state end_state;
};


struct stableclocks_command {
	/** number of clock cycles that should be sent */
	unsigned int num_cycles;
};


struct reset_command {
	/** Set TRST output: 0 = deassert, 1 = assert, -1 = no change */
	int trst;
	/** Set SRST output: 0 = deassert, 1 = assert, -1 = no change */
	int srst;
};

struct end_state_command {
	/** state in which JTAG commands should finish */
	enum tap_state end_state;
};

struct sleep_command {
	/** number of microseconds to sleep */
	uint32_t us;
};

/**
 * Encapsulates a series of bits to be clocked out, affecting state
 * and mode of the interface.
 *
 * In JTAG mode these are clocked out on TMS, using TCK.  They may be
 * used for link resets, transitioning between JTAG and SWD modes, or
 * to implement JTAG state machine transitions (implementing pathmove
 * or statemove operations).
 *
 * In SWD mode these are clocked out on SWDIO, using SWCLK, and are
 * used for link resets and transitioning between SWD and JTAG modes.
 */
struct tms_command {
	/** How many bits should be clocked out. */
	unsigned int num_bits;
	/** The bits to clock out; the LSB is bit 0 of bits[0]. */
	const uint8_t *bits;
};

/**
 * Defines a container type that hold a pointer to a JTAG command
 * structure of any defined type.
 */
union jtag_command_container {
	struct scan_command *scan;
	struct statemove_command *statemove;
	struct pathmove_command *pathmove;
	struct runtest_command *runtest;
	struct stableclocks_command *stableclocks;
	struct reset_command *reset;
	struct end_state_command *end_state;
	struct sleep_command *sleep;
	struct tms_command *tms;
};

/**
 * The type of the @c jtag_command_container contained by a
 * @c jtag_command structure.
 */
enum jtag_command_type {
	JTAG_SCAN         = 1,
	/* JTAG_TLR_RESET's non-minidriver implementation is a
	 * vestige from a statemove cmd. The statemove command
	 * is obsolete and replaced by pathmove.
	 *
	 * pathmove does not support reset as one of it's states,
	 * hence the need for an explicit statemove command.
	 */
	JTAG_TLR_RESET    = 2,
	JTAG_RUNTEST      = 3,
	JTAG_RESET        = 4,
	JTAG_PATHMOVE     = 6,
	JTAG_SLEEP        = 7,
	JTAG_STABLECLOCKS = 8,
	JTAG_TMS          = 9,
};

struct jtag_command {
	union jtag_command_container cmd;
	enum jtag_command_type type;
	struct jtag_command *next;
};

void *cmd_queue_alloc(size_t size);

void jtag_queue_command(struct jtag_command *cmd);
void jtag_command_queue_reset(void);
struct jtag_command *jtag_command_queue_get(void);

void jtag_scan_field_clone(struct scan_field *dst, const struct scan_field *src);
enum scan_type jtag_scan_type(const struct scan_command *cmd);
unsigned int jtag_scan_size(const struct scan_command *cmd);
int jtag_read_buffer(uint8_t *buffer, const struct scan_command *cmd);
int jtag_build_buffer(const struct scan_command *cmd, uint8_t **buffer);

#endif /* OPENOCD_JTAG_COMMANDS_H */
