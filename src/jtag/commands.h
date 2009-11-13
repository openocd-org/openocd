/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2009 Zachary T Welch                                    *
 *   zw@superlucidity.net                                                  *
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
#ifndef JTAG_COMMANDS_H
#define JTAG_COMMANDS_H

#include "jtag.h"

/**
 * The inferred type of a scan_command_s structure, indicating whether
 * the command has the host scan in from the device, the host scan out
 * to the device, or both.
 */
enum scan_type {
	/// From device to host,
	SCAN_IN = 1,
	/// From host to device,
	SCAN_OUT = 2,
	/// Full-duplex scan.
	SCAN_IO = 3
};

/**
 * The scan_command provide a means of encapsulating a set of scan_field_s
 * structures that should be scanned in/out to the device.
 */
struct scan_command {
	/// instruction/not data scan
	bool ir_scan;
	/// number of fields in *fields array
	int num_fields;
	/// pointer to an array of data scan fields
	struct scan_field* fields;
	/// state in which JTAG commands should finish
	tap_state_t end_state;
};

struct statemove_command {
	/// state in which JTAG commands should finish
	tap_state_t end_state;
};

typedef struct pathmove_command_s
{
	/// number of states in *path
	int num_states;
	/// states that have to be passed
	tap_state_t* path;
} pathmove_command_t;

typedef struct runtest_command_s
{
	/// number of cycles to spend in Run-Test/Idle state
	int num_cycles;
	/// state in which JTAG commands should finish
	tap_state_t end_state;
} runtest_command_t;


typedef struct stableclocks_command_s
{
	/// number of clock cycles that should be sent
	int num_cycles;
} stableclocks_command_t;


typedef struct reset_command_s
{
	/// Set TRST output: 0 = deassert, 1 = assert, -1 = no change
	int trst;
	/// Set SRST output: 0 = deassert, 1 = assert, -1 = no change
	int srst;
} reset_command_t;

typedef struct end_state_command_s
{
	/// state in which JTAG commands should finish
	tap_state_t end_state;
} end_state_command_t;

typedef struct sleep_command_s
{
	/// number of microseconds to sleep
	uint32_t us;
} sleep_command_t;

/**
 * Defines a container type that hold a pointer to a JTAG command
 * structure of any defined type.
 */
typedef union jtag_command_container_u
{
	struct scan_command*         scan;
	struct statemove_command*    statemove;
	pathmove_command_t*     pathmove;
	runtest_command_t*      runtest;
	stableclocks_command_t* stableclocks;
	reset_command_t*        reset;
	end_state_command_t*    end_state;
	sleep_command_t* sleep;
} jtag_command_container_t;

/**
 * The type of the @c jtag_command_container_u contained by a
 * @c jtag_command_s structure.
 */
enum jtag_command_type {
	JTAG_SCAN         = 1,
	JTAG_STATEMOVE    = 2,
	JTAG_RUNTEST      = 3,
	JTAG_RESET        = 4,
	JTAG_PATHMOVE     = 6,
	JTAG_SLEEP        = 7,
	JTAG_STABLECLOCKS = 8
};

typedef struct jtag_command_s
{
	jtag_command_container_t cmd;
	enum jtag_command_type   type;
	struct jtag_command_s*   next;
} jtag_command_t;

/// The current queue of jtag_command_s structures.
extern jtag_command_t* jtag_command_queue;

void* cmd_queue_alloc(size_t size);
void cmd_queue_free(void);

void jtag_queue_command(jtag_command_t *cmd);
void jtag_command_queue_reset(void);

enum scan_type jtag_scan_type(const struct scan_command* cmd);
int jtag_scan_size(const struct scan_command* cmd);
int jtag_read_buffer(uint8_t* buffer, const struct scan_command* cmd);
int jtag_build_buffer(const struct scan_command* cmd, uint8_t** buffer);

#endif // JTAG_COMMANDS_H
