/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "bitbang.h"

/* project specific includes */
#include "log.h"
#include "types.h"
#include "jtag.h"
#include "configuration.h"

/* system includes */
#include <string.h>
#include <stdlib.h>
#include <unistd.h>


/**
 * Function bitbang_stableclocks
 * issues a number of clock cycles while staying in a stable state.
 * Because the TMS value required to stay in the RESET state is a 1, whereas
 * the TMS value required to stay in any of the other stable states is a 0,
 * this function checks the current stable state to decide on the value of TMS
 * to use.
 */
static void bitbang_stableclocks(int num_cycles);


bitbang_interface_t *bitbang_interface;

/* DANGER!!!! clock absolutely *MUST* be 0 in idle or reset won't work!
 *
 * Set this to 1 and str912 reset halt will fail.
 *
 * If someone can submit a patch with an explanation it will be greatly
 * appreciated, but as far as I can tell (ØH) DCLK is generated upon
 * clk=0 in TAP_IDLE. Good luck deducing that from the ARM documentation!
 * The ARM documentation uses the term "DCLK is asserted while in the TAP_IDLE
 * state". With hardware there is no such thing as *while* in a state. There
 * are only edges. So clk => 0 is in fact a very subtle state transition that
 * happens *while* in the TAP_IDLE state. "#&¤"#¤&"#&"#&
 *
 * For "reset halt" the last thing that happens before srst is asserted
 * is that the breakpoint is set up. If DCLK is not wiggled one last
 * time before the reset, then the breakpoint is not set up and
 * "reset halt" will fail to halt.
 *
 */
#define CLOCK_IDLE() 0

int bitbang_execute_queue(void);


/* The bitbang driver leaves the TCK 0 when in idle */

void bitbang_end_state(tap_state_t state)
{
	if (tap_is_state_stable(state))
		tap_set_end_state(state);
	else
	{
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

void bitbang_state_move(void)
{
	int i=0, tms=0;
	u8 tms_scan = tap_get_tms_path(tap_get_state(), tap_get_end_state());

	for (i = 0; i < 7; i++)
	{
		tms = (tms_scan >> i) & 1;
		bitbang_interface->write(0, tms, 0);
		bitbang_interface->write(1, tms, 0);
	}
	bitbang_interface->write(CLOCK_IDLE(), tms, 0);

	tap_set_state(tap_get_end_state());
}

void bitbang_path_move(pathmove_command_t *cmd)
{
	int num_states = cmd->num_states;
	int state_count;
	int tms = 0;

	state_count = 0;
	while (num_states)
	{
		if (tap_state_transition(tap_get_state(), false) == cmd->path[state_count])
		{
			tms = 0;
		}
		else if (tap_state_transition(tap_get_state(), true) == cmd->path[state_count])
		{
			tms = 1;
		}
		else
		{
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition", tap_state_name(tap_get_state()), tap_state_name(cmd->path[state_count]));
			exit(-1);
		}

		bitbang_interface->write(0, tms, 0);
		bitbang_interface->write(1, tms, 0);

		tap_set_state(cmd->path[state_count]);
		state_count++;
		num_states--;
	}

	bitbang_interface->write(CLOCK_IDLE(), tms, 0);

	tap_set_end_state(tap_get_state());
}

void bitbang_runtest(int num_cycles)
{
	int i;

	tap_state_t saved_end_state = tap_get_end_state();

	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE)
	{
		bitbang_end_state(TAP_IDLE);
		bitbang_state_move();
	}

	/* execute num_cycles */
	for (i = 0; i < num_cycles; i++)
	{
		bitbang_interface->write(0, 0, 0);
		bitbang_interface->write(1, 0, 0);
	}
	bitbang_interface->write(CLOCK_IDLE(), 0, 0);

	/* finish in end_state */
	bitbang_end_state(saved_end_state);
	if (tap_get_state() != tap_get_end_state())
		bitbang_state_move();
}


static void bitbang_stableclocks(int num_cycles)
{
	int tms = (tap_get_state() == TAP_RESET ? 1 : 0);
	int i;

	/* send num_cycles clocks onto the cable */
	for (i = 0; i < num_cycles; i++)
	{
		bitbang_interface->write(1, tms, 0);
		bitbang_interface->write(0, tms, 0);
	}
}



void bitbang_scan(int ir_scan, enum scan_type type, u8 *buffer, int scan_size)
{
	tap_state_t saved_end_state = tap_get_end_state();
	int bit_cnt;

	if (!((!ir_scan && (tap_get_state() == TAP_DRSHIFT)) || (ir_scan && (tap_get_state() == TAP_IRSHIFT))))
	{
		if (ir_scan)
			bitbang_end_state(TAP_IRSHIFT);
		else
			bitbang_end_state(TAP_DRSHIFT);

		bitbang_state_move();
		bitbang_end_state(saved_end_state);
	}

	for (bit_cnt = 0; bit_cnt < scan_size; bit_cnt++)
	{
		int val=0;
		int tms=(bit_cnt==scan_size-1) ? 1 : 0;
		int tdi;
		int bytec=bit_cnt/8;
		int bcval=1<<(bit_cnt % 8);

		/* if we're just reading the scan, but don't care about the output
		 * default to outputting 'low', this also makes valgrind traces more readable,
		 * as it removes the dependency on an uninitialised value
		 */
		tdi=0;
		if ((type != SCAN_IN) && (buffer[bytec] & bcval))
			tdi=1;

		bitbang_interface->write(0, tms, tdi);

		if (type!=SCAN_OUT)
			val=bitbang_interface->read();

		bitbang_interface->write(1, tms, tdi);

		if (type != SCAN_OUT)
		{
			if (val)
				buffer[bytec] |= bcval;
			else
				buffer[bytec] &= ~bcval;
		}
	}

	/* TAP_DRSHIFT & TAP_IRSHIFT are illegal end states, so we always transition to the pause
	 * state which is a legal stable state from which statemove will work.
	 *
	 * Exit1 -> Pause
	 */
	bitbang_interface->write(0, 0, 0);
	bitbang_interface->write(1, 0, 0);
	bitbang_interface->write(CLOCK_IDLE(), 0, 0);

	if (ir_scan)
		tap_set_state(TAP_IRPAUSE);
	else
		tap_set_state(TAP_DRPAUSE);

	if (tap_get_state() != tap_get_end_state())
		bitbang_state_move();
}

int bitbang_execute_queue(void)
{
	jtag_command_t *cmd = jtag_command_queue; /* currently processed command */
	int scan_size;
	enum scan_type type;
	u8 *buffer;
	int retval;

	if (!bitbang_interface)
	{
		LOG_ERROR("BUG: Bitbang interface called, but not yet initialized");
		exit(-1);
	}

	/* return ERROR_OK, unless a jtag_read_buffer returns a failed check
	 * that wasn't handled by a caller-provided error handler
	 */
	retval = ERROR_OK;

	if(bitbang_interface->blink)
		bitbang_interface->blink(1);

	while (cmd)
	{
		switch (cmd->type)
		{
			case JTAG_END_STATE:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("end_state: %s", tap_state_name(cmd->cmd.end_state->end_state) );
#endif
				if (cmd->cmd.end_state->end_state != -1)
					bitbang_end_state(cmd->cmd.end_state->end_state);
				break;
			case JTAG_RESET:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("reset trst: %i srst %i", cmd->cmd.reset->trst, cmd->cmd.reset->srst);
#endif
				if ((cmd->cmd.reset->trst == 1) || (cmd->cmd.reset->srst && (jtag_reset_config & RESET_SRST_PULLS_TRST)))
				{
					tap_set_state(TAP_RESET);
				}
				bitbang_interface->reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
				break;
			case JTAG_RUNTEST:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("runtest %i cycles, end in %s", cmd->cmd.runtest->num_cycles, tap_state_name(cmd->cmd.runtest->end_state) );
#endif
				if (cmd->cmd.runtest->end_state != -1)
					bitbang_end_state(cmd->cmd.runtest->end_state);
				bitbang_runtest(cmd->cmd.runtest->num_cycles);
				break;

			case JTAG_STABLECLOCKS:
				/* this is only allowed while in a stable state.  A check for a stable
				 * state was done in jtag_add_clocks()
				 */
				bitbang_stableclocks(cmd->cmd.stableclocks->num_cycles);
				break;

			case JTAG_STATEMOVE:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("statemove end in %s", tap_state_name(cmd->cmd.statemove->end_state));
#endif
				if (cmd->cmd.statemove->end_state != -1)
					bitbang_end_state(cmd->cmd.statemove->end_state);
				bitbang_state_move();
				break;
			case JTAG_PATHMOVE:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("pathmove: %i states, end in %s", cmd->cmd.pathmove->num_states,
					tap_state_name(cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]));
#endif
				bitbang_path_move(cmd->cmd.pathmove);
				break;
			case JTAG_SCAN:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("%s scan end in %s",  (cmd->cmd.scan->ir_scan) ? "IR" : "DR", tap_state_name(cmd->cmd.scan->end_state) );
#endif
				if (cmd->cmd.scan->end_state != -1)
					bitbang_end_state(cmd->cmd.scan->end_state);
				scan_size = jtag_build_buffer(cmd->cmd.scan, &buffer);
				type = jtag_scan_type(cmd->cmd.scan);
				bitbang_scan(cmd->cmd.scan->ir_scan, type, buffer, scan_size);
				if (jtag_read_buffer(buffer, cmd->cmd.scan) != ERROR_OK)
					retval = ERROR_JTAG_QUEUE_FAILED;
				if (buffer)
					free(buffer);
				break;
			case JTAG_SLEEP:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("sleep %i", cmd->cmd.sleep->us);
#endif
				jtag_sleep(cmd->cmd.sleep->us);
				break;
			default:
				LOG_ERROR("BUG: unknown JTAG command type encountered");
				exit(-1);
		}
		cmd = cmd->next;
	}
	if(bitbang_interface->blink)
		bitbang_interface->blink(0);

	return retval;
}
