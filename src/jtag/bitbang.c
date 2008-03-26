/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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

#include <sys/time.h>
#include <time.h>

bitbang_interface_t *bitbang_interface;

int bitbang_execute_queue(void);

/* The bitbang driver leaves the TCK 0 when in idle */


void bitbang_end_state(enum tap_state state)
{
	if (tap_move_map[state] != -1)
		end_state = state;
	else
	{
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
}

void bitbang_state_move(void) {
	
	int i=0, tms=0;
	u8 tms_scan = TAP_MOVE(cur_state, end_state);
	
	for (i = 0; i < 7; i++)
	{
		tms = (tms_scan >> i) & 1;
		bitbang_interface->write(0, tms, 0);
		bitbang_interface->write(1, tms, 0);
	}
	bitbang_interface->write(0, tms, 0);
	
	cur_state = end_state;
}

void bitbang_path_move(pathmove_command_t *cmd)
{
	int num_states = cmd->num_states;
	int state_count;
	int tms = 0;

	state_count = 0;
	while (num_states)
	{
		if (tap_transitions[cur_state].low == cmd->path[state_count])
		{
			tms = 0;
		}
		else if (tap_transitions[cur_state].high == cmd->path[state_count])
		{
			tms = 1;
		}
		else
		{
			LOG_ERROR("BUG: %s -> %s isn't a valid TAP transition", tap_state_strings[cur_state], tap_state_strings[cmd->path[state_count]]);
			exit(-1);
		}
		
		bitbang_interface->write(0, tms, 0);
		bitbang_interface->write(1, tms, 0);

		cur_state = cmd->path[state_count];
		state_count++;
		num_states--;
	}
	
	bitbang_interface->write(0, tms, 0);

	end_state = cur_state;
}

void bitbang_runtest(int num_cycles)
{
	int i;
	
	enum tap_state saved_end_state = end_state;
	
	/* only do a state_move when we're not already in RTI */
	if (cur_state != TAP_RTI)
	{
		bitbang_end_state(TAP_RTI);
		bitbang_state_move();
	}
	
	/* execute num_cycles */
	bitbang_interface->write(0, 0, 0);
	for (i = 0; i < num_cycles; i++)
	{
		bitbang_interface->write(1, 0, 0);
		bitbang_interface->write(0, 0, 0);
	}
	
	/* finish in end_state */
	bitbang_end_state(saved_end_state);
	if (cur_state != end_state)
		bitbang_state_move();
}

void bitbang_scan(int ir_scan, enum scan_type type, u8 *buffer, int scan_size)
{
	enum tap_state saved_end_state = end_state;
	int bit_cnt;
	
	if (!((!ir_scan && (cur_state == TAP_SD)) || (ir_scan && (cur_state == TAP_SI))))
	{
		if (ir_scan)
			bitbang_end_state(TAP_SI);
		else
			bitbang_end_state(TAP_SD);

		bitbang_state_move();
		bitbang_end_state(saved_end_state);
	}

	for (bit_cnt = 0; bit_cnt < scan_size; bit_cnt++)
	{
		/* if we're just reading the scan, but don't care about the output
		 * default to outputting 'low', this also makes valgrind traces more readable,
		 * as it removes the dependency on an uninitialised value
		 */ 
		if ((type != SCAN_IN) && ((buffer[bit_cnt/8] >> (bit_cnt % 8)) & 0x1))
		{
			bitbang_interface->write(0, (bit_cnt==scan_size-1) ? 1 : 0, 1);
			bitbang_interface->write(1, (bit_cnt==scan_size-1) ? 1 : 0, 1);
		} else {
			bitbang_interface->write(0, (bit_cnt==scan_size-1) ? 1 : 0, 0);
			bitbang_interface->write(1, (bit_cnt==scan_size-1) ? 1 : 0, 0);
		}
		
		if (type != SCAN_OUT)
		{
			/*
			TDO should be sampled on the rising edge, and will change 
			on the falling edge. 
			
			Because there is no way to read the signal exactly at the rising edge,
			read after the rising edge.

			This is plain IEEE 1149 JTAG - nothing specific to the OpenOCD or its JTAG
			API. 
			*/
			if (bitbang_interface->read())
				buffer[(bit_cnt)/8] |= 1 << ((bit_cnt) % 8);
			else
				buffer[(bit_cnt)/8] &= ~(1 << ((bit_cnt) % 8));
		}
	}
	
	/* TAP_SD & TAP_SI are illegal end states, so we always transition to the pause
	 * state which is a legal stable state from which statemove will work.
	 *  
	 * Exit1 -> Pause 
	 */
	bitbang_interface->write(0, 0, 0);
	bitbang_interface->write(1, 0, 0);
	bitbang_interface->write(0, 0, 0);
	
	if (ir_scan)
		cur_state = TAP_PI;
	else
		cur_state = TAP_PD;
	
	if (cur_state != end_state)
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
				LOG_DEBUG("end_state: %i", cmd->cmd.end_state->end_state);
#endif
				if (cmd->cmd.end_state->end_state != -1)
					bitbang_end_state(cmd->cmd.end_state->end_state);
				break;
			case JTAG_RESET:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("reset trst: %i srst %i", cmd->cmd.reset->trst, cmd->cmd.reset->srst);
#endif
				if (cmd->cmd.reset->trst == 1)
				{
					cur_state = TAP_TLR;
				}
				bitbang_interface->reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
				break;
			case JTAG_RUNTEST:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("runtest %i cycles, end in %i", cmd->cmd.runtest->num_cycles, cmd->cmd.runtest->end_state);
#endif
				if (cmd->cmd.runtest->end_state != -1)
					bitbang_end_state(cmd->cmd.runtest->end_state);
				bitbang_runtest(cmd->cmd.runtest->num_cycles);
				break;
			case JTAG_STATEMOVE:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("statemove end in %i", cmd->cmd.statemove->end_state);
#endif
				if (cmd->cmd.statemove->end_state != -1)
					bitbang_end_state(cmd->cmd.statemove->end_state);
				bitbang_state_move();
				break;
			case JTAG_PATHMOVE:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("pathmove: %i states, end in %i", cmd->cmd.pathmove->num_states, cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);
#endif
				bitbang_path_move(cmd->cmd.pathmove);
				break;
			case JTAG_SCAN:
#ifdef _DEBUG_JTAG_IO_
				LOG_DEBUG("%s scan end in %i",  (cmd->cmd.scan->ir_scan) ? "IR" : "DR", cmd->cmd.scan->end_state);
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

