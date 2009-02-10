/***************************************************************************
*   Copyright (C) 2007 by Pavel Chromy                                    *
*   chromy@asix.cz                                                        *
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

#include "bitq.h"

/* project specific includes */
#include "log.h"
#include "types.h"
#include "jtag.h"
#include "configuration.h"

/* system includes */
#include <string.h>
#include <stdlib.h>
#include <unistd.h>

bitq_interface_t* bitq_interface;       /* low level bit queue interface */

bitq_state_t      bitq_in_state;        /* state of input queue */

u8* bitq_in_buffer;                     /* buffer dynamically reallocated as needed */
unsigned long     bitq_in_bufsize = 32; /* min. buffer size */

/*
 * input queue processing does not use jtag_read_buffer() to avoid unnecessary overhead
 * also the buffer for incomming data is reallocated only if necessary
 * no parameters, makes use of stored state information
 */
void bitq_in_proc(void)
{
	/* static information preserved between calls to increase performance */
	static u8*    in_buff;  /* pointer to buffer for scanned data */
	static int    in_idx;   /* index of byte being scanned */
	static u8     in_mask;  /* mask of next bit to be scanned */

	scan_field_t* field;
	int           tdo;

	/* loop through the queue */
	while (bitq_in_state.cmd)
	{
		/* only JTAG_SCAN command may return data */
		if (bitq_in_state.cmd->type==JTAG_SCAN)
		{
			/* loop through the fields */
			while (bitq_in_state.field_idx<bitq_in_state.cmd->cmd.scan->num_fields)
			{
				field = &bitq_in_state.cmd->cmd.scan->fields[bitq_in_state.field_idx];
				if (field->in_value || field->in_handler)
				{
					if (bitq_in_state.bit_pos==0)
					{
						/* initialize field scanning */
						in_mask = 0x01;
						in_idx  = 0;
						if (field->in_value)
							in_buff = field->in_value;
						else
						{
							/* buffer reallocation needed? */
							if (field->num_bits>bitq_in_bufsize * 8)
							{
								/* buffer previously allocated? */
								if (bitq_in_buffer!=NULL)
								{
									/* free it */
									free(bitq_in_buffer);
									bitq_in_buffer = NULL;
								}
								/* double the buffer size until it fits */
								while (field->num_bits>bitq_in_bufsize * 8)
									bitq_in_bufsize *= 2;
							}
							/* if necessary, allocate buffer and check for malloc error */
							if (bitq_in_buffer==NULL && ( bitq_in_buffer = malloc(bitq_in_bufsize) )==NULL)
							{
								LOG_ERROR("malloc error");
								exit(-1);
							}
							in_buff = (void*) bitq_in_buffer;
						}
					}

					/* field scanning */
					while (bitq_in_state.bit_pos<field->num_bits)
					{
						if ( ( tdo = bitq_interface->in() )<0 )
						{
#ifdef _DEBUG_JTAG_IO_
							LOG_DEBUG("bitq in EOF");
#endif
							return;
						}
						if (in_mask==0x01)
							in_buff[in_idx] = 0;
						if (tdo)
							in_buff[in_idx] |= in_mask;
						if (in_mask==0x80)
						{
							in_mask = 0x01;
							in_idx++;
						}
						else
							in_mask <<= 1;
						bitq_in_state.bit_pos++;
					}


					if (field->in_handler && bitq_in_state.status==ERROR_OK)
					{
						bitq_in_state.status = (*field->in_handler)(in_buff, field->in_handler_priv, field);
					}
				}

				bitq_in_state.field_idx++;  /* advance to next field */
				bitq_in_state.bit_pos = 0;  /* start next field from the first bit */
			}
		}
		bitq_in_state.cmd = bitq_in_state.cmd->next;    /* advance to next command */
		bitq_in_state.field_idx = 0;                    /* preselect first field */
	}
}


void bitq_io(int tms, int tdi, int tdo_req)
{
	bitq_interface->out(tms, tdi, tdo_req);
	/* check and process the input queue */
	if ( bitq_interface->in_rdy() )
		bitq_in_proc();
}


void bitq_end_state(tap_state_t state)
{
	if (state==-1)
		return;

	if (!tap_is_state_stable(state))
	{
		LOG_ERROR("BUG: %i is not a valid end state", state);
		exit(-1);
	}
	tap_set_end_state(state);
}


void bitq_state_move(tap_state_t new_state)
{
	int i = 0;
	u8  tms_scan;

	if (!tap_is_state_stable(tap_get_state()) || !tap_is_state_stable(new_state))
	{
		LOG_ERROR("TAP move from or to unstable state");
		exit(-1);
	}

	tms_scan = tap_get_tms_path(tap_get_state(), new_state);

	for (i = 0; i<7; i++)
	{
		bitq_io(tms_scan & 1, 0, 0);
		tms_scan >>= 1;
	}

	tap_set_state(new_state);
}


void bitq_path_move(pathmove_command_t* cmd)
{
	int i;

	for (i = 0; i<=cmd->num_states; i++)
	{
		if (tap_state_transition(tap_get_state(), false) == cmd->path[i])
			bitq_io(0, 0, 0);
		else if (tap_state_transition(tap_get_state(), true) == cmd->path[i])
			bitq_io(1, 0, 0);
		else
		{
			LOG_ERROR( "BUG: %s -> %s isn't a valid TAP transition", tap_state_name(
							 tap_get_state() ), tap_state_name(cmd->path[i]) );
			exit(-1);
		}

		tap_set_state(cmd->path[i]);
	}

	tap_set_end_state( tap_get_state() );
}


void bitq_runtest(int num_cycles)
{
	int i;

	/* only do a state_move when we're not already in IDLE */
	if (tap_get_state() != TAP_IDLE)
		bitq_state_move(TAP_IDLE);

	/* execute num_cycles */
	for (i = 0; i < num_cycles; i++)
		bitq_io(0, 0, 0);

	/* finish in end_state */
	if ( tap_get_state() != tap_get_end_state() )
		bitq_state_move( tap_get_end_state() );
}


void bitq_scan_field(scan_field_t* field, int pause)
{
	int bit_cnt;
	int tdo_req;

	u8* out_ptr;
	u8  out_mask;

	if (field->in_value || field->in_handler)
		tdo_req = 1;
	else
		tdo_req = 0;

	if (field->out_value==NULL)
	{
		/* just send zeros and request data from TDO */
		for (bit_cnt = field->num_bits; bit_cnt>1; bit_cnt--)
			bitq_io(0, 0, tdo_req);

		bitq_io(pause, 0, tdo_req);
	}
	else
	{
		/* send data, and optionally request TDO */
		out_mask = 0x01;
		out_ptr  = field->out_value;
		for (bit_cnt = field->num_bits; bit_cnt>1; bit_cnt--)
		{
			bitq_io(0, ( (*out_ptr) & out_mask )!=0, tdo_req);
			if (out_mask==0x80)
			{
				out_mask = 0x01;
				out_ptr++;
			}
			else
				out_mask <<= 1;
		}

		bitq_io(pause, ( (*out_ptr) & out_mask )!=0, tdo_req);
	}

	if (pause)
	{
		bitq_io(0, 0, 0);
		if (tap_get_state()==TAP_IRSHIFT)
			tap_set_state(TAP_IRPAUSE);
		else if (tap_get_state()==TAP_DRSHIFT)
			tap_set_state(TAP_DRPAUSE);
	}
}


void bitq_scan(scan_command_t* cmd)
{
	int i;

	if (cmd->ir_scan)
		bitq_state_move(TAP_IRSHIFT);
	else
		bitq_state_move(TAP_DRSHIFT);

	for (i = 0; i < cmd->num_fields - 1; i++)
		bitq_scan_field(&cmd->fields[i], 0);

	bitq_scan_field(&cmd->fields[i], 1);
}


int bitq_execute_queue(void)
{
	jtag_command_t* cmd = jtag_command_queue; /* currently processed command */

	bitq_in_state.cmd = jtag_command_queue;
	bitq_in_state.field_idx = 0;
	bitq_in_state.bit_pos   = 0;
	bitq_in_state.status    = ERROR_OK;

	while (cmd)
	{
		switch (cmd->type)
		{
		case JTAG_END_STATE:
#ifdef _DEBUG_JTAG_IO_
			LOG_DEBUG("end_state: %i", cmd->cmd.end_state->end_state);
#endif
			bitq_end_state(cmd->cmd.end_state->end_state);
			break;

		case JTAG_RESET:
#ifdef _DEBUG_JTAG_IO_
			LOG_DEBUG("reset trst: %i srst %i", cmd->cmd.reset->trst, cmd->cmd.reset->srst);
#endif
			if ( (cmd->cmd.reset->trst == 1) || ( cmd->cmd.reset->srst && (jtag_reset_config & RESET_SRST_PULLS_TRST) ) )
			{
				tap_set_state(TAP_RESET);
			}
			bitq_interface->reset(cmd->cmd.reset->trst, cmd->cmd.reset->srst);
			if ( bitq_interface->in_rdy() )
				bitq_in_proc();
			break;

		case JTAG_RUNTEST:
#ifdef _DEBUG_JTAG_IO_
			LOG_DEBUG("runtest %i cycles, end in %i", cmd->cmd.runtest->num_cycles, cmd->cmd.runtest->end_state);
#endif
			bitq_end_state(cmd->cmd.runtest->end_state);
			bitq_runtest(cmd->cmd.runtest->num_cycles);
			break;

		case JTAG_STATEMOVE:
#ifdef _DEBUG_JTAG_IO_
			LOG_DEBUG("statemove end in %i", cmd->cmd.statemove->end_state);
#endif
			bitq_end_state(cmd->cmd.statemove->end_state);
			bitq_state_move( tap_get_end_state() );   /* uncoditional TAP move */
			break;

		case JTAG_PATHMOVE:
#ifdef _DEBUG_JTAG_IO_
			LOG_DEBUG("pathmove: %i states, end in %i", cmd->cmd.pathmove->num_states,
					cmd->cmd.pathmove->path[cmd->cmd.pathmove->num_states - 1]);
#endif
			bitq_path_move(cmd->cmd.pathmove);
			break;

		case JTAG_SCAN:
#ifdef _DEBUG_JTAG_IO_
			LOG_DEBUG("scan end in %i", cmd->cmd.scan->end_state);
			if (cmd->cmd.scan->ir_scan)
				LOG_DEBUG("scan ir");
			else
				LOG_DEBUG("scan dr");
#endif
			bitq_end_state(cmd->cmd.scan->end_state);
			bitq_scan(cmd->cmd.scan);
			if ( tap_get_state() != tap_get_end_state() )
				bitq_state_move( tap_get_end_state() );
			break;

		case JTAG_SLEEP:
#ifdef _DEBUG_JTAG_IO_
			LOG_DEBUG("sleep %i", cmd->cmd.sleep->us);
#endif
			bitq_interface->sleep(cmd->cmd.sleep->us);
			if ( bitq_interface->in_rdy() )
				bitq_in_proc();
			break;

		default:
			LOG_ERROR("BUG: unknown JTAG command type encountered");
			exit(-1);
		}

		cmd = cmd->next;
	}

	bitq_interface->flush();
	bitq_in_proc();

	if (bitq_in_state.cmd)
	{
		LOG_ERROR("missing data from bitq interface");
		return ERROR_JTAG_QUEUE_FAILED;
	}
	if (bitq_interface->in()>=0)
	{
		LOG_ERROR("extra data from bitq interface");
		return ERROR_JTAG_QUEUE_FAILED;
	}

	return bitq_in_state.status;
}


void bitq_cleanup(void)
{
	if (bitq_in_buffer!=NULL)
	{
		free(bitq_in_buffer);
		bitq_in_buffer = NULL;
	}
}
