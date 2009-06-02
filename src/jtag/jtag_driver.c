/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2009 SoftPLC Corporation                                *
 *       http://softplc.com                                                *
 *   dick@softplc.com                                                      *
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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#define INCLUDE_JTAG_MINIDRIVER_H
#define INCLUDE_JTAG_INTERFACE_H
#include "jtag.h"
#include "command.h"

struct jtag_callback_entry
{
	struct jtag_callback_entry *next;

	jtag_callback_t callback;
	u8 *in;
	jtag_callback_data_t data1;
	jtag_callback_data_t data2;
	jtag_callback_data_t data3;
};

static struct jtag_callback_entry *jtag_callback_queue_head = NULL;
static struct jtag_callback_entry *jtag_callback_queue_tail = NULL;

static void jtag_callback_queue_reset(void)
{
	jtag_callback_queue_head = NULL;
	jtag_callback_queue_tail = NULL;
}

/**
 * Copy a scan_field_t for insertion into the queue.
 *
 * This allocates a new copy of out_value using cmd_queue_alloc.
 */
static void cmd_queue_scan_field_clone(scan_field_t * dst, const scan_field_t * src)
{
	dst->tap		= src->tap;
	dst->num_bits	= src->num_bits;
	dst->out_value	= buf_cpy(src->out_value, cmd_queue_alloc(CEIL(src->num_bits, 8)), src->num_bits);
	dst->in_value	= src->in_value;
}


/**
 * see jtag_add_ir_scan()
 *
 */
int interface_jtag_add_ir_scan(int in_num_fields, const scan_field_t *in_fields, tap_state_t state)
{
	size_t num_taps = jtag_NumEnabledTaps();

	jtag_command_t * cmd		= cmd_queue_alloc(sizeof(jtag_command_t));
	scan_command_t * scan		= cmd_queue_alloc(sizeof(scan_command_t));
	scan_field_t * out_fields	= cmd_queue_alloc(num_taps  * sizeof(scan_field_t));

	jtag_queue_command(cmd);

	cmd->type				= JTAG_SCAN;
	cmd->cmd.scan			= scan;

	scan->ir_scan			= true;
	scan->num_fields		= num_taps;	/* one field per device */
	scan->fields			= out_fields;
	scan->end_state			= state;


	scan_field_t * field = out_fields;	/* keep track where we insert data */

	/* loop over all enabled TAPs */

	for (jtag_tap_t * tap = jtag_NextEnabledTap(NULL); tap != NULL; tap = jtag_NextEnabledTap(tap))
	{
		/* search the input field list for fields for the current TAP */

		bool found = false;

		for (int j = 0; j < in_num_fields; j++)
		{
			if (tap != in_fields[j].tap)
				continue;

			/* if TAP is listed in input fields, copy the value */

			found = true;

			tap->bypass = 0;

			assert(in_fields[j].num_bits == tap->ir_length); /* input fields must have the same length as the TAP's IR */

			cmd_queue_scan_field_clone(field, in_fields + j);

			break;
		}

		if (!found)
		{
			/* if a TAP isn't listed in input fields, set it to BYPASS */

			tap->bypass = 1;

			field->tap			= tap;
			field->num_bits		= tap->ir_length;
			field->out_value	= buf_set_ones(cmd_queue_alloc(CEIL(tap->ir_length, 8)), tap->ir_length);
			field->in_value		= NULL; /* do not collect input for tap's in bypass */
		}

		/* update device information */
		buf_cpy(field->out_value, tap->cur_instr, tap->ir_length);

		field++;
	}

	assert(field == out_fields + num_taps); /* paranoia: jtag_NumEnabledTaps() and jtag_NextEnabledTap() not in sync */

	return ERROR_OK;
}

/**
 * see jtag_add_plain_ir_scan()
 *
 */
int interface_jtag_add_plain_ir_scan(int in_num_fields, const scan_field_t *in_fields, tap_state_t state)
{

	jtag_command_t * cmd		= cmd_queue_alloc(sizeof(jtag_command_t));
	scan_command_t * scan		= cmd_queue_alloc(sizeof(scan_command_t));
	scan_field_t * out_fields	= cmd_queue_alloc(in_num_fields * sizeof(scan_field_t));

	jtag_queue_command(cmd);

	cmd->type				= JTAG_SCAN;
	cmd->cmd.scan			= scan;

	scan->ir_scan			= true;
	scan->num_fields		= in_num_fields;
	scan->fields			= out_fields;
	scan->end_state			= state;

	for (int i = 0; i < in_num_fields; i++)
		cmd_queue_scan_field_clone(out_fields + i, in_fields + i);

	return ERROR_OK;
}



/**
 * see jtag_add_dr_scan()
 *
 */
int interface_jtag_add_dr_scan(int in_num_fields, const scan_field_t *in_fields, tap_state_t state)
{
	/* count devices in bypass */

	size_t bypass_devices = 0;

	for (jtag_tap_t * tap = jtag_NextEnabledTap(NULL); tap != NULL; tap = jtag_NextEnabledTap(tap))
	{
		if (tap->bypass)
			bypass_devices++;
	}

	jtag_command_t * cmd		= cmd_queue_alloc(sizeof(jtag_command_t));
	scan_command_t * scan		= cmd_queue_alloc(sizeof(scan_command_t));
	scan_field_t * out_fields	= cmd_queue_alloc((in_num_fields + bypass_devices) * sizeof(scan_field_t));

	jtag_queue_command(cmd);

	cmd->type				= JTAG_SCAN;
	cmd->cmd.scan			= scan;

	scan->ir_scan			= false;
	scan->num_fields		= in_num_fields + bypass_devices;
	scan->fields			= out_fields;
	scan->end_state			= state;


	scan_field_t * field = out_fields;	/* keep track where we insert data */

	/* loop over all enabled TAPs */

	for (jtag_tap_t * tap = jtag_NextEnabledTap(NULL); tap != NULL; tap = jtag_NextEnabledTap(tap))
	{
		/* if TAP is not bypassed insert matching input fields */

		if (!tap->bypass)
		{
			scan_field_t * start_field = field;	/* keep initial position for assert() */

			for (int j = 0; j < in_num_fields; j++)
			{
				if (tap != in_fields[j].tap)
					continue;

				cmd_queue_scan_field_clone(field, in_fields + j);

				field++;
			}

			assert(field > start_field);	/* must have at least one input field per not bypassed TAP */
		}

		/* if a TAP is bypassed, generated a dummy bit*/
		else
		{
			field->tap			= tap;
			field->num_bits		= 1;
			field->out_value	= NULL;
			field->in_value		= NULL;

			field++;
		}
	}

	assert(field == out_fields + scan->num_fields); /* no superfluous input fields permitted */

	return ERROR_OK;
}



/**
 * Generate a DR SCAN using the array of output values passed to the function
 *
 * This function assumes that the parameter target_tap specifies the one TAP
 * that is not bypassed. All other TAPs must be bypassed and the function will
 * generate a dummy 1bit field for them.
 *
 * For the target_tap a sequence of output-only fields will be generated where
 * each field has the size num_bits and the field's values are taken from
 * the array value.
 *
 * The bypass status of TAPs is set by jtag_add_ir_scan().
 *
 */
void interface_jtag_add_dr_out(jtag_tap_t *target_tap,
		int in_num_fields,
		const int *num_bits,
		const u32 *value,
		tap_state_t end_state)
{
	/* count devices in bypass */

	size_t bypass_devices = 0;

	for (jtag_tap_t * tap = jtag_NextEnabledTap(NULL); tap != NULL; tap = jtag_NextEnabledTap(tap))
	{
		if (tap->bypass)
			bypass_devices++;
	}


	jtag_command_t * cmd		= cmd_queue_alloc(sizeof(jtag_command_t));
	scan_command_t * scan		= cmd_queue_alloc(sizeof(scan_command_t));
	scan_field_t * out_fields	= cmd_queue_alloc((in_num_fields + bypass_devices) * sizeof(scan_field_t));

	jtag_queue_command(cmd);

	cmd->type				= JTAG_SCAN;
	cmd->cmd.scan			= scan;

	scan->ir_scan			= false;
	scan->num_fields		= in_num_fields + bypass_devices;
	scan->fields			= out_fields;
	scan->end_state			= end_state;


	bool target_tap_match	= false;

	scan_field_t * field = out_fields;	/* keep track where we insert data */

	/* loop over all enabled TAPs */

	for (jtag_tap_t * tap = jtag_NextEnabledTap(NULL); tap != NULL; tap = jtag_NextEnabledTap(tap))
	{
		/* if TAP is not bypassed insert matching input fields */

		if (!tap->bypass)
		{
			assert(tap == target_tap); /* target_tap must match the one not bypassed TAP */

			target_tap_match = true;

			for (int j = 0; j < in_num_fields; j++)
			{
				u8 out_value[4];
				size_t scan_size = num_bits[j];
				buf_set_u32(out_value, 0, scan_size, value[j]);

				field->tap			= tap;
				field->num_bits		= scan_size;
				field->out_value	= buf_cpy(out_value, cmd_queue_alloc(CEIL(scan_size, 8)), scan_size);
				field->in_value		= NULL;

				field++;
			}
		}

		/* if a TAP is bypassed, generated a dummy bit*/
		else
		{

			field->tap				= tap;
			field->num_bits			= 1;
			field->out_value		= NULL;
			field->in_value			= NULL;

			field++;
		}
	}

	assert(target_tap_match);	/* target_tap should be enabled and not bypassed */
}

/**
 * see jtag_add_plain_dr_scan()
 *
 */
int interface_jtag_add_plain_dr_scan(int in_num_fields, const scan_field_t *in_fields, tap_state_t state)
{
	jtag_command_t * cmd		= cmd_queue_alloc(sizeof(jtag_command_t));
	scan_command_t * scan		= cmd_queue_alloc(sizeof(scan_command_t));
	scan_field_t * out_fields	= cmd_queue_alloc(in_num_fields * sizeof(scan_field_t));

	jtag_queue_command(cmd);

	cmd->type				= JTAG_SCAN;
	cmd->cmd.scan			= scan;

	scan->ir_scan			= false;
	scan->num_fields		= in_num_fields;
	scan->fields			= out_fields;
	scan->end_state			= state;

	for (int i = 0; i < in_num_fields; i++)
		cmd_queue_scan_field_clone(out_fields + i, in_fields + i);

	return ERROR_OK;
}

int interface_jtag_add_tlr(void)
{
	tap_state_t state = TAP_RESET;

	/* allocate memory for a new list member */
	jtag_command_t * cmd = cmd_queue_alloc(sizeof(jtag_command_t));

	jtag_queue_command(cmd);

	cmd->type = JTAG_STATEMOVE;

	cmd->cmd.statemove = cmd_queue_alloc(sizeof(statemove_command_t));
	cmd->cmd.statemove->end_state = state;

	return ERROR_OK;
}

int interface_jtag_add_pathmove(int num_states, const tap_state_t *path)
{
	/* allocate memory for a new list member */
	jtag_command_t * cmd = cmd_queue_alloc(sizeof(jtag_command_t));

	jtag_queue_command(cmd);

	cmd->type = JTAG_PATHMOVE;

	cmd->cmd.pathmove = cmd_queue_alloc(sizeof(pathmove_command_t));
	cmd->cmd.pathmove->num_states = num_states;
	cmd->cmd.pathmove->path = cmd_queue_alloc(sizeof(tap_state_t) * num_states);

	for (int i = 0; i < num_states; i++)
		cmd->cmd.pathmove->path[i] = path[i];

	return ERROR_OK;
}

int interface_jtag_add_runtest(int num_cycles, tap_state_t state)
{
	/* allocate memory for a new list member */
	jtag_command_t * cmd = cmd_queue_alloc(sizeof(jtag_command_t));

	jtag_queue_command(cmd);

	cmd->type = JTAG_RUNTEST;

	cmd->cmd.runtest = cmd_queue_alloc(sizeof(runtest_command_t));
	cmd->cmd.runtest->num_cycles = num_cycles;
	cmd->cmd.runtest->end_state = state;

	return ERROR_OK;
}

int interface_jtag_add_clocks( int num_cycles )
{
	/* allocate memory for a new list member */
	jtag_command_t * cmd = cmd_queue_alloc(sizeof(jtag_command_t));

	jtag_queue_command(cmd);

	cmd->type = JTAG_STABLECLOCKS;

	cmd->cmd.stableclocks = cmd_queue_alloc(sizeof(stableclocks_command_t));
	cmd->cmd.stableclocks->num_cycles = num_cycles;

	return ERROR_OK;
}

int interface_jtag_add_reset(int req_trst, int req_srst)
{
	/* allocate memory for a new list member */
	jtag_command_t * cmd = cmd_queue_alloc(sizeof(jtag_command_t));

	jtag_queue_command(cmd);

	cmd->type = JTAG_RESET;

	cmd->cmd.reset = cmd_queue_alloc(sizeof(reset_command_t));
	cmd->cmd.reset->trst = req_trst;
	cmd->cmd.reset->srst = req_srst;

	return ERROR_OK;
}

int interface_jtag_add_sleep(u32 us)
{
	/* allocate memory for a new list member */
	jtag_command_t * cmd = cmd_queue_alloc(sizeof(jtag_command_t));

	jtag_queue_command(cmd);

	cmd->type = JTAG_SLEEP;

	cmd->cmd.sleep = cmd_queue_alloc(sizeof(sleep_command_t));
	cmd->cmd.sleep->us = us;

	return ERROR_OK;
}

/* add callback to end of queue */
void jtag_add_callback4(jtag_callback_t callback, u8 *in, jtag_callback_data_t data1, jtag_callback_data_t data2, jtag_callback_data_t data3)
{
	struct jtag_callback_entry *entry=cmd_queue_alloc(sizeof(struct jtag_callback_entry));

	entry->next=NULL;
	entry->callback=callback;
	entry->in=in;
	entry->data1=data1;
	entry->data2=data2;
	entry->data3=data3;

	if (jtag_callback_queue_head==NULL)
	{
		jtag_callback_queue_head=entry;
		jtag_callback_queue_tail=entry;
	} else
	{
		jtag_callback_queue_tail->next=entry;
		jtag_callback_queue_tail=entry;
	}
}

void interface_jtag_add_scan_check_alloc(scan_field_t *field)
{
	unsigned num_bytes = TAP_SCAN_BYTES(field->num_bits);
	field->in_value = (u8 *)cmd_queue_alloc(num_bytes);
}

int interface_jtag_execute_queue(void)
{
	int retval = default_interface_jtag_execute_queue();
	if (retval == ERROR_OK)
	{
		struct jtag_callback_entry *entry;
		for (entry=jtag_callback_queue_head; entry!=NULL; entry=entry->next)
		{
			retval=entry->callback(entry->in, entry->data1, entry->data2, entry->data3);
			if (retval!=ERROR_OK)
				break;
		}
	}

	jtag_command_queue_reset();
	jtag_callback_queue_reset();

	return retval;
}

void jtag_alloc_in_value32(scan_field_t *field)
{
	field->in_value=(u8 *)cmd_queue_alloc(4);
}

static int jtag_convert_to_callback4(u8 *in, jtag_callback_data_t data1, jtag_callback_data_t data2, jtag_callback_data_t data3)
{
	((jtag_callback1_t)data1)(in);
	return ERROR_OK;
}

void jtag_add_callback(jtag_callback1_t callback, u8 *in)
{
	jtag_add_callback4(jtag_convert_to_callback4, in, (jtag_callback_data_t)callback, 0, 0);
}

