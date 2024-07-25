// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007-2010 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2009 SoftPLC Corporation                                *
 *       http://softplc.com                                                *
 *   dick@softplc.com                                                      *
 *                                                                         *
 *   Copyright (C) 2009 Zachary T Welch                                    *
 *   zw@superlucidity.net                                                  *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/jtag.h>
#include <jtag/interface.h>
#include <jtag/commands.h>
#include <jtag/minidriver.h>
#include <helper/command.h>

struct jtag_callback_entry {
	struct jtag_callback_entry *next;

	jtag_callback_t callback;
	jtag_callback_data_t data0;
	jtag_callback_data_t data1;
	jtag_callback_data_t data2;
	jtag_callback_data_t data3;
};

static struct jtag_callback_entry *jtag_callback_queue_head;
static struct jtag_callback_entry *jtag_callback_queue_tail;

static void jtag_callback_queue_reset(void)
{
	jtag_callback_queue_head = NULL;
	jtag_callback_queue_tail = NULL;
}

static int
check_ir_scan_fields_on_tap(struct scan_fields_on_tap tap_fields)
{
	unsigned int ir_length = 0;
	for (size_t i = 0; i < tap_fields.num_fields; ++i)
		ir_length += tap_fields.fields[i].num_bits;
	if (ir_length == tap_fields.tap->ir_length)
		return ERROR_OK;

	LOG_ERROR("BUG: %u bits are to be scanned into the IR of TAP %s with IR "
			"length of %u.", ir_length, jtag_tap_name(tap_fields.tap),
			tap_fields.tap->ir_length);
	return ERROR_FAIL;
}

static int
check_fields_on_taps(bool ir_scan, const struct scan_fields_on_tap *tap_fields,
		size_t count)
{
	for (size_t i = 0; i < count; ++i) {
		if (!tap_fields[i].tap->enabled) {
			LOG_ERROR("BUG: TAP %s is disabled.",
					jtag_tap_name(tap_fields[i].tap));
			return ERROR_FAIL;
		}
		if (ir_scan && check_ir_scan_fields_on_tap(tap_fields[i]) != ERROR_OK)
			return ERROR_FAIL;
	}
	for (size_t i = 1; i < count; ++i) {
		if (tap_fields[i - 1].tap->abs_chain_position
				>= tap_fields[i].tap->abs_chain_position) {
			LOG_ERROR("BUG: Expecting TAPs %s and %s to be passed "
					"in the same order as in the chain.",
					jtag_tap_name(tap_fields[i - 1].tap),
					jtag_tap_name(tap_fields[i].tap));
			return ERROR_FAIL;
		}
	}
	return ERROR_OK;
}

static size_t get_num_fields(const struct scan_fields_on_tap *tap_fields,
		size_t count)
{
	size_t num_fields = 0;
	for (size_t i = 0; i < count; ++i)
		num_fields += tap_fields[i].num_fields;
	return num_fields;
}

static void fill_bypass_scan_field(struct scan_field *field, bool ir_scan,
		const struct jtag_tap *tap)
{
	field->in_value = NULL; /* do not collect input for tap's in bypass */
	if (!ir_scan) {
		field->num_bits = 1;
		field->out_value = NULL;
		return;
	}
	field->num_bits = tap->ir_length;
	uint8_t *out_value = cmd_queue_alloc(DIV_ROUND_UP(tap->ir_length, 8));
	if (tap->ir_bypass_value)
		buf_set_u64(out_value, 0, tap->ir_length, tap->ir_bypass_value);
	else
		buf_set_ones(out_value, tap->ir_length);
	field->out_value = out_value;
}

/* The total length of "fields" should be equal to "tap->ir_length".
 * This is achived by definiton for TAPs in bypass (see
 * "fill_bypass_scan_field()") and pre-validated in "check_fields_on_taps()"
 * for active TAPs.*/
static void set_tap_cur_instr(struct jtag_tap *tap, struct scan_field *fields)
{
	size_t i = 0;
	for (unsigned int dst_offset = 0; dst_offset < tap->ir_length;
			dst_offset += fields[i].num_bits, ++i) {
		assert(fields[i].num_bits <= tap->ir_length - dst_offset);
		buf_set_buf(fields[i].out_value, 0, tap->cur_instr, dst_offset,
				fields[i].num_bits);
	}
}

static int fill_scan_fields(struct scan_field *out_fields, bool ir_scan,
		const struct scan_fields_on_tap *tap_fields)
{
	const struct scan_fields_on_tap *in = tap_fields;
	struct scan_field *out = out_fields;
	for (struct jtag_tap *tap = jtag_tap_next_enabled(NULL); tap;
			tap = jtag_tap_next_enabled(tap)) {
		bool bypass = tap != in->tap;

		if (ir_scan) {
			tap->bypass = bypass;
		} else if (tap->bypass != bypass) {
			LOG_ERROR("TAP %s %s expected to be in BYPASS",
					jtag_tap_name(tap), bypass ? "is" : "is not");
			return ERROR_FAIL;
		}

		size_t n_fields;
		if (bypass) {
			fill_bypass_scan_field(out, ir_scan, tap);
			n_fields = 1;
		} else {
			for (size_t i = 0; i < in->num_fields; ++i)
				jtag_scan_field_clone(out + i, in->fields + i);
			n_fields = in->num_fields;
			++in;
		}

		/* update device information */
		if (ir_scan)
			set_tap_cur_instr(tap, out);

		out += n_fields;
	}
	return ERROR_OK;
}

/**
 * see jtag_add_ir/dr_scan()
 *
 */
int interface_jtag_add_scan(bool ir_scan, const struct scan_fields_on_tap *tap_fields,
		size_t n_active_taps, enum tap_state state)
{
	int res = check_fields_on_taps(ir_scan, tap_fields, n_active_taps);
	if (res != ERROR_OK)
		return ERROR_FAIL;

	size_t bypass_taps = jtag_tap_count_enabled() - n_active_taps;
	size_t num_fields = bypass_taps + get_num_fields(tap_fields, n_active_taps);

	struct scan_field *out_fields = cmd_queue_alloc(num_fields * sizeof(struct scan_field));
	struct jtag_command *cmd = cmd_queue_alloc(sizeof(struct jtag_command));
	struct scan_command *scan = cmd_queue_alloc(sizeof(struct scan_command));


	res = fill_scan_fields(out_fields, ir_scan, tap_fields);
	if (res != ERROR_OK)
		return res;

	jtag_queue_command(cmd);

	cmd->type = JTAG_SCAN;
	cmd->cmd.scan = scan;

	scan->ir_scan = ir_scan;
	scan->num_fields = num_fields;
	scan->fields = out_fields;

	scan->end_state = state;

	return ERROR_OK;
}

static int jtag_add_plain_scan(int num_bits, const uint8_t *out_bits,
		uint8_t *in_bits, enum tap_state state, bool ir_scan)
{
	struct jtag_command *cmd = cmd_queue_alloc(sizeof(struct jtag_command));
	struct scan_command *scan = cmd_queue_alloc(sizeof(struct scan_command));
	struct scan_field *out_fields = cmd_queue_alloc(sizeof(struct scan_field));

	jtag_queue_command(cmd);

	cmd->type = JTAG_SCAN;
	cmd->cmd.scan = scan;

	scan->ir_scan = ir_scan;
	scan->num_fields = 1;
	scan->fields = out_fields;
	scan->end_state = state;

	out_fields->num_bits = num_bits;
	out_fields->out_value = buf_cpy(out_bits, cmd_queue_alloc(DIV_ROUND_UP(num_bits, 8)), num_bits);
	out_fields->in_value = in_bits;

	return ERROR_OK;
}

int interface_jtag_add_plain_dr_scan(int num_bits, const uint8_t *out_bits, uint8_t *in_bits, enum tap_state state)
{
	return jtag_add_plain_scan(num_bits, out_bits, in_bits, state, false);
}

int interface_jtag_add_plain_ir_scan(int num_bits, const uint8_t *out_bits, uint8_t *in_bits, enum tap_state state)
{
	return jtag_add_plain_scan(num_bits, out_bits, in_bits, state, true);
}

int interface_jtag_add_tlr(void)
{
	enum tap_state state = TAP_RESET;

	/* allocate memory for a new list member */
	struct jtag_command *cmd = cmd_queue_alloc(sizeof(struct jtag_command));

	jtag_queue_command(cmd);

	cmd->type = JTAG_TLR_RESET;

	cmd->cmd.statemove = cmd_queue_alloc(sizeof(struct statemove_command));
	cmd->cmd.statemove->end_state = state;

	return ERROR_OK;
}

int interface_add_tms_seq(unsigned int num_bits, const uint8_t *seq, enum tap_state state)
{
	struct jtag_command *cmd;

	cmd = cmd_queue_alloc(sizeof(struct jtag_command));
	if (!cmd)
		return ERROR_FAIL;

	cmd->type = JTAG_TMS;
	cmd->cmd.tms = cmd_queue_alloc(sizeof(*cmd->cmd.tms));
	if (!cmd->cmd.tms)
		return ERROR_FAIL;

	/* copy the bits; our caller doesn't guarantee they'll persist */
	cmd->cmd.tms->num_bits = num_bits;
	cmd->cmd.tms->bits = buf_cpy(seq,
			cmd_queue_alloc(DIV_ROUND_UP(num_bits, 8)), num_bits);
	if (!cmd->cmd.tms->bits)
		return ERROR_FAIL;

	jtag_queue_command(cmd);

	return ERROR_OK;
}

int interface_jtag_add_pathmove(unsigned int num_states, const enum tap_state *path)
{
	/* allocate memory for a new list member */
	struct jtag_command *cmd = cmd_queue_alloc(sizeof(struct jtag_command));

	jtag_queue_command(cmd);

	cmd->type = JTAG_PATHMOVE;

	cmd->cmd.pathmove = cmd_queue_alloc(sizeof(struct pathmove_command));
	cmd->cmd.pathmove->num_states = num_states;
	cmd->cmd.pathmove->path = cmd_queue_alloc(sizeof(enum tap_state) * num_states);

	for (unsigned int i = 0; i < num_states; i++)
		cmd->cmd.pathmove->path[i] = path[i];

	return ERROR_OK;
}

int interface_jtag_add_runtest(unsigned int num_cycles, enum tap_state state)
{
	/* allocate memory for a new list member */
	struct jtag_command *cmd = cmd_queue_alloc(sizeof(struct jtag_command));

	jtag_queue_command(cmd);

	cmd->type = JTAG_RUNTEST;

	cmd->cmd.runtest = cmd_queue_alloc(sizeof(struct runtest_command));
	cmd->cmd.runtest->num_cycles = num_cycles;
	cmd->cmd.runtest->end_state = state;

	return ERROR_OK;
}

int interface_jtag_add_clocks(unsigned int num_cycles)
{
	/* allocate memory for a new list member */
	struct jtag_command *cmd = cmd_queue_alloc(sizeof(struct jtag_command));

	jtag_queue_command(cmd);

	cmd->type = JTAG_STABLECLOCKS;

	cmd->cmd.stableclocks = cmd_queue_alloc(sizeof(struct stableclocks_command));
	cmd->cmd.stableclocks->num_cycles = num_cycles;

	return ERROR_OK;
}

int interface_jtag_add_reset(int req_trst, int req_srst)
{
	/* allocate memory for a new list member */
	struct jtag_command *cmd = cmd_queue_alloc(sizeof(struct jtag_command));

	jtag_queue_command(cmd);

	cmd->type = JTAG_RESET;

	cmd->cmd.reset = cmd_queue_alloc(sizeof(struct reset_command));
	cmd->cmd.reset->trst = req_trst;
	cmd->cmd.reset->srst = req_srst;

	return ERROR_OK;
}

int interface_jtag_add_sleep(uint32_t us)
{
	/* allocate memory for a new list member */
	struct jtag_command *cmd = cmd_queue_alloc(sizeof(struct jtag_command));

	jtag_queue_command(cmd);

	cmd->type = JTAG_SLEEP;

	cmd->cmd.sleep = cmd_queue_alloc(sizeof(struct sleep_command));
	cmd->cmd.sleep->us = us;

	return ERROR_OK;
}

/* add callback to end of queue */
static void interface_jtag_add_callback4(jtag_callback_t callback,
		jtag_callback_data_t data0, jtag_callback_data_t data1,
		jtag_callback_data_t data2, jtag_callback_data_t data3)
{
	struct jtag_callback_entry *entry = cmd_queue_alloc(sizeof(struct jtag_callback_entry));

	entry->next = NULL;
	entry->callback = callback;
	entry->data0 = data0;
	entry->data1 = data1;
	entry->data2 = data2;
	entry->data3 = data3;

	if (!jtag_callback_queue_head) {
		jtag_callback_queue_head = entry;
		jtag_callback_queue_tail = entry;
	} else {
		jtag_callback_queue_tail->next = entry;
		jtag_callback_queue_tail = entry;
	}
}

int interface_jtag_execute_queue(void)
{
	static int reentry;

	assert(reentry == 0);
	reentry++;

	int retval = default_interface_jtag_execute_queue();
	if (retval == ERROR_OK) {
		struct jtag_callback_entry *entry;
		for (entry = jtag_callback_queue_head; entry; entry = entry->next) {
			retval = entry->callback(entry->data0, entry->data1, entry->data2, entry->data3);
			if (retval != ERROR_OK)
				break;
		}
	}

	jtag_command_queue_reset();
	jtag_callback_queue_reset();

	reentry--;

	return retval;
}

static int jtag_convert_to_callback4(jtag_callback_data_t data0,
		jtag_callback_data_t data1, jtag_callback_data_t data2, jtag_callback_data_t data3)
{
	((jtag_callback1_t)data1)(data0);
	return ERROR_OK;
}

static void interface_jtag_add_callback(jtag_callback1_t callback, jtag_callback_data_t data0)
{
	jtag_add_callback4(jtag_convert_to_callback4, data0, (jtag_callback_data_t)callback, 0, 0);
}

void jtag_add_callback(jtag_callback1_t f, jtag_callback_data_t data0)
{
	interface_jtag_add_callback(f, data0);
}

void jtag_add_callback4(jtag_callback_t f, jtag_callback_data_t data0,
		jtag_callback_data_t data1, jtag_callback_data_t data2,
		jtag_callback_data_t data3)
{
	interface_jtag_add_callback4(f, data0, data1, data2, data3);
}
