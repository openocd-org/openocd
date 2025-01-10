// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arm_jtag.h"

#if 0
#define _ARM_JTAG_SCAN_N_CHECK_
#endif

int arm_jtag_set_instr_inner(struct jtag_tap *tap,
		uint32_t new_instr, void *no_verify_capture, enum tap_state end_state)
{
	struct scan_field field;
	uint8_t t[4] = { 0 };

	field.num_bits = tap->ir_length;
	field.out_value = t;
	buf_set_u32(t, 0, field.num_bits, new_instr);
	field.in_value = NULL;

	if (!no_verify_capture)
		jtag_add_ir_scan(tap, &field, end_state);
	else {
		/* FIX!!!! this is a kludge!!! arm926ejs.c should reimplement this arm_jtag_set_instr to
		 * have special verification code.
		 */
		jtag_add_ir_scan_noverify(tap, &field, end_state);
	}

	return ERROR_OK;
}

int arm_jtag_scann_inner(struct arm_jtag *jtag_info, uint32_t new_scan_chain, enum tap_state end_state)
{
	int retval = ERROR_OK;

	uint8_t out_value[4] = { 0 };
	buf_set_u32(out_value, 0, jtag_info->scann_size, new_scan_chain);
	struct scan_field field = { .num_bits = jtag_info->scann_size, .out_value = out_value, };

	retval = arm_jtag_set_instr(jtag_info->tap, jtag_info->scann_instr, NULL, end_state);
	if (retval != ERROR_OK)
		return retval;

	jtag_add_dr_scan(jtag_info->tap,
			1,
			&field,
			end_state);

	jtag_info->cur_scan_chain = new_scan_chain;

	return retval;
}

static int arm_jtag_reset_callback(enum jtag_event event, void *priv)
{
	struct arm_jtag *jtag_info = priv;

	if (event == JTAG_TRST_ASSERTED)
		jtag_info->cur_scan_chain = 0;

	return ERROR_OK;
}

int arm_jtag_setup_connection(struct arm_jtag *jtag_info)
{
	jtag_info->scann_instr = 0x2;
	jtag_info->cur_scan_chain = 0;
	jtag_info->intest_instr = 0xc;

	return jtag_register_event_callback(arm_jtag_reset_callback, jtag_info);
}

int arm_jtag_close_connection(struct arm_jtag *jtag_info)
{
	return jtag_unregister_event_callback(arm_jtag_reset_callback, jtag_info);
}
