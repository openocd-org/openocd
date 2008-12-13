/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
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

#include "virtex2.h"

#include "pld.h"
#include "xilinx_bit.h"
#include "command.h"
#include "log.h"
#include "jtag.h"

#include <stdlib.h>

int virtex2_register_commands(struct command_context_s *cmd_ctx);
int virtex2_pld_device_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct pld_device_s *pld_device);
int virtex2_load(struct pld_device_s *pld_device, char *filename);

pld_driver_t virtex2_pld =
{
	.name = "virtex2",
	.register_commands = virtex2_register_commands,
	.pld_device_command = virtex2_pld_device_command,
	.load = virtex2_load,
};

int virtex2_set_instr(jtag_tap_t *tap, u32 new_instr)
{
	if (tap==NULL)
		return ERROR_FAIL;

	if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) != new_instr)
	{
		scan_field_t field;

		field.tap = tap;
		field.num_bits = tap->ir_length;
		field.out_value = calloc(CEIL(field.num_bits, 8), 1);
		buf_set_u32(field.out_value, 0, field.num_bits, new_instr);
		field.out_mask = NULL;
		field.in_value = NULL;
		field.in_check_value = NULL;
		field.in_check_mask = NULL;
		field.in_handler = NULL;
		field.in_handler_priv = NULL;

		jtag_add_ir_scan(1, &field, TAP_IDLE);

		free(field.out_value);
	}

	return ERROR_OK;
}

int virtex2_send_32(struct pld_device_s *pld_device, int num_words, u32 *words)
{
	virtex2_pld_device_t *virtex2_info = pld_device->driver_priv;
	scan_field_t scan_field;
	u8 *values;
	int i;

	values = malloc(num_words * 4);

	scan_field.tap = virtex2_info->tap;
	scan_field.num_bits = num_words * 32;
	scan_field.out_value = values;
	scan_field.out_mask = NULL;
	scan_field.in_value = NULL;
	scan_field.in_check_value = NULL;
	scan_field.in_check_mask = NULL;
	scan_field.in_handler = NULL;
	scan_field.in_handler_priv = NULL;

	for (i = 0; i < num_words; i++)
		buf_set_u32(values + 4 * i, 0, 32, flip_u32(*words++, 32));

	virtex2_set_instr(virtex2_info->tap, 0x5); /* CFG_IN */

	jtag_add_dr_scan(1, &scan_field, TAP_DRPAUSE);

	free(values);

	return ERROR_OK;
}

int virtex2_jtag_buf_to_u32(u8 *in_buf, void *priv, struct scan_field_s *field)
{
	u32 *dest = priv;
	*dest = flip_u32(le_to_h_u32(in_buf), 32);
	return ERROR_OK;
}

int virtex2_receive_32(struct pld_device_s *pld_device, int num_words, u32 *words)
{
	virtex2_pld_device_t *virtex2_info = pld_device->driver_priv;
	scan_field_t scan_field;

	scan_field.tap = virtex2_info->tap;
	scan_field.num_bits = 32;
	scan_field.out_value = NULL;
	scan_field.out_mask = NULL;
	scan_field.in_value = NULL;
	scan_field.in_check_value = NULL;
	scan_field.in_check_mask = NULL;
	scan_field.in_handler = virtex2_jtag_buf_to_u32;

	virtex2_set_instr(virtex2_info->tap, 0x4); /* CFG_OUT */

	while (num_words--)
	{
		scan_field.in_handler_priv = words++;
		jtag_add_dr_scan(1, &scan_field, TAP_DRPAUSE);
	}

	return ERROR_OK;
}

int virtex2_read_stat(struct pld_device_s *pld_device, u32 *status)
{
	u32 data[5];

	jtag_add_tlr();

	data[0] = 0xaa995566; /* synch word */
	data[1] = 0x2800E001; /* Type 1, read, address 7, 1 word */
	data[2] = 0x20000000; /* NOOP (Type 1, read, address 0, 0 words */
	data[3] = 0x20000000; /* NOOP */
	data[4] = 0x20000000; /* NOOP */
	virtex2_send_32(pld_device, 5, data);

	virtex2_receive_32(pld_device, 1, status);

	jtag_execute_queue();

	LOG_DEBUG("status: 0x%8.8x", *status);

	return ERROR_OK;
}

int virtex2_load(struct pld_device_s *pld_device, char *filename)
{
	virtex2_pld_device_t *virtex2_info = pld_device->driver_priv;
	xilinx_bit_file_t bit_file;
	int retval;
	int i;

	scan_field_t field;

	field.tap = virtex2_info->tap;
	field.out_mask = NULL;
	field.in_value = NULL;
	field.in_check_value = NULL;
	field.in_check_mask = NULL;
	field.in_handler = NULL;
	field.in_handler_priv = NULL;

	if ((retval = xilinx_read_bit_file(&bit_file, filename)) != ERROR_OK)
		return retval;

	jtag_add_end_state(TAP_IDLE);
	virtex2_set_instr(virtex2_info->tap, 0xb); /* JPROG_B */
	jtag_execute_queue();
	jtag_add_sleep(1000);

	virtex2_set_instr(virtex2_info->tap, 0x5); /* CFG_IN */
	jtag_execute_queue();

	for (i = 0; i < bit_file.length; i++)
		bit_file.data[i] = flip_u32(bit_file.data[i], 8);

	field.num_bits = bit_file.length * 8;
	field.out_value = bit_file.data;

	jtag_add_dr_scan(1, &field, TAP_DRPAUSE);
	jtag_execute_queue();

	jtag_add_tlr();

	jtag_add_end_state(TAP_IDLE);
	virtex2_set_instr(virtex2_info->tap, 0xc); /* JSTART */
	jtag_add_runtest(13, TAP_IDLE);
	virtex2_set_instr(virtex2_info->tap, 0x3f); /* BYPASS */
	virtex2_set_instr(virtex2_info->tap, 0x3f); /* BYPASS */
	virtex2_set_instr(virtex2_info->tap, 0xc); /* JSTART */
	jtag_add_runtest(13, TAP_IDLE);
	virtex2_set_instr(virtex2_info->tap, 0x3f); /* BYPASS */
	jtag_execute_queue();

	return ERROR_OK;
}

int virtex2_handle_read_stat_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	pld_device_t *device;
	virtex2_pld_device_t *virtex2_info;
	u32 status;

	if (argc < 1)
	{
		command_print(cmd_ctx, "usage: virtex2 read_stat <num>");
		return ERROR_OK;
	}

	device = get_pld_device_by_num(strtoul(args[0], NULL, 0));
	if (!device)
	{
		command_print(cmd_ctx, "pld device '#%s' is out of bounds", args[0]);
		return ERROR_OK;
	}

	virtex2_info = device->driver_priv;

	virtex2_read_stat(device, &status);

	command_print(cmd_ctx, "virtex2 status register: 0x%8.8x", status);

	return ERROR_OK;
}

int virtex2_register_commands(struct command_context_s *cmd_ctx)
{
	command_t *virtex2_cmd = register_command(cmd_ctx, NULL, "virtex2", NULL, COMMAND_ANY, "virtex2 specific commands");

	register_command(cmd_ctx, virtex2_cmd, "read_stat", virtex2_handle_read_stat_command, COMMAND_EXEC,
					 "read Virtex-II status register");

	return ERROR_OK;
}

int virtex2_pld_device_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc, struct pld_device_s *pld_device)
{
	jtag_tap_t *tap;

	virtex2_pld_device_t *virtex2_info;

	if (argc < 2)
	{
		LOG_WARNING("incomplete pld device 'virtex2' configuration");
		return ERROR_PLD_DEVICE_INVALID;
	}

	tap = jtag_TapByString( args[1] );
	if( tap == NULL ){
		command_print( cmd_ctx, "Tap: %s does not exist", args[1] );
		return ERROR_OK;
	}

	virtex2_info = malloc(sizeof(virtex2_pld_device_t));
	pld_device->driver_priv = virtex2_info;
	virtex2_info->tap = tap;

	return ERROR_OK;
}
