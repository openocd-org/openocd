// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "virtex2.h"
#include "xilinx_bit.h"
#include "pld.h"

static const struct virtex2_command_set virtex2_default_commands = {
	.cfg_out   = 0x04,
	.cfg_in    = 0x05,
	.jprog_b   = 0x0b,
	.jstart    = 0x0c,
	.jshutdown = 0x0d,
	.bypass    = 0x3f,
	.user      = {0x02, 0x03},
	.num_user  = 2, /* virtex II has only 2 user instructions */
};

static int virtex2_set_instr(struct jtag_tap *tap, uint64_t new_instr)
{
	if (!tap)
		return ERROR_FAIL;

	if (buf_get_u64(tap->cur_instr, 0, tap->ir_length) != new_instr) {
		struct scan_field field;

		field.num_bits = tap->ir_length;
		void *t = calloc(DIV_ROUND_UP(field.num_bits, 8), 1);
		if (!t) {
			LOG_ERROR("Out of memory");
			return ERROR_FAIL;
		}
		field.out_value = t;
		buf_set_u64(t, 0, field.num_bits, new_instr);
		field.in_value = NULL;

		jtag_add_ir_scan(tap, &field, TAP_IDLE);

		free(t);
	}

	return ERROR_OK;
}

static int virtex2_send_32(struct pld_device *pld_device,
	int num_words, uint32_t *words)
{
	struct virtex2_pld_device *virtex2_info = pld_device->driver_priv;
	struct scan_field scan_field;
	uint8_t *values;
	int i;

	values = malloc(num_words * 4);
	if (!values) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	scan_field.num_bits = num_words * 32;
	scan_field.out_value = values;
	scan_field.in_value = NULL;

	for (i = 0; i < num_words; i++)
		buf_set_u32(values + 4 * i, 0, 32, flip_u32(*words++, 32));

	int retval = virtex2_set_instr(virtex2_info->tap, virtex2_info->command_set.cfg_in);
	if (retval != ERROR_OK) {
		free(values);
		return retval;
	}

	jtag_add_dr_scan(virtex2_info->tap, 1, &scan_field, TAP_DRPAUSE);

	free(values);

	return ERROR_OK;
}

static inline void virtexflip32(jtag_callback_data_t arg)
{
	uint8_t *in = (uint8_t *)arg;
	*((uint32_t *)arg) = flip_u32(le_to_h_u32(in), 32);
}

static int virtex2_receive_32(struct pld_device *pld_device,
	int num_words, uint32_t *words)
{
	struct virtex2_pld_device *virtex2_info = pld_device->driver_priv;
	struct scan_field scan_field;

	scan_field.num_bits = 32;
	scan_field.out_value = NULL;
	scan_field.in_value = NULL;

	int retval = virtex2_set_instr(virtex2_info->tap, virtex2_info->command_set.cfg_out);
	if (retval != ERROR_OK)
		return retval;

	while (num_words--) {
		scan_field.in_value = (uint8_t *)words;

		jtag_add_dr_scan(virtex2_info->tap, 1, &scan_field, TAP_DRPAUSE);

		jtag_add_callback(virtexflip32, (jtag_callback_data_t)words);

		words++;
	}

	return ERROR_OK;
}

static int virtex2_read_stat(struct pld_device *pld_device, uint32_t *status)
{
	uint32_t data[5];

	jtag_add_tlr();

	data[0] = 0xaa995566;	/* synch word */
	data[1] = 0x2800E001;	/* Type 1, read, address 7, 1 word */
	data[2] = 0x20000000;	/* NOOP (Type 1, read, address 0, 0 words */
	data[3] = 0x20000000;	/* NOOP */
	data[4] = 0x20000000;	/* NOOP */
	int retval = virtex2_send_32(pld_device, 5, data);
	if (retval != ERROR_OK)
		return retval;

	retval = virtex2_receive_32(pld_device, 1, status);
	if (retval != ERROR_OK)
		return retval;

	retval = jtag_execute_queue();
	if (retval == ERROR_OK)
		LOG_DEBUG("status: 0x%8.8" PRIx32, *status);

	return retval;
}

static int virtex2_program(struct pld_device *pld_device)
{
	struct virtex2_pld_device *virtex2_info = pld_device->driver_priv;
	if (!virtex2_info)
		return ERROR_FAIL;

	int retval = virtex2_set_instr(virtex2_info->tap, virtex2_info->command_set.jshutdown);
	if (retval != ERROR_OK)
		return retval;

	retval = virtex2_set_instr(virtex2_info->tap, virtex2_info->command_set.jprog_b);
	if (retval != ERROR_OK)
		return retval;

	jtag_add_runtest(62000, TAP_IDLE);
	if (!(virtex2_info->no_jstart)) {
		retval = virtex2_set_instr(virtex2_info->tap, virtex2_info->command_set.jstart);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = virtex2_set_instr(virtex2_info->tap, virtex2_info->command_set.bypass);
	if (retval != ERROR_OK)
		return retval;
	jtag_add_runtest(2000, TAP_IDLE);

	return jtag_execute_queue();
}

static int virtex2_load_prepare(struct pld_device *pld_device)
{
	struct virtex2_pld_device *virtex2_info = pld_device->driver_priv;
	int retval;

	retval = virtex2_set_instr(virtex2_info->tap, virtex2_info->command_set.jprog_b);
	if (retval != ERROR_OK)
		return retval;

	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;
	jtag_add_sleep(1000);

	retval = virtex2_set_instr(virtex2_info->tap, virtex2_info->command_set.cfg_in);
	if (retval != ERROR_OK)
		return retval;

	return jtag_execute_queue();
}

static int virtex2_load_cleanup(struct pld_device *pld_device)
{
	struct virtex2_pld_device *virtex2_info = pld_device->driver_priv;
	int retval;

	jtag_add_tlr();

	if (!(virtex2_info->no_jstart)) {
		retval = virtex2_set_instr(virtex2_info->tap, virtex2_info->command_set.jstart);
		if (retval != ERROR_OK)
			return retval;
	}
	jtag_add_runtest(13, TAP_IDLE);
	retval = virtex2_set_instr(virtex2_info->tap, virtex2_info->command_set.bypass);
	if (retval != ERROR_OK)
		return retval;
	retval = virtex2_set_instr(virtex2_info->tap, virtex2_info->command_set.bypass);
	if (retval != ERROR_OK)
		return retval;
	if (!(virtex2_info->no_jstart)) {
		retval = virtex2_set_instr(virtex2_info->tap, virtex2_info->command_set.jstart);
		if (retval != ERROR_OK)
			return retval;
	}
	jtag_add_runtest(13, TAP_IDLE);
	retval = virtex2_set_instr(virtex2_info->tap, virtex2_info->command_set.bypass);
	if (retval != ERROR_OK)
		return retval;

	return jtag_execute_queue();
}

static int virtex2_load(struct pld_device *pld_device, const char *filename)
{
	struct virtex2_pld_device *virtex2_info = pld_device->driver_priv;
	struct xilinx_bit_file bit_file;
	int retval;
	unsigned int i;
	struct scan_field field;

	field.in_value = NULL;

	retval = xilinx_read_bit_file(&bit_file, filename);
	if (retval != ERROR_OK)
		return retval;

	retval = virtex2_load_prepare(pld_device);
	if (retval != ERROR_OK) {
		xilinx_free_bit_file(&bit_file);
		return retval;
	}

	for (i = 0; i < bit_file.length; i++)
		bit_file.data[i] = flip_u32(bit_file.data[i], 8);

	field.num_bits = bit_file.length * 8;
	field.out_value = bit_file.data;

	jtag_add_dr_scan(virtex2_info->tap, 1, &field, TAP_DRPAUSE);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		xilinx_free_bit_file(&bit_file);
		return retval;
	}

	retval = virtex2_load_cleanup(pld_device);

	xilinx_free_bit_file(&bit_file);

	return retval;
}

COMMAND_HANDLER(virtex2_handle_refresh_command)
{
	struct pld_device *device;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	device = get_pld_device_by_name_or_numstr(CMD_ARGV[0]);
	if (!device) {
		command_print(CMD, "pld device '#%s' is out of bounds or unknown", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	return virtex2_program(device);
}

COMMAND_HANDLER(virtex2_handle_read_stat_command)
{
	struct pld_device *device;
	uint32_t status;

	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	device = get_pld_device_by_name_or_numstr(CMD_ARGV[0]);
	if (!device) {
		command_print(CMD, "pld device '#%s' is out of bounds or unknown", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	int retval = virtex2_read_stat(device, &status);
	if (retval != ERROR_OK) {
		command_print(CMD, "cannot read virtex2 status register");
		return retval;
	}

	command_print(CMD, "virtex2 status register: 0x%8.8" PRIx32, status);

	return ERROR_OK;
}

static int xilinx_get_ipdbg_hub(int user_num, struct pld_device *pld_device, struct pld_ipdbg_hub *hub)
{
	if (!pld_device)
		return ERROR_FAIL;

	struct virtex2_pld_device *pld_device_info = pld_device->driver_priv;

	if (!pld_device_info || !pld_device_info->tap)
		return ERROR_FAIL;

	hub->tap = pld_device_info->tap;
	if (user_num < 1 || (unsigned int)user_num > pld_device_info->command_set.num_user) {
		LOG_ERROR("device has only user register 1 to %d", pld_device_info->command_set.num_user);
		return ERROR_FAIL;
	}

	hub->user_ir_code = pld_device_info->command_set.user[user_num - 1];
	return ERROR_OK;
}

static int xilinx_get_jtagspi_userircode(struct pld_device *pld_device, unsigned int *ir)
{
	if (!pld_device || !pld_device->driver_priv)
		return ERROR_FAIL;
	struct virtex2_pld_device *pld_device_info = pld_device->driver_priv;

	if (pld_device_info->command_set.num_user < 1) {
		LOG_ERROR("code for command 'select user1' is unknown");
		return ERROR_FAIL;
	}

	*ir = pld_device_info->command_set.user[0];
	return ERROR_OK;
}

COMMAND_HANDLER(virtex2_handle_set_instuction_codes_command)
{
	if (CMD_ARGC < 6 || CMD_ARGC > (6 + VIRTEX2_MAX_USER_INSTRUCTIONS))
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct pld_device *device = get_pld_device_by_name(CMD_ARGV[0]);
	if (!device) {
		command_print(CMD, "pld device '#%s' is unknown", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	struct virtex2_pld_device *virtex2_info = device->driver_priv;
	if (!virtex2_info)
		return ERROR_FAIL;

	struct virtex2_command_set instr_codes;
	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[1], instr_codes.cfg_out);
	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[2], instr_codes.cfg_in);
	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[3], instr_codes.jprog_b);
	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[4], instr_codes.jstart);
	COMMAND_PARSE_NUMBER(u64, CMD_ARGV[5], instr_codes.jshutdown);
	instr_codes.bypass = 0xffffffffffffffffULL;

	unsigned int num_user = CMD_ARGC - 6;
	for (unsigned int i = 0; i < num_user; ++i)
		COMMAND_PARSE_NUMBER(u64, CMD_ARGV[6 + i], instr_codes.user[i]);
	instr_codes.num_user = num_user;

	virtex2_info->command_set = instr_codes;
	return ERROR_OK;
}

COMMAND_HANDLER(virtex2_handle_set_user_codes_command)
{
	if (CMD_ARGC < 2 || CMD_ARGC > (1 + VIRTEX2_MAX_USER_INSTRUCTIONS))
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct pld_device *device = get_pld_device_by_name(CMD_ARGV[0]);
	if (!device) {
		command_print(CMD, "pld device '#%s' is unknown", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	struct virtex2_pld_device *virtex2_info = device->driver_priv;
	if (!virtex2_info)
		return ERROR_FAIL;

	uint64_t user[VIRTEX2_MAX_USER_INSTRUCTIONS];
	unsigned int num_user = CMD_ARGC - 1;
	for (unsigned int i = 0; i < num_user; ++i)
		COMMAND_PARSE_NUMBER(u64, CMD_ARGV[1 + i], user[i]);
	virtex2_info->command_set.num_user = num_user;
	memcpy(virtex2_info->command_set.user, user, num_user * sizeof(uint64_t));
	return ERROR_OK;
}

PLD_CREATE_COMMAND_HANDLER(virtex2_pld_create_command)
{
	if (CMD_ARGC < 4)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (strcmp(CMD_ARGV[2], "-chain-position") != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct jtag_tap *tap = jtag_tap_by_string(CMD_ARGV[3]);
	if (!tap) {
		command_print(CMD, "Tap: %s does not exist", CMD_ARGV[3]);
		return ERROR_FAIL;
	}

	struct virtex2_pld_device *virtex2_info = malloc(sizeof(struct virtex2_pld_device));
	if (!virtex2_info) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}
	virtex2_info->tap = tap;
	virtex2_info->command_set = virtex2_default_commands;

	virtex2_info->no_jstart = 0;
	if (CMD_ARGC >= 5 && strcmp(CMD_ARGV[4], "-no_jstart") == 0)
		virtex2_info->no_jstart = 1;

	pld->driver_priv = virtex2_info;

	return ERROR_OK;
}

static const struct command_registration virtex2_exec_command_handlers[] = {
	{
		.name = "read_stat",
		.mode = COMMAND_EXEC,
		.handler = virtex2_handle_read_stat_command,
		.help = "read status register",
		.usage = "pld_name",
	}, {
		.name = "set_instr_codes",
		.mode = COMMAND_ANY,
		.handler = virtex2_handle_set_instuction_codes_command,
		.help = "set instructions codes used for loading the bitstream/refreshing/jtag-hub",
		.usage = "pld_name cfg_out cfg_in jprogb jstart jshutdown"
				 " [user1 [user2 [user3 [user4]]]]",
	}, {
		.name = "set_user_codes",
		.mode = COMMAND_ANY,
		.handler = virtex2_handle_set_user_codes_command,
		.help = "set instructions codes used for jtag-hub",
		.usage = "pld_name user1 [user2 [user3 [user4]]]",
	}, {
		.name = "refresh",
		.mode = COMMAND_EXEC,
		.handler = virtex2_handle_refresh_command,
		.help = "start loading of configuration (program)",
		.usage = "pld_name",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration virtex2_command_handler[] = {
	{
		.name = "virtex2",
		.mode = COMMAND_ANY,
		.help = "Virtex-II specific commands",
		.usage = "",
		.chain = virtex2_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct pld_driver virtex2_pld = {
	.name = "virtex2",
	.commands = virtex2_command_handler,
	.pld_create_command = &virtex2_pld_create_command,
	.load = &virtex2_load,
	.get_ipdbg_hub = xilinx_get_ipdbg_hub,
	.get_jtagspi_userircode = xilinx_get_jtagspi_userircode,
};
