// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2022 by Daniel Anselmi                                  *
 *   danselmi@gmx.ch                                                       *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/jtag.h>
#include <jtag/adapter.h>
#include <helper/system.h>
#include <helper/log.h>

#include "pld.h"
#include "raw_bit.h"

#define BYPASS 0x3FF
#define USER0  0x00C
#define USER1  0x00E

enum intel_family_e {
	INTEL_CYCLONEIII,
	INTEL_CYCLONEIV,
	INTEL_CYCLONEV,
	INTEL_CYCLONE10,
	INTEL_ARRIAII,
	INTEL_UNKNOWN
};

struct intel_pld_device {
	struct jtag_tap *tap;
	unsigned int boundary_scan_length;
	int checkpos;
	enum intel_family_e family;
};

static int intel_check_config(struct intel_pld_device *intel_info)
{
	if (intel_info->boundary_scan_length == 0) {
		LOG_ERROR("unknown boundary scan length. Please specify with 'intel set_bscan'.");
			return ERROR_FAIL;
	}

	if (intel_info->checkpos >= 0 && (unsigned int)intel_info->checkpos >= intel_info->boundary_scan_length) {
		LOG_ERROR("checkpos has to be smaller than scan length %d < %u",
					intel_info->checkpos, intel_info->boundary_scan_length);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int intel_read_file(struct raw_bit_file *bit_file, const char *filename)
{
	if (!filename || !bit_file)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* check if binary .bin or ascii .bit/.hex */
	const char *file_ending_pos = strrchr(filename, '.');
	if (!file_ending_pos) {
		LOG_ERROR("Unable to detect filename suffix");
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	if (strcasecmp(file_ending_pos, ".rbf") == 0)
		return cpld_read_raw_bit_file(bit_file, filename);

	LOG_ERROR("Unable to detect filetype");
	return ERROR_PLD_FILE_LOAD_FAILED;
}

static int intel_set_instr(struct jtag_tap *tap, uint16_t new_instr)
{
	struct scan_field field;
	field.num_bits = tap->ir_length;
	void *t = calloc(DIV_ROUND_UP(field.num_bits, 8), 1);
	if (!t) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}
	field.out_value = t;
	buf_set_u32(t, 0, field.num_bits, new_instr);
	field.in_value = NULL;
	jtag_add_ir_scan(tap, &field, TAP_IDLE);
	free(t);
	return ERROR_OK;
}


static int intel_load(struct pld_device *pld_device, const char *filename)
{
	unsigned int speed = adapter_get_speed_khz();
	if (speed < 1)
		speed = 1;

	unsigned int cycles = DIV_ROUND_UP(speed, 200);
	if (cycles < 1)
		cycles = 1;

	if (!pld_device || !pld_device->driver_priv)
		return ERROR_FAIL;

	struct intel_pld_device *intel_info = pld_device->driver_priv;
	if (!intel_info || !intel_info->tap)
		return ERROR_FAIL;
	struct jtag_tap *tap = intel_info->tap;

	int retval = intel_check_config(intel_info);
	if (retval != ERROR_OK)
		return retval;

	struct raw_bit_file bit_file;
	retval = intel_read_file(&bit_file, filename);
	if (retval != ERROR_OK)
		return retval;

	retval = intel_set_instr(tap, 0x002);
	if (retval != ERROR_OK) {
		free(bit_file.data);
		return retval;
	}
	jtag_add_runtest(speed, TAP_IDLE);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		free(bit_file.data);
		return retval;
	}

	/* shift in the bitstream */
	struct scan_field field;
	field.num_bits = bit_file.length * 8;
	field.out_value = bit_file.data;
	field.in_value = NULL;

	jtag_add_dr_scan(tap, 1, &field, TAP_DRPAUSE);
	retval = jtag_execute_queue();
	free(bit_file.data);
	if (retval != ERROR_OK)
		return retval;

	retval = intel_set_instr(tap, 0x004);
	if (retval != ERROR_OK)
		return retval;
	jtag_add_runtest(cycles, TAP_IDLE);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	if (intel_info->boundary_scan_length != 0) {
		uint8_t *buf = calloc(DIV_ROUND_UP(intel_info->boundary_scan_length, 8), 1);
		if (!buf) {
			LOG_ERROR("Out of memory");
			return ERROR_FAIL;
		}

		field.num_bits = intel_info->boundary_scan_length;
		field.out_value = buf;
		field.in_value = buf;
		jtag_add_dr_scan(tap, 1, &field, TAP_DRPAUSE);
		retval = jtag_execute_queue();
		if (retval != ERROR_OK) {
			free(buf);
			return retval;
		}

		if (intel_info->checkpos != -1)
			retval = ((buf[intel_info->checkpos / 8] & (1 << (intel_info->checkpos % 8)))) ?
					ERROR_OK : ERROR_FAIL;
		free(buf);
		if (retval != ERROR_OK) {
			LOG_ERROR("Check failed");
			return ERROR_FAIL;
		}
	} else {
		LOG_INFO("unable to check. Please specify with position 'intel set_check_pos'.");
	}

	retval = intel_set_instr(tap, 0x003);
	if (retval != ERROR_OK)
		return retval;
	switch (intel_info->family) {
	case INTEL_CYCLONEIII:
	case INTEL_CYCLONEIV:
		jtag_add_runtest(5 * speed + 512, TAP_IDLE);
		break;
	case INTEL_CYCLONEV:
		jtag_add_runtest(5 * speed + 512, TAP_IDLE);
		break;
	case INTEL_CYCLONE10:
		jtag_add_runtest(DIV_ROUND_UP(512ul * speed, 125ul) + 512, TAP_IDLE);
		break;
	case INTEL_ARRIAII:
		jtag_add_runtest(DIV_ROUND_UP(64ul * speed, 125ul) + 512, TAP_IDLE);
		break;
	case INTEL_UNKNOWN:
		LOG_ERROR("unknown family");
		return ERROR_FAIL;
	}

	retval = intel_set_instr(tap, BYPASS);
	if (retval != ERROR_OK)
		return retval;
	jtag_add_runtest(speed, TAP_IDLE);
	return jtag_execute_queue();
}

static int intel_get_ipdbg_hub(int user_num, struct pld_device *pld_device, struct pld_ipdbg_hub *hub)
{
	if (!pld_device)
		return ERROR_FAIL;

	struct intel_pld_device *pld_device_info = pld_device->driver_priv;

	if (!pld_device_info || !pld_device_info->tap)
		return ERROR_FAIL;

	hub->tap = pld_device_info->tap;

	if (user_num == 0) {
		hub->user_ir_code = USER0;
	} else if (user_num == 1) {
		hub->user_ir_code = USER1;
	} else {
		LOG_ERROR("intel devices only have user register 0 & 1");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int intel_get_jtagspi_userircode(struct pld_device *pld_device, unsigned int *ir)
{
	*ir = USER1;
	return ERROR_OK;
}

COMMAND_HANDLER(intel_set_bscan_command_handler)
{
	unsigned int boundary_scan_length;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct pld_device *pld_device = get_pld_device_by_name_or_numstr(CMD_ARGV[0]);
	if (!pld_device) {
		command_print(CMD, "pld device '#%s' is out of bounds or unknown", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[1], boundary_scan_length);

	struct intel_pld_device *intel_info = pld_device->driver_priv;

	if (!intel_info)
		return ERROR_FAIL;

	intel_info->boundary_scan_length = boundary_scan_length;

	return  ERROR_OK;
}

COMMAND_HANDLER(intel_set_check_pos_command_handler)
{
	int checkpos;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct pld_device *pld_device = get_pld_device_by_name_or_numstr(CMD_ARGV[0]);
	if (!pld_device) {
		command_print(CMD, "pld device '#%s' is out of bounds or unknown", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	COMMAND_PARSE_NUMBER(int, CMD_ARGV[1], checkpos);

	struct intel_pld_device *intel_info = pld_device->driver_priv;

	if (!intel_info)
		return ERROR_FAIL;

	intel_info->checkpos = checkpos;

	return ERROR_OK;
}

PLD_CREATE_COMMAND_HANDLER(intel_pld_create_command)
{
	if (CMD_ARGC != 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (strcmp(CMD_ARGV[2], "-chain-position") != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct jtag_tap *tap = jtag_tap_by_string(CMD_ARGV[3]);
	if (!tap) {
		command_print(CMD, "Tap: %s does not exist", CMD_ARGV[3]);
		return ERROR_FAIL;
	}

	enum intel_family_e family = INTEL_UNKNOWN;

	if (strcmp(CMD_ARGV[4], "-family") != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (strcmp(CMD_ARGV[5], "cycloneiii") == 0) {
		family = INTEL_CYCLONEIII;
	} else if (strcmp(CMD_ARGV[5], "cycloneiv") == 0) {
		family = INTEL_CYCLONEIV;
	} else if (strcmp(CMD_ARGV[5], "cyclonev") == 0) {
		family = INTEL_CYCLONEV;
	} else if (strcmp(CMD_ARGV[5], "cyclone10") == 0) {
		family = INTEL_CYCLONE10;
	} else if (strcmp(CMD_ARGV[5], "arriaii") == 0) {
		family = INTEL_ARRIAII;
	} else {
		command_print(CMD, "unknown family");
		return ERROR_FAIL;
	}

	struct intel_pld_device *intel_info = malloc(sizeof(struct intel_pld_device));
	if (!intel_info) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	intel_info->tap = tap;
	intel_info->boundary_scan_length = 0;
	intel_info->checkpos = -1;
	intel_info->family = family;

	pld->driver_priv = intel_info;

	return ERROR_OK;
}

static const struct command_registration intel_exec_command_handlers[] = {
	{
		.name = "set_bscan",
		.mode = COMMAND_ANY,
		.handler = intel_set_bscan_command_handler,
		.help = "set boundary scan register length of FPGA",
		.usage = "pld_name len",
	}, {
		.name = "set_check_pos",
		.mode = COMMAND_ANY,
		.handler = intel_set_check_pos_command_handler,
		.help = "set check_pos of FPGA",
		.usage = "pld_name pos",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration intel_command_handler[] = {
	{
		.name = "intel",
		.mode = COMMAND_ANY,
		.help = "intel specific commands",
		.usage = "",
		.chain = intel_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct pld_driver intel_pld = {
	.name = "intel",
	.commands = intel_command_handler,
	.pld_create_command = &intel_pld_create_command,
	.load = &intel_load,
	.get_ipdbg_hub = intel_get_ipdbg_hub,
	.get_jtagspi_userircode = intel_get_jtagspi_userircode,
};
