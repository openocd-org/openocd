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

struct intel_device_parameters_elem {
	uint32_t id;
	unsigned int boundary_scan_length;
	int checkpos;
	enum intel_family_e family;
};

static const struct intel_device_parameters_elem intel_device_parameters[] = {
	{0x020f10dd,  603,  226, INTEL_CYCLONEIII}, /* EP3C5 EP3C10 */
	{0x020f20dd, 1080,  409, INTEL_CYCLONEIII}, /* EP3C16 */
	{0x020f30dd,  732,  286, INTEL_CYCLONEIII}, /* EP3C25 */
	{0x020f40dd, 1632,  604, INTEL_CYCLONEIII}, /* EP3C40 */
	{0x020f50dd, 1164,  442, INTEL_CYCLONEIII}, /* EP3C55 */
	{0x020f60dd, 1314,  502, INTEL_CYCLONEIII}, /* EP3C80 */
	{0x020f70dd, 1620,  613, INTEL_CYCLONEIII}, /* EP3C120*/
	{0x027010dd, 1314,  226, INTEL_CYCLONEIII}, /* EP3CLS70 */
	{0x027000dd, 1314,  226, INTEL_CYCLONEIII}, /* EP3CLS100 */
	{0x027030dd, 1314,  409, INTEL_CYCLONEIII}, /* EP3CLS150 */
	{0x027020dd, 1314,  409, INTEL_CYCLONEIII}, /* EP3CLS200 */

	{0x020f10dd,  603,  226, INTEL_CYCLONEIV}, /* EP4CE6 EP4CE10 */
	{0x020f20dd, 1080,  409, INTEL_CYCLONEIV}, /* EP4CE15 */
	{0x020f30dd,  732,  286, INTEL_CYCLONEIV}, /* EP4CE22 */
	{0x020f40dd, 1632,  604, INTEL_CYCLONEIV}, /* EP4CE30 EP4CE40 */
	{0x020f50dd, 1164,  442, INTEL_CYCLONEIV}, /* EP4CE55 */
	{0x020f60dd, 1314,  502, INTEL_CYCLONEIV}, /* EP4CE75 */
	{0x020f70dd, 1620,  613, INTEL_CYCLONEIV}, /* EP4CE115 */
	{0x028010dd,  260,  229, INTEL_CYCLONEIV}, /* EP4CGX15 */
	{0x028120dd,  494,  463, INTEL_CYCLONEIV}, /* EP4CGX22 */
	{0x028020dd,  494,  463, INTEL_CYCLONEIV}, /* EP4CGX30 */
	{0x028230dd, 1006,  943, INTEL_CYCLONEIV}, /* EP4CGX30 */
	{0x028130dd, 1006,  943, INTEL_CYCLONEIV}, /* EP4CGX50 */
	{0x028030dd, 1006,  943, INTEL_CYCLONEIV}, /* EP4CGX75 */
	{0x028140dd, 1495, 1438, INTEL_CYCLONEIV}, /* EP4CGX110 */
	{0x028040dd, 1495, 1438, INTEL_CYCLONEIV}, /* EP4CGX150 */

	{0x02b150dd,  864, 163, INTEL_CYCLONEV}, /* 5CEBA2F23 5CEBA2F17 5CEFA2M13 5CEFA2F23 5CEBA2U15 5CEFA2U19 5CEBA2U19 */
	{0x02d020dd, 1485,  19, INTEL_CYCLONEV}, /* 5CSXFC6D6F31 5CSTFD6D5F31 5CSEBA6U23 5CSEMA6U23 5CSEBA6U19 5CSEBA6U23
											5CSEBA6U19 5CSEMA6F31 5CSXFC6C6U23 */
	{0x02b040dd, 1728,  -1, INTEL_CYCLONEV}, /* 5CGXFC9EF35 5CGXBC9AU19 5CGXBC9CF23 5CGTFD9CF23 5CGXFC9AU19 5CGXFC9CF23
											5CGXFC9EF31 5CGXFC9DF27 5CGXBC9DF27 5CGXBC9EF31 5CGTFD9EF31 5CGTFD9EF35
											5CGTFD9AU19 5CGXBC9EF35 5CGTFD9DF27 */
	{0x02b050dd,  864, 163, INTEL_CYCLONEV}, /* 5CEFA4U19 5CEFA4F23 5CEFA4M13 5CEBA4F17 5CEBA4U15 5CEBA4U19 5CEBA4F23 */
	{0x02b030dd, 1488,  19, INTEL_CYCLONEV}, /* 5CGXBC7CU19 5CGTFD7CU19 5CGTFD7DF27 5CGXFC7BM15 5CGXFC7DF27 5CGXFC7DF31
											5CGTFD7CF23 5CGXBC7CF23 5CGXBC7DF31 5CGTFD7BM15 5CGXFC7CU19 5CGTFD7DF31
											5CGXBC7BM15 5CGXFC7CF23 5CGXBC7DF27 */
	{0x02d120dd, 1485,  -1, INTEL_CYCLONEV}, /* 5CSEBA5U23 5CSEBA5U23 5CSTFD5D5F31 5CSEBA5U19 5CSXFC5D6F31 5CSEMA5U23
											5CSEMA5F31 5CSXFC5C6U23 5CSEBA5U19 */
	{0x02b220dd, 1104,  19, INTEL_CYCLONEV}, /* 5CEBA5U19 5CEFA5U19 5CEFA5M13 5CEBA5F23 5CEFA5F23 */
	{0x02b020dd, 1104,  19, INTEL_CYCLONEV}, /* 5CGXBC5CU19 5CGXFC5F6M11 5CGXFC5CM13 5CGTFD5CF23 5CGXBC5CF23 5CGTFD5CF27
											5CGTFD5F5M11 5CGXFC5CF27 5CGXFC5CU19 5CGTFD5CM13 5CGXFC5CF23 5CGXBC5CF27
											5CGTFD5CU19 */
	{0x02d010dd, 1197,  -1, INTEL_CYCLONEV}, /* 5CSEBA4U23 5CSXFC4C6U23 5CSEMA4U23 5CSEBA4U23 5CSEBA4U19 5CSEBA4U19
											5CSXFC2C6U23 */
	{0x02b120dd, 1104,  19, INTEL_CYCLONEV}, /* 5CGXFC4CM13 5CGXFC4CU19 5CGXFC4F6M11 5CGXBC4CU19 5CGXFC4CF27 5CGXBC4CF23
											5CGXBC4CF27 5CGXFC4CF23 */
	{0x02b140dd, 1728,  -1, INTEL_CYCLONEV}, /* 5CEFA9F31 5CEBA9F31 5CEFA9F27 5CEBA9U19 5CEBA9F27 5CEFA9U19 5CEBA9F23
											5CEFA9F23 */
	{0x02b010dd,  720,  19, INTEL_CYCLONEV}, /* 5CGXFC3U15 5CGXBC3U15 5CGXFC3F23 5CGXFC3U19 5CGXBC3U19 5CGXBC3F23 */
	{0x02b130dd, 1488,  19, INTEL_CYCLONEV}, /* 5CEFA7F31 5CEBA7F27 5CEBA7M15 5CEFA7U19 5CEBA7F23 5CEFA7F23 5CEFA7F27
											5CEFA7M15 5CEBA7U19 5CEBA7F31 */
	{0x02d110dd, 1197,  -1, INTEL_CYCLONEV}, /* 5CSEBA2U23 5CSEMA2U23 5CSEBA2U23 5CSEBA2U19 5CSEBA2U19 */

	{0x020f10dd,  603, 226, INTEL_CYCLONE10}, /* 10CL006E144 10CL006U256 10CL010M164 10CL010U256 10CL010E144 */
	{0x020f20dd, 1080, 409, INTEL_CYCLONE10}, /* 10CL016U256 10CL016E144 10CL016U484 10CL016F484 10CL016M164 */
	{0x020f30dd,  732, 286, INTEL_CYCLONE10}, /* 10CL025U256 10CL025E144 */
	{0x020f40dd, 1632, 604, INTEL_CYCLONE10}, /* 10CL040F484 10CL040U484 */
	{0x020f50dd, 1164, 442, INTEL_CYCLONE10}, /* 10CL055F484 10CL055U484 */
	{0x020f60dd, 1314, 502, INTEL_CYCLONE10}, /* 10CL080F484 10CL080F780 10CL080U484 */
	{0x020f70dd, 1620, 613, INTEL_CYCLONE10}, /* 10CL120F484 10CL120F780 */

	{0x02e120dd, 1339, -1, INTEL_CYCLONE10}, /* 10CX085U484 10CX085F672 */
	{0x02e320dd, 1339, -1, INTEL_CYCLONE10}, /* 10CX105F780 10CX105U484 10CX105F672 */
	{0x02e720dd, 1339, -1, INTEL_CYCLONE10}, /* 10CX150F672 10CX150F780 10CX150U484 */
	{0x02ef20dd, 1339, -1, INTEL_CYCLONE10}, /* 10CX220F672 10CX220F780 10CX220U484 */

	{0x025120dd, 1227, 1174, INTEL_ARRIAII}, /* EP2AGX45 */
	{0x025020dd, 1227,   -1, INTEL_ARRIAII}, /* EP2AGX65 */
	{0x025130dd, 1467,   -1, INTEL_ARRIAII}, /* EP2AGX95 */
	{0x025030dd, 1467,   -1, INTEL_ARRIAII}, /* EP2AGX125 */
	{0x025140dd, 1971,   -1, INTEL_ARRIAII}, /* EP2AGX190 */
	{0x025040dd, 1971,   -1, INTEL_ARRIAII}, /* EP2AGX260 */
	{0x024810dd, 2274,   -1, INTEL_ARRIAII}, /* EP2AGZ225 */
	{0x0240a0dd, 2682,   -1, INTEL_ARRIAII}, /* EP2AGZ300 */
	{0x024820dd, 2682,   -1, INTEL_ARRIAII}, /* EP2AGZ350 */
};

static int intel_fill_device_parameters(struct intel_pld_device *intel_info)
{
	for (size_t i = 0; i < ARRAY_SIZE(intel_device_parameters); ++i) {
		if (intel_device_parameters[i].id == intel_info->tap->idcode &&
			intel_info->family == intel_device_parameters[i].family) {
			if (intel_info->boundary_scan_length == 0)
				intel_info->boundary_scan_length = intel_device_parameters[i].boundary_scan_length;

			if (intel_info->checkpos == -1)
				intel_info->checkpos = intel_device_parameters[i].checkpos;

			return ERROR_OK;
		}
	}

	return ERROR_FAIL;
}

static int intel_check_for_unique_id(struct intel_pld_device *intel_info)
{
	int found = 0;
	for (size_t i = 0; i < ARRAY_SIZE(intel_device_parameters); ++i) {
		if (intel_device_parameters[i].id == intel_info->tap->idcode) {
			++found;
			intel_info->family = intel_device_parameters[i].family;
		}
	}

	return (found == 1) ? ERROR_OK : ERROR_FAIL;
}

static int intel_check_config(struct intel_pld_device *intel_info)
{
	if (!intel_info->tap->hasidcode) {
		LOG_ERROR("no IDCODE");
		return ERROR_FAIL;
	}

	if (intel_info->family == INTEL_UNKNOWN) {
		if (intel_check_for_unique_id(intel_info) != ERROR_OK) {
			LOG_ERROR("id is ambiguous, please specify family");
			return ERROR_FAIL;
		}
	}

	if (intel_info->boundary_scan_length == 0 || intel_info->checkpos == -1) {
		int ret = intel_fill_device_parameters(intel_info);
		if (ret != ERROR_OK)
			return ret;
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
	if (CMD_ARGC != 4 && CMD_ARGC != 6)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (strcmp(CMD_ARGV[2], "-chain-position") != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct jtag_tap *tap = jtag_tap_by_string(CMD_ARGV[3]);
	if (!tap) {
		command_print(CMD, "Tap: %s does not exist", CMD_ARGV[3]);
		return ERROR_FAIL;
	}

	enum intel_family_e family = INTEL_UNKNOWN;
	if (CMD_ARGC == 6) {
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
