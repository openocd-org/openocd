// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2022 by Daniel Anselmi                                  *
 *   danselmi@gmx.ch                                                       *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "lattice.h"
#include <jtag/jtag.h>
#include "pld.h"
#include "lattice_bit.h"
#include "ecp2_3.h"

#define PRELOAD              0x1C

struct lattice_devices_elem {
	uint32_t id;
	size_t preload_length;
	enum lattice_family_e family;
};

static const struct lattice_devices_elem lattice_devices[] = {
	{0x01270043,  654, LATTICE_ECP2 /* ecp2-6e */},
	{0x01271043,  643, LATTICE_ECP2 /* ecp2-12e */},
	{0x01272043,  827, LATTICE_ECP2 /* ecp2-20e */},
	{0x01274043, 1011, LATTICE_ECP2 /* ecp2-35e */},
	{0x01273043, 1219, LATTICE_ECP2 /* ecp2-50e */},
	{0x01275043,  654, LATTICE_ECP2 /* ecp2-70e */},
	{0x01279043,  680, LATTICE_ECP2 /* ecp2m20e */},
	{0x0127A043,  936, LATTICE_ECP2 /* ecp2m35e */},
	{0x0127B043, 1056, LATTICE_ECP2 /* ecp2m50e */},
	{0x0127C043, 1039, LATTICE_ECP2 /* ecp2m70e */},
	{0x0127D043, 1311, LATTICE_ECP2 /* ecp2m100e */},
	{0x01010043,  467, LATTICE_ECP3 /* ecp3 lae3-17ea & lfe3-17ea*/},
	{0x01012043,  675, LATTICE_ECP3 /* ecp3 lae3-35ea & lfe3-35ea*/},
	{0x01014043, 1077, LATTICE_ECP3 /* ecp3 lfe3-70ea & lfe3-70e & lfe3-95ea && lfe3-95e*/},
	{0x01015043, 1326, LATTICE_ECP3 /* ecp3 lfe3-150e*/},
};

int lattice_set_instr(struct jtag_tap *tap, uint8_t new_instr, tap_state_t endstate)
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
	jtag_add_ir_scan(tap, &field, endstate);
	free(t);
	return ERROR_OK;
}

static int lattice_check_device_family(struct lattice_pld_device *lattice_device)
{
	if (lattice_device->family != LATTICE_UNKNOWN && lattice_device->preload_length != 0)
		return ERROR_OK;

	if (!lattice_device->tap || !lattice_device->tap->hasidcode)
		return ERROR_FAIL;

	for (size_t i = 0; i < ARRAY_SIZE(lattice_devices); ++i) {
		if (lattice_devices[i].id == lattice_device->tap->idcode) {
			if (lattice_device->family == LATTICE_UNKNOWN)
				lattice_device->family = lattice_devices[i].family;
			if (lattice_device->preload_length == 0)
				lattice_device->preload_length = lattice_devices[i].preload_length;
			return ERROR_OK;
		}
	}
	LOG_ERROR("Unknown id! Specify family and preload-length manually.");
	return ERROR_FAIL;
}

int lattice_read_u32_register(struct jtag_tap *tap, uint8_t cmd, uint32_t *in_val,
							uint32_t out_val, bool do_idle)
{
	struct scan_field field;
	uint8_t buffer[4];

	int retval = lattice_set_instr(tap, cmd, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	if (do_idle) {
		jtag_add_runtest(2, TAP_IDLE);
		jtag_add_sleep(1000);
	}

	h_u32_to_le(buffer, out_val);
	field.num_bits = 32;
	field.out_value = buffer;
	field.in_value = buffer;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
	retval = jtag_execute_queue();
	if (retval == ERROR_OK)
		*in_val = le_to_h_u32(buffer);

	return retval;
}

int lattice_preload(struct lattice_pld_device *lattice_device)
{
	struct scan_field field;
	size_t sz_bytes = DIV_ROUND_UP(lattice_device->preload_length, 8);

	int retval = lattice_set_instr(lattice_device->tap, PRELOAD, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	uint8_t *buffer = malloc(sz_bytes);
	if (!buffer) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}
	memset(buffer, 0xff, sz_bytes);

	field.num_bits = lattice_device->preload_length;
	field.out_value = buffer;
	field.in_value = NULL;
	jtag_add_dr_scan(lattice_device->tap, 1, &field, TAP_IDLE);
	retval = jtag_execute_queue();
	free(buffer);
	return retval;
}

static int lattice_read_usercode(struct lattice_pld_device *lattice_device, uint32_t *usercode, uint32_t out)
{
	struct jtag_tap *tap = lattice_device->tap;
	if (!tap)
		return ERROR_FAIL;

	if (lattice_device->family == LATTICE_ECP2 || lattice_device->family == LATTICE_ECP3)
		return lattice_ecp2_3_read_usercode(tap, usercode, out);

	return ERROR_FAIL;
}

int lattice_verify_usercode(struct lattice_pld_device *lattice_device, uint32_t out,
						uint32_t expected, uint32_t mask)
{
	uint32_t usercode;

	int retval = lattice_read_usercode(lattice_device, &usercode, out);
	if (retval != ERROR_OK)
		return retval;

	if ((usercode & mask) != expected) {
		LOG_ERROR("verifying user code register failed got: 0x%08" PRIx32 " expected: 0x%08" PRIx32,
			usercode & mask, expected);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int lattice_write_usercode(struct lattice_pld_device *lattice_device, uint32_t usercode)
{
	if (lattice_device->family == LATTICE_ECP2 || lattice_device->family == LATTICE_ECP3)
		return lattice_ecp2_3_write_usercode(lattice_device, usercode);

	return ERROR_FAIL;
}

static int lattice_read_status_u32(struct lattice_pld_device *lattice_device, uint32_t *status,
								uint32_t out, bool do_idle)
{
	if (!lattice_device->tap)
		return ERROR_FAIL;

	if (lattice_device->family == LATTICE_ECP2 || lattice_device->family == LATTICE_ECP3)
		return lattice_ecp2_3_read_status(lattice_device->tap, status, out, do_idle);

	return ERROR_FAIL;
}

int lattice_verify_status_register_u32(struct lattice_pld_device *lattice_device, uint32_t out,
						uint32_t expected, uint32_t mask, bool do_idle)
{
	uint32_t status;

	int retval = lattice_read_status_u32(lattice_device, &status, out, do_idle);
	if (retval != ERROR_OK)
		return retval;

	if ((status & mask) != expected) {
		LOG_ERROR("verifying status register failed got: 0x%08" PRIx32 " expected: 0x%08" PRIx32,
			status & mask, expected);
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int lattice_load_command(struct pld_device *pld_device, const char *filename)
{
	if (!pld_device)
		return ERROR_FAIL;

	struct lattice_pld_device *lattice_device = pld_device->driver_priv;

	if (!lattice_device || !lattice_device->tap)
		return ERROR_FAIL;
	struct jtag_tap *tap = lattice_device->tap;

	if (!tap || !tap->hasidcode)
		return ERROR_FAIL;

	int retval = lattice_check_device_family(lattice_device);
	if (retval != ERROR_OK)
		return retval;

	struct lattice_bit_file bit_file;
	retval = lattice_read_file(&bit_file, filename, lattice_device->family);
	if (retval != ERROR_OK)
		return retval;

	retval = ERROR_FAIL;
	switch (lattice_device->family) {
	case LATTICE_ECP2:
		retval = lattice_ecp2_load(lattice_device, &bit_file);
		break;
	case LATTICE_ECP3:
		retval = lattice_ecp3_load(lattice_device, &bit_file);
		break;
	default:
		LOG_ERROR("loading unknown device family");
		break;
	}
	free(bit_file.raw_bit.data);
	return retval;
}

PLD_DEVICE_COMMAND_HANDLER(lattice_pld_device_command)
{
	if (CMD_ARGC < 2 || CMD_ARGC > 3)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct jtag_tap *tap = jtag_tap_by_string(CMD_ARGV[1]);
	if (!tap) {
		command_print(CMD, "Tap: %s does not exist", CMD_ARGV[1]);
		return ERROR_FAIL;
	}

	struct lattice_pld_device *lattice_device = malloc(sizeof(struct lattice_pld_device));
	if (!lattice_device) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}
	/* id is not known yet -> postpone lattice_check_device_family() */
	enum lattice_family_e family = LATTICE_UNKNOWN;
	if (CMD_ARGC == 3) {
		if (strcasecmp(CMD_ARGV[2], "ecp2") == 0) {
			family = LATTICE_ECP2;
		} else if (strcasecmp(CMD_ARGV[2], "ecp3") == 0) {
			family = LATTICE_ECP3;
		} else {
			command_print(CMD, "unknown family");
			free(lattice_device);
			return ERROR_FAIL;
		}
	}
	lattice_device->tap = tap;
	lattice_device->family = family;
	lattice_device->preload_length = 0;

	pld->driver_priv = lattice_device;

	return ERROR_OK;
}

COMMAND_HANDLER(lattice_read_usercode_register_command_handler)
{
	int dev_id;
	uint32_t usercode;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], dev_id);
	struct pld_device *device = get_pld_device_by_num(dev_id);
	if (!device) {
		command_print(CMD, "pld device '#%s' is out of bounds", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	struct lattice_pld_device *lattice_device = device->driver_priv;
	if (!lattice_device)
		return ERROR_FAIL;

	int retval = lattice_check_device_family(lattice_device);
	if (retval != ERROR_OK)
		return retval;

	retval = lattice_read_usercode(lattice_device, &usercode, 0x0);
	if (retval == ERROR_OK)
		command_print(CMD, "0x%8.8" PRIx32, usercode);

	return retval;
}

COMMAND_HANDLER(lattice_set_preload_command_handler)
{
	int dev_id;
	unsigned int preload_length;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], dev_id);
	struct pld_device *device = get_pld_device_by_num(dev_id);
	if (!device) {
		command_print(CMD, "pld device '#%s' is out of bounds", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[1], preload_length);

	struct lattice_pld_device *lattice_device = device->driver_priv;

	if (!lattice_device)
		return ERROR_FAIL;

	lattice_device->preload_length = preload_length;

	return ERROR_OK;
}

COMMAND_HANDLER(lattice_write_usercode_register_command_handler)
{
	int dev_id;
	uint32_t usercode;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], dev_id);
	struct pld_device *device = get_pld_device_by_num(dev_id);
	if (!device) {
		command_print(CMD, "pld device '#%s' is out of bounds", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], usercode);

	struct lattice_pld_device *lattice_device = device->driver_priv;
	if (!lattice_device)
		return ERROR_FAIL;

	int retval = lattice_check_device_family(lattice_device);
	if (retval != ERROR_OK)
		return retval;

	return lattice_write_usercode(lattice_device, usercode);
}

COMMAND_HANDLER(lattice_read_status_command_handler)
{
	int dev_id;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], dev_id);
	struct pld_device *device = get_pld_device_by_num(dev_id);
	if (!device) {
		command_print(CMD, "pld device '#%s' is out of bounds", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	struct lattice_pld_device *lattice_device = device->driver_priv;
	if (!lattice_device)
		return ERROR_FAIL;

	int retval = lattice_check_device_family(lattice_device);
	if (retval != ERROR_OK)
		return retval;

	uint32_t status;
	retval = lattice_read_status_u32(lattice_device, &status, 0x0, false);
	if (retval == ERROR_OK)
		command_print(CMD, "0x%8.8" PRIx32, status);

	return retval;
}

static const struct command_registration lattice_exec_command_handlers[] = {
	{
		.name = "read_status",
		.mode = COMMAND_EXEC,
		.handler = lattice_read_status_command_handler,
		.help = "reading status register from FPGA",
		.usage = "num_pld",
	}, {
		.name = "read_user",
		.mode = COMMAND_EXEC,
		.handler = lattice_read_usercode_register_command_handler,
		.help = "reading usercode register from FPGA",
		.usage = "num_pld",
	}, {
		.name = "write_user",
		.mode = COMMAND_EXEC,
		.handler = lattice_write_usercode_register_command_handler,
		.help = "writing usercode register to FPGA",
		.usage = "num_pld value",
	}, {
		.name = "set_preload",
		.mode = COMMAND_EXEC,
		.handler = lattice_set_preload_command_handler,
		.help = "set length for preload (device specific)",
		.usage = "num_pld value",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration lattice_command_handler[] = {
	{
		.name = "lattice",
		.mode = COMMAND_ANY,
		.help = "lattice specific commands",
		.usage = "",
		.chain = lattice_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

struct pld_driver lattice_pld = {
	.name = "lattice",
	.commands = lattice_command_handler,
	.pld_device_command = &lattice_pld_device_command,
	.load = &lattice_load_command,
};
