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
#include "ecp5.h"
#include "certus.h"

#define PRELOAD              0x1C
#define USER1                0x32
#define USER2                0x38


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
	{0x21111043,  409, LATTICE_ECP5 /* "LAE5U-12F & LFE5U-12F" */},
	{0x41111043,  409, LATTICE_ECP5 /* "LFE5U-25F" */},
	{0x41112043,  510, LATTICE_ECP5 /* "LFE5U-45F" */},
	{0x41113043,  750, LATTICE_ECP5 /* "LFE5U-85F" */},
	{0x81111043,  409, LATTICE_ECP5 /* "LFE5UM5G-25F" */},
	{0x81112043,  510, LATTICE_ECP5 /* "LFE5UM5G-45F" */},
	{0x81113043,  750, LATTICE_ECP5 /* "LFE5UM5G-85F" */},
	{0x01111043,  409, LATTICE_ECP5 /* "LAE5UM-25F" */},
	{0x01112043,  510, LATTICE_ECP5 /* "LAE5UM-45F" */},
	{0x01113043,  750, LATTICE_ECP5 /* "LAE5UM-85F" */},
	{0x310f0043,  362, LATTICE_CERTUS /* LFD2NX-17 */},
	{0x310f1043,  362, LATTICE_CERTUS /* LFD2NX-40 */},
	{0x010f4043,  362, LATTICE_CERTUS /* LFCPNX-100 */},
};

int lattice_set_instr(struct jtag_tap *tap, uint8_t new_instr, enum tap_state endstate)
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

	if (!lattice_device->tap || !lattice_device->tap->has_idcode)
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

int lattice_read_u64_register(struct jtag_tap *tap, uint8_t cmd, uint64_t *in_val,
							uint64_t out_val)
{
	struct scan_field field;
	uint8_t buffer[8];

	int retval = lattice_set_instr(tap, cmd, TAP_IDLE);
	if (retval != ERROR_OK)
		return retval;
	h_u64_to_le(buffer, out_val);
	field.num_bits = 64;
	field.out_value = buffer;
	field.in_value = buffer;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
	retval = jtag_execute_queue();
	if (retval == ERROR_OK)
		*in_val = le_to_h_u64(buffer);

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
	else if (lattice_device->family == LATTICE_ECP5)
		return lattice_ecp5_read_usercode(tap, usercode, out);
	else if (lattice_device->family == LATTICE_CERTUS)
		return lattice_certus_read_usercode(tap, usercode, out);

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
	else if (lattice_device->family == LATTICE_ECP5)
		return lattice_ecp5_write_usercode(lattice_device, usercode);
	else if (lattice_device->family == LATTICE_CERTUS)
		return lattice_certus_write_usercode(lattice_device, usercode);

	return ERROR_FAIL;
}

static int lattice_read_status_u32(struct lattice_pld_device *lattice_device, uint32_t *status,
								uint32_t out, bool do_idle)
{
	if (!lattice_device->tap)
		return ERROR_FAIL;

	if (lattice_device->family == LATTICE_ECP2 || lattice_device->family == LATTICE_ECP3)
		return lattice_ecp2_3_read_status(lattice_device->tap, status, out, do_idle);
	else if (lattice_device->family == LATTICE_ECP5)
		return lattice_ecp5_read_status(lattice_device->tap, status, out, do_idle);

	return ERROR_FAIL;
}
static int lattice_read_status_u64(struct lattice_pld_device *lattice_device, uint64_t *status,
								uint64_t out)
{
	if (!lattice_device->tap)
		return ERROR_FAIL;

	if (lattice_device->family == LATTICE_CERTUS)
		return lattice_certus_read_status(lattice_device->tap, status, out);

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

int lattice_verify_status_register_u64(struct lattice_pld_device *lattice_device, uint64_t out,
						uint64_t expected, uint64_t mask)
{
	uint64_t status;
	int retval = lattice_read_status_u64(lattice_device, &status, out);
	if (retval != ERROR_OK)
		return retval;

	if ((status & mask) != expected) {
		LOG_ERROR("verifying status register failed got: 0x%08" PRIx64 " expected: 0x%08" PRIx64,
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

	if (!tap || !tap->has_idcode)
		return ERROR_FAIL;

	int retval = lattice_check_device_family(lattice_device);
	if (retval != ERROR_OK)
		return retval;

	struct lattice_bit_file bit_file;
	retval = lattice_read_file(&bit_file, filename, lattice_device->family);
	if (retval != ERROR_OK)
		return retval;

	uint32_t id = tap->idcode;
	retval = ERROR_FAIL;
	switch (lattice_device->family) {
	case LATTICE_ECP2:
		retval = lattice_ecp2_load(lattice_device, &bit_file);
		break;
	case LATTICE_ECP3:
		retval = lattice_ecp3_load(lattice_device, &bit_file);
		break;
	case LATTICE_ECP5:
	case LATTICE_CERTUS:
		if (bit_file.has_id && id != bit_file.idcode)
			LOG_WARNING("Id on device (0x%8.8" PRIx32 ") and id in bit-stream (0x%8.8" PRIx32 ") don't match.",
				id, bit_file.idcode);
		if (lattice_device->family == LATTICE_ECP5)
			retval = lattice_ecp5_load(lattice_device, &bit_file);
		else
			retval = lattice_certus_load(lattice_device, &bit_file);
		break;
	default:
		LOG_ERROR("loading unknown device family");
		break;
	}
	free(bit_file.raw_bit.data);
	return retval;
}

static int lattice_get_ipdbg_hub(int user_num, struct pld_device *pld_device, struct pld_ipdbg_hub *hub)
{
	if (!pld_device)
		return ERROR_FAIL;

	struct lattice_pld_device *pld_device_info = pld_device->driver_priv;

	if (!pld_device_info || !pld_device_info->tap)
		return ERROR_FAIL;

	hub->tap = pld_device_info->tap;

	if (user_num == 1) {
		hub->user_ir_code = USER1;
	} else if (user_num == 2) {
		hub->user_ir_code = USER2;
	} else {
		LOG_ERROR("lattice devices only have user register 1 & 2");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

static int lattice_connect_spi_to_jtag(struct pld_device *pld_device)
{
	if (!pld_device)
		return ERROR_FAIL;

	struct lattice_pld_device *pld_device_info = pld_device->driver_priv;

	int retval = lattice_check_device_family(pld_device_info);
	if (retval != ERROR_OK)
		return retval;

	if (pld_device_info->family == LATTICE_ECP2 || pld_device_info->family == LATTICE_ECP3)
		return lattice_ecp2_3_connect_spi_to_jtag(pld_device_info);
	else if (pld_device_info->family == LATTICE_ECP5)
		return lattice_ecp5_connect_spi_to_jtag(pld_device_info);
	else if (pld_device_info->family == LATTICE_CERTUS)
		return lattice_certus_connect_spi_to_jtag(pld_device_info);

	return ERROR_FAIL;
}

static int lattice_disconnect_spi_from_jtag(struct pld_device *pld_device)
{
	if (!pld_device)
		return ERROR_FAIL;

	struct lattice_pld_device *pld_device_info = pld_device->driver_priv;

	int retval = lattice_check_device_family(pld_device_info);
	if (retval != ERROR_OK)
		return retval;

	if (pld_device_info->family == LATTICE_ECP2 || pld_device_info->family == LATTICE_ECP3)
		return lattice_ecp2_3_disconnect_spi_from_jtag(pld_device_info);
	else if (pld_device_info->family == LATTICE_ECP5)
		return lattice_ecp5_disconnect_spi_from_jtag(pld_device_info);
	else if (pld_device_info->family == LATTICE_CERTUS)
		return lattice_certus_disconnect_spi_from_jtag(pld_device_info);

	return ERROR_FAIL;
}

static int lattice_get_stuff_bits(struct pld_device *pld_device, unsigned int *facing_read_bits,
		unsigned int *trailing_write_bits)
{
	if (!pld_device)
		return ERROR_FAIL;

	struct lattice_pld_device *pld_device_info = pld_device->driver_priv;

	int retval = lattice_check_device_family(pld_device_info);
	if (retval != ERROR_OK)
		return retval;

	if (pld_device_info->family == LATTICE_ECP2 || pld_device_info->family == LATTICE_ECP3)
		return lattice_ecp2_3_get_facing_read_bits(pld_device_info, facing_read_bits);
	else if (pld_device_info->family == LATTICE_ECP5)
		return lattice_ecp5_get_facing_read_bits(pld_device_info, facing_read_bits);
	else if (pld_device_info->family == LATTICE_CERTUS)
		return lattice_certus_get_facing_read_bits(pld_device_info, facing_read_bits);

	return ERROR_FAIL;
}

static int lattice_has_jtagspi_instruction(struct pld_device *device, bool *has_instruction)
{
	*has_instruction = true;
	return ERROR_OK;
}

PLD_CREATE_COMMAND_HANDLER(lattice_pld_create_command)
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

	/* id is not known yet -> postpone lattice_check_device_family() */
	enum lattice_family_e family = LATTICE_UNKNOWN;
	if (CMD_ARGC == 6) {
		if (strcmp(CMD_ARGV[4], "-family") != 0)
			return ERROR_COMMAND_SYNTAX_ERROR;

		if (strcasecmp(CMD_ARGV[5], "ecp2") == 0) {
			family = LATTICE_ECP2;
		} else if (strcasecmp(CMD_ARGV[5], "ecp3") == 0) {
			family = LATTICE_ECP3;
		} else if (strcasecmp(CMD_ARGV[5], "ecp5") == 0) {
			family = LATTICE_ECP5;
		} else if (strcasecmp(CMD_ARGV[5], "certus") == 0) {
			family = LATTICE_CERTUS;
		} else {
			command_print(CMD, "unknown family");
			return ERROR_FAIL;
		}
	}

	struct lattice_pld_device *lattice_device = malloc(sizeof(struct lattice_pld_device));
	if (!lattice_device) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	lattice_device->tap = tap;
	lattice_device->family = family;
	lattice_device->preload_length = 0;

	pld->driver_priv = lattice_device;

	return ERROR_OK;
}

COMMAND_HANDLER(lattice_read_usercode_register_command_handler)
{
	uint32_t usercode;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct pld_device *device = get_pld_device_by_name_or_numstr(CMD_ARGV[0]);
	if (!device) {
		command_print(CMD, "pld device '#%s' is out of bounds or unknown", CMD_ARGV[0]);
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
	unsigned int preload_length;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct pld_device *device = get_pld_device_by_name_or_numstr(CMD_ARGV[0]);
	if (!device) {
		command_print(CMD, "pld device '#%s' is out of bounds or unknown", CMD_ARGV[0]);
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
	uint32_t usercode;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct pld_device *device = get_pld_device_by_name_or_numstr(CMD_ARGV[0]);
	if (!device) {
		command_print(CMD, "pld device '#%s' is out of bounds or unknown", CMD_ARGV[0]);
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
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct pld_device *device = get_pld_device_by_name_or_numstr(CMD_ARGV[0]);
	if (!device) {
		command_print(CMD, "pld device '#%s' is out of bounds or unknown", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	struct lattice_pld_device *lattice_device = device->driver_priv;
	if (!lattice_device)
		return ERROR_FAIL;

	int retval = lattice_check_device_family(lattice_device);
	if (retval != ERROR_OK)
		return retval;

	if (lattice_device->family == LATTICE_CERTUS) {
		uint64_t status;
		retval = lattice_read_status_u64(lattice_device, &status, 0x0);
		if (retval == ERROR_OK)
			command_print(CMD, "0x%016" PRIx64, status);
	} else {
		uint32_t status;
		const bool do_idle = lattice_device->family == LATTICE_ECP5;
		retval = lattice_read_status_u32(lattice_device, &status, 0x0, do_idle);
		if (retval == ERROR_OK)
			command_print(CMD, "0x%8.8" PRIx32, status);
	}

	return retval;
}

COMMAND_HANDLER(lattice_refresh_command_handler)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct pld_device *device = get_pld_device_by_name_or_numstr(CMD_ARGV[0]);
	if (!device) {
		command_print(CMD, "pld device '#%s' is out of bounds or unknown", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	struct lattice_pld_device *lattice_device = device->driver_priv;
	if (!lattice_device)
		return ERROR_FAIL;

	int retval = lattice_check_device_family(lattice_device);
	if (retval != ERROR_OK)
		return retval;

	if (lattice_device->family == LATTICE_ECP2 || lattice_device->family == LATTICE_ECP3)
		return lattice_ecp2_3_refresh(lattice_device);
	else if (lattice_device->family == LATTICE_ECP5)
		return lattice_ecp5_refresh(lattice_device);
	else if (lattice_device->family == LATTICE_CERTUS)
		return lattice_certus_refresh(lattice_device);

	return ERROR_FAIL;
}

static const struct command_registration lattice_exec_command_handlers[] = {
	{
		.name = "read_status",
		.mode = COMMAND_EXEC,
		.handler = lattice_read_status_command_handler,
		.help = "reading status register from FPGA",
		.usage = "pld_name",
	}, {
		.name = "read_user",
		.mode = COMMAND_EXEC,
		.handler = lattice_read_usercode_register_command_handler,
		.help = "reading usercode register from FPGA",
		.usage = "pld_name",
	}, {
		.name = "write_user",
		.mode = COMMAND_EXEC,
		.handler = lattice_write_usercode_register_command_handler,
		.help = "writing usercode register to FPGA",
		.usage = "pld_name value",
	}, {
		.name = "set_preload",
		.mode = COMMAND_ANY,
		.handler = lattice_set_preload_command_handler,
		.help = "set length for preload (device specific)",
		.usage = "pld_name value",
	}, {
		.name = "refresh",
		.mode = COMMAND_EXEC,
		.handler = lattice_refresh_command_handler,
		.help = "refresh from configuration memory",
		.usage = "pld_name",
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
	.pld_create_command = &lattice_pld_create_command,
	.load = &lattice_load_command,
	.get_ipdbg_hub = lattice_get_ipdbg_hub,
	.has_jtagspi_instruction = lattice_has_jtagspi_instruction,
	.connect_spi_to_jtag = lattice_connect_spi_to_jtag,
	.disconnect_spi_from_jtag = lattice_disconnect_spi_from_jtag,
	.get_stuff_bits = lattice_get_stuff_bits,
};
