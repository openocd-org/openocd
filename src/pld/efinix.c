// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2022 by Daniel Anselmi                                  *
 *   danselmi@gmx.ch                                                       *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <jtag/jtag.h>

#include "pld.h"
#include "raw_bit.h"

#define PROGRAM   0x4
#define ENTERUSER 0x7

#define TRAILING_ZEROS 4000
#define RUNTEST_START_CYCLES 100
#define RUNTEST_FINISH_CYCLES 100

struct efinix_pld_device {
	struct jtag_tap *tap;
};

static int efinix_read_bit_file(struct raw_bit_file *bit_file, const char *filename)
{
	FILE *input_file = fopen(filename, "r");
	if (!input_file) {
		LOG_ERROR("couldn't open %s: %s", filename, strerror(errno));
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	fseek(input_file, 0, SEEK_END);
	long length = ftell(input_file);
	fseek(input_file, 0, SEEK_SET);

	if (length < 0 || ((length % 3))) {
		fclose(input_file);
		LOG_ERROR("Failed to get length from file %s: %s", filename, strerror(errno));
		return ERROR_PLD_FILE_LOAD_FAILED;
	}
	bit_file->length = DIV_ROUND_UP(length, 3);

	bit_file->data = malloc(bit_file->length);
	if (!bit_file->data) {
		fclose(input_file);
		LOG_ERROR("Out of memory");
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	bool end_detected = false;
	char buffer[3];
	for (size_t idx = 0; !end_detected && idx < bit_file->length; ++idx) {
		size_t read_count = fread(buffer, sizeof(char), 3, input_file);
		end_detected = feof(input_file);
		if ((read_count == 3 && buffer[2] != '\n') ||
			(read_count != 3 && !end_detected) ||
			(read_count != 2 && end_detected)) {
			fclose(input_file);
			free(bit_file->data);
			bit_file->data = NULL;
			LOG_ERROR("unexpected line length");
			return ERROR_PLD_FILE_LOAD_FAILED;
		}

		if (!isxdigit(buffer[0]) || !isxdigit(buffer[1])) {
			fclose(input_file);
			free(bit_file->data);
			bit_file->data = NULL;
			LOG_ERROR("unexpected char in hex string");
			return ERROR_PLD_FILE_LOAD_FAILED;
		}
		unhexify(&bit_file->data[idx], buffer, 2);
	}

	fclose(input_file);

	return ERROR_OK;
}

static int efinix_read_file(struct raw_bit_file *bit_file, const char *filename)
{
	if (!filename || !bit_file)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* check if binary .bin or ascii .bit/.hex */
	const char *file_ending_pos = strrchr(filename, '.');
	if (!file_ending_pos) {
		LOG_ERROR("Unable to detect filename suffix");
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	if (strcasecmp(file_ending_pos, ".bin") == 0) {
		return cpld_read_raw_bit_file(bit_file, filename);
	} else if ((strcasecmp(file_ending_pos, ".bit") == 0) ||
			(strcasecmp(file_ending_pos, ".hex") == 0)) {
		return efinix_read_bit_file(bit_file, filename);
	}

	LOG_ERROR("Unable to detect filetype");
	return ERROR_PLD_FILE_LOAD_FAILED;
}

static int efinix_set_instr(struct jtag_tap *tap, uint8_t new_instr)
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

static int efinix_load(struct pld_device *pld_device, const char *filename)
{
	struct raw_bit_file bit_file;
	struct scan_field field[2];

	if (!pld_device || !pld_device->driver_priv)
		return ERROR_FAIL;

	struct efinix_pld_device *efinix_info = pld_device->driver_priv;
	if (!efinix_info || !efinix_info->tap)
		return ERROR_FAIL;
	struct jtag_tap *tap = efinix_info->tap;

	jtag_add_tlr();

	int retval = efinix_set_instr(tap, PROGRAM);
	if (retval != ERROR_OK)
		return retval;
	jtag_add_runtest(RUNTEST_START_CYCLES, TAP_IDLE);
	retval = efinix_set_instr(tap, PROGRAM); /* fix for T20 */
	if (retval != ERROR_OK)
		return retval;
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	retval = efinix_read_file(&bit_file, filename);
	if (retval != ERROR_OK)
		return retval;

	for (size_t i = 0; i < bit_file.length; i++)
		bit_file.data[i] = flip_u32(bit_file.data[i], 8);

	/* shift in the bitstream */
	field[0].num_bits = bit_file.length * 8;
	field[0].out_value = bit_file.data;
	field[0].in_value = NULL;

	/* followed by zeros */
	field[1].num_bits = TRAILING_ZEROS;
	uint8_t *buf = calloc(TRAILING_ZEROS / 8, 1);
	if (!buf) {
		free(bit_file.data);
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}
	field[1].out_value = buf;
	field[1].in_value = NULL;

	jtag_add_dr_scan(tap, 2, field, TAP_DRPAUSE);
	retval = jtag_execute_queue();
	free(bit_file.data);
	free(buf);
	if (retval != ERROR_OK)
		return retval;

	retval = efinix_set_instr(tap, ENTERUSER);
	if (retval != ERROR_OK)
		return retval;

	/* entering RUN/TEST for 100 cycles */
	jtag_add_runtest(RUNTEST_FINISH_CYCLES, TAP_IDLE);
	retval = jtag_execute_queue();

	return retval;
}

PLD_DEVICE_COMMAND_HANDLER(efinix_pld_device_command)
{
	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct jtag_tap *tap = jtag_tap_by_string(CMD_ARGV[1]);
	if (!tap) {
		command_print(CMD, "Tap: %s does not exist", CMD_ARGV[1]);
		return ERROR_FAIL;
	}

	struct efinix_pld_device *efinix_info = malloc(sizeof(struct efinix_pld_device));
	if (!efinix_info) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}
	efinix_info->tap = tap;

	pld->driver_priv = efinix_info;

	return ERROR_OK;
}

struct pld_driver efinix_pld = {
	.name = "efinix",
	.pld_device_command = &efinix_pld_device_command,
	.load = &efinix_load,
};
