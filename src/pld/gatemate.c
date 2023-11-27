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
#include "pld.h"
#include "raw_bit.h"

#define JTAG_CONFIGURE  0x06
#define JTAG_SPI_BYPASS 0x05
#define BYPASS          0x3F

struct gatemate_pld_device {
	struct jtag_tap *tap;
};

struct gatemate_bit_file {
	struct raw_bit_file raw_file;
	size_t capacity;
};

static int gatemate_add_byte_to_bitfile(struct gatemate_bit_file *bit_file, uint8_t byte)
{
	const size_t chunk_size = 8192;
	if (bit_file->raw_file.length + 1 > bit_file->capacity) {
		uint8_t *buffer;
		if (bit_file->raw_file.data)
			buffer = realloc(bit_file->raw_file.data, bit_file->capacity + chunk_size);
		else
			buffer = malloc(chunk_size);
		if (!buffer) {
			LOG_ERROR("Out of memory");
			return ERROR_FAIL;
		}
		bit_file->raw_file.data = buffer;
		bit_file->capacity += chunk_size;
	}

	bit_file->raw_file.data[bit_file->raw_file.length++] = byte;

	return ERROR_OK;
}

static int gatemate_read_cfg_line(struct gatemate_bit_file *bit_file, const char *line_buffer, size_t nread)
{
	for (size_t idx = 0; idx < nread; ++idx) {
		if (line_buffer[idx] == ' ') {
			continue;
		} else if (line_buffer[idx] == 0) {
			break;
		} else if (idx + 1 < nread) {
			if (isxdigit(line_buffer[idx]) && isxdigit(line_buffer[idx + 1])) {
				uint8_t byte;
				unhexify(&byte, line_buffer + idx, 2);
				int retval = gatemate_add_byte_to_bitfile(bit_file, byte);
				if (retval != ERROR_OK)
					return retval;
			} else if (line_buffer[idx] == '/' && line_buffer[idx + 1] == '/') {
				break;
			}
			++idx;
		} else {
			LOG_ERROR("parsing failed");
			return ERROR_FAIL;
		}
	}
	return ERROR_OK;
}

static int gatemate_getline(char **buffer, size_t *buf_size, FILE *input_file)
{
	const size_t chunk_size = 32;
	if (!*buffer)
		*buf_size = 0;

	size_t read = 0;
	do {
		if (read + 1 > *buf_size) {
			char *new_buffer;
			if (*buffer)
				new_buffer = realloc(*buffer, *buf_size + chunk_size);
			else
				new_buffer = malloc(chunk_size);
			if (!new_buffer) {
				LOG_ERROR("Out of memory");
				return -1;
			}
			*buffer = new_buffer;
			*buf_size += chunk_size;
		}

		int c = fgetc(input_file);
		if ((c == EOF && read) || (char)c == '\n') {
			(*buffer)[read++] = 0;
			return read;
		} else if (c == EOF) {
			return -1;
		}

		(*buffer)[read++] = (char)c;
	} while (1);

	return -1;
}

static int gatemate_read_cfg_file(struct gatemate_bit_file *bit_file, const char *filename)
{
	FILE *input_file = fopen(filename, "r");

	if (!input_file) {
		LOG_ERROR("Couldn't open %s: %s", filename, strerror(errno));
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	int retval = ERROR_OK;
	char *line_buffer = NULL;
	size_t buffer_length = 0;
	int nread;
	while (((nread = gatemate_getline(&line_buffer, &buffer_length, input_file)) != -1) && (retval == ERROR_OK))
		retval = gatemate_read_cfg_line(bit_file, line_buffer, (size_t)nread);

	if (line_buffer)
		free(line_buffer);

	fclose(input_file);
	if (retval != ERROR_OK)
		free(bit_file->raw_file.data);
	return retval;
}

static int gatemate_read_file(struct gatemate_bit_file *bit_file, const char *filename)
{
	memset(bit_file, 0, sizeof(struct gatemate_bit_file));

	if (!filename || !bit_file)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* check if binary .bit or ascii .cfg */
	const char *file_suffix_pos = strrchr(filename, '.');
	if (!file_suffix_pos) {
		LOG_ERROR("Unable to detect filename suffix");
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	if (strcasecmp(file_suffix_pos, ".bit") == 0)
		return cpld_read_raw_bit_file(&bit_file->raw_file, filename);
	else if (strcasecmp(file_suffix_pos, ".cfg") == 0)
		return gatemate_read_cfg_file(bit_file, filename);

	LOG_ERROR("Filetype not supported, expecting .bit or .cfg file");
	return ERROR_PLD_FILE_LOAD_FAILED;
}

static int gatemate_set_instr(struct jtag_tap *tap, uint8_t new_instr)
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
	jtag_add_runtest(3, TAP_IDLE);
	free(t);
	return ERROR_OK;
}

static int gatemate_load(struct pld_device *pld_device, const char *filename)
{
	if (!pld_device)
		return ERROR_FAIL;

	struct gatemate_pld_device *gatemate_info = pld_device->driver_priv;

	if (!gatemate_info || !gatemate_info->tap)
		return ERROR_FAIL;
	struct jtag_tap *tap = gatemate_info->tap;

	struct gatemate_bit_file bit_file;
	int retval = gatemate_read_file(&bit_file, filename);
	if (retval != ERROR_OK)
		return retval;

	retval = gatemate_set_instr(tap, JTAG_CONFIGURE);
	if (retval != ERROR_OK) {
		free(bit_file.raw_file.data);
		return retval;
	}

	struct scan_field field;
	field.num_bits = bit_file.raw_file.length * 8;
	field.out_value = bit_file.raw_file.data;
	field.in_value = NULL;
	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);

	retval = jtag_execute_queue();
	free(bit_file.raw_file.data);

	return retval;
}

static int gatemate_has_jtagspi_instruction(struct pld_device *device, bool *has_instruction)
{
	*has_instruction = true;
	return ERROR_OK;
}

static int gatemate_connect_spi_to_jtag(struct pld_device *pld_device)
{
	if (!pld_device)
		return ERROR_FAIL;

	struct gatemate_pld_device *pld_device_info = pld_device->driver_priv;
	if (!pld_device_info)
		return ERROR_FAIL;

	struct jtag_tap *tap = pld_device_info->tap;
	if (!tap)
		return ERROR_FAIL;

	if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) == JTAG_SPI_BYPASS)
		return ERROR_OK;

	gatemate_set_instr(tap, JTAG_SPI_BYPASS);

	return jtag_execute_queue();
}

static int gatemate_disconnect_spi_from_jtag(struct pld_device *pld_device)
{
	if (!pld_device)
		return ERROR_FAIL;

	struct gatemate_pld_device *pld_device_info = pld_device->driver_priv;
	if (!pld_device_info)
		return ERROR_FAIL;

	struct jtag_tap *tap = pld_device_info->tap;
	if (!tap)
		return ERROR_FAIL;

	if (buf_get_u32(tap->cur_instr, 0, tap->ir_length) != JTAG_SPI_BYPASS)
		return ERROR_OK;

	gatemate_set_instr(tap, BYPASS);

	return jtag_execute_queue();
}

static int gatemate_get_stuff_bits(struct pld_device *pld_device, unsigned int *facing_read_bits,
									unsigned int *trailing_write_bits)
{
	if (!pld_device)
		return ERROR_FAIL;

	*facing_read_bits = 1;
	*trailing_write_bits = 1;

	return ERROR_OK;
}

PLD_CREATE_COMMAND_HANDLER(gatemate_pld_create_command)
{
	if (CMD_ARGC != 4)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (strcmp(CMD_ARGV[2], "-chain-position") != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct jtag_tap *tap = jtag_tap_by_string(CMD_ARGV[3]);
	if (!tap) {
		command_print(CMD, "Tap: %s does not exist", CMD_ARGV[3]);
		return ERROR_FAIL;
	}

	struct gatemate_pld_device *gatemate_info = malloc(sizeof(struct gatemate_pld_device));
	if (!gatemate_info) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}
	gatemate_info->tap = tap;

	pld->driver_priv = gatemate_info;

	return ERROR_OK;
}

struct pld_driver gatemate_pld = {
	.name = "gatemate",
	.pld_create_command = &gatemate_pld_create_command,
	.load = &gatemate_load,
	.has_jtagspi_instruction = gatemate_has_jtagspi_instruction,
	.connect_spi_to_jtag = gatemate_connect_spi_to_jtag,
	.disconnect_spi_from_jtag = gatemate_disconnect_spi_from_jtag,
	.get_stuff_bits = gatemate_get_stuff_bits,
};
