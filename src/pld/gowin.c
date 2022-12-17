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
#include <helper/bits.h>
#include "pld.h"
#include "raw_bit.h"

#define NO_OP                       0x02
#define ERASE_SRAM                  0x05
#define SRAM_ERASE_DONE             0x09
#define IDCODE                      0x11
#define ADDRESS_INITIALIZATION      0x12
#define READ_USERCODE               0x13
#define CONFIG_ENABLE               0x15
#define TRANSFER_CONFIGURATION_DATA 0x17
#define CONFIG_DISABLE              0x3A
#define RELOAD                      0x3C
#define STATUS_REGISTER             0x41
#define ERASE_FLASH                 0x75
#define ENABLE_2ND_FLASH            0x78

#define USER1                       0x42
#define USER2                       0x43

#define STAUS_MASK_MEMORY_ERASE     BIT(5)
#define STAUS_MASK_SYSTEM_EDIT_MODE BIT(7)

struct gowin_pld_device {
	struct jtag_tap *tap;
};

struct gowin_bit_file {
	struct raw_bit_file raw_file;
	size_t capacity;
	uint32_t id;
	uint16_t stored_checksum;
	int compressed;
	int crc_en;
	uint16_t checksum;
	uint8_t replace8x;
	uint8_t replace4x;
	uint8_t replace2x;
};

static uint64_t gowin_read_fs_file_bitsequence(const char *bits, int length)
{
	uint64_t res = 0;
	for (int i = 0; i < length; i++)
		res = (res << 1) | (*bits++ == '1' ? 1 : 0);
	return res;
}

static int gowin_add_byte_to_bit_file(struct gowin_bit_file *bit_file, uint8_t byte)
{
	if (bit_file->raw_file.length + 1 > bit_file->capacity) {
		uint8_t *buffer;
		if (bit_file->raw_file.data)
			buffer = realloc(bit_file->raw_file.data, bit_file->capacity + 8192);
		else
			buffer = malloc(8192);
		if (!buffer) {
			LOG_ERROR("Out of memory");
			return ERROR_FAIL;
		}
		bit_file->raw_file.data = buffer;
		bit_file->capacity += 8192;
	}

	bit_file->raw_file.data[bit_file->raw_file.length++] = byte;

	return ERROR_OK;
}

static int gowin_read_fs_file_header(struct gowin_bit_file *bit_file, FILE *stream)
{
	if (!bit_file)
		return ERROR_FAIL;

	int end_of_header = 0;
	while (!end_of_header) {
		char buffer[256];
		char *line = fgets(buffer, 256, stream);
		if (!line || feof(stream) || ferror(stream))
			return ERROR_FAIL;

		if (line[0] == '/')
			continue;

		size_t line_length = strlen(line);
		if (line[line_length - 1] != '\n')
			return ERROR_FAIL;
		line_length--;

		for (unsigned int i = 0; i < line_length; i += 8) {
			uint8_t byte = gowin_read_fs_file_bitsequence(line + i, 8);
			int retval = gowin_add_byte_to_bit_file(bit_file, byte);
			if (retval != ERROR_OK)
				return retval;
		}

		uint8_t key = gowin_read_fs_file_bitsequence(line, 8);
		line += 8;
		uint64_t value = gowin_read_fs_file_bitsequence(line, line_length - 8);

		if (key == 0x06) {
			bit_file->id = value & 0xffffffff;
		} else if (key == 0x3B) {
			end_of_header = 1;
			bit_file->crc_en = (value & BIT(23)) ? 1 : 0;
		}
	}

	return ERROR_OK;
}

static int gowin_read_fs_file(struct gowin_bit_file *bit_file, const char *filename)
{
	FILE *input_file = fopen(filename, "r");

	if (!input_file) {
		LOG_ERROR("Couldn't open %s: %s", filename, strerror(errno));
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	int retval = gowin_read_fs_file_header(bit_file, input_file);
	if (retval != ERROR_OK) {
		free(bit_file->raw_file.data);
		fclose(input_file);
		return retval;
	}

	char digits_buffer[9]; /* 8 + 1 trailing zero */
	do {
		char *digits = fgets(digits_buffer, 9, input_file);
		if (feof(input_file))
			break;
		if (!digits || ferror(input_file)) {
			free(bit_file->raw_file.data);
			fclose(input_file);
			return ERROR_FAIL;
		}
		if (digits[0] == '\n')
			continue;

		if (strlen(digits) != 8) {
			free(bit_file->raw_file.data);
			fclose(input_file);
			return ERROR_FAIL;
		}
		uint8_t byte = gowin_read_fs_file_bitsequence(digits, 8);
		retval = gowin_add_byte_to_bit_file(bit_file, byte);
		if (retval != ERROR_OK) {
			free(bit_file->raw_file.data);
			fclose(input_file);
			return ERROR_FAIL;
		}
	} while (1);

	fclose(input_file);
	return ERROR_OK;
}

static int gowin_read_file(struct gowin_bit_file *bit_file, const char *filename, bool *is_fs)
{
	memset(bit_file, 0, sizeof(struct gowin_bit_file));

	if (!filename || !bit_file)
		return ERROR_COMMAND_SYNTAX_ERROR;

	const char *file_suffix_pos = strrchr(filename, '.');
	if (!file_suffix_pos) {
		LOG_ERROR("Unable to detect filename suffix");
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	/* check if binary .bin or ascii .fs */
	if (strcasecmp(file_suffix_pos, ".bin") == 0) {
		*is_fs = false;
		return cpld_read_raw_bit_file(&bit_file->raw_file, filename);
	} else if (strcasecmp(file_suffix_pos, ".fs") == 0) {
		*is_fs = true;
		return gowin_read_fs_file(bit_file, filename);
	}

	LOG_ERROR("Filetype not supported, expecting .fs or .bin file");
	return ERROR_PLD_FILE_LOAD_FAILED;
}

static int gowin_set_instr(struct jtag_tap *tap, uint8_t new_instr)
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

static int gowin_read_register(struct jtag_tap *tap, uint32_t reg, uint32_t *result)
{
	struct scan_field field;

	int retval = gowin_set_instr(tap, reg);
	if (retval != ERROR_OK)
		return retval;
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	uint8_t buf[4] = {0};
	field.check_mask = NULL;
	field.check_value = NULL;
	field.num_bits = 32;
	field.out_value = buf;
	field.in_value = buf;

	jtag_add_dr_scan(tap, 1, &field, TAP_IDLE);
	retval = jtag_execute_queue();
	*result = le_to_h_u32(buf);
	return retval;
}

static int gowin_check_status_flag(struct jtag_tap *tap, uint32_t mask, uint32_t flag)
{
	uint32_t status = 0;

	int retries = 0;
	do {
		int retval = gowin_read_register(tap, STATUS_REGISTER, &status);
		if (retval != ERROR_OK)
			return retval;
		if (retries++ == 100000)
			return ERROR_FAIL;
	} while ((status & mask) != flag);

	return ERROR_OK;
}

static int gowin_enable_config(struct jtag_tap *tap)
{
	int retval = gowin_set_instr(tap, CONFIG_ENABLE);
	if (retval != ERROR_OK)
		return retval;
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	return gowin_check_status_flag(tap, STAUS_MASK_SYSTEM_EDIT_MODE, STAUS_MASK_SYSTEM_EDIT_MODE);
}

static int gowin_disable_config(struct jtag_tap *tap)
{
	int retval = gowin_set_instr(tap, CONFIG_DISABLE);
	if (retval != ERROR_OK)
		return retval;
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	return gowin_check_status_flag(tap, STAUS_MASK_SYSTEM_EDIT_MODE, 0);
}

static int gowin_reload(struct jtag_tap *tap)
{
	int retval = gowin_set_instr(tap, RELOAD);
	if (retval != ERROR_OK)
		return retval;
	retval = gowin_set_instr(tap, NO_OP);
	if (retval != ERROR_OK)
		return retval;
	return jtag_execute_queue();
}

static int gowin_runtest_idle(struct jtag_tap *tap, unsigned int frac_sec)
{
	int speed = adapter_get_speed_khz() * 1000;
	int cycles = DIV_ROUND_UP(speed, frac_sec);
	jtag_add_runtest(cycles, TAP_IDLE);
	return jtag_execute_queue();
}

static int gowin_erase_sram(struct jtag_tap *tap, bool tx_erase_done)
{
	/* config is already enabled */
	int retval = gowin_set_instr(tap, ERASE_SRAM);
	if (retval != ERROR_OK)
		return retval;
	retval = gowin_set_instr(tap, NO_OP);
	if (retval != ERROR_OK)
		return retval;

	/* Delay or Run Test 2~10ms */
	/* 10 ms is worst case for GW2A-55 */
	jtag_add_sleep(10000);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
		return retval;

	retval = gowin_check_status_flag(tap, STAUS_MASK_MEMORY_ERASE,
									STAUS_MASK_MEMORY_ERASE);
	if (retval != ERROR_OK)
		return retval;

	if (tx_erase_done) {
		retval = gowin_set_instr(tap, SRAM_ERASE_DONE);
		if (retval != ERROR_OK)
			return retval;
		retval = gowin_set_instr(tap, NO_OP);
		if (retval != ERROR_OK)
			return retval;
		retval = jtag_execute_queue();
		if (retval != ERROR_OK)
			return retval;
		/* gen clock cycles in RUN/IDLE for 500us -> 1/500us = 2000/s */
		retval = gowin_runtest_idle(tap, 2000);
		if (retval != ERROR_OK)
			return retval;
	}

	retval = gowin_set_instr(tap, NO_OP);
	if (retval != ERROR_OK)
		return retval;
	return jtag_execute_queue();
}

static int gowin_load_to_sram(struct pld_device *pld_device, const char *filename)
{
	if (!pld_device)
		return ERROR_FAIL;

	struct gowin_pld_device *gowin_info = pld_device->driver_priv;

	if (!gowin_info || !gowin_info->tap)
		return ERROR_FAIL;
	struct jtag_tap *tap = gowin_info->tap;

	bool is_fs = false;
	struct gowin_bit_file bit_file;
	int retval = gowin_read_file(&bit_file, filename, &is_fs);
	if (retval != ERROR_OK)
		return retval;

	for (unsigned int i = 0; i < bit_file.raw_file.length; i++)
		bit_file.raw_file.data[i] = flip_u32(bit_file.raw_file.data[i], 8);

	uint32_t id;
	retval = gowin_read_register(tap, IDCODE, &id);
	if (retval != ERROR_OK) {
		free(bit_file.raw_file.data);
		return retval;
	}

	if (is_fs && id != bit_file.id) {
		free(bit_file.raw_file.data);
		LOG_ERROR("Id on device (0x%8.8" PRIx32 ") and id in bit-stream (0x%8.8" PRIx32 ") don't match.",
			id, bit_file.id);
		return ERROR_FAIL;
	}

	retval = gowin_enable_config(tap);
	if (retval != ERROR_OK) {
		free(bit_file.raw_file.data);
		return retval;
	}

	retval = gowin_erase_sram(tap, false);
	if (retval != ERROR_OK) {
		free(bit_file.raw_file.data);
		return retval;
	}

	retval = gowin_set_instr(tap, ADDRESS_INITIALIZATION);
	if (retval != ERROR_OK) {
		free(bit_file.raw_file.data);
		return retval;
	}
	retval = gowin_set_instr(tap, TRANSFER_CONFIGURATION_DATA);
	if (retval != ERROR_OK) {
		free(bit_file.raw_file.data);
		return retval;
	}

	/* scan out the bitstream */
	struct scan_field field;
	field.num_bits = bit_file.raw_file.length * 8;
	field.out_value = bit_file.raw_file.data;
	field.in_value = bit_file.raw_file.data;
	jtag_add_dr_scan(gowin_info->tap, 1, &field, TAP_IDLE);
	jtag_add_runtest(3, TAP_IDLE);

	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		free(bit_file.raw_file.data);
		return retval;
	}

	retval = gowin_disable_config(tap);
	free(bit_file.raw_file.data);
	if (retval != ERROR_OK)
		return retval;

	retval = gowin_set_instr(gowin_info->tap, NO_OP);
	if (retval != ERROR_OK)
		return retval;

	retval = jtag_execute_queue();

	return retval;
}

static int gowin_read_register_command(struct pld_device *pld_device, uint32_t cmd, uint32_t *value)
{
	if (!pld_device)
		return ERROR_FAIL;

	struct gowin_pld_device *gowin_info = pld_device->driver_priv;

	if (!gowin_info || !gowin_info->tap)
		return ERROR_FAIL;

	return gowin_read_register(gowin_info->tap, cmd, value);
}

static int gowin_reload_command(struct pld_device *pld_device)
{
	if (!pld_device)
		return ERROR_FAIL;

	struct gowin_pld_device *gowin_info = pld_device->driver_priv;

	if (!gowin_info || !gowin_info->tap)
		return ERROR_FAIL;

	return gowin_reload(gowin_info->tap);
}

static int gowin_get_ipdbg_hub(int user_num, struct pld_device *pld_device, struct pld_ipdbg_hub *hub)
{
	if (!pld_device)
		return ERROR_FAIL;

	struct gowin_pld_device *pld_device_info = pld_device->driver_priv;

	if (!pld_device_info || !pld_device_info->tap)
		return ERROR_FAIL;

	hub->tap = pld_device_info->tap;

	if (user_num == 1) {
		hub->user_ir_code = USER1;
	} else if (user_num == 2) {
		hub->user_ir_code = USER2;
	} else {
		LOG_ERROR("gowin devices only have user register 1 & 2");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(gowin_read_status_command_handler)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct pld_device *device = get_pld_device_by_name_or_numstr(CMD_ARGV[0]);
	if (!device) {
		command_print(CMD, "pld device '#%s' is out of bounds or unknown", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	uint32_t status = 0;
	int retval = gowin_read_register_command(device, STATUS_REGISTER, &status);

	if (retval == ERROR_OK)
		command_print(CMD, "0x%8.8" PRIx32, status);

	return retval;
}

COMMAND_HANDLER(gowin_read_user_register_command_handler)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct pld_device *device = get_pld_device_by_name_or_numstr(CMD_ARGV[0]);
	if (!device) {
		command_print(CMD, "pld device '#%s' is out of bounds or unknown", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	uint32_t user_reg = 0;
	int retval = gowin_read_register_command(device, READ_USERCODE, &user_reg);

	if (retval == ERROR_OK)
		command_print(CMD, "0x%8.8" PRIx32, user_reg);

	return retval;
}

COMMAND_HANDLER(gowin_reload_command_handler)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct pld_device *device = get_pld_device_by_name_or_numstr(CMD_ARGV[0]);
	if (!device) {
		command_print(CMD, "pld device '#%s' is out of bounds or unknown", CMD_ARGV[0]);
		return ERROR_FAIL;
	}

	return gowin_reload_command(device);
}

static const struct command_registration gowin_exec_command_handlers[] = {
	{
		.name = "read_status",
		.mode = COMMAND_EXEC,
		.handler = gowin_read_status_command_handler,
		.help = "reading status register from FPGA",
		.usage = "pld_name",
	}, {
		.name = "read_user",
		.mode = COMMAND_EXEC,
		.handler = gowin_read_user_register_command_handler,
		.help = "reading user register from FPGA",
		.usage = "pld_name",
	}, {
		.name = "refresh",
		.mode = COMMAND_EXEC,
		.handler = gowin_reload_command_handler,
		.help = "reload bitstream from flash to SRAM",
		.usage = "pld_name",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration gowin_command_handler[] = {
	{
		.name = "gowin",
		.mode = COMMAND_ANY,
		.help = "gowin specific commands",
		.usage = "",
		.chain = gowin_exec_command_handlers
	},
	COMMAND_REGISTRATION_DONE
};

PLD_CREATE_COMMAND_HANDLER(gowin_pld_create_command)
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

	struct gowin_pld_device *gowin_info = malloc(sizeof(struct gowin_pld_device));
	if (!gowin_info) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}
	gowin_info->tap = tap;

	pld->driver_priv = gowin_info;

	return ERROR_OK;
}

struct pld_driver gowin_pld = {
	.name = "gowin",
	.commands = gowin_command_handler,
	.pld_create_command = &gowin_pld_create_command,
	.load = &gowin_load_to_sram,
	.get_ipdbg_hub = gowin_get_ipdbg_hub,
};
