// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2022 by Daniel Anselmi                                  *
 *   danselmi@gmx.ch                                                       *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "raw_bit.h"
#include "pld.h"

#include <helper/system.h>
#include <helper/log.h>


int cpld_read_raw_bit_file(struct raw_bit_file *bit_file, const char *filename)
{
	FILE *input_file = fopen(filename, "rb");

	if (!input_file) {
		LOG_ERROR("Couldn't open %s: %s", filename, strerror(errno));
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	fseek(input_file, 0, SEEK_END);
	long length = ftell(input_file);
	fseek(input_file, 0, SEEK_SET);

	if (length < 0) {
		fclose(input_file);
		LOG_ERROR("Failed to get length of file %s: %s", filename, strerror(errno));
		return ERROR_PLD_FILE_LOAD_FAILED;
	}
	bit_file->length = (size_t)length;

	bit_file->data = malloc(bit_file->length);
	if (!bit_file->data) {
		fclose(input_file);
		LOG_ERROR("Out of memory");
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	size_t read_count = fread(bit_file->data, sizeof(char), bit_file->length, input_file);
	fclose(input_file);
	if (read_count != bit_file->length) {
		free(bit_file->data);
		bit_file->data = NULL;
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	return ERROR_OK;
}
