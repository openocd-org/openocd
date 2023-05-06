// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2022 by Daniel Anselmi                                  *
 *   danselmi@gmx.ch                                                       *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include "lattice_bit.h"
#include "raw_bit.h"
#include "pld.h"
#include <helper/system.h>
#include <helper/log.h>
#include <helper/binarybuffer.h>

enum read_bit_state {
	SEEK_HEADER_START,
	SEEK_HEADER_END,
	SEEK_PREAMBLE,
	SEEK_ID,
	DONE,
};

static int lattice_read_bit_file(struct lattice_bit_file *bit_file, const char *filename, enum lattice_family_e family)
{
	int retval = cpld_read_raw_bit_file(&bit_file->raw_bit, filename);
	if (retval != ERROR_OK)
		return retval;

	bit_file->part = NULL;
	bit_file->has_id = false;
	enum read_bit_state state = SEEK_HEADER_START;
	for (size_t pos = 1; pos < bit_file->raw_bit.length && state != DONE; ++pos) {
		switch (state) {
		case SEEK_HEADER_START:
			if (bit_file->raw_bit.data[pos] == 0 && bit_file->raw_bit.data[pos - 1] == 0xff)
				state = SEEK_HEADER_END;
			break;
		case SEEK_HEADER_END:
			if (pos + 6 < bit_file->raw_bit.length &&
					strncmp((const char *)(bit_file->raw_bit.data + pos), "Part: ", 6) == 0) {
				bit_file->part = (const char *)bit_file->raw_bit.data + pos + 6;
				LOG_INFO("part found: %s\n", bit_file->part);
			} else if (bit_file->raw_bit.data[pos] == 0xff && bit_file->raw_bit.data[pos - 1] == 0) {
				bit_file->offset = pos;
				state = (family != LATTICE_ECP2 && family != LATTICE_ECP3) ? SEEK_PREAMBLE : DONE;
			}
			break;
		case SEEK_PREAMBLE:
			if (pos >= 4) {
				uint32_t preamble = be_to_h_u32(bit_file->raw_bit.data + pos - 3);
				switch (preamble) {
				case 0xffffbdb3:
					state = SEEK_ID;
					break;
				case 0xffffbfb3:
				case 0xffffbeb3:
					state = DONE;
					break;
				}
			}
			break;
		case SEEK_ID:
			if (pos + 7 < bit_file->raw_bit.length && bit_file->raw_bit.data[pos] == 0xe2) {
				bit_file->idcode = be_to_h_u32(&bit_file->raw_bit.data[pos + 4]);
				bit_file->has_id = true;
				state = DONE;
			}
			break;
		default:
			break;
		}
	}

	if (state != DONE) {
		LOG_ERROR("parsing bitstream failed");
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	for (size_t i = bit_file->offset; i < bit_file->raw_bit.length; i++)
		bit_file->raw_bit.data[i] = flip_u32(bit_file->raw_bit.data[i], 8);

	return ERROR_OK;
}

int lattice_read_file(struct lattice_bit_file *bit_file, const char *filename, enum lattice_family_e family)
{
	if (!filename || !bit_file)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* check if binary .bin or ascii .bit/.hex */
	const char *file_suffix_pos = strrchr(filename, '.');
	if (!file_suffix_pos) {
		LOG_ERROR("Unable to detect filename suffix");
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	if (strcasecmp(file_suffix_pos, ".bit") == 0)
		return lattice_read_bit_file(bit_file, filename, family);

	LOG_ERROR("Filetype not supported");
	return ERROR_PLD_FILE_LOAD_FAILED;
}
