/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
 ***************************************************************************/

#ifndef OPENOCD_FLASH_NAND_FILEIO_H
#define OPENOCD_FLASH_NAND_FILEIO_H

#include <helper/time_support.h>
#include <helper/fileio.h>

struct nand_fileio_state {
	uint32_t address;
	uint32_t size;

	uint8_t *page;
	uint32_t page_size;

	enum oob_formats oob_format;
	uint8_t *oob;
	uint32_t oob_size;

	const int *eccpos;

	bool file_opened;
	struct fileio *fileio;

	struct duration bench;
};

void nand_fileio_init(struct nand_fileio_state *state);
int nand_fileio_start(struct command_invocation *cmd,
		struct nand_device *nand, const char *filename, int filemode,
		struct nand_fileio_state *state);
int nand_fileio_cleanup(struct nand_fileio_state *state);
int nand_fileio_finish(struct nand_fileio_state *state);

COMMAND_HELPER(nand_fileio_parse_args, struct nand_fileio_state *state,
	struct nand_device **dev, enum fileio_access filemode,
	bool need_size, bool sw_ecc);

int nand_fileio_read(struct nand_device *nand, struct nand_fileio_state *s);

#endif /* OPENOCD_FLASH_NAND_FILEIO_H */
