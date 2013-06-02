/***************************************************************************
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifndef FLASH_NAND_FILEIO_H
#define FLASH_NAND_FILEIO_H

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
	struct fileio fileio;

	struct duration bench;
};

void nand_fileio_init(struct nand_fileio_state *state);
int nand_fileio_start(struct command_context *cmd_ctx,
		struct nand_device *nand, const char *filename, int filemode,
		struct nand_fileio_state *state);
int nand_fileio_cleanup(struct nand_fileio_state *state);
int nand_fileio_finish(struct nand_fileio_state *state);

COMMAND_HELPER(nand_fileio_parse_args, struct nand_fileio_state *state,
	struct nand_device **dev, enum fileio_access filemode,
	bool need_size, bool sw_ecc);

int nand_fileio_read(struct nand_device *nand, struct nand_fileio_state *s);

#endif	/* FLASH_NAND_FILEIO_H */
