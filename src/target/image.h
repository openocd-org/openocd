/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef IMAGE_H
#define IMAGE_H

#include "fileio.h"

#ifdef HAVE_ELF_H
#include <elf.h>
#endif

#define IMAGE_MAX_ERROR_STRING		(256)
#define IMAGE_MAX_SECTIONS			(512)

#define IMAGE_MEMORY_CACHE_SIZE		(2048)

typedef enum image_type
{
	IMAGE_BINARY,	/* plain binary */
	IMAGE_IHEX,		/* intel hex-record format */
	IMAGE_MEMORY,	/* target-memory pseudo-image */
	IMAGE_ELF,		/* ELF binary */
	IMAGE_SRECORD,	/* motorola s19 */
	IMAGE_BUILDER,	/* when building a new image */
} image_type_t;

struct image_section
{
	uint32_t base_address;
	uint32_t size;
	int flags;
	void *private;		/* private data */
};

typedef struct image_s
{
	image_type_t type;		/* image type (plain, ihex, ...) */
	void *type_private;		/* type private data */
	int num_sections;		/* number of sections contained in the image */
	struct image_section *sections;	/* array of sections */
	int base_address_set;	/* whether the image has a base address set (for relocation purposes) */
	int base_address;		/* base address, if one is set */
	int start_address_set;	/* whether the image has a start address (entry point) associated */
	uint32_t start_address;		/* start address, if one is set */
} image_t;

struct image_binary
{
	struct fileio fileio;
};

struct image_ihex
{
	struct fileio fileio;
	uint8_t *buffer;
};

typedef struct image_memory_s
{
	struct target_s *target;
	uint8_t *cache;
	uint32_t cache_address;
} image_memory_t;

typedef struct fileio_elf_s
{
	struct fileio fileio;
	Elf32_Ehdr *header;
	Elf32_Phdr *segments;
	uint32_t segment_count;
	uint8_t endianness;
} image_elf_t;

typedef struct image_mot_s
{
	struct fileio fileio;
	uint8_t *buffer;
} image_mot_t;

int image_open(image_t *image, const char *url, const char *type_string);
int image_read_section(image_t *image, int section, uint32_t offset,
		uint32_t size, uint8_t *buffer, uint32_t *size_read);
void image_close(image_t *image);

int image_add_section(image_t *image, uint32_t base, uint32_t size,
		int flags, uint8_t *data);

int image_calculate_checksum(uint8_t* buffer, uint32_t nbytes,
		uint32_t* checksum);

#define ERROR_IMAGE_FORMAT_ERROR	(-1400)
#define ERROR_IMAGE_TYPE_UNKNOWN	(-1401)
#define ERROR_IMAGE_TEMPORARILY_UNAVAILABLE		(-1402)
#define ERROR_IMAGE_CHECKSUM		(-1403)

#endif /* IMAGE_H */
