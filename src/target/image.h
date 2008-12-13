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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef HAVE_ELF_H
#include <elf.h>
#endif
#include "replacements.h"
#include "fileio.h"
#include "target.h"

#define IMAGE_MAX_ERROR_STRING		(256)
#define IMAGE_MAX_SECTIONS			(128)

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

typedef struct image_section_s
{
	u32 base_address;
	u32 size;
	int flags;
	void *private;		/* private data */
} image_section_t;

typedef struct image_s
{
	image_type_t type;		/* image type (plain, ihex, ...) */
	void *type_private;		/* type private data */
	int num_sections;		/* number of sections contained in the image */
	image_section_t *sections;	/* array of sections */
	int base_address_set;	/* whether the image has a base address set (for relocation purposes) */
	int base_address;		/* base address, if one is set */
	int start_address_set;	/* whether the image has a start address (entry point) associated */
	u32 start_address;		/* start address, if one is set */
} image_t;

typedef struct image_binary_s
{
	fileio_t fileio;
} image_binary_t;

typedef struct image_ihex_s
{
	fileio_t fileio;
	u8 *buffer;
} image_ihex_t;

typedef struct image_memory_s
{
	target_t *target;
	u8 *cache;
	u32 cache_address;
} image_memory_t;

typedef struct fileio_elf_s
{
	fileio_t fileio;
	Elf32_Ehdr *header;
	Elf32_Phdr *segments;
	u32 segment_count;
	u8 endianness;
} image_elf_t;

typedef struct image_mot_s
{
	fileio_t fileio;
	u8 *buffer;
} image_mot_t;

extern int image_open(image_t *image, char *url, char *type_string);
extern int image_read_section(image_t *image, int section, u32 offset, u32 size, u8 *buffer, u32 *size_read);
extern void image_close(image_t *image);
extern int image_add_section(image_t *image, u32 base, u32 size, int flags, u8 *data);

extern int image_calculate_checksum(u8* buffer, u32 nbytes, u32* checksum);

#define ERROR_IMAGE_FORMAT_ERROR	(-1400)
#define ERROR_IMAGE_TYPE_UNKNOWN	(-1401)
#define ERROR_IMAGE_TEMPORARILY_UNAVAILABLE		(-1402)
#define ERROR_IMAGE_CHECKSUM		(-1403)

#endif /* IMAGE_H */
