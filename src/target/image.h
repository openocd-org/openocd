/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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
#include "target.h"

#define IMAGE_MAX_ERROR_STRING		(128)
#define IMAGE_MAX_SECTIONS			(128)

typedef enum image_type
{
    IMAGE_BINARY,	/* plain binary */
    IMAGE_IHEX,		/* intel hex-record format */
    IMAGE_MEMORY,	/* target-memory pseudo-image */
/*
 * Possible future enhancements:
 * IMAGE_ELF,
 * IMAGE_SRECORD,
 */
} image_type_t;

typedef struct image_section_s
{
	u32 base_address;
	u32 size;
	int flags;
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
	char error_str[IMAGE_MAX_ERROR_STRING];
} image_t;

typedef struct image_binary_s
{
	fileio_t fileio;
} image_binary_t;

typedef struct image_ihex_s
{
	fileio_t fileio;
	u8 *buffer;
	u8 **section_pointer;
} image_ihex_t;

typedef struct image_memory_s
{
	target_t *target;
} image_memory_t;

extern int image_open(image_t *image, void *source, enum fileio_access access);
extern int image_read_section(image_t *image, int section, u32 offset, u32 size, u8 *buffer, u32 *size_read);
extern int image_close(image_t *image);
extern int identify_image_type(image_type_t *type, char *type_string);

#define ERROR_IMAGE_FORMAT_ERROR	(-1400)
#define ERROR_IMAGE_TYPE_UNKNOWN	(-1401)

#endif /* IMAGE_H */
