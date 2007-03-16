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
#ifndef FILEIO_H
#define FILEIO_H

#define FILEIO_MAX_ERROR_STRING		(128)

/* make buffering of complete intel-hex format files optional
 * to account for resource-limited hosts
 */
#define FILEIO_BUFFER_COMPLETE_IHEX

#include <sys/stat.h>

enum fileio_pri_type
{
	FILEIO_TEXT = 0x1,
	FILEIO_IMAGE = 0x2,
};

enum fileio_sec_type
{
	FILEIO_PLAIN = 0x10,
	FILEIO_IHEX = 0x20,
/*
 * Possible future enhancements:
 * FILEIO_ELF,
 * FILEIO_SRECORD,
 */
};

enum fileio_location
{
	FILEIO_LOCAL,
/*
 * Possible future enhancements:
 * FILEIO_NFS,
 * FILEIO_BOOTP,
 * FILEIO_[XYZ]MODEM,
 * FILEIO_HTTP,
 * FILEIO_FTP,
 */
};

enum fileio_access
{
	FILEIO_READ,		/* open for reading, position at beginning */
	FILEIO_WRITE,		/* open for writing, position at beginning */
	FILEIO_READWRITE,	/* open for writing, position at beginning, allow reading */
	FILEIO_APPEND,		/* open for writing, position at end */
	FILEIO_APPENDREAD,	/* open for writing, position at end, allow reading */
};

typedef struct fileio_s
{
	char *url;
	char error_str[FILEIO_MAX_ERROR_STRING];
	long long size;
	enum fileio_pri_type pri_type;
	enum fileio_sec_type sec_type;
	enum fileio_location location;
	enum fileio_access access;
	void *location_private;
	void *pri_type_private;
	void *sec_type_private;
} fileio_t;

typedef struct fileio_text_s
{
} fileio_text_t;

typedef struct fileio_image_s
{
	u32 base_address;
	int has_start_address;
	u32 start_address;
} fileio_image_t;

typedef struct fileio_local_s
{
	FILE *file;
	struct stat file_stat;
} fileio_local_t;

typedef struct fileio_ihex_s
{
	u32 position;
	u32 raw_size;
#ifdef FILEIO_BUFFER_COMPLETE_IHEX
	u8 *buffer;
#endif
} fileio_ihex_t;

extern int fileio_identify_image_type(enum fileio_sec_type *sec_type, char *type_string);
extern int fileio_write(fileio_t *fileio, u32 size, u8 *buffer, u32 *size_written);
extern int fileio_read(fileio_t *fileio, u32 size, u8 *buffer, u32 *size_read);
extern int fileio_seek(fileio_t *fileio, u32 position);
extern int fileio_close(fileio_t *fileio);
extern int fileio_open(fileio_t *fileio, char *url, enum fileio_access access,
	enum fileio_pri_type pri_type, void *pri_info, enum fileio_sec_type sec_type);
	
#define ERROR_FILEIO_LOCATION_UNKNOWN	(-1200)
#define ERROR_FILEIO_NOT_FOUND			(-1201)
#define ERROR_FILEIO_OPERATION_FAILED		(-1202)
#define ERROR_FILEIO_ACCESS_NOT_SUPPORTED	(-1203)
#define ERROR_FILEIO_RESOURCE_TYPE_UNKNOWN	(-1204)
#define ERROR_FILEIO_OPERATION_NOT_SUPPORTED	(-1205)

#endif /* FILEIO_H */
