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
#ifndef FILEIO_H
#define FILEIO_H

#define FILEIO_MAX_ERROR_STRING		(128)

#include "types.h"

#include <stdio.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <ctype.h>

enum fileio_type
{
	FILEIO_TEXT,
	FILEIO_BINARY,
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
	long long size;
	enum fileio_type type;
	enum fileio_access access;
	FILE *file;
} fileio_t;

extern int fileio_write(fileio_t *fileio, u32 size, u8 *buffer, u32 *size_written);
extern int fileio_read(fileio_t *fileio, u32 size, u8 *buffer, u32 *size_read);
extern int fileio_fgets(fileio_t *fileio, u32 size, char *buffer);
extern int fileio_seek(fileio_t *fileio, u32 position);
extern int fileio_close(fileio_t *fileio);
extern int fileio_open(fileio_t *fileio, char *url, enum fileio_access access, enum fileio_type type);
extern int fileio_read_u32(fileio_t *fileio, u32 *data);
extern int fileio_write_u32(fileio_t *fileio, u32 data);

#define ERROR_FILEIO_LOCATION_UNKNOWN	(-1200)
#define ERROR_FILEIO_NOT_FOUND			(-1201)
#define ERROR_FILEIO_OPERATION_FAILED		(-1202)
#define ERROR_FILEIO_ACCESS_NOT_SUPPORTED	(-1203)
#define ERROR_FILEIO_RESOURCE_TYPE_UNKNOWN	(-1204)
#define ERROR_FILEIO_OPERATION_NOT_SUPPORTED	(-1205)

#endif /* FILEIO_H */
