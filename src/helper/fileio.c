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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "types.h"
#include "replacements.h"
#include "log.h"

#include "fileio.h"

#include <stdio.h>
#include <string.h>
#include <unistd.h>
#include <stdlib.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <errno.h>
#include <ctype.h>

int fileio_close(fileio_t *fileio);
int fileio_dispatch_read(fileio_t *fileio, u32 size, u8 *buffer, u32 *size_read);

int fileio_open_local(fileio_t *fileio)
{
	fileio_local_t *fileio_local = malloc(sizeof(fileio_local_t));
	char access[4];
	
	fileio->location_private = fileio_local;
	
	if ((fileio->access != FILEIO_WRITE) && (fileio->access != FILEIO_READWRITE))
	{
		if (stat(fileio->url, &fileio_local->file_stat) == -1)
		{
			free(fileio_local);
			snprintf(fileio->error_str, FILEIO_MAX_ERROR_STRING,
				"couldn't stat() %s: %s", fileio->url, strerror(errno));
			return ERROR_FILEIO_NOT_FOUND;
		}
	
		if (S_ISDIR(fileio_local->file_stat.st_mode))
		{
			free(fileio_local);
			snprintf(fileio->error_str, FILEIO_MAX_ERROR_STRING, "%s is a directory", fileio->url);
			return ERROR_FILEIO_NOT_FOUND;
		}
	}
	
	switch (fileio->access)
	{
		case FILEIO_READ:
			strcpy(access, "r");
			break;
		case FILEIO_WRITE:
			strcpy(access, "w");
			break;
		case FILEIO_READWRITE:
			strcpy(access, "w+");
			break;
		case FILEIO_APPEND:
			strcpy(access, "a");	
			break;
		case FILEIO_APPENDREAD:
			strcpy(access, "a+");	
			break;
		default:
			free(fileio_local);
			ERROR("BUG: access neither read, write nor readwrite");
			return ERROR_INVALID_ARGUMENTS;
	}
	
	if (fileio->access == FILEIO_READ)
	{
		if (fileio_local->file_stat.st_size == 0)
		{
			/* tried to open an empty file for reading */
			free(fileio_local);
			snprintf(fileio->error_str, FILEIO_MAX_ERROR_STRING, "empty file %s", fileio->url);
			return ERROR_FILEIO_OPERATION_FAILED;
		}
	}
	
	if (fileio->type == FILEIO_BINARY)
		strcat(access, "b");
	
	if (!(fileio_local->file = fopen(fileio->url, access)))
	{
		free(fileio_local);
		snprintf(fileio->error_str, FILEIO_MAX_ERROR_STRING, "couldn't open %s", fileio->url);
		return ERROR_FILEIO_OPERATION_FAILED;
	}
	
	if ((fileio->access != FILEIO_WRITE) || (fileio->access == FILEIO_READWRITE))
	{
		fileio->size = fileio_local->file_stat.st_size;
	}
	else
	{
		fileio->size = 0x0;
	}
	
	return ERROR_OK;
}

int fileio_open(fileio_t *fileio, char *url, enum fileio_access access,	enum fileio_type type)
{
	int retval = ERROR_OK;
	char *resource_identifier = NULL;

	/* try to identify file location */
	if ((resource_identifier = strstr(url, "bootp://")) && (resource_identifier == url))
	{
		ERROR("bootp resource location isn't supported yet");
		return ERROR_FILEIO_RESOURCE_TYPE_UNKNOWN;
	}
	else if ((resource_identifier = strstr(url, "tftp://")) && (resource_identifier == url))
	{
		ERROR("tftp resource location isn't supported yet");
		return ERROR_FILEIO_RESOURCE_TYPE_UNKNOWN;
	}
	else
	{
		/* default to local files */
		fileio->location = FILEIO_LOCAL;
	}
	
	fileio->type = type;
	fileio->access = access;
	fileio->url = strdup(url);
	
	switch (fileio->location)
	{
		case FILEIO_LOCAL:
			retval = fileio_open_local(fileio);
			break;
		default:
			ERROR("BUG: should never get here");
			exit(-1);
	}
	
	if (retval != ERROR_OK)
		return retval;
	
	return ERROR_OK;
}

int fileio_close_local(fileio_t *fileio)
{
	int retval;
	fileio_local_t *fileio_local = fileio->location_private;
	
	if ((retval = fclose(fileio_local->file)) != 0)
	{
		if (retval == EBADF)
		{
			snprintf(fileio->error_str, FILEIO_MAX_ERROR_STRING, "BUG: fileio_local->file not a valid file descriptor");
		}
		else
		{
			snprintf(fileio->error_str, FILEIO_MAX_ERROR_STRING, "couldn't close %s: %s", fileio->url, strerror(errno));
		}

		return ERROR_FILEIO_OPERATION_FAILED;
	}
	
	free(fileio->location_private);
	
	return ERROR_OK;
}

int fileio_close(fileio_t *fileio)
{
	int retval;
	
	switch (fileio->location)
	{
		case FILEIO_LOCAL:
			retval = fileio_close_local(fileio);
			break;
		default:
			ERROR("BUG: should never get here");
			retval = ERROR_FILEIO_OPERATION_FAILED;
	}
	
	if (retval != ERROR_OK)
		return retval;
	
	free(fileio->url);
	
	return ERROR_OK;
}

int fileio_seek_local(fileio_t *fileio, u32 position)
{
	int retval;
	fileio_local_t *fileio_local = fileio->location_private;
	
	if ((retval = fseek(fileio_local->file, position, SEEK_SET)) != 0)
	{
		snprintf(fileio->error_str, FILEIO_MAX_ERROR_STRING, "couldn't seek file %s: %s", fileio->url, strerror(errno));
		return ERROR_FILEIO_OPERATION_FAILED;
	}
	
	return ERROR_OK;
}

int fileio_seek(fileio_t *fileio, u32 position)
{
	switch (fileio->location)
	{
		case FILEIO_LOCAL:
			return fileio_seek_local(fileio, position);
			break;
		default:
			ERROR("BUG: should never get here");
	}
	
	return ERROR_OK;
}

int fileio_local_read(fileio_t *fileio, u32 size, u8 *buffer, u32 *size_read)
{
	fileio_local_t *fileio_local = fileio->location_private;
	
	*size_read = fread(buffer, 1, size, fileio_local->file);
	
	return ERROR_OK;
}

int fileio_read(fileio_t *fileio, u32 size, u8 *buffer, u32 *size_read)
{
	switch (fileio->location)
	{
		case FILEIO_LOCAL:
			return fileio_local_read(fileio, size, buffer, size_read);
			break;
		default:
			ERROR("BUG: should never get here");
			exit(-1);
	}
}

int fileio_read_u32(fileio_t *fileio, u32 *data)
{
	u8 buf[4];
	u32 size_read;
	int retval;
	
	switch (fileio->location)
	{
		case FILEIO_LOCAL:
			if ((retval = fileio_local_read(fileio, 4, buf, &size_read)) != ERROR_OK)
				return retval;
			*data = be_to_h_u32(buf);
			break;
		default:
			ERROR("BUG: should never get here");
			exit(-1);
	}
	
	return ERROR_OK;
}

int fileio_local_write(fileio_t *fileio, u32 size, u8 *buffer, u32 *size_written)
{
	fileio_local_t *fileio_local = fileio->location_private;
	
	*size_written = fwrite(buffer, 1, size, fileio_local->file);
	
	return ERROR_OK;
}

int fileio_write(fileio_t *fileio, u32 size, u8 *buffer, u32 *size_written)
{
	switch (fileio->location)
	{
		case FILEIO_LOCAL:
			return fileio_local_write(fileio, size, buffer, size_written);
			break;
		default:
			ERROR("BUG: should never get here");
	}
	
	return ERROR_OK;
}

int fileio_write_u32(fileio_t *fileio, u32 data)
{
	u8 buf[4];
	u32 size_written;
	int retval;
	
	h_u32_to_be(buf, data);
	
	switch (fileio->location)
	{
		case FILEIO_LOCAL:
			if ((retval = fileio_local_write(fileio, 4, buf, &size_written)) != ERROR_OK)
				return retval;
			break;
		default:
			ERROR("BUG: should never get here");
	}
	
	return ERROR_OK;
}
