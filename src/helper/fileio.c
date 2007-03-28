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
	
	if (fileio->pri_type == FILEIO_IMAGE)
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

//#ifdef FILEIO_BUFFER_COMPLETE_IHEX
int fileio_ihex_buffer_complete(fileio_t *fileio)
{
	fileio_image_t *image = fileio->pri_type_private;
	fileio_ihex_t *ihex = fileio->sec_type_private;
	u32 raw_bytes_read, raw_bytes;
	int retval;
	u32 full_address = image->base_address;
	char *buffer = malloc(ihex->raw_size);
	u32 cooked_bytes = 0x0;
	
	ihex->raw_size = fileio->size;
	ihex->buffer = malloc(ihex->raw_size >> 1);
	
	if ((retval = fileio_dispatch_read(fileio, ihex->raw_size, (u8*)buffer, &raw_bytes_read)) != ERROR_OK)
	{
		free(buffer);
		ERROR("failed buffering IHEX file, read failed");
		return ERROR_FILEIO_OPERATION_FAILED;
	}
	
	if (raw_bytes_read != ihex->raw_size)
	{
		free(buffer);
		ERROR("failed buffering complete IHEX file, only partially read");
		return ERROR_FILEIO_OPERATION_FAILED;
	}
	
	raw_bytes = 0x0;
	while (raw_bytes < raw_bytes_read)
	{
		u32 count;
		u32 address;
		u32 record_type;
		u32 checksum;
		
		if (sscanf(&buffer[raw_bytes], ":%2x%4x%2x", &count, &address, &record_type) != 3)
		{
			snprintf(fileio->error_str, FILEIO_MAX_ERROR_STRING, "invalid IHEX record");
			return ERROR_FILEIO_OPERATION_FAILED;
		}
		raw_bytes += 9;
		
		if (record_type == 0)
		{
			if ((full_address & 0xffff) != address)
			{
				free(buffer);
				ERROR("can't handle non-linear IHEX file");
				snprintf(fileio->error_str, FILEIO_MAX_ERROR_STRING, "can't handle non-linear IHEX file");
				return ERROR_FILEIO_OPERATION_FAILED;
			}
			
			while (count-- > 0)
			{
				sscanf(&buffer[raw_bytes], "%2hhx", &ihex->buffer[cooked_bytes]);
				raw_bytes += 2;
				cooked_bytes += 1;
				full_address++;
			}
		}
		else if (record_type == 1)
		{
			free(buffer);
			fileio->size = cooked_bytes;
			return ERROR_OK;
		}
		else if (record_type == 4)
		{
			u16 upper_address;
			
			sscanf(&buffer[raw_bytes], "%4hx", &upper_address);
			raw_bytes += 4;
			
			if ((full_address >> 16) != upper_address)
			{
				free(buffer);
				ERROR("can't handle non-linear IHEX file");
				snprintf(fileio->error_str, FILEIO_MAX_ERROR_STRING, "can't handle non-linear IHEX file");
				return ERROR_FILEIO_OPERATION_FAILED;
			}
		}
		else if (record_type == 5)
		{
			u32 start_address;
			
			sscanf(&buffer[raw_bytes], "%8x", &start_address);
			raw_bytes += 8;
			
			image->has_start_address = 1;
			image->start_address = be_to_h_u32((u8*)&start_address);
		}
		else
		{
			free(buffer);
			ERROR("unhandled IHEX record type: %i", record_type);
			snprintf(fileio->error_str, FILEIO_MAX_ERROR_STRING, "unhandled IHEX record type: %i", record_type);
			return ERROR_FILEIO_OPERATION_FAILED;
		}
		
		sscanf(&buffer[raw_bytes], "%2x", &checksum);
		raw_bytes += 2;
		
		/* consume new-line character(s) */
		if ((buffer[raw_bytes] == '\n') || (buffer[raw_bytes] == '\r'))
			raw_bytes++;

		if ((buffer[raw_bytes] == '\n') || (buffer[raw_bytes] == '\r'))
			raw_bytes++;
	}

	free(buffer);
	ERROR("premature end of IHEX file, no end-of-file record found");
	snprintf(fileio->error_str, FILEIO_MAX_ERROR_STRING, "premature end of IHEX file, no end-of-file record found");
	return ERROR_FILEIO_OPERATION_FAILED;	
}
//#endif

int fileio_open(fileio_t *fileio, char *url, enum fileio_access access,
	enum fileio_pri_type pri_type, void *pri_info, enum fileio_sec_type sec_type)
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
	
	fileio->access = access;
	fileio->pri_type = pri_type;
	fileio->sec_type = sec_type;
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
	
	if (fileio->pri_type == FILEIO_TEXT)
	{
		/* do nothing for now */
		return ERROR_OK;
	}
	else if (fileio->pri_type == FILEIO_IMAGE)
	{
		fileio_image_t *image = malloc(sizeof(fileio_image_t));
		fileio_image_t *image_info = pri_info;
		
		fileio->pri_type_private = image;
		*image = *image_info; 
		
		if (fileio->sec_type == FILEIO_PLAIN)
		{
			fileio->sec_type_private = NULL;
		}
		else if (fileio->sec_type == FILEIO_IHEX)
		{
			fileio_ihex_t *fileio_ihex;
			
			if (fileio->access != FILEIO_READ)
			{
				ERROR("can't write/append to a IHEX file");
				snprintf(fileio->error_str, FILEIO_MAX_ERROR_STRING, "can't write/append to a IHEX file");
				fileio_close(fileio);
				return ERROR_FILEIO_OPERATION_FAILED;
			}
			
			fileio_ihex = malloc(sizeof(fileio_ihex_t));
			fileio->sec_type_private = fileio_ihex;
			
			fileio_ihex->position = 0;
			fileio_ihex->raw_size = fileio->size;
#ifdef FILEIO_BUFFER_COMPLETE_IHEX
			if (fileio_ihex_buffer_complete(fileio) != ERROR_OK)
			{
				fileio_close(fileio);
				return ERROR_FILEIO_OPERATION_FAILED;
			}
#endif
		}
	}
	
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
	
	if (fileio->pri_type == FILEIO_TEXT)
	{
		/* do nothing for now */
	}
	else if (fileio->pri_type == FILEIO_IMAGE)
	{
		if (fileio->sec_type == FILEIO_PLAIN)
		{
			/* nothing special to do for plain binary */
		}
		else if (fileio->sec_type == FILEIO_IHEX)
		{
			fileio_ihex_t *fileio_ihex = fileio->sec_type_private;
	
			if (fileio_ihex->buffer)
				free(fileio_ihex->buffer);
			
			free(fileio->sec_type_private);
		}
		
		free(fileio->pri_type_private);
	}
	
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

int fileio_dispatch_read(fileio_t *fileio, u32 size, u8 *buffer, u32 *size_read)
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

int fileio_read_ihex(fileio_t *fileio, u32 size, u8 *buffer, u32 *size_read)
{
	fileio_ihex_t *fileio_ihex = fileio->sec_type_private;

	if ((fileio_ihex->position + size) > fileio->size)
	{
		/* don't read past the end of the file */
		size = (fileio->size - fileio_ihex->position);
	}
	
#ifdef FILEIO_BUFFER_COMPLETE_IHEX
	memcpy(buffer, fileio_ihex->buffer + fileio_ihex->position, size);
	*size_read = size;
#endif

	return ERROR_OK;
}

int fileio_read(fileio_t *fileio, u32 size, u8 *buffer, u32 *size_read)
{
	if (fileio->sec_type == FILEIO_PLAIN)
	{
		return fileio_dispatch_read(fileio, size, buffer, size_read);
	}
	else if (fileio->sec_type == FILEIO_IHEX)
	{
		return fileio_read_ihex(fileio, size, buffer, size_read);
	}
	
	return ERROR_OK;
}

int fileio_local_write(fileio_t *fileio, u32 size, u8 *buffer, u32 *size_written)
{
	fileio_local_t *fileio_local = fileio->location_private;
	
	*size_written = fwrite(buffer, 1, size, fileio_local->file);
	
	return ERROR_OK;
}

int fileio_dispatch_write(fileio_t *fileio, u32 size, u8 *buffer, u32 *size_written)
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

int fileio_write(fileio_t *fileio, u32 size, u8 *buffer, u32 *size_written)
{
	int retval = ERROR_FILEIO_OPERATION_NOT_SUPPORTED;
	if (fileio->sec_type == FILEIO_PLAIN)
	{
		retval = fileio_dispatch_write(fileio, size, buffer, size_written);
	}
	else if (fileio->sec_type == FILEIO_IHEX)
	{
		return ERROR_FILEIO_OPERATION_NOT_SUPPORTED;
	}
	
	if (retval != ERROR_OK)
		return retval;
		
	fileio->size += size;
	
	return ERROR_OK;
}

int fileio_identify_image_type(enum fileio_sec_type *sec_type, char *type_string)
{
	if (type_string)
	{
		if (!strcmp(type_string, "bin"))
		{
			*sec_type = FILEIO_PLAIN;
		}
		else if (!strcmp(type_string, "ihex"))
		{
			*sec_type = FILEIO_IHEX;
		}
		else
		{
			return ERROR_FILEIO_RESOURCE_TYPE_UNKNOWN;
		}
	}
	else
	{
		*sec_type = FILEIO_PLAIN;
	}
	
	return ERROR_OK;
}
