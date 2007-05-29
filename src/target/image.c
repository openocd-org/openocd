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

#include <stdlib.h>
#include <string.h>

#include "image.h"

#include "types.h"
#include "replacements.h"
#include "log.h"

#include "fileio.h"
#include "target.h"

int image_ihex_buffer_complete(image_t *image)
{
	image_ihex_t *ihex = image->type_private;
	fileio_t *fileio = &ihex->fileio;
	u32 raw_bytes_read, raw_bytes;
	int retval;
	u32 full_address = image->base_address;
	char *buffer = malloc(ihex->raw_size);
	u32 cooked_bytes = 0x0;
	
	ihex->raw_size = fileio->size;
	ihex->buffer = malloc(ihex->raw_size >> 1);
	
	if ((retval = fileio_read(fileio, ihex->raw_size, (u8*)buffer, &raw_bytes_read)) != ERROR_OK)
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
	
	image->size = 0x0;
	raw_bytes = 0x0;
	while (raw_bytes < raw_bytes_read)
	{
		u32 count;
		u32 address;
		u32 record_type;
		u32 checksum;
		
		if (sscanf(&buffer[raw_bytes], ":%2x%4x%2x", &count, &address, &record_type) != 3)
		{
			return ERROR_IMAGE_FORMAT_ERROR;
		}
		raw_bytes += 9;
		
		if (record_type == 0)
		{
			if ((full_address & 0xffff) != address)
			{
				free(buffer);
				ERROR("can't handle non-linear IHEX file");
				return ERROR_IMAGE_FORMAT_ERROR;
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
			image->size = cooked_bytes;
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
				return ERROR_IMAGE_FORMAT_ERROR;
			}
		}
		else if (record_type == 5)
		{
			u32 start_address;
			
			sscanf(&buffer[raw_bytes], "%8x", &start_address);
			raw_bytes += 8;
			
			image->start_address_set = 1;
			image->start_address = be_to_h_u32((u8*)&start_address);
		}
		else
		{
			free(buffer);
			ERROR("unhandled IHEX record type: %i", record_type);
			return ERROR_IMAGE_FORMAT_ERROR;
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
	return ERROR_IMAGE_FORMAT_ERROR;
}

int image_open(image_t *image, void *source, enum fileio_access access)
{
	int retval = ERROR_OK;
	
	if (image->type == IMAGE_BINARY)
	{
		image_binary_t *image_binary;
		char *url = source;
		
		image_binary = image->type_private = malloc(sizeof(image_binary_t));
		
		if ((retval = fileio_open(&image_binary->fileio, url, access, FILEIO_BINARY)) != ERROR_OK)
		{
			strncpy(image->error_str, image_binary->fileio.error_str, IMAGE_MAX_ERROR_STRING); 
			ERROR(image->error_str);
			return retval;
		}
		
		if (access == FILEIO_WRITE)
			image->size = 0;
		else
			image->size = image_binary->fileio.size;
		
		return ERROR_OK;
	}
	else if (image->type == IMAGE_IHEX)
	{
		image_ihex_t *image_ihex;
		char *url = source;
		
		if (access != FILEIO_READ)
		{
			snprintf(image->error_str, IMAGE_MAX_ERROR_STRING,
				"can't open IHEX file for writing");
			ERROR(image->error_str);
			return ERROR_FILEIO_ACCESS_NOT_SUPPORTED;
		}
		
		image_ihex = image->type_private = malloc(sizeof(image_ihex_t));
		
		if ((retval = fileio_open(&image_ihex->fileio, url, FILEIO_READ, FILEIO_TEXT)) != ERROR_OK)
		{
			strncpy(image->error_str, image_ihex->fileio.error_str, IMAGE_MAX_ERROR_STRING); 
			ERROR(image->error_str);
			return retval;
		}
		
		image_ihex->position = 0;
		image_ihex->raw_size = image_ihex->fileio.size;
		
		if ((retval = image_ihex_buffer_complete(image)) != ERROR_OK)
		{
			snprintf(image->error_str, IMAGE_MAX_ERROR_STRING,
				"failed buffering IHEX image, check daemon output for additional information");
			ERROR(image->error_str);
			fileio_close(&image_ihex->fileio);
			return retval;
		}
	}
	else if (image->type == IMAGE_MEMORY)
	{
		image_memory_t *image_memory;
		target_t *target = source;
		
		image_memory = image->type_private = malloc(sizeof(image_memory_t));
		
		image_memory->target = target;
	}
	
	return retval;
};

int image_read(image_t *image, u32 size, u8 *buffer, u32 *size_read)
{
	int retval;
	
	if (image->type == IMAGE_BINARY)
	{
		image_binary_t *image_binary = image->type_private;
		
		if ((retval = fileio_read(&image_binary->fileio, size, buffer, size_read)) != ERROR_OK)
		{
			strncpy(image->error_str, image_binary->fileio.error_str, IMAGE_MAX_ERROR_STRING);
			return retval;
		}
	}
	else if (image->type == IMAGE_IHEX)
	{
		image_ihex_t *image_ihex = image->type_private;
	
		if ((image_ihex->position + size) > image->size)
		{
			/* don't read past the end of the file */
			size = (image->size - image_ihex->position);
		}
	
		memcpy(buffer, image_ihex->buffer + image_ihex->position, size);
		image_ihex->position += size;
		*size_read = size;
		image->error_str[0] = '\0';
		
		return ERROR_OK;
	}
	else if (image->type == IMAGE_MEMORY)
	{
		/* TODO: handle target memory pseudo image */
	}
	
	return ERROR_OK;
}

int image_write(image_t *image, u32 size, u8 *buffer, u32 *size_written)
{
	int retval = ERROR_FILEIO_OPERATION_NOT_SUPPORTED;
	
	if (image->type == IMAGE_BINARY)
	{
		image_binary_t *image_binary = image->type_private;
		
		if ((retval = fileio_write(&image_binary->fileio, size, buffer, size_written)) != ERROR_OK)
		{
			strncpy(image->error_str, image_binary->fileio.error_str, IMAGE_MAX_ERROR_STRING);
			return retval;
		}
	}
	else if (image->type == IMAGE_IHEX)
	{
		return ERROR_FILEIO_OPERATION_NOT_SUPPORTED;
	}
	else if (image->type == IMAGE_MEMORY)
	{
		/* TODO: handle target memory pseudo image */
	}
	
	if (retval != ERROR_OK)
		return retval;
		
	image->size += size;
	
	return ERROR_OK;
}

int image_close(image_t *image)
{
	if (image->type == IMAGE_BINARY)
	{
		image_binary_t *image_binary = image->type_private;
		
		fileio_close(&image_binary->fileio);
	}
	else if (image->type == IMAGE_IHEX)
	{
		image_ihex_t *image_ihex = image->type_private;
		
		fileio_close(&image_ihex->fileio);
		
		if (image_ihex->buffer)
			free(image_ihex->buffer);
	}
	else if (image->type == IMAGE_MEMORY)
	{
		/* do nothing for now */
	}

	free(image->type_private);
	
	return ERROR_OK;
}

int identify_image_type(image_type_t *type, char *type_string)
{
	if (type_string)
	{
		if (!strcmp(type_string, "bin"))
		{
			*type = IMAGE_BINARY;
		}
		else if (!strcmp(type_string, "ihex"))
		{
			*type = IMAGE_IHEX;
		}
		else
		{
			return ERROR_IMAGE_TYPE_UNKNOWN;
		}
	}
	else
	{
		*type = IMAGE_BINARY;
	}
	
	return ERROR_OK;
}
