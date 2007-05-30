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
	u32 full_address = 0x0;
	char *buffer = malloc(fileio->size);
	u32 cooked_bytes;
	int i;
	
	/* we can't determine the number of sections that we'll have to create ahead of time,
	 * so we locally hold them until parsing is finished */
	image_section_t section[IMAGE_MAX_SECTIONS];
	u8 *section_pointer[IMAGE_MAX_SECTIONS];
	
	if ((retval = fileio_read(fileio, fileio->size, (u8*)buffer, &raw_bytes_read)) != ERROR_OK)
	{
		free(buffer);
		ERROR("failed buffering IHEX file, read failed");
		return ERROR_FILEIO_OPERATION_FAILED;
	}
	
	if (raw_bytes_read != fileio->size)
	{
		free(buffer);
		ERROR("failed buffering complete IHEX file, only partially read");
		return ERROR_FILEIO_OPERATION_FAILED;
	}

	ihex->buffer = malloc(fileio->size >> 1);
	raw_bytes = 0x0;
	cooked_bytes = 0x0;
	image->num_sections = 0;
	section_pointer[image->num_sections] = &ihex->buffer[cooked_bytes];
	section[image->num_sections].base_address = 0x0;
	section[image->num_sections].size = 0x0;
	section[image->num_sections].flags = 0;
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
		
		if (record_type == 0) /* Data Record */
		{
			if ((full_address & 0xffff) != address)
			{
				/* we encountered a nonconsecutive location, create a new section,
				 * unless the current section has zero size, in which case this specifies
				 * the current section's base address
				 */
				if (section[image->num_sections].size != 0)
				{
					image->num_sections++;
					section[image->num_sections].size = 0x0;
					section[image->num_sections].flags = 0;
					section_pointer[image->num_sections] = &ihex->buffer[cooked_bytes];
				}
				section[image->num_sections].base_address =
					(full_address & 0xffff0000) | address;
				full_address = (full_address & 0xffff0000) | address;
			}
			
			while (count-- > 0)
			{
				sscanf(&buffer[raw_bytes], "%2hhx", &ihex->buffer[cooked_bytes]);
				raw_bytes += 2;
				cooked_bytes += 1;
				section[image->num_sections].size += 1;
				full_address++;
			}
		}
		else if (record_type == 1) /* End of File Record */
		{
			/* finish the current section */
			image->num_sections++;
			
			/* copy section information */
			ihex->section_pointer = malloc(sizeof(u8*) * image->num_sections);
			image->sections = malloc(sizeof(image_section_t) * image->num_sections);
			for (i = 0; i < image->num_sections; i++)
			{
				ihex->section_pointer[i] = section_pointer[i];
				image->sections[i].base_address = section[i].base_address +
					((image->base_address_set) ? image->base_address : 0);
				image->sections[i].size = section[i].size;
				image->sections[i].flags = section[i].flags;
			}
			
			free(buffer);
			return ERROR_OK;
		}
		else if (record_type == 4) /* Extended Linear Address Record */
		{
			u16 upper_address;
			
			sscanf(&buffer[raw_bytes], "%4hx", &upper_address);
			raw_bytes += 4;
			
			if ((full_address >> 16) != upper_address)
			{
				/* we encountered a nonconsecutive location, create a new section,
				 * unless the current section has zero size, in which case this specifies
				 * the current section's base address
				 */
				if (section[image->num_sections].size != 0)
				{
					image->num_sections++;
					section[image->num_sections].size = 0x0;
					section[image->num_sections].flags = 0;
					section_pointer[image->num_sections] = &ihex->buffer[cooked_bytes];
				}
				section[image->num_sections].base_address = 
					(full_address & 0xffff) | (upper_address << 16);
				full_address = (full_address & 0xffff) | (upper_address << 16);
			}
		}
		else if (record_type == 5) /* Start Linear Address Record */
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
		
		image->num_sections = 1;
		image->sections = malloc(sizeof(image_section_t));
		image->sections[0].base_address = 0x0;
		image->sections[0].size = image_binary->fileio.size;
		image->sections[0].flags = 0;
		
		if (image->base_address_set == 1)
			image->sections[0].base_address = image->base_address;
		
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

int image_read_section(image_t *image, int section, u32 offset, u32 size, u8 *buffer, u32 *size_read)
{
	int retval;
	
	if (image->type == IMAGE_BINARY)
	{
		image_binary_t *image_binary = image->type_private;
		
		/* only one section in a plain binary */
		if (section != 0)
			return ERROR_INVALID_ARGUMENTS;
			
		if ((offset > image->sections[0].size) || (offset + size > image->sections[0].size))
			return ERROR_INVALID_ARGUMENTS;
		
		/* seek to offset */
		if ((retval = fileio_seek(&image_binary->fileio, offset)) != ERROR_OK)
		{
			strncpy(image->error_str, image_binary->fileio.error_str, IMAGE_MAX_ERROR_STRING);
			return retval;
		}
		
		/* return requested bytes */
		if ((retval = fileio_read(&image_binary->fileio, size, buffer, size_read)) != ERROR_OK)
		{
			strncpy(image->error_str, image_binary->fileio.error_str, IMAGE_MAX_ERROR_STRING);
			return retval;
		}
	}
	else if (image->type == IMAGE_IHEX)
	{
		image_ihex_t *image_ihex = image->type_private;

		memcpy(buffer, image_ihex->section_pointer[section] + offset, size);
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
		
		if (image_ihex->section_pointer)
			free(image_ihex->section_pointer);
		
		if (image_ihex->buffer)
			free(image_ihex->buffer);
	}
	else if (image->type == IMAGE_MEMORY)
	{
		/* do nothing for now */
	}

	if (image->type_private)
		free(image->type_private);
	
	if (image->sections)
		free(image->sections);
	
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
