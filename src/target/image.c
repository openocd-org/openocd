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
#ifdef HAVE_ELF_H
#include <elf.h>
#endif

#include "image.h"

#include "types.h"
#include "replacements.h"
#include "log.h"

#include "fileio.h"
#include "target.h"

/* convert ELF header field to host endianness */
#define field16(elf,field)\
	((elf->endianness==ELFDATA2LSB)? \
		le_to_h_u16((u8*)&field):be_to_h_u16((u8*)&field)) 

#define field32(elf,field)\
	((elf->endianness==ELFDATA2LSB)? \
		le_to_h_u32((u8*)&field):be_to_h_u32((u8*)&field)) 

static int autodetect_image_type(image_t *image, char *url)
{
	int retval;
	fileio_t fileio;
	u32 read_bytes;
	u8 buffer[9];
	
	/* read the first 4 bytes of image */
	if ((retval = fileio_open(&fileio, url, FILEIO_READ, FILEIO_BINARY)) != ERROR_OK)
	{
		snprintf(image->error_str, IMAGE_MAX_ERROR_STRING, "cannot open image: %s", fileio.error_str); 
		ERROR(image->error_str);
		return retval;
	}
	if ((retval = fileio_read(&fileio, 9, buffer, &read_bytes)) != ERROR_OK)
	{
		snprintf(image->error_str, IMAGE_MAX_ERROR_STRING, "cannot read image header: %s", fileio.error_str);
		ERROR(image->error_str);
		return ERROR_FILEIO_OPERATION_FAILED;
	}
	if (read_bytes != 9)
	{
		snprintf(image->error_str, IMAGE_MAX_ERROR_STRING, "cannot read image, only partially read");
		ERROR(image->error_str);
		return ERROR_FILEIO_OPERATION_FAILED;
	}
	fileio_close(&fileio);

	/* check header against known signatures */
	if (strncmp((char*)buffer,ELFMAG,SELFMAG)==0)
	{
		DEBUG("ELF image detected.");
		image->type = IMAGE_ELF;
	}
	else if  ((buffer[0]==':') /* record start byte */
	        &&(isxdigit(buffer[1]))
	        &&(isxdigit(buffer[2]))
	        &&(isxdigit(buffer[3]))
	        &&(isxdigit(buffer[4]))
	        &&(isxdigit(buffer[5]))
	        &&(isxdigit(buffer[6]))
	        &&(buffer[7]=='0') /* record type : 00 -> 05 */
	        &&(buffer[8]>='0')&&(buffer[8]<'6'))
	{
		DEBUG("IHEX image detected.");
		image->type = IMAGE_IHEX;
	}
	else
	{
		image->type = IMAGE_BINARY;
	}

	return ERROR_OK;
}

int identify_image_type(image_t *image, char *type_string, char *url)
{
	if (type_string)
	{
		if (!strcmp(type_string, "bin"))
		{
			image->type = IMAGE_BINARY;
		}
		else if (!strcmp(type_string, "ihex"))
		{
			image->type = IMAGE_IHEX;
		}
		else if (!strcmp(type_string, "elf"))
		{
			image->type = IMAGE_ELF;
		}
		else if (!strcmp(type_string, "mem"))
		{
			image->type = IMAGE_MEMORY;
		}
		else
		{
			return ERROR_IMAGE_TYPE_UNKNOWN;
		}
	}
	else
	{
		return autodetect_image_type(image, url);
	}
	
	return ERROR_OK;
}

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
	section[image->num_sections].private = &ihex->buffer[cooked_bytes];
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
					section[image->num_sections].private = &ihex->buffer[cooked_bytes];
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
			image->sections = malloc(sizeof(image_section_t) * image->num_sections);
			for (i = 0; i < image->num_sections; i++)
			{
				image->sections[i].private = section[i].private;
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
					section[image->num_sections].private = &ihex->buffer[cooked_bytes];
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

int image_elf_read_headers(image_t *image)
{
	image_elf_t *elf = image->type_private;
	u32 read_bytes;
	u32 i,j;
	int retval;

	elf->header = malloc(sizeof(Elf32_Ehdr));

	if ((retval = fileio_read(&elf->fileio, sizeof(Elf32_Ehdr), (u8*)elf->header, &read_bytes)) != ERROR_OK)
	{
		ERROR("cannot read ELF file header, read failed");
		return ERROR_FILEIO_OPERATION_FAILED;
	}
	if (read_bytes != sizeof(Elf32_Ehdr))
	{
		ERROR("cannot read ELF file header, only partially read");
		return ERROR_FILEIO_OPERATION_FAILED;
	}

	if (strncmp((char*)elf->header->e_ident,ELFMAG,SELFMAG)!=0)
	{
		ERROR("invalid ELF file, bad magic number");
		return ERROR_IMAGE_FORMAT_ERROR;
	}
	if (elf->header->e_ident[EI_CLASS]!=ELFCLASS32)
	{
		ERROR("invalid ELF file, only 32bits files are supported");
		return ERROR_IMAGE_FORMAT_ERROR;
	}


	elf->endianness = elf->header->e_ident[EI_DATA];
	if ((elf->endianness!=ELFDATA2LSB)
		 &&(elf->endianness!=ELFDATA2MSB))
	{
		ERROR("invalid ELF file, unknown endianess setting");
		return ERROR_IMAGE_FORMAT_ERROR;
	}

	elf->segment_count = field16(elf,elf->header->e_phnum);
	if (elf->segment_count==0)
	{
		ERROR("invalid ELF file, no program headers");
		return ERROR_IMAGE_FORMAT_ERROR;
	}

	elf->segments = malloc(elf->segment_count*sizeof(Elf32_Phdr));

	if ((retval = fileio_read(&elf->fileio, elf->segment_count*sizeof(Elf32_Phdr), (u8*)elf->segments, &read_bytes)) != ERROR_OK)
	{
		ERROR("cannot read ELF segment headers, read failed");
		return retval;
	}
	if (read_bytes != elf->segment_count*sizeof(Elf32_Phdr))
	{
		ERROR("cannot read ELF segment headers, only partially read");
		return ERROR_FILEIO_OPERATION_FAILED;
	}

	/* count useful segments (loadable) */
	image->num_sections = 0;
	for (i=0;i<elf->segment_count;i++)
		if (field32(elf,elf->segments[i].p_type) == PT_LOAD)
			image->num_sections++;
	/* alloc and fill sections array with loadable segments */
	image->sections = malloc(image->num_sections * sizeof(image_section_t));
	for (i=0,j=0;i<elf->segment_count;i++)
	{
		if (field32(elf,elf->segments[i].p_type) == PT_LOAD)
		{
			image->sections[j].size = field32(elf,elf->segments[i].p_memsz);
			image->sections[j].base_address = field32(elf,elf->segments[i].p_vaddr);
			image->sections[j].private = &elf->segments[i];
			image->sections[j].flags = field32(elf,elf->segments[i].p_flags);
			j++;
		}
	}
		
	image->start_address_set = 1;
	image->start_address = field32(elf,elf->header->e_entry);

	return ERROR_OK;
}

int image_elf_read_section(image_t *image, int section, u32 offset, u32 size, u8 *buffer, u32 *size_read)
{
	image_elf_t *elf = image->type_private;
	Elf32_Phdr *segment = (Elf32_Phdr *)image->sections[section].private;
	u32 read_size,really_read;
	int retval;

	*size_read = 0;
	
	DEBUG("load segment %d at 0x%x (sz=0x%x)",section,offset,size);

	/* read initialized data in current segment if any */
	if (offset<field32(elf,segment->p_filesz))
	{
		/* maximal size present in file for the current segment */
		read_size = MIN(size, field32(elf,segment->p_filesz)-offset);
		DEBUG("read elf: size = 0x%x at 0x%x",read_size,
			field32(elf,segment->p_offset)+offset);
		/* read initialized area of the segment */
		if ((retval = fileio_seek(&elf->fileio, field32(elf,segment->p_offset)+offset)) != ERROR_OK)
		{
			ERROR("cannot find ELF segment content, seek failed");
			return retval;
		}
		if ((retval = fileio_read(&elf->fileio, read_size, buffer, &really_read)) != ERROR_OK)
		{
			ERROR("cannot read ELF segment content, read failed");
			return retval;
		}
		buffer += read_size;
		size -= read_size;
		offset += read_size;
		*size_read += read_size;
		/* need more data ? */
		if (!size)
			return ERROR_OK;
	}
	/* if there is remaining zeroed area in current segment */
	if (offset<field32(elf,segment->p_memsz))
	{
		/* fill zeroed part (BSS) of the segment */
		read_size = MIN(size, field32(elf,segment->p_memsz)-offset);
		DEBUG("zero fill: size = 0x%x",read_size);
		memset(buffer,0,read_size);
		*size_read += read_size;
	}
	
	return ERROR_OK;
}

int image_open(image_t *image, char *url, char *type_string)
{
	int retval = ERROR_OK;
	
	if ((retval = identify_image_type(image, type_string, url)) != ERROR_OK)
	{
		return retval;
	}
	
	if (image->type == IMAGE_BINARY)
	{
		image_binary_t *image_binary;
		
		image_binary = image->type_private = malloc(sizeof(image_binary_t));
		
		if ((retval = fileio_open(&image_binary->fileio, url, FILEIO_READ, FILEIO_BINARY)) != ERROR_OK)
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
	else if (image->type == IMAGE_ELF)
	{
		image_elf_t *image_elf;
		
		image_elf = image->type_private = malloc(sizeof(image_elf_t));
		
		if ((retval = fileio_open(&image_elf->fileio, url, FILEIO_READ, FILEIO_BINARY)) != ERROR_OK)
		{
			strncpy(image->error_str, image_elf->fileio.error_str, IMAGE_MAX_ERROR_STRING); 
			ERROR(image->error_str);
			return retval;
		}
		
		if ((retval = image_elf_read_headers(image)) != ERROR_OK)
		{
			snprintf(image->error_str, IMAGE_MAX_ERROR_STRING,
				"failed to read ELF headers, check daemon output for additional information");
			ERROR(image->error_str);
			fileio_close(&image_elf->fileio);
			return retval;
		}
	}
	else if (image->type == IMAGE_MEMORY)
	{
		image_memory_t *image_memory;
		
		image->num_sections = 1;
		image->sections = malloc(sizeof(image_section_t));
		image->sections[0].base_address = 0x0;
		image->sections[0].size = 0xffffffff;
		image->sections[0].flags = 0;
		
		image_memory = image->type_private = malloc(sizeof(image_memory_t));
		
		image_memory->target = get_target_by_num(strtoul(url, NULL, 0));;
		image_memory->cache = NULL;
		image_memory->cache_address = 0x0;
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
		memcpy(buffer, (u8*)image->sections[section].private + offset, size);
		*size_read = size;
		image->error_str[0] = '\0';
		
		return ERROR_OK;
	}
	else if (image->type == IMAGE_ELF)
	{
		return image_elf_read_section(image, section, offset, size, buffer, size_read);
	}
	else if (image->type == IMAGE_MEMORY)
	{
		image_memory_t *image_memory = image->type_private;
		u32 address = image->sections[section].base_address + offset;
		
		*size_read = 0;
		
		while ((size - *size_read) > 0)
		{
			u32 size_in_cache;
			
			if (!image_memory->cache
				|| (address < image_memory->cache_address)
				|| (address >= (image_memory->cache_address + IMAGE_MEMORY_CACHE_SIZE)))
			{
				if (!image_memory->cache)
					image_memory->cache = malloc(IMAGE_MEMORY_CACHE_SIZE);
				
				if (target_read_buffer(image_memory->target, address & ~(IMAGE_MEMORY_CACHE_SIZE - 1),
					IMAGE_MEMORY_CACHE_SIZE, image_memory->cache) != ERROR_OK)
				{
					free(image_memory->cache);
					return ERROR_IMAGE_TEMPORARILY_UNAVAILABLE;
				}
				image_memory->cache_address = address & ~(IMAGE_MEMORY_CACHE_SIZE - 1);
			}
			
			size_in_cache = (image_memory->cache_address + IMAGE_MEMORY_CACHE_SIZE) - address;
			
			memcpy(buffer + *size_read,
				image_memory->cache + (address - image_memory->cache_address),
				(size_in_cache > size) ? size : size_in_cache
				);
				
			*size_read += (size_in_cache > size) ? size : size_in_cache;
			address += (size_in_cache > size) ? size : size_in_cache;
		}
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
		
		if (image_ihex->buffer)
			free(image_ihex->buffer);
	}
	else if (image->type == IMAGE_ELF)
	{
		image_elf_t *image_elf = image->type_private;
		
		fileio_close(&image_elf->fileio);

		if (image_elf->header)
			free(image_elf->header);

		if (image_elf->segments)
			free(image_elf->segments);
	}
	else if (image->type == IMAGE_MEMORY)
	{
		image_memory_t *image_memory = image->type_private;
		
		if (image_memory->cache)
			free(image_memory->cache);
	}

	if (image->type_private)
		free(image->type_private);
	
	if (image->sections)
		free(image->sections);
	
	return ERROR_OK;
}
