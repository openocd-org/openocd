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
		return retval;
	}
	retval = fileio_read(&fileio, 9, buffer, &read_bytes);

	if (retval==ERROR_OK)
	{
		if (read_bytes != 9)
		{
			retval=ERROR_FILEIO_OPERATION_FAILED;
		}
	}
	fileio_close(&fileio);

	if (retval!=ERROR_OK)
		return retval;

	/* check header against known signatures */
	if (strncmp((char*)buffer,ELFMAG,SELFMAG)==0)
	{
		LOG_DEBUG("ELF image detected.");
		image->type = IMAGE_ELF;
	}
	else if ((buffer[0]==':') /* record start byte */
		&&(isxdigit(buffer[1]))
		&&(isxdigit(buffer[2]))
		&&(isxdigit(buffer[3]))
		&&(isxdigit(buffer[4]))
		&&(isxdigit(buffer[5]))
		&&(isxdigit(buffer[6]))
		&&(buffer[7]=='0') /* record type : 00 -> 05 */
		&&(buffer[8]>='0')&&(buffer[8]<'6'))
	{
		LOG_DEBUG("IHEX image detected.");
		image->type = IMAGE_IHEX;
	}
	else if ((buffer[0] == 'S') /* record start byte */
		&&(isxdigit(buffer[1]))
		&&(isxdigit(buffer[2]))
		&&(isxdigit(buffer[3]))
		&&(buffer[1] >= '0') && (buffer[1] < '9'))
	{
		LOG_DEBUG("S19 image detected.");
		image->type = IMAGE_SRECORD;
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
		else if (!strcmp(type_string, "s19"))
		{
			image->type = IMAGE_SRECORD;
		}
		else if (!strcmp(type_string, "build"))
		{
			image->type = IMAGE_BUILDER;
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
	u32 full_address = 0x0;
	u32 cooked_bytes;
	int i;
	char lpszLine[1023];

	/* we can't determine the number of sections that we'll have to create ahead of time,
	 * so we locally hold them until parsing is finished */
	image_section_t section[IMAGE_MAX_SECTIONS];

	ihex->buffer = malloc(fileio->size >> 1);
	cooked_bytes = 0x0;
	image->num_sections = 0;
	section[image->num_sections].private = &ihex->buffer[cooked_bytes];
	section[image->num_sections].base_address = 0x0;
	section[image->num_sections].size = 0x0;
	section[image->num_sections].flags = 0;

	while (fileio_fgets(fileio, 1023, lpszLine) == ERROR_OK)
	{
		u32 count;
		u32 address;
		u32 record_type;
		u32 checksum;
		u8 cal_checksum = 0;
		u32 bytes_read = 0;

		if (sscanf(&lpszLine[bytes_read], ":%2x%4x%2x", &count, &address, &record_type) != 3)
		{
			return ERROR_IMAGE_FORMAT_ERROR;
		}
		bytes_read += 9;

		cal_checksum += (u8)count;
		cal_checksum += (u8)(address >> 8);
		cal_checksum += (u8)address;
		cal_checksum += (u8)record_type;

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
				sscanf(&lpszLine[bytes_read], "%2x", (u32*)&ihex->buffer[cooked_bytes]);
				cal_checksum += (u8)ihex->buffer[cooked_bytes];
				bytes_read += 2;
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
				image->sections[i].base_address = section[i].base_address;
				image->sections[i].size = section[i].size;
				image->sections[i].flags = section[i].flags;
			}

			return ERROR_OK;
		}
		else if (record_type == 2) /* Linear Address Record */
		{
			u16 upper_address;

			sscanf(&lpszLine[bytes_read], "%4hx", &upper_address);
			cal_checksum += (u8)(upper_address >> 8);
			cal_checksum += (u8)upper_address;
			bytes_read += 4;

			if ((full_address >> 4) != upper_address)
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
					(full_address & 0xffff) | (upper_address << 4);
				full_address = (full_address & 0xffff) | (upper_address << 4);
			}
		}
		else if (record_type == 3) /* Start Segment Address Record */
		{
			u32 dummy;

			/* "Start Segment Address Record" will not be supported */
			/* but we must consume it, and do not create an error.  */
			while (count-- > 0)
			{
				sscanf(&lpszLine[bytes_read], "%2x", &dummy);
				cal_checksum += (u8)dummy;
				bytes_read += 2;
			}
		}
		else if (record_type == 4) /* Extended Linear Address Record */
		{
			u16 upper_address;

			sscanf(&lpszLine[bytes_read], "%4hx", &upper_address);
			cal_checksum += (u8)(upper_address >> 8);
			cal_checksum += (u8)upper_address;
			bytes_read += 4;

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

			sscanf(&lpszLine[bytes_read], "%8x", &start_address);
			cal_checksum += (u8)(start_address >> 24);
			cal_checksum += (u8)(start_address >> 16);
			cal_checksum += (u8)(start_address >> 8);
			cal_checksum += (u8)start_address;
			bytes_read += 8;

			image->start_address_set = 1;
			image->start_address = be_to_h_u32((u8*)&start_address);
		}
		else
		{
			LOG_ERROR("unhandled IHEX record type: %i", record_type);
			return ERROR_IMAGE_FORMAT_ERROR;
		}

		sscanf(&lpszLine[bytes_read], "%2x", &checksum);
		bytes_read += 2;

		if ((u8)checksum != (u8)(~cal_checksum + 1))
		{
			/* checksum failed */
			LOG_ERROR("incorrect record checksum found in IHEX file");
			return ERROR_IMAGE_CHECKSUM;
		}
	}

	LOG_ERROR("premature end of IHEX file, no end-of-file record found");
	return ERROR_IMAGE_FORMAT_ERROR;
}

int image_elf_read_headers(image_t *image)
{
	image_elf_t *elf = image->type_private;
	u32 read_bytes;
	u32 i,j;
	int retval;

	elf->header = malloc(sizeof(Elf32_Ehdr));

	if(elf->header == NULL)
	{
		LOG_ERROR("insufficient memory to perform operation ");
		return ERROR_FILEIO_OPERATION_FAILED;
	}

	if ((retval = fileio_read(&elf->fileio, sizeof(Elf32_Ehdr), (u8*)elf->header, &read_bytes)) != ERROR_OK)
	{
		LOG_ERROR("cannot read ELF file header, read failed");
		return ERROR_FILEIO_OPERATION_FAILED;
	}
	if (read_bytes != sizeof(Elf32_Ehdr))
	{
		LOG_ERROR("cannot read ELF file header, only partially read");
		return ERROR_FILEIO_OPERATION_FAILED;
	}

	if (strncmp((char*)elf->header->e_ident,ELFMAG,SELFMAG)!=0)
	{
		LOG_ERROR("invalid ELF file, bad magic number");
		return ERROR_IMAGE_FORMAT_ERROR;
	}
	if (elf->header->e_ident[EI_CLASS]!=ELFCLASS32)
	{
		LOG_ERROR("invalid ELF file, only 32bits files are supported");
		return ERROR_IMAGE_FORMAT_ERROR;
	}

	elf->endianness = elf->header->e_ident[EI_DATA];
	if ((elf->endianness!=ELFDATA2LSB)
		 &&(elf->endianness!=ELFDATA2MSB))
	{
		LOG_ERROR("invalid ELF file, unknown endianess setting");
		return ERROR_IMAGE_FORMAT_ERROR;
	}

	elf->segment_count = field16(elf,elf->header->e_phnum);
	if (elf->segment_count==0)
	{
		LOG_ERROR("invalid ELF file, no program headers");
		return ERROR_IMAGE_FORMAT_ERROR;
	}

	if ((retval = fileio_seek(&elf->fileio, field32(elf,elf->header->e_phoff))) != ERROR_OK)
	{
		LOG_ERROR("cannot seek to ELF program header table, read failed");
		return retval;
	}

	elf->segments = malloc(elf->segment_count*sizeof(Elf32_Phdr));
	if(elf->segments == NULL)
	{
		LOG_ERROR("insufficient memory to perform operation ");
		return ERROR_FILEIO_OPERATION_FAILED;
	}

	if ((retval = fileio_read(&elf->fileio, elf->segment_count*sizeof(Elf32_Phdr), (u8*)elf->segments, &read_bytes)) != ERROR_OK)
	{
		LOG_ERROR("cannot read ELF segment headers, read failed");
		return retval;
	}
	if (read_bytes != elf->segment_count*sizeof(Elf32_Phdr))
	{
		LOG_ERROR("cannot read ELF segment headers, only partially read");
		return ERROR_FILEIO_OPERATION_FAILED;
	}

	/* count useful segments (loadable), ignore BSS section */
	image->num_sections = 0;
	for (i=0;i<elf->segment_count;i++)
		if ((field32(elf, elf->segments[i].p_type) == PT_LOAD) && (field32(elf, elf->segments[i].p_filesz) != 0))
			image->num_sections++;
	/* alloc and fill sections array with loadable segments */
	image->sections = malloc(image->num_sections * sizeof(image_section_t));
	for (i=0,j=0;i<elf->segment_count;i++)
	{
		if ((field32(elf, elf->segments[i].p_type) == PT_LOAD) && (field32(elf, elf->segments[i].p_filesz) != 0))
		{
			image->sections[j].size = field32(elf,elf->segments[i].p_filesz);
			image->sections[j].base_address = field32(elf,elf->segments[i].p_paddr);
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

	LOG_DEBUG("load segment %d at 0x%x (sz=0x%x)",section,offset,size);

	/* read initialized data in current segment if any */
	if (offset<field32(elf,segment->p_filesz))
	{
		/* maximal size present in file for the current segment */
		read_size = MIN(size, field32(elf,segment->p_filesz)-offset);
		LOG_DEBUG("read elf: size = 0x%x at 0x%x",read_size,
			field32(elf,segment->p_offset)+offset);
		/* read initialized area of the segment */
		if ((retval = fileio_seek(&elf->fileio, field32(elf,segment->p_offset)+offset)) != ERROR_OK)
		{
			LOG_ERROR("cannot find ELF segment content, seek failed");
			return retval;
		}
		if ((retval = fileio_read(&elf->fileio, read_size, buffer, &really_read)) != ERROR_OK)
		{
			LOG_ERROR("cannot read ELF segment content, read failed");
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

	return ERROR_OK;
}

int image_mot_buffer_complete(image_t *image)
{
	image_mot_t *mot = image->type_private;
	fileio_t *fileio = &mot->fileio;
	u32 full_address = 0x0;
	u32 cooked_bytes;
	int i;
	char lpszLine[1023];

	/* we can't determine the number of sections that we'll have to create ahead of time,
	 * so we locally hold them until parsing is finished */
	image_section_t section[IMAGE_MAX_SECTIONS];

	mot->buffer = malloc(fileio->size >> 1);
	cooked_bytes = 0x0;
	image->num_sections = 0;
	section[image->num_sections].private = &mot->buffer[cooked_bytes];
	section[image->num_sections].base_address = 0x0;
	section[image->num_sections].size = 0x0;
	section[image->num_sections].flags = 0;

	while (fileio_fgets(fileio, 1023, lpszLine) == ERROR_OK)
	{
		u32 count;
		u32 address;
		u32 record_type;
		u32 checksum;
		u8 cal_checksum = 0;
		u32 bytes_read = 0;

		/* get record type and record length */
		if (sscanf(&lpszLine[bytes_read], "S%1x%2x", &record_type, &count) != 2)
		{
			return ERROR_IMAGE_FORMAT_ERROR;
		}

		bytes_read += 4;
		cal_checksum += (u8)count;

		/* skip checksum byte */
		count -=1;

		if (record_type == 0)
		{
			/* S0 - starting record (optional) */
			int iValue;

			while (count-- > 0) {
				sscanf(&lpszLine[bytes_read], "%2x", &iValue);
				cal_checksum += (u8)iValue;
				bytes_read += 2;
			}
		}
		else if (record_type >= 1 && record_type <= 3)
		{
			switch( record_type )
			{
				case 1:
					/* S1 - 16 bit address data record */
					sscanf(&lpszLine[bytes_read], "%4x", &address);
					cal_checksum += (u8)(address >> 8);
					cal_checksum += (u8)address;
					bytes_read += 4;
					count -=2;
					break;

				case 2:
					/* S2 - 24 bit address data record */
					sscanf(&lpszLine[bytes_read], "%6x", &address);
					cal_checksum += (u8)(address >> 16);
					cal_checksum += (u8)(address >> 8);
					cal_checksum += (u8)address;
					bytes_read += 6;
					count -=3;
					break;

				case 3:
					/* S3 - 32 bit address data record */
					sscanf(&lpszLine[bytes_read], "%8x", &address);
					cal_checksum += (u8)(address >> 24);
					cal_checksum += (u8)(address >> 16);
					cal_checksum += (u8)(address >> 8);
					cal_checksum += (u8)address;
					bytes_read += 8;
					count -=4;
					break;

			}

			if (full_address != address)
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
					section[image->num_sections].private = &mot->buffer[cooked_bytes];
				}
				section[image->num_sections].base_address = address;
				full_address = address;
			}

			while (count-- > 0)
			{
				sscanf(&lpszLine[bytes_read], "%2x", (u32*)&mot->buffer[cooked_bytes]);
				cal_checksum += (u8)mot->buffer[cooked_bytes];
				bytes_read += 2;
				cooked_bytes += 1;
				section[image->num_sections].size += 1;
				full_address++;
			}
		}
		else if (record_type == 5)
		{
			/* S5 is the data count record, we ignore it */
			u32 dummy;

			while (count-- > 0)
			{
				sscanf(&lpszLine[bytes_read], "%2x", &dummy);
				cal_checksum += (u8)dummy;
				bytes_read += 2;
			}
		}
		else if (record_type >= 7 && record_type <= 9)
		{
			/* S7, S8, S9 - ending records for 32, 24 and 16bit */
			image->num_sections++;

			/* copy section information */
			image->sections = malloc(sizeof(image_section_t) * image->num_sections);
			for (i = 0; i < image->num_sections; i++)
			{
				image->sections[i].private = section[i].private;
				image->sections[i].base_address = section[i].base_address;
				image->sections[i].size = section[i].size;
				image->sections[i].flags = section[i].flags;
			}

			return ERROR_OK;
		}
		else
		{
			LOG_ERROR("unhandled S19 record type: %i", record_type);
			return ERROR_IMAGE_FORMAT_ERROR;
		}

		/* account for checksum, will always be 0xFF */
		sscanf(&lpszLine[bytes_read], "%2x", &checksum);
		cal_checksum += (u8)checksum;
		bytes_read += 2;

		if( cal_checksum != 0xFF )
		{
			/* checksum failed */
			LOG_ERROR("incorrect record checksum found in S19 file");
			return ERROR_IMAGE_CHECKSUM;
		}
	}

	LOG_ERROR("premature end of S19 file, no end-of-file record found");
	return ERROR_IMAGE_FORMAT_ERROR;
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
			return retval;
		}

		image->num_sections = 1;
		image->sections = malloc(sizeof(image_section_t));
		image->sections[0].base_address = 0x0;
		image->sections[0].size = image_binary->fileio.size;
		image->sections[0].flags = 0;
	}
	else if (image->type == IMAGE_IHEX)
	{
		image_ihex_t *image_ihex;

		image_ihex = image->type_private = malloc(sizeof(image_ihex_t));

		if ((retval = fileio_open(&image_ihex->fileio, url, FILEIO_READ, FILEIO_TEXT)) != ERROR_OK)
		{
			return retval;
		}

		if ((retval = image_ihex_buffer_complete(image)) != ERROR_OK)
		{
			LOG_ERROR("failed buffering IHEX image, check daemon output for additional information");
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
			return retval;
		}

		if ((retval = image_elf_read_headers(image)) != ERROR_OK)
		{
			fileio_close(&image_elf->fileio);
			return retval;
		}
	}
	else if (image->type == IMAGE_MEMORY)
	{
		target_t *target = get_target_by_num(strtoul(url, NULL, 0));
		if (target==NULL)
		{
			LOG_ERROR("Target '%s' does not exist", url);
			return ERROR_FAIL;
		}

		image_memory_t *image_memory;

		image->num_sections = 1;
		image->sections = malloc(sizeof(image_section_t));
		image->sections[0].base_address = 0x0;
		image->sections[0].size = 0xffffffff;
		image->sections[0].flags = 0;

		image_memory = image->type_private = malloc(sizeof(image_memory_t));

		image_memory->target = target;
		image_memory->cache = NULL;
		image_memory->cache_address = 0x0;
	}
	else if (image->type == IMAGE_SRECORD)
	{
		image_mot_t *image_mot;

		image_mot = image->type_private = malloc(sizeof(image_mot_t));

		if ((retval = fileio_open(&image_mot->fileio, url, FILEIO_READ, FILEIO_TEXT)) != ERROR_OK)
		{
			return retval;
		}

		if ((retval = image_mot_buffer_complete(image)) != ERROR_OK)
		{
			LOG_ERROR("failed buffering S19 image, check daemon output for additional information");
			fileio_close(&image_mot->fileio);
			return retval;
		}
	}
	else if (image->type == IMAGE_BUILDER)
	{
		image->num_sections = 0;
		image->sections = NULL;
		image->type_private = NULL;
	}

	if (image->base_address_set)
	{
		/* relocate */
		int section;
		for (section=0; section < image->num_sections; section++)
		{
			image->sections[section].base_address+=image->base_address;
		}
		/* we're done relocating. The two statements below are mainly
		 * for documenation purposes: stop anyone from empirically
		 * thinking they should use these values henceforth. */
		image->base_address=0;
		image->base_address_set=0;
	}

	return retval;
};

int image_read_section(image_t *image, int section, u32 offset, u32 size, u8 *buffer, u32 *size_read)
{
	int retval;

	/* don't read past the end of a section */
	if (offset + size > image->sections[section].size)
	{
		LOG_DEBUG("read past end of section: 0x%8.8x + 0x%8.8x > 0x%8.8x",
				offset, size, image->sections[section].size);
		return ERROR_INVALID_ARGUMENTS;
	}

	if (image->type == IMAGE_BINARY)
	{
		image_binary_t *image_binary = image->type_private;

		/* only one section in a plain binary */
		if (section != 0)
			return ERROR_INVALID_ARGUMENTS;

		/* seek to offset */
		if ((retval = fileio_seek(&image_binary->fileio, offset)) != ERROR_OK)
		{
			return retval;
		}

		/* return requested bytes */
		if ((retval = fileio_read(&image_binary->fileio, size, buffer, size_read)) != ERROR_OK)
		{
			return retval;
		}
	}
	else if (image->type == IMAGE_IHEX)
	{
		memcpy(buffer, (u8*)image->sections[section].private + offset, size);
		*size_read = size;

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
					image_memory->cache = NULL;
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
	else if (image->type == IMAGE_SRECORD)
	{
		memcpy(buffer, (u8*)image->sections[section].private + offset, size);
		*size_read = size;

		return ERROR_OK;
	}
	else if (image->type == IMAGE_BUILDER)
	{
		memcpy(buffer, (u8*)image->sections[section].private + offset, size);
		*size_read = size;

		return ERROR_OK;
	}

	return ERROR_OK;
}

int image_add_section(image_t *image, u32 base, u32 size, int flags, u8 *data)
{
	image_section_t *section;

	/* only image builder supports adding sections */
	if (image->type != IMAGE_BUILDER)
		return ERROR_INVALID_ARGUMENTS;

	/* see if there's a previous section */
	if (image->num_sections)
	{
		section = &image->sections[image->num_sections - 1];

		/* see if it's enough to extend the last section,
		 * adding data to previous sections or merging is not supported */
		if (((section->base_address + section->size) == base) && (section->flags == flags))
		{
			section->private = realloc(section->private, section->size + size);
			memcpy((u8*)section->private + section->size, data, size);
			section->size += size;
			return ERROR_OK;
		}
	}

	/* allocate new section */
	image->num_sections++;
	image->sections = realloc(image->sections, sizeof(image_section_t) * image->num_sections);
	section = &image->sections[image->num_sections - 1];
	section->base_address = base;
	section->size = size;
	section->flags = flags;
	section->private = malloc(sizeof(u8) * size);
	memcpy((u8*)section->private, data, size);

	return ERROR_OK;
}

void image_close(image_t *image)
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
		{
			free(image_ihex->buffer);
			image_ihex->buffer = NULL;
		}
	}
	else if (image->type == IMAGE_ELF)
	{
		image_elf_t *image_elf = image->type_private;

		fileio_close(&image_elf->fileio);

		if (image_elf->header)
		{
			free(image_elf->header);
			image_elf->header = NULL;
		}

		if (image_elf->segments)
		{
			free(image_elf->segments);
			image_elf->segments = NULL;
		}
	}
	else if (image->type == IMAGE_MEMORY)
	{
		image_memory_t *image_memory = image->type_private;

		if (image_memory->cache)
		{
			free(image_memory->cache);
			image_memory->cache = NULL;
		}
	}
	else if (image->type == IMAGE_SRECORD)
	{
		image_mot_t *image_mot = image->type_private;

		fileio_close(&image_mot->fileio);

		if (image_mot->buffer)
		{
			free(image_mot->buffer);
			image_mot->buffer = NULL;
		}
	}
	else if (image->type == IMAGE_BUILDER)
	{
		int i;

		for (i = 0; i < image->num_sections; i++)
		{
			free(image->sections[i].private);
			image->sections[i].private = NULL;
		}
	}

	if (image->type_private)
	{
		free(image->type_private);
		image->type_private = NULL;
	}

	if (image->sections)
	{
		free(image->sections);
		image->sections = NULL;
	}
}

int image_calculate_checksum(u8* buffer, u32 nbytes, u32* checksum)
{
	u32 crc = 0xffffffff;
	LOG_DEBUG("Calculating checksum");

	u32 crc32_table[256];

	/* Initialize the CRC table and the decoding table.  */
	int i, j;
	unsigned int c;
	for (i = 0; i < 256; i++)
	{
		/* as per gdb */
		for (c = i << 24, j = 8; j > 0; --j)
			c = c & 0x80000000 ? (c << 1) ^ 0x04c11db7 : (c << 1);
		crc32_table[i] = c;
	}

	while (nbytes>0)
	{
		int run=nbytes;
		if (run>32768)
		{
			run=32768;
		}
		nbytes-=run;
		while (run--)
		{
			/* as per gdb */
			crc = (crc << 8) ^ crc32_table[((crc >> 24) ^ *buffer++) & 255];
		}
		keep_alive();
	}

	LOG_DEBUG("Calculating checksum done");

	*checksum = crc;
	return ERROR_OK;
}
