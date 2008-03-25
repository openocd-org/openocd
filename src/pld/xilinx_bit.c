/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
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

#include "xilinx_bit.h"

#include "pld.h"
#include "log.h"

#include <stdlib.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>
#include <errno.h>

#include <sys/time.h>
#include <time.h>

int read_section(FILE *input_file, int length_size, char section, u32 *buffer_length, u8 **buffer) 
{
	u8 length_buffer[4];
	u32 length;
	char section_char;
	int read_count;
	
	if ((length_size != 2) && (length_size != 4))
	{
		LOG_ERROR("BUG: length_size neither 2 nor 4");
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	if ((read_count = fread(&section_char, 1, 1, input_file)) != 1)
	{
		return ERROR_PLD_FILE_LOAD_FAILED;
	}
	
	if (section_char != section)
	{
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	if ((read_count = fread(length_buffer, 1, length_size, input_file)) != length_size)
	{
		return ERROR_PLD_FILE_LOAD_FAILED;
	}
	
	if (length_size == 4)
		length = be_to_h_u32(length_buffer);
	else /* (length_size == 2) */
		length = be_to_h_u16(length_buffer);
		
	if (buffer_length)
		*buffer_length = length;
	
	*buffer = malloc(length);
	
	if ((read_count = fread(*buffer, 1, length, input_file)) != length)
	{
		return ERROR_PLD_FILE_LOAD_FAILED;
	}
	
	return ERROR_OK;
}

int xilinx_read_bit_file(xilinx_bit_file_t *bit_file, char *filename)
{
	FILE *input_file;
	struct stat input_stat;
	int read_count;
	
	if (!filename || !bit_file)
		return ERROR_INVALID_ARGUMENTS;
	
	if (stat(filename, &input_stat) == -1)
	{
		LOG_ERROR("couldn't stat() %s: %s", filename, strerror(errno));
		return ERROR_PLD_FILE_LOAD_FAILED;
	}

	if (S_ISDIR(input_stat.st_mode))
	{
		LOG_ERROR("%s is a directory", filename);
		return ERROR_PLD_FILE_LOAD_FAILED;
	}
		
	if (input_stat.st_size == 0){
		LOG_ERROR("Empty file %s", filename);
		return ERROR_PLD_FILE_LOAD_FAILED;
	}
		
	if (!(input_file = fopen(filename, "rb")))
	{
		LOG_ERROR("couldn't open %s: %s", filename, strerror(errno));
		return ERROR_PLD_FILE_LOAD_FAILED;
	}
	
	if ((read_count = fread(bit_file->unknown_header, 1, 13, input_file)) != 13)
	{
		LOG_ERROR("couldn't read unknown_header from file '%s'", filename);
		return ERROR_PLD_FILE_LOAD_FAILED;
	}
	
	if (read_section(input_file, 2, 'a', NULL, &bit_file->source_file) != ERROR_OK)
		return ERROR_PLD_FILE_LOAD_FAILED;
	
	if (read_section(input_file, 2, 'b', NULL, &bit_file->part_name) != ERROR_OK)
		return ERROR_PLD_FILE_LOAD_FAILED;
	
	if (read_section(input_file, 2, 'c', NULL, &bit_file->date) != ERROR_OK)
		return ERROR_PLD_FILE_LOAD_FAILED;

	if (read_section(input_file, 2, 'd', NULL, &bit_file->time) != ERROR_OK)
		return ERROR_PLD_FILE_LOAD_FAILED;

	if (read_section(input_file, 4, 'e', &bit_file->length, &bit_file->data) != ERROR_OK)
		return ERROR_PLD_FILE_LOAD_FAILED;
	
	LOG_DEBUG("bit_file: %s %s %s,%s %i", bit_file->source_file, bit_file->part_name,
		bit_file->date, bit_file->time, bit_file->length);
	
	fclose(input_file);
	
	return ERROR_OK;
}
