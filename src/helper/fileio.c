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

#include "log.h"
#include "configuration.h"
#include "fileio.h"

static inline int fileio_open_local(struct fileio *fileio)
{
	char access[4];

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
			LOG_ERROR("BUG: access neither read, write nor readwrite");
			return ERROR_INVALID_ARGUMENTS;
	}

	/* win32 always opens in binary mode */
#ifndef _WIN32
	if (fileio->type == FILEIO_BINARY)
#endif
	{
		strcat(access, "b");
	}

	if (!(fileio->file = open_file_from_path (fileio->url, access)))
	{
		LOG_ERROR("couldn't open %s", fileio->url);
		return ERROR_FILEIO_OPERATION_FAILED;
	}

	if ((fileio->access != FILEIO_WRITE) || (fileio->access == FILEIO_READWRITE))
	{
		/* NB! Here we use fseek() instead of stat(), since stat is a
		 * more advanced operation that might not apply to e.g. a disk path
		 * that refers to e.g. a tftp client */
		int result, result2;

		result = fseek(fileio->file, 0, SEEK_END);

		fileio->size = ftell(fileio->file);

		result2 = fseek(fileio->file, 0, SEEK_SET);

		if ((fileio->size < 0)||(result < 0)||(result2 < 0))
		{
			fileio_close(fileio);
			return ERROR_FILEIO_OPERATION_FAILED;
		}
	}
	else
	{
		fileio->size = 0x0;
	}

	return ERROR_OK;
}

int fileio_open(struct fileio *fileio, const char *url, enum fileio_access access,	enum fileio_type type)
{
	int retval = ERROR_OK;

	fileio->type = type;
	fileio->access = access;
	fileio->url = strdup(url);

	retval = fileio_open_local(fileio);

	return retval;
}

static inline int fileio_close_local(struct fileio *fileio)
{
	int retval;
	if ((retval = fclose(fileio->file)) != 0)
	{
		if (retval == EBADF)
		{
			LOG_ERROR("BUG: fileio_local->file not a valid file descriptor");
		}
		else
		{
			LOG_ERROR("couldn't close %s: %s", fileio->url, strerror(errno));
		}

		return ERROR_FILEIO_OPERATION_FAILED;
	}

	return ERROR_OK;
}

int fileio_close(struct fileio *fileio)
{
	int retval;

	retval = fileio_close_local(fileio);

	free((void*)fileio->url);
	fileio->url = NULL;

	return retval;
}

int fileio_seek(struct fileio *fileio, size_t position)
{
	int retval;
	if ((retval = fseek(fileio->file, position, SEEK_SET)) != 0)
	{
		LOG_ERROR("couldn't seek file %s: %s", fileio->url, strerror(errno));
		return ERROR_FILEIO_OPERATION_FAILED;
	}

	return ERROR_OK;
}

static int fileio_local_read(struct fileio *fileio,
		size_t size, void *buffer, size_t *size_read)
{
	ssize_t retval = fread(buffer, 1, size, fileio->file);
	*size_read = (retval >= 0) ? retval : 0;
	return (retval < 0) ? retval : ERROR_OK;
}

int fileio_read(struct fileio *fileio, size_t size, void *buffer,
		size_t *size_read)
{
	return fileio_local_read(fileio, size, buffer, size_read);
}

int fileio_read_u32(struct fileio *fileio, uint32_t *data)
{
	uint8_t buf[4];
	size_t size_read;
	int retval = fileio_local_read(fileio, sizeof(uint32_t), buf, &size_read);
	if (ERROR_OK == retval && sizeof(uint32_t) != size_read)
		retval = -EIO;
	if (ERROR_OK == retval)
		*data = be_to_h_u32(buf);
	return retval;
}

static int fileio_local_fgets(struct fileio *fileio,
		size_t size, void *buffer)
{
	if (fgets(buffer, size, fileio->file) == NULL)
		return ERROR_FILEIO_OPERATION_FAILED;

	return ERROR_OK;
}

int fileio_fgets(struct fileio *fileio, size_t size, void *buffer)
{
	return fileio_local_fgets(fileio, size, buffer);
}

static int fileio_local_write(struct fileio *fileio,
		size_t size, const void *buffer, size_t *size_written)
{
	ssize_t retval = fwrite(buffer, 1, size, fileio->file);
	*size_written = (retval >= 0) ? retval : 0;
	return (retval < 0) ? retval : ERROR_OK;
}

int fileio_write(struct fileio *fileio,
		size_t size, const void *buffer, size_t *size_written)
{
	int retval = fileio_local_write(fileio, size, buffer, size_written);
	if (retval == ERROR_OK)
		fileio->size += *size_written;
	return retval;
}

int fileio_write_u32(struct fileio *fileio, uint32_t data)
{
	uint8_t buf[4];
	h_u32_to_be(buf, data);

	size_t size_written;
	int retval = fileio_write(fileio, 4, buf, &size_written);
	if (ERROR_OK == retval && size_written != sizeof(uint32_t))
		retval = -EIO;

	return retval;
}
