/*
 * This file is part of the libjaylink project.
 *
 * Copyright (C) 2015 Marc Schink <jaylink-dev@marcschink.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdint.h>
#include <stdbool.h>
#include <string.h>

#include "libjaylink.h"
#include "libjaylink-internal.h"

/**
 * @file
 *
 * File I/O functions.
 */

/** @cond PRIVATE */
#define CMD_FILE_IO		0x1e

#define FILE_IO_CMD_READ	0x64
#define FILE_IO_CMD_WRITE	0x65
#define FILE_IO_CMD_GET_SIZE	0x66
#define FILE_IO_CMD_DELETE	0x67

#define FILE_IO_PARAM_FILENAME	0x01
#define FILE_IO_PARAM_OFFSET	0x02
#define FILE_IO_PARAM_LENGTH	0x03

#define FILE_IO_ERR		0x80000000
/** @endcond */

/**
 * Read from a file.
 *
 * The maximum amount of data that can be read from a file at once is
 * #JAYLINK_FILE_MAX_TRANSFER_SIZE bytes. Multiple reads in conjunction with
 * the @p offset parameter are needed for larger files.
 *
 * @note This function must only be used if the device has the
 *       #JAYLINK_DEV_CAP_FILE_IO capability.
 *
 * @param[in,out] devh Device handle.
 * @param[in] filename Name of the file to read from. The length of the name
 *                     must not exceed #JAYLINK_FILE_NAME_MAX_LENGTH bytes.
 * @param[out] buffer Buffer to store read data on success. Its content is
 *                    undefined on failure
 * @param[in] offset Offset in bytes relative to the beginning of the file from
 *                   where to start reading.
 * @param[in,out] length Number of bytes to read. On success, the value gets
 *                       updated with the actual number of bytes read. The
 *                       value is undefined on failure.
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR_DEV Unspecified device error, or the file was not found.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_file_read(struct jaylink_device_handle *devh,
		const char *filename, uint8_t *buffer, uint32_t offset,
		uint32_t *length)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[18 + JAYLINK_FILE_NAME_MAX_LENGTH];
	size_t filename_length;
	uint32_t tmp;

	if (!devh || !filename || !buffer || !length)
		return JAYLINK_ERR_ARG;

	if (!*length)
		return JAYLINK_ERR_ARG;

	if (*length > JAYLINK_FILE_MAX_TRANSFER_SIZE)
		return JAYLINK_ERR_ARG;

	filename_length = strlen(filename);

	if (!filename_length)
		return JAYLINK_ERR_ARG;

	if (filename_length > JAYLINK_FILE_NAME_MAX_LENGTH)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;
	ret = transport_start_write(devh, 18 + filename_length, true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	buf[0] = CMD_FILE_IO;
	buf[1] = FILE_IO_CMD_READ;
	buf[2] = 0x00;

	buf[3] = filename_length;
	buf[4] = FILE_IO_PARAM_FILENAME;
	memcpy(buf + 5, filename, filename_length);

	buf[filename_length + 5] = 0x04;
	buf[filename_length + 6] = FILE_IO_PARAM_OFFSET;
	buffer_set_u32(buf, offset, filename_length + 7);

	buf[filename_length + 11] = 0x04;
	buf[filename_length + 12] = FILE_IO_PARAM_LENGTH;
	buffer_set_u32(buf, *length, filename_length + 13);

	buf[filename_length + 17] = 0x00;

	ret = transport_write(devh, buf, 18 + filename_length);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_start_read(devh, *length);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_read(devh, buffer, *length);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_start_read(devh, 4);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_read(devh, buf, 4);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	tmp = buffer_get_u32(buf, 0);

	if (tmp & FILE_IO_ERR)
		return JAYLINK_ERR_DEV;

	*length = tmp;

	return JAYLINK_OK;
}

/**
 * Write to a file.
 *
 * If a file does not exist, a new file is created.
 *
 * The maximum amount of data that can be written to a file at once is
 * #JAYLINK_FILE_MAX_TRANSFER_SIZE bytes. Multiple writes in conjunction with
 * the @p offset parameter are needed for larger files.
 *
 * @note This function must only be used if the device has the
 *       #JAYLINK_DEV_CAP_FILE_IO capability.
 *
 * @param[in,out] devh Device handle.
 * @param[in] filename Name of the file to write to. The length of the name
 *                     must not exceed #JAYLINK_FILE_NAME_MAX_LENGTH bytes.
 * @param[in] buffer Buffer to write data from.
 * @param[in] offset Offset in bytes relative to the beginning of the file from
 *                   where to start writing.
 * @param[in,out] length Number of bytes to write. On success, the value gets
 *                       updated with the actual number of bytes written. The
 *                       value is undefined on failure.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR_DEV Unspecified device error, or the file was not found.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_file_write(struct jaylink_device_handle *devh,
		const char *filename, const uint8_t *buffer, uint32_t offset,
		uint32_t *length)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[18 + JAYLINK_FILE_NAME_MAX_LENGTH];
	size_t filename_length;
	uint32_t tmp;

	if (!devh || !filename || !buffer || !length)
		return JAYLINK_ERR_ARG;

	if (!*length)
		return JAYLINK_ERR_ARG;

	if (*length > JAYLINK_FILE_MAX_TRANSFER_SIZE)
		return JAYLINK_ERR_ARG;

	filename_length = strlen(filename);

	if (!filename_length)
		return JAYLINK_ERR_ARG;

	if (filename_length > JAYLINK_FILE_NAME_MAX_LENGTH)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;
	ret = transport_start_write(devh, 18 + filename_length, true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	buf[0] = CMD_FILE_IO;
	buf[1] = FILE_IO_CMD_WRITE;
	buf[2] = 0x00;

	buf[3] = filename_length;
	buf[4] = FILE_IO_PARAM_FILENAME;
	memcpy(buf + 5, filename, filename_length);

	buf[filename_length + 5] = 0x04;
	buf[filename_length + 6] = FILE_IO_PARAM_OFFSET;
	buffer_set_u32(buf, offset, filename_length + 7);

	buf[filename_length + 11] = 0x04;
	buf[filename_length + 12] = FILE_IO_PARAM_LENGTH;
	buffer_set_u32(buf, *length, filename_length + 13);

	buf[filename_length + 17] = 0x00;

	ret = transport_write(devh, buf, 18 + filename_length);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_start_write(devh, *length, true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_write(devh, buffer, *length);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_start_read(devh, 4);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_read(devh, buf, 4);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	tmp = buffer_get_u32(buf, 0);

	if (tmp & FILE_IO_ERR)
		return JAYLINK_ERR_DEV;

	*length = tmp;

	return JAYLINK_OK;
}

/**
 * Retrieve the size of a file.
 *
 * @note This function must only be used if the device has the
 *       #JAYLINK_DEV_CAP_FILE_IO capability.
 *
 * @param[in,out] devh Device handle.
 * @param[in] filename Name of the file to retrieve the size of. The length
 *                     of the name must not exceed
 *                     #JAYLINK_FILE_NAME_MAX_LENGTH bytes.
 * @param[out] size Size of the file in bytes on success, and undefined on
 *                  failure.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR_DEV Unspecified device error, or the file was not found.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_file_get_size(struct jaylink_device_handle *devh,
		const char *filename, uint32_t *size)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[6 + JAYLINK_FILE_NAME_MAX_LENGTH];
	size_t length;
	uint32_t tmp;

	if (!devh || !filename || !size)
		return JAYLINK_ERR_ARG;

	length = strlen(filename);

	if (!length)
		return JAYLINK_ERR_ARG;

	if (length > JAYLINK_FILE_NAME_MAX_LENGTH)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;
	ret = transport_start_write(devh, 6 + length, true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	buf[0] = CMD_FILE_IO;
	buf[1] = FILE_IO_CMD_GET_SIZE;
	buf[2] = 0x00;

	buf[3] = length;
	buf[4] = FILE_IO_PARAM_FILENAME;
	memcpy(buf + 5, filename, length);

	buf[length + 5] = 0x00;

	ret = transport_write(devh, buf, 6 + length);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_start_read(devh, 4);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_read(devh, buf, 4);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	tmp = buffer_get_u32(buf, 0);

	if (tmp & FILE_IO_ERR)
		return JAYLINK_ERR_DEV;

	*size = tmp;

	return JAYLINK_OK;
}

/**
 * Delete a file.
 *
 * @note This function must only be used if the device has the
 *       #JAYLINK_DEV_CAP_FILE_IO capability.
 *
 * @param[in,out] devh Device handle.
 * @param[in] filename Name of the file to delete. The length of the name
 *                     must not exceed #JAYLINK_FILE_NAME_MAX_LENGTH bytes.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR_DEV Unspecified device error, or the file was not found.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_file_delete(struct jaylink_device_handle *devh,
		const char *filename)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[6 + JAYLINK_FILE_NAME_MAX_LENGTH];
	size_t length;
	uint32_t tmp;

	if (!devh || !filename)
		return JAYLINK_ERR_ARG;

	length = strlen(filename);

	if (!length)
		return JAYLINK_ERR_ARG;

	if (length > JAYLINK_FILE_NAME_MAX_LENGTH)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;
	ret = transport_start_write(devh, 6 + length, true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	buf[0] = CMD_FILE_IO;
	buf[1] = FILE_IO_CMD_DELETE;
	buf[2] = 0x00;

	buf[3] = length;
	buf[4] = FILE_IO_PARAM_FILENAME;
	memcpy(buf + 5, filename, length);

	buf[length + 5] = 0x00;

	ret = transport_write(devh, buf, 6 + length);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_start_read(devh, 4);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_read(devh, buf, 4);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	tmp = buffer_get_u32(buf, 0);

	if (tmp & FILE_IO_ERR)
		return JAYLINK_ERR_DEV;

	return JAYLINK_OK;
}
