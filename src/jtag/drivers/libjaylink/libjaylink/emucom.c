/*
 * This file is part of the libjaylink project.
 *
 * Copyright (C) 2015-2016 Marc Schink <jaylink-dev@marcschink.de>
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

#include "libjaylink.h"
#include "libjaylink-internal.h"

/**
 * @file
 *
 * Emulator communication (EMUCOM).
 */

/** @cond PRIVATE */
#define CMD_EMUCOM			0xee

#define EMUCOM_CMD_READ			0x00
#define EMUCOM_CMD_WRITE		0x01

/** Bitmask for the error indication bit of an EMUCOM status code. */
#define EMUCOM_ERR			0x80000000

/** Error code indicating that the channel is not supported by the device. */
#define EMUCOM_ERR_NOT_SUPPORTED	0x80000001

/**
 * Error code indicating that the channel is not available for the requested
 * number of bytes to be read.
 *
 * The number of bytes available on this channel is encoded in the lower
 * 24 bits of the EMUCOM status code.
 *
 * @see EMUCOM_AVAILABLE_BYTES_MASK
 */
#define EMUCOM_ERR_NOT_AVAILABLE	0x81000000

/**
 * Bitmask to extract the number of available bytes on a channel from an EMUCOM
 * status code.
 */
#define EMUCOM_AVAILABLE_BYTES_MASK	0x00ffffff
/** @endcond */

/**
 * Read from an EMUCOM channel.
 *
 * @note This function must only be used if the device has the
 *       #JAYLINK_DEV_CAP_EMUCOM capability.
 *
 * @param[in,out] devh Device handle.
 * @param[in] channel Channel to read data from.
 * @param[out] buffer Buffer to store read data on success. Its content is
 *                    undefined on failure.
 * @param[in,out] length Number of bytes to read. On success, the value gets
 *                       updated with the actual number of bytes read. Unless
 *                       otherwise specified, the value is undefined on
 *                       failure.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_PROTO Protocol violation.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR_DEV_NOT_SUPPORTED Channel is not supported by the
 *                                       device.
 * @retval JAYLINK_ERR_DEV_NOT_AVAILABLE Channel is not available for the
 *                                       requested amount of data. @p length is
 *                                       updated with the number of bytes
 *                                       available on this channel.
 * @retval JAYLINK_ERR_DEV Unspecified device error.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_emucom_read(struct jaylink_device_handle *devh,
		uint32_t channel, uint8_t *buffer, uint32_t *length)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[10];
	uint32_t tmp;

	if (!devh || !buffer || !length)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;
	ret = transport_start_write_read(devh, 10, 4, true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	buf[0] = CMD_EMUCOM;
	buf[1] = EMUCOM_CMD_READ;

	buffer_set_u32(buf, channel, 2);
	buffer_set_u32(buf, *length, 6);

	ret = transport_write(devh, buf, 10);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
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

	if (tmp == EMUCOM_ERR_NOT_SUPPORTED)
		return JAYLINK_ERR_DEV_NOT_SUPPORTED;

	if ((tmp & ~EMUCOM_AVAILABLE_BYTES_MASK) == EMUCOM_ERR_NOT_AVAILABLE) {
		*length = tmp & EMUCOM_AVAILABLE_BYTES_MASK;
		return JAYLINK_ERR_DEV_NOT_AVAILABLE;
	}

	if (tmp & EMUCOM_ERR) {
		log_err(ctx, "Failed to read from channel 0x%x: 0x%x.",
			channel, tmp);
		return JAYLINK_ERR_DEV;
	}

	if (tmp > *length) {
		log_err(ctx, "Requested at most %u bytes but device "
			"returned %u bytes.", *length, tmp);
		return JAYLINK_ERR_PROTO;
	}

	*length = tmp;

	if (!tmp)
		return JAYLINK_OK;

	ret = transport_start_read(devh, tmp);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_read(devh, buffer, tmp);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	return JAYLINK_OK;
}

/**
 * Write to an EMUCOM channel.
 *
 * @note This function must only be used if the device has the
 *       #JAYLINK_DEV_CAP_EMUCOM capability.
 *
 * @param[in,out] devh Device handle.
 * @param[in] channel Channel to write data to.
 * @param[in] buffer Buffer to write data from.
 * @param[in,out] length Number of bytes to write. On success, the value gets
 *                       updated with the actual number of bytes written. The
 *                       value is undefined on failure.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_PROTO Protocol violation.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR_DEV_NOT_SUPPORTED Channel is not supported by the
 *                                       device.
 * @retval JAYLINK_ERR_DEV Unspecified device error.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_emucom_write(struct jaylink_device_handle *devh,
		uint32_t channel, const uint8_t *buffer, uint32_t *length)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[10];
	uint32_t tmp;

	if (!devh || !buffer || !length)
		return JAYLINK_ERR_ARG;

	if (!*length)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;
	ret = transport_start_write(devh, 10, true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	buf[0] = CMD_EMUCOM;
	buf[1] = EMUCOM_CMD_WRITE;

	buffer_set_u32(buf, channel, 2);
	buffer_set_u32(buf, *length, 6);

	ret = transport_write(devh, buf, 10);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_start_write_read(devh, *length, 4, false);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_write(devh, buffer, *length);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
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

	if (tmp == EMUCOM_ERR_NOT_SUPPORTED)
		return JAYLINK_ERR_DEV_NOT_SUPPORTED;

	if (tmp & EMUCOM_ERR) {
		log_err(ctx, "Failed to write to channel 0x%x: 0x%x.",
			channel, tmp);
		return JAYLINK_ERR_DEV;
	}

	if (tmp > *length) {
		log_err(ctx, "Only %u bytes were supposed to be written, but "
			"the device reported %u written bytes.", *length, tmp);
		return JAYLINK_ERR_PROTO;
	}

	*length = tmp;

	return JAYLINK_OK;
}
