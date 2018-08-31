/*
 * This file is part of the libjaylink project.
 *
 * Copyright (C) 2014-2015 Marc Schink <jaylink-dev@marcschink.de>
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
 * JTAG functions.
 */

/** @cond PRIVATE */
#define CMD_JTAG_IO_V2		0xce
#define CMD_JTAG_IO_V3		0xcf
#define CMD_JTAG_CLEAR_TRST	0xde
#define CMD_JTAG_SET_TRST	0xdf

/**
 * Error code indicating that there is not enough free memory on the device to
 * perform the JTAG I/O operation.
 */
#define JTAG_IO_ERR_NO_MEMORY	0x06
/** @endcond */

/**
 * Perform a JTAG I/O operation.
 *
 * @note This function must only be used if the #JAYLINK_TIF_JTAG interface is
 *       available and selected. Nevertheless, this function can be used if the
 *       device doesn't have the #JAYLINK_DEV_CAP_SELECT_TIF capability.
 *
 * @param[in,out] devh Device handle.
 * @param[in] tms Buffer to read TMS data from.
 * @param[in] tdi Buffer to read TDI data from.
 * @param[out] tdo Buffer to store TDO data on success. Its content is
 *                 undefined on failure. The buffer must be large enough to
 *                 contain at least the specified number of bits to transfer.
 * @param[in] length Number of bits to transfer.
 * @param[in] version Version of the JTAG command to use.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR_DEV_NO_MEMORY Not enough memory on the device to perform
 *                                   the operation.
 * @retval JAYLINK_ERR_DEV Unspecified device error.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @see jaylink_select_interface()
 * @see jaylink_set_speed()
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_jtag_io(struct jaylink_device_handle *devh,
		const uint8_t *tms, const uint8_t *tdi, uint8_t *tdo,
		uint16_t length, enum jaylink_jtag_version version)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[4];
	uint16_t num_bytes;
	uint16_t read_length;
	uint8_t status;
	uint8_t cmd;

	if (!devh || !tms || !tdi || !tdo || !length)
		return JAYLINK_ERR_ARG;

	num_bytes = (length + 7) / 8;
	read_length = num_bytes;

	switch (version) {
	case JAYLINK_JTAG_VERSION_2:
		cmd = CMD_JTAG_IO_V2;
		break;
	case JAYLINK_JTAG_VERSION_3:
		cmd = CMD_JTAG_IO_V3;
		/* In this version, the response includes a status byte. */
		read_length++;
		break;
	default:
		return JAYLINK_ERR_ARG;
	}

	ctx = devh->dev->ctx;
	ret = transport_start_write_read(devh, 4 + 2 * num_bytes,
		read_length, true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	buf[0] = cmd;
	buf[1] = 0x00;
	buffer_set_u16(buf, length, 2);

	ret = transport_write(devh, buf, 4);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_write(devh, tms, num_bytes);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_write(devh, tdi, num_bytes);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_read(devh, tdo, num_bytes);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	if (version == JAYLINK_JTAG_VERSION_2)
		return JAYLINK_OK;

	ret = transport_read(devh, &status, 1);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	if (status == JTAG_IO_ERR_NO_MEMORY) {
		return JAYLINK_ERR_DEV_NO_MEMORY;
	} else if (status > 0) {
		log_err(ctx, "JTAG I/O operation failed: 0x%x.", status);
		return JAYLINK_ERR_DEV;
	}

	return JAYLINK_OK;
}

/**
 * Clear the JTAG test reset (TRST) signal.
 *
 * @param[in,out] devh Device handle.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_jtag_clear_trst(struct jaylink_device_handle *devh)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[1];

	if (!devh)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;
	ret = transport_start_write(devh, 1, true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	buf[0] = CMD_JTAG_CLEAR_TRST;

	ret = transport_write(devh, buf, 1);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	return JAYLINK_OK;
}

/**
 * Set the JTAG test reset (TRST) signal.
 *
 * @param[in,out] devh Device handle.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_jtag_set_trst(struct jaylink_device_handle *devh)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[1];

	if (!devh)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;
	ret = transport_start_write(devh, 1, true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	buf[0] = CMD_JTAG_SET_TRST;

	ret = transport_write(devh, buf, 1);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	return JAYLINK_OK;
}
