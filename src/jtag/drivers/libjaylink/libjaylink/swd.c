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
 * Serial Wire Debug (SWD) functions.
 */

/** @cond PRIVATE */
#define CMD_SWD_IO 0xcf

/**
 * Error code indicating that there is not enough free memory on the device to
 * perform the SWD I/O operation.
 */
#define SWD_IO_ERR_NO_MEMORY	0x06
/** @endcond */

/**
 * Perform a SWD I/O operation.
 *
 * @note This function must only be used if the #JAYLINK_TIF_SWD interface is
 *       available and selected.
 *
 * @param[in,out] devh Device handle.
 * @param[in] direction Buffer to read the transfer direction from.
 * @param[in] out Buffer to read host-to-target data from.
 * @param[out] in Buffer to store target-to-host data on success. Its content
 *                is undefined on failure. The buffer must be large enough to
 *                contain at least the specified number of bits to transfer.
 * @param[in] length Total number of bits to transfer from host to target and
 *                   vice versa.
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
JAYLINK_API int jaylink_swd_io(struct jaylink_device_handle *devh,
		const uint8_t *direction, const uint8_t *out, uint8_t *in,
		uint16_t length)
{
	int ret;
	struct jaylink_context *ctx;
	uint16_t num_bytes;
	uint8_t buf[4];
	uint8_t status;

	if (!devh || !direction || !out || !in || !length)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;
	num_bytes = (length + 7) / 8;

	ret = transport_start_write_read(devh, 4 + 2 * num_bytes,
		num_bytes + 1, true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	buf[0] = CMD_SWD_IO;
	buf[1] = 0x00;
	buffer_set_u16(buf, length, 2);

	ret = transport_write(devh, buf, 4);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_write(devh, direction, num_bytes);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_write(devh, out, num_bytes);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_read(devh, in, num_bytes);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_read(devh, &status, 1);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	if (status == SWD_IO_ERR_NO_MEMORY) {
		return JAYLINK_ERR_DEV_NO_MEMORY;
	} else if (status > 0) {
		log_err(ctx, "SWD I/O operation failed: 0x%x.", status);
		return JAYLINK_ERR_DEV;
	}

	return JAYLINK_OK;
}
