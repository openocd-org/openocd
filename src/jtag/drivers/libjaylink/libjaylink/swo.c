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

#include "libjaylink.h"
#include "libjaylink-internal.h"

/**
 * @file
 *
 * Serial Wire Output (SWO) functions.
 */

/** @cond PRIVATE */
#define CMD_SWO			0xeb

#define SWO_CMD_START		0x64
#define SWO_CMD_STOP		0x65
#define SWO_CMD_READ		0x66
#define SWO_CMD_GET_SPEEDS	0x6e

#define SWO_PARAM_MODE		0x01
#define SWO_PARAM_BAUDRATE	0x02
#define SWO_PARAM_READ_SIZE	0x03
#define SWO_PARAM_BUFFER_SIZE	0x04

#define SWO_ERR			0x80000000
/** @endcond */

/**
 * Start SWO capture.
 *
 * @note This function must be used only if the device has the
 *       #JAYLINK_DEV_CAP_SWO capability.
 *
 * @param[in,out] devh Device handle.
 * @param[in] mode Mode to capture data with.
 * @param[in] baudrate Baudrate to capture data in bit per second.
 * @param[in] size Device internal buffer size in bytes to use for capturing.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR_DEV Unspecified device error.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @see jaylink_swo_get_speeds()
 * @see jaylink_get_free_memory()
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_swo_start(struct jaylink_device_handle *devh,
		enum jaylink_swo_mode mode, uint32_t baudrate, uint32_t size)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[32];
	uint32_t status;

	if (!devh || !baudrate || !size)
		return JAYLINK_ERR_ARG;

	if (mode != JAYLINK_SWO_MODE_UART)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;
	ret = transport_start_write_read(devh, 21, 4, true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	buf[0] = CMD_SWO;
	buf[1] = SWO_CMD_START;

	buf[2] = 0x04;
	buf[3] = SWO_PARAM_MODE;
	buffer_set_u32(buf, mode, 4);

	buf[8] = 0x04;
	buf[9] = SWO_PARAM_BAUDRATE;
	buffer_set_u32(buf, baudrate, 10);

	buf[14] = 0x04;
	buf[15] = SWO_PARAM_BUFFER_SIZE;
	buffer_set_u32(buf, size, 16);

	buf[20] = 0x00;

	ret = transport_write(devh, buf, 21);

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

	status = buffer_get_u32(buf, 0);

	if (status > 0) {
		log_err(ctx, "Failed to start capture: 0x%x.", status);
		return JAYLINK_ERR_DEV;
	}

	return JAYLINK_OK;
}

/**
 * Stop SWO capture.
 *
 * @note This function must be used only if the device has the
 *       #JAYLINK_DEV_CAP_SWO capability.
 *
 * @param[in,out] devh Device handle.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR_DEV Unspecified device error.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @see jaylink_swo_start()
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_swo_stop(struct jaylink_device_handle *devh)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[4];
	uint32_t status;

	if (!devh)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;
	ret = transport_start_write_read(devh, 3, 4, true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	buf[0] = CMD_SWO;
	buf[1] = SWO_CMD_STOP;
	buf[2] = 0x00;

	ret = transport_write(devh, buf, 3);

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

	status = buffer_get_u32(buf, 0);

	if (status > 0) {
		log_err(ctx, "Failed to stop capture: 0x%x.", status);
		return JAYLINK_ERR_DEV;
	}

	return JAYLINK_OK;
}

/**
 * Read SWO trace data.
 *
 * @note This function must be used only if the device has the
 *       #JAYLINK_DEV_CAP_SWO capability.
 *
 * @param[in,out] devh Device handle.
 * @param[out] buffer Buffer to store trace data on success. Its content is
 *                    undefined on failure.
 * @param[in,out] length Maximum number of bytes to read. On success, the value
 *                       gets updated with the actual number of bytes read. The
 *                       value is undefined on failure.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_PROTO Protocol violation.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR_DEV Unspecified device error.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @see jaylink_swo_start()
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_swo_read(struct jaylink_device_handle *devh,
		uint8_t *buffer, uint32_t *length)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[32];
	uint32_t status;
	uint32_t tmp;

	if (!devh || !buffer || !length)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;
	ret = transport_start_write_read(devh, 9, 8, true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	buf[0] = CMD_SWO;
	buf[1] = SWO_CMD_READ;

	buf[2] = 0x04;
	buf[3] = SWO_PARAM_READ_SIZE;
	buffer_set_u32(buf, *length, 4);

	buf[8] = 0x00;

	ret = transport_write(devh, buf, 9);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_write() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_read(devh, buf, 8);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	status = buffer_get_u32(buf, 0);
	tmp = buffer_get_u32(buf, 4);

	if (tmp > *length) {
		log_err(ctx, "Received %u bytes but only %u bytes were "
			"requested.", tmp, *length);
		return JAYLINK_ERR_PROTO;
	}

	*length = tmp;

	if (tmp > 0) {
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
	}

	if (status > 0) {
		log_err(ctx, "Failed to read data: 0x%x.", status);
		return JAYLINK_ERR_DEV;
	}

	return JAYLINK_OK;
}

/**
 * Retrieve SWO speeds.

 * The speeds are calculated as follows:
 *
 * @par
 * <tt>speeds = @a freq / n</tt> with <tt>n >= @a min_div</tt> and
 * <tt>n <= @a max_div</tt>, where @p n is an integer
 *
 * Assuming, for example, a base frequency @a freq of 4500 kHz, a minimum
 * divider @a min_div of 1 and a maximum divider @a max_div of 8 then the
 * highest possible SWO speed is 4500 kHz / 1 = 4500 kHz. The next highest
 * speed is 2250 kHz for a divider of 2, and so on. Accordingly, the lowest
 * possible speed is 4500 kHz / 8 = 562.5 kHz.
 *
 * @note This function must be used only if the device has the
 *       #JAYLINK_DEV_CAP_SWO capability.
 *
 * @param[in,out] devh Device handle.
 * @param[in] mode Capture mode to retrieve speeds for.
 * @param[out] speed Speed information on success, and undefined on failure.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR_TIMEOUT A timeout occurred.
 * @retval JAYLINK_ERR_PROTO Protocol violation.
 * @retval JAYLINK_ERR_IO Input/output error.
 * @retval JAYLINK_ERR_DEV Unspecified device error.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_swo_get_speeds(struct jaylink_device_handle *devh,
		enum jaylink_swo_mode mode, struct jaylink_swo_speed *speed)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[24];
	uint32_t tmp;
	uint32_t length;

	if (!devh || !speed)
		return JAYLINK_ERR_ARG;

	if (mode != JAYLINK_SWO_MODE_UART)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;
	ret = transport_start_write_read(devh, 9, 4, true);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_write_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	buf[0] = CMD_SWO;
	buf[1] = SWO_CMD_GET_SPEEDS;

	buf[2] = 0x04;
	buf[3] = SWO_PARAM_MODE;
	buffer_set_u32(buf, mode, 4);

	buf[8] = 0x00;

	ret = transport_write(devh, buf, 9);

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

	if (tmp & SWO_ERR) {
		log_err(ctx, "Failed to retrieve speed information: 0x%x.",
			tmp);
		return JAYLINK_ERR_DEV;
	}

	length = tmp;

	if (length != 28) {
		log_err(ctx, "Unexpected number of bytes received: %u.",
			length);
		return JAYLINK_ERR_PROTO;
	}

	length = length - 4;
	ret = transport_start_read(devh, length);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_start_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	ret = transport_read(devh, buf, length);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "transport_read() failed: %s.",
			jaylink_strerror(ret));
		return ret;
	}

	speed->freq = buffer_get_u32(buf, 4);
	speed->min_div = buffer_get_u32(buf, 8);

	if (!speed->min_div) {
		log_err(ctx, "Minimum frequency divider is zero.");
		return JAYLINK_ERR_PROTO;
	}

	speed->max_div = buffer_get_u32(buf, 12);

	if (speed->max_div < speed->min_div) {
		log_err(ctx, "Maximum frequency divider is less than minimum "
			"frequency divider.");
		return JAYLINK_ERR_PROTO;
	}

	speed->min_prescaler = buffer_get_u32(buf, 16);
	speed->max_prescaler = buffer_get_u32(buf, 20);

	if (speed->max_prescaler < speed->min_prescaler) {
		log_err(ctx, "Maximum prescaler is less than minimum "
			"prescaler.");
		return JAYLINK_ERR_PROTO;
	}

	return JAYLINK_OK;
}
