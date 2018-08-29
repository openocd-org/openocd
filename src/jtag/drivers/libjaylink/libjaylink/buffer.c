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
#include <string.h>

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif
#include "libjaylink-internal.h"

/**
 * @file
 *
 * Buffer helper functions.
 */

/**
 * Write a 16-bit unsigned integer value to a buffer.
 *
 * The value is stored in the buffer in device byte order.
 *
 * @param[out] buffer Buffer to write the value into.
 * @param[in] value Value to write into the buffer in host byte order.
 * @param[in] offset Offset of the value within the buffer in bytes.
 */
JAYLINK_PRIV void buffer_set_u16(uint8_t *buffer, uint16_t value,
		size_t offset)
{
	/*
	 * Store the value in the buffer and swap byte order depending on the
	 * host byte order.
	 */
#ifdef WORDS_BIGENDIAN
	buffer[offset + 0] = value;
	buffer[offset + 1] = value >> 8;
#else
	memcpy(buffer + offset, &value, sizeof(value));
#endif
}

/**
 * Read a 16-bit unsigned integer value from a buffer.
 *
 * The value in the buffer is expected to be stored in device byte order.
 *
 * @param[in] buffer Buffer to read the value from.
 * @param[in] offset Offset of the value within the buffer in bytes.
 *
 * @return The value read from the buffer in host byte order.
 */
JAYLINK_PRIV uint16_t buffer_get_u16(const uint8_t *buffer, size_t offset)
{
	uint16_t value;

	/*
	 * Read the value from the buffer and swap byte order depending on the
	 * host byte order.
	 */
#ifdef WORDS_BIGENDIAN
	value = (((uint16_t)buffer[offset + 1])) | \
		(((uint16_t)buffer[offset + 0]) << 8);
#else
	memcpy(&value, buffer + offset, sizeof(value));
#endif

	return value;
}

/**
 * Write a 32-bit unsigned integer value to a buffer.
 *
 * The value is stored in the buffer in device byte order.
 *
 * @param[out] buffer Buffer to write the value into.
 * @param[in] value Value to write into the buffer in host byte order.
 * @param[in] offset Offset of the value within the buffer in bytes.
 */
JAYLINK_PRIV void buffer_set_u32(uint8_t *buffer, uint32_t value,
		size_t offset)
{
	/*
	 * Store the value in the buffer and swap byte order depending on the
	 * host byte order.
	 */
#ifdef WORDS_BIGENDIAN
	buffer[offset + 0] = value;
	buffer[offset + 1] = value >> 8;
	buffer[offset + 2] = value >> 16;
	buffer[offset + 3] = value >> 24;
#else
	memcpy(buffer + offset, &value, sizeof(value));
#endif
}

/**
 * Read a 32-bit unsigned integer value from a buffer.
 *
 * The value in the buffer is expected to be stored in device byte order.
 *
 * @param[in] buffer Buffer to read the value from.
 * @param[in] offset Offset of the value within the buffer in bytes.
 *
 * @return The value read from the buffer in host byte order.
 */
JAYLINK_PRIV uint32_t buffer_get_u32(const uint8_t *buffer, size_t offset)
{
	uint32_t value;

	/*
	 * Read the value from the buffer and swap byte order depending on the
	 * host byte order.
	 */
#ifdef WORDS_BIGENDIAN
	value = (((uint32_t)buffer[offset + 3])) | \
		(((uint32_t)buffer[offset + 2]) << 8) | \
		(((uint32_t)buffer[offset + 1]) << 16) | \
		(((uint32_t)buffer[offset + 0]) << 24);
#else
	memcpy(&value, buffer + offset, sizeof(value));
#endif

	return value;
}
