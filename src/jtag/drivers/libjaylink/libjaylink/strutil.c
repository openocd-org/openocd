/*
 * This file is part of the libjaylink project.
 *
 * Copyright (C) 2016 Marc Schink <jaylink-dev@marcschink.de>
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

#include <stdlib.h>
#include <stdint.h>
#include <errno.h>

#include "libjaylink.h"

/**
 * @file
 *
 * String utility functions.
 */

/**
 * Convert a string representation of a serial number to an integer.
 *
 * The string representation of the serial number must be in decimal form.
 *
 * @param[in] str String representation to convert.
 * @param[out] serial_number Serial number on success, and undefined on
 *                           failure.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR Conversion error. Serial number is invalid or string
 *                     representation contains invalid character(s).
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_parse_serial_number(const char *str,
		uint32_t *serial_number)
{
	char *end_ptr;
	unsigned long long tmp;

	if (!str || !serial_number)
		return JAYLINK_ERR_ARG;

	errno = 0;
	tmp = strtoull(str, &end_ptr, 10);

	if (*end_ptr != '\0' || errno != 0 || tmp > UINT32_MAX)
		return JAYLINK_ERR;

	*serial_number = tmp;

	return JAYLINK_OK;
}
