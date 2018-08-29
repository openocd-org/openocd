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

#include "libjaylink.h"

/**
 * @file
 *
 * Error handling.
 */

/**
 * Return a human-readable description of a libjaylink error code.
 *
 * @param[in] error_code A libjaylink error code. See #jaylink_error for valid
 *                       values.
 *
 * @return A string which describes the given error code, or the string
 *         <i>unknown error</i> if the error code is not known. The string is
 *         null-terminated and must not be free'd by the caller.
 *
 * @since 0.1.0
 */
JAYLINK_API const char *jaylink_strerror(int error_code)
{
	switch (error_code) {
	case JAYLINK_OK:
		return "no error";
	case JAYLINK_ERR:
		return "unspecified error";
	case JAYLINK_ERR_ARG:
		return "invalid argument";
	case JAYLINK_ERR_MALLOC:
		return "memory allocation error";
	case JAYLINK_ERR_TIMEOUT:
		return "timeout occurred";
	case JAYLINK_ERR_PROTO:
		return "protocol violation";
	case JAYLINK_ERR_NOT_AVAILABLE:
		return "entity not available";
	case JAYLINK_ERR_NOT_SUPPORTED:
		return "operation not supported";
	case JAYLINK_ERR_IO:
		return "input/output error";
	case JAYLINK_ERR_DEV:
		return "device: unspecified error";
	case JAYLINK_ERR_DEV_NOT_SUPPORTED:
		return "device: operation not supported";
	case JAYLINK_ERR_DEV_NOT_AVAILABLE:
		return "device: entity not available";
	case JAYLINK_ERR_DEV_NO_MEMORY:
		return "device: not enough memory to perform operation";
	default:
		return "unknown error";
	}
}

/**
 * Return the name of a libjaylink error code.
 *
 * @param[in] error_code A libjaylink error code. See #jaylink_error for valid
 *                       values.
 *
 * @return A string which contains the name for the given error code, or the
 *         string <i>unknown error code</i> if the error code is not known. The
 *         string is null-terminated and must not be free'd by the caller.
 *
 * @since 0.1.0
 */
JAYLINK_API const char *jaylink_strerror_name(int error_code)
{
	switch (error_code) {
	case JAYLINK_OK:
		return "JAYLINK_OK";
	case JAYLINK_ERR:
		return "JAYLINK_ERR";
	case JAYLINK_ERR_ARG:
		return "JAYLINK_ERR_ARG";
	case JAYLINK_ERR_MALLOC:
		return "JAYLINK_ERR_MALLOC";
	case JAYLINK_ERR_TIMEOUT:
		return "JAYLINK_ERR_TIMEOUT";
	case JAYLINK_ERR_PROTO:
		return "JAYLINK_ERR_PROTO";
	case JAYLINK_ERR_NOT_AVAILABLE:
		return "JAYLINK_ERR_NOT_AVAILABLE";
	case JAYLINK_ERR_NOT_SUPPORTED:
		return "JAYLINK_ERR_NOT_SUPPORTED";
	case JAYLINK_ERR_IO:
		return "JAYLINK_ERR_IO";
	case JAYLINK_ERR_DEV:
		return "JAYLINK_ERR_DEV";
	case JAYLINK_ERR_DEV_NOT_SUPPORTED:
		return "JAYLINK_ERR_DEV_NOT_SUPPORTED";
	case JAYLINK_ERR_DEV_NOT_AVAILABLE:
		return "JAYLINK_ERR_DEV_NOT_AVAILABLE";
	case JAYLINK_ERR_DEV_NO_MEMORY:
		return "JAYLINK_ERR_DEV_NO_MEMORY";
	default:
		return "unknown error code";
	}
}
