/*
 * This file is part of the libjaylink project.
 *
 * Copyright (C) 2016-2017 Marc Schink <jaylink-dev@marcschink.de>
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

#ifdef _WIN32
#include <winsock2.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include "libjaylink.h"
#include "libjaylink-internal.h"

/**
 * @file
 *
 * Socket abstraction layer.
 */

/**
 * Close a socket.
 *
 * @param[in] sock Socket descriptor.
 *
 * @return Whether the socket was successfully closed.
 */
JAYLINK_PRIV bool socket_close(int sock)
{
	int ret;

#ifdef _WIN32
	ret = closesocket(sock);
#else
	ret = close(sock);
#endif

	if (!ret)
		return true;

	return false;
}

/**
 * Bind an address to a socket.
 *
 * @param[in] sock Socket descriptor.
 * @param[in] address Address to be bound to the socket.
 * @param[in] length Length of the structure pointed to by @p address in bytes.
 *
 * @return Whether the address was successfully assigned to the socket.
 */
JAYLINK_PRIV bool socket_bind(int sock, const struct sockaddr *address,
		size_t length)
{
	int ret;

	ret = bind(sock, address, length);

#ifdef _WIN32
	if (ret == SOCKET_ERROR)
		return false;
#else
	if (ret < 0)
		return false;
#endif

	return true;
}

/**
 * Send a message on a socket.
 *
 * @param[in] sock Socket descriptor.
 * @param[in] buffer Buffer of the message to be sent.
 * @param[in,out] length Length of the message in bytes. On success, the value
 *                       gets updated with the actual number of bytes sent. The
 *                       value is undefined on failure.
 * @param[in] flags Flags to modify the function behaviour. Use bitwise OR to
 *                  specify multiple flags.
 *
 * @return Whether the message was sent successfully.
 */
JAYLINK_PRIV bool socket_send(int sock, const void *buffer, size_t *length,
		int flags)
{
	ssize_t ret;

	ret = send(sock, buffer, *length, flags);
#ifdef _WIN32
	if (ret == SOCKET_ERROR)
		return false;
#else
	if (ret < 0)
		return false;
#endif
	*length = ret;

	return true;
}

/**
 * Receive a message from a socket.
 *
 * @param[in] sock Socket descriptor.
 * @param[out] buffer Buffer to store the received message on success. Its
 *                    content is undefined on failure.
 * @param[in,out] length Maximum length of the message in bytes. On success,
 *                       the value gets updated with the actual number of
 *                       received bytes. The value is undefined on failure.
 * @param[in] flags Flags to modify the function behaviour. Use bitwise OR to
 *                  specify multiple flags.
 *
 * @return Whether a message was successfully received.
 */
JAYLINK_PRIV bool socket_recv(int sock, void *buffer, size_t *length,
		int flags)
{
	ssize_t ret;

	ret = recv(sock, buffer, *length, flags);

#ifdef _WIN32
	if (ret == SOCKET_ERROR)
		return false;
#else
	if (ret < 0)
		return false;
#endif

	*length = ret;

	return true;
}

/**
 * Send a message on a socket.
 *
 * @param[in] sock Socket descriptor.
 * @param[in] buffer Buffer to send message from.
 * @param[in,out] length Number of bytes to send. On success, the value gets
 *                       updated with the actual number of bytes sent. The
 *                       value is undefined on failure.
 * @param[in] flags Flags to modify the function behaviour. Use bitwise OR to
 *                  specify multiple flags.
 * @param[in] address Destination address of the message.
 * @param[in] address_length Length of the structure pointed to by @p address
 *                           in bytes.
 *
 * @return Whether the message was successfully sent.
 */
JAYLINK_PRIV bool socket_sendto(int sock, const void *buffer, size_t *length,
		int flags, const struct sockaddr *address,
		size_t address_length)
{
	ssize_t ret;

	ret = sendto(sock, buffer, *length, flags, address, address_length);

#ifdef _WIN32
	if (ret == SOCKET_ERROR)
		return false;
#else
	if (ret < 0)
		return false;
#endif

	*length = ret;

	return true;
}

/**
 * Receive a message from a socket.
 *
 * @param[in] sock Socket descriptor.
 * @param[out] buffer Buffer to store the received message on success. Its
 *                    content is undefined on failure.
 * @param[in,out] length Maximum length of the message in bytes. On success,
 *                       the value gets updated with the actual number of
 *                       received bytes. The value is undefined on failure.
 * @param[in] flags Flags to modify the function behaviour. Use bitwise OR to
 *                  specify multiple flags.
 * @param[out] address Structure to store the source address of the message on
 *                     success. Its content is undefined on failure.
 *                     Can be NULL.
 * @param[in,out] address_length Length of the structure pointed to by
 *                               @p address in bytes. On success, the value
 *                               gets updated with the actual length of the
 *                               structure. The value is undefined on failure.
 *                               Should be NULL if @p address is NULL.
 *
 * @return Whether a message was successfully received.
 */
JAYLINK_PRIV bool socket_recvfrom(int sock, void *buffer, size_t *length,
		int flags, struct sockaddr *address, size_t *address_length)
{
	ssize_t ret;
#ifdef _WIN32
	int tmp;

	tmp = *address_length;
	ret = recvfrom(sock, buffer, *length, flags, address, &tmp);

	if (ret == SOCKET_ERROR)
		return false;
#else
	socklen_t tmp;

	tmp = *address_length;
	ret = recvfrom(sock, buffer, *length, flags, address, &tmp);

	if (ret < 0)
		return false;
#endif

	*address_length = tmp;
	*length = ret;

	return true;
}

/**
 * Set an option on a socket.
 *
 * @param[in] sock Socket descriptor.
 * @param[in] level Level at which the option is defined.
 * @param[in] option Option to set the value for.
 * @param[in] value Buffer of the value to be set.
 * @param[in] length Length of the value buffer in bytes.
 *
 * @return Whether the option was set successfully.
 */
JAYLINK_PRIV bool socket_set_option(int sock, int level, int option,
		const void *value, size_t length)
{
	if (!setsockopt(sock, level, option, value, length))
		return true;

	return false;
}
