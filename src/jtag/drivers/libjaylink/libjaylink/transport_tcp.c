/*
 * This file is part of the libjaylink project.
 *
 * Copyright (C) 2015-2017 Marc Schink <jaylink-dev@marcschink.de>
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
#include <string.h>
#include <sys/types.h>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#else
#include <sys/time.h>
#include <sys/types.h>
#include <sys/socket.h>
#include <netdb.h>
#include <netinet/in.h>
#endif

#include "libjaylink.h"
#include "libjaylink-internal.h"

/**
 * @file
 *
 * Transport abstraction layer (TCP/IP).
 */

/** @cond PRIVATE */
#define CMD_SERVER		0x00
#define CMD_CLIENT		0x07

/**
 * Response status code indicating that the maximum number of simultaneous
 * connections on the device has been reached.
 */
#define RESP_MAX_CONNECTIONS	0xfe

/** Buffer size in bytes. */
#define BUFFER_SIZE	2048

/** Timeout of a receive operation in milliseconds. */
#define RECV_TIMEOUT	5000
/** Timeout of a send operation in milliseconds. */
#define SEND_TIMEOUT	5000

/** String of the port number for the J-Link TCP/IP protocol. */
#define PORT_STRING	"19020"

/** Size of the server's hello message in bytes. */
#define SERVER_HELLO_SIZE	4
/**
 * Maximum length of the server name including trailing null-terminator in
 * bytes.
 */
#define SERVER_NAME_MAX_LENGTH	256
/** @endcond */

static int initialize_handle(struct jaylink_device_handle *devh)
{
	struct jaylink_context *ctx;

	ctx = devh->dev->ctx;

	devh->buffer_size = BUFFER_SIZE;
	devh->buffer = malloc(devh->buffer_size);

	if (!devh->buffer) {
		log_err(ctx, "Transport buffer malloc failed.");
		return JAYLINK_ERR_MALLOC;
	}

	devh->read_length = 0;
	devh->bytes_available = 0;
	devh->read_pos = 0;

	devh->write_length = 0;
	devh->write_pos = 0;

	return JAYLINK_OK;
}

static void cleanup_handle(struct jaylink_device_handle *devh)
{
	free(devh->buffer);
}

static int _recv(struct jaylink_device_handle *devh, uint8_t *buffer,
		size_t length)
{
	struct jaylink_context *ctx;
	size_t tmp;

	ctx = devh->dev->ctx;

	while (length > 0) {
		tmp = length;

		if (!socket_recv(devh->sock, buffer, &tmp, 0)) {
			log_err(ctx, "Failed to receive data from device.");
			return JAYLINK_ERR_IO;
		} else if (!tmp) {
			log_err(ctx, "Failed to receive data from device: "
				"remote connection closed.");
			return JAYLINK_ERR_IO;
		}

		buffer += tmp;
		length -= tmp;

		log_dbgio(ctx, "Received %zu bytes from device.", tmp);
	}

	return JAYLINK_OK;
}

static int handle_server_hello(struct jaylink_device_handle *devh)
{
	int ret;
	struct jaylink_context *ctx;
	uint8_t buf[SERVER_HELLO_SIZE];
	char name[SERVER_NAME_MAX_LENGTH];
	uint16_t proto_version;
	size_t length;

	ctx = devh->dev->ctx;

	ret = _recv(devh, buf, sizeof(buf));

	if (ret != JAYLINK_OK) {
		log_err(ctx, "Failed to receive hello message.");
		return ret;
	}

	if (buf[0] == RESP_MAX_CONNECTIONS) {
		log_err(ctx, "Maximum number of connections reached.");
		return JAYLINK_ERR;
	}

	if (buf[0] != CMD_SERVER) {
		log_err(ctx, "Invalid hello message received.");
		return JAYLINK_ERR_PROTO;
	}

	proto_version = buffer_get_u16(buf, 1);

	log_dbg(ctx, "Protocol version: 0x%04x.", proto_version);

	length = buf[3];
	ret = _recv(devh, (uint8_t *)name, length);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "Failed to receive server name.");
		return ret;
	}

	name[length] = '\0';

	log_dbg(ctx, "Server name: %s.", name);

	return JAYLINK_OK;
}

static int set_socket_timeouts(struct jaylink_device_handle *devh)
{
	struct jaylink_context *ctx;

	ctx = devh->dev->ctx;
#ifdef _WIN32
	DWORD timeout;

	timeout = RECV_TIMEOUT;

	if (!socket_set_option(devh->sock, SOL_SOCKET, SO_RCVTIMEO, &timeout,
			sizeof(timeout))) {
		log_err(ctx, "Failed to set socket receive timeout.");
		return JAYLINK_ERR;
	}

	timeout = SEND_TIMEOUT;

	if (!socket_set_option(devh->sock, SOL_SOCKET, SO_SNDTIMEO, &timeout,
			sizeof(timeout))) {
		log_err(ctx, "Failed to set socket send timeout.");
		return JAYLINK_ERR;
	}
#else
	struct timeval timeout;

	timeout.tv_sec = RECV_TIMEOUT / 1000;
	timeout.tv_usec = (RECV_TIMEOUT % 1000) * 1000;

	if (!socket_set_option(devh->sock, SOL_SOCKET, SO_RCVTIMEO, &timeout,
			sizeof(struct timeval))) {
		log_err(ctx, "Failed to set socket receive timeout.");
		return JAYLINK_ERR;
	}

	timeout.tv_sec = SEND_TIMEOUT / 1000;
	timeout.tv_usec = (SEND_TIMEOUT % 1000) * 1000;

	if (!socket_set_option(devh->sock, SOL_SOCKET, SO_SNDTIMEO, &timeout,
			sizeof(struct timeval))) {
		log_err(ctx, "Failed to set socket send timeout.");
		return JAYLINK_ERR;
	}
#endif
	return JAYLINK_OK;
}

JAYLINK_PRIV int transport_tcp_open(struct jaylink_device_handle *devh)
{
	int ret;
	struct jaylink_context *ctx;
	struct jaylink_device *dev;
	struct addrinfo hints;
	struct addrinfo *info;
	struct addrinfo *rp;
	int sock;

	dev = devh->dev;
	ctx = dev->ctx;

	log_dbg(ctx, "Trying to open device (IPv4 address = %s).",
		dev->ipv4_address);

	ret = initialize_handle(devh);

	if (ret != JAYLINK_OK) {
		log_err(ctx, "Initialize device handle failed.");
		return ret;
	}

	memset(&hints, 0, sizeof(struct addrinfo));
	hints.ai_family = AF_INET;
	hints.ai_socktype = SOCK_STREAM;
	hints.ai_protocol = IPPROTO_TCP;

	ret = getaddrinfo(dev->ipv4_address, PORT_STRING, &hints, &info);

	if (ret != 0) {
		log_err(ctx, "Address lookup failed.");
		cleanup_handle(devh);
		return JAYLINK_ERR;
	}

	sock = -1;

	for (rp = info; rp != NULL; rp = rp->ai_next) {
		sock = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);

		if (sock < 0)
			continue;

		if (!connect(sock, info->ai_addr, info->ai_addrlen))
			break;

		socket_close(sock);
		sock = -1;
	}

	freeaddrinfo(info);

	if (sock < 0) {
		log_err(ctx, "Failed to open device.");
		cleanup_handle(devh);
		return JAYLINK_ERR;
	}

	log_dbg(ctx, "Device opened successfully.");

	devh->sock = sock;
	ret = set_socket_timeouts(devh);

	if (ret != JAYLINK_OK) {
		socket_close(sock);
		cleanup_handle(devh);
		return ret;
	}

	ret = handle_server_hello(devh);

	if (ret != JAYLINK_OK) {
		socket_close(sock);
		cleanup_handle(devh);
		return ret;
	}

	return JAYLINK_OK;
}

JAYLINK_PRIV int transport_tcp_close(struct jaylink_device_handle *devh)
{
	struct jaylink_context *ctx;

	ctx = devh->dev->ctx;

	log_dbg(ctx, "Closing device (IPv4 address = %s).",
		devh->dev->ipv4_address);

	cleanup_handle(devh);

	log_dbg(ctx, "Device closed successfully.");

	return JAYLINK_OK;
}

JAYLINK_PRIV int transport_tcp_start_write(struct jaylink_device_handle *devh,
		size_t length, bool has_command)
{
	struct jaylink_context *ctx;

	if (!length)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;

	log_dbgio(ctx, "Starting write operation (length = %zu bytes).",
		length);

	if (devh->write_pos > 0)
		log_warn(ctx, "Last write operation left %zu bytes in the "
			"buffer.", devh->write_pos);

	if (devh->write_length > 0)
		log_warn(ctx, "Last write operation was not performed.");

	devh->write_length = length;
	devh->write_pos = 0;

	if (has_command) {
		devh->buffer[0] = CMD_CLIENT;
		devh->write_pos++;
	}

	return JAYLINK_OK;
}

JAYLINK_PRIV int transport_tcp_start_read(struct jaylink_device_handle *devh,
		size_t length)
{
	struct jaylink_context *ctx;

	if (!length)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;

	log_dbgio(ctx, "Starting read operation (length = %zu bytes).",
		length);

	if (devh->bytes_available > 0)
		log_dbg(ctx, "Last read operation left %zu bytes in the "
			"buffer.", devh->bytes_available);

	if (devh->read_length > 0)
		log_warn(ctx, "Last read operation left %zu bytes.",
			devh->read_length);

	devh->read_length = length;

	return JAYLINK_OK;
}

JAYLINK_PRIV int transport_tcp_start_write_read(
		struct jaylink_device_handle *devh, size_t write_length,
		size_t read_length, bool has_command)
{
	struct jaylink_context *ctx;

	if (!read_length || !write_length)
		return JAYLINK_ERR_ARG;

	ctx = devh->dev->ctx;

	log_dbgio(ctx, "Starting write / read operation (length = "
		"%zu / %zu bytes).", write_length, read_length);

	if (devh->write_pos > 0)
		log_warn(ctx, "Last write operation left %zu bytes in the "
			"buffer.", devh->write_pos);

	if (devh->write_length > 0)
		log_warn(ctx, "Last write operation was not performed.");

	if (devh->bytes_available > 0)
		log_warn(ctx, "Last read operation left %zu bytes in the "
			"buffer.", devh->bytes_available);

	if (devh->read_length > 0)
		log_warn(ctx, "Last read operation left %zu bytes.",
			devh->read_length);

	devh->write_length = write_length;
	devh->write_pos = 0;

	if (has_command) {
		devh->buffer[0] = CMD_CLIENT;
		devh->write_pos++;
	}

	devh->read_length = read_length;
	devh->bytes_available = 0;
	devh->read_pos = 0;

	return JAYLINK_OK;
}

static int _send(struct jaylink_device_handle *devh, const uint8_t *buffer,
		size_t length)
{
	struct jaylink_context *ctx;
	size_t tmp;

	ctx = devh->dev->ctx;

	while (length > 0) {
		tmp = length;

		if (!socket_send(devh->sock, buffer, &tmp, 0)) {
			log_err(ctx, "Failed to send data to device.");
			return JAYLINK_ERR_IO;
		}

		buffer += tmp;
		length -= tmp;

		log_dbgio(ctx, "Sent %zu bytes to device.", tmp);
	}

	return JAYLINK_OK;
}

static bool adjust_buffer(struct jaylink_device_handle *devh, size_t size)
{
	struct jaylink_context *ctx;
	uint8_t *buffer;
	size_t num;

	ctx = devh->dev->ctx;

	/* Adjust buffer size to a multiple of BUFFER_SIZE bytes. */
	num = size / BUFFER_SIZE;

	if (size % BUFFER_SIZE > 0)
		num++;

	size = num * BUFFER_SIZE;
	buffer = realloc(devh->buffer, size);

	if (!buffer) {
		log_err(ctx, "Failed to adjust buffer size to %zu bytes.",
			size);
		return false;
	}

	devh->buffer = buffer;
	devh->buffer_size = size;

	log_dbg(ctx, "Adjusted buffer size to %zu bytes.", size);

	return true;
}

JAYLINK_PRIV int transport_tcp_write(struct jaylink_device_handle *devh,
		const uint8_t *buffer, size_t length)
{
	int ret;
	struct jaylink_context *ctx;
	size_t tmp;

	ctx = devh->dev->ctx;

	if (length > devh->write_length) {
		log_err(ctx, "Requested to write %zu bytes but only %zu bytes "
			"are expected for the write operation.", length,
			devh->write_length);
		return JAYLINK_ERR_ARG;
	}

	/*
	 * Store data in the buffer if the expected number of bytes for the
	 * write operation is not reached.
	 */
	if (length < devh->write_length) {
		if (devh->write_pos + length > devh->buffer_size) {
			if (!adjust_buffer(devh, devh->write_pos + length))
				return JAYLINK_ERR_MALLOC;
		}

		memcpy(devh->buffer + devh->write_pos, buffer, length);

		devh->write_length -= length;
		devh->write_pos += length;

		log_dbgio(ctx, "Wrote %zu bytes into buffer.", length);
		return JAYLINK_OK;
	}

	/*
	 * Expected number of bytes for this write operation is reached and
	 * therefore the write operation will be performed.
	 */
	devh->write_length = 0;

	/* Send data directly to the device if the buffer is empty. */
	if (!devh->write_pos)
		return _send(devh, buffer, length);

	tmp = MIN(length, devh->buffer_size - devh->write_pos);

	/*
	 * Fill up the internal buffer in order to reduce the number of
	 * messages sent to the device for performance reasons.
	 */
	memcpy(devh->buffer + devh->write_pos, buffer, tmp);

	length -= tmp;
	buffer += tmp;

	log_dbgio(ctx, "Buffer filled up with %zu bytes.", tmp);

	ret = _send(devh, devh->buffer, devh->write_pos + tmp);

	devh->write_pos = 0;

	if (ret != JAYLINK_OK)
		return ret;

	if (!length)
		return JAYLINK_OK;

	return _send(devh, buffer, length);
}

JAYLINK_PRIV int transport_tcp_read(struct jaylink_device_handle *devh,
		uint8_t *buffer, size_t length)
{
	int ret;
	struct jaylink_context *ctx;

	ctx = devh->dev->ctx;

	if (length > devh->read_length) {
		log_err(ctx, "Requested to read %zu bytes but only %zu bytes "
			"are expected for the read operation.", length,
			devh->read_length);
		return JAYLINK_ERR_ARG;
	}

	if (length <= devh->bytes_available) {
		memcpy(buffer, devh->buffer + devh->read_pos, length);

		devh->read_length -= length;
		devh->bytes_available -= length;
		devh->read_pos += length;

		log_dbgio(ctx, "Read %zu bytes from buffer.", length);
		return JAYLINK_OK;
	}

	if (devh->bytes_available) {
		memcpy(buffer, devh->buffer + devh->read_pos,
			devh->bytes_available);

		buffer += devh->bytes_available;
		length -= devh->bytes_available;
		devh->read_length -= devh->bytes_available;

		log_dbgio(ctx, "Read %zu bytes from buffer to flush it.",
			devh->bytes_available);

		devh->bytes_available = 0;
		devh->read_pos = 0;
	}

	ret = _recv(devh, buffer, length);

	if (ret != JAYLINK_OK)
		return ret;

	devh->read_length -= length;

	return JAYLINK_OK;
}
