// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *	 Provides CMSIS-DAP protocol over a TCP/IP socket.					   *
 *	 UART and SWO are currently unsupported.							   *
 *																		   *
 *	 Copyright (C) 2025 by Brian Kuschak <bkuschak@gmail.com>			   *
 *																		   *
 *	 Adapted from cmsis_dap_usb_hid.c. Copyright (C) 2013-2018 by:		   *
 *	   Mickaël Thomas <mickael9@gmail.com>								   *
 *	   Maksym Hilliaka <oter@frozen-team.com>							   *
 *	   Phillip Pearson <pp@myelin.co.nz>								   *
 *	   Paul Fertser <fercerpav@gmail.com>								   *
 *	   mike brown <mike@theshedworks.org.uk>							   *
 *	   Spencer Oliver <spen@spen-soft.co.uk>							   *
 *																		   *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef HAVE_ARPA_INET_H
#include <arpa/inet.h>
#endif
#include <errno.h>
#ifdef HAVE_NETDB_H
#include <netdb.h>
#endif
#ifdef HAVE_NETINET_TCP_H
#include <netinet/tcp.h>
#endif
#include <stdbool.h>
#include <string.h>
#ifdef HAVE_SYS_IOCTL_H
#include <sys/ioctl.h>
#endif
#ifdef HAVE_SYS_SOCKET_H
#include <sys/socket.h>
#endif
#include <sys/types.h>
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#endif

#include "helper/command.h"
#include "helper/log.h"
#include "helper/replacements.h"
#include "helper/system.h"
#include "cmsis_dap.h"

#define STRINGIFY(x) #x

// If the protocol changes in the future, the SIGNATURE should also be changed.
#define DAP_PKT_HDR_SIGNATURE	0x00504144	// "DAP"
#define DAP_PKT_TYPE_REQUEST	0x01
#define DAP_PKT_TYPE_RESPONSE	0x02

#define CMSIS_DAP_TCP_PORT		4441		// Default. Can be overridden.
#define CMSIS_DAP_PACKET_SIZE	1024		// Max payload size not including
											// header.

/* When flushing after an error, the CMSIS-DAP driver assumes the pipeline is
 * empty if it doesn't get a response after a short 10 msec timeout. While this
 * works for USB, it may not work for TCP/IP due to higher network latency. TCP
 * response packets may take longer to arrive. We set a lower bound on timeout
 * for blocking reads, to give enough time for packets to arrive.
 *
 * The user may override this default value by setting the parameter
 * 'cmsis-dap tcp min_timeout'
 */
#define DEFAULT_MIN_TIMEOUT_MS	150

/* CMSIS-DAP requests are variable length. With CMSIS-DAP over USB, the
 * transfer sizes are preserved by the USB stack. However, TCP/IP is stream
 * oriented so we perform our own packetization to preserve the boundaries
 * between each request. This short header is prepended to each CMSIS-DAP
 * request and response before being sent over the socket. Little endian format
 * is used for multibyte values.
 */
struct __attribute__((packed)) cmsis_dap_tcp_packet_hdr {
	uint32_t signature;		// "DAP"
	uint16_t length;		// Not including header length.
	uint8_t packet_type;
	uint8_t reserved;		// Reserved for future use.
};

/* Defines for struct cmsis_dap_tcp_packet_hdr requested by reviewer. */
#define HEADER_SIGNATURE_OFFSET		0
#define HEADER_LENGTH_OFFSET		sizeof(uint32_t)
#define HEADER_PACKET_TYPE_OFFSET	(sizeof(uint32_t) + sizeof(uint16_t))
#define HEADER_RESERVED_OFFSET		(sizeof(uint32_t) + sizeof(uint16_t) + \
									 sizeof(uint8_t))
#define HEADER_SIZE					(sizeof(uint32_t) + sizeof(uint16_t) + \
									 2 * sizeof(uint8_t))

struct cmsis_dap_backend_data {
	int sockfd;
};

static char *cmsis_dap_tcp_host;
static char cmsis_dap_tcp_port_default[] = STRINGIFY(CMSIS_DAP_TCP_PORT);
static char *cmsis_dap_tcp_port = cmsis_dap_tcp_port_default;
static int cmsis_dap_tcp_min_timeout_ms = DEFAULT_MIN_TIMEOUT_MS;

static void cmsis_dap_tcp_close(struct cmsis_dap *dap);
static int cmsis_dap_tcp_alloc(struct cmsis_dap *dap, unsigned int pkt_sz);
static void cmsis_dap_tcp_free(struct cmsis_dap *dap);

static int cmsis_dap_tcp_open(struct cmsis_dap *dap,
		uint16_t vids[] __attribute__((unused)),
		uint16_t pids[] __attribute__((unused)),
		const char *serial __attribute__((unused)))
{
	// Skip the open if the user has not provided a hostname.
	if (!cmsis_dap_tcp_host) {
		LOG_DEBUG("No TCP hostname, skipping open.");
		return ERROR_FAIL;
	}

	// Ignore vids, pids, serial. We use host and port subcommands instead.

	dap->bdata = malloc(sizeof(struct cmsis_dap_backend_data));
	if (!dap->bdata) {
		LOG_ERROR("CMSIS-DAP: unable to allocate memory");
		return ERROR_FAIL;
	}

	struct addrinfo hints = {
		.ai_family = AF_UNSPEC,
		.ai_socktype = SOCK_STREAM
	};
	struct addrinfo *result, *rp;
	int fd = 0;

	LOG_INFO("CMSIS-DAP: Connecting to %s:%s using TCP backend",
			cmsis_dap_tcp_host ? cmsis_dap_tcp_host : "localhost",
			cmsis_dap_tcp_port);

	/* Some of the following code was taken from remote_bitbang.c */
	/* Obtain address(es) matching host/port */
	int s = getaddrinfo(cmsis_dap_tcp_host, cmsis_dap_tcp_port, &hints,
			&result);
	if (s != 0) {
		LOG_ERROR("CMSIS-DAP: getaddrinfo: %s\n", gai_strerror(s));
		free(dap->bdata);
		return ERROR_FAIL;
	}

	/* getaddrinfo() returns a list of address structures.
	 Try each address until we successfully connect(2).
	 If socket(2) (or connect(2)) fails, we (close the socket
	 and) try the next address. */

	for (rp = result; rp ; rp = rp->ai_next) {
		fd = socket(rp->ai_family, rp->ai_socktype, rp->ai_protocol);
		if (fd == -1)
			continue;

		if (connect(fd, rp->ai_addr, rp->ai_addrlen) != -1) {
			LOG_DEBUG("Connected.");
			break; /* Success */
		}

		close(fd);
	}

	freeaddrinfo(result);

	if (!rp) { /* No address succeeded */
		LOG_ERROR("CMSIS-DAP: unable to connect to device %s:%s",
			cmsis_dap_tcp_host ? cmsis_dap_tcp_host : "localhost",
			cmsis_dap_tcp_port);
		log_socket_error("Failed to connect");
		free(dap->bdata);
		dap->bdata = NULL;
		return ERROR_FAIL;
	}

	/* Set NODELAY to minimize latency. */
	int one = 1;
	/* On Windows optval has to be a const char *. */
	setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, (const char *)&one, sizeof(one));

	dap->bdata->sockfd = fd;

	int retval = cmsis_dap_tcp_alloc(dap, CMSIS_DAP_PACKET_SIZE);
	if (retval != ERROR_OK) {
		cmsis_dap_tcp_close(dap);
		return retval;
	}
	return ERROR_OK;
}

static void cmsis_dap_tcp_close(struct cmsis_dap *dap)
{
	if (close_socket(dap->bdata->sockfd) != 0)
		log_socket_error("close_socket");

	if (dap->bdata)
		free(dap->bdata);
	dap->bdata = NULL;
	cmsis_dap_tcp_free(dap);
}

static int socket_bytes_available(int sock, unsigned int *out_avail)
{
#ifdef _WIN32
	u_long avail = 0;
	if (ioctlsocket((SOCKET)sock, FIONREAD, &avail) == SOCKET_ERROR)
		return -1;
#else
	int avail = 0;
	if (ioctl(sock, FIONREAD, &avail) < 0)
		return -1;
#endif
	*out_avail = avail;
	return 0;
}

static inline int readall_socket(int handle, void *buffer, unsigned int count)
{
	// Return after all count bytes available, or timeout, or error.
	return recv(handle, buffer, count, MSG_WAITALL);
}

static int peekall_socket(int handle, void *buffer, unsigned int count,
		enum cmsis_dap_blocking blocking, unsigned int timeout_ms)
{
	/* Windows doesn't support MSG_PEEK in combination with MSG_WAITALL:
	 *	 return recv(handle, buffer, count, MSG_PEEK | MSG_WAITALL);
	 *
	 * So, use this method instead which should work for Windows and others.
	 *
	 * Data remains unread on the socket until recv() is called later without
	 * the MSG_PEEK flag. Return after all count bytes available, or timeout,
	 * or error.
	 */

	if (count == 0)
		return 0;

	while (true) {
		int ret;
		unsigned int avail;
		if (socket_bytes_available(handle, &avail) < 0)
			return -1;

		if (avail >= count) {
			ret = recv(handle, (char *)buffer, (int)count, MSG_PEEK);
			if (ret < 0) {
#ifdef _WIN32
				int err = WSAGetLastError();
				if (err == WSAEINTR)
					continue;
				if (err == WSAEWOULDBLOCK)
					return -1;
#else
				if (errno == EINTR)
					continue;
				if (errno == EAGAIN || errno == EWOULDBLOCK)
					return -1;	// Timeout or nonblocking.
#endif
			}
			return ret; // 0: Closed, <0: Other error, >0 Success.
		}

		// Not enough data available.
		if (blocking == CMSIS_DAP_NON_BLOCKING) {
#ifdef _WIN32
			WSASetLastError(WSAEWOULDBLOCK);
#else
			errno = EAGAIN;
#endif
			return -1;
		}

		// Blocking wait.
		fd_set rfds;
		FD_ZERO(&rfds);
		FD_SET(handle, &rfds);

		struct timeval tv;
		tv.tv_sec  = timeout_ms / 1000;
		tv.tv_usec = (timeout_ms % 1000) * 1000;

		ret = select(handle + 1, &rfds, NULL, NULL, &tv);
		if (ret > 0)
			continue;		// Readable

		if (ret == 0) {		// Timeout
#ifdef _WIN32
			WSASetLastError(WSAEWOULDBLOCK);
#else
			errno = EAGAIN;
#endif
			return -1;
		}

		// Error
#ifndef _WIN32
		if (errno == EINTR)
			continue;
#endif
		return ret;
	}
}

static int cmsis_dap_tcp_read(struct cmsis_dap *dap, int transfer_timeout_ms,
							  enum cmsis_dap_blocking blocking)
{
	int wait_ms = (blocking == CMSIS_DAP_NON_BLOCKING) ? 0 :
		transfer_timeout_ms;
	if (wait_ms) {
		LOG_DEBUG_IO("CMSIS-DAP: using tcp timeout %d msec", wait_ms);

		// Don't use very short timeouts with TCP/IP as it may not be as fast
		// to respond as USB. User configurable minimum value.
		if (wait_ms < cmsis_dap_tcp_min_timeout_ms) {
			wait_ms = cmsis_dap_tcp_min_timeout_ms;
			LOG_DEBUG_IO("CMSIS-DAP: extending timeout to %d msec", wait_ms);
		}
	}
	socket_recv_timeout(dap->bdata->sockfd, wait_ms);

	if (blocking == CMSIS_DAP_NON_BLOCKING)
		socket_nonblock(dap->bdata->sockfd);
	else
		socket_block(dap->bdata->sockfd);

	// Peek at the header first to find the length.
	int retval = peekall_socket(dap->bdata->sockfd, dap->packet_buffer,
			HEADER_SIZE, blocking, wait_ms);
	LOG_DEBUG_IO("Reading header returned %d", retval);
	if (retval == 0) {
		LOG_DEBUG_IO("CMSIS-DAP: tcp timeout reached 1");
		return ERROR_TIMEOUT_REACHED;
	} else if (retval == -1) {
		if (errno == EAGAIN || errno == EWOULDBLOCK) {
			if (blocking == CMSIS_DAP_NON_BLOCKING)
				return ERROR_TIMEOUT_REACHED;

			LOG_DEBUG_IO("CMSIS-DAP: tcp timeout reached 2. timeout = %d msec",
					wait_ms);
			return ERROR_TIMEOUT_REACHED;
		}

		LOG_ERROR("CMSIS-DAP: error reading header");
		log_socket_error("peek_socket");
		return ERROR_FAIL;
	} else if (retval != HEADER_SIZE) {
		LOG_ERROR("CMSIS-DAP: short header read");
		log_socket_error("peek_socket header short read");
		return ERROR_FAIL;
	}

	struct cmsis_dap_tcp_packet_hdr header;
	header.signature = le_to_h_u32(dap->packet_buffer +
			HEADER_SIGNATURE_OFFSET);
	header.length = le_to_h_u16(dap->packet_buffer + HEADER_LENGTH_OFFSET);
	header.packet_type = dap->packet_buffer[HEADER_PACKET_TYPE_OFFSET];
	header.reserved = dap->packet_buffer[HEADER_RESERVED_OFFSET];

	if (header.signature != DAP_PKT_HDR_SIGNATURE) {
		LOG_ERROR("CMSIS-DAP: Unrecognized packet signature 0x%08x",
				header.signature);
		return ERROR_FAIL;
	} else if (header.packet_type != DAP_PKT_TYPE_RESPONSE) {
		LOG_ERROR("CMSIS-DAP: Unrecognized packet type 0x%02x",
				header.packet_type);
		return ERROR_FAIL;
	} else if (header.length + HEADER_SIZE > dap->packet_buffer_size) {
		LOG_ERROR("CMSIS-DAP: Packet length %d too large to fit.",
				header.length);
		return ERROR_FAIL;
	}

	// Read the complete packet.
	int read_len = HEADER_SIZE + header.length;
	LOG_DEBUG_IO("Reading %d bytes (%d payload)...", read_len, header.length);
	retval = readall_socket(dap->bdata->sockfd, dap->packet_buffer, read_len);

	if (retval == 0) {
		LOG_DEBUG_IO("CMSIS-DAP: tcp timeout reached 3");
		return ERROR_TIMEOUT_REACHED;
	} else if (retval == -1) {
		LOG_ERROR("CMSIS-DAP: error reading data");
		log_socket_error("read_socket");
		return ERROR_FAIL;
	} else if (retval != read_len) {
		LOG_ERROR("CMSIS-DAP: short read. retval = %d. read_len = %d. "
				"blocking = %s. wait_ms = %d", retval, read_len,
				(blocking == CMSIS_DAP_NON_BLOCKING) ? "yes" : "no", wait_ms);
		log_socket_error("read_socket short read");
		return ERROR_FAIL;
	}
	return retval;
}

static int cmsis_dap_tcp_write(struct cmsis_dap *dap, int txlen,
		int timeout_ms __attribute__((unused)))
{
	const unsigned int len = txlen + HEADER_SIZE;
	if (len > dap->packet_buffer_size) {
		LOG_ERROR("CMSIS-DAP: Packet length %d exceeds TCP buffer size!", len);
		return ERROR_FAIL;
	}

	/* Set the header values. */
	h_u32_to_le(dap->packet_buffer + HEADER_SIGNATURE_OFFSET,
			DAP_PKT_HDR_SIGNATURE);
	h_u16_to_le(dap->packet_buffer + HEADER_LENGTH_OFFSET, txlen);
	dap->packet_buffer[HEADER_PACKET_TYPE_OFFSET] = DAP_PKT_TYPE_REQUEST;
	dap->packet_buffer[HEADER_RESERVED_OFFSET] = 0;

	/* write data to device */
	LOG_DEBUG_IO("Writing %d bytes (%d payload)", len, txlen);
	int retval = write_socket(dap->bdata->sockfd, dap->packet_buffer, len);
	if (retval < 0) {
		log_socket_error("write_socket");
		return ERROR_FAIL;
	} else if (retval != (int)len) {
		LOG_ERROR("CMSIS-DAP: error writing data");
		log_socket_error("write_socket short write");
		return ERROR_FAIL;
	}
	return retval;
}

static int cmsis_dap_tcp_alloc(struct cmsis_dap *dap, unsigned int pkt_sz)
{
	// Reserve space for the packet header.
	unsigned int packet_buffer_size = pkt_sz + HEADER_SIZE;
	uint8_t *buf = malloc(packet_buffer_size);
	if (!buf) {
		LOG_ERROR("CMSIS-DAP: unable to allocate CMSIS-DAP packet buffer");
		return ERROR_FAIL;
	}

	dap->packet_buffer = buf;
	dap->packet_size = pkt_sz;
	dap->packet_usable_size = pkt_sz;
	dap->packet_buffer_size = packet_buffer_size;

	dap->command = dap->packet_buffer + HEADER_SIZE;
	dap->response = dap->packet_buffer + HEADER_SIZE;
	return ERROR_OK;
}

static void cmsis_dap_tcp_free(struct cmsis_dap *dap)
{
	free(dap->packet_buffer);
	dap->packet_buffer = NULL;
}

static void cmsis_dap_tcp_cancel_all(struct cmsis_dap *dap)
{
}

COMMAND_HANDLER(cmsis_dap_handle_tcp_port)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (cmsis_dap_tcp_port != cmsis_dap_tcp_port_default)
		free(cmsis_dap_tcp_port);

	cmsis_dap_tcp_port = strdup(CMD_ARGV[0]);
	if (!cmsis_dap_tcp_port) {
		LOG_ERROR("CMSIS-DAP: out of memory");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(cmsis_dap_handle_tcp_host)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	free(cmsis_dap_tcp_host);
	cmsis_dap_tcp_host = strdup(CMD_ARGV[0]);
	if (!cmsis_dap_tcp_host) {
		LOG_ERROR("CMSIS-DAP: out of memory");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

COMMAND_HANDLER(cmsis_dap_handle_tcp_min_timeout)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], cmsis_dap_tcp_min_timeout_ms);
	LOG_INFO("CMSIS-DAP: using minimum timeout of %d ms for TCP packets.",
			cmsis_dap_tcp_min_timeout_ms);
	return ERROR_OK;
}

const struct command_registration cmsis_dap_tcp_subcommand_handlers[] = {
	{
		.name = "host",
		.handler = &cmsis_dap_handle_tcp_host,
		.mode = COMMAND_CONFIG,
		.help = "set the host name to use (for TCP backend only)",
		.usage = "<host_name>",
	},
	{
		.name = "port",
		.handler = &cmsis_dap_handle_tcp_port,
		.mode = COMMAND_CONFIG,
		.help = "set the port number to use for DAP (for TCP backend only)",
		.usage = "<port_number>",
	},
	{
		.name = "min_timeout",
		.handler = &cmsis_dap_handle_tcp_min_timeout,
		.mode = COMMAND_CONFIG,
		.help = "set the minimum timeout in milliseconds to wait for response "
			"packets (for TCP backend only)",
		.usage = "<milliseconds>",
	},
	COMMAND_REGISTRATION_DONE
};

const struct cmsis_dap_backend cmsis_dap_tcp_backend = {
	.name = "tcp",
	.open = cmsis_dap_tcp_open,
	.close = cmsis_dap_tcp_close,
	.read = cmsis_dap_tcp_read,
	.write = cmsis_dap_tcp_write,
	.packet_buffer_alloc = cmsis_dap_tcp_alloc,
	.packet_buffer_free = cmsis_dap_tcp_free,
	.cancel_all = cmsis_dap_tcp_cancel_all,
};
