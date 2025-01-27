// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2011 by Richard Uhler                                   *
 *   ruhler@mit.edu                                                        *
 *                                                                         *
 *   Copyright (C) 2021 by Manuel Wick <manuel@matronix.de>                *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifndef _WIN32
#include <sys/un.h>
#include <netdb.h>
#include <netinet/tcp.h>
#endif
#include "helper/system.h"
#include "helper/replacements.h"
#include <jtag/interface.h>
#include "bitbang.h"

/* arbitrary limit on host name length: */
#define REMOTE_BITBANG_HOST_MAX 255

static char *remote_bitbang_host;
static char *remote_bitbang_port;

static int remote_bitbang_fd;
static uint8_t remote_bitbang_send_buf[512];
static unsigned int remote_bitbang_send_buf_used;

static bool use_remote_sleep;

/* Circular buffer. When start == end, the buffer is empty. */
static char remote_bitbang_recv_buf[256];
static unsigned int remote_bitbang_recv_buf_start;
static unsigned int remote_bitbang_recv_buf_end;

static bool remote_bitbang_recv_buf_full(void)
{
	return remote_bitbang_recv_buf_end ==
		((remote_bitbang_recv_buf_start + sizeof(remote_bitbang_recv_buf) - 1) %
		 sizeof(remote_bitbang_recv_buf));
}

static bool remote_bitbang_recv_buf_empty(void)
{
	return remote_bitbang_recv_buf_start == remote_bitbang_recv_buf_end;
}

static unsigned int remote_bitbang_recv_buf_contiguous_available_space(void)
{
	if (remote_bitbang_recv_buf_end >= remote_bitbang_recv_buf_start) {
		unsigned int space = sizeof(remote_bitbang_recv_buf) -
				     remote_bitbang_recv_buf_end;
		if (remote_bitbang_recv_buf_start == 0)
			space -= 1;
		return space;
	} else {
		return remote_bitbang_recv_buf_start -
		       remote_bitbang_recv_buf_end - 1;
	}
}

static int remote_bitbang_flush(void)
{
	if (remote_bitbang_send_buf_used <= 0)
		return ERROR_OK;

	unsigned int offset = 0;
	while (offset < remote_bitbang_send_buf_used) {
		ssize_t written = write_socket(remote_bitbang_fd, remote_bitbang_send_buf + offset,
									   remote_bitbang_send_buf_used - offset);
		if (written < 0) {
			log_socket_error("remote_bitbang_putc");
			remote_bitbang_send_buf_used = 0;
			return ERROR_FAIL;
		}
		offset += written;
	}
	remote_bitbang_send_buf_used = 0;
	return ERROR_OK;
}

enum block_bool {
	NO_BLOCK,
	BLOCK
};

/* Read any incoming data, placing it into the buffer. */
static int remote_bitbang_fill_buf(enum block_bool block)
{
	if (remote_bitbang_recv_buf_empty()) {
		/* If the buffer is empty, reset it to 0 so we get more
		 * contiguous space. */
		remote_bitbang_recv_buf_start = 0;
		remote_bitbang_recv_buf_end = 0;
	}

	if (block == BLOCK) {
		if (remote_bitbang_flush() != ERROR_OK)
			return ERROR_FAIL;
		socket_block(remote_bitbang_fd);
	}

	bool first = true;
	while (!remote_bitbang_recv_buf_full()) {
		unsigned int contiguous_available_space =
				remote_bitbang_recv_buf_contiguous_available_space();
		ssize_t count = read_socket(remote_bitbang_fd,
				remote_bitbang_recv_buf + remote_bitbang_recv_buf_end,
				contiguous_available_space);
		if (first && block == BLOCK)
			socket_nonblock(remote_bitbang_fd);
		if (count > 0) {
			remote_bitbang_recv_buf_end += count;
			if (remote_bitbang_recv_buf_end == sizeof(remote_bitbang_recv_buf))
				remote_bitbang_recv_buf_end = 0;
		} else if (count == 0) {
			/* When read_socket returns 0, socket reached EOF and there is
			 * no data to read. But if request was blocking, the caller
			 * expected some data. Such situations should be treated as ERROR. */
			if (first && block == BLOCK) {
				LOG_ERROR("remote_bitbang: socket closed by remote");
				return ERROR_FAIL;
			}
			return ERROR_OK;
		} else if (count < 0) {
#ifdef _WIN32
			if (WSAGetLastError() == WSAEWOULDBLOCK) {
#else
			if (errno == EAGAIN) {
#endif
				return ERROR_OK;
			} else {
				log_socket_error("remote_bitbang_fill_buf");
				return ERROR_FAIL;
			}
		}
		first = false;
	}

	return ERROR_OK;
}

typedef enum {
	NO_FLUSH,
	FLUSH_SEND_BUF
} flush_bool_t;

static int remote_bitbang_queue(int c, flush_bool_t flush)
{
	remote_bitbang_send_buf[remote_bitbang_send_buf_used++] = c;
	if (flush == FLUSH_SEND_BUF ||
			remote_bitbang_send_buf_used >= ARRAY_SIZE(remote_bitbang_send_buf))
		return remote_bitbang_flush();
	return ERROR_OK;
}

static int remote_bitbang_quit(void)
{
	if (remote_bitbang_queue('Q', FLUSH_SEND_BUF) == ERROR_FAIL)
		return ERROR_FAIL;

	if (close_socket(remote_bitbang_fd) != 0) {
		log_socket_error("close_socket");
		return ERROR_FAIL;
	}

	free(remote_bitbang_host);
	free(remote_bitbang_port);

	LOG_INFO("remote_bitbang interface quit");
	return ERROR_OK;
}

static bb_value_t char_to_int(int c)
{
	switch (c) {
		case '0':
			return BB_LOW;
		case '1':
			return BB_HIGH;
		default:
			remote_bitbang_quit();
			LOG_ERROR("remote_bitbang: invalid read response: %c(%i)", c, c);
			return BB_ERROR;
	}
}

static int remote_bitbang_sample(void)
{
	if (remote_bitbang_fill_buf(NO_BLOCK) != ERROR_OK)
		return ERROR_FAIL;
	assert(!remote_bitbang_recv_buf_full());
	return remote_bitbang_queue('R', NO_FLUSH);
}

static bb_value_t remote_bitbang_read_sample(void)
{
	if (remote_bitbang_recv_buf_empty()) {
		if (remote_bitbang_fill_buf(BLOCK) != ERROR_OK)
			return BB_ERROR;
	}
	assert(!remote_bitbang_recv_buf_empty());
	int c = remote_bitbang_recv_buf[remote_bitbang_recv_buf_start];
	remote_bitbang_recv_buf_start =
		(remote_bitbang_recv_buf_start + 1) % sizeof(remote_bitbang_recv_buf);
	return char_to_int(c);
}

static int remote_bitbang_write(int tck, int tms, int tdi)
{
	char c = '0' + ((tck ? 0x4 : 0x0) | (tms ? 0x2 : 0x0) | (tdi ? 0x1 : 0x0));
	return remote_bitbang_queue(c, NO_FLUSH);
}

static int remote_bitbang_reset(int trst, int srst)
{
	char c = 'r' + ((trst ? 0x2 : 0x0) | (srst ? 0x1 : 0x0));
	/* Always flush the send buffer on reset, because the reset call need not be
	 * followed by jtag_execute_queue(). */
	return remote_bitbang_queue(c, FLUSH_SEND_BUF);
}

static int remote_bitbang_sleep(unsigned int microseconds)
{
	if (!use_remote_sleep) {
		jtag_sleep(microseconds);
		return ERROR_OK;
	}

	int tmp;
	unsigned int ms = microseconds / 1000;
	unsigned int us = microseconds % 1000;

	for (unsigned int i = 0; i < ms; i++) {
		tmp = remote_bitbang_queue('Z', NO_FLUSH);
		if (tmp != ERROR_OK)
			return tmp;
	}

	for (unsigned int i = 0; i < us; i++) {
		tmp = remote_bitbang_queue('z', NO_FLUSH);
		if (tmp != ERROR_OK)
			return tmp;
	}

	return remote_bitbang_flush();
}

static int remote_bitbang_blink(bool on)
{
	char c = on ? 'B' : 'b';
	return remote_bitbang_queue(c, FLUSH_SEND_BUF);
}

static void remote_bitbang_swdio_drive(bool is_output)
{
	char c = is_output ? 'O' : 'o';
	if (remote_bitbang_queue(c, FLUSH_SEND_BUF) == ERROR_FAIL)
		LOG_ERROR("Error setting direction for swdio");
}

static int remote_bitbang_swdio_read(void)
{
	if (remote_bitbang_queue('c', FLUSH_SEND_BUF) != ERROR_FAIL)
		return remote_bitbang_read_sample();
	else
		return BB_ERROR;
}

static int remote_bitbang_swd_write(int swclk, int swdio)
{
	char c = 'd' + ((swclk ? 0x2 : 0x0) | (swdio ? 0x1 : 0x0));
	return remote_bitbang_queue(c, NO_FLUSH);
}

static const struct bitbang_interface remote_bitbang_bitbang = {
	.buf_size = sizeof(remote_bitbang_recv_buf) - 1,
	.sample = &remote_bitbang_sample,
	.read_sample = &remote_bitbang_read_sample,
	.write = &remote_bitbang_write,
	.swdio_read = &remote_bitbang_swdio_read,
	.swdio_drive = &remote_bitbang_swdio_drive,
	.swd_write = &remote_bitbang_swd_write,
	.blink = &remote_bitbang_blink,
	.sleep = &remote_bitbang_sleep,
	.flush = &remote_bitbang_flush,
};

static int remote_bitbang_init_tcp(void)
{
	struct addrinfo hints = { .ai_family = AF_UNSPEC, .ai_socktype = SOCK_STREAM };
	struct addrinfo *result, *rp;
	int fd = 0;

	LOG_INFO("Connecting to %s:%s",
			remote_bitbang_host ? remote_bitbang_host : "localhost",
			remote_bitbang_port);

	/* Obtain address(es) matching host/port */
	int s = getaddrinfo(remote_bitbang_host, remote_bitbang_port, &hints, &result);
	if (s != 0) {
		LOG_ERROR("getaddrinfo: %s\n", gai_strerror(s));
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

		if (connect(fd, rp->ai_addr, rp->ai_addrlen) != -1)
			break; /* Success */

		close(fd);
	}

	/* We work hard to collapse the writes into the minimum number, so when
	 * we write something we want to get it to the other end of the
	 * connection as fast as possible. */
	int one = 1;
	/* On Windows optval has to be a const char *. */
	setsockopt(fd, IPPROTO_TCP, TCP_NODELAY, (const char *)&one, sizeof(one));

	freeaddrinfo(result); /* No longer needed */

	if (!rp) { /* No address succeeded */
		log_socket_error("Failed to connect");
		return ERROR_FAIL;
	}

	return fd;
}

static int remote_bitbang_init_unix(void)
{
	if (!remote_bitbang_host) {
		LOG_ERROR("host/socket not specified");
		return ERROR_FAIL;
	}

	LOG_INFO("Connecting to unix socket %s", remote_bitbang_host);
	int fd = socket(PF_UNIX, SOCK_STREAM, 0);
	if (fd < 0) {
		log_socket_error("socket");
		return ERROR_FAIL;
	}

	struct sockaddr_un addr;
	addr.sun_family = AF_UNIX;
	strncpy(addr.sun_path, remote_bitbang_host, sizeof(addr.sun_path));
	addr.sun_path[sizeof(addr.sun_path)-1] = '\0';

	if (connect(fd, (struct sockaddr *)&addr, sizeof(struct sockaddr_un)) < 0) {
		log_socket_error("connect");
		return ERROR_FAIL;
	}

	return fd;
}

static int remote_bitbang_init(void)
{
	bitbang_interface = &remote_bitbang_bitbang;

	remote_bitbang_recv_buf_start = 0;
	remote_bitbang_recv_buf_end = 0;

	LOG_INFO("Initializing remote_bitbang driver");
	if (!remote_bitbang_port)
		remote_bitbang_fd = remote_bitbang_init_unix();
	else
		remote_bitbang_fd = remote_bitbang_init_tcp();

	if (remote_bitbang_fd < 0)
		return remote_bitbang_fd;

	socket_nonblock(remote_bitbang_fd);

	LOG_INFO("remote_bitbang driver initialized");
	return ERROR_OK;
}

COMMAND_HANDLER(remote_bitbang_handle_remote_bitbang_port_command)
{
	if (CMD_ARGC == 1) {
		uint16_t port;
		COMMAND_PARSE_NUMBER(u16, CMD_ARGV[0], port);
		free(remote_bitbang_port);
		remote_bitbang_port = port == 0 ? NULL : strdup(CMD_ARGV[0]);
		return ERROR_OK;
	}
	return ERROR_COMMAND_SYNTAX_ERROR;
}

COMMAND_HANDLER(remote_bitbang_handle_remote_bitbang_host_command)
{
	if (CMD_ARGC == 1) {
		free(remote_bitbang_host);
		remote_bitbang_host = strdup(CMD_ARGV[0]);
		return ERROR_OK;
	}
	return ERROR_COMMAND_SYNTAX_ERROR;
}

static const char * const remote_bitbang_transports[] = { "jtag", "swd", NULL };

COMMAND_HANDLER(remote_bitbang_handle_remote_bitbang_use_remote_sleep_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], use_remote_sleep);

	return ERROR_OK;
}

static const struct command_registration remote_bitbang_subcommand_handlers[] = {
	{
		.name = "port",
		.handler = remote_bitbang_handle_remote_bitbang_port_command,
		.mode = COMMAND_CONFIG,
		.help = "Set the port to use to connect to the remote jtag.\n"
			"  if 0 or unset, use unix sockets to connect to the remote jtag.",
		.usage = "port_number",
	},
	{
		.name = "host",
		.handler = remote_bitbang_handle_remote_bitbang_host_command,
		.mode = COMMAND_CONFIG,
		.help = "Set the host to use to connect to the remote jtag.\n"
			"  if port is 0 or unset, this is the name of the unix socket to use.",
		.usage = "host_name",
	},
	{
		.name = "use_remote_sleep",
		.handler = remote_bitbang_handle_remote_bitbang_use_remote_sleep_command,
		.mode = COMMAND_CONFIG,
		.help = "Rather than executing sleep locally, include delays in the "
			"instruction stream for the remote host.",
		.usage = "(on|off)",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration remote_bitbang_command_handlers[] = {
	{
		.name = "remote_bitbang",
		.mode = COMMAND_ANY,
		.help = "perform remote_bitbang management",
		.chain = remote_bitbang_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

static int remote_bitbang_execute_queue(struct jtag_command *cmd_queue)
{
	/* safety: the send buffer must be empty, no leftover characters from
	 * previous transactions */
	assert(remote_bitbang_send_buf_used == 0);

	/* process the JTAG command queue */
	int ret = bitbang_execute_queue(cmd_queue);
	if (ret != ERROR_OK)
		return ret;

	/* flush not-yet-sent characters, if any */
	return remote_bitbang_flush();
}

static struct jtag_interface remote_bitbang_interface = {
	.execute_queue = &remote_bitbang_execute_queue,
};

struct adapter_driver remote_bitbang_adapter_driver = {
	.name = "remote_bitbang",
	.transports = remote_bitbang_transports,
	.commands = remote_bitbang_command_handlers,

	.init = &remote_bitbang_init,
	.quit = &remote_bitbang_quit,
	.reset = &remote_bitbang_reset,

	.jtag_ops = &remote_bitbang_interface,
	.swd_ops = &bitbang_swd,
};
