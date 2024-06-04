#include <netinet/in.h>
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#ifdef HAVE_ARPA_INET_H
#include <arpa/inet.h>
#endif

#ifndef _WIN32
#include <netinet/tcp.h>
#endif

/* For gethostbyname */
#include <netdb.h>

#include <helper/log.h>
#include <helper/replacements.h>
#include <helper/system.h>
#include <jtag/jtag.h> /* ERROR_JTAG_DEVICE_ERROR only */

#include "cmsis_dap.h"

static int cmsis_dap_net_server_port = 2806;
static char *cmsis_dap_net_server_address = NULL;


struct cmsis_dap_backend_data {
	uint16_t server_port;
	char *server_address;
	int sockfd;
	struct sockaddr_in serv_addr;
};

static void cmsis_dap_net_free(struct cmsis_dap *dap);

static void cmsis_dap_net_close(struct cmsis_dap *dap) {
	if (!dap->bdata) {
		return;
	}
	close(dap->bdata->sockfd);

	free(dap->bdata->server_address);
	free(dap->bdata);
	dap->bdata = NULL;

	cmsis_dap_net_free(dap);
}

static int cmsis_dap_net_alloc(struct cmsis_dap *dap, unsigned int pkt_sz) {
	unsigned int packet_buffer_size = pkt_sz;
	uint8_t *buf = malloc(packet_buffer_size);
	if (!buf) {
		LOG_ERROR("unable to allocate CMSIS-DAP packet buffer");
		return ERROR_FAIL;
	}

	dap->packet_buffer = buf;
	dap->packet_size = pkt_sz;
	dap->packet_usable_size = pkt_sz;
	dap->packet_buffer_size = packet_buffer_size;

	// LOG_INFO("Allocated CMSIS-DAP packet buffer of size %u", pkt_sz);

	return ERROR_OK;
}

static void cmsis_dap_net_free(struct cmsis_dap *dap) {
	free(dap->packet_buffer);
	dap->packet_buffer = NULL;
}

static int cmsis_dap_net_open(struct cmsis_dap *dap, uint16_t vids[],
							  uint16_t pids[], const char *serial) {

	if (!cmsis_dap_net_server_address) {
		LOG_ERROR("server address not set (cmsis-dap net address <address>)");
		return ERROR_FAIL;
	}

	struct cmsis_dap_backend_data *bdata =
		malloc(sizeof(struct cmsis_dap_backend_data));
	dap->bdata = bdata;
	if (!dap->bdata) {
		LOG_ERROR("unable to allocate memory");
		return ERROR_FAIL;
	}

	bdata->server_port = cmsis_dap_net_server_port;
	bdata->server_address = cmsis_dap_net_server_address;

	// bdata->sockfd = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);
	bdata->sockfd = socket(AF_INET, SOCK_STREAM, IPPROTO_IP);
	if (bdata->sockfd < 0) {
		LOG_ERROR("socket: %s", strerror(errno));
		return ERROR_FAIL;
	}

	memset(&bdata->serv_addr, 0, sizeof(bdata->serv_addr));

	bdata->serv_addr.sin_family = AF_INET;
	bdata->serv_addr.sin_port = htons(bdata->server_port);

	// Resolve hostname to IP address
	struct hostent *host = gethostbyname(bdata->server_address);
	if (host == NULL) {
		LOG_ERROR("gethostbyname error occurred for '%s'", bdata->server_address);
		return ERROR_FAIL;
	}
	struct in_addr **addr_list = (struct in_addr **)host->h_addr_list;
	if (addr_list[0] == NULL) {
		LOG_ERROR("unable to resolve hostname to IP address for '%s'",
				bdata->server_address);
		return ERROR_FAIL;
	}

	memcpy(&bdata->serv_addr.sin_addr, addr_list[0], sizeof(struct in_addr));

	if (bdata->serv_addr.sin_addr.s_addr == INADDR_NONE) {
		LOG_ERROR("inet_addr error occurred for '%s'", bdata->server_address);
		return ERROR_FAIL;
	}

	if (connect(bdata->sockfd, (struct sockaddr *)&bdata->serv_addr,
				sizeof(bdata->serv_addr)) < 0) {
		close(bdata->sockfd);
		LOG_ERROR("Can't connect to %s : %" PRIu16, bdata->server_address,
				bdata->server_port);
		return ERROR_FAIL;
	}

	// if (bdata->serv_addr.sin_addr.s_addr == htonl(INADDR_LOOPBACK)) {
	int flag = 1;
	setsockopt(bdata->sockfd, IPPROTO_TCP, TCP_NODELAY, (char *)&flag,
	           sizeof(int));
	// }

	char ip_addr_str[INET_ADDRSTRLEN];
	inet_ntop(AF_INET, &(bdata->serv_addr.sin_addr), ip_addr_str, INET_ADDRSTRLEN);

	LOG_INFO("Connection to %s (%s), port %" PRIu16 " succeed", bdata->server_address, 
			ip_addr_str, bdata->server_port);

	int retval = cmsis_dap_net_alloc(dap, 64);
	if (retval != ERROR_OK) {
		cmsis_dap_net_close(dap);
		return ERROR_FAIL;
	}

	dap->command = dap->packet_buffer;
	dap->response = dap->packet_buffer;

	return ERROR_OK;
}

static int cmsis_dap_net_read(struct cmsis_dap *dap, int transfer_timeout_ms,
							  struct timeval *wait_timeout) {
	// LOG_INFO("Reading data from device");

	// Set up read timeout on the socket
	struct timeval timeout;
	timeout.tv_sec = transfer_timeout_ms / 1000;
	timeout.tv_usec = (transfer_timeout_ms % 1000) * 1000;
	setsockopt(dap->bdata->sockfd, SOL_SOCKET, SO_RCVTIMEO,
				(const char *)&timeout, sizeof(timeout));

	int retval = read_socket(dap->bdata->sockfd, dap->packet_buffer,
							dap->packet_buffer_size);
	if (retval == 0) {
		return ERROR_TIMEOUT_REACHED;
	} else if (retval == -1) {
		int error = errno;
		LOG_ERROR("error reading data: %s (timeout %d)", strerror(error),
				transfer_timeout_ms);
		if (error == EAGAIN || error == EWOULDBLOCK)
			return ERROR_TIMEOUT_REACHED;
		else if (error == ECONNRESET)
			return ERROR_JTAG_DEVICE_ERROR;
		else
			return ERROR_FAIL;
	}

	// LOG_INFO("n: Read %d bytes", retval);
	return retval;
}

static int cmsis_dap_net_write(struct cmsis_dap *dap, int txlen,
							   int timeout_ms) {
	(void)timeout_ms;

	// LOG_INFO("Writing data to device: %d bytes", txlen);

	int retval = write_socket(dap->bdata->sockfd, dap->packet_buffer, txlen);
	if (retval == -1) {
		LOG_ERROR("error writing data: %s", strerror(errno));
		return ERROR_FAIL;
	}

	// LOG_INFO("n: Wrote %d bytes", retval);

	return retval;
}

static void cmsis_dap_net_cancel_all(struct cmsis_dap *dap) {}

/* CLI handlers */

COMMAND_HANDLER(cmsis_dap_handle_net_dap_server_address_command) {
	if (CMD_ARGC == 1) {
		if (cmsis_dap_net_server_address) {
		free(cmsis_dap_net_server_address);
		}
		cmsis_dap_net_server_address = strdup(CMD_ARGV[0]);
	} else
		LOG_ERROR("expected exactly one argument to cmsis_dap_usb_interface "
				"<interface_number>");

	return ERROR_OK;
}

COMMAND_HANDLER(cmsis_dap_handle_net_dap_server_port_command) {
	if (CMD_ARGC == 1)
		COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], cmsis_dap_net_server_port);
	else
		LOG_ERROR("expected exactly one argument to cmsis_dap_usb_interface "
				"<interface_number>");

	return ERROR_OK;
}

const struct command_registration cmsis_dap_net_subcommand_handlers[] = {
	{
		.name = "address",
		.handler = &cmsis_dap_handle_net_dap_server_address_command,
		.mode = COMMAND_CONFIG,
		.help =
			"set the DAP network server address to use (for NET backend only)",
		.usage = "<hostname_or_ip>",
	},
	{
		.name = "port",
		.handler = &cmsis_dap_handle_net_dap_server_port_command,
		.mode = COMMAND_CONFIG,
		.help = "set the DAP network server port to use (for NET backend only)",
		.usage = "<port>",
	},
	COMMAND_REGISTRATION_DONE
};


/* Backend registration */

const struct cmsis_dap_backend cmsis_dap_net_backend = {
	.name = "net",
	.open = cmsis_dap_net_open,
	.close = cmsis_dap_net_close,
	.read = cmsis_dap_net_read,
	.write = cmsis_dap_net_write,
	.packet_buffer_alloc = cmsis_dap_net_alloc,
	.packet_buffer_free = cmsis_dap_net_free,
	.cancel_all = cmsis_dap_net_cancel_all,
};
