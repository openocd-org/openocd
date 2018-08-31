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
#include <ctype.h>
#ifdef _WIN32
#include <winsock2.h>
#else
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#endif

#include "libjaylink.h"
#include "libjaylink-internal.h"

/**
 * @file
 *
 * Device discovery (TCP/IP).
 */

/** @cond PRIVATE */
/** Size of the advertisement message in bytes. */
#define ADV_MESSAGE_SIZE	128

/** Device discovery port number. */
#define DISC_PORT		19020

/** Size of the discovery message in bytes. */
#define DISC_MESSAGE_SIZE	64

/** Discovery timeout in milliseconds. */
#define DISC_TIMEOUT		20
/** @endcond */

static bool compare_devices(const void *a, const void *b)
{
	const struct jaylink_device *dev;
	const struct jaylink_device *new_dev;

	dev = a;
	new_dev = b;

	if (dev->iface != JAYLINK_HIF_TCP)
		return false;

	if (memcmp(dev->ipv4_address, new_dev->ipv4_address,
			sizeof(dev->ipv4_address)) != 0)
		return false;

	if (dev->serial_number != new_dev->serial_number)
		return false;

	if (memcmp(dev->mac_address, new_dev->mac_address,
			sizeof(dev->mac_address)) != 0)
		return false;

	if (strcmp(dev->product_name, new_dev->product_name) != 0)
		return false;

	if (strcmp(dev->nickname, new_dev->nickname) != 0)
		return false;

	if (dev->hw_version.type != new_dev->hw_version.type)
		return false;

	if (dev->hw_version.major != new_dev->hw_version.major)
		return false;

	if (dev->hw_version.minor != new_dev->hw_version.minor)
		return false;

	if (dev->hw_version.revision != new_dev->hw_version.revision)
		return false;

	return true;
}

static struct jaylink_device *find_device(struct list *list,
		const struct jaylink_device *dev)
{
	struct list *item;

	item = list_find_custom(list, &compare_devices, dev);

	if (item)
		return item->data;

	return NULL;
}

static bool parse_adv_message(struct jaylink_device *dev,
		const uint8_t *buffer)
{
	struct in_addr in;
	uint32_t tmp;

	if (memcmp(buffer, "Found", 5) != 0)
		return false;

	/*
	 * Use inet_ntoa() instead of inet_ntop() because the latter requires
	 * at least Windows Vista.
	 */
	memcpy(&in, buffer + 16, 4);
	memcpy(dev->ipv4_address, inet_ntoa(in), sizeof(dev->ipv4_address));

	memcpy(dev->mac_address, buffer + 32, sizeof(dev->mac_address));
	dev->has_mac_address = true;

	dev->serial_number = buffer_get_u32(buffer, 48);
	dev->valid_serial_number = true;

	tmp = buffer_get_u32(buffer, 52);
	dev->hw_version.type = (tmp / 1000000) % 100;
	dev->hw_version.major = (tmp / 10000) % 100;
	dev->hw_version.minor = (tmp / 100) % 100;
	dev->hw_version.revision = tmp % 100;
	dev->has_hw_version = true;

	memcpy(dev->product_name, buffer + 64, sizeof(dev->product_name));
	dev->product_name[JAYLINK_PRODUCT_NAME_MAX_LENGTH - 1] = '\0';
	dev->has_product_name = isprint((unsigned char)dev->product_name[0]);

	memcpy(dev->nickname, buffer + 96, sizeof(dev->nickname));
	dev->nickname[JAYLINK_NICKNAME_MAX_LENGTH - 1] = '\0';
	dev->has_nickname = isprint((unsigned char)dev->nickname[0]);

	return true;
}

static struct jaylink_device *probe_device(struct jaylink_context *ctx,
		struct sockaddr_in *addr, const uint8_t *buffer)
{
	struct jaylink_device tmp;
	struct jaylink_device *dev;

	/*
	 * Use inet_ntoa() instead of inet_ntop() because the latter requires
	 * at least Windows Vista.
	 */
	log_dbg(ctx, "Received advertisement message (IPv4 address = %s).",
		inet_ntoa(addr->sin_addr));

	if (!parse_adv_message(&tmp, buffer)) {
		log_dbg(ctx, "Received invalid advertisement message.");
		return NULL;
	}

	log_dbg(ctx, "Found device (IPv4 address = %s).", tmp.ipv4_address);
	log_dbg(ctx, "Device: MAC address = %02x:%02x:%02x:%02x:%02x:%02x.",
		tmp.mac_address[0], tmp.mac_address[1], tmp.mac_address[2],
		tmp.mac_address[3], tmp.mac_address[4], tmp.mac_address[5]);
	log_dbg(ctx, "Device: Serial number = %u.", tmp.serial_number);

	if (tmp.has_product_name)
		log_dbg(ctx, "Device: Product = %s.", tmp.product_name);

	if (tmp.has_nickname)
		log_dbg(ctx, "Device: Nickname = %s.", tmp.nickname);

	dev = find_device(ctx->discovered_devs, &tmp);

	if (dev) {
		log_dbg(ctx, "Ignoring already discovered device.");
		return NULL;
	}

	dev = find_device(ctx->devs, &tmp);

	if (dev) {
		log_dbg(ctx, "Using existing device instance.");
		return jaylink_ref_device(dev);
	}

	log_dbg(ctx, "Allocating new device instance.");

	dev = device_allocate(ctx);

	if (!dev) {
		log_warn(ctx, "Device instance malloc failed.");
		return NULL;
	}

	dev->iface = JAYLINK_HIF_TCP;

	dev->serial_number = tmp.serial_number;
	dev->valid_serial_number = tmp.valid_serial_number;

	memcpy(dev->ipv4_address, tmp.ipv4_address, sizeof(dev->ipv4_address));

	memcpy(dev->mac_address, tmp.mac_address, sizeof(dev->mac_address));
	dev->has_mac_address = tmp.has_mac_address;

	memcpy(dev->product_name, tmp.product_name, sizeof(dev->product_name));
	dev->has_product_name = tmp.has_product_name;

	memcpy(dev->nickname, tmp.nickname, sizeof(dev->nickname));
	dev->has_nickname = tmp.has_nickname;

	dev->hw_version = tmp.hw_version;
	dev->has_hw_version = tmp.has_hw_version;

	return dev;
}

/** @private */
JAYLINK_PRIV int discovery_tcp_scan(struct jaylink_context *ctx)
{
	int ret;
	int sock;
	int opt_value;
	fd_set rfds;
	struct sockaddr_in addr;
	size_t addr_length;
	struct timeval timeout;
	uint8_t buf[ADV_MESSAGE_SIZE];
	struct jaylink_device *dev;
	size_t length;
	size_t num_devs;

	sock = socket(AF_INET, SOCK_DGRAM, IPPROTO_UDP);

	if (sock < 0) {
		log_err(ctx, "Failed to create discovery socket.");
		return JAYLINK_ERR;
	}

	opt_value = true;

	if (!socket_set_option(sock, SOL_SOCKET, SO_BROADCAST, &opt_value,
			sizeof(opt_value))) {
		log_err(ctx, "Failed to enable broadcast option for discovery "
			"socket.");
		socket_close(sock);
		return JAYLINK_ERR;
	}

	memset(&addr, 0, sizeof(struct sockaddr_in));
	addr.sin_family = AF_INET;
	addr.sin_port = htons(DISC_PORT);
	addr.sin_addr.s_addr = INADDR_ANY;

	if (!socket_bind(sock, (struct sockaddr *)&addr,
			sizeof(struct sockaddr_in))) {
		log_err(ctx, "Failed to bind discovery socket.");
		socket_close(sock);
		return JAYLINK_ERR;
	}

	addr.sin_family = AF_INET;
	addr.sin_port = htons(DISC_PORT);
	addr.sin_addr.s_addr = INADDR_BROADCAST;

	memset(buf, 0, DISC_MESSAGE_SIZE);
	memcpy(buf, "Discover", 8);

	log_dbg(ctx, "Sending discovery message.");

	length = DISC_MESSAGE_SIZE;

	if (!socket_sendto(sock, (char *)buf, &length, 0,
			(const struct sockaddr *)&addr, sizeof(addr))) {
		log_err(ctx, "Failed to send discovery message.");
		socket_close(sock);
		return JAYLINK_ERR_IO;
	}

	if (length < DISC_MESSAGE_SIZE) {
		log_err(ctx, "Only sent %zu bytes of discovery message.",
			length);
		socket_close(sock);
		return JAYLINK_ERR_IO;
	}

	timeout.tv_sec = DISC_TIMEOUT / 1000;
	timeout.tv_usec = (DISC_TIMEOUT % 1000) * 1000;

	num_devs = 0;

	while (true) {
		FD_ZERO(&rfds);
		FD_SET(sock, &rfds);

		ret = select(sock + 1, &rfds, NULL, NULL, &timeout);

		if (ret <= 0)
			break;

		if (!FD_ISSET(sock, &rfds))
			continue;

		length = ADV_MESSAGE_SIZE;
		addr_length = sizeof(struct sockaddr_in);

		if (!socket_recvfrom(sock, buf, &length, 0,
				(struct sockaddr *)&addr, &addr_length)) {
			log_warn(ctx, "Failed to receive advertisement "
				"message.");
			continue;
		}

		/*
		 * Filter out messages with an invalid size. This includes the
		 * broadcast message we sent before.
		 */
		if (length != ADV_MESSAGE_SIZE)
			continue;

		dev = probe_device(ctx, &addr, buf);

		if (dev) {
			ctx->discovered_devs = list_prepend(
				ctx->discovered_devs, dev);
			num_devs++;
		}
	}

	socket_close(sock);

	if (ret < 0) {
		log_err(ctx, "select() failed.");
		return JAYLINK_ERR;
	}

	log_dbg(ctx, "Found %zu TCP/IP device(s).", num_devs);

	return JAYLINK_OK;
}
