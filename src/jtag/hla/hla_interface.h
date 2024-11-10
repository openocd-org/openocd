/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2011 by Mathias Kuester                                 *
 *   Mathias Kuester <kesmtp@freenet.de>                                   *
 *                                                                         *
 *   Copyright (C) 2012 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 ***************************************************************************/

#ifndef OPENOCD_JTAG_HLA_HLA_INTERFACE_H
#define OPENOCD_JTAG_HLA_HLA_INTERFACE_H

/** */
struct target;
/** */
enum e_hl_transports;
/** */
extern const char *hl_transports[];

#define HLA_MAX_USB_IDS 16

struct hl_interface_param {
	/** */
	const char *device_desc;
	/** List of recognised VIDs */
	uint16_t vid[HLA_MAX_USB_IDS + 1];
	/** List of recognised PIDs */
	uint16_t pid[HLA_MAX_USB_IDS + 1];
	/** */
	enum hl_transports transport;
	/** */
	bool connect_under_reset;
	/** */
	bool use_stlink_tcp;
	/** */
	uint16_t stlink_tcp_port;
};

struct hl_interface {
	/** */
	struct hl_interface_param param;
	/** */
	const struct hl_layout *layout;
	/** */
	void *handle;
};

/** */
int hl_interface_open(enum hl_transports tr);
/** */

int hl_interface_init_target(struct target *t);
int hl_interface_init_reset(void);
int hl_interface_override_target(const char **targetname);

#endif /* OPENOCD_JTAG_HLA_HLA_INTERFACE_H */
