/***************************************************************************
 *   Copyright (C) 2011 by Mathias Kuester                                 *
 *   Mathias Kuester <kesmtp@freenet.de>                                   *
 *                                                                         *
 *   Copyright (C) 2012 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifndef _HL_INTERFACE
#define _HL_INTERFACE

/** */
struct target;
/** */
enum e_hl_transports;
/** */
extern const char *hl_transports[];

struct hl_interface_param_s {
	/** */
	char *device_desc;
	/** */
	char *serial;
	/** */
	uint16_t vid;
	/** */
	uint16_t pid;
	/** */
	unsigned api;
	/** */
	enum hl_transports transport;
	/** */
	bool connect_under_reset;
	/** Output file for trace data (if any) */
	FILE *trace_f;
	/** Trace module source clock rate */
	uint32_t trace_source_hz;
};

struct hl_interface_s {
	/** */
	struct hl_interface_param_s param;
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

#endif /* _HL_INTERFACE */
