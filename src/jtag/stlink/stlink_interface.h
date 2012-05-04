/***************************************************************************
 *   Copyright (C) 2011 by Mathias Kuester                                 *
 *   Mathias Kuester <kesmtp@freenet.de>                                   *
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/

#ifndef _STLINK_INTERFACE_
#define _STLINK_INTERFACE_

/** */
struct target;
/** */
enum e_stlink_transports;
/** */
extern const char *stlink_transports[];

struct stlink_interface_param_s {
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
	enum stlink_transports transport;
};

struct stlink_interface_s {
	/** */
	struct stlink_interface_param_s param;
	/** */
	const struct stlink_layout *layout;
	/** */
	void *fd;
};

/** */
int stlink_interface_open(enum stlink_transports tr);
/** */
int stlink_interface_init_target(struct target *t);

#endif
