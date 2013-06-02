/***************************************************************************
 *   Copyright (C) 2007 by Pavel Chromy                                    *
 *   chromy@asix.cz                                                        *
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

#ifndef BITQ_H
#define BITQ_H

#include <jtag/commands.h>

struct bitq_interface {
	/* function to enqueueing low level IO requests */
	int (*out)(int tms, int tdi, int tdo_req);
	int (*flush)(void);

	int (*sleep)(unsigned long us);
	int (*reset)(int trst, int srst);

	/* delayed read of requested TDO data,
	 * the input shall be checked after call to any enqueuing function
	 */
	int (*in_rdy)(void);
	int (*in)(void);
};

extern struct bitq_interface *bitq_interface;

int bitq_execute_queue(void);

void bitq_cleanup(void);

#endif /* BITQ_H */
