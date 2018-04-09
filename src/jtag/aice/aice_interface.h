/***************************************************************************
 *   Copyright (C) 2013 by Andes Technology                                *
 *   Hsiangkai Wang <hkwang@andestech.com>                                 *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifndef OPENOCD_JTAG_AICE_AICE_INTERFACE_H
#define OPENOCD_JTAG_AICE_AICE_INTERFACE_H

struct aice_interface_param_s {
	/** */
	const char *device_desc;
	/** */
	const char *serial;
	/** */
	uint16_t vid;
	/** */
	uint16_t pid;
};

int aice_init_targets(void);
int aice_scan_jtag_chain(void);

#endif /* OPENOCD_JTAG_AICE_AICE_INTERFACE_H */
