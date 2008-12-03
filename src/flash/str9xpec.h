/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef STR9XPEC_H
#define STR9XPEC_H

#include "flash.h"
#include "target.h"
#include "jtag.h"

typedef struct str9xpec_flash_controller_s
{
	jtag_tap_t *tap;
	u32 *sector_bits;
	int chain_pos;
	int isc_enable;
	u8 options[8];
} str9xpec_flash_controller_t;

enum str9xpec_status_codes
{
	STR9XPEC_INVALID_COMMAND = 1,
	STR9XPEC_ISC_SUCCESS = 2,
	STR9XPEC_ISC_DISABLED = 3,
	STR9XPEC_ISC_INTFAIL = 32,
};

/* ISC commands */

#define ISC_IDCODE				0xFE
#define ISC_MFG_READ			0x4C
#define ISC_CONFIGURATION		0x07
#define ISC_ENABLE				0x0C
#define ISC_DISABLE				0x0F
#define ISC_NOOP				0x10
#define ISC_ADDRESS_SHIFT		0x11
#define ISC_CLR_STATUS			0x13
#define ISC_PROGRAM				0x20
#define ISC_PROGRAM_SECURITY	0x22
#define ISC_PROGRAM_UC			0x23
#define ISC_ERASE				0x30
#define ISC_READ				0x50
#define ISC_BLANK_CHECK			0x60

/* ISC_DEFAULT bit definitions */

#define ISC_STATUS_SECURITY		0x40
#define ISC_STATUS_INT_ERROR	0x30
#define ISC_STATUS_MODE			0x08
#define ISC_STATUS_BUSY			0x04
#define ISC_STATUS_ERROR		0x03

/* Option bytes definitions */

#define STR9XPEC_OPT_CSMAPBIT		48
#define STR9XPEC_OPT_LVDTHRESBIT	49
#define STR9XPEC_OPT_LVDSELBIT		50
#define STR9XPEC_OPT_LVDWARNBIT		51
#define STR9XPEC_OPT_OTPBIT			63

#endif /* STR9XPEC_H */
