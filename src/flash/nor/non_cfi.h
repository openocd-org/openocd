/***************************************************************************
 *   Copyright (C) 2007 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
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

#ifndef NON_CFI_H
#define NON_CFI_H

struct non_cfi {
	uint16_t mfr;
	uint16_t id;
	uint16_t pri_id;
	uint32_t dev_size;
	uint16_t interface_desc;
	uint16_t max_buf_write_size;
	uint8_t num_erase_regions;
	uint32_t erase_region_info[6];
	uint8_t  status_poll_mask;
};

void cfi_fixup_non_cfi(struct flash_bank *bank);

#endif /* NON_CFI_H */
