/***************************************************************************
 *   Copyright (C) 2016 by Matthias Welwarsky                              *
 *   matthias.welwarsky@sysgo.com                                          *
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
#ifndef OPENOCD_TARGET_ARMV8_CACHE_H_
#define OPENOCD_TARGET_ARMV8_CACHE_H_

#include "armv8.h"

extern int armv8_cache_d_inner_flush_virt(struct armv8_common *armv8, target_addr_t va, size_t size);
extern int armv8_cache_i_inner_inval_virt(struct armv8_common *armv8, target_addr_t va, size_t size);

#endif /* OPENOCD_TARGET_ARMV8_CACHE_H_ */
