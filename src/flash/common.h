/***************************************************************************
 *   Copyright (C) 2009 by Zachary T Welch <zw@superlucidity.net>          *
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
#ifndef FLASH_COMMON_H
#define FLASH_COMMON_H

#include <helper/types.h>

/**
 * Parses the optional '.index' portion of a flash bank identifier.
 * @param name The desired driver name, passed by the user.
 * @returns The parsed index request, or 0 if not present.  If the
 * name provides a suffix but it does not parse as an unsigned integer, 
 * the routine returns ~0U.  This will prevent further matching.
 */ 
unsigned get_flash_name_index(const char *name);
/**
 * Attempt to match the @c expected name with the @c name of a driver.
 * @param name The name of the driver (from the bank's device structure).
 * @param expected The expected driver name, passed by the user.
 */
bool flash_driver_name_matches(const char *name, const char *expected);

#endif // FLASH_COMMON_H
