/***************************************************************************
 *   Copyright (C) 2015 Andreas Fritiofson                                 *
 *   andreas.fritiofson@gmail.com                                          *
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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "jep106.h"
#include "log.h"

static const char * const jep106[][126] = {
#include "jep106.inc"
};

const char *jep106_table_manufacturer(unsigned int bank, unsigned int id)
{
	if (id < 1 || id > 126) {
		LOG_DEBUG("BUG: Caller passed out-of-range JEP106 ID!");
		return "<invalid>";
	}

	/* index is zero based */
	id--;

	if (bank >= ARRAY_SIZE(jep106) || jep106[bank][id] == 0)
		return "<unknown>";

	return jep106[bank][id];
}
