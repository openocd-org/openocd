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

#ifndef OPENOCD_HELPER_JEP106_H
#define OPENOCD_HELPER_JEP106_H

/**
 * Get the manufacturer name associated with a JEP106 ID.
 * @param bank The bank (number of continuation codes) of the manufacturer ID.
 * @param id The 7-bit manufacturer ID (i.e. with parity stripped).
 * @return A pointer to static const storage containing the name of the
 *         manufacturer associated with bank and id, or one of the strings
 *         "<invalid>" and "<unknown>".
 */
const char *jep106_manufacturer(unsigned bank, unsigned id);

#endif /* OPENOCD_HELPER_JEP106_H */
