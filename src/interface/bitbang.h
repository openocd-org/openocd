/*
 * Copyright (c) 2011-2012 Tomasz Boleslaw CEDRO
 * cederom@tlen.pl, http://www.tomek.cedro.info
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.

 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.

 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software Foundation,
 * Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
 */

/** @file Framework to work with interface signals, header file. */

#ifndef OOCD_INTERFACE_BITBANG_H
#define OOCD_INTERFACE_BITBANG_H

#define OOCD_BITBANG_PARAM_CMD_MAX_LEN 45
int oocd_interface_bitbang_register_commands(struct command_context *ctx);

#endif
