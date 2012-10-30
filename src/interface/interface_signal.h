/*
 * Copyright (C) 2011-2012 Tomasz Boleslaw CEDRO
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

#ifndef OOCD_INTERFACE_SIGNAL_H
#define OOCD_INTERFACE_SIGNAL_H

/** Some generic interface signal definitions. */

/** Interface Signal type declaration (single linked list element). */
typedef struct oocd_interface_signal {
	/** Signal name string. */
	char *name;
	/** Mask value for selected signal. */
	unsigned int mask;
	/** Value can be 0,1, or -1 for unknown state. */
	int value;
	/** Pointer to the next element on the signals list. */
	struct oocd_interface_signal *next;
} oocd_interface_signal_t;

typedef enum oocd_interface_operation {
	UNDEFINED = 0,
	READ,
	WRITE,
	SET,
	CLEAR
} oocd_interface_operation_t;

int oocd_interface_signal_add(char *name, unsigned int mask);
int oocd_interface_signal_del(char *name);
oocd_interface_signal_t *oocd_interface_signal_find(char *name);

#endif
