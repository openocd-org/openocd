/***************************************************************************
 *   Copyright (C) 2009 Zachary T Welch <zw@superlucidity.net>             *
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

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif
#include <flash/flash.h>
#include <flash/nor/imp.h>

// in flash.c, to be moved here
extern struct flash_driver *flash_drivers[];
extern struct flash_bank *flash_banks;

struct flash_driver *flash_driver_find_by_name(const char *name)
{
	for (unsigned i = 0; flash_drivers[i]; i++)
	{
		if (strcmp(name, flash_drivers[i]->name) == 0)
			return flash_drivers[i];
	}
	return NULL;
}

void flash_bank_add(struct flash_bank *bank)
{
	/* put flash bank in linked list */
	unsigned bank_num = 0;
	if (flash_banks)
	{
		/* find last flash bank */
		struct flash_bank *p = flash_banks;
		while (NULL != p->next)
		{
			bank_num += 1;
			p = p->next;
		}
		p->next = bank;
		bank_num += 1;
	}
	else
		flash_banks = bank;

	bank->bank_number = bank_num;
}

struct flash_bank *flash_bank_list(void)
{
	return flash_banks;
}
