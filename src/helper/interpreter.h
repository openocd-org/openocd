/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
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
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifndef INTERPRETER_H
#define INTERPRETER_H

#include "types.h"
#include "command.h"
#include "log.h"

typedef struct var_field_s
{
	int num_bits;
	u32 value;
} var_field_t;

typedef struct var_s
{
	char *name;
	int num_fields;
	var_field_t *fields;
	struct var_s *next;
} var_t;

extern var_t *variables;

extern int field_le_to_host(u8 *buffer, void *priv);

extern var_t* get_var_by_namenum(char *namenum);
extern int interpreter_register_commands(struct command_context_s *cmd_ctx);

#endif /* INTERPRETER_H */
