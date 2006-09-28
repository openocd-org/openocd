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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "interpreter.h"

#include "binarybuffer.h"
#include <stdlib.h>
#include <string.h>

var_t *variables = NULL;

int handle_var_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_field_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_script_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

int interpreter_register_commands(struct command_context_s *cmd_ctx)
{
	register_command(cmd_ctx, NULL, "var", handle_var_command,
		COMMAND_ANY, "allocate, display or delete variable <name> [num_fields|'del'] [size1] ...");
	register_command(cmd_ctx, NULL, "field", handle_field_command,
		COMMAND_ANY, "display/modify variable field <var> <field> [value|'flip']");
	register_command(cmd_ctx, NULL, "script", handle_script_command,
		COMMAND_ANY, "execute commands from <file>");

	return ERROR_OK;
}

var_t* get_var_by_num(int num)
{
	int count = 0;
	var_t *var = variables;

	if (var)	
	{
		if (num == count)
			return var;
		while (var->next)
		{
			var = var->next;
			count++;
			if (num == count)
				return var;
		}
	}
	return NULL;
}

var_t* get_var_by_name(char *name)
{
	var_t *var = variables;

	if (var)	
	{
		if (strcmp(var->name, name) == 0)
			return var;
		while (var->next)
		{
			var = var->next;
			if (strcmp(var->name, name) == 0)
				return var;
		}
	}
	return NULL;
}

var_t* get_var_by_namenum(char *namenum)
{
	if ((namenum[0] >= '0') && (namenum[0] <= '9'))
		return get_var_by_num(strtol(namenum, NULL, 0));
	else
		return get_var_by_name(namenum);
	
}

int field_le_to_host(u8 *buffer, void *priv)
{
	var_field_t *field = priv;
	field->value = buf_get_u32(buffer, 0, field->num_bits);

	return ERROR_OK;
}

int handle_var_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	var_t **last_var_p = &variables;
	int i;

	if (argc >= 2)
	{
		while (*last_var_p)
		{
			if (strcmp((*last_var_p)->name, args[0]) == 0)
			{
				if (strcmp(args[1], "del") == 0)
				{
					var_t *next = (*last_var_p)->next;
					free ((*last_var_p)->fields);
					free (*last_var_p);
					*last_var_p = next;
					command_print(cmd_ctx, "variable %s deleted", args[0]);
				}
				else
					command_print(cmd_ctx, "variable of that name already exists");
				return ERROR_OK;
			}
			last_var_p = &((*last_var_p)->next);
		}

		if ((args[0][0] >= '0') && (args[0][0] <= '9'))
		{
			command_print(cmd_ctx, "invalid name specified (first character may not be a number)");
			return ERROR_OK;
		}

		*last_var_p = malloc(sizeof(var_t));
		(*last_var_p)->name = strdup(args[0]);
		(*last_var_p)->num_fields = argc - 1;
		(*last_var_p)->next = NULL;

		(*last_var_p)->fields = malloc(sizeof(var_field_t) * (*last_var_p)->num_fields);
		for (i = 0; i < (*last_var_p)->num_fields; i++)
		{
			(*last_var_p)->fields[i].num_bits = strtol(args[1+i], NULL, 0);
			(*last_var_p)->fields[i].value = 0x0;
		}
		return ERROR_OK;
	}

	if (argc == 1)
	{
		var_t *var = get_var_by_namenum(args[0]);
		if (var)
		{
			int i;
			command_print(cmd_ctx, "%s (%i fields):", var->name, var->num_fields);
			for (i = 0; i < (var->num_fields); i++)
			{
				command_print(cmd_ctx, "0x%x (/%i)", var->fields[i].value, var->fields[i].num_bits);
			}
		}
		else
		{
			command_print(cmd_ctx, "variable %s doesn't exist", args[0]);
		}
	}

	if (argc == 0)
	{
		var_t *var = variables;
		int count = 0;
		while (var)
		{
			command_print(cmd_ctx, "%i: %s (%i fields)", count, var->name, var->num_fields);
			var = var->next;
			count++;
		}
	}

	return ERROR_OK;
}

int handle_field_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{

	if (argc < 2)
		command_print(cmd_ctx, "usage: field <var> <field> [value|'flip']");

	if (argc >= 2)
	{
		var_t *var = get_var_by_namenum(args[0]);
		int field_num = strtol(args[1], NULL, 0);
		if (!var)
		{
			command_print(cmd_ctx, "variable %s doesn't exist", args[0]);
			return ERROR_OK;
		}
		if (field_num >= var->num_fields)
			command_print(cmd_ctx, "variable field %i is out of bounds (max. %i)", field_num, var->num_fields - 1);
		if ((var) && (field_num < var->num_fields))
		{
			if (argc > 2)
			{
				if (strcmp(args[2], "flip") == 0)
					var->fields[field_num].value = flip_u32(var->fields[field_num].value, var->fields[field_num].num_bits);
				else
					var->fields[field_num].value = strtoul(args[2], NULL, 0);
			}

			command_print(cmd_ctx, "%s(%i): 0x%x (/%i)", var->name, field_num, var->fields[field_num].value, var->fields[field_num].num_bits);
		}
	}

	return ERROR_OK;
}

int handle_script_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	FILE *script_file;
	int echo;

	if (argc != 1)
		command_print(cmd_ctx, "usage: script <file>");

	script_file = fopen(args[0], "r");
	if (!script_file)
	{
		command_print(cmd_ctx, "couldn't open script file %s", args[0]);
		return ERROR_OK;
	}

	echo = cmd_ctx->echo;
	cmd_ctx->echo = 1;
	
	command_run_file(cmd_ctx, script_file, COMMAND_EXEC);
	
	cmd_ctx->echo = echo;
	
	fclose(script_file);

	return ERROR_OK;
}
