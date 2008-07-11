/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   part of this file is taken from libcli (libcli.sourceforge.net)       *
 *   Copyright (C) David Parrish (david@dparrish.com)                      *
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

#include "replacements.h"

#include "command.h"

#include "log.h"
#include "time_support.h"

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>

#include <openocd_tcl.h>

int fast_and_dangerous = 0;

int handle_sleep_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_time_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_fast_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

/* forward declaration of jim_command */
extern int jim_command(command_context_t *context, char *line);



command_t* register_command(command_context_t *context, command_t *parent, char *name, int (*handler)(struct command_context_s *context, char* name, char** args, int argc), enum command_mode mode, char *help)
{
	command_t *c, *p;
	
	if (!context || !name)
		return NULL;
				
	c = malloc(sizeof(command_t));
	
	c->name = strdup(name);
	c->parent = parent;
	c->children = NULL;
	c->handler = handler;
	c->mode = mode;
	if (!help)
		help="";
	c->next = NULL;
	
	/* place command in tree */
	if (parent)
	{
		if (parent->children)
		{
			/* find last child */
			for (p = parent->children; p && p->next; p = p->next);
			if (p)
				p->next = c;
		}
		else
		{
			parent->children = c;
		}
	}
	else
	{
		if (context->commands)
		{
			/* find last command */
			for (p = context->commands; p && p->next; p = p->next);
			if (p)
				p->next = c;
		}
		else
		{
			context->commands = c;
		}
	}
	/* accumulate help text in Tcl helptext list.  */
    Jim_Obj *helptext=Jim_GetGlobalVariableStr(interp, "ocd_helptext", JIM_ERRMSG);
	Jim_Obj *cmd_entry=Jim_NewListObj(interp, NULL, 0);
	
	Jim_Obj *cmd_list=Jim_NewListObj(interp, NULL, 0);

	/* maximum of two levels :-) */
	if (c->parent!=NULL)
	{
		Jim_ListAppendElement(interp, cmd_list, Jim_NewStringObj(interp, c->parent->name, -1));
	} 
	Jim_ListAppendElement(interp, cmd_list, Jim_NewStringObj(interp, c->name, -1));
	
	Jim_ListAppendElement(interp, cmd_entry, cmd_list);
	Jim_ListAppendElement(interp, cmd_entry, Jim_NewStringObj(interp, help, -1));
	Jim_ListAppendElement(interp, helptext, cmd_entry);
	return c;
}

int unregister_all_commands(command_context_t *context)
{
	command_t *c, *c2;
	
	if (context == NULL)
		return ERROR_OK;
	
	
	while(NULL != context->commands)
	{
		c = context->commands;
		
		while(NULL != c->children)
		{
			c2 = c->children;
			c->children = c->children->next;
			free(c2->name);
			c2->name = NULL;
			free(c2);
			c2 = NULL;
		}
		
		context->commands = context->commands->next;
		
		free(c->name);
		c->name = NULL;
		free(c);
		c = NULL;		
	}
	
	return ERROR_OK;
}

int unregister_command(command_context_t *context, char *name)
{
	command_t *c, *p = NULL, *c2;
	
	if ((!context) || (!name))
		return ERROR_INVALID_ARGUMENTS;
	
	/* find command */
	for (c = context->commands; c; c = c->next)
	{
		if (strcmp(name, c->name) == 0)
		{
			/* unlink command */
			if (p)
			{
				p->next = c->next;
			}
			else
			{
				context->commands = c->next;
			}
			
			/* unregister children */
			if (c->children)
			{
				for (c2 = c->children; c2; c2 = c2->next)
				{
					free(c2->name);
					free(c2);
				}
			}
			
			/* delete command */
			free(c->name);
			free(c);
		}
		
		/* remember the last command for unlinking */
		p = c;
	}
	
	return ERROR_OK;
}

int parse_line(char *line, char *words[], int max_words)
{
	int nwords = 0;
	char *p = line;
	char *word_start = line;
	int inquote = 0;

	while (nwords < max_words - 1)
	{
		/* check if we reached
		 * a terminating NUL
		 * a matching closing quote character " or '
		 * we're inside a word but not a quote, and the current character is whitespace
		 */
		if (!*p || *p == inquote || (word_start && !inquote && isspace(*p)))
		{
			/* we're inside a word or quote, and reached its end*/
			if (word_start)
			{
				int len;
				char *word_end=p;
				
				/* This will handle extra whitespace within quotes */
				while (isspace(*word_start)&&(word_start<word_end))
					word_start++;
				while (isspace(*(word_end-1))&&(word_start<word_end))
					word_end--;
				len = word_end - word_start;
				
				if (len>0)
				{
					/* copy the word */
					memcpy(words[nwords] = malloc(len + 1), word_start, len);
					/* add terminating NUL */
					words[nwords++][len] = 0;
				}
			}
			/* we're done parsing the line */
			if (!*p)
				break;

			/* skip over trailing quote or whitespace*/
			if (inquote || isspace(*p))
				p++;
			while (isspace(*p))
				p++;
			
			inquote = 0;
			word_start = 0;
		}
		else if (*p == '"' || *p == '\'')
		{
			/* we've reached the beginning of a quote */
			inquote = *p++;
			word_start = p;
		}
		else
		{
			/* we've reached the beginning of a new word */
			if (!word_start)
				word_start = p;
			
			/* normal character, skip */
			p++;
		}
	}
	
	return nwords;
}

void command_output_text(command_context_t *context, const char *data)
{
	if( context && context->output_handler && data  ){
		context->output_handler( context, data );
	}
}

void command_print_n(command_context_t *context, char *format, ...)
{
	char *string;
	
	va_list ap;
	va_start(ap, format);

	string = alloc_vprintf(format, ap);
	if (string != NULL)
	{
		context->output_handler(context, string);
		free(string);
	}

	va_end(ap);
}

void command_print(command_context_t *context, char *format, ...)
{
	char *string;

	va_list ap;
	va_start(ap, format);

	string = alloc_vprintf(format, ap);
	if (string != NULL)
	{
		strcat(string, "\n"); /* alloc_vprintf guaranteed the buffer to be at least one char longer */
		context->output_handler(context, string);
		free(string);
	}

	va_end(ap);
}

command_t *find_command(command_context_t *context, command_t *commands, char *words[], int num_words, int start_word, int *new_start_word)
{
	command_t *c;
	
	for (c = commands; c; c = c->next)
	{
		if (strcasecmp(c->name, words[start_word]))
			continue;

		if ((context->mode == COMMAND_CONFIG) || (c->mode == COMMAND_ANY) || (c->mode == context->mode) )
		{
			if (!c->children)
			{
				if (!c->handler)
				{
					return NULL;
				}
				else
				{
					*new_start_word=start_word;
					return c;
				}
			}
			else
			{
				if (start_word == num_words - 1)
				{
					return NULL;
				}
				return find_command(context, c->children, words, num_words, start_word + 1, new_start_word);
			}
		}
	}
	return NULL;
}

int find_and_run_command(command_context_t *context, command_t *commands, char *words[], int num_words)
{
	int start_word=0;
	command_t *c;
	c = find_command(context, commands, words, num_words, start_word, &start_word);
	if (c == NULL)
	{
		/* just return command not found */
		return ERROR_COMMAND_NOTFOUND;
	}
	
	int retval = c->handler(context, c->name, words + start_word + 1, num_words - start_word - 1);
	if (retval == ERROR_COMMAND_SYNTAX_ERROR)
	{
		
	}
	else if (retval == ERROR_COMMAND_CLOSE_CONNECTION)
	{
		/* just fall through for a shutdown request */
	}
	else if (retval != ERROR_OK)
	{
		/* we do not print out an error message because the command *should*
		 * have printed out an error
		 */
		LOG_DEBUG("Command failed with error code %d", retval); 
	}
	
	return retval; 
}

int command_run_line_internal(command_context_t *context, char *line)
{
	LOG_USER_N("%s", ""); /* Keep GDB connection alive*/ 
	
	int nwords;
	char *words[128] = {0};
	int retval;
	int i;

	/* skip preceding whitespace */
	while (isspace(*line))
		line++;
	
	/* empty line, ignore */
	if (!*line)
		return ERROR_OK;
	
	/* ignore comments */
	if (*line && (line[0] == '#'))
		return ERROR_OK;
	
	LOG_DEBUG("%s", line);

	nwords = parse_line(line, words, sizeof(words) / sizeof(words[0]));
	
	if (nwords > 0)
	{
		retval = find_and_run_command(context, context->commands, words, nwords);
	}
	else
		return ERROR_INVALID_ARGUMENTS;
	
	for (i = 0; i < nwords; i++)
		free(words[i]);
	
	return retval;
}

int command_run_line(command_context_t *context, char *line)
{
	/* if a command is unknown to the "unknown" proc in tcl/commands.tcl will
	 * redirect it to OpenOCD.
	 * 
	 * This avoids having to type the "openocd" prefix and makes OpenOCD
	 * commands "native" to Tcl.
	 */
	return jim_command(context, line);
}


int command_run_linef(command_context_t *context, char *format, ...)
{
	int retval=ERROR_FAIL;
	char *string;
	va_list ap;
	va_start(ap, format);
	string = alloc_vprintf(format, ap);
	if (string!=NULL)
	{
		retval=command_run_line(context, string);
	}
	va_end(ap);
	return retval;
}



void command_set_output_handler(command_context_t* context, int (*output_handler)(struct command_context_s *context, const char* line), void *priv)
{
	context->output_handler = output_handler;
	context->output_handler_priv = priv;
}

command_context_t* copy_command_context(command_context_t* context)
{
	command_context_t* copy_context = malloc(sizeof(command_context_t));

	*copy_context = *context;
	
	return copy_context;
}

int command_done(command_context_t *context)
{
	free(context);
	context = NULL;
	
	return ERROR_OK;
}

command_context_t* command_init()
{
	command_context_t* context = malloc(sizeof(command_context_t));
	
	context->mode = COMMAND_EXEC;
	context->commands = NULL;
	context->current_target = 0;
	context->output_handler = NULL;
	context->output_handler_priv = NULL;
	
	register_command(context, NULL, "sleep", handle_sleep_command,
					 COMMAND_ANY, "sleep for <n> milliseconds");
	
	register_command(context, NULL, "time", handle_time_command,
					 COMMAND_ANY, "time <cmd + args> - execute <cmd + args> and print time it took");
	
	register_command(context, NULL, "fast", handle_fast_command,
					 COMMAND_ANY, "fast <enable/disable> - place at beginning of config files. Sets defaults to fast and dangerous.");
	
	return context;
}

/* sleep command sleeps for <n> miliseconds
 * this is useful in target startup scripts
 */
int handle_sleep_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	unsigned long duration = 0;
	
	if (argc == 1)
	{
		duration = strtoul(args[0], NULL, 0);
		usleep(duration * 1000);
	}

	return ERROR_OK;
}

int handle_fast_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc!=1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	
	fast_and_dangerous = strcmp("enable", args[0])==0;
	
	return ERROR_OK;
}


int handle_time_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	duration_t duration;
	char *duration_text;
	int retval;
	float t;
	
	if (argc<1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	
	duration_start_measure(&duration);
	
	retval = find_and_run_command(cmd_ctx, cmd_ctx->commands, args, argc);
	if (retval == ERROR_COMMAND_NOTFOUND)
	{
		command_print(cmd_ctx, "Command %s not found", args[0]);
	}
	
	duration_stop_measure(&duration, &duration_text);
	
	t=duration.duration.tv_sec;
	t+=((float)duration.duration.tv_usec / 1000000.0);
	command_print(cmd_ctx, "%s took %fs", args[0], t);
	
	free(duration_text);

	return retval;
}
