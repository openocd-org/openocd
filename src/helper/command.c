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

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>

int handle_sleep_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

int build_unique_lengths(command_context_t *context, command_t *commands)
{
	command_t *c, *p;

	/* iterate through all commands */
	for (c = commands; c; c = c->next)
	{
		/* find out how many characters are required to uniquely identify a command */
		for (c->unique_len = 1; c->unique_len <= strlen(c->name); c->unique_len++)
		{
			int foundmatch = 0;
			
			/* for every command, see if the current length is enough */
			for (p = commands; p; p = p->next)
			{
				/* ignore the command itself */
				if (c == p)
					continue;
				
				/* compare commands up to the current length */
				if (strncmp(p->name, c->name, c->unique_len) == 0)
					foundmatch++;
			}
			
			/* when none of the commands matched, we've found the minimum length required */
			if (!foundmatch)
				break;
		}
		
		/* if the current command has children, build the unique lengths for them */
		if (c->children)
			build_unique_lengths(context, c->children);
	}
	
	return ERROR_OK;
}

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
	if (help)
		c->help = strdup(help);
	else
		c->help = NULL;
	c->unique_len = 0;
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
	
	/* update unique lengths */
	build_unique_lengths(context, (parent) ? parent : context->commands);
	
	return c;
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
					if (c2->help)
						free(c2->help);
					free(c2);
				}
			}
			
			/* delete command */
			free(c->name);
			if (c->help)
				free(c->help);
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

void command_print(command_context_t *context, char *format, ...)
{
	va_list ap;
	char *buffer = NULL;
	int n, size = 0;
	char *p;

	va_start(ap, format);
	
	/* process format string */
	/* TODO: possible bug. va_list is undefined after the first call to vsnprintf */
	while (!buffer || (n = vsnprintf(buffer, size, format, ap)) >= size)
	{
		/* increase buffer until it fits the whole string */
		if (!(p = realloc(buffer, size += 4096)))
			return;

		buffer = p;
	}
	
	/* vsnprintf failed */
	if (n < 0)
		return;

	p = buffer;
	
	/* process lines in buffer */
	do {
		char *next = strchr(p, '\n');
		
		if (next)
			*next++ = 0;

		if (context->output_handler)
			context->output_handler(context, p);

		p = next;
	} while (p);
	
	if (buffer)
		free(buffer);
	
	va_end(ap);
}

int find_and_run_command(command_context_t *context, command_t *commands, char *words[], int num_words, int start_word)
{
	command_t *c;
	
	for (c = commands; c; c = c->next)
	{
		if (strncasecmp(c->name, words[start_word], c->unique_len))
			continue;

		if (strncasecmp(c->name, words[start_word], strlen(words[start_word])))
			continue;
		
		if ((c->mode == context->mode) || (c->mode == COMMAND_ANY))
		{
			if (!c->children)
			{
				if (!c->handler)
				{
					command_print(context, "No handler for command");
					break;
				}
				else
				{
					return c->handler(context, c->name, words + start_word + 1, num_words - start_word - 1);
				}
			}
			else
			{
				if (start_word == num_words - 1)
				{
					command_print(context, "Incomplete command");
					break;
				}
				return find_and_run_command(context, c->children, words, num_words, start_word + 1);
			}
		}
	}
	
	command_print(context, "Command %s not found", words[start_word]);
	return ERROR_OK;
}

int command_run_line(command_context_t *context, char *line)
{
	int nwords;
	char *words[128] = {0};
	int retval;
	int i;
	
	if ((!context) || (!line))
		return ERROR_INVALID_ARGUMENTS;
	
	/* skip preceding whitespace */
	while (isspace(*line))
		line++;
	
	/* empty line, ignore */
	if (!*line)
		return ERROR_OK;
	
	if (context->echo)
	{
		command_print(context, "%s", line);
	}

	nwords = parse_line(line, words, sizeof(words) / sizeof(words[0]));
	
	if (nwords > 0)
		retval = find_and_run_command(context, context->commands, words, nwords, 0);
	else
		return ERROR_INVALID_ARGUMENTS;
	
	for (i = 0; i < nwords; i++)
		free(words[i]);
	
	return retval;
}

int command_run_file(command_context_t *context, FILE *file, enum command_mode mode)
{
	int retval = ERROR_OK;
	int old_command_mode;
	char buffer[4096];
	
	old_command_mode = context->mode;
	context->mode = mode;
	
	while (fgets(buffer, 4096, file))
	{
		char *p;
		char *cmd, *end;
		
		/* stop processing line after a comment (#, !) or a LF, CR were encountered */
		if ((p = strpbrk(buffer, "#!\r\n")))
			*p = 0;

		/* skip over leading whitespace */
		cmd = buffer;
		while (isspace(*cmd))
			cmd++;

		/* empty (all whitespace) line? */
		if (!*cmd)
			continue;
		
		/* search the end of the current line, ignore trailing whitespace */
		for (p = end = cmd; *p; p++)
			if (!isspace(*p))
				end = p;
		
		/* terminate end */
		*++end = 0;
		if (strcasecmp(cmd, "quit") == 0)
			break;

		/* run line */
		if ((retval = command_run_line(context, cmd)) == ERROR_COMMAND_CLOSE_CONNECTION)
			break;
	}
	
	context->mode = old_command_mode;
	
	return retval;
}

void command_print_help_line(command_context_t* context, struct command_s *command, int indent)
{
	command_t *c;
	char indents[32] = {0};
	char *help = "no help available";
	char name_buf[64];
	int i;
	
	for (i = 0; i < indent; i+=2)
	{
		indents[i*2] = ' ';
		indents[i*2+1] = '-';
	}
	indents[i*2] = 0;
	
	if ((command->mode == COMMAND_EXEC) || (command->mode == COMMAND_ANY))
	{
		if (command->help)
			help = command->help;
		
		snprintf(name_buf, 64, command->name);
		strncat(name_buf, indents, 64);
		command_print(context, "%20s\t%s", name_buf, help);
	}
	
	if (command->children)
	{
		for (c = command->children; c; c = c->next)
		{
			command_print_help_line(context, c, indent + 1);
		}
	}
}

int command_print_help(command_context_t* context, char* name, char** args, int argc)
{
	command_t *c;

	for (c = context->commands; c; c = c->next)
	{
		if (argc == 1)
		{
			 if (strncasecmp(c->name, args[0], c->unique_len))
				 continue;

			 if (strncasecmp(c->name, args[0], strlen(args[0])))
				 continue;
		} 

		command_print_help_line(context, c, 0);
	}
	
	return ERROR_OK;
}

void command_set_output_handler(command_context_t* context, int (*output_handler)(struct command_context_s *context, char* line), void *priv)
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
	
	return ERROR_OK;
}

command_context_t* command_init()
{
	command_context_t* context = malloc(sizeof(command_context_t));
	
	context->mode = COMMAND_EXEC;
	context->commands = NULL;
	context->current_target = 0;
	context->echo = 0;
	context->output_handler = NULL;
	context->output_handler_priv = NULL;
	
	register_command(context, NULL, "help", command_print_help,
					 COMMAND_EXEC, "display this help");
	
	register_command(context, NULL, "sleep", handle_sleep_command,
					 COMMAND_ANY, "sleep for <n> milliseconds");
	
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
