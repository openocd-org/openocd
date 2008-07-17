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
int handle_fast_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static void tcl_output(void *privData, const char *file, int line, const char *function, const char *string)
{		
	Jim_Obj *tclOutput=(Jim_Obj *)privData;

	Jim_AppendString(interp, tclOutput, string, strlen(string));
}

static int script_command(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	/* the private data is stashed in the interp structure */
	command_t *c;
	command_context_t *context;
	int *retval;
	int i;
	int nwords;
	char **words;

	target_call_timer_callbacks_now();
	LOG_USER_N("%s", ""); /* Keep GDB connection alive*/ 
	
	c = interp->cmdPrivData;
	LOG_DEBUG("script_command - %s", c->name);

	nwords = argc;
	words = malloc(sizeof(char *) * nwords);
	for (i = 0; i < nwords; i++)
	{
		int len;

		words[i] = strdup(Jim_GetString(argv[i], &len));
		if (words[i] == NULL) 
		{
			return JIM_ERR;
		}
		LOG_DEBUG("script_command - %s, argv[%u]=%s", c->name, i, words[i]);
	}

	/* grab the command context from the associated data */
	context = Jim_GetAssocData(interp, "context");
	retval = Jim_GetAssocData(interp, "retval"); 
	if (context != NULL && retval != NULL)
	{
		/* capture log output and return it */
		Jim_Obj *tclOutput = Jim_NewStringObj(interp, "", 0);
		log_add_callback(tcl_output, tclOutput);
		
		*retval = run_command(context, c, words, nwords);
		
		log_remove_callback(tcl_output, tclOutput);
		
		/* We dump output into this local variable */
		Jim_SetVariableStr(interp, "openocd_output", tclOutput);
	}

	for (i = 0; i < nwords; i++)
		free(words[i]);
	free(words);

	return (*retval==ERROR_OK)?JIM_OK:JIM_ERR;
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
	
	/* just a placeholder, no handler */
	if (c->handler==NULL)
		return c;

	/* If this is a two level command, e.g. "flash banks", then the
	 * "unknown" proc in startup.tcl must redirect to  this command.
	 * 
	 * "flash banks" is translated by "unknown" to "flash_banks"
	 * if such a proc exists
	 */
	/* Print help for command */
	const char *t1="";
	const char *t2="";
	const char *t3="";
	/* maximum of two levels :-) */
	if (c->parent!=NULL)
	{
		t1=c->parent->name;
		t2="_";
	}
	t3=c->name;
	const char *full_name=alloc_printf("%s%s%s", t1, t2, t3);
	Jim_CreateCommand(interp, full_name, script_command, c, NULL);
	free((void *)full_name);
	
	
	/* accumulate help text in Tcl helptext list.  */
    Jim_Obj *helptext=Jim_GetGlobalVariableStr(interp, "ocd_helptext", JIM_ERRMSG);
    if (Jim_IsShared(helptext))
        helptext = Jim_DuplicateObj(interp, helptext);
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
		/* we want this collected in the log + we also want to pick it up as a tcl return
		 * value.
		 * 
		 * The latter bit isn't precisely neat, but will do for now.
		 */
		LOG_USER_N("%s", string);
		// We already printed it above
		//command_output_text(context, string);
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
		/* we want this collected in the log + we also want to pick it up as a tcl return
		 * value.
		 * 
		 * The latter bit isn't precisely neat, but will do for now.
		 */
		LOG_USER_N("%s", string);
		// We already printed it above
		//command_output_text(context, string);
		free(string);
	}

	va_end(ap);
}

int run_command(command_context_t *context, command_t *c, char *words[], int num_words)
{
	int start_word=0;
	if (!((context->mode == COMMAND_CONFIG) || (c->mode == COMMAND_ANY) || (c->mode == context->mode) ))
	{
		/* Config commands can not run after the config stage */
		return ERROR_FAIL;
	}
	
	int retval = c->handler(context, c->name, words + start_word + 1, num_words - start_word - 1);
	if (retval == ERROR_COMMAND_SYNTAX_ERROR)
	{
		/* Print help for command */
		const char *t1="";
		const char *t2="";
		const char *t3="";
		/* maximum of two levels :-) */
		if (c->parent!=NULL)
		{
			t1=c->parent->name;
			t2=" ";
		}
		t3=c->name;
		command_run_linef(context, "help {%s%s%s}", t1, t2, t3);
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

int command_run_line(command_context_t *context, char *line)
{
	/* all the parent commands have been registered with the interpreter
	 * so, can just evaluate the line as a script and check for
	 * results
	 */
	/* run the line thru a script engine */
	int retval;
	int retcode;
	Jim_DeleteAssocData(interp, "context"); /* remove existing */
	retcode = Jim_SetAssocData(interp, "context", NULL, context);
	if (retcode != JIM_OK)
		return ERROR_FAIL;

	/* associated the return value */
	retval = ERROR_OK;
	Jim_DeleteAssocData(interp, "retval"); /* remove existing */
	retcode = Jim_SetAssocData(interp, "retval", NULL, &retval);
	if (retcode != JIM_OK)
		return ERROR_FAIL;

	retcode = Jim_Eval(interp, line);	
	if (retcode == JIM_ERR) {
		if (retval!=ERROR_COMMAND_CLOSE_CONNECTION)
		{
			/* We do not print the connection closed error message */
			Jim_PrintErrorMessage(interp);
		}
		if (retval==ERROR_OK)
		{
			/* It wasn't a low level OpenOCD command that failed */
			return ERROR_FAIL; 
		}
		return retval;
	} else if (retcode == JIM_EXIT) {
		/* ignore. */
		/* exit(Jim_GetExitCode(interp)); */
	} else {
		const char *result;
		int reslen;

		result = Jim_GetString(Jim_GetResult(interp), &reslen);
		if (reslen) {
			int i;
			char buff[256+1];
			for (i = 0; i < reslen; i += 256)
			{
				int chunk;
				chunk = reslen - i;
				if (chunk > 256)
					chunk = 256;
				strncpy(buff, result+i, chunk);
				buff[chunk] = 0; 
				LOG_USER_N("%s", buff);
			}
			LOG_USER_N("%s", "\n");
		}
	}
	return retval;
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
	
	register_command(context, NULL, "fast", handle_fast_command,
					 COMMAND_ANY, "fast <enable/disable> - place at beginning of config files. Sets defaults to fast and dangerous.");
	
	return context;
}

int command_context_mode(command_context_t *cmd_ctx, enum command_mode mode)
{
	if (!cmd_ctx)
		return ERROR_INVALID_ARGUMENTS;

	cmd_ctx->mode = mode;
	return ERROR_OK;
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
