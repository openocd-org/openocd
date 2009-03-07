/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008, Duane Ellis                                       *
 *   openocd@duaneeellis.com                                               *
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
#include "target.h"
#include "command.h"
#include "configuration.h"

#include "log.h"
#include "time_support.h"
#include "jim-eventloop.h"

#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <stdarg.h>
#include <stdio.h>
#include <unistd.h>
#include <errno.h>

int fast_and_dangerous = 0;
Jim_Interp *interp = NULL;

int handle_sleep_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
int handle_fast_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

int run_command(command_context_t *context, command_t *c, char *words[], int num_words);

static void tcl_output(void *privData, const char *file, int line, const char *function, const char *string)
{
	Jim_Obj *tclOutput=(Jim_Obj *)privData;

	Jim_AppendString(interp, tclOutput, string, strlen(string));
}

extern command_context_t *global_cmd_ctx;

static int script_command(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	/* the private data is stashed in the interp structure */
	command_t *c;
	command_context_t *context;
	int retval;
	int i;
	int nwords;
	char **words;

	/* DANGER!!!! be careful what we invoke here, since interp->cmdPrivData might
	 * get overwritten by running other Jim commands! Treat it as an
	 * emphemeral global variable that is used in lieu of an argument
	 * to the fn and fish it out manually.
	 */
	c = interp->cmdPrivData;
	if (c==NULL)
	{
		LOG_ERROR("BUG: interp->cmdPrivData==NULL");
		return JIM_ERR;
	}
	target_call_timer_callbacks_now();
	LOG_USER_N("%s", ""); /* Keep GDB connection alive*/

	LOG_DEBUG("script_command - %s", c->name);

	words = malloc(sizeof(char *) * argc);
	for (i = 0; i < argc; i++)
	{
		int len;
		const char *w=Jim_GetString(argv[i], &len);
		if (*w=='#')
		{
			/* hit an end of line comment */
			break;
		}
		words[i] = strdup(w);
		if (words[i] == NULL)
		{
			return JIM_ERR;
		}
		LOG_DEBUG("script_command - %s, argv[%u]=%s", c->name, i, words[i]);
	}
	nwords = i;

	/* grab the command context from the associated data */
	context = Jim_GetAssocData(interp, "context");
	if (context == NULL)
	{
		/* Tcl can invoke commands directly instead of via command_run_line(). This would
		 * happen when the Jim Tcl interpreter is provided by eCos.
		 */
		context = global_cmd_ctx;
	}

	/* capture log output and return it */
	Jim_Obj *tclOutput = Jim_NewStringObj(interp, "", 0);
	/* a garbage collect can happen, so we need a reference count to this object */
	Jim_IncrRefCount(tclOutput);

	log_add_callback(tcl_output, tclOutput);

	retval = run_command(context, c, words, nwords);

	log_remove_callback(tcl_output, tclOutput);

	/* We dump output into this local variable */
	Jim_SetResult(interp, tclOutput);
	Jim_DecrRefCount(interp, tclOutput);

	for (i = 0; i < nwords; i++)
		free(words[i]);
	free(words);

	int *return_retval = Jim_GetAssocData(interp, "retval");
	if (return_retval != NULL)
	{
		*return_retval = retval;
	}

	return (retval==ERROR_OK)?JIM_OK:JIM_ERR;
}

/* nice short description of source file */
#define __THIS__FILE__ "command.c"

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
	const char *full_name=alloc_printf("ocd_%s%s%s", t1, t2, t3);
	Jim_CreateCommand(interp, full_name, script_command, c, NULL);
	free((void *)full_name);

	/* we now need to add an overrideable proc */
	const char *override_name=alloc_printf("proc %s%s%s {args} {if {[catch {eval \"ocd_%s%s%s $args\"}]==0} {return \"\"} else { return -code error }", t1, t2, t3, t1, t2, t3);
	Jim_Eval_Named(interp, override_name, __THIS__FILE__, __LINE__ );
	free((void *)override_name);

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
	c = context->commands;

	while(NULL != c)
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
				/* first element in command list */
				context->commands = c->next;
			}

			/* unregister children */
			while(NULL != c->children)
			{
				c2 = c->children;
				c->children = c->children->next;
				free(c2->name);
				c2->name = NULL;
				free(c2);
				c2 = NULL;
			}

			/* delete command */
			free(c->name);
			c->name = NULL;
			free(c);
			c = NULL;
			return ERROR_OK;
		}

		/* remember the last command for unlinking */
		p = c;
		c = c->next;
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
		/* We already printed it above */
		/* command_output_text(context, string); */
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
		/* We already printed it above */
		/* command_output_text(context, string); */
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
		LOG_ERROR("Illegal mode for command");
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
	int retval=ERROR_FAIL;
	int retcode;
	/* Beware! This code needs to be reentrant. It is also possible
	 * for OpenOCD commands to be invoked directly from Tcl. This would
	 * happen when the Jim Tcl interpreter is provided by eCos for
	 * instance.
	 */
	Jim_DeleteAssocData(interp, "context");
	retcode = Jim_SetAssocData(interp, "context", NULL, context);
	if (retcode == JIM_OK)
	{
		/* associated the return value */
		Jim_DeleteAssocData(interp, "retval");
		retcode = Jim_SetAssocData(interp, "retval", NULL, &retval);
		if (retcode == JIM_OK)
		{
			retcode = Jim_Eval_Named(interp, line, __THIS__FILE__, __LINE__ );

			Jim_DeleteAssocData(interp, "retval");
		}
		Jim_DeleteAssocData(interp, "context");
	}
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
		retval=ERROR_OK;
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

/* find full path to file */
static int jim_find(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	if (argc != 2)
		return JIM_ERR;
	const char *file = Jim_GetString(argv[1], NULL);
	char *full_path = find_file(file);
	if (full_path == NULL)
		return JIM_ERR;
	Jim_Obj *result = Jim_NewStringObj(interp, full_path, strlen(full_path));
	free(full_path);

	Jim_SetResult(interp, result);
	return JIM_OK;
}

static int jim_echo(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	if (argc != 2)
		return JIM_ERR;
	const char *str = Jim_GetString(argv[1], NULL);
	LOG_USER("%s", str);
	return JIM_OK;
}

static size_t openocd_jim_fwrite(const void *_ptr, size_t size, size_t n, void *cookie)
{
	size_t nbytes;
	const char *ptr;
	Jim_Interp *interp;

	/* make it a char easier to read code */
	ptr = _ptr;
	interp = cookie;
	nbytes = size * n;
	if (ptr == NULL || interp == NULL || nbytes == 0) {
		return 0;
	}

	/* do we have to chunk it? */
	if (ptr[nbytes] == 0)
	{
		/* no it is a C style string */
		LOG_USER_N("%s", ptr);
		return strlen(ptr);
	}
	/* GRR we must chunk - not null terminated */
	while (nbytes) {
		char chunk[128+1];
		int x;

		x = nbytes;
		if (x > 128) {
			x = 128;
		}
		/* copy it */
		memcpy(chunk, ptr, x);
		/* terminate it */
		chunk[n] = 0;
		/* output it */
		LOG_USER_N("%s", chunk);
		ptr += x;
		nbytes -= x;
	}

	return n;
}

static size_t openocd_jim_fread(void *ptr, size_t size, size_t n, void *cookie)
{
	/* TCL wants to read... tell him no */
	return 0;
}

static int openocd_jim_vfprintf(void *cookie, const char *fmt, va_list ap)
{
	char *cp;
	int n;
	Jim_Interp *interp;

	n = -1;
	interp = cookie;
	if (interp == NULL)
		return n;

	cp = alloc_vprintf(fmt, ap);
	if (cp)
	{
		LOG_USER_N("%s", cp);
		n = strlen(cp);
		free(cp);
	}
	return n;
}

static int openocd_jim_fflush(void *cookie)
{
	/* nothing to flush */
	return 0;
}

static char* openocd_jim_fgets(char *s, int size, void *cookie)
{
	/* not supported */
	errno = ENOTSUP;
	return NULL;
}

static int jim_capture(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	if (argc != 2)
		return JIM_ERR;
	int retcode;
	const char *str = Jim_GetString(argv[1], NULL);

	/* capture log output and return it */
	Jim_Obj *tclOutput = Jim_NewStringObj(interp, "", 0);
	/* a garbage collect can happen, so we need a reference count to this object */
	Jim_IncrRefCount(tclOutput);

	log_add_callback(tcl_output, tclOutput);

	retcode = Jim_Eval_Named(interp, str, __THIS__FILE__, __LINE__ );

	log_remove_callback(tcl_output, tclOutput);

	/* We dump output into this local variable */
	Jim_SetResult(interp, tclOutput);
	Jim_DecrRefCount(interp, tclOutput);

	return retcode;
}

command_context_t* command_init()
{
	command_context_t* context = malloc(sizeof(command_context_t));
	extern const char startup_tcl[];
	const char *HostOs;

	context->mode = COMMAND_EXEC;
	context->commands = NULL;
	context->current_target = 0;
	context->output_handler = NULL;
	context->output_handler_priv = NULL;

#ifdef JIM_EMBEDDED
	Jim_InitEmbedded();
	/* Create an interpreter */
	interp = Jim_CreateInterp();
	/* Add all the Jim core commands */
	Jim_RegisterCoreCommands(interp);
#endif

#if defined( _MSC_VER )
	/* WinXX - is generic, the forward
	 * looking problem is this:
	 * 
	 *   "win32" or "win64"
	 *
	 * "winxx" is generic.
	 */
	HostOs = "winxx";
#elif defined( __LINUX__)
	HostOs = "linux";
#elif defined( __DARWIN__ )
	HostOs = "darwin";
#elif defined( __CYGWIN__ )
	HostOs = "cygwin";
#elif defined( __MINGW32__ )
	HostOs = "mingw32";
#else
	HostOs = "other";
#endif
	Jim_SetGlobalVariableStr( interp, "ocd_HOSTOS", Jim_NewStringObj( interp, HostOs , strlen(HostOs)) );

	Jim_CreateCommand(interp, "ocd_find", jim_find, NULL, NULL);
	Jim_CreateCommand(interp, "echo", jim_echo, NULL, NULL);
	Jim_CreateCommand(interp, "capture", jim_capture, NULL, NULL);

	/* Set Jim's STDIO */
	interp->cookie_stdin = interp;
	interp->cookie_stdout = interp;
	interp->cookie_stderr = interp;
	interp->cb_fwrite = openocd_jim_fwrite;
	interp->cb_fread = openocd_jim_fread ;
	interp->cb_vfprintf = openocd_jim_vfprintf;
	interp->cb_fflush = openocd_jim_fflush;
	interp->cb_fgets = openocd_jim_fgets;

	add_default_dirs();

#ifdef JIM_EMBEDDED
	Jim_EventLoopOnLoad(interp);
#endif
	if (Jim_Eval_Named(interp, startup_tcl, "embedded:startup.tcl",1)==JIM_ERR)
	{
		LOG_ERROR("Failed to run startup.tcl (embedded into OpenOCD compile time)");
		Jim_PrintErrorMessage(interp);
		exit(-1);
	}

	register_command(context, NULL, "sleep", handle_sleep_command,
					 COMMAND_ANY, "<n> [busy] - sleep for n milliseconds. \"busy\" means busy wait");

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
	int busy = 0;

	if (argc==1)
	{

	} else if (argc==2)
	{
		if (strcmp(args[1], "busy")!=0)
			return ERROR_COMMAND_SYNTAX_ERROR;
		busy = 1;
	} else
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	duration = strtoul(args[0], NULL, 0);

	if (busy)
	{
		busy_sleep(duration);
	} else
	{
		long long then=timeval_ms();
		while ((timeval_ms()-then)<duration)
		{
			target_call_timer_callbacks_now();
			usleep(1000);
		}
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

void process_jim_events(void)
{
#ifdef JIM_EMBEDDED
	static int recursion = 0;

	if (!recursion)
	{
		recursion++;
		Jim_ProcessEvents (interp, JIM_ALL_EVENTS|JIM_DONT_WAIT);
		recursion--;
	}
#endif
}

void register_jim(struct command_context_s *cmd_ctx, const char *name, int (*cmd)(Jim_Interp *interp, int argc, Jim_Obj *const *argv), const char *help)
{
	Jim_CreateCommand(interp, name, cmd, NULL, NULL);

	/* FIX!!! it would be prettier to invoke add_help_text...
	 * accumulate help text in Tcl helptext list.  */
	Jim_Obj *helptext=Jim_GetGlobalVariableStr(interp, "ocd_helptext", JIM_ERRMSG);
	if (Jim_IsShared(helptext))
		helptext = Jim_DuplicateObj(interp, helptext);

	Jim_Obj *cmd_entry=Jim_NewListObj(interp, NULL, 0);

	Jim_Obj *cmd_list=Jim_NewListObj(interp, NULL, 0);
	Jim_ListAppendElement(interp, cmd_list, Jim_NewStringObj(interp, name, -1));

	Jim_ListAppendElement(interp, cmd_entry, cmd_list);
	Jim_ListAppendElement(interp, cmd_entry, Jim_NewStringObj(interp, help, -1));
	Jim_ListAppendElement(interp, helptext, cmd_entry);
}

/* return global variable long value or 0 upon failure */
long jim_global_long(const char *variable)
{
	Jim_Obj *objPtr=Jim_GetGlobalVariableStr(interp, variable, JIM_ERRMSG);
	long t;
	if (Jim_GetLong(interp, objPtr, &t)==JIM_OK)
	{
		return t;
	}
	return 0;
}
