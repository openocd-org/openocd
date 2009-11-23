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

#if !BUILD_ECOSBOARD
/* see Embedder-HOWTO.txt in Jim Tcl project hosted on BerliOS*/
#define JIM_EMBEDDED
#endif

// @todo the inclusion of target.h here is a layering violation
#include "target.h"
#include "command.h"
#include "configuration.h"
#include "log.h"
#include "time_support.h"
#include "jim-eventloop.h"


Jim_Interp *interp = NULL;

static int run_command(struct command_context *context,
		struct command *c, const char *words[], unsigned num_words);

static void tcl_output(void *privData, const char *file, unsigned line,
		const char *function, const char *string)
{
	Jim_Obj *tclOutput = (Jim_Obj *)privData;
	Jim_AppendString(interp, tclOutput, string, strlen(string));
}

extern struct command_context *global_cmd_ctx;

void script_debug(Jim_Interp *interp, const char *name,
		unsigned argc, Jim_Obj *const *argv)
{
	LOG_DEBUG("command - %s", name);
	for (unsigned i = 0; i < argc; i++)
	{
		int len;
		const char *w = Jim_GetString(argv[i], &len);

		/* end of line comment? */
		if (*w == '#')
			break;

		LOG_DEBUG("%s - argv[%d]=%s", name, i, w);
	}
}

static void script_command_args_free(const char **words, unsigned nwords)
{
	for (unsigned i = 0; i < nwords; i++)
		free((void *)words[i]);
	free(words);
}
static const char **script_command_args_alloc(
		unsigned argc, Jim_Obj *const *argv, unsigned *nwords)
{
	const char **words = malloc(argc * sizeof(char *));
	if (NULL == words)
		return NULL;

	unsigned i;
	for (i = 0; i < argc; i++)
	{
		int len;
		const char *w = Jim_GetString(argv[i], &len);
		/* a comment may end the line early */
		if (*w == '#')
			break;

		words[i] = strdup(w);
		if (words[i] == NULL)
		{
			script_command_args_free(words, i);
			return NULL;
		}
	}
	*nwords = i;
	return words;
}

static struct command_context *current_command_context(void)
{
	/* grab the command context from the associated data */
	struct command_context *cmd_ctx = Jim_GetAssocData(interp, "context");
	if (NULL == cmd_ctx)
	{
		/* Tcl can invoke commands directly instead of via command_run_line(). This would
		 * happen when the Jim Tcl interpreter is provided by eCos.
		 */
		cmd_ctx = global_cmd_ctx;
	}
	return cmd_ctx;
}

static int script_command(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	/* the private data is stashed in the interp structure */
	struct command *c;
	int retval;

	/* DANGER!!!! be careful what we invoke here, since interp->cmdPrivData might
	 * get overwritten by running other Jim commands! Treat it as an
	 * emphemeral global variable that is used in lieu of an argument
	 * to the fn and fish it out manually.
	 */
	c = interp->cmdPrivData;
	if (c == NULL)
	{
		LOG_ERROR("BUG: interp->cmdPrivData == NULL");
		return JIM_ERR;
	}
	target_call_timer_callbacks_now();
	LOG_USER_N("%s", ""); /* Keep GDB connection alive*/

	script_debug(interp, c->name, argc, argv);

	unsigned nwords;
	const char **words = script_command_args_alloc(argc, argv, &nwords);
	if (NULL == words)
		return JIM_ERR;

	/* capture log output and return it */
	Jim_Obj *tclOutput = Jim_NewStringObj(interp, "", 0);
	/* a garbage collect can happen, so we need a reference count to this object */
	Jim_IncrRefCount(tclOutput);

	log_add_callback(tcl_output, tclOutput);

	struct command_context *cmd_ctx = current_command_context();
	retval = run_command(cmd_ctx, c, (const char **)words, nwords);

	log_remove_callback(tcl_output, tclOutput);

	/* We dump output into this local variable */
	Jim_SetResult(interp, tclOutput);
	Jim_DecrRefCount(interp, tclOutput);

	script_command_args_free(words, nwords);

	int *return_retval = Jim_GetAssocData(interp, "retval");
	if (return_retval != NULL)
	{
		*return_retval = retval;
	}

	return (retval == ERROR_OK)?JIM_OK:JIM_ERR;
}

/* nice short description of source file */
#define __THIS__FILE__ "command.c"

/**
 * Find a command by name from a list of commands.
 * @returns Returns the named command if it exists in the list.
 * Returns NULL otherwise.
 */
static struct command *command_find(struct command *head, const char *name)
{
	for (struct command *cc = head; cc; cc = cc->next)
	{
		if (strcmp(cc->name, name) == 0)
			return cc;
	}
	return NULL;
}
struct command *command_find_in_context(struct command_context *cmd_ctx,
		const char *name)
{
	return command_find(cmd_ctx->commands, name);
}
struct command *command_find_in_parent(struct command *parent,
		const char *name)
{
	return command_find(parent->children, name);
}

/**
 * Add the command into the linked list, sorted by name.
 * @param head Address to head of command list pointer, which may be
 * updated if @c c gets inserted at the beginning of the list.
 * @param c The command to add to the list pointed to by @c head.
 */
static void command_add_child(struct command **head, struct command *c)
{
	assert(head);
	if (NULL == *head)
	{
		*head = c;
		return;
	}

	while ((*head)->next && (strcmp(c->name, (*head)->name) > 0))
		head = &(*head)->next;

	if (strcmp(c->name, (*head)->name) > 0) {
		c->next = (*head)->next;
		(*head)->next = c;
	} else {
		c->next = *head;
		*head = c;
	}
}

static struct command **command_list_for_parent(
		struct command_context *cmd_ctx, struct command *parent)
{
	return parent ? &parent->children : &cmd_ctx->commands;
}

static struct command *command_new(struct command_context *cmd_ctx,
		struct command *parent, const struct command_registration *cr)
{
	assert(cr->name);

	struct command *c = malloc(sizeof(struct command));
	memset(c, 0, sizeof(struct command));

	c->name = strdup(cr->name);
	if (cr->help)
		c->help = strdup(cr->help);
	if (cr->usage)
		c->usage = strdup(cr->usage);
	c->parent = parent;
	c->handler = cr->handler;
	c->jim_handler = cr->jim_handler;
	c->jim_handler_data = cr->jim_handler_data;
	c->mode = cr->mode;

	command_add_child(command_list_for_parent(cmd_ctx, parent), c);

	return c;
}
static void command_free(struct command *c)
{
	/// @todo if command has a handler, unregister its jim command!

	while (NULL != c->children)
	{
		struct command *tmp = c->children;
		c->children = tmp->next;
		command_free(tmp);
	}

	if (c->name)
		free(c->name);
	if (c->help)
		free((void*)c->help);
	if (c->usage)
		free((void*)c->usage);
	free(c);
}

static int register_command_handler(struct command *c)
{
	int retval = -ENOMEM;
	const char *full_name = command_name(c, '_');
	if (NULL == full_name)
		return retval;

	const char *ocd_name = alloc_printf("ocd_%s", full_name);
	if (NULL == full_name)
		goto free_full_name;

	Jim_CreateCommand(interp, ocd_name, script_command, c, NULL);
	free((void *)ocd_name);

	/* we now need to add an overrideable proc */
	const char *override_name = alloc_printf("proc %s {args} {"
			"if {[catch {eval ocd_%s $args}] == 0} "
			"{return \"\"} else {return -code error}}",
			full_name, full_name);
	if (NULL == full_name)
		goto free_full_name;

	Jim_Eval_Named(interp, override_name, __THIS__FILE__, __LINE__);
	free((void *)override_name);

	retval = ERROR_OK;

free_full_name:
	free((void *)full_name);
	return retval;
}

struct command* register_command(struct command_context *context,
		struct command *parent, const struct command_registration *cr)
{
	if (!context || !cr->name)
		return NULL;

	const char *name = cr->name;
	struct command **head = command_list_for_parent(context, parent);
	struct command *c = command_find(*head, name);
	if (NULL != c)
	{
		LOG_ERROR("command '%s' is already registered in '%s' context",
				name, parent ? parent->name : "<global>");
		return c;
	}

	c = command_new(context, parent, cr);
	if (NULL == c)
		return NULL;

	if (NULL != c->handler)
	{
		int retval = register_command_handler(c);
		if (ERROR_OK != retval)
		{
			unregister_command(context, parent, name);
			return NULL;
		}
	}

	if (NULL != cr->jim_handler && NULL == parent)
		Jim_CreateCommand(interp, cr->name, cr->jim_handler, cr->jim_handler_data, NULL);

	return c;
}

int register_commands(struct command_context *cmd_ctx, struct command *parent,
		const struct command_registration *cmds)
{
	int retval = ERROR_OK;
	unsigned i;
	for (i = 0; cmds[i].name || cmds[i].chain; i++)
	{
		const struct command_registration *cr = cmds + i;

		struct command *c = NULL;
		if (NULL != cr->name)
		{
			c = register_command(cmd_ctx, parent, cr);
			if (NULL == c)
			{
				retval = ERROR_FAIL;
				break;
			}
		}
		if (NULL != cr->chain)
		{
			struct command *p = c ? : parent;
			retval = register_commands(cmd_ctx, p, cr->chain);
			if (ERROR_OK != retval)
				break;
		}
	}
	if (ERROR_OK != retval)
	{
		for (unsigned j = 0; j < i; j++)
			unregister_command(cmd_ctx, parent, cmds[j].name);
	}
	return retval;
}

int unregister_all_commands(struct command_context *context,
		struct command *parent)
{
	if (context == NULL)
		return ERROR_OK;

	struct command **head = command_list_for_parent(context, parent);
	while (NULL != *head)
	{
		struct command *tmp = *head;
		*head = tmp->next;
		command_free(tmp);
	}

	return ERROR_OK;
}

int unregister_command(struct command_context *context,
		struct command *parent, const char *name)
{
	if ((!context) || (!name))
		return ERROR_INVALID_ARGUMENTS;

	struct command *p = NULL;
	struct command **head = command_list_for_parent(context, parent);
	for (struct command *c = *head; NULL != c; p = c, c = c->next)
	{
		if (strcmp(name, c->name) != 0)
			continue;

		if (p)
			p->next = c->next;
		else
			*head = c->next;

		command_free(c);
		return ERROR_OK;
	}

	return ERROR_OK;
}

void command_output_text(struct command_context *context, const char *data)
{
	if (context && context->output_handler && data) {
		context->output_handler(context, data);
	}
}

void command_print_sameline(struct command_context *context, const char *format, ...)
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

void command_print(struct command_context *context, const char *format, ...)
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

static char *__command_name(struct command *c, char delim, unsigned extra)
{
	char *name;
	unsigned len = strlen(c->name);
	if (NULL == c->parent) {
		// allocate enough for the name, child names, and '\0'
		name = malloc(len + extra + 1);
		strcpy(name, c->name);
	} else {
		// parent's extra must include both the space and name
		name = __command_name(c->parent, delim, 1 + len + extra);
		char dstr[2] = { delim, 0 };
		strcat(name, dstr);
		strcat(name, c->name);
	}
	return name;
}
char *command_name(struct command *c, char delim)
{
	return __command_name(c, delim, 0);
}

static int run_command(struct command_context *context,
		struct command *c, const char *words[], unsigned num_words)
{
	if (!((context->mode == COMMAND_CONFIG) || (c->mode == COMMAND_ANY) || (c->mode == context->mode)))
	{
		/* Config commands can not run after the config stage */
		LOG_ERROR("Command '%s' only runs during configuration stage", c->name);
		return ERROR_FAIL;
	}

	struct command_invocation cmd = {
			.ctx = context,
			.name = c->name,
			.argc = num_words - 1,
			.argv = words + 1,
		};
	int retval = c->handler(&cmd);
	if (retval == ERROR_COMMAND_SYNTAX_ERROR)
	{
		/* Print help for command */
		char *full_name = command_name(c, ' ');
		if (NULL != full_name) {
			command_run_linef(context, "help %s", full_name);
			free(full_name);
		} else
			retval = -ENOMEM;
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

int command_run_line(struct command_context *context, char *line)
{
	/* all the parent commands have been registered with the interpreter
	 * so, can just evaluate the line as a script and check for
	 * results
	 */
	/* run the line thru a script engine */
	int retval = ERROR_FAIL;
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
			retcode = Jim_Eval_Named(interp, line, __THIS__FILE__, __LINE__);

			Jim_DeleteAssocData(interp, "retval");
		}
		Jim_DeleteAssocData(interp, "context");
	}
	if (retcode == JIM_ERR) {
		if (retval != ERROR_COMMAND_CLOSE_CONNECTION)
		{
			/* We do not print the connection closed error message */
			Jim_PrintErrorMessage(interp);
		}
		if (retval == ERROR_OK)
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
		if (reslen > 0)
		{
			int i;
			char buff[256 + 1];
			for (i = 0; i < reslen; i += 256)
			{
				int chunk;
				chunk = reslen - i;
				if (chunk > 256)
					chunk = 256;
				strncpy(buff, result + i, chunk);
				buff[chunk] = 0;
				LOG_USER_N("%s", buff);
			}
			LOG_USER_N("%s", "\n");
		}
		retval = ERROR_OK;
	}
	return retval;
}

int command_run_linef(struct command_context *context, const char *format, ...)
{
	int retval = ERROR_FAIL;
	char *string;
	va_list ap;
	va_start(ap, format);
	string = alloc_vprintf(format, ap);
	if (string != NULL)
	{
		retval = command_run_line(context, string);
	}
	va_end(ap);
	return retval;
}

void command_set_output_handler(struct command_context* context,
		command_output_handler_t output_handler, void *priv)
{
	context->output_handler = output_handler;
	context->output_handler_priv = priv;
}

struct command_context* copy_command_context(struct command_context* context)
{
	struct command_context* copy_context = malloc(sizeof(struct command_context));

	*copy_context = *context;

	return copy_context;
}

int command_done(struct command_context *context)
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
		char chunk[128 + 1];
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

	retcode = Jim_Eval_Named(interp, str, __THIS__FILE__, __LINE__);

	log_remove_callback(tcl_output, tclOutput);

	/* We dump output into this local variable */
	Jim_SetResult(interp, tclOutput);
	Jim_DecrRefCount(interp, tclOutput);

	return retcode;
}

static COMMAND_HELPER(command_help_find, struct command *head,
		struct command **out)
{
	if (0 == CMD_ARGC)
		return ERROR_INVALID_ARGUMENTS;
	*out = command_find(head, CMD_ARGV[0]);
	if (NULL == *out)
		return ERROR_INVALID_ARGUMENTS;
	if (--CMD_ARGC == 0)
		return ERROR_OK;
	CMD_ARGV++;
	return CALL_COMMAND_HANDLER(command_help_find, (*out)->children, out);
}

static COMMAND_HELPER(command_help_show, struct command *c, unsigned n,
		bool show_help);

static COMMAND_HELPER(command_help_show_list, struct command *head, unsigned n,
		bool show_help)
{
	for (struct command *c = head; NULL != c; c = c->next)
		CALL_COMMAND_HANDLER(command_help_show, c, n, show_help);
	return ERROR_OK;
}
static COMMAND_HELPER(command_help_show, struct command *c, unsigned n,
		bool show_help)
{
	const char *usage = c->usage ? : "";
	const char *help = "";
	const char *sep = "";
	if (show_help && c->help)
	{
		help = c->help ? : "";
		sep = c->usage ? " | " : "";
	}
	command_run_linef(CMD_CTX, "cmd_help {%s} {%s%s%s} %d",
			command_name(c, ' '), usage, sep, help, n);

	if (++n >= 2)
		return ERROR_OK;

	return CALL_COMMAND_HANDLER(command_help_show_list,
			c->children, n, show_help);
}
COMMAND_HANDLER(handle_help_command)
{
	struct command *c = CMD_CTX->commands;

	if (0 == CMD_ARGC)
		return CALL_COMMAND_HANDLER(command_help_show_list, c, 0, true);

	int retval = CALL_COMMAND_HANDLER(command_help_find, c, &c);
	if (ERROR_OK != retval)
		return retval;

	return CALL_COMMAND_HANDLER(command_help_show, c, 0, true);
}

COMMAND_HANDLER(handle_usage_command)
{
	struct command *c = CMD_CTX->commands;

	if (0 == CMD_ARGC)
		return CALL_COMMAND_HANDLER(command_help_show_list, c, 0, false);

	int retval = CALL_COMMAND_HANDLER(command_help_find, c, &c);
	if (ERROR_OK != retval)
		return retval;

	return CALL_COMMAND_HANDLER(command_help_show, c, 0, false);
}

static int command_unknown_find(unsigned argc, Jim_Obj *const *argv,
		struct command *head, struct command **out)
{
	if (0 == argc)
		return argc;
	struct command *c = command_find(head, Jim_GetString(argv[0], NULL));
	if (NULL == c)
		return argc;
	*out = c;
	return command_unknown_find(--argc, ++argv, (*out)->children, out);
}

static int command_unknown(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	const char *cmd_name = Jim_GetString(argv[0], NULL);
	script_debug(interp, cmd_name, argc - 1, argv + 1);

	struct command_context *cmd_ctx = current_command_context();
	struct command *c = cmd_ctx->commands;
	int remaining = command_unknown_find(argc - 1, argv + 1, c, &c);
	// if nothing could be consumed, then it's really an unknown command
	if (remaining == argc - 1)
	{
		const char *cmd = Jim_GetString(argv[1], NULL);
		LOG_ERROR("Unknown command:\n  %s", cmd);
		return JIM_OK;
	}

	bool found = true;
	Jim_Obj *const *start;
	unsigned count;
	if (c->handler || c->jim_handler)
	{
		// include the command name in the list
		count = remaining + 1;
		start = argv + (argc - remaining - 1);
	}
	else
	{
		c = command_find(cmd_ctx->commands, "help");
		if (NULL == c)
		{
			LOG_ERROR("unknown command, but help is missing too");
			return JIM_ERR;
		}
		count = argc - remaining;
		start = argv;
		found = false;
	}
	// pass the command through to the intended handler
	if (c->jim_handler)
	{
		interp->cmdPrivData = c->jim_handler_data;
		return (*c->jim_handler)(interp, count, start);
	}

	unsigned nwords;
	const char **words = script_command_args_alloc(count, start, &nwords);
	if (NULL == words)
		return JIM_ERR;

	int retval = run_command(cmd_ctx, c, words, nwords);

	script_command_args_free(words, nwords);

	if (!found && ERROR_OK == retval)
		retval = ERROR_FAIL;

	return retval;
}

int help_add_command(struct command_context *cmd_ctx, struct command *parent,
		const char *cmd_name, const char *help_text, const char *usage)
{
	struct command **head = command_list_for_parent(cmd_ctx, parent);
	struct command *nc = command_find(*head, cmd_name);
	if (NULL == nc)
	{
		// add a new command with help text
		struct command_registration cr = {
				.name = cmd_name,
				.mode = COMMAND_ANY,
				.help = help_text,
				.usage = usage,
			};
		nc = register_command(cmd_ctx, parent, &cr);
		if (NULL == nc)
		{
			LOG_ERROR("failed to add '%s' help text", cmd_name);
			return ERROR_FAIL;
		}
		LOG_DEBUG("added '%s' help text", cmd_name);
	}
	else
	{
		bool replaced = false;
		if (nc->help)
		{
			free((void *)nc->help);
			replaced = true;
		}
		nc->help = strdup(help_text);

		if (replaced)
			LOG_INFO("replaced existing '%s' help", cmd_name);
		else
			LOG_DEBUG("added '%s' help text", cmd_name);
	}
	return ERROR_OK;
}

COMMAND_HANDLER(handle_help_add_command)
{
	if (CMD_ARGC < 2)
	{
		LOG_ERROR("%s: insufficient arguments", CMD_NAME);
		return ERROR_INVALID_ARGUMENTS;
	}

	// save help text and remove it from argument list
	const char *help_text = CMD_ARGV[--CMD_ARGC];
	// likewise for the leaf command name
	const char *cmd_name = CMD_ARGV[--CMD_ARGC];

	struct command *c = NULL;
	if (CMD_ARGC > 0)
	{
		c = CMD_CTX->commands;
		int retval = CALL_COMMAND_HANDLER(command_help_find, c, &c);
		if (ERROR_OK != retval)
			return retval;
	}
	return help_add_command(CMD_CTX, c, cmd_name, help_text, NULL);
}

/* sleep command sleeps for <n> miliseconds
 * this is useful in target startup scripts
 */
COMMAND_HANDLER(handle_sleep_command)
{
	bool busy = false;
	if (CMD_ARGC == 2)
	{
		if (strcmp(CMD_ARGV[1], "busy") == 0)
			busy = true;
		else
			return ERROR_COMMAND_SYNTAX_ERROR;
	}
	else if (CMD_ARGC < 1 || CMD_ARGC > 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	unsigned long duration = 0;
	int retval = parse_ulong(CMD_ARGV[0], &duration);
	if (ERROR_OK != retval)
		return retval;

	if (!busy)
	{
		long long then = timeval_ms();
		while (timeval_ms() - then < (long long)duration)
		{
			target_call_timer_callbacks_now();
			usleep(1000);
		}
	}
	else
		busy_sleep(duration);

	return ERROR_OK;
}

static const struct command_registration command_builtin_handlers[] = {
	{
		.name = "add_help_text",
		.handler = &handle_help_add_command,
		.mode = COMMAND_ANY,
		.help = "add new command help text",
		.usage = "<command> [...] <help_text>]",
	},
	{
		.name = "sleep",
		.handler = &handle_sleep_command,
		.mode = COMMAND_ANY,
		.help = "sleep for n milliseconds.  "
			"\"busy\" will busy wait",
		.usage = "<n> [busy]",
	},
	{
		.name = "help",
		.handler = &handle_help_command,
		.mode = COMMAND_ANY,
		.help = "show built-in command help",
		.usage = "[<command_name> ...]",
	},
	{
		.name = "usage",
		.handler = &handle_usage_command,
		.mode = COMMAND_ANY,
		.help = "show command usage",
		.usage = "[<command_name> ...]",
	},
	COMMAND_REGISTRATION_DONE
};

struct command_context* command_init(const char *startup_tcl)
{
	struct command_context* context = malloc(sizeof(struct command_context));
	const char *HostOs;

	context->mode = COMMAND_EXEC;
	context->commands = NULL;
	context->current_target = 0;
	context->output_handler = NULL;
	context->output_handler_priv = NULL;

#if !BUILD_ECOSBOARD
	Jim_InitEmbedded();
	/* Create an interpreter */
	interp = Jim_CreateInterp();
	/* Add all the Jim core commands */
	Jim_RegisterCoreCommands(interp);
#endif

#if defined(_MSC_VER)
	/* WinXX - is generic, the forward
	 * looking problem is this:
	 *
	 *   "win32" or "win64"
	 *
	 * "winxx" is generic.
	 */
	HostOs = "winxx";
#elif defined(__linux__)
	HostOs = "linux";
#elif defined(__DARWIN__)
	HostOs = "darwin";
#elif defined(__CYGWIN__)
	HostOs = "cygwin";
#elif defined(__MINGW32__)
	HostOs = "mingw32";
#elif defined(__ECOS)
	HostOs = "ecos";
#else
#warn unrecognized host OS...
	HostOs = "other";
#endif
	Jim_SetGlobalVariableStr(interp, "ocd_HOSTOS",
			Jim_NewStringObj(interp, HostOs , strlen(HostOs)));

	Jim_CreateCommand(interp, "unknown", &command_unknown, NULL, NULL);
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

	register_commands(context, NULL, command_builtin_handlers);

#if !BUILD_ECOSBOARD
	Jim_EventLoopOnLoad(interp);
#endif
	Jim_SetAssocData(interp, "context", NULL, context);
	if (Jim_Eval_Named(interp, startup_tcl, "embedded:startup.tcl",1) == JIM_ERR)
	{
		LOG_ERROR("Failed to run startup.tcl (embedded into OpenOCD)");
		Jim_PrintErrorMessage(interp);
		exit(-1);
	}
	Jim_DeleteAssocData(interp, "context");

	return context;
}

int command_context_mode(struct command_context *cmd_ctx, enum command_mode mode)
{
	if (!cmd_ctx)
		return ERROR_INVALID_ARGUMENTS;

	cmd_ctx->mode = mode;
	return ERROR_OK;
}

void process_jim_events(void)
{
#if !BUILD_ECOSBOARD
	static int recursion = 0;

	if (!recursion)
	{
		recursion++;
		Jim_ProcessEvents (interp, JIM_ALL_EVENTS | JIM_DONT_WAIT);
		recursion--;
	}
#endif
}

#define DEFINE_PARSE_NUM_TYPE(name, type, func, min, max) \
	int parse##name(const char *str, type *ul) \
	{ \
		if (!*str) \
		{ \
			LOG_ERROR("Invalid command argument"); \
			return ERROR_COMMAND_ARGUMENT_INVALID; \
		} \
		char *end; \
		*ul = func(str, &end, 0); \
		if (*end) \
		{ \
			LOG_ERROR("Invalid command argument"); \
			return ERROR_COMMAND_ARGUMENT_INVALID; \
		} \
		if ((max == *ul) && (ERANGE == errno)) \
		{ \
			LOG_ERROR("Argument overflow"); \
			return ERROR_COMMAND_ARGUMENT_OVERFLOW; \
		} \
		if (min && (min == *ul) && (ERANGE == errno)) \
		{ \
			LOG_ERROR("Argument underflow"); \
			return ERROR_COMMAND_ARGUMENT_UNDERFLOW; \
		} \
		return ERROR_OK; \
	}
DEFINE_PARSE_NUM_TYPE(_ulong, unsigned long , strtoul, 0, ULONG_MAX)
DEFINE_PARSE_NUM_TYPE(_ullong, unsigned long long, strtoull, 0, ULLONG_MAX)
DEFINE_PARSE_NUM_TYPE(_long, long , strtol, LONG_MIN, LONG_MAX)
DEFINE_PARSE_NUM_TYPE(_llong, long long, strtoll, LLONG_MIN, LLONG_MAX)

#define DEFINE_PARSE_WRAPPER(name, type, min, max, functype, funcname) \
	int parse##name(const char *str, type *ul) \
	{ \
		functype n; \
		int retval = parse##funcname(str, &n); \
		if (ERROR_OK != retval) \
			return retval; \
		if (n > max) \
			return ERROR_COMMAND_ARGUMENT_OVERFLOW; \
		if (min) \
			return ERROR_COMMAND_ARGUMENT_UNDERFLOW; \
		*ul = n; \
		return ERROR_OK; \
	}

#define DEFINE_PARSE_ULONG(name, type, min, max) \
	DEFINE_PARSE_WRAPPER(name, type, min, max, unsigned long, _ulong)
DEFINE_PARSE_ULONG(_uint, unsigned, 0, UINT_MAX)
DEFINE_PARSE_ULONG(_u32, uint32_t, 0, UINT32_MAX)
DEFINE_PARSE_ULONG(_u16, uint16_t, 0, UINT16_MAX)
DEFINE_PARSE_ULONG(_u8, uint8_t, 0, UINT8_MAX)

#define DEFINE_PARSE_LONG(name, type, min, max) \
	DEFINE_PARSE_WRAPPER(name, type, min, max, long, _long)
DEFINE_PARSE_LONG(_int, int, n < INT_MIN, INT_MAX)
DEFINE_PARSE_LONG(_s32, int32_t, n < INT32_MIN, INT32_MAX)
DEFINE_PARSE_LONG(_s16, int16_t, n < INT16_MIN, INT16_MAX)
DEFINE_PARSE_LONG(_s8, int8_t, n < INT8_MIN, INT8_MAX)

static int command_parse_bool(const char *in, bool *out,
		const char *on, const char *off)
{
	if (strcasecmp(in, on) == 0)
		*out = true;
	else if (strcasecmp(in, off) == 0)
		*out = false;
	else
		return ERROR_COMMAND_SYNTAX_ERROR;
	return  ERROR_OK;
}

int command_parse_bool_arg(const char *in, bool *out)
{
	if (command_parse_bool(in, out, "on", "off") == ERROR_OK)
		return ERROR_OK;
	if (command_parse_bool(in, out, "enable", "disable") == ERROR_OK)
		return ERROR_OK;
	if (command_parse_bool(in, out, "true", "false") == ERROR_OK)
		return ERROR_OK;
	if (command_parse_bool(in, out, "yes", "no") == ERROR_OK)
		return ERROR_OK;
	if (command_parse_bool(in, out, "1", "0") == ERROR_OK)
		return ERROR_OK;
	return ERROR_INVALID_ARGUMENTS;
}

COMMAND_HELPER(handle_command_parse_bool, bool *out, const char *label)
{
	switch (CMD_ARGC) {
	case 1: {
		const char *in = CMD_ARGV[0];
		if (command_parse_bool_arg(in, out) != ERROR_OK)
		{
			LOG_ERROR("%s: argument '%s' is not valid", CMD_NAME, in);
			return ERROR_INVALID_ARGUMENTS;
		}
		// fall through
	}
	case 0:
		LOG_INFO("%s is %s", label, *out ? "enabled" : "disabled");
		break;
	default:
		return ERROR_INVALID_ARGUMENTS;
	}
	return ERROR_OK;
}
