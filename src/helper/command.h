/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
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
#ifndef COMMAND_H
#define COMMAND_H

#include "types.h"

/* Integrate the JIM TCL interpretor into the command processing. */
#if BUILD_ECOSBOARD
#include <stdio.h>
#include <stdarg.h>
/* Jim is provied by eCos */
#include <cyg/jimtcl/jim.h>
#else
#include "jim.h"
#endif

/* To achieve C99 printf compatibility in MinGW, gnu_printf should be
 * used for __attribute__((format( ... ))), with GCC v4.4 or later
 */
#if (defined(IS_MINGW) && (((__GNUC__ << 16) + __GNUC_MINOR__) >= 0x00040004))
#define PRINTF_ATTRIBUTE_FORMAT gnu_printf
#else
#define PRINTF_ATTRIBUTE_FORMAT printf
#endif

enum command_mode
{
	COMMAND_EXEC,
	COMMAND_CONFIG,
	COMMAND_ANY,
};

struct command_context;

/// The type signature for command context's output handler.
typedef int (*command_output_handler_t)(struct command_context *context,
				const char* line);

struct command_context
{
	enum command_mode mode;
	struct command *commands;
	int current_target;
	/* Execute a command.
	 *
	 * If the command fails, it *MUST* return a value != ERROR_OK
	 * (many commands break this rule, patches welcome!)
	 *
	 * This is *especially* important for commands such as writing
	 * to flash or verifying memory. The reason is that those commands
	 * can be used by programs to determine if the operation succeded
	 * or not. If the operation failed, then a program can try
	 * an alternative approach.
	 *
	 * Returning ERROR_COMMAND_SYNTAX_ERROR will have the effect of
	 * printing out the syntax of the command.
	 */
	command_output_handler_t output_handler;
	void *output_handler_priv;
};

/**
 * When run_command is called, a new instance will be created on the
 * stack, filled with the proper values, and passed by reference to the
 * required COMMAND_HANDLER routine.
 */
struct command_invocation {
	struct command_context *ctx;
	const char *name;
	unsigned argc;
	const char **argv;
};

/**
 * Command handlers may be defined with more parameters than the base
 * set provided by command.c.  This macro uses C99 magic to allow
 * defining all such derivative types using this macro.
 */
#define __COMMAND_HANDLER(name, extra...) \
		int name(struct command_invocation *cmd, ##extra)

/**
 * Use this to macro to call a command helper (or a nested handler).
 * It provides command handler authors protection against reordering or
 * removal of unused parameters.
 *
 * @b Note: This macro uses lexical capture to provide some arguments.
 * As a result, this macro should be used @b only within functions
 * defined by the COMMAND_HANDLER or COMMAND_HELPER macros.  Those
 * macros provide the expected lexical context captured by this macro.
 * Furthermore, it should be used only from the top-level of handler or
 * helper function, or care must be taken to avoid redefining the same
 * variables in intervening scope(s) by accident.
 */
#define CALL_COMMAND_HANDLER(name, extra...) \
		name(cmd, ##extra)

/**
 * Always use this macro to define new command handler functions.
 * It ensures the parameters are ordered, typed, and named properly, so
 * they be can be used by other macros (e.g. COMMAND_PARSE_NUMBER).
 * All command handler functions must be defined as static in scope.
 */
#define COMMAND_HANDLER(name) static __COMMAND_HANDLER(name)

/**
 * Similar to COMMAND_HANDLER, except some parameters are expected.
 * A helper is globally-scoped because it may be shared between several
 * source files (e.g. the s3c24xx device command helper).
 */
#define COMMAND_HELPER(name, extra...) __COMMAND_HANDLER(name, extra)

/**
 * Use this macro to access the context of the command being handled,
 * rather than accessing the variable directly.  It may be moved.
 */
#define CMD_CTX cmd->ctx
/**
 * Use this macro to access the number of arguments for the command being
 * handled, rather than accessing the variable directly.  It may be moved.
 */
#define CMD_ARGC cmd->argc
/**
 * Use this macro to access the arguments for the command being handled,
 * rather than accessing the variable directly.  It may be moved.
 */
#define CMD_ARGV cmd->argv
/**
 * Use this macro to access the name of the command being handled,
 * rather than accessing the variable directly.  It may be moved.
 */
#define CMD_NAME cmd->name


/// The type signature for commands' handler functions.
typedef __COMMAND_HANDLER((*command_handler_t));

struct command
{
	char *name;
	struct command *parent;
	struct command *children;
	command_handler_t handler;
	enum command_mode mode;
	struct command *next;
};

/**
 * @param c The command to be named.
 * @param delim The character to place between command names.
 * @returns A malloc'd string containing the full command name,
 * which may include one or more ancestor components.  Multiple names
 * are separated by single spaces.  The caller must free() the string
 * when done with it.
 */
char *command_name(struct command *c, char delim);

struct command* register_command(struct command_context *context,
		struct command *parent, char *name, command_handler_t handler,
		enum command_mode mode, char *help);

int unregister_command(struct command_context *context, char *name);
int unregister_all_commands(struct command_context *context);

void command_set_output_handler(struct command_context* context,
		command_output_handler_t output_handler, void *priv);

struct command_context* copy_command_context(struct command_context* context);

int command_context_mode(struct command_context *context, enum command_mode mode);

/**
 * Creates a new command context using the startup TCL provided.
 */
struct command_context* command_init(const char *startup_tcl);
int command_done(struct command_context *context);

void command_print(struct command_context *context, const char *format, ...)
		__attribute__ ((format (PRINTF_ATTRIBUTE_FORMAT, 2, 3)));
void command_print_sameline(struct command_context *context, const char *format, ...)
		__attribute__ ((format (PRINTF_ATTRIBUTE_FORMAT, 2, 3)));
int command_run_line(struct command_context *context, char *line);
int command_run_linef(struct command_context *context, const char *format, ...)
		__attribute__ ((format (PRINTF_ATTRIBUTE_FORMAT, 2, 3)));
void command_output_text(struct command_context *context, const char *data);

void process_jim_events(void);

#define		ERROR_COMMAND_CLOSE_CONNECTION		(-600)
#define		ERROR_COMMAND_SYNTAX_ERROR			(-601)
#define		ERROR_COMMAND_NOTFOUND				(-602)
#define		ERROR_COMMAND_ARGUMENT_INVALID		(-603)
#define		ERROR_COMMAND_ARGUMENT_OVERFLOW		(-604)
#define		ERROR_COMMAND_ARGUMENT_UNDERFLOW	(-605)

extern int fast_and_dangerous;

extern Jim_Interp *interp;

void register_jim(struct command_context *context, const char *name,
		Jim_CmdProc cmd, const char *help);

long jim_global_long(const char *variable);

int parse_ulong(const char *str, unsigned long *ul);
int parse_ullong(const char *str, unsigned long long *ul);

int parse_long(const char *str, long *ul);
int parse_llong(const char *str, long long *ul);

#define DECLARE_PARSE_WRAPPER(name, type) \
	int parse##name(const char *str, type *ul)

DECLARE_PARSE_WRAPPER(_uint, unsigned);
DECLARE_PARSE_WRAPPER(_u32, uint32_t);
DECLARE_PARSE_WRAPPER(_u16, uint16_t);
DECLARE_PARSE_WRAPPER(_u8, uint8_t);

DECLARE_PARSE_WRAPPER(_int, int);
DECLARE_PARSE_WRAPPER(_s32, int32_t);
DECLARE_PARSE_WRAPPER(_s16, int16_t);
DECLARE_PARSE_WRAPPER(_s8, int8_t);

/**
 * @brief parses the string @a in into @a out as a @a type, or prints
 * a command error and passes the error code to the caller.  If an error
 * does occur, the calling function will return the error code produced
 * by the parsing function (one of ERROR_COMMAND_ARGUMENT_*).
 *
 * This function may cause the calling function to return immediately,
 * so it should be used carefully to avoid leaking resources.  In most
 * situations, parsing should be completed in full before proceding
 * to allocate resources, and this strategy will most prevents leaks.
 */
#define COMMAND_PARSE_NUMBER(type, in, out) \
	do { \
		int retval = parse_##type(in, &(out)); \
		if (ERROR_OK != retval) { \
			command_print(CMD_CTX, stringify(out) \
				" option value ('%s') is not valid", in); \
			return retval; \
		} \
	} while (0)

void script_debug(Jim_Interp *interp, const char *cmd,
		unsigned argc, Jim_Obj *const *argv);

#endif /* COMMAND_H */
