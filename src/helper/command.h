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

struct command_context_s;

/// The type signature for command context's output handler.
typedef int (*command_output_handler_t)(struct command_context_s *context,
				const char* line);

typedef struct command_context_s
{
	enum command_mode mode;
	struct command_s *commands;
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
} command_context_t;


/**
 * Command handlers may be defined with more parameters than the base
 * set provided by command.c.  This macro uses C99 magic to allow
 * defining all such derivative types using this macro.
 */
#define __COMMAND_HANDLER(name, extra...) \
		int name(struct command_context_s *cmd_ctx, \
				char *cmd, char **args, int argc, ##extra)

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
		name(cmd_ctx, cmd, args, argc, ##extra)

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


/// The type signature for commands' handler functions.
typedef __COMMAND_HANDLER((*command_handler_t));

typedef struct command_s
{
	char *name;
	struct command_s *parent;
	struct command_s *children;
	command_handler_t handler;
	enum command_mode mode;
	struct command_s *next;
} command_t;

/**
 * @param c The command to be named.
 * @param delim The character to place between command names.
 * @returns A malloc'd string containing the full command name,
 * which may include one or more ancestor components.  Multiple names
 * are separated by single spaces.  The caller must free() the string
 * when done with it.
 */
char *command_name(struct command_s *c, char delim);

command_t* register_command(command_context_t *context,
		command_t *parent, char *name, command_handler_t handler,
		enum command_mode mode, char *help);

int unregister_command(command_context_t *context, char *name);
int unregister_all_commands(command_context_t *context);

void command_set_output_handler(command_context_t* context,
		command_output_handler_t output_handler, void *priv);

command_context_t* copy_command_context(command_context_t* context);

int command_context_mode(command_context_t *context, enum command_mode mode);

command_context_t* command_init(void);
int command_done(command_context_t *context);

void command_print(command_context_t *context, const char *format, ...)
		__attribute__ ((format (PRINTF_ATTRIBUTE_FORMAT, 2, 3)));
void command_print_sameline(command_context_t *context, const char *format, ...)
		__attribute__ ((format (PRINTF_ATTRIBUTE_FORMAT, 2, 3)));
int command_run_line(command_context_t *context, char *line);
int command_run_linef(command_context_t *context, const char *format, ...)
		__attribute__ ((format (PRINTF_ATTRIBUTE_FORMAT, 2, 3)));
void command_output_text(command_context_t *context, const char *data);

void process_jim_events(void);

#define		ERROR_COMMAND_CLOSE_CONNECTION		(-600)
#define		ERROR_COMMAND_SYNTAX_ERROR			(-601)
#define		ERROR_COMMAND_NOTFOUND				(-602)
#define		ERROR_COMMAND_ARGUMENT_INVALID		(-603)
#define		ERROR_COMMAND_ARGUMENT_OVERFLOW		(-604)
#define		ERROR_COMMAND_ARGUMENT_UNDERFLOW	(-605)

extern int fast_and_dangerous;

extern Jim_Interp *interp;

void register_jim(command_context_t *context, const char *name, int (*cmd)(Jim_Interp *interp, int argc, Jim_Obj *const *argv), const char *help);

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
			command_print(cmd_ctx, stringify(out) \
				" option value ('%s') is not valid", in); \
			return retval; \
		} \
	} while (0)

void script_debug(Jim_Interp *interp, const char *cmd,
		unsigned argc, Jim_Obj *const *argv);

#endif /* COMMAND_H */
