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

enum command_mode
{
	COMMAND_EXEC,
	COMMAND_CONFIG,
	COMMAND_ANY,
};

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
	int (*output_handler)(struct command_context_s *context, const char* line);
	void *output_handler_priv;
} command_context_t;

typedef struct command_s
{
	char *name;
	struct command_s *parent;
	struct command_s *children;
	int (*handler)(struct command_context_s *context, char* name, char** args, int argc);
	enum command_mode mode;
	struct command_s *next;
} command_t;

extern command_t* register_command(command_context_t *context, command_t *parent, char *name, int (*handler)(struct command_context_s *context, char* name, char** args, int argc), enum command_mode mode, char *help);
extern int unregister_command(command_context_t *context, char *name);
extern int unregister_all_commands(command_context_t *context);
extern void command_set_output_handler(command_context_t* context, int (*output_handler)(struct command_context_s *context, const char* line), void *priv);
extern command_context_t* copy_command_context(command_context_t* context);
extern int command_context_mode(command_context_t *context, enum command_mode mode);
extern command_context_t* command_init(void);
extern int command_done(command_context_t *context);

extern void command_print(command_context_t *context, const char *format, ...)
		__attribute__ ((format (printf, 2, 3)));
extern void command_print_sameline(command_context_t *context, const char *format, ...)
		__attribute__ ((format (printf, 2, 3)));
extern int command_run_line(command_context_t *context, char *line);
extern int command_run_linef(command_context_t *context, const char *format, ...)
		__attribute__ ((format (printf, 2, 3)));
extern void command_output_text(command_context_t *context, const char *data);

extern void process_jim_events(void);

#define		ERROR_COMMAND_CLOSE_CONNECTION		(-600)
#define		ERROR_COMMAND_SYNTAX_ERROR			(-601)
#define		ERROR_COMMAND_NOTFOUND				(-602)

extern int fast_and_dangerous;

extern Jim_Interp *interp;

void register_jim(command_context_t *context, const char *name, int (*cmd)(Jim_Interp *interp, int argc, Jim_Obj *const *argv), const char *help);

long jim_global_long(const char *variable);

int parse_ulong(const char *str, unsigned long *ul);
int parse_ullong(const char *str, unsigned long long *ul);

int parse_long(const char *str, long *ul);
int parse_llong(const char *str, long long *ul);

#define DEFINE_PARSE_NUM_WRAP(name, type, max, functype, funcname) \
	static inline int parse_##name(const char *str, type *ul) \
	{ \
		functype n; \
		int retval = parse##funcname(str, &n); \
		*ul = n; \
		return n > (max) ? ERROR_COMMAND_SYNTAX_ERROR : retval; \
	}	

#define DEFINE_PARSE_ULONG(name, type, max) \
	DEFINE_PARSE_NUM_WRAP(name, type, max, unsigned long, _ulong)
DEFINE_PARSE_ULONG(uint, unsigned, UINT_MAX)
DEFINE_PARSE_ULONG(u32, uint32_t, UINT32_MAX)
DEFINE_PARSE_ULONG(u16, uint16_t, UINT16_MAX)
DEFINE_PARSE_ULONG(u8, uint8_t, UINT8_MAX)

#define DEFINE_PARSE_LONG(name, type, max) \
	DEFINE_PARSE_NUM_WRAP(name, type, max, long, _long)
DEFINE_PARSE_LONG(int, int, INT_MAX)
DEFINE_PARSE_LONG(s32, int32_t, INT32_MAX)
DEFINE_PARSE_LONG(s16, int16_t, INT16_MAX)
DEFINE_PARSE_LONG(s8, int8_t, INT8_MAX)

void script_debug(Jim_Interp *interp, const char *cmd, int argc, Jim_Obj *const *argv);

#endif /* COMMAND_H */
