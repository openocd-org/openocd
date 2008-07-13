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

#define OPENOCD_VERSION "Open On-Chip Debugger " VERSION " (" PKGBLDDATE ") svn:" PKGBLDREV

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "log.h"
#include "types.h"
#include "jtag.h"
#include "configuration.h"
#include "xsvf.h"
#include "target.h"
#include "flash.h"
#include "nand.h"
#include "pld.h"

#include "command.h"
#include "server.h"
#include "telnet_server.h"
#include "gdb_server.h"
#include "tcl_server.h"

#include <sys/time.h>
#include <sys/types.h>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#ifdef _WIN32
#include <malloc.h>
#else
#include <alloca.h>
#endif

#ifdef __ECOS
/* Jim is provied by eCos */
#include <cyg/jimtcl/jim.h>
#else
#define JIM_EMBEDDED
#include "jim.h"
#endif

#include "replacements.h"


/* Give TELNET a way to find out what version this is */
int handle_version_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	command_print(cmd_ctx, OPENOCD_VERSION);

	return ERROR_OK;
}

static int daemon_startup = 0;

int handle_daemon_startup_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc==0)
		return ERROR_OK;
	if (argc > 1 )
		return ERROR_COMMAND_SYNTAX_ERROR;
	
	daemon_startup = strcmp("reset", args[0])==0;
	
	command_print(cmd_ctx, OPENOCD_VERSION);

	return ERROR_OK;
}

void exit_handler(void)
{
	/* close JTAG interface */
	if (jtag && jtag->quit)
		jtag->quit();
}

/* OpenOCD can't really handle failure of this command. Patches welcome! :-) */
int handle_init_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int retval;
	static int initialized=0;
	if (initialized)
		return ERROR_OK;
	
	initialized=1;
	
	command_set_output_handler(cmd_ctx, configuration_output_handler, NULL);

	atexit(exit_handler);
	
	if (target_init(cmd_ctx) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("target init complete");

	if ((retval=jtag_interface_init(cmd_ctx)) != ERROR_OK)
	{
		/* we must be able to set up the jtag interface */
		return retval;
	}
	LOG_DEBUG("jtag interface init complete");

	/* Try to initialize & examine the JTAG chain at this point, but
	 * continue startup regardless */
	if (jtag_init(cmd_ctx) == ERROR_OK)
	{
		LOG_DEBUG("jtag init complete");
		if (target_examine(cmd_ctx) == ERROR_OK)
		{
			LOG_DEBUG("jtag examine complete");
		}
	}
	
	if (flash_init_drivers(cmd_ctx) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("flash init complete");

	if (nand_init(cmd_ctx) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("NAND init complete");

	if (pld_init(cmd_ctx) != ERROR_OK)
		return ERROR_FAIL;
	LOG_DEBUG("pld init complete");

	/* initialize tcp server */
	server_init();

	/* initialize telnet subsystem */
	telnet_init("Open On-Chip Debugger");
	gdb_init();
	tcl_init(); /* allows tcl to just connect without going thru telnet */

	return ERROR_OK;
}

Jim_Interp *interp;
command_context_t *active_cmd_ctx;

static int new_int_array_element(Jim_Interp * interp, const char *varname, int idx, u32 val)
{
	char *namebuf;
	Jim_Obj *nameObjPtr, *valObjPtr;
	int result;

	namebuf = alloc_printf("%s(%d)", varname, idx);
	if (!namebuf)
		return JIM_ERR;
	
	nameObjPtr = Jim_NewStringObj(interp, namebuf, -1);
	valObjPtr = Jim_NewIntObj(interp, val);
	if (!nameObjPtr || !valObjPtr)
	{
		free(namebuf);
		return JIM_ERR;
	}

	Jim_IncrRefCount(nameObjPtr);
	Jim_IncrRefCount(valObjPtr);
	result = Jim_SetVariable(interp, nameObjPtr, valObjPtr);
	Jim_DecrRefCount(interp, nameObjPtr);
	Jim_DecrRefCount(interp, valObjPtr);
	free(namebuf);
	/* printf("%s(%d) <= 0%08x\n", varname, idx, val); */
	return result;
}

static int Jim_Command_mem2array(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	target_t *target;
	long l;
	u32 width;
	u32 len;
	u32 addr;
	u32 count;
	u32 v;
	const char *varname;
	u8 buffer[4096];
	int  i, n, e, retval;

	/* argv[1] = name of array to receive the data
	 * argv[2] = desired width
	 * argv[3] = memory address 
	 * argv[4] = count of times to read
	 */
	if (argc != 5) {
		Jim_WrongNumArgs(interp, 1, argv, "varname width addr nelems");
		return JIM_ERR;
	}
	varname = Jim_GetString(argv[1], &len);
	/* given "foo" get space for worse case "foo(%d)" .. add 20 */

	e = Jim_GetLong(interp, argv[2], &l);
	width = l;
	if (e != JIM_OK) {
		return e;
	}
	
	e = Jim_GetLong(interp, argv[3], &l);
	addr = l;
	if (e != JIM_OK) {
		return e;
	}
	e = Jim_GetLong(interp, argv[4], &l);
	len = l;
	if (e != JIM_OK) {
		return e;
	}
	switch (width) {
		case 8:
			width = 1;
			break;
		case 16:
			width = 2;
			break;
		case 32:
			width = 4;
			break;
		default:
			Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
			Jim_AppendStrings( interp, Jim_GetResult(interp), "Invalid width param, must be 8/16/32", NULL );
			return JIM_ERR;
	}
	if (len == 0) {
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		Jim_AppendStrings(interp, Jim_GetResult(interp), "mem2array: zero width read?", NULL);
		return JIM_ERR;
	}
	if ((addr + (len * width)) < addr) {
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		Jim_AppendStrings(interp, Jim_GetResult(interp), "mem2array: addr + len - wraps to zero?", NULL);
		return JIM_ERR;
	}
	/* absurd transfer size? */
	if (len > 65536) {
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		Jim_AppendStrings(interp, Jim_GetResult(interp), "mem2array: absurd > 64K item request", NULL);
		return JIM_ERR;
	}		
		
	if ((width == 1) ||
		((width == 2) && ((addr & 1) == 0)) ||
		((width == 4) && ((addr & 3) == 0))) {
		/* all is well */
	} else {
		char buf[100];
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		sprintf(buf, "mem2array address: 0x%08x is not aligned for %d byte reads", addr, width); 
		Jim_AppendStrings(interp, Jim_GetResult(interp), buf , NULL);
		return JIM_ERR;
	}

	target = get_current_target(active_cmd_ctx);
	
	/* Transfer loop */

	/* index counter */
	n = 0;
	/* assume ok */
	e = JIM_OK;
	while (len) {
		/* Slurp... in buffer size chunks */
		
		count = len; /* in objects.. */
		if (count > (sizeof(buffer)/width)) {
			count = (sizeof(buffer)/width);
		}
		
		retval = target->type->read_memory( target, addr, width, count, buffer );
		if (retval != ERROR_OK) {
			/* BOO !*/
			LOG_ERROR("mem2array: Read @ 0x%08x, w=%d, cnt=%d, failed", addr, width, count);
			Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
			Jim_AppendStrings(interp, Jim_GetResult(interp), "mem2array: cannot read memory", NULL);
			e = JIM_ERR;
			len = 0;
		} else {
			v = 0; /* shut up gcc */
			for (i = 0 ;i < count ;i++, n++) {
				switch (width) {
					case 4:
						v = target_buffer_get_u32(target, &buffer[i*width]);
						break;
					case 2:
						v = target_buffer_get_u16(target, &buffer[i*width]);
						break;
					case 1:
						v = buffer[i] & 0x0ff;
						break;
				}
				new_int_array_element(interp, varname, n, v);
			}
			len -= count;
		}
	}
	
	Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));

	return JIM_OK;
}

static int get_int_array_element(Jim_Interp * interp, const char *varname, int idx, u32 *val)
{
	char *namebuf;
	Jim_Obj *nameObjPtr, *valObjPtr;
	int result;
	long l;

	namebuf = alloc_printf("%s(%d)", varname, idx);
	if (!namebuf)
		return JIM_ERR;

	nameObjPtr = Jim_NewStringObj(interp, namebuf, -1);
	if (!nameObjPtr)
	{
		free(namebuf);
		return JIM_ERR;
	}

	Jim_IncrRefCount(nameObjPtr);
	valObjPtr = Jim_GetVariable(interp, nameObjPtr, JIM_ERRMSG);
	Jim_DecrRefCount(interp, nameObjPtr);
	free(namebuf);
	if (valObjPtr == NULL)
		return JIM_ERR;

	result = Jim_GetLong(interp, valObjPtr, &l);
	/* printf("%s(%d) => 0%08x\n", varname, idx, val); */
	*val = l;
	return result;
}

static int Jim_Command_array2mem(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	target_t *target;
	long l;
	u32 width;
	u32 len;
	u32 addr;
	u32 count;
	u32 v;
	const char *varname;
	u8 buffer[4096];
	int  i, n, e, retval;

	/* argv[1] = name of array to get the data
	 * argv[2] = desired width
	 * argv[3] = memory address 
	 * argv[4] = count to write
	 */
	if (argc != 5) {
		Jim_WrongNumArgs(interp, 1, argv, "varname width addr nelems");
		return JIM_ERR;
	}
	varname = Jim_GetString(argv[1], &len);
	/* given "foo" get space for worse case "foo(%d)" .. add 20 */

	e = Jim_GetLong(interp, argv[2], &l);
	width = l;
	if (e != JIM_OK) {
		return e;
	}
	
	e = Jim_GetLong(interp, argv[3], &l);
	addr = l;
	if (e != JIM_OK) {
		return e;
	}
	e = Jim_GetLong(interp, argv[4], &l);
	len = l;
	if (e != JIM_OK) {
		return e;
	}
	switch (width) {
		case 8:
			width = 1;
			break;
		case 16:
			width = 2;
			break;
		case 32:
			width = 4;
			break;
		default:
			Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
			Jim_AppendStrings( interp, Jim_GetResult(interp), "Invalid width param, must be 8/16/32", NULL );
			return JIM_ERR;
	}
	if (len == 0) {
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		Jim_AppendStrings(interp, Jim_GetResult(interp), "array2mem: zero width read?", NULL);
		return JIM_ERR;
	}
	if ((addr + (len * width)) < addr) {
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		Jim_AppendStrings(interp, Jim_GetResult(interp), "array2mem: addr + len - wraps to zero?", NULL);
		return JIM_ERR;
	}
	/* absurd transfer size? */
	if (len > 65536) {
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		Jim_AppendStrings(interp, Jim_GetResult(interp), "array2mem: absurd > 64K item request", NULL);
		return JIM_ERR;
	}		
		
	if ((width == 1) ||
		((width == 2) && ((addr & 1) == 0)) ||
		((width == 4) && ((addr & 3) == 0))) {
		/* all is well */
	} else {
		char buf[100];
		Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
		sprintf(buf, "array2mem address: 0x%08x is not aligned for %d byte reads", addr, width); 
		Jim_AppendStrings(interp, Jim_GetResult(interp), buf , NULL);
		return JIM_ERR;
	}

	target = get_current_target(active_cmd_ctx);
	
	/* Transfer loop */

	/* index counter */
	n = 0;
	/* assume ok */
	e = JIM_OK;
	while (len) {
		/* Slurp... in buffer size chunks */
		
		count = len; /* in objects.. */
		if (count > (sizeof(buffer)/width)) {
			count = (sizeof(buffer)/width);
		}

		v = 0; /* shut up gcc */
		for (i = 0 ;i < count ;i++, n++) {
			get_int_array_element(interp, varname, n, &v);
			switch (width) {
			case 4:
				target_buffer_set_u32(target, &buffer[i*width], v);
				break;
			case 2:
				target_buffer_set_u16(target, &buffer[i*width], v);
				break;
			case 1:
				buffer[i] = v & 0x0ff;
				break;
			}
		}
		len -= count;

		retval = target->type->write_memory(target, addr, width, count, buffer);
		if (retval != ERROR_OK) {
			/* BOO !*/
			LOG_ERROR("array2mem: Write @ 0x%08x, w=%d, cnt=%d, failed", addr, width, count);
			Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));
			Jim_AppendStrings(interp, Jim_GetResult(interp), "mem2array: cannot read memory", NULL);
			e = JIM_ERR;
			len = 0;
		}
	}
	
	Jim_SetResult(interp, Jim_NewEmptyStringObj(interp));

	return JIM_OK;
}

static void tcl_output(void *privData, const char *file, int line, const char *function, const char *string)
{		
	Jim_Obj *tclOutput=(Jim_Obj *)privData;

	Jim_AppendString(interp, tclOutput, string, strlen(string));
}

/* try to execute as Jim command, otherwise fall back to standard command.
 * Note that even if the Jim command caused an error, then we succeeded
 * to execute it, hence this fn pretty much always returns ERROR_OK. */
int jim_command(command_context_t *context, char *line)
{
	int retval=ERROR_OK;
	int retcode;

	active_cmd_ctx = context;
	retcode = Jim_Eval(interp, line);
	
	if (retcode == JIM_ERR) {
		Jim_PrintErrorMessage(interp);
	    long t;
	    Jim_Obj *openocd_result=Jim_GetVariableStr(interp, "openocd_result", JIM_ERRMSG);
	    if (openocd_result)
	    {
		    if (Jim_GetLong(interp, openocd_result, &t)==JIM_OK)
		    {
		    	return t;
		    }
		}
	    return ERROR_FAIL;
	} 
	const char *result;
	int reslen;
	result = Jim_GetString(Jim_GetResult(interp), &reslen);
		
	if (retcode == JIM_EXIT) {
		/* ignore. */
	/* exit(Jim_GetExitCode(interp)); */
	} else {
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

int startLoop = 0;

static int Jim_Command_openocd_ignore(Jim_Interp *interp, int argc, Jim_Obj *const *argv, int ignore)
{
	int retval;
	char *cmd = (char*)Jim_GetString(argv[1], NULL);
	
	Jim_Obj *tclOutput = Jim_NewStringObj(interp, "", 0);
	
	if (startLoop)
	{
		/* We don't know whether or not the telnet/gdb server is running... */
		target_call_timer_callbacks_now();
	}
	
	log_add_callback(tcl_output, tclOutput);
	retval=command_run_line_internal(active_cmd_ctx, cmd);

	/* we need to be able to get at the retval, so we store in a variable
	 */
	Jim_Obj *resultvar=Jim_NewIntObj(interp, retval);
	Jim_IncrRefCount(resultvar);
	Jim_SetGlobalVariableStr(interp, "openocd_result", resultvar);
	Jim_DecrRefCount(interp, resultvar);
	
	if (startLoop)
	{
		target_call_timer_callbacks_now();
	}
	log_remove_callback(tcl_output, tclOutput);
	
	Jim_SetResult(interp, tclOutput);
	
	return (ignore||(retval==ERROR_OK))?JIM_OK:JIM_ERR;
}

static int Jim_Command_openocd(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	return Jim_Command_openocd_ignore(interp, argc, argv, 1); 
}

static int Jim_Command_openocd_throw(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	return Jim_Command_openocd_ignore(interp, argc, argv, 0); 
}

/* find full path to file */
static int Jim_Command_find(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	if (argc != 2)
		return JIM_ERR;
	char *file = (char*)Jim_GetString(argv[1], NULL);
	char *full_path = find_file(file);
	if (full_path == NULL)
		return JIM_ERR;
	Jim_Obj *result = Jim_NewStringObj(interp, full_path, strlen(full_path));
	free(full_path);
	
	Jim_SetResult(interp, result);
	return JIM_OK;
}

static int Jim_Command_echo(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	if (argc != 2)
		return JIM_ERR;
	char *str = (char*)Jim_GetString(argv[1], NULL);
	LOG_USER("%s", str);
	return JIM_OK;
}

static size_t openocd_jim_fwrite(const void *_ptr, size_t size, size_t n, void *cookie)
{
	size_t nbytes;
	const char *ptr;

	/* make it a char easier to read code */
	ptr = _ptr;

	nbytes = size * n;
	if (nbytes == 0) {
		return 0;
	}

	if (!active_cmd_ctx) {
		/* TODO: Where should this go? */		
		return n;
	}

	/* do we have to chunk it? */
	if (ptr[nbytes] == 0) {
		/* no it is a C style string */
		command_output_text(active_cmd_ctx, ptr);
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
		command_output_text(active_cmd_ctx, chunk);
		ptr += x;
		nbytes -= x;
	}
	
	return n;
}

static size_t openocd_jim_fread(void *ptr, size_t size, size_t n, void *cookie )
{
	/* TCL wants to read... tell him no */
	return 0;
}

static int openocd_jim_vfprintf(void *cookie, const char *fmt, va_list ap)
{
	char *cp;
	int n;
	
	n = -1;
	if (active_cmd_ctx) {
		cp = alloc_vprintf(fmt, ap);
		if (cp) {
			command_output_text(active_cmd_ctx, cp);
			n = strlen(cp);
			free(cp);
		}
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

void add_jim(const char *name, int (*cmd)(Jim_Interp *interp, int argc, Jim_Obj *const *argv), const char *help)
{
	Jim_CreateCommand(interp, name, cmd, NULL, NULL);
	
	/* FIX!!! it would be prettier to invoke add_help_text... 
	accumulate help text in Tcl helptext list.  */
    Jim_Obj *helptext=Jim_GetGlobalVariableStr(interp, "ocd_helptext", JIM_ERRMSG);
	Jim_Obj *cmd_entry=Jim_NewListObj(interp, NULL, 0);
	
	Jim_Obj *cmd_list=Jim_NewListObj(interp, NULL, 0);
	Jim_ListAppendElement(interp, cmd_list, Jim_NewStringObj(interp, name, -1));
	
	Jim_ListAppendElement(interp, cmd_entry, cmd_list);
	Jim_ListAppendElement(interp, cmd_entry, Jim_NewStringObj(interp, help, -1));
	Jim_ListAppendElement(interp, helptext, cmd_entry);
}

extern unsigned const char startup_tcl[];

void initJim(void)
{	
	Jim_CreateCommand(interp, "openocd", Jim_Command_openocd, NULL, NULL);
	Jim_CreateCommand(interp, "openocd_throw", Jim_Command_openocd_throw, NULL, NULL);
	Jim_CreateCommand(interp, "find", Jim_Command_find, NULL, NULL);
	Jim_CreateCommand(interp, "echo", Jim_Command_echo, NULL, NULL);
	Jim_CreateCommand(interp, "mem2array", Jim_Command_mem2array, NULL, NULL );
	Jim_CreateCommand(interp, "array2mem", Jim_Command_array2mem, NULL, NULL );

	/* Set Jim's STDIO */
	interp->cookie_stdin = NULL;
	interp->cookie_stdout = NULL;
	interp->cookie_stderr = NULL;
	interp->cb_fwrite = openocd_jim_fwrite;
	interp->cb_fread = openocd_jim_fread ;
	interp->cb_vfprintf = openocd_jim_vfprintf;
	interp->cb_fflush = openocd_jim_fflush;
	interp->cb_fgets = openocd_jim_fgets;
	
	add_default_dirs();
	
	if (Jim_Eval(interp, startup_tcl)==JIM_ERR)
	{
		LOG_ERROR("Failed to run startup.tcl (embedded into OpenOCD compile time)");
		Jim_PrintErrorMessage(interp);
		exit(-1);
	}
}

int handle_script_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	
	/* Run a tcl script file */
	return command_run_linef(cmd_ctx, "source [find {%s}]", args[0]);
}

command_context_t *setup_command_handler(void)
{
	command_context_t *cmd_ctx;
	
	cmd_ctx = command_init();
	
	register_command(cmd_ctx, NULL, "version", handle_version_command,
					 COMMAND_EXEC, "show OpenOCD version");
	register_command(cmd_ctx, NULL, "daemon_startup", handle_daemon_startup_command, COMMAND_CONFIG, 
			"deprecated - use \"init\" and \"reset\" at end of startup script instead");
	
	/* register subsystem commands */
	server_register_commands(cmd_ctx);
	telnet_register_commands(cmd_ctx);
	gdb_register_commands(cmd_ctx);
	tcl_register_commands(cmd_ctx); /* tcl server commands */
	log_register_commands(cmd_ctx);
	jtag_register_commands(cmd_ctx);
	register_command(cmd_ctx, NULL, "script", handle_script_command, COMMAND_ANY, "execute commands from <file>");
	xsvf_register_commands(cmd_ctx);
	target_register_commands(cmd_ctx);
	flash_register_commands(cmd_ctx);
	nand_register_commands(cmd_ctx);
	pld_register_commands(cmd_ctx);
	
	if (log_init(cmd_ctx) != ERROR_OK)
	{
		exit(-1);
	}
	LOG_DEBUG("log init complete");

	LOG_OUTPUT( OPENOCD_VERSION "\n" );
	
	
	register_command(cmd_ctx, NULL, "init", handle_init_command,
					 COMMAND_ANY, "initializes target and servers - nop on subsequent invocations");

	return cmd_ctx;
}

/* normally this is the main() function entry, but if OpenOCD is linked
 * into application, then this fn will not be invoked, but rather that
 * application will have it's own implementation of main(). */
int openocd_main(int argc, char *argv[])
{
#ifdef JIM_EMBEDDED
	Jim_InitEmbedded();
	/* Create an interpreter */
	interp = Jim_CreateInterp();
	/* Add all the Jim core commands */
	Jim_RegisterCoreCommands(interp);
#endif
	
	initJim();
	
	/* initialize commandline interface */
	command_context_t *cmd_ctx;
	cmd_ctx=setup_command_handler();
	
	/* DANGER!!! make sure that the line below does not appear in a patch, do not remove */
	/* DANGER!!! make sure that the line below does not appear in a patch, do not remove */
	/* DANGER!!! make sure that the line below does not appear in a patch, do not remove */
	/* DANGER!!! make sure that the line below does not appear in a patch, do not remove */
	/* DANGER!!! make sure that the line below does not appear in a patch, do not remove */
	LOG_OUTPUT( "$URL$\n");
	/* DANGER!!! make sure that the line above does not appear in a patch, do not remove */
	/* DANGER!!! make sure that the line above does not appear in a patch, do not remove */
	/* DANGER!!! make sure that the line above does not appear in a patch, do not remove */
	/* DANGER!!! make sure that the line above does not appear in a patch, do not remove */
	/* DANGER!!! make sure that the line above does not appear in a patch, do not remove */

	command_context_t *cfg_cmd_ctx;
	cfg_cmd_ctx = copy_command_context(cmd_ctx);
	cfg_cmd_ctx->mode = COMMAND_CONFIG;
	command_set_output_handler(cfg_cmd_ctx, configuration_output_handler, NULL);
	
	active_cmd_ctx=cfg_cmd_ctx;
	

	if (parse_cmdline_args(cfg_cmd_ctx, argc, argv) != ERROR_OK)
		return EXIT_FAILURE;
	
	if (parse_config_file(cfg_cmd_ctx) != ERROR_OK)
		return EXIT_FAILURE;
	
	active_cmd_ctx=cmd_ctx;
	
	command_done(cfg_cmd_ctx);

	if (command_run_line(cmd_ctx, "init")!=ERROR_OK)
		return EXIT_FAILURE;
	
	if (daemon_startup)
		command_run_line(cmd_ctx, "reset");
	
	startLoop=1;

	/* handle network connections */
	server_loop(cmd_ctx);

	/* shut server down */
	server_quit();

	unregister_all_commands(cmd_ctx);
	
	/* free commandline interface */
	command_done(cmd_ctx);

	return EXIT_SUCCESS;
}
