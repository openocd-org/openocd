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
#include "interpreter.h"
#include "xsvf.h"
#include "target.h"
#include "flash.h"
#include "nand.h"
#include "pld.h"

#include "command.h"
#include "server.h"
#include "telnet_server.h"
#include "gdb_server.h"

#include <sys/time.h>
#include <sys/types.h>
#include <strings.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <errno.h>

#define JIM_EMBEDDED
#include "jim.h"


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
	 * continue startup regardless
	 */
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

	return ERROR_OK;
}


/* implementations of OpenOCD that uses multithreading needs to lock OpenOCD while calling
 * OpenOCD fn's. No-op in vanilla OpenOCD
 */
void lockBigLock()
{
}
void unlockBigLock()
{
}





Jim_Interp *interp;
command_context_t *active_cmd_ctx;

static void tcl_output(void *privData, const char *file, int line, 
		const char *function, const char *string)
{		
	Jim_Obj *tclOutput=(Jim_Obj *)privData;

	Jim_AppendString(interp, tclOutput, string, strlen(string));
}

/* try to execute as Jim command, otherwise fall back to standard command.

	Note that even if the Jim command caused an error, then we succeeded
	to execute it, hence this fn pretty much always returns ERROR_OK. 

 */
int jim_command(command_context_t *context, char *line)
{
	int retval=ERROR_OK;
	/* FIX!!!! in reality there is only one cmd_ctx handler, but consider
	what might happen here if there are multiple handlers w/reentrant callback
	fn's... shudder!  */
	active_cmd_ctx=context;
	int retcode=Jim_Eval(interp, line);
	
	const char *result;
	int reslen;
    result = Jim_GetString(Jim_GetResult(interp), &reslen);
    if (retcode == JIM_ERR) {
	    int len, i;
	
	    LOG_USER_N("Runtime error, file \"%s\", line %d:" JIM_NL,
	            interp->errorFileName, interp->errorLine);
	    LOG_USER_N("    %s" JIM_NL,
	            Jim_GetString(interp->result, NULL));
	    Jim_ListLength(interp, interp->stackTrace, &len);
	    for (i = 0; i < len; i+= 3) {
	        Jim_Obj *objPtr;
	        const char *proc, *file, *line;
	
	        Jim_ListIndex(interp, interp->stackTrace, i, &objPtr, JIM_NONE);
	        proc = Jim_GetString(objPtr, NULL);
	        Jim_ListIndex(interp, interp->stackTrace, i+1, &objPtr,
	                JIM_NONE);
	        file = Jim_GetString(objPtr, NULL);
	        Jim_ListIndex(interp, interp->stackTrace, i+2, &objPtr,
	                JIM_NONE);
	        line = Jim_GetString(objPtr, NULL);
	        LOG_USER_N("In procedure '%s' called at file \"%s\", line %s" JIM_NL,
	                proc, file, line);
	    }
    } else if (retcode == JIM_EXIT) {
    	// ignore.
        //exit(Jim_GetExitCode(interp));
    } else {
        if (reslen) {
        	int i;
        	char buff[256+1];
        	for (i=0; i<reslen; i+=256)
        	{
        		int chunk;
        		chunk=reslen-i;
        		if (chunk>256)
        			chunk=256;
        		strncpy(buff, result, chunk);
        		buff[chunk]=0; 
            	LOG_USER_N("%s", buff);
        	}
        	LOG_USER_N("%s", "\n");
        }
    }
	return retval;
}

static int startLoop=0;

static int
Jim_Command_openocd_ignore(Jim_Interp *interp, 
                                   int argc,
                                   Jim_Obj *const *argv,
                                   int ignore)
{
	int retval;
    char *cmd = (char*)Jim_GetString(argv[1], NULL);

	lockBigLock();
	
    Jim_Obj *tclOutput = Jim_NewStringObj(interp, "", 0);
    
    if (startLoop)
    {
    	// We don't know whether or not the telnet/gdb server is running...
    	target_call_timer_callbacks_now();
    }
	
	log_add_callback(tcl_output, tclOutput);
    retval=command_run_line_internal(active_cmd_ctx, cmd);
    
    if (startLoop)
    {
    	target_call_timer_callbacks_now();
    }
	log_remove_callback(tcl_output, tclOutput);
    
	Jim_SetResult(interp, tclOutput);
    unlockBigLock();
        
    return (ignore||(retval==ERROR_OK))?JIM_OK:JIM_ERR;
}

static int
Jim_Command_openocd(Jim_Interp *interp, 
                                   int argc,
                                   Jim_Obj *const *argv)
{
	return Jim_Command_openocd_ignore(interp, argc, argv, 1); 
}

static int
Jim_Command_openocd_throw(Jim_Interp *interp, 
                                   int argc,
                                   Jim_Obj *const *argv)
{
	return Jim_Command_openocd_ignore(interp, argc, argv, 0); 
}
  



/* find full path to file */
static int
Jim_Command_find(Jim_Interp *interp, 
                                   int argc,
                                   Jim_Obj *const *argv)
{
	if (argc!=2)
		return JIM_ERR;
	char *file = (char*)Jim_GetString(argv[1], NULL);
	char *full_path=find_file(file);
	if (full_path==NULL)
		return JIM_ERR;
    Jim_Obj *result = Jim_NewStringObj(interp, full_path, strlen(full_path));
    free(full_path);
    
	Jim_SetResult(interp, result);
	return JIM_OK;
}

static int
Jim_Command_echo(Jim_Interp *interp, 
                                   int argc,
                                   Jim_Obj *const *argv)
{
	if (argc!=2)
		return JIM_ERR;
	char *str = (char*)Jim_GetString(argv[1], NULL);
	LOG_USER("%s", str);
	return JIM_OK;
}

void initJim(void)
{
    Jim_InitEmbedded();
  
    /* Create an interpreter */
    interp = Jim_CreateInterp();
    /* Add all the Jim core commands */
    Jim_RegisterCoreCommands(interp);
    Jim_CreateCommand(interp, "openocd", Jim_Command_openocd, NULL, NULL);
    Jim_CreateCommand(interp, "openocd_throw", Jim_Command_openocd_throw, NULL, NULL);
    Jim_CreateCommand(interp, "find", Jim_Command_find, NULL, NULL);
    Jim_CreateCommand(interp, "echo", Jim_Command_echo, NULL, NULL);
}

int main(int argc, char *argv[])
{
	initJim();
	
	/* initialize commandline interface */
	command_context_t *cmd_ctx, *cfg_cmd_ctx;
	cmd_ctx = command_init();

	register_command(cmd_ctx, NULL, "version", handle_version_command,
					 COMMAND_EXEC, "show OpenOCD version");
	register_command(cmd_ctx, NULL, "daemon_startup", handle_daemon_startup_command, COMMAND_CONFIG, 
			"deprecated - use \"init\" and \"reset\" at end of startup script instead");
	
	/* register subsystem commands */
	server_register_commands(cmd_ctx);
	telnet_register_commands(cmd_ctx);
	gdb_register_commands(cmd_ctx);
	log_register_commands(cmd_ctx);
	jtag_register_commands(cmd_ctx);
	interpreter_register_commands(cmd_ctx);
	xsvf_register_commands(cmd_ctx);
	target_register_commands(cmd_ctx);
	flash_register_commands(cmd_ctx);
	nand_register_commands(cmd_ctx);
	pld_register_commands(cmd_ctx);
	
	if (log_init(cmd_ctx) != ERROR_OK)
		return EXIT_FAILURE;
	LOG_DEBUG("log init complete");

	LOG_OUTPUT( OPENOCD_VERSION "\n" );
	
	
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

	register_command(cmd_ctx, NULL, "init", handle_init_command,
					 COMMAND_ANY, "initializes target and servers - nop on subsequent invocations");

	cfg_cmd_ctx = copy_command_context(cmd_ctx);
	cfg_cmd_ctx->mode = COMMAND_CONFIG;
	command_set_output_handler(cfg_cmd_ctx, configuration_output_handler, NULL);
	
	if (parse_cmdline_args(cfg_cmd_ctx, argc, argv) != ERROR_OK)
		return EXIT_FAILURE;

    Jim_Eval(interp, "source [find tcl/commands.tcl]");

	if (parse_config_file(cfg_cmd_ctx) != ERROR_OK)
		return EXIT_FAILURE;
	
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

