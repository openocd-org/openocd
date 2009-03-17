/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
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

#include "log.h"
#include "configuration.h"
#include "time_support.h"
#include "command.h"
#include "server.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <stdarg.h>

#define PRINT_MEM() 0
#if PRINT_MEM()
#include <malloc.h>
#endif

int debug_level = -1;

static FILE* log_output;
static log_callback_t *log_callbacks = NULL;

static long long last_time;
static long long current_time;

static long long start;

static char *log_strings[5] =
{
	"User : ",
	"Error: ",
	"Warn : ",  /* want a space after each colon, all same width, colons aligned */
	"Info : ",
	"Debug: "
};


static int count = 0;

/* The log_puts() serves to somewhat different goals:
 *
 * - logging
 * - feeding low-level info to the user in GDB or Telnet
 *
 * The latter dictates that strings without newline are not logged, lest there
 * will be *MANY log lines when sending one char at the time(e.g.
 * target_request.c).
 *
 */
static void log_puts(enum log_levels level, const char *file, int line, const char *function, const char *string)
{
	char *f;
	if (level == LOG_LVL_OUTPUT)
	{
		/* do not prepend any headers, just print out what we were given and return */
		fputs(string, log_output);
		fflush(log_output);
		return;
	}

	f = strrchr(file, '/');
	if (f != NULL)
		file = f + 1;

	if (strchr(string, '\n')!=NULL)
	{
		if (debug_level >= LOG_LVL_DEBUG)
		{
			/* print with count and time information */
			int t=(int)(timeval_ms()-start);
#if PRINT_MEM()
			struct mallinfo info;
			info = mallinfo();
#endif
			fprintf(log_output, "%s%d %d %s:%d %s()"
#if PRINT_MEM()
					" %d"
#endif
					": %s", log_strings[level+1], count, t, file, line, function,
#if PRINT_MEM()
					info.fordblks,
#endif
					string);
		}
		else if(server_use_pipes == 0)
		{
			/* if we are using gdb through pipes then we do not want any output
			 * to the pipe otherwise we get repeated strings */
			if (strcmp(string, "\n") != 0)
			{
				/* print human readable output - but skip empty lines */
				fprintf(log_output, "%s%s",
						(level > LOG_LVL_USER)?log_strings[level+1]:"", string);
			}
		}
	} else
	{
		/* only entire lines are logged. Otherwise it's
		 * single chars intended for the log callbacks. */
	}

	fflush(log_output);

	/* Never forward LOG_LVL_DEBUG, too verbose and they can be found in the log if need be */
	if (level <= LOG_LVL_INFO)
	{
		log_callback_t *cb, *next;
		cb = log_callbacks;
		/* DANGER!!!! the log callback can remove itself!!!! */
		while (cb)
		{
			next=cb->next;
			cb->fn(cb->priv, file, line, function, string);
			cb=next;
		}
	}
}

void log_printf(enum log_levels level, const char *file, int line, const char *function, const char *format, ...)
{
	char *string;
	va_list ap;

	count++;
	if (level > debug_level)
		return;

	va_start(ap, format);

	string = alloc_vprintf(format, ap);
	if (string != NULL)
	{
		log_puts(level, file, line, function, string);
		free(string);
	}

	va_end(ap);
}

void log_printf_lf(enum log_levels level, const char *file, int line, const char *function, const char *format, ...)
{
	char *string;
	va_list ap;

	count++;
	if (level > debug_level)
		return;

	va_start(ap, format);

	string = alloc_vprintf(format, ap);
	if (string != NULL)
	{
		strcat(string, "\n"); /* alloc_vprintf guaranteed the buffer to be at least one char longer */
		log_puts(level, file, line, function, string);
		free(string);
	}

	va_end(ap);
}

/* change the current debug level on the fly
 * 0: only ERRORS
 * 1: + WARNINGS
 * 2: + INFORMATIONAL MSGS
 * 3: + DEBUG MSGS
 */
int handle_debug_level_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc == 0)
		command_print(cmd_ctx, "debug_level: %i", debug_level);

	if (argc > 0)
		debug_level = strtoul(args[0], NULL, 0);

	if (debug_level < 0)
		debug_level = 0;

	if (debug_level > 3)
		debug_level = 3;

	if (debug_level >= LOG_LVL_DEBUG && server_use_pipes == 1)
	{
		/* if we are enabling debug info then we need to write to a log file
		 * otherwise the pipe will get full and cause issues with gdb */
		FILE* file = fopen("openocd.log", "w");
		if (file)
		{
			log_output = file;
			LOG_WARNING("enabling log output as we are using pipes");
		}
	}

	return ERROR_OK;
}

int handle_log_output_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc == 1)
	{
		FILE* file = fopen(args[0], "w");

		if (file)
		{
			log_output = file;
		}
	}

	return ERROR_OK;
}

int log_register_commands(struct command_context_s *cmd_ctx)
{
	start = timeval_ms();
	register_command(cmd_ctx, NULL, "log_output", handle_log_output_command,
		COMMAND_ANY, "redirect logging to <file> (default: stderr)");
	register_command(cmd_ctx, NULL, "debug_level", handle_debug_level_command,
		COMMAND_ANY, "adjust debug level <0-3>");

	return ERROR_OK;
}

int log_init(struct command_context_s *cmd_ctx)
{
	/* set defaults for daemon configuration, if not set by cmdline or cfgfile */
	if (debug_level == -1)
		debug_level = LOG_LVL_INFO;

	if (log_output == NULL)
	{
		log_output = stderr;
	}

	start=last_time=timeval_ms();

	return ERROR_OK;
}

int set_log_output(struct command_context_s *cmd_ctx, FILE *output)
{
	log_output = output;
	return ERROR_OK;
}

/* add/remove log callback handler */
int log_add_callback(log_callback_fn fn, void *priv)
{
	log_callback_t *cb;

	/* prevent the same callback to be registered more than once, just for sure */
	for (cb = log_callbacks; cb; cb = cb->next)
	{
		if (cb->fn == fn && cb->priv == priv)
			return ERROR_INVALID_ARGUMENTS;
	}

	/* alloc memory, it is safe just to return in case of an error, no need for the caller to check this */
	if ((cb = malloc(sizeof(log_callback_t))) == NULL)
		return ERROR_BUF_TOO_SMALL;

	/* add item to the beginning of the linked list */
	cb->fn = fn;
	cb->priv = priv;
	cb->next = log_callbacks;
	log_callbacks = cb;

	return ERROR_OK;
}

int log_remove_callback(log_callback_fn fn, void *priv)
{
	log_callback_t *cb, **p;

	for (p = &log_callbacks; (cb = *p); p = &(*p)->next)
	{
		if (cb->fn == fn && cb->priv == priv)
		{
			*p = cb->next;
			free(cb);
			return ERROR_OK;
		}
	}

	/* no such item */
	return ERROR_INVALID_ARGUMENTS;
}

/* return allocated string w/printf() result */
char *alloc_vprintf(const char *fmt, va_list ap)
{
	/* no buffer at the beginning, force realloc to do the job */
	char *string = NULL;

	/* start with buffer size suitable for typical messages */
	int size = 128;

	for (;;)
	{
		char *t = string;
		va_list ap_copy;
		int ret;
		string = realloc(string, size);
		if (string == NULL)
		{
			if (t != NULL)
				free(t);
			return NULL;
		}

		va_copy(ap_copy, ap);

		ret = vsnprintf(string, size, fmt, ap_copy);
		/* NB! The result of the vsnprintf() might be an *EMPTY* string! */
		if ((ret >= 0) && ((ret + 1) < size))
			break;

		/* there was just enough or not enough space, allocate more in the next round */
		size *= 2; /* double the buffer size */
	}

	/* the returned buffer is by principle guaranteed to be at least one character longer */
	return string;
}

char *alloc_printf(const char *format, ...)
{
	char *string;
	va_list ap;
	va_start(ap, format);
	string = alloc_vprintf(format, ap);
	va_end(ap);
	return string;
}

/* Code must return to the server loop before 1000ms has returned or invoke
 * this function.
 *
 * The GDB connection will time out if it spends >2000ms and you'll get nasty
 * error messages from GDB:
 *
 * Ignoring packet error, continuing...
 * Reply contains invalid hex digit 116
 *
 * While it is possible use "set remotetimeout" to more than the default 2000ms
 * in GDB, OpenOCD guarantees that it sends keep-alive packages on the
 * GDB protocol and it is a bug in OpenOCD not to either return to the server
 * loop or invoke keep_alive() every 1000ms.
 *
 * This function will send a keep alive packet if >500ms has passed since last time
 * it was invoked.
 *
 * Note that this function can be invoked often, so it needs to be relatively
 * fast when invoked more often than every 500ms.
 *
 */
void keep_alive()
{
	current_time=timeval_ms();
	if (current_time-last_time>1000)
	{
		LOG_WARNING("BUG: keep_alive() was not invoked in the 1000ms timelimit. GDB alive packet not sent! (%lld)", current_time-last_time);
	}
	if (current_time-last_time>500)
	{
		/* this will keep the GDB connection alive */
		LOG_USER_N("%s", "");

		/* DANGER!!!! do not add code to invoke e.g. target event processing,
		 * jim timer processing, etc. it can cause infinite recursion +
		 * jim event callbacks need to happen at a well defined time,
		 * not anywhere keep_alive() is invoked.
		 *
		 * These functions should be invoked at a well defined spot in server.c
		 */

		last_time=current_time;
	}
}

/* reset keep alive timer without sending message */
void kept_alive()
{
	current_time=timeval_ms();
	last_time=current_time;
}

/* if we sleep for extended periods of time, we must invoke keep_alive() intermittantly */
void alive_sleep(int ms)
{
	int i;
	int napTime=10;
	for (i=0; i<ms; i+=napTime)
	{
		int sleep_a_bit=ms-i;
		if (sleep_a_bit>napTime)
		{
			sleep_a_bit=napTime;
		}
		usleep(sleep_a_bit*1000);
		keep_alive();
	}
}

void busy_sleep(int ms)
{
	long long then;
	then=timeval_ms();
	while ((timeval_ms()-then)<ms)
	{
		/* busy wait */
	}
}
