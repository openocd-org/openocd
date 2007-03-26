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
#ifndef ERROR_H
#define ERROR_H

#include "command.h"

#include <stdarg.h>

/* logging priorities 
 * LOG_ERROR - fatal errors, that are likely to cause program abort
 * LOG_WARNING - non-fatal errors, that may be resolved later
 * LOG_INFO - state information, etc.
 * LOG_DEBUG - debug statements, execution trace
 */
enum log_levels
{
	LOG_ERROR = 0,
	LOG_WARNING = 1,
	LOG_INFO = 2,
	LOG_DEBUG = 3
};

extern void log_printf(enum log_levels level, const char *file, int line, 
	const char *function, const char *format, ...) 
	__attribute__ ((format (printf, 5, 6)));
extern int log_register_commands(struct command_context_s *cmd_ctx);
extern int log_init(struct command_context_s *cmd_ctx);

extern int debug_level;

#define DEBUG(expr ...) \
	do { \
		log_printf (LOG_DEBUG, __FILE__, __LINE__, __FUNCTION__, expr); \
	} while(0)

#define INFO(expr ...) \
	do { \
		log_printf (LOG_INFO, __FILE__, __LINE__, __FUNCTION__, expr); \
	} while(0)

#define WARNING(expr ...) \
	do { \
		log_printf (LOG_WARNING, __FILE__, __LINE__, __FUNCTION__, expr); \
	} while(0)

#define ERROR(expr ...) \
	do { \
		log_printf (LOG_ERROR, __FILE__, __LINE__, __FUNCTION__, expr); \
	} while(0)

#define SDEBUG(expr ...) \
	do { \
		short_log_printf (LOG_DEBUG, expr); \
	} while(0)

#define SINFO(expr ...) \
	do { \
		short_log_printf (LOG_INFO, expr); \
	} while(0)

#define SWARNING(expr ...) \
	do { \
		short_log_printf (LOG_WARNING, expr); \
	} while(0)

#define SERROR(expr ...) \
	do { \
		short_log_printf (LOG_ERROR, expr); \
	} while(0)

/* general failures
 * error codes < 100
 */
#define ERROR_OK					(0)
#define ERROR_INVALID_ARGUMENTS		(-1)
#define ERROR_NO_CONFIG_FILE		(-2)
#define ERROR_BUF_TOO_SMALL			(-3)

#endif /* ERROR_H */
