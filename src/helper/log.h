/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 ***************************************************************************/

#ifndef OPENOCD_HELPER_LOG_H
#define OPENOCD_HELPER_LOG_H

#include <helper/command.h>

/* To achieve C99 printf compatibility in MinGW, gnu_printf should be
 * used for __attribute__((format( ... ))), with GCC v4.4 or later
 */
#if (defined(IS_MINGW) && (((__GNUC__ << 16) + __GNUC_MINOR__) >= 0x00040004)) && !defined(__clang__)
#define PRINTF_ATTRIBUTE_FORMAT gnu_printf
#else
#define PRINTF_ATTRIBUTE_FORMAT printf
#endif

/* logging priorities
 * LOG_LVL_SILENT - turn off all output. In lieu of try + catch this can be used as a
 *                  feeble ersatz.
 * LOG_LVL_USER - user messages. Could be anything from information
 *                to progress messages. These messages do not represent
 *                incorrect or unexpected behaviour, just normal execution.
 * LOG_LVL_ERROR - fatal errors, that are likely to cause program abort
 * LOG_LVL_WARNING - non-fatal errors, that may be resolved later
 * LOG_LVL_INFO - state information, etc.
 * LOG_LVL_DEBUG - debug statements, execution trace
 * LOG_LVL_DEBUG_IO - verbose debug, low-level I/O trace
 */
enum log_levels {
	LOG_LVL_SILENT = -3,
	LOG_LVL_OUTPUT = -2,
	LOG_LVL_USER = -1,
	LOG_LVL_ERROR = 0,
	LOG_LVL_WARNING = 1,
	LOG_LVL_INFO = 2,
	LOG_LVL_DEBUG = 3,
	LOG_LVL_DEBUG_IO = 4,
};

void log_printf(enum log_levels level, const char *file, unsigned int line,
		const char *function, const char *format, ...)
__attribute__ ((format (PRINTF_ATTRIBUTE_FORMAT, 5, 6)));
void log_vprintf_lf(enum log_levels level, const char *file, unsigned int line,
		const char *function, const char *format, va_list args);
void log_printf_lf(enum log_levels level, const char *file, unsigned int line,
		const char *function, const char *format, ...)
__attribute__ ((format (PRINTF_ATTRIBUTE_FORMAT, 5, 6)));

/**
 * Initialize logging module.  Call during program startup.
 */
void log_init(void);
void log_exit(void);

int log_register_commands(struct command_context *cmd_ctx);

void keep_alive(void);
void kept_alive(void);

void alive_sleep(uint64_t ms);
void busy_sleep(uint64_t ms);

void log_socket_error(const char *socket_desc);

typedef void (*log_callback_fn)(void *priv, const char *file, unsigned int line,
		const char *function, const char *string);

struct log_callback {
	log_callback_fn fn;
	void *priv;
	struct log_callback *next;
};

int log_add_callback(log_callback_fn fn, void *priv);
int log_remove_callback(log_callback_fn fn, void *priv);

char *alloc_vprintf(const char *fmt, va_list ap);
char *alloc_printf(const char *fmt, ...)
	__attribute__ ((format (PRINTF_ATTRIBUTE_FORMAT, 1, 2)));

const char *find_nonprint_char(const char *buf, unsigned int buf_len);

extern int debug_level;

/* Avoid fn call and building parameter list if we're not outputting the information.
 * Matters on feeble CPUs for DEBUG/INFO statements that are involved frequently */

#define LOG_LEVEL_IS(FOO)  ((debug_level) >= (FOO))

#define LOG_DEBUG_IO(expr ...) \
	do { \
		if (debug_level >= LOG_LVL_DEBUG_IO) \
			log_printf_lf(LOG_LVL_DEBUG, \
				__FILE__, __LINE__, __func__, \
				expr); \
	} while (0)

#define LOG_DEBUG(expr ...) \
	do { \
		if (debug_level >= LOG_LVL_DEBUG) \
			log_printf_lf(LOG_LVL_DEBUG, \
				__FILE__, __LINE__, __func__, \
				expr); \
	} while (0)

#define LOG_CUSTOM_LEVEL(level, expr ...) \
	do { \
		enum log_levels _level = level; \
		if (debug_level >= _level) \
			log_printf_lf(_level, \
				__FILE__, __LINE__, __func__, \
				expr); \
	} while (0)

#define LOG_INFO(expr ...) \
	log_printf_lf(LOG_LVL_INFO, __FILE__, __LINE__, __func__, expr)

#define LOG_WARNING(expr ...) \
	log_printf_lf(LOG_LVL_WARNING, __FILE__, __LINE__, __func__, expr)

#define LOG_ERROR(expr ...) \
	log_printf_lf(LOG_LVL_ERROR, __FILE__, __LINE__, __func__, expr)

#define LOG_USER(expr ...) \
	log_printf_lf(LOG_LVL_USER, __FILE__, __LINE__, __func__, expr)

#define LOG_USER_N(expr ...) \
	log_printf(LOG_LVL_USER, __FILE__, __LINE__, __func__, expr)

#define LOG_OUTPUT(expr ...) \
	log_printf(LOG_LVL_OUTPUT, __FILE__, __LINE__, __func__, expr)

/* Output a log entry that is related to a given target */

#define LOG_TARGET_DEBUG_IO(target, fmt_str, ...) \
	LOG_DEBUG_IO("[%s] " fmt_str, target_name(target), ##__VA_ARGS__)

#define LOG_TARGET_DEBUG(target, fmt_str, ...) \
	LOG_DEBUG("[%s] " fmt_str, target_name(target), ##__VA_ARGS__)

#define LOG_TARGET_INFO(target, fmt_str, ...) \
	LOG_INFO("[%s] " fmt_str, target_name(target), ##__VA_ARGS__)

#define LOG_TARGET_USER(target, fmt_str, ...) \
	LOG_USER("[%s] " fmt_str, target_name(target), ##__VA_ARGS__)

#define LOG_TARGET_WARNING(target, fmt_str, ...) \
	LOG_WARNING("[%s] " fmt_str, target_name(target), ##__VA_ARGS__)

#define LOG_TARGET_ERROR(target, fmt_str, ...) \
	LOG_ERROR("[%s] " fmt_str, target_name(target), ##__VA_ARGS__)

/* general failures
 * error codes < 100
 */
#define ERROR_OK						(0)
#define ERROR_NO_CONFIG_FILE			(-2)
#define ERROR_BUF_TOO_SMALL				(-3)
/* see "Error:" log entry for meaningful message to the user. The caller should
 * make no assumptions about what went wrong and try to handle the problem.
 */
#define ERROR_FAIL						(-4)
#define ERROR_WAIT						(-5)
/* ERROR_TIMEOUT is already taken by winerror.h. */
#define ERROR_TIMEOUT_REACHED			(-6)
#define ERROR_NOT_IMPLEMENTED			(-7)


#endif /* OPENOCD_HELPER_LOG_H */
