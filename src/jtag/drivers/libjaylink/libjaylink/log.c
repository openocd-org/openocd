/*
 * This file is part of the libjaylink project.
 *
 * Copyright (C) 2014-2015 Marc Schink <jaylink-dev@marcschink.de>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdio.h>
#include <stdarg.h>

#include "libjaylink.h"
#include "libjaylink-internal.h"

/**
 * @file
 *
 * Logging functions.
 */

/**
 * Set the libjaylink log level.
 *
 * @param[in,out] ctx libjaylink context.
 * @param[in] level Log level to set.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_log_set_level(struct jaylink_context *ctx,
		enum jaylink_log_level level)
{
	if (!ctx)
		return JAYLINK_ERR_ARG;

	if (level > JAYLINK_LOG_LEVEL_DEBUG_IO)
		return JAYLINK_ERR_ARG;

	ctx->log_level = level;

	return JAYLINK_OK;
}

/**
 * Get the libjaylink log level.
 *
 * @param[in] ctx libjaylink context.
 * @param[out] level Log level on success, and undefined on failure.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_log_get_level(const struct jaylink_context *ctx,
		enum jaylink_log_level *level)
{
	if (!ctx || !level)
		return JAYLINK_ERR_ARG;

	*level = ctx->log_level;

	return JAYLINK_OK;
}

/**
 * Set the libjaylink log callback function.
 *
 * @param[in,out] ctx libjaylink context.
 * @param[in] callback Callback function to use, or NULL to use the default log
 *                     function.
 * @param[in] user_data User data to be passed to the callback function.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_log_set_callback(struct jaylink_context *ctx,
		jaylink_log_callback callback, void *user_data)
{
	if (!ctx)
		return JAYLINK_ERR_ARG;

	if (callback) {
		ctx->log_callback = callback;
		ctx->log_callback_data = user_data;
	} else {
		ctx->log_callback = &log_vprintf;
		ctx->log_callback_data = NULL;
	}

	return JAYLINK_OK;
}

/**
 * Set the libjaylink log domain.
 *
 * The log domain is a string which is used as prefix for all log messages to
 * differentiate them from messages of other libraries.
 *
 * The maximum length of the log domain is #JAYLINK_LOG_DOMAIN_MAX_LENGTH
 * bytes, excluding the trailing null-terminator. A log domain which exceeds
 * this length will be silently truncated.
 *
 * @param[in,out] ctx libjaylink context.
 * @param[in] domain Log domain to use. To set the default log domain, use
 *                   #JAYLINK_LOG_DOMAIN_DEFAULT.
 *
 * @retval JAYLINK_OK Success.
 * @retval JAYLINK_ERR_ARG Invalid arguments.
 * @retval JAYLINK_ERR Other error conditions.
 *
 * @since 0.1.0
 */
JAYLINK_API int jaylink_log_set_domain(struct jaylink_context *ctx,
		const char *domain)
{
	int ret;

	if (!ctx || !domain)
		return JAYLINK_ERR_ARG;

	ret = snprintf(ctx->log_domain, JAYLINK_LOG_DOMAIN_MAX_LENGTH + 1,
		"%s", domain);

	if (ret < 0)
		return JAYLINK_ERR;

	return JAYLINK_OK;
}

/**
 * Get the libjaylink log domain.
 *
 * @param[in] ctx libjaylink context.
 *
 * @return A string which contains the current log domain on success, or NULL
 *         on failure. The string is null-terminated and must not be free'd by
 *         the caller.
 *
 * @since 0.1.0
 */
JAYLINK_API const char *jaylink_log_get_domain(
		const struct jaylink_context *ctx)
{
	if (!ctx)
		return NULL;

	return ctx->log_domain;
}

/** @private */
JAYLINK_PRIV int log_vprintf(const struct jaylink_context *ctx,
		enum jaylink_log_level level, const char *format, va_list args,
		void *user_data)
{
	(void)user_data;

	/*
	 * Filter out messages with higher verbosity than the verbosity of the
	 * current log level.
	 */
	if (level > ctx->log_level)
		return 0;

	if (ctx->log_domain[0] != '\0')
		fprintf(stderr, "%s", ctx->log_domain);

	vfprintf(stderr, format, args);
	fprintf(stderr, "\n");

	return 0;
}

/** @private */
JAYLINK_PRIV void log_err(const struct jaylink_context *ctx,
		const char *format, ...)
{
	va_list args;

	if (!ctx)
		return;

	va_start(args, format);
	ctx->log_callback(ctx, JAYLINK_LOG_LEVEL_ERROR, format, args,
		ctx->log_callback_data);
	va_end(args);
}

/** @private */
JAYLINK_PRIV void log_warn(const struct jaylink_context *ctx,
		const char *format, ...)
{
	va_list args;

	if (!ctx)
		return;

	va_start(args, format);
	ctx->log_callback(ctx, JAYLINK_LOG_LEVEL_WARNING, format, args,
		ctx->log_callback_data);
	va_end(args);
}

/** @private */
JAYLINK_PRIV void log_info(const struct jaylink_context *ctx,
		const char *format, ...)
{
	va_list args;

	if (!ctx)
		return;

	va_start(args, format);
	ctx->log_callback(ctx, JAYLINK_LOG_LEVEL_INFO, format, args,
		ctx->log_callback_data);
	va_end(args);
}

/** @private */
JAYLINK_PRIV void log_dbg(const struct jaylink_context *ctx,
		const char *format, ...)
{
	va_list args;

	if (!ctx)
		return;

	va_start(args, format);
	ctx->log_callback(ctx, JAYLINK_LOG_LEVEL_DEBUG, format, args,
		ctx->log_callback_data);
	va_end(args);
}

/** @private */
JAYLINK_PRIV void log_dbgio(const struct jaylink_context *ctx,
		const char *format, ...)
{
	va_list args;

	if (!ctx)
		return;

	va_start(args, format);
	ctx->log_callback(ctx, JAYLINK_LOG_LEVEL_DEBUG_IO, format, args,
		ctx->log_callback_data);
	va_end(args);
}
