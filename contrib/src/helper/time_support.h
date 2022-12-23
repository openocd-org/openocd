/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 ***************************************************************************/

#ifndef OPENOCD_HELPER_TIME_SUPPORT_H
#define OPENOCD_HELPER_TIME_SUPPORT_H

#include <time.h>
#include "types.h"

#ifdef HAVE_SYS_TIME_H
#include <sys/time.h>
#endif

int timeval_subtract(struct timeval *result, struct timeval *x, struct timeval *y);
int timeval_add_time(struct timeval *result, long sec, long usec);
int timeval_compare(const struct timeval *x, const struct timeval *y);

/** @returns gettimeofday() timeval as 64-bit in ms */
int64_t timeval_ms(void);

struct duration {
	struct timeval start;
	struct timeval elapsed;
};

/** Update the duration->start field to start the @a duration measurement. */
int duration_start(struct duration *duration);
/** Update the duration->elapsed field to finish the @a duration measurement. */
int duration_measure(struct duration *duration);

/** @returns Elapsed time in seconds. */
float duration_elapsed(const struct duration *duration);
/** @returns KB/sec for the elapsed @a duration and @a count bytes. */
float duration_kbps(const struct duration *duration, size_t count);

#endif /* OPENOCD_HELPER_TIME_SUPPORT_H */
