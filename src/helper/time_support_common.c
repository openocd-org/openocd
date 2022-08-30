// SPDX-License-Identifier: GPL-2.0-or-later

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

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "time_support.h"

/* simple and low overhead fetching of ms counter. Use only
 * the difference between ms counters returned from this fn.
 */
int64_t timeval_ms(void)
{
	struct timeval now;
	int retval = gettimeofday(&now, NULL);
	if (retval < 0)
		return retval;
	return (int64_t)now.tv_sec * 1000 + now.tv_usec / 1000;
}
