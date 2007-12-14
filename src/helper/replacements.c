/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/* DANGER!!!! These must be defined *BEFORE* replacements.h and the malloc() macro!!!! */

#include <stdlib.h>
#include <strings.h>
/* 
 * clear_malloc
 *
 * will alloc memory and clear it
 */
void *clear_malloc(size_t size)
{
	void *t = malloc(size);
	if (t!=NULL)
	{
		memset(t, 0x00, size);
	}
	return t;
}

void *fill_malloc(size_t size)
{
	void *t = malloc(size);
	if (t!=NULL)
	{
		/* We want to initialize memory to some known bad state.  */
		/* 0 and 0xff yields 0 and -1 as integers, which often		*/
		/* have meaningful values. 0x5555... is not often a valid	*/
		/* integer and is quite easily spotted in the debugger		*/
		/* also it is almost certainly an invalid address					*/
		memset(t, 0x55, size);
	}
	return t;
}

#include "replacements.h"

#include <stdio.h>

/* replacements for gettimeofday */
#ifndef HAVE_GETTIMEOFDAY

/* Windows */
#ifdef _WIN32

#ifndef __GNUC__
#define EPOCHFILETIME (116444736000000000i64)
#else
#define EPOCHFILETIME (116444736000000000LL)
#endif

int gettimeofday(struct timeval *tv, struct timezone *tz)
{
	FILETIME        ft;
	LARGE_INTEGER   li;
	__int64         t;
	static int      tzflag;

	if (tv)
	{
		GetSystemTimeAsFileTime(&ft);
		li.LowPart  = ft.dwLowDateTime;
		li.HighPart = ft.dwHighDateTime;
		t  = li.QuadPart;                   /* In 100-nanosecond intervals */
		t -= EPOCHFILETIME;                 /* Offset to the Epoch time */
		t /= 10;                            /* In microseconds */
		tv->tv_sec  = (long)(t / 1000000);
		tv->tv_usec = (long)(t % 1000000);
	}

	if (tz)
	{
		if (!tzflag)
		{
			_tzset();
			tzflag++;
		}
		tz->tz_minuteswest = _timezone / 60;
		tz->tz_dsttime = _daylight;
	}

	return 0;
}
#endif /* _WIN32 */

#endif /* HAVE_GETTIMEOFDAY */

#ifndef HAVE_STRNLEN
size_t strnlen(const char *s, size_t maxlen)
{
	const char *end= (const char *)memchr(s, '\0', maxlen);
	return end ? (size_t) (end - s) : maxlen;
}
#endif

#ifndef HAVE_STRNDUP
char* strndup(const char *s, size_t n)
{
	size_t len = strnlen (s, n);
	char *new = (char *) malloc (len + 1);

	if (new == NULL)
		return NULL;

	new[len] = '\0';
	return (char *) memcpy (new, s, len);
}
#endif
