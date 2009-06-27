/***************************************************************************
 *   Copyright (C) 2009 By Duane Ellis                                     *
 *   openocd@duaneellis.com                                                *
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

#include <stdio.h>
#include <stdarg.h>
#include <stdlib.h>
#include <string.h>

#include "membuf.h"

struct membuf {
    // buflen is alway "+1" bigger then
    // what is shown here, the +1 is for
    // the NULL string terminator
#define DEFAULT_BUFSIZE 100
    size_t maxlen; // allocated size
    size_t curlen; // where we are inserting at
    char *_strtoklast;
    void *buf;
};


#define space_avail(pBuf)  (pBuf->maxlen - pBuf->curlen)
#define dataend(pBuf)      (((char *)(pBuf->buf)) + pBuf->curlen)

size_t
membuf_len(struct membuf *pBuf)
{
    return pBuf->curlen;
}

const void *
membuf_datapointer(struct membuf *pBuf)
{
    return ((void *)(pBuf->buf));
}

const char *
membuf_strtok(struct membuf *pBuf, const char *sep, void **pLast)
{
    if (pBuf) {
	pBuf->_strtoklast = NULL;
	*pLast = pBuf;
	// this should be "strtok_r()" but windows lacks */
	return strtok(((char *)(pBuf->buf)), sep);
    } else {
	// recover our pBuf
	pBuf = *((struct membuf **)(pLast));
	// this should be "strtok_r()" but windows lacks */
	return strtok( NULL, sep);
    }
}



struct membuf *
membuf_new(void)
{
    // by default - parameters are zero.
    struct membuf *pBuf;

    pBuf = calloc(1, sizeof(*pBuf));
    if (pBuf) {
	// we *ALWAYS* allocate +1 for null terminator.
	pBuf->buf = calloc(DEFAULT_BUFSIZE + 1, sizeof(char));
	if (pBuf->buf == NULL) {
	    free(pBuf);
	    pBuf = NULL;
	} else {
	    pBuf->maxlen = DEFAULT_BUFSIZE;
	}
    }
    return pBuf;
}


struct membuf *
membuf_grow(struct membuf *pBuf, int n)
{
    void *vp;
    signed int newsize;

    // this is a *SIGNED* value
    newsize = ((int)(pBuf->maxlen)) + n;

    // do not go negative, or too small
    if (newsize < DEFAULT_BUFSIZE) {
	newsize = DEFAULT_BUFSIZE;
    }

    // always alloc +1 for the null terminator
    vp = realloc(pBuf->buf, newsize + 1);
    if (vp) {
	pBuf->buf    = vp;
	pBuf->maxlen = newsize;
	return pBuf;
    } else {
	return NULL;
    }
}


void membuf_reset(struct membuf *pBuf)
{
    pBuf->curlen = 0;
}


void membuf_delete(struct membuf *pBuf)
{
    if (pBuf) {
	if (pBuf->buf) {
	    // wack data so it cannot be reused
	    memset(pBuf->buf,0,pBuf->maxlen);
	    free(pBuf->buf);
	}
	// wack dat so it cannot be reused
	memset(pBuf,0,sizeof(pBuf));
	free(pBuf);
    }
}

int
membuf_sprintf(struct membuf *pBuf , const char *fmt, ...)
{
    int r;
    va_list ap;
    va_start(ap, fmt);
    r = membuf_vsprintf(pBuf, fmt, ap);
    va_end(ap);
    return r;
}

int
membuf_vsprintf(struct membuf *pBuf, const char *fmt, va_list ap)
{
    int r;
    size_t sa;
    int grew;


    grew = 0;
    for (;;) {
	sa = space_avail(pBuf);

	// do work
	r = vsnprintf(dataend(pBuf),
		       sa,
		       fmt,
		       ap);
	if ((r > 0) && (((size_t)(r)) < sa)) {
	    // Success!
	    pBuf->curlen += ((size_t)(r));
	    // remember: We always alloc'ed +1
	    // so this does not overflow
	    ((char *)(pBuf->buf))[ pBuf->curlen ] = 0;
	    r = 0;
	    break;
	}

	// failure
	if (r < 0) {
	    // Option(A) format error
	    // Option(B) glibc2.0 bug
	    // assume (B).
	    r = (4 * DEFAULT_BUFSIZE);
	}

	// don't do this again
	if (grew) {
	    r = -1;
	    break;
	}
	grew = 1;
	pBuf = membuf_grow(pBuf, r);
	if (pBuf == NULL) {
	    // grow failed
	    r = -1;
	    break;
	}
    }
    return r;
}

struct membuf *
membuf_strcat(struct membuf *pBuf, const char *pStr)
{
    return membuf_append(pBuf, pStr, strlen(pStr));
}

struct membuf *
membuf_append(struct membuf *pBuf, const void *pData, size_t len)
{
    size_t sa;
    int r;

    // how much room is there?
    sa = space_avail(pBuf);

    // will it fit?
    if (sa < len) {
	// if not, how much do we need?
	r = ((int)(sa - len));
	// do the grow.
	pBuf = membuf_grow(pBuf, r);
	// failed?
	if (pBuf == NULL) {
	    return pBuf;
	}
    }
    // append
    memcpy(dataend(pBuf),
	    pData,
	    len);
    pBuf->curlen += len;
    return pBuf;
}






