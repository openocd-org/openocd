/* Jim - A small embeddable Tcl interpreter
 *
 * Copyright 2005 Salvatore Sanfilippo <antirez@invece.org>
 * Copyright 2005 Clemens Hintze <c.hintze@gmx.net>
 * Copyright 2005 patthoyts - Pat Thoyts <patthoyts@users.sf.net>
 * Copyright 2008 oharboe - Øyvind Harboe - oyvind.harboe@zylin.com
 * Copyright 2008 Andrew Lunn <andrew@lunn.ch>
 * Copyright 2008 Duane Ellis <openocd@duaneellis.com>
 * Copyright 2008 Uwe Klein <uklein@klein-messgeraete.de>
 *
 * The FreeBSD license
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above
 *    copyright notice, this list of conditions and the following
 *    disclaimer in the documentation and/or other materials
 *    provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE JIM TCL PROJECT ``AS IS'' AND ANY
 * EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * JIM TCL PROJECT OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
 * INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF
 * ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 * The views and conclusions contained in the software and documentation
 * are those of the authors and should not be interpreted as representing
 * official policies, either expressed or implied, of the Jim Tcl Project.
 **/
/* TODO:
 *
 *  - to really use flags in Jim_ProcessEvents()
 *  - more complete [after] command with [after info] and other subcommands.
 *  - Win32 port
 */
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#define JIM_EXTENSION
#define __JIM_EVENTLOOP_CORE__
#ifdef __ECOS
#include <pkgconf/jimtcl.h>
#include <sys/time.h>
#include <cyg/jimtcl/jim.h>
#include <cyg/jimtcl/jim-eventloop.h>
#else
#include "jim.h"
#include "jim-eventloop.h"
#endif

/* File event structure */
typedef struct Jim_FileEvent {
    void *handle;
    int mask; /* one of JIM_EVENT_(READABLE | WRITABLE | EXCEPTION) */
    Jim_FileProc *fileProc;
    Jim_EventFinalizerProc *finalizerProc;
    void *clientData;
    struct Jim_FileEvent *next;
} Jim_FileEvent;

/* Time event structure */
typedef struct Jim_TimeEvent {
    jim_wide id; /* time event identifier. */
    int mode;	/* restart, repetitive .. UK */
    long initialms; /* initial relativ timer value UK */
    long when_sec; /* seconds */
    long when_ms; /* milliseconds */
    Jim_TimeProc *timeProc;
    Jim_EventFinalizerProc *finalizerProc;
    void *clientData;
    struct Jim_TimeEvent *next;
} Jim_TimeEvent;

/* Per-interp stucture containing the state of the event loop */
typedef struct Jim_EventLoop {
    jim_wide timeEventNextId;
    Jim_FileEvent *fileEventHead;
    Jim_TimeEvent *timeEventHead;
} Jim_EventLoop;

void Jim_CreateFileHandler(Jim_Interp *interp, void *handle, int mask,
        Jim_FileProc *proc, void *clientData,
        Jim_EventFinalizerProc *finalizerProc)
{
    Jim_FileEvent *fe;
    Jim_EventLoop *eventLoop = Jim_GetAssocData(interp, "eventloop");

	// fprintf(stderr,"rein\n");
    fe = Jim_Alloc(sizeof(*fe));
    fe->handle = handle;
    fe->mask = mask;
    fe->fileProc = proc;
    fe->finalizerProc = finalizerProc;
    fe->clientData = clientData;
    fe->next = eventLoop->fileEventHead;
    eventLoop->fileEventHead = fe;
	// fprintf(stderr,"raus\n");
}

void Jim_DeleteFileHandler(Jim_Interp *interp, void *handle)
{
    Jim_FileEvent *fe, *prev = NULL;
    Jim_EventLoop *eventLoop = Jim_GetAssocData(interp, "eventloop");

    fe = eventLoop->fileEventHead;
    while (fe) {
        if (fe->handle == handle) {
            if (prev == NULL)
                eventLoop->fileEventHead = fe->next;
            else
                prev->next = fe->next;
            if (fe->finalizerProc)
                fe->finalizerProc(interp, fe->clientData);
            Jim_Free(fe);
            return;
        }
        prev = fe;
        fe = fe->next;
    }
}

// The same for signals.
void Jim_CreateSignalHandler(Jim_Interp *interp, int signum,
        Jim_FileProc *proc, void *clientData,
        Jim_EventFinalizerProc *finalizerProc)
{
}
void Jim_DeleteSignalHandler(Jim_Interp *interp, int signum)
{
}

/* That's another part of this extension that needs to be ported
 * to WIN32. */
static void JimGetTime(long *seconds, long *milliseconds)
{
    struct timeval tv;

    gettimeofday(&tv, NULL);
    *seconds = tv.tv_sec;
    *milliseconds = tv.tv_usec/1000;
}

jim_wide Jim_CreateTimeHandler(Jim_Interp *interp, jim_wide milliseconds,
        Jim_TimeProc *proc, void *clientData,
        Jim_EventFinalizerProc *finalizerProc)
{
    Jim_EventLoop *eventLoop = Jim_GetAssocData(interp, "eventloop");
    jim_wide id = eventLoop->timeEventNextId++;
    Jim_TimeEvent *te;
    long cur_sec, cur_ms;

    JimGetTime(&cur_sec, &cur_ms);

    te = Jim_Alloc(sizeof(*te));
    te->id = id;
    te->mode = 0;
    te->initialms = milliseconds;
    te->when_sec = cur_sec + milliseconds/1000;
    te->when_ms = cur_ms + milliseconds%1000;
    if (te->when_ms >= 1000) {
        te->when_sec ++;
        te->when_ms -= 1000;
    }
    te->timeProc = proc;
    te->finalizerProc = finalizerProc;
    te->clientData = clientData;
    te->next = eventLoop->timeEventHead;
    eventLoop->timeEventHead = te;
    return id;
}

jim_wide Jim_DeleteTimeHandler(Jim_Interp *interp, jim_wide id)
{
    Jim_TimeEvent *te, *prev = NULL;
    Jim_EventLoop *eventLoop = Jim_GetAssocData(interp, "eventloop");
    long cur_sec, cur_ms;
    jim_wide remain ;

    JimGetTime(&cur_sec, &cur_ms);

    te = eventLoop->timeEventHead;
    if (id >= eventLoop->timeEventNextId)
    	return -2; /* wrong event ID */
    while (te) {
        if (te->id == id) {
            remain  = (te->when_sec - cur_sec) * 1000;
            remain += (te->when_ms  - cur_ms) ;
	    remain = (remain < 0) ? 0 : remain ;

            if (prev == NULL)
                eventLoop->timeEventHead = te->next;
            else
                prev->next = te->next;
            if (te->finalizerProc)
                te->finalizerProc(interp, te->clientData);
            Jim_Free(te);
            return remain;
        }
        prev = te;
        te = te->next;
    }
    return -1; /* NO event with the specified ID found */
}

/* Search the first timer to fire.
 * This operation is useful to know how many time the select can be
 * put in sleep without to delay any event.
 * If there are no timers NULL is returned. */
static Jim_TimeEvent *JimSearchNearestTimer(Jim_EventLoop *eventLoop)
{
    Jim_TimeEvent *te = eventLoop->timeEventHead;
    Jim_TimeEvent *nearest = NULL;

    while (te) {
        if (!nearest || te->when_sec < nearest->when_sec ||
                (te->when_sec == nearest->when_sec &&
                 te->when_ms < nearest->when_ms))
            nearest = te;
        te = te->next;
    }
    return nearest;
}

/* --- POSIX version of Jim_ProcessEvents, for now the only available --- */
#define JIM_FILE_EVENTS 1
#define JIM_TIME_EVENTS 2
#define JIM_ALL_EVENTS (JIM_FILE_EVENTS | JIM_TIME_EVENTS)
#define JIM_DONT_WAIT 4

/* Process every pending time event, then every pending file event
 * (that may be registered by time event callbacks just processed).
 * Without special flags the function sleeps until some file event
 * fires, or when the next time event occurrs (if any).
 *
 * If flags is 0, the function does nothing and returns.
 * if flags has JIM_ALL_EVENTS set, all the kind of events are processed.
 * if flags has JIM_FILE_EVENTS set, file events are processed.
 * if flags has JIM_TIME_EVENTS set, time events are processed.
 * if flags has JIM_DONT_WAIT set the function returns ASAP until all
 * the events that's possible to process without to wait are processed.
 *
 * The function returns the number of events processed. */
int Jim_ProcessEvents(Jim_Interp *interp, int flags)
{
    int maxfd = 0, numfd = 0, processed = 0;
    fd_set rfds, wfds, efds;
    Jim_EventLoop *eventLoop = Jim_GetAssocData(interp, "eventloop");
    Jim_FileEvent *fe = eventLoop->fileEventHead;
    Jim_TimeEvent *te;
    jim_wide maxId;
    JIM_NOTUSED(flags);

    FD_ZERO(&rfds);
    FD_ZERO(&wfds);
    FD_ZERO(&efds);

    /* Check file events */
    while (fe != NULL) {
        int fd = fileno((FILE*)fe->handle);

        if (fe->mask & JIM_EVENT_READABLE)
		FD_SET(fd, &rfds);
        if (fe->mask & JIM_EVENT_WRITABLE) FD_SET(fd, &wfds);
        if (fe->mask & JIM_EVENT_EXCEPTION) FD_SET(fd, &efds);
        if (maxfd < fd) maxfd = fd;
        numfd++;
        fe = fe->next;
    }
    /* Note that we want call select() even if there are no
     * file events to process as long as we want to process time
     * events, in order to sleep until the next time event is ready
     * to fire. */
    if (numfd || ((flags & JIM_TIME_EVENTS) && !(flags & JIM_DONT_WAIT))) {
        int retval;
        Jim_TimeEvent *shortest;
        struct timeval tv, *tvp;
	jim_wide dt;

        shortest = JimSearchNearestTimer(eventLoop);
        if (shortest) {
            long now_sec, now_ms;

            /* Calculate the time missing for the nearest
             * timer to fire. */
            JimGetTime(&now_sec, &now_ms);
            tvp = &tv;
	    dt   = 1000 * (shortest->when_sec - now_sec);
	    dt  += (shortest->when_ms  - now_ms);
            if (dt < 0) {
		dt = 1;
	    }
	    tvp->tv_sec  = dt / 1000;
	    tvp->tv_usec = dt % 1000;
	    // fprintf(stderr,"Next %d.% 8d\n",(int)tvp->tv_sec,(int)tvp->tv_usec);
        } else {
            tvp = NULL; /* wait forever */
		// fprintf(stderr,"No Event\n");
        }

        retval = select(maxfd + 1, &rfds, &wfds, &efds, tvp);
        if (retval < 0) {
	   switch (errno) {
	       case EINTR:   fprintf(stderr,"select EINTR\n"); break;
	       case EINVAL:  fprintf(stderr,"select EINVAL\n"); break;
 	       case ENOMEM:  fprintf(stderr,"select ENOMEM\n"); break;
	   }
	} else if (retval > 0) {
            fe = eventLoop->fileEventHead;
            while (fe != NULL) {
                int fd = fileno((FILE*)fe->handle);

		// fprintf(stderr,"fd: %d mask: %02x \n",fd,fe->mask);

                if ((fe->mask & JIM_EVENT_READABLE && FD_ISSET(fd, &rfds)) ||
                    (fe->mask & JIM_EVENT_WRITABLE && FD_ISSET(fd, &wfds)) ||
                    (fe->mask & JIM_EVENT_EXCEPTION && FD_ISSET(fd, &efds)))
                {
                    int mask = 0;

                    if (fe->mask & JIM_EVENT_READABLE && FD_ISSET(fd, &rfds)) {
                        mask |= JIM_EVENT_READABLE;
			if ((fe->mask & JIM_EVENT_FEOF) && feof((FILE *)fe->handle))
				mask |= JIM_EVENT_FEOF;
		    }
                    if (fe->mask & JIM_EVENT_WRITABLE && FD_ISSET(fd, &wfds))
                        mask |= JIM_EVENT_WRITABLE;
                    if (fe->mask & JIM_EVENT_EXCEPTION && FD_ISSET(fd, &efds))
                        mask |= JIM_EVENT_EXCEPTION;
                    if (fe->fileProc(interp, fe->clientData, mask) == JIM_ERR) {
                        /* Remove the element on handler error */
                        Jim_DeleteFileHandler(interp, fe->handle);
                    }
                    processed++;
                    /* After an event is processed our file event list
                     * may no longer be the same, so what we do
                     * is to clear the bit for this file descriptor and
                     * restart again from the head. */
                    fe = eventLoop->fileEventHead;
                    FD_CLR(fd, &rfds);
                    FD_CLR(fd, &wfds);
                    FD_CLR(fd, &efds);
                } else {
                    fe = fe->next;
                }
            }
        }
    }
    /* Check time events */
    te = eventLoop->timeEventHead;
    maxId = eventLoop->timeEventNextId-1;
    while (te) {
        long now_sec, now_ms;
        jim_wide id;

        if (te->id > maxId) {
            te = te->next;
            continue;
        }
        JimGetTime(&now_sec, &now_ms);
        if (now_sec > te->when_sec ||
            (now_sec == te->when_sec && now_ms >= te->when_ms))
        {
            id = te->id;
            te->timeProc(interp, te->clientData);
            /* After an event is processed our time event list may
             * no longer be the same, so we restart from head.
             * Still we make sure to don't process events registered
             * by event handlers itself in order to don't loop forever
             * even in case an [after 0] that continuously register
             * itself. To do so we saved the max ID we want to handle. */
            Jim_DeleteTimeHandler(interp, id);
            te = eventLoop->timeEventHead;
        } else {
            te = te->next;
        }
    }

    return processed;
}
/* ---------------------------------------------------------------------- */

void JimELAssocDataDeleProc(Jim_Interp *interp, void *data)
{
    void *next;
    Jim_FileEvent *fe;
    Jim_TimeEvent *te;
    Jim_EventLoop *eventLoop = data;

    fe = eventLoop->fileEventHead;
    while (fe) {
        next = fe->next;
        if (fe->finalizerProc)
            fe->finalizerProc(interp, fe->clientData);
        Jim_Free(fe);
        fe = next;
    }

    te = eventLoop->timeEventHead;
    while (te) {
        next = te->next;
        if (te->finalizerProc)
            te->finalizerProc(interp, te->clientData);
        Jim_Free(te);
        te = next;
    }
    Jim_Free(data);
}

static int JimELVwaitCommand(Jim_Interp *interp, int argc,
        Jim_Obj *const *argv)
{
    Jim_Obj *oldValue;

    if (argc != 2) {
        Jim_WrongNumArgs(interp, 1, argv, "name");
        return JIM_ERR;
    }
    oldValue = Jim_GetGlobalVariable(interp, argv[1], JIM_NONE);
    if (oldValue) Jim_IncrRefCount(oldValue);
    while (1) {
        Jim_Obj *currValue;

        Jim_ProcessEvents(interp, JIM_ALL_EVENTS);
        currValue = Jim_GetGlobalVariable(interp, argv[1], JIM_NONE);
        /* Stop the loop if the vwait-ed variable changed value,
         * or if was unset and now is set (or the contrary). */
        if ((oldValue && !currValue) ||
            (!oldValue && currValue) ||
            (oldValue && currValue &&
             !Jim_StringEqObj(oldValue, currValue, JIM_CASESENS)))
            break;
    }
    if (oldValue) Jim_DecrRefCount(interp, oldValue);
    return JIM_OK;
}

void JimAfterTimeHandler(Jim_Interp *interp, void *clientData)
{
    Jim_Obj *objPtr = clientData;

    Jim_EvalObjBackground(interp, objPtr);
}

void JimAfterTimeEventFinalizer(Jim_Interp *interp, void *clientData)
{
    Jim_Obj *objPtr = clientData;

    Jim_DecrRefCount(interp, objPtr);
}

static int JimELAfterCommand(Jim_Interp *interp, int argc,
        Jim_Obj *const *argv)
{
    jim_wide ms, id;
    Jim_Obj *objPtr, *idObjPtr;
    const char *options[] = {
	"info", "cancel", "restart", "expire", NULL
    };
    enum {INFO, CANCEL, RESTART, EXPIRE, CREATE };
    int option = CREATE ;

    if (argc < 3) {
        Jim_WrongNumArgs(interp, 1, argv, "<after milliseconds> script");
        return JIM_ERR;
    }
    if (Jim_GetWide(interp, argv[1], &ms) != JIM_OK)
        if (Jim_GetEnum(interp, argv[1], options, &option, "after options",
                    JIM_ERRMSG) != JIM_OK)
            return JIM_ERR;
    switch (option) {
    case CREATE:
        Jim_IncrRefCount(argv[2]);
        id = Jim_CreateTimeHandler(interp, ms, JimAfterTimeHandler, argv[2],
                JimAfterTimeEventFinalizer);
        objPtr = Jim_NewStringObj(interp, NULL, 0);
        Jim_AppendString(interp, objPtr, "after#", -1);
        idObjPtr = Jim_NewIntObj(interp, id);
        Jim_IncrRefCount(idObjPtr);
        Jim_AppendObj(interp, objPtr, idObjPtr);
        Jim_DecrRefCount(interp, idObjPtr);
        Jim_SetResult(interp, objPtr);
        return JIM_OK;
    case CANCEL:
	{
	int tlen ;
	jim_wide remain = 0;
	const char *tok = Jim_GetString(argv[2], &tlen);
	if (sscanf(tok,"after#%" JIM_WIDE_MODIFIER, &id) == 1) {
		remain =  Jim_DeleteTimeHandler(interp, id);
		if (remain > -2)  {
			Jim_SetResult(interp, Jim_NewIntObj(interp, remain));
			return JIM_OK;
		}
	}
        Jim_SetResultString(interp, "invalid event" , -1);
        return JIM_ERR;
	}
    default:
	fprintf(stderr,"unserviced option to after %d\n",option);
    }
    return JIM_OK;
}

/* This extension is not dynamically loaded, instead it's linked statically,
   which is why we shouldn't use the unspecific 'Jim_OnLoad' name */
int Jim_EventLoopOnLoad(Jim_Interp *interp)
{
    Jim_EventLoop *eventLoop;

    Jim_InitExtension(interp);
    if (Jim_PackageProvide(interp, "eventloop", "1.0", JIM_ERRMSG) != JIM_OK)
        return JIM_ERR;

    eventLoop = Jim_Alloc(sizeof(*eventLoop));
    eventLoop->fileEventHead = NULL;
    eventLoop->timeEventHead = NULL;
    eventLoop->timeEventNextId = 1;
    Jim_SetAssocData(interp, "eventloop", JimELAssocDataDeleProc, eventLoop);

    Jim_CreateCommand(interp, "vwait", JimELVwaitCommand, NULL, NULL);
    Jim_CreateCommand(interp, "after", JimELAfterCommand, NULL, NULL);

    /* Export events API */
    Jim_RegisterApi(interp, "Jim_CreateFileHandler", Jim_CreateFileHandler);
    Jim_RegisterApi(interp, "Jim_DeleteFileHandler", Jim_DeleteFileHandler);
    Jim_RegisterApi(interp, "Jim_CreateTimeHandler", Jim_CreateTimeHandler);
    Jim_RegisterApi(interp, "Jim_DeleteTimeHandler", Jim_DeleteTimeHandler);
    Jim_RegisterApi(interp, "Jim_ProcessEvents", Jim_ProcessEvents);
    return JIM_OK;
}
