/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath <Dominic.Rath@gmx.de>              *
 *   Copyright (C) 2007-2008 by Øyvind Harboe <oyvind.harboe@zylin.com>    *
 *   Copyright (C) 2008 by Spencer Oliver <spen@spen-soft.co.uk>           *
 *   Copyright (C) 2009 by Zachary T Welch <zw@superlucidity.net>          *
 ***************************************************************************/

#ifndef OPENOCD_HELPER_SYSTEM_H
#define OPENOCD_HELPER_SYSTEM_H

/* +++ platform specific headers +++ */
#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#include <sys/types.h>
#include <sys/stat.h>
#endif
/* --- platform specific headers --- */

/* standard C library header files */
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <ctype.h>
#include <errno.h>
#include <time.h>

#ifdef HAVE_SYS_TIME_H
#include <sys/time.h>
#endif

#ifdef HAVE_SYS_SOCKET_H
#include <sys/socket.h>
#endif
#ifdef HAVE_POLL_H
#include <poll.h>
#endif

#ifdef __ECOS
/* missing from eCos */
#ifndef EFAULT
#define EFAULT 14	/* Bad address */
#endif
#endif

#ifdef HAVE_NETINET_IN_H
#include <netinet/in.h>
#endif
#ifdef HAVE_SYS_SELECT_H
#include <sys/select.h>	/* select, FD_SET and friends (POSIX.1-2001) */
#endif
#ifdef HAVE_SYS_PARAM_H
#include <sys/param.h>	/* for MIN/MAX macros */
#endif
#ifdef HAVE_UNISTD_H
#include <unistd.h>
#endif
#ifdef HAVE_FCNTL_H
#include <fcntl.h>
#endif

#ifndef true
#define true    1
#define false   0
#endif

#endif /* OPENOCD_HELPER_SYSTEM_H */
