/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
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
#ifndef REPLACEMENTS_H
#define REPLACEMENTS_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "types.h"

#if BUILD_ECOSBOARD
#include <pkgconf/system.h>
#include <stdlib.h>
#include <sys/select.h>
#endif

/* include necessary headers for socket functionality */
#ifdef _WIN32
#include <winsock2.h>
#else
#include <sys/socket.h>
#include <sys/poll.h>
#include <netinet/in.h>
#include <unistd.h>
#include <fcntl.h>
#endif

#ifdef HAVE_SYS_PARAM_H
#include <sys/param.h> /* for MIN/MAX macros */
#endif

/* MIN,MAX macros */
#ifndef MIN
#define MIN(a,b) (((a)<(b))?(a):(b))
#endif
#ifndef MAX
#define MAX(a,b) (((a)>(b))?(a):(b))
#endif

/* for systems that do not support ENOTSUP
 * win32 being one of them */
#ifndef ENOTSUP
#define ENOTSUP 134		/* Not supported */
#endif

#ifndef HAVE_SYS_TIME_H

#ifndef _TIMEVAL_DEFINED
#define _TIMEVAL_DEFINED

struct timeval {
	long tv_sec;
	long tv_usec;
};

#endif /* _TIMEVAL_DEFINED */

#endif

/* gettimeofday() */
#ifndef HAVE_GETTIMEOFDAY

#ifdef _WIN32
struct timezone {
	int tz_minuteswest;
	int tz_dsttime;
};
#endif
struct timezone;

extern int gettimeofday(struct timeval *tv, struct timezone *tz);
#endif

/**** clear_malloc & fill_malloc ****/
void *clear_malloc(size_t size);
void *fill_malloc(size_t size);

/*
 * Now you have 3 ways for the malloc function:
 *
 * 1. Do not change anything, use the original malloc
 *
 * 2. Use the clear_malloc function instead of the original malloc.
 *    In this case you must use the following define:
 *    #define malloc((_a)) clear_malloc((_a))
 *
 * 3. Use the fill_malloc function instead of the original malloc.
 *    In this case you must use the following define:
 *    #define malloc((_a)) fill_malloc((_a))
 *
 * We have figured out that there could exist some malloc problems
 * where variables are using without to be initialise. To find this
 * places, use the fill_malloc function. With this function we want
 * to initialize memory to some known bad state. This is quite easily
 * spotted in the debugger and will trap to an invalid address.
 *
 * clear_malloc can be used if you want to set not initialise
 * variable to 0.
 *
 * If you do not want to change the malloc function, to not use one of
 * the following macros. Which is the default way.
 */

/* #define malloc(_a) clear_malloc(_a) */
/* #define malloc(_a) fill_malloc(_a) */

/* GNU extensions to the C library that may be missing on some systems */
#ifndef HAVE_STRNDUP
extern char* strndup(const char *s, size_t n);
#endif /* HAVE_STRNDUP */

#ifndef HAVE_STRNLEN
extern size_t strnlen(const char *s, size_t maxlen);
#endif /* HAVE_STRNLEN */

#ifndef HAVE_USLEEP
#ifdef _WIN32
static __inline unsigned usleep(unsigned int usecs)
{
	Sleep((usecs/1000));
	return 0;
}
#else
#if BUILD_ECOSBOARD
void usleep(int us);
#else
#error no usleep defined for your platform
#endif
#endif
#endif /* HAVE_USLEEP */

/* Windows specific */
#ifdef _WIN32

#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#include <time.h>

/* win32 systems do not support ETIMEDOUT */

#ifndef ETIMEDOUT
#define ETIMEDOUT WSAETIMEDOUT
#endif

#if IS_MINGW == 1
static __inline unsigned char inb(unsigned short int port)
{
	unsigned char _v;
	__asm__ __volatile__ ("inb %w1,%0":"=a" (_v):"Nd" (port));
	return _v;
}

static __inline void outb(unsigned char value, unsigned short int port)
{
	__asm__ __volatile__ ("outb %b0,%w1": :"a" (value), "Nd" (port));
}

#endif /* IS_MINGW */

int win_select(int max_fd, fd_set *rfds, fd_set *wfds, fd_set *efds, struct timeval *tv);

#endif  /* _WIN32 */

/* generic socket functions for Windows and Posix */
static __inline int write_socket( int handle, const void *buffer, unsigned int count )
{
#ifdef _WIN32
	return send(handle, buffer, count, 0);
#else
	return write(handle, buffer, count);
#endif
}

static __inline int read_socket( int handle, void *buffer, unsigned int count )
{
#ifdef _WIN32
	return recv(handle, buffer, count, 0);
#else
	return read(handle, buffer, count);
#endif
}

static __inline int close_socket(int sock)
{
#ifdef _WIN32
	return closesocket(sock);
#else
	return close(sock);
#endif
}

static __inline void socket_nonblock(int fd)
{
#ifdef _WIN32
	unsigned long nonblock = 1;
	ioctlsocket(fd, FIONBIO, &nonblock );
#else
	int oldopts = fcntl(fd, F_GETFL, 0);
	fcntl(fd, F_SETFL, oldopts | O_NONBLOCK);
#endif
}

static __inline int socket_select(int max_fd, fd_set *rfds, fd_set *wfds, fd_set *efds, struct timeval *tv)
{
#ifdef _WIN32
	return win_select(max_fd, rfds, wfds, efds, tv);
#else
	return select(max_fd, rfds, wfds, efds, tv);
#endif
}

#ifndef HAVE_ELF_H

typedef struct
{
	unsigned char	e_ident[16];	/* Magic number and other info */
	u16	e_type;			/* Object file type */
	u16	e_machine;		/* Architecture */
	u32	e_version;		/* Object file version */
	u32 e_entry;		/* Entry point virtual address */
	u32 e_phoff;		/* Program header table file offset */
	u32	e_shoff;		/* Section header table file offset */
	u32	e_flags;		/* Processor-specific flags */
	u16	e_ehsize;		/* ELF header size in bytes */
	u16	e_phentsize;	/* Program header table entry size */
	u16	e_phnum;		/* Program header table entry count */
	u16	e_shentsize;	/* Section header table entry size */
	u16	e_shnum;		/* Section header table entry count */
	u16	e_shstrndx;		/* Section header string table index */
} Elf32_Ehdr;

#define	ELFMAG		"\177ELF"
#define	SELFMAG		4

#define EI_CLASS	4		/* File class byte index */
#define ELFCLASS32	1		/* 32-bit objects */
#define ELFCLASS64	2		/* 64-bit objects */

#define EI_DATA		5		/* Data encoding byte index */
#define ELFDATA2LSB	1		/* 2's complement, little endian */
#define ELFDATA2MSB	2		/* 2's complement, big endian */

typedef struct
{
	u32 p_type;			/* Segment type */
	u32 p_offset;		/* Segment file offset */
	u32 p_vaddr;		/* Segment virtual address */
	u32 p_paddr;		/* Segment physical address */
	u32 p_filesz;		/* Segment size in file */
	u32 p_memsz;		/* Segment size in memory */
	u32 p_flags;		/* Segment flags */
	u32 p_align;		/* Segment alignment */
} Elf32_Phdr;

#define PT_LOAD		1		/* Loadable program segment */

#endif /* HAVE_ELF_H */

#endif /* REPLACEMENTS_H */
