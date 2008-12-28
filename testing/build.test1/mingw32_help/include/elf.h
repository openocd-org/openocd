/* elf.h

  Copyright 2005 Red Hat, Inc.

This file is part of Cygwin.

This software is a copyrighted work licensed under the terms of the
Cygwin license.  Please consult the file "CYGWIN_LICENSE" for
details. */

#ifndef	_ELF_H_
#define	_ELF_H_

#include <stdint.h>

typedef signed char int8_t;
typedef unsigned char u_int8_t;
typedef short int16_t;
typedef unsigned short u_int16_t;
typedef int int32_t;
typedef unsigned int u_int32_t;
typedef long long int64_t;
typedef unsigned long long u_int64_t;
typedef int32_t register_t;


#ifdef __cplusplus
extern "C" {
#endif
#include <sys/types.h>
#include <sys/elf32.h>
#include <sys/elf64.h>
#include <sys/elf_generic.h>
#ifdef __cplusplus
}
#endif

#endif /*_ELF_H_*/
