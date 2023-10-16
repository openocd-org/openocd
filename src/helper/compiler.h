/* SPDX-License-Identifier: GPL-2.0-or-later */

/*
 * This file contains compiler specific workarounds to handle different
 * compilers and different compiler versions.
 * Inspired by Linux's include/linux/compiler_attributes.h
 * and file sys/cdefs.h in libc and newlib.
 */

#ifndef OPENOCD_HELPER_COMPILER_H
#define OPENOCD_HELPER_COMPILER_H

/*
 * __has_attribute is supported on gcc >= 5, clang >= 2.9 and icc >= 17.
 */
#ifndef __has_attribute
# define __has_attribute(x) 0
#endif

/*
 * The __returns_nonnull function attribute marks the return type of the function
 * as always being non-null.
 */
#ifndef __returns_nonnull
# if __has_attribute(__returns_nonnull__)
#  define __returns_nonnull __attribute__((__returns_nonnull__))
# else
#  define __returns_nonnull
# endif
#endif

/*
 * The __nonnull function attribute marks pointer parameters that
 * must not be NULL.
 *
 * clang for Apple defines
 * #define __nonnull _Nonnull
 * that is a per argument attribute, incompatible with the gcc per function attribute __nonnull__.
 * gcc for Apple includes sys/cdefs.h from MacOSX.sdk that defines
 * #define __nonnull
 * In both cases, undefine __nonnull to keep compatibility among compilers and platforms.
 */
#if defined(__APPLE__)
# undef __nonnull
#endif
#ifndef __nonnull
# if __has_attribute(__nonnull__)
#  define __nonnull(params) __attribute__ ((__nonnull__ params))
# else
#  define __nonnull(params)
# endif
#endif

#endif /* OPENOCD_HELPER_COMPILER_H */
