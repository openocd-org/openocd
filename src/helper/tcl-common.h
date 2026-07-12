/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_HELPER_TCL_COMMON_H
#define OPENOCD_HELPER_TCL_COMMON_H

#include <jim.h>

/**
 * Convert a C string to a string that can be used for Tcl list.
 * The returned string has to be deallocated through free().
 * @param interp: the Tcl interpreter
 * @param s: the C string to convert
 * @returns converted string or NULL on error
 */
char *tcl_escape_alloc(Jim_Interp *interp, const char *s);

#endif /* OPENOCD_HELPER_TCL_COMMON_H */
