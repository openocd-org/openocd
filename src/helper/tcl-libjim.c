// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * This file collects all the functions to interact with Jim Tcl library.
 *
 * The purposes are:
 * - to decouple the jimtcl error codes (JIM_OK, JIM_ERR, ...) from the error
 *   codes of OpenOCD;
 * - to decouple the internal Jim_Obj and its garbage collection;
 * - to concentrate the Jim Tcl CamelCase symbols, now spread in OpenOCD code.
 *
 * The Jim Tcl CamelCase symbols used in this file should be reported in the
 * file 'tools/scripts/camelcase.txt' to prevent errors from checkpatch.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <assert.h>
#include <string.h>

#include <helper/tcl-common.h>

char *tcl_escape_alloc(Jim_Interp *interp, const char *s)
{
	assert(s);

	Jim_Obj *o1 = Jim_NewStringObj(interp, s, -1);
	Jim_Obj *o2 = Jim_NewListObj(interp, &o1, 1);
	Jim_IncrRefCount(o2);

	char *out = strdup(Jim_String(o2));

	Jim_DecrRefCount(interp, o2);

	return out;
}
