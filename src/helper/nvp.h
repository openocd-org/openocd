/* SPDX-License-Identifier: BSD-2-Clause-Views */

/*
 * Copyright 2005 Salvatore Sanfilippo <antirez@invece.org>
 * Copyright 2005 Clemens Hintze <c.hintze@gmx.net>
 * Copyright 2005 patthoyts - Pat Thoyts <patthoyts@users.sf.net>
 * Copyright 2008 oharboe - Øyvind Harboe - oyvind.harboe@zylin.com
 * Copyright 2008 Andrew Lunn <andrew@lunn.ch>
 * Copyright 2008 Duane Ellis <openocd@duaneellis.com>
 * Copyright 2008 Uwe Klein <uklein@klein-messgeraete.de>
 * Copyright 2008 Steve Bennett <steveb@workware.net.au>
 * Copyright 2009 Nico Coesel <ncoesel@dealogic.nl>
 * Copyright 2009 Zachary T Welch zw@superlucidity.net
 * Copyright 2009 David Brownell
 * Copyright (c) 2005-2011 Jim Tcl Project. All rights reserved.
 *
 * This file is extracted from jim_nvp.h, originally part of jim TCL code.
 */

#ifndef OPENOCD_HELPER_NVP_H
#define OPENOCD_HELPER_NVP_H

#include <helper/compiler.h>

/** Name Value Pairs, aka: NVP
 *   -  Given a string - return the associated int.
 *   -  Given a number - return the associated string.
 *   .
 *
 * Very useful when the number is not a simple index into an array of
 * known string, or there may be multiple strings (aliases) that mean then same
 * thing.
 *
 * An NVP Table is terminated with ".name = NULL".
 *
 * During the 'name2value' operation, if no matching string is found
 * the pointer to the terminal element (with p->name == NULL) is returned.
 *
 * Example:
 * \code
 *      const struct nvp yn[] = {
 *          { "yes", 1 },
 *          { "no" , 0 },
 *          { "yep", 1 },
 *          { "nope", 0 },
 *          { NULL, -1 },
 *      };
 *
 *  struct nvp *result;
 *  result = nvp_name2value(yn, "yes");
 *         returns &yn[0];
 *  result = nvp_name2value(yn, "no");
 *         returns &yn[1];
 *  result = jim_nvp_name2value(yn, "Blah");
 *         returns &yn[4];
 * \endcode
 *
 * During the number2name operation, the first matching value is returned.
 */

struct nvp {
	const char *name;
	int value;
};

struct command_invocation;

/* Name Value Pairs Operations */
const struct nvp *nvp_name2value(const struct nvp *nvp_table, const char *name)
	__returns_nonnull __nonnull((1));
const struct nvp *nvp_value2name(const struct nvp *nvp_table, int v)
	__returns_nonnull __nonnull((1));

void nvp_unknown_command_print(struct command_invocation *cmd, const struct nvp *nvp,
	const char *param_name, const char *param_value);

#endif /* OPENOCD_HELPER_NVP_H */
