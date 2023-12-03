// SPDX-License-Identifier: BSD-2-Clause-Views

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
 * This file is extracted from jim-nvp.c, originally part of jim TCL code.
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <string.h>

#include <helper/command.h>
#include <helper/nvp.h>

const struct nvp *nvp_name2value(const struct nvp *p, const char *name)
{
	while (p->name) {
		if (strcmp(name, p->name) == 0)
			break;
		p++;
	}
	return p;
}

const struct nvp *nvp_value2name(const struct nvp *p, int value)
{
	while (p->name) {
		if (value == p->value)
			break;
		p++;
	}
	return p;
}

void nvp_unknown_command_print(struct command_invocation *cmd, const struct nvp *nvp,
	const char *param_name, const char *param_value)
{
	if (param_name)
		command_print_sameline(cmd, "%s: Unknown: %s, try one of: ", param_name, param_value);
	else
		command_print_sameline(cmd, "Unknown param: %s, try one of: ", param_value);

	while (nvp->name) {
		if ((nvp + 1)->name)
			command_print_sameline(cmd, "%s, ", nvp->name);
		else
			command_print(cmd, "or %s", nvp->name);

		nvp++;
	}

	/* We assume nvp to be not empty and loop has been taken; no need to add a '\n' */
}
