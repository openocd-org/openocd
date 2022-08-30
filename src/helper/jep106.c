// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2015 Andreas Fritiofson                                 *
 *   andreas.fritiofson@gmail.com                                          *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "jep106.h"
#include "log.h"

static const char * const jep106[][126] = {
#include "jep106.inc"
};

const char *jep106_table_manufacturer(unsigned int bank, unsigned int id)
{
	if (id < 1 || id > 126) {
		LOG_DEBUG("BUG: Caller passed out-of-range JEP106 ID!");
		return "<invalid>";
	}

	/* index is zero based */
	id--;

	if (bank >= ARRAY_SIZE(jep106) || jep106[bank][id] == 0)
		return "<unknown>";

	return jep106[bank][id];
}
