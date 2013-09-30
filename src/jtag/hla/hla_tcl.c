/***************************************************************************
 *   Copyright (C) 2011 by Mathias Kuester                                 *
 *   Mathias Kuester <kesmtp@freenet.de>                                   *
 *                                                                         *
 *   Copyright (C) 2012 by Spencer Oliver                                  *
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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

/* project specific includes */
#include <jtag/interface.h>
#include <transport/transport.h>
#include <helper/time_support.h>

static int jim_newtap_expected_id(Jim_Nvp *n, Jim_GetOptInfo *goi,
				  struct jtag_tap *pTap)
{
	jim_wide w;
	int e = Jim_GetOpt_Wide(goi, &w);
	if (e != JIM_OK) {
		Jim_SetResultFormatted(goi->interp, "option: %s bad parameter",
				       n->name);
		return e;
	}

	unsigned expected_len = sizeof(uint32_t) * pTap->expected_ids_cnt;
	uint32_t *new_expected_ids = malloc(expected_len + sizeof(uint32_t));
	if (new_expected_ids == NULL) {
		Jim_SetResultFormatted(goi->interp, "no memory");
		return JIM_ERR;
	}

	memcpy(new_expected_ids, pTap->expected_ids, expected_len);

	new_expected_ids[pTap->expected_ids_cnt] = w;

	free(pTap->expected_ids);
	pTap->expected_ids = new_expected_ids;
	pTap->expected_ids_cnt++;

	return JIM_OK;
}

#define NTAP_OPT_EXPECTED_ID 0

static int jim_hl_newtap_cmd(Jim_GetOptInfo *goi)
{
	struct jtag_tap *pTap;
	int x;
	int e;
	Jim_Nvp *n;
	char *cp;
	const Jim_Nvp opts[] = {
		{.name = "-expected-id", .value = NTAP_OPT_EXPECTED_ID},
		{.name = NULL, .value = -1},
	};

	pTap = calloc(1, sizeof(struct jtag_tap));
	if (!pTap) {
		Jim_SetResultFormatted(goi->interp, "no memory");
		return JIM_ERR;
	}

	/*
	 * we expect CHIP + TAP + OPTIONS
	 * */
	if (goi->argc < 3) {
		Jim_SetResultFormatted(goi->interp,
				       "Missing CHIP TAP OPTIONS ....");
		free(pTap);
		return JIM_ERR;
	}
	Jim_GetOpt_String(goi, &cp, NULL);
	pTap->chip = strdup(cp);

	Jim_GetOpt_String(goi, &cp, NULL);
	pTap->tapname = strdup(cp);

	/* name + dot + name + null */
	x = strlen(pTap->chip) + 1 + strlen(pTap->tapname) + 1;
	cp = malloc(x);
	sprintf(cp, "%s.%s", pTap->chip, pTap->tapname);
	pTap->dotted_name = cp;

	LOG_DEBUG("Creating New Tap, Chip: %s, Tap: %s, Dotted: %s, %d params",
		  pTap->chip, pTap->tapname, pTap->dotted_name, goi->argc);

	while (goi->argc) {
		e = Jim_GetOpt_Nvp(goi, opts, &n);
		if (e != JIM_OK) {
			Jim_GetOpt_NvpUnknown(goi, opts, 0);
			free(cp);
			free(pTap);
			return e;
		}
		LOG_DEBUG("Processing option: %s", n->name);
		switch (n->value) {
		case NTAP_OPT_EXPECTED_ID:
			e = jim_newtap_expected_id(n, goi, pTap);
			if (JIM_OK != e) {
				free(cp);
				free(pTap);
				return e;
			}
			break;
		}		/* switch (n->value) */
	}			/* while (goi->argc) */

	/* default is enabled-after-reset */
	pTap->enabled = !pTap->disabled_after_reset;

	jtag_tap_init(pTap);
	return JIM_OK;
}

int jim_hl_newtap(Jim_Interp *interp, int argc, Jim_Obj * const *argv)
{
	Jim_GetOptInfo goi;
	Jim_GetOpt_Setup(&goi, interp, argc - 1, argv + 1);
	return jim_hl_newtap_cmd(&goi);
}
