/***************************************************************************
 *   Copyright (C) 2006 by Magnus Lundin                                   *
 *   lundin@mlu.mine.nu                                                    *
 *                                                                         *
 *   Copyright (C) 2008 by Spencer Oliver                                  *
 *   spen@spen-soft.co.uk                                                  *
 *                                                                         *
 *   Copyright (C) 2009-2010 by Oyvind Harboe                              *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2009-2010 by David Brownell                             *
 *                                                                         *
 *   Copyright (C) 2013 by Andreas Fritiofson                              *
 *   andreas.fritiofson@gmail.com                                          *
 *                                                                         *
 *   Copyright (C) 2019, Ampere Computing LLC                              *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "jtag/interface.h"
#include "arm.h"
#include "arm_adi.h"
#include "jtag/swd.h"
#include "transport/transport.h"
#include <helper/jep106.h>
#include <helper/time_support.h>
#include <helper/list.h>
#include <helper/jim-nvp.h>


enum adi_cfg_param {
	CFG_DAP,
	CFG_AP_NUM,
	CFG_AP_BASE
};

static const Jim_Nvp nvp_config_opts[] = {
	{ .name = "-dap",    .value = CFG_DAP },
	{ .name = "-ap-num", .value = CFG_AP_NUM },
	{ .name = "-apbase", .value = CFG_AP_BASE },
	{ .name = NULL, .value = -1 }
};

int adi_jim_configure(struct target *target, Jim_GetOptInfo *goi)
{
	struct adi_private_config *pc;
	int e;

	pc = (struct adi_private_config *)target->private_config;
	if (pc == NULL) {
		pc = calloc(1, sizeof(struct adi_private_config));
		pc->ap_num = DP_APSEL_INVALID;
		target->private_config = pc;
	}

	target->has_dap = true;

	if (goi->argc > 0) {
		Jim_Nvp *n;

		Jim_SetEmptyResult(goi->interp);

		/* check first if topmost item is for us */
		e = Jim_Nvp_name2value_obj(goi->interp, nvp_config_opts,
								   goi->argv[0], &n);
		if (e != JIM_OK)
			return JIM_CONTINUE;

		e = Jim_GetOpt_Obj(goi, NULL);
		if (e != JIM_OK)
			return e;

		switch (n->value) {
		case CFG_DAP:
			if (goi->isconfigure) {
				Jim_Obj *o_t;
				struct adi_dap *dap;
				e = Jim_GetOpt_Obj(goi, &o_t);
				if (e != JIM_OK)
					return e;
				dap = dap_instance_by_jim_obj(goi->interp, o_t);
				if (dap == NULL) {
					Jim_SetResultString(goi->interp, "DAP name invalid!", -1);
					return JIM_ERR;
				}
				if (pc->dap != NULL && pc->dap != dap) {
					Jim_SetResultString(goi->interp,
						"DAP assignment cannot be changed after target was created!", -1);
					return JIM_ERR;
				}
				if (target->tap_configured) {
					Jim_SetResultString(goi->interp,
						"-chain-position and -dap configparams are mutually exclusive!", -1);
					return JIM_ERR;
				}
				pc->dap = dap;
				target->tap = dap->tap;
				target->dap_configured = true;
			} else {
				if (goi->argc != 0) {
					Jim_WrongNumArgs(goi->interp,
										goi->argc, goi->argv,
					"NO PARAMS");
					return JIM_ERR;
				}

				if (pc->dap == NULL) {
					Jim_SetResultString(goi->interp, "DAP not configured", -1);
					return JIM_ERR;
				}
				Jim_SetResultString(goi->interp, adi_dap_name(pc->dap), -1);
			}
			break;

		case CFG_AP_NUM:
			if (goi->isconfigure) {
				jim_wide ap_num;
				e = Jim_GetOpt_Wide(goi, &ap_num);
				if (e != JIM_OK)
					return e;
				if (ap_num < 0 || ap_num > DP_APSEL_MAX) {
					Jim_SetResultString(goi->interp, "Invalid AP number!", -1);
					return JIM_ERR;
				}
				pc->ap_num = ap_num;
			} else {
				if (goi->argc != 0) {
					Jim_WrongNumArgs(goi->interp,
									 goi->argc, goi->argv,
					  "NO PARAMS");
					return JIM_ERR;
				}

				if (pc->ap_num == DP_APSEL_INVALID) {
					Jim_SetResultString(goi->interp, "AP number not configured", -1);
					return JIM_ERR;
				}
				Jim_SetResult(goi->interp, Jim_NewIntObj(goi->interp, pc->ap_num));
			}
			break;

		case CFG_AP_BASE:
			if (goi->isconfigure) {
				jim_wide ap_base;
				e = Jim_GetOpt_Wide(goi, &ap_base);
				if (e != JIM_OK)
					return e;
				if (ap_base & 0xffful) {
					Jim_SetResultString(goi->interp, "Invalid AP Base Address!", -1);
					return JIM_ERR;
				}
				if (pc->ap_num == DP_APSEL_INVALID) {
					Jim_SetResultString(goi->interp, "config -ap-num must precede -apbase!", -1);
					return JIM_ERR;
				}
				pc->dap->ap[pc->ap_num].base_addr = ap_base;
			} else {
				if (goi->argc != 0) {
					Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv, "NO PARAMS");
					return JIM_ERR;
				}

				if (pc->ap_num == DP_APSEL_INVALID) {
					Jim_SetResultString(goi->interp, "AP number not configured", -1);
					return JIM_ERR;
				}
				Jim_SetResult(goi->interp, Jim_NewIntObj(goi->interp, pc->ap_num));
			}
			break;
		}
	}

	return JIM_OK;
}

int adi_verify_config(struct adi_private_config *pc)
{
	if (pc == NULL)
		return ERROR_FAIL;

	if (pc->dap == NULL)
		return ERROR_FAIL;

	return ERROR_OK;
}
