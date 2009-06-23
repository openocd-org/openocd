/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007,2008 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2009 SoftPLC Corporation                                *
 *       http://softplc.com                                                *
 *   dick@softplc.com                                                      *
 *                                                                         *
 *   Copyright (C) 2009 Zachary T Welch                                    *
 *   zw@superlucidity.net                                                  *
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
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "jtag.h"
#include "minidriver.h"
#include "interface.h"

#ifdef HAVE_STRINGS_H
#include <strings.h>
#endif

static const Jim_Nvp nvp_jtag_tap_event[] = {
	{ .value = JTAG_TAP_EVENT_ENABLE,       .name = "tap-enable" },
	{ .value = JTAG_TAP_EVENT_DISABLE,      .name = "tap-disable" },

	{ .name = NULL, .value = -1 }
};

/* jtag interfaces (parport, FTDI-USB, TI-USB, ...)
 */

#if BUILD_ZY1000 == 1
	extern jtag_interface_t zy1000_interface;
#elif defined(BUILD_MINIDRIVER_DUMMY)
	extern jtag_interface_t minidummy_interface;
#else // standard drivers
#if BUILD_PARPORT == 1
	extern jtag_interface_t parport_interface;
#endif

#if BUILD_DUMMY == 1
	extern jtag_interface_t dummy_interface;
#endif

#if BUILD_FT2232_FTD2XX == 1
	extern jtag_interface_t ft2232_interface;
#endif

#if BUILD_FT2232_LIBFTDI == 1
	extern jtag_interface_t ft2232_interface;
#endif

#if BUILD_AMTJTAGACCEL == 1
	extern jtag_interface_t amt_jtagaccel_interface;
#endif

#if BUILD_EP93XX == 1
	extern jtag_interface_t ep93xx_interface;
#endif

#if BUILD_AT91RM9200 == 1
	extern jtag_interface_t at91rm9200_interface;
#endif

#if BUILD_GW16012 == 1
	extern jtag_interface_t gw16012_interface;
#endif

#if BUILD_PRESTO_LIBFTDI == 1 || BUILD_PRESTO_FTD2XX == 1
	extern jtag_interface_t presto_interface;
#endif

#if BUILD_USBPROG == 1
	extern jtag_interface_t usbprog_interface;
#endif

#if BUILD_JLINK == 1
	extern jtag_interface_t jlink_interface;
#endif

#if BUILD_VSLLINK == 1
	extern jtag_interface_t vsllink_interface;
#endif

#if BUILD_RLINK == 1
	extern jtag_interface_t rlink_interface;
#endif

#if BUILD_ARMJTAGEW == 1
	extern jtag_interface_t armjtagew_interface;
#endif
#endif // standard drivers

/**
 * The list of built-in JTAG interfaces, containing entries for those
 * drivers that were enabled by the @c configure script.
 *
 * The list should be defined to contain either one minidriver interface
 * or some number of standard driver interfaces, never both.
 */
jtag_interface_t *jtag_interfaces[] = {
#if BUILD_ZY1000 == 1
	&zy1000_interface,
#elif defined(BUILD_MINIDRIVER_DUMMY)
	&minidummy_interface,
#else // standard drivers
#if BUILD_PARPORT == 1
	&parport_interface,
#endif
#if BUILD_DUMMY == 1
	&dummy_interface,
#endif
#if BUILD_FT2232_FTD2XX == 1
	&ft2232_interface,
#endif
#if BUILD_FT2232_LIBFTDI == 1
	&ft2232_interface,
#endif
#if BUILD_AMTJTAGACCEL == 1
	&amt_jtagaccel_interface,
#endif
#if BUILD_EP93XX == 1
	&ep93xx_interface,
#endif
#if BUILD_AT91RM9200 == 1
	&at91rm9200_interface,
#endif
#if BUILD_GW16012 == 1
	&gw16012_interface,
#endif
#if BUILD_PRESTO_LIBFTDI == 1 || BUILD_PRESTO_FTD2XX == 1
	&presto_interface,
#endif
#if BUILD_USBPROG == 1
	&usbprog_interface,
#endif
#if BUILD_JLINK == 1
	&jlink_interface,
#endif
#if BUILD_VSLLINK == 1
	&vsllink_interface,
#endif
#if BUILD_RLINK == 1
	&rlink_interface,
#endif
#if BUILD_ARMJTAGEW == 1
	&armjtagew_interface,
#endif
#endif // standard drivers
	NULL,
};

extern jtag_interface_t *jtag_interface;

/* jtag commands */
static int handle_interface_list_command(struct command_context_s *cmd_ctx,
		char *cmd, char **args, int argc);
static int handle_interface_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_jtag_speed_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_jtag_khz_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_jtag_device_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_reset_config_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_jtag_nsrst_delay_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_jtag_ntrst_delay_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

static int handle_scan_chain_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

static int handle_jtag_reset_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_runtest_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_irscan_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int Jim_Command_drscan(Jim_Interp *interp, int argc, Jim_Obj *const *argv);
static int Jim_Command_pathmove(Jim_Interp *interp, int argc, Jim_Obj *const *argv);
static int Jim_Command_flush_count(Jim_Interp *interp, int argc, Jim_Obj *const *args);

static int handle_verify_ircapture_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_verify_jtag_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);
static int handle_tms_sequence_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc);

extern int jtag_examine_chain(void);
extern int jtag_validate_chain(void);

enum jtag_tap_cfg_param {
	JCFG_EVENT
};

static Jim_Nvp nvp_config_opts[] = {
	{ .name = "-event",      .value = JCFG_EVENT },

	{ .name = NULL,          .value = -1 }
};

static int jtag_tap_configure_cmd(Jim_GetOptInfo *goi, jtag_tap_t * tap)
{
	Jim_Nvp *n;
	Jim_Obj *o;
	int e;

	/* parse config or cget options */
	while (goi->argc > 0) {
		Jim_SetEmptyResult (goi->interp);

		e = Jim_GetOpt_Nvp(goi, nvp_config_opts, &n);
		if (e != JIM_OK) {
			Jim_GetOpt_NvpUnknown(goi, nvp_config_opts, 0);
			return e;
		}

		switch (n->value) {
			case JCFG_EVENT:
				if (goi->argc == 0) {
					Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv, "-event ?event-name? ...");
					return JIM_ERR;
				}

				e = Jim_GetOpt_Nvp(goi, nvp_jtag_tap_event, &n);
				if (e != JIM_OK) {
					Jim_GetOpt_NvpUnknown(goi, nvp_jtag_tap_event, 1);
					return e;
				}

				if (goi->isconfigure) {
					if (goi->argc != 1) {
						Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv, "-event ?event-name? ?EVENT-BODY?");
						return JIM_ERR;
					}
				} else {
					if (goi->argc != 0) {
						Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv, "-event ?event-name?");
						return JIM_ERR;
					}
				}

				{
					jtag_tap_event_action_t *jteap;

					jteap = tap->event_action;
					/* replace existing? */
					while (jteap) {
						if (jteap->event == (enum jtag_event)n->value) {
							break;
						}
						jteap = jteap->next;
					}

					if (goi->isconfigure) {
						if (jteap == NULL) {
							/* create new */
							jteap = calloc(1, sizeof (*jteap));
						}
						jteap->event = n->value;
						Jim_GetOpt_Obj(goi, &o);
						if (jteap->body) {
							Jim_DecrRefCount(interp, jteap->body);
						}
						jteap->body = Jim_DuplicateObj(goi->interp, o);
						Jim_IncrRefCount(jteap->body);

						/* add to head of event list */
						jteap->next = tap->event_action;
						tap->event_action = jteap;
						Jim_SetEmptyResult(goi->interp);
					} else {
						/* get */
						if (jteap == NULL) {
							Jim_SetEmptyResult(goi->interp);
						} else {
							Jim_SetResult(goi->interp, Jim_DuplicateObj(goi->interp, jteap->body));
						}
					}
				}
				/* loop for more */
				break;
		}
	} /* while (goi->argc) */

	return JIM_OK;
}

static int is_bad_irval(int ir_length, jim_wide w)
{
	jim_wide v = 1;

	v <<= ir_length;
	v -= 1;
	v = ~v;
	return (w & v) != 0;
}

static int jim_newtap_cmd(Jim_GetOptInfo *goi)
{
	jtag_tap_t *pTap;
	jim_wide w;
	int x;
	int e;
	int reqbits;
	Jim_Nvp *n;
	char *cp;
	const Jim_Nvp opts[] = {
#define NTAP_OPT_IRLEN     0
		{ .name = "-irlen"			,	.value = NTAP_OPT_IRLEN },
#define NTAP_OPT_IRMASK    1
		{ .name = "-irmask"			,	.value = NTAP_OPT_IRMASK },
#define NTAP_OPT_IRCAPTURE 2
		{ .name = "-ircapture"		,	.value = NTAP_OPT_IRCAPTURE },
#define NTAP_OPT_ENABLED   3
		{ .name = "-enable"			,	.value = NTAP_OPT_ENABLED },
#define NTAP_OPT_DISABLED  4
		{ .name = "-disable"		,	.value = NTAP_OPT_DISABLED },
#define NTAP_OPT_EXPECTED_ID 5
		{ .name = "-expected-id"	,	.value = NTAP_OPT_EXPECTED_ID },
		{ .name = NULL				,	.value = -1 },
	};

	pTap = calloc(1, sizeof(jtag_tap_t));
	if (!pTap) {
		Jim_SetResult_sprintf(goi->interp, "no memory");
		return JIM_ERR;
	}

	/*
	 * we expect CHIP + TAP + OPTIONS
	 * */
	if (goi->argc < 3) {
		Jim_SetResult_sprintf(goi->interp, "Missing CHIP TAP OPTIONS ....");
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

	/* deal with options */
#define NTREQ_IRLEN      1
#define NTREQ_IRCAPTURE  2
#define NTREQ_IRMASK     4

	/* clear them as we find them */
	reqbits = (NTREQ_IRLEN | NTREQ_IRCAPTURE | NTREQ_IRMASK);

	while (goi->argc) {
		e = Jim_GetOpt_Nvp(goi, opts, &n);
		if (e != JIM_OK) {
			Jim_GetOpt_NvpUnknown(goi, opts, 0);
			return e;
		}
		LOG_DEBUG("Processing option: %s", n->name);
		switch (n->value) {
		case NTAP_OPT_ENABLED:
			pTap->disabled_after_reset = false;
			break;
		case NTAP_OPT_DISABLED:
			pTap->disabled_after_reset = true;
			break;
		case NTAP_OPT_EXPECTED_ID:
		{
			uint32_t *new_expected_ids;

			e = Jim_GetOpt_Wide(goi, &w);
			if (e != JIM_OK) {
				Jim_SetResult_sprintf(goi->interp, "option: %s bad parameter", n->name);
				return e;
			}

			new_expected_ids = malloc(sizeof(uint32_t) * (pTap->expected_ids_cnt + 1));
			if (new_expected_ids == NULL) {
				Jim_SetResult_sprintf(goi->interp, "no memory");
				return JIM_ERR;
			}

			memcpy(new_expected_ids, pTap->expected_ids, sizeof(uint32_t) * pTap->expected_ids_cnt);

			new_expected_ids[pTap->expected_ids_cnt] = w;

			free(pTap->expected_ids);
			pTap->expected_ids = new_expected_ids;
			pTap->expected_ids_cnt++;
			break;
		}
		case NTAP_OPT_IRLEN:
		case NTAP_OPT_IRMASK:
		case NTAP_OPT_IRCAPTURE:
			e = Jim_GetOpt_Wide(goi, &w);
			if (e != JIM_OK) {
				Jim_SetResult_sprintf(goi->interp, "option: %s bad parameter", n->name);
				return e;
			}
			switch (n->value) {
			case NTAP_OPT_IRLEN:
				if (w > (jim_wide) (8 * sizeof(pTap->ir_capture_value)))
					LOG_WARNING("huge IR length %d", (int) w);
				pTap->ir_length = w;
				reqbits &= (~(NTREQ_IRLEN));
				break;
			case NTAP_OPT_IRMASK:
				if (is_bad_irval(pTap->ir_length, w)) {
					LOG_ERROR("IR mask %x too big",
							(int) w);
					return ERROR_FAIL;
				}
				pTap->ir_capture_mask = w;
				reqbits &= (~(NTREQ_IRMASK));
				break;
			case NTAP_OPT_IRCAPTURE:
				if (is_bad_irval(pTap->ir_length, w)) {
					LOG_ERROR("IR capture %x too big",
							(int) w);
					return ERROR_FAIL;
				}
				pTap->ir_capture_value = w;
				reqbits &= (~(NTREQ_IRCAPTURE));
				break;
			}
		} /* switch (n->value) */
	} /* while (goi->argc) */

	/* default is enabled-after-reset */
	pTap->enabled = !pTap->disabled_after_reset;

	/* Did all the required option bits get cleared? */
	if (0 == reqbits)
	{
		jtag_tap_init(pTap);
		return ERROR_OK;
	}

	Jim_SetResult_sprintf(goi->interp,
			"newtap: %s missing required parameters",
			pTap->dotted_name);
	jtag_tap_free(pTap);
	return JIM_ERR;
}

static void jtag_tap_handle_event(jtag_tap_t *tap, enum jtag_event e)
{
	jtag_tap_event_action_t * jteap;
	int done;

	jteap = tap->event_action;

	done = 0;
	while (jteap) {
		if (jteap->event == e) {
			done = 1;
			LOG_DEBUG("JTAG tap: %s event: %d (%s) action: %s\n",
					tap->dotted_name,
					e,
					Jim_Nvp_value2name_simple(nvp_jtag_tap_event, e)->name,
					Jim_GetString(jteap->body, NULL));
			if (Jim_EvalObj(interp, jteap->body) != JIM_OK) {
				Jim_PrintErrorMessage(interp);
			} else {
				/* NOTE:  we currently assume the handlers
				 * can't fail.  That presumes later code
				 * will be verifying the scan chains ...
				 */
				tap->enabled = (e == JTAG_TAP_EVENT_ENABLE);
			}
		}

		jteap = jteap->next;
	}

	if (!done) {
		LOG_DEBUG("event %d %s - no action",
				e,
				Jim_Nvp_value2name_simple(nvp_jtag_tap_event, e)->name);
	}
}


static int jim_jtag_command(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	Jim_GetOptInfo goi;
	int e;
	Jim_Nvp *n;
	Jim_Obj *o;
	struct command_context_s *context;

	enum {
		JTAG_CMD_INTERFACE,
		JTAG_CMD_INIT_RESET,
		JTAG_CMD_NEWTAP,
		JTAG_CMD_TAPENABLE,
		JTAG_CMD_TAPDISABLE,
		JTAG_CMD_TAPISENABLED,
		JTAG_CMD_CONFIGURE,
		JTAG_CMD_CGET
	};

	const Jim_Nvp jtag_cmds[] = {
		{ .name = "interface"     , .value = JTAG_CMD_INTERFACE },
		{ .name = "arp_init-reset", .value = JTAG_CMD_INIT_RESET },
		{ .name = "newtap"        , .value = JTAG_CMD_NEWTAP },
		{ .name = "tapisenabled"     , .value = JTAG_CMD_TAPISENABLED },
		{ .name = "tapenable"     , .value = JTAG_CMD_TAPENABLE },
		{ .name = "tapdisable"    , .value = JTAG_CMD_TAPDISABLE },
		{ .name = "configure"     , .value = JTAG_CMD_CONFIGURE },
		{ .name = "cget"          , .value = JTAG_CMD_CGET },

		{ .name = NULL, .value = -1 },
	};

	context = Jim_GetAssocData(interp, "context");
	/* go past the command */
	Jim_GetOpt_Setup(&goi, interp, argc-1, argv + 1);

	e = Jim_GetOpt_Nvp(&goi, jtag_cmds, &n);
	if (e != JIM_OK) {
		Jim_GetOpt_NvpUnknown(&goi, jtag_cmds, 0);
		return e;
	}
		Jim_SetEmptyResult(goi.interp);
	switch (n->value) {
	case JTAG_CMD_INTERFACE:
		/* return the name of the interface */
		/* TCL code might need to know the exact type... */
		/* FUTURE: we allow this as a means to "set" the interface. */
		if (goi.argc != 0) {
			Jim_WrongNumArgs(goi.interp, 1, goi.argv-1, "(no params)");
			return JIM_ERR;
		}
		Jim_SetResultString(goi.interp, jtag_interface->name, -1);
		return JIM_OK;
	case JTAG_CMD_INIT_RESET:
		if (goi.argc != 0) {
			Jim_WrongNumArgs(goi.interp, 1, goi.argv-1, "(no params)");
			return JIM_ERR;
		}
		e = jtag_init_reset(context);
		if (e != ERROR_OK) {
			Jim_SetResult_sprintf(goi.interp, "error: %d", e);
			return JIM_ERR;
		}
		return JIM_OK;
	case JTAG_CMD_NEWTAP:
		return jim_newtap_cmd(&goi);
		break;
	case JTAG_CMD_TAPISENABLED:
	case JTAG_CMD_TAPENABLE:
	case JTAG_CMD_TAPDISABLE:
		if (goi.argc != 1) {
			Jim_SetResultString(goi.interp, "Too many parameters",-1);
			return JIM_ERR;
		}

		{
			jtag_tap_t *t;

			t = jtag_tap_by_jim_obj(goi.interp, goi.argv[0]);
			if (t == NULL)
				return JIM_ERR;

			switch (n->value) {
			case JTAG_CMD_TAPISENABLED:
				break;
			case JTAG_CMD_TAPENABLE:
				if (t->enabled)
					break;
				jtag_tap_handle_event(t, JTAG_TAP_EVENT_ENABLE);
				if (!t->enabled)
					break;

				/* FIXME add JTAG sanity checks, w/o TLR
				 *  - scan chain length grew by one (this)
				 *  - IDs and IR lengths are as expected
				 */

				jtag_call_event_callbacks(JTAG_TAP_EVENT_ENABLE);
				break;
			case JTAG_CMD_TAPDISABLE:
				if (!t->enabled)
					break;
				jtag_tap_handle_event(t, JTAG_TAP_EVENT_DISABLE);
				if (t->enabled)
					break;

				/* FIXME add JTAG sanity checks, w/o TLR
				 *  - scan chain length shrank by one (this)
				 *  - IDs and IR lengths are as expected
				 */

				jtag_call_event_callbacks(JTAG_TAP_EVENT_DISABLE);
				break;
			}
			e = t->enabled;
			Jim_SetResult(goi.interp, Jim_NewIntObj(goi.interp, e));
			return JIM_OK;
		}
		break;

	case JTAG_CMD_CGET:
		if (goi.argc < 2) {
			Jim_WrongNumArgs(goi.interp, 0, NULL, "?tap-name? -option ...");
			return JIM_ERR;
		}

		{
			jtag_tap_t *t;

			Jim_GetOpt_Obj(&goi, &o);
			t = jtag_tap_by_jim_obj(goi.interp, o);
			if (t == NULL) {
				return JIM_ERR;
			}

			goi.isconfigure = 0;
			return jtag_tap_configure_cmd(&goi, t);
		}
		break;

	case JTAG_CMD_CONFIGURE:
		if (goi.argc < 3) {
			Jim_WrongNumArgs(goi.interp, 0, NULL, "?tap-name? -option ?VALUE? ...");
			return JIM_ERR;
		}

		{
			jtag_tap_t *t;

			Jim_GetOpt_Obj(&goi, &o);
			t = jtag_tap_by_jim_obj(goi.interp, o);
			if (t == NULL) {
				return JIM_ERR;
			}

			goi.isconfigure = 1;
			return jtag_tap_configure_cmd(&goi, t);
		}
	}

	return JIM_ERR;
}

int jtag_register_commands(struct command_context_s *cmd_ctx)
{
	register_jim(cmd_ctx, "jtag", jim_jtag_command, "perform jtag tap actions");

	register_command(cmd_ctx, NULL, "interface", handle_interface_command,
		COMMAND_CONFIG, "try to configure interface");
	register_command(cmd_ctx, NULL,
		"interface_list", &handle_interface_list_command,
		COMMAND_ANY, "list all built-in interfaces");
	register_command(cmd_ctx, NULL, "jtag_speed", handle_jtag_speed_command,
		COMMAND_ANY, "(DEPRECATED) set jtag speed (if supported)");
	register_command(cmd_ctx, NULL, "jtag_khz", handle_jtag_khz_command,
		COMMAND_ANY, "set maximum jtag speed (if supported); "
		"parameter is maximum khz, or 0 for adaptive clocking (RTCK).");
	register_command(cmd_ctx, NULL, "jtag_device", handle_jtag_device_command,
		COMMAND_CONFIG, "(DEPRECATED) jtag_device <ir_length> <ir_expected> <ir_mask>");
	register_command(cmd_ctx, NULL, "reset_config", handle_reset_config_command,
		COMMAND_ANY,
		"[none/trst_only/srst_only/trst_and_srst] [srst_pulls_trst/trst_pulls_srst] [combined/separate] [trst_push_pull/trst_open_drain] [srst_push_pull/srst_open_drain]");
	register_command(cmd_ctx, NULL, "jtag_nsrst_delay", handle_jtag_nsrst_delay_command,
		COMMAND_ANY, "jtag_nsrst_delay <ms> - delay after deasserting srst in ms");
	register_command(cmd_ctx, NULL, "jtag_ntrst_delay", handle_jtag_ntrst_delay_command,
		COMMAND_ANY, "jtag_ntrst_delay <ms> - delay after deasserting trst in ms");

	register_command(cmd_ctx, NULL, "scan_chain", handle_scan_chain_command,
		COMMAND_EXEC, "print current scan chain configuration");

	register_command(cmd_ctx, NULL, "jtag_reset", handle_jtag_reset_command,
		COMMAND_EXEC, "toggle reset lines <trst> <srst>");
	register_command(cmd_ctx, NULL, "runtest", handle_runtest_command,
		COMMAND_EXEC, "move to Run-Test/Idle, and execute <num_cycles>");
	register_command(cmd_ctx, NULL, "irscan", handle_irscan_command,
		COMMAND_EXEC, "execute IR scan <device> <instr> [dev2] [instr2] ...");
	register_jim(cmd_ctx, "drscan", Jim_Command_drscan, "execute DR scan <device> <num_bits> <value> <num_bits1> <value2> ...");
	register_jim(cmd_ctx, "flush_count", Jim_Command_flush_count, "returns number of times the JTAG queue has been flushed");
	register_jim(cmd_ctx, "pathmove", Jim_Command_pathmove, "move JTAG to state1 then to state2, state3, etc. <state1>,<state2>,<stat3>...");

	register_command(cmd_ctx, NULL, "verify_ircapture", handle_verify_ircapture_command,
		COMMAND_ANY, "verify value captured during Capture-IR <enable | disable>");
	register_command(cmd_ctx, NULL, "verify_jtag", handle_verify_jtag_command,
		COMMAND_ANY, "verify value capture <enable | disable>");
	register_command(cmd_ctx, NULL, "tms_sequence", handle_tms_sequence_command,
		COMMAND_ANY, "choose short(default) or long tms_sequence <short | long>");
	return ERROR_OK;
}

static int default_khz(int khz, int *jtag_speed)
{
	LOG_ERROR("Translation from khz to jtag_speed not implemented");
	return ERROR_FAIL;
}

static int default_speed_div(int speed, int *khz)
{
	LOG_ERROR("Translation from jtag_speed to khz not implemented");
	return ERROR_FAIL;
}

static int default_power_dropout(int *dropout)
{
	*dropout = 0; /* by default we can't detect power dropout */
	return ERROR_OK;
}

static int default_srst_asserted(int *srst_asserted)
{
	*srst_asserted = 0; /* by default we can't detect srst asserted */
	return ERROR_OK;
}

static int handle_interface_command(struct command_context_s *cmd_ctx,
		char *cmd, char **args, int argc)
{
	/* check whether the interface is already configured */
	if (jtag_interface)
	{
		LOG_WARNING("Interface already configured, ignoring");
		return ERROR_OK;
	}

	/* interface name is a mandatory argument */
	if (argc != 1 || args[0][0] == '\0')
		return ERROR_COMMAND_SYNTAX_ERROR;

	for (unsigned i = 0; NULL != jtag_interfaces[i]; i++)
	{
		if (strcmp(args[0], jtag_interfaces[i]->name) != 0)
			continue;

		int retval = jtag_interfaces[i]->register_commands(cmd_ctx);
		if (ERROR_OK != retval)
				return retval;

		jtag_interface = jtag_interfaces[i];

		if (jtag_interface->khz == NULL)
			jtag_interface->khz = default_khz;
		if (jtag_interface->speed_div == NULL)
			jtag_interface->speed_div = default_speed_div;
		if (jtag_interface->power_dropout == NULL)
			jtag_interface->power_dropout = default_power_dropout;
		if (jtag_interface->srst_asserted == NULL)
			jtag_interface->srst_asserted = default_srst_asserted;

		return ERROR_OK;
	}

	/* no valid interface was found (i.e. the configuration option,
	 * didn't match one of the compiled-in interfaces
	 */
	LOG_ERROR("The specified JTAG interface was not found (%s)", args[0]);
	handle_interface_list_command(cmd_ctx, cmd, args, argc);
	return ERROR_JTAG_INVALID_INTERFACE;
}

static int handle_interface_list_command(struct command_context_s *cmd_ctx,
		char *cmd, char **args, int argc)
{
	if (strcmp(cmd, "interface_list") == 0 && argc > 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	command_print(cmd_ctx, "The following JTAG interfaces are available:");
	for (unsigned i = 0; NULL != jtag_interfaces[i]; i++)
	{
		const char *name = jtag_interfaces[i]->name;
		command_print(cmd_ctx, "%u: %s", i + 1, name);
	}

	return ERROR_OK;
}

static int handle_jtag_device_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int e;
	char buf[1024];
	Jim_Obj *newargs[ 10 ];
	/*
	 * CONVERT SYNTAX
	 * argv[-1] = command
	 * argv[ 0] = ir length
	 * argv[ 1] = ir capture
	 * argv[ 2] = ir mask
	 * argv[ 3] = not actually used by anything but in the docs
	 */

	if (argc < 4) {
		command_print(cmd_ctx, "OLD DEPRECATED SYNTAX: Please use the NEW syntax");
		return ERROR_OK;
	}
	command_print(cmd_ctx, "OLD SYNTAX: DEPRECATED - translating to new syntax");
	command_print(cmd_ctx, "jtag newtap CHIP TAP -irlen %s -ircapture %s -irvalue %s",
				   args[0],
				   args[1],
				   args[2]);
	command_print(cmd_ctx, "Example: STM32 has 2 taps, the cortexM3(len4) + boundaryscan(len5)");
	command_print(cmd_ctx, "jtag newtap stm32 cortexm3 ....., thus creating the tap: \"stm32.cortexm3\"");
	command_print(cmd_ctx, "jtag newtap stm32 boundary ....., and the tap: \"stm32.boundary\"");
	command_print(cmd_ctx, "And then refer to the taps by the dotted name.");

	newargs[0] = Jim_NewStringObj(interp, "jtag", -1);
	newargs[1] = Jim_NewStringObj(interp, "newtap", -1);
	sprintf(buf, "chip%d", jtag_tap_count());
	newargs[2] = Jim_NewStringObj(interp, buf, -1);
	sprintf(buf, "tap%d", jtag_tap_count());
	newargs[3] = Jim_NewStringObj(interp, buf, -1);
	newargs[4] = Jim_NewStringObj(interp, "-irlen", -1);
	newargs[5] = Jim_NewStringObj(interp, args[0], -1);
	newargs[6] = Jim_NewStringObj(interp, "-ircapture", -1);
	newargs[7] = Jim_NewStringObj(interp, args[1], -1);
	newargs[8] = Jim_NewStringObj(interp, "-irmask", -1);
	newargs[9] = Jim_NewStringObj(interp, args[2], -1);

	command_print(cmd_ctx, "NEW COMMAND:");
	sprintf(buf, "%s %s %s %s %s %s %s %s %s %s",
			 Jim_GetString(newargs[0], NULL),
			 Jim_GetString(newargs[1], NULL),
			 Jim_GetString(newargs[2], NULL),
			 Jim_GetString(newargs[3], NULL),
			 Jim_GetString(newargs[4], NULL),
			 Jim_GetString(newargs[5], NULL),
			 Jim_GetString(newargs[6], NULL),
			 Jim_GetString(newargs[7], NULL),
			 Jim_GetString(newargs[8], NULL),
			 Jim_GetString(newargs[9], NULL));

	e = jim_jtag_command(interp, 10, newargs);
	if (e != JIM_OK) {
		command_print(cmd_ctx, "%s", Jim_GetString(Jim_GetResult(interp), NULL));
	}
	return e;
}

static int handle_scan_chain_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	jtag_tap_t *tap;

	tap = jtag_all_taps();
	command_print(cmd_ctx, "     TapName            | Enabled |   IdCode      Expected    IrLen IrCap  IrMask Instr     ");
	command_print(cmd_ctx, "---|--------------------|---------|------------|------------|------|------|------|---------");

	while (tap) {
		uint32_t expected, expected_mask, cur_instr, ii;
		expected = buf_get_u32(tap->expected, 0, tap->ir_length);
		expected_mask = buf_get_u32(tap->expected_mask, 0, tap->ir_length);
		cur_instr = buf_get_u32(tap->cur_instr, 0, tap->ir_length);

		command_print(cmd_ctx,
					  "%2d | %-18s |    %c    | 0x%08x | 0x%08x | 0x%02x | 0x%02x | 0x%02x | 0x%02x",
					  tap->abs_chain_position,
					  tap->dotted_name,
					  tap->enabled ? 'Y' : 'n',
					  (unsigned int)(tap->idcode),
					  (unsigned int)(tap->expected_ids_cnt > 0 ? tap->expected_ids[0] : 0),
					  (unsigned int)(tap->ir_length),
					  (unsigned int)(expected),
					  (unsigned int)(expected_mask),
					  (unsigned int)(cur_instr));

		for (ii = 1; ii < tap->expected_ids_cnt; ii++) {
			command_print(cmd_ctx, "   |                    |         |            | 0x%08x |      |      |      |         ",
						  (unsigned int)(tap->expected_ids[ii]));
		}

		tap = tap->next_tap;
	}

	return ERROR_OK;
}

static int handle_reset_config_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int new_cfg = 0;
	int mask = 0;

	if (argc < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* Original versions cared about the order of these tokens:
	 *   reset_config signals [combination [trst_type [srst_type]]]
	 * They also clobbered the previous configuration even on error.
	 *
	 * Here we don't care about the order, and only change values
	 * which have been explicitly specified.
	 */
	for (; argc; argc--, args++) {
		int tmp = 0;
		int m;

		/* signals */
		m = RESET_HAS_TRST | RESET_HAS_SRST;
		if (strcmp(*args, "none") == 0)
			tmp = RESET_NONE;
		else if (strcmp(*args, "trst_only") == 0)
			tmp = RESET_HAS_TRST;
		else if (strcmp(*args, "srst_only") == 0)
			tmp = RESET_HAS_SRST;
		else if (strcmp(*args, "trst_and_srst") == 0)
			tmp = RESET_HAS_TRST | RESET_HAS_SRST;
		else
			m = 0;
		if (mask & m) {
			LOG_ERROR("extra reset_config %s spec (%s)",
					"signal", *args);
			return ERROR_INVALID_ARGUMENTS;
		}
		if (m)
			goto next;

		/* combination (options for broken wiring) */
		m = RESET_SRST_PULLS_TRST | RESET_TRST_PULLS_SRST;
		if (strcmp(*args, "separate") == 0)
			/* separate reset lines - default */;
		else if (strcmp(*args, "srst_pulls_trst") == 0)
			tmp |= RESET_SRST_PULLS_TRST;
		else if (strcmp(*args, "trst_pulls_srst") == 0)
			tmp |= RESET_TRST_PULLS_SRST;
		else if (strcmp(*args, "combined") == 0)
			tmp |= RESET_SRST_PULLS_TRST | RESET_TRST_PULLS_SRST;
		else
			m = 0;
		if (mask & m) {
			LOG_ERROR("extra reset_config %s spec (%s)",
					"combination", *args);
			return ERROR_INVALID_ARGUMENTS;
		}
		if (m)
			goto next;

		/* trst_type (NOP without HAS_TRST) */
		m = RESET_TRST_OPEN_DRAIN;
		if (strcmp(*args, "trst_open_drain") == 0)
			tmp |= RESET_TRST_OPEN_DRAIN;
		else if (strcmp(*args, "trst_push_pull") == 0)
			/* push/pull from adapter - default */;
		else
			m = 0;
		if (mask & m) {
			LOG_ERROR("extra reset_config %s spec (%s)",
					"trst_type", *args);
			return ERROR_INVALID_ARGUMENTS;
		}
		if (m)
			goto next;

		/* srst_type (NOP without HAS_SRST) */
		m |= RESET_SRST_PUSH_PULL;
		if (strcmp(*args, "srst_push_pull") == 0)
			tmp |= RESET_SRST_PUSH_PULL;
		else if (strcmp(*args, "srst_open_drain") == 0)
			/* open drain from adapter - default */;
		else
			m = 0;
		if (mask & m) {
			LOG_ERROR("extra reset_config %s spec (%s)",
					"srst_type", *args);
			return ERROR_INVALID_ARGUMENTS;
		}
		if (m)
			goto next;

		/* caller provided nonsense; fail */
		LOG_ERROR("unknown reset_config flag (%s)", *args);
		return ERROR_INVALID_ARGUMENTS;

next:
		/* Remember the bits which were specified (mask)
		 * and their new values (new_cfg).
		 */
		mask |= m;
		new_cfg |= tmp;
	}

	/* clear previous values of those bits, save new values */
	enum reset_types old_cfg = jtag_get_reset_config();
	old_cfg &= ~mask;
	new_cfg |= old_cfg;
	jtag_set_reset_config(new_cfg);

	return ERROR_OK;
}

static int handle_jtag_nsrst_delay_command(struct command_context_s *cmd_ctx,
		char *cmd, char **args, int argc)
{
	if (argc > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (argc == 1)
	{
		unsigned delay;
		int retval = parse_uint(args[0], &delay);
		if (ERROR_OK != retval)
			return retval;
		jtag_set_nsrst_delay(delay);
	}
	command_print(cmd_ctx, "jtag_nsrst_delay: %u", jtag_get_nsrst_delay());
	return ERROR_OK;
}

static int handle_jtag_ntrst_delay_command(struct command_context_s *cmd_ctx,
		char *cmd, char **args, int argc)
{
	if (argc > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (argc == 1)
	{
		unsigned delay;
		int retval = parse_uint(args[0], &delay);
		if (ERROR_OK != retval)
			return retval;
		jtag_set_ntrst_delay(delay);
	}
	command_print(cmd_ctx, "jtag_ntrst_delay: %u", jtag_get_ntrst_delay());
	return ERROR_OK;
}

static int handle_jtag_speed_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int retval = ERROR_OK;

	if (argc > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (argc == 1)
	{
		LOG_DEBUG("handle jtag speed");

		unsigned cur_speed = 0;
		int retval = parse_uint(args[0], &cur_speed);
		if (ERROR_OK != retval)
			return retval;
		retval = jtag_set_speed(cur_speed);

	}
	command_print(cmd_ctx, "jtag_speed: %d", jtag_get_speed());

	return retval;
}

static int handle_jtag_khz_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	int retval = ERROR_OK;
	if (argc == 1)
	{
		unsigned khz = 0;
		int retval = parse_uint(args[0], &khz);
		if (ERROR_OK != retval)
			return retval;
		retval = jtag_config_khz(khz);
		if (ERROR_OK != retval)
			return retval;
	}

	int cur_speed = jtag_get_speed_khz();
	retval = jtag_get_speed_readable(&cur_speed);
	if (ERROR_OK != retval)
		return retval;

	if (cur_speed)
		command_print(cmd_ctx, "%d kHz", cur_speed);
	else
		command_print(cmd_ctx, "RCLK - adaptive");

	return retval;
}

static int handle_jtag_reset_command(struct command_context_s *cmd_ctx,
		char *cmd, char **args, int argc)
{
	if (argc != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	int trst = -1;
	if (args[0][0] == '1')
		trst = 1;
	else if (args[0][0] == '0')
		trst = 0;
	else
		return ERROR_COMMAND_SYNTAX_ERROR;

	int srst = -1;
	if (args[1][0] == '1')
		srst = 1;
	else if (args[1][0] == '0')
		srst = 0;
	else
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (jtag_interface_init(cmd_ctx) != ERROR_OK)
		return ERROR_JTAG_INIT_FAILED;

	jtag_add_reset(trst, srst);
	jtag_execute_queue();

	return ERROR_OK;
}

static int handle_runtest_command(struct command_context_s *cmd_ctx,
		char *cmd, char **args, int argc)
{
	if (argc != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	unsigned num_clocks;
	int retval = parse_uint(args[0], &num_clocks);
	if (ERROR_OK != retval)
		return retval;

	jtag_add_runtest(num_clocks, TAP_IDLE);
	jtag_execute_queue();

	return ERROR_OK;
}

/*
 * For "irscan" or "drscan" commands, the "end" (really, "next") state
 * should be stable ... and *NOT* a shift state, otherwise free-running
 * jtag clocks could change the values latched by the update state.
 */
static bool scan_is_safe(tap_state_t state)
{
	switch (state)
	{
	case TAP_RESET:
	case TAP_IDLE:
	case TAP_DRPAUSE:
	case TAP_IRPAUSE:
		return true;
	default:
		return false;
	}
}


static int handle_irscan_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	int i;
	scan_field_t *fields;
	jtag_tap_t *tap;
	tap_state_t endstate;

	if ((argc < 2) || (argc % 2))
	{
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	/* optional "-endstate" "statename" at the end of the arguments,
	 * so that e.g. IRPAUSE can let us load the data register before
	 * entering RUN/IDLE to execute the instruction we load here.
	 */
	endstate = TAP_IDLE;

	if (argc >= 4) {
		/* have at least one pair of numbers. */
		/* is last pair the magic text? */
		if (0 == strcmp("-endstate", args[ argc - 2 ])) {
			const char *cpA;
			const char *cpS;
			cpA = args[ argc-1 ];
			for (endstate = 0 ; endstate < TAP_NUM_STATES ; endstate++) {
				cpS = tap_state_name(endstate);
				if (0 == strcmp(cpA, cpS)) {
					break;
				}
			}
			if (endstate >= TAP_NUM_STATES) {
				return ERROR_COMMAND_SYNTAX_ERROR;
			} else {
				if (!scan_is_safe(endstate))
					LOG_WARNING("irscan with unsafe "
							"endstate \"%s\"", cpA);
				/* found - remove the last 2 args */
				argc -= 2;
			}
		}
	}

	int num_fields = argc / 2;
	size_t fields_len = sizeof(scan_field_t) * num_fields;
	fields = malloc(fields_len);
	memset(fields, 0, fields_len);

	int retval;
	for (i = 0; i < num_fields; i++)
	{
		tap = jtag_tap_by_string(args[i*2]);
		if (tap == NULL)
		{
			command_print(cmd_ctx, "Tap: %s unknown", args[i*2]);
			return ERROR_FAIL;
		}
		int field_size = tap->ir_length;
		fields[i].tap = tap;
		fields[i].num_bits = field_size;
		fields[i].out_value = malloc(CEIL(field_size, 8));

		uint32_t value;
		retval = parse_u32(args[i * 2 + 1], &value);
		if (ERROR_OK != retval)
			goto error_return;
		buf_set_u32(fields[i].out_value, 0, field_size, value);
		fields[i].in_value = NULL;
	}

	/* did we have an endstate? */
	jtag_add_ir_scan(num_fields, fields, endstate);

	retval = jtag_execute_queue();

error_return:
	for (i = 0; i < num_fields; i++)
	{
		if (NULL != fields[i].out_value)
			free(fields[i].out_value);
	}

	free (fields);

	return retval;
}

static int Jim_Command_drscan(Jim_Interp *interp, int argc, Jim_Obj *const *args)
{
	int retval;
	scan_field_t *fields;
	int num_fields;
	int field_count = 0;
	int i, e;
	jtag_tap_t *tap;
	tap_state_t endstate;

	/* args[1] = device
	 * args[2] = num_bits
	 * args[3] = hex string
	 * ... repeat num bits and hex string ...
	 *
	 * .. optionally:
	*     args[N-2] = "-endstate"
	 *     args[N-1] = statename
	 */
	if ((argc < 4) || ((argc % 2) != 0))
	{
		Jim_WrongNumArgs(interp, 1, args, "wrong arguments");
		return JIM_ERR;
	}

	endstate = TAP_IDLE;

	script_debug(interp, "drscan", argc, args);

	/* validate arguments as numbers */
	e = JIM_OK;
	for (i = 2; i < argc; i += 2)
	{
		long bits;
		const char *cp;

		e = Jim_GetLong(interp, args[i], &bits);
		/* If valid - try next arg */
		if (e == JIM_OK) {
			continue;
		}

		/* Not valid.. are we at the end? */
		if (((i + 2) != argc)) {
			/* nope, then error */
			return e;
		}

		/* it could be: "-endstate FOO"
		 * e.g. DRPAUSE so we can issue more instructions
		 * before entering RUN/IDLE and executing them.
		 */

		/* get arg as a string. */
		cp = Jim_GetString(args[i], NULL);
		/* is it the magic? */
		if (0 == strcmp("-endstate", cp)) {
			/* is the statename valid? */
			cp = Jim_GetString(args[i + 1], NULL);

			/* see if it is a valid state name */
			endstate = tap_state_by_name(cp);
			if (endstate < 0) {
				/* update the error message */
				Jim_SetResult_sprintf(interp,"endstate: %s invalid", cp);
			} else {
				if (!scan_is_safe(endstate))
					LOG_WARNING("drscan with unsafe "
							"endstate \"%s\"", cp);

				/* valid - so clear the error */
				e = JIM_OK;
				/* and remove the last 2 args */
				argc -= 2;
			}
		}

		/* Still an error? */
		if (e != JIM_OK) {
			return e; /* too bad */
		}
	} /* validate args */

	tap = jtag_tap_by_jim_obj(interp, args[1]);
	if (tap == NULL) {
		return JIM_ERR;
	}

	num_fields = (argc-2)/2;
	fields = malloc(sizeof(scan_field_t) * num_fields);
	for (i = 2; i < argc; i += 2)
	{
		long bits;
		int len;
		const char *str;

		Jim_GetLong(interp, args[i], &bits);
		str = Jim_GetString(args[i + 1], &len);

		fields[field_count].tap = tap;
		fields[field_count].num_bits = bits;
		fields[field_count].out_value = malloc(CEIL(bits, 8));
		str_to_buf(str, len, fields[field_count].out_value, bits, 0);
		fields[field_count].in_value = fields[field_count].out_value;
		field_count++;
	}

	jtag_add_dr_scan(num_fields, fields, endstate);

	retval = jtag_execute_queue();
	if (retval != ERROR_OK)
	{
		Jim_SetResultString(interp, "drscan: jtag execute failed",-1);
		return JIM_ERR;
	}

	field_count = 0;
	Jim_Obj *list = Jim_NewListObj(interp, NULL, 0);
	for (i = 2; i < argc; i += 2)
	{
		long bits;
		char *str;

		Jim_GetLong(interp, args[i], &bits);
		str = buf_to_str(fields[field_count].in_value, bits, 16);
		free(fields[field_count].out_value);

		Jim_ListAppendElement(interp, list, Jim_NewStringObj(interp, str, strlen(str)));
		free(str);
		field_count++;
	}

	Jim_SetResult(interp, list);

	free(fields);

	return JIM_OK;
}


static int Jim_Command_pathmove(Jim_Interp *interp, int argc, Jim_Obj *const *args)
{
	tap_state_t states[8];

	if ((argc < 2) || ((size_t)argc > (sizeof(states)/sizeof(*states) + 1)))
	{
		Jim_WrongNumArgs(interp, 1, args, "wrong arguments");
		return JIM_ERR;
	}

	script_debug(interp, "pathmove", argc, args);

	int i;
	for (i = 0; i < argc-1; i++)
	{
		const char *cp;
		cp = Jim_GetString(args[i + 1], NULL);
		states[i] = tap_state_by_name(cp);
		if (states[i] < 0)
		{
			/* update the error message */
			Jim_SetResult_sprintf(interp,"endstate: %s invalid", cp);
			return JIM_ERR;
		}
	}

	if ((jtag_add_statemove(states[0]) != ERROR_OK) || (jtag_execute_queue()!= ERROR_OK))
	{
		Jim_SetResultString(interp, "pathmove: jtag execute failed",-1);
		return JIM_ERR;
	}

	jtag_add_pathmove(argc-2, states + 1);

	if (jtag_execute_queue()!= ERROR_OK)
	{
		Jim_SetResultString(interp, "pathmove: failed",-1);
		return JIM_ERR;
	}

	return JIM_OK;
}


static int Jim_Command_flush_count(Jim_Interp *interp, int argc, Jim_Obj *const *args)
{
	script_debug(interp, "flush_count", argc, args);

	Jim_SetResult(interp, Jim_NewIntObj(interp, jtag_get_flush_queue_count()));

	return JIM_OK;
}


static int handle_verify_ircapture_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (argc == 1)
	{
		if (strcmp(args[0], "enable") == 0)
			jtag_set_verify_capture_ir(true);
		else if (strcmp(args[0], "disable") == 0)
			jtag_set_verify_capture_ir(false);
		else
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	const char *status = jtag_will_verify_capture_ir() ? "enabled": "disabled";
	command_print(cmd_ctx, "verify Capture-IR is %s", status);

	return ERROR_OK;
}

static int handle_verify_jtag_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (argc == 1)
	{
		if (strcmp(args[0], "enable") == 0)
			jtag_set_verify(true);
		else if (strcmp(args[0], "disable") == 0)
			jtag_set_verify(false);
		else
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	const char *status = jtag_will_verify() ? "enabled": "disabled";
	command_print(cmd_ctx, "verify jtag capture is %s", status);

	return ERROR_OK;
}

static int handle_tms_sequence_command(struct command_context_s *cmd_ctx, char *cmd, char **args, int argc)
{
	if (argc > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (argc == 1)
	{
		bool use_new_table;
		if (strcmp(args[0], "short") == 0)
			use_new_table = true;
		else if (strcmp(args[0], "long") == 0)
			use_new_table = false;
		else
			return ERROR_COMMAND_SYNTAX_ERROR;

		tap_use_new_tms_table(use_new_table);
	}

	command_print(cmd_ctx, "tms sequence is  %s",
			tap_uses_new_tms_table() ? "short": "long");

	return ERROR_OK;
}
