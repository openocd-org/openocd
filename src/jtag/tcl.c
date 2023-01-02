// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   Copyright (C) 2007-2010 Øyvind Harboe                                 *
 *   oyvind.harboe@zylin.com                                               *
 *                                                                         *
 *   Copyright (C) 2009 SoftPLC Corporation                                *
 *       http://softplc.com                                                *
 *   dick@softplc.com                                                      *
 *                                                                         *
 *   Copyright (C) 2009 Zachary T Welch                                    *
 *   zw@superlucidity.net                                                  *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "adapter.h"
#include "jtag.h"
#include "swd.h"
#include "minidriver.h"
#include "interface.h"
#include "interfaces.h"
#include "tcl.h"

#ifdef HAVE_STRINGS_H
#include <strings.h>
#endif

#include <helper/time_support.h>
#include "transport/transport.h"

/**
 * @file
 * Holds support for accessing JTAG-specific mechanisms from TCl scripts.
 */

static const struct jim_nvp nvp_jtag_tap_event[] = {
	{ .value = JTAG_TRST_ASSERTED,          .name = "post-reset" },
	{ .value = JTAG_TAP_EVENT_SETUP,        .name = "setup" },
	{ .value = JTAG_TAP_EVENT_ENABLE,       .name = "tap-enable" },
	{ .value = JTAG_TAP_EVENT_DISABLE,      .name = "tap-disable" },

	{ .name = NULL, .value = -1 }
};

struct jtag_tap *jtag_tap_by_jim_obj(Jim_Interp *interp, Jim_Obj *o)
{
	const char *cp = Jim_GetString(o, NULL);
	struct jtag_tap *t = cp ? jtag_tap_by_string(cp) : NULL;
	if (!cp)
		cp = "(unknown)";
	if (!t)
		Jim_SetResultFormatted(interp, "Tap '%s' could not be found", cp);
	return t;
}

static bool scan_is_safe(tap_state_t state)
{
	switch (state) {
	    case TAP_RESET:
	    case TAP_IDLE:
	    case TAP_DRPAUSE:
	    case TAP_IRPAUSE:
		    return true;
	    default:
		    return false;
	}
}

static COMMAND_HELPER(handle_jtag_command_drscan_fields, struct scan_field *fields)
{
	unsigned int field_count = 0;
	for (unsigned int i = 1; i < CMD_ARGC; i += 2) {
		unsigned int bits;
		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[i], bits);
		fields[field_count].num_bits = bits;

		void *t = malloc(DIV_ROUND_UP(bits, 8));
		if (!t) {
			LOG_ERROR("Out of memory");
			return ERROR_FAIL;
		}
		fields[field_count].out_value = t;
		str_to_buf(CMD_ARGV[i + 1], strlen(CMD_ARGV[i + 1]), t, bits, 0);
		fields[field_count].in_value = t;
		field_count++;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_jtag_command_drscan)
{
	/*
	 * CMD_ARGV[0] = device
	 * CMD_ARGV[1] = num_bits
	 * CMD_ARGV[2] = hex string
	 * ... repeat num bits and hex string ...
	 *
	 * ... optionally:
	 * CMD_ARGV[CMD_ARGC-2] = "-endstate"
	 * CMD_ARGV[CMD_ARGC-1] = statename
	 */

	if (CMD_ARGC < 3 || (CMD_ARGC % 2) != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct jtag_tap *tap = jtag_tap_by_string(CMD_ARGV[0]);
	if (!tap) {
		command_print(CMD, "Tap '%s' could not be found", CMD_ARGV[0]);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	tap_state_t endstate = TAP_IDLE;
	if (CMD_ARGC > 3 && !strcmp("-endstate", CMD_ARGV[CMD_ARGC - 2])) {
		const char *state_name = CMD_ARGV[CMD_ARGC - 1];
		endstate = tap_state_by_name(state_name);
		if (endstate < 0) {
			command_print(CMD, "endstate: %s invalid", state_name);
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}

		if (!scan_is_safe(endstate))
			LOG_WARNING("drscan with unsafe endstate \"%s\"", state_name);

		CMD_ARGC -= 2;
	}

	unsigned int num_fields = (CMD_ARGC - 1) / 2;
	struct scan_field *fields = calloc(num_fields, sizeof(struct scan_field));
	if (!fields) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	int retval = CALL_COMMAND_HANDLER(handle_jtag_command_drscan_fields, fields);
	if (retval != ERROR_OK)
		goto fail;

	jtag_add_dr_scan(tap, num_fields, fields, endstate);

	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		command_print(CMD, "drscan: jtag execute failed");
		goto fail;
	}

	for (unsigned int i = 0; i < num_fields; i++) {
		char *str = buf_to_hex_str(fields[i].in_value, fields[i].num_bits);
		command_print(CMD, "%s", str);
		free(str);
	}

fail:
	for (unsigned int i = 0; i < num_fields; i++)
		free(fields[i].in_value);
	free(fields);

	return retval;
}

COMMAND_HANDLER(handle_jtag_command_pathmove)
{
	tap_state_t states[8];

	if (CMD_ARGC < 1 || CMD_ARGC > ARRAY_SIZE(states))
		return ERROR_COMMAND_SYNTAX_ERROR;

	for (unsigned int i = 0; i < CMD_ARGC; i++) {
		states[i] = tap_state_by_name(CMD_ARGV[i]);
		if (states[i] < 0) {
			command_print(CMD, "endstate: %s invalid", CMD_ARGV[i]);
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
	}

	int retval = jtag_add_statemove(states[0]);
	if (retval == ERROR_OK)
		retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		command_print(CMD, "pathmove: jtag execute failed");
		return retval;
	}

	jtag_add_pathmove(CMD_ARGC - 1, states + 1);
	retval = jtag_execute_queue();
	if (retval != ERROR_OK) {
		command_print(CMD, "pathmove: failed");
		return retval;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_jtag_flush_count)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	int count = jtag_get_flush_queue_count();
	command_print_sameline(CMD, "%d", count);

	return ERROR_OK;
}

/* REVISIT Just what about these should "move" ... ?
 * These registrations, into the main JTAG table?
 *
 * There's a minor compatibility issue, these all show up twice;
 * that's not desirable:
 *  - jtag drscan ... NOT DOCUMENTED!
 *  - drscan ...
 *
 * The "irscan" command (for example) doesn't show twice.
 */
static const struct command_registration jtag_command_handlers_to_move[] = {
	{
		.name = "drscan",
		.mode = COMMAND_EXEC,
		.handler = handle_jtag_command_drscan,
		.help = "Execute Data Register (DR) scan for one TAP.  "
			"Other TAPs must be in BYPASS mode.",
		.usage = "tap_name [num_bits value]* ['-endstate' state_name]",
	},
	{
		.name = "flush_count",
		.mode = COMMAND_EXEC,
		.handler = handle_jtag_flush_count,
		.help = "Returns the number of times the JTAG queue "
			"has been flushed.",
		.usage = "",
	},
	{
		.name = "pathmove",
		.mode = COMMAND_EXEC,
		.handler = handle_jtag_command_pathmove,
		.usage = "start_state state1 [state2 [state3 ...]]",
		.help = "Move JTAG state machine from current state "
			"(start_state) to state1, then state2, state3, etc.",
	},
	COMMAND_REGISTRATION_DONE
};


enum jtag_tap_cfg_param {
	JCFG_EVENT,
	JCFG_IDCODE,
};

static struct jim_nvp nvp_config_opts[] = {
	{ .name = "-event",      .value = JCFG_EVENT },
	{ .name = "-idcode",     .value = JCFG_IDCODE },

	{ .name = NULL,          .value = -1 }
};

static int jtag_tap_configure_event(struct jim_getopt_info *goi, struct jtag_tap *tap)
{
	if (goi->argc == 0) {
		Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv, "-event <event-name> ...");
		return JIM_ERR;
	}

	struct jim_nvp *n;
	int e = jim_getopt_nvp(goi, nvp_jtag_tap_event, &n);
	if (e != JIM_OK) {
		jim_getopt_nvp_unknown(goi, nvp_jtag_tap_event, 1);
		return e;
	}

	if (goi->isconfigure) {
		if (goi->argc != 1) {
			Jim_WrongNumArgs(goi->interp,
				goi->argc,
				goi->argv,
				"-event <event-name> <event-body>");
			return JIM_ERR;
		}
	} else {
		if (goi->argc != 0) {
			Jim_WrongNumArgs(goi->interp, goi->argc, goi->argv, "-event <event-name>");
			return JIM_ERR;
		}
	}

	struct jtag_tap_event_action *jteap  = tap->event_action;
	/* replace existing event body */
	bool found = false;
	while (jteap) {
		if (jteap->event == (enum jtag_event)n->value) {
			found = true;
			break;
		}
		jteap = jteap->next;
	}

	Jim_SetEmptyResult(goi->interp);

	if (goi->isconfigure) {
		if (!found)
			jteap = calloc(1, sizeof(*jteap));
		else if (jteap->body)
			Jim_DecrRefCount(goi->interp, jteap->body);

		jteap->interp = goi->interp;
		jteap->event = n->value;

		Jim_Obj *o;
		jim_getopt_obj(goi, &o);
		jteap->body = Jim_DuplicateObj(goi->interp, o);
		Jim_IncrRefCount(jteap->body);

		if (!found) {
			/* add to head of event list */
			jteap->next = tap->event_action;
			tap->event_action = jteap;
		}
	} else if (found) {
		jteap->interp = goi->interp;
		Jim_SetResult(goi->interp,
			Jim_DuplicateObj(goi->interp, jteap->body));
	}
	return JIM_OK;
}

static int jtag_tap_configure_cmd(struct jim_getopt_info *goi, struct jtag_tap *tap)
{
	/* parse config or cget options */
	while (goi->argc > 0) {
		Jim_SetEmptyResult(goi->interp);

		struct jim_nvp *n;
		int e = jim_getopt_nvp(goi, nvp_config_opts, &n);
		if (e != JIM_OK) {
			jim_getopt_nvp_unknown(goi, nvp_config_opts, 0);
			return e;
		}

		switch (n->value) {
			case JCFG_EVENT:
				e = jtag_tap_configure_event(goi, tap);
				if (e != JIM_OK)
					return e;
				break;
			case JCFG_IDCODE:
				if (goi->isconfigure) {
					Jim_SetResultFormatted(goi->interp,
							"not settable: %s", n->name);
					return JIM_ERR;
				} else {
					if (goi->argc != 0) {
						Jim_WrongNumArgs(goi->interp,
								goi->argc, goi->argv,
								"NO PARAMS");
						return JIM_ERR;
					}
				}
				Jim_SetResult(goi->interp, Jim_NewIntObj(goi->interp, tap->idcode));
				break;
			default:
				Jim_SetResultFormatted(goi->interp, "unknown value: %s", n->name);
				return JIM_ERR;
		}
	}

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

static int jim_newtap_expected_id(struct jim_nvp *n, struct jim_getopt_info *goi,
	struct jtag_tap *tap)
{
	jim_wide w;
	int e = jim_getopt_wide(goi, &w);
	if (e != JIM_OK) {
		Jim_SetResultFormatted(goi->interp, "option: %s bad parameter", n->name);
		return e;
	}

	uint32_t *p = realloc(tap->expected_ids,
			      (tap->expected_ids_cnt + 1) * sizeof(uint32_t));
	if (!p) {
		Jim_SetResultFormatted(goi->interp, "no memory");
		return JIM_ERR;
	}

	tap->expected_ids = p;
	tap->expected_ids[tap->expected_ids_cnt++] = w;

	return JIM_OK;
}

#define NTAP_OPT_IRLEN     0
#define NTAP_OPT_IRMASK    1
#define NTAP_OPT_IRCAPTURE 2
#define NTAP_OPT_ENABLED   3
#define NTAP_OPT_DISABLED  4
#define NTAP_OPT_EXPECTED_ID 5
#define NTAP_OPT_VERSION   6
#define NTAP_OPT_BYPASS    7

static int jim_newtap_ir_param(struct jim_nvp *n, struct jim_getopt_info *goi,
	struct jtag_tap *tap)
{
	jim_wide w;
	int e = jim_getopt_wide(goi, &w);
	if (e != JIM_OK) {
		Jim_SetResultFormatted(goi->interp,
			"option: %s bad parameter", n->name);
		return e;
	}
	switch (n->value) {
	    case NTAP_OPT_IRLEN:
		    if (w > (jim_wide) (8 * sizeof(tap->ir_capture_value))) {
			    LOG_WARNING("%s: huge IR length %d",
				    tap->dotted_name, (int) w);
		    }
		    tap->ir_length = w;
		    break;
	    case NTAP_OPT_IRMASK:
		    if (is_bad_irval(tap->ir_length, w)) {
			    LOG_ERROR("%s: IR mask %x too big",
				    tap->dotted_name,
				    (int) w);
			    return JIM_ERR;
		    }
		    if ((w & 3) != 3)
			    LOG_WARNING("%s: nonstandard IR mask", tap->dotted_name);
		    tap->ir_capture_mask = w;
		    break;
	    case NTAP_OPT_IRCAPTURE:
		    if (is_bad_irval(tap->ir_length, w)) {
			    LOG_ERROR("%s: IR capture %x too big",
				    tap->dotted_name, (int) w);
			    return JIM_ERR;
		    }
		    if ((w & 3) != 1)
			    LOG_WARNING("%s: nonstandard IR value",
				    tap->dotted_name);
		    tap->ir_capture_value = w;
		    break;
	    default:
		    return JIM_ERR;
	}
	return JIM_OK;
}

static int jim_newtap_cmd(struct jim_getopt_info *goi)
{
	struct jtag_tap *tap;
	int x;
	int e;
	struct jim_nvp *n;
	char *cp;
	const struct jim_nvp opts[] = {
		{ .name = "-irlen",       .value = NTAP_OPT_IRLEN },
		{ .name = "-irmask",       .value = NTAP_OPT_IRMASK },
		{ .name = "-ircapture",       .value = NTAP_OPT_IRCAPTURE },
		{ .name = "-enable",       .value = NTAP_OPT_ENABLED },
		{ .name = "-disable",       .value = NTAP_OPT_DISABLED },
		{ .name = "-expected-id",       .value = NTAP_OPT_EXPECTED_ID },
		{ .name = "-ignore-version",       .value = NTAP_OPT_VERSION },
		{ .name = "-ignore-bypass",       .value = NTAP_OPT_BYPASS },
		{ .name = NULL,       .value = -1 },
	};

	tap = calloc(1, sizeof(struct jtag_tap));
	if (!tap) {
		Jim_SetResultFormatted(goi->interp, "no memory");
		return JIM_ERR;
	}

	/*
	 * we expect CHIP + TAP + OPTIONS
	 * */
	if (goi->argc < 3) {
		Jim_SetResultFormatted(goi->interp, "Missing CHIP TAP OPTIONS ....");
		free(tap);
		return JIM_ERR;
	}

	const char *tmp;
	jim_getopt_string(goi, &tmp, NULL);
	tap->chip = strdup(tmp);

	jim_getopt_string(goi, &tmp, NULL);
	tap->tapname = strdup(tmp);

	/* name + dot + name + null */
	x = strlen(tap->chip) + 1 + strlen(tap->tapname) + 1;
	cp = malloc(x);
	sprintf(cp, "%s.%s", tap->chip, tap->tapname);
	tap->dotted_name = cp;

	LOG_DEBUG("Creating New Tap, Chip: %s, Tap: %s, Dotted: %s, %d params",
		tap->chip, tap->tapname, tap->dotted_name, goi->argc);

	/* IEEE specifies that the two LSBs of an IR scan are 01, so make
	 * that the default.  The "-ircapture" and "-irmask" options are only
	 * needed to cope with nonstandard TAPs, or to specify more bits.
	 */
	tap->ir_capture_mask = 0x03;
	tap->ir_capture_value = 0x01;

	while (goi->argc) {
		e = jim_getopt_nvp(goi, opts, &n);
		if (e != JIM_OK) {
			jim_getopt_nvp_unknown(goi, opts, 0);
			free(cp);
			free(tap);
			return e;
		}
		LOG_DEBUG("Processing option: %s", n->name);
		switch (n->value) {
		    case NTAP_OPT_ENABLED:
			    tap->disabled_after_reset = false;
			    break;
		    case NTAP_OPT_DISABLED:
			    tap->disabled_after_reset = true;
			    break;
		    case NTAP_OPT_EXPECTED_ID:
			    e = jim_newtap_expected_id(n, goi, tap);
			    if (e != JIM_OK) {
				    free(cp);
				    free(tap);
				    return e;
			    }
			    break;
		    case NTAP_OPT_IRLEN:
		    case NTAP_OPT_IRMASK:
		    case NTAP_OPT_IRCAPTURE:
			    e = jim_newtap_ir_param(n, goi, tap);
			    if (e != JIM_OK) {
				    free(cp);
				    free(tap);
				    return e;
			    }
			    break;
		    case NTAP_OPT_VERSION:
			    tap->ignore_version = true;
			    break;
		    case NTAP_OPT_BYPASS:
			    tap->ignore_bypass = true;
			    break;
		}	/* switch (n->value) */
	}	/* while (goi->argc) */

	/* default is enabled-after-reset */
	tap->enabled = !tap->disabled_after_reset;

	/* Did all the required option bits get cleared? */
	if (!transport_is_jtag() || tap->ir_length != 0) {
		jtag_tap_init(tap);
		return JIM_OK;
	}

	Jim_SetResultFormatted(goi->interp,
		"newtap: %s missing IR length",
		tap->dotted_name);
	jtag_tap_free(tap);
	return JIM_ERR;
}

static void jtag_tap_handle_event(struct jtag_tap *tap, enum jtag_event e)
{
	struct jtag_tap_event_action *jteap;
	int retval;

	for (jteap = tap->event_action; jteap; jteap = jteap->next) {
		if (jteap->event != e)
			continue;

		struct jim_nvp *nvp = jim_nvp_value2name_simple(nvp_jtag_tap_event, e);
		LOG_DEBUG("JTAG tap: %s event: %d (%s)\n\taction: %s",
			tap->dotted_name, e, nvp->name,
			Jim_GetString(jteap->body, NULL));

		retval = Jim_EvalObj(jteap->interp, jteap->body);
		if (retval == JIM_RETURN)
			retval = jteap->interp->returnCode;

		if (retval != JIM_OK) {
			Jim_MakeErrorMessage(jteap->interp);
			LOG_USER("%s", Jim_GetString(Jim_GetResult(jteap->interp), NULL));
			continue;
		}

		switch (e) {
		    case JTAG_TAP_EVENT_ENABLE:
		    case JTAG_TAP_EVENT_DISABLE:
				/* NOTE:  we currently assume the handlers
				 * can't fail.  Right here is where we should
				 * really be verifying the scan chains ...
				 */
			    tap->enabled = (e == JTAG_TAP_EVENT_ENABLE);
			    LOG_INFO("JTAG tap: %s %s", tap->dotted_name,
				tap->enabled ? "enabled" : "disabled");
			    break;
		    default:
			    break;
		}
	}
}

COMMAND_HANDLER(handle_jtag_arp_init)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	return jtag_init_inner(CMD_CTX);
}

COMMAND_HANDLER(handle_jtag_arp_init_reset)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (transport_is_jtag())
		return jtag_init_reset(CMD_CTX);

	if (transport_is_swd())
		return swd_init_reset(CMD_CTX);

	return ERROR_OK;
}

int jim_jtag_newtap(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	struct jim_getopt_info goi;
	jim_getopt_setup(&goi, interp, argc-1, argv + 1);
	return jim_newtap_cmd(&goi);
}

static bool jtag_tap_enable(struct jtag_tap *t)
{
	if (t->enabled)
		return false;
	jtag_tap_handle_event(t, JTAG_TAP_EVENT_ENABLE);
	if (!t->enabled)
		return false;

	/* FIXME add JTAG sanity checks, w/o TLR
	 *  - scan chain length grew by one (this)
	 *  - IDs and IR lengths are as expected
	 */
	jtag_call_event_callbacks(JTAG_TAP_EVENT_ENABLE);
	return true;
}
static bool jtag_tap_disable(struct jtag_tap *t)
{
	if (!t->enabled)
		return false;
	jtag_tap_handle_event(t, JTAG_TAP_EVENT_DISABLE);
	if (t->enabled)
		return false;

	/* FIXME add JTAG sanity checks, w/o TLR
	 *  - scan chain length shrank by one (this)
	 *  - IDs and IR lengths are as expected
	 */
	jtag_call_event_callbacks(JTAG_TAP_EVENT_DISABLE);
	return true;
}

int jim_jtag_tap_enabler(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	struct command *c = jim_to_command(interp);
	const char *cmd_name = c->name;
	struct jim_getopt_info goi;
	jim_getopt_setup(&goi, interp, argc-1, argv + 1);
	if (goi.argc != 1) {
		Jim_SetResultFormatted(goi.interp, "usage: %s <name>", cmd_name);
		return JIM_ERR;
	}

	struct jtag_tap *t;

	t = jtag_tap_by_jim_obj(goi.interp, goi.argv[0]);
	if (!t)
		return JIM_ERR;

	if (strcasecmp(cmd_name, "tapisenabled") == 0) {
		/* do nothing, just return the value */
	} else if (strcasecmp(cmd_name, "tapenable") == 0) {
		if (!jtag_tap_enable(t)) {
			LOG_WARNING("failed to enable tap %s", t->dotted_name);
			return JIM_ERR;
		}
	} else if (strcasecmp(cmd_name, "tapdisable") == 0) {
		if (!jtag_tap_disable(t)) {
			LOG_WARNING("failed to disable tap %s", t->dotted_name);
			return JIM_ERR;
		}
	} else {
		LOG_ERROR("command '%s' unknown", cmd_name);
		return JIM_ERR;
	}
	bool e = t->enabled;
	Jim_SetResult(goi.interp, Jim_NewIntObj(goi.interp, e));
	return JIM_OK;
}

int jim_jtag_configure(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	struct command *c = jim_to_command(interp);
	const char *cmd_name = c->name;
	struct jim_getopt_info goi;
	jim_getopt_setup(&goi, interp, argc-1, argv + 1);
	goi.isconfigure = !strcmp(cmd_name, "configure");
	if (goi.argc < 2 + goi.isconfigure) {
		Jim_WrongNumArgs(goi.interp, 0, NULL,
			"<tap_name> <attribute> ...");
		return JIM_ERR;
	}

	struct jtag_tap *t;

	Jim_Obj *o;
	jim_getopt_obj(&goi, &o);
	t = jtag_tap_by_jim_obj(goi.interp, o);
	if (!t)
		return JIM_ERR;

	return jtag_tap_configure_cmd(&goi, t);
}

COMMAND_HANDLER(handle_jtag_names)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	for (struct jtag_tap *tap = jtag_all_taps(); tap; tap = tap->next_tap)
		command_print(CMD, "%s", tap->dotted_name);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_jtag_init_command)
{
	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	static bool jtag_initialized;
	if (jtag_initialized) {
		LOG_INFO("'jtag init' has already been called");
		return ERROR_OK;
	}
	jtag_initialized = true;

	LOG_DEBUG("Initializing jtag devices...");
	return jtag_init(CMD_CTX);
}

static const struct command_registration jtag_subcommand_handlers[] = {
	{
		.name = "init",
		.mode = COMMAND_ANY,
		.handler = handle_jtag_init_command,
		.help = "initialize jtag scan chain",
		.usage = ""
	},
	{
		.name = "arp_init",
		.mode = COMMAND_ANY,
		.handler = handle_jtag_arp_init,
		.help = "Validates JTAG scan chain against the list of "
			"declared TAPs using just the four standard JTAG "
			"signals.",
		.usage = "",
	},
	{
		.name = "arp_init-reset",
		.mode = COMMAND_ANY,
		.handler = handle_jtag_arp_init_reset,
		.help = "Uses TRST and SRST to try resetting everything on "
			"the JTAG scan chain, then performs 'jtag arp_init'.",
		.usage = "",
	},
	{
		.name = "newtap",
		.mode = COMMAND_CONFIG,
		.jim_handler = jim_jtag_newtap,
		.help = "Create a new TAP instance named basename.tap_type, "
			"and appends it to the scan chain.",
		.usage = "basename tap_type '-irlen' count "
			"['-enable'|'-disable'] "
			"['-expected_id' number] "
			"['-ignore-version'] "
			"['-ignore-bypass'] "
			"['-ircapture' number] "
			"['-mask' number]",
	},
	{
		.name = "tapisenabled",
		.mode = COMMAND_EXEC,
		.jim_handler = jim_jtag_tap_enabler,
		.help = "Returns a Tcl boolean (0/1) indicating whether "
			"the TAP is enabled (1) or not (0).",
		.usage = "tap_name",
	},
	{
		.name = "tapenable",
		.mode = COMMAND_EXEC,
		.jim_handler = jim_jtag_tap_enabler,
		.help = "Try to enable the specified TAP using the "
			"'tap-enable' TAP event.",
		.usage = "tap_name",
	},
	{
		.name = "tapdisable",
		.mode = COMMAND_EXEC,
		.jim_handler = jim_jtag_tap_enabler,
		.help = "Try to disable the specified TAP using the "
			"'tap-disable' TAP event.",
		.usage = "tap_name",
	},
	{
		.name = "configure",
		.mode = COMMAND_ANY,
		.jim_handler = jim_jtag_configure,
		.help = "Provide a Tcl handler for the specified "
			"TAP event.",
		.usage = "tap_name '-event' event_name handler",
	},
	{
		.name = "cget",
		.mode = COMMAND_EXEC,
		.jim_handler = jim_jtag_configure,
		.help = "Return any Tcl handler for the specified "
			"TAP event.",
		.usage = "tap_name '-event' event_name",
	},
	{
		.name = "names",
		.mode = COMMAND_ANY,
		.handler = handle_jtag_names,
		.help = "Returns list of all JTAG tap names.",
		.usage = "",
	},
	{
		.chain = jtag_command_handlers_to_move,
	},
	COMMAND_REGISTRATION_DONE
};

void jtag_notify_event(enum jtag_event event)
{
	struct jtag_tap *tap;

	for (tap = jtag_all_taps(); tap; tap = tap->next_tap)
		jtag_tap_handle_event(tap, event);
}


COMMAND_HANDLER(handle_scan_chain_command)
{
	struct jtag_tap *tap;
	char expected_id[12];

	tap = jtag_all_taps();
	command_print(CMD,
		"   TapName             Enabled  IdCode     Expected   IrLen IrCap IrMask");
	command_print(CMD,
		"-- ------------------- -------- ---------- ---------- ----- ----- ------");

	while (tap) {
		uint32_t expected, expected_mask, ii;

		snprintf(expected_id, sizeof(expected_id), "0x%08x",
			(unsigned)((tap->expected_ids_cnt > 0)
				   ? tap->expected_ids[0]
				   : 0));
		if (tap->ignore_version)
			expected_id[2] = '*';

		expected = buf_get_u32(tap->expected, 0, tap->ir_length);
		expected_mask = buf_get_u32(tap->expected_mask, 0, tap->ir_length);

		command_print(CMD,
			"%2d %-18s     %c     0x%08x %s %5d 0x%02x  0x%02x",
			tap->abs_chain_position,
			tap->dotted_name,
			tap->enabled ? 'Y' : 'n',
			(unsigned int)(tap->idcode),
			expected_id,
			(unsigned int)(tap->ir_length),
			(unsigned int)(expected),
			(unsigned int)(expected_mask));

		for (ii = 1; ii < tap->expected_ids_cnt; ii++) {
			snprintf(expected_id, sizeof(expected_id), "0x%08x",
				(unsigned) tap->expected_ids[ii]);
			if (tap->ignore_version)
				expected_id[2] = '*';

			command_print(CMD,
				"                                           %s",
				expected_id);
		}

		tap = tap->next_tap;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_jtag_ntrst_delay_command)
{
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (CMD_ARGC == 1) {
		unsigned delay;
		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], delay);

		jtag_set_ntrst_delay(delay);
	}
	command_print(CMD, "jtag_ntrst_delay: %u", jtag_get_ntrst_delay());
	return ERROR_OK;
}

COMMAND_HANDLER(handle_jtag_ntrst_assert_width_command)
{
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;
	if (CMD_ARGC == 1) {
		unsigned delay;
		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], delay);

		jtag_set_ntrst_assert_width(delay);
	}
	command_print(CMD, "jtag_ntrst_assert_width: %u", jtag_get_ntrst_assert_width());
	return ERROR_OK;
}

COMMAND_HANDLER(handle_jtag_rclk_command)
{
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	int retval = ERROR_OK;
	if (CMD_ARGC == 1) {
		unsigned khz = 0;
		COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], khz);

		retval = adapter_config_rclk(khz);
		if (retval != ERROR_OK)
			return retval;
	}

	int cur_khz = adapter_get_speed_khz();
	retval = adapter_get_speed_readable(&cur_khz);
	if (retval != ERROR_OK)
		return retval;

	if (cur_khz)
		command_print(CMD, "RCLK not supported - fallback to %d kHz", cur_khz);
	else
		command_print(CMD, "RCLK - adaptive");

	return retval;
}

COMMAND_HANDLER(handle_runtest_command)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	unsigned num_clocks;
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], num_clocks);

	jtag_add_runtest(num_clocks, TAP_IDLE);
	return jtag_execute_queue();
}

/*
 * For "irscan" or "drscan" commands, the "end" (really, "next") state
 * should be stable ... and *NOT* a shift state, otherwise free-running
 * jtag clocks could change the values latched by the update state.
 * Not surprisingly, this is the same constraint as SVF; the "irscan"
 * and "drscan" commands are a write-only subset of what SVF provides.
 */

COMMAND_HANDLER(handle_irscan_command)
{
	int i;
	struct scan_field *fields;
	struct jtag_tap *tap = NULL;
	tap_state_t endstate;

	if ((CMD_ARGC < 2) || (CMD_ARGC % 2))
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* optional "-endstate" "statename" at the end of the arguments,
	 * so that e.g. IRPAUSE can let us load the data register before
	 * entering RUN/IDLE to execute the instruction we load here.
	 */
	endstate = TAP_IDLE;

	if (CMD_ARGC >= 4) {
		/* have at least one pair of numbers.
		 * is last pair the magic text? */
		if (strcmp("-endstate", CMD_ARGV[CMD_ARGC - 2]) == 0) {
			endstate = tap_state_by_name(CMD_ARGV[CMD_ARGC - 1]);
			if (endstate == TAP_INVALID)
				return ERROR_COMMAND_SYNTAX_ERROR;
			if (!scan_is_safe(endstate))
				LOG_WARNING("unstable irscan endstate \"%s\"",
					CMD_ARGV[CMD_ARGC - 1]);
			CMD_ARGC -= 2;
		}
	}

	int num_fields = CMD_ARGC / 2;
	if (num_fields > 1) {
		/* we really should be looking at plain_ir_scan if we want
		 * anything more fancy.
		 */
		LOG_ERROR("Specify a single value for tap");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	fields = calloc(num_fields, sizeof(*fields));

	int retval;
	for (i = 0; i < num_fields; i++) {
		tap = jtag_tap_by_string(CMD_ARGV[i*2]);
		if (!tap) {
			free(fields);
			command_print(CMD, "Tap: %s unknown", CMD_ARGV[i*2]);

			return ERROR_FAIL;
		}
		uint64_t value;
		retval = parse_u64(CMD_ARGV[i * 2 + 1], &value);
		if (retval != ERROR_OK)
			goto error_return;

		int field_size = tap->ir_length;
		fields[i].num_bits = field_size;
		uint8_t *v = calloc(1, DIV_ROUND_UP(field_size, 8));
		if (!v) {
			LOG_ERROR("Out of memory");
			goto error_return;
		}

		buf_set_u64(v, 0, field_size, value);
		fields[i].out_value = v;
		fields[i].in_value = NULL;
	}

	/* did we have an endstate? */
	jtag_add_ir_scan(tap, fields, endstate);

	retval = jtag_execute_queue();

error_return:
	for (i = 0; i < num_fields; i++)
		free((void *)fields[i].out_value);

	free(fields);

	return retval;
}

COMMAND_HANDLER(handle_verify_ircapture_command)
{
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC == 1) {
		bool enable;
		COMMAND_PARSE_ENABLE(CMD_ARGV[0], enable);
		jtag_set_verify_capture_ir(enable);
	}

	const char *status = jtag_will_verify_capture_ir() ? "enabled" : "disabled";
	command_print(CMD, "verify Capture-IR is %s", status);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_verify_jtag_command)
{
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC == 1) {
		bool enable;
		COMMAND_PARSE_ENABLE(CMD_ARGV[0], enable);
		jtag_set_verify(enable);
	}

	const char *status = jtag_will_verify() ? "enabled" : "disabled";
	command_print(CMD, "verify jtag capture is %s", status);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_tms_sequence_command)
{
	if (CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if (CMD_ARGC == 1) {
		bool use_new_table;
		if (strcmp(CMD_ARGV[0], "short") == 0)
			use_new_table = true;
		else if (strcmp(CMD_ARGV[0], "long") == 0)
			use_new_table = false;
		else
			return ERROR_COMMAND_SYNTAX_ERROR;

		tap_use_new_tms_table(use_new_table);
	}

	command_print(CMD, "tms sequence is  %s",
		tap_uses_new_tms_table() ? "short" : "long");

	return ERROR_OK;
}

COMMAND_HANDLER(handle_jtag_flush_queue_sleep)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	int sleep_ms;
	COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], sleep_ms);

	jtag_set_flush_queue_sleep(sleep_ms);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_wait_srst_deassert)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	int timeout_ms;
	COMMAND_PARSE_NUMBER(int, CMD_ARGV[0], timeout_ms);
	if ((timeout_ms <= 0) || (timeout_ms > 100000)) {
		LOG_ERROR("Timeout must be an integer between 0 and 100000");
		return ERROR_FAIL;
	}

	LOG_USER("Waiting for srst assert + deassert for at most %dms", timeout_ms);
	int asserted_yet;
	int64_t then = timeval_ms();
	while (jtag_srst_asserted(&asserted_yet) == ERROR_OK) {
		if ((timeval_ms() - then) > timeout_ms) {
			LOG_ERROR("Timed out");
			return ERROR_FAIL;
		}
		if (asserted_yet)
			break;
	}
	while (jtag_srst_asserted(&asserted_yet) == ERROR_OK) {
		if ((timeval_ms() - then) > timeout_ms) {
			LOG_ERROR("Timed out");
			return ERROR_FAIL;
		}
		if (!asserted_yet)
			break;
	}

	return ERROR_OK;
}

static const struct command_registration jtag_command_handlers[] = {

	{
		.name = "jtag_flush_queue_sleep",
		.handler = handle_jtag_flush_queue_sleep,
		.mode = COMMAND_ANY,
		.help = "For debug purposes(simulate long delays of interface) "
			"to test performance or change in behavior. Default 0ms.",
		.usage = "[sleep in ms]",
	},
	{
		.name = "jtag_rclk",
		.handler = handle_jtag_rclk_command,
		.mode = COMMAND_ANY,
		.help = "With an argument, change to to use adaptive clocking "
			"if possible; else to use the fallback speed.  "
			"With or without argument, display current setting.",
		.usage = "[fallback_speed_khz]",
	},
	{
		.name = "jtag_ntrst_delay",
		.handler = handle_jtag_ntrst_delay_command,
		.mode = COMMAND_ANY,
		.help = "delay after deasserting trst in ms",
		.usage = "[milliseconds]",
	},
	{
		.name = "jtag_ntrst_assert_width",
		.handler = handle_jtag_ntrst_assert_width_command,
		.mode = COMMAND_ANY,
		.help = "delay after asserting trst in ms",
		.usage = "[milliseconds]",
	},
	{
		.name = "scan_chain",
		.handler = handle_scan_chain_command,
		.mode = COMMAND_ANY,
		.help = "print current scan chain configuration",
		.usage = ""
	},
	{
		.name = "runtest",
		.handler = handle_runtest_command,
		.mode = COMMAND_EXEC,
		.help = "Move to Run-Test/Idle, and issue TCK for num_cycles.",
		.usage = "num_cycles"
	},
	{
		.name = "irscan",
		.handler = handle_irscan_command,
		.mode = COMMAND_EXEC,
		.help = "Execute Instruction Register (IR) scan.  The "
			"specified opcodes are put into each TAP's IR, "
			"and other TAPs are put in BYPASS.",
		.usage = "[tap_name instruction]* ['-endstate' state_name]",
	},
	{
		.name = "verify_ircapture",
		.handler = handle_verify_ircapture_command,
		.mode = COMMAND_ANY,
		.help = "Display or assign flag controlling whether to "
			"verify values captured during Capture-IR.",
		.usage = "['enable'|'disable']",
	},
	{
		.name = "verify_jtag",
		.handler = handle_verify_jtag_command,
		.mode = COMMAND_ANY,
		.help = "Display or assign flag controlling whether to "
			"verify values captured during IR and DR scans.",
		.usage = "['enable'|'disable']",
	},
	{
		.name = "tms_sequence",
		.handler = handle_tms_sequence_command,
		.mode = COMMAND_ANY,
		.help = "Display or change what style TMS sequences to use "
			"for JTAG state transitions:  short (default) or "
			"long.  Only for working around JTAG bugs.",
		/* Specifically for working around DRIVER bugs... */
		.usage = "['short'|'long']",
	},
	{
		.name = "wait_srst_deassert",
		.handler = handle_wait_srst_deassert,
		.mode = COMMAND_ANY,
		.help = "Wait for an SRST deassert. "
			"Useful for cases where you need something to happen within ms "
			"of an srst deassert. Timeout in ms",
		.usage = "ms",
	},
	{
		.name = "jtag",
		.mode = COMMAND_ANY,
		.help = "perform jtag tap actions",
		.usage = "",

		.chain = jtag_subcommand_handlers,
	},
	{
		.chain = jtag_command_handlers_to_move,
	},
	COMMAND_REGISTRATION_DONE
};

int jtag_register_commands(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, jtag_command_handlers);
}
