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

#include <helper/command.h>
#include <helper/nvp.h>
#include <helper/time_support.h>
#include "transport/transport.h"
#include "drivers/minidriver_imp.h"

/**
 * @file
 * Holds support for accessing JTAG-specific mechanisms from TCl scripts.
 */

static const struct nvp nvp_jtag_tap_event[] = {
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

static bool scan_is_safe(enum tap_state state)
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

static COMMAND_HELPER(parse_tap_names, unsigned int *cur_arg_p,
		unsigned int *n_active_taps_p, const struct jtag_tap **taps,
		const char * const *end_args)
{
	unsigned int cur_arg = *cur_arg_p;
	unsigned int n_active_taps = 0;
	for (; cur_arg < CMD_ARGC; ++n_active_taps) {
		for (const char * const *end_arg_p = end_args;
				*end_arg_p; ++end_arg_p)
			if (strcmp(CMD_ARGV[cur_arg], *end_arg_p) == 0)
				goto exit;
		const char *tap_name = CMD_ARGV[cur_arg++];
		struct jtag_tap *tap = jtag_tap_by_string(tap_name);
		if (!tap)
			return ERROR_COMMAND_ARGUMENT_INVALID;
		taps[n_active_taps] = tap;
	}
exit:
	if (n_active_taps == 0)
		return ERROR_COMMAND_SYNTAX_ERROR;
	*n_active_taps_p = n_active_taps;
	*cur_arg_p = cur_arg;
	return ERROR_OK;
}

struct scan_print_info {
	struct command_invocation *cmd;
	size_t n_active_taps;
	const struct scan_fields_on_tap *tap_scans;
	bool ir_scan;
};

static int cmp_scan_fields_on_tap(const void *lhs, const void *rhs)
{
	return ((const struct scan_fields_on_tap *)lhs)->tap->abs_chain_position
		- ((const struct scan_fields_on_tap *)rhs)->tap->abs_chain_position;
}

static const char *buf_to_hex_str_err_fmt = "Unable to allocate memory for a "
	"hexadecimal string represantation of %u-bit long buffer.";

static int print_ir_tap_scan(struct command_invocation *cmd,
		const struct scan_fields_on_tap *tap_field, const char *separator)
{
	assert(tap_field->num_fields == 1);
	unsigned int num_bits = tap_field->fields->num_bits;
	char *in_str = buf_to_hex_str(tap_field->fields->in_value, num_bits);
	if (!in_str) {
		command_print(cmd, buf_to_hex_str_err_fmt, num_bits);
		return ERROR_FAIL;
	}
	command_print_sameline(cmd, "%s %s%s", jtag_tap_name(tap_field->tap),
			in_str, separator);
	free(in_str);
	return ERROR_OK;
}

static int print_dr_tap_scan(struct command_invocation *cmd,
		const struct scan_fields_on_tap *tap_field, const char *separator)
{
	command_print_sameline(cmd, "%s ",
			jtag_tap_name(tap_field->tap));
	const char *field_separator = "";
	for (size_t i = 0; i < tap_field->num_fields; ++i) {
		unsigned int num_bits = tap_field->fields[i].num_bits;
		char *in_str = buf_to_hex_str(tap_field->fields[i].in_value,
				num_bits);
		if (!in_str) {
			command_print(cmd, buf_to_hex_str_err_fmt, num_bits);
			return ERROR_FAIL;
		}
		command_print_sameline(cmd, "%s%s", field_separator, in_str);
		free(in_str);
		field_separator = " ";
	}
	command_print_sameline(cmd, "%s", separator);
	return ERROR_OK;
}

static int print_scans(jtag_callback_data_t arg_info, jtag_callback_data_t arg2,
		jtag_callback_data_t arg3, jtag_callback_data_t arg4)
{
	const struct scan_print_info *info =
		(const struct scan_print_info *)arg_info;
	command_print_sameline(info->cmd, "%s ", info->ir_scan ? "IR" : "DR");
	assert(info->n_active_taps > 0);
	unsigned int last_i = info->n_active_taps - 1;
	for (unsigned int i = 0; i < last_i; ++i) {
		int res = (info->ir_scan ? print_ir_tap_scan : print_dr_tap_scan)
			(info->cmd, &info->tap_scans[i], " ");
		if (res != ERROR_OK)
			return res;
	}
	return (info->ir_scan ? print_ir_tap_scan : print_dr_tap_scan)(info->cmd,
			&info->tap_scans[last_i], "\n");
}

static COMMAND_HELPER(parse_ir_scan_fields_on_tap, unsigned int *cur_arg_p,
		struct scan_fields_on_tap *tap_scan, const struct jtag_tap *tap)
{
	unsigned int cur_arg = *cur_arg_p;

	struct scan_field *field = cmd_queue_alloc(sizeof(*field));
	const char *ir_val_str = CMD_ARGV[cur_arg++];
	size_t bits = tap->ir_length;
	uint8_t *value = cmd_queue_alloc(DIV_ROUND_UP(bits, 8));
	int ret = CALL_COMMAND_HANDLER(command_parse_str_to_buf, ir_val_str,
			value, bits);
	if (ret != ERROR_OK)
		return ret;
	field->num_bits = bits;
	field->in_value = value;
	field->out_value = value;
	tap_scan->fields = field;
	tap_scan->num_fields = 1;
	tap_scan->tap = tap;

	*cur_arg_p = cur_arg;
	return ERROR_OK;
}

static COMMAND_HELPER(parse_dr_scan_fields_on_tap, unsigned int *cur_arg_p,
		struct scan_fields_on_tap *tap_scan, const struct jtag_tap *tap)
{
	unsigned int cur_arg = *cur_arg_p;

	char *arg_copy = strdup(CMD_ARGV[cur_arg++]);
	if (!arg_copy) {
		command_print(CMD, "Out of memory.");
		return ERROR_FAIL;
	}
	size_t min_chars_per_field = 3;
	size_t max_fields = DIV_ROUND_UP(strlen(arg_copy),
			min_chars_per_field);
	struct scan_field *fields = cmd_queue_alloc(max_fields
			* sizeof(*fields));
	char *val_str = NULL;
	size_t num_fields = 0;
	const char *separators = ",";
	int ret;
	for (char *field_str = strtok(arg_copy, separators);
			field_str; field_str = strtok(NULL, separators)) {
		val_str = realloc(val_str, strlen(field_str));
		if (!val_str) {
			command_print(CMD, "Out of memory.");
			ret = ERROR_FAIL;
			goto cleanup;
		}
		int signed_bits;
		if (sscanf(field_str, "%i:%s", &signed_bits, val_str) != 2) {
			command_print(CMD, "Scan field should be passed as "
					"'<bits>:<value>', not '%s'", arg_copy);
			ret = ERROR_COMMAND_ARGUMENT_INVALID;
			goto cleanup;
		}
		if (signed_bits <= 0) {
			command_print(CMD, "Length of a scan field in bits "
				       "should be a positive integer.");
			ret = ERROR_COMMAND_ARGUMENT_UNDERFLOW;
			goto cleanup;
		}
		unsigned int bits = signed_bits;
		if (bits / 8 / 1024 > 640) {
			command_print(CMD,
					"%d-bits long scan fields are not supported. "
					"640K of memory should be enough for anybody.", bits);
			ret = ERROR_COMMAND_ARGUMENT_OVERFLOW;
			goto cleanup;
		}
		uint8_t *value = cmd_queue_alloc(DIV_ROUND_UP(bits, 8));
		ret = CALL_COMMAND_HANDLER(command_parse_str_to_buf, val_str,
				value, bits);
		if (ret != ERROR_OK)
			goto cleanup;
		fields[num_fields].num_bits = bits;
		fields[num_fields].in_value = value;
		fields[num_fields].out_value = value;
		++num_fields;
	}
	if (num_fields == 0) {
		command_print(CMD, "There are no fields in '%s'", arg_copy);
		ret = ERROR_COMMAND_ARGUMENT_INVALID;
		goto cleanup;
	}
	tap_scan->fields = fields;
	tap_scan->num_fields = num_fields;
	tap_scan->tap = tap;

	*cur_arg_p = cur_arg;
	ret = ERROR_OK;
cleanup:
	free(val_str);
	free(arg_copy);
	return ret;
}

#define JTAG_EXECUTE_SCAN_ENDSTATE_NEXT_ARG "-endstate"

static COMMAND_HELPER(parse_endstate, unsigned int *cur_arg_p, enum tap_state *endstate)
{
	unsigned int cur_arg = *cur_arg_p;
	*endstate = TAP_IDLE;
	if (cur_arg == CMD_ARGC
			|| strcmp(CMD_ARGV[cur_arg], JTAG_EXECUTE_SCAN_ENDSTATE_NEXT_ARG))
		return ERROR_OK;
	if (++cur_arg == CMD_ARGC)
		return ERROR_COMMAND_SYNTAX_ERROR;
	const char *endstate_name = CMD_ARGV[cur_arg++];
	*endstate = tap_state_by_name(endstate_name);
	if (!scan_is_safe(*endstate)) {
		command_print(CMD, "endstate: %s is not stable", endstate_name);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}
	*cur_arg_p = cur_arg;
	return ERROR_OK;
}

static COMMAND_HELPER(validate_fields_on_taps, struct scan_fields_on_tap *tap_fields,
		unsigned int n_active_taps)
{
	qsort(tap_fields, n_active_taps, sizeof(*tap_fields), cmp_scan_fields_on_tap);

	for (unsigned int i = 1; i < n_active_taps; ++i) {
		const struct jtag_tap *curr_tap = tap_fields[i].tap;
		const struct jtag_tap *prev_tap = tap_fields[i - 1].tap;
		if (curr_tap->abs_chain_position == prev_tap->abs_chain_position) {
			command_print(CMD, "TAP %s is passed more then once.",
					jtag_tap_name(curr_tap));
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
	}
	return ERROR_OK;
}

#define JTAG_EXECUTE_SCAN_IR_START_ARG "-ir"
#define JTAG_EXECUTE_SCAN_DR_START_ARG "-dr"

static COMMAND_HELPER(parse_scan_fields, unsigned int *cur_arg_p, bool ir_scan,
		unsigned int n_active_taps, const struct jtag_tap **taps,
		const char * const *end_args)
{
	unsigned int cur_arg = *cur_arg_p;

	const char *start_arg = ir_scan
		? JTAG_EXECUTE_SCAN_IR_START_ARG
		: JTAG_EXECUTE_SCAN_DR_START_ARG;
	if (cur_arg == CMD_ARGC || strcmp(CMD_ARGV[cur_arg++], start_arg))
		return ERROR_OK;

	unsigned int i = 0;
	int ret = ERROR_OK;
	struct scan_fields_on_tap *tap_fields = cmd_queue_alloc((CMD_ARGC - cur_arg) *
			sizeof(*tap_fields));
	for (; cur_arg < CMD_ARGC && i < n_active_taps; ++i) {
		for (const char * const *end_arg_p = end_args; *end_arg_p;
				++end_arg_p)
			if (strcmp(CMD_ARGV[cur_arg], *end_arg_p) == 0)
				goto exit;
		if (strcmp(CMD_ARGV[cur_arg], JTAG_EXECUTE_SCAN_ENDSTATE_NEXT_ARG) == 0)
			goto exit;
		ret = CALL_COMMAND_HANDLER((ir_scan ? parse_ir_scan_fields_on_tap : parse_dr_scan_fields_on_tap),
				&cur_arg, &tap_fields[i], taps[i]);
		if (ret != ERROR_OK)
			return ret;
	}
exit:
	if (i != n_active_taps) {
		command_print(CMD,
				"Scan fields should be specified for each active TAP.");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	enum tap_state endstate;
	ret = CALL_COMMAND_HANDLER(parse_endstate, &cur_arg, &endstate);
	if (ret != ERROR_OK)
		return ret;

	ret = CALL_COMMAND_HANDLER(validate_fields_on_taps, tap_fields, n_active_taps);
	if (ret != ERROR_OK)
		return ret;

	(ir_scan ? jtag_add_multitap_ir_scan : jtag_add_multitap_dr_scan)(n_active_taps, tap_fields, endstate);
	struct scan_print_info *print_info = cmd_queue_alloc(sizeof(*print_info));
	print_info->cmd = CMD,
	print_info->ir_scan = ir_scan,
	print_info->n_active_taps = n_active_taps,
	print_info->tap_scans = tap_fields,
	jtag_add_callback4(print_scans, (jtag_callback_data_t)print_info, 0, 0, 0);

	*cur_arg_p = cur_arg;
	return ERROR_OK;
}

#define JTAG_EXECUTE_SCAN_START_ARG "scan"

static COMMAND_HELPER(parse_one_scan, unsigned int *cur_arg_p)
{
	unsigned int cur_arg = *cur_arg_p;
	unsigned int min_args_per_tap = 2;
	unsigned int max_n_taps = DIV_ROUND_UP(CMD_ARGC - cur_arg, min_args_per_tap);
	const struct jtag_tap **taps = cmd_queue_alloc(max_n_taps * sizeof(*taps));
	unsigned int n_active_taps;
	static const char * const end_args[] = {
		JTAG_EXECUTE_SCAN_IR_START_ARG,
		JTAG_EXECUTE_SCAN_DR_START_ARG,
		JTAG_EXECUTE_SCAN_START_ARG,
		NULL
	};
	int res = CALL_COMMAND_HANDLER(parse_tap_names, &cur_arg,
			&n_active_taps, taps, end_args);
	if (res != ERROR_OK)
		return res;
	res = CALL_COMMAND_HANDLER(parse_scan_fields, &cur_arg,
			/* ir_scan */ true, n_active_taps, taps, end_args);
	if (res != ERROR_OK)
		return res;
	res = CALL_COMMAND_HANDLER(parse_scan_fields, &cur_arg,
			/* ir_scan */ false, n_active_taps, taps, end_args);
	if (res != ERROR_OK)
		return res;
	*cur_arg_p = cur_arg;
	return ERROR_OK;
}

static COMMAND_HELPER(jtag_execute_parse_one_command, unsigned int *cur_arg_p)
{
	unsigned int cur_arg = *cur_arg_p;
	/* TODO: support other command types. */
	if (strcmp(JTAG_EXECUTE_SCAN_START_ARG, CMD_ARGV[cur_arg++]))
		return ERROR_COMMAND_ARGUMENT_INVALID;
	int res = CALL_COMMAND_HANDLER(parse_one_scan, &cur_arg);
	if (res == ERROR_OK)
		*cur_arg_p = cur_arg;
	return res;
}

COMMAND_HANDLER(handle_jtag_execute)
{
	int res;
	unsigned int cur_arg = 0;
	while (cur_arg < CMD_ARGC) {
		res = CALL_COMMAND_HANDLER(jtag_execute_parse_one_command, &cur_arg);
		jtag_set_error(res);
		if (res != ERROR_OK)
			break;
	}
	return jtag_execute_queue();
}

COMMAND_HANDLER(handle_jtag_command_pathmove)
{
	enum tap_state states[8];

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

	const unsigned int count = jtag_get_flush_queue_count();
	command_print_sameline(CMD, "%u", count);

	return ERROR_OK;
}

/* REVISIT Just what about these should "move" ... ?
 * These registrations, into the main JTAG table?
 *
 * There's a minor compatibility issue, these all show up twice;
 * that's not desirable:
 *  - jtag pathmove ...
 *  - pathmove ...
 *
 */
static const struct command_registration jtag_command_handlers_to_move[] = {
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

static struct nvp nvp_config_opts[] = {
	{ .name = "-event",      .value = JCFG_EVENT },
	{ .name = "-idcode",     .value = JCFG_IDCODE },

	{ .name = NULL,          .value = -1 }
};

static int jtag_tap_set_event(struct command_context *cmd_ctx, struct jtag_tap *tap,
		 const struct nvp *event, Jim_Obj *body)
{
	struct jtag_tap_event_action *jteap = tap->event_action;

	while (jteap) {
		if (jteap->event == (enum jtag_event)event->value)
			break;
		jteap = jteap->next;
	}

	if (!jteap) {
		jteap = calloc(1, sizeof(*jteap));
		if (!jteap) {
			LOG_ERROR("Out of memory");
			return ERROR_FAIL;
		}

		/* add to head of event list */
		jteap->next = tap->event_action;
		tap->event_action = jteap;
	} else {
		Jim_DecrRefCount(cmd_ctx->interp, jteap->body);
	}

	jteap->interp = cmd_ctx->interp;
	jteap->event = (enum jtag_event)event->value;
	jteap->body = Jim_DuplicateObj(cmd_ctx->interp, body);
	Jim_IncrRefCount(jteap->body);

	return ERROR_OK;
}

__COMMAND_HANDLER(handle_jtag_configure)
{
	bool is_configure = !strcmp(CMD_NAME, "configure");

	if (CMD_ARGC < (is_configure ? 3 : 2))
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* FIXME: rework jtag_tap_by_jim_obj */
	struct jtag_tap *tap = jtag_tap_by_jim_obj(CMD_CTX->interp, CMD_JIMTCL_ARGV[0]);
	if (!tap)
		return ERROR_FAIL;

	for (unsigned int i = 1; i < CMD_ARGC; i++) {
		const struct nvp *n = nvp_name2value(nvp_config_opts, CMD_ARGV[i]);
		switch (n->value) {
		case JCFG_EVENT:
			if (i + (is_configure ? 2 : 1) >= CMD_ARGC) {
				command_print(CMD, "wrong # args: should be \"-event <event-name>%s\"",
						is_configure ? " <event-body>" : "");
				return ERROR_COMMAND_ARGUMENT_INVALID;
			}

			const struct nvp *event = nvp_name2value(nvp_jtag_tap_event, CMD_ARGV[i + 1]);
			if (!event->name) {
				nvp_unknown_command_print(CMD, nvp_jtag_tap_event, CMD_ARGV[i], CMD_ARGV[i + 1]);
				return ERROR_COMMAND_ARGUMENT_INVALID;
			}

			if (is_configure) {
				int retval = jtag_tap_set_event(CMD_CTX, tap, event, CMD_JIMTCL_ARGV[i + 2]);
				if (retval != ERROR_OK)
					return retval;
			} else {
				struct jtag_tap_event_action *jteap = tap->event_action;
				while (jteap) {
					if (jteap->event == (enum jtag_event)event->value) {
						command_print(CMD, "%s", Jim_GetString(jteap->body, NULL));
						break;
					}
					jteap = jteap->next;
				}
			}

			i += is_configure ? 2 : 1;
			break;
		case JCFG_IDCODE:
			if (is_configure) {
				command_print(CMD, "not settable: %s", n->name);
				return ERROR_COMMAND_ARGUMENT_INVALID;
			}
			command_print(CMD, "0x%08x", tap->idcode);
			break;
		default:
			nvp_unknown_command_print(CMD, nvp_config_opts, NULL, CMD_ARGV[i]);
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
	}
	return ERROR_OK;
}

#define NTAP_OPT_IRLEN     0
#define NTAP_OPT_IRMASK    1
#define NTAP_OPT_IRCAPTURE 2
#define NTAP_OPT_ENABLED   3
#define NTAP_OPT_DISABLED  4
#define NTAP_OPT_EXPECTED_ID 5
#define NTAP_OPT_VERSION   6
#define NTAP_OPT_BYPASS    7
#define NTAP_OPT_IRBYPASS    8

static const struct nvp jtag_newtap_opts[] = {
	{ .name = "-irlen",          .value = NTAP_OPT_IRLEN },
	{ .name = "-irmask",         .value = NTAP_OPT_IRMASK },
	{ .name = "-ircapture",      .value = NTAP_OPT_IRCAPTURE },
	{ .name = "-enable",         .value = NTAP_OPT_ENABLED },
	{ .name = "-disable",        .value = NTAP_OPT_DISABLED },
	{ .name = "-expected-id",    .value = NTAP_OPT_EXPECTED_ID },
	{ .name = "-ignore-version", .value = NTAP_OPT_VERSION },
	{ .name = "-ignore-bypass",  .value = NTAP_OPT_BYPASS },
	{ .name = "-ir-bypass",      .value = NTAP_OPT_IRBYPASS },
	{ .name = NULL,              .value = -1 },
};

static COMMAND_HELPER(handle_jtag_newtap_args, struct jtag_tap *tap)
{
	/* we expect CHIP + TAP + OPTIONS */
	if (CMD_ARGC < 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	tap->chip = strdup(CMD_ARGV[0]);
	tap->tapname = strdup(CMD_ARGV[1]);
	tap->dotted_name = alloc_printf("%s.%s", CMD_ARGV[0], CMD_ARGV[1]);
	if (!tap->chip || !tap->tapname || !tap->dotted_name) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}
	CMD_ARGC -= 2;
	CMD_ARGV += 2;

	LOG_DEBUG("Creating New Tap, Chip: %s, Tap: %s, Dotted: %s, %d params",
		  tap->chip, tap->tapname, tap->dotted_name, CMD_ARGC);

	/*
	 * IEEE specifies that the two LSBs of an IR scan are 01, so make
	 * that the default.  The "-ircapture" and "-irmask" options are only
	 * needed to cope with nonstandard TAPs, or to specify more bits.
	 */
	tap->ir_capture_mask = 0x03;
	tap->ir_capture_value = 0x01;

	while (CMD_ARGC) {
		const struct nvp *n = nvp_name2value(jtag_newtap_opts, CMD_ARGV[0]);
		CMD_ARGC--;
		CMD_ARGV++;
		switch (n->value) {
	    case NTAP_OPT_ENABLED:
		    tap->disabled_after_reset = false;
		    break;

	    case NTAP_OPT_DISABLED:
		    tap->disabled_after_reset = true;
		    break;

		case NTAP_OPT_EXPECTED_ID:
			if (!CMD_ARGC)
				return ERROR_COMMAND_ARGUMENT_INVALID;

			tap->expected_ids = realloc(tap->expected_ids,
										(tap->expected_ids_cnt + 1) * sizeof(uint32_t));
			if (!tap->expected_ids) {
				LOG_ERROR("Out of memory");
				return ERROR_FAIL;
			}

			uint32_t id;
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], id);
			CMD_ARGC--;
			CMD_ARGV++;
			tap->expected_ids[tap->expected_ids_cnt++] = id;

			break;

		case NTAP_OPT_IRLEN:
			if (!CMD_ARGC)
				return ERROR_COMMAND_ARGUMENT_INVALID;

			COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], tap->ir_length);
			CMD_ARGC--;
			CMD_ARGV++;
			if (tap->ir_length > (8 * sizeof(tap->ir_capture_value)))
				LOG_WARNING("%s: huge IR length %u", tap->dotted_name, tap->ir_length);
			break;

		case NTAP_OPT_IRMASK:
			if (!CMD_ARGC)
				return ERROR_COMMAND_ARGUMENT_INVALID;

			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], tap->ir_capture_mask);
			CMD_ARGC--;
			CMD_ARGV++;
			if ((tap->ir_capture_mask & 3) != 3)
				LOG_WARNING("%s: nonstandard IR mask", tap->dotted_name);
			break;

		case NTAP_OPT_IRCAPTURE:
			if (!CMD_ARGC)
				return ERROR_COMMAND_ARGUMENT_INVALID;

			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], tap->ir_capture_value);
			CMD_ARGC--;
			CMD_ARGV++;
			if ((tap->ir_capture_value & 3) != 1)
				LOG_WARNING("%s: nonstandard IR value", tap->dotted_name);
			break;

		case NTAP_OPT_VERSION:
			tap->ignore_version = true;
			break;

		case NTAP_OPT_BYPASS:
			tap->ignore_bypass = true;
			break;

		case NTAP_OPT_IRBYPASS:
			if (!CMD_ARGC)
				return ERROR_COMMAND_ARGUMENT_INVALID;

			COMMAND_PARSE_NUMBER(u64, CMD_ARGV[0], tap->ir_bypass_value);
			CMD_ARGC--;
			CMD_ARGV++;
			break;

		default:
			nvp_unknown_command_print(CMD, jtag_newtap_opts, NULL, CMD_ARGV[-1]);
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
	}

	/* default is enabled-after-reset */
	tap->enabled = !tap->disabled_after_reset;

	if (transport_is_jtag() && tap->ir_length == 0) {
		command_print(CMD, "newtap: %s missing IR length", tap->dotted_name);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	return ERROR_OK;
}

__COMMAND_HANDLER(handle_jtag_newtap)
{
	struct jtag_tap *tap = calloc(1, sizeof(struct jtag_tap));
	if (!tap) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	int retval = CALL_COMMAND_HANDLER(handle_jtag_newtap_args, tap);
	if (retval != ERROR_OK) {
		free(tap->chip);
		free(tap->tapname);
		free(tap->dotted_name);
		free(tap->expected_ids);
		free(tap);
		return retval;
	}

	jtag_tap_init(tap);
	return ERROR_OK;
}

static void jtag_tap_handle_event(struct jtag_tap *tap, enum jtag_event e)
{
	struct jtag_tap_event_action *jteap;
	int retval;

	for (jteap = tap->event_action; jteap; jteap = jteap->next) {
		if (jteap->event != e)
			continue;

		const struct nvp *nvp = nvp_value2name(nvp_jtag_tap_event, e);
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

static bool jtag_tap_enable(struct jtag_tap *t)
{
	if (t->enabled)
		return true;
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
		return true;
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

__COMMAND_HANDLER(handle_jtag_tap_enabler)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct jtag_tap *t = jtag_tap_by_string(CMD_ARGV[0]);
	if (!t) {
		command_print(CMD, "Tap '%s' could not be found", CMD_ARGV[0]);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	if (strcmp(CMD_NAME, "tapisenabled") == 0) {
		/* do nothing, just return the value */
	} else if (strcmp(CMD_NAME, "tapenable") == 0) {
		if (!jtag_tap_enable(t)) {
			command_print(CMD, "failed to enable tap %s", t->dotted_name);
			return ERROR_FAIL;
		}
	} else if (strcmp(CMD_NAME, "tapdisable") == 0) {
		if (!jtag_tap_disable(t)) {
			command_print(CMD, "failed to disable tap %s", t->dotted_name);
			return ERROR_FAIL;
		}
	} else {
		command_print(CMD, "command '%s' unknown", CMD_NAME);
		return ERROR_FAIL;
	}

	command_print(CMD, "%d", t->enabled ? 1 : 0);
	return ERROR_OK;
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
		.handler = handle_jtag_newtap,
		.help = "Create a new TAP instance named basename.tap_type, "
			"and appends it to the scan chain.",
		.usage = "basename tap_type '-irlen' count "
			"['-enable'|'-disable'] "
			"['-expected_id' number] "
			"['-ignore-version'] "
			"['-ignore-bypass'] "
			"['-ircapture' number] "
			"['-ir-bypass' number] "
			"['-mask' number]",
	},
	{
		.name = "tapisenabled",
		.mode = COMMAND_EXEC,
		.handler = handle_jtag_tap_enabler,
		.help = "Returns a Tcl boolean (0/1) indicating whether "
			"the TAP is enabled (1) or not (0).",
		.usage = "tap_name",
	},
	{
		.name = "tapenable",
		.mode = COMMAND_EXEC,
		.handler = handle_jtag_tap_enabler,
		.help = "Try to enable the specified TAP using the "
			"'tap-enable' TAP event.",
		.usage = "tap_name",
	},
	{
		.name = "tapdisable",
		.mode = COMMAND_EXEC,
		.handler = handle_jtag_tap_enabler,
		.help = "Try to disable the specified TAP using the "
			"'tap-disable' TAP event.",
		.usage = "tap_name",
	},
	{
		.name = "configure",
		.mode = COMMAND_ANY,
		.handler = handle_jtag_configure,
		.help = "Provide a Tcl handler for the specified "
			"TAP event.",
		.usage = "tap_name '-event' event_name handler",
	},
	{
		.name = "cget",
		.mode = COMMAND_EXEC,
		.handler = handle_jtag_configure,
		.help = "Return any Tcl handler for the specified "
			"TAP event or the value of the IDCODE found in hardware.",
		.usage = "tap_name '-event' event_name | "
		    "tap_name '-idcode'",
	},
	{
		.name = "names",
		.mode = COMMAND_ANY,
		.handler = handle_jtag_names,
		.help = "Returns list of all JTAG tap names.",
		.usage = "",
	},
	{
		.name = "execute",
		.mode = COMMAND_EXEC,
		.handler = handle_jtag_execute,
		.help = "Execute a chain of JTAG commands",
		.usage = "['scan' tap_name+ ['" JTAG_EXECUTE_SCAN_IR_START_ARG
			"' ir_value+ ['" JTAG_EXECUTE_SCAN_ENDSTATE_NEXT_ARG
			"' tap_state]] ['" JTAG_EXECUTE_SCAN_DR_START_ARG
			"' scan_fields+ ['" JTAG_EXECUTE_SCAN_ENDSTATE_NEXT_ARG
			"' tap_state]]]+",
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

		snprintf(expected_id, sizeof(expected_id), "0x%08" PRIx32,
			(tap->expected_ids_cnt > 0) ? tap->expected_ids[0] : 0);
		if (tap->ignore_version)
			expected_id[2] = '*';

		expected = buf_get_u32(tap->expected, 0, tap->ir_length);
		expected_mask = buf_get_u32(tap->expected_mask, 0, tap->ir_length);

		command_print(CMD,
			"%2u %-18s     %c     0x%08x %s %5u 0x%02x  0x%02x",
			tap->abs_chain_position,
			tap->dotted_name,
			tap->enabled ? 'Y' : 'n',
			(unsigned int)(tap->idcode),
			expected_id,
			tap->ir_length,
			(unsigned int)(expected),
			(unsigned int)(expected_mask));

		for (ii = 1; ii < tap->expected_ids_cnt; ii++) {
			snprintf(expected_id, sizeof(expected_id), "0x%08" PRIx32, tap->expected_ids[ii]);
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
		unsigned int delay;
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
		unsigned int delay;
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
		unsigned int khz = 0;
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

	unsigned int num_clocks;
	COMMAND_PARSE_NUMBER(uint, CMD_ARGV[0], num_clocks);

	jtag_add_runtest(num_clocks, TAP_IDLE);
	return jtag_execute_queue();
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
