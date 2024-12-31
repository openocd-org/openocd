// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2016 by Matthias Welwarsky                              *
 *                                                                         *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <stdint.h>
#include "target/arm_adi_v5.h"
#include "target/arm_cti.h"
#include "target/target.h"
#include "helper/time_support.h"
#include "helper/list.h"
#include "helper/command.h"

struct arm_cti {
	struct list_head lh;
	char *name;
	struct adiv5_mem_ap_spot spot;
	struct adiv5_ap *ap;
};

static OOCD_LIST_HEAD(all_cti);

const char *arm_cti_name(struct arm_cti *self)
{
	return self->name;
}

struct arm_cti *cti_instance_by_jim_obj(Jim_Interp *interp, Jim_Obj *o)
{
	struct arm_cti *obj = NULL;
	const char *name;
	bool found = false;

	name = Jim_GetString(o, NULL);

	list_for_each_entry(obj, &all_cti, lh) {
		if (!strcmp(name, obj->name)) {
			found = true;
			break;
		}
	}

	if (found)
		return obj;
	return NULL;
}

static int arm_cti_mod_reg_bits(struct arm_cti *self, unsigned int reg, uint32_t mask, uint32_t value)
{
	struct adiv5_ap *ap = self->ap;
	uint32_t tmp;

	/* Read register */
	int retval = mem_ap_read_atomic_u32(ap, self->spot.base + reg, &tmp);
	if (retval != ERROR_OK)
		return retval;

	/* clear bitfield */
	tmp &= ~mask;
	/* put new value */
	tmp |= value & mask;

	/* write new value */
	return mem_ap_write_atomic_u32(ap, self->spot.base + reg, tmp);
}

int arm_cti_enable(struct arm_cti *self, bool enable)
{
	uint32_t val = enable ? 1 : 0;

	return mem_ap_write_atomic_u32(self->ap, self->spot.base + CTI_CTR, val);
}

int arm_cti_ack_events(struct arm_cti *self, uint32_t event)
{
	struct adiv5_ap *ap = self->ap;
	int retval;
	uint32_t tmp;

	retval = mem_ap_write_atomic_u32(ap, self->spot.base + CTI_INACK, event);
	if (retval == ERROR_OK) {
		int64_t then = timeval_ms();
		for (;;) {
			retval = mem_ap_read_atomic_u32(ap, self->spot.base + CTI_TROUT_STATUS, &tmp);
			if (retval != ERROR_OK)
				break;
			if ((tmp & event) == 0)
				break;
			if (timeval_ms() > then + 1000) {
				LOG_ERROR("timeout waiting for target");
				retval = ERROR_TARGET_TIMEOUT;
				break;
			}
		}
	}

	return retval;
}

int arm_cti_gate_channel(struct arm_cti *self, uint32_t channel)
{
	if (channel > 31)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	return arm_cti_mod_reg_bits(self, CTI_GATE, CTI_CHNL(channel), 0);
}

int arm_cti_ungate_channel(struct arm_cti *self, uint32_t channel)
{
	if (channel > 31)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	return arm_cti_mod_reg_bits(self, CTI_GATE, CTI_CHNL(channel), 0xFFFFFFFF);
}

int arm_cti_write_reg(struct arm_cti *self, unsigned int reg, uint32_t value)
{
	return mem_ap_write_atomic_u32(self->ap, self->spot.base + reg, value);
}

int arm_cti_read_reg(struct arm_cti *self, unsigned int reg, uint32_t *p_value)
{
	if (!p_value)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	return mem_ap_read_atomic_u32(self->ap, self->spot.base + reg, p_value);
}

int arm_cti_pulse_channel(struct arm_cti *self, uint32_t channel)
{
	if (channel > 31)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	return arm_cti_write_reg(self, CTI_APPPULSE, CTI_CHNL(channel));
}

int arm_cti_set_channel(struct arm_cti *self, uint32_t channel)
{
	if (channel > 31)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	return arm_cti_write_reg(self, CTI_APPSET, CTI_CHNL(channel));
}

int arm_cti_clear_channel(struct arm_cti *self, uint32_t channel)
{
	if (channel > 31)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	return arm_cti_write_reg(self, CTI_APPCLEAR, CTI_CHNL(channel));
}

static const struct {
	uint32_t offset;
	const char *label;
} cti_names[] = {
	{ CTI_CTR,	"CTR" },
	{ CTI_GATE,	"GATE" },
	{ CTI_INEN0,	"INEN0" },
	{ CTI_INEN1,	"INEN1" },
	{ CTI_INEN2,	"INEN2" },
	{ CTI_INEN3,	"INEN3" },
	{ CTI_INEN4,	"INEN4" },
	{ CTI_INEN5,	"INEN5" },
	{ CTI_INEN6,	"INEN6" },
	{ CTI_INEN7,	"INEN7" },
	{ CTI_INEN8,	"INEN8" },
	{ CTI_OUTEN0,	"OUTEN0" },
	{ CTI_OUTEN1,	"OUTEN1" },
	{ CTI_OUTEN2,	"OUTEN2" },
	{ CTI_OUTEN3,	"OUTEN3" },
	{ CTI_OUTEN4,	"OUTEN4" },
	{ CTI_OUTEN5,	"OUTEN5" },
	{ CTI_OUTEN6,	"OUTEN6" },
	{ CTI_OUTEN7,	"OUTEN7" },
	{ CTI_OUTEN8,	"OUTEN8" },
	{ CTI_TRIN_STATUS,	"TRIN" },
	{ CTI_TROUT_STATUS,	"TROUT" },
	{ CTI_CHIN_STATUS,	"CHIN" },
	{ CTI_CHOU_STATUS,	"CHOUT" },
	{ CTI_APPSET,	"APPSET" },
	{ CTI_APPCLEAR,	"APPCLR" },
	{ CTI_APPPULSE,	"APPPULSE" },
	{ CTI_INACK,	"INACK" },
	{ CTI_DEVCTL,	"DEVCTL" },
};

static int cti_find_reg_offset(const char *name)
{
	for (size_t i = 0; i < ARRAY_SIZE(cti_names); i++) {
		if (!strcmp(name, cti_names[i].label))
			return cti_names[i].offset;
	}

	LOG_ERROR("unknown CTI register %s", name);
	return -1;
}

int arm_cti_cleanup_all(void)
{
	struct arm_cti *obj, *tmp;

	list_for_each_entry_safe(obj, tmp, &all_cti, lh) {
		if (obj->ap)
			dap_put_ap(obj->ap);
		free(obj->name);
		free(obj);
	}

	return ERROR_OK;
}

COMMAND_HANDLER(handle_cti_dump)
{
	struct arm_cti *cti = CMD_DATA;
	struct adiv5_ap *ap = cti->ap;
	int retval = ERROR_OK;
	uint32_t values[ARRAY_SIZE(cti_names)];

	for (size_t i = 0; (retval == ERROR_OK) && (i < ARRAY_SIZE(cti_names)); i++)
		retval = mem_ap_read_u32(ap,
				cti->spot.base + cti_names[i].offset, &values[i]);

	if (retval == ERROR_OK)
		retval = dap_run(ap->dap);

	if (retval != ERROR_OK)
		return JIM_ERR;

	for (size_t i = 0; i < ARRAY_SIZE(cti_names); i++)
		command_print(CMD, "%8.8s (0x%04"PRIx32") 0x%08"PRIx32,
				cti_names[i].label, cti_names[i].offset, values[i]);

	return JIM_OK;
}

COMMAND_HANDLER(handle_cti_enable)
{
	struct arm_cti *cti = CMD_DATA;
	bool on_off;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], on_off);

	return arm_cti_enable(cti, on_off);
}

COMMAND_HANDLER(handle_cti_testmode)
{
	struct arm_cti *cti = CMD_DATA;
	bool on_off;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], on_off);

	return arm_cti_write_reg(cti, 0xf00, on_off ? 0x1 : 0x0);
}

COMMAND_HANDLER(handle_cti_write)
{
	struct arm_cti *cti = CMD_DATA;
	int offset;
	uint32_t value;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	offset = cti_find_reg_offset(CMD_ARGV[0]);
	if (offset < 0)
		return ERROR_FAIL;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);

	return arm_cti_write_reg(cti, offset, value);
}

COMMAND_HANDLER(handle_cti_read)
{
	struct arm_cti *cti = CMD_DATA;
	int offset;
	int retval;
	uint32_t value;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	offset = cti_find_reg_offset(CMD_ARGV[0]);
	if (offset < 0)
		return ERROR_FAIL;

	retval = arm_cti_read_reg(cti, offset, &value);
	if (retval != ERROR_OK)
		return retval;

	command_print(CMD, "0x%08"PRIx32, value);

	return ERROR_OK;
}

COMMAND_HANDLER(handle_cti_ack)
{
	struct arm_cti *cti = CMD_DATA;
	uint32_t event;

	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], event);

	int retval = arm_cti_ack_events(cti, 1 << event);

	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

COMMAND_HANDLER(handle_cti_channel)
{
	struct arm_cti *cti = CMD_DATA;
	int retval = ERROR_OK;
	uint32_t ch_num;

	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], ch_num);

	if (!strcmp(CMD_ARGV[1], "gate"))
		retval = arm_cti_gate_channel(cti, ch_num);
	else if (!strcmp(CMD_ARGV[1], "ungate"))
		retval = arm_cti_ungate_channel(cti, ch_num);
	else if (!strcmp(CMD_ARGV[1], "pulse"))
		retval = arm_cti_pulse_channel(cti, ch_num);
	else if (!strcmp(CMD_ARGV[1], "set"))
		retval = arm_cti_set_channel(cti, ch_num);
	else if (!strcmp(CMD_ARGV[1], "clear"))
		retval = arm_cti_clear_channel(cti, ch_num);
	else {
		command_print(CMD, "Possible channel operations: gate|ungate|set|clear|pulse");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	if (retval != ERROR_OK)
		return retval;

	return ERROR_OK;
}

static const struct command_registration cti_instance_command_handlers[] = {
	{
		.name  = "dump",
		.mode  = COMMAND_EXEC,
		.handler = handle_cti_dump,
		.help  = "dump CTI registers",
		.usage = "",
	},
	{
		.name = "enable",
		.mode = COMMAND_EXEC,
		.handler = handle_cti_enable,
		.help = "enable or disable the CTI",
		.usage = "'on'|'off'",
	},
	{
		.name = "testmode",
		.mode = COMMAND_EXEC,
		.handler = handle_cti_testmode,
		.help = "enable or disable integration test mode",
		.usage = "'on'|'off'",
	},
	{
		.name = "write",
		.mode = COMMAND_EXEC,
		.handler = handle_cti_write,
		.help = "write to a CTI register",
		.usage = "register_name value",
	},
	{
		.name = "read",
		.mode = COMMAND_EXEC,
		.handler = handle_cti_read,
		.help = "read a CTI register",
		.usage = "register_name",
	},
	{
		.name = "ack",
		.mode = COMMAND_EXEC,
		.handler = handle_cti_ack,
		.help = "acknowledge a CTI event",
		.usage = "event",
	},
	{
		.name = "channel",
		.mode = COMMAND_EXEC,
		.handler = handle_cti_channel,
		.help = "do an operation on one CTI channel, possible operations: "
				"gate, ungate, set, clear and pulse",
		.usage = "channel_number operation",
	},
	COMMAND_REGISTRATION_DONE
};

static int cti_configure(struct jim_getopt_info *goi, struct arm_cti *cti)
{
	/* parse config or cget options ... */
	while (goi->argc > 0) {
		int e = adiv5_jim_mem_ap_spot_configure(&cti->spot, goi);

		if (e == JIM_CONTINUE)
			Jim_SetResultFormatted(goi->interp, "unknown option '%s'",
				Jim_String(goi->argv[0]));

		if (e != JIM_OK)
			return JIM_ERR;
	}

	if (!cti->spot.dap) {
		Jim_SetResultString(goi->interp, "-dap required when creating CTI", -1);
		return JIM_ERR;
	}

	return JIM_OK;
}

static int cti_create(struct jim_getopt_info *goi)
{
	struct command_context *cmd_ctx;
	static struct arm_cti *cti;
	Jim_Obj *new_cmd;
	Jim_Cmd *cmd;
	const char *cp;
	int e;

	cmd_ctx = current_command_context(goi->interp);
	assert(cmd_ctx);

	if (goi->argc < 3) {
		Jim_WrongNumArgs(goi->interp, 1, goi->argv, "?name? ..options...");
		return JIM_ERR;
	}
	/* COMMAND */
	jim_getopt_obj(goi, &new_cmd);
	/* does this command exist? */
	cmd = Jim_GetCommand(goi->interp, new_cmd, JIM_NONE);
	if (cmd) {
		cp = Jim_GetString(new_cmd, NULL);
		Jim_SetResultFormatted(goi->interp, "Command: %s Exists", cp);
		return JIM_ERR;
	}

	/* Create it */
	cti = calloc(1, sizeof(*cti));
	if (!cti)
		return JIM_ERR;

	adiv5_mem_ap_spot_init(&cti->spot);

	/* Do the rest as "configure" options */
	goi->is_configure = true;
	e = cti_configure(goi, cti);
	if (e != JIM_OK) {
		free(cti);
		return e;
	}

	cp = Jim_GetString(new_cmd, NULL);
	cti->name = strdup(cp);

	/* now - create the new cti name command */
	const struct command_registration cti_subcommands[] = {
		{
			.chain = cti_instance_command_handlers,
		},
		COMMAND_REGISTRATION_DONE
	};
	const struct command_registration cti_commands[] = {
		{
			.name = cp,
			.mode = COMMAND_ANY,
			.help = "cti instance command group",
			.usage = "",
			.chain = cti_subcommands,
		},
		COMMAND_REGISTRATION_DONE
	};
	e = register_commands_with_data(cmd_ctx, NULL, cti_commands, cti);
	if (e != ERROR_OK)
		return JIM_ERR;

	list_add_tail(&cti->lh, &all_cti);

	cti->ap = dap_get_ap(cti->spot.dap, cti->spot.ap_num);
	if (!cti->ap) {
		Jim_SetResultString(goi->interp, "Cannot get AP", -1);
		return JIM_ERR;
	}

	return JIM_OK;
}

static int jim_cti_create(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	struct jim_getopt_info goi;
	jim_getopt_setup(&goi, interp, argc - 1, argv + 1);
	if (goi.argc < 2) {
		Jim_WrongNumArgs(goi.interp, goi.argc, goi.argv,
			"<name> [<cti_options> ...]");
		return JIM_ERR;
	}
	return cti_create(&goi);
}

COMMAND_HANDLER(cti_handle_names)
{
	struct arm_cti *obj;

	if (CMD_ARGC != 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	list_for_each_entry(obj, &all_cti, lh)
		command_print(CMD, "%s", obj->name);

	return ERROR_OK;
}

static const struct command_registration cti_subcommand_handlers[] = {
	{
		.name = "create",
		.mode = COMMAND_ANY,
		.jim_handler = jim_cti_create,
		.usage = "name '-chain-position' name [options ...]",
		.help = "Creates a new CTI object",
	},
	{
		.name = "names",
		.mode = COMMAND_ANY,
		.handler = cti_handle_names,
		.usage = "",
		.help = "Lists all registered CTI objects by name",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration cti_command_handlers[] = {
	{
		.name = "cti",
		.mode = COMMAND_CONFIG,
		.help = "CTI commands",
		.chain = cti_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

int cti_register_commands(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, cti_command_handlers);
}
