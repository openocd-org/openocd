/***************************************************************************
 *   Copyright (C) 2016 by Matthias Welwarsky                              *
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
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *                                                                         *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <stdint.h>
#include "target/arm_adi.h"
#include "target/arm.h"
#include "helper/list.h"
#include "helper/command.h"
#include "transport/transport.h"
#include "jtag/interface.h"

extern const struct command_registration adi_dap_instance_commands[];

static LIST_HEAD(all_dap);

extern const struct dp_ops adiv5_swd_dp_ops;
extern const struct dp_ops adiv6_swd_dp_ops;
extern const struct dp_ops adiv5_jtag_dp_ops;
extern const struct dp_ops adiv6_jtag_dp_ops;
extern void adiv5_dap_instance_init(struct adi_dap *dap);
extern void adiv6_dap_instance_init(struct adi_dap *dap);
extern struct adapter_driver *adapter_driver;

/* DAP command support */
struct arm_dap_object {
	struct list_head lh;
	struct adi_dap dap;
	char *name;
	const struct swd_driver *swd;
	bool adiv6;
	bool adiv5;
};

const char *adi_dap_name(struct adi_dap *self)
{
	struct arm_dap_object *obj = container_of(self, struct arm_dap_object, dap);
	return obj->name;
}

const struct swd_driver *adi_dap_swd_driver(struct adi_dap *self)
{
	struct arm_dap_object *obj = container_of(self, struct arm_dap_object, dap);
	return obj->swd;
}

struct adi_dap *adi_get_dap(struct arm_dap_object *obj)
{
	return &obj->dap;
}
struct adi_dap *dap_instance_by_jim_obj(Jim_Interp *interp, Jim_Obj *o)
{
	struct arm_dap_object *obj = NULL;
	const char *name;
	bool found = false;

	name = Jim_GetString(o, NULL);

	list_for_each_entry(obj, &all_dap, lh) {
		if (!strcmp(name, obj->name)) {
			found = true;
			break;
		}
	}

	if (found)
		return &obj->dap;
	return NULL;
}

static int dap_init_all(void)
{
	struct arm_dap_object *obj;
	int retval;
	uint32_t dpidr;

	LOG_DEBUG("Initializing all DAPs ...");

	list_for_each_entry(obj, &all_dap, lh) {
		struct adi_dap *dap = &obj->dap;

		/* with hla, dap is just a dummy */
		if (transport_is_hla())
			continue;

		/* skip taps that are disabled */
		if (!dap->tap->enabled)
			continue;

		if (transport_is_swd()) {
			dap->dp_ops = &adiv6_swd_dp_ops; /* assume adi-v6 for now */
			obj->swd = adapter_driver->swd_ops;
		} else if (transport_is_dapdirect_swd()) {
			dap->dp_ops = adapter_driver->dap_swd_ops;
		} else if (transport_is_dapdirect_jtag()) {
			dap->dp_ops = adapter_driver->dap_jtag_ops;
		} else {
			/* assume ADI-v6 as the default; change later based on DPIDR read */
			dap->dp_ops = &adiv6_jtag_dp_ops;
		}

		retval = dap->dp_ops->connect(dap);
		if (retval != ERROR_OK)
			return retval;

		if (obj->adiv6)
			LOG_INFO("DAP %s configured to use ADIv6 protocol by user cfg file", jtag_tap_name(dap->tap));
		else if (!obj->adiv5) {
			/***************************************************/
			/* User did not specify ADI-v5 (or v6) override    */
			/* so read DPIDR and switch ADI version if need be */
			/***************************************************/
			retval = dap->dp_ops->queue_dp_read(dap, DP_DPIDR, &dpidr);
			if (retval != ERROR_OK) {
				LOG_ERROR("DAP read of DPIDR failed...");
				return retval;
			}
			retval = dap_run(dap);
			if (retval != ERROR_OK) {
				LOG_ERROR("DAP read of DPIDR failed...");
				return retval;
			}

			if (((dpidr & 0x0000F000) >> 12) < 3) {
				LOG_INFO("DAP %s DPIDR indicates ADIv5 protocol is being used", jtag_tap_name(dap->tap));
				obj->adiv5 = true;
				if (transport_is_swd())
					dap->dp_ops = &adiv5_swd_dp_ops;
				else
					dap->dp_ops = &adiv5_jtag_dp_ops;
				adiv5_dap_instance_init(dap);
				retval = dap->dp_ops->connect(dap);
				if (retval != ERROR_OK)
					return retval;
			} else {   /* target is using an ADI v6 DAP */
				obj->adiv6 = true;
				LOG_INFO("DAP %s DPIDR indicates ADIv6 protocol is being used", jtag_tap_name(dap->tap));
			}
		} else {
			/**************************************************/
			/* User configuration wants to force use of ADI-v5*/
			/* This may be required on DPv0 parts that do not */
			/* have a DPIDR register value indicating ADI-v5  */
			/**************************************************/
			LOG_INFO("DAP %s configured to use ADIv5 protocol by user cfg file", jtag_tap_name(dap->tap));
			if (transport_is_swd())
				dap->dp_ops = &adiv5_swd_dp_ops;
			else
				dap->dp_ops = &adiv5_jtag_dp_ops;
			adiv5_dap_instance_init(dap);
			retval = dap->dp_ops->connect(dap);
			if (retval != ERROR_OK)
				return retval;
		}
		/* see if address size of ROM Table is greater than 32-bits */
		if (obj->adiv6) {
			retval = dap->dp_ops->queue_dp_read(dap, DP_DPIDR1, &dpidr);
			if (retval != ERROR_OK) {
				LOG_ERROR("DAP read of DPIDR1 failed...");
				return retval;
			}
			retval = dap_run(dap);
			if (retval != ERROR_OK) {
				LOG_ERROR("DAP read of DPIDR1 failed...");
				return retval;
			}
			dap->asize = dpidr & 0x0000007F;
		} else
			dap->asize = 32;  /* ADIv5 only supports one select reg */
	}

	return ERROR_OK;
}

int dap_cleanup_all(void)
{
	struct arm_dap_object *obj, *tmp;
	struct adi_dap *dap;

	list_for_each_entry_safe(obj, tmp, &all_dap, lh) {
		dap = &obj->dap;
		if (dap->dp_ops && dap->dp_ops->quit)
			dap->dp_ops->quit(dap);

		free(obj->name);
		free(obj);
	}

	return ERROR_OK;
}

enum dap_cfg_param {
	CFG_CHAIN_POSITION,
	CFG_IGNORE_SYSPWRUPACK,
	CFG_ADIV6,
	CFG_ADIV5,
};

static const Jim_Nvp nvp_config_opts[] = {
	{ .name = "-chain-position",   .value = CFG_CHAIN_POSITION },
	{ .name = "-ignore-syspwrupack", .value = CFG_IGNORE_SYSPWRUPACK },
	{ .name = "-adiv6",   .value = CFG_ADIV6 },
	{ .name = "-adiv5",   .value = CFG_ADIV5 },
	{ .name = NULL, .value = -1 }
};

static int dap_configure(Jim_GetOptInfo *goi, struct arm_dap_object *dap)
{
	struct jtag_tap *tap = NULL;
	Jim_Nvp *n;
	int e;

	/* parse config or cget options ... */
	while (goi->argc > 0) {
		Jim_SetEmptyResult(goi->interp);

		e = Jim_GetOpt_Nvp(goi, nvp_config_opts, &n);
		if (e != JIM_OK) {
			Jim_GetOpt_NvpUnknown(goi, nvp_config_opts, 0);
			return e;
		}
		switch (n->value) {
		case CFG_CHAIN_POSITION: {
			Jim_Obj *o_t;
			e = Jim_GetOpt_Obj(goi, &o_t);
			if (e != JIM_OK)
				return e;
			tap = jtag_tap_by_jim_obj(goi->interp, o_t);
			if (tap == NULL) {
				Jim_SetResultString(goi->interp, "-chain-position is invalid", -1);
				return JIM_ERR;
			}
			/* loop for more */
			break;
		}
		case CFG_IGNORE_SYSPWRUPACK:
			dap->dap.ignore_syspwrupack = true;
			break;
		case CFG_ADIV6:
			dap->adiv5 = false; /* make sure last user designated ADI flag is used */
			dap->adiv6 = true;
			break;
		case CFG_ADIV5:
			dap->adiv6 = false; /* make sure last user designated ADI flag is used */
			dap->adiv5 = true;
			break;
		default:
			break;
		}
	}

	if (tap == NULL) {
		Jim_SetResultString(goi->interp, "-chain-position required when creating DAP", -1);
		return JIM_ERR;
	}

	if (dap->adiv5)
		adiv5_dap_instance_init(&dap->dap);
	else
		adiv6_dap_instance_init(&dap->dap);
	dap->dap.tap = tap;

	return JIM_OK;
}

static int dap_create(Jim_GetOptInfo *goi)
{
	struct command_context *cmd_ctx;
	static struct arm_dap_object *dap;
	Jim_Obj *new_cmd;
	Jim_Cmd *cmd;
	const char *cp;
	int e;

	cmd_ctx = current_command_context(goi->interp);
	assert(cmd_ctx != NULL);

	if (goi->argc < 3) {
		Jim_WrongNumArgs(goi->interp, 1, goi->argv, "?name? ..options...");
		return JIM_ERR;
	}
	/* COMMAND */
	Jim_GetOpt_Obj(goi, &new_cmd);
	/* does this command exist? */
	cmd = Jim_GetCommand(goi->interp, new_cmd, JIM_ERRMSG);
	if (cmd) {
		cp = Jim_GetString(new_cmd, NULL);
		Jim_SetResultFormatted(goi->interp, "Command: %s Exists", cp);
		return JIM_ERR;
	}

	/* Create it */
	dap = calloc(1, sizeof(struct arm_dap_object));
	if (dap == NULL)
		return JIM_ERR;

	e = dap_configure(goi, dap);
	if (e != JIM_OK) {
		free(dap);
		return e;
	}

	cp = Jim_GetString(new_cmd, NULL);
	dap->name = strdup(cp);

	struct command_registration dap_commands[] = {
		{
			.name = cp,
			.mode = COMMAND_ANY,
			.help = "dap instance command group",
			.usage = "",
			.chain = adi_dap_instance_commands,

		},
		COMMAND_REGISTRATION_DONE
	};

	/* don't expose the instance commands when using hla */
	if (transport_is_hla())
		dap_commands[0].chain = NULL;

	e = register_commands(cmd_ctx, NULL, dap_commands);
	if (ERROR_OK != e)
		return JIM_ERR;

	struct command *c = command_find_in_context(cmd_ctx, cp);
	assert(c);
	command_set_handler_data(c, dap);

	list_add_tail(&dap->lh, &all_dap);

	return (ERROR_OK == e) ? JIM_OK : JIM_ERR;
}

static int jim_dap_create(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	Jim_GetOptInfo goi;
	Jim_GetOpt_Setup(&goi, interp, argc - 1, argv + 1);
	if (goi.argc < 2) {
		Jim_WrongNumArgs(goi.interp, goi.argc, goi.argv,
			"<name> [<dap_options> ...]");
		return JIM_ERR;
	}
	return dap_create(&goi);
}

static int jim_dap_names(Jim_Interp *interp, int argc, Jim_Obj *const *argv)
{
	struct arm_dap_object *obj;

	if (argc != 1) {
		Jim_WrongNumArgs(interp, 1, argv, "Too many parameters");
		return JIM_ERR;
	}
	Jim_SetResult(interp, Jim_NewListObj(interp, NULL, 0));
	list_for_each_entry(obj, &all_dap, lh) {
		Jim_ListAppendElement(interp, Jim_GetResult(interp),
			Jim_NewStringObj(interp, obj->name, -1));
	}
	return JIM_OK;
}

COMMAND_HANDLER(handle_dap_init)
{
	return dap_init_all();
}

COMMAND_HANDLER(handle_dap_info_command)
{
	struct adi_dap *dap = adi_get_dap(CMD_DATA);
	uint32_t apsel;

	if (dap == NULL) {
		LOG_ERROR("DAP instance not available. Probably a HLA target...");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	switch (CMD_ARGC) {
		case 0:
			apsel = dap->apsel;
			break;
		case 1:
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], apsel);
			if (apsel > DP_APSEL_MAX)
				return ERROR_COMMAND_SYNTAX_ERROR;
			break;
		default:
			return ERROR_COMMAND_SYNTAX_ERROR;
	}

	return dap->dap_ops->dap_info_command(CMD, &dap->ap[apsel]);

}

COMMAND_HANDLER(dap_apsel_command)
{
	struct adi_dap *dap = adi_get_dap(CMD_DATA);
	uint32_t apsel;

	switch (CMD_ARGC) {
	case 0:
		command_print(CMD, "%" PRIi32, dap->apsel);
		return ERROR_OK;
	case 1:
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], apsel);
		/* AP address is in bits 31:24 of DP_SELECT */
		if (apsel > DP_APSEL_MAX)
			return ERROR_COMMAND_SYNTAX_ERROR;
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	dap->apsel = apsel;
	return ERROR_OK;
}

COMMAND_HANDLER(dap_apcsw_command)
{
	struct adi_dap *dap = adi_get_dap(CMD_DATA);

	return dap->dap_ops->dap_apcsw_command(CMD);
}

COMMAND_HANDLER(dap_apid_command)
{
	struct adi_dap *dap = adi_get_dap(CMD_DATA);

	return dap->dap_ops->dap_apid_command(CMD);
}

COMMAND_HANDLER(dap_apreg_command)
{
	struct adi_dap *dap = adi_get_dap(CMD_DATA);

	return dap->dap_ops->dap_apreg_command(CMD);
}

COMMAND_HANDLER(dap_dpreg_command)
{
	struct adi_dap *dap = adi_get_dap(CMD_DATA);

	return dap->dap_ops->dap_dpreg_command(CMD);
}

COMMAND_HANDLER(dap_baseaddr_command)
{
	struct adi_dap *dap = adi_get_dap(CMD_DATA);

	return dap->dap_ops->dap_baseaddr_command(CMD);
}

COMMAND_HANDLER(dap_memaccess_command)
{
	struct adi_dap *dap = adi_get_dap(CMD_DATA);

	return dap->dap_ops->dap_memaccess_command(CMD);
}

COMMAND_HANDLER(dap_ti_be_32_quirks_command)
{
	struct adi_dap *dap = adi_get_dap(CMD_DATA);
	uint32_t enable = dap->ti_be_32_quirks;

	switch (CMD_ARGC) {
	case 0:
		break;
	case 1:
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], enable);
		if (enable > 1)
			return ERROR_COMMAND_SYNTAX_ERROR;
		break;
	default:
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	dap->ti_be_32_quirks = enable;
	command_print(CMD, "TI BE-32 quirks mode %s",
		enable ? "enabled" : "disabled");

	return 0;
}

static const struct command_registration dap_subcommand_handlers[] = {
	{
		.name = "create",
		.mode = COMMAND_ANY,
		.jim_handler = jim_dap_create,
		.usage = "name '-chain-position' name",
		.help = "Creates a new DAP instance",
	},
	{
		.name = "names",
		.mode = COMMAND_ANY,
		.jim_handler = jim_dap_names,
		.usage = "",
		.help = "Lists all registered DAP instances by name",
	},
	{
		.name = "init",
		.mode = COMMAND_ANY,
		.handler = handle_dap_init,
		.usage = "",
		.help = "Initialize all registered DAP instances"
	},
#if 0
	/* v5/v6 discovery processes are different */
	{
		.name = "info",
		.handler = handle_dap_info_command,
		.mode = COMMAND_EXEC,
		.help = "display ROM table for MEM-AP of current target "
		"(default currently selected AP)",
		.usage = "[ap_num]",
	},
#endif
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration dap_commands[] = {
	{
		.name = "dap",
		.mode = COMMAND_CONFIG,
		.help = "DAP commands",
		.chain = dap_subcommand_handlers,
		.usage = "",
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration adi_dap_instance_commands[] = {
	{
		.name = "info",
		.handler = handle_dap_info_command,
		.mode = COMMAND_EXEC,
		.help = "display ROM table for MEM-AP "
			"(default currently selected AP)",
		.usage = "[ap_num]",
	},
	{
		.name = "apsel",
		.handler = dap_apsel_command,
		.mode = COMMAND_ANY,
		.help = "Set the currently selected AP (default 0) "
			"and display the result",
		.usage = "[ap_num]",
	},
	{
		.name = "apcsw",
		.handler = dap_apcsw_command,
		.mode = COMMAND_ANY,
		.help = "Set CSW default bits",
		.usage = "[value [mask]]",
	},

	{
		.name = "apid",
		.handler = dap_apid_command,
		.mode = COMMAND_EXEC,
		.help = "return ID register from AP "
			"(default currently selected AP)",
		.usage = "[ap_num]",
	},
	{
		.name = "apreg",
		.handler = dap_apreg_command,
		.mode = COMMAND_EXEC,
		.help = "read/write a register from AP "
			"(reg is byte address of a word register, like 0 4 8...)",
		.usage = "ap_num reg [value]",
	},
	{
		.name = "dpreg",
		.handler = dap_dpreg_command,
		.mode = COMMAND_EXEC,
		.help = "read/write a register from DP "
			"(reg is byte address (bank << 4 | reg) of a word register, like 0 4 8...)",
		.usage = "reg [value]",
	},
	{
		.name = "baseaddr",
		.handler = dap_baseaddr_command,
		.mode = COMMAND_EXEC,
		.help = "return debug base address from MEM-AP "
			"(default currently selected AP)",
		.usage = "[ap_num]",
	},
	{
		.name = "memaccess",
		.handler = dap_memaccess_command,
		.mode = COMMAND_EXEC,
		.help = "set/get number of extra tck for MEM-AP memory "
			"bus access [0-255]",
		.usage = "[cycles]",
	},
	{
		.name = "ti_be_32_quirks",
		.handler = dap_ti_be_32_quirks_command,
		.mode = COMMAND_CONFIG,
		.help = "set/get quirks mode for TI TMS450/TMS570 processors",
		.usage = "[enable]",
	},
	COMMAND_REGISTRATION_DONE
};


int dap_register_commands(struct command_context *cmd_ctx)
{
	return register_commands(cmd_ctx, NULL, dap_commands);
}
