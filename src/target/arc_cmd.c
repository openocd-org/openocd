// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2013-2015,2019-2020 Synopsys, Inc.                      *
 *   Frank Dols <frank.dols@synopsys.com>                                  *
 *   Mischa Jonker <mischa.jonker@synopsys.com>                            *
 *   Anton Kolesov <anton.kolesov@synopsys.com>                            *
 *   Evgeniy Didin <didin@synopsys.com>                                    *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "arc.h"
#include <helper/nvp.h>

/* --------------------------------------------------------------------------
 *
 *   ARC targets expose command interface.
 *   It can be accessed via GDB through the (gdb) monitor command.
 *
 * ------------------------------------------------------------------------- */


enum add_reg_types {
	CFG_ADD_REG_TYPE_FLAG,
	CFG_ADD_REG_TYPE_STRUCT,
};
/* Add flags register data type */
enum add_reg_type_flags {
	CFG_ADD_REG_TYPE_FLAGS_NAME,
	CFG_ADD_REG_TYPE_FLAGS_FLAG,
};

static const struct nvp nvp_add_reg_type_flags_opts[] = {
	{ .name = "-name",  .value = CFG_ADD_REG_TYPE_FLAGS_NAME },
	{ .name = "-flag",  .value = CFG_ADD_REG_TYPE_FLAGS_FLAG },
	{ .name = NULL,     .value = -1 }
};

/* Helper function to check if all field required for register
 * are set up */
static const char *validate_register(const struct arc_reg_desc * const reg, bool arch_num_set)
{
	/* Check that required fields are set */
	if (!reg->name)
		return "-name option is required";
	if (!reg->gdb_xml_feature)
		return "-feature option is required";
	if (!arch_num_set)
		return "-num option is required";
	if (reg->is_bcr && reg->is_core)
		return "Register cannot be both -core and -bcr.";
	return NULL;
}

static COMMAND_HELPER(arc_handle_add_reg_type_flags_ops, struct arc_reg_data_type *type)
{
	struct reg_data_type_flags_field *fields = type->reg_type_flags_field;
	struct arc_reg_bitfield *bitfields = type->bitfields;
	struct reg_data_type_flags *flags = &type->data_type_flags;
	unsigned int cur_field = 0;

	while (CMD_ARGC) {
		const struct nvp *n = nvp_name2value(nvp_add_reg_type_flags_opts, CMD_ARGV[0]);
		CMD_ARGC--;
		CMD_ARGV++;
		switch (n->value) {
		case CFG_ADD_REG_TYPE_FLAGS_NAME:
			if (!CMD_ARGC)
				return ERROR_COMMAND_ARGUMENT_INVALID;

			const char *name = CMD_ARGV[0];
			CMD_ARGC--;
			CMD_ARGV++;

			if (strlen(name) >= REG_TYPE_MAX_NAME_LENGTH) {
				command_print(CMD, "Reg type name is too big.");
				return ERROR_COMMAND_ARGUMENT_INVALID;
			}

			strcpy((void *)type->data_type.id, name);
			break;

		case CFG_ADD_REG_TYPE_FLAGS_FLAG:
			if (CMD_ARGC < 2)
				return ERROR_COMMAND_ARGUMENT_INVALID;

			uint32_t val;
			const char *field_name = CMD_ARGV[0];
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], val);
			CMD_ARGC -= 2;
			CMD_ARGV += 2;
			bitfields[cur_field].bitfield.start = val;
			bitfields[cur_field].bitfield.end = val;

			if (strlen(field_name) >= REG_TYPE_MAX_NAME_LENGTH) {
				command_print(CMD, "Reg type field_name is too big.");
				return ERROR_COMMAND_ARGUMENT_INVALID;
			}

			fields[cur_field].name = bitfields[cur_field].name;
			strcpy(bitfields[cur_field].name, field_name);

			fields[cur_field].bitfield = &bitfields[cur_field].bitfield;
			if (cur_field > 0)
				fields[cur_field - 1].next = &fields[cur_field];
			else
				flags->fields = fields;

			cur_field += 1;
			break;

		default:
			nvp_unknown_command_print(CMD, nvp_add_reg_type_flags_opts, NULL, CMD_ARGV[-1]);
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
	}

	if (!type->data_type.id) {
		command_print(CMD, "-name is a required option");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(arc_handle_add_reg_type_flags)
{
	int retval;

	LOG_DEBUG("-");

	struct target *target = get_current_target(CMD_CTX);
	if (!target) {
		command_print(CMD, "No current target");
		return ERROR_FAIL;
	}

	/* Check if the amount of arguments is not zero */
	if (CMD_ARGC == 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* Estimate number of registers as (argc - 2)/3 as each -flag option has 2
	 * arguments while -name is required. */
	unsigned int fields_sz = (CMD_ARGC - 2) / 3;

	/* The maximum amount of bitfields is 32 */
	if (fields_sz > 32) {
		command_print(CMD, "The amount of bitfields exceed 32");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	struct arc_reg_data_type *type = calloc(1, sizeof(*type));
	struct reg_data_type_flags_field *fields = calloc(fields_sz, sizeof(*fields));
	struct arc_reg_bitfield *bitfields = calloc(fields_sz, sizeof(*bitfields));
	if (!type || !fields || !bitfields) {
		LOG_ERROR("Out of memory");
		retval = ERROR_FAIL;
		goto fail;
	}
	struct reg_data_type_flags *flags = &type->data_type_flags;
	type->reg_type_flags_field = fields;

	/* Initialize type */
	type->bitfields = bitfields;
	type->data_type.id = type->data_type_id;
	type->data_type.type = REG_TYPE_ARCH_DEFINED;
	type->data_type.type_class = REG_TYPE_CLASS_FLAGS;
	type->data_type.reg_type_flags = flags;
	flags->size = 4; /* For now ARC has only 32-bit registers */

	retval = CALL_COMMAND_HANDLER(arc_handle_add_reg_type_flags_ops, type);
	if (retval != ERROR_OK)
		goto fail;

	arc_reg_data_type_add(target, type);

	LOG_DEBUG("added flags type {name=%s}", type->data_type.id);

	return ERROR_OK;

fail:
	free(type);
	free(fields);
	free(bitfields);

	return retval;
}

/* Add struct register data type */
enum add_reg_type_struct {
	CFG_ADD_REG_TYPE_STRUCT_NAME,
	CFG_ADD_REG_TYPE_STRUCT_BITFIELD,
};

static const struct nvp nvp_add_reg_type_struct_opts[] = {
	{ .name = "-name",     .value = CFG_ADD_REG_TYPE_STRUCT_NAME },
	{ .name = "-bitfield", .value = CFG_ADD_REG_TYPE_STRUCT_BITFIELD },
	{ .name = NULL,     .value = -1 }
};

COMMAND_HANDLER(arc_handle_set_aux_reg)
{
	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);
	if (!target) {
		command_print(CMD, "No current target");
		return ERROR_FAIL;
	}

	/* Register number */
	uint32_t regnum;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], regnum);

	/* Register value */
	uint32_t value;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);

	struct arc_common *arc = target_to_arc(target);
	assert(arc);

	CHECK_RETVAL(arc_jtag_write_aux_reg_one(&arc->jtag_info, regnum, value));

	return ERROR_OK;
}

COMMAND_HANDLER(arc_handle_get_aux_reg)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);
	if (!target) {
		command_print(CMD, "No current target");
		return ERROR_FAIL;
	}

	/* Register number */
	uint32_t regnum;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], regnum);

	struct arc_common *arc = target_to_arc(target);
	assert(arc);

	uint32_t value;
	CHECK_RETVAL(arc_jtag_read_aux_reg_one(&arc->jtag_info, regnum, &value));

	command_print(CMD, "0x%" PRIx32, value);

	return ERROR_OK;
}

COMMAND_HANDLER(arc_handle_get_core_reg)
{
	if (CMD_ARGC != 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);
	if (!target) {
		command_print(CMD, "No current target");
		return ERROR_FAIL;
	}

	/* Register number */
	uint32_t regnum;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], regnum);
	if (regnum > CORE_REG_MAX_NUMBER || regnum == ARC_R61 || regnum == ARC_R62) {
		command_print(CMD, "Core register number %i "
			"is invalid. Must less then 64 and not 61 and 62.", regnum);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	struct arc_common *arc = target_to_arc(target);
	assert(arc);

	/* Read value */
	uint32_t value;
	CHECK_RETVAL(arc_jtag_read_core_reg_one(&arc->jtag_info, regnum, &value));

	command_print(CMD, "0x%" PRIx32, value);

	return ERROR_OK;
}

COMMAND_HANDLER(arc_handle_set_core_reg)
{
	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);
	if (!target) {
		command_print(CMD, "No current target");
		return ERROR_FAIL;
	}

	/* Register number */
	uint32_t regnum;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], regnum);
	if (regnum > CORE_REG_MAX_NUMBER || regnum == ARC_R61 || regnum == ARC_R62) {
		command_print(CMD, "Core register number %i "
			"is invalid. Must less then 64 and not 61 and 62.", regnum);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	/* Register value */
	uint32_t value;
	COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], value);

	struct arc_common *arc = target_to_arc(target);
	assert(arc);

	CHECK_RETVAL(arc_jtag_write_core_reg_one(&arc->jtag_info, regnum, value));

	return ERROR_OK;
}

static const struct command_registration arc_jtag_command_group[] = {
	{
		.name = "get-aux-reg",
		.handler = arc_handle_get_aux_reg,
		.mode = COMMAND_EXEC,
		.help = "Get AUX register by number. This command does a "
			"raw JTAG request that bypasses OpenOCD register cache "
			"and thus is unsafe and can have unexpected consequences. "
			"Use at your own risk.",
		.usage = "<regnum>"
	},
	{
		.name = "set-aux-reg",
		.handler = arc_handle_set_aux_reg,
		.mode = COMMAND_EXEC,
		.help = "Set AUX register by number. This command does a "
			"raw JTAG request that bypasses OpenOCD register cache "
			"and thus is unsafe and can have unexpected consequences. "
			"Use at your own risk.",
		.usage = "<regnum> <value>"
	},
	{
		.name = "get-core-reg",
		.handler = arc_handle_get_core_reg,
		.mode = COMMAND_EXEC,
		.help = "Get/Set core register by number. This command does a "
			"raw JTAG request that bypasses OpenOCD register cache "
			"and thus is unsafe and can have unexpected consequences. "
			"Use at your own risk.",
		.usage = "<regnum> [<value>]"
	},
	{
		.name = "set-core-reg",
		.handler = arc_handle_set_core_reg,
		.mode = COMMAND_EXEC,
		.help = "Get/Set core register by number. This command does a "
			"raw JTAG request that bypasses OpenOCD register cache "
			"and thus is unsafe and can have unexpected consequences. "
			"Use at your own risk.",
		.usage = "<regnum> [<value>]"
	},
	COMMAND_REGISTRATION_DONE
};


/* This function supports only bitfields. */
static COMMAND_HELPER(arc_handle_add_reg_type_struct_opts, struct arc_reg_data_type *type)
{
	struct reg_data_type_struct_field *fields = type->reg_type_struct_field;
	struct arc_reg_bitfield *bitfields = type->bitfields;
	struct reg_data_type_struct *struct_type = &type->data_type_struct;
	unsigned int cur_field = 0;

	while (CMD_ARGC) {
		const struct nvp *n = nvp_name2value(nvp_add_reg_type_struct_opts, CMD_ARGV[0]);
		CMD_ARGC--;
		CMD_ARGV++;
		switch (n->value) {
		case CFG_ADD_REG_TYPE_STRUCT_NAME:
			if (!CMD_ARGC)
				return ERROR_COMMAND_ARGUMENT_INVALID;

			const char *name = CMD_ARGV[0];
			CMD_ARGC--;
			CMD_ARGV++;

			if (strlen(name) >= REG_TYPE_MAX_NAME_LENGTH) {
				command_print(CMD, "Reg type name is too big.");
				return ERROR_COMMAND_ARGUMENT_INVALID;
			}

			strcpy((void *)type->data_type.id, name);
			break;

		case CFG_ADD_REG_TYPE_STRUCT_BITFIELD:
			if (CMD_ARGC < 3)
				return ERROR_COMMAND_ARGUMENT_INVALID;

			uint32_t start_pos, end_pos;
			const char *field_name = CMD_ARGV[0];
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[1], start_pos);
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[2], end_pos);
			CMD_ARGC -= 3;
			CMD_ARGV += 3;
			bitfields[cur_field].bitfield.start = start_pos;
			bitfields[cur_field].bitfield.end = end_pos;
			bitfields[cur_field].bitfield.type = REG_TYPE_INT;

			if (strlen(field_name) >= REG_TYPE_MAX_NAME_LENGTH) {
				command_print(CMD, "Reg type field_name is too big.");
				return ERROR_COMMAND_ARGUMENT_INVALID;
			}

			fields[cur_field].name = bitfields[cur_field].name;
			strcpy(bitfields[cur_field].name, field_name);

			fields[cur_field].bitfield = &bitfields[cur_field].bitfield;
			fields[cur_field].use_bitfields = true;
			if (cur_field > 0)
				fields[cur_field - 1].next = &fields[cur_field];
			else
				struct_type->fields = fields;

			cur_field += 1;

			break;

		default:
			nvp_unknown_command_print(CMD, nvp_add_reg_type_struct_opts, NULL, CMD_ARGV[-1]);
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
	}

	if (!type->data_type.id) {
		command_print(CMD, "-name is a required option");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(arc_handle_add_reg_type_struct)
{
	int retval;

	LOG_DEBUG("-");

	struct target *target = get_current_target(CMD_CTX);
	if (!target) {
		command_print(CMD, "No current target");
		return ERROR_FAIL;
	}

	/* Check if the amount of arguments is not zero */
	if (CMD_ARGC == 0)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* Estimate number of registers as (argc - 2)/4 as each -bitfield option has 3
	 * arguments while -name is required. */
	unsigned int fields_sz = (CMD_ARGC - 2) / 4;

	/* The maximum amount of bitfields is 32 */
	if (fields_sz > 32) {
		command_print(CMD, "The amount of bitfields exceed 32");
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	struct arc_reg_data_type *type = calloc(1, sizeof(*type));
	struct reg_data_type_struct_field *fields = calloc(fields_sz, sizeof(*fields));
	struct arc_reg_bitfield *bitfields = calloc(fields_sz, sizeof(*bitfields));
	if (!type || !fields || !bitfields) {
		LOG_ERROR("Out of memory");
		retval = ERROR_FAIL;
		goto fail;
	}
	struct reg_data_type_struct *struct_type = &type->data_type_struct;
	type->reg_type_struct_field = fields;

	/* Initialize type */
	type->data_type.id = type->data_type_id;
	type->bitfields = bitfields;
	type->data_type.type = REG_TYPE_ARCH_DEFINED;
	type->data_type.type_class = REG_TYPE_CLASS_STRUCT;
	type->data_type.reg_type_struct = struct_type;
	struct_type->size = 4; /* For now ARC has only 32-bit registers */

	retval = CALL_COMMAND_HANDLER(arc_handle_add_reg_type_struct_opts, type);
	if (retval != ERROR_OK)
		goto fail;

	arc_reg_data_type_add(target, type);

	LOG_DEBUG("added struct type {name=%s}", type->data_type.id);

	return ERROR_OK;

fail:
	free(type);
	free(fields);
	free(bitfields);

	return retval;
}

/* Add register */
enum opts_add_reg {
	CFG_ADD_REG_NAME,
	CFG_ADD_REG_ARCH_NUM,
	CFG_ADD_REG_IS_CORE,
	CFG_ADD_REG_IS_BCR,
	CFG_ADD_REG_GDB_FEATURE,
	CFG_ADD_REG_TYPE,
	CFG_ADD_REG_GENERAL,
};

static const struct nvp opts_nvp_add_reg[] = {
	{ .name = "-name",    .value = CFG_ADD_REG_NAME },
	{ .name = "-num",     .value = CFG_ADD_REG_ARCH_NUM },
	{ .name = "-core",    .value = CFG_ADD_REG_IS_CORE },
	{ .name = "-bcr",     .value = CFG_ADD_REG_IS_BCR },
	{ .name = "-feature", .value = CFG_ADD_REG_GDB_FEATURE },
	{ .name = "-type",    .value = CFG_ADD_REG_TYPE },
	{ .name = "-g",       .value = CFG_ADD_REG_GENERAL },
	{ .name = NULL,       .value = -1 }
};

void free_reg_desc(struct arc_reg_desc *r)
{
	free(r->name);
	free(r->gdb_xml_feature);
	free(r);
}

static COMMAND_HELPER(arc_handle_add_reg_do, struct arc_reg_desc *reg)
{
	/* There is no architecture number that we could treat as invalid, so
	 * separate variable required to ensure that arch num has been set. */
	bool arch_num_set = false;
	const char *type_name = "int"; /* Default type */

	/* At least we need to specify 4 parameters: name, number and gdb_feature,
	 * which means there should be 6 arguments. Also there can be additional parameters
	 * "-type <type>", "-g" and  "-core" or "-bcr" which makes maximum 10 parameters. */
	if (CMD_ARGC < 6 || CMD_ARGC > 10)
		return ERROR_COMMAND_SYNTAX_ERROR;

	/* Parse options. */
	while (CMD_ARGC) {
		const struct nvp *n = nvp_name2value(opts_nvp_add_reg, CMD_ARGV[0]);
		CMD_ARGC--;
		CMD_ARGV++;
		switch (n->value) {
		case CFG_ADD_REG_NAME:
			if (!CMD_ARGC)
				return ERROR_COMMAND_ARGUMENT_INVALID;

			reg->name = strdup(CMD_ARGV[0]);
			if (!reg->name) {
				LOG_ERROR("Out of memory");
				return ERROR_FAIL;
			}

			CMD_ARGC--;
			CMD_ARGV++;
			break;

		case CFG_ADD_REG_IS_CORE:
			reg->is_core = true;
			break;

		case CFG_ADD_REG_IS_BCR:
			reg->is_bcr = true;
			break;

		case CFG_ADD_REG_ARCH_NUM:
			if (!CMD_ARGC)
				return ERROR_COMMAND_ARGUMENT_INVALID;

			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], reg->arch_num);
			CMD_ARGC--;
			CMD_ARGV++;

			arch_num_set = true;
			break;

		case CFG_ADD_REG_GDB_FEATURE:
			if (!CMD_ARGC)
				return ERROR_COMMAND_ARGUMENT_INVALID;

			reg->gdb_xml_feature = strdup(CMD_ARGV[0]);
			if (!reg->gdb_xml_feature) {
				LOG_ERROR("Out of memory");
				return ERROR_FAIL;
			}

			CMD_ARGC--;
			CMD_ARGV++;
			break;

		case CFG_ADD_REG_TYPE:
			if (!CMD_ARGC)
				return ERROR_COMMAND_ARGUMENT_INVALID;

			type_name = CMD_ARGV[0];
			CMD_ARGC--;
			CMD_ARGV++;
			break;

		case CFG_ADD_REG_GENERAL:
			reg->is_general = true;
			break;

		default:
			nvp_unknown_command_print(CMD, opts_nvp_add_reg, NULL, CMD_ARGV[-1]);
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}
	}

	/* Check that required fields are set */
	const char * const errmsg = validate_register(reg, arch_num_set);
	if (errmsg) {
		command_print(CMD, "%s", errmsg);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	/* Add new register */
	struct target *target = get_current_target(CMD_CTX);
	if (!target) {
		command_print(CMD, "No current target");
		return ERROR_FAIL;
	}

	reg->target = target;

	int retval = arc_reg_add(target, reg, type_name, strlen(type_name));
	if (retval == ERROR_ARC_REGTYPE_NOT_FOUND) {
		command_print(CMD,
			"Cannot find type `%s' for register `%s'.",
			type_name, reg->name);
		return retval;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(arc_handle_add_reg)
{
	struct arc_reg_desc *reg = calloc(1, sizeof(*reg));
	if (!reg) {
		LOG_ERROR("Out of memory");
		return ERROR_FAIL;
	}

	int retval = CALL_COMMAND_HANDLER(arc_handle_add_reg_do, reg);
	if (retval != ERROR_OK) {
		free_reg_desc(reg);
		return retval;
	}

	return ERROR_OK;
}

/* arc set-reg-exists ($reg_name)+
 * Accepts any amount of register names - will set them as existing in a loop.*/
COMMAND_HANDLER(arc_set_reg_exists)
{
	struct target * const target = get_current_target(CMD_CTX);
	if (!target) {
		command_print(CMD, "Unable to get current target.");
		return ERROR_FAIL;
	}

	if (!CMD_ARGC) {
		command_print(CMD, "At least one register name must be specified.");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	for (unsigned int i = 0; i < CMD_ARGC; i++) {
		const char * const reg_name = CMD_ARGV[i];
		struct reg * const r = arc_reg_get_by_name(target->reg_cache, reg_name, true);

		if (!r) {
			command_print(CMD, "Register `%s' is not found.", reg_name);
			return ERROR_COMMAND_ARGUMENT_INVALID;
		}

		r->exist = true;
	}

	return ERROR_OK;
}

/* arc reg-field  ($reg_name) ($reg_field)
 * Reads struct type register field */
COMMAND_HANDLER(arc_handle_get_reg_field)
{
	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);
	if (!target) {
		command_print(CMD, "No current target");
		return ERROR_FAIL;
	}

	const char *reg_name = CMD_ARGV[0];
	const char *field_name = CMD_ARGV[1];
	uint32_t value;
	int retval = arc_reg_get_field(target, reg_name, field_name, &value);

	switch (retval) {
		case ERROR_OK:
			break;
		case ERROR_ARC_REGISTER_NOT_FOUND:
			command_print(CMD,
				"Register `%s' has not been found.", reg_name);
			return ERROR_COMMAND_ARGUMENT_INVALID;
		case ERROR_ARC_REGISTER_IS_NOT_STRUCT:
			command_print(CMD,
				"Register `%s' must have 'struct' type.", reg_name);
			return ERROR_COMMAND_ARGUMENT_INVALID;
		case ERROR_ARC_REGISTER_FIELD_NOT_FOUND:
			command_print(CMD,
				"Field `%s' has not been found in register `%s'.",
				field_name, reg_name);
			return ERROR_COMMAND_ARGUMENT_INVALID;
		case ERROR_ARC_FIELD_IS_NOT_BITFIELD:
			command_print(CMD,
				"Field `%s' is not a 'bitfield' field in a structure.",
				field_name);
			return ERROR_COMMAND_ARGUMENT_INVALID;
		default:
			/* Pass through other errors. */
			return retval;
	}

	command_print(CMD, "0x%" PRIx32, value);

	return ERROR_OK;
}

COMMAND_HANDLER(arc_l1_cache_disable_auto_cmd)
{
	bool value;
	int retval = 0;
	struct arc_common *arc = target_to_arc(get_current_target(CMD_CTX));
	retval = CALL_COMMAND_HANDLER(handle_command_parse_bool,
		&value, "target has caches enabled");
	arc->has_l2cache = value;
	arc->has_dcache = value;
	arc->has_icache = value;
	return retval;
}

COMMAND_HANDLER(arc_l2_cache_disable_auto_cmd)
{
	struct arc_common *arc = target_to_arc(get_current_target(CMD_CTX));
	return CALL_COMMAND_HANDLER(handle_command_parse_bool,
		&arc->has_l2cache, "target has l2 cache enabled");
}

COMMAND_HANDLER(arc_handle_actionpoints_num)
{
	LOG_DEBUG("-");

	if (CMD_ARGC >= 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD_CTX);
	if (!target) {
		command_print(CMD, "No current target");
		return ERROR_FAIL;
	}

	struct arc_common *arc = target_to_arc(target);
	/* It is not possible to pass &arc->actionpoints_num directly to
	 * handle_command_parse_uint, because this value should be valid during
	 * "actionpoint reset, initiated by arc_set_actionpoints_num.  */
	uint32_t ap_num = arc->actionpoints_num;

	if (CMD_ARGC == 1) {
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[0], ap_num);
		int e = arc_set_actionpoints_num(target, ap_num);
		if (e != ERROR_OK) {
			command_print(CMD,
				"Failed to set number of actionpoints");
			return e;
		}
	}

	command_print(CMD, "%" PRIu32, ap_num);

	return ERROR_OK;
}

/* ----- Exported target commands ------------------------------------------ */

static const struct command_registration arc_l2_cache_group_handlers[] = {
	{
		.name = "auto",
		.handler = arc_l2_cache_disable_auto_cmd,
		.mode = COMMAND_ANY,
		.usage = "(1|0)",
		.help = "Disable or enable L2",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration arc_cache_group_handlers[] = {
	{
		.name = "auto",
		.handler = arc_l1_cache_disable_auto_cmd,
		.mode = COMMAND_ANY,
		.help = "Disable or enable L1",
		.usage = "(1|0)",
	},
	{
		.name = "l2",
		.mode = COMMAND_ANY,
		.help = "L2 cache command group",
		.usage = "",
		.chain = arc_l2_cache_group_handlers,
	},
	COMMAND_REGISTRATION_DONE
};


static const struct command_registration arc_core_command_handlers[] = {
	{
		.name = "add-reg-type-flags",
		.handler = arc_handle_add_reg_type_flags,
		.mode = COMMAND_CONFIG,
		.usage = "-name <string> -flag <name> <position> "
			"[-flag <name> <position>]...",
		.help = "Add new 'flags' register data type. Only single bit flags "
			"are supported. Type name is global. Bitsize of register is fixed "
			"at 32 bits.",
	},
	{
		.name = "add-reg-type-struct",
		.handler = arc_handle_add_reg_type_struct,
		.mode = COMMAND_CONFIG,
		.usage = "-name <string> -bitfield <name> <start> <end> "
			"[-bitfield <name> <start> <end>]...",
		.help = "Add new 'struct' register data type. Only bit-fields are "
			"supported so far, which means that for each bitfield start and end "
			"position bits must be specified. GDB also support type-fields, "
			"where common type can be used instead. Type name is global. Bitsize of "
			"register is fixed at 32 bits.",
	},
	{
		.name = "add-reg",
		.handler = arc_handle_add_reg,
		.mode = COMMAND_CONFIG,
		.usage = "-name <string> -num <int> -feature <string> [-gdbnum <int>] "
			"[-core|-bcr] [-type <type_name>] [-g]",
		.help = "Add new register. Name, architectural number and feature name "
			"are required options. GDB regnum will default to previous register "
			"(gdbnum + 1) and shouldn't be specified in most cases. Type "
			"defaults to default GDB 'int'.",
	},
	{
		.name = "set-reg-exists",
		.handler = arc_set_reg_exists,
		.mode = COMMAND_ANY,
		.usage = "<register-name> [<register-name>]...",
		.help = "Set that register exists. Accepts multiple register names as "
			"arguments.",
	},
	{
		.name = "get-reg-field",
		.handler = arc_handle_get_reg_field,
		.mode = COMMAND_ANY,
		.usage = "<regname> <field_name>",
		.help = "Returns value of field in a register with 'struct' type.",
	},
	{
		.name = "jtag",
		.mode = COMMAND_ANY,
		.help = "ARC JTAG specific commands",
		.usage = "",
		.chain = arc_jtag_command_group,
	},
	{
		.name = "cache",
		.mode = COMMAND_ANY,
		.help = "cache command group",
		.usage = "",
		.chain = arc_cache_group_handlers,
	},
	{
		.name = "num-actionpoints",
		.handler = arc_handle_actionpoints_num,
		.mode = COMMAND_ANY,
		.usage = "[<unsigned integer>]",
		.help = "Prints or sets amount of actionpoints in the processor.",
	},
	COMMAND_REGISTRATION_DONE
};

const struct command_registration arc_monitor_command_handlers[] = {
	{
		.name = "arc",
		.mode = COMMAND_ANY,
		.help = "ARC monitor command group",
		.usage = "",
		.chain = arc_core_command_handlers,
	},
	{
		.chain = rtt_target_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};
