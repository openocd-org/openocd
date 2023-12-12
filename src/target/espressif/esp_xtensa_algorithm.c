// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Module to run arbitrary code on Xtensa using OpenOCD                  *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include <config.h>
#endif

#include <target/xtensa/xtensa.h>
#include "esp_xtensa_algorithm.h"

static int esp_xtensa_algo_init(struct target *target, struct esp_algorithm_run_data *run,
	uint32_t num_args, va_list ap);
static int esp_xtensa_algo_cleanup(struct target *target, struct esp_algorithm_run_data *run);
static const uint8_t *esp_xtensa_stub_tramp_get(struct target *target, size_t *size);

const struct esp_algorithm_hw xtensa_algo_hw = {
	.algo_init = esp_xtensa_algo_init,
	.algo_cleanup = esp_xtensa_algo_cleanup,
	.stub_tramp_get = esp_xtensa_stub_tramp_get,
};

/* Generated from contrib/loaders/trampoline/espressif/xtensa/esp_xtensa_stub_tramp_win.S */
static const uint8_t esp_xtensa_stub_tramp_win[] = {
#include "../../../contrib/loaders/trampoline/espressif/xtensa/esp_xtensa_stub_tramp_win.inc"
};

static const uint8_t *esp_xtensa_stub_tramp_get(struct target *target, size_t *size)
{
	struct xtensa *xtensa = target_to_xtensa(target);

	if (!xtensa->core_config->windowed) {
		LOG_ERROR("Running stubs is not supported for cores without windowed registers option!");
		return NULL;
	}
	*size = sizeof(esp_xtensa_stub_tramp_win);
	return esp_xtensa_stub_tramp_win;
}

static int esp_xtensa_algo_regs_init_start(struct target *target, struct esp_algorithm_run_data *run)
{
	uint32_t stack_addr = run->stub.stack_addr;

	LOG_TARGET_DEBUG(target, "Check stack addr 0x%x", stack_addr);
	if (stack_addr & 0xFUL) {
		stack_addr &= ~0xFUL;
		LOG_TARGET_DEBUG(target, "Adjust stack addr to 0x%x", stack_addr);
	}
	stack_addr -= 16;
	struct reg_param *params = run->reg_args.params;
	init_reg_param(&params[0], "a0", 32, PARAM_OUT);		/*TODO: move to tramp */
	init_reg_param(&params[1], "a1", 32, PARAM_OUT);
	init_reg_param(&params[2], "a8", 32, PARAM_OUT);
	init_reg_param(&params[3], "windowbase", 32, PARAM_OUT);	/*TODO: move to tramp */
	init_reg_param(&params[4], "windowstart", 32, PARAM_OUT);	/*TODO: move to tramp */
	init_reg_param(&params[5], "ps", 32, PARAM_OUT);
	buf_set_u32(params[0].value, 0, 32, 0);	/* a0 TODO: move to tramp */
	buf_set_u32(params[1].value, 0, 32, stack_addr);	/* a1 */
	buf_set_u32(params[2].value, 0, 32, run->stub.entry);	/* a8 */
	buf_set_u32(params[3].value, 0, 32, 0x0);	/* initial window base TODO: move to tramp */
	buf_set_u32(params[4].value, 0, 32, 0x1);	/* initial window start TODO: move to tramp */
	buf_set_u32(params[5].value, 0, 32, 0x60025);	/* enable WOE, UM and debug interrupts level (6) */
	return ERROR_OK;
}

static int esp_xtensa_algo_init(struct target *target, struct esp_algorithm_run_data *run,
	uint32_t num_args, va_list ap)
{
	enum xtensa_mode core_mode = XT_MODE_ANY;
	static const char *const arg_regs[] = { "a2", "a3", "a4", "a5", "a6" };

	if (!run)
		return ERROR_FAIL;

	if (num_args > ARRAY_SIZE(arg_regs)) {
		LOG_ERROR("Too many algo user args %u! Max %zu args are supported.", num_args, ARRAY_SIZE(arg_regs));
		return ERROR_FAIL;
	}

	struct xtensa_algorithm *ainfo = calloc(1, sizeof(struct xtensa_algorithm));
	if (!ainfo) {
		LOG_ERROR("Unable to allocate memory");
		return ERROR_FAIL;
	}

	if (run->arch_info) {
		struct xtensa_algorithm *xtensa_algo = run->arch_info;
		core_mode = xtensa_algo->core_mode;
	}

	run->reg_args.first_user_param = ESP_XTENSA_STUB_ARGS_FUNC_START;
	run->reg_args.count = run->reg_args.first_user_param + num_args;
	if (num_args == 0)
		run->reg_args.count++;	/* a2 reg is used as the 1st arg and return code */
	LOG_DEBUG("reg params count %d (%d/%d).",
		run->reg_args.count,
		run->reg_args.first_user_param,
		num_args);
	run->reg_args.params = calloc(run->reg_args.count, sizeof(struct reg_param));
	if (!run->reg_args.params) {
		free(ainfo);
		LOG_ERROR("Unable to allocate memory");
		return ERROR_FAIL;
	}

	esp_xtensa_algo_regs_init_start(target, run);

	init_reg_param(&run->reg_args.params[run->reg_args.first_user_param + 0], "a2", 32, PARAM_IN_OUT);

	if (num_args > 0) {
		uint32_t arg = va_arg(ap, uint32_t);
		esp_algorithm_user_arg_set_uint(run, 0, arg);
		LOG_DEBUG("Set arg[0] = %d (%s)", arg, run->reg_args.params[run->reg_args.first_user_param + 0].reg_name);
	} else {
		esp_algorithm_user_arg_set_uint(run, 0, 0);
	}

	for (unsigned int i = 1; i < num_args; i++) {
		uint32_t arg = va_arg(ap, uint32_t);
		init_reg_param(&run->reg_args.params[run->reg_args.first_user_param + i], (char *)arg_regs[i], 32, PARAM_OUT);
		esp_algorithm_user_arg_set_uint(run, i, arg);
		LOG_DEBUG("Set arg[%d] = %d (%s)", i, arg, run->reg_args.params[run->reg_args.first_user_param + i].reg_name);
	}

	ainfo->core_mode = core_mode;
	run->stub.ainfo = ainfo;
	return ERROR_OK;
}

static int esp_xtensa_algo_cleanup(struct target *target, struct esp_algorithm_run_data *run)
{
	free(run->stub.ainfo);
	for (uint32_t i = 0; i < run->reg_args.count; i++)
		destroy_reg_param(&run->reg_args.params[i]);
	free(run->reg_args.params);
	return ERROR_OK;
}
