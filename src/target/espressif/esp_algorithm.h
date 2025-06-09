/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Espressif chips common algorithm API for OpenOCD                      *
 *   Copyright (C) 2022 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ESP_ALGORITHM_H
#define OPENOCD_TARGET_ESP_ALGORITHM_H

#include "helper/log.h"
#include "helper/binarybuffer.h"
#include <helper/time_support.h>
#include <target/algorithm.h>
#include <target/image.h>

/**
 * API defined below allows executing pieces of code on target without breaking the execution of the running program.
 * This functionality can be useful for various debugging and maintenance procedures.
 * @note ESP flashing code to load flasher stub on target and write/read/erase flash.
 * Also ESP GCOV command uses some of these functions to run onboard routines to dump coverage info.
 * Stub entry function can take up to 5 arguments and should be of the following form:
 *
 * int stub_entry([uint32_t a1 [, uint32_t a2 [, uint32_t a3 [, uint32_t a4 [, uint32_t a5]]]]]);
 *
 * The general scheme of stub code execution is shown below.
 *
 *  -------                                                    -----------   (initial frame)    ----
 * |       | -------(registers, stub entry, stub args)------> |trampoline | ---(stub args)---> |    |
 * |       |                                                  |           |                    |    |
 * |OpenOCD| <----------(stub-specific communications)---------------------------------------> |stub|
 * |       |                                                  |           |                    |    |
 * |       | <---------(target halted event, ret code)------- |tramp break| <---(ret code)---- |    |
 *  -------                                                    -----------                      ----
 *
 * Procedure of executing stub on target includes:
 * 1) User prepares struct esp_algorithm_run_data and calls one of algorithm_run_xxx() functions.
 * 2) Routine allocates all necessary stub code and data sections.
 * 3) If a user specifies an initializer func esp_algorithm_run_data::usr_func_init
 *    it is called just before the stub starts.
 * 4) If user specifies stub communication func esp_algorithm_run_data::usr_func
 *    (@see esp_flash_write/read in ESP flash driver)
 *    it is called just after the stub starts. When communication with stub is finished this function must return.
 * 5) OpenOCD waits for the stub to finish (hit exit breakpoint).
 * 6) If the user specified arguments cleanup func esp_algorithm_run_data::usr_func_done,
 *    it is called just after the stub finishes.
 *
 * There are two options to run code on target under OpenOCD control:
 * - Run externally compiled stub code.
 * - Run onboard pre-compiled code. @note For ESP chips debug stubs must be enabled in target code @see ESP IDF docs.
 * The main difference between the execution of external stub code and target built-in functions is that
 * in the latter case working areas can not be used to allocate target memory for code and data because they can overlap
 * with code and data involved in onboard function execution. For example, if memory allocated in the working area
 * for the stub stack will overlap with some on-board data used by the stub the stack will get overwritten.
 * The same stands for allocations in target code space.
 *
 * External Code Execution
 * -----------------------
 * To run external code on the target user should use esp_algorithm_run_func_image().
 * In this case all necessary memory (code/data) is allocated in working areas that have fixed configuration
 * defined in target TCL file. Stub code is actually a standalone program, so all its segments must have known
 * addresses due to position-dependent code nature. So stub must be linked in such a way that its code segment
 * starts at the beginning of the working area for code space defined in TCL. The same restriction must be applied
 * to stub's data segment and base addresses of working area for data space. @see ESP stub flasher LD scripts.
 * Also in order to simplify memory allocation BSS section must follow the DATA section in the stub image.
 * The size of the BSS section must be specified in the bss_size field of struct algorithm_image.
 * Sample stub memory map is shown below.
 *  ___________________________________________
 * | data space working area start             |
 * |                                           |
 * | <stub .data segment>                      |
 * |___________________________________________|
 * | stub .bss start                           |
 * |                                           |
 * | <stub .bss segment of size 'bss_size'>    |
 * |___________________________________________|
 * | stub stack base                           |
 * |                                           |
 * | <stub stack>                              |
 * |___________________________________________|
 * |                                           |
 * | <stub mem arg1>                           |
 * |___________________________________________|
 * |                                           |
 * | <stub mem arg2>                           |
 * |___________________________________________|
 *  ___________________________________________
 * | code space working area start             |
 * |                                           |
 * | <stub .text segment>                      |
 * |___________________________________________|
 * |                                           |
 * | <stub trampoline with exit breakpoint>    |
 * |___________________________________________|
 *
 * For example on how to execute external code with memory arguments @see esp_algo_flash_blank_check in
 * ESP flash driver.
 *
 * On-Board Code Execution
 * -----------------------
 * To run on-board code on the target user should use esp_algorithm_run_onboard_func().
 * On-board code execution process does not need to allocate target memory for stub code and data,
 * Because the stub is pre-compiled to the code running on the target.
 * But it still needs memory for stub trampoline, stack, and memory arguments.
 * Working areas can not be used due to possible memory layout conflicts with on-board stub code and data.
 * Debug stubs functionality provided by ESP IDF allows OpenOCD to overcome the above problem.
 * It provides a special descriptor which provides info necessary to safely allocate memory on target.
 * @see struct esp_dbg_stubs_desc.
 * That info is also used to locate memory for stub trampoline code.
 * User can execute target function at any address, but @see ESP IDF debug stubs also provide a way to pass to the host
 * an entry address of pre-defined registered stub functions.
 * For example of an on-board code execution @see esp32_cmd_gcov() in ESP32 apptrace module.
*/

/**
 * Algorithm image data.
 * Helper struct to work with algorithms consisting of code and data segments.
 */
struct esp_algorithm_image {
	/** Image. */
	struct image image;
	/** BSS section size. */
	uint32_t bss_size;
	/** IRAM start address in the linker script */
	uint32_t iram_org;
	/** Total reserved IRAM size */
	uint32_t iram_len;
	/** DRAM start address in the linker script */
	uint32_t dram_org;
	/** Total reserved DRAM size */
	uint32_t dram_len;
	/** IRAM DRAM address range reversed or not */
	bool reverse;
};

#define ESP_IMAGE_ELF_PHF_EXEC			0x1

/**
 * Algorithm stub data.
 */
struct esp_algorithm_stub {
	/** Entry addr. */
	target_addr_t entry;
	/** Working area for code segment. */
	struct working_area *code;
	/** Working area for data segment. */
	struct working_area *data;
	/** Working area for trampoline. */
	struct working_area *tramp;
	/** Working area for padding between code and data area. */
	struct working_area *padding;
	/** Address of the target buffer for stub trampoline. If zero tramp->address will be used. */
	target_addr_t tramp_addr;
	/** Tramp code area will be filled from dbus.
	 *  We need to map it to the ibus to be able to initialize PC register to start algorithm execution from.
	 */
	target_addr_t tramp_mapped_addr;
	/** Working area for stack. */
	struct working_area *stack;
	/** Address of the target buffer for stack. If zero tramp->address will be used. */
	target_addr_t stack_addr;
	/** Address of the log buffer */
	target_addr_t log_buff_addr;
	/** Size of the log buffer */
	uint32_t log_buff_size;
	/** Algorithm's arch-specific info. */
	void *ainfo;
};

/**
 * Algorithm stub in-memory arguments.
 */
struct esp_algorithm_mem_args {
	/** Memory params. */
	struct mem_param *params;
	/** Number of memory params. */
	uint32_t count;
};

/**
 * Algorithm stub register arguments.
 */
struct esp_algorithm_reg_args {
	/** Algorithm register params. User args start from user_first_reg_param */
	struct reg_param *params;
	/** Number of register params. */
	uint32_t count;
	/** The first several reg_params can be used by stub itself (e.g. for trampoline).
	 * This is the index of the first reg_param available for user to pass args to algorithm stub. */
	uint32_t first_user_param;
};

struct esp_algorithm_run_data;

struct esp_algorithm_hw {
	int (*algo_init)(struct target *target, struct esp_algorithm_run_data *run, uint32_t num_args, va_list ap);
	int (*algo_cleanup)(struct target *target, struct esp_algorithm_run_data *run);
	const uint8_t *(*stub_tramp_get)(struct target *target, size_t *size);
};

/**
 * Algorithm run data.
 */
struct esp_algorithm_run_data {
	/** Algorithm completion timeout in ms. If 0, default value will be used */
	uint32_t timeout_ms;
	/** Algorithm stack size. */
	uint32_t stack_size;
	/** Algorithm register arguments. */
	struct esp_algorithm_reg_args reg_args;
	/** Algorithm memory arguments. */
	struct esp_algorithm_mem_args mem_args;
	/** Algorithm arch-specific info. For Xtensa this should point to struct xtensa_algorithm. */
	void *arch_info;
	/** Algorithm return code. */
	int32_t ret_code;
	/** Stub. */
	struct esp_algorithm_stub stub;
	union {
		struct {
			/** Size of the pre-alocated on-board buffer for stub's code. */
			uint32_t code_buf_size;
			/** Address of pre-compiled target buffer for stub trampoline. */
			target_addr_t code_buf_addr;
			/** Size of the pre-alocated on-board buffer for stub's stack. */
			uint32_t min_stack_size;
			/** Pre-compiled target buffer's addr for stack. */
			target_addr_t min_stack_addr;
		} on_board;
		struct esp_algorithm_image image;
	};
	/** Host side algorithm function argument. */
	void *usr_func_arg;

	/**
	 * @brief Host part of algorithm.
	 *        This function will be called while stub is running on target.
	 *        It can be used for communication with stub.
	 *
	 * @param target  Pointer to target.
	 * @param usr_arg Function specific argument.
	 *
	 * @return ERROR_OK on success, otherwise ERROR_XXX.
	 */
	int (*usr_func)(struct target *target, void *usr_arg);

	/**
	 * @brief Algorithm's arguments setup function.
	 *        This function will be called just before stub start.
	 *        It must return when all operations with running stub are completed.
	 *        It can be used to prepare stub memory parameters.
	 *
	 * @param target  Pointer to target.
	 * @param run     Pointer to algo run data.
	 * @param usr_arg Function specific argument. The same as for usr_func.
	 *
	 * @return ERROR_OK on success, otherwise ERROR_XXX.
	 */
	int (*usr_func_init)(struct target *target,
		struct esp_algorithm_run_data *run,
		void *usr_arg);

	/**
	 * @brief Algorithm's arguments cleanup function.
	 *        This function will be called just after stub exit.
	 *        It can be used to cleanup stub memory parameters.
	 *
	 * @param target  Pointer to target.
	 * @param run     Pointer to algo run data.
	 * @param usr_arg Function specific argument. The same as for usr_func.
	 *
	 * @return ERROR_OK on success, otherwise ERROR_XXX.
	 */
	void (*usr_func_done)(struct target *target,
		struct esp_algorithm_run_data *run,
		void *usr_arg);

	/**
	 * @brief Algorithm run function.
	 *
	 * @param target Pointer to target.
	 * @param run    Pointer to algo run data.
	 * @param arg    Function specific argument.
	 *
	 * @return ERROR_OK on success, otherwise ERROR_XXX.
	 */
	int (*algo_func)(struct target *target, struct esp_algorithm_run_data *run, void *arg);

	/** HW specific API */
	const struct esp_algorithm_hw *hw;
};

int esp_algorithm_load_func_image(struct target *target, struct esp_algorithm_run_data *run);
int esp_algorithm_unload_func_image(struct target *target, struct esp_algorithm_run_data *run);

int esp_algorithm_exec_func_image_va(struct target *target,
	struct esp_algorithm_run_data *run,
	uint32_t num_args,
	va_list ap);

/**
 * @brief Loads and runs stub from specified image.
 *        This function should be used to run external stub code on target.
 *
 * @param target   Pointer to target.
 * @param run      Pointer to algo run data.
 * @param num_args Number of stub arguments that follow.
 *
 * @return ERROR_OK on success, otherwise ERROR_XXX. Stub return code is in run->ret_code.
 */
static inline int esp_algorithm_run_func_image_va(struct target *target,
	struct esp_algorithm_run_data *run,
	uint32_t num_args,
	va_list ap)
{
	int ret = esp_algorithm_load_func_image(target, run);
	if (ret != ERROR_OK)
		return ret;
	ret = esp_algorithm_exec_func_image_va(target, run, num_args, ap);
	int rc = esp_algorithm_unload_func_image(target, run);
	return ret != ERROR_OK ? ret : rc;
}

static inline int esp_algorithm_run_func_image(struct target *target,
	struct esp_algorithm_run_data *run,
	uint32_t num_args,
	...)
{
	va_list ap;
	va_start(ap, num_args);
	int retval = esp_algorithm_run_func_image_va(target, run, num_args, ap);
	va_end(ap);
	return retval;
}

int esp_algorithm_load_onboard_func(struct target *target,
	target_addr_t func_addr,
	struct esp_algorithm_run_data *run);
int esp_algorithm_unload_onboard_func(struct target *target, struct esp_algorithm_run_data *run);
int esp_algorithm_exec_onboard_func_va(struct target *target,
	struct esp_algorithm_run_data *run,
	uint32_t num_args,
	va_list ap);

/**
 * @brief Runs pre-compiled on-board function.
 *        This function should be used to run on-board stub code.
 *
 * @param target     Pointer to target.
 * @param run        Pointer to algo run data.
 * @param func_entry Address of the function to run.
 * @param num_args   Number of function arguments that follow.
 *
 * @return ERROR_OK on success, otherwise ERROR_XXX. Stub return code is in run->ret_code.
 */
static inline int esp_algorithm_run_onboard_func_va(struct target *target,
	struct esp_algorithm_run_data *run,
	target_addr_t func_addr,
	uint32_t num_args,
	va_list ap)
{
	int ret = esp_algorithm_load_onboard_func(target, func_addr, run);
	if (ret != ERROR_OK)
		return ret;
	ret = esp_algorithm_exec_onboard_func_va(target, run, num_args, ap);
	if (ret != ERROR_OK)
		return ret;
	return esp_algorithm_unload_onboard_func(target, run);
}

static inline int esp_algorithm_run_onboard_func(struct target *target,
	struct esp_algorithm_run_data *run,
	target_addr_t func_addr,
	uint32_t num_args,
	...)
{
	va_list ap;
	va_start(ap, num_args);
	int retval = esp_algorithm_run_onboard_func_va(target, run, func_addr, num_args, ap);
	va_end(ap);
	return retval;
}

/**
 * @brief Set the value of an argument passed via registers to the stub main function.
 */
static inline void esp_algorithm_user_arg_set_uint(struct esp_algorithm_run_data *run,
	int arg_num,
	uint64_t val)
{
	struct reg_param *param = &run->reg_args.params[run->reg_args.first_user_param + arg_num];

	assert(param->size <= 64);

	if (param->size <= 32)
		buf_set_u32(param->value, 0, param->size, val);
	else
		buf_set_u64(param->value, 0, param->size, val);
}

/**
 * @brief Get the value of an argument passed via registers from the stub main function.
 */
static inline uint64_t esp_algorithm_user_arg_get_uint(struct esp_algorithm_run_data *run, int arg_num)
{
	struct reg_param *param = &run->reg_args.params[run->reg_args.first_user_param + arg_num];

	assert(param->size <= 64);

	if (param->size <= 32)
		return buf_get_u32(param->value, 0, param->size);
	return buf_get_u64(param->value, 0, param->size);
}

#endif	/* OPENOCD_TARGET_ESP_ALGORITHM_H */
