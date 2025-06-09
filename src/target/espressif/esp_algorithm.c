// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Espressif chips common algorithm API for OpenOCD                      *
 *   Copyright (C) 2022 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <helper/align.h>
#include <target/algorithm.h>
#include <target/target.h>
#include "esp_algorithm.h"

#define DEFAULT_ALGORITHM_TIMEOUT_MS    40000	/* ms */

static int esp_algorithm_read_stub_logs(struct target *target, struct esp_algorithm_stub *stub)
{
	if (!stub || stub->log_buff_addr == 0 || stub->log_buff_size == 0)
		return ERROR_FAIL;

	uint32_t len = 0;
	int retval = target_read_u32(target, stub->log_buff_addr, &len);
	if (retval != ERROR_OK)
		return retval;

	/* sanity check. log_buff_size = sizeof(len) + sizeof(log_buff) */
	if (len == 0 || len > stub->log_buff_size - 4)
		return ERROR_FAIL;

	uint8_t *log_buff = calloc(1, len);
	if (!log_buff) {
		LOG_ERROR("Failed to allocate memory for the stub log!");
		return ERROR_FAIL;
	}
	retval = target_read_memory(target, stub->log_buff_addr + 4, 1, len, log_buff);
	if (retval == ERROR_OK)
		LOG_OUTPUT("%*.*s", len, len, log_buff);
	free(log_buff);
	return retval;
}

static int esp_algorithm_run_image(struct target *target,
	struct esp_algorithm_run_data *run,
	uint32_t num_args,
	va_list ap)
{
	struct working_area **mem_handles = NULL;

	if (!run || !run->hw)
		return ERROR_FAIL;

	int retval = run->hw->algo_init(target, run, num_args, ap);
	if (retval != ERROR_OK)
		return retval;

	/* allocate memory arguments and fill respective reg params */
	if (run->mem_args.count > 0) {
		mem_handles = calloc(run->mem_args.count, sizeof(*mem_handles));
		if (!mem_handles) {
			LOG_ERROR("Failed to alloc target mem handles!");
			retval = ERROR_FAIL;
			goto _cleanup;
		}
		/* alloc memory args target buffers */
		for (uint32_t i = 0; i < run->mem_args.count; i++) {
			/* small hack: if we need to update some reg param this field holds
			 * appropriate user argument number, */
			/* otherwise should hold UINT_MAX */
			uint32_t usr_param_num = run->mem_args.params[i].address;
			static struct working_area *area;
			retval = target_alloc_working_area(target, run->mem_args.params[i].size, &area);
			if (retval != ERROR_OK) {
				LOG_ERROR("Failed to alloc target buffer!");
				retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
				goto _cleanup;
			}
			mem_handles[i] = area;
			run->mem_args.params[i].address = area->address;
			if (usr_param_num != UINT_MAX) /* if we need update some register param with mem param value */
				esp_algorithm_user_arg_set_uint(run, usr_param_num, run->mem_args.params[i].address);
		}
	}

	if (run->usr_func_init) {
		retval = run->usr_func_init(target, run, run->usr_func_arg);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to prepare algorithm host side args stub (%d)!", retval);
			goto _cleanup;
		}
	}

	LOG_DEBUG("Algorithm start @ " TARGET_ADDR_FMT ", stack %d bytes @ " TARGET_ADDR_FMT,
		run->stub.tramp_mapped_addr, run->stack_size, run->stub.stack_addr);
	retval = target_start_algorithm(target,
		run->mem_args.count, run->mem_args.params,
		run->reg_args.count, run->reg_args.params,
		run->stub.tramp_mapped_addr, 0,
		run->stub.ainfo);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to start algorithm (%d)!", retval);
		goto _cleanup;
	}

	if (run->usr_func) {
		/* give target algorithm stub time to init itself, then user func can communicate to it safely */
		alive_sleep(100);
		retval = run->usr_func(target, run->usr_func_arg);
		if (retval != ERROR_OK)
			LOG_ERROR("Failed to exec algorithm user func (%d)!", retval);
	}
	uint32_t timeout_ms = 0;	/* do not wait if 'usr_func' returned error */
	if (retval == ERROR_OK)
		timeout_ms = run->timeout_ms ? run->timeout_ms : DEFAULT_ALGORITHM_TIMEOUT_MS;
	LOG_DEBUG("Wait algorithm completion");
	retval = target_wait_algorithm(target,
		run->mem_args.count, run->mem_args.params,
		run->reg_args.count, run->reg_args.params,
		0, timeout_ms,
		run->stub.ainfo);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to wait algorithm (%d)!", retval);
		/* target has been forced to stop in target_wait_algorithm() */
	}
	esp_algorithm_read_stub_logs(target, &run->stub);

	if (run->usr_func_done)
		run->usr_func_done(target, run, run->usr_func_arg);

	if (retval != ERROR_OK) {
		LOG_ERROR("Algorithm run failed (%d)!", retval);
	} else {
		run->ret_code = esp_algorithm_user_arg_get_uint(run, 0);
		LOG_DEBUG("Got algorithm RC 0x%" PRIx32, run->ret_code);
	}

_cleanup:
	/* free memory arguments */
	if (mem_handles) {
		for (uint32_t i = 0; i < run->mem_args.count; i++) {
			if (mem_handles[i])
				target_free_working_area(target, mem_handles[i]);
		}
		free(mem_handles);
	}
	run->hw->algo_cleanup(target, run);

	return retval;
}

static int esp_algorithm_run_debug_stub(struct target *target,
	struct esp_algorithm_run_data *run,
	uint32_t num_args,
	va_list ap)
{
	if (!run || !run->hw)
		return ERROR_FAIL;

	int retval = run->hw->algo_init(target, run, num_args, ap);
	if (retval != ERROR_OK)
		return retval;

	LOG_DEBUG("Algorithm start @ " TARGET_ADDR_FMT ", stack %d bytes @ " TARGET_ADDR_FMT,
		run->stub.tramp_mapped_addr, run->stack_size, run->stub.stack_addr);
	retval = target_start_algorithm(target,
		run->mem_args.count, run->mem_args.params,
		run->reg_args.count, run->reg_args.params,
		run->stub.tramp_mapped_addr, 0,
		run->stub.ainfo);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to start algorithm (%d)!", retval);
		goto _cleanup;
	}

	uint32_t timeout_ms = 0;	/* do not wait if 'usr_func' returned error */
	if (retval == ERROR_OK)
		timeout_ms = run->timeout_ms ? run->timeout_ms : DEFAULT_ALGORITHM_TIMEOUT_MS;
	LOG_DEBUG("Wait algorithm completion");
	retval = target_wait_algorithm(target,
		run->mem_args.count, run->mem_args.params,
		run->reg_args.count, run->reg_args.params,
		0, timeout_ms,
		run->stub.ainfo);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to wait algorithm (%d)!", retval);
		/* target has been forced to stop in target_wait_algorithm() */
	}

	if (retval != ERROR_OK) {
		LOG_ERROR("Algorithm run failed (%d)!", retval);
	} else {
		run->ret_code = esp_algorithm_user_arg_get_uint(run, 0);
		LOG_DEBUG("Got algorithm RC 0x%" PRIx32, run->ret_code);
	}

_cleanup:
	run->hw->algo_cleanup(target, run);

	return retval;
}

static void reverse_binary(const uint8_t *src, uint8_t *dest, size_t length)
{
	size_t remaining = length % 4;
	size_t offset = 0;
	size_t aligned_len = ALIGN_UP(length, 4);

	if (remaining > 0) {
		/* Put extra bytes to the beginning with padding */
		memset(dest + remaining, 0xFF, 4 - remaining);
		for (size_t i = 0; i < remaining; i++)
			dest[i] = src[length - remaining + i];
		length -= remaining; /* reverse the others */
		offset = 4;
	}

	for (size_t i = offset; i < aligned_len; i += 4) {
		dest[i + 0] = src[length - i + offset - 4];
		dest[i + 1] = src[length - i + offset - 3];
		dest[i + 2] = src[length - i + offset - 2];
		dest[i + 3] = src[length - i + offset - 1];
	}
}

static int load_section_from_image(struct target *target,
	struct esp_algorithm_run_data *run,
	int section_num,
	bool reverse)
{
	if (!run)
		return ERROR_FAIL;

	struct imagesection *section = &run->image.image.sections[section_num];
	uint32_t sec_wr = 0;
	uint8_t buf[1024];

	assert(sizeof(buf) % 4 == 0);

	while (sec_wr < section->size) {
		uint32_t nb = section->size - sec_wr > sizeof(buf) ? sizeof(buf) : section->size - sec_wr;
		size_t size_read = 0;
		int retval = image_read_section(&run->image.image, section_num, sec_wr, nb, buf, &size_read);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to read stub section (%d)!", retval);
			return retval;
		}

		if (reverse) {
			size_t aligned_len = ALIGN_UP(size_read, 4);
			uint8_t reversed_buf[aligned_len];

			/* Send original size to allow padding */
			reverse_binary(buf, reversed_buf, size_read);

			/*
				The address range accessed via the instruction bus is in reverse order (word-wise) compared to access
				via the data bus. That is to say, address
				0x3FFE_0000 and 0x400B_FFFC access the same word
				0x3FFE_0004 and 0x400B_FFF8 access the same word
				0x3FFE_0008 and 0x400B_FFF4 access the same word
				...
				The data bus and instruction bus of the CPU are still both little-endian,
				so the byte order of individual words is not reversed between address spaces.
				For example, address
				0x3FFE_0000 accesses the least significant byte in the word accessed by 0x400B_FFFC.
				0x3FFE_0001 accesses the second least significant byte in the word accessed by 0x400B_FFFC.
				0x3FFE_0002 accesses the second most significant byte in the word accessed by 0x400B_FFFC.
				For more details, please refer to ESP32 TRM, Internal SRAM1 section.
			*/
			retval = target_write_buffer(target, run->image.dram_org - sec_wr - aligned_len, aligned_len, reversed_buf);
			if (retval != ERROR_OK) {
				LOG_ERROR("Failed to write stub section!");
				return retval;
			}
		} else {
			retval = target_write_buffer(target, section->base_address + sec_wr, size_read, buf);
			if (retval != ERROR_OK) {
				LOG_ERROR("Failed to write stub section!");
				return retval;
			}
		}

		sec_wr += size_read;
	}

	return ERROR_OK;
}

/*
 * Configuration:
 * ----------------------------
 * The linker scripts defines the memory layout for the stub code.
 * The OpenOCD script specifies the workarea address and it's size
 * Sections defined in the linker are organized to share the same addresses with the workarea.
 * code and data sections are located in Internal SRAM1 and OpenOCD fills these sections using the data bus.
 */
int esp_algorithm_load_func_image(struct target *target, struct esp_algorithm_run_data *run)
{
	int retval;
	size_t tramp_sz = 0;
	const uint8_t *tramp = NULL;
	struct duration algo_time;
	bool alloc_code_working_area = true;

	if (!run || !run->hw)
		return ERROR_FAIL;

	if (duration_start(&algo_time) != 0) {
		LOG_ERROR("Failed to start algo time measurement!");
		return ERROR_FAIL;
	}

	if (run->hw->stub_tramp_get) {
		tramp = run->hw->stub_tramp_get(target, &tramp_sz);
		if (!tramp)
			return ERROR_FAIL;
	}

	LOG_DEBUG("stub: base 0x%x, start 0x%" PRIx32 ", %d sections",
		run->image.image.base_address_set ? (unsigned int)run->image.image.base_address : 0,
		run->image.image.start_address,
		run->image.image.num_sections);
	run->stub.entry = run->image.image.start_address;

	/* [code + trampoline] + <padding> + [data] */

	/* ESP32 has reversed memory region. It will use the last part of DRAM, the others will use the first part.
	 * To avoid complexity for the backup/restore process, we will allocate a workarea for all IRAM region from
	 * the beginning. In that case no need to have a padding area.
	 */
	if (run->image.reverse) {
		if (target_alloc_working_area(target, run->image.iram_len, &run->stub.code) != ERROR_OK) {
			LOG_ERROR("no working area available, can't alloc space for stub code!");
			retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
			goto _on_error;
		}
		alloc_code_working_area = false;
	}

	uint32_t code_size = 0;

	/* Load code section */
	for (unsigned int i = 0; i < run->image.image.num_sections; i++) {
		struct imagesection *section = &run->image.image.sections[i];

		if (section->size == 0)
			continue;

		if (section->flags & ESP_IMAGE_ELF_PHF_EXEC) {
			LOG_DEBUG("addr " TARGET_ADDR_FMT ", sz %d, flags %" PRIx64,
				section->base_address, section->size, section->flags);

			if (alloc_code_working_area) {
				retval = target_alloc_working_area(target, section->size, &run->stub.code);
				if (retval != ERROR_OK) {
					LOG_ERROR("no working area available, can't alloc space for stub code!");
					retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
					goto _on_error;
				}
			}

			if (section->base_address == 0) {
				section->base_address = run->stub.code->address;
				/* sanity check, stub is compiled to be run from working area */
			} else if (run->stub.code->address != section->base_address) {
				LOG_ERROR("working area " TARGET_ADDR_FMT " and stub code section " TARGET_ADDR_FMT
					" address mismatch!",
					section->base_address,
					run->stub.code->address);
				retval = ERROR_FAIL;
				goto _on_error;
			}

			retval = load_section_from_image(target, run, i, run->image.reverse);
			if (retval != ERROR_OK)
				goto _on_error;

			code_size += ALIGN_UP(section->size, 4);
			break; /* Stub has one executable text section */
		}
	}

	/* If exists, load trampoline to the code area */
	if (tramp) {
		if (run->stub.tramp_addr == 0) {
			if (alloc_code_working_area) {
				/* alloc trampoline in code working area */
				if (target_alloc_working_area(target, tramp_sz, &run->stub.tramp) != ERROR_OK) {
					LOG_ERROR("no working area available, can't alloc space for stub jumper!");
					retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
					goto _on_error;
				}
				run->stub.tramp_addr = run->stub.tramp->address;
			}
		}

		size_t al_tramp_size = ALIGN_UP(tramp_sz, 4);

		if (run->image.reverse) {
			target_addr_t reversed_tramp_addr = run->image.dram_org - code_size;
			uint8_t reversed_tramp[al_tramp_size];

			/* Send original size to allow padding */
			reverse_binary(tramp, reversed_tramp, tramp_sz);
			run->stub.tramp_addr = reversed_tramp_addr - al_tramp_size;
			LOG_DEBUG("Write reversed tramp to addr " TARGET_ADDR_FMT ", sz %zu", run->stub.tramp_addr, al_tramp_size);
			retval = target_write_buffer(target, run->stub.tramp_addr, al_tramp_size, reversed_tramp);
		} else {
			LOG_DEBUG("Write tramp to addr " TARGET_ADDR_FMT ", sz %zu", run->stub.tramp_addr, tramp_sz);
			retval = target_write_buffer(target, run->stub.tramp_addr, tramp_sz, tramp);
		}

		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to write stub jumper!");
			goto _on_error;
		}

		run->stub.tramp_mapped_addr = run->image.iram_org + code_size;
		code_size += al_tramp_size;
		LOG_DEBUG("Tramp mapped to addr " TARGET_ADDR_FMT, run->stub.tramp_mapped_addr);
	}

	/* allocate dummy space until the data address */
	if (alloc_code_working_area) {
		/* we dont need to restore padding area. */
		uint32_t backup_working_area_prev = target->backup_working_area;
		target->backup_working_area = 0;
		if (target_alloc_working_area(target, run->image.iram_len - code_size, &run->stub.padding) != ERROR_OK) {
			LOG_ERROR("no working area available, can't alloc space for stub code!");
			retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
			goto _on_error;
		}
		target->backup_working_area = backup_working_area_prev;
	}

	/*  Load the data section */
	for (unsigned int i = 0; i < run->image.image.num_sections; i++) {
		struct imagesection *section = &run->image.image.sections[i];

		if (section->size == 0)
			continue;

		if (!(section->flags & ESP_IMAGE_ELF_PHF_EXEC)) {
			LOG_DEBUG("addr " TARGET_ADDR_FMT ", sz %d, flags %" PRIx64, section->base_address, section->size,
				section->flags);
			/* target_alloc_working_area() aligns the whole working area size to 4-byte boundary.
			   We alloc one area for both DATA and BSS, so align each of them ourselves. */
			uint32_t data_sec_sz = ALIGN_UP(section->size, 4);
			LOG_DEBUG("DATA sec size %" PRIu32 " -> %" PRIu32, section->size, data_sec_sz);
			uint32_t bss_sec_sz = ALIGN_UP(run->image.bss_size, 4);
			LOG_DEBUG("BSS sec size %" PRIu32 " -> %" PRIu32, run->image.bss_size, bss_sec_sz);
			if (target_alloc_working_area(target, data_sec_sz + bss_sec_sz, &run->stub.data) != ERROR_OK) {
				LOG_ERROR("no working area available, can't alloc space for stub data!");
				retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
				goto _on_error;
			}
			if (section->base_address == 0) {
				section->base_address = run->stub.data->address;
				/* sanity check, stub is compiled to be run from working area */
			} else if (run->stub.data->address != section->base_address) {
				LOG_ERROR("working area " TARGET_ADDR_FMT
					" and stub data section " TARGET_ADDR_FMT
					" address mismatch!",
					section->base_address,
					run->stub.data->address);
				retval = ERROR_FAIL;
				goto _on_error;
			}

			retval = load_section_from_image(target, run, i, false);
			if (retval != ERROR_OK)
				goto _on_error;
		}
	}

	/* stack */
	if (run->stub.stack_addr == 0 && run->stack_size > 0) {
		/* allocate stack in data working area */
		if (target_alloc_working_area(target, run->stack_size, &run->stub.stack) != ERROR_OK) {
			LOG_ERROR("no working area available, can't alloc stub stack!");
			retval = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
			goto _on_error;
		}
		run->stub.stack_addr = run->stub.stack->address + run->stack_size;
	}

	if (duration_measure(&algo_time) != 0) {
		LOG_ERROR("Failed to stop algo run measurement!");
		retval = ERROR_FAIL;
		goto _on_error;
	}
	LOG_DEBUG("Stub loaded in %g ms", duration_elapsed(&algo_time) * 1000);
	return ERROR_OK;

_on_error:
	esp_algorithm_unload_func_image(target, run);
	return retval;
}

int esp_algorithm_unload_func_image(struct target *target, struct esp_algorithm_run_data *run)
{
	if (!run)
		return ERROR_FAIL;

	target_free_all_working_areas(target);

	run->stub.tramp = NULL;
	run->stub.stack = NULL;
	run->stub.code = NULL;
	run->stub.data = NULL;
	run->stub.padding = NULL;

	return ERROR_OK;
}

int esp_algorithm_exec_func_image_va(struct target *target,
	struct esp_algorithm_run_data *run,
	uint32_t num_args,
	va_list ap)
{
	if (!run || !run->image.image.start_address_set || run->image.image.start_address == 0)
		return ERROR_FAIL;

	return esp_algorithm_run_image(target, run, num_args, ap);
}

int esp_algorithm_load_onboard_func(struct target *target, target_addr_t func_addr, struct esp_algorithm_run_data *run)
{
	int res;
	const uint8_t *tramp = NULL;
	size_t tramp_sz = 0;
	struct duration algo_time;

	if (!run || !run->hw)
		return ERROR_FAIL;

	if (duration_start(&algo_time) != 0) {
		LOG_ERROR("Failed to start algo time measurement!");
		return ERROR_FAIL;
	}

	if (run->hw->stub_tramp_get) {
		tramp = run->hw->stub_tramp_get(target, &tramp_sz);
		if (!tramp)
			return ERROR_FAIL;
	}

	if (tramp_sz > run->on_board.code_buf_size) {
		LOG_ERROR("Stub tramp size %zu bytes exceeds target buf size %d bytes!",
			tramp_sz, run->on_board.code_buf_size);
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (run->stack_size > run->on_board.min_stack_size) {
		LOG_ERROR("Algorithm stack size not fit into the allocated target stack!");
		return ERROR_FAIL;
	}

	run->stub.stack_addr = run->on_board.min_stack_addr + run->stack_size;
	run->stub.tramp_addr = run->on_board.code_buf_addr;
	run->stub.tramp_mapped_addr = run->stub.tramp_addr;
	run->stub.entry = func_addr;

	if (tramp) {
		res = target_write_buffer(target, run->stub.tramp_addr, tramp_sz, tramp);
		if (res != ERROR_OK) {
			LOG_ERROR("Failed to write stub jumper!");
			esp_algorithm_unload_onboard_func(target, run);
			return res;
		}
	}

	if (duration_measure(&algo_time) != 0) {
		LOG_ERROR("Failed to stop algo run measurement!");
		return ERROR_FAIL;
	}
	LOG_DEBUG("Stub loaded in %g ms", duration_elapsed(&algo_time) * 1000);

	return ERROR_OK;
}

int esp_algorithm_unload_onboard_func(struct target *target, struct esp_algorithm_run_data *run)
{
	return ERROR_OK;
}

int esp_algorithm_exec_onboard_func_va(struct target *target,
	struct esp_algorithm_run_data *run,
	uint32_t num_args,
	va_list ap)
{
	return esp_algorithm_run_debug_stub(target, run, num_args, ap);
}
