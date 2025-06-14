/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Generic flash driver for Espressif chips                              *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
 ***************************************************************************/

/**
* Overview
* --------
* Like many other flash drivers this one uses special binary program (stub) running on target
* to perform all operations and communicate to the host. Stub has entry function which accepts
* variable number of arguments and therefore can handle different flash operation requests.
* Only the first argument of the stub entry function is mandatory for all operations it must
* specify the type of flash function to perform (read, write etc.). Actually stub main function
* is a dispatcher which determines the type of flash operation to perform, retrieves other
* arguments and calls corresponding handler. In C notation entry function looks like the following:

* int stub_main(int cmd, ...);

* In general every flash operation consists of the following steps:
* 1) Stub is loaded to target.
* 2) Necessary arguments are prepared and stub's main function is called.
* 3) Stub does the work and returns the result.

* Stub Loading
* ------------
* To execute the stub, the code and data sections need to be loaded onto the target device.
* This is accomplished using the working area API. In ESP32, the code and data address spaces are separate,
* but the CPU can read and write memory from both the data bus and the instruction bus.
* This means that data written from the data bus can be accessed from the instruction bus.
* However, ESP32 differs from other chips as it maps these buses differently.
* To address this, the load function in the algorithm will make the necessary adjustments.
* It is crucial that both the stub code and data sections are located at the beginning of their respective
* working areas because the stub code is linked as ELF and therefore its position is dependent.
* Hence, allocating target memory for the stub code and data is the first step in this process.

* Stub Execution
* --------------
* Special wrapping code is used to enter and exit the stub's main function. It prepares register arguments
* before Windowed ABI call to stub entry and upon return from it executes break command to indicate to OpenOCD
* that operation is finished.

* Flash Data Transfers
* --------------------
* To transfer data from/to target a buffer should be allocated at ESP32 side. Also during the data transfer
* target and host must maintain the state of that buffer (read/write pointers etc.). So host needs to check
* the state of that buffer periodically and write to or read from it (depending on flash operation type).
* ESP32 does not support access to its memory via JTAG when it is not halted, so accessing target memory would
* requires halting the CPUs every time the host needs to check if there are incoming data or free space available
* in the buffer. This fact can slow down flash write/read operations dramatically. To avoid this flash driver and
* stub use application level tracing module API to transfer the data in 'non-stop' mode.
*
**/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/sha256.h>
#include <helper/binarybuffer.h>
#include <target/register.h>
#include <target/xtensa/xtensa.h>
#include <target/espressif/esp.h>
#include <helper/time_support.h>
#include <helper/align.h>
#include <target/smp.h>
#include "contrib/loaders/flash/espressif/stub_flasher.h"
#include <target/smp.h>
#include "esp_flash.h"

#define ESP_FLASH_RW_TMO                20000	/* ms */
#define ESP_FLASH_ERASE_TMO             60000	/* ms */
#define ESP_FLASH_VERIFY_TMO            30000	/* ms */
#define ESP_FLASH_MAPS_MAX              2

struct esp_flash_rw_args {
	int (*xfer)(struct target *target, uint32_t block_id, uint32_t len, void *priv);
	uint8_t *buffer;
	uint32_t count;
	uint32_t total_count;
	bool connected;
	const struct esp_flash_apptrace_hw *apptrace;
	target_addr_t apptrace_ctrl_addr;
};

struct esp_flash_write_state {
	struct esp_flash_rw_args rw;
	uint32_t prev_block_id;
	struct working_area *target_buf;
	struct working_area *stub_wargs_area;
	struct esp_flash_stub_flash_write_args stub_wargs;
};

struct esp_flash_read_state {
	struct esp_flash_rw_args rw;
	uint8_t *rd_buf;
};

struct esp_flash_erase_check_args {
	struct working_area *erased_state_buf;
	uint32_t num_sectors;
};

struct esp_flash_bp_op_state {
	struct working_area *target_buf;
	struct esp_flash_bank *esp_info;
	struct esp_flash_breakpoint *sw_bp;
	size_t num_bps;
};

#if BUILD_ESP_COMPRESSION
#include <zlib.h>
static int esp_algo_flash_compress(const uint8_t *in, uint32_t in_len, uint8_t **out, uint32_t *out_len)
{
	z_stream strm;
	int wbits = -MAX_WBITS;		/*deflate */
	int level = Z_DEFAULT_COMPRESSION;	/*Z_BEST_SPEED; */

	/* Don't use Z_NULL to make Sparse tool happy */
	strm.zalloc = NULL;
	strm.zfree = NULL;
	strm.opaque = NULL;

	if (deflateInit2(&strm, level, Z_DEFLATED, wbits, MAX_MEM_LEVEL,
			Z_DEFAULT_STRATEGY) != Z_OK) {
		LOG_ERROR("deflateInit2 error!");
		return ERROR_FAIL;
	}

	strm.avail_out = deflateBound(&strm, (uLong)in_len);

	/* Some compression methods may need a little more space */
	strm.avail_out += 100;

	if (strm.avail_out > INT_MAX) {
		deflateEnd(&strm);
		return ERROR_FAIL;
	}

	*out = (uint8_t *)malloc((int)strm.avail_out);
	if (*out == NULL) {
		LOG_ERROR("out buffer allocation failed!");
		return ERROR_FAIL;
	}
	strm.next_out = *out;
	strm.next_in = (uint8_t *)in;
	strm.avail_in = (uInt)in_len;

	/* always compress in one pass - the return value holds the entire
	 * decompressed data anyway, so there's no reason to do chunked
	 * decompression */
	if (deflate(&strm, Z_FINISH) != Z_STREAM_END) {
		free(*out);
		deflateEnd(&strm);
		LOG_ERROR("not enough output space");
		return ERROR_FAIL;
	}

	deflateEnd(&strm);

	if (strm.total_out > INT_MAX) {
		free(*out);
		LOG_ERROR("too much output");
		return ERROR_FAIL;
	}

	*out_len = strm.total_out;

	LOG_DEBUG("inlen:(%u) outlen:(%u)!", in_len, *out_len);

	return ERROR_OK;
}
#else
static int esp_algo_flash_compress(const uint8_t *in, uint32_t in_len, uint8_t **out, uint32_t *out_len)
{
	return ERROR_FAIL;
}
#endif

static int esp_algo_calc_hash(const uint8_t *data, size_t datalen, uint8_t *hash)
{
	if (data == NULL || hash == NULL || datalen == 0)
		return ERROR_FAIL;

	struct tc_sha256_state_struct sha256_state;

	if (tc_sha256_init(&sha256_state) != TC_CRYPTO_SUCCESS) {
		LOG_ERROR("tc_sha256_init failed!");
		return ERROR_FAIL;
	}

	if (tc_sha256_update(&sha256_state, data, datalen) != TC_CRYPTO_SUCCESS) {
		LOG_ERROR("tc_sha256_update failed!");
		return ERROR_FAIL;
	}

	if (tc_sha256_final(hash, &sha256_state) != TC_CRYPTO_SUCCESS) {
		LOG_ERROR("tc_sha256_final failed!");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

static int esp_algo_flasher_algorithm_init(struct esp_algorithm_run_data *algo,
	const struct esp_algorithm_hw *stub_hw,
	const struct esp_flasher_stub_config *stub_cfg)
{
	if (!stub_cfg) {
		LOG_ERROR("Invalid stub!");
		return ERROR_FAIL;
	}

	memset(algo, 0, sizeof(*algo));
	algo->hw = stub_hw;
	algo->reg_args.first_user_param = stub_cfg->first_user_reg_param;
	algo->image.code_size = stub_cfg->code_sz;
	algo->image.data_size = stub_cfg->data_sz;
	algo->image.bss_size = stub_cfg->bss_sz;
	algo->image.iram_org = stub_cfg->iram_org;
	algo->image.iram_len = stub_cfg->iram_len;
	algo->image.dram_org = stub_cfg->dram_org;
	algo->image.dram_len = stub_cfg->dram_len;
	algo->image.reverse = stub_cfg->reverse;
	algo->stub.log_buff_addr = stub_cfg->log_buff_addr;
	algo->stub.log_buff_size = stub_cfg->log_buff_size;
	memset(&algo->image.image, 0, sizeof(algo->image.image));
	int ret = image_open(&algo->image.image, NULL, "build");
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to create image (%d)!", ret);
		return ret;
	}
	algo->image.image.start_address_set = 1;
	algo->image.image.start_address = stub_cfg->entry_addr;
	ret = image_add_section(&algo->image.image,
		0,
		stub_cfg->code_sz,
		ESP_IMAGE_ELF_PHF_EXEC,
		stub_cfg->code);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to create image (%d)!", ret);
		image_close(&algo->image.image);
		return ret;
	}
	ret = image_add_section(&algo->image.image, 0, stub_cfg->data_sz, 0, stub_cfg->data);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to create image (%d)!", ret);
		image_close(&algo->image.image);
		return ret;
	}
	LOG_DEBUG("base=%08x set=%d",
		(unsigned int)algo->image.image.base_address,
		algo->image.image.base_address_set);
	return ret;
}

int esp_algo_flash_init(struct esp_flash_bank *esp_info, uint32_t sec_sz,
	int (*run_func_image)(struct target *target, struct esp_algorithm_run_data *run,
		uint32_t num_args, ...),
	bool (*is_irom_address)(target_addr_t addr),
	bool (*is_drom_address)(target_addr_t addr),
	const struct esp_flasher_stub_config *(*get_stub)(struct flash_bank *bank, int cmd),
	const struct esp_flash_apptrace_hw *apptrace_hw,
	const struct esp_algorithm_hw *stub_hw,
	bool check_preloaded_binary)
{
	esp_info->probed = 0;
	esp_info->sec_sz = sec_sz;
	esp_info->get_stub = get_stub;
	esp_info->run_func_image = run_func_image;
	esp_info->is_irom_address = is_irom_address;
	esp_info->is_drom_address = is_drom_address;
	esp_info->hw_flash_base = 0;
	esp_info->appimage_flash_base = (uint32_t)-1;
	esp_info->compression = BUILD_ESP_COMPRESSION;
	esp_info->apptrace_hw = apptrace_hw;
	esp_info->stub_hw = stub_hw;
	esp_info->check_preloaded_binary = check_preloaded_binary;

	return ERROR_OK;
}

int esp_algo_flash_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last)
{
	return ERROR_FAIL;
}

int esp_algo_flash_protect_check(struct flash_bank *bank)
{
	return ERROR_OK;
}

int esp_algo_flash_blank_check(struct flash_bank *bank)
{
	struct esp_flash_bank *esp_info = bank->driver_priv;
	struct esp_algorithm_run_data run;
	const struct esp_flasher_stub_config *stub_cfg = esp_info->get_stub(bank, ESP_STUB_CMD_FLASH_ERASE_CHECK);
	const uint32_t stack_size = esp_info->stub_log_enabled ?
		stub_cfg->stack_default_sz * 2 : stub_cfg->stack_default_sz;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted!");
		return ERROR_TARGET_NOT_HALTED;
	}

	int ret = esp_algo_flasher_algorithm_init(&run, esp_info->stub_hw, stub_cfg);
	if (ret != ERROR_OK)
		return ret;

	run.stack_size = stack_size;
	struct mem_param mp;
	init_mem_param(&mp, 3 /*3rd usr arg*/, bank->num_sectors /*size in bytes*/, PARAM_IN);
	run.mem_args.params = &mp;
	run.mem_args.count = 1;

	ret = esp_info->run_func_image(bank->target,
		&run,
		4,
		ESP_STUB_CMD_FLASH_ERASE_CHECK /*cmd*/,
		esp_info->hw_flash_base / esp_info->sec_sz /*start*/,
		bank->num_sectors /*sectors num*/,
		0 /*address to store sectors' state*/);
	image_close(&run.image.image);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to run flasher stub (%d)!", ret);
		destroy_mem_param(&mp);
		return ret;
	}
	if (run.ret_code != ESP_STUB_ERR_OK) {
		LOG_ERROR("Failed to check erase flash (%" PRId32 ")!", run.ret_code);
		ret = ERROR_FAIL;
	} else {
		for (unsigned int i = 0; i < bank->num_sectors; i++)
			bank->sectors[i].is_erased = mp.value[i];
	}
	destroy_mem_param(&mp);
	return ret;
}

static int esp_algo_flash_get_mappings(struct flash_bank *bank,
	struct esp_flash_bank *esp_info,
	struct esp_stub_flash_map *flash_map,
	uint32_t appimage_flash_base)
{
	struct esp_algorithm_run_data run;
	const struct esp_flasher_stub_config *stub_cfg = esp_info->get_stub(bank, ESP_STUB_CMD_FLASH_MAP_GET);
	const uint32_t stack_size = esp_info->stub_log_enabled ?
		stub_cfg->stack_default_sz * 2 : stub_cfg->stack_default_sz;

	int ret = esp_algo_flasher_algorithm_init(&run, esp_info->stub_hw, stub_cfg);
	if (ret != ERROR_OK)
		return ret;

	run.stack_size = stack_size;
	run.check_preloaded_binary = esp_info->stub_log_enabled ? false : esp_info->check_preloaded_binary;

	struct mem_param mp;
	init_mem_param(&mp,
		2 /*2nd usr arg*/,
		sizeof(struct esp_stub_flash_map) /*size in bytes*/,
		PARAM_IN);
	run.mem_args.params = &mp;
	run.mem_args.count = 1;

	ret = esp_info->run_func_image(bank->target,
		&run,
		3 /*args num*/,
		ESP_STUB_CMD_FLASH_MAP_GET /*cmd*/,
		appimage_flash_base,
		0 /*address to store mappings*/);
	image_close(&run.image.image);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to run flasher stub (%d)!", ret);
		destroy_mem_param(&mp);
		return ret;
	}

	flash_map->flash_size = target_buffer_get_u32(bank->target, mp.value + ESP_STUB_FLASHMAP_FLASH_SIZE);
	flash_map->retcode = target_buffer_get_u32(bank->target, mp.value + ESP_STUB_FLASHMAP_RETCODE);

	if (flash_map->retcode != ESP_STUB_ERR_OK) {
		LOG_WARNING("Failed to get flash maps (%" PRId32 ")!", flash_map->retcode);
		if (flash_map->retcode == ESP_STUB_ERR_INVALID_IMAGE)
			LOG_WARNING(
				"Application image is invalid! Check configured binary flash offset 'appimage_offset'.");
		else if (flash_map->retcode == ESP_STUB_ERR_INVALID_PARTITION)
			LOG_WARNING("Invalid partition! One of the partition size exceeds the flash chip size!");
		else if (flash_map->retcode == ESP_STUB_ERR_INVALID_APP_MAGIC)
			LOG_WARNING("Invalid magic number in app image!");
		else if (flash_map->retcode == ESP_STUB_ERR_FLASH_SIZE)
			LOG_WARNING("Failed to read flash size!");
		else if (flash_map->retcode == ESP_STUB_ERR_READ_PARTITION)
			LOG_WARNING("Failed to read partititon table!");
		else if (flash_map->retcode == ESP_STUB_ERR_READ_APP_SEGMENT)
			LOG_WARNING("Failed to read app segment header!");
		else if (flash_map->retcode == ESP_STUB_ERR_READ_APP_IMAGE_HEADER)
			LOG_WARNING("Failed to read app image header!");
		ret = ERROR_FAIL;
	} else {
		flash_map->map.maps_num = target_buffer_get_u32(bank->target, mp.value + ESP_STUB_FLASHMAP_MAPSNUM_OFF);
		if (flash_map->map.maps_num > ESP_FLASH_MAPS_MAX) {
			LOG_ERROR("Too many flash mappings %d! Must be %d.",
				flash_map->map.maps_num,
				ESP_FLASH_MAPS_MAX);
			ret = ERROR_FAIL;
		} else if (flash_map->map.maps_num == 0) {
			LOG_WARNING("Empty flash mapping!");
		} else {
			for (uint32_t i = 0; i < flash_map->map.maps_num; i++) {
				flash_map->map.maps[i].phy_addr =
					target_buffer_get_u32(bank->target, mp.value + ESP_STUB_FLASHMAP_PHYADDR_OFF(i));
				flash_map->map.maps[i].load_addr =
					target_buffer_get_u32(bank->target, mp.value + ESP_STUB_FLASHMAP_LOADADDR_OFF(i));
				flash_map->map.maps[i].size =
					target_buffer_get_u32(bank->target, mp.value + ESP_STUB_FLASHMAP_SIZE_OFF(i));
				LOG_INFO("Flash mapping %d: 0x%x -> 0x%x, %d KB",
					i,
					flash_map->map.maps[i].phy_addr,
					flash_map->map.maps[i].load_addr,
					flash_map->map.maps[i].size / 1024);
			}
		}
	}
	destroy_mem_param(&mp);
	return ret;
}

int esp_algo_flash_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct esp_flash_bank *esp_info = bank->driver_priv;
	struct esp_algorithm_run_data run;
	const struct esp_flasher_stub_config *stub_cfg = esp_info->get_stub(bank, ESP_STUB_CMD_FLASH_ERASE);
	const uint32_t stack_size = esp_info->stub_log_enabled ?
		stub_cfg->stack_default_sz * 2 : stub_cfg->stack_default_sz;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}
	assert((first <= last) && (last < bank->num_sectors));

	struct duration bench;
	duration_start(&bench);

	int ret = esp_algo_flasher_algorithm_init(&run, esp_info->stub_hw, stub_cfg);
	if (ret != ERROR_OK)
		return ret;

	run.stack_size = stack_size;
	run.timeout_ms = ESP_FLASH_ERASE_TMO;
	ret = esp_info->run_func_image(bank->target,
		&run,
		3,
		ESP_STUB_CMD_FLASH_ERASE,
		/* cmd */
		esp_info->hw_flash_base + first * esp_info->sec_sz,
		/* start addr */
		(last - first + 1) * esp_info->sec_sz);			/* size */
	image_close(&run.image.image);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to run flasher stub (%d)!", ret);
		return ret;
	}
	if (run.ret_code != ESP_STUB_ERR_OK) {
		LOG_ERROR("Failed to erase flash (%" PRId32 ")!", run.ret_code);
		ret = ERROR_FAIL;
	} else {
		duration_measure(&bench);
		LOG_INFO("PROF: Erased %d bytes in %g ms",
			(last - first + 1) * esp_info->sec_sz,
			duration_elapsed(&bench) * 1000);
	}
	return ret;
}

static int esp_algo_flash_rw_do(struct target *target, void *priv)
{
	struct duration algo_time, tmo_time;
	struct esp_flash_rw_args *rw = (struct esp_flash_rw_args *)priv;
	int retval = ERROR_OK, busy_num = 0;

	if (duration_start(&algo_time) != 0) {
		LOG_ERROR("Failed to start data write time measurement!");
		return ERROR_FAIL;
	}
	while (rw->total_count < rw->count) {
		uint32_t block_id = 0, len = 0;
		LOG_DEBUG("Transfer block on %s", target_name(target));
		retval = rw->apptrace->data_len_read(target, &block_id, &len);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to read apptrace status (%d)!", retval);
			return retval;
		}
		/* transfer block */
		LOG_DEBUG("Transfer block %d, read %d bytes from target", block_id, len);
		retval = rw->xfer(target, block_id, len, rw);
		if (retval == ERROR_WAIT) {
			LOG_DEBUG("Block not ready");
			if (busy_num++ == 0) {
				if (duration_start(&tmo_time) != 0) {
					LOG_ERROR("Failed to start data write time measurement!");
					return ERROR_FAIL;
				}
			} else {
				/* if no transfer check tmo */
				if (duration_measure(&tmo_time) != 0) {
					LOG_ERROR("Failed to stop algo run measurement!");
					return ERROR_FAIL;
				}
				if (1000 * duration_elapsed(&tmo_time) > ESP_FLASH_RW_TMO) {
					LOG_ERROR("Transfer data tmo!");
					return ERROR_WAIT;
				}
			}
		} else if (retval != ERROR_OK) {
			LOG_ERROR("Failed to transfer flash data block (%d)!", retval);
			return retval;
		} else {
			busy_num = 0;
		}
		if (rw->total_count < rw->count && target->state != TARGET_DEBUG_RUNNING) {
			LOG_ERROR(
				"Algorithm accidentally stopped (%d)! Transferred %" PRIu32 " of %"
				PRIu32,
				target->state,
				rw->total_count,
				rw->count);
			return ERROR_FAIL;
		}
		alive_sleep(10);
		int smp = target->smp;
		target->smp = 0;
		target_poll(target);
		target->smp = smp;
	}
	if (duration_measure(&algo_time) != 0) {
		LOG_ERROR("Failed to stop data write measurement!");
		return ERROR_FAIL;
	}
	LOG_INFO("PROF: Data transferred in %g ms @ %g KB/s",
		duration_elapsed(&algo_time) * 1000,
		duration_kbps(&algo_time, rw->total_count));

	return ERROR_OK;
}

static int esp_algo_flash_write_xfer(struct target *target, uint32_t block_id, uint32_t len, void *priv)
{
	struct esp_flash_write_state *state = (struct esp_flash_write_state *)priv;
	int retval;

	/* check for target to get connected */
	if (!state->rw.connected) {
		retval =
			state->rw.apptrace->ctrl_reg_read(target, NULL, NULL, &state->rw.connected);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to read apptrace control reg (%d)!", retval);
			return retval;
		}
		if (!state->rw.connected)
			return ERROR_WAIT;
		/* got connected, apptrace is initied on target, call `info_init` once more to
		 * update control block info */
		if (state->rw.apptrace->info_init) {
			retval = state->rw.apptrace->info_init(target,
				state->rw.apptrace_ctrl_addr,
				NULL);
			if (retval != ERROR_OK)
				return retval;
		}
	}

	if (state->prev_block_id == block_id)
		return ERROR_WAIT;

	uint32_t wr_sz = state->rw.count - state->rw.total_count <
		state->rw.apptrace->usr_block_max_size_get(target) ?
		state->rw.count -
		state->rw.total_count : state->rw.apptrace->usr_block_max_size_get(target);
	retval = state->rw.apptrace->usr_block_write(target,
		block_id,
		state->rw.buffer + state->rw.total_count,
		wr_sz);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to write apptrace data (%d)!", retval);
		return retval;
	}
	state->rw.total_count += wr_sz;
	state->prev_block_id = block_id;
	LOG_DEBUG("Transferred %u bytes to target. %u/%u", wr_sz, state->rw.total_count, state->rw.count);

	return ERROR_OK;
}

static int esp_algo_flash_write_state_init(struct target *target,
	struct esp_algorithm_run_data *run,
	void *arg)
{
	struct esp_flash_write_state *state = (struct esp_flash_write_state *)arg;
	struct duration algo_time;

	/* clear control register, stub will set APPTRACE_HOST_CONNECT bit when it will be
	 * ready */
	int ret = state->rw.apptrace->ctrl_reg_write(target,
		0 /*block_id*/,
		0 /*len*/,
		false /*conn*/,
		false /*data*/);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to clear apptrace ctrl reg (%d)!", ret);
		return ret;
	}

	/* alloc memory for stub flash write arguments in data working area */
	if (target_alloc_working_area(target, sizeof(state->stub_wargs),
			&state->stub_wargs_area) != ERROR_OK) {
		LOG_ERROR("no working area available, can't alloc space for stub flash arguments!");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	/* memory buffer */
	if (duration_start(&algo_time) != 0) {
		LOG_ERROR("Failed to start workarea alloc time measurement!");
		return ERROR_FAIL;
	}
	uint32_t buffer_size = 64 * 1024;
	while (target_alloc_working_area_try(target, buffer_size,
			&state->target_buf) != ERROR_OK) {
		buffer_size /= 2;
		if (buffer_size == 0) {
			LOG_ERROR("Failed to alloc target buffer for flash data!");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}
	if (duration_measure(&algo_time) != 0) {
		LOG_ERROR("Failed to stop workarea alloc measurement!");
		return ERROR_FAIL;
	}
	LOG_DEBUG("PROF: Allocated target buffer %d bytes in %g ms",
		buffer_size,
		duration_elapsed(&algo_time) * 1000);

	state->stub_wargs.down_buf_addr = state->target_buf->address;
	state->stub_wargs.down_buf_size = state->target_buf->size;

	ret = target_write_buffer(target, state->stub_wargs_area->address,
		sizeof(state->stub_wargs), (uint8_t *)&state->stub_wargs);
	if (ret != ERROR_OK) {
		LOG_ERROR("Write memory at address " TARGET_ADDR_FMT " failed",
			state->stub_wargs_area->address);
		return ERROR_TARGET_FAILURE;
	}

	esp_algorithm_user_arg_set_uint(run, 1, state->stub_wargs_area->address);

	return ERROR_OK;
}

static void esp_algo_flash_write_state_cleanup(struct target *target,
	struct esp_algorithm_run_data *run,
	void *arg)
{
	struct esp_flash_write_state *state = (struct esp_flash_write_state *)arg;
	struct duration algo_time;

	if (!state->target_buf)
		return;
	if (duration_start(&algo_time) != 0)
		LOG_ERROR("Failed to start workarea alloc time measurement!");
	target_free_working_area(target, state->target_buf);
	target_free_working_area(target, state->stub_wargs_area);
	if (duration_measure(&algo_time) != 0)
		LOG_ERROR("Failed to stop data write measurement!");
	LOG_DEBUG("PROF: Workarea freed in %g ms", duration_elapsed(&algo_time) * 1000);
}

static int esp_algo_flash_apptrace_info_init(struct target *target, struct esp_flash_bank *esp_info,
	target_addr_t new_addr, target_addr_t *old_addr)
{
	if (esp_info->apptrace_hw->info_init)
		return esp_info->apptrace_hw->info_init(target, new_addr, old_addr);
	*old_addr = 0;
	return ERROR_OK;
}

static int esp_algo_flash_apptrace_info_restore(struct target *target,
	struct esp_flash_bank *esp_info,
	target_addr_t old_addr)
{
	if (esp_info->apptrace_hw->info_init && old_addr > 0)
		return esp_info->apptrace_hw->info_init(target, old_addr, NULL);
	return ERROR_OK;
}

int esp_algo_flash_write(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct esp_flash_bank *esp_info = bank->driver_priv;
	struct esp_algorithm_run_data run;
	struct esp_flash_write_state wr_state;
	const struct esp_flasher_stub_config *stub_cfg = esp_info->get_stub(bank,
		esp_info->compression ? ESP_STUB_CMD_FLASH_WRITE_DEFLATED : ESP_STUB_CMD_FLASH_WRITE);
	uint8_t *compressed_buff = NULL;
	uint32_t compressed_len = 0;
	uint32_t stack_size = esp_info->stub_log_enabled ?
		stub_cfg->stack_default_sz * 2 : stub_cfg->stack_default_sz;

	if (offset & 0x3UL) {
		LOG_ERROR("Unaligned offset!");
		return ERROR_FAIL;
	}
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	target_addr_t old_addr = 0;
	/* apptrace is not running on target, so not all fields are inited. */
	/* Now we just set control struct addr to be able to communicate and detect that apptrace is
	 * inited */
	/* TODO: for m-core chip stub_cfg->apptrace_ctrl_addr is array address of control structs
	 * for all cores */
	int ret = esp_algo_flash_apptrace_info_init(bank->target,
		esp_info,
		stub_cfg->apptrace_ctrl_addr,
		&old_addr);
	if (ret != ERROR_OK)
		return ret;

	ret = esp_algo_flasher_algorithm_init(&run, esp_info->stub_hw, stub_cfg);
	if (ret != ERROR_OK) {
		esp_algo_flash_apptrace_info_restore(bank->target, esp_info, old_addr);
		return ret;
	}

	if (esp_info->compression) {
		struct duration bench;
		duration_start(&bench);
		if (esp_algo_flash_compress(buffer, count, &compressed_buff,
				&compressed_len) != ERROR_OK) {
			LOG_ERROR("Compression failed!");
			image_close(&run.image.image);
			return ERROR_FAIL;
		}
		duration_measure(&bench);
		LOG_INFO("PROF: Compressed %" PRIu32 " bytes to %" PRIu32 " bytes "
			"in %fms",
			count,
			compressed_len,
			duration_elapsed(&bench) * 1000);

		stack_size += ESP_STUB_IFLATOR_SIZE;
	}

	run.stack_size = stack_size + ESP_STUB_UNZIP_BUFF_SIZE + stub_cfg->stack_data_pool_sz;
	run.usr_func = esp_algo_flash_rw_do;
	run.usr_func_arg = &wr_state;
	run.usr_func_init = esp_algo_flash_write_state_init;
	run.usr_func_done = esp_algo_flash_write_state_cleanup;
	memset(&wr_state, 0, sizeof(struct esp_flash_write_state));
	wr_state.rw.buffer = esp_info->compression ? compressed_buff : (uint8_t *)buffer;
	wr_state.rw.count = esp_info->compression ? compressed_len : count;
	wr_state.rw.xfer = esp_algo_flash_write_xfer;
	wr_state.rw.apptrace = esp_info->apptrace_hw;
	wr_state.prev_block_id = (uint32_t)-1;
	wr_state.rw.apptrace_ctrl_addr = stub_cfg->apptrace_ctrl_addr;
	/* stub flasher arguments */
	wr_state.stub_wargs.size = wr_state.rw.count;
	wr_state.stub_wargs.total_size = count;
	wr_state.stub_wargs.start_addr = esp_info->hw_flash_base + offset;
	wr_state.stub_wargs.down_buf_addr = 0;
	wr_state.stub_wargs.down_buf_size = 0;
	wr_state.stub_wargs.options = ESP_STUB_FLASH_WR_RAW;
	if (esp_info->encryption_needed_on_chip)
		wr_state.stub_wargs.options |= ESP_STUB_FLASH_ENCRYPT_BINARY;

	struct duration wr_time;
	duration_start(&wr_time);

	ret = esp_info->run_func_image(bank->target,
		&run,
		2,
		esp_info->compression ? ESP_STUB_CMD_FLASH_WRITE_DEFLATED :
		ESP_STUB_CMD_FLASH_WRITE,
		/* cmd */
		0
		/* esp_stub_flash_write_args */);
	image_close(&run.image.image);
	if (compressed_buff)
		free(compressed_buff);
	esp_algo_flash_apptrace_info_restore(bank->target, esp_info, old_addr);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to run flasher stub (%d)!", ret);
		return ret;
	}
	if (run.ret_code != ESP_STUB_ERR_OK) {
		LOG_ERROR("Failed to write flash (%" PRId32 ")!", run.ret_code);
		ret = ERROR_FAIL;
	} else {
		duration_measure(&wr_time);
		LOG_INFO("PROF: Wrote %d bytes in %g ms (data transfer time included)",
			wr_state.stub_wargs.total_size,
			duration_elapsed(&wr_time) * 1000);
	}
	return ret;
}

static int esp_algo_flash_read_xfer(struct target *target, uint32_t block_id, uint32_t len, void *priv)
{
	struct esp_flash_read_state *state = (struct esp_flash_read_state *)priv;
	int retval;

	/* check for target to get connected */
	if (!state->rw.connected) {
		retval =
			state->rw.apptrace->ctrl_reg_read(target, NULL, NULL, &state->rw.connected);
		if (retval != ERROR_OK) {
			LOG_ERROR("Failed to read apptrace control reg (%d)!", retval);
			return retval;
		}
		if (!state->rw.connected)
			return ERROR_WAIT;
		/* got connected, apptrace is initied on target, call `info_init` once more to
		 * update control block info */
		if (state->rw.apptrace->info_init) {
			retval = state->rw.apptrace->info_init(target,
				state->rw.apptrace_ctrl_addr,
				NULL);
			if (retval != ERROR_OK)
				return retval;
		}
		state->rd_buf = malloc(state->rw.apptrace->block_max_size_get(target));
		if (!state->rd_buf) {
			LOG_ERROR("Failed to alloc read buffer!");
			return ERROR_FAIL;
		}
	}

	if (len == 0)
		return ERROR_WAIT;

	retval = state->rw.apptrace->data_read(target, len, state->rd_buf, block_id, 1 /*ack*/);
	if (retval != ERROR_OK) {
		LOG_ERROR("Failed to read apptrace status (%d)!", retval);
		return retval;
	}
	LOG_DEBUG("DATA %d bytes: %x %x %x %x %x %x %x %x", len,
		state->rd_buf[0], state->rd_buf[1], state->rd_buf[2], state->rd_buf[3],
		state->rd_buf[4], state->rd_buf[5], state->rd_buf[6], state->rd_buf[7]);

	uint8_t *ptr = state->rd_buf;
	while (ptr < state->rd_buf + len) {
		uint32_t data_sz = 0;
		ptr = state->rw.apptrace->usr_block_get(target, ptr, &data_sz);
		if (data_sz > 0)
			memcpy(state->rw.buffer + state->rw.total_count, ptr, data_sz);
		ptr += data_sz;
		state->rw.total_count += data_sz;
	}

	return ERROR_OK;
}

static int esp_algo_flash_read_state_init(struct target *target,
	struct esp_algorithm_run_data *run,
	void *arg)
{
	struct esp_flash_write_state *state = (struct esp_flash_write_state *)arg;
	/* clear control register, stub will set APPTRACE_HOST_CONNECT bit when it will be
	 * ready */
	int ret = state->rw.apptrace->ctrl_reg_write(target,
		0 /*block_id*/,
		0 /*len*/,
		false /*conn*/,
		false /*data*/);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to clear apptrace ctrl reg (%d)!", ret);
		return ret;
	}
	return ERROR_OK;
}

int esp_algo_flash_read(struct flash_bank *bank, uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct esp_flash_bank *esp_info = bank->driver_priv;
	struct esp_algorithm_run_data run;
	struct esp_flash_read_state rd_state;
	const struct esp_flasher_stub_config *stub_cfg = esp_info->get_stub(bank, ESP_STUB_CMD_FLASH_READ);
	const uint32_t stack_size = esp_info->stub_log_enabled ?
		stub_cfg->stack_default_sz * 2 : stub_cfg->stack_default_sz;

	if (offset & 0x3UL) {
		LOG_ERROR("Unaligned offset!");
		return ERROR_FAIL;
	}
	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	target_addr_t old_addr = 0;
	/* apptrace is not running on target, so not all fields are inited. */
	/* Now we just set control struct addr to be able to communicate and detect that apptrace is
	 * inited */
	/* TODO: for m-core chip stub_cfg->apptrace_ctrl_addr is array address of control structs
	 * for all cores */
	int ret = esp_algo_flash_apptrace_info_init(bank->target,
		esp_info,
		stub_cfg->apptrace_ctrl_addr,
		&old_addr);
	if (ret != ERROR_OK)
		return ret;

	ret = esp_algo_flasher_algorithm_init(&run, esp_info->stub_hw, stub_cfg);
	if (ret != ERROR_OK) {
		esp_algo_flash_apptrace_info_restore(bank->target, esp_info, old_addr);
		return ret;
	}

	run.stack_size = stack_size + stub_cfg->stack_data_pool_sz;
	run.usr_func_init = esp_algo_flash_read_state_init;
	run.usr_func = esp_algo_flash_rw_do;
	run.usr_func_arg = &rd_state;
	memset(&rd_state, 0, sizeof(struct esp_flash_read_state));
	rd_state.rw.buffer = buffer;
	rd_state.rw.count = count;
	rd_state.rw.xfer = esp_algo_flash_read_xfer;
	rd_state.rw.apptrace = esp_info->apptrace_hw;
	rd_state.rw.apptrace_ctrl_addr = stub_cfg->apptrace_ctrl_addr;

	ret = esp_info->run_func_image(bank->target,
		&run,
		3,
		/* cmd */
		ESP_STUB_CMD_FLASH_READ,
		/* start addr */
		esp_info->hw_flash_base + offset,
		/* size */
		count);
	image_close(&run.image.image);
	free(rd_state.rd_buf);
	esp_algo_flash_apptrace_info_restore(bank->target, esp_info, old_addr);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to run flasher stub (%d)!", ret);
		return ret;
	}
	if (run.ret_code != ESP_STUB_ERR_OK) {
		LOG_ERROR("Failed to read flash (%" PRId32 ")!", run.ret_code);
		ret = ERROR_FAIL;
	}
	return ret;
}

#define BANK_SUBNAME(_b_, _n_)  (strcmp((_b_)->name + strlen((_b_)->name) - strlen(_n_), _n_) == 0)

int esp_algo_flash_probe(struct flash_bank *bank)
{
	struct esp_flash_bank *esp_info = bank->driver_priv;
	struct esp_stub_flash_map flash_map = { .map.maps_num = 0 };
	uint32_t irom_base = 0, irom_sz = 0, drom_base = 0, drom_sz = 0, irom_flash_base = 0,
		drom_flash_base = 0;

	esp_info->probed = 0;

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	LOG_DEBUG("Flash size = %d KB @ "TARGET_ADDR_FMT " '%s' - '%s'",
		bank->size / 1024,
		bank->base,
		target_name(bank->target),
		target_state_name(bank->target));

	if (bank->sectors) {
		free(bank->sectors);
		bank->sectors = NULL;
	}

	int ret = esp_algo_flash_get_mappings(bank,
		esp_info,
		&flash_map,
		esp_info->appimage_flash_base);
	if (ret != ERROR_OK || flash_map.map.maps_num == 0) {
		LOG_WARNING("Failed to get flash mappings (%d)!", ret);
		/* if no DROM/IROM mappings so pretend they are at the end of the HW flash bank and
		 * have zero size to allow correct memory map with non zero RAM region */
		irom_base = flash_map.flash_size;
		drom_base = flash_map.flash_size;
	} else {
		/* flash map index 0 belongs to drom */
		if (esp_info->is_drom_address(flash_map.map.maps[0].load_addr)) {
			drom_flash_base = flash_map.map.maps[0].phy_addr & ~(esp_info->sec_sz - 1);
			drom_base = flash_map.map.maps[0].load_addr & ~(esp_info->sec_sz - 1);
			drom_sz = flash_map.map.maps[0].size;
			if (drom_sz & (esp_info->sec_sz - 1))
				drom_sz = (drom_sz & ~(esp_info->sec_sz - 1)) + esp_info->sec_sz;
		}
		/* flash map index 1 belongs to irom */
		if (flash_map.map.maps_num > 1 && esp_info->is_irom_address(flash_map.map.maps[1].load_addr)) {
			irom_flash_base = flash_map.map.maps[1].phy_addr & ~(esp_info->sec_sz - 1);
			irom_base = flash_map.map.maps[1].load_addr & ~(esp_info->sec_sz - 1);
			irom_sz = flash_map.map.maps[1].size;
			if (irom_sz & (esp_info->sec_sz - 1))
				irom_sz = (irom_sz & ~(esp_info->sec_sz - 1)) + esp_info->sec_sz;
		}
	}

	if (BANK_SUBNAME(bank, ".irom")) {
		esp_info->hw_flash_base = irom_flash_base;
		bank->base = irom_base;
		bank->size = irom_sz;
	} else if (BANK_SUBNAME(bank, ".drom")) {
		esp_info->hw_flash_base = drom_flash_base;
		bank->base = drom_base;
		bank->size = drom_sz;
	} else {
		esp_info->hw_flash_base = 0;
		bank->size = flash_map.flash_size;
		if (bank->size == 0) {
			LOG_ERROR("Failed to probe flash, size %d KB", bank->size / 1024);
			return ERROR_FAIL;
		}
		LOG_INFO("Auto-detected flash bank '%s' size %d KB", bank->name, bank->size / 1024);
	}
	LOG_INFO("Using flash bank '%s' size %d KB", bank->name, bank->size / 1024);

	if (bank->size) {
		/* Bank size can be 0 for IROM/DROM emulated banks when there is no app in flash */
		bank->num_sectors = bank->size / esp_info->sec_sz;
		bank->sectors = malloc(sizeof(struct flash_sector) * bank->num_sectors);
		if (bank->sectors == NULL) {
			LOG_ERROR("Failed to alloc mem for sectors!");
			return ERROR_FAIL;
		}
		for (unsigned int i = 0; i < bank->num_sectors; i++) {
			bank->sectors[i].offset = i * esp_info->sec_sz;
			bank->sectors[i].size = esp_info->sec_sz;
			bank->sectors[i].is_erased = -1;
			bank->sectors[i].is_protected = 0;
		}
	}
	LOG_DEBUG("allocated %d sectors", bank->num_sectors);
	esp_info->probed = 1;

	return ERROR_OK;
}

int esp_algo_flash_auto_probe(struct flash_bank *bank)
{
	struct esp_flash_bank *esp_info = bank->driver_priv;
	if (esp_info->probed)
		return ERROR_OK;
	return esp_algo_flash_probe(bank);
}

static int esp_algo_flash_bp_op_state_init(struct target *target,
	struct esp_algorithm_run_data *run,
	void *arg)
{
	struct esp_flash_bp_op_state *state = (struct esp_flash_bp_op_state *)arg;
	struct xtensa *xtensa = target->arch_info;

	LOG_DEBUG("SEC_SIZE %d", state->esp_info->sec_sz);

	size_t alloc_size = state->esp_info->sec_sz;

	/* Check whether we need to allocate 2 sectors for any bp address. */
	for (size_t slot = 0; slot < state->num_bps; ++slot) {
		int inst_size = 2; /* riscv uses 2 byte c.ebreak instruction */
		if (xtensa->common_magic == XTENSA_COMMON_MAGIC) {
			/* Depends on the original inst size. While adding bp, we don't know the size yet. */
			inst_size = state->sw_bp[slot].insn_sz > 0 ? state->sw_bp[slot].insn_sz : XT_ISNS_SZ_MAX;
		}
		/* Check if replacing the break instruction will overlap with the next sector. */
		int sector_no = ALIGN_DOWN(state->sw_bp[slot].bp_flash_addr,
			state->esp_info->sec_sz - 1) / state->esp_info->sec_sz;
		int next_sector_no = ALIGN_DOWN(state->sw_bp[slot].bp_flash_addr + inst_size,
			state->esp_info->sec_sz - 1) / state->esp_info->sec_sz;
		if (next_sector_no > sector_no) {
			alloc_size *= 2;
			/* we don't need to check the others */
			break;
		}
	}

	if (run->run_preloaded_binary) {
		/* On the target, as a BP backup/restore space 4K is allocated at the (iram_start + 0x1000) address */
		/* If alloc_size is 8K, we will allocate memory from working area by adding necessary padding of 4K bytes */
		/* Because first 4K is reserved for the code at the target. We don't want stub code to override it */
		if (alloc_size > state->esp_info->sec_sz) {
			uint32_t backup_working_area_prev = target->backup_working_area;
			target->backup_working_area = 0;
			if (target_alloc_working_area(target, 0x1000, &run->stub.padding) != ERROR_OK) {
				LOG_ERROR("no working area available, can't alloc space for stub code!");
				return ERROR_FAIL;
			}
			target->backup_working_area = backup_working_area_prev;
		} else {
			target_addr_t addr = run->image.iram_org + state->esp_info->sec_sz;
			esp_algorithm_user_arg_set_uint(run, 3, addr);
			return ERROR_OK;
		}
	}

	/* allocate target buffer for temp storage of flash sections contents when modifying instruction */
	int ret = target_alloc_working_area(target, alloc_size, &state->target_buf);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to alloc (%zu) bytes target buffer for insn sectors!", alloc_size);
		return ret;
	}
	/* insn sectors buffer */
	esp_algorithm_user_arg_set_uint(run, 3, state->target_buf->address);

	return ERROR_OK;
}

static void esp_algo_flash_bp_op_state_cleanup(struct target *target,
	struct esp_algorithm_run_data *run,
	void *arg)
{
	struct esp_flash_bp_op_state *state = (struct esp_flash_bp_op_state *)arg;
	if (!state->target_buf)
		return;
	target_free_working_area(target, state->target_buf);
}

int esp_algo_flash_breakpoint_prepare(struct target *target,
	struct breakpoint *breakpoint,
	struct esp_flash_breakpoint *sw_bp)
{
	struct flash_bank *bank;

	int ret = get_flash_bank_by_addr(target, breakpoint->address, true, &bank);
	if (ret != ERROR_OK) {
		LOG_ERROR("%s: Failed to get flash bank (%d)!", target_name(target), ret);
		return ret;
	}

	/* can set set breakpoints in mapped app regions only */
	if (!BANK_SUBNAME(bank, ".irom")) {
		LOG_ERROR("%s: Can not set BP outside of IROM (BP addr " TARGET_ADDR_FMT ")!",
			target_name(target),
			breakpoint->address);
		return ERROR_FAIL;
	}

	struct esp_flash_bank *esp_info = bank->driver_priv;

	sw_bp->oocd_bp = breakpoint;
	sw_bp->bank = bank;
	sw_bp->bp_address = breakpoint->address;

	uint32_t bp_flash_addr = esp_info->hw_flash_base + (breakpoint->address - bank->base);
	sw_bp->bp_flash_addr = bp_flash_addr;
	bp_flash_addr &= ~(esp_info->sec_sz - 1);
	sw_bp->sector_num = bp_flash_addr / esp_info->sec_sz;
	return ERROR_OK;
}

int esp_algo_flash_breakpoint_add(struct target *target, struct esp_flash_breakpoint *sw_bp, size_t num_bps)
{
	struct flash_bank *bank = sw_bp[0].bank;
	struct esp_flash_bank *esp_info = bank->driver_priv;
	struct esp_algorithm_run_data run;
	struct esp_flash_bp_op_state op_state;
	struct mem_param mp[2]; /* in and out */
	const size_t size_bp_inst = sizeof(sw_bp->bp_flash_addr);
	const struct esp_flasher_stub_config *stub_cfg = esp_info->get_stub(bank, ESP_STUB_CMD_FLASH_BP_SET);
	const uint32_t stack_size = esp_info->stub_log_enabled ?
		stub_cfg->stack_default_sz * 2 : stub_cfg->stack_default_sz;

	op_state.esp_info = esp_info;
	op_state.sw_bp = sw_bp;
	op_state.num_bps = num_bps;
	op_state.target_buf = NULL;

	LOG_DEBUG("SEC_SIZE % " PRId32 " num_bps:(%zu)", esp_info->sec_sz, num_bps);

	int ret = esp_algo_flasher_algorithm_init(&run, esp_info->stub_hw, stub_cfg);
	if (ret != ERROR_OK)
		return ret;

	run.stack_size = stack_size;
	run.usr_func_arg = &op_state;
	run.usr_func_init = esp_algo_flash_bp_op_state_init;
	run.usr_func_done = esp_algo_flash_bp_op_state_cleanup;
	run.check_preloaded_binary = esp_info->stub_log_enabled ? false : esp_info->check_preloaded_binary;

	init_mem_param(&mp[0], 1 /* First user arg */, num_bps * size_bp_inst /* size in bytes */, PARAM_OUT);
	/* bp0_flash_addr + bp1_flash_addr + bp2_flash_addr + ... bpn_flash_addr */
	for (size_t slot = 0; slot < num_bps; ++slot)
		target_buffer_set_u32(target, &mp[0].value[slot * size_bp_inst], sw_bp[slot].bp_flash_addr);

	init_mem_param(&mp[1], 2 /* 2nd usr arg */, num_bps * size_bp_inst /* size in bytes */, PARAM_IN);

	run.mem_args.params = mp;
	run.mem_args.count = 2;

	ret = esp_info->run_func_image(target,
		&run,
		5 /* args num */,
		ESP_STUB_CMD_FLASH_BP_SET /* cmd */,
		0 /* address to store bp flash addresses (out) */,
		0 /* address to store original instructions for each breakpoints (in) */,
		0 /* address to store insn sectors */,
		num_bps);
	image_close(&run.image.image);
	if (ret != ERROR_OK) {
		LOG_ERROR("%s: Failed to run flasher stub (%d)!", target_name(target), ret);
		destroy_mem_param(&mp[0]);
		destroy_mem_param(&mp[1]);
		sw_bp->oocd_bp = NULL;
		return ret;
	}
	if (run.ret_code <= 0) {
		LOG_ERROR("%s: Failed to set bp (%" PRId32 ")!", target_name(target), run.ret_code);
		destroy_mem_param(&mp[0]);
		destroy_mem_param(&mp[1]);
		sw_bp->oocd_bp = NULL;
		return ERROR_FAIL;
	}
	/* insn_sz0 + inst0 + insn_sz1 + inst1 + insn_sz2 + inst2 + ... insn_szn + instn */
	assert((uint8_t)run.ret_code == num_bps * sizeof(struct esp_flash_stub_bp_instructions));
	struct esp_flash_stub_bp_instructions *bp_insts = (struct esp_flash_stub_bp_instructions *)mp[1].value;
	for (size_t slot = 0; slot < num_bps; ++slot) {
		sw_bp[slot].insn_sz = bp_insts[slot].size;
		memcpy(sw_bp[slot].insn, &bp_insts[slot].buff, sw_bp[slot].insn_sz);
		sw_bp[slot].oocd_bp->is_set = true;
		sw_bp[slot].action = ESP_BP_ACT_ADD; /* to be compatible with non lazy-process */
		sw_bp[slot].status = ESP_BP_STAT_DONE;
		LOG_TARGET_DEBUG(target, "Placed flash SW breakpoint at " TARGET_ADDR_FMT
			", insn [%02x %02x %02x %02x] %d bytes",
			sw_bp[slot].bp_address,
			sw_bp[slot].insn[0],
			sw_bp[slot].insn[1],
			sw_bp[slot].insn[2],
			sw_bp[slot].insn[3],
			sw_bp[slot].insn_sz);
	}
	destroy_mem_param(&mp[0]);
	destroy_mem_param(&mp[1]);

	return ERROR_OK;
}

int esp_algo_flash_breakpoint_remove(struct target *target, struct esp_flash_breakpoint *sw_bp, size_t num_bps)
{
	struct flash_bank *bank = (struct flash_bank *)(sw_bp->bank);
	struct esp_flash_bank *esp_info = bank->driver_priv;
	struct esp_algorithm_run_data run;
	struct esp_flash_bp_op_state op_state;
	struct mem_param mp[2]; /* out and out */
	const size_t size_bp_inst = sizeof(struct esp_flash_stub_bp_instructions);
	const struct esp_flasher_stub_config *stub_cfg = esp_info->get_stub(bank, ESP_STUB_CMD_FLASH_BP_CLEAR);
	const uint32_t stack_size = esp_info->stub_log_enabled ?
		stub_cfg->stack_default_sz * 2 : stub_cfg->stack_default_sz;

	int ret = esp_algo_flasher_algorithm_init(&run, esp_info->stub_hw, stub_cfg);
	if (ret != ERROR_OK)
		return ret;

	op_state.esp_info = esp_info;
	op_state.sw_bp = sw_bp;
	op_state.num_bps = num_bps;
	op_state.target_buf = NULL;

	run.stack_size = stack_size;
	run.usr_func_arg = &op_state;
	run.usr_func_init = esp_algo_flash_bp_op_state_init;
	run.usr_func_done = esp_algo_flash_bp_op_state_cleanup;
	run.check_preloaded_binary = esp_info->stub_log_enabled ? false : esp_info->check_preloaded_binary;

	init_mem_param(&mp[0], 1 /* First user arg */, 1 + num_bps * size_bp_inst /* size in bytes */, PARAM_OUT);
	/* bp0_flash_addr + bp1_flash_addr + bp2_flash_addr + ... bpn_flash_addr */
	for (size_t slot = 0; slot < num_bps; ++slot)
		target_buffer_set_u32(target, &mp[0].value[slot * size_bp_inst], sw_bp[slot].bp_flash_addr);

	init_mem_param(&mp[1], 2 /* 2nd usr arg */, num_bps * size_bp_inst /* size in bytes */, PARAM_OUT);
	struct esp_flash_stub_bp_instructions *bp_insts = (struct esp_flash_stub_bp_instructions *)mp[1].value;
	for (size_t slot = 0; slot < num_bps; ++slot) {
		bp_insts[slot].size = sw_bp[slot].insn_sz;
		memcpy(&bp_insts[slot].buff, sw_bp[slot].insn, sw_bp[slot].insn_sz);
	}
	run.mem_args.params = mp;
	run.mem_args.count = 2;
	for (size_t slot = 0; slot < num_bps; ++slot) {
		LOG_TARGET_DEBUG(target, "Remove flash SW breakpoint at " TARGET_ADDR_FMT
			", insn [%02x %02x %02x %02x] %d bytes",
			sw_bp[slot].bp_address,
			sw_bp[slot].insn[0],
			sw_bp[slot].insn[1],
			sw_bp[slot].insn[2],
			sw_bp[slot].insn[3],
			sw_bp[slot].insn_sz);
	}

	ret = esp_info->run_func_image(target,
		&run,
		5 /*args num*/,
		ESP_STUB_CMD_FLASH_BP_CLEAR /*cmd*/,
		0 /* address to store bp flash address (out) */,
		0 /* address to restore original instructions for each breakpoints (out) */,
		0 /* address to store insn sectors */,
		num_bps);
	image_close(&run.image.image);
	destroy_mem_param(&mp[0]);
	destroy_mem_param(&mp[1]);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to run flasher stub (%d)!", ret);
		return ret;
	}
	if (run.ret_code != ESP_STUB_ERR_OK) {
		LOG_ERROR("Failed to clear bp (%" PRId32 ")!", run.ret_code);
		return ERROR_FAIL;
	}

	for (size_t slot = 0; slot < num_bps; ++slot) {
		sw_bp[slot].oocd_bp = NULL;
		sw_bp[slot].insn_sz = 0;
		sw_bp[slot].bp_address = 0;
		sw_bp[slot].bp_flash_addr = 0;
		sw_bp[slot].action = ESP_BP_ACT_REM;
		sw_bp[slot].status = ESP_BP_STAT_DONE;
	}

	return ret;
}

static int esp_algo_flash_calc_hash(struct flash_bank *bank, uint8_t *hash,
	uint32_t offset, uint32_t count, bool verbose)
{
	struct esp_flash_bank *esp_info = bank->driver_priv;
	struct esp_algorithm_run_data run;
	const struct esp_flasher_stub_config *stub_cfg = esp_info->get_stub(bank, ESP_STUB_CMD_FLASH_CALC_HASH);
	const uint32_t stack_size = esp_info->stub_log_enabled ?
		stub_cfg->stack_default_sz * 2 : stub_cfg->stack_default_sz;

	if (offset & 0x3UL) {
		LOG_ERROR("Unaligned offset!");
		return ERROR_FAIL;
	}

	if (bank->target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int ret = esp_algo_flasher_algorithm_init(&run, esp_info->stub_hw, stub_cfg);
	if (ret != ERROR_OK)
		return ret;

	run.stack_size = stack_size + ESP_STUB_RDWR_BUFF_SIZE;

	struct mem_param mp;
	init_mem_param(&mp,
		3 /*2nd usr arg*/,
		32 /*sha256 hash size in bytes*/,
		PARAM_IN);
	run.mem_args.params = &mp;
	run.mem_args.count = 1;
	run.timeout_ms = ESP_FLASH_VERIFY_TMO;

	struct duration bench;
	duration_start(&bench);

	ret = esp_info->run_func_image(bank->target,
		&run,
		4 /*args num*/,
		ESP_STUB_CMD_FLASH_CALC_HASH /*cmd*/,
		esp_info->hw_flash_base + offset,
		count,
		0 /*address to store hash value*/);
	image_close(&run.image.image);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to run flasher stub (%d)!", ret);
		destroy_mem_param(&mp);
		return ret;
	}
	if (run.ret_code != ESP_STUB_ERR_OK) {
		LOG_ERROR("Failed to get hash value (%" PRId32 ")!", run.ret_code);
		ret = ERROR_FAIL;
	} else {
		memcpy(hash, mp.value, 32);
		duration_measure(&bench);
		if (verbose)
			LOG_INFO("PROF: Flash verified in %g ms ",
				duration_elapsed(&bench) * 1000);
	}
	destroy_mem_param(&mp);
	return ret;
}

static int esp_algo_flash_boost_clock_freq(struct flash_bank *bank, bool boost)
{
	struct esp_flash_bank *esp_info = bank->driver_priv;
	struct esp_algorithm_run_data run;
	int new_cpu_freq = -1;	/* set to max level */
	const struct esp_flasher_stub_config *stub_cfg = esp_info->get_stub(bank, ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE);
	const uint32_t stack_size = esp_info->stub_log_enabled ?
		stub_cfg->stack_default_sz * 2 : stub_cfg->stack_default_sz;

	int ret = esp_algo_flasher_algorithm_init(&run, esp_info->stub_hw, stub_cfg);
	if (ret != ERROR_OK)
		return ret;

	/* restore */
	if (!boost)
		new_cpu_freq = esp_info->old_cpu_freq;

	run.stack_size = stack_size;
	ret = esp_info->run_func_image(bank->target,
		&run,
		2,
		ESP_STUB_CMD_FLASH_CLOCK_CONFIGURE,
		new_cpu_freq);
	image_close(&run.image.image);
	if (ret != ERROR_OK) {
		LOG_ERROR("Failed to run flasher stub (%d)!", ret);
		return ERROR_FAIL;
	}
	esp_info->old_cpu_freq = (int)run.ret_code;
	LOG_DEBUG("%s old_freq (%d) new_freq (%d)",
		__func__,
		esp_info->old_cpu_freq,
		new_cpu_freq);
	return ERROR_OK;
}

static int esp_algo_target_to_flash_bank(struct target *target,
	struct flash_bank **bank, char *bank_name_suffix, bool probe)
{
	if (target == NULL || bank == NULL)
		return ERROR_FAIL;

	char bank_name[64];
	int retval = snprintf(bank_name,
		sizeof(bank_name),
		"%s.%s",
		target_name(target),
		bank_name_suffix);
	if (retval == sizeof(bank_name)) {
		LOG_ERROR("Failed to build bank name string!");
		return ERROR_FAIL;
	}
	if (probe) {
		retval = get_flash_bank_by_name(bank_name, bank);
		if (retval != ERROR_OK || bank == NULL) {
			LOG_ERROR("Failed to find bank '%s'!", bank_name);
			return retval;
		}
	} else {/* noprobe */
		*bank = get_flash_bank_by_name_noprobe(bank_name);
		if (*bank == NULL) {
			LOG_ERROR("Failed to find bank '%s'!", bank_name);
			return ERROR_FAIL;
		}
	}

	return ERROR_OK;
}

static int esp_algo_flash_appimage_base_update(struct target *target,
	char *bank_name_suffix,
	uint32_t appimage_flash_base)
{
	struct flash_bank *bank;
	struct esp_flash_bank *esp_info;

	int retval = esp_algo_target_to_flash_bank(target, &bank, bank_name_suffix, false);
	if (retval != ERROR_OK)
		return ERROR_FAIL;

	esp_info = (struct esp_flash_bank *)bank->driver_priv;
	esp_info->probed = 0;
	esp_info->appimage_flash_base = appimage_flash_base;
	retval = bank->driver->auto_probe(bank);
	return retval;
}

static COMMAND_HELPER(esp_algo_flash_cmd_appimage_flashoff_do, struct target *target)
{
	if (CMD_ARGC != 1) {
		command_print(CMD, "Flash offset not specified!");
		return ERROR_FAIL;
	}

	/* update app image base */
	uint32_t appimage_flash_base = strtoul(CMD_ARGV[0], NULL, 16);

	int ret = esp_algo_flash_appimage_base_update(target, "irom", appimage_flash_base);
	if (ret != ERROR_OK)
		return ret;
	ret = esp_algo_flash_appimage_base_update(target, "drom", appimage_flash_base);
	if (ret != ERROR_OK)
		return ret;

	return esp_algo_flash_appimage_base_update(target, "flash", appimage_flash_base);
}

static int esp_algo_flash_set_compression(struct target *target,
	char *bank_name_suffix,
	bool compression)
{
	struct flash_bank *bank;
	struct esp_flash_bank *esp_info;

	int retval = esp_algo_target_to_flash_bank(target, &bank, bank_name_suffix, false);
	if (retval != ERROR_OK)
		return ERROR_FAIL;

	esp_info = (struct esp_flash_bank *)bank->driver_priv;
	esp_info->compression = compression;

#if !BUILD_ESP_COMPRESSION
	if (esp_info->compression) {
		LOG_ERROR("ESP compression option disabled in the configuration");
		return ERROR_FAIL;
	}
#endif
	return ERROR_OK;
}

static COMMAND_HELPER(esp_algo_flash_cmd_set_compression, struct target *target)
{
	if (CMD_ARGC != 1) {
		command_print(CMD, "Compression not specified!");
		return ERROR_FAIL;
	}

	bool compression = false;
	COMMAND_PARSE_BOOL(CMD_ARGV[0], compression, "on", "off");
	LOG_DEBUG("Flash compressed upload is %s", compression ? "on" : "off");

	return esp_algo_flash_set_compression(target, "flash", compression);
}

static int esp_algo_flash_set_encryption(struct target *target,
	char *bank_name_suffix,
	int encryption)
{
	struct flash_bank *bank;
	int retval = esp_algo_target_to_flash_bank(target, &bank, bank_name_suffix, false);
	if (retval != ERROR_OK)
		return ERROR_FAIL;

	struct esp_flash_bank *esp_info = (struct esp_flash_bank *)bank->driver_priv;
	esp_info->encryption_needed_on_chip = encryption;
	return ERROR_OK;
}

static int esp_algo_flash_set_stub_log(struct target *target, char *bank_name_suffix, bool log_stat)
{
	struct flash_bank *bank;
	int retval = esp_algo_target_to_flash_bank(target, &bank, bank_name_suffix, false);
	if (retval != ERROR_OK)
		return ERROR_FAIL;

	struct esp_flash_bank *esp_info = (struct esp_flash_bank *)bank->driver_priv;
	esp_info->stub_log_enabled = log_stat;
	return ERROR_OK;
}

static COMMAND_HELPER(esp_algo_flash_cmd_set_encryption, struct target *target)
{
	if (CMD_ARGC != 1) {
		command_print(CMD, "Encryption not specified!");
		return ERROR_FAIL;
	}

	bool encryption = false;
	COMMAND_PARSE_ON_OFF(CMD_ARGV[0], encryption);

	return esp_algo_flash_set_encryption(target, "flash", encryption);
}

static int esp_flash_verify_bank_hash(struct target *target,
	uint32_t offset,
	const char *file_name,
	bool verbose)
{
	uint8_t file_hash[TC_SHA256_DIGEST_SIZE], target_hash[TC_SHA256_DIGEST_SIZE];
	uint8_t *buffer_file;
	struct fileio *fileio;
	size_t filesize, length, read_cnt;
	int differ, retval;
	struct flash_bank *bank;

	retval = esp_algo_target_to_flash_bank(target, &bank, "flash", true);
	if (retval != ERROR_OK)
		return ERROR_FAIL;

	if (offset > bank->size) {
		LOG_ERROR("Offset 0x%8.8" PRIx32 " is out of range of the flash bank",
			offset);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	retval = fileio_open(&fileio, file_name, FILEIO_READ, FILEIO_BINARY);
	if (retval != ERROR_OK) {
		LOG_ERROR("Could not open file");
		return retval;
	}

	retval = fileio_size(fileio, &filesize);
	if (retval != ERROR_OK) {
		fileio_close(fileio);
		return retval;
	}

	length = MIN(filesize, bank->size - offset);

	if (!length) {
		LOG_INFO("Nothing to compare with flash bank");
		fileio_close(fileio);
		return ERROR_OK;
	}

	if (length != filesize)
		LOG_WARNING("File content exceeds flash bank size. Only comparing the "
			"first %zu bytes of the file", length);

	LOG_DEBUG("File size: %zu bank_size: %u offset: %u",
		filesize, bank->size, offset);

	buffer_file = malloc(length);
	if (buffer_file == NULL) {
		LOG_ERROR("Out of memory");
		fileio_close(fileio);
		return ERROR_FAIL;
	}

	retval = fileio_read(fileio, length, buffer_file, &read_cnt);
	fileio_close(fileio);
	if (retval != ERROR_OK || read_cnt != length) {
		LOG_ERROR("File read failure");
		free(buffer_file);
		return retval;
	}

	retval = esp_algo_calc_hash(buffer_file, length, file_hash);
	free(buffer_file);
	if (retval != ERROR_OK) {
		LOG_ERROR("File sha256 calculation failure");
		return retval;
	}

	retval = esp_algo_flash_calc_hash(bank, target_hash, offset, length, verbose);
	if (retval != ERROR_OK) {
		LOG_ERROR("Flash sha256 calculation failure");
		return retval;
	}

	differ = memcmp(file_hash, target_hash, TC_SHA256_DIGEST_SIZE);

	if (differ && verbose) {
		LOG_ERROR("**** Verification failure! ****");
		LOG_ERROR("target_hash %x%x%x...%x%x%x",
			target_hash[0], target_hash[1], target_hash[2],
			target_hash[29], target_hash[30], target_hash[31]);
		LOG_ERROR("file_hash: %x%x%x...%x%x%x",
			file_hash[0], file_hash[1], file_hash[2],
			file_hash[29], file_hash[30], file_hash[31]);
	}

	return differ ? ERROR_FAIL : ERROR_OK;
}

static COMMAND_HELPER(esp_algo_flash_parse_cmd_verify_bank_hash, struct target *target)
{
	if (CMD_ARGC < 2 || CMD_ARGC > 4)
		return ERROR_COMMAND_SYNTAX_ERROR;

	uint32_t offset = 0;
	bool verbose = true;

	for (unsigned int i = 2; i < CMD_ARGC; ++i) {
		if (strcmp("quiet", CMD_ARGV[i]) == 0)
			verbose = false;
		else
			COMMAND_PARSE_NUMBER(u32, CMD_ARGV[i], offset);
	}

	return esp_flash_verify_bank_hash(target, offset, CMD_ARGV[1], verbose);
}

static COMMAND_HELPER(esp_algo_flash_parse_cmd_clock_boost, struct target *target)
{
	if (CMD_ARGC != 1) {
		command_print(CMD, "Clock boost flag not specified!");
		return ERROR_FAIL;
	}

	bool boost = 0;
	COMMAND_PARSE_BOOL(CMD_ARGV[0], boost, "on", "off");
	LOG_DEBUG("Clock boost is %s", boost ? "on" : "off");

	struct flash_bank *bank;
	int retval = esp_algo_target_to_flash_bank(target, &bank, "flash", true);
	if (retval != ERROR_OK)
		return ERROR_FAIL;

	return esp_algo_flash_boost_clock_freq(bank, boost);
}

static COMMAND_HELPER(esp_algo_flash_parse_cmd_stub_log, struct target *target)
{
	if (CMD_ARGC != 1) {
		LOG_TARGET_ERROR(target, "Stub log flag not specified!");
		return ERROR_FAIL;
	}

	bool log_stat = false;
	COMMAND_PARSE_BOOL(CMD_ARGV[0], log_stat, "on", "off");
	LOG_TARGET_INFO(target, "Stub logs %s!", log_stat ? "enabled" : "disabled");

	int ret = esp_algo_flash_set_stub_log(target, "irom", log_stat);
	if (ret != ERROR_OK)
		return ret;
	ret = esp_algo_flash_set_stub_log(target, "drom", log_stat);
	if (ret != ERROR_OK)
		return ret;

	return esp_algo_flash_set_stub_log(target, "flash", log_stat);
}

COMMAND_HANDLER(esp_algo_flash_cmd_clock_boost)
{
	return CALL_COMMAND_HANDLER(esp_algo_flash_parse_cmd_clock_boost, get_current_target(CMD_CTX));
}

COMMAND_HANDLER(esp_algo_flash_cmd_verify_bank_hash)
{
	return CALL_COMMAND_HANDLER(esp_algo_flash_parse_cmd_verify_bank_hash, get_current_target(CMD_CTX));
}

#define COMMAND_HANDLER_SMP(name, handler) \
COMMAND_HANDLER(name) \
{ \
	struct target *target = get_current_target(CMD_CTX); \
	if (target->smp) { \
		struct target_list *head; \
		foreach_smp_target(head, target->smp_targets) { \
			int ret = CALL_COMMAND_HANDLER(handler, head->target); \
			if (ret != ERROR_OK) \
				return ret; \
		} \
		return ERROR_OK; \
	} \
	return CALL_COMMAND_HANDLER(handler, target); \
}

COMMAND_HANDLER_SMP(esp_algo_flash_cmd_stub_log, esp_algo_flash_parse_cmd_stub_log)
COMMAND_HANDLER_SMP(esp_algo_flash_cmd_encryption, esp_algo_flash_cmd_set_encryption)
COMMAND_HANDLER_SMP(esp_algo_flash_cmd_compression, esp_algo_flash_cmd_set_compression)
COMMAND_HANDLER_SMP(esp_algo_flash_cmd_appimage_flashoff, esp_algo_flash_cmd_appimage_flashoff_do)

const struct command_registration esp_flash_exec_flash_command_handlers[] = {
	{
		.name = "appimage_offset",
		.handler = esp_algo_flash_cmd_appimage_flashoff,
		.mode = COMMAND_ANY,
		.help =
			"Set offset of application image in flash. Use -1 to debug the first application image from partition table.",
		.usage = "offset",
	},
	{
		.name = "compression",
		.handler = esp_algo_flash_cmd_compression,
		.mode = COMMAND_ANY,
		.help =
			"Set compression flag",
		.usage = "['on'|'off']",
	},
	{
		.name = "verify_bank_hash",
		.handler = esp_algo_flash_cmd_verify_bank_hash,
		.mode = COMMAND_ANY,
		.help = "Perform a comparison between the file and the contents of the "
			"flash bank using SHA256 hash values. Allow optional offset from beginning of the bank "
			"(defaults to zero).",
		.usage = "bank_id filename [offset]",
	},
	{
		.name = "flash_stub_clock_boost",
		.handler = esp_algo_flash_cmd_clock_boost,
		.mode = COMMAND_ANY,
		.help =
			"Set cpu clock freq to the max level. Use 'off' to restore the clock speed",
		.usage = "['on'|'off']",
	},
	{
		.name = "encrypt_binary",
		.handler = esp_algo_flash_cmd_encryption,
		.mode = COMMAND_ANY,
		.help =
			"Set if binary encryption needs to be handled on chip before writing to flash",
		.usage = "['yes'|'no']",
	},
	{
		.name = "stub_log",
		.handler = esp_algo_flash_cmd_stub_log,
		.mode = COMMAND_ANY,
		.help = "Enable stub flasher logs",
		.usage = "['on'|'off']",
	},
	COMMAND_REGISTRATION_DONE
};
