/***************************************************************************
 *                                                                         *
 *   Copyright (C) 2018 by Bohdan Tymkiv                                   *
 *   bohdan.tymkiv@infineon.com bohdan200@gmail.com                        *
 *                                                                         *
 *   Copyright (C) <2019-2021>                                             *
 *     <Cypress Semiconductor Corporation (an Infineon company)>           *
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
 *   along with this program.  If not, see <http://www.gnu.org/licenses/>. *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include "target/image.h"

enum cmsis_operation {
	CMSIS_OPERATION_INVALID = 0,
	CMSIS_OPERATION_ERASE = 1,
	CMSIS_OPERATION_PROGRAM = 2,
	CMSIS_OPERATION_VERIFY = 3,
};

static struct symbol cmsis_symbols[] = {
	{"Init", 0},
	{"UnInit", 0},
	{"EraseSector", 0},
	{"ProgramPage", 0},
	{"BlankCheck", 0},
	{"EraseChip", 0},
	{"Verify", 0},
	{"FlashDevice", 0},
	{"PrgData", 0},
	{NULL, 0},
};

struct cmsis_flash_sectors {
	uint32_t size;
	uint32_t addr;
};

struct cmsis_flash_dev {
	uint16_t vers;			/* Version Number and Architecture */
	uint8_t dev_name[128];	/* Device Name and Description */
	uint16_t dev_type;		/* Device Type: ONCHIP, EXT8BIT, EXT16BIT, ... */
	uint32_t dev_addr;		/* Default Device Start Address */
	uint32_t sz_dev;		/* Total Size of Device */
	uint32_t sz_page;		/* Programming Page Size */
	uint32_t _resvd;		/* Reserved for future Extension */
	uint8_t val_empty;		/* Content of Erased Memory */
	uint32_t timeout_prog;	/* Time Out of Program Page Function */
	uint32_t timeout_erase;	/* Time Out of Erase Sector Function */
	struct cmsis_flash_sectors sectors[512];
};

struct cmsis_flash {
	struct image image;
	struct cmsis_flash_dev flash_dev;
	struct working_area *algo_wa;
	enum cmsis_operation init_op;
	bool is_probed;
	bool is_loaded;
	bool prefer_sector_erase;
	uint32_t footprint;
	uint32_t stack_size;
	uint32_t of_init;
	uint32_t of_uninit;
	uint32_t of_erase_sector;
	uint32_t of_program_page;
	uint32_t of_blank_check;
	uint32_t of_erase_chip;
	uint32_t of_verify;
	uint32_t of_flash_device;
	uint32_t of_prg_data;
};

struct ram_params {
	uint32_t work_area;
	uint32_t fifo_end;
	uint32_t flash_addr;
	uint32_t num_pages;
	uint32_t page_size;
	uint32_t program_page_p;
};

const uint8_t program_page_wrapper[] = {
 	#include "../../../contrib/loaders/flash/cmsis_flash/program_page.inc"
};

/** ***********************************************************************************************
 * @brief Allocates working area for the algorithm, stack and breakpoint instruction and writes
 * contents of Flash Algorithm image to the target memory
 * @param algo pointer to the algorithm structure
 * @param target current target
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int cmsis_flash_load_algo(struct cmsis_flash *algo, struct target *target)
{
  LOG_DEBUG("---> cmsis_flash_load_algo");

	if (algo->is_loaded) {
		LOG_ERROR("CMSIS algorithm already loaded");
		return ERROR_FAIL;
	}

	const uint32_t wa_size = (algo->footprint + algo->stack_size + 0x03u) & ~0x03u;

	int hr = target_alloc_working_area(target, wa_size, &algo->algo_wa);
	if (hr != ERROR_OK)
		return hr;

	for (unsigned int i = 0; i < algo->image.num_sections; i++) {
		struct imagesection *section = &algo->image.sections[i];

		/* Skip 'DevDscr' section, usually it is not required */
		if (section->base_address == algo->of_flash_device)
			continue;

		uint8_t buffer[section->size];
		size_t read;

		hr = image_read_section(&algo->image, i, 0, section->size, buffer, &read);
		if (hr != ERROR_OK)
			goto free_wa;

		if (section->size != read) {
			hr = ERROR_FAIL;
			goto free_wa;
		}

		const target_addr_t tgt_buf = algo->algo_wa->address +
			section->base_address;

		hr = target_write_buffer(target, tgt_buf, section->size, buffer);
		if (hr != ERROR_OK)
			goto free_wa;
	}

	algo->is_loaded = true;
	return ERROR_OK;

free_wa:
	target_free_working_area(target, algo->algo_wa);
	algo->algo_wa = NULL;
	return hr;
}

/** ***********************************************************************************************
 * @brief Releases resources allocated by cmsis_algo_load
 * @param algo pointer to the algorithm structure
 * @param target current target
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int cmsis_flash_unload_algo(struct cmsis_flash *algo, struct target *target)
{
  LOG_DEBUG("---> cmsis_flash_unload_algo");
	if (!algo->is_loaded) {
		LOG_ERROR("CMSIS algorithm already unloaded");
		return ERROR_FAIL;
	}

	if (algo->algo_wa) {
		target_free_working_area(target, algo->algo_wa);
		algo->algo_wa = NULL;
	}

	algo->is_loaded = false;
	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Parses FlashDevice structure in algorithm image and returns total number of Flash sectors
 * @param algo pointer to the algorithm structure
 * @param num_sectors will be populated with total number of sectors
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int cmsis_flash_get_sector_num(struct cmsis_flash *algo, unsigned int *num_sectors)
{
	*num_sectors = 0;
	size_t idx = 0;
	const struct cmsis_flash_sectors *sect = algo->flash_dev.sectors;
	const struct cmsis_flash_dev *dev = &algo->flash_dev;

	while (sect[idx + 1].addr != UINT32_MAX) {
		if (idx >= 512 || sect[idx].size == 0) {
			LOG_ERROR("cmsis_flash: FlashDevice structure is invalid");
			return ERROR_IMAGE_FORMAT_ERROR;
		}

		*num_sectors += (sect[idx + 1].addr - sect[idx].addr) / sect[idx].size;
		idx++;
	}

	if (sect[idx].size == 0) {
		LOG_ERROR("cmsis_flash: FlashDevice structure is invalid");
		return ERROR_IMAGE_FORMAT_ERROR;
	}

	*num_sectors += (dev->sz_dev - sect[idx].addr) / sect[idx].size;
	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Parses FlashDevice structure in algorithm image and returns geometry of the given sector
 * @param algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param sector_idx index of the sector
 * @param offset will be populated with the offset of the sector within flash bank
 * @param size will be populated with the size of the sector
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int cmsis_flash_get_sector_info(struct cmsis_flash *algo, size_t sector_idx,
	uint32_t *sect_offset, uint32_t *sect_size)
{
	const struct cmsis_flash_sectors *sect = algo->flash_dev.sectors;
	uint32_t size = sect[0].size;
	uint32_t offset = 0;
	size_t fd_sec_idx = 0;

	for (size_t i = 0; i < sector_idx; i++) {

		offset += sect[fd_sec_idx].size;

		if (offset == sect[fd_sec_idx + 1].addr)
			fd_sec_idx++;

		size = sect[fd_sec_idx].size;

		if (sect[fd_sec_idx].size == UINT32_MAX || sect[fd_sec_idx].addr == UINT32_MAX ||
			offset + size > algo->flash_dev.sz_dev) {
			LOG_ERROR("cmsis_flash: FlashDevice structure is invalid");
			return ERROR_IMAGE_FORMAT_ERROR;
		}
	}

	*sect_offset = offset;
	*sect_size = size;
	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Executes FlashLoader function. Function takes variable number of arguments and stores
 * them in r0...r3 registers, address to PrgData section is stored in r9, SP is set to the end of
 * working area for the algorithm. Function also writes BKPT #0 instruction right after algorithm
 * code space and stores address of this instruction in lr.
 * @param algo pointer to the algorithm structure
 * @param target current target
 * @param fn_offset offset of the entry point of the algorithm (relative to the working area address)
 * @param timeout execution timeout, in ms
 * @param result value is populated with exit code of the algorithm (value in r0 register)
 * @param argc number of arguments followed by variable number of arguments (values for r0...r3)
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int cmsis_algo_execute(struct cmsis_flash *algo, struct target *target,
	uint32_t fn_offset, size_t timeout, uint32_t *result, size_t argc, ...)
{
	assert(argc <= 4);
  LOG_DEBUG("---> cmsis_algo_execute");

	/* TODO: only ARMv6M/v7M are currently supported. I have no experience with other ARM architectures
	 * so I can not make this code completely generic. This will be fixed in the future */
	if (!is_armv7m(target_to_armv7m(target))) {
		LOG_ERROR("cmsis algorithm: only armv7m targets are supported");
		return ERROR_TARGET_INVALID;
	}

	static char *param_regs[4] = { "r0", "r1", "r2", "r3" };
	struct reg_param regs[7];

	struct armv7m_algorithm armv7m;
	armv7m.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m.core_mode = ARM_MODE_THREAD;
	const struct working_area *wa = algo->algo_wa;
	int hr;

	size_t param_cnt = 0;
	va_list args;

	va_start(args, argc);
	if (argc) {
		/* If number of arguments is greater than zero r0 should be PARAM_IN_OUT */
		for (size_t i = 0; i < argc; i++) {
			/* loop through arguments and populate r0...r3 registers */
			enum param_direction dir = (i == 0) ? PARAM_IN_OUT : PARAM_OUT;
			init_reg_param(&regs[i], param_regs[i], 32, dir);
			buf_set_u32(regs[i].value, 0, 32,  va_arg(args, uint32_t));
			param_cnt++;
		}
	} else {
		/* No arguments have been supplied, r0 is PARAM_IN in this case */
		init_reg_param(&regs[0], param_regs[0], 32, PARAM_IN);
		buf_set_u32(regs[0].value, 0, 32,  0);
		param_cnt++;
	}
	va_end(args);

	/* Calculate address of the 'BKPT #0' instruction */
	uint32_t bkpt_adddr = (wa->address + algo->footprint + 0x03u) & ~0x03u;

	/* Populate all common registers */
	init_reg_param(&regs[param_cnt + 0], "r9", 32, PARAM_OUT);
	init_reg_param(&regs[param_cnt + 1], "sp", 32, PARAM_OUT);
	init_reg_param(&regs[param_cnt + 2], "lr", 32, PARAM_OUT);

	buf_set_u32(regs[param_cnt + 0].value, 0, 32, wa->address + algo->of_prg_data);
  buf_set_u32(regs[param_cnt + 1].value, 0, 32, wa->address + wa->size);
	buf_set_u32(regs[param_cnt + 2].value, 0, 32, bkpt_adddr | 0x01);
  
  LOG_DEBUG("writing SP with value 0x%08x", 
    buf_get_u32(regs[param_cnt + 1].value, 0, 32));
  LOG_DEBUG("writing LR with value 0x%08x", 
    buf_get_u32(regs[param_cnt + 2].value, 0, 32));
	
	param_cnt += 3;

	/* Write 'BKPT #0' instruction */
	hr = target_write_u32(target, bkpt_adddr, 0xBE00BE00);
  if (hr != ERROR_OK)
		goto cleanup;

	/* Operation can take a while, send keep_alive packet here */
	keep_alive();

	/* Start the algorithm */
	hr = target_run_algorithm(target, 0, NULL, param_cnt, regs,
			wa->address + fn_offset, 0, timeout, &armv7m);
	if (hr != ERROR_OK)
		goto cleanup;

	/* Retrieve algorithm exit code */
	*result = buf_get_u32(regs[0].value, 0, 32);
  LOG_DEBUG("---> cmsis_algo_execute return code %d", *result);

cleanup:
	for (size_t i = 0; i < param_cnt; i++)
		destroy_reg_param(&regs[i]);

	return hr;
}

/** ***********************************************************************************************
 * @brief Calls Init() function of the CMSIS FlashLoader
 * @param algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param target current target
 * @param address base address of the next operation
 * @param clock clock speed for the target device
 * @param op type of next operation
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int cmsis_algo_init(struct cmsis_flash *algo, struct target *target, uint32_t address,
	uint32_t clock, enum cmsis_operation op)
{
  LOG_DEBUG("---> cmsis_algo_init");
	if (algo->init_op != CMSIS_OPERATION_INVALID) {
		LOG_ERROR("CMSIS algorithm already initialized");
		return ERROR_FAIL;
	}

	uint32_t result = 0;
	int hr = cmsis_algo_execute(algo, target, algo->of_init, algo->flash_dev.timeout_prog,
			&result, 3, address, clock, op);

	if (hr != ERROR_OK)
		return hr;

	if (result) {
		LOG_ERROR("cmsis algorithm: Init operation failed");
		return ERROR_FAIL;
	}

	algo->init_op = op;
	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Calls UnInit() function of the CMSIS-PACK FlashLoader
 * @param algo pointer to the algorithm structure
 * @param target current target
 * @param op type of next operation
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int cmsis_algo_uninit(struct cmsis_flash *algo, struct target *target, uint32_t op)
{
  LOG_DEBUG("---> cmsis_algo_uinit");

	if (algo->init_op == CMSIS_OPERATION_INVALID) {
		LOG_ERROR("CMSIS algorithm already uninitialized");
		return ERROR_FAIL;
	}

	if (algo->init_op != op) {
		LOG_ERROR("CMSIS algorithm Init/UnInit operation mismatch");
		return ERROR_FAIL;
	}

	uint32_t result = 0;
	int hr = cmsis_algo_execute(algo, target, algo->of_uninit, algo->flash_dev.timeout_prog,
			&result, 1, op);

	if (hr != ERROR_OK)
		return hr;

	if (result) {
		LOG_ERROR("cmsis algorithm: UnInit operation failed");
		return ERROR_FAIL;
	}

	algo->init_op = CMSIS_OPERATION_INVALID;
	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Calls BlankCheck() function of the CMSIS-PACK FlashLoader
 * @param algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param target current target
 * @param addr start address of the block to check for erased state
 * @param size size of the block to check for erased state
 * @param is_erased erasure status: 0 = not erased, 1 = erased, other = unknown
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int cmsis_algo_blank_check(struct cmsis_flash *algo, struct target *target, uint32_t addr, uint32_t size, int *is_erased)
{
  LOG_DEBUG("---> cmsis_algo_blank_check");

	uint32_t result = 0;
	int hr = cmsis_algo_execute(algo, target, algo->of_blank_check,
				algo->flash_dev.timeout_erase, &result, 3, addr, size,
				algo->flash_dev.val_empty);

	if (hr != ERROR_OK) {
		*is_erased = -1;
	} else {
		*is_erased = (result == 0) ? 1 : 0;
	}

	return hr;
}

/** ***********************************************************************************************
 * @brief Calls EraseSector() function of the CMSIS-PACK FlashLoader
 * @param algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param target current target
 * @param addr address of the sector to erase
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int cmsis_algo_erase_sector(struct cmsis_flash *algo, struct target *target, uint32_t addr)
{
  LOG_DEBUG("---> cmsis_algo_erase_sector");

	uint32_t result = 0;
	int hr = cmsis_algo_execute(algo, target, algo->of_erase_sector,
			algo->flash_dev.timeout_erase, &result, 1, addr);

	if (hr != ERROR_OK)
		return hr;

	if (result) {
		LOG_ERROR("cmsis algorithm: EraseSector operation failed");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Calls EraseChip() function of the CMSIS-PACK FlashLoader
 * @param algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param target current target
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int cmsis_algo_erase_chip(struct cmsis_flash *algo, struct target *target, uint32_t timeout)
{
  LOG_DEBUG("---> cmsis_algo_erase_chip");

	if (algo->of_erase_chip == UINT32_MAX)
		return ERROR_COMMAND_NOTFOUND;

	uint32_t result = 0;
	int hr = cmsis_algo_execute(algo, target, algo->of_erase_chip, timeout, &result, 0);

	if (hr != ERROR_OK)
		return hr;

	if (result) {
		LOG_ERROR("cmsis algorithm: EraseChip operation failed");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}


/** ***********************************************************************************************
 * @brief Calls ProgramPage() function of the CMSIS-PACK FlashLoader
 * @param algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @param target current target
 * @param addr address of the sector to erase
 * @param size size of the data to program
 * @param buf_addr adress of the buffer with data
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int cmsis_algo_program_page(struct cmsis_flash *algo, struct target *target, uint32_t addr,
	uint32_t size, uint32_t buf_addr)
{
  LOG_DEBUG("---> cmsis_algo_program_page");

	uint32_t result = 0;
  LOG_DEBUG("algo->flash_dev.timeout_prog is %d", algo->flash_dev.timeout_prog);
	int hr = cmsis_algo_execute(algo, target, algo->of_program_page,
			algo->flash_dev.timeout_prog, &result, 3, addr, size, buf_addr);

	if (hr != ERROR_OK)
		return hr;

	if (result) {
		LOG_ERROR("cmsis algorithm: ProgramPage operation failed");
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Loads algorithm to target memory and initializes it for given operation
 * @param bank current flash bank structure
 * @param op requested operation type
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int cmsis_flash_prepare_algo(struct flash_bank *bank, enum cmsis_operation op)
{
  LOG_DEBUG("---> cmsis_flash_prepare_algo");

	struct cmsis_flash *algo = bank->driver_priv;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int hr = cmsis_flash_load_algo(algo, target);
	if (hr != ERROR_OK)
		return hr;

	hr = cmsis_algo_init(algo, target, (uint32_t)bank->base, 0, op);
	if (hr != ERROR_OK)
		cmsis_flash_unload_algo(algo, target);

	return hr;
}

/** ***********************************************************************************************
 * @brief Uninitializes and unloads algorithm
 * @param bank current flash bank structure
 *************************************************************************************************/
static void cmsis_flash_release_algo(struct flash_bank *bank)
{
  LOG_DEBUG("---> cmsis_flash_release_algo");

	struct cmsis_flash *algo = bank->driver_priv;
	struct target *target = bank->target;

	cmsis_algo_uninit(algo, target, algo->init_op);

  /***** The following does not work for STM32F4 and STM32G0 *****/
	/* Hack: Reinitialize algo to make sure memory is mapped for read/verify/mdw */
	//cmsis_algo_init(algo, target, (uint32_t)bank->base, 0, CMSIS_OPERATION_VERIFY);
	//algo->init_op = CMSIS_OPERATION_INVALID; 

	cmsis_flash_unload_algo(algo, target);
}

/** ***********************************************************************************************
 * @brief Populates flash_bank structure with values stored in FlashDevice structure of the Flash Algorithm
 * @param bank flash_bank structure to populate
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int cmsis_flash_probe(struct flash_bank *bank)
{
  LOG_DEBUG("---> cmsis_flash_probe");

	int hr;
	struct cmsis_flash *algo = bank->driver_priv;

	uint32_t base = algo->flash_dev.dev_addr;
	uint32_t size = algo->flash_dev.sz_dev;
	bank->erased_value = algo->flash_dev.val_empty;

	if (!bank->base)
		bank->base = base;

	if (!bank->size)
		bank->size = size;

	hr = cmsis_flash_get_sector_num(algo, &bank->num_sectors);
	if (hr != ERROR_OK)
		return hr;

	bank->sectors = calloc(bank->num_sectors, sizeof(struct flash_sector));
	if (bank->sectors == NULL) {
		LOG_ERROR("cmsis_flash: out of memory");
		return ERROR_FAIL;
	}

	unsigned int real_sec_num = 0;
	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		hr = cmsis_flash_get_sector_info(algo, i, &bank->sectors[i].offset,
				&bank->sectors[i].size);

		if (hr != ERROR_OK)
			goto error_free_sectors;

		if (bank->sectors[i].offset >= bank->size)
			break;

		real_sec_num++;
		bank->sectors[i].is_erased = -1;
		bank->sectors[i].is_protected = -1;
	}

	if (bank->num_sectors != real_sec_num) {
		bank->num_sectors = real_sec_num;
		bank->sectors = realloc(bank->sectors, real_sec_num * sizeof(struct flash_sector));
	}

	bank->write_start_alignment = algo->flash_dev.sz_page;
	bank->write_end_alignment = algo->flash_dev.sz_page;
	bank->minimal_write_gap = FLASH_WRITE_GAP_SECTOR;
	bank->erased_value = algo->flash_dev.val_empty;
	bank->default_padded_value = algo->flash_dev.val_empty;
	bank->bus_width = 4;
	bank->chip_width = 4;

	algo->is_probed = true;
	return ERROR_OK;

error_free_sectors:
	free(bank->sectors);
	bank->sectors = NULL;
	bank->num_sectors = 0;
	return hr;
}

/** ***********************************************************************************************
 * @brief Probes target device only if it hasn't been probed yet
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int cmsis_flash_auto_probe(struct flash_bank *bank)
{
	struct cmsis_flash *algo = bank->driver_priv;

	return algo->is_probed ? ERROR_OK : cmsis_flash_probe(bank);
}

/** ***********************************************************************************************
 * Provides erased-bank check handling.
 * Uses BlankCheck function in flash loader if exists, otherwise - default_flash_blank_check
 * @param bank current flash bank
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int cmsis_flash_blank_check(struct flash_bank *bank)
{
	struct cmsis_flash *algo = bank->driver_priv;
	struct target *target = bank->target;
	int is_erased = 0;
	int hr;

	/* If optional BlankCheck function is not defined in CMSIS flash algorithms,
	 * use default one from core.c */
	if (algo->of_blank_check == UINT32_MAX) {
		return default_flash_blank_check(bank);
	}

	hr = cmsis_flash_prepare_algo(bank, CMSIS_OPERATION_VERIFY);
	if (hr != ERROR_OK)
		return hr;

	//progress_init(bank->num_sectors, BLANKCHECK);

	for (unsigned int i = 0; i < bank->num_sectors; i++) {
		hr = cmsis_algo_blank_check(algo, target,
				bank->base + bank->sectors[i].offset,
				 bank->sectors[i].size, &is_erased);
		bank->sectors[i].is_erased = is_erased;

		if (hr != ERROR_OK)
			goto release;

		//progress_sofar(i);
	}

release:
	//progress_done(hr);
	cmsis_flash_release_algo(bank);
	return hr;
}

/** ***********************************************************************************************
 * @brief Dummy function, protect check operation is not supported by this driver
 * @return ERROR_OK always
 *************************************************************************************************/
static int cmsis_flash_protect_check(struct flash_bank *bank)
{
	for (unsigned int i = 0; i < bank->num_sectors; i++)
		bank->sectors[i].is_protected = false;

	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Dummy function, protect operation is not supported by this driver
 * @return ERROR_FAIL always
 *************************************************************************************************/
static int cmsis_flash_protect(struct flash_bank *bank, int set, unsigned int first, unsigned int last)
{
	(void)bank; (void)set; (void)first; (void)last;

	LOG_ERROR("Flash protection is not supported by cmsis_flash driver");
	return ERROR_FAIL;
}

/** ***********************************************************************************************
 * @brief Performs erasure of given sector range
 * @param bank current flash bank
 * @param first first sector to erase
 * @param last last sector to erase
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int cmsis_flash_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
  LOG_DEBUG("---> cmsis_flash_erase");
  
	struct cmsis_flash *algo = bank->driver_priv;
	struct target *target = bank->target;
	int hr;

	hr = cmsis_flash_prepare_algo(bank, CMSIS_OPERATION_ERASE);
	if (hr != ERROR_OK)
		return hr;

	//progress_init(last - first + 1, ERASING);

	if (bank->num_sectors == last - first + 1 && algo->of_erase_chip != UINT32_MAX
			&& !algo->prefer_sector_erase) {
		LOG_INFO("Using EraseChip API to erase '%s' bank.", bank->name);
		hr = cmsis_algo_erase_chip(algo, target, bank->num_sectors * algo->flash_dev.timeout_erase);
		goto release;
	}

	for (unsigned int i = first; i <= last; i++) {
		hr = cmsis_algo_erase_sector(algo, target, bank->base + bank->sectors[i].offset);
		if (hr != ERROR_OK)
			goto release;

		//progress_left(last - i);
	}

release:
	//progress_done(hr);
	cmsis_flash_release_algo(bank);
	return hr;
}

/** ***********************************************************************************************
 * @brief Performs Program operation
 * @param bank current flash bank
 * @param buffer pointer to the buffer with data
 * @param offset starting offset in falsh bank
 * @param count number of bytes in buffer
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int cmsis_flash_program_slow(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset,
	uint32_t count)
{
  LOG_DEBUG("---> cmsis_flash_program_slow");
	struct cmsis_flash *algo = bank->driver_priv;
	struct target *target = bank->target;
	int hr;

	struct working_area *wa;
	hr = target_alloc_working_area(target, algo->flash_dev.sz_page, &wa);
	if (hr != ERROR_OK)
		return hr;

	hr = cmsis_flash_prepare_algo(bank, CMSIS_OPERATION_PROGRAM);
	if (hr != ERROR_OK)
		goto exit_free_wa;

	//progress_init(count / algo->flash_dev.sz_page, PROGRAMMING);

	for (size_t i = 0; i < count / algo->flash_dev.sz_page; i++) {
		hr = target_write_buffer(target, wa->address, algo->flash_dev.sz_page, buffer);
		if (hr != ERROR_OK)
			goto cleanup;

		const uint32_t page_addr = bank->base + offset + i * algo->flash_dev.sz_page;

		hr = cmsis_algo_program_page(algo, target, page_addr, algo->flash_dev.sz_page,
				wa->address);
		if (hr != ERROR_OK)
			goto cleanup;

		//progress_sofar(i + 1);
		buffer += algo->flash_dev.sz_page;
	}

cleanup:
	cmsis_flash_release_algo(bank);

exit_free_wa:
	target_free_working_area(target, wa);
	//progress_done(hr);
	return hr;
}

/** ***********************************************************************************************
 * @brief Performs Program operation
 * @param bank current flash bank
 * @param buffer pointer to the buffer with data
 * @param offset starting offset in falsh bank
 * @param count number of bytes in buffer
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
#define ENABLE_ASYNC_ALGO 1
static int cmsis_flash_program(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset,
	uint32_t count)
{
  LOG_DEBUG("---> cmsis_flash_program");

	struct cmsis_flash *algo = bank->driver_priv;
	struct target *target = bank->target;
#if ENABLE_ASYNC_ALGO
	int hr = cmsis_flash_prepare_algo(bank, CMSIS_OPERATION_PROGRAM);
	if (hr != ERROR_OK)
		return hr;

	struct working_area *wa_wrapper;

	hr = target_alloc_working_area(target, sizeof(program_page_wrapper), &wa_wrapper);
	if (hr != ERROR_OK)
		goto exit_release_algo;

	hr = target_write_buffer(target, wa_wrapper->address, sizeof(program_page_wrapper),
			program_page_wrapper);
	if (hr != ERROR_OK)
		goto exit_free_wa_wrapper;

	struct working_area *wa_params;
	hr = target_alloc_working_area(target, sizeof(struct ram_params), &wa_params);
	if (hr != ERROR_OK)
		goto exit_free_wa_wrapper;

	/* Try to allocate as large RAM Buffer as possible starting form 16 Page Buffers*/
	struct working_area *wa_buffer;
	uint32_t buffer_size = 16 * algo->flash_dev.sz_page;
	while (target_alloc_working_area_try(target, buffer_size + 8, &wa_buffer) != ERROR_OK) {
		buffer_size -= algo->flash_dev.sz_page;
		if (buffer_size < 3 * algo->flash_dev.sz_page) {
			LOG_WARNING("Failed to allocate circular buffer, will use slow algorithm");

			target_free_working_area(target, wa_params);
			target_free_working_area(target, wa_wrapper);
			cmsis_flash_release_algo(bank);
#endif
			return cmsis_flash_program_slow(bank, buffer, offset, count);
#if ENABLE_ASYNC_ALGO
		}
	}
	LOG_DEBUG("Allocated %u bytes for circular buffer", buffer_size);

	struct ram_params rp;
	rp.work_area = wa_buffer->address;
	rp.fifo_end = wa_buffer->address + wa_buffer->size;
	rp.flash_addr = bank->base + offset;
	rp.num_pages = count / algo->flash_dev.sz_page;
	rp.page_size = algo->flash_dev.sz_page;
	rp.program_page_p = algo->algo_wa->address + algo->of_program_page;

	hr = target_write_buffer(target, wa_params->address, sizeof(struct ram_params),
			(uint8_t *)&rp);
	if (hr != ERROR_OK)
		goto exit_free_wa_buffer;

	struct armv7m_algorithm armv7m_algo;
	armv7m_algo.common_magic = ARMV7M_COMMON_MAGIC;
	armv7m_algo.core_mode = ARM_MODE_THREAD;

	struct reg_param reg_params[3];
	init_reg_param(&reg_params[0], "r0", 32, PARAM_IN_OUT);
	init_reg_param(&reg_params[1], "r9", 32, PARAM_OUT);
	init_reg_param(&reg_params[2], "sp", 32, PARAM_OUT);

	buf_set_u32(reg_params[0].value, 0, 32, wa_params->address);
	buf_set_u32(reg_params[1].value, 0, 32, algo->algo_wa->address + algo->of_prg_data);
	buf_set_u32(reg_params[2].value, 0, 32, algo->algo_wa->address + algo->algo_wa->size);

	hr = target_run_flash_async_algorithm(target, buffer, rp.num_pages, rp.page_size,
			0, NULL, ARRAY_SIZE(reg_params), reg_params,
			wa_buffer->address, wa_buffer->size,
			wa_wrapper->address, 0, &armv7m_algo);

	destroy_reg_param(&reg_params[0]);
	destroy_reg_param(&reg_params[1]);
	destroy_reg_param(&reg_params[2]);

exit_free_wa_buffer:
	target_free_working_area(target, wa_buffer);
	target_free_working_area(target, wa_params);
exit_free_wa_wrapper:
	target_free_working_area(target, wa_wrapper);
exit_release_algo:
	cmsis_flash_release_algo(bank);
	return hr;
#endif  
}

static int cmsis_flash_verify(struct flash_bank *bank, const uint8_t *buffer,
	uint32_t offset, uint32_t count)
{
	struct cmsis_flash *algo = bank->driver_priv;
	struct target *target = bank->target;
	int hr;

  LOG_DEBUG("offset=0x%08" PRIx32 " count=%" PRId32, offset, count);

  if (algo->of_verify == 0xFFFFFFFF) {
    LOG_DEBUG("verify() is not implemented");
    return ERROR_NOT_IMPLEMENTED;
  }
  LOG_WARNING("Using custom verify function");

	hr = cmsis_flash_prepare_algo(bank, CMSIS_OPERATION_VERIFY);
	if (hr != ERROR_OK)
		goto cleanup;

  /* Try to allocate as large RAM buffer as possible starting from 16 page buffers */
	struct working_area *wa_buffer;
  uint32_t buffer_size = 16 * algo->flash_dev.sz_page;
  while (target_alloc_working_area_try(target, buffer_size + 8, &wa_buffer) != ERROR_OK) {
		buffer_size -= algo->flash_dev.sz_page;
		if (buffer_size < algo->flash_dev.sz_page) {
			LOG_ERROR("Failed to allocate working buffer");
      goto cleanup;
    }
  }

  /* some sensible timeout in ms */
  const uint32_t timeout = 100*(buffer_size/algo->flash_dev.sz_page);

  size_t written = 0;
	for (size_t i = 0; i < (count + buffer_size/2) / buffer_size; i++) {
    size_t write_size = buffer_size;
    if (count - written < buffer_size) {
      write_size = count - written;
    }
 		hr = target_write_buffer(target, wa_buffer->address, write_size, buffer);
		if (hr != ERROR_OK)
			goto exit_free_wa;

		const uint32_t addr = bank->base + offset + i * buffer_size;
    uint32_t result = 0;
    LOG_DEBUG("verify %" PRId32 " bytes at base address 0x%08" PRIx32, write_size, addr);
		hr = cmsis_algo_execute(algo, target, algo->of_verify, timeout, &result, 3, addr, write_size, wa_buffer->address);
		if (hr != ERROR_OK)
			goto exit_free_wa;
    if (result != addr + write_size) {
      LOG_ERROR("verify fail at 0x%08" PRIx32, result);
      goto exit_free_wa;
    }
    LOG_INFO("verified %" PRId32 " bytes at base address 0x%08" PRIx32 " ok", write_size, addr);
    written += write_size;
		buffer += write_size;
	}

exit_free_wa:
	target_free_working_area(target, wa_buffer);

cleanup:
	cmsis_flash_release_algo(bank);

	return hr;

}

/** ***********************************************************************************************
 * @brief Displays human-readable information about flash device
 * @param bank current flash bank
 * @param buf pointer to buffer for human-readable text
 * @param buf_size size of the buffer
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static int cmsis_flash_get_info(struct flash_bank *bank, struct command_invocation *cmd)
{
	struct cmsis_flash *algo = bank->driver_priv;

	if (algo->is_probed == false)
		return ERROR_FAIL;

	command_print_sameline(cmd, "CMSIS Flash Device: %s", algo->flash_dev.dev_name);
	return ERROR_OK;
}

/** ***********************************************************************************************
 * @brief Releases resurces allocated by cmsis_algo_open
 * @param cmsis_algo pointer to the algorithm structure returned by cmsis_algo_open()
 * @return ERROR_OK in case of success, ERROR_XXX code otherwise
 *************************************************************************************************/
static void cmsis_flash_free_driver_priv(struct flash_bank *bank)
{
	struct cmsis_flash *algo = bank->driver_priv;

	if (algo) {
		image_close(&algo->image);
		free(algo);
		algo = NULL;
	}
}

/* flash bank <name> cmsis_flash <addr> <size> 0 0 <target> <algorithm_elf> <stack_size> [prefer_sector_erase]*/
FLASH_BANK_COMMAND_HANDLER(cmsis_flash_bank_command)
{
	if (CMD_ARGC < 8 || CMD_ARGC > 9) {
		LOG_ERROR("cmsis_flash: wrong number of params");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	const char *algo_url = CMD_ARGV[6];
	uint32_t stack_size = strtol(CMD_ARGV[7], NULL, 0);

	if (!stack_size) {
		LOG_ERROR("cmsis_flash: invalid stack size");
		return ERROR_FAIL;
	}

	struct cmsis_flash *algo = calloc(1, sizeof(struct cmsis_flash));
	if (!algo) {
		LOG_ERROR("cmsis_flash: out of memory");
		return ERROR_FAIL;
	}

	algo->stack_size = stack_size;

	if(CMD_ARGC == 9) {
		if(strcmp(CMD_ARGV[8], "prefer_sector_erase") == 0) {
			algo->prefer_sector_erase = true;
		} else {
			LOG_ERROR("cmsis_flash: unknown parameter '%s'", CMD_ARGV[8]);
			free(algo);
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
	}

	/* Open Flash Loader image */
	int hr = image_open(&algo->image, algo_url, "elf");
	if (hr != ERROR_OK)
		goto free_algo;

	/* Initialize all symbol offsets to some invalid value */
	for (size_t i = 0; i < sizeof(cmsis_symbols) / sizeof(cmsis_symbols[0]); i++)
		cmsis_symbols[i].offset = UINT32_MAX;

	/* Resolve all required and optional symbols */
	hr = image_resolve_symbols(&algo->image, cmsis_symbols);
	if (hr != ERROR_OK)
		goto close_free_algo;

	algo->of_init         = cmsis_symbols[0].offset;
	algo->of_uninit       = cmsis_symbols[1].offset;
	algo->of_erase_sector = cmsis_symbols[2].offset;
	algo->of_program_page = cmsis_symbols[3].offset;
  algo->of_blank_check  = cmsis_symbols[4].offset;
  algo->of_erase_chip   = cmsis_symbols[5].offset;
  algo->of_verify       = cmsis_symbols[6].offset;
	algo->of_flash_device = cmsis_symbols[7].offset;
	algo->of_prg_data     = cmsis_symbols[8].offset;

  LOG_DEBUG("################################");
  LOG_DEBUG("init         = 0x%08x", algo->of_init);
  LOG_DEBUG("uninit       = 0x%08x", algo->of_uninit);
  LOG_DEBUG("erase_sector = 0x%08x", algo->of_erase_sector);
  LOG_DEBUG("program_page = 0x%08x", algo->of_program_page);
  LOG_DEBUG("blank_check  = 0x%08x", algo->of_blank_check);
  LOG_DEBUG("erase_chip   = 0x%08x", algo->of_erase_chip);
  LOG_DEBUG("verify       = 0x%08x", algo->of_verify);
  LOG_DEBUG("flash_device = 0x%08x", algo->of_flash_device);
  LOG_DEBUG("prg_data     = 0x%08x", algo->of_prg_data);
  LOG_DEBUG("################################");
  LOG_DEBUG("stack size is %d", algo->stack_size);
  
	/* Ensure all required symbols are resolved */
	if (algo->of_init == UINT32_MAX ||
		algo->of_uninit == UINT32_MAX ||
		algo->of_erase_sector == UINT32_MAX ||
		algo->of_program_page == UINT32_MAX ||
		algo->of_flash_device == UINT32_MAX ||
		algo->of_prg_data == UINT32_MAX) {
		LOG_ERROR("cmsis_flash: algorithm does not define all required symbols");
		hr = ERROR_IMAGE_FORMAT_ERROR;
		goto close_free_algo;
	}

	/* Locate elf section containing FlashDevice structure */
	size_t min_addr = UINT32_MAX;
	size_t max_addr = 0;

	bool flash_dev_found = false;
	for (unsigned int i = 0; i < algo->image.num_sections; i++) {
		struct imagesection *section = &algo->image.sections[i];

		if (section->base_address == algo->of_flash_device) {
			size_t read = 0;
			hr = image_read_section(&algo->image, i, 0, sizeof(struct cmsis_flash_dev),
					(uint8_t *)&algo->flash_dev, &read);

			if (hr != ERROR_OK)
				goto close_free_algo;

			if (read != sizeof(struct cmsis_flash_dev)) {
				hr = ERROR_FAIL;
				goto close_free_algo;
			}

			flash_dev_found = true;

			/* Skip 'DevDscr' section, usually it is not required */
			continue;
		}

		/* Calculate total footprint of the loadable sections */
		if (min_addr > section->base_address)
			min_addr = section->base_address;
		if (max_addr < section->base_address + section->size)
			max_addr = section->base_address + section->size;
	}

	if (!flash_dev_found) {
		LOG_ERROR("cmsis_flash: 'FlashDev' structure can not be found");
		hr = ERROR_IMAGE_FORMAT_ERROR;
		goto close_free_algo;
	} else {

		/* Calculate total footprint of the loadable sections */
		algo->footprint = max_addr - min_addr;
		LOG_INFO("Using CMSIS-flash algorithms '%s' for bank '%s' (footprint %d bytes)",
			algo->flash_dev.dev_name, bank->name, algo->footprint);

		/* Print basic flash loader information */
		LOG_INFO("CMSIS-flash: ELF path: %s", algo_url);
		LOG_INFO("CMSIS-flash: Address range:     0x%08X-0x%08X",
			algo->flash_dev.dev_addr, algo->flash_dev.dev_addr + algo->flash_dev.sz_dev -1);
		LOG_INFO("CMSIS-flash: Program page size: 0x%08X bytes", algo->flash_dev.sz_page);
		/* Validate the 'FlashDevice' structure and calculate total number of sectors */
		unsigned int num_sectors;
		hr = cmsis_flash_get_sector_num(algo, &num_sectors);
		if (hr == ERROR_OK) {

			/* Check how many 'FlashSectors' records (less than the number of sectors) */
			const struct cmsis_flash_sectors *sect = algo->flash_dev.sectors;
			uint32_t idx = 0;
			while ((idx < num_sectors) && (sect[idx].addr != UINT32_MAX)) idx++;
			num_sectors = idx;

			/* Print the erase sector description */
			if (num_sectors == 1) {
				LOG_INFO("CMSIS-flash: Erase sector size: 0x%08X bytes, unified", sect[0].size);
			} else for (idx = 0; idx < num_sectors; idx++) {
				LOG_INFO("CMSIS-flash: Erase sector [%u]: 0x%08X bytes @ 0x%08X", idx, sect[idx].size, sect[idx].addr);
			}
		}
	}

	bank->driver_priv = algo;
	return ERROR_OK;

close_free_algo:
	image_close(&algo->image);

free_algo:
	free(algo);
	return hr;
}

COMMAND_HANDLER(cmsis_flash_initialize)
{
  LOG_DEBUG("---> cmsis_flash_initiaize");
	const char *bank_name = NULL;

	if(CMD_ARGC > 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	if(CMD_ARGC)
		bank_name = CMD_ARGV[0];

	bool was_initialized = false;
	for(struct flash_bank *bank_iter = flash_bank_list(); bank_iter; bank_iter = bank_iter->next) {
		struct flash_bank *bank = bank_iter;

		if(bank_name && strcmp(bank_name, bank->name))
			continue;

		/* Handle also 'virtual' banks */
		const char *name = bank->name;
		if(bank_name && strcmp(bank->driver->name, "virtual") == 0)
			bank = get_flash_bank_by_name_noprobe(bank->driver_priv);

		if(strcmp("cmsis_flash", bank->driver->name))
			continue;

		struct cmsis_flash *algo = bank->driver_priv;
		LOG_INFO("Initializing cmsis_flash bank %s (%s)", name, algo->flash_dev.dev_name);
		was_initialized = true;

		//enum log_levels lvl = change_debug_level(LOG_LVL_USER);
		int hr = cmsis_flash_prepare_algo(bank, CMSIS_OPERATION_VERIFY);
		if(hr == ERROR_OK)
			cmsis_flash_release_algo(bank);

		//change_debug_level(lvl);

		if(hr != ERROR_OK)
			LOG_ERROR("Failed to initialize bank %s", name);
	}

	if(bank_name && !was_initialized) {
		LOG_ERROR("Flash bank '%s' not found", bank_name);
		return ERROR_FAIL;
	}

	return ERROR_OK;
}

COMMAND_HANDLER(prefer_sector_erase)
{
  LOG_DEBUG("---> prefer_sector_erase");
	int arg_index_bank = -1;
	const char *arg_str_onoff = NULL;

	if(CMD_ARGC == 1) {
		arg_str_onoff = CMD_ARGV[0];
	} else if(CMD_ARGC == 2) {
		arg_index_bank = 0; /* CMD_ARGV[0] */
		arg_str_onoff = CMD_ARGV[1];
	} else {
		LOG_ERROR("cmsis_flash prefer_sector_erase: wrong number of params");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}
	
	/* Parse on/off parameter accepting any of following: "on|off|enable|disable|yes|no|true|false|1|0" */
	bool arg_onoff = false;
	int retval = command_parse_bool_arg(arg_str_onoff, &arg_onoff);
	if (ERROR_OK != retval) {
		LOG_ERROR("cmsis_flash prefer_sector_erase: wrong arguments");
		return ERROR_COMMAND_SYNTAX_ERROR;
	}

	if (arg_index_bank >= 0) {
		/* Optional bank Id is provided, apply for specified bank only */
		struct flash_bank *bank;
		retval = CALL_COMMAND_HANDLER(flash_command_get_bank, arg_index_bank, &bank);
		if ((ERROR_OK != retval) || (strcmp("cmsis_flash", bank->driver->name) != 0)) {
			LOG_ERROR("cmsis_flash prefer_sector_erase: wrong bank_id or driver is not cmsis_flash");
			return ERROR_COMMAND_SYNTAX_ERROR;
		}
		struct cmsis_flash *algo = bank->driver_priv;
		LOG_INFO("Setting \'prefer_sector_erase %s\' for cmsis_flash bank %s (%s)", arg_str_onoff, bank->name, algo->flash_dev.dev_name);
		algo->prefer_sector_erase = arg_onoff;
	} else {
		/* Bank Id is not provided, apply for all banks */
		for(struct flash_bank *bank_iter = flash_bank_list(); bank_iter; bank_iter = bank_iter->next) {
			struct flash_bank *bank = bank_iter;
			if(strcmp("cmsis_flash", bank->driver->name) != 0)
				continue;
			struct cmsis_flash *algo = bank->driver_priv;
			LOG_INFO("Setting \'prefer_sector_erase %s\' for cmsis_flash bank %s (%s)", arg_str_onoff, bank->name, algo->flash_dev.dev_name);
			algo->prefer_sector_erase = arg_onoff;
		}
	}

	return ERROR_OK;
}

static const struct command_registration cmsis_flash_exec_command_handlers[] = {
	{
		.name = "init",
		.handler = cmsis_flash_initialize,
		.mode = COMMAND_EXEC,
		.usage = "[bank name]",
		.help = "initializes all cmsis_flash banks",
	},
	{
		.name = "prefer_sector_erase",
		.handler = prefer_sector_erase,
		.mode = COMMAND_EXEC,
		.usage = "[bank_id] <on|off|enable|disable|yes|no|true|false|1|0>",
		.help = "sets prefer_sector_erase property",
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration cmsis_flash_command_handlers[] = {
	{
		.name = "cmsis_flash",
		.mode = COMMAND_EXEC,
		.help = "cmsis flash command group",
		.usage = "",
		.chain = cmsis_flash_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver cmsis_flash = {
  .usage = "$_FLASHNAME cmsis_flash <addr:0> <size:0> 0 0 <target> <algorithm_elf> <stack_size>",
	.name = "cmsis_flash",
	.commands = cmsis_flash_command_handlers,
	.flash_bank_command = cmsis_flash_bank_command,
	.erase = cmsis_flash_erase,
  .verify = cmsis_flash_verify,
	.protect = cmsis_flash_protect,
	.write = cmsis_flash_program,
	.read = default_flash_read,
	.probe = cmsis_flash_probe,
	.auto_probe = cmsis_flash_auto_probe,
	.erase_check = default_flash_blank_check,
	.protect_check = cmsis_flash_protect_check,
	.info = cmsis_flash_get_info,
	.free_driver_priv = cmsis_flash_free_driver_priv,
};
