// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include "spi.h"

/* NOTE THAT THIS CODE REQUIRES FLASH ROUTINES in BOOTROM WITH FUNCTION TABLE PTR AT 0x00000010
   Your gdbinit should load the bootrom.elf if appropriate */

/* this is 'M' 'u', 1 (version) */
#define BOOTROM_MAGIC 0x01754d
#define BOOTROM_MAGIC_ADDR 0x00000010

/* Call a ROM function via the debug trampoline
   Up to four arguments passed in r0...r3 as per ABI
   Function address is passed in r7
   the trampoline is needed because OpenOCD "algorithm" code insists on sw breakpoints. */

#define MAKE_TAG(a, b) (((b)<<8) | a)
#define FUNC_DEBUG_TRAMPOLINE       MAKE_TAG('D', 'T')
#define FUNC_DEBUG_TRAMPOLINE_END   MAKE_TAG('D', 'E')
#define FUNC_FLASH_EXIT_XIP         MAKE_TAG('E', 'X')
#define FUNC_CONNECT_INTERNAL_FLASH MAKE_TAG('I', 'F')
#define FUNC_FLASH_RANGE_ERASE      MAKE_TAG('R', 'E')
#define FUNC_FLASH_RANGE_PROGRAM    MAKE_TAG('R', 'P')
#define FUNC_FLASH_FLUSH_CACHE      MAKE_TAG('F', 'C')
#define FUNC_FLASH_ENTER_CMD_XIP    MAKE_TAG('C', 'X')

struct rp2040_flash_bank {
	/* flag indicating successful flash probe */
	bool probed;
	/* stack used by Boot ROM calls */
	struct working_area *stack;
	/* function jump table populated by rp2040_flash_probe() */
	uint16_t jump_debug_trampoline;
	uint16_t jump_debug_trampoline_end;
	uint16_t jump_flash_exit_xip;
	uint16_t jump_connect_internal_flash;
	uint16_t jump_flash_range_erase;
	uint16_t jump_flash_range_program;
	uint16_t jump_flush_cache;
	uint16_t jump_enter_cmd_xip;
	/* detected model of SPI flash */
	const struct flash_device *dev;
};

/* guessed SPI flash description if autodetection disabled (same as win w25q16jv) */
static const struct flash_device rp2040_default_spi_device =
	FLASH_ID("autodetect disabled", 0x03, 0x00, 0x02, 0xd8, 0xc7, 0, 0x100, 0x10000, 0);

static uint32_t rp2040_lookup_symbol(struct target *target, uint32_t tag, uint16_t *symbol)
{
	uint32_t magic;
	int err = target_read_u32(target, BOOTROM_MAGIC_ADDR, &magic);
	if (err != ERROR_OK)
		return err;

	magic &= 0xffffff; /* ignore bootrom version */
	if (magic != BOOTROM_MAGIC) {
		if (!((magic ^ BOOTROM_MAGIC)&0xffff))
			LOG_ERROR("Incorrect RP2040 BOOT ROM version");
		else
			LOG_ERROR("RP2040 BOOT ROM not found");
		return ERROR_FAIL;
	}

	/* dereference the table pointer */
	uint16_t table_entry;
	err = target_read_u16(target, BOOTROM_MAGIC_ADDR + 4, &table_entry);
	if (err != ERROR_OK)
		return err;

	uint16_t entry_tag;
	do {
		err = target_read_u16(target, table_entry, &entry_tag);
		if (err != ERROR_OK)
			return err;
		if (entry_tag == tag) {
			/* 16 bit symbol is next */
			return target_read_u16(target, table_entry + 2, symbol);
		}
		table_entry += 4;
	} while (entry_tag);
	return ERROR_FAIL;
}

static int rp2040_call_rom_func(struct target *target, struct rp2040_flash_bank *priv,
		uint16_t func_offset, uint32_t argdata[], unsigned int n_args, unsigned int timeout_ms)
{
	char *regnames[4] = { "r0", "r1", "r2", "r3" };

	assert(n_args <= ARRAY_SIZE(regnames)); /* only allow register arguments */

	if (!priv->stack) {
		LOG_ERROR("no stack for flash programming code");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	target_addr_t stacktop = priv->stack->address + priv->stack->size;

	LOG_TARGET_DEBUG(target, "Calling ROM func @0x%" PRIx16 " with %u arguments", func_offset, n_args);

	struct reg_param args[ARRAY_SIZE(regnames) + 2];
	struct armv7m_algorithm alg_info;

	for (unsigned int i = 0; i < n_args; ++i) {
		init_reg_param(&args[i], regnames[i], 32, PARAM_OUT);
		buf_set_u32(args[i].value, 0, 32, argdata[i]);
	}
	/* Pass function pointer in r7 */
	init_reg_param(&args[n_args], "r7", 32, PARAM_OUT);
	buf_set_u32(args[n_args].value, 0, 32, func_offset);
	/* Setup stack */
	init_reg_param(&args[n_args + 1], "sp", 32, PARAM_OUT);
	buf_set_u32(args[n_args + 1].value, 0, 32, stacktop);
	unsigned int n_reg_params = n_args + 2;	/* User arguments + r7 + sp */

	for (unsigned int i = 0; i < n_reg_params; ++i)
		LOG_DEBUG("Set %s = 0x%" PRIx32, args[i].reg_name, buf_get_u32(args[i].value, 0, 32));

	/* Actually call the function */
	alg_info.common_magic = ARMV7M_COMMON_MAGIC;
	alg_info.core_mode = ARM_MODE_THREAD;
	int err = target_run_algorithm(
		target,
		0, NULL,          /* No memory arguments */
		n_reg_params, args, /* User arguments + r7 + sp */
		priv->jump_debug_trampoline, priv->jump_debug_trampoline_end,
		timeout_ms,
		&alg_info
	);

	for (unsigned int i = 0; i < n_reg_params; ++i)
		destroy_reg_param(&args[i]);

	if (err != ERROR_OK)
		LOG_ERROR("Failed to invoke ROM function @0x%" PRIx16, func_offset);

	return err;
}

/* Finalize flash write/erase/read ID
 * - flush cache
 * - enters memory-mapped (XIP) mode to make flash data visible
 * - deallocates target ROM func stack if previously allocated
 */
static int rp2040_finalize_stack_free(struct flash_bank *bank)
{
	struct rp2040_flash_bank *priv = bank->driver_priv;
	struct target *target = bank->target;

	/* Always flush before returning to execute-in-place, to invalidate stale
	 * cache contents. The flush call also restores regular hardware-controlled
	 * chip select following a rp2040_flash_exit_xip().
	 */
	LOG_DEBUG("Flushing flash cache after write behind");
	int err = rp2040_call_rom_func(target, priv, priv->jump_flush_cache, NULL, 0, 1000);
	if (err != ERROR_OK) {
		LOG_ERROR("Failed to flush flash cache");
		/* Intentionally continue after error and try to setup xip anyway */
	}

	LOG_DEBUG("Configuring SSI for execute-in-place");
	err = rp2040_call_rom_func(target, priv, priv->jump_enter_cmd_xip, NULL, 0, 1000);
	if (err != ERROR_OK)
		LOG_ERROR("Failed to set SSI to XIP mode");

	target_free_working_area(target, priv->stack);
	priv->stack = NULL;
	return err;
}

/* Prepare flash write/erase/read ID
 * - allocates a stack for target ROM func
 * - switches the SPI interface from memory-mapped mode to direct command mode
 * Always pair with a call of rp2040_finalize_stack_free()
 * after flash operation finishes or fails.
 */
static int rp2040_stack_grab_and_prep(struct flash_bank *bank)
{
	struct rp2040_flash_bank *priv = bank->driver_priv;
	struct target *target = bank->target;

	/* target_alloc_working_area always allocates multiples of 4 bytes, so no worry about alignment */
	const int STACK_SIZE = 256;
	int err = target_alloc_working_area(target, STACK_SIZE, &priv->stack);
	if (err != ERROR_OK) {
		LOG_ERROR("Could not allocate stack for flash programming code");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	LOG_DEBUG("Connecting internal flash");
	err = rp2040_call_rom_func(target, priv, priv->jump_connect_internal_flash, NULL, 0, 1000);
	if (err != ERROR_OK) {
		LOG_ERROR("Failed to connect internal flash");
		return err;
	}

	LOG_DEBUG("Kicking flash out of XIP mode");
	err = rp2040_call_rom_func(target, priv, priv->jump_flash_exit_xip, NULL, 0, 1000);
	if (err != ERROR_OK) {
		LOG_ERROR("Failed to exit flash XIP mode");
		return err;
	}

	return ERROR_OK;
}

static int rp2040_flash_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	LOG_DEBUG("Writing %d bytes starting at 0x%" PRIx32, count, offset);

	struct rp2040_flash_bank *priv = bank->driver_priv;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	struct working_area *bounce = NULL;

	int err = rp2040_stack_grab_and_prep(bank);
	if (err != ERROR_OK)
		goto cleanup;

	unsigned int avail_pages = target_get_working_area_avail(target) / priv->dev->pagesize;
	/* We try to allocate working area rounded down to device page size,
	 * al least 1 page, at most the write data size
	 */
	unsigned int chunk_size = MIN(MAX(avail_pages, 1) * priv->dev->pagesize, count);
	err = target_alloc_working_area(target, chunk_size, &bounce);
	if (err != ERROR_OK) {
		LOG_ERROR("Could not allocate bounce buffer for flash programming. Can't continue");
		goto cleanup;
	}

	LOG_DEBUG("Allocated flash bounce buffer @" TARGET_ADDR_FMT, bounce->address);

	while (count > 0) {
		uint32_t write_size = count > chunk_size ? chunk_size : count;
		LOG_DEBUG("Writing %d bytes to offset 0x%" PRIx32, write_size, offset);
		err = target_write_buffer(target, bounce->address, write_size, buffer);
		if (err != ERROR_OK) {
			LOG_ERROR("Could not load data into target bounce buffer");
			break;
		}
		uint32_t args[3] = {
			offset, /* addr */
			bounce->address, /* data */
			write_size /* count */
		};
		err = rp2040_call_rom_func(target, priv, priv->jump_flash_range_program,
								 args, ARRAY_SIZE(args), 3000);
		if (err != ERROR_OK) {
			LOG_ERROR("Failed to invoke flash programming code on target");
			break;
		}

		buffer += write_size;
		offset += write_size;
		count -= write_size;
	}

cleanup:
	target_free_working_area(target, bounce);

	rp2040_finalize_stack_free(bank);

	return err;
}

static int rp2040_flash_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct rp2040_flash_bank *priv = bank->driver_priv;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	uint32_t start_addr = bank->sectors[first].offset;
	uint32_t length = bank->sectors[last].offset + bank->sectors[last].size - start_addr;
	LOG_DEBUG("RP2040 erase %d bytes starting at 0x%" PRIx32, length, start_addr);

	int err = rp2040_stack_grab_and_prep(bank);
	if (err != ERROR_OK)
		goto cleanup;

	LOG_DEBUG("Remote call flash_range_erase");

	uint32_t args[4] = {
		bank->sectors[first].offset, /* addr */
		bank->sectors[last].offset + bank->sectors[last].size - bank->sectors[first].offset, /* count */
		priv->dev->sectorsize, /* block_size */
		priv->dev->erase_cmd /* block_cmd */
	};

	/*
	The RP2040 Boot ROM provides a _flash_range_erase() API call documented in Section 2.8.3.1.3:
	https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf
	and the particular source code for said Boot ROM function can be found here:
	https://github.com/raspberrypi/pico-bootrom/blob/master/bootrom/program_flash_generic.c

	In theory, the function algorithm provides for erasing both a smaller "sector" (4096 bytes) and
	an optional larger "block" (size and command provided in args).
	*/

	unsigned int timeout_ms = 2000 * (last - first) + 1000;
	err = rp2040_call_rom_func(target, priv, priv->jump_flash_range_erase,
							args, ARRAY_SIZE(args), timeout_ms);

cleanup:
	rp2040_finalize_stack_free(bank);

	return err;
}

/* -----------------------------------------------------------------------------
   Driver probing etc */

static int rp2040_ssel_active(struct target *target, bool active)
{
	const target_addr_t qspi_ctrl_addr = 0x4001800c;
	const uint32_t qspi_ctrl_outover_low  = 2UL << 8;
	const uint32_t qspi_ctrl_outover_high = 3UL << 8;
	uint32_t state = (active) ? qspi_ctrl_outover_low : qspi_ctrl_outover_high;
	uint32_t val;

	int err = target_read_u32(target, qspi_ctrl_addr, &val);
	if (err != ERROR_OK)
		return err;

	val = (val & ~qspi_ctrl_outover_high) | state;

	err = target_write_u32(target, qspi_ctrl_addr, val);
	if (err != ERROR_OK)
		return err;

	return ERROR_OK;
}

static int rp2040_spi_read_flash_id(struct target *target, uint32_t *devid)
{
	uint32_t device_id = 0;
	const target_addr_t ssi_dr0 = 0x18000060;

	int err = rp2040_ssel_active(target, true);

	/* write RDID request into SPI peripheral's FIFO */
	for (int count = 0; (count < 4) && (err == ERROR_OK); count++)
		err = target_write_u32(target, ssi_dr0, SPIFLASH_READ_ID);

	/* by this time, there is a receive FIFO entry for every write */
	for (int count = 0; (count < 4) && (err == ERROR_OK); count++) {
		uint32_t status;
		err = target_read_u32(target, ssi_dr0, &status);

		device_id >>= 8;
		device_id |= (status & 0xFF) << 24;
	}

	if (err == ERROR_OK)
		*devid = device_id >> 8;

	int err2 = rp2040_ssel_active(target, false);
	if (err2 != ERROR_OK)
		LOG_ERROR("SSEL inactive failed");

	return err;
}

static int rp2040_flash_probe(struct flash_bank *bank)
{
	struct rp2040_flash_bank *priv = bank->driver_priv;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	int err = rp2040_lookup_symbol(target, FUNC_DEBUG_TRAMPOLINE, &priv->jump_debug_trampoline);
	if (err != ERROR_OK) {
		LOG_ERROR("Debug trampoline not found in RP2040 ROM.");
		return err;
	}
	priv->jump_debug_trampoline &= ~1u; /* mask off thumb bit */

	err = rp2040_lookup_symbol(target, FUNC_DEBUG_TRAMPOLINE_END, &priv->jump_debug_trampoline_end);
	if (err != ERROR_OK) {
		LOG_ERROR("Debug trampoline end not found in RP2040 ROM.");
		return err;
	}
	priv->jump_debug_trampoline_end &= ~1u; /* mask off thumb bit */

	err = rp2040_lookup_symbol(target, FUNC_FLASH_EXIT_XIP, &priv->jump_flash_exit_xip);
	if (err != ERROR_OK) {
		LOG_ERROR("Function FUNC_FLASH_EXIT_XIP not found in RP2040 ROM.");
		return err;
	}

	err = rp2040_lookup_symbol(target, FUNC_CONNECT_INTERNAL_FLASH, &priv->jump_connect_internal_flash);
	if (err != ERROR_OK) {
		LOG_ERROR("Function FUNC_CONNECT_INTERNAL_FLASH not found in RP2040 ROM.");
		return err;
	}

	err = rp2040_lookup_symbol(target, FUNC_FLASH_RANGE_ERASE, &priv->jump_flash_range_erase);
	if (err != ERROR_OK) {
		LOG_ERROR("Function FUNC_FLASH_RANGE_ERASE not found in RP2040 ROM.");
		return err;
	}

	err = rp2040_lookup_symbol(target, FUNC_FLASH_RANGE_PROGRAM, &priv->jump_flash_range_program);
	if (err != ERROR_OK) {
		LOG_ERROR("Function FUNC_FLASH_RANGE_PROGRAM not found in RP2040 ROM.");
		return err;
	}

	err = rp2040_lookup_symbol(target, FUNC_FLASH_FLUSH_CACHE, &priv->jump_flush_cache);
	if (err != ERROR_OK) {
		LOG_ERROR("Function FUNC_FLASH_FLUSH_CACHE not found in RP2040 ROM.");
		return err;
	}

	err = rp2040_lookup_symbol(target, FUNC_FLASH_ENTER_CMD_XIP, &priv->jump_enter_cmd_xip);
	if (err != ERROR_OK) {
		LOG_ERROR("Function FUNC_FLASH_ENTER_CMD_XIP not found in RP2040 ROM.");
		return err;
	}

	if (bank->size) {
		/* size overridden, suppress reading SPI flash ID */
		priv->dev = &rp2040_default_spi_device;
		LOG_DEBUG("SPI flash autodetection disabled, using configured size");

	} else {
		/* zero bank size in cfg, read SPI flash ID and autodetect */
		err = rp2040_stack_grab_and_prep(bank);

		uint32_t device_id = 0;
		if (err == ERROR_OK)
			err = rp2040_spi_read_flash_id(target, &device_id);

		rp2040_finalize_stack_free(bank);

		if (err != ERROR_OK)
			return err;

		/* search for a SPI flash Device ID match */
		priv->dev = NULL;
		for (const struct flash_device *p = flash_devices; p->name ; p++)
			if (p->device_id == device_id) {
				priv->dev = p;
				break;
			}

		if (!priv->dev) {
			LOG_ERROR("Unknown flash device (ID 0x%08" PRIx32 ")", device_id);
			return ERROR_FAIL;
		}
		LOG_INFO("Found flash device '%s' (ID 0x%08" PRIx32 ")",
			priv->dev->name, priv->dev->device_id);

		bank->size = priv->dev->size_in_bytes;
	}

	/* the Boot ROM flash_range_program() routine requires page alignment */
	bank->write_start_alignment = priv->dev->pagesize;
	bank->write_end_alignment = priv->dev->pagesize;

	bank->num_sectors = bank->size / priv->dev->sectorsize;
	LOG_INFO("RP2040 B0 Flash Probe: %" PRIu32 " bytes @" TARGET_ADDR_FMT ", in %u sectors\n",
		bank->size, bank->base, bank->num_sectors);
	bank->sectors = alloc_block_array(0, priv->dev->sectorsize, bank->num_sectors);
	if (!bank->sectors)
		return ERROR_FAIL;

	if (err == ERROR_OK)
		priv->probed = true;

	return err;
}

static int rp2040_flash_auto_probe(struct flash_bank *bank)
{
	struct rp2040_flash_bank *priv = bank->driver_priv;

	if (priv->probed)
		return ERROR_OK;

	return rp2040_flash_probe(bank);
}

static void rp2040_flash_free_driver_priv(struct flash_bank *bank)
{
	free(bank->driver_priv);
	bank->driver_priv = NULL;
}

/* -----------------------------------------------------------------------------
   Driver boilerplate */

FLASH_BANK_COMMAND_HANDLER(rp2040_flash_bank_command)
{
	struct rp2040_flash_bank *priv;
	priv = malloc(sizeof(struct rp2040_flash_bank));
	priv->probed = false;

	/* Set up driver_priv */
	bank->driver_priv = priv;

	return ERROR_OK;
}

const struct flash_driver rp2040_flash = {
	.name = "rp2040_flash",
	.flash_bank_command = rp2040_flash_bank_command,
	.erase =  rp2040_flash_erase,
	.write = rp2040_flash_write,
	.read = default_flash_read,
	.probe = rp2040_flash_probe,
	.auto_probe = rp2040_flash_auto_probe,
	.erase_check = default_flash_blank_check,
	.free_driver_priv = rp2040_flash_free_driver_priv
};
