// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include "spi.h"
#include <target/cortex_m.h>

/* NOTE THAT THIS CODE REQUIRES FLASH ROUTINES in BOOTROM WITH FUNCTION TABLE PTR AT 0x00000010
   Your gdbinit should load the bootrom.elf if appropriate */

/* this is 'M' 'u', 1 (version) */
#define BOOTROM_RP2040_MAGIC 0x01754d
/* this is 'M' 'u', 2 (version) */
#define BOOTROM_RP2350_MAGIC 0x02754d
#define BOOTROM_MAGIC_ADDR 0x00000010

#define RT_ARM_FUNC 0x1

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
#define FUNC_BOOTROM_STATE_RESET    MAKE_TAG('S', 'R')

// these form a bit set
#define BOOTROM_STATE_RESET_CURRENT_CORE 0x01
#define BOOTROM_STATE_RESET_OTHER_CORE   0x02
#define BOOTROM_STATE_RESET_GLOBAL_STATE 0x04

#define ACCESSCTRL_LOCK_OFFSET     0x40060000u
#define ACCESSCTRL_LOCK_DEBUG_BITS 0x00000008u
#define ACCESSCTRL_CFGRESET_OFFSET 0x40060008u
#define ACCESSCTRL_WRITE_PASSWORD  0xacce0000u

// Calling bootrom functions requires the redundancy coprocessor (RCP) to be
// initialised. Usually this is done first thing by the bootrom, but the
// debugger may skip this, e.g. by resetting the cores and then running a
// NO_FLASH binary, or by reset-halting the cores before flash programming.
//
// The first case can be handled by a stub in the binary itself to initialise
// the RCP with dummy values if the bootrom has not already initialised it.
// (Note this case is only reachable via the debugger.) The second case
// requires the debugger itself to initialise the RCP, using this stub code:

static const int rcp_init_code_bkpt_offset = 24;
static const uint16_t rcp_init_code[] = {
	// Just enable the RCP which is fine if it already was (we assume no other
	// co-processors are enabled at this point to save space)
	0x4806,         // ldr r0, = PPB_BASE + M33_CPACR_OFFSET
	0xf45f, 0x4140, // movs r1, #M33_CPACR_CP7_BITS
	0x6001,         // str r1, [r0]
	// Only initialize canary seeds if they haven't been (as to do so twice is a fault)
	0xee30, 0xf710, // mrc p7, #1, r15, c0, c0, #0
	0xd404,         // bmi 1f
	// Todo should we use something random here and pass it into the algorithm?
	0xec40, 0x0780, // mcrr p7, #8, r0, r0, c0
	0xec40, 0x0781, // mcrr p7, #8, r0, r0, c1
	// Let other core know
	0xbf40,         // sev
	// 1:
	0xbe00,         // bkpt (end of algorithm)
	0x0000,         // pad
	0xed88, 0xe000  // PPB_BASE + M33_CPACR_OFFSET
};

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
	uint16_t jump_bootrom_reset_state;
};

static uint32_t rp2040_lookup_symbol(struct target *target, uint32_t tag, uint16_t *symbol)
{
	uint32_t magic, magic_addr;
	bool found_rp2040_magic, found_rp2350_magic;
	magic_addr = BOOTROM_MAGIC_ADDR;
	int err = target_read_u32(target, BOOTROM_MAGIC_ADDR, &magic);
	if (err != ERROR_OK)
		return err;

	magic &= 0xffffff; /* ignore bootrom version */

	found_rp2040_magic = magic == BOOTROM_RP2040_MAGIC;
	found_rp2350_magic = magic == BOOTROM_RP2350_MAGIC;

	if (!(found_rp2040_magic || found_rp2350_magic)) {
		LOG_ERROR("RP2040/RP2350 BOOT ROM not found");
		return ERROR_FAIL;
	}

	/* dereference the table pointer */
	uint16_t table_entry;
	err = target_read_u16(target, magic_addr + 4, &table_entry);
	if (err != ERROR_OK)
		return err;

	uint16_t entry_tag;
	do {
		err = target_read_u16(target, table_entry, &entry_tag);
		if (err != ERROR_OK)
			return err;
		if (entry_tag == tag) {
			if (found_rp2350_magic) {
				uint16_t flags;
				/* flags are next */
				err = target_read_u16(target, table_entry + 4, &flags);
				if (err != ERROR_OK)
					return err;
				//
				if (flags & RT_ARM_FUNC) {
					/* 16 bit symbol */
					return target_read_u16(target, table_entry + 2, symbol);
				}
			} else {
				/* 16 bit symbol is next */
				return target_read_u16(target, table_entry + 2, symbol);
			}
		}
		table_entry += found_rp2350_magic ? 6 : 4;
	} while (entry_tag);
	return ERROR_FAIL;
}

static int rp2040_call_rom_func(struct target *target, struct rp2040_flash_bank *priv,
		uint16_t func_offset, uint32_t argdata[], unsigned int n_args)
{
	char *regnames[4] = { "r0", "r1", "r2", "r3" };

	assert(n_args <= ARRAY_SIZE(regnames)); /* only allow register arguments */

	if (!priv->stack) {
		LOG_ERROR("no stack for flash programming code");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	target_addr_t stacktop = priv->stack->address + priv->stack->size;

	LOG_DEBUG("Calling ROM func @0x%" PRIx16 " with %d arguments", func_offset, n_args);
	LOG_DEBUG("Calling on core \"%s\"", target->cmd_name);

	struct reg_param args[ARRAY_SIZE(regnames) + 12];
	struct armv7m_algorithm alg_info;

	for (unsigned int i = 0; i < n_args; ++i) {
		init_reg_param(&args[i], regnames[i], 32, PARAM_OUT);
		buf_set_u32(args[i].value, 0, 32, argdata[i]);
	}
	/* Pass function pointer in r7 */
	unsigned int extra_args = 0;
	init_reg_param(&args[n_args + extra_args], "r7", 32, PARAM_OUT);
	buf_set_u32(args[n_args + extra_args++].value, 0, 32, func_offset);
	/* Set stack pointer, have seen the caching get confused by the aliases of sp so
	   take the shotgun approach*/
	init_reg_param(&args[n_args + extra_args], "msp_s", 32, PARAM_OUT);
	buf_set_u32(args[n_args + extra_args++].value, 0, 32, stacktop);
	init_reg_param(&args[n_args + extra_args], "msp_ns", 32, PARAM_OUT);
	buf_set_u32(args[n_args + extra_args++].value, 0, 32, stacktop);
	init_reg_param(&args[n_args + extra_args], "psp_s", 32, PARAM_OUT);
	buf_set_u32(args[n_args + extra_args++].value, 0, 32, stacktop);
	init_reg_param(&args[n_args + extra_args], "psp_ns", 32, PARAM_OUT);
	buf_set_u32(args[n_args + extra_args++].value, 0, 32, stacktop);
	init_reg_param(&args[n_args + extra_args], "msp", 32, PARAM_OUT);
	buf_set_u32(args[n_args + extra_args++].value, 0, 32, stacktop);
	init_reg_param(&args[n_args + extra_args], "psp", 32, PARAM_OUT);
	buf_set_u32(args[n_args + extra_args++].value, 0, 32, stacktop);
	init_reg_param(&args[n_args + extra_args], "sp", 32, PARAM_OUT);
	buf_set_u32(args[n_args + extra_args++].value, 0, 32, stacktop);
	/* Clear stack pointer limits, as they may be above the algorithm stack */
	init_reg_param(&args[n_args + extra_args], "msplim_s", 32, PARAM_OUT);
	buf_set_u32(args[n_args + extra_args++].value, 0, 32, 0);
	init_reg_param(&args[n_args + extra_args], "psplim_s", 32, PARAM_OUT);
	buf_set_u32(args[n_args + extra_args++].value, 0, 32, 0);
	init_reg_param(&args[n_args + extra_args], "msplim_ns", 32, PARAM_OUT);
	buf_set_u32(args[n_args + extra_args++].value, 0, 32, 0);
	init_reg_param(&args[n_args + extra_args], "psplim_ns", 32, PARAM_OUT);
	buf_set_u32(args[n_args + extra_args++].value, 0, 32, 0);

	for (unsigned int i = 0; i < n_args + extra_args; ++i)
		LOG_DEBUG("Set %s = 0x%" PRIx32, args[i].reg_name, buf_get_u32(args[i].value, 0, 32));

	/* Actually call the function */
	alg_info.common_magic = ARMV7M_COMMON_MAGIC;
	alg_info.core_mode = ARM_MODE_THREAD;
	int err = target_run_algorithm(
		target,
		0, NULL,          /* No memory arguments */
		n_args + extra_args, args, /* User arguments + r7 + SPs */
		priv->jump_debug_trampoline, priv->jump_debug_trampoline_end,
		3000, /* 3s timeout */
		&alg_info
	);
	for (unsigned int i = 0; i < n_args + extra_args; ++i)
		destroy_reg_param(&args[i]);
	if (err != ERROR_OK)
		LOG_ERROR("Failed to invoke ROM function @0x%" PRIx16 "\n", func_offset);
	return err;
}

static int rp2350_init_core(struct target *target, struct rp2040_flash_bank *priv)
{
	if (!priv->stack) {
		LOG_ERROR("no stack for flash programming code");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	struct armv7m_algorithm alg_info;

	// copy rcp_init code onto stack, as we don't actually need any stack during the call
	if (priv->stack->size < sizeof(rcp_init_code)) {
		LOG_ERROR("Working area too small for rcp_init");
		return ERROR_BUF_TOO_SMALL;
	}

	// Attempt to reset ACCESSCTRL before running RCP init, in case Secure
	// access to SRAM has been blocked. (Also ROM, QMI regs are needed later)
	uint32_t accessctrl_lock_reg;
	if (target_read_u32(target, ACCESSCTRL_LOCK_OFFSET, &accessctrl_lock_reg) != ERROR_OK) {
		LOG_ERROR("Failed to read ACCESSCTRL lock register");
		// Failed to read an APB register which should always be readable from
		// any security/privilege level. Something fundamental is wrong. E.g.:
		//
		// - The debugger is attempting to perform Secure bus accesses on a
		//   system where Secure debug has been disabled
		// - clk_sys or busfabric clock are stopped (try doing a rescue reset)
		return ERROR_FAIL;
	}
	if (accessctrl_lock_reg & ACCESSCTRL_LOCK_DEBUG_BITS) {
		LOG_ERROR("ACCESSCTRL is locked, so can't reset permissions. Following steps might fail.\n");
	} else {
		LOG_DEBUG("Reset ACCESSCTRL permissions via CFGRESET\n");
		target_write_u32(target, ACCESSCTRL_CFGRESET_OFFSET, ACCESSCTRL_WRITE_PASSWORD | 1u);
	}

	int err = target_write_memory(target,
		priv->stack->address,
		1,
		sizeof(rcp_init_code),
		(const uint8_t *)rcp_init_code
	);
	if (err != ERROR_OK) {
		LOG_ERROR("Failed to load rcp_init algorithm into RAM\n");
		return ERROR_FAIL;
	}

	LOG_DEBUG("Calling rcp_init core \"%s\" code at 0x%" PRIx16 "\n", target->cmd_name, (uint32_t)priv->stack->address);

	/* Actually call the function */
	alg_info.common_magic = ARMV7M_COMMON_MAGIC;
	alg_info.core_mode = ARM_MODE_THREAD;
	err = target_run_algorithm(target,
			0, NULL,          /* No memory arguments */
			0, NULL,          /* No register arguments */
			priv->stack->address,
			priv->stack->address + rcp_init_code_bkpt_offset,
			1000, /* 1s timeout */
			&alg_info
	);
	if (err != ERROR_OK) {
		LOG_ERROR("Failed to invoke rcp_init\n");
		return err;
	}

	uint32_t reset_args[1] = {
			BOOTROM_STATE_RESET_CURRENT_CORE
	};
	if (!priv->jump_bootrom_reset_state) {
		LOG_WARNING("RP2350 flash: no bootrom_reset_method\n");
	} else {
		err = rp2040_call_rom_func(target, priv, priv->jump_bootrom_reset_state, reset_args, ARRAY_SIZE(reset_args));
		if (err != ERROR_OK) {
			LOG_ERROR("RP2040 flash: failed to call reset core state");
			return err;
		}
	}

	return err;
}

static int setup_for_rom_call(struct flash_bank *bank)
{
	struct rp2040_flash_bank *priv = bank->driver_priv;

	struct target *target = bank->target;

	int err = ERROR_OK;
	if (!priv->stack) {
		/* target_alloc_working_area always allocates multiples of 4 bytes, so no worry about alignment */
		const int STACK_SIZE = 256;
		target_alloc_working_area(bank->target, STACK_SIZE, &priv->stack);
		if (err != ERROR_OK) {
			LOG_ERROR("Could not allocate stack for flash programming code");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	// Flash algorithms must run in Secure state
	uint32_t dscsr;
	(void)target_read_u32(target, DCB_DSCSR, &dscsr);
	LOG_DEBUG("DSCSR:  %08x\n", dscsr);
	if (!(dscsr & DSCSR_CDS)) {
		LOG_DEBUG("Setting Current Domain Secure in DSCSR\n");
		(void)target_write_u32(target, DCB_DSCSR, (dscsr & ~DSCSR_CDSKEY) | DSCSR_CDS);
		(void)target_read_u32(target, DCB_DSCSR, &dscsr);
		LOG_DEBUG("DSCSR*: %08x\n", dscsr);
	}

	// hacky RP2350 check
	if (target_to_arm(target)->arch == ARM_ARCH_V8M) {
		err = rp2350_init_core(target, priv);
		if (err != ERROR_OK) {
			LOG_ERROR("RP2350 flash: failed to init core");
			return err;
		}
	}
	return err;
}

static int setup_for_raw_flash_cmd(struct flash_bank *bank)
{
	struct rp2040_flash_bank *priv = bank->driver_priv;
	int err = setup_for_rom_call(bank);
	err = rp2040_call_rom_func(bank->target, priv, priv->jump_connect_internal_flash, NULL, 0);
	if (err != ERROR_OK) {
		LOG_ERROR("RP2040 flash: failed to setup for rom call");
		return err;
	}

	LOG_DEBUG("Connecting internal flash");
	err = rp2040_call_rom_func(bank->target, priv, priv->jump_connect_internal_flash, NULL, 0);
	if (err != ERROR_OK) {
		LOG_ERROR("RP2040 flash: failed to connect internal flash");
		return err;
	}

	LOG_DEBUG("Kicking flash out of XIP mode");
	err = rp2040_call_rom_func(bank->target, priv, priv->jump_flash_exit_xip, NULL, 0);
	if (err != ERROR_OK) {
		LOG_ERROR("RP2040 flash: failed to exit flash XIP mode");
		return err;
	}

	return ERROR_OK;
}

static int rp2040_flash_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	LOG_DEBUG("Writing %d bytes starting at 0x%" PRIx32, count, offset);

	struct rp2040_flash_bank *priv = bank->driver_priv;
	struct target *target = bank->target;
	struct working_area *bounce;

	int err = setup_for_raw_flash_cmd(bank);
	if (err != ERROR_OK)
		return err;

	// Allocate as much memory as possible, rounded down to a whole number of flash pages
	const unsigned int chunk_size = target_get_working_area_avail(target) & ~0xffu;
	if (chunk_size == 0 || target_alloc_working_area(target, chunk_size, &bounce) != ERROR_OK) {
		LOG_ERROR("Could not allocate bounce buffer for flash programming. Can't continue");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
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
		err = rp2040_call_rom_func(target, priv, priv->jump_flash_range_program, args, ARRAY_SIZE(args));
		if (err != ERROR_OK) {
			LOG_ERROR("Failed to invoke flash programming code on target");
			break;
		}

		buffer += write_size;
		offset += write_size;
		count -= write_size;
	}
	target_free_working_area(target, bounce);

	if (err != ERROR_OK)
		return err;

	/* Flash is successfully programmed. We can now do a bit of poking to make the flash
	   contents visible to us via memory-mapped (XIP) interface in the 0x1... memory region */
	LOG_DEBUG("Flushing flash cache after write behind");
	err = rp2040_call_rom_func(bank->target, priv, priv->jump_flush_cache, NULL, 0);
	if (err != ERROR_OK) {
		LOG_ERROR("RP2040 write: failed to flush flash cache");
		return err;
	}
	LOG_DEBUG("Configuring SSI for execute-in-place");
	err = rp2040_call_rom_func(bank->target, priv, priv->jump_enter_cmd_xip, NULL, 0);
	if (err != ERROR_OK)
		LOG_ERROR("RP2040 write: failed to flush flash cache");
	return err;
}

static int rp2040_flash_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct rp2040_flash_bank *priv = bank->driver_priv;
	uint32_t start_addr = bank->sectors[first].offset;
	uint32_t length = bank->sectors[last].offset + bank->sectors[last].size - start_addr;
	LOG_DEBUG("RP2040 erase %d bytes starting at 0x%" PRIx32, length, start_addr);

	int err = setup_for_raw_flash_cmd(bank);
	if (err != ERROR_OK)
		return err;

	LOG_DEBUG("Remote call flash_range_erase");

	uint32_t args[4] = {
		bank->sectors[first].offset, /* addr */
		bank->sectors[last].offset + bank->sectors[last].size - bank->sectors[first].offset, /* count */
		65536, /* block_size */
		0xd8   /* block_cmd */
	};

	/*
	The RP2040 Boot ROM provides a _flash_range_erase() API call documented in Section 2.8.3.1.3:
	https://datasheets.raspberrypi.org/rp2040/rp2040-datasheet.pdf
	and the particular source code for said Boot ROM function can be found here:
	https://github.com/raspberrypi/pico-bootrom/blob/master/bootrom/program_flash_generic.c

	In theory, the function algorithm provides for erasing both a smaller "sector" (4096 bytes) and
	an optional larger "block" (size and command provided in args).  OpenOCD's spi.c only uses "block" sizes.
	*/

	err = rp2040_call_rom_func(bank->target, priv, priv->jump_flash_range_erase, args, ARRAY_SIZE(args));

	return err;
}

/* -----------------------------------------------------------------------------
   Driver probing etc */

static int rp2040_flash_probe(struct flash_bank *bank)
{
	struct rp2040_flash_bank *priv = bank->driver_priv;
	struct target *target = bank->target;

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
	LOG_WARNING("GOT FLASH ERASE AT %08x\n", (int)priv->jump_flash_range_erase);

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

	err = rp2040_lookup_symbol(target, FUNC_BOOTROM_STATE_RESET, &priv->jump_bootrom_reset_state);
	if (err != ERROR_OK) {
		priv->jump_bootrom_reset_state = 0;
//        LOG_ERROR("Function FUNC_BOOTROM_STATE_RESET not found in RP2040 ROM.");
//        return err;
	}

	/* the Boot ROM flash_range_program() routine requires page alignment */
	bank->write_start_alignment = 256;
	bank->write_end_alignment = 256;

	// Max size -- up to two devices (two chip selects) in adjacent 24-bit address windows
	bank->size = 32 * 1024 * 1024;

	bank->num_sectors = bank->size / 4096;

	LOG_INFO("RP2040 Flash Probe: %d bytes @" TARGET_ADDR_FMT ", in %d sectors\n",
		bank->size, bank->base, bank->num_sectors);
	bank->sectors = alloc_block_array(0, 4096, bank->num_sectors);
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
