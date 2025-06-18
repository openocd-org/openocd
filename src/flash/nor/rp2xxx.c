// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "imp.h"
#include <helper/binarybuffer.h>
#include <target/algorithm.h>
#include <target/armv7m.h>
#include "spi.h"
#include "sfdp.h"
#include <target/cortex_m.h>

/* this is 'M' 'u', 1 (version) */
#define BOOTROM_RP2040_MAGIC 0x01754d
/* this is 'M' 'u', 2 (version) */
#define BOOTROM_RP2350_MAGIC 0x02754d
#define BOOTROM_MAGIC_MASK	 0xffffff
#define BOOTROM_MAGIC_ADDR 0x00000010

#define MAKE_TAG(a, b) (((b)<<8) | a)
#define FUNC_FLASH_EXIT_XIP            MAKE_TAG('E', 'X')
#define FUNC_CONNECT_INTERNAL_FLASH    MAKE_TAG('I', 'F')
#define FUNC_FLASH_RANGE_ERASE         MAKE_TAG('R', 'E')
#define FUNC_FLASH_RANGE_PROGRAM       MAKE_TAG('R', 'P')
#define FUNC_FLASH_FLUSH_CACHE         MAKE_TAG('F', 'C')
#define FUNC_FLASH_ENTER_CMD_XIP       MAKE_TAG('C', 'X')
#define FUNC_BOOTROM_STATE_RESET       MAKE_TAG('S', 'R')
#define FUNC_BOOTROM_SET_STACK         MAKE_TAG('S', 'S')
#define FUNC_FLASH_RESET_ADDRESS_TRANS MAKE_TAG('R', 'A')

/* ROM table flags for RP2350 A1 ROM onwards */
#define RT_FLAG_FUNC_RISCV      0x01
#define RT_FLAG_FUNC_ARM_SEC    0x04
#define RT_FLAG_FUNC_ARM_NONSEC 0x10
#define RT_FLAG_DATA            0x40

// these form a bit set
#define BOOTROM_STATE_RESET_CURRENT_CORE 0x01
#define BOOTROM_STATE_RESET_OTHER_CORE   0x02
#define BOOTROM_STATE_RESET_GLOBAL_STATE 0x04

#define ACCESSCTRL_LOCK_OFFSET			0x40060000u
#define ACCESSCTRL_CFGRESET_OFFSET		0x40060008u
#define ACCESSCTRL_GPIO_NSMASK0_OFFSET	0x4006000cu
#define ACCESSCTRL_GPIO_ROM_OFFSET		0x40060014u
#define ACCESSCTRL_GPIO_XIP_AUX_OFFSET	0x400600e8u

#define ACCESSCTRL_SAVE_BASE			ACCESSCTRL_GPIO_NSMASK0_OFFSET
#define ACCESSCTRL_SAVE_SIZE \
			(ACCESSCTRL_GPIO_XIP_AUX_OFFSET + 4 - ACCESSCTRL_SAVE_BASE)

#define ACCESSCTRL_LOCK_DEBUG_BITS		0x00000008u
#define ACCESSCTRL_WRITE_PASSWORD		0xacce0000u

#define RP2040_SSI_DR0			0x18000060
#define RP2040_QSPI_CTRL		0x4001800c

#define RP2040_QSPI_CTRL_OUTOVER_MASK	(3ul << 8)
#define RP2040_QSPI_CTRL_OUTOVER_LOW	(2ul << 8)
#define RP2040_QSPI_CTRL_OUTOVER_HIGH	(3ul << 8)

#define RP2350_QMI_DIRECT_CSR		0x400d0000
#define RP2350_QMI_DIRECT_TX		0x400d0004
#define RP2350_QMI_DIRECT_RX		0x400d0008

#define RP2350_QMI_DIRECT_CSR_EN			BIT(0)
#define RP2350_QMI_DIRECT_CSR_ASSERT_CS0N	BIT(2)
#define RP2350_QMI_DIRECT_TX_NOPUSH			BIT(20)
#define RP2350_QMI_DIRECT_TX_OE				BIT(19)

#define RP2XXX_SYSINFO_CHIP_ID		0x40000000
#define RP2XXX_CHIP_ID_PART_MANUFACTURER(id) ((id) & 0x0fffffff)
#define RP2XXX_CHIP_ID_MANUFACTURER			0x493
#define RP2XXX_MK_PART(part)				(((part) << 12) | (RP2XXX_CHIP_ID_MANUFACTURER << 1) | 1)
#define RP2040_CHIP_ID_PART					0x0002
#define IS_RP2040(id)	(RP2XXX_CHIP_ID_PART_MANUFACTURER(id) == RP2XXX_MK_PART(RP2040_CHIP_ID_PART))
#define RP2350_CHIP_ID_PART					0x0004
#define IS_RP2350(id)	(RP2XXX_CHIP_ID_PART_MANUFACTURER(id) == RP2XXX_MK_PART(RP2350_CHIP_ID_PART))
#define RP2XXX_CHIP_ID_REVISION(id)			((id) >> 28)

#define RP2XXX_MAX_ALGO_STACK_USAGE 1024
#define RP2XXX_MAX_RAM_ALGO_SIZE 1024

#define RP2XXX_ROM_API_FIXED_FLASH_PAGE		256
#define RP2XXX_ROM_API_FIXED_FLASH_SECTOR	4096

// Calling bootrom functions on Arm RP2350 requires the redundancy
// coprocessor (RCP) to be initialised. Usually this is done first thing by
// the bootrom, but the debugger may skip this, e.g. by resetting the cores
// and then running a NO_FLASH binary, or by reset-halting the cores before
// flash programming.
//
// The first case can be handled by a stub in the binary itself to initialise
// the RCP with dummy values if the bootrom has not already initialised it.
// (Note this case is only reachable via the debugger.) The second case
// requires the debugger itself to initialise the RCP, using this stub code:

static const int rcp_init_code_bkpt_offset = 24;
static const uint8_t rcp_init_code[] = {
	// Just enable the RCP which is fine if it already was (we assume no other
	// co-processors are enabled at this point to save space)
	0x06, 0x48,             // ldr r0, = PPB_BASE + M33_CPACR_OFFSET
	0x5f, 0xf4, 0x40, 0x41, // movs r1, #M33_CPACR_CP7_BITS
	0x01, 0x60,             // str r1, [r0]
	// Only initialize canary seeds if they haven't been (as to do so twice is a fault)
	0x30, 0xee, 0x10, 0xf7, // mrc p7, #1, r15, c0, c0, #0
	0x04, 0xd4,             // bmi 1f
	// Todo should we use something random here and pass it into the algorithm?
	0x40, 0xec, 0x80, 0x07, // mcrr p7, #8, r0, r0, c0
	0x40, 0xec, 0x81, 0x07, // mcrr p7, #8, r0, r0, c1
	// Let other core know
	0x40, 0xbf,             // sev
	// 1:
	0x00, 0xbe,             // bkpt (end of algorithm)
	0x00, 0x00,             // pad
	0x88, 0xed, 0x00, 0xe0  // PPB_BASE + M33_CPACR_OFFSET
};

// An algorithm stub that can be concatenated with a null-terminated list of
// (PC, SP, r0-r3) records to perform a batch of ROM calls under a single
// OpenOCD algorithm call, to save on algorithm overhead:
#define ROM_CALL_BATCH_ALGO_SIZE_BYTES 32
static const int rp2xxx_rom_call_batch_algo_bkpt_offset = ROM_CALL_BATCH_ALGO_SIZE_BYTES - 2;
static const uint8_t rp2xxx_rom_call_batch_algo_armv6m[ROM_CALL_BATCH_ALGO_SIZE_BYTES] = {
// <_start>:
	0x07, 0xa7,             // add r7, pc, #28 ; (adr r7, 20 <_args>)
// <_do_next>:
	0x10, 0xcf,             // ldmia   r7!, {r4}
	0x00, 0x2c,             // cmp r4, #0
	0x0a, 0xd0,             // beq.n   1e <_done>
	0x20, 0xcf,             // ldmia   r7!, {r5}
	0xad, 0x46,             // mov sp, r5
	0x0f, 0xcf,             // ldmia   r7!, {r0, r1, r2, r3}
	0xa0, 0x47,             // blx r4
	0xf7, 0xe7,             // b.n 2 <_do_next>
	0xc0, 0x46,             // nop
	0xc0, 0x46,             // nop
	0xc0, 0x46,             // nop
	0xc0, 0x46,             // nop
	0xc0, 0x46,             // nop
	0xc0, 0x46,             // nop
// <_done>:
	0x00, 0xbe,             // bkpt    0x0000
// <_args>:
};

// The same as rom_call_batch_algo_armv6m, but clearing stack limits before setting stack:
static const uint8_t rp2xxx_rom_call_batch_algo_armv8m[ROM_CALL_BATCH_ALGO_SIZE_BYTES] = {
// <_start>:
	0x07, 0xa7,             // add   r7, pc, #28 ; (adr r7, 20 <_args>)
	0x00, 0x20,             // movs  r0, #0
	0x80, 0xf3, 0x0a, 0x88, // msr   MSPLIM, r0
	0x80, 0xf3, 0x0b, 0x88, // msr   PSPLIM, r0
// <_do_next>:
	0x10, 0xcf,             // ldmia r7!, {r4}
	0x00, 0x2c,             // cmp   r4, #0
	0x05, 0xd0,             // beq.n 1e <_done>
	0x20, 0xcf,             // ldmia r7!, {r5}
	0xad, 0x46,             // mov   sp, r5
	0x0f, 0xcf,             // ldmia r7!, {r0, r1, r2, r3}
	0xa0, 0x47,             // blx   r4
	0xf7, 0xe7,             // b.n   c <_do_next>
	0xc0, 0x46,             // nop
// <_done>:
	0x00, 0xbe,             // bkpt  0x0000
// <_args>:
};

// The same as rom_call_batch_algo_armv6m, but placing arguments in a0-a3 on RISC-V:
static const uint8_t rp2xxx_rom_call_batch_algo_riscv[ROM_CALL_BATCH_ALGO_SIZE_BYTES] = {
// <_start>:
	0x97, 0x04, 0x00, 0x00, // auipc s1,0
	0x93, 0x84, 0x04, 0x02, // add   s1,s1,32 # 20 <_args>
// <_do_next>:
	0x98, 0x40,             // lw    a4,0(s1)
	0x11, 0xcb,             // beqz  a4,1e <_done>
	0x03, 0xa1, 0x44, 0x00, // lw    sp,4(s1)
	0x88, 0x44,             // lw    a0,8(s1)
	0xcc, 0x44,             // lw    a1,12(s1)
	0x90, 0x48,             // lw    a2,16(s1)
	0xd4, 0x48,             // lw    a3,20(s1)
	0xe1, 0x04,             // add   s1,s1,24
	0x02, 0x97,             // jalr  a4
	0xf5, 0xb7,             // j     8 <_do_next>
// <_done>:
	0x02, 0x90,             // ebreak
// <_args>:
};

struct rp2xxx_rom_call_batch_record {
	uint32_t pc;
	uint32_t sp;
	uint32_t args[4];
};

struct rp2xxx_flash_bank {
	bool probed;						/* flag indicating successful flash probe */
	uint32_t id;						/* cached SYSINFO CHIP_ID */
	struct working_area *stack;			/* stack used by Boot ROM calls */
	/* static code scratchpad used for RAM algorithms -- allocated in advance
	   so that higher-level calls can just grab all remaining workarea: */
	struct working_area *ram_algo_space;
	/* function jump table populated by rp2xxx_flash_probe() */
	uint16_t jump_flash_exit_xip;
	uint16_t jump_connect_internal_flash;
	uint16_t jump_flash_range_erase;
	uint16_t jump_flash_range_program;
	uint16_t jump_flush_cache;
	uint16_t jump_flash_reset_address_trans;
	uint16_t jump_enter_cmd_xip;
	uint16_t jump_bootrom_reset_state;
	uint16_t jump_bootrom_set_varm_stack;

	char dev_name[20];
	bool size_override;
	struct flash_device spi_dev;		/* detected model of SPI flash */
	unsigned int sfdp_dummy, sfdp_dummy_detect;

	struct cortex_m_saved_security saved_security;
	bool accessctrl_dirty;
	uint8_t saved_accessctrl[ACCESSCTRL_SAVE_SIZE]; /* in target byte order */
};

#ifndef LOG_ROM_SYMBOL_DEBUG
#define LOG_ROM_SYMBOL_DEBUG LOG_DEBUG
#endif

static int rp2040_lookup_rom_symbol(struct target *target, uint16_t tag, uint16_t flags, uint16_t *symbol_out)
{
	LOG_ROM_SYMBOL_DEBUG("Looking up ROM symbol '%c%c' in RP2040 table", tag & 0xff, (tag >> 8) & 0xff);
	if (flags != RT_FLAG_FUNC_ARM_SEC && flags != RT_FLAG_DATA) {
		/* Note RT flags do not exist on RP2040, so just sanity check that we
		   are asked for a type of thing that actually exists in the ROM table */
		LOG_ERROR("Only data and Secure Arm functions can be looked up in RP2040 ROM table");
		return ERROR_FAIL;
	}

	uint16_t ptr_to_entry;
	unsigned int offset_magic_to_table_ptr = flags == RT_FLAG_DATA ? 6 : 4;
	int err = target_read_u16(target, BOOTROM_MAGIC_ADDR + offset_magic_to_table_ptr, &ptr_to_entry);
	if (err != ERROR_OK)
		return err;

	uint16_t entry_tag;
	do {
		err = target_read_u16(target, ptr_to_entry, &entry_tag);
		if (err != ERROR_OK)
			return err;

		if (entry_tag == tag) {
			/* 16 bit symbol is next */
			err = target_read_u16(target, ptr_to_entry + 2, symbol_out);
			if (err != ERROR_OK)
				return err;

			LOG_ROM_SYMBOL_DEBUG(" -> found: 0x%04" PRIx16, *symbol_out);
			return ERROR_OK;
		}
		ptr_to_entry += 4;
	} while (entry_tag);
	*symbol_out = 0;
	return ERROR_FAIL;
}

static int rp2350_a0_lookup_symbol(struct target *target, uint16_t tag, uint16_t flags, uint16_t *symbol_out)
{
	LOG_ROM_SYMBOL_DEBUG("Looking up ROM symbol '%c%c' in RP2350 A0 table", tag & 0xff, (tag >> 8) & 0xff);

	/* RP2350 A0 table format is the same as RP2040 except with 16 bits of
	   flags after each 16-bit pointer. We ignore the flags, as each symbol
	   only has one datum associated with it. */

	uint32_t magic_ptr = BOOTROM_MAGIC_ADDR;
	if (flags == RT_FLAG_FUNC_RISCV) {
		/* RP2350 A0 used split function tables for Arm/RISC-V -- not used on
		   any other device or any other version of this device. There is a
		   well-known RISC-V table at the top of ROM, matching the well-known
		   Arm table at the bottom of ROM. */
		magic_ptr = 0x7decu;
	} else if (flags != RT_FLAG_FUNC_ARM_SEC) {
		LOG_WARNING("Ignoring non-default flags for RP2350 A0 lookup, hope you like Secure Arm functions");
	}

	uint16_t ptr_to_entry;
	const unsigned int offset_magic_to_table_ptr = 4;
	int err = target_read_u16(target, magic_ptr + offset_magic_to_table_ptr, &ptr_to_entry);
	if (err != ERROR_OK)
		return err;

	uint16_t entry_tag;
	do {
		err = target_read_u16(target, ptr_to_entry, &entry_tag);
		if (err != ERROR_OK)
			return err;

		if (entry_tag == tag) {
			err = target_read_u16(target, ptr_to_entry + 2, symbol_out);
			if (err != ERROR_OK)
				return err;

			LOG_ROM_SYMBOL_DEBUG(" -> found: 0x%04" PRIx16, *symbol_out);
			return ERROR_OK;
		}
		ptr_to_entry += 6;
	} while (entry_tag);
	*symbol_out = 0;
	return ERROR_FAIL;
}

static int rp2350_lookup_rom_symbol(struct target *target, uint32_t ptr_to_entry,
									uint16_t tag, uint16_t flags, uint16_t *symbol_out)
{
	LOG_ROM_SYMBOL_DEBUG("Looking up ROM symbol '%c%c' in RP2350 A1 table", tag & 0xff, (tag >> 8) & 0xff);

	/* On RP2350 A1, Each entry has a flag bitmap identifying the type of its
	   contents. The entry contains one halfword of data for each set flag
	   bit. There may be both Arm and RISC-V entries under the same tag, or
	   separate Arm Secure/NonSecure entries (or all three, why not). */

	while (true) {
		uint16_t entry_tag, entry_flags;

		int err = target_read_u16(target, ptr_to_entry, &entry_tag);
		if (err != ERROR_OK)
			return err;

		if (entry_tag == 0) {
			*symbol_out = 0;
			return ERROR_FAIL;
		}
		ptr_to_entry += 2;

		err = target_read_u16(target, ptr_to_entry, &entry_flags);
		if (err != ERROR_OK)
			return err;

		ptr_to_entry += 2;

		uint16_t matching_flags = flags & entry_flags;

		if (tag == entry_tag && matching_flags != 0) {
			/* This is our entry, seek to the correct data item and return it. */
			bool is_riscv_func = matching_flags & RT_FLAG_FUNC_RISCV;
			while (!(matching_flags & 1)) {
				if (entry_flags & 1)
					ptr_to_entry += 2;

				matching_flags >>= 1;
				entry_flags >>= 1;
			}
			if (is_riscv_func) {
				/* For RISC-V, the table entry itself is the entry point -- trick
				   to make shared function implementations smaller */
				*symbol_out = ptr_to_entry;
				return ERROR_OK;
			}
			err = target_read_u16(target, ptr_to_entry, symbol_out);
			if (err != ERROR_OK)
				return err;

			LOG_ROM_SYMBOL_DEBUG(" -> found: 0x%04" PRIx16, *symbol_out);
			return ERROR_OK;
		}
		/* Skip past this entry */
		while (entry_flags) {
			if (entry_flags & 1)
				ptr_to_entry += 2;

			entry_flags >>= 1;
		}
	}
}

static int rp2xxx_lookup_rom_symbol(struct target *target, uint16_t tag, uint16_t flags, uint16_t *symbol_out)
{
	uint32_t magic;
	int err = target_read_u32(target, BOOTROM_MAGIC_ADDR, &magic);
	if (err != ERROR_OK)
		return err;

	/* Ignore version */
	magic &= BOOTROM_MAGIC_MASK;

	if (magic == BOOTROM_RP2350_MAGIC) {
		/* Distinguish old-style RP2350 ROM table (A0, and earlier A1 builds)
		   based on position of table -- a high address means it is shared with
		   RISC-V, i.e. new-style. */
		uint16_t table_ptr;
		err = target_read_u16(target, BOOTROM_MAGIC_ADDR + 4, &table_ptr);
		if (err != ERROR_OK)
			return err;
		if (table_ptr < 0x7c00)
			return rp2350_a0_lookup_symbol(target, tag, flags, symbol_out);
		else
			return rp2350_lookup_rom_symbol(target, table_ptr, tag, flags, symbol_out);

	} else if (magic == BOOTROM_RP2040_MAGIC) {
		return rp2040_lookup_rom_symbol(target, tag, flags, symbol_out);
	}
	LOG_ERROR("RP2040/RP2350 BOOT ROM not found");
	return ERROR_FAIL;
}

static int rp2xxx_populate_rom_pointer_cache(struct target *target, struct rp2xxx_flash_bank *priv)
{
	uint16_t symtype_func = is_arm(target_to_arm(target))
							 ? RT_FLAG_FUNC_ARM_SEC : RT_FLAG_FUNC_RISCV;
	int err;
	err = rp2xxx_lookup_rom_symbol(target, FUNC_FLASH_EXIT_XIP,
								   symtype_func, &priv->jump_flash_exit_xip);
	if (err != ERROR_OK) {
		LOG_ERROR("Function FUNC_FLASH_EXIT_XIP not found in RP2xxx ROM");
		return err;
	}

	err = rp2xxx_lookup_rom_symbol(target, FUNC_CONNECT_INTERNAL_FLASH,
								   symtype_func, &priv->jump_connect_internal_flash);
	if (err != ERROR_OK) {
		LOG_ERROR("Function FUNC_CONNECT_INTERNAL_FLASH not found in RP2xxx ROM");
		return err;
	}

	err = rp2xxx_lookup_rom_symbol(target, FUNC_FLASH_RANGE_ERASE, symtype_func, &priv->jump_flash_range_erase);
	if (err != ERROR_OK) {
		LOG_ERROR("Function FUNC_FLASH_RANGE_ERASE not found in RP2xxx ROM");
		return err;
	}

	err = rp2xxx_lookup_rom_symbol(target, FUNC_FLASH_RANGE_PROGRAM, symtype_func, &priv->jump_flash_range_program);
	if (err != ERROR_OK) {
		LOG_ERROR("Function FUNC_FLASH_RANGE_PROGRAM not found in RP2xxx ROM");
		return err;
	}

	err = rp2xxx_lookup_rom_symbol(target, FUNC_FLASH_FLUSH_CACHE, symtype_func, &priv->jump_flush_cache);
	if (err != ERROR_OK) {
		LOG_ERROR("Function FUNC_FLASH_FLUSH_CACHE not found in RP2xxx ROM");
		return err;
	}

	err = rp2xxx_lookup_rom_symbol(target, FUNC_FLASH_ENTER_CMD_XIP, symtype_func, &priv->jump_enter_cmd_xip);
	if (err != ERROR_OK) {
		LOG_ERROR("Function FUNC_FLASH_ENTER_CMD_XIP not found in RP2xxx ROM");
		return err;
	}

	// From this point are optional functions which do not exist on e.g. RP2040
	// or pre-production RP2350 ROM versions:
	if (IS_RP2040(priv->id)) {
		priv->jump_bootrom_reset_state = 0;
		priv->jump_flash_reset_address_trans = 0;
		return ERROR_OK;
	}

	err = rp2xxx_lookup_rom_symbol(target, FUNC_BOOTROM_STATE_RESET, symtype_func, &priv->jump_bootrom_reset_state);
	if (err != ERROR_OK) {
		priv->jump_bootrom_reset_state = 0;
		LOG_WARNING("Function FUNC_BOOTROM_STATE_RESET not found in RP2xxx ROM. (probably an RP2350 A0)");
	}

	if (!is_arm(target_to_arm(target))) {
		err = rp2xxx_lookup_rom_symbol(target, FUNC_BOOTROM_SET_STACK, symtype_func,
									   &priv->jump_bootrom_set_varm_stack);
		if (err != ERROR_OK) {
			priv->jump_bootrom_set_varm_stack = 0;
			LOG_WARNING("Function FUNC_BOOTROM_SET_STACK not found in RP2xxx ROM. (probably an RP2350 A0)");
		}
	}

	err = rp2xxx_lookup_rom_symbol(target, FUNC_FLASH_RESET_ADDRESS_TRANS,
								   symtype_func, &priv->jump_flash_reset_address_trans);
	if (err != ERROR_OK) {
		priv->jump_flash_reset_address_trans = 0;
		LOG_WARNING("Function FUNC_FLASH_RESET_ADDRESS_TRANS not found in RP2xxx ROM. (probably an RP2350 A0)");
	}
	return ERROR_OK;
}

// Call a list of PC + SP + r0-r3 function call tuples with a single OpenOCD
// algorithm invocation, to amortise the algorithm overhead over multiple calls:
static int rp2xxx_call_rom_func_batch(struct target *target, struct rp2xxx_flash_bank *priv,
	struct rp2xxx_rom_call_batch_record *calls, unsigned int n_calls)
{
	// Note + sizeof(uint32_t) is for the null terminator
	unsigned int batch_size = ROM_CALL_BATCH_ALGO_SIZE_BYTES
						+ n_calls * sizeof(struct rp2xxx_rom_call_batch_record)
						+ sizeof(uint32_t);

	if (!priv->ram_algo_space) {
		LOG_ERROR("No RAM code space allocated for ROM call");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	if (priv->ram_algo_space->size < batch_size) {
		LOG_ERROR("RAM code space too small for call batch size of %u", n_calls);
		return ERROR_BUF_TOO_SMALL;
	}

	LOG_TARGET_DEBUG(target, "Calling batch of %u ROM functions:", n_calls);
	for (unsigned int i = 0; i < n_calls; ++i) {
		LOG_DEBUG("  func @ %" PRIx32, calls[i].pc);
		LOG_DEBUG("    sp = %" PRIx32, calls[i].sp);
		for (unsigned int j = 0; j < 4; ++j)
			LOG_DEBUG("    a%u = %" PRIx32, j, calls[i].args[j]);
	}

	if (n_calls <= 0) {
		LOG_DEBUG("Returning early from call of 0 ROM functions");
		return ERROR_OK;
	}

	const uint8_t *algo_code;
	if (is_arm(target_to_arm(target))) {
		if (target_to_arm(target)->arch == ARM_ARCH_V8M) {
			LOG_DEBUG("Using algo: rp2xxx_rom_call_batch_algo_armv8m");
			algo_code = rp2xxx_rom_call_batch_algo_armv8m;
		} else {
			LOG_DEBUG("Using algo: rp2xxx_rom_call_batch_algo_armv6m");
			algo_code = rp2xxx_rom_call_batch_algo_armv6m;
		}
	} else {
		LOG_DEBUG("Using algo: rp2xxx_rom_call_batch_algo_riscv");
		algo_code = rp2xxx_rom_call_batch_algo_riscv;
	}

	uint8_t *batch_bf = malloc(batch_size);
	if (!batch_bf) {
		LOG_ERROR("No memory for batch buffer");
		return ERROR_FAIL;
	}
	memcpy(batch_bf, algo_code, ROM_CALL_BATCH_ALGO_SIZE_BYTES);
	unsigned int words = n_calls * sizeof(struct rp2xxx_rom_call_batch_record)
						/ sizeof(uint32_t);
	target_buffer_set_u32_array(target,
							&batch_bf[ROM_CALL_BATCH_ALGO_SIZE_BYTES], words,
							(const uint32_t *)calls);
	/* Null terminator */
	target_buffer_set_u32(target, &batch_bf[batch_size - sizeof(uint32_t)], 0);

	int err = target_write_buffer(target,
		priv->ram_algo_space->address,
		batch_size,
		batch_bf
	);
	free(batch_bf);

	if (err != ERROR_OK) {
		LOG_ERROR("Failed to write ROM batch algorithm to RAM code space");
		return err;
	}

	// Call into the ROM batch algorithm -- this will in turn call each ROM
	// call specified by the batch records.
	target_addr_t algo_start_addr = priv->ram_algo_space->address;
	target_addr_t algo_end_addr = priv->ram_algo_space->address + rp2xxx_rom_call_batch_algo_bkpt_offset;
	unsigned int algo_timeout_ms = 3000;
	if (is_arm(target_to_arm(target))) {
		struct armv7m_algorithm alg_info;
		alg_info.common_magic = ARMV7M_COMMON_MAGIC;
		alg_info.core_mode = ARM_MODE_THREAD;
		err = target_run_algorithm(target,
			0, NULL,          /* No memory arguments */
			0, NULL,          /* No register arguments */
			algo_start_addr, algo_end_addr,
			algo_timeout_ms,
			&alg_info
		);
	} else {
		// Presumed RISC-V -- there is no RISCV_COMMON_MAGIC on older OpenOCD
		err = target_run_algorithm(target,
			0, NULL,          /* No memory arguments */
			0, NULL,          /* No register arguments */
			algo_start_addr, algo_end_addr,
			algo_timeout_ms,
			NULL              /* Currently no RISC-V-specific algorithm info */
		);
	}
	if (err != ERROR_OK) {
		LOG_ERROR("Failed to call ROM function batch");
		/* This case is hit when loading new ROM images on FPGA, but can also be hit on real
		   hardware if you swap two devices with different ROM versions without restarting OpenOCD: */
		LOG_ROM_SYMBOL_DEBUG("Repopulating ROM address cache after failed ROM call");
		/* We ignore the error on this next call because we have already failed, this is just
		   recovery for the next attempt. */
		(void)rp2xxx_populate_rom_pointer_cache(target, priv);
		return err;
	}
	return ERROR_OK;
}

// Call a single ROM function, using the default algorithm stack.
static int rp2xxx_call_rom_func(struct target *target, struct rp2xxx_flash_bank *priv,
		uint16_t func_offset, uint32_t argdata[], unsigned int n_args)
{
	assert(n_args <= 4); /* only allow register arguments -- capped at just 4 on Arm */

	if (!priv->stack) {
		LOG_ERROR("no stack for flash programming code");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}
	target_addr_t stacktop = priv->stack->address + priv->stack->size;

	struct rp2xxx_rom_call_batch_record call = {
		.pc = func_offset,
		.sp = stacktop
	};
	for (unsigned int i = 0; i < n_args; ++i)
		call.args[i] = argdata[i];

	return rp2xxx_call_rom_func_batch(target, priv, &call, 1);
}

static int rp2350_save_accessctrl(struct target *target, struct rp2xxx_flash_bank *priv)
{
	return target_read_memory(target, ACCESSCTRL_SAVE_BASE, 4, ACCESSCTRL_SAVE_SIZE / 4,
							  priv->saved_accessctrl);
}

static int rp2350_restore_accessctrl(struct target *target, struct rp2xxx_flash_bank *priv)
{
	// Add write passwords to all ACCESSCTRL regs from ACCESSCTRL_GPIO_ROM to the end
	// (exclude not keyed ACCESSCTRL_GPIO_NSMASK0 and ACCESSCTRL_GPIO_NSMASK1
	for (unsigned int i = ACCESSCTRL_GPIO_ROM_OFFSET - ACCESSCTRL_SAVE_BASE;
			 i < ACCESSCTRL_SAVE_SIZE; i += 4)
		target_buffer_set_u32(target, priv->saved_accessctrl + i,
			target_buffer_get_u32(target, priv->saved_accessctrl + i) | ACCESSCTRL_WRITE_PASSWORD);

	return target_write_memory(target, ACCESSCTRL_SAVE_BASE, 4, ACCESSCTRL_SAVE_SIZE / 4,
							   priv->saved_accessctrl);
}

static int rp2350_init_accessctrl(struct target *target, struct rp2xxx_flash_bank *priv)
{
	priv->accessctrl_dirty = false;
	// Attempt to reset ACCESSCTRL, in case Secure access to SRAM has been
	// blocked, which will stop us from loading/running algorithms such as RCP
	// init. (Also ROM, QMI regs are needed later)
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
		LOG_ERROR("ACCESSCTRL is locked, so can't reset permissions. Following steps might fail");
	} else {
		int retval = rp2350_save_accessctrl(target, priv);
		if (retval == ERROR_OK)
			priv->accessctrl_dirty = true;
		// If the ACCESSCTRL backup copy is valid, mark it dirty
		// as we immediately proceed to ACCESSCTRL config reset.
		// Don't fail on save ACCESSCTRL error, not vital for ROM API call

		LOG_DEBUG("Reset ACCESSCTRL permissions via CFGRESET");
		return target_write_u32(target, ACCESSCTRL_CFGRESET_OFFSET, ACCESSCTRL_WRITE_PASSWORD | 1u);
	}
	return ERROR_OK;
}

static int rp2350_init_arm_core0(struct target *target, struct rp2xxx_flash_bank *priv)
{
	// Flash algorithms (and the RCP init stub called by this function) must
	// run in the Secure state, so flip the state now before attempting to
	// execute any code on the core.
	int retval;
	retval = cortex_m_set_secure(target, &priv->saved_security);
	if (retval != ERROR_OK) {
		LOG_ERROR("RP2350 init ARM core: set secure mode failed");
		return retval;
	}

	if (!priv->stack) {
		LOG_ERROR("No stack for flash programming code");
		return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
	}

	if (!priv->ram_algo_space || priv->ram_algo_space->size < sizeof(rcp_init_code)) {
		LOG_ERROR("No algorithm space for rcp_init code");
		return ERROR_BUF_TOO_SMALL;
	}

	int err = target_write_memory(target,
		priv->ram_algo_space->address,
		1,
		sizeof(rcp_init_code),
		rcp_init_code
	);
	if (err != ERROR_OK) {
		LOG_ERROR("Failed to load rcp_init algorithm into RAM");
		return ERROR_FAIL;
	}

	LOG_TARGET_DEBUG(target, "Calling rcp_init, code at " TARGET_ADDR_FMT,
					 priv->ram_algo_space->address);

	/* Actually call the function */
	struct armv7m_algorithm alg_info;
	alg_info.common_magic = ARMV7M_COMMON_MAGIC;
	alg_info.core_mode = ARM_MODE_THREAD;
	err = target_run_algorithm(target,
			0, NULL,          /* No memory arguments */
			0, NULL,          /* No register arguments */
			priv->ram_algo_space->address,
			priv->ram_algo_space->address + rcp_init_code_bkpt_offset,
			1000, /* 1s timeout */
			&alg_info
	);
	if (err != ERROR_OK) {
		LOG_ERROR("Failed to invoke rcp_init");
		return err;
	}

	return err;
}

static int setup_for_raw_flash_cmd(struct target *target, struct rp2xxx_flash_bank *priv)
{
	int err = ERROR_OK;

	if (!priv->stack) {
		/* target_alloc_working_area always allocates multiples of 4 bytes, so no worry about alignment */
		err = target_alloc_working_area(target, RP2XXX_MAX_ALGO_STACK_USAGE, &priv->stack);
		if (err != ERROR_OK) {
			LOG_ERROR("Could not allocate stack for flash programming code -- insufficient space");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	if (!priv->ram_algo_space) {
		err = target_alloc_working_area(target, RP2XXX_MAX_RAM_ALGO_SIZE, &priv->ram_algo_space);
		if (err != ERROR_OK) {
			LOG_ERROR("Could not allocate RAM code space for ROM calls -- insufficient space");
			return ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
		}
	}

	if (IS_RP2350(priv->id)) {
		err = rp2350_init_accessctrl(target, priv);
		if (err != ERROR_OK) {
			LOG_ERROR("Failed to init ACCESSCTRL before ROM call");
			return err;
		}
		if (is_arm(target_to_arm(target))) {
			err = rp2350_init_arm_core0(target, priv);
			if (err != ERROR_OK) {
				LOG_ERROR("Failed to init Arm core 0 before ROM call");
				return err;
			}
		}
		uint32_t reset_args[1] = {
				BOOTROM_STATE_RESET_CURRENT_CORE
		};
		if (!priv->jump_bootrom_reset_state) {
			LOG_WARNING("RP2350 flash: no bootrom_reset_method");
		} else {
			/* This is mainly required to clear varmulet_enclosing_cpu pointers on RISC-V, in case
			   an Arm -> RISC-V call has been interrupted (these pointers are used to handle
			   reentrant calls to the ROM emulator) */
			LOG_DEBUG("Clearing core 0 ROM state");
			err = rp2xxx_call_rom_func(target, priv, priv->jump_bootrom_reset_state,
									   reset_args, ARRAY_SIZE(reset_args));
			if (err != ERROR_OK) {
				LOG_ERROR("RP2350 flash: failed to call reset core state");
				return err;
			}
		}
		if (!is_arm(target_to_arm(target)) && priv->jump_bootrom_set_varm_stack) {
			/* Pass {0, 0} to set_varmulet_user_stack() to enable automatic emulation of Arm APIs
			   using the ROM's default stacks. Usually the bootrom does this before exiting to user
			   code, but it needs to be done manually when the USB bootloader has been interrupted. */
			LOG_DEBUG("Enabling default Arm emulator stacks for RISC-V ROM calls");
			struct working_area *set_stack_mem_args;
			err = target_alloc_working_area(target, 2 * sizeof(uint32_t), &set_stack_mem_args);
			if (err != ERROR_OK) {
				LOG_ERROR("Failed to allocate memory for arguments to set_varmulet_user_stack()");
				return err;
			}

			err = target_write_u32(target, set_stack_mem_args->address, 0);
			if (err == ERROR_OK)
				err = target_write_u32(target, set_stack_mem_args->address + 4, 0);

			if (err != ERROR_OK) {
				LOG_ERROR("Failed to initialise memory arguments for set_varmulet_user_stack()");
				target_free_working_area(target, set_stack_mem_args);
				return err;
			}

			uint32_t set_stack_register_args[1] = {
				set_stack_mem_args->address
			};
			err = rp2xxx_call_rom_func(target, priv, priv->jump_bootrom_set_varm_stack,
									   set_stack_register_args, ARRAY_SIZE(set_stack_register_args));
			target_free_working_area(target, set_stack_mem_args);
			if (err != ERROR_OK) {
				LOG_ERROR("Failed to initialise Arm emulation stacks for RISC-V");
				return err;
			}
		}
	}

	LOG_DEBUG("Connecting flash IOs and issuing XIP exit sequence to flash");
	struct rp2xxx_rom_call_batch_record calls[2] = {
		{
			.pc = priv->jump_connect_internal_flash,
			.sp = priv->stack->address + priv->stack->size
		},
		{
			.pc = priv->jump_flash_exit_xip,
			.sp = priv->stack->address + priv->stack->size
		}
	};
	err = rp2xxx_call_rom_func_batch(target, priv, calls, 2);
	if (err != ERROR_OK) {
		LOG_ERROR("RP2xxx flash: failed to exit flash XIP mode");
		return err;
	}

	return ERROR_OK;
}

static int rp2xxx_invalidate_cache_restore_xip(struct target *target, struct rp2xxx_flash_bank *priv)
{
	// Flash content has changed. We can now do a bit of poking to make
	// the new flash contents visible to us via memory-mapped (XIP) interface
	// in the 0x1... memory region.

	LOG_DEBUG("Flushing flash cache after write behind");

	struct rp2xxx_rom_call_batch_record finishing_calls[2] = {
		{
			.pc = priv->jump_flush_cache,
			.sp = priv->stack->address + priv->stack->size
		},
		{
			.sp = priv->stack->address + priv->stack->size
		},
	};

	int num_finishing_calls = 1;
	// Note on RP2350 it's not *required* to call flash_enter_cmd_xip, since
	// the ROM leaves flash XIPable by default in between direct-mode
	// accesses
	if (IS_RP2040(priv->id)) {
		finishing_calls[num_finishing_calls++].pc = priv->jump_enter_cmd_xip;
	} else if (priv->jump_flash_reset_address_trans) {
		// Note flash_reset_address_trans function does not exist on older devices
		finishing_calls[num_finishing_calls++].pc = priv->jump_flash_reset_address_trans;
	}

	int retval = rp2xxx_call_rom_func_batch(target, priv, finishing_calls, num_finishing_calls);
	if (retval != ERROR_OK)
		LOG_ERROR("RP2xxx: failed to flush flash cache/restore XIP");

	return retval;
}

static void cleanup_after_raw_flash_cmd(struct target *target, struct rp2xxx_flash_bank *priv)
{
	/* OpenOCD is prone to trashing work-area allocations on target state
	   transitions, which leaves us with stale work area pointers in our
	   driver state. Best to clean up our allocations manually after
	   completing each flash call, so we know to make fresh ones next time. */
	LOG_DEBUG("Cleaning up after flash operations");

	if (IS_RP2350(priv->id)) {
		int retval1 = ERROR_OK;
		if (priv->accessctrl_dirty) {
			retval1 = rp2350_restore_accessctrl(target, priv);
			priv->accessctrl_dirty = false;
		}

		int retval2 = ERROR_OK;
		if (is_arm(target_to_arm(target)))
			retval2 = cortex_m_security_restore(target, &priv->saved_security);

		if (retval1 != ERROR_OK || retval2 != ERROR_OK)
			LOG_WARNING("RP2xxx: security state was not restored properly. Debug 'resume' will probably fail, use 'reset' instead");
		/* Don't fail on security restore error, not vital for flash operation */
	}
	if (priv->stack) {
		target_free_working_area(target, priv->stack);
		priv->stack = 0;
	}
	if (priv->ram_algo_space) {
		target_free_working_area(target, priv->ram_algo_space);
		priv->ram_algo_space = 0;
	}
}

static int rp2xxx_flash_write(struct flash_bank *bank, const uint8_t *buffer, uint32_t offset, uint32_t count)
{
	LOG_DEBUG("Writing %" PRIu32 " bytes starting at 0x%" PRIx32, count, offset);

	struct rp2xxx_flash_bank *priv = bank->driver_priv;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	struct working_area *bounce = NULL;

	int err = setup_for_raw_flash_cmd(target, priv);
	if (err != ERROR_OK)
		goto cleanup_and_return;

	unsigned int avail_pages = target_get_working_area_avail(target) / RP2XXX_ROM_API_FIXED_FLASH_PAGE;
	/* We try to allocate working area rounded down to device page size,
	 * al least 1 page, at most the write data size */
	unsigned int chunk_size = MIN(MAX(avail_pages, 1) * RP2XXX_ROM_API_FIXED_FLASH_PAGE, count);
	err = target_alloc_working_area(target, chunk_size, &bounce);
	if (err != ERROR_OK) {
		LOG_ERROR("Could not allocate bounce buffer for flash programming. Can't continue");
		goto cleanup_and_return;
	}

	LOG_DEBUG("Allocated flash bounce buffer @" TARGET_ADDR_FMT, bounce->address);

	while (count > 0) {
		uint32_t write_size = count > chunk_size ? chunk_size : count;
		LOG_DEBUG("Writing %" PRIu32 " bytes to offset 0x%" PRIx32,
				  write_size, offset);
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
		err = rp2xxx_call_rom_func(target, priv, priv->jump_flash_range_program,
								   args, ARRAY_SIZE(args));
		keep_alive();
		if (err != ERROR_OK) {
			LOG_ERROR("Failed to invoke flash programming code on target");
			break;
		}

		buffer += write_size;
		offset += write_size;
		count -= write_size;
	}

cleanup_and_return:
	target_free_working_area(target, bounce);

	/* Don't propagate error or user gets fooled the flash write failed */
	(void)rp2xxx_invalidate_cache_restore_xip(target, priv);

	cleanup_after_raw_flash_cmd(target, priv);
	return err;
}

static int rp2xxx_flash_erase(struct flash_bank *bank, unsigned int first, unsigned int last)
{
	struct rp2xxx_flash_bank *priv = bank->driver_priv;
	struct target *target = bank->target;

	if (target->state != TARGET_HALTED) {
		LOG_ERROR("Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	uint32_t offset_start = bank->sectors[first].offset;
	uint32_t offset_last = bank->sectors[last].offset + bank->sectors[last].size;
	uint32_t length = offset_last - offset_start;
	LOG_DEBUG("erase %" PRIu32 " bytes starting at 0x%" PRIx32,
			  length, offset_start);

	int err = setup_for_raw_flash_cmd(target, priv);
	if (err != ERROR_OK)
		goto cleanup_and_return;

	uint32_t offset_next = offset_start;

	/* Break erase into multiple calls to avoid timeout on large erase. Choose 128k chunk which has
	   fairly low ROM call overhead and empirically seems to avoid the default keep_alive() limit
	   as well as our ROM call timeout. */
	const uint32_t erase_chunk_size = 128 * 1024;

	/* Promote log level for long erases to provide feedback */
	bool requires_loud_prints = offset_last - offset_next >= 2 * erase_chunk_size;
	enum log_levels chunk_log_level = requires_loud_prints ? LOG_LVL_INFO : LOG_LVL_DEBUG;

	while (offset_next < offset_last) {
		uint32_t remaining = offset_last - offset_next;
		uint32_t call_size = remaining < erase_chunk_size ? remaining : erase_chunk_size;
		/* Shorten the first call of a large erase if necessary to align subsequent calls */
		if (offset_next % erase_chunk_size != 0 && call_size == erase_chunk_size)
			call_size = erase_chunk_size - offset_next % erase_chunk_size;

		LOG_CUSTOM_LEVEL(chunk_log_level,
			"  Erase chunk: 0x%08" PRIx32 " -> 0x%08" PRIx32,
			offset_next,
			offset_next + call_size - 1
		);

		/* This ROM function uses the optimal mixture of 4k 20h and 64k D8h erases, without
		   over-erase. This is why we force the flash_bank sector size attribute to 4k even if
		   OpenOCD prefers to give the block size instead. */
		uint32_t args[4] = {
			offset_next,
			call_size,
			65536, /* block_size */
			0xd8   /* block_cmd */
		};

		err = rp2xxx_call_rom_func(target, priv, priv->jump_flash_range_erase, args, ARRAY_SIZE(args));
		keep_alive();

		if (err != ERROR_OK)
			break;

		offset_next += call_size;
	}


cleanup_and_return:
	/* Don't propagate error or user gets fooled the flash erase failed */
	(void)rp2xxx_invalidate_cache_restore_xip(target, priv);

	cleanup_after_raw_flash_cmd(target, priv);
	return err;
}

/* -----------------------------------------------------------------------------
   Driver probing etc */


static int rp2040_ssel_active(struct target *target, bool active)
{
	uint32_t state = active ? RP2040_QSPI_CTRL_OUTOVER_LOW : RP2040_QSPI_CTRL_OUTOVER_HIGH;
	uint32_t val;

	int err = target_read_u32(target, RP2040_QSPI_CTRL, &val);
	if (err != ERROR_OK)
		return err;

	val = (val & ~RP2040_QSPI_CTRL_OUTOVER_MASK) | state;

	err = target_write_u32(target, RP2040_QSPI_CTRL, val);
	if (err != ERROR_OK)
		return err;

	return ERROR_OK;
}

static int rp2040_spi_tx_rx(struct target *target,
		const uint8_t *tx, unsigned int tx_len,
		unsigned int dummy_len,
		uint8_t *rx, unsigned int rx_len)
{
	int retval, retval2;

	retval = rp2040_ssel_active(target, true);
	if (retval != ERROR_OK) {
		LOG_ERROR("QSPI select failed");
		goto deselect;
	}

	unsigned int tx_cnt = 0;
	unsigned int rx_cnt = 0;
	unsigned int xfer_len = tx_len + dummy_len + rx_len;
	while (rx_cnt < xfer_len) {
		int in_flight = tx_cnt - rx_cnt;
		if (tx_cnt < xfer_len && in_flight < 14) {
			uint32_t dr = tx_cnt < tx_len ? tx[tx_cnt] : 0;
			retval = target_write_u32(target, RP2040_SSI_DR0, dr);
			if (retval != ERROR_OK)
				break;

			tx_cnt++;
			continue;
		}
		uint32_t dr;
		retval = target_read_u32(target, RP2040_SSI_DR0, &dr);
		if (retval != ERROR_OK)
			break;

		if (rx_cnt >= tx_len + dummy_len)
			rx[rx_cnt - tx_len - dummy_len] = (uint8_t)dr;

		rx_cnt++;
	}

deselect:
	retval2 = rp2040_ssel_active(target, false);

	if (retval != ERROR_OK) {
		LOG_ERROR("QSPI Tx/Rx failed");
		return retval;
	}
	if (retval2 != ERROR_OK)
		LOG_ERROR("QSPI deselect failed");

	return retval2;
}

static int rp2350_spi_tx_rx(struct target *target,
		const uint8_t *tx, unsigned int tx_len,
		unsigned int dummy_len,
		uint8_t *rx, unsigned int rx_len)
{
	uint32_t direct_csr;
	int retval = target_read_u32(target, RP2350_QMI_DIRECT_CSR, &direct_csr);
	if (retval != ERROR_OK) {
		LOG_ERROR("QMI DIRECT_CSR read failed");
		return retval;
	}
	direct_csr |= RP2350_QMI_DIRECT_CSR_EN | RP2350_QMI_DIRECT_CSR_ASSERT_CS0N;
	retval = target_write_u32(target, RP2350_QMI_DIRECT_CSR, direct_csr);
	if (retval != ERROR_OK) {
		LOG_ERROR("QMI DIRECT mode enable failed");
		goto deselect;
	}

	unsigned int tx_cnt = 0;
	unsigned int rx_cnt = 0;
	unsigned int xfer_len = tx_len + dummy_len + rx_len;
	while (tx_cnt < xfer_len || rx_cnt < rx_len) {
		int in_flight = tx_cnt - tx_len - dummy_len - rx_cnt;
		if (tx_cnt < xfer_len && in_flight < 4) {
			uint32_t tx_cmd;
			if (tx_cnt < tx_len)
				tx_cmd = tx[tx_cnt] | RP2350_QMI_DIRECT_TX_NOPUSH | RP2350_QMI_DIRECT_TX_OE;
			else if (tx_cnt < tx_len + dummy_len)
				tx_cmd = RP2350_QMI_DIRECT_TX_NOPUSH;
			else
				tx_cmd = 0;

			retval = target_write_u32(target, RP2350_QMI_DIRECT_TX, tx_cmd);
			if (retval != ERROR_OK)
				break;

			tx_cnt++;
			continue;
		}
		if (rx_cnt < rx_len) {
			uint32_t dr;
			retval = target_read_u32(target, RP2350_QMI_DIRECT_RX, &dr);
			if (retval != ERROR_OK)
				break;

			rx[rx_cnt] = (uint8_t)dr;
			rx_cnt++;
		}
	}

deselect:
	direct_csr &= ~(RP2350_QMI_DIRECT_CSR_EN | RP2350_QMI_DIRECT_CSR_ASSERT_CS0N);
	int retval2 = target_write_u32(target, RP2350_QMI_DIRECT_CSR, direct_csr);

	if (retval != ERROR_OK) {
		LOG_ERROR("QSPI Tx/Rx failed");
		return retval;
	}
	if (retval2 != ERROR_OK)
		LOG_ERROR("QMI DIRECT mode disable failed");

	return retval2;
}

static int rp2xxx_spi_tx_rx(struct flash_bank *bank,
		const uint8_t *tx, unsigned int tx_len,
		unsigned int dummy_len,
		uint8_t *rx, unsigned int rx_len)
{
	struct rp2xxx_flash_bank *priv = bank->driver_priv;
	struct target *target = bank->target;

	if (IS_RP2040(priv->id))
		return rp2040_spi_tx_rx(target, tx, tx_len, dummy_len, rx, rx_len);
	else if (IS_RP2350(priv->id))
		return rp2350_spi_tx_rx(target, tx, tx_len, dummy_len, rx, rx_len);
	else
		return ERROR_FAIL;
}

static int rp2xxx_read_sfdp_block(struct flash_bank *bank, uint32_t addr,
		unsigned int words, uint32_t *buffer)
{
	struct rp2xxx_flash_bank *priv = bank->driver_priv;

	uint8_t cmd[4] = { SPIFLASH_READ_SFDP };
	uint8_t data[4 * words + priv->sfdp_dummy_detect];

	h_u24_to_be(&cmd[1], addr);

	int retval = rp2xxx_spi_tx_rx(bank, cmd, sizeof(cmd), priv->sfdp_dummy,
								  data, 4 * words + priv->sfdp_dummy_detect);
	if (retval != ERROR_OK)
		return retval;

	if (priv->sfdp_dummy_detect) {
		for (unsigned int i = 0; i < priv->sfdp_dummy_detect; i++)
			if (le_to_h_u32(&data[i]) == SFDP_MAGIC) {
				priv->sfdp_dummy_detect = 0;
				priv->sfdp_dummy = i;
				break;
			}
		for (unsigned int i = 0; i < words; i++)
			buffer[i] = le_to_h_u32(&data[4 * i + priv->sfdp_dummy]);
	} else {
		for (unsigned int i = 0; i < words; i++)
			buffer[i] = le_to_h_u32(&data[4 * i]);
	}
	return retval;
}

static int rp2xxx_flash_probe(struct flash_bank *bank)
{
	struct rp2xxx_flash_bank *priv = bank->driver_priv;
	struct target *target = bank->target;

	int retval = target_read_u32(target, RP2XXX_SYSINFO_CHIP_ID, &priv->id);
	if (retval != ERROR_OK) {
		LOG_ERROR("SYSINFO CHIP_ID read failed");
		return retval;
	}
	if (!IS_RP2040(priv->id) && !IS_RP2350(priv->id)) {
		LOG_ERROR("Unknown SYSINFO CHIP_ID 0x%08" PRIx32, priv->id);
		return ERROR_FLASH_BANK_INVALID;
	}

	retval = rp2xxx_populate_rom_pointer_cache(target, priv);
	if (retval != ERROR_OK)
		return retval;

	/* the Boot ROM flash_range_program() routine requires page alignment */
	bank->write_start_alignment = RP2XXX_ROM_API_FIXED_FLASH_PAGE;
	bank->write_end_alignment = RP2XXX_ROM_API_FIXED_FLASH_PAGE;

	uint32_t flash_id = 0;
	if (priv->size_override) {
		priv->spi_dev.name = "size override";
		LOG_DEBUG("SPI flash autodetection disabled, using configured size");
	} else {
		if (target->state != TARGET_HALTED) {
			LOG_ERROR("Target not halted");
			return ERROR_TARGET_NOT_HALTED;
		}

		bank->size = 0;

		(void)setup_for_raw_flash_cmd(target, priv);
		/* ignore error, flash size detection could work anyway */

		const uint8_t cmd[] = { SPIFLASH_READ_ID };
		uint8_t data[3];
		retval = rp2xxx_spi_tx_rx(bank, cmd, sizeof(cmd), 0, data, sizeof(data));
		if (retval == ERROR_OK) {
			flash_id = le_to_h_u24(data);

			/* search for a SPI flash Device ID match */
			for (const struct flash_device *p = flash_devices; p->name ; p++) {
				if (p->device_id == flash_id) {
					priv->spi_dev = *p;
					bank->size = p->size_in_bytes;
					break;
				}
			}
		}

		if (bank->size == 0) {
			priv->sfdp_dummy_detect = 8;
			priv->sfdp_dummy = 0;
			retval = spi_sfdp(bank, &priv->spi_dev, &rp2xxx_read_sfdp_block);
			if (retval == ERROR_OK)
				bank->size = priv->spi_dev.size_in_bytes;
		}

		cleanup_after_raw_flash_cmd(target, priv);
	}

	snprintf(priv->dev_name, sizeof(priv->dev_name), "%s rev %u",
			 IS_RP2350(priv->id) ? "RP2350" : "RP2040",
			 RP2XXX_CHIP_ID_REVISION(priv->id));

	if (bank->size == 0) {
		LOG_ERROR("%s, QSPI Flash id = 0x%06" PRIx32 " not recognised",
				  priv->dev_name, flash_id);
		return ERROR_FLASH_BANK_INVALID;
	}

	bank->num_sectors = bank->size / RP2XXX_ROM_API_FIXED_FLASH_SECTOR;

	if (priv->size_override) {
		LOG_INFO("%s, QSPI Flash size override = %u KiB in %u sectors",
				 priv->dev_name, bank->size / 1024, bank->num_sectors);
	} else {
		LOG_INFO("%s, QSPI Flash %s id = 0x%06" PRIx32 " size = %u KiB in %u sectors",
				 priv->dev_name, priv->spi_dev.name, flash_id,
				 bank->size / 1024, bank->num_sectors);
	}

	free(bank->sectors);
	bank->sectors = alloc_block_array(0, RP2XXX_ROM_API_FIXED_FLASH_SECTOR, bank->num_sectors);
	if (!bank->sectors)
		return ERROR_FAIL;

	priv->probed = true;

	return ERROR_OK;
}

static int rp2xxx_flash_auto_probe(struct flash_bank *bank)
{
	struct rp2xxx_flash_bank *priv = bank->driver_priv;

	if (priv->probed)
		return ERROR_OK;

	return rp2xxx_flash_probe(bank);
}

/* -----------------------------------------------------------------------------
   Driver boilerplate */

FLASH_BANK_COMMAND_HANDLER(rp2xxx_flash_bank_command)
{
	struct rp2xxx_flash_bank *priv;
	priv = calloc(1, sizeof(struct rp2xxx_flash_bank));
	priv->size_override = bank->size != 0;

	/* Set up driver_priv */
	bank->driver_priv = priv;

	return ERROR_OK;
}


COMMAND_HANDLER(rp2xxx_rom_api_call_handler)
{
	if (CMD_ARGC < 1)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *target = get_current_target(CMD->ctx);

	struct flash_bank *bank;
	for (bank = flash_bank_list(); bank; bank = bank->next) {
		if (bank->driver != &rp2xxx_flash)
			continue;

		if (bank->target == target)
			break;
	}

	if (!bank) {
		command_print(CMD, "[%s] No associated RP2xxx flash bank found",
					  target_name(target));
		return ERROR_FAIL;
	}

	int retval = rp2xxx_flash_auto_probe(bank);
	if (retval != ERROR_OK) {
		command_print(CMD, "auto_probe failed");
		return retval;
	}

	uint16_t tag = MAKE_TAG(CMD_ARGV[0][0], CMD_ARGV[0][1]);

	uint32_t args[4] = { 0 };
	for (unsigned int i = 0; i + 1 < CMD_ARGC && i < ARRAY_SIZE(args); i++)
		COMMAND_PARSE_NUMBER(u32, CMD_ARGV[i + 1], args[i]);

	if (target->state != TARGET_HALTED) {
		command_print(CMD, "Target not halted");
		return ERROR_TARGET_NOT_HALTED;
	}

	struct rp2xxx_flash_bank *priv = bank->driver_priv;
	retval = setup_for_raw_flash_cmd(target, priv);
	if (retval != ERROR_OK)
		goto cleanup_and_return;

	uint16_t symtype_func = is_arm(target_to_arm(target))
				 ? RT_FLAG_FUNC_ARM_SEC : RT_FLAG_FUNC_RISCV;

	uint16_t fc;
	retval = rp2xxx_lookup_rom_symbol(target, tag, symtype_func, &fc);
	if (retval != ERROR_OK) {
		command_print(CMD, "Function %.2s not found in RP2xxx ROM",
					  CMD_ARGV[0]);
		goto cleanup_and_return;
	}

	/* command_print() output gets lost if the command is called
	 * in an event handler, use LOG_INFO instead */
	LOG_INFO("RP2xxx ROM API function %.2s @ %04" PRIx16, CMD_ARGV[0], fc);

	retval = rp2xxx_call_rom_func(target, priv, fc, args, ARRAY_SIZE(args));
	if (retval != ERROR_OK)
		command_print(CMD, "RP2xxx ROM API call failed");

cleanup_and_return:
	cleanup_after_raw_flash_cmd(target, priv);
	return retval;
}

COMMAND_HANDLER(rp2xxx_switch_target_handler)
{
	if (CMD_ARGC != 2)
		return ERROR_COMMAND_SYNTAX_ERROR;

	struct target *old_target = get_target(CMD_ARGV[0]);
	if (!old_target) {
		command_print(CMD, "Unrecognised old target %s", CMD_ARGV[0]);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	struct target *new_target = get_target(CMD_ARGV[1]);
	if (!new_target) {
		command_print(CMD, "Unrecognised new target %s", CMD_ARGV[1]);
		return ERROR_COMMAND_ARGUMENT_INVALID;
	}

	struct flash_bank *bank;
	for (bank = flash_bank_list(); bank; bank = bank->next) {
		if (bank->driver == &rp2xxx_flash) {
			if (bank->target == old_target) {
				bank->target = new_target;
				struct rp2xxx_flash_bank *priv = bank->driver_priv;
				priv->probed = false;
				return ERROR_OK;
			} else if (bank->target == new_target) {
				return ERROR_OK;
			}
		}
	}

	command_print(CMD, "Neither old nor new target %s found in flash bank list",
				  CMD_ARGV[0]);
	return ERROR_FAIL;
}

static const struct command_registration rp2xxx_exec_command_handlers[] = {
	{
		.name = "rom_api_call",
		.mode = COMMAND_EXEC,
		.help = "arbitrary ROM API call",
		.usage = "fc [p0 [p1 [p2 [p3]]]]",
		.handler = rp2xxx_rom_api_call_handler,
	},
	{
		.name = "_switch_target",
		.mode = COMMAND_EXEC,
		.help = "internal use",
		.usage = "old_target new_target",
		.handler = rp2xxx_switch_target_handler,
	},
	COMMAND_REGISTRATION_DONE
};

static const struct command_registration rp2xxx_command_handler[] = {
	{
		.name = "rp2xxx",
		.mode = COMMAND_ANY,
		.help = "rp2xxx flash controller commands",
		.usage = "",
		.chain = rp2xxx_exec_command_handlers,
	},
	COMMAND_REGISTRATION_DONE
};

const struct flash_driver rp2xxx_flash = {
	.name = "rp2xxx",
	.commands = rp2xxx_command_handler,
	.flash_bank_command = rp2xxx_flash_bank_command,
	.erase =  rp2xxx_flash_erase,
	.write = rp2xxx_flash_write,
	.read = default_flash_read,
	.probe = rp2xxx_flash_probe,
	.auto_probe = rp2xxx_flash_auto_probe,
	.erase_check = default_flash_blank_check,
	.free_driver_priv = default_flash_free_driver_priv
};
