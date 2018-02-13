/*
 * Copyright(c) 2013-2016 Intel Corporation.
 *
 * Adrian Burns (adrian.burns@intel.com)
 * Thomas Faust (thomas.faust@intel.com)
 * Ivan De Cesaris (ivan.de.cesaris@intel.com)
 * Julien Carreno (julien.carreno@intel.com)
 * Jeffrey Maxwell (jeffrey.r.maxwell@intel.com)
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
 * General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Contact Information:
 * Intel Corporation
 */

/*
 * @file
 * This is the interface to the x86 32 bit memory and breakpoint operations.
 */

#ifndef OPENOCD_TARGET_X86_32_COMMON_H
#define OPENOCD_TARGET_X86_32_COMMON_H

#include <jtag/jtag.h>
#include <helper/command.h>
#include <helper/types.h>

extern const struct command_registration x86_32_command_handlers[];

/* for memory access */
#define BYTE			1
#define WORD			2
#define DWORD			4

#define EFLAGS_TF		((uint32_t)0x00000100) /* Trap Flag */
#define EFLAGS_IF		((uint32_t)0x00000200) /* Interrupt Flag */
#define EFLAGS_RF		((uint32_t)0x00010000) /* Resume Flag */
#define EFLAGS_VM86		((uint32_t)0x00020000) /* Virtual 8086 Mode */

#define CSAR_DPL		((uint32_t)0x00006000)
#define CSAR_D			((uint32_t)0x00400000)
#define SSAR_DPL		((uint32_t)0x00006000)

#define CR0_PE			((uint32_t)0x00000001) /* Protected Mode Enable */
#define CR0_NW			((uint32_t)0x20000000) /* Non Write-Through */
#define CR0_CD			((uint32_t)0x40000000) /* Cache Disable */
#define CR0_PG			((uint32_t)0x80000000) /* Paging Enable */

/* TODO - move back to PM specific file */
#define PM_DR6			((uint32_t)0xFFFF0FF0)

#define DR6_BRKDETECT_0		((uint32_t)0x00000001) /* B0 through B3 */
#define DR6_BRKDETECT_1		((uint32_t)0x00000002) /* breakpoint condition detected */
#define DR6_BRKDETECT_2		((uint32_t)0x00000004)
#define DR6_BRKDETECT_3		((uint32_t)0x00000008)

enum {
	/* general purpose registers */
	EAX = 0,
	ECX,
	EDX,
	EBX,
	ESP,
	EBP,
	ESI,
	EDI,
	/* instruction pointer & flags */
	EIP,
	EFLAGS,

	/* segment registers */
	CS,
	SS,
	DS,
	ES,
	FS,
	GS,

	/* floating point unit registers */
	ST0,
	ST1,
	ST2,
	ST3,
	ST4,
	ST5,
	ST6,
	ST7,
	FCTRL,
	FSTAT,
	FTAG,
	FISEG,
	FIOFF,
	FOSEG,
	FOOFF,
	FOP,

	/* control registers */
	CR0,
	CR2,
	CR3,
	CR4,

	/* debug registers */
	DR0,
	DR1,
	DR2,
	DR3,
	DR6,
	DR7,

	/* descriptor tables */
	IDTB,
	IDTL,
	IDTAR,
	GDTB,
	GDTL,
	GDTAR,
	TR,
	LDTR,
	LDTB,
	LDTL,
	LDTAR,

	/* segment registers */
	CSB,
	CSL,
	CSAR,
	DSB,
	DSL,
	DSAR,
	ESB,
	ESL,
	ESAR,
	FSB,
	FSL,
	FSAR,
	GSB,
	GSL,
	GSAR,
	SSB,
	SSL,
	SSAR,
	TSSB,
	TSSL,
	TSSAR,

	/* PM control reg */
	PMCR,
};

#define X86_32_COMMON_MAGIC 0x86328632

enum {
	/* memory read/write */
	MEMRDB32 = 0,
	MEMRDB16,
	MEMRDH32,
	MEMRDH16,
	MEMRDW32,
	MEMRDW16,
	MEMWRB32,
	MEMWRB16,
	MEMWRH32,
	MEMWRH16,
	MEMWRW32,
	MEMWRW16,
	/* IO read/write */
	IORDB32,
	IORDB16,
	IORDH32,
	IORDH16,
	IORDW32,
	IORDW16,
	IOWRB32,
	IOWRB16,
	IOWRH32,
	IOWRH16,
	IOWRW32,
	IOWRW16,
	/* lakemont1 core shadow ram access opcodes */
	SRAMACCESS,
	SRAM2PDR,
	PDR2SRAM,
	WBINVD,
};

enum x86_core_type {
	LMT1,
	LMT3_5
};

struct swbp_mem_patch {
	uint8_t orig_byte;
	uint32_t swbp_unique_id;
	uint32_t physaddr;
	struct swbp_mem_patch *next;
};

/* TODO - probemode specific - consider removing */
#define NUM_PM_REGS		18 /* regs used in save/restore */

struct x86_32_common {
	uint32_t common_magic;
	void *arch_info;
	enum x86_core_type core_type;
	struct reg_cache *cache;
	struct jtag_tap *curr_tap;
	uint32_t stored_pc;
	int forced_halt_for_reset;
	int flush;

	/* pm_regs are for probemode save/restore state */
	uint32_t pm_regs[NUM_PM_REGS];

	/* working area for fastdata access */
	struct working_area *fast_data_area;

	int num_hw_bpoints;
	struct x86_32_dbg_reg *hw_break_list;
	struct swbp_mem_patch *swbbp_mem_patch_list;

	/* core probemode implementation dependent functions */
	uint8_t (*get_num_user_regs)(struct target *t);
	bool (*is_paging_enabled)(struct target *t);
	int (*disable_paging)(struct target *t);
	int (*enable_paging)(struct target *t);
	bool (*sw_bpts_supported)(struct target *t);
	int (*transaction_status)(struct target *t);
	int (*submit_instruction)(struct target *t, int num);
	int (*read_hw_reg)(struct target *t, int reg, uint32_t *regval, uint8_t cache);
	int (*write_hw_reg)(struct target *t, int reg,
				uint32_t regval, uint8_t cache);

	/* register cache to processor synchronization */
	int (*read_hw_reg_to_cache)(struct target *target, int num);
	int (*write_hw_reg_from_cache)(struct target *target, int num);
};

static inline struct x86_32_common *
target_to_x86_32(struct target *target)
{
	return target->arch_info;
}
bool check_not_halted(const struct target *t);

/* breakpoint defines */
#define MAX_DEBUG_REGS		4
#define SW_BP_OPCODE		0xf1
#define MAX_SW_BPTS		20

struct x86_32_dbg_reg {
	int used;
	uint32_t bp_value;
};

#define DR7_G_ENABLE_SHIFT		1
#define DR7_ENABLE_SIZE			2 /* 2 bits per debug reg */
#define DR7_RW_SHIFT			16
#define DR7_LENGTH_SHIFT		18
#define DR7_RW_LEN_SIZE			4
#define DR7_BP_EXECUTE			0 /* 00 - only on instruction execution*/
#define DR7_BP_WRITE			1 /* 01 - only on data writes */
/*#define DR7_RW_IORW			2 UNSUPPORTED 10 - an I/O read and I/O write */
#define DR7_BP_READWRITE		3 /* on data read or data write */
#define DR7_BP_LENGTH_1			0 /* 00 - 1 byte length */
#define DR7_BP_LENGTH_2			1 /* 01 - 2 byte length */
#define DR7_BP_LENGTH_4			3 /* 11 - 4 byte length */

#define DR7_GLOBAL_ENABLE(val, regnum) \
	(val |= (1 << (DR7_G_ENABLE_SHIFT + (DR7_ENABLE_SIZE * (regnum)))))

#define DR7_GLOBAL_DISABLE(val, regnum) \
	(val &= ~(3 << (DR7_ENABLE_SIZE * (regnum))))

#define DR7_BP_FREE(val, regnum) \
	((val & (3 << (DR7_ENABLE_SIZE * (regnum)))) == 0)

#define DR7_RESET_RWLEN_BITS(val, regnum) \
	(val &= ~(0x0f << (DR7_RW_SHIFT + DR7_RW_LEN_SIZE * (regnum))))

#define DR7_SET_EXE(val, regnum) \
	(val &= ~(0x0f << (DR7_RW_SHIFT + DR7_RW_LEN_SIZE * (regnum))))

#define DR7_SET_WRITE(val, regnum) \
	(val |= (DR7_BP_WRITE << (DR7_RW_SHIFT + DR7_RW_LEN_SIZE * (regnum))))

#define DR7_SET_ACCESS(val, regnum) \
	(val |= (DR7_BP_READWRITE << (DR7_RW_SHIFT + DR7_RW_LEN_SIZE * (regnum))))

#define DR7_SET_LENGTH(val, regnum, len) \
	(val |= (len == 1) ? (DR7_BP_LENGTH_1 << (DR7_LENGTH_SHIFT + DR7_RW_LEN_SIZE * (regnum))) : \
	(len == 2) ? (DR7_BP_LENGTH_2 << (DR7_LENGTH_SHIFT + DR7_RW_LEN_SIZE * (regnum))) : \
	(DR7_BP_LENGTH_4 << (DR7_LENGTH_SHIFT + DR7_RW_LEN_SIZE * (regnum))))

/* public interface */
int x86_32_get_gdb_reg_list(struct target *t,
			struct reg **reg_list[], int *reg_list_size,
			enum target_register_class reg_class);
int x86_32_common_init_arch_info(struct target *target,
			struct x86_32_common *x86_32);
int x86_32_common_mmu(struct target *t, int *enabled);
int x86_32_common_virt2phys(struct target *t, target_addr_t address, target_addr_t *physical);
int x86_32_common_read_phys_mem(struct target *t, target_addr_t phys_address,
			uint32_t size, uint32_t count, uint8_t *buffer);
int x86_32_common_write_phys_mem(struct target *t, target_addr_t phys_address,
			uint32_t size, uint32_t count, const uint8_t *buffer);
int x86_32_common_read_memory(struct target *t, target_addr_t addr,
			uint32_t size, uint32_t count, uint8_t *buf);
int x86_32_common_write_memory(struct target *t, target_addr_t addr,
			uint32_t size, uint32_t count, const uint8_t *buf);
int x86_32_common_read_io(struct target *t, uint32_t addr,
			uint32_t size, uint8_t *buf);
int x86_32_common_write_io(struct target *t, uint32_t addr,
			uint32_t size, const uint8_t *buf);
int x86_32_common_add_breakpoint(struct target *t, struct breakpoint *bp);
int x86_32_common_remove_breakpoint(struct target *t, struct breakpoint *bp);
int x86_32_common_add_watchpoint(struct target *t, struct watchpoint *wp);
int x86_32_common_remove_watchpoint(struct target *t, struct watchpoint *wp);
void x86_32_common_reset_breakpoints_watchpoints(struct target *t);

#endif /* OPENOCD_TARGET_X86_32_COMMON_H */
