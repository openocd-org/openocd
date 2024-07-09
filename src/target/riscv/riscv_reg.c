// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "gdb_regs.h"
#include "riscv.h"
#include "riscv_reg.h"
#include "riscv_reg_impl.h"
/**
 * TODO: Currently `reg->get/set` is implemented in terms of
 * `riscv_get/set_register`.  However, the intention behind
 * `riscv_get/set_register` is to work with the cache, therefore it accesses
 * and modifyes register cache directly.  The idea is to implement
 * `riscv_get/set_register` in terms of `riscv_reg_impl_cache_entry` and
 * `reg->get/set`.
 * Once this is done, the following includes should be removed.
 */
#include "debug_defines.h"
#include "riscv-011.h"
#include "riscv-013.h"
#include "field_helpers.h"

static const char * const default_reg_names[GDB_REGNO_COUNT] = {
	[GDB_REGNO_ZERO] = "zero",
	[GDB_REGNO_RA] = "ra",
	[GDB_REGNO_SP] = "sp",
	[GDB_REGNO_GP] = "gp",
	[GDB_REGNO_TP] = "tp",
	[GDB_REGNO_T0] = "t0",
	[GDB_REGNO_T1] = "t1",
	[GDB_REGNO_T2] = "t2",
	[GDB_REGNO_FP] = "fp",
	[GDB_REGNO_S1] = "s1",
	[GDB_REGNO_A0] = "a0",
	[GDB_REGNO_A1] = "a1",
	[GDB_REGNO_A2] = "a2",
	[GDB_REGNO_A3] = "a3",
	[GDB_REGNO_A4] = "a4",
	[GDB_REGNO_A5] = "a5",
	[GDB_REGNO_A6] = "a6",
	[GDB_REGNO_A7] = "a7",
	[GDB_REGNO_S2] = "s2",
	[GDB_REGNO_S3] = "s3",
	[GDB_REGNO_S4] = "s4",
	[GDB_REGNO_S5] = "s5",
	[GDB_REGNO_S6] = "s6",
	[GDB_REGNO_S7] = "s7",
	[GDB_REGNO_S8] = "s8",
	[GDB_REGNO_S9] = "s9",
	[GDB_REGNO_S10] = "s10",
	[GDB_REGNO_S11] = "s11",
	[GDB_REGNO_T3] = "t3",
	[GDB_REGNO_T4] = "t4",
	[GDB_REGNO_T5] = "t5",
	[GDB_REGNO_T6] = "t6",
	[GDB_REGNO_PC] = "pc",
	[GDB_REGNO_CSR0] = "csr0",
	[GDB_REGNO_PRIV] = "priv",
	[GDB_REGNO_FT0] = "ft0",
	[GDB_REGNO_FT1] = "ft1",
	[GDB_REGNO_FT2] = "ft2",
	[GDB_REGNO_FT3] = "ft3",
	[GDB_REGNO_FT4] = "ft4",
	[GDB_REGNO_FT5] = "ft5",
	[GDB_REGNO_FT6] = "ft6",
	[GDB_REGNO_FT7] = "ft7",
	[GDB_REGNO_FS0] = "fs0",
	[GDB_REGNO_FS1] = "fs1",
	[GDB_REGNO_FA0] = "fa0",
	[GDB_REGNO_FA1] = "fa1",
	[GDB_REGNO_FA2] = "fa2",
	[GDB_REGNO_FA3] = "fa3",
	[GDB_REGNO_FA4] = "fa4",
	[GDB_REGNO_FA5] = "fa5",
	[GDB_REGNO_FA6] = "fa6",
	[GDB_REGNO_FA7] = "fa7",
	[GDB_REGNO_FS2] = "fs2",
	[GDB_REGNO_FS3] = "fs3",
	[GDB_REGNO_FS4] = "fs4",
	[GDB_REGNO_FS5] = "fs5",
	[GDB_REGNO_FS6] = "fs6",
	[GDB_REGNO_FS7] = "fs7",
	[GDB_REGNO_FS8] = "fs8",
	[GDB_REGNO_FS9] = "fs9",
	[GDB_REGNO_FS10] = "fs10",
	[GDB_REGNO_FS11] = "fs11",
	[GDB_REGNO_FT8] = "ft8",
	[GDB_REGNO_FT9] = "ft9",
	[GDB_REGNO_FT10] = "ft10",
	[GDB_REGNO_FT11] = "ft11",

	#define DECLARE_CSR(csr_name, number)[(number) + GDB_REGNO_CSR0] = #csr_name,
	#include "encoding.h"
	#undef DECLARE_CSR
};

static void free_custom_register_names(struct target *target)
{
	RISCV_INFO(info);

	if (!info->custom_register_names.reg_names)
		return;

	for (unsigned int i = 0; i < info->custom_register_names.num_entries; i++)
		free(info->custom_register_names.reg_names[i]);
	free(info->custom_register_names.reg_names);
	info->custom_register_names.reg_names = NULL;
}

static void free_reg_names(struct target *target)
{
	RISCV_INFO(info);

	if (!info->reg_names)
		return;

	for (unsigned int i = 0; i < GDB_REGNO_COUNT; ++i)
		free(info->reg_names[i]);
	free(info->reg_names);
	info->reg_names = NULL;

	free_custom_register_names(target);
}

static char *init_reg_name(const char *name)
{
	const int size_buf = strlen(name) + 1;

	char * const buf = calloc(size_buf, sizeof(char));
	if (!buf) {
		LOG_ERROR("Failed to allocate memory for a register name.");
		return NULL;
	}
	strcpy(buf, name);
	return buf;
}

static void init_custom_csr_names(const struct target *target)
{
	RISCV_INFO(info);
	range_list_t *entry;

	list_for_each_entry(entry, &info->expose_csr, list) {
		if (!entry->name)
			continue;
		assert(entry->low == entry->high);
		const unsigned int regno = entry->low + GDB_REGNO_CSR0;
		assert(regno <= GDB_REGNO_CSR4095);
		if (info->reg_names[regno])
			return;
		info->reg_names[regno] = init_reg_name(entry->name);
	}
}

static char *init_reg_name_with_prefix(const char *name_prefix,
	unsigned int num)
{
	const int size_buf = snprintf(NULL, 0, "%s%d", name_prefix, num) + 1;

	char * const buf = calloc(size_buf, sizeof(char));
	if (!buf) {
		LOG_ERROR("Failed to allocate memory for a register name.");
		return NULL;
	}
	int result = snprintf(buf, size_buf, "%s%d", name_prefix, num);
	assert(result > 0 && result <= (size_buf - 1));
	return buf;
}

const char *riscv_reg_gdb_regno_name(const struct target *target, enum gdb_regno regno)
{
	RISCV_INFO(info);

	if (regno >= GDB_REGNO_COUNT) {
		assert(info->custom_register_names.reg_names);
		assert(regno - GDB_REGNO_COUNT <= info->custom_register_names.num_entries);
		return info->custom_register_names.reg_names[regno - GDB_REGNO_COUNT];
	}

	if (!info->reg_names)
		info->reg_names = calloc(GDB_REGNO_COUNT, sizeof(char *));

	if (info->reg_names[regno])
		return info->reg_names[regno];
	if (default_reg_names[regno])
		return default_reg_names[regno];
	if (regno <= GDB_REGNO_XPR31) {
		info->reg_names[regno] = init_reg_name_with_prefix("x", regno - GDB_REGNO_ZERO);
		return info->reg_names[regno];
	}
	if (regno <= GDB_REGNO_V31 && regno >= GDB_REGNO_V0) {
		info->reg_names[regno] = init_reg_name_with_prefix("v", regno - GDB_REGNO_V0);
		return info->reg_names[regno];
	}
	if (regno >= GDB_REGNO_CSR0 && regno <= GDB_REGNO_CSR4095) {
		init_custom_csr_names(target);
		info->reg_names[regno] = init_reg_name_with_prefix("csr", regno - GDB_REGNO_CSR0);
		return info->reg_names[regno];
	}
	assert(!"Encountered uninitialized entry in reg_names table");

	return NULL;
}

struct target *riscv_reg_impl_get_target(const struct reg *reg)
{
	assert(riscv_reg_impl_is_initialized(reg));
	return ((const riscv_reg_info_t *)reg->arch_info)->target;
}

static struct reg_feature *gdb_regno_feature(uint32_t regno)
{
	if (regno <= GDB_REGNO_XPR31 || regno == GDB_REGNO_PC) {
		static struct reg_feature feature_cpu = {
			.name = "org.gnu.gdb.riscv.cpu"
		};
		return &feature_cpu;
	}
	if ((regno >= GDB_REGNO_FPR0 && regno <= GDB_REGNO_FPR31) ||
			regno == GDB_REGNO_FFLAGS ||
			regno == GDB_REGNO_FRM ||
			regno ==  GDB_REGNO_FCSR) {
		static struct reg_feature feature_fpu = {
			.name = "org.gnu.gdb.riscv.fpu"
		};
		return &feature_fpu;
	}
	if (regno >= GDB_REGNO_V0 && regno <= GDB_REGNO_V31) {
		static struct reg_feature feature_vector = {
			.name = "org.gnu.gdb.riscv.vector"
		};
		return &feature_vector;
	}
	if (regno >= GDB_REGNO_CSR0 && regno <= GDB_REGNO_CSR4095) {
		static struct reg_feature feature_csr = {
			.name = "org.gnu.gdb.riscv.csr"
		};
		return &feature_csr;
	}
	if (regno == GDB_REGNO_PRIV) {
		static struct reg_feature feature_virtual = {
			.name = "org.gnu.gdb.riscv.virtual"
		};
		return &feature_virtual;
	}
	assert(regno >= GDB_REGNO_COUNT);
	static struct reg_feature feature_custom = {
		.name = "org.gnu.gdb.riscv.custom"
	};
	return &feature_custom;
}

static bool gdb_regno_caller_save(uint32_t regno)
{
	return regno <= GDB_REGNO_XPR31 ||
		regno == GDB_REGNO_PC ||
		(regno >= GDB_REGNO_FPR0 && regno <= GDB_REGNO_FPR31);
}

static struct reg_data_type *gdb_regno_reg_data_type(const struct target *target,
		uint32_t regno)
{
	if (regno >= GDB_REGNO_FPR0 && regno <= GDB_REGNO_FPR31) {
		static struct reg_data_type type_ieee_single = {
			.type = REG_TYPE_IEEE_SINGLE,
			.id = "ieee_single"
		};
		static struct reg_data_type type_ieee_double = {
			.type = REG_TYPE_IEEE_DOUBLE,
			.id = "ieee_double"
		};
		static struct reg_data_type_union_field single_double_fields[] = {
			{"float", &type_ieee_single, single_double_fields + 1},
			{"double", &type_ieee_double, NULL},
		};
		static struct reg_data_type_union single_double_union = {
			.fields = single_double_fields
		};
		static struct reg_data_type type_ieee_single_double = {
			.type = REG_TYPE_ARCH_DEFINED,
			.id = "FPU_FD",
			.type_class = REG_TYPE_CLASS_UNION,
			{.reg_type_union = &single_double_union}
		};
		return riscv_supports_extension(target, 'D') ?
			&type_ieee_single_double :
			&type_ieee_single;
	}
	if (regno >= GDB_REGNO_V0 && regno <= GDB_REGNO_V31) {
		RISCV_INFO(info);
		return &info->type_vector;
	}
	return NULL;
}

static const char *gdb_regno_group(uint32_t regno)
{
	if (regno <= GDB_REGNO_XPR31 ||
			regno == GDB_REGNO_PC ||
			regno == GDB_REGNO_PRIV)
		return "general";
	if ((regno >= GDB_REGNO_FPR0 && regno <= GDB_REGNO_FPR31) ||
			regno == GDB_REGNO_FFLAGS ||
			regno == GDB_REGNO_FRM ||
			regno == GDB_REGNO_FCSR)
		return "float";
	if (regno >= GDB_REGNO_CSR0 && regno <= GDB_REGNO_CSR4095)
		return "csr";
	if (regno >= GDB_REGNO_V0 && regno <= GDB_REGNO_V31)
		return "vector";
	assert(regno >= GDB_REGNO_COUNT);
	return "custom";
}

uint32_t gdb_regno_size(const struct target *target, uint32_t regno)
{
	if (regno >= GDB_REGNO_FPR0 && regno <= GDB_REGNO_FPR31)
		return riscv_supports_extension(target, 'D') ? 64 : 32;
	if (regno >= GDB_REGNO_V0 && regno <= GDB_REGNO_V31)
		return riscv_vlenb(target) * 8;
	if (regno == GDB_REGNO_PRIV)
		return 8;
	if (regno >= GDB_REGNO_CSR0 && regno <= GDB_REGNO_CSR4095) {
		const unsigned int csr_number = regno - GDB_REGNO_CSR0;
		switch (csr_number) {
			case CSR_DCSR:
			case CSR_MVENDORID:
			case CSR_MCOUNTINHIBIT:

			case CSR_FFLAGS:
			case CSR_FRM:
			case CSR_FCSR:

			case CSR_SCOUNTEREN:
			case CSR_MCOUNTEREN:
				return 32;
		}
	}
	return riscv_xlen(target);
}

static bool vlenb_exists(const struct target *target)
{
	return riscv_vlenb(target) != 0;
}

static bool mtopi_exists(const struct target *target)
{
	RISCV_INFO(info)
	/* TODO: The naming is quite unfortunate here. `mtopi_readable` refers
	 * to how the fact that `mtopi` exists was deduced during examine.
	 */
	return info->mtopi_readable;
}

static bool mtopei_exists(const struct target *target)
{
	RISCV_INFO(info)
	/* TODO: The naming is quite unfortunate here. `mtopei_readable` refers
	 * to how the fact that `mtopei` exists was deduced during examine.
	 */
	return info->mtopei_readable;
}

static bool is_known_standard_csr(unsigned int csr_num)
{
	static const bool is_csr_in_buf[GDB_REGNO_CSR4095 - GDB_REGNO_CSR0 + 1] = {
		#define DECLARE_CSR(csr_name, number)[number] = true,
		#include "encoding.h"
		#undef DECLARE_CSR
	};
	assert(csr_num < ARRAY_SIZE(is_csr_in_buf));

	return is_csr_in_buf[csr_num];
}

static bool gdb_regno_exist(const struct target *target, uint32_t regno)
{
	if (regno <= GDB_REGNO_XPR15 ||
			regno == GDB_REGNO_PC ||
			regno == GDB_REGNO_PRIV)
		return true;
	if (regno > GDB_REGNO_XPR15 && regno <= GDB_REGNO_XPR31)
		return !riscv_supports_extension(target, 'E');
	if (regno >= GDB_REGNO_FPR0 && regno <= GDB_REGNO_FPR31)
		return riscv_supports_extension(target, 'F');
	if (regno >= GDB_REGNO_V0 && regno <= GDB_REGNO_V31)
		return vlenb_exists(target);
	if (regno >= GDB_REGNO_COUNT)
		return true;
	assert(regno >= GDB_REGNO_CSR0 && regno <= GDB_REGNO_CSR4095);
	const unsigned int csr_number = regno - GDB_REGNO_CSR0;
	switch (csr_number) {
		case CSR_FFLAGS:
		case CSR_FRM:
		case CSR_FCSR:
			return riscv_supports_extension(target, 'F');
		case CSR_VSTART:
		case CSR_VXSAT:
		case CSR_VXRM:
		case CSR_VL:
		case CSR_VCSR:
		case CSR_VTYPE:
		case CSR_VLENB:
			return vlenb_exists(target);
		case CSR_SCOUNTEREN:
		case CSR_SSTATUS:
		case CSR_STVEC:
		case CSR_SIP:
		case CSR_SIE:
		case CSR_SSCRATCH:
		case CSR_SEPC:
		case CSR_SCAUSE:
		case CSR_STVAL:
		case CSR_SATP:
			return riscv_supports_extension(target, 'S');
		case CSR_MEDELEG:
		case CSR_MIDELEG:
			/* "In systems with only M-mode, or with both M-mode and
			 * U-mode but without U-mode trap support, the medeleg and
			 * mideleg registers should not exist." */
			return riscv_supports_extension(target, 'S') ||
				riscv_supports_extension(target, 'N');

		case CSR_PMPCFG1:
		case CSR_PMPCFG3:
		case CSR_CYCLEH:
		case CSR_TIMEH:
		case CSR_INSTRETH:
		case CSR_HPMCOUNTER3H:
		case CSR_HPMCOUNTER4H:
		case CSR_HPMCOUNTER5H:
		case CSR_HPMCOUNTER6H:
		case CSR_HPMCOUNTER7H:
		case CSR_HPMCOUNTER8H:
		case CSR_HPMCOUNTER9H:
		case CSR_HPMCOUNTER10H:
		case CSR_HPMCOUNTER11H:
		case CSR_HPMCOUNTER12H:
		case CSR_HPMCOUNTER13H:
		case CSR_HPMCOUNTER14H:
		case CSR_HPMCOUNTER15H:
		case CSR_HPMCOUNTER16H:
		case CSR_HPMCOUNTER17H:
		case CSR_HPMCOUNTER18H:
		case CSR_HPMCOUNTER19H:
		case CSR_HPMCOUNTER20H:
		case CSR_HPMCOUNTER21H:
		case CSR_HPMCOUNTER22H:
		case CSR_HPMCOUNTER23H:
		case CSR_HPMCOUNTER24H:
		case CSR_HPMCOUNTER25H:
		case CSR_HPMCOUNTER26H:
		case CSR_HPMCOUNTER27H:
		case CSR_HPMCOUNTER28H:
		case CSR_HPMCOUNTER29H:
		case CSR_HPMCOUNTER30H:
		case CSR_HPMCOUNTER31H:
		case CSR_MCYCLEH:
		case CSR_MINSTRETH:
		case CSR_MHPMCOUNTER4H:
		case CSR_MHPMCOUNTER5H:
		case CSR_MHPMCOUNTER6H:
		case CSR_MHPMCOUNTER7H:
		case CSR_MHPMCOUNTER8H:
		case CSR_MHPMCOUNTER9H:
		case CSR_MHPMCOUNTER10H:
		case CSR_MHPMCOUNTER11H:
		case CSR_MHPMCOUNTER12H:
		case CSR_MHPMCOUNTER13H:
		case CSR_MHPMCOUNTER14H:
		case CSR_MHPMCOUNTER15H:
		case CSR_MHPMCOUNTER16H:
		case CSR_MHPMCOUNTER17H:
		case CSR_MHPMCOUNTER18H:
		case CSR_MHPMCOUNTER19H:
		case CSR_MHPMCOUNTER20H:
		case CSR_MHPMCOUNTER21H:
		case CSR_MHPMCOUNTER22H:
		case CSR_MHPMCOUNTER23H:
		case CSR_MHPMCOUNTER24H:
		case CSR_MHPMCOUNTER25H:
		case CSR_MHPMCOUNTER26H:
		case CSR_MHPMCOUNTER27H:
		case CSR_MHPMCOUNTER28H:
		case CSR_MHPMCOUNTER29H:
		case CSR_MHPMCOUNTER30H:
		case CSR_MHPMCOUNTER31H:
			return riscv_xlen(target) == 32;
		case CSR_MCOUNTEREN:
			return riscv_supports_extension(target, 'U');
			/* Interrupts M-Mode CSRs. */
		case CSR_MISELECT:
		case CSR_MIREG:
		case CSR_MVIEN:
		case CSR_MVIP:
		case CSR_MIEH:
		case CSR_MIPH:
			return mtopi_exists(target);
		case CSR_MIDELEGH:
		case CSR_MVIENH:
		case CSR_MVIPH:
			return mtopi_exists(target) &&
				riscv_xlen(target) == 32 &&
				riscv_supports_extension(target, 'S');
			/* Interrupts S-Mode CSRs. */
		case CSR_SISELECT:
		case CSR_SIREG:
		case CSR_STOPI:
			return mtopi_exists(target) &&
				riscv_supports_extension(target, 'S');
		case CSR_STOPEI:
			return mtopei_exists(target) &&
				riscv_supports_extension(target, 'S');
		case CSR_SIEH:
		case CSR_SIPH:
			return mtopi_exists(target) &&
				riscv_xlen(target) == 32 &&
				riscv_supports_extension(target, 'S');
			/* Interrupts Hypervisor and VS CSRs. */
		case CSR_HVIEN:
		case CSR_HVICTL:
		case CSR_HVIPRIO1:
		case CSR_HVIPRIO2:
		case CSR_VSISELECT:
		case CSR_VSIREG:
		case CSR_VSTOPI:
			return mtopi_exists(target) &&
				riscv_supports_extension(target, 'H');
		case CSR_VSTOPEI:
			return mtopei_exists(target) &&
				riscv_supports_extension(target, 'H');
		case CSR_HIDELEGH:
		case CSR_HVIENH:
		case CSR_HVIPH:
		case CSR_HVIPRIO1H:
		case CSR_HVIPRIO2H:
		case CSR_VSIEH:
		case CSR_VSIPH:
			return mtopi_exists(target) &&
				riscv_xlen(target) == 32 &&
				riscv_supports_extension(target, 'H');
	}
	return is_known_standard_csr(csr_number);
}

static unsigned int gdb_regno_custom_number(const struct target *target, uint32_t regno)
{
	if (regno < GDB_REGNO_COUNT)
		return 0;

	RISCV_INFO(info);
	assert(!list_empty(&info->expose_custom));
	range_list_t *range;
	unsigned int regno_start = GDB_REGNO_COUNT;
	unsigned int start = 0;
	unsigned int offset = 0;
	list_for_each_entry(range, &info->expose_custom, list) {
		start = range->low;
		assert(regno >= regno_start);
		offset = regno - regno_start;
		const unsigned int regs_in_range = range->high - range->low + 1;
		if (offset < regs_in_range)
			break;
		regno_start += regs_in_range;
	}
	return start + offset;
}

struct reg *riscv_reg_impl_cache_entry(const struct target *target,
		uint32_t number)
{
	assert(target->reg_cache);
	assert(target->reg_cache->reg_list);
	assert(number < target->reg_cache->num_regs);
	return &target->reg_cache->reg_list[number];
}

static int resize_reg(const struct target *target, uint32_t regno, bool exist,
		uint32_t size)
{
	struct reg *reg = riscv_reg_impl_cache_entry(target, regno);
	assert(riscv_reg_impl_is_initialized(reg));
	free(reg->value);
	reg->size = size;
	reg->exist = exist;
	if (reg->exist) {
		reg->value = malloc(DIV_ROUND_UP(reg->size, 8));
		if (!reg->value) {
			LOG_ERROR("Failed to allocate memory.");
			return ERROR_FAIL;
		}
	} else {
		reg->value = NULL;
	}
	assert(riscv_reg_impl_is_initialized(reg));
	return ERROR_OK;
}

static int set_reg_exist(const struct target *target, uint32_t regno, bool exist)
{
	const struct reg *reg = riscv_reg_impl_cache_entry(target, regno);
	assert(riscv_reg_impl_is_initialized(reg));
	return resize_reg(target, regno, exist, reg->size);
}

int riscv_reg_impl_init_one(struct target *target, uint32_t regno, const struct reg_arch_type *reg_type)
{
	struct reg * const reg = riscv_reg_impl_cache_entry(target, regno);
	if (riscv_reg_impl_is_initialized(reg))
		return ERROR_OK;
	reg->number = regno;
	reg->type = reg_type;
	reg->dirty = false;
	reg->valid = false;
	reg->hidden = false;
	reg->name = riscv_reg_gdb_regno_name(target, regno);
	reg->feature = gdb_regno_feature(regno);
	reg->caller_save = gdb_regno_caller_save(regno);
	reg->reg_data_type = gdb_regno_reg_data_type(target, regno);
	reg->group = gdb_regno_group(regno);
	if (regno < GDB_REGNO_COUNT) {
		RISCV_INFO(info);
		reg->arch_info = &info->shared_reg_info;
	} else {
		reg->arch_info = calloc(1, sizeof(riscv_reg_info_t));
		if (!reg->arch_info) {
			LOG_ERROR("Out of memory.");
			return ERROR_FAIL;
		}
		riscv_reg_info_t * const reg_arch_info = reg->arch_info;
		reg_arch_info->target = target;
		reg_arch_info->custom_number = gdb_regno_custom_number(target, regno);
	}
	return resize_reg(target, regno, gdb_regno_exist(target, regno),
			gdb_regno_size(target, regno));
}

static int init_custom_register_names(struct list_head *expose_custom,
		struct reg_name_table *custom_register_names)
{
	unsigned int custom_regs_num = 0;
	if (!list_empty(expose_custom)) {
		range_list_t *entry;
		list_for_each_entry(entry, expose_custom, list)
			custom_regs_num += entry->high - entry->low + 1;
	}

	if (!custom_regs_num)
		return ERROR_OK;

	custom_register_names->reg_names = calloc(custom_regs_num, sizeof(char *));
	if (!custom_register_names->reg_names) {
		LOG_ERROR("Failed to allocate memory for custom_register_names->reg_names");
		return ERROR_FAIL;
	}
	custom_register_names->num_entries = custom_regs_num;
	char **reg_names = custom_register_names->reg_names;
	range_list_t *range;
	unsigned int next_custom_reg_index = 0;
	list_for_each_entry(range, expose_custom, list) {
		for (unsigned int custom_number = range->low; custom_number <= range->high; ++custom_number) {
			if (range->name)
				reg_names[next_custom_reg_index] = init_reg_name(range->name);
			else
				reg_names[next_custom_reg_index] =
					init_reg_name_with_prefix("custom", custom_number);

			if (!reg_names[next_custom_reg_index])
				return ERROR_FAIL;
			++next_custom_reg_index;
		}
	}
	return ERROR_OK;
}

int riscv_reg_impl_init_cache(struct target *target)
{
	RISCV_INFO(info);

	riscv_reg_free_all(target);

	target->reg_cache = calloc(1, sizeof(*target->reg_cache));
	if (!target->reg_cache) {
		LOG_TARGET_ERROR(target, "Failed to allocate memory for target->reg_cache");
		return ERROR_FAIL;
	}
	target->reg_cache->name = "RISC-V Registers";

	if (init_custom_register_names(&info->expose_custom, &info->custom_register_names) != ERROR_OK) {
		LOG_TARGET_ERROR(target, "init_custom_register_names failed");
		return ERROR_FAIL;
	}

	target->reg_cache->num_regs = GDB_REGNO_COUNT + info->custom_register_names.num_entries;
	LOG_TARGET_DEBUG(target, "create register cache for %d registers",
			target->reg_cache->num_regs);

	target->reg_cache->reg_list =
		calloc(target->reg_cache->num_regs, sizeof(struct reg));
	if (!target->reg_cache->reg_list) {
		LOG_TARGET_ERROR(target, "Failed to allocate memory for target->reg_cache->reg_list");
		return ERROR_FAIL;
	}
	return ERROR_OK;
}

int riscv_reg_impl_expose_csrs(const struct target *target)
{
	RISCV_INFO(info);
	range_list_t *entry;
	list_for_each_entry(entry, &info->expose_csr, list) {
		assert(entry->low <= entry->high);
		assert(entry->high <= GDB_REGNO_CSR4095 - GDB_REGNO_CSR0);
		const enum gdb_regno last_regno = GDB_REGNO_CSR0 + entry->high;
		for (enum gdb_regno regno = GDB_REGNO_CSR0 + entry->low;
				regno <= last_regno; ++regno) {
			struct reg * const reg = riscv_reg_impl_cache_entry(target, regno);
			const unsigned int csr_number = regno - GDB_REGNO_CSR0;
			if (reg->exist) {
				LOG_TARGET_WARNING(target,
						"Not exposing CSR %d: register already exists.",
						csr_number);
				continue;
			}
			if (set_reg_exist(target, regno, /*exist*/ true) != ERROR_OK)
				return ERROR_FAIL;
			LOG_TARGET_DEBUG(target, "Exposing additional CSR %d (name=%s)",
					csr_number, reg->name);
		}
	}
	return ERROR_OK;
}

void riscv_reg_impl_hide_csrs(const struct target *target)
{
	RISCV_INFO(info);
	range_list_t *entry;
	list_for_each_entry(entry, &info->hide_csr, list) {
		assert(entry->high <= GDB_REGNO_CSR4095 - GDB_REGNO_CSR0);
		const enum gdb_regno last_regno = GDB_REGNO_CSR0 + entry->high;
		for (enum gdb_regno regno = GDB_REGNO_CSR0 + entry->low;
				regno <= last_regno; ++regno) {
			struct reg * const reg = riscv_reg_impl_cache_entry(target, regno);
			const unsigned int csr_number = regno - GDB_REGNO_CSR0;
			if (!reg->exist) {
				LOG_TARGET_WARNING(target,
						"Not hiding CSR %d: register does not exist.",
						csr_number);
				continue;
			}
			LOG_TARGET_DEBUG(target, "Hiding CSR %d (name=%s).", csr_number, reg->name);
			reg->hidden = true;
		}
	}
}

void riscv_reg_free_all(struct target *target)
{
	free_reg_names(target);
	/* Free the shared structure use for most registers. */
	if (!target->reg_cache)
		return;
	if (target->reg_cache->reg_list) {
		for (unsigned int i = GDB_REGNO_COUNT; i < target->reg_cache->num_regs; i++)
			free(target->reg_cache->reg_list[i].arch_info);
		for (unsigned int i = 0; i < target->reg_cache->num_regs; i++)
			free(target->reg_cache->reg_list[i].value);
		free(target->reg_cache->reg_list);
	}
	free(target->reg_cache);
	target->reg_cache = NULL;
}

int riscv_reg_flush_all(struct target *target)
{
	if (!target->reg_cache)
		return ERROR_OK;

	LOG_TARGET_DEBUG(target, "Flushing register cache");

	/* Writing non-GPR registers may require progbuf execution, and some GPRs
	 * may become dirty in the process (e.g. S0, S1). For that reason, flush
	 * registers in reverse order, so that GPRs are flushed last.
	 */
	for (unsigned int number = target->reg_cache->num_regs; number-- > 0; ) {
		struct reg *reg = riscv_reg_impl_cache_entry(target, number);
		if (reg->valid && reg->dirty) {
			riscv_reg_t value = buf_get_u64(reg->value, 0, reg->size);

			LOG_TARGET_DEBUG(target, "%s is dirty; write back 0x%" PRIx64,
					reg->name, value);
			if (riscv_reg_write(target, number, value) != ERROR_OK)
				return ERROR_FAIL;
		}
	}
	LOG_TARGET_DEBUG(target, "Flush of register cache completed");
	return ERROR_OK;
}

/**
 * This function is used internally by functions that change register values.
 * If `write_through` is true, it is ensured that the value of the target's
 * register is set to be equal to the `value` argument. The cached value is
 * updated if the register is cacheable.
 * TODO: Currently `reg->get/set` is implemented in terms of
 * `riscv_get/set_register`.  However, the intention behind
 * `riscv_get/set_register` is to work with the cache, therefore it accesses
 * and modifyes register cache directly.  The idea is to implement
 * `riscv_get/set_register` in terms of `riscv_reg_impl_cache_entry` and
 * `reg->get/set`.
 */
static int riscv_set_or_write_register(struct target *target,
		enum gdb_regno regid, riscv_reg_t value, bool write_through)
{
	RISCV_INFO(r);
	assert(r);
	if (r->dtm_version == DTM_DTMCS_VERSION_0_11)
		return riscv011_set_register(target, regid, value);

	keep_alive();

	if (regid == GDB_REGNO_PC) {
		return riscv_set_or_write_register(target, GDB_REGNO_DPC, value, write_through);
	} else if (regid == GDB_REGNO_PRIV) {
		riscv_reg_t dcsr;

		if (riscv_reg_get(target, &dcsr, GDB_REGNO_DCSR) != ERROR_OK)
			return ERROR_FAIL;
		dcsr = set_field(dcsr, CSR_DCSR_PRV, get_field(value, VIRT_PRIV_PRV));
		dcsr = set_field(dcsr, CSR_DCSR_V, get_field(value, VIRT_PRIV_V));
		return riscv_set_or_write_register(target, GDB_REGNO_DCSR, dcsr, write_through);
	}

	if (!target->reg_cache) {
		assert(!target_was_examined(target));
		LOG_TARGET_DEBUG(target,
				"No cache, writing to target: %s <- 0x%" PRIx64,
				riscv_reg_gdb_regno_name(target, regid), value);
		return riscv013_set_register(target, regid, value);
	}

	struct reg *reg = riscv_reg_impl_cache_entry(target, regid);

	if (!reg->exist) {
		LOG_TARGET_DEBUG(target, "Register %s does not exist.", reg->name);
		return ERROR_FAIL;
	}

	if (target->state != TARGET_HALTED) {
		LOG_TARGET_DEBUG(target,
				"Target not halted, writing to target: %s <- 0x%" PRIx64,
				reg->name, value);
		return riscv013_set_register(target, regid, value);
	}

	const bool need_to_write = !reg->valid || reg->dirty ||
		value != buf_get_u64(reg->value, 0, reg->size);
	const bool cacheable = riscv_reg_impl_gdb_regno_cacheable(regid, need_to_write);

	if (!cacheable || (write_through && need_to_write)) {
		LOG_TARGET_DEBUG(target,
				"Writing to target: %s <- 0x%" PRIx64 " (cacheable=%s, valid=%s, dirty=%s)",
				reg->name, value, cacheable ? "true" : "false",
				reg->valid ? "true" : "false",
				reg->dirty ? "true" : "false");
		if (riscv013_set_register(target, regid, value) != ERROR_OK)
			return ERROR_FAIL;

		reg->dirty = false;
	} else {
		reg->dirty = need_to_write;
	}

	buf_set_u64(reg->value, 0, reg->size, value);
	reg->valid = cacheable;

	LOG_TARGET_DEBUG(target,
			"Wrote 0x%" PRIx64 " to %s (cacheable=%s, valid=%s, dirty=%s)",
			value, reg->name, cacheable ? "true" : "false",
			reg->valid ? "true" : "false",
			reg->dirty ? "true" : "false");
	return ERROR_OK;
}

/**
 * This function is used to change the value of a register. The new value may
 * be cached, and may not be written until the hart is resumed.
 * TODO: Currently `reg->get/set` is implemented in terms of
 * `riscv_get/set_register`.  However, the intention behind
 * `riscv_get/set_register` is to work with the cache, therefore it accesses
 * and modifyes register cache directly.  The idea is to implement
 * `riscv_get/set_register` in terms of `riscv_reg_impl_cache_entry` and
 * `reg->get/set`.
 */
int riscv_reg_set(struct target *target, enum gdb_regno regid,
		riscv_reg_t value)
{
	return riscv_set_or_write_register(target, regid, value,
			/* write_through */ false);
}

/**
 * This function is used to change the value of a register. The new value may
 * be cached, but it will be written to hart immediately.
 * TODO: Currently `reg->get/set` is implemented in terms of
 * `riscv_get/set_register`.  However, the intention behind
 * `riscv_get/set_register` is to work with the cache, therefore it accesses
 * and modifyes register cache directly.  The idea is to implement
 * `riscv_get/set_register` in terms of `riscv_reg_impl_cache_entry` and
 * `reg->get/set`.
 */
int riscv_reg_write(struct target *target, enum gdb_regno regid,
		riscv_reg_t value)
{
	return riscv_set_or_write_register(target, regid, value,
			/* write_through */ true);
}

/**
 * This function is used to get the value of a register. If possible, the value
 * in cache will be updated.
 * TODO: Currently `reg->get/set` is implemented in terms of
 * `riscv_get/set_register`.  However, the intention behind
 * `riscv_get/set_register` is to work with the cache, therefore it accesses
 * and modifyes register cache directly.  The idea is to implement
 * `riscv_get/set_register` in terms of `riscv_reg_impl_cache_entry` and
 * `reg->get/set`.
 */
int riscv_reg_get(struct target *target, riscv_reg_t *value,
		enum gdb_regno regid)
{
	RISCV_INFO(r);
	assert(r);
	if (r->dtm_version == DTM_DTMCS_VERSION_0_11)
		return riscv013_get_register(target, value, regid);

	keep_alive();

	if (regid == GDB_REGNO_PC)
		return riscv_reg_get(target, value, GDB_REGNO_DPC);

	if (!target->reg_cache) {
		assert(!target_was_examined(target));
		LOG_TARGET_DEBUG(target, "No cache, reading %s from target",
				riscv_reg_gdb_regno_name(target, regid));
		return riscv013_get_register(target, value, regid);
	}

	struct reg *reg = riscv_reg_impl_cache_entry(target, regid);
	if (!reg->exist) {
		LOG_TARGET_DEBUG(target, "Register %s does not exist.", reg->name);
		return ERROR_FAIL;
	}

	if (reg->valid) {
		*value = buf_get_u64(reg->value, 0, reg->size);
		LOG_TARGET_DEBUG(target, "Read %s: 0x%" PRIx64 " (cached)", reg->name,
				*value);
		return ERROR_OK;
	}

	LOG_TARGET_DEBUG(target, "Reading %s from target", reg->name);
	if (riscv013_get_register(target, value, regid) != ERROR_OK)
		return ERROR_FAIL;

	buf_set_u64(reg->value, 0, reg->size, *value);
	reg->valid = riscv_reg_impl_gdb_regno_cacheable(regid, /* is write? */ false) &&
		target->state == TARGET_HALTED;
	reg->dirty = false;

	LOG_TARGET_DEBUG(target, "Read %s: 0x%" PRIx64, reg->name, *value);
	return ERROR_OK;
}
