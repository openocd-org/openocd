// SPDX-License-Identifier: GPL-2.0-or-later

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "target/target.h"
#include "target/target_type.h"

#include "riscv.h"
#include "riscv_reg.h"
#include "riscv-013.h"

enum scr5_cfg_opts {
	SCR5_CFG_TLB_CSR_BASE,
	SCR5_CFG_TLB_N_ENTRIES,
	SCR5_CFG_TLB_LOOKUP,
	N_SCR5_CFG
};

struct scr5_private_config {
	struct riscv_private_config riscv_pc;

	bool configured[N_SCR5_CFG];

	bool tlb_lookup;
	unsigned int tlb_csr_base;
	unsigned int tlb_n_entries;
};

struct scr5_private_config *scr5_private_config(const struct target *target)
{
	return container_of(target->private_config, struct scr5_private_config, riscv_pc);
}

struct scr5_private_config *alloc_default_scr5_private_config(void)
{
	struct riscv_private_config * const riscv_pc = alloc_default_riscv_private_config();
	if (!riscv_pc)
		return NULL;
	struct scr5_private_config * const pc = malloc(sizeof(*pc));
	if (!pc) {
		LOG_ERROR("Out of memory!");
		return NULL;
	}
	memcpy(&pc->riscv_pc, riscv_pc, sizeof(*riscv_pc));
	free(riscv_pc);

	memset(pc->configured, false, sizeof(pc->configured));

	pc->tlb_lookup = true;
	pc->configured[SCR5_CFG_TLB_LOOKUP] = true;

	pc->tlb_csr_base = 0xbc0;
	pc->configured[SCR5_CFG_TLB_CSR_BASE] = true;
	return pc;
}

static int scr5_init_target(struct command_context *cmd_ctx, struct target *target)
{
	if (!target->private_config)
		goto config_error;
	struct scr5_private_config *pc = scr5_private_config(target);
	for (int i = 0; i < N_SCR5_CFG; ++i)
		if (!pc->configured[i])
			goto config_error;

	const char *scr5_tlb_csr_names[] = {
		"tlb_attr",
		"tlb_va",
		"tlb_update",
		"tlb_scan"
	};
	range_list_t *range = NULL;
	for (unsigned i = 0; i < ARRAY_SIZE(scr5_tlb_csr_names); ++i) {
		RISCV_INFO(info);
		range = calloc(1, sizeof(*range));
		if (!range)
			goto out_of_memory;

		const unsigned int csr_addr = pc->tlb_csr_base + i;
		range->low = csr_addr;
		range->high = csr_addr;
		range->name = strdup(scr5_tlb_csr_names[i]);
		if (!range->name)
			goto out_of_memory;

		list_add(&range->list, &info->expose_csr);

	}

	return riscv_init_target(cmd_ctx, target);

config_error:
	LOG_TARGET_ERROR(target, "SCR5 target is misconfigured! "
			"Please, consult documentation.");
	return ERROR_FAIL;
out_of_memory:
	free(range);
	LOG_ERROR("Out of memory.");
	return ERROR_FAIL;
}

static void scr5_deinit_target(struct target *target)
{
	free(scr5_private_config(target));
	target->private_config = NULL;
	riscv_deinit_target(target);
}

static struct jim_nvp nvp_config_opts[] = {
	{ .name = "-scr5-tlb-csr-base", .value = SCR5_CFG_TLB_CSR_BASE },
	{ .name = "-scr5-tlb-n-entries", .value = SCR5_CFG_TLB_N_ENTRIES },
	{ .name = "-scr5-tlb-lookup", .value = SCR5_CFG_TLB_LOOKUP },
	{ .name = NULL, .value = N_SCR5_CFG }
};

static struct jim_nvp nvp_enable[] = {
	{ .name = "disable", .value = false },
	{ .name = "enable", .value = true },
	{ .name = NULL, .value = -1 }
};

static int scr5_jim_configure(struct target *target,
		struct jim_getopt_info *goi)
{
	struct scr5_private_config *pc = scr5_private_config(target);
	if (!pc) {
		pc = alloc_default_scr5_private_config();
		if (!pc)
			return JIM_ERR;
		target->private_config = &pc->riscv_pc;
	}

	int e = riscv_jim_configure(target, goi);
	if (e != JIM_CONTINUE)
		return e;

	if (!goi->argc)
		return JIM_OK;

	struct jim_nvp *n;
	e = jim_nvp_name2value_obj(goi->interp, nvp_config_opts,
				goi->argv[0], &n);
	if (e != JIM_OK)
		return JIM_CONTINUE;

	e = jim_getopt_obj(goi, NULL);
	if (e != JIM_OK)
		return e;

	jim_wide w;
	switch (n->value) {
	case SCR5_CFG_TLB_CSR_BASE:
		if (!goi->is_configure) {
			if (goi->argc != 0)
				goto cget_extra_args;
			Jim_SetResultFormatted(goi->interp, "%u", pc->tlb_n_entries);
			return JIM_OK;
		}
		e = jim_getopt_wide(goi, &w);
		if (e != JIM_OK)
			return e;
		if (w > UINT_MAX || w < 0) {
			Jim_SetResultFormatted(goi->interp, "%s is out of range for %s",
					goi->argv[-1], goi->argv[-2]);
			return JIM_ERR;
		}
		pc->tlb_csr_base = w;
		break;
	case SCR5_CFG_TLB_N_ENTRIES:
		if (!goi->is_configure) {
			if (goi->argc != 0)
				goto cget_extra_args;
			Jim_SetResultFormatted(goi->interp, "%u", pc->tlb_n_entries);
			return JIM_OK;
		}
		e = jim_getopt_wide(goi, &w);
		if (e != JIM_OK)
			return e;
		if (w > UINT_MAX || w < 0) {
			Jim_SetResultFormatted(goi->interp, "%s is out of range for %s",
					goi->argv[-1], goi->argv[-2]);
			return JIM_ERR;
		}
		pc->tlb_n_entries = w;
		break;
	case SCR5_CFG_TLB_LOOKUP:
		if (!goi->is_configure) {
			if (goi->argc != 0)
				goto cget_extra_args;
			Jim_SetResultString(goi->interp,
					jim_nvp_value2name_simple(nvp_enable, pc->tlb_lookup)->name, -1);
			return JIM_OK;
		}
		struct jim_nvp *tlb_lookup;
		e = jim_getopt_nvp(goi, nvp_enable, &tlb_lookup);
		if (e != JIM_OK)
			return e;
		pc->tlb_lookup = tlb_lookup->value;
		break;
	default:
		assert(false && "'jim_getopt_nvp' should have returned an error.");
	}
	assert(goi->is_configure);
	pc->configured[n->value] = true;
	return JIM_OK;
cget_extra_args:
	Jim_SetResultString(goi->interp, "'cget' takes exactly one argument", -1);
	return JIM_ERR;
}

static int scr5_search_tlb(struct target *target, target_addr_t virt_addr,
		target_addr_t *phys_addr)
{
	const struct scr5_private_config * const pc = scr5_private_config(target);
	assert(pc->tlb_lookup);
	const unsigned int tlb_attr_csr = GDB_REGNO_CSR0 + pc->tlb_csr_base + 0;
	const unsigned int tlb_va_csr = GDB_REGNO_CSR0 + pc->tlb_csr_base + 1;
	const unsigned int tlb_scan_csr = GDB_REGNO_CSR0 + pc->tlb_csr_base + 3;
	const unsigned int tlb_sel_offset = riscv_xlen(target) - 1;
	const unsigned int megapage_attr_offset = 8;
	const unsigned int valid_mask = 1;

	riscv_reg_t orig_tlb_scan;
	int res = riscv_reg_get(target, &orig_tlb_scan, tlb_scan_csr);
	if (res != ERROR_OK)
		return res;

	for (unsigned int tlb_sel = 0; tlb_sel <= 1; ++tlb_sel) {
		for (unsigned int pte_idx = 0; pte_idx < pc->tlb_n_entries; ++pte_idx) {
			res = riscv_reg_set(target, tlb_scan_csr,
					tlb_sel << tlb_sel_offset | pte_idx);
			riscv_reg_t tlb_attr, tlb_va;
			res = riscv_reg_get(target, &tlb_attr, tlb_attr_csr);
			if (res != ERROR_OK)
				goto cleanup;
			if (!(tlb_attr & valid_mask)) {
				LOG_TARGET_DEBUG(target, "Skipping invalid TLB entry.");
				continue;
			}
			res = riscv_reg_get(target, &tlb_va, tlb_va_csr);
			if (res != ERROR_OK)
				goto cleanup;

			const unsigned int pte_page_number_shift =
				(tlb_attr & BIT(megapage_attr_offset)) ? 20 : 10;
			const unsigned int addr_page_number_shift =
				pte_page_number_shift + 2;
			const riscv_reg_t tlb_vpn = tlb_va >> addr_page_number_shift;
			const riscv_reg_t addr_vpn = virt_addr >> addr_page_number_shift;
			if (tlb_vpn != addr_vpn) {
				LOG_TARGET_DEBUG(target, "VPN from TLB entry (%" PRIx64
						"). doesn't match the one from address (%" PRIx64 ").",
						tlb_vpn, addr_vpn);
				continue;
			}

			*phys_addr = (virt_addr & GENMASK(addr_page_number_shift - 1, 0)) |
				((tlb_attr >> pte_page_number_shift) << addr_page_number_shift);

			res = ERROR_OK;
			goto cleanup;
		}
	}
	LOG_TARGET_DEBUG(target, "TLB entry not found.");
	res = ERROR_TARGET_RESOURCE_NOT_AVAILABLE;
cleanup:
	if (riscv_reg_set(target, tlb_scan_csr, orig_tlb_scan) != ERROR_OK)
		return ERROR_FAIL;

	return res;
}

static int scr5_virt2phys(struct target *target, target_addr_t virt_addr,
		target_addr_t *phys_addr)
{
	const struct scr5_private_config * const pc = scr5_private_config(target);
	if (!pc->tlb_lookup) {
		LOG_TARGET_DEBUG(target, "TLB lookup is disabled in the configuration");
		return riscv_virt2phys(target, virt_addr, phys_addr);
	}

	int mmu_enabled;
	int res = riscv_mmu(target, &mmu_enabled);
	if (res != ERROR_OK)
		return res;
	if (!mmu_enabled) {
		*phys_addr = virt_addr;
		LOG_TARGET_DEBUG(target, "MMU is disabled. 0x%" TARGET_PRIxADDR
				" -> 0x%" TARGET_PRIxADDR, virt_addr, *phys_addr);
		return ERROR_OK;
	}
	res = scr5_search_tlb(target, virt_addr, phys_addr);
	if (res != ERROR_TARGET_RESOURCE_NOT_AVAILABLE)
		return res;
	return riscv_virt2phys(target, virt_addr, phys_addr);
}

struct target_type scr5_target = {
	.name = "scr5",
	.target_create = riscv_create_target,
	.target_jim_configure = scr5_jim_configure,
	.init_target = scr5_init_target,
	.deinit_target = scr5_deinit_target,
	.examine = riscv_examine,
	.poll = riscv_openocd_poll,
	.halt = riscv_halt,
	.resume = riscv_target_resume,
	.step = riscv_openocd_step,
	.assert_reset = riscv_assert_reset,
	.deassert_reset = riscv_deassert_reset,
	.read_memory = riscv_read_memory,
	.write_memory = riscv_write_memory,
	.read_phys_memory = riscv_read_phys_memory,
	.write_phys_memory = riscv_write_phys_memory,
	.checksum_memory = riscv_checksum_memory,
	.mmu = riscv_mmu,
	.virt2phys = scr5_virt2phys,
	.get_gdb_arch = riscv_get_gdb_arch,
	.get_gdb_reg_list = riscv_get_gdb_reg_list,
	.get_gdb_reg_list_noread = riscv_get_gdb_reg_list_noread,
	.add_breakpoint = riscv_add_breakpoint,
	.remove_breakpoint = riscv_remove_breakpoint,
	.add_watchpoint = riscv_add_watchpoint,
	.remove_watchpoint = riscv_remove_watchpoint,
	.hit_watchpoint = riscv_hit_watchpoint,
	.arch_state = riscv_arch_state,
	.run_algorithm = riscv_run_algorithm,
	.commands = riscv_command_handlers,
	.address_bits = riscv_xlen_nonconst,
	.data_bits = riscv_data_bits
};
