/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef OPENOCD_TARGET_MEM_AP_H
#define OPENOCD_TARGET_MEM_AP_H

#define MEM_AP_COMMON_MAGIC 0x4DE4DA50

struct mem_ap {
	int common_magic;
	struct adiv5_dap *dap;
	struct adiv5_ap *ap;
	uint64_t ap_num;
};

static inline bool is_mem_ap(const struct mem_ap *mem_ap)
{
	return mem_ap->common_magic == MEM_AP_COMMON_MAGIC;
}

/**
 * @returns the pointer to the target specific struct
 * without matching a magic number.
 * Use in target specific service routines, where the correct
 * type of arch_info is certain.
 */
static inline struct mem_ap *
target_to_mem_ap(struct target *target)
{
    return target->arch_info;
}

/**
 * @returns the pointer to the target specific struct
 * or NULL if the magic number does not match.
 * Use in a flash driver or any place where mismatch of the arch_info
 * type can happen.
 */
static inline struct mem_ap *
target_to_mem_ap_safe(struct target *target)
{
    if (!target)
        return NULL;
 
    if (!target->arch_info)
        return NULL;
 
    struct mem_ap *mem_ap = target_to_mem_ap(target);
    if (!is_mem_ap(mem_ap))
        return NULL;
 
    return mem_ap;
}

#endif /* OPENOCD_TARGET_MEM_AP_H */
