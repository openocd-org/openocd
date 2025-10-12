// SPDX-License-Identifier: GPL-2.0-or-later

/*
 * Copyright (C) 2025 by STMicroelectronics
 * Copyright (C) 2025 by Antonio Borneo <borneo.antonio@gmail.com>
 */

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdint.h>

#include <helper/align.h>
#include <helper/bitfield.h>
#include <helper/bits.h>
#include <helper/command.h>
#include <helper/log.h>
#include <helper/types.h>
#include <target/arm_adi_v5.h>
#include <target/armv7m_cache.h>
#include <target/cortex_m.h>

static int get_cache_info(struct adiv5_ap *ap, unsigned int cl,
	unsigned int ind, uint32_t *ccsidr)
{
	uint32_t csselr = FIELD_PREP(CSSELR_LEVEL_MASK, cl)
					| FIELD_PREP(CSSELR_IND_MASK, ind);

	int retval = mem_ap_write_u32(ap, CSSELR, csselr);
	if (retval != ERROR_OK)
		return retval;

	return mem_ap_read_u32(ap, CCSIDR, ccsidr);
}

static int get_d_u_cache_info(struct adiv5_ap *ap, unsigned int cl,
	uint32_t *ccsidr)
{
	return get_cache_info(ap, cl, CSSELR_IND_DATA_OR_UNIFIED_CACHE, ccsidr);
}

static int get_i_cache_info(struct adiv5_ap *ap, unsigned int cl,
	uint32_t *ccsidr)
{
	return get_cache_info(ap, cl, CSSELR_IND_INSTRUCTION_CACHE, ccsidr);
}

static struct armv7m_cache_size decode_ccsidr(uint32_t ccsidr)
{
	struct armv7m_cache_size size;

	size.line_len = 16 << FIELD_GET(CCSIDR_LINESIZE_MASK, ccsidr);
	size.associativity = FIELD_GET(CCSIDR_ASSOCIATIVITY_MASK, ccsidr) + 1;
	size.num_sets = FIELD_GET(CCSIDR_NUMSETS_MASK, ccsidr) + 1;
	size.cache_size = size.line_len * size.associativity * size.num_sets / 1024;

	// compute info for set way operation on cache
	size.index_shift = FIELD_GET(CCSIDR_LINESIZE_MASK, ccsidr) + 2;
	size.index = FIELD_GET(CCSIDR_NUMSETS_MASK, ccsidr);
	size.way = FIELD_GET(CCSIDR_ASSOCIATIVITY_MASK, ccsidr);

	unsigned int i = 0;
	while (((size.way << i) & 0x80000000) == 0)
		i++;
	size.way_shift = i;

	return size;
}

int armv7m_identify_cache(struct target *target)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct armv7m_cache_common *cache = &armv7m->armv7m_cache;

	uint32_t clidr;
	int retval = mem_ap_read_u32(armv7m->debug_ap, CLIDR, &clidr);
	if (retval != ERROR_OK)
		return retval;

	uint32_t ctr;
	retval = mem_ap_read_u32(armv7m->debug_ap, CTR, &ctr);
	if (retval != ERROR_OK)
		return retval;

	//  retrieve selected cache for later restore
	uint32_t csselr;
	retval = mem_ap_read_atomic_u32(armv7m->debug_ap, CSSELR, &csselr);
	if (retval != ERROR_OK)
		return retval;

	if (clidr == 0) {
		LOG_TARGET_DEBUG(target, "No cache detected");
		return ERROR_OK;
	}

	if (FIELD_GET(CTR_FORMAT_MASK, ctr) != CTR_FORMAT_PROVIDED) {
		LOG_ERROR("Wrong value in CTR register");
		return ERROR_FAIL;
	}

	cache->i_min_line_len = 4UL << FIELD_GET(CTR_IMINLINE_MASK, ctr);
	cache->d_min_line_len = 4UL << FIELD_GET(CTR_DMINLINE_MASK, ctr);
	LOG_TARGET_DEBUG(target,
		"ctr=0x%" PRIx32 " ctr.i_min_line_len=%" PRIu32 " ctr.d_min_line_len=%" PRIu32,
		ctr, cache->i_min_line_len, cache->d_min_line_len);

	cache->loc = FIELD_GET(CLIDR_LOC_MASK, clidr);
	LOG_TARGET_DEBUG(target,
		"clidr=0x%" PRIx32 " Number of cache levels to PoC=%" PRIu32,
		clidr, cache->loc);

	// retrieve all available inner caches
	uint32_t d_u_ccsidr[8] = {0}, i_ccsidr[8] = {0};
	for (unsigned int cl = 0; cl < cache->loc; cl++) {
		unsigned int ctype = FIELD_GET(CLIDR_CTYPE_MASK(cl + 1), clidr);

		// skip reserved values
		if (ctype > CLIDR_CTYPE_UNIFIED_CACHE)
			continue;

		cache->arch[cl].ctype = ctype;

		// separate d or unified d/i cache at this level ?
		if (ctype & (CLIDR_CTYPE_UNIFIED_CACHE | CLIDR_CTYPE_D_CACHE)) {
			// retrieve d-cache info
			retval = get_d_u_cache_info(armv7m->debug_ap, cl, &d_u_ccsidr[cl]);
			if (retval != ERROR_OK)
				break;
		}

		if (ctype & CLIDR_CTYPE_I_CACHE) {
			// retrieve i-cache info
			retval = get_i_cache_info(armv7m->debug_ap, cl, &i_ccsidr[cl]);
			if (retval != ERROR_OK)
				break;
		}
	}

	// restore selected cache
	int retval1 = mem_ap_write_atomic_u32(armv7m->debug_ap, CSSELR, csselr);

	if (retval != ERROR_OK)
		return retval;
	if (retval1 != ERROR_OK)
		return retval1;

	for (unsigned int cl = 0; cl < cache->loc; cl++) {
		unsigned int ctype = cache->arch[cl].ctype;

		// separate d or unified d/i cache at this level ?
		if (ctype & (CLIDR_CTYPE_UNIFIED_CACHE | CLIDR_CTYPE_D_CACHE)) {
			cache->has_d_u_cache = true;
			cache->arch[cl].d_u_size = decode_ccsidr(d_u_ccsidr[cl]);

			LOG_TARGET_DEBUG(target,
				"data/unified cache index %" PRIu32 " << %" PRIu32 ", way %" PRIu32 " << %" PRIu32,
				cache->arch[cl].d_u_size.index,
				cache->arch[cl].d_u_size.index_shift,
				cache->arch[cl].d_u_size.way,
				cache->arch[cl].d_u_size.way_shift);

			LOG_TARGET_DEBUG(target,
				"cache line %" PRIu32 " bytes %" PRIu32 " KBytes asso %" PRIu32 " ways",
				cache->arch[cl].d_u_size.line_len,
				cache->arch[cl].d_u_size.cache_size,
				cache->arch[cl].d_u_size.associativity);
		}

		if (ctype & CLIDR_CTYPE_I_CACHE) {
			cache->has_i_cache = true;
			cache->arch[cl].i_size = decode_ccsidr(i_ccsidr[cl]);

			LOG_TARGET_DEBUG(target,
				"instruction cache index %" PRIu32 " << %" PRIu32 ", way %" PRIu32 " << %" PRIu32,
				cache->arch[cl].i_size.index,
				cache->arch[cl].i_size.index_shift,
				cache->arch[cl].i_size.way,
				cache->arch[cl].i_size.way_shift);

			LOG_TARGET_DEBUG(target,
				"cache line %" PRIu32 " bytes %" PRIu32 " KBytes asso %" PRIu32 " ways",
				cache->arch[cl].i_size.line_len,
				cache->arch[cl].i_size.cache_size,
				cache->arch[cl].i_size.associativity);
		}
	}

	cache->info_valid = true;

	return ERROR_OK;
}

int armv7m_d_cache_flush(struct target *target, uint32_t address,
	unsigned int length)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct armv7m_cache_common *cache = &armv7m->armv7m_cache;

	if (!cache->info_valid || !cache->has_d_u_cache)
		return ERROR_OK;

	uint32_t line_len = cache->d_min_line_len;
	uint32_t addr_line = ALIGN_DOWN(address, line_len);
	uint32_t addr_end = address + length;

	while (addr_line < addr_end) {
		int retval = mem_ap_write_u32(armv7m->debug_ap, DCCIMVAC, addr_line);
		if (retval != ERROR_OK)
			return retval;
		addr_line += line_len;
		keep_alive();
	}

	return dap_run(armv7m->debug_ap->dap);
}

int armv7m_i_cache_inval(struct target *target, uint32_t address,
	unsigned int length)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct armv7m_cache_common *cache = &armv7m->armv7m_cache;

	if (!cache->info_valid || !cache->has_i_cache)
		return ERROR_OK;

	uint32_t line_len = cache->i_min_line_len;
	uint32_t addr_line = ALIGN_DOWN(address, line_len);
	uint32_t addr_end = address + length;

	while (addr_line < addr_end) {
		int retval = mem_ap_write_u32(armv7m->debug_ap, ICIMVAU, addr_line);
		if (retval != ERROR_OK)
			return retval;
		addr_line += line_len;
		keep_alive();
	}

	return dap_run(armv7m->debug_ap->dap);
}

int armv7m_handle_cache_info_command(struct command_invocation *cmd,
	struct target *target)
{
	struct armv7m_common *armv7m = target_to_armv7m(target);
	struct armv7m_cache_common *cache = &armv7m->armv7m_cache;

	if (!target_was_examined(target)) {
		command_print(cmd, "Target not examined yet");
		return ERROR_FAIL;
	}

	if (!cache->info_valid) {
		command_print(cmd, "No cache detected");
		return ERROR_OK;
	}

	for (unsigned int cl = 0; cl < cache->loc; cl++) {
		struct armv7m_arch_cache *arch = &cache->arch[cl];

		if (arch->ctype & CLIDR_CTYPE_I_CACHE)
			command_print(cmd,
				"L%d I-Cache: line length %" PRIu32 ", associativity %" PRIu32
				", num sets %" PRIu32 ", cache size %" PRIu32 " KBytes",
				cl + 1,
				arch->i_size.line_len,
				arch->i_size.associativity,
				arch->i_size.num_sets,
				arch->i_size.cache_size);

		if (arch->ctype & (CLIDR_CTYPE_UNIFIED_CACHE | CLIDR_CTYPE_D_CACHE))
			command_print(cmd,
				"L%d %c-Cache: line length %" PRIu32 ", associativity %" PRIu32
				", num sets %" PRIu32 ", cache size %" PRIu32 " KBytes",
				cl + 1,
				(arch->ctype & CLIDR_CTYPE_D_CACHE) ? 'D' : 'U',
				arch->d_u_size.line_len,
				arch->d_u_size.associativity,
				arch->d_u_size.num_sets,
				arch->d_u_size.cache_size);
	}

	return ERROR_OK;
}
