/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2011 by Broadcom Corporation                            *
 *   Evan Hunter - ehunter@broadcom.com                                    *
 ***************************************************************************/

#ifndef OPENOCD_RTOS_RTOS_STANDARD_STACKINGS_H
#define OPENOCD_RTOS_RTOS_STANDARD_STACKINGS_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rtos.h"

extern const struct rtos_register_stacking rtos_standard_cortex_m3_stacking;
extern const struct rtos_register_stacking rtos_standard_cortex_m4f_stacking;
extern const struct rtos_register_stacking rtos_standard_cortex_m4f_fpu_stacking;
extern const struct rtos_register_stacking rtos_standard_cortex_r4_stacking;
extern const struct rtos_register_stacking rtos_standard_nds32_n1068_stacking;
target_addr_t rtos_generic_stack_align8(struct target *target,
	const uint8_t *stack_data, const struct rtos_register_stacking *stacking,
	target_addr_t stack_ptr);
target_addr_t rtos_cortex_m_stack_align(struct target *target,
	const uint8_t *stack_data, const struct rtos_register_stacking *stacking,
	target_addr_t stack_ptr, size_t xpsr_offset);

#endif /* OPENOCD_RTOS_RTOS_STANDARD_STACKINGS_H */
