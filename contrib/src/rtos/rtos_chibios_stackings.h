/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2011 by Broadcom Corporation                            *
 *   Evan Hunter - ehunter@broadcom.com                                    *
 ***************************************************************************/

#ifndef OPENOCD_RTOS_RTOS_CHIBIOS_STACKINGS_H
#define OPENOCD_RTOS_RTOS_CHIBIOS_STACKINGS_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rtos.h"

extern const struct rtos_register_stacking rtos_chibios_arm_v7m_stacking;
extern const struct rtos_register_stacking rtos_chibios_arm_v7m_stacking_w_fpu;

#endif /* OPENOCD_RTOS_RTOS_CHIBIOS_STACKINGS_H */
