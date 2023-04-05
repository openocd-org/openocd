/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2017 by Square, Inc.                                    *
 *   Steven Stallion <stallion@squareup.com>                               *
 ***************************************************************************/

#ifndef OPENOCD_RTOS_RTOS_UCOS_III_STACKINGS_H
#define OPENOCD_RTOS_RTOS_UCOS_III_STACKINGS_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtos/rtos.h>

extern const struct rtos_register_stacking rtos_ucos_iii_cortex_m_stacking;
extern const struct rtos_register_stacking rtos_ucos_iii_esi_risc_stacking;

#endif /* OPENOCD_RTOS_RTOS_UCOS_III_STACKINGS_H */
