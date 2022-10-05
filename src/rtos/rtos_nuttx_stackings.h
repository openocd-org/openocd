/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2020 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifndef INCLUDED_RTOS_NUTTX_STACKINGS_H
#define INCLUDED_RTOS_NUTTX_STACKINGS_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <rtos/rtos.h>

extern const struct rtos_register_stacking nuttx_stacking_cortex_m;
extern const struct rtos_register_stacking nuttx_stacking_cortex_m_fpu;

#endif	/* INCLUDED_RTOS_NUTTX_STACKINGS_H */
