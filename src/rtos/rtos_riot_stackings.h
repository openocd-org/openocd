/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2015 by Daniel Krebs                                    *
 *   Daniel Krebs - github@daniel-krebs.net                                *
 ***************************************************************************/

#ifndef OPENOCD_RTOS_RTOS_RIOT_STACKINGS_H
#define OPENOCD_RTOS_RTOS_RIOT_STACKINGS_H

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "rtos.h"

extern const struct rtos_register_stacking rtos_riot_cortex_m0_stacking;
extern const struct rtos_register_stacking rtos_riot_cortex_m34_stacking;

#endif	/* OPENOCD_RTOS_RTOS_RIOT_STACKINGS_H */

