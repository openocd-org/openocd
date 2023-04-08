/* SPDX-License-Identifier: GPL-2.0-or-later */

#ifndef INCLUDED_RTOS_NUTTX_STACKINGS_H
#define INCLUDED_RTOS_NUTTX_STACKINGS_H

#include "rtos.h"

extern const struct rtos_register_stacking nuttx_stacking_cortex_m;
extern const struct rtos_register_stacking nuttx_stacking_cortex_m_fpu;
extern const struct rtos_register_stacking nuttx_riscv_stacking;
extern const struct rtos_register_stacking nuttx_esp32_stacking;
extern const struct rtos_register_stacking nuttx_esp32s2_stacking;
extern const struct rtos_register_stacking nuttx_esp32s3_stacking;

#endif	/* INCLUDED_RTOS_NUTTX_STACKINGS_H */
