/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Module to run arbitrary code on Xtensa using OpenOCD                  *
 *   Copyright (C) 2019 Espressif Systems Ltd.                             *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ESP_XTENSA_ALGO_H
#define OPENOCD_TARGET_ESP_XTENSA_ALGO_H

#include <target/xtensa/xtensa.h>
#include <target/espressif/esp_algorithm.h>

/** Index of the first user-defined algo arg. @see algorithm_stub */
#define ESP_XTENSA_STUB_ARGS_FUNC_START             6

extern const struct esp_algorithm_hw xtensa_algo_hw;

#endif	/* OPENOCD_TARGET_XTENSA_ALGO_H */
