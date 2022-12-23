/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ALGORITHM_H
#define OPENOCD_TARGET_ALGORITHM_H

#include "helper/types.h"
#include "helper/replacements.h"

enum param_direction {
	PARAM_IN,
	PARAM_OUT,
	PARAM_IN_OUT
};

struct mem_param {
	target_addr_t address;
	uint32_t size;
	uint8_t *value;
	enum param_direction direction;
};

struct reg_param {
	const char *reg_name;
	uint32_t size;
	uint8_t *value;
	enum param_direction direction;
};

void init_mem_param(struct mem_param *param,
		uint32_t address, uint32_t size, enum param_direction dir);
void destroy_mem_param(struct mem_param *param);

void init_reg_param(struct reg_param *param,
		char *reg_name, uint32_t size, enum param_direction dir);
void destroy_reg_param(struct reg_param *param);

#endif /* OPENOCD_TARGET_ALGORITHM_H */
