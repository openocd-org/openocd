// SPDX-License-Identifier: GPL-2.0-or-later

/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "algorithm.h"
#include <helper/binarybuffer.h>

void init_mem_param(struct mem_param *param, uint32_t address, uint32_t size, enum param_direction direction)
{
	param->address = address;
	param->size = size;
	param->value = malloc(size);
	param->direction = direction;
}

void destroy_mem_param(struct mem_param *param)
{
	free(param->value);
	param->value = NULL;
}

void init_reg_param(struct reg_param *param, char *reg_name, uint32_t size, enum param_direction direction)
{
	param->reg_name = reg_name;
	param->size = size;
	param->value = malloc(DIV_ROUND_UP(size, 8));
	param->direction = direction;
}

void destroy_reg_param(struct reg_param *param)
{
	free(param->value);
	param->value = NULL;
}
