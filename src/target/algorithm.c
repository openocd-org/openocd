/***************************************************************************
 *   Copyright (C) 2005 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 *                                                                         *
 *   This program is free software; you can redistribute it and/or modify  *
 *   it under the terms of the GNU General Public License as published by  *
 *   the Free Software Foundation; either version 2 of the License, or     *
 *   (at your option) any later version.                                   *
 *                                                                         *
 *   This program is distributed in the hope that it will be useful,       *
 *   but WITHOUT ANY WARRANTY; without even the implied warranty of        *
 *   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the         *
 *   GNU General Public License for more details.                          *
 *                                                                         *
 *   You should have received a copy of the GNU General Public License     *
 *   along with this program; if not, write to the                         *
 *   Free Software Foundation, Inc.,                                       *
 *   59 Temple Place - Suite 330, Boston, MA  02111-1307, USA.             *
 ***************************************************************************/
#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include "algorithm.h"

#include "log.h"
#include "configuration.h"
#include "binarybuffer.h"

#include <stdlib.h>


void init_mem_param(mem_param_t *param, u32 address, u32 size, enum param_direction direction)
{
	param->address = address;
	param->size = size;
	param->value = malloc(size);
	param->direction = direction;
}

void destroy_mem_param(mem_param_t *param)
{
	free(param->value);
	param->value = NULL;
}

void init_reg_param(reg_param_t *param, char *reg_name, u32 size, enum param_direction direction)
{
	param->reg_name = reg_name;
	param->size = size;
	param->value = malloc(CEIL(size, 8));
	param->direction = direction;
}

void destroy_reg_param(reg_param_t *param)
{
	free(param->value);
	param->value = NULL;
}
