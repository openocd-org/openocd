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
#ifndef ALGORITHM_H
#define ALGORITHM_H

#include "types.h"

enum param_direction
{
	PARAM_IN,
	PARAM_OUT,
	PARAM_IN_OUT
};

typedef struct mem_param_s
{
	uint32_t address;
	uint32_t size;
	uint8_t *value;
	enum param_direction direction;
} mem_param_t;

typedef struct reg_param_s
{
	char *reg_name;
	uint32_t size;
	uint8_t *value;
	enum param_direction direction;
} reg_param_t;

extern void init_mem_param(mem_param_t *param, uint32_t address, uint32_t size, enum param_direction direction);
extern void destroy_mem_param(mem_param_t *param);
extern void init_reg_param(reg_param_t *param, char *reg_name, uint32_t size, enum param_direction direction);
extern void destroy_reg_param(reg_param_t *param);

#endif /* ALGORITHM_H */
