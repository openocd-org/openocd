/***************************************************************************
 *   Copyright (C) 2016 by Matthias Welwarsky                              *
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
 *                                                                         *
 ***************************************************************************/

#ifdef HAVE_CONFIG_H
#include "config.h"
#endif

#include <stdlib.h>
#include <stdint.h>
#include "target/arm_adi_v5.h"
#include "target/arm_cti.h"
#include "target/target.h"
#include "helper/time_support.h"

struct arm_cti {
	uint32_t base;
	struct adiv5_ap *ap;
};

struct arm_cti *arm_cti_create(struct adiv5_ap *ap, uint32_t base)
{
	struct arm_cti *self = calloc(1, sizeof(struct arm_cti));
	if (!self)
		return NULL;

	self->base = base;
	self->ap = ap;
	return self;
}

static int arm_cti_mod_reg_bits(struct arm_cti *self, unsigned int reg, uint32_t mask, uint32_t value)
{
	uint32_t tmp;

	/* Read register */
	int retval = mem_ap_read_atomic_u32(self->ap, self->base + reg, &tmp);
	if (ERROR_OK != retval)
		return retval;

	/* clear bitfield */
	tmp &= ~mask;
	/* put new value */
	tmp |= value & mask;

	/* write new value */
	return mem_ap_write_atomic_u32(self->ap, self->base + reg, tmp);
}

int arm_cti_enable(struct arm_cti *self, bool enable)
{
	uint32_t val = enable ? 1 : 0;

	return mem_ap_write_atomic_u32(self->ap, self->base + CTI_CTR, val);
}

int arm_cti_ack_events(struct arm_cti *self, uint32_t event)
{
	int retval;
	uint32_t tmp;

	retval = mem_ap_write_atomic_u32(self->ap, self->base + CTI_INACK, event);
	if (retval == ERROR_OK) {
		int64_t then = timeval_ms();
		for (;;) {
			retval = mem_ap_read_atomic_u32(self->ap, self->base + CTI_TROUT_STATUS, &tmp);
			if (retval != ERROR_OK)
				break;
			if ((tmp & event) == 0)
				break;
			if (timeval_ms() > then + 1000) {
				LOG_ERROR("timeout waiting for target");
				retval = ERROR_TARGET_TIMEOUT;
				break;
			}
		}
	}

	return retval;
}

int arm_cti_gate_channel(struct arm_cti *self, uint32_t channel)
{
	if (channel > 31)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	return arm_cti_mod_reg_bits(self, CTI_GATE, CTI_CHNL(channel), 0);
}

int arm_cti_ungate_channel(struct arm_cti *self, uint32_t channel)
{
	if (channel > 31)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	return arm_cti_mod_reg_bits(self, CTI_GATE, CTI_CHNL(channel), 0xFFFFFFFF);
}

int arm_cti_write_reg(struct arm_cti *self, unsigned int reg, uint32_t value)
{
	return mem_ap_write_atomic_u32(self->ap, self->base + reg, value);
}

int arm_cti_read_reg(struct arm_cti *self, unsigned int reg, uint32_t *p_value)
{
	if (p_value == NULL)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	return mem_ap_read_atomic_u32(self->ap, self->base + reg, p_value);
}

int arm_cti_pulse_channel(struct arm_cti *self, uint32_t channel)
{
	if (channel > 31)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	return arm_cti_write_reg(self, CTI_APPPULSE, CTI_CHNL(channel));
}

int arm_cti_set_channel(struct arm_cti *self, uint32_t channel)
{
	if (channel > 31)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	return arm_cti_write_reg(self, CTI_APPSET, CTI_CHNL(channel));
}

int arm_cti_clear_channel(struct arm_cti *self, uint32_t channel)
{
	if (channel > 31)
		return ERROR_COMMAND_ARGUMENT_INVALID;

	return arm_cti_write_reg(self, CTI_APPCLEAR, CTI_CHNL(channel));
}
