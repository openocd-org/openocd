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

#ifndef OPENOCD_TARGET_ARM_CTI_H
#define OPENOCD_TARGET_ARM_CTI_H

/*define CTI(cross trigger interface)*/
#define CTI_CTR				0x0
#define CTI_INACK			0x10
#define CTI_APPSET			0x14
#define CTI_APPCLEAR		0x18
#define CTI_APPPULSE		0x1C
#define CTI_INEN0			0x20
#define CTI_INEN1			0x24
#define CTI_INEN2			0x28
#define CTI_INEN3			0x2C
#define CTI_INEN4			0x30
#define CTI_INEN5			0x34
#define CTI_INEN6			0x38
#define CTI_INEN7			0x3C
#define CTI_INEN8			0x40
#define CTI_INEN(n)			(0x20 + 4 * n)
#define CTI_OUTEN0			0xA0
#define CTI_OUTEN1			0xA4
#define CTI_OUTEN2			0xA8
#define CTI_OUTEN3			0xAC
#define CTI_OUTEN4			0xB0
#define CTI_OUTEN5			0xB4
#define CTI_OUTEN6			0xB8
#define CTI_OUTEN7			0xBC
#define CTI_OUTEN8			0xC0
#define CTI_OUTEN(n)		(0xA0 + 4 * n)
#define CTI_TRIN_STATUS		0x130
#define CTI_TROUT_STATUS	0x134
#define CTI_CHIN_STATUS		0x138
#define CTI_CHOU_STATUS		0x13C
#define CTI_GATE			0x140
#define CTI_UNLOCK			0xFB0

#define CTI_CHNL(x)			(1 << x)
#define CTI_TRIG_HALT		0
#define CTI_TRIG_RESUME		1
#define CTI_TRIG(n)			(1 << CTI_TRIG_##n)

/* forward-declare arm_cti struct */
struct arm_cti;
struct adiv5_ap;

extern const char *arm_cti_name(struct arm_cti *self);
extern struct arm_cti *cti_instance_by_jim_obj(Jim_Interp *interp, Jim_Obj *o);
extern int arm_cti_enable(struct arm_cti *self, bool enable);
extern int arm_cti_ack_events(struct arm_cti *self, uint32_t event);
extern int arm_cti_gate_channel(struct arm_cti *self, uint32_t channel);
extern int arm_cti_ungate_channel(struct arm_cti *self, uint32_t channel);
extern int arm_cti_write_reg(struct arm_cti *self, unsigned int reg, uint32_t value);
extern int arm_cti_read_reg(struct arm_cti *self, unsigned int reg, uint32_t *value);
extern int arm_cti_pulse_channel(struct arm_cti *self, uint32_t channel);
extern int arm_cti_set_channel(struct arm_cti *self, uint32_t channel);
extern int arm_cti_clear_channel(struct arm_cti *self, uint32_t channel);
extern int arm_cti_cleanup_all(void);
extern int cti_register_commands(struct command_context *cmd_ctx);

#endif /* OPENOCD_TARGET_ARM_CTI_H */
