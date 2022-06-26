/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
 *   Dominic.Rath@gmx.de                                                   *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ARM_SIMULATOR_H
#define OPENOCD_TARGET_ARM_SIMULATOR_H

struct target;

struct arm_sim_interface {
	void *user_data;
	uint32_t (*get_reg)(struct arm_sim_interface *sim, int reg);
	void (*set_reg)(struct arm_sim_interface *sim, int reg, uint32_t value);
	uint32_t (*get_reg_mode)(struct arm_sim_interface *sim, int reg);
	void (*set_reg_mode)(struct arm_sim_interface *sim, int reg, uint32_t value);
	uint32_t (*get_cpsr)(struct arm_sim_interface *sim, int pos, int bits);
	enum arm_state (*get_state)(struct arm_sim_interface *sim);
	void (*set_state)(struct arm_sim_interface *sim, enum arm_state mode);
	enum arm_mode (*get_mode)(struct arm_sim_interface *sim);
};

/* armv4_5 version */
int arm_simulate_step(struct target *target, uint32_t *dry_run_pc);

#endif /* OPENOCD_TARGET_ARM_SIMULATOR_H */
