/***************************************************************************
 *   Copyright (C) 2006 by Dominic Rath                                    *
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
 *   51 Franklin Street, Fifth Floor, Boston, MA 02110-1301 USA.           *
 ***************************************************************************/

#ifndef ARM_SIMULATOR_H
#define ARM_SIMULATOR_H

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

#endif /* ARM_SIMULATOR_H */
