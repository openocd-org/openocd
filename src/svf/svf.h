/***************************************************************************
 *   Copyright (C) 2009 by Simon Qian                                      *
 *   SimonQian@SimonQian.com                                               *
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

#ifndef SVF_H
#define SVF_H

#include <jtag/jtag.h>

int svf_register_commands(struct command_context *cmd_ctx);

/**
 * svf_add_statemove() moves from the current state to @a goal_state.
 *
 * @param goal_state The final TAP state.
 * @return ERROR_OK on success, or an error code on failure.
 *
 * The current and goal states must satisfy svf_tap_state_is_stable().
 * State transition paths used by this routine are those given in the
 * SVF specification for single-argument STATE commands (and also used
 * for various other state transitions).
 */
int svf_add_statemove(tap_state_t goal_state);

/**
 * svf_tap_state_is_stable() returns true for stable non-SHIFT states
 *
 * @param state The TAP state in question
 * @return true iff the state is stable and not a SHIFT state.
 */
bool svf_tap_state_is_stable(tap_state_t state);

#endif	/* SVF_H */
