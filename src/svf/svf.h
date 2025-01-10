/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2009 by Simon Qian                                      *
 *   SimonQian@SimonQian.com                                               *
 ***************************************************************************/

#ifndef OPENOCD_SVF_SVF_H
#define OPENOCD_SVF_SVF_H

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
int svf_add_statemove(enum tap_state goal_state);

/**
 * svf_tap_state_is_stable() returns true for stable non-SHIFT states
 *
 * @param state The TAP state in question
 * @return true iff the state is stable and not a SHIFT state.
 */
bool svf_tap_state_is_stable(enum tap_state state);

#endif /* OPENOCD_SVF_SVF_H */
