/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2011 by Martin Schmoelzer                               *
 *   <martin.schmoelzer@student.tuwien.ac.at>                              *
 ***************************************************************************/

#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include "common.h"
#include <stdbool.h>

void execute_set_led_command(void);

bool execute_command(void);
void command_loop(void);

#endif
