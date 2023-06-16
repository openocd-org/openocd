/* SPDX-License-Identifier: GPL-2.0-or-later */
/****************************************************************************
    File : protocol.h                                                       *
    Contents : Jtag commands handling protocol header file for NanoXplore   *
    USB-JTAG ANGIE adapter hardware.                                        *
    Based on openULINK project code by: Martin Schmoelzer.                  *
    Copyright 2023, Ahmed Errached BOUDJELIDA, NanoXplore SAS.              *
    <aboudjelida@nanoxplore.com>                                            *
	<ahmederrachedbjld@gmail.com>											*
*****************************************************************************/

#ifndef __PROTOCOL_H
#define __PROTOCOL_H

#include <stdbool.h>

bool execute_command(void);
void command_loop(void);

#endif
