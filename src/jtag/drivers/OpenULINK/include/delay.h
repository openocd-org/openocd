/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2011 by Martin Schmoelzer                               *
 *   <martin.schmoelzer@student.tuwien.ac.at>                              *
 ***************************************************************************/

#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>

#define NOP { __asm nop __endasm; }

void delay_5us(void);
void delay_1ms(void);

void delay_us(uint16_t delay);
void delay_ms(uint16_t delay);

#endif
