/* SPDX-License-Identifier: GPL-2.0-or-later */
/****************************************************************
    File : delay.h                                              *
    Contents : Delays handling header file for NanoXplore       *
    USB-JTAG ANGIE adapter hardware.                            *
    Based on openULINK project code by: Martin Schmoelzer.      *
    Copyright 2023, Ahmed Errached BOUDJELIDA, NanoXplore SAS.  *
    <aboudjelida@nanoxplore.com>                                *
    <ahmederrachedbjld@gmail.com>                               *
*****************************************************************/

#ifndef __DELAY_H
#define __DELAY_H

#include <stdint.h>

void syncdelay(uint8_t count);
void delay_5us(void);
void delay_1ms(void);
void delay_us(uint16_t delay);
void delay_ms(uint16_t delay);

#ifndef _IFREQ
#define _IFREQ 48000   /* IFCLK frequency in kHz */
#endif

/* CFREQ can be any one of: 48000, 24000, or 12000 */
#ifndef _CFREQ
#define _CFREQ 48000   /* CLKOUT frequency in kHz */
#endif

#if (_IFREQ < 5000)
#error "_IFREQ too small!  Valid Range: 5000 to 48000..."
#endif

#if (_IFREQ > 48000)
#error "_IFREQ too large!  Valid Range: 5000 to 48000..."
#endif

#if (_CFREQ != 48000)
#if (_CFREQ != 24000)
#if (_CFREQ != 12000)
#error "_CFREQ invalid!  Valid values: 48000, 24000, 12000..."
#endif
#endif
#endif

/* Synchronization Delay formula: see TRM section 15-14 */
#define _SCYCL (3 * (_CFREQ) + 5 * (_IFREQ) - 1) / (2 * (_IFREQ))
#endif
