/* SPDX-License-Identifier: LGPL-2.1-or-later */

/**
 * This code was taken from the fx2lib project from this link:
 * https://github.com/djmuhlestein/fx2lib
 *
 * Copyright (C) 2009 Ubixum, Inc.
 **/

/** \file serial.h
 * defines functions to print to a serial console with SIO0
 **/

#include "fx2macros.h"
#include <stdint.h>
/**
 * This function inits sio0 to use T2CON (timer 2)
 * See TRM 14.3.4.1 (Table 14-16)
 * Certain baud rates have too high an error rate to work.  All baud rates are .16%
 * except:
 *
 *          12MHZ      24MHZ
 *   \li 57600  -6.99%
 *   \li 38400  -2.34%     -2.34%
 *   \li 19200  -2.34%
 *
 *   Possible Baud rates:
 *    \li 2400
 *    \li 4800
 *    \li 9600
 *    \li 19200
 *    \li 28800
 *    \li 38400
 *    \li 57600
 *
 *    Any of these rates should work except 57600 at 12mhz.  -2.34% is pushing
 *    most hardware specs for working.  All rates at 48mhz work at .16%
 **/

void sio0_init(uint32_t baud_rate) __critical; /* baud_rate max should be 57600 since int=2 bytes */

/**
 * putchar('\\n') or putchar('\\r') both transmit \\r\\n
 * Just use one or the other. (This makes terminal echo easy)
 **/
int putchar(char c);
int getchar(void);
