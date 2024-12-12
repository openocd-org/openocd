/* SPDX-License-Identifier: GPL-2.0-or-later */
/****************************************************************************
    File : io.h                                                             *
    Contents : input/output declaration header file for NanoXplore          *
    USB-JTAG ANGIE adapter hardware.                                        *
    Based on openULINK project code by: Martin Schmoelzer.                  *
    Copyright 2023, Ahmed Errached BOUDJELIDA, NanoXplore SAS.              *
    <aboudjelida@nanoxplore.com>                                            *
	<ahmederrachedbjld@gmail.com>											*
*****************************************************************************/

#ifndef __IO_H
#define __IO_H

#include "reg_ezusb.h"

/* PORT A */
#define PIN_SDA_DIR     IOA0
/* PA1 Not Connected */
#define PIN_RDWR_B      IOA2
#define PIN_SDA         IOA3
#define PIN_SCL         IOA4
#define PIN_PROGRAM_B   IOA5
/* PA6 Not Connected */
/* PA7 Not Connected */

/* PORT B */
/* PB0 Not Connected */
/* PB1 Not Connected */
/* PB2 Not Connected */
/* PB3 Not Connected */
/* PB4 Not Connected */
/* PB5 Not Connected */
/* PB6 Not Connected */
/* PB7 Not Connected */

/* PORT C */
#define PIN_T0      IOC0
#define PIN_T1      IOC1
#define PIN_T2      IOC2
#define PIN_T3      IOC3
#define PIN_T4      IOC4
/* PC5 Not Connected */
/* PC6 Not Connected */
/* PC7 Not Connected */

/* PORT D */
/* PD0 Not Connected */
/* PD1 Not Connected */
/* PD2 Not Connected */
/* PD3 Not Connected */
/* PD4 Not Connected */
/* PD5 Not Connected */
/* PD6 Not Connected */
/* PD7 Not Connected */

#endif
