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

/***************************************************************************
 *  JTAG Signals:                                                          *
 ***************************************************************************
 * TMS ....... Test Mode Select                                            *
 * TCK ....... Test Clock                                                  *
 * TDI ....... Test Data Input  (from device point of view, not JTAG       *
 *             adapter point of view!)                                     *
 * TDO ....... Test Data Output (from device point of view, not JTAG       *
 *             adapter point of view!)                                     *
 * TRST ...... Test Reset: Used to reset the TAP Finite State Machine      *
 *             into the Test Logic Reset state                             *
 * SRST .....  Chip Reset                                                  *
 ***************************************************************************/

/* PORT A */
/* PA0 Not Connected */
/* PA1 Not Connected */
#define PIN_RDWR_B        IOA2
#define PIN_CSI_B         IOA3
#define PIN_INIT_B        IOA4
#define PIN_PROGRAM_B     IOA5
/* PA6 Not Connected */
/* PA7 Not Connected */

/* PORT B */
#define PIN_TRST      IOB0
#define PIN_TMS       IOB1
#define PIN_TCK       IOB2
#define PIN_TDI       IOB3
#define PIN_TDO       IOB4
#define PIN_SRST      IOB5
/* PB6 Not Connected */
/* PB7 Not Connected */

/* JTAG Signals with direction 'OUT' on port B */
/* PIN_TDI - PIN_TCK - PIN_TMS - PIN_TRST - PIN_SRST */
#define MASK_PORTB_DIRECTION_OUT (bmbit0 | bmbit1 | bmbit2 | bmbit3 | bmbit5)

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
#define PIN_SDA         IOD0
#define PIN_SCL         IOD1
#define PIN_SDA_DIR     IOD2
#define PIN_SCL_DIR     IOD3
/* PD4 Not Connected */
/* PD5 Not Connected */
/* PD6 Not Connected */
/* PD7 Not Connected */

#endif
