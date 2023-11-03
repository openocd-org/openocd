/* SPDX-License-Identifier: LGPL-2.1-or-later */

/*
 * This code was taken from the fx2lib project from this link:
 * https://github.com/djmuhlestein/fx2lib
 *
 * Copyright (C) 2009 Ubixum, Inc.
*/

/*! \file
 *  Macros for simple common tasks in fx2 firmware.
 * */

#ifndef FX2MACROS_H
#define FX2MACROS_H

#include "reg_ezusb.h"

typedef enum {FALSE = 0, TRUE} BOOL_VALS;

/**
 * \brief Used for getting and setting the CPU clock speed.
 **/
typedef enum {CLK_12M = 0, CLK_24M, CLK_48M} CLK_SPD;

/**
 * \brief Evaluates to a CLK_SPD enum.
 **/
#define CPUFREQ (CLK_SPD)((CPUCS & bmclkspd) >> 3)

#endif
