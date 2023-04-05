/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2009 by Marvell Technology Group Ltd.                   *
 *   Written by Nicolas Pitre <nico@marvell.com>                           *
 ***************************************************************************/

#ifndef OPENOCD_TARGET_ARM_SEMIHOSTING_H
#define OPENOCD_TARGET_ARM_SEMIHOSTING_H

#include "semihosting_common.h"

int arm_semihosting_init(struct target *target);
int arm_semihosting(struct target *target, int *retval);

#endif /* OPENOCD_TARGET_ARM_SEMIHOSTING_H */
