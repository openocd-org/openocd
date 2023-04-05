/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2007 by Pavel Chromy                                    *
 *   chromy@asix.cz                                                        *
 ***************************************************************************/
#ifndef dccH
#define dccH

#include "platform.h"

/* debug channel read (debugger->MCU) */
uint32 dcc_rd(void);

/* debug channel write (MCU->debugger) */
int dcc_wr(uint32 data);

#endif
