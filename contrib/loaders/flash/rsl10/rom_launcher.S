/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Copyright (C) 2022 by Toms Stūrmanis                                  *
 *   toms.sturmanis@gmail.com                                              *
 ***************************************************************************/

	.text
	.syntax unified
	.cpu cortex-m4
	.thumb
    .align 8

/*
 * Params :
 * r0-r2 = arguments
 * r3 = target address in rom
 */

	.thumb_func
	.global _start
_start:
launch_program_in_rom:
    // variables are already set, addres to jump is in r3
    blx r3
exit:
	// Wait for OpenOCD
	bkpt	#0x00
