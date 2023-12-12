/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   Xtensa flasher stub wrapper                                           *
 *   Copyright (C) 2017 Espressif Systems Ltd.                             *
 ***************************************************************************/

/*
 * Expects :
 * a0 = zero
 * a1 = stack_base + stack_size - 16, 16 bytes aligned
 * a8 = address of the function to call
 * Params :
 * a2 = command arg0, result (out)
 * a3 = command arg1
 * a4 = command arg2
 * a5 = command arg3
 * a6 = command arg4
 * Maximum 5 user args
 */
    .text

    .align  4
_stub_enter:
    /* initialize initial stack frame for callx8 */
    addi    a9, sp,  32 /* point 16 past extra save area */
    s32e    a9, sp, -12 /* access to extra save area */
    /* prepare args */
    mov     a10, a2
    mov     a11, a3
    mov     a12, a4
    mov     a13, a5
    mov     a14, a6
    /* call stub */
    callx8  a8
    /* prepare return value */
    mov     a2, a10
    break 0,0

_idle_loop:
    j _idle_loop
