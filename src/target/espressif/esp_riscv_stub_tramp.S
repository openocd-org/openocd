/* SPDX-License-Identifier: GPL-2.0-or-later */

/***************************************************************************
 *   RISCV flasher stub wrapper                                            *
 *   Copyright (C) 2021 Espressif Systems Ltd.                             *
 ***************************************************************************/

/* To assemble:
 * riscv32-esp-elf-gcc -c riscv_stub_tramp.S -o riscv_stub_tramp.o
 *
 * To disassemble:
 * riscv32-esp-elf-objdump -d riscv_stub_tramp.o
 *
 * To generate binary file:
 * riscv32-esp-elf-objcopy -O binary riscv_stub_tramp.o riscv_stub_tramp.bin
 *
 * To generate include file:
 * cat riscv_stub_tramp.bin | xxd -i > riscv_stub_tramp.inc
 */

/*
 * Expects :
 * ra = zero
 * sp = stack_base + stack_size - 16, 16 bytes aligned
 * a7 = address of the function to call
 * Params :
 * a0 = command arg0, result (out)
 * a1..a6 = other command args
 */
    .text

    /*.balign  4*/
_stub_enter:
    /* call stub */
    jalr a7
    ebreak
