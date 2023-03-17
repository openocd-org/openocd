/* SPDX-License-Identifier: GPL-2.0-or-later */

OUTPUT_ARCH( "riscv" )

SECTIONS
{
  . = 0x12340000;
  .text :
  {
    *(.text.entry)
    *(.text)
  }
  .data : { *(.data) }
}
