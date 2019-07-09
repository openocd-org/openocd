OUTPUT_ARCH( "riscv" )

SECTIONS
{
  . = 0x80000000;
  .text :
  {
    *(.text.entry)
    *(.text)
  }
  .data : { *(.data) }
}
