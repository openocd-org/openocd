/* simple app.

modify test.ld to change address.

Even if the app is position independent, the symbols
need to match to test basic debugging.

To load the app to 0x20000000 in GDB, use:

load a.out
monitor reg sp 0x20004000
monitor reg pc 0x20002000
stepi

arm-elf-gcc -mthumb -mcpu = cortex-m3 -nostdlib -Ttest.ld test.c


*/
int j;
void _start()
{
  int i;
  for (i = 0; i < 1000; i++)
    {
      j++;
    }
}
