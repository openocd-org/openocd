arm-none-eabi-gcc -c debug_handler.S -o debug_handler.o
arm-none-eabi-ld -EL -n -Tdebug_handler.cmd debug_handler.o -o debug_handler.out
arm-none-eabi-objcopy -O binary debug_handler.out debug_handler.bin

#arm-none-eabi-gcc -mbig-endian -c debug_handler.S -o debug_handler_be.o
#arm-none-eabi-ld -EB -n -Tdebug_handler.cmd debug_handler_be.o -o debug_handler_be.out
#arm-none-eabi-objcopy -O binary debug_handler_be.out debug_handler_be.bin
