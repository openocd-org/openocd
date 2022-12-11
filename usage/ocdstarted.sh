#!/bin/bash


openocd [find /ft232r.cfg] [find /esp32.cfg]
espressif
openocd -f interface/ft232r.cfg\
          -s target/esp32.cfg

          openocd -f interface/ft232r.cfg \
        -c "gdb_memory_map enable" \
        -c "gdb_flash_program enable" \
        -f target/sam7x256.cfg
