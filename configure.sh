#!/bin/sh
./configure --enable-maintainer-mode --prefix=/tmp/openocd/target \
--enable-ft2232-libftdi CFLAGS="-g -L/usr/local/lib -I/usr/local/include" \
--enable-doxygen-pdf --enable-parport --enable-usb_blaster_libftdi \
--enable-ep93xx --enable-at91rm9200 --enable-presto_libftdi --enable-usbprog \
--enable-jlink --enable-vsllink --enable-rlink --enable-ulink \
--enable-arm-jtag-ew --enable-buspirate --enable-stlink --enable-osbdm \
--enable-sysfsgpio --enable-remote-bitbang
