include $(top_srcdir)/common.mk

noinst_LTLIBRARIES = libocdjtagdrivers.la
libocdjtagdrivers_la_LIBADD =

libocdjtagdrivers_la_SOURCES = \
	$(DRIVERFILES)

libocdjtagdrivers_la_CPPFLAGS = $(AM_CPPFLAGS) $(LIBUSB1_CFLAGS) \
	$(LIBUSB0_CFLAGS) $(HIDAPI_CFLAGS) $(LIBFTDI_CFLAGS)

ULINK_FIRMWARE = $(srcdir)/OpenULINK

EXTRA_DIST = $(ULINK_FIRMWARE) \
	usb_blaster/README.CheapClone \
	Makefile.rlink \
	rlink_call.m4 \
	rlink_init.m4

DRIVERFILES =
SUBDIRS=

# Standard Driver: common files
DRIVERFILES += driver.c

if USE_LIBUSB1
DRIVERFILES += libusb1_common.c
endif

if USE_LIBUSB0
DRIVERFILES += usb_common.c
if !USE_LIBUSB1
DRIVERFILES += libusb0_common.c
endif
endif

if BITBANG
DRIVERFILES += bitbang.c
endif
if PARPORT
DRIVERFILES += parport.c
endif
if DUMMY
DRIVERFILES += dummy.c
endif
if FT2232_DRIVER
DRIVERFILES += ft2232.c
endif
if FTDI
DRIVERFILES += ftdi.c mpsse.c
endif
if JTAG_VPI
DRIVERFILES += jtag_vpi.c
endif
if USB_BLASTER_DRIVER
SUBDIRS += usb_blaster
libocdjtagdrivers_la_LIBADD += $(top_builddir)/src/jtag/drivers/usb_blaster/libocdusbblaster.la
endif
if AMTJTAGACCEL
DRIVERFILES += amt_jtagaccel.c
endif
if EP93XX
DRIVERFILES += ep93xx.c
endif
if AT91RM9200
DRIVERFILES += at91rm9200.c
endif
if GW16012
DRIVERFILES += gw16012.c
endif
if BITQ
DRIVERFILES += bitq.c
endif
if PRESTO_DRIVER
DRIVERFILES += presto.c
endif
if USBPROG
DRIVERFILES += usbprog.c
endif
if JLINK
DRIVERFILES += jlink.c
endif
if RLINK
DRIVERFILES += rlink.c rlink_speed_table.c
endif
if ULINK
DRIVERFILES += ulink.c
ulinkdir = $(pkgdatadir)/OpenULINK
dist_ulink_DATA = $(ULINK_FIRMWARE)/ulink_firmware.hex
endif
if VSLLINK
DRIVERFILES += versaloon/usbtoxxx/usbtogpio.c
DRIVERFILES += versaloon/usbtoxxx/usbtojtagraw.c
DRIVERFILES += versaloon/usbtoxxx/usbtoswd.c
DRIVERFILES += versaloon/usbtoxxx/usbtopwr.c
DRIVERFILES += versaloon/usbtoxxx/usbtoxxx.c
DRIVERFILES += versaloon/versaloon.c
DRIVERFILES += vsllink.c
endif
if ARMJTAGEW
DRIVERFILES += arm-jtag-ew.c
endif
if BUSPIRATE
DRIVERFILES += buspirate.c
endif
if REMOTE_BITBANG
DRIVERFILES += remote_bitbang.c
endif
if HLADAPTER
DRIVERFILES += stlink_usb.c
DRIVERFILES += ti_icdi_usb.c
endif
if OSBDM
DRIVERFILES += osbdm.c
endif
if OPENDOUS
DRIVERFILES += opendous.c
endif
if SYSFSGPIO
DRIVERFILES += sysfsgpio.c
endif
if BCM2835GPIO
DRIVERFILES += bcm2835gpio.c
endif

if OPENJTAG
DRIVERFILES += openjtag.c
endif

if CMSIS_DAP
DRIVERFILES += cmsis_dap_usb.c
endif

noinst_HEADERS = \
	bitbang.h \
	bitq.h \
	ftd2xx_common.h \
	libusb0_common.h \
	libusb1_common.h \
	libusb_common.h \
	minidriver_imp.h \
	mpsse.h \
	rlink.h \
	rlink_dtc_cmd.h \
	rlink_ep1_cmd.h \
	rlink_st7.h \
	usb_common.h \
	versaloon/usbtoxxx/usbtoxxx.h \
	versaloon/usbtoxxx/usbtoxxx_internal.h \
	versaloon/versaloon.h \
	versaloon/versaloon_include.h \
	versaloon/versaloon_internal.h

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in
