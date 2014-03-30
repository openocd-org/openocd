include $(top_srcdir)/common.mk

AM_CPPFLAGS += -I$(top_srcdir)/src/jtag/drivers $(LIBUSB1_CFLAGS) $(LIBFTDI_CFLAGS)

noinst_LTLIBRARIES = libocdusbblaster.la
libocdusbblaster_la_SOURCES = $(USB_BLASTER_SRC)

USB_BLASTER_SRC = usb_blaster.c

if USB_BLASTER_LIBFTDI
USB_BLASTER_SRC += ublast_access_ftdi.c
endif

if USB_BLASTER_FTD2XX
USB_BLASTER_SRC += ublast_access_ftd2xx.c
endif

if USB_BLASTER_2
USB_BLASTER_SRC += ublast2_access_libusb.c
endif

noinst_HEADERS = ublast_access.h

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in
