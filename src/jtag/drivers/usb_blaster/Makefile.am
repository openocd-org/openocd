include $(top_srcdir)/common.mk

noinst_LTLIBRARIES = libocdusbblaster.la
libocdusbblaster_la_SOURCES = $(USB_BLASTER_SRC)

USB_BLASTER_SRC = usb_blaster.c

if USB_BLASTER_LIBFTDI
USB_BLASTER_SRC += ublast_access_ftdi.c
endif

if USB_BLASTER_FTD2XX
USB_BLASTER_SRC += ublast_access_ftd2xx.c
endif

noinst_HEADERS = ublast_access.h

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in
