include $(top_srcdir)/common.mk

AM_CPPFLAGS += -I$(top_srcdir)/src/jtag/drivers $(LIBUSB1_CFLAGS) $(LIBUSB0_CFLAGS)

noinst_LTLIBRARIES = libocdaice.la

libocdaice_la_SOURCES = \
	$(AICEFILES)

AICEFILES =

if AICE
AICEFILES += aice_transport.c
AICEFILES += aice_interface.c
AICEFILES += aice_port.c
AICEFILES += aice_usb.c
AICEFILES += aice_pipe.c
endif

noinst_HEADERS = \
	aice_transport.h \
	aice_interface.h \
	aice_port.h \
	aice_usb.h \
	aice_pipe.h

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in
