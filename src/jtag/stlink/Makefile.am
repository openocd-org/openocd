include $(top_srcdir)/common.mk

noinst_LTLIBRARIES = libocdstlink.la

libocdstlink_la_SOURCES = \
	$(STLINKFILES)

nobase_dist_pkglib_DATA =

STLINKFILES =

if STLINK
STLINKFILES += stlink_transport.c
STLINKFILES += stlink_tcl.c
STLINKFILES += stlink_interface.c
STLINKFILES += stlink_layout.c
endif

noinst_HEADERS = \
	stlink_tcl.h

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in
