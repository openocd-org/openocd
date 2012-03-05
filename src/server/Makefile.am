include $(top_srcdir)/common.mk

METASOURCES = AUTO
noinst_LTLIBRARIES = libserver.la
noinst_HEADERS = server.h telnet_server.h gdb_server.h
libserver_la_SOURCES = server.c telnet_server.c gdb_server.c

libserver_la_SOURCES += server_stubs.c

libserver_la_CFLAGS =
if IS_MINGW
# FD_* macros are sloppy with their signs on MinGW32 platform
libserver_la_CFLAGS += -Wno-sign-compare
endif

# tcl server addons
noinst_HEADERS += tcl_server.h
libserver_la_SOURCES += tcl_server.c

EXTRA_DIST = \
	startup.tcl

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in
