AM_CPPFLAGS = \
	-I$(top_srcdir)/src/helper \
	-I$(top_srcdir)/src/target \
	-I$(top_srcdir)/src/flash \
	-I$(top_srcdir)/src/jtag  \
	-DPKGLIBDIR=\"$(pkglibdir)\"

METASOURCES = AUTO
noinst_LTLIBRARIES = libserver.la
noinst_HEADERS = server.h telnet_server.h gdb_server.h
libserver_la_SOURCES = server.c telnet_server.c gdb_server.c

if HTTPD
libserver_la_SOURCES += httpd.c
endif

libserver_la_CFLAGS =
if IS_MINGW
# FD_* macros are sloppy with their signs on MinGW32 platform
libserver_la_CFLAGS += -Wno-sign-compare
endif

# tcl server addons
noinst_HEADERS += tcl_server.h
libserver_la_SOURCES += tcl_server.c

if HTTPD
nobase_dist_pkglib_DATA = $(wildcard $(srcdir)/httpd/*.tcl $(srcdir)/httpd/*.css $(srcdir)/httpd/menu_cuts/*.png)
endif

MAINTAINERCLEANFILES = Makefile.in
