AM_CPPFLAGS = \
	-I$(top_srcdir)/src \
	-I$(top_builddir)/src \
	-DPKGDATADIR=\"$(pkgdatadir)\"

METASOURCES = AUTO
noinst_LTLIBRARIES = libserver.la
noinst_HEADERS = server.h telnet_server.h gdb_server.h
libserver_la_SOURCES = server.c telnet_server.c gdb_server.c

if HTTPD
libserver_la_SOURCES += httpd.c
else
libserver_la_SOURCES += httpd_stubs.c
if !ECOSBOARD
libserver_la_SOURCES += server_stubs.c
endif
endif
noinst_HEADERS += httpd.h

libserver_la_CFLAGS =
if IS_MINGW
# FD_* macros are sloppy with their signs on MinGW32 platform
libserver_la_CFLAGS += -Wno-sign-compare
endif

# tcl server addons
noinst_HEADERS += tcl_server.h
libserver_la_SOURCES += tcl_server.c

if HTTPD
nobase_dist_pkgdata_DATA = \
	$(wildcard \
		$(srcdir)/httpd/*.tcl \
		$(srcdir)/httpd/*.css \
		$(srcdir)/httpd/menu_cuts/*.png \
	)
endif

EXTRA_DIST = \
	startup.tcl \
	httpd/readme.txt \
	httpd/menu.xml \
	httpd/menu.xsl \
	httpd/build.sh \
	httpd/html2tcl.sh \
	httpd/Stylizer.java \
	httpd/Stylizer.class

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in
