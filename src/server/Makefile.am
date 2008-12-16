INCLUDES = -I$(top_srcdir)/src/helper -I$(top_srcdir)/src/target -I$(top_srcdir)/src/flash -I$(top_srcdir)/src/jtag $(all_includes)
METASOURCES = AUTO
noinst_LIBRARIES = libserver.a
noinst_HEADERS = server.h telnet_server.h gdb_server.h
libserver_a_SOURCES = server.c telnet_server.c gdb_server.c

if HTTPD
libserver_a_SOURCES += httpd.c
endif


# tcl server addons
noinst_HEADERS += tcl_server.h
libserver_a_SOURCES += tcl_server.c
