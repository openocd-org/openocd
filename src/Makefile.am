SUBDIRS = \
	helper \
	jtag \
	target \
	flash \
	svf \
	xsvf \
	pld \
	server

lib_LTLIBRARIES = libopenocd.la
bin_PROGRAMS = openocd

if ECOSBOARD
MAINFILE = ecosboard.c
else
MAINFILE = main.c
endif

openocd_SOURCES = $(MAINFILE)
openocd_LDADD = libopenocd.la

libopenocd_la_SOURCES = \
	hello.c \
	openocd.c \
	startup_tcl.c

noinst_HEADERS = \
	hello.h \
	openocd.h


# set the include path found by configure
AM_CPPFLAGS = \
	-I$(top_srcdir)/src \
	-I$(top_builddir)/src

libopenocd_la_CPPFLAGS = -DPKGBLDDATE=\"`date +%F-%R`\"

# banner output includes RELSTR appended to $VERSION from the configure script
# guess-rev.sh returns either a repository version ID or "-snapshot"
if RELEASE
libopenocd_la_CPPFLAGS += -DRELSTR=\"\"
else
libopenocd_la_CPPFLAGS += -DRELSTR=\"`$(top_srcdir)/guess-rev.sh $(top_srcdir)`\"
endif

# add default CPPFLAGS
libopenocd_la_CPPFLAGS += $(AM_CPPFLAGS) $(CPPFLAGS)

# the library search path.
libopenocd_la_LDFLAGS = $(all_libraries)

if IS_MINGW
MINGWLDADD = -lws2_32
else
MINGWLDADD =
endif

if FT2232_LIBFTDI
FTDI2232LIB = -lftdi -lusb
else
if USB_BLASTER_LIBFTDI
FTDI2232LIB = -lftdi -lusb
else
if PRESTO_LIBFTDI
FTDI2232LIB = -lftdi -lusb
else
FTDI2232LIB =
endif
endif
endif

if USBPROG
LIBUSB = -lusb
else
if JLINK
LIBUSB = -lusb
else
if RLINK
LIBUSB = -lusb
else
if VSLLINK
LIBUSB = -lusb
else
LIBUSB =
endif
endif
endif
endif

libopenocd_la_LIBADD = \
	$(top_builddir)/src/xsvf/libxsvf.la \
	$(top_builddir)/src/svf/libsvf.la \
	$(top_builddir)/src/pld/libpld.la \
	$(top_builddir)/src/jtag/libjtag.la \
	$(top_builddir)/src/flash/libflash.la \
	$(top_builddir)/src/target/libtarget.la \
	$(top_builddir)/src/server/libserver.la \
	$(top_builddir)/src/helper/libhelper.la \
	$(FTDI2232LIB) $(MINGWLDADD) $(LIBUSB)

if HTTPD
libopenocd_la_LIBADD += -lmicrohttpd
endif

STARTUP_TCL_SRCS = \
	$(srcdir)/helper/startup.tcl \
	$(srcdir)/jtag/startup.tcl \
	$(srcdir)/target/startup.tcl \
	$(srcdir)/flash/startup.tcl \
	$(srcdir)/server/startup.tcl

EXTRA_DIST = $(STARTUP_TCL_SRCS)

BUILT_SOURCES = startup.tcl

startup.tcl: $(STARTUP_TCL_SRCS)
	cat $^ > $@

BIN2C = $(top_builddir)/src/helper/bin2char$(EXEEXT_FOR_BUILD)

# Convert .tcl to cfile
startup_tcl.c: startup.tcl $(BIN2C)
	$(BIN2C) openocd_startup_tcl < $< > $@ || rm -f $@

# add startup_tcl.c to make clean list
CLEANFILES = startup.tcl startup_tcl.c


MAINTAINERCLEANFILES = $(srcdir)/Makefile.in

# The "quick" target builds executables & reinstalls the executables
# Primary use: developer types to quicken the edit/compile/debug
# cycle.  by not requiring a "full build and full install". Note the
# assumption is: You are only rebuilding the EXE.... and everything
# else is/was previously installed.
#
# use at your own risk
quick: all install-binPROGRAMS

