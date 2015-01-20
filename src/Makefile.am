include $(top_srcdir)/common.mk

SUBDIRS = \
	jtag \
	helper \
	target \
	transport \
	flash \
	svf \
	xsvf \
	pld \
	server \
	rtos

noinst_LTLIBRARIES = libopenocd.la
bin_PROGRAMS = openocd

MAINFILE = main.c

openocd_SOURCES = $(MAINFILE)
openocd_LDADD = libopenocd.la

if INTERNAL_JIMTCL
openocd_LDADD += $(top_builddir)/jimtcl/libjim.a
else
openocd_LDADD += -ljim
endif

if ULINK
openocd_LDADD += -lm
endif

libopenocd_la_SOURCES = \
	hello.c \
	openocd.c

noinst_HEADERS = \
	hello.h \
	openocd.h

libopenocd_la_CPPFLAGS = -DPKGBLDDATE=\"`date +%F-%R`\"

# banner output includes RELSTR appended to $VERSION from the configure script
# guess-rev.sh returns either a repository version ID or "-snapshot"
if RELEASE
libopenocd_la_CPPFLAGS += -DRELSTR=\"\"
libopenocd_la_CPPFLAGS += -DGITVERSION=\"\"
else
libopenocd_la_CPPFLAGS += -DRELSTR=\"`$(top_srcdir)/guess-rev.sh $(top_srcdir)`\"
libopenocd_la_CPPFLAGS += -DGITVERSION=\"`cd $(top_srcdir) && git describe`\"
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

libopenocd_la_LIBADD = \
	$(top_builddir)/src/xsvf/libxsvf.la \
	$(top_builddir)/src/svf/libsvf.la \
	$(top_builddir)/src/pld/libpld.la \
	$(top_builddir)/src/jtag/libjtag.la \
	$(top_builddir)/src/transport/libtransport.la \
	$(top_builddir)/src/flash/libflash.la \
	$(top_builddir)/src/target/libtarget.la \
	$(top_builddir)/src/server/libserver.la \
	$(top_builddir)/src/rtos/librtos.la \
	$(top_builddir)/src/helper/libhelper.la \
	$(LIBFTDI_LIBS) $(MINGWLDADD) \
	$(HIDAPI_LIBS) $(LIBUSB0_LIBS) $(LIBUSB1_LIBS)

STARTUP_TCL_SRCS = \
	$(srcdir)/helper/startup.tcl \
	$(srcdir)/jtag/startup.tcl \
	$(srcdir)/target/startup.tcl \
	$(srcdir)/flash/startup.tcl \
	$(srcdir)/server/startup.tcl

EXTRA_DIST = $(STARTUP_TCL_SRCS)

BUILT_SOURCES = startup_tcl.inc

startup.tcl: $(STARTUP_TCL_SRCS)
	cat $^ > $@

BIN2C = $(top_srcdir)/src/helper/bin2char.sh

# Convert .tcl to c-array
startup_tcl.inc: startup.tcl $(BIN2C)
	$(BIN2C) < $< > $@ || { rm -f $@; false; }

# add generated files to make clean list
CLEANFILES = startup.tcl startup_tcl.inc

# we do not want generated file in the dist
dist-hook:
	rm -f $(distdir)/startup_tcl.inc

MAINTAINERCLEANFILES = $(srcdir)/Makefile.in

# The "quick" target builds executables & reinstalls the executables
# Primary use: developer types to quicken the edit/compile/debug
# cycle.  by not requiring a "full build and full install". Note the
# assumption is: You are only rebuilding the EXE.... and everything
# else is/was previously installed.
#
# use at your own risk
quick: all install-binPROGRAMS

