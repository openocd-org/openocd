bin_PROGRAMS = openocd

if ECOSBOARD
MAINFILE = ecosboard.c
else
MAINFILE = main.c
endif

openocd_SOURCES = $(MAINFILE) openocd.c

# set the include path found by configure
INCLUDES = -I$(top_srcdir)/src -I$(top_srcdir)/src/helper \
	-I$(top_srcdir)/src/jtag -I$(top_srcdir)/src/target -I$(top_srcdir)/src/xsvf -I$(top_srcdir)/src/svf \
	-I$(top_srcdir)/src/server -I$(top_srcdir)/src/flash -I$(top_srcdir)/src/pld $(all_includes)

# pass path to prefix path
openocd_CPPFLAGS = \
 -DPKGLIBDIR=\"$(pkglibdir)\" \
 -DPKGBLDDATE=\"`date +%F-%R`\"

if RELEASE
openocd_CPPFLAGS += -DRELSTR=\"Release\" -DPKGBLDREV=\"\"
else
openocd_CPPFLAGS += -DRELSTR=\"svn:\" -DPKGBLDREV=\"`$(top_srcdir)/guess-rev.sh $(top_srcdir)`\"
endif

# add default CPPFLAGS
openocd_CPPFLAGS += @CPPFLAGS@

# the library search path.
openocd_LDFLAGS = $(all_libraries) 
SUBDIRS = helper jtag xsvf svf target server flash pld

if IS_MINGW
MINGWLDADD = -lwsock32
else
MINGWLDADD =
endif

if FT2232_LIBFTDI
FTDI2232LIB = -lftdi -lusb
else
if PRESTO_LIBFTDI
FTDI2232LIB = -lftdi -lusb
else
FTDI2232LIB =
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

openocd_LDADD = $(top_builddir)/src/xsvf/libxsvf.a $(top_builddir)/src/svf/libsvf.a \
	$(top_builddir)/src/target/libtarget.a $(top_builddir)/src/jtag/libjtag.a \
	$(top_builddir)/src/helper/libhelper.a \
	$(top_builddir)/src/server/libserver.a $(top_builddir)/src/helper/libhelper.a \
	$(top_builddir)/src/flash/libflash.a $(top_builddir)/src/target/libtarget.a \
	$(top_builddir)/src/pld/libpld.a \
	$(FTDI2232LIB) $(MINGWLDADD) $(LIBUSB)

if HTTPD
openocd_LDADD += -lmicrohttpd
endif

nobase_dist_pkglib_DATA = \
	tcl/bitsbytes.tcl			\
	tcl/chip/atmel/at91/aic.tcl		\
	tcl/chip/atmel/at91/at91sam7x128.tcl	\
	tcl/chip/atmel/at91/at91sam7x256.tcl	\
	tcl/chip/atmel/at91/pmc.tcl		\
	tcl/chip/atmel/at91/rtt.tcl		\
	tcl/chip/atmel/at91/usarts.tcl		\
	tcl/chip/st/stm32/stm32.tcl		\
	tcl/chip/st/stm32/stm32_rcc.tcl		\
	tcl/chip/st/stm32/stm32_regs.tcl	\
	tcl/cpu/arm/arm7tdmi.tcl		\
	tcl/cpu/arm/arm920.tcl			\
	tcl/cpu/arm/arm946.tcl			\
	tcl/cpu/arm/arm966.tcl			\
	tcl/cpu/arm/cortex_m3.tcl               \
	tcl/memory.tcl				\
	tcl/mmr_helpers.tcl			\
	tcl/readable.tcl

MAINTAINERCLEANFILES = Makefile.in
