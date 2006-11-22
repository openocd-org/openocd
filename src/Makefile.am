bin_PROGRAMS = openocd
openocd_SOURCES = openocd.c

# set the include path found by configure
INCLUDES = -I$(top_srcdir)/src/helper \
	-I$(top_srcdir)/src/jtag -I$(top_srcdir)/src/target -I$(top_srcdir)/src/xsvf -I$(top_srcdir)/src/server \
	-I$(top_srcdir)/src/flash -I$(top_srcdir)/src/pld $(all_includes)

# the library search path.
openocd_LDFLAGS = $(all_libraries) 
SUBDIRS = helper jtag xsvf target server flash pld

if IS_MINGW
MINGWLDADD = -lwsock32
else
MINGWLDADD =
endif

if FT2232_LIBFTDI
FTDI2232LIB = -lftdi
else
FTDI2232LIB =
endif

if IS_WIN32
if FTD2XXDIR
FTD2XXLDADD = @WITH_FTD2XX@/FTD2XX.lib
else
FTD2XXLDADD = -lftd2xx
endif
else
FTD2XXLDADD = -lftd2xx 
endif
  
if FT2232_FTD2XX
FTD2XXLIB = $(FTD2XXLDADD)
else
FTD2XXLIB =
endif

openocd_LDADD = $(top_builddir)/src/xsvf/libxsvf.a \
	$(top_builddir)/src/target/libtarget.a $(top_builddir)/src/jtag/libjtag.a \
	$(top_builddir)/src/helper/libhelper.a \
	$(top_builddir)/src/server/libserver.a $(top_builddir)/src/helper/libhelper.a \
	$(top_builddir)/src/flash/libflash.a $(top_builddir)/src/target/libtarget.a \
	$(top_builddir)/src/pld/libpld.a \
	$(FTDI2232LIB) $(FTD2XXLIB) $(MINGWLDADD)
