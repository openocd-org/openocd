INCLUDES = -I$(top_srcdir)/src $(all_includes) -I$(top_srcdir)/src/target -I$(top_srcdir)/src/server
METASOURCES = AUTO
AM_CPPFLAGS = -DPKGDATADIR=\"$(pkgdatadir)\" -DPKGLIBDIR=\"$(pkglibdir)\" @CPPFLAGS@
noinst_LIBRARIES = libhelper.a

if ECOSBOARD
CONFIGFILES = 
else
CONFIGFILES = options.c jim.c jim-eventloop.c 
endif



libhelper_a_SOURCES = binarybuffer.c $(CONFIGFILES) configuration.c log.c command.c time_support.c \
	replacements.c fileio.c startup_tcl.c

if IOUTIL
libhelper_a_SOURCES += ioutil.c
endif


noinst_HEADERS = binarybuffer.h configuration.h types.h log.h command.h \
	time_support.h replacements.h fileio.h jim.h jim-eventloop.h \
	startup.tcl bin2char.c

bin2char$(EXEEXT_FOR_BUILD): bin2char.c
	${CC_FOR_BUILD} ${CFLAGS_FOR_BUILD} $(srcdir)/bin2char.c -o $@

# Convert .tcl to cfile
startup_tcl.c: startup.tcl bin2char$(EXEEXT_FOR_BUILD)
	./bin2char$(EXEEXT_FOR_BUILD) startup_tcl < $(srcdir)/startup.tcl > $@

# add startup_tcl.c to make clean list
CLEANFILES = startup_tcl.c bin2char$(EXEEXT_FOR_BUILD)

MAINTAINERCLEANFILES = Makefile.in
