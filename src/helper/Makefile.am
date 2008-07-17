INCLUDES = -I$(top_srcdir)/src $(all_includes)
METASOURCES = AUTO
AM_CPPFLAGS = -DPKGDATADIR=\"$(pkgdatadir)\" -DPKGLIBDIR=\"$(pkglibdir)\" @CPPFLAGS@
noinst_LIBRARIES = libhelper.a

if ECOSBOARD
CONFIGFILES = 
else
CONFIGFILES = options.c jim.c
endif

libhelper_a_SOURCES = binarybuffer.c $(CONFIGFILES) configuration.c log.c command.c time_support.c \
	replacements.c fileio.c
noinst_HEADERS = binarybuffer.h configuration.h types.h log.h command.h \
	time_support.h replacements.h fileio.h \
	jim.h

libhelper_a_SOURCES += startup_tcl.c

noinst_PROGRAMS = bin2char

bin2char_SOURCES = bin2char.c

# Convert .tcl to cfile
startup_tcl.c: bin2char startup.tcl
	./bin2char startup_tcl < $(srcdir)/startup.tcl > startup_tcl.c

# add startup_tcl.c to make clean list
CLEANFILES = startup_tcl.c
