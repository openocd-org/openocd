INCLUDES = -I$(top_srcdir)/src $(all_includes) -I$(top_srcdir)/src/target
METASOURCES = AUTO
AM_CPPFLAGS = -DPKGDATADIR=\"$(pkgdatadir)\" -DPKGLIBDIR=\"$(pkglibdir)\" @CPPFLAGS@
noinst_LIBRARIES = libhelper.a

if ECOSBOARD
CONFIGFILES = 
else
CONFIGFILES = options.c jim.c
endif

libhelper_a_SOURCES = binarybuffer.c $(CONFIGFILES) configuration.c log.c command.c time_support.c \
	replacements.c fileio.c startup_tcl.c

noinst_HEADERS = binarybuffer.h configuration.h types.h log.h command.h \
	time_support.h replacements.h fileio.h jim.h


# Convert .tcl to cfile
#
# Fix!!! how do we make this more automake friendly?
# 
startup_tcl.c: startup.tcl
	gcc -o bin2char $(srcdir)/bin2char.c 
	./bin2char startup_tcl < $(srcdir)/startup.tcl > $@

# add startup_tcl.c to make clean list
CLEANFILES = startup_tcl.c
