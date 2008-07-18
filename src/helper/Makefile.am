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

noinst_PROGRAMS = bin2char

bin2char_SOURCES = bin2char.c

# Convert .tcl to cfile
startup_tcl.c: startup.tcl bin2char$(EXEEXT)
	./bin2char$(EXEEXT) startup_tcl < $(srcdir)/$< > $@

# add startup_tcl.c to make clean list
CLEANFILES = startup_tcl.c
