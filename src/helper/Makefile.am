INCLUDES = $(all_includes)
METASOURCES = AUTO
AM_CPPFLAGS = -DPKGDATADIR=\"$(pkgdatadir)\" -DPKGLIBDIR=\"$(pkglibdir)\" @CPPFLAGS@
noinst_LIBRARIES = libhelper.a

if ECOSBOARD
CONFIGFILES = 
else
CONFIGFILES = options.c
endif

libhelper_a_SOURCES = binarybuffer.c $(CONFIGFILES) configuration.c log.c command.c time_support.c \
	replacements.c fileio.c
noinst_HEADERS = binarybuffer.h configuration.h types.h log.h command.h \
	time_support.h replacements.h fileio.h
