# not a GNU package. You can remove this line, if
# have all needed files, that a GNU package needs
AUTOMAKE_OPTIONS = foreign 1.6

nobase_dist_pkgdata_DATA = \
	contrib/libdcc/dcc_stdio.c \
	contrib/libdcc/dcc_stdio.h \
	contrib/libdcc/example.c \
	contrib/libdcc/README \
	contrib/openocd.udev

SUBDIRS = src doc

EXTRA_DIST = Doxyfile

docs: pdf html doxygen

doxygen::
	doxygen Doxyfile 2>&1 | perl tools/logger.pl > doxygen.log

doxygen-clean:
	rm -f -r doxygen doxygen.log

MAINTAINERCLEANFILES = \
	configure \
	Makefile.in \
	depcomp \
	config.guess \
	config.sub \
	config.h.in \
	config.h.in~ \
	compile \
	ltmain.sh \
	missing \
	aclocal.m4 \
	install-sh
