
# common flags used in openocd build
AM_CPPFLAGS = -I$(top_srcdir)/src \
			  -I$(top_builddir)/src \
			  -DPKGDATADIR=\"$(pkgdatadir)\" \
			  -DPKGLIBDIR=\"$(pkglibdir)\"

if INTERNAL_JIMTCL
AM_CPPFLAGS += -I$(top_srcdir)/jimtcl \
			   -I$(top_builddir)/jimtcl
endif
