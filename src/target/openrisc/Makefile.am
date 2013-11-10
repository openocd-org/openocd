include $(top_srcdir)/common.mk

noinst_LTLIBRARIES = libopenrisc.la
libopenrisc_la_SOURCES = $(OPENRISC_SRC)

OPENRISC_SRC = \
	or1k.c \
	or1k_du_adv.c \
	or1k_tap_mohor.c \
	or1k_tap_vjtag.c \
	or1k_tap_xilinx_bscan.c

noinst_HEADERS = \
	or1k.h \
	or1k_du.h \
	or1k_tap.h
