INCLUDES = -I$(top_srcdir)/src/helper -I$(top_srcdir)/src/jtag -I$(top_srcdir)/src/target $(all_includes)
AM_CPPFLAGS = -DPKGLIBDIR=\"$(pkglibdir)\" @CPPFLAGS@
METASOURCES = AUTO
noinst_LIBRARIES = libflash.a
libflash_a_SOURCES = flash.c lpc2000.c cfi.c non_cfi.c at91sam7.c str7x.c str9x.c nand.c lpc3180_nand_controller.c \
					 stellaris.c str9xpec.c stm32x.c
noinst_HEADERS = flash.h lpc2000.h cfi.h non_cfi.h at91sam7.h str7x.h str9x.h nand.h lpc3180_nand_controller.h \
				 stellaris.h str9xpec.h stm32x.h
