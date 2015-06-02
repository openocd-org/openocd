AC_PREREQ(2.64)
AC_INIT([openocd], [0.10.0-dev],
  [OpenOCD Mailing List <openocd-devel@lists.sourceforge.net>])
AC_CONFIG_SRCDIR([src/openocd.c])

m4_include([config_subdir.m4])dnl

# check for makeinfo before calling AM_INIT_AUTOMAKE
AC_CHECK_PROG([MAKEINFO], [makeinfo], [makeinfo])
if test "x$MAKEINFO" = "x"; then
  MAKEINFO='echo makeinfo missing; true'
  AC_MSG_WARN([Info documentation will not be built.])
fi
AC_SUBST([MAKEINFO])

AM_INIT_AUTOMAKE([-Wall -Wno-portability dist-bzip2 dist-zip subdir-objects])

AC_CONFIG_HEADERS([config.h])
AH_BOTTOM([
#include <helper/system.h>
#include <helper/types.h>
#include <helper/replacements.h>
])

AC_LANG_C
AC_PROG_CC
AC_PROG_CC_C99
AM_PROG_CC_C_O
AC_PROG_RANLIB
PKG_PROG_PKG_CONFIG([0.23])

dnl disable checks for C++, Fortran and GNU Java Compiler
m4_defun([_LT_AC_LANG_CXX_CONFIG], [:])
m4_defun([_LT_AC_LANG_F77_CONFIG], [:])
m4_defun([_LT_AC_LANG_GCJ_CONFIG], [:])
AC_DISABLE_SHARED
AC_PROG_LIBTOOL
AC_SUBST([LIBTOOL_DEPS])

dnl configure checks required for Jim files (these are obsolete w/ C99)
AC_C_CONST
AC_TYPE_LONG_LONG_INT

AC_SEARCH_LIBS([ioperm], [ioperm])
AC_SEARCH_LIBS([dlopen], [dl])

AC_CHECK_HEADERS([sys/socket.h])
AC_CHECK_HEADERS([arpa/inet.h], [], [], [dnl
#include <stdio.h>
#ifdef STDC_HEADERS
# include <stdlib.h>
# include <stddef.h>
#else
# ifdef HAVE_STDLIB_H
#  include <stdlib.h>
# endif
#endif
#ifdef HAVE_SYS_SOCKET_H
# include <sys/socket.h>
#endif
])
AC_CHECK_HEADERS([elf.h])
AC_CHECK_HEADERS([dirent.h])
AC_CHECK_HEADERS([fcntl.h])
AC_CHECK_HEADERS([ifaddrs.h], [], [], [dnl
#include <stdio.h>
#ifdef STDC_HEADERS
# include <stdlib.h>
# include <stddef.h>
#else
# ifdef HAVE_STDLIB_H
#  include <stdlib.h>
# endif
#endif
#ifdef HAVE_SYS_SOCKET_H
# include <sys/socket.h>
#endif
])
AC_CHECK_HEADERS([malloc.h])
AC_CHECK_HEADERS([netdb.h])
AC_CHECK_HEADERS([netinet/in.h], [], [], [dnl
#include <stdio.h>
#ifdef STDC_HEADERS
# include <stdlib.h>
# include <stddef.h>
#else
# ifdef HAVE_STDLIB_H
#  include <stdlib.h>
# endif
#endif
#ifdef HAVE_SYS_SOCKET_H
# include <sys/socket.h>
#endif
])
AC_CHECK_HEADERS([netinet/tcp.h], [], [], [dnl
#include <stdio.h>
#ifdef STDC_HEADERS
# include <stdlib.h>
# include <stddef.h>
#else
# ifdef HAVE_STDLIB_H
#  include <stdlib.h>
# endif
#endif
#ifdef HAVE_SYS_SOCKET_H
# include <sys/socket.h>
#endif
])
AC_CHECK_HEADERS([poll.h])
AC_CHECK_HEADERS([pthread.h])
AC_CHECK_HEADERS([strings.h])
AC_CHECK_HEADERS([sys/ioctl.h])
AC_CHECK_HEADERS([sys/param.h])
AC_CHECK_HEADERS([sys/select.h])
AC_CHECK_HEADERS([sys/stat.h])
AC_CHECK_HEADERS([sys/time.h])
AC_CHECK_HEADERS([sys/types.h])
AC_CHECK_HEADERS([unistd.h])
AC_CHECK_HEADERS([net/if.h], [], [], [dnl
#include <stdio.h>
#ifdef STDC_HEADERS
# include <stdlib.h>
# include <stddef.h>
#else
# ifdef HAVE_STDLIB_H
#  include <stdlib.h>
# endif
#endif
#ifdef HAVE_SYS_SOCKET_H
# include <sys/socket.h>
#endif
])

AC_HEADER_ASSERT
AC_HEADER_STDBOOL
AC_HEADER_TIME

AC_C_BIGENDIAN

AC_CHECK_FUNCS([strndup])
AC_CHECK_FUNCS([strnlen])
AC_CHECK_FUNCS([gettimeofday])
AC_CHECK_FUNCS([usleep])
AC_CHECK_FUNCS([vasprintf])

build_bitbang=no
build_bitq=no
is_cygwin=no
is_mingw=no
is_win32=no
is_darwin=no

# guess-rev.sh only exists in the repository, not in the released archives
AC_MSG_CHECKING([whether to build a release])
if test -x $srcdir/guess-rev.sh ; then
  build_release=no
else
  build_release=yes
fi
AC_MSG_RESULT([$build_release])

# Adapter drivers
# 1st column -- configure option
# 2nd column -- description
# 3rd column -- symbol used for both config.h and automake
m4_define([ADAPTER_ARG], [m4_argn([1], $1)])
m4_define([ADAPTER_DESC], [m4_argn([2], $1)])
m4_define([ADAPTER_SYM], [m4_argn([3], $1)])
m4_define([ADAPTER_VAR], [enable_[]ADAPTER_ARG($1)])
m4_define([ADAPTER_OPT], [m4_translit(ADAPTER_ARG($1), [_], [-])])

m4_define([USB1_ADAPTERS],
	[[[ftdi], [MPSSE mode of FTDI based devices], [FTDI]],
	[[stlink], [ST-Link JTAG Programmer], [HLADAPTER_STLINK]],
	[[ti_icdi], [TI ICDI JTAG Programmer], [HLADAPTER_ICDI]],
	[[ulink], [Keil ULINK JTAG Programmer], [ULINK]],
	[[usb_blaster_2], [Altera USB-Blaster II Compatible], [USB_BLASTER_2]],
	[[vsllink], [Versaloon-Link JTAG Programmer], [VSLLINK]]])

m4_define([USB_ADAPTERS],
	[[[osbdm], [OSBDM (JTAG only) Programmer], [OSBDM]],
	[[opendous], [eStick/opendous JTAG Programmer], [OPENDOUS]],
	[[aice], [Andes JTAG Programmer], [AICE]]])

m4_define([USB0_ADAPTERS],
	[[[usbprog], [USBProg JTAG Programmer], [USBPROG]],
	[[rlink], [Raisonance RLink JTAG Programmer], [RLINK]],
	[[armjtagew], [Olimex ARM-JTAG-EW Programmer], [ARMJTAGEW]]])

m4_define([HIDAPI_ADAPTERS],
	[[[cmsis_dap], [CMSIS-DAP Compliant Debugger], [CMSIS_DAP]]])

m4_define([LIBJAYLINK_ADAPTERS],
	[[[jlink], [SEGGER J-Link Programmer], [JLINK]]])


AC_ARG_ENABLE([doxygen-html],
  AS_HELP_STRING([--disable-doxygen-html],
    [Disable building Doxygen manual as HTML.]),
  [doxygen_as_html=$enableval], [doxygen_as_html=yes])
AC_SUBST([doxygen_as_html])
AC_MSG_CHECKING([whether to build Doxygen as HTML])
AC_MSG_RESULT([$doxygen_as_html])

AC_ARG_ENABLE([doxygen-pdf],
  AS_HELP_STRING([--enable-doxygen-pdf],
    [Enable building Doxygen manual as PDF.]),
  [doxygen_as_pdf=$enableval], [doxygen_as_pdf=no])
AC_SUBST([doxygen_as_pdf])
AC_MSG_CHECKING([whether to build Doxygen as PDF])
AC_MSG_RESULT([$doxygen_as_pdf])

AC_ARG_ENABLE([gccwarnings],
  AS_HELP_STRING([--disable-gccwarnings], [Disable compiler warnings]),
  [gcc_warnings=$enableval], [gcc_warnings=yes])

AC_ARG_ENABLE([wextra],
  AS_HELP_STRING([--disable-wextra], [Disable extra compiler warnings]),
  [gcc_wextra=$enableval], [gcc_wextra=$gcc_warnings])

AC_ARG_ENABLE([werror],
  AS_HELP_STRING([--disable-werror], [Do not treat warnings as errors]),
  [gcc_werror=$enableval], [gcc_werror=$gcc_warnings])

# set default verbose options, overridden by following options
debug_jtag_io=no
debug_usb_io=no
debug_usb_comms=no

AC_ARG_ENABLE([verbose],
  AS_HELP_STRING([--enable-verbose],
      [Enable verbose JTAG I/O messages (for debugging).]),
  [
  debug_jtag_io=$enableval
  debug_usb_io=$enableval
  debug_usb_comms=$enableval
  ], [])

AC_ARG_ENABLE([verbose_jtag_io],
  AS_HELP_STRING([--enable-verbose-jtag-io],
      [Enable verbose JTAG I/O messages (for debugging).]),
  [debug_jtag_io=$enableval], [])

AC_ARG_ENABLE([verbose_usb_io],
  AS_HELP_STRING([--enable-verbose-usb-io],
      [Enable verbose USB I/O messages (for debugging)]),
  [debug_usb_io=$enableval], [])

AC_ARG_ENABLE([verbose_usb_comms],
  AS_HELP_STRING([--enable-verbose-usb-comms],
      [Enable verbose USB communication messages (for debugging)]),
  [debug_usb_comms=$enableval], [])

AC_MSG_CHECKING([whether to enable verbose JTAG I/O messages]);
AC_MSG_RESULT([$debug_jtag_io])
if test $debug_jtag_io = yes; then
  AC_DEFINE([_DEBUG_JTAG_IO_],[1], [Print verbose JTAG I/O messages])
fi

AC_MSG_CHECKING([whether to enable verbose USB I/O messages]);
AC_MSG_RESULT([$debug_usb_io])
if test $debug_usb_io = yes; then
  AC_DEFINE([_DEBUG_USB_IO_],[1], [Print verbose USB I/O messages])
fi

AC_MSG_CHECKING([whether to enable verbose USB communication messages]);
AC_MSG_RESULT([$debug_usb_comms])
if test $debug_usb_comms = yes; then
  AC_DEFINE([_DEBUG_USB_COMMS_],[1], [Print verbose USB communication messages])
fi

debug_malloc=no
AC_ARG_ENABLE([malloc_logging],
  AS_HELP_STRING([--enable-malloc-logging],
      [Include free space in logging messages (requires malloc.h).]),
  [debug_malloc=$enableval], [])

AC_MSG_CHECKING([whether to enable malloc free space logging]);
AC_MSG_RESULT([$debug_malloc])
if test $debug_malloc = yes; then
  AC_DEFINE([_DEBUG_FREE_SPACE_],[1], [Include malloc free space in logging])
fi

AC_ARG_ENABLE([dummy],
  AS_HELP_STRING([--enable-dummy], [Enable building the dummy port driver]),
  [build_dummy=$enableval], [build_dummy=no])

m4_define([AC_ARG_ADAPTERS], [
  m4_foreach([adapter], [$1],
	[AC_ARG_ENABLE(ADAPTER_OPT([adapter]),
		AS_HELP_STRING([--enable-ADAPTER_OPT([adapter])],
			[Enable building support for the ]ADAPTER_DESC([adapter])[ (default is $2)]),
		[], [ADAPTER_VAR([adapter])=$2])
  ])
])

AC_ARG_ADAPTERS([USB1_ADAPTERS, USB_ADAPTERS, USB0_ADAPTERS, HIDAPI_ADAPTERS, LIBJAYLINK_ADAPTERS], [auto])

AC_ARG_ENABLE([parport],
  AS_HELP_STRING([--enable-parport], [Enable building the pc parallel port driver]),
  [build_parport=$enableval], [build_parport=no])

AC_ARG_ENABLE([parport_ppdev],
  AS_HELP_STRING([--disable-parport-ppdev],
      [Disable use of ppdev (/dev/parportN) for parport (for x86 only)]),
    [parport_use_ppdev=$enableval], [parport_use_ppdev=yes])

AC_ARG_ENABLE([parport_giveio],
    AS_HELP_STRING([--enable-parport-giveio],
      [Enable use of giveio for parport (for CygWin only)]),
    [parport_use_giveio=$enableval], [parport_use_giveio=])

AC_ARG_ENABLE([jtag_vpi],
  AS_HELP_STRING([--enable-jtag_vpi], [Enable building support for JTAG VPI]),
  [build_jtag_vpi=$enableval], [build_jtag_vpi=no])

AC_ARG_ENABLE([usb_blaster_libftdi],
  AS_HELP_STRING([--enable-usb_blaster_libftdi], [Enable building support for the Altera USB-Blaster using the libftdi driver]),
  [build_usb_blaster=$enableval], [build_usb_blaster=no])

AC_ARG_ENABLE([amtjtagaccel],
  AS_HELP_STRING([--enable-amtjtagaccel], [Enable building the Amontec JTAG-Accelerator driver]),
  [build_amtjtagaccel=$enableval], [build_amtjtagaccel=no])

AC_ARG_ENABLE([zy1000_master],
  AS_HELP_STRING([--enable-zy1000-master], [Use ZY1000 JTAG master registers]),
  [build_zy1000_master=$enableval], [build_zy1000_master=no])

AC_ARG_ENABLE([zy1000],
  AS_HELP_STRING([--enable-zy1000], [Enable ZY1000 interface]),
  [build_zy1000=$enableval], [build_zy1000=no])

AC_ARG_ENABLE([ioutil],
  AS_HELP_STRING([--enable-ioutil], [Enable ioutil functions - useful for standalone OpenOCD implementations]),
  [build_ioutil=$enableval], [build_ioutil=no])

case "${host_cpu}" in
  arm*)
    AC_ARG_ENABLE([ep93xx],
      AS_HELP_STRING([--enable-ep93xx], [Enable building support for EP93xx based SBCs]),
      [build_ep93xx=$enableval], [build_ep93xx=no])

    AC_ARG_ENABLE([at91rm9200],
      AS_HELP_STRING([--enable-at91rm9200], [Enable building support for AT91RM9200 based SBCs]),
      [build_at91rm9200=$enableval], [build_at91rm9200=no])

    AC_ARG_ENABLE([bcm2835gpio],
      AS_HELP_STRING([--enable-bcm2835gpio], [Enable building support for bitbanging on BCM2835 (as found in Raspberry Pi)]),
      [build_bcm2835gpio=$enableval], [build_bcm2835gpio=no])
    ;;

  *)
    build_ep93xx=no
    build_at91rm9200=no
    build_bcm2835gpio=no
    ;;
esac

AC_ARG_ENABLE([gw16012],
  AS_HELP_STRING([--enable-gw16012], [Enable building support for the Gateworks GW16012 JTAG Programmer]),
  [build_gw16012=$enableval], [build_gw16012=no])

AC_ARG_ENABLE([presto_libftdi],
  AS_HELP_STRING([--enable-presto_libftdi], [Enable building support for ASIX Presto Programmer using the libftdi library]),
  [build_presto=$enableval], [build_presto=no])

AC_ARG_ENABLE([openjtag_ftdi],
  AS_HELP_STRING([--enable-openjtag_ftdi], [Enable building support for the OpenJTAG Programmer with libftdi library]),
  [build_openjtag=$enableval], [build_openjtag=no])

AC_ARG_ENABLE([oocd_trace],
  AS_HELP_STRING([--enable-oocd_trace],
  [Enable building support for some prototype OpenOCD+trace ETM capture hardware]),
  [build_oocd_trace=$enableval], [build_oocd_trace=no])

AC_ARG_ENABLE([buspirate],
  AS_HELP_STRING([--enable-buspirate], [Enable building support for the Buspirate]),
  [build_buspirate=$enableval], [build_buspirate=no])

AC_ARG_ENABLE([sysfsgpio],
  AS_HELP_STRING([--enable-sysfsgpio], [Enable building support for programming driven via sysfs gpios.]),
  [build_sysfsgpio=$enableval], [build_sysfsgpio=no])

AC_ARG_ENABLE([minidriver_dummy],
  AS_HELP_STRING([--enable-minidriver-dummy], [Enable the dummy minidriver.]),
  [build_minidriver_dummy=$enableval], [build_minidriver_dummy=no])

AC_ARG_ENABLE([internal-jimtcl],
  AS_HELP_STRING([--disable-internal-jimtcl], [Disable building internal jimtcl]),
  [use_internal_jimtcl=$enableval], [use_internal_jimtcl=yes])

AC_ARG_ENABLE([internal-libjaylink],
  AS_HELP_STRING([--disable-internal-libjaylink],
  [Disable building internal libjaylink]),
  [use_internal_libjaylink=$enableval], [use_internal_libjaylink=yes])

build_minidriver=no
AC_MSG_CHECKING([whether to enable ZY1000 minidriver])
if test $build_zy1000 = yes; then
  if test $build_minidriver = yes; then
    AC_MSG_ERROR([Multiple minidriver options have been enabled.])
  fi
  AC_DEFINE([HAVE_JTAG_MINIDRIVER_H], [1],
      [Define to 1 if you have the <jtag_minidriver.h> header file.])
  build_minidriver=yes
fi
AC_MSG_RESULT([$build_zy1000])

AC_ARG_ENABLE([remote-bitbang],
  AS_HELP_STRING([--enable-remote-bitbang], [Enable building support for the Remote Bitbang jtag driver]),
  [build_remote_bitbang=$enableval], [build_remote_bitbang=no])

AC_MSG_CHECKING([whether to enable dummy minidriver])
if test $build_minidriver_dummy = yes; then
  if test $build_minidriver = yes; then
    AC_MSG_ERROR([Multiple minidriver options have been enabled.])
  fi
  build_minidriver=yes
  AC_DEFINE([BUILD_MINIDRIVER_DUMMY], [1], [Use the dummy minidriver.])
  AC_DEFINE([HAVE_JTAG_MINIDRIVER_H], [1],
      [Define to 1 if you have the <jtag_minidriver.h> header file.])
fi
AC_MSG_RESULT([$build_minidriver_dummy])

AC_MSG_CHECKING([whether standard drivers can be built])
if test "$build_minidriver" = yes; then
  AC_MSG_RESULT([no])
  AC_MSG_WARN([Using the minidriver disables all other drivers.])
  sleep 2
else
  AC_MSG_RESULT([yes])
fi

case "${host_cpu}" in
  i?86|x86*)
    ;;
  *)
    if test x$parport_use_ppdev = xno; then
      AC_MSG_WARN([--disable-parport-ppdev is not supported by the host CPU])
    fi
    parport_use_ppdev=yes
    ;;
esac

case $host in
  *-cygwin*)
    is_win32=yes
    parport_use_ppdev=no

    AC_COMPILE_IFELSE([AC_LANG_PROGRAM([], [[return __MINGW32__;]])],
      [is_mingw=yes],[is_mingw=no])
    if test $is_mingw = yes; then
      AC_DEFINE([IS_MINGW], [1], [1 if building for MinGW.])
      if test x$parport_use_giveio = xno; then
        AC_MSG_WARN([--disable-parport-giveio is not supported by MinGW32 hosts])
      fi
      parport_use_giveio=yes
      is_cygwin=no
    else
      is_cygwin=yes
      AC_DEFINE([IS_CYGWIN], [1], [1 if building for Cygwin.])
      # sys/io.h needed under cygwin for parport access
      if test $build_parport = yes; then
        AC_CHECK_HEADERS([sys/io.h],[],AC_MSG_ERROR([Please install the cygwin ioperm package]))
      fi
    fi

    AC_DEFINE([IS_WIN32], [1], [1 if building for Win32.])
    AC_DEFINE([IS_DARWIN], [0], [0 if not building for Darwin.])
    ;;
  *-mingw* | *-msys*)
    is_mingw=yes
    is_win32=yes
    parport_use_ppdev=no

    if test x$parport_use_giveio = xno; then
      AC_MSG_WARN([--disable-parport-giveio is not supported by MinGW32 hosts])
    fi
    parport_use_giveio=yes

    if test x$build_buspirate = xyes; then
      AC_MSG_ERROR([buspirate currently not supported by MinGW32 hosts])
    fi

    CFLAGS="$CFLAGS -D__USE_MINGW_ANSI_STDIO"

    AC_DEFINE([IS_MINGW], [1], [1 if building for MinGW.])
    AC_DEFINE([IS_WIN32], [1], [1 if building for Win32.])
    AC_DEFINE([IS_DARWIN], [0], [0 if not building for Darwin.])
    ;;
  *darwin*)
    is_darwin=yes

    if test x$parport_use_giveio = xyes; then
      AC_MSG_WARN([--enable-parport-giveio cannot be used by Darwin hosts])
    fi
    parport_use_giveio=no

    AC_DEFINE([IS_CYGWIN], [0], [0 if not building for Cygwin.])
    AC_DEFINE([IS_WIN32], [0], [0 if not building for Win32.])
    AC_DEFINE([IS_DARWIN], [1], [1 if building for Darwin.])
    ;;
  *)
    if test x$parport_use_giveio = xyes; then
      AC_MSG_WARN([--enable-parport-giveio cannot be used by ]$host[ hosts])
    fi
    parport_use_giveio=no
    AC_DEFINE([IS_CYGWIN], [0], [0 if not building for Cygwin.])
    AC_DEFINE([IS_WIN32], [0], [0 if not building for Win32.])
    AC_DEFINE([IS_DARWIN], [0], [0 if not building for Darwin.])
    ;;
esac

if test $is_win32 = yes; then
    AC_DEFINE([WIN32_LEAN_AND_MEAN], [1], [1 to exclude old conflicting definitions when building on Windows])
fi

if test $build_parport = yes; then
  build_bitbang=yes
  AC_DEFINE([BUILD_PARPORT], [1], [1 if you want parport.])
else
  AC_DEFINE([BUILD_PARPORT], [0], [0 if you don't want parport.])
fi

if test $build_dummy = yes; then
  build_bitbang=yes
  AC_DEFINE([BUILD_DUMMY], [1], [1 if you want dummy driver.])
else
  AC_DEFINE([BUILD_DUMMY], [0], [0 if you don't want dummy driver.])
fi

if test $build_ep93xx = yes; then
  build_bitbang=yes
  AC_DEFINE([BUILD_EP93XX], [1], [1 if you want ep93xx.])
else
  AC_DEFINE([BUILD_EP93XX], [0], [0 if you don't want ep93xx.])
fi

if test $build_zy1000 = yes; then
  AC_DEFINE([BUILD_ZY1000], [1], [1 if you want ZY1000.])
else
  AC_DEFINE([BUILD_ZY1000], [0], [0 if you don't want ZY1000.])
fi

if test $build_zy1000_master = yes; then
  AC_DEFINE([BUILD_ZY1000_MASTER], [1], [1 if you want ZY1000 JTAG master registers.])
else
  AC_DEFINE([BUILD_ZY1000_MASTER], [0], [0 if you don't want ZY1000 JTAG master registers.])
fi

if test $build_at91rm9200 = yes; then
  build_bitbang=yes
  AC_DEFINE([BUILD_AT91RM9200], [1], [1 if you want at91rm9200.])
else
  AC_DEFINE([BUILD_AT91RM9200], [0], [0 if you don't want at91rm9200.])
fi

if test $build_bcm2835gpio = yes; then
  build_bitbang=yes
  AC_DEFINE([BUILD_BCM2835GPIO], [1], [1 if you want bcm2835gpio.])
else
  AC_DEFINE([BUILD_BCM2835GPIO], [0], [0 if you don't want bcm2835gpio.])
fi

if test x$parport_use_ppdev = xyes; then
  AC_DEFINE([PARPORT_USE_PPDEV], [1], [1 if you want parport to use ppdev.])
else
  AC_DEFINE([PARPORT_USE_PPDEV], [0], [0 if you don't want parport to use ppdev.])
fi

if test x$parport_use_giveio = xyes; then
  AC_DEFINE([PARPORT_USE_GIVEIO], [1], [1 if you want parport to use giveio.])
else
  AC_DEFINE([PARPORT_USE_GIVEIO], [0], [0 if you don't want parport to use giveio.])
fi

if test $build_bitbang = yes; then
  AC_DEFINE([BUILD_BITBANG], [1], [1 if you want a bitbang interface.])
else
  AC_DEFINE([BUILD_BITBANG], [0], [0 if you don't want a bitbang interface.])
fi

if test $build_usb_blaster = yes; then
  AC_DEFINE([BUILD_USB_BLASTER], [1], [1 if you want libftdi usb_blaster.])
else
  AC_DEFINE([BUILD_USB_BLASTER], [0], [0 if you don't want libftdi usb_blaster.])
fi

if test $build_jtag_vpi = yes; then
  AC_DEFINE([BUILD_JTAG_VPI], [1], [1 if you want JTAG VPI.])
else
  AC_DEFINE([BUILD_JTAG_VPI], [0], [0 if you don't want JTAG VPI.])
fi

if test $build_amtjtagaccel = yes; then
  AC_DEFINE([BUILD_AMTJTAGACCEL], [1], [1 if you want the Amontec JTAG-Accelerator driver.])
else
  AC_DEFINE([BUILD_AMTJTAGACCEL], [0], [0 if you don't want the Amontec JTAG-Accelerator driver.])
fi

if test $build_gw16012 = yes; then
  AC_DEFINE([BUILD_GW16012], [1], [1 if you want the Gateworks GW16012 driver.])
else
  AC_DEFINE([BUILD_GW16012], [0], [0 if you don't want the Gateworks GW16012 driver.])
fi

if test $build_presto = yes; then
  build_bitq=yes
  AC_DEFINE([BUILD_PRESTO], [1], [1 if you want the ASIX PRESTO driver using libftdi.])
else
  AC_DEFINE([BUILD_PRESTO], [0], [0 if you don't want the ASIX PRESTO driver using libftdi.])
fi

if test $build_bitq = yes; then
  AC_DEFINE([BUILD_BITQ], [1], [1 if you want a bitq interface.])
else
  AC_DEFINE([BUILD_BITQ], [0], [0 if you don't want a bitq interface.])
fi


if test $build_openjtag = yes; then
  AC_DEFINE([BUILD_OPENJTAG], [1], [1 if you want the OpenJTAG driver.])
else
  AC_DEFINE([BUILD_OPENJTAG], [0], [0 if you don't want the OpenJTAG driver.])
fi

if test $build_oocd_trace = yes; then
  AC_DEFINE([BUILD_OOCD_TRACE], [1], [1 if you want the OpenOCD+trace ETM capture driver.])
else
  AC_DEFINE([BUILD_OOCD_TRACE], [0], [0 if you don't want the OpenOCD+trace ETM capture driver.])
fi

if test $build_buspirate = yes; then
  AC_DEFINE([BUILD_BUSPIRATE], [1], [1 if you want the Buspirate JTAG driver.])
else
  AC_DEFINE([BUILD_BUSPIRATE], [0], [0 if you don't want the Buspirate JTAG driver.])
fi

if test $use_internal_jimtcl = yes; then
  if test -f "$srcdir/jimtcl/configure.ac"; then
    AX_CONFIG_SUBDIR_OPTION([jimtcl], [--disable-install-jim])
  else
    AC_MSG_ERROR([jimtcl not found, run git submodule init and git submodule update.])
  fi
fi

if test $build_remote_bitbang = yes; then
  build_bitbang=yes
  AC_DEFINE([BUILD_REMOTE_BITBANG], [1], [1 if you want the Remote Bitbang JTAG driver.])
else
  AC_DEFINE([BUILD_REMOTE_BITBANG], [0], [0 if you don't want the Remote Bitbang JTAG driver.])
fi

if test $build_sysfsgpio = yes; then
  build_bitbang=yes
  AC_DEFINE([BUILD_SYSFSGPIO], [1], [1 if you want the SysfsGPIO driver.])
else
  AC_DEFINE([BUILD_SYSFSGPIO], [0], [0 if you don't want SysfsGPIO driver.])
fi

if test $build_usb_blaster = yes -o $build_openjtag = yes -o $build_presto = yes; then

  # we can have libftdi or libftdi1, so check it and use the latest one
  PKG_CHECK_MODULES([LIBFTDI], [libftdi1], [use_libftdi=yes], [use_libftdi=no])
  if test $use_libftdi = no; then
	PKG_CHECK_MODULES([LIBFTDI], [libftdi], [use_libftdi=yes], [use_libftdi=no])
  fi
  if test $use_libftdi = no; then
	AC_MSG_ERROR([The libftdi driver is not present on your system.])
  fi

  #
  # Try to build a small program.
  AC_MSG_CHECKING([Build & Link with libftdi...])

  LDFLAGS_SAVE=$LDFLAGS
  CFLAGS_SAVE=$CFLAGS
  LIBS_SAVE=$LIBS
  _LDFLAGS=`eval echo $LDFLAGS`
  _CFLAGS=`eval echo $CFLAGS`
  _LIBS=`eval echo $LIBS`
  LDFLAGS=$_LDFLAGS
  CFLAGS="$_CFLAGS $LIBFTDI_CFLAGS"
  LIBS="$_LIBS $LIBFTDI_LIBS"

  AC_RUN_IFELSE([AC_LANG_PROGRAM([[
#include <stdio.h>
#include <ftdi.h>
  ]], [[
  struct ftdi_context *p;
  p = ftdi_new();
  if( p != NULL ){
      return 0;
  } else {
      fprintf( stderr, "calling ftdi_new() failed\n");
      return 1;
        }
    ]])], [
      AC_MSG_RESULT([Success])
    ], [
      AC_MSG_ERROR([Cannot build & run test program using libftdi])
    ], [
      AC_MSG_RESULT([Skipping as we are cross-compiling, trying build only])
      AC_SEARCH_LIBS([ftdi_new], [], [], [AC_MSG_ERROR([Cannot link with libftdi])])
    ])

AC_MSG_CHECKING([for libftdi FT232H device support])
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[
#include <stdio.h>
#include <ftdi.h>
  ]], [[
enum ftdi_chip_type x = TYPE_232H;
    ]])], [
      AC_DEFINE([HAS_ENUM_FT232H], [1],
        [Support FT232H with libftdi.])
      has_enum_ft232h=yes
    ], [
      has_enum_ft232h=no
    ])
    AC_MSG_RESULT([$has_enum_ft232h])

    if test $has_enum_ft232h = no; then
      AC_MSG_WARN([You need a newer libftdi version (0.20 or later).])
    fi

  # Restore the 'unexpanded ldflags'
  LDFLAGS=$LDFLAGS_SAVE
  CFLAGS=$CFLAGS_SAVE
  LIBS=$LIBS_SAVE
fi

PKG_CHECK_MODULES([LIBUSB1], [libusb-1.0], [
	use_libusb1=yes
	AC_DEFINE([HAVE_LIBUSB1], [1], [Define if you have libusb-1.x])
	PKG_CHECK_EXISTS([libusb-1.0 >= 1.0.9],
		[AC_DEFINE([HAVE_LIBUSB_ERROR_NAME], [1], [Define if your libusb has libusb_error_name()])],
		[AC_MSG_WARN([libusb-1.x older than 1.0.9 detected, consider updating])])
	LIBUSB1_CFLAGS=`echo $LIBUSB1_CFLAGS | sed 's/-I/-isystem /'`
	AC_MSG_NOTICE([libusb-1.0 header bug workaround: LIBUSB1_CFLAGS changed to "$LIBUSB1_CFLAGS"])
	PKG_CHECK_EXISTS([libusb-1.0 >= 1.0.16],
		[AC_DEFINE([HAVE_LIBUSB_GET_PORT_NUMBERS], [1], [Define if your libusb has libusb_get_port_numbers()])])
  ], [
	use_libusb1=no
	AC_MSG_WARN([libusb-1.x not found, trying legacy libusb-0.1 as a fallback; consider installing libusb-1.x instead])
])

PKG_CHECK_MODULES([LIBUSB0], [libusb], [use_libusb0=yes], [use_libusb0=no])

for hidapi_lib in hidapi hidapi-hidraw hidapi-libusb; do
	PKG_CHECK_MODULES([HIDAPI],[$hidapi_lib],[
		use_hidapi=yes
		break
	],[
		use_hidapi=no
	])
done

PKG_CHECK_MODULES([libjaylink], [libjaylink >= 0.1],
	[HAVE_LIBJAYLINK=yes], [HAVE_LIBJAYLINK=no])

m4_define([PROCESS_ADAPTERS], [
  m4_foreach([adapter], [$1], [
	if test $2; then
		if test $ADAPTER_VAR([adapter]) != no; then
			AC_DEFINE([BUILD_]ADAPTER_SYM([adapter]), [1], [1 if you want the ]ADAPTER_DESC([adapter]).)
		else
			AC_DEFINE([BUILD_]ADAPTER_SYM([adapter]), [0], [0 if you do not want the ]ADAPTER_DESC([adapter]).)
		fi
	else
		if test $ADAPTER_VAR([adapter]) = yes; then
			AC_MSG_ERROR([$3 is required for the ADAPTER_DESC([adapter])])
		fi
		ADAPTER_VAR([adapter])=no
	fi
	AM_CONDITIONAL(ADAPTER_SYM([adapter]), [test $ADAPTER_VAR([adapter]) != no])
  ])
])

PROCESS_ADAPTERS([USB1_ADAPTERS], [$use_libusb1 = yes], [libusb-1.x])
PROCESS_ADAPTERS([USB_ADAPTERS], [$use_libusb1 = yes -o $use_libusb0 = yes], [libusb-1.x or libusb-0.1])
PROCESS_ADAPTERS([USB0_ADAPTERS], [$use_libusb0 = yes], [libusb-0.1])
PROCESS_ADAPTERS([HIDAPI_ADAPTERS], [$use_hidapi = yes], [hidapi])
PROCESS_ADAPTERS([LIBJAYLINK_ADAPTERS], [$use_libusb1 = yes -a $use_internal_libjaylink = yes -o $HAVE_LIBJAYLINK = yes], [libusb-1.x or libjaylink-0.1])

if test $build_openjtag = yes; then
	if test $use_libusb1 != yes -a $use_libusb0 != yes; then
		AC_MSG_ERROR([libusb-1.x or libusb-0.1 is required for the OpenJTAG Programmer])
		build_openjtag=no
	fi
fi

if test $enable_stlink != no -o $enable_ti_icdi != no; then
	AC_DEFINE([BUILD_HLADAPTER], [1], [1 if you want the High Level JTAG driver.])
else
	AC_DEFINE([BUILD_HLADAPTER], [0], [0 if you want the High Level JTAG driver.])
fi
AM_CONDITIONAL([HLADAPTER], [test $enable_stlink != no -o $enable_ti_icdi != no])

if test $enable_jlink != no; then
  if test $use_internal_libjaylink = yes; then
    if test -f "$srcdir/src/jtag/drivers/libjaylink/configure.ac"; then
      AX_CONFIG_SUBDIR_OPTION([src/jtag/drivers/libjaylink],
		[--enable-subproject-build])
    else
      AC_MSG_ERROR([Internal libjaylink not found, run either 'git submodule init' and 'git submodule update' or disable internal libjaylink with --disable-internal-libjaylink.])
    fi
  fi
fi

AM_CONDITIONAL([RELEASE], [test $build_release = yes])
AM_CONDITIONAL([PARPORT], [test $build_parport = yes])
AM_CONDITIONAL([DUMMY], [test $build_dummy = yes])
AM_CONDITIONAL([GIVEIO], [test x$parport_use_giveio = xyes])
AM_CONDITIONAL([EP93XX], [test $build_ep93xx = yes])
AM_CONDITIONAL([ZY1000], [test $build_zy1000 = yes])
AM_CONDITIONAL([ZY1000_MASTER], [test $build_zy1000_master = yes])
AM_CONDITIONAL([IOUTIL], [test $build_ioutil = yes])
AM_CONDITIONAL([AT91RM9200], [test $build_at91rm9200 = yes])
AM_CONDITIONAL([BCM2835GPIO], [test $build_bcm2835gpio = yes])
AM_CONDITIONAL([BITBANG], [test $build_bitbang = yes])
AM_CONDITIONAL([JTAG_VPI], [test $build_jtag_vpi = yes -o $build_jtag_vpi = yes])
AM_CONDITIONAL([USB_BLASTER_DRIVER], [test $build_usb_blaster = yes -o $enable_usb_blaster_2 != no])
AM_CONDITIONAL([USB_BLASTER], [test $build_usb_blaster = yes])
AM_CONDITIONAL([AMTJTAGACCEL], [test $build_amtjtagaccel = yes])
AM_CONDITIONAL([GW16012], [test $build_gw16012 = yes])
AM_CONDITIONAL([PRESTO], [test $build_presto = yes])
AM_CONDITIONAL([OPENJTAG], [test $build_openjtag = yes])
AM_CONDITIONAL([OOCD_TRACE], [test $build_oocd_trace = yes])
AM_CONDITIONAL([REMOTE_BITBANG], [test $build_remote_bitbang = yes])
AM_CONDITIONAL([BUSPIRATE], [test $build_buspirate = yes])
AM_CONDITIONAL([SYSFSGPIO], [test $build_sysfsgpio = yes])
AM_CONDITIONAL([USE_LIBUSB0], [test $use_libusb0 = yes])
AM_CONDITIONAL([USE_LIBUSB1], [test $use_libusb1 = yes])
AM_CONDITIONAL([IS_CYGWIN], [test $is_cygwin = yes])
AM_CONDITIONAL([IS_MINGW], [test $is_mingw = yes])
AM_CONDITIONAL([IS_WIN32], [test $is_win32 = yes])
AM_CONDITIONAL([IS_DARWIN], [test $is_darwin = yes])
AM_CONDITIONAL([BITQ], [test $build_bitq = yes])
AM_CONDITIONAL([CMSIS_DAP], [test $use_hidapi = yes])

AM_CONDITIONAL([MINIDRIVER], [test $build_minidriver = yes])
AM_CONDITIONAL([MINIDRIVER_DUMMY], [test $build_minidriver_dummy = yes])

AM_CONDITIONAL([INTERNAL_JIMTCL], [test $use_internal_jimtcl = yes])
AM_CONDITIONAL([INTERNAL_LIBJAYLINK], [test $use_internal_libjaylink = yes])

# Look for environ alternatives.  Possibility #1: is environ in unistd.h or stdlib.h?
AC_MSG_CHECKING([for environ in unistd.h and stdlib.h])
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[
#define _GNU_SOURCE
#include <unistd.h>
#include <stdlib.h>
  ]], [[char **ep = environ;]]
  )], [
    AC_MSG_RESULT([yes])
    has_environ=yes
  ], [
    AC_MSG_RESULT([no])

    # Possibility #2: can environ be found in an available library?
    AC_MSG_CHECKING([for extern environ])
    AC_LINK_IFELSE([AC_LANG_PROGRAM([[
    extern char **environ;
    ]], [[char **ep = environ;]]
      )], [
        AC_DEFINE(NEED_ENVIRON_EXTERN, [1], [Must declare 'environ' to use it.])
        has_environ=yes
      ], [
        has_environ=no
      ])
    AC_MSG_RESULT([${has_environ}])
  ])

if test "${has_environ}" != "yes" ; then
  AC_MSG_FAILURE([Could not find 'environ' in unistd.h or available libraries.])
fi

AC_DEFINE([_GNU_SOURCE],[1],[Use GNU C library extensions (e.g. stdndup).])

# set default gcc warnings
GCC_WARNINGS="-Wall -Wstrict-prototypes -Wformat-security -Wshadow"
if test "${gcc_wextra}" = yes; then
  GCC_WARNINGS="${GCC_WARNINGS} -Wextra -Wno-unused-parameter"
  GCC_WARNINGS="${GCC_WARNINGS} -Wbad-function-cast"
  GCC_WARNINGS="${GCC_WARNINGS} -Wcast-align"
  GCC_WARNINGS="${GCC_WARNINGS} -Wredundant-decls"
fi
if test "${gcc_werror}" = yes; then
  GCC_WARNINGS="${GCC_WARNINGS} -Werror"
fi

# overide default gcc cflags
if test $gcc_warnings = yes; then
  CFLAGS="$CFLAGS $GCC_WARNINGS"
fi

AC_CONFIG_FILES([
  Makefile
  src/Makefile
  src/helper/Makefile
  src/jtag/Makefile
  src/jtag/drivers/Makefile
  src/jtag/drivers/usb_blaster/Makefile
  src/jtag/hla/Makefile
  src/jtag/aice/Makefile
  src/transport/Makefile
  src/target/openrisc/Makefile
  src/xsvf/Makefile
  src/svf/Makefile
  src/target/Makefile
  src/rtos/Makefile
  src/server/Makefile
  src/flash/Makefile
  src/flash/nor/Makefile
  src/flash/nand/Makefile
  src/pld/Makefile
  doc/Makefile
])
AC_OUTPUT

echo
echo
echo OpenOCD configuration summary
echo --------------------------------------------------
m4_foreach([adapter], [USB1_ADAPTERS, USB_ADAPTERS, USB0_ADAPTERS,
	HIDAPI_ADAPTERS, LIBJAYLINK_ADAPTERS],
	[s=m4_format(["%-40s"], ADAPTER_DESC([adapter]))
	case $ADAPTER_VAR([adapter]) in
		auto)
			echo "$s"yes '(auto)'
			;;
		yes)
			echo "$s"yes
			;;
		no)
			echo "$s"no
			;;
	esac
])
echo
