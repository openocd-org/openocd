AC_PREREQ(2.64)
AC_INIT([openocd], [0.8.0],
  [OpenOCD Mailing List <openocd-devel@lists.sourceforge.net>])
AC_CONFIG_SRCDIR([src/openocd.c])

m4_include([config_subdir.m4])dnl

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
AC_CHECK_HEADERS([pthread.h])
AC_CHECK_HEADERS([strings.h])
AC_CHECK_HEADERS([sys/ioctl.h])
AC_CHECK_HEADERS([sys/param.h])
AC_CHECK_HEADERS([sys/poll.h])
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

AC_ARG_WITH(ftd2xx,
   AS_HELP_STRING([--with-ftd2xx=<PATH>],[This option has been removed.]),
  [
# Option Given.
cat << __EOF__

The option: --with-ftd2xx=<PATH> has been removed.
On Linux, the new option is:

  --with-ftd2xx-linux-tardir=/path/to/files

Where <path> is the path the the directory where the "tar.gz" file
from FTDICHIP.COM was unpacked, for example:

  --with-ftd2xx-linux-tardir=${HOME}/libftd2xx0.4.16

On Cygwin/MingW32, the new option is:

  --with-ftd2xx-win32-zipdir=/path/to/files

Where <path> is the path to the directory where the "zip" file from
FTDICHIP.COM was unpacked, for example:

  --with-ftd2xx-win32-zipdir=${HOME}/ftd2xx.cdm.files

__EOF__

   AC_MSG_ERROR([Sorry Cannot continue])
  ], [true])

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
	[[usb_blaster_2], [Altera USB-Blaster II Compatible], [USB_BLASTER_2]]])

m4_define([USB_ADAPTERS],
	[[[jlink], [Segger J-Link JTAG Programmer], [JLINK]],
	[[osbdm], [OSBDM (JTAG only) Programmer], [OSBDM]],
	[[opendous], [eStick/opendous JTAG Programmer], [OPENDOUS]],
	[[aice], [Andes JTAG Programmer], [AICE]]])

m4_define([USB0_ADAPTERS],
	[[[vsllink], [Versaloon-Link JTAG Programmer], [VSLLINK]],
	[[usbprog], [USBProg JTAG Programmer], [USBPROG]],
	[[rlink], [Raisonance RLink JTAG Programmer], [RLINK]],
	[[armjtagew], [Olimex ARM-JTAG-EW Programmer], [ARMJTAGEW]]])

m4_define([HIDAPI_ADAPTERS],
	[[[cmsis_dap], [CMSIS-DAP Compliant Debugger], [CMSIS_DAP]]])

#========================================
# FTD2XXX support comes in 4 forms.
#    (1) win32 - via a zip file
#    (2) linux - via a tar file
#    (3) linux/cygwin/mingw - via libftdi
#    (4) darwin - installed under /usr/local
#
# In case (1) and (2) we need to know where the package was unpacked.

AC_ARG_WITH(ftd2xx-win32-zipdir,
  AS_HELP_STRING([--with-ftd2xx-win32-zipdir],[Where (CYGWIN/MINGW) the zip file from ftdichip.com was unpacked (default=search)]),
  [
  # option present
  if test -d $with_ftd2xx_win32_zipdir
  then
    with_ftd2xx_win32_zipdir=`cd $with_ftd2xx_win32_zipdir && pwd`
    AC_MSG_NOTICE([Using: ftdichip.com library: $with_ftd2xx_win32_zipdir])
  else
    AC_MSG_ERROR([Parameter to --with-ftd2xx-win32-zipdir is not a dir: $with_ftd2xx_win32_zipdir])
  fi
  ], [true])

AC_ARG_WITH(ftd2xx-linux-tardir,
  AS_HELP_STRING([--with-ftd2xx-linux-tardir], [Where (Linux/Unix) the tar file from ftdichip.com was unpacked (default=search)]),
  [
  # Option present
  if test $is_win32 = yes ; then
     AC_MSG_ERROR([The option: --with-ftd2xx-linux-tardir is only usable on linux])
  fi
  if test -d $with_ftd2xx_linux_tardir
  then
    with_ftd2xx_linux_tardir=`cd $with_ftd2xx_linux_tardir && pwd`
    AC_MSG_NOTICE([Using: ftdichip.com library: $with_ftd2xx_linux_tardir])
  else
    AC_MSG_ERROR([Parameter to --with-ftd2xx-linux-tardir is not a dir: $with_ftd2xx_linux_tardir])
  fi
  ], [true])

AC_ARG_WITH(ftd2xx-lib,
  AS_HELP_STRING([--with-ftd2xx-lib],
    [Use static or shared ftd2xx libs (default=static)]),
  [
  case "$withval" in
  static)
    with_ftd2xx_lib=$withval
    ;;
  shared)
    with_ftd2xx_lib=$withval
    ;;
  *)
    AC_MSG_ERROR([Option: --with-ftd2xx-lib=static or --with-ftd2xx-lib=shared not, $withval])
    ;;
  esac
  ], [
  # Default is static - it is simpler :-(
  with_ftd2xx_lib=static
  ])

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

AC_ARG_ADAPTERS([USB1_ADAPTERS, USB_ADAPTERS, USB0_ADAPTERS, HIDAPI_ADAPTERS], [auto])

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

AC_ARG_ENABLE([ft2232_libftdi], [], [
if test $enableval = yes; then
  AC_MSG_ERROR([The ft2232 driver is deprecated, use --enable-ftdi to build its replacement, or force the old driver with --enable-legacy-ft2232_libftdi])
fi
])

AC_ARG_ENABLE([ft2232_ftd2xx], [], [
if test $enableval = yes; then
  AC_MSG_ERROR([The ft2232 driver is deprecated, use --enable-ftdi to build its replacement, or force the old driver with --enable-legacy-ft2232_ftd2xx])
fi
])

AC_ARG_ENABLE([legacy-ft2232_libftdi],
  AS_HELP_STRING([--enable-legacy-ft2232_libftdi], [(DEPRECATED) Enable building support for FT2232 based devices using the libftdi library]),
  [build_ft2232_libftdi=$enableval], [build_ft2232_libftdi=no])

AC_ARG_ENABLE([legacy-ft2232_ftd2xx],
  AS_HELP_STRING([--enable-legacy-ft2232_ftd2xx], [(DEPRECATED) Enable building support for FT2232 based devices using the D2XX library from ftdichip.com]),
  [build_ft2232_ftd2xx=$enableval], [build_ft2232_ftd2xx=no])

AC_ARG_ENABLE([jtag_vpi],
  AS_HELP_STRING([--enable-jtag_vpi], [Enable building support for JTAG VPI]),
  [build_jtag_vpi=$enableval], [build_jtag_vpi=no])

AC_ARG_ENABLE([usb_blaster_libftdi],
  AS_HELP_STRING([--enable-usb_blaster_libftdi], [Enable building support for the Altera USB-Blaster using the libftdi driver, opensource alternate of FTD2XX]),
  [build_usb_blaster_libftdi=$enableval], [build_usb_blaster_libftdi=no])

AC_ARG_ENABLE([usb_blaster_ftd2xx],
  AS_HELP_STRING([--enable-usb_blaster_ftd2xx], [Enable building support for the Altera USB-Blaster using the FTD2XX driver from ftdichip.com]),
  [build_usb_blaster_ftd2xx=$enableval], [build_usb_blaster_ftd2xx=no])

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
  AS_HELP_STRING([--enable-presto_libftdi], [Enable building support for ASIX Presto Programmer using the libftdi driver]),
  [build_presto_libftdi=$enableval], [build_presto_libftdi=no])

AC_ARG_ENABLE([presto_ftd2xx],
  AS_HELP_STRING([--enable-presto_ftd2xx], [Enable building support for ASIX Presto Programmer using the FTD2XX driver]),
  [build_presto_ftd2xx=$enableval], [build_presto_ftd2xx=no])

AC_ARG_ENABLE([openjtag_ftd2xx],
  AS_HELP_STRING([--enable-openjtag_ftd2xx], [Enable building support for the OpenJTAG Programmer with ftd2xx driver]),
  [build_openjtag_ftd2xx=$enableval], [build_openjtag_ftd2xx=no])

AC_ARG_ENABLE([openjtag_ftdi],
  AS_HELP_STRING([--enable-openjtag_ftdi], [Enable building support for the OpenJTAG Programmer with ftdi driver]),
  [build_openjtag_ftdi=$enableval], [build_openjtag_ftdi=no])

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
  *-mingw*)
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

if test $build_ft2232_libftdi = yes; then
  AC_DEFINE([BUILD_FT2232_LIBFTDI], [1], [1 if you want libftdi ft2232.])
else
  AC_DEFINE([BUILD_FT2232_LIBFTDI], [0], [0 if you don't want libftdi ft2232.])
fi

if test $build_ft2232_ftd2xx = yes; then
  AC_DEFINE([BUILD_FT2232_FTD2XX], [1], [1 if you want ftd2xx ft2232.])
else
  AC_DEFINE([BUILD_FT2232_FTD2XX], [0], [0 if you don't want ftd2xx ft2232.])
fi

if test $build_usb_blaster_libftdi = yes; then
  AC_DEFINE([BUILD_USB_BLASTER_LIBFTDI], [1], [1 if you want libftdi usb_blaster.])
else
  AC_DEFINE([BUILD_USB_BLASTER_LIBFTDI], [0], [0 if you don't want libftdi usb_blaster.])
fi

if test $build_jtag_vpi = yes; then
  AC_DEFINE([BUILD_JTAG_VPI], [1], [1 if you want JTAG VPI.])
else
  AC_DEFINE([BUILD_JTAG_VPI], [0], [0 if you don't want JTAG VPI.])
fi

if test $build_usb_blaster_ftd2xx = yes; then
  AC_DEFINE([BUILD_USB_BLASTER_FTD2XX], [1], [1 if you want ftd2xx usb_blaster.])
else
  AC_DEFINE([BUILD_USB_BLASTER_FTD2XX], [0], [0 if you don't want ftd2xx usb_blaster.])
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

if test $build_presto_libftdi = yes; then
  build_bitq=yes
  AC_DEFINE([BUILD_PRESTO_LIBFTDI], [1], [1 if you want the ASIX PRESTO driver using libftdi.])
else
  AC_DEFINE([BUILD_PRESTO_LIBFTDI], [0], [0 if you don't want the ASIX PRESTO driver using libftdi.])
fi

if test $build_presto_ftd2xx = yes; then
  build_bitq=yes
  AC_DEFINE([BUILD_PRESTO_FTD2XX], [1], [1 if you want the ASIX PRESTO driver using FTD2XX.])
else
  AC_DEFINE([BUILD_PRESTO_FTD2XX], [0], [0 if you don't want the ASIX PRESTO driver using FTD2XX.])
fi

if test $build_bitq = yes; then
  AC_DEFINE([BUILD_BITQ], [1], [1 if you want a bitq interface.])
else
  AC_DEFINE([BUILD_BITQ], [0], [0 if you don't want a bitq interface.])
fi

AC_DEFINE([BUILD_OPENJTAG], [0], [0 if you don't want the OpenJTAG driver.])
AC_DEFINE([BUILD_OPENJTAG_FTD2XX], [0], [0 if you don't want the OpenJTAG driver with FTD2XX driver.])
AC_DEFINE([BUILD_OPENJTAG_LIBFTDI], [0], [0 if you don't want to build OpenJTAG driver with libftdi.])

if test $build_openjtag_ftd2xx = yes; then
  AC_DEFINE([BUILD_OPENJTAG], [1], [1 if you want the OpenJTAG driver.])
  AC_DEFINE([BUILD_OPENJTAG_FTD2XX], [1], [1 if you want the OpenJTAG driver with FTD2XX driver.])
fi
if test $build_openjtag_ftdi = yes; then
  AC_DEFINE([BUILD_OPENJTAG], [1], [1 if you want the OpenJTAG drvier.])
  AC_DEFINE([BUILD_OPENJTAG_LIBFTDI], [1], [1 if you want to build OpenJTAG with FTDI driver.])
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
#-- Deal with MingW/Cygwin FTD2XX issues

if test $is_win32 = yes; then
if test "${with_ftd2xx_linux_tardir+set}" = set
then
  AC_MSG_ERROR([The option: with_ftd2xx_linux_tardir is for LINUX only.])
fi

if test $build_ft2232_ftd2xx = yes -o $build_presto_ftd2xx = yes -o $build_usb_blaster_ftd2xx = yes -o $build_openjtag_ftd2xx = yes; then
  AC_MSG_CHECKING([for ftd2xx.lib exists (win32)])

  # if we are given a zipdir...
  if test "${with_ftd2xx_win32_zipdir+set}" = set
  then
    # Set the CFLAGS for "ftd2xx.h"
    f=$with_ftd2xx_win32_zipdir/ftd2xx.h
    if test ! -f $f ; then
      AC_MSG_ERROR([File: $f cannot be found])
    fi
    CFLAGS="$CFLAGS -I$with_ftd2xx_win32_zipdir"

    # And calculate the LDFLAGS for the machine
    case "$host_cpu" in
    i?86|x86_32)
      LDFLAGS="$LDFLAGS -L$with_ftd2xx_win32_zipdir/i386"
      LIBS="$LIBS -lftd2xx"
      f=$with_ftd2xx_win32_zipdir/i386/ftd2xx.lib
      ;;
    amd64|x86_64)
      LDFLAGS="$LDFLAGS -L$with_ftd2xx_win32_zipdir/amd64"
      LIBS="$LIBS -lftd2xx"
      f=$with_ftd2xx_win32_zipdir/amd64/ftd2xx.lib
      ;;
    *)
      AC_MSG_ERROR([Unknown Win32 host cpu: $host_cpu])
      ;;
    esac
    if test ! -f $f ; then
         AC_MSG_ERROR([Library: $f not found])
    fi
  else
    LIBS="$LIBS -lftd2xx"
    AC_MSG_WARN([ASSUMPTION: The (win32) FTDICHIP.COM files: ftd2xx.h and ftd2xx.lib are in a proper place])
  fi
fi
fi # win32

if test $is_darwin = yes ; then
if test "${with_ftd2xx_win32_zipdir+set}" = set
then
   AC_MSG_ERROR([The option: --with-ftd2xx-win32-zipdir is for win32 only])
fi
if test "${with_ftd2xx_linux_tardir+set}" = set
then
   AC_MSG_ERROR([The option: with_ftd2xx_linux_tardir is for LINUX only.])
fi

if test $build_ft2232_ftd2xx = yes -o $build_presto_ftd2xx = yes -o $build_usb_blaster_ftd2xx = yes ; then
   AC_MSG_CHECKING([for libftd2xx.a (darwin)])

   if test ! -f /usr/local/include/ftd2xx.h ; then
      AC_MSG_ERROR([ftd2xx library from FTDICHIP.com seems to be missing, cannot find: /usr/local/include/ftd2xx.h])
   fi

   CFLAGS="$CFLAGS -I/usr/local/include"
   LDFLAGS="$LDFLAGS -L/usr/local/lib"
   LIBS="$LIBS -lftd2xx"
   AC_MSG_RESULT([-L/usr/local/lib -lftd2xx])
fi
fi # darwin

if test $is_win32 = no && test $is_darwin = no ; then

if test "${with_ftd2xx_win32_zipdir+set}" = set
then
   AC_MSG_ERROR([The option: --with-ftd2xx-win32-zipdir is for win32 only])
fi

if test $build_ft2232_ftd2xx = yes -o $build_presto_ftd2xx = yes -o $build_usb_blaster_ftd2xx = yes -o $build_openjtag_ftd2xx = yes; then
   # Must be linux
   if test $host_os != linux-gnu && test $host_os != linux ; then
      AC_MSG_ERROR([The (linux) ftd2xx library from FTDICHIP.com is linux only. Try --enable-ft2232-libftdi instead])
  fi
  # Are we given a TAR directory?
  if test "${with_ftd2xx_linux_tardir+set}" = set
  then
    AC_MSG_CHECKING([uninstalled ftd2xx distribution])
    # The .H file is simple..
    FTD2XX_H=$with_ftd2xx_linux_tardir/ftd2xx.h
    if test ! -f "${FTD2XX_H}"; then
    AC_MSG_ERROR([Option: --with-ftd2xx-linux-tardir appears wrong, cannot find: ${FTD2XX_H}])
    fi
    CFLAGS="$CFLAGS -I$with_ftd2xx_linux_tardir"
    if test $with_ftd2xx_lib = shared; then
       FTD2XX_LDFLAGS="-L$with_ftd2xx_linux_tardir"
       FTD2XX_LIB="-lftd2xx"
    else
      # Test #1 - v1.0.x
      case "$host_cpu" in
      i?86|x86_32)
              dir=build/i386;;
      amd64|x86_64)
              dir=build/x86_64;;
      *)
              dir=none;;
      esac
      if test -f "$with_ftd2xx_linux_tardir/$dir/libftd2xx.a"; then
          FTD2XX_LDFLAGS="-L$with_ftd2xx_linux_tardir/$dir"
          # Also needs -lrt
          FTD2XX_LIB="-lftd2xx -lrt"
      else
        # Test Number2.
        # Grr.. perhaps it exists as a version number?
        FTD2XX_LIB="$with_ftd2xx_linux_tardir/static_lib/libftd2xx.a.*.*.*"
        count=`ls ${FTD2XX_LIB} | wc -l`
        if test $count -gt 1 ; then
          AC_MSG_ERROR([Multiple libftd2xx.a files found in: $with_ftd2xx_linux_tardir/static_lib sorry cannot handle this yet])
        fi
        if test $count -ne 1 ; then
          AC_MSG_ERROR([Not found: $f, option: --with-ftd2xx-linux-tardir appears to be wrong])
        fi
        # Because the "-l" rules do not understand version numbers...
        # we will just stuff the absolute path onto the LIBS variable
        FTD2XX_LIB="`ls ${FTD2XX_LIB}` -lpthread"
        FTD2XX_LDFLAGS=""
      fi
    fi
    LDFLAGS="${LDFLAGS} ${FTD2XX_LDFLAGS}"
    LIBS="${FTD2XX_LIB} ${LIBS}"
    AC_MSG_RESULT([${FTD2XX_LDFLAGS} ${FTD2XX_LIB}])
  else
    AC_CHECK_HEADER([ftd2xx.h],[],[
        AC_MSG_ERROR([You seem to be missing the FTD2xx driver header file.])
      ])
    AC_SEARCH_LIBS([FT_GetLibraryVersion],[ftd2xx],,[
        AC_MSG_ERROR([You appear to be missing the FTD2xx driver library.])
      ],[-lrt -lusb-1.0])
  fi
fi
fi # linux

if test $build_ft2232_ftd2xx = yes -o $build_presto_ftd2xx = yes -o $build_usb_blaster_ftd2xx = yes ; then

# Before we go any further - make sure we can *BUILD* and *RUN*
# a simple app with the "ftd2xx.lib" file - in what ever form we where given
# We should be able to compile, link and run this test program now
AC_MSG_CHECKING([whether ftd2xx library works])

#
# Save the LDFLAGS for later..
LDFLAGS_SAVE=$LDFLAGS
CFLAGS_SAVE=$CFLAGS
_LDFLAGS=`eval echo $LDFLAGS`
_CFLAGS=`eval echo $CFLAGS`
LDFLAGS=$_LDFLAGS
CFLAGS=$_CFLAGS

AC_RUN_IFELSE([AC_LANG_PROGRAM([[
#include "confdefs.h"
#if IS_WIN32
#include "windows.h"
#endif
#include <stdio.h>
#include <ftd2xx.h>
  ]], [[
  DWORD x;
  FT_GetLibraryVersion( &x );
  ]])], [
    AC_MSG_RESULT([Success!])
  ], [
    AC_MSG_ERROR([Cannot build & run test program using ftd2xx.lib])
  ], [
    AC_MSG_RESULT([Skipping as we are cross-compiling])
  ])

AC_MSG_CHECKING([for ftd2xx highspeed device support])
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[
#include "confdefs.h"
#if IS_WIN32
#include "windows.h"
#endif
#include <stdio.h>
#include <ftd2xx.h>

DWORD x = FT_DEVICE_4232H;
    ]], [])], [
      AC_DEFINE([BUILD_FT2232_HIGHSPEED], [1],
        [Support FT2232H/FT4232HS with FTD2XX or libftdi.])
      build_ft2232_highspeed=yes
    ], [
      build_ft2232_highspeed=no
    ])
  AC_MSG_RESULT([$build_ft2232_highspeed])

  if test $build_ft2232_highspeed = no; then
    AC_MSG_WARN([You need a newer FTD2XX driver (version 2.04.16 or later).])
  fi

AC_MSG_CHECKING([for ftd2xx FT232H device support])
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[
#include "confdefs.h"
#if IS_WIN32
#include "windows.h"
#endif
#include <stdio.h>
#include <ftd2xx.h>

DWORD x = FT_DEVICE_232H;
    ]], [])], [
      AC_DEFINE([HAS_ENUM_FT232H], [1],
        [Support FT232H with FTD2XX or libftdi.])
      has_enum_ft232h=yes
    ], [
      has_enum_ft232h=no
    ])
  AC_MSG_RESULT([$has_enum_ft232h])

  if test $has_enum_ft232h = no; then
    AC_MSG_WARN([You need a newer FTD2XX driver (version 2.08.12 or later).])
  fi

LDFLAGS=$LDFLAGS_SAVE
CFLAGS=$CFLAGS_SAVE
fi

if test $build_ft2232_libftdi = yes -o $build_usb_blaster_libftdi = yes -o \
  $build_openjtag_ftdi = yes -o $build_presto_libftdi = yes; then

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

AC_MSG_CHECKING([for libftdi highspeed device support])
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[
#include <stdio.h>
#include <ftdi.h>
  ]], [[
enum ftdi_chip_type x = TYPE_2232H;
    ]])], [
      AC_DEFINE([BUILD_FT2232_HIGHSPEED], [1],
        [Support FT2232H/FT4232HS with FTD2XX or libftdi.])
      build_ft2232_highspeed=yes
    ], [
      build_ft2232_highspeed=no
    ])
    AC_MSG_RESULT([$build_ft2232_highspeed])

    if test $build_ft2232_highspeed = no; then
      AC_MSG_WARN([You need a newer libftdi version (0.16 or later).])
    fi

AC_MSG_CHECKING([for libftdi FT232H device support])
AC_COMPILE_IFELSE([AC_LANG_PROGRAM([[
#include <stdio.h>
#include <ftdi.h>
  ]], [[
enum ftdi_chip_type x = TYPE_232H;
    ]])], [
      AC_DEFINE([HAS_ENUM_FT232H], [1],
        [Support FT232H with FTD2XX or libftdi.])
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

if test $enable_stlink != no -o $enable_ti_icdi != no; then
	AC_DEFINE([BUILD_HLADAPTER], [1], [1 if you want the High Level JTAG driver.])
else
	AC_DEFINE([BUILD_HLADAPTER], [0], [0 if you want the High Level JTAG driver.])
fi
AM_CONDITIONAL([HLADAPTER], [test $enable_stlink != no -o $enable_ti_icdi != no])

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
AM_CONDITIONAL([FT2232_LIBFTDI], [test $build_ft2232_libftdi = yes])
AM_CONDITIONAL([FT2232_DRIVER], [test $build_ft2232_ftd2xx = yes -o $build_ft2232_libftdi = yes])
AM_CONDITIONAL([USB_BLASTER_LIBFTDI], [test $build_usb_blaster_libftdi = yes])
AM_CONDITIONAL([USB_BLASTER_FTD2XX], [test $build_usb_blaster_ftd2xx = yes])
AM_CONDITIONAL([JTAG_VPI], [test $build_jtag_vpi = yes -o $build_jtag_vpi = yes])
AM_CONDITIONAL([USB_BLASTER_DRIVER], [test $build_usb_blaster_ftd2xx = yes -o $build_usb_blaster_libftdi = yes -o $use_libusb1 = yes])
AM_CONDITIONAL([AMTJTAGACCEL], [test $build_amtjtagaccel = yes])
AM_CONDITIONAL([GW16012], [test $build_gw16012 = yes])
AM_CONDITIONAL([PRESTO_LIBFTDI], [test $build_presto_libftdi = yes])
AM_CONDITIONAL([PRESTO_DRIVER], [test $build_presto_ftd2xx = yes -o $build_presto_libftdi = yes])
AM_CONDITIONAL([OPENJTAG], [test $build_openjtag_ftd2xx = yes -o $build_openjtag_ftdi = yes])
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

# Setup for compiling build tools
AC_MSG_CHECKING([for a C compiler for build tools])
if test $cross_compiling = yes; then
  AC_CHECK_PROGS(CC_FOR_BUILD, gcc cc)
  CFLAGS_FOR_BUILD="-g -O2 $GCC_WARNINGS"
else
  CC_FOR_BUILD=$CC
  CFLAGS_FOR_BUILD=$CFLAGS
fi

AC_MSG_RESULT([$CC_FOR_BUILD])
AC_SUBST([CC_FOR_BUILD])
AC_SUBST([CFLAGS_FOR_BUILD])

AC_MSG_CHECKING([for suffix of executable build tools])
if test $cross_compiling = yes; then
  cat >conftest.c <<\_______EOF
int main ()
{
  exit (0);
}
_______EOF
  for i in .exe ""; do
    compile="$CC_FOR_BUILD conftest.c -o conftest$i"
    if AC_TRY_EVAL(compile); then
      if (./conftest) 2>&AC_FD_CC; then
        EXEEXT_FOR_BUILD=$i
        break
      fi
    fi
  done
  rm -f conftest*
  if test "${EXEEXT_FOR_BUILD+set}" != set; then
    AC_MSG_ERROR([Cannot determine suffix of executable build tools])
  fi
else
  EXEEXT_FOR_BUILD=$EXEEXT
fi
AC_MSG_RESULT([$EXEEXT_FOR_BUILD])
AC_SUBST([EXEEXT_FOR_BUILD])

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
m4_foreach([adapter], [USB1_ADAPTERS, USB_ADAPTERS, USB0_ADAPTERS, HIDAPI_ADAPTERS],
	[echo -n m4_format(["%-40s"], ADAPTER_DESC([adapter]))
	case $ADAPTER_VAR([adapter]) in
		auto)
			echo yes '(auto)'
			;;
		yes)
			echo yes
			;;
		no)
			echo no
			;;
	esac
])
echo

if test $build_ft2232_libftdi = yes -o $build_ft2232_ftd2xx = yes; then
	if test $enable_ftdi = no; then
		AC_MSG_WARN([Building the deprecated 'ft2232' adapter driver but not its replacement!])
		AC_MSG_WARN([Please consider using --enable-ftdi instead.])
	else
		AC_MSG_WARN([Building the deprecated 'ft2232' adapter driver.])
	fi
fi
