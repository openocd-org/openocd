# -*- mode: makefile -*-
#========================================
# DO NOT DELETE THE LINE BELOW
_default: default
# DO NOT DELETE THE LINE_ABOVE
#========================================

#========================================
#
# There are no user configurable options here.
#
# All user configurable options are in local.uses
#
include ./local.uses
#
#========================================

#========================================
# This is the USB driver for the FTDI2XX chips.
# It is a "closed" solution from FTDICHIP.COM
# Some claim it is faster then the open/free
# solution:  win32-libusb+libftdi.
# 
ftd2xx.download:
	mkdir -p ${VIRGINS}
	wget -O ${FTD2XX_ZIPFILE_LOCAL} \
		 ${FTD2XX_ZIPFILE_URL}

ftd2xx.unpack:
	rm -rf ${FTD2XX_WIN32_DIR}
	mkdir -p ${FTD2XX_WIN32_DIR}
	cd ${FTD2XX_WIN32_DIR} && unzip \
		${FTD2XX_ZIPFILE_LOCAL}

ftd2xx.build:
	@echo "Nothing to do for: $@"

ftd2xx.configure:
	@echo "Nothing to do for: $@"

ftd2xx.install:
	@echo "Nothing to do for: $@"

clean::
	rm -rf ${FTD2XX_WIN32_DIR}

ftd2xx.all: ftd2xx.unpack ftd2xx.configure \
	ftd2xx.build ftd2xx.install


#========================================-
# LIBFTDI - requires LIBCONFUSE..
#   So we handle it here :-(

libconfuse.download: 
	mkdir -p virgins
	wget -O ${LIBCONFUSE_TARFILE_LOCAL} \
		${LIBCONFUSE_TARFILE_URL}

libconfuse.unpack:
	tar xfz ${LIBCONFUSE_TARFILE_LOCAL}

clean::
	rm -rf ${LIBCONFUSE_SRC_DIR}

libconfuse.configure:
	rm -rf ${LIBCONFUSE_BUILD_DIR}
	mkdir -p ${LIBCONFUSE_BUILD_DIR}
	cd ${LIBCONFUSE_BUILD_DIR} && \
		${LIBCONFUSE_SRC_DIR}/configure \
		--prefix=${PREFIX}

clean::
	rm -rf ${LIBCONFUSE_BUILD_DIR}

libconfuse.build:
	cd ${LIBCONFUSE_BUILD_DIR} && ${MAKE}

libconfuse.install:
	cd ${LIBCONFUSE_BUILD_DIR} && ${MAKE} install

libconfuse.all: libconfuse.unpack libconfuse.configure \
	 libconfuse.build libconfuse.install

#========================================
# LIBFTDI - the open source (and free)
# alternative to (closed) FTD2XX drivers.

libftdi.download: 
	mkdir -p virgins
	wget -O ${LIBFTDI_TARFILE_LOCAL} \
		${LIBFTDI_TARFILE_URL}

libftdi.unpack:
	tar xfz ${LIBFTDI_TARFILE_LOCAL}

clean::
	rm -rf ${LIBFTDI_SRC_DIR}

libftdi.configure:
	rm -rf ${LIBFTDI_BUILD_DIR}
	mkdir -p ${LIBFTDI_BUILD_DIR}
	cd ${LIBFTDI_BUILD_DIR} && \
		${LIBFTDI_SRC_DIR}/configure \
		--prefix=${PREFIX}

clean::
	rm -rf ${LIBFTDI_BUILD_DIR}

libftdi.build:
	cd ${LIBFTDI_BUILD_DIR} && ${MAKE}

libftdi.install:
	cd ${LIBFTDI_BUILD_DIR} && ${MAKE} install

libftdi.all: libftdi.unpack libftdi.configure \
	libftdi.build libftdi.install

#========================================
# Openocd...

openocd.bootstrap:
	cd ${OPENOCD_SRC_DIR} && bash ./bootstrap

openocd.configure:
	rm -rf ${OPENOCD_BUILD_DIR}
	mkdir -p ${OPENOCD_BUILD_DIR}
	cd ${OPENOCD_BUILD_DIR} && ${OPENOCD_SRC_DIR}/configure \
		--prefix=${INSTALL_DIR} \
		${OPENOCD_CONFIG_OPTIONS}

openocd.build:
	cd ${OPENOCD_BUILD_DIR} && ${MAKE}

openocd.docs:
	cd ${OPENOCD_BUILD_DIR}/docs && ${MAKE}

openocd.docs.pdf:
	cd ${OPENOCD_BUILD_DIR}/docs && ${MAKE} pdf

openocd.docs.html:
	cd ${OPENOCD_BUILD_DIR}/docs && ${MAKE} html

# fixme:
#   need to add a "make one huge html file target"

openocd.install:
	cd ${OPENOCD_BUILD_DIR} && ${MAKE} install

#========================================
# The world...

# Manual step.
download.all: \
	ftd2xx.download		\
	libconfuse.download	\
	libftdi.download

ifeq (x"${FT2232_DRIVER}",x"ftd2xx")
prebuild: ftd2xx.all
endif

ifeq (x"${FT2232_DRIVER}",x"libftdi")
prebuild: libconfuse.all libftdi.all
endif

remake: \
	openocd.bootstrap \
	openocd.configure \
	openocd.build     \
	openocd.install

initial.build : download.all prebuild remake

all: 
	@echo ""
	@echo " This makefile does not support an 'all' target"
	@echo ""
	@echo " If this is your *FIRST* time building... "
	@echo " Then use this command: \"make initial.build\""
	@echo ""
	@echo " The \"default\" target is for openocd developers"
	@echo " and rebuilds openocd completely.."
	@echo ""

default: 
	test -d ${OPENOCD_SRC_DIR} || (echo "Where is: The OPENOCD source?"; exit 1)
	${MAKE} remake

whatis_%:
	@echo ""
	@echo "Makevariable: $*  => ${${*}}"
	@echo ""
