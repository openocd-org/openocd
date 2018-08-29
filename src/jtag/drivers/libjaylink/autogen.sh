#!/bin/sh
##
## This file is part of the libjaylink project.
##
## Copyright (C) 2014-2015 Marc Schink <jaylink-dev@marcschink.de>
##
## This program is free software: you can redistribute it and/or modify
## it under the terms of the GNU General Public License as published by
## the Free Software Foundation, either version 2 of the License, or
## (at your option) any later version.
##
## This program is distributed in the hope that it will be useful,
## but WITHOUT ANY WARRANTY; without even the implied warranty of
## MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
## GNU General Public License for more details.
##
## You should have received a copy of the GNU General Public License
## along with this program.  If not, see <http://www.gnu.org/licenses/>.
##

OS=`uname -s`
LIBTOOLIZE=libtoolize

if [ "x$OS" = "xDarwin" ]; then
	LIBTOOLIZE=glibtoolize
fi

echo "Generating build system..."

$LIBTOOLIZE --install --copy || exit 1
aclocal -I m4 || exit 1
autoheader || exit 1
autoconf || exit 1
automake --add-missing --copy || exit 1
