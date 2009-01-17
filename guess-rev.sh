#!/bin/sh
#

REV=unknown

which svnversion > /dev/null 2>&1 && REV=`svnversion -n`

`which echo` -n $REV

