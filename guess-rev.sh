#!/bin/bash
#

REV=unknown

which svnversion > /dev/null 2>&1 && REV=`svnversion -n "$1"`

echo -n $REV
