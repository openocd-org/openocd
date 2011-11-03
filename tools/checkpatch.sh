#!/bin/sh
#

git format-patch -M --stdout HEAD^ | tools/scripts/checkpatch.pl - --no-tree
