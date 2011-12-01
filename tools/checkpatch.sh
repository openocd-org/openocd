#!/bin/sh
#

since=${1:-HEAD^}
git format-patch -M --stdout $since | tools/scripts/checkpatch.pl - --no-tree
