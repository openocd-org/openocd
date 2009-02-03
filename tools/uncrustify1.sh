#!/bin/sh
# Run the beautifier "Uncrustify" on a single file.
# Because the file "uncrustify.cfg" only exists in the top level of the project
# you should run this script from there so this script can find your uncrustify.cfg file.


UNCRUSTIFYTMP=/tmp/uncrustify.tmp


if [ ! -f uncrustify.cfg ]; then
    echo "unable to find uncrustify.cfg, aborting"
    exit 1
fi

UNCRUSTIFYBIN=`which uncrustify`

if [ "$UNCRUSTIFYBIN" = "" ]; then
    echo "you must specify uncrustify in your PATH, I cannot find it"
    exit 2
fi

if [ $# -lt 1 ]; then
    echo "Usage $0 <filename .c or .h>"
    exit 3
fi

uncrustify -c uncrustify.cfg <$1 >$UNCRUSTIFYTMP

# you can comment this out while tuning the uncrustify.cfg file:
mv $UNCRUSTIFYTMP $1
