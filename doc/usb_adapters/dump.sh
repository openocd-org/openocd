#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-or-later

devs=$(lsusb -d $1:$2 | wc -l)
case "$devs" in
	0 )
		echo "Error: USB device $1:$2 not found" > /dev/stderr
		exit 1
		;;
	1 )
		echo "Dumping $(lsusb -d $1:$2)" > /dev/stderr
		;;
	* )
		echo "Error: Multiple matches for 'lsusb -d $1:$2'" > /dev/stderr
		exit 1
		;;
esac

# break SPDX tag to hide it to checkpatch
echo '# SPDX-''License-Identifier: GPL-2.0-or-later OR GFDL-1.2-no-invariants-or-later'
echo ''
echo '# Optional comment'

lsusb -v -d $1:$2 | sed 's/ *$//'
