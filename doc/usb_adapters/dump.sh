#!/bin/sh
# SPDX-License-Identifier: GPL-2.0-or-later

hid_unavailable_report() {
	a=$(echo $1 | tr '[:lower:]' '[:upper:]')
	b=$(basename $(dirname $(ls -d /sys/bus/usb/drivers/usbhid/*/*:$a.*)))

	echo ""
	echo "ATTENTION!"
	echo "Unable to read completely the USB descriptors."
	echo "Please run the following command(s) and then run this script again"
	for i in $b; do
		echo "    sudo sh -c \"echo -n $i > /sys/bus/usb/drivers/usbhid/unbind\""
	done
	echo ""
	echo "Please notice that the USB device will not function after the above"
	echo "operations; you should unplug and replug it to get it working again."
}

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

lsusb -v -d $1:$2 2>&1 | grep -Fq '** UNAVAILABLE **' && (hid_unavailable_report $1:$2 > /dev/stderr)
