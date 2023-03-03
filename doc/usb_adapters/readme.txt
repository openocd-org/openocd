# SPDX-License-Identifier: GPL-2.0-or-later OR GFDL-1.2-no-invariants-or-later

This folder contains a collection of dumps of USB descriptors, obtained through
Linux lsusb command, of several USB adapters supported by OpenOCD.
This collection should help maintaining adapter drivers even if the developer
doesn't have access to all the devices supported by the driver.

To add a new file, run:

	./doc/usb_adapters/dump.sh ${vid} ${pid} \
		> doc/usb_adapters/${driver}/${vid}_${pid}_${short_description}.txt

eventually edit the file to add some extra comment, then submit the file to
OpenOCD gerrit, as explained in HACKING.

The dumps are organized in subfolders corresponding to OpenOCD drivers:
- cmsis_dap;
- esp_usb_jtag;
- ft232r;
- ftdi;
- icdi;
- jlink;
- kitprog;
- nulink;
- stlink;
- xds110.

The script above assumes the user has granted access permissions to the USB
device file in
	/dev/bus/usb/<n>/<m>
This is usually the case when the device is listed in
	contrib/60-openocd.rules
and this udev rules file is properly installed in the host machine.
If the user has no proper access permissions, the script has to be run as
root or through 'sudo'.

Old versions of 'lsusb -v' dump cryptic errors like:
	can't get device qualifier: Resource temporarily unavailable
	can't get debug descriptor: Resource temporarily unavailable
when some optional descriptor is not present.
This is fixed in usbutils v014.
If you get such messages simply ignore them. They are printed on stderr, so
will not be included in the generated file as the redirection '>' does only
redirects stdout.
