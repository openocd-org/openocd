# implements Tcl procedures/functions
proc peek {address} {
	return [openocd mdw $address]
}