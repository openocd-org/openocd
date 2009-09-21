proc iswithin { ADDRESS BASE LEN } {
    return [expr ((($ADDRESS - $BASE) > 0) && (($ADDRESS - $BASE + $LEN) > 0))]
}

proc memorytype { ADDRESS } {
    for { set chip 0 } { $chip < $N_CHIP } { incr chip } {
	if { iswithin $ADDRESS $FLASH($chip,BASE) $FLASH($chip,LEN) } {
	    return "flash"
	}
    }

    for { set chip 0 } { $chip < $N_RAM } { incr chip } {
	if { iswithin $ADDRESS $RAM($chip,BASE) $RAM($chip,LEN) } {
	    return "ram"
	}
    }
}

# default to 32bit reads.
proc isreadable { ADDRESS } {
     return isreadable32 $ADDRESS
}

proc isreadable32 { ADDRESS } {
